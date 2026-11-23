// SPDX-License-Identifier: LGPL-2.1-only
/*
 * uvtransport - ICE and DTLS-SRTP media transport for one stream
 * Copyright (C) 2026 John Crispin <john@phrozen.org>
 */
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <juice/juice.h>
#include <openssl/bio.h>
#include <openssl/bn.h>
#include <openssl/evp.h>
#include <openssl/srtp.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>
#include <uvsrtp/uvsrtp.h>

#include "uvtransport/uvtransport.h"

#ifdef UVT_TEST_HOOKS
#include <stdatomic.h>
#endif

#define UVT_CANDIDATE_RING	16U
#define UVT_DTLS_MTU		1200L
#define UVT_SRTP_PROFILES	"SRTP_AES128_CM_SHA1_80:SRTP_AES128_CM_SHA1_32"
#define UVT_EXPORT_LABEL	"EXTRACTOR-dtls_srtp"
#define UVT_KEY_LEN		16U
#define UVT_SALT_LEN		14U
#define UVT_EXPORT_LEN		(2U * (UVT_KEY_LEN + UVT_SALT_LEN))
#define UVT_CERT_DAYS		365L
#define UVT_DESC_MAX		1024U
#define UVT_DRAIN_MAX		64U
#define UVT_BUF_MAX		(UVT_PACKET_MAX + UVSRTP_MAX_SRTCP_TRAILER)
/*
 * Media can take every slot of the output queue except UVT_QUEUE_RESERVE,
 * so a DTLS flight or a close_notify still finds room.
 */
#define UVT_QUEUE_LEN		16U
#define UVT_QUEUE_RESERVE	8U
/* Bound of the DTLS input BIO, so a far end cannot grow it without limit. */
#define UVT_RBIO_MAX		(8U * UVT_PACKET_MAX)
#define UVT_ICE_CHARS		"ABCDEFGHIJKLMNOPQRSTUVWXYZ" \
				"abcdefghijklmnopqrstuvwxyz0123456789+/"

enum uvt_recv_verdict {
	UVT_RECV_NONE = 0,
	UVT_RECV_RTP,
	UVT_RECV_RTCP,
	UVT_RECV_UP,
	UVT_RECV_FAIL,
	UVT_RECV_CLOSED,
};

enum uvt_remote {
	UVT_REMOTE_NONE = 0,
	UVT_REMOTE_INSTALLING,
	UVT_REMOTE_SET,
};

struct uvt_global {
	pthread_mutex_t lock;
	SSL_CTX *ctx;
	X509 *cert;
	EVP_PKEY *key;
	BIO_METHOD *bio_meth;
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	bool ready;
};

struct uvt_datagram {
	_Alignas(uint32_t) uint8_t data[UVT_BUF_MAX];
	size_t len;
	uint8_t dscp;
};

struct uvt_stream {
	pthread_mutex_t lock;
	pthread_cond_t drained;
	struct uvt_cfg cfg;
	juice_agent_t *agent;
	juice_turn_server_t turn[UVT_TURN_MAX];
	SSL *ssl;
	BIO *rbio;
	struct uvsrtp *srtp;
	enum uvt_setup setup;
	enum uvt_ice_state ice;
	enum uvt_remote remote;
	uint8_t remote_fp[UVT_FINGERPRINT_LEN];
	bool remote_fp_set;
	bool gathered;
	bool dtls_up;
	bool dtls_failed;
	bool dtls_closed;
	bool draining;
	bool destroying;
	unsigned int users;
	char cand[UVT_CANDIDATE_RING][UVT_CANDIDATE_MAX];
	unsigned int cand_head;
	unsigned int cand_count;
	struct uvt_datagram queue[UVT_QUEUE_LEN];
	unsigned int queue_head;
	unsigned int queue_count;
	unsigned long media_dropped;
};

static struct uvt_global global = { .lock = PTHREAD_MUTEX_INITIALIZER };

#ifdef UVT_TEST_HOOKS
static _Thread_local unsigned int lock_depth;
static atomic_uint hook_lock_faults;
static void (*hook_entry)(struct uvt_stream *stream, const char *call);
static bool (*hook_send)(struct uvt_stream *stream, const uint8_t *data,
			 size_t len, int ds);
static DTLS_timer_cb hook_dtls_timer;
#endif

static void stream_lock(struct uvt_stream *stream)
{
	pthread_mutex_lock(&stream->lock);
#ifdef UVT_TEST_HOOKS
	lock_depth++;
#endif
}

static void stream_unlock(struct uvt_stream *stream)
{
#ifdef UVT_TEST_HOOKS
	lock_depth--;
#endif
	pthread_mutex_unlock(&stream->lock);
}

/*
 * A call that uses the stream after it releases the stream lock holds a user
 * count, so uvt_destroy() cannot free the stream under it. No callback waits
 * for the count, so a user that waits for the libjuice registry mutex gets it.
 *
 * Context: the stream lock is held.
 * Return: false when uvt_destroy() runs.
 */
static bool user_take(struct uvt_stream *stream)
{
	if (stream->destroying)
		return false;
	stream->users++;

	return true;
}

static void user_drop(struct uvt_stream *stream)
{
	stream_lock(stream);
	if (!--stream->users)
		pthread_cond_broadcast(&stream->drained);
	stream_unlock(stream);
}

/*
 * Take a user count with no stream lock held after it.
 *
 * Return: false when uvt_destroy() runs.
 */
static bool user_enter(struct uvt_stream *stream)
{
	bool ok;

	stream_lock(stream);
	ok = user_take(stream);
	stream_unlock(stream);

	return ok;
}

static void hook_point(struct uvt_stream *stream, const char *point)
{
#ifdef UVT_TEST_HOOKS
	if (hook_entry)
		hook_entry(stream, point);
#endif
}

/*
 * Every libjuice call goes through one of the ice_*() functions. A libjuice
 * call can wait for the libjuice registry mutex. libjuice holds that mutex
 * while it runs the callbacks, and the callbacks take the stream lock. So no
 * thread enters libjuice with a stream lock held.
 */
static void ice_enter(struct uvt_stream *stream, const char *call)
{
#ifdef UVT_TEST_HOOKS
	if (lock_depth)
		atomic_fetch_add(&hook_lock_faults, 1U);
#endif
	hook_point(stream, call);
}

static juice_agent_t *ice_create(struct uvt_stream *stream,
				 const juice_config_t *config)
{
	ice_enter(stream, __func__);

	return juice_create(config);
}

static void ice_destroy(struct uvt_stream *stream)
{
	ice_enter(stream, __func__);
	juice_destroy(stream->agent);
}

static int ice_gather(struct uvt_stream *stream)
{
	ice_enter(stream, __func__);

	return juice_gather_candidates(stream->agent);
}

static int ice_local_get(struct uvt_stream *stream, char *buf, size_t len)
{
	ice_enter(stream, __func__);

	return juice_get_local_description(stream->agent, buf, len);
}

static int ice_remote_set(struct uvt_stream *stream, const char *sdp)
{
	ice_enter(stream, __func__);

	return juice_set_remote_description(stream->agent, sdp);
}

static int ice_candidate_add(struct uvt_stream *stream, const char *sdp)
{
	ice_enter(stream, __func__);

	return juice_add_remote_candidate(stream->agent, sdp);
}

static int ice_remote_done(struct uvt_stream *stream)
{
	ice_enter(stream, __func__);

	return juice_set_remote_gathering_done(stream->agent);
}

static int ice_send(struct uvt_stream *stream,
		    const struct uvt_datagram *dgram)
{
	/* libjuice takes the TOS byte, with the DSCP in its upper six bits. */
	int ds = (dgram->dscp << 2) & 0xFC;

	ice_enter(stream, __func__);
#ifdef UVT_TEST_HOOKS
	if (hook_send && hook_send(stream, dgram->data, dgram->len, ds))
		return JUICE_ERR_FAILED;
#endif

	return juice_send_diffserv(stream->agent, (const char *)dgram->data,
				   dgram->len, ds);
}

/*
 * Return: the free slot at the tail of the output queue, or NULL when no more
 * than @reserve slots are free.
 */
static struct uvt_datagram *queue_tail(struct uvt_stream *stream,
				       unsigned int reserve)
{
	unsigned int tail;

	if (stream->queue_count + reserve >= UVT_QUEUE_LEN)
		return NULL;
	tail = (stream->queue_head + stream->queue_count) % UVT_QUEUE_LEN;

	return &stream->queue[tail];
}

/*
 * One thread at a time sends the output queue, so datagrams leave in queue
 * order. A thread that finds a sender at work leaves its datagrams to that
 * sender and does not wait. A failed send is a lost datagram: DTLS sends a
 * lost record again on its timer.
 */
static void queue_drain(struct uvt_stream *stream)
{
	struct uvt_datagram *dgram;

	stream_lock(stream);
	if (stream->draining) {
		stream_unlock(stream);
		return;
	}
	stream->draining = true;
	while (stream->queue_count) {
		/*
		 * The head slot stays counted while it is sent, so no producer
		 * writes it.
		 */
		dgram = &stream->queue[stream->queue_head];
		stream_unlock(stream);
		ice_send(stream, dgram);
		stream_lock(stream);
		stream->queue_head = (stream->queue_head + 1) % UVT_QUEUE_LEN;
		stream->queue_count--;
	}
	stream->draining = false;
	pthread_cond_broadcast(&stream->drained);
	stream_unlock(stream);
}

/**
 * event_emit() - report one transport event to the owner
 * @stream: the stream
 * @event: the event
 * @arg: its argument
 */
static void event_emit(struct uvt_stream *stream, enum uvt_event event, int arg)
{
	if (stream->cfg.on_event)
		stream->cfg.on_event(stream->cfg.user, event, arg);
}

/**
 * bio_write() - queue one DTLS record for the ICE agent
 * @bio: the write BIO of the stream
 * @buf: the record
 * @len: its length
 *
 * Context: the stream lock is held. queue_drain() sends the record after the
 * caller releases the lock.
 * Return: @len. A record that finds no room counts as lost, and the DTLS timer
 * sends it again.
 */
static int bio_write(BIO *bio, const char *buf, int len)
{
	struct uvt_stream *stream = BIO_get_data(bio);
	struct uvt_datagram *dgram;

	if (len <= 0)
		return 0;
	dgram = queue_tail(stream, 0);
	if (!dgram || (size_t)len > sizeof(dgram->data))
		return len;
	memcpy(dgram->data, buf, (size_t)len);
	dgram->len = (size_t)len;
	dgram->dscp = stream->cfg.rtp_dscp;
	stream->queue_count++;

	return len;
}

/**
 * bio_ctrl() - answer the BIO controls the DTLS record layer asks
 * @bio: the write BIO of the stream
 * @cmd: the control
 * @num: its numeric argument
 * @ptr: its pointer argument
 *
 * Return: 1 for a flush, the MTU for the MTU query, 0 otherwise.
 */
static long bio_ctrl(BIO *bio, int cmd, long num, void *ptr)
{
	switch (cmd) {
	case BIO_CTRL_FLUSH:
		return 1;
	case BIO_CTRL_DGRAM_QUERY_MTU:
		return UVT_DTLS_MTU;
	default:
		return 0;
	}
}

/**
 * bio_create() - mark a new write BIO as initialized
 * @bio: the BIO
 *
 * Return: 1.
 */
static int bio_create(BIO *bio)
{
	BIO_set_init(bio, 1);

	return 1;
}

/**
 * bio_destroy() - release a write BIO, which owns nothing
 * @bio: the BIO
 *
 * Return: 1.
 */
static int bio_destroy(BIO *bio)
{
	return 1;
}

static bool cert_fingerprint_ok(const struct uvt_stream *stream, X509 *cert)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	unsigned int len = 0;

	if (!cert || !stream->remote_fp_set)
		return false;
	if (X509_digest(cert, EVP_sha256(), fingerprint, &len) != 1)
		return false;

	return len == UVT_FINGERPRINT_LEN &&
	       !memcmp(fingerprint, stream->remote_fp, UVT_FINGERPRINT_LEN);
}

/**
 * verify_peer() - check the peer certificate at handshake time
 * @ok: the verdict of the chain check
 * @ctx: the store context
 *
 * The signaled fingerprint replaces the chain check: the leaf certificate
 * must match it, so a mismatch aborts the handshake with a fatal alert and
 * fails both ends. dtls_finish() checks the fingerprint once more.
 *
 * Return: 1 to go on, 0 to abort the handshake.
 */
static int verify_peer(int ok, X509_STORE_CTX *ctx)
{
	SSL *ssl = X509_STORE_CTX_get_ex_data(ctx,
					      SSL_get_ex_data_X509_STORE_CTX_idx());
	struct uvt_stream *stream = ssl ? SSL_get_app_data(ssl) : NULL;

	if (X509_STORE_CTX_get_error_depth(ctx) > 0)
		return 1;

	return stream &&
	       cert_fingerprint_ok(stream, X509_STORE_CTX_get_current_cert(ctx));
}

/**
 * cert_make() - generate the self-signed certificate of this process
 *
 * Return: 0, or a negative errno.
 */
static int cert_make(void)
{
	EVP_PKEY *key = EVP_PKEY_Q_keygen(NULL, NULL, "EC", "P-256");
	X509 *cert = X509_new();
	BIGNUM *serial = BN_new();
	X509_NAME *name;
	unsigned int len = 0;
	int ret = -EIO;

	if (!key || !cert || !serial)
		goto out;
	if (!BN_rand(serial, 64, BN_RAND_TOP_ANY, BN_RAND_BOTTOM_ANY) ||
	    !BN_to_ASN1_INTEGER(serial, X509_get_serialNumber(cert)))
		goto out;
	if (!X509_set_version(cert, 2) ||
	    !X509_gmtime_adj(X509_getm_notBefore(cert), -86400L) ||
	    !X509_gmtime_adj(X509_getm_notAfter(cert), UVT_CERT_DAYS * 86400L) ||
	    !X509_set_pubkey(cert, key))
		goto out;
	name = X509_get_subject_name(cert);
	if (!X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
					(const unsigned char *)"uvtransport",
					-1, -1, 0) ||
	    !X509_set_issuer_name(cert, name))
		goto out;
	if (X509_sign(cert, key, EVP_sha256()) <= 0)
		goto out;
	if (!X509_digest(cert, EVP_sha256(), global.fingerprint, &len) ||
	    len != UVT_FINGERPRINT_LEN)
		goto out;

	global.cert = cert;
	global.key = key;
	cert = NULL;
	key = NULL;
	ret = 0;
out:
	BN_free(serial);
	X509_free(cert);
	EVP_PKEY_free(key);

	return ret;
}

/**
 * ctx_make() - build the DTLS context every stream handshakes with
 *
 * Return: 0, or a negative errno.
 */
static int ctx_make(void)
{
	SSL_CTX *ctx = SSL_CTX_new(DTLS_method());

	if (!ctx)
		return -ENOMEM;
	if (!SSL_CTX_set_min_proto_version(ctx, DTLS1_2_VERSION) ||
	    SSL_CTX_use_certificate(ctx, global.cert) != 1 ||
	    SSL_CTX_use_PrivateKey(ctx, global.key) != 1 ||
	    SSL_CTX_set_tlsext_use_srtp(ctx, UVT_SRTP_PROFILES) != 0) {
		SSL_CTX_free(ctx);
		return -EIO;
	}
	SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT,
			   verify_peer);
	SSL_CTX_set_options(ctx, SSL_OP_NO_QUERY_MTU);
	SSL_CTX_set_read_ahead(ctx, 1);
	global.ctx = ctx;

	return 0;
}

/**
 * bio_meth_make() - build the write BIO method that queues for libjuice
 *
 * Return: 0, or -ENOMEM.
 */
static int bio_meth_make(void)
{
	BIO_METHOD *method = BIO_meth_new(BIO_TYPE_SOURCE_SINK |
					  BIO_get_new_index(), "uvtransport");

	if (!method)
		return -ENOMEM;
	BIO_meth_set_write(method, bio_write);
	BIO_meth_set_ctrl(method, bio_ctrl);
	BIO_meth_set_create(method, bio_create);
	BIO_meth_set_destroy(method, bio_destroy);
	global.bio_meth = method;

	return 0;
}

int uvt_init(void)
{
	int ret = 0;

	pthread_mutex_lock(&global.lock);
	if (global.ready)
		goto out;
	ret = uvsrtp_init();
	if (ret)
		goto out;
	ret = cert_make();
	if (ret)
		goto out;
	ret = ctx_make();
	if (ret)
		goto out;
	ret = bio_meth_make();
	if (ret)
		goto out;
	global.ready = true;
out:
	pthread_mutex_unlock(&global.lock);

	return ret;
}

int uvt_fingerprint_get(uint8_t out[UVT_FINGERPRINT_LEN])
{
	int ret = -EAGAIN;

	pthread_mutex_lock(&global.lock);
	if (global.ready) {
		memcpy(out, global.fingerprint, UVT_FINGERPRINT_LEN);
		ret = 0;
	}
	pthread_mutex_unlock(&global.lock);

	return ret;
}

/**
 * peer_fingerprint_ok() - compare the peer certificate with the signaled one
 * @stream: the stream, after its handshake
 *
 * Return: true when the SHA-256 fingerprints match.
 */
static bool peer_fingerprint_ok(struct uvt_stream *stream)
{
	X509 *peer = SSL_get1_peer_certificate(stream->ssl);
	bool ok = cert_fingerprint_ok(stream, peer);

	X509_free(peer);

	return ok;
}

/**
 * srtp_suite_of() - map the negotiated use_srtp profile onto libuvsrtp
 * @stream: the stream, after its handshake
 * @suite: receives the suite
 *
 * Return: true when the profile is one this library supports.
 */
static bool srtp_suite_of(struct uvt_stream *stream, enum uvsrtp_suite *suite)
{
	const SRTP_PROTECTION_PROFILE *profile;

	profile = SSL_get_selected_srtp_profile(stream->ssl);
	if (!profile)
		return false;
	if (profile->id == SRTP_AES128_CM_SHA1_80) {
		*suite = UVSRTP_SUITE_AES_CM_128_HMAC_SHA1_80;
		return true;
	}
	if (profile->id == SRTP_AES128_CM_SHA1_32) {
		*suite = UVSRTP_SUITE_AES_CM_128_HMAC_SHA1_32;
		return true;
	}

	return false;
}

/**
 * srtp_keys_take() - export the SRTP keys of the handshake into libuvsrtp
 * @stream: the stream, after its handshake
 * @suite: the negotiated suite
 *
 * RFC 5764 section 4.2 orders the material as client key, server key,
 * client salt, server salt. The DTLS client sends with the client half.
 *
 * Return: 0, or a negative errno.
 */
static int srtp_keys_take(struct uvt_stream *stream, enum uvsrtp_suite suite)
{
	uint8_t material[UVT_EXPORT_LEN];
	uint8_t client[UVSRTP_KEY_LEN];
	uint8_t server[UVSRTP_KEY_LEN];
	bool is_client = stream->setup == UVT_SETUP_ACTIVE;
	int ret;

	if (SSL_export_keying_material(stream->ssl, material, sizeof(material),
				       UVT_EXPORT_LABEL, strlen(UVT_EXPORT_LABEL),
				       NULL, 0, 0) != 1)
		return -EPROTO;

	memcpy(client, material, UVT_KEY_LEN);
	memcpy(client + UVT_KEY_LEN, material + 2 * UVT_KEY_LEN, UVT_SALT_LEN);
	memcpy(server, material + UVT_KEY_LEN, UVT_KEY_LEN);
	memcpy(server + UVT_KEY_LEN, material + 2 * UVT_KEY_LEN + UVT_SALT_LEN,
	       UVT_SALT_LEN);
	ret = uvsrtp_create(&stream->srtp, suite, is_client ? client : server,
			    is_client ? server : client);
	explicit_bzero(material, sizeof(material));
	explicit_bzero(client, sizeof(client));
	explicit_bzero(server, sizeof(server));

	return ret;
}

/**
 * dtls_finish() - verify the far end and key the stream after the handshake
 * @stream: the stream
 *
 * Context: the stream lock is held.
 * Return: 1 when media may flow, -1 when the stream failed.
 */
static int dtls_finish(struct uvt_stream *stream)
{
	enum uvsrtp_suite suite;

	if (peer_fingerprint_ok(stream) && srtp_suite_of(stream, &suite) &&
	    !srtp_keys_take(stream, suite)) {
		stream->dtls_up = true;
		return 1;
	}

	stream->dtls_failed = true;
	SSL_shutdown(stream->ssl);

	return -1;
}

/**
 * dtls_drive() - run the handshake as far as the records at hand allow
 * @stream: the stream, with a handshake that neither completed nor failed
 *
 * Context: the stream lock is held.
 * Return: 1 when the handshake completed, 0 while it waits, -1 on failure.
 */
static int dtls_drive(struct uvt_stream *stream)
{
	int ret, err;

	ret = SSL_do_handshake(stream->ssl);
	if (ret == 1)
		return dtls_finish(stream);

	err = SSL_get_error(stream->ssl, ret);
	if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
		return 0;

	stream->dtls_failed = true;

	return -1;
}

/**
 * dtls_start() - make the DTLS object of a connected stream and open the flight
 * @stream: the stream
 *
 * Context: the stream lock is held.
 * Return: the dtls_drive() verdict, -1 when the object could not be made.
 */
static int dtls_start(struct uvt_stream *stream)
{
	BIO *wbio;

	stream->ssl = SSL_new(global.ctx);
	stream->rbio = BIO_new(BIO_s_mem());
	wbio = BIO_new(global.bio_meth);
	if (!stream->ssl || !stream->rbio || !wbio) {
		SSL_free(stream->ssl);
		BIO_free(stream->rbio);
		BIO_free(wbio);
		stream->ssl = NULL;
		stream->rbio = NULL;
		stream->dtls_failed = true;
		return -1;
	}
	BIO_set_mem_eof_return(stream->rbio, -1);
	BIO_set_data(wbio, stream);
	SSL_set_app_data(stream->ssl, stream);
	SSL_set_bio(stream->ssl, stream->rbio, wbio);
	SSL_set_mtu(stream->ssl, UVT_DTLS_MTU);
#ifdef UVT_TEST_HOOKS
	if (hook_dtls_timer)
		DTLS_set_timer_cb(stream->ssl, hook_dtls_timer);
#endif
	if (stream->setup == UVT_SETUP_ACTIVE)
		SSL_set_connect_state(stream->ssl);
	else
		SSL_set_accept_state(stream->ssl);

	return dtls_drive(stream);
}

/**
 * dtls_due() - whether the handshake can start now
 * @stream: the stream
 *
 * Return: true once ICE connected and the far end is set, while no DTLS object
 * exists and none failed.
 */
static bool dtls_due(const struct uvt_stream *stream)
{
	bool connected = stream->ice == UVT_ICE_CONNECTED ||
			 stream->ice == UVT_ICE_COMPLETED;

	return connected && !stream->ssl && !stream->dtls_failed &&
	       stream->setup != UVT_SETUP_ACTPASS &&
	       stream->remote == UVT_REMOTE_SET;
}

/**
 * dtls_drain() - read what the far end sent after the handshake
 * @stream: the stream
 *
 * Context: the stream lock is held.
 * Return: UVT_RECV_CLOSED when the far end closed the session, UVT_RECV_FAIL
 * when a fatal error ended it, else NONE.
 */
static enum uvt_recv_verdict dtls_drain(struct uvt_stream *stream)
{
	char tmp[UVT_DRAIN_MAX];
	int ret, err;

	for (;;) {
		ret = SSL_read(stream->ssl, tmp, sizeof(tmp));
		if (ret > 0)
			continue;
		err = SSL_get_error(stream->ssl, ret);
		if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE)
			return UVT_RECV_NONE;
		if (err == SSL_ERROR_ZERO_RETURN) {
			stream->dtls_closed = true;
			return UVT_RECV_CLOSED;
		}
		stream->dtls_failed = true;
		return UVT_RECV_FAIL;
	}
}

/**
 * dtls_input() - feed one DTLS datagram to the record layer
 * @stream: the stream
 * @data: the datagram
 * @size: its length
 *
 * A session that ended reads no more input, so its datagrams are dropped
 * before they reach the BIO.
 *
 * Context: the stream lock is held.
 * Return: the verdict the owner is told about.
 */
static enum uvt_recv_verdict dtls_input(struct uvt_stream *stream,
					const char *data, size_t size)
{
	int verdict;

	if (!stream->ssl || stream->dtls_failed || stream->dtls_closed)
		return UVT_RECV_NONE;
	if (BIO_ctrl_pending(stream->rbio) + size > UVT_RBIO_MAX ||
	    BIO_write(stream->rbio, data, (int)size) != (int)size)
		return UVT_RECV_NONE;
	if (stream->dtls_up)
		return dtls_drain(stream);

	verdict = dtls_drive(stream);
	if (verdict > 0)
		return UVT_RECV_UP;

	return verdict < 0 ? UVT_RECV_FAIL : UVT_RECV_NONE;
}

/**
 * rtcp_is() - tell RTCP from RTP on the one component (RFC 5761 section 4)
 * @pkt: the datagram
 * @len: its length
 *
 * Return: true for a packet type of 200 to 207.
 */
static bool rtcp_is(const uint8_t *pkt, size_t len)
{
	return len >= 2 && pkt[1] >= 200 && pkt[1] <= 207;
}

/**
 * media_input() - unprotect one SRTP or SRTCP datagram in place
 * @stream: the stream
 * @buf: the datagram in a buffer of UVT_BUF_MAX bytes
 * @size: its length
 * @out: receives the plaintext length
 *
 * Context: the stream lock is held.
 * Return: UVT_RECV_RTP or UVT_RECV_RTCP, NONE for a datagram that failed.
 */
static enum uvt_recv_verdict media_input(struct uvt_stream *stream,
					 uint8_t *buf, size_t size, size_t *out)
{
	bool rtcp = rtcp_is(buf, size);
	int ret;

	ret = rtcp ? uvsrtp_unprotect_rtcp(stream->srtp, buf, size, UVT_BUF_MAX,
					   out) :
		     uvsrtp_unprotect(stream->srtp, buf, size, UVT_BUF_MAX, out);
	if (ret)
		return UVT_RECV_NONE;

	return rtcp ? UVT_RECV_RTCP : UVT_RECV_RTP;
}

/**
 * on_recv() - one datagram from the connected pair (RFC 7983 demultiplexing)
 * @agent: the ICE agent
 * @data: the datagram
 * @size: its length
 * @user: the stream
 *
 * Context: the libjuice thread.
 */
static void on_recv(juice_agent_t *agent, const char *data, size_t size,
		    void *user)
{
	struct uvt_stream *stream = user;
	_Alignas(uint32_t) uint8_t buf[UVT_BUF_MAX];
	enum uvt_recv_verdict verdict = UVT_RECV_NONE;
	uint8_t first;
	bool dtls;
	size_t out = 0;

	if (!size || size > UVT_PACKET_MAX)
		return;
	first = (uint8_t)data[0];
	dtls = first >= 20 && first <= 63;

	stream_lock(stream);
	if (stream->destroying) {
		stream_unlock(stream);
		return;
	}
	if (dtls) {
		verdict = dtls_input(stream, data, size);
	} else if (first >= 128 && first <= 191 && stream->dtls_up &&
		   !stream->dtls_closed && !stream->dtls_failed) {
		memcpy(buf, data, size);
		verdict = media_input(stream, buf, size, &out);
	}
	stream_unlock(stream);

	if (dtls)
		queue_drain(stream);

	switch (verdict) {
	case UVT_RECV_RTP:
		if (stream->cfg.on_rtp)
			stream->cfg.on_rtp(stream->cfg.user, buf, out);
		break;
	case UVT_RECV_RTCP:
		if (stream->cfg.on_rtcp)
			stream->cfg.on_rtcp(stream->cfg.user, buf, out);
		break;
	case UVT_RECV_UP:
		event_emit(stream, UVT_EV_DTLS_UP, 0);
		break;
	case UVT_RECV_FAIL:
		event_emit(stream, UVT_EV_DTLS_FAIL, 0);
		break;
	case UVT_RECV_CLOSED:
		event_emit(stream, UVT_EV_DTLS_CLOSED, 0);
		break;
	case UVT_RECV_NONE:
		break;
	}
}

/**
 * on_state() - the ICE state moved
 * @agent: the ICE agent
 * @state: the new state
 * @user: the stream
 *
 * Context: the libjuice thread, or the caller of a gathering.
 */
static void on_state(juice_agent_t *agent, juice_state_t state, void *user)
{
	struct uvt_stream *stream = user;
	int dtls = 0;

	stream_lock(stream);
	if (stream->destroying) {
		stream_unlock(stream);
		return;
	}
	stream->ice = (enum uvt_ice_state)state;
	if (dtls_due(stream))
		dtls = dtls_start(stream);
	stream_unlock(stream);

	queue_drain(stream);
	event_emit(stream, UVT_EV_ICE_STATE, (int)state);
	if (dtls < 0)
		event_emit(stream, UVT_EV_DTLS_FAIL, 0);
}

/**
 * on_candidate() - one local candidate gathered
 * @agent: the ICE agent
 * @sdp: the a=candidate line
 * @user: the stream
 *
 * Context: the libjuice thread, or the caller of a gathering.
 */
static void on_candidate(juice_agent_t *agent, const char *sdp, void *user)
{
	struct uvt_stream *stream = user;
	unsigned int pending;

	stream_lock(stream);
	if (stream->destroying) {
		stream_unlock(stream);
		return;
	}
	if (stream->cand_count < UVT_CANDIDATE_RING) {
		unsigned int slot = (stream->cand_head + stream->cand_count) %
				    UVT_CANDIDATE_RING;

		snprintf(stream->cand[slot], UVT_CANDIDATE_MAX, "%s", sdp);
		stream->cand_count++;
	}
	pending = stream->cand_count;
	stream_unlock(stream);

	event_emit(stream, UVT_EV_CANDIDATE, (int)pending);
}

/**
 * on_gathering_done() - the agent gathered its last candidate
 * @agent: the ICE agent
 * @user: the stream
 *
 * Context: the libjuice thread, or the caller of a gathering.
 */
static void on_gathering_done(juice_agent_t *agent, void *user)
{
	struct uvt_stream *stream = user;
	bool destroying;

	stream_lock(stream);
	destroying = stream->destroying;
	stream_unlock(stream);

	if (!destroying)
		event_emit(stream, UVT_EV_GATHERING_DONE, 0);
}

/**
 * juice_cfg_fill() - the libjuice configuration of a stream
 * @stream: the stream, whose own copies the pointers name
 * @config: receives the configuration
 */
static void juice_cfg_fill(struct uvt_stream *stream, juice_config_t *config)
{
	unsigned int i;

	memset(config, 0, sizeof(*config));
	config->concurrency_mode = JUICE_CONCURRENCY_MODE_POLL;
	if (stream->cfg.stun_host[0]) {
		config->stun_server_host = stream->cfg.stun_host;
		config->stun_server_port = stream->cfg.stun_port;
	}
	for (i = 0; i < stream->cfg.turns && i < UVT_TURN_MAX; i++) {
		stream->turn[i].host = stream->cfg.turn[i].host;
		stream->turn[i].username = stream->cfg.turn[i].user;
		stream->turn[i].password = stream->cfg.turn[i].pass;
		stream->turn[i].port = stream->cfg.turn[i].port;
	}
	config->turn_servers = stream->turn;
	config->turn_servers_count = (int)i;
	if (stream->cfg.bind_address[0])
		config->bind_address = stream->cfg.bind_address;
	config->local_port_range_begin = stream->cfg.port_min;
	config->local_port_range_end = stream->cfg.port_max;
	config->cb_state_changed = on_state;
	config->cb_candidate = on_candidate;
	config->cb_gathering_done = on_gathering_done;
	config->cb_recv = on_recv;
	config->user_ptr = stream;
}

struct uvt_stream *uvt_create(const struct uvt_cfg *cfg)
{
	juice_config_t config;
	struct uvt_stream *stream;

	if (!cfg || uvt_init())
		return NULL;

	stream = calloc(1, sizeof(*stream));
	if (!stream)
		return NULL;

	pthread_mutex_init(&stream->lock, NULL);
	pthread_cond_init(&stream->drained, NULL);
	stream->cfg = *cfg;
	stream->setup = cfg->setup;
	stream->ice = UVT_ICE_DISCONNECTED;
	juice_cfg_fill(stream, &config);

	stream->agent = ice_create(stream, &config);
	if (!stream->agent) {
		pthread_cond_destroy(&stream->drained);
		pthread_mutex_destroy(&stream->lock);
		free(stream);
		return NULL;
	}

	if (cfg->role == UVT_ROLE_CONTROLLING && ice_gather(stream) == 0)
		stream->gathered = true;

	return stream;
}

/**
 * credential_ok() - check one ICE credential (RFC 8839 section 5.4)
 * @str: the credential
 * @min: the shortest length allowed
 * @max: the longest length allowed
 *
 * libjuice reads the credentials as lines of a description, so a character
 * outside ice-char, such as CR or LF, can add a line to it.
 *
 * Return: true when the length is inside the bounds and every character is an
 * ice-char: ALPHA, DIGIT, "+" or "/".
 */
static bool credential_ok(const char *str, size_t min, size_t max)
{
	size_t len;

	if (!str)
		return false;
	len = strspn(str, UVT_ICE_CHARS);

	return !str[len] && len >= min && len <= max;
}

int uvt_remote_set(struct uvt_stream *stream, const char *ufrag,
		   const char *pwd,
		   const uint8_t fingerprint[UVT_FINGERPRINT_LEN],
		   enum uvt_setup setup)
{
	char desc[UVT_DESC_MAX];
	bool gather;
	int dtls = 0;

	if (!stream || !fingerprint ||
	    !credential_ok(ufrag, 4, UVT_UFRAG_MAX - 1) ||
	    !credential_ok(pwd, 22, UVT_PWD_MAX - 1))
		return -EINVAL;

	/*
	 * INSTALLING refuses a second caller while libjuice installs the
	 * description with no lock held. The handshake waits for SET, so it
	 * starts with the remote fingerprint in place.
	 */
	stream_lock(stream);
	if (stream->remote != UVT_REMOTE_NONE) {
		stream_unlock(stream);
		return -EALREADY;
	}
	if (!user_take(stream)) {
		stream_unlock(stream);
		return -EPIPE;
	}
	stream->remote = UVT_REMOTE_INSTALLING;
	stream_unlock(stream);

	snprintf(desc, sizeof(desc), "a=ice-ufrag:%s\r\na=ice-pwd:%s\r\n", ufrag,
		 pwd);
	if (ice_remote_set(stream, desc) < 0) {
		stream_lock(stream);
		stream->remote = UVT_REMOTE_NONE;
		stream_unlock(stream);
		user_drop(stream);
		return -EINVAL;
	}

	stream_lock(stream);
	stream->remote = UVT_REMOTE_SET;
	memcpy(stream->remote_fp, fingerprint, UVT_FINGERPRINT_LEN);
	stream->remote_fp_set = true;
	if (stream->setup == UVT_SETUP_ACTPASS)
		stream->setup = setup == UVT_SETUP_ACTIVE ? UVT_SETUP_PASSIVE :
							    UVT_SETUP_ACTIVE;
	if (dtls_due(stream))
		dtls = dtls_start(stream);
	gather = !stream->gathered;
	stream->gathered = true;
	stream_unlock(stream);

	queue_drain(stream);
	if (gather)
		ice_gather(stream);
	if (dtls < 0)
		event_emit(stream, UVT_EV_DTLS_FAIL, 0);
	user_drop(stream);

	return 0;
}

int uvt_candidate_add(struct uvt_stream *stream, const char *sdp)
{
	int ret;

	if (!stream || !sdp)
		return -EINVAL;
	if (!user_enter(stream))
		return -EPIPE;

	/* libjuice ignores a type it does not pair, such as prflx. */
	ret = ice_candidate_add(stream, sdp);
	user_drop(stream);

	return ret < 0 && ret != JUICE_ERR_IGNORED ? -EINVAL : 0;
}

void uvt_remote_done(struct uvt_stream *stream)
{
	if (!stream || !user_enter(stream))
		return;

	ice_remote_done(stream);
	user_drop(stream);
}

/**
 * desc_value() - the value of one attribute line of a libjuice description
 * @desc: the description text
 * @key: the attribute with its colon, such as "a=ice-pwd:"
 * @out: receives the value
 * @len: capacity of @out
 *
 * Return: true when the line exists and its value fits.
 */
static bool desc_value(const char *desc, const char *key, char *out,
		       size_t len)
{
	const char *value = strstr(desc, key);
	size_t span;

	if (!value)
		return false;
	value += strlen(key);
	span = strcspn(value, "\r\n");
	if (span >= len)
		return false;
	memcpy(out, value, span);
	out[span] = 0;

	return true;
}

int uvt_local_get(struct uvt_stream *stream, char *ufrag, char *pwd)
{
	char desc[UVT_DESC_MAX];
	int ret;

	if (!stream || !ufrag || !pwd)
		return -EINVAL;
	if (!user_enter(stream))
		return -EPIPE;

	ret = ice_local_get(stream, desc, sizeof(desc));
	user_drop(stream);
	if (ret < 0)
		return -EIO;
	if (!desc_value(desc, "a=ice-ufrag:", ufrag, UVT_UFRAG_MAX) ||
	    !desc_value(desc, "a=ice-pwd:", pwd, UVT_PWD_MAX))
		return -EPROTO;

	return 0;
}

int uvt_candidate_next(struct uvt_stream *stream, char *out, size_t len)
{
	if (!stream || !out || !len)
		return 0;

	stream_lock(stream);
	if (!stream->cand_count) {
		stream_unlock(stream);
		return 0;
	}
	snprintf(out, len, "%s", stream->cand[stream->cand_head]);
	stream->cand_head = (stream->cand_head + 1) % UVT_CANDIDATE_RING;
	stream->cand_count--;
	stream_unlock(stream);

	return 1;
}

/**
 * media_send() - protect one packet and queue it for the agent
 * @stream: the stream
 * @buf: the plain packet
 * @len: its length
 * @rtcp: whether the packet is RTCP
 *
 * The packet is protected in its queue slot under the stream lock, so the
 * SRTP index order is the wire order.
 *
 * Return: 0, -EAGAIN before DTLS is up, -EPIPE after the session ended,
 * -ENOBUFS when the queue holds no room for media, or another negative errno.
 */
static int media_send(struct uvt_stream *stream, const uint8_t *buf, size_t len,
		      bool rtcp)
{
	struct uvt_datagram *dgram;
	size_t out = 0;
	int ret;

	if (!stream || !buf || !len || len > UVT_PACKET_MAX)
		return -EINVAL;

	stream_lock(stream);
	if (!stream->dtls_up) {
		stream_unlock(stream);
		return -EAGAIN;
	}
	if (stream->dtls_closed || stream->dtls_failed || stream->destroying) {
		stream_unlock(stream);
		return -EPIPE;
	}
	dgram = queue_tail(stream, UVT_QUEUE_RESERVE);
	if (!dgram) {
		stream->media_dropped++;
		stream_unlock(stream);
		return -ENOBUFS;
	}
	memcpy(dgram->data, buf, len);
	ret = rtcp ? uvsrtp_protect_rtcp(stream->srtp, dgram->data, len,
					 sizeof(dgram->data), &out) :
		     uvsrtp_protect(stream->srtp, dgram->data, len,
				    sizeof(dgram->data), &out);
	if (ret) {
		stream_unlock(stream);
		return ret;
	}
	dgram->len = out;
	dgram->dscp = rtcp ? stream->cfg.rtcp_dscp : stream->cfg.rtp_dscp;
	stream->queue_count++;
	user_take(stream);
	stream_unlock(stream);

	hook_point(stream, "media_queued");
	queue_drain(stream);
	user_drop(stream);

	return 0;
}

int uvt_send_rtp(struct uvt_stream *stream, const uint8_t *buf, size_t len)
{
	return media_send(stream, buf, len, false);
}

int uvt_send_rtcp(struct uvt_stream *stream, const uint8_t *buf, size_t len)
{
	return media_send(stream, buf, len, true);
}

void uvt_tick(struct uvt_stream *stream)
{
	struct timeval left;
	bool fail = false;
	int ret = 0;

	if (!stream)
		return;

	stream_lock(stream);
	if (!user_take(stream)) {
		stream_unlock(stream);
		return;
	}
	if (stream->ssl && !stream->dtls_up && !stream->dtls_failed &&
	    DTLSv1_get_timeout(stream->ssl, &left) && !left.tv_sec &&
	    !left.tv_usec) {
		ret = DTLSv1_handle_timeout(stream->ssl);
		if (ret < 0) {
			stream->dtls_failed = true;
			fail = true;
		}
	}
	stream_unlock(stream);

	if (ret)
		queue_drain(stream);
	if (fail)
		event_emit(stream, UVT_EV_DTLS_FAIL, 0);
	user_drop(stream);
}

enum uvt_ice_state uvt_ice_state(struct uvt_stream *stream)
{
	enum uvt_ice_state state;

	if (!stream)
		return UVT_ICE_FAILED;
	stream_lock(stream);
	state = stream->ice;
	stream_unlock(stream);

	return state;
}

bool uvt_up(struct uvt_stream *stream)
{
	bool up;

	if (!stream)
		return false;
	stream_lock(stream);
	up = stream->dtls_up && !stream->dtls_failed && !stream->dtls_closed;
	stream_unlock(stream);

	return up;
}

int uvt_close(struct uvt_stream *stream)
{
	if (!stream)
		return -ENOTCONN;

	stream_lock(stream);
	if (!stream->dtls_up || stream->dtls_closed || stream->dtls_failed ||
	    !user_take(stream)) {
		stream_unlock(stream);
		return -ENOTCONN;
	}
	stream->dtls_closed = true;
	SSL_shutdown(stream->ssl);
	stream_unlock(stream);

	/*
	 * No wait for a sender on another thread: that sender sends the
	 * close_notify. From a callback, the wait can deadlock, because that
	 * sender can wait for the registry mutex that this thread holds.
	 */
	queue_drain(stream);
	user_drop(stream);

	return 0;
}

void uvt_destroy(struct uvt_stream *stream)
{
	if (!stream)
		return;

	/*
	 * The callbacks ignore a destroying stream. A callback past that check
	 * ends before juice_destroy() returns: libjuice runs it under its
	 * registry mutex or on a thread that juice_destroy() joins.
	 */
	stream_lock(stream);
	stream->destroying = true;
	while (stream->users || stream->draining) {
		hook_point(stream, __func__);
		pthread_cond_wait(&stream->drained, &stream->lock);
	}
	stream_unlock(stream);

	ice_destroy(stream);
	stream->agent = NULL;
	SSL_free(stream->ssl);
	uvsrtp_destroy(stream->srtp);
	explicit_bzero(stream->remote_fp, sizeof(stream->remote_fp));
	pthread_cond_destroy(&stream->drained);
	pthread_mutex_destroy(&stream->lock);
	free(stream);
}
