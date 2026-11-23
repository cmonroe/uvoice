// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 John Crispin <john@phrozen.org>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * Unit tests for the client to server stream (src/proto/xmpp/stream.c).
 *
 * Real sockets, real uloop, a scripted server. The server listens on the
 * IPv4 loopback, accepts the one connection the stream opens and walks a
 * script: each step waits for a substring in what the client sent, answers
 * with a canned text and may switch the connection to TLS or close it. The
 * SASL steps follow the RFC 5802 section 5 exchange, so the client nonce
 * comes from the entropy hook of the stream configuration. The server queues
 * what the client does not take at once and writes it when the socket
 * drains, so no callback runs the loop.
 *
 * The suite shrinks the tick, the ping period and the backoff through the
 * UVOICE_XMPP_* variables of xmpp/stream.h. A case that needs a ping or the
 * end of a backoff sets that deadline to the current time, so no case waits
 * for a cadence. Each wait runs uloop one pass at a time and ends at the
 * event it waits for.
 *
 * The TLS fixtures are the committed PEM files of test/unit/tls/, found the
 * way test_sip_stream finds them: the UVOICE_TLS_FIXTURES variable first,
 * the cmake define second, the tls directory beside this file third.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <libubox/utils.h>

#include "posix.h"
#include "trace.h"
#include "proto/xmpp/disco.h"
#include "proto/xmpp/stream.h"
#include "proto_host_fake.h"
#include "test_util.h"

#define WAIT_MS		4000
/* Passes of loop_settle(), more than the sockets of a case. */
#define SETTLE_PASSES	8U
#define STEPS_MAX	24
#define LOG_MAX		32
#define FAKE_RX_MAX	16384
#define FAKE_ALL_MAX	65536
#define FAKE_TX_MAX	131072
#define TRACE_TEXT_MAX	16384

/* The cadences the suite runs with, and the default of each. */
#define TEST_TICK_MS	"1"
#define TEST_PING_MS	"300"
#define TEST_BACKOFF_MS	"100"

/* Three records of a 27000 byte stanza leave 808 bytes in TLS after a burst
 * of XMPP_RX_BURST reads of XMPP_RX_CHUNK bytes. */
#define RECORD_LEN	9000U
#define RECORDS		3U
/* A stanza that fills the socket in a few sends. */
#define FILL_BODY	4000U
#define FILL_SENDS_MAX	2000U
/* Top ups of a held socket. The wait for an acknowledgment between two of
 * them ends above TCP_DELACK_MAX, the 200 ms bound of a Linux delayed
 * acknowledgment. */
#define FILL_ROUNDS	8U
#define FILL_ACK_MS	250U
/* Names a server offers before the one this client takes. */
#define MECHS_JUNK	16U
/* Elements and attributes past the parser bounds of xmpp/xml.h. */
#define DEEP_LEVELS	(XML_DEPTH_MAX + 4U)
/* An iq id past 2048 bytes whose answer fits XMPP_TX_MAX, and one whose
 * answer does not. */
#define LONG_ID		2500U
#define HUGE_ID		XMPP_TX_MAX

#define DOMAIN		"sip.example.test"
#define USER_JID	"user@" DOMAIN
#define BOUND_JID	USER_JID "/uvoice"

#define SERVER_HEADER							\
	"<?xml version='1.0'?><stream:stream xmlns='jabber:client' "	\
	"xmlns:stream='http://etherx.jabber.org/streams' id='s1' "	\
	"from='" DOMAIN "' version='1.0'>"
#define FEATURES_STARTTLS						\
	"<stream:features><starttls xmlns='" XMPP_NS_TLS "'>"		\
	"<required/></starttls></stream:features>"
#define FEATURES_NO_TLS							\
	"<stream:features><mechanisms xmlns='" XMPP_NS_SASL "'>"	\
	"<mechanism>PLAIN</mechanism></mechanisms></stream:features>"
#define FEATURES_SCRAM							\
	"<stream:features><mechanisms xmlns='" XMPP_NS_SASL "'>"	\
	"<mechanism>PLAIN</mechanism><mechanism>SCRAM-SHA-1</mechanism>"\
	"</mechanisms></stream:features>"
#define FEATURES_PLAIN							\
	"<stream:features><mechanisms xmlns='" XMPP_NS_SASL "'>"	\
	"<mechanism>PLAIN</mechanism></mechanisms></stream:features>"
#define FEATURES_BIND							\
	"<stream:features><bind xmlns='" XMPP_NS_BIND "'/>"		\
	"<session xmlns='" XMPP_NS_SESSION "'><optional/></session>"	\
	"<sm xmlns='" XMPP_NS_SM "'/></stream:features>"
#define FEATURES_BIND_ONLY						\
	"<stream:features><bind xmlns='" XMPP_NS_BIND "'/>"		\
	"</stream:features>"
#define PROCEED		"<proceed xmlns='" XMPP_NS_TLS "'/>"
#define BIND_RESULT							\
	"<iq type='result' id='%s'><bind xmlns='" XMPP_NS_BIND "'>"	\
	"<jid>" BOUND_JID "</jid></bind></iq>"
#define ENABLED		"<enabled xmlns='" XMPP_NS_SM "' id='sm1'/>"
#define IQ_RESULT	"<iq type='result' id='%s'/>"
#define SASL_FAILURE							\
	"<failure xmlns='" XMPP_NS_SASL "'><not-authorized/></failure>"
#define SASL_SUCCESS_EMPTY	"<success xmlns='" XMPP_NS_SASL "'/>"

/* The RFC 5802 section 5 exchange. */
#define RFC_NONCE_B64	"fyko+d2lbbFgONRv9qkxdawL"
#define RFC_CLIENT_FIRST "n,,n=user,r=" RFC_NONCE_B64
#define RFC_SERVER_FIRST						\
	"r=fyko+d2lbbFgONRv9qkxdawL3rfcNHYJY1ZVvWVs7j,s=QSXCR+Q6sek8bf92,"\
	"i=4096"
#define RFC_CLIENT_FINAL						\
	"c=biws,r=fyko+d2lbbFgONRv9qkxdawL3rfcNHYJY1ZVvWVs7j,"		\
	"p=v0X8v3Bz2T0CJGbJQyF0X+HI4Ts="
#define RFC_SERVER_FINAL "v=rmF9pqV8S7suAoZWja4dJRkFsKQ="

enum step_action {
	STEP_NONE = 0,
	STEP_TLS,
	STEP_CLOSE,
};

struct step {
	const char *wait;
	const char *reply;		/* NULL sends nothing */
	bool with_id;			/* reply holds %s for the iq id */
	enum step_action action;
};

#define STEP(w, r)		{ .wait = (w), .reply = (r) }
#define STEP_ID(w, r)		{ .wait = (w), .reply = (r), .with_id = true }
#define STEP_ACT(w, r, a)	{ .wait = (w), .reply = (r), .action = (a) }

/* STARTTLS and SCRAM, up to the SASL success. */
#define SASL_STEPS							\
	STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),	\
	STEP_ACT("<starttls", PROCEED, STEP_TLS),			\
	STEP("<stream:stream", SERVER_HEADER FEATURES_SCRAM),		\
	STEP(auth_scram, challenge),					\
	STEP(response_final, success_final)

/* The SASL steps, then bind, stream management and carbons, up to ready. */
#define NEGOTIATE_STEPS							\
	SASL_STEPS,							\
	STEP("<stream:stream", SERVER_HEADER FEATURES_BIND),		\
	STEP_ID("<bind", BIND_RESULT),					\
	STEP("<enable xmlns='" XMPP_NS_SM "'/>", ENABLED),		\
	STEP_ID(XMPP_NS_CARBONS, IQ_RESULT)
#define NEGOTIATE_COUNT		9U

#define JULIET		"juliet@capulet.example/pc"
#define ROMEO		"romeo@montague.example/orchard"
#define TURN_SECRET	"tUrN-s3cret-9f2c"

/*
 * tx holds what the client did not take yet. tx_span is the TLS write a
 * retry repeats. hold stops the reads, so the socket of the client fills.
 */
struct fake {
	struct uloop_fd listen;
	struct uloop_fd conn;
	struct sip_tls_conn tls;
	bool tls_up;
	bool tls_handshake;
	bool direct_tls;
	bool hold;
	const struct step *steps;
	unsigned int nsteps;
	unsigned int step;
	unsigned int accepts;
	uint16_t port;
	char rx[FAKE_RX_MAX];
	size_t rx_len;
	uint64_t rx_total;
	char all[FAKE_ALL_MAX];
	size_t all_len;
	char tx[FAKE_TX_MAX];
	size_t tx_len;
	size_t tx_off;
	size_t tx_span;
	char tls_err[SIP_TLS_ERR_MAX];
};

/* What the owner of the stream saw. depth counts the state callbacks that
 * run at once, nested the ones that started inside another. */
enum ready_action {
	READY_NONE = 0,
	READY_SEND_FAIL,
};

struct clog {
	unsigned int states;
	uint8_t state[LOG_MAX];
	char reason[LOG_MAX][XMPP_REASON_MAX];
	unsigned int depth;
	unsigned int nested;
	enum ready_action ready_action;
	int ready_send;
	unsigned int stanzas;
	char name[LOG_MAX][32];
	char from[LOG_MAX][JID_MAX];
	unsigned int iq_replies;
	bool iq_reply_null;
	char iq_reply_type[16];
};

struct trace_cap {
	char text[TRACE_TEXT_MAX];
	size_t len;
	unsigned int lines;
};

static struct fake fake;
static struct clog clog;
static struct trace_cap tcap;
static struct xmpp_stream xs;
static struct sip_dns dns;
static struct sip_tls cli_tls;
static struct sip_tls srv_tls;

static char fix_cert[PATH_MAX];
static char fix_key[PATH_MAX];
static char fix_ca[PATH_MAX];
static char fix_other_ca[PATH_MAX];

static char auth_scram[512];
static char challenge[512];
static char response_final[512];
static char success_final[512];
static char auth_plain[512];
static char features_many[2048];
static char success_more[1024];
static char big[FAKE_TX_MAX / 2];

static uint8_t rfc_nonce[SASL_NONCE_BYTES];

/* ------------------------------------------------------------- the loop */

/*
 * One pass of uloop, with a wait that ends at the next event, the next timer
 * or end. A wait loop checks its condition after each pass, so it ends at the
 * event it waits for. Return: false once end passed.
 */
static bool loop_step(uint64_t end)
{
	uint64_t now = mono_ms();

	if (now >= end)
		return false;

	uloop_run_timeout((int)(end - now));

	return true;
}

/*
 * Handle the timers that are due and the events that are ready, without a
 * wait. A check that a report came only once, or not at all, runs after it.
 */
static void loop_settle(void)
{
	unsigned int i;

	for (i = 0; i < SETTLE_PASSES; i++)
		uloop_run_timeout(0);
}

/**
 * wait_state() - pump until the stream reaches a state
 * @want: the state
 * @ms: the window
 *
 * Return: true when it got there.
 */
static bool wait_state(enum xmpp_stream_state want, unsigned int ms)
{
	uint64_t end = mono_ms() + ms;

	while (xs.state != want && loop_step(end))
		;

	return xs.state == want;
}

/**
 * wait_step() - pump until the server script passed a step
 * @step: the index the script must reach
 * @ms: the window
 *
 * Return: true when it got there.
 */
static bool wait_step(unsigned int step, unsigned int ms)
{
	uint64_t end = mono_ms() + ms;

	while (fake.step < step && loop_step(end))
		;

	return fake.step >= step;
}

/**
 * wait_stanzas() - pump until the owner saw a number of stanzas
 * @count: the number
 * @ms: the window
 *
 * Return: true when it got there.
 */
static bool wait_stanzas(unsigned int count, unsigned int ms)
{
	uint64_t end = mono_ms() + ms;

	while (clog.stanzas < count && loop_step(end))
		;

	return clog.stanzas >= count;
}

/* Pump until the owner saw a number of iq answers. */
static bool wait_replies(unsigned int count, unsigned int ms)
{
	uint64_t end = mono_ms() + ms;

	while (clog.iq_replies < count && loop_step(end))
		;

	return clog.iq_replies >= count;
}

static unsigned int state_seen(enum xmpp_stream_state state);

/* Pump until the state callback reported a state. */
static bool wait_report(enum xmpp_stream_state want, unsigned int ms)
{
	uint64_t end = mono_ms() + ms;

	while (!state_seen(want) && loop_step(end))
		;

	return state_seen(want) != 0;
}

/* Pump until the server received a text. */
static bool wait_text(const char *text, unsigned int ms)
{
	uint64_t end = mono_ms() + ms;

	while (!strstr(fake.all, text) && loop_step(end))
		;

	return strstr(fake.all, text) != NULL;
}

/* ------------------------------------------------------------ fixtures */

/**
 * fixture_dir_take() - where the PEM files live
 * @out: receives the directory
 * @olen: its capacity
 *
 * Return: true when a directory was named.
 */
static bool fixture_dir_take(char *out, size_t olen)
{
	const char *env = getenv("UVOICE_TLS_FIXTURES");
	const char *sep;

	if (env && env[0]) {
		snprintf(out, olen, "%s", env);
		return true;
	}

#ifdef UVOICE_TLS_FIXTURE_DIR
	snprintf(out, olen, "%s", UVOICE_TLS_FIXTURE_DIR);
	return true;
#endif

	sep = strrchr(__FILE__, '/');
	if (!sep)
		return false;

	snprintf(out, olen, "%.*s/tls", (int)(sep - __FILE__), __FILE__);

	return true;
}

/**
 * fixtures_take() - name the four PEM files
 *
 * Return: true when the server pair is readable.
 */
static bool fixtures_take(void)
{
	char dir[PATH_MAX - 32];

	if (!fixture_dir_take(dir, sizeof(dir)))
		return false;

	snprintf(fix_cert, sizeof(fix_cert), "%s/server.crt", dir);
	snprintf(fix_key, sizeof(fix_key), "%s/server.key", dir);
	snprintf(fix_ca, sizeof(fix_ca), "%s/ca.crt", dir);
	snprintf(fix_other_ca, sizeof(fix_other_ca), "%s/other-ca.crt", dir);

	return !access(fix_cert, R_OK) && !access(fix_key, R_OK) &&
	       !access(fix_ca, R_OK);
}

/**
 * sasl_texts_build() - the base64 forms of the RFC 5802 exchange
 */
static void sasl_texts_build(void)
{
	char b64[256];
	static const char plain[] = "\0user\0pencil";

	b64_decode(RFC_NONCE_B64, rfc_nonce, sizeof(rfc_nonce));

	b64_encode(RFC_CLIENT_FIRST, strlen(RFC_CLIENT_FIRST), b64,
		   sizeof(b64));
	snprintf(auth_scram, sizeof(auth_scram),
		 "<auth xmlns=\"" XMPP_NS_SASL "\" mechanism=\"SCRAM-SHA-1\">%s"
		 "</auth>", b64);

	b64_encode(RFC_SERVER_FIRST, strlen(RFC_SERVER_FIRST), b64,
		   sizeof(b64));
	snprintf(challenge, sizeof(challenge),
		 "<challenge xmlns='" XMPP_NS_SASL "'>%s</challenge>", b64);

	b64_encode(RFC_CLIENT_FINAL, strlen(RFC_CLIENT_FINAL), b64,
		   sizeof(b64));
	snprintf(response_final, sizeof(response_final),
		 "<response xmlns=\"" XMPP_NS_SASL "\">%s</response>", b64);

	b64_encode(RFC_SERVER_FINAL, strlen(RFC_SERVER_FINAL), b64,
		   sizeof(b64));
	snprintf(success_final, sizeof(success_final),
		 "<success xmlns='" XMPP_NS_SASL "'>%s</success>", b64);

	b64_encode(plain, sizeof(plain) - 1, b64, sizeof(b64));
	snprintf(auth_plain, sizeof(auth_plain),
		 "<auth xmlns=\"" XMPP_NS_SASL "\" mechanism=\"PLAIN\">%s"
		 "</auth>", b64);

	snprintf(success_more, sizeof(success_more), "%s%s", success_final,
		 SERVER_HEADER);
}

/* Features with MECHS_JUNK names this client lacks before SCRAM-SHA-1. */
static void features_many_build(void)
{
	size_t len;
	unsigned int i;

	len = (size_t)snprintf(features_many, sizeof(features_many),
			       "%s<stream:features><mechanisms xmlns='%s'>",
			       SERVER_HEADER, XMPP_NS_SASL);
	for (i = 0; i < MECHS_JUNK; i++)
		len += (size_t)snprintf(features_many + len,
					sizeof(features_many) - len,
					"<mechanism>X-JUNK-%u</mechanism>", i);
	snprintf(features_many + len, sizeof(features_many) - len,
		 "<mechanism>SCRAM-SHA-1</mechanism></mechanisms>"
		 "</stream:features>");
}

/* --------------------------------------------------------- fake server */

static void fake_io(struct uloop_fd *ufd, unsigned int events);

/**
 * fake_conn_close() - drop the accepted connection
 */
static void fake_conn_close(void)
{
	if (fake.conn.fd < 0)
		return;

	sip_tls_conn_close(&fake.tls);
	uloop_fd_delete(&fake.conn);
	close(fake.conn.fd);
	fake.conn.fd = -1;
	fake.tls_up = false;
	fake.tls_handshake = false;
	fake.tx_len = 0;
	fake.tx_off = 0;
	fake.tx_span = 0;
}

/* The events the accepted connection waits for, outside the handshake. */
static void fake_mask_set(void)
{
	unsigned int mask = fake.hold ? 0 : ULOOP_READ;

	if (fake.conn.fd < 0 || fake.tls_handshake)
		return;

	if (fake.tx_off < fake.tx_len)
		mask |= ULOOP_WRITE;

	if (mask)
		uloop_fd_add(&fake.conn, mask);
	else
		uloop_fd_delete(&fake.conn);
}

/* One write of the queue. Return: the bytes moved, 0 when the socket is
 * full, -1 when the connection broke. */
static ssize_t fake_write(const char *buf, size_t len)
{
	enum sip_tls_io io;
	ssize_t moved;
	size_t done;

	if (fake.tls_up) {
		io = sip_tls_conn_write(&fake.tls, buf, len, &done);
		if (io == SIP_TLS_ERR) {
			snprintf(fake.tls_err, sizeof(fake.tls_err), "%s",
				 sip_tls_conn_error(&fake.tls));
			return -1;
		}
		return (ssize_t)done;
	}

	moved = send(fake.conn.fd, buf, len, MSG_DONTWAIT | MSG_NOSIGNAL);
	if (moved < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;

	return moved;
}

/* Write what the queue holds until the socket is full. */
static void fake_flush(void)
{
	size_t len;
	ssize_t moved;

	while (fake.conn.fd >= 0 && fake.tx_off < fake.tx_len) {
		len = fake.tx_span ? fake.tx_span : fake.tx_len - fake.tx_off;
		moved = fake_write(fake.tx + fake.tx_off, len);
		if (moved < 0) {
			fake_conn_close();
			return;
		}
		if (!moved) {
			fake.tx_span = fake.tls_up ? len : 0;
			break;
		}
		fake.tx_span = 0;
		fake.tx_off += (size_t)moved;
	}

	if (fake.tx_off == fake.tx_len) {
		fake.tx_off = 0;
		fake.tx_len = 0;
	}

	fake_mask_set();
}

/**
 * fake_send() - queue text for the client, in the clear or through TLS
 * @text: the text
 */
static void fake_send(const char *text)
{
	size_t len = strlen(text);
	size_t held = fake.tx_len - fake.tx_off;

	if (fake.conn.fd < 0 || len > sizeof(fake.tx) - held)
		return;

	memmove(fake.tx, fake.tx + fake.tx_off, held);
	fake.tx_len = held;
	fake.tx_off = 0;
	memcpy(fake.tx + fake.tx_len, text, len);
	fake.tx_len += len;
	fake_flush();
}

/**
 * fake_handshake() - one step of the server side handshake
 */
static void fake_handshake(void)
{
	enum sip_tls_io io = sip_tls_conn_io(&fake.tls);

	if (io == SIP_TLS_ERR) {
		snprintf(fake.tls_err, sizeof(fake.tls_err), "%s",
			 sip_tls_conn_error(&fake.tls));
		fake_conn_close();
		return;
	}

	if (io == SIP_TLS_OK) {
		fake.tls_up = true;
		fake.tls_handshake = false;
		fake_mask_set();
		return;
	}

	uloop_fd_add(&fake.conn, io == SIP_TLS_WANT_WRITE ? ULOOP_WRITE :
							   ULOOP_READ);
}

/**
 * fake_tls_start() - switch the accepted connection to TLS
 */
static void fake_tls_start(void)
{
	if (!sip_tls_conn_start(&srv_tls, &fake.tls, fake.conn.fd, true,
				NULL)) {
		snprintf(fake.tls_err, sizeof(fake.tls_err), "%s",
			 sip_tls_conn_error(&fake.tls));
		fake_conn_close();
		return;
	}

	fake.tls_handshake = true;
	fake_handshake();
}

/**
 * fake_id_take() - the id of the last iq the client sent
 * @out: receives the id
 * @len: its capacity
 */
static void fake_id_take(char *out, size_t len)
{
	const char *at = NULL, *next = fake.rx, *end;

	out[0] = 0;
	while ((next = strstr(next, " id=\"")) != NULL) {
		at = next + 5;
		next = at;
	}
	if (!at)
		return;

	end = strchr(at, '"');
	if (!end)
		return;

	snprintf(out, len, "%.*s", (int)(end - at), at);
}

/**
 * fake_steps_run() - answer every step the received text satisfies
 */
static void fake_steps_run(void)
{
	char id[XMPP_ID_MAX];
	char reply[1024];
	const struct step *step;

	while (fake.step < fake.nsteps && fake.conn.fd >= 0) {
		step = &fake.steps[fake.step];
		if (!strstr(fake.rx, step->wait))
			return;

		fake_id_take(id, sizeof(id));
		fake.rx_len = 0;
		fake.rx[0] = 0;
		fake.step++;

		if (step->reply) {
			if (step->with_id)
				snprintf(reply, sizeof(reply), step->reply, id);
			else
				snprintf(reply, sizeof(reply), "%s",
					 step->reply);
			fake_send(reply);
		}

		if (step->action == STEP_TLS)
			fake_tls_start();
		else if (step->action == STEP_CLOSE)
			fake_conn_close();
	}
}

/**
 * fake_rx_take() - append received bytes to the two buffers
 * @chunk: the bytes
 * @got: their count
 */
static void fake_rx_take(const char *chunk, size_t got)
{
	if (fake.rx_len + got < sizeof(fake.rx)) {
		memcpy(fake.rx + fake.rx_len, chunk, got);
		fake.rx_len += got;
		fake.rx[fake.rx_len] = 0;
	}

	if (fake.all_len + got < sizeof(fake.all)) {
		memcpy(fake.all + fake.all_len, chunk, got);
		fake.all_len += got;
		fake.all[fake.all_len] = 0;
	}

	fake.rx_total += got;
}

/* One read of the accepted connection. Return: false when the socket and TLS
 * hold nothing more to read. */
static bool fake_read_one(void)
{
	char chunk[4096];
	size_t got;
	ssize_t moved;
	enum sip_tls_io io;

	if (fake.tls_up) {
		io = sip_tls_conn_read(&fake.tls, chunk, sizeof(chunk), &got);
		if (io == SIP_TLS_ERR || (io == SIP_TLS_OK && !got)) {
			fake_conn_close();
			return false;
		}
		if (io != SIP_TLS_OK)
			return false;
		fake_rx_take(chunk, got);
		fake_steps_run();
		return true;
	}

	moved = recv(fake.conn.fd, chunk, sizeof(chunk), MSG_DONTWAIT);
	if (moved == 0 || (moved < 0 && errno != EAGAIN)) {
		fake_conn_close();
		return false;
	}
	if (moved < 0)
		return false;

	fake_rx_take(chunk, (size_t)moved);
	fake_steps_run();

	return true;
}

/**
 * fake_io() - the accepted connection has an event
 * @ufd: the connection
 * @events: ULOOP_READ, ULOOP_WRITE or both
 *
 * The reads run until the socket and TLS are empty, so no plaintext waits
 * inside TLS for an event that never comes.
 */
static void fake_io(struct uloop_fd *ufd, unsigned int events)
{
	if (fake.tls_handshake) {
		fake_handshake();
		return;
	}

	if (events & ULOOP_WRITE)
		fake_flush();

	while (fake.conn.fd >= 0 && !fake.tls_handshake && !fake.hold) {
		if (!fake_read_one())
			break;
	}

	fake_mask_set();
}

/**
 * fake_accept() - take the connection the stream opened
 * @ufd: the listener
 * @events: unused
 */
static void fake_accept(struct uloop_fd *ufd, unsigned int events)
{
	int fd = accept4(fake.listen.fd, NULL, NULL,
			 SOCK_NONBLOCK | SOCK_CLOEXEC);
	int on = 1;

	if (fd < 0)
		return;

	/* Nagle would hold a reply behind the delayed acknowledgment of the
	 * record before it, such as a TLS session ticket. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
	fake_conn_close();
	fake.conn.fd = fd;
	fake.conn.cb = fake_io;
	fake.step = 0;
	fake.rx_len = 0;
	fake.rx[0] = 0;
	fake.accepts++;
	uloop_fd_add(&fake.conn, ULOOP_READ);

	if (fake.direct_tls)
		fake_tls_start();
}

/**
 * fake_listen() - open the loopback listener
 *
 * Return: true when it listens.
 */
static bool fake_listen(void)
{
	struct sockaddr_in sa = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = htonl(INADDR_LOOPBACK),
	};
	socklen_t len = sizeof(sa);
	int fd;

	fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
	if (fd < 0)
		return false;

	if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0 ||
	    listen(fd, 4) < 0 ||
	    getsockname(fd, (struct sockaddr *)&sa, &len)) {
		close(fd);
		return false;
	}

	fake.port = ntohs(sa.sin_port);
	fake.conn.fd = -1;
	fake.listen.fd = fd;
	fake.listen.cb = fake_accept;
	uloop_fd_add(&fake.listen, ULOOP_READ);

	return true;
}

/**
 * fake_script_set() - arm the next scenario
 * @steps: the script
 * @nsteps: its length
 * @direct_tls: whether the connection starts with TLS
 */
static void fake_script_set(const struct step *steps, unsigned int nsteps,
			    bool direct_tls)
{
	fake_conn_close();
	fake.steps = steps;
	fake.nsteps = nsteps;
	fake.step = 0;
	fake.direct_tls = direct_tls;
	fake.hold = false;
	fake.rx_len = 0;
	fake.rx[0] = 0;
	fake.all_len = 0;
	fake.all[0] = 0;
	fake.tls_err[0] = 0;
}

/* ------------------------------------------------------------- the client */

/**
 * on_state() - record a transition
 * @stream: the stream
 * @state: the new state
 * @reason: the text
 */
static void on_state(struct xmpp_stream *stream, enum xmpp_stream_state state,
		     const char *reason)
{
	unsigned int i = clog.states;

	if (clog.depth)
		clog.nested++;
	clog.depth++;

	if (i < LOG_MAX) {
		clog.state[i] = (uint8_t)state;
		snprintf(clog.reason[i], sizeof(clog.reason[i]), "%s", reason);
	}
	clog.states++;

	/* A write on a socket that refuses it fails inside this callback. */
	if (state == XMPP_ST_READY && clog.ready_action == READY_SEND_FAIL) {
		shutdown(stream->ufd.fd, SHUT_WR);
		clog.ready_send = xmpp_stream_send(stream, "<presence/>", 11);
	}

	clog.depth--;
}

/**
 * on_stanza() - record a stanza, answer the test iq
 * @stream: the stream
 * @doc: the stanza
 *
 * Return: true for an iq that held the test element.
 */
static bool on_stanza(struct xmpp_stream *stream, const struct xml_doc *doc)
{
	unsigned int i = clog.stanzas;
	const char *from = xml_attr_get(doc, 0, "from");

	if (i < LOG_MAX) {
		snprintf(clog.name[i], sizeof(clog.name[i]), "%s",
			 doc->node[0].name);
		snprintf(clog.from[i], sizeof(clog.from[i]), "%s",
			 from ? from : "");
	}
	clog.stanzas++;

	if (xml_child(doc, XML_NONE, "test", "test:ns") == XML_NONE)
		return false;

	xmpp_stream_iq_result(stream, doc);

	return true;
}

/**
 * on_iq_reply() - record the answer to the iq the test sent
 * @stream: the stream
 * @reply: the answer, NULL for none
 * @priv: unused
 */
static void on_iq_reply(struct xmpp_stream *stream, const struct xml_doc *reply,
			void *priv)
{
	const char *type = reply ? xml_attr_get(reply, 0, "type") : NULL;

	clog.iq_replies++;
	clog.iq_reply_null = !reply;
	snprintf(clog.iq_reply_type, sizeof(clog.iq_reply_type), "%s",
		 type ? type : "");
}

/**
 * fixed_random() - the RFC nonce for a nonce request, zeros otherwise
 * @buf: receives the bytes
 * @len: how many
 */
static void fixed_random(void *buf, size_t len)
{
	if (len == SASL_NONCE_BYTES) {
		memcpy(buf, rfc_nonce, len);
		return;
	}

	memset(buf, 0x5a, len);
}

/**
 * dns_done() - hand the resolver answer to the stream
 * @resolver: the resolver
 * @query: the query handle
 * @owner: the owner handle
 * @res: the answer
 * @priv: unused
 */
static void dns_done(struct sip_dns *resolver, uint32_t query, uint32_t owner,
		     const struct sip_dns_result *res, void *priv)
{
	xmpp_stream_dns_done(&xs, query, res);
}

static const struct xmpp_stream_cb client_cb = {
	.state = on_state,
	.stanza = on_stanza,
};

/**
 * client_cfg() - the account of every scenario
 * @cfg: receives it
 * @direct_tls: XEP-0368 instead of STARTTLS
 */
static void client_cfg(struct xmpp_stream_cfg *cfg, bool direct_tls)
{
	memset(cfg, 0, sizeof(*cfg));
	snprintf(cfg->jid, sizeof(cfg->jid), "%s", USER_JID);
	snprintf(cfg->password, sizeof(cfg->password), "pencil");
	snprintf(cfg->host, sizeof(cfg->host), "127.0.0.1");
	cfg->port = fake.port;
	cfg->direct_tls = direct_tls;
	cfg->tls_verify = true;
	cfg->allow_plain = true;
	cfg->random = fixed_random;
}

/* A fresh stream with an account the case shaped. */
static int client_start_cfg(const struct xmpp_stream_cfg *cfg,
			    struct sip_tls *tls)
{
	struct xmpp_stream_env env = {
		.host = proto_host_fake(),
		.dns = &dns,
		.dns_owner = 1,
		.tls = tls,
	};

	memset(&clog, 0, sizeof(clog));
	xmpp_stream_init(&xs, &client_cb, NULL, &env);

	return xmpp_stream_start(&xs, cfg);
}

/**
 * client_start() - a fresh stream against the armed script
 * @direct_tls: XEP-0368 instead of STARTTLS
 * @tls: the client contexts
 *
 * Return: the xmpp_stream_start() verdict.
 */
static int client_start(bool direct_tls, struct sip_tls *tls)
{
	struct xmpp_stream_cfg cfg;

	client_cfg(&cfg, direct_tls);

	return client_start_cfg(&cfg, tls);
}

/**
 * client_end() - stop the stream and release the parser
 */
static void client_end(void)
{
	xmpp_stream_done(&xs);
	loop_settle();
}

/**
 * state_seen() - whether a transition to a state was recorded
 * @state: the state
 *
 * Return: the index of the record plus one, 0 when none.
 */
static unsigned int state_seen(enum xmpp_stream_state state)
{
	unsigned int i, n = clog.states < LOG_MAX ? clog.states : LOG_MAX;

	for (i = 0; i < n; i++) {
		if (clog.state[i] == state)
			return i + 1;
	}

	return 0;
}

/* How many transitions to a state were recorded. */
static unsigned int state_count(enum xmpp_stream_state state)
{
	unsigned int i, count = 0;
	unsigned int n = clog.states < LOG_MAX ? clog.states : LOG_MAX;

	for (i = 0; i < n; i++)
		count += clog.state[i] == state;

	return count;
}

/**
 * last_reason() - the reason of the last recorded transition
 *
 * Return: the text.
 */
static const char *last_reason(void)
{
	unsigned int n = clog.states < LOG_MAX ? clog.states : LOG_MAX;

	return n ? clog.reason[n - 1] : "";
}

/* The last recorded state, XMPP_ST_MAX when none was recorded. */
static enum xmpp_stream_state last_state(void)
{
	unsigned int n = clog.states < LOG_MAX ? clog.states : LOG_MAX;

	return n ? (enum xmpp_stream_state)clog.state[n - 1] : XMPP_ST_MAX;
}

/* Arm a script that starts with the negotiation and bring a stream to it. */
static bool session_ready(const struct step *steps, unsigned int nsteps)
{
	fake_script_set(steps, nsteps, false);
	if (client_start(false, &cli_tls))
		return false;

	return wait_report(XMPP_ST_READY, WAIT_MS) &&
	       fake.step == NEGOTIATE_COUNT;
}

/* A connection that fails at the next write. */
static void client_write_break(void)
{
	shutdown(xs.ufd.fd, SHUT_WR);
}

/* Copy each trace line of the XMPP domain. */
static void trace_take(enum trace_domain dom, const char *msg)
{
	size_t len = strlen(msg);

	tcap.lines++;
	if (tcap.len + len + 2 > sizeof(tcap.text))
		return;

	memcpy(tcap.text + tcap.len, msg, len);
	tcap.len += len;
	tcap.text[tcap.len++] = '\n';
	tcap.text[tcap.len] = 0;
}

/* ---------------------------------------------------------------- cases */

/*
 * STARTTLS, SCRAM, bind, sm, carbons, then traffic.
 */
static void test_full_session(void)
{
	struct step steps[STEPS_MAX] = {
		NEGOTIATE_STEPS,
		STEP_ID("<query xmlns=\"test:ns\"",
			"<iq type='result' id='%s'><query xmlns='test:ns'/>"
			"</iq>"),
		STEP("<a xmlns=\"" XMPP_NS_SM "\" h=\"3\"/>", NULL),
		STEP("<iq type=\"result\" to=\"" DOMAIN "\" id=\"p1\"/>", NULL),
		STEP("<feature var=\"urn:xmpp:jingle:1\"/>", NULL),
		STEP("<service-unavailable xmlns=\"" XMPP_NS_STANZAS "\"/>",
		     NULL),
		STEP("<iq type=\"result\" to=\"" DOMAIN "\" id=\"h1\"/>", NULL),
		STEP("<r xmlns='" XMPP_NS_SM "'/>",
		     "<a xmlns='" XMPP_NS_SM "' h='7'/>"),
	};
	char iq[256];
	char id[XMPP_ID_MAX];
	uint64_t end;
	int len;

	fake_script_set(steps, 16, false);

	CASE("the stream negotiates its way to ready over starttls");
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_state(XMPP_ST_READY, WAIT_MS));
	CHECK(fake.step == 9);
	CHECK(xs.tls_up);
	CHECK(xs.authenticated);
	CHECK(xs.sm.enabled);
	CHECK(xs.carbons);
	CHECK(!strcmp(xs.bound.resource, "uvoice"));
	CHECK(state_seen(XMPP_ST_RESOLVE));
	CHECK(state_seen(XMPP_ST_CONNECT));
	CHECK(state_seen(XMPP_ST_STARTTLS));
	CHECK(state_seen(XMPP_ST_TLS));
	CHECK(state_seen(XMPP_ST_SASL));
	CHECK(state_seen(XMPP_ST_BIND));
	CHECK(!state_seen(XMPP_ST_SESSION));
	CHECK(state_seen(XMPP_ST_SM));
	CHECK(state_seen(XMPP_ST_CARBONS));
	CHECK(strstr(fake.all, "<stream:stream from='" USER_JID "' to='"
		     DOMAIN "'") != NULL);
	CHECK(strstr(fake.all, "<resource>uvoice</resource>") != NULL);
	CHECK(xmpp_stream_ready(&xs));

	CASE("the stream socket holds TCP_NODELAY");
	{
		socklen_t vlen = sizeof(int);
		int val = 0;

		CHECK(getsockopt(xs.ufd.fd, IPPROTO_TCP, TCP_NODELAY, &val,
				 &vlen) == 0);
		CHECK(val != 0);
	}

	CASE("an iq of the owner gets its answer through the callback");
	xmpp_stream_id_next(&xs, id, sizeof(id));
	len = snprintf(iq, sizeof(iq),
		       "<iq type=\"get\" id=\"%s\"><query xmlns=\"test:ns\"/>"
		       "</iq>", id);
	CHECK(xmpp_stream_iq_send(&xs, iq, (size_t)len, id, NULL, on_iq_reply,
				  NULL) == 0);
	CHECK(wait_step(10, WAIT_MS));
	CHECK(wait_replies(1, WAIT_MS));
	CHECK(clog.iq_replies == 1);
	CHECK(!clog.iq_reply_null);
	CHECK(!strcmp(clog.iq_reply_type, "result"));

	CASE("a message reaches the owner, the stream answers the sm request");
	fake_send("<message from='juliet@capulet.example/pc' to='" BOUND_JID
		  "' type='chat'><body>hi</body></message>");
	CHECK(wait_stanzas(1, WAIT_MS));
	CHECK(!strcmp(clog.name[0], "message"));
	CHECK(!strcmp(clog.from[0], "juliet@capulet.example/pc"));
	fake_send("<r xmlns='" XMPP_NS_SM "'/>");
	CHECK(wait_step(11, WAIT_MS));

	CASE("an iq ping is answered inside the stream");
	fake_send("<iq type='get' from='" DOMAIN "' to='" BOUND_JID
		  "' id='p1'><ping xmlns='" XMPP_NS_PING "'/></iq>");
	CHECK(wait_step(12, WAIT_MS));
	CHECK(clog.stanzas == 1);
	CHECK(xs.stat.pings == 1);

	CASE("a disco#info query is answered with the feature set");
	fake_send("<iq type='get' from='" DOMAIN "' to='" BOUND_JID
		  "' id='d1'><query xmlns='" DISCO_NS_INFO "'/></iq>");
	CHECK(wait_step(13, WAIT_MS));
	CHECK(clog.stanzas == 1);

	CASE("an iq the owner does not handle gets service-unavailable");
	fake_send("<iq type='set' from='" DOMAIN "' to='" BOUND_JID
		  "' id='u1'><foo xmlns='x:y'/></iq>");
	CHECK(wait_step(14, WAIT_MS));
	CHECK(clog.stanzas == 2);
	CHECK(!strcmp(clog.name[1], "iq"));

	CASE("an iq the owner handles is answered by the owner");
	fake_send("<iq type='set' from='" DOMAIN "' to='" BOUND_JID
		  "' id='h1'><test xmlns='test:ns'/></iq>");
	CHECK(wait_step(15, WAIT_MS));
	CHECK(clog.stanzas == 3);

	CASE("silence makes the stream ask for an ack, the answer is kept");
	xs.ping_due_ms = mono_ms();
	CHECK(wait_step(16, WAIT_MS));
	end = mono_ms() + WAIT_MS;
	while (xs.sm.acked != 7 && loop_step(end))
		;
	CHECK(xs.sm.acked == 7);
	CHECK(xs.stat.pings == 2);
	CHECK(!xs.ping_waiting);

	CASE("a send outside ready is refused, a peer close starts a backoff");
	fake_send("<message from='x@y' type='chat'><body>bye</body></message>");
	CHECK(wait_stanzas(4, WAIT_MS));
	fake_conn_close();
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "peer closed the connection"));
	CHECK(xmpp_stream_send(&xs, "<x/>", 4) == -ENOTCONN);
	CHECK(xs.fails == 1);
	CHECK(xs.ufd.fd < 0);

	CASE("the backoff ends in a new connection");
	xs.backoff_until = mono_ms();
	CHECK(wait_state(XMPP_ST_FEATURES, WAIT_MS));
	CHECK(fake.accepts == 2);
	CHECK(wait_step(1, WAIT_MS));
	CHECK(wait_state(XMPP_ST_READY, WAIT_MS));

	CASE("a stop closes the document and reconnects no more");
	xmpp_stream_stop(&xs);
	CHECK(xs.state == XMPP_ST_IDLE);
	CHECK(!strcmp(last_reason(), "stopped"));
	CHECK(wait_text("</stream:stream>", WAIT_MS));
	loop_settle();
	CHECK(xs.state == XMPP_ST_IDLE);
	CHECK(!xs.tick.pending);
	CHECK(fake.accepts == 2);
	client_end();
}

/*
 * XEP-0368 with PLAIN and a mandatory session.
 */
static void test_direct_tls_plain(void)
{
	struct step steps[STEPS_MAX] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_PLAIN),
		STEP(auth_plain, SASL_SUCCESS_EMPTY),
		STEP("<stream:stream", SERVER_HEADER
		     "<stream:features><bind xmlns='" XMPP_NS_BIND "'/>"
		     "<session xmlns='" XMPP_NS_SESSION "'/>"
		     "</stream:features>"),
		STEP_ID("<bind", BIND_RESULT),
		STEP_ID("<session xmlns=\"" XMPP_NS_SESSION "\"/>", IQ_RESULT),
		STEP_ID(XMPP_NS_CARBONS,
			"<iq type='error' id='%s'><error type='cancel'>"
			"<feature-not-implemented xmlns='" XMPP_NS_STANZAS "'/>"
			"</error></iq>"),
		STEP_ID("<iq type=\"get\" to=\"" DOMAIN "\" id=\"", IQ_RESULT),
	};
	uint64_t end;

	fake_script_set(steps, 7, true);

	CASE("direct tls, plain, a session request and no carbons still end ready");
	CHECK(client_start(true, &cli_tls) == 0);
	CHECK(wait_state(XMPP_ST_READY, WAIT_MS));
	CHECK(fake.step == 6);
	CHECK(!state_seen(XMPP_ST_STARTTLS));
	CHECK(state_seen(XMPP_ST_SESSION));
	CHECK(!state_seen(XMPP_ST_SM));
	CHECK(!xs.sm.enabled);
	CHECK(!xs.carbons);

	CASE("without stream management the liveness check is an iq ping");
	xs.ping_due_ms = mono_ms();
	CHECK(wait_step(7, WAIT_MS));
	end = mono_ms() + WAIT_MS;
	while (xs.ping_waiting && loop_step(end))
		;
	CHECK(xs.stat.pings == 1);
	CHECK(!xs.ping_waiting);
	CHECK(xs.state == XMPP_ST_READY);
	client_end();
}

/*
 * The server shapes that end an attempt.
 */
static void test_refusals(void)
{
	struct step no_tls[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_NO_TLS),
	};
	struct step bad_password[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP_ACT("<starttls", PROCEED, STEP_TLS),
		STEP("<stream:stream", SERVER_HEADER FEATURES_SCRAM),
		STEP(auth_scram, SASL_FAILURE),
	};
	struct step stream_error[] = {
		STEP("<stream:stream", SERVER_HEADER
		     "<stream:error><conflict xmlns='" XMPP_NS_STREAMS "'/>"
		     "</stream:error></stream:stream>"),
	};
	struct step plain_only[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP_ACT("<starttls", PROCEED, STEP_TLS),
		STEP("<stream:stream", SERVER_HEADER FEATURES_PLAIN),
	};
	struct step garbage[] = {
		STEP("<stream:stream", SERVER_HEADER "<!-- no -->"),
	};
	struct sip_tls_cfg wrong = { .ca_path = fix_other_ca, .verify = true };
	struct sip_tls other_tls;
	struct xmpp_stream_cfg cfg;

	CASE("a server without starttls is refused");
	fake_script_set(no_tls, 1, false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "server offers no starttls"));
	client_end();

	CASE("a refused password jumps to the long backoff");
	fake_script_set(bad_password, 4, false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "sasl: not-authorized"));
	CHECK(xs.fails == XMPP_AUTH_FAILS + 1);
	CHECK(xs.backoff_until - mono_ms() > 8 * xs.backoff_ms);
	CHECK(phf.n[PHF_LOG] >= 2);
	client_end();

	CASE("a stream error names its condition");
	fake_script_set(stream_error, 1, false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "stream error: conflict"));
	client_end();

	CASE("plain is refused when the configuration forbids it");
	fake_script_set(plain_only, 3, false);
	CHECK(client_start(false, &cli_tls) == 0);
	xs.cfg.allow_plain = false;
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "sasl: no usable mechanism"));
	client_end();

	CASE("a comment in the stream fails it");
	fake_script_set(garbage, 1, false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strncmp(xs.reason, "xml: ", 5));
	client_end();

	CASE("a certificate of another authority fails the handshake");
	sip_tls_init(&other_tls);
	CHECK(sip_tls_ctx_load(&other_tls, &wrong));
	fake_script_set(bad_password, 2, false);
	CHECK(client_start(false, &other_tls) == 0);
	CHECK(wait_state(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strncmp(xs.reason, "tls: ", 5));
	client_end();
	sip_tls_done(&other_tls);

	CASE("the start refuses a bad jid, an empty password and a second start");
	client_cfg(&cfg, false);
	snprintf(cfg.jid, sizeof(cfg.jid), "nolocal.example");
	CHECK(xmpp_stream_start(&xs, &cfg) == -EINVAL);
	client_cfg(&cfg, false);
	cfg.password[0] = 0;
	CHECK(xmpp_stream_start(&xs, &cfg) == -EINVAL);
	fake_script_set(no_tls, 1, false);
	client_cfg(&cfg, false);
	CHECK(xmpp_stream_start(&xs, &cfg) == 0);
	CHECK(xmpp_stream_start(&xs, &cfg) == -EBUSY);
	client_end();
}

/* Send an iq of the owner that awaits its answer through on_iq_reply(). */
static int iq_out(char *id, size_t id_len, const char *peer)
{
	char iq[256];
	int len;

	xmpp_stream_id_next(&xs, id, id_len);
	len = snprintf(iq, sizeof(iq), "<iq type=\"get\" id=\"%s\">"
		       "<query xmlns=\"test:ns\"/></iq>", id);

	return xmpp_stream_iq_send(&xs, iq, (size_t)len, id, peer, on_iq_reply,
				   NULL);
}

/* Answer an iq from a sender, NULL for none, and pump until the stream
 * matched the answer or counted it as unmatched. */
static void result_send(const char *id, const char *from)
{
	uint64_t seen = xs.stat.iq_unmatched + clog.iq_replies;
	uint64_t end = mono_ms() + WAIT_MS;
	char reply[512];

	if (from)
		snprintf(reply, sizeof(reply),
			 "<iq type='result' id='%s' from='%s'/>", id, from);
	else
		snprintf(reply, sizeof(reply), "<iq type='result' id='%s'/>",
			 id);

	fake_send(reply);
	while (xs.stat.iq_unmatched + clog.iq_replies == seen &&
	       loop_step(end))
		;
}

/* The cadences: the production defaults, and the variables of the suite. */
static void test_cadences(void)
{
	static const char *const names[] = {
		"UVOICE_XMPP_TICK_MS",
		"UVOICE_XMPP_PING_MS",
		"UVOICE_XMPP_BACKOFF_MS",
	};
	struct xmpp_stream_env env = { .host = proto_host_fake() };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(names); i++)
		unsetenv(names[i]);

	CASE("without the variables the stream takes the production cadences");
	xmpp_stream_init(&xs, &client_cb, NULL, &env);
	CHECK_UINT(xs.tick_ms, 1000);
	CHECK_UINT(xs.ping_ms, 60000);
	CHECK_UINT(xs.backoff_ms, 2000);

	CASE("a value of 0, with trailing text or over one hour keeps the "
	     "default");
	setenv(names[0], "0", 1);
	setenv(names[1], "12x", 1);
	setenv(names[2], "3600001", 1);
	xmpp_stream_init(&xs, &client_cb, NULL, &env);
	CHECK_UINT(xs.tick_ms, 1000);
	CHECK_UINT(xs.ping_ms, 60000);
	CHECK_UINT(xs.backoff_ms, 2000);

	CASE("the variables shrink the cadences for the rest of the suite");
	setenv(names[0], TEST_TICK_MS, 1);
	setenv(names[1], TEST_PING_MS, 1);
	setenv(names[2], TEST_BACKOFF_MS, 1);
	xmpp_stream_init(&xs, &client_cb, NULL, &env);
	CHECK_UINT(xs.tick_ms, strtoul(TEST_TICK_MS, NULL, 10));
	CHECK_UINT(xs.ping_ms, strtoul(TEST_PING_MS, NULL, 10));
	CHECK_UINT(xs.backoff_ms, strtoul(TEST_BACKOFF_MS, NULL, 10));
}

/*
 * The server closes while a negotiation step waits for its iq. The owner
 * hears one backoff with the first reason and no ready, and the failure
 * counts once.
 */
static void close_run(const struct step *steps, unsigned int nsteps)
{
	fake_script_set(steps, nsteps, false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	loop_settle();
	CHECK_UINT(state_count(XMPP_ST_BACKOFF), 1);
	CHECK(!state_seen(XMPP_ST_READY));
	CHECK_UINT(clog.nested, 0);
	CHECK_UINT(xs.stat.failures, 1);
	CHECK_UINT(xs.fails, 1);
	CHECK(!strcmp(xs.reason, "peer closed the connection"));
	CHECK(!strcmp(last_reason(), "peer closed the connection"));
	client_end();
}

/* A connection that breaks during the negotiation, or inside a callback. */
static void test_broken_connection(void)
{
	struct step at_bind[] = {
		SASL_STEPS,
		STEP("<stream:stream", SERVER_HEADER FEATURES_BIND),
		STEP_ACT("<bind", NULL, STEP_CLOSE),
	};
	struct step at_session[] = {
		SASL_STEPS,
		STEP("<stream:stream", SERVER_HEADER
		     "<stream:features><bind xmlns='" XMPP_NS_BIND "'/>"
		     "<session xmlns='" XMPP_NS_SESSION "'/>"
		     "</stream:features>"),
		STEP_ID("<bind", BIND_RESULT),
		STEP_ACT("<session", NULL, STEP_CLOSE),
	};
	struct step at_carbons[] = {
		SASL_STEPS,
		STEP("<stream:stream", SERVER_HEADER FEATURES_BIND),
		STEP_ID("<bind", BIND_RESULT),
		STEP("<enable xmlns='" XMPP_NS_SM "'/>", ENABLED),
		STEP_ACT(XMPP_NS_CARBONS, NULL, STEP_CLOSE),
	};
	struct step at_enable[] = {
		SASL_STEPS,
		STEP("<stream:stream", SERVER_HEADER FEATURES_BIND),
		STEP_ID("<bind", BIND_RESULT),
		STEP("<enable xmlns='" XMPP_NS_SM "'/>", NULL),
	};
	struct step steps[] = { NEGOTIATE_STEPS };
	char id[XMPP_ID_MAX];

	CASE("a close while the bind waits reports one backoff");
	close_run(at_bind, ARRAY_SIZE(at_bind));

	CASE("a close while the session request waits reports one backoff");
	close_run(at_session, ARRAY_SIZE(at_session));

	CASE("a close while the carbons request waits reports no ready");
	close_run(at_carbons, ARRAY_SIZE(at_carbons));

	CASE("a send that fails inside the ready callback reports later");
	fake_script_set(steps, NEGOTIATE_COUNT, false);
	CHECK(client_start(false, &cli_tls) == 0);
	clog.ready_action = READY_SEND_FAIL;
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	loop_settle();
	CHECK(clog.ready_send == -ENOTCONN);
	CHECK_UINT(clog.nested, 0);
	CHECK_UINT(state_count(XMPP_ST_BACKOFF), 1);
	CHECK(state_seen(XMPP_ST_BACKOFF) > state_seen(XMPP_ST_READY));
	CHECK(!strncmp(xs.reason, "write: ", 7));
	CHECK_UINT(xs.stat.failures, 1);
	client_end();

	CASE("a write that breaks inside a negotiation send keeps its reason");
	fake_script_set(at_enable, ARRAY_SIZE(at_enable), false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_step(ARRAY_SIZE(at_enable), WAIT_MS));
	client_write_break();
	fake_send(ENABLED);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	loop_settle();
	CHECK(!strncmp(xs.reason, "write: ", 7));
	CHECK(!strncmp(last_reason(), "write: ", 7));
	CHECK_UINT(state_count(XMPP_ST_BACKOFF), 1);
	CHECK_UINT(xs.stat.failures, 1);
	CHECK_UINT(xs.fails, 1);
	client_end();

	CASE("a stop before the failure report answers the iq and reports "
	     "stopped");
	CHECK(session_ready(steps, NEGOTIATE_COUNT));
	CHECK(iq_out(id, sizeof(id), NULL) == 0);
	client_write_break();
	CHECK(xmpp_stream_send(&xs, "<presence/>", 11) == -ENOTCONN);
	CHECK(xs.fail_pending);
	CHECK(xmpp_stream_send(&xs, "<presence/>", 11) == -ENOTCONN);
	CHECK_UINT(clog.iq_replies, 0);
	xmpp_stream_stop(&xs);
	CHECK_UINT(clog.iq_replies, 1);
	CHECK(clog.iq_reply_null);
	CHECK(last_state() == XMPP_ST_IDLE);
	CHECK(!strcmp(last_reason(), "stopped"));
	loop_settle();
	CHECK(!state_seen(XMPP_ST_BACKOFF));
	CHECK_UINT(clog.iq_replies, 1);
	CHECK_UINT(xs.stat.failures, 1);
	client_end();
}

/* Answer the pending bind from a sender, and pump until the stream took the
 * answer or counted it as unmatched. */
static void bind_answer_send(const char *from)
{
	uint64_t seen = xs.stat.iq_unmatched;
	uint64_t end = mono_ms() + WAIT_MS;
	char reply[512] = "";
	unsigned int i;

	for (i = 0; i < XMPP_IQ_MAX; i++) {
		if (!xs.iq[i].in_use)
			continue;
		snprintf(reply, sizeof(reply),
			 "<iq type='result' id='%s' from='%s'><bind xmlns='"
			 XMPP_NS_BIND "'><jid>" BOUND_JID "</jid></bind></iq>",
			 xs.iq[i].id, from);
	}
	CHECK(reply[0]);
	fake_send(reply);
	while (xs.stat.iq_unmatched == seen && xs.state == XMPP_ST_BIND &&
	       loop_step(end))
		;
}

/* The sender an iq answer must come from. */
static void test_iq_sender(void)
{
	struct step at_bind[] = {
		SASL_STEPS,
		STEP("<stream:stream", SERVER_HEADER FEATURES_BIND),
		STEP("<bind", NULL),
	};
	struct step steps[] = { NEGOTIATE_STEPS };
	char id[XMPP_ID_MAX];
	uint64_t unmatched;

	CASE("during the bind, an answer from another resource of the account "
	     "leaves the bind pending");
	fake_script_set(at_bind, ARRAY_SIZE(at_bind), false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_step(ARRAY_SIZE(at_bind), WAIT_MS));
	unmatched = xs.stat.iq_unmatched;
	bind_answer_send(USER_JID "/other");
	CHECK_UINT(xs.stat.iq_unmatched, unmatched + 1);
	CHECK(xs.state == XMPP_ST_BIND);
	bind_answer_send(USER_JID);
	CHECK(xs.state != XMPP_ST_BIND);
	CHECK(!state_seen(XMPP_ST_BACKOFF));
	CHECK(!strcmp(xs.bound.resource, "uvoice"));
	client_end();

	CASE("a result from a third JID leaves the request pending");
	CHECK(session_ready(steps, NEGOTIATE_COUNT));
	unmatched = xs.stat.iq_unmatched;
	CHECK(iq_out(id, sizeof(id), NULL) == 0);
	result_send(id, "mallory@evil.example/x");
	CHECK_UINT(clog.iq_replies, 0);
	CHECK_UINT(xs.stat.iq_unmatched, unmatched + 1);

	CASE("the answer of the server afterwards completes the request");
	result_send(id, DOMAIN);
	CHECK_UINT(clog.iq_replies, 1);
	CHECK(!strcmp(clog.iq_reply_type, "result"));

	CASE("the server answers from the account, bare or with the bound "
	     "resource");
	CHECK(iq_out(id, sizeof(id), NULL) == 0);
	result_send(id, USER_JID "/other");
	CHECK_UINT(clog.iq_replies, 1);
	result_send(id, USER_JID);
	CHECK_UINT(clog.iq_replies, 2);
	CHECK(iq_out(id, sizeof(id), NULL) == 0);
	result_send(id, BOUND_JID);
	CHECK_UINT(clog.iq_replies, 3);
	CHECK(iq_out(id, sizeof(id), NULL) == 0);
	result_send(id, NULL);
	CHECK_UINT(clog.iq_replies, 4);

	CASE("a request to a peer takes the answer of that full JID alone");
	unmatched = xs.stat.iq_unmatched;
	CHECK(iq_out(id, sizeof(id), ROMEO) == 0);
	result_send(id, NULL);
	result_send(id, DOMAIN);
	result_send(id, "romeo@montague.example/garden");
	CHECK_UINT(clog.iq_replies, 4);
	CHECK_UINT(xs.stat.iq_unmatched, unmatched + 3);
	result_send(id, ROMEO);
	CHECK_UINT(clog.iq_replies, 5);

	CASE("a peer that does not parse is refused");
	CHECK(iq_out(id, sizeof(id), "@nolocal") == -EINVAL);
	CHECK(xs.state == XMPP_ST_READY);
	client_end();
}

/* Bytes behind <proceed/> or <success/> in the same read. */
static void test_stop_points(void)
{
	struct step proceed_more[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP("<starttls", PROCEED "<iq type='get' id='x1'>"
		     "<ping xmlns='" XMPP_NS_PING "'/></iq>"),
	};
	struct step success_early[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP_ACT("<starttls", PROCEED, STEP_TLS),
		STEP("<stream:stream", SERVER_HEADER FEATURES_SCRAM),
		STEP(auth_scram, challenge),
		STEP(response_final, success_more),
	};

	CASE("an iq behind <proceed/> fails the stream and gets no plain "
	     "answer");
	fake_script_set(proceed_more, ARRAY_SIZE(proceed_more), false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "xml: data after the end of the document"));
	CHECK(!state_seen(XMPP_ST_TLS));
	CHECK(strstr(fake.all, "id=\"x1\"") == NULL);
	client_end();

	CASE("a stream header behind <success/> fails the stream");
	fake_script_set(success_early, ARRAY_SIZE(success_early), false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "xml: data after the end of the document"));
	CHECK(!state_seen(XMPP_ST_BIND));
	client_end();
}

/* Write junk to the socket of the client until it takes no more. */
static size_t client_socket_top_up(void)
{
	static const char junk[4096];
	size_t total = 0;
	ssize_t moved;
	unsigned int i;

	for (i = 0; i < FILL_SENDS_MAX; i++) {
		moved = send(xs.ufd.fd, junk, sizeof(junk),
			     MSG_DONTWAIT | MSG_NOSIGNAL);
		if (moved <= 0)
			break;
		total += (size_t)moved;
	}

	return total;
}

/* Bytes of the client in flight that the server did not acknowledge, -1 when
 * the query fails. */
static int client_socket_unacked(void)
{
	int queued, unsent;

	if (ioctl(xs.ufd.fd, SIOCOUTQ, &queued) ||
	    ioctl(xs.ufd.fd, SIOCOUTQNSD, &unsent))
		return -1;

	return queued - unsent;
}

/*
 * Fill the socket of the client while the server holds its reads. The small
 * receive buffer of the server closes its window, because the client sends
 * each write at once. Only an acknowledgment frees room then, so the fill
 * ends when the socket takes no more and holds no byte in flight. The server
 * acknowledges at once where the kernel lets it; otherwise the fill waits for
 * the delayed acknowledgment.
 * Return: the bytes the socket took, which the server must read before its
 * handshake.
 */
static size_t client_socket_fill(void)
{
	int small = 4096, on = 1;
	size_t total = 0;
	unsigned int round;
	uint64_t end;

	setsockopt(xs.ufd.fd, SOL_SOCKET, SO_SNDBUF, &small, sizeof(small));
	setsockopt(fake.conn.fd, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
	fake.hold = true;
	fake_mask_set();
	for (round = 0; round < FILL_ROUNDS; round++) {
		total += client_socket_top_up();
		if (!client_socket_unacked())
			break;
		setsockopt(fake.conn.fd, IPPROTO_TCP, TCP_QUICKACK, &on,
			   sizeof(on));
		end = mono_ms() + FILL_ACK_MS;
		while (client_socket_unacked() && loop_step(end))
			;
	}

	return total;
}

/*
 * Read and drop the bytes that client_socket_fill() put in the stream. The
 * wait polls the server socket alone, and the kernel sends the queued bytes
 * of the client meanwhile. A client that ran during the drain could fail its
 * handshake and close with unread bytes, and the reset would drop the bytes
 * it had not sent yet.
 */
static bool junk_drain(size_t len)
{
	struct pollfd pfd = { .fd = fake.conn.fd, .events = POLLIN };
	uint64_t end = mono_ms() + WAIT_MS;
	char scratch[4096];
	size_t done = 0;
	size_t want;
	ssize_t moved;
	uint64_t now;

	while (done < len) {
		want = len - done;
		if (want > sizeof(scratch))
			want = sizeof(scratch);
		moved = recv(fake.conn.fd, scratch, want, MSG_DONTWAIT);
		if (moved > 0) {
			done += (size_t)moved;
			continue;
		}
		now = mono_ms();
		if (!moved || errno != EAGAIN || now >= end)
			return false;
		poll(&pfd, 1, (int)(end - now));
	}

	return true;
}

/* Send <proceed/> while the socket of the client is full, then let the
 * server read and start its handshake. */
static void handshake_held_run(const struct step *steps, unsigned int nsteps,
			       const char *proceed)
{
	size_t junk;

	fake_script_set(steps, nsteps, false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_step(2, WAIT_MS));
	junk = client_socket_fill();
	CHECK(junk > 0);
	fake_send(proceed);
	CHECK(wait_state(XMPP_ST_TLS, WAIT_MS));
	CHECK(junk_drain(junk));
	fake.hold = false;
	fake_tls_start();
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
}

/*
 * The handshake starts in the read loop. When its first record finds the
 * socket full, it waits for a write event, and the read loop must keep that
 * wait. The bytes after <proceed/> belong to TLS, also when they arrive in a
 * later read of the same burst.
 */
static void test_handshake_write(void)
{
	static char proceed_late[XMPP_RX_CHUNK + 64];
	struct step steps[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP("<starttls", NULL),
		STEP("<stream:stream", SERVER_HEADER FEATURES_SCRAM),
		STEP(auth_scram, SASL_FAILURE),
	};
	size_t pad = XMPP_RX_CHUNK - (sizeof(PROCEED) - 1);

	CASE("a handshake that waits for a full socket goes on when it drains");
	handshake_held_run(steps, ARRAY_SIZE(steps), PROCEED);
	CHECK(state_seen(XMPP_ST_SASL) != 0);
	CHECK(!strcmp(xs.reason, "sasl: not-authorized"));
	client_end();

	CASE("bytes behind <proceed/> in the next read go to TLS, not the "
	     "parser");
	memset(proceed_late, ' ', pad);
	snprintf(proceed_late + pad, sizeof(proceed_late) - pad,
		 PROCEED "<iq type='get' id='x2'/>");
	handshake_held_run(steps, ARRAY_SIZE(steps), proceed_late);
	CHECK(!strncmp(xs.reason, "tls: ", 5));
	CHECK(!state_seen(XMPP_ST_SASL));
	client_end();
}

/* The SASL mechanism: PLAIN on a verified connection only, and the whole
 * list of names. */
static void test_mechanisms(void)
{
	struct step plain_only[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP_ACT("<starttls", PROCEED, STEP_TLS),
		STEP("<stream:stream", SERVER_HEADER FEATURES_PLAIN),
		STEP(auth_plain, SASL_FAILURE),
	};
	struct step many[] = {
		STEP("<stream:stream", SERVER_HEADER FEATURES_STARTTLS),
		STEP_ACT("<starttls", PROCEED, STEP_TLS),
		STEP("<stream:stream", features_many),
		STEP(auth_scram, SASL_FAILURE),
	};
	struct sip_tls_cfg open = { .verify = false };
	struct sip_tls open_tls;
	struct xmpp_stream_cfg cfg;

	CASE("PLAIN is refused when the account does not verify the server");
	fake_script_set(plain_only, ARRAY_SIZE(plain_only), false);
	client_cfg(&cfg, false);
	cfg.tls_verify = false;
	CHECK(client_start_cfg(&cfg, &cli_tls) == 0);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(!strcmp(xs.reason, "sasl: no usable mechanism"));
	CHECK(strstr(fake.all, "mechanism=\"PLAIN\"") == NULL);
	client_end();

	CASE("PLAIN is refused when the client context does not verify the "
	     "chain");
	sip_tls_init(&open_tls);
	CHECK(sip_tls_ctx_load(&open_tls, &open));
	fake_script_set(plain_only, ARRAY_SIZE(plain_only), false);
	CHECK(client_start(false, &open_tls) == 0);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK(state_seen(XMPP_ST_SASL) == 0);
	CHECK(!strcmp(xs.reason, "sasl: no usable mechanism"));
	CHECK(strstr(fake.all, "mechanism=\"PLAIN\"") == NULL);
	client_end();
	sip_tls_done(&open_tls);

	CASE("SCRAM wins after 16 names this client lacks");
	fake_script_set(many, ARRAY_SIZE(many), false);
	CHECK(client_start(false, &cli_tls) == 0);
	CHECK(wait_report(XMPP_ST_BACKOFF, WAIT_MS));
	CHECK_UINT(fake.step, ARRAY_SIZE(many));
	CHECK(!strcmp(xs.reason, "sasl: not-authorized"));
	client_end();
}

/* Plaintext that a burst of reads leaves inside TLS. */
static void test_tls_held(void)
{
	static const char head[] = "<message from='" JULIET "' type='chat'>"
				   "<body>";
	static const char tail[] = "</body></message>";
	struct step steps[] = { NEGOTIATE_STEPS };
	size_t len = RECORD_LEN * RECORDS;
	size_t head_len = sizeof(head) - 1, tail_len = sizeof(tail) - 1;
	unsigned int stanzas, i;
	uint64_t rx_bytes;
	size_t done;

	CASE("a burst that leaves plaintext inside TLS reads the rest at once");
	CHECK(session_ready(steps, NEGOTIATE_COUNT));
	CHECK_UINT(fake.tx_len, 0);
	stanzas = clog.stanzas;
	rx_bytes = xs.stat.rx_bytes;
	memcpy(big, head, head_len);
	memset(big + head_len, 'z', len - head_len - tail_len);
	memcpy(big + len - tail_len, tail, tail_len);
	for (i = 0; i < RECORDS; i++) {
		CHECK(sip_tls_conn_write(&fake.tls, big + i * RECORD_LEN,
					 RECORD_LEN, &done) == SIP_TLS_OK);
		CHECK_UINT(done, RECORD_LEN);
	}
	CHECK(wait_stanzas(stanzas + 1, WAIT_MS));
	CHECK(!strcmp(clog.name[stanzas], "message"));
	CHECK_UINT(xs.stat.rx_bytes - rx_bytes, len);
	client_end();
}

/* A TLS write that the full socket stopped, with more staged behind it. */
static void test_write_retry(void)
{
	struct step steps[] = { NEGOTIATE_STEPS };
	int small = 4096;
	uint64_t rx_base, tx_base, end;
	unsigned int i;
	size_t len, span;

	len = (size_t)snprintf(big, sizeof(big),
			       "<message to='" JULIET "' type='chat'><body>");
	memset(big + len, 'y', FILL_BODY);
	len += FILL_BODY;
	len += (size_t)snprintf(big + len, sizeof(big) - len,
				"</body></message>");

	CASE("a write retry repeats its span while more is staged behind it");
	CHECK(session_ready(steps, NEGOTIATE_COUNT));
	setsockopt(xs.ufd.fd, SOL_SOCKET, SO_SNDBUF, &small, sizeof(small));
	setsockopt(fake.conn.fd, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
	fake.hold = true;
	fake_mask_set();
	rx_base = fake.rx_total;
	tx_base = xs.stat.tx_bytes;
	for (i = 0; i < FILL_SENDS_MAX && !xs.tx_span; i++) {
		if (xmpp_stream_send(&xs, big, len))
			break;
	}
	CHECK(xs.tx_span != 0);
	span = xs.tx_span;
	CHECK(xmpp_stream_send(&xs, big, len) == 0);
	CHECK(xs.state == XMPP_ST_READY);
	CHECK_UINT(xs.tx_span, span);

	CASE("the socket drains the span, then every byte staged behind it");
	fake.hold = false;
	fake_mask_set();
	end = mono_ms() + WAIT_MS;
	while ((xs.tx_len ||
		fake.rx_total - rx_base < xs.stat.tx_bytes - tx_base) &&
	       loop_step(end))
		;
	CHECK_UINT(xs.tx_len, 0);
	CHECK_UINT(xs.tx_span, 0);
	CHECK_UINT(fake.rx_total - rx_base, xs.stat.tx_bytes - tx_base);
	CHECK(xs.state == XMPP_ST_READY);
	CHECK(!fake.tls_err[0]);
	client_end();
}

/* How many times a text occurs in what the server received. */
static unsigned int text_count(const char *text)
{
	const char *at = fake.all;
	unsigned int count = 0;

	while ((at = strstr(at, text)) != NULL) {
		count++;
		at += strlen(text);
	}

	return count;
}

/**
 * deep_iq_send() - an iq get past the depth bound, with a long id
 * @id_len: the length of its id
 */
static void deep_iq_send(size_t id_len)
{
	unsigned int i;
	size_t len;

	len = (size_t)sprintf(big, "<iq type='get' from='" DOMAIN "' id='");
	memset(big + len, 'i', id_len);
	len += id_len;
	len += (size_t)sprintf(big + len, "'>");
	for (i = 0; i < DEEP_LEVELS; i++)
		len += (size_t)sprintf(big + len, "<a>");
	for (i = 0; i < DEEP_LEVELS; i++)
		len += (size_t)sprintf(big + len, "</a>");
	sprintf(big + len, "</iq>");
	fake_send(big);
}

/* Stanzas over the parser bounds on a ready stream. */
static void test_dropped(void)
{
	struct step steps[] = { NEGOTIATE_STEPS };
	unsigned int stanzas, logs, i;
	uint64_t rx_stanzas;
	size_t len;

	CASE("a stanza over a parser bound is dropped and the stream stays up");
	CHECK(session_ready(steps, NEGOTIATE_COUNT));
	stanzas = clog.stanzas;
	logs = phf.n[PHF_LOG];
	rx_stanzas = xs.stat.rx_stanzas;

	len = (size_t)sprintf(big, "<iq type='get' id='big1' from='" DOMAIN
			      "'>");
	for (i = 0; i < DEEP_LEVELS; i++)
		len += (size_t)sprintf(big + len, "<a>");
	for (i = 0; i < DEEP_LEVELS; i++)
		len += (size_t)sprintf(big + len, "</a>");
	sprintf(big + len, "</iq>");
	fake_send(big);

	len = (size_t)sprintf(big, "<message from='" JULIET "' type='chat'>");
	for (i = 0; i <= XML_NODES_MAX; i++)
		len += (size_t)sprintf(big + len, "<x/>");
	sprintf(big + len, "</message>");
	fake_send(big);

	len = (size_t)sprintf(big, "<iq type='set' id='big2' from='" DOMAIN
			      "'");
	for (i = 0; i <= XML_ATTRS_MAX; i++)
		len += (size_t)sprintf(big + len, " a%u='v'", i);
	sprintf(big + len, "/>");
	fake_send(big);

	fake_send("<message from='" JULIET "' type='chat'><body>after</body>"
		  "</message>");
	CHECK(wait_stanzas(stanzas + 1, WAIT_MS));
	CHECK(xs.state == XMPP_ST_READY);
	CHECK(!strcmp(clog.name[stanzas], "message"));
	CHECK_UINT(clog.stanzas, stanzas + 1);
	CHECK_UINT(phf.n[PHF_LOG], logs + 3);
	CHECK_UINT(xs.stat.rx_stanzas, rx_stanzas + 4);

	CASE("a dropped iq of type get or set gets resource-constraint");
	CHECK(wait_text("id=\"big2\"", WAIT_MS));
	CHECK(strstr(fake.all, "id=\"big1\"") != NULL);
	CHECK_UINT(text_count("<resource-constraint"), 2);

	CASE("the dropped stanzas count in the stream management answer");
	fake_send("<r xmlns='" XMPP_NS_SM "'/>");
	CHECK(wait_text("<a xmlns=\"" XMPP_NS_SM "\" h=\"5\"/>", WAIT_MS));

	CASE("a dropped iq with an id past 2048 bytes gets "
	     "resource-constraint");
	deep_iq_send(LONG_ID);
	fake_send("<r xmlns='" XMPP_NS_SM "'/>");
	CHECK(wait_text("<a xmlns=\"" XMPP_NS_SM "\" h=\"6\"/>", WAIT_MS));
	CHECK_UINT(text_count("<resource-constraint"), 3);
	CHECK_UINT(xs.stat.reply_dropped, 0);

	CASE("a ping with an id past 2048 bytes gets its result");
	len = (size_t)sprintf(big, "<iq type='get' from='" DOMAIN "' id='");
	memset(big + len, 'p', LONG_ID);
	len += LONG_ID;
	sprintf(big + len, "'><ping xmlns='" XMPP_NS_PING "'/></iq>");
	fake_send(big);
	fake_send("<r xmlns='" XMPP_NS_SM "'/>");
	CHECK(wait_text("<a xmlns=\"" XMPP_NS_SM "\" h=\"7\"/>", WAIT_MS));
	CHECK(strstr(fake.all, "<iq type=\"result\" to=\"" DOMAIN
		     "\" id=\"pppp") != NULL);
	CHECK_UINT(xs.stat.reply_dropped, 0);

	CASE("a dropped iq whose answer does not fit XMPP_TX_MAX gets none, "
	     "and the stream stays up");
	deep_iq_send(HUGE_ID);
	fake_send("<r xmlns='" XMPP_NS_SM "'/>");
	CHECK(wait_text("<a xmlns=\"" XMPP_NS_SM "\" h=\"8\"/>", WAIT_MS));
	CHECK_UINT(xs.stat.reply_dropped, 1);
	CHECK_UINT(text_count("<resource-constraint"), 3);
	CHECK(xs.state == XMPP_ST_READY);
	client_end();
}

/* The receive trace and what it must not show. */
static void test_rx_trace(void)
{
	struct step steps[] = { NEGOTIATE_STEPS };
	char reply[512];
	char id[XMPP_ID_MAX];

	CASE("the receive trace names the first element and holds no byte of "
	     "it");
	CHECK(session_ready(steps, NEGOTIATE_COUNT));
	memset(&tcap, 0, sizeof(tcap));
	trace_sink_set(trace_take);
	trace_ring_set(1u << TRACE_XMPP);
	CHECK(iq_out(id, sizeof(id), NULL) == 0);
	snprintf(reply, sizeof(reply), "<iq type='result' id='%s'>"
		 "<services xmlns='urn:xmpp:extdisco:2'><service type='turn' "
		 "host='turn.example' port='3478' username='u1' password='"
		 TURN_SECRET "'/></services></iq>", id);
	fake_send(reply);
	wait_replies(1, WAIT_MS);
	trace_ring_set(0);
	trace_sink_set(NULL);
	CHECK_UINT(clog.iq_replies, 1);
	CHECK(tcap.lines > 0);
	CHECK(strstr(tcap.text, "bytes, first element iq") != NULL);
	CHECK(strstr(tcap.text, TURN_SECRET) == NULL);
	client_end();
}

/*
 * The state words.
 */
static void test_names(void)
{
	CASE("every state has a word");
	CHECK(!strcmp(xmpp_stream_state_name(XMPP_ST_IDLE), "idle"));
	CHECK(!strcmp(xmpp_stream_state_name(XMPP_ST_READY), "ready"));
	CHECK(!strcmp(xmpp_stream_state_name(XMPP_ST_BACKOFF), "backoff"));
	CHECK(!strcmp(xmpp_stream_state_name(XMPP_ST_MAX), "unknown"));
}

int main(void)
{
	struct sip_tls_cfg client = { .ca_path = fix_ca, .verify = true };
	struct sip_tls_cfg server = { .cert_path = fix_cert, .key_path = fix_key };
	struct sip_dns_cb dcb = { .done = dns_done };
	struct sip_dns_cfg dcfg;
	bool fixtures_ok = fixtures_take();

	signal(SIGPIPE, SIG_IGN);
	alarm(240);

	SUITE_CHECK(fixtures_ok);
	if (!fixtures_ok) {
		printf("    set UVOICE_TLS_FIXTURES to test/unit/tls\n");
		return tu_summary("test_xmpp_stream");
	}

	uloop_init();
	proto_host_fake_reset();
	sasl_texts_build();
	features_many_build();

	sip_tls_init(&cli_tls);
	sip_tls_init(&srv_tls);
	SUITE_CHECK(sip_tls_ctx_load(&cli_tls, &client));
	SUITE_CHECK(sip_tls_ctx_load(&srv_tls, &server));
	SUITE_CHECK(sip_tls_server_ready(&srv_tls));

	sip_dns_cfg_default(&dcfg);
	dcfg.srv_fallback = true;
	sip_dns_init(&dns, &dcfg, &dcb, NULL);

	SUITE_CHECK(fake_listen());

	test_names();
	test_cadences();
	test_full_session();
	test_direct_tls_plain();
	test_refusals();
	test_broken_connection();
	test_iq_sender();
	test_stop_points();
	test_handshake_write();
	test_mechanisms();
	test_tls_held();
	test_write_retry();
	test_dropped();
	test_rx_trace();

	fake_conn_close();
	sip_dns_done(&dns);
	sip_tls_done(&cli_tls);
	sip_tls_done(&srv_tls);
	uloop_done();

	return tu_summary("test_xmpp_stream");
}
