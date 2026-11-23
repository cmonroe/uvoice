// SPDX-License-Identifier: LGPL-2.1-only
/*
 * uvtransport - unit suite
 * Copyright (C) 2026 John Crispin <john@phrozen.org>
 */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* The suite reads stream internals and sets the test hooks of the library. */
#include "../uvtransport.c"

#define HANDSHAKE_MS	15000U
#define MEDIA_MS	3000U
/* Bound of the whole suite, so a deadlock fails the run and does not hang. */
#define SUITE_S		300U
#define PKT_MAX		256U
#define RTP_LEN		32U
#define RTCP_RR_LEN	8U
#define RTP_DSCP	46U
#define RTCP_DSCP	26U
#define FLOW_WINDOW	8U
#define FLOW_RTP	20U
#define INJECT_COUNT	32U
#define INJECT_LEN	100U
#define SENT_LOG	32U
/* The send log marks a DTLS record with its content type above this bit. */
#define SENT_DTLS	0x10000U
#define DTLS_ALERT	21U
/*
 * First DTLS timeout of the suite, so a lost flight costs milliseconds. It
 * stays above 15 ms: DTLSv1_get_timeout() reports a timer as due below that.
 */
#define DTLS_TIMER_US	50000U
/* OpenSSL starts the DTLS timer at 1 s when no timer callback is set. */
#define DTLS_DEFAULT_MS	1000L

#define CASE(n) do { current_case = (n); } while (0)

#define CHECK(expr) do {						\
	tests_run++;							\
	if (!(expr)) {							\
		tests_failed++;						\
		fprintf(stderr, "not ok - %s: %s (%s:%d)\n",		\
			current_case, #expr, __FILE__, __LINE__);	\
	}								\
} while (0)

struct side {
	struct uvt_stream *stream;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	unsigned int candidate_events;
	bool gathering_done;
	bool done_forwarded;
	int ice_state;
	bool dtls_up;
	unsigned int dtls_fail;
	bool dtls_closed;
	uint8_t rtp[PKT_MAX];
	size_t rtp_len;
	unsigned int rtp_count;
	uint8_t rtcp[PKT_MAX];
	size_t rtcp_len;
	unsigned int rtcp_count;
	unsigned int dtls_drop;
	unsigned int dtls_dropped;
	long dtls_timer_first;
	int ds_dtls;
	int ds_rtp;
	int ds_rtcp;
	unsigned int ds_wrong;
	bool pause_armed;
	bool paused;
	bool pause_hold;
	unsigned int sent[SENT_LOG];
	unsigned int sent_count;
};

/* One thread inside an API call while another thread destroys the stream. */
struct inflight {
	pthread_mutex_t lock;
	pthread_cond_t cond;
	struct uvt_stream *stream;
	const char *pause_at;
	int (*call)(struct uvt_stream *stream);
	bool paused;
	bool released;
	bool returned;
	bool destroy_waits;
	bool destroy_early;
	int ret;
};

struct flow {
	struct side *from;
	struct side *to;
	unsigned int sent;
	int last;
	bool stalled;
};

typedef bool (*pair_done_fn)(struct side *caller, struct side *callee);

static unsigned int tests_run;
static unsigned int tests_failed;
static const char *current_case;
static int wake_fd[2] = { -1, -1 };

static struct {
	struct uvt_stream *stream;
	enum uvt_remote state;
	int ret;
} reenter;

static struct inflight inflight = {
	.lock = PTHREAD_MUTEX_INITIALIZER,
	.cond = PTHREAD_COND_INITIALIZER,
};

/**
 * wake_post() - tell the waiters of the suite that a callback changed a side
 * @side: the side
 */
static void wake_post(struct side *side)
{
	char byte = 1;

	pthread_cond_broadcast(&side->cond);
	while (write(wake_fd[1], &byte, 1) < 0 && errno == EINTR)
		;
}

/**
 * wake_wait() - wait for a callback of either side or for a timeout
 * @timeout_ms: the longest wait
 */
static void wake_wait(int timeout_ms)
{
	struct pollfd pfd = { .fd = wake_fd[0], .events = POLLIN };
	char buf[64];

	if (poll(&pfd, 1, timeout_ms) <= 0)
		return;
	while (read(wake_fd[0], buf, sizeof(buf)) > 0)
		;
}

/**
 * clock_ms() - the monotonic clock
 *
 * Return: milliseconds.
 */
static uint64_t clock_ms(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);

	return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
}

/**
 * side_rtp() - record the plaintext of one received RTP packet
 * @user: the side
 * @buf: the packet
 * @len: its length
 */
static void side_rtp(void *user, const uint8_t *buf, size_t len)
{
	struct side *side = user;

	pthread_mutex_lock(&side->lock);
	if (len <= sizeof(side->rtp)) {
		memcpy(side->rtp, buf, len);
		side->rtp_len = len;
	}
	side->rtp_count++;
	pthread_mutex_unlock(&side->lock);
	wake_post(side);
}

/**
 * side_rtcp() - record the plaintext of one received RTCP packet
 * @user: the side
 * @buf: the packet
 * @len: its length
 */
static void side_rtcp(void *user, const uint8_t *buf, size_t len)
{
	struct side *side = user;

	pthread_mutex_lock(&side->lock);
	if (len <= sizeof(side->rtcp)) {
		memcpy(side->rtcp, buf, len);
		side->rtcp_len = len;
	}
	side->rtcp_count++;
	pthread_mutex_unlock(&side->lock);
	wake_post(side);
}

/**
 * side_event() - record one transport event
 * @user: the side
 * @event: the event
 * @arg: its argument
 */
static void side_event(void *user, enum uvt_event event, int arg)
{
	struct side *side = user;

	pthread_mutex_lock(&side->lock);
	switch (event) {
	case UVT_EV_CANDIDATE:
		side->candidate_events++;
		break;
	case UVT_EV_GATHERING_DONE:
		side->gathering_done = true;
		break;
	case UVT_EV_ICE_STATE:
		side->ice_state = arg;
		break;
	case UVT_EV_DTLS_UP:
		side->dtls_up = true;
		break;
	case UVT_EV_DTLS_FAIL:
		side->dtls_fail++;
		break;
	case UVT_EV_DTLS_CLOSED:
		side->dtls_closed = true;
		break;
	}
	pthread_mutex_unlock(&side->lock);
	wake_post(side);
}

/**
 * timer_ms() - the time to the next DTLS timer of a side
 * @side: the side
 *
 * uvt_tick() sends a lost flight again only when this timer is due, and no
 * callback marks that moment.
 *
 * Return: milliseconds, or -1 when no handshake timer runs.
 */
static long timer_ms(struct side *side)
{
	struct uvt_stream *stream = side->stream;
	struct timeval left;
	long ms = -1;

	stream_lock(stream);
	if (stream->ssl && !stream->dtls_up && !stream->dtls_failed &&
	    DTLSv1_get_timeout(stream->ssl, &left))
		ms = (long)left.tv_sec * 1000L +
		     ((long)left.tv_usec + 999L) / 1000L;
	stream_unlock(stream);

	return ms;
}

/**
 * test_dtls_timer() - the DTLS timer of the suite
 * @ssl: the DTLS object
 * @timer_us: the last timeout, 0 for the first one
 *
 * Return: DTLS_TIMER_US first, then the double of the last timeout, as
 * OpenSSL does.
 */
static unsigned int test_dtls_timer(SSL *ssl, unsigned int timer_us)
{
	return timer_us ? 2U * timer_us : DTLS_TIMER_US;
}

/**
 * sent_log() - record one sent datagram, and hold the sender on request
 * @side: the sending side, with its lock held
 * @data: the datagram
 * @dtls: whether it is a DTLS record
 *
 * The log holds bytes 2 and 3 of a media datagram, which are the RTP sequence
 * number in plain text, or SENT_DTLS with the content type of a DTLS record.
 * An armed pause holds the next media sender, so the output queue fills
 * behind it.
 */
static void sent_log(struct side *side, const uint8_t *data, bool dtls)
{
	if (side->sent_count < SENT_LOG)
		side->sent[side->sent_count++] = dtls ?
			SENT_DTLS | data[0] :
			(unsigned int)data[2] << 8 | data[3];
	if (dtls || !side->pause_armed)
		return;
	side->pause_armed = false;
	side->paused = true;
	pthread_cond_broadcast(&side->cond);
	while (side->pause_hold)
		pthread_cond_wait(&side->cond, &side->lock);
}

/**
 * test_send() - the send hook: check the TOS byte and drop DTLS on request
 * @stream: the sending stream
 * @data: the datagram
 * @len: its length
 * @ds: the TOS byte that goes to libjuice
 *
 * OpenSSL starts the DTLS timer before it writes a flight, so the timer read
 * here at the first DTLS datagram is the first timeout.
 *
 * Return: true to drop the datagram as a failed send.
 */
static bool test_send(struct uvt_stream *stream, const uint8_t *data,
		      size_t len, int ds)
{
	struct side *side = stream->cfg.user;
	bool dtls = data[0] >= 20 && data[0] <= 63;
	long timer = dtls ? timer_ms(side) : -1;
	bool drop = false;
	int want;

	pthread_mutex_lock(&side->lock);
	if (dtls) {
		if (side->dtls_timer_first < 0)
			side->dtls_timer_first = timer;
		side->ds_dtls = ds;
		want = (int)(RTP_DSCP << 2);
		if (side->dtls_drop) {
			side->dtls_drop--;
			side->dtls_dropped++;
			drop = true;
		}
	} else if (rtcp_is(data, len)) {
		side->ds_rtcp = ds;
		want = (int)(RTCP_DSCP << 2);
	} else {
		side->ds_rtp = ds;
		want = (int)(RTP_DSCP << 2);
	}
	if (ds != want)
		side->ds_wrong++;
	sent_log(side, data, dtls);
	pthread_mutex_unlock(&side->lock);

	return drop;
}

/**
 * inflight_entry() - the entry hook of the cases "destroy waits for a call"
 * @stream: the stream of the hook
 * @call: the name of the hook point
 *
 * The call under test stops at its hook point until the test releases it.
 * uvt_destroy() reports each wait for a call in progress. A destroy that
 * reaches libjuice while the call is still held at its hook point is a
 * fault. That hook then waits for the call to return, so the fault frees
 * nothing in use.
 *
 * Return: true when @stream is the stream of the case.
 */
static bool inflight_entry(struct uvt_stream *stream, const char *call)
{
	struct inflight *f = &inflight;

	pthread_mutex_lock(&f->lock);
	if (!stream || stream != f->stream) {
		pthread_mutex_unlock(&f->lock);
		return false;
	}
	if (!strcmp(call, f->pause_at) && !f->paused) {
		f->paused = true;
		pthread_cond_broadcast(&f->cond);
		while (!f->released)
			pthread_cond_wait(&f->cond, &f->lock);
	} else if (!strcmp(call, "uvt_destroy")) {
		f->destroy_waits = true;
		pthread_cond_broadcast(&f->cond);
	} else if (!strcmp(call, "ice_destroy")) {
		f->destroy_early = !f->released;
		pthread_cond_broadcast(&f->cond);
		while (!f->returned)
			pthread_cond_wait(&f->cond, &f->lock);
	}
	pthread_mutex_unlock(&f->lock);

	return true;
}

/**
 * test_entry() - the libjuice entry hook: set the far end a second time
 * @stream: the stream of the libjuice call
 * @call: the name of the libjuice wrapper
 *
 * The second uvt_remote_set() runs while the first one installs the far end
 * in libjuice with no lock held.
 */
static void test_entry(struct uvt_stream *stream, const char *call)
{
	static const uint8_t fingerprint[UVT_FINGERPRINT_LEN];

	if (inflight_entry(stream, call))
		return;
	if (strcmp(call, "ice_remote_set") || stream != reenter.stream)
		return;
	reenter.stream = NULL;
	stream_lock(stream);
	reenter.state = stream->remote;
	stream_unlock(stream);
	reenter.ret = uvt_remote_set(stream, "abcd", "abcdefghijklmnopqrstuv",
				     fingerprint, UVT_SETUP_ACTPASS);
}

/**
 * lock_faults_check() - no libjuice call ran with a stream lock held
 */
static void lock_faults_check(void)
{
	CHECK(atomic_load(&hook_lock_faults) == 0);
}

/**
 * side_open() - make one side with host candidates alone
 * @side: the side
 * @role: its ICE role
 * @setup: its DTLS setup
 *
 * Return: true when the stream exists.
 */
static bool side_open(struct side *side, enum uvt_role role,
		      enum uvt_setup setup)
{
	pthread_condattr_t attr;
	struct uvt_cfg cfg;

	memset(side, 0, sizeof(*side));
	pthread_mutex_init(&side->lock, NULL);
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&side->cond, &attr);
	pthread_condattr_destroy(&attr);
	side->ice_state = -1;
	side->dtls_timer_first = -1;
	side->ds_dtls = -1;
	side->ds_rtp = -1;
	side->ds_rtcp = -1;

	memset(&cfg, 0, sizeof(cfg));
	cfg.role = role;
	cfg.setup = setup;
	cfg.on_rtp = side_rtp;
	cfg.on_rtcp = side_rtcp;
	cfg.on_event = side_event;
	cfg.user = side;
	cfg.rtp_dscp = RTP_DSCP;
	cfg.rtcp_dscp = RTCP_DSCP;
	side->stream = uvt_create(&cfg);

	return side->stream != NULL;
}

/**
 * side_close() - release one side
 * @side: the side
 */
static void side_close(struct side *side)
{
	uvt_destroy(side->stream);
	side->stream = NULL;
	pthread_cond_destroy(&side->cond);
	pthread_mutex_destroy(&side->lock);
}

/**
 * candidates_forward() - move the gathered candidates of one side to the other
 * @from: the side that gathered
 * @to: the side that learns them
 */
static void candidates_forward(struct side *from, struct side *to)
{
	char line[UVT_CANDIDATE_MAX];
	bool done;

	while (uvt_candidate_next(from->stream, line, sizeof(line)) == 1)
		uvt_candidate_add(to->stream, line);

	pthread_mutex_lock(&from->lock);
	done = from->gathering_done && !from->done_forwarded;
	if (done)
		from->done_forwarded = true;
	pthread_mutex_unlock(&from->lock);

	if (done)
		uvt_remote_done(to->stream);
}

/**
 * flag_read() - read one flag of a side under its lock
 * @side: the side
 * @flag: the flag inside it
 *
 * Return: the flag.
 */
static bool flag_read(struct side *side, const bool *flag)
{
	bool value;

	pthread_mutex_lock(&side->lock);
	value = *flag;
	pthread_mutex_unlock(&side->lock);

	return value;
}

/**
 * count_read() - read one counter of a side under its lock
 * @side: the side
 * @count: the counter inside it
 *
 * Return: the counter.
 */
static unsigned int count_read(struct side *side, const unsigned int *count)
{
	unsigned int value;

	pthread_mutex_lock(&side->lock);
	value = *count;
	pthread_mutex_unlock(&side->lock);

	return value;
}

/**
 * rbio_pending() - the bytes that wait in the DTLS input BIO of a side
 * @side: the side
 *
 * Return: the byte count, 0 without a DTLS object.
 */
static size_t rbio_pending(struct side *side)
{
	struct uvt_stream *stream = side->stream;
	size_t pending = 0;

	stream_lock(stream);
	if (stream->rbio)
		pending = BIO_ctrl_pending(stream->rbio);
	stream_unlock(stream);

	return pending;
}

/**
 * dtls_inject() - hand INJECT_COUNT DTLS datagrams to a side as if received
 * @side: the side
 * @type: the DTLS content type of the record header
 */
static void dtls_inject(struct side *side, uint8_t type)
{
	char record[INJECT_LEN];
	unsigned int i;

	memset(record, 0, sizeof(record));
	record[0] = (char)type;
	record[1] = (char)0xfe;
	record[2] = (char)0xfd;
	for (i = 0; i < INJECT_COUNT; i++)
		on_recv(side->stream->agent, record, sizeof(record),
			side->stream);
}

/**
 * pair_pump() - exchange candidates and tick both sides until done
 * @caller: the controlling side
 * @callee: the controlled side
 * @done: the condition to wait for
 * @timeout_ms: how long to wait at most
 *
 * Between two passes it waits for a callback of either side, the next DTLS
 * timer of either side or the deadline, whichever comes first.
 *
 * Return: the condition at the end.
 */
static bool pair_pump(struct side *caller, struct side *callee,
		      pair_done_fn done, unsigned int timeout_ms)
{
	uint64_t deadline = clock_ms() + timeout_ms;
	uint64_t now;
	long wait, timer;

	for (;;) {
		candidates_forward(caller, callee);
		candidates_forward(callee, caller);
		uvt_tick(caller->stream);
		uvt_tick(callee->stream);
		if (done(caller, callee))
			return true;
		now = clock_ms();
		if (now >= deadline)
			return false;
		wait = (long)(deadline - now);
		timer = timer_ms(caller);
		if (timer >= 0 && timer < wait)
			wait = timer;
		timer = timer_ms(callee);
		if (timer >= 0 && timer < wait)
			wait = timer;
		wake_wait((int)wait);
	}
}

/**
 * both_up() - both handshakes completed
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool both_up(struct side *caller, struct side *callee)
{
	return flag_read(caller, &caller->dtls_up) &&
	       flag_read(callee, &callee->dtls_up);
}

/**
 * both_fail() - both handshakes failed
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool both_fail(struct side *caller, struct side *callee)
{
	return count_read(caller, &caller->dtls_fail) >= 1 &&
	       count_read(callee, &callee->dtls_fail) >= 1;
}

/**
 * callee_closed() - the callee saw the close of the caller
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool callee_closed(struct side *caller, struct side *callee)
{
	return flag_read(callee, &callee->dtls_closed);
}

/**
 * callee_has_rtp() - the callee received an RTP packet
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool callee_has_rtp(struct side *caller, struct side *callee)
{
	return count_read(callee, &callee->rtp_count) >= 1;
}

/**
 * callee_has_flow() - the callee received FLOW_RTP packets
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool callee_has_flow(struct side *caller, struct side *callee)
{
	return count_read(callee, &callee->rtp_count) >= FLOW_RTP;
}

/**
 * caller_has_rtcp() - the caller received an RTCP packet
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool caller_has_rtcp(struct side *caller, struct side *callee)
{
	return count_read(caller, &caller->rtcp_count) >= 1;
}

/**
 * timer_first_read() - the first DTLS timeout of a side
 * @side: the side
 *
 * Return: milliseconds, or -1 before the side sent its first flight.
 */
static long timer_first_read(struct side *side)
{
	long value;

	pthread_mutex_lock(&side->lock);
	value = side->dtls_timer_first;
	pthread_mutex_unlock(&side->lock);

	return value;
}

/**
 * callee_sent_flight() - the callee sent its first DTLS flight
 * @caller: the controlling side
 * @callee: the controlled side
 *
 * Return: the condition.
 */
static bool callee_sent_flight(struct side *caller, struct side *callee)
{
	return timer_first_read(callee) >= 0;
}

/**
 * pair_credentials() - exchange the ICE credentials in the order of a call
 * @caller: the controlling side, which describes itself first
 * @callee: the controlled side, which learns the caller before it describes
 * @fp_for_caller: the fingerprint the caller verifies the callee against
 * @fp_for_callee: the fingerprint the callee verifies the caller against
 *
 * Return: true when every call succeeded.
 */
static bool pair_credentials(struct side *caller, struct side *callee,
			     const uint8_t *fp_for_caller,
			     const uint8_t *fp_for_callee)
{
	char caller_ufrag[UVT_UFRAG_MAX], caller_pwd[UVT_PWD_MAX];
	char callee_ufrag[UVT_UFRAG_MAX], callee_pwd[UVT_PWD_MAX];

	if (uvt_local_get(caller->stream, caller_ufrag, caller_pwd))
		return false;
	if (uvt_remote_set(callee->stream, caller_ufrag, caller_pwd,
			   fp_for_callee, UVT_SETUP_ACTPASS))
		return false;
	if (uvt_local_get(callee->stream, callee_ufrag, callee_pwd))
		return false;

	return uvt_remote_set(caller->stream, callee_ufrag, callee_pwd,
			      fp_for_caller, UVT_SETUP_ACTIVE) == 0;
}

/**
 * pair_open() - two connected sides with their handshake completed
 * @caller: receives the controlling side
 * @callee: receives the controlled side
 *
 * Return: true when both sides are up.
 */
static bool pair_open(struct side *caller, struct side *callee)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	bool ok;

	ok = side_open(caller, UVT_ROLE_CONTROLLING, UVT_SETUP_ACTPASS);
	ok = side_open(callee, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE) && ok;
	if (!ok || uvt_fingerprint_get(fingerprint) ||
	    !pair_credentials(caller, callee, fingerprint, fingerprint))
		return false;

	return pair_pump(caller, callee, both_up, HANDSHAKE_MS);
}

/**
 * rtp_build() - one PCMU packet with a known payload
 * @pkt: receives RTP_LEN bytes
 * @seq: the sequence number
 *
 * Return: RTP_LEN.
 */
static size_t rtp_build(uint8_t *pkt, uint16_t seq)
{
	size_t i;

	memset(pkt, 0, RTP_LEN);
	pkt[0] = 0x80;
	pkt[1] = 0;
	pkt[2] = (uint8_t)(seq >> 8);
	pkt[3] = (uint8_t)seq;
	pkt[7] = 0xa0;
	pkt[8] = 0x12;
	pkt[9] = 0x34;
	pkt[10] = 0x56;
	pkt[11] = 0x78;
	for (i = 12; i < RTP_LEN; i++)
		pkt[i] = (uint8_t)(0xd5 ^ i);

	return RTP_LEN;
}

/**
 * rtcp_rr_build() - one empty receiver report
 * @pkt: receives RTCP_RR_LEN bytes
 *
 * Return: RTCP_RR_LEN.
 */
static size_t rtcp_rr_build(uint8_t *pkt)
{
	memset(pkt, 0, RTCP_RR_LEN);
	pkt[0] = 0x80;
	pkt[1] = 201;
	pkt[3] = 1;
	pkt[4] = 0x12;
	pkt[5] = 0x34;
	pkt[6] = 0x56;
	pkt[7] = 0x78;

	return RTCP_RR_LEN;
}

/**
 * flow_wait() - wait until the sender is less than FLOW_WINDOW packets ahead
 * @flow: the flow
 * @until: the CLOCK_MONOTONIC deadline, which sets flow->stalled
 *
 * Return: true once the receiver saw the close.
 */
static bool flow_wait(struct flow *flow, const struct timespec *until)
{
	struct side *to = flow->to;
	bool closed;

	pthread_mutex_lock(&to->lock);
	while (!flow->stalled && !to->dtls_closed &&
	       to->rtp_count + FLOW_WINDOW <= flow->sent)
		flow->stalled = pthread_cond_timedwait(&to->cond, &to->lock,
						       until) != 0;
	closed = to->dtls_closed;
	pthread_mutex_unlock(&to->lock);

	return closed;
}

/**
 * flow_run() - send RTP until the stream refuses it
 * @arg: the flow
 *
 * The sender stays at most FLOW_WINDOW packets ahead of the receiver, so no
 * socket buffer drops a packet. The receiver sees the close only after the
 * sender closed, so the next send after that must fail.
 *
 * Return: NULL.
 */
static void *flow_run(void *arg)
{
	struct flow *flow = arg;
	uint8_t pkt[PKT_MAX];
	struct timespec until;
	bool closed = false;
	uint16_t seq = 1;
	int ret;

	clock_gettime(CLOCK_MONOTONIC, &until);
	until.tv_sec += HANDSHAKE_MS / 1000U;
	for (;;) {
		rtp_build(pkt, seq++);
		ret = uvt_send_rtp(flow->from->stream, pkt, RTP_LEN);
		if (ret && ret != -ENOBUFS)
			break;
		if (!ret)
			flow->sent++;
		if (closed)
			break;
		closed = flow_wait(flow, &until);
		if (flow->stalled)
			break;
	}
	flow->last = ret;

	return NULL;
}

/*
 * The process identity and the refusals of one stream.
 */
static void test_local(void)
{
	char ufrag[UVT_UFRAG_MAX], pwd[UVT_PWD_MAX];
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	uint8_t again[UVT_FINGERPRINT_LEN];
	struct side side;

	CASE("local");
	CHECK(uvt_init() == 0);
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	CHECK(uvt_fingerprint_get(again) == 0);
	CHECK(!memcmp(fingerprint, again, sizeof(fingerprint)));

	CHECK(side_open(&side, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	CHECK(uvt_local_get(side.stream, ufrag, pwd) == 0);
	CHECK(strlen(ufrag) >= 4);
	CHECK(strlen(pwd) >= 22);
	CHECK(uvt_up(side.stream) == false);
	CHECK(uvt_send_rtp(side.stream, fingerprint, sizeof(fingerprint)) ==
	      -EAGAIN);
	CHECK(uvt_remote_set(side.stream, "ab", pwd, fingerprint,
			     UVT_SETUP_ACTPASS) == -EINVAL);
	CHECK(uvt_remote_set(side.stream, ufrag, "short", fingerprint,
			     UVT_SETUP_ACTPASS) == -EINVAL);
	CHECK(uvt_remote_set(side.stream, ufrag, pwd, fingerprint,
			     UVT_SETUP_ACTPASS) == 0);
	CHECK(uvt_remote_set(side.stream, ufrag, pwd, fingerprint,
			     UVT_SETUP_ACTPASS) == -EALREADY);
	side_close(&side);
	lock_faults_check();
}

/*
 * The ICE credentials keep to the ice-char set and length.
 */
static void test_credentials(void)
{
	static const char pwd[] = "abcdefghij+/0123456789AB";
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	char longest[UVT_UFRAG_MAX + 1];
	struct side side;

	CASE("credentials");
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	CHECK(side_open(&side, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	CHECK(uvt_remote_set(side.stream, "ab-d", pwd, fingerprint,
			     UVT_SETUP_ACTPASS) == -EINVAL);
	CHECK(uvt_remote_set(side.stream, "abcd\r\na=ice-lite", pwd,
			     fingerprint, UVT_SETUP_ACTPASS) == -EINVAL);
	CHECK(uvt_remote_set(side.stream, "abcd", "abcdefghij0123456789AB\r\n",
			     fingerprint, UVT_SETUP_ACTPASS) == -EINVAL);
	CHECK(uvt_remote_set(side.stream, "abcd", "abcdefghij 0123456789AB",
			     fingerprint, UVT_SETUP_ACTPASS) == -EINVAL);
	memset(longest, 'a', UVT_UFRAG_MAX);
	longest[UVT_UFRAG_MAX] = 0;
	CHECK(uvt_remote_set(side.stream, longest, pwd, fingerprint,
			     UVT_SETUP_ACTPASS) == -EINVAL);
	longest[UVT_UFRAG_MAX - 1] = 0;
	CHECK(uvt_remote_set(side.stream, longest, pwd, fingerprint,
			     UVT_SETUP_ACTPASS) == 0);
	side_close(&side);
	lock_faults_check();
}

/*
 * A second far end is refused while the first installs.
 */
static void test_remote_twice(void)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	struct side side;

	CASE("remote twice");
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	CHECK(side_open(&side, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	reenter.stream = side.stream;
	reenter.state = UVT_REMOTE_NONE;
	reenter.ret = 0;
	CHECK(uvt_remote_set(side.stream, "wxyz", "ABCDEFGHIJKLMNOPQRSTUV",
			     fingerprint, UVT_SETUP_ACTPASS) == 0);
	CHECK(reenter.stream == NULL);
	CHECK(reenter.state == UVT_REMOTE_INSTALLING);
	CHECK(reenter.ret == -EALREADY);
	CHECK(uvt_remote_set(side.stream, "wxyz", "ABCDEFGHIJKLMNOPQRSTUV",
			     fingerprint, UVT_SETUP_ACTPASS) == -EALREADY);
	side_close(&side);
	lock_faults_check();
}

/*
 * The first DTLS timeout without the timer of the suite.
 *
 * The callee is the DTLS client, so it sends the first flight. The case ends
 * after that flight and waits for no DTLS timer.
 */
static void test_timer_default(void)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	struct side caller, callee;
	long first;

	CASE("timer default");
	hook_dtls_timer = NULL;
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	CHECK(side_open(&caller, UVT_ROLE_CONTROLLING, UVT_SETUP_ACTPASS));
	CHECK(side_open(&callee, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	CHECK(pair_credentials(&caller, &callee, fingerprint, fingerprint));
	CHECK(pair_pump(&caller, &callee, callee_sent_flight, HANDSHAKE_MS));
	first = timer_first_read(&callee);
	CHECK(first > DTLS_DEFAULT_MS - 100L);
	CHECK(first <= DTLS_DEFAULT_MS);
	side_close(&caller);
	side_close(&callee);
	hook_dtls_timer = test_dtls_timer;
	lock_faults_check();
}

/*
 * Two streams connect, complete the handshake and exchange media.
 */
static void test_handshake_and_media(void)
{
	uint8_t pkt[PKT_MAX];
	struct side caller, callee;
	size_t len;

	CASE("handshake");
	CHECK(pair_open(&caller, &callee));
	CHECK(uvt_up(caller.stream));
	CHECK(uvt_up(callee.stream));
	CHECK(count_read(&caller, &caller.candidate_events) >= 1);
	CHECK(count_read(&callee, &callee.candidate_events) >= 1);
	CHECK(uvt_ice_state(caller.stream) == UVT_ICE_CONNECTED ||
	      uvt_ice_state(caller.stream) == UVT_ICE_COMPLETED);
	CHECK(count_read(&caller, &caller.dtls_fail) == 0);
	CHECK(count_read(&callee, &callee.dtls_fail) == 0);
	CHECK(timer_first_read(&callee) >= 0);
	CHECK(timer_first_read(&callee) <= (long)(DTLS_TIMER_US / 1000U));

	CASE("rtp");
	len = rtp_build(pkt, 1);
	CHECK(uvt_send_rtp(caller.stream, pkt, len) == 0);
	CHECK(pair_pump(&caller, &callee, callee_has_rtp, MEDIA_MS));
	pthread_mutex_lock(&callee.lock);
	CHECK(callee.rtp_len == len);
	CHECK(!memcmp(callee.rtp, pkt, len));
	pthread_mutex_unlock(&callee.lock);

	CASE("rtcp");
	len = rtcp_rr_build(pkt);
	CHECK(uvt_send_rtcp(callee.stream, pkt, len) == 0);
	CHECK(pair_pump(&caller, &callee, caller_has_rtcp, MEDIA_MS));
	pthread_mutex_lock(&caller.lock);
	CHECK(caller.rtcp_len == len);
	CHECK(!memcmp(caller.rtcp, pkt, len));
	pthread_mutex_unlock(&caller.lock);

	CASE("dscp");
	pthread_mutex_lock(&caller.lock);
	CHECK(caller.ds_dtls == (int)(RTP_DSCP << 2));
	CHECK(caller.ds_rtp == (int)(RTP_DSCP << 2));
	CHECK(caller.ds_wrong == 0);
	pthread_mutex_unlock(&caller.lock);
	pthread_mutex_lock(&callee.lock);
	CHECK(callee.ds_dtls == (int)(RTP_DSCP << 2));
	CHECK(callee.ds_rtcp == (int)(RTCP_DSCP << 2));
	CHECK(callee.ds_wrong == 0);
	pthread_mutex_unlock(&callee.lock);

	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

/*
 * The DTLS timer sends a lost record again.
 *
 * Each side loses its first DTLS datagram: the ClientHello of the callee and
 * the first record of the caller's answer.
 */
static void test_lost_record(void)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	struct side caller, callee;

	CASE("lost record");
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	CHECK(side_open(&caller, UVT_ROLE_CONTROLLING, UVT_SETUP_ACTPASS));
	CHECK(side_open(&callee, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	pthread_mutex_lock(&caller.lock);
	caller.dtls_drop = 1;
	pthread_mutex_unlock(&caller.lock);
	pthread_mutex_lock(&callee.lock);
	callee.dtls_drop = 1;
	pthread_mutex_unlock(&callee.lock);
	CHECK(pair_credentials(&caller, &callee, fingerprint, fingerprint));
	CHECK(pair_pump(&caller, &callee, both_up, HANDSHAKE_MS));
	CHECK(count_read(&caller, &caller.dtls_dropped) == 1);
	CHECK(count_read(&callee, &callee.dtls_dropped) == 1);
	CHECK(count_read(&caller, &caller.dtls_fail) == 0);
	CHECK(count_read(&callee, &callee.dtls_fail) == 0);
	CHECK(uvt_up(caller.stream));
	CHECK(uvt_up(callee.stream));
	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

/*
 * A wrong fingerprint fails both sides once.
 */
static void test_fingerprint_mismatch(void)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	uint8_t wrong[UVT_FINGERPRINT_LEN];
	struct side caller, callee;
	size_t caller_bio, callee_bio;

	CASE("fingerprint");
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	memcpy(wrong, fingerprint, sizeof(wrong));
	wrong[0] ^= 0xff;
	CHECK(side_open(&caller, UVT_ROLE_CONTROLLING, UVT_SETUP_ACTPASS));
	CHECK(side_open(&callee, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	CHECK(pair_credentials(&caller, &callee, fingerprint, wrong));
	CHECK(pair_pump(&caller, &callee, both_fail, HANDSHAKE_MS));
	CHECK(!uvt_up(callee.stream));
	CHECK(!uvt_up(caller.stream));

	CASE("fail once");
	caller_bio = rbio_pending(&caller);
	callee_bio = rbio_pending(&callee);
	dtls_inject(&caller, 22);
	dtls_inject(&callee, 22);
	uvt_tick(caller.stream);
	uvt_tick(callee.stream);
	CHECK(count_read(&caller, &caller.dtls_fail) == 1);
	CHECK(count_read(&callee, &callee.dtls_fail) == 1);
	CHECK(rbio_pending(&caller) == caller_bio);
	CHECK(rbio_pending(&callee) == callee_bio);
	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

/*
 * A close_notify ends the media of both sides without a failure.
 */
static void test_close(void)
{
	uint8_t fingerprint[UVT_FINGERPRINT_LEN];
	uint8_t pkt[PKT_MAX];
	struct side caller, callee;
	size_t caller_bio, callee_bio;
	size_t len;

	CASE("close");
	CHECK(uvt_fingerprint_get(fingerprint) == 0);
	CHECK(side_open(&caller, UVT_ROLE_CONTROLLING, UVT_SETUP_ACTPASS));
	CHECK(side_open(&callee, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	CHECK(uvt_close(caller.stream) == -ENOTCONN);
	CHECK(pair_credentials(&caller, &callee, fingerprint, fingerprint));
	CHECK(pair_pump(&caller, &callee, both_up, HANDSHAKE_MS));

	CHECK(uvt_close(caller.stream) == 0);
	CHECK(uvt_close(caller.stream) == -ENOTCONN);
	CHECK(!uvt_up(caller.stream));
	CHECK(pair_pump(&caller, &callee, callee_closed, MEDIA_MS));
	CHECK(!uvt_up(callee.stream));
	CHECK(count_read(&callee, &callee.dtls_fail) == 0);
	CHECK(count_read(&caller, &caller.dtls_fail) == 0);
	CHECK(!flag_read(&caller, &caller.dtls_closed));

	len = rtp_build(pkt, 2);
	CHECK(uvt_send_rtp(caller.stream, pkt, len) == -EPIPE);
	CHECK(uvt_send_rtp(callee.stream, pkt, len) == -EPIPE);
	len = rtcp_rr_build(pkt);
	CHECK(uvt_send_rtcp(callee.stream, pkt, len) == -EPIPE);

	CASE("input after close");
	caller_bio = rbio_pending(&caller);
	callee_bio = rbio_pending(&callee);
	dtls_inject(&caller, 23);
	dtls_inject(&callee, 23);
	CHECK(rbio_pending(&caller) == caller_bio);
	CHECK(rbio_pending(&callee) == callee_bio);
	CHECK(count_read(&callee, &callee.dtls_fail) == 0);
	CHECK(count_read(&caller, &caller.dtls_fail) == 0);

	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

/*
 * A close on one thread while another thread sends RTP.
 */
static void test_close_flow(void)
{
	struct side caller, callee;
	struct flow flow;
	pthread_t thread;
	uint8_t pkt[PKT_MAX];
	bool started;

	CASE("close while media flows");
	CHECK(pair_open(&caller, &callee));
	memset(&flow, 0, sizeof(flow));
	flow.from = &caller;
	flow.to = &callee;
	started = pthread_create(&thread, NULL, flow_run, &flow) == 0;
	CHECK(started);
	CHECK(pair_pump(&caller, &callee, callee_has_flow, MEDIA_MS));
	CHECK(uvt_close(caller.stream) == 0);
	CHECK(pair_pump(&caller, &callee, callee_closed, MEDIA_MS));
	if (started)
		pthread_join(thread, NULL);
	CHECK(flow.last == -EPIPE);
	CHECK(!flow.stalled);
	CHECK(flow.sent >= FLOW_RTP);
	CHECK(count_read(&callee, &callee.rtp_count) <= flow.sent);
	CHECK(count_read(&caller, &caller.dtls_fail) == 0);
	CHECK(count_read(&callee, &callee.dtls_fail) == 0);
	CHECK(uvt_send_rtp(caller.stream, pkt, rtp_build(pkt, 1)) == -EPIPE);
	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

static int inflight_remote_set(struct uvt_stream *stream)
{
	static const uint8_t fingerprint[UVT_FINGERPRINT_LEN];

	return uvt_remote_set(stream, "wxyz", "ABCDEFGHIJKLMNOPQRSTUV",
			      fingerprint, UVT_SETUP_ACTPASS);
}

static int inflight_candidate_add(struct uvt_stream *stream)
{
	return uvt_candidate_add(stream, "a=candidate:2 1 UDP 2130706431 "
				 "192.0.2.10 40002 typ host");
}

static int inflight_local_get(struct uvt_stream *stream)
{
	char ufrag[UVT_UFRAG_MAX], pwd[UVT_PWD_MAX];

	return uvt_local_get(stream, ufrag, pwd);
}

static int inflight_remote_done(struct uvt_stream *stream)
{
	uvt_remote_done(stream);

	return 0;
}

static int inflight_send(struct uvt_stream *stream)
{
	uint8_t pkt[PKT_MAX];

	return uvt_send_rtp(stream, pkt, rtp_build(pkt, 1));
}

/**
 * inflight_run() - run the call of the case
 * @arg: the inflight record
 *
 * Return: NULL.
 */
static void *inflight_run(void *arg)
{
	struct inflight *f = arg;
	int ret;

	ret = f->call(f->stream);
	pthread_mutex_lock(&f->lock);
	f->ret = ret;
	f->returned = true;
	pthread_cond_broadcast(&f->cond);
	pthread_mutex_unlock(&f->lock);

	return NULL;
}

/**
 * destroy_run() - destroy one stream
 * @arg: the stream
 *
 * Return: NULL.
 */
static void *destroy_run(void *arg)
{
	uvt_destroy(arg);

	return NULL;
}

/**
 * inflight_destroy() - destroy a stream while another thread is held inside
 * one call of it
 * @side: the side of the stream, whose stream the destroy takes
 * @pause_at: the hook point that holds the call, with no stream lock held
 * @call: the call
 */
static void inflight_destroy(struct side *side, const char *pause_at,
			     int (*call)(struct uvt_stream *stream))
{
	struct inflight *f = &inflight;
	pthread_t caller, destroyer;
	bool caller_up, destroyer_up = false;

	pthread_mutex_lock(&f->lock);
	f->stream = side->stream;
	f->pause_at = pause_at;
	f->call = call;
	f->paused = false;
	f->released = false;
	f->returned = false;
	f->destroy_waits = false;
	f->destroy_early = false;
	f->ret = -1;
	pthread_mutex_unlock(&f->lock);
	caller_up = pthread_create(&caller, NULL, inflight_run, f) == 0;
	CHECK(caller_up);

	pthread_mutex_lock(&f->lock);
	while (caller_up && !f->paused)
		pthread_cond_wait(&f->cond, &f->lock);
	pthread_mutex_unlock(&f->lock);
	if (caller_up)
		destroyer_up = pthread_create(&destroyer, NULL, destroy_run,
					      side->stream) == 0;
	CHECK(destroyer_up);

	pthread_mutex_lock(&f->lock);
	while (destroyer_up && !f->destroy_waits && !f->destroy_early)
		pthread_cond_wait(&f->cond, &f->lock);
	f->released = true;
	pthread_cond_broadcast(&f->cond);
	pthread_mutex_unlock(&f->lock);
	if (caller_up)
		pthread_join(caller, NULL);
	if (destroyer_up)
		pthread_join(destroyer, NULL);

	pthread_mutex_lock(&f->lock);
	CHECK(f->ret == 0);
	CHECK(f->destroy_waits);
	CHECK(!f->destroy_early);
	f->stream = NULL;
	pthread_mutex_unlock(&f->lock);
	if (destroyer_up)
		side->stream = NULL;
}

/*
 * A destroy on one thread waits for a call that another thread runs.
 *
 * Each case holds the call where it uses the stream with no stream lock
 * held: uvt_remote_set() of a controlled stream at its gathering, the ICE
 * calls at their libjuice entry, and a media sender after it queued its
 * packet and before it owns the drain.
 */
static void test_destroy_inflight(void)
{
	static const struct {
		const char *name;
		enum uvt_role role;
		const char *pause_at;
		int (*call)(struct uvt_stream *stream);
	} cases[] = {
		{ "destroy waits for uvt_remote_set", UVT_ROLE_CONTROLLED,
		  "ice_gather", inflight_remote_set },
		{ "destroy waits for uvt_candidate_add", UVT_ROLE_CONTROLLED,
		  "ice_candidate_add", inflight_candidate_add },
		{ "destroy waits for uvt_local_get", UVT_ROLE_CONTROLLING,
		  "ice_local_get", inflight_local_get },
		{ "destroy waits for uvt_remote_done", UVT_ROLE_CONTROLLED,
		  "ice_remote_done", inflight_remote_done },
	};
	struct side caller, callee, side;
	unsigned int i;

	for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		CASE(cases[i].name);
		CHECK(side_open(&side, cases[i].role, UVT_SETUP_ACTIVE));
		inflight_destroy(&side, cases[i].pause_at, cases[i].call);
		side_close(&side);
		lock_faults_check();
	}

	CASE("destroy waits for a sender between its queue and its drain");
	CHECK(pair_open(&caller, &callee));
	inflight_destroy(&caller, "media_queued", inflight_send);
	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

/*
 * A candidate type that libjuice ignores is no failure.
 */
static void test_candidate_ignored(void)
{
	struct side side;

	CASE("candidate ignored");
	CHECK(side_open(&side, UVT_ROLE_CONTROLLED, UVT_SETUP_ACTIVE));
	CHECK(uvt_candidate_add(side.stream, "a=candidate:1 1 UDP 1862270975 "
				"192.0.2.10 40000 typ prflx raddr 192.0.2.1 "
				"rport 40000") == 0);
	CHECK(uvt_candidate_add(side.stream, "a=candidate:2 1 UDP 2130706431 "
				"192.0.2.10 40002 typ host") == 0);
	CHECK(uvt_candidate_add(side.stream, "a=candidate:3 1 UDP") == -EINVAL);
	side_close(&side);
	lock_faults_check();
}

/*
 * A live stream drops DTLS input that does not fit under the bound of its
 * input BIO, and stays up.
 */
static void test_input_bound(void)
{
	static const char fill[UVT_RBIO_MAX];
	struct side caller, callee;
	size_t pending, want = UVT_RBIO_MAX - INJECT_LEN + 1;

	CASE("input bound of a live stream");
	CHECK(pair_open(&caller, &callee));
	stream_lock(caller.stream);
	pending = BIO_ctrl_pending(caller.stream->rbio);
	if (pending < want)
		BIO_write(caller.stream->rbio, fill, (int)(want - pending));
	stream_unlock(caller.stream);
	CHECK(rbio_pending(&caller) == want);
	dtls_inject(&caller, 23);
	CHECK(rbio_pending(&caller) == want);
	CHECK(uvt_up(caller.stream));
	CHECK(count_read(&caller, &caller.dtls_fail) == 0);
	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

/**
 * send_run() - send one RTP packet
 * @arg: the flow, whose last receives the result
 *
 * Return: NULL.
 */
static void *send_run(void *arg)
{
	struct flow *flow = arg;
	uint8_t pkt[PKT_MAX];

	flow->last = uvt_send_rtp(flow->from->stream, pkt, rtp_build(pkt, 1));

	return NULL;
}

/*
 * The output queue behind a sender that libjuice holds.
 *
 * Media fills the queue up to UVT_QUEUE_RESERVE free slots and then counts
 * each drop. A close_notify still finds a slot. The held sender then sends
 * the whole queue in its order.
 */
static void test_queue_held(void)
{
	const unsigned int media = UVT_QUEUE_LEN - UVT_QUEUE_RESERVE;
	struct side caller, callee;
	unsigned int queued = 0, refused = 0, i;
	struct flow flow = { .from = &caller, .to = &callee };
	uint8_t pkt[PKT_MAX];
	pthread_t thread;
	bool started;
	int ret;

	CASE("queue behind a held sender");
	CHECK(pair_open(&caller, &callee));
	pthread_mutex_lock(&caller.lock);
	caller.sent_count = 0;
	caller.pause_armed = true;
	caller.pause_hold = true;
	pthread_mutex_unlock(&caller.lock);
	started = pthread_create(&thread, NULL, send_run, &flow) == 0;
	CHECK(started);
	pthread_mutex_lock(&caller.lock);
	while (started && !caller.paused)
		pthread_cond_wait(&caller.cond, &caller.lock);
	pthread_mutex_unlock(&caller.lock);

	for (i = 2; i <= media + 2; i++) {
		ret = uvt_send_rtp(caller.stream, pkt, rtp_build(pkt, i));
		queued += ret == 0;
		refused += ret == -ENOBUFS;
	}
	CHECK(queued == media - 1);
	CHECK(refused == 2);
	CHECK(uvt_close(caller.stream) == 0);
	stream_lock(caller.stream);
	CHECK(caller.stream->media_dropped == 2);
	CHECK(caller.stream->queue_count == media + 1);
	CHECK(caller.stream->queue[(caller.stream->queue_head + media) %
				   UVT_QUEUE_LEN].data[0] == DTLS_ALERT);
	stream_unlock(caller.stream);

	pthread_mutex_lock(&caller.lock);
	caller.pause_hold = false;
	pthread_cond_broadcast(&caller.cond);
	pthread_mutex_unlock(&caller.lock);
	if (started)
		pthread_join(thread, NULL);
	CHECK(flow.last == 0);
	CHECK(pair_pump(&caller, &callee, callee_closed, MEDIA_MS));

	pthread_mutex_lock(&caller.lock);
	CHECK(caller.sent_count == media + 1);
	for (i = 0; i < media && i < caller.sent_count; i++)
		CHECK(caller.sent[i] == i + 1);
	CHECK(caller.sent[media] == (SENT_DTLS | DTLS_ALERT));
	pthread_mutex_unlock(&caller.lock);
	CHECK(count_read(&callee, &callee.rtp_count) == media);
	side_close(&caller);
	side_close(&callee);
	lock_faults_check();
}

int main(void)
{
	alarm(SUITE_S);
	if (pipe2(wake_fd, O_NONBLOCK | O_CLOEXEC)) {
		perror("pipe2");
		return 1;
	}
	hook_send = test_send;
	hook_entry = test_entry;
	hook_dtls_timer = test_dtls_timer;

	test_local();
	test_credentials();
	test_remote_twice();
	test_timer_default();
	test_handshake_and_media();
	test_lost_record();
	test_fingerprint_mismatch();
	test_close();
	test_close_flow();
	test_destroy_inflight();
	test_candidate_ignored();
	test_input_bound();
	test_queue_held();

	close(wake_fd[0]);
	close(wake_fd[1]);
	printf("%u checks, %u failed\n", tests_run, tests_failed);

	return tests_failed ? 1 : 0;
}
