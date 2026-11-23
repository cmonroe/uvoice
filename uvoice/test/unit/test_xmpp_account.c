// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 John Crispin <john@phrozen.org>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * Unit tests for the XMPP account layer (src/proto/xmpp/account.c): the
 * STUN and TURN relays, their expiry and their generations, the iq relay of
 * the legs and the environment the layer gives the legs.
 *
 * No socket and no daemon. The suite defines the xmpp_stream_ functions, so
 * the stream is a recorder that a case moves to ready and whose iq answers
 * the case scripts. The linker wraps the resolver entries, the leg layer
 * init, the timer arm and the timer environment (CMakeLists.txt): a case
 * reads each question with its owner, answers it through the done callback
 * of the layer, reads the environment of the legs and the delay of each
 * timer. The timers run on the virtual clock of test_clock.h, which a case
 * moves by hand.
 */
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <libubox/uloop.h>

#include "proto/shared/resolve.h"
#include "proto/shared/timer.h"
#include "proto/xmpp/account.h"
#include "proto/xmpp/call.h"
#include "proto/xmpp/stream.h"
#include "proto/xmpp/xml.h"
#include "proto_host_fake.h"
#include "test_util.h"

#define STREAM_OPEN							\
	"<stream:stream xmlns='jabber:client' "				\
	"xmlns:stream='http://etherx.jabber.org/streams'>"
#define SELF_JID	"phone@sip.example.test"
#define LEG_IQ		"<iq type='set' id='j1'/>"
#define SERVICE_STUN(host)						\
	"<service type='stun' host='" host "' port='3478' transport='udp'/>"
#define IQS_MAX		32
#define QUERIES_MAX	32
#define ARMS_MAX	64
#define TEXT_MAX	2048
#define TIMERS_MAX	16
/* A timer clock that does not start at 0, which the layer reads as no
 * expiry. */
#define CLOCK_START_MS	1000000ULL
#define EXPIRY_MS	600000LL
#define DAY_MS		86400000LL

static struct {
	uint64_t now;
	struct proto_timer *timer[TIMERS_MAX];
	unsigned int timers;
	unsigned int fired;
} fk;

#include "test_clock.h"

struct iq_rec {
	char text[TEXT_MAX];
	xmpp_iq_cb cb;
	void *priv;
	bool answered;
};

struct query_rec {
	char name[SIP_DNS_NAME_MAX];
	uint32_t owner;
	uint32_t query;
};

static struct {
	struct xmpp_stream *xs;
	bool ready;
	bool iq_refuse;
	unsigned int ids;
	unsigned int sends;
	unsigned int dns_answers;
	unsigned int iq_attempts;
	struct iq_rec iq[IQS_MAX];
	unsigned int iqs;
} fs;

static struct {
	struct sip_dns *dns;
	struct query_rec q[QUERIES_MAX];
	unsigned int n;
	uint32_t next;
	uint32_t cancelled[QUERIES_MAX];
	unsigned int cancels;
} fr;

static struct {
	unsigned int ms[ARMS_MAX];
	unsigned int n;
} arms;

static struct xcall_env legs_env;
static struct cfg cfg;
static struct xml_stream parser;
static xmpp_iq_cb reply_cb;
static void *reply_priv;
static unsigned int dnd_asks;
static unsigned int held_line;
static const struct call_leg_ops *held_legs;
static unsigned int leg_replies;
static bool leg_reply_ok;

/* ------------------------------------------------------------ the stream */

void xmpp_stream_init(struct xmpp_stream *xs, const struct xmpp_stream_cb *cb,
		      void *priv, const struct xmpp_stream_env *env)
{
	memset(xs, 0, sizeof(*xs));
	xs->cb = *cb;
	xs->priv = priv;
	xs->env = *env;
	fs.xs = xs;
}

int xmpp_stream_start(struct xmpp_stream *xs,
		      const struct xmpp_stream_cfg *scfg)
{
	xs->cfg = *scfg;

	return jid_parse(scfg->jid, &xs->jid) ? 0 : -EINVAL;
}

/**
 * iqs_flush() - answer every waiting iq with nothing, as the stream does when
 * its connection ends
 * @xs: the stream
 */
static void iqs_flush(struct xmpp_stream *xs)
{
	unsigned int i;

	for (i = 0; i < fs.iqs; i++) {
		if (fs.iq[i].answered)
			continue;
		fs.iq[i].answered = true;
		fs.iq[i].cb(xs, NULL, fs.iq[i].priv);
	}
}

void xmpp_stream_done(struct xmpp_stream *xs)
{
	iqs_flush(xs);
	if (fs.xs == xs)
		fs.xs = NULL;
}

void xmpp_stream_dns_done(struct xmpp_stream *xs, uint32_t query,
			  const struct sip_dns_result *res)
{
	fs.dns_answers++;
}

bool xmpp_stream_ready(const struct xmpp_stream *xs)
{
	return fs.ready;
}

const struct jid *xmpp_stream_jid(const struct xmpp_stream *xs)
{
	return &xs->bound;
}

const char *xmpp_stream_reason(const struct xmpp_stream *xs)
{
	return "";
}

const char *xmpp_stream_state_name(enum xmpp_stream_state state)
{
	return "";
}

void xmpp_stream_id_next(struct xmpp_stream *xs, char *out, size_t len)
{
	snprintf(out, len, "q%u", ++fs.ids);
}

int xmpp_stream_send(struct xmpp_stream *xs, const char *text, size_t len)
{
	fs.sends++;

	return 0;
}

int xmpp_stream_iq_send(struct xmpp_stream *xs, const char *text, size_t len,
			const char *id, const char *peer, xmpp_iq_cb cb,
			void *priv)
{
	struct iq_rec *rec;

	fs.iq_attempts++;
	if (fs.iq_refuse)
		return -EBUSY;
	if (fs.iqs >= IQS_MAX)
		return -ENOBUFS;

	rec = &fs.iq[fs.iqs++];
	snprintf(rec->text, sizeof(rec->text), "%.*s", (int)len, text);
	rec->cb = cb;
	rec->priv = priv;

	return 0;
}

int xmpp_stream_iq_result(struct xmpp_stream *xs, const struct xml_doc *iq)
{
	return 0;
}

int xmpp_stream_iq_error(struct xmpp_stream *xs, const struct xml_doc *iq,
			 const char *type, const char *condition)
{
	return 0;
}

const struct xmpp_stream_stat *xmpp_stream_stat(const struct xmpp_stream *xs)
{
	return &xs->stat;
}

/* ------------------------------------------------------ the wrapped entries */

uint32_t __wrap_sip_dns_resolve(struct sip_dns *r, const char *name,
				enum sip_dns_kind kind, uint16_t port,
				uint32_t owner);
void __wrap_sip_dns_cancel(struct sip_dns *r, uint32_t query);
void __real_xcalls_init(const struct xcall_env *env);
void __wrap_xcalls_init(const struct xcall_env *env);
void __real_proto_timer_arm(struct proto_timer_env *env,
			    struct proto_timer *timer, unsigned int ms);
void __wrap_proto_timer_arm(struct proto_timer_env *env,
			    struct proto_timer *timer, unsigned int ms);
void __wrap_proto_timer_env_uloop_init(struct proto_timer_env *env);

/* Record a question with its owner; the case answers it through dns_answer().
 */
uint32_t __wrap_sip_dns_resolve(struct sip_dns *r, const char *name,
				enum sip_dns_kind kind, uint16_t port,
				uint32_t owner)
{
	struct query_rec *rec;

	fr.dns = r;
	if (fr.n >= QUERIES_MAX)
		return SIP_HANDLE_NONE;

	rec = &fr.q[fr.n++];
	snprintf(rec->name, sizeof(rec->name), "%s", name);
	rec->owner = owner;
	rec->query = ++fr.next;

	return rec->query;
}

void __wrap_sip_dns_cancel(struct sip_dns *r, uint32_t query)
{
	if (fr.cancels < QUERIES_MAX)
		fr.cancelled[fr.cancels++] = query;
}

/* Keep the environment the layer gives the legs. */
void __wrap_xcalls_init(const struct xcall_env *env)
{
	legs_env = *env;
	__real_xcalls_init(env);
}

/* Record the delay of every timer the layer arms. */
void __wrap_proto_timer_arm(struct proto_timer_env *env,
			    struct proto_timer *timer, unsigned int ms)
{
	if (arms.n < ARMS_MAX)
		arms.ms[arms.n++] = ms;
	__real_proto_timer_arm(env, timer, ms);
}

/* Run the timers of the layer on the virtual clock. */
void __wrap_proto_timer_env_uloop_init(struct proto_timer_env *env)
{
	env->now_ms = fake_now;
	env->timer_set = fake_timer_set;
	env->timer_cancel = fake_timer_cancel;
}

/* --------------------------------------------------------------- helpers */

/**
 * on_stanza() - hand a parsed reply to the iq callback the case picked
 * @user: unused
 * @doc: the tree
 */
static void on_stanza(void *user, const struct xml_doc *doc)
{
	if (reply_cb)
		reply_cb(fs.xs, doc, reply_priv);
}

static const struct xml_stream_cb parser_cb = {
	.stanza = on_stanza,
};

/**
 * reply_feed() - answer a recorded iq with a parsed stanza
 * @rec: the iq
 * @xml: the answer
 */
static void reply_feed(struct iq_rec *rec, const char *xml)
{
	rec->answered = true;
	reply_cb = rec->cb;
	reply_priv = rec->priv;
	xml_stream_reset(&parser);
	xml_stream_feed(&parser, STREAM_OPEN, strlen(STREAM_OPEN));
	xml_stream_feed(&parser, xml, strlen(xml));
	reply_cb = NULL;
}

/**
 * services_iq() - the last discovery request of the layer
 *
 * Return: the record, or NULL.
 */
static struct iq_rec *services_iq(void)
{
	unsigned int i;

	for (i = fs.iqs; i > 0; i--) {
		if (strstr(fs.iq[i - 1].text, "urn:xmpp:extdisco:2"))
			return &fs.iq[i - 1];
	}

	return NULL;
}

/**
 * services_count() - the discovery requests the layer sent
 *
 * Return: the count.
 */
static unsigned int services_count(void)
{
	unsigned int i, n = 0;

	for (i = 0; i < fs.iqs; i++) {
		if (strstr(fs.iq[i].text, "urn:xmpp:extdisco:2"))
			n++;
	}

	return n;
}

/**
 * services_waiting() - the discovery requests that wait for an answer
 *
 * Return: the count.
 */
static unsigned int services_waiting(void)
{
	unsigned int i, n = 0;

	for (i = 0; i < fs.iqs; i++) {
		if (!fs.iq[i].answered &&
		    strstr(fs.iq[i].text, "urn:xmpp:extdisco:2"))
			n++;
	}

	return n;
}

/**
 * iq_lost() - the stream gives up on an iq, as after its timeout
 * @rec: the iq
 */
static void iq_lost(struct iq_rec *rec)
{
	rec->answered = true;
	rec->cb(fs.xs, NULL, rec->priv);
}

/**
 * stream_reset() - the connection ends: the stream answers its iqs with
 * nothing, then reports the backoff
 */
static void stream_reset(void)
{
	fs.ready = false;
	iqs_flush(fs.xs);
	fs.xs->cb.state(fs.xs, XMPP_ST_BACKOFF, "reset");
}

/**
 * stream_up() - the stream is ready again
 */
static void stream_up(void)
{
	fs.ready = true;
	fs.xs->cb.state(fs.xs, XMPP_ST_READY, "");
}

/**
 * expires_text() - an XEP-0082 DateTime at a distance from the wall clock
 * @delta_ms: the distance, negative for the past
 * @out: receives the text
 * @len: capacity of @out
 */
static void expires_text(int64_t delta_ms, char *out, size_t len)
{
	struct timespec wall;
	int64_t at;
	time_t secs;
	struct tm tm;
	char base[32];

	clock_gettime(CLOCK_REALTIME, &wall);
	at = (int64_t)wall.tv_sec * 1000 + wall.tv_nsec / 1000000 + delta_ms;
	secs = (time_t)(at / 1000);
	gmtime_r(&secs, &tm);
	strftime(base, sizeof(base), "%Y-%m-%dT%H:%M:%S", &tm);
	snprintf(out, len, "%s.%03dZ", base, (int)(at % 1000));
}

/**
 * services_answer() - answer the last discovery request with one TURN row
 * @rec: the request
 * @host: the host of the relay
 * @port: its port attribute, NULL for none
 * @user: its username
 * @expires: its expires attribute, NULL for none
 */
static void services_answer(struct iq_rec *rec, const char *host,
			    const char *port, const char *user,
			    const char *expires)
{
	char port_attr[32] = "", expires_attr[64] = "";
	char xml[1024];

	if (port)
		snprintf(port_attr, sizeof(port_attr), " port='%s'", port);
	if (expires)
		snprintf(expires_attr, sizeof(expires_attr), " expires='%s'",
			 expires);
	snprintf(xml, sizeof(xml), "<iq type='result' id='x' "
		 "from='sip.example.test'><services xmlns='urn:xmpp:extdisco:2'>"
		 "<service type='turn' host='%s'%s transport='udp' "
		 "username='%s' password='pw'%s/></services></iq>", host,
		 port_attr, user, expires_attr);
	reply_feed(rec, xml);
}

/**
 * query_find() - the last question about a name
 * @name: the name
 *
 * Return: the record, or NULL.
 */
static const struct query_rec *query_find(const char *name)
{
	unsigned int i;

	for (i = fr.n; i > 0; i--) {
		if (!strcmp(fr.q[i - 1].name, name))
			return &fr.q[i - 1];
	}

	return NULL;
}

/**
 * dns_answer() - the resolver answers a question with one address
 * @rec: the question
 * @addr: the address
 */
static void dns_answer(const struct query_rec *rec, const char *addr)
{
	struct sip_dns_result res;

	memset(&res, 0, sizeof(res));
	res.status = SIP_DNS_OK;
	res.count = 1;
	snprintf(res.target[0].host, sizeof(res.target[0].host), "%s", addr);
	fr.dns->cb.done(fr.dns, rec->query, rec->owner, &res, fr.dns->priv);
}

/**
 * cancelled() - whether the layer cancelled a question
 * @query: the handle
 *
 * Return: true when it did.
 */
static bool cancelled(uint32_t query)
{
	unsigned int i;

	for (i = 0; i < fr.cancels; i++) {
		if (fr.cancelled[i] == query)
			return true;
	}

	return false;
}

/**
 * turn_of() - the TURN rows a new call of line 0 would take
 * @out: receives the transport block
 *
 * Return: the number of rows.
 */
static unsigned int turn_of(struct hal_transport *out)
{
	memset(out, 0, sizeof(*out));
	legs_env.ice_servers(legs_env.priv, 0, out);

	return out->turns;
}

/**
 * turn_resolved() - resolve the relay a discovery answer named, then count
 * the TURN rows a new call would take
 * @host: the host of the relay
 * @addr: the address the resolver answers
 *
 * Return: the number of rows.
 */
static unsigned int turn_resolved(const char *host, const char *addr)
{
	const struct query_rec *q = query_find(host);
	struct hal_transport tp;

	if (q && !cancelled(q->query))
		dns_answer(q, addr);

	return turn_of(&tp);
}

/**
 * shell_dnd() - the do-not-disturb answer of the fake shell
 * @line: the line
 *
 * Return: true for line 0.
 */
static bool shell_dnd(unsigned int line)
{
	dnd_asks++;

	return line == 0;
}

/**
 * shell_legs_held() - the leg table answer of the fake shell
 * @line: the line
 * @legs: the table the legs ask about
 *
 * Return: true for line 0.
 */
static bool shell_legs_held(unsigned int line, const struct call_leg_ops *legs)
{
	held_line = line;
	held_legs = legs;

	return line == 0;
}

/**
 * leg_reply() - the reply callback a leg gave
 * @priv: unused
 * @ok: whether a result came
 * @reply: unused
 */
static void leg_reply(void *priv, bool ok, const struct xml_doc *reply)
{
	leg_replies++;
	leg_reply_ok = ok;
}

/* The legs report to the shell; no case here places a call. */
static const struct call_cb shell_cb = { 0 };

/**
 * layer_start() - a fresh layer with one xmpp line, its stream ready
 * @turn: the TURN relay the channel names, NULL for none
 */
static void layer_start(const char *turn)
{
	memset(&fs, 0, sizeof(fs));
	memset(&fr, 0, sizeof(fr));
	memset(&arms, 0, sizeof(arms));
	memset(&fk, 0, sizeof(fk));
	fk.now = CLOCK_START_MS;
	proto_host_fake_reset();

	memset(&cfg, 0, sizeof(cfg));
	cfg.enabled = true;
	cfg.channels = 1;
	cfg.channel[0].present = true;
	cfg.channel[0].enabled = true;
	cfg.channel[0].account_type = CFG_ACCOUNT_XMPP;
	snprintf(cfg.channel[0].xmpp_jid, sizeof(cfg.channel[0].xmpp_jid),
		 SELF_JID);
	snprintf(cfg.channel[0].xmpp_password,
		 sizeof(cfg.channel[0].xmpp_password), "pencil");
	cfg.channel[0].xmpp_tls_verify = true;
	if (turn) {
		snprintf(cfg.channel[0].xmpp_turn,
			 sizeof(cfg.channel[0].xmpp_turn), "%s", turn);
		snprintf(cfg.channel[0].xmpp_turn_user,
			 sizeof(cfg.channel[0].xmpp_turn_user), "cfguser");
		snprintf(cfg.channel[0].xmpp_turn_password,
			 sizeof(cfg.channel[0].xmpp_turn_password), "cfgpass");
	}

	xmpp_accounts_init(proto_host_fake(), &shell_cb, shell_dnd,
			   shell_legs_held);
	xmpp_accounts_start(&cfg);
	fs.ready = true;
	if (fs.xs)
		fs.xs->cb.state(fs.xs, XMPP_ST_READY, "");
}

/**
 * refresh_waiting() - a layer whose refresh query waits for an answer
 *
 * Return: the refresh request, or NULL.
 */
static struct iq_rec *refresh_waiting(void)
{
	struct iq_rec *rec;
	char when[64];

	layer_start(NULL);
	rec = services_iq();
	if (!rec)
		return NULL;

	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u1", when);
	fake_advance(EXPIRY_MS - XMPP_RELAY_LEAD_MS);
	rec = services_iq();
	if (!rec || rec->answered)
		return NULL;

	return rec;
}

/* ----------------------------------------------------------------- cases */

/*
 * The relays of the server, their credentials and expiry.
 */
static void test_relays(void)
{
	struct hal_transport tp;
	struct iq_rec *rec;
	char when[64];
	unsigned int resolves;

	layer_start(NULL);

	CASE("the ready stream asks the server for its relays");
	CHECK(xmpp_account_active(0));
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;

	CASE("a TURN relay of the answer resolves and goes into a call");
	services_answer(rec, "turn.example.net", "3478", "u1", NULL);
	CHECK(query_find("turn.example.net") != NULL);
	if (!query_find("turn.example.net"))
		return;
	CHECK_UINT(turn_of(&tp), 0);
	dns_answer(query_find("turn.example.net"), "192.0.2.50");
	CHECK_UINT(turn_of(&tp), 1);
	CHECK(!strcmp(tp.turn[0].host, "192.0.2.50"));
	CHECK(!strcmp(tp.turn[0].user, "u1"));
	CHECK_UINT(tp.turn[0].port, 3478);

	CASE("a second answer for the same relay replaces its credentials");
	resolves = fr.n;
	services_answer(rec, "turn.example.net", "3478", "u2", NULL);
	CHECK_UINT(fr.n, resolves);
	CHECK_UINT(turn_of(&tp), 1);
	CHECK(!strcmp(tp.turn[0].user, "u2"));

	CASE("a relay without a port is the relay on 3478");
	services_answer(rec, "turn.example.net", NULL, "u3", NULL);
	CHECK_UINT(fr.n, resolves);
	CHECK_UINT(turn_of(&tp), 1);
	CHECK(!strcmp(tp.turn[0].user, "u3"));

	CASE("expired credentials go into no new call");
	expires_text(-10000, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u5", when);
	CHECK_UINT(turn_of(&tp), 0);

	CASE("credentials that expire later go into a call");
	expires_text(600000, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u6", when);
	CHECK_UINT(turn_of(&tp), 1);
	CHECK(!strcmp(tp.turn[0].user, "u6"));

	CASE("the refresh runs one minute before the expiry by default");
	CHECK_UINT(XMPP_RELAY_LEAD_MS, 60000);
	CHECK(arms.n >= 1);
	CHECK(arms.n && arms.ms[arms.n - 1] > 539000 &&
	      arms.ms[arms.n - 1] <= 540000);

	CASE("an expiry in a time zone offset is the same instant");
	expires_text(600000, when, sizeof(when));
	CHECK(strlen(when) > 1);
	when[strlen(when) - 1] = 0;
	strncat(when, "+00:00", sizeof(when) - strlen(when) - 1);
	services_answer(rec, "turn.example.net", "3478", "u7", when);
	CHECK(arms.n && arms.ms[arms.n - 1] > 539000 &&
	      arms.ms[arms.n - 1] <= 540000);

	CASE("a relay on another port is another relay");
	services_answer(rec, "turn.example.net", "5349", "u4", NULL);
	CHECK_UINT(fr.n, resolves + 1);

	xmpp_accounts_done();
}

/*
 * The layer asks again before the credentials expire.
 */
static void test_refresh(void)
{
	struct iq_rec *rec;
	char when[64];
	unsigned int asked;

	layer_start(NULL);
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;

	CASE("the refresh asks the server again one lead before the expiry");
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u1", when);
	asked = services_count();
	fake_advance(EXPIRY_MS - XMPP_RELAY_LEAD_MS - 1000);
	CHECK_UINT(services_count(), asked);
	fake_advance(1000);
	CHECK_UINT(services_count(), asked + 1);

	CASE("a teardown cancels the refresh");
	rec = services_iq();
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u2", when);
	CHECK_UINT(fk.timers, 1);
	asked = services_count();
	cfg.channel[0].enabled = false;
	xmpp_accounts_reload(&cfg, &cfg);
	CHECK(!xmpp_account_active(0));
	CHECK_UINT(fk.timers, 0);
	fake_advance(EXPIRY_MS);
	CHECK_UINT(services_count(), asked);

	xmpp_accounts_done();

	CASE("an expiry past the reach of one timer arms it in steps of at most INT_MAX ms");
	layer_start(NULL);
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;
	expires_text(40 * DAY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u3", when);
	CHECK(arms.n && arms.ms[arms.n - 1] <= INT_MAX);
	asked = services_count();
	fake_advance(INT_MAX);
	CHECK_UINT(services_count(), asked);
	CHECK_UINT(fk.timers, 1);
	CHECK(arms.n && arms.ms[arms.n - 1] <= INT_MAX);

	CASE("the stepped refresh asks once, one lead before the expiry");
	fake_advance(40 * DAY_MS - XMPP_RELAY_LEAD_MS - INT_MAX - 1000);
	CHECK_UINT(services_count(), asked);
	fake_advance(2000);
	CHECK_UINT(services_count(), asked + 1);

	xmpp_accounts_done();
}

/*
 * A refresh that fails asks again after a backoff, a bounded number of times.
 */
static void test_refresh_retry(void)
{
	struct iq_rec *rec;
	char when[64];
	unsigned int asked, tries;

	CASE("the first retry of a refresh waits 5 s, and 4 retries are the bound");
	CHECK_UINT(XMPP_RELAY_RETRY_MS, 5000);
	CHECK_UINT(XMPP_RELAY_RETRIES, 4);

	layer_start(NULL);
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u1", when);
	fake_advance(EXPIRY_MS - XMPP_RELAY_LEAD_MS);
	rec = services_iq();
	asked = services_count();

	CASE("a refresh the server refuses asks again after XMPP_RELAY_RETRY_MS");
	reply_feed(rec, "<iq type='error' id='x' from='sip.example.test'>"
		   "<error type='cancel'><service-unavailable "
		   "xmlns='urn:ietf:params:xml:ns:xmpp-stanzas'/></error></iq>");
	fake_advance(XMPP_RELAY_RETRY_MS - 1);
	CHECK_UINT(services_count(), asked);
	fake_advance(1);
	CHECK_UINT(services_count(), asked + 1);

	CASE("a refresh without an answer asks again after twice the wait");
	rec = services_iq();
	iq_lost(rec);
	fake_advance(2 * XMPP_RELAY_RETRY_MS - 1);
	CHECK_UINT(services_count(), asked + 1);
	fake_advance(1);
	CHECK_UINT(services_count(), asked + 2);

	CASE("a refresh the stream does not send asks again, up to the bound");
	rec = services_iq();
	iq_lost(rec);
	fs.iq_refuse = true;
	tries = fs.iq_attempts;
	fake_advance(4 * XMPP_RELAY_RETRY_MS);
	CHECK_UINT(fs.iq_attempts, tries + 1);
	fake_advance(8 * XMPP_RELAY_RETRY_MS);
	CHECK_UINT(fs.iq_attempts, tries + 2);

	CASE("after the bound no timer is left and the layer asks no more");
	CHECK_UINT(fk.timers, 0);
	fake_advance(DAY_MS);
	CHECK_UINT(fs.iq_attempts, tries + 2);

	CASE("a refresh answer after a failure ends the retries");
	xmpp_accounts_done();
	layer_start(NULL);
	rec = services_iq();
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u1", when);
	fake_advance(EXPIRY_MS - XMPP_RELAY_LEAD_MS);
	rec = services_iq();
	iq_lost(rec);
	fake_advance(XMPP_RELAY_RETRY_MS);
	rec = services_iq();
	asked = services_count();
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u2", when);
	fake_advance(EXPIRY_MS - XMPP_RELAY_LEAD_MS - 1000);
	CHECK_UINT(services_count(), asked);
	fake_advance(1000);
	CHECK_UINT(services_count(), asked + 1);

	CASE("a refresh result without a services element asks again after "
	     "XMPP_RELAY_RETRY_MS");
	rec = services_iq();
	asked = services_count();
	reply_feed(rec, "<iq type='result' id='x' from='sip.example.test'/>");
	CHECK_UINT(turn_resolved("turn.example.net", "192.0.2.50"), 1);
	fake_advance(XMPP_RELAY_RETRY_MS - 1);
	CHECK_UINT(services_count(), asked);
	fake_advance(1);
	CHECK_UINT(services_count(), asked + 1);

	xmpp_accounts_done();
}

/*
 * A discovery answer replaces the relays an earlier answer named.
 */
static void test_relays_replaced(void)
{
	const struct query_rec *q;
	struct hal_transport tp;
	struct iq_rec *rec;
	char when[64];

	layer_start(NULL);
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;

	CASE("a relay that a newer answer does not name leaves every new call");
	services_answer(rec, "turn.example.net", "3478", "u1", NULL);
	CHECK_UINT(turn_resolved("turn.example.net", "192.0.2.50"), 1);
	services_answer(rec, "turn2.example.net", "3478", "u2", NULL);
	CHECK_UINT(turn_of(&tp), 0);
	CHECK_UINT(turn_resolved("turn2.example.net", "192.0.2.51"), 1);
	CHECK(turn_of(&tp) && !strcmp(tp.turn[0].host, "192.0.2.51"));

	CASE("a relay dropped before it resolved loses its resolver question");
	services_answer(rec, "turn3.example.net", "3478", "u3", NULL);
	q = query_find("turn3.example.net");
	CHECK(q != NULL);
	services_answer(rec, "turn4.example.net", "3478", "u4", NULL);
	CHECK(q && cancelled(q->query));
	CHECK_UINT(turn_resolved("turn4.example.net", "192.0.2.54"), 1);
	CHECK(turn_of(&tp) && !strcmp(tp.turn[0].host, "192.0.2.54"));

	xmpp_accounts_done();

	CASE("an answer without the relay the channel names keeps that relay");
	layer_start("relay.example.org");
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;
	CHECK_UINT(turn_resolved("relay.example.org", "198.51.100.7"), 1);
	services_answer(rec, "turn.example.net", "3478", "u1", NULL);
	CHECK_UINT(turn_resolved("turn.example.net", "192.0.2.50"), 2);
	reply_feed(rec, "<iq type='result' id='x' from='sip.example.test'>"
		   "<services xmlns='urn:xmpp:extdisco:2'/></iq>");
	CHECK_UINT(turn_of(&tp), 1);
	CHECK(!strcmp(tp.turn[0].host, "198.51.100.7"));

	CASE("an answer that withdraws the relay the channel names restores "
	     "its configured credentials");
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "relay.example.org", "3478", "u5", when);
	CHECK(turn_of(&tp) == 1 && !strcmp(tp.turn[0].user, "u5"));
	reply_feed(rec, "<iq type='result' id='x' from='sip.example.test'>"
		   "<services xmlns='urn:xmpp:extdisco:2'/></iq>");
	CHECK(turn_of(&tp) == 1 && !strcmp(tp.turn[0].user, "cfguser") &&
	      !strcmp(tp.turn[0].pass, "cfgpass"));
	fake_advance(2 * EXPIRY_MS);
	CHECK_UINT(turn_of(&tp), 1);

	CASE("a result without a services element keeps the relays");
	services_answer(rec, "turn.example.net", "3478", "u1", NULL);
	CHECK_UINT(turn_resolved("turn.example.net", "192.0.2.50"), 2);
	reply_feed(rec, "<iq type='result' id='x' from='sip.example.test'/>");
	CHECK_UINT(turn_of(&tp), 2);

	xmpp_accounts_done();

	CASE("duplicate rows of one relay leave room for the next relay");
	layer_start(NULL);
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;
	reply_feed(rec, "<iq type='result' id='x' from='sip.example.test'>"
		   "<services xmlns='urn:xmpp:extdisco:2'>"
		   SERVICE_STUN("stun.example.net")
		   SERVICE_STUN("stun.example.net")
		   SERVICE_STUN("stun.example.net")
		   SERVICE_STUN("stun.example.net")
		   "<service type='turn' host='turn.example.net' "
		   "transport='udp' username='u1' password='pw'/>"
		   "</services></iq>");
	CHECK_UINT(turn_resolved("turn.example.net", "192.0.2.50"), 1);

	CASE("the relay limit counts distinct relays");
	reply_feed(rec, "<iq type='result' id='x' from='sip.example.test'>"
		   "<services xmlns='urn:xmpp:extdisco:2'>"
		   SERVICE_STUN("s1.example.net") SERVICE_STUN("s2.example.net")
		   SERVICE_STUN("s3.example.net") SERVICE_STUN("s4.example.net")
		   SERVICE_STUN("s5.example.net") "</services></iq>");
	CHECK(query_find("s4.example.net") != NULL);
	CHECK(query_find("s5.example.net") == NULL);

	xmpp_accounts_done();
}

/*
 * A teardown or a reload with a refresh that waits for its answer.
 */
static void test_refresh_teardown(void)
{
	static struct cfg moved;
	struct iq_rec *rec;
	unsigned int asked;

	CASE("a teardown with a refresh in flight leaves no relay timer");
	rec = refresh_waiting();
	CHECK(rec != NULL);
	if (!rec)
		return;
	cfg.channel[0].enabled = false;
	xmpp_accounts_reload(&cfg, &cfg);
	CHECK(!xmpp_account_active(0));
	CHECK(rec->answered);
	CHECK_UINT(fk.timers, 0);
	xmpp_accounts_done();

	CASE("a reload that rebuilds the stream with a refresh in flight "
	     "leaves no relay timer");
	rec = refresh_waiting();
	CHECK(rec != NULL);
	if (!rec)
		return;
	moved = cfg;
	snprintf(moved.channel[0].xmpp_host, sizeof(moved.channel[0].xmpp_host),
		 "xmpp2.example.test");
	xmpp_accounts_reload(&cfg, &moved);
	CHECK(xmpp_account_active(0));
	CHECK(rec->answered);
	CHECK_UINT(fk.timers, 0);
	xmpp_accounts_done();

	CASE("a reload that keeps the stream ignores the lost refresh of the "
	     "earlier configuration");
	rec = refresh_waiting();
	CHECK(rec != NULL);
	if (!rec)
		return;
	moved = cfg;
	snprintf(moved.channel[0].xmpp_stun, sizeof(moved.channel[0].xmpp_stun),
		 "stun.example.net");
	xmpp_accounts_reload(&cfg, &moved);
	asked = services_count();
	iq_lost(rec);
	CHECK_UINT(fk.timers, 0);
	fake_advance(DAY_MS);
	CHECK_UINT(services_count(), asked);
	xmpp_accounts_done();

	CASE("the end of the layer with a refresh in flight leaves no relay "
	     "timer");
	rec = refresh_waiting();
	CHECK(rec != NULL);
	if (!rec)
		return;
	xmpp_accounts_done();
	CHECK(rec->answered);
	CHECK_UINT(fk.timers, 0);
}

/*
 * A connection that ends and comes back.
 */
static void test_reconnect(void)
{
	struct iq_rec *rec;
	char when[64];

	CASE("a disconnect during a refresh leaves no relay timer");
	rec = refresh_waiting();
	CHECK(rec != NULL);
	if (!rec)
		return;
	stream_reset();
	CHECK(rec->answered);
	CHECK_UINT(fk.timers, 0);

	CASE("a new connection has one discovery query in flight");
	stream_up();
	CHECK_UINT(services_waiting(), 1);
	fake_advance(32 * XMPP_RELAY_RETRY_MS);
	CHECK_UINT(services_waiting(), 1);

	CASE("the answer on the new connection plans the next refresh");
	rec = services_iq();
	expires_text(EXPIRY_MS, when, sizeof(when));
	services_answer(rec, "turn.example.net", "3478", "u2", when);
	CHECK_UINT(fk.timers, 1);
	fake_advance(EXPIRY_MS - XMPP_RELAY_LEAD_MS);
	CHECK_UINT(services_waiting(), 1);

	xmpp_accounts_done();
}

/*
 * An expires attribute that the layer cannot read leaves the credential out.
 */
static void test_expiry_unreadable(void)
{
	static const char *const bad[] = {
		"soon",
		"2099-02-30T00:00:00Z",
		"2099-13-01T00:00:00Z",
		"2099-01-01T24:00:00Z",
		"2099-01-01T00:00:00+24:00",
		"2099-01-01T00:00:00.Z",
		"2099-1-01T00:00:00Z",
	};
	char host[64], when[64];
	struct iq_rec *rec;
	unsigned int i;

	layer_start(NULL);
	rec = services_iq();
	CHECK(rec != NULL);
	if (!rec)
		return;

	for (i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		CASE("a TURN relay with an unreadable expires goes into no call");
		snprintf(host, sizeof(host), "bad%u.example.net", i);
		services_answer(rec, host, "3478", "u1", bad[i]);
		CHECK_UINT(turn_resolved(host, "192.0.2.70"), 0);
	}

	CASE("an expires with an offset east of UTC is its UTC instant");
	expires_text(EXPIRY_MS + 5 * 3600000LL, when, sizeof(when));
	CHECK(strlen(when) > 1);
	when[strlen(when) - 1] = 0;
	strncat(when, "+05:00", sizeof(when) - strlen(when) - 1);
	services_answer(rec, "good.example.net", "3478", "u2", when);
	CHECK_UINT(turn_resolved("good.example.net", "192.0.2.71"), 1);
	CHECK(arms.n && arms.ms[arms.n - 1] > 539000 &&
	      arms.ms[arms.n - 1] <= 540000);

	CASE("an expires with an offset west of UTC is its UTC instant");
	expires_text(EXPIRY_MS - 5 * 3600000LL, when, sizeof(when));
	CHECK(strlen(when) > 1);
	when[strlen(when) - 1] = 0;
	strncat(when, "-05:00", sizeof(when) - strlen(when) - 1);
	services_answer(rec, "good.example.net", "3478", "u4", when);
	CHECK_UINT(turn_resolved("good.example.net", "192.0.2.71"), 1);
	CHECK(arms.n && arms.ms[arms.n - 1] > 539000 &&
	      arms.ms[arms.n - 1] <= 540000);

	CASE("a newer answer with an unreadable expires ends usable credentials");
	services_answer(rec, "good.example.net", "3478", "u3", "soon");
	CHECK_UINT(turn_resolved("good.example.net", "192.0.2.71"), 0);

	xmpp_accounts_done();
}

/*
 * The layer drops answers of an earlier configuration.
 */
static void test_generations(void)
{
	static struct cfg moved;
	const struct query_rec *old_q, *new_q;
	struct hal_transport tp;
	struct iq_rec *old_rec;
	char old_text[TEXT_MAX];

	layer_start(NULL);
	old_rec = services_iq();
	CHECK(old_rec != NULL);
	if (!old_rec)
		return;
	snprintf(old_text, sizeof(old_text), "%s", old_rec->text);
	services_answer(old_rec, "turn.example.net", "3478", "u1", NULL);
	old_q = query_find("turn.example.net");
	CHECK(old_q != NULL);
	if (!old_q)
		return;

	CASE("a reload cancels the resolver questions of the relays");
	moved = cfg;
	snprintf(moved.channel[0].xmpp_turn, sizeof(moved.channel[0].xmpp_turn),
		 "turn2.example.net");
	xmpp_accounts_reload(&cfg, &moved);
	CHECK(xmpp_account_active(0));
	CHECK(cancelled(old_q->query));
	CHECK(services_iq() != old_rec);

	CASE("a resolver answer of the earlier configuration is dropped");
	dns_answer(old_q, "198.51.100.66");
	CHECK_UINT(turn_of(&tp), 0);
	new_q = query_find("turn2.example.net");
	CHECK(new_q != NULL);
	if (!new_q)
		return;
	dns_answer(new_q, "192.0.2.60");
	CHECK_UINT(turn_of(&tp), 1);
	CHECK(!strcmp(tp.turn[0].host, "192.0.2.60"));

	CASE("a discovery answer of the earlier configuration is dropped");
	services_answer(old_rec, "old.example.net", "3478", "u9", NULL);
	CHECK(query_find("old.example.net") == NULL);
	CHECK_UINT(turn_of(&tp), 1);

	CASE("a stream answer of the resolver reaches the stream");
	fs.dns_answers = 0;
	fr.q[0].owner = 0;
	dns_answer(&fr.q[0], "192.0.2.1");
	CHECK_UINT(fs.dns_answers, 1);

	xmpp_accounts_done();
}

/*
 * The environment that the layer gives the legs.
 */
static void test_legs_env(void)
{
	struct iq_rec *rec;

	layer_start(NULL);

	CASE("do-not-disturb of the legs asks the shell for the line");
	dnd_asks = 0;
	CHECK(legs_env.dnd != NULL);
	if (!legs_env.dnd)
		return;
	CHECK(legs_env.dnd(legs_env.priv, 0));
	CHECK(!legs_env.dnd(legs_env.priv, 1));
	CHECK_UINT(dnd_asks, 2);

	CASE("the legs ask the shell whether the line drives the Jingle table");
	held_line = 99;
	held_legs = NULL;
	CHECK(legs_env.legs_held != NULL);
	if (!legs_env.legs_held)
		return;
	CHECK(legs_env.legs_held(legs_env.priv, 0));
	CHECK_UINT(held_line, 0);
	CHECK(held_legs == &xcalls_leg_ops);
	CHECK(!legs_env.legs_held(legs_env.priv, 1));

	CASE("an iq of a leg that the stream flushes fails toward the leg");
	leg_replies = 0;
	CHECK(legs_env.iq_send(legs_env.priv, 0, LEG_IQ, strlen(LEG_IQ), "j1",
			       "romeo@montague.example/orchard", leg_reply,
			       NULL) == 0);
	rec = &fs.iq[fs.iqs - 1];
	iq_lost(rec);
	CHECK_UINT(leg_replies, 1);
	CHECK(!leg_reply_ok);

	xmpp_accounts_done();
}

int main(void)
{
	alarm(240);
	uloop_init();
	/* The cases run the defaults that a caller environment could replace. */
	unsetenv("UVOICE_XMPP_RELAY_LEAD_MS");
	unsetenv("UVOICE_XMPP_RELAY_RETRY_MS");

	memset(&parser, 0, sizeof(parser));
	SUITE_CHECK(xml_stream_init(&parser, &parser_cb, NULL) == 0);

	test_relays();
	test_refresh();
	test_refresh_retry();
	test_relays_replaced();
	test_refresh_teardown();
	test_reconnect();
	test_expiry_unreadable();
	test_generations();
	test_legs_env();
	SUITE_CHECK_UINT(fake_timer_dropped, 0);

	xml_stream_done(&parser);
	uloop_done();

	return tu_summary("test_xmpp_account");
}
