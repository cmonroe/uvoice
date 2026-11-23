// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 John Crispin <john@phrozen.org>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * Unit tests for the Jingle call legs (src/proto/xmpp/call.c).
 *
 * No daemon. The environment of the legs is a fake: the stanzas the legs
 * send go to a log, the iq answers are scripted, the clock is virtual
 * with a timer table, the media ops are those of proto_host_fake.c and the
 * callbacks toward the shell are recorded. The stanzas of the peer are
 * texts that run through the real parser of xml.c, so every path the stream
 * would take is the path the suite takes.
 */
#include <string.h>
#include <unistd.h>

#include "trace.h"
#include "proto/xmpp/call.h"
#include "proto/xmpp/jmi.h"
#include "proto_host_fake.h"
#include "test_util.h"

#define STREAM_OPEN							\
	"<stream:stream xmlns='jabber:client' "				\
	"xmlns:stream='http://etherx.jabber.org/streams'>"
#define SELF		"user@sip.example.test/uvoice"
#define SELF_BARE	"user@sip.example.test"
#define PEER_BARE	"romeo@montague.example"
#define PEER		"romeo@montague.example/orchard"
#define LOCAL_ADDR	"192.168.1.20"
#define FP_TEXT								\
	"02:1A:CC:54:27:AB:EB:9C:53:3F:3E:4A:65:2E:58:AD:"		\
	"62:61:2F:B4:F7:D1:E8:C0:8C:5E:FF:F4:FB:F6:76:F2"
#define CAND_PEER							\
	"<candidate component='1' foundation='1' generation='0' id='c1' "\
	"ip='10.0.1.1' network='1' port='8998' priority='2130706431' "	\
	"protocol='udp' type='host'/>"
#define TRANSPORT_PEER(setup)						\
	"<transport xmlns='urn:xmpp:jingle:transports:ice-udp:1' "	\
	"pwd='asd88fgpdd777uzjYhagZg' ufrag='8hhy'>"			\
	"<fingerprint xmlns='urn:xmpp:jingle:apps:dtls:0' hash='sha-256' "\
	"setup='" setup "'>" FP_TEXT "</fingerprint>" CAND_PEER		\
	"</transport>"
#define TRANSPORT_EMPTY(setup)						\
	"<transport xmlns='urn:xmpp:jingle:transports:ice-udp:1' "	\
	"pwd='asd88fgpdd777uzjYhagZg' ufrag='8hhy'>"			\
	"<fingerprint xmlns='urn:xmpp:jingle:apps:dtls:0' hash='sha-256' "\
	"setup='" setup "'>" FP_TEXT "</fingerprint></transport>"
#define DESC_OF(payloads)						\
	"<description xmlns='urn:xmpp:jingle:apps:rtp:1' media='audio'>"\
	payloads "<rtcp-mux/></description>"
#define DESC_PEER							\
	"<description xmlns='urn:xmpp:jingle:apps:rtp:1' media='audio'>"\
	"<payload-type id='9' name='G722' clockrate='8000'/>"		\
	"<payload-type id='0' name='PCMU' clockrate='8000'/>"		\
	"<payload-type id='101' name='telephone-event' clockrate='8000'/>"\
	"<rtcp-mux/></description>"

#define LOG_MAX		32
#define TEXT_MAX	JINGLE_STANZA_MAX
#define TIMERS_MAX	8
#define ID_MAX		64
#define TRACE_TEXT_MAX	4096

struct sent {
	char text[TEXT_MAX];
	char id[ID_MAX];
	bool iq;
	xcall_reply_cb cb;
	void *cb_priv;
};

struct fake {
	struct sent sent[LOG_MAX];
	unsigned int sends;
	unsigned int results;
	unsigned int errors;
	char error_condition[64];
	unsigned int ids;
	bool ready;
	bool self_known;
	bool iq_fail;
	bool result_fail;
	bool legs_lost;
	bool dnd;
	unsigned int dnd_asks;
	unsigned int relay_missing;
	unsigned int turns;
	struct proto_timer *timer[TIMERS_MAX];
	unsigned int timers;
	uint64_t now;
};

struct cblog {
	unsigned int incoming;
	unsigned int proceeding;
	unsigned int answered;
	unsigned int failed;
	unsigned int remote_bye;
	unsigned int remote_cancel;
	unsigned int hold_changed;
	unsigned int reinvite_done;
	unsigned int ended;
	unsigned int elsewhere;
	enum call_fail_class fail_class;
	bool held;
	char cid_number[JID_MAX];
	char cid_name[XCALL_NAME_MAX];
	struct proto_call_ended ended_info;
	char ended_number[JID_MAX];
	char ended_remote[JID_MAX];
	/* one letter per report in call order: F failed, B remote_bye,
	 * C remote_cancel, W answered_elsewhere, E ended */
	char order[16];
	/* the shell hangs up from inside each end report, as fsm.c does */
	bool hangup_in_report;
	unsigned int hangups_taken;
};

struct trace_cap {
	char text[TRACE_TEXT_MAX];
	size_t len;
	unsigned int lines;
};

static struct fake fake;
static struct cblog cbl;
static struct trace_cap tcap;
static struct proto_timer_env timer_env;
static struct cfg cfg;
static struct xml_stream parser;
static const struct xml_doc *parsed;
static unsigned int parsed_count;
static unsigned int line_now;
static bool parsed_is_iq;

/* ------------------------------------------------------------- the clock */

/**
 * fake_now() - the virtual clock
 * @env: unused
 *
 * Return: the time.
 */
static uint64_t fake_now(struct proto_timer_env *env)
{
	return fake.now;
}

/**
 * timer_drop() - forget a timer of the table
 * @timer: the timer
 */
static void timer_drop(struct proto_timer *timer)
{
	unsigned int i;

	for (i = 0; i < fake.timers; i++) {
		if (fake.timer[i] != timer)
			continue;
		fake.timer[i] = fake.timer[--fake.timers];
		return;
	}
}

/**
 * fake_timer_set() - arm a timer on the table
 * @env: unused
 * @timer: the timer
 * @ms: the delay
 */
static void fake_timer_set(struct proto_timer_env *env,
			   struct proto_timer *timer, unsigned int ms)
{
	timer_drop(timer);
	timer->due_ms = fake.now + ms;
	if (fake.timers < TIMERS_MAX)
		fake.timer[fake.timers++] = timer;
}

/**
 * fake_timer_cancel() - disarm a timer
 * @env: unused
 * @timer: the timer
 */
static void fake_timer_cancel(struct proto_timer_env *env,
			      struct proto_timer *timer)
{
	timer_drop(timer);
}

/**
 * advance() - move the clock, firing what falls due
 * @ms: how far
 */
static void advance(uint64_t ms)
{
	uint64_t target = fake.now + ms;
	struct proto_timer *next;
	unsigned int i;

	for (;;) {
		next = NULL;
		for (i = 0; i < fake.timers; i++) {
			if (fake.timer[i]->due_ms <= target &&
			    (!next || fake.timer[i]->due_ms < next->due_ms))
				next = fake.timer[i];
		}
		if (!next)
			break;
		fake.now = next->due_ms;
		timer_drop(next);
		proto_timer_expire(next);
	}

	fake.now = target;
}

/* ------------------------------------------------------- the environment */

/**
 * sent_add() - record one stanza the legs sent
 * @text: the stanza
 * @len: its length
 *
 * Return: the record, or NULL when the log is full.
 */
static struct sent *sent_add(const char *text, size_t len)
{
	struct sent *rec;

	if (fake.sends >= LOG_MAX) {
		fake.sends++;
		return NULL;
	}

	rec = &fake.sent[fake.sends++];
	memset(rec, 0, sizeof(*rec));
	snprintf(rec->text, sizeof(rec->text), "%.*s", (int)len, text);

	return rec;
}

/**
 * env_ready() - the readiness the fake stream reports
 * @priv: unused
 * @line: unused
 *
 * Return: fake.ready.
 */
static bool env_ready(void *priv, unsigned int line)
{
	return fake.ready;
}

/**
 * env_send() - record a stanza
 * @priv: unused
 * @line: unused
 * @text: the stanza
 * @len: its length
 *
 * Return: 0 while the fake stream is ready.
 */
static int env_send(void *priv, unsigned int line, const char *text,
		    size_t len)
{
	sent_add(text, len);

	return fake.ready ? 0 : -ENOTCONN;
}

/**
 * env_iq_send() - record an iq with the callback that awaits its answer
 * @priv: unused
 * @line: unused
 * @text: the iq
 * @len: its length
 * @id: its id
 * @peer: the JID that must answer, the address of the iq
 * @cb: the callback
 * @cb_priv: its pointer
 *
 * Return: 0 while the fake stream is ready, -EIO while fake.iq_fail holds,
 * -EINVAL for a peer the iq does not address.
 */
static int env_iq_send(void *priv, unsigned int line, const char *text,
		       size_t len, const char *id, const char *peer,
		       xcall_reply_cb cb, void *cb_priv)
{
	struct sent *rec = sent_add(text, len);
	char to[ID_MAX + JID_MAX];
	int to_len;

	if (fake.iq_fail)
		return -EIO;

	to_len = snprintf(to, sizeof(to), "to=\"%s\"", peer ? peer : "");
	if (!peer || !peer[0] || !memmem(text, len, to, (size_t)to_len))
		return -EINVAL;

	if (rec) {
		rec->iq = true;
		snprintf(rec->id, sizeof(rec->id), "%s", id);
		rec->cb = cb;
		rec->cb_priv = cb_priv;
	}

	return fake.ready ? 0 : -ENOTCONN;
}

/**
 * env_reply_result() - count a result the legs sent
 * @priv: unused
 * @line: unused
 * @iq: unused
 *
 * Return: 0, or -ENOBUFS while fake.result_fail holds.
 */
static int env_reply_result(void *priv, unsigned int line,
			    const struct xml_doc *iq)
{
	fake.results++;

	return fake.result_fail ? -ENOBUFS : 0;
}

/**
 * env_reply_error() - count an error the legs sent and keep its condition
 * @priv: unused
 * @line: unused
 * @iq: unused
 * @type: unused
 * @condition: the condition element
 *
 * Return: 0.
 */
static int env_reply_error(void *priv, unsigned int line,
			   const struct xml_doc *iq, const char *type,
			   const char *condition)
{
	fake.errors++;
	snprintf(fake.error_condition, sizeof(fake.error_condition), "%s",
		 condition);

	return 0;
}

/**
 * env_id_next() - a counted stanza id
 * @priv: unused
 * @line: unused
 * @out: receives the id
 * @len: capacity of @out
 */
static void env_id_next(void *priv, unsigned int line, char *out, size_t len)
{
	snprintf(out, len, "id%u", ++fake.ids);
}

/**
 * env_self_jid() - the bound JID of the fake account
 * @priv: unused
 * @line: unused
 * @out: receives it
 *
 * Return: fake.self_known.
 */
static bool env_self_jid(void *priv, unsigned int line, struct jid *out)
{
	return fake.self_known && jid_parse(SELF, out);
}

/**
 * env_local_addr() - the fixed address of the fake interface
 * @priv: unused
 * @line: unused
 * @out: receives it
 * @len: capacity of @out
 *
 * Return: true.
 */
static bool env_local_addr(void *priv, unsigned int line, char *out,
			   size_t len)
{
	snprintf(out, len, "%s", LOCAL_ADDR);

	return true;
}

/**
 * env_bind_dev() - the fixed netdev of the fake interface
 * @priv: unused
 * @line: unused
 *
 * Return: "eth0".
 */
static const char *env_bind_dev(void *priv, unsigned int line)
{
	return "eth0";
}

/**
 * env_contact_name() - the one roster entry of the fake account
 * @priv: unused
 * @line: unused
 * @bare: the bare JID
 * @out: receives the name
 * @len: capacity of @out
 *
 * Return: true for the peer of the suite.
 */
static bool env_contact_name(void *priv, unsigned int line, const char *bare,
			     char *out, size_t len)
{
	if (strcmp(bare, PEER_BARE))
		return false;

	snprintf(out, len, "Romeo");

	return true;
}

/**
 * env_speed_dial() - the one speed dial entry of the fake account
 * @priv: unused
 * @line: unused
 * @digits: the digits
 * @out: receives the JID
 * @len: capacity of @out
 *
 * Return: true for "42".
 */
static bool env_speed_dial(void *priv, unsigned int line, const char *digits,
			   char *out, size_t len)
{
	if (strcmp(digits, "42"))
		return false;

	snprintf(out, len, "%s", PEER_BARE);

	return true;
}

/**
 * env_ice_servers() - a STUN server and, when scripted, one TURN relay
 * @priv: unused
 * @line: unused
 * @out: the transport block
 */
static void env_ice_servers(void *priv, unsigned int line,
			    struct hal_transport *out)
{
	snprintf(out->stun_host, sizeof(out->stun_host), "203.0.113.9");
	out->stun_port = 3478;
	out->turns = (uint8_t)fake.turns;
	if (fake.turns) {
		snprintf(out->turn[0].host, sizeof(out->turn[0].host),
			 "203.0.113.9");
		out->turn[0].port = 3478;
	}
}

/**
 * env_relay_missing() - count the warning of a call without a relay
 * @priv: unused
 * @line: unused
 */
static void env_relay_missing(void *priv, unsigned int line)
{
	fake.relay_missing++;
}

/**
 * env_dnd() - the scripted do-not-disturb state, with a count of the asks
 * @priv: unused
 * @line: unused
 *
 * Return: fake.dnd.
 */
static bool env_dnd(void *priv, unsigned int line)
{
	fake.dnd_asks++;

	return fake.dnd;
}

/**
 * env_legs_held() - whether the scripted line drives the Jingle legs
 * @priv: unused
 * @line: unused
 *
 * Return: false while fake.legs_lost holds.
 */
static bool env_legs_held(void *priv, unsigned int line)
{
	return !fake.legs_lost;
}

/* ------------------------------------------------------- the callbacks */

/**
 * report_take() - note one end report and, when scripted, hang up inside it
 * @line: the line
 * @slot: the slot
 * @letter: the letter of the report in cbl.order
 */
static void report_take(unsigned int line, unsigned int slot, char letter)
{
	size_t len = strlen(cbl.order);

	if (len + 1 < sizeof(cbl.order))
		cbl.order[len] = letter;

	if (cbl.hangup_in_report && letter != 'E' && xcall_hangup(line, slot))
		cbl.hangups_taken++;
}

/**
 * cb_incoming() - record an offered call
 * @line: unused
 * @slot: unused
 * @cid_number: the caller
 * @cid_name: the caller's name
 * @number_private: unused
 * @name_private: unused
 * @ring_id: unused
 */
static void cb_incoming(unsigned int line, unsigned int slot,
			const char *cid_number, const char *cid_name,
			bool number_private, bool name_private,
			unsigned int ring_id)
{
	cbl.incoming++;
	snprintf(cbl.cid_number, sizeof(cbl.cid_number), "%s", cid_number);
	snprintf(cbl.cid_name, sizeof(cbl.cid_name), "%s", cid_name);
}

/**
 * cb_proceeding() - count a proceeding report
 * @line: unused
 * @slot: unused
 * @early_media: unused
 */
static void cb_proceeding(unsigned int line, unsigned int slot,
			  bool early_media)
{
	cbl.proceeding++;
}

/**
 * cb_answered() - count an answer
 * @line: unused
 * @slot: unused
 */
static void cb_answered(unsigned int line, unsigned int slot)
{
	cbl.answered++;
}

/**
 * cb_failed() - count a failure and keep its class
 * @line: the line
 * @slot: the slot
 * @cls: the class
 */
static void cb_failed(unsigned int line, unsigned int slot,
		      enum call_fail_class cls)
{
	cbl.failed++;
	cbl.fail_class = cls;
	report_take(line, slot, 'F');
}

/**
 * cb_remote_bye() - count a far end clear of an answered call
 * @line: the line
 * @slot: the slot
 */
static void cb_remote_bye(unsigned int line, unsigned int slot)
{
	cbl.remote_bye++;
	report_take(line, slot, 'B');
}

/**
 * cb_remote_cancel() - count a withdrawn offer
 * @line: the line
 * @slot: the slot
 */
static void cb_remote_cancel(unsigned int line, unsigned int slot)
{
	cbl.remote_cancel++;
	report_take(line, slot, 'C');
}

/**
 * cb_hold_changed() - record a hold of the peer
 * @line: unused
 * @slot: unused
 * @held: the new state
 */
static void cb_hold_changed(unsigned int line, unsigned int slot, bool held)
{
	cbl.hold_changed++;
	cbl.held = held;
}

/**
 * cb_reinvite_done() - count a finished hold or resume
 * @line: unused
 * @slot: unused
 * @ok: unused
 */
static void cb_reinvite_done(unsigned int line, unsigned int slot, bool ok)
{
	cbl.reinvite_done++;
}

/**
 * cb_ended() - keep the record of the leg that ended
 * @line: the line
 * @slot: the slot
 * @info: the record, copied with its two strings
 */
static void cb_ended(unsigned int line, unsigned int slot,
		     const struct call_ended *info)
{
	report_take(line, slot, 'E');
	cbl.ended++;
	cbl.ended_info = *info;
	snprintf(cbl.ended_number, sizeof(cbl.ended_number), "%s",
		 info->number ? info->number : "");
	snprintf(cbl.ended_remote, sizeof(cbl.ended_remote), "%s",
		 info->remote_number ? info->remote_number : "");
}

/**
 * cb_elsewhere() - count an offer another device took
 * @line: the line
 * @slot: the slot
 */
static void cb_elsewhere(unsigned int line, unsigned int slot)
{
	cbl.elsewhere++;
	report_take(line, slot, 'W');
}

static const struct call_cb call_cb = {
	.incoming = cb_incoming,
	.proceeding = cb_proceeding,
	.answered = cb_answered,
	.failed = cb_failed,
	.remote_bye = cb_remote_bye,
	.remote_cancel = cb_remote_cancel,
	.hold_changed = cb_hold_changed,
	.reinvite_done = cb_reinvite_done,
	.ended = cb_ended,
	.answered_elsewhere = cb_elsewhere,
};

/* ------------------------------------------------------------ the parser */

/**
 * on_stanza() - hand a parsed peer stanza to the legs
 * @user: unused
 * @doc: the tree
 */
static void on_stanza(void *user, const struct xml_doc *doc)
{
	parsed_count++;
	parsed = doc;
	if (xml_node_is(doc, 0, "iq", XMPP_NS_CLIENT))
		parsed_is_iq = xcall_jingle_rx(line_now, doc);
	else
		parsed_is_iq = xcall_jmi_rx(line_now, doc);
	parsed = NULL;
}

static const struct xml_stream_cb parser_cb = {
	.stanza = on_stanza,
};

/**
 * rx() - the peer sends one stanza to line 0
 * @xml: the stanza
 *
 * Return: true when the legs took it as theirs.
 */
static bool rx(const char *xml)
{
	parsed_count = 0;
	parsed_is_iq = false;
	line_now = 0;
	xml_stream_feed(&parser, xml, strlen(xml));

	return parsed_count == 1 && parsed_is_iq;
}

/* ------------------------------------------------------------ fixtures */

/**
 * reset() - a fresh environment and a fresh leg layer
 */
static void reset(void)
{
	struct xcall_env env = {
		.timer = &timer_env,
		.media = &proto_host_fake()->media,
		.cb = &call_cb,
		.host = proto_host_fake(),
		.ready = env_ready,
		.send = env_send,
		.iq_send = env_iq_send,
		.reply_result = env_reply_result,
		.reply_error = env_reply_error,
		.id_next = env_id_next,
		.self_jid = env_self_jid,
		.local_addr = env_local_addr,
		.bind_dev = env_bind_dev,
		.contact_name = env_contact_name,
		.speed_dial = env_speed_dial,
		.ice_servers = env_ice_servers,
		.relay_missing = env_relay_missing,
		.dnd = env_dnd,
		.legs_held = env_legs_held,
	};

	memset(&fake, 0, sizeof(fake));
	memset(&cbl, 0, sizeof(cbl));
	fake.ready = true;
	fake.self_known = true;
	fake.now = 1000;

	proto_host_fake_reset();
	snprintf(phf.ice_local.ufrag, sizeof(phf.ice_local.ufrag), "myufrag");
	snprintf(phf.ice_local.pwd, sizeof(phf.ice_local.pwd),
		 "mypasswordmypasswordxx");
	phf.ice_local.fp_hash = HAL_FP_SHA256;
	memset(phf.ice_local.fp, 0xab, sizeof(phf.ice_local.fp));
	phf.ice_local.gen = 1;

	memset(&cfg, 0, sizeof(cfg));
	cfg.channels = CFG_CHANNEL_MAX;
	cfg.channel[0].present = true;
	cfg.channel[0].enabled = true;
	cfg.channel[0].dtmf_mode = CFG_DTMF_RFC2833;
	cfg.channel[0].ptime_ms = 20;
	cfg.channel[0].jitter_target = 60;
	cfg.channel[0].jitter_max = 200;
	cfg.channel[0].rtp_tos = 184;
	cfg.channel[0].rtcp_tos = 184;
	cfg.rtcp_enabled = true;
	cfg.rtcp_interval = 5;

	xcalls_init(&env);
	xcalls_reload(&cfg);
	xcalls_hal_features_set(HAL_FEAT_ICE_DTLS);
	xcalls_hal_codecs_set(~0u);

	xml_stream_reset(&parser);
	xml_stream_feed(&parser, STREAM_OPEN, strlen(STREAM_OPEN));
}

/* A transport record of line 0, stream 0, from the agent that the fake
 * backend holds now. */
static void transport_now(unsigned int what, unsigned int state)
{
	xcall_transport_event(0, 0, what, state, phf.ice_local.gen);
}

/**
 * sent_find() - the last stanza that holds a text
 * @needle: the text
 *
 * Return: the record, or NULL.
 */
static struct sent *sent_find(const char *needle)
{
	unsigned int i, n = fake.sends < LOG_MAX ? fake.sends : LOG_MAX;

	for (i = n; i > 0; i--) {
		if (strstr(fake.sent[i - 1].text, needle))
			return &fake.sent[i - 1];
	}

	return NULL;
}

/**
 * terminate_untracked() - whether the last session-terminate of the legs
 * waits for no answer
 *
 * Return: true when it left as a plain stanza.
 */
static bool terminate_untracked(void)
{
	struct sent *rec = sent_find("action=\"session-terminate\"");

	return rec && !rec->iq && !rec->cb;
}

/**
 * sid_take() - the session id of the propose the legs sent
 * @out: receives it
 * @len: capacity of @out
 *
 * Return: true when a propose was sent.
 */
static bool sid_take(char *out, size_t len)
{
	static const char prefix[] =
		"<propose xmlns=\"urn:xmpp:jingle-message:0\" id=\"";
	struct sent *rec = sent_find("<propose ");
	const char *at, *end;

	if (!rec)
		return false;

	at = strstr(rec->text, prefix);
	if (!at)
		return false;
	at += sizeof(prefix) - 1;
	end = strchr(at, '"');
	if (!end)
		return false;

	snprintf(out, len, "%.*s", (int)(end - at), at);

	return true;
}

/* ---------------------------------------------------------------- cases */

/*
 * A call this phone places, up to its end by the peer.
 */
static void test_outbound(void)
{
	char sid[JINGLE_SID_MAX];
	char xml[2048];
	struct sent *rec;
	unsigned int slot;

	reset();
	phf.cand_script[0] = "a=candidate:1 1 UDP 2122317823 192.168.1.20 "
			     "50000 typ host";
	phf.cand_n = 1;

	CASE("a dial proposes to the bare jid of the callee");
	CHECK(xcall_slot_free(0, &slot) && slot == 0);
	CHECK(xcall_line_dial_ready(0));
	CHECK(xcall_dial(0, 0, "42"));
	CHECK(fake.sends == 1);
	rec = sent_find("<propose ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" PEER_BARE "\"") != NULL);
	CHECK(rec && strstr(rec->text, "media=\"audio\"") != NULL);
	CHECK(sid_take(sid, sizeof(sid)));
	CHECK(xcall_slot_stream(0, 0) == 0);
	CHECK(!xcall_slot_free(0, &slot) || slot == 1);

	CASE("a ringing from a device of the callee reports proceeding");
	snprintf(xml, sizeof(xml), "<message from='" PEER "' to='" SELF
		 "' type='chat'><ringing xmlns='" JMI_NS "' id='%s'/>"
		 "</message>", sid);
	CHECK(rx(xml));
	CHECK(cbl.proceeding == 1);

	CASE("a proceed opens the session toward that device");
	snprintf(xml, sizeof(xml), "<message from='" PEER "' to='" SELF
		 "' type='chat'><proceed xmlns='" JMI_NS "' id='%s'/>"
		 "</message>", sid);
	CHECK(rx(xml));
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 1);
	CHECK(phf.n[PHF_MEDIA_START] == 1);
	CHECK(phf.hop[0].media.transport.kind == HAL_TRANSPORT_ICE_DTLS);
	CHECK(phf.hop[0].media.transport.ice_role == HAL_ICE_CONTROLLING);
	CHECK(phf.hop[0].media.transport.dtls_setup == HAL_DTLS_ACTPASS);
	CHECK(!phf.hop[0].media.transport.remote_ufrag[0]);
	CHECK(!strcmp(phf.hop[0].media.transport.stun_host, "203.0.113.9"));
	CHECK(phf.hop[0].media.local.ipver == 4);
	CHECK(phf.hop[0].media.codec == HAL_CODEC_PCMU);
	rec = sent_find("action=\"session-initiate\"");
	CHECK(rec != NULL);
	CHECK(rec && rec->iq);
	CHECK(rec && strstr(rec->text, "to=\"" PEER "\"") != NULL);
	CHECK(rec && strstr(rec->text, "initiator=\"" SELF "\"") != NULL);
	CHECK(rec && strstr(rec->text, "<payload-type id=\"0\" name=\"PCMU\" "
					"clockrate=\"8000\"/>") != NULL);
	CHECK(rec && strstr(rec->text, "name=\"telephone-event\"") != NULL);
	CHECK(rec && strstr(rec->text, "ufrag=\"myufrag\"") != NULL);
	CHECK(rec && strstr(rec->text, "setup=\"actpass\">AB:AB:") != NULL);
	CHECK(rec && strstr(rec->text, "<candidate") == NULL);
	rec = sent_find("action=\"transport-info\"");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "ip=\"192.168.1.20\"") != NULL);
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_NEXT] >= 2);

	CASE("a second ringing on the session reports proceeding again");
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' to='" SELF
		 "' type='set' id='r1'><jingle xmlns='" JINGLE_NS
		 "' action='session-info' sid='%s'><ringing xmlns='"
		 JINGLE_NS_RTP_INFO "'/></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(fake.results == 1);
	CHECK(cbl.proceeding == 2);

	CASE("the session-accept programs the far end and answers the call");
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' to='" SELF
		 "' type='set' id='a1'><jingle xmlns='" JINGLE_NS
		 "' action='session-accept' sid='%s' responder='" PEER "'>"
		 "<content creator='initiator' name='audio'>" DESC_PEER
		 TRANSPORT_PEER("active") "</content></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(fake.results == 2);
	CHECK(cbl.answered == 1);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 2);
	CHECK(phf.n[PHF_MEDIA_START] == 1);
	CHECK(phf.hop[2].entry == PHF_MEDIA_CONFIG_SET);
	CHECK(!strcmp(phf.hop[2].media.transport.remote_ufrag, "8hhy"));
	CHECK(phf.hop[2].media.transport.dtls_setup == HAL_DTLS_PASSIVE);
	CHECK(phf.hop[2].media.transport.remote_fp[0] == 0x02);
	CHECK(phf.hop[2].media.codec == HAL_CODEC_PCMU);
	CHECK(phf.hop[2].media.dtmf_pt == 101);
	CHECK(phf.hop[3].entry == PHF_MEDIA_CANDIDATE_ADD);
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 1);
	CHECK(strstr(phf.candidate, "10.0.1.1 8998 typ host") != NULL);

	CASE("a trickled candidate of the peer reaches the backend");
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='t1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='transport-info' "
		 "sid='%s'><content creator='initiator' name='audio'>"
		 "<transport xmlns='" JINGLE_NS_ICE "' ufrag='8hhy' "
		 "pwd='asd88fgpdd777uzjYhagZg'><candidate component='1' "
		 "foundation='2' generation='0' id='c2' ip='192.0.2.3' "
		 "network='1' port='45664' priority='1694498815' "
		 "protocol='udp' type='srflx'/></transport></content></jingle>"
		 "</iq>", sid);
	CHECK(rx(xml));
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 2);
	CHECK(strstr(phf.candidate, "192.0.2.3 45664 typ srflx") != NULL);

	CASE("the media comes up, a digit goes out as a telephone event");
	transport_now(HAL_TRANSPORT_EV_ICE_STATE, HAL_ICE_CONNECTED);
	transport_now(HAL_TRANSPORT_EV_DTLS_UP, 0);
	CHECK(fake.timers == 0);
	CHECK(xcall_dtmf(0, 0, '5', 100, false));
	CHECK(phf.n[PHF_MEDIA_DTMF_SEND] == 1);
	CHECK(phf.digit == '5');

	CASE("a hold reprograms the stream silent and tells the peer");
	CHECK(xcall_hold(0, 0));
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 3);
	CHECK(phf.hop[phf.nhop - 1].media.tx_silence);
	CHECK(sent_find("<hold xmlns=\"" JINGLE_NS_RTP_INFO "\"/>") != NULL);
	CHECK(cbl.reinvite_done == 1);
	CHECK(xcall_resume(0, 0));
	CHECK(!phf.hop[phf.nhop - 1].media.tx_silence);
	CHECK(cbl.reinvite_done == 2);

	CASE("a hold of the peer reports hold_changed");
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='h1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-info' sid='%s'>"
		 "<hold xmlns='" JINGLE_NS_RTP_INFO "'/></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(cbl.hold_changed == 1 && cbl.held);

	CASE("the peer ends the session: remote_bye and a record");
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='e1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-terminate' "
		 "sid='%s'><reason><success/></reason></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(cbl.remote_bye == 1);
	CHECK(cbl.ended == 1);
	CHECK(!cbl.ended_info.inbound);
	CHECK(cbl.ended_info.answered);
	CHECK(!cbl.ended_info.failed);
	CHECK(cbl.ended_info.media_ran);
	CHECK(cbl.ended_info.cleared);
	CHECK(cbl.ended_info.codec == HAL_CODEC_PCMU);
	CHECK(!strcmp(cbl.ended_number, PEER_BARE));
	CHECK(!strcmp(cbl.ended_remote, PEER_BARE));
	CHECK(phf.n[PHF_MEDIA_STOP] == 1);
	CHECK(phf.n[PHF_MEDIA_STATS_SNAPSHOT] == 1);
	CHECK(!xcalls_line_busy(0));
	CHECK(xcall_slot_stream(0, 0) == CALL_SLOTS);
}

/**
 * dtls_close_call() - bring one outbound call to flowing media
 * @sid: receives the session id
 * @len: capacity of @sid
 */
static void dtls_close_call(char *sid, size_t len)
{
	char xml[2048];

	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, len));
	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='a1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-accept' "
		 "sid='%s'><content creator='initiator' name='audio'>" DESC_PEER
		 TRANSPORT_PEER("active") "</content></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(cbl.answered == 1);
	transport_now(HAL_TRANSPORT_EV_DTLS_UP, 0);
	CHECK(fake.timers == 0);
}

/*
 * A close of the media by the peer is no failure.
 */
static void test_dtls_close(void)
{
	char sid[JINGLE_SID_MAX];
	char xml[1024];

	reset();

	CASE("a close of the media waits for the session-terminate of the peer");
	dtls_close_call(sid, sizeof(sid));
	fake.sends = 0;
	transport_now(HAL_TRANSPORT_EV_DTLS_CLOSED, 0);
	CHECK(fake.timers == 1);
	CHECK(fake.sends == 0);
	CHECK(cbl.remote_bye == 0);
	CHECK(xcalls_line_busy(0));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='e1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-terminate' "
		 "sid='%s'><reason><success/></reason></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(cbl.remote_bye == 1);
	CHECK(cbl.failed == 0);
	CHECK(cbl.ended == 1 && !cbl.ended_info.failed);
	CHECK(sent_find("<security-error/>") == NULL);
	CHECK(fake.timers == 0);
	CHECK(!xcalls_line_busy(0));

	reset();

	CASE("a close of the media without a session-terminate ends the call");
	dtls_close_call(sid, sizeof(sid));
	transport_now(HAL_TRANSPORT_EV_DTLS_CLOSED, 0);
	transport_now(HAL_TRANSPORT_EV_DTLS_CLOSED, 0);
	CHECK(fake.timers == 1);
	advance(XCALL_CLOSE_MS);
	CHECK(sent_find("<success/>") != NULL);
	CHECK(sent_find("<security-error/>") == NULL);
	CHECK(terminate_untracked());
	CHECK(cbl.remote_bye == 1);
	CHECK(cbl.failed == 0);
	CHECK(cbl.ended == 1 && !cbl.ended_info.failed);
	CHECK(cbl.ended_info.cleared);
	CHECK(!xcalls_line_busy(0));
}

/*
 * A record of the agent of an earlier call in the slot.
 *
 * The backend can queue a record of an agent before the stream stops. The
 * record can reach the legs after a new call took the same slot and a new
 * agent. Its generation differs from the one the new leg read, so it changes
 * nothing.
 */
static void test_transport_gen(void)
{
	char sid[JINGLE_SID_MAX];
	char xml[2048];
	uint16_t old_gen;

	reset();
	old_gen = phf.ice_local.gen;
	dtls_close_call(sid, sizeof(sid));
	CHECK(xcall_hangup(0, 0));
	CHECK(!xcalls_line_busy(0));
	phf.ice_local.gen = (uint16_t)(old_gen + 1);

	CASE("an old record before the new leg programs its stream changes "
	     "nothing");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	xcall_transport_event(0, 0, HAL_TRANSPORT_EV_DTLS_FAIL, 0, old_gen);
	CHECK(xcalls_line_busy(0));
	CHECK(cbl.failed == 0);

	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='a2'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-accept' "
		 "sid='%s'><content creator='initiator' name='audio'>" DESC_PEER
		 TRANSPORT_PEER("active") "</content></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(cbl.answered == 2);
	CHECK(fake.timers == 1);

	CASE("an old record after the new leg took its agent changes nothing");
	fake.sends = 0;
	xcall_transport_event(0, 0, HAL_TRANSPORT_EV_DTLS_UP, 0, old_gen);
	CHECK(fake.timers == 1);
	xcall_transport_event(0, 0, HAL_TRANSPORT_EV_DTLS_FAIL, 0, old_gen);
	xcall_transport_event(0, 0, HAL_TRANSPORT_EV_ICE_STATE, HAL_ICE_FAILED,
			      old_gen);
	xcall_transport_event(0, 0, HAL_TRANSPORT_EV_DTLS_CLOSED, 0, old_gen);
	CHECK(fake.timers == 1);
	CHECK(fake.sends == 0);
	CHECK(fake.relay_missing == 0);
	CHECK(cbl.failed == 0);
	CHECK(xcalls_line_busy(0));

	CASE("a record of the agent of the new leg still moves it");
	transport_now(HAL_TRANSPORT_EV_DTLS_UP, 0);
	CHECK(fake.timers == 0);
	transport_now(HAL_TRANSPORT_EV_DTLS_FAIL, 0);
	CHECK(sent_find("<security-error/>") != NULL);
	CHECK(cbl.failed == 1);
	CHECK(!xcalls_line_busy(0));
}

/*
 * The ways an outbound attempt ends early.
 */
static void test_outbound_refusals(void)
{
	char sid[JINGLE_SID_MAX];
	char xml[1024];
	struct sent *rec;

	reset();

	CASE("a target nothing maps reports unobtainable and places nothing");
	CHECK(!xcall_dial(0, 0, "99"));
	CHECK(cbl.failed == 1 && cbl.fail_class == CALL_FAIL_UNOBTAINABLE);
	CHECK(fake.sends == 0);
	CHECK(!xcall_dial(0, 0, "nolocal.example"));
	CHECK(cbl.failed == 2);

	CASE("a stream that is not ready reports congestion");
	fake.ready = false;
	CHECK(!xcall_dial(0, 0, PEER_BARE));
	CHECK(cbl.failed == 3 && cbl.fail_class == CALL_FAIL_CONGESTION);
	fake.ready = true;

	CASE("a reject by a device of the callee reports busy");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<reject xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	CHECK(cbl.failed == 4 && cbl.fail_class == CALL_FAIL_BUSY);
	CHECK(cbl.ended == 1 && cbl.ended_info.failed);

	CASE("a propose nobody answers is retracted as unobtainable");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(fake.timers == 1);
	advance(XCALL_RING_MS);
	CHECK(sent_find("<retract ") != NULL);
	CHECK(cbl.failed == 5 && cbl.fail_class == CALL_FAIL_UNOBTAINABLE);

	CASE("a hangup while proposing retracts");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	fake.sends = 0;
	CHECK(xcall_hangup(0, 0));
	rec = sent_find("<retract ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" PEER_BARE "\"") != NULL);
	CHECK(cbl.ended == 3 && cbl.ended_info.cleared);
	CHECK(!cbl.ended_info.failed);

	CASE("a peer that accepts without a fingerprint gets security-error");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='a1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-accept' "
		 "sid='%s'><content creator='initiator' name='audio'>" DESC_PEER
		 "<transport xmlns='" JINGLE_NS_ICE "' ufrag='a' pwd='b'/>"
		 "</content></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(sent_find("<security-error/>") != NULL);
	CHECK(cbl.failed == 6 && cbl.fail_class == CALL_FAIL_CONGESTION);

	CASE("a peer with no common codec ends with failed-application");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='a2'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-accept' "
		 "sid='%s'><content creator='initiator' name='audio'>"
		 "<description xmlns='" JINGLE_NS_RTP "' media='audio'>"
		 "<payload-type id='96' "
		 "name='speex' clockrate='16000'/><rtcp-mux/></description>"
		 TRANSPORT_PEER("active") "</content></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(sent_find("<failed-application/>") != NULL);
	CHECK(cbl.failed == 7);

	CASE("a session nobody accepts times out");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	advance(XCALL_SETUP_MS);
	CHECK(sent_find("<timeout/>") != NULL);
	CHECK(cbl.failed == 8 && cbl.fail_class == CALL_FAIL_CONGESTION);

	CASE("a failed transport without a relay tells the user");
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	snprintf(xml, sizeof(xml), "<message from='" PEER "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='%s'/></message>", sid);
	CHECK(rx(xml));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='a3'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-accept' "
		 "sid='%s'><content creator='initiator' name='audio'>" DESC_PEER
		 TRANSPORT_PEER("active") "</content></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(cbl.answered == 1);
	transport_now(HAL_TRANSPORT_EV_ICE_STATE, HAL_ICE_FAILED);
	CHECK(fake.relay_missing == 1);
	CHECK(sent_find("<connectivity-error/>") != NULL);
	CHECK(cbl.failed == 9);

	CASE("a stanza for an unknown session gets item-not-found");
	CHECK(rx("<iq from='" PEER "' type='set' id='u1'><jingle xmlns='"
		 JINGLE_NS "' action='session-terminate' sid='nosuch'>"
		 "<reason><success/></reason></jingle></iq>"));
	CHECK(fake.errors >= 1);
	CHECK(!strcmp(fake.error_condition, "item-not-found"));

	CASE("a stanza that is no jingle and no jmi is not taken");
	CHECK(!rx("<iq from='" PEER "' type='set' id='q1'><query xmlns='x:y'/>"
		  "</iq>"));
	CHECK(!rx("<message from='" PEER "' type='chat'><body>hi</body>"
		  "</message>"));
}

/*
 * A call another account places to this phone.
 */
static void test_inbound(void)
{
	char xml[2048];
	struct sent *rec;

	reset();
	fake.turns = 1;
	phf.cand_script[0] = "a=candidate:1 1 UDP 2122317823 192.168.1.20 "
			     "50000 typ host";
	phf.cand_n = 1;

	CASE("a propose rings the shell with the roster name");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s1'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(cbl.incoming == 1);
	CHECK(!strcmp(cbl.cid_number, PEER_BARE));
	CHECK(!strcmp(cbl.cid_name, "Romeo"));
	rec = sent_find("<ringing ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" PEER "\"") != NULL);
	CHECK(xcall_ring(0, 0));
	CHECK(xcall_slot_stream(0, 0) == 0);

	CASE("the same propose again changes nothing");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s1'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(cbl.incoming == 1);

	CASE("the answer proceeds to the caller and accepts to the account");
	fake.sends = 0;
	CHECK(xcall_answer(0, 0));
	rec = sent_find("<proceed ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" PEER "\"") != NULL);
	rec = sent_find("<accept ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" SELF_BARE "\"") != NULL);
	CHECK(fake.timers == 1);

	CASE("the session-initiate is answered with a session-accept");
	fake.sends = 0;
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' to='" SELF
		 "' type='set' id='i1'><jingle xmlns='" JINGLE_NS
		 "' action='session-initiate' sid='s1' initiator='" PEER "'>"
		 "<content creator='initiator' name='voice' senders='both'>"
		 DESC_PEER TRANSPORT_PEER("actpass") "</content></jingle>"
		 "</iq>");
	CHECK(rx(xml));
	CHECK(fake.results == 1);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 1);
	CHECK(phf.n[PHF_MEDIA_START] == 1);
	CHECK(phf.hop[0].entry == PHF_MEDIA_CONFIG_SET);
	CHECK(phf.hop[0].media.transport.ice_role == HAL_ICE_CONTROLLED);
	CHECK(phf.hop[0].media.transport.dtls_setup == HAL_DTLS_ACTIVE);
	CHECK(!strcmp(phf.hop[0].media.transport.remote_ufrag, "8hhy"));
	CHECK(phf.hop[0].media.transport.turns == 1);
	CHECK(phf.hop[2].entry == PHF_MEDIA_CANDIDATE_ADD);
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 1);
	rec = sent_find("action=\"session-accept\"");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "responder=\"" SELF "\"") != NULL);
	CHECK(rec && strstr(rec->text, "name=\"voice\"") != NULL);
	CHECK(rec && strstr(rec->text, "<payload-type id=\"0\" name=\"PCMU\" "
					"clockrate=\"8000\"/>") != NULL);
	CHECK(rec && strstr(rec->text, "name=\"G722\"") == NULL);
	CHECK(rec && strstr(rec->text, "setup=\"active\"") != NULL);
	CHECK(sent_find("action=\"transport-info\"") != NULL);
	CHECK(cbl.answered == 0);

	CASE("a transport that never comes up with a relay stays quiet");
	advance(XCALL_ICE_MS);
	CHECK(fake.relay_missing == 0);
	CHECK(sent_find("<connectivity-error/>") != NULL);
	CHECK(cbl.failed == 1);
	CHECK(cbl.ended == 1 && cbl.ended_info.inbound);
	CHECK(cbl.ended_info.answered);
}

/*
 * The ways an offered call ends without an answer.
 */
static void test_inbound_ends(void)
{
	struct sent *rec;

	reset();

	CASE("a retract of the caller is a remote cancel");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s2'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<retract xmlns='" JMI_NS "' id='s2'/></message>"));
	CHECK(cbl.remote_cancel == 1);
	CHECK(cbl.ended == 1 && cbl.ended_info.cleared);

	CASE("an accept another device sent ends the ring, answered elsewhere");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s3'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(cbl.incoming == 2);
	CHECK(rx("<message from='" SELF_BARE "' to='" SELF "' type='chat'>"
		 "<sent xmlns='urn:xmpp:carbons:2'><forwarded xmlns='"
		 XMPP_NS_FORWARD "'><message xmlns='jabber:client' from='"
		 SELF_BARE "/pc' to='" SELF_BARE "' type='chat'><accept xmlns='"
		 JMI_NS "' id='s3'/></message></forwarded></sent></message>"));
	CHECK(cbl.elsewhere == 1);
	CHECK(cbl.remote_cancel == 1);
	CHECK(cbl.ended == 2);

	CASE("a proceed of another device, seen as a received carbon, too");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s4'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(rx("<message from='" SELF_BARE "' to='" SELF "' type='chat'>"
		 "<proceed xmlns='" JMI_NS "' id='s4'/></message>"));
	CHECK(cbl.elsewhere == 2);

	CASE("a reject by the user goes to the caller");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s5'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	fake.sends = 0;
	CHECK(xcall_reject(0, 0, 486));
	rec = sent_find("<reject ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" PEER "\"") != NULL);
	CHECK(cbl.ended == 4);

	CASE("an offer with both slots taken is rejected at once");
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s6'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s7'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(cbl.incoming == 6);
	fake.sends = 0;
	CHECK(rx("<message from='" PEER "' to='" SELF_BARE "' type='chat'>"
		 "<propose xmlns='" JMI_NS "' id='s8'><description xmlns='"
		 JINGLE_NS_RTP "' media='audio'/></propose></message>"));
	CHECK(cbl.incoming == 6);
	rec = sent_find("<reject ");
	CHECK(rec != NULL);
	CHECK(rec && strstr(rec->text, "id=\"s8\"") != NULL);

	CASE("an offer nobody takes ends on its guard");
	advance(XCALL_OFFER_MS);
	CHECK(cbl.remote_cancel == 3);
	CHECK(!xcalls_line_busy(0));

	CASE("a direct session-initiate rings without a propose");
	CHECK(rx("<iq from='" PEER "' to='" SELF "' type='set' id='d1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-initiate' "
		 "sid='d1' initiator='" PEER "'><content creator='initiator' "
		 "name='audio'>" DESC_PEER TRANSPORT_PEER("actpass")
		 "</content></jingle></iq>"));
	CHECK(cbl.incoming == 7);
	CHECK(sent_find("<ringing xmlns=\"" JINGLE_NS_RTP_INFO "\"/>") != NULL);
	fake.sends = 0;
	CHECK(xcall_answer(0, 0));
	CHECK(sent_find("action=\"session-accept\"") != NULL);
	CHECK(phf.n[PHF_MEDIA_START] == 1);

	CASE("a stream that falls ends the call quietly");
	xcalls_stream_down(0);
	CHECK(cbl.failed == 1 && cbl.fail_class == CALL_FAIL_QUIET);
	CHECK(!xcalls_line_busy(0));
	CHECK(fake.timers == 0);
}

/* ------------------------------------------------- stanzas of the cases */

/**
 * propose_rx() - a propose of a caller reaches line 0
 * @from: the sender
 * @id: the session id
 *
 * Return: true when the legs took it.
 */
static bool propose_rx(const char *from, const char *id)
{
	char xml[1024];

	snprintf(xml, sizeof(xml), "<message from='%s' to='" SELF_BARE
		 "' type='chat'><propose xmlns='" JMI_NS "' id='%s'>"
		 "<description xmlns='" JINGLE_NS_RTP "' media='audio'/>"
		 "</propose></message>", from, id);

	return rx(xml);
}

/**
 * jmi_rx() - one JMI element without a child reaches line 0
 * @from: the sender
 * @kind: the element name
 * @id: the session id
 *
 * Return: true when the legs took it.
 */
static bool jmi_rx(const char *from, const char *kind, const char *id)
{
	char xml[1024];

	snprintf(xml, sizeof(xml), "<message from='%s' to='" SELF_BARE
		 "' type='chat'><%s xmlns='" JMI_NS "' id='%s'/></message>",
		 from, kind, id);

	return rx(xml);
}

/**
 * initiate_rx() - a session-initiate reaches line 0
 * @from: the from attribute, NULL for none
 * @sid: the session id
 * @content: the children of the content
 *
 * Return: true when the legs took it.
 */
static bool initiate_rx(const char *from, const char *sid, const char *content)
{
	char from_attr[2 * JID_MAX] = "";
	char xml[8192];

	if (from)
		snprintf(from_attr, sizeof(from_attr), " from='%s'", from);
	snprintf(xml, sizeof(xml), "<iq%s to='" SELF "' type='set' id='i-%s'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-initiate' "
		 "sid='%s' initiator='" PEER "'><content creator='initiator' "
		 "name='audio'>%s</content></jingle></iq>", from_attr, sid, sid,
		 content);

	return rx(xml);
}

/**
 * accept_rx() - the session-accept of the callee reaches line 0
 * @sid: the session id
 * @name: the content name
 * @content: the children of the content
 *
 * Return: true when the legs took it.
 */
static bool accept_rx(const char *sid, const char *name, const char *content)
{
	char xml[4096];

	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='a-%s'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-accept' "
		 "sid='%s' responder='" PEER "'><content creator='initiator' "
		 "name='%s'>%s</content></jingle></iq>", sid, sid, name,
		 content);

	return rx(xml);
}

/**
 * trickle_rx() - a transport-info with one candidate of the peer
 * @sid: the session id
 * @n: the number of the candidate, which sets its foundation and port
 *
 * Return: true when the legs took it.
 */
static bool trickle_rx(const char *sid, unsigned int n)
{
	char xml[2048];

	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='t%u'>"
		 "<jingle xmlns='" JINGLE_NS "' action='transport-info' "
		 "sid='%s'><content creator='initiator' name='audio'>"
		 "<transport xmlns='" JINGLE_NS_ICE "' ufrag='8hhy' "
		 "pwd='asd88fgpdd777uzjYhagZg'><candidate component='1' "
		 "foundation='%u' generation='0' id='c%u' ip='192.0.2.3' "
		 "network='1' port='%u' priority='1694498815' protocol='udp' "
		 "type='srflx'/></transport></content></jingle></iq>", n, sid,
		 n, n, 40000 + n);

	return rx(xml);
}

/**
 * outbound_initiated() - dial the peer and take its proceed
 * @sid: receives the session id
 * @len: capacity of @sid
 */
static void outbound_initiated(char *sid, size_t len)
{
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, len));
	CHECK(jmi_rx(PEER, "proceed", sid));
}

/**
 * hop_find() - the first call of an entry at or after a hop
 * @entry: the entry
 * @from: the first hop to look at
 *
 * Return: the index, or PHF_HOP_MAX when none.
 */
static unsigned int hop_find(enum phf_entry entry, unsigned int from)
{
	unsigned int i;

	for (i = from; i < phf.nhop && i < PHF_HOP_MAX; i++) {
		if (phf.hop[i].entry == entry)
			return i;
	}

	return PHF_HOP_MAX;
}

/**
 * hop_last() - the last call of an entry
 * @entry: the entry
 *
 * Return: the index, or PHF_HOP_MAX when none.
 */
static unsigned int hop_last(enum phf_entry entry)
{
	unsigned int i, found = PHF_HOP_MAX;

	for (i = 0; i < phf.nhop && i < PHF_HOP_MAX; i++) {
		if (phf.hop[i].entry == entry)
			found = i;
	}

	return found;
}

/* ----------------------------------------------------------- end reports */

/*
 * The reason of an end comes before its ended record.
 *
 * The shell and the desk classify a call from its end report. The ended
 * record closes a call they classified, as on a SIP line.
 */
static void test_end_order(void)
{
	char sid[JINGLE_SID_MAX];
	char xml[1024];

	reset();

	CASE("a retract reports remote_cancel before the ended record");
	CHECK(propose_rx(PEER, "o1"));
	CHECK(jmi_rx(PEER, "retract", "o1"));
	CHECK(!strcmp(cbl.order, "CE"));

	CASE("an accept of another device reports elsewhere before the record");
	reset();
	CHECK(propose_rx(PEER, "o2"));
	CHECK(rx("<message from='" SELF_BARE "' to='" SELF "' type='chat'>"
		 "<sent xmlns='urn:xmpp:carbons:2'><forwarded xmlns='"
		 XMPP_NS_FORWARD "'><message xmlns='jabber:client' from='"
		 SELF_BARE "/pc' to='" SELF_BARE "' type='chat'><accept xmlns='"
		 JMI_NS "' id='o2'/></message></forwarded></sent></message>"));
	CHECK(!strcmp(cbl.order, "WE"));

	CASE("a session-terminate reports remote_bye before the record");
	reset();
	dtls_close_call(sid, sizeof(sid));
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='e1'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-terminate' "
		 "sid='%s'><reason><success/></reason></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(!strcmp(cbl.order, "BE"));

	CASE("a reject of the callee reports failed before the record");
	reset();
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	CHECK(jmi_rx(PEER, "reject", sid));
	CHECK(!strcmp(cbl.order, "FE"));

	CASE("an offer nobody takes reports remote_cancel before the record");
	reset();
	CHECK(propose_rx(PEER, "o3"));
	advance(XCALL_OFFER_MS);
	CHECK(!strcmp(cbl.order, "CE"));

	CASE("a close of the media without a terminate reports remote_bye first");
	reset();
	dtls_close_call(sid, sizeof(sid));
	transport_now(HAL_TRANSPORT_EV_DTLS_CLOSED, 0);
	advance(XCALL_CLOSE_MS);
	CHECK(!strcmp(cbl.order, "BE"));

	CASE("a hangup inside the report finds no leg; the release follows");
	reset();
	cbl.hangup_in_report = true;
	dtls_close_call(sid, sizeof(sid));
	fake.sends = 0;
	snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' id='e2'>"
		 "<jingle xmlns='" JINGLE_NS "' action='session-terminate' "
		 "sid='%s'><reason><success/></reason></jingle></iq>", sid);
	CHECK(rx(xml));
	CHECK(!strcmp(cbl.order, "BE"));
	CHECK(cbl.hangups_taken == 0);
	CHECK(sent_find("action=\"session-terminate\"") == NULL);
	CHECK(phf.n[PHF_MEDIA_STOP] == 1);
	CHECK(cbl.ended == 1);
	CHECK(!xcalls_line_busy(0));
}

/* -------------------------------------------------------------- admission */

/*
 * The checks an inbound call passes before it rings.
 */
static void test_admission(void)
{
	struct sent *rec;

	reset();

	CASE("a DSP that is not ready refuses a propose before it rings");
	phf.hal_ready = false;
	fake.dnd = true;
	CHECK(propose_rx(PEER, "r1"));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<ringing ") == NULL);
	rec = sent_find("<reject ");
	CHECK(rec && strstr(rec->text, "id=\"r1\"") != NULL);
	CHECK(rec && strstr(rec->text, "to=\"" PEER "\"") != NULL);
	CHECK(fake.dnd_asks == 0);
	CHECK(!xcalls_line_busy(0));

	CASE("a DSP that is not ready refuses a direct initiate: general-error");
	fake.sends = 0;
	CHECK(initiate_rx(PEER, "r2", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 0);
	CHECK(fake.results == 1);
	CHECK(sent_find("<general-error/>") != NULL);
	CHECK(sent_find("<busy/>") == NULL);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 0);
	CHECK(fake.dnd_asks == 0);
	phf.hal_ready = true;

	CASE("a backend without ICE refuses the call");
	xcalls_hal_features_set(0);
	fake.sends = 0;
	CHECK(propose_rx(PEER, "r3"));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<reject ") != NULL);
	CHECK(fake.dnd_asks == 0);
	xcalls_hal_features_set(HAL_FEAT_ICE_DTLS);

	CASE("a line out of service refuses the call");
	phf.line_in_service = false;
	fake.sends = 0;
	CHECK(propose_rx(PEER, "r4"));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<reject ") != NULL);
	CHECK(fake.dnd_asks == 0);
	phf.line_in_service = true;

	CASE("do-not-disturb refuses a propose and a direct initiate: busy");
	fake.sends = 0;
	CHECK(propose_rx(PEER, "r5"));
	CHECK(cbl.incoming == 0);
	CHECK(fake.dnd_asks == 1);
	CHECK(sent_find("<reject ") != NULL);
	CHECK(initiate_rx(PEER, "r6", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<busy/>") != NULL);
	CHECK(!xcalls_line_busy(0));

	CASE("the terminate of a refused initiate waits for no answer");
	rec = sent_find("action=\"session-terminate\"");
	CHECK(rec && !rec->iq && !rec->cb);

	CASE("the reject of a refused propose is a message, not an iq");
	rec = sent_find("<reject ");
	CHECK(rec && !rec->iq && !rec->cb);
	fake.dnd = false;

	CASE("a line that drives the SIP legs refuses a propose and an initiate: "
	     "busy");
	fake.legs_lost = true;
	fake.sends = 0;
	CHECK(propose_rx(PEER, "r10"));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<ringing ") == NULL);
	CHECK(sent_find("<reject ") != NULL);
	CHECK(initiate_rx(PEER, "r11", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<busy/>") != NULL);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 0);
	CHECK(!xcalls_line_busy(0));
	fake.legs_lost = false;

	CASE("any JID can call the line");
	CHECK(propose_rx("juliet@capulet.example/balcony", "r7"));
	CHECK(cbl.incoming == 1);
	CHECK(!strcmp(cbl.cid_number, "juliet@capulet.example"));
	CHECK(!cbl.cid_name[0]);

	CASE("a propose whose ringing does not leave ends quietly, no ring");
	reset();
	fake.ready = false;
	CHECK(propose_rx(PEER, "r8"));
	CHECK(cbl.incoming == 0);
	CHECK(cbl.failed == 1 && cbl.fail_class == CALL_FAIL_QUIET);
	CHECK(!strcmp(cbl.order, "FE"));
	CHECK(!xcalls_line_busy(0));
	fake.ready = true;

	CASE("a direct initiate whose ringing does not leave ends quietly");
	fake.iq_fail = true;
	CHECK(initiate_rx(PEER, "r9", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 0);
	CHECK(cbl.failed == 2 && cbl.fail_class == CALL_FAIL_QUIET);
	CHECK(!xcalls_line_busy(0));
	fake.iq_fail = false;

	CASE("a direct initiate whose result does not leave ends quietly, no ring");
	reset();
	fake.result_fail = true;
	CHECK(initiate_rx(PEER, "r12", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find("<ringing ") == NULL);
	CHECK(cbl.failed == 1 && cbl.fail_class == CALL_FAIL_QUIET);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 0);
	CHECK(!xcalls_line_busy(0));

	CASE("a session behind a proceed whose result does not leave ends the "
	     "call");
	fake.result_fail = false;
	CHECK(propose_rx(PEER, "r13"));
	CHECK(xcall_answer(0, 0));
	fake.result_fail = true;
	CHECK(initiate_rx(PEER, "r13", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(sent_find("action=\"session-accept\"") == NULL);
	CHECK(cbl.failed == 2 && cbl.fail_class == CALL_FAIL_CONGESTION);
	CHECK(!xcalls_line_busy(0));
	fake.result_fail = false;
}

/* --------------------------------------------------- the proposed leg */

/*
 * An outbound leg knows no device of the callee before the proceed, so no
 * Jingle stanza can name its session yet.
 */
static void test_proposed_peer(void)
{
	static const struct {
		const char *action;
		const char *body;
	} stanza[] = {
		{ "session-terminate", "<reason><success/></reason>" },
		{ "session-info",
		  "<ringing xmlns='" JINGLE_NS_RTP_INFO "'/>" },
		{ "transport-info", "<content creator='initiator' "
		  "name='audio'>" TRANSPORT_PEER("active") "</content>" },
		{ "session-accept", "<content creator='initiator' "
		  "name='audio'>" DESC_PEER TRANSPORT_PEER("active")
		  "</content>" },
		{ "content-add", "<content creator='initiator' "
		  "name='audio'>" DESC_PEER "</content>" },
	};
	char sid[JINGLE_SID_MAX];
	char xml[4096];
	unsigned int i;

	reset();
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));

	for (i = 0; i < sizeof(stanza) / sizeof(stanza[0]); i++) {
		CASE("a jingle action on a proposed call gets item-not-found");
		fake.errors = 0;
		fake.error_condition[0] = 0;
		snprintf(xml, sizeof(xml), "<iq from='" PEER "' type='set' "
			 "id='p%u'><jingle xmlns='" JINGLE_NS "' action='%s' "
			 "sid='%s'>%s</jingle></iq>", i, stanza[i].action, sid,
			 stanza[i].body);
		CHECK(rx(xml));
		CHECK(fake.errors == 1 &&
		      !strcmp(fake.error_condition, "item-not-found"));
		CHECK(fake.results == 0);
	}

	CASE("the proposed call stays as it was");
	CHECK(xcall_slot_stream(0, 0) == 0);
	CHECK(cbl.failed == 0 && cbl.ended == 0 && cbl.answered == 0);
	CHECK(cbl.proceeding == 0);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 0);
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 0);
	CHECK(jmi_rx(PEER, "proceed", sid));
	CHECK(sent_find("action=\"session-initiate\"") != NULL);
	CHECK(xcall_hangup(0, 0));
}

/* ------------------------------------------------------------- the sender */

/*
 * A session or a proceed must come from one device.
 */
static void test_sender(void)
{
	char from[JID_MAX + JID_PART_MAX];
	char sid[JINGLE_SID_MAX];
	unsigned int configs;

	reset();

	CASE("a session-initiate from a bare JID is refused before it rings");
	CHECK(initiate_rx(PEER_BARE, "k1", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(fake.errors == 1 && !strcmp(fake.error_condition, "bad-request"));
	CHECK(cbl.incoming == 0);
	CHECK(fake.dnd_asks == 0);
	CHECK(!xcalls_line_busy(0));

	CASE("a session-initiate without a from is refused");
	CHECK(initiate_rx(NULL, "k2", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(fake.errors == 2 && !strcmp(fake.error_condition, "bad-request"));
	CHECK(cbl.incoming == 0);

	CASE("a sender whose resource does not fit is refused");
	snprintf(from, sizeof(from), PEER_BARE "/%0*d", (int)JID_PART_MAX, 0);
	CHECK(initiate_rx(from, "k3", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(fake.errors == 3 && !strcmp(fake.error_condition, "bad-request"));
	CHECK(cbl.incoming == 0);

	CASE("a propose from a bare JID does not ring");
	CHECK(propose_rx(PEER_BARE, "k6"));
	CHECK(cbl.incoming == 0);
	CHECK(fake.sends == 0);

	CASE("a session behind a proceed must come from the proposing device");
	CHECK(propose_rx(PEER, "k4"));
	CHECK(xcall_answer(0, 0));
	fake.sends = 0;
	CHECK(initiate_rx(PEER_BARE "/kitchen", "k4",
			  DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(fake.errors == 4 &&
	      !strcmp(fake.error_condition, "item-not-found"));
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 0);
	CHECK(sent_find("action=\"session-accept\"") == NULL);
	CHECK(initiate_rx(PEER, "k4", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(sent_find("action=\"session-accept\"") != NULL);
	CHECK(xcall_hangup(0, 0));

	CASE("a session-initiate for an offer nobody answered is refused");
	CHECK(propose_rx(PEER, "k5"));
	CHECK(initiate_rx(PEER, "k5", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(!strcmp(fake.error_condition, "unexpected-request"));
	CHECK(xcall_ring(0, 0));
	CHECK(xcall_hangup(0, 0));

	CASE("a proceed from a bare JID opens no session");
	configs = phf.n[PHF_MEDIA_CONFIG_SET];
	CHECK(xcall_dial(0, 0, PEER_BARE));
	CHECK(sid_take(sid, sizeof(sid)));
	fake.sends = 0;
	CHECK(jmi_rx(PEER_BARE, "proceed", sid));
	CHECK(sent_find("action=\"session-initiate\"") == NULL);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == configs);
	CHECK(jmi_rx(PEER, "proceed", sid));
	CHECK(sent_find("action=\"session-initiate\"") != NULL);
}

/* ---------------------------------------------------------------- carbons */

/*
 * A carbon counts only when the account's server sent it.
 */
static void test_carbons(void)
{
	reset();

	CASE("a carbon whose wrapper another account sent is dropped");
	CHECK(rx("<message from='mallory@evil.example' to='" SELF "' "
		 "type='chat'><received xmlns='urn:xmpp:carbons:2'><forwarded "
		 "xmlns='" XMPP_NS_FORWARD "'><message xmlns='jabber:client' "
		 "from='" PEER "' to='" SELF_BARE "' type='chat'><propose "
		 "xmlns='" JMI_NS "' id='c1'><description xmlns='" JINGLE_NS_RTP
		 "' media='audio'/></propose></message></forwarded></received>"
		 "</message>"));
	CHECK(cbl.incoming == 0);
	CHECK(fake.sends == 0);

	CASE("a carbon whose wrapper names a device of the account is dropped");
	CHECK(rx("<message from='" SELF "' to='" SELF "' type='chat'>"
		 "<received xmlns='urn:xmpp:carbons:2'><forwarded xmlns='"
		 XMPP_NS_FORWARD "'><message xmlns='jabber:client' from='" PEER
		 "' to='" SELF_BARE "' type='chat'><propose xmlns='" JMI_NS
		 "' id='c1'><description xmlns='" JINGLE_NS_RTP "' media='audio'"
		 "/></propose></message></forwarded></received></message>"));
	CHECK(cbl.incoming == 0);
	CHECK(fake.sends == 0);

	CASE("a received carbon of a propose rings once");
	CHECK(rx("<message from='" SELF_BARE "' to='" SELF "' type='chat'>"
		 "<received xmlns='urn:xmpp:carbons:2'><forwarded xmlns='"
		 XMPP_NS_FORWARD "'><message xmlns='jabber:client' from='" PEER
		 "' to='" SELF_BARE "' type='chat'><propose xmlns='" JMI_NS
		 "' id='c2'><description xmlns='" JINGLE_NS_RTP "' media='audio'"
		 "/></propose></message></forwarded></received></message>"));
	CHECK(cbl.incoming == 1);
	CHECK(!strcmp(cbl.cid_number, PEER_BARE));
	CHECK(propose_rx(PEER, "c2"));
	CHECK(cbl.incoming == 1);

	CASE("a retract from another device of the caller is ignored");
	CHECK(jmi_rx(PEER_BARE "/kitchen", "retract", "c2"));
	CHECK(jmi_rx("mallory@evil.example/x", "retract", "c2"));
	CHECK(cbl.remote_cancel == 0);
	CHECK(xcall_ring(0, 0));

	CASE("a retract from the proposing device ends the ring");
	CHECK(jmi_rx(PEER, "retract", "c2"));
	CHECK(cbl.remote_cancel == 1);
	CHECK(!xcalls_line_busy(0));

	CASE("a forged sent carbon of an accept does not end the ring");
	CHECK(propose_rx(PEER, "c3"));
	CHECK(rx("<message from='" PEER_BARE "' to='" SELF "' type='chat'>"
		 "<sent xmlns='urn:xmpp:carbons:2'><forwarded xmlns='"
		 XMPP_NS_FORWARD "'><message xmlns='jabber:client' from='"
		 SELF_BARE "/pc' to='" SELF_BARE "' type='chat'><accept xmlns='"
		 JMI_NS "' id='c3'/></message></forwarded></sent></message>"));
	CHECK(cbl.elsewhere == 0);
	CHECK(xcall_ring(0, 0));
}

/* ------------------------------------------------------ the iq replies */

/*
 * An iq answer reaches only the leg that sent the iq.
 *
 * The stream answers an iq with a result or an error stanza, or with no
 * stanza on a timeout and on a flush of its table. Each reaches the leg as
 * a callback with ok false, which is what the cases send.
 */
static void test_iq_gen(void)
{
	char sid[JINGLE_SID_MAX];
	xcall_reply_cb old_cb, cb;
	void *old_priv, *priv;
	struct sent *rec;

	reset();
	outbound_initiated(sid, sizeof(sid));
	rec = sent_find("action=\"session-initiate\"");
	CHECK(rec && rec->cb);
	if (!rec || !rec->cb)
		return;
	old_cb = rec->cb;
	old_priv = rec->cb_priv;
	CHECK(xcall_hangup(0, 0));

	CASE("a failed reply to an iq of an earlier leg of the slot is dropped");
	outbound_initiated(sid, sizeof(sid));
	CHECK(xcall_slot_stream(0, 0) == 0);
	old_cb(old_priv, false, NULL);
	CHECK(cbl.failed == 0);
	CHECK(xcalls_line_busy(0));

	CASE("a failed reply to the initiate of the leg ends it");
	rec = sent_find("action=\"session-initiate\"");
	CHECK(rec && rec->cb);
	if (!rec || !rec->cb)
		return;
	cb = rec->cb;
	priv = rec->cb_priv;
	cb(priv, false, NULL);
	CHECK(cbl.failed == 1 && cbl.fail_class == CALL_FAIL_CONGESTION);
	CHECK(!xcalls_line_busy(0));

	CASE("a reply after the leg ended changes nothing");
	cb(priv, false, NULL);
	CHECK(cbl.failed == 1);
	CHECK(cbl.ended == 2);
}

/* ------------------------------------------------- the remote candidates */

/**
 * transport_many() - a transport of the peer with many candidates
 * @out: receives the element
 * @len: capacity of @out
 * @n: the number of candidates, whose ports run from 41001
 */
static void transport_many(char *out, size_t len, unsigned int n)
{
	size_t used;
	unsigned int i;

	used = (size_t)snprintf(out, len, "<transport xmlns='" JINGLE_NS_ICE
				"' pwd='asd88fgpdd777uzjYhagZg' ufrag='8hhy'>"
				"<fingerprint xmlns='urn:xmpp:jingle:apps:dtls:0'"
				" hash='sha-256' setup='actpass'>" FP_TEXT
				"</fingerprint>");
	for (i = 1; i <= n && used < len; i++)
		used += (size_t)snprintf(out + used, len - used,
					 "<candidate component='1' "
					 "foundation='%u' generation='0' "
					 "id='m%u' ip='192.0.2.9' network='1' "
					 "port='%u' priority='1694498815' "
					 "protocol='udp' type='srflx'/>", i, i,
					 41000 + i);
	if (used < len)
		snprintf(out + used, len - used, "</transport>");
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

/* Copy the trace lines of the legs from here to trace_take_stop(). */
static void trace_take_start(void)
{
	memset(&tcap, 0, sizeof(tcap));
	trace_sink_set(trace_take);
	trace_ring_set(1u << TRACE_XMPP);
}

static void trace_take_stop(void)
{
	trace_ring_set(0);
	trace_sink_set(NULL);
}

/*
 * Candidates of the peer wait for its description.
 */
static void test_candidates(void)
{
	char sid[JINGLE_SID_MAX];
	char transport[6144];
	char content[6656];
	unsigned int i;

	reset();

	CASE("a candidate trickled before the accept waits");
	outbound_initiated(sid, sizeof(sid));
	CHECK(trickle_rx(sid, 1));
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 0);
	CHECK(cbl.failed == 0);

	CASE("an accept without a candidate hands the waiting one down after "
	     "the description");
	CHECK(accept_rx(sid, "audio", DESC_PEER TRANSPORT_EMPTY("active")));
	CHECK(cbl.answered == 1);
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 1);
	CHECK(strstr(phf.candidate, "192.0.2.3 40001 typ srflx") != NULL);
	CHECK(hop_last(PHF_MEDIA_CONFIG_SET) <
	      hop_find(PHF_MEDIA_CANDIDATE_ADD, 0));
	CHECK(!strcmp(phf.hop[hop_last(PHF_MEDIA_CONFIG_SET)].media.transport
		      .remote_ufrag, "8hhy"));

	CASE("a direct offer keeps its candidates until the answer");
	reset();
	CHECK(initiate_rx(PEER, "q2", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 1);
	CHECK(trickle_rx("q2", 2));
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 0);
	CHECK(xcall_answer(0, 0));
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == 2);
	CHECK(hop_find(PHF_MEDIA_CONFIG_SET, 0) <
	      hop_find(PHF_MEDIA_CANDIDATE_ADD, 0));

	CASE("a candidate past the full queue is dropped, and the call goes on");
	reset();
	CHECK(initiate_rx(PEER, "q3", DESC_PEER TRANSPORT_PEER("actpass")));
	for (i = 1; i < JINGLE_CANDIDATES_MAX; i++)
		CHECK(trickle_rx("q3", i));
	trace_take_start();
	CHECK(trickle_rx("q3", JINGLE_CANDIDATES_MAX));
	trace_take_stop();
	CHECK(cbl.failed == 0);
	CHECK(fake.results == 1 + JINGLE_CANDIDATES_MAX);
	CHECK(sent_find("<failed-transport/>") == NULL);
	CHECK(tcap.lines == 1);
	CHECK(strstr(tcap.text, "remote candidates dropped: 1, in the call: 1") !=
	      NULL);
	CHECK(xcall_answer(0, 0));
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == JINGLE_CANDIDATES_MAX);
	CHECK(strstr(phf.candidate, " 40015 ") != NULL);
	CHECK(xcalls_line_busy(0));

	CASE("a stanza with more candidates than fit keeps the first, and the "
	     "call goes on");
	reset();
	transport_many(transport, sizeof(transport), JINGLE_CANDIDATES_MAX + 2);
	snprintf(content, sizeof(content), DESC_PEER "%s", transport);
	trace_take_start();
	CHECK(initiate_rx(PEER, "q4", content));
	trace_take_stop();
	CHECK(cbl.failed == 0);
	CHECK(cbl.incoming == 1);
	CHECK(tcap.lines == 1);
	CHECK(strstr(tcap.text, "remote candidates dropped: 2, in the call: 2") !=
	      NULL);
	CHECK(xcall_answer(0, 0));
	CHECK(phf.n[PHF_MEDIA_CANDIDATE_ADD] == JINGLE_CANDIDATES_MAX);
	CHECK(strstr(phf.candidate, " 41016 ") != NULL);
	CHECK(xcalls_line_busy(0));

	CASE("a candidate the backend refuses fails the call");
	reset();
	dtls_close_call(sid, sizeof(sid));
	phf.candidate_add_fail = 1;
	CHECK(trickle_rx(sid, 7));
	CHECK(cbl.failed == 1 && cbl.fail_class == CALL_FAIL_CONGESTION);
	CHECK(sent_find("<failed-transport/>") != NULL);
	CHECK(!xcalls_line_busy(0));

	CASE("a candidate of the answer the backend refuses fails the call");
	reset();
	outbound_initiated(sid, sizeof(sid));
	phf.candidate_add_fail = 1;
	CHECK(accept_rx(sid, "audio", DESC_PEER TRANSPORT_PEER("active")));
	CHECK(cbl.answered == 0);
	CHECK(cbl.failed == 1);
	CHECK(sent_find("<failed-transport/>") != NULL);
	CHECK(!xcalls_line_busy(0));
}

/* ------------------------------------------------- the offer and answer */

/**
 * answer_refused() - an outbound call whose answer the leg refuses
 * @name: the content name of the answer
 * @content: the children of its content
 * @reason: the reason element the peer must get
 */
static void answer_refused(const char *name, const char *content,
			   const char *reason)
{
	char sid[JINGLE_SID_MAX];

	reset();
	outbound_initiated(sid, sizeof(sid));
	CHECK(accept_rx(sid, name, content));
	CHECK(cbl.answered == 0);
	CHECK(cbl.failed == 1);
	CHECK(sent_find(reason) != NULL);
	CHECK(terminate_untracked());
	CHECK(!xcalls_line_busy(0));
}

/**
 * offer_refused() - a direct offer the leg refuses before it rings
 * @sid: the session id
 * @content: the children of its content
 * @reason: the reason element the peer must get
 */
static void offer_refused(const char *sid, const char *content,
			  const char *reason)
{
	reset();
	CHECK(initiate_rx(PEER, sid, content));
	CHECK(cbl.incoming == 0);
	CHECK(sent_find(reason) != NULL);
	CHECK(terminate_untracked());
	CHECK(sent_find("<ringing ") == NULL);
	CHECK(!xcalls_line_busy(0));
}

/*
 * A session-terminate releases its leg at once, so it waits for no answer
 * and holds no slot of the iq table (XEP-0166 section 6.7).
 */
static void test_terminate_untracked(void)
{
	char sid[JINGLE_SID_MAX];

	CASE("the cancel of an initiated session waits for no answer");
	reset();
	outbound_initiated(sid, sizeof(sid));
	fake.sends = 0;
	CHECK(xcall_hangup(0, 0));
	CHECK(sent_find("<cancel/>") != NULL);
	CHECK(terminate_untracked());

	CASE("the timeout of an initiated session waits for no answer");
	reset();
	outbound_initiated(sid, sizeof(sid));
	fake.sends = 0;
	advance(XCALL_SETUP_MS);
	CHECK(sent_find("<timeout/>") != NULL);
	CHECK(terminate_untracked());

	CASE("the decline of a direct offer waits for no answer");
	reset();
	CHECK(initiate_rx(PEER, "u1", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 1);
	fake.sends = 0;
	CHECK(xcall_reject(0, 0, 486));
	CHECK(sent_find("<decline/>") != NULL);
	CHECK(terminate_untracked());

	CASE("the end of an answered call waits for no answer");
	reset();
	CHECK(initiate_rx(PEER, "u2", DESC_PEER TRANSPORT_PEER("actpass")));
	CHECK(xcall_answer(0, 0));
	fake.sends = 0;
	CHECK(xcall_hangup(0, 0));
	CHECK(sent_find("<success/>") != NULL);
	CHECK(terminate_untracked());

	CASE("the teardown of a line waits for no answer");
	reset();
	outbound_initiated(sid, sizeof(sid));
	fake.sends = 0;
	xcalls_line_teardown(0);
	CHECK(sent_find("<cancel/>") != NULL);
	CHECK(terminate_untracked());
	CHECK(!xcalls_line_busy(0));
}

/*
 * Both directions of the description checks.
 */
static void test_offer_answer(void)
{
	char sid[JINGLE_SID_MAX];

	reset();

	CASE("an offer without rtcp-mux is refused");
	offer_refused("m1", "<description xmlns='" JINGLE_NS_RTP "' "
		      "media='audio'><payload-type id='0' name='PCMU' "
		      "clockrate='8000'/></description>"
		      TRANSPORT_PEER("actpass"), "<failed-application/>");

	CASE("an offer without a setup role is refused");
	offer_refused("m2", DESC_PEER TRANSPORT_PEER(""), "<security-error/>");

	CASE("an offer of video is refused");
	offer_refused("m3", "<description xmlns='" JINGLE_NS_RTP "' "
		      "media='video'><payload-type id='96' name='VP8' "
		      "clockrate='90000'/><rtcp-mux/></description>"
		      TRANSPORT_PEER("actpass"), "<failed-application/>");

	CASE("an offer that moves the clock of PCMU is refused");
	offer_refused("m4", DESC_OF("<payload-type id='0' name='PCMU' "
				    "clockrate='16000'/>")
		      TRANSPORT_PEER("actpass"), "<failed-application/>");

	CASE("an offer that moves the channels of PCMU is refused");
	offer_refused("m5", DESC_OF("<payload-type id='0' name='PCMU' "
				    "clockrate='8000' channels='2'/>")
		      TRANSPORT_PEER("actpass"), "<failed-application/>");

	CASE("an offer of a dynamic PCMU without its clock is refused");
	offer_refused("m6", DESC_OF("<payload-type id='97' name='PCMU'/>")
		      TRANSPORT_PEER("actpass"), "<failed-application/>");

	CASE("an offer takes the first codec whose numbers fit");
	reset();
	CHECK(initiate_rx(PEER, "m7", DESC_OF("<payload-type id='0' name='PCMU' "
					      "clockrate='16000'/><payload-type "
					      "id='8' name='PCMA' "
					      "clockrate='8000'/>")
			  TRANSPORT_PEER("actpass")));
	CHECK(cbl.incoming == 1);
	CHECK(xcall_answer(0, 0));
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] >= 1);
	CHECK(phf.hop[hop_last(PHF_MEDIA_CONFIG_SET)].media.codec ==
	      HAL_CODEC_PCMA);

	CASE("an answer that names another content is refused");
	answer_refused("video", DESC_PEER TRANSPORT_PEER("active"),
		       "<failed-application/>");

	CASE("an answer without rtcp-mux is refused");
	answer_refused("audio", "<description xmlns='" JINGLE_NS_RTP "' "
		       "media='audio'><payload-type id='0' name='PCMU' "
		       "clockrate='8000'/></description>"
		       TRANSPORT_PEER("active"), "<failed-application/>");

	CASE("an answer that moves the payload number of PCMU is refused");
	answer_refused("audio", DESC_OF("<payload-type id='96' name='PCMU' "
					"clockrate='8000'/>")
		       TRANSPORT_PEER("active"), "<failed-application/>");

	CASE("an answer that moves the clock of PCMU is refused");
	answer_refused("audio", DESC_OF("<payload-type id='0' name='PCMU' "
					"clockrate='16000'/>")
		       TRANSPORT_PEER("active"), "<failed-application/>");

	CASE("an answer that moves the channels of PCMU is refused");
	answer_refused("audio", DESC_OF("<payload-type id='0' name='PCMU' "
					"clockrate='8000' channels='2'/>")
		       TRANSPORT_PEER("active"), "<failed-application/>");

	CASE("an answer that moves the telephone-event number is refused");
	answer_refused("audio", DESC_OF("<payload-type id='0' name='PCMU' "
					"clockrate='8000'/><payload-type "
					"id='100' name='telephone-event' "
					"clockrate='8000'/>")
		       TRANSPORT_PEER("active"), "<failed-application/>");

	CASE("an answer with the setup role actpass is refused");
	answer_refused("audio", DESC_PEER TRANSPORT_PEER("actpass"),
		       "<security-error/>");

	CASE("an answer of a static payload without its clock is taken");
	reset();
	outbound_initiated(sid, sizeof(sid));
	CHECK(accept_rx(sid, "audio", DESC_OF("<payload-type id='0' "
					      "name='PCMU'/>")
			TRANSPORT_PEER("passive")));
	CHECK(cbl.answered == 1);
	CHECK(phf.hop[hop_last(PHF_MEDIA_CONFIG_SET)].media.codec ==
	      HAL_CODEC_PCMU);
}

/* ------------------------------------------------------------ the stream */

/*
 * A configured stream is stopped once.
 */
static void test_stream_stop(void)
{
	char sid[JINGLE_SID_MAX];

	reset();

	CASE("a stream configured that fails to start is stopped once");
	phf.start_fail = 1;
	outbound_initiated(sid, sizeof(sid));
	CHECK(cbl.failed == 1);
	CHECK(phf.n[PHF_MEDIA_CONFIG_SET] == 1);
	CHECK(phf.n[PHF_MEDIA_START] == 1);
	CHECK(phf.n[PHF_MEDIA_STOP] == 1);
	CHECK(phf.n[PHF_MEDIA_STATS_SNAPSHOT] == 0);
	CHECK(!xcalls_line_busy(0));
}

int main(void)
{
	alarm(240);
	memset(&timer_env, 0, sizeof(timer_env));
	timer_env.now_ms = fake_now;
	timer_env.timer_set = fake_timer_set;
	timer_env.timer_cancel = fake_timer_cancel;

	memset(&parser, 0, sizeof(parser));
	SUITE_CHECK(xml_stream_init(&parser, &parser_cb, NULL) == 0);

	test_outbound();
	test_outbound_refusals();
	test_dtls_close();
	test_transport_gen();
	test_inbound();
	test_inbound_ends();
	test_end_order();
	test_admission();
	test_proposed_peer();
	test_sender();
	test_carbons();
	test_iq_gen();
	test_candidates();
	test_offer_answer();
	test_stream_stop();
	test_terminate_untracked();

	xcalls_done();
	xml_stream_done(&parser);

	return tu_summary("test_xmpp_call");
}
