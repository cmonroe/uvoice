// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 John Crispin <john@phrozen.org>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * The XMPP account layer. See xmpp/account.h for the contracts.
 *
 * One resolver serves every account. Its done callback routes by the owner
 * handle: the low byte names what asked, the stream or one relay, the rest
 * names the line.
 *
 * The iq answers of the legs. The stream reports an answer to the function
 * that sent the iq with its own callback type, so a small table relays each
 * pending answer to the callback the leg layer gave.
 */
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

#include <libubox/blobmsg.h>

#include "posix.h"
#include "str.h"
#include "proto/proto_trace.h"
#include "proto/shared/resolve.h"
#include "proto/shared/tls.h"
#include "proto/xmpp/account.h"
#include "proto/xmpp/call.h"
#include "proto/xmpp/disco.h"
#include "proto/xmpp/jingle.h"
#include "proto/xmpp/jmi.h"
#include "proto/xmpp/stream.h"

#define XMPP_NS_ROSTER		"jabber:iq:roster"
#define XMPP_NS_EXTDISCO	"urn:xmpp:extdisco:2"
#define XMPP_SERVICE_STUN	"stun"
#define XMPP_SERVICE_TURN	"turn"
#define XMPP_STUN_PORT		3478
#define XMPP_RELAYS_MAX		16U
#define XMPP_OWNER_STREAM	0U
#define XMPP_OWNER_SHIFT	8U
#define XMPP_OWNER_GEN_SHIFT	16U
#define XMPP_TIMER_REFRESH	1U
#define XMPP_STANZA_MAX		4096U
#define XMPP_FAULT_TEXT							\
	"the call could not connect and no TURN relay is configured"

struct xmpp_contact {
	char jid[JID_MAX];
	char name[XCALL_NAME_MAX];
};

/* A row with an empty host is free. */
struct xmpp_service {
	bool turn;
	bool resolved;
	bool configured;	/* the channel names it */
	char host[HAL_TRANSPORT_HOST_MAX];
	char addr[HAL_TRANSPORT_HOST_MAX];
	char user[HAL_TRANSPORT_CRED_MAX];
	char pass[HAL_TRANSPORT_CRED_MAX];
	uint16_t port;
	uint32_t query;
	uint64_t expires_ms;	/* on the timer clock, 0 for no expiry */
};

/* One service element of a discovery answer, valid while the answer is. */
struct service_row {
	bool turn;
	uint16_t port;
	const char *host;
	const char *user;
	const char *pass;
	uint64_t expires_ms;
};

struct xmpp_account {
	bool active;
	bool ready_posted;
	unsigned int line;
	uint16_t gen;			/* the generation of the configuration */
	struct cfg_channel ch;
	char iface[CFG_STR_MAX];
	struct sip_tls tls;
	struct xmpp_stream stream;
	struct xmpp_contact roster[XMPP_ROSTER_MAX];
	unsigned int contacts;
	struct xmpp_service service[XMPP_SERVICES_MAX];
	unsigned int services;
	struct proto_timer refresh;	/* asks for the relays again */
	uint64_t refresh_at;		/* its deadline on the timer clock */
	unsigned int refresh_fails;	/* failed refreshes in a row */
	bool refreshing;		/* the pending discovery is a refresh */
	char label[JID_MAX];
};

struct xmpp_relay {
	bool in_use;
	xcall_reply_cb cb;
	void *priv;
};

static struct {
	const struct proto_host *host;
	const struct cfg *cfg;
	struct sip_dns dns;
	struct proto_timer_env timer_env;
	struct xmpp_account acct[CFG_CHANNEL_MAX];
	struct xmpp_relay relay[XMPP_RELAYS_MAX];
	bool (*dnd)(unsigned int line);
	bool (*legs_held)(unsigned int line, const struct call_leg_ops *legs);
	uint16_t gen;
	bool inited;
	bool started;
} xa;

#define XMPP_ACCT_TRACE(acct, fmt, ...)					\
	PROTO_TRACE(xa.host, TRACE_XMPP, "xmpp line %u: " fmt, (acct)->line, \
		    ##__VA_ARGS__)

static void account_teardown(struct xmpp_account *acct);

/* --------------------------------------------------------------- helpers */

/**
 * owner_make() - the resolver owner of a question
 * @line: the line
 * @kind: XMPP_OWNER_STREAM, or the relay index plus one
 * @gen: the generation of the account a relay question belongs to
 *
 * Return: the owner handle.
 */
static uint32_t owner_make(unsigned int line, unsigned int kind, uint16_t gen)
{
	return ((uint32_t)gen << XMPP_OWNER_GEN_SHIFT) |
	       ((uint32_t)line << XMPP_OWNER_SHIFT) | kind;
}

/**
 * gen_tag() - the reply pointer of a discovery request
 * @acct: the account
 *
 * Return: the line and the generation of the account in one value.
 */
static void *gen_tag(const struct xmpp_account *acct)
{
	return (void *)(((uintptr_t)acct->gen << XMPP_OWNER_GEN_SHIFT) |
			acct->line);
}

/**
 * now_ms() - the clock of the relay timers
 *
 * Return: milliseconds.
 */
static uint64_t now_ms(void)
{
	return xa.timer_env.now_ms(&xa.timer_env);
}

/**
 * acct_get() - the account of a line
 * @line: the line
 *
 * Return: the account, or NULL when the line runs none.
 */
static struct xmpp_account *acct_get(unsigned int line)
{
	if (line >= CFG_CHANNEL_MAX || !xa.acct[line].active)
		return NULL;

	return &xa.acct[line];
}

/**
 * channel_xmpp() - whether a channel names an XMPP account
 * @cfg: the configuration
 * @ch: the channel
 *
 * Return: true for an enabled xmpp channel with a JID and a password.
 */
static bool channel_xmpp(const struct cfg *cfg, const struct cfg_channel *ch)
{
	return cfg->enabled && ch->present && ch->enabled &&
	       ch->account_type == CFG_ACCOUNT_XMPP && ch->xmpp_jid[0] &&
	       ch->xmpp_password[0];
}

/**
 * host_port_split() - "host[:port]" into its parts
 * @text: the text
 * @host: receives the host
 * @len: capacity of @host
 * @port: receives the port, left alone when the text names none
 */
static void host_port_split(const char *text, char *host, size_t len,
			    uint16_t *port)
{
	const char *colon = strrchr(text, ':');
	unsigned long value;

	if (!colon || strchr(text, ':') != colon) {
		str_copy(host, len, text);
		return;
	}

	snprintf(host, len, "%.*s", (int)(colon - text), text);
	value = strtoul(colon + 1, NULL, 10);
	if (value && value <= 65535)
		*port = (uint16_t)value;
}

/* ---------------------------------------------------------------- relays */

/**
 * service_is() - whether a relay row is the relay of a type, host and port
 * @svc: the row
 * @turn: whether the relay relays
 * @host: its host
 * @port: its port
 *
 * Return: true on a match.
 */
static bool service_is(const struct xmpp_service *svc, bool turn,
		       const char *host, uint16_t port)
{
	return svc->host[0] && svc->turn == turn && svc->port == port &&
	       !strcmp(svc->host, host);
}

/**
 * service_find() - the row of a relay the account knows
 * @acct: the account
 * @turn: whether the relay relays
 * @host: its host
 * @port: its port
 *
 * Return: the row, or NULL.
 */
static struct xmpp_service *service_find(struct xmpp_account *acct, bool turn,
					 const char *host, uint16_t port)
{
	unsigned int i;

	for (i = 0; i < acct->services; i++) {
		if (service_is(&acct->service[i], turn, host, port))
			return &acct->service[i];
	}

	return NULL;
}

/**
 * service_slot() - a free row of the relay table
 * @acct: the account
 *
 * A row keeps its index while it is used, because the index is the owner of
 * its resolver question.
 *
 * Return: the row, or NULL when the table is full.
 */
static struct xmpp_service *service_slot(struct xmpp_account *acct)
{
	unsigned int i;

	for (i = 0; i < acct->services; i++) {
		if (!acct->service[i].host[0])
			return &acct->service[i];
	}

	if (acct->services >= XMPP_SERVICES_MAX)
		return NULL;

	return &acct->service[acct->services++];
}

/**
 * service_drop() - forget one relay and its resolver question
 * @acct: the account
 * @svc: the row
 */
static void service_drop(struct xmpp_account *acct, struct xmpp_service *svc)
{
	XMPP_ACCT_TRACE(acct, "%s server %s:%u dropped",
			svc->turn ? "turn" : "stun", svc->host, svc->port);
	if (svc->query != SIP_HANDLE_NONE)
		sip_dns_cancel(&xa.dns, svc->query);
	memset(svc, 0, sizeof(*svc));

	while (acct->services && !acct->service[acct->services - 1].host[0])
		acct->services--;
}

/**
 * service_add() - remember one STUN or TURN server and resolve it
 * @acct: the account
 * @turn: whether it relays
 * @host: its host, a name or an address
 * @port: its port, 0 for the default
 * @user: the TURN user, "" for none
 * @pass: the TURN password, "" for none
 * @expires_ms: when the credentials expire on the timer clock, 0 for never
 *
 * A server the account knows keeps its address and takes the credentials
 * of the newer answer.
 *
 * Return: the row of the server, or NULL when the table is full.
 */
static struct xmpp_service *service_add(struct xmpp_account *acct, bool turn,
					const char *host, uint16_t port,
					const char *user, const char *pass,
					uint64_t expires_ms)
{
	struct xmpp_service *svc;

	if (!host || !host[0])
		return NULL;

	if (!port)
		port = XMPP_STUN_PORT;

	svc = service_find(acct, turn, host, port);
	if (svc) {
		str_copy(svc->user, sizeof(svc->user), user);
		str_copy(svc->pass, sizeof(svc->pass), pass);
		svc->expires_ms = expires_ms;
		return svc;
	}

	svc = service_slot(acct);
	if (!svc)
		return NULL;

	memset(svc, 0, sizeof(*svc));
	svc->turn = turn;
	svc->port = port;
	svc->expires_ms = expires_ms;
	str_copy(svc->host, sizeof(svc->host), host);
	str_copy(svc->user, sizeof(svc->user), user);
	str_copy(svc->pass, sizeof(svc->pass), pass);

	svc->query = sip_dns_resolve(&xa.dns, svc->host, SIP_DNS_ADDR,
				     svc->port,
				     owner_make(acct->line,
						(unsigned int)(svc -
							       acct->service) +
						1, acct->gen));
	XMPP_ACCT_TRACE(acct, "%s server %s:%u", turn ? "turn" : "stun",
			svc->host, svc->port);

	return svc;
}

/**
 * services_cancel() - drop the resolver questions of the relays
 * @acct: the account
 */
static void services_cancel(struct xmpp_account *acct)
{
	unsigned int i;

	for (i = 0; i < acct->services; i++) {
		if (acct->service[i].query == SIP_HANDLE_NONE)
			continue;
		sip_dns_cancel(&xa.dns, acct->service[i].query);
		acct->service[i].query = SIP_HANDLE_NONE;
	}
}

/**
 * services_configured() - the relays the channel names
 * @acct: the account
 */
static void services_configured(struct xmpp_account *acct)
{
	char host[HAL_TRANSPORT_HOST_MAX];
	struct xmpp_service *svc;
	uint16_t port;

	if (acct->ch.xmpp_stun[0]) {
		port = XMPP_STUN_PORT;
		host_port_split(acct->ch.xmpp_stun, host, sizeof(host), &port);
		svc = service_add(acct, false, host, port, "", "", 0);
		if (svc)
			svc->configured = true;
	}

	if (acct->ch.xmpp_turn[0]) {
		port = XMPP_STUN_PORT;
		host_port_split(acct->ch.xmpp_turn, host, sizeof(host), &port);
		svc = service_add(acct, true, host, port,
				  acct->ch.xmpp_turn_user,
				  acct->ch.xmpp_turn_password, 0);
		if (svc)
			svc->configured = true;
	}
}

/**
 * service_resolved() - the resolver answered a relay
 * @acct: the account
 * @index: the relay
 * @res: the answer
 */
static void service_resolved(struct xmpp_account *acct, unsigned int index,
			     const struct sip_dns_result *res)
{
	struct xmpp_service *svc;

	if (index >= acct->services)
		return;

	svc = &acct->service[index];
	svc->query = SIP_HANDLE_NONE;
	if (res->status != SIP_DNS_OK || !res->count) {
		XMPP_ACCT_TRACE(acct, "relay %s did not resolve: %s", svc->host,
				sip_dns_status_name(res->status));
		return;
	}

	str_copy(svc->addr, sizeof(svc->addr), res->target[0].host);
	svc->resolved = true;
	XMPP_ACCT_TRACE(acct, "relay %s is %s", svc->host, svc->addr);
}

/**
 * field_get() - a decimal field of a fixed width and the separator after it
 * @text: the cursor, moved past the field and the separator
 * @width: the number of digits
 * @sep: the separator, 0 for none
 * @out: receives the value
 *
 * Return: true when @width digits and @sep stand at the cursor.
 */
static bool field_get(const char **text, unsigned int width, char sep,
		      int *out)
{
	const char *p = *text;
	unsigned int i;
	int value = 0;

	for (i = 0; i < width; i++) {
		if (p[i] < '0' || p[i] > '9')
			return false;
		value = value * 10 + (p[i] - '0');
	}

	p += width;
	if (sep && *p++ != sep)
		return false;

	*text = p;
	*out = value;

	return true;
}

/**
 * month_days() - the days of a month
 * @year: the year
 * @month: the month, 1 to 12
 *
 * Return: the count.
 */
static int month_days(int year, int month)
{
	static const int days[] = {
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
	bool leap = (!(year % 4) && year % 100) || !(year % 400);

	return month == 2 && leap ? 29 : days[month - 1];
}

/**
 * datetime_ms() - the UTC milliseconds of an XEP-0082 DateTime
 * @text: CCYY-MM-DDThh:mm:ss[.sss]TZD, TZD being Z or +hh:mm or -hh:mm
 * @out: receives the milliseconds since the epoch
 *
 * timegm() moves a field out of its range into the next one, so the fields
 * are checked first. A second of 60 is a leap second.
 *
 * Return: true when the text is such a DateTime.
 */
static bool datetime_ms(const char *text, int64_t *out)
{
	int year, month, day, hour, min, sec, off_h = 0, off_m = 0;
	int64_t ms = 0, scale = 100, offset;
	struct tm tm;
	time_t secs;
	char tzd;

	if (!text || !field_get(&text, 4, '-', &year) ||
	    !field_get(&text, 2, '-', &month) ||
	    !field_get(&text, 2, 'T', &day) ||
	    !field_get(&text, 2, ':', &hour) ||
	    !field_get(&text, 2, ':', &min) || !field_get(&text, 2, 0, &sec))
		return false;

	if (month < 1 || month > 12 || day < 1 ||
	    day > month_days(year, month) || hour > 23 || min > 59 || sec > 60)
		return false;

	if (*text == '.') {
		if (text[1] < '0' || text[1] > '9')
			return false;
		for (text++; *text >= '0' && *text <= '9'; text++) {
			ms += (*text - '0') * scale;
			scale /= 10;
		}
	}

	tzd = *text;
	if (tzd != 'Z' && tzd != '+' && tzd != '-')
		return false;
	text++;
	if (tzd != 'Z' && (!field_get(&text, 2, ':', &off_h) ||
			   !field_get(&text, 2, 0, &off_m) || off_h > 23 ||
			   off_m > 59))
		return false;
	if (*text)
		return false;

	offset = (int64_t)(off_h * 60 + off_m) * 60;
	if (tzd == '-')
		offset = -offset;

	memset(&tm, 0, sizeof(tm));
	tm.tm_year = year - 1900;
	tm.tm_mon = month - 1;
	tm.tm_mday = day;
	tm.tm_hour = hour;
	tm.tm_min = min;
	tm.tm_sec = sec;
	secs = timegm(&tm);
	if (secs == (time_t)-1)
		return false;

	*out = ((int64_t)secs - offset) * 1000 + ms;

	return true;
}

/**
 * expiry_take() - the timer clock deadline of an expires attribute
 * @text: the attribute
 * @out: receives the deadline, now for a time that passed
 *
 * The attribute names a wall clock time, and the timers run on a monotonic
 * clock, so only the distance from now goes into the deadline.
 *
 * Return: false when the attribute is no XEP-0082 DateTime.
 */
static bool expiry_take(const char *text, uint64_t *out)
{
	struct timespec wall;
	int64_t expires, left;
	uint64_t now = now_ms();

	if (!datetime_ms(text, &expires) ||
	    clock_gettime(CLOCK_REALTIME, &wall))
		return false;

	left = expires - ((int64_t)wall.tv_sec * 1000 +
			  wall.tv_nsec / 1000000);
	*out = left > 0 ? now + (uint64_t)left : now;

	return true;
}

/**
 * refresh_timer_arm() - arm the refresh timer toward its deadline
 * @acct: the account
 *
 * The uloop timer holds an int, so a deadline further away than INT_MAX ms
 * takes more than one expiry of the timer.
 */
static void refresh_timer_arm(struct xmpp_account *acct)
{
	uint64_t now = now_ms();
	uint64_t left = acct->refresh_at > now ? acct->refresh_at - now : 0;

	proto_timer_arm(&xa.timer_env, &acct->refresh,
			left < INT_MAX ? (unsigned int)left : INT_MAX);
}

/**
 * relays_refresh_arm() - plan the next discovery before an expiry
 * @acct: the account
 *
 * The timer runs XMPP_RELAY_LEAD_MS before the first expiry that is still
 * ahead, or at half the time left when less than that remains.
 */
static void relays_refresh_arm(struct xmpp_account *acct)
{
	uint64_t lead = env_ms_get("UVOICE_XMPP_RELAY_LEAD_MS",
				   XMPP_RELAY_LEAD_MS);
	uint64_t now = now_ms(), first = 0, left;
	const struct xmpp_service *svc;
	unsigned int i;

	proto_timer_cancel(&xa.timer_env, &acct->refresh);
	acct->refresh_at = 0;

	for (i = 0; i < acct->services; i++) {
		svc = &acct->service[i];
		if (svc->turn && svc->expires_ms > now &&
		    (!first || svc->expires_ms < first))
			first = svc->expires_ms;
	}
	if (!first)
		return;

	left = first - now;
	left = left > lead ? left - lead : left / 2;
	acct->refresh_at = now + left;
	refresh_timer_arm(acct);
}

/**
 * relays_retry() - plan the next try of a failed refresh
 * @acct: the account
 * @why: what failed, for the trace
 *
 * The wait doubles from XMPP_RELAY_RETRY_MS, and the layer stops after
 * XMPP_RELAY_RETRIES retries. A new connection asks the server again.
 */
static void relays_retry(struct xmpp_account *acct, const char *why)
{
	unsigned int base = env_ms_get("UVOICE_XMPP_RELAY_RETRY_MS",
				       XMPP_RELAY_RETRY_MS);

	if (acct->refresh_fails >= XMPP_RELAY_RETRIES) {
		XMPP_ACCT_TRACE(acct, "relay refresh %s, no retry left", why);
		acct->refreshing = false;
		acct->refresh_fails = 0;
		return;
	}

	acct->refresh_fails++;
	XMPP_ACCT_TRACE(acct, "relay refresh %s, retry %u of %u", why,
			acct->refresh_fails, XMPP_RELAY_RETRIES);
	acct->refresh_at = now_ms() +
			   retry_delay_ms(acct->refresh_fails, base, UINT_MAX);
	refresh_timer_arm(acct);
}

/**
 * relays_refresh_stop() - drop the refresh schedule and its retries
 * @acct: the account
 */
static void relays_refresh_stop(struct xmpp_account *acct)
{
	proto_timer_cancel(&xa.timer_env, &acct->refresh);
	acct->refresh_at = 0;
	acct->refresh_fails = 0;
	acct->refreshing = false;
}

/**
 * service_row_get() - one service element of a discovery answer
 * @acct: the account
 * @reply: the answer
 * @node: the service element
 * @row: receives the relay
 *
 * A TURN relay with an expires that the layer cannot read is left out,
 * because its credentials can be stale already.
 *
 * Return: true for a STUN or TURN relay over UDP that the layer can use.
 */
static bool service_row_get(struct xmpp_account *acct,
			    const struct xml_doc *reply, unsigned int node,
			    struct service_row *row)
{
	const char *type = xml_attr_get(reply, node, "type");
	const char *transport = xml_attr_get(reply, node, "transport");
	const char *port_text = xml_attr_get(reply, node, "port");
	const char *expires = xml_attr_get(reply, node, "expires");
	unsigned long port = port_text ? strtoul(port_text, NULL, 10) : 0;

	memset(row, 0, sizeof(*row));
	row->host = xml_attr_get(reply, node, "host");
	if (!type || !row->host || !row->host[0] || port > 65535 ||
	    (transport && strcmp(transport, "udp")))
		return false;

	row->port = port ? (uint16_t)port : XMPP_STUN_PORT;
	if (!strcmp(type, XMPP_SERVICE_STUN))
		return true;
	if (strcmp(type, XMPP_SERVICE_TURN))
		return false;

	row->turn = true;
	row->user = xml_attr_get(reply, node, "username");
	row->pass = xml_attr_get(reply, node, "password");
	if (!expires || expiry_take(expires, &row->expires_ms))
		return true;

	XMPP_ACCT_TRACE(acct, "turn server %s: expires '%s' unreadable, dropped",
			row->host, expires);

	return false;
}

/**
 * service_named() - whether a discovery answer names a relay
 * @svc: the row of the relay
 * @row: the relays of the answer
 * @rows: their count
 *
 * Return: true when one of @row is @svc.
 */
static bool service_named(const struct xmpp_service *svc,
			  const struct service_row *row, unsigned int rows)
{
	unsigned int i;

	for (i = 0; i < rows; i++) {
		if (service_is(svc, row[i].turn, row[i].host, row[i].port))
			return true;
	}

	return false;
}

/**
 * service_row_merge() - add a relay of a discovery answer to its list
 * @row: the distinct relays of the answer so far
 * @rows: their count
 * @cand: the relay
 *
 * A relay that the answer names twice keeps its later row. The list holds
 * at most XMPP_SERVICES_MAX distinct relays.
 *
 * Return: the new count.
 */
static unsigned int service_row_merge(struct service_row *row,
				      unsigned int rows,
				      const struct service_row *cand)
{
	unsigned int i;

	for (i = 0; i < rows; i++) {
		if (row[i].turn == cand->turn && row[i].port == cand->port &&
		    !strcmp(row[i].host, cand->host)) {
			row[i] = *cand;
			return rows;
		}
	}

	if (rows < XMPP_SERVICES_MAX)
		row[rows++] = *cand;

	return rows;
}

/**
 * service_configured_restore() - give a relay the channel names its
 * configured credentials back
 * @acct: the account
 * @svc: the row of the relay
 */
static void service_configured_restore(struct xmpp_account *acct,
				       struct xmpp_service *svc)
{
	str_copy(svc->user, sizeof(svc->user),
		 svc->turn ? acct->ch.xmpp_turn_user : "");
	str_copy(svc->pass, sizeof(svc->pass),
		 svc->turn ? acct->ch.xmpp_turn_password : "");
	svc->expires_ms = 0;
}

/**
 * services_replace() - take the relays of a discovery answer
 * @acct: the account
 * @row: the relays the answer names
 * @rows: their count
 *
 * The answer replaces the relays of an earlier answer. The relays the
 * channel names stay, with their configured credentials when the answer
 * does not name them.
 */
static void services_replace(struct xmpp_account *acct,
			     const struct service_row *row, unsigned int rows)
{
	struct xmpp_service *svc;
	unsigned int i;

	for (i = 0; i < acct->services; i++) {
		svc = &acct->service[i];
		if (!svc->host[0] || service_named(svc, row, rows))
			continue;
		if (svc->configured)
			service_configured_restore(acct, svc);
		else
			service_drop(acct, svc);
	}

	for (i = 0; i < rows; i++)
		service_add(acct, row[i].turn, row[i].host, row[i].port,
			    row[i].user, row[i].pass, row[i].expires_ms);
}

/**
 * services_reply() - the answer of the XEP-0215 query
 * @xs: the stream
 * @reply: the iq, NULL for none
 * @priv: the line and the generation of the request (gen_tag())
 */
static void services_reply(struct xmpp_stream *xs, const struct xml_doc *reply,
			   void *priv)
{
	uintptr_t tag = (uintptr_t)priv;
	struct xmpp_account *acct = acct_get(tag & 0xff);
	struct service_row row[XMPP_SERVICES_MAX], cand;
	unsigned int rows = 0, list = XML_NONE, node;
	const char *why = "unanswered";
	const char *type;

	if (!acct || (uint16_t)(tag >> XMPP_OWNER_GEN_SHIFT) != acct->gen)
		return;

	if (reply) {
		type = xml_attr_get(reply, 0, "type");
		why = "refused";
		if (type && !strcmp(type, "result")) {
			why = "answered without services";
			list = xml_child(reply, XML_NONE, "services",
					 XMPP_NS_EXTDISCO);
		}
	}

	if (list == XML_NONE) {
		if (acct->refreshing)
			relays_retry(acct, why);
		return;
	}

	node = xml_child(reply, list, "service", XMPP_NS_EXTDISCO);
	for (; node != XML_NONE;
	     node = xml_next(reply, node, "service", XMPP_NS_EXTDISCO)) {
		if (service_row_get(acct, reply, node, &cand))
			rows = service_row_merge(row, rows, &cand);
	}

	services_replace(acct, row, rows);
	acct->refreshing = false;
	acct->refresh_fails = 0;
	relays_refresh_arm(acct);
}

/**
 * services_ask() - the XEP-0215 query to the server
 * @acct: the account
 *
 * Return: 0, or the errno of the stream.
 */
static int services_ask(struct xmpp_account *acct)
{
	char mem[XMPP_STANZA_MAX];
	char id[XMPP_ID_MAX];
	struct xml_w writer;
	size_t len;

	xmpp_stream_id_next(&acct->stream, id, sizeof(id));
	xml_w_init(&writer, mem, sizeof(mem));
	xml_w_start(&writer, "iq");
	xml_w_attr(&writer, "type", "get");
	xml_w_attr(&writer, "to", acct->stream.jid.domain);
	xml_w_attr(&writer, "id", id);
	xml_w_start(&writer, "services");
	xml_w_attr(&writer, "xmlns", XMPP_NS_EXTDISCO);
	xml_w_end(&writer);
	xml_w_end(&writer);
	len = xml_w_finish(&writer);
	if (!len)
		return -EMSGSIZE;

	return xmpp_stream_iq_send(&acct->stream, mem, len, id, NULL,
				   services_reply, gen_tag(acct));
}

/**
 * relays_refresh_fire() - the credentials of a relay expire soon
 * @timer: the refresh timer of the account
 */
static void relays_refresh_fire(struct proto_timer *timer)
{
	struct xmpp_account *acct = timer->owner;

	if (now_ms() < acct->refresh_at) {
		refresh_timer_arm(acct);
		return;
	}

	acct->refresh_at = 0;
	XMPP_ACCT_TRACE(acct, "relay credentials expire, asking again");
	if (!xmpp_stream_ready(&acct->stream))
		return;

	acct->refreshing = true;
	if (services_ask(acct))
		relays_retry(acct, "not sent");
}

/* ---------------------------------------------------------------- roster */

/**
 * roster_reply() - the answer of the roster query
 * @xs: the stream
 * @reply: the iq, NULL for none
 * @priv: the account
 */
static void roster_reply(struct xmpp_stream *xs, const struct xml_doc *reply,
			 void *priv)
{
	struct xmpp_account *acct = priv;
	struct xmpp_contact *contact;
	const char *type, *jid, *name;
	unsigned int query, node;

	if (!reply)
		return;

	type = xml_attr_get(reply, 0, "type");
	if (!type || strcmp(type, "result"))
		return;

	acct->contacts = 0;
	query = xml_child(reply, XML_NONE, "query", XMPP_NS_ROSTER);
	node = xml_child(reply, query, "item", XMPP_NS_ROSTER);
	while (node != XML_NONE && acct->contacts < XMPP_ROSTER_MAX) {
		jid = xml_attr_get(reply, node, "jid");
		name = xml_attr_get(reply, node, "name");
		if (jid && jid[0]) {
			contact = &acct->roster[acct->contacts++];
			jid_text_bare(jid, contact->jid, sizeof(contact->jid));
			str_copy(contact->name, sizeof(contact->name),
				 name ? name : "");
		}
		node = xml_next(reply, node, "item", XMPP_NS_ROSTER);
	}

	XMPP_ACCT_TRACE(acct, "roster with %u contacts", acct->contacts);
}

/**
 * roster_ask() - the roster query
 * @acct: the account
 */
static void roster_ask(struct xmpp_account *acct)
{
	char mem[XMPP_STANZA_MAX];
	char id[XMPP_ID_MAX];
	struct xml_w writer;
	size_t len;

	xmpp_stream_id_next(&acct->stream, id, sizeof(id));
	xml_w_init(&writer, mem, sizeof(mem));
	xml_w_start(&writer, "iq");
	xml_w_attr(&writer, "type", "get");
	xml_w_attr(&writer, "id", id);
	xml_w_start(&writer, "query");
	xml_w_attr(&writer, "xmlns", XMPP_NS_ROSTER);
	xml_w_end(&writer);
	xml_w_end(&writer);
	len = xml_w_finish(&writer);
	if (len)
		xmpp_stream_iq_send(&acct->stream, mem, len, id, NULL,
				    roster_reply, acct);
}

/**
 * presence_send() - the initial presence with the entity capabilities
 * @acct: the account
 */
static void presence_send(struct xmpp_account *acct)
{
	char mem[XMPP_STANZA_MAX];
	struct xml_w writer;
	size_t len;

	xml_w_init(&writer, mem, sizeof(mem));
	xml_w_start(&writer, "presence");
	disco_caps_write(&writer);
	xml_w_end(&writer);
	len = xml_w_finish(&writer);
	if (len)
		xmpp_stream_send(&acct->stream, mem, len);
}

/* ---------------------------------------------------------- the stream */

/**
 * reg_post() - tell the core the registration state of the line
 * @acct: the account
 * @up: whether the stream is ready
 */
static void reg_post(struct xmpp_account *acct, bool up)
{
	if (acct->ready_posted == up)
		return;

	acct->ready_posted = up;
	if (xa.host && xa.host->reg_state_post)
		xa.host->reg_state_post(xa.host->ctx, acct->line, up,
					acct->stream.jid.domain, false);
}

/**
 * stream_state() - a transition of the stream
 * @xs: the stream
 * @state: the new state
 * @reason: the text of a connection that ended
 */
static void stream_state(struct xmpp_stream *xs, enum xmpp_stream_state state,
			 const char *reason)
{
	struct xmpp_account *acct = xs->priv;

	if (state == XMPP_ST_READY) {
		presence_send(acct);
		roster_ask(acct);
		services_ask(acct);
		reg_post(acct, true);
		return;
	}

	/* The next connection asks the server again. */
	relays_refresh_stop(acct);
	if (acct->ready_posted) {
		xcalls_stream_down(acct->line);
		reg_post(acct, false);
	}
}

/**
 * stream_stanza() - a stanza the stream did not answer itself
 * @xs: the stream
 * @doc: the stanza
 *
 * Return: true when the legs took it.
 */
static bool stream_stanza(struct xmpp_stream *xs, const struct xml_doc *doc)
{
	struct xmpp_account *acct = xs->priv;

	if (xml_node_is(doc, 0, "iq", XMPP_NS_CLIENT))
		return xcall_jingle_rx(acct->line, doc);
	if (xml_node_is(doc, 0, "message", XMPP_NS_CLIENT))
		return xcall_jmi_rx(acct->line, doc);

	return false;
}

static const struct xmpp_stream_cb stream_cb = {
	.state = stream_state,
	.stanza = stream_stanza,
};

/**
 * dns_done() - the shared resolver answered
 * @resolver: the resolver
 * @query: the query handle
 * @owner: the owner handle
 * @res: the answer
 * @priv: unused
 */
static void dns_done(struct sip_dns *resolver, uint32_t query, uint32_t owner,
		     const struct sip_dns_result *res, void *priv)
{
	unsigned int line = (owner >> XMPP_OWNER_SHIFT) & 0xff;
	unsigned int kind = owner & ((1u << XMPP_OWNER_SHIFT) - 1);
	uint16_t gen = (uint16_t)(owner >> XMPP_OWNER_GEN_SHIFT);
	struct xmpp_account *acct = acct_get(line);

	if (!acct)
		return;

	if (kind == XMPP_OWNER_STREAM)
		xmpp_stream_dns_done(&acct->stream, query, res);
	else if (gen == acct->gen)
		service_resolved(acct, kind - 1, res);
}

/* -------------------------------------------------- the leg environment */

/**
 * relay_take() - a free slot of the iq relay table
 * @cb: the callback of the leg
 * @priv: its pointer
 *
 * Return: the slot, or NULL.
 */
static struct xmpp_relay *relay_take(xcall_reply_cb cb, void *priv)
{
	unsigned int i;

	for (i = 0; i < XMPP_RELAYS_MAX; i++) {
		if (xa.relay[i].in_use)
			continue;
		xa.relay[i].in_use = true;
		xa.relay[i].cb = cb;
		xa.relay[i].priv = priv;
		return &xa.relay[i];
	}

	return NULL;
}

/**
 * relay_reply() - hand an iq answer to the leg that sent the iq
 * @xs: the stream
 * @reply: the answer, NULL for none
 * @priv: the relay slot
 */
static void relay_reply(struct xmpp_stream *xs, const struct xml_doc *reply,
			void *priv)
{
	struct xmpp_relay *relay = priv;
	xcall_reply_cb cb = relay->cb;
	void *cb_priv = relay->priv;
	const char *type = reply ? xml_attr_get(reply, 0, "type") : NULL;

	memset(relay, 0, sizeof(*relay));
	if (cb)
		cb(cb_priv, type && !strcmp(type, "result"), reply);
}

/**
 * env_ready() - whether the stream of a line exchanges stanzas
 * @priv: unused
 * @line: the line
 *
 * Return: true in the ready state.
 */
static bool env_ready(void *priv, unsigned int line)
{
	struct xmpp_account *acct = acct_get(line);

	return acct && xmpp_stream_ready(&acct->stream);
}

/**
 * env_send() - one stanza on the stream of a line
 * @priv: unused
 * @line: the line
 * @text: the stanza
 * @len: its length
 *
 * Return: 0, or a negative errno.
 */
static int env_send(void *priv, unsigned int line, const char *text,
		    size_t len)
{
	struct xmpp_account *acct = acct_get(line);

	if (!acct)
		return -ENOTCONN;

	return xmpp_stream_send(&acct->stream, text, len);
}

/**
 * env_iq_send() - an iq of a leg, with its answer relayed back
 * @priv: unused
 * @line: the line
 * @text: the iq
 * @len: its length
 * @id: its id
 * @peer: the full JID that must answer
 * @cb: the callback of the leg
 * @cb_priv: its pointer
 *
 * Return: 0, -EBUSY when the relay table is full, or the stream's errno.
 */
static int env_iq_send(void *priv, unsigned int line, const char *text,
		       size_t len, const char *id, const char *peer,
		       xcall_reply_cb cb, void *cb_priv)
{
	struct xmpp_account *acct = acct_get(line);
	struct xmpp_relay *relay;
	int ret;

	if (!acct)
		return -ENOTCONN;

	relay = relay_take(cb, cb_priv);
	if (!relay)
		return -EBUSY;

	ret = xmpp_stream_iq_send(&acct->stream, text, len, id, peer,
				  relay_reply, relay);
	if (ret)
		memset(relay, 0, sizeof(*relay));

	return ret;
}

/**
 * env_reply_result() - the empty result of an iq a leg took
 * @priv: unused
 * @line: the line
 * @iq: the iq
 *
 * Return: 0, or a negative errno.
 */
static int env_reply_result(void *priv, unsigned int line,
			    const struct xml_doc *iq)
{
	struct xmpp_account *acct = acct_get(line);

	if (!acct)
		return -ENOTCONN;

	return xmpp_stream_iq_result(&acct->stream, iq);
}

/**
 * env_reply_error() - the stanza error of an iq a leg refused
 * @priv: unused
 * @line: the line
 * @iq: the iq
 * @type: the error type
 * @condition: the condition element
 *
 * Return: 0, or a negative errno.
 */
static int env_reply_error(void *priv, unsigned int line,
			   const struct xml_doc *iq, const char *type,
			   const char *condition)
{
	struct xmpp_account *acct = acct_get(line);

	if (!acct)
		return -ENOTCONN;

	return xmpp_stream_iq_error(&acct->stream, iq, type, condition);
}

/**
 * env_id_next() - a fresh stanza id of the stream of a line
 * @priv: unused
 * @line: the line
 * @out: receives the id
 * @len: capacity of @out
 */
static void env_id_next(void *priv, unsigned int line, char *out, size_t len)
{
	struct xmpp_account *acct = acct_get(line);

	if (!acct) {
		snprintf(out, len, "x%u", line);
		return;
	}

	xmpp_stream_id_next(&acct->stream, out, len);
}

/**
 * env_self_jid() - the full JID the stream of a line bound
 * @priv: unused
 * @line: the line
 * @out: receives it
 *
 * Return: true once the resource is bound.
 */
static bool env_self_jid(void *priv, unsigned int line, struct jid *out)
{
	struct xmpp_account *acct = acct_get(line);
	const struct jid *bound;

	if (!acct)
		return false;

	bound = xmpp_stream_jid(&acct->stream);
	if (!bound->resource[0])
		return false;

	*out = *bound;

	return true;
}

/**
 * env_local_addr() - the address of the voice interface the media binds to
 * @priv: unused
 * @line: unused, every line shares the interface
 * @out: receives the address text
 * @len: capacity of @out
 *
 * Return: true when the tracker knows an address.
 */
static bool env_local_addr(void *priv, unsigned int line, char *out,
			   size_t len)
{
	const struct net_iface_state *state;

	out[0] = 0;
	if (!xa.host || !xa.host->net_iface_state_get)
		return false;

	state = xa.host->net_iface_state_get(xa.host->ctx);
	if (!state)
		return false;

	if (state->addr4_known) {
		str_copy(out, len, state->addr4);
		return true;
	}
	if (state->addr6_known && state->ipv6_served) {
		str_copy(out, len, state->addr6);
		return true;
	}

	return false;
}

/**
 * env_bind_dev() - the netdev the media of a line scopes to
 * @priv: unused
 * @line: the line
 *
 * Return: the netdev, "" for none.
 */
static const char *env_bind_dev(void *priv, unsigned int line)
{
	struct xmpp_account *acct = acct_get(line);
	const char *dev;

	if (!acct || !xa.host || !xa.host->net_iface_bind_device)
		return "";

	dev = xa.host->net_iface_bind_device(xa.host->ctx, acct->iface);

	return dev ? dev : "";
}

/**
 * env_contact_name() - the roster name of a bare JID
 * @priv: unused
 * @line: the line
 * @bare: the bare JID
 * @out: receives the name
 * @len: capacity of @out
 *
 * Return: true when the roster names the contact.
 */
static bool env_contact_name(void *priv, unsigned int line, const char *bare,
			     char *out, size_t len)
{
	struct xmpp_account *acct = acct_get(line);
	unsigned int i;

	if (!acct)
		return false;

	for (i = 0; i < acct->contacts; i++) {
		if (strcmp(acct->roster[i].jid, bare) ||
		    !acct->roster[i].name[0])
			continue;
		str_copy(out, len, acct->roster[i].name);
		return true;
	}

	return false;
}

/**
 * env_speed_dial() - the JID a dialed digit string stands for
 * @priv: unused
 * @line: the line
 * @digits: the digits
 * @out: receives the JID
 * @len: capacity of @out
 *
 * Return: true when a speed dial entry "digits=jid" matched.
 */
static bool env_speed_dial(void *priv, unsigned int line, const char *digits,
			   char *out, size_t len)
{
	struct xmpp_account *acct = acct_get(line);
	const char *entry, *eq;
	size_t dlen = strlen(digits);
	unsigned int i;

	if (!acct)
		return false;

	for (i = 0; i < CFG_XMPP_SPEED_DIAL_MAX; i++) {
		entry = acct->ch.xmpp_speed_dial[i];
		eq = strchr(entry, '=');
		if (!eq || (size_t)(eq - entry) != dlen ||
		    strncmp(entry, digits, dlen) || !eq[1])
			continue;
		str_copy(out, len, eq + 1);
		return true;
	}

	return false;
}

/**
 * env_ice_servers() - the resolved relays of a line, into a transport
 * @priv: unused
 * @line: the line
 * @out: the transport block, its STUN host and TURN rows filled
 */
static void env_ice_servers(void *priv, unsigned int line,
			    struct hal_transport *out)
{
	struct xmpp_account *acct = acct_get(line);
	const struct xmpp_service *svc;
	struct hal_turn *turn;
	uint64_t now = now_ms();
	unsigned int i;

	if (!acct)
		return;

	for (i = 0; i < acct->services; i++) {
		svc = &acct->service[i];
		if (!svc->resolved || (svc->expires_ms && now >= svc->expires_ms))
			continue;
		if (!svc->turn) {
			if (out->stun_host[0])
				continue;
			str_copy(out->stun_host, sizeof(out->stun_host),
				 svc->addr);
			out->stun_port = svc->port;
			continue;
		}
		if (out->turns >= HAL_TURN_MAX)
			continue;
		turn = &out->turn[out->turns++];
		str_copy(turn->host, sizeof(turn->host), svc->addr);
		str_copy(turn->user, sizeof(turn->user), svc->user);
		str_copy(turn->pass, sizeof(turn->pass), svc->pass);
		turn->port = svc->port;
	}
}

/**
 * env_relay_missing() - a call failed its checks with no relay to fall to
 * @priv: unused
 * @line: the line
 */
static void env_relay_missing(void *priv, unsigned int line)
{
	if (!xa.host)
		return;

	if (xa.host->line_fault)
		xa.host->line_fault(xa.host->ctx, line, true, XMPP_FAULT_TEXT);
	if (xa.host->log)
		xa.host->log(xa.host->ctx, LOG_WARNING, "xmpp line %u: %s",
			     line, XMPP_FAULT_TEXT);
}

/**
 * env_dnd() - whether do-not-disturb holds a line
 * @priv: unused
 * @line: the line
 *
 * Return: what the shell answers.
 */
static bool env_dnd(void *priv, unsigned int line)
{
	return xa.dnd && xa.dnd(line);
}

/**
 * env_legs_held() - whether a line drives the Jingle legs
 * @priv: unused
 * @line: the line
 *
 * Return: what the shell answers.
 */
static bool env_legs_held(void *priv, unsigned int line)
{
	return xa.legs_held && xa.legs_held(line, &xcalls_leg_ops);
}

/* ---------------------------------------------------------------- build */

/**
 * stream_cfg_fill() - the stream configuration of a channel
 * @ch: the channel
 * @out: receives it
 */
static void stream_cfg_fill(const struct cfg_channel *ch,
			    struct xmpp_stream_cfg *out)
{
	memset(out, 0, sizeof(*out));
	if (ch->xmpp_resource[0] && !strchr(ch->xmpp_jid, '/'))
		snprintf(out->jid, sizeof(out->jid), "%s/%s", ch->xmpp_jid,
			 ch->xmpp_resource);
	else
		str_copy(out->jid, sizeof(out->jid), ch->xmpp_jid);
	str_copy(out->password, sizeof(out->password), ch->xmpp_password);
	str_copy(out->host, sizeof(out->host), ch->xmpp_host);
	out->port = (uint16_t)ch->xmpp_port;
	out->direct_tls = ch->xmpp_direct_tls;
	out->tls_verify = ch->xmpp_tls_verify;
	out->allow_plain = ch->xmpp_allow_plain;
}

/**
 * account_build() - start the account of a line
 * @acct: the account
 * @cfg: the configuration
 * @line: the line
 */
static void account_build(struct xmpp_account *acct, const struct cfg *cfg,
			  unsigned int line)
{
	const struct cfg_channel *ch = &cfg->channel[line];
	struct sip_tls_cfg tls = {
		.ca_path = ch->xmpp_tls_ca[0] ? ch->xmpp_tls_ca : NULL,
		.verify = ch->xmpp_tls_verify,
	};
	struct xmpp_stream_env env = {
		.host = xa.host,
		.dns = &xa.dns,
		.dns_owner = owner_make(line, XMPP_OWNER_STREAM, 0),
	};
	struct xmpp_stream_cfg scfg;
	int ret;

	memset(acct, 0, sizeof(*acct));
	acct->line = line;
	acct->gen = ++xa.gen;
	acct->ch = *ch;
	proto_timer_init(&acct->refresh, acct, XMPP_TIMER_REFRESH,
			 relays_refresh_fire);
	str_copy(acct->iface, sizeof(acct->iface),
		 ch->local_interface[0] ? ch->local_interface :
					  cfg->local_interface);
	jid_text_bare(ch->xmpp_jid, acct->label, sizeof(acct->label));

	sip_tls_init(&acct->tls);
	if (!sip_tls_ctx_load(&acct->tls, &tls)) {
		if (xa.host && xa.host->log)
			xa.host->log(xa.host->ctx, LOG_ERR,
				     "xmpp line %u: tls: %s", line,
				     sip_tls_error(&acct->tls));
		return;
	}
	env.tls = &acct->tls;

	xmpp_stream_init(&acct->stream, &stream_cb, acct, &env);
	stream_cfg_fill(ch, &scfg);
	acct->active = true;

	ret = xmpp_stream_start(&acct->stream, &scfg);
	if (ret) {
		if (xa.host && xa.host->log)
			xa.host->log(xa.host->ctx, LOG_ERR,
				     "xmpp line %u: the jid or the password is "
				     "unusable (%d)", line, ret);
		account_teardown(acct);
		return;
	}

	services_configured(acct);
}

/**
 * account_teardown() - end the account of a line
 * @acct: the account
 */
static void account_teardown(struct xmpp_account *acct)
{
	if (!acct->active)
		return;

	xcalls_line_teardown(acct->line);
	services_cancel(acct);
	/* The iq flush of the stream answers a refresh in flight, and that
	 * answer must not plan a retry. */
	relays_refresh_stop(acct);
	xmpp_stream_done(&acct->stream);
	reg_post(acct, false);
	sip_tls_done(&acct->tls);
	acct->active = false;
	memset(acct, 0, sizeof(*acct));
}

/**
 * stream_options_moved() - whether a channel change rebuilds the stream
 * @old_ch: the channel as built
 * @new_ch: the channel now
 *
 * Return: true when an option the stream or its TLS context read moved.
 */
static bool stream_options_moved(const struct cfg_channel *old_ch,
				 const struct cfg_channel *new_ch)
{
	return strcmp(old_ch->xmpp_jid, new_ch->xmpp_jid) ||
	       strcmp(old_ch->xmpp_password, new_ch->xmpp_password) ||
	       strcmp(old_ch->xmpp_resource, new_ch->xmpp_resource) ||
	       strcmp(old_ch->xmpp_host, new_ch->xmpp_host) ||
	       strcmp(old_ch->xmpp_tls_ca, new_ch->xmpp_tls_ca) ||
	       strcmp(old_ch->local_interface, new_ch->local_interface) ||
	       old_ch->xmpp_port != new_ch->xmpp_port ||
	       old_ch->xmpp_direct_tls != new_ch->xmpp_direct_tls ||
	       old_ch->xmpp_tls_verify != new_ch->xmpp_tls_verify ||
	       old_ch->xmpp_allow_plain != new_ch->xmpp_allow_plain;
}

/* -------------------------------------------------------------- lifecycle */

void xmpp_accounts_init(const struct proto_host *host, const struct call_cb *cb,
			bool (*dnd)(unsigned int line),
			bool (*legs_held)(unsigned int line,
					  const struct call_leg_ops *legs))
{
	struct sip_dns_cb dcb = { .done = dns_done };
	struct sip_dns_cfg dcfg;
	struct xcall_env env = {
		.timer = &xa.timer_env,
		.media = host ? &host->media : NULL,
		.cb = cb,
		.host = host,
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

	memset(&xa, 0, sizeof(xa));
	xa.host = host;
	xa.dnd = dnd;
	xa.legs_held = legs_held;
	proto_timer_env_uloop_init(&xa.timer_env);

	sip_dns_cfg_default(&dcfg);
	dcfg.srv_fallback = true;
	dcfg.trace_host = host;
	dcfg.trace_dom = TRACE_XMPP;
	sip_dns_init(&xa.dns, &dcfg, &dcb, NULL);

	xcalls_init(&env);
	xa.inited = true;
}

void xmpp_accounts_start(const struct cfg *cfg)
{
	unsigned int i;

	if (!xa.inited)
		return;

	xa.cfg = cfg;
	xcalls_reload(cfg);
	sip_dns_ipv6_set(&xa.dns, cfg->ipv6_enabled);

	for (i = 0; i < CFG_CHANNEL_MAX && i < cfg->channels; i++) {
		if (channel_xmpp(cfg, &cfg->channel[i]))
			account_build(&xa.acct[i], cfg, i);
	}

	xa.started = true;
}

void xmpp_accounts_reload(const struct cfg *old_cfg, const struct cfg *new_cfg)
{
	struct xmpp_account *acct;
	bool want, rebuild;
	unsigned int i;

	if (!xa.inited || !xa.started)
		return;

	xa.cfg = new_cfg;
	xcalls_reload(new_cfg);
	sip_dns_ipv6_set(&xa.dns, new_cfg->ipv6_enabled);

	for (i = 0; i < CFG_CHANNEL_MAX; i++) {
		acct = &xa.acct[i];
		want = i < new_cfg->channels &&
		       channel_xmpp(new_cfg, &new_cfg->channel[i]);
		rebuild = acct->active && want &&
			  (stream_options_moved(&acct->ch,
						&new_cfg->channel[i]) ||
			   strcmp(old_cfg->local_interface,
				  new_cfg->local_interface));

		if (acct->active && (!want || rebuild))
			account_teardown(acct);
		if (want && !acct->active) {
			account_build(acct, new_cfg, i);
			continue;
		}
		if (!acct->active)
			continue;

		acct->ch = new_cfg->channel[i];
		services_cancel(acct);
		relays_refresh_stop(acct);
		acct->gen = ++xa.gen;
		acct->services = 0;
		services_configured(acct);
		if (xmpp_stream_ready(&acct->stream))
			services_ask(acct);
	}
}

void xmpp_accounts_hal_set(uint32_t features, uint32_t codecs)
{
	xcalls_hal_features_set(features);
	xcalls_hal_codecs_set(codecs);
}

void xmpp_accounts_shutdown_begin(void)
{
	unsigned int i;

	for (i = 0; i < CFG_CHANNEL_MAX; i++)
		account_teardown(&xa.acct[i]);
}

void xmpp_accounts_done(void)
{
	if (!xa.inited)
		return;

	xmpp_accounts_shutdown_begin();
	xcalls_done();
	sip_dns_done(&xa.dns);
	xa.inited = false;
	xa.started = false;
}

bool xmpp_account_active(unsigned int line)
{
	return acct_get(line) != NULL;
}

bool xmpp_account_ready(unsigned int line)
{
	struct xmpp_account *acct = acct_get(line);

	return acct && xmpp_stream_ready(&acct->stream);
}

const char *xmpp_account_label(unsigned int line)
{
	struct xmpp_account *acct = acct_get(line);

	return acct ? acct->label : "";
}

/* ----------------------------------------------------------------- dumps */

/**
 * account_dump() - one account for the status verb
 * @acct: the account
 * @b: the blob buffer
 */
static void account_dump(const struct xmpp_account *acct, struct blob_buf *b)
{
	const struct xmpp_stream_stat *stat = xmpp_stream_stat(&acct->stream);
	const struct xmpp_service *svc;
	char jid[JID_MAX];
	void *tbl, *arr, *row;
	unsigned int i;

	tbl = blobmsg_open_table(b, NULL);
	blobmsg_add_u32(b, "line", acct->line);
	blobmsg_add_string(b, "jid", acct->label);
	jid_full(xmpp_stream_jid(&acct->stream), jid, sizeof(jid));
	blobmsg_add_string(b, "bound", jid);
	blobmsg_add_string(b, "state",
			   xmpp_stream_state_name(acct->stream.state));
	blobmsg_add_string(b, "reason", xmpp_stream_reason(&acct->stream));
	blobmsg_add_u8(b, "tls", acct->stream.tls_up);
	blobmsg_add_u8(b, "sm", acct->stream.sm.enabled);
	blobmsg_add_u8(b, "carbons", acct->stream.carbons);
	blobmsg_add_u32(b, "contacts", acct->contacts);
	blobmsg_add_u64(b, "connects", stat->connects);
	blobmsg_add_u64(b, "failures", stat->failures);
	blobmsg_add_u64(b, "rx_stanzas", stat->rx_stanzas);
	blobmsg_add_u64(b, "tx_stanzas", stat->tx_stanzas);

	arr = blobmsg_open_array(b, "relays");
	for (i = 0; i < acct->services; i++) {
		svc = &acct->service[i];
		if (!svc->host[0])
			continue;
		row = blobmsg_open_table(b, NULL);
		blobmsg_add_string(b, "type", svc->turn ? XMPP_SERVICE_TURN :
							  XMPP_SERVICE_STUN);
		blobmsg_add_string(b, "host", svc->host);
		blobmsg_add_u32(b, "port", svc->port);
		blobmsg_add_string(b, "address", svc->addr);
		blobmsg_close_table(b, row);
	}
	blobmsg_close_array(b, arr);

	xcalls_status_dump(b, acct->line);
	blobmsg_close_table(b, tbl);
}

void xmpp_accounts_status_dump(struct blob_buf *b, int want_line)
{
	unsigned int i;
	void *arr;

	arr = blobmsg_open_array(b, "xmpp");
	for (i = 0; i < CFG_CHANNEL_MAX; i++) {
		if (!xa.acct[i].active)
			continue;
		if (want_line >= 0 && (unsigned int)want_line != i)
			continue;
		account_dump(&xa.acct[i], b);
	}
	blobmsg_close_array(b, arr);
}

void xmpp_roster_dump(struct blob_buf *b)
{
	const struct xmpp_account *acct;
	unsigned int i, j;
	void *arr, *tbl;

	arr = blobmsg_open_array(b, "roster");
	for (i = 0; i < CFG_CHANNEL_MAX; i++) {
		acct = &xa.acct[i];
		if (!acct->active)
			continue;
		for (j = 0; j < acct->contacts; j++) {
			tbl = blobmsg_open_table(b, NULL);
			blobmsg_add_u32(b, "line", i);
			blobmsg_add_string(b, "jid", acct->roster[j].jid);
			blobmsg_add_string(b, "name", acct->roster[j].name);
			blobmsg_close_table(b, tbl);
		}
	}
	blobmsg_close_array(b, arr);
}
