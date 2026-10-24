// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * The writer of the uvoice nft sets. See net/peer.h for the contract.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <libubox/uloop.h>

#include "net/conntrack.h"
#include "net/peer.h"
#include "posix.h"
#include "spawn_run.h"
#include "str.h"
#include "trace.h"

#define NET_PEER_NFT		"/usr/sbin/nft"
#define NET_PEER_SNIPPET	"/usr/share/nftables.d/table-pre/20-uvoice.nft"
#define NET_PEER_PATH_MAX	128

/* IFNAMSIZ: the width of the ifname part of a set key */
#define NET_PEER_DEV_MAX	16
#define NET_PEER_HOST_MAX	INET6_ADDRSTRLEN

/* nft add element inet fw4 <set> { <dev> . <host> . <port> timeout <t>
 * expires <t> }, and the NULL the spawn needs */
#define NET_PEER_ARGV_MAX	18

/* uloop_timeout_set() takes an int of milliseconds */
#define NET_PEER_TICK_MAX_S	86400

enum peer_set {
	PEER_SET_V4 = 0,
	PEER_SET_V6,
};

static const char *const peer_set_name[] = {
	[PEER_SET_V4] = "uvoice_peer4",
	[PEER_SET_V6] = "uvoice_peer6",
};

struct peer_key {
	enum peer_set set;
	char dev[NET_PEER_DEV_MAX];
	char host[NET_PEER_HOST_MAX];
	uint16_t port;
};

struct peer_req {
	struct peer_key key;
	unsigned int timeout_s;
};

/*
 * The last successful write of a key, on the monotonic clock: when it ended,
 * and that end plus its timeout. nft sets the expiry while it runs, so a
 * deadline counted from the end is never earlier than the kernel's.
 */
struct peer_written {
	struct peer_key key;
	uint64_t deadline_ms;
	uint64_t at_ms;
	bool used;
};

struct peer_hold {
	struct peer_key key;
	bool used;
};

struct peer_argv {
	char path[NET_PEER_PATH_MAX];
	char dev[NET_PEER_DEV_MAX + 2];
	char port[8];
	char timeout[16];
	char *argv[NET_PEER_ARGV_MAX];
};

static struct {
	struct peer_req queue[NET_PEER_QUEUE_MAX];
	unsigned int queued;
	struct peer_written written[NET_PEER_QUEUE_MAX];
	struct peer_hold hold[NET_PEER_HOLD_MAX];
	struct uloop_timeout start;
	struct uloop_timeout tick;
	struct uloop_process proc;
	struct peer_req cur;
	uint64_t cur_deadline_ms;
	bool busy;
	bool failing;
} peer;

static bool peer_key_eq(const struct peer_key *a, const struct peer_key *b)
{
	return a->set == b->set && a->port == b->port &&
	       !strcmp(a->dev, b->dev) && !strcmp(a->host, b->host);
}

/*
 * The host must parse as a literal of its family. nft joins its arguments and
 * parses them again, so any other text could carry tokens of its own.
 */
static bool peer_key_make(struct peer_key *key, const char *dev,
			  const char *host, bool ipv6, uint16_t port)
{
	struct sockaddr_storage sa;

	memset(key, 0, sizeof(*key));

	/* a double quote would end the quoted device token early */
	if (!dev || !*dev || strlen(dev) >= sizeof(key->dev) ||
	    strchr(dev, '"')) {
		TRACE(TRACE_FSM, "firewall: no usable device, no set element");
		return false;
	}

	if (!port) {
		TRACE(TRACE_FSM, "firewall: port 0 on %s, no set element", dev);
		return false;
	}

	if (!host || !*host || !addr_pton(ipv6 ? AF_INET6 : AF_INET, host, &sa)) {
		TRACE(TRACE_FSM, "firewall: '%s' is no %s literal, "
		      "no set element", host ? host : "(none)",
		      ipv6 ? "IPv6" : "IPv4");
		return false;
	}

	key->set = ipv6 ? PEER_SET_V6 : PEER_SET_V4;
	key->port = port;
	str_copy(key->dev, sizeof(key->dev), dev);
	str_copy(key->host, sizeof(key->host), host);

	return true;
}

static void peer_nft_path(char *out, size_t olen)
{
	const char *path = getenv("UVOICE_NFT");

	snprintf(out, olen, "%s", (path && *path) ? path : NET_PEER_NFT);
}

static bool peer_nft_ready(void)
{
	char path[NET_PEER_PATH_MAX];

	peer_nft_path(path, sizeof(path));
	if (!access(path, X_OK))
		return true;

	TRACE(TRACE_FSM, "firewall: %s absent, no set element", path);

	return false;
}

static uint64_t peer_deadline(unsigned int timeout_s)
{
	return mono_ms() + (uint64_t)timeout_s * 1000;
}

static struct peer_written *peer_written_find(const struct peer_key *key)
{
	unsigned int i;

	for (i = 0; i < NET_PEER_QUEUE_MAX; i++) {
		struct peer_written *w = &peer.written[i];

		if (w->used && peer_key_eq(&w->key, key))
			return w;
	}

	return NULL;
}

/* the record of key, else a free one, else the one written longest ago */
static struct peer_written *peer_written_slot(const struct peer_key *key)
{
	struct peer_written *w = peer_written_find(key);
	unsigned int i, oldest = 0;

	if (w)
		return w;

	for (i = 0; i < NET_PEER_QUEUE_MAX; i++) {
		if (!peer.written[i].used)
			return &peer.written[i];
		if (peer.written[i].at_ms < peer.written[oldest].at_ms)
			oldest = i;
	}

	return &peer.written[oldest];
}

/*
 * An add with "expires" replaces the expiry of an existing element, so a
 * write whose deadline falls before the last one, or before the one the
 * running child writes, would end the element sooner.
 */
static bool peer_shortens(const struct peer_key *key, uint64_t deadline)
{
	const struct peer_written *w = peer_written_find(key);

	if (w && w->deadline_ms > deadline)
		return true;

	return peer.busy && peer_key_eq(&peer.cur.key, key) &&
	       peer.cur_deadline_ms > deadline;
}

static struct peer_req *peer_queue_find(const struct peer_key *key)
{
	unsigned int i;

	for (i = 0; i < peer.queued; i++) {
		if (peer_key_eq(&peer.queue[i].key, key))
			return &peer.queue[i];
	}

	return NULL;
}

static void peer_start_cb(struct uloop_timeout *t);

static bool peer_enqueue(const struct peer_key *key, unsigned int timeout_s)
{
	struct peer_req *q;

	if (peer_shortens(key, peer_deadline(timeout_s))) {
		TRACE(TRACE_FSM, "firewall: %s element of %s port %u lives "
		      "longer, kept", peer_set_name[key->set], key->dev,
		      key->port);
		return true;
	}

	q = peer_queue_find(key);
	if (q) {
		if (timeout_s > q->timeout_s)
			q->timeout_s = timeout_s;
		return true;
	}

	if (peer.queued >= NET_PEER_QUEUE_MAX) {
		TRACE(TRACE_FSM, "firewall: queue full, %s element of %s "
		      "dropped", peer_set_name[key->set], key->dev);
		return false;
	}

	q = &peer.queue[peer.queued++];
	q->key = *key;
	q->timeout_s = timeout_s;

	/* from the loop, so the requests of one frame share the queue */
	if (!peer.busy) {
		peer.start.cb = peer_start_cb;
		uloop_timeout_set(&peer.start, 0);
	}

	return true;
}

static void peer_argv_build(const struct peer_req *r, struct peer_argv *a)
{
	const struct peer_key *k = &r->key;
	unsigned int n = 0;

	peer_nft_path(a->path, sizeof(a->path));
	snprintf(a->dev, sizeof(a->dev), "\"%s\"", k->dev);
	snprintf(a->port, sizeof(a->port), "%u", k->port);
	snprintf(a->timeout, sizeof(a->timeout), "%us", r->timeout_s);

	a->argv[n++] = a->path;
	a->argv[n++] = (char *)"add";
	a->argv[n++] = (char *)"element";
	a->argv[n++] = (char *)"inet";
	a->argv[n++] = (char *)"fw4";
	a->argv[n++] = (char *)peer_set_name[k->set];
	a->argv[n++] = (char *)"{";
	a->argv[n++] = a->dev;
	a->argv[n++] = (char *)".";
	a->argv[n++] = (char *)k->host;
	a->argv[n++] = (char *)".";
	a->argv[n++] = a->port;
	a->argv[n++] = (char *)"timeout";
	a->argv[n++] = a->timeout;
	a->argv[n++] = (char *)"expires";
	a->argv[n++] = a->timeout;
	a->argv[n++] = (char *)"}";
	a->argv[n] = NULL;
}

/*
 * A failed write most often meets a set that fw4 never loaded, so the first
 * failure of a run names the snippet and the rest of the run traces only.
 */
static void peer_fail(const struct peer_req *r, const char *what)
{
	if (peer.failing) {
		TRACE(TRACE_FSM, "firewall: nft add element %s failed (%s)",
		      peer_set_name[r->key.set], what);
		return;
	}

	peer.failing = true;
	ULOG_WARN("firewall: nft add element %s failed (%s), check that fw4 "
		  "loaded %s\n", peer_set_name[r->key.set], what,
		  NET_PEER_SNIPPET);
}

static void peer_status_text(int status, char *out, size_t olen)
{
	if (WIFEXITED(status))
		snprintf(out, olen, "exit %d", WEXITSTATUS(status));
	else
		snprintf(out, olen, "signal %d", WTERMSIG(status));
}

static void peer_complete(int status)
{
	struct peer_written *w;
	char what[32];

	if (!WIFEXITED(status) || WEXITSTATUS(status)) {
		peer_status_text(status, what, sizeof(what));
		peer_fail(&peer.cur, what);
		return;
	}

	w = peer_written_slot(&peer.cur.key);
	w->key = peer.cur.key;
	w->deadline_ms = peer_deadline(peer.cur.timeout_s);
	w->at_ms = mono_ms();
	w->used = true;

	if (!peer.failing)
		return;

	peer.failing = false;
	ULOG_INFO("firewall: nft add element works again\n");
}

static void peer_run(void);

static void peer_proc_cb(struct uloop_process *p, int ret)
{
	peer.busy = false;
	peer_complete(ret);
	peer_run();
}

/* Leaves peer.busy set while the child runs, clear when it ended here. */
static void peer_spawn(void)
{
	const struct peer_key *k = &peer.cur.key;
	struct peer_argv a;
	char what[64];
	int status;
	pid_t pid;

	peer_argv_build(&peer.cur, &a);
	TRACE(TRACE_FSM, "firewall: %s element %s %s port %u for %u s",
	      peer_set_name[k->set], k->dev, k->host, k->port,
	      peer.cur.timeout_s);

	pid = spawn_run(a.argv, SPAWN_GROUP_DAEMON);
	if (pid < 0) {
		snprintf(what, sizeof(what), "spawn: %s", strerror(errno));
		peer_fail(&peer.cur, what);
		return;
	}

	peer.proc.pid = pid;
	peer.proc.cb = peer_proc_cb;
	if (!uloop_process_add(&peer.proc)) {
		peer.busy = true;
		return;
	}

	if (waitpid(pid, &status, 0) == pid)
		peer_complete(status);
}

static void peer_run(void)
{
	while (!peer.busy && peer.queued) {
		peer.cur = peer.queue[0];
		peer.queued--;
		memmove(&peer.queue[0], &peer.queue[1],
			peer.queued * sizeof(peer.queue[0]));

		/* a write that ended after the request was queued may cover
		 * this key for longer now */
		peer.cur_deadline_ms = peer_deadline(peer.cur.timeout_s);
		if (peer_shortens(&peer.cur.key, peer.cur_deadline_ms))
			continue;

		peer_spawn();
	}
}

static void peer_start_cb(struct uloop_timeout *t)
{
	peer_run();
}

bool net_peer_admit(const char *dev, const char *host, bool ipv6,
		    uint16_t port, unsigned int timeout_s)
{
	struct peer_key key;

	if (!peer_key_make(&key, dev, host, ipv6, port))
		return false;

	if (!timeout_s) {
		TRACE(TRACE_FSM, "firewall: timeout 0 on %s, no set element",
		      dev);
		return false;
	}

	if (!peer_nft_ready())
		return false;

	return peer_enqueue(&key, timeout_s);
}

static bool peer_holds_any(void)
{
	unsigned int i;

	for (i = 0; i < NET_PEER_HOLD_MAX; i++) {
		if (peer.hold[i].used)
			return true;
	}

	return false;
}

static void peer_holds_write(void)
{
	unsigned int bound = net_ct_udp_bound_s();
	unsigned int i;

	if (!peer_nft_ready())
		return;

	for (i = 0; i < NET_PEER_HOLD_MAX; i++) {
		if (peer.hold[i].used)
			peer_enqueue(&peer.hold[i].key, bound);
	}
}

static void peer_tick_cb(struct uloop_timeout *t);

/* the bound is read again at every arm: /etc/config/conntrack can move it */
static void peer_tick_arm(void)
{
	unsigned int bound = net_ct_udp_bound_s();

	if (bound > NET_PEER_TICK_MAX_S)
		bound = NET_PEER_TICK_MAX_S;

	peer.tick.cb = peer_tick_cb;
	uloop_timeout_set(&peer.tick, (int)(bound * 500));
}

static void peer_tick_cb(struct uloop_timeout *t)
{
	peer_holds_write();
	if (peer_holds_any())
		peer_tick_arm();
}

static int peer_hold_free(void)
{
	int i;

	for (i = 0; i < NET_PEER_HOLD_MAX; i++) {
		if (!peer.hold[i].used)
			return i;
	}

	return -1;
}

int net_peer_hold(const char *dev, const char *host, bool ipv6, uint16_t port)
{
	struct peer_key key;
	int i;

	if (!peer_key_make(&key, dev, host, ipv6, port))
		return -1;

	i = peer_hold_free();
	if (i < 0) {
		TRACE(TRACE_FSM, "firewall: hold table full, %s element of %s "
		      "refused", peer_set_name[key.set], key.dev);
		return -1;
	}

	peer.hold[i].key = key;
	peer.hold[i].used = true;
	if (peer_nft_ready())
		peer_enqueue(&key, net_ct_udp_bound_s());
	if (!peer.tick.pending)
		peer_tick_arm();

	return i;
}

void net_peer_release(int handle)
{
	if (handle < 0 || handle >= NET_PEER_HOLD_MAX)
		return;

	peer.hold[handle].used = false;
	if (!peer_holds_any())
		uloop_timeout_cancel(&peer.tick);
}

void net_peer_done(void)
{
	uloop_timeout_cancel(&peer.start);
	uloop_timeout_cancel(&peer.tick);
	peer.queued = 0;
	memset(peer.hold, 0, sizeof(peer.hold));

	if (!peer.busy)
		return;

	uloop_process_delete(&peer.proc);
	waitpid(peer.proc.pid, NULL, 0);
	peer.busy = false;
}

unsigned int net_peer_test_queued(void)
{
	return peer.queued;
}

bool net_peer_test_busy(void)
{
	return peer.busy;
}

pid_t net_peer_test_pid(void)
{
	return peer.busy ? peer.proc.pid : -1;
}

bool net_peer_test_timer_armed(void)
{
	return peer.tick.pending || peer.start.pending;
}

int64_t net_peer_test_tick_ms(void)
{
	return uloop_timeout_remaining64(&peer.tick);
}
