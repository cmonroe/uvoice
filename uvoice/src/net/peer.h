// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * The timed elements of the uvoice nft sets (src/net/peer.c). The package
 * snippets declare two sets in table inet fw4 and the input rules that
 * accept what they hold:
 *
 *   uvoice_peer4   device . peer IPv4 address . local SIP port;
 *   uvoice_peer6   device . peer IPv6 address . local SIP port.
 *
 * Each element names the netdev of the SIP socket. The rules sit in the input
 * chain ahead of every zone jump, so an element must admit traffic to that
 * socket alone. Every element carries a timeout, so an element that nobody
 * writes again leaves the set by itself. uvoiced never edits uci and never
 * reloads fw4.
 *
 * One writer. The layer runs nft as a child, one at a time, from a queue. A
 * request merges with a queued request for the same key, and the queued
 * request keeps the longer timeout. A request for the key of the running
 * child queues behind it. A request after the write completed writes again,
 * with its own timeout, so an element carries the timeout of its latest
 * write. An add with "expires" replaces the expiry of an existing element, so
 * a requester must write again within its own timeout. The first child starts
 * from the loop, never inside the caller.
 *
 * Standing elements. A hold is an element that this layer writes again every
 * half conntrack bound until its owner releases it, with the bound as its
 * timeout. A protocol with no round of its own (MGCP) holds its elements
 * here.
 *
 * The nft binary is /usr/sbin/nft, and UVOICE_NFT names another path. A path
 * that is not executable refuses each admit with a trace. A hold stays, and
 * each tick checks the path again.
 */
#ifndef UVOICE_NET_PEER_H
#define UVOICE_NET_PEER_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#define NET_PEER_QUEUE_MAX	64
#define NET_PEER_HOLD_MAX	16

/*
 * Queue one timed element. dev is the netdev of the socket, host an IPv4 or
 * IPv6 literal of the family ipv6 names, port the local SIP port. The caller
 * must request the element again within timeout_s. Returns false when it does
 * not queue the request: no usable device (absent, too long, or with a double
 * quote), no host or a host that is not a literal of its family, port or
 * timeout 0, nft absent, or a full queue.
 */
bool net_peer_admit(const char *dev, const char *host, bool ipv6,
		    uint16_t port, unsigned int timeout_s);

/*
 * A standing element, queued at once and then every half conntrack bound
 * until it is released. The arguments are those of net_peer_admit(). Returns
 * a handle, or -1 for invalid arguments or a full table. An absent nft does
 * not refuse a hold: the next tick tries again.
 */
int net_peer_hold(const char *dev, const char *host, bool ipv6, uint16_t port);

/* Stop writing the element of handle. The element times out by itself. */
void net_peer_release(int handle);

/*
 * Drop all queued requests and holds, stop both timers and wait for a running
 * child. proto_done() calls it.
 */
void net_peer_done(void);

/*
 * Test hooks, never used by the daemon: queued request count, whether a child
 * runs, its pid (-1 when none), whether the hold tick or the queue start is
 * armed, and milliseconds left on the hold tick (-1 when not armed).
 */
unsigned int net_peer_test_queued(void);
bool net_peer_test_busy(void);
pid_t net_peer_test_pid(void);
bool net_peer_test_timer_armed(void);
int64_t net_peer_test_tick_ms(void);

#endif
