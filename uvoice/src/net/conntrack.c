// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 */
#include <stdio.h>

#include "net/conntrack.h"

/*
 * Conntrack udp timeouts, read from procfs because /etc/config/conntrack can
 * override both. The defaults are the values that the box ships. The kernel
 * gives the stream timeout only to an entry that saw a reply more than 2 s
 * after creation. A fast exchange keeps the plain timeout, so the media punch
 * and the net/peer.h holds use the smaller value.
 */
#define NET_CT_PROC_DIR			"/proc/sys/net/netfilter/"
#define NET_CT_UDP_TIMEOUT_PATH \
	NET_CT_PROC_DIR "nf_conntrack_udp_timeout"
#define NET_CT_UDP_STREAM_PATH \
	NET_CT_PROC_DIR "nf_conntrack_udp_timeout_stream"
#define NET_CT_UDP_TIMEOUT_DEFAULT_S	60
#define NET_CT_UDP_STREAM_DEFAULT_S	180

/*
 * The longest an idle media descriptor stays silent before the backend sends
 * its empty datagram (hal.h, hal_media.punch_interval_ms): the RFC 6263
 * section 7 interval, which the backend also defaults to, and less when the
 * conntrack bound above asks for it.
 */
#define NET_CT_PUNCH_MAX_MS		15000

static unsigned int proc_uint_read(const char *path, unsigned int def)
{
	FILE *f = fopen(path, "r");
	unsigned int v;
	int n;

	if (!f)
		return def;

	n = fscanf(f, "%u", &v);
	fclose(f);

	return n == 1 ? v : def;
}

/* the pair the suite puts in place of the two procfs files (conntrack.h) */
static struct {
	unsigned int timeout_s;
	unsigned int stream_s;
} ct_test;

void net_ct_udp_bound_test_set(unsigned int timeout_s, unsigned int stream_s)
{
	ct_test.timeout_s = timeout_s;
	ct_test.stream_s = stream_s;
}

unsigned int net_ct_udp_bound_s(void)
{
	unsigned int timeout = ct_test.timeout_s;
	unsigned int stream = ct_test.stream_s;

	if (!timeout && !stream) {
		timeout = proc_uint_read(NET_CT_UDP_TIMEOUT_PATH,
					 NET_CT_UDP_TIMEOUT_DEFAULT_S);
		stream = proc_uint_read(NET_CT_UDP_STREAM_PATH,
					NET_CT_UDP_STREAM_DEFAULT_S);
	}

	if (!timeout)
		timeout = NET_CT_UDP_TIMEOUT_DEFAULT_S;
	if (!stream)
		stream = NET_CT_UDP_STREAM_DEFAULT_S;

	return timeout < stream ? timeout : stream;
}

/*
 * Half the bound, so the backend refreshes the conntrack entry of an idle
 * media descriptor before it expires. No firewall rule opens the RTP range,
 * so inbound media depends on that entry.
 */
uint16_t net_ct_punch_interval_ms(void)
{
	unsigned int bound = net_ct_udp_bound_s();

	if (bound >= NET_CT_PUNCH_MAX_MS / 500)
		return NET_CT_PUNCH_MAX_MS;

	return (uint16_t)(bound * 500);
}
