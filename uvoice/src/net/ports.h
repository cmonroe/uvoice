// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * RTP port pool of the daemon and effective port range of a channel
 * (src/net/ports.c). The pool hands out even ports; RTCP takes the odd port
 * above each one.
 */
#ifndef UVOICE_NET_PORTS_H
#define UVOICE_NET_PORTS_H

#include <stdbool.h>
#include <stdint.h>

#include "cfg/cfg.h"

/* Return every port to the pool. */
void net_port_reset(void);

/*
 * Take the next free port of the pool into *out, round robin. The pool takes
 * the range min..max only while no port is out, so a range that moves under a
 * live leg applies once the pool drains. False when the pool holds no free
 * port.
 */
bool net_port_alloc(unsigned int min, unsigned int max, uint16_t *out);

/* Give a port back to the pool. */
void net_port_free(uint16_t port);

/*
 * The effective RTP port range of one channel: the configured pair, or the
 * built-in one when the config leaves it unset or incoherent. ch can be NULL,
 * which answers the built-in range.
 */
void net_port_range(const struct cfg_channel *ch, uint16_t *min, uint16_t *max);

/*
 * Whether two channel configurations name the same effective range. A reload
 * clears the legs of a line whose range moved, because the pool takes a new
 * range only while no port is out.
 */
bool net_port_range_eq(const struct cfg_channel *a,
		       const struct cfg_channel *b);

#endif
