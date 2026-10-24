// SPDX-License-Identifier: GPL-2.0-only
/*
 * uvoice - native voice stack for OpenWrt
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Licensed under GPL-2.0-only with the linking exception stated in LICENSE.
 *
 * Unit tests for the writer of the uvoice nft sets (src/net/peer.c).
 *
 * The suite runs the device path: the spawn of spawn_run.h and the
 * uloop_process reap. UVOICE_NFT names a generated script in place of nft.
 * That script appends its arguments to a file as one line, and exits with the
 * code in UVOICE_NFT_EXIT. With UVOICE_NFT_FIFO set, it first waits on that
 * fifo, so a case decides when the write ends. Each case reads its writes
 * back from that file. Reason: a recorder inside the layer proves only itself.
 *
 * net_ct_udp_bound_test_set() puts a conntrack bound of 2 s in place, so a
 * hold writes again every second. Every wait drives uloop to a deadline.
 * Each case uses keys of its own, because the layer keeps the deadline of
 * each write for the length of the run.
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "net/conntrack.h"
#include "net/peer.h"
#include "test_util.h"
#include "test_uloop.h"

#define WARN_CAPTURE_ULOG_OWN
#include "test_warn.h"

#define OUT_LINES_MAX	16
#define OUT_LINE_LEN	160

static char nft_script[128];
static char nft_out[128];
static char nft_fifo[128];

static bool nft_fixture_make(void)
{
	FILE *f;

	if (!tu_tmp_file_make(nft_out, sizeof(nft_out), "uvoice-nft-out") ||
	    !tu_tmp_file_make(nft_script, sizeof(nft_script), "uvoice-nft") ||
	    !tu_tmp_file_make(nft_fifo, sizeof(nft_fifo), "uvoice-nft-fifo"))
		return false;

	unlink(nft_fifo);
	if (mkfifo(nft_fifo, 0600))
		return false;

	f = fopen(nft_script, "w");
	if (!f)
		return false;

	fprintf(f, "#!/bin/sh\n"
		"printf '%%s\\n' \"$*\" >> %s\n"
		"[ -z \"$UVOICE_NFT_FIFO\" ] || read -r go < \"$UVOICE_NFT_FIFO\"\n"
		"exit \"${UVOICE_NFT_EXIT:-0}\"\n", nft_out);
	fclose(f);

	return chmod(nft_script, 0755) == 0;
}

static void nft_fixture_drop(void)
{
	unlink(nft_script);
	unlink(nft_out);
	unlink(nft_fifo);
}

/* ends a child that waits on the fifo; fails at once when no child reads it */
static bool nft_release(void)
{
	int fd = open(nft_fifo, O_WRONLY | O_NONBLOCK);
	bool ok;

	if (fd < 0)
		return false;

	ok = write(fd, "go\n", 3) == 3;
	close(fd);

	return ok;
}

/*
 * Passes once the running child opened the fifo and got its release, or kills
 * that child and fails after 2 s, so that no later reap waits on it forever.
 * A child that waits is never reaped, so driving the loop between tries is
 * safe.
 */
static bool nft_release_wait(void)
{
	pid_t pid = net_peer_test_pid();
	unsigned int i;

	for (i = 0; i < 200; i++) {
		if (nft_release())
			return true;
		uloop_pump(10);
	}

	if (pid > 0)
		kill(pid, SIGKILL);

	return false;
}

static void out_reset(void)
{
	FILE *f = fopen(nft_out, "w");

	if (f)
		fclose(f);
}

static unsigned int out_lines(char out[OUT_LINES_MAX][OUT_LINE_LEN])
{
	char line[OUT_LINE_LEN];
	unsigned int n = 0;
	FILE *f = fopen(nft_out, "r");

	if (!f)
		return 0;

	while (n < OUT_LINES_MAX && fgets(line, sizeof(line), f)) {
		line[strcspn(line, "\n")] = 0;
		snprintf(out[n++], OUT_LINE_LEN, "%s", line);
	}

	fclose(f);

	return n;
}

static unsigned int out_count(void)
{
	char got[OUT_LINES_MAX][OUT_LINE_LEN];

	return out_lines(got);
}

/* passes once the queue is empty and no child runs, or fails after 2 s */
static bool peer_idle_wait(void)
{
	unsigned int i;

	for (i = 0; i < 200; i++) {
		if (!net_peer_test_busy() && !net_peer_test_queued())
			return true;
		uloop_pump(10);
	}

	return false;
}

/* passes once a child runs, or fails after 2 s */
static bool peer_busy_wait(void)
{
	unsigned int i;

	for (i = 0; i < 200 && !net_peer_test_busy(); i++)
		uloop_pump(10);

	return net_peer_test_busy();
}

/*
 * A request that the layer cannot write is refused at once, and no child
 * runs.
 */
static void test_refusals(void)
{
	CASE("refuse: an absent nft refuses the request and runs nothing");
	out_reset();
	setenv("UVOICE_NFT", "/nonexistent/uvoice/nft", 1);
	CHECK(!net_peer_admit("lo", "192.0.2.1", false, 5060, 30));
	CHECK_UINT(net_peer_test_queued(), 0);
	CHECK(!net_peer_test_timer_armed());
	uloop_pump(50);
	CHECK_UINT(out_count(), 0);
	setenv("UVOICE_NFT", nft_script, 1);

	CASE("refuse: a NULL or an empty device refuses the request and runs "
	     "nothing");
	CHECK(!net_peer_admit(NULL, "192.0.2.2", false, 5060, 30));
	CHECK(!net_peer_admit("", "192.0.2.2", false, 5060, 30));
	CHECK(net_peer_hold(NULL, "192.0.2.2", false, 5060) < 0);
	CHECK(net_peer_hold("", "192.0.2.2", false, 5060) < 0);
	CHECK_UINT(net_peer_test_queued(), 0);
	CHECK(!net_peer_test_timer_armed());
	uloop_pump(50);
	CHECK_UINT(out_count(), 0);

	CASE("refuse: a host that is absent or no literal of its family runs "
	     "nothing");
	CHECK(!net_peer_admit("lo", NULL, false, 5060, 30));
	CHECK(net_peer_hold("lo", NULL, false, 5060) < 0);
	CHECK(!net_peer_admit("lo", "2001:db8::2", false, 5060, 30));
	CHECK(!net_peer_admit("lo", "192.0.2.2", true, 5060, 30));
	CHECK(!net_peer_admit("lo", "192.0.2.2 } flush ruleset", false, 5060,
			      30));
	CHECK_UINT(net_peer_test_queued(), 0);
	uloop_pump(50);
	CHECK_UINT(out_count(), 0);
}

/*
 * One write per element, with the argv tokens of each set. The device token
 * carries its double quotes, because nft parses the joined arguments again.
 */
static void test_writes(void)
{
	char got[OUT_LINES_MAX][OUT_LINE_LEN];

	CASE("write: an IPv4 peer and an IPv6 peer each write one element");
	out_reset();
	CHECK(net_peer_admit("lo", "192.0.2.1", false, 5060, 30));
	CHECK(net_peer_admit("lo", "2001:db8::1", true, 5060, 30));
	CHECK(peer_idle_wait());
	CHECK_UINT(out_lines(got), 2);
	CHECK(!strcmp(got[0], "add element inet fw4 uvoice_peer4 "
		      "{ \"lo\" . 192.0.2.1 . 5060 timeout 30s expires 30s }"));
	CHECK(!strcmp(got[1], "add element inet fw4 uvoice_peer6 "
		      "{ \"lo\" . 2001:db8::1 . 5060 "
		      "timeout 30s expires 30s }"));
}

/*
 * The queue writes one element per key, in request order. It never ends an
 * element sooner than its last write, because an add with "expires" replaces
 * the expiry.
 */
static void test_queue(void)
{
	char got[OUT_LINES_MAX][OUT_LINE_LEN];

	CASE("queue: six requests before the loop runs write five keys once "
	     "each, in request order");
	out_reset();
	CHECK(net_peer_admit("lo", "192.0.2.11", false, 5060, 30));
	CHECK(net_peer_admit("lo", "192.0.2.12", false, 5060, 30));
	CHECK(net_peer_admit("lo", "192.0.2.13", false, 5060, 30));
	CHECK(net_peer_admit("lo", "192.0.2.14", false, 5060, 30));
	CHECK(net_peer_admit("lo", "192.0.2.15", false, 5060, 30));
	CHECK(net_peer_admit("lo", "192.0.2.12", false, 5060, 45));
	CHECK_UINT(net_peer_test_queued(), 5);
	CHECK(!net_peer_test_busy());
	CHECK(peer_idle_wait());
	CHECK_UINT(out_lines(got), 5);
	CHECK(strstr(got[0], " 192.0.2.11 ") != NULL);
	CHECK(strstr(got[2], " 192.0.2.13 ") != NULL);
	CHECK(strstr(got[3], " 192.0.2.14 ") != NULL);
	CHECK(strstr(got[4], " 192.0.2.15 ") != NULL);

	CASE("queue: the repeat is written once, with the later timeout");
	CHECK(strstr(got[1], " 192.0.2.12 . 5060 timeout 45s expires 45s }") !=
	      NULL);

	CASE("queue: a shorter repeat keeps the longer queued timeout");
	out_reset();
	CHECK(net_peer_admit("lo", "192.0.2.16", false, 5060, 45));
	CHECK(net_peer_admit("lo", "192.0.2.16", false, 5060, 30));
	CHECK_UINT(net_peer_test_queued(), 1);
	CHECK(peer_idle_wait());
	CHECK_UINT(out_lines(got), 1);
	CHECK(strstr(got[0], " 192.0.2.16 . 5060 timeout 45s expires 45s }") !=
	      NULL);

	CASE("queue: a shorter timeout after a write of 60 s writes nothing");
	out_reset();
	CHECK(net_peer_admit("lo", "192.0.2.21", false, 5060, 60));
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 1);
	CHECK(net_peer_admit("lo", "192.0.2.21", false, 5060, 10));
	CHECK_UINT(net_peer_test_queued(), 0);
	uloop_pump(50);
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 1);
}

/*
 * nft sets the expiry when it runs, after the spawn, so the layer counts the
 * deadline of a write from the end of its child. The child here ends 1.5 s
 * after its spawn. A 59 s request then reaches past a deadline counted from
 * the spawn, but not past one counted from the end.
 */
static void test_late_end(void)
{
	CASE("late end: a 60 s write whose child ends 1.5 s after its spawn");
	out_reset();
	setenv("UVOICE_NFT_FIFO", nft_fifo, 1);
	CHECK(net_peer_admit("lo", "192.0.2.61", false, 5060, 60));
	CHECK(peer_busy_wait());
	unsetenv("UVOICE_NFT_FIFO");
	uloop_pump(1500);
	CHECK(net_peer_test_busy());
	CHECK(nft_release_wait());
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 1);

	CASE("late end: a 59 s request after that end writes nothing");
	CHECK(net_peer_admit("lo", "192.0.2.61", false, 5060, 59));
	CHECK_UINT(net_peer_test_queued(), 0);
	uloop_pump(50);
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 1);

	CASE("late end: a 60 s request after that end writes");
	CHECK(net_peer_admit("lo", "192.0.2.61", false, 5060, 60));
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 2);
}

/*
 * A request for the key of the running child compares with the deadline of
 * that child, because the child replaces the expiry when it ends.
 */
static void test_running_child(void)
{
	CASE("running child: a shorter request for its key is dropped");
	out_reset();
	setenv("UVOICE_NFT_FIFO", nft_fifo, 1);
	CHECK(net_peer_admit("lo", "192.0.2.71", false, 5060, 60));
	CHECK(peer_busy_wait());
	unsetenv("UVOICE_NFT_FIFO");
	CHECK(net_peer_admit("lo", "192.0.2.71", false, 5060, 59));
	CHECK_UINT(net_peer_test_queued(), 0);

	CASE("running child: an equal request for its key is queued");
	CHECK(net_peer_admit("lo", "192.0.2.71", false, 5060, 60));
	CHECK_UINT(net_peer_test_queued(), 1);

	CASE("running child: after the release, the equal request writes");
	CHECK(net_peer_test_busy());
	CHECK(nft_release_wait());
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 2);
}

/*
 * A hold writes at once and then every half bound, with the bound as its
 * timeout, until its owner releases it.
 */
static void test_hold(void)
{
	char got[OUT_LINES_MAX][OUT_LINE_LEN];
	int h;

	CASE("hold: one write at once, with the bound as its timeout");
	out_reset();
	h = net_peer_hold("lo", "192.0.2.31", false, 5060);
	CHECK(h >= 0);
	CHECK(net_peer_test_timer_armed());
	CHECK(peer_idle_wait());
	CHECK_UINT(out_lines(got), 1);
	CHECK(!strcmp(got[0], "add element inet fw4 uvoice_peer4 "
		      "{ \"lo\" . 192.0.2.31 . 5060 timeout 2s expires 2s }"));

	CASE("hold: the next write follows one second later, half the bound");
	uloop_pump(600);
	CHECK_UINT(out_count(), 1);
	uloop_pump(600);
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 2);

	CASE("hold: a release stops the writes and the timer");
	net_peer_release(h);
	CHECK(!net_peer_test_timer_armed());
	uloop_pump(1200);
	CHECK(peer_idle_wait());
	CHECK_UINT(out_count(), 2);
}

/*
 * The shipped conntrack timeouts, 60 s and 180 s, give a bound of 60 s. A
 * hold then writes with a 60 s timeout, and its tick comes every 30 s.
 */
static void test_hold_period(void)
{
	char got[OUT_LINES_MAX][OUT_LINE_LEN];
	int64_t tick;
	int h;

	CASE("hold period: the shipped bound arms the tick at 30000 ms");
	out_reset();
	net_ct_udp_bound_test_set(60, 180);
	h = net_peer_hold("lo", "192.0.2.32", false, 5060);
	CHECK(h >= 0);
	tick = net_peer_test_tick_ms();
	CHECK(tick > 29000 && tick <= 30000);

	CASE("hold period: the write takes the shipped bound as its timeout");
	CHECK(peer_idle_wait());
	CHECK_UINT(out_lines(got), 1);
	CHECK(!strcmp(got[0], "add element inet fw4 uvoice_peer4 "
		      "{ \"lo\" . 192.0.2.32 . 5060 timeout 60s expires 60s }"));
	net_peer_release(h);
	CHECK(net_peer_test_tick_ms() == -1);
	net_ct_udp_bound_test_set(2, 6);
}

/*
 * A run of failures warns once, at its first failure, and names the snippet:
 * a set that fw4 did not load is the likely cause. Later failures of a run
 * trace only. The first success after them logs once at info.
 */
static void test_failure_log(void)
{
	CASE("log: the first failure warns once and names the snippet");
	setenv("UVOICE_NFT_EXIT", "1", 1);
	warn_capture_begin();
	CHECK(net_peer_admit("lo", "192.0.2.41", false, 5060, 30));
	CHECK(peer_idle_wait());
	CHECK_UINT(warn_capture_count_at(LOG_WARNING,
		   "firewall: nft add element uvoice_peer4 failed "
		   "(exit 1)"), 1);
	CHECK_UINT(warn_capture_count("check that fw4 loaded "
		   "/usr/share/nftables.d/table-pre/20-uvoice.nft"), 1);

	CASE("log: a second failure of the run adds no warning");
	warn_capture_begin();
	CHECK(net_peer_admit("lo", "192.0.2.42", false, 5060, 30));
	CHECK(peer_idle_wait());
	CHECK_UINT(warn_capture_count("firewall: nft add element"), 0);

	CASE("log: the first success after the failures logs once at info");
	unsetenv("UVOICE_NFT_EXIT");
	warn_capture_begin();
	CHECK(net_peer_admit("lo", "192.0.2.43", false, 5060, 30));
	CHECK(net_peer_admit("lo", "192.0.2.44", false, 5060, 30));
	CHECK(peer_idle_wait());
	CHECK_UINT(warn_capture_count_at(LOG_INFO,
		   "firewall: nft add element works again"), 1);
	CHECK_UINT(warn_capture_count_at(LOG_WARNING, "firewall:"), 0);
	warn_capture_end();
}

/*
 * net_peer_done() empties the queue and the holds, stops the timers and
 * reaps the running child before it returns. The child waits on the fifo
 * until the release, and the loop does not run after the release, so only
 * net_peer_done() can reap it.
 */
static void test_done(void)
{
	pid_t pid;

	CASE("done: a queued request, a hold and a running child all end");
	out_reset();
	setenv("UVOICE_NFT_FIFO", nft_fifo, 1);
	CHECK(net_peer_admit("lo", "192.0.2.51", false, 5060, 30));
	CHECK(peer_busy_wait());
	unsetenv("UVOICE_NFT_FIFO");
	pid = net_peer_test_pid();
	CHECK(pid > 0);
	CHECK(net_peer_admit("lo", "192.0.2.52", false, 5060, 30));
	CHECK(net_peer_hold("lo", "192.0.2.53", false, 5060) >= 0);
	CHECK_UINT(net_peer_test_queued(), 2);
	CHECK(net_peer_test_timer_armed());
	CHECK(nft_release_wait());
	CHECK(net_peer_test_busy());
	net_peer_done();
	CHECK_UINT(net_peer_test_queued(), 0);
	CHECK(!net_peer_test_timer_armed());
	CHECK(!net_peer_test_busy());
	errno = 0;
	CHECK(pid > 0 && waitpid(pid, NULL, WNOHANG) == -1 && errno == ECHILD);

	CASE("done: nothing runs after it, the hold included");
	uloop_pump(1200);
	CHECK(!net_peer_test_busy());
	CHECK_UINT(out_count(), 1);
}

int main(void)
{
	uloop_init();
	net_ct_udp_bound_test_set(2, 6);

	SUITE_CHECK(nft_fixture_make());
	setenv("UVOICE_NFT", nft_script, 1);

	test_refusals();
	test_writes();
	test_queue();
	test_late_end();
	test_running_child();
	test_hold();
	test_hold_period();
	test_failure_log();
	test_done();

	unsetenv("UVOICE_NFT");
	nft_fixture_drop();
	net_ct_udp_bound_test_set(0, 0);
	uloop_done();

	return tu_summary("test_net_peer");
}
