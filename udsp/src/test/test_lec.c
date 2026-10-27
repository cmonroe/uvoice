// SPDX-License-Identifier: LGPL-2.1-only
/*
 * udsp - software voice DSP for FXS lines
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Unit suite for the NLMS line echo canceller (dsp/lec.c).
 *
 * The test scaffolding in bench/lec_paths.h builds every echo path and every
 * level. The ERLE reference is therefore the injected echo signal, never the
 * estimate of the canceller. A case can pass only if the residual echo really
 * fell against a signal that the test knows independently.
 *
 * The cases assert 25 dB of ERLE after one second of adaptation, at the 48 ms
 * default tail. The hybrid, not the algorithm, bounds a 16 ms tail.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "udsp/udsp.h"

#include "../dsp/dsp.h"
#include "../dsp/lec.h"
#include "../dsp/resamp.h"
#include "bench/lec_paths.h"
#include "bench/lec_paths_field.h"
#include "bench/lec_replay.h"
#include "util.h"

/* The engine tick is 10 ms at every line rate (core/clock.h), so a frame
 * count is a time and reads the same at 8 and 16 kHz. */
#define FRAMES_PER_SECOND	100u

/* LEC_COHERE_BAR, LEC_COHERE_FRAMES, LEC_COHERE_SHARE_MAX,
 * LEC_PATH_CHANGE_RATIO and LEC_ERLE_ALPHA of dsp/lec.c. */
#define COHERE_BAR		-0.10
#define COHERE_FRAMES		32u
#define COHERE_SHARE_MAX	0.5
#define COHERE_PATH_RATIO	1.26
#define COHERE_ALPHA		0.1

/* Only the wideband leg adds the rate to a case name. An 8 kHz case name
 * stays the bare name, so a case that gains a 16 kHz leg keeps its name. */
static void case_rate(unsigned rate, const char *what)
{
	char name[160];

	snprintf(name, sizeof(name), "%s%s", what,
		 rate == UDSP_SAMPLE_RATE_WB ? " at 16 kHz" : "");
	CASE(name);
}

struct dt_result {
	double erle_pre;
	double erle_post;
	double passthrough_db;
	float coef_before;
	float coef_after;
	uint32_t detected_frames;
	uint32_t diverge_resets;
};

static double energy_dbm0_get(double energy, unsigned count)
{
	return rms_dbm0_get(sqrt(energy / count));
}

/*
 * The path carries the rate it is built at, and the canceller instance that the
 * caller pairs with it runs the same frame. offset moves the reference against
 * the drive (ref_delay_run). half_db is a loss ahead of the reference alone, as
 * the engine applies it; the drive keeps the line level.
 */
static double erle_offset_run(struct udsp_ec *ec, struct echo_path *path,
			      uint32_t *state, double far_dbm0, int offset,
			      unsigned half_db, unsigned frames,
			      unsigned measure)
{
	struct ref_delay delay;
	int16_t source[UDSP_FRAME_MAX];
	int16_t drive[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	double echo_energy = 0.0;
	double residual_energy = 0.0;
	unsigned f;

	ref_delay_init(&delay, offset);
	for (f = 0; f < frames; f++) {
		unsigned i;

		noise_fill(source, path->frame, far_dbm0, state);
		ref_delay_run(&delay, source, path->frame, drive, ref);
		if (half_db)
			level_loss_apply(ref, ref, path->frame, half_db);
		path_run(path, drive, echo);
		memcpy(mic, echo, path->frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, mic, ref);
		if (f + measure < frames)
			continue;
		for (i = 0; i < path->frame; i++) {
			echo_energy += (double)echo[i] * echo[i];
			residual_energy += (double)mic[i] * mic[i];
		}
	}
	if (residual_energy < 1e-6)
		return 200.0;
	return 10.0 * log10(echo_energy / residual_energy);
}

static double erle_run(struct udsp_ec *ec, struct echo_path *path,
		       uint32_t *state, double far_dbm0, unsigned frames,
		       unsigned measure)
{
	return erle_offset_run(ec, path, state, far_dbm0, 0, 0, frames,
			       measure);
}

static double level_run(struct udsp_ec *ec, struct echo_path *path,
			uint32_t *state, double far_dbm0, double near_dbm0,
			unsigned frames, unsigned measure)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	uint32_t near_state = 0x2468acefu;
	double energy = 0.0;
	unsigned f;

	for (f = 0; f < frames; f++) {
		int16_t near[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, path->frame, far_dbm0, state);
		path_run(path, ref, echo);
		noise_fill(near, path->frame, near_dbm0, &near_state);
		for (i = 0; i < path->frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		if (f + measure < frames)
			continue;
		for (i = 0; i < path->frame; i++)
			energy += (double)mic[i] * mic[i];
	}
	return energy_dbm0_get(energy, measure * path->frame);
}

static void silence_run(struct udsp_ec *ec, double near_dbm0, unsigned frames)
{
	uint32_t near_state = 0x9e3779b9u;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	unsigned f;

	memset(ref, 0, sizeof(ref));
	for (f = 0; f < frames; f++) {
		noise_fill(mic, UDSP_FRAME_SAMPLES, near_dbm0, &near_state);
		udsp_ec_nlms.run(ec, mic, ref);
	}
}

static void tail_clamp_test(void)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec *ec;

	CASE("lec tail clamp below the supported range");
	ec = udsp_ec_nlms.create(15);
	CHECK(ec != NULL);
	if (!ec)
		return;
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 16);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.tail_ms, 16);
	CHECK_UINT(info.taps, 128);
	udsp_ec_nlms.destroy(ec);

	CASE("lec tail clamp above the supported range");
	ec = udsp_ec_nlms.create(129);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 128);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.taps, 1024);
	udsp_ec_nlms.destroy(ec);

	CASE("lec tail request honoured inside the range");
	ec = udsp_ec_nlms.create(16);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 16);
	udsp_ec_nlms.destroy(ec);
	ec = udsp_ec_nlms.create(48);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 48);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.taps, 384);
	udsp_ec_nlms.destroy(ec);
	ec = udsp_ec_nlms.create(128);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 128);
	udsp_ec_nlms.destroy(ec);

	CASE("lec tail zero request takes the default tail");
	ec = udsp_ec_nlms.create(0);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 48);
	CHECK_UINT(udsp_ec_nlms_tail_get(NULL), 0);
	udsp_ec_nlms.destroy(ec);
	/* The wideband default is the wideband cap: real lines read more
	 * echo loss at 64 ms, and the tick budget is measured at that tail. */
	ec = udsp_ec_nlms.create_rate(0, UDSP_SAMPLE_RATE_WB);
	CHECK(ec != NULL);
	if (ec) {
		CHECK_UINT(udsp_ec_nlms_tail_get(ec), 64);
		udsp_ec_nlms_info_get(ec, &info);
		CHECK_UINT(info.taps, 1024);
		udsp_ec_nlms.destroy(ec);
	}
}

/*
 * The same three hybrids at both line rates. The wideband leg asserts the same
 * 25 dB in one second. At the same 48 ms tail, the 16 kHz instance carries
 * twice the taps of the 8 kHz one. The bar shows that the extra taps still
 * converge inside the second.
 */
static void convergence_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state;
	double erle;

	/*
	 * The NLP is off in every ERLE case: the oracle must see the linear
	 * residual, not the comfort noise that replaces it.
	 */
	case_rate(rate, "lec flat path converges");
	state = 0x13572468u;
	ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, 12.0);
	erle = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 20);
	CHECK(erle >= 25.0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.hold_frames, 0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);

	/*
	 * The dispersive path runs 10 dB below the flat one: a step that is
	 * not normalized by the reference power cannot serve both levels.
	 */
	case_rate(rate, "lec dispersive path converges");
	state = 0x2468ace0u;
	ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(ec, false);
	path_dispersive_init(&path, rate, 12.0);
	erle = erle_run(ec, &path, &state, -20.0, FRAMES_PER_SECOND, 20);
	CHECK(erle >= 25.0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.hold_frames, 0);
	udsp_ec_nlms.destroy(ec);

	case_rate(rate, "lec composite path converges");
	state = 0x0f1e2d3cu;
	ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(ec, false);
	path_composite_init(&path, rate, 12.0);
	erle = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 20);
	CHECK(erle >= 25.0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.hold_frames, 0);
	udsp_ec_nlms.destroy(ec);
}

static void tail_bound_test(void)
{
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x5a5a1234u;
	double inside;
	double outside;

	ec = udsp_ec_nlms.create(48);
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 40, 12.0);
	inside = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 20);
	udsp_ec_nlms.destroy(ec);

	ec = udsp_ec_nlms.create(48);
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 60, 12.0);
	outside = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 20);
	udsp_ec_nlms.destroy(ec);

	/* The tail is counted after the bulk delay (ec.h): an echo further out
	 * than the tail is not the canceller's to remove. */
	CASE("lec tail bounds the cancelled delay");
	CHECK(inside >= 25.0);
	CHECK(outside < 3.0);
}

static void doubletalk_probe(unsigned rate, bool nlp, struct dt_result *result)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x0badc0deu;
	uint32_t near_state = 0xfeedface;
	double product = 0.0;
	double near_energy = 0.0;
	unsigned f;

	memset(result, 0, sizeof(*result));
	udsp_ec_nlms.nlp(ec, nlp);
	path_flat_init(&path, rate, 8, 12.0);
	result->erle_pre = erle_run(ec, &path, &state, -10.0,
				    FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2,
				    20);
	udsp_ec_nlms_info_get(ec, &info);
	result->coef_before = info.coef_energy;
	for (f = 0; f < FRAMES_PER_SECOND / 2; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, path.frame, -10.0, &state);
		path_run(&path, ref, echo);
		/* 3 dB above the echo the -12 dB hybrid returns. */
		noise_fill(near, path.frame, -19.0, &near_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.double_talk)
			result->detected_frames++;
		/* The detector needs the frame it fires in and the NLP one
		 * more to cross-fade out; measure the settled burst. */
		if (f < 5)
			continue;
		for (i = 0; i < path.frame; i++) {
			product += (double)mic[i] * near[i];
			near_energy += (double)near[i] * near[i];
		}
	}
	udsp_ec_nlms_info_get(ec, &info);
	result->coef_after = info.coef_energy;
	result->passthrough_db = -20.0 * log10(product / near_energy);
	result->erle_post = erle_run(ec, &path, &state, -10.0,
				     FRAMES_PER_SECOND / 2, 20);
	udsp_ec_nlms_info_get(ec, &info);
	result->diverge_resets = info.diverge_resets;
	udsp_ec_nlms.destroy(ec);
}

static void doubletalk_test(unsigned rate)
{
	struct dt_result linear;
	struct dt_result gated;

	doubletalk_probe(rate, false, &linear);
	doubletalk_probe(rate, true, &gated);

	/* The burst must leave the filter exactly where it found it: the frame
	 * the detector needs to see before it can fire is adapted on and then
	 * rolled back, so the whole burst is a no-op on the coefficients. */
	case_rate(rate, "lec double talk holds adaptation");
	CHECK(linear.erle_pre >= 25.0);
	CHECK(linear.detected_frames >= FRAMES_PER_SECOND / 4);
	CHECK(linear.erle_post >= 20.0);
	CHECK(linear.coef_after == linear.coef_before);
	CHECK(gated.coef_after == gated.coef_before);
	CHECK_UINT(linear.diverge_resets, 0);

	case_rate(rate, "lec double talk passes the near end");
	CHECK(gated.detected_frames >= FRAMES_PER_SECOND / 4);
	CHECK(gated.passthrough_db <= 3.0);
	CHECK(gated.passthrough_db >= -3.0);
}

static void freeze_test(void)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path old_path;
	struct echo_path new_path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x77778888u;
	double erle_old;
	double erle_held;
	double erle_new;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&old_path, UDSP_SAMPLE_RATE, 8, 12.0);
	path_dispersive_init(&new_path, UDSP_SAMPLE_RATE, 12.0);
	erle_old = erle_run(ec, &old_path, &state, -10.0,
			    FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	udsp_ec_nlms_info_get(ec, &before);
	udsp_ec_nlms.freeze(ec, true);
	(void)erle_run(ec, &new_path, &state, -10.0, FRAMES_PER_SECOND, 1);
	udsp_ec_nlms_info_get(ec, &after);
	erle_held = erle_run(ec, &old_path, &state, -10.0,
			     FRAMES_PER_SECOND / 2, 20);

	CASE("lec freeze holds adaptation across a path change");
	CHECK(erle_old >= 25.0);
	CHECK(after.coef_energy == before.coef_energy);
	CHECK(after.hold_frames >= FRAMES_PER_SECOND);
	CHECK(erle_held >= erle_old - 3.0);
	CHECK_UINT(after.diverge_resets, 0);

	udsp_ec_nlms.freeze(ec, false);
	erle_new = erle_run(ec, &new_path, &state, -10.0,
			    FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	CASE("lec freeze release reconverges on the new path");
	CHECK(erle_new >= 25.0);
	udsp_ec_nlms.destroy(ec);
}

static void nlp_test(void)
{
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state;
	double suppressed;
	double linear;
	double matched;

	state = 0x31415926u;
	ec = udsp_ec_nlms.create(48);
	path_composite_init(&path, UDSP_SAMPLE_RATE, 12.0);
	/* About -33 dB of residual non-linearity: with a purely linear path
	 * the two NLP modes are indistinguishable at this level. */
	path.cubic = 0.12;
	silence_run(ec, -200.0, 20);
	suppressed = level_run(ec, &path, &state, 0.0, -200.0,
			       2u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms.destroy(ec);

	state = 0x31415926u;
	ec = udsp_ec_nlms.create(48);
	udsp_ec_nlms.nlp(ec, false);
	path_composite_init(&path, UDSP_SAMPLE_RATE, 12.0);
	path.cubic = 0.12;
	silence_run(ec, -200.0, 20);
	linear = level_run(ec, &path, &state, 0.0, -200.0,
			   2u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms.destroy(ec);

	CASE("lec nlp suppresses the converged residual");
	CHECK(suppressed <= -60.0);

	CASE("lec nlp off passes the linear residual");
	CHECK(linear >= -55.0);
	CHECK(linear - suppressed >= 10.0);

	state = 0x27182818u;
	ec = udsp_ec_nlms.create(48);
	path_composite_init(&path, UDSP_SAMPLE_RATE, 12.0);
	path.cubic = 0.12;
	silence_run(ec, -50.0, 40);
	matched = level_run(ec, &path, &state, 0.0, -50.0,
			    2u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms.destroy(ec);

	CASE("lec nlp comfort noise matches the near-end floor");
	CHECK(fabs(matched + 50.0) <= 4.0);
}

static void saturation_test(void)
{
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	uint32_t state = 0x1a2b3c4du;
	unsigned i;
	unsigned rail = 0;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	udsp_ec_nlms.freeze(ec, true);
	for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
		ref[i] = 32767;
	for (i = 0; i < 2u; i++) {
		unsigned n;

		for (n = 0; n < UDSP_FRAME_SAMPLES; n++)
			mic[n] = -32768;
		udsp_ec_nlms.run(ec, mic, ref);
	}
	for (i = 0; i < UDSP_FRAME_SAMPLES; i++) {
		if (mic[i] == -32768)
			rail++;
	}

	/* -32768 minus a positive estimate has no int16 form: it must clamp,
	 * not wrap. */
	CASE("lec output saturates");
	CHECK_UINT(rail, UDSP_FRAME_SAMPLES);
	udsp_ec_nlms.destroy(ec);
}

static void determinism_test(void)
{
	struct udsp_ec *first = udsp_ec_nlms.create(48);
	struct udsp_ec *second = udsp_ec_nlms.create(48);
	struct udsp_ec_nlms_info info_first;
	struct udsp_ec_nlms_info info_second;
	struct echo_path path_first;
	struct echo_path path_second;
	uint32_t state_first = 0x24681357u;
	uint32_t state_second = 0x24681357u;
	unsigned differences = 0;
	unsigned f;

	path_composite_init(&path_first, UDSP_SAMPLE_RATE, 12.0);
	path_composite_init(&path_second, UDSP_SAMPLE_RATE, 12.0);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		int16_t ref_first[UDSP_FRAME_SAMPLES];
		int16_t ref_second[UDSP_FRAME_SAMPLES];
		int16_t mic_first[UDSP_FRAME_SAMPLES];
		int16_t mic_second[UDSP_FRAME_SAMPLES];

		noise_fill(ref_first, UDSP_FRAME_SAMPLES, -10.0, &state_first);
		noise_fill(ref_second, UDSP_FRAME_SAMPLES, -10.0,
			   &state_second);
		path_run(&path_first, ref_first, mic_first);
		path_run(&path_second, ref_second, mic_second);
		udsp_ec_nlms.run(first, mic_first, ref_first);
		udsp_ec_nlms.run(second, mic_second, ref_second);
		if (memcmp(mic_first, mic_second, sizeof(mic_first)))
			differences++;
	}
	udsp_ec_nlms_info_get(first, &info_first);
	udsp_ec_nlms_info_get(second, &info_second);

	CASE("lec run is deterministic");
	CHECK_UINT(differences, 0);
	CHECK(info_first.coef_energy == info_second.coef_energy);
	CHECK(info_first.erle_db == info_second.erle_db);
	udsp_ec_nlms.destroy(first);
	udsp_ec_nlms.destroy(second);
}

/*
 * A hybrid that gets louder leaves the echo estimate short. Once the detector
 * trusts the filter, it thresholds against that estimate, so the hold feeds
 * itself. Only the escape ends the hold.
 */
static void detector_latch_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x0c0ffee1u;
	double erle;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	/* The same hybrid, 6 dB louder, with its history intact: a fresh path
	 * object would break the echo for one frame and collapse the tracker,
	 * which is the one thing that ends the hold by accident. */
	path_erl_set(&path, 6.0);
	erle = erle_run(ec, &path, &state, -10.0, 10u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec detector releases a hybrid that got louder");
	CHECK(erle >= 25.0);
	CHECK(info.hold_frames >= 4u * FRAMES_PER_SECOND);
	CHECK(info.hold_frames <= 7u * FRAMES_PER_SECOND);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * A converged instance with the NLP on must not mute the caller the moment
 * the far end stops. With no reference, there is no echo to replace. The
 * detector also cannot flag near-end speech that no reference explains.
 */
static void nlp_near_only_test(void)
{
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x51ec0de5u;
	uint32_t near_state = 0xa1b2c3d4u;
	double product = 0.0;
	double near_energy = 0.0;
	unsigned f;

	path_composite_init(&path, UDSP_SAMPLE_RATE, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		unsigned i;

		memset(ref, 0, sizeof(ref));
		noise_fill(near, UDSP_FRAME_SAMPLES, -20.0, &near_state);
		memcpy(mic, near, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
		/* The tail window has to flush before the reference reads as
		 * silent, and the NLP takes one frame to fade out. */
		if (f < 20u)
			continue;
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++) {
			product += (double)mic[i] * near[i];
			near_energy += (double)near[i] * near[i];
		}
	}

	CASE("lec nlp passes near speech with a silent far end");
	CHECK(-20.0 * log10(product / near_energy) <= 3.0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * A hybrid that steps to 3 dB ERL is louder than every expectation that the
 * detector holds, so no verdict releases the hold. Only the adaptation probe
 * at the end of the latch can prove that the line changed.
 */
static void path_step_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x9a8b7c6du;
	double erle;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	path_erl_set(&path, 3.0);
	erle = erle_run(ec, &path, &state, -10.0, 8u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate,
		  "lec adaptation probe recovers a hybrid that got much louder");
	CHECK(erle >= 20.0);
	CHECK(info.probes >= 1u);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The latch escape reads five seconds of unbroken double talk as a hybrid that
 * its estimate does not explain. A talker who holds the line that long
 * therefore gets an adaptation probe run against their speech. The trial must
 * be thrown away, because what the filter learns from a near-end talker is not
 * the hybrid.
 */
static void sustained_doubletalk_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x0badc0deu;
	uint32_t near_state = 0xfeedfaceu;
	double product = 0.0;
	double near_energy = 0.0;
	double pre;
	double post;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	pre = erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	for (f = 0; f < 12u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		unsigned i;

		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		noise_fill(near, UDSP_FRAME_SAMPLES, -19.0, &near_state);
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		if (f < 5u)
			continue;
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++) {
			product += (double)mic[i] * near[i];
			near_energy += (double)near[i] * near[i];
		}
	}
	post = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND / 2, 20);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec sustained double talk survives the adaptation probe");
	CHECK(info.probes >= 1u);
	CHECK(post >= pre - 6.0);
	CHECK(-20.0 * log10(product / near_energy) <= 3.0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * Speech-shaped far end, as on a real line: syllabic raised-cosine envelope at
 * 3.5 Hz with a 14 dB swing, 2 s spurts with 0.5 s pauses, 40 percent
 * frame-granularity DTX inside a spurt (as PCMU sends it), and a spectral
 * color that cycles every two seconds. The white-noise cases miss every one
 * of these properties, and the bootstrap case needs them.
 */
static double syllabic_dbm0_get(unsigned frame, double mean_dbm0)
{
	double phase = 2.0 * M_PI * 3.5 * frame * UDSP_FRAME_SAMPLES /
		       UDSP_SAMPLE_RATE;

	return mean_dbm0 + 7.0 * cos(phase);
}

static void color_apply(int16_t *frame, unsigned index, double *m1, double *m2)
{
	double a = 0.0;
	double b = 0.0;
	unsigned k;

	switch ((index / (2u * FRAMES_PER_SECOND)) % 3u) {
	case 0: a = 1.2; b = -0.6; break;
	case 1: a = 1.7; b = -0.9; break;
	case 2: a = 0.5; b = 0.2; break;
	}
	for (k = 0; k < UDSP_FRAME_SAMPLES; k++) {
		double y = 0.3 * frame[k] + a * *m1 + b * *m2;

		*m2 = *m1;
		*m1 = y;
		frame[k] = sample_clamp(y);
	}
}

static void far_speech_fill(int16_t *ref, unsigned f, uint32_t *state,
			    double *m1, double *m2)
{
	bool talking = f % (5u * FRAMES_PER_SECOND / 2u) <
		       2u * FRAMES_PER_SECOND;
	bool gap;

	memset(ref, 0, UDSP_FRAME_SAMPLES * sizeof(*ref));
	if (!talking)
		return;
	noise_fill(ref, UDSP_FRAME_SAMPLES, syllabic_dbm0_get(f, -10.0), state);
	/* The DTX gap zeroes the frame after the color runs, so a gap frame is
	 * an exact zero on the wire while the color memory keeps running. */
	gap = rng_next(state) % 100u < 40u;
	color_apply(ref, f, m1, m2);
	if (gap)
		memset(ref, 0, UDSP_FRAME_SAMPLES * sizeof(*ref));
}

/*
 * The fixed activity bar of dsp/lec.c (LEC_REF_ACTIVE_POWER), in the float
 * arithmetic of the canceller, as a property of the signal. The canceller's bar
 * stands over it on a stationary far end; a case that needs the canceller's
 * view reads ref_bar, background or window_run of the reading.
 */
static bool frame_loud(const int16_t *frame, unsigned count)
{
	return udsp_ec_nlms_frame_power_get(frame, count) >= 2500.0f;
}

/*
 * The bootstrap on a weak hybrid of 22 to 24 dB of ERL. Near speech at -25 dBm0
 * sits far under the classic quarter-power expectation there. With the
 * learned bound, the bursts read as double talk and roll back, and the spurts
 * between them converge the filter. A tracker that adapts on every burst
 * collapses inside a burst, so the read at the end tells the two apart.
 */
static void bootstrap_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(0);
	uint32_t state = 0x13572468u;
	uint32_t near_state = 0x0badcafeu;
	double m1 = 0.0;
	double m2 = 0.0;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	/* 58.5 s, so the run ends exactly with a near burst that overlaps a
	 * far spurt: a tracker blind to the burst collapses inside it, and the
	 * read at the end is the discriminating one. */
	unsigned total = 5850u;
	unsigned active = 0;
	unsigned burst_loud = 0;
	unsigned burst_flagged = 0;
	unsigned held_active;
	unsigned quiet_run = 1000u;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 20, 23.0);
	for (f = 0; f < total; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		bool burst = f % 150u >= 100u;
		bool loud;
		unsigned i;

		far_speech_fill(ref, f, &state, &m1, &m2);
		loud = frame_loud(ref, UDSP_FRAME_SAMPLES);
		if (loud)
			active++;
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		/* 500 ms of near speech every 1.5 s, its own syllabic phase. */
		if (burst) {
			int16_t near[UDSP_FRAME_SAMPLES];

			noise_fill(near, UDSP_FRAME_SAMPLES,
				   syllabic_dbm0_get(f + 37u, -25.0),
				   &near_state);
			for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
				mic[i] = sample_clamp((double)mic[i] + near[i]);
		}
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (burst && loud) {
			burst_loud++;
			if (info.double_talk)
				burst_flagged++;
		}
		/* The reference counts the echo that the filter removes on the
		 * frames that the near end leaves clean: a burst and its
		 * hangover would count near speech as residual. */
		quiet_run = burst ? 0 : quiet_run + 1u;
		if (f + 1000u < total || quiet_run <= 30u)
			continue;
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++) {
			echo_energy += (double)echo[i] * echo[i];
			res_energy += (double)mic[i] * mic[i];
		}
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec learned bound carries the bootstrap through near speech");
	/* The detector must see the near bursts, which sit under the classic
	 * quarter-power ceiling, and the filter they would have corrupted must
	 * genuinely cancel the echo. */
	CHECK(burst_flagged * 10u >= burst_loud * 6u);
	CHECK(res_energy > 0.0);
	CHECK(10.0 * log10(echo_energy / res_energy) >= 25.0);
	CHECK(info.erle_db >= 15.0f);
	/* Every inactive frame holds by definition; the fraction that means
	 * anything is the held share of the active frames. */
	held_active = info.hold_frames - (total - active);
	CHECK(held_active * 5u < active * 3u);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The gain estimator against reference DTX. Silent frames inside an active
 * tail window carry no echo. A tracker that still compares them against the
 * standing window maximum walks its minimum far under the hybrid. The
 * estimate must stay within a few dB of the true gain on a heavily gapped
 * reference.
 */
static void gain_bound_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x6a11b0b1u;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 2, 12.0);
	for (f = 0; f < 20u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];

		if (rng_next(&state) % 100u < 50u)
			memset(ref, 0, sizeof(ref));
		else
			noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec gain estimator reads the hybrid through reference DTX");
	/* True power gain 10^-1.2 = 0.0631: the minimum statistic may
	 * under-read it, within 3 dB. */
	CHECK(info.echo_gain >= 0.0631f * 0.5f);
	CHECK(info.echo_gain <= 0.0631f * 2.0f);
	udsp_ec_nlms.destroy(ec);
}

/* One frame of the lockout-escape far signal: 300 ms spurts, 200 ms gaps,
 * every fifth pause 500 ms (the inter-sentence pause of real speech). */
struct dtx_far {
	uint32_t state;
	unsigned pos;
	unsigned spurt;
};

static void dtx_far_fill(struct dtx_far *g, int16_t *ref)
{
	unsigned gap = g->spurt == 4u ? 50u : 20u;

	if (g->pos < 30u)
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &g->state);
	else
		memset(ref, 0, UDSP_FRAME_SAMPLES * sizeof(*ref));
	g->pos++;
	if (g->pos >= 30u + gap) {
		g->pos = 0;
		g->spurt = (g->spurt + 1u) % 5u;
	}
}

/*
 * The hot-hybrid lockout (SLIC digital loopback, 0 dB echo): the detector reads
 * the unit echo as double talk from the first frame, and only the latch escape
 * ends it. DTX far speech pauses often enough to clear a run that resets at
 * each pause; the run that decays by four per clear frame still reaches the
 * latch.
 */
static void lockout_escape_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(0);
	struct dtx_far far = { .state = 0x0c0ffee1u };
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 0.0);
	for (f = 0; f < 30u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];

		dtx_far_fill(&far, ref);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec latch escape fires through far-end DTX");
	CHECK(info.probes >= 1u);
	CHECK(info.erle_db >= 15.0f);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/* The microphone of one 8 kHz frame: the echo plus a near-end floor, or, when
 * floor_state is NULL, the echo alone and bit for bit. */
static void floor_add(int16_t *mic, const int16_t *echo, double floor_dbm0,
		      uint32_t *floor_state)
{
	int16_t hiss[UDSP_FRAME_SAMPLES];
	unsigned i;

	if (!floor_state) {
		memcpy(mic, echo, sizeof(hiss));
		return;
	}
	noise_fill(hiss, UDSP_FRAME_SAMPLES, floor_dbm0, floor_state);
	for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
		mic[i] = sample_clamp((double)echo[i] + hiss[i]);
}

/* Runs the escape shape until the first adaptation trial fires; the caller
 * takes over with the trial freshly started and its budget intact. */
static void latch_drive(struct udsp_ec *ec, struct echo_path *path,
			struct dtx_far *far, double floor_dbm0,
			uint32_t *floor_state)
{
	struct udsp_ec_nlms_info info;
	unsigned f;

	for (f = 0; f < 15u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];

		dtx_far_fill(far, ref);
		path_run(path, ref, echo);
		floor_add(mic, echo, floor_dbm0, floor_state);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.probes)
			return;
	}
}

/*
 * The budget of the trial counts active frames only. A far-end pause right
 * after the escape must not consume the trial against silence. There the
 * filter cannot move, and the collapsed residual would fake the improvement
 * verdict.
 */
static void probe_pause_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(0);
	struct dtx_far far = { .state = 0x0c0ffee1u };
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 0.0);
	latch_drive(ec, &path, &far, 0.0, NULL);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec probe trial waits out a far-end pause");
	CHECK_UINT(info.probes, 1);
	/* 1.2 s of silence is more than the whole 100-frame budget. */
	for (f = 0; f < 120u; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];

		memset(ref, 0, sizeof(ref));
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	for (f = 0; f < 4u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];

		dtx_far_fill(&far, ref);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.probes, 1);
	CHECK(info.erle_db >= 15.0f);
	udsp_ec_nlms.destroy(ec);
}

/*
 * A second latch trip while a trial runs must not restart the trial. The
 * snapshot would then hold half-adapted trial coefficients, which
 * probe_finish then cannot undo.
 */
static void probe_rearm_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(0);
	struct dtx_far far = { .state = 0x0c0ffee1u };
	uint32_t near_state = 0xfeedfaceu;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 0.0);
	latch_drive(ec, &path, &far, 0.0, NULL);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec latch cannot re-arm inside a running trial");
	CHECK_UINT(info.probes, 1);
	/* Continuous near talk over a sparse far end: the double-talk run
	 * climbs a full latch period again while the trial, which spends
	 * active frames only, is still open. */
	for (f = 0; f < 6u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		unsigned i;

		if (f % 10u == 0u)
			noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &far.state);
		else
			memset(ref, 0, sizeof(ref));
		path_run(&path, ref, echo);
		noise_fill(near, UDSP_FRAME_SAMPLES, -10.0, &near_state);
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.probes, 1);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The combined-loss NLP gate, with the hybrid loss from echo_gain times the
 * headroom. A 23 dB hybrid with a filter near 13 dB of ERLE delivers the 33 dB
 * gate; the same filter on a 6 dB or 12 dB hybrid does not. The probe freezes
 * its instances mid-convergence at one tracker ERLE. half_db sets the reference
 * loss alone.
 */
struct gate_probe {
	bool nlp_far;
	bool nlp_doubletalk;
	float erle;
	float echo_gain;
};

static void nlp_gate_probe(unsigned rate, unsigned half_db, double erl_db,
			   unsigned adapt_frames, struct gate_probe *out)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xfeedfaceu;
	unsigned frame = rate / 100u;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned f;

	udsp_ec_nlms_ref_loss_set(ec, half_db);
	path_flat_init(&path, rate, 8, erl_db);
	for (f = 0; f < adapt_frames; f++) {
		noise_fill(ref, frame, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms.freeze(ec, true);
	/* Two seconds of far single talk settle the gain tracker and the
	 * frozen filter's ERLE before the gate is read. */
	for (f = 0; f < 2u * FRAMES_PER_SECOND; f++) {
		noise_fill(ref, frame, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);
	out->nlp_far = info.nlp_active;
	out->erle = info.erle_db;
	out->echo_gain = info.echo_gain;
	noise_fill(ref, frame, -10.0, &state);
	path_run(&path, ref, echo);
	noise_fill(mic, frame, -15.0, &near_state);
	for (f = 0; f < frame; f++)
		mic[f] = sample_clamp((double)mic[f] + echo[f]);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, &info);
	out->nlp_doubletalk = !info.double_talk || info.nlp_active;
	udsp_ec_nlms.destroy(ec);
}

static void nlp_combined_loss_test(void)
{
	struct gate_probe weak;
	struct gate_probe hot;
	struct gate_probe mid;
	struct gate_probe edge;

	nlp_gate_probe(UDSP_SAMPLE_RATE, 0, 23.0, 36, &weak);
	nlp_gate_probe(UDSP_SAMPLE_RATE, 0, 6.0, 36, &hot);
	nlp_gate_probe(UDSP_SAMPLE_RATE, 0, 12.0, 36, &mid);
	nlp_gate_probe(UDSP_SAMPLE_RATE, 0, 17.0, 36, &edge);

	/* The fixtures sit under the 18 dB fast path, so the combined arm
	 * engages, and over the ERLE from which a converging filter reads
	 * echo-like. */
	CASE("lec nlp engages on the combined loss of a weak hybrid");
	CHECK(weak.erle >= 15.0f);
	CHECK(weak.erle < 17.9f);
	CHECK(weak.nlp_far);
	CHECK(!weak.nlp_doubletalk);

	CASE("lec nlp stays out of the way on a hot hybrid at the same erle");
	CHECK(hot.erle >= 15.0f);
	CHECK(hot.erle < 17.9f);
	CHECK(!hot.nlp_far);
	CHECK(!hot.nlp_doubletalk);

	/* The 12 dB hybrid stands under the 33 dB gate: a gate constant that
	 * drifts loose fails here. */
	CASE("lec nlp refuses a combined loss under the gate");
	CHECK(mid.erle >= 15.0f);
	CHECK(mid.erle < 17.9f);
	CHECK(!mid.nlp_far);
	CHECK(!mid.nlp_doubletalk);

	/* On the 17 dB hybrid the conservative bound refuses, where the raw
	 * minimum statistic would engage: the case checks that the loss term
	 * takes echo_gain times the headroom. */
	CASE("lec nlp loss term takes the conservative bound");
	CHECK(edge.erle >= 15.0f);
	CHECK(edge.erle < 17.9f);
	CHECK(!edge.nlp_far);
	CHECK(!edge.nlp_doubletalk);
}

/* One frame of the turn-taking stimulus of nlp_echo_like_test. Returns true
 * on a frame that carries the near talker. */
static bool turn_frame_fill(unsigned f, int16_t *ref, int16_t *near,
			    uint32_t *state, uint32_t *near_state)
{
	unsigned phase = f % 16u;

	if (phase < 10u)
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, state);
	else
		memset(ref, 0, UDSP_FRAME_SAMPLES * sizeof(*ref));
	if (phase < 12u || phase >= 14u)
		return false;
	noise_fill(near, UDSP_FRAME_SAMPLES, -34.0, near_state);
	return true;
}

/*
 * The frames that need echo-likeness: far DTX gaps inside an active tail
 * window, where a near talker answers. The estimate is near zero, the classic
 * bound explains the talker, and the tracker does not update. Only the
 * per-frame verdict then releases the gate; without it, comfort noise replaces
 * the talker.
 */
static void nlp_echo_like_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xa5a5c3c3u;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	unsigned engaged = 0;
	unsigned near_frames = 0;
	unsigned nlp_frames = 0;
	unsigned dt_frames = 0;
	unsigned f;

	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 23.0);
	for (f = 0; f < 36u; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms.freeze(ec, true);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.nlp_active)
			engaged++;
	}
	/* 100 ms far spurts with 60 ms pauses; the near talker answers in the
	 * pauses, 1 dB under the echo, far under the classic bound. */
	for (f = 0; f < 2u * FRAMES_PER_SECOND; f++) {
		int16_t near[UDSP_FRAME_SAMPLES];
		bool near_now = turn_frame_fill(f, ref, near, &state,
						&near_state);
		unsigned i;

		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		if (near_now) {
			for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
				mic[i] = sample_clamp((double)mic[i] +
						      near[i]);
		}
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (!near_now)
			continue;
		near_frames++;
		if (info.nlp_active)
			nlp_frames++;
		if (info.double_talk)
			dt_frames++;
	}

	CASE("lec nlp releases for a near talker the detector cannot see");
	CHECK(engaged >= 80u);
	CHECK(near_frames >= 20u);
	CHECK_UINT(nlp_frames, 0);
	CHECK(dt_frames <= 5u);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The reset on a move of the reference generation must forget all adaptive
 * state with the coefficients. A kept window ring fires the detector on a
 * reference that is gone, and a kept cross-fade writes comfort noise. A kept
 * energy sum biases the step, and a kept gain estimate skips its warm-up.
 */
static void reset_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x77665544u;
	uint32_t near_state = 0x600dcafeu;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	int16_t before[UDSP_FRAME_SAMPLES];
	double erle;
	unsigned f;

	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	for (f = 0; f < FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec reset clears the adaptive state whole");
	/* Converged, trusted and with the NLP engaged: the richest state a
	 * reset has to take apart. */
	CHECK(info.erle_db >= 18.0f);
	CHECK(info.nlp_active);
	CHECK(info.frames > 0u);
	CHECK(info.coef_energy > 0.0f);
	udsp_ec_nlms.freeze(ec, true);
	udsp_ec_nlms_reset(ec);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.frames, 0);
	CHECK_UINT(info.hold_frames, 0);
	CHECK(info.erle_db == 0.0f);
	CHECK(info.coef_energy == 0.0f);
	CHECK(info.echo_gain == 0.25f);
	CHECK(!info.double_talk);
	CHECK(!info.nlp_active);

	/* A silent reference under a loud near frame: a kept window ring
	 * fires the detector against the flushed reference, and a kept NLP
	 * cross-fade rewrites the output although the gate is down. */
	memset(ref, 0, sizeof(ref));
	noise_fill(mic, UDSP_FRAME_SAMPLES, -10.0, &near_state);
	memcpy(before, mic, sizeof(before));
	udsp_ec_nlms.run(ec, mic, ref);
	CHECK(memcmp(mic, before, sizeof(mic)) == 0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK(!info.double_talk);

	/* With freeze on, the check frames cannot re-learn inside themselves:
	 * a filter that forgot cancels nothing, sample for sample, and the
	 * gain estimator is back inside its warmup. */
	for (f = 0; f < 2u; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
		CHECK(memcmp(mic, echo, sizeof(mic)) == 0);
	}
	udsp_ec_nlms_info_get(ec, &info);
	CHECK(info.echo_gain == 0.25f);
	udsp_ec_nlms.freeze(ec, false);
	udsp_ec_nlms.nlp(ec, false);

	CASE("lec reset instance reconverges");
	erle = erle_run(ec, &path, &state, -10.0,
			FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	/* A fresh instance clears the bound with headroom; a kept ref_power,
	 * whose biased NLMS denominator caps the run far under it, fails. */
	CHECK(erle >= 45.0);
	udsp_ec_nlms.destroy(ec);
}

/* The cost of the learned bound: after a hybrid step, the tighter expectation
 * holds more of the transition than a fixed 6 dB bound. The escape must do the
 * rest. A constant change that slows the recovery further fails here. */
static void reconverge_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x5eed5eedu;
	double erle;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, 23.0);
	(void)erle_run(ec, &path, &state, -10.0, 4u * FRAMES_PER_SECOND, 20);
	/* The hybrid steps 11 dB louder with its history intact. */
	path_erl_set(&path, 12.0);
	erle = erle_run(ec, &path, &state, -10.0, 10u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate,
		  "lec learned bound recovers a stepped hybrid in bounded time");
	CHECK(erle >= 25.0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The hybrid-change lockout: the estimate predicts the echo of the path that is
 * gone, the residual carries both energies, and the detector reads it as a
 * talker. The far history moves into the replacement hybrid, so the change
 * carries echo, not silence. The line converges 4 s, so the trust count
 * completes; the seeds are entries 7 to 10 of summary_seed_get.
 */
struct lockout_result {
	double erle_late;
	unsigned hold_run;
	uint32_t probes;
	uint32_t diverge_resets;
};

static void lockout_drive(unsigned rate, uint32_t seed,
			  struct lockout_result *out)
{
	struct udsp_ec_nlms_info info;
	struct echo_path first;
	struct echo_path second;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = seed;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned held = 0;
	unsigned run = 0;
	unsigned f;

	memset(out, 0, sizeof(*out));
	udsp_ec_nlms.nlp(ec, false);
	path_late_init(&first, rate, 12.0);
	path_flat_init(&second, rate, 8, 12.0);
	(void)erle_run(ec, &first, &state, -10.0, 4u * FRAMES_PER_SECOND, 20);
	path_history_carry(&second, &first);
	udsp_ec_nlms_info_get(ec, &info);
	held = info.hold_frames;
	for (f = 0; f < 2u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, second.frame, -10.0, &state);
		path_run(&second, ref, echo);
		memcpy(mic, echo, second.frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		run = info.hold_frames != held ? run + 1u : 0u;
		if (run > out->hold_run)
			out->hold_run = run;
		held = info.hold_frames;
		/* The second second after the change. */
		if (f < FRAMES_PER_SECOND)
			continue;
		for (i = 0; i < second.frame; i++) {
			echo_energy += (double)echo[i] * echo[i];
			res_energy += (double)mic[i] * mic[i];
		}
	}
	out->erle_late = 10.0 * log10(echo_energy / res_energy);
	out->probes = info.probes;
	out->diverge_resets = info.diverge_resets;
	udsp_ec_nlms.destroy(ec);
}

static void lockout_release_test(unsigned rate)
{
	static const uint32_t seeds[] = {
		0x66db7877u, 0x0512f230u, 0xa34a6be9u, 0x4181e5a2u,
	};
	double worst_erle = 1e9;
	unsigned worst_run = 0;
	unsigned probes = 0;
	unsigned resets = 0;
	unsigned i;

	for (i = 0; i < sizeof(seeds) / sizeof(seeds[0]); i++) {
		struct lockout_result result;

		lockout_drive(rate, seeds[i], &result);
		if (result.erle_late < worst_erle)
			worst_erle = result.erle_late;
		if (result.hold_run > worst_run)
			worst_run = result.hold_run;
		probes += result.probes;
		resets += result.diverge_resets;
	}

	case_rate(rate, "lec path-change escape releases the hybrid lockout");
	/* Without the escape the instance stays latched. */
	CHECK(worst_erle >= 20.0);
	CHECK(worst_run <= 100u);
	CHECK_UINT(probes, 4);
	CHECK_UINT(resets, 0);
}

/*
 * The escape never treats a near talker over an active far end as a path
 * change: near speech adds the same energy to residual and microphone. The case
 * compares the near end out of the canceller with the near end in, sample for
 * sample. It compares the level by a correlation gain (0 dB unattenuated).
 */
static void near_end_preserve_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x0badc0deu;
	uint32_t near_state = 0xfeedfaceu;
	double product = 0.0;
	double near_energy = 0.0;
	double error_energy = 0.0;
	unsigned detected = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	for (f = 0; f < 2u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, path.frame, -10.0, &state);
		path_run(&path, ref, echo);
		/* 3 dB above the echo the -12 dB hybrid returns. */
		noise_fill(near, path.frame, -19.0, &near_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.double_talk)
			detected++;
		/* The detector needs the frame it fires in before it holds. */
		if (f < 5u)
			continue;
		for (i = 0; i < path.frame; i++) {
			double error = (double)mic[i] - near[i];

			product += (double)mic[i] * near[i];
			near_energy += (double)near[i] * near[i];
			error_energy += error * error;
		}
	}
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec keeps the near end whole through double talk");
	CHECK(detected >= FRAMES_PER_SECOND);
	/* A near end halved on the way out reads 6.02 dB here and 6.02 dB of
	 * loss below. */
	CHECK(10.0 * log10(near_energy / error_energy) >= 30.0);
	CHECK(fabs(-20.0 * log10(product / near_energy)) <= 0.5);
	CHECK_UINT(info.probes, 0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * A refused trial restores the trackers, the learned bound and the hangover
 * with the coefficients. Five seconds of near speech drive a trial that the
 * near end fails, so a probe_finish that keeps the trial fails this case. The
 * first frame of the trial does not adapt, so it reads the snapshot.
 */
static void probe_reject_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x0badc0deu;
	uint32_t near_state = 0xfeedfaceu;
	float coef_at_start = 0.0f;
	float erle_at_start = 0.0f;
	float gain_at_start = 0.0f;
	unsigned start_frame = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	for (f = 0; f < 7u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		unsigned i;

		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		noise_fill(near, UDSP_FRAME_SAMPLES, -19.0, &near_state);
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.probes == 1u && !start_frame) {
			start_frame = f + 1u;
			coef_at_start = info.coef_energy;
			erle_at_start = info.erle_db;
			gain_at_start = info.echo_gain;
		}
		/* The countdown starts on the frame after the one that armed
		 * the trial, and every frame here adapts, so the verdict comes
		 * exactly LEC_PROBE_FRAMES later. */
		if (start_frame && f + 1u == start_frame + 100u)
			break;
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec rejected trial gives the detector back");
	CHECK_UINT(info.probes, 1);
	CHECK(start_frame != 0u);
	/* Restoring the filter alone leaves the learned bound off the value it
	 * started from. */
	CHECK(info.echo_gain == gain_at_start);
	CHECK(info.coef_energy == coef_at_start);
	CHECK(info.erle_db == erle_at_start);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The countdown of a trial spends the frames that the filter adapts on, not the
 * frames with an active reference. A countdown through a freeze would close the
 * trial on a filter that never moved. After a 2 s freeze the trial is still
 * open and moving.
 */
static void probe_freeze_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x0badc0deu;
	uint32_t near_state = 0xfeedfaceu;
	float coef_at_start = 0.0f;
	float coef_frozen = 0.0f;
	float coef_released = 0.0f;
	unsigned started = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	for (f = 0; f < 10u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		unsigned i;

		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		noise_fill(near, UDSP_FRAME_SAMPLES, -19.0, &near_state);
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (!started && info.probes == 1u) {
			started = f + 1u;
			coef_at_start = info.coef_energy;
			udsp_ec_nlms.freeze(ec, true);
		}
		if (started && f + 1u == started + 2u * FRAMES_PER_SECOND) {
			coef_frozen = info.coef_energy;
			udsp_ec_nlms.freeze(ec, false);
		}
		if (started && f + 1u == started + 250u) {
			coef_released = info.coef_energy;
			break;
		}
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec trial does not spend a frozen frame");
	CHECK(started != 0u);
	CHECK_UINT(info.probes, 1);
	/* The frozen filter cannot move, so the trial cannot be judged. */
	CHECK(coef_frozen == coef_at_start);
	/* Released: the trial is still open and still adapting. */
	CHECK(coef_released != coef_at_start);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The stand-down after a refused trial (LEC_TRIAL_STANDDOWN_FRAMES), on a
 * hybrid change under a near talker who refuses each trial. The two escapes
 * draw one trial each, the second when its run completes after the first
 * refusal (first_gap). Every later trial is a latch trial that carries both
 * stand-downs, LEC_TRIAL_STANDDOWN_FRAMES apart or more (gap_min).
 */
static void probe_cooldown_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path first;
	struct echo_path second;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x13572468u;
	uint32_t near_state = 0x13572468u ^ 0xfeedfaceu;
	double product = 0.0;
	double near_energy = 0.0;
	unsigned arms[4] = { 0, 0, 0, 0 };
	unsigned last_probe = 0;
	unsigned first_gap = 0;
	unsigned gap_min = 0;
	unsigned probes = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_late_init(&first, rate, 12.0);
	path_flat_init(&second, rate, 8, 12.0);
	(void)erle_run(ec, &first, &state, -10.0, 2u * FRAMES_PER_SECOND, 20);
	path_history_carry(&second, &first);
	for (f = 0; f < 15u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, second.frame, -10.0, &state);
		path_run(&second, ref, echo);
		/* 3 dB over the echo: loud enough to refuse the trial, quiet
		 * enough to leave the escape's ratio above its bar. */
		noise_fill(near, second.frame, -19.0, &near_state);
		for (i = 0; i < second.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (info.probes != probes) {
			unsigned gap = last_probe ? f + 1u - last_probe : 0u;

			if (probes < 4u)
				arms[probes] = reading.trial_arm;
			if (probes == 1u)
				first_gap = gap;
			else if (gap && (!gap_min || gap < gap_min))
				gap_min = gap;
			last_probe = f + 1u;
			probes = info.probes;
		}
		if (f < 2u * FRAMES_PER_SECOND)
			continue;
		for (i = 0; i < second.frame; i++) {
			product += (double)mic[i] * near[i];
			near_energy += (double)near[i] * near[i];
		}
	}
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec escape stands down after a trial the line refused");
	printf("# %u trials, arms %s %s %s %s; %u frames between the first "
	       "two, %u at least between the others\n", info.probes,
	       lecr_trial_arm_name(arms[0]), lecr_trial_arm_name(arms[1]),
	       lecr_trial_arm_name(arms[2]), lecr_trial_arm_name(arms[3]),
	       first_gap, gap_min);
	CHECK(info.probes >= 3u);
	CHECK(info.probes <= 4u);
	CHECK(arms[0] == UDSP_EC_NLMS_TRIAL_ARM_COHERE ||
	      arms[0] == UDSP_EC_NLMS_TRIAL_ARM_PATH);
	CHECK_UINT(arms[1], arms[0] == UDSP_EC_NLMS_TRIAL_ARM_COHERE ?
		   UDSP_EC_NLMS_TRIAL_ARM_PATH : UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	/* The refused trial, then the run of the other escape: at least
	 * LEC_PROBE_FRAMES plus LEC_PATH_CHANGE_FRAMES. */
	CHECK(first_gap >= 100u + 20u);
	/* One whole latch cycle between the later trials, whichever arm
	 * starts them: LEC_GEIGEL_LATCH_MAX plus LEC_PROBE_FRAMES. */
	CHECK(gap_min >= 600u);
	CHECK(fabs(-20.0 * log10(product / near_energy)) <= 0.5);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The other side of the same rule. A latch trial that the line refuses on its
 * own, with no stand-down pending, hands the escape nothing to carry. A
 * stand-down there too keeps a real hybrid change after a refused latch trial
 * in lockout, and the narrow rule escapes it.
 */
static void latch_refusal_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path first;
	struct echo_path second;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x13572468u;
	uint32_t near_state = 0x13572468u ^ 0xfeedfaceu;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned held;
	unsigned run = 0;
	unsigned hold_run = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_late_init(&first, rate, 12.0);
	path_flat_init(&second, rate, 8, 12.0);
	(void)erle_run(ec, &first, &state, -10.0, 2u * FRAMES_PER_SECOND, 20);
	/* 6.5 s of near speech over the unchanged hybrid: the latch arm runs
	 * one trial and the talker refuses it. */
	for (f = 0; f < 650u; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, first.frame, -10.0, &state);
		path_run(&first, ref, echo);
		noise_fill(near, first.frame, -19.0, &near_state);
		for (i = 0; i < first.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);
	held = info.hold_frames;
	path_history_carry(&second, &first);
	for (f = 0; f < 8u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, second.frame, -10.0, &state);
		path_run(&second, ref, echo);
		memcpy(mic, echo, second.frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		run = info.hold_frames != held ? run + 1u : 0u;
		if (run > hold_run)
			hold_run = run;
		held = info.hold_frames;
		if (f < 4u * FRAMES_PER_SECOND)
			continue;
		for (i = 0; i < second.frame; i++) {
			echo_energy += (double)echo[i] * echo[i];
			res_energy += (double)mic[i] * mic[i];
		}
	}
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec latch refusal alone does not stand the escape down");
	/* A refused latch trial that arms the stand-down keeps the change held;
	 * with only the escape's own stand-down the line recovers. */
	CHECK(hold_run <= 100u);
	CHECK(10.0 * log10(echo_energy / res_energy) >= 20.0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The bootstrap false detection: a converging estimate under-reads the
 * microphone by the residual it leaves. An expectation without that term reads
 * an excursion of the echo as a talker and holds a hangover. The fixtures are
 * the seeds where that costs most. The far end talks alone, so every held frame
 * is a false detection.
 */
struct bootstrap_fixture {
	unsigned rate;
	unsigned tail_ms;
	uint32_t seed;
	bool late;
	double erle_bar;
};

static void bootstrap_hold_test(void)
{
	static const struct bootstrap_fixture fixtures[] = {
		{ UDSP_SAMPLE_RATE, 64u, 0x5897463fu, false, 25.0 },
		{ UDSP_SAMPLE_RATE, 48u, 0x8c350b4cu, true, 25.0 },
		{ UDSP_SAMPLE_RATE_WB, 48u, 0xdfb95f5bu, true, 20.0 },
	};
	unsigned held = 0;
	unsigned resets = 0;
	unsigned under_bar = 0;
	unsigned i;

	for (i = 0; i < sizeof(fixtures) / sizeof(fixtures[0]); i++) {
		struct udsp_ec_nlms_info info;
		struct echo_path path;
		struct udsp_ec *ec;
		uint32_t state = fixtures[i].seed;
		double erle;

		ec = udsp_ec_nlms.create_rate(fixtures[i].tail_ms,
					      fixtures[i].rate);
		udsp_ec_nlms.nlp(ec, false);
		if (fixtures[i].late)
			path_late_init(&path, fixtures[i].rate, 12.0);
		else
			path_composite_init(&path, fixtures[i].rate, 12.0);
		erle = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 20);
		udsp_ec_nlms_info_get(ec, &info);
		held += info.hold_frames;
		resets += info.diverge_resets;
		if (erle < fixtures[i].erle_bar)
			under_bar++;
		udsp_ec_nlms.destroy(ec);
	}

	CASE("lec bootstrap holds nothing on far-end single talk");
	/* 25, 25 and 25 held frames without the misalignment term. */
	CHECK_UINT(held, 0);
	CHECK_UINT(under_bar, 0);
	CHECK_UINT(resets, 0);
}

static void watchdog_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(64);
	uint32_t state = 0x600d1ecu;

	udsp_ec_nlms.nlp(ec, false);
	path_composite_init(&path, UDSP_SAMPLE_RATE, 12.0);
	(void)erle_run(ec, &path, &state, -10.0, 2u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms_info_get(ec, &info);

	/* A converged filter carries the energy of the path it learned
	 * (10^-1.2 for a 12 dB ERL), an order of magnitude below the
	 * watchdog bound. */
	CASE("lec watchdog leaves a converged filter alone");
	CHECK_UINT(info.diverge_resets, 0);
	CHECK(info.coef_energy > 0.03f);
	CHECK(info.coef_energy < 0.13f);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The create_rate contract (ec.h) and the wideband tail cap. create() must
 * stay the 8 kHz form of the same call, sample for sample, because every
 * 8 kHz line reaches the canceller through it.
 */
static void create_rate_test(void)
{
	static const unsigned tails[] = { 0u, 15u, 16u, 48u, 64u, 129u };
	struct udsp_ec_nlms_info wide;
	struct udsp_ec_nlms_info narrow;
	struct echo_path path;
	struct udsp_ec *plain;
	struct udsp_ec *rated;
	struct udsp_ec *ec;
	uint32_t state = 0x4c454331u;
	unsigned differences = 0;
	unsigned i;

	CASE("lec create_rate builds the wideband instance");
	ec = udsp_ec_nlms.create_rate(48, UDSP_SAMPLE_RATE_WB);
	CHECK(ec != NULL);
	if (!ec)
		return;
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 48);
	udsp_ec_nlms_info_get(ec, &wide);
	CHECK_UINT(wide.taps, 768);
	udsp_ec_nlms.destroy(ec);
	ec = udsp_ec_nlms.create_rate(0, UDSP_SAMPLE_RATE_WB);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 64);
	udsp_ec_nlms_info_get(ec, &wide);
	CHECK_UINT(wide.taps, 1024);
	udsp_ec_nlms.destroy(ec);

	/* ec.h states two line rates and nothing else; a handle at any other
	 * rate has no canceller rather than a filter of the wrong length. */
	CASE("lec create_rate refuses a rate that is not a line rate");
	CHECK(udsp_ec_nlms.create_rate(48, 0) == NULL);
	CHECK(udsp_ec_nlms.create_rate(48, 4000) == NULL);
	CHECK(udsp_ec_nlms.create_rate(48, 11025) == NULL);
	CHECK(udsp_ec_nlms.create_rate(48, 32000) == NULL);
	CHECK(udsp_ec_nlms.create_rate(48, 48000) == NULL);

	CASE("lec wideband tail clamps to the 64 ms cap");
	ec = udsp_ec_nlms.create_rate(128, UDSP_SAMPLE_RATE_WB);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 64);
	udsp_ec_nlms_info_get(ec, &wide);
	CHECK_UINT(wide.taps, 1024);
	udsp_ec_nlms.destroy(ec);
	ec = udsp_ec_nlms.create_rate(64, UDSP_SAMPLE_RATE_WB);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 64);
	udsp_ec_nlms.destroy(ec);
	/* 65 and 66 ms are the first two millisecond steps past the cap. A
	 * clamp that compares one too high still passes a 128 ms probe and
	 * fails here. */
	ec = udsp_ec_nlms.create_rate(65, UDSP_SAMPLE_RATE_WB);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 64);
	udsp_ec_nlms_info_get(ec, &wide);
	CHECK_UINT(wide.taps, 1024);
	udsp_ec_nlms.destroy(ec);
	ec = udsp_ec_nlms.create_rate(66, UDSP_SAMPLE_RATE_WB);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 64);
	udsp_ec_nlms_info_get(ec, &wide);
	CHECK_UINT(wide.taps, 1024);
	udsp_ec_nlms.destroy(ec);
	/* The floor of the range is the narrowband one at both rates. */
	ec = udsp_ec_nlms.create_rate(15, UDSP_SAMPLE_RATE_WB);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 16);
	udsp_ec_nlms_info_get(ec, &wide);
	CHECK_UINT(wide.taps, 256);
	udsp_ec_nlms.destroy(ec);

	CASE("lec narrowband tail range survives the wideband cap");
	ec = udsp_ec_nlms.create_rate(128, UDSP_SAMPLE_RATE);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 128);
	udsp_ec_nlms_info_get(ec, &narrow);
	CHECK_UINT(narrow.taps, 1024);
	udsp_ec_nlms.destroy(ec);
	ec = udsp_ec_nlms.create_rate(129, UDSP_SAMPLE_RATE);
	CHECK_UINT(udsp_ec_nlms_tail_get(ec), 128);
	udsp_ec_nlms.destroy(ec);

	CASE("lec create is create_rate at 8 kHz");
	for (i = 0; i < sizeof(tails) / sizeof(tails[0]); i++) {
		plain = udsp_ec_nlms.create(tails[i]);
		rated = udsp_ec_nlms.create_rate(tails[i], UDSP_SAMPLE_RATE);
		udsp_ec_nlms_info_get(plain, &narrow);
		udsp_ec_nlms_info_get(rated, &wide);
		CHECK_UINT(wide.tail_ms, narrow.tail_ms);
		CHECK_UINT(wide.taps, narrow.taps);
		udsp_ec_nlms.destroy(plain);
		udsp_ec_nlms.destroy(rated);
	}
	plain = udsp_ec_nlms.create(48);
	rated = udsp_ec_nlms.create_rate(48, UDSP_SAMPLE_RATE);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	for (i = 0; i < FRAMES_PER_SECOND; i++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t plain_out[UDSP_FRAME_MAX];
		int16_t rated_out[UDSP_FRAME_MAX];

		noise_fill(ref, path.frame, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(plain_out, echo, path.frame * sizeof(*plain_out));
		memcpy(rated_out, echo, path.frame * sizeof(*rated_out));
		udsp_ec_nlms.run(plain, plain_out, ref);
		udsp_ec_nlms.run(rated, rated_out, ref);
		if (memcmp(plain_out, rated_out,
			   path.frame * sizeof(*plain_out)))
			differences++;
	}
	CHECK_UINT(differences, 0);
	udsp_ec_nlms_info_get(plain, &narrow);
	udsp_ec_nlms_info_get(rated, &wide);
	CHECK(narrow.coef_energy == wide.coef_energy);
	CHECK(narrow.erle_db == wide.erle_db);
	udsp_ec_nlms.destroy(plain);
	udsp_ec_nlms.destroy(rated);
}

/* Power response of a hybrid at one frequency, straight from the taps: the
 * oracle that says which band the shape occupies. */
static double path_response_db(const struct echo_path *path, double hz,
			       unsigned rate)
{
	double re = 0.0;
	double im = 0.0;
	unsigned k;

	for (k = 0; k < path->taps; k++) {
		double angle = 2.0 * M_PI * hz * (double)k / rate;

		re += (double)path->h[k] * cos(angle);
		im -= (double)path->h[k] * sin(angle);
	}
	return 10.0 * log10(re * re + im * im);
}

/*
 * The upper octave. Every other hybrid in the suite lives under 4 kHz. No
 * other case therefore asks the wideband filter to learn a response that the
 * narrowband line cannot carry at all. The decision on a two-band fallback
 * depends on this case. A band-split canceller must hold coherence across
 * 4 kHz to reach the bar that the full-band filter reaches here.
 */
static void upper_band_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x60c7a5e5u;
	double erle;

	path_upper_init(&path, UDSP_SAMPLE_RATE_WB, 12.0);

	/* The band pin: without it a shape that quietly drifted into the low
	 * band would still converge and the case would prove nothing. */
	CASE("lec upper-band hybrid sits above the narrowband band edge");
	CHECK_UINT(path.taps, 160);
	CHECK(path_response_db(&path, 6000.0, UDSP_SAMPLE_RATE_WB) -
	      path_response_db(&path, 1000.0, UDSP_SAMPLE_RATE_WB) >= 20.0);
	CHECK(path_response_db(&path, 6000.0, UDSP_SAMPLE_RATE_WB) -
	      path_response_db(&path, 3000.0, UDSP_SAMPLE_RATE_WB) >= 10.0);

	CASE("lec upper-band hybrid converges at 16 kHz");
	ec = udsp_ec_nlms.create_rate(48, UDSP_SAMPLE_RATE_WB);
	udsp_ec_nlms.nlp(ec, false);
	erle = erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 20);
	CHECK(erle >= 25.0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The tap that the fused pass contends for: the windows of two consecutive
 * samples collide on the last tap of the older one. The ring keeps one sample
 * more than the window to avoid it (hist_push). This hybrid puts the whole echo
 * on the last tap, so a ring one sample short cannot reach the bar.
 */
static void last_tap_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(16, rate);
	uint32_t state = 0x1a577a95u;
	unsigned taps = 16u * (rate / 1000u);
	double erle;

	udsp_ec_nlms.nlp(ec, false);
	/* A hybrid the shape helpers cannot state in whole milliseconds: one
	 * reflection at the last tap of a 16 ms tail. */
	path_geometry_set(&path, rate);
	path.taps = taps;
	path.h[taps - 1u] = 1.0f;
	path_erl_set(&path, 12.0);
	erle = erle_run(ec, &path, &state, -10.0, 2u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec cancels an echo that lands on the last tap");
	CHECK_UINT(info.taps, taps);
	CHECK(erle >= 25.0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The update that the fused pass cannot carry: the last sample of a frame has
 * no estimate to fuse with. A reference silent but for the frame's last sample
 * makes that update the only one, so the filter moves only if the trailing
 * update runs.
 */
static void last_sample_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(16, rate);
	int16_t ref[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned frame = rate / 100u;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 0, 12.0);
	memset(ref, 0, sizeof(ref));
	ref[frame - 1u] = 32767;
	path_run(&path, ref, mic);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec adapts on the last sample of a frame");
	CHECK_UINT(info.frames, 1);
	CHECK_UINT(info.hold_frames, 0);
	CHECK(info.coef_energy > 0.0f);
	udsp_ec_nlms.destroy(ec);
}

/* The equivalence suite runs each scenario through the scalar and the Advanced
 * SIMD loops on the same frames. It bounds the drift of the samples, the ERLE
 * and each decision. A target with one path prints a skip note and passes. */
#define EQUIV_DELTA_MAX		4u
#define EQUIV_ERLE_DELTA_DB	0.1
#define EQUIV_MEASURE_FROM	20u

/* Bounds of the internal state at the end of a scenario. echo_gain
 * compares exactly, because only a split trial verdict moves it, and that
 * verdict fails the coefficient energy bound first. */
#define EQUIV_COEF_REL		1e-5
#define EQUIV_ERLE_STATE_DB	0.05
#define EQUIV_COVER_CASE	"lec equivalence scenarios reach every decision"

enum equiv_hybrid {
	EQUIV_FLAT,
	EQUIV_DISPERSIVE,
	EQUIV_COMPOSITE,
	EQUIV_LATE,
	EQUIV_UPPER,
};

struct equiv_scenario {
	const char *name;
	unsigned rate;
	unsigned tail_ms;
	uint32_t seed;
	unsigned frames;
	enum equiv_hybrid hybrid;
	enum equiv_hybrid change_to;
	unsigned change_at;
	double erl_db;
	/* The return loss of the hybrid the line changes to, when it differs
	 * from the one it starts on. 0 means the two match, which is what
	 * every scenario but the amplifying one says. */
	double change_erl_db;
	double near_dbm0;
	unsigned near_from;
	unsigned near_to;
	bool nlp;
	/* What this scenario must make the canceller do, so that a scenario
	 * cannot pass on the decisions of another. coef_live: the state
	 * comparison ran on a filter that carries energy. */
	unsigned min_probes;
	unsigned min_resets;
	unsigned min_hold_run;
	bool coef_live;
};

struct equiv_result {
	unsigned max_delta;
	unsigned diff_frames;
	unsigned decision_frames;
	unsigned dt_frames;
	unsigned nlp_frames;
	unsigned max_hold_run;
	bool coef_live;
	double erle_scalar;
	double erle_simd;
	struct udsp_ec_nlms_info scalar;
	struct udsp_ec_nlms_info simd;
};

/* What the scenarios between them must have made the canceller do, so the
 * per-frame decision oracle above is exercised rather than vacuous. */
struct equiv_fired {
	uint32_t probes;
	uint32_t resets;
	uint32_t holds;
	unsigned dt_frames;
	unsigned nlp_frames;
};

static void equiv_hybrid_build(struct echo_path *path, enum equiv_hybrid kind,
			       unsigned rate, double erl_db)
{
	switch (kind) {
	case EQUIV_DISPERSIVE:
		path_dispersive_init(path, rate, erl_db);
		return;
	case EQUIV_COMPOSITE:
		path_composite_init(path, rate, erl_db);
		return;
	case EQUIV_LATE:
		path_late_init(path, rate, erl_db);
		return;
	case EQUIV_UPPER:
		path_upper_init(path, rate, erl_db);
		return;
	default:
		path_flat_init(path, rate, 8, erl_db);
		return;
	}
}

static double rel_diff_get(float first, float second)
{
	double top = fabs((double)first) > fabs((double)second) ?
		     fabs((double)first) : fabs((double)second);

	if (top == 0.0)
		return 0.0;
	return fabs((double)first - (double)second) / top;
}

static void equiv_run(const struct equiv_scenario *s, struct equiv_result *out)
{
	struct udsp_ec *scalar = udsp_ec_nlms.create_rate(s->tail_ms, s->rate);
	struct udsp_ec *simd = udsp_ec_nlms.create_rate(s->tail_ms, s->rate);
	struct echo_path first;
	struct echo_path second;
	struct echo_path *active = &first;
	uint32_t state = s->seed;
	uint32_t near_state = s->seed ^ 0xfeedfaceu;
	unsigned frame = s->rate / 100u;
	double echo_energy = 0.0;
	double scalar_energy = 0.0;
	double simd_energy = 0.0;
	uint32_t held = 0;
	unsigned run = 0;
	unsigned f;

	memset(out, 0, sizeof(*out));
	udsp_ec_nlms.nlp(scalar, s->nlp);
	udsp_ec_nlms.nlp(simd, s->nlp);
	equiv_hybrid_build(&first, s->hybrid, s->rate, s->erl_db);
	equiv_hybrid_build(&second, s->change_to, s->rate,
			   s->change_erl_db != 0.0 ? s->change_erl_db :
						     s->erl_db);
	for (f = 0; f < s->frames; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		int16_t scalar_out[UDSP_FRAME_MAX];
		int16_t simd_out[UDSP_FRAME_MAX];
		unsigned i;

		if (s->change_at && f == s->change_at) {
			path_history_carry(&second, &first);
			active = &second;
		}
		noise_fill(ref, frame, -10.0, &state);
		path_run(active, ref, echo);
		memcpy(mic, echo, frame * sizeof(*mic));
		if (s->near_dbm0 < 0.0 && f >= s->near_from && f < s->near_to) {
			int16_t near[UDSP_FRAME_MAX];

			noise_fill(near, frame, s->near_dbm0, &near_state);
			for (i = 0; i < frame; i++)
				mic[i] = sample_clamp((double)mic[i] + near[i]);
		}
		memcpy(scalar_out, mic, frame * sizeof(*mic));
		memcpy(simd_out, mic, frame * sizeof(*mic));
		udsp_ec_nlms_path_set(UDSP_EC_NLMS_PATH_SCALAR);
		udsp_ec_nlms.run(scalar, scalar_out, ref);
		udsp_ec_nlms_info_get(scalar, &out->scalar);
		udsp_ec_nlms_path_set(UDSP_EC_NLMS_PATH_SIMD);
		udsp_ec_nlms.run(simd, simd_out, ref);
		udsp_ec_nlms_info_get(simd, &out->simd);
		/*
		 * Every decision on every frame: a probe that fires on frame
		 * 100 in one arm and on frame 200 in the other leaves the
		 * totals equal and the cancellers different.
		 */
		if (out->scalar.double_talk != out->simd.double_talk ||
		    out->scalar.nlp_active != out->simd.nlp_active ||
		    out->scalar.probes != out->simd.probes ||
		    out->scalar.diverge_resets != out->simd.diverge_resets ||
		    out->scalar.hold_frames != out->simd.hold_frames)
			out->decision_frames++;
		if (out->scalar.double_talk)
			out->dt_frames++;
		if (out->scalar.nlp_active)
			out->nlp_frames++;
		/* The longest unbroken hold, which is what says the detector
		 * latched rather than flickering (the lockout_drive idiom). */
		run = out->scalar.hold_frames != held ? run + 1u : 0u;
		if (run > out->max_hold_run)
			out->max_hold_run = run;
		held = out->scalar.hold_frames;
		if (memcmp(scalar_out, simd_out, frame * sizeof(*mic)))
			out->diff_frames++;
		for (i = 0; i < frame; i++) {
			unsigned delta = (unsigned)abs(scalar_out[i] -
						       simd_out[i]);

			if (delta > out->max_delta)
				out->max_delta = delta;
		}
		if (f < EQUIV_MEASURE_FROM)
			continue;
		for (i = 0; i < frame; i++) {
			echo_energy += (double)echo[i] * echo[i];
			scalar_energy += (double)scalar_out[i] * scalar_out[i];
			simd_energy += (double)simd_out[i] * simd_out[i];
		}
	}
	out->erle_scalar = 10.0 * log10(echo_energy / scalar_energy);
	out->erle_simd = 10.0 * log10(echo_energy / simd_energy);
	/* The state comparison is only a comparison if there is state: two
	 * zeroes satisfy a relative bound without meeting. */
	out->coef_live = out->scalar.coef_energy > 0.0f ||
			 out->simd.coef_energy > 0.0f;
	udsp_ec_nlms.destroy(scalar);
	udsp_ec_nlms.destroy(simd);
}

/* The scenarios: convergence, the wideband worst case at 64 ms, detector and
 * NLP under a near burst, the path-change escape. Then 136 taps at 8 kHz for
 * the four-wide tail, a refused trial, the upper-octave hybrid. */
static const struct equiv_scenario equiv_scenarios[] = {
	{
		.name = "lec paths agree on the flat hybrid",
		.rate = UDSP_SAMPLE_RATE, .tail_ms = 48,
		.seed = 0x13572468u, .frames = 300,
		.hybrid = EQUIV_FLAT, .change_to = EQUIV_FLAT, .erl_db = 12.0,
	},
	{
		.name = "lec paths agree at the wideband 64 ms tail",
		.rate = UDSP_SAMPLE_RATE_WB, .tail_ms = 64,
		.seed = 0x0f1e2d3cu, .frames = 300,
		.hybrid = EQUIV_COMPOSITE, .change_to = EQUIV_COMPOSITE,
		.erl_db = 12.0,
	},
	{
		.name = "lec paths agree through double talk and the nlp",
		.rate = UDSP_SAMPLE_RATE_WB, .tail_ms = 48,
		.seed = 0x0badc0deu, .frames = 400,
		.hybrid = EQUIV_COMPOSITE, .change_to = EQUIV_COMPOSITE,
		.erl_db = 12.0,
		.near_dbm0 = -19.0, .near_from = 200, .near_to = 300,
		.nlp = true,
	},
	{
		.name = "lec paths agree across a hybrid change",
		.rate = UDSP_SAMPLE_RATE_WB, .tail_ms = 48,
		.seed = 0xc8a3febeu, .frames = 400,
		.hybrid = EQUIV_LATE, .change_to = EQUIV_FLAT,
		.change_at = 200, .erl_db = 12.0,
	},
	{
		.name = "lec paths agree on a tap count with a remainder",
		.rate = UDSP_SAMPLE_RATE, .tail_ms = 17,
		.seed = 0x5a5a1234u, .frames = 300,
		.hybrid = EQUIV_DISPERSIVE, .change_to = EQUIV_DISPERSIVE,
		.erl_db = 12.0,
	},
	{
		.name = "lec paths agree through a refused trial",
		.rate = UDSP_SAMPLE_RATE, .tail_ms = 48,
		.seed = 0xfeedfaceu, .frames = 800,
		.hybrid = EQUIV_FLAT, .change_to = EQUIV_FLAT, .erl_db = 12.0,
		.near_dbm0 = -19.0, .near_from = 150, .near_to = 800,
		/* Five seconds of near speech over an active far end is the
		 * latch arm's own trial (sustained_doubletalk_test). */
		.min_probes = 1, .coef_live = true,
	},
	{
		.name = "lec paths agree on the upper-octave hybrid",
		.rate = UDSP_SAMPLE_RATE_WB, .tail_ms = 48,
		.seed = 0x60c7a5e5u, .frames = 300,
		.hybrid = EQUIV_UPPER, .change_to = EQUIV_UPPER, .erl_db = 12.0,
	},
	/* An amplifying hybrid holds from the first frame, and the latch trial
	 * adapts toward more energy than a passive path holds. The watchdog
	 * fires: a hold for the run, a trial and a reset. */
	{
		.name = "lec paths agree on an amplifying hybrid",
		.rate = UDSP_SAMPLE_RATE, .tail_ms = 48,
		.seed = 0x0c0ffee1u, .frames = 1500,
		.hybrid = EQUIV_FLAT, .change_to = EQUIV_FLAT, .erl_db = -8.0,
		/* The step control slows the runaway, so the amplifying phase
		 * runs past the reset; a passive last 2 s ends the run on a
		 * converged filter, not on zeroes. */
		.change_at = 1300, .change_erl_db = 12.0,
		/* The hold runs unbroken until the latch, LEC_GEIGEL_LATCH_MAX
		 * frames. The trial then adapts on an echo louder than its
		 * reference, so the watchdog fires as well. */
		.min_probes = 1, .min_resets = 1, .min_hold_run = 500,
		.coef_live = true,
	},
};

static void equivalence_test(void)
{
	unsigned count = sizeof(equiv_scenarios) / sizeof(equiv_scenarios[0]);
	enum udsp_ec_nlms_path outer = udsp_ec_nlms_path_get();
	struct equiv_fired fired;
	unsigned i;

	memset(&fired, 0, sizeof(fired));
	if (!udsp_ec_nlms_path_set(UDSP_EC_NLMS_PATH_SIMD)) {
		for (i = 0; i <= count; i++) {
			CASE(i < count ? equiv_scenarios[i].name :
					 EQUIV_COVER_CASE);
			printf("# skip: this build has the %s path only\n",
			       udsp_ec_nlms_path_name(udsp_ec_nlms_path_get()));
		}
		return;
	}
	for (i = 0; i < count; i++) {
		struct equiv_result result;

		equiv_run(&equiv_scenarios[i], &result);
		CASE(equiv_scenarios[i].name);
		/* The record the on-target run leaves behind: the bounds are
		 * asserted, the measurements are printed beside them. */
		printf("# %u taps, worst sample delta %u of %u counts, %u of %u frames differ, erle %.2f / %.2f dB\n",
		       result.scalar.taps, result.max_delta, EQUIV_DELTA_MAX,
		       result.diff_frames, equiv_scenarios[i].frames,
		       result.erle_scalar, result.erle_simd);
		printf("#   fired: %u probes, %u resets, %u holds (longest run %u), %u dt frames, %u nlp frames; state: coef %.2e, erle %.2e dB\n",
		       result.scalar.probes, result.scalar.diverge_resets,
		       result.scalar.hold_frames, result.max_hold_run,
		       result.dt_frames, result.nlp_frames,
		       rel_diff_get(result.scalar.coef_energy,
				    result.simd.coef_energy),
		       fabs((double)result.scalar.erle_db -
			    result.simd.erle_db));
		CHECK(result.max_delta <= EQUIV_DELTA_MAX);
		CHECK(fabs(result.erle_scalar - result.erle_simd) <=
		      EQUIV_ERLE_DELTA_DB);
		/* A decision that came out differently is not a rounding
		 * difference: it is a different canceller. Compared on the
		 * frame it happened, not as a total that offsets hide in. */
		CHECK_UINT(result.decision_frames, 0);
		CHECK_UINT(result.simd.probes, result.scalar.probes);
		CHECK_UINT(result.simd.diverge_resets,
			   result.scalar.diverge_resets);
		CHECK_UINT(result.simd.hold_frames, result.scalar.hold_frames);
		CHECK_UINT(result.simd.taps, result.scalar.taps);
		/* The state the next frame would run on, not only the samples
		 * this one emitted. */
		CHECK(rel_diff_get(result.scalar.coef_energy,
				   result.simd.coef_energy) <=
		      EQUIV_COEF_REL);
		CHECK(result.simd.echo_gain == result.scalar.echo_gain);
		CHECK(fabs((double)result.scalar.erle_db -
			   result.simd.erle_db) <= EQUIV_ERLE_STATE_DB);
		/* This scenario's own decisions, so it cannot go quiet and
		 * lean on another one's totals, and the state guard above
		 * cannot pass on a filter that holds nothing. */
		CHECK(result.scalar.probes >= equiv_scenarios[i].min_probes);
		CHECK(result.scalar.diverge_resets >=
		      equiv_scenarios[i].min_resets);
		CHECK(result.max_hold_run >= equiv_scenarios[i].min_hold_run);
		CHECK(!equiv_scenarios[i].coef_live || result.coef_live);
		fired.probes += result.scalar.probes;
		fired.resets += result.scalar.diverge_resets;
		fired.holds += result.scalar.hold_frames;
		fired.dt_frames += result.dt_frames;
		fired.nlp_frames += result.nlp_frames;
	}
	CASE(EQUIV_COVER_CASE);
	printf("# %u probes, %u resets, %u held frames, %u dt frames, %u nlp frames over %u scenarios\n",
	       fired.probes, fired.resets, fired.holds, fired.dt_frames,
	       fired.nlp_frames, count);
	CHECK(fired.probes > 0u);
	CHECK(fired.resets > 0u);
	CHECK(fired.holds > 0u);
	CHECK(fired.dt_frames > 0u);
	CHECK(fired.nlp_frames > 0u);
	udsp_ec_nlms_path_set(outer);
}

/* The weak hybrid: a 26 to 30 dB line, the echo of a quiet far end about 10 dB
 * over the near floor. The fault depends on that ratio, so it sets the row.
 * Each bar fails without the floor term and the step control. */
#define WEAK_FAR_DBM0		-25.0
#define WEAK_FLOOR_DBM0		-55.0
#define WEAK_ERL_DB		28.0
#define WEAK_WINDOW		10u

/* The burst bar: with the step control the worst window of talkover_test stands
 * near pass-through, and 3 dB leaves room for the noise of the regression
 * estimate. */
#define TALK_BURST_DB		3.0

struct weak_stat {
	unsigned active;
	unsigned adapt;
	unsigned flagged;
	unsigned windows;
	double worst_window;
	double echo;
	double residual;
	double floor;
};

/*
 * One frame through the weak hybrid with the line's own floor under it. near
 * is NULL for single talk. The statistics split on the canceller's own
 * activity boundary, so a frame the canceller ignores is not counted against
 * it.
 */
static void weak_step(struct udsp_ec *ec, struct echo_path *path,
		      struct word_source *far, struct word_source *near,
		      uint32_t *floor_state, struct weak_stat *st,
		      uint32_t *held, double *window, bool floor_quiet)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t talk[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
	struct udsp_ec_nlms_info info;
	unsigned frame = path->frame;
	double mic_energy = 0.0;
	unsigned i;

	word_fill(far, ref, frame);
	path_run(path, ref, echo);
	memset(talk, 0, frame * sizeof(*talk));
	if (near)
		word_fill(near, talk, frame);
	if (floor_quiet)
		memset(hiss, 0, frame * sizeof(*hiss));
	else
		noise_fill(hiss, frame, WEAK_FLOOR_DBM0, floor_state);
	for (i = 0; i < frame; i++)
		mic[i] = sample_clamp((double)echo[i] + talk[i] + hiss[i]);
	memcpy(out, mic, frame * sizeof(*mic));
	udsp_ec_nlms.run(ec, out, ref);
	udsp_ec_nlms_info_get(ec, &info);
	for (i = 0; i < frame; i++) {
		mic_energy += (double)mic[i] * mic[i];
		st->echo += (double)echo[i] * echo[i];
		st->residual += (double)out[i] * out[i];
		st->floor += (double)hiss[i] * hiss[i];
		window[0] += (double)out[i] * out[i];
	}
	window[1] += mic_energy;
	/* A near source in one of its own pauses is not double talk, and a
	 * frame the canceller reads as silent is not one it could learn
	 * from: the counters take the frames each verdict is about. */
	if (frame_loud(ref, frame) && (!near || frame_loud(talk, frame))) {
		st->active++;
		if (info.hold_frames == *held)
			st->adapt++;
		if (info.double_talk)
			st->flagged++;
	}
	*held = info.hold_frames;
}

/*
 * What the filter takes out: the residual less the injected floor energy. Floor
 * and misalignment are uncorrelated, so their energies add and the subtraction
 * is unbiased. The acceptance is stated in this figure, because the floor
 * bounds the delivered ratio whatever the canceller does.
 */
static double weak_filter_erle_get(double echo, double residual, double floor)
{
	double left = residual - floor;

	if (left < 1e-6)
		return 200.0;
	return 10.0 * log10(echo / left);
}

static void weak_hybrid_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(rate == UDSP_SAMPLE_RATE ?
						      48u : 64u, rate);
	struct word_source far;
	struct word_source near;
	struct weak_stat single;
	struct weak_stat both;
	uint32_t floor_state = 0x0badcafeu;
	uint32_t held = 0;
	double window[2] = { 0.0, 0.0 };
	unsigned f;

	memset(&single, 0, sizeof(single));
	memset(&both, 0, sizeof(both));
	single.worst_window = 200.0;
	udsp_ec_nlms.nlp(ec, false);
	path_weak_init(&path, rate, WEAK_ERL_DB);
	word_source_init(&far, rate, WEAK_FAR_DBM0, 0x13572468u);
	word_source_init(&near, rate, WEAK_FAR_DBM0, 0xfeedfaceu);
	for (f = 0; f < 20u * FRAMES_PER_SECOND; f++) {
		weak_step(ec, &path, &far, NULL, &floor_state, &single, &held,
			  window, false);
		if ((f + 1u) % WEAK_WINDOW || f < 5u * FRAMES_PER_SECOND)
			continue;
		if (window[1] > 0.0 && window[0] > 0.0) {
			double db = 10.0 * log10(window[1] / window[0]);

			single.windows++;
			if (db < single.worst_window)
				single.worst_window = db;
		}
		window[0] = window[1] = 0.0;
	}

	case_rate(rate, "lec adapts through single talk on the weak hybrid");
	/* The far end alone is on the line, so every far-active frame is one
	 * the filter must be allowed to learn from. */
	CHECK(single.active > 500u);
	CHECK(single.adapt * 10u >= single.active * 9u);
	CHECK(single.flagged * 20u <= single.active);
	/* The canceller must not hand the line more energy than it found in
	 * any window of it: the far end hears that as a repeated syllable. */
	CHECK(single.windows > 50u);
	CHECK(single.worst_window >= -1.0);
	CHECK(weak_filter_erle_get(single.echo, single.residual,
				   single.floor) >= 8.0);

	/* The near end joins, at the far end's level, on the same hybrid and
	 * the same floor: the detector that stopped hearing the floor must
	 * still hear a talker. */
	for (f = 0; f < 10u * FRAMES_PER_SECOND; f++)
		weak_step(ec, &path, &far, &near, &floor_state, &both, &held,
			  window, false);

	case_rate(rate, "lec still holds for a near talker on the weak hybrid");
	CHECK(both.active > 100u);
	CHECK(both.flagged * 10u >= both.active * 8u);
	CHECK((both.active - both.adapt) * 10u >= both.active * 8u);
	/* No escape trial and therefore no divergence: the verdicts here are
	 * the true ones and the pauses of ordinary speech pay the run down. */
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

#define DIVERGE_AMP_DB		-8.0
#define DIVERGE_WORDS_FRAMES	(5u * FRAMES_PER_SECOND)
#define DIVERGE_AMP_FRAMES	(30u * FRAMES_PER_SECOND)

/*
 * The divergence line: far words converge the filter on the weak hybrid for 5
 * s. Then the hybrid of watchdog_recovery_test, flat and 8 dB amplifying,
 * replaces it under a white far end. The talker guard reads its echo as echo,
 * and no passive path returns it.
 */
struct divergence_line {
	struct echo_path weak;
	struct echo_path amp;
	struct word_source far;
	struct udsp_ec *ec;
	uint32_t state;
};

static void divergence_line_init(struct divergence_line *d, unsigned rate)
{
	d->ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(d->ec, false);
	path_weak_init(&d->weak, rate, WEAK_ERL_DB);
	path_flat_init(&d->amp, rate, 8, DIVERGE_AMP_DB);
	word_source_init(&d->far, rate, WEAK_FAR_DBM0, 0x13572468u);
	d->state = 0x13572468u;
}

/* Frame f of the divergence line. The hybrid that amplifies takes over on
 * frame DIVERGE_WORDS_FRAMES. */
static void divergence_frame_run(struct divergence_line *d, unsigned f)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];

	if (f < DIVERGE_WORDS_FRAMES) {
		word_fill(&d->far, ref, d->weak.frame);
		path_run(&d->weak, ref, mic);
		udsp_ec_nlms.run(d->ec, mic, ref);
		return;
	}
	if (f == DIVERGE_WORDS_FRAMES)
		path_history_carry(&d->amp, &d->weak);
	noise_fill(ref, d->amp.frame, WEAK_FAR_DBM0, &d->state);
	path_run(&d->amp, ref, mic);
	udsp_ec_nlms.run(d->ec, mic, ref);
}

/*
 * On the hybrid that amplifies, a latch trial drives the coefficients through
 * LEC_COEF_ENERGY_MAX, and probe_diverged restores the filter of the weak
 * hybrid. The far end runs without a pause. The line must run a trial and a
 * reset, never leave the filter zeroed, and reset at most once per trial.
 */
static void probe_divergence_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct divergence_line d;
	unsigned zeroed = 0;
	bool alive = false;
	unsigned f;

	divergence_line_init(&d, rate);
	for (f = 0; f < DIVERGE_WORDS_FRAMES + DIVERGE_AMP_FRAMES; f++) {
		divergence_frame_run(&d, f);
		udsp_ec_nlms_info_get(d.ec, &info);
		if (info.coef_energy > 0.0f)
			alive = true;
		else if (alive)
			zeroed++;
	}
	udsp_ec_nlms.destroy(d.ec);

	case_rate(rate, "lec undoes a trial the watchdog stops");
	printf("# %u trials, %u resets, %u zeroed frames\n", info.probes,
	       info.diverge_resets, zeroed);
	/* The latch fires: five unbroken seconds of an echo louder than any
	 * expectation end in a trial. */
	CHECK(info.probes >= 1u);
	/* A trial is exempt from the talk scale, so it adapts at the full step
	 * toward the hybrid that amplifies and reaches the fault. */
	CHECK(info.diverge_resets >= 1u);
	/* The filter is never left dead under a running trial. */
	CHECK_UINT(zeroed, 0);
	/*
	 * And one trial can trip the watchdog at most once, because the trip
	 * ends the trial.
	 */
	CHECK(info.diverge_resets <= info.probes);
}

/*
 * The floor has one way in here: the first reference-silent frame is taken
 * whole, so a near talkspurt in a far pause sets the floor at its level. The
 * step control reads that floor; without the stale rule of an active frame's
 * residual, the filter never adapts again while the far end talks.
 */
static void noise_floor_stale_test(void)
{
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x51ade0f1u;
	uint32_t near_state = 0x600dcafeu;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	/* One far-end pause carrying near-end speech, before any quiet pause
	 * has measured the real floor. */
	memset(ref, 0, sizeof(ref));
	noise_fill(mic, UDSP_FRAME_SAMPLES, -10.0, &near_state);
	udsp_ec_nlms.run(ec, mic, ref);

	CASE("lec drops a near-end floor the residual falsifies");
	CHECK(erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20) >= 45.0);
	udsp_ec_nlms.destroy(ec);
}

/* A quiet 10 ms frame is one realization of a near end, not evidence of a stale
 * floor. A floor dropped under a talking far end does not come back. Every
 * tenth frame of the weak line is quiet; the line must keep its floor. */
#define WEAK_OUTLIER_EVERY	10u

static void noise_floor_outlier_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	struct word_source far;
	struct weak_stat st;
	uint32_t floor_state = 0x0badcafeu;
	uint32_t held = 0;
	double window[2] = { 0.0, 0.0 };
	unsigned f;

	memset(&st, 0, sizeof(st));
	udsp_ec_nlms.nlp(ec, false);
	path_weak_init(&path, rate, WEAK_ERL_DB);
	word_source_init(&far, rate, WEAK_FAR_DBM0, 0x13572468u);
	for (f = 0; f < 20u * FRAMES_PER_SECOND; f++) {
		/* The quiet moment falls on a frame that the far end talks on,
		 * the frame that the stale rule reads. One in a far pause is a
		 * lower floor that the tracker follows. */
		bool quiet = f % WEAK_OUTLIER_EVERY == 0u && far.talking;

		weak_step(ec, &path, &far, NULL, &floor_state, &st, &held,
			  window, quiet);
	}
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec keeps a floor a single quiet frame contradicts");
	CHECK(st.active > 500u);
	CHECK(st.adapt * 10u >= st.active * 9u);
	CHECK(st.flagged * 20u <= st.active);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * A hybrid that amplifies justifies the microphone that the filter chases, so
 * the runaway is real. The detector holds it from the first frame, so each
 * reset is a trial abort. No published field is ever not a number, one trial
 * cannot trip the watchdog twice, and the canceller keeps learning afterwards.
 */
static void watchdog_recovery_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x0c0ffee1u;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	struct echo_path passive;
	unsigned finite_bad = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, -8.0);
	path_flat_init(&passive, UDSP_SAMPLE_RATE, 8, 12.0);
	for (f = 0; f < 20u * FRAMES_PER_SECOND; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		path_run(&path, ref, echo);
		memcpy(mic, echo, sizeof(mic));
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (!isfinite((double)info.coef_energy) ||
		    !isfinite((double)info.erle_db) ||
		    !isfinite((double)info.echo_gain))
			finite_bad++;
	}

	CASE("lec recovers from the watchdog with every tracker finite");
	/* The scenario has to reach the branch it is about. */
	CHECK(info.diverge_resets >= 1u);
	CHECK_UINT(finite_bad, 0);
	/* Each trip inside a trial ends it and restores the snapshot, here a
	 * filter of zeroes. One trial therefore cannot trip the watchdog twice,
	 * and a zero energy is the restore, not a zeroing. */
	CHECK(info.diverge_resets <= info.probes);
	/* The line goes passive. A live power that is not a number would take
	 * the zero step for ever, so recovery within 2 s tests the reseed. */
	path_history_carry(&passive, &path);
	CHECK(erle_run(ec, &passive, &state, -10.0, 2u * FRAMES_PER_SECOND,
		       20) >= 25.0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK(info.coef_energy > 0.0f);
	CHECK(isfinite((double)info.coef_energy));
	udsp_ec_nlms.destroy(ec);
}

/*
 * A divergence on the last countdown frame of a trial must still abort, or
 * probe_finish keeps the diverged filter and the watchdog zeroes it. The hybrid
 * crosses on that frame; a twin frozen at the trial's start holds the snapshot.
 */
#define TRIAL_BUDGET		100u
#define TRIAL_QUIET_DBM0	-48.0
#define TRIAL_LATCH_DBM0	-16.0
#define TRIAL_AMP_DB		-8.0
#define TRIAL_FLUSH_FRAMES	10u
#define TRIAL_AMP_SEED		0x0c0ffee1u

/* One frame to both instances, and the echo the hybrid returned. mic is
 * written in place by the canceller, so each arm gets its own copy. */
static void twin_step(struct udsp_ec *lead, struct udsp_ec *twin,
		      struct echo_path *path, const int16_t *ref,
		      const int16_t *near, int16_t *lead_out,
		      int16_t *twin_out, double *mic_energy)
{
	int16_t echo[UDSP_FRAME_MAX];
	unsigned i;

	path_run(path, ref, echo);
	for (i = 0; i < path->frame; i++)
		lead_out[i] = near ? sample_clamp((double)echo[i] + near[i]) :
				     echo[i];
	/* The microphone as the canceller receives it, before run() writes
	 * the residual over it. */
	if (mic_energy) {
		for (i = 0; i < path->frame; i++)
			*mic_energy += (double)lead_out[i] * lead_out[i];
	}
	memcpy(twin_out, lead_out, path->frame * sizeof(*lead_out));
	udsp_ec_nlms.run(lead, lead_out, ref);
	udsp_ec_nlms.run(twin, twin_out, ref);
}

/* The two instances of the boundary walk, the weak hybrid, the hybrid that
 * amplifies, and the outputs of the last frame. */
struct boundary_line {
	struct echo_path weak;
	struct echo_path amp;
	struct udsp_ec *lead;
	struct udsp_ec *twin;
	unsigned frame;
	uint32_t state;
	uint32_t near_state;
	uint32_t amp_state;
	int16_t lead_out[UDSP_FRAME_MAX];
	int16_t twin_out[UDSP_FRAME_MAX];
};

static void boundary_line_init(struct boundary_line *b, unsigned rate)
{
	memset(b, 0, sizeof(*b));
	b->lead = udsp_ec_nlms.create_rate(48, rate);
	b->twin = udsp_ec_nlms.create_rate(48, rate);
	b->frame = rate / 100u;
	b->state = 0x5eed10c5u;
	b->near_state = 0x0badf00du;
	udsp_ec_nlms.nlp(b->lead, false);
	udsp_ec_nlms.nlp(b->twin, false);
	path_weak_init(&b->weak, rate, WEAK_ERL_DB);
	path_flat_init(&b->amp, rate, 8, TRIAL_AMP_DB);
}

/* One frame of the far end on the weak hybrid, with a white talker at
 * TRIAL_LATCH_DBM0 when talk is set. */
static void boundary_weak_run(struct boundary_line *b, bool talk)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];

	noise_fill(ref, b->frame, WEAK_FAR_DBM0, &b->state);
	if (talk)
		noise_fill(near, b->frame, TRIAL_LATCH_DBM0, &b->near_state);
	twin_step(b->lead, b->twin, &b->weak, ref, talk ? near : NULL,
		  b->lead_out, b->twin_out, NULL);
}

/*
 * Converge for 2 s, so the abort gives back a real filter. A talker 9 dB over
 * the far end, under the talker guard, then joins. It stays until the latch
 * starts a trial on a double-talk frame, which adapts on nothing. The twin
 * freezes on that frame and holds the snapshot from then on.
 */
static void boundary_trial_start(struct boundary_line *b,
				 struct udsp_ec_nlms_info *converged)
{
	struct udsp_ec_nlms_info info;
	unsigned f;

	for (f = 0; f < 2u * FRAMES_PER_SECOND; f++)
		boundary_weak_run(b, false);
	udsp_ec_nlms_info_get(b->lead, converged);
	for (f = 0; f < 20u * FRAMES_PER_SECOND; f++) {
		boundary_weak_run(b, true);
		udsp_ec_nlms_info_get(b->lead, &info);
		if (info.probes)
			break;
	}
	udsp_ec_nlms.freeze(b->twin, true);
}

/* The silent reference of TRIAL_FLUSH_FRAMES frames, and the hybrid that
 * amplifies after it, with its far end from TRIAL_AMP_SEED. */
static void boundary_flush_run(struct boundary_line *b)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned f;

	memset(ref, 0, sizeof(ref));
	for (f = 0; f < TRIAL_FLUSH_FRAMES; f++)
		twin_step(b->lead, b->twin, &b->weak, ref, NULL, b->lead_out,
			  b->twin_out, NULL);
	path_history_carry(&b->amp, &b->weak);
	b->amp_state = TRIAL_AMP_SEED;
}

/* One frame of the white far end on the hybrid that amplifies. It returns
 * the talker verdict of the frame. */
static bool boundary_amp_run(struct boundary_line *b, double *mic_energy)
{
	struct udsp_ec_nlms_frame reading;
	int16_t ref[UDSP_FRAME_MAX];

	noise_fill(ref, b->frame, WEAK_FAR_DBM0, &b->amp_state);
	twin_step(b->lead, b->twin, &b->amp, ref, NULL, b->lead_out,
		  b->twin_out, mic_energy);
	udsp_ec_nlms_frame_get(b->lead, &reading);
	return reading.talker;
}

/* The frames of the hybrid that amplifies that a trial with its whole budget
 * left needs to cross the bound, or 0 when it does not cross it. */
static unsigned boundary_amp_count(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct boundary_line b;
	unsigned count = 0;
	unsigned f;

	boundary_line_init(&b, rate);
	boundary_trial_start(&b, &info);
	boundary_flush_run(&b);
	for (f = 1; f <= TRIAL_BUDGET && !count; f++) {
		boundary_amp_run(&b, NULL);
		udsp_ec_nlms_info_get(b.lead, &info);
		if (info.diverge_resets)
			count = f;
	}
	udsp_ec_nlms.destroy(b.lead);
	udsp_ec_nlms.destroy(b.twin);
	return count;
}

static void probe_boundary_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info lead_info;
	struct udsp_ec_nlms_info twin_info;
	struct boundary_line b;
	unsigned amp_frames = boundary_amp_count(rate);
	unsigned spent = 0;
	unsigned talker = 0;
	unsigned differ = 0;
	uint32_t held;
	double last_mic = 0.0;
	double last_out = 0.0;
	unsigned f;

	case_rate(rate, "lec aborts a trial that blows up on its last frame");
	CHECK(amp_frames > 1u);
	boundary_line_init(&b, rate);
	boundary_trial_start(&b, &lead_info);
	CHECK(lead_info.erle_db >= 18.0f);
	CHECK(lead_info.coef_energy > 0.0f);
	CHECK_UINT(lead_info.probes, 0);
	udsp_ec_nlms_info_get(b.lead, &lead_info);
	CHECK_UINT(lead_info.probes, 1);
	CHECK(lead_info.coef_energy > 0.0f);
	held = lead_info.hold_frames;

	/* The budget is spent in active frames: the far end alone first, and
	 * the hybrid that amplifies after the silence. */
	while (spent + amp_frames < TRIAL_BUDGET) {
		boundary_weak_run(&b, false);
		spent++;
	}
	boundary_flush_run(&b);
	for (f = 1; f < amp_frames; f++) {
		talker += boundary_amp_run(&b, NULL);
		spent++;
	}
	udsp_ec_nlms_info_get(b.lead, &lead_info);
	/* Only the silent frames held, so the trial adapted on the other
	 * TRIAL_BUDGET - 1 frames and has exactly one left. */
	CHECK_UINT(lead_info.hold_frames - held, TRIAL_FLUSH_FRAMES);
	CHECK_UINT(lead_info.diverge_resets, 0);
	CHECK_UINT(lead_info.probes, 1);

	talker += boundary_amp_run(&b, &last_mic);
	for (f = 0; f < b.frame; f++)
		last_out += (double)b.lead_out[f] * b.lead_out[f];
	udsp_ec_nlms_frame_get(b.lead, &reading);
	udsp_ec_nlms_info_get(b.lead, &lead_info);
	udsp_ec_nlms_info_get(b.twin, &twin_info);
	printf("# %u frames of the hybrid that amplifies cross the bound, "
	       "after %u of the far end alone; %u talker frames among them; "
	       "the last frame costs %.4f dB\n", amp_frames,
	       TRIAL_BUDGET - amp_frames, talker,
	       10.0 * log10(last_out / last_mic));

	/* The guard reads the louder echo as echo, not as a talker. */
	CHECK_UINT(talker, 0);
	/* The watchdog saw it, and it left the line's filter in place. */
	CHECK_UINT(lead_info.diverge_resets, 1);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_DIVERGED);
	CHECK(lead_info.coef_energy > 0.0f);
	/* The abort undoes the update but not the output of the frame, computed
	 * while the filter diverged: it must stay under the microphone by the
	 * bar of a burst. */
	CHECK(last_out <= last_mic * pow(10.0, TALK_BURST_DB / 10.0));
	/* The published energy is the restored filter's, to the bit: the twin
	 * holds that filter and computes it the same way. */
	CHECK(lead_info.coef_energy == twin_info.coef_energy);

	/* And the coefficients themselves are the snapshot, sample for
	 * sample, on frames neither instance can adapt on. */
	udsp_ec_nlms.freeze(b.lead, true);
	path_history_carry(&b.weak, &b.amp);
	for (f = 0; f < 20u; f++) {
		boundary_weak_run(&b, false);
		if (memcmp(b.lead_out, b.twin_out,
			   b.frame * sizeof(*b.lead_out)))
			differ++;
	}
	CHECK_UINT(differ, 0);

	/* The trial is over, not paused: the run counters went back to zero
	 * and the escape stood down, so no second trial follows it. */
	udsp_ec_nlms.freeze(b.lead, false);
	for (f = 0; f < 3u * FRAMES_PER_SECOND; f++)
		boundary_weak_run(&b, false);
	udsp_ec_nlms_info_get(b.lead, &lead_info);
	CHECK_UINT(lead_info.probes, 1);
	CHECK_UINT(lead_info.diverge_resets, 1);
	udsp_ec_nlms.destroy(b.lead);
	udsp_ec_nlms.destroy(b.twin);
}

/* A louder hybrid, 23 dB of return loss to STEP_TO_DB: the learned gain cannot
 * describe it, and path_change_get misses a gain step. The latch trial, free of
 * the talk scale, recovers it within STEP_SETTLE_MAX. */
#define STEP_FROM_DB		23.0
#define STEP_TO_DB		3.0
#define STEP_BAR_DB		10.0
#define STEP_BLOCK_FRAMES	25u
/* LEC_GEIGEL_LATCH_MAX of dsp/lec.c: the run the clear frames cannot pay
 * down, and the earliest this recovery can begin. */
#define STEP_LATCH_FRAMES	500u
#define STEP_SETTLE_MAX		(STEP_LATCH_FRAMES + 3u * STEP_BLOCK_FRAMES)
#define STEP_LIMIT_FRAMES	800u

static void louder_hybrid_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path first;
	struct echo_path second;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	unsigned frame = rate / 100u;
	uint32_t state = 0x13572468u;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned settled = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&first, rate, 8, STEP_FROM_DB);
	path_flat_init(&second, rate, 8, STEP_TO_DB);
	CHECK(erle_run(ec, &first, &state, -10.0, 3u * FRAMES_PER_SECOND,
		       20) >= 25.0);
	path_history_carry(&second, &first);
	for (f = 0; f < STEP_LIMIT_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, frame, -10.0, &state);
		path_run(&second, ref, echo);
		memcpy(mic, echo, frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, mic, ref);
		for (i = 0; i < frame; i++) {
			echo_energy += (double)echo[i] * echo[i];
			res_energy += (double)mic[i] * mic[i];
		}
		if ((f + 1u) % STEP_BLOCK_FRAMES)
			continue;
		if (!settled && res_energy > 0.0 &&
		    10.0 * log10(echo_energy / res_energy) >= STEP_BAR_DB)
			settled = f + 1u;
		echo_energy = res_energy = 0.0;
	}
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec recovers a hybrid that got twenty dB louder");
	/* The escape is the only way out of this one, so it has to fire. */
	CHECK(info.probes >= 1u);
	CHECK(settled > 0u);
	CHECK(settled <= STEP_SETTLE_MAX);
	CHECK(info.erle_db >= 25.0f);
	udsp_ec_nlms.destroy(ec);
}

/* Talk-over: without the step control, each frame that adapts under near talk
 * passes the estimate arm at nearly full step and the watchdog resets. Both
 * talkers speak words at call levels, in step and then skewed. */
#define TALK_NEAR_DBM0		-10.0
#define TALK_SKEW		15u
#define TALK_DUET_FRAMES	2000u
#define TALK_SKEWED_FRAMES	4000u
#define TALK_WINDOW		50u
#define TALK_WINDOW_MIN		10u

struct talk_stat {
	unsigned overlap;
	unsigned clean;
	unsigned clean_adapt;
	unsigned windows;
	unsigned zeroed;
	double worst_window;
	double win[2];
	unsigned win_n;
	unsigned win_used;
	bool alive;
};

/*
 * One frame with both talkers on the line. The echo window is the regression
 * gain of the output on the injected echo over half a second in which the
 * near end is silent, where the far caller hears their
 * own echo in the clear and a regression through near speech would measure
 * the near speech.
 */
static void talk_step(struct udsp_ec *ec, struct echo_path *path,
		      struct word_source *far, struct word_source *near,
		      uint32_t *floor_state, struct talk_stat *st,
		      uint32_t *held)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t talk[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	struct udsp_ec_nlms_info info;
	unsigned frame = path->frame;
	double echo_energy = 0.0;
	double product = 0.0;
	bool far_on;
	bool near_on;
	unsigned i;

	word_fill(far, ref, frame);
	word_fill(near, talk, frame);
	path_run(path, ref, echo);
	noise_fill(hiss, frame, WEAK_FLOOR_DBM0, floor_state);
	for (i = 0; i < frame; i++)
		mic[i] = sample_clamp((double)echo[i] + talk[i] + hiss[i]);
	far_on = frame_loud(ref, frame);
	near_on = frame_loud(talk, frame);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, &info);
	if (info.coef_energy > 0.0f)
		st->alive = true;
	else if (st->alive)
		st->zeroed++;
	for (i = 0; i < frame; i++) {
		echo_energy += (double)echo[i] * echo[i];
		product += (double)mic[i] * echo[i];
	}
	if (far_on && near_on)
		st->overlap++;
	if (far_on && !near_on) {
		st->clean++;
		if (info.hold_frames == *held)
			st->clean_adapt++;
		st->win[0] += product;
		st->win[1] += echo_energy;
		st->win_used++;
	}
	*held = info.hold_frames;
	if (++st->win_n < TALK_WINDOW)
		return;
	if (st->win_used >= TALK_WINDOW_MIN && st->win[1] > 0.0) {
		double gain = 20.0 * log10(fabs(st->win[0] / st->win[1]) +
					   1e-12);

		st->windows++;
		if (gain > st->worst_window)
			st->worst_window = gain;
	}
	st->win_n = 0;
	st->win_used = 0;
	st->win[0] = st->win[1] = 0.0;
}

static void talkover_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	struct word_source far;
	struct word_source near;
	struct talk_stat st;
	uint32_t floor_state = 0x0badcafeu;
	uint32_t state = 0x51ade0f1u;
	uint32_t held = 0;
	unsigned f;

	memset(&st, 0, sizeof(st));
	st.worst_window = -200.0;
	udsp_ec_nlms.nlp(ec, false);
	path_weak_init(&path, rate, WEAK_ERL_DB);
	word_source_init(&far, rate, WEAK_FAR_DBM0, 0x13572468u);
	word_source_init(&near, rate, TALK_NEAR_DBM0, 0xfeedfaceu);
	for (f = 0; f < TALK_DUET_FRAMES; f++)
		talk_step(ec, &path, &far, &near, &floor_state, &st, &held);
	/* The talkers fall out of step, which is what talk-over is: a partial
	 * overlap with clean far-end frames between the collisions. */
	for (f = 0; f < TALK_SKEW; f++) {
		int16_t skip[UDSP_FRAME_MAX];

		word_fill(&near, skip, rate / 100u);
	}
	for (f = 0; f < TALK_SKEWED_FRAMES; f++)
		talk_step(ec, &path, &far, &near, &floor_state, &st, &held);
	udsp_ec_nlms_info_get(ec, &info);

	case_rate(rate, "lec holds the filter through talk-over");
	/* The scenario has to reach the collisions it is about. */
	CHECK(st.overlap > 500u);
	CHECK(st.windows > 50u);
	CHECK_UINT(info.diverge_resets, 0);
	CHECK_UINT(st.zeroed, 0);
	/* And it must never hand the far end more of their own voice back
	 * than the hybrid returned. */
	CHECK(st.worst_window <= TALK_BURST_DB);
	/* The learned gain still describes the hybrid: a power gain at unity or
	 * more reads near speech, which the release arm, gated to clean frames,
	 * keeps out. The line statistics report it as echo_gain_micro. */
	CHECK(info.echo_gain < 1.0f);
	/*
	 * The talk-over ends. A control that answers a talker by refusing to
	 * learn has to hand the line back afterwards, so the filter is
	 * required to be worth having again inside a second and a half.
	 */
	CHECK(erle_run(ec, &path, &state, WEAK_FAR_DBM0,
		       3u * FRAMES_PER_SECOND / 2u, 20) >= 6.0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * --path runs the whole suite on one form of the per-tap loops, which is how
 * the behavioral oracles are proved to pass identically on either. Without
 * it, the suite runs the form that the target selects by default.
 */
static bool path_select(int argc, char **argv)
{
	enum udsp_ec_nlms_path path;

	if (argc == 1)
		return true;
	if (argc == 3 && !strcmp(argv[1], "--path")) {
		for (path = UDSP_EC_NLMS_PATH_SCALAR;
		     path <= UDSP_EC_NLMS_PATH_SIMD; path++) {
			if (strcmp(argv[2], udsp_ec_nlms_path_name(path)))
				continue;
			if (udsp_ec_nlms_path_set(path))
				return true;
			fprintf(stderr,
				"test_lec: this build has no %s path\n",
				argv[2]);
			return false;
		}
	}
	fprintf(stderr, "usage: %s [--path scalar|simd]\n", argv[0]);
	return false;
}

static bool zero_skip_info_same(const struct udsp_ec_nlms_info *a,
				const struct udsp_ec_nlms_info *b)
{
	return a->tail_ms == b->tail_ms && a->taps == b->taps &&
	       a->frames == b->frames && a->hold_frames == b->hold_frames &&
	       a->diverge_resets == b->diverge_resets &&
	       a->probes == b->probes && a->coef_energy == b->coef_energy &&
	       a->erle_db == b->erle_db && a->echo_gain == b->echo_gain &&
	       a->double_talk == b->double_talk &&
	       a->nlp_active == b->nlp_active &&
	       a->nlp_active_frames == b->nlp_active_frames &&
	       a->nlp_wait_frames == b->nlp_wait_frames &&
	       a->nlp_erle_frames == b->nlp_erle_frames &&
	       a->nlp_unlike_frames == b->nlp_unlike_frames &&
	       a->nlp_dt_frames == b->nlp_dt_frames &&
	       a->nlp_quiet_frames == b->nlp_quiet_frames &&
	       a->coef_peak_tap == b->coef_peak_tap &&
	       a->noise_dbov == b->noise_dbov &&
	       a->nlp_dt_fired_frames == b->nlp_dt_fired_frames &&
	       a->ref_floor == b->ref_floor;
}

/*
 * The zero skip and the full form run side by side, bit for bit on every sample
 * and state field. The line runs through silences with and without near speech
 * and the NLP on. A pause of P frames skips P less the frames the window takes
 * to drain; the full form skips none.
 */
static void zero_skip_test(unsigned rate)
{
	struct udsp_ec *full = udsp_ec_nlms.create_rate(48, rate);
	struct udsp_ec *gated = udsp_ec_nlms.create_rate(48, rate);
	struct udsp_ec_nlms_info full_info;
	struct udsp_ec_nlms_info gated_info;
	struct echo_path path;
	uint32_t state = 0x5eed0123u ^ rate;
	uint32_t near_state = 0x0badbeefu;
	unsigned frame = rate / 100u;
	unsigned diff_frames = 0;
	unsigned state_frames = 0;
	unsigned expect_skip = 0;
	unsigned drain;
	unsigned f;

	udsp_ec_nlms.nlp(full, true);
	udsp_ec_nlms.nlp(gated, true);
	udsp_ec_nlms_zero_skip_set(full, false);
	udsp_ec_nlms_zero_skip_set(gated, true);
	path_composite_init(&path, rate, 12.0);
	udsp_ec_nlms_info_get(full, &full_info);
	drain = (full_info.taps + frame - 1u) / frame;
	for (f = 0; f < 500u; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		int16_t full_out[UDSP_FRAME_MAX];
		int16_t gated_out[UDSP_FRAME_MAX];
		bool pause = (f >= 200u && f < 300u) || (f >= 400u && f < 450u);
		unsigned pause_at = f >= 400u ? f - 400u : f - 200u;
		unsigned i;

		if (pause) {
			memset(ref, 0, frame * sizeof(*ref));
			if (pause_at >= drain)
				expect_skip++;
		} else {
			noise_fill(ref, frame, -10.0, &state);
		}
		path_run(&path, ref, mic);
		if (f >= 400u && f < 450u) {
			int16_t near[UDSP_FRAME_MAX];

			noise_fill(near, frame, -19.0, &near_state);
			for (i = 0; i < frame; i++)
				mic[i] = sample_clamp((double)mic[i] + near[i]);
		}
		memcpy(full_out, mic, frame * sizeof(*mic));
		memcpy(gated_out, mic, frame * sizeof(*mic));
		udsp_ec_nlms.run(full, full_out, ref);
		udsp_ec_nlms_info_get(full, &full_info);
		udsp_ec_nlms.run(gated, gated_out, ref);
		udsp_ec_nlms_info_get(gated, &gated_info);
		if (memcmp(full_out, gated_out, frame * sizeof(*mic)))
			diff_frames++;
		if (!zero_skip_info_same(&full_info, &gated_info))
			state_frames++;
	}

	case_rate(rate, "lec zero-reference skip matches the full form bit for bit");
	printf("# %u frames differ, %u states differ, %u of %u frames skipped, coef %.3e\n",
	       diff_frames, state_frames, gated_info.skip_frames,
	       gated_info.frames, (double)gated_info.coef_energy);
	CHECK_UINT(diff_frames, 0);
	CHECK_UINT(state_frames, 0);
	CHECK_UINT(gated_info.skip_frames, expect_skip);
	CHECK_UINT(expect_skip, 150u - 2u * drain);
	CHECK_UINT(full_info.skip_frames, 0);
	CHECK(gated_info.hold_frames >= gated_info.skip_frames);
	/* A comparison of two idle filters would pass on nothing. */
	CHECK(gated_info.coef_energy > 0.0f);
	udsp_ec_nlms.destroy(full);
	udsp_ec_nlms.destroy(gated);
}

static uint32_t verdict_sum_get(const struct udsp_ec_nlms_info *info)
{
	return info->nlp_active_frames + info->nlp_wait_frames +
	       info->nlp_erle_frames + info->nlp_unlike_frames +
	       info->nlp_dt_frames + info->nlp_quiet_frames;
}

/*
 * One frame through the hybrid and the canceller, with the near talker on top
 * of the echo when there is one. out takes the output of the frame. acted
 * counts the frames that read nlp_active, the per-frame view that
 * nlp_active_frames must match.
 */
static void verdict_out_step(struct udsp_ec *ec, struct echo_path *path,
			     const int16_t *ref, const int16_t *near,
			     int16_t *out, unsigned *acted)
{
	struct udsp_ec_nlms_info info;
	unsigned i;

	path_run(path, ref, out);
	if (near) {
		for (i = 0; i < path->frame; i++)
			out[i] = sample_clamp((double)out[i] + near[i]);
	}
	udsp_ec_nlms.run(ec, out, ref);
	udsp_ec_nlms_info_get(ec, &info);
	if (info.nlp_active)
		(*acted)++;
}

static void verdict_step(struct udsp_ec *ec, struct echo_path *path,
			 const int16_t *ref, const int16_t *near,
			 unsigned *acted)
{
	int16_t out[UDSP_FRAME_MAX];

	verdict_out_step(ec, path, ref, near, out, acted);
}

static void verdict_far_run(struct udsp_ec *ec, struct echo_path *path,
			    uint32_t *state, unsigned frames, unsigned *acted)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned f;

	for (f = 0; f < frames; f++) {
		noise_fill(ref, path->frame, -10.0, state);
		verdict_step(ec, path, ref, NULL, acted);
	}
}

static void verdict_silence_run(struct udsp_ec *ec, struct echo_path *path,
				unsigned frames, unsigned *acted)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned f;

	memset(ref, 0, sizeof(ref));
	for (f = 0; f < frames; f++)
		verdict_step(ec, path, ref, NULL, acted);
}

/* An instance whose processor is on for every frame counts each frame once. Its
 * active count is the count of the frames that read nlp_active. */
static void verdict_identity_check(const struct udsp_ec *ec, unsigned acted)
{
	struct udsp_ec_nlms_info info;

	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(verdict_sum_get(&info), info.frames);
	CHECK_UINT(info.nlp_active_frames, acted);
}

/*
 * The fixtures of nlp_gate_probe and reset_test with each frame read, at the
 * line rate: a flat hybrid at delay_ms, far-end noise from a zero filter with
 * the processor on, then the filter frozen.
 */
static struct udsp_ec *gate_fixture_rate_create(struct echo_path *path,
						unsigned rate,
						unsigned delay_ms,
						double erl_db, unsigned frames,
						uint32_t *state,
						unsigned *acted)
{
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);

	path_flat_init(path, rate, delay_ms, erl_db);
	verdict_far_run(ec, path, state, frames, acted);
	udsp_ec_nlms.freeze(ec, true);
	return ec;
}

/* The fixture above at 8 kHz on a flat 8 ms hybrid. */
static struct udsp_ec *gate_fixture_create(struct echo_path *path,
					   double erl_db, unsigned frames,
					   uint32_t *state, unsigned *acted)
{
	return gate_fixture_rate_create(path, UDSP_SAMPLE_RATE, 8u, erl_db,
					frames, state, acted);
}

/*
 * A window with no reference is the first refusal, and a processor that is
 * switched off is no refusal at all. Zero frames on a fresh instance take the
 * skip path of ec_run, which still reaches the gate. A count placed after the
 * return that leaves a silent frame alone reads no quiet frame here. A count
 * that files the switched-off frames as quiet reads 100.
 */
static void nlp_quiet_count_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	unsigned acted = 0;

	CASE("lec nlp counts a silent window as quiet and a switched-off "
	     "processor nowhere");
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	verdict_silence_run(ec, &path, 50u, &acted);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.nlp(ec, false);
	verdict_silence_run(ec, &path, 50u, &acted);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.skip_frames, 100);
	CHECK_UINT(info.nlp_quiet_frames, 50);
	CHECK_UINT(info.nlp_active_frames, 0);
	CHECK_UINT(info.nlp_wait_frames, 0);
	CHECK_UINT(info.nlp_erle_frames, 0);
	CHECK_UINT(info.nlp_unlike_frames, 0);
	CHECK_UINT(info.nlp_dt_frames, 0);
	CHECK_UINT(info.frames - verdict_sum_get(&info), 50);
	CHECK_UINT(acted, 0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The ERLE gate on the fixtures of nlp_gate_probe: the 12 dB hybrid stands
 * under both arms, so the gate refuses its echo-like frames. On the 23 dB
 * hybrid the processor acts. A count that files an ERLE refusal as unlike, or
 * an arm that passes what it refuses, fails.
 */
static void nlp_erle_count_test(void)
{
	static const double erl_db[2] = { 12.0, 23.0 };
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t erle[2];
	uint32_t active[2];
	unsigned i;

	CASE("lec nlp counts the frames its loss gate refuses");
	for (i = 0; i < 2u; i++) {
		uint32_t state = 0x31415926u;
		unsigned acted = 0;

		ec = gate_fixture_create(&path, erl_db[i], 36u, &state, &acted);
		udsp_ec_nlms_info_get(ec, &before);
		verdict_far_run(ec, &path, &state, 2u * FRAMES_PER_SECOND,
				&acted);
		udsp_ec_nlms_info_get(ec, &after);
		verdict_identity_check(ec, acted);
		erle[i] = after.nlp_erle_frames - before.nlp_erle_frames;
		active[i] = after.nlp_active_frames - before.nlp_active_frames;
		printf("# %.0f dB: erle %.1f dB, over 200 frames erle %u "
		       "unlike %u active %u\n", erl_db[i],
		       (double)after.erle_db, erle[i],
		       after.nlp_unlike_frames - before.nlp_unlike_frames,
		       active[i]);
		udsp_ec_nlms.destroy(ec);
	}
	CHECK(erle[0] >= 150u);
	CHECK_UINT(active[0], 0);
	CHECK(active[1] >= 150u);
}

/*
 * The near talker of nlp_echo_like_test answers in the far-end pauses, 1 dB
 * under the echo. The detector cannot see it, so the frame passes the
 * double-talk test and stops at echo-likeness. A count that files it under the
 * ERLE gate, or drops it, fails here.
 */
static void nlp_unlike_count_test(void)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xa5a5c3c3u;
	unsigned near_frames = 0;
	unsigned acted = 0;
	unsigned f;

	CASE("lec nlp counts a near talker the echo does not explain");
	ec = gate_fixture_create(&path, 23.0, 36u, &state, &acted);
	verdict_far_run(ec, &path, &state, FRAMES_PER_SECOND, &acted);
	udsp_ec_nlms_info_get(ec, &before);
	for (f = 0; f < 2u * FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		bool near_now = turn_frame_fill(f, ref, near, &state,
						&near_state);

		if (near_now)
			near_frames++;
		verdict_step(ec, &path, ref, near_now ? near : NULL, &acted);
	}
	udsp_ec_nlms_info_get(ec, &after);
	printf("# %u near frames, unlike %u erle %u dt %u\n", near_frames,
	       after.nlp_unlike_frames - before.nlp_unlike_frames,
	       after.nlp_erle_frames - before.nlp_erle_frames,
	       after.nlp_dt_frames - before.nlp_dt_frames);
	CHECK(near_frames >= 20u);
	CHECK(after.nlp_unlike_frames - before.nlp_unlike_frames + 5u >=
	      near_frames);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

/*
 * A near talker 18 dB over the echo is double talk, and its frame is not
 * echo-like either. The gate reads the detector first, so the frame counts as
 * double talk. A gate that tests echo-likeness first files these frames as not
 * echo-like and fails here.
 */
static void nlp_dt_count_test(void)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xfeedfaceu;
	unsigned acted = 0;
	unsigned f;

	CASE("lec nlp counts double talk ahead of echo-likeness");
	ec = gate_fixture_create(&path, 23.0, 36u, &state, &acted);
	verdict_far_run(ec, &path, &state, FRAMES_PER_SECOND, &acted);
	udsp_ec_nlms_info_get(ec, &before);
	for (f = 0; f < 30u; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];

		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		noise_fill(near, UDSP_FRAME_SAMPLES, -15.0, &near_state);
		verdict_step(ec, &path, ref, near, &acted);
	}
	udsp_ec_nlms_info_get(ec, &after);
	printf("# of 30 frames: dt %u unlike %u\n",
	       after.nlp_dt_frames - before.nlp_dt_frames,
	       after.nlp_unlike_frames - before.nlp_unlike_frames);
	CHECK(after.nlp_dt_frames - before.nlp_dt_frames >= 25u);
	CHECK_UINT(after.nlp_active_frames - before.nlp_active_frames, 0);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

/* A near burst over far digital silence: on a quiet frame it stands over the
 * echo-likeness bar of a line without a floor. It stands 27 dB or more under
 * the echo of the fixtures, so its floor barely moves their bars. */
#define PAUSE_BURST_DBM0	-60.0
/* The near talker of nlp_echo_like_test, 1 dB under the echo of its 23 dB
 * hybrid. */
#define FLAP_NEAR_DBM0		-34.0

/* One frame of far-end digital silence with white near noise at near_dbm0. */
static void near_step(struct udsp_ec *ec, struct echo_path *path,
		      double near_dbm0, uint32_t *near_state, unsigned *acted)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];

	memset(ref, 0, sizeof(ref));
	noise_fill(near, path->frame, near_dbm0, near_state);
	verdict_step(ec, path, ref, near, acted);
}

/* The window of an instance in frames: the tail rounded up to whole 10 ms
 * frames (ec_create_rate of dsp/lec.c), 5 on the 48 ms tail and 7 on the
 * 64 ms tail. */
static unsigned window_frames_get(const struct udsp_ec *ec)
{
	return (udsp_ec_nlms_tail_get(ec) + 9u) / 10u;
}

/* A refused frame every 4th frame leaves 3 pass frames between two refusals,
 * the most that still holds the gate off. A refused frame every 5th frame is
 * the boundary, where the processor acts on one frame per cycle. */
#define FLAP_PERIOD		4u
#define FLAP_COST_PERIOD	5u
#define FLAP_FRAMES		(2u * FRAMES_PER_SECOND)

/*
 * The line of the flap cases: from a complete run, the far end leaves a gap of
 * one frame per period. A near talker 1 dB under the echo fills it, so the
 * frame reads unlike in an active window. The detector does not fire, and the
 * tracker does not read the inactive gap frame.
 */
struct flap_line {
	struct echo_path path;
	struct udsp_ec *ec;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	unsigned flaps;
	unsigned acted;
	unsigned start;
};

static void flap_run(struct flap_line *l, unsigned rate, unsigned period)
{
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xa5a5c3c3u;
	unsigned f;

	memset(l, 0, sizeof(*l));
	l->ec = gate_fixture_rate_create(&l->path, rate, 8u, 23.0, 36u, &state,
					 &l->acted);
	verdict_far_run(l->ec, &l->path, &state, FRAMES_PER_SECOND, &l->acted);
	udsp_ec_nlms_info_get(l->ec, &l->before);
	l->start = l->acted;
	for (f = 0; f < FLAP_FRAMES; f++) {
		if (f % period) {
			verdict_far_run(l->ec, &l->path, &state, 1u, &l->acted);
			continue;
		}
		near_step(l->ec, &l->path, FLAP_NEAR_DBM0, &near_state,
			  &l->acted);
		l->flaps++;
	}
	udsp_ec_nlms_info_get(l->ec, &l->after);
}

static void flap_print(const struct flap_line *l)
{
	printf("# engaged %u before the flap; %u flap frames: unlike %u, "
	       "wait %u, active %u, dt %u, erle %u, quiet %u\n",
	       l->before.nlp_active, l->flaps,
	       l->after.nlp_unlike_frames - l->before.nlp_unlike_frames,
	       l->after.nlp_wait_frames - l->before.nlp_wait_frames,
	       l->after.nlp_active_frames - l->before.nlp_active_frames,
	       l->after.nlp_dt_frames - l->before.nlp_dt_frames,
	       l->after.nlp_erle_frames - l->before.nlp_erle_frames,
	       l->after.nlp_quiet_frames - l->before.nlp_quiet_frames);
}

/*
 * The engage hysteresis against a flap: a gap in every 4 frames. The first
 * refusal cuts the run to LEC_NLP_DECAY_FRAMES, and each later one meets a run
 * under 8 and zeroes it, so the processor never acts. A run kept through a
 * refusal, or a larger cut, fails.
 */
static void nlp_hysteresis_test(unsigned rate)
{
	struct flap_line l;
	unsigned refused;

	flap_run(&l, rate, FLAP_PERIOD);
	refused = l.after.nlp_unlike_frames - l.before.nlp_unlike_frames +
		  l.after.nlp_dt_frames - l.before.nlp_dt_frames;
	case_rate(rate, "lec nlp stays off under a refused frame every few "
		  "frames of far speech");
	flap_print(&l);
	CHECK(l.before.nlp_active);
	CHECK_UINT(l.acted - l.start, 0);
	CHECK(refused >= l.flaps);
	CHECK_UINT(l.after.nlp_wait_frames - l.before.nlp_wait_frames,
		   FLAP_FRAMES - refused);
	verdict_identity_check(l.ec, l.acted);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The cut run at the boundary of the flap guard: a gap in every 5 frames. The
 * 4th pass frame completes the cut run again, so the processor acts on one
 * frame per cycle. A run zeroed on each refusal acts on none; a run kept
 * through it acts on all 4.
 */
static void nlp_flap_cost_test(unsigned rate)
{
	struct flap_line l;

	flap_run(&l, rate, FLAP_COST_PERIOD);
	case_rate(rate, "lec nlp acts on one frame per cycle under a refused "
		  "frame every fifth frame, the cost of the cut run");
	flap_print(&l);
	CHECK(l.before.nlp_active);
	CHECK_UINT(l.flaps, FLAP_FRAMES / FLAP_COST_PERIOD);
	CHECK_UINT(l.acted - l.start, l.flaps);
	CHECK_UINT(l.after.nlp_unlike_frames - l.before.nlp_unlike_frames,
		   l.flaps);
	CHECK_UINT(l.after.nlp_wait_frames - l.before.nlp_wait_frames,
		   FLAP_FRAMES - 2u * l.flaps);
	verdict_identity_check(l.ec, l.acted);
	udsp_ec_nlms.destroy(l.ec);
}

#define CYCLE_FRAMES		10u
/* The near burst at the end of a cycle: its frames stand over the quiet bar by
 * more than LEC_NLP_PEAK_RATIO, or the line has no floor. Its first frame
 * therefore zeroes the run. */
#define CYCLE_BURST_FRAMES	2u

/*
 * One cycle of the engage-run cases: far frames of far-end noise, then
 * digital silence. When near_state is given, a near burst covers the last
 * CYCLE_BURST_FRAMES frames of the cycle. It returns the first frame of the
 * cycle that reads nlp_active, or CYCLE_FRAMES when none does.
 */
static unsigned cycle_run(struct udsp_ec *ec, struct echo_path *path,
			  uint32_t *state, unsigned far, uint32_t *near_state,
			  unsigned *acted)
{
	unsigned at = CYCLE_FRAMES;
	unsigned p;

	for (p = 0; p < CYCLE_FRAMES; p++) {
		unsigned was = *acted;

		if (p < far)
			verdict_far_run(ec, path, state, 1u, acted);
		else if (near_state && p + CYCLE_BURST_FRAMES >= CYCLE_FRAMES)
			near_step(ec, path, PAUSE_BURST_DBM0, near_state,
				  acted);
		else
			verdict_silence_run(ec, path, 1u, acted);
		if (at == CYCLE_FRAMES && *acted != was)
			at = p;
	}
	return at;
}

#define SPURT_FAR		3u
#define SPURT_CYCLES		20u

/*
 * The complement of nlp_hysteresis_test: cycles of 3 frames of far noise and 7
 * of digital silence, 7 eligible frames, one short of the run. Silence is
 * echo-like and keeps the run, and a near burst zeroes it before the first
 * spurt, so only the first spurt waits. Zeroing the run on silence fails.
 */
static void nlp_spurt_test(unsigned rate)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xa5a5c3c3u;
	unsigned first_acted = 0;
	unsigned late = 0;
	unsigned acted = 0;
	unsigned c;

	ec = gate_fixture_rate_create(&path, rate, 8u, 23.0, 36u, &state,
				      &acted);
	verdict_far_run(ec, &path, &state, FRAMES_PER_SECOND, &acted);
	(void)cycle_run(ec, &path, &state, 0u, &near_state, &acted);
	udsp_ec_nlms_info_get(ec, &before);
	for (c = 0; c < SPURT_CYCLES; c++) {
		unsigned was = acted;
		unsigned at = cycle_run(ec, &path, &state, SPURT_FAR, NULL,
					&acted);

		if (!c)
			first_acted = acted - was;
		else
			late += at != 0u;
	}
	udsp_ec_nlms_info_get(ec, &after);

	case_rate(rate, "lec nlp acts from the second far spurt over echo-like "
		  "pauses");
	printf("# %u spurts: the first acts on %u frames, %u later spurts act "
	       "after their first frame; wait %u, active %u, quiet %u\n",
	       SPURT_CYCLES, first_acted, late,
	       after.nlp_wait_frames - before.nlp_wait_frames,
	       after.nlp_active_frames - before.nlp_active_frames,
	       after.nlp_quiet_frames - before.nlp_quiet_frames);
	CHECK_UINT(first_acted, 0);
	CHECK_UINT(late, 0);
	CHECK_UINT(after.nlp_wait_frames - before.nlp_wait_frames,
		   SPURT_FAR + window_frames_get(ec) - 1u);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

#define WAIT_FAR		4u
#define WAIT_BLOCKS		7u

/*
 * The engage run on the fixture of reset_test: cycles of 4 frames of far noise
 * and 6 of silence keep the run. A near burst at the end of every third cycle
 * zeroes it. The next cycle then acts on its 8th frame, and every other cycle
 * on its first. A wrong count, an early engage or a wrong zeroing fails.
 */
static void nlp_wait_count_test(unsigned rate)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x77665544u;
	uint32_t near_state = 0x600dcafeu;
	unsigned eighth = 0;
	unsigned first = 0;
	unsigned acted = 0;
	unsigned start;
	unsigned c;

	case_rate(rate, "lec nlp waits seven frames after a pause frame over "
		  "the echo-like bar and acts at once after an echo-like "
		  "pause");
	ec = gate_fixture_rate_create(&path, rate, 8u, 12.0, 150u, &state,
				      &acted);
	udsp_ec_nlms_info_get(ec, &before);
	CHECK(before.erle_db >= 18.0f);
	CHECK(before.nlp_active);
	start = acted;
	for (c = 0; c < 3u * WAIT_BLOCKS; c++) {
		unsigned at = cycle_run(ec, &path, &state, WAIT_FAR,
					c % 3u ? NULL : &near_state, &acted);

		if (c % 3u == 1u)
			eighth += at == 7u;
		else
			first += at == 0u;
	}
	udsp_ec_nlms_info_get(ec, &after);
	printf("# %u cycles: %u wait cycles act from position 7, %u others "
	       "from position 0; active %u wait %u quiet %u\n",
	       3u * WAIT_BLOCKS, eighth, first,
	       after.nlp_active_frames - before.nlp_active_frames,
	       after.nlp_wait_frames - before.nlp_wait_frames,
	       after.nlp_quiet_frames - before.nlp_quiet_frames);
	CHECK_UINT(after.nlp_active_frames - before.nlp_active_frames,
		   WAIT_BLOCKS * (8u + 1u + 8u));
	CHECK_UINT(after.nlp_wait_frames - before.nlp_wait_frames,
		   WAIT_BLOCKS * 7u);
	CHECK_UINT(after.nlp_quiet_frames - before.nlp_quiet_frames,
		   3u * WAIT_BLOCKS * 2u);
	CHECK_UINT(after.nlp_erle_frames - before.nlp_erle_frames, 0);
	CHECK_UINT(after.nlp_unlike_frames - before.nlp_unlike_frames, 0);
	CHECK_UINT(after.nlp_dt_frames - before.nlp_dt_frames, 0);
	CHECK_UINT(eighth, WAIT_BLOCKS);
	CHECK_UINT(first, 2u * WAIT_BLOCKS);
	CHECK_UINT(acted - start, after.nlp_active_frames -
		   before.nlp_active_frames);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

#define REFUSAL_LEAD_FRAMES	20u
#define REFUSAL_TAIL_FRAMES	100u
#define REFUSAL_DT_FRAMES	2u
#define REFUSAL_DT_NEAR_DBM0	-15.0
#define REFUSAL_ERL_STEP_DB	15.0
#define REFUSAL_ERLE_FRAMES	40u
/* A zeroed run acts on its 8th eligible frame (LEC_NLP_ON_FRAMES of dsp/lec.c),
 * 7 counted from 0. A run cut to LEC_NLP_DECAY_FRAMES acts on the 4th after the
 * refusal, CUT_WAIT counted from 0. */
#define REFUSAL_WAIT		7u
#define CUT_WAIT		3u

/*
 * The frames of the engage-run cases of one refusal. refused counts the
 * frames of that verdict, and like those of them whose microphone is
 * echo-like. other counts the frames of every other refusal. passes counts
 * the pass frames since the last refused frame, and at holds that count on
 * the first frame after a refusal that reads nlp_active.
 */
struct refusal_watch {
	unsigned verdict;
	unsigned refused;
	unsigned like;
	unsigned other;
	unsigned passes;
	unsigned at;
	bool waiting;
};

/* The echo-likeness test of doubletalk_update on a frame reading. 1.41 is
 * LEC_GEIGEL_MARGIN, 2.0 LEC_GEIGEL_FLOOR_HEADROOM and 2.8
 * LEC_GEIGEL_PAUSE_HEADROOM of dsp/lec.c. A window_run of 0 is a quiet
 * frame. */
static bool reading_echo_like_get(const struct udsp_ec_nlms_frame *r)
{
	float headroom = r->window_run ? 2.0f : 2.8f;

	return r->mic_power <= (r->estimate_power + r->noise_power * headroom) *
			       1.41f;
}

static void refusal_note(struct refusal_watch *w, const struct udsp_ec *ec)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;

	udsp_ec_nlms_frame_get(ec, &reading);
	udsp_ec_nlms_info_get(ec, &info);
	if (reading.nlp_verdict == w->verdict) {
		w->refused++;
		w->like += reading_echo_like_get(&reading);
		w->passes = 0;
		w->waiting = true;
		return;
	}
	if (reading.nlp_verdict != LECR_VERDICT_PASS) {
		w->other++;
		return;
	}
	if (w->waiting && info.nlp_active) {
		w->at = w->passes;
		w->waiting = false;
	}
	w->passes++;
}

static void refusal_far_run(struct udsp_ec *ec, struct echo_path *path,
			    uint32_t *state, unsigned frames,
			    struct refusal_watch *w, unsigned *acted)
{
	unsigned f;

	for (f = 0; f < frames; f++) {
		verdict_far_run(ec, path, state, 1u, acted);
		refusal_note(w, ec);
	}
}

static void refusal_print(const char *what, const struct refusal_watch *w)
{
	printf("# %s: %u refused frames, %u of them echo-like, %u other "
	       "refused frames; the processor acts on eligible frame %u after "
	       "the last refused frame, counted from 0\n", what, w->refused,
	       w->like, w->other, w->at);
}

/*
 * From a complete run, a near burst 7 dB over the echo fires the detector on
 * far speech. Its first dt refusal cuts the run to LEC_NLP_DECAY_FRAMES, and
 * the next zeroes it. The far speech after the hangover then acts on its 8th
 * eligible frame. A run kept through a dt frame fails here.
 */
static void nlp_dt_wait_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct refusal_watch w;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x77665544u;
	uint32_t near_state = 0xfeedfaceu;
	unsigned acted = 0;
	unsigned f;

	memset(&w, 0, sizeof(w));
	w.verdict = LECR_VERDICT_DT;
	w.at = REFUSAL_TAIL_FRAMES;
	ec = gate_fixture_rate_create(&path, rate, 8u, 12.0, 150u, &state,
				      &acted);
	verdict_far_run(ec, &path, &state, REFUSAL_LEAD_FRAMES, &acted);
	udsp_ec_nlms_info_get(ec, &info);
	for (f = 0; f < REFUSAL_DT_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(ref, path.frame, -10.0, &state);
		noise_fill(near, path.frame, REFUSAL_DT_NEAR_DBM0, &near_state);
		verdict_step(ec, &path, ref, near, &acted);
		refusal_note(&w, ec);
	}
	refusal_far_run(ec, &path, &state, REFUSAL_TAIL_FRAMES, &w, &acted);

	case_rate(rate, "lec nlp waits seven eligible frames after the double "
		  "talk of a near burst");
	refusal_print("near burst", &w);
	CHECK(info.nlp_active);
	CHECK(w.refused > REFUSAL_DT_FRAMES);
	CHECK(w.like > 0u);
	CHECK_UINT(w.other, 0);
	CHECK_UINT(w.at, REFUSAL_WAIT);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

/*
 * From a complete run, the hybrid turns 3 dB quieter under far speech, so the
 * frames stay echo-like and read erle until it returns. The first erle frame
 * cuts the run, and the next zeroes it. The far speech then acts on the 8th
 * eligible frame after the last erle frame. A run kept through it fails.
 */
static void nlp_erle_wait_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct refusal_watch w;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x77665544u;
	unsigned acted = 0;

	memset(&w, 0, sizeof(w));
	w.verdict = LECR_VERDICT_ERLE;
	w.at = REFUSAL_TAIL_FRAMES;
	ec = gate_fixture_rate_create(&path, rate, 8u, 12.0, 150u, &state,
				      &acted);
	verdict_far_run(ec, &path, &state, REFUSAL_LEAD_FRAMES, &acted);
	udsp_ec_nlms_info_get(ec, &info);
	path_erl_set(&path, REFUSAL_ERL_STEP_DB);
	refusal_far_run(ec, &path, &state, REFUSAL_ERLE_FRAMES, &w, &acted);
	path_erl_set(&path, 12.0);
	refusal_far_run(ec, &path, &state, REFUSAL_TAIL_FRAMES, &w, &acted);

	case_rate(rate, "lec nlp waits seven eligible frames after the erle "
		  "gate refuses far speech");
	refusal_print("quieter hybrid", &w);
	CHECK(info.nlp_active);
	CHECK(w.refused > 0u);
	CHECK_UINT(w.like, w.refused);
	CHECK_UINT(w.other, 0);
	CHECK_UINT(w.at, REFUSAL_WAIT);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

#define CUT_GAP_FRAMES		1u

/* One leg of nlp_cut_test from a complete run: refusals unlike frames, as
 * flap_run makes them, with CUT_GAP_FRAMES far frames between two of them,
 * then far speech. */
static void cut_leg_run(struct udsp_ec *ec, struct echo_path *path,
			uint32_t *state, uint32_t *near_state,
			unsigned refusals, struct refusal_watch *w,
			unsigned *acted)
{
	unsigned n;

	memset(w, 0, sizeof(*w));
	w->verdict = LECR_VERDICT_UNLIKE;
	w->at = REFUSAL_TAIL_FRAMES;
	for (n = 0; n < refusals; n++) {
		if (n)
			refusal_far_run(ec, path, state, CUT_GAP_FRAMES, w,
					acted);
		near_step(ec, path, FLAP_NEAR_DBM0, near_state, acted);
		refusal_note(w, ec);
	}
	refusal_far_run(ec, path, state, REFUSAL_TAIL_FRAMES, w, acted);
}

/*
 * The cut of a complete run: one unlike frame in far speech cuts it, so the
 * processor acts on the 4th eligible frame after it. A second unlike frame one
 * pass frame later zeroes it, so the processor acts on the 8th. A run zeroed on
 * every refusal fails the first leg, and a run kept through one fails both.
 */
static void nlp_cut_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct refusal_watch one;
	struct refusal_watch two;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xa5a5c3c3u;
	unsigned acted = 0;
	bool engaged;

	ec = gate_fixture_rate_create(&path, rate, 8u, 23.0, 36u, &state,
				      &acted);
	verdict_far_run(ec, &path, &state, FRAMES_PER_SECOND, &acted);
	udsp_ec_nlms_info_get(ec, &info);
	engaged = info.nlp_active;
	cut_leg_run(ec, &path, &state, &near_state, 1u, &one, &acted);
	cut_leg_run(ec, &path, &state, &near_state, 2u, &two, &acted);

	case_rate(rate, "lec nlp cuts a complete run on a single refused frame "
		  "and zeroes it on a second one 2 frames later");
	refusal_print("one unlike frame", &one);
	refusal_print("two unlike frames", &two);
	CHECK(engaged);
	CHECK_UINT(one.refused, 1);
	CHECK_UINT(two.refused, 2);
	CHECK_UINT(one.other + two.other, 0);
	CHECK_UINT(one.at, CUT_WAIT);
	CHECK_UINT(two.at, REFUSAL_WAIT);
	verdict_identity_check(ec, acted);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The peak tap of a filter on a flat hybrid is the tap of the hybrid delay,
 * counted from tap 0 of the reference the canceller reads. A fresh filter
 * reads 0 with no energy. A peak one tap off, the smallest coefficient in
 * place of the largest, or a constant fails here.
 */
static void peak_tap_test(unsigned rate)
{
	static const unsigned delay_ms[2] = { 8u, 20u };
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x13572468u;
	unsigned i;

	case_rate(rate, "lec peak tap names the tap of the hybrid delay");
	for (i = 0; i < 2u; i++) {
		ec = udsp_ec_nlms.create_rate(48, rate);
		udsp_ec_nlms.nlp(ec, false);
		udsp_ec_nlms_info_get(ec, &info);
		CHECK_UINT(info.coef_peak_tap, 0);
		CHECK(info.coef_energy == 0.0f);
		path_flat_init(&path, rate, delay_ms[i], 12.0);
		(void)erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 1);
		udsp_ec_nlms_info_get(ec, &info);
		CHECK(info.coef_energy > 0.0f);
		CHECK_UINT(info.coef_peak_tap,
			   path_samples_get(delay_ms[i], rate));
		udsp_ec_nlms.destroy(ec);
	}
}

#define RESET_RETURN_FRAMES	40u

/*
 * The reset forgets the verdict counts, which describe one alignment of the
 * reference, and the peak with the filter. A pause, a near talker and a far
 * return raise each count over 0 first. Trial and watchdog counters keep their
 * lifetime meaning.
 */
static void reset_count_test(void)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info after;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x77665544u;
	uint32_t near_state = 0x600dcafeu;
	unsigned acted = 0;
	unsigned f;

	CASE("lec reset zeroes the processor counters and the peak tap");
	ec = gate_fixture_create(&path, 12.0, 150u, &state, &acted);
	verdict_silence_run(ec, &path, 10u, &acted);
	for (f = 0; f < 5u; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];

		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		noise_fill(near, UDSP_FRAME_SAMPLES, -15.0, &near_state);
		verdict_step(ec, &path, ref, near, &acted);
	}
	verdict_far_run(ec, &path, &state, RESET_RETURN_FRAMES, &acted);
	udsp_ec_nlms_info_get(ec, &before);
	printf("# before the reset: active %u wait %u erle %u unlike %u "
	       "dt %u fired %u quiet %u peak %u\n", before.nlp_active_frames,
	       before.nlp_wait_frames, before.nlp_erle_frames,
	       before.nlp_unlike_frames, before.nlp_dt_frames,
	       before.nlp_dt_fired_frames, before.nlp_quiet_frames,
	       before.coef_peak_tap);
	verdict_identity_check(ec, acted);
	CHECK(before.nlp_active_frames > 0u);
	CHECK(before.nlp_wait_frames >= 7u);
	CHECK(before.nlp_erle_frames > 0u);
	CHECK(before.nlp_unlike_frames > 0u);
	CHECK(before.nlp_dt_frames > 0u);
	CHECK(before.nlp_quiet_frames > 0u);
	CHECK(before.nlp_dt_fired_frames > 0u);
	CHECK_UINT(before.coef_peak_tap, 64);
	udsp_ec_nlms_reset(ec);
	udsp_ec_nlms_info_get(ec, &after);
	CHECK_UINT(after.nlp_active_frames, 0);
	CHECK_UINT(after.nlp_wait_frames, 0);
	CHECK_UINT(after.nlp_erle_frames, 0);
	CHECK_UINT(after.nlp_unlike_frames, 0);
	CHECK_UINT(after.nlp_dt_frames, 0);
	CHECK_UINT(after.nlp_quiet_frames, 0);
	CHECK_UINT(after.nlp_dt_fired_frames, 0);
	CHECK_UINT(after.coef_peak_tap, 0);
	CHECK_UINT(after.diverge_resets, before.diverge_resets);
	CHECK_UINT(after.probes, before.probes);
	udsp_ec_nlms.destroy(ec);
}

/* The ERLE tracker smooths over about ten frames, so a step waits that long
 * before it reads the tracker as settled. */
#define RAMP_STEP_DB		0.5
#define RAMP_SETTLED_DB		24.0f
#define RAMP_DWELL_FRAMES	10u

/*
 * The plain zero branch of the watchdog: the hybrid grows louder in 0.5 dB
 * steps while the ERLE stays over 18 dB. Plain adaptation, outside a trial,
 * then carries the energy past LEC_COEF_ENERGY_MAX. The 16 ms tail reaches it
 * within 10 s; a zero branch that leaves the peak standing fails.
 */
static void divergence_peak_test(void)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(16);
	uint32_t state = 0x0c0ffee2u;
	double erl_db = 12.0;
	float erle_min = 200.0f;
	unsigned dwell = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, erl_db);
	(void)erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 1);
	udsp_ec_nlms_info_get(ec, &before);
	for (f = 0; f < 10u * FRAMES_PER_SECOND; f++) {
		(void)erle_run(ec, &path, &state, -10.0, 1u, 1u);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.diverge_resets != before.diverge_resets)
			break;
		if (info.erle_db < erle_min)
			erle_min = info.erle_db;
		if (++dwell < RAMP_DWELL_FRAMES ||
		    info.erle_db < RAMP_SETTLED_DB)
			continue;
		erl_db -= RAMP_STEP_DB;
		path_erl_set(&path, erl_db);
		dwell = 0;
	}

	CASE("lec divergence reset clears the peak tap");
	printf("# frame %u, ERL %.1f dB, erle min %.1f dB, resets %u, "
	       "probes %u, coef %.3f, peak %u\n", f, erl_db,
	       (double)erle_min, info.diverge_resets, info.probes,
	       (double)info.coef_energy, info.coef_peak_tap);
	CHECK(info.diverge_resets > before.diverge_resets);
	CHECK_UINT(info.probes, before.probes);
	CHECK(info.coef_energy == 0.0f);
	CHECK_UINT(info.coef_peak_tap, 0);
	udsp_ec_nlms.destroy(ec);
}

#define FEED_PAUSE_FRAMES	10u

/* One frame of far end at -10 dBm0, or of silence, through the hybrid. */
static void feed_frame_make(struct echo_path *path, uint32_t *state, bool loud,
			    int16_t *ref, int16_t *echo)
{
	if (loud)
		noise_fill(ref, path->frame, -10.0, state);
	else
		memset(ref, 0, path->frame * sizeof(*ref));
	path_run(path, ref, echo);
}

/* Both instances run one frame, each on its own copy of the microphone. */
static void feed_pair_run(struct udsp_ec *a, struct udsp_ec *b,
			  const struct echo_path *path, const int16_t *ref,
			  const int16_t *mic, int16_t *out_a, int16_t *out_b)
{
	memcpy(out_a, mic, path->frame * sizeof(*mic));
	memcpy(out_b, mic, path->frame * sizeof(*mic));
	udsp_ec_nlms.run(a, out_a, ref);
	udsp_ec_nlms.run(b, out_b, ref);
}

/* The pause: a runs each loud frame, frozen by the caller, and b takes only
 * the reference. */
static void feed_pause_run(struct udsp_ec *a, struct udsp_ec *b,
			   struct echo_path *path, uint32_t *state)
{
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	unsigned f;

	for (f = 0; f < FEED_PAUSE_FRAMES; f++) {
		feed_frame_make(path, state, true, ref, mic);
		udsp_ec_nlms.run(a, mic, ref);
		udsp_ec_nlms_ref_feed(b, ref);
	}
}

static void feed_history_check(void)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	int16_t out_a[UDSP_FRAME_SAMPLES];
	int16_t out_b[UDSP_FRAME_SAMPLES];
	struct udsp_ec *a = udsp_ec_nlms.create(48);
	struct udsp_ec *b = udsp_ec_nlms.create(48);
	struct echo_path path;
	uint32_t state = 0x2718281du;
	unsigned f;

	udsp_ec_nlms.nlp(a, false);
	udsp_ec_nlms.nlp(b, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		feed_frame_make(&path, &state, true, ref, mic);
		feed_pair_run(a, b, &path, ref, mic, out_a, out_b);
	}
	udsp_ec_nlms_info_get(a, &info);
	udsp_ec_nlms_info_get(b, &before);
	CHECK(info.coef_energy == before.coef_energy);
	/* A comparison of two idle filters would pass on nothing. */
	CHECK(before.coef_energy > 0.0f);
	udsp_ec_nlms.freeze(a, true);
	feed_pause_run(a, b, &path, &state);
	udsp_ec_nlms_info_get(b, &info);
	CHECK(info.coef_energy == before.coef_energy);
	CHECK_UINT(info.frames, before.frames);
	udsp_ec_nlms.freeze(b, true);
	feed_frame_make(&path, &state, true, ref, mic);
	feed_pair_run(a, b, &path, ref, mic, out_a, out_b);
	CHECK(!memcmp(out_a, out_b, path.frame * sizeof(*out_a)));
	udsp_ec_nlms.destroy(a);
	udsp_ec_nlms.destroy(b);
}

static void feed_window_check(void)
{
	struct udsp_ec_nlms_info a0, a1, b0, b1;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	int16_t out_a[UDSP_FRAME_SAMPLES];
	int16_t out_b[UDSP_FRAME_SAMPLES];
	struct udsp_ec *a = udsp_ec_nlms.create(48);
	struct udsp_ec *b = udsp_ec_nlms.create(48);
	struct echo_path path;
	uint32_t state = 0x31415926u;

	udsp_ec_nlms.nlp(a, false);
	udsp_ec_nlms.nlp(b, false);
	udsp_ec_nlms.freeze(a, true);
	udsp_ec_nlms.freeze(b, true);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	feed_pause_run(a, b, &path, &state);
	udsp_ec_nlms.nlp(a, true);
	udsp_ec_nlms.nlp(b, true);
	udsp_ec_nlms_info_get(a, &a0);
	udsp_ec_nlms_info_get(b, &b0);
	feed_frame_make(&path, &state, false, ref, mic);
	feed_pair_run(a, b, &path, ref, mic, out_a, out_b);
	udsp_ec_nlms_info_get(a, &a1);
	udsp_ec_nlms_info_get(b, &b1);
	printf("# quiet frames on the silent frame: run %u, fed %u\n",
	       a1.nlp_quiet_frames - a0.nlp_quiet_frames,
	       b1.nlp_quiet_frames - b0.nlp_quiet_frames);
	CHECK_UINT(a1.nlp_quiet_frames - a0.nlp_quiet_frames, 0);
	CHECK_UINT(b1.nlp_quiet_frames - b0.nlp_quiet_frames, 0);
	udsp_ec_nlms.destroy(a);
	udsp_ec_nlms.destroy(b);
}

static void feed_skip_run(struct udsp_ec *ec, unsigned loud_frames,
			  unsigned zero_frames)
{
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	uint32_t state = 0x16180339u;
	unsigned f;

	for (f = 0; f < loud_frames; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		udsp_ec_nlms_ref_feed(ec, ref);
	}
	memset(ref, 0, sizeof(ref));
	for (f = 0; f < zero_frames; f++)
		udsp_ec_nlms_ref_feed(ec, ref);
	memset(mic, 0, sizeof(mic));
	udsp_ec_nlms.run(ec, mic, ref);
}

static void feed_skip_check(void)
{
	struct udsp_ec_nlms_info early;
	struct udsp_ec_nlms_info flushed;
	struct udsp_ec *a = udsp_ec_nlms.create(48);
	struct udsp_ec *b = udsp_ec_nlms.create(48);
	unsigned full;

	udsp_ec_nlms_info_get(a, &early);
	full = (early.taps + UDSP_FRAME_SAMPLES - 1u) / UDSP_FRAME_SAMPLES;
	feed_skip_run(a, full, full - 1u);
	feed_skip_run(b, full, full);
	udsp_ec_nlms_info_get(a, &early);
	udsp_ec_nlms_info_get(b, &flushed);
	printf("# skip frames after %u and %u fed zero frames: %u, %u\n",
	       full - 1u, full, early.skip_frames, flushed.skip_frames);
	CHECK_UINT(early.skip_frames, 0);
	CHECK_UINT(flushed.skip_frames, 1);
	udsp_ec_nlms.destroy(a);
	udsp_ec_nlms.destroy(b);
}

/*
 * Instance A runs frozen over a pause and B takes the reference feed: on one
 * frozen frame the outputs must match sample for sample, so only the history
 * differs. Fresh instances fed loud frames must not count the next silent frame
 * as quiet, and fed zeros must reach the skip path exactly.
 */
static void ref_feed_test(void)
{
	CASE("lec ref feed keeps the history in step");
	feed_history_check();
	feed_window_check();
	feed_skip_check();
}

/*
 * The case moves the reference against the echo onset of the composite hybrid.
 * The onset stands 2 ms after tap 0, a period after it, or 1 ms before it. The
 * first two runs read within 1 dB; the third under half the first and under
 * 3 dB, because the echo before tap 0 stays in the microphone. Each run takes
 * 3 s.
 */
static void ref_offset_test(unsigned rate)
{
	const char *what[3] = { "2 ms after tap 0", "one period after",
				"1 ms before" };
	int onset = (int)path_samples_get(4.0, rate);
	int margin[3] = { (int)path_samples_get(2.0, rate), (int)(rate / 100u),
			  -(int)path_samples_get(1.0, rate) };
	struct echo_path path;
	double erle[3];
	unsigned i;

	for (i = 0; i < 3u; i++) {
		struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
		uint32_t state = 0x0f1e2d3cu;

		udsp_ec_nlms.nlp(ec, false);
		path_composite_init(&path, rate, 12.0);
		erle[i] = erle_offset_run(ec, &path, &state, -10.0,
					  onset - margin[i], 0,
					  3u * FRAMES_PER_SECOND, 50u);
		udsp_ec_nlms.destroy(ec);
	}

	case_rate(rate, "lec echo loss needs the onset past the first tap");
	printf("# ERLE with the onset %s %.2f dB, %s %.2f dB, %s %.2f dB\n",
	       what[0], erle[0], what[1], erle[1], what[2], erle[2]);
	CHECK(erle[0] >= 25.0);
	CHECK(fabs(erle[1] - erle[0]) <= 1.0);
	CHECK(erle[2] < erle[0] / 2.0);
	CHECK(erle[2] < 3.0);
}

/*
 * A pass over a filter with no energy reports tap 0. A fresh instance takes
 * one frame with a loud reference and a silent microphone. The error is 0 on
 * every sample, so the frame adapts with a step of exactly 0. The frame still
 * marks the filter as written, so the pass after it reads every tap as 0. The
 * frame must count no hold, which shows that it adapted and that the pass ran.
 */
static void peak_zero_test(void)
{
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x5a5a1234u;

	CASE("lec peak tap of a filter with no energy reads tap 0 "
	     "after a pass");
	udsp_ec_nlms.nlp(ec, false);
	noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
	memset(mic, 0, sizeof(mic));
	CHECK(frame_loud(ref, UDSP_FRAME_SAMPLES));
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.frames, 1);
	CHECK_UINT(info.hold_frames, 0);
	CHECK_UINT(info.coef_peak_tap, 0);
	CHECK(info.coef_energy == 0.0f);
	udsp_ec_nlms.destroy(ec);
}

/*
 * Two taps of equal magnitude report the first. One reference impulse on sample
 * 0 and a microphone of +1000 on sample 20 and -1000 on sample 50 are the line.
 * They give two taps the same error magnitude. The peak reads tap 20. The frame
 * must count no hold, so the update stands.
 */
static void peak_tie_test(void)
{
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	struct udsp_ec *ec = udsp_ec_nlms.create(48);

	CASE("lec peak tap of a tie names the first tap");
	udsp_ec_nlms.nlp(ec, false);
	memset(ref, 0, sizeof(ref));
	memset(mic, 0, sizeof(mic));
	ref[0] = 16384;
	mic[20] = 1000;
	mic[50] = -1000;
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.hold_frames, 0);
	CHECK(info.coef_energy > 0.0f);
	CHECK_UINT(info.coef_peak_tap, 20);
	udsp_ec_nlms.destroy(ec);
}

/* LEC_ERLE_ALPHA of dsp/lec.c: the pole of the live powers that a trial is
 * judged by. */
#define LIVE_ALPHA		0.1f
/* LEC_PROBE_IMPROVE of dsp/lec.c: the share a trial must remove. */
#define LIVE_IMPROVE		0.5f

/*
 * The live microphone and output powers of ec_run(), rebuilt from the frames
 * the caller passes in and gets back. The trial verdict reads the canceller's
 * own copy of the two, so a case can compute what a compare of the output
 * power alone would have decided.
 */
struct live_track {
	float mic;
	float res;
};

static void live_track_add(struct live_track *t, const int16_t *mic,
			   const int16_t *out, unsigned n)
{
	t->mic += (udsp_ec_nlms_frame_power_get(mic, n) - t->mic) * LIVE_ALPHA;
	t->res += (udsp_ec_nlms_frame_power_get(out, n) - t->res) * LIVE_ALPHA;
}

/* One 8 kHz frame with a near end added to the echo; out receives the output
 * of the canceller. */
static void trial_frame_run(struct udsp_ec *ec, struct echo_path *path,
			    const int16_t *ref, double near_dbm0,
			    uint32_t *near_state, struct live_track *live,
			    int16_t *out)
{
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];

	path_run(path, ref, echo);
	floor_add(mic, echo, near_dbm0, near_state);
	memcpy(out, mic, sizeof(mic));
	udsp_ec_nlms.run(ec, out, ref);
	live_track_add(live, mic, out, UDSP_FRAME_SAMPLES);
}

static void trial_step(struct udsp_ec *ec, struct echo_path *path,
		       const int16_t *ref, double near_dbm0,
		       uint32_t *near_state, struct live_track *live,
		       struct udsp_ec_nlms_info *info)
{
	int16_t out[UDSP_FRAME_SAMPLES];

	trial_frame_run(ec, path, ref, near_dbm0, near_state, live, out);
	udsp_ec_nlms_info_get(ec, info);
}

/*
 * The first trial of a run: the frame the probe count moves on, and the frame
 * TRIAL_BUDGET later. The start frame never adapts and every frame after it
 * does, so the verdict falls on that later frame.
 */
struct trial_watch {
	struct udsp_ec_nlms_info start;
	struct udsp_ec_nlms_info verdict;
	unsigned start_frame;
	bool started;
	bool reached;
	float res_before;
	float res_after;
};

static void trial_watch_step(struct trial_watch *w,
			     const struct udsp_ec_nlms_info *info,
			     const struct live_track *live, unsigned f)
{
	if (!w->started && info->probes == 1u) {
		w->started = true;
		w->start_frame = f;
		w->start = *info;
		w->res_before = live->res;
		return;
	}
	if (!w->started || w->reached || f != w->start_frame + TRIAL_BUDGET)
		return;
	w->reached = true;
	w->verdict = *info;
	w->res_after = live->res;
}

static void trial_watch_print(const char *what, const struct trial_watch *w)
{
	double ratio = w->res_before > 0.0f ?
		       (double)w->res_after / w->res_before : 0.0;

	printf("# %s: start %u, verdict %u, smoothed residual echo after "
	       "over before %.3f\n", what, w->start_frame,
	       w->start_frame + TRIAL_BUDGET, ratio);
}

#define PAUSE_FIRST		490u
#define PAUSE_END		510u
#define PAUSE_LAST_FRAME	620u

/*
 * A latch that comes due in a far pause starts its trial on the next active
 * frame. The hybrid steps from 12 dB to 3 dB under a converged filter and a
 * near floor 6 dB under the echo. The trial must start on frame 510, after a
 * pause of 20 frames, and stand, as the ratio rule and not the echo alone
 * keeps.
 */
static void latch_pause_test(void)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info pause_info = { .probes = 0 };
	struct trial_watch watch = { .started = false };
	struct live_track live = { .mic = 0.0f };
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x9a8b7c6du;
	uint32_t floor_state = 0x0badcafeu;
	unsigned held = 0;
	bool start_loud = false;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	path_erl_set(&path, 3.0);
	for (f = 0; f <= PAUSE_LAST_FRAME; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];

		if (f >= PAUSE_FIRST && f < PAUSE_END)
			memset(ref, 0, sizeof(ref));
		else
			noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		trial_step(ec, &path, ref, -19.0, &floor_state, &live, &info);
		if (f < PAUSE_FIRST && info.double_talk)
			held++;
		if (f + 1u == PAUSE_FIRST)
			pause_info = info;
		if (!watch.started && info.probes == 1u)
			start_loud = frame_loud(ref, UDSP_FRAME_SAMPLES);
		trial_watch_step(&watch, &info, &live, f);
	}

	CASE("lec latch due in a far-end pause starts its trial on the next "
	     "active frame");
	trial_watch_print("latch after a pause", &watch);
	CHECK_UINT(pause_info.probes, 0);
	CHECK_UINT(held, PAUSE_FIRST);
	CHECK(watch.started);
	CHECK_UINT(watch.start_frame, PAUSE_END);
	CHECK(start_loud);
	CHECK(watch.reached);
	CHECK(!watch.verdict.double_talk);
	CHECK(watch.verdict.coef_energy >= 0.25f);
	CHECK_UINT(watch.verdict.probes, 1);
	CHECK_UINT(watch.verdict.diverge_resets, 0);
	CHECK(watch.res_after > LIVE_IMPROVE * watch.res_before);
	udsp_ec_nlms.destroy(ec);
}

/* The near end of a syllable case: a steady floor, or a talker on the
 * envelope of the far end. */
struct syllable_near {
	double dbm0;
	bool syllabic;
	uint32_t state;
};

#define SYLLABLE_FRAMES_MAX	(7u * FRAMES_PER_SECOND)
/* 3 s, so the trust count of dsp/lec.c completes before the syllables. */
#define SYLLABLE_TRAIN_FRAMES	(3u * FRAMES_PER_SECOND)

/* The far end follows the syllabic envelope from the first frame on, and the
 * run ends on the verdict of the first trial. */
static void syllable_run(struct udsp_ec *ec, struct echo_path *path,
			 uint32_t *state, struct syllable_near *near,
			 unsigned phi, struct trial_watch *watch)
{
	struct udsp_ec_nlms_info info;
	struct live_track live = { .mic = 0.0f };
	unsigned f;

	for (f = 0; f < SYLLABLE_FRAMES_MAX && !watch->reached; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		double near_dbm0 = near->syllabic ?
				   syllabic_dbm0_get(f + phi, near->dbm0) :
				   near->dbm0;

		noise_fill(ref, UDSP_FRAME_SAMPLES,
			   syllabic_dbm0_get(f + phi, -10.0), state);
		trial_step(ec, path, ref, near_dbm0, &near->state, &live,
			   &info);
		trial_watch_step(watch, &info, &live, f);
	}
}

/*
 * A trial is judged by residual echo over microphone, not by the echo alone. On
 * a far end with a 14 dB syllabic swing, a trial on a hybrid 2.5 dB louder
 * stands on a loud syllable. A talker level with a hybrid turned 3 dB louder
 * reads a changed path, and its trial falls on a quiet syllable.
 */
static void probe_syllable_test(void)
{
	struct syllable_near floor = { .dbm0 = -30.0, .state = 0x0badcafeu };
	struct syllable_near talker = { .dbm0 = -19.0, .syllabic = true,
					.state = 0xfeedfaceu };
	struct trial_watch loud = { .started = false };
	struct trial_watch quiet = { .started = false };
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x9a8b7c6du;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0, SYLLABLE_TRAIN_FRAMES, 20);
	udsp_ec_nlms_info_get(ec, &info);
	path_erl_set(&path, 9.5);
	syllable_run(ec, &path, &state, &floor, 5u, &loud);
	udsp_ec_nlms.destroy(ec);

	CASE("lec trial that ends on a loud syllable stands on a converged "
	     "filter");
	trial_watch_print("trial on a louder hybrid", &loud);
	CHECK(info.erle_db >= 18.0f);
	CHECK(loud.reached);
	CHECK(!loud.verdict.double_talk);
	CHECK(loud.verdict.coef_energy > loud.start.coef_energy);
	CHECK(loud.res_after > LIVE_IMPROVE * loud.res_before);

	ec = udsp_ec_nlms.create(48);
	state = 0x0badc0deu;
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0, SYLLABLE_TRAIN_FRAMES, 20);
	path_erl_set(&path, 9.0);
	syllable_run(ec, &path, &state, &talker, 19u, &quiet);
	udsp_ec_nlms.destroy(ec);

	CASE("lec trial that ends on a quiet syllable falls on a filter that "
	     "did not converge");
	trial_watch_print("trial on a near talker", &quiet);
	CHECK(quiet.reached);
	CHECK(quiet.verdict.coef_energy == quiet.start.coef_energy);
	CHECK(quiet.verdict.erle_db == quiet.start.erle_db);
	CHECK(quiet.verdict.echo_gain == quiet.start.echo_gain);
	CHECK_UINT(quiet.verdict.diverge_resets, 0);
	CHECK(quiet.res_after <= LIVE_IMPROVE * quiet.res_before);
}

/* The dBov of udsp_cn_level_get (dsp/cn.c) for a noise_fill floor, before
 * the canceller rounds it to a whole dB. */
static double floor_dbov_get(double dbm0)
{
	return -20.0 * log10(dbm0_rms_get(dbm0) / 32767.0);
}

#define ONSET_FLOOR_DBM0	-40.0
#define ONSET_ADAPT_FRAMES	20u
#define ONSET_SILENT_FRAMES	8u
#define ONSET_MOVE_MAX		0.001f

/*
 * A trial takes an onset after a silent tail at a bounded step. A hot hybrid
 * behind a 12 ms bulk delay latches over a floor at -40 dBm0, and the reference
 * goes silent. An onset at -48 dBm0 then carries the floor alone as error. The
 * onset frame adapts, and the coefficient energy moves by less than 0.001.
 */
static void probe_onset_test(void)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info quiet;
	struct live_track live = { .mic = 0.0f };
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	struct dtx_far far = { .state = 0x0c0ffee1u };
	uint32_t floor_state = 0x0badcafeu;
	int16_t ref[UDSP_FRAME_SAMPLES];
	double want = floor_dbov_get(ONSET_FLOOR_DBM0);
	uint32_t skipped;
	float move;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 12, 0.0);
	latch_drive(ec, &path, &far, ONSET_FLOOR_DBM0, &floor_state);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec step stays bounded at an onset after a silent tail inside a "
	     "trial");
	CHECK_UINT(info.probes, 1);
	for (f = 0; f < ONSET_ADAPT_FRAMES; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &far.state);
		trial_step(ec, &path, ref, ONSET_FLOOR_DBM0, &floor_state,
			   &live, &info);
	}
	skipped = info.skip_frames;
	memset(ref, 0, sizeof(ref));
	for (f = 0; f < ONSET_SILENT_FRAMES; f++)
		trial_step(ec, &path, ref, ONSET_FLOOR_DBM0, &floor_state,
			   &live, &quiet);
	noise_fill(ref, UDSP_FRAME_SAMPLES, TRIAL_QUIET_DBM0, &far.state);
	CHECK(frame_loud(ref, UDSP_FRAME_SAMPLES));
	trial_step(ec, &path, ref, ONSET_FLOOR_DBM0, &floor_state, &live,
		   &info);
	move = fabsf(info.coef_energy - quiet.coef_energy);
	printf("# coefficient energy before the onset %.6f, after %.6f, "
	       "move %.3g\n", quiet.coef_energy, info.coef_energy, move);
	printf("# near-end floor %.2f dBov, tracked floor %.2f dBov\n", want,
	       info.noise_dbov);
	/* The last three silent frames skip the tap loops, which the canceller
	 * does only on a window energy of exactly zero. */
	CHECK_UINT(quiet.skip_frames - skipped, 3);
	CHECK_UINT(info.hold_frames, quiet.hold_frames);
	CHECK(move < ONSET_MOVE_MAX);
	CHECK_UINT(info.probes, 1);
	CHECK_UINT(info.diverge_resets, 0);
	/* The floor term of the step denominator bounds the step, so the case
	 * means nothing unless the floor is measured. */
	CHECK(fabs(info.noise_dbov - want) <= 1.0);
	udsp_ec_nlms.destroy(ec);
}

/* G.168 test 3C t1 and t2, a trial on taps with no echo evidence. The composite
 * hybrid carries white far bursts from a third into a frame, and two near
 * sources talk from the start. The latch starts at 5 s. */
#define BARE_FAR_DBM0		-20.0
#define BARE_ERL_DB		6.0
#define BARE_LOSS_HALF_DB	18u
#define BARE_NEAR_OVER_DB	6.0
#define BARE_FLOOR_DBM0		-70.0
#define BARE_TAIL_MS		64u
#define BARE_ON_MS		230u
#define BARE_OFF_MS		70u
#define BARE_LEAD_MS		2u
#define BARE_LEAD_DB		30.0

struct bare_line {
	struct echo_path path;
	struct udsp_ec *ec;
	unsigned rate;
	unsigned frame;
	unsigned long pos;
	unsigned long near_phase[2];
	uint32_t far_state;
	uint32_t near_state[2];
	uint32_t hiss;
	bool pause;
};

static void bare_line_init(struct bare_line *l, unsigned rate)
{
	memset(l, 0, sizeof(*l));
	l->rate = rate;
	l->frame = rate / 100u;
	l->pos = l->frame * 2u / 3u;
	l->near_phase[1] = rate / 4u;
	l->far_state = 0x13572468u;
	l->near_state[0] = 0x2468ace0u;
	l->near_state[1] = 0x0badcafeu;
	l->hiss = 0x2468acefu;
	path_composite_init(&l->path, rate, BARE_ERL_DB);
	l->ec = udsp_ec_nlms.create_rate(BARE_TAIL_MS, rate);
	udsp_ec_nlms_ref_loss_set(l->ec, BARE_LOSS_HALF_DB);
}

/* One frame of the far end at the line: the burst, its lead and the
 * silence between bursts, or silence alone while pause is set. */
static void bare_far_fill(struct bare_line *l, int16_t *drive)
{
	unsigned long period = (BARE_ON_MS + BARE_OFF_MS) * l->rate / 1000u;
	unsigned long on = BARE_ON_MS * l->rate / 1000u;
	unsigned long lead = BARE_LEAD_MS * l->rate / 1000u;
	double low = pow(10.0, -BARE_LEAD_DB / 20.0);
	int16_t noise[UDSP_FRAME_MAX];
	unsigned i;

	noise_fill(noise, l->frame, BARE_FAR_DBM0, &l->far_state);
	for (i = 0; i < l->frame; i++) {
		unsigned long p = (l->pos + i) % period;

		drive[i] = !l->pause && p < on ?
			   sample_clamp((p < lead ? low : 1.0) * noise[i]) : 0;
	}
	l->pos += l->frame;
}

/* One frame of the line, with the near talker when talk is set. */
static void bare_frame_run(struct bare_line *l, bool talk,
			   struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info)
{
	double loss = pow(10.0, -(double)BARE_LOSS_HALF_DB / 40.0);
	double near_dbm0 = BARE_FAR_DBM0 - BARE_ERL_DB + BARE_NEAR_OVER_DB;
	int16_t drive[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t floor[UDSP_FRAME_MAX];
	int16_t near[2][UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	bare_far_fill(l, drive);
	css_fill(near[0], l->frame, near_dbm0, &l->near_state[0],
		 &l->near_phase[0], l->rate);
	css_fill(near[1], l->frame, near_dbm0, &l->near_state[1],
		 &l->near_phase[1], l->rate);
	path_run(&l->path, drive, echo);
	noise_fill(floor, l->frame, BARE_FLOOR_DBM0, &l->hiss);
	for (i = 0; i < l->frame; i++) {
		double talker = talk ? (double)near[0][i] + near[1][i] : 0.0;

		ref[i] = sample_clamp(loss * drive[i]);
		mic[i] = sample_clamp((double)echo[i] + floor[i] + talker);
	}
	udsp_ec_nlms.run(l->ec, mic, ref);
	udsp_ec_nlms_frame_get(l->ec, reading);
	udsp_ec_nlms_info_get(l->ec, info);
}

/* The near end talks BARE_TALK_AFTER frames into the trial, then the far end
 * runs alone for the 1.4 s of t2. BARE_REF_FRAMES of far end alone give the
 * echo's coefficient energy. */
#define BARE_TALK_AFTER		57u
#define BARE_ALONE_FRAMES	140u
#define BARE_LATCH_LIMIT	(8u * FRAMES_PER_SECOND)
#define BARE_REF_FRAMES		(7u * FRAMES_PER_SECOND)
#define BARE_COEF_RATIO		2.0
#define BARE_ERLE_DB		18.0

struct bare_figures {
	float echo_coef;
	float keep_coef;
	float rise;
	float erle_db;
	unsigned start;
	unsigned end;
	unsigned arm;
	unsigned how;
};

static void bare_run(unsigned rate, struct bare_figures *fig)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct bare_line l;
	unsigned talk = BARE_LATCH_LIMIT;
	float before = 0.0f;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	bare_line_init(&l, rate);
	for (f = 0; f < BARE_REF_FRAMES; f++)
		bare_frame_run(&l, false, &reading, &info);
	fig->echo_coef = info.coef_energy;
	udsp_ec_nlms.destroy(l.ec);
	bare_line_init(&l, rate);
	for (f = 0; f < talk + BARE_ALONE_FRAMES; f++) {
		bare_frame_run(&l, f < talk, &reading, &info);
		if (!fig->arm && reading.trial_arm) {
			fig->arm = reading.trial_arm;
			fig->start = f;
			talk = f + BARE_TALK_AFTER;
		} else if (fig->arm && !fig->how) {
			if (info.coef_energy - before > fig->rise)
				fig->rise = info.coef_energy - before;
			if (reading.trial_end) {
				fig->how = reading.trial_end;
				fig->end = f;
				fig->keep_coef = info.coef_energy;
			}
		}
		before = info.coef_energy;
	}
	fig->erle_db = info.erle_db;
	udsp_ec_nlms.destroy(l.ec);
}

/* The latch run at which the near end of bare_snapshot_run stops, short of
 * LEC_GEIGEL_LATCH_MAX of dsp/lec.c by less than one hangover. */
#define BARE_SNAP_RUN		481u

/*
 * The line of bare_run with the latch due on a frame that adapts and fires
 * (probe_snapshot_test): that frame starts the trial, and its update rolls
 * back. fig takes the figures of bare_run; before and start the coefficient
 * energy before and after the start frame.
 */
static void bare_snapshot_run(unsigned rate, struct bare_figures *fig,
			      float *before, float *start)
{
	unsigned long period = (BARE_ON_MS + BARE_OFF_MS) * rate / 1000u;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct bare_line l;
	unsigned talk = BARE_LATCH_LIMIT;
	float last = 0.0f;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	*before = 0.0f;
	*start = 0.0f;
	bare_line_init(&l, rate);
	for (f = 0; f < BARE_REF_FRAMES; f++)
		bare_frame_run(&l, false, &reading, &info);
	fig->echo_coef = info.coef_energy;
	udsp_ec_nlms.destroy(l.ec);
	bare_line_init(&l, rate);
	for (f = 0; f < talk + BARE_ALONE_FRAMES; f++) {
		bare_frame_run(&l, !l.pause && f < talk, &reading, &info);
		if (!fig->arm && !l.pause && reading.dt_fired &&
		    reading.dt_run >= BARE_SNAP_RUN) {
			l.pause = true;
		} else if (l.pause && !info.double_talk) {
			l.pause = false;
			l.pos = (l.pos / period + 1u) * period - l.frame / 3u;
		}
		if (!fig->arm && reading.trial_arm) {
			fig->arm = reading.trial_arm;
			fig->start = f;
			*before = last;
			*start = info.coef_energy;
			talk = f + BARE_TALK_AFTER;
		} else if (fig->arm && !fig->how) {
			if (info.coef_energy - last > fig->rise)
				fig->rise = info.coef_energy - last;
			if (reading.trial_end) {
				fig->how = reading.trial_end;
				fig->end = f;
				fig->keep_coef = info.coef_energy;
			}
		}
		last = info.coef_energy;
	}
	fig->erle_db = info.erle_db;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * A trial on taps without echo evidence: the first trial is a latch trial, kept
 * at BARE_COEF_RATIO times the echo's coefficient energy or less. The ERLE
 * after the far end alone reaches BARE_ERLE_DB. Without the step floor of such
 * a trial, the burst after silence drives it diverged on the talker.
 */
static void trial_bare_test(unsigned rate)
{
	struct bare_figures fig;

	bare_run(rate, &fig);
	case_rate(rate, "lec trial on taps without echo evidence keeps a "
		  "filter of the echo on a line that starts in double talk");
	printf("# echo coefficient energy %.3f; first trial arm %u from frame "
	       "%u, end %u on frame %u, coefficient energy %.3f at its end, "
	       "%.2f of the echo; largest rise in one frame %.3f; ERLE %.1f dB "
	       "after %u frames of the far end alone\n", fig.echo_coef,
	       fig.arm, fig.start, fig.how, fig.end, fig.keep_coef,
	       fig.keep_coef / fig.echo_coef, fig.rise, fig.erle_db,
	       BARE_ALONE_FRAMES);
	CHECK_UINT(fig.arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK(fig.start < BARE_LATCH_LIMIT);
	CHECK_UINT(fig.how, UDSP_EC_NLMS_TRIAL_END_KEPT);
	CHECK(fig.keep_coef <= BARE_COEF_RATIO * fig.echo_coef);
	CHECK(fig.erle_db >= BARE_ERLE_DB);
}

#define WINDOW_BURST_FRAMES	50u
#define WINDOW_PAUSE_FRAMES	8u
#define WINDOW_FLOOR_DBM0	-40.0

/*
 * The floor is measured only on a whole quiet window. A far burst behind a
 * 40 ms bulk delay starts on the first frame. Its tail fills four pause frames
 * 18 dB over a floor at -40 dBm0. After eight pause frames the tracked floor
 * must read the near level within 1 dB.
 */
static void noise_floor_window_test(void)
{
	struct udsp_ec_nlms_info info;
	struct live_track live = { .mic = 0.0f };
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x51ade0f1u;
	uint32_t floor_state = 0x600dcafeu;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t out[UDSP_FRAME_SAMPLES];
	double want = floor_dbov_get(WINDOW_FLOOR_DBM0);
	int echo_dbov = 127;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	udsp_ec_nlms.freeze(ec, true);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 40, 12.0);
	for (f = 0; f < WINDOW_BURST_FRAMES; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
		trial_step(ec, &path, ref, WINDOW_FLOOR_DBM0, &floor_state,
			   &live, &info);
	}
	memset(ref, 0, sizeof(ref));
	for (f = 0; f < WINDOW_PAUSE_FRAMES; f++) {
		trial_frame_run(ec, &path, ref, WINDOW_FLOOR_DBM0,
				&floor_state, &live, out);
		if (f == 0u)
			echo_dbov = udsp_cn_level_get(out, UDSP_FRAME_SAMPLES);
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec measures the near-end floor once the tail window is quiet");
	printf("# near-end floor %.2f dBov, pause frame 1 output %d dBov, "
	       "tracked floor %.2f dBov\n", want, echo_dbov, info.noise_dbov);
	CHECK(echo_dbov <= want - 10.0);
	CHECK(fabs(info.noise_dbov - want) <= 1.0);
	udsp_ec_nlms.destroy(ec);
}

#define NLP_FLOOR_DBM0		-48.0
#define NLP_FLOOR_TALK_DBM0	-40.0
#define NLP_FLOOR_DRAIN		10u
#define NLP_FLOOR_BURST		10u
#define NLP_FLOOR_CYCLE		13u
#define NLP_FLOOR_SETTLE	100u
#define NLP_FLOOR_FRAMES	300u
#define NLP_FLOOR_TALK_FRAMES	100u

/* A NULL talk_state means no talker, while the floor is always on the line. */
static void nlp_floor_step(struct udsp_ec *ec, struct echo_path *path,
			   bool far, uint32_t *state, uint32_t *floor_state,
			   uint32_t *talk_state, struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];

	if (far)
		noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, state);
	else
		memset(ref, 0, sizeof(ref));
	path_run(path, ref, echo);
	floor_add(mic, echo, NLP_FLOOR_DBM0, floor_state);
	if (talk_state)
		floor_add(mic, mic, NLP_FLOOR_TALK_DBM0, talk_state);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, info);
}

/*
 * The processor stays engaged over a near floor 8 dB under the echo. In the
 * gaps of far bursts, the echo-likeness test counts the floor with its headroom
 * as echo. The processor acts on 29 of 32 gap frames or more. A near talker
 * 8 dB over the floor still releases it, and the detector stays clear.
 */
static void nlp_floor_test(void)
{
	struct udsp_ec_nlms_info converged;
	struct udsp_ec_nlms_info seeded;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x31415926u;
	uint32_t floor_state = 0x0badcafeu;
	uint32_t talk_state = 0xa5a5c3c3u;
	double want = floor_dbov_get(NLP_FLOOR_DBM0);
	unsigned gap_frames = 0;
	unsigned gap_engaged = 0;
	unsigned talk_frames = 0;
	unsigned talk_nlp = 0;
	unsigned talk_dt = 0;
	unsigned f;

	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 30.0);
	(void)erle_run(ec, &path, &state, -10.0, 150u, 20);
	udsp_ec_nlms_info_get(ec, &converged);
	udsp_ec_nlms.freeze(ec, true);
	for (f = 0; f < NLP_FLOOR_DRAIN; f++)
		nlp_floor_step(ec, &path, false, &state, &floor_state, NULL,
			       &seeded);
	for (f = 0; f < NLP_FLOOR_FRAMES; f++) {
		unsigned pos = f % NLP_FLOOR_CYCLE;

		nlp_floor_step(ec, &path, pos < NLP_FLOOR_BURST, &state,
			       &floor_state, NULL, &info);
		/* The first gap frame still carries the echo of the burst. */
		if (f < NLP_FLOOR_SETTLE || pos <= NLP_FLOOR_BURST)
			continue;
		gap_frames++;
		if (info.nlp_active)
			gap_engaged++;
	}

	CASE("lec nlp engages over a near-end floor close to the echo");
	printf("# %u of %u gap frames engaged (%.1f%%), converged erle %.1f dB, "
	       "erle %.1f dB, near-end floor %.2f dBov, seed %.2f dBov, "
	       "tracked %.2f dBov\n", gap_engaged, gap_frames,
	       gap_frames ? 100.0 * gap_engaged / gap_frames : 0.0,
	       (double)converged.erle_db, (double)info.erle_db, want,
	       (double)seeded.noise_dbov, (double)info.noise_dbov);
	CHECK(converged.erle_db >= 18.0f);
	CHECK(fabs(seeded.noise_dbov - want) <= 1.0);
	CHECK(gap_frames >= 20u);
	CHECK(gap_engaged * 32u >= gap_frames * 29u);
	CHECK(info.erle_db >= 6.0f);
	CHECK(info.erle_db < 18.0f);

	for (f = NLP_FLOOR_FRAMES; f < NLP_FLOOR_FRAMES + NLP_FLOOR_TALK_FRAMES;
	     f++) {
		unsigned pos = f % NLP_FLOOR_CYCLE;
		bool talk = pos == NLP_FLOOR_BURST + 1u;

		nlp_floor_step(ec, &path, pos < NLP_FLOOR_BURST, &state,
			       &floor_state, talk ? &talk_state : NULL, &info);
		if (!talk)
			continue;
		talk_frames++;
		if (info.nlp_active)
			talk_nlp++;
		if (info.double_talk)
			talk_dt++;
	}

	CASE("lec nlp still releases for a near talker over the floor");
	printf("# %u talk frames, processor %u, double talk %u, erle %.1f dB, "
	       "tracked floor %.2f dBov\n", talk_frames, talk_nlp, talk_dt,
	       (double)info.erle_db, (double)info.noise_dbov);
	CHECK(talk_frames >= 5u);
	CHECK_UINT(talk_nlp, 0);
	CHECK_UINT(talk_dt, 0);
	udsp_ec_nlms.destroy(ec);
}

/* 1 when dt_fired disagrees with the rule of the detector, in either
 * direction: the detector fires when the window is active and the microphone
 * reaches the bar. window_run reads 0 exactly on an inactive window. */
static unsigned reading_bar_miss(const struct udsp_ec_nlms_frame *reading)
{
	return reading->dt_fired != (reading->window_run != 0u &&
				     reading->mic_power >= reading->bar);
}

#define READING_FLOOR_DBM0	-40.0

/* One frame of far end or of silence through the hybrid, and its reading.
 * floor_state, when given, adds the 8 kHz near-end floor of floor_add at
 * READING_FLOOR_DBM0. */
static void reading_frame_run(struct udsp_ec *ec, struct echo_path *path,
			      uint32_t *state, uint32_t *floor_state, bool loud,
			      struct udsp_ec_nlms_frame *reading)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];

	feed_frame_make(path, state, loud, ref, mic);
	if (floor_state)
		floor_add(mic, mic, READING_FLOOR_DBM0, floor_state);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_frame_get(ec, reading);
}

/*
 * The frame reading: one near frame at -15 dBm0 fires with a hold of 0 after a
 * far end that raises no verdict. The hangover keeps 23 more frames, so the
 * verdict counts 24 dt frames and 1 fired frame. The reading fires exactly
 * where the microphone reaches the bar.
 */
static void reading_verdict_test(void)
{
	struct udsp_ec_nlms_info start, before, after;
	struct udsp_ec_nlms_frame reading;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t near[UDSP_FRAME_SAMPLES];
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xfeedfaceu;
	unsigned bar_miss = 0;
	unsigned acted = 0;
	unsigned f;

	CASE("lec frame reading names a fresh verdict and the hold behind it");
	ec = gate_fixture_create(&path, 23.0, 36u, &state, &acted);
	udsp_ec_nlms_info_get(ec, &start);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		verdict_far_run(ec, &path, &state, 1u, &acted);
		udsp_ec_nlms_frame_get(ec, &reading);
		bar_miss += reading_bar_miss(&reading);
	}
	udsp_ec_nlms_info_get(ec, &before);
	printf("# before the near frame: erle %.1f dB, dt %u over the second\n",
	       (double)before.erle_db,
	       before.nlp_dt_frames - start.nlp_dt_frames);
	CHECK_UINT(before.nlp_dt_frames - start.nlp_dt_frames, 0);
	CHECK(before.erle_db < 18.0f);
	noise_fill(ref, UDSP_FRAME_SAMPLES, -10.0, &state);
	noise_fill(near, UDSP_FRAME_SAMPLES, -15.0, &near_state);
	verdict_step(ec, &path, ref, near, &acted);
	udsp_ec_nlms_frame_get(ec, &reading);
	udsp_ec_nlms_info_get(ec, &after);
	bar_miss += reading_bar_miss(&reading);
	CHECK(reading.dt_fired);
	CHECK_UINT(reading.hold_left, 0);
	CHECK(reading.mic_power >= reading.bar);
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(reading.nlp_verdict), "dt"));
	CHECK(after.double_talk);
	for (f = 1; f <= 24u; f++) {
		verdict_far_run(ec, &path, &state, 1u, &acted);
		udsp_ec_nlms_frame_get(ec, &reading);
		udsp_ec_nlms_info_get(ec, &after);
		bar_miss += reading_bar_miss(&reading);
		if (f == 1u) {
			CHECK(!reading.dt_fired);
			CHECK_UINT(reading.hold_left, 24);
			CHECK(after.double_talk);
			CHECK(reading.mic_power < reading.bar);
		}
		if (f == 23u)
			CHECK(after.double_talk);
	}
	printf("# after 24 far frames: fired %u dt %u, bar misses %u\n",
	       after.nlp_dt_fired_frames - before.nlp_dt_fired_frames,
	       after.nlp_dt_frames - before.nlp_dt_frames, bar_miss);
	CHECK(!after.double_talk);
	CHECK_UINT(bar_miss, 0);
	CHECK_UINT(after.nlp_dt_fired_frames - before.nlp_dt_fired_frames, 1);
	CHECK_UINT(after.nlp_dt_frames - before.nlp_dt_frames, 24);
	CHECK_UINT((after.nlp_dt_frames - after.nlp_dt_fired_frames) -
		   (before.nlp_dt_frames - before.nlp_dt_fired_frames), 23);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The window count of a 48 ms instance: the window covers 5 frames, so 3 loud
 * frames keep it active for 4 zero frames. The count reads 1 to 3, then 4 on
 * the first zero frame, the tail; the fifth zero frame ends both. A count that
 * never resets, or a tail read from the window alone, fails.
 */
static void reading_window_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x2718281du;
	unsigned f;

	case_rate(rate, "lec frame reading counts the frames since the window "
		  "woke and marks the tail");
	path_flat_init(&path, rate, 8, 12.0);
	for (f = 0; f < 10u; f++)
		reading_frame_run(ec, &path, &state, NULL, false, &reading);
	CHECK_UINT(reading.window_run, 0);
	for (f = 0; f < 3u; f++) {
		reading_frame_run(ec, &path, &state, NULL, true, &reading);
		CHECK_UINT(reading.window_run, f + 1u);
		CHECK(!reading.ref_quiet);
	}
	reading_frame_run(ec, &path, &state, NULL, false, &reading);
	CHECK_UINT(reading.window_run, 4);
	CHECK(reading.ref_quiet);
	for (f = 1; f < 5u; f++)
		reading_frame_run(ec, &path, &state, NULL, false, &reading);
	CHECK_UINT(reading.window_run, 0);
	CHECK(!reading.ref_quiet);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The reset inside an active window. Three loud frames leave the count at 3,
 * and the reset clears the whole reading. A read right after it returns
 * zeros, and the next loud frame reads a count of 1 and a frame count of 1. A
 * reset that keeps the count reads 4 on that frame. A reset that keeps the
 * reading reads 3 frames before it.
 */
static void reading_reset_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x16180339u;
	unsigned f;

	CASE("lec reset clears the frame reading inside an active window");
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 12.0);
	for (f = 0; f < 3u; f++)
		reading_frame_run(ec, &path, &state, NULL, true, &reading);
	CHECK_UINT(reading.window_run, 3);
	CHECK(reading.bar > 0.0f);
	udsp_ec_nlms_reset(ec);
	udsp_ec_nlms_frame_get(ec, &reading);
	CHECK_UINT(reading.frames, 0);
	CHECK_UINT(reading.window_run, 0);
	CHECK(!reading.dt_fired);
	CHECK(reading.bar == 0.0f);
	reading_frame_run(ec, &path, &state, NULL, true, &reading);
	CHECK_UINT(reading.window_run, 1);
	CHECK_UINT(reading.frames, 1);
	udsp_ec_nlms.destroy(ec);
}

#define READING_PAUSE_FRAMES	10u
#define READING_FAR_FRAMES	20u
/* The far-end training of the floor fixture, and the longer one that completes
 * the trust count of dsp/lec.c for a case on a trusted bar. With no near floor
 * the tracked ERLE stays over the trust line while the count runs. */
#define READING_TRAIN_FRAMES	150u
#define READING_TRUST_TRAIN_FRAMES	250u

/* A run of far end or of silence over the near-end floor. It returns the count
 * of frames whose dt_fired disagrees with the bar (reading_bar_miss). */
static unsigned reading_floor_frames_run(struct udsp_ec *ec,
					 struct echo_path *path,
					 uint32_t *state, uint32_t *floor_state,
					 bool loud, unsigned frames,
					 struct udsp_ec_nlms_frame *reading)
{
	unsigned miss = 0;
	unsigned f;

	for (f = 0; f < frames; f++) {
		reading_frame_run(ec, path, state, floor_state, loud, reading);
		miss += reading_bar_miss(reading);
	}
	return miss;
}

/*
 * The floor term of the bar: trained until the trust count completes, with a
 * floor at -40 dBm0 measured in a pause. On the first tail frame with an
 * estimate under half the floor, the bar reaches the margin times the headroom
 * times the floor. A bar without either term, or a disagreeing flag, fails.
 */
static void reading_floor_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame first = { 0 };
	struct udsp_ec_nlms_frame tail = { 0 };
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x77665544u;
	uint32_t floor_state = 0x0f100f10u;
	unsigned bar_miss;
	unsigned acted = 0;
	unsigned f;

	CASE("lec frame reading carries the floor in the bar");
	ec = gate_fixture_create(&path, 12.0, READING_TRUST_TRAIN_FRAMES,
				 &state, &acted);
	bar_miss = reading_floor_frames_run(ec, &path, &state, &floor_state,
					    false, READING_PAUSE_FRAMES,
					    &reading);
	printf("# floor after the first pause %.1f\n",
	       (double)reading.noise_power);
	CHECK(reading.noise_power > 0.0f);
	bar_miss += reading_floor_frames_run(ec, &path, &state, &floor_state,
					     true, READING_FAR_FRAMES,
					     &reading);
	for (f = 0; f < READING_PAUSE_FRAMES; f++) {
		reading_frame_run(ec, &path, &state, &floor_state, false,
				  &reading);
		bar_miss += reading_bar_miss(&reading);
		if (!first.ref_quiet && reading.ref_quiet)
			first = reading;
		if (!tail.ref_quiet && reading.ref_quiet &&
		    reading.estimate_power * 2.0f < reading.noise_power)
			tail = reading;
	}
	printf("# first tail frame %u: estimate %.1f bar %.1f floor %.1f\n",
	       first.frames, (double)first.estimate_power, (double)first.bar,
	       (double)first.noise_power);
	printf("# first tail frame under half the floor %u: estimate %.1f "
	       "bar %.1f floor %.1f\n", tail.frames,
	       (double)tail.estimate_power, (double)tail.bar,
	       (double)tail.noise_power);
	CHECK(tail.ref_quiet);
	/* 1.41 is LEC_GEIGEL_MARGIN and 2.0 LEC_GEIGEL_FLOOR_HEADROOM: the bar
	 * is every term plus the floor with its headroom, times the margin, so
	 * it never stands under their product with the floor. */
	CHECK(tail.bar >= 1.41f * 2.0f * tail.noise_power);
	CHECK_UINT(bar_miss, 0);
	udsp_ec_nlms.destroy(ec);
}

#define HANGOVER_MARGINAL_DBM0		-29.0
#define HANGOVER_SOFT_DBM0		-45.0
#define HANGOVER_EXCURSION_DBM0		-36.0
#define HANGOVER_LOUD_NEAR_DBM0		-22.0
#define HANGOVER_UNDER_TWICE_DBM0	-34.5
#define HANGOVER_OVER_TWICE_DBM0	-33.0
#define HANGOVER_UNDER_SOFT_FAR_DBM0	-26.0
#define HANGOVER_UNDER_SOFT_DBM0	-31.5
#define HANGOVER_OVER_SOFT_FAR_DBM0	-21.0
#define HANGOVER_OVER_SOFT_DBM0		-27.5
#define HANGOVER_SETTLE_FRAMES		30u
#define HANGOVER_FAR_LIMIT		40u
/* LEC_GEIGEL_MARGINAL_RATIO and LEC_GEIGEL_SOFT_RATIO of dsp/lec.c: a verdict
 * under both ratios, with no hangover running, sets the short hangover. */
#define HANGOVER_MARGINAL_RATIO		2.0f
#define HANGOVER_SOFT_RATIO		16.0f

/*
 * The fixture of reading_verdict_test, or with floor set the fixture of
 * reading_floor_test. The far end runs at far_dbm0 or, with pause set, stays
 * silent. A floor line carries a near-end floor at floor_dbm0 on every frame.
 */
struct hangover_line {
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state;
	uint32_t near_state;
	uint32_t floor_state;
	double far_dbm0;
	double floor_dbm0;
	bool floor;
	bool pause;
};

/* One frame, with a near-end burst on top when near is given. */
static void hangover_frame_run(struct hangover_line *line, const int16_t *near,
			       struct udsp_ec_nlms_frame *reading,
			       struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	unsigned i;

	if (line->pause)
		memset(ref, 0, sizeof(ref));
	else
		noise_fill(ref, UDSP_FRAME_SAMPLES, line->far_dbm0,
			   &line->state);
	path_run(&line->path, ref, echo);
	floor_add(mic, echo, line->floor_dbm0,
		  line->floor ? &line->floor_state : NULL);
	for (i = 0; near && i < UDSP_FRAME_SAMPLES; i++)
		mic[i] = sample_clamp((double)mic[i] + near[i]);
	udsp_ec_nlms.run(line->ec, mic, ref);
	udsp_ec_nlms_frame_get(line->ec, reading);
	udsp_ec_nlms_info_get(line->ec, info);
}

static void hangover_near_run(struct hangover_line *line, double near_dbm0,
			      struct udsp_ec_nlms_frame *reading,
			      struct udsp_ec_nlms_info *info)
{
	int16_t near[UDSP_FRAME_SAMPLES];

	noise_fill(near, UDSP_FRAME_SAMPLES, near_dbm0, &line->near_state);
	hangover_frame_run(line, near, reading, info);
}

/* The 23 dB hybrid of reading_verdict_test, after its second of far end
 * alone, with no near-end floor. */
static void hangover_line_create(struct hangover_line *line)
{
	unsigned acted = 0;

	memset(line, 0, sizeof(*line));
	line->state = 0x31415926u;
	line->near_state = 0xfeedfaceu;
	line->far_dbm0 = -10.0;
	line->ec = gate_fixture_create(&line->path, 23.0, 36u, &line->state,
				       &acted);
	verdict_far_run(line->ec, &line->path, &line->state,
			FRAMES_PER_SECOND, &acted);
}

/* The 12 dB hybrid of reading_floor_test after train frames of far end. A
 * far-end pause then measures its floor, and HANGOVER_SETTLE_FRAMES of far
 * end at far_dbm0 follow. */
static void hangover_floor_line_train(struct hangover_line *line,
				      double far_dbm0, unsigned train)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned acted = 0;
	unsigned f;

	memset(line, 0, sizeof(*line));
	line->state = 0x77665544u;
	line->near_state = 0xfeedfaceu;
	line->floor_state = 0x0f100f10u;
	line->far_dbm0 = far_dbm0;
	line->floor_dbm0 = READING_FLOOR_DBM0;
	line->floor = true;
	line->ec = gate_fixture_create(&line->path, 12.0, train, &line->state,
				       &acted);
	(void)reading_floor_frames_run(line->ec, &line->path, &line->state,
				       &line->floor_state, false,
				       READING_PAUSE_FRAMES, &reading);
	for (f = 0; f < HANGOVER_SETTLE_FRAMES; f++)
		hangover_frame_run(line, NULL, &reading, &info);
}

static void hangover_floor_line_create(struct hangover_line *line,
				       double far_dbm0)
{
	hangover_floor_line_train(line, far_dbm0, READING_TRAIN_FRAMES);
}

/* Far-end frames up to the first one without double talk, at most limit of
 * them. It returns the count of the frames that still read double talk. */
static unsigned hangover_far_run(struct hangover_line *line, unsigned limit)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned f;

	for (f = 0; f < limit; f++) {
		hangover_frame_run(line, NULL, &reading, &info);
		if (!info.double_talk)
			return f;
	}
	return limit;
}

static double ratio_db_get(float power, float reference)
{
	return 10.0 * log10((double)power / (double)reference);
}

static void hangover_verdict_print(const char *what, double near_dbm0,
				   const struct udsp_ec_nlms_frame *reading)
{
	printf("# %s at %.1f dBm0: %.2f dB over the bar, ", what, near_dbm0,
	       ratio_db_get(reading->mic_power, reading->bar));
	if (reading->noise_power > 0.0f)
		printf("%.2f dB over the floor", ratio_db_get(
		       reading->mic_power, reading->noise_power));
	else
		printf("no floor");
	printf(", hangover %u\n", reading->hold_left);
}

struct hangover_result {
	struct udsp_ec_nlms_frame verdict;
	unsigned lasted;
	uint32_t fired;
	uint32_t dt;
};

/* One near-end frame and then far-end frames until double talk ends. */
static void hangover_verdict_run(struct hangover_line *line, const char *what,
				 double near_dbm0,
				 struct hangover_result *result)
{
	struct udsp_ec_nlms_info before, after;

	udsp_ec_nlms_info_get(line->ec, &before);
	hangover_near_run(line, near_dbm0, &result->verdict, &after);
	hangover_verdict_print(what, near_dbm0, &result->verdict);
	result->lasted = hangover_far_run(line, HANGOVER_FAR_LIMIT);
	udsp_ec_nlms_info_get(line->ec, &after);
	result->fired = after.nlp_dt_fired_frames - before.nlp_dt_fired_frames;
	result->dt = after.nlp_dt_frames - before.nlp_dt_frames;
	printf("# double talk for %u far frames: fired %u dt %u\n",
	       result->lasted, result->fired, result->dt);
}

/* The verdict fired with no hangover running, under twice the bar and within
 * 12 dB of a measured floor. The reading shows every condition of the short
 * hangover but the repeat limit. */
static void hangover_short_check(const struct udsp_ec_nlms_frame *verdict)
{
	CHECK(verdict->dt_fired);
	CHECK_UINT(verdict->hold_left, 0);
	CHECK(verdict->mic_power < HANGOVER_MARGINAL_RATIO * verdict->bar);
	CHECK(verdict->noise_power > 0.0f);
	CHECK(verdict->mic_power < HANGOVER_SOFT_RATIO * verdict->noise_power);
}

/*
 * A floor excursion: a far end at -45 dBm0 puts the estimate under the floor.
 * One near frame within 3 dB of the bar and 12 dB of the floor sets a hangover
 * of 4 frames. Double talk ends on the fourth far frame. A detector that
 * gives every verdict 24 frames fails here.
 */
static void hangover_floor_test(void)
{
	struct hangover_result result;
	struct hangover_line line;

	CASE("lec hangover lasts four frames on a floor excursion");
	hangover_floor_line_create(&line, HANGOVER_SOFT_DBM0);
	hangover_verdict_run(&line, "excursion frame", HANGOVER_EXCURSION_DBM0,
			     &result);
	hangover_short_check(&result.verdict);
	CHECK_UINT(result.lasted, 3);
	CHECK_UINT(result.fired, 1);
	CHECK_UINT(result.dt, 4);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A marginal verdict on a line without a measured floor, on the fixture of
 * reading_verdict_test. The near-end frame reaches the bar by less than 3 dB.
 * The line reads a floor of 0, so the frame does not stand within 12 dB of it
 * and sets the long hangover. A detector that gives the short hangover to a
 * line without a floor fails here.
 */
static void hangover_no_floor_test(void)
{
	struct hangover_result result;
	struct hangover_line line;

	CASE("lec hangover lasts 24 frames on a marginal verdict without a "
	     "floor");
	hangover_line_create(&line);
	hangover_verdict_run(&line, "marginal frame", HANGOVER_MARGINAL_DBM0,
			     &result);
	CHECK(result.verdict.dt_fired);
	CHECK_UINT(result.verdict.hold_left, 0);
	CHECK(result.verdict.mic_power <
	      HANGOVER_MARGINAL_RATIO * result.verdict.bar);
	CHECK(result.verdict.noise_power == 0.0f);
	CHECK_UINT(result.lasted, 23);
	CHECK_UINT(result.fired, 1);
	CHECK_UINT(result.dt, 24);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A marginal verdict far over the floor. The fixture of reading_floor_test
 * carries a far end at -10 dBm0, so the echo stands near -22 dBm0. It trains
 * until the trust count completes. The near-end frame reaches the bar by less
 * than 3 dB, more than 12 dB over the floor. A frame that loud is a talker, so
 * it sets the long hangover.
 */
static void hangover_loud_test(void)
{
	struct hangover_result result;
	struct hangover_line line;

	CASE("lec hangover lasts 24 frames on a marginal verdict far over the "
	     "floor");
	hangover_floor_line_train(&line, -10.0, READING_TRUST_TRAIN_FRAMES);
	hangover_verdict_run(&line, "marginal frame", HANGOVER_LOUD_NEAR_DBM0,
			     &result);
	CHECK(result.verdict.dt_fired);
	CHECK_UINT(result.verdict.hold_left, 0);
	CHECK(result.verdict.mic_power <
	      HANGOVER_MARGINAL_RATIO * result.verdict.bar);
	CHECK(result.verdict.noise_power > 0.0f);
	CHECK(result.verdict.mic_power >=
	      HANGOVER_SOFT_RATIO * result.verdict.noise_power);
	CHECK_UINT(result.lasted, 23);
	CHECK_UINT(result.fired, 1);
	CHECK_UINT(result.dt, 24);
	udsp_ec_nlms.destroy(line.ec);
}

/* A verdict inside a running hangover: a second excursion on the frame after
 * the first meets the hangover and the repeat limit. It takes the long one,
 * although it stands within the marginal and soft bounds. */
static void hangover_rearm_test(void)
{
	struct udsp_ec_nlms_info before, after;
	struct udsp_ec_nlms_frame reading;
	struct hangover_line line;
	unsigned lasted;

	CASE("lec hangover lasts 24 frames on a verdict inside a hangover");
	hangover_floor_line_create(&line, HANGOVER_SOFT_DBM0);
	udsp_ec_nlms_info_get(line.ec, &before);
	hangover_near_run(&line, HANGOVER_EXCURSION_DBM0, &reading, &after);
	hangover_verdict_print("first excursion", HANGOVER_EXCURSION_DBM0,
			       &reading);
	hangover_short_check(&reading);
	hangover_near_run(&line, HANGOVER_EXCURSION_DBM0, &reading, &after);
	hangover_verdict_print("second excursion", HANGOVER_EXCURSION_DBM0,
			       &reading);
	CHECK(reading.dt_fired);
	CHECK_UINT(reading.hold_left, 4);
	CHECK(reading.mic_power < HANGOVER_MARGINAL_RATIO * reading.bar);
	CHECK(reading.mic_power < HANGOVER_SOFT_RATIO * reading.noise_power);
	lasted = hangover_far_run(&line, HANGOVER_FAR_LIMIT);
	udsp_ec_nlms_info_get(line.ec, &after);
	printf("# double talk for %u far frames: fired %u\n", lasted,
	       after.nlp_dt_fired_frames - before.nlp_dt_fired_frames);
	CHECK_UINT(lasted, 23);
	CHECK_UINT(after.nlp_dt_fired_frames - before.nlp_dt_fired_frames, 2);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * The edge of the short hangover at twice the bar, on the fixture of
 * hangover_floor_test. A frame about 2.5 dB over the bar takes the short
 * hangover. A frame about 3.5 dB over the bar takes the long hangover. Both
 * stand within 12 dB of the floor. A marginal ratio of 1.5 or 2.5 fails one of
 * the two.
 */
static void hangover_marginal_edge_test(void)
{
	struct hangover_result result;
	struct hangover_line line;

	CASE("lec double talk takes the short hangover just under twice the "
	     "bar");
	hangover_floor_line_create(&line, HANGOVER_SOFT_DBM0);
	hangover_verdict_run(&line, "frame under twice the bar",
			     HANGOVER_UNDER_TWICE_DBM0, &result);
	hangover_short_check(&result.verdict);
	CHECK_UINT(result.lasted, 3);
	CHECK_UINT(result.fired, 1);
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec double talk takes the long hangover just over twice the bar");
	hangover_floor_line_create(&line, HANGOVER_SOFT_DBM0);
	hangover_verdict_run(&line, "frame over twice the bar",
			     HANGOVER_OVER_TWICE_DBM0, &result);
	CHECK(result.verdict.dt_fired);
	CHECK_UINT(result.verdict.hold_left, 0);
	CHECK(result.verdict.mic_power >=
	      HANGOVER_MARGINAL_RATIO * result.verdict.bar);
	CHECK(result.verdict.mic_power <
	      HANGOVER_SOFT_RATIO * result.verdict.noise_power);
	CHECK_UINT(result.lasted, 23);
	CHECK_UINT(result.fired, 1);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * The edge of the short hangover at 12 dB over the floor, on the fixture of
 * hangover_floor_test. A louder far end raises the estimate, so each frame
 * stays within 3 dB of the bar. A frame about 10 dB over the floor takes the
 * short hangover, and one about 14 dB over it takes the long hangover. A soft
 * ratio of 8 or 32 fails one of the two.
 */
static void hangover_soft_edge_test(void)
{
	struct hangover_result result;
	struct hangover_line line;

	CASE("lec double talk takes the short hangover just under 12 dB over "
	     "the floor");
	hangover_floor_line_create(&line, HANGOVER_UNDER_SOFT_FAR_DBM0);
	hangover_verdict_run(&line, "frame under 12 dB over the floor",
			     HANGOVER_UNDER_SOFT_DBM0, &result);
	hangover_short_check(&result.verdict);
	CHECK_UINT(result.lasted, 3);
	CHECK_UINT(result.fired, 1);
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec double talk takes the long hangover just over 12 dB over "
	     "the floor");
	hangover_floor_line_create(&line, HANGOVER_OVER_SOFT_FAR_DBM0);
	hangover_verdict_run(&line, "frame over 12 dB over the floor",
			     HANGOVER_OVER_SOFT_DBM0, &result);
	CHECK(result.verdict.dt_fired);
	CHECK_UINT(result.verdict.hold_left, 0);
	CHECK(result.verdict.mic_power <
	      HANGOVER_MARGINAL_RATIO * result.verdict.bar);
	CHECK(result.verdict.noise_power > 0.0f);
	CHECK(result.verdict.mic_power >=
	      HANGOVER_SOFT_RATIO * result.verdict.noise_power);
	CHECK_UINT(result.lasted, 23);
	CHECK_UINT(result.fired, 1);
	udsp_ec_nlms.destroy(line.ec);
}

/* A fresh instance with the processor off, on a flat 8 ms hybrid. The filter
 * converges for 150 frames with the near-end floor on, and a far-end pause
 * measures the floor. */
static void hangover_talk_line_create(struct hangover_line *line,
				      double erl_db, double far_dbm0,
				      double floor_dbm0)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned f;

	memset(line, 0, sizeof(*line));
	line->state = 0x0badc0deu;
	line->near_state = 0xfeedfaceu;
	line->floor_state = 0x0f100f10u;
	line->far_dbm0 = far_dbm0;
	line->floor_dbm0 = floor_dbm0;
	line->floor = true;
	line->ec = udsp_ec_nlms.create(48);
	udsp_ec_nlms.nlp(line->ec, false);
	path_flat_init(&line->path, UDSP_SAMPLE_RATE, 8, erl_db);
	for (f = 0; f < FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2; f++)
		hangover_frame_run(line, NULL, &reading, &info);
	line->pause = true;
	for (f = 0; f < READING_PAUSE_FRAMES; f++)
		hangover_frame_run(line, NULL, &reading, &info);
	line->pause = false;
}

/* One talk frame at near_dbm0. It returns true when the filter adapted on
 * the frame. */
static bool hangover_talk_step(struct hangover_line *line, double near_dbm0,
			       struct udsp_ec_nlms_frame *reading,
			       struct udsp_ec_nlms_info *info)
{
	uint32_t held;

	udsp_ec_nlms_info_get(line->ec, info);
	held = info->hold_frames;
	hangover_near_run(line, near_dbm0, reading, info);
	return info->hold_frames == held;
}

#define HANGOVER_TALK_FLOOR_DBM0	-34.0
#define HANGOVER_TALK_DBM0		-19.0
#define HANGOVER_TALK_FRAMES		300u
#define HANGOVER_TALK_ADAPT_MAX		8u

/*
 * The talker of doubletalk_probe, 3 dB over the echo of a 12 dB hybrid. The
 * line's floor, 12 dB under the echo, keeps the ERLE under the trust line. Its
 * verdicts stand far over the floor, so the soft ratio gives the long hangover.
 * The case fails when more than 8 of the 300 talk frames adapt.
 */
static void hangover_talker_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info start, info;
	struct hangover_line line;
	unsigned first = HANGOVER_TALK_FRAMES;
	unsigned adapted = 0;
	unsigned early = 0;
	double lowest = 1e9;
	unsigned f;

	hangover_talk_line_create(&line, 12.0, -10.0,
				  HANGOVER_TALK_FLOOR_DBM0);
	udsp_ec_nlms_info_get(line.ec, &start);
	for (f = 0; f < HANGOVER_TALK_FRAMES; f++) {
		bool adapt = hangover_talk_step(&line, HANGOVER_TALK_DBM0,
						&reading, &info);
		double over;

		adapted += adapt;
		if (first == HANGOVER_TALK_FRAMES)
			early += adapt;
		if (!reading.dt_fired)
			continue;
		if (first == HANGOVER_TALK_FRAMES)
			first = f;
		over = ratio_db_get(reading.mic_power, reading.noise_power);
		if (over < lowest)
			lowest = over;
	}

	CASE("lec double talk keeps a talker 3 dB over the echo out of the "
	     "filter on a line with a floor");
	printf("# floor %.2f dBov, tracked %.2f dBov, %u of %u talk frames "
	       "adapted, %u before the first verdict on frame %u, lowest "
	       "verdict %.2f dB over the floor, probes %u, tracked erle "
	       "%.1f dB before the talk and %.1f dB after it\n",
	       floor_dbov_get(HANGOVER_TALK_FLOOR_DBM0),
	       (double)start.noise_dbov, adapted, HANGOVER_TALK_FRAMES, early,
	       first, lowest, info.probes - start.probes,
	       (double)start.erle_db, (double)info.erle_db);
	CHECK(fabs(start.noise_dbov -
		   floor_dbov_get(HANGOVER_TALK_FLOOR_DBM0)) <= 1.0);
	CHECK(adapted <= HANGOVER_TALK_ADAPT_MAX);
	udsp_ec_nlms.destroy(line.ec);
}

#define HANGOVER_HEADROOM_DBM0	-37.0

/* The upper bound of the floor headroom: on a tail frame with an estimate under
 * half the floor, the bar stands 4.5 dB over the floor. A near frame 5.5 dB
 * over it fires. A headroom of 3 puts the bar 6.3 dB over and fails. */
static void hangover_headroom_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct hangover_line line;

	CASE("lec double talk fires on a frame 5.5 dB over the floor");
	hangover_floor_line_train(&line, -10.0, READING_TRUST_TRAIN_FRAMES);
	line.pause = true;
	hangover_frame_run(&line, NULL, &reading, &info);
	hangover_near_run(&line, HANGOVER_HEADROOM_DBM0, &reading, &info);
	printf("# tail frame at %.1f dBm0: %.2f dB over the floor, estimate "
	       "%.2f dB under it, bar %.2f dB over it\n",
	       HANGOVER_HEADROOM_DBM0,
	       ratio_db_get(reading.mic_power, reading.noise_power),
	       ratio_db_get(reading.noise_power, reading.estimate_power),
	       ratio_db_get(reading.bar, reading.noise_power));
	CHECK(reading.ref_quiet);
	CHECK(reading.estimate_power * 2.0f < reading.noise_power);
	/* 4.0 is 6 dB: the frame stands under the bar of a headroom of 3. */
	CHECK(reading.mic_power < 4.0f * reading.noise_power);
	CHECK(reading.dt_fired);
	udsp_ec_nlms.destroy(line.ec);
}

/* A floor excursion on the fixture of hangover_floor_test, far-end frames,
 * and a second excursion on frame n after the first. */
static void hangover_repeat_run(unsigned n, struct hangover_result *result)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct hangover_line line;
	unsigned fired = 0;
	unsigned f;

	hangover_floor_line_create(&line, HANGOVER_SOFT_DBM0);
	hangover_near_run(&line, HANGOVER_EXCURSION_DBM0, &reading, &info);
	hangover_short_check(&reading);
	for (f = 1; f < n; f++) {
		hangover_frame_run(&line, NULL, &reading, &info);
		fired += reading.dt_fired;
	}
	printf("# %u frames between the excursions, %u verdicts among them\n",
	       n - 1u, fired);
	CHECK_UINT(fired, 0);
	hangover_verdict_run(&line, "second excursion",
			     HANGOVER_EXCURSION_DBM0, result);
	hangover_short_check(&result->verdict);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * The repeat limit: an excursion 24 frames after the first counts 23 frames
 * without a verdict, under the limit, and takes the long hangover. One 25
 * frames after takes the short one. A talker that fires every few frames meets
 * the long hangover.
 */
static void hangover_repeat_test(void)
{
	struct hangover_result result;

	CASE("lec double talk takes the long hangover on a verdict 24 frames "
	     "after the last");
	hangover_repeat_run(24u, &result);
	CHECK_UINT(result.lasted, 23);
	CHECK_UINT(result.fired, 1);

	CASE("lec double talk takes the short hangover on a verdict 25 frames "
	     "after the last");
	hangover_repeat_run(25u, &result);
	CHECK_UINT(result.lasted, 3);
	CHECK_UINT(result.fired, 1);
}

/* The start of the repeat count: a reset keeps the floor and puts the count
 * back at the limit, and a fresh instance starts there too. An excursion just
 * after either takes the short hangover. A count that starts at 0 fails. */
static void hangover_repeat_start_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct hangover_result result;
	struct hangover_line line;
	unsigned f;

	CASE("lec double talk takes the short hangover on the first excursion "
	     "after a reset");
	hangover_floor_line_create(&line, HANGOVER_SOFT_DBM0);
	hangover_near_run(&line, HANGOVER_EXCURSION_DBM0, &reading, &info);
	CHECK(reading.dt_fired);
	udsp_ec_nlms_reset(line.ec);
	hangover_verdict_run(&line, "excursion after the reset",
			     HANGOVER_EXCURSION_DBM0, &result);
	hangover_short_check(&result.verdict);
	CHECK_UINT(result.lasted, 3);
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec double talk takes the short hangover on the first excursion "
	     "of a fresh instance");
	memset(&line, 0, sizeof(line));
	line.state = 0x77665544u;
	line.near_state = 0xfeedfaceu;
	line.floor_state = 0x0f100f10u;
	line.far_dbm0 = HANGOVER_SOFT_DBM0;
	line.floor_dbm0 = READING_FLOOR_DBM0;
	line.floor = true;
	line.ec = udsp_ec_nlms.create(48);
	path_flat_init(&line.path, UDSP_SAMPLE_RATE, 8, 12.0);
	line.pause = true;
	for (f = 0; f < READING_PAUSE_FRAMES; f++)
		hangover_frame_run(&line, NULL, &reading, &info);
	line.pause = false;
	for (f = 0; f < 3u; f++)
		hangover_frame_run(&line, NULL, &reading, &info);
	hangover_verdict_run(&line, "first excursion", HANGOVER_EXCURSION_DBM0,
			     &result);
	CHECK_UINT(result.verdict.frames, 14);
	hangover_short_check(&result.verdict);
	CHECK_UINT(result.lasted, 3);
	udsp_ec_nlms.destroy(line.ec);
}

#define HANGOVER_SPARSE_FRAMES		3000u
/* 415 talk frames adapt on this seed; the bound adds 20 percent. */
#define HANGOVER_SPARSE_ADAPT_MAX	498u

/*
 * A talker who fires every fifth frame: a 24 dB hybrid and a far end at -20
 * dBm0 over a floor at -45 dBm0. Every fifth near frame stands at -36 dBm0 over
 * -50. The repeat limit gives its verdicts the long hangover, so few talk
 * frames adapt outside the trials of the latch.
 */
static void hangover_sparse_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info start, info;
	struct hangover_line line;
	unsigned adapted = 0;
	unsigned in_trial = 0;
	unsigned fired = 0;
	double erle;
	unsigned f;

	hangover_talk_line_create(&line, 24.0, -20.0, -45.0);
	udsp_ec_nlms_info_get(line.ec, &start);
	for (f = 0; f < HANGOVER_SPARSE_FRAMES; f++) {
		bool adapt = hangover_talk_step(&line,
						f % 5u ? -50.0 : -36.0,
						&reading, &info);

		adapted += adapt;
		in_trial += adapt && reading.probing;
		fired += reading.dt_fired;
	}
	udsp_ec_nlms.freeze(line.ec, true);
	erle = erle_run(line.ec, &line.path, &line.state, -20.0, 20u, 20u);

	CASE("lec repeat limit keeps a talker who fires every fifth frame out "
	     "of the filter");
	printf("# %u of %u talk frames adapted, %u of them in %u trials, %u "
	       "verdicts, resets %u, true erle %.1f dB after the talk\n",
	       adapted, HANGOVER_SPARSE_FRAMES, in_trial,
	       info.probes - start.probes, fired,
	       info.diverge_resets - start.diverge_resets, erle);
	CHECK(adapted <= HANGOVER_SPARSE_ADAPT_MAX);
	udsp_ec_nlms.destroy(line.ec);
}

/* The call of the replay cases: far noise bursts through the composite hybrid
 * over a near floor, and near bursts that fire the detector. The first instance
 * adds a reset, a freeze, the processor off, FEED records and a skip. */
#define REPLAY_PERIOD		70u
#define REPLAY_FAR_ON		40u
#define REPLAY_FAR_DBM0		-10.0
#define REPLAY_NEAR_DBM0	-15.0
#define REPLAY_FLOOR_DBM0	-55.0
#define REPLAY_ERL_DB		12.0
#define REPLAY_NEAR_FRAMES	6u
#define REPLAY_FIRST_FRAMES	1000u
#define REPLAY_SECOND_FRAMES	200u
#define REPLAY_SECOND_FEEDS	5u
#define REPLAY_RESET_AT		500u
#define REPLAY_FREEZE_FROM	380u
#define REPLAY_NLP_OFF_FROM	700u
#define REPLAY_STRETCH		50u
#define REPLAY_FEED_FROM	800u
#define REPLAY_FEED_FRAMES	20u
#define REPLAY_TICK_GAP		500u
#define REPLAY_LOSS_AT		600u
#define REPLAY_SKIP_AT		650u
#define REPLAY_SKIP_TICKS	3u
#define REPLAY_SEGMENTS		3u
#define REPLAY_RECORDS		(REPLAY_FIRST_FRAMES + REPLAY_SECOND_FRAMES + 2u)
#define REPLAY_TICK_NS		10000000ull

struct replay_call {
	unsigned rate;
	struct echo_path path;
	uint32_t far_state;
	uint32_t floor_state;
	uint32_t near_state;
	struct udsp_ec *ec;
	bool nlp;
	bool failed;
	uint16_t gen;
	uint64_t tick;
	uint16_t skip_next;
	struct lecr_record *recs;
	unsigned count;
	struct lecr_tally seg_info[REPLAY_SEGMENTS];
	unsigned segments;
	unsigned runs;
	unsigned feeds;
	unsigned off_frames;
	unsigned freeze_frames;
	unsigned fired;
};

static bool replay_talk_get(unsigned g)
{
	static const unsigned from[] = { 300u, 630u, 910u, 1140u };
	unsigned i;

	for (i = 0; i < sizeof(from) / sizeof(from[0]); i++)
		if (g >= from[i] && g < from[i] + REPLAY_NEAR_FRAMES)
			return true;
	return false;
}

static void replay_frame_make(struct replay_call *c, unsigned g,
			      struct lecr_record *rec)
{
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	unsigned n = c->path.frame;
	bool talk = replay_talk_get(g);
	unsigned i;

	if (g % REPLAY_PERIOD < REPLAY_FAR_ON)
		noise_fill(rec->ref, n, REPLAY_FAR_DBM0, &c->far_state);
	path_run(&c->path, rec->ref, echo);
	noise_fill(hiss, n, REPLAY_FLOOR_DBM0, &c->floor_state);
	if (talk)
		noise_fill(near, n, REPLAY_NEAR_DBM0, &c->near_state);
	for (i = 0; i < n; i++)
		rec->mic[i] = sample_clamp((double)echo[i] + hiss[i] +
					   (talk ? near[i] : 0));
}

/* The counters of the instance at the end of a segment: before a reset and
 * before the instance goes. */
static void replay_segment_end(struct replay_call *c)
{
	struct udsp_ec_nlms_info info;

	udsp_ec_nlms_info_get(c->ec, &info);
	if (c->segments < REPLAY_SEGMENTS)
		lecr_tally_set(&c->seg_info[c->segments], &info);
	c->segments++;
}

/* One tick of the line in the order of chan_lec_run of core/chan.c: the
 * feed on a skip, else the processor switch, the reset, the freeze and the
 * run. feed is the SKIP flag of a FEED, or 0 for a RUN. */
static void replay_tick_run(struct replay_call *c, unsigned g, bool fresh,
			    uint16_t feed, bool reset, bool freeze, bool nlp)
{
	struct lecr_record *rec;

	if (c->failed)
		return;
	rec = &c->recs[c->count++];
	memset(rec, 0, sizeof(*rec));
	replay_frame_make(c, g, rec);
	rec->samples = (uint16_t)c->path.frame;
	rec->tick = c->tick++;
	rec->stamp_ns = rec->tick * REPLAY_TICK_NS;
	if (c->skip_next) {
		rec->flags |= LECR_FLAG_SKIPPED;
		rec->skipped = c->skip_next;
		c->skip_next = 0;
	}
	if (fresh) {
		c->ec = udsp_ec_nlms.create_rate(0, c->rate);
		c->nlp = true;
		c->gen++;
		rec->flags |= LECR_FLAG_NEW;
		if (!c->ec) {
			c->failed = true;
			return;
		}
	}
	if (feed) {
		udsp_ec_nlms_ref_feed(c->ec, rec->ref);
		rec->kind = LECR_KIND_FEED;
		rec->flags |= feed;
		c->feeds++;
	} else {
		if (c->nlp != nlp) {
			c->nlp = nlp;
			udsp_ec_nlms.nlp(c->ec, nlp);
		}
		if (reset) {
			replay_segment_end(c);
			udsp_ec_nlms_reset(c->ec);
			c->gen++;
			rec->flags |= LECR_FLAG_RESET;
		}
		udsp_ec_nlms.freeze(c->ec, freeze);
		memcpy(rec->out, rec->mic, rec->samples * sizeof(*rec->out));
		udsp_ec_nlms.run(c->ec, rec->out, rec->ref);
		rec->kind = LECR_KIND_RUN;
		rec->flags |= (nlp ? LECR_FLAG_NLP : 0u) |
			      (freeze ? LECR_FLAG_FREEZE : 0u);
		c->runs++;
		c->off_frames += !nlp;
		c->freeze_frames += freeze;
	}
	rec->gen = c->gen;
	lecr_record_fill(rec, c->ec);
	c->fired += (rec->flags & LECR_FLAG_FIRED) != 0;
}

static void replay_end_push(struct replay_call *c)
{
	struct lecr_record *rec;

	if (c->failed)
		return;
	replay_segment_end(c);
	rec = &c->recs[c->count++];
	memset(rec, 0, sizeof(*rec));
	rec->kind = LECR_KIND_END;
	rec->samples = (uint16_t)c->path.frame;
	rec->tick = c->tick;
	rec->stamp_ns = rec->tick * REPLAY_TICK_NS;
	rec->gen = c->gen;
	lecr_record_fill(rec, c->ec);
	udsp_ec_nlms.destroy(c->ec);
	c->ec = NULL;
}

static bool replay_stretch_get(unsigned f, unsigned from, unsigned frames)
{
	return f >= from && f < from + frames;
}

/* The line of a synthetic call, with room for records records. */
static bool replay_call_init(struct replay_call *c, unsigned rate,
			     unsigned records)
{
	memset(c, 0, sizeof(*c));
	c->recs = calloc(records, sizeof(*c->recs));
	if (!c->recs)
		return false;
	c->rate = rate;
	c->far_state = 0x13579bdfu;
	c->floor_state = 0x2468ace0u;
	c->near_state = 0x0badcafeu;
	path_composite_init(&c->path, rate, REPLAY_ERL_DB);
	return true;
}

static bool replay_call_build(struct replay_call *c, unsigned rate)
{
	unsigned f;

	if (!replay_call_init(c, rate, REPLAY_RECORDS))
		return false;
	for (f = 0; f < REPLAY_FIRST_FRAMES; f++) {
		if (f == REPLAY_SKIP_AT) {
			c->tick += REPLAY_SKIP_TICKS;
			c->skip_next = REPLAY_SKIP_TICKS;
		}
		replay_tick_run(c, f, f == 0u,
				replay_stretch_get(f, REPLAY_FEED_FROM,
						   REPLAY_FEED_FRAMES) ?
				LECR_FLAG_SKIP_OPEN : 0u,
				f == REPLAY_RESET_AT,
				replay_stretch_get(f, REPLAY_FREEZE_FROM,
						   REPLAY_STRETCH),
				!replay_stretch_get(f, REPLAY_NLP_OFF_FROM,
						    REPLAY_STRETCH));
	}
	replay_end_push(c);
	c->tick += REPLAY_TICK_GAP;
	for (f = 0; f < REPLAY_SECOND_FRAMES; f++)
		replay_tick_run(c, REPLAY_FIRST_FRAMES + f, f == 0u,
				f < REPLAY_SECOND_FEEDS ?
				LECR_FLAG_SKIP_INACTIVE : 0u, false, false,
				true);
	replay_end_push(c);
	return !c->failed;
}

/* The records through the writer into memory. The caller frees *buf. */
static bool replay_file_write(const struct lecr_record *recs, unsigned count,
			      unsigned rate, char **buf, size_t *size)
{
	struct lecr_file file;
	FILE *f = open_memstream(buf, size);
	bool ok;
	unsigned i;

	if (!f)
		return false;
	ok = !lecr_write_open(f, &file, 0, rate);
	for (i = 0; ok && i < count; i++)
		ok = !lecr_write_record(f, &file, &recs[i]);
	ok = ok && !lecr_write_close(f, &file);
	if (fclose(f))
		ok = false;
	return ok;
}

/* What the replay of a file saw. twin_frames counts the RUN frames on which
 * neither this frame nor the one before read ACTIVE, where the shadow output
 * must equal the replay output. */
struct replay_seen {
	struct lecr_segment seg[REPLAY_SEGMENTS];
	unsigned segments;
	unsigned runs;
	unsigned feeds;
	unsigned diffs;
	unsigned twin_frames;
	unsigned twin_same;
	bool active_before;
};

static void replay_seen_add(void *ctx, const struct lecr_record *rec,
			    const struct lecr_replay *r,
			    const struct lecr_step *st,
			    const struct lecr_agree *a, bool closed)
{
	struct replay_seen *seen = ctx;
	bool active;

	if (closed && seen->segments < REPLAY_SEGMENTS)
		seen->seg[seen->segments] = a->done;
	seen->segments += closed;
	if (!rec || !st->replayed)
		return;
	if (rec->kind == LECR_KIND_FEED) {
		seen->feeds++;
		return;
	}
	seen->runs++;
	seen->diffs += st->diff != 0u;
	active = (rec->flags & LECR_FLAG_ACTIVE) != 0;
	if (r->shadow && !active && !seen->active_before && !st->fresh &&
	    !(rec->flags & LECR_FLAG_RESET)) {
		seen->twin_frames++;
		seen->twin_same += !memcmp(r->out, r->residual,
					   rec->samples * sizeof(*r->out));
	}
	seen->active_before = active;
}

/* The steady-state figures leave out the first skip_s of each segment. */
static int replay_buffer_skip_run(char *buf, size_t size, bool shadow,
				  unsigned skip_s, struct lecr_agree *a,
				  struct replay_seen *seen)
{
	struct lecr_replay r;
	struct lecr_file file;
	FILE *f = fmemopen(buf, size, "rb");
	int err;

	memset(seen, 0, sizeof(*seen));
	lecr_agree_init(a, skip_s);
	if (!f)
		return -ENOMEM;
	lecr_replay_init(&r, shadow);
	err = lecr_stream_replay(f, &file, &r, a, replay_seen_add, seen);
	fclose(f);
	return err;
}

static int replay_buffer_run(char *buf, size_t size, bool shadow,
			     struct lecr_agree *a, struct replay_seen *seen)
{
	return replay_buffer_skip_run(buf, size, shadow, 0, a, seen);
}

/* A copy of the records of a call that the caller edited, through the writer
 * and the replay. The answer is false when the writer fails. */
static bool replay_recs_run(const struct replay_call *c,
			    const struct lecr_record *recs, unsigned skip_s,
			    struct lecr_agree *a, struct replay_seen *seen)
{
	char *buf = NULL;
	size_t size = 0;
	bool ok = replay_file_write(recs, c->count, c->rate, &buf, &size);

	if (ok)
		replay_buffer_skip_run(buf, size, false, skip_s, a, seen);
	free(buf);
	return ok;
}

/* The call without record drop, and with lost_before set on the record
 * that follows it, through the writer and the replay. */
static bool replay_drop_run(const struct replay_call *c, unsigned drop,
			    uint32_t lost, struct lecr_agree *a,
			    struct replay_seen *seen)
{
	struct lecr_record *recs = malloc(c->count * sizeof(*recs));
	char *buf = NULL;
	size_t size = 0;
	bool ok;

	if (!recs)
		return false;
	memcpy(recs, c->recs, drop * sizeof(*recs));
	memcpy(recs + drop, c->recs + drop + 1u,
	       (c->count - drop - 1u) * sizeof(*recs));
	recs[drop].lost_before = lost;
	ok = replay_file_write(recs, c->count - 1u, c->rate, &buf, &size) &&
	     !replay_buffer_run(buf, size, false, a, seen);
	free(buf);
	free(recs);
	return ok;
}

/* UDSP_TEST_LECR_OUT=<prefix> keeps a copy of the file of each rate, as
 * <prefix>-8k.lecr and <prefix>-16k.lecr, for a run of lec_replay. */
static void replay_file_keep(const char *buf, size_t size, unsigned rate)
{
	const char *prefix = getenv("UDSP_TEST_LECR_OUT");
	char path[512];
	FILE *f;

	if (!prefix)
		return;
	snprintf(path, sizeof(path), "%s-%uk.lecr", prefix, rate / 1000u);
	f = fopen(path, "wb");
	CHECK(f != NULL);
	if (!f)
		return;
	CHECK_UINT(fwrite(buf, 1, size, f), size);
	CHECK(!fclose(f));
	printf("# kept %s\n", path);
}

/* The raw bytes of a record against the offsets of the lec_replay.h table:
 * frames at 20, flags at 46, kind at 59, the verdict at 60 and the planes
 * from 64, all little-endian. */
static void replay_bytes_check(const uint8_t *b, const struct lecr_record *rec)
{
	unsigned n = rec->samples;

	CHECK_UINT(b[20] | b[21] << 8 | b[22] << 16 | (uint32_t)b[23] << 24,
		   rec->frames);
	CHECK_UINT(b[46] | b[47] << 8, rec->flags);
	CHECK_UINT(b[48] | b[49] << 8, n);
	CHECK_UINT(b[59], rec->kind);
	CHECK_UINT(b[60], rec->nlp_verdict);
	CHECK_UINT(b[62] | b[63], 0);
	CHECK_UINT((uint16_t)(b[64] | b[65] << 8), (uint16_t)rec->ref[0]);
	CHECK_UINT((uint16_t)(b[64 + 2 * n] | b[65 + 2 * n] << 8),
		   (uint16_t)rec->mic[0]);
	CHECK_UINT((uint16_t)(b[64 + 4 * n] | b[65 + 4 * n] << 8),
		   (uint16_t)rec->out[0]);
}

/*
 * The file of the synthetic call through the writer and the reader. The header
 * carries the magic, version 2, rate, samples, count and NEW at the offsets of
 * the table, and every record reads back equal. A writer that swaps two planes
 * or writes big-endian fails the byte checks.
 */
static void replay_file_check(const struct replay_call *c, char *buf,
			      size_t size)
{
	unsigned n = c->path.frame;
	size_t record = LECR_HEAD_BYTES + 6u * n;
	unsigned probe = REPLAY_FEED_FROM - 1u;
	struct lecr_record rec;
	struct lecr_file file;
	unsigned same = 0;
	unsigned i;
	FILE *f;

	CHECK_UINT(size, LECR_FILE_BYTES + c->count * record);
	if (size != LECR_FILE_BYTES + c->count * record)
		return;
	CHECK(!memcmp(buf, "UDSPLECR", 8));
	CHECK_UINT(buf[8] | buf[9] << 8, 2);
	CHECK_UINT((uint8_t)buf[12] | (uint8_t)buf[13] << 8, c->rate);
	CHECK_UINT((uint8_t)buf[16], n);
	CHECK_UINT((uint8_t)buf[20] | (uint8_t)buf[21] << 8, c->count);
	CHECK_UINT(buf[24], LECR_FILE_FIRST_NEW);
	replay_bytes_check((const uint8_t *)buf + LECR_FILE_BYTES +
			   probe * record, &c->recs[probe]);
	f = fmemopen(buf, size, "rb");
	CHECK(f != NULL);
	if (!f)
		return;
	if (lecr_read_open(f, &file)) {
		CHECK(!"the reader refuses the file header");
		fclose(f);
		return;
	}
	CHECK_UINT(file.rate, c->rate);
	CHECK_UINT(file.samples, n);
	CHECK_UINT(file.records, c->count);
	for (i = 0; i < c->count && lecr_read_record(f, &file, &rec) == 1; i++)
		same += !memcmp(&rec, &c->recs[i], sizeof(rec));
	CHECK_UINT(same, c->count);
	CHECK(!lecr_read_record(f, &file, &rec));
	fclose(f);
}

/*
 * The replay of a synthetic call (bench/lec_replay.h): each RUN replays bit for
 * bit, and the shadow output matches where the processor did not act. Each
 * segment tally equals the live counters. A missing first NEW reads INCOMPLETE;
 * a frames mismatch or an unexplained jump reads FAIL.
 */
static void replay_call_test(unsigned rate)
{
	struct replay_seen seen;
	struct replay_call call;
	struct lecr_agree a;
	char *buf = NULL;
	size_t size = 0;
	unsigned total[LECR_COUNTS] = { 0 };
	unsigned probe;
	unsigned i, k;
	bool built = replay_call_build(&call, rate);
	bool written = built && replay_file_write(call.recs, call.count, rate,
						  &buf, &size);

	case_rate(rate,
		  "lec replay file keeps every field of a synthetic call");
	CHECK(built);
	CHECK(written);
	CHECK_UINT(call.count, REPLAY_RECORDS);
	if (!written) {
		free(buf);
		free(call.recs);
		return;
	}
	replay_file_check(&call, buf, size);
	replay_file_keep(buf, size, rate);

	case_rate(rate, "lec replay reproduces a synthetic call bit for bit");
	CHECK(!replay_buffer_run(buf, size, true, &a, &seen));
	printf("# %u runs, %u feeds replayed, %u differ; %u twin frames, %u "
	       "equal\n", seen.runs, seen.feeds, seen.diffs, seen.twin_frames,
	       seen.twin_same);
	CHECK_UINT(seen.runs, call.runs);
	CHECK_UINT(seen.feeds, call.feeds);
	CHECK_UINT(seen.diffs, 0);
	CHECK(seen.twin_frames > 0u);
	CHECK_UINT(seen.twin_same, seen.twin_frames);
	CHECK_UINT(a.instances, 2);
	CHECK_UINT(a.resets, 1);
	CHECK_UINT(a.frames_bad, 0);
	CHECK_UINT(a.evaluated, REPLAY_SEGMENTS);
	/* The skip of the engine is a jump with SKIPPED, not a gap. */
	CHECK_UINT(a.skip_marks, 1);
	CHECK_UINT(a.skip_periods, REPLAY_SKIP_TICKS);
	CHECK_UINT(a.gaps, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);

	case_rate(rate, "lec replay tally equals the canceller counters of "
		  "each segment");
	CHECK_UINT(seen.segments, REPLAY_SEGMENTS);
	CHECK_UINT(call.segments, REPLAY_SEGMENTS);
	CHECK_UINT(a.losses + a.gaps + a.skipped, 0);
	for (i = 0; i < REPLAY_SEGMENTS && i < seen.segments; i++) {
		for (k = 0; k < LECR_COUNTS; k++) {
			CHECK_UINT(seen.seg[i].live.n[k],
				   call.seg_info[i].n[k]);
			CHECK_UINT(seen.seg[i].replay.n[k],
				   call.seg_info[i].n[k]);
			total[k] += seen.seg[i].live.n[k];
		}
	}
	printf("# active %u wait %u erle %u unlike %u dt %u quiet %u; fired "
	       "%u, off %u, frozen %u, feeds %u\n", total[LECR_COUNT_ACTIVE],
	       total[LECR_COUNT_WAIT], total[LECR_COUNT_ERLE],
	       total[LECR_COUNT_UNLIKE], total[LECR_COUNT_DT],
	       total[LECR_COUNT_QUIET], call.fired, call.off_frames,
	       call.freeze_frames, call.feeds);
	CHECK(total[LECR_COUNT_ACTIVE] > 0u);
	CHECK(total[LECR_COUNT_WAIT] > 0u);
	CHECK(total[LECR_COUNT_DT] > 0u);
	CHECK(total[LECR_COUNT_QUIET] > 0u);
	CHECK(call.fired > 0u);
	CHECK_UINT(call.off_frames, REPLAY_STRETCH);
	CHECK_UINT(call.freeze_frames, REPLAY_STRETCH);
	free(buf);
	buf = NULL;

	case_rate(rate, "lec replay reads a capture without its first NEW as "
		  "incomplete");
	call.recs[0].flags &= (uint16_t)~LECR_FLAG_NEW;
	CHECK(replay_file_write(call.recs, call.count, rate, &buf, &size));
	CHECK(buf && !replay_buffer_run(buf, size, false, &a, &seen));
	CHECK_UINT(a.evaluated, 1);
	CHECK_UINT(a.skipped, REPLAY_FIRST_FRAMES);
	CHECK_UINT(lecr_agree_result(&a), LECR_INCOMPLETE);
	call.recs[0].flags |= LECR_FLAG_NEW;
	free(buf);
	buf = NULL;

	case_rate(rate, "lec replay reads a frames mismatch as a failure");
	probe = REPLAY_RESET_AT / 2u;
	call.recs[probe].frames++;
	CHECK(replay_file_write(call.recs, call.count, rate, &buf, &size));
	CHECK(buf && !replay_buffer_run(buf, size, false, &a, &seen));
	CHECK_UINT(a.frames_bad, 1);
	CHECK_UINT(a.failed, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	call.recs[probe].frames--;
	free(buf);

	case_rate(rate, "lec replay drops a segment after a loss and fails a "
		  "tick gap");
	CHECK(replay_drop_run(&call, REPLAY_LOSS_AT, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(a.losses, 0);
	CHECK_UINT(a.evaluated, REPLAY_SEGMENTS - 1u);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	CHECK(replay_drop_run(&call, REPLAY_LOSS_AT, 1, &a, &seen));
	CHECK_UINT(a.gaps, 0);
	CHECK_UINT(a.losses, 1);
	CHECK_UINT(a.lost, 1);
	CHECK_UINT(a.skipped, REPLAY_FIRST_FRAMES - 1u - REPLAY_LOSS_AT);
	CHECK_UINT(seen.segments, REPLAY_SEGMENTS);
	CHECK(seen.seg[1].lossy);
	CHECK_UINT(a.evaluated, REPLAY_SEGMENTS - 1u);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);
	CHECK(replay_drop_run(&call, REPLAY_FIRST_FRAMES, 1, &a, &seen));
	CHECK_UINT(a.gaps, 0);
	CHECK_UINT(a.losses, 1);
	CHECK_UINT(a.instances, 2);
	CHECK_UINT(seen.segments, REPLAY_SEGMENTS);
	CHECK(seen.seg[1].lossy);
	CHECK_UINT(seen.diffs, 0);
	CHECK_UINT(a.evaluated, REPLAY_SEGMENTS - 1u);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);

	case_rate(rate, "lec replay reads a skip that does not match its jump "
		  "as a gap, and a skip beside a loss as a loss");
	/* The record before the skip is lost from the archive, so the jump is
	 * one tick longer than the count explains. */
	CHECK(replay_drop_run(&call, REPLAY_SKIP_AT - 1u, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(a.skip_marks, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	CHECK(replay_drop_run(&call, REPLAY_SKIP_AT - 1u, 1, &a, &seen));
	CHECK_UINT(a.gaps, 0);
	CHECK_UINT(a.losses, 1);
	CHECK_UINT(a.skip_marks, 0);
	CHECK_UINT(a.evaluated, REPLAY_SEGMENTS - 1u);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);
	buf = NULL;
	call.recs[REPLAY_SKIP_AT].skipped--;
	CHECK(replay_file_write(call.recs, call.count, rate, &buf, &size));
	CHECK(buf && !replay_buffer_run(buf, size, false, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(a.skip_marks, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	call.recs[REPLAY_SKIP_AT].skipped++;
	free(buf);
	free(call.recs);
}

/* Sets the live record and the replay record of frame f of a driven
 * segment. */
typedef void (*replay_frame_edit)(unsigned f, struct lecr_record *rec,
				  struct lecr_record *got, const void *arg);

/* One segment of frames RUN records from a NEW, through the agreement alone
 * and without a canceller. The steady-state figures leave out skip_s. */
static void replay_seg_drive(struct lecr_agree *a, unsigned skip_s,
			     unsigned frames, replay_frame_edit edit,
			     const void *arg)
{
	struct lecr_record rec, got;
	struct lecr_step st;
	unsigned f;

	lecr_agree_init(a, skip_s);
	for (f = 0; f < frames; f++) {
		memset(&rec, 0, sizeof(rec));
		rec.kind = LECR_KIND_RUN;
		rec.samples = UDSP_FRAME_SAMPLES;
		rec.tick = f;
		rec.frames = f + 1u;
		rec.flags = f ? 0u : LECR_FLAG_NEW;
		got = rec;
		edit(f, &rec, &got, arg);
		memset(&st, 0, sizeof(st));
		st.fresh = f == 0u;
		st.replayed = true;
		st.diff = lecr_diff_get(&rec, &got, got.out, rec.samples,
					&st.out_delta);
		lecr_agree_add(a, &rec, &st, &got);
	}
	lecr_agree_finish(a);
}

static void replay_flip_edit(unsigned f, struct lecr_record *rec,
			     struct lecr_record *got, const void *arg)
{
	unsigned flipped = *(const unsigned *)arg;

	rec->nlp_verdict = f % 2u ? LECR_VERDICT_QUIET : LECR_VERDICT_ERLE;
	got->nlp_verdict = rec->nlp_verdict;
	if (f < flipped)
		got->nlp_verdict = f % 2u ? LECR_VERDICT_ERLE :
					    LECR_VERDICT_QUIET;
}

/* 1000 RUN frames that alternate two verdicts. The replay swaps the
 * verdicts of the first flipped frames in pairs, so the six counts and the
 * fired frames agree and only the verdict agreement can fail. */
static void replay_flip_run(unsigned flipped, struct lecr_agree *a)
{
	replay_seg_drive(a, 0, 1000u, replay_flip_edit, &flipped);
}

/*
 * The verdict agreement of lec_replay.h needs 95 percent or more of the
 * frames of a segment. 60 swapped frames in 1000 fail on the agreement
 * alone. 40 and 50 swapped frames pass. An agreement bar that is not
 * checked, or checked above 95 percent, fails here.
 */
static void replay_agree_test(void)
{
	struct lecr_agree a;
	unsigned i;

	CASE("lec replay agreement flags 6 percent of flipped verdicts");
	replay_flip_run(60u, &a);
	for (i = 0; i < LECR_COUNTS; i++)
		CHECK_UINT(a.done.live.n[i], a.done.replay.n[i]);
	CHECK_UINT(a.done.verdict_same, 940);
	CHECK_UINT(lecr_segment_faults(&a.done), LECR_FAULT_VERDICT);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);

	CASE("lec replay agreement passes 4 percent of flipped verdicts");
	replay_flip_run(40u, &a);
	CHECK_UINT(a.done.verdict_same, 960);
	CHECK_UINT(lecr_segment_faults(&a.done), 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);
	replay_flip_run(50u, &a);
	CHECK_UINT(a.done.verdict_same, 950);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);
}

/*
 * The verdict of the capture record has a stable order, and dsp/lec.c
 * writes its private order into the frame reading. The words of
 * udsp_ec_nlms_verdict_name for 0 to 5 must be the words of that stable
 * order. A reordered private enum fails here.
 */
static void replay_verdict_word_test(void)
{
	static const char *const words[] = {
		"off", "quiet", "dt", "unlike", "erle", "pass",
	};
	unsigned i;

	CASE("lec replay verdict order gives the words of the canceller");
	for (i = 0; i < sizeof(words) / sizeof(words[0]); i++)
		CHECK(!strcmp(udsp_ec_nlms_verdict_name(i), words[i]));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(LECR_VERDICT_OFF), "off"));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(LECR_VERDICT_QUIET), "quiet"));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(LECR_VERDICT_DT), "dt"));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(LECR_VERDICT_UNLIKE),
		      "unlike"));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(LECR_VERDICT_ERLE), "erle"));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(LECR_VERDICT_PASS), "pass"));
	CHECK(!strcmp(udsp_ec_nlms_verdict_name(6), "?"));
}

/* Sample k of record i, as fixture_records() of src/tools/lec-capture.py
 * writes it: ref 100 x i + k, mic i, and out -(100 x i + k) on a RUN. A
 * FEED has zeros in out, and an END has zeros in the three planes. */
static bool capture_fixture_planes_ok(const struct lecr_record *rec,
				      unsigned i)
{
	bool run = rec->kind == LECR_KIND_RUN;
	bool end = rec->kind == LECR_KIND_END;
	int base = (int)(100u * i);
	unsigned k;

	for (k = 0; k < 80u; k++) {
		if (rec->ref[k] != (end ? 0 : base + (int)k) ||
		    rec->mic[k] != (end ? 0 : (int)i) ||
		    rec->out[k] != (run ? -(base + (int)k) : 0))
			return false;
	}
	return true;
}

/*
 * The replay file of the ctest fixture test_lec_capture_fixture, which
 * lec-capture.py writes and UDSP_LEC_CAPTURE_FIXTURE names (unset: skip). The
 * literals here and the docstring of fixture_records() state the same values. A
 * writer that swaps two planes, moves a field or writes big-endian fails.
 */
static void capture_fixture_test(void)
{
	static const uint8_t kinds[] = {
		LECR_KIND_RUN, LECR_KIND_RUN, LECR_KIND_FEED,
		LECR_KIND_RUN, LECR_KIND_RUN, LECR_KIND_END,
	};
	static const uint16_t flags[] = {
		0x0009, 0x0048, 0x0100, 0x000e, 0x10b8, 0x0000,
	};
	static const uint32_t frames[] = { 1, 2, 2, 1, 2, 2 };
	static const uint8_t verdicts[] = { 5, 5, 0, 1, 2, 0 };
	const char *path = getenv("UDSP_LEC_CAPTURE_FIXTURE");
	struct lecr_record rec;
	struct lecr_file file;
	unsigned i;
	FILE *f;

	CASE("lec capture fixture reads back the documented values");
	if (!path) {
		tu_skip("UDSP_LEC_CAPTURE_FIXTURE not set");
		return;
	}
	f = fopen(path, "rb");
	CHECK(f != NULL);
	if (!f)
		return;
	if (lecr_read_open(f, &file)) {
		CHECK(!"the reader refuses the fixture header");
		fclose(f);
		return;
	}
	CHECK_UINT(file.line, 1);
	CHECK_UINT(file.rate, UDSP_SAMPLE_RATE);
	CHECK_UINT(file.samples, 80);
	CHECK_UINT(file.records, 6);
	CHECK_UINT(file.flags, LECR_FILE_FIRST_NEW | LECR_FILE_REF_LOSS);
	CHECK_UINT(file.ref_loss_half_db, 18);
	CHECK_UINT(lecr_file_ref_loss_get(&file), 18);
	for (i = 0; i < 6u && lecr_read_record(f, &file, &rec) == 1; i++) {
		CHECK_UINT(rec.stamp_ns, 1000000000ull + 10000000ull * i);
		CHECK_UINT(rec.tick, 1000u + i);
		CHECK_UINT(rec.lost_before, i == 4u ? 7 : 0);
		CHECK_UINT(rec.frames, frames[i]);
		CHECK_UINT(rec.hold_frames, i);
		CHECK(rec.erle_mdb == -1500 * (int32_t)i);
		CHECK_UINT(rec.echo_gain_micro, 250000);
		CHECK_UINT(rec.coef_energy_micro, 1000u * i);
		CHECK_UINT(rec.ref_delay_samples, 80u + i);
		CHECK_UINT(rec.gen, 7);
		CHECK_UINT(rec.flags, flags[i]);
		CHECK_UINT(rec.samples, 80);
		CHECK_UINT(rec.tail_ms, 48);
		CHECK_UINT(rec.noise_ddbov, i ? 653u : LECR_NOISE_UNMEASURED);
		CHECK_UINT(rec.probes, i);
		CHECK_UINT(rec.diverge_resets, 0);
		CHECK_UINT(rec.line, 1);
		CHECK_UINT(rec.kind, kinds[i]);
		CHECK_UINT(rec.nlp_verdict, verdicts[i]);
		CHECK_UINT(rec.hold_left, i == 4u ? 3 : 0);
		CHECK_UINT(rec.skipped, i == 4u ? 5 : 0);
		CHECK(capture_fixture_planes_ok(&rec, i));
	}
	CHECK_UINT(i, 6);
	CHECK(!lecr_read_record(f, &file, &rec));
	fclose(f);
}

/*
 * The verdict of udsp.h struct udsp_lec_capture is a stable enum, and the
 * engine copies the private value of dsp/lec.c into it unchanged. Each public
 * value must therefore name the verdict of its word. A reordered public or
 * private enum fails here.
 */
static void capture_verdict_word_test(void)
{
	static const struct {
		unsigned verdict;
		const char *word;
	} map[] = {
		{ UDSP_LEC_CAPTURE_VERDICT_OFF, "off" },
		{ UDSP_LEC_CAPTURE_VERDICT_QUIET, "quiet" },
		{ UDSP_LEC_CAPTURE_VERDICT_DT, "dt" },
		{ UDSP_LEC_CAPTURE_VERDICT_UNLIKE, "unlike" },
		{ UDSP_LEC_CAPTURE_VERDICT_ERLE, "erle" },
		{ UDSP_LEC_CAPTURE_VERDICT_PASS, "pass" },
	};
	unsigned i;

	CASE("lec capture verdict values give the words of the canceller");
	for (i = 0; i < sizeof(map) / sizeof(map[0]); i++)
		CHECK(!strcmp(udsp_ec_nlms_verdict_name(map[i].verdict),
			      map[i].word));
}

/* Three records of the synthetic call, by their index: a FEED of the open-loop
 * stretch, the second instance's NEW record and the last RUN before its END. */
#define REPLAY_FEED_PROBE	(REPLAY_FEED_FROM + 5u)
#define REPLAY_SECOND_NEW	(REPLAY_FIRST_FRAMES + 1u)
#define REPLAY_SECOND_LAST	(REPLAY_RECORDS - 2u)

/*
 * One edit of the synthetic call for each rule of one record in the header
 * comment of lec_replay.h. samples 0 keeps the samples of the record, and
 * line is added to its line.
 */
static const struct replay_refusal {
	unsigned index;
	uint16_t set;
	uint16_t clear;
	uint16_t skipped;
	uint8_t line;
	uint16_t samples;
} replay_refusals[] = {
	{ REPLAY_FEED_PROBE, 0, LECR_FLAG_SKIP_OPEN, 0, 0, 0 },
	{ REPLAY_FEED_PROBE, LECR_FLAG_SKIP_MODEM, 0, 0, 0, 0 },
	{ REPLAY_FEED_FROM - 1u, LECR_FLAG_SKIP_OFF, 0, 0, 0, 0 },
	{ REPLAY_FIRST_FRAMES, LECR_FLAG_SKIP_INACTIVE, 0, 0, 0, 0 },
	{ REPLAY_FEED_PROBE, 0, 0, 7, 0, 0 },
	{ REPLAY_FEED_PROBE, LECR_FLAG_SKIPPED, 0, 0, 0, 0 },
	{ 0, LECR_FLAG_SKIPPED, 0, 2, 0, 0 },
	{ REPLAY_FIRST_FRAMES, LECR_FLAG_SKIPPED, 0, 2, 0, 0 },
	{ REPLAY_FEED_PROBE, 0, 0, 0, 1, 0 },
	{ REPLAY_FIRST_FRAMES, 0, 0, 0, 0, 100 },
};

static void replay_refusal_apply(struct lecr_record *recs,
				 const struct replay_refusal *e)
{
	struct lecr_record *rec = &recs[e->index];

	rec->flags = (uint16_t)((rec->flags | e->set) & ~e->clear);
	if (e->skipped)
		rec->skipped = e->skipped;
	rec->line = (uint8_t)(rec->line + e->line);
	if (e->samples)
		rec->samples = e->samples;
}

/*
 * The rules of a lineage and of one record (bench/lec_replay.h) on the
 * synthetic call at 8 kHz. A lost final RUN, a lost NEW, a lost END and a
 * FEED whose frames moved each read as a gap and fail. The same record lost
 * in the ring reads as a loss and passes. A record that breaks a rule of one
 * record stops the file there, which reads INCOMPLETE, and names the rule.
 */
static void replay_rule_test(void)
{
	struct replay_seen seen;
	struct replay_call call;
	struct lecr_record *recs = NULL;
	struct lecr_agree a;
	unsigned i;

	if (replay_call_build(&call, UDSP_SAMPLE_RATE))
		recs = malloc(call.count * sizeof(*recs));

	CASE("lec replay reads an END after a lost final RUN as a gap");
	if (!recs) {
		CHECK(false);
		free(call.recs);
		return;
	}
	CHECK(replay_drop_run(&call, REPLAY_SECOND_LAST, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(a.losses, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	CHECK(replay_drop_run(&call, REPLAY_SECOND_LAST, 1, &a, &seen));
	CHECK_UINT(a.gaps, 0);
	CHECK_UINT(a.losses, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);

	CASE("lec replay reads records after an END without NEW as a gap");
	CHECK(replay_drop_run(&call, REPLAY_SECOND_NEW, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(a.instances, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	CHECK(replay_drop_run(&call, REPLAY_SECOND_NEW, 1, &a, &seen));
	CHECK_UINT(a.gaps, 0);
	CHECK_UINT(a.losses, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);

	CASE("lec replay reads a NEW without an END before it as a gap");
	CHECK(replay_drop_run(&call, REPLAY_FIRST_FRAMES, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(a.losses, 0);
	CHECK_UINT(a.instances, 2);
	CHECK_UINT(seen.segments, REPLAY_SEGMENTS);
	CHECK(seen.seg[1].lossy);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);

	CASE("lec replay reads a FEED whose frames moved as a gap");
	memcpy(recs, call.recs, call.count * sizeof(*recs));
	recs[REPLAY_FEED_PROBE].frames = 777u;
	CHECK(replay_recs_run(&call, recs, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	memcpy(recs, call.recs, call.count * sizeof(*recs));
	recs[REPLAY_SECOND_NEW].frames = 1u;
	CHECK(replay_recs_run(&call, recs, 0, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);

	CASE("lec replay refuses a record that breaks a rule of one record");
	for (i = 0; i < sizeof(replay_refusals) / sizeof(replay_refusals[0]);
	     i++) {
		const struct replay_refusal *e = &replay_refusals[i];

		memcpy(recs, call.recs, call.count * sizeof(*recs));
		replay_refusal_apply(recs, e);
		CHECK(replay_recs_run(&call, recs, 0, &a, &seen));
		printf("# edit %u of record %u: %s\n", i, e->index,
		       a.refusal ? a.refusal : "not refused");
		CHECK(a.parse_error);
		CHECK(a.refusal != NULL);
		CHECK_UINT(a.records, e->index);
		CHECK_UINT(lecr_agree_result(&a), LECR_INCOMPLETE);
	}

	CASE("lec replay reads version 1 and 2 files, and a synthetic FEED in "
	     "version 2 alone");
	for (i = 0; i < 4u; i++) {
		bool synthetic = i & 1u;
		uint16_t version = i < 2u ? 2u : 1u;
		char *buf = NULL;
		size_t size = 0;

		memcpy(recs, call.recs, call.count * sizeof(*recs));
		if (synthetic)
			recs[REPLAY_FEED_PROBE].flags = (uint16_t)
				((recs[REPLAY_FEED_PROBE].flags &
				  ~LECR_FLAGS_REASON) |
				 LECR_FLAG_SKIP_SYNTHETIC);
		CHECK(replay_file_write(recs, call.count, call.rate, &buf,
					&size));
		if (buf && size > 9u) {
			buf[8] = (char)version;
			replay_buffer_run(buf, size, false, &a, &seen);
			printf("# version %u, synthetic FEED %u: %s\n",
			       version, synthetic,
			       a.refusal ? a.refusal : "read");
			CHECK_UINT(a.parse_error, version == 1u && synthetic);
			CHECK_UINT(lecr_agree_result(&a),
				   version == 1u && synthetic ?
				   LECR_INCOMPLETE : LECR_PASS);
		}
		free(buf);
	}
	free(recs);
	free(call.recs);
}

/* The first 1500 frames, 15 s, read the other verdict and the other FIRED in
 * the replay. The 500 frames after them agree. */
static void replay_early_edit(unsigned f, struct lecr_record *rec,
			      struct lecr_record *got, const void *arg)
{
	(void)arg;
	rec->nlp_verdict = LECR_VERDICT_QUIET;
	got->nlp_verdict = LECR_VERDICT_QUIET;
	if (f >= 15u * LECR_TICKS_PER_SECOND)
		return;
	got->nlp_verdict = LECR_VERDICT_ERLE;
	rec->flags |= f % 2u ? LECR_FLAG_FIRED : 0u;
	got->flags |= f % 2u ? 0u : LECR_FLAG_FIRED;
}

/*
 * The default exclusion of lec_replay leaves 15 s of each segment out of the
 * steady-state figures, while control and continuity count from the first
 * record. A segment whose first 15 s disagree passes on the 500 frames after
 * them and fails without the exclusion; a mismatch or a gap there still fails.
 */
static void replay_skip_test(void)
{
	struct replay_seen seen;
	struct replay_call call;
	struct lecr_record *recs = NULL;
	struct lecr_agree a;

	CASE("lec replay leaves the first 15 s of a segment out of the "
	     "steady-state figures by default");
	CHECK_UINT(LECR_SKIP_S_DEFAULT, 15);
	replay_seg_drive(&a, LECR_SKIP_S_DEFAULT, 2000u, replay_early_edit,
			 NULL);
	CHECK_UINT(a.done.runs, 2000);
	CHECK_UINT(a.done.body, 500);
	CHECK_UINT(a.done.verdict_same, 500);
	CHECK_UINT(a.done.fired_diff, 0);
	CHECK_UINT(lecr_segment_faults(&a.done), 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_PASS);
	replay_seg_drive(&a, 0, 2000u, replay_early_edit, NULL);
	CHECK_UINT(a.done.body, 2000);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);

	CASE("lec replay counts the frames and the gaps from the first record "
	     "under the default exclusion");
	if (replay_call_build(&call, UDSP_SAMPLE_RATE))
		recs = malloc(call.count * sizeof(*recs));
	if (!recs) {
		CHECK(false);
		free(call.recs);
		return;
	}
	memcpy(recs, call.recs, call.count * sizeof(*recs));
	CHECK(replay_recs_run(&call, recs, LECR_SKIP_S_DEFAULT, &a, &seen));
	CHECK_UINT(a.evaluated, 0);
	CHECK_UINT(a.frames_bad + a.gaps, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_INCOMPLETE);
	recs[REPLAY_RESET_AT / 2u].frames++;
	CHECK(replay_recs_run(&call, recs, LECR_SKIP_S_DEFAULT, &a, &seen));
	CHECK_UINT(a.frames_bad, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	recs[REPLAY_RESET_AT / 2u].frames--;
	memmove(recs + REPLAY_LOSS_AT, recs + REPLAY_LOSS_AT + 1u,
		(call.count - REPLAY_LOSS_AT - 1u) * sizeof(*recs));
	call.count--;
	CHECK(replay_recs_run(&call, recs, LECR_SKIP_S_DEFAULT, &a, &seen));
	CHECK_UINT(a.gaps, 1);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);
	free(recs);
	free(call.recs);
}

/* The live and the replay figures of one segment of 1000 frames. The first
 * quiet frames read quiet and the rest pass with ACTIVE. The replay reads
 * the first moved of them as a pass with ACTIVE. The last fired frames carry
 * FIRED, and the replay clears it on the first flipped of them. */
struct replay_bound {
	unsigned quiet;
	unsigned moved;
	unsigned fired;
	unsigned flipped;
};

#define REPLAY_BOUND_FRAMES	1000u

static void replay_bound_edit(unsigned f, struct lecr_record *rec,
			      struct lecr_record *got, const void *arg)
{
	const struct replay_bound *b = arg;
	unsigned fire_from = REPLAY_BOUND_FRAMES - b->fired;

	rec->nlp_verdict = f < b->quiet ? LECR_VERDICT_QUIET :
					  LECR_VERDICT_PASS;
	rec->flags |= f < b->quiet ? 0u : LECR_FLAG_ACTIVE;
	rec->flags |= f >= fire_from ? LECR_FLAG_FIRED : 0u;
	got->nlp_verdict = rec->nlp_verdict;
	got->flags = rec->flags;
	if (f < b->moved) {
		got->nlp_verdict = LECR_VERDICT_PASS;
		got->flags |= LECR_FLAG_ACTIVE;
	}
	if (f >= fire_from && f < fire_from + b->flipped)
		got->flags &= (uint16_t)~LECR_FLAG_FIRED;
}

static unsigned replay_bound_faults(unsigned quiet, unsigned moved,
				    unsigned fired, unsigned flipped)
{
	struct replay_bound b = { quiet, moved, fired, flipped };
	struct lecr_agree a;

	replay_seg_drive(&a, 0, REPLAY_BOUND_FRAMES, replay_bound_edit, &b);
	CHECK_UINT(a.done.body, REPLAY_BOUND_FRAMES);
	CHECK_UINT(a.failed, lecr_segment_faults(&a.done) ? 1u : 0u);
	return lecr_segment_faults(&a.done);
}

/*
 * The bound of a count and of the FIRED symmetric difference is max(5,
 * ceil(0.05 x live)). 210 live frames give 11 and 40 give 5. A difference on
 * the bound passes, and one over it fails. A bound rounded down, a bound
 * without its floor of 5 or a test with >= fails here.
 */
static void replay_bound_test(void)
{
	CASE("lec replay passes a count on its bound and fails it one over");
	CHECK_UINT(lecr_bound_get(210u), 11);
	CHECK_UINT(lecr_bound_get(40u), 5);
	CHECK_UINT(replay_bound_faults(210u, 11u, 0, 0), 0);
	CHECK_UINT(replay_bound_faults(210u, 12u, 0, 0), LECR_FAULT_COUNTS);
	CHECK_UINT(replay_bound_faults(40u, 5u, 0, 0), 0);
	CHECK_UINT(replay_bound_faults(40u, 6u, 0, 0), LECR_FAULT_COUNTS);

	CASE("lec replay passes the fired difference on its bound and fails "
	     "it one over");
	CHECK_UINT(replay_bound_faults(0, 0, 210u, 11u), 0);
	CHECK_UINT(replay_bound_faults(0, 0, 210u, 12u), LECR_FAULT_FIRED);
	CHECK_UINT(replay_bound_faults(0, 0, 40u, 5u), 0);
	CHECK_UINT(replay_bound_faults(0, 0, 40u, 6u), LECR_FAULT_FIRED);
}

/*
 * The two jumps of the gap fixture, in the shape of a capture whose archive
 * lost FEED records. Before RUN record at, the live instance takes feeds FEED
 * records that the file loses. The engine then skips skipped periods, which
 * the RUN record carries. Each jump is feeds + skipped + 1 = 7 ticks,
 * and the record accounts for skipped + 1 of them. The lost FEED records
 * carry the far end.
 */
static const struct replay_gap {
	unsigned at;
	unsigned feeds;
	uint16_t skipped;
} replay_gaps[] = {
	{ 220u, 4u, 2u },
	{ 420u, 3u, 3u },
};

#define REPLAY_GAPS		(sizeof(replay_gaps) / sizeof(replay_gaps[0]))
#define REPLAY_GAP_RUNS		600u
#define REPLAY_GAP_JUMP		7u
/* The RUN records, the seven lost FEED records and the END. */
#define REPLAY_GAP_RECORDS	(REPLAY_GAP_RUNS + 8u)

/* One lineage at 8 kHz from NEW to END, with the processor on, the jumps of
 * replay_gaps and no other FEED. The records of the file leave out the
 * FEED records that the live instance took. */
static bool replay_gap_build(struct replay_call *c)
{
	unsigned g = 0, k = 0, kept = 0;
	unsigned f, j;

	if (!replay_call_init(c, UDSP_SAMPLE_RATE, REPLAY_GAP_RECORDS))
		return false;
	for (f = 0; f < REPLAY_GAP_RUNS; f++) {
		if (k < REPLAY_GAPS && f == replay_gaps[k].at) {
			for (j = 0; j < replay_gaps[k].feeds; j++)
				replay_tick_run(c, g++, false,
						LECR_FLAG_SKIP_OPEN, false,
						false, true);
			c->tick += replay_gaps[k].skipped;
			c->skip_next = replay_gaps[k].skipped;
			k++;
		}
		replay_tick_run(c, g++, f == 0u, 0, false, false, true);
	}
	replay_end_push(c);
	for (j = 0; j < c->count; j++)
		if (c->recs[j].kind != LECR_KIND_FEED)
			c->recs[kept++] = c->recs[j];
	c->count = kept;
	return !c->failed;
}

/*
 * A lineage has two tick gaps of 7 ticks each: skipped 2 accounts for 3, and
 * skipped 3 for 4. The live instance took FEED records in the gaps, and the
 * file lacks them. The replay closes the open segment as lossy at each gap
 * and goes on. Every RUN replays, in three lossy segments that NEW and the
 * two gaps open. Frame counts agree across both gaps, output after them
 * differs, and the gaps alone fail the file. A replay that stops at a gap
 * fails on the replayed and segment counts.
 */
static void replay_gap_test(void)
{
	static const unsigned from[] = {
		LECR_FROM_NEW, LECR_FROM_GAP, LECR_FROM_GAP,
	};
	struct replay_seen seen;
	struct replay_call call;
	struct lecr_agree a;
	bool built = replay_gap_build(&call);
	bool ran = built && replay_recs_run(&call, call.recs, 0, &a, &seen);
	unsigned i;

	CASE("lec replay goes on past a tick gap without a recorded loss");
	CHECK(built);
	CHECK(ran);
	if (!ran) {
		free(call.recs);
		return;
	}
	printf("# %u of %u RUN records replayed in %u segments; %u gaps, %u "
	       "records skipped, %u frames mismatches, %u RUN records "
	       "differ\n", a.runs, REPLAY_GAP_RUNS, a.segments, a.gaps,
	       a.skipped, a.frames_bad, seen.diffs);
	CHECK_UINT(seen.runs, REPLAY_GAP_RUNS);
	CHECK_UINT(a.runs, REPLAY_GAP_RUNS);
	CHECK_UINT(a.skipped, 0);
	CHECK_UINT(a.segments, 3);
	CHECK_UINT(seen.segments, 3);
	CHECK_UINT(a.gaps, REPLAY_GAPS);
	CHECK_UINT(a.losses + a.skip_marks, 0);
	CHECK_UINT(a.frames_bad, 0);
	CHECK_UINT(a.evaluated, 0);
	CHECK_UINT(lecr_agree_result(&a), LECR_FAIL);

	CASE("lec replay opens a lossy segment at each tick gap");
	for (i = 0; i < 3u && i < seen.segments; i++) {
		const struct lecr_segment *s = &seen.seg[i];
		unsigned first = i ? replay_gaps[i - 1u].at : 0u;
		unsigned end = i < REPLAY_GAPS ? replay_gaps[i].at :
						 REPLAY_GAP_RUNS;

		printf("# segment %u from %s, ticks %llu to %llu, %u RUN "
		       "records, lossy %u, frames agree on %u, out on %u\n",
		       s->index, lecr_from_name(s->from),
		       (unsigned long long)s->tick_first,
		       (unsigned long long)s->tick_last, s->runs, s->lossy,
		       s->frames_same, s->out_same);
		CHECK_UINT(s->from, from[i]);
		CHECK(s->lossy);
		CHECK_UINT(s->runs, end - first);
		CHECK_UINT(s->frames_same, s->runs);
		if (i)
			CHECK_UINT(s->tick_first - seen.seg[i - 1u].tick_last,
				   REPLAY_GAP_JUMP);
	}
	/* Before the first gap, the replay is the live run. After it, the
	 * replay lacks the far end of the lost FEED records. */
	CHECK_UINT(seen.seg[0].out_same, seen.seg[0].runs);
	CHECK(seen.seg[1].out_same < seen.seg[1].runs);
	free(call.recs);
}

#define PAUSE_ERL_DB		24.0
#define PAUSE_FLOOR_DBM0	-60.0
#define PAUSE_FAR_DBM0		-10.0
#define PAUSE_NEAR_DBM0		-25.0
#define PAUSE_FAR_FRAMES	800u
#define PAUSE_WORDS_FRAMES	300u
#define PAUSE_WORDS_DELAY	50u
#define PAUSE_SPEECH_FRAMES	500u
#define PAUSE_RISE_MAX_DB	1.0

/*
 * The line of the pause cases: 8 kHz, a flat 24 dB hybrid at 8 ms, and a
 * floor at -60 dBm0 on every frame. Far-end words at -10 dBm0 run for 8 s,
 * and their pauses measure the floor.
 */
struct pause_line {
	struct echo_path path;
	struct udsp_ec *ec;
	struct word_source far;
	struct word_source near;
	uint32_t floor_state;
	uint32_t near_state;
};

/* One frame of far-end words, or of a far-end pause, with near on top when
 * it is given. It returns the tracked floor. */
static float pause_frame_run(struct pause_line *line, bool far,
			     const int16_t *near)
{
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	unsigned i;

	if (far)
		word_fill(&line->far, ref, UDSP_FRAME_SAMPLES);
	else
		memset(ref, 0, sizeof(ref));
	path_run(&line->path, ref, echo);
	floor_add(mic, echo, PAUSE_FLOOR_DBM0, &line->floor_state);
	for (i = 0; near && i < UDSP_FRAME_SAMPLES; i++)
		mic[i] = sample_clamp((double)mic[i] + near[i]);
	udsp_ec_nlms.run(line->ec, mic, ref);
	udsp_ec_nlms_info_get(line->ec, &info);
	return info.noise_dbov;
}

/* The line after its 8 s of far-end words. It returns the tracked floor. */
static float pause_line_create(struct pause_line *line)
{
	float floor = 127.0f;
	unsigned f;

	memset(line, 0, sizeof(*line));
	line->floor_state = 0x0f100f10u;
	line->near_state = 0x600dcafeu;
	line->ec = udsp_ec_nlms.create(48);
	path_flat_init(&line->path, UDSP_SAMPLE_RATE, 8, PAUSE_ERL_DB);
	word_source_init(&line->far, UDSP_SAMPLE_RATE, PAUSE_FAR_DBM0,
			 0x13572468u);
	word_source_init(&line->near, UDSP_SAMPLE_RATE, PAUSE_NEAR_DBM0,
			 0xfeedfaceu);
	for (f = 0; f < PAUSE_FAR_FRAMES; f++)
		floor = pause_frame_run(line, true, NULL);
	return floor;
}

/*
 * Near-end words in a far-end pause. After 8 s of far-end words, a 3 s pause
 * carries 4 near-end words at -25 dBm0 from its first 0.5 s on. The floor
 * must not rise by more than 1 dB on any frame of the pause. A tracker
 * without the rise test rises by 17 dB on the first word.
 */
static void floor_pause_words_test(void)
{
	int16_t near[UDSP_FRAME_SAMPLES];
	struct pause_line line;
	float start = pause_line_create(&line);
	float loudest = start;
	unsigned words = 0;
	unsigned f;

	for (f = 0; f < PAUSE_WORDS_FRAMES; f++) {
		float floor;

		memset(near, 0, sizeof(near));
		if (f >= PAUSE_WORDS_DELAY) {
			word_fill(&line.near, near, UDSP_FRAME_SAMPLES);
			words += line.near.offset;
		}
		floor = pause_frame_run(&line, false, near);
		if (floor < loudest)
			loudest = floor;
	}

	CASE("lec keeps near-end words in a far-end pause out of the floor");
	printf("# near-end floor %.2f dBov, tracked %.2f dBov before the "
	       "pause, loudest %.2f dBov in it, %u near words\n",
	       floor_dbov_get(PAUSE_FLOOR_DBM0), (double)start,
	       (double)loudest, words);
	CHECK(fabs(start - floor_dbov_get(PAUSE_FLOOR_DBM0)) <= 1.5);
	CHECK_UINT(words, 4);
	CHECK(loudest >= start - PAUSE_RISE_MAX_DB);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * Continuous speech in a far-end pause. The pause carries 5 s of syllabic
 * near-end speech at a mean of -25 dBm0, with the 14 dB swing of the
 * syllabic envelope. The floor must not rise by more than 1 dB. A tracker
 * without the rise test rises by 30 dB within about 1 s.
 */
static void floor_pause_speech_test(void)
{
	int16_t near[UDSP_FRAME_SAMPLES];
	struct pause_line line;
	float start = pause_line_create(&line);
	float loudest = start;
	unsigned f;

	for (f = 0; f < PAUSE_SPEECH_FRAMES; f++) {
		float floor;

		noise_fill(near, UDSP_FRAME_SAMPLES,
			   syllabic_dbm0_get(f, PAUSE_NEAR_DBM0),
			   &line.near_state);
		floor = pause_frame_run(&line, false, near);
		if (floor < loudest)
			loudest = floor;
	}

	CASE("lec keeps continuous speech in a far-end pause out of the floor");
	printf("# tracked floor %.2f dBov before the pause, loudest %.2f dBov "
	       "in it\n", (double)start, (double)loudest);
	CHECK(fabs(start - floor_dbov_get(PAUSE_FLOOR_DBM0)) <= 1.5);
	CHECK(loudest >= start - PAUSE_RISE_MAX_DB);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A near-end floor that rises by 15 dB and stays, then falls back, with the
 * far end silent. After 60 frames the floor must be within 1.5 dB of
 * -45 dBm0. It must be within 1 dB of -60 dBm0 20 frames after the fall. A
 * tracker that never follows a rise, or that follows a fall slowly, fails
 * here.
 */
static void floor_rise_test(void)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);

	silence_run(ec, -60.0, 50u);
	silence_run(ec, -45.0, 60u);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec follows a near-end floor that rises and stays");
	printf("# near-end floor %.2f dBov, tracked %.2f dBov after 60 "
	       "frames\n", floor_dbov_get(-45.0), (double)info.noise_dbov);
	CHECK(fabs(info.noise_dbov - floor_dbov_get(-45.0)) <= 1.5);

	silence_run(ec, -60.0, 20u);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec follows a near-end floor that falls");
	printf("# near-end floor %.2f dBov, tracked %.2f dBov after 20 "
	       "frames\n", floor_dbov_get(-60.0), (double)info.noise_dbov);
	CHECK(fabs(info.noise_dbov - floor_dbov_get(-60.0)) <= 1.0);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The first measurable floor after digital silence. A fresh instance sees
 * 50 frames of digital silence and then 10 frames of a floor at -50 dBm0.
 * The floor must read 127, no floor, after the silence, and the level of the
 * floor within 1 dB after the 10 frames. A silence that seeds the floor at
 * 127 leaves 127 there, and the case fails.
 */
static void floor_silence_test(void)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	float silent;

	silence_run(ec, -200.0, 50u);
	udsp_ec_nlms_info_get(ec, &info);
	silent = info.noise_dbov;
	silence_run(ec, -50.0, 10u);
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec takes the first measurable floor whole after digital "
	     "silence");
	printf("# tracked floor %.2f dBov after the silence, %.2f dBov after "
	       "10 frames of a floor at %.2f dBov\n", (double)silent,
	       (double)info.noise_dbov, floor_dbov_get(-50.0));
	CHECK(silent == 127.0f);
	CHECK(fabs(info.noise_dbov - floor_dbov_get(-50.0)) <= 1.0);
	udsp_ec_nlms.destroy(ec);
}

#define SHORT_PAUSE_WORD	25u
#define SHORT_PAUSE_CYCLE	55u
#define SHORT_PAUSE_LOW_DBM0	-60.0
#define SHORT_PAUSE_HIGH_DBM0	-45.0
#define SHORT_PAUSE_LOW_FRAMES	300u
#define SHORT_PAUSE_HIGH_FRAMES	500u

/*
 * A near-end floor rises from -60 to -45 dBm0. The far end talks in syllabic
 * bursts of 25 frames with pauses of 30 frames. A pause gives 26
 * quiet-window readings, fewer than a full rise run. The floor must be
 * within 1.5 dB of -45 dBm0 5 s after the rise. A rise run that the far-end
 * frames clear never fills, and fails here.
 */
static void floor_short_pause_test(void)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x2468ace1u;
	uint32_t floor_state = 0x0f100f10u;
	double high = floor_dbov_get(SHORT_PAUSE_HIGH_DBM0);
	float low = 127.0f;
	unsigned followed = 0;
	unsigned f;

	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, PAUSE_ERL_DB);
	for (f = 0; f < SHORT_PAUSE_LOW_FRAMES + SHORT_PAUSE_HIGH_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t echo[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];
		bool rose = f >= SHORT_PAUSE_LOW_FRAMES;

		memset(ref, 0, sizeof(ref));
		if (f % SHORT_PAUSE_CYCLE < SHORT_PAUSE_WORD)
			noise_fill(ref, UDSP_FRAME_SAMPLES,
				   syllabic_dbm0_get(f, PAUSE_FAR_DBM0),
				   &state);
		path_run(&path, ref, echo);
		floor_add(mic, echo, rose ? SHORT_PAUSE_HIGH_DBM0 :
			  SHORT_PAUSE_LOW_DBM0, &floor_state);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (!rose)
			low = info.noise_dbov;
		else if (!followed && fabs(info.noise_dbov - high) <= 1.5)
			followed = f + 1u - SHORT_PAUSE_LOW_FRAMES;
	}

	CASE("lec follows a floor that rises through short far-end pauses");
	printf("# tracked floor %.2f dBov before the rise, %.2f dBov 5 s "
	       "after it against %.2f dBov, within 1.5 dB after %u frames\n",
	       (double)low, (double)info.noise_dbov, high, followed);
	CHECK(fabs(low - floor_dbov_get(SHORT_PAUSE_LOW_DBM0)) <= 1.5);
	CHECK(fabs(info.noise_dbov - high) <= 1.5);
	udsp_ec_nlms.destroy(ec);
}

/*
 * One frame of samples of a frozen instance whose microphone reads dbov
 * exactly: a square wave at the amplitude of that level. With far set, far-end
 * noise makes the frame active. The filter holds no energy, so the residual is
 * the microphone. It returns the tracked floor, and sets *quiet when the whole
 * window of the frame is quiet.
 */
static float exact_samples_run(struct udsp_ec *ec, unsigned samples, int dbov,
			       bool far, uint32_t *state, bool *quiet)
{
	int16_t amplitude = (int16_t)lrint(32767.0 * pow(10.0, -dbov / 20.0));
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	for (i = 0; i < samples; i++)
		mic[i] = i % 2u ? (int16_t)-amplitude : amplitude;
	CHECK(udsp_cn_level_get(mic, samples) == dbov);
	memset(ref, 0, sizeof(ref));
	if (far)
		noise_fill(ref, samples, PAUSE_FAR_DBM0, state);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_info_get(ec, &info);
	udsp_ec_nlms_frame_get(ec, &reading);
	*quiet = reading.window_run == 0u;
	return info.noise_dbov;
}

/* exact_samples_run on a frame of an 8 kHz instance. */
static float exact_frame_run(struct udsp_ec *ec, int dbov, bool far,
			     uint32_t *state, bool *quiet)
{
	return exact_samples_run(ec, UDSP_FRAME_SAMPLES, dbov, far, state,
				 quiet);
}

/* n active frames at dbov. */
static void exact_far_run(struct udsp_ec *ec, int dbov, unsigned n,
			  uint32_t *state)
{
	bool quiet;
	unsigned i;

	for (i = 0; i < n; i++)
		(void)exact_frame_run(ec, dbov, true, state, &quiet);
}

/* Far-end pause frames at dbov until n of them are quiet-window frames, the
 * readings of the tracker. *tail counts the frames before them whose window is
 * still active. It returns the tracked floor. */
static float exact_quiet_run(struct udsp_ec *ec, int dbov, unsigned n,
			     uint32_t *state, unsigned *tail)
{
	float floor = 127.0f;
	unsigned read = 0;

	*tail = 0;
	while (read < n) {
		bool quiet;

		floor = exact_frame_run(ec, dbov, false, state, &quiet);
		if (quiet)
			read++;
		else
			(*tail)++;
	}
	return floor;
}

/* exact_quiet_run on a window that is already quiet. */
static float exact_read_run(struct udsp_ec *ec, int dbov, unsigned n,
			    uint32_t *state)
{
	unsigned tail;
	float floor = exact_quiet_run(ec, dbov, n, state, &tail);

	CHECK_UINT(tail, 0);
	return floor;
}

/* A frozen 48 ms instance whose floor reads dbov, taken whole from its first
 * frame. */
static struct udsp_ec *exact_line_create(int dbov, uint32_t *state)
{
	struct udsp_ec *ec = udsp_ec_nlms.create(48);

	udsp_ec_nlms.freeze(ec, true);
	(void)exact_read_run(ec, dbov, 1u, state);
	return ec;
}

/*
 * The edge of the rise test at LEC_NOISE_RISE_DB, 3 dB. A floor at 60.90 dBov
 * reads a frame at 58 dBov, 2.90 dB over it, and moves by 2 percent of the
 * gap. A floor at 64.10 dBov reads a frame at 61 dBov, 3.10 dB over it, and
 * holds. That reading is the first of a run of 50 at 61 dBov, which sets the
 * floor to 61. A bound of 2 or 4 dB fails one of the two.
 */
static void floor_bound_test(void)
{
	uint32_t state = 0x31415927u;
	struct udsp_ec *ec;
	float before;
	float after;

	CASE("lec moves the floor slowly on a reading 2.9 dB over it");
	ec = exact_line_create(60, &state);
	before = exact_read_run(ec, 61, 8u, &state);
	after = exact_read_run(ec, 58, 1u, &state);
	printf("# floor %.4f dBov, reading 2.9 dB over it, floor %.4f dBov "
	       "after\n", (double)before, (double)after);
	CHECK(before - 58.0f > 2.85f && before - 58.0f < 2.95f);
	CHECK(fabsf(after - (before - (before - 58.0f) * 0.02f)) < 0.001f);
	udsp_ec_nlms.destroy(ec);

	CASE("lec holds the floor on a reading 3.1 dB over it");
	ec = exact_line_create(60, &state);
	before = exact_read_run(ec, 66, 4u, &state);
	after = exact_read_run(ec, 61, 1u, &state);
	printf("# floor %.4f dBov, reading 3.1 dB over it, floor %.4f dBov "
	       "after\n", (double)before, (double)after);
	CHECK(before - 61.0f > 3.05f && before - 61.0f < 3.15f);
	CHECK(after == before);
	CHECK(exact_read_run(ec, 61, 48u, &state) == before);
	CHECK(exact_read_run(ec, 61, 1u, &state) == 61.0f);
	udsp_ec_nlms.destroy(ec);
}

/* A floor at 60 dBov reads lead frames at 50 dBov, one frame at dbov, 49
 * more at 50 dBov, and then one more. */
static void floor_break_check(unsigned lead, int dbov)
{
	uint32_t state = 0x27182819u;
	struct udsp_ec *ec = exact_line_create(60, &state);
	float broken;
	float held;

	CHECK(exact_read_run(ec, 50, lead, &state) == 60.0f);
	broken = exact_read_run(ec, dbov, 1u, &state);
	held = exact_read_run(ec, 50, 49u, &state);
	printf("# floor %.2f dBov after the reading at %d dBov, %.2f dBov 49 "
	       "readings later\n", (double)broken, dbov, (double)held);
	CHECK(held == broken);
	CHECK(exact_read_run(ec, 50, 1u, &state) == 50.0f);
	udsp_ec_nlms.destroy(ec);
}

/*
 * An in-band reading ends a rise run that holds fewer than
 * LEC_NOISE_CREEP_SHARE readings per in-band reading, and a quieter reading
 * ends any rise run. The floor holds after 49 more louder readings, and the
 * 50th completes a run that sets it. noise_creep_test covers a kept reading.
 */
static void floor_run_break_test(void)
{
	CASE("lec starts the rise run again after an in-band reading");
	floor_break_check(9u, 60);

	CASE("lec starts the rise run again after a quieter reading");
	floor_break_check(49u, 62);
}

#define QUIET6_READINGS		6u
#define QUIET6_ACTIVE		10u
#define QUIET6_CYCLES		9u

/*
 * Quiet windows of 6 frames between far-end bursts of 10 frames, with a floor
 * that rose from 60 to 50 dBov. The rise run holds its readings across the
 * bursts, and the 9th window completes it and sets the floor to 50 dBov. The
 * 50 readings come from 9 windows, fewer than the agreement run needs, so a
 * rise run that the bursts clear fails here.
 */
static void floor_quiet6_test(void)
{
	uint32_t state = 0x16180340u;
	struct udsp_ec *ec;
	float before_last = 127.0f;
	float floor = 127.0f;
	unsigned tails = 0;
	unsigned c;

	CASE("lec adds up the rise readings of quiet windows of 6 frames");
	ec = exact_line_create(60, &state);
	for (c = 0; c < QUIET6_CYCLES; c++) {
		unsigned tail;

		exact_far_run(ec, 50, QUIET6_ACTIVE, &state);
		before_last = floor;
		floor = exact_quiet_run(ec, 50, QUIET6_READINGS, &state,
					&tail);
		tails += tail;
	}
	printf("# floor %.2f dBov after 8 windows, %.2f dBov after 9, %u "
	       "tail frames\n", (double)before_last, (double)floor, tails);
	CHECK_UINT(tails, 4u * QUIET6_CYCLES);
	CHECK(before_last == 60.0f);
	CHECK(floor == 50.0f);
	udsp_ec_nlms.destroy(ec);
}

/* The agreement line: a frozen 48 ms instance whose microphone reads whole dBov
 * levels (exact_samples_run). A far noise burst of AGREE_BURST_FRAMES leads
 * each quiet window, which reads once the reference window is quiet. */
#define AGREE_BURST_FRAMES	10u

struct agree_line {
	struct udsp_ec *ec;
	unsigned frame;
	uint32_t state;
};

/* A line at rate whose floor reads dbov, taken whole from its first frame. */
static void agree_line_init(struct agree_line *l, unsigned rate, int dbov)
{
	bool quiet;

	l->ec = udsp_ec_nlms.create_rate(48, rate);
	l->frame = rate / 100u;
	l->state = 0x2545f491u;
	udsp_ec_nlms.freeze(l->ec, true);
	(void)exact_samples_run(l->ec, l->frame, dbov, false, &l->state,
				&quiet);
	CHECK(quiet);
}

/* n more readings at dbov in the quiet window that is open. It returns the
 * floor after the last. */
static float agree_read_run(struct agree_line *l, int dbov, unsigned n)
{
	float floor = 127.0f;
	unsigned i;

	for (i = 0; i < n; i++) {
		bool quiet;

		floor = exact_samples_run(l->ec, l->frame, dbov, false,
					  &l->state, &quiet);
		CHECK(quiet);
	}
	return floor;
}

/*
 * A burst of len frames whose microphone reads burst_dbov, with far-end noise
 * on frame i when bit i of far is set. Pause frames at dbov follow until n of
 * them are readings: one quiet window. It returns the floor after the last
 * reading.
 */
static float agree_pattern_run(struct agree_line *l, unsigned far,
			       unsigned len, int burst_dbov, int dbov,
			       unsigned n)
{
	float floor = 127.0f;
	unsigned read = 0;
	unsigned i;
	bool quiet;

	for (i = 0; i < len; i++)
		(void)exact_samples_run(l->ec, l->frame, burst_dbov,
					(far >> i) & 1u, &l->state, &quiet);
	while (read < n) {
		floor = exact_samples_run(l->ec, l->frame, dbov, false,
					  &l->state, &quiet);
		read += quiet;
	}
	return floor;
}

/*
 * A far burst of AGREE_BURST_FRAMES whose filter output reads burst_dbov,
 * then pause frames at dbov until n of them are readings: one quiet window.
 * It returns the floor after the last reading.
 */
static float agree_window_run(struct agree_line *l, int burst_dbov, int dbov,
			      unsigned n)
{
	return agree_pattern_run(l, (1u << AGREE_BURST_FRAMES) - 1u,
				 AGREE_BURST_FRAMES, burst_dbov, dbov, n);
}

/* count quiet windows of n readings at dbov, each after a burst at dbov. It
 * returns the floor after the last reading. */
static float agree_windows_run(struct agree_line *l, int dbov, unsigned n,
			       unsigned count)
{
	float floor = 127.0f;
	unsigned w;

	for (w = 0; w < count; w++)
		floor = agree_window_run(l, dbov, dbov, n);
	return floor;
}

/*
 * The count of the agreement run, LEC_NOISE_AGREE_READINGS. A floor at
 * 60 dBov reads quiet windows of 2 frames at 54 dBov and holds through 31
 * readings. The 32nd completes the run, and the floor takes its mean plus
 * LEC_NOISE_AGREE_BIAS_DB, 55 dBov. A run of 24 readings, or a floor at the
 * mean of the run, fails here.
 */
static void agree_readings_test(unsigned rate)
{
	struct agree_line l;
	float held;
	float floor;

	case_rate(rate, "lec raises the floor on the 32nd reading of the "
		  "agreement run");
	agree_line_init(&l, rate, 60);
	(void)agree_windows_run(&l, 54, 2u, 15u);
	held = agree_window_run(&l, 54, 54, 1u);
	floor = agree_read_run(&l, 54, 1u);
	printf("# %u Hz: floor %.2f dBov after 31 readings, %.2f dBov after "
	       "32\n", rate, (double)held, (double)floor);
	CHECK(held == 60.0f);
	CHECK(floor == 55.0f);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The count of quiet windows of the agreement run, LEC_NOISE_AGREE_PAUSES.
 * Windows of 4 frames at 54 dBov over a floor at 60 dBov hold the floor through
 * 9 windows. The 10th sets it to 55 dBov. A count of 8 windows, or of
 * readings in place of windows, fails here.
 */
static void agree_pauses_test(unsigned rate)
{
	struct agree_line l;
	float held;
	float floor;

	case_rate(rate, "lec raises the floor on readings from 10 quiet "
		  "windows");
	agree_line_init(&l, rate, 60);
	held = agree_windows_run(&l, 54, 4u, 9u);
	floor = agree_window_run(&l, 54, 54, 1u);
	printf("# %u Hz: floor %.2f dBov after 9 windows of 4 readings, %.2f "
	       "dBov on the first reading of the 10th\n", rate, (double)held,
	       (double)floor);
	CHECK(held == 60.0f);
	CHECK(floor == 55.0f);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The readings of one quiet window are one look at the near end. A floor at
 * 60 dBov reads 49 frames at 54 dBov in one window, and holds. The 50th
 * reading completes the rise run, and the floor takes its mean, 54 dBov. An
 * agreement run that counts each reading as a window fails here.
 */
static void agree_window_test(unsigned rate)
{
	struct agree_line l;
	float held;
	float floor;

	case_rate(rate, "lec counts the readings of one quiet window as one "
		  "look in the agreement run");
	agree_line_init(&l, rate, 60);
	held = agree_window_run(&l, 54, 54, 49u);
	floor = agree_read_run(&l, 54, 1u);
	printf("# %u Hz: floor %.2f dBov after 49 readings of one window, "
	       "%.2f dBov after 50\n", rate, (double)held, (double)floor);
	CHECK(held == 60.0f);
	CHECK(floor == 54.0f);
	udsp_ec_nlms.destroy(l.ec);
}

/* 16 quiet windows of 2 readings, at the first level and the second in turn,
 * on a line whose floor reads floor_dbov. Each burst reads the first level,
 * the louder one, so no burst falsifies the run. *held is the floor after
 * 15. */
static float agree_band_run(unsigned rate, int floor_dbov, const int levels[2],
			    float *held)
{
	struct agree_line l;
	float floor = 127.0f;
	unsigned w;

	agree_line_init(&l, rate, floor_dbov);
	for (w = 0; w < 16u; w++) {
		if (w == 15u)
			*held = floor;
		floor = agree_window_run(&l, levels[0], levels[w % 2u], 2u);
	}
	udsp_ec_nlms.destroy(l.ec);
	return floor;
}

/* 30 quiet windows of 2 readings at 51, 55, 60 and 55 dBov in turn, after
 * bursts at 51 dBov. The floor of the line reads 64 dBov. It returns the
 * floor after the last reading. */
static float agree_ramp_run(unsigned rate)
{
	static const int levels[4] = { 51, 55, 60, 55 };
	struct agree_line l;
	float floor = 127.0f;
	unsigned w;

	agree_line_init(&l, rate, 64);
	for (w = 0; w < 30u; w++)
		floor = agree_window_run(&l, 51, levels[w % 4u], 2u);
	udsp_ec_nlms.destroy(l.ec);
	return floor;
}

/* On a line whose floor reads 63 dBov, after bursts at 50 dBov: 13 readings
 * at 50 dBov in windows of 4. From the second reading of the 4th window on,
 * 59 dBov follows, in 4 windows of 3 and 4 windows of 4. held is the floor
 * after the 12th window; the return is the floor after the first reading of
 * the 13th. */
static float agree_restart_run(unsigned rate, float *held)
{
	struct agree_line l;
	float floor;
	unsigned w;

	agree_line_init(&l, rate, 63);
	(void)agree_windows_run(&l, 50, 4u, 3u);
	(void)agree_window_run(&l, 50, 50, 1u);
	(void)agree_read_run(&l, 59, 3u);
	for (w = 0; w < 8u; w++)
		*held = agree_window_run(&l, 50, 59, w < 4u ? 3u : 4u);
	floor = agree_window_run(&l, 50, 59, 1u);
	udsp_ec_nlms.destroy(l.ec);
	return floor;
}

/*
 * The band of the agreement run, LEC_NOISE_AGREE_DB, spans all its readings.
 * Readings 8 dB apart raise the floor, and 9 dB apart hold it, also in steps of
 * 4 and 5 dB. A reading out of the band restarts the run with that reading and
 * its window. A band of 6 or 10 dB, or one around the mean, fails here.
 */
static void agree_band_test(unsigned rate)
{
	static const int within[2] = { 50, 58 };
	static const int beyond[2] = { 50, 59 };
	float held_within;
	float held_beyond;
	float floor_within;
	float floor_beyond;
	float floor_ramp;
	float held_restart = 127.0f;
	float floor_restart;

	case_rate(rate, "lec raises the floor on readings within 8 dB of each "
		  "other");
	floor_within = agree_band_run(rate, 62, within, &held_within);
	floor_beyond = agree_band_run(rate, 63, beyond, &held_beyond);
	floor_ramp = agree_ramp_run(rate);
	floor_restart = agree_restart_run(rate, &held_restart);
	printf("# %u Hz: readings 8 dB apart: floor %.2f dBov after 15 "
	       "windows, %.2f dBov after 16; 9 dB apart: %.2f and %.2f "
	       "dBov; steps of 5 dB over 9 dB: %.2f dBov after 30 windows; "
	       "restart in the 4th window: %.2f dBov after 12 windows, %.2f "
	       "dBov on the first reading of the 13th\n", rate,
	       (double)held_within, (double)floor_within, (double)held_beyond,
	       (double)floor_beyond, (double)floor_ramp, (double)held_restart,
	       (double)floor_restart);
	CHECK(held_within == 62.0f);
	CHECK(floor_within == 55.0f);
	CHECK(held_beyond == 63.0f);
	CHECK(floor_beyond == 63.0f);
	CHECK(floor_ramp == 64.0f);
	CHECK(held_restart == 63.0f);
	CHECK(floor_restart == 60.0f);
}

/* Quiet windows of 1 reading at 54 dBov after bursts of len frames at
 * burst_dbov. Far-end noise is on the frames that the bits of far mark. The
 * floor of the line reads 60 dBov. floor[0] is the floor after 31 windows,
 * floor[1] after 32 and floor[2] after 40. */
static void agree_falsify_run(unsigned rate, unsigned far, unsigned len,
			      int burst_dbov, float floor[3])
{
	struct agree_line l;
	float now = 127.0f;
	unsigned w;

	agree_line_init(&l, rate, 60);
	for (w = 1; w <= 40u; w++) {
		now = agree_pattern_run(&l, far, len, burst_dbov, 54, 1u);
		if (w == 31u)
			floor[0] = now;
		if (w == 32u)
			floor[1] = now;
	}
	floor[2] = now;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The falsification of the agreement run: LEC_NOISE_STALE_FRAMES active frames
 * in a row whose filter output stands more than LEC_NOISE_STALE_DB under the
 * run's level restart it. Bursts that miss a condition leave the run and the
 * floor rises; bursts that meet all three hold it.
 */
static void agree_falsify_test(unsigned rate)
{
	static const struct {
		unsigned far;
		unsigned len;
		int dbov;
		bool kept;
	} legs[5] = {
		{ 0x3ffu, 10u, 57, true },
		{ 0x3ffu, 10u, 58, false },
		{ 0x7u, 3u, 58, true },
		{ 0xfu, 4u, 58, false },
		{ 0x1bu, 5u, 58, true },
	};
	static const char *const names[5] = {
		"10 frames 3 dB under", "10 frames 4 dB under", "3 frames",
		"4 frames", "2, a gap and 2 frames",
	};
	unsigned i;

	case_rate(rate, "lec starts the agreement run over on far bursts "
		  "quieter than it");
	for (i = 0; i < 5u; i++) {
		float floor[3];

		agree_falsify_run(rate, legs[i].far, legs[i].len, legs[i].dbov,
				  floor);
		printf("# %u Hz, bursts of %s: floor %.2f, %.2f and %.2f dBov "
		       "after 31, 32 and 40 windows\n", rate, names[i],
		       (double)floor[0], (double)floor[1], (double)floor[2]);
		CHECK(floor[0] == 60.0f);
		CHECK(floor[1] == (legs[i].kept ? 55.0f : 60.0f));
		if (!legs[i].kept)
			CHECK(floor[2] == 60.0f);
	}
}

/*
 * The floor after a stale drop of a floor that an agreement run raised by
 * LEC_NOISE_AGREE_RISE_DB. Far bursts at the floor before the rise drop it, and
 * the first reading after each drop takes the dropped level. The floor
 * therefore stays within 8 dB of where it began. A first reading taken whole
 * fails here.
 */
static void agree_stale_test(unsigned rate)
{
	struct agree_line l;
	float raised;
	float floor = 127.0f;
	unsigned w;

	case_rate(rate, "lec takes no floor louder than the one that the stale "
		  "rule dropped");
	agree_line_init(&l, rate, 60);
	raised = agree_windows_run(&l, 40, 2u, 16u);
	for (w = 0; w < 10u; w++)
		floor = agree_window_run(&l, 60, 40, 2u);
	printf("# %u Hz: floor %.2f dBov after 16 windows at 40 dBov, %.2f dBov "
	       "after 10 more between bursts at 60 dBov\n", rate,
	       (double)raised, (double)floor);
	CHECK(raised == 52.0f);
	CHECK(floor == 52.0f);
	udsp_ec_nlms.destroy(l.ec);
}

/* The silent frames that agree_feed_test feeds after its far burst: more
 * than the 5 frames of the 48 ms window at either rate. */
#define AGREE_FEED_QUIET_FRAMES	8u

/*
 * A far burst that arrives through udsp_ec_nlms_ref_feed ends a quiet window.
 * After 9 windows of readings, a far burst and silent frames arrive through
 * the feed alone. The next reading of run() opens the 10th window and
 * completes the agreement run. A feed that leaves the quiet window open fails
 * here.
 */
static void agree_feed_test(unsigned rate)
{
	int16_t ref[UDSP_FRAME_MAX];
	struct agree_line l;
	float held;
	float floor;
	unsigned i;
	bool quiet;

	case_rate(rate, "lec opens a new quiet window after a far burst of the "
		  "reference feed");
	agree_line_init(&l, rate, 60);
	held = agree_windows_run(&l, 54, 4u, 9u);
	for (i = 0; i < AGREE_BURST_FRAMES + AGREE_FEED_QUIET_FRAMES; i++) {
		memset(ref, 0, sizeof(ref));
		if (i < AGREE_BURST_FRAMES)
			noise_fill(ref, l.frame, PAUSE_FAR_DBM0, &l.state);
		udsp_ec_nlms_ref_feed(l.ec, ref);
	}
	floor = exact_samples_run(l.ec, l.frame, 54, false, &l.state, &quiet);
	printf("# %u Hz: floor %.2f dBov after 9 windows of 4 readings, %.2f "
	       "dBov on the first reading after the fed burst\n", rate,
	       (double)held, (double)floor);
	CHECK(quiet);
	CHECK(held == 60.0f);
	CHECK(floor == 55.0f);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The rise bound of the agreement run, LEC_NOISE_AGREE_RISE_DB. Readings
 * 12 dB louder than a floor at 60 dBov complete a run. The floor rises 8 dB
 * to 52 dBov, and both runs start over. The next run takes the floor to the
 * readings plus the bias, 49 dBov. A run without the bound, or a bound that
 * leaves a run in place, fails here.
 */
static void agree_cap_test(unsigned rate)
{
	struct agree_line l;
	float held[2];
	float floor[2];

	case_rate(rate, "lec raises the floor by at most 8 dB per agreement "
		  "run");
	agree_line_init(&l, rate, 60);
	held[0] = agree_windows_run(&l, 48, 2u, 15u);
	floor[0] = agree_window_run(&l, 48, 48, 2u);
	held[1] = agree_windows_run(&l, 48, 2u, 15u);
	floor[1] = agree_window_run(&l, 48, 48, 2u);
	printf("# %u Hz: floor %.2f dBov after 15 windows, %.2f dBov after "
	       "16, %.2f dBov after 31, %.2f dBov after 32\n", rate,
	       (double)held[0], (double)floor[0], (double)held[1],
	       (double)floor[1]);
	CHECK(held[0] == 60.0f);
	CHECK(floor[0] == 52.0f);
	CHECK(held[1] == 52.0f);
	CHECK(floor[1] == 49.0f);
	udsp_ec_nlms.destroy(l.ec);
}

#define AGE_TAIL_FRAMES		4u

/* The floor after the 50th reading, distance frames after the first. The
 * pause before the second 25 readings starts with AGE_TAIL_FRAMES frames of
 * an active window. */
static float floor_age_run(unsigned distance)
{
	uint32_t state = 0x57721567u;
	struct udsp_ec *ec = exact_line_create(60, &state);
	unsigned tail;
	float floor;

	(void)exact_read_run(ec, 50, 25u, &state);
	exact_far_run(ec, 50, distance - 49u - AGE_TAIL_FRAMES, &state);
	floor = exact_quiet_run(ec, 50, 25u, &state, &tail);
	CHECK_UINT(tail, AGE_TAIL_FRAMES);
	udsp_ec_nlms.destroy(ec);
	return floor;
}

/*
 * The age limit of a rise run, LEC_NOISE_RISE_AGE, 500 frames. A floor at
 * 60 dBov reads 25 frames at 50 dBov, then far-end frames and a pause, then
 * 25 more readings. The run takes the 50th reading 500 frames after the
 * first, and sets the floor to 50. At 501 frames the run starts again, and
 * the floor holds. A run without the limit, or with a shorter one, fails.
 */
static void floor_age_test(void)
{
	float floor;

	CASE("lec keeps a rise run whose first reading is 500 frames old");
	floor = floor_age_run(500u);
	printf("# floor %.2f dBov at 500 frames\n", (double)floor);
	CHECK(floor == 50.0f);

	CASE("lec starts a rise run again when its first reading is 501 "
	     "frames old");
	floor = floor_age_run(501u);
	printf("# floor %.2f dBov at 501 frames\n", (double)floor);
	CHECK(floor == 60.0f);
}

/*
 * The reset and the rise run. A floor at 60 dBov reads 30 frames at 50 dBov,
 * and the reset follows. The reset keeps the floor. Another 30 readings
 * leave it at 60, and 20 after them complete a run of 50 and set it to 50. A
 * reset that keeps the run sets the floor 20 readings after the reset.
 */
static void floor_reset_test(void)
{
	uint32_t state = 0x14142136u;
	struct udsp_ec_nlms_info info;
	struct udsp_ec *ec;
	float held;

	CASE("lec reset starts the rise run again");
	ec = exact_line_create(60, &state);
	(void)exact_read_run(ec, 50, 30u, &state);
	udsp_ec_nlms_reset(ec);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK(info.noise_dbov == 60.0f);
	held = exact_read_run(ec, 50, 30u, &state);
	printf("# floor %.2f dBov 30 readings after the reset\n",
	       (double)held);
	CHECK(held == 60.0f);
	CHECK(exact_read_run(ec, 50, 20u, &state) == 50.0f);
	udsp_ec_nlms.destroy(ec);
}

#define FAN_DBM0		-38.0
#define FAN_WORDS_DBM0		-20.0
#define FAN_FLOOR_DBM0		-72.0
#define FAN_ERL_DB		24.0
#define FAN_FRAMES		(30u * FRAMES_PER_SECOND)
#define FAN_SEED_BOUND		(10u * FRAMES_PER_SECOND)
#define FAN_FLOOR_TOL_DB	1.5
#define FAN_DT_SHARE_MAX	0.10
#define FAN_LOWPASS_HZ		300.0

/*
 * The near speech of a line over the frames that hold it, where the near plane
 * alone crosses the activity bar. It holds the energies of the regression gain
 * and the exposure frames, whose echo estimate stands at or over the near power
 * less 10 dB.
 */
struct near_stat {
	unsigned frames;
	unsigned exposure;
	double near;
	double product;
	double mic;
	double out;
};

static void near_stat_add(struct near_stat *st, const struct udsp_ec *ec,
			  const int16_t *near, const int16_t *mic,
			  const int16_t *out, unsigned frame)
{
	struct udsp_ec_nlms_frame reading;
	unsigned i;

	if (!frame_loud(near, frame))
		return;
	udsp_ec_nlms_frame_get(ec, &reading);
	st->frames++;
	if (reading.estimate_power >=
	    udsp_ec_nlms_frame_power_get(near, frame) / 10.0f)
		st->exposure++;
	for (i = 0; i < frame; i++) {
		st->near += (double)near[i] * near[i];
		st->product += (double)out[i] * near[i];
		st->mic += (double)mic[i] * mic[i];
		st->out += (double)out[i] * out[i];
	}
}

/*
 * The regression gain of an output on the near speech it carries, in dB. 0
 * dB means that the output does not scale the speech, and a line with no near
 * energy reads 0 dB. An output that carries none of the near speech, or its
 * opposite, reads the sentinel -200 dB, so a bound on the loss fails there.
 */
static double near_gain_db_get(double product, double near)
{
	if (!(near > 0.0))
		return 0.0;
	if (!(product > 0.0))
		return -200.0;
	return 20.0 * log10(product / near);
}

/*
 * The fan line: a flat 24 dB hybrid at 8 ms with the processor off. A near
 * floor at -72 dBm0 lies under a far fan at -38 dBm0 that never pauses. Its
 * echo stands over the -65 dBm0 cap, so the fan frames stay active, 10 dB over
 * the floor. Far words go on top; the fields count the filter ERLE, the near
 * speech and the output of the last frame.
 */
struct fan_line {
	struct echo_path path;
	struct word_source far;
	struct udsp_ec *ec;
	unsigned frame;
	uint32_t fan_state;
	uint32_t floor_state;
	double fan_dbm0;
	double erl_db;
	double floor_dbm0;
	double pole;
	double gain;
	double memory;
	bool measure;
	bool measure_words;
	bool word_loud;
	double echo_energy;
	double filter_energy;
	struct near_stat speech;
	int16_t out[UDSP_FRAME_MAX];
};

/* A first-order low-pass fan at lowpass_hz when it is over 0, with the gain
 * that keeps its level at fan_dbm0. */
static void fan_line_init(struct fan_line *line, unsigned rate,
			  double lowpass_hz)
{
	memset(line, 0, sizeof(*line));
	line->frame = rate / 100u;
	line->ec = udsp_ec_nlms.create_rate(0, rate);
	udsp_ec_nlms.nlp(line->ec, false);
	line->fan_dbm0 = FAN_DBM0;
	line->erl_db = FAN_ERL_DB;
	path_flat_init(&line->path, rate, 8, line->erl_db);
	word_source_init(&line->far, rate, FAN_WORDS_DBM0, 0x13572468u);
	line->fan_state = 0x0f00f00du;
	line->floor_state = 0x2468acefu;
	line->floor_dbm0 = FAN_FLOOR_DBM0;
	line->gain = 1.0;
	if (lowpass_hz > 0.0) {
		line->pole = exp(-2.0 * M_PI * lowpass_hz / rate);
		line->gain = sqrt((1.0 + line->pole) / (1.0 - line->pole));
	}
}

/* Another fan level and another hybrid, before the line runs. */
static void fan_line_set(struct fan_line *line, double fan_dbm0, double erl_db)
{
	line->fan_dbm0 = fan_dbm0;
	line->erl_db = erl_db;
	path_erl_set(&line->path, erl_db);
}

static void fan_fill(struct fan_line *line, int16_t *dst)
{
	unsigned i;

	noise_fill(dst, line->frame, line->fan_dbm0, &line->fan_state);
	if (line->pole == 0.0)
		return;
	for (i = 0; i < line->frame; i++) {
		line->memory = line->pole * line->memory +
			       (1.0 - line->pole) * dst[i];
		dst[i] = sample_clamp(line->memory * line->gain);
	}
}

/* One frame of the fan line, with near on top when it is given. It returns
 * the state after the frame. */
static struct udsp_ec_nlms_info fan_frame_run(struct fan_line *line, bool fan,
					      bool words, const int16_t *near)
{
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t part[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t in[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	memset(ref, 0, sizeof(ref));
	if (fan)
		fan_fill(line, ref);
	if (words) {
		word_fill(&line->far, part, line->frame);
		for (i = 0; i < line->frame; i++)
			ref[i] = sample_clamp((double)ref[i] + part[i]);
	}
	path_run(&line->path, ref, echo);
	noise_fill(hiss, line->frame, line->floor_dbm0, &line->floor_state);
	for (i = 0; i < line->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + hiss[i] +
				      (near ? near[i] : 0));
	memcpy(in, mic, line->frame * sizeof(*mic));
	udsp_ec_nlms.run(line->ec, mic, ref);
	memcpy(line->out, mic, line->frame * sizeof(*mic));
	udsp_ec_nlms_info_get(line->ec, &info);
	if (near)
		near_stat_add(&line->speech, line->ec, near, in, mic,
			      line->frame);
	line->word_loud = words && frame_loud(part, line->frame);
	if (!line->measure || near)
		return info;
	if (!(line->measure_words ? line->word_loud :
				    frame_loud(ref, line->frame)))
		return info;
	for (i = 0; i < line->frame; i++) {
		double d = (double)mic[i] - hiss[i];

		line->echo_energy += (double)echo[i] * echo[i];
		line->filter_energy += d * d;
	}
	return info;
}

static double fan_filter_erle_get(const struct fan_line *line)
{
	if (!(line->filter_energy > 0.0))
		return 200.0;
	return 10.0 * log10(line->echo_energy / line->filter_energy);
}

/* True when the frame just run carried a window-active double talk
 * verdict or hangover. */
static bool fan_dt_get(const struct fan_line *line,
		       const struct udsp_ec_nlms_info *info)
{
	struct udsp_ec_nlms_frame reading;

	udsp_ec_nlms_frame_get(line->ec, &reading);
	return reading.window_run && info->double_talk;
}

static bool fan_window_get(const struct fan_line *line)
{
	struct udsp_ec_nlms_frame reading;

	udsp_ec_nlms_frame_get(line->ec, &reading);
	return reading.window_run != 0u;
}

/*
 * A far end that never pauses: over 30 s the seed measures the floor within
 * FAN_SEED_BOUND, within 1.5 dB of -72 dBm0, on an active window. After 10 s
 * the window stays active with no trial or reset and double talk on under 10
 * percent of frames. Without the seed, the line reads no floor.
 */
static void fan_seed_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info at;
	struct fan_line line;
	double floor = floor_dbov_get(FAN_FLOOR_DBM0);
	float seed = 127.0f;
	unsigned seeded = 0;
	unsigned window = 0;
	unsigned dt = 0;
	bool seed_window = false;
	unsigned f;

	fan_line_init(&line, rate, 0.0);
	memset(&at, 0, sizeof(at));
	for (f = 0; f < FAN_FRAMES; f++) {
		info = fan_frame_run(&line, true, true, NULL);
		if (!seeded && info.noise_dbov < 127.0f) {
			seeded = f + 1u;
			seed = info.noise_dbov;
			seed_window = fan_window_get(&line);
		}
		if (f + 1u == FAN_SEED_BOUND)
			at = info;
		if (f < FAN_SEED_BOUND || !fan_window_get(&line))
			continue;
		window++;
		if (fan_dt_get(&line, &info))
			dt++;
	}

	case_rate(rate, "lec measures the near-end floor under a far end that "
		  "never pauses");
	printf("# seed %.2f dBov at %u ms against %.2f dBov, window active %u, "
	       "floor %.2f dBov at the end; after %u ms: %u trials, %u resets, "
	       "double talk on %u of %u window-active frames\n", (double)seed,
	       10u * seeded, floor, seed_window, (double)info.noise_dbov,
	       10u * FAN_SEED_BOUND, info.probes - at.probes,
	       info.diverge_resets - at.diverge_resets, dt, window);
	CHECK(seeded > 0u && seeded <= FAN_SEED_BOUND);
	CHECK(seed_window);
	CHECK(fabs(at.noise_dbov - floor) <= FAN_FLOOR_TOL_DB);
	CHECK(fabs(info.noise_dbov - floor) <= FAN_FLOOR_TOL_DB);
	CHECK_UINT(info.probes - at.probes, 0);
	CHECK_UINT(info.diverge_resets - at.diverge_resets, 0);
	CHECK_UINT(window, FAN_FRAMES - FAN_SEED_BOUND);
	CHECK(dt < FAN_DT_SHARE_MAX * window);
	udsp_ec_nlms.destroy(line.ec);
}

#define TURNS_CYCLE		500u
#define TURNS_FRAMES		300u
#define TURNS_NEAR_START	250u
#define TURNS_NEAR_DBM0		-20.0
#define TURNS_SEED_BOUND	(15u * FRAMES_PER_SECOND)
/* The fan of fan_turns_test with the far-end floor off: at FAN_DBM0 the seed
 * comes before the first near turn, so the line tests no seed under the
 * turns. */
#define TURNS_FAN_DBM0		-48.0

/*
 * The fan at -48 dBm0 with the far-end floor off, at 16 kHz. Far words and a
 * near talker at -20 dBm0 take turns of 3 s in 5 s. The seed comes within
 * 15 s, the floor ends within 1.5 dB of -72 dBm0, and no trial or reset runs.
 * The talker guard clears the latch run before the seed.
 */
static void fan_turns_test(void)
{
	int16_t near[UDSP_FRAME_MAX];
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info at;
	struct word_source talker;
	struct fan_line line;
	double floor = floor_dbov_get(FAN_FLOOR_DBM0);
	unsigned seeded = 0;
	unsigned f;

	fan_line_init(&line, UDSP_SAMPLE_RATE_WB, 0.0);
	udsp_ec_nlms_ref_floor_set(line.ec, false);
	fan_line_set(&line, TURNS_FAN_DBM0, FAN_ERL_DB);
	word_source_init(&talker, UDSP_SAMPLE_RATE_WB, TURNS_NEAR_DBM0,
			 0x0badcafeu);
	memset(&at, 0, sizeof(at));
	for (f = 0; f < FAN_FRAMES; f++) {
		unsigned p = f % TURNS_CYCLE;
		bool far = p < TURNS_FRAMES;
		bool talk = p >= TURNS_NEAR_START ||
			    (f >= TURNS_CYCLE &&
			     p < TURNS_NEAR_START + TURNS_FRAMES - TURNS_CYCLE);

		if (p == 0u)
			word_source_restart(&line.far);
		if (p == TURNS_NEAR_START)
			word_source_restart(&talker);
		if (talk)
			word_fill(&talker, near, line.frame);
		info = fan_frame_run(&line, true, far, talk ? near : NULL);
		if (!seeded && info.noise_dbov < 127.0f) {
			seeded = f + 1u;
			at = info;
		}
	}

	CASE("lec runs no trial for a talker in turns under a far end that "
	     "never pauses");
	printf("# seed at %u ms, %u trials and %u resets before it, %u and %u "
	       "after it, floor %.2f dBov against %.2f dBov\n", 10u * seeded,
	       at.probes, at.diverge_resets, info.probes - at.probes,
	       info.diverge_resets - at.diverge_resets,
	       (double)info.noise_dbov, floor);
	CHECK(seeded > 0u && seeded <= TURNS_SEED_BOUND);
	CHECK_UINT(at.probes, 0);
	CHECK_UINT(at.diverge_resets, 0);
	CHECK_UINT(info.probes - at.probes, 0);
	CHECK_UINT(info.diverge_resets - at.diverge_resets, 0);
	CHECK(fabs(info.noise_dbov - floor) <= FAN_FLOOR_TOL_DB);
	udsp_ec_nlms.destroy(line.ec);
}

#define MEASURED_PAUSE_FRAMES	100u
#define MEASURED_NOISE_FROM	500u
#define MEASURED_FRAMES		1500u
#define MEASURED_NOISE_DBM0	-60.0

/*
 * A measured floor under a far end that never pauses: a pause measures -72
 * dBm0, then the fan and the words run. From 5 s a near noise at -60 dBm0
 * joins. The floor stays within 1.5 dB of -72 dBm0; a seed that also runs on a
 * measured floor takes the noise in.
 */
static void fan_measured_test(void)
{
	int16_t noise[UDSP_FRAME_SAMPLES];
	struct udsp_ec_nlms_info info;
	struct fan_line line;
	double floor = floor_dbov_get(FAN_FLOOR_DBM0);
	uint32_t noise_state = 0x600dcafeu;
	double worst = 0.0;
	float start = 127.0f;
	unsigned f;

	fan_line_init(&line, UDSP_SAMPLE_RATE, 0.0);
	for (f = 0; f < MEASURED_PAUSE_FRAMES; f++)
		info = fan_frame_run(&line, false, false, NULL);
	start = info.noise_dbov;
	for (f = 0; f < MEASURED_FRAMES; f++) {
		bool noisy = f >= MEASURED_NOISE_FROM;

		if (noisy)
			noise_fill(noise, UDSP_FRAME_SAMPLES,
				   MEASURED_NOISE_DBM0, &noise_state);
		info = fan_frame_run(&line, true, true, noisy ? noise : NULL);
		if (fabs(info.noise_dbov - floor) > worst)
			worst = fabs(info.noise_dbov - floor);
	}

	CASE("lec never raises a measured floor from an active frame");
	printf("# floor %.2f dBov after the pause against %.2f dBov, %.2f dBov "
	       "at the end, largest distance %.2f dB\n", (double)start, floor,
	       (double)info.noise_dbov, worst);
	CHECK(fabs(start - floor) <= FAN_FLOOR_TOL_DB);
	CHECK(worst <= FAN_FLOOR_TOL_DB);
	udsp_ec_nlms.destroy(line.ec);
}

#define RESEED_FLOOR_DBM0	-82.0
#define RESEED_FRAMES		(2u * FRAMES_PER_SECOND)

/*
 * A seeded floor that the filter output falsifies. The fan line at 8 kHz runs
 * until the seed and then 1 s more. The near-end floor then drops to -82 dBm0,
 * and the stale rule drops the seeded floor. Within 2 s the seed must measure
 * the lower floor within 1.5 dB, on a frame of an active window. A seed that
 * opens once per instance leaves the line without a floor.
 */
static void fan_reseed_test(void)
{
	struct udsp_ec_nlms_info info;
	struct fan_line line;
	double low = floor_dbov_get(RESEED_FLOOR_DBM0);
	unsigned seeded = 0;
	unsigned found = 0;
	unsigned dropped = 0;
	unsigned reseeded = 0;
	bool reseed_window = false;
	float before;
	unsigned f;

	fan_line_init(&line, UDSP_SAMPLE_RATE, 0.0);
	for (f = 0; f < FAN_FRAMES && (!seeded || f < seeded + 100u); f++) {
		info = fan_frame_run(&line, true, true, NULL);
		if (!seeded && info.noise_dbov < 127.0f)
			seeded = f + 1u;
	}
	before = info.noise_dbov;
	line.floor_dbm0 = RESEED_FLOOR_DBM0;
	for (f = 0; f < RESEED_FRAMES; f++) {
		info = fan_frame_run(&line, true, true, NULL);
		if (!dropped && info.noise_dbov >= 127.0f)
			dropped = f + 1u;
		if (dropped && !reseeded && info.noise_dbov < 127.0f) {
			reseeded = f + 1u;
			reseed_window = fan_window_get(&line);
		}
		if (!found && fabs(info.noise_dbov - low) <= FAN_FLOOR_TOL_DB)
			found = f + 1u;
	}

	CASE("lec re-seeds a floor that the residual falsified");
	printf("# seed at %u ms, floor %.2f dBov before the drop; dropped "
	       "after %u ms, re-seeded after %u ms with the window active %u, "
	       "%.2f dBov after 2 s against %.2f dBov, within 1.5 dB after %u "
	       "ms\n", 10u * seeded, (double)before, 10u * dropped,
	       10u * reseeded, reseed_window, (double)info.noise_dbov, low,
	       10u * found);
	CHECK(seeded > 0u);
	CHECK(dropped > 0u);
	CHECK(reseed_window);
	CHECK(found > 0u);
	CHECK(fabs(info.noise_dbov - low) <= FAN_FLOOR_TOL_DB);
	udsp_ec_nlms.destroy(line.ec);
}

/* A fan low-passed at 300 Hz at -38 dBm0: the far words teach the band under
 * 300 Hz. The seed then reads the floor within 1.5 dB of -72 dBm0 on an active
 * window, as under a white fan. Gate (c) does not change the result. */
static void fan_lowpass_test(void)
{
	struct udsp_ec_nlms_info info;
	struct fan_line line;
	double floor = floor_dbov_get(FAN_FLOOR_DBM0);
	float seed = 127.0f;
	unsigned seeded = 0;
	bool seed_window = false;
	unsigned f;

	fan_line_init(&line, UDSP_SAMPLE_RATE, FAN_LOWPASS_HZ);
	for (f = 0; f < FAN_FRAMES; f++) {
		info = fan_frame_run(&line, true, true, NULL);
		if (!seeded && info.noise_dbov < 127.0f) {
			seeded = f + 1u;
			seed = info.noise_dbov;
			seed_window = fan_window_get(&line);
		}
	}

	CASE("lec keeps the echo of a fan outside the learned band out of the "
	     "seed");
	printf("# seed %.2f dBov at %u ms against %.2f dBov, window active %u, "
	       "%.2f dBov at the end\n", (double)seed, 10u * seeded, floor,
	       seed_window, (double)info.noise_dbov);
	CHECK(seeded > 0u);
	CHECK(seed_window);
	CHECK(fabs(seed - floor) <= FAN_FLOOR_TOL_DB);
	CHECK(fabs(info.noise_dbov - floor) <= FAN_FLOOR_TOL_DB);
	udsp_ec_nlms.destroy(line.ec);
}

#define STEP_MEASURE_FROM	(10u * FRAMES_PER_SECOND)
/* The filter ERLE of a canceller without the seed on the fan line. */
#define STEP_HEAD_ERLE_DB	23.9
#define STEP_ERLE_TOL_DB	1.0

/*
 * A seeded floor and the step control: over the last 20 s of 30 the filter ERLE
 * comes within 1 dB of a canceller without the seed. A step that a measured
 * floor freezes fails; a seed louder than the floor closes the step on fan-only
 * frames, and the ERLE rises.
 */
static void fan_step_test(void)
{
	struct udsp_ec_nlms_info info;
	struct fan_line line;
	unsigned f;

	fan_line_init(&line, UDSP_SAMPLE_RATE, 0.0);
	for (f = 0; f < FAN_FRAMES; f++) {
		line.measure = f >= STEP_MEASURE_FROM;
		info = fan_frame_run(&line, true, true, NULL);
	}

	CASE("lec keeps a seeded floor from freezing the step");
	printf("# filter ERLE %.2f dB over the last 20 s against %.2f dB, "
	       "floor %.2f dBov\n", fan_filter_erle_get(&line),
	       STEP_HEAD_ERLE_DB, (double)info.noise_dbov);
	CHECK(fan_filter_erle_get(&line) >= STEP_HEAD_ERLE_DB -
	      STEP_ERLE_TOL_DB);
	udsp_ec_nlms.destroy(line.ec);
}

#define BLOCK_NEAR_DBM0		-19.0

/*
 * Gate (f) on a line without a floor: after convergence, a steady white talker
 * 3 dB over the echo, and in a second instance a hybrid 6 dB louder. The ERLE
 * passes gates (b) to (e) in both, and the floor stays unmeasured. Without gate
 * (f) the seed takes the talker or the misaligned echo in.
 */
static void seed_block_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x0badc0deu;
	uint32_t near_state = 0xfeedfaceu;
	float loudest = 127.0f;
	unsigned f;

	ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, path.frame, -10.0, &state);
		path_run(&path, ref, echo);
		noise_fill(near, path.frame, BLOCK_NEAR_DBM0, &near_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.noise_dbov < loudest)
			loudest = info.noise_dbov;
	}
	udsp_ec_nlms.destroy(ec);

	case_rate(rate, "lec keeps a steady near talker out of the seed");
	printf("# floor %.2f dBov through 1 s of a talker 3 dB over the echo\n",
	       (double)loudest);
	CHECK(loudest == 127.0f);

	ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, 12.0);
	(void)erle_run(ec, &path, &state, -10.0,
		       FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2, 20);
	path_erl_set(&path, 6.0);
	loudest = 127.0f;
	for (f = 0; f < 3u * FRAMES_PER_SECOND; f++) {
		(void)erle_run(ec, &path, &state, -10.0, 1u, 1u);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.noise_dbov < loudest)
			loudest = info.noise_dbov;
	}

	case_rate(rate, "lec keeps the echo of a changed hybrid out of the "
		  "seed");
	printf("# floor %.2f dBov through 3 s after the hybrid got 6 dB "
	       "louder, %u trials\n", (double)loudest, info.probes);
	CHECK(loudest == 127.0f);
	udsp_ec_nlms.destroy(ec);
}

#define TRIAL_SEED_LOUD_DBM0	-20.0
#define TRIAL_SEED_TALK_DBM0	-10.0
#define TRIAL_SEED_CONVERGE	300u
#define TRIAL_SEED_RUN_AT	485u
#define TRIAL_SEED_LIMIT	1000u

/* One 8 kHz frame of the trial line: white far end at far_dbm0, the floor,
 * and a white talker at talk_dbm0 when talk is set. */
static void seed_trial_frame_run(struct udsp_ec *ec, struct echo_path *path,
				 double far_dbm0, bool talk, uint32_t *state,
				 uint32_t *floor_state)
{
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t echo[UDSP_FRAME_SAMPLES];
	int16_t near[UDSP_FRAME_SAMPLES];
	int16_t mic[UDSP_FRAME_SAMPLES];
	unsigned i;

	noise_fill(ref, UDSP_FRAME_SAMPLES, far_dbm0, state);
	path_run(path, ref, echo);
	floor_add(mic, echo, FAN_FLOOR_DBM0, floor_state);
	if (talk) {
		noise_fill(near, UDSP_FRAME_SAMPLES, TRIAL_SEED_TALK_DBM0,
			   state);
		for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
			mic[i] = sample_clamp((double)mic[i] + near[i]);
	}
	udsp_ec_nlms.run(ec, mic, ref);
}

/*
 * A flat 24 dB hybrid with a floor at -72 dBm0 converges on a far end at
 * -20 dBm0. Gate (c) keeps the seed shut there. A talker then builds the latch
 * run, and at the fan level the latch starts a trial before the seed run is
 * full. The trial frames clear the run, so the seed comes 20 frames after it.
 */
static void seed_trial_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x31415926u;
	uint32_t floor_state = 0x2468acefu;
	unsigned before = 0;
	unsigned trial_end = 0;
	unsigned seeded = 0;
	float floor_at_start = 0.0f;
	bool started = false;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, FAN_ERL_DB);
	for (f = 0; f < TRIAL_SEED_CONVERGE; f++)
		seed_trial_frame_run(ec, &path, TRIAL_SEED_LOUD_DBM0, false,
				     &state, &floor_state);
	for (f = 0; f < TRIAL_SEED_LIMIT; f++) {
		seed_trial_frame_run(ec, &path, TRIAL_SEED_LOUD_DBM0, true,
				     &state, &floor_state);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (reading.dt_run >= TRIAL_SEED_RUN_AT)
			break;
	}
	for (f = 0; f < TRIAL_SEED_LIMIT && !seeded; f++) {
		seed_trial_frame_run(ec, &path, FAN_DBM0, false, &state,
				     &floor_state);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (!info.probes)
			before++;
		else if (!before || info.probes > 1u)
			break;
		if (info.probes && !started) {
			started = true;
			floor_at_start = info.noise_dbov;
		}
		if (reading.probing)
			trial_end = f + 1u;
		if (info.noise_dbov < 127.0f)
			seeded = f + 1u;
	}
	udsp_ec_nlms_info_get(ec, &info);

	CASE("lec starts the seed run again after a trial");
	printf("# %u fan frames before the trial, floor %.2f dBov as it "
	       "started, trial ended at frame %u, seed at frame %u, %u "
	       "trials\n", before, (double)floor_at_start, trial_end, seeded,
	       info.probes);
	CHECK_UINT(info.probes, 1);
	CHECK(floor_at_start == 127.0f);
	CHECK(trial_end > before);
	CHECK_UINT(seeded - trial_end + 1u, 20);
	udsp_ec_nlms.destroy(ec);
}

/* LEC_GEIGEL_LATCH_MAX of dsp/lec.c, and the latch run at which the talk of
 * the snapshot case stops, short of it by less than one hangover. */
#define SNAP_LATCH_FRAMES	500u
#define SNAP_TALK_RUN		481u
#define SNAP_LIMIT_FRAMES	(10u * FRAMES_PER_SECOND)
#define SNAP_COMPARE_FRAMES	20u
#define SNAP_FAR_DBM0		-10.0
/* The talker under the far end: 3 dB over the echo of the 12 dB hybrid. */
#define SNAP_TALK_UNDER_DB	9.0

/*
 * The snapshot line: two instances that take the same frames on a flat 12 dB
 * hybrid at 8 kHz, under a white far end at far_dbm0. near holds the talker
 * of the next frame, and the two outputs hold the outputs of the last frame.
 */
struct snap_line {
	struct echo_path path;
	struct udsp_ec *lead;
	struct udsp_ec *twin;
	double far_dbm0;
	uint32_t state;
	uint32_t near_state;
	unsigned run_before;
	bool clear_before;
	int16_t near[UDSP_FRAME_SAMPLES];
	int16_t lead_out[UDSP_FRAME_SAMPLES];
	int16_t twin_out[UDSP_FRAME_SAMPLES];
};

/* One frame of the far end to both instances, with near on top when it is
 * given. */
static void snap_frame_run(struct snap_line *s, const int16_t *near)
{
	int16_t ref[UDSP_FRAME_SAMPLES];

	noise_fill(ref, UDSP_FRAME_SAMPLES, s->far_dbm0, &s->state);
	twin_step(s->lead, s->twin, &s->path, ref, near, s->lead_out,
		  s->twin_out, NULL);
}

/* A white talker at talk_dbm0 for the next frame. */
static const int16_t *snap_talk_get(struct snap_line *s, double talk_dbm0)
{
	noise_fill(s->near, UDSP_FRAME_SAMPLES, talk_dbm0, &s->near_state);
	return s->near;
}

/*
 * The run up to a latch that is due. After 1.5 s of far end, a near talker
 * 3 dB over the echo talks until the latch run nears its limit. A far-end
 * pause then lets the hangover run out, so the latch comes due on the next
 * frame of the far end. clear_before and run_before hold the double talk and
 * the latch run at the end of the pause.
 */
static void snap_line_build(struct snap_line *s, double far_dbm0)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_SAMPLES];
	unsigned f;

	memset(s, 0, sizeof(*s));
	memset(&info, 0, sizeof(info));
	s->lead = udsp_ec_nlms.create(48);
	s->twin = udsp_ec_nlms.create(48);
	s->far_dbm0 = far_dbm0;
	s->state = 0x0badc0deu;
	s->near_state = 0xfeedfaceu;
	udsp_ec_nlms.nlp(s->lead, false);
	udsp_ec_nlms.nlp(s->twin, false);
	path_flat_init(&s->path, UDSP_SAMPLE_RATE, 8, 12.0);
	for (f = 0; f < FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2; f++)
		snap_frame_run(s, NULL);
	for (f = 0; f < SNAP_LIMIT_FRAMES; f++) {
		snap_frame_run(s, snap_talk_get(s, far_dbm0 -
						SNAP_TALK_UNDER_DB));
		udsp_ec_nlms_frame_get(s->lead, &reading);
		if (reading.dt_fired && reading.dt_run >= SNAP_TALK_RUN)
			break;
	}
	memset(ref, 0, sizeof(ref));
	for (f = 0; f < SNAP_LIMIT_FRAMES; f++) {
		twin_step(s->lead, s->twin, &s->path, ref, NULL, s->lead_out,
			  s->twin_out, NULL);
		udsp_ec_nlms_info_get(s->lead, &info);
		if (!info.double_talk)
			break;
	}
	udsp_ec_nlms_frame_get(s->lead, &reading);
	s->clear_before = !info.double_talk;
	s->run_before = reading.dt_run;
}

/*
 * The snapshot of a trial that starts on a frame that adapted and fired: the
 * talker refuses it. Both frozen filters then match to the bit, as does the
 * energy that the start frame reports. On the line of bare_snapshot_run, the
 * trial on taps without echo evidence passes trial_bare_test.
 */
static void probe_snapshot_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info lead_info;
	struct udsp_ec_nlms_info twin_info;
	struct bare_figures bare;
	struct snap_line s;
	float bare_before;
	float bare_start;
	unsigned differ = 0;
	unsigned f;

	bare_snapshot_run(UDSP_SAMPLE_RATE, &bare, &bare_before, &bare_start);
	snap_line_build(&s, SNAP_FAR_DBM0);
	udsp_ec_nlms.freeze(s.twin, true);
	CASE("lec trial restores the filter that its start frame ended with");
	for (f = 0; f <= TRIAL_BUDGET; f++) {
		snap_frame_run(&s, snap_talk_get(&s, SNAP_FAR_DBM0 -
						 SNAP_TALK_UNDER_DB));
		if (f)
			continue;
		udsp_ec_nlms_info_get(s.lead, &lead_info);
		udsp_ec_nlms_info_get(s.twin, &twin_info);
		udsp_ec_nlms_frame_get(s.lead, &reading);
		printf("# start frame: coefficient energy %.9f against the "
		       "twin's %.9f\n", (double)lead_info.coef_energy,
		       (double)twin_info.coef_energy);
		CHECK(s.clear_before);
		CHECK(s.run_before >= SNAP_LATCH_FRAMES);
		CHECK_UINT(lead_info.probes, 1);
		CHECK(reading.dt_fired);
		CHECK(lead_info.coef_energy == twin_info.coef_energy);
	}
	udsp_ec_nlms.freeze(s.lead, true);
	for (f = 0; f < SNAP_COMPARE_FRAMES; f++) {
		snap_frame_run(&s, NULL);
		if (memcmp(s.lead_out, s.twin_out, sizeof(s.lead_out)))
			differ++;
	}
	udsp_ec_nlms_info_get(s.lead, &lead_info);
	udsp_ec_nlms_info_get(s.twin, &twin_info);
	printf("# latch run %u after the pause; coefficient energy %.9f "
	       "against the twin's %.9f; %u of %u frames differ\n",
	       s.run_before, (double)lead_info.coef_energy,
	       (double)twin_info.coef_energy, differ, SNAP_COMPARE_FRAMES);
	CHECK_UINT(lead_info.probes, 1);
	CHECK_UINT(lead_info.diverge_resets, 0);
	CHECK(lead_info.coef_energy == twin_info.coef_energy);
	CHECK_UINT(differ, 0);
	udsp_ec_nlms.destroy(s.lead);
	udsp_ec_nlms.destroy(s.twin);
	printf("# taps without echo evidence: trial arm %u from frame %u, "
	       "coefficient energy %.6f before its start frame and %.6f after "
	       "it; end %u on frame %u at %.3f, %.2f of the echo; ERLE %.1f dB "
	       "after the far end alone\n", bare.arm, bare.start,
	       (double)bare_before, (double)bare_start, bare.how, bare.end,
	       bare.keep_coef, bare.keep_coef / bare.echo_coef, bare.erle_db);
	CHECK_UINT(bare.arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK(bare_start == bare_before);
	CHECK_UINT(bare.how, UDSP_EC_NLMS_TRIAL_END_KEPT);
	CHECK(bare.keep_coef <= BARE_COEF_RATIO * bare.echo_coef);
	CHECK(bare.erle_db >= BARE_ERLE_DB);
}

#define RING_DBM0		-20.0
#define RING_ON_FRAMES		200u
#define RING_CYCLE_FRAMES	600u
#define RING_FRAMES		(2u * RING_CYCLE_FRAMES)
#define RING_ERL_DB		24.0
#define RING_FLOOR_DBM0		-70.0
#define RING_WORDS_DBM0		-20.0
#define RING_SPEECH_FRAMES	(20u * FRAMES_PER_SECOND)
#define RING_FIRST_FRAMES	(2u * FRAMES_PER_SECOND)
#define RING_LAST_FRAMES	(8u * FRAMES_PER_SECOND)
/* LEC_GEIGEL_TRUST_DB and LEC_GEIGEL_TRUST_COEF_RATIO of dsp/lec.c. */
#define RING_TRUST_DB		18.0f
#define RING_COEF_RATIO		10.0f

/*
 * The ringback line: a flat 24 dB hybrid at 8 ms, the default tail, the
 * processor off and a near-end floor at -70 dBm0. The ringback is 440 plus
 * 480 Hz at -20 dBm0 each, 2 s on and 4 s off, for two cycles. Far-end words
 * at -20 dBm0 follow it. A line of the lockout cases (struct ring_spec) sets
 * another floor level, and clears floor on the frames before its floor.
 */
struct ring_line {
	struct echo_path path;
	struct word_source far;
	struct udsp_ec *ec;
	unsigned rate;
	unsigned frame;
	unsigned long phase;
	uint32_t far_state;
	uint32_t near_state;
	uint32_t floor_state;
	uint32_t held;
	double floor_dbm0;
	bool floor;
};

/*
 * The frames of one stretch whose own reference is active: those whose
 * update did not stand, those in double talk, and those that fired on a bar
 * that the estimate set. The energies of the same frames give the filter
 * ERLE (weak_filter_erle_get).
 */
struct ring_stat {
	unsigned active;
	unsigned held;
	unsigned dt;
	unsigned est_fired;
	double echo;
	double residual;
	double floor;
};

static void ring_line_init(struct ring_line *line, unsigned rate)
{
	memset(line, 0, sizeof(*line));
	line->rate = rate;
	line->frame = rate / 100u;
	line->ec = udsp_ec_nlms.create_rate(0, rate);
	udsp_ec_nlms.nlp(line->ec, false);
	path_flat_init(&line->path, rate, 8, RING_ERL_DB);
	word_source_init(&line->far, rate, RING_WORDS_DBM0, 0x13572468u);
	line->far_state = 0x0badc0deu;
	line->near_state = 0xfeedfaceu;
	line->floor_state = 0x2468acefu;
	line->floor_dbm0 = RING_FLOOR_DBM0;
	line->floor = true;
}

/* The two tones of the ringback, at RING_DBM0 each. */
static void ring_tones_fill(struct ring_line *line, int16_t *ref)
{
	int16_t high[UDSP_FRAME_MAX];
	unsigned long phase = line->phase;
	unsigned i;

	tone_fill(ref, line->frame, RING_DBM0, 440.0, &line->phase,
		  line->rate);
	tone_fill(high, line->frame, RING_DBM0, 480.0, &phase, line->rate);
	for (i = 0; i < line->frame; i++)
		ref[i] = sample_clamp((double)ref[i] + high[i]);
}

static void ring_fill(struct ring_line *line, int16_t *ref, unsigned f)
{
	if (f % RING_CYCLE_FRAMES >= RING_ON_FRAMES) {
		memset(ref, 0, line->frame * sizeof(*ref));
		return;
	}
	ring_tones_fill(line, ref);
}

/* The planes of the last frame of a ring line. They hold the reference, the
 * near speech, the echo, the floor, the microphone and the output. */
struct ring_planes {
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
};

/* One frame of the line, with near on top when it is given. st takes the
 * frame when it is given and the frame's own reference is active. */
static void ring_planes_run(struct ring_line *line, const int16_t *ref,
			    const int16_t *near, struct ring_stat *st,
			    struct ring_planes *p)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	int16_t *echo = p->echo;
	int16_t *hiss = p->hiss;
	int16_t *mic = p->out;
	uint32_t held = line->held;
	unsigned i;

	memcpy(p->ref, ref, line->frame * sizeof(*ref));
	memset(p->near, 0, line->frame * sizeof(*p->near));
	if (near)
		memcpy(p->near, near, line->frame * sizeof(*near));
	path_run(&line->path, ref, echo);
	memset(hiss, 0, line->frame * sizeof(*hiss));
	if (line->floor)
		noise_fill(hiss, line->frame, line->floor_dbm0,
			   &line->floor_state);
	for (i = 0; i < line->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + hiss[i] + p->near[i]);
	memcpy(p->mic, mic, line->frame * sizeof(*mic));
	udsp_ec_nlms.run(line->ec, mic, ref);
	udsp_ec_nlms_info_get(line->ec, &info);
	udsp_ec_nlms_frame_get(line->ec, &reading);
	line->held = info.hold_frames;
	if (!st || !frame_loud(ref, line->frame))
		return;
	st->active++;
	st->held += info.hold_frames != held;
	st->dt += info.double_talk;
	st->est_fired += reading.dt_fired && !reading.classic_arm;
	for (i = 0; i < line->frame; i++) {
		st->echo += (double)echo[i] * echo[i];
		st->residual += (double)mic[i] * mic[i];
		st->floor += (double)hiss[i] * hiss[i];
	}
}

static void ring_frame_run(struct ring_line *line, const int16_t *ref,
			   const int16_t *near, struct ring_stat *st)
{
	struct ring_planes p;

	ring_planes_run(line, ref, near, st, &p);
}

static void ring_tone_run(struct ring_line *line)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned f;

	for (f = 0; f < RING_FRAMES; f++) {
		ring_fill(line, ref, f);
		ring_frame_run(line, ref, NULL, NULL);
	}
}

static void ring_words_run(struct ring_line *line, struct ring_stat *st)
{
	int16_t ref[UDSP_FRAME_MAX];

	word_fill(&line->far, ref, line->frame);
	ring_frame_run(line, ref, NULL, st);
}

static double ring_gap_db_get(const struct udsp_ec_nlms_info *info)
{
	if (!(info->coef_energy > 0.0f) || !(info->echo_gain > 0.0f))
		return -200.0;
	return 10.0 * log10((double)info->coef_energy / info->echo_gain);
}

/* LEC_GEIGEL_TRUST_FRAMES and LEC_GEIGEL_TRUST_PAUSE_FRAMES of dsp/lec.c,
 * and the second burst of the ringback of ring_trust_test. */
#define TRUST_FRAMES		200u
#define TRUST_PAUSE_FRAMES	200u
#define RING_TRUST_ON_FRAMES	300u

/* The ringback of ring_tone_run with a second burst of RING_TRUST_ON_FRAMES
 * frames. The far end returns after a pause of 4 s, so the trust count of
 * dsp/lec.c restarts at the second burst and completes on it. */
static void ring_trust_tone_run(struct ring_line *line)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned f;

	for (f = 0; f < RING_FRAMES; f++) {
		unsigned on = f < RING_CYCLE_FRAMES ? RING_ON_FRAMES :
						      RING_TRUST_ON_FRAMES;

		if (f % RING_CYCLE_FRAMES < on)
			ring_tones_fill(line, ref);
		else
			memset(ref, 0, line->frame * sizeof(*ref));
		ring_frame_run(line, ref, NULL, NULL);
	}
}

/*
 * The trust rule on a filter that learned a ringback. The tone earns a tracked
 * ERLE over the trust line, but its coefficient energy stands more than 10 dB
 * under the learned gain. The energy rule alone then refuses the trust, and no
 * frame of the first far words fires on an estimate bar.
 */
static void ring_trust_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info at;
	struct ring_stat first;
	struct ring_line line;
	unsigned f;

	memset(&first, 0, sizeof(first));
	ring_line_init(&line, rate);
	ring_trust_tone_run(&line);
	udsp_ec_nlms_info_get(line.ec, &at);
	udsp_ec_nlms_frame_get(line.ec, &reading);
	for (f = 0; f < FRAMES_PER_SECOND; f++)
		ring_words_run(&line, &first);
	udsp_ec_nlms.destroy(line.ec);

	case_rate(rate, "lec refuses the trust of a filter trained on a "
		  "ringback");
	printf("# after the ringback: tracked ERLE %.2f dB, coefficient "
	       "energy %.2f dB against the learned gain, trust count %u, "
	       "trusted %u; first second of words: %u of %u active frames in "
	       "double talk, %u fired on the estimate\n", (double)at.erle_db,
	       ring_gap_db_get(&at), reading.trust_run, reading.trusted,
	       first.dt, first.active, first.est_fired);
	CHECK(at.erle_db >= RING_TRUST_DB);
	CHECK(at.coef_energy * RING_COEF_RATIO < at.echo_gain);
	CHECK(reading.trust_run >= TRUST_FRAMES);
	CHECK(!reading.trusted);
	CHECK(first.active > 0u);
	CHECK_UINT(first.est_fired, 0);
}

/*
 * The ringback line, then 20 s of far words: the hold of a stale trusted
 * estimate. The trust rules refuse the tone's trust, so the words run no trial.
 * They hold double talk on 20 percent or less of the first 2 s. The last 8 s
 * hold 20 percent at most, with no reset, and the filter ERLE reaches 15 dB.
 */
static void stale_hold_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info at;
	struct ring_stat first;
	struct ring_stat last;
	struct ring_line line;
	unsigned trial = 0;
	double erle;
	unsigned f;

	memset(&first, 0, sizeof(first));
	memset(&last, 0, sizeof(last));
	ring_line_init(&line, rate);
	ring_tone_run(&line);
	udsp_ec_nlms_info_get(line.ec, &at);
	udsp_ec_nlms_frame_get(line.ec, &reading);
	for (f = 0; f < RING_SPEECH_FRAMES; f++) {
		struct ring_stat *st = NULL;

		if (f < RING_FIRST_FRAMES)
			st = &first;
		else if (f >= RING_SPEECH_FRAMES - RING_LAST_FRAMES)
			st = &last;
		ring_words_run(&line, st);
		udsp_ec_nlms_info_get(line.ec, &info);
		if (!trial && info.probes > at.probes)
			trial = f + 1u;
	}
	udsp_ec_nlms.destroy(line.ec);
	erle = weak_filter_erle_get(last.echo, last.residual, last.floor);

	case_rate(rate, "lec ends the hold of a stale trusted estimate within "
		  "seconds");
	printf("# tracked ERLE %.2f dB at the first word, trusted %u; first "
	       "2 s: double talk on %u of %u active frames; %u trials, the "
	       "first at %u ms, %u resets; last 8 s: %u of %u active frames "
	       "held, filter ERLE %.2f dB\n", (double)at.erle_db,
	       reading.trusted, first.dt, first.active,
	       info.probes - at.probes, 10u * trial, info.diverge_resets,
	       last.held, last.active, erle);
	CHECK(at.erle_db >= RING_TRUST_DB);
	CHECK(!reading.trusted);
	CHECK(first.dt * 10u <= first.active * 2u);
	CHECK_UINT(info.probes - at.probes, 0);
	CHECK_UINT(info.diverge_resets, 0);
	CHECK(last.active > 0u);
	CHECK(last.held * 10u <= last.active * 2u);
	CHECK(erle >= 15.0);
}

#define TALKER_CONVERGE_FRAMES	(3u * FRAMES_PER_SECOND)
#define TALKER_FRAMES		(20u * FRAMES_PER_SECOND)
/* The echo of the far end through the hybrid of the ringback line. */
#define TALKER_ECHO_DBM0	(RING_WORDS_DBM0 - RING_ERL_DB)
/* LEC_GEIGEL_STALE_FRAMES and LEC_GEIGEL_LATCH_MAX of dsp/lec.c. */
#define TALKER_STALE_FRAMES	200u
#define TALKER_LATCH_FRAMES	500u

/* Runs the ringback line on white noise at the level of the words. */
static void ring_white_run(struct ring_line *line, unsigned frames)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned f;

	for (f = 0; f < frames; f++) {
		noise_fill(ref, line->frame, RING_WORDS_DBM0, &line->far_state);
		ring_frame_run(line, ref, NULL, NULL);
	}
}

/*
 * A talker that the far end cannot explain starts no stale-run trial. Far words
 * with a near talker 24 dB over the echo fire on the trusted estimate, but each
 * frame stands over the classic bar. The stale run stays under its trigger,
 * with no trial and no reset.
 */
static void stale_talker_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct ring_line line;
	unsigned fired = 0;
	unsigned longest = 0;
	unsigned f;

	ring_line_init(&line, UDSP_SAMPLE_RATE);
	word_source_init(&talker, UDSP_SAMPLE_RATE, RING_WORDS_DBM0,
			 0x0badcafeu);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 0; f < TALKER_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];

		word_fill(&line.far, ref, line.frame);
		word_fill(&talker, near, line.frame);
		ring_frame_run(&line, ref, near, NULL);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		fired += reading.dt_fired && reading.trusted;
		if (reading.stale_run > longest)
			longest = reading.stale_run;
	}
	udsp_ec_nlms_info_get(line.ec, &info);
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec starts no stale-estimate trial for a talker the far end "
	     "cannot explain");
	printf("# %u frames fired on a trusted estimate, longest stale run "
	       "%u, %u trials, %u resets\n", fired, longest, info.probes,
	       info.diverge_resets);
	CHECK(fired >= TALKER_STALE_FRAMES);
	CHECK(longest < TALKER_STALE_FRAMES);
	CHECK_UINT(info.probes, 0);
	CHECK_UINT(info.diverge_resets, 0);
}

/* The shortest distance between two stale-run trial starts in active frames,
 * stand-down plus trial; the stand-down alone separates trials of any arm under
 * a talker who never pauses. */
#define EXPOSE_CADENCE		700u
#define EXPOSE_ANY_CADENCE	600u
#define EXPOSE_FRAMES		(30u * FRAMES_PER_SECOND)
#define EXPOSE_RESTORE		0.01
#define EXPOSE_NEAR_DB		2.0
#define EXPOSE_LEAK_DB		3.0

/*
 * The trials of one exposure run: a stale-run trial follows a stale run at the
 * trigger and a latch run under its own. A trial counts as kept when its end
 * energy stands more than 1 percent from its start. Gaps count active frames;
 * the energies cover the trial frames.
 */
struct expose_stat {
	unsigned trials;
	unsigned stale;
	unsigned kept;
	unsigned gap_min;
	unsigned stale_gap_min;
	unsigned active;
	unsigned trial_at;
	unsigned stale_at;
	float start_energy;
	bool open;
	double mic;
	double out;
	double echo;
	double leak;
	double near;
	double product;
};

static void expose_gap_note(unsigned *gap_min, unsigned *at, unsigned count,
			    unsigned active)
{
	if (count && (!*gap_min || active - *at < *gap_min))
		*gap_min = active - *at;
	*at = active;
}

static void expose_trial_end(struct expose_stat *st,
			     const struct udsp_ec_nlms_info *info)
{
	double miss = fabs((double)info->coef_energy - st->start_energy);

	st->open = false;
	if (miss > EXPOSE_RESTORE * st->start_energy)
		st->kept++;
}

static void expose_trial_step(struct expose_stat *st,
			      const struct udsp_ec_nlms_frame *before,
			      const struct udsp_ec_nlms_frame *reading,
			      const struct udsp_ec_nlms_info *info,
			      unsigned probes)
{
	bool stale = before->stale_run >= TALKER_STALE_FRAMES &&
		     before->dt_run < TALKER_LATCH_FRAMES;

	if (st->open && !reading->probing)
		expose_trial_end(st, info);
	if (info->probes == probes)
		return;
	expose_gap_note(&st->gap_min, &st->trial_at, st->trials, st->active);
	st->trials++;
	st->open = true;
	st->start_energy = info->coef_energy;
	if (!stale)
		return;
	expose_gap_note(&st->stale_gap_min, &st->stale_at, st->stale,
			st->active);
	st->stale++;
}

static void expose_energy_add(struct expose_stat *st, const int16_t *near,
			      const struct ring_planes *p, unsigned frame)
{
	unsigned i;

	for (i = 0; i < frame; i++) {
		double talk = near ? near[i] : 0.0;
		double leak = (double)p->out[i] - talk - p->hiss[i];

		st->mic += (double)p->mic[i] * p->mic[i];
		st->out += (double)p->out[i] * p->out[i];
		st->echo += (double)p->echo[i] * p->echo[i];
		st->leak += leak * leak;
		st->near += talk * talk;
		st->product += (double)p->out[i] * talk;
	}
}

/* One frame of an exposure run, with near on top when it is given. */
static void expose_frame_run(struct ring_line *line, const int16_t *ref,
			     const int16_t *near, struct expose_stat *st)
{
	struct udsp_ec_nlms_frame before;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_planes p;
	uint32_t probes;

	udsp_ec_nlms_info_get(line->ec, &info);
	udsp_ec_nlms_frame_get(line->ec, &before);
	probes = info.probes;
	ring_planes_run(line, ref, near, NULL, &p);
	udsp_ec_nlms_info_get(line->ec, &info);
	udsp_ec_nlms_frame_get(line->ec, &reading);
	if (frame_loud(ref, line->frame))
		st->active++;
	expose_trial_step(st, &before, &reading, &info, probes);
	if (st->open)
		expose_energy_add(st, near, &p, line->frame);
}

/* An exposure frame on a white far end at the level of the words. */
static void expose_white_run(struct ring_line *line, const int16_t *near,
			     struct expose_stat *st)
{
	int16_t ref[UDSP_FRAME_MAX];

	noise_fill(ref, line->frame, RING_WORDS_DBM0, &line->far_state);
	expose_frame_run(line, ref, near, st);
}

static double expose_db_get(double num, double den)
{
	if (!(num > 0.0) || !(den > 0.0))
		return 200.0;
	return 10.0 * log10(num / den);
}

static double expose_near_db_get(const struct expose_stat *st)
{
	return near_gain_db_get(st->product, st->near);
}

static void expose_print(const char *what, const struct expose_stat *st)
{
	printf("# %s: %u trials over %u active frames, %u of the stale run, "
	       "%u kept; shortest gaps %u, and %u between two of the stale "
	       "run; in the trials: near speech %.2f dB through, output %.2f "
	       "dB against the microphone, echo %.2f dB under its level\n",
	       what, st->trials, st->active, st->stale, st->kept, st->gap_min,
	       st->stale_gap_min, expose_near_db_get(st),
	       expose_db_get(st->out, st->mic),
	       expose_db_get(st->echo, st->leak));
}

/* The bounds of the trial frames that every exposure case shares. */
static void expose_bounds_check(const struct expose_stat *st)
{
	CHECK(fabs(expose_near_db_get(st)) <= EXPOSE_NEAR_DB);
	CHECK(expose_db_get(st->out, st->mic) <= TALK_BURST_DB);
	CHECK(expose_db_get(st->echo, st->leak) >= EXPOSE_LEAK_DB);
}

#define QUIET_FRAMES		(60u * FRAMES_PER_SECOND)
#define QUIET_FIRST_FRAMES	(20u * FRAMES_PER_SECOND)
#define QUIET_UNDER_DB		2.0

/*
 * Near words 2 dB under the echo on far words for 60 s fire under the classic
 * bar. The stale run then starts trials that the talker refuses. One comes in
 * the first 20 s, and each restores the energy within 1 percent. Two stand at
 * least 700 active frames apart, within the bounds of the steady talker.
 */
static void stale_quiet_test(void)
{
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct expose_stat st;
	struct ring_line line;
	unsigned first = 0;
	unsigned f;

	memset(&st, 0, sizeof(st));
	ring_line_init(&line, UDSP_SAMPLE_RATE);
	word_source_init(&talker, UDSP_SAMPLE_RATE,
			 TALKER_ECHO_DBM0 - QUIET_UNDER_DB, 0x31415926u);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 0; f < QUIET_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];

		word_fill(&line.far, ref, line.frame);
		word_fill(&talker, near, line.frame);
		expose_frame_run(&line, ref, near, &st);
		if (f + 1u == QUIET_FIRST_FRAMES)
			first = st.trials;
	}
	udsp_ec_nlms_info_get(line.ec, &info);
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec rolls back a stale-estimate trial on a quiet talker and "
	     "stands down");
	expose_print("quiet talker", &st);
	printf("# %u trials in the first 20 s, %u resets\n", first,
	       info.diverge_resets);
	CHECK_UINT(first, 1);
	CHECK(st.stale >= 2u);
	CHECK_UINT(st.stale, st.trials);
	CHECK_UINT(st.kept, 0);
	CHECK(st.stale_gap_min >= EXPOSE_CADENCE);
	CHECK_UINT(info.diverge_resets, 0);
	expose_bounds_check(&st);
}

/*
 * A white near talker at echo level on the converged ringback line for 30 s:
 * the stale run starts the trials and the latch run waits on the independent
 * talker. Each trial is refused, EXPOSE_ANY_CADENCE frames apart or more; in
 * the trial frames near speech passes within 2 dB, and the echo stays reduced.
 */
static void expose_steady_test(void)
{
	struct expose_stat st;
	struct ring_line line;
	unsigned f;

	memset(&st, 0, sizeof(st));
	ring_line_init(&line, UDSP_SAMPLE_RATE);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 0; f < EXPOSE_FRAMES; f++) {
		int16_t near[UDSP_FRAME_SAMPLES];

		noise_fill(near, line.frame, TALKER_ECHO_DBM0,
			   &line.near_state);
		expose_white_run(&line, near, &st);
	}
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec bounds the stale-estimate trials of a steady talker between "
	     "the two bars");
	expose_print("steady talker", &st);
	CHECK(st.stale >= 1u);
	CHECK(st.trials >= 2u);
	CHECK_UINT(st.kept, 0);
	CHECK(st.gap_min >= EXPOSE_ANY_CADENCE);
	expose_bounds_check(&st);
}

#define TURNS_EXPOSE_CYCLE	500u
#define TURNS_EXPOSE_FAR	300u

/* One turn pattern of expose_turns_test: the near turn starts near_from
 * frames into the cycle. */
static void expose_turns_run(unsigned near_from, struct expose_stat *st)
{
	struct word_source talker;
	struct ring_line line;
	unsigned f;

	memset(st, 0, sizeof(*st));
	ring_line_init(&line, UDSP_SAMPLE_RATE);
	word_source_init(&talker, UDSP_SAMPLE_RATE,
			 TALKER_ECHO_DBM0 - QUIET_UNDER_DB, 0x31415926u);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 0; f < QUIET_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t near[UDSP_FRAME_SAMPLES];
		unsigned p = f % TURNS_EXPOSE_CYCLE;
		bool talk = p >= near_from &&
			    p < near_from + TURNS_EXPOSE_FAR;

		if (p == 0u)
			word_source_restart(&line.far);
		if (p == near_from)
			word_source_restart(&talker);
		memset(ref, 0, sizeof(ref));
		memset(near, 0, sizeof(near));
		if (p < TURNS_EXPOSE_FAR)
			word_fill(&line.far, ref, line.frame);
		if (talk)
			word_fill(&talker, near, line.frame);
		expose_frame_run(&line, ref, talk ? near : NULL, st);
	}
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * Turns of a talker between the two bars, on the converged ringback line.
 * Far-end words and near words 2 dB under the echo take turns for 60 s, in a
 * cycle of 5 s. Each talker talks for 3 s of the cycle. The near turn
 * overlaps the end of the far turn by 0.5, 1.5 or 3 s. The clear frames of
 * each far turn repay the stale run, so the turns must cost no trial.
 */
static void expose_turns_test(void)
{
	static const unsigned overlap[] = { 50u, 150u, 300u };
	unsigned i;

	CASE("lec bounds the stale-estimate trials of a talker in turns "
	     "between the two bars");
	for (i = 0; i < sizeof(overlap) / sizeof(overlap[0]); i++) {
		struct expose_stat st;
		char what[40];

		expose_turns_run(TURNS_EXPOSE_FAR - overlap[i], &st);
		snprintf(what, sizeof(what), "overlap of %u ms",
			 10u * overlap[i]);
		expose_print(what, &st);
		CHECK(st.active > 0u);
		CHECK_UINT(st.trials, 0);
	}
}

#define STOP_TALK_FRAMES	50u
#define STOP_AFTER_FRAMES	(3u * FRAMES_PER_SECOND)
#define STOP_ERLE_LOSS_DB	3.0

/*
 * A talker between the two bars stops 50 frames into a stale-run trial, and the
 * far end then talks alone for 3 s. The verdict can keep a filter that adapted
 * on the talker, at a cost of 3 dB of filter ERLE at most. The trial frames
 * keep the bounds of the steady talker.
 */
static void expose_stop_test(void)
{
	struct ring_stat before;
	struct ring_stat after;
	struct expose_stat st;
	struct ring_line line;
	unsigned talked = 0;
	double erle_before;
	double erle_after;
	unsigned f;

	memset(&before, 0, sizeof(before));
	memset(&after, 0, sizeof(after));
	memset(&st, 0, sizeof(st));
	ring_line_init(&line, UDSP_SAMPLE_RATE);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES - FRAMES_PER_SECOND);
	for (f = 0; f < FRAMES_PER_SECOND; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];

		noise_fill(ref, line.frame, RING_WORDS_DBM0, &line.far_state);
		ring_frame_run(&line, ref, NULL, &before);
	}
	for (f = 0; f < EXPOSE_FRAMES && talked < STOP_TALK_FRAMES; f++) {
		int16_t near[UDSP_FRAME_SAMPLES];

		noise_fill(near, line.frame, TALKER_ECHO_DBM0,
			   &line.near_state);
		expose_white_run(&line, near, &st);
		if (st.trials)
			talked++;
	}
	for (f = 0; f < STOP_AFTER_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_SAMPLES];

		if (f + FRAMES_PER_SECOND < STOP_AFTER_FRAMES) {
			expose_white_run(&line, NULL, &st);
			continue;
		}
		noise_fill(ref, line.frame, RING_WORDS_DBM0, &line.far_state);
		ring_frame_run(&line, ref, NULL, &after);
	}
	udsp_ec_nlms.destroy(line.ec);
	erle_before = weak_filter_erle_get(before.echo, before.residual,
					   before.floor);
	erle_after = weak_filter_erle_get(after.echo, after.residual,
					  after.floor);

	CASE("lec bounds a stale-estimate trial whose talker stops during the "
	     "trial");
	expose_print("talker who stops", &st);
	printf("# filter ERLE %.2f dB before the talker, %.2f dB in the last "
	       "second\n", erle_before, erle_after);
	CHECK_UINT(st.stale, 1);
	CHECK_UINT(st.trials, 1);
	CHECK(erle_after >= erle_before - STOP_ERLE_LOSS_DB);
	expose_bounds_check(&st);
}

#define TWIN_COMPARE_FRAMES	20u

/* One frame of white far end to two ringback lines, with near on top when it
 * is given. Both lines start from the same seeds, so they take the same
 * frames, and out of each holds its output. */
static void twin_ring_run(struct ring_line *lead, struct ring_line *twin,
			  const int16_t *near, struct ring_planes *lead_p,
			  struct ring_planes *twin_p)
{
	int16_t ref[UDSP_FRAME_MAX];

	noise_fill(ref, lead->frame, RING_WORDS_DBM0, &lead->far_state);
	noise_fill(ref, twin->frame, RING_WORDS_DBM0, &twin->far_state);
	ring_planes_run(lead, ref, near, NULL, lead_p);
	ring_planes_run(twin, ref, near, NULL, twin_p);
}

/*
 * The filter after a refused stale-run trial against a twin frozen on the
 * trial's start frame: both freeze after the refusal and take 20 frames of far
 * end alone. Their outputs must match sample for sample, which holds only when
 * they hold the same coefficients.
 */
static void expose_twin_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info lead_info;
	struct udsp_ec_nlms_info twin_info;
	struct ring_planes lead_p;
	struct ring_planes twin_p;
	struct ring_line lead;
	struct ring_line twin;
	unsigned started = 0;
	unsigned ended = 0;
	unsigned differ = 0;
	unsigned f;

	ring_line_init(&lead, UDSP_SAMPLE_RATE);
	ring_line_init(&twin, UDSP_SAMPLE_RATE);
	for (f = 0; f < TALKER_CONVERGE_FRAMES; f++)
		twin_ring_run(&lead, &twin, NULL, &lead_p, &twin_p);
	for (f = 0; f < EXPOSE_FRAMES && !ended; f++) {
		int16_t near[UDSP_FRAME_SAMPLES];

		noise_fill(near, lead.frame, TALKER_ECHO_DBM0,
			   &lead.near_state);
		twin_ring_run(&lead, &twin, near, &lead_p, &twin_p);
		udsp_ec_nlms_info_get(lead.ec, &lead_info);
		udsp_ec_nlms_frame_get(lead.ec, &reading);
		if (!started && lead_info.probes) {
			started = f + 1u;
			udsp_ec_nlms.freeze(twin.ec, true);
		} else if (started && !reading.probing) {
			ended = f + 1u;
		}
	}
	udsp_ec_nlms.freeze(lead.ec, true);
	for (f = 0; f < TWIN_COMPARE_FRAMES; f++) {
		twin_ring_run(&lead, &twin, NULL, &lead_p, &twin_p);
		if (memcmp(lead_p.out, twin_p.out,
			   lead.frame * sizeof(*lead_p.out)))
			differ++;
	}
	udsp_ec_nlms_info_get(lead.ec, &lead_info);
	udsp_ec_nlms_info_get(twin.ec, &twin_info);
	udsp_ec_nlms.destroy(lead.ec);
	udsp_ec_nlms.destroy(twin.ec);

	CASE("lec restores the filter exactly after a rejected stale-estimate "
	     "trial");
	printf("# trial from frame %u to frame %u, %u trials; coefficient "
	       "energy %.9f against the twin's %.9f; %u of %u frames "
	       "differ\n", started, ended, lead_info.probes,
	       (double)lead_info.coef_energy, (double)twin_info.coef_energy,
	       differ, TWIN_COMPARE_FRAMES);
	CHECK(started > 0u);
	CHECK(ended > started);
	CHECK_UINT(lead_info.probes, 1);
	CHECK(lead_info.coef_energy == twin_info.coef_energy);
	CHECK_UINT(differ, 0);
}

#define MARK_TRIALS_MAX		16u

/*
 * The trial fields of the frame reading over a run of frames. A trial starts
 * on the frame that moves the probe count. It ends on the last frame that
 * reads probing before a frame that does not. arm and end hold the fields of
 * those frames in order, up to MARK_TRIALS_MAX trials. The stray counts take
 * the other frames that read an arm or an end.
 */
struct trial_marks {
	struct udsp_ec_nlms_frame last;
	uint32_t probes;
	bool have_last;
	unsigned starts;
	unsigned ends;
	uint8_t arm[MARK_TRIALS_MAX];
	uint8_t end[MARK_TRIALS_MAX];
	unsigned start_probing;
	unsigned stray_arms;
	unsigned stray_ends;
};

/* The frame in last ends a trial when ended is set. */
static void trial_marks_end_note(struct trial_marks *m, bool ended)
{
	if (!ended) {
		if (m->last.trial_end != UDSP_EC_NLMS_TRIAL_END_NONE)
			m->stray_ends++;
		return;
	}
	if (m->ends < MARK_TRIALS_MAX)
		m->end[m->ends] = m->last.trial_end;
	m->ends++;
}

/* A fixture calls it after each frame. A NULL m takes nothing, so a fixture
 * serves a case that reads no trial field. */
static void trial_marks_add(struct trial_marks *m, const struct udsp_ec *ec)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;

	if (!m)
		return;
	udsp_ec_nlms_frame_get(ec, &reading);
	udsp_ec_nlms_info_get(ec, &info);
	if (m->have_last)
		trial_marks_end_note(m, m->last.probing && !reading.probing);
	m->last = reading;
	m->have_last = true;
	if (info.probes == m->probes) {
		if (reading.trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE)
			m->stray_arms++;
		return;
	}
	m->probes = info.probes;
	m->start_probing += reading.probing;
	if (m->starts < MARK_TRIALS_MAX)
		m->arm[m->starts] = reading.trial_arm;
	m->starts++;
}

/* No frame follows the last frame of a run, so it counts as a frame that
 * ends no trial. Each leg therefore runs on after its last trial. */
static void trial_marks_finish(struct trial_marks *m)
{
	if (m->have_last)
		trial_marks_end_note(m, false);
	m->have_last = false;
}

static void trial_marks_print(const char *what, const struct trial_marks *m)
{
	unsigned i;

	printf("# %s: %u trials, arms", what, m->starts);
	for (i = 0; i < m->starts && i < MARK_TRIALS_MAX; i++)
		printf(" %s", lecr_trial_arm_name(m->arm[i]));
	printf("; %u ends", m->ends);
	for (i = 0; i < m->ends && i < MARK_TRIALS_MAX; i++)
		printf(" %s", lecr_trial_end_name(m->end[i]));
	printf("; %u start frames read probing, %u other frames read an arm, "
	       "%u an end\n", m->start_probing, m->stray_arms, m->stray_ends);
}

/* The checks of every leg: a start frame reads probing 0, and no other
 * frame reads an arm or an end. */
static void trial_marks_check(const struct trial_marks *m)
{
	CHECK_UINT(m->start_probing, 0);
	CHECK_UINT(m->stray_arms, 0);
	CHECK_UINT(m->stray_ends, 0);
}

#define CHANGE_FAR_DBM0		-10.0
#define CHANGE_ERL_DB		12.0
#define CHANGE_FLOOR_DBM0	-70.0
/* 4 s, so the trust count of dsp/lec.c completes before the talker. */
#define CHANGE_CONVERGE_FRAMES	(4u * FRAMES_PER_SECOND)
#define CHANGE_TALK_FRAMES	400u
#define CHANGE_FRAMES		(3u * FRAMES_PER_SECOND)
#define CHANGE_BLOCK_FRAMES	25u
#define CHANGE_BAR_DB		20.0
#define CHANGE_ECHO_DBM0	(CHANGE_FAR_DBM0 - CHANGE_ERL_DB)

/* The trials and the settle of one hybrid change: the frames after the
 * change to the end of the first block of 25 frames whose ERLE reaches
 * 20 dB. */
struct change_result {
	unsigned talk_trials;
	unsigned trials;
	unsigned settled;
	unsigned diverge_resets;
};

/* One frame of the change line, with a white near talker at talk_dbm0 when
 * talk is set. It returns the echo and the output energies. */
static void change_frame_run(struct udsp_ec *ec, struct echo_path *path,
			     uint32_t state[3], bool talk, double talk_dbm0,
			     double *echo_energy, double *res_energy)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	noise_fill(ref, path->frame, CHANGE_FAR_DBM0, &state[0]);
	path_run(path, ref, echo);
	memset(near, 0, sizeof(near));
	if (talk)
		noise_fill(near, path->frame, talk_dbm0, &state[1]);
	noise_fill(hiss, path->frame, CHANGE_FLOOR_DBM0, &state[2]);
	for (i = 0; i < path->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + near[i] + hiss[i]);
	udsp_ec_nlms.run(ec, mic, ref);
	for (i = 0; i < path->frame; i++) {
		*echo_energy += (double)echo[i] * echo[i];
		*res_energy += (double)mic[i] * mic[i];
	}
}

/*
 * A talker between the two bars during talk-over, then a hybrid change. A flat
 * 12 dB hybrid converges, and a white talker at echo level joins for 400 frames
 * and refuses a stale-run trial. The hybrid then changes to the late one at 28
 * ms. marks takes each frame when it is not NULL.
 */
static void change_run(unsigned rate, struct change_result *out,
		       struct trial_marks *marks)
{
	struct udsp_ec_nlms_info info;
	struct echo_path first;
	struct echo_path second;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state[3] = { 0x13572468u, 0xfeedfaceu, 0x2468acefu };
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned f;

	memset(out, 0, sizeof(*out));
	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&first, rate, 8, CHANGE_ERL_DB);
	path_late_init(&second, rate, CHANGE_ERL_DB);
	for (f = 0; f < CHANGE_CONVERGE_FRAMES + CHANGE_TALK_FRAMES; f++) {
		change_frame_run(ec, &first, state,
				 f >= CHANGE_CONVERGE_FRAMES, CHANGE_ECHO_DBM0,
				 &echo_energy, &res_energy);
		trial_marks_add(marks, ec);
	}
	udsp_ec_nlms_info_get(ec, &info);
	out->talk_trials = info.probes;
	path_history_carry(&second, &first);
	echo_energy = res_energy = 0.0;
	for (f = 0; f < CHANGE_FRAMES; f++) {
		change_frame_run(ec, &second, state, false, CHANGE_ECHO_DBM0,
				 &echo_energy, &res_energy);
		trial_marks_add(marks, ec);
		if ((f + 1u) % CHANGE_BLOCK_FRAMES)
			continue;
		if (!out->settled && res_energy > 0.0 &&
		    10.0 * log10(echo_energy / res_energy) >= CHANGE_BAR_DB)
			out->settled = f + 1u;
		echo_energy = res_energy = 0.0;
	}
	udsp_ec_nlms_info_get(ec, &info);
	out->trials = info.probes;
	out->diverge_resets = info.diverge_resets;
	udsp_ec_nlms.destroy(ec);
}

/* The figures of change_run on a canceller without the stale run, at both
 * rates, with the convergence of CHANGE_CONVERGE_FRAMES. */
#define CHANGE_HEAD_TRIALS	1u
#define CHANGE_HEAD_SETTLED	125u

/*
 * The refused stale-run trial stands the stale run down and not the path-change
 * escape, so the escape answers the change. The line runs one trial more than
 * without the stale run, settles within CHANGE_HEAD_SETTLED frames and runs no
 * reset.
 */
static void change_test(unsigned rate)
{
	struct change_result r;

	change_run(rate, &r, NULL);

	case_rate(rate, "lec leaves the path-change escape free after a "
		  "rejected stale-estimate trial");
	printf("# %u trials in the talk-over, %u in all, %u resets; ERLE "
	       "%.0f dB after %u ms, %u ms before the stale run\n",
	       r.talk_trials, r.trials, r.diverge_resets, CHANGE_BAR_DB,
	       10u * r.settled, 10u * CHANGE_HEAD_SETTLED);
	CHECK_UINT(r.talk_trials, 1);
	CHECK_UINT(r.trials, CHANGE_HEAD_TRIALS + 1u);
	CHECK(r.settled > 0u);
	CHECK(r.settled <= CHANGE_HEAD_SETTLED);
	CHECK_UINT(r.diverge_resets, 0);
}

#define RETURN_PAUSE_FRAMES	50u
#define RETURN_TALK_DBM0	RING_WORDS_DBM0
#define RETURN_OVER_DB		15.0
/* LEC_GEIGEL_GAIN_HEADROOM, LEC_GEIGEL_GAIN_MIN, LEC_GEIGEL_FLOOR_HEADROOM and
 * LEC_GEIGEL_MARGIN of dsp/lec.c. */
#define RETURN_GAIN_HEADROOM	2.0
#define RETURN_GAIN_MIN		0.25
#define RETURN_FLOOR_HEADROOM	2.0
#define RETURN_MARGIN		1.41

/* The microphone of the last frame over the bar that the classic bound of
 * dsp/lec.c alone sets (echo_bound_get), in dB. */
static double return_over_db_get(const struct udsp_ec_nlms_frame *reading,
				 const struct udsp_ec_nlms_info *info)
{
	double bound = info->echo_gain * RETURN_GAIN_HEADROOM;
	double bar;

	if (bound > RETURN_GAIN_MIN)
		bound = RETURN_GAIN_MIN;
	bar = (reading->window_power * bound +
	       reading->noise_power * RETURN_FLOOR_HEADROOM) * RETURN_MARGIN;
	return 10.0 * log10(reading->mic_power / bar);
}

/* One frame of a white far end at the level of the words, with a white near
 * talker at talk_dbm0 on top, and the frame reading after it. */
static void return_white_run(struct ring_line *line, double talk_dbm0,
			     struct udsp_ec_nlms_frame *reading)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];

	noise_fill(ref, line->frame, RING_WORDS_DBM0, &line->far_state);
	noise_fill(near, line->frame, talk_dbm0, &line->near_state);
	ring_frame_run(line, ref, near, NULL);
	udsp_ec_nlms_frame_get(line->ec, reading);
}

/*
 * A talker on the frame of the far return starts no stale-run trial. After a
 * run of 200 and a 500 ms pause, a talker 15 dB or more over the classic bar
 * speaks on the return. The run reads 0 after that frame, and no trial starts
 * on it or the next 100 active frames.
 */
static void stale_return_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_line line;
	int16_t ref[UDSP_FRAME_MAX];
	unsigned built;
	unsigned kept;
	bool held;
	bool trusted;
	unsigned run;
	unsigned at_return;
	double over;
	unsigned f;

	memset(&reading, 0, sizeof(reading));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 0; f < EXPOSE_FRAMES &&
		    reading.stale_run < TALKER_STALE_FRAMES; f++)
		return_white_run(&line, TALKER_ECHO_DBM0, &reading);
	built = reading.stale_run;
	memset(ref, 0, sizeof(ref));
	for (f = 0; f < RETURN_PAUSE_FRAMES; f++)
		ring_frame_run(&line, ref, NULL, NULL);
	udsp_ec_nlms_frame_get(line.ec, &reading);
	udsp_ec_nlms_info_get(line.ec, &info);
	kept = reading.stale_run;
	held = info.double_talk;
	return_white_run(&line, RETURN_TALK_DBM0, &reading);
	udsp_ec_nlms_info_get(line.ec, &info);
	over = return_over_db_get(&reading, &info);
	trusted = reading.trusted;
	run = reading.stale_run;
	at_return = info.probes;
	for (f = 0; f < TRIAL_BUDGET; f++)
		return_white_run(&line, RETURN_TALK_DBM0, &reading);
	udsp_ec_nlms_info_get(line.ec, &info);
	udsp_ec_nlms.destroy(line.ec);

	case_rate(rate, "lec starts no stale-estimate trial on the frame a "
		  "loud talker returns");
	printf("# stale run %u under the far end, %u after the pause, double "
	       "talk %u at its end; return frame: talker %.2f dB over the "
	       "classic bar, trusted %u, stale run %u, %u trials; %u trials "
	       "after %u more active frames\n", built, kept, held, over,
	       trusted, run, at_return, info.probes, TRIAL_BUDGET);
	CHECK(built >= TALKER_STALE_FRAMES);
	CHECK(kept >= TALKER_STALE_FRAMES);
	CHECK(!held);
	CHECK(trusted);
	CHECK(over >= RETURN_OVER_DB);
	CHECK_UINT(run, 0);
	CHECK_UINT(at_return, 0);
	CHECK_UINT(info.probes, 0);
}

/* LEC_TRIAL_STANDDOWN_FRAMES of dsp/lec.c. */
#define CARRY_COOLDOWN		600u
#define CARRY_LOUD_DBM0		(CHANGE_ECHO_DBM0 + 3.0)
#define CARRY_CLEAR_FRAMES	150u
#define CARRY_LIMIT_FRAMES	(10u * FRAMES_PER_SECOND)

/*
 * One trial of the carry case: start and end count the frames to its start and
 * past its end. The runs of the frame before name the arm: a stale run at its
 * trigger with the latch run under, the stale run's; both under, the
 * path-change escape's. Kept as in struct expose_stat.
 */
struct carry_trial {
	unsigned start;
	unsigned end;
	unsigned arm;
	bool path;
	bool stale;
	bool kept;
};

/* Runs frames of the change line with a near talker at talk_dbm0 until a
 * trial starts and ends, or for limit frames. */
static void carry_trial_run(struct udsp_ec *ec, struct echo_path *path,
			    uint32_t state[3], double talk_dbm0, unsigned limit,
			    struct carry_trial *t)
{
	struct udsp_ec_nlms_frame before;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	float energy = 0.0f;
	uint32_t probes;
	unsigned f;

	memset(t, 0, sizeof(*t));
	for (f = 0; f < limit && !t->end; f++) {
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &before);
		probes = info.probes;
		change_frame_run(ec, path, state, true, talk_dbm0,
				 &echo_energy, &res_energy);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (info.probes != probes) {
			t->start = f + 1u;
			t->arm = reading.trial_arm;
			t->stale = before.stale_run >= TALKER_STALE_FRAMES &&
				   before.dt_run < TALKER_LATCH_FRAMES;
			t->path = before.stale_run < TALKER_STALE_FRAMES &&
				  before.dt_run < TALKER_LATCH_FRAMES;
			energy = info.coef_energy;
		} else if (t->start && !reading.probing) {
			t->end = f + 1u;
			t->kept = fabs((double)info.coef_energy - energy) >
				  EXPOSE_RESTORE * energy;
		}
	}
}

/*
 * Frames of the change line with a near talker at talk_dbm0 until a trial of
 * arm starts, or for limit frames. It returns the frames to that start, limit
 * for none. first takes the arm of the first trial that starts, NONE for
 * none.
 */
static unsigned carry_arm_run(struct udsp_ec *ec, struct echo_path *path,
			      uint32_t state[3], double talk_dbm0,
			      unsigned limit, unsigned arm, unsigned *first)
{
	struct udsp_ec_nlms_frame reading;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned f;

	*first = UDSP_EC_NLMS_TRIAL_ARM_NONE;
	for (f = 0; f < limit; f++) {
		change_frame_run(ec, path, state, true, talk_dbm0,
				 &echo_energy, &res_energy);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_NONE)
			continue;
		if (*first == UDSP_EC_NLMS_TRIAL_ARM_NONE)
			*first = reading.trial_arm;
		if (reading.trial_arm == arm)
			return f + 1u;
	}
	return limit;
}

/*
 * A refused stale-run trial inside an escape's stand-down carries it: a talker
 * refuses a path-change trial, then a stale-run trial inside its stand-down.
 * When the late hybrid returns, no path-change trial starts within 600 frames
 * of the second refusal, and the coherence escape answers first.
 */
static void stale_carry_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct carry_trial change;
	struct carry_trial stale;
	struct echo_path first;
	struct echo_path second;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state[3] = { 0x13572468u, 0xfeedfaceu, 0x2468acefu };
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned next_arm;
	unsigned again_at;
	unsigned gap;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&first, rate, 8, CHANGE_ERL_DB);
	path_late_init(&second, rate, CHANGE_ERL_DB);
	for (f = 0; f < CHANGE_CONVERGE_FRAMES; f++)
		change_frame_run(ec, &first, state, false, CHANGE_ECHO_DBM0,
				 &echo_energy, &res_energy);
	path_history_carry(&second, &first);
	carry_trial_run(ec, &second, state, CARRY_LOUD_DBM0, CARRY_LIMIT_FRAMES,
			&change);
	path_history_carry(&first, &second);
	carry_trial_run(ec, &first, state, CHANGE_ECHO_DBM0, CARRY_LIMIT_FRAMES,
			&stale);
	for (f = 0; f < CARRY_CLEAR_FRAMES; f++)
		change_frame_run(ec, &first, state, false, CHANGE_ECHO_DBM0,
				 &echo_energy, &res_energy);
	path_history_carry(&second, &first);
	again_at = carry_arm_run(ec, &second, state, CARRY_LOUD_DBM0,
				 CARRY_LIMIT_FRAMES, change.arm, &next_arm);
	udsp_ec_nlms_info_get(ec, &info);
	udsp_ec_nlms.destroy(ec);
	/* The refusal falls on the frame before end, and the frame at end and
	 * the clear frames follow it. */
	gap = 1u + CARRY_CLEAR_FRAMES + again_at;

	case_rate(rate, "lec carries the stand-down of an escape through a "
		  "refused stale-estimate trial");
	printf("# %s escape trial %u frames after the change, kept %u; "
	       "stale-run trial %u frames after the refusal of the escape "
	       "trial, stale %u, kept %u; next trial of the %s escape %u "
	       "frames after the refusal of the stale-run trial (%u for "
	       "none), first trial after it %s; %u trials, %u resets\n",
	       lecr_trial_arm_name(change.arm), change.start, change.kept,
	       1u + stale.start, stale.stale, stale.kept,
	       lecr_trial_arm_name(change.arm), gap,
	       1u + CARRY_CLEAR_FRAMES + CARRY_LIMIT_FRAMES,
	       lecr_trial_arm_name(next_arm), info.probes,
	       info.diverge_resets);
	CHECK(change.end > 0u);
	CHECK(change.path);
	CHECK_UINT(change.arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
	CHECK(!change.kept);
	CHECK(stale.end > 0u);
	CHECK(stale.stale);
	CHECK_UINT(stale.arm, UDSP_EC_NLMS_TRIAL_ARM_STALE);
	CHECK(!stale.kept);
	CHECK(1u + stale.start < CARRY_COOLDOWN);
	CHECK_UINT(next_arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(gap > CARRY_COOLDOWN);
	CHECK_UINT(info.diverge_resets, 0);
}

#define REPAY_BUILD		160u
#define REPAY_CLEAR_FRAMES	48u
#define REPAY_RESUME_FRAMES	100u
/* LEC_GEIGEL_RUN_DECAY of dsp/lec.c. */
#define REPAY_DECAY		4u

/*
 * The clear frames of an active window repay the stale run. After a run of 160,
 * the far end talks alone, and each clear frame repays 4. The run then reads
 * 160 less 4 per clear frame. A talker after it starts no trial. A run that is
 * not repaid reaches 200 and starts one.
 */
static void stale_repay_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_line line;
	unsigned built;
	unsigned clear = 0;
	unsigned fired = 0;
	unsigned repaid;
	unsigned longest = 0;
	unsigned f;

	memset(&reading, 0, sizeof(reading));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 0; f < EXPOSE_FRAMES && reading.stale_run < REPAY_BUILD; f++)
		return_white_run(&line, TALKER_ECHO_DBM0, &reading);
	built = reading.stale_run;
	for (f = 0; f < REPAY_CLEAR_FRAMES; f++) {
		ring_white_run(&line, 1u);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		udsp_ec_nlms_info_get(line.ec, &info);
		fired += reading.dt_fired;
		clear += !info.double_talk;
	}
	repaid = reading.stale_run;
	for (f = 0; f < REPAY_RESUME_FRAMES; f++) {
		return_white_run(&line, TALKER_ECHO_DBM0, &reading);
		if (reading.stale_run > longest)
			longest = reading.stale_run;
	}
	udsp_ec_nlms_info_get(line.ec, &info);
	udsp_ec_nlms.destroy(line.ec);

	case_rate(rate, "lec repays the stale run on the clear frames of an "
		  "active window");
	printf("# stale run %u under the talker; far end alone: %u clear "
	       "frames, %u fired, stale run %u against %u; talker again: "
	       "longest stale run %u, %u trials\n", built, clear, fired,
	       repaid, built - REPAY_DECAY * clear, longest, info.probes);
	CHECK_UINT(built, REPAY_BUILD);
	CHECK_UINT(fired, 0);
	CHECK(clear > 0u && REPAY_DECAY * clear < built);
	CHECK_UINT(repaid, built - REPAY_DECAY * clear);
	CHECK(longest < TALKER_STALE_FRAMES);
	CHECK_UINT(info.probes, 0);
}

/* The talker of the latch leg, 12 dB over the echo: the estimate holds under
 * 0.10 of its microphone energy, so the latch run grows on its frames. */
#define MARK_LATCH_DBM0		-10.0
#define MARK_LATCH_FRAMES	650u
#define MARK_PAUSE_PHASE	21u
#define MARK_PAUSE_FRAMES	(20u * FRAMES_PER_SECOND)

/* The step line of the talker cases: a white far end at TALKER_FAR_DBM0 and a
 * flat hybrid stepping from TALKER_STEP_FROM_DB to TALKER_STEP_TO_DB. A white
 * talker TALKER_OVER_DB over the far end joins on TALKER_TRIAL_FRAME. */
#define TALKER_FAR_DBM0		-20.0
#define TALKER_OVER_DB		16.0
#define TALKER_TRIAL_FRAME	10u
#define TALKER_STEP_FROM_DB	12.0
#define TALKER_STEP_TO_DB	3.0

/* One frame of the step line to one instance, with the talker when talk is
 * set. */
static void talker_step_frame_run(struct udsp_ec *ec, struct echo_path *path,
				  uint32_t *state, uint32_t *near_state,
				  bool talk)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	noise_fill(ref, path->frame, TALKER_FAR_DBM0, state);
	path_run(path, ref, mic);
	if (talk) {
		noise_fill(near, path->frame, TALKER_FAR_DBM0 + TALKER_OVER_DB,
			   near_state);
		for (i = 0; i < path->frame; i++)
			mic[i] = sample_clamp((double)mic[i] + near[i]);
	}
	udsp_ec_nlms.run(ec, mic, ref);
}

static void trial_word_check(void)
{
	static const struct {
		unsigned value;
		const char *word;
	} arms[] = {
		{ UDSP_EC_NLMS_TRIAL_ARM_NONE, "" },
		{ UDSP_EC_NLMS_TRIAL_ARM_LATCH, "latch" },
		{ UDSP_EC_NLMS_TRIAL_ARM_PATH, "path" },
		{ UDSP_EC_NLMS_TRIAL_ARM_STALE, "stale" },
		{ UDSP_EC_NLMS_TRIAL_ARM_COHERE, "cohere" },
		{ UDSP_EC_NLMS_TRIAL_ARM_COHERE + 1u, "?" },
	}, ends[] = {
		{ UDSP_EC_NLMS_TRIAL_END_NONE, "" },
		{ UDSP_EC_NLMS_TRIAL_END_KEPT, "kept" },
		{ UDSP_EC_NLMS_TRIAL_END_REJECTED, "rejected" },
		{ UDSP_EC_NLMS_TRIAL_END_DIVERGED, "diverged" },
		{ UDSP_EC_NLMS_TRIAL_END_TALKER, "talker" },
		{ UDSP_EC_NLMS_TRIAL_END_TALKER + 1u, "?" },
	};
	unsigned i;

	for (i = 0; i < sizeof(arms) / sizeof(arms[0]); i++)
		CHECK(!strcmp(lecr_trial_arm_name(arms[i].value),
			      arms[i].word));
	for (i = 0; i < sizeof(ends) / sizeof(ends[0]); i++)
		CHECK(!strcmp(lecr_trial_end_name(ends[i].value),
			      ends[i].word));
}

/* The reset of the instance on a frame that reads the arm and the end given.
 * The reading after the reset must read NONE in both fields. */
static void trial_reset_check(struct udsp_ec *ec, unsigned arm, unsigned end)
{
	struct udsp_ec_nlms_frame reading;

	udsp_ec_nlms_frame_get(ec, &reading);
	CHECK_UINT(reading.trial_arm, arm);
	CHECK_UINT(reading.trial_end, end);
	udsp_ec_nlms_reset(ec);
	udsp_ec_nlms_frame_get(ec, &reading);
	CHECK_UINT(reading.trial_arm, UDSP_EC_NLMS_TRIAL_ARM_NONE);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_NONE);
}

/*
 * The latch refusal of latch_refusal_test on the change line: a talker 12 dB
 * over the echo of a converged late hybrid (MARK_LATCH_DBM0) refuses the
 * latch trial. The instance resets on the end frame, and the frame after the
 * reset closes the trial.
 */
static void trial_latch_leg(void)
{
	struct trial_marks m = { .probes = 0 };
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state[3] = { 0x13572468u, 0xfeedfaceu, 0x2468acefu };
	double echo = 0.0;
	double res = 0.0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_late_init(&path, UDSP_SAMPLE_RATE, CHANGE_ERL_DB);
	for (f = 0; f < CHANGE_CONVERGE_FRAMES + MARK_LATCH_FRAMES; f++) {
		change_frame_run(ec, &path, state, f >= CHANGE_CONVERGE_FRAMES,
				 MARK_LATCH_DBM0, &echo, &res);
		trial_marks_add(&m, ec);
		if (m.last.trial_end != UDSP_EC_NLMS_TRIAL_END_NONE)
			break;
	}
	trial_reset_check(ec, UDSP_EC_NLMS_TRIAL_ARM_NONE,
			  UDSP_EC_NLMS_TRIAL_END_REJECTED);
	change_frame_run(ec, &path, state, true, MARK_LATCH_DBM0, &echo, &res);
	trial_marks_add(&m, ec);
	trial_marks_finish(&m);
	udsp_ec_nlms.destroy(ec);
	trial_marks_print("latch leg", &m);
	CHECK_UINT(m.starts, 1);
	CHECK_UINT(m.arm[0], UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK_UINT(m.ends, 1);
	CHECK_UINT(m.end[0], UDSP_EC_NLMS_TRIAL_END_REJECTED);
	trial_marks_check(&m);
}

/* The fixture of change_run. A talker at the echo level refuses a trial of
 * the stale run. The change to the late hybrid then starts a trial of the
 * path-change escape, and the line keeps it. */
static void trial_change_leg(void)
{
	struct trial_marks m = { .probes = 0 };
	struct change_result r;

	change_run(UDSP_SAMPLE_RATE, &r, &m);
	trial_marks_finish(&m);
	trial_marks_print("change leg", &m);
	CHECK_UINT(m.starts, 2);
	CHECK_UINT(m.arm[0], UDSP_EC_NLMS_TRIAL_ARM_STALE);
	CHECK_UINT(m.arm[1], UDSP_EC_NLMS_TRIAL_ARM_PATH);
	CHECK_UINT(m.ends, 2);
	CHECK_UINT(m.end[0], UDSP_EC_NLMS_TRIAL_END_REJECTED);
	CHECK_UINT(m.end[1], UDSP_EC_NLMS_TRIAL_END_KEPT);
	trial_marks_check(&m);
}

/*
 * The fixture of probe_pause_test: from word phase MARK_PAUSE_PHASE the latch
 * comes due inside a far pause and the trial waits for the next active frame;
 * at phase 0 none waits. Trial frames in the pauses do not adapt; the arm reads
 * on the start frame only, and the end on the end frame only.
 */
static void trial_pause_leg(void)
{
	struct trial_marks m = { .probes = 0 };
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(0);
	struct dtx_far far = { .state = 0x0c0ffee1u, .pos = MARK_PAUSE_PHASE };
	unsigned waits = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, 0.0);
	for (f = 0; f < MARK_PAUSE_FRAMES; f++) {
		struct udsp_ec_nlms_frame before = m.last;
		int16_t ref[UDSP_FRAME_SAMPLES];
		int16_t mic[UDSP_FRAME_SAMPLES];

		dtx_far_fill(&far, ref);
		path_run(&path, ref, mic);
		udsp_ec_nlms.run(ec, mic, ref);
		trial_marks_add(&m, ec);
		if (!frame_loud(ref, UDSP_FRAME_SAMPLES) && !m.last.probing &&
		    before.dt_run >= TALKER_LATCH_FRAMES)
			waits++;
	}
	trial_marks_finish(&m);
	udsp_ec_nlms.destroy(ec);
	trial_marks_print("pause leg", &m);
	printf("# pause leg: the latch was due on %u frames of a far-end "
	       "pause\n", waits);
	CHECK_UINT(m.starts, 1);
	CHECK(waits > 0u);
	CHECK_UINT(m.arm[0], UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK_UINT(m.ends, 1);
	CHECK_UINT(m.end[0], UDSP_EC_NLMS_TRIAL_END_KEPT);
	trial_marks_check(&m);
}

/*
 * The fixture of the first run of trial_talker_test: a talker 16 dB over the
 * far end ends a latch trial on trial frame 10. The latch then starts the
 * next trial, and the instance resets on its start frame, so that trial has
 * no end frame.
 */
static void trial_talker_leg(void)
{
	struct trial_marks m = { .probes = 0 };
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(48);
	uint32_t state = 0x13572468u;
	uint32_t near_state = 0xfeedfaceu;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, TALKER_STEP_FROM_DB);
	for (f = 0; f < FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2; f++) {
		talker_step_frame_run(ec, &path, &state, &near_state, false);
		trial_marks_add(&m, ec);
	}
	path_erl_set(&path, TALKER_STEP_TO_DB);
	for (f = 0; f < SNAP_LIMIT_FRAMES && !m.starts; f++) {
		talker_step_frame_run(ec, &path, &state, &near_state, false);
		trial_marks_add(&m, ec);
	}
	for (f = 1; f <= TALKER_TRIAL_FRAME; f++) {
		talker_step_frame_run(ec, &path, &state, &near_state,
				      f == TALKER_TRIAL_FRAME);
		trial_marks_add(&m, ec);
	}
	for (f = 0; f < SNAP_LIMIT_FRAMES && m.starts < 2u; f++) {
		talker_step_frame_run(ec, &path, &state, &near_state, false);
		trial_marks_add(&m, ec);
	}
	trial_reset_check(ec, UDSP_EC_NLMS_TRIAL_ARM_LATCH,
			  UDSP_EC_NLMS_TRIAL_END_NONE);
	talker_step_frame_run(ec, &path, &state, &near_state, false);
	trial_marks_add(&m, ec);
	trial_marks_finish(&m);
	udsp_ec_nlms.destroy(ec);
	trial_marks_print("talker leg", &m);
	CHECK_UINT(m.starts, 2);
	CHECK_UINT(m.arm[0], UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK_UINT(m.arm[1], UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK_UINT(m.ends, 1);
	CHECK_UINT(m.end[0], UDSP_EC_NLMS_TRIAL_END_TALKER);
	trial_marks_check(&m);
}

/* The fixture of probe_divergence_test: the hybrid that amplifies drives
 * latch trials through the watchdog bound. */
static void trial_diverge_leg(void)
{
	struct udsp_ec_nlms_info info;
	struct trial_marks m = { .probes = 0 };
	struct divergence_line d;
	unsigned latch = 0;
	unsigned diverged = 0;
	unsigned i;
	unsigned f;

	divergence_line_init(&d, UDSP_SAMPLE_RATE);
	for (f = 0; f < DIVERGE_WORDS_FRAMES + DIVERGE_AMP_FRAMES; f++) {
		divergence_frame_run(&d, f);
		trial_marks_add(&m, d.ec);
	}
	trial_marks_finish(&m);
	udsp_ec_nlms_info_get(d.ec, &info);
	udsp_ec_nlms.destroy(d.ec);
	for (i = 0; i < m.starts && i < MARK_TRIALS_MAX; i++)
		latch += m.arm[i] == UDSP_EC_NLMS_TRIAL_ARM_LATCH;
	for (i = 0; i < m.ends && i < MARK_TRIALS_MAX; i++)
		diverged += m.end[i] == UDSP_EC_NLMS_TRIAL_END_DIVERGED;
	trial_marks_print("divergence leg", &m);
	CHECK(m.starts <= MARK_TRIALS_MAX);
	CHECK_UINT(m.starts, info.probes);
	CHECK_UINT(latch, m.starts);
	CHECK(info.diverge_resets >= 1u);
	CHECK_UINT(diverged, info.diverge_resets);
	trial_marks_check(&m);
}

/*
 * lec_replay prints the trial fields of each frame: a start frame reads its arm
 * and probing 0, an end frame its end and probing 1. Every other frame reads
 * NONE, and a reset on either clears both. The lines cover each arm and each
 * end, and the words of lecr_trial_arm_name and lecr_trial_end_name match the
 * enums.
 */
static void trial_mark_test(void)
{
	CASE("lec frame reading names the arm and the end of each trial");
	trial_word_check();
	trial_latch_leg();
	trial_change_leg();
	trial_pause_leg();
	trial_talker_leg();
	trial_diverge_leg();
}

/* LEC_TALKER_RATIO of dsp/lec.c. */
#define TALKER_RATIO		16.0f
#define DENSE_PAUSE_FRAMES	15u

/*
 * A dense talker: a word source that starts its next word 15 frames after
 * each word ends (word_source_restart). The pauses are shorter than the long
 * hangover of 24 frames, so one verdict bridges a pause. The restart comes
 * before the shortest pause of the word source, so every word has the length
 * and the color of the first.
 */
struct dense_talker {
	struct word_source words;
	unsigned silent;
};

static void dense_talker_init(struct dense_talker *t, unsigned rate,
			      double level_dbm0, uint32_t seed)
{
	word_source_init(&t->words, rate, level_dbm0, seed);
	t->silent = 0;
}

static void dense_talker_fill(struct dense_talker *t, int16_t *dst,
			      unsigned frame)
{
	if (!t->words.talking && t->silent == DENSE_PAUSE_FRAMES)
		word_source_restart(&t->words);
	word_fill(&t->words, dst, frame);
	if (t->words.talking || t->words.offset)
		t->silent = 0;
	else
		t->silent++;
}

#define LATCH_FAN_DBM0		-26.0
#define LATCH_NEAR_DBM0		-10.0
#define LATCH_TRAIN_FRAMES	(3u * FRAMES_PER_SECOND)
#define LATCH_TALK_FRAMES	(30u * FRAMES_PER_SECOND)

/*
 * A near talker over a far end that never pauses: a dense talker at -10 dBm0
 * over a fan at -26 dBm0. Most of its word frames stand 12 dB over the fan, so
 * the talker guard clears the latch run on each word. No trial, no reset, no
 * exposure frame, and the near speech passes within 2 dB.
 */
static void latch_talker_test(unsigned rate)
{
	int16_t near[UDSP_FRAME_MAX];
	struct udsp_ec_nlms_info info;
	struct dense_talker talker;
	struct fan_line line;
	const struct near_stat *sp = &line.speech;
	unsigned f;

	fan_line_init(&line, rate, 0.0);
	fan_line_set(&line, LATCH_FAN_DBM0, FAN_ERL_DB);
	dense_talker_init(&talker, rate, LATCH_NEAR_DBM0, 0x0badcafeu);
	for (f = 0; f < LATCH_TRAIN_FRAMES; f++)
		info = fan_frame_run(&line, true, false, NULL);
	for (f = 0; f < LATCH_TALK_FRAMES; f++) {
		dense_talker_fill(&talker, near, line.frame);
		info = fan_frame_run(&line, true, false, near);
	}
	udsp_ec_nlms.destroy(line.ec);

	case_rate(rate, "lec keeps a near talker over a far-end fan out of "
		  "the latch");
	printf("# fan %.0f dBm0 on a %.0f dB hybrid, talker %.0f dBm0: %u "
	       "trials, %u resets; near speech %.2f dB through on %u frames, "
	       "output %.2f dB against the microphone, %u exposure frames\n",
	       line.fan_dbm0, line.erl_db, LATCH_NEAR_DBM0, info.probes,
	       info.diverge_resets, near_gain_db_get(sp->product, sp->near),
	       sp->frames, expose_db_get(sp->out, sp->mic), sp->exposure);
	CHECK_UINT(info.probes, 0);
	CHECK_UINT(info.diverge_resets, 0);
	CHECK(sp->frames > 0u);
	CHECK(fabs(near_gain_db_get(sp->product, sp->near)) <= EXPOSE_NEAR_DB);
	CHECK_UINT(sp->exposure, 0);
}

#define LOUDER_TALK_ERL_DB	12.0
#define LOUDER_TALK_STEP_DB	3.0
#define LOUDER_TALK_OVER_DB	9.0
#define LOUDER_TALK_FRAMES	600u

/*
 * A white near talker over a path that got louder with the same shape. The
 * residual echo carries a copy of the estimate, so the latch rule of
 * dsp/lec.c reads no independent talker (the LEC_LATCH_NCC_BAND comment).
 * The filter must be trusted on the step, and the latch must start a trial
 * within LOUDER_TALK_FRAMES frames of it.
 */
static void latch_louder_talker_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x9a8b7c6du;
	uint32_t near_state = 0x2468acefu;
	double near_dbm0 = -10.0 - LOUDER_TALK_ERL_DB + LOUDER_TALK_OVER_DB;
	unsigned arm = UDSP_EC_NLMS_TRIAL_ARM_NONE;
	unsigned started = 0;
	bool trusted;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, rate, 8, LOUDER_TALK_ERL_DB);
	(void)erle_run(ec, &path, &state, -10.0, 3u * FRAMES_PER_SECOND, 20);
	udsp_ec_nlms_frame_get(ec, &reading);
	udsp_ec_nlms_info_get(ec, &info);
	trusted = reading.trusted;
	path_erl_set(&path, LOUDER_TALK_ERL_DB - LOUDER_TALK_STEP_DB);
	for (f = 0; f < LOUDER_TALK_FRAMES && !started; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, path.frame, -10.0, &state);
		path_run(&path, ref, echo);
		noise_fill(near, path.frame, near_dbm0, &near_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_NONE)
			continue;
		started = f + 1u;
		arm = reading.trial_arm;
	}
	udsp_ec_nlms.destroy(ec);

	case_rate(rate, "lec latch starts a trial on a louder path under a "
		  "near talker");
	printf("# hybrid %.0f dB louder, talker %.0f dB over the old echo: "
	       "trusted %d on the step, %u trials before it; first trial on "
	       "frame %u, arm %s\n", LOUDER_TALK_STEP_DB, LOUDER_TALK_OVER_DB,
	       trusted, info.probes, started, lecr_trial_arm_name(arm));
	CHECK(trusted);
	CHECK_UINT(info.probes, 0);
	CHECK(started > 0u);
	CHECK_UINT(arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
}

#define ROLLBACK_ERL_DB		12.0
#define ROLLBACK_FAR_DBM0	-10.0
#define ROLLBACK_TALK_DBM0	-10.0
#define ROLLBACK_TRAIN_FRAMES	(3u * FRAMES_PER_SECOND)

/* Both instances of the rollback cases on the converged flat hybrid: a white
 * far end, no near end, the processor off. */
static void rollback_line_train(struct udsp_ec *lead, struct udsp_ec *twin,
				struct echo_path *path, uint32_t *state)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t lead_out[UDSP_FRAME_MAX];
	int16_t twin_out[UDSP_FRAME_MAX];
	unsigned f;

	udsp_ec_nlms.nlp(lead, false);
	udsp_ec_nlms.nlp(twin, false);
	for (f = 0; f < ROLLBACK_TRAIN_FRAMES; f++) {
		noise_fill(ref, path->frame, ROLLBACK_FAR_DBM0, state);
		twin_step(lead, twin, path, ref, NULL, lead_out, twin_out,
			  NULL);
	}
}

/*
 * The output of a frame that the detector rolls back (the frame_undo comment
 * of dsp/lec.c). A lead and a twin converge a flat hybrid on identical
 * frames. A near talker at the far-end level then starts halfway into a
 * frame on which the twin is frozen. The lead must fire on that frame and
 * roll it back, and its output must equal the twin's, sample for sample.
 */
static void rollback_output_test(unsigned rate)
{
	struct udsp_ec_nlms_frame before;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info twin_info;
	struct echo_path path;
	struct udsp_ec *lead = udsp_ec_nlms.create_rate(48, rate);
	struct udsp_ec *twin = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x13572468u;
	uint32_t near_state = 0xfeedfaceu;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	int16_t lead_out[UDSP_FRAME_MAX];
	int16_t twin_out[UDSP_FRAME_MAX];
	unsigned differ = 0;
	unsigned i;

	path_flat_init(&path, rate, 8, ROLLBACK_ERL_DB);
	rollback_line_train(lead, twin, &path, &state);
	udsp_ec_nlms_frame_get(lead, &before);
	noise_fill(ref, path.frame, ROLLBACK_FAR_DBM0, &state);
	noise_fill(near, path.frame, ROLLBACK_TALK_DBM0, &near_state);
	for (i = 0; i < path.frame / 2u; i++)
		near[i] = 0;
	udsp_ec_nlms.freeze(twin, true);
	twin_step(lead, twin, &path, ref, near, lead_out, twin_out, NULL);
	udsp_ec_nlms_frame_get(lead, &reading);
	udsp_ec_nlms_info_get(lead, &info);
	udsp_ec_nlms_info_get(twin, &twin_info);
	for (i = 0; i < path.frame; i++)
		differ += lead_out[i] != twin_out[i];
	udsp_ec_nlms.destroy(lead);
	udsp_ec_nlms.destroy(twin);

	case_rate(rate, "lec rolled-back frame emits the output of the "
		  "coefficients it started with");
	printf("# talker frame: trusted %d before it, fired %d, coefficient "
	       "energy %.9f against the twin's %.9f; %u of %u samples differ "
	       "from the frozen twin\n", before.trusted, reading.dt_fired,
	       (double)info.coef_energy, (double)twin_info.coef_energy,
	       differ, path.frame);
	CHECK(before.trusted);
	CHECK(!before.dt_fired);
	CHECK_UINT(before.hold_left, 0);
	CHECK(reading.dt_fired);
	CHECK(!reading.probing);
	CHECK(info.coef_energy == twin_info.coef_energy);
	CHECK_UINT(differ, 0);
}

#define MASK_STEP_DB		6.0
#define MASK_OVER_DB		2.5
#define MASK_NEXT_FRAMES	3u
#define MASK_RUN_FRAMES		(3u * FRAMES_PER_SECOND)
/* The rolled-back frames of the masking run at 8 and 16 kHz. */
#define MASK_ROLLBACKS		1u

static double frame_power_get(const int16_t *x, unsigned n)
{
	double sum = 0.0;
	unsigned i;

	for (i = 0; i < n; i++)
		sum += (double)x[i] * x[i];
	return sum / n;
}

/*
 * The cost of the rollback output on a path change (frame_undo): after the
 * training of rollback_output_test the hybrid turns MASK_STEP_DB louder. The
 * first rolled-back frame carries the echo of its start coefficients and stands
 * MASK_OVER_DB or less over the loudest next held frame.
 */
static void rollback_mask_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *lead = udsp_ec_nlms.create_rate(48, rate);
	struct udsp_ec *twin = udsp_ec_nlms.create_rate(48, rate);
	uint32_t state = 0x2468acefu;
	double power[MASK_NEXT_FRAMES + 1u] = { 0.0 };
	double loudest = 0.0;
	double over_db;
	unsigned rollbacks = 0;
	unsigned held = 0;
	int after = -1;
	bool double_talk;
	unsigned f;

	path_flat_init(&path, rate, 8, ROLLBACK_ERL_DB);
	rollback_line_train(lead, twin, &path, &state);
	udsp_ec_nlms.destroy(twin);
	udsp_ec_nlms_info_get(lead, &info);
	double_talk = info.double_talk;
	path_erl_set(&path, ROLLBACK_ERL_DB - MASK_STEP_DB);
	for (f = 0; f < MASK_RUN_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		bool active;
		bool rollback;

		noise_fill(ref, path.frame, ROLLBACK_FAR_DBM0, &state);
		path_run(&path, ref, echo);
		udsp_ec_nlms.run(lead, echo, ref);
		udsp_ec_nlms_frame_get(lead, &reading);
		udsp_ec_nlms_info_get(lead, &info);
		active = reading.ref_power >= reading.ref_bar;
		rollback = active && reading.dt_fired && !double_talk &&
			   !reading.probing;
		rollbacks += rollback;
		if (after < 0 && rollback) {
			after = 0;
			power[0] = frame_power_get(echo, path.frame);
		} else if (after >= 0 && after < (int)MASK_NEXT_FRAMES) {
			power[++after] = frame_power_get(echo, path.frame);
			held += info.double_talk;
		}
		double_talk = info.double_talk;
	}
	udsp_ec_nlms.destroy(lead);
	for (f = 1; after == (int)MASK_NEXT_FRAMES && f <= MASK_NEXT_FRAMES;
	     f++) {
		if (power[f] > loudest)
			loudest = power[f];
	}
	over_db = after == (int)MASK_NEXT_FRAMES ?
		  10.0 * log10((power[0] + 1.0) / (loudest + 1.0)) : 200.0;

	case_rate(rate, "lec rolled-back frame of a path change stands under "
		  "the held frames after it");
	printf("# hybrid %.0f dB louder: %u rolled-back frames in 3 s; the "
	       "first stands %.2f dB against the loudest of the next %u, %u "
	       "of them held\n", MASK_STEP_DB, rollbacks, over_db,
	       MASK_NEXT_FRAMES, held);
	CHECK_UINT(rollbacks, MASK_ROLLBACKS);
	CHECK_UINT(held, MASK_NEXT_FRAMES);
	CHECK(over_db <= MASK_OVER_DB);
}

/*
 * The first run of trial_talker_test: a flat 12 dB hybrid converges at -20 dBm0
 * and steps to 3 dB, and the latch starts a trial on a held frame. The twin
 * freezes after it, and on trial frame 10 a talker 16 dB over the far end
 * joins.
 */
static void trial_talker_step_check(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info twin_info;
	struct echo_path path;
	struct udsp_ec *lead = udsp_ec_nlms.create(48);
	struct udsp_ec *twin = udsp_ec_nlms.create(48);
	uint32_t state = 0x13572468u;
	uint32_t near_state = 0xfeedfaceu;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t near[UDSP_FRAME_SAMPLES];
	int16_t lead_out[UDSP_FRAME_SAMPLES];
	int16_t twin_out[UDSP_FRAME_SAMPLES];
	unsigned started = 0;
	unsigned f;

	memset(&info, 0, sizeof(info));
	udsp_ec_nlms.nlp(lead, false);
	udsp_ec_nlms.nlp(twin, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, TALKER_STEP_FROM_DB);
	for (f = 0; f < FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, TALKER_FAR_DBM0, &state);
		twin_step(lead, twin, &path, ref, NULL, lead_out, twin_out,
			  NULL);
	}
	path_erl_set(&path, TALKER_STEP_TO_DB);
	for (f = 0; f < SNAP_LIMIT_FRAMES && !started; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, TALKER_FAR_DBM0, &state);
		twin_step(lead, twin, &path, ref, NULL, lead_out, twin_out,
			  NULL);
		udsp_ec_nlms_info_get(lead, &info);
		if (info.probes)
			started = f + 1u;
	}
	udsp_ec_nlms.freeze(twin, true);
	for (f = 1; f < TALKER_TRIAL_FRAME; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, TALKER_FAR_DBM0, &state);
		twin_step(lead, twin, &path, ref, NULL, lead_out, twin_out,
			  NULL);
	}
	udsp_ec_nlms_info_get(lead, &before);
	noise_fill(ref, UDSP_FRAME_SAMPLES, TALKER_FAR_DBM0, &state);
	noise_fill(near, UDSP_FRAME_SAMPLES, TALKER_FAR_DBM0 + TALKER_OVER_DB,
		   &near_state);
	twin_step(lead, twin, &path, ref, near, lead_out, twin_out, NULL);
	udsp_ec_nlms_frame_get(lead, &reading);
	udsp_ec_nlms_info_get(lead, &info);
	udsp_ec_nlms_info_get(twin, &twin_info);
	CHECK(started > 0u);
	CHECK(reading.talker);
	CHECK(reading.probing);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK_UINT(info.hold_frames - before.hold_frames, 1);
	CHECK(info.coef_energy == twin_info.coef_energy);
	printf("# step trial from frame %u after the step; talker frame: "
	       "mic %.2f dB over the window, held %u; coefficient energy %.9f "
	       "against the twin's %.9f\n", started,
	       10.0 * log10((double)reading.mic_power / reading.window_power),
	       info.hold_frames - before.hold_frames,
	       (double)info.coef_energy, (double)twin_info.coef_energy);
	noise_fill(ref, UDSP_FRAME_SAMPLES, TALKER_FAR_DBM0, &state);
	twin_step(lead, twin, &path, ref, NULL, lead_out, twin_out, NULL);
	udsp_ec_nlms_frame_get(lead, &reading);
	udsp_ec_nlms_info_get(lead, &info);
	CHECK(!reading.probing);
	CHECK_UINT(info.probes, before.probes);
	CHECK_UINT(info.diverge_resets, before.diverge_resets);
	udsp_ec_nlms.destroy(lead);
	udsp_ec_nlms.destroy(twin);
}

/*
 * The second run: the latch starts a trial after the hangover ran out, so the
 * snapshot holds no double talk. On trial frame 10 a talker 16 dB over the far
 * end replaces the talker 3 dB over the echo. That frame's output equals the
 * frozen twin's, sample for sample.
 */
static void trial_talker_snap_check(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct snap_line s;
	unsigned started;
	unsigned f;

	snap_line_build(&s, TALKER_FAR_DBM0);
	udsp_ec_nlms.freeze(s.twin, true);
	for (f = 0; f < TALKER_TRIAL_FRAME; f++)
		snap_frame_run(&s, snap_talk_get(&s, TALKER_FAR_DBM0 -
						 SNAP_TALK_UNDER_DB));
	udsp_ec_nlms_info_get(s.lead, &info);
	started = info.probes;
	snap_frame_run(&s, snap_talk_get(&s, TALKER_FAR_DBM0 + TALKER_OVER_DB));
	udsp_ec_nlms_frame_get(s.lead, &reading);
	udsp_ec_nlms_info_get(s.lead, &info);
	printf("# snapshot trial: latch run %u after the pause; talker frame: "
	       "end %s, output %s the twin's\n", s.run_before,
	       lecr_trial_end_name(reading.trial_end),
	       memcmp(s.lead_out, s.twin_out, sizeof(s.lead_out)) ?
	       "differs from" : "equals");
	CHECK(s.clear_before);
	CHECK(s.run_before >= SNAP_LATCH_FRAMES);
	CHECK_UINT(started, 1);
	CHECK(reading.talker);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK(!memcmp(s.lead_out, s.twin_out, sizeof(s.lead_out)));
	CHECK_UINT(info.probes, 1);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(s.lead);
	udsp_ec_nlms.destroy(s.twin);
}

/*
 * A talker 16 dB over the far end on frame 10 of a running trial: the frame
 * reads the talker, probing and the end talker, with one held frame. The next
 * reads no trial, with no new trial or reset. The energy and, in a second run,
 * the output equal those of a twin frozen on the start frame.
 */
static void trial_talker_test(void)
{
	CASE("lec ends a trial on the first frame of a near talker");
	trial_talker_step_check();
	trial_talker_snap_check();
}

/* The far end after the pause of latch_return_test, under its level before
 * the pause. */
#define RETURN_QUIET_DB		8.0

/*
 * The latch comes due on a talker frame: after a pause, the far end returns 8
 * dB quieter under a talker 16 dB over it. The talker clears the latch run
 * before the arms read it, so no trial starts in the next 100 frames. The guard
 * window follows the silent pause frames, not the louder far end before.
 */
static void latch_return_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct snap_line s;
	unsigned at_return;
	unsigned f;

	snap_line_build(&s, TALKER_FAR_DBM0);
	s.far_dbm0 = TALKER_FAR_DBM0 - RETURN_QUIET_DB;
	snap_frame_run(&s, snap_talk_get(&s, s.far_dbm0 + TALKER_OVER_DB));
	udsp_ec_nlms_frame_get(s.lead, &reading);
	udsp_ec_nlms_info_get(s.lead, &info);
	at_return = info.probes;
	for (f = 0; f < TRIAL_BUDGET; f++)
		snap_frame_run(&s, snap_talk_get(&s, s.far_dbm0 -
						 SNAP_TALK_UNDER_DB));
	udsp_ec_nlms_info_get(s.lead, &info);
	udsp_ec_nlms.destroy(s.lead);
	udsp_ec_nlms.destroy(s.twin);

	CASE("lec starts no latch trial on the frame a loud talker returns");
	printf("# latch run %u after the pause, double talk %u at its end; "
	       "return frame %.0f dB quieter: talker %u, mic %.2f dB over the "
	       "window, %u trials; %u trials after %u more frames\n",
	       s.run_before, !s.clear_before, RETURN_QUIET_DB, reading.talker,
	       10.0 * log10((double)reading.mic_power / reading.window_power),
	       at_return, info.probes, TRIAL_BUDGET);
	CHECK(s.clear_before);
	CHECK(s.run_before >= SNAP_LATCH_FRAMES);
	CHECK(reading.talker);
	CHECK_UINT(at_return, 0);
	CHECK_UINT(info.probes, 0);
}

#define TAIL_ERL_DB		6.0
#define TAIL_WORDS_DBM0		-20.0
#define TAIL_NOISE_DBM0		-48.0
#define TAIL_FRAMES		(10u * FRAMES_PER_SECOND)
#define TAIL_FEED_FRAMES	8u

/* One frame of the tail line: far-end words on a white noise at
 * TAIL_NOISE_DBM0, through the hybrid, with the near end silent. */
static void tail_frame_run(struct udsp_ec *ec, struct echo_path *path,
			   struct word_source *far, uint32_t *state,
			   struct udsp_ec_nlms_frame *reading)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t words[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	noise_fill(ref, path->frame, TAIL_NOISE_DBM0, state);
	word_fill(far, words, path->frame);
	for (i = 0; i < path->frame; i++)
		ref[i] = sample_clamp((double)ref[i] + words[i]);
	path_run(path, ref, mic);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_frame_get(ec, reading);
}

/* Runs the tail line to a frame inside the body of a word. */
static void tail_word_run(struct udsp_ec *ec, struct echo_path *path,
			  struct word_source *far, uint32_t *state)
{
	struct udsp_ec_nlms_frame reading;

	do {
		tail_frame_run(ec, path, far, state, &reading);
	} while (!far->talking || far->pos <= WORD_ONSET_FRAMES);
}

/* One frame of the noise alone, and a microphone of a white talker 16 dB
 * over it and nothing else. It returns the talker verdict of the frame. */
static bool tail_talker_get(struct udsp_ec *ec, unsigned frame,
			    uint32_t *state)
{
	struct udsp_ec_nlms_frame reading;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];

	noise_fill(ref, frame, TAIL_NOISE_DBM0, state);
	noise_fill(mic, frame, TAIL_NOISE_DBM0 + TALKER_OVER_DB, state);
	udsp_ec_nlms.run(ec, mic, ref);
	udsp_ec_nlms_frame_get(ec, &reading);
	return reading.talker;
}

/*
 * A late echo is not a talker: a flat 6 dB hybrid 1 ms short of the tail and
 * far words that stop into noise. Every frame reads no talker, because the
 * frame before covers the word end's echo. After a reset and after a feed, a
 * white talker 16 dB over the noise still reads as a talker.
 */
static void talker_tail_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct word_source far;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state = 0x2718281du;
	int16_t ref[UDSP_FRAME_MAX];
	unsigned talker = 0;
	unsigned window = 0;
	bool after_reset;
	bool after_feed;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	udsp_ec_nlms_info_get(ec, &info);
	path_flat_init(&path, rate, info.tail_ms - 1u, TAIL_ERL_DB);
	word_source_init(&far, rate, TAIL_WORDS_DBM0, 0x13572468u);
	for (f = 0; f < TAIL_FRAMES; f++) {
		tail_frame_run(ec, &path, &far, &state, &reading);
		talker += reading.talker;
		if (reading.window_run &&
		    reading.mic_power > TALKER_RATIO * reading.window_power)
			window++;
	}
	tail_word_run(ec, &path, &far, &state);
	udsp_ec_nlms_reset(ec);
	after_reset = tail_talker_get(ec, path.frame, &state);
	tail_word_run(ec, &path, &far, &state);
	for (f = 0; f < TAIL_FEED_FRAMES; f++) {
		noise_fill(ref, path.frame, TAIL_NOISE_DBM0, &state);
		udsp_ec_nlms_ref_feed(ec, ref);
	}
	after_feed = tail_talker_get(ec, path.frame, &state);
	udsp_ec_nlms.destroy(ec);

	case_rate(rate, "lec reads no talker in the late echo of the tail");
	printf("# %u ms hybrid delay: %u of %u frames read a talker, %u read "
	       "one by the window of the frame alone; talker after a reset "
	       "%u, after a feed %u\n", info.tail_ms - 1u, talker, TAIL_FRAMES,
	       window, after_reset, after_feed);
	CHECK(window > 0u);
	CHECK_UINT(talker, 0);
	CHECK(after_reset);
	CHECK(after_feed);
}

/* LEC_REF_ACTIVE_POWER and LEC_REF_FLOOR_MARGIN of dsp/lec.c, and the mean
 * square of the identity, the fixed bar over the margin. */
#define REF_BAR_FIXED		2500.0f
#define REF_FLOOR_MARGIN	6.3
#define REF_FLOOR_IDENTITY	396.8f

/*
 * The cap of the activity bar by the formula of dsp/lec.c, from the learned
 * gain that the introspection read after the frame before, in the float
 * arithmetic of ref_bar_max_get: LEC_GEIGEL_GAIN_HEADROOM, LEC_GEIGEL_GAIN_MIN
 * and LEC_REF_RETURN_MAX.
 */
static float cap_bar_max_get(float gain)
{
	float bound = gain * 2.0f;
	float bar_max;

	if (bound > 0.25f)
		bound = 0.25f;
	bar_max = 82.4f / bound;
	return bar_max > REF_BAR_FIXED ? bar_max : REF_BAR_FIXED;
}

/*
 * The frames of one far-floor line that break a rule. Such a frame has a bar
 * over the cap or a cap off the gain formula. A bar that is not fixed or a gain
 * that rose also breaks a rule. restores counts the trials whose end frame left
 * another gain; there the cap still reads the gain before the frame.
 */
struct cap_stat {
	unsigned frames;
	unsigned over;
	unsigned wrong;
	unsigned raised;
	unsigned rises;
	unsigned restores;
};

/* A fixture calls it after each frame, with the gain before the frame. */
static void cap_check(struct cap_stat *st, const struct udsp_ec *ec,
		      float gain)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	bool restore;

	udsp_ec_nlms_frame_get(ec, &reading);
	udsp_ec_nlms_info_get(ec, &info);
	restore = reading.trial_end == UDSP_EC_NLMS_TRIAL_END_REJECTED ||
		  reading.trial_end == UDSP_EC_NLMS_TRIAL_END_TALKER;
	st->frames++;
	st->over += reading.ref_bar > reading.ref_bar_max;
	st->wrong += reading.ref_bar_max != cap_bar_max_get(gain);
	st->raised += reading.ref_bar != REF_BAR_FIXED;
	st->rises += info.echo_gain > gain;
	st->restores += restore && info.echo_gain != gain;
}

static void cap_print(const char *what, const struct cap_stat *st)
{
	printf("# %s: %u frames, %u bars over the cap, %u caps off the "
	       "formula, %u raised bars, %u frames with a rising gain, %u "
	       "trial ends that restored another gain\n", what, st->frames,
	       st->over, st->wrong, st->raised, st->rises, st->restores);
}

/* The bounds of the cap that every far-floor line keeps. */
static void cap_bounds_check(const struct cap_stat *st)
{
	CHECK(st->frames > 0u);
	CHECK_UINT(st->over, 0);
	CHECK_UINT(st->wrong, 0);
}

/* One frame of the fan line, and the cap check of the frame. */
static struct udsp_ec_nlms_info fan_cap_frame_run(struct fan_line *line,
						  bool fan, bool words,
						  const int16_t *near,
						  struct cap_stat *cap)
{
	struct udsp_ec_nlms_info info;
	float gain = 0.0f;

	udsp_ec_nlms_info_get(line->ec, &info);
	gain = info.echo_gain;
	info = fan_frame_run(line, fan, words, near);
	cap_check(cap, line->ec, gain);
	return info;
}

/* Far words in phrases of 1.2 s in cycles of 1.8 s. A line that decays the
 * latch run in a pause loses more there than a phrase adds. The pauses are
 * therefore the harder test of the background rule. */
#define PHRASE_FRAMES		120u
#define PHRASE_CYCLE_FRAMES	180u

/* index counts the frames of the schedule from 0, and silent the frames since
 * the last far word. */
struct phrase_words {
	unsigned index;
	unsigned silent;
};

/*
 * One frame of the fan line with the far words of the phrase schedule. Inside
 * a phrase the next word starts 15 frames after a word ends
 * (DENSE_PAUSE_FRAMES), the restart of the dense talker.
 */
static struct udsp_ec_nlms_info phrase_frame_run(struct fan_line *line,
						 struct phrase_words *pw,
						 const int16_t *near,
						 struct cap_stat *cap)
{
	struct udsp_ec_nlms_info info;
	unsigned p = pw->index++ % PHRASE_CYCLE_FRAMES;
	bool words = p < PHRASE_FRAMES;

	if (p == 0u || (words && !line->far.talking &&
			pw->silent == DENSE_PAUSE_FRAMES))
		word_source_restart(&line->far);
	info = fan_cap_frame_run(line, true, words, near, cap);
	pw->silent = line->word_loud ? 0u : pw->silent + 1u;
	return info;
}

/* The line of far_floor_talker_test: the fan line with a flat FLOOR_ERL_DB
 * hybrid, whose cap stands at -38 dBm0, and a fan at -45 dBm0. A dense talker
 * under the talker guard talks from FLOOR_TALK_FROM to FLOOR_TALK_TO. */
#define FLOOR_FAN_DBM0		-45.0
#define FLOOR_ERL_DB		30.0
#define FLOOR_TALK_DBM0		-42.0
#define FLOOR_SILENT_FRAMES	(2u * FRAMES_PER_SECOND)
#define FLOOR_WORDS_TO		(FLOOR_SILENT_FRAMES + 5u * FRAMES_PER_SECOND)
#define FLOOR_TALK_FROM		(15u * FRAMES_PER_SECOND)
#define FLOOR_TALK_TO		(55u * FRAMES_PER_SECOND)
#define FLOOR_FRAMES		(FLOOR_SILENT_FRAMES + 60u * FRAMES_PER_SECOND)
#define FLOOR_QUIET_SHARE	0.90

/*
 * One leg of far_floor_talker_test: from and to hold the state at the start and
 * end of the talk, and end at the end of the line. The counts cover the fan
 * frames after 15 s. With words set, far words go on through the talk, so that
 * leg can still latch, and its trials are reported with no bar.
 */
struct floor_talk {
	struct udsp_ec_nlms_info from;
	struct udsp_ec_nlms_info to;
	struct udsp_ec_nlms_info end;
	struct near_stat speech;
	struct trial_marks marks;
	struct cap_stat cap;
	unsigned fan_frames;
	unsigned quiet;
	unsigned background;
};

static void floor_talk_run(unsigned rate, bool words, struct floor_talk *t)
{
	int16_t near[UDSP_FRAME_MAX];
	struct udsp_ec_nlms_frame reading;
	struct phrase_words pw = { .index = 0 };
	struct dense_talker talker;
	struct fan_line line;
	unsigned f;

	memset(t, 0, sizeof(*t));
	fan_line_init(&line, rate, 0.0);
	fan_line_set(&line, FLOOR_FAN_DBM0, FLOOR_ERL_DB);
	dense_talker_init(&talker, rate, FLOOR_TALK_DBM0, 0x0badcafeu);
	for (f = 0; f < FLOOR_FRAMES; f++) {
		bool fan = f >= FLOOR_SILENT_FRAMES;
		bool talk = f >= FLOOR_TALK_FROM && f < FLOOR_TALK_TO;

		if (talk)
			dense_talker_fill(&talker, near, line.frame);
		if (words && talk)
			t->end = phrase_frame_run(&line, &pw, near, &t->cap);
		else
			t->end = fan_cap_frame_run(&line, fan,
						   fan && f < FLOOR_WORDS_TO,
						   talk ? near : NULL, &t->cap);
		trial_marks_add(&t->marks, line.ec);
		if (f + 1u == FLOOR_TALK_FROM)
			t->from = t->end;
		if (f + 1u == FLOOR_TALK_TO)
			t->to = t->end;
		if (f < FLOOR_TALK_FROM)
			continue;
		udsp_ec_nlms_frame_get(line.ec, &reading);
		t->fan_frames++;
		t->quiet += !reading.window_run;
		t->background += reading.background;
	}
	trial_marks_finish(&t->marks);
	t->speech = line.speech;
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A dense near talker under a stationary far end that the far-end floor reads
 * as quiet. No trial or reset runs during the talk, and 90 percent or more of
 * the fan frames read a quiet window and a background. The floor ends near -72
 * dBm0, and near speech passes within 2 dB with no exposure frame.
 */
static void far_floor_talker_test(unsigned rate)
{
	double floor = floor_dbov_get(FAN_FLOOR_DBM0);
	const struct near_stat *sp;
	const struct near_stat *wp;
	struct floor_talk alone;
	struct floor_talk over;

	floor_talk_run(rate, false, &alone);
	floor_talk_run(rate, true, &over);
	sp = &alone.speech;
	wp = &over.speech;

	case_rate(rate, "lec keeps a near talker under a stationary far end "
		  "out of double talk");
	printf("# fan %.0f dBm0 on a %.0f dB hybrid, talker %.0f dBm0: %u "
	       "trials and %u resets while the talker speaks, %u and %u in the "
	       "run; %u of %u fan frames after 15 s in a quiet window; floor "
	       "%.2f dBov against %.2f dBov; near speech %.2f dB through on %u "
	       "frames, output %.2f dB against the microphone, %u exposure "
	       "frames\n", FLOOR_FAN_DBM0, FLOOR_ERL_DB, FLOOR_TALK_DBM0,
	       alone.to.probes - alone.from.probes,
	       alone.to.diverge_resets - alone.from.diverge_resets,
	       alone.end.probes, alone.end.diverge_resets, alone.quiet,
	       alone.fan_frames, (double)alone.end.noise_dbov, floor,
	       near_gain_db_get(sp->product, sp->near), sp->frames,
	       expose_db_get(sp->out, sp->mic), sp->exposure);
	printf("# %u of %u fan frames after 15 s read a background\n",
	       alone.background, alone.fan_frames);
	cap_print("talker alone", &alone.cap);
	trial_marks_print("talker over far words", &over.marks);
	printf("# talker over far words: %u resets; near speech %.2f dB "
	       "through on %u frames, output %.2f dB against the microphone, "
	       "%u exposure frames\n", over.end.diverge_resets,
	       near_gain_db_get(wp->product, wp->near), wp->frames,
	       expose_db_get(wp->out, wp->mic), wp->exposure);
	cap_print("talker over far words", &over.cap);
	CHECK_UINT(alone.to.probes - alone.from.probes, 0);
	CHECK_UINT(alone.to.diverge_resets - alone.from.diverge_resets, 0);
	CHECK(alone.quiet >= FLOOR_QUIET_SHARE * alone.fan_frames);
	CHECK_UINT(alone.background, alone.fan_frames);
	CHECK(fabs(alone.end.noise_dbov - floor) <= FAN_FLOOR_TOL_DB);
	CHECK(sp->frames > 0u);
	CHECK(fabs(near_gain_db_get(sp->product, sp->near)) <= EXPOSE_NEAR_DB);
	CHECK_UINT(sp->exposure, 0);
	cap_bounds_check(&alone.cap);
	CHECK_UINT(over.end.diverge_resets, 0);
	CHECK(wp->frames > 0u);
	CHECK(fabs(near_gain_db_get(wp->product, wp->near)) <= EXPOSE_NEAR_DB);
	cap_bounds_check(&over.cap);
}

#define IDENT_FRAMES		(30u * FRAMES_PER_SECOND)
#define IDENT_FAN_DBM0		-45.0

/* The far ends of far_floor_identity_test. The fan is the negative
 * control. */
enum ident_far {
	IDENT_FAR_WORDS,
	IDENT_FAR_RING,
	IDENT_FAR_SPEECH,
	IDENT_FAR_DTX,
	IDENT_FAR_FAN,
};

/*
 * The twins of far_floor_identity_test: the instance of a ringback line,
 * which runs with the far-end floor on, and a twin that runs with it off.
 * Both have the processor on. The counts take the frames whose outputs
 * differ, the frames whose introspection differs, and the frames on which
 * the bar of the first instance stands over the fixed bar.
 */
struct ident_line {
	struct ring_line line;
	struct udsp_ec *twin;
	unsigned differ;
	unsigned states;
	unsigned raised;
};

static void ident_line_init(struct ident_line *l, unsigned rate)
{
	memset(l, 0, sizeof(*l));
	ring_line_init(&l->line, rate);
	l->twin = udsp_ec_nlms.create_rate(0, rate);
	udsp_ec_nlms.nlp(l->line.ec, true);
	udsp_ec_nlms.nlp(l->twin, true);
	udsp_ec_nlms_ref_floor_set(l->line.ec, true);
	udsp_ec_nlms_ref_floor_set(l->twin, false);
}

/* One frame to both instances: the lead runs with the floor switch on, the
 * twin with it off. */
static void ident_step(struct ident_line *l, const int16_t *ref)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info lead_info;
	struct udsp_ec_nlms_info twin_info;
	struct ring_planes p;
	int16_t out[UDSP_FRAME_MAX];
	size_t bytes = l->line.frame * sizeof(*out);

	ring_planes_run(&l->line, ref, NULL, NULL, &p);
	memcpy(out, p.mic, bytes);
	udsp_ec_nlms.run(l->twin, out, ref);
	udsp_ec_nlms_frame_get(l->line.ec, &reading);
	udsp_ec_nlms_info_get(l->line.ec, &lead_info);
	udsp_ec_nlms_info_get(l->twin, &twin_info);
	l->differ += memcmp(out, p.out, bytes) != 0;
	/* zero_skip_info_same leaves skip_frames out, because its first
	 * callers compare a full form against a skip form. The twins here run
	 * one form, so they must also skip the same frames. */
	l->states += !zero_skip_info_same(&lead_info, &twin_info) ||
		     lead_info.skip_frames != twin_info.skip_frames;
	l->raised += reading.ref_bar != REF_BAR_FIXED;
}

/* 30 s of one far end through the twins. far_speech_fill and dtx_far_fill
 * exist at 8 kHz only. */
static void ident_run(enum ident_far far, unsigned rate, struct ident_line *l)
{
	struct dtx_far dtx = { .state = 0x0c0ffee1u };
	uint32_t state = 0x13572468u;
	int16_t ref[UDSP_FRAME_MAX];
	double m1 = 0.0;
	double m2 = 0.0;
	unsigned f;

	ident_line_init(l, rate);
	for (f = 0; f < IDENT_FRAMES; f++) {
		switch (far) {
		case IDENT_FAR_RING:
			if (f < RING_FRAMES) {
				ring_fill(&l->line, ref, f);
				break;
			}
			word_fill(&l->line.far, ref, l->line.frame);
			break;
		case IDENT_FAR_WORDS:
			word_fill(&l->line.far, ref, l->line.frame);
			break;
		case IDENT_FAR_SPEECH:
			far_speech_fill(ref, f, &state, &m1, &m2);
			break;
		case IDENT_FAR_DTX:
			dtx_far_fill(&dtx, ref);
			break;
		case IDENT_FAR_FAN:
			noise_fill(ref, l->line.frame, IDENT_FAN_DBM0, &state);
			break;
		}
		ident_step(l, ref);
	}
	udsp_ec_nlms.destroy(l->line.ec);
	udsp_ec_nlms.destroy(l->twin);
}

static void ident_print(const char *what, unsigned rate,
			const struct ident_line *l)
{
	printf("# %s at %u Hz: %u of %u frames differ, %u states differ, %u "
	       "frames read a raised bar\n", what, rate, l->differ,
	       IDENT_FRAMES, l->states, l->raised);
}

/*
 * The identity of the far-end floor: twins with the switch on and off take far
 * ends that pause to silence. These are the word source, the ringback, and the
 * speech-shaped and DTX far ends at 8 kHz. Outputs and introspection match, and
 * the bar reads the fixed bar; a fan at -45 dBm0 must make them differ.
 */
static void far_floor_identity_test(void)
{
	static const struct {
		enum ident_far far;
		unsigned rate;
		const char *what;
	} runs[] = {
		{ IDENT_FAR_WORDS, UDSP_SAMPLE_RATE, "words" },
		{ IDENT_FAR_WORDS, UDSP_SAMPLE_RATE_WB, "words" },
		{ IDENT_FAR_RING, UDSP_SAMPLE_RATE, "ringback line" },
		{ IDENT_FAR_RING, UDSP_SAMPLE_RATE_WB, "ringback line" },
		{ IDENT_FAR_SPEECH, UDSP_SAMPLE_RATE, "speech-shaped far end" },
		{ IDENT_FAR_DTX, UDSP_SAMPLE_RATE, "DTX far end" },
	};
	struct ident_line l;
	unsigned i;

	CASE("lec runs the arithmetic of the fixed bar on a far end that "
	     "pauses to silence");
	for (i = 0; i < sizeof(runs) / sizeof(runs[0]); i++) {
		ident_run(runs[i].far, runs[i].rate, &l);
		ident_print(runs[i].what, runs[i].rate, &l);
		CHECK_UINT(l.differ, 0);
		CHECK_UINT(l.states, 0);
		CHECK_UINT(l.raised, 0);
	}
	ident_run(IDENT_FAR_FAN, UDSP_SAMPLE_RATE, &l);
	ident_print("fan", UDSP_SAMPLE_RATE, &l);
	CHECK(l.differ > 0u);
	CHECK(l.raised > 0u);
}

#define BAR_WORDS_FRAMES	(5u * FRAMES_PER_SECOND)
#define BAR_SILENT_FRAMES	100u
#define BAR_STEP_FRAMES		500u
#define BAR_LOW_DBM0		-52.0
#define BAR_HIGH_DBM0		-46.0
#define BAR_WHITE_DBM0		-20.0
/* The history of the far-end floor, 300 to 349 frames. */
#define BAR_HELD_FRAMES		300u
#define BAR_SETTLED_FRAMES	350u
#define BAR_TOL_DB		3.0

/* What the bar of a frame of far_floor_bar_test must read: any bar, the
 * fixed bar, the level of a white fan at dbm0 times the margin within 3 dB,
 * or the cap of the frame. */
enum bar_want {
	BAR_WANT_ANY,
	BAR_WANT_FIXED,
	BAR_WANT_LEVEL,
	BAR_WANT_CAP,
};

/* One part of a step: the bar it wants, the frames, the frames that miss the
 * bar and those that read a background, and the largest distance from the
 * level in dB. */
struct bar_part {
	enum bar_want want;
	double dbm0;
	unsigned frames;
	unsigned miss;
	unsigned background;
	double worst_db;
};

static void bar_part_check(struct bar_part *p,
			   const struct udsp_ec_nlms_frame *reading)
{
	double rms = dbm0_rms_get(p->dbm0);
	double db;

	p->frames++;
	p->background += reading->background;
	if (p->want == BAR_WANT_ANY)
		return;
	if (p->want == BAR_WANT_FIXED) {
		p->miss += reading->ref_bar != REF_BAR_FIXED;
		return;
	}
	if (p->want == BAR_WANT_CAP) {
		p->miss += reading->ref_bar != reading->ref_bar_max;
		return;
	}
	db = 10.0 * log10((double)reading->ref_bar /
			  (rms * rms * REF_FLOOR_MARGIN));
	if (fabs(db) > fabs(p->worst_db))
		p->worst_db = db;
	p->miss += fabs(db) > BAR_TOL_DB;
}

/* One step of the fan. held takes frames 1 to 300 of the step, and settled
 * the frames from 350 on. */
static void bar_step_run(struct fan_line *line, double fan_dbm0,
			 struct bar_part *held, struct bar_part *settled,
			 struct cap_stat *cap)
{
	struct udsp_ec_nlms_frame reading;
	unsigned k;

	line->fan_dbm0 = fan_dbm0;
	for (k = 1; k <= BAR_STEP_FRAMES; k++) {
		(void)fan_cap_frame_run(line, true, false, NULL, cap);
		udsp_ec_nlms_frame_get(line->ec, &reading);
		if (k <= BAR_HELD_FRAMES)
			bar_part_check(held, &reading);
		if (k >= BAR_SETTLED_FRAMES)
			bar_part_check(settled, &reading);
	}
}

/*
 * A flat 30 dB hybrid after far words, then a fan stepping through silence, -52
 * and -46 dBm0, loud white noise and silence. The bar holds the fixed bar for
 * 300 frames, then the level times the margin within 3 dB. It keeps a level for
 * 300 frames after a step and reads the cap on the loud noise. Only the fan
 * over the fixed bar reads a background.
 */
static void far_floor_bar_test(void)
{
	struct bar_part low_held = { .want = BAR_WANT_FIXED };
	struct bar_part low = { .want = BAR_WANT_LEVEL, .dbm0 = BAR_LOW_DBM0 };
	struct bar_part high_held = {
		.want = BAR_WANT_LEVEL, .dbm0 = BAR_LOW_DBM0,
	};
	struct bar_part high = {
		.want = BAR_WANT_LEVEL, .dbm0 = BAR_HIGH_DBM0,
	};
	struct bar_part white_held = { .want = BAR_WANT_ANY };
	struct bar_part white = { .want = BAR_WANT_CAP };
	struct udsp_ec_nlms_frame first;
	struct udsp_ec_nlms_frame second;
	struct cap_stat cap;
	struct fan_line line;
	unsigned f;

	memset(&cap, 0, sizeof(cap));
	fan_line_init(&line, UDSP_SAMPLE_RATE, 0.0);
	fan_line_set(&line, BAR_LOW_DBM0, FLOOR_ERL_DB);
	for (f = 0; f < BAR_WORDS_FRAMES; f++)
		(void)fan_cap_frame_run(&line, false, true, NULL, &cap);
	for (f = 0; f < BAR_SILENT_FRAMES; f++)
		(void)fan_cap_frame_run(&line, false, false, NULL, &cap);
	bar_step_run(&line, BAR_LOW_DBM0, &low_held, &low, &cap);
	bar_step_run(&line, BAR_HIGH_DBM0, &high_held, &high, &cap);
	bar_step_run(&line, BAR_WHITE_DBM0, &white_held, &white, &cap);
	(void)fan_cap_frame_run(&line, false, false, NULL, &cap);
	udsp_ec_nlms_frame_get(line.ec, &first);
	(void)fan_cap_frame_run(&line, false, false, NULL, &cap);
	udsp_ec_nlms_frame_get(line.ec, &second);
	udsp_ec_nlms.destroy(line.ec);

	CASE("lec raises the activity bar over the stationary level of the "
	     "reference");
	printf("# -52 dBm0 after silence: %u frames of the first 300 off the "
	       "fixed bar, %u from frame 350 off the level times the margin, "
	       "largest %.2f dB; -46 dBm0: %u and %u, largest %.2f and %.2f "
	       "dB; white: %u frames from 350 off the cap; silence: bar %.0f "
	       "then %.0f\n", low_held.miss, low.miss, low.worst_db,
	       high_held.miss, high.miss, high_held.worst_db, high.worst_db,
	       white.miss, (double)first.ref_bar, (double)second.ref_bar);
	printf("# background from frame 350: -52 dBm0 %u of %u, -46 dBm0 %u of "
	       "%u, white %u of %u and %u of the first 300; silence %u then "
	       "%u\n", low.background, low.frames, high.background,
	       high.frames, white.background, white.frames,
	       white_held.background, first.background, second.background);
	cap_print("bar line", &cap);
	CHECK_UINT(low_held.miss, 0);
	CHECK_UINT(low.miss, 0);
	CHECK_UINT(high_held.miss, 0);
	CHECK_UINT(high.miss, 0);
	CHECK_UINT(white.miss, 0);
	CHECK(first.ref_bar == first.ref_bar_max);
	CHECK(first.ref_bar > REF_BAR_FIXED);
	CHECK(second.ref_bar == REF_BAR_FIXED);
	CHECK_UINT(low.background, 0);
	CHECK(high.frames > 0u && high.background == high.frames);
	CHECK_UINT(white.background, 0);
	CHECK_UINT(white_held.background, 0);
	CHECK(!first.background && !second.background);
	cap_bounds_check(&cap);
}

#define CAP_FAN_DBM0		-48.0
#define CAP_ERL_DB		6.0
#define CAP_FAN_FRAMES		(10u * FRAMES_PER_SECOND)
#define CAP_TALK_FRAMES		(20u * FRAMES_PER_SECOND)

/* The fan at -48 dBm0 on the 6 dB hybrid, with the far words on it. */
static void cap_fan_run(struct cap_stat *st)
{
	struct udsp_ec_nlms_info info;
	struct fan_line line;
	unsigned f;

	memset(st, 0, sizeof(*st));
	fan_line_init(&line, UDSP_SAMPLE_RATE, 0.0);
	fan_line_set(&line, CAP_FAN_DBM0, CAP_ERL_DB);
	for (f = 0; f < CAP_FAN_FRAMES; f++) {
		udsp_ec_nlms_info_get(line.ec, &info);
		(void)fan_frame_run(&line, true, true, NULL);
		cap_check(st, line.ec, info.echo_gain);
	}
	udsp_ec_nlms.destroy(line.ec);
}

/* The fixture of stale_quiet_test for 20 s: the stale run starts trials
 * that a word talker 2 dB under the echo refuses. */
static void cap_trial_run(struct cap_stat *st)
{
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct ring_line line;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	unsigned f;

	memset(st, 0, sizeof(*st));
	ring_line_init(&line, UDSP_SAMPLE_RATE);
	word_source_init(&talker, UDSP_SAMPLE_RATE,
			 TALKER_ECHO_DBM0 - QUIET_UNDER_DB, 0x31415926u);
	for (f = 0; f < TALKER_CONVERGE_FRAMES + CAP_TALK_FRAMES; f++) {
		bool talk = f >= TALKER_CONVERGE_FRAMES;

		if (talk) {
			word_fill(&line.far, ref, line.frame);
			word_fill(&talker, near, line.frame);
		} else {
			noise_fill(ref, line.frame, RING_WORDS_DBM0,
				   &line.far_state);
		}
		udsp_ec_nlms_info_get(line.ec, &info);
		ring_frame_run(&line, ref, talk ? near : NULL, NULL);
		cap_check(st, line.ec, info.echo_gain);
	}
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * The cap of the activity bar follows the learned gain of the frame: a fan on a
 * 6 dB hybrid keeps the fixed bar. On that line and the line of
 * stale_quiet_test the bar stands at or under the cap. The cap is the larger of
 * 2500 and 82.4 over the bound of the gain read after the frame before.
 */
static void far_floor_cap_test(void)
{
	struct cap_stat fan;
	struct cap_stat trial;

	cap_fan_run(&fan);
	cap_trial_run(&trial);

	CASE("lec caps the activity bar by the learned gain of the frame");
	cap_print("fan on the 6 dB hybrid", &fan);
	cap_print("stale-estimate trials", &trial);
	CHECK_UINT(fan.raised, 0);
	CHECK_UINT(fan.over, 0);
	CHECK_UINT(fan.wrong, 0);
	CHECK_UINT(trial.over, 0);
	CHECK_UINT(trial.wrong, 0);
	CHECK(trial.raised > 0u);
	CHECK(trial.rises > 0u);
	CHECK(trial.restores > 0u);
}

#define FEED_FAN_DBM0		-45.0
#define FEED_TRAIN_FRAMES	(3u * FRAMES_PER_SECOND)
#define FEED_FAN_FRAMES		400u

/*
 * The far-end floor takes the frames of the reference feed and survives the
 * reset. Twins take a fan through run() and through udsp_ec_nlms_ref_feed,
 * their floors match over 396.8, and the reset keeps the floor. No bar is
 * compared, because the learned gain moves on run() frames only.
 */
static void far_floor_feed_test(void)
{
	struct udsp_ec_nlms_info lead_info;
	struct udsp_ec_nlms_info twin_info;
	struct udsp_ec_nlms_info reset_info;
	struct word_source far;
	struct echo_path path;
	struct udsp_ec *lead = udsp_ec_nlms.create(0);
	struct udsp_ec *twin = udsp_ec_nlms.create(0);
	uint32_t state = 0x0f00f00du;
	int16_t ref[UDSP_FRAME_SAMPLES];
	int16_t lead_out[UDSP_FRAME_SAMPLES];
	int16_t twin_out[UDSP_FRAME_SAMPLES];
	struct cap_stat cap;
	unsigned f;

	memset(&cap, 0, sizeof(cap));
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, FAN_ERL_DB);
	word_source_init(&far, UDSP_SAMPLE_RATE, FAN_WORDS_DBM0, 0x13572468u);
	for (f = 0; f < FEED_TRAIN_FRAMES; f++) {
		word_fill(&far, ref, UDSP_FRAME_SAMPLES);
		udsp_ec_nlms_info_get(lead, &lead_info);
		twin_step(lead, twin, &path, ref, NULL, lead_out, twin_out,
			  NULL);
		cap_check(&cap, lead, lead_info.echo_gain);
	}
	for (f = 0; f < FEED_FAN_FRAMES; f++) {
		noise_fill(ref, UDSP_FRAME_SAMPLES, FEED_FAN_DBM0, &state);
		path_run(&path, ref, lead_out);
		udsp_ec_nlms_info_get(lead, &lead_info);
		udsp_ec_nlms.run(lead, lead_out, ref);
		cap_check(&cap, lead, lead_info.echo_gain);
		udsp_ec_nlms_ref_feed(twin, ref);
	}
	udsp_ec_nlms_info_get(lead, &lead_info);
	udsp_ec_nlms_info_get(twin, &twin_info);
	udsp_ec_nlms_reset(lead);
	udsp_ec_nlms_info_get(lead, &reset_info);
	udsp_ec_nlms.destroy(lead);
	udsp_ec_nlms.destroy(twin);

	CASE("lec feeds the far-end floor from the reference feed and keeps it "
	     "through a reset");
	printf("# far-end floor %.1f after run(), %.1f after the feed, %.1f "
	       "after the reset\n", (double)lead_info.ref_floor,
	       (double)twin_info.ref_floor, (double)reset_info.ref_floor);
	cap_print("feed line, first twin", &cap);
	CHECK(lead_info.ref_floor == twin_info.ref_floor);
	CHECK(lead_info.ref_floor > REF_FLOOR_IDENTITY);
	CHECK(reset_info.ref_floor == lead_info.ref_floor);
	cap_bounds_check(&cap);
}

#define FLOOR_TRIAL_LOUD_DBM0	-35.0
#define FLOOR_TRIAL_QUIET_DBM0	-45.0
#define FLOOR_TRIAL_STEP_DB	10.0
#define FLOOR_TRIAL_TRAIN	(5u * FRAMES_PER_SECOND)
#define FLOOR_TRIAL_DROP	20u
#define FLOOR_TRIAL_TALKER	40u
/* A trial frame after the hangover of the trial ran out. The learned gain
 * rises on the clear frames before it, so the talker end restores another
 * gain. */
#define FLOOR_TRIAL_LATE_TALKER	80u
#define FLOOR_TRIAL_LIMIT	200u

/*
 * The trial under a raised bar. talker_at 0 runs no talker. start holds the
 * frame of the step at which the trial starts, end the trial frame that ends
 * it, and miss the trial frames before the talker whose bar is not the fixed
 * bar. reading holds the reading of the frame that ended the trial, and cap
 * the cap check of every frame of the line.
 */
struct floor_trial {
	unsigned start;
	unsigned end;
	unsigned miss;
	struct udsp_ec_nlms_frame reading;
	struct cap_stat cap;
};

/*
 * The fan line at 8 kHz, a 30 dB hybrid and the fan at -35 dBm0 with far
 * words for 5 s, so the bar reads the cap. The hybrid then steps to 10 dB,
 * and the latch starts a trial. The fan drops to -45 dBm0 on trial frame 20.
 * A talker 16 dB over the fan joins on trial frame talker_at.
 */
static void floor_trial_run(unsigned talker_at, struct floor_trial *t)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_frame reading;
	struct fan_line line;
	uint32_t near_state = 0xfeedfaceu;
	int16_t near[UDSP_FRAME_MAX];
	unsigned k;
	unsigned f;

	memset(t, 0, sizeof(*t));
	fan_line_init(&line, UDSP_SAMPLE_RATE, 0.0);
	fan_line_set(&line, FLOOR_TRIAL_LOUD_DBM0, FLOOR_ERL_DB);
	for (f = 0; f < FLOOR_TRIAL_TRAIN; f++)
		(void)fan_cap_frame_run(&line, true, true, NULL, &t->cap);
	path_erl_set(&line.path, FLOOR_TRIAL_STEP_DB);
	for (f = 0; f < SNAP_LIMIT_FRAMES && !t->start; f++) {
		info = fan_cap_frame_run(&line, true, false, NULL, &t->cap);
		if (info.probes)
			t->start = f + 1u;
	}
	for (k = 1; t->start && k <= FLOOR_TRIAL_LIMIT && !t->end; k++) {
		bool talk = k == talker_at;

		if (k == FLOOR_TRIAL_DROP)
			line.fan_dbm0 = FLOOR_TRIAL_QUIET_DBM0;
		if (talk)
			noise_fill(near, line.frame,
				   FLOOR_TRIAL_QUIET_DBM0 + TALKER_OVER_DB,
				   &near_state);
		(void)fan_cap_frame_run(&line, true, false, talk ? near : NULL,
					&t->cap);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		if (!talk && reading.probing)
			t->miss += reading.ref_bar != REF_BAR_FIXED;
		if (reading.trial_end == UDSP_EC_NLMS_TRIAL_END_NONE)
			continue;
		t->end = k;
		t->reading = reading;
	}
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A hybrid step under a fan over a raised bar starts a latch trial, and the fan
 * drops on trial frame 20. Every trial frame reads the fixed bar, and the trial
 * ends within 100 frames. A talker that ends the trial reads the raised bar on
 * its frame, and the cap there reads the gain before the frame.
 */
static void far_floor_trial_test(void)
{
	struct floor_trial quiet;
	struct floor_trial talk;
	struct floor_trial late;

	floor_trial_run(0, &quiet);
	floor_trial_run(FLOOR_TRIAL_TALKER, &talk);
	floor_trial_run(FLOOR_TRIAL_LATE_TALKER, &late);

	CASE("lec reads the fixed bar during a trial under a raised bar");
	printf("# trial from frame %u after the step, ended on trial frame %u "
	       "(%s), %u trial frames off the fixed bar; with the talker: "
	       "ended on trial frame %u (%s), talker %u, bar %.0f, %u frames "
	       "before it off the fixed bar\n", quiet.start, quiet.end,
	       lecr_trial_end_name(quiet.reading.trial_end), quiet.miss,
	       talk.end, lecr_trial_end_name(talk.reading.trial_end),
	       talk.reading.talker, (double)talk.reading.ref_bar, talk.miss);
	CHECK(quiet.start > 0u);
	CHECK(quiet.end > 0u && quiet.end <= TRIAL_BUDGET);
	CHECK_UINT(quiet.miss, 0);
	CHECK_UINT(talk.end, FLOOR_TRIAL_TALKER);
	CHECK_UINT(talk.reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK(talk.reading.talker);
	CHECK(talk.reading.probing);
	CHECK(talk.reading.ref_bar > REF_BAR_FIXED);
	CHECK_UINT(talk.miss, 0);
	cap_print("trial line", &quiet.cap);
	cap_print("trial line with the talker", &talk.cap);
	printf("# with the talker on trial frame %u: ended on trial frame %u "
	       "(%s)\n", FLOOR_TRIAL_LATE_TALKER, late.end,
	       lecr_trial_end_name(late.reading.trial_end));
	cap_print("trial line with the late talker", &late.cap);
	cap_bounds_check(&quiet.cap);
	cap_bounds_check(&talk.cap);
	CHECK_UINT(late.end, FLOOR_TRIAL_LATE_TALKER);
	CHECK_UINT(late.reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK_UINT(late.cap.restores, 1);
	cap_bounds_check(&late.cap);
}

#define LOUDER_STEP_DB		18.0
#define LOUDER_TRAIN_FRAMES	(5u * FRAMES_PER_SECOND)
#define LOUDER_LIMIT_FRAMES	(12u * FRAMES_PER_SECOND)
#define LOUDER_MEASURE_FRAMES	(5u * FRAMES_PER_SECOND)
#define LOUDER_ERLE_TOL_DB	3.0
/* A pause frame is one whose window holds no far word: the window of 7
 * frames at 16 kHz and one frame more. */
#define LOUDER_PAUSE_FRAMES	8u

/*
 * The louder path of far_floor_latch_test: the frames that start and end the
 * first trial, its arm and end, and whether the start frame is active. Then the
 * pause and background frames before it, and the cap check of every frame.
 * Last, the filter ERLE of the far words before the step and after the trial.
 */
struct louder_stat {
	unsigned start;
	unsigned end;
	unsigned arm;
	unsigned end_kind;
	bool start_active;
	unsigned pause;
	unsigned background;
	double before_db;
	double after_db;
	struct cap_stat cap;
};

/* One frame of the louder path, and the pause count of the frames before the
 * trial. pw->index is the frame count of the line after the frame. */
static void louder_frame_run(struct fan_line *line, struct louder_stat *st,
			     struct phrase_words *pw)
{
	struct udsp_ec_nlms_frame reading;

	(void)phrase_frame_run(line, pw, NULL, &st->cap);
	udsp_ec_nlms_frame_get(line->ec, &reading);
	if (st->start || pw->index < BAR_SETTLED_FRAMES ||
	    pw->silent < LOUDER_PAUSE_FRAMES)
		return;
	st->pause++;
	st->background += reading.background;
}

/* The ERLE of the far-word frames over the next frames of the line. */
static double louder_erle_run(struct fan_line *line, struct louder_stat *st,
			      struct phrase_words *pw)
{
	unsigned f;

	line->echo_energy = 0.0;
	line->filter_energy = 0.0;
	line->measure = true;
	for (f = 0; f < LOUDER_MEASURE_FRAMES; f++)
		louder_frame_run(line, st, pw);
	line->measure = false;
	return fan_filter_erle_get(line);
}

/*
 * The fan line with a flat 30 dB hybrid, a far-end background at -45 dBm0
 * from the first frame and far words at -20 dBm0 over it, in the phrases of
 * phrase_frame_run. After 5 s the hybrid steps to 18 dB. The run looks for
 * the first trial within limit frames of the step.
 */
static void louder_run(unsigned rate, unsigned limit, struct louder_stat *st)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct phrase_words pw = { .index = 0 };
	struct fan_line line;
	unsigned f;

	memset(st, 0, sizeof(*st));
	fan_line_init(&line, rate, 0.0);
	fan_line_set(&line, FLOOR_FAN_DBM0, FLOOR_ERL_DB);
	line.measure_words = true;
	line.measure = true;
	for (f = 0; f < LOUDER_TRAIN_FRAMES; f++)
		louder_frame_run(&line, st, &pw);
	line.measure = false;
	st->before_db = fan_filter_erle_get(&line);
	path_erl_set(&line.path, LOUDER_STEP_DB);
	for (f = 1; f <= limit && !st->end; f++) {
		louder_frame_run(&line, st, &pw);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		udsp_ec_nlms_info_get(line.ec, &info);
		if (!st->start && info.probes) {
			st->start = f;
			st->arm = reading.trial_arm;
			st->start_active = reading.ref_power >= reading.ref_bar;
		}
		if (!st->start ||
		    reading.trial_end == UDSP_EC_NLMS_TRIAL_END_NONE)
			continue;
		st->end = f;
		st->end_kind = reading.trial_end;
	}
	if (st->end)
		st->after_db = louder_erle_run(&line, st, &pw);
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * The hybrid steps from 30 to 18 dB under a background and phrases of far
 * words. A latch trial starts within 12 s on an active frame and stands, and
 * the pause frames before it read a background. The filter ERLE after it
 * reaches that of the 5 s before the step, less 3 dB.
 */
static void far_floor_latch_test(unsigned rate)
{
	struct louder_stat st;

	louder_run(rate, LOUDER_LIMIT_FRAMES, &st);

	case_rate(rate, "lec reaches the latch trial of a louder path under a "
		  "far-end background");
	printf("# step from %.0f to %.0f dB: trial %s from frame %u after the "
	       "step, active %u, ended %s on frame %u; %u of %u pause frames "
	       "before it read a background; filter ERLE %.2f dB before the "
	       "step, %.2f dB after the trial\n", FLOOR_ERL_DB, LOUDER_STEP_DB,
	       lecr_trial_arm_name(st.arm), st.start, st.start_active,
	       lecr_trial_end_name(st.end_kind), st.end, st.background,
	       st.pause, st.before_db, st.after_db);
	cap_print("louder path", &st.cap);
	CHECK(st.start > 0u && st.start <= LOUDER_LIMIT_FRAMES);
	CHECK_UINT(st.arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK(st.start_active);
	CHECK_UINT(st.end_kind, UDSP_EC_NLMS_TRIAL_END_KEPT);
	CHECK(st.after_db >= st.before_db - LOUDER_ERLE_TOL_DB);
	CHECK(st.pause > 0u);
	CHECK_UINT(st.background, st.pause);
	cap_bounds_check(&st.cap);
}

#define WORDS_ERL_DB		27.0
#define WORDS_FAN_DBM0		-45.0
/* A far word every 45 frames, each the first word of the source, WORDS_LENGTH
 * frames long. The window holds its last frame until frame 21 of the cycle at 8
 * kHz and 23 at 16 kHz (words_pause_from_get), then the fan alone. */
#define WORDS_CYCLE		45u
#define WORDS_LENGTH		18u
#define WORDS_TRAIN_FRAMES	(5u * FRAMES_PER_SECOND)
#define WORDS_FRAMES		(25u * FRAMES_PER_SECOND)
#define WORDS_COUNT		((WORDS_FRAMES + WORDS_CYCLE - 1u) / \
				 WORDS_CYCLE)
/* The first word after the training, and the word of the near-end legs. */
#define WORDS_FIRST		((WORDS_TRAIN_FRAMES + WORDS_CYCLE - 1u) / \
				 WORDS_CYCLE)
#define WORDS_MARK		(WORDS_FIRST + 4u)
#define WORDS_ACT_MAX		2u
/* A run that starts on the first frame of a word engages on its 8th frame
 * (LEC_NLP_ON_FRAMES of dsp/lec.c). */
#define WORDS_ENGAGE_FRAME	7u
#define WORDS_BURST_DBM0	-30.0
#define WORDS_BURST_FRAMES	10u
#define WORDS_BURST_LEAD	5u
#define WORDS_UNDER_DB		6.0
#define WORDS_NEAR_FRAMES	6u
/* The near word through the processor on a build that zeroes the engage run on
 * every quiet frame. It reads on the first far frame, and from the second frame
 * it is the figure of the filter alone. */
#define WORDS_NEAR_ZEROED_DB	-0.0226
#define WORDS_LATE_ZEROED_DB	-0.53

/*
 * One run of far_floor_words_test or of the fan-off cases: the line, the far
 * words, a near burst before word burst_word and a near word on near_word, from
 * frame near_from. latency holds the first acting frame of each word, and the
 * counts cover the frames and the near word on each line. alt_over_db and
 * alt_pattern set the levels of the near frames before the far word.
 */
struct words_run {
	unsigned rate;
	bool fan;
	double far_dbm0;
	unsigned burst_word;
	unsigned near_word;
	int near_from;
	unsigned near_frames;
	double near_dbm0;
	unsigned latency[WORDS_COUNT];
	unsigned quiet;
	unsigned quiet_acted;
	unsigned acted;
	unsigned lead;
	unsigned lead_background;
	unsigned loud;
	unsigned loud_idle;
	unsigned pause;
	unsigned pause_background;
	unsigned far_frames;
	unsigned far_acted;
	unsigned near_acted;
	unsigned near_later;
	unsigned near_pause;
	double near_pause_lo;
	double near_pause_hi;
	double far_out;
	double near;
	double near_on;
	double near_off;
	double erle;
	double overlap;
	double overlap_on;
	double overlap_off;
	double alt_over_db;
	double alt_under_db;
	const char *alt_pattern;
};

/*
 * The lines of a run: the fan line with the processor off, and a twin fan
 * line with it on. Both take the same frames, because a fan line builds its
 * planes from its own seeded sources. fan_frame_run measures the filter ERLE
 * on the output, so only the line with the processor off gives it.
 * pause_from is the first pause frame of a cycle (words_pause_from_get).
 */
struct words_lines {
	struct fan_line off;
	struct fan_line on;
	struct word_source talker;
	uint32_t burst_state;
	unsigned pause_from;
	bool fan;
	bool waiting;
};

/* The fan line of far_floor_words_test at the line rate, with the processor
 * off. */
static void words_line_init(struct fan_line *line, unsigned rate)
{
	fan_line_init(line, rate, 0.0);
	fan_line_set(line, WORDS_FAN_DBM0, WORDS_ERL_DB);
}

/* One frame of the far words of far_floor_words_test on one line, with the
 * fan when fan is set: a word starts on every frame f that is a multiple of
 * WORDS_CYCLE. */
static struct udsp_ec_nlms_info words_frame_run(struct fan_line *line,
						bool fan, unsigned f,
						const int16_t *near)
{
	if (f % WORDS_CYCLE == 0u)
		word_source_restart(&line->far);
	return fan_frame_run(line, fan, true, near);
}

/* The first frame of a cycle after its window stops holding the far word. */
static unsigned words_pause_from_get(const struct udsp_ec *ec)
{
	return WORDS_LENGTH + window_frames_get(ec) - 1u;
}

static void words_lines_init(struct words_lines *l, const struct words_run *r)
{
	words_line_init(&l->off, r->rate);
	words_line_init(&l->on, r->rate);
	udsp_ec_nlms.nlp(l->on.ec, true);
	l->off.measure_words = true;
	word_source_init(&l->talker, r->rate, r->near_dbm0, 0x0badcafeu);
	l->burst_state = 0xfeedfaceu;
	l->pause_from = words_pause_from_get(l->on.ec);
	l->fan = r->fan;
	l->waiting = false;
}

/* The frame of the near word that frame f carries, counted from 0, or -1
 * when f carries none. The near word holds near_frames frames from frame
 * near_from of word near_word. */
static long words_near_index_get(const struct words_run *r, unsigned f)
{
	long index = (long)f - (long)r->near_word * WORDS_CYCLE -
		     r->near_from;

	if (!r->near_word || index < 0 || index >= (long)r->near_frames)
		return -1;
	return index;
}

/* The near plane of frame f, or NULL. The near word is a word source at
 * near_dbm0. On far_floor_words_test it stands 6 dB under the echo of the far
 * word, so the two follow one envelope. */
static const int16_t *words_near_get(struct words_lines *l,
				     const struct words_run *r, unsigned f,
				     int16_t *near)
{
	unsigned p = f % WORDS_CYCLE;
	unsigned n = f / WORDS_CYCLE;

	if (n + 1u == r->burst_word &&
	    p + WORDS_BURST_LEAD + WORDS_BURST_FRAMES >= WORDS_CYCLE &&
	    p + WORDS_BURST_LEAD < WORDS_CYCLE) {
		noise_fill(near, l->on.frame, WORDS_BURST_DBM0,
			   &l->burst_state);
		return near;
	}
	if (words_near_index_get(r, f) < 0)
		return NULL;
	word_fill(&l->talker, near, l->on.frame);
	return near;
}

/* The line with the processor on: the first frame of each word that reads
 * nlp_active, the quiet frames, the frames before the word of the burst, and
 * the far frames after the training. */
static void words_on_note(struct words_lines *l, struct words_run *r,
			  unsigned f, const struct udsp_ec_nlms_info *info)
{
	struct udsp_ec_nlms_frame reading;
	unsigned p = f % WORDS_CYCLE;
	unsigned n = f / WORDS_CYCLE;
	unsigned i;

	udsp_ec_nlms_frame_get(l->on.ec, &reading);
	r->acted += info->nlp_active;
	if (reading.nlp_verdict == LECR_VERDICT_QUIET) {
		r->quiet++;
		r->quiet_acted += info->nlp_active;
	}
	if (p == 0u) {
		r->latency[n] = WORDS_CYCLE;
		l->waiting = true;
	}
	if (l->waiting && info->nlp_active) {
		r->latency[n] = p;
		l->waiting = false;
	}
	if (n + 1u == r->burst_word &&
	    p + WORDS_BURST_LEAD + WORDS_BURST_FRAMES >= WORDS_CYCLE) {
		r->lead++;
		r->lead_background += reading.background;
	}
	if (f < WORDS_TRAIN_FRAMES || p >= WORDS_LENGTH)
		return;
	r->far_frames++;
	r->far_acted += info->nlp_active;
	for (i = 0; i < l->on.frame; i++)
		r->far_out += (double)l->on.out[i] * l->on.out[i];
}

/* The line with the processor off: the activity of the word frames and the
 * background of the pause frames after the training. */
static void words_off_note(const struct words_lines *l, struct words_run *r,
			   unsigned f)
{
	struct udsp_ec_nlms_frame reading;
	unsigned p = f % WORDS_CYCLE;

	if (f < WORDS_TRAIN_FRAMES)
		return;
	udsp_ec_nlms_frame_get(l->off.ec, &reading);
	if (l->off.word_loud) {
		r->loud++;
		r->loud_idle += reading.ref_power < reading.ref_bar;
	}
	if (p < l->pause_from)
		return;
	r->pause++;
	r->pause_background += reading.background;
}

/* A frame of the near word before the far word, into the range of
 * near_pause_lo and near_pause_hi. */
static void words_pause_note(const struct words_lines *l, struct words_run *r)
{
	struct udsp_ec_nlms_frame reading;
	double db;

	udsp_ec_nlms_frame_get(l->on.ec, &reading);
	db = ratio_db_get(reading.mic_power, reading.noise_power);
	if (!r->near_pause || db < r->near_pause_lo)
		r->near_pause_lo = db;
	if (!r->near_pause || db > r->near_pause_hi)
		r->near_pause_hi = db;
	r->near_pause++;
}

/* The near word against the output of each line: the regression products of
 * near_gain_db_get, and the frames of the word that read nlp_active. index
 * counts the frames of the word from 0. */
static void words_near_note(const struct words_lines *l, struct words_run *r,
			    const int16_t *near, unsigned index, bool acted)
{
	bool overlap = (long)index >= -(long)r->near_from;
	unsigned i;

	for (i = 0; i < l->on.frame; i++) {
		double energy = (double)near[i] * near[i];
		double on = (double)l->on.out[i] * near[i];
		double off = (double)l->off.out[i] * near[i];

		r->near += energy;
		r->near_on += on;
		r->near_off += off;
		if (!overlap)
			continue;
		r->overlap += energy;
		r->overlap_on += on;
		r->overlap_off += off;
	}
	r->near_acted += acted;
	if (index)
		r->near_later += acted;
	if ((long)index < -(long)r->near_from)
		words_pause_note(l, r);
}

/* Scales a frame of the near word before the far word, with index counted
 * from 0. The microphone, the floor plus the word, then stands alt_over_db or
 * alt_under_db over the floor of the previous frame. That floor is the
 * reading of the line with the processor on. */
static void words_alt_scale(const struct words_lines *l,
			    const struct words_run *r, int16_t *near,
			    unsigned index)
{
	struct udsp_ec_nlms_frame reading;
	unsigned lead = (unsigned)-r->near_from;
	double db = (lead - 1u - index) % 2u ? r->alt_under_db :
					       r->alt_over_db;
	double power = udsp_ec_nlms_frame_power_get(near, l->on.frame);
	double gain;
	unsigned i;

	if (r->alt_pattern)
		db = r->alt_pattern[index] == 'o' ? r->alt_over_db :
						    r->alt_under_db;
	udsp_ec_nlms_frame_get(l->on.ec, &reading);
	if (!(reading.noise_power > 0.0f) || !(power > 0.0))
		return;
	gain = sqrt((double)reading.noise_power *
		    (pow(10.0, db / 10.0) - 1.0) / power);
	for (i = 0; i < l->on.frame; i++)
		near[i] = sample_clamp(gain * near[i]);
}

/* The run checks two figures of the line with the processor on: the six
 * counts add up to frames, and nlp_quiet_frames counts every frame that reads
 * quiet. It therefore runs inside the case. */
static void words_run(struct words_run *r)
{
	struct udsp_ec_nlms_info info;
	struct words_lines l;
	int16_t near[UDSP_FRAME_MAX];
	unsigned f;

	words_lines_init(&l, r);
	for (f = 0; f < WORDS_FRAMES; f++) {
		const int16_t *plane = words_near_get(&l, r, f, near);
		long index = words_near_index_get(r, f);

		if (f == WORDS_TRAIN_FRAMES) {
			l.off.far.level_dbm0 = r->far_dbm0;
			l.on.far.level_dbm0 = r->far_dbm0;
			l.off.measure = true;
		}
		if (plane && r->alt_over_db > 0.0 && index >= 0 &&
		    index < -(long)r->near_from)
			words_alt_scale(&l, r, near, (unsigned)index);
		(void)words_frame_run(&l.off, l.fan, f, plane);
		info = words_frame_run(&l.on, l.fan, f, plane);
		words_on_note(&l, r, f, &info);
		words_off_note(&l, r, f);
		if (plane && index >= 0)
			words_near_note(&l, r, plane, (unsigned)index,
					info.nlp_active);
	}
	r->erle = fan_filter_erle_get(&l.off);
	verdict_identity_check(l.on.ec, r->acted);
	udsp_ec_nlms_info_get(l.on.ec, &info);
	CHECK_UINT(info.nlp_quiet_frames, r->quiet);
	udsp_ec_nlms.destroy(l.off.ec);
	udsp_ec_nlms.destroy(l.on.ec);
}

/* A run of far_floor_words_test: 8 kHz, the fan on, and the near word of
 * WORDS_NEAR_FRAMES frames 6 dB under the echo. */
static void words_run_set(struct words_run *r, double far_dbm0)
{
	memset(r, 0, sizeof(*r));
	r->rate = UDSP_SAMPLE_RATE;
	r->fan = true;
	r->far_dbm0 = far_dbm0;
	r->near_frames = WORDS_NEAR_FRAMES;
	r->near_dbm0 = FAN_WORDS_DBM0 - WORDS_ERL_DB - WORDS_UNDER_DB;
}

/*
 * The words after the training whose processor acts more than WORDS_ACT_MAX
 * frames into the word. worst takes the largest latency. The build that
 * zeroes the engage run on every quiet frame engages on frame 7 of each of
 * these words, and acts on 495 of their 805 frames.
 */
static unsigned words_slow_count(const struct words_run *r, unsigned *worst)
{
	unsigned slow = 0;
	unsigned n;

	*worst = 0;
	for (n = WORDS_FIRST; n < WORDS_COUNT; n++) {
		slow += r->latency[n] > WORDS_ACT_MAX;
		if (r->latency[n] > *worst)
			*worst = r->latency[n];
	}
	return slow;
}

static void words_far_print(const char *what, const struct words_run *r)
{
	printf("# %s: %u of %u loud word frames inactive; the processor acts "
	       "on %u of %u far frames after the training, output %.2f dBm0 "
	       "over them\n", what, r->loud_idle, r->loud, r->far_acted,
	       r->far_frames, energy_dbm0_get(r->far_out, r->far_frames *
					      (r->rate / 100u)));
}

/* Softer far words after the training, printed with no bound: the raised bar
 * holds some word frames inactive, and the processor must not act on a quiet
 * frame. */
static void words_levels_run(void)
{
	static const double levels[3] = { -44.0, -38.0, -32.0 };
	struct words_run r;
	char what[48];
	unsigned i;

	for (i = 0; i < 3u; i++) {
		words_run_set(&r, levels[i]);
		words_run(&r);
		snprintf(what, sizeof(what), "far words at %.0f dBm0",
			 levels[i]);
		words_far_print(what, &r);
		CHECK(r.quiet > 0u);
		CHECK_UINT(r.quiet_acted, 0);
	}
}

/*
 * Twin lines with the processor off and on: a fan on a 27 dB hybrid, far words
 * at -20 dBm0. The first holds the words active. On the second each word
 * engages within 2 frames, and no quiet frame acts. A near burst before a word
 * holds it to its 8th frame. A near word 6 dB under the echo loses only
 * its first frame.
 */
static void far_floor_words_test(void)
{
	struct words_run run;
	struct words_run burst;
	struct words_run start;
	struct words_run late;
	unsigned worst;
	unsigned slow;

	CASE("lec keeps the processor ready for far words over a far-end "
	     "background");
	words_run_set(&run, FAN_WORDS_DBM0);
	words_run(&run);
	words_run_set(&burst, FAN_WORDS_DBM0);
	burst.burst_word = WORDS_MARK;
	words_run(&burst);
	words_run_set(&start, FAN_WORDS_DBM0);
	start.near_word = WORDS_MARK;
	words_run(&start);
	words_run_set(&late, FAN_WORDS_DBM0);
	late.near_word = WORDS_MARK;
	late.near_from = 1u;
	words_run(&late);
	slow = words_slow_count(&run, &worst);

	printf("# fan %.0f dBm0 on a %.0f dB hybrid, far words %.0f dBm0, "
	       "processor off: %u of %u pause frames read a background, filter "
	       "ERLE %.2f dB against %.2f dB\n", WORDS_FAN_DBM0, WORDS_ERL_DB,
	       FAN_WORDS_DBM0, run.pause_background, run.pause, run.erle,
	       STEP_HEAD_ERLE_DB);
	printf("# processor on: %u of %u words after the training engage "
	       "later than frame %u, the latest on frame %u; %u of %u quiet "
	       "frames read nlp_active\n", slow, WORDS_COUNT - WORDS_FIRST,
	       WORDS_ACT_MAX, worst, run.quiet_acted, run.quiet);
	words_far_print("far words at -20 dBm0", &run);
	printf("# near burst at %.0f dBm0: %u of %u frames before the word "
	       "read a background, the word engages on its frame %u\n",
	       WORDS_BURST_DBM0, burst.lead_background, burst.lead,
	       burst.latency[WORDS_MARK]);
	printf("# near word %.0f dB under the echo for %u frames, from the "
	       "second frame of the far word: %.2f dB through, filter alone "
	       "%.2f dB (%.2f dB with the run zeroed); the processor acts on "
	       "%u of its frames, %u after the first\n", WORDS_UNDER_DB,
	       WORDS_NEAR_FRAMES, near_gain_db_get(late.near_on, late.near),
	       near_gain_db_get(late.near_off, late.near),
	       WORDS_LATE_ZEROED_DB, late.near_acted, late.near_later);
	printf("# the same word from the first frame of the far word, printed "
	       "with no bound: %.4f dB through (%.4f dB with the run zeroed)\n",
	       near_gain_db_get(start.near_on, start.near),
	       WORDS_NEAR_ZEROED_DB);
	words_levels_run();
	CHECK(run.loud > 0u);
	CHECK_UINT(run.loud_idle, 0);
	CHECK(run.pause > 0u);
	CHECK_UINT(run.pause_background, run.pause);
	CHECK(run.erle >= STEP_HEAD_ERLE_DB - STEP_ERLE_TOL_DB);
	CHECK_UINT(slow, 0);
	CHECK(run.quiet > 0u);
	CHECK_UINT(run.quiet_acted + burst.quiet_acted + start.quiet_acted +
		   late.quiet_acted, 0);
	CHECK_UINT(burst.lead, WORDS_BURST_FRAMES + WORDS_BURST_LEAD);
	CHECK_UINT(burst.lead_background, burst.lead);
	CHECK_UINT(burst.latency[WORDS_MARK], WORDS_ENGAGE_FRAME);
	CHECK(late.near > 0.0);
	CHECK_UINT(late.near_later, 0);
}

#define TOGGLE_TRAIN_FRAMES	900u
#define TOGGLE_FRAMES		300u
#define TOGGLE_SETTLE_FRAMES	30u
/* The window of 5 frames at 8 kHz plus one frame: the shortest period of a
 * release and an engage over a background. */
#define TOGGLE_PERIOD		6u
/* The engage events of this case on a build that zeroes the engage run on
 * every quiet frame. A period holds 5 pass frames, so its run never
 * completes. */
#define TOGGLE_ZEROED_ENGAGES	0u

/*
 * The processor of far_floor_toggle_test: acting frames and those that act
 * without pass over the whole line. After the settle it counts the frames,
 * engage events, acting and gap frames, and quiet gap frames with a background.
 * It also sums the output energy of acting and gap frames.
 */
struct toggle_stat {
	unsigned all_acted;
	unsigned stray;
	unsigned frames;
	unsigned engages;
	unsigned acted;
	unsigned gaps;
	unsigned gap_quiet;
	double act_out;
	double gap_out;
	bool last;
};

static void toggle_note(struct toggle_stat *st, const struct fan_line *line,
			bool active, bool measured)
{
	struct udsp_ec_nlms_frame reading;
	double energy = 0.0;
	bool rose = active && !st->last;
	unsigned i;

	udsp_ec_nlms_frame_get(line->ec, &reading);
	st->all_acted += active;
	st->stray += active && reading.nlp_verdict != LECR_VERDICT_PASS;
	st->last = active;
	if (!measured)
		return;
	for (i = 0; i < line->frame; i++)
		energy += (double)line->out[i] * line->out[i];
	st->frames++;
	st->engages += rose;
	if (active) {
		st->acted++;
		st->act_out += energy;
		return;
	}
	st->gaps++;
	st->gap_quiet += reading.nlp_verdict == LECR_VERDICT_QUIET &&
			 reading.background;
	st->gap_out += energy;
}

/*
 * A loud far frame every 6 frames after training, so the 8 kHz window holds 5
 * active frames and one background frame per period. The background frame
 * releases the processor, and the next pass frame engages it, once per period,
 * on pass frames only.
 */
static void far_floor_toggle_test(void)
{
	struct udsp_ec_nlms_info info;
	struct toggle_stat st;
	struct fan_line line;
	unsigned f;

	CASE("lec engages the processor again at most once per window over a "
	     "far-end background");
	memset(&st, 0, sizeof(st));
	words_line_init(&line, UDSP_SAMPLE_RATE);
	udsp_ec_nlms.nlp(line.ec, true);
	for (f = 0; f < TOGGLE_TRAIN_FRAMES; f++) {
		info = words_frame_run(&line, true, f, NULL);
		toggle_note(&st, &line, info.nlp_active, false);
	}
	for (f = 0; f < TOGGLE_FRAMES; f++) {
		bool loud = f % TOGGLE_PERIOD == 0u;

		if (loud)
			word_source_restart(&line.far);
		info = fan_frame_run(&line, true, loud, NULL);
		toggle_note(&st, &line, info.nlp_active,
			    f >= TOGGLE_SETTLE_FRAMES);
	}
	verdict_identity_check(line.ec, st.all_acted);
	udsp_ec_nlms.destroy(line.ec);

	printf("# a loud far frame every %u frames over the fan: %u engage "
	       "events in %u frames (%u with the run zeroed), %u acting "
	       "frames, %.2f per period; %u of %u gap frames read quiet with a "
	       "background; gap frames %.2f dB against the acting frames\n",
	       TOGGLE_PERIOD, st.engages, st.frames, TOGGLE_ZEROED_ENGAGES,
	       st.acted, (double)st.acted * TOGGLE_PERIOD / st.frames,
	       st.gap_quiet, st.gaps,
	       expose_db_get(st.gap_out * st.acted, st.act_out * st.gaps));
	CHECK(st.engages > 0u);
	CHECK(st.engages <= st.frames / TOGGLE_PERIOD);
	CHECK_UINT(st.stray, 0);
}

/*
 * A run of the cases of a line with the fan off, at the line rate: the far
 * words of far_floor_words_test pause to digital silence, and the near end
 * carries its floor at -72 dBm0 alone. Every pause frame reads quiet with no
 * background. The floor that the pauses measure stands under the
 * echo-likeness bar of a quiet frame, so a pause keeps the engage run.
 */
static void words_quiet_set(struct words_run *r, unsigned rate)
{
	words_run_set(r, FAN_WORDS_DBM0);
	r->rate = rate;
	r->fan = false;
}

/* Far words that pause to silence: each word engages on its first frame, as the
 * pause keeps a complete run. A near burst ending 5 frames before a word zeroes
 * it, so that word engages on its 8th frame. No quiet frame acts. */
static void pause_burst_test(unsigned rate)
{
	struct words_run run;
	unsigned late = 0;
	unsigned n;

	case_rate(rate, "lec nlp acts on the first frame of a far word after a "
		  "pause to silence and waits again after a near burst");
	words_quiet_set(&run, rate);
	run.burst_word = WORDS_MARK;
	words_run(&run);
	for (n = WORDS_FIRST; n < WORDS_COUNT; n++)
		late += n != WORDS_MARK && run.latency[n] != 0u;
	printf("# far words at %.0f dBm0 with pauses to silence: %u of %u "
	       "pause frames read a background; %u of %u words after the "
	       "training and not after the burst engage after their first "
	       "frame; the "
	       "word after a near burst at %.0f dBm0 engages on its frame %u; "
	       "%u of %u quiet frames read nlp_active\n", FAN_WORDS_DBM0,
	       run.pause_background, run.pause, late,
	       WORDS_COUNT - WORDS_FIRST - 1u, WORDS_BURST_DBM0,
	       run.latency[WORDS_MARK], run.quiet_acted, run.quiet);
	words_far_print("far words at -20 dBm0 with pauses to silence", &run);
	CHECK(run.pause > 0u);
	CHECK_UINT(run.pause_background, 0);
	CHECK_UINT(late, 0);
	CHECK_UINT(run.latency[WORDS_MARK], WORDS_ENGAGE_FRAME);
	CHECK(run.quiet > 0u);
	CHECK_UINT(run.quiet_acted, 0);
}

#define PAUSE_LEAD_UNDER_DB	3.0
#define PAUSE_LEAD_FRAMES	4u
/* The mid word: PAUSE_MID_OVER_DB is its source over the floor level. Its onset
 * frame before the far word stands between the bars of the two headrooms at
 * both rates, which the leg prints. */
#define PAUSE_MID_OVER_DB	4.5
#define PAUSE_MID_FRAMES	1u

/* The figure that a rule which keeps the run over a background only reads on
 * a leg of pause_expose_test. */
#define BACKGROUND_ONLY_NOTE	", the figure with the run kept over a " \
				"background only"

/* The late near word of a run against the output of each line. note follows
 * the figure through the processor when through is set, else the figure of
 * the filter alone. */
static void words_late_print(const char *what, const struct words_run *r,
			     bool through)
{
	printf("# %s: near word %.0f dB under the echo for %u frames, from the "
	       "second frame of the far word: %.2f dB through%s; filter alone "
	       "%.2f dB%s; the processor acts on %u of its frames, %u after "
	       "the first\n", what, WORDS_UNDER_DB, r->near_frames,
	       near_gain_db_get(r->near_on, r->near),
	       through ? BACKGROUND_ONLY_NOTE : "",
	       near_gain_db_get(r->near_off, r->near),
	       through ? "" : BACKGROUND_ONLY_NOTE, r->near_acted,
	       r->near_later);
}

/* A run of a near word whose source stands at over_db against the floor
 * level, on the line of pause_burst_test, from lead frames before the far
 * word. */
static void words_lead_run(struct words_run *r, unsigned rate, double over_db,
			   unsigned lead)
{
	words_quiet_set(r, rate);
	r->near_word = WORDS_MARK;
	r->near_from = -(int)lead;
	r->near_frames = lead + WORDS_NEAR_FRAMES;
	r->near_dbm0 = FAN_FLOOR_DBM0 + over_db;
	words_run(r);
}

static void words_lead_print(const char *what, const struct words_run *r,
			     double over_db)
{
	printf("# %s, pauses to silence: near word source %.1f dB %s the "
	       "floor level for %u frames, %u of them in the pause, whose "
	       "microphone stands %.2f to %.2f dB over the tracked floor: "
	       "%.2f dB through; filter alone %.2f dB%s; the processor acts on "
	       "%u of its frames; the far word engages on its frame %u\n",
	       what, fabs(over_db), over_db < 0.0 ? "under" : "over",
	       r->near_frames, r->near_pause, r->near_pause_lo,
	       r->near_pause_hi, near_gain_db_get(r->near_on, r->near),
	       near_gain_db_get(r->near_off, r->near), BACKGROUND_ONLY_NOTE,
	       r->near_acted, r->latency[WORDS_MARK]);
}

/*
 * Each leg prints a near word through the processor and the filter alone, with
 * no bound. The legs are the late word of far_floor_words_test and the late
 * word on the fan line. Two more legs start a word 4 frames and 1 frame before
 * the far word, with their pause levels.
 */
static void pause_expose_test(unsigned rate)
{
	struct words_run late;
	struct words_run lead;
	struct words_run mid;
	struct words_run fan;

	case_rate(rate, "lec nlp exposure of a near word to a kept engage run");
	words_quiet_set(&late, rate);
	late.near_word = WORDS_MARK;
	late.near_from = 1;
	words_run(&late);
	words_lead_run(&lead, rate, -PAUSE_LEAD_UNDER_DB, PAUSE_LEAD_FRAMES);
	words_lead_run(&mid, rate, PAUSE_MID_OVER_DB, PAUSE_MID_FRAMES);
	words_run_set(&fan, FAN_WORDS_DBM0);
	fan.rate = rate;
	fan.near_word = WORDS_MARK;
	fan.near_from = 1;
	words_run(&fan);
	words_late_print("late, pauses to silence", &late, false);
	words_lead_print("lead", &lead, -PAUSE_LEAD_UNDER_DB);
	words_lead_print("mid", &mid, PAUSE_MID_OVER_DB);
	words_late_print("late, background", &fan, true);
	CHECK(late.near > 0.0);
	CHECK(lead.near > 0.0);
	CHECK(mid.near > 0.0);
	CHECK(fan.near > 0.0);
}

#define STATE_ROOM_DBM0		-60.0
#define STATE_PAUSE_FRAMES	12u
#define STATE_FAR_FRAMES	20u
/* Longer than a frame, so the first frame of a far return carries no echo. */
#define STATE_LATE_MS		20u
/* The 4 tail frames of the window, then more than the
 * LEC_GEIGEL_TRUST_PAUSE_FRAMES quiet frames of dsp/lec.c. */
#define STATE_TRUST_PAUSE	210u

/*
 * The line of the state cases of the engage run: the fixture of
 * nlp_wait_count_test at the line rate on a flat 12 dB hybrid at delay_ms,
 * with the processor engaged and no measured floor. Room noise at
 * STATE_ROOM_DBM0 joins the near end on the frames that ask for it. room and
 * out hold the near plane and the output of the last frame.
 */
struct state_line {
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t far_state;
	uint32_t room_state;
	unsigned acted;
	int16_t room[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
};

static void state_line_init(struct state_line *s, unsigned rate,
			    unsigned delay_ms)
{
	memset(s, 0, sizeof(*s));
	s->far_state = 0x77665544u;
	s->room_state = 0x600dcafeu;
	s->ec = gate_fixture_rate_create(&s->path, rate, delay_ms, 12.0, 150u,
					 &s->far_state, &s->acted);
}

/* One frame: far-end noise at -10 dBm0 when far is set, else digital silence,
 * with the room noise on top when room is set. It returns true when the frame
 * reads nlp_active. */
static bool state_frame_run(struct state_line *s, bool far, bool room)
{
	int16_t ref[UDSP_FRAME_MAX];
	unsigned was = s->acted;

	memset(ref, 0, sizeof(ref));
	memset(s->room, 0, sizeof(s->room));
	if (far)
		noise_fill(ref, s->path.frame, -10.0, &s->far_state);
	if (room)
		noise_fill(s->room, s->path.frame, STATE_ROOM_DBM0,
			   &s->room_state);
	verdict_out_step(s->ec, &s->path, ref, room ? s->room : NULL, s->out,
			 &s->acted);
	return s->acted != was;
}

/* frames frames of far-end digital silence, with the room noise from frame
 * from on when room is set. */
static void state_pause_run(struct state_line *s, bool room, unsigned from,
			    unsigned frames)
{
	unsigned f;

	for (f = 0; f < frames; f++)
		(void)state_frame_run(s, false, room && f >= from);
}

/* A far return, with the room noise when room is set. It returns the first
 * frame of the return that reads nlp_active, counted from 0, or
 * STATE_FAR_FRAMES when none does. */
static unsigned state_far_run(struct state_line *s, bool room)
{
	unsigned f;

	for (f = 0; f < STATE_FAR_FRAMES; f++) {
		if (state_frame_run(s, true, room))
			return f;
	}
	return STATE_FAR_FRAMES;
}

/*
 * A pause of pause frames, then a far return (state_far_run). The frames
 * whose window still holds the far end carry digital silence. With room set,
 * the room noise starts on the first quiet frame and runs through the return.
 */
static unsigned state_return_run(struct state_line *s, bool room,
				 unsigned pause)
{
	state_pause_run(s, room, window_frames_get(s->ec) - 1u, pause);
	return state_far_run(s, room);
}

/*
 * A line with no measured floor gives a quiet frame a floor term of 0. Room
 * noise then stands over the echo-likeness bar and zeroes the run on the first
 * quiet frame. The far return then engages on its 8th frame; digital silence
 * keeps the run (state_silence_test).
 */
static void state_room_test(unsigned rate)
{
	struct state_line s;
	unsigned at;

	state_line_init(&s, rate, 8u);
	at = state_return_run(&s, true, STATE_PAUSE_FRAMES);
	case_rate(rate, "lec nlp zeroes the engage run on a pause with room "
		  "noise and no measured floor");
	printf("# the far return acts from its frame %u\n", at);
	CHECK_UINT(at, WORDS_ENGAGE_FRAME);
	udsp_ec_nlms.destroy(s.ec);
}

/* On a line with no measured floor, the echo-likeness bar of a quiet frame
 * is the estimate times the margin. Digital silence at the near end never
 * stands over that bar, because 0 stands at or under any bar. A pause of
 * digital silence keeps the run, and the far return acts on its first frame. */
static void state_silence_test(unsigned rate)
{
	struct state_line s;
	unsigned at;

	state_line_init(&s, rate, 8u);
	at = state_return_run(&s, false, STATE_PAUSE_FRAMES);
	case_rate(rate, "lec nlp keeps the engage run through a pause of "
		  "digital silence");
	printf("# the far return acts from its frame %u\n", at);
	CHECK_UINT(at, 0);
	udsp_ec_nlms.destroy(s.ec);
}

/* The return frame of state_early_test fades into comfort noise LEC_CN_RAISE_DB
 * over the floor. It stands within EARLY_FLOOR_TOL_DB of the floor, at a gain
 * under EARLY_GAIN_MAX_DB; a passed residual reads 0. */
#define EARLY_FLOOR_TOL_DB	3.0
#define EARLY_GAIN_MAX_DB	-3.0

/*
 * The first pass frame of a far return can come before its echo reaches the
 * microphone, as on this 20 ms hybrid. After a second pause the processor acts
 * on it. Comfort noise replaces the room noise within 3 dB of the floor, not
 * over the microphone, with a gain under -3 dB.
 */
static void state_early_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct state_line s;
	double product = 0.0;
	double room = 0.0;
	double out_dbov;
	double gain_db;
	float out_power;
	unsigned first;
	unsigned at;
	unsigned i;

	state_line_init(&s, rate, STATE_LATE_MS);
	first = state_return_run(&s, true, STATE_PAUSE_FRAMES);
	at = state_return_run(&s, true, STATE_PAUSE_FRAMES);
	udsp_ec_nlms_frame_get(s.ec, &reading);
	udsp_ec_nlms_info_get(s.ec, &info);
	out_power = udsp_ec_nlms_frame_power_get(s.out, s.path.frame);
	out_dbov = floor_dbov_get(energy_dbm0_get(out_power, 1u));
	for (i = 0; i < s.path.frame; i++) {
		product += (double)s.out[i] * s.room[i];
		room += (double)s.room[i] * s.room[i];
	}
	gain_db = near_gain_db_get(product, room);
	case_rate(rate, "lec nlp replaces the first return frame before its "
		  "echo with comfort noise at the floor");
	printf("# the return after the pause that measures the floor acts from "
	       "its frame %u, the next return from its frame %u; that frame: "
	       "microphone %.2f dBm0, estimate %.2f dBm0, output %.2f dBm0, "
	       "floor %.2f dBov, output %.2f dBov, gain on the room noise "
	       "%.2f dB\n", first, at,
	       energy_dbm0_get(reading.mic_power, 1u),
	       energy_dbm0_get(reading.estimate_power, 1u),
	       energy_dbm0_get(out_power, 1u), (double)info.noise_dbov,
	       out_dbov, gain_db);
	CHECK_UINT(at, 0);
	CHECK(fabs(out_dbov - info.noise_dbov) <= EARLY_FLOOR_TOL_DB);
	CHECK(out_power <= reading.mic_power);
	CHECK(gain_db <= EARLY_GAIN_MAX_DB);
	udsp_ec_nlms.destroy(s.ec);
}

/*
 * A far return after LEC_GEIGEL_TRUST_PAUSE_FRAMES quiet frames restarts a
 * trust count that never held (quiet_run_update). After 150 training frames a
 * silent pause keeps the run, the return acts on its first frame, and its
 * reading shows the count started over.
 */
static void state_trust_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct state_line s;
	unsigned before;
	unsigned at;

	state_line_init(&s, rate, 8u);
	udsp_ec_nlms_frame_get(s.ec, &reading);
	before = reading.trust_run;
	at = state_return_run(&s, false, STATE_TRUST_PAUSE);
	udsp_ec_nlms_frame_get(s.ec, &reading);
	case_rate(rate, "lec nlp keeps the engage run through a restart of the "
		  "trust count");
	printf("# trust count %u before the pause, %u after the return frame; "
	       "the return acts from its frame %u\n", before, reading.trust_run,
	       at);
	CHECK_UINT(at, 0);
	CHECK(reading.trust_run < before);
	udsp_ec_nlms.destroy(s.ec);
}

/* A 1000 Hz tone holds whole periods in a frame, so its level against the floor
 * is set to hundredths of a dB. The tone of HEADROOM_KEEP_DB stands between the
 * bars of the two headrooms, HEADROOM_ZERO_DB over both. */
#define STATE_TONE_HZ		1000.0
#define STATE_TONE_AT		4u
/* Three quiet frames in a row over the echo-likeness bar zero the engage
 * run; two keep it (quiet_miss_test). */
#define STATE_TONE_FRAMES	3u
#define HEADROOM_KEEP_DB	5.3
#define HEADROOM_ZERO_DB	7.0

/* One frame of far-end digital silence with the tone at over_db over the
 * floor that the detector reads on the frame, in place of the room noise. */
static void state_tone_run(struct state_line *s, double over_db)
{
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_MAX];
	unsigned long phase = 0;
	double floor;

	udsp_ec_nlms_info_get(s->ec, &info);
	floor = 32767.0 * 32767.0 * pow(10.0, -info.noise_dbov / 10.0);
	memset(ref, 0, sizeof(ref));
	memset(s->room, 0, sizeof(s->room));
	tone_fill(s->room, s->path.frame,
		  energy_dbm0_get(floor * pow(10.0, over_db / 10.0), 1u),
		  STATE_TONE_HZ, &phase, s->path.frame * 100u);
	verdict_out_step(s->ec, &s->path, ref, s->room, s->out, &s->acted);
}

/*
 * A pause with room noise and STATE_TONE_FRAMES tone frames in a row in its
 * middle, then a far return with room noise (state_far_run). reading takes
 * the reading of the last tone frame. The tone frames cannot move the floor:
 * a reading more than LEC_NOISE_RISE_DB over it joins a rise run of
 * LEC_NOISE_RISE_FRAMES.
 */
static unsigned state_tone_return_run(struct state_line *s, double over_db,
				      struct udsp_ec_nlms_frame *reading)
{
	unsigned tail = window_frames_get(s->ec) - 1u;
	unsigned f;

	state_pause_run(s, true, tail, tail + STATE_TONE_AT);
	for (f = 0; f < STATE_TONE_FRAMES; f++)
		state_tone_run(s, over_db);
	udsp_ec_nlms_frame_get(s->ec, reading);
	state_pause_run(s, true, 0u, STATE_TONE_AT);
	return state_far_run(s, true);
}

/*
 * The two floor headrooms of the echo-likeness test: three tone frames in a
 * pause, 5.3 dB over the tracked floor, stand between the bars and keep the
 * run. At 7 dB they stand over both and zero it. The far return resumes the run
 * (LEC_NLP_RESUME_DB), so each return acts on its first frame.
 */
static void pause_headroom_test(unsigned rate)
{
	struct udsp_ec_nlms_frame keep;
	struct udsp_ec_nlms_frame zero;
	struct state_line s;
	unsigned first;
	unsigned kept;
	unsigned zeroed;

	state_line_init(&s, rate, 8u);
	first = state_return_run(&s, true, STATE_PAUSE_FRAMES);
	kept = state_tone_return_run(&s, HEADROOM_KEEP_DB, &keep);
	zeroed = state_tone_return_run(&s, HEADROOM_ZERO_DB, &zero);
	case_rate(rate, "lec nlp keeps the engage run through a floor "
		  "excursion in a pause and resumes it after one over the "
		  "pause headroom");
	printf("# the return after the pause that measures the floor acts from "
	       "its frame %u; a tone %.2f dB over the floor: the next return "
	       "acts from its frame %u; a tone %.2f dB over the floor: the "
	       "next return acts from its frame %u\n", first,
	       ratio_db_get(keep.mic_power, keep.noise_power), kept,
	       ratio_db_get(zero.mic_power, zero.noise_power), zeroed);
	CHECK(keep.noise_power > 0.0f);
	CHECK_UINT(keep.nlp_verdict, LECR_VERDICT_QUIET);
	CHECK_UINT(zero.nlp_verdict, LECR_VERDICT_QUIET);
	CHECK(keep.estimate_power == 0.0f);
	CHECK(zero.estimate_power == 0.0f);
	CHECK_UINT(kept, 0);
	CHECK_UINT(zeroed, 0);
	udsp_ec_nlms.destroy(s.ec);
}

/*
 * The active headroom: on the second of 2 stopped far frames the window stays
 * active with no echo. A tone 5.3 dB over the floor reads unlike and cuts the
 * run, so the return acts on its 4th frame. A test that takes the pause
 * headroom on an active window fails.
 */
static void active_headroom_test(unsigned rate)
{
	struct udsp_ec_nlms_frame gap;
	struct udsp_ec_nlms_frame tone;
	struct state_line s;
	unsigned first;
	unsigned at;

	state_line_init(&s, rate, 8u);
	first = state_return_run(&s, true, STATE_PAUSE_FRAMES);
	(void)state_frame_run(&s, false, true);
	udsp_ec_nlms_frame_get(s.ec, &gap);
	state_tone_run(&s, HEADROOM_KEEP_DB);
	udsp_ec_nlms_frame_get(s.ec, &tone);
	at = state_far_run(&s, true);
	case_rate(rate, "lec nlp cuts the engage run on a tone between the "
		  "bars of an active window");
	printf("# the return after the pause that measures the floor acts from "
	       "its frame %u; first gap frame: window run %u, estimate %.3g, "
	       "floor %.3g; a tone %.2f dB over the floor on the second: "
	       "window run %u, estimate %.3g, verdict %s; the far return acts "
	       "from its frame %u\n", first, gap.window_run,
	       (double)gap.estimate_power, (double)gap.noise_power,
	       ratio_db_get(tone.mic_power, tone.noise_power), tone.window_run,
	       (double)tone.estimate_power,
	       udsp_ec_nlms_verdict_name(tone.nlp_verdict), at);
	CHECK(tone.noise_power > 0.0f);
	CHECK(tone.window_run > 0u);
	CHECK(tone.estimate_power * 100.0f < tone.noise_power);
	CHECK_UINT(tone.nlp_verdict, LECR_VERDICT_UNLIKE);
	CHECK_UINT(at, CUT_WAIT);
	udsp_ec_nlms.destroy(s.ec);
}

/*
 * A ringback line for the lockout cases, frame f counted from the instance's
 * creation as lec_replay counts it. It carries the ringback with optional noise
 * and a cut, far words from frame answer and a far background. A flat or
 * captured hybrid, a near floor, a dense talker and the processor switch
 * complete it.
 */
struct ring_spec {
	unsigned rate;
	unsigned ring_frames;
	unsigned noise_from;
	double noise_db;
	unsigned cut_from;
	unsigned cut_frames;
	unsigned answer;
	unsigned first_word;
	double words_dbm0;
	unsigned background_from;
	double background_dbm0;
	double erl_db;
	const struct field_path *field;
	unsigned floor_from;
	double floor_dbm0;
	unsigned talk_from;
	unsigned talk_to;
	double talk_dbm0;
	bool nlp;
	struct dense_talker talker;
};

static void ring_spec_line_init(struct ring_line *line, struct ring_spec *s)
{
	ring_line_init(line, s->rate);
	udsp_ec_nlms.nlp(line->ec, s->nlp);
	if (s->field)
		path_field_init(&line->path, s->rate, s->field);
	else
		path_flat_init(&line->path, s->rate, 8, s->erl_db);
	line->far.level_dbm0 = s->words_dbm0;
	line->far.word = s->first_word;
	line->floor_dbm0 = s->floor_dbm0;
	dense_talker_init(&s->talker, s->rate, s->talk_dbm0, 0x0badcafeu);
}

static void ring_part_add(int16_t *dst, const int16_t *part, unsigned n)
{
	unsigned i;

	for (i = 0; i < n; i++)
		dst[i] = sample_clamp((double)dst[i] + part[i]);
}

/* The reference of frame f. */
static void ring_spec_fill(struct ring_line *line, const struct ring_spec *s,
			   unsigned f, int16_t *ref)
{
	int16_t part[UDSP_FRAME_MAX];
	bool cut = s->cut_frames && f >= s->cut_from &&
		   f < s->cut_from + s->cut_frames;

	memset(ref, 0, line->frame * sizeof(*ref));
	if (f <= s->ring_frames && !cut)
		ring_tones_fill(line, ref);
	if (s->noise_from && f >= s->noise_from && f <= s->ring_frames) {
		noise_fill(part, line->frame, RING_DBM0 - s->noise_db,
			   &line->far_state);
		ring_part_add(ref, part, line->frame);
	}
	if (f >= s->answer) {
		word_fill(&line->far, part, line->frame);
		ring_part_add(ref, part, line->frame);
	}
	if (s->background_from && f >= s->background_from) {
		noise_fill(part, line->frame, s->background_dbm0,
			   &line->far_state);
		ring_part_add(ref, part, line->frame);
	}
}

/* Frame f of the line, with the talker and the floor of the spec. */
static void ring_spec_frame_run(struct ring_line *line, struct ring_spec *s,
				unsigned f, struct ring_stat *st,
				struct ring_planes *p)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	bool talk = f >= s->talk_from && f < s->talk_to;

	ring_spec_fill(line, s, f, ref);
	if (talk)
		dense_talker_fill(&s->talker, near, line->frame);
	line->floor = f >= s->floor_from;
	ring_planes_run(line, ref, talk ? near : NULL, st, p);
}

/* The three forms of ring_spec_set. */
enum ring_form {
	RING_FORM_INSIDE,
	RING_FORM_REFUSED,
	RING_FORM_BACKGROUND,
};

#define SPEC_ERL_DB		28.0
#define SPEC_TALK_DBM0		-25.0
#define INSIDE_RING_FRAMES	300u
#define INSIDE_NOISE_FROM	101u
#define INSIDE_NOISE_DB		14.0
#define INSIDE_FIELD_NOISE_DB	25.0
#define INSIDE_ANSWER		633u
#define INSIDE_FLOOR_DBM0	-75.0
#define REFUSED_RING_FRAMES	430u
#define REFUSED_CUT_FROM	201u
#define REFUSED_CUT_FRAMES	20u
#define REFUSED_ANSWER		688u
#define REFUSED_WORDS_DBM0	-28.0
#define REFUSED_FLOOR_DBM0	-77.6
#define BACKGROUND_RING_FRAMES	200u
#define BACKGROUND_FROM		241u
#define BACKGROUND_DBM0		-45.0
#define BACKGROUND_ANSWER	291u
#define BACKGROUND_FLOOR_DBM0	-76.5
#define BACKGROUND_TALK_FROM	341u
#define BACKGROUND_TALK_TO	841u

static const struct field_path *field_path_get(const char *name)
{
	unsigned i;

	for (i = 0; i < sizeof(field_paths) / sizeof(field_paths[0]); i++) {
		if (!strcmp(field_paths[i].name, name))
			return &field_paths[i];
	}
	return NULL;
}

/*
 * The forms of the lockout cases, with the levels of the line captures they
 * name. INSIDE has one burst with noise 14 dB under each tone and a floor from
 * frame 301. REFUSED has two bursts that end with the energy rule refused.
 * BACKGROUND has a background after the tones with words and a dense talker.
 */
static void ring_spec_set(struct ring_spec *s, enum ring_form form,
			  unsigned rate)
{
	memset(s, 0, sizeof(*s));
	s->rate = rate;
	s->words_dbm0 = RING_WORDS_DBM0;
	s->erl_db = SPEC_ERL_DB;
	s->talk_dbm0 = SPEC_TALK_DBM0;
	s->floor_from = 1u;
	s->nlp = true;
	switch (form) {
	case RING_FORM_INSIDE:
		s->ring_frames = INSIDE_RING_FRAMES;
		s->noise_from = INSIDE_NOISE_FROM;
		s->noise_db = INSIDE_NOISE_DB;
		s->answer = INSIDE_ANSWER;
		s->floor_from = INSIDE_RING_FRAMES + 1u;
		s->floor_dbm0 = INSIDE_FLOOR_DBM0;
		if (rate == UDSP_SAMPLE_RATE)
			break;
		s->field = field_path_get("an7581 raw");
		s->noise_db = INSIDE_FIELD_NOISE_DB;
		break;
	case RING_FORM_REFUSED:
		s->ring_frames = REFUSED_RING_FRAMES;
		s->cut_from = REFUSED_CUT_FROM;
		s->cut_frames = REFUSED_CUT_FRAMES;
		s->answer = REFUSED_ANSWER;
		s->words_dbm0 = REFUSED_WORDS_DBM0;
		s->floor_dbm0 = REFUSED_FLOOR_DBM0;
		break;
	case RING_FORM_BACKGROUND:
		s->ring_frames = BACKGROUND_RING_FRAMES;
		s->background_from = BACKGROUND_FROM;
		s->background_dbm0 = BACKGROUND_DBM0;
		s->answer = BACKGROUND_ANSWER;
		s->floor_dbm0 = BACKGROUND_FLOOR_DBM0;
		s->talk_from = BACKGROUND_TALK_FROM;
		s->talk_to = BACKGROUND_TALK_TO;
		break;
	}
}

/*
 * The census of lec_replay (lec_replay.c) from frame RING_CENSUS_FROM: a far
 * frame has a window frame at or over RING_CENSUS_FAR_DBFS. A leak frame is a
 * far frame whose output less near and floor stands over RING_CENSUS_LEAK_DBFS.
 */
#define RING_CENSUS_FROM	300u
#define RING_CENSUS_TO		3000u
#define RING_CENSUS_HISTORY	6u
#define RING_CENSUS_FAR_DBFS	-40.0
#define RING_CENSUS_LEAK_DBFS	-65.0
#define RING_CENSUS_NEAR_RATIO	16.0
/* A lockout second: 10 far frames, dt on RING_CENSUS_LOCKOUT_DT frames and a
 * loss under RING_CENSUS_LOCKOUT_DB. */
#define RING_CENSUS_SECOND_FAR	10u
#define RING_CENSUS_LOCKOUT_DT	30u
#define RING_CENSUS_LOCKOUT_DB	8.0
#define RING_CENSUS_SECONDS	(RING_CENSUS_TO / FRAMES_PER_SECOND)

struct ring_census_second {
	unsigned far;
	unsigned dt;
	double mic;
	double out;
	double echo;
	double leak;
};

/* A census selection: the far frames, or the far frames that are not near
 * frames. leak_max is the largest power of the echo leak, the output less the
 * near plane and the floor plane. */
struct ring_census_sel {
	unsigned frames;
	unsigned leak;
	double out_sum;
	double out_max;
	double leak_max;
};

/* ref_plane and out_plane keep the powers of the planes of each frame for
 * ring_census_leak_count. */
struct ring_census {
	float ref_last[RING_CENSUS_HISTORY];
	float window_last;
	struct ring_census_sel far;
	struct ring_census_sel far_not_near;
	unsigned near;
	struct ring_census_second second[RING_CENSUS_SECONDS];
	float ref_plane[RING_CENSUS_TO];
	float out_plane[RING_CENSUS_TO];
};

static double census_power_get(double dbfs)
{
	return 32768.0 * 32768.0 * pow(10.0, dbfs / 10.0);
}

static void ring_census_sel_add(struct ring_census_sel *s, bool leak,
				float out, double leak_power)
{
	s->frames++;
	s->leak += leak;
	s->out_sum += out;
	if (out > s->out_max)
		s->out_max = out;
	if (leak_power > s->leak_max)
		s->leak_max = leak_power;
}

static double ring_leak_power_get(const struct ring_planes *p, unsigned frame)
{
	double leak = 0.0;
	unsigned i;

	for (i = 0; i < frame; i++) {
		double l = (double)p->out[i] - p->near[i] - p->hiss[i];

		leak += l * l;
	}
	return leak / frame;
}

/* The second of far frame f takes the frame. */
static void ring_census_second_add(struct ring_census *c,
				   const struct udsp_ec_nlms_frame *reading,
				   const struct ring_planes *p, unsigned frame,
				   unsigned f, double leak_power)
{
	struct ring_census_second *sec = &c->second[f / FRAMES_PER_SECOND];

	sec->far++;
	sec->mic += reading->mic_power;
	sec->out += udsp_ec_nlms_frame_power_get(p->out, frame);
	sec->echo += udsp_ec_nlms_frame_power_get(p->echo, frame);
	sec->leak += leak_power;
}

/* Frame f of a line into the census, after the frame ran. */
static void ring_census_add(struct ring_census *c, const struct udsp_ec *ec,
			    const struct ring_planes *p, unsigned frame,
			    unsigned f)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	double leak_power;
	double guard;
	double factor;
	float ref;
	float out = udsp_ec_nlms_frame_power_get(p->out, frame);
	bool far;
	bool leak;
	bool near;
	unsigned i;

	udsp_ec_nlms_frame_get(ec, &reading);
	udsp_ec_nlms_info_get(ec, &info);
	guard = reading.window_power > c->window_last ? reading.window_power :
							  c->window_last;
	factor = 2.0 * info.echo_gain;
	if (factor > 0.25)
		factor = 0.25;
	ref = reading.ref_power;
	for (i = 0; i < RING_CENSUS_HISTORY; i++) {
		if (c->ref_last[i] > ref)
			ref = c->ref_last[i];
	}
	far = ref >= census_power_get(RING_CENSUS_FAR_DBFS);
	leak = far && out > census_power_get(RING_CENSUS_LEAK_DBFS);
	near = reading.mic_power >= RING_CENSUS_NEAR_RATIO * guard * factor;
	memmove(c->ref_last + 1, c->ref_last,
		sizeof(c->ref_last) - sizeof(c->ref_last[0]));
	c->ref_last[0] = reading.ref_power;
	c->window_last = reading.window_power;
	if (f < RING_CENSUS_TO) {
		c->ref_plane[f] = udsp_ec_nlms_frame_power_get(p->ref, frame);
		c->out_plane[f] = out;
	}
	if (f < RING_CENSUS_FROM || f >= RING_CENSUS_TO)
		return;
	c->near += near;
	c->second[f / FRAMES_PER_SECOND].dt +=
		reading.nlp_verdict == LECR_VERDICT_DT;
	if (!far)
		return;
	leak_power = ring_leak_power_get(p, frame);
	ring_census_second_add(c, &reading, p, frame, f, leak_power);
	ring_census_sel_add(&c->far, leak, out, leak_power);
	if (!near)
		ring_census_sel_add(&c->far_not_near, leak, out, leak_power);
}

/* The lockout seconds by the loss of the census, or by the echo loss when
 * echo is set. list takes the seconds, comma separated. */
static unsigned ring_census_lockout_get(const struct ring_census *c, bool echo,
					char *list, size_t size)
{
	size_t used = 0;
	unsigned count = 0;
	unsigned k;

	list[0] = '\0';
	for (k = RING_CENSUS_FROM / FRAMES_PER_SECOND;
	     k < RING_CENSUS_SECONDS; k++) {
		const struct ring_census_second *s = &c->second[k];
		double num = echo ? s->echo : s->mic;
		double den = echo ? s->leak : s->out;

		if (s->far < RING_CENSUS_SECOND_FAR ||
		    s->dt < RING_CENSUS_LOCKOUT_DT)
			continue;
		if (!(10.0 * log10(num / den) < RING_CENSUS_LOCKOUT_DB))
			continue;
		count++;
		if (used < size)
			used += (size_t)snprintf(list + used, size - used,
						 "%s%u", used ? "," : "", k);
	}
	if (!count)
		snprintf(list, size, "-");
	return count;
}

/* The leak frames again, from the powers of the reference and output planes
 * that the census kept. */
static unsigned ring_census_leak_count(const struct ring_census *c)
{
	unsigned count = 0;
	unsigned f;

	for (f = RING_CENSUS_FROM; f < RING_CENSUS_TO; f++) {
		float ref = c->ref_plane[f];
		unsigned k;

		for (k = 1; k <= RING_CENSUS_HISTORY; k++) {
			if (c->ref_plane[f - k] > ref)
				ref = c->ref_plane[f - k];
		}
		count += ref >= census_power_get(RING_CENSUS_FAR_DBFS) &&
			 c->out_plane[f] >
			 census_power_get(RING_CENSUS_LEAK_DBFS);
	}
	return count;
}

#define INSIDE_WORDS_FRAMES	(20u * FRAMES_PER_SECOND)
#define INSIDE_GAP_MIN_DB	-10.0
#define INSIDE_GAP_MAX_DB	-7.0
/* LEC_GEIGEL_GAIN_FLOOR of dsp/lec.c. */
#define INSIDE_GAIN_FLOOR	0.001f

/* The figures of ring_inside_test. at holds the state at the end of the
 * ringback, and answer the reading of the first word frame. */
struct inside_run {
	struct udsp_ec_nlms_info at;
	struct udsp_ec_nlms_frame answer;
	struct udsp_ec_nlms_info end;
	struct ring_stat first;
	struct ring_stat last;
	uint32_t first_probes;
	unsigned last_trusted;
	unsigned first_trust;
};

static void inside_run(struct inside_run *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_planes p;
	struct ring_spec s;
	struct ring_line line;
	unsigned end;
	unsigned f;

	memset(r, 0, sizeof(*r));
	ring_spec_set(&s, RING_FORM_INSIDE, UDSP_SAMPLE_RATE);
	s.nlp = false;
	ring_spec_line_init(&line, &s);
	end = s.answer + INSIDE_WORDS_FRAMES;
	for (f = 1; f < end; f++) {
		bool first = f >= s.answer && f < s.answer + RING_FIRST_FRAMES;
		bool last = f + RING_LAST_FRAMES >= end;

		ring_spec_frame_run(&line, &s, f, first ? &r->first :
				    last ? &r->last : NULL, &p);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		udsp_ec_nlms_info_get(line.ec, &info);
		if (f == s.ring_frames)
			r->at = info;
		if (f == s.answer)
			r->answer = reading;
		if (f + 1u == s.answer + RING_FIRST_FRAMES)
			r->first_probes = info.probes;
		if (f >= s.answer && reading.trusted && !r->first_trust)
			r->first_trust = f - s.answer + 1u;
		r->last_trusted += last && reading.trusted;
	}
	r->end = info;
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A tone-trained filter inside the energy rule (INSIDE at 8 kHz, processor
 * off): the ringback reaches 18 dB of ERLE with energy 7 to 10 dB under the
 * gain. The first word meets no trust, and the first 2 s hold 20 percent or
 * less without a trial. The last 8 s regain the trust at a filter ERLE of 15
 * dB.
 */
static void ring_inside_test(void)
{
	struct inside_run r;
	double gap;
	double erle;

	inside_run(&r);
	gap = ring_gap_db_get(&r.at);
	erle = weak_filter_erle_get(r.last.echo, r.last.residual,
				    r.last.floor);

	CASE("lec withholds the trust of a tone-trained filter inside the "
	     "energy rule");
	printf("# after the ringback: tracked ERLE %.2f dB, coefficient energy "
	       "%.2f dB against the learned gain of %.2f dB, floor %.0f dBov; "
	       "first word: trusted %u, trust count %u\n",
	       (double)r.at.erle_db, gap, 10.0 * log10(r.at.echo_gain),
	       (double)r.at.noise_dbov, r.answer.trusted, r.answer.trust_run);
	printf("# first 2 s: double talk on %u of %u active frames, %u trials; "
	       "last 8 s: %u frames trusted, %u of %u active frames held, "
	       "filter ERLE %.2f dB; first trusted frame %u ms after the first "
	       "word; %u trials, %u resets\n", r.first.dt, r.first.active,
	       r.first_probes, r.last_trusted, r.last.held, r.last.active, erle,
	       10u * r.first_trust, r.end.probes, r.end.diverge_resets);
	CHECK(r.at.erle_db >= RING_TRUST_DB);
	CHECK(gap >= INSIDE_GAP_MIN_DB && gap <= INSIDE_GAP_MAX_DB);
	CHECK(r.at.echo_gain == INSIDE_GAIN_FLOOR);
	CHECK(r.at.noise_dbov == 127.0f);
	CHECK(!r.answer.trusted);
	CHECK(r.answer.trust_run < TRUST_FRAMES);
	CHECK(r.first.active > 0u);
	CHECK(r.first.dt * 10u <= r.first.active * 2u);
	CHECK_UINT(r.first_probes, 0);
	CHECK(r.last_trusted > 0u);
	CHECK(r.last.active > 0u);
	CHECK(r.last.held * 10u <= r.last.active * 2u);
	CHECK(erle >= 15.0);
}

#define LOCKOUT_LEAK_MAX	100u
#define LOCKOUT_SECONDS_MAX	1u
#define LOCKOUT_TALK_AFTER	(5u * FRAMES_PER_SECOND)
#define LOCKOUT_TALK_FRAMES	(10u * FRAMES_PER_SECOND)
#define LOCKOUT_SHORT_GAP	100u

/*
 * The census of the legs of ring_lockout_test on a canceller without the trust
 * count, the bars of each leg. For BACKGROUND the figures cover the far frames
 * that are not near frames, and out_max reads the echo leak. A near talker
 * under the near mark makes the output's peak a phase of the speech.
 */
struct lockout_head {
	unsigned rate;
	enum ring_form form;
	double out_sum;
	double out_max;
	double leak_max;
};

static const struct lockout_head lockout_heads[] = {
	{ UDSP_SAMPLE_RATE, RING_FORM_INSIDE, 535924.44934463501,
	  14975.474609375, 0.0 },
	{ UDSP_SAMPLE_RATE, RING_FORM_REFUSED, 102090.23774576187,
	  2728.550048828125, 0.0 },
	{ UDSP_SAMPLE_RATE_WB, RING_FORM_REFUSED, 117826.65641379356,
	  2597.143798828125, 0.0 },
	{ UDSP_SAMPLE_RATE, RING_FORM_BACKGROUND, 4363741.2377746105,
	  414770.1875, 17708.9875 },
	{ UDSP_SAMPLE_RATE_WB, RING_FORM_BACKGROUND, 4208286.3980493546,
	  346661.46875, 16424.05 },
};

static const struct lockout_head *lockout_head_get(unsigned rate,
						   enum ring_form form)
{
	unsigned i;

	for (i = 0; i < sizeof(lockout_heads) / sizeof(lockout_heads[0]); i++) {
		if (lockout_heads[i].rate == rate &&
		    lockout_heads[i].form == form)
			return &lockout_heads[i];
	}
	return NULL;
}

/*
 * One leg of ring_lockout_test, frames 1 to 2999: end holds the state after the
 * last frame, and first_trust the first trusted frame after the ringback.
 * restart is its first count reset, complete the frame that reads TRUST_FRAMES
 * after it, and early the trusted frames between.
 */
struct lockout_leg {
	struct ring_census c;
	struct near_stat speech;
	struct trial_marks marks;
	struct udsp_ec_nlms_info end;
	unsigned first_trust;
	unsigned restart;
	unsigned complete;
	unsigned early;
};

static void lockout_trust_note(struct lockout_leg *g,
			       const struct ring_spec *s,
			       const struct udsp_ec_nlms_frame *reading,
			       unsigned f, unsigned last)
{
	if (f <= s->ring_frames)
		return;
	if (!g->first_trust && reading->trusted)
		g->first_trust = f;
	if (!g->restart && reading->trust_run <= 1u &&
	    last > reading->trust_run)
		g->restart = f;
	if (!g->restart || g->complete)
		return;
	g->early += reading->trusted;
	if (reading->trust_run >= TRUST_FRAMES)
		g->complete = f;
}

static void lockout_leg_run(struct ring_spec *s, struct lockout_leg *g)
{
	struct udsp_ec_nlms_frame reading;
	struct ring_planes p;
	struct ring_line line;
	unsigned last = 0;
	unsigned f;

	memset(g, 0, sizeof(*g));
	ring_spec_line_init(&line, s);
	for (f = 1; f < RING_CENSUS_TO; f++) {
		ring_spec_frame_run(&line, s, f, NULL, &p);
		ring_census_add(&g->c, line.ec, &p, line.frame, f);
		trial_marks_add(&g->marks, line.ec);
		near_stat_add(&g->speech, line.ec, p.near, p.mic, p.out,
			      line.frame);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		lockout_trust_note(g, s, &reading, f, last);
		last = reading.trust_run;
	}
	trial_marks_finish(&g->marks);
	udsp_ec_nlms_info_get(line.ec, &g->end);
	udsp_ec_nlms.destroy(line.ec);
}

static void lockout_speech_print(const char *what,
				 const struct near_stat *sp)
{
	printf("# %s, near speech: %.2f dB through on %u frames, output %.2f "
	       "dB against the microphone, %u exposure frames\n", what,
	       near_gain_db_get(sp->product, sp->near), sp->frames,
	       expose_db_get(sp->out, sp->mic), sp->exposure);
}

static void lockout_speech_check(const struct near_stat *sp)
{
	CHECK(sp->frames > 0u);
	CHECK(fabs(near_gain_db_get(sp->product, sp->near)) <= EXPOSE_NEAR_DB);
	CHECK_UINT(sp->exposure, 0);
}

/*
 * The bars of a leg over frames 300 to 2999: 100 leak frames and one lockout
 * second or fewer, and no reset and no trial. out_sum and out_max stand no
 * higher than a canceller without the trust count. A BACKGROUND leg reads the
 * echo loss and the echo leak, and prints the output out_max with no bar.
 */
static void lockout_leg_check(const char *what, const struct ring_spec *s,
			      enum ring_form form, const struct lockout_leg *g)
{
	const struct lockout_head *h = lockout_head_get(s->rate, form);
	bool echo = form == RING_FORM_BACKGROUND;
	const struct ring_census_sel *sel = echo ? &g->c.far_not_near :
						   &g->c.far;
	double max = echo ? sel->leak_max : sel->out_max;
	char list[160];
	unsigned lockout = ring_census_lockout_get(&g->c, echo, list,
						   sizeof(list));

	printf("# %s: %u frames, %u leak frames, out_sum %.9g against %.9g, "
	       "out_max %.9g against %.9g, %u lockout seconds %s; first "
	       "trusted frame %u\n", what, sel->frames, sel->leak,
	       sel->out_sum, h ? h->out_sum : 0.0, sel->out_max,
	       h ? h->out_max : 0.0, lockout, list, g->first_trust);
	if (echo)
		printf("# %s: echo leak max %.9g against %.9g\n", what,
		       sel->leak_max, h ? h->leak_max : 0.0);
	trial_marks_print(what, &g->marks);
	CHECK(h != NULL);
	CHECK_UINT(ring_census_leak_count(&g->c), g->c.far.leak);
	CHECK(sel->leak <= LOCKOUT_LEAK_MAX);
	CHECK(lockout <= LOCKOUT_SECONDS_MAX);
	CHECK(h && sel->out_sum <= h->out_sum);
	CHECK(h && max <= (echo ? h->leak_max : h->out_max));
	CHECK_UINT(g->end.diverge_resets, 0);
	CHECK_UINT(g->end.probes, 0);
	trial_marks_check(&g->marks);
}

/* The census figures of a leg with no bar, from frame 300 to 2999. */
static void lockout_leg_print(const char *what, const struct lockout_leg *g)
{
	char list[160];
	unsigned lockout = ring_census_lockout_get(&g->c, false, list,
						   sizeof(list));

	printf("# %s: %u far frames, %u leak frames, out_sum %.9g, out_max "
	       "%.9g, %u lockout seconds %s; first trusted frame %u, %u "
	       "trials, %u resets\n", what, g->c.far.frames, g->c.far.leak,
	       g->c.far.out_sum, g->c.far.out_max, lockout, list,
	       g->first_trust, g->end.probes, g->end.diverge_resets);
	trial_marks_print(what, &g->marks);
}

/*
 * The INSIDE or REFUSED leg, then the same leg with a dense talker at -25 dBm0.
 * The talker starts 5 s after the first word and talks for 10 s. The INSIDE
 * leg at 16 kHz runs on a field path with no bar. Its filter holds the energy
 * rule through the ringback, so the count completes there and survives the
 * answer gap.
 */
static void lockout_words_leg(unsigned rate, enum ring_form form,
			      const char *what, struct lockout_leg *g)
{
	struct ring_spec s;
	char talk[64];

	ring_spec_set(&s, form, rate);
	lockout_leg_run(&s, g);
	if (s.field)
		lockout_leg_print(what, g);
	else
		lockout_leg_check(what, &s, form, g);
	s.talk_from = s.answer + LOCKOUT_TALK_AFTER;
	s.talk_to = s.talk_from + LOCKOUT_TALK_FRAMES;
	lockout_leg_run(&s, g);
	snprintf(talk, sizeof(talk), "%s with a talker", what);
	lockout_speech_print(talk, &g->speech);
	lockout_speech_check(&g->speech);
}

/* The BACKGROUND leg. The count restarts after the ringback, and the trust
 * must stay withheld from that frame through the frame that completes the
 * count. The census of the output is printed beside the bars. */
static void lockout_background_leg(unsigned rate, struct lockout_leg *g)
{
	struct ring_spec s;
	char list[160];
	unsigned lockout;

	ring_spec_set(&s, RING_FORM_BACKGROUND, rate);
	lockout_leg_run(&s, g);
	lockout = ring_census_lockout_get(&g->c, false, list, sizeof(list));
	lockout_leg_check("background leg", &s, RING_FORM_BACKGROUND, g);
	printf("# background leg, census of the output: %u far frames, %u "
	       "leak frames, out_sum %.9g, out_max %.9g, %u lockout seconds "
	       "%s; count restart on frame %u, complete on frame %u, %u "
	       "trusted frames between\n", g->c.far.frames, g->c.far.leak,
	       g->c.far.out_sum, g->c.far.out_max, lockout, list, g->restart,
	       g->complete, g->early);
	lockout_speech_print("background leg", &g->speech);
	CHECK(g->restart > 0u);
	CHECK(g->complete > g->restart);
	CHECK_UINT(g->early, 0);
	lockout_speech_check(&g->speech);
}

/* The INSIDE form with its first word 100 frames after the ringback, printed
 * with no bar. */
static void lockout_short_leg(unsigned rate, struct lockout_leg *g)
{
	struct ring_spec s;

	ring_spec_set(&s, RING_FORM_INSIDE, rate);
	s.answer = s.ring_frames + LOCKOUT_SHORT_GAP;
	lockout_leg_run(&s, g);
	lockout_leg_print("short gap leg", g);
}

/*
 * The lockout after a ringback that ends without a trial, on the three forms
 * for 30 s with the processor on. Each leg with a bar keeps the bars of
 * lockout_leg_check, and a near talker's speech passes within 2 dB with no
 * exposure frame. A canceller without the trust count fails each.
 */
static void ring_lockout_test(unsigned rate)
{
	static struct lockout_leg g;

	case_rate(rate, "lec ends the lockout after the ringback without a "
		  "trial");
	lockout_words_leg(rate, RING_FORM_INSIDE, "inside leg", &g);
	lockout_words_leg(rate, RING_FORM_REFUSED, "refused leg", &g);
	lockout_background_leg(rate, &g);
	lockout_short_leg(rate, &g);
}

#define RESTART_EXTRA_FRAMES	300u
#define RESTART_TALK_FROM	100u
#define RESTART_TALK_FRAMES	10u
#define RESTART_TALK_OVER_DB	10.0
#define RESTART_SILENT_FROM	200u
#define RESTART_SILENT_FRAMES	50u
#define RESTART_LIMIT_FRAMES	(10u * FRAMES_PER_SECOND)
#define RESTART_BEFORE_STEP	50u
#define RESTART_BEFORE_PAUSE	50u
#define RESTART_STEP_DB		12.0
#define RESTART_TRIAL_FRAMES	30u
#define RESTART_RETURN_TALK	10u

/* One frame of the ring line. far adds the white far end, and talk adds a
 * white talker talk_db over the echo. */
static void restart_frame_run(struct ring_line *line, bool far, bool talk,
			      double talk_db,
			      struct udsp_ec_nlms_frame *reading,
			      struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];

	memset(ref, 0, sizeof(ref));
	if (far)
		noise_fill(ref, line->frame, RING_WORDS_DBM0, &line->far_state);
	if (talk)
		noise_fill(near, line->frame, TALKER_ECHO_DBM0 + talk_db,
			   &line->near_state);
	ring_frame_run(line, ref, talk ? near : NULL, NULL);
	udsp_ec_nlms_frame_get(line->ec, reading);
	udsp_ec_nlms_info_get(line->ec, info);
}

/*
 * A far-end pause whose quiet-window run reaches quiet frames, then the frame
 * that the far end returns on. The window turns quiet when the last loud
 * frame leaves it, so the pause holds quiet plus the window less 1 frames.
 */
static void restart_pause_run(struct ring_line *line, unsigned quiet,
			      struct udsp_ec_nlms_frame *ret)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned silent = quiet + window_frames_get(line->ec) - 1u;
	unsigned f;

	for (f = 0; f < silent; f++)
		restart_frame_run(line, false, false, 0.0, &reading, &info);
	restart_frame_run(line, true, false, 0.0, ret, &info);
}

/*
 * The white far end until the energy rule of dsp/lec.c turns held, at most
 * limit frames. The rule of frame k compares the coefficient energy of info
 * after frame k-1, times 10, with the learned gain after frame k. It returns
 * the frames that ran, or 0 when the rule did not turn, and at the reading of
 * the frame on which it turned.
 */
static unsigned restart_turn_run(struct ring_line *line, unsigned limit,
				 struct udsp_ec_nlms_frame *at)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info before;
	bool held = false;
	unsigned f;

	udsp_ec_nlms_info_get(line->ec, &before);
	for (f = 1; f <= limit; f++) {
		bool rule;

		restart_frame_run(line, true, false, 0.0, &reading, &info);
		rule = before.coef_energy * RING_COEF_RATIO >= info.echo_gain;
		before = info;
		if (rule && !held) {
			*at = reading;
			return f;
		}
		held = rule;
	}
	return 0;
}

/*
 * The white far end until the first trusted frame, at most limit frames. The
 * frames count from 1:
 * - count_at takes the first frame that reads a count of TRUST_FRAMES;
 * - first takes the first frame that reads trusted;
 * - early counts the trusted frames with a count of TRUST_FRAMES or less.
 */
struct restart_complete {
	unsigned count_at;
	unsigned first;
	unsigned early;
};

static void restart_complete_run(struct ring_line *line, unsigned limit,
				 struct restart_complete *c)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned f;

	memset(c, 0, sizeof(*c));
	for (f = 1; f <= limit && !c->first; f++) {
		restart_frame_run(line, true, false, 0.0, &reading, &info);
		if (!c->count_at && reading.trust_run == TRUST_FRAMES)
			c->count_at = f;
		if (reading.trusted && reading.trust_run <= TRUST_FRAMES)
			c->early++;
		if (reading.trusted)
			c->first = f;
	}
}

/* Part 1: the count moves by 1 on a clear active frame and stays on every
 * other frame. A talker burst and a far-end pause give the other frames. */
static void trust_count_part(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_line line;
	unsigned miss = 0;
	unsigned fired = 0;
	unsigned held = 0;
	unsigned quiet = 0;
	unsigned before;
	unsigned f;

	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_frame_get(line.ec, &reading);
	before = reading.trust_run;
	for (f = 0; f < RESTART_EXTRA_FRAMES; f++) {
		bool talk = f >= RESTART_TALK_FROM &&
			    f < RESTART_TALK_FROM + RESTART_TALK_FRAMES;
		bool far = f < RESTART_SILENT_FROM ||
			   f >= RESTART_SILENT_FROM + RESTART_SILENT_FRAMES;
		bool active;

		restart_frame_run(&line, far, talk, RESTART_TALK_OVER_DB,
				  &reading, &info);
		active = reading.ref_power >= reading.ref_bar;
		miss += reading.trust_run !=
			before + (active && !info.double_talk);
		fired += reading.dt_fired;
		held += active && info.double_talk;
		quiet += !active;
		before = reading.trust_run;
	}
	udsp_ec_nlms.destroy(line.ec);
	printf("# count: %u frames off the rule, %u fired, %u held active "
	       "frames, %u inactive frames, count %u at the end\n", miss, fired,
	       held, quiet, before);
	CHECK_UINT(miss, 0);
	CHECK(fired > 0u);
	CHECK(held > 0u);
	CHECK(quiet > 0u);
	CHECK(before > TRUST_FRAMES);
}

/*
 * Parts 2 and 3: a pause of quiet quiet-window frames, RESTART_BEFORE_PAUSE
 * frames after the turn of the rule and before the first trust. A restart
 * then shows as a fall of the count. Part 3 then completes the count.
 */
static void trust_pause_part(unsigned rate, unsigned quiet)
{
	struct udsp_ec_nlms_frame turn;
	struct udsp_ec_nlms_frame before;
	struct udsp_ec_nlms_frame ret;
	struct udsp_ec_nlms_info info;
	struct restart_complete c;
	struct ring_line line;
	unsigned turned;
	unsigned f;

	memset(&turn, 0, sizeof(turn));
	memset(&before, 0, sizeof(before));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_reset(line.ec);
	turned = restart_turn_run(&line, RESTART_LIMIT_FRAMES, &turn);
	for (f = 0; f < RESTART_BEFORE_PAUSE; f++)
		restart_frame_run(&line, true, false, 0.0, &before, &info);
	restart_pause_run(&line, quiet, &ret);
	printf("# pause of %u quiet frames: rule turned on frame %u, count %u "
	       "before the pause and %u on the return\n", quiet, turned,
	       before.trust_run, ret.trust_run);
	CHECK(turned > 0u);
	CHECK(before.trust_run > 1u && before.trust_run < TRUST_FRAMES);
	CHECK(!before.trusted);
	if (quiet < TRUST_PAUSE_FRAMES) {
		CHECK(ret.trust_run >= before.trust_run &&
		      ret.trust_run <= before.trust_run + 1u);
		udsp_ec_nlms.destroy(line.ec);
		return;
	}
	CHECK(ret.trust_run <= 1u);
	CHECK(!ret.trusted);
	restart_complete_run(&line, RESTART_LIMIT_FRAMES, &c);
	printf("# count of %u on frame %u after the return, first trusted "
	       "frame %u, %u trusted frames with the count incomplete\n",
	       TRUST_FRAMES, c.count_at, c.first, c.early);
	CHECK(c.count_at > 0u);
	CHECK_UINT(c.first, c.count_at + 1u);
	CHECK_UINT(c.early, 0);
	udsp_ec_nlms.destroy(line.ec);
}

/* Part 4: a pause after the trust held restarts nothing. */
static void trust_held_part(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame ret;
	struct ring_line line;

	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_frame_get(line.ec, &reading);
	restart_pause_run(&line, TRUST_PAUSE_FRAMES, &ret);
	udsp_ec_nlms.destroy(line.ec);
	printf("# pause after the trust: count %u before the pause, trusted "
	       "%u; return frame: count %u, trusted %u\n", reading.trust_run,
	       reading.trusted, ret.trust_run, ret.trusted);
	CHECK(reading.trusted);
	CHECK(reading.trust_run > TRUST_FRAMES);
	CHECK(ret.trust_run >= reading.trust_run &&
	      ret.trust_run <= reading.trust_run + 1u);
	CHECK(ret.trusted);
}

/* Part 5: the reset and the turn of the rule restart the count. */
static void trust_reset_part(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame turn;
	struct udsp_ec_nlms_info info;
	struct restart_complete c;
	struct ring_line line;
	unsigned turned;

	memset(&turn, 0, sizeof(turn));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_reset(line.ec);
	restart_frame_run(&line, true, false, 0.0, &reading, &info);
	turned = restart_turn_run(&line, RESTART_LIMIT_FRAMES, &turn);
	restart_complete_run(&line, RESTART_LIMIT_FRAMES, &c);
	udsp_ec_nlms.destroy(line.ec);
	printf("# reset: count %u on the first frame; rule turned on frame %u "
	       "after it with a count of %u; count of %u on frame %u after the "
	       "turn, first trusted frame %u\n", reading.trust_run, turned + 1u,
	       turn.trust_run, TRUST_FRAMES, c.count_at, c.first);
	CHECK(reading.trust_run <= 1u);
	CHECK(turned > 0u);
	CHECK(turn.trust_run <= 1u);
	CHECK(c.count_at > 0u);
	CHECK_UINT(c.first, c.count_at + 1u);
	CHECK_UINT(c.early, 0);
}

/*
 * Part 6: a restart inside a refused trial. After the turn of the rule, the
 * hybrid steps up by 12 dB, and the classic bound holds the louder echo until
 * a trial starts. A pause of 2 s falls inside the trial. On the tenth frame
 * of the return, a talker 16 dB over the far end ends the trial.
 */
static void trust_reject_part(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame turn;
	struct udsp_ec_nlms_frame ret;
	struct udsp_ec_nlms_info info;
	struct ring_line line;
	unsigned trusted = 0;
	unsigned start = 0;
	unsigned arm = 0;
	unsigned snap;
	unsigned f;

	memset(&turn, 0, sizeof(turn));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_reset(line.ec);
	(void)restart_turn_run(&line, RESTART_LIMIT_FRAMES, &turn);
	for (f = 0; f < RESTART_BEFORE_STEP; f++)
		restart_frame_run(&line, true, false, 0.0, &reading, &info);
	snap = reading.trust_run;
	path_erl_set(&line.path, RING_ERL_DB - RESTART_STEP_DB);
	for (f = 1; f <= RESTART_LIMIT_FRAMES && !start; f++) {
		restart_frame_run(&line, true, false, 0.0, &reading, &info);
		trusted += reading.trusted;
		if (!info.probes)
			continue;
		start = f;
		arm = reading.trial_arm;
		snap = reading.trust_run;
	}
	for (f = 0; f < RESTART_TRIAL_FRAMES; f++)
		restart_frame_run(&line, true, false, 0.0, &reading, &info);
	restart_pause_run(&line, TRUST_PAUSE_FRAMES, &ret);
	for (f = 2; f < RESTART_RETURN_TALK; f++)
		restart_frame_run(&line, true, false, 0.0, &reading, &info);
	restart_frame_run(&line, true, true, RING_ERL_DB + TALKER_OVER_DB,
			  &reading, &info);
	restart_frame_run(&line, true, false, 0.0, &ret, &info);
	udsp_ec_nlms.destroy(line.ec);
	printf("# trial after the step: %s trial on frame %u, count %u at its "
	       "start, %u trusted frames before it; talker end %s, count %u "
	       "on the next frame\n", lecr_trial_arm_name(arm), start, snap,
	       trusted, lecr_trial_end_name(reading.trial_end), ret.trust_run);
	CHECK(start > 0u);
	CHECK_UINT(trusted, 0);
	CHECK(snap > RESTART_RETURN_TALK);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK(ret.trust_run <= RESTART_RETURN_TALK);
}

/* A far-end pause of TRUST_PAUSE_FRAMES quiet-window frames on the hybrid
 * that amplifies, then the frame that the far end returns on. */
static void trust_refused_pause_run(struct boundary_line *b,
				    struct udsp_ec_nlms_frame *ret)
{
	struct udsp_ec_nlms_info info;
	int16_t ref[UDSP_FRAME_MAX];
	unsigned window;
	unsigned f;

	udsp_ec_nlms_info_get(b->lead, &info);
	window = (info.taps + b->frame - 1u) / b->frame;
	memset(ref, 0, sizeof(ref));
	for (f = 0; f < TRUST_PAUSE_FRAMES + window; f++)
		twin_step(b->lead, b->twin, &b->amp, ref, NULL, b->lead_out,
			  b->twin_out, NULL);
	(void)boundary_amp_run(b, NULL);
	udsp_ec_nlms_frame_get(b->lead, ret);
}

/*
 * Part 7: a refused trial with clear frames, on the line of
 * probe_boundary_test. The trial spends its budget on the far end alone, and
 * the watchdog refuses it on its last frame. That frame must read the count of
 * the frame the trial started on. The trust holds inside the trial only, so a
 * pause of 2 s after the refusal must restart the count.
 */
static void trust_refused_part(unsigned rate)
{
	struct udsp_ec_nlms_frame start;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame ret;
	struct udsp_ec_nlms_info info;
	struct boundary_line b;
	unsigned amp_frames = boundary_amp_count(rate);
	unsigned trusted = 0;
	unsigned spent = 0;
	unsigned most = 0;
	unsigned f;

	boundary_line_init(&b, rate);
	boundary_trial_start(&b, &info);
	udsp_ec_nlms_frame_get(b.lead, &start);
	while (spent + amp_frames < TRIAL_BUDGET) {
		boundary_weak_run(&b, false);
		udsp_ec_nlms_frame_get(b.lead, &reading);
		if (reading.trust_run > most)
			most = reading.trust_run;
		trusted += reading.trusted;
		spent++;
	}
	boundary_flush_run(&b);
	for (f = 1; f <= amp_frames; f++)
		(void)boundary_amp_run(&b, NULL);
	udsp_ec_nlms_frame_get(b.lead, &reading);
	trust_refused_pause_run(&b, &ret);
	udsp_ec_nlms.destroy(b.lead);
	udsp_ec_nlms.destroy(b.twin);
	printf("# refused trial: count %u on its start frame, %u at most on "
	       "its frames, %u on the frame the watchdog ends it; %u trusted "
	       "frames in the trial; count %u on the return after a pause\n",
	       start.trust_run, most, reading.trust_run, trusted,
	       ret.trust_run);
	CHECK(amp_frames > 1u);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_DIVERGED);
	CHECK(most > start.trust_run);
	CHECK_UINT(reading.trust_run, start.trust_run);
	CHECK(!start.trusted);
	CHECK(trusted > 0u);
	CHECK(ret.trust_run <= 1u);
}

#define RESTART_FREEZE_FRAMES	50u

/* Part 8: a frozen frame counts. The freeze holds the filter, and a clear
 * active frame under it still moves the tracker and the count. */
static void trust_freeze_part(unsigned rate)
{
	struct udsp_ec_nlms_frame before;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info start;
	struct udsp_ec_nlms_info info;
	struct ring_line line;
	unsigned clear = 0;
	unsigned f;

	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_frame_get(line.ec, &before);
	udsp_ec_nlms_info_get(line.ec, &start);
	udsp_ec_nlms.freeze(line.ec, true);
	for (f = 0; f < RESTART_FREEZE_FRAMES; f++) {
		restart_frame_run(&line, true, false, 0.0, &reading, &info);
		clear += reading.ref_power >= reading.ref_bar &&
			 !info.double_talk;
	}
	udsp_ec_nlms.destroy(line.ec);
	printf("# freeze: count %u before and %u after %u frozen frames, %u of "
	       "them clear, %u held\n", before.trust_run, reading.trust_run,
	       RESTART_FREEZE_FRAMES, clear,
	       info.hold_frames - start.hold_frames);
	CHECK(clear > 0u);
	CHECK_UINT(info.hold_frames - start.hold_frames, RESTART_FREEZE_FRAMES);
	CHECK_UINT(reading.trust_run, before.trust_run + clear);
}

#define RESTART_BG_DBM0		-48.0
#define RESTART_BG_FLOOR_DBM0	-85.0
#define RESTART_BG_SEED		0x5eed1234u
#define RESTART_BG_LEAD		400u
#define RESTART_BG_TALK_DBM0	-30.0
#define RESTART_BG_RUN		497u
#define RESTART_BG_TAIL		6u
#define RESTART_BG_PAUSE	260u

/* One frame of the ring line over a white far-end background. far adds the
 * white far end, and talk adds a white talker at talk_dbm0. */
static void restart_bg_frame_run(struct ring_line *line, uint32_t *bg_state,
				 bool far, bool talk, double talk_dbm0,
				 struct udsp_ec_nlms_frame *reading,
				 struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t part[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];

	noise_fill(ref, line->frame, RESTART_BG_DBM0, bg_state);
	if (far) {
		noise_fill(part, line->frame, RING_WORDS_DBM0,
			   &line->far_state);
		ring_part_add(ref, part, line->frame);
	}
	if (talk)
		noise_fill(near, line->frame, talk_dbm0, &line->near_state);
	ring_frame_run(line, ref, talk ? near : NULL, NULL);
	udsp_ec_nlms_frame_get(line->ec, reading);
	udsp_ec_nlms_info_get(line->ec, info);
}

/*
 * Part 9: a restart and a trial on one frame before the first trust. The latch
 * comes due at the end of a far phrase, and a background holds the run through
 * a 2.6 s pause. The return restarts the count and starts a trial, which a
 * talker ends; the count reads 0, as the trial keeps no count from before it.
 */
static void trust_return_trial_part(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame ret;
	struct udsp_ec_nlms_info info;
	struct ring_line line;
	uint32_t bg_state = RESTART_BG_SEED;
	uint32_t probes;
	unsigned trusted = 0;
	unsigned before;
	unsigned run;
	unsigned end;
	unsigned f;

	ring_line_init(&line, rate);
	line.floor_dbm0 = RESTART_BG_FLOOR_DBM0;
	for (f = 0; f < RESTART_BG_LEAD; f++) {
		restart_bg_frame_run(&line, &bg_state, false, false, 0.0,
				     &reading, &info);
		trusted += reading.trusted;
	}
	for (f = 0; f < RESTART_LIMIT_FRAMES &&
		    reading.dt_run < RESTART_BG_RUN; f++) {
		restart_bg_frame_run(&line, &bg_state, true, true,
				     RESTART_BG_TALK_DBM0, &reading, &info);
		trusted += reading.trusted;
	}
	for (f = 0; f < RESTART_BG_TAIL; f++)
		restart_bg_frame_run(&line, &bg_state, false, true,
				     RESTART_BG_TALK_DBM0, &reading, &info);
	for (f = 0; f < RESTART_BG_PAUSE; f++)
		restart_bg_frame_run(&line, &bg_state, false, false, 0.0,
				     &reading, &info);
	before = reading.trust_run;
	run = reading.dt_run;
	probes = info.probes;
	restart_bg_frame_run(&line, &bg_state, true, false, 0.0, &ret, &info);
	for (f = 2; f < RESTART_RETURN_TALK; f++)
		restart_bg_frame_run(&line, &bg_state, true, false, 0.0,
				     &reading, &info);
	restart_bg_frame_run(&line, &bg_state, true, true,
			     RING_WORDS_DBM0 + TALKER_OVER_DB, &reading, &info);
	end = reading.trial_end;
	restart_bg_frame_run(&line, &bg_state, true, false, 0.0, &reading,
			     &info);
	udsp_ec_nlms.destroy(line.ec);
	printf("# trial on the return: count %u and latch run %u before the "
	       "return, %u trusted frames before it; return frame: %s trial, "
	       "count %u; talker end %s, count %u on the next frame\n", before,
	       run, trusted, lecr_trial_arm_name(ret.trial_arm), ret.trust_run,
	       lecr_trial_end_name(end), reading.trust_run);
	CHECK_UINT(trusted, 0);
	CHECK(before > RESTART_RETURN_TALK);
	CHECK(run >= STEP_LATCH_FRAMES);
	CHECK_UINT(probes, 0);
	CHECK_UINT(ret.trial_arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK(ret.trust_run <= 1u);
	CHECK_UINT(end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK_UINT(reading.trust_run, 0);
}

#define RESTART_LATCH_TALK_DB	14.0
#define RESTART_DROP_DB		30.0
#define RESTART_DROP_FRAMES	40u

/*
 * Part 10: a refused trial in which the energy rule turns. On the start frame
 * the hybrid falls 30 dB, and the trial shrinks the filter. The rule refuses on
 * its last frame before a talker ends it. The refusal gives back the filter,
 * gain and rule of the start frame, so the count does not restart.
 */
static void trust_rule_part(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame start;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info prev;
	struct ring_line line;
	bool rule = true;
	unsigned f;

	memset(&start, 0, sizeof(start));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	udsp_ec_nlms_info_get(line.ec, &info);
	for (f = 0; f < RESTART_LIMIT_FRAMES && !info.probes; f++)
		restart_frame_run(&line, true, true, RESTART_LATCH_TALK_DB,
				  &start, &info);
	path_erl_set(&line.path, RING_ERL_DB + RESTART_DROP_DB);
	prev = info;
	for (f = 0; f < RESTART_DROP_FRAMES; f++) {
		restart_frame_run(&line, true, false, 0.0, &reading, &info);
		rule = prev.coef_energy * RING_COEF_RATIO >= info.echo_gain;
		prev = info;
	}
	restart_frame_run(&line, true, true, RING_ERL_DB + TALKER_OVER_DB,
			  &reading, &info);
	udsp_ec_nlms.destroy(line.ec);
	printf("# rule turn in a trial: %s trial with count %u, trusted %u; "
	       "rule %u on its last frame; talker end %s, count %u, trusted "
	       "%u\n", lecr_trial_arm_name(start.trial_arm), start.trust_run,
	       start.trusted, rule, lecr_trial_end_name(reading.trial_end),
	       reading.trust_run, reading.trusted);
	CHECK_UINT(start.trial_arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	CHECK(start.trusted);
	CHECK(!rule);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK_UINT(reading.trust_run, start.trust_run);
	CHECK(reading.trusted);
}

/*
 * The trust count on the ring line, in ten parts. A clear active frame counts,
 * also under a freeze, and a pause of 200 quiet frames before the first trust
 * restarts it; one of 199 does not. A later pause, the reset and the rule's
 * turn act as stated, and a refused trial keeps the smaller count and its rule.
 */
static void trust_restart_test(unsigned rate)
{
	case_rate(rate, "lec restarts the trust count at a far return, a rule "
		  "turn and a reset");
	trust_count_part(rate);
	trust_pause_part(rate, TRUST_PAUSE_FRAMES - 1u);
	trust_pause_part(rate, TRUST_PAUSE_FRAMES);
	trust_held_part(rate);
	trust_reset_part(rate);
	trust_reject_part(rate);
	trust_refused_part(rate);
	trust_freeze_part(rate);
	trust_return_trial_part(rate);
	trust_rule_part(rate);
}

/* The line of the snapshot cases: a flat 12 dB hybrid at 8 ms, a white far
 * end at SNAP_FAR_DBM0 and no near floor. It trains for 1.5 s. */
#define WINDOW_ERL_DB		12.0
#define WINDOW_TRAIN_FRAMES	(FRAMES_PER_SECOND + FRAMES_PER_SECOND / 2)
#define WINDOW_TALK_FRAMES	150u
#define WINDOW_AFTER_FRAMES	(2u * FRAMES_PER_SECOND)
#define WINDOW_ERLE_FRAMES	FRAMES_PER_SECOND
/* The talker frames that adapt on a canceller without the trust count. */
#define WINDOW_HEAD_ADAPTED	0u

/*
 * The figures of trust_window_talker_test. The energies give the filter
 * ERLE over the last second of the training and over the second that ends
 * 2 s after the talker stops. first_trust counts the frames from the return
 * to the first trusted one. early counts the trusted frames from the return
 * through the frame that completes the count.
 */
struct window_talk {
	struct near_stat speech;
	unsigned adapted;
	unsigned trained_trust;
	unsigned count_at_pause;
	unsigned return_count;
	unsigned first_trust;
	unsigned early;
	bool complete;
	double echo_dbm0;
	double before_echo;
	double before_out;
	double after_echo;
	double after_out;
};

/* One frame of the snapshot line, with near on top when it is given. */
static void window_frame_run(struct ring_line *line, bool far,
			     const int16_t *near, struct ring_planes *p)
{
	int16_t ref[UDSP_FRAME_MAX];

	memset(ref, 0, sizeof(ref));
	if (far)
		noise_fill(ref, line->frame, SNAP_FAR_DBM0, &line->far_state);
	ring_planes_run(line, ref, near, NULL, p);
}

static void window_energy_add(const struct ring_planes *p, unsigned frame,
			      double *echo, double *out)
{
	unsigned i;

	for (i = 0; i < frame; i++) {
		*echo += (double)p->echo[i] * p->echo[i];
		*out += (double)p->out[i] * p->out[i];
	}
}

/* The trust of frame k after the return, which reads the count of the frame
 * before it. */
static void window_trust_note(struct window_talk *w,
			      const struct ring_line *line, unsigned k)
{
	struct udsp_ec_nlms_frame reading;

	udsp_ec_nlms_frame_get(line->ec, &reading);
	if (k == 1u)
		w->return_count = reading.trust_run;
	if (!w->first_trust && reading.trusted)
		w->first_trust = k;
	if (w->complete)
		return;
	w->early += reading.trusted;
	w->complete = reading.trust_run >= TRUST_FRAMES;
}

/* The talker frames of the return, at the level of the echo. */
static void window_talk_run(struct window_talk *w, struct ring_line *line,
			    unsigned *k)
{
	struct udsp_ec_nlms_info info;
	int16_t near[UDSP_FRAME_MAX];
	struct ring_planes p;
	unsigned f;

	for (f = 0; f < WINDOW_TALK_FRAMES; f++) {
		uint32_t held = line->held;

		noise_fill(near, line->frame, w->echo_dbm0, &line->near_state);
		window_frame_run(line, true, near, &p);
		near_stat_add(&w->speech, line->ec, p.near, p.mic, p.out,
			      line->frame);
		udsp_ec_nlms_info_get(line->ec, &info);
		w->adapted += info.hold_frames == held;
		window_trust_note(w, line, ++*k);
	}
}

static void window_run(unsigned rate, struct window_talk *w)
{
	struct udsp_ec_nlms_frame reading;
	struct ring_planes p;
	struct ring_line line;
	unsigned window;
	unsigned k = 0;
	unsigned f;

	memset(w, 0, sizeof(*w));
	ring_line_init(&line, rate);
	path_flat_init(&line.path, rate, 8, WINDOW_ERL_DB);
	line.floor = false;
	window = window_frames_get(line.ec);
	for (f = 0; f < WINDOW_TRAIN_FRAMES; f++) {
		window_frame_run(&line, true, NULL, &p);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		w->trained_trust += reading.trusted;
		if (f + WINDOW_ERLE_FRAMES >= WINDOW_TRAIN_FRAMES)
			window_energy_add(&p, line.frame, &w->before_echo,
					  &w->before_out);
	}
	w->count_at_pause = reading.trust_run;
	w->echo_dbm0 = energy_dbm0_get(w->before_echo,
				       WINDOW_ERLE_FRAMES * line.frame);
	for (f = 0; f < TRUST_PAUSE_FRAMES + window - 1u; f++)
		window_frame_run(&line, false, NULL, &p);
	window_talk_run(w, &line, &k);
	for (f = 0; f < WINDOW_AFTER_FRAMES; f++) {
		window_frame_run(&line, true, NULL, &p);
		window_trust_note(w, &line, ++k);
		if (f + WINDOW_ERLE_FRAMES >= WINDOW_AFTER_FRAMES)
			window_energy_add(&p, line.frame, &w->after_echo,
					  &w->after_out);
	}
	udsp_ec_nlms.destroy(line.ec);
}

/*
 * A talker at echo level while the trust count is open: after 1.5 s of training
 * and a 200-frame pause, a white talker joins the return for 150 frames. The
 * trust stays withheld until the count completes. Without the count, the line
 * holds every talker frame.
 */
static void trust_window_talker_test(unsigned rate)
{
	struct window_talk w;
	const struct near_stat *sp = &w.speech;

	window_run(rate, &w);

	case_rate(rate, "lec reports a near talker at the echo level while the "
		  "trust count is open");
	printf("# count %u at the pause, %u trusted frames before it; talker "
	       "at %.2f dBm0: %u of %u frames adapted, against %u before the "
	       "trust count; near speech %.2f dB through; filter ERLE %.2f dB "
	       "before the pause and %.2f dB 2 s after the talker; first "
	       "trusted frame %u after the return\n", w.count_at_pause,
	       w.trained_trust, w.echo_dbm0, w.adapted, WINDOW_TALK_FRAMES,
	       WINDOW_HEAD_ADAPTED, near_gain_db_get(sp->product, sp->near),
	       expose_db_get(w.before_echo, w.before_out),
	       expose_db_get(w.after_echo, w.after_out), w.first_trust);
	CHECK(w.count_at_pause < TRUST_FRAMES);
	CHECK_UINT(w.trained_trust, 0);
	CHECK(w.return_count <= 1u);
	CHECK(w.complete);
	CHECK_UINT(w.early, 0);
}

/* The form of trust_turn_test: a word talker, which leaves the floor, where a
 * stationary talker in a pause raises the floor and covers itself at the
 * return. */
#define TURN_CYCLES		10u
#define TURN_CYCLE_FRAMES	700u
#define TURN_FAR_FRAMES		400u
#define TURN_OVERLAP_FRAMES	150u
#define TURN_OVER_DB		6.0
#define TURN_ERLE_TOL_DB	1.0
#define TURN_TALKER_SEED	0x31415926u

/*
 * The figures of a canceller without the trust count for trust_turn_test. Each
 * row holds one rate and one talker level: the echo level, or 6 dB over it.
 * They give the mean filter ERLE and the adapted talker frames of the far
 * returns.
 */
struct turn_head {
	unsigned rate;
	double over_db;
	double erle_db;
	unsigned adapted;
};

static const struct turn_head turn_heads[] = {
	{ UDSP_SAMPLE_RATE, 0.0, 28.38, 174u },
	{ UDSP_SAMPLE_RATE, TURN_OVER_DB, 33.15, 125u },
	{ UDSP_SAMPLE_RATE_WB, 0.0, 28.47, 200u },
	{ UDSP_SAMPLE_RATE_WB, TURN_OVER_DB, 32.79, 125u },
};

static const struct turn_head *turn_head_get(unsigned rate, double over_db)
{
	unsigned i;

	for (i = 0; i < sizeof(turn_heads) / sizeof(turn_heads[0]); i++) {
		if (turn_heads[i].rate == rate &&
		    turn_heads[i].over_db == over_db)
			return &turn_heads[i];
	}
	return NULL;
}

/*
 * The figures of trust_turn_test: the mean filter ERLE, and the lapses of
 * the count after it first passed TRUST_FRAMES. talk counts the talker
 * frames of the far returns whose own reference is active. trusted and
 * adapted count those of them that read trusted, and those that adapted.
 */
struct turn_result {
	double erle;
	unsigned lapses;
	unsigned talk;
	unsigned trusted;
	unsigned adapted;
	bool passed;
	uint32_t probes;
	uint32_t resets;
};

/* The talker frames of a far return, after the frame ran. */
static void turn_return_note(const struct ring_line *line, uint32_t held,
			     struct turn_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;

	udsp_ec_nlms_frame_get(line->ec, &reading);
	udsp_ec_nlms_info_get(line->ec, &info);
	if (!(reading.ref_power >= reading.ref_bar))
		return;
	r->talk++;
	r->trusted += reading.trusted;
	r->adapted += info.hold_frames == held;
}

/*
 * One cycle of trust_turn_test: far words for 4 s and a pause of 3 s. A word
 * talker over_db over the echo speaks for 1.5 s on each side of the return.
 */
static double turn_cycle_run(struct ring_line *line, struct word_source *talker,
			     struct turn_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct ring_stat st;
	unsigned f;

	memset(&st, 0, sizeof(st));
	for (f = 0; f < TURN_CYCLE_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		bool talk = f < TURN_OVERLAP_FRAMES ||
			    f >= TURN_CYCLE_FRAMES - TURN_OVERLAP_FRAMES;
		bool alone = f >= TURN_OVERLAP_FRAMES && f < TURN_FAR_FRAMES;
		uint32_t held = line->held;

		if (f == 0u)
			word_source_restart(&line->far);
		if (f == TURN_CYCLE_FRAMES - TURN_OVERLAP_FRAMES)
			word_source_restart(talker);
		memset(ref, 0, sizeof(ref));
		if (f < TURN_FAR_FRAMES)
			word_fill(&line->far, ref, line->frame);
		if (talk)
			word_fill(talker, near, line->frame);
		ring_frame_run(line, ref, talk ? near : NULL,
			       alone ? &st : NULL);
		if (f < TURN_OVERLAP_FRAMES)
			turn_return_note(line, held, r);
		udsp_ec_nlms_frame_get(line->ec, &reading);
		if (reading.trust_run > TRUST_FRAMES)
			r->passed = true;
		r->lapses += r->passed && reading.trust_run <= 1u;
	}
	return weak_filter_erle_get(st.echo, st.residual, st.floor);
}

static void turn_run(unsigned rate, double over_db, struct turn_result *r)
{
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct ring_line line;
	double sum = 0.0;
	unsigned c;

	memset(r, 0, sizeof(*r));
	ring_line_init(&line, rate);
	word_source_init(&talker, rate, TALKER_ECHO_DBM0 + over_db,
			 TURN_TALKER_SEED);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (c = 0; c < TURN_CYCLES; c++)
		sum += turn_cycle_run(&line, &talker, r);
	udsp_ec_nlms_info_get(line.ec, &info);
	udsp_ec_nlms.destroy(line.ec);
	r->erle = sum / TURN_CYCLES;
	r->probes = info.probes;
	r->resets = info.diverge_resets;
}

/*
 * Ten cycles of far words for 4 s and a far pause of 3 s, with a word talker
 * overlapping each return by 1.5 s. Talker frames adapt no more often than on a
 * canceller without the trust count (turn_heads), and the ERLE stays within 1
 * dB of it. After the count first passes 200 it never reads 0 or 1.
 */
static void trust_turn_test(unsigned rate)
{
	static const double over[2] = { 0.0, TURN_OVER_DB };
	unsigned i;

	case_rate(rate, "lec keeps its trust through the far pauses of a "
		  "two-way call");
	for (i = 0; i < 2u; i++) {
		const struct turn_head *h = turn_head_get(rate, over[i]);
		struct turn_result r;

		turn_run(rate, over[i], &r);
		printf("# talker %.0f dB over the echo: filter ERLE %.2f dB "
		       "against %.2f dB; talker frames of the returns: %u "
		       "trusted and %u adapted of %u, against %u adapted; %u "
		       "lapses of the count, %u trials, %u resets\n", over[i],
		       r.erle, h ? h->erle_db : 0.0, r.trusted, r.adapted,
		       r.talk, h ? h->adapted : 0u, r.lapses, r.probes,
		       r.resets);
		CHECK(h != NULL);
		CHECK(r.passed);
		CHECK(r.talk > 0u);
		CHECK(h && fabs(r.erle - h->erle_db) <= TURN_ERLE_TOL_DB);
		CHECK(h && r.adapted <= h->adapted);
		CHECK_UINT(r.lapses, 0);
	}
}

/* The loss of 18 half-dB (REF_LOSS_DB), the default SLIC receive gain of
 * uvoice; REF_LOSS_SEED is the learned gain an instance starts from under it,
 * LEC_GEIGEL_GAIN_MIN raised by that loss. */
#define REF_LOSS_HALF_DB	18u
#define REF_LOSS_DB		9.0
#define REF_LOSS_SEED		(0.25 * pow(10.0, REF_LOSS_DB / 10.0))
#define REF_LOSS_GATE_DB	33.0
#define REF_LOSS_ERLE_TOL_DB	1.0
#define REF_LOSS_SHARE_TOL	5.0
#define REF_LOSS_WORD_FRAMES	(30u * FRAMES_PER_SECOND)
#define REF_LOSS_WORD_ERL_DB	20.0
#define REF_LOSS_WORD_FLOOR_DBM0	-65.0

static bool ref_loss_seed_ok(float echo_gain, double seed)
{
	return fabs((double)echo_gain - seed) <= 1e-5 * seed;
}

/*
 * A fresh instance reads 0. The loss survives the reset, and the same loss
 * again leaves a running instance alone. A changed loss on an instance that
 * ran resets it, and each reset seeds the learned gain of the loss. A loss
 * set before the first frame seeds it with no reset.
 */
static void ref_loss_setter_test(unsigned rate)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state = 0x13572468u;

	case_rate(rate, "lec reference loss reads 0 fresh and survives the "
		  "reset");
	CHECK(ec != NULL);
	if (!ec)
		return;
	CHECK_UINT(udsp_ec_nlms_ref_loss_get(ec), 0);
	CHECK_UINT(udsp_ec_nlms_ref_loss_get(NULL), 0);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK(info.echo_gain == 0.25f);
	udsp_ec_nlms.nlp(ec, false);
	udsp_ec_nlms_ref_loss_set(ec, REF_LOSS_HALF_DB);
	path_flat_init(&path, rate, 8, 12.0);
	erle_offset_run(ec, &path, &state, -10.0, 0, REF_LOSS_HALF_DB,
			FRAMES_PER_SECOND, 1u);
	udsp_ec_nlms_ref_loss_set(ec, REF_LOSS_HALF_DB);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(info.frames, FRAMES_PER_SECOND);
	CHECK(info.coef_energy > 0.0f);
	udsp_ec_nlms_reset(ec);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(udsp_ec_nlms_ref_loss_get(ec), REF_LOSS_HALF_DB);
	CHECK_UINT(info.frames, 0);
	CHECK(ref_loss_seed_ok(info.echo_gain, REF_LOSS_SEED));
	erle_offset_run(ec, &path, &state, -10.0, 0, REF_LOSS_HALF_DB,
			4u * FRAMES_PER_SECOND, 1u);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK(info.ref_floor > 0.0f);
	/* The far-end floor stands in the scale before the change, so it starts
	 * over. */
	udsp_ec_nlms_ref_loss_set(ec, 12u);
	udsp_ec_nlms_info_get(ec, &info);
	CHECK_UINT(udsp_ec_nlms_ref_loss_get(ec), 12);
	CHECK_UINT(info.frames, 0);
	CHECK(info.coef_energy == 0.0f);
	CHECK(info.ref_floor == 0.0f);
	CHECK(ref_loss_seed_ok(info.echo_gain, 0.25 * pow(10.0, 0.6)));
	udsp_ec_nlms.destroy(ec);

	case_rate(rate, "lec reference loss before the first frame seeds the "
		  "learned gain");
	ec = udsp_ec_nlms.create_rate(0, rate);
	CHECK(ec != NULL);
	if (!ec)
		return;
	udsp_ec_nlms_ref_loss_set(ec, REF_LOSS_HALF_DB);
	udsp_ec_nlms_info_get(ec, &info);
	printf("# echo_gain %.6f, the seed %.6f\n", (double)info.echo_gain,
	       REF_LOSS_SEED);
	CHECK_UINT(info.frames, 0);
	CHECK(ref_loss_seed_ok(info.echo_gain, REF_LOSS_SEED));
	udsp_ec_nlms.destroy(ec);
}

/*
 * The combined-loss arm with the loss alone moved: the probe of
 * nlp_combined_loss_test on the 12 dB hybrid at 0 and at 18. The ERLE stays
 * between the floor and the fast path. The combined loss stands under the gate
 * at 0, and the 9 dB of the loss take it over.
 */
static void ref_loss_gate_test(unsigned rate)
{
	struct gate_probe plain;
	struct gate_probe lossy;
	double plain_db;
	double lossy_db;

	nlp_gate_probe(rate, 0, 12.0, 36, &plain);
	nlp_gate_probe(rate, REF_LOSS_HALF_DB, 12.0, 36, &lossy);
	plain_db = plain.erle - 10.0 * log10(2.0 * plain.echo_gain);
	lossy_db = lossy.erle - 10.0 * log10(2.0 * lossy.echo_gain) +
		   REF_LOSS_DB;

	case_rate(rate, "lec reference loss carries the combined loss over "
		  "the gate");
	printf("# erle %.2f and %.2f dB, combined loss %.2f dB at 0 and "
	       "%.2f dB at %u half-dB\n", (double)plain.erle,
	       (double)lossy.erle, plain_db, lossy_db, REF_LOSS_HALF_DB);
	CHECK(plain.erle >= 6.0f && plain.erle < 18.0f);
	CHECK(lossy.erle >= 6.0f && lossy.erle < 18.0f);
	CHECK(plain_db < REF_LOSS_GATE_DB);
	CHECK(lossy_db >= REF_LOSS_GATE_DB);
	CHECK(!plain.nlp_far);
	CHECK(lossy.nlp_far);
	CHECK(!lossy.nlp_doubletalk);
}

/* The balance shape of path_weak_init 20 dB under the word source, with a near
 * floor and no talker. The processor acts on most active frames, the ERLE gate
 * refuses some, and the microphone is the same at every loss. */
static void ref_loss_words_run(unsigned rate, unsigned half_db,
			       struct udsp_ec_nlms_info *info)
{
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	struct word_source far;
	struct echo_path path;
	uint32_t floor_state = 0x0badcafeu;
	unsigned frame = rate / 100u;
	unsigned f;

	udsp_ec_nlms_ref_loss_set(ec, half_db);
	path_weak_init(&path, rate, REF_LOSS_WORD_ERL_DB);
	word_source_init(&far, rate, WEAK_FAR_DBM0, 0x13572468u);
	for (f = 0; f < REF_LOSS_WORD_FRAMES; f++) {
		int16_t line[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		unsigned i;

		word_fill(&far, line, frame);
		path_run(&path, line, echo);
		noise_fill(hiss, frame, REF_LOSS_WORD_FLOOR_DBM0,
			   &floor_state);
		for (i = 0; i < frame; i++)
			mic[i] = sample_clamp((double)echo[i] + hiss[i]);
		level_loss_apply(ref, line, frame, half_db);
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, info);
	udsp_ec_nlms.destroy(ec);
}

/* A count over the window-active frames, the five counts other than
 * nlp_quiet_frames, in percent. */
static double ref_loss_share_get(const struct udsp_ec_nlms_info *info,
				 uint32_t count)
{
	uint32_t judged = info->nlp_active_frames + info->nlp_wait_frames +
			  info->nlp_erle_frames + info->nlp_unlike_frames +
			  info->nlp_dt_frames;

	return judged ? 100.0 * count / judged : 0.0;
}

/*
 * A reference 9 dB lower over a path 9 dB stronger, with the loss of 18, runs
 * against the line at 0. Both arms see the same microphone. One second on
 * the composite hybrid compares the ERLE of the last 20 frames. The word source
 * on the weak hybrid compares the verdict shares and the resets.
 */
static void ref_loss_equiv_test(unsigned rate)
{
	struct udsp_ec_nlms_info plain;
	struct udsp_ec_nlms_info lossy;
	struct echo_path path;
	double erle[2];
	double share[2][2];
	unsigned i;

	for (i = 0; i < 2u; i++) {
		struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, rate);
		uint32_t state = 0x0f1e2d3cu;
		unsigned half_db = i ? REF_LOSS_HALF_DB : 0u;

		udsp_ec_nlms.nlp(ec, false);
		udsp_ec_nlms_ref_loss_set(ec, half_db);
		path_composite_init(&path, rate, 12.0);
		erle[i] = erle_offset_run(ec, &path, &state, -10.0, 0, half_db,
					  FRAMES_PER_SECOND, 20);
		udsp_ec_nlms.destroy(ec);
	}
	case_rate(rate, "lec reference loss converges as the line at 0");
	printf("# erle %.2f dB at 0, %.2f dB at %u half-dB\n", erle[0],
	       erle[1], REF_LOSS_HALF_DB);
	CHECK(erle[0] >= 25.0);
	CHECK(fabs(erle[1] - erle[0]) <= REF_LOSS_ERLE_TOL_DB);

	ref_loss_words_run(rate, 0, &plain);
	ref_loss_words_run(rate, REF_LOSS_HALF_DB, &lossy);
	share[0][0] = ref_loss_share_get(&plain, plain.nlp_active_frames);
	share[0][1] = ref_loss_share_get(&plain, plain.nlp_erle_frames);
	share[1][0] = ref_loss_share_get(&lossy, lossy.nlp_active_frames);
	share[1][1] = ref_loss_share_get(&lossy, lossy.nlp_erle_frames);
	case_rate(rate, "lec reference loss keeps the verdict shares of the "
		  "line at 0");
	printf("# active %.1f%% and %.1f%%, erle %.1f%% and %.1f%%, resets "
	       "%u and %u, trials %u and %u\n", share[0][0], share[1][0],
	       share[0][1], share[1][1], plain.diverge_resets,
	       lossy.diverge_resets, plain.probes, lossy.probes);
	CHECK(share[0][0] > 0.0);
	CHECK(share[0][1] > 0.0);
	CHECK(fabs(share[1][0] - share[0][0]) <= REF_LOSS_SHARE_TOL);
	CHECK(fabs(share[1][1] - share[0][1]) <= REF_LOSS_SHARE_TOL);
	CHECK_UINT(lossy.diverge_resets, plain.diverge_resets);
}

/* The loss of each instance that a NEW record of the fixture created. */
struct fixture_loss {
	unsigned fresh;
	unsigned ec_loss;
	unsigned shadow_loss;
	bool shadow;
};

static void fixture_loss_seen(void *ctx, const struct lecr_record *rec,
			      const struct lecr_replay *r,
			      const struct lecr_step *st,
			      const struct lecr_agree *a, bool closed)
{
	struct fixture_loss *seen = ctx;

	(void)a;
	(void)closed;
	if (!rec || !st->fresh)
		return;
	seen->fresh++;
	seen->ec_loss = udsp_ec_nlms_ref_loss_get(r->ec);
	seen->shadow = r->shadow != NULL;
	seen->shadow_loss = udsp_ec_nlms_ref_loss_get(r->shadow);
}

static void fixture_loss_replay(char *buf, size_t size,
				struct fixture_loss *seen)
{
	struct lecr_replay r;
	struct lecr_agree a;
	struct lecr_file file;
	FILE *f = fmemopen(buf, size, "rb");

	memset(seen, 0, sizeof(*seen));
	if (!f)
		return;
	lecr_replay_init(&r, true);
	lecr_agree_init(&a, 0);
	lecr_stream_replay(f, &file, &r, &a, fixture_loss_seen, seen);
	fclose(f);
}

/* The bytes of the file that path names, or NULL. The caller frees them. */
static char *fixture_bytes_get(const char *path, size_t *size)
{
	FILE *f = fopen(path, "rb");
	char *buf = NULL;
	long end;

	if (!f)
		return NULL;
	if (!fseek(f, 0, SEEK_END) && (end = ftell(f)) > 0 &&
	    !fseek(f, 0, SEEK_SET)) {
		buf = malloc((size_t)end);
		*size = (size_t)end;
	}
	if (buf && fread(buf, 1, *size, f) != *size) {
		free(buf);
		buf = NULL;
	}
	fclose(f);
	return buf;
}

/*
 * The replay of the fixture of capture_fixture_test gives the loss of its
 * header to the instance and to its shadow. The same bytes with flag bit 1
 * clear replay at 0, although offset 18 still holds the loss.
 */
static void capture_fixture_loss_test(void)
{
	const char *path = getenv("UDSP_LEC_CAPTURE_FIXTURE");
	struct fixture_loss seen;
	size_t size = 0;
	char *buf;

	CASE("lec capture fixture gives its reference loss to both instances");
	if (!path) {
		tu_skip("UDSP_LEC_CAPTURE_FIXTURE not set");
		CASE("lec capture fixture with the loss flag clear replays at 0");
		tu_skip("UDSP_LEC_CAPTURE_FIXTURE not set");
		return;
	}
	buf = fixture_bytes_get(path, &size);
	CHECK(buf != NULL && size > LECR_FILE_BYTES);
	if (!buf || size <= LECR_FILE_BYTES) {
		free(buf);
		return;
	}
	fixture_loss_replay(buf, size, &seen);
	CHECK_UINT(seen.fresh, 1);
	CHECK(seen.shadow);
	CHECK_UINT(seen.ec_loss, REF_LOSS_HALF_DB);
	CHECK_UINT(seen.shadow_loss, REF_LOSS_HALF_DB);

	CASE("lec capture fixture with the loss flag clear replays at 0");
	buf[24] = (char)(buf[24] & ~LECR_FILE_REF_LOSS);
	fixture_loss_replay(buf, size, &seen);
	CHECK_UINT(seen.fresh, 1);
	CHECK(seen.shadow);
	CHECK_UINT(seen.ec_loss, 0);
	CHECK_UINT(seen.shadow_loss, 0);
	free(buf);
}

/* A header loss is read with a bound of 30 dB: the reach of the engine, and
 * the point past which the scaled rules leave their range. */
static int header_loss_open(uint16_t half_db)
{
	struct lecr_file file;
	uint8_t head[LECR_FILE_BYTES];
	FILE *f;
	int rc;

	memset(&file, 0, sizeof(file));
	file.rate = UDSP_SAMPLE_RATE;
	file.samples = UDSP_FRAME_SAMPLES;
	lecr_file_ref_loss_set(&file, half_db);
	lecr_file_encode(head, &file);
	f = fmemopen(head, sizeof(head), "rb");
	if (!f)
		return -ENOMEM;
	rc = lecr_read_open(f, &file);
	fclose(f);
	return rc;
}

static void ref_loss_bound_test(void)
{
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(48, UDSP_SAMPLE_RATE);

	CASE("lec reference loss over the reach of the engine clamps to it");
	udsp_ec_nlms_ref_loss_set(ec, UDSP_EC_NLMS_REF_LOSS_MAX + 1u);
	CHECK_UINT(udsp_ec_nlms_ref_loss_get(ec), UDSP_EC_NLMS_REF_LOSS_MAX);
	udsp_ec_nlms.destroy(ec);

	CASE("lec replay reader refuses a header loss over the reach");
	CHECK(header_loss_open(UDSP_EC_NLMS_REF_LOSS_MAX) == 0);
	CHECK(header_loss_open(UDSP_EC_NLMS_REF_LOSS_MAX + 1u) == -EINVAL);
}

/* The frames from a change to the first frame that the coherence run counts, on
 * a white far end. The first louder frame fires the detector, and the share of
 * a path 6 dB louder falls under its bar by the sixth held frame. */
#define COHERE_ONSET_FRAMES	10u
#define COHERE_STEP_DB		10.0
#define COHERE_AFTER_FRAMES	(4u * FRAMES_PER_SECOND)
#define COHERE_KEPT_FRAMES	(2u * FRAMES_PER_SECOND)
/* The ERLE blocks of the coherence cases, 100 ms, so a deadline reads the
 * output to a tenth of a second. */
#define COHERE_BLOCK_FRAMES	10u
/* The louder leg of cohere_shape_test runs past the latch and its trial. */
#define COHERE_LATCH_FRAMES	(8u * FRAMES_PER_SECOND)

/*
 * The readings of the path-change and coherence escapes, rebuilt from the
 * frames a case passes in and gets back, with the smoothing of dsp/lec.c. The
 * estimate is the microphone less the output, within half an LSB. The sums
 * start over at a trial edge as runs_clear does: after a start or a completed
 * end, before a talker end.
 */
struct cohere_view {
	double mic;
	double res;
	double cross;
	double res_power;
	double est_power;
	bool restart;
};

static void cohere_view_restart(struct cohere_view *v)
{
	v->cross = 0.0;
	v->res_power = 0.0;
	v->est_power = 0.0;
}

static void cohere_view_add(struct cohere_view *v, const int16_t *mic,
			    const int16_t *out, unsigned frame,
			    const struct udsp_ec_nlms_frame *reading)
{
	double mic_e = 0.0;
	double res_e = 0.0;
	double est_e = 0.0;
	double cross = 0.0;
	unsigned i;

	for (i = 0; i < frame; i++) {
		double est = (double)mic[i] - out[i];

		mic_e += (double)mic[i] * mic[i];
		res_e += (double)out[i] * out[i];
		est_e += est * est;
		cross += (double)out[i] * est;
	}
	v->mic += (mic_e / frame - v->mic) * COHERE_ALPHA;
	v->res += (res_e / frame - v->res) * COHERE_ALPHA;
	if (v->restart ||
	    reading->trial_end == UDSP_EC_NLMS_TRIAL_END_TALKER)
		cohere_view_restart(v);
	if (reading->ref_power >= reading->ref_bar) {
		v->cross += (cross / frame - v->cross) * COHERE_ALPHA;
		v->res_power += (res_e / frame - v->res_power) * COHERE_ALPHA;
		v->est_power += (est_e / frame - v->est_power) * COHERE_ALPHA;
	}
	v->restart = reading->trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE ||
		     (reading->trial_end != UDSP_EC_NLMS_TRIAL_END_NONE &&
		      reading->trial_end != UDSP_EC_NLMS_TRIAL_END_TALKER);
}

static double cohere_view_ratio_get(const struct cohere_view *v)
{
	return v->mic > 0.0 ? v->res / v->mic : NAN;
}

static double cohere_view_ncc_get(const struct cohere_view *v)
{
	if (!(v->res_power >= 1.0) || !(v->est_power >= 1.0))
		return NAN;
	return v->cross / sqrt(v->res_power * v->est_power);
}

/* The estimate energy over the microphone energy, the residual plus the
 * estimate. */
static double cohere_view_share_get(const struct cohere_view *v)
{
	double mic = v->res_power + v->est_power + 2.0 * v->cross;

	return mic > 0.0 ? v->est_power / mic : NAN;
}

/*
 * The change line of the coherence cases, the line of change_run: a white far
 * end at -10 dBm0, a floor at -70 dBm0 and the processor off. The late 28 ms
 * hybrid at 12 dB replaces a flat hybrid converged for 4 s. Its louder echo is
 * one that the estimate before the change does not predict.
 */
struct cohere_line {
	struct udsp_ec *ec;
	struct echo_path path;
	struct cohere_view view;
	struct ring_planes *planes;
	uint32_t state[3];
	unsigned long phase;
	unsigned rate;
	unsigned frame;
	bool css;
	bool quiet;
};

static void cohere_frame_run(struct cohere_line *l, const int16_t *near,
			     double *echo_energy, double *out_energy,
			     struct udsp_ec_nlms_frame *reading,
			     struct udsp_ec_nlms_info *info);

/*
 * The convergence frames feed the view, so it carries the history of the
 * canceller into the change. tail_ms 0 takes the default tail; css runs the
 * composite source; seed moves the far source; planes takes the planes of each
 * frame; quiet silences the far end.
 */
static void cohere_line_begin(struct cohere_line *l, unsigned rate,
			      const struct echo_path *path, unsigned tail_ms,
			      bool css, uint32_t seed)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	double echo_energy = 0.0;
	double res_energy = 0.0;
	unsigned f;

	memset(l, 0, sizeof(*l));
	l->rate = rate;
	l->frame = rate / 100u;
	l->css = css;
	l->state[0] = 0x13572468u ^ seed;
	l->state[1] = 0xfeedfaceu;
	l->state[2] = 0x2468acefu;
	l->ec = udsp_ec_nlms.create_rate(tail_ms, rate);
	udsp_ec_nlms.nlp(l->ec, false);
	l->path = *path;
	for (f = 0; f < CHANGE_CONVERGE_FRAMES; f++)
		cohere_frame_run(l, NULL, &echo_energy, &res_energy, &reading,
				 &info);
}

/* The flat 8 ms hybrid at from_erl_db. */
static void cohere_line_start(struct cohere_line *l, unsigned rate,
			      double from_erl_db, unsigned tail_ms, bool css)
{
	struct echo_path path;

	path_flat_init(&path, rate, 8, from_erl_db);
	cohere_line_begin(l, rate, &path, tail_ms, css, 0u);
}

static void cohere_line_init(struct cohere_line *l, unsigned rate,
			     double from_erl_db)
{
	cohere_line_start(l, rate, from_erl_db, 48, false);
}

/* The hybrid changes to the late one at to_erl_db, the far-end history
 * carried. */
static void cohere_line_change(struct cohere_line *l, double to_erl_db)
{
	struct echo_path late;

	path_late_init(&late, l->rate, to_erl_db);
	path_history_carry(&late, &l->path);
	l->path = late;
}

/* One frame of the change line, with near on top when it is given. echo and
 * out take the echo and the output energies of the frame. */
static void cohere_frame_run(struct cohere_line *l, const int16_t *near,
			     double *echo_energy, double *out_energy,
			     struct udsp_ec_nlms_frame *reading,
			     struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t talk[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
	unsigned i;

	if (l->css)
		css_fill(ref, l->frame, CHANGE_FAR_DBM0, &l->state[0],
			 &l->phase, l->rate);
	else
		noise_fill(ref, l->frame, CHANGE_FAR_DBM0, &l->state[0]);
	if (l->quiet)
		memset(ref, 0, l->frame * sizeof(*ref));
	path_run(&l->path, ref, echo);
	noise_fill(hiss, l->frame, CHANGE_FLOOR_DBM0, &l->state[2]);
	for (i = 0; i < l->frame; i++) {
		talk[i] = near ? near[i] : 0;
		mic[i] = sample_clamp((double)echo[i] + hiss[i] + talk[i]);
	}
	memcpy(out, mic, l->frame * sizeof(*mic));
	udsp_ec_nlms.run(l->ec, out, ref);
	udsp_ec_nlms_frame_get(l->ec, reading);
	udsp_ec_nlms_info_get(l->ec, info);
	cohere_view_add(&l->view, mic, out, l->frame, reading);
	if (l->planes) {
		struct ring_planes *p = l->planes;

		memcpy(p->ref, ref, l->frame * sizeof(*ref));
		memcpy(p->near, talk, l->frame * sizeof(*talk));
		memcpy(p->echo, echo, l->frame * sizeof(*echo));
		memcpy(p->hiss, hiss, l->frame * sizeof(*hiss));
		memcpy(p->mic, mic, l->frame * sizeof(*mic));
		memcpy(p->out, out, l->frame * sizeof(*out));
	}
	for (i = 0; i < l->frame; i++) {
		*echo_energy += (double)echo[i] * echo[i];
		*out_energy += (double)out[i] * out[i];
	}
}

/*
 * The figures of one louder change: the frames to the first trial and its arm,
 * the rebuilt readings before it, and the first settled block at 20 dB. Then
 * its end and kind, and the held frames and low blocks after a keep. Last, the
 * coherence trials, the active frames to the trial, and the largest voiced
 * reading.
 */
struct cohere_result {
	unsigned start;
	unsigned onset;
	unsigned arm;
	unsigned cohere;
	double ratio;
	double ncc;
	unsigned settled;
	unsigned end;
	unsigned end_kind;
	unsigned held;
	unsigned low;
	unsigned trials;
	unsigned resets;
	float gain_before;
	float gain_after;
	double burst_ncc;
};

/*
 * The runs of the path-change and coherence escapes, rebuilt from the view by
 * the rules of path_change_get and cohere_get. path_due and cohere_due hold the
 * first frame each run reaches its length. longest holds the longest coherence
 * run, and alone the run on the reading without the share.
 */
struct cohere_runs {
	unsigned path;
	unsigned cohere;
	unsigned path_due;
	unsigned cohere_due;
	unsigned longest;
	unsigned alone;
	unsigned alone_longest;
};

static void cohere_runs_add(struct cohere_runs *r, const struct cohere_view *v,
			    const struct udsp_ec_nlms_frame *reading,
			    bool held_before, unsigned f)
{
	double ncc = cohere_view_ncc_get(v);

	if (reading->probing &&
	    reading->trial_arm == UDSP_EC_NLMS_TRIAL_ARM_NONE) {
		r->path = 0;
		r->cohere = 0;
		r->alone = 0;
		return;
	}
	if (reading->ref_power < reading->ref_bar)
		return;
	if (!held_before) {
		r->path = 0;
		r->cohere = 0;
		r->alone = 0;
		return;
	}
	if (cohere_view_ratio_get(v) >= COHERE_PATH_RATIO)
		r->path++;
	else
		r->path = 0;
	if (reading->talker) {
		r->cohere = 0;
		r->alone = 0;
	} else if (!isnan(ncc)) {
		r->cohere = ncc <= COHERE_BAR &&
			    cohere_view_share_get(v) <= COHERE_SHARE_MAX ?
			    r->cohere + 1u : 0u;
		r->alone = ncc <= COHERE_BAR ? r->alone + 1u : 0u;
	}
	if (r->cohere > r->longest)
		r->longest = r->cohere;
	if (r->alone > r->alone_longest)
		r->alone_longest = r->alone;
	if (!r->path_due && r->path >= 20u)
		r->path_due = f;
	if (!r->cohere_due && r->cohere >= COHERE_FRAMES)
		r->cohere_due = f;
	if (reading->trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE ||
	    reading->trial_end != UDSP_EC_NLMS_TRIAL_END_NONE) {
		r->path = 0;
		r->cohere = 0;
		r->alone = 0;
	}
}

/* The echo energy under which a block of the change line reads no ERLE: a
 * pause of the composite source, after its echo dies away. */
#define COHERE_BLOCK_ECHO_DBM0	-40.0

/*
 * The change line. The flat hybrid step_db over the late one converges, and
 * to replaces it. When to is NULL, the late hybrid at 12 dB replaces it. css
 * runs the composite source at the default tail, and otherwise white noise at
 * the 48 ms tail.
 */
static void cohere_change_path_run(unsigned rate, double step_db, bool css,
				   const struct echo_path *to,
				   struct cohere_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_runs runs;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	double block_floor;
	uint32_t probes;
	bool held;
	unsigned f;

	memset(r, 0, sizeof(*r));
	memset(&runs, 0, sizeof(runs));
	r->ratio = NAN;
	r->ncc = NAN;
	r->burst_ncc = -2.0;
	cohere_line_start(&l, rate, CHANGE_ERL_DB + step_db, css ? 0u : 48u,
			  css);
	block_floor = (double)COHERE_BLOCK_FRAMES * l.frame *
		      pow(dbm0_rms_get(COHERE_BLOCK_ECHO_DBM0), 2.0);
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	held = info.double_talk;
	if (to) {
		struct echo_path next = *to;

		path_history_carry(&next, &l.path);
		l.path = next;
	} else {
		cohere_line_change(&l, CHANGE_ERL_DB);
	}
	for (f = 1; f <= COHERE_AFTER_FRAMES; f++) {
		double before_ratio = cohere_view_ratio_get(&l.view);
		double before_ncc = cohere_view_ncc_get(&l.view);
		bool voiced = css && l.phase % (l.rate / 2u) < l.rate / 20u;

		cohere_frame_run(&l, NULL, &echo_energy, &out_energy, &reading,
				 &info);
		cohere_runs_add(&runs, &l.view, &reading, held, f);
		held = info.double_talk;
		if (!r->start && voiced && runs.cohere &&
		    reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_NONE &&
		    cohere_view_ncc_get(&l.view) > r->burst_ncc)
			r->burst_ncc = cohere_view_ncc_get(&l.view);
		if (!r->start && reading.ref_power >= reading.ref_bar)
			r->onset++;
		if (!r->start && info.probes != probes) {
			r->start = f;
			r->arm = reading.trial_arm;
			r->ratio = before_ratio;
			r->ncc = before_ncc;
			r->gain_before = info.echo_gain;
		}
		if (r->start && !r->end &&
		    reading.trial_end != UDSP_EC_NLMS_TRIAL_END_NONE) {
			r->end = f;
			r->end_kind = reading.trial_end;
			r->gain_after = info.echo_gain;
		} else if (r->end_kind == UDSP_EC_NLMS_TRIAL_END_KEPT &&
			   f <= r->end + COHERE_KEPT_FRAMES) {
			r->held += info.double_talk;
		}
		if (f % COHERE_BLOCK_FRAMES)
			continue;
		if (echo_energy < block_floor) {
			echo_energy = out_energy = 0.0;
			continue;
		}
		if (!r->settled && out_energy > 0.0 &&
		    10.0 * log10(echo_energy / out_energy) >= CHANGE_BAR_DB)
			r->settled = f;
		if (r->end_kind == UDSP_EC_NLMS_TRIAL_END_KEPT &&
		    f > r->end + COHERE_BLOCK_FRAMES &&
		    f <= r->end + COHERE_KEPT_FRAMES && out_energy > 0.0 &&
		    10.0 * log10(echo_energy / out_energy) < CHANGE_BAR_DB)
			r->low++;
		echo_energy = out_energy = 0.0;
	}
	r->trials = info.probes - probes;
	r->resets = info.diverge_resets;
	udsp_ec_nlms.destroy(l.ec);
}

static void cohere_change_run(unsigned rate, double step_db, bool css,
			      struct cohere_result *r)
{
	cohere_change_path_run(rate, step_db, css, NULL, r);
}

static void cohere_change_print(unsigned rate, double step_db, bool css,
				const struct cohere_result *r)
{
	printf("# %u Hz, %s, %.0f dB louder: trial of arm %s on frame %u, "
	       "active frame %u; live residual over microphone %.3f, "
	       "coherence %.3f before it; %.0f dB of ERLE on frame %u; ended "
	       "%s on frame %u; after the keep %u double-talk frames and %u "
	       "blocks under %.0f dB in %u ms; gain %.4f before the trial, "
	       "%.4f after it; %u trials, %u resets\n", rate,
	       css ? "composite source" : "white noise", step_db,
	       lecr_trial_arm_name(r->arm), r->start, r->onset, r->ratio,
	       r->ncc, CHANGE_BAR_DB, r->settled,
	       lecr_trial_end_name(r->end_kind), r->end, r->held, r->low,
	       CHANGE_BAR_DB, 10u * COHERE_KEPT_FRAMES,
	       (double)r->gain_before, (double)r->gain_after, r->trials,
	       r->resets);
}

/* The deadline of the white-noise legs, 1.1 s: the share falls under its bar on
 * the sixth held frame, and the run of 32 counts from there. The deadline also
 * passes a trial on the plain step. */
#define COHERE_SETTLED_FRAMES	110u
/* The bounds of the composite-source legs from the change: the run of 32 plus
 * an onset that the level swings stretch, and 3 s to 20 dB. The trial counts
 * active frames, and the source spends its pauses. */
#define COHERE_CSS_ONSET_FRAMES	(COHERE_FRAMES + 32u)
#define COHERE_CSS_SETTLED_FRAMES	(3u * FRAMES_PER_SECOND)

/* The checks of a change that the coherence escape answers. The ratio check
 * of cohere_change_test is the caller's. */
static void cohere_change_check(const struct cohere_result *r, bool css)
{
	CHECK_UINT(r->arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(r->start > 0u);
	CHECK(r->ratio > 1.0);
	CHECK(r->ncc <= COHERE_BAR);
	CHECK(r->settled > 0u);
	CHECK_UINT(r->end_kind, UDSP_EC_NLMS_TRIAL_END_KEPT);
	CHECK_UINT(r->held, 0);
	CHECK_UINT(r->trials, 1);
	CHECK_UINT(r->resets, 0);
	if (css) {
		CHECK(r->onset <= COHERE_CSS_ONSET_FRAMES);
		CHECK(r->settled <= COHERE_CSS_SETTLED_FRAMES);
		return;
	}
	CHECK(r->start <= COHERE_FRAMES + COHERE_ONSET_FRAMES);
	CHECK(r->settled <= COHERE_SETTLED_FRAMES);
	CHECK_UINT(r->low, 0);
}

/*
 * A louder echo that the estimate before the change does not predict reads a
 * live ratio of about 1.10. On white noise its trial starts within 32 active
 * frames plus an onset. It reaches 20 dB within 1.1 s and stands with no hold
 * for 2 s. On the composite source it starts within 64 active frames and
 * settles within 3 s.
 */
static void cohere_change_test(unsigned rate)
{
	struct cohere_result r;
	struct cohere_result c;

	cohere_change_run(rate, COHERE_STEP_DB, false, &r);
	cohere_change_run(rate, COHERE_STEP_DB, true, &c);
	case_rate(rate, "lec coherence escape answers a louder echo the old "
		  "path does not predict");
	cohere_change_print(rate, COHERE_STEP_DB, false, &r);
	cohere_change_print(rate, COHERE_STEP_DB, true, &c);
	cohere_change_check(&r, false);
	CHECK(r.ratio < COHERE_PATH_RATIO);
	cohere_change_check(&c, true);
}

/*
 * The coherence escape on changes 6 dB and 12 dB louder. At 6 dB the live ratio
 * stands at the edge of the path-change bar, but a path-change run of 20 does
 * not complete first. The coherence trial starts on both steps within the
 * bounds of cohere_change_test.
 */
static void cohere_step_test(unsigned rate)
{
	static const double steps[] = { 6.0, 12.0 };
	unsigned i;
	unsigned k;

	case_rate(rate, "lec coherence escape answers a louder echo 6 and "
		  "12 dB louder");
	for (k = 0; k < 2u; k++)
		for (i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
			struct cohere_result r;

			cohere_change_run(rate, steps[i], k == 1u, &r);
			cohere_change_print(rate, steps[i], k == 1u, &r);
			cohere_change_check(&r, k == 1u);
		}
}

/*
 * The same hybrid with a new gain or polarity. The residual is then a copy of
 * the estimate. The coherence reads +1 for a louder echo, and -1 for a
 * quieter one or a flip. held counts the frames after the change that end in
 * double talk. The other fields follow struct cohere_result.
 */
static void cohere_shape_run(unsigned rate, double gain,
			     struct cohere_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	uint32_t probes;
	unsigned k;
	unsigned f;

	memset(r, 0, sizeof(*r));
	cohere_line_init(&l, rate, CHANGE_ERL_DB);
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	for (k = 0; k < l.path.taps; k++)
		l.path.h[k] = (float)(l.path.h[k] * gain);
	for (f = 1; f <= COHERE_LATCH_FRAMES; f++) {
		cohere_frame_run(&l, NULL, &echo_energy, &out_energy, &reading,
				 &info);
		r->held += info.double_talk;
		r->cohere += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE;
		if (!r->start && info.probes != probes) {
			r->start = f;
			r->arm = reading.trial_arm;
		}
		if (f % COHERE_BLOCK_FRAMES)
			continue;
		if (!r->settled && out_energy > 0.0 &&
		    10.0 * log10(echo_energy / out_energy) >= CHANGE_BAR_DB)
			r->settled = f;
		echo_energy = out_energy = 0.0;
	}
	r->trials = info.probes - probes;
	r->resets = info.diverge_resets;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * A hybrid of unchanged shape, 6 dB quieter or flipped: the detector holds
 * neither, no coherence trial starts, and plain adaptation reaches 20 dB within
 * 1 s. A step 6 dB louder reads +1 and starts none either; the latch ends that
 * hold within 8 s.
 */
static void cohere_shape_test(unsigned rate)
{
	static const struct {
		double gain;
		const char *what;
	} legs[] = {
		{ 0.5, "6 dB quieter" },
		{ -1.0, "flipped" },
		{ 2.0, "6 dB louder" },
	};
	unsigned i;

	case_rate(rate, "lec coherence escape leaves a hybrid of the same "
		  "shape to adaptation");
	for (i = 0; i < sizeof(legs) / sizeof(legs[0]); i++) {
		struct cohere_result r;

		cohere_shape_run(rate, legs[i].gain, &r);
		printf("# %u Hz, %s: %u double-talk frames in %u ms, %u "
		       "coherence trials, first trial %s on frame %u, %.0f dB "
		       "of ERLE on frame %u; %u trials, %u resets\n", rate,
		       legs[i].what, r.held, 10u * COHERE_LATCH_FRAMES,
		       r.cohere,
		       r.start ? lecr_trial_arm_name(r.arm) : "none", r.start,
		       CHANGE_BAR_DB, r.settled, r.trials, r.resets);
		CHECK_UINT(r.cohere, 0);
		CHECK_UINT(r.resets, 0);
		CHECK(r.settled > 0u);
		if (legs[i].gain > 1.0) {
			CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
			continue;
		}
		CHECK_UINT(r.held, 0);
		CHECK_UINT(r.trials, 0);
		CHECK(r.settled <= FRAMES_PER_SECOND);
	}
}

#define TIE_QUIETER_DB		2.0
#define TIE_FROM_DB		3.0
#define TIE_STEP_DB		0.1
#define TIE_STEPS		21u

/* The first trial after a change to the late hybrid, TIE_QUIETER_DB under the
 * flat one. A white talker talk_db over the echo before the change speaks from
 * the change on. runs takes the frames on which the rebuilt runs come due. */
static void cohere_tie_run(unsigned rate, double talk_db, unsigned *arm,
			   unsigned *start, struct cohere_runs *runs)
{
	uint32_t near_state = 0xfeedfaceu;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	uint32_t probes;
	bool held;
	unsigned f;

	memset(runs, 0, sizeof(*runs));
	*arm = UDSP_EC_NLMS_TRIAL_ARM_NONE;
	*start = 0;
	cohere_line_init(&l, rate, CHANGE_ERL_DB);
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	held = info.double_talk;
	cohere_line_change(&l, CHANGE_ERL_DB + TIE_QUIETER_DB);
	for (f = 1; f <= 2u * FRAMES_PER_SECOND && !*start; f++) {
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(near, l.frame, CHANGE_ECHO_DBM0 + talk_db,
			   &near_state);
		cohere_frame_run(&l, near, &echo_energy, &out_energy, &reading,
				 &info);
		cohere_runs_add(runs, &l.view, &reading, held, f);
		held = info.double_talk;
		if (info.probes == probes)
			continue;
		*arm = reading.trial_arm;
		*start = f;
	}
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * Precedence of the arms: a late hybrid 2 dB quieter under a white talker swept
 * from 3 to 5 dB over the echo before the change. The path-change run comes due
 * later as the talker grows. On a tie the path-change escape starts the trial,
 * otherwise the arm due first; each rate ties once at least.
 */
static void cohere_precedence_test(unsigned rate)
{
	unsigned ties = 0;
	unsigned i;

	case_rate(rate, "lec starts the path-change trial when both escapes "
		  "are due on one frame");
	for (i = 0; i < TIE_STEPS; i++) {
		double step = TIE_FROM_DB + TIE_STEP_DB * i;
		struct cohere_runs runs;
		unsigned arm;
		unsigned start;
		bool tie;

		cohere_tie_run(rate, step, &arm, &start, &runs);
		tie = start && runs.path_due == start &&
		      runs.cohere_due == start;
		printf("# %u Hz, talker %+.1f dB: trial %s on frame %u; path "
		       "run due on frame %u, coherence run on frame %u%s\n",
		       rate, step, start ? lecr_trial_arm_name(arm) : "none",
		       start,
		       runs.path_due, runs.cohere_due, tie ? ", a tie" : "");
		if (!start)
			continue;
		if (tie) {
			ties++;
			CHECK_UINT(arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
		} else if (runs.path_due == start) {
			CHECK_UINT(arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
		} else {
			CHECK_UINT(runs.cohere_due, start);
			CHECK_UINT(arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
		}
	}
	CHECK(ties > 0u);
}

/* The refusal line: the flat hybrid steps 6 dB quieter under a white talker 3
 * dB over the echo before the change. The talker refuses the coherence trial,
 * short of the 3 dB cut, and a twin frozen on it holds its snapshot. */
#define REFUSE_GAIN		0.5
#define REFUSE_TALK_DBM0	CARRY_LOUD_DBM0
#define REFUSE_FRAMES		(30u * FRAMES_PER_SECOND)
/* The bound in REFUSE_FRAMES: the 5 trials of the latch arm alone under a
 * talker who never pauses. One trial of each of the REFUSE_ESCAPES escapes
 * comes before the carry holds them. */
#define REFUSE_LATCH_TRIALS	5u
#define REFUSE_ESCAPES		3u

struct refuse_result {
	unsigned arms[MARK_TRIALS_MAX];
	unsigned starts[MARK_TRIALS_MAX];
	unsigned trials;
	unsigned cohere;
	unsigned first_end;
	unsigned gap_min;
	unsigned resets;
	unsigned differ;
	float lead_energy;
	float twin_energy;
};

/* stop ends the run on the frame the first trial ends, for the twin
 * compare; otherwise the run takes REFUSE_FRAMES frames. */
static void refuse_run(unsigned rate, bool stop, struct refuse_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info twin_info;
	struct cohere_line lead;
	struct cohere_line twin;
	uint32_t near_state = 0xfeedfaceu;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	uint32_t probes;
	unsigned last = 0;
	unsigned frozen = 0;
	unsigned k;
	unsigned f;

	memset(r, 0, sizeof(*r));
	cohere_line_init(&lead, rate, CHANGE_ERL_DB);
	cohere_line_init(&twin, rate, CHANGE_ERL_DB);
	udsp_ec_nlms_info_get(lead.ec, &info);
	probes = info.probes;
	for (k = 0; k < lead.path.taps; k++) {
		lead.path.h[k] = (float)(lead.path.h[k] * REFUSE_GAIN);
		twin.path.h[k] = (float)(twin.path.h[k] * REFUSE_GAIN);
	}
	for (f = 1; f <= REFUSE_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(near, lead.frame, REFUSE_TALK_DBM0, &near_state);
		cohere_frame_run(&lead, near, &echo_energy, &out_energy,
				 &reading, &info);
		cohere_frame_run(&twin, near, &echo_energy, &out_energy,
				 &reading, &twin_info);
		udsp_ec_nlms_frame_get(lead.ec, &reading);
		if (r->trials == 1u && !r->first_end &&
		    reading.trial_end != UDSP_EC_NLMS_TRIAL_END_NONE) {
			r->first_end = reading.trial_end;
			if (stop)
				break;
		}
		if (info.probes == probes)
			continue;
		probes = info.probes;
		if (r->trials < MARK_TRIALS_MAX) {
			r->arms[r->trials] = reading.trial_arm;
			r->starts[r->trials] = f;
		}
		if (r->trials && (!r->gap_min || f - last < r->gap_min))
			r->gap_min = f - last;
		r->cohere += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE;
		last = f;
		if (!r->trials++) {
			udsp_ec_nlms.freeze(twin.ec, true);
			frozen = f;
		}
	}
	r->resets = info.diverge_resets;
	/* Both filters frozen, the far end alone: two frozen filters give one
	 * output only when they hold the same coefficients. */
	udsp_ec_nlms.freeze(lead.ec, true);
	for (f = 0; f < TWIN_COMPARE_FRAMES && frozen; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t lead_out[UDSP_FRAME_MAX];
		int16_t twin_out[UDSP_FRAME_MAX];

		noise_fill(ref, lead.frame, CHANGE_FAR_DBM0, &lead.state[0]);
		path_run(&lead.path, ref, echo);
		memcpy(lead_out, echo, lead.frame * sizeof(*echo));
		memcpy(twin_out, echo, lead.frame * sizeof(*echo));
		udsp_ec_nlms.run(lead.ec, lead_out, ref);
		udsp_ec_nlms.run(twin.ec, twin_out, ref);
		r->differ += memcmp(lead_out, twin_out,
				    lead.frame * sizeof(*echo)) != 0;
	}
	udsp_ec_nlms_info_get(lead.ec, &info);
	udsp_ec_nlms_info_get(twin.ec, &twin_info);
	r->lead_energy = info.coef_energy;
	r->twin_energy = twin_info.coef_energy;
	udsp_ec_nlms.destroy(lead.ec);
	udsp_ec_nlms.destroy(twin.ec);
}

static void refuse_print(unsigned rate, const struct refuse_result *r)
{
	unsigned i;

	printf("# %u Hz: %u trials in 30 s, arms", rate, r->trials);
	for (i = 0; i < r->trials && i < MARK_TRIALS_MAX; i++)
		printf(" %s@%u", lecr_trial_arm_name(r->arms[i]), r->starts[i]);
	printf("; first ends %s; shortest gap %u frames; filter %.9f against "
	       "the twin's %.9f, %u of %u frames differ; %u resets\n",
	       lecr_trial_end_name(r->first_end), r->gap_min,
	       (double)r->lead_energy, (double)r->twin_energy, r->differ,
	       TWIN_COMPARE_FRAMES, r->resets);
}

/*
 * A refused coherence trial restores its snapshot and stands its escape down: a
 * twin frozen on its start and the lead give equal output after the refusal.
 * Over 30 s more of talker the escape starts no second trial, as the latch and
 * stale-run trials inside it carry its stand-down.
 */
static void cohere_refuse_test(unsigned rate)
{
	struct refuse_result first;
	struct refuse_result r;

	refuse_run(rate, true, &first);
	refuse_run(rate, false, &r);
	case_rate(rate, "lec coherence escape stands down after a trial a "
		  "talker refused");
	refuse_print(rate, &first);
	refuse_print(rate, &r);
	CHECK_UINT(first.trials, 1);
	CHECK_UINT(first.arms[0], UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK_UINT(first.first_end, UDSP_EC_NLMS_TRIAL_END_REJECTED);
	CHECK(first.lead_energy == first.twin_energy);
	CHECK_UINT(first.differ, 0);
	CHECK(r.trials >= 2u);
	CHECK_UINT(r.arms[0], UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK_UINT(r.cohere, 1);
	CHECK_UINT(r.resets, 0);
}

/*
 * Trials of a refusal line in 30 s: one per escape at most before the carry
 * holds them all, then one per latch cycle. That gives 5 trials of the latch
 * arm plus 3 at most while the latch comes due in each cycle. cohere_cycle_test
 * covers a line where it does not.
 */
static void cohere_bound_test(unsigned rate)
{
	struct refuse_result r;
	unsigned per_arm[UDSP_EC_NLMS_TRIAL_ARM_COHERE + 1u];
	unsigned i;

	memset(per_arm, 0, sizeof(per_arm));
	refuse_run(rate, false, &r);
	for (i = 0; i < r.trials && i < MARK_TRIALS_MAX; i++)
		if (r.arms[i] <= UDSP_EC_NLMS_TRIAL_ARM_COHERE)
			per_arm[r.arms[i]]++;
	case_rate(rate, "lec bounds the trials of three escapes and the latch "
		  "in 30 s");
	printf("# %u Hz: %u trials, latch %u, path %u, stale %u, coherence "
	       "%u; bound %u, the latch arm alone %u\n", rate, r.trials,
	       per_arm[UDSP_EC_NLMS_TRIAL_ARM_LATCH],
	       per_arm[UDSP_EC_NLMS_TRIAL_ARM_PATH],
	       per_arm[UDSP_EC_NLMS_TRIAL_ARM_STALE],
	       per_arm[UDSP_EC_NLMS_TRIAL_ARM_COHERE],
	       REFUSE_LATCH_TRIALS + REFUSE_ESCAPES, REFUSE_LATCH_TRIALS);
	CHECK(r.trials <= REFUSE_LATCH_TRIALS + REFUSE_ESCAPES);
	CHECK(per_arm[UDSP_EC_NLMS_TRIAL_ARM_PATH] <= 1u);
	CHECK(per_arm[UDSP_EC_NLMS_TRIAL_ARM_STALE] <= 1u);
	CHECK(per_arm[UDSP_EC_NLMS_TRIAL_ARM_COHERE] <= 1u);
	CHECK_UINT(r.resets, 0);
}

/* The share line: the flat hybrid steps 3 dB quieter under a white talker at
 * the level of the echo before the change. The reading stands under the bar
 * while the estimate holds over half of the microphone energy. */
#define SHARE_GAIN		0.7
#define SHARE_TALK_DBM0		CHANGE_ECHO_DBM0

/*
 * Estimate share in the coherence escape. On a share line the detector holds
 * every frame, and its reading alone runs past a run of 32. Its share stands
 * over 0.5, so the escape must start no trial, and each hold stays with the
 * latch. A talker refuses any trial of a hybrid that turns quieter under it,
 * so the share spares its line that trial.
 */
static void cohere_share_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_runs runs;
	struct cohere_line l;
	uint32_t near_state = 0xfeedfaceu;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned cohere = 0;
	unsigned latch = 0;
	bool held;
	unsigned k;
	unsigned f;

	memset(&runs, 0, sizeof(runs));
	cohere_line_init(&l, rate, CHANGE_ERL_DB);
	for (k = 0; k < l.path.taps; k++)
		l.path.h[k] = (float)(l.path.h[k] * SHARE_GAIN);
	udsp_ec_nlms_info_get(l.ec, &info);
	held = info.double_talk;
	for (f = 1; f <= REFUSE_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(near, l.frame, SHARE_TALK_DBM0, &near_state);
		cohere_frame_run(&l, near, &echo_energy, &out_energy, &reading,
				 &info);
		cohere_runs_add(&runs, &l.view, &reading, held, f);
		held = info.double_talk;
		cohere += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE;
		latch += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_LATCH;
	}
	case_rate(rate, "lec coherence escape leaves a quieter hybrid under a "
		  "talker to the latch");
	printf("# %u Hz: %u held frames of %u; longest run on the reading "
	       "alone %u, with the share %u; %u coherence trials, %u latch "
	       "trials, %u trials in all\n", rate, info.hold_frames,
	       REFUSE_FRAMES, runs.alone_longest, runs.longest, cohere, latch,
	       info.probes);
	CHECK(runs.alone_longest >= COHERE_FRAMES);
	CHECK(runs.longest < COHERE_FRAMES);
	CHECK_UINT(cohere, 0);
	CHECK(latch > 0u);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(l.ec);
}

/* The partial line: a path 8.5 dB louder that the first path predicts on voiced
 * bursts, not on noise. It is twice the first under PARTIAL_CUT_HZ plus a
 * dispersive and a late hybrid (Annex D m6, m7). */
#define PARTIAL_GAIN		2.0
#define PARTIAL_CUT_HZ		450.0
#define PARTIAL_DISP_ERL_DB	14.0
#define PARTIAL_LATE_ERL_DB	24.0
/* Under this bar the latch, not the coherence escape, answers the partial
 * line. */
#define PARTIAL_OLD_BAR		-0.15

static void partial_path_add(struct echo_path *p, const struct echo_path *q)
{
	unsigned k;

	for (k = 0; k < q->taps; k++)
		p->h[k] += q->h[k];
	if (q->taps > p->taps)
		p->taps = q->taps;
}

static void partial_path_init(struct echo_path *p, const struct echo_path *old,
			      unsigned rate)
{
	int half = (int)path_samples_get(1.0, rate);
	struct echo_path disp;
	struct echo_path late;
	unsigned k;
	int j;

	path_geometry_set(p, rate);
	for (k = 0; k < old->taps; k++) {
		if (old->h[k] == 0.0f)
			continue;
		for (j = -half; j <= half; j++) {
			double w = 0.5 + 0.5 * cos(M_PI * j / (half + 1));
			double s = j ? sin(2.0 * M_PI * PARTIAL_CUT_HZ * j /
					   rate) / (M_PI * j) :
				   2.0 * PARTIAL_CUT_HZ / rate;

			if ((int)k + j >= 0)
				p->h[(int)k + j] += (float)(PARTIAL_GAIN *
							    old->h[k] * s * w);
		}
		if (k + (unsigned)half + 1u > p->taps)
			p->taps = k + (unsigned)half + 1u;
	}
	path_dispersive_init(&disp, rate, PARTIAL_DISP_ERL_DB);
	path_late_init(&late, rate, PARTIAL_LATE_ERL_DB);
	partial_path_add(p, &disp);
	partial_path_add(p, &late);
}

static double partial_gain_db_get(const struct echo_path *p,
				  const struct echo_path *q)
{
	double ep = 0.0;
	double eq = 0.0;
	unsigned k;

	for (k = 0; k < LEC_PATH_TAPS_MAX; k++) {
		ep += (double)p->h[k] * p->h[k];
		eq += (double)q->h[k] * q->h[k];
	}
	return 10.0 * log10(ep / eq);
}

/*
 * On a partial line the smoothed reading climbs back toward 0 on each voiced
 * burst, so a run of 32 must survive a burst. The trial starts within 64 active
 * frames, stands with no hold and reaches 20 dB within 3 s. Its burst readings
 * stand over -0.15, where the latch would answer.
 */
static void cohere_partial_test(unsigned rate)
{
	struct cohere_result r;
	struct echo_path old;
	struct echo_path to;

	path_flat_init(&old, rate, 8, CHANGE_ERL_DB + COHERE_STEP_DB);
	partial_path_init(&to, &old, rate);
	cohere_change_path_run(rate, COHERE_STEP_DB, true, &to, &r);
	case_rate(rate, "lec coherence escape answers a new path the old one "
		  "partly predicts");
	printf("# %u Hz, new path %.1f dB over the old: trial of arm %s on "
	       "frame %u, active frame %u; reading %.3f before it, %.3f at "
	       "most on the voiced bursts of the run; %.0f dB of ERLE on frame "
	       "%u; ended %s on frame %u; %u double-talk frames after the "
	       "keep; %u trials, %u resets\n", rate,
	       partial_gain_db_get(&to, &old), lecr_trial_arm_name(r.arm),
	       r.start, r.onset, r.ncc, r.burst_ncc, CHANGE_BAR_DB, r.settled,
	       lecr_trial_end_name(r.end_kind), r.end, r.held, r.trials,
	       r.resets);
	CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(r.start > 0u);
	CHECK(r.onset <= COHERE_CSS_ONSET_FRAMES);
	CHECK_UINT(r.end_kind, UDSP_EC_NLMS_TRIAL_END_KEPT);
	CHECK_UINT(r.held, 0);
	CHECK(r.settled > 0u);
	CHECK(r.settled <= COHERE_CSS_SETTLED_FRAMES);
	CHECK_UINT(r.trials, 1);
	CHECK_UINT(r.resets, 0);
	CHECK(r.burst_ncc > PARTIAL_OLD_BAR);
	CHECK(r.burst_ncc <= COHERE_BAR);
}

/* The reseed line: the change line on the composite source at a 31 ms tail,
 * with a late hybrid to 40 ms. The kept filter then stands under the trust line
 * and under-reads the microphone on some frames. */
#define GAIN_RESEED_TAIL_MS	31u
#define GAIN_RESEED_AFTER_FRAMES	(10u * FRAMES_PER_SECOND)
#define GAIN_RESEED_FRAMES	(4u * FRAMES_PER_SECOND)

/* kept, held and trials count the keep, the held frames and the trials after
 * it. */
struct reseed_result {
	unsigned arm;
	unsigned kept;
	unsigned held;
	unsigned trials;
	float erle_db;
	float gain_before;
	float gain_after;
};

static void reseed_run(unsigned rate, struct reseed_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	uint32_t probes;
	unsigned f;

	memset(r, 0, sizeof(*r));
	cohere_line_start(&l, rate, CHANGE_ERL_DB + COHERE_STEP_DB,
			  GAIN_RESEED_TAIL_MS, true);
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	cohere_line_change(&l, CHANGE_ERL_DB);
	for (f = 1; f <= GAIN_RESEED_AFTER_FRAMES; f++) {
		cohere_frame_run(&l, NULL, &echo_energy, &out_energy, &reading,
				 &info);
		if (!r->arm && info.probes != probes) {
			r->arm = reading.trial_arm;
			probes = info.probes;
			r->gain_before = info.echo_gain;
			continue;
		}
		if (r->arm && !r->kept &&
		    reading.trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT) {
			r->kept = f;
			r->erle_db = info.erle_db;
			r->gain_after = info.echo_gain;
			probes = info.probes;
			continue;
		}
		if (!r->kept || f > r->kept + GAIN_RESEED_FRAMES)
			continue;
		r->held += info.double_talk;
		r->trials += info.probes != probes;
		probes = info.probes;
	}
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * After a kept escape trial, the learned gain takes its echo evidence
 * (echo_gain_reseed): the coherence trial stands under the trust line. The 4 s
 * after the keep hold no double-talk frame and start no trial. The gain of the
 * quieter path would leave the classic bound under the echo.
 */
static void cohere_reseed_test(unsigned rate)
{
	struct reseed_result r;

	reseed_run(rate, &r);
	case_rate(rate, "lec reseeds the learned gain after a kept trial");
	printf("# %u Hz: trial %s, kept on frame %u at %.2f dB of ERLE; gain "
	       "%.4f before, %.4f after; %u double-talk frames and %u trials "
	       "in the %u ms after the keep\n", rate,
	       lecr_trial_arm_name(r.arm), r.kept, (double)r.erle_db,
	       (double)r.gain_before, (double)r.gain_after, r.held, r.trials,
	       10u * GAIN_RESEED_FRAMES);
	CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(r.kept > 0u);
	CHECK(r.erle_db < RING_TRUST_DB);
	CHECK(r.gain_after > r.gain_before);
	CHECK_UINT(r.held, 0);
	CHECK_UINT(r.trials, 0);
}

/*
 * The exposure rows of cohere_expose_test on the ringback line of the expose
 * cases, with the processor off and on. Each runs 3 s of white noise, then
 * EXPOSE_ROW_FRAMES of each row; near levels are in dB over TALKER_ECHO_DBM0.
 */
#define EXPOSE_ROW_FRAMES	(20u * FRAMES_PER_SECOND)
#define EXPOSE_TONE_HZ		1000.0
/* A copy of the far end delayed past the default tails, 64 ms at 16 kHz. */
#define EXPOSE_PAST_MS		80u
/* The coupling of the far end into the near microphone: 2 ms, inside the
 * tail and 6 ms ahead of the hybrid. */
#define EXPOSE_COUPLED_MS	2u
/* The echo delay of the ringback line. At that delay a copy turned over
 * stands at 3 or 12 times the echo. The microphone then holds 2 or 11 times
 * the echo, with its sign turned. */
#define EXPOSE_ECHO_MS		8u
#define EXPOSE_INVERT_LOW_DB	9.54
#define EXPOSE_INVERT_HIGH_DB	21.58
#define EXPOSE_HIST		(2u * UDSP_SAMPLE_RATE_WB / 10u)

enum expose_src {
	EXPOSE_WHITE,
	EXPOSE_WORDS,
	EXPOSE_CSS,
	EXPOSE_TONE,
	EXPOSE_COPY,
};

/*
 * One row: far source, near source and level, the delay of a copy, the phase of
 * a near tone in quarter cycles, and invert. An independent row starts no
 * trial. A coherent row carries what the reference carries, so its trials
 * cancel it with the echo, at least by cut_db, keeping at most kept trials.
 */
struct expose_row {
	const char *what;
	enum expose_src far;
	enum expose_src near;
	double near_db;
	unsigned delay_ms;
	unsigned tone_phase;
	bool independent;
	bool invert;
	double cut_db;
	unsigned kept;
};

/* LEC_GEIGEL_HANGOVER_SHORT of dsp/lec.c. */
#define EXPOSE_HOLD_SHORT_FRAMES	4u

/* The trials of one instance of a row by arm, with the exposure figures of
 * its trial frames. hold_pass counts the first pass frames after a short
 * hold of the detector inside a trial. hold_acted counts those of them on
 * which the processor acts (expose_hold_step). */
struct expose_count {
	unsigned arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE + 1u];
	unsigned kept;
	unsigned cohere_gap_min;
	unsigned cohere_at;
	unsigned active;
	unsigned hold_pass;
	unsigned hold_acted;
	bool hold_fired;
	bool hold_short;
	bool hold_running;
	struct expose_stat st;
};

struct expose_src_state {
	struct word_source words;
	uint32_t state;
	unsigned long phase;
};

static void expose_src_fill(struct expose_src_state *s, enum expose_src kind,
			    double level_dbm0, int16_t *dst, unsigned frame,
			    unsigned rate)
{
	switch (kind) {
	case EXPOSE_WORDS:
		word_fill(&s->words, dst, frame);
		return;
	case EXPOSE_CSS:
		css_fill(dst, frame, level_dbm0, &s->state, &s->phase, rate);
		return;
	case EXPOSE_TONE:
		tone_fill(dst, frame, level_dbm0, EXPOSE_TONE_HZ, &s->phase,
			  rate);
		return;
	default:
		noise_fill(dst, frame, level_dbm0, &s->state);
		return;
	}
}

static void expose_count_step(struct expose_count *c,
			      const struct udsp_ec_nlms_frame *reading,
			      const int16_t *ref, unsigned frame)
{
	if (frame_loud(ref, frame))
		c->active++;
	if (reading->trial_arm <= UDSP_EC_NLMS_TRIAL_ARM_COHERE)
		c->arms[reading->trial_arm]++;
	if (reading->trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT)
		c->kept++;
	if (reading->trial_arm != UDSP_EC_NLMS_TRIAL_ARM_COHERE)
		return;
	if (c->arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE] > 1u &&
	    (!c->cohere_gap_min ||
	     c->active - c->cohere_at < c->cohere_gap_min))
		c->cohere_gap_min = c->active - c->cohere_at;
	c->cohere_at = c->active;
}

/* Counts the first pass frame after a short hold of the detector inside a
 * trial. hold_left reads the hangover that the verdict of the frame before
 * set. */
static void expose_hold_step(struct expose_count *c,
			     const struct udsp_ec_nlms_frame *reading,
			     const struct udsp_ec_nlms_info *info)
{
	if (c->hold_fired)
		c->hold_short = reading->hold_left == EXPOSE_HOLD_SHORT_FRAMES;
	c->hold_fired = reading->dt_fired;
	if (reading->dt_fired) {
		c->hold_running = true;
		return;
	}
	if (!c->hold_running || reading->nlp_verdict == LECR_VERDICT_DT)
		return;
	c->hold_running = false;
	if (c->hold_short && reading->probing &&
	    reading->nlp_verdict == LECR_VERDICT_PASS) {
		c->hold_pass++;
		c->hold_acted += info->nlp_active;
	}
}

static void expose_row_run(const struct expose_row *row, unsigned rate,
			   struct expose_count *off, struct expose_count *on)
{
	static int16_t hist[EXPOSE_HIST];
	struct ring_line lines[2];
	struct expose_src_state far;
	struct expose_src_state near;
	unsigned frame = rate / 100u;
	unsigned delay = row->delay_ms * rate / 1000u;
	double gain = pow(10.0, (row->near_db - RING_ERL_DB) / 20.0);
	unsigned pos = 0;
	unsigned i;
	unsigned f;

	memset(off, 0, sizeof(*off));
	memset(on, 0, sizeof(*on));
	memset(hist, 0, sizeof(hist));
	memset(&far, 0, sizeof(far));
	memset(&near, 0, sizeof(near));
	far.state = 0x0badc0deu;
	near.state = 0x31415926u;
	near.phase = row->near == EXPOSE_CSS ? rate / 4u :
		     (unsigned long)lrint(row->tone_phase * (double)rate /
					  (4.0 * EXPOSE_TONE_HZ));
	word_source_init(&far.words, rate, RING_WORDS_DBM0, 0x13572468u);
	word_source_init(&near.words, rate, TALKER_ECHO_DBM0 + row->near_db,
			 0x0badcafeu);
	for (i = 0; i < 2u; i++) {
		ring_line_init(&lines[i], rate);
		udsp_ec_nlms.nlp(lines[i].ec, i == 1u);
		ring_white_run(&lines[i], TALKER_CONVERGE_FRAMES);
	}
	for (f = 0; f < EXPOSE_ROW_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t talk[UDSP_FRAME_MAX];

		expose_src_fill(&far, row->far, RING_WORDS_DBM0, ref, frame,
				rate);
		if (row->near == EXPOSE_COPY) {
			for (i = 0; i < frame; i++) {
				hist[pos % EXPOSE_HIST] = ref[i];
				talk[i] = sample_clamp((row->invert ? -gain :
							gain) *
					hist[(pos + EXPOSE_HIST - delay) %
					     EXPOSE_HIST]);
				pos++;
			}
		} else {
			expose_src_fill(&near, row->near,
					TALKER_ECHO_DBM0 + row->near_db, talk,
					frame, rate);
		}
		for (i = 0; i < 2u; i++) {
			struct expose_count *c = i ? on : off;
			struct udsp_ec_nlms_frame reading;
			struct udsp_ec_nlms_info info;

			expose_frame_run(&lines[i], ref, talk, &c->st);
			udsp_ec_nlms_frame_get(lines[i].ec, &reading);
			udsp_ec_nlms_info_get(lines[i].ec, &info);
			expose_count_step(c, &reading, ref, frame);
			expose_hold_step(c, &reading, &info);
		}
	}
	for (i = 0; i < 2u; i++)
		udsp_ec_nlms.destroy(lines[i].ec);
}

static void expose_row_print(const struct expose_row *row, unsigned rate,
			     const struct expose_count *c, bool nlp)
{
	printf("# %u Hz, %s, processor %s: trials latch %u, path %u, stale "
	       "%u, coherence %u; %u kept; shortest gap between coherence "
	       "trials %u active frames; ", rate, row->what, nlp ? "on" : "off",
	       c->arms[UDSP_EC_NLMS_TRIAL_ARM_LATCH],
	       c->arms[UDSP_EC_NLMS_TRIAL_ARM_PATH],
	       c->arms[UDSP_EC_NLMS_TRIAL_ARM_STALE],
	       c->arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE], c->kept,
	       c->cohere_gap_min);
	if (c->st.trials)
		printf("in the trials: near %.2f dB through, output %.2f dB "
		       "against the microphone, echo %.2f dB under its level\n",
		       expose_near_db_get(&c->st),
		       expose_db_get(c->st.out, c->st.mic),
		       expose_db_get(c->st.echo, c->st.leak));
	else
		printf("no trial frames\n");
}

/* The benign sweep: the composite source over the composite hybrid at 12 dB
 * with a word talker over the echo, EXPOSE_SWEEP_SEEDS seeds of 30 s. Returns
 * the coherence trials; longest takes the longest rebuilt run. */
#define EXPOSE_SWEEP_SEEDS	8u
#define EXPOSE_SWEEP_FRAMES	(30u * FRAMES_PER_SECOND)
#define EXPOSE_SWEEP_MARGIN	10u

static unsigned expose_sweep_run(unsigned rate, double over_db, uint32_t seed,
				 unsigned *longest)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct cohere_runs runs;
	struct echo_path path;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned cohere = 0;
	bool held;
	unsigned f;

	memset(&runs, 0, sizeof(runs));
	path_composite_init(&path, rate, CHANGE_ERL_DB);
	cohere_line_begin(&l, rate, &path, 0u, true, seed);
	word_source_init(&talker, rate, CHANGE_ECHO_DBM0 + over_db,
			 0x0badcafeu + seed * 0x9e3779b9u);
	udsp_ec_nlms_info_get(l.ec, &info);
	held = info.double_talk;
	for (f = 1; f <= EXPOSE_SWEEP_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];

		word_fill(&talker, near, l.frame);
		cohere_frame_run(&l, near, &echo_energy, &out_energy, &reading,
				 &info);
		cohere_runs_add(&runs, &l.view, &reading, held, f);
		held = info.double_talk;
		cohere += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE;
	}
	*longest = runs.longest;
	udsp_ec_nlms.destroy(l.ec);
	return cohere;
}

/* The checks of a coherent row with trial frames: the output at most
 * TALK_BURST_DB over the microphone, the cancellation bound and the keeps. */
static void expose_row_check(const struct expose_row *row,
			     const struct expose_count *c)
{
	CHECK(!c->cohere_gap_min || c->cohere_gap_min >= EXPOSE_ANY_CADENCE);
	CHECK(c->kept <= row->kept);
	if (!c->st.trials)
		return;
	CHECK(expose_db_get(c->st.out, c->st.mic) <= TALK_BURST_DB);
	CHECK(expose_db_get(c->st.out, c->st.mic) <= -row->cut_db);
}

/*
 * Exposure of the coherence escape: independent rows and copies past the tail
 * start no trial. Coherent rows keep their cancellation bound and keeps, with
 * output at most 3 dB over the microphone. On the benign sweep no trial starts,
 * and the longest run stands 10 frames or more under 32.
 */
static void cohere_expose_test(unsigned rate)
{
	static const struct expose_row rows[] = {
		{ "css over css -4 dB", EXPOSE_CSS, EXPOSE_CSS, -4.0, 0, 0,
		  true, false, 0.0, 0 },
		{ "css over css 0 dB", EXPOSE_CSS, EXPOSE_CSS, 0.0, 0, 0,
		  true, false, 0.0, 0 },
		{ "css over css +6 dB", EXPOSE_CSS, EXPOSE_CSS, 6.0, 0, 0,
		  true, false, 0.0, 0 },
		{ "css over css +12 dB", EXPOSE_CSS, EXPOSE_CSS, 12.0, 0, 0,
		  true, false, 0.0, 0 },
		{ "words over words -4 dB", EXPOSE_WORDS, EXPOSE_WORDS, -4.0,
		  0, 0, true, false, 0.0, 0 },
		{ "words over words 0 dB", EXPOSE_WORDS, EXPOSE_WORDS, 0.0,
		  0, 0, true, false, 0.0, 0 },
		{ "words over words +6 dB", EXPOSE_WORDS, EXPOSE_WORDS, 6.0,
		  0, 0, true, false, 0.0, 0 },
		{ "words over words +12 dB", EXPOSE_WORDS, EXPOSE_WORDS, 12.0,
		  0, 0, true, false, 0.0, 0 },
		{ "white over white -4 dB", EXPOSE_WHITE, EXPOSE_WHITE, -4.0,
		  0, 0, true, false, 0.0, 0 },
		{ "white over white 0 dB", EXPOSE_WHITE, EXPOSE_WHITE, 0.0,
		  0, 0, true, false, 0.0, 0 },
		{ "white over white +6 dB", EXPOSE_WHITE, EXPOSE_WHITE, 6.0,
		  0, 0, true, false, 0.0, 0 },
		{ "white over white +12 dB", EXPOSE_WHITE, EXPOSE_WHITE, 12.0,
		  0, 0, true, false, 0.0, 0 },
		{ "words copy past the tail 0 dB", EXPOSE_WORDS, EXPOSE_COPY,
		  0.0, EXPOSE_PAST_MS, 0, true, false, 0.0, 0 },
		{ "white copy past the tail +6 dB", EXPOSE_WHITE, EXPOSE_COPY,
		  6.0, EXPOSE_PAST_MS, 0, true, false, 0.0, 0 },
		{ "tone over tone in phase +6 dB", EXPOSE_TONE, EXPOSE_TONE,
		  6.0, 0, 0, false, false, 20.0, 1 },
		{ "tone over tone a quarter cycle +6 dB", EXPOSE_TONE,
		  EXPOSE_TONE, 6.0, 0, 1, false, false, 18.0, 1 },
		{ "tone over tone half a cycle +6 dB", EXPOSE_TONE, EXPOSE_TONE,
		  6.0, 0, 2, false, false, 12.0, 1 },
		{ "css coupled -20 dB", EXPOSE_CSS, EXPOSE_COPY, -20.0,
		  EXPOSE_COUPLED_MS, 0, false, false, 0.0, 0 },
		{ "css coupled -10 dB", EXPOSE_CSS, EXPOSE_COPY, -10.0,
		  EXPOSE_COUPLED_MS, 0, false, false, 0.0, 0 },
		{ "css coupled 0 dB", EXPOSE_CSS, EXPOSE_COPY, 0.0,
		  EXPOSE_COUPLED_MS, 0, false, false, 0.0, 0 },
		{ "white coupled 0 dB", EXPOSE_WHITE, EXPOSE_COPY, 0.0,
		  EXPOSE_COUPLED_MS, 0, false, false, 10.0, 1 },
		{ "white copy at the echo delay turned over, 3 times the echo",
		  EXPOSE_WHITE, EXPOSE_COPY, EXPOSE_INVERT_LOW_DB,
		  EXPOSE_ECHO_MS, 0, false, true, 4.0, 1 },
		{ "white copy at the echo delay turned over, 12 times the echo",
		  EXPOSE_WHITE, EXPOSE_COPY, EXPOSE_INVERT_HIGH_DB,
		  EXPOSE_ECHO_MS, 0, false, true, 6.0, 1 },
	};
	unsigned i;
	unsigned k;

	case_rate(rate, "lec coherence escape starts no trial for an "
		  "independent near signal");
	for (i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
		struct expose_count off;
		struct expose_count on;

		expose_row_run(&rows[i], rate, &off, &on);
		expose_row_print(&rows[i], rate, &off, false);
		expose_row_print(&rows[i], rate, &on, true);
		if (rows[i].independent) {
			CHECK_UINT(off.arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE], 0);
			CHECK_UINT(on.arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE], 0);
			continue;
		}
		expose_row_check(&rows[i], &off);
		expose_row_check(&rows[i], &on);
	}
	for (k = 0; k < 3u; k++) {
		double over = 4.0 + 4.0 * k;
		unsigned worst = 0;
		unsigned cohere = 0;
		uint32_t s;

		for (s = 0; s < EXPOSE_SWEEP_SEEDS; s++) {
			unsigned longest;

			cohere += expose_sweep_run(rate, over, s, &longest);
			if (longest > worst)
				worst = longest;
		}
		printf("# %u Hz, composite source under words %+.0f dB over "
		       "the composite hybrid at 12 dB, %u seeds of 30 s: %u "
		       "coherence trials, longest run %u active frames "
		       "against the run of %u\n", rate, over,
		       EXPOSE_SWEEP_SEEDS, cohere, worst, COHERE_FRAMES);
		CHECK_UINT(cohere, 0);
		CHECK(worst + EXPOSE_SWEEP_MARGIN <= COHERE_FRAMES);
	}
}

/*
 * A talker end of a coherence trial: on trial frame 10 a white talker 16 dB
 * over the far end joins for one frame. That frame reads a talker, a running
 * trial and a talker end, counts one held frame and leaves the twin's filter.
 * The escape it stands down starts no trial for LEC_TRIAL_STANDDOWN_FRAMES.
 */
static void cohere_talker_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_info twin_info;
	struct cohere_line lead;
	struct cohere_line twin;
	uint32_t near_state = 0xfeedfaceu;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned arm = UDSP_EC_NLMS_TRIAL_ARM_NONE;
	unsigned started = 0;
	unsigned cohere = 0;
	unsigned f;

	cohere_line_init(&lead, rate, CHANGE_ERL_DB + COHERE_STEP_DB);
	cohere_line_init(&twin, rate, CHANGE_ERL_DB + COHERE_STEP_DB);
	cohere_line_change(&lead, CHANGE_ERL_DB);
	cohere_line_change(&twin, CHANGE_ERL_DB);
	for (f = 1; f <= COHERE_AFTER_FRAMES && !started; f++) {
		cohere_frame_run(&lead, NULL, &echo_energy, &out_energy,
				 &reading, &info);
		cohere_frame_run(&twin, NULL, &echo_energy, &out_energy,
				 &reading, &twin_info);
		if (!info.probes)
			continue;
		started = f;
		udsp_ec_nlms_frame_get(lead.ec, &reading);
		arm = reading.trial_arm;
	}
	udsp_ec_nlms.freeze(twin.ec, true);
	for (f = 1; f < TALKER_TRIAL_FRAME; f++)
		cohere_frame_run(&lead, NULL, &echo_energy, &out_energy,
				 &reading, &info);
	before = info;
	{
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(near, lead.frame, CHANGE_FAR_DBM0 + TALKER_OVER_DB,
			   &near_state);
		cohere_frame_run(&lead, near, &echo_energy, &out_energy,
				 &reading, &info);
	}
	udsp_ec_nlms_info_get(twin.ec, &twin_info);
	case_rate(rate, "lec ends a coherence trial on the first frame of a "
		  "near talker");
	printf("# %u Hz: trial of arm %s on frame %u after the change; talker "
	       "frame: talker %u, probing %u, end %s, held %u; filter %.9f "
	       "against the twin's %.9f\n", rate, lecr_trial_arm_name(arm),
	       started, reading.talker,
	       reading.probing, lecr_trial_end_name(reading.trial_end),
	       info.hold_frames - before.hold_frames,
	       (double)info.coef_energy, (double)twin_info.coef_energy);
	CHECK(started > 0u);
	CHECK_UINT(arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(reading.talker);
	CHECK(reading.probing);
	CHECK_UINT(reading.trial_end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK_UINT(info.hold_frames - before.hold_frames, 1);
	CHECK(info.coef_energy == twin_info.coef_energy);
	for (f = 0; f < EXPOSE_ANY_CADENCE - 10u; f++) {
		cohere_frame_run(&lead, NULL, &echo_energy, &out_energy,
				 &reading, &info);
		cohere += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE;
	}
	printf("# %u Hz: %u coherence trials in the %u frames after the "
	       "talker end, %u trials in all\n", rate, cohere,
	       EXPOSE_ANY_CADENCE - 10u, info.probes);
	CHECK_UINT(cohere, 0);
	CHECK_UINT(info.diverge_resets, 0);
	udsp_ec_nlms.destroy(lead.ec);
	udsp_ec_nlms.destroy(twin.ec);
}

/* The frame of the change line on which a far-end pause starts, inside the
 * run of the coherence escape. The two pause lengths end inside the long
 * hangover of LEC_GEIGEL_HANGOVER and past it. */
#define PAUSE_RUN_AT		15u
#define PAUSE_SHORT_FRAMES	10u
#define PAUSE_LONG_FRAMES	40u

/* The first trial on the change line 10 dB louder with a far-end pause of
 * pause frames from frame PAUSE_RUN_AT on. */
static unsigned cohere_pause_run(unsigned rate, unsigned pause,
				 unsigned *arm)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned start = 0;
	unsigned f;

	cohere_line_init(&l, rate, CHANGE_ERL_DB + COHERE_STEP_DB);
	cohere_line_change(&l, CHANGE_ERL_DB);
	for (f = 1; f <= COHERE_AFTER_FRAMES && !start; f++) {
		if (f >= PAUSE_RUN_AT && f < PAUSE_RUN_AT + pause) {
			int16_t ref[UDSP_FRAME_MAX];
			int16_t echo[UDSP_FRAME_MAX];
			int16_t hiss[UDSP_FRAME_MAX];
			unsigned i;

			memset(ref, 0, sizeof(ref));
			path_run(&l.path, ref, echo);
			noise_fill(hiss, l.frame, CHANGE_FLOOR_DBM0,
				   &l.state[2]);
			for (i = 0; i < l.frame; i++)
				hiss[i] = sample_clamp((double)hiss[i] +
						       echo[i]);
			udsp_ec_nlms.run(l.ec, hiss, ref);
			udsp_ec_nlms_info_get(l.ec, &info);
		} else {
			cohere_frame_run(&l, NULL, &echo_energy, &out_energy,
					 &reading, &info);
		}
		if (!info.probes)
			continue;
		udsp_ec_nlms_frame_get(l.ec, &reading);
		start = f;
		*arm = reading.trial_arm;
	}
	udsp_ec_nlms.destroy(l.ec);
	return start;
}

/*
 * A far pause during a coherence run: a pause of 10 frames ends inside the long
 * hangover. Its frames neither arm nor disarm the run, and the trial starts one
 * pause later. A pause of 40 frames ends the hold, so the run starts over. The
 * trial then comes within a run plus an onset of the return.
 */
static void cohere_pause_test(unsigned rate)
{
	unsigned arm_none = 0;
	unsigned arm_short = 0;
	unsigned arm_long = 0;
	unsigned none = cohere_pause_run(rate, 0u, &arm_none);
	unsigned brief = cohere_pause_run(rate, PAUSE_SHORT_FRAMES, &arm_short);
	unsigned lasting = cohere_pause_run(rate, PAUSE_LONG_FRAMES, &arm_long);
	unsigned back = PAUSE_RUN_AT + PAUSE_LONG_FRAMES;

	case_rate(rate, "lec coherence run waits through a far-end pause");
	printf("# %u Hz: trial on frame %u (%s) with no pause, %u (%s) with a "
	       "pause of %u frames, %u (%s) with a pause of %u frames\n", rate,
	       none, lecr_trial_arm_name(arm_none), brief,
	       lecr_trial_arm_name(arm_short), PAUSE_SHORT_FRAMES, lasting,
	       lecr_trial_arm_name(arm_long), PAUSE_LONG_FRAMES);
	CHECK_UINT(arm_none, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK_UINT(arm_short, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK_UINT(arm_long, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK_UINT(brief, none + PAUSE_SHORT_FRAMES);
	CHECK(lasting >= back + COHERE_FRAMES);
	CHECK(lasting <= back + COHERE_FRAMES + COHERE_ONSET_FRAMES);
}

/*
 * The coherence trials of one line, with their exposure figures: the trials,
 * the keeps and the end of the last one. st takes the frames and energies of
 * their frames (struct expose_stat), the start frame and the end frame
 * included. held takes the same energies over the held frames before the
 * first trial.
 */
struct cohere_trials {
	struct expose_stat st;
	struct expose_stat held;
	unsigned trials;
	unsigned kept;
	unsigned end;
	unsigned frames;
	bool open;
};

static void cohere_trials_step(struct cohere_trials *c,
			       const struct udsp_ec_nlms_frame *reading,
			       const struct ring_planes *p, unsigned frame)
{
	if (reading->trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE) {
		c->trials++;
		c->open = true;
	}
	if (c->open) {
		expose_energy_add(&c->st, p->near, p, frame);
		c->frames++;
	} else if (!c->trials && reading->ref_power >= reading->ref_bar &&
		   reading->dt_fired) {
		expose_energy_add(&c->held, p->near, p, frame);
	}
	if (!c->open || reading->trial_end == UDSP_EC_NLMS_TRIAL_END_NONE)
		return;
	c->open = false;
	c->end = reading->trial_end;
	c->kept += reading->trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT;
}

static void cohere_trials_print(unsigned rate, const char *what,
				const struct cohere_trials *c)
{
	printf("# %u Hz, %s: %u coherence trials, %u kept, the last ended "
	       "%s after %u frames in all; in the trials: near %.2f dB "
	       "through, output %.2f dB against the microphone, echo %.2f dB "
	       "under its level\n", rate, what, c->trials, c->kept,
	       lecr_trial_end_name(c->end), c->frames,
	       expose_near_db_get(&c->st),
	       expose_db_get(c->st.out, c->st.mic),
	       expose_db_get(c->st.echo, c->st.leak));
}

/* The refusal line for REFUSE_FRAMES, its coherence trials into c. */
static void near_trial_refuse_run(unsigned rate, struct cohere_trials *c)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_planes planes;
	struct cohere_line l;
	uint32_t near_state = 0xfeedfaceu;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned k;
	unsigned f;

	memset(c, 0, sizeof(*c));
	cohere_line_init(&l, rate, CHANGE_ERL_DB);
	l.planes = &planes;
	for (k = 0; k < l.path.taps; k++)
		l.path.h[k] = (float)(l.path.h[k] * REFUSE_GAIN);
	for (f = 1; f <= REFUSE_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(near, l.frame, REFUSE_TALK_DBM0, &near_state);
		cohere_frame_run(&l, near, &echo_energy, &out_energy, &reading,
				 &info);
		cohere_trials_step(c, &reading, &planes, l.frame);
	}
	udsp_ec_nlms.destroy(l.ec);
}

/* The talker of the sweep over a change: after convergence on the composite
 * source, the late hybrid 10 dB louder, under a word talker over_db over the
 * old echo. */
#define NEAR_WORDS_FRAMES	(10u * FRAMES_PER_SECOND)

static void near_trial_words_run(unsigned rate, double over_db,
				 struct cohere_trials *c)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct ring_planes planes;
	struct echo_path path;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned f;

	memset(c, 0, sizeof(*c));
	path_composite_init(&path, rate, CHANGE_ERL_DB);
	cohere_line_begin(&l, rate, &path, 0u, true, 0u);
	l.planes = &planes;
	word_source_init(&talker, rate, CHANGE_ECHO_DBM0 + over_db,
			 0x0badcafeu);
	cohere_line_change(&l, CHANGE_ERL_DB - COHERE_STEP_DB);
	for (f = 1; f <= NEAR_WORDS_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];

		word_fill(&talker, near, l.frame);
		cohere_frame_run(&l, near, &echo_energy, &out_energy, &reading,
				 &info);
		cohere_trials_step(c, &reading, &planes, l.frame);
	}
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * Near speech through coherence trials: on every trial frame it passes within 2
 * dB, the output at most 3 dB over the microphone. A refusal line's trial ends
 * refused. Under words 6 dB under a changed echo 10 dB louder, a trial stays
 * kept with the echo 3 dB down; at 2 dB under it, the talker refuses each.
 */
static void cohere_near_trial_test(unsigned rate)
{
	struct cohere_trials refuse;
	struct cohere_trials words;
	struct cohere_trials louder;

	near_trial_refuse_run(rate, &refuse);
	near_trial_words_run(rate, 4.0, &words);
	near_trial_words_run(rate, 8.0, &louder);
	case_rate(rate, "lec coherence trials pass the near speech of a "
		  "talker");
	cohere_trials_print(rate, "refusal line", &refuse);
	cohere_trials_print(rate, "words +4 dB over a change", &words);
	cohere_trials_print(rate, "words +8 dB over a change", &louder);
	printf("# %u Hz, refusal line: held frames before the trial: echo "
	       "%.2f dB under its level\n", rate,
	       expose_db_get(refuse.held.echo, refuse.held.leak));
	CHECK(refuse.trials >= 1u);
	CHECK_UINT(refuse.kept, 0);
	CHECK(fabs(expose_near_db_get(&refuse.st)) <= EXPOSE_NEAR_DB);
	CHECK(expose_db_get(refuse.st.out, refuse.st.mic) <= TALK_BURST_DB);
	CHECK(expose_db_get(refuse.st.echo, refuse.st.leak) >=
	      expose_db_get(refuse.held.echo, refuse.held.leak) - 1.0);
	CHECK(words.trials >= 1u);
	CHECK_UINT(words.kept, words.trials);
	CHECK(fabs(expose_near_db_get(&words.st)) <= EXPOSE_NEAR_DB);
	CHECK(expose_db_get(words.st.out, words.st.mic) <= TALK_BURST_DB);
	if (words.kept)
		CHECK(expose_db_get(words.st.echo, words.st.leak) >=
		      EXPOSE_LEAK_DB);
	CHECK(louder.trials >= 1u);
	CHECK_UINT(louder.kept, 0);
	CHECK(fabs(expose_near_db_get(&louder.st)) <= EXPOSE_NEAR_DB);
	CHECK(expose_db_get(louder.st.out, louder.st.mic) <= TALK_BURST_DB);
}

/* The intermittent line: the refusal line whose talker and far end pause
 * together. The pause frames hold no double talk and no reference, so they pay
 * the latch run down without adapting. */
#define STANDDOWN_CYCLE_FRAMES	400u
#define STANDDOWN_PAUSE_FRAMES	100u

struct standdown_cycle {
	unsigned arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE + 1u];
	unsigned at[UDSP_EC_NLMS_TRIAL_ARM_COHERE + 1u];
	unsigned gap_min[UDSP_EC_NLMS_TRIAL_ARM_COHERE + 1u];
	unsigned trials;
	unsigned active;
	unsigned resets;
};

static void standdown_cycle_run(unsigned rate, struct standdown_cycle *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	uint32_t near_state = 0xfeedfaceu;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned k;
	unsigned f;

	memset(r, 0, sizeof(*r));
	cohere_line_init(&l, rate, CHANGE_ERL_DB);
	for (k = 0; k < l.path.taps; k++)
		l.path.h[k] = (float)(l.path.h[k] * REFUSE_GAIN);
	for (f = 0; f < REFUSE_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];
		unsigned arm;

		l.quiet = f % STANDDOWN_CYCLE_FRAMES >=
			  STANDDOWN_CYCLE_FRAMES - STANDDOWN_PAUSE_FRAMES;
		noise_fill(near, l.frame, REFUSE_TALK_DBM0, &near_state);
		if (l.quiet)
			memset(near, 0, l.frame * sizeof(*near));
		cohere_frame_run(&l, near, &echo_energy, &out_energy, &reading,
				 &info);
		if (reading.ref_power >= reading.ref_bar)
			r->active++;
		arm = reading.trial_arm;
		if (arm == UDSP_EC_NLMS_TRIAL_ARM_NONE ||
		    arm > UDSP_EC_NLMS_TRIAL_ARM_COHERE)
			continue;
		r->trials++;
		if (r->arms[arm] &&
		    (!r->gap_min[arm] || r->active - r->at[arm] <
		     r->gap_min[arm]))
			r->gap_min[arm] = r->active - r->at[arm];
		r->arms[arm]++;
		r->at[arm] = r->active;
	}
	r->resets = info.diverge_resets;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * A bound per stand-down cycle: pauses keep the latch from coming due, so no
 * latch trial carries a stand-down. Each escape starts one trial at most per
 * cycle of 700 active frames. The coherence escape comes back, two trials or
 * more in 30 s, past the bound of cohere_bound_test.
 */
static void cohere_cycle_test(unsigned rate)
{
	struct standdown_cycle r;
	unsigned arm;

	standdown_cycle_run(rate, &r);
	case_rate(rate, "lec coherence escape starts one trial per stand-down "
		  "cycle under an intermittent talker");
	printf("# %u Hz: %u trials over %u active frames: latch %u, path %u, "
	       "stale %u, coherence %u; shortest gaps between two trials of "
	       "one arm: latch %u, path %u, stale %u, coherence %u; %u "
	       "resets\n", rate, r.trials, r.active,
	       r.arms[UDSP_EC_NLMS_TRIAL_ARM_LATCH],
	       r.arms[UDSP_EC_NLMS_TRIAL_ARM_PATH],
	       r.arms[UDSP_EC_NLMS_TRIAL_ARM_STALE],
	       r.arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE],
	       r.gap_min[UDSP_EC_NLMS_TRIAL_ARM_LATCH],
	       r.gap_min[UDSP_EC_NLMS_TRIAL_ARM_PATH],
	       r.gap_min[UDSP_EC_NLMS_TRIAL_ARM_STALE],
	       r.gap_min[UDSP_EC_NLMS_TRIAL_ARM_COHERE], r.resets);
	CHECK(r.arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE] >= 2u);
	for (arm = UDSP_EC_NLMS_TRIAL_ARM_PATH;
	     arm <= UDSP_EC_NLMS_TRIAL_ARM_COHERE; arm++)
		CHECK(!r.gap_min[arm] || r.gap_min[arm] >= EXPOSE_CADENCE);
	CHECK(r.arms[UDSP_EC_NLMS_TRIAL_ARM_COHERE] <=
	      r.active / EXPOSE_CADENCE + 1u);
	CHECK_UINT(r.resets, 0);
}

/* The divergence line of divergence_peak_test: the flat hybrid at 12 dB grows
 * louder in 0.5 dB steps until the watchdog zeroes the filter. A white talker
 * DIVERGE_TALK_DB over the echo then joins. */
#define DIVERGE_TALK_DB		3.0
#define DIVERGE_TALK_FRAMES	(3u * FRAMES_PER_SECOND)

/*
 * Coherence sums at a divergence reset. A watchdog reset throws its filter
 * away, so sums that read a diverged filter describe a filter that this line
 * does not have. They start over with their run. A talker holds the detector
 * after that reset, and the escape must start no trial in its 3 s of speech.
 */
static void cohere_diverge_test(void)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create(16);
	uint32_t state = 0x0c0ffee2u;
	uint32_t near_state = 0xfeedfaceu;
	double erl_db = 12.0;
	unsigned dwell = 0;
	unsigned cohere = 0;
	unsigned held = 0;
	unsigned f;

	udsp_ec_nlms.nlp(ec, false);
	path_flat_init(&path, UDSP_SAMPLE_RATE, 8, erl_db);
	(void)erle_run(ec, &path, &state, -10.0, FRAMES_PER_SECOND, 1);
	udsp_ec_nlms_info_get(ec, &before);
	for (f = 0; f < 10u * FRAMES_PER_SECOND; f++) {
		(void)erle_run(ec, &path, &state, -10.0, 1u, 1u);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.diverge_resets != before.diverge_resets)
			break;
		if (++dwell < RAMP_DWELL_FRAMES ||
		    info.erle_db < RAMP_SETTLED_DB)
			continue;
		erl_db -= RAMP_STEP_DB;
		path_erl_set(&path, erl_db);
		dwell = 0;
	}
	before = info;
	for (f = 0; f < DIVERGE_TALK_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		unsigned i;

		noise_fill(ref, path.frame, -10.0, &state);
		path_run(&path, ref, echo);
		noise_fill(near, path.frame, -10.0 - erl_db + DIVERGE_TALK_DB,
			   &near_state);
		for (i = 0; i < path.frame; i++)
			echo[i] = sample_clamp((double)echo[i] + near[i]);
		udsp_ec_nlms.run(ec, echo, ref);
		udsp_ec_nlms_frame_get(ec, &reading);
		udsp_ec_nlms_info_get(ec, &info);
		cohere += reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE;
		held += info.double_talk;
	}

	CASE("lec coherence escape starts no trial from the sums of a "
	     "diverged filter");
	printf("# reset at ERL %.1f dB, %u resets; in %u frames of the "
	       "talker: %u double-talk frames, %u coherence trials, %u "
	       "trials\n", erl_db, before.diverge_resets, DIVERGE_TALK_FRAMES,
	       held, cohere, info.probes - before.probes);
	CHECK(before.diverge_resets > 0u);
	CHECK(held > 0u);
	CHECK_UINT(cohere, 0);
	udsp_ec_nlms.destroy(ec);
}

/* The gain line of cohere_gain_test: the ringback line converged on white
 * noise, then a far tone with a near tone at its frequency. White noise then
 * runs under a white talker GAIN_TALK_DB over the echo for GAIN_TALK_FRAMES. */
#define GAIN_TONE_FRAMES	(20u * FRAMES_PER_SECOND)
#define GAIN_TALK_DB		6.0
#define GAIN_TALK_FRAMES	(3u * FRAMES_PER_SECOND)
/* Three bounds: LEC_GAIN_RESEED_COEF_RATIO of dsp/lec.c with the rounding of a
 * float product. GAIN_RISE_TRIAL is the rise of LEC_GEIGEL_GAIN_RISE over a
 * second of a trial, GAIN_TALK_SPREAD the spread of a reseed under a talker. */
#define GAIN_RESEED_COEF_RATIO	5.0001
#define GAIN_RISE_TRIAL		1.585f
#define GAIN_TALK_SPREAD	1.259f

struct gain_result {
	unsigned arm;
	unsigned kept;
	unsigned held;
	float gain_before;
	float gain_after;
	float coef_energy;
	struct expose_stat st;
};

/*
 * A near tone tone_db over the echo, quarters of a cycle from it, until the
 * first trial ends kept. In phase at 6 dB the latch starts that trial. Half a
 * cycle off, at 6 and 14 dB, the path-change escape does. The estimate that it
 * learns stands 1 and 16 times the converged filter's echo. At 14 dB the
 * reseed's cap holds the trust's energy rule.
 */
static void gain_run(unsigned rate, unsigned quarters, double tone_db,
		     struct gain_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct ring_planes p;
	struct ring_line line;
	unsigned long far_phase = 0;
	unsigned long near_phase = (unsigned long)lrint(quarters *
		(double)rate / (4.0 * EXPOSE_TONE_HZ));
	uint32_t talk_state = 0x31415926u;
	float gain = 0.0f;
	unsigned f;

	memset(r, 0, sizeof(*r));
	ring_line_init(&line, rate);
	ring_white_run(&line, TALKER_CONVERGE_FRAMES);
	for (f = 1; f <= GAIN_TONE_FRAMES && !r->kept; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];

		udsp_ec_nlms_info_get(line.ec, &info);
		gain = info.echo_gain;
		tone_fill(ref, line.frame, RING_WORDS_DBM0, EXPOSE_TONE_HZ,
			  &far_phase, rate);
		tone_fill(near, line.frame, TALKER_ECHO_DBM0 + tone_db,
			  EXPOSE_TONE_HZ, &near_phase, rate);
		ring_planes_run(&line, ref, near, NULL, &p);
		udsp_ec_nlms_frame_get(line.ec, &reading);
		udsp_ec_nlms_info_get(line.ec, &info);
		if (reading.trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE) {
			r->arm = reading.trial_arm;
			r->gain_before = gain;
		}
		if (reading.trial_end != UDSP_EC_NLMS_TRIAL_END_KEPT)
			continue;
		r->kept = f;
		r->gain_after = info.echo_gain;
		r->coef_energy = info.coef_energy;
	}
	for (f = 0; f < GAIN_TALK_FRAMES && r->kept; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(ref, line.frame, RING_WORDS_DBM0, &line.far_state);
		noise_fill(near, line.frame, TALKER_ECHO_DBM0 + GAIN_TALK_DB,
			   &talk_state);
		ring_planes_run(&line, ref, near, NULL, &p);
		udsp_ec_nlms_info_get(line.ec, &info);
		r->held += info.double_talk;
		expose_energy_add(&r->st, near, &p, line.frame);
	}
	udsp_ec_nlms.destroy(line.ec);
}

/* The change line 10 dB louder on white noise, with a white talker
 * GAIN_CHANGE_TALK_DB under the changed echo when talker is set. It returns the
 * learned gain where the first trial ends kept, else 0, and its arm. */
#define GAIN_CHANGE_TALK_DB	-3.0

static float gain_change_run(unsigned rate, bool talker, unsigned *arm)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	uint32_t near_state = 0x31415926u;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	float gain = 0.0f;
	unsigned f;

	*arm = UDSP_EC_NLMS_TRIAL_ARM_NONE;
	cohere_line_init(&l, rate, CHANGE_ERL_DB + COHERE_STEP_DB);
	cohere_line_change(&l, CHANGE_ERL_DB);
	for (f = 1; f <= COHERE_AFTER_FRAMES && !gain; f++) {
		int16_t near[UDSP_FRAME_MAX];

		noise_fill(near, l.frame,
			   CHANGE_ECHO_DBM0 + GAIN_CHANGE_TALK_DB, &near_state);
		cohere_frame_run(&l, talker ? near : NULL, &echo_energy,
				 &out_energy, &reading, &info);
		if (reading.trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE)
			*arm = reading.trial_arm;
		if (reading.trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT)
			gain = info.echo_gain;
	}
	udsp_ec_nlms.destroy(l.ec);
	return gain;
}

/*
 * A latch trial kept on a near tone in phase leaves the gain to the tracker,
 * which rises 2 dB at most in a second. After each keep the gain stands at most
 * LEC_GAIN_RESEED_COEF_RATIO times coefficient energy, and the detector holds a
 * white talker. A reseed under a talker stands within 1 dB of one.
 */
static void cohere_gain_test(unsigned rate)
{
	static const struct {
		unsigned quarters;
		double tone_db;
		const char *what;
	} legs[] = {
		{ 0, 6.0, "in phase, 6 dB" },
		{ 2, 6.0, "half a cycle, 6 dB" },
		{ 2, 14.0, "half a cycle, 14 dB" },
	};
	unsigned clean_arm;
	unsigned talk_arm;
	float clean;
	float talk;
	unsigned i;

	case_rate(rate, "lec reseeds the learned gain from echo evidence "
		  "alone");
	for (i = 0; i < sizeof(legs) / sizeof(legs[0]); i++) {
		struct gain_result r;

		gain_run(rate, legs[i].quarters, legs[i].tone_db, &r);
		printf("# %u Hz, near tone %s: trial %s kept on frame %u; gain "
		       "%.6f before, %.6f after, coefficient energy %.6f; "
		       "talker held on %u of %u frames, near %.2f dB through, "
		       "output %.2f dB against the microphone, echo %.2f dB "
		       "under its level\n", rate, legs[i].what,
		       lecr_trial_arm_name(r.arm), r.kept,
		       (double)r.gain_before, (double)r.gain_after,
		       (double)r.coef_energy, r.held, GAIN_TALK_FRAMES,
		       expose_near_db_get(&r.st),
		       expose_db_get(r.st.out, r.st.mic),
		       expose_db_get(r.st.echo, r.st.leak));
		CHECK(r.kept > 0u);
		CHECK((double)r.gain_after <=
		      GAIN_RESEED_COEF_RATIO * r.coef_energy);
		CHECK(10u * r.held >= 9u * GAIN_TALK_FRAMES);
		CHECK(fabs(expose_near_db_get(&r.st)) <= EXPOSE_NEAR_DB);
		if (r.arm == UDSP_EC_NLMS_TRIAL_ARM_LATCH)
			CHECK(r.gain_after <= r.gain_before * GAIN_RISE_TRIAL);
		else
			CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
	}
	clean = gain_change_run(rate, false, &clean_arm);
	talk = gain_change_run(rate, true, &talk_arm);
	printf("# %u Hz, change 10 dB louder: gain %.6f after the %s trial "
	       "with no talker, %.6f after the %s trial under a talker %.0f dB "
	       "under the new echo, %+.2f dB\n", rate, (double)clean,
	       lecr_trial_arm_name(clean_arm), (double)talk,
	       lecr_trial_arm_name(talk_arm), -GAIN_CHANGE_TALK_DB,
	       clean > 0.0f && talk > 0.0f ?
	       10.0 * log10((double)talk / clean) : 0.0);
	CHECK_UINT(talk_arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(clean > 0.0f);
	CHECK(talk > 0.0f);
	CHECK(talk <= clean * GAIN_TALK_SPREAD);
	CHECK(talk >= clean / GAIN_TALK_SPREAD);
}

/* The creep line: the floor measured at CREEP_FLOOR_DBM0 rises CREEP_RISE_DB.
 * On given frames its readings dip to CREEP_DIP_DB over the tracked floor, so
 * each dip creeps the floor. */
#define CREEP_FLOOR_DBM0	-60.0
#define CREEP_RISE_DB		10.0
#define CREEP_DIP_DB		1.5
#define CREEP_MEASURE_FRAMES	100u
#define CREEP_RISE_FRAMES	120u
#define CREEP_RISE_TOL_DB	1.5
#define CREEP_HOLD_DB		1.0

static double creep_floor_dbm0_get(const struct udsp_ec_nlms_info *info)
{
	return rms_dbm0_get(32767.0 * pow(10.0, -info->noise_dbov / 20.0));
}

/* A floor rise, with a dip on each frame f of the rise for which f % period
 * is in dips[0..ndips). It returns the loudest tracked floor of the rise, in
 * dBov. Reason: once the run completes, a dip pulls the floor down at
 * LEC_NOISE_ALPHA_DOWN, the rule that keeps the floor under its mean. */
static float creep_rise_run(unsigned rate, unsigned period,
			    const unsigned *dips, unsigned ndips,
			    float *before)
{
	struct udsp_ec_nlms_info info;
	uint32_t state = 0x9e3779b9u;
	int16_t ref[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned frame = rate / 100u;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	float loudest;
	double dip_dbm0;
	unsigned f;
	unsigned i;

	memset(ref, 0, sizeof(ref));
	for (f = 0; f < CREEP_MEASURE_FRAMES; f++) {
		noise_fill(mic, frame, CREEP_FLOOR_DBM0, &state);
		udsp_ec_nlms.run(ec, mic, ref);
	}
	udsp_ec_nlms_info_get(ec, &info);
	*before = info.noise_dbov;
	loudest = info.noise_dbov;
	dip_dbm0 = creep_floor_dbm0_get(&info) + CREEP_DIP_DB;
	for (f = 0; f < CREEP_RISE_FRAMES; f++) {
		bool dip = false;

		for (i = 0; i < ndips; i++)
			dip |= f % period == dips[i];
		noise_fill(mic, frame, dip ? dip_dbm0 :
			   CREEP_FLOOR_DBM0 + CREEP_RISE_DB, &state);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.noise_dbov < loudest)
			loudest = info.noise_dbov;
	}
	udsp_ec_nlms.destroy(ec);
	return loudest;
}

/*
 * A floor rises 10 dB with dips within 3 dB of the floor before the rise. One
 * dip in 20 lets the rise run complete; two in 16 exceed 1 in 16 of its
 * readings, so the run clears and the floor stays. A run that clears on each
 * dip, or keeps any number, fails.
 */
static void noise_creep_test(unsigned rate)
{
	static const unsigned one[] = { 19u };
	static const unsigned two[] = { 7u, 15u };
	double target = floor_dbov_get(CREEP_FLOOR_DBM0 + CREEP_RISE_DB);
	float before_one;
	float before_two;
	float rose = creep_rise_run(rate, 20u, one, 1u, &before_one);
	float held = creep_rise_run(rate, 16u, two, 2u, &before_two);

	case_rate(rate, "lec rise run of the floor survives one reading near "
		  "the old floor in 20 and clears on two in 16");
	printf("# floor measured at %.2f dBov; a floor %.0f dB louder (%.2f "
	       "dBov) with a dip once in 20 frames: loudest tracked %.2f dBov "
	       "in %u frames; with two dips in 16: %.2f dBov (measured "
	       "%.2f)\n",
	       (double)before_one, CREEP_RISE_DB, target, (double)rose,
	       CREEP_RISE_FRAMES, (double)held, (double)before_two);
	CHECK(fabs(rose - target) <= CREEP_RISE_TOL_DB);
	CHECK(held >= before_two - CREEP_HOLD_DB);
}

/* The late line: the composite hybrid at 12 dB plus a copy of the reference
 * LATE_MS late, past the tail, at late_db under the echo (0: off). The ERLE
 * then holds near the processor's gate. */
#define LATE_MS			70u
#define LATE_HIST_MAX		(LATE_MS * UDSP_SAMPLE_RATE_WB / 1000u + \
				 UDSP_FRAME_MAX)

struct late_line {
	struct udsp_ec *ec;
	struct echo_path path;
	int16_t hist[LATE_HIST_MAX];
	uint32_t hiss;
	unsigned rate;
	unsigned frame;
	unsigned delay;
	double late_db;
};

static void late_line_init(struct late_line *l, unsigned rate,
			   unsigned tail_ms)
{
	memset(l, 0, sizeof(*l));
	l->rate = rate;
	l->frame = rate / 100u;
	l->delay = LATE_MS * rate / 1000u;
	l->hiss = 0x2468acefu;
	path_composite_init(&l->path, rate, CHANGE_ERL_DB);
	l->ec = udsp_ec_nlms.create_rate(tail_ms, rate);
	udsp_ec_nlms.nlp(l->ec, true);
}

/* One frame of the late line with ref at the far end and near on top when
 * it is given. info takes the counts after the frame. */
static void late_frame_run(struct late_line *l, const int16_t *ref,
			   const int16_t *near,
			   struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info)
{
	int16_t echo[UDSP_FRAME_MAX];
	int16_t floor[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	double gain = l->late_db > 0.0 ?
		      pow(10.0, -(CHANGE_ERL_DB + l->late_db) / 20.0) : 0.0;
	unsigned i;

	path_run(&l->path, ref, echo);
	noise_fill(floor, l->frame, CHANGE_FLOOR_DBM0, &l->hiss);
	memmove(l->hist, l->hist + l->frame, l->delay * sizeof(*l->hist));
	memcpy(l->hist + l->delay, ref, l->frame * sizeof(*ref));
	for (i = 0; i < l->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + floor[i] +
				      gain * l->hist[i] +
				      (near ? near[i] : 0));
	udsp_ec_nlms.run(l->ec, mic, ref);
	udsp_ec_nlms_frame_get(l->ec, reading);
	udsp_ec_nlms_info_get(l->ec, info);
}

/* G.168 test 3C on the late line from a fresh canceller: near talker alone,
 * double talk, far end alone. Later come a copy dip under the gate and its
 * return, then a reset and a second convergence. */
#define FIRST_NEAR_FRAMES	(2u * FRAMES_PER_SECOND)
#define FIRST_TALK_FRAMES	(2u * FRAMES_PER_SECOND)
#define FIRST_FAR_FRAMES	(4u * FRAMES_PER_SECOND)
#define FIRST_DIP_FRAMES	(2u * FRAMES_PER_SECOND)
#define FIRST_LATE_DB		16.0
/* Past the 8th pass frame from the first one, the end of any engage wait
 * there. */
#define FIRST_WAIT_SPAN		16u

/* first, later and reset take the figures of each convergence. */
struct first_figures {
	unsigned erle;
	unsigned waits;
	unsigned first_pass;
	bool acted_first;
	bool found;
};

/* The far end alone for frames frames. fig takes the erle frames before the
 * first pass frame and the wait frames up to FIRST_WAIT_SPAN frames after it.
 * It also takes whether the processor acts on that pass frame. */
static void first_far_run(struct late_line *l, uint32_t *state,
			  unsigned long *phase, unsigned frames,
			  struct first_figures *fig)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	uint32_t waits;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	udsp_ec_nlms_info_get(l->ec, &info);
	waits = info.nlp_wait_frames;
	for (f = 0; f < frames; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		css_fill(ref, l->frame, CHANGE_FAR_DBM0, state, phase, l->rate);
		late_frame_run(l, ref, NULL, &reading, &info);
		if (fig->found) {
			if (f == fig->first_pass + FIRST_WAIT_SPAN)
				fig->waits = info.nlp_wait_frames - waits;
			continue;
		}
		fig->erle += reading.nlp_verdict == LECR_VERDICT_ERLE;
		if (reading.nlp_verdict == LECR_VERDICT_PASS) {
			fig->found = true;
			fig->first_pass = f;
			fig->acted_first = info.nlp_active;
		}
	}
	if (!fig->found || fig->first_pass + FIRST_WAIT_SPAN >= frames)
		fig->waits = info.nlp_wait_frames - waits;
}

static void first_engage_run(unsigned rate, struct first_figures *first,
			     struct first_figures *later,
			     struct first_figures *reset, unsigned *dip_erle)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct late_line l;
	uint32_t state = 0x13572468u;
	unsigned long phase = 0;
	unsigned f;

	late_line_init(&l, rate, 0u);
	word_source_init(&talker, rate, CHANGE_ECHO_DBM0, 0x0badcafeu);
	for (f = 0; f < FIRST_NEAR_FRAMES + FIRST_TALK_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];

		css_fill(ref, l.frame, CHANGE_FAR_DBM0, &state, &phase, rate);
		if (f < FIRST_NEAR_FRAMES)
			memset(ref, 0, sizeof(ref));
		word_fill(&talker, near, l.frame);
		late_frame_run(&l, ref, near, &reading, &info);
	}
	first_far_run(&l, &state, &phase, FIRST_FAR_FRAMES, first);
	l.late_db = FIRST_LATE_DB;
	*dip_erle = 0;
	for (f = 0; f < FIRST_DIP_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		css_fill(ref, l.frame, CHANGE_FAR_DBM0, &state, &phase, rate);
		late_frame_run(&l, ref, NULL, &reading, &info);
		*dip_erle += reading.nlp_verdict == LECR_VERDICT_ERLE;
	}
	l.late_db = 0.0;
	first_far_run(&l, &state, &phase, FIRST_FAR_FRAMES, later);
	udsp_ec_nlms_reset(l.ec);
	first_far_run(&l, &state, &phase, FIRST_FAR_FRAMES, reset);
	udsp_ec_nlms.destroy(l.ec);
}

/* The processor switches on over a line converged with it off; a later copy
 * drops the ERLE under the gate and the processor off. The pass runs before the
 * drop exceed LEC_NLP_FIRST_FRAMES, short of LEC_NLP_ON_FRAMES. */
#define ENABLE_CONVERGE_FRAMES	(4u * FRAMES_PER_SECOND)
#define ENABLE_OFF_AT		6u
#define ENABLE_LATE_AT		12u
#define ENABLE_LATE_DB		10.0
#define ENABLE_LATE_FRAMES	20u
#define ENABLE_FRAMES		60u
#define ENABLE_FAR_DBM0		-15.0

/* fig takes the pass runs, the dip and the return. */
static void first_enable_run(unsigned rate, struct first_figures *fig,
			     unsigned *early_acted)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct late_line l;
	uint32_t state = 0x13572468u;
	uint32_t waits = 0;
	unsigned since = 0;
	bool back = false;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	*early_acted = 0;
	late_line_init(&l, rate, 48u);
	udsp_ec_nlms.nlp(l.ec, false);
	for (f = 0; f < ENABLE_CONVERGE_FRAMES + ENABLE_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		unsigned at = f - ENABLE_CONVERGE_FRAMES;

		if (f >= ENABLE_CONVERGE_FRAMES)
			udsp_ec_nlms.nlp(l.ec, at != ENABLE_OFF_AT);
		if (f >= ENABLE_CONVERGE_FRAMES)
			l.late_db = at >= ENABLE_LATE_AT &&
				    at < ENABLE_LATE_AT + ENABLE_LATE_FRAMES ?
				    ENABLE_LATE_DB : 0.0;
		noise_fill(ref, l.frame, ENABLE_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
		if (f < ENABLE_CONVERGE_FRAMES)
			continue;
		if (!fig->erle && reading.nlp_verdict == LECR_VERDICT_PASS) {
			fig->first_pass++;
			*early_acted += info.nlp_active;
			continue;
		}
		if (!back) {
			fig->erle += reading.nlp_verdict == LECR_VERDICT_ERLE;
			if (!fig->erle ||
			    reading.nlp_verdict != LECR_VERDICT_PASS)
				continue;
			back = true;
			fig->found = true;
			fig->acted_first = info.nlp_active;
			waits = info.nlp_wait_frames - !info.nlp_active;
		}
		if (since++ <= FIRST_WAIT_SPAN)
			fig->waits = info.nlp_wait_frames - waits;
	}
	udsp_ec_nlms.destroy(l.ec);
}

/* The ERLE touches the gate at the first engage: after the switch-on the copy
 * stops for fewer acts than LEC_NLP_FIRST_FRAMES, returns to dip it, and stops.
 * With reset_acts the line converges with the processor on and resets first. */
#define TOUCH_LATE_DB		17.0
#define TOUCH_SETTLE_FRAMES	40u
#define TOUCH_OFF_FRAMES	4u
#define TOUCH_DIP_FRAMES	46u
#define TOUCH_FRAMES		100u

static void first_touch_run(unsigned rate, struct first_figures *fig,
			    unsigned *touch_acts, unsigned *reset_acts)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct late_line l;
	uint32_t state = 0x13572468u;
	unsigned off_at = TOUCH_SETTLE_FRAMES;
	unsigned back_at = off_at + TOUCH_OFF_FRAMES;
	unsigned end_at = back_at + TOUCH_DIP_FRAMES;
	uint32_t waits = 0;
	unsigned since = 0;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	*touch_acts = 0;
	late_line_init(&l, rate, 48u);
	for (f = 0; reset_acts && f < ENABLE_CONVERGE_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		if (!f)
			*reset_acts = 0;
		noise_fill(ref, l.frame, ENABLE_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
		*reset_acts += info.nlp_active;
	}
	if (reset_acts)
		udsp_ec_nlms_reset(l.ec);
	udsp_ec_nlms.nlp(l.ec, false);
	for (f = 0; f < ENABLE_CONVERGE_FRAMES + TOUCH_SETTLE_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		l.late_db = f >= ENABLE_CONVERGE_FRAMES ? TOUCH_LATE_DB : 0.0;
		noise_fill(ref, l.frame, ENABLE_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
	}
	udsp_ec_nlms.nlp(l.ec, true);
	for (f = 0; f < end_at + TOUCH_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		l.late_db = (f >= off_at && f < back_at) || f >= end_at ?
			    0.0 : TOUCH_LATE_DB;
		noise_fill(ref, l.frame, ENABLE_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
		if (f < end_at) {
			*touch_acts += info.nlp_active;
			fig->erle += f >= back_at &&
				     reading.nlp_verdict == LECR_VERDICT_ERLE;
			continue;
		}
		if (!fig->found) {
			if (reading.nlp_verdict != LECR_VERDICT_PASS)
				continue;
			fig->found = true;
			fig->acted_first = info.nlp_active;
			waits = info.nlp_wait_frames - !info.nlp_active;
		}
		if (since++ <= FIRST_WAIT_SPAN)
			fig->waits = info.nlp_wait_frames - waits;
	}
	udsp_ec_nlms.destroy(l.ec);
}

/* The ERLE that dips under the gate after DIP_ACTS acting frames of a first
 * engage, fewer than LEC_NLP_FIRST_FRAMES. dip_acts takes the acts before the
 * dip, and fig the erle frames of the dip and the crossing after it. */
#define DIP_PLATEAU_DB		20.0
#define DIP_DB			14.0
#define DIP_ACTS		10u
#define DIP_ERLE		2u
#define DIP_FRAMES		200u

static void first_dip_run(unsigned rate, struct first_figures *fig,
			  unsigned *dip_acts)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct late_line l;
	uint32_t state = 0x13572468u;
	uint32_t waits = 0;
	unsigned since = 0;
	unsigned stage = 0;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	*dip_acts = 0;
	late_line_init(&l, rate, 48u);
	udsp_ec_nlms.nlp(l.ec, false);
	for (f = 0; f < ENABLE_CONVERGE_FRAMES + TOUCH_SETTLE_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		l.late_db = f >= ENABLE_CONVERGE_FRAMES ? TOUCH_LATE_DB : 0.0;
		noise_fill(ref, l.frame, ENABLE_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
	}
	udsp_ec_nlms.nlp(l.ec, true);
	for (f = 0; f < DIP_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		l.late_db = stage == 0 ? DIP_PLATEAU_DB :
			    stage == 1 ? DIP_DB : 0.0;
		noise_fill(ref, l.frame, ENABLE_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
		if (stage == 0) {
			*dip_acts += info.nlp_active;
			stage = *dip_acts >= DIP_ACTS;
			continue;
		}
		if (stage == 1) {
			fig->erle += reading.nlp_verdict == LECR_VERDICT_ERLE;
			if (fig->erle >= DIP_ERLE)
				stage = 2;
			continue;
		}
		if (!fig->found) {
			if (reading.nlp_verdict != LECR_VERDICT_PASS)
				continue;
			fig->found = true;
			fig->acted_first = info.nlp_active;
			waits = info.nlp_wait_frames - !info.nlp_active;
		}
		if (since++ <= FIRST_WAIT_SPAN)
			fig->waits = info.nlp_wait_frames - waits;
	}
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The first engage: in a far-alone 3C convergence, erle frames fill the run, so
 * the processor acts on its first pass frame; later a return from a dip waits.
 * No frame waits after a reset, a switch-on, a touch of the gate or a dip after
 * 10 acts.
 */
static void nlp_first_engage_test(unsigned rate)
{
	struct first_figures first;
	struct first_figures later;
	struct first_figures reset;
	struct first_figures enable;
	struct first_figures touch;
	struct first_figures again;
	struct first_figures dip;
	unsigned early_acted;
	unsigned dip_acts;
	unsigned touch_acts;
	unsigned again_acts;
	unsigned reset_acts;
	unsigned dip_erle;

	first_engage_run(rate, &first, &later, &reset, &dip_erle);
	first_enable_run(rate, &enable, &early_acted);
	first_touch_run(rate, &touch, &touch_acts, NULL);
	first_touch_run(rate, &again, &again_acts, &reset_acts);
	first_dip_run(rate, &dip, &dip_acts);
	case_rate(rate, "lec nlp acts from the first pass frame of the first "
		  "engage and keeps the hysteresis after it");
	printf("# 3C sequence from a fresh canceller, far alone: %u erle "
	       "frames before the first pass frame %u, which the processor %s; "
	       "%u wait frames; later, %u erle frames in the dip, then %u "
	       "erle frames before the first pass frame %u and %u wait "
	       "frames; after a reset, %u erle frames before the first pass "
	       "frame %u, %u wait frames\n", first.erle, first.first_pass,
	       first.acted_first ? "acts on" : "does not act on", first.waits,
	       dip_erle, later.erle, later.first_pass, later.waits, reset.erle,
	       reset.first_pass, reset.waits);
	printf("# switched on over a converged line: %u pass frames, %u of "
	       "them acted on, then %u erle frames; the return %s its first "
	       "pass frame, %u wait frames\n", enable.first_pass, early_acted,
	       enable.erle, enable.acted_first ? "acts on" : "waits on",
	       enable.waits);
	printf("# ERLE touching the gate: %u acting frames, then %u erle "
	       "frames; the crossing %s its first pass frame, %u wait "
	       "frames\n", touch_acts, touch.erle,
	       touch.acted_first ? "acts on" : "waits on", touch.waits);
	printf("# the same after a reset, %u acting frames before it: %u "
	       "acting frames, then %u erle frames; the crossing %s its first "
	       "pass frame, %u wait frames\n", reset_acts, again_acts,
	       again.erle, again.acted_first ? "acts on" : "waits on",
	       again.waits);
	printf("# ERLE dipping after %u acting frames: %u erle frames; the "
	       "crossing %s its first pass frame, %u wait frames\n", dip_acts,
	       dip.erle, dip.acted_first ? "acts on" : "waits on", dip.waits);
	CHECK(first.found);
	CHECK(first.erle > 0u);
	CHECK(first.acted_first);
	CHECK_UINT(first.waits, 0);
	CHECK(dip_erle > 0u);
	CHECK(later.found);
	CHECK_UINT(later.waits, WORDS_ENGAGE_FRAME);
	CHECK(reset.found);
	CHECK(reset.erle > 0u);
	CHECK(reset.acted_first);
	CHECK_UINT(reset.waits, 0);
	CHECK(enable.first_pass > WORDS_ENGAGE_FRAME);
	CHECK_UINT(early_acted, 0);
	CHECK(enable.erle > 0u);
	CHECK(enable.found);
	CHECK(enable.acted_first);
	CHECK_UINT(enable.waits, 0);
	CHECK(touch_acts > 0u);
	CHECK(touch_acts < WORDS_ENGAGE_FRAME + 1u);
	CHECK(touch.erle > 0u);
	CHECK(touch.found);
	CHECK(touch.acted_first);
	CHECK_UINT(touch.waits, 0);
	CHECK(reset_acts > ENABLE_CONVERGE_FRAMES / 2u);
	CHECK(again_acts > 0u);
	CHECK(again_acts < WORDS_ENGAGE_FRAME + 1u);
	CHECK(again.erle > 0u);
	CHECK(again.found);
	CHECK(again.acted_first);
	CHECK_UINT(again.waits, 0);
	CHECK(dip_acts >= DIP_ACTS);
	CHECK(dip.erle >= DIP_ERLE);
	CHECK(dip.found);
	CHECK(dip.acted_first);
	CHECK_UINT(dip.waits, 0);
}

/* The hover line: the late line with the copy HOVER_LATE_DB under the echo, so
 * the ERLE settles at the gate; the case counts the processor's changes of
 * state over HOVER_FRAMES. */
#define HOVER_LATE_DB		18.0
#define HOVER_FAR_DBM0		-15.0
#define HOVER_SETTLE_FRAMES	(10u * FRAMES_PER_SECOND)
#define HOVER_FRAMES		(30u * FRAMES_PER_SECOND)
#define HOVER_EDGES_8K		20u
#define HOVER_EDGES_16K		18u

static void nlp_gate_hover_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct late_line l;
	uint32_t state = 0x13572468u;
	unsigned edges = 0;
	unsigned erle = 0;
	double erle_db = 0.0;
	bool last = false;
	unsigned f;

	late_line_init(&l, rate, 48u);
	l.late_db = HOVER_LATE_DB;
	for (f = 0; f < HOVER_SETTLE_FRAMES + HOVER_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		noise_fill(ref, l.frame, HOVER_FAR_DBM0, &state);
		late_frame_run(&l, ref, NULL, &reading, &info);
		if (f >= HOVER_SETTLE_FRAMES) {
			edges += info.nlp_active != last;
			erle += reading.nlp_verdict == LECR_VERDICT_ERLE;
			erle_db += info.erle_db;
		}
		last = info.nlp_active;
	}
	udsp_ec_nlms.destroy(l.ec);
	case_rate(rate, "lec nlp keeps its flap guard where the ERLE hovers at "
		  "the gate");
	printf("# ERLE %.1f dB on average, %u erle frames of %u; %u changes of "
	       "state, %.2f a second, against %u for a run of pass frames "
	       "alone\n",
	       erle_db / HOVER_FRAMES, erle, HOVER_FRAMES, edges,
	       edges * (double)FRAMES_PER_SECOND / HOVER_FRAMES,
	       rate == UDSP_SAMPLE_RATE ? HOVER_EDGES_8K : HOVER_EDGES_16K);
	CHECK(erle > 0u);
	CHECK(edges <= (rate == UDSP_SAMPLE_RATE ? HOVER_EDGES_8K :
			HOVER_EDGES_16K));
}

/* The legs of hangover_tail_pair_test on the late line: pair (late copy alone)
 * and floor (second verdicts 12 to 15 dB over the floor). Then bar (6 dB or
 * more over it) and talker (test 3B at X 30, in far pauses). */
#define TAIL_PAIR_FAR_DBM0	-10.0
#define TAIL_PAIR_SETTLE_FRAMES	(10u * FRAMES_PER_SECOND)
#define TAIL_PAIR_FRAMES	(6u * FRAMES_PER_SECOND)
/* The far frames of the pair leg that the processor must act on, a share. */
#define TAIL_PAIR_ACT_SHARE	0.5
/* LEC_GEIGEL_HANGOVER_SHORT of dsp/lec.c. */
#define TAIL_PAIR_SHORT_FRAMES	4u
/* The pairs that each leg must form. */
#define TAIL_PAIR_MIN		4u
/* The bounds of dsp/lec.c as power ratios over the bar and the floor. The bar
 * leg must form pairs over the bar bound and within the floor bound. */
#define TAIL_PAIR_MARGINAL_RATIO	2.0
#define TAIL_PAIR_BAR_RATIO	4.0
#define TAIL_PAIR_FLOOR_RATIO	16.0
#define TAIL_PAIR_BAR_MIN	2u

struct tail_pair_leg {
	const char *name;
	double late_db[2];
	double near_dbm0[2];
	unsigned pos[2];
};

static const struct tail_pair_leg tail_pair_legs[] = {
	{ "pair", { 40.0, 40.0 }, { 0.0, 0.0 }, { 0u, 48u } },
	{ "floor", { 36.5, 36.0 }, { 0.0, 0.0 }, { 8u, 24u } },
	{ "bar", { 0.0, 39.0 }, { -59.0, 0.0 }, { 128u, 136u } },
	{ "talker", { 0.0, 0.0 }, { -40.0, -40.0 }, { 24u, 16u } },
};

/*
 * The figures of a leg: pairs whose second verdict stands on a thin frame and
 * those that keep the short hangover. Pairs on an active frame and those that
 * take the long one follow. Then the levels of thin second verdicts, the pairs
 * over the bar bound and in the floor bound, and the far and acting frames.
 */
struct tail_pair_result {
	unsigned thin;
	unsigned thin_short;
	unsigned active;
	unsigned active_long;
	double bar_lo, bar_hi;
	double floor_lo, floor_hi;
	unsigned bar_pairs;
	unsigned bar_short;
	unsigned far;
	unsigned acted;
};

static double tail_pair_db(double power)
{
	return 10.0 * log10(power > 1e-9 ? power : 1e-9);
}

/* Whether the frame of reading is a thin frame (thin_frame_get of
 * dsp/lec.c). */
static bool pair_thin_get(const struct udsp_ec_nlms_frame *reading)
{
	bool active = reading->ref_power >= reading->ref_bar;

	return (reading->classic_arm && reading->window_run == 1u) || !active;
}

/* One verdict that is the second of a pair: its side and its levels. The
 * function returns whether the pair stands over the bar bound and within the
 * floor bound. */
static bool tail_pair_count(struct tail_pair_result *r, bool thin,
			    const struct udsp_ec_nlms_frame *reading)
{
	double over_bar = tail_pair_db(reading->mic_power) -
			  tail_pair_db(reading->bar);
	double over_floor = tail_pair_db(reading->mic_power) -
			    tail_pair_db(reading->noise_power);

	if (!thin) {
		r->active++;
		return false;
	}
	r->thin++;
	r->bar_lo = fmin(r->bar_lo, over_bar);
	r->bar_hi = fmax(r->bar_hi, over_bar);
	r->floor_lo = fmin(r->floor_lo, over_floor);
	r->floor_hi = fmax(r->floor_hi, over_floor);
	if (over_bar < 10.0 * log10(TAIL_PAIR_BAR_RATIO) ||
	    over_floor >= 10.0 * log10(TAIL_PAIR_FLOOR_RATIO))
		return false;
	r->bar_pairs++;
	return true;
}

static void tail_pair_run(unsigned rate, const struct tail_pair_leg *leg,
			  struct tail_pair_result *r)
{
	unsigned k = rate == UDSP_SAMPLE_RATE_WB;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct css_annex css;
	struct late_line l;
	uint32_t near_state = 0x31415926u;
	unsigned long near_phase = 0;
	bool last_thin = false, last_short = false, fired_before = false;
	bool pending = false, pending_thin = false, pending_bar = false;
	unsigned f;

	memset(r, 0, sizeof(*r));
	r->bar_lo = r->floor_lo = 99.0;
	r->bar_hi = r->floor_hi = -99.0;
	late_line_init(&l, rate, 0u);
	l.late_db = leg->late_db[k];
	css_annex_init(&css, rate, TAIL_PAIR_FAR_DBM0, 7u);
	css.pos = leg->pos[k];
	for (f = 0; f < TAIL_PAIR_SETTLE_FRAMES + TAIL_PAIR_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		bool active, thin;

		css_annex_fill(ref, l.frame, &css);
		if (leg->near_dbm0[k] < 0.0)
			css_fill(near, l.frame, leg->near_dbm0[k], &near_state,
				 &near_phase, rate);
		late_frame_run(&l, ref, leg->near_dbm0[k] < 0.0 ? near : NULL,
			       &reading, &info);
		active = reading.ref_power >= reading.ref_bar;
		thin = pair_thin_get(&reading);
		/* hold_left reads the hangover that the verdict of the frame
		 * before set. */
		if (fired_before) {
			last_short = reading.hold_left ==
				     TAIL_PAIR_SHORT_FRAMES;
			if (pending && pending_bar)
				r->bar_short += reading.hold_left <=
						TAIL_PAIR_SHORT_FRAMES;
			if (pending && pending_thin)
				r->thin_short += reading.hold_left <=
						 TAIL_PAIR_SHORT_FRAMES;
			else if (pending)
				r->active_long += reading.hold_left >
						  TAIL_PAIR_SHORT_FRAMES;
		}
		pending = false;
		if (f >= TAIL_PAIR_SETTLE_FRAMES && reading.dt_fired &&
		    reading.hold_left > 0u &&
		    reading.hold_left <= TAIL_PAIR_SHORT_FRAMES &&
		    last_thin && last_short) {
			pending_bar = tail_pair_count(r, thin, &reading);
			pending = true;
			pending_thin = thin;
		}
		if (reading.dt_fired)
			last_thin = thin;
		fired_before = reading.dt_fired;
		if (f >= TAIL_PAIR_SETTLE_FRAMES && active) {
			r->far++;
			r->acted += info.nlp_active;
		}
	}
	udsp_ec_nlms.destroy(l.ec);
}

static void tail_pair_print(unsigned rate, const char *name,
			    const struct tail_pair_result *r)
{
	printf("# %u Hz, %s: %u thin pairs, %u keep the short hangover, "
	       "%.1f to %.1f dB over the bar, %.1f to %.1f dB over the "
	       "floor; %u over the bar bound and within the floor bound, %u "
	       "of them keep the short hangover; %u active pairs, %u take the "
	       "long one; the processor acts on %u of %u far frames\n", rate,
	       name, r->thin, r->thin_short, r->bar_lo, r->bar_hi,
	       r->floor_lo, r->floor_hi, r->bar_pairs, r->bar_short,
	       r->active, r->active_long, r->acted, r->far);
}

/*
 * The thin pair of dsp/lec.c (the LEC_GEIGEL_HANGOVER_SHORT comment) on the
 * late line. Its copy past the tail fires on the tail frames of each burst
 * end. On the pair leg every second verdict keeps the short hangover, and the
 * processor acts on TAIL_PAIR_ACT_SHARE of the far frames or more. The floor,
 * bar and talker legs take the long hangover through each bound.
 */
static void hangover_tail_pair_test(unsigned rate)
{
	unsigned i;

	for (i = 0; i < sizeof(tail_pair_legs) / sizeof(tail_pair_legs[0]);
	     i++) {
		const struct tail_pair_leg *leg = &tail_pair_legs[i];
		struct tail_pair_result r;
		char what[96];

		tail_pair_run(rate, leg, &r);
		snprintf(what, sizeof(what), "lec hangover of a thin pair, %s "
			 "leg", leg->name);
		case_rate(rate, what);
		tail_pair_print(rate, leg->name, &r);
		CHECK(r.thin >= TAIL_PAIR_MIN);
		if (i == 0u) {
			CHECK_UINT(r.thin_short, r.thin);
			CHECK((double)r.acted >= TAIL_PAIR_ACT_SHARE * r.far);
		} else if (i == 2u) {
			CHECK(r.bar_pairs >= TAIL_PAIR_BAR_MIN);
			CHECK_UINT(r.bar_short, 0);
		} else {
			CHECK_UINT(r.thin_short, 0);
		}
		CHECK_UINT(r.active_long, r.active);
	}
}

/* The frames of hangover_pair_script_test: after convergence, cycles of pause,
 * burst, far gap and a burst whose onset frame holds PAIR_SCRIPT_ONSET_MS.
 * Pulses sit on the last gap and onset frames, or on the burst's last frame. */
#define PAIR_SCRIPT_CONVERGE_FRAMES	(4u * FRAMES_PER_SECOND)
#define PAIR_SCRIPT_CYCLES	4u
#define PAIR_SCRIPT_PAUSE_FRAMES	30u
#define PAIR_SCRIPT_BURST_FRAMES	20u
#define PAIR_SCRIPT_GAP_FRAMES	4u
#define PAIR_SCRIPT_ONSET_MS	1u
#define PAIR_SCRIPT_PULSE_DB	5.0
#define PAIR_SCRIPT_LOUD_DBM0	-15.0
/* The bounds of a thin pair in dsp/lec.c: LEC_GEIGEL_PAIR_RATIO over the bar
 * and LEC_GEIGEL_SOFT_RATIO over the floor, power ratios. */
#define PAIR_SCRIPT_BAR_RATIO	4.0
#define PAIR_SCRIPT_FLOOR_RATIO	16.0

/*
 * The figures of a leg. An event is a second verdict inside the hangover of a
 * first one, outside a trial. It stands within both bounds of a thin pair, on
 * the frame types that the leg sets. events_long counts those that take the
 * long hangover; bar_lo .. floor_hi their levels in dB.
 */
struct pair_script_result {
	unsigned events;
	unsigned events_long;
	double bar_lo, bar_hi;
	double floor_lo, floor_hi;
};

/* Counts the second verdict of a cycle when it is an event, and returns
 * whether it is one. */
static bool pair_script_count(struct pair_script_result *r, bool onset_leg,
			      const struct udsp_ec_nlms_frame *reading)
{
	bool inside = reading->hold_left > 0u &&
		      (!onset_leg ||
		       reading->hold_left <= TAIL_PAIR_SHORT_FRAMES);

	if (!reading->dt_fired || reading->probing || !inside ||
	    pair_thin_get(reading) == onset_leg ||
	    reading->mic_power >= PAIR_SCRIPT_BAR_RATIO * reading->bar ||
	    reading->mic_power >=
	    PAIR_SCRIPT_FLOOR_RATIO * reading->noise_power)
		return false;
	r->events++;
	r->bar_lo = fmin(r->bar_lo, tail_pair_db(reading->mic_power) -
				    tail_pair_db(reading->bar));
	r->bar_hi = fmax(r->bar_hi, tail_pair_db(reading->mic_power) -
				    tail_pair_db(reading->bar));
	r->floor_lo = fmin(r->floor_lo, tail_pair_db(reading->mic_power) -
					tail_pair_db(reading->noise_power));
	r->floor_hi = fmax(r->floor_hi, tail_pair_db(reading->mic_power) -
					tail_pair_db(reading->noise_power));
	return true;
}

/* The reference of frame at of a cycle of the scripted frames. */
static void pair_script_ref_fill(int16_t *ref, unsigned frame, unsigned at,
				 unsigned onset_samples, uint32_t *state)
{
	unsigned end = PAIR_SCRIPT_PAUSE_FRAMES + PAIR_SCRIPT_BURST_FRAMES;
	unsigned onset = end + PAIR_SCRIPT_GAP_FRAMES;

	memset(ref, 0, frame * sizeof(*ref));
	if ((at >= PAIR_SCRIPT_PAUSE_FRAMES && at < end) || at > onset)
		noise_fill(ref, frame, TAIL_PAIR_FAR_DBM0, state);
	else if (at == onset)
		noise_fill(ref + frame - onset_samples, onset_samples,
			   TAIL_PAIR_FAR_DBM0, state);
}

/* One leg of the scripted frames on a new instance; r takes its figures. */
static void pair_script_run(unsigned rate, bool onset_leg,
			    struct pair_script_result *r)
{
	unsigned end = PAIR_SCRIPT_PAUSE_FRAMES + PAIR_SCRIPT_BURST_FRAMES;
	unsigned onset = end + PAIR_SCRIPT_GAP_FRAMES;
	unsigned cycle = onset + PAIR_SCRIPT_BURST_FRAMES;
	unsigned first_at = onset_leg ? onset - 1u : end - 1u;
	unsigned second_at = onset_leg ? onset : onset - 1u;
	unsigned onset_samples = PAIR_SCRIPT_ONSET_MS * rate / 1000u;
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct late_line l;
	uint32_t far_state = 0x13572468u, near_state = 0x24681357u;
	bool set = false, pending = false;
	unsigned f;

	memset(r, 0, sizeof(*r));
	r->bar_lo = r->floor_lo = 99.0;
	r->bar_hi = r->floor_hi = -99.0;
	late_line_init(&l, rate, 0u);
	for (f = 0; f < FRAMES_PER_SECOND + PAIR_SCRIPT_CONVERGE_FRAMES; f++) {
		int16_t ref[UDSP_FRAME_MAX];

		memset(ref, 0, sizeof(ref));
		if (f >= FRAMES_PER_SECOND)
			noise_fill(ref, l.frame, TAIL_PAIR_FAR_DBM0,
				   &far_state);
		late_frame_run(&l, ref, NULL, &reading, &info);
	}
	for (f = 0; f < PAIR_SCRIPT_CYCLES * cycle; f++) {
		int16_t ref[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		unsigned at = f % cycle;
		bool pulse = at == first_at || at == second_at;

		pair_script_ref_fill(ref, l.frame, at, onset_samples,
				     &far_state);
		if (pulse)
			noise_fill(near, l.frame,
				   !onset_leg && at == first_at ?
				   PAIR_SCRIPT_LOUD_DBM0 :
				   CHANGE_FLOOR_DBM0 + PAIR_SCRIPT_PULSE_DB,
				   &near_state);
		late_frame_run(&l, ref, pulse ? near : NULL, &reading, &info);
		/* hold_left reads the hangover that the verdict of the frame
		 * before set. */
		if (pending)
			r->events_long += reading.hold_left >
					  TAIL_PAIR_SHORT_FRAMES;
		pending = at == second_at && set &&
			  pair_script_count(r, onset_leg, &reading);
		if (reading.dt_fired)
			set = at == first_at && !reading.probing &&
			      pair_thin_get(&reading) == onset_leg;
	}
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The thin pair on scripted frames: a verdict on an active frame inside the
 * short hangover of a thin verdict takes the long hangover. So does a thin
 * verdict inside the hangover of an active one. Each event stands within both
 * bounds, and each leg forms one or more at each rate.
 */
static void hangover_pair_script_test(unsigned rate)
{
	static const char *const names[] = { "onset", "burst end" };
	unsigned k;

	for (k = 0; k < 2u; k++) {
		struct pair_script_result r;
		char what[96];

		pair_script_run(rate, k == 0u, &r);
		snprintf(what, sizeof(what), "lec hangover of a thin pair, %s "
			 "leg", names[k]);
		case_rate(rate, what);
		printf("# %u Hz, %s: %u events, %u take the long hangover, "
		       "%.1f to %.1f dB over the bar, %.1f to %.1f dB over the "
		       "floor\n", rate, names[k], r.events, r.events_long,
		       r.bar_lo, r.bar_hi, r.floor_lo, r.floor_hi);
		CHECK(r.events >= 1u);
		CHECK_UINT(r.events_long, r.events);
	}
}

/* The floor peak line (G.168 test 3C t3): the composite hybrid at 12 dB, the
 * composite source at -10 dBm0, a -70 dBm0 floor, the processor on. Each noise
 * segment completes the engage run; the echo-like pause frames keep it. */
#define PEAK_CYCLE_FRAMES	50u
#define PEAK_ONSET_FRAME	20u
#define PEAK_ONSET_FRAMES	20u
#define PEAK_SETTLE_PERIODS	4u
/* The peak of the line floor over the tracked floor in a captured 3C t3 pause.
 * It stands just over the bar of a quiet frame with no estimate. */
#define PEAK_OVER_DB		6.5
#define PEAK_FRAMES_MAX		3u

/*
 * One frame of the floor peak line, with its floor over_db over the tracked
 * floor of reading. reading holds the reading of the previous frame. The
 * frame carries the line's own floor for that frame, scaled, so the peak is
 * the line floor itself. reading and info then take the readings of the
 * frame.
 */
static void peak_frame_run(struct cohere_line *l, double over_db,
			   struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info)
{
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	uint32_t state = l->state[2];
	double echo_energy = 0.0;
	double out_energy = 0.0;
	double gain;
	unsigned i;

	noise_fill(hiss, l->frame, CHANGE_FLOOR_DBM0, &state);
	gain = sqrt((double)reading->noise_power * pow(10.0, over_db / 10.0) /
		    udsp_ec_nlms_frame_power_get(hiss, l->frame));
	for (i = 0; i < l->frame; i++)
		near[i] = sample_clamp((gain - 1.0) * hiss[i]);
	cohere_frame_run(l, near, &echo_energy, &out_energy, reading, info);
}

/* The figures of a floor peak run: the readings of the peak frames, and the
 * wait frames and first acting frame of the noise segment after them. The later
 * peak follows with its waits. */
struct peak_result {
	struct udsp_ec_nlms_frame peak[PEAK_FRAMES_MAX];
	struct udsp_ec_nlms_frame again;
	unsigned at;
	unsigned waits;
	unsigned acts_from;
	unsigned again_waits;
};

/* The later peak of a floor peak run, on the first quiet frame after the
 * noise segment. The line then runs to the end of the next noise segment. */
static void peak_again_run(struct cohere_line *l, struct peak_result *r,
			   struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info)
{
	unsigned from = PEAK_ONSET_FRAME + PEAK_ONSET_FRAMES;
	unsigned quiet_at = from + window_frames_get(l->ec) - 1u;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	uint32_t waits;
	unsigned f;

	for (f = from; f < quiet_at; f++)
		cohere_frame_run(l, NULL, &echo_energy, &out_energy, reading,
				 info);
	peak_frame_run(l, PEAK_OVER_DB, reading, info);
	r->again = *reading;
	waits = info->nlp_wait_frames;
	for (f = quiet_at + 1u; f < PEAK_CYCLE_FRAMES + from; f++)
		cohere_frame_run(l, NULL, &echo_energy, &out_energy, reading,
				 info);
	r->again_waits = info->nlp_wait_frames - waits;
}

/* A floor peak on peaks frames in a row, ending on the last quiet frame
 * before the noise segment of a period. That segment follows, and then the
 * later peak of peak_again_run when again is set. */
static void peak_run(unsigned rate, unsigned peaks, bool again,
		     struct peak_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	uint32_t waits;
	unsigned f;

	memset(r, 0, sizeof(*r));
	path_composite_init(&path, rate, CHANGE_ERL_DB);
	cohere_line_begin(&l, rate, &path, 0u, true, 0u);
	udsp_ec_nlms.nlp(l.ec, true);
	for (f = 0; f < PEAK_SETTLE_PERIODS * PEAK_CYCLE_FRAMES +
			PEAK_ONSET_FRAME - peaks; f++)
		cohere_frame_run(&l, NULL, &echo_energy, &out_energy, &reading,
				 &info);
	r->at = (unsigned)(l.phase / l.frame);
	for (f = 0; f < peaks; f++) {
		peak_frame_run(&l, PEAK_OVER_DB, &reading, &info);
		r->peak[f] = reading;
	}
	waits = info.nlp_wait_frames;
	r->acts_from = PEAK_ONSET_FRAMES;
	for (f = 0; f < PEAK_ONSET_FRAMES; f++) {
		cohere_frame_run(&l, NULL, &echo_energy, &out_energy, &reading,
				 &info);
		if (info.nlp_active && r->acts_from == PEAK_ONSET_FRAMES)
			r->acts_from = f;
	}
	r->waits = info.nlp_wait_frames - waits;
	if (again)
		peak_again_run(&l, r, &reading, &info);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * Floor peaks PEAK_OVER_DB over the tracked floor stand over the quiet bar and
 * under the bound; one or two in a row before a far onset keep the engage run.
 * Three in a row zero it, and the onset resumes it (LEC_NLP_RESUME_DB), so each
 * onset acts on its first frame.
 */
static void floor_peak_test(unsigned rate)
{
	struct peak_result none;
	struct peak_result one;
	struct peak_result two;
	struct peak_result three;
	unsigned f;

	peak_run(rate, 0u, false, &none);
	peak_run(rate, 1u, true, &one);
	peak_run(rate, 2u, false, &two);
	peak_run(rate, 3u, false, &three);
	case_rate(rate, "lec nlp keeps the engage run through two floor peaks "
		  "in a row before a far onset and resumes it after three");
	printf("# no peak: %u wait frames at the onset, acts from its frame "
	       "%u; one peak frame %.2f dB over the tracked floor, frame %u of "
	       "the period: %u wait frames, acts from frame %u; a later peak "
	       "%.2f dB over it on the first quiet frame after the segment: "
	       "%u wait frames to the end of the next segment; two peak "
	       "frames %.2f and %.2f dB over it: %u wait frames, acts from "
	       "frame %u; three peak frames %.2f, %.2f and %.2f dB over it: "
	       "%u wait frames, acts from frame %u\n", none.waits,
	       none.acts_from,
	       ratio_db_get(one.peak[0].mic_power, one.peak[0].noise_power),
	       one.at, one.waits, one.acts_from,
	       ratio_db_get(one.again.mic_power, one.again.noise_power),
	       one.again_waits,
	       ratio_db_get(two.peak[0].mic_power, two.peak[0].noise_power),
	       ratio_db_get(two.peak[1].mic_power, two.peak[1].noise_power),
	       two.waits, two.acts_from,
	       ratio_db_get(three.peak[0].mic_power,
			    three.peak[0].noise_power),
	       ratio_db_get(three.peak[1].mic_power,
			    three.peak[1].noise_power),
	       ratio_db_get(three.peak[2].mic_power,
			    three.peak[2].noise_power),
	       three.waits, three.acts_from);
	CHECK_UINT(none.waits, 0);
	CHECK_UINT(none.acts_from, 0);
	CHECK_UINT(one.at, PEAK_ONSET_FRAME - 1u);
	CHECK_UINT(two.at, PEAK_ONSET_FRAME - 2u);
	CHECK_UINT(three.at, PEAK_ONSET_FRAME - 3u);
	for (f = 0; f < 3u; f++) {
		const struct udsp_ec_nlms_frame *p = &three.peak[f];

		CHECK_UINT(p->nlp_verdict, LECR_VERDICT_QUIET);
		CHECK(p->noise_power > 0.0f);
		CHECK(p->estimate_power == 0.0f);
		CHECK(ratio_db_get(p->mic_power, p->noise_power) >
		      PEAK_OVER_DB - 0.25);
	}
	for (f = 0; f < 2u; f++) {
		const struct udsp_ec_nlms_frame *p = f ? &two.peak[1] :
							 &one.peak[0];

		CHECK_UINT(p->nlp_verdict, LECR_VERDICT_QUIET);
		CHECK(p->noise_power > 0.0f);
		CHECK(p->estimate_power == 0.0f);
		CHECK(ratio_db_get(p->mic_power, p->noise_power) >
		      PEAK_OVER_DB - 0.25);
	}
	CHECK_UINT(two.peak[0].nlp_verdict, LECR_VERDICT_QUIET);
	CHECK_UINT(one.again.nlp_verdict, LECR_VERDICT_QUIET);
	CHECK(ratio_db_get(one.again.mic_power, one.again.noise_power) >
	      PEAK_OVER_DB - 0.25);
	CHECK_UINT(one.waits, 0);
	CHECK_UINT(one.acts_from, 0);
	CHECK_UINT(one.again_waits, 0);
	CHECK_UINT(two.waits, 0);
	CHECK_UINT(two.acts_from, 0);
	CHECK_UINT(three.waits, 0);
	CHECK_UINT(three.acts_from, 0);
}

/*
 * G.168 test 3C on the floor peak line, a floor peak on each far pause's last
 * quiet frame. After the first engage, SEQ_LEVEL_DB bounds the active-window
 * output over the tracked floor, not the quiet frames, where the NLP releases.
 */
#define SEQ_NEAR_FRAMES		(2u * FRAMES_PER_SECOND)
#define SEQ_TALK_FRAMES		(2u * FRAMES_PER_SECOND)
#define SEQ_FAR_FRAMES		(5u * FRAMES_PER_SECOND)
#define SEQ_LEVEL_DB		6.0

/* The figures of the far-alone phase: the peak frames and those that read
 * quiet, and the first engage. After it come the waits, the refusals, the
 * active frames and those that act, and the loudest output of each kind. */
struct seq_result {
	unsigned peaks;
	unsigned peaks_quiet;
	unsigned first;
	unsigned waits;
	unsigned dt;
	unsigned unlike;
	unsigned erle;
	unsigned active;
	unsigned acted;
	double active_db;
	double quiet_db;
	bool engaged;
};

static void seq_note(struct seq_result *q, const struct cohere_line *l,
		     const struct ring_planes *planes,
		     const struct udsp_ec_nlms_frame *reading, bool acted)
{
	double energy = 0.0;
	double db;
	unsigned i;

	for (i = 0; i < l->frame; i++)
		energy += (double)planes->out[i] * planes->out[i];
	db = ratio_db_get((float)(energy / l->frame), reading->noise_power);
	if (reading->nlp_verdict == LECR_VERDICT_QUIET) {
		if (db > q->quiet_db)
			q->quiet_db = db;
		return;
	}
	q->active++;
	q->acted += acted;
	if (db > q->active_db)
		q->active_db = db;
}

/* The near-alone and double-talk phases of the sequence. */
static void seq_talk_run(struct cohere_line *l, struct word_source *talker,
			 struct udsp_ec_nlms_frame *reading,
			 struct udsp_ec_nlms_info *info)
{
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned f;

	l->quiet = true;
	for (f = 0; f < SEQ_NEAR_FRAMES + SEQ_TALK_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];

		if (f == SEQ_NEAR_FRAMES)
			l->quiet = false;
		word_fill(talker, near, l->frame);
		cohere_frame_run(l, near, &echo_energy, &out_energy, reading,
				 info);
	}
}

static void seq_run(unsigned rate, struct seq_result *q)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info engage;
	struct udsp_ec_nlms_info info;
	struct word_source talker;
	struct ring_planes planes;
	struct echo_path path;
	struct cohere_line l;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned f;

	memset(q, 0, sizeof(*q));
	memset(&engage, 0, sizeof(engage));
	q->active_db = -200.0;
	q->quiet_db = -200.0;
	path_composite_init(&path, rate, CHANGE_ERL_DB);
	cohere_line_begin(&l, rate, &path, 0u, true, 0u);
	l.planes = &planes;
	udsp_ec_nlms.nlp(l.ec, true);
	word_source_init(&talker, rate, CHANGE_ECHO_DBM0, 0x0badcafeu);
	seq_talk_run(&l, &talker, &reading, &info);
	for (f = 0; f < SEQ_FAR_FRAMES; f++) {
		unsigned p = (unsigned)(l.phase / l.frame);
		bool peak = (p == PEAK_ONSET_FRAME - 1u ||
			     p == PEAK_CYCLE_FRAMES - 1u) &&
			    reading.noise_power > 0.0f;

		if (peak)
			peak_frame_run(&l, PEAK_OVER_DB, &reading, &info);
		else
			cohere_frame_run(&l, NULL, &echo_energy, &out_energy,
					 &reading, &info);
		q->peaks += peak;
		q->peaks_quiet += peak &&
				  reading.nlp_verdict == LECR_VERDICT_QUIET;
		if (q->engaged) {
			seq_note(q, &l, &planes, &reading, info.nlp_active);
		} else if (info.nlp_active) {
			q->engaged = true;
			q->first = f;
			engage = info;
		}
	}
	q->waits = info.nlp_wait_frames - engage.nlp_wait_frames;
	q->dt = info.nlp_dt_frames - engage.nlp_dt_frames;
	q->unlike = info.nlp_unlike_frames - engage.nlp_unlike_frames;
	q->erle = info.nlp_erle_frames - engage.nlp_erle_frames;
	udsp_ec_nlms.destroy(l.ec);
}

/* After the first engage of the far-alone phase, every far onset acts on its
 * first frame. Every active-window frame stands at most SEQ_LEVEL_DB over the
 * tracked floor, so a line that waits no frame but passes echo fails too. */
static void floor_peak_sequence_test(unsigned rate)
{
	struct seq_result q;

	seq_run(rate, &q);
	case_rate(rate, "lec nlp acts on the first frame of every far onset of "
		  "a 3C sequence with floor peaks in the far pauses");
	printf("# near alone %u frames, double talk %u, far alone %u: %u floor "
	       "peaks %.1f dB over the tracked floor, %u of them on quiet "
	       "frames; first engage on far frame %u; after it %u wait, %u dt, "
	       "%u unlike and %u erle frames; the processor acts on %u of %u "
	       "frames with an active window, the loudest %.2f dB over the "
	       "tracked floor; quiet frames up to %.2f dB over it\n",
	       SEQ_NEAR_FRAMES, SEQ_TALK_FRAMES, SEQ_FAR_FRAMES, q.peaks,
	       PEAK_OVER_DB, q.peaks_quiet, q.first, q.waits, q.dt, q.unlike,
	       q.erle, q.acted, q.active, q.active_db, q.quiet_db);
	CHECK(q.engaged);
	CHECK(q.peaks > 0u);
	CHECK_UINT(q.peaks_quiet, q.peaks);
	CHECK(q.active > 0u);
	CHECK_UINT(q.waits, 0);
	CHECK(q.active_db <= SEQ_LEVEL_DB);
}

/* A soft near word ALT_WORD_OVER_DB over the floor and 19 dB under the echo
 * starts ALT_LEAD_FRAMES before the far word. Its pause frames alternate under
 * and over the bar of a quiet frame. */
#define ALT_LEAD_FRAMES		4u
#define ALT_WORD_OVER_DB	6.0
#define ALT_UNDER_DB		3.0
#define ALT_IN_DB		7.0
#define ALT_EDGE_DB		8.5
#define ALT_OUT_DB		11.0

static void alt_word_run(struct words_run *r, unsigned rate, double over_db)
{
	words_quiet_set(r, rate);
	r->near_word = WORDS_MARK;
	r->near_from = -(int)ALT_LEAD_FRAMES;
	r->near_frames = ALT_LEAD_FRAMES + WORDS_NEAR_FRAMES;
	r->near_dbm0 = FAN_FLOOR_DBM0 + ALT_WORD_OVER_DB;
	r->alt_over_db = over_db;
	r->alt_under_db = ALT_UNDER_DB;
	words_run(r);
}

static void alt_word_print(const char *what, const struct words_run *r)
{
	printf("# %s: pause frames %.2f to %.2f dB over the tracked floor; the "
	       "processor acts on %u of the %u frames of the word, from frame "
	       "%u of the far word; over its %u frames in the far word %.2f dB "
	       "through, filter alone %.2f dB; over all %u frames %.2f dB "
	       "through, filter alone %.2f dB\n",
	       what, r->near_pause_lo, r->near_pause_hi, r->near_acted,
	       r->near_frames, r->latency[WORDS_MARK], WORDS_NEAR_FRAMES,
	       near_gain_db_get(r->overlap_on, r->overlap),
	       near_gain_db_get(r->overlap_off, r->overlap), r->near_frames,
	       near_gain_db_get(r->near_on, r->near),
	       near_gain_db_get(r->near_off, r->near));
}

/*
 * A soft near word whose pause frames alternate over and under the bar. Under
 * the bound its frames keep the run and the processor replaces the word; over
 * the bound they zero the run, and the processor acts on no frame of it. Legs
 * that keep the run under any rule print with no check.
 */
static void floor_peak_talker_test(unsigned rate)
{
	struct words_run under;
	struct words_run in;
	struct words_run edge;
	struct words_run out;

	alt_word_run(&under, rate, ALT_UNDER_DB);
	alt_word_run(&in, rate, ALT_IN_DB);
	alt_word_run(&edge, rate, ALT_EDGE_DB);
	alt_word_run(&out, rate, ALT_OUT_DB);
	case_rate(rate, "lec nlp exposure of a soft near word that alternates "
		  "over and under the bar in a far pause");
	alt_word_print("all frames under the bar", &under);
	alt_word_print("frames over the bar under the bound", &in);
	alt_word_print("frames over the bar near the bound", &edge);
	alt_word_print("frames over the bound", &out);
	CHECK(under.near > 0.0);
	CHECK(in.near > 0.0);
	CHECK(edge.near > 0.0);
	CHECK(out.near > 0.0);
	CHECK_UINT(in.near_pause, ALT_LEAD_FRAMES);
	CHECK_UINT(out.near_pause, ALT_LEAD_FRAMES);
	CHECK(out.near_pause_hi > ALT_OUT_DB - 1.5);
	CHECK_UINT(out.near_acted, 0);
	CHECK_UINT(out.latency[WORDS_MARK], WORDS_ENGAGE_FRAME);
}

/* The lull line: a flat LULL_ERL_DB path behind LULL_BULK_MS of bulk delay, and
 * far noise bursts of LULL_ON_FRAMES in 30 from a third into a frame. A near
 * floor dips on one silent frame in LULL_DIP_EVERY. */
#define LULL_PERIOD_FRAMES	30u
#define LULL_ON_FRAMES		23u
#define LULL_ONSET_PHASE	(LULL_PERIOD_FRAMES - 1u)
#define LULL_BULK_MS		25u
#define LULL_ERL_DB		12.0
#define LULL_LATE_MS		44.0
#define LULL_LATE_DB		33.0
#define LULL_TAIL_MS		48u
#define LULL_FAR_DBM0		-20.0
#define LULL_FLOOR_DBM0		-63.0
#define LULL_DIP_DB		8.0
#define LULL_DIP_EVERY		3u
#define LULL_SETTLE_FRAMES	600u
#define LULL_BURSTS		50u
/* LEC_GEIGEL_FLOOR_HEADROOM of dsp/lec.c: the floor term of the bar. */
#define LULL_FLOOR_HEADROOM	2.0f
#define LULL_TALK_DB		6.0
#define LULL_TALK_LEAD		5u
#define LULL_CAP_TALK_DB	12.0
#define LULL_CAP_TALK_LEAD	(LULL_ONSET_PHASE - LULL_ON_FRAMES)
#define LULL_OLD_TALK_DB	12.0
#define LULL_AFTER_FRAMES	30u
#define LULL_SKIP_FRAMES	4u
#define LULL_SKIP_RATIO		1.5f
/* LEC_GEIGEL_MARGIN of dsp/lec.c: the margin of the bar. */
#define LULL_MARGIN		1.41f

struct lull_line {
	struct echo_path path;
	struct udsp_ec *ec;
	unsigned rate;
	unsigned frame;
	unsigned long pos;
	unsigned long count;
	unsigned quiet;
	double dip_db;
	uint32_t far_state;
	uint32_t floor_state;
	uint32_t talk_state;
};

/* late_db > 0 adds the late reflection that far under the main one. Onset, tail
 * and pre-echo frames carry no echo, but the reflection puts echo under the
 * floor on tail frames. */
static void lull_line_init(struct lull_line *l, unsigned rate, double dip_db,
			   double late_db)
{
	unsigned late = path_samples_get(LULL_LATE_MS, rate);

	memset(l, 0, sizeof(*l));
	l->rate = rate;
	l->frame = rate / 100u;
	l->pos = l->frame * 2u / 3u;
	l->dip_db = dip_db;
	l->far_state = 0x1234567u;
	l->floor_state = 0x2468aceu;
	l->talk_state = 0x5eed5eedu;
	path_flat_init(&l->path, rate, LULL_BULK_MS, LULL_ERL_DB);
	if (late_db > 0.0) {
		l->path.h[late] = (float)(l->path.h[l->path.taps - 1u] *
					  pow(10.0, -late_db / 20.0));
		l->path.taps = late + 1u;
	}
	l->ec = udsp_ec_nlms.create_rate(LULL_TAIL_MS, rate);
}

/* The phase of the next frame in its period; LULL_ONSET_PHASE is the frame a
 * burst starts in. */
static unsigned lull_phase_get(const struct lull_line *l)
{
	return (unsigned)(l->count % LULL_PERIOD_FRAMES);
}

/* The reference of the next frame: the far bursts when bursts is set, else
 * the far end throughout the frame. Returns whether the frame is silent. */
static bool lull_ref_fill(struct lull_line *l, bool bursts, int16_t *ref)
{
	unsigned long period = LULL_PERIOD_FRAMES * l->frame;
	unsigned long on = LULL_ON_FRAMES * l->frame;
	bool silent = true;
	unsigned i;

	noise_fill(ref, l->frame, LULL_FAR_DBM0, &l->far_state);
	for (i = 0; i < l->frame; i++) {
		if (!bursts || (l->pos + i) % period < on)
			silent = false;
		else
			ref[i] = 0;
	}
	l->pos += l->frame;
	l->count++;
	return silent;
}

/* One frame of the line: the reference of lull_ref_fill, and the near
 * talker at talk_db over the floor when talk is set. */
static void lull_frame_run(struct lull_line *l, bool bursts, bool talk,
			   double talk_db, struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t floor[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	bool silent = lull_ref_fill(l, bursts, ref);
	double floor_dbm0;
	unsigned i;

	path_run(&l->path, ref, echo);
	if (silent)
		l->quiet++;
	floor_dbm0 = LULL_FLOOR_DBM0;
	if (silent && l->quiet % LULL_DIP_EVERY == 0u)
		floor_dbm0 -= l->dip_db;
	noise_fill(floor, l->frame, floor_dbm0, &l->floor_state);
	noise_fill(near, l->frame, LULL_FLOOR_DBM0 + talk_db, &l->talk_state);
	for (i = 0; i < l->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + floor[i] +
				      (talk ? near[i] : 0));
	udsp_ec_nlms.run(l->ec, mic, ref);
	udsp_ec_nlms_frame_get(l->ec, reading);
	udsp_ec_nlms_info_get(l->ec, info);
}

/* One frame of the bursts through the reference intake alone, as the
 * engine feeds a skipped frame (udsp_ec_nlms_ref_feed). */
static void lull_frame_feed(struct lull_line *l)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];

	if (lull_ref_fill(l, true, ref))
		l->quiet++;
	path_run(&l->path, ref, echo);
	udsp_ec_nlms_ref_feed(l->ec, ref);
}

/* LULL_SETTLE_FRAMES of the far end throughout and LULL_BURSTS bursts, then
 * the bursts on up to the frame before phase. */
static void lull_line_settle(struct lull_line *l, unsigned phase,
			     struct udsp_ec_nlms_frame *reading,
			     struct udsp_ec_nlms_info *info)
{
	unsigned long end = LULL_SETTLE_FRAMES +
			    LULL_BURSTS * LULL_PERIOD_FRAMES;

	while (l->count < LULL_SETTLE_FRAMES)
		lull_frame_run(l, false, false, 0.0, reading, info);
	while (l->count < end || lull_phase_get(l) != phase)
		lull_frame_run(l, true, false, 0.0, reading, info);
}

struct lull_far_figures {
	unsigned onset;
	unsigned tail;
	unsigned bare;
	unsigned other;
	unsigned bursts;
	float noise_dbov;
};

/* The far end alone: after LULL_SETTLE_FRAMES, LULL_BURSTS bursts. The
 * detector's fires count from the first onset that reads a measured floor, by
 * the class of the frame: onset, tail, pre-echo and the rest. */
static void lull_far_run(unsigned rate, double late_db,
			 struct lull_far_figures *fig)
{
	struct udsp_ec_nlms_frame rd;
	struct udsp_ec_nlms_info info;
	struct lull_line l;
	bool counting = false;
	unsigned f;

	memset(fig, 0, sizeof(*fig));
	lull_line_init(&l, rate, LULL_DIP_DB, late_db);
	for (f = 0; f < LULL_SETTLE_FRAMES; f++)
		lull_frame_run(&l, false, false, 0.0, &rd, &info);
	for (f = 0; f < LULL_BURSTS * LULL_PERIOD_FRAMES; f++) {
		unsigned phase = lull_phase_get(&l);

		lull_frame_run(&l, true, false, 0.0, &rd, &info);
		if (phase == LULL_ONSET_PHASE && rd.noise_power > 0.0f)
			counting = true;
		fig->bursts += counting && phase == LULL_ONSET_PHASE;
		if (!counting || !rd.dt_fired)
			continue;
		if (phase == LULL_ONSET_PHASE)
			fig->onset++;
		else if (rd.ref_quiet)
			fig->tail++;
		else if (rd.estimate_power <
			 rd.noise_power * LULL_FLOOR_HEADROOM)
			fig->bare++;
		else
			fig->other++;
	}
	fig->noise_dbov = info.noise_dbov;
	udsp_ec_nlms.destroy(l.ec);
}

static void lull_far_print(const char *line,
			   const struct lull_far_figures *fig)
{
	printf("# %s: %u bursts, tracked floor -%.1f dBov; fires on the onset "
	       "frame %u, on tail frames %u, on pre-echo frames %u, on other "
	       "frames %u\n", line, fig->bursts, fig->noise_dbov, fig->onset,
	       fig->tail, fig->bare, fig->other);
}

/*
 * On the lull line, onset, tail and pre-echo frames carry the floor and no
 * echo, 8 dB over the tracked floor. Each takes the pause level as its floor
 * term (LEC_PAUSE_RING), so the far end alone fires on none. A rule without one
 * of the conditions, or a level of one quiet frame, fails.
 */
static void lull_far_test(unsigned rate)
{
	struct lull_far_figures flat;
	struct lull_far_figures late;

	lull_far_run(rate, 0.0, &flat);
	lull_far_run(rate, LULL_LATE_DB, &late);
	case_rate(rate, "lec far end alone does not fire on the onset, tail "
		  "and pre-echo frames of its bursts");
	lull_far_print("flat path", &flat);
	lull_far_print("with the late reflection", &late);
	CHECK(flat.bursts >= LULL_BURSTS - 1u);
	CHECK_UINT(flat.onset, 0);
	CHECK_UINT(flat.tail, 0);
	CHECK_UINT(flat.bare, 0);
	CHECK(late.bursts >= LULL_BURSTS - 1u);
	CHECK_UINT(late.onset, 0);
	CHECK_UINT(late.tail, 0);
	CHECK_UINT(late.bare, 0);
}

struct lull_talk_figures {
	float move;
	float energy;
	float over_db;
	bool fired;
};

/* A near talker at talk_db over the floor from lead frames before an onset
 * frame through LULL_AFTER_FRAMES after it. dip_db sets the dips of the
 * floor; old_db > 0 adds a talker at old_db through the pause two pauses
 * before the onset's own. */
static void lull_talk_run(unsigned rate, double dip_db, double talk_db,
			  unsigned lead, double old_db,
			  struct lull_talk_figures *fig)
{
	struct udsp_ec_nlms_frame rd;
	struct udsp_ec_nlms_info info;
	struct lull_line l;
	unsigned k;

	memset(fig, 0, sizeof(*fig));
	lull_line_init(&l, rate, dip_db, 0.0);
	lull_line_settle(&l, LULL_ON_FRAMES, &rd, &info);
	for (k = 0; old_db > 0.0 && k < 2u * LULL_PERIOD_FRAMES; k++)
		lull_frame_run(&l, true, k < LULL_ONSET_PHASE - LULL_ON_FRAMES,
			       old_db, &rd, &info);
	while (lull_phase_get(&l) !=
	       (LULL_ONSET_PHASE + LULL_PERIOD_FRAMES - lead) %
	       LULL_PERIOD_FRAMES)
		lull_frame_run(&l, true, false, 0.0, &rd, &info);
	for (k = 0; k < lead + LULL_AFTER_FRAMES; k++) {
		float before = info.coef_energy;

		lull_frame_run(&l, true, true, talk_db, &rd, &info);
		if (k == lead) {
			fig->energy = before;
			fig->move = info.coef_energy - before;
			fig->fired = rd.dt_fired;
			fig->over_db = (float)(10.0 * log10(rd.mic_power /
							   rd.bar));
		}
	}
	udsp_ec_nlms.destroy(l.ec);
}

/* A near talker LULL_TALK_DB over the floor starts on an onset frame or in the
 * pause before. That onset frame fires and does not adapt, because the cap of
 * LEC_PAUSE_LEVEL_CAP bounds the pause level. */
static void lull_talk_test(unsigned rate)
{
	struct lull_talk_figures on;
	struct lull_talk_figures lead;

	lull_talk_run(rate, LULL_DIP_DB, LULL_TALK_DB, 0u, 0.0, &on);
	lull_talk_run(rate, LULL_DIP_DB, LULL_TALK_DB, LULL_TALK_LEAD, 0.0,
		      &lead);
	case_rate(rate, "lec near talker over the floor fires on the onset "
		  "frame of a far burst and the frame does not adapt");
	printf("# talker %.0f dB over the floor from the onset frame: fired "
	       "%d, %+.1f dB over the bar, coefficient energy %+.2e of %.4f; "
	       "from %u frames before: fired %d, %+.1f dB, %+.2e of %.4f\n",
	       LULL_TALK_DB, on.fired, on.over_db, on.move, on.energy,
	       LULL_TALK_LEAD, lead.fired, lead.over_db, lead.move,
	       lead.energy);
	CHECK(on.fired);
	CHECK(on.move == 0.0f);
	CHECK(lead.fired);
	CHECK(lead.move == 0.0f);
}

/*
 * A near talker LULL_CAP_TALK_DB over the floor, from a pause's first frame
 * on into its burst. The pause's quiet-window frames carry that talker, so
 * the pause level reads it. With a cap, a bar's floor term stands 6 dB over
 * the tracked floor, so the onset frame fires and does not adapt. Without a
 * cap, that talker sets the bar and fails here.
 */
static void lull_cap_test(unsigned rate)
{
	struct lull_talk_figures fig;

	lull_talk_run(rate, LULL_DIP_DB, LULL_CAP_TALK_DB, LULL_CAP_TALK_LEAD,
		      0.0, &fig);
	case_rate(rate, "lec near talker through the far pause fires on the "
		  "onset frame under the capped pause level");
	printf("# talker %.0f dB over the floor from the first pause frame: "
	       "fired %d, %+.1f dB over the bar, coefficient energy %+.2e of "
	       "%.4f\n", LULL_CAP_TALK_DB, fig.fired, fig.over_db, fig.move,
	       fig.energy);
	CHECK(fig.fired);
	CHECK(fig.move == 0.0f);
}

/*
 * A talker speaks through one pause and stops; two pauses later a talker starts
 * on an onset frame. The pause level reads the last LEC_PAUSE_RING quiet
 * frames, which exclude the first talker, so that onset fires; a level over
 * eight frames fails.
 */
static void lull_ring_test(unsigned rate)
{
	struct lull_talk_figures fig;

	lull_talk_run(rate, 0.0, LULL_TALK_DB, 0u, LULL_OLD_TALK_DB, &fig);
	case_rate(rate, "lec talker two far pauses back does not lift the bar "
		  "of the next onset frame");
	printf("# talker %.0f dB over a flat floor through the pause two "
	       "before; talker %.0f dB from the onset frame: fired %d, %+.1f "
	       "dB over the bar, coefficient energy %+.2e of %.4f\n",
	       LULL_OLD_TALK_DB, LULL_TALK_DB, fig.fired, fig.over_db,
	       fig.move, fig.energy);
	CHECK(fig.fired);
	CHECK(fig.move == 0.0f);
}

struct lull_skip_figures {
	float term_ratio;
	float noise_dbov;
	float est_dbov;
	bool active;
	bool trusted;
	bool classic;
};

/*
 * A skipped frame takes the reference through udsp_ec_nlms_ref_feed alone. The
 * canceller runs a quiet frame and feeds the onset and the frames after it. It
 * runs again on an echo frame, whose bar gives the floor term
 * (doubletalk_update).
 */
static void lull_skip_run(unsigned rate, struct lull_skip_figures *fig)
{
	struct udsp_ec_nlms_frame rd;
	struct udsp_ec_nlms_info info;
	struct lull_line l;
	float misalign;
	float term;
	unsigned k;

	memset(fig, 0, sizeof(*fig));
	lull_line_init(&l, rate, LULL_DIP_DB, 0.0);
	lull_line_settle(&l, LULL_ONSET_PHASE, &rd, &info);
	for (k = 0; k < LULL_SKIP_FRAMES; k++)
		lull_frame_feed(&l);
	lull_frame_run(&l, true, false, 0.0, &rd, &info);
	misalign = rd.erle_db > 0.0f ? powf(10.0f, -rd.erle_db / 10.0f) :
				       1.0f;
	term = (rd.bar / LULL_MARGIN -
		rd.estimate_power * (1.0f + misalign)) / LULL_FLOOR_HEADROOM;
	fig->term_ratio = term / rd.noise_power;
	fig->noise_dbov = (float)(10.0 * log10(rd.noise_power / 536854528.0));
	fig->est_dbov = (float)(10.0 * log10(rd.estimate_power /
					     536854528.0));
	fig->active = rd.window_power >= rd.ref_bar && !rd.ref_quiet;
	fig->trusted = rd.trusted;
	fig->classic = rd.classic_arm;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The first frame that runs after a burst starts inside the reference intake is
 * no onset, as its previous frame's window is active. Its bar therefore carries
 * the tracked floor, not the pause level. A window flag that only run() keeps
 * reads an onset there and admits near speech; that form fails.
 */
static void lull_skip_test(unsigned rate)
{
	struct lull_skip_figures fig;

	lull_skip_run(rate, &fig);
	case_rate(rate, "lec frame after a burst start inside a reference-only "
		  "intake reads the tracked floor");
	printf("# %u frames through the intake from the onset frame; next "
	       "frame: estimate %.1f dBov, tracked floor %.1f dBov, floor term "
	       "of the bar %.2f times the tracked floor; trusted %d, classic "
	       "arm %d\n", LULL_SKIP_FRAMES, fig.est_dbov, fig.noise_dbov,
	       fig.term_ratio, fig.trusted, fig.classic);
	CHECK(fig.active);
	CHECK(fig.trusted);
	CHECK(!fig.classic);
	CHECK(fig.term_ratio < LULL_SKIP_RATIO);
}

/* The quiet-miss line (G.168 test 3C t3): a flat path returns the echo
 * MISS_DELAY_MS late. Its product with an independent noise stands a set level
 * under it, which the ERLE tracker reads. Bursts start inside a frame. */
#define MISS_DELAY_MS		28.5
#define MISS_ERL_DB		6.0
#define MISS_RES_DB		25.0
#define MISS_FAR_DBM0		-20.0
#define MISS_FLOOR_DBM0		-70.0
#define MISS_BURST_FRAMES	25u
#define MISS_QUIET_FRAMES	4u
#define MISS_SETTLE_FRAMES	(6u * FRAMES_PER_SECOND)
#define MISS_PHASES		8u
/* The floor of a pattern frame over the tracked floor: g under the bar, h
 * between the two bars, m over the bar under the bound. p and P stand over the
 * bound, under and over 12 dB over the floor; '-' is the line floor. */
#define MISS_GOOD_DB		4.5
#define MISS_HEAD_DB		5.3
#define MISS_OVER_DB		7.0
#define MISS_PEAK_DB		10.5
#define MISS_LOUD_DB		13.5
/* The bars of a quiet frame with no estimate and of an active window over the
 * tracked floor (headroom times margin). Then the peak bound over the bar. */
#define MISS_PAUSE_BAR		(2.8f * 1.41f)
#define MISS_ACTIVE_BAR		(2.0f * 1.41f)
#define MISS_PEAK_RATIO		2.0f
/* The first frames of a burst whose output the edge figure covers, with the
 * cross-fade frame. The bound of that figure against the engaged level. */
#define MISS_EDGE_FRAMES	8u
#define MISS_EDGE_TOL_DB	1.0
/* The ERLE of the resume and of the gate under it. Then the product levels at
 * which the tracker reads about 6 dB over the resume, and between the two. */
#define MISS_RESUME_DB		24.0
#define MISS_GATE_DB		18.0
#define MISS_RES_DEEP_DB	30.0
#define MISS_RES_LOW_DB		21.0

struct miss_line {
	struct echo_path path;
	struct udsp_ec *ec;
	unsigned frame;
	unsigned quiet;
	unsigned period;
	unsigned offset;
	unsigned long count;
	uint32_t far_state;
	uint32_t floor_state;
	uint32_t res_state;
	double res_db;
	double far_dbm0;
	int16_t out[UDSP_FRAME_MAX];
};

/* The line at the rate, with its bursts at far_dbm0 (MISS_FAR_DBM0) and offset
 * samples into their first frame. Its echo product stands res_db under the
 * echo, with the processor on and the default tail of the rate. Each pause
 * holds MISS_QUIET_FRAMES quiet frames (miss_line_quiet_set). */
static void miss_line_init(struct miss_line *l, unsigned rate,
			   unsigned offset, double res_db)
{
	unsigned delay = path_samples_get(MISS_DELAY_MS, rate);

	memset(l, 0, sizeof(*l));
	l->frame = rate / 100u;
	l->offset = offset;
	l->res_db = res_db;
	l->far_dbm0 = MISS_FAR_DBM0;
	l->far_state = 0x31415926u;
	l->floor_state = 0x27182818u;
	l->res_state = 0x16180339u;
	path_geometry_set(&l->path, rate);
	l->path.taps = delay + 1u;
	l->path.h[delay] = 1.0f;
	path_erl_set(&l->path, MISS_ERL_DB);
	l->ec = udsp_ec_nlms.create_rate(0u, rate);
	udsp_ec_nlms.nlp(l->ec, true);
	l->quiet = MISS_QUIET_FRAMES;
	l->period = MISS_BURST_FRAMES + window_frames_get(l->ec) +
		    l->quiet - 1u;
}

/* The pauses of the line hold quiet quiet frames, from the next period on.
 * The line takes it before it settles. */
static void miss_line_quiet_set(struct miss_line *l, unsigned quiet)
{
	l->quiet = quiet;
	l->period = MISS_BURST_FRAMES + window_frames_get(l->ec) + quiet - 1u;
}

/* The frame of the period that the next frame is, counted from 0. */
static unsigned miss_phase_get(const struct miss_line *l)
{
	return (unsigned)(l->count % l->period);
}

/* The level of a pattern frame over the tracked floor, or 0 for the line
 * floor. */
static double miss_level_get(char c)
{
	switch (c) {
	case 'g':
		return MISS_GOOD_DB;
	case 'h':
		return MISS_HEAD_DB;
	case 'm':
		return MISS_OVER_DB;
	case 'p':
		return MISS_PEAK_DB;
	case 'P':
		return MISS_LOUD_DB;
	}
	return 0.0;
}

/*
 * One frame of the line. A nonzero over_db scales the floor of the frame to
 * stand that far over the floor that the detector reads on the frame.
 */
static void miss_frame_run(struct miss_line *l, double over_db,
			   struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t floor[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned p = miss_phase_get(l);
	double res = pow(10.0, -l->res_db / 20.0) * sqrt(3.0);
	double gain = 1.0;
	unsigned i;

	noise_fill(ref, l->frame, l->far_dbm0, &l->far_state);
	for (i = 0; i < l->frame; i++) {
		if (p >= MISS_BURST_FRAMES || (p == 0u && i < l->offset))
			ref[i] = 0;
	}
	path_run(&l->path, ref, echo);
	noise_fill(floor, l->frame, MISS_FLOOR_DBM0, &l->floor_state);
	if (over_db != 0.0) {
		udsp_ec_nlms_info_get(l->ec, info);
		gain = sqrt(32767.0 * 32767.0 *
			    pow(10.0, (over_db - info->noise_dbov) / 10.0) /
			    udsp_ec_nlms_frame_power_get(floor, l->frame));
	}
	for (i = 0; i < l->frame; i++) {
		double unit = rng_next(&l->res_state) / 4294967295.0 * 2.0 -
			      1.0;

		mic[i] = sample_clamp((double)echo[i] * (1.0 + res * unit) +
				      gain * floor[i]);
	}
	udsp_ec_nlms.run(l->ec, mic, ref);
	memcpy(l->out, mic, l->frame * sizeof(*mic));
	udsp_ec_nlms_frame_get(l->ec, reading);
	udsp_ec_nlms_info_get(l->ec, info);
	l->count++;
}

/* MISS_SETTLE_FRAMES of bursts and pauses with the line floor, up to the
 * first pause frame of a period. */
static void miss_line_settle(struct miss_line *l)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;

	while (l->count < MISS_SETTLE_FRAMES ||
	       miss_phase_get(l) != MISS_BURST_FRAMES)
		miss_frame_run(l, 0.0, &reading, &info);
}

/*
 * The figures of one pause and the burst after it: its quiet frames and the
 * level ranges of each pattern class over the bar. Then the first acting frame
 * of the burst, whether it reads pass, and the waits. Last, edge_db (the first
 * frames' output against the engaged level), and the onset's verdict and ERLE.
 */
struct miss_leg {
	unsigned quiet;
	double hi_good;
	double lo_miss;
	double hi_miss;
	double lo_head;
	double hi_head;
	double lo_peak;
	double hi_peak;
	double lo_peak_floor;
	double hi_peak_floor;
	unsigned first;
	bool first_pass;
	unsigned waits;
	double edge_db;
	unsigned onset_verdict;
	double onset_erle_db;
};

/* The figures of a p or P frame: db over the bar, floor_db over the tracked
 * floor. */
static void miss_peak_keep(struct miss_leg *g, double db, double floor_db)
{
	if (db < g->lo_peak)
		g->lo_peak = db;
	if (db > g->hi_peak)
		g->hi_peak = db;
	if (floor_db < g->lo_peak_floor)
		g->lo_peak_floor = floor_db;
	if (floor_db > g->hi_peak_floor)
		g->hi_peak_floor = floor_db;
}

/* The pause of the period from the frame the line stands on, with pattern on
 * its quiet frames, then the burst of the next period. */
static void miss_leg_run(struct miss_line *l, const char *pattern,
			 struct miss_leg *g)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned from = l->period - l->quiet;
	double edge = 0.0;
	double engaged = 0.0;
	uint32_t waits;
	unsigned f;
	unsigned i;

	memset(g, 0, sizeof(*g));
	g->hi_good = -200.0;
	g->lo_miss = 200.0;
	g->hi_miss = -200.0;
	g->lo_head = 200.0;
	g->hi_head = -200.0;
	g->lo_peak = 200.0;
	g->hi_peak = -200.0;
	g->lo_peak_floor = 200.0;
	g->hi_peak_floor = -200.0;
	g->first = MISS_BURST_FRAMES;
	for (f = miss_phase_get(l); f < l->period; f++) {
		char c = f >= from ? pattern[f - from] : '-';
		double db;

		miss_frame_run(l, miss_level_get(c), &reading, &info);
		g->quiet += reading.nlp_verdict == LECR_VERDICT_QUIET;
		db = ratio_db_get(reading.mic_power,
				  reading.estimate_power +
				  reading.noise_power * MISS_PAUSE_BAR);
		if (c == 'g' && db > g->hi_good)
			g->hi_good = db;
		if (c == 'm' && db < g->lo_miss)
			g->lo_miss = db;
		if (c == 'm' && db > g->hi_miss)
			g->hi_miss = db;
		if (c == 'h' && db < g->lo_head)
			g->lo_head = db;
		if (c == 'h' && db > g->hi_head)
			g->hi_head = db;
		if (c == 'p' || c == 'P')
			miss_peak_keep(g, db,
				       ratio_db_get(reading.mic_power,
						    reading.noise_power));
	}
	waits = info.nlp_wait_frames;
	for (f = 0; f < MISS_BURST_FRAMES; f++) {
		miss_frame_run(l, 0.0, &reading, &info);
		if (f == 0u) {
			g->onset_verdict = reading.nlp_verdict;
			g->onset_erle_db = reading.erle_db;
		}
		if (info.nlp_active && g->first == MISS_BURST_FRAMES) {
			g->first = f;
			g->first_pass = reading.nlp_verdict ==
					LECR_VERDICT_PASS;
		}
		for (i = 0; f < 2u * MISS_EDGE_FRAMES && i < l->frame; i++) {
			double e = (double)l->out[i] * l->out[i];

			if (f < MISS_EDGE_FRAMES)
				edge += e;
			else
				engaged += e;
		}
	}
	g->waits = info.nlp_wait_frames - waits;
	g->edge_db = 10.0 * log10(edge / engaged);
}

/*
 * The legs of one line, in order: the line floor, then the captured patterns
 * miss, miss, good and good, miss, miss. Then a pause that starts with two
 * misses after one that ends with two. Last come a miss, a kept frame and two
 * misses, then three misses, and one frame over the bound.
 */
enum {
	MISS_LEG_FLOOR,
	MISS_LEG_PAIR_GOOD,
	MISS_LEG_GOOD_PAIR,
	MISS_LEG_PAIR_AGAIN,
	MISS_LEG_SPLIT,
	MISS_LEG_FLOOR_2,
	MISS_LEG_THREE,
	MISS_LEG_FLOOR_3,
	MISS_LEG_PEAK,
	MISS_LEGS
};

static const char *const miss_patterns[MISS_LEGS] = {
	"----", "-mmg", "-gmm", "mmgg", "mgmm", "----", "-mmm", "----", "-gpg",
};

/* All legs of one line, bursts offset samples into their first frame, with
 * its echo product res_db under the echo. */
static void miss_line_run(unsigned rate, unsigned offset, double res_db,
			  struct miss_leg *legs)
{
	struct miss_line l;
	unsigned n;

	miss_line_init(&l, rate, offset, res_db);
	miss_line_settle(&l);
	for (n = 0; n < MISS_LEGS; n++)
		miss_leg_run(&l, miss_patterns[n], &legs[n]);
	udsp_ec_nlms.destroy(l.ec);
}

/* The legs of every phase on a line with its echo product res_db under the
 * echo: the offset of phase k is k eighths of a frame. */
static void miss_phases_run(unsigned rate, double res_db,
			    struct miss_leg legs[MISS_PHASES][MISS_LEGS])
{
	unsigned frame = rate / 100u;
	unsigned k;

	for (k = 0; k < MISS_PHASES; k++)
		miss_line_run(rate, k * frame / MISS_PHASES, res_db, legs[k]);
}

/* The checks of a leg that keeps its run:
 * - its pattern frames stand where their pattern puts them;
 * - the whole pause reads quiet;
 * - the burst acts from its first frame, a pass frame, with no wait;
 * - its edge stands within MISS_EDGE_TOL_DB of the engaged level. */
static void miss_kept_check(const struct miss_leg *g, const char *pattern)
{
	CHECK_UINT(g->quiet, MISS_QUIET_FRAMES);
	if (strchr(pattern, 'g'))
		CHECK(g->hi_good < 0.0);
	if (strchr(pattern, 'm')) {
		CHECK(g->lo_miss > 0.0);
		CHECK(g->hi_miss < 10.0 * log10(MISS_PEAK_RATIO));
	}
	CHECK_UINT(g->first, 0);
	CHECK(g->first_pass);
	CHECK_UINT(g->waits, 0);
	CHECK(fabs(g->edge_db) <= MISS_EDGE_TOL_DB);
}

/* The figures of leg over every phase:
 * - the largest edge figure;
 * - the range of the miss frames over the bar;
 * - the first acting frame and the waits of the phase that waits most. */
static void miss_legs_print(const char *what,
			    struct miss_leg legs[MISS_PHASES][MISS_LEGS],
			    unsigned leg)
{
	double edge = -200.0;
	double lo = 200.0;
	double hi = -200.0;
	double good = -200.0;
	unsigned first = 0;
	unsigned waits = 0;
	unsigned k;

	for (k = 0; k < MISS_PHASES; k++) {
		const struct miss_leg *g = &legs[k][leg];

		if (g->edge_db > edge)
			edge = g->edge_db;
		if (g->lo_miss < lo)
			lo = g->lo_miss;
		if (g->hi_miss > hi)
			hi = g->hi_miss;
		if (g->hi_good > good)
			good = g->hi_good;
		if (g->first > first)
			first = g->first;
		if (g->waits > waits)
			waits = g->waits;
	}
	printf("# %s, pause %s over %u phases: good frames up to %.2f dB "
	       "over the bar, miss frames %.2f to %.2f dB over it; the burst "
	       "acts from its frame %u at most, %u wait frames at most; its "
	       "first %u frames up to %.2f dB over the engaged level\n", what,
	       miss_patterns[leg], MISS_PHASES, good, lo, hi, first, waits,
	       MISS_EDGE_FRAMES, edge);
}

/*
 * Captured patterns of G.168 test 3C t3 at 8 phases: two quiet misses in a row
 * keep the run (LEC_NLP_QUIET_MISSES). The burst then acts on its first frame,
 * and its first 8 frames stay within 1 dB of the engaged level. A count that a
 * far burst or a kept frame does not reset fails.
 */
static void quiet_miss_test(unsigned rate)
{
	static struct miss_leg legs[MISS_PHASES][MISS_LEGS];
	unsigned k;

	miss_phases_run(rate, MISS_RES_DB, legs);
	case_rate(rate, "lec nlp keeps the engage run through a far pause of "
		  "miss, miss, good");
	miss_legs_print("line floor", legs, MISS_LEG_FLOOR);
	miss_legs_print("miss, miss, good", legs, MISS_LEG_PAIR_GOOD);
	miss_legs_print("the same after a pause that ends with two misses",
			legs, MISS_LEG_PAIR_AGAIN);
	for (k = 0; k < MISS_PHASES; k++) {
		miss_kept_check(&legs[k][MISS_LEG_FLOOR],
				miss_patterns[MISS_LEG_FLOOR]);
		miss_kept_check(&legs[k][MISS_LEG_PAIR_GOOD],
				miss_patterns[MISS_LEG_PAIR_GOOD]);
		miss_kept_check(&legs[k][MISS_LEG_PAIR_AGAIN],
				miss_patterns[MISS_LEG_PAIR_AGAIN]);
	}
	case_rate(rate, "lec nlp keeps the engage run through a far pause of "
		  "good, miss, miss");
	miss_legs_print("good, miss, miss", legs, MISS_LEG_GOOD_PAIR);
	for (k = 0; k < MISS_PHASES; k++)
		miss_kept_check(&legs[k][MISS_LEG_GOOD_PAIR],
				miss_patterns[MISS_LEG_GOOD_PAIR]);
	case_rate(rate, "lec nlp keeps the engage run through a far pause of "
		  "miss, good, miss, miss");
	miss_legs_print("miss, good, miss, miss", legs, MISS_LEG_SPLIT);
	for (k = 0; k < MISS_PHASES; k++)
		miss_kept_check(&legs[k][MISS_LEG_SPLIT],
				miss_patterns[MISS_LEG_SPLIT]);
}

/* The range of the ERLE tracker on the first frame of the burst of leg, over
 * every phase. */
static void miss_onset_erle_get(struct miss_leg legs[MISS_PHASES][MISS_LEGS],
				unsigned leg, double *lo, double *hi)
{
	unsigned k;

	*lo = 200.0;
	*hi = -200.0;
	for (k = 0; k < MISS_PHASES; k++) {
		if (legs[k][leg].onset_erle_db < *lo)
			*lo = legs[k][leg].onset_erle_db;
		if (legs[k][leg].onset_erle_db > *hi)
			*hi = legs[k][leg].onset_erle_db;
	}
}

/*
 * Three quiet misses in a row, and one frame over the peak bound, at 8 phases.
 * After the three misses the first pass frame resumes the run
 * (LEC_NLP_RESUME_DB); after the frame over the bound the burst waits 7 frames.
 */
static void quiet_miss_limit_test(unsigned rate)
{
	static struct miss_leg legs[MISS_PHASES][MISS_LEGS];
	double lo;
	double hi;
	unsigned k;

	miss_phases_run(rate, MISS_RES_DEEP_DB, legs);
	case_rate(rate, "lec nlp resumes the engage run at a far onset after "
		  "three quiet misses and waits after one over the peak bound");
	miss_legs_print("three misses", legs, MISS_LEG_THREE);
	miss_legs_print("one frame over the bound", legs, MISS_LEG_PEAK);
	miss_onset_erle_get(legs, MISS_LEG_THREE, &lo, &hi);
	printf("# ERLE tracker on the first burst frame after three misses: "
	       "%.2f to %.2f dB\n", lo, hi);
	for (k = 0; k < MISS_PHASES; k++) {
		const struct miss_leg *three = &legs[k][MISS_LEG_THREE];
		const struct miss_leg *peak = &legs[k][MISS_LEG_PEAK];

		miss_kept_check(three, miss_patterns[MISS_LEG_THREE]);
		CHECK(three->onset_erle_db >= MISS_RESUME_DB);
		CHECK_UINT(peak->quiet, MISS_QUIET_FRAMES);
		CHECK(peak->hi_good < 0.0);
		CHECK_UINT(peak->first, WORDS_ENGAGE_FRAME);
		CHECK_UINT(peak->waits, WORDS_ENGAGE_FRAME);
		CHECK(peak->edge_db > MISS_EDGE_TOL_DB);
	}
}

/* The legs of one line whose echo product stands res_db under the echo:
 * the line floor, then three misses. */
static void miss_three_run(unsigned rate, unsigned offset, double res_db,
			   struct miss_leg *legs)
{
	struct miss_line l;

	miss_line_init(&l, rate, offset, res_db);
	miss_line_settle(&l);
	miss_leg_run(&l, miss_patterns[MISS_LEG_FLOOR], &legs[MISS_LEG_FLOOR]);
	miss_leg_run(&l, miss_patterns[MISS_LEG_THREE], &legs[MISS_LEG_THREE]);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * Three quiet misses on the quiet-miss line whose ERLE tracker reads over the
 * gate and under LEC_NLP_RESUME_DB of dsp/lec.c. No resume follows, so the
 * burst waits 7 frames. A resume without the ERLE condition, or with it at
 * the gate, fails here.
 */
static void resume_erle_test(unsigned rate)
{
	static struct miss_leg legs[MISS_PHASES][MISS_LEGS];
	unsigned frame = rate / 100u;
	double lo;
	double hi;
	unsigned k;

	for (k = 0; k < MISS_PHASES; k++)
		miss_three_run(rate, k * frame / MISS_PHASES, MISS_RES_LOW_DB,
			       legs[k]);
	case_rate(rate, "lec nlp waits at a far onset after three quiet misses "
		  "with the ERLE tracker under the resume level");
	miss_legs_print("three misses", legs, MISS_LEG_THREE);
	miss_onset_erle_get(legs, MISS_LEG_THREE, &lo, &hi);
	printf("# ERLE tracker on the first burst frame: %.2f to %.2f dB\n",
	       lo, hi);
	for (k = 0; k < MISS_PHASES; k++) {
		const struct miss_leg *three = &legs[k][MISS_LEG_THREE];

		CHECK_UINT(three->quiet, MISS_QUIET_FRAMES);
		CHECK(three->lo_miss > 0.0);
		CHECK(three->hi_miss < 10.0 * log10(MISS_PEAK_RATIO));
		CHECK_UINT(three->onset_verdict, LECR_VERDICT_PASS);
		CHECK(three->onset_erle_db >= MISS_GATE_DB);
		CHECK(three->onset_erle_db < MISS_RESUME_DB);
		CHECK_UINT(three->first, WORDS_ENGAGE_FRAME);
		CHECK_UINT(three->waits, WORDS_ENGAGE_FRAME);
		CHECK(three->edge_db > MISS_EDGE_TOL_DB);
	}
}

/* The near sound of resume_talker_test: its level over the tracked floor on
 * the first MISS_TALK_FRAMES frames of a burst, about 2 dB over the echo. */
#define MISS_TALK_DB		46.0
#define MISS_TALK_FRAMES	8u

/*
 * A near sound that holds a burst, then the pause after it. The sound stands
 * MISS_TALK_DB over the tracked floor on the first MISS_TALK_FRAMES frames of
 * the burst and MISS_OVER_DB over it on the rest. It goes on at that level on
 * the quiet frames of the pause, and the burst after the pause follows. acts
 * counts the frames of the held burst that read nlp_active.
 */
static void miss_talk_leg_run(struct miss_line *l, struct miss_leg *g,
			      unsigned *acts)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned f;

	for (f = MISS_BURST_FRAMES; f < l->period; f++)
		miss_frame_run(l, 0.0, &reading, &info);
	*acts = 0;
	for (f = 0; f < MISS_BURST_FRAMES; f++) {
		miss_frame_run(l, f < MISS_TALK_FRAMES ? MISS_TALK_DB :
			       MISS_OVER_DB, &reading, &info);
		*acts += info.nlp_active;
	}
	miss_leg_run(l, "mmmm", g);
}

/*
 * A near sound that the detector holds over a far burst goes on in the far
 * pause after it, under the bound. The processor acts on no frame of that
 * burst, so no resume follows the pause, and the next burst waits 7 frames.
 * A resume without the acted burst fails here.
 */
static void resume_talker_test(unsigned rate)
{
	unsigned frame = rate / 100u;
	unsigned k;

	case_rate(rate, "lec nlp waits at a far onset after a near sound that "
		  "held the burst before the pause");
	for (k = 0; k < MISS_PHASES; k++) {
		struct miss_leg floor;
		struct miss_leg g;
		struct miss_line l;
		unsigned acts;

		miss_line_init(&l, rate, k * frame / MISS_PHASES,
			       MISS_RES_DEEP_DB);
		miss_line_settle(&l);
		miss_leg_run(&l, miss_patterns[MISS_LEG_FLOOR], &floor);
		miss_talk_leg_run(&l, &g, &acts);
		udsp_ec_nlms.destroy(l.ec);
		printf("# phase %u: held burst %u acting frames; pause frames "
		       "%.2f to %.2f dB over the bar; next burst: verdict %s "
		       "and ERLE tracker %.2f dB on its first frame, acts "
		       "from its frame %u, %u wait frames\n", k, acts,
		       g.lo_miss, g.hi_miss,
		       udsp_ec_nlms_verdict_name(g.onset_verdict),
		       g.onset_erle_db, g.first, g.waits);
		CHECK_UINT(acts, 0);
		CHECK_UINT(g.quiet, MISS_QUIET_FRAMES);
		CHECK(g.hi_miss < 10.0 * log10(MISS_PEAK_RATIO));
		CHECK_UINT(g.onset_verdict, LECR_VERDICT_PASS);
		CHECK(g.onset_erle_db >= MISS_RESUME_DB);
		CHECK_UINT(g.first, WORDS_ENGAGE_FRAME);
		CHECK_UINT(g.waits, WORDS_ENGAGE_FRAME);
	}
}

/* The far end in the pause of resume_floor_test, under the active bar of the
 * reference, and the room noise at the level of its echo. */
#define STATE_LOW_FAR_DBM0	-65.0
#define STATE_LOW_ROOM_DBM0	-78.5
/* The bar of a quiet frame with no measured floor over the estimate,
 * LEC_GEIGEL_MARGIN of dsp/lec.c. */
#define STATE_LIKE_MARGIN	1.41f

/* One pause frame of resume_floor_test: the far end at STATE_LOW_FAR_DBM0,
 * with the room noise at STATE_LOW_ROOM_DBM0 when room is set. */
static void state_low_frame_run(struct state_line *s, bool room,
				struct udsp_ec_nlms_frame *reading)
{
	int16_t ref[UDSP_FRAME_MAX];

	noise_fill(ref, s->path.frame, STATE_LOW_FAR_DBM0, &s->far_state);
	memset(s->room, 0, sizeof(s->room));
	if (room)
		noise_fill(s->room, s->path.frame, STATE_LOW_ROOM_DBM0,
			   &s->room_state);
	verdict_out_step(s->ec, &s->path, ref, room ? s->room : NULL, s->out,
			 &s->acted);
	udsp_ec_nlms_frame_get(s->ec, reading);
}

/*
 * The first pause of the state line, with no measured floor. Room noise at
 * the level of the far echo on the first quiet frame zeroes the engage run.
 * No resume follows a pause frame without a floor, so the far return acts
 * on its 8th frame. A resume without that condition fails here.
 */
static void resume_floor_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_frame first;
	struct udsp_ec_nlms_info info;
	struct state_line s;
	unsigned tail;
	unsigned at;
	unsigned f;
	bool engaged;

	state_line_init(&s, rate, 8u);
	udsp_ec_nlms_info_get(s.ec, &info);
	engaged = info.nlp_active;
	tail = window_frames_get(s.ec) - 1u;
	memset(&first, 0, sizeof(first));
	for (f = 0; f < STATE_PAUSE_FRAMES; f++) {
		state_low_frame_run(&s, f >= tail, &reading);
		if (f == tail)
			first = reading;
	}
	at = state_far_run(&s, false);
	udsp_ec_nlms_frame_get(s.ec, &reading);
	case_rate(rate, "lec nlp waits at a far return after a pause frame "
		  "between the bar and the bound with no measured floor");
	printf("# engaged before the pause %d; first quiet frame: verdict %s, "
	       "floor %.3g, %.2f dB over its bar, bound %.2f dB; ERLE tracker "
	       "%.2f dB; the far return acts from its frame %u\n", engaged,
	       udsp_ec_nlms_verdict_name(first.nlp_verdict),
	       (double)first.noise_power,
	       ratio_db_get(first.mic_power,
			    first.estimate_power * STATE_LIKE_MARGIN),
	       10.0 * log10(MISS_PEAK_RATIO), reading.erle_db, at);
	CHECK(engaged);
	CHECK_UINT(first.nlp_verdict, LECR_VERDICT_QUIET);
	CHECK(first.noise_power == 0.0f);
	CHECK(first.mic_power > first.estimate_power * STATE_LIKE_MARGIN);
	CHECK(first.mic_power <= first.estimate_power * STATE_LIKE_MARGIN *
				 MISS_PEAK_RATIO);
	CHECK(reading.erle_db >= MISS_RESUME_DB);
	CHECK_UINT(at, WORDS_ENGAGE_FRAME);
	udsp_ec_nlms.destroy(s.ec);
}

/* The patterns of quiet_miss_low_test. Each follows a pause of the line floor,
 * so its first quiet frame carries the line floor. */
static const char *const miss_low_patterns[] = { "-mmg", "-gmm" };
#define MISS_LOW_LEGS		2u

/* The legs of every phase on the quiet-miss line with its echo product
 * MISS_RES_LOW_DB under the echo: each of count patterns after a pause of the
 * line floor. */
static void miss_low_run(unsigned rate, const char *const *patterns,
			 unsigned count,
			 struct miss_leg legs[MISS_PHASES][MISS_LOW_LEGS])
{
	unsigned frame = rate / 100u;
	unsigned k;
	unsigned n;

	for (k = 0; k < MISS_PHASES; k++) {
		struct miss_leg floor;
		struct miss_line l;

		miss_line_init(&l, rate, k * frame / MISS_PHASES,
			       MISS_RES_LOW_DB);
		miss_line_settle(&l);
		for (n = 0; n < count; n++) {
			miss_leg_run(&l, miss_patterns[MISS_LEG_FLOOR], &floor);
			miss_leg_run(&l, patterns[n], &legs[k][n]);
		}
		udsp_ec_nlms.destroy(l.ec);
	}
}

/*
 * The patterns miss, miss, good and good, miss, miss on the quiet-miss line.
 * Its ERLE tracker reads under LEC_NLP_RESUME_DB of dsp/lec.c, so a zeroed
 * run waits. Two quiet misses in a row keep the run (LEC_NLP_QUIET_MISSES),
 * and the burst acts on its first frame. A count of 1 fails here.
 */
static void quiet_miss_low_test(unsigned rate)
{
	static struct miss_leg legs[MISS_PHASES][MISS_LOW_LEGS];
	unsigned k;
	unsigned n;

	miss_low_run(rate, miss_low_patterns, MISS_LOW_LEGS, legs);
	case_rate(rate, "lec nlp keeps the engage run through two quiet misses "
		  "on a line under the resume level");
	for (n = 0; n < MISS_LOW_LEGS; n++) {
		double lo = 200.0;
		double hi = -200.0;
		double good = -200.0;
		double edge = -200.0;
		double erle = 200.0;
		unsigned first = 0;

		for (k = 0; k < MISS_PHASES; k++) {
			const struct miss_leg *g = &legs[k][n];

			lo = g->lo_miss < lo ? g->lo_miss : lo;
			hi = g->hi_miss > hi ? g->hi_miss : hi;
			good = g->hi_good > good ? g->hi_good : good;
			edge = g->edge_db > edge ? g->edge_db : edge;
			erle = g->onset_erle_db < erle ? g->onset_erle_db :
							erle;
			first = g->first > first ? g->first : first;
			miss_kept_check(g, miss_low_patterns[n]);
			CHECK(g->onset_erle_db < MISS_RESUME_DB);
		}
		printf("# pause %s over %u phases: good frames up to %.2f dB "
		       "over the bar, miss frames %.2f to %.2f dB over it; the "
		       "burst acts from its frame %u at most, its first %u "
		       "frames up to %.2f dB over the engaged level; ERLE "
		       "tracker from %.2f dB\n", miss_low_patterns[n],
		       MISS_PHASES, good, lo, hi, first, MISS_EDGE_FRAMES,
		       edge, erle);
	}
}

/*
 * A pause of the line floor after an acted burst. The processor is then off
 * from the next burst through the frames of the window after it. It is on
 * again for the quiet frames of that pause and the burst after them.
 * acts counts the frames of the switched-off burst that read nlp_active.
 */
static void miss_off_leg_run(struct miss_line *l, struct miss_leg *g,
			     unsigned *acts)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned from = l->period - l->quiet;
	unsigned f;

	for (f = MISS_BURST_FRAMES; f < l->period; f++)
		miss_frame_run(l, 0.0, &reading, &info);
	udsp_ec_nlms.nlp(l->ec, false);
	*acts = 0;
	for (f = 0; f < from; f++) {
		miss_frame_run(l, 0.0, &reading, &info);
		*acts += info.nlp_active;
	}
	udsp_ec_nlms.nlp(l->ec, true);
	miss_leg_run(l, "----", g);
}

/* The processor switch: a frame with the switch off clears the evidence of the
 * resume. A burst before the pause ran with the switch off, so the burst after
 * the pause waits 7 frames. */
static void resume_off_test(unsigned rate)
{
	unsigned frame = rate / 100u;
	unsigned k;

	case_rate(rate, "lec nlp waits at the first far onset after the "
		  "processor switch returns");
	for (k = 0; k < MISS_PHASES; k++) {
		struct miss_leg floor;
		struct miss_leg g;
		struct miss_line l;
		unsigned acts;

		miss_line_init(&l, rate, k * frame / MISS_PHASES,
			       MISS_RES_DEEP_DB);
		miss_line_settle(&l);
		miss_leg_run(&l, miss_patterns[MISS_LEG_FLOOR], &floor);
		miss_off_leg_run(&l, &g, &acts);
		udsp_ec_nlms.destroy(l.ec);
		printf("# phase %u: burst before the switch acts from its "
		       "frame %u; switched-off burst %u acting frames; next "
		       "burst: verdict %s and ERLE tracker %.2f dB on its "
		       "first frame, acts from its frame %u, %u wait frames\n",
		       k, floor.first, acts,
		       udsp_ec_nlms_verdict_name(g.onset_verdict),
		       g.onset_erle_db, g.first, g.waits);
		CHECK_UINT(floor.first, 0);
		CHECK_UINT(acts, 0);
		CHECK_UINT(g.quiet, MISS_QUIET_FRAMES);
		CHECK_UINT(g.onset_verdict, LECR_VERDICT_PASS);
		CHECK(g.onset_erle_db >= MISS_RESUME_DB);
		CHECK_UINT(g.first, WORDS_ENGAGE_FRAME);
		CHECK_UINT(g.waits, WORDS_ENGAGE_FRAME);
	}
}

/* The pattern of quiet_headroom_low_test: three frames between the bars. */
static const char *const miss_head_patterns[] = { "-hhh" };

/*
 * Three quiet frames between the bar of an active window and the bar of a
 * quiet frame (LEC_GEIGEL_PAUSE_HEADROOM), on the quiet-miss line. Its ERLE
 * tracker reads under LEC_NLP_RESUME_DB. The frames keep the run, and the burst
 * acts on its first frame. A pause headroom equal to the active one makes
 * them misses that zero the run, and fails here.
 */
static void quiet_headroom_low_test(unsigned rate)
{
	static struct miss_leg legs[MISS_PHASES][MISS_LOW_LEGS];
	double gap = 10.0 * log10(MISS_PAUSE_BAR / MISS_ACTIVE_BAR);
	double lo = 200.0;
	double hi = -200.0;
	double edge = -200.0;
	double erle = 200.0;
	unsigned first = 0;
	unsigned k;

	miss_low_run(rate, miss_head_patterns, 1u, legs);
	case_rate(rate, "lec nlp keeps the engage run through quiet frames "
		  "between the bars of an active window and of a pause");
	for (k = 0; k < MISS_PHASES; k++) {
		const struct miss_leg *g = &legs[k][0];

		lo = g->lo_head < lo ? g->lo_head : lo;
		hi = g->hi_head > hi ? g->hi_head : hi;
		edge = g->edge_db > edge ? g->edge_db : edge;
		erle = g->onset_erle_db < erle ? g->onset_erle_db : erle;
		first = g->first > first ? g->first : first;
		CHECK(g->hi_head < 0.0);
		CHECK(g->lo_head + gap > 0.0);
		CHECK(g->onset_erle_db < MISS_RESUME_DB);
		miss_kept_check(g, miss_head_patterns[0]);
	}
	printf("# pause %s over %u phases: frames %.2f to %.2f dB over the "
	       "bar of a quiet frame, %.2f to %.2f dB over the bar of an "
	       "active window; the burst acts from its frame %u at most, its "
	       "first %u frames up to %.2f dB over the engaged level; ERLE "
	       "tracker from %.2f dB\n", miss_head_patterns[0], MISS_PHASES,
	       lo, hi, lo + gap, hi + gap, first, MISS_EDGE_FRAMES, edge,
	       erle);
}

/*
 * The soft near word of floor_peak_talker_test, its pause frames by pattern:
 * - o: at ALT_IN_DB over the tracked floor, over the bar and under the bound;
 * - u: at ALT_UNDER_DB, under the bar.
 */
static void quiet_word_run(struct words_run *r, unsigned rate,
			   const char *pattern)
{
	words_quiet_set(r, rate);
	r->near_word = WORDS_MARK;
	r->near_from = -(int)ALT_LEAD_FRAMES;
	r->near_frames = ALT_LEAD_FRAMES + WORDS_NEAR_FRAMES;
	r->near_dbm0 = FAN_FLOOR_DBM0 + ALT_WORD_OVER_DB;
	r->alt_over_db = ALT_IN_DB;
	r->alt_under_db = ALT_UNDER_DB;
	r->alt_pattern = pattern;
	words_run(r);
}

/* The bound of the word of quiet_miss_talker_test against the word whose
 * pause frames alternate over and under the bar. */
#define QUIET_WORD_TOL_DB	0.5

/*
 * Exposure of LEC_NLP_QUIET_MISSES to the soft word of floor_peak_talker_test
 * with pause frames good, miss, miss, good. It keeps the run, and the processor
 * replaces its 6 frames in the far word. The word loses at most
 * QUIET_WORD_TOL_DB more than a word that alternates over and under the bar.
 */
static void quiet_miss_talker_test(unsigned rate)
{
	struct words_run alt;
	struct words_run pair;
	struct words_run three;

	quiet_word_run(&alt, rate, "uouo");
	quiet_word_run(&pair, rate, "uoou");
	quiet_word_run(&three, rate, "ooou");
	case_rate(rate, "lec nlp exposure of a soft near word with two "
		  "frames in a row over the bar in a far pause");
	alt_word_print("over and under the bar in turn", &alt);
	alt_word_print("good, miss, miss, good", &pair);
	alt_word_print("three frames over the bar", &three);
	CHECK(pair.near > 0.0);
	CHECK(three.near > 0.0);
	CHECK_UINT(pair.near_pause, ALT_LEAD_FRAMES);
	CHECK_UINT(three.near_pause, ALT_LEAD_FRAMES);
	CHECK(pair.near_pause_hi > ALT_IN_DB - 1.5);
	CHECK(pair.near_pause_hi < ALT_EDGE_DB);
	CHECK_UINT(pair.near_acted, WORDS_NEAR_FRAMES);
	CHECK_UINT(pair.latency[WORDS_MARK], 0);
	CHECK(near_gain_db_get(pair.overlap_on, pair.overlap) >=
	      near_gain_db_get(alt.overlap_on, alt.overlap) -
	      QUIET_WORD_TOL_DB);
	CHECK_UINT(three.near_acted, 0);
	CHECK_UINT(three.latency[WORDS_MARK], WORDS_ENGAGE_FRAME);
}

/* The tone line: a far dial tone at TONE_DIAL_DBM0 over a white floor, PCMU, a
 * flat TONE_ERL_DB hybrid, TONE_LOSS_HALF_DB of loss, the processor on. After
 * TONE_WARM_FRAMES each trial adds five near digits at TONE_DIGIT_DBM0. */
#define TONE_LO_HZ		350.0
#define TONE_HI_HZ		440.0
#define TONE_DIAL_DBM0		-18.0
#define TONE_FLOOR_DBM0		-35.0
#define TONE_DIGIT_DBM0		-12.0
#define TONE_LEAD_US		375u
#define TONE_ERL_DB		6.0
#define TONE_LOSS_HALF_DB	18u
#define TONE_WARM_FRAMES	(2u * FRAMES_PER_SECOND)
#define TONE_TRIAL_FRAMES	150u
#define TONE_ON_FRAMES		100u
#define TONE_DIGIT_FROM		45u
#define TONE_DIGIT_PERIOD	15u
#define TONE_DIGIT_ON		5u
#define TONE_DIGITS		5u
#define TONE_TRIALS		20u
#define TONE_LOSS_DB		1.0

struct tone_result {
	unsigned arms;
	unsigned run_max;
	unsigned digits;
	double worst_db;
};

/* One sample through a G.711 mu-law round trip: the segment law of
 * udsp_ulaw_encode and udsp_ulaw_decode in dsp/g711.c. */
static int16_t tone_ulaw_get(int16_t sample)
{
	int magnitude = sample;
	unsigned exponent = 0;
	unsigned code;

	if (magnitude < 0)
		magnitude = -magnitude;
	if (magnitude > 32635)
		magnitude = 32635;
	magnitude += 0x84;
	while (exponent < 7 && magnitude >= (0x100 << exponent))
		exponent++;
	code = (exponent << 4) | (((unsigned)magnitude >> (exponent + 3)) &
				  0x0fu);
	magnitude = (int)(((code & 0x0fu) << 3) + 0x84u) << (code >> 4);
	return sample < 0 ? (int16_t)(0x84 - magnitude) :
			    (int16_t)(magnitude - 0x84);
}

/* The power of x at hz over one frame, a single-bin DFT. */
static double tone_bin_get(const int16_t *x, unsigned n, double hz,
			   unsigned rate)
{
	double re = 0.0;
	double im = 0.0;
	unsigned i;

	for (i = 0; i < n; i++) {
		double w = 2.0 * M_PI * hz * (double)i / rate;

		re += x[i] * cos(w);
		im -= x[i] * sin(w);
	}
	return (re * re + im * im) / ((double)n * n);
}

/* One frame of the near DTMF digits of a trial: digit k of the line takes
 * row k % 4 and column k / 4 % 4. Returns the digit, or -1 off a digit. */
static int tone_digit_fill(int16_t *near, unsigned frame, unsigned rate,
			   unsigned trial, unsigned tf, unsigned long n0)
{
	static const double row[4] = { 697.0, 770.0, 852.0, 941.0 };
	static const double col[4] = { 1209.0, 1336.0, 1477.0, 1633.0 };
	double a = dbm0_rms_get(TONE_DIGIT_DBM0) * sqrt(2.0);
	long lead = (long)(rate * TONE_LEAD_US / 1000000u);
	unsigned slot;
	int digit;
	unsigned i;

	memset(near, 0, frame * sizeof(*near));
	if (tf + 1u < TONE_DIGIT_FROM ||
	    tf >= TONE_DIGIT_FROM + TONE_DIGITS * TONE_DIGIT_PERIOD)
		return -1;
	slot = (tf + 1u - TONE_DIGIT_FROM) / TONE_DIGIT_PERIOD;
	if (slot >= TONE_DIGITS)
		return -1;
	digit = (int)((trial * TONE_DIGITS + slot) % 16u);
	for (i = 0; i < frame; i++) {
		long pos = ((long)tf + 1 - (long)TONE_DIGIT_FROM -
			    (long)(slot * TONE_DIGIT_PERIOD)) * (long)frame +
			   (long)i - (long)frame + lead;
		unsigned long k = n0 + i;

		if (pos < 0 || pos >= (long)(TONE_DIGIT_ON * frame))
			continue;
		near[i] = sample_clamp(a * (sin(2.0 * M_PI * row[digit % 4] *
						 (double)k / rate) +
					    sin(2.0 * M_PI *
						col[digit / 4 % 4] *
						(double)k / rate)));
	}
	return digit;
}

/*
 * The tone line at rate. r takes the coherence trials and the longest run that
 * the rebuilt readings of the escape allow (struct cohere_view). It also takes
 * the worst loss of a digit tone, read on the third and fourth frames of each
 * digit, after the processor's release.
 */
static void tone_line_run(unsigned rate, struct tone_result *r)
{
	static const double row[4] = { 697.0, 770.0, 852.0, 941.0 };
	static const double col[4] = { 1209.0, 1336.0, 1477.0, 1633.0 };
	unsigned frame = rate / 100u;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_view view;
	struct echo_path path;
	uint32_t state = 0x70ae1u;
	double a = dbm0_rms_get(TONE_DIAL_DBM0) * sqrt(2.0);
	unsigned long n = 0;
	unsigned total = TONE_WARM_FRAMES + TONE_TRIALS * TONE_TRIAL_FRAMES;
	unsigned run = 0;
	unsigned f;

	memset(r, 0, sizeof(*r));
	memset(&view, 0, sizeof(view));
	udsp_ec_nlms.nlp(ec, true);
	udsp_ec_nlms_ref_loss_set(ec, TONE_LOSS_HALF_DB);
	path_flat_init(&path, rate, 8, TONE_ERL_DB);
	for (f = 0; f < total; f++) {
		int16_t drive[UDSP_FRAME_MAX];
		int16_t floor[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		int16_t out[UDSP_FRAME_MAX];
		bool trial = f >= TONE_WARM_FRAMES;
		unsigned tf = trial ? (f - TONE_WARM_FRAMES) %
				      TONE_TRIAL_FRAMES : 0u;
		bool on = !trial || tf < TONE_ON_FRAMES;
		unsigned index = trial ? (f - TONE_WARM_FRAMES) /
					 TONE_TRIAL_FRAMES : 0u;
		int digit = trial ? tone_digit_fill(near, frame, rate, index,
						    tf, n) : -1;
		double ncc;
		double share;
		unsigned i;

		if (!trial)
			memset(near, 0, sizeof(near));
		noise_fill(floor, frame, TONE_FLOOR_DBM0, &state);
		for (i = 0; i < frame; i++, n++) {
			double v = a * (sin(2.0 * M_PI * TONE_LO_HZ *
					    (double)n / rate) +
					sin(2.0 * M_PI * TONE_HI_HZ *
					    (double)n / rate));

			drive[i] = on ? tone_ulaw_get(sample_clamp(v +
								   floor[i])) :
					0;
		}
		level_loss_apply(ref, drive, frame, TONE_LOSS_HALF_DB);
		path_run(&path, drive, echo);
		for (i = 0; i < frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i]);
		memcpy(out, mic, frame * sizeof(*mic));
		udsp_ec_nlms.run(ec, out, ref);
		udsp_ec_nlms_frame_get(ec, &reading);
		udsp_ec_nlms_info_get(ec, &info);
		cohere_view_add(&view, mic, out, frame, &reading);
		if (reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE)
			r->arms++;
		ncc = cohere_view_ncc_get(&view);
		share = cohere_view_share_get(&view);
		if (reading.ref_power >= reading.ref_bar) {
			if (info.double_talk && !reading.talker &&
			    ncc <= COHERE_BAR && share <= COHERE_SHARE_MAX)
				run++;
			else
				run = 0;
			if (run > r->run_max)
				r->run_max = run;
		}
		if (digit >= 0) {
			unsigned in = (tf + 1u - TONE_DIGIT_FROM) %
				      TONE_DIGIT_PERIOD;

			if (in >= 2u && in + 1u < TONE_DIGIT_ON) {
				double lo = 10.0 * log10(
					tone_bin_get(out, frame,
						     row[digit % 4], rate) /
					tone_bin_get(near, frame,
						     row[digit % 4], rate));
				double hi = 10.0 * log10(
					tone_bin_get(out, frame,
						     col[digit / 4 % 4],
						     rate) /
					tone_bin_get(near, frame,
						     col[digit / 4 % 4],
						     rate));

				if (lo < r->worst_db)
					r->worst_db = lo;
				if (hi < r->worst_db)
					r->worst_db = hi;
				r->digits++;
			}
		}
	}
	udsp_ec_nlms.destroy(ec);
}

/*
 * A near DTMF digit over a far dial tone starts no coherence trial, and digits
 * pass within TONE_LOSS_DB. The rebuilt readings reach a run of COHERE_FRAMES,
 * but a tonal frame of that run at LEC_COHERE_TONE_SHARE or more starts it over
 * first.
 */
static void cohere_tone_test(unsigned rate)
{
	struct tone_result r;

	tone_line_run(rate, &r);
	case_rate(rate, "lec coherence escape starts no trial on near DTMF "
		  "over a far dial tone");
	printf("# coherence trials %u, longest run of the readings %u, "
	       "%u digit frames, worst digit tone %.2f dB through the "
	       "canceller\n", r.arms, r.run_max, r.digits, r.worst_db);
	CHECK_UINT(r.arms, 0);
	CHECK(r.digits > 0u);
	CHECK(r.worst_db >= -TONE_LOSS_DB);
	CHECK(r.run_max >= COHERE_FRAMES);
}

/* A near tone NEAR_TONE_OVER_DB over the echo of the composite source, over the
 * composite hybrid at CHANGE_ERL_DB. */
#define NEAR_TONE_HZ		1004.0
#define NEAR_TONE_OVER_DB	12.0
#define NEAR_TONE_FRAMES	(4u * FRAMES_PER_SECOND)

/* A near tone at 1004 Hz, 12 dB over the echo, over a far broadband source
 * starts no coherence trial. The reference stays broadband, so the tonal bound
 * does not act, and the readings stay under the run on their own. */
static void cohere_near_tone_test(unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_line l;
	struct echo_path path;
	double echo_energy = 0.0;
	double out_energy = 0.0;
	unsigned long phase = 0;
	unsigned arms = 0;
	unsigned run = 0;
	unsigned run_max = 0;
	unsigned f;

	path_composite_init(&path, rate, CHANGE_ERL_DB);
	cohere_line_begin(&l, rate, &path, 0u, true, 0u);
	for (f = 0; f < NEAR_TONE_FRAMES; f++) {
		int16_t near[UDSP_FRAME_MAX];
		double ncc;
		double share;

		tone_fill(near, l.frame, CHANGE_ECHO_DBM0 + NEAR_TONE_OVER_DB,
			  NEAR_TONE_HZ, &phase, rate);
		cohere_frame_run(&l, near, &echo_energy, &out_energy,
				 &reading, &info);
		if (reading.trial_arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE)
			arms++;
		ncc = cohere_view_ncc_get(&l.view);
		share = cohere_view_share_get(&l.view);
		if (reading.ref_power >= reading.ref_bar) {
			if (info.double_talk && !reading.talker &&
			    ncc <= COHERE_BAR && share <= COHERE_SHARE_MAX)
				run++;
			else
				run = 0;
			if (run > run_max)
				run_max = run;
		}
	}
	udsp_ec_nlms.destroy(l.ec);
	case_rate(rate, "lec coherence escape starts no trial on a near tone "
		  "over a far broadband source");
	printf("# coherence trials %u, longest run of the readings %u\n",
	       arms, run_max);
	CHECK_UINT(arms, 0);
	CHECK(run_max < COHERE_FRAMES);
}

/* G.168 tests 2A, 2B and 12 on the bench schedule, the processor off: the
 * composite source or white noise at -10 dBm0 through m1. It changes to m4 or
 * m6 at 6 dB, or m7 or m8 at 12 dB, over Table D.1a's least ERL. */
#define PHASE2_FAR_DBM0		-10.0
#define PHASE2_LOSS_HALF_DB	18u
#define PHASE2_HISS_DBM0	-73.0
#define PHASE2_DELAY_MS		32u
#define PHASE2_DELTA_ERL_DB	10.0
#define PHASE2_CONVERGE_FRAMES	(6u * FRAMES_PER_SECOND)
#define PHASE2_FRAMES		(10u * FRAMES_PER_SECOND)
/* The window of the 1 s anchor of G.168 Figures 10b and 12b: one CSS period
 * of 0.7 s from 1 s after the change. */
#define PHASE2_ANCHOR_FRAMES	FRAMES_PER_SECOND
#define PHASE2_WINDOW_FRAMES	70u
#define PHASE2_STEADY_FRAMES	(3u * FRAMES_PER_SECOND)
#define PHASE2_BLOCK_FRAMES	10u
#define PHASE2_SETTLED_DB	20.0
#define PHASE2_DEEP_DB		30.0
#define PHASE2_MODELS		4u

static const enum annex_d_model phase2_models[PHASE2_MODELS] = {
	ANNEX_D_M4, ANNEX_D_M6, ANNEX_D_M7, ANNEX_D_M8,
};
static const double phase2_erl_db[PHASE2_MODELS] = { 6.0, 6.0, 12.0, 12.0 };

/*
 * The figures of one change: the ERLE at the 1 s anchor and over the last
 * PHASE2_STEADY_FRAMES, and the excess over the line noise. Then the frames to
 * the first block at PHASE2_SETTLED_DB and PHASE2_DEEP_DB. Last, the first
 * trial, its arm and end, and the trials and resets after the change.
 */
struct phase2_result {
	double erle_db;
	double steady_db;
	double excess_db;
	unsigned settled;
	unsigned deep;
	unsigned start;
	unsigned arm;
	unsigned end;
	unsigned trials;
	unsigned resets;
};

struct phase2_sums {
	double echo;
	double out;
	double excess;
	double hiss;
};

static void phase2_frame_run(struct udsp_ec *ec, struct echo_path *path,
			     uint32_t state[2], unsigned long *phase,
			     double hiss_dbm0, bool css,
			     struct phase2_sums *sum)
{
	int16_t far[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	if (css)
		css_fill(far, path->frame, PHASE2_FAR_DBM0, &state[0], phase,
			 path->frame * 100u);
	else
		noise_fill(far, path->frame, PHASE2_FAR_DBM0, &state[0]);
	level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
	path_run(path, far, echo);
	noise_fill(hiss, path->frame, hiss_dbm0, &state[1]);
	for (i = 0; i < path->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + hiss[i]);
	udsp_ec_nlms.run(ec, mic, ref);
	for (i = 0; i < path->frame; i++) {
		double left = (double)mic[i] - hiss[i];

		sum->echo += (double)echo[i] * echo[i];
		sum->out += (double)mic[i] * mic[i];
		sum->excess += left * left;
		sum->hiss += (double)hiss[i] * hiss[i];
	}
}

static double phase2_db_get(double num, double den)
{
	return den > 0.0 ? 10.0 * log10(num / den) : 200.0;
}

/* The change from m1 at to_erl_db plus PHASE2_DELTA_ERL_DB to the path to,
 * on a line noise at hiss_dbm0. */
static void phase2_path_run(unsigned rate, const struct echo_path *to,
			    double to_erl_db, double hiss_dbm0, bool css,
			    struct phase2_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct phase2_sums window = { 0 };
	struct phase2_sums steady = { 0 };
	struct phase2_sums block = { 0 };
	struct phase2_sums lead = { 0 };
	struct echo_path path;
	struct echo_path next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	unsigned long phase = 0;
	uint32_t probes;
	unsigned f;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms.nlp(ec, false);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	path_annex_d_init(&path, rate, ANNEX_D_M1,
			  to_erl_db + PHASE2_DELTA_ERL_DB, PHASE2_DELAY_MS);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		phase2_frame_run(ec, &path, state, &phase, hiss_dbm0, css,
				 &lead);
	udsp_ec_nlms_info_get(ec, &info);
	probes = info.probes;
	r->resets = info.diverge_resets;
	next = *to;
	path_history_carry(&next, &path);
	for (f = 1; f <= PHASE2_FRAMES; f++) {
		phase2_frame_run(ec, &next, state, &phase, hiss_dbm0, css,
				 &block);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (!r->start &&
		    reading.trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE) {
			r->start = f;
			r->arm = reading.trial_arm;
		}
		if (r->start && !r->end &&
		    reading.trial_end != UDSP_EC_NLMS_TRIAL_END_NONE)
			r->end = f;
		if (f % PHASE2_BLOCK_FRAMES)
			continue;
		if (f > PHASE2_ANCHOR_FRAMES &&
		    f <= PHASE2_ANCHOR_FRAMES + PHASE2_WINDOW_FRAMES) {
			window.echo += block.echo;
			window.out += block.out;
		}
		if (f > PHASE2_FRAMES - PHASE2_STEADY_FRAMES) {
			steady.echo += block.echo;
			steady.out += block.out;
			steady.excess += block.excess;
			steady.hiss += block.hiss;
		}
		if (!r->settled &&
		    phase2_db_get(block.echo, block.out) >= PHASE2_SETTLED_DB)
			r->settled = f;
		if (!r->deep &&
		    phase2_db_get(block.echo, block.out) >= PHASE2_DEEP_DB)
			r->deep = f;
		memset(&block, 0, sizeof(block));
	}
	udsp_ec_nlms_info_get(ec, &info);
	r->trials = info.probes - probes;
	r->resets = info.diverge_resets - r->resets;
	r->erle_db = phase2_db_get(window.echo, window.out);
	r->steady_db = phase2_db_get(steady.echo, steady.out);
	r->excess_db = phase2_db_get(steady.excess, steady.hiss);
	udsp_ec_nlms.destroy(ec);
}

static void phase2_print(unsigned rate, const char *what,
			 const struct phase2_result *r)
{
	printf("# %u Hz, %s: %.2f dB of ERLE at 1 s, %.2f dB at 7 to 10 s, "
	       "excess %.2f dB relative to the line noise; 20 dB on frame %u, "
	       "30 dB on frame %u; trial of arm %s on frame %u, ended on "
	       "frame %u; %u trials, %u resets\n", rate, what, r->erle_db,
	       r->steady_db, r->excess_db, r->settled, r->deep,
	       lecr_trial_arm_name(r->arm), r->start, r->end, r->trials,
	       r->resets);
}

/* The bound of phase2_test on the composite source, which a trial on the plain
 * step stays under. */
#define PHASE2_ERLE_DB		11.0

/*
 * The change of G.168 tests 2A, 2B and 12 on each model, at both rates. On
 * the composite source each change must read PHASE2_ERLE_DB of ERLE or more
 * at 1 s, with no reset. The white-noise legs print their figures.
 */
static void phase2_test(unsigned rate)
{
	unsigned i;

	case_rate(rate, "lec reconverges on the Annex D path changes of G.168 "
		  "test 2 within 1 s");
	for (i = 0; i < 2u * PHASE2_MODELS; i++) {
		struct phase2_result r;
		struct echo_path to;
		unsigned m = i % PHASE2_MODELS;
		char what[64];

		path_annex_d_init(&to, rate, phase2_models[m], phase2_erl_db[m],
				  PHASE2_DELAY_MS);
		phase2_path_run(rate, &to, phase2_erl_db[m], PHASE2_HISS_DBM0,
				i >= PHASE2_MODELS, &r);
		snprintf(what, sizeof(what), "%s, %s",
			 annex_d_tables[phase2_models[m]].name,
			 i >= PHASE2_MODELS ? "composite source" :
			 "white noise");
		phase2_print(rate, what, &r);
		CHECK_UINT(r.resets, 0);
		if (i >= PHASE2_MODELS)
			CHECK(r.erle_db >= PHASE2_ERLE_DB);
	}
}

/* The dense path: two reflections 20 ms apart, the second at DENSE_SECOND of
 * the first. A tail from 4 to 44 ms at DENSE_TAIL follows, with the white-noise
 * ERL of the change. */
#define DENSE_SECOND		0.8
#define DENSE_TAIL		0.25
#define DENSE_TAIL_MS		12.0
/*
 * The frames to 30 dB of ERLE on the dense path with the plain step of an
 * escape trial. They cover 8 and 16 kHz, white noise and the composite source.
 * The change must reach 30 dB within DENSE_DEEP_SLACK of them.
 */
#define DENSE_DEEP_SLACK	1.1

static const unsigned dense_plain_deep[2][2] = {
	{ 120u, 210u },
	{ 140u, 260u },
};

static void path_dense_init(struct echo_path *path, unsigned rate,
			    double erl_db)
{
	double per_ms = (double)rate / 1000.0;
	unsigned first = path_samples_get(4.0, rate);
	unsigned second = path_samples_get(24.0, rate);
	unsigned span = path_samples_get(8.0, rate);
	unsigned k;

	path_geometry_set(path, rate);
	path->taps = path_samples_get(44.0, rate);
	for (k = first; k < path->taps; k++) {
		double t = (double)(k - first);

		path->h[k] = (float)(DENSE_TAIL * exp(-t / (DENSE_TAIL_MS *
							    per_ms)) *
				     sin(2.0 * M_PI * 450.0 * t / rate));
		if (k < first + span)
			path->h[k] += (float)(exp(-t / (1.75 * per_ms)) *
					      sin(5.6 / per_ms * t));
		if (k >= second && k < second + span)
			path->h[k] += (float)(DENSE_SECOND *
					      exp(-(double)(k - second) /
						  (1.75 * per_ms)) *
					      sin(5.6 / per_ms *
						  (double)(k - second)));
	}
	path_erl_set(path, erl_db);
}

/*
 * A change to the dense path, 10 dB louder than m1, in the blocks that the
 * filter of m1 does not hold. On both sources its output must reach 30 dB of
 * ERLE within DENSE_DEEP_SLACK of the frames of the plain step
 * (dense_plain_deep), with no reset.
 */
static void dense_change_test(unsigned rate)
{
	const unsigned *plain = dense_plain_deep[rate == UDSP_SAMPLE_RATE_WB];
	struct phase2_result r;
	struct echo_path to;
	unsigned css;

	path_dense_init(&to, rate, 6.0);
	case_rate(rate, "lec reconverges on a dense path after a change");
	for (css = 0; css < 2u; css++) {
		phase2_path_run(rate, &to, 6.0, PHASE2_HISS_DBM0, css, &r);
		phase2_print(rate, css ? "dense path, composite source" :
			     "dense path, white noise", &r);
		CHECK_UINT(r.resets, 0);
		CHECK(r.deep > 0u);
		CHECK(r.deep <= DENSE_DEEP_SLACK * plain[css]);
	}
}

/* The line noise of misadjust_test, over the echo that a converged filter
 * leaves, and the slack of the excess over it; misadjust_plain_db holds the
 * excess of the plain step. */
#define MISADJUST_HISS_DBM0	-45.0
#define MISADJUST_SLACK_DB	1.0

static const double misadjust_plain_db[2] = { -9.28, -8.40 };

/*
 * The misadjustment line: the change to m4 on white noise, with a line noise
 * of MISADJUST_HISS_DBM0. Over the last 3 s the excess relative to the line
 * noise must stand within MISADJUST_SLACK_DB of that of the plain step,
 * misadjust_plain_db.
 */
static void misadjust_test(unsigned rate)
{
	struct phase2_result r;
	struct echo_path to;

	path_annex_d_init(&to, rate, ANNEX_D_M4, 6.0, PHASE2_DELAY_MS);
	phase2_path_run(rate, &to, 6.0, MISADJUST_HISS_DBM0, false, &r);
	case_rate(rate, "lec adds no misadjustment on a converged line after a "
		  "change");
	phase2_print(rate, "m4 on a noisy line, white noise", &r);
	CHECK_UINT(r.resets, 0);
	CHECK(r.excess_db <= misadjust_plain_db[rate == UDSP_SAMPLE_RATE_WB] +
			     MISADJUST_SLACK_DB);
}

/*
 * Phase I of G.168 tests 2A and 12, with the processor on: the filter converges
 * from zero on m1 at the change's ERL plus PHASE2_DELTA_ERL_DB. The meter is
 * the PHASE1_METER_MS one-pole of G.168 clause 6.4.1.2.1 without its weighting.
 */
#define PHASE1_FRAMES		(6u * FRAMES_PER_SECOND)
#define PHASE1_METER_MS		35.0
#define PHASE1_LINES		2u

static const double phase1_erl_db[PHASE1_LINES] = { 16.0, 22.0 };

/* The bound of phase1_test on css_annex_fill: 1.5 dB under the -63.5 dBm0 that
 * G.168 Figure 9 allows at an active level of -10 dBm0. */
#define PHASE1_LRET_DBM0	-65.0

/* l_ret_dbm0 is the loudest meter reading from 1 s on, first_act the first act
 * of the processor. */
struct phase1_result {
	double l_ret_dbm0;
	unsigned first_act;
};

/* One frame of the line of phase1_line_run. near, when set, adds a near
 * talker to the microphone, and out takes the output. */
static void phase1_frame_run(struct udsp_ec *ec, struct echo_path *path,
			     struct css_annex *css, uint32_t state[2],
			     const int16_t *near, int16_t *out)
{
	int16_t far[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	unsigned i;

	if (css)
		css_annex_fill(far, path->frame, css);
	else
		noise_fill(far, path->frame, PHASE2_FAR_DBM0, &state[0]);
	level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
	path_run(path, far, echo);
	noise_fill(hiss, path->frame, PHASE2_HISS_DBM0, &state[1]);
	for (i = 0; i < path->frame; i++)
		out[i] = sample_clamp((double)echo[i] + hiss[i] +
				      (near ? near[i] : 0));
	udsp_ec_nlms.run(ec, out, ref);
}

static void phase1_line_run(unsigned rate, double erl_db, bool use_css,
			    struct phase1_result *r)
{
	struct udsp_ec_nlms_info info;
	struct css_annex css;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	unsigned long from = rate + path_samples_get(PHASE2_DELAY_MS, rate);
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double loudest = 0.0;
	int16_t out[UDSP_FRAME_MAX];
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	path_annex_d_init(&path, rate, ANNEX_D_M1, erl_db, PHASE2_DELAY_MS);
	for (f = 0; f < PHASE1_FRAMES; f++) {
		phase1_frame_run(ec, &path, use_css ? &css : NULL, state, NULL,
				 out);
		udsp_ec_nlms_info_get(ec, &info);
		if (!r->first_act && info.nlp_active)
			r->first_act = f + 1u;
		for (i = 0; i < path.frame; i++) {
			meter = a * meter + (1.0 - a) * out[i] * out[i];
			if ((unsigned long)f * path.frame + i >= from &&
			    meter > loudest)
				loudest = meter;
		}
	}
	r->l_ret_dbm0 = rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/*
 * The steady state of Phase I against G.168 Figure 9, at both rates. On the
 * source of css_annex_fill each line must read PHASE1_LRET_DBM0 or less.
 * The white-noise legs print their figures.
 */
static void phase1_test(unsigned rate)
{
	unsigned i;

	case_rate(rate, "lec processor holds the converging line of G.168 "
		  "test 2 under the limit of Figure 9");
	for (i = 0; i < 2u * PHASE1_LINES; i++) {
		struct phase1_result r;
		double erl_db = phase1_erl_db[i % PHASE1_LINES];
		bool use_css = i >= PHASE1_LINES;

		phase1_line_run(rate, erl_db, use_css, &r);
		printf("# %u Hz, m1 at %.0f dB, %s: l_ret %.1f dBm0 from 1 s, "
		       "first act on frame %u\n", rate, erl_db,
		       use_css ? "CSS structure" : "white noise", r.l_ret_dbm0,
		       r.first_act);
		if (use_css)
			CHECK(r.l_ret_dbm0 <= PHASE1_LRET_DBM0);
	}
}

/* LEC_NLP_BOOT_FRAMES of dsp/lec.c: the active frames of the bootstrap. */
#define NEAR_START_BOOT_FRAMES	150u

/* The line of nlp_boot_near_start_test: m1 at 16 dB, the echo at -26 dBm0, and
 * a dense talker at NEAR_START_DBM0 through the bootstrap. The detector does
 * not fire on it, so the combined-loss arm decides its echo-like frames. */
#define NEAR_START_ERL_DB	16.0
#define NEAR_START_DBM0		-39.0
#define NEAR_START_FRAMES	250u
#define NEAR_START_SETTLE_FRAMES	(4u * FRAMES_PER_SECOND)
#define NEAR_START_RUN_FRAMES	(8u * FRAMES_PER_SECOND)
#define NEAR_START_TAKE_DB	0.5
#define NEAR_START_NEW_TAKE_DB	1.0
#define NEAR_START_SEED		0x0badcafeu

/*
 * One leg: the talker's energy and the output against it over its frames, and
 * the frames that fire the detector and those that the processor acts on. Then
 * the bootstrap's last active frame and the loudest meter reading from 1 s
 * after the talker.
 */
struct near_start_result {
	double near;
	double product;
	unsigned frames;
	unsigned fired;
	unsigned acted;
	unsigned boot_frame;
	double l_ret_dbm0;
};

/* The talker frame of a leg: the frame of the dense talker, or silence
 * after NEAR_START_FRAMES. Returns whether the frame carries the talker. */
static bool near_start_fill(struct dense_talker *talker, int16_t *near,
			    unsigned frame, unsigned f)
{
	unsigned i;

	memset(near, 0, frame * sizeof(*near));
	if (f >= NEAR_START_FRAMES)
		return false;
	dense_talker_fill(talker, near, frame);
	for (i = 0; i < frame; i++)
		if (near[i])
			return true;
	return false;
}

static void near_start_leg_run(unsigned rate, bool reset, bool nlp,
			       uint32_t seed, struct near_start_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct dense_talker talker;
	struct css_annex css;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double loudest = 0.0;
	int16_t near[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
	unsigned active = 0;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	dense_talker_init(&talker, rate, NEAR_START_DBM0, seed);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, nlp);
	path_annex_d_init(&path, rate, ANNEX_D_M1, NEAR_START_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; reset && f < NEAR_START_SETTLE_FRAMES; f++)
		phase1_frame_run(ec, &path, &css, state, NULL, out);
	if (reset)
		udsp_ec_nlms_reset(ec);
	for (f = 0; f < NEAR_START_RUN_FRAMES; f++) {
		bool talking = near_start_fill(&talker, near, path.frame, f);

		phase1_frame_run(ec, &path, &css, state, near, out);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (reading.ref_power >= reading.ref_bar &&
		    ++active == NEAR_START_BOOT_FRAMES)
			r->boot_frame = f + 1u;
		r->frames += talking;
		r->fired += talking && reading.dt_fired;
		r->acted += talking && info.nlp_active;
		for (i = 0; talking && i < path.frame; i++) {
			r->near += (double)near[i] * near[i];
			r->product += (double)out[i] * near[i];
		}
		for (i = 0; i < path.frame; i++) {
			meter = a * meter + (1.0 - a) * out[i] * out[i];
			if (f >= NEAR_START_FRAMES + FRAMES_PER_SECOND &&
			    meter > loudest)
				loudest = meter;
		}
	}
	r->l_ret_dbm0 = rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/*
 * A quiet near talker at the start of the line, in the bootstrap: from NEW and
 * after a reset, with the processor on and off. The processor takes at most
 * NEAR_START_NEW_TAKE_DB from NEW and NEAR_START_TAKE_DB after the reset. The
 * meter reads PHASE1_LRET_DBM0 or less from 1 s after the talker.
 */
static void nlp_boot_near_start_test(unsigned rate)
{
	unsigned leg;

	case_rate(rate, "lec nlp keeps a quiet near talker in the bootstrap of "
		  "the combined-loss arm and after a reset");
	for (leg = 0; leg < 2u; leg++) {
		struct near_start_result on;
		struct near_start_result off;
		double take_db;

		near_start_leg_run(rate, leg == 1u, true, NEAR_START_SEED, &on);
		near_start_leg_run(rate, leg == 1u, false, NEAR_START_SEED,
				   &off);
		take_db = near_gain_db_get(off.product, off.near) -
			  near_gain_db_get(on.product, on.near);
		printf("# %u Hz, %s: talker at %.0f dBm0 on %u frames, %u "
		       "fire the detector, the bootstrap ends on frame %u; "
		       "%.2f dB through, filter alone %.2f dB, the processor "
		       "takes %.2f dB and acts on %u of its frames; l_ret "
		       "%.1f dBm0 from 1 s after the talker\n", rate,
		       leg ? "after a reset" : "from NEW", NEAR_START_DBM0,
		       on.frames, on.fired, on.boot_frame,
		       near_gain_db_get(on.product, on.near),
		       near_gain_db_get(off.product, off.near), take_db,
		       on.acted, on.l_ret_dbm0);
		CHECK(on.boot_frame > 0u && on.boot_frame <= NEAR_START_FRAMES);
		CHECK(take_db <= (leg ? NEAR_START_TAKE_DB :
					NEAR_START_NEW_TAKE_DB));
		CHECK(on.l_ret_dbm0 <= PHASE1_LRET_DBM0);
	}
}

/* The talker seeds of nlp_boot_seeds_test: NEAR_SEEDS seeds after
 * NEAR_START_SEED, each NEAR_SEED_STEP after the one before. */
#define NEAR_SEEDS		3u
#define NEAR_SEED_STEP		0x9e3779b9u

/* The leg from NEW of nlp_boot_near_start_test on NEAR_SEEDS other seeds: on
 * each the processor takes NEAR_START_NEW_TAKE_DB or less and l_ret_dbm0 reads
 * PHASE1_LRET_DBM0 or less. */
static void nlp_boot_seeds_test(unsigned rate)
{
	unsigned k;

	case_rate(rate, "lec nlp keeps quiet near talkers of other seeds in "
		  "the bootstrap of the combined-loss arm");
	for (k = 1u; k <= NEAR_SEEDS; k++) {
		uint32_t seed = NEAR_START_SEED + k * NEAR_SEED_STEP;
		struct near_start_result on;
		struct near_start_result off;
		double take_db;

		near_start_leg_run(rate, false, true, seed, &on);
		near_start_leg_run(rate, false, false, seed, &off);
		take_db = near_gain_db_get(off.product, off.near) -
			  near_gain_db_get(on.product, on.near);
		printf("# %u Hz, talker seed %u: the processor takes %.2f dB "
		       "and acts on %u of its %u frames; l_ret %.1f dBm0 from "
		       "1 s after the talker\n", rate, k, take_db, on.acted,
		       on.frames, on.l_ret_dbm0);
		CHECK(take_db <= NEAR_START_NEW_TAKE_DB);
		CHECK(on.l_ret_dbm0 <= PHASE1_LRET_DBM0);
	}
}

/* G.168 test 9A on the bench schedule: white far noise through m1, near noise
 * steps with the far end off then on, and an uncancellable cubic term. After
 * the far return the output stands within CN9A_TOL_DB of l_sgen. */
#define CN9A_FAR_DBM0		-10.0
#define CN9A_ERL_DB		12.0
#define CN9A_CUBIC		1.0
#define CN9A_WARMUP_FRAMES	560u
#define CN9A_QUIET_FRAMES	(3u * FRAMES_PER_SECOND)
#define CN9A_ON_FRAMES		(5u * FRAMES_PER_SECOND)
#define CN9A_SKIP_FRAMES	FRAMES_PER_SECOND
#define CN9A_SETTLE_FRAMES	(2u * FRAMES_PER_SECOND + 4u)
#define CN9A_WINDOW_FRAMES	70u
#define CN9A_TOL_DB		2.0

#define CN9A_LEVELS		4u

static const double cn9a_levels[CN9A_LEVELS] = { -45.0, -50.0, -55.0, -60.0 };

struct cn9a_line {
	struct udsp_ec *ec;
	struct echo_path path;
	struct room_noise near;
	struct udsp_resamp up_far;
	struct udsp_resamp up_near;
	struct udsp_resamp down;
	uint32_t far_state;
	unsigned rate;
	int16_t out[UDSP_FRAME_MAX];
};

/* The errors of one step: the least and the most window error, in dB. */
struct cn9a_step {
	double l_sgen_dbm0;
	double low_db;
	double high_db;
	float floor_dbov;
};

static void cn9a_line_init(struct cn9a_line *l, unsigned rate)
{
	memset(l, 0, sizeof(*l));
	l->rate = rate;
	l->far_state = 0x9a0f0a1du;
	l->ec = udsp_ec_nlms.create_rate(0, rate);
	udsp_ec_nlms_ref_loss_set(l->ec, PHASE2_LOSS_HALF_DB);
	path_annex_d_init(&l->path, rate, ANNEX_D_M1, CN9A_ERL_DB,
			  PHASE2_DELAY_MS);
	l->path.cubic = CN9A_CUBIC;
	udsp_resamp_reset(&l->up_far);
	udsp_resamp_reset(&l->up_near);
	udsp_resamp_reset(&l->down);
}

/*
 * One frame of the line; returns the energy of its wire plane, and keeps the
 * output of the canceller in l->out. Without near_on the near noise is zero
 * at the line rate, so a stop of the near noise leaves no interpolator tail.
 */
static double cn9a_frame_run(struct cn9a_line *l, bool far_on, bool near_on)
{
	int16_t far8[UDSP_FRAME_SAMPLES];
	int16_t near8[UDSP_FRAME_SAMPLES];
	int16_t far[UDSP_FRAME_MAX];
	int16_t near[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	int16_t wire[UDSP_FRAME_SAMPLES];
	unsigned n = l->path.frame;
	double energy = 0.0;
	unsigned i;

	memset(far8, 0, sizeof(far8));
	memset(near8, 0, sizeof(near8));
	if (far_on)
		noise_fill(far8, UDSP_FRAME_SAMPLES, CN9A_FAR_DBM0,
			   &l->far_state);
	if (near_on)
		room_noise_fill(near8, UDSP_FRAME_SAMPLES, &l->near);
	if (l->rate == UDSP_SAMPLE_RATE_WB) {
		udsp_resamp_up2(&l->up_far, far8, UDSP_FRAME_SAMPLES, far);
		memset(near, 0, sizeof(near));
		if (near_on)
			udsp_resamp_up2(&l->up_near, near8, UDSP_FRAME_SAMPLES,
					near);
	} else {
		memcpy(far, far8, sizeof(far8));
		memcpy(near, near8, sizeof(near8));
	}
	level_loss_apply(ref, far, n, PHASE2_LOSS_HALF_DB);
	path_run(&l->path, far, echo);
	for (i = 0; i < n; i++)
		mic[i] = sample_clamp((double)echo[i] + near[i]);
	udsp_ec_nlms.run(l->ec, mic, ref);
	memcpy(l->out, mic, sizeof(mic[0]) * n);
	if (l->rate == UDSP_SAMPLE_RATE_WB)
		udsp_resamp_down2(&l->down, mic, n, wire);
	else
		memcpy(wire, mic, sizeof(wire));
	for (i = 0; i < UDSP_FRAME_SAMPLES; i++)
		energy += (double)wire[i] * wire[i];
	return energy;
}

/* One step of near noise of kind at level_dbm0 on a converged line. */
static void cn9a_step_run(struct cn9a_line *l, enum room_kind kind,
			  double level_dbm0, uint32_t seed,
			  struct cn9a_step *st)
{
	struct udsp_ec_nlms_info info;
	double quiet = 0.0;
	double window = 0.0;
	unsigned f;

	room_noise_init(&l->near, kind, level_dbm0, seed);
	for (f = 0; f < CN9A_QUIET_FRAMES; f++) {
		double e = cn9a_frame_run(l, false, true);

		if (f >= CN9A_SKIP_FRAMES)
			quiet += e;
	}
	st->l_sgen_dbm0 = energy_dbm0_get(quiet, (CN9A_QUIET_FRAMES -
						  CN9A_SKIP_FRAMES) *
						 UDSP_FRAME_SAMPLES);
	st->low_db = INFINITY;
	st->high_db = -INFINITY;
	for (f = 0; f < CN9A_ON_FRAMES; f++) {
		double e = cn9a_frame_run(l, true, true);
		unsigned at;

		if (f < CN9A_SETTLE_FRAMES)
			continue;
		at = f - CN9A_SETTLE_FRAMES;
		if (at / CN9A_WINDOW_FRAMES >=
		    (CN9A_ON_FRAMES - CN9A_SETTLE_FRAMES) / CN9A_WINDOW_FRAMES)
			continue;
		window += e;
		if ((at + 1u) % CN9A_WINDOW_FRAMES == 0u) {
			double err = energy_dbm0_get(window,
						     CN9A_WINDOW_FRAMES *
						     UDSP_FRAME_SAMPLES) -
				     st->l_sgen_dbm0;

			if (err < st->low_db)
				st->low_db = err;
			if (err > st->high_db)
				st->high_db = err;
			window = 0.0;
		}
	}
	udsp_ec_nlms_info_get(l->ec, &info);
	st->floor_dbov = info.noise_dbov;
}

static void cn9a_warmup_run(struct cn9a_line *l)
{
	unsigned f;

	for (f = 0; f < CN9A_WARMUP_FRAMES; f++)
		(void)cn9a_frame_run(l, true, false);
}

static void cn9a_print(unsigned rate, const char *kind, double level_dbm0,
		       const struct cn9a_step *st)
{
	printf("# %u Hz, %s near noise at %.2f dBm0: l_sgen %.2f dBm0, "
	       "windows %+.2f to %+.2f dB, floor %.2f dBov\n", rate, kind,
	       level_dbm0, st->l_sgen_dbm0, st->low_db, st->high_db,
	       (double)st->floor_dbov);
}

/*
 * The comfort noise of the 9A line at both rates, on white and on
 * Hoth-shaped near noise at each level of cn9a_levels. Every window must
 * stand within CN9A_TOL_DB of l_sgen. The raise and the band of the fill
 * (dsp/lec.c, LEC_CN_RAISE_DB) hold the windows there at 16 kHz.
 */
static void cn9a_test(unsigned rate)
{
	unsigned k;

	for (k = 0; k < 2u; k++) {
		enum room_kind kind = k ? ROOM_HOTH : ROOM_WHITE;
		const char *name = k ? "Hoth-shaped" : "white";
		struct cn9a_line l;
		double low = INFINITY;
		double high = -INFINITY;
		unsigned i;

		cn9a_line_init(&l, rate);
		cn9a_warmup_run(&l);
		if (k)
			case_rate(rate, "lec comfort noise matches Hoth-shaped "
				  "near noise of G.168 test 9A");
		else
			case_rate(rate, "lec comfort noise matches white near "
				  "noise of G.168 test 9A");
		for (i = 0; i < CN9A_LEVELS; i++) {
			struct cn9a_step st;

			cn9a_step_run(&l, kind, cn9a_levels[i],
				      0x9a000000u + 16u * k + i, &st);
			cn9a_print(rate, name, cn9a_levels[i], &st);
			if (st.low_db < low)
				low = st.low_db;
			if (st.high_db > high)
				high = st.high_db;
		}
		CHECK(low >= -CN9A_TOL_DB);
		CHECK(high <= CN9A_TOL_DB);
		udsp_ec_nlms.destroy(l.ec);
	}
}

/*
 * The levels of cn_quiet_test, CN_QUIET_STEP_DB apart so the fill's whole-dB
 * step falls at four places, and the lowest level that the tracker reads. The
 * fill rounds to a whole dB: CN_QUIET_BOUND_DB is LEC_CN_RAISE_DB plus 0.5 dB.
 */
#define CN_QUIET_DBM0		-70.0
#define CN_QUIET_STEP_DB	0.25
#define CN_QUIET_STEPS		4u
#define CN_QUIET_LOW_DBM0	-80.0
#define CN_QUIET_BOUND_DB	1.5

/*
 * The comfort noise of a quiet line at both rates: the 9A line with white
 * near noise at each level of cn_quiet_test. White noise is the near noise
 * of the least tracker bias, so its fill stands highest. Each window must
 * stand at most CN_QUIET_BOUND_DB over l_sgen. The line must hold a measured
 * floor at the lowest level.
 */
static void cn_quiet_test(unsigned rate)
{
	struct cn9a_line l;
	struct cn9a_step st;
	double high = -INFINITY;
	unsigned i;

	cn9a_line_init(&l, rate);
	cn9a_warmup_run(&l);
	case_rate(rate, "lec comfort noise stays near the noise of a quiet "
		  "line");
	for (i = 0; i <= CN_QUIET_STEPS; i++) {
		double level = i < CN_QUIET_STEPS ?
			       CN_QUIET_DBM0 - CN_QUIET_STEP_DB * i :
			       CN_QUIET_LOW_DBM0;

		cn9a_step_run(&l, ROOM_WHITE, level, 0x9b000000u + i, &st);
		cn9a_print(rate, "white", level, &st);
		if (st.high_db > high)
			high = st.high_db;
	}
	CHECK(high <= CN_QUIET_BOUND_DB);
	CHECK(st.floor_dbov < 127.0f);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The near noise of cn_drop_test, the loudest level of cn9a_levels, and the
 * length of its far phase. CN_DROP_TAIL_FRAMES bounds the frames from the
 * stop of the far end to the release of the processor.
 */
#define CN_DROP_DBM0		-45.0
#define CN_DROP_FAR_FRAMES	(2u * FRAMES_PER_SECOND)
#define CN_DROP_TAIL_FRAMES	64u

/* The frame before the release and the release frame of cn_drop_run. */
struct cn_drop {
	unsigned release;
	bool acted;
	int fill_dbov;
	int res_dbov;
	int out_dbov;
};

/* One frame of both lines of cn_drop_run; true when the processor acted. */
static bool cn_drop_frame_run(struct cn9a_line *on, struct cn9a_line *off,
			      bool far_on, bool near_on)
{
	struct udsp_ec_nlms_info info;

	(void)cn9a_frame_run(on, far_on, near_on);
	(void)cn9a_frame_run(off, far_on, near_on);
	udsp_ec_nlms_info_get(on->ec, &info);
	return info.nlp_active;
}

/*
 * One run of cn_drop_test: the 9A line at 16 kHz with the processor on and a
 * twin with it off, whose output is the residual. A 9A step runs, then the far
 * end stops and the near noise runs on for stop frames. The release is the
 * first frame after the stop with no act.
 */
static void cn_drop_run(unsigned stop, struct cn_drop *d)
{
	struct cn9a_line on;
	struct cn9a_line off;
	unsigned n;
	unsigned f;

	cn9a_line_init(&on, UDSP_SAMPLE_RATE_WB);
	cn9a_line_init(&off, UDSP_SAMPLE_RATE_WB);
	udsp_ec_nlms.nlp(off.ec, false);
	cn9a_warmup_run(&on);
	cn9a_warmup_run(&off);
	room_noise_init(&on.near, ROOM_WHITE, CN_DROP_DBM0, 0x9c000000u);
	room_noise_init(&off.near, ROOM_WHITE, CN_DROP_DBM0, 0x9c000000u);
	n = on.path.frame;
	memset(d, 0, sizeof(*d));
	for (f = 0; f < CN9A_QUIET_FRAMES + CN_DROP_FAR_FRAMES; f++)
		d->acted = cn_drop_frame_run(&on, &off,
					     f >= CN9A_QUIET_FRAMES, true);
	d->fill_dbov = udsp_cn_level_get(on.out, n);
	for (f = 0; f < CN_DROP_TAIL_FRAMES; f++) {
		if (!cn_drop_frame_run(&on, &off, false, f < stop))
			break;
		d->acted = true;
		d->fill_dbov = udsp_cn_level_get(on.out, n);
	}
	d->release = f;
	d->res_dbov = udsp_cn_level_get(off.out, n);
	d->out_dbov = udsp_cn_level_get(on.out, n);
	udsp_ec_nlms.destroy(on.ec);
	udsp_ec_nlms.destroy(off.ec);
}

/*
 * A fill at a loud floor followed by a silent request at 16 kHz. The near noise
 * stops on the release frame, so the frame before is a fill at the near level.
 * The release frame reads under 1 LSB, as the level applies after the
 * interpolator (LEC_CN_DRAW_DBOV).
 */
static void cn_drop_test(void)
{
	struct cn_drop first;
	struct cn_drop d;

	case_rate(UDSP_SAMPLE_RATE_WB, "lec comfort noise goes silent with a "
		  "silent request after a loud fill");
	cn_drop_run(CN_DROP_TAIL_FRAMES, &first);
	cn_drop_run(first.release, &d);
	printf("# 16000 Hz: the processor releases %u frames after the far "
	       "stop; fill %d dBov before it, release frame residual %d dBov "
	       "and output %d dBov\n", d.release, d.fill_dbov, d.res_dbov,
	       d.out_dbov);
	CHECK_UINT(d.release, first.release);
	CHECK(d.release < CN_DROP_TAIL_FRAMES);
	CHECK(d.acted);
	CHECK(fabs(d.fill_dbov - floor_dbov_get(CN_DROP_DBM0)) <= 2.0);
	CHECK(d.res_dbov == 127);
	CHECK(d.out_dbov == 127);
}

/* G.168 tests 3A and 3B on the bench schedule, the processor off: the source of
 * css_annex_init through m1 and white line noise. The near source of
 * css_annex_dt_init plays x_db under it from the echo onset. */
#define DT_ERL_DB		6.0
#define DT_HISS_DBM0		-64.6
#define DT_LEAD_FRAMES		100u
#define DT_PHASE_PCT		46u
#define DT_PERIOD_FRAMES	70u
#define DT_PERIODS		16u
#define DT_PROBE_PERIODS	4u
#define DT_CONVERGE_S		5.0
/* G.168 clause 6.4.1.2: the active level of a CSS over its level, single
 * talk and double talk. */
#define DT_ACT_SINGLE_DB	1.49
#define DT_ACT_DOUBLE_DB	1.66

/* l_res_dbm0 is the residual meter from DT_CONVERGE_S on, worst_dbm0 its
 * loudest reading over the double talk, held the frames that the detector
 * holds. */
struct dt_line_result {
	double l_res_dbm0;
	double worst_dbm0;
	double l_sgen_act;
	double l_rin_act;
	unsigned trials;
	unsigned resets;
	unsigned held;
};

/* The phase of dt3a_test and dt3b_test: DT_PHASE_PCT of a frame, in
 * samples. */
static unsigned dt_phase_get(unsigned rate)
{
	return rate / 100u * DT_PHASE_PCT / 100u;
}

/* One sample of a source that plays on [from, to) of the line. */
static int16_t dt_sample_get(struct css_annex *s, unsigned long n,
			     unsigned long from, unsigned long to)
{
	int16_t v = 0;

	if (n >= from && n < to)
		css_annex_fill(&v, 1, s);
	return v;
}

/* The figures of the line at far_dbm0 with a near end x_db under it. They
 * follow converge periods of the far end alone. The far end starts phase
 * samples into a frame. */
static void dt_line_run(unsigned rate, double far_dbm0, double x_db,
			unsigned converge, unsigned phase,
			struct dt_line_result *r)
{
	struct udsp_ec_nlms_info info;
	struct css_annex far_src;
	struct css_annex near_src;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t hiss_state = 0x51ab0dd5u;
	unsigned frame = rate / 100u;
	unsigned long period = (unsigned long)DT_PERIOD_FRAMES * frame;
	unsigned long t0 = (unsigned long)DT_LEAD_FRAMES * frame + phase;
	unsigned long dt0 = t0 + converge * period;
	unsigned long dt1 = dt0 + DT_PERIODS * period;
	unsigned long near_at = dt0 + path_samples_get(PHASE2_DELAY_MS, rate);
	unsigned long near_end = dt1 + DT_PROBE_PERIODS * period;
	unsigned long from = near_at + (unsigned long)(DT_CONVERGE_S * rate);
	unsigned frames = (unsigned)(near_end / frame) + 1u;
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double loudest = 0.0;
	double worst = 0.0;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	r->l_rin_act = far_dbm0 + DT_ACT_SINGLE_DB;
	r->l_sgen_act = r->l_rin_act - x_db;
	css_annex_init(&far_src, rate, far_dbm0, 0x0c55a11eu);
	css_annex_dt_init(&near_src, rate, r->l_sgen_act - DT_ACT_DOUBLE_DB,
			  0x5eed0005u);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, false);
	path_annex_d_init(&path, rate, ANNEX_D_M1, DT_ERL_DB, PHASE2_DELAY_MS);
	for (f = 0; f < frames; f++) {
		int16_t far[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];

		for (i = 0; i < frame; i++) {
			unsigned long n = (unsigned long)f * frame + i;

			far[i] = dt_sample_get(&far_src, n, t0, dt1);
			near[i] = dt_sample_get(&near_src, n, near_at,
						near_end);
		}
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		noise_fill(hiss, frame, DT_HISS_DBM0, &hiss_state);
		for (i = 0; i < frame; i++)
			mic[i] = sample_clamp((double)echo[i] + hiss[i] +
					      near[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if ((unsigned long)f * frame >= dt0 &&
		    (unsigned long)f * frame < dt1 && info.double_talk)
			r->held++;
		for (i = 0; i < frame; i++) {
			unsigned long n = (unsigned long)f * frame + i;
			double e = (double)mic[i] - near[i];

			meter = a * meter + (1.0 - a) * e * e;
			if (n >= from && n < dt1 && meter > loudest)
				loudest = meter;
			if (n >= near_at && n < dt1 && meter > worst)
				worst = meter;
		}
	}
	udsp_ec_nlms_info_get(ec, &info);
	r->l_res_dbm0 = rms_dbm0_get(sqrt(loudest));
	r->worst_dbm0 = rms_dbm0_get(sqrt(worst));
	r->trials = info.probes;
	r->resets = info.diverge_resets;
	udsp_ec_nlms.destroy(ec);
}

static void dt_line_print(unsigned rate, const char *what,
			  const struct dt_line_result *r)
{
	printf("# %u Hz, %s: l_res %.2f dBm0 from 5 s, %.2f dBm0 over the "
	       "double talk, against L_Sgen,act %.2f dBm0; %u trials, %u "
	       "resets\n", rate, what, r->l_res_dbm0, r->worst_dbm0,
	       r->l_sgen_act, r->trials, r->resets);
}

/*
 * G.168 test 3A at L_Rin -10 and -25 dBm0: the near end starts 15 dB under
 * the far end with it, on a new instance. Each leg must read l_res_dbm0 at
 * L_Sgen,act or under, the limit of the clause. Without the onset floor of
 * dsp/lec.c (onset_floor_get), the -10 dBm0 line reads over that limit.
 */
static void dt3a_test(unsigned rate)
{
	struct dt_line_result r;

	dt_line_run(rate, -10.0, 15.0, 0u, dt_phase_get(rate), &r);
	case_rate(rate, "lec converges under double talk from the start of "
		  "G.168 test 3A");
	dt_line_print(rate, "test 3A at -10 dBm0", &r);
	CHECK(r.l_res_dbm0 <= r.l_sgen_act);
	dt_line_run(rate, -25.0, 15.0, 0u, dt_phase_get(rate), &r);
	case_rate(rate, "lec keeps the residual of G.168 test 3A at -25 dBm0 "
		  "under the limit");
	dt_line_print(rate, "test 3A at -25 dBm0", &r);
	CHECK(r.l_res_dbm0 <= r.l_sgen_act);
}

/* The phases of dt3a_phase_test as samples at 16 kHz: DT_THIN_SAMPLES_WB before
 * a frame end for a thin onset. DT_TAIL_SAMPLES_WB into a frame sets the end of
 * a burst, the tail-frame class. */
#define DT_THIN_SAMPLES_WB	5u
#define DT_TAIL_SAMPLES_WB	2u

/* The bound of dt3a_phase_test on held: about twice what these lines hold with
 * the short hangover on a thin frame. */
#define DT_HELD_MAX		200u

/* n samples at 16 kHz in samples at rate, rounded. */
static unsigned dt_samples_get(unsigned n, unsigned rate)
{
	return (n * rate + UDSP_SAMPLE_RATE_WB / 2u) / UDSP_SAMPLE_RATE_WB;
}

/* The phase at which the far end of dt_line_run starts DT_THIN_SAMPLES_WB
 * before a frame end. */
static unsigned dt_thin_phase_get(unsigned rate)
{
	return rate / 100u - dt_samples_get(DT_THIN_SAMPLES_WB, rate);
}

/* The phase at which each far burst of dt_line_run, the voiced and the noise
 * burst of css_annex_init, ends DT_TAIL_SAMPLES_WB into a frame. */
static unsigned dt_tail_phase_get(unsigned rate)
{
	unsigned frame = rate / 100u;
	unsigned burst = path_samples_get(CSS_ANNEX_VOICED_MS, rate) +
			 path_samples_get(CSS_ANNEX_NOISE_MS, rate);

	return (dt_samples_get(DT_TAIL_SAMPLES_WB, rate) + frame -
		burst % frame) % frame;
}

/* G.168 test 3A at -10 dBm0 with a thin onset and a tail phase: each leg reads
 * l_res_dbm0 at L_Sgen,act or under. It holds DT_HELD_MAX or fewer of the 1120
 * frames of the double talk. */
static void dt3a_phase_test(unsigned rate)
{
	struct dt_line_result r;
	unsigned phase;

	phase = dt_thin_phase_get(rate);
	dt_line_run(rate, -10.0, 15.0, 0u, phase, &r);
	case_rate(rate, "lec converges in G.168 test 3A with the far onset a "
		  "few samples before a frame end");
	printf("# %u Hz, far onset at sample %u of the frame: %u frames "
	       "held\n", rate, phase, r.held);
	dt_line_print(rate, "test 3A, thin onset", &r);
	CHECK(r.l_res_dbm0 <= r.l_sgen_act);
	CHECK(r.held <= DT_HELD_MAX);
	phase = dt_tail_phase_get(rate);
	dt_line_run(rate, -10.0, 15.0, 0u, phase, &r);
	case_rate(rate, "lec converges in G.168 test 3A with the burst end a "
		  "few samples into a frame");
	printf("# %u Hz, far onset at sample %u of the frame: %u frames "
	       "held\n", rate, phase, r.held);
	dt_line_print(rate, "test 3A, tail frame", &r);
	CHECK(r.l_res_dbm0 <= r.l_sgen_act);
	CHECK(r.held <= DT_HELD_MAX);
}

/* The bounds of dt3b_test on the rise over the line of G.168 Figure 11: 10 dB
 * at X 0 and DT3B_RISE_6_DB at X 6 to 30. The latch wait, the rollback output
 * and the drop hold keep them. */
#define DT3B_RISE_0_DB		10.0
#define DT3B_RISE_6_DB		3.0
/* G.168 Figure 11: L_RES <= DT3B_FIG11_SLOPE * L_Rin,act + DT3B_FIG11_AT_0. */
#define DT3B_FIG11_SLOPE	(25.0 / 30.0)
#define DT3B_FIG11_AT_0		-30.0

static const double dt3b_x_db[3] = { 0.0, 6.0, 15.0 };
static const double dt3b_bound_db[3] = { DT3B_RISE_0_DB, DT3B_RISE_6_DB,
					 DT3B_RISE_6_DB };

/*
 * G.168 test 3B at L_Rin -10 dBm0, after 16 periods of the far end alone.
 * The near end stands X 0, 6 and 15 dB under the far end. The rise of each
 * leg must stand at its bound of dt3b_bound_db or under, and each leg must
 * start no trial.
 */
static void dt3b_test(unsigned rate)
{
	unsigned i;

	case_rate(rate, "lec bounds the residual of G.168 test 3B in double "
		  "talk after convergence");
	for (i = 0; i < 3u; i++) {
		struct dt_line_result r;
		double line;
		double rise;
		char what[48];

		dt_line_run(rate, -10.0, dt3b_x_db[i], DT_PERIODS,
			    dt_phase_get(rate), &r);
		line = DT3B_FIG11_SLOPE * r.l_rin_act + DT3B_FIG11_AT_0;
		rise = r.worst_dbm0 - line;
		snprintf(what, sizeof(what), "test 3B at X %.0f dB",
			 dt3b_x_db[i]);
		dt_line_print(rate, what, &r);
		printf("# %u Hz, X %.0f dB: rise %.2f dB over Figure 11\n",
		       rate, dt3b_x_db[i], rise);
		CHECK(rise <= dt3b_bound_db[i]);
		CHECK_UINT(r.trials, 0);
	}
}

/* The drop hold line: dt_line_run at -10 dBm0, converged. Then come a talker
 * x_db under the far end, a copy DROP_COPY_MS late, or a change from m6 to m1
 * with an optional near break. */
#define DROP_COPY_MS		2.0
#define DROP_COPY_FRAMES	3u
#define DROP_CHANGE_FRAMES	4u
#define DROP_MISFIT_BREAK_FRAME	11u
#define DROP_MISFIT_BREAK_DBM0	-6.0
#define DROP_AFTER_PERIODS	8u
#define DROP_TAIL_FRAMES	70u
#define DROP_NONE		0xffffffffu

enum drop_event {
	DROP_TALKER,
	DROP_COPY,
	DROP_CHANGE,
	DROP_CHANGE_BREAK,
};

/* The held frames, the hold's start and end, the talker's stop, and the meter
 * and tail levels. */
struct drop_line_result {
	unsigned start;
	unsigned end;
	unsigned held;
	unsigned stop;
	unsigned trials;
	double worst_dbm0;
	double tail_dbm0;
	double l_rin_act;
};

static void drop_line_run(unsigned rate, enum drop_event kind, double x_db,
			  unsigned event, bool nlp, struct drop_line_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct css_annex far_src;
	struct css_annex near_src;
	struct echo_path path;
	struct echo_path changed;
	bool copy = kind == DROP_COPY;
	bool talker = kind == DROP_TALKER;
	bool change = kind == DROP_CHANGE || kind == DROP_CHANGE_BREAK;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t hiss_state = 0x51ab0dd5u;
	uint32_t burst_state = 0x13572468u;
	unsigned frame = rate / 100u;
	unsigned delay = path_samples_get(DROP_COPY_MS, rate);
	unsigned long period = (unsigned long)DT_PERIOD_FRAMES * frame;
	unsigned long t0 = (unsigned long)DT_LEAD_FRAMES * frame +
			   dt_phase_get(rate);
	unsigned long dt0 = t0 + DT_PERIODS * period;
	unsigned long near_at = dt0 + path_samples_get(PHASE2_DELAY_MS, rate);
	unsigned long near_end = near_at + event * period;
	unsigned long on = talker ? near_at :
			   dt0 + (copy ? DROP_COPY_FRAMES : DROP_CHANGE_FRAMES) *
				 frame;
	unsigned long t1 = talker ? near_end + DROP_AFTER_PERIODS * period :
			   on + event * period;
	unsigned frames = (unsigned)(t1 / frame) + 1u;
	unsigned first = (unsigned)(on / frame);
	double gain = pow(10.0, -(DT_ERL_DB + x_db) / 20.0);
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double worst = 0.0;
	double tail = 0.0;
	int16_t *far_hist = calloc(delay + frame, sizeof(*far_hist));
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	r->start = DROP_NONE;
	r->end = DROP_NONE;
	r->stop = (unsigned)(near_end / frame) - first;
	r->l_rin_act = -10.0 + DT_ACT_SINGLE_DB;
	css_annex_init(&far_src, rate, -10.0, 0x0c55a11eu);
	css_annex_dt_init(&near_src, rate, r->l_rin_act - x_db -
			  DT_ACT_DOUBLE_DB, 0x5eed0005u);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, nlp);
	path_annex_d_init(&path, rate, change ? ANNEX_D_M6 : ANNEX_D_M1,
			  DT_ERL_DB, PHASE2_DELAY_MS);
	path_annex_d_init(&changed, rate, ANNEX_D_M1, DT_ERL_DB + x_db,
			  PHASE2_DELAY_MS);
	for (f = 0; f < frames && far_hist; f++) {
		int16_t far[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t echo_changed[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX];
		int16_t burst[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];

		for (i = 0; i < frame; i++) {
			unsigned long n = (unsigned long)f * frame + i;

			far[i] = dt_sample_get(&far_src, n, t0, t1);
			near[i] = talker ? dt_sample_get(&near_src, n, near_at,
							 near_end) : 0;
		}
		if (kind == DROP_CHANGE_BREAK &&
		    f == first + DROP_MISFIT_BREAK_FRAME) {
			noise_fill(burst, frame, DROP_MISFIT_BREAK_DBM0,
				   &burst_state);
			for (i = 0; i < frame; i++)
				near[i] = burst[i];
		}
		memmove(far_hist, far_hist + frame, delay * sizeof(*far_hist));
		memcpy(far_hist + delay, far, frame * sizeof(*far));
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		path_run(&changed, far, echo_changed);
		noise_fill(hiss, frame, DT_HISS_DBM0, &hiss_state);
		for (i = 0; i < frame; i++) {
			unsigned long n = (unsigned long)f * frame + i;
			double c = copy && n >= on ? gain * far_hist[i] : 0.0;
			double e = change && n >= on ? echo_changed[i] :
						       echo[i];

			mic[i] = sample_clamp(e + hiss[i] + near[i] + c);
		}
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (f >= first && reading.drop_hold) {
			r->held++;
			if (r->start == DROP_NONE)
				r->start = f - first;
		} else if (f >= first && r->start != DROP_NONE &&
			   r->end == DROP_NONE) {
			r->end = f - first;
		}
		for (i = 0; i < frame; i++) {
			double e = (double)mic[i] - near[i];

			meter = a * meter + (1.0 - a) * e * e;
			if (f >= first && meter > worst)
				worst = meter;
			if (f >= first + DROP_TAIL_FRAMES - 10u &&
			    f < first + DROP_TAIL_FRAMES)
				tail += e * e;
		}
	}
	udsp_ec_nlms_info_get(ec, &info);
	r->trials = info.probes;
	r->worst_dbm0 = rms_dbm0_get(sqrt(worst));
	r->tail_dbm0 = rms_dbm0_get(sqrt(tail / (10.0 * frame)));
	free(far_hist);
	udsp_ec_nlms.destroy(ec);
}

/* The periods of the talker and the frame bounds of drop_talker_test. */
#define DROP_TALK_PERIODS	8u
#define DROP_START_FRAMES	140u
#define DROP_CLEAN_FRAMES	150u

/*
 * The talker of G.168 test 3B at X 15 dB: the drop hold starts within
 * DROP_START_FRAMES of the talker and holds to its end. The rise stays at
 * DT3B_RISE_6_DB or under, clean pause runs end the hold within
 * DROP_CLEAN_FRAMES after the talker, and no trial starts.
 */
static void drop_talker_test(unsigned rate)
{
	struct drop_line_result r;
	double rise;

	case_rate(rate, "lec drop hold keeps a talker under the echo out of the "
		  "filter and ends after it");
	drop_line_run(rate, DROP_TALKER, 15.0, DROP_TALK_PERIODS, false, &r);
	rise = r.worst_dbm0 - (DT3B_FIG11_SLOPE * r.l_rin_act +
			       DT3B_FIG11_AT_0);
	printf("# X 15 dB talker for %u frames: hold from frame %d to frame "
	       "%d, %u held; rise %.2f dB over Figure 11; %u trials\n",
	       r.stop, (int)r.start, (int)r.end, r.held, rise, r.trials);
	CHECK(r.start <= DROP_START_FRAMES);
	CHECK(r.end != DROP_NONE && r.end >= r.stop);
	CHECK(r.end != DROP_NONE && r.end - r.stop <= DROP_CLEAN_FRAMES);
	CHECK(rise <= DT3B_RISE_6_DB);
	CHECK_UINT(r.trials, 0);
}

/* The periods of the copy and the bounds of drop_copy_test. */
#define DROP_COPY_PERIODS	4u
#define DROP_COPY_HOLD_FRAMES	30u
#define DROP_COPY_UNDER_DB	20.0

/*
 * A copy of the far end DROP_COPY_MS late and 10 dB under the echo, inside the
 * tail but not in the filter. The drop hold starts on it and ends within
 * DROP_COPY_HOLD_FRAMES on the reference coherence; the output then stands
 * DROP_COPY_UNDER_DB or more under the copy.
 */
static void drop_copy_test(unsigned rate)
{
	struct drop_line_result r;
	double copy_dbm0 = -10.0 + DT_ACT_SINGLE_DB - DT_ERL_DB - 10.0;

	case_rate(rate, "lec drop hold ends on a copy of the far end inside the "
		  "tail");
	drop_line_run(rate, DROP_COPY, 10.0, DROP_COPY_PERIODS, true, &r);
	printf("# copy 10 dB under the echo: hold from frame %d to frame %d, "
	       "%u held; output %.2f dBm0 at %u frames, against the copy at "
	       "%.2f dBm0; %u trials\n", (int)r.start, (int)r.end, r.held,
	       r.tail_dbm0, DROP_TAIL_FRAMES, copy_dbm0, r.trials);
	CHECK(r.start != DROP_NONE);
	CHECK(r.end != DROP_NONE && r.end <= DROP_COPY_HOLD_FRAMES);
	CHECK(r.tail_dbm0 <= copy_dbm0 - DROP_COPY_UNDER_DB);
}

/* The quieter path and the frame bound of drop_misfit_test. */
#define DROP_MISFIT_DB		3.0
#define DROP_MISFIT_PERIODS	4u
#define DROP_MISFIT_HOLD_FRAMES	13u

/*
 * A line converged on the dispersive m6 whose echo turns into m1 quieter: the
 * change starts the drop hold. Its residual spreads over the lags of m6 under
 * the coherence bar, and the misfit end stops the hold within
 * DROP_MISFIT_HOLD_FRAMES.
 */
static void drop_misfit_test(unsigned rate)
{
	struct drop_line_result r;

	case_rate(rate, "lec drop hold ends on a changed path that the estimate "
		  "misfits");
	drop_line_run(rate, DROP_CHANGE, DROP_MISFIT_DB, DROP_MISFIT_PERIODS,
		      false, &r);
	printf("# m6 to m1 %.0f dB quieter: hold from frame %d to frame %d, "
	       "%u held; output %.2f dBm0 at %u frames; %u trials\n",
	       DROP_MISFIT_DB, (int)r.start, (int)r.end, r.held, r.tail_dbm0,
	       DROP_TAIL_FRAMES, r.trials);
	CHECK(r.start != DROP_NONE);
	CHECK(r.end != DROP_NONE && r.end <= DROP_MISFIT_HOLD_FRAMES);
}

/* The misfit run of dsp/lec.c (LEC_DROP_HOLD_MISFIT_FRAMES) and the frame
 * bound of drop_misfit_break_test. */
#define DROP_MISFIT_RUN_FRAMES		8u
#define DROP_MISFIT_BREAK_HOLD_FRAMES	22u

/*
 * The line of drop_misfit_test with a one-frame near burst after the change. On
 * the burst the estimate holds under a quarter of the microphone, so the misfit
 * run starts over; the hold ends no sooner than DROP_MISFIT_RUN_FRAMES after it
 * and within DROP_MISFIT_BREAK_HOLD_FRAMES.
 */
static void drop_misfit_break_test(unsigned rate)
{
	struct drop_line_result r;

	case_rate(rate, "lec drop hold counts the misfit frames in a row");
	drop_line_run(rate, DROP_CHANGE_BREAK, DROP_MISFIT_DB,
		      DROP_MISFIT_PERIODS, false, &r);
	printf("# m6 to m1 %.0f dB quieter, near burst on frame %u: hold from "
	       "frame %d to frame %d, %u held; %u trials\n", DROP_MISFIT_DB,
	       DROP_MISFIT_BREAK_FRAME, (int)r.start, (int)r.end, r.held,
	       r.trials);
	CHECK(r.start != DROP_NONE);
	CHECK(r.end != DROP_NONE && r.end >= DROP_MISFIT_BREAK_FRAME +
					     DROP_MISFIT_RUN_FRAMES);
	CHECK(r.end != DROP_NONE && r.end <= DROP_MISFIT_BREAK_HOLD_FRAMES);
}

/* The line of onset_mic_test: white far bursts through m1 about 16 dB over a
 * white near noise, so the trust does not hold. The classic arm reads a near
 * end on an onset frame as echo; a near burst then plays on one onset frame. */
#define MICGATE_FAR_DBM0	-10.0
#define MICGATE_ERL_DB		6.0
#define MICGATE_FLOOR_DBM0	-32.0
#define MICGATE_ON_FRAMES	30u
#define MICGATE_OFF_FRAMES	20u
#define MICGATE_QUIET_PERIODS	2u
#define MICGATE_BURSTS		12u
/* LEC_GEIGEL_FLOOR_HEADROOM and LEC_GEIGEL_SOFT_RATIO of dsp/lec.c: 3 and
 * 12 dB over the floor, as powers. */
#define MICGATE_HEADROOM	2.0
#define MICGATE_SOFT_RATIO	16.0

/* What onset_mic_test reads on the onset frame of the near burst. */
struct micgate_result {
	double over_db;
	double move;
	unsigned active;
	unsigned window_run;
	unsigned peak;
	bool fired;
	bool trusted;
	bool floor_valid;
};

/* One line of onset_mic_test with its near burst over_db over the floor. */
static void micgate_line_run(unsigned rate, double over_db,
			     struct micgate_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t far_state = 0x00012345u;
	uint32_t noise_state = 0x51ab0dd5u;
	uint32_t near_state = 0x0badcafeu;
	unsigned period = MICGATE_ON_FRAMES + MICGATE_OFF_FRAMES;
	unsigned test = MICGATE_BURSTS * period;
	unsigned frame = rate / 100u;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, false);
	path_annex_d_init(&path, rate, ANNEX_D_M1, MICGATE_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; f <= test; f++) {
		int16_t far[UDSP_FRAME_MAX] = { 0 };
		int16_t near[UDSP_FRAME_MAX] = { 0 };
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t noise[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];

		if (f >= MICGATE_QUIET_PERIODS * period &&
		    f % period < MICGATE_ON_FRAMES)
			noise_fill(far, frame, MICGATE_FAR_DBM0, &far_state);
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		noise_fill(noise, frame, MICGATE_FLOOR_DBM0, &noise_state);
		if (f == test)
			noise_fill(near, frame, MICGATE_FLOOR_DBM0 + over_db,
				   &near_state);
		for (i = 0; i < frame; i++)
			mic[i] = sample_clamp((double)echo[i] + noise[i] +
					      near[i]);
		udsp_ec_nlms_info_get(ec, &before);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (f < test) {
			r->active += reading.ref_power >= reading.ref_bar;
			continue;
		}
		r->floor_valid = reading.noise_power > 0.0f;
		r->over_db = r->floor_valid ?
			     10.0 * log10(reading.mic_power /
					  reading.noise_power) : 0.0;
		r->move = (double)info.coef_energy - before.coef_energy;
		r->window_run = reading.window_run;
		r->peak = before.coef_peak_tap;
		r->fired = reading.dt_fired;
		r->trusted = reading.trusted;
	}
	udsp_ec_nlms.destroy(ec);
}

/* The bounds of onset_mic_test on the move of the coefficient energy over the
 * onset frame. They stand between the move of the plain denominator and that
 * under the step floor. */
#define MICGATE_PLAIN_DB	8.0
#define MICGATE_FLOOR_DB	14.0
#define MICGATE_PLAIN_MOVE	2.0e-4
#define MICGATE_FLOOR_MOVE	1.0e-4

/*
 * The microphone condition of the step floor of an onset frame
 * (onset_floor_get). A near burst between 3 and 12 dB over the floor takes the
 * plain denominator and moves the energy by MICGATE_PLAIN_MOVE or more. One 12
 * dB or more over it moves it by MICGATE_FLOOR_MOVE or less.
 */
static void onset_mic_test(unsigned rate)
{
	static const double over_db[2] = { MICGATE_PLAIN_DB, MICGATE_FLOOR_DB };
	unsigned i;

	case_rate(rate, "lec takes the plain step on an onset frame near the "
		  "floor and the onset floor 12 dB over it");
	for (i = 0; i < 2u; i++) {
		struct micgate_result r;
		bool plain = i == 0u;

		micgate_line_run(rate, over_db[i], &r);
		printf("# %u Hz, near burst %.0f dB over the floor: microphone "
		       "%.1f dB over the tracked floor, coefficient energy "
		       "move %+.2e; window run %u, peak tap %u, %u active "
		       "frames before it, fired %d, trusted %d\n", rate,
		       over_db[i], r.over_db, r.move, r.window_run, r.peak,
		       r.active, r.fired, r.trusted);
		CHECK(r.active >= NEAR_START_BOOT_FRAMES);
		CHECK(r.floor_valid);
		CHECK(r.window_run == 1u);
		CHECK(r.peak >= rate / 100u);
		CHECK(!r.fired && !r.trusted);
		if (plain) {
			CHECK(r.over_db > 10.0 * log10(MICGATE_HEADROOM) &&
			      r.over_db < 10.0 * log10(MICGATE_SOFT_RATIO));
			CHECK(r.move >= MICGATE_PLAIN_MOVE);
		} else {
			CHECK(r.over_db >= 10.0 * log10(MICGATE_SOFT_RATIO));
			CHECK(fabs(r.move) <= MICGATE_FLOOR_MOVE);
		}
	}
}

/*
 * The line of onset_peak_test: each bootstrap onset frame also carries the
 * reference ONSET_PEAK_LAG samples late, a near end that the plain step learns
 * on the first taps. With early_echo every later active frame carries the copy.
 */
#define ONSET_PEAK_FROM		30u
#define ONSET_PEAK_GAIN		1.4
#define ONSET_PEAK_LAG		1u
#define ONSET_PEAK_NEAR_DB	14.0
#define ONSET_PEAK_QUIET_DBM0	-18.0
#define ONSET_PEAK_TRUST_FLOOR_DBM0	-60.0
#define ONSET_PEAK_TRUST_NEAR_DB	30.0
/* A bound on the frames of a line: the trust holds by frame 500. */
#define ONSET_PEAK_MAX_FRAMES	3000u

struct onset_peak_line {
	double floor_dbm0;
	double inject_dbm0;
	double near_db;
	bool early_echo;
};

/* What onset_peak_test reads on the bootstrap onset frames that carry the
 * reference, and on the onset frame with the near noise. */
struct onset_peak_result {
	double inject_over_db;
	double over_db;
	double energy;
	double move;
	unsigned injected;
	unsigned injected_fired;
	unsigned first_peak;
	unsigned active;
	unsigned window_run;
	unsigned peak;
	bool fired;
	bool trusted;
};

/* The far frame of onset_peak_test at frame f: level_dbm0 on the frames of
 * a burst, silence in a pause. Returns whether f is an onset frame. */
static bool onset_peak_far_fill(unsigned f, unsigned frame, int16_t *far,
				double level_dbm0, uint32_t *state)
{
	unsigned period = MICGATE_ON_FRAMES + MICGATE_OFF_FRAMES;
	bool on = f >= MICGATE_QUIET_PERIODS * period &&
		  f % period < MICGATE_ON_FRAMES;

	memset(far, 0, frame * sizeof(*far));
	if (on)
		noise_fill(far, frame, level_dbm0, state);
	return on && f % period == 0u;
}

/* One line of onset_peak_test, up to the onset frame with the near noise. */
static void onset_peak_line_run(unsigned rate,
				const struct onset_peak_line *line,
				struct onset_peak_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t far_state = 0x00012345u;
	uint32_t noise_state = 0x51ab0dd5u;
	uint32_t near_state = 0x0badcafeu;
	unsigned period = MICGATE_ON_FRAMES + MICGATE_OFF_FRAMES;
	unsigned frame = rate / 100u;
	unsigned active = 0;
	bool trusted = false;
	bool frozen = false;
	bool done = false;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1, MICGATE_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; !done && f < ONSET_PEAK_MAX_FRAMES; f++) {
		int16_t far[UDSP_FRAME_MAX];
		int16_t near[UDSP_FRAME_MAX] = { 0 };
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t noise[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		bool boot = active < NEAR_START_BOOT_FRAMES;
		bool onset = f % period == 0u;
		bool inject = onset && boot && active >= ONSET_PEAK_FROM;
		bool copy;

		onset = onset_peak_far_fill(f, frame, far, inject ?
					    line->inject_dbm0 :
					    MICGATE_FAR_DBM0, &far_state);
		inject = inject && onset;
		copy = inject || (line->early_echo && !boot &&
				  (f % period) < MICGATE_ON_FRAMES);
		done = onset && !boot && (!line->early_echo || trusted);
		noise_fill(noise, frame, line->floor_dbm0, &noise_state);
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		if (done)
			noise_fill(near, frame,
				   line->floor_dbm0 + line->near_db,
				   &near_state);
		for (i = 0; i < frame; i++) {
			double v = (double)echo[i] + noise[i] + near[i];

			if (copy && i >= ONSET_PEAK_LAG)
				v += ONSET_PEAK_GAIN * ref[i - ONSET_PEAK_LAG];
			mic[i] = sample_clamp(v);
		}
		if (onset && frozen) {
			udsp_ec_nlms.freeze(ec, false);
			frozen = false;
		}
		udsp_ec_nlms_info_get(ec, &before);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		trusted = trusted || reading.trusted;
		if (inject) {
			double over = 10.0 * log10(reading.mic_power /
						   reading.noise_power);

			if (!r->injected || over > r->inject_over_db)
				r->inject_over_db = over;
			if (!r->injected)
				r->first_peak = before.coef_peak_tap;
			r->injected++;
			r->injected_fired += reading.dt_fired;
			udsp_ec_nlms.freeze(ec, true);
			frozen = true;
		}
		if (!done) {
			active += reading.ref_power >= reading.ref_bar;
			continue;
		}
		r->active = active;
		r->over_db = 10.0 * log10(reading.mic_power /
					  reading.noise_power);
		r->energy = before.coef_energy;
		r->move = (double)info.coef_energy - before.coef_energy;
		r->window_run = reading.window_run;
		r->peak = before.coef_peak_tap;
		r->fired = reading.dt_fired;
		r->trusted = reading.trusted;
	}
	udsp_ec_nlms.destroy(ec);
}
/* The bounds of onset_peak_test on the move of the coefficient energy over the
 * onset frame with near noise, on each leg. They stand between the moves under
 * the step floor and under the plain step. */
#define ONSET_PEAK_MOVE		0.15
#define ONSET_PEAK_TRUST_MOVE	0.01

/* The legs of onset_peak_test: near end (bootstrap onsets carry it, set the
 * onset peak, and the floor holds after) and quiet onset (no near end, no
 * peak). The trust leg's echo in the frame trains the filter to the trust. */
static const struct onset_peak_line onset_peak_lines[3] = {
	{ MICGATE_FLOOR_DBM0, MICGATE_FAR_DBM0, ONSET_PEAK_NEAR_DB, false },
	{ MICGATE_FLOOR_DBM0, ONSET_PEAK_QUIET_DBM0, ONSET_PEAK_NEAR_DB,
	  false },
	{ ONSET_PEAK_TRUST_FLOOR_DBM0, MICGATE_FAR_DBM0,
	  ONSET_PEAK_TRUST_NEAR_DB, true },
};

static const char *const onset_peak_names[3] = {
	"near end", "quiet onset", "trust",
};

/*
 * The onset peak (onset_peak_update): after the bootstrap the near-end leg
 * takes the floor and moves the energy by ONSET_PEAK_MOVE or less. The
 * quiet-onset leg takes the plain step, and the trust leg moves by
 * ONSET_PEAK_TRUST_MOVE or more.
 */
static void onset_peak_test(unsigned rate)
{
	unsigned frame = rate / 100u;
	unsigned i;

	case_rate(rate, "lec keeps the onset floor on a peak that plain onset "
		  "steps put under the frame");
	for (i = 0; i < 3u; i++) {
		struct onset_peak_result r;
		double soft_db = 10.0 * log10(MICGATE_SOFT_RATIO);
		double share;

		onset_peak_line_run(rate, &onset_peak_lines[i], &r);
		share = r.energy > 0.0 ? fabs(r.move) / r.energy : 0.0;
		printf("# %u Hz, %s: %u bootstrap onset frames carry the "
		       "reference, up to %.1f dB over the floor, %u of them "
		       "fire, peak tap %u on the first; then: peak tap %u, "
		       "microphone %.1f dB over the floor, coefficient energy "
		       "%.4f, move %+.2e (%.2f percent); window run %u, %u "
		       "active frames before it, fired %d, trusted %d\n", rate,
		       onset_peak_names[i], r.injected, r.inject_over_db,
		       r.injected_fired, r.first_peak, r.peak, r.over_db,
		       r.energy, r.move, 100.0 * share, r.window_run,
		       r.active, r.fired, r.trusted);
		CHECK(r.injected_fired < r.injected);
		CHECK(r.first_peak >= frame);
		CHECK(r.active >= NEAR_START_BOOT_FRAMES);
		CHECK(r.window_run == 1u);
		CHECK(r.peak < frame);
		CHECK(r.over_db >= soft_db);
		CHECK(!r.fired);
		if (i == 0u) {
			CHECK(r.inject_over_db >= soft_db);
			CHECK(!r.trusted);
			CHECK(share <= ONSET_PEAK_MOVE);
		} else if (i == 1u) {
			CHECK(r.inject_over_db < soft_db);
			CHECK(!r.trusted);
			CHECK(share > ONSET_PEAK_MOVE);
		} else {
			CHECK(r.trusted);
			CHECK(share >= ONSET_PEAK_TRUST_MOVE);
		}
	}
}

/* The clear and toggle line, the processor on: the bootstrap of onset_peak_test
 * sets the onset peak. A silent far end and a frozen filter follow, then
 * peak-tap toggles, an optional reset and a near onset. */
/* The count of dsp/lec.c (LEC_ONSET_PEAK_CLEAR_FRAMES). */
#define ONSET_CLEAR_FRAMES	8u
#define ONSET_CLEAR_FROM_WB	60u
#define ONSET_CLEAR_LEAD	20u
#define ONSET_CLEAR_OUT_FRAMES	10u
#define ONSET_CLEAR_OUT_MAX	12u
#define ONSET_CLEAR_SPLIT	20u
#define ONSET_CLEAR_RETURN_MAX	4u

struct onset_clear_line {
	unsigned runs[2];
	unsigned toggles;
	unsigned gap;
	bool split;
	bool reset;
};

/* What onset_clear_line_run reads: the frames in a row that ended with the peak
 * tap past the frame length on each toggle, and the toggles that returned. Then
 * the near onset and the loudest residual meter reading after it. */
struct onset_clear_result {
	unsigned runs[2];
	unsigned toggles;
	unsigned active;
	unsigned since_far;
	unsigned window_run;
	unsigned peak;
	double over_db;
	double energy;
	double move;
	double l_res_dbm0;
	bool fired;
	bool trusted;
};

enum onset_clear_step {
	CLEAR_BOOT,
	CLEAR_WAIT,
	CLEAR_OUT,
	CLEAR_HOLD,
	CLEAR_RETURN,
	CLEAR_NEAR,
	CLEAR_DONE,
};

/* The microphone of a frame of the line: the echo, the line noise, the near
 * noise and, with copy, the reference ONSET_PEAK_LAG samples late at
 * ONSET_PEAK_GAIN. */
static void onset_clear_mic_fill(int16_t *mic, const int16_t *echo,
				 const int16_t *noise, const int16_t *near,
				 const int16_t *ref, unsigned frame, bool copy)
{
	unsigned i;

	for (i = 0; i < frame; i++) {
		double v = (double)echo[i] + noise[i] + near[i];

		if (copy && i >= ONSET_PEAK_LAG)
			v += ONSET_PEAK_GAIN * ref[i - ONSET_PEAK_LAG];
		mic[i] = sample_clamp(v);
	}
}

/* One line of onset_peak_clear_test or onset_peak_toggle_test, to the end
 * of the meter after the near onset. */
static void onset_clear_line_run(unsigned rate,
				 const struct onset_clear_line *line,
				 struct onset_clear_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t far_state = 0x00012345u;
	uint32_t noise_state = 0x51ab0dd5u;
	uint32_t near_state = 0x0badcafeu;
	unsigned period = MICGATE_ON_FRAMES + MICGATE_OFF_FRAMES;
	unsigned frame = rate / 100u;
	unsigned from = rate == UDSP_SAMPLE_RATE ? ONSET_PEAK_FROM :
						 ONSET_CLEAR_FROM_WB;
	enum onset_clear_step step = CLEAR_BOOT;
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double loudest = 0.0;
	unsigned active = 0;
	unsigned next = 0;
	unsigned out = 0;
	unsigned at = 0;
	unsigned back = 0;
	unsigned last_far = 0;
	unsigned near_at = 0;
	bool second = false;
	bool frozen = false;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1, MICGATE_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; step != CLEAR_DONE && f < ONSET_PEAK_MAX_FRAMES; f++) {
		int16_t far[UDSP_FRAME_MAX] = { 0 };
		int16_t near[UDSP_FRAME_MAX] = { 0 };
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t noise[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		double far_dbm0 = MICGATE_FAR_DBM0;
		bool on = false;
		bool copy = false;
		bool adapt = false;
		bool inject = false;

		if (step == CLEAR_BOOT && active >= NEAR_START_BOOT_FRAMES) {
			if (line->reset && !second) {
				udsp_ec_nlms_reset(ec);
				udsp_ec_nlms.freeze(ec, true);
				frozen = true;
				second = true;
				active = 0;
			} else {
				step = CLEAR_WAIT;
				next = f + ONSET_CLEAR_LEAD;
			}
		}
		if (step == CLEAR_WAIT && f == next)
			step = r->toggles < line->toggles ? CLEAR_OUT :
							    CLEAR_NEAR;
		if (step == CLEAR_HOLD && f == at)
			step = CLEAR_RETURN;
		if (step == CLEAR_NEAR && !near_at)
			near_at = f;
		switch (step) {
		case CLEAR_BOOT:
			on = f >= MICGATE_QUIET_PERIODS * period &&
			     f % period < MICGATE_ON_FRAMES;
			inject = on && f % period == 0u &&
				 active >= (second ? 0u : from);
			copy = inject;
			if (inject && second)
				far_dbm0 = ONSET_PEAK_QUIET_DBM0;
			adapt = !frozen || (on && f % period == 0u);
			break;
		case CLEAR_OUT:
			on = f == next;
			adapt = on;
			break;
		case CLEAR_HOLD:
			on = f + 1u == at;
			break;
		case CLEAR_RETURN:
			on = true;
			copy = true;
			adapt = true;
			break;
		case CLEAR_NEAR:
			on = f - near_at < MICGATE_ON_FRAMES;
			adapt = true;
			break;
		default:
			break;
		}
		if (on)
			noise_fill(far, frame, far_dbm0, &far_state);
		noise_fill(noise, frame, MICGATE_FLOOR_DBM0, &noise_state);
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		if (step == CLEAR_NEAR && f == near_at)
			noise_fill(near, frame,
				   MICGATE_FLOOR_DBM0 + ONSET_PEAK_NEAR_DB,
				   &near_state);
		onset_clear_mic_fill(mic, echo, noise, near, ref, frame, copy);
		if (adapt == frozen) {
			udsp_ec_nlms.freeze(ec, !adapt);
			frozen = !adapt;
		}
		udsp_ec_nlms_info_get(ec, &before);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (step == CLEAR_BOOT) {
			if (inject) {
				udsp_ec_nlms.freeze(ec, true);
				frozen = true;
			}
			active += reading.ref_power >= reading.ref_bar;
			continue;
		}
		switch (step) {
		case CLEAR_OUT:
			if (info.coef_peak_tap >= frame) {
				at = f + line->runs[r->toggles];
				step = CLEAR_HOLD;
			} else if (f == next && ++out < ONSET_CLEAR_OUT_MAX) {
				next = f + ONSET_CLEAR_OUT_FRAMES;
			} else if (f == next) {
				step = CLEAR_DONE;
			}
			break;
		case CLEAR_RETURN:
			back++;
			if (info.coef_peak_tap < frame) {
				r->runs[r->toggles] = f - at +
					line->runs[r->toggles];
				r->toggles++;
				back = 0;
				out = 0;
				step = CLEAR_WAIT;
				next = r->toggles < line->toggles ?
				       f + ONSET_CLEAR_SPLIT :
				       last_far + line->gap;
			} else if (back >= ONSET_CLEAR_RETURN_MAX) {
				step = CLEAR_DONE;
			}
			break;
		case CLEAR_NEAR:
			if (f == near_at) {
				r->active = active;
				r->since_far = line->toggles ? f - last_far : 0u;
				r->over_db = 10.0 * log10(reading.mic_power /
							  reading.noise_power);
				r->energy = before.coef_energy;
				r->move = (double)info.coef_energy -
					  before.coef_energy;
				r->window_run = reading.window_run;
				r->peak = before.coef_peak_tap;
				r->fired = reading.dt_fired;
				r->trusted = reading.trusted;
				break;
			}
			for (i = 0; i < frame; i++) {
				double e = (double)mic[i] - noise[i];

				meter = a * meter + (1.0 - a) * e * e;
				if (meter > loudest)
					loudest = meter;
			}
			if (f - near_at >= MICGATE_ON_FRAMES + MICGATE_OFF_FRAMES)
				step = CLEAR_DONE;
			break;
		default:
			break;
		}
		if (info.coef_peak_tap >= frame && step != CLEAR_NEAR)
			last_far = f;
	}
	r->l_res_dbm0 = rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/* The bound on the move of the coefficient energy over the near onset after
 * toggles. It stands between the move under the floor and the plain step once
 * the onset peak clears. The reset leg reads against ONSET_PEAK_MOVE. */
#define ONSET_CLEAR_MOVE	0.05

static void onset_clear_print(unsigned rate, const char *what,
			      const struct onset_clear_line *line,
			      const struct onset_clear_result *r)
{
	double share = r->energy > 0.0 ? fabs(r->move) / r->energy : 0.0;

	printf("# %u Hz, %s: %u of %u toggles, runs at or past the frame %u "
	       "and %u, near onset %u frames after the last; peak tap %u, "
	       "microphone %.1f dB over the floor, coefficient energy %.4f, "
	       "move %+.2e (%.2f percent); window run %u, %u active frames "
	       "before it, fired %d, trusted %d; l_res %.2f dBm0\n", rate,
	       what, r->toggles, line->toggles, r->runs[0], r->runs[1],
	       r->since_far, r->peak, r->over_db, r->energy, r->move,
	       100.0 * share, r->window_run, r->active, r->fired, r->trusted,
	       r->l_res_dbm0);
}

/* The conditions of the near onset that every leg checks first: the toggles
 * ran, gap frames came before, and the onset meets the other floor conditions
 * on an untrusted filter. Returns the move as a fraction of the energy. */
static double onset_clear_check(unsigned rate,
				const struct onset_clear_line *line,
				const struct onset_clear_result *r)
{
	unsigned i;

	CHECK(r->toggles == line->toggles);
	if (line->toggles)
		CHECK(r->since_far == line->gap);
	for (i = 0; i < line->toggles; i++) {
		if (line->split)
			CHECK(r->runs[i] < ONSET_CLEAR_FRAMES);
		else
			CHECK(r->runs[i] == line->runs[i]);
	}
	if (line->split)
		CHECK(r->runs[0] + r->runs[1] >= ONSET_CLEAR_FRAMES);
	CHECK(r->active >= NEAR_START_BOOT_FRAMES);
	CHECK(r->window_run == 1u);
	CHECK(r->peak < rate / 100u);
	CHECK(r->over_db >= 10.0 * log10(MICGATE_SOFT_RATIO));
	CHECK(!r->fired);
	CHECK(!r->trusted);
	return r->energy > 0.0 ? fabs(r->move) / r->energy : 0.0;
}

/* The legs of onset_peak_clear_test: 7 frames past the frame length, under
 * LEC_ONSET_PEAK_CLEAR_FRAMES, keep the onset peak, and 8 clear it. A split run
 * restarts the count, and a reset clears it while the quiet copies set none. */
static const struct onset_clear_line onset_clear_lines[4] = {
	{ { 7u, 0u }, 1u, 22u, false, false },
	{ { 8u, 0u }, 1u, 22u, false, false },
	{ { 7u, 1u }, 2u, 22u, true, false },
	{ { 0u, 0u }, 0u, 0u, false, true },
};

static const char *const onset_clear_names[4] = {
	"7 frames", "8 frames", "split", "reset",
};

/*
 * The clear of the onset peak on an untrusted filter: the floor holds the move
 * over the near onset to ONSET_CLEAR_MOVE on the 7-frame and split legs. The
 * plain step moves it more on the 8-frame leg, and more than ONSET_PEAK_MOVE on
 * the reset leg.
 */
static void onset_peak_clear_test(unsigned rate)
{
	unsigned i;

	case_rate(rate, "lec clears the onset peak of an untrusted filter after "
		  "8 frames in a row past the frame");
	for (i = 0; i < 4u; i++) {
		struct onset_clear_result r;
		double share;

		onset_clear_line_run(rate, &onset_clear_lines[i], &r);
		onset_clear_print(rate, onset_clear_names[i],
				  &onset_clear_lines[i], &r);
		share = onset_clear_check(rate, &onset_clear_lines[i], &r);
		if (i == 0u || i == 2u)
			CHECK(share <= ONSET_CLEAR_MOVE);
		else if (i == 1u)
			CHECK(share > ONSET_CLEAR_MOVE);
		else
			CHECK(share > ONSET_PEAK_MOVE);
	}
}

/* The legs of onset_peak_toggle_test: a plain step also removes what the
 * bootstrap put on the first taps, so the floor check carries the verdict. The
 * meter bounds the residual echo that the floor leaves. */
static const struct onset_clear_line onset_toggle_lines[2] = {
	{ { 1u, 0u }, 1u, 24u, false, false },
	{ { 2u, 0u }, 1u, 21u, false, false },
};

/*
 * The shape of a bench capture of G.168 test 3A. After the bootstrap sets the
 * onset peak, the peak tap visits the echo tap for 1 or 2 frames before a near
 * onset. That onset still takes the floor (ONSET_CLEAR_MOVE), and the residual
 * of its burst stays under L_Sgen,act.
 */
static void onset_peak_toggle_test(unsigned rate)
{
	static const char *const names[2] = { "1 frame", "2 frames" };
	double line_dbm0 = MICGATE_FLOOR_DBM0 + ONSET_PEAK_NEAR_DB;
	unsigned i;

	case_rate(rate, "lec keeps the onset peak through a toggle of the peak "
		  "tap to the echo tap");
	for (i = 0; i < 2u; i++) {
		struct onset_clear_result r;

		onset_clear_line_run(rate, &onset_toggle_lines[i], &r);
		onset_clear_print(rate, names[i], &onset_toggle_lines[i], &r);
		CHECK(onset_clear_check(rate, &onset_toggle_lines[i], &r) <=
		      ONSET_CLEAR_MOVE);
		CHECK(r.l_res_dbm0 <= line_dbm0);
	}
}

/* The line of onset_start_test, G.168 test 2C(c) with the processor off: the
 * idle hiss, then a Hoth-shaped background at ONSET_START_SGEN_DBM0 before the
 * composite source through m1. */
#define ONSET_START_IDLE_FRAMES	30u
#define ONSET_START_LEAD_FRAMES	20u
#define ONSET_START_FAR_FRAMES	(2u * FRAMES_PER_SECOND)
#define ONSET_START_SGEN_DBM0	-30.0
#define ONSET_START_ERL_DB	6.0
#define ONSET_START_X_MAX_DB	17.0
#define ONSET_START_X_OFFSET_DB	6.0
#define ONSET_START_ONSETS	3u
#define ONSET_START_SEEDS	4u

/* The seeds of the Hoth-shaped noise of onset_start_test, which bound the onset
 * moves with the floor under ONSET_PEAK_MOVE. */
static const uint32_t onset_start_seeds[ONSET_START_SEEDS] = {
	0x2c0c2c0cu, 0x01234567u, 0x9e3779b9u, 0x7f4a7c15u,
};

/* onsets, share and low_peak read the bootstrap's near onsets; conv_db is the
 * combined loss at 1 s, against the objective X. */
struct onset_start_result {
	unsigned onsets;
	unsigned low_peak;
	double share;
	double conv_db;
};

/* The near background of a frame of the line of onset_start_test. */
static void onset_start_near_fill(struct room_noise *room,
				  struct udsp_resamp *up, unsigned rate,
				  int16_t *near)
{
	int16_t near8[UDSP_FRAME_SAMPLES];

	room_noise_fill(near8, UDSP_FRAME_SAMPLES, room);
	if (rate == UDSP_SAMPLE_RATE_WB)
		udsp_resamp_up2(up, near8, UDSP_FRAME_SAMPLES, near);
	else
		memcpy(near, near8, sizeof(near8));
}

/* One onset frame of the bootstrap of the line of onset_start_test, read
 * against the floor conditions of onset_floor_get but the bootstrap. */
static void onset_start_onset_read(struct onset_start_result *r,
				   const struct udsp_ec_nlms_info *before,
				   const struct udsp_ec_nlms_info *info,
				   const struct udsp_ec_nlms_frame *reading,
				   unsigned frame)
{
	double over;
	double share;

	if (reading->window_run != 1u || before->coef_peak_tap < frame)
		return;
	over = reading->mic_power / reading->noise_power;
	if (over < MICGATE_SOFT_RATIO)
		return;
	share = fabs((double)info->coef_energy - before->coef_energy) /
		before->coef_energy;
	if (!r->onsets || share > r->share)
		r->share = share;
	if (!r->onsets || info->coef_peak_tap < r->low_peak)
		r->low_peak = info->coef_peak_tap;
	r->onsets++;
}

static void onset_start_line_run(unsigned rate, uint32_t seed,
				 struct onset_start_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct room_noise room;
	struct udsp_resamp up;
	struct css_annex css;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t hiss_state = 0x51ab0dd5u;
	unsigned far0 = ONSET_START_IDLE_FRAMES + ONSET_START_LEAD_FRAMES;
	unsigned long from;
	unsigned long to;
	unsigned long n = 0;
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double loudest = 0.0;
	unsigned active = 0;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	room_noise_init(&room, ROOM_HOTH, ONSET_START_SGEN_DBM0, seed);
	udsp_resamp_reset(&up);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, false);
	path_annex_d_init(&path, rate, ANNEX_D_M1, ONSET_START_ERL_DB,
			  PHASE2_DELAY_MS);
	from = (unsigned long)far0 * path.frame + rate +
	       path_samples_get(PHASE2_DELAY_MS, rate);
	to = from + (unsigned long)rate * 7u / 10u;
	for (f = 0; f < far0 + ONSET_START_FAR_FRAMES; f++) {
		int16_t far[UDSP_FRAME_MAX] = { 0 };
		int16_t near[UDSP_FRAME_MAX] = { 0 };
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];

		if (f >= far0)
			css_annex_fill(far, path.frame, &css);
		if (f >= ONSET_START_IDLE_FRAMES)
			onset_start_near_fill(&room, &up, rate, near);
		level_loss_apply(ref, far, path.frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		noise_fill(hiss, path.frame, PHASE2_HISS_DBM0, &hiss_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + hiss[i] +
					      near[i]);
		udsp_ec_nlms_info_get(ec, &before);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (active < NEAR_START_BOOT_FRAMES)
			onset_start_onset_read(r, &before, &info, &reading,
					       path.frame);
		active += reading.ref_power >= reading.ref_bar;
		for (i = 0; i < path.frame; i++, n++) {
			meter = a * meter + (1.0 - a) * mic[i] * mic[i];
			if (n >= from && n < to && meter > loudest)
				loudest = meter;
		}
	}
	r->conv_db = PHASE2_FAR_DBM0 + DT_ACT_SINGLE_DB -
		     rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/*
 * The onset floor from the first active frame with the processor off. On each
 * seed, ONSET_START_ONSETS or more bootstrap onsets with the near end take the
 * floor and end with the peak past the frame. The combined loss at 1 s reaches
 * X.
 */
static void onset_start_test(unsigned rate)
{
	double x_db = fmin(ONSET_START_X_MAX_DB,
			   PHASE2_FAR_DBM0 + DT_ACT_SINGLE_DB -
			   ONSET_START_SGEN_DBM0 - ONSET_START_X_OFFSET_DB);
	unsigned k;

	case_rate(rate, "lec holds the onset floor from the first active frame "
		  "with the processor off");
	for (k = 0; k < ONSET_START_SEEDS; k++) {
		struct onset_start_result r;

		onset_start_line_run(rate, onset_start_seeds[k], &r);
		printf("# %u Hz, Hoth seed %u: %u bootstrap onset frames with "
		       "the near end past the frame, move up to %.2f percent, "
		       "least peak tap after them %u; combined loss %.2f dB at "
		       "1 s against X %.2f dB\n", rate, k, r.onsets,
		       100.0 * r.share, r.low_peak, r.conv_db, x_db);
		CHECK(r.onsets >= ONSET_START_ONSETS);
		CHECK(r.share <= ONSET_PEAK_MOVE);
		CHECK(r.low_peak >= rate / 100u);
		CHECK(r.conv_db >= x_db);
	}
}

/* The line of onset_zero_test: test 2C(c) on a quiet line, with the
 * processor off. The idle hiss runs 2 s. The Hoth-shaped noise of
 * onset_start_test starts ONSET_START_LEAD_FRAMES before the composite
 * source through m1. At 16 kHz the source is raised from 8 kHz, as the
 * reference of a line that a codec fills. A burst then starts over several
 * small samples. phase delays the source within the frame. A reset or the
 * watchdog can zero the filter before the idle hiss (enum onset_zero_start).
 */
#define ONSET_ZERO_HISS_DBM0	-80.0
#define ONSET_ZERO_IDLE_FRAMES	(2u * FRAMES_PER_SECOND)
#define ONSET_ZERO_PRE_FRAMES	(3u * FRAMES_PER_SECOND)
#define ONSET_ZERO_PHASES	8u
#define ONSET_ZERO_SEEDS	2u
#define ONSET_ZERO_ENERGY	2.0
#define ONSET_ZERO_CLEAR_FRAMES	8u
/* The white far end of the watchdog start, and the bound of that start. The
 * far end keeps the echo of a hybrid that amplifies under the clipping. */
#define ONSET_ZERO_WATCH_FAR_DBM0	-20.0
#define ONSET_ZERO_WATCH_MAX_FRAMES	(30u * FRAMES_PER_SECOND)

/* The start of the filter of the line: a new filter, a filter that a reset
 * zeroes after ONSET_ZERO_PRE_FRAMES of the far end alone, or a filter that
 * the watchdog zeroes (onset_zero_watch_run). */
enum onset_zero_start {
	ONSET_ZERO_NEW,
	ONSET_ZERO_RESET,
	ONSET_ZERO_WATCHDOG,
	ONSET_ZERO_STARTS
};

/* zero_onsets and zero_energy read the near onsets on a peak under the frame
 * from the first far frame, the plain steps of a zeroed filter. conv_db is
 * the combined loss at 1 s, as in onset_start_test. cleared, zeroed and
 * watch_frames report the watchdog start. */
struct onset_zero_result {
	unsigned zero_onsets;
	double zero_energy;
	double conv_db;
	bool cleared;
	bool zeroed;
	unsigned watch_frames;
};

/* The far end of a frame of the lines of onset_zero_test and
 * onset_zero_boot_test, phase samples late. */
struct onset_zero_far {
	struct css_annex css;
	struct udsp_resamp up;
	int16_t line[UDSP_FRAME_MAX];
	unsigned phase;
};

static void onset_zero_far_init(struct onset_zero_far *s, unsigned rate,
				unsigned phase)
{
	memset(s, 0, sizeof(*s));
	css_annex_init(&s->css, rate == UDSP_SAMPLE_RATE_WB ?
		       UDSP_SAMPLE_RATE : rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	udsp_resamp_reset(&s->up);
	s->phase = phase;
}

static void onset_zero_far_fill(struct onset_zero_far *s, unsigned rate,
				int16_t *far)
{
	unsigned frame = rate / 100u;
	int16_t now[UDSP_FRAME_MAX];

	if (rate == UDSP_SAMPLE_RATE_WB) {
		int16_t far8[UDSP_FRAME_SAMPLES];

		css_annex_fill(far8, UDSP_FRAME_SAMPLES, &s->css);
		udsp_resamp_up2(&s->up, far8, UDSP_FRAME_SAMPLES, now);
	} else {
		css_annex_fill(now, frame, &s->css);
	}
	memcpy(far, s->line + frame - s->phase, s->phase * sizeof(*far));
	memcpy(far + s->phase, now, (frame - s->phase) * sizeof(*far));
	memcpy(s->line, now, frame * sizeof(*now));
}

/* One near onset on a peak under the frame, read against the conditions of
 * onset_near_get. */
static void onset_zero_read(struct onset_zero_result *r,
			    const struct udsp_ec_nlms_info *before,
			    const struct udsp_ec_nlms_info *info,
			    const struct udsp_ec_nlms_frame *reading,
			    unsigned frame)
{
	if (reading->window_run != 1u || before->coef_peak_tap >= frame ||
	    reading->mic_power < reading->noise_power * MICGATE_SOFT_RATIO)
		return;
	if (info->coef_energy > r->zero_energy)
		r->zero_energy = info->coef_energy;
	r->zero_onsets++;
}

/*
 * The watchdog start of the line, before its idle hiss. A white far end, at
 * 16 kHz raised from 8 kHz, converges the filter on the hybrid. The peak tap
 * then stands past the frame for ONSET_ZERO_CLEAR_FRAMES frames in a row. From
 * there the hybrid grows louder by RAMP_STEP_DB each time the ERLE settles, as
 * in divergence_peak_test. The watchdog then zeroes the filter outside a
 * trial, and the hybrid returns to its response. cleared reads the peak run,
 * zeroed a zeroing with no trial in the start, and watch_frames the frame of
 * the zeroing.
 */
static void onset_zero_watch_run(struct udsp_ec *ec, struct echo_path *path,
				 uint32_t *hiss_state,
				 struct onset_zero_result *r)
{
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct udsp_resamp up;
	float h[LEC_PATH_TAPS_MAX];
	uint32_t far_state = 0x0c0ffee2u;
	double gain_db = 0.0;
	double gain;
	unsigned far_run = 0;
	unsigned dwell = 0;
	unsigned f;
	unsigned i;

	memcpy(h, path->h, sizeof(h));
	udsp_resamp_reset(&up);
	udsp_ec_nlms_info_get(ec, &before);
	for (f = 0; f < ONSET_ZERO_WATCH_MAX_FRAMES; f++) {
		int16_t far8[UDSP_FRAME_SAMPLES];
		int16_t far[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];

		noise_fill(far8, UDSP_FRAME_SAMPLES, ONSET_ZERO_WATCH_FAR_DBM0,
			   &far_state);
		if (path->frame == UDSP_FRAME_SAMPLES)
			memcpy(far, far8, sizeof(far8));
		else
			udsp_resamp_up2(&up, far8, UDSP_FRAME_SAMPLES, far);
		level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
		path_run(path, far, echo);
		noise_fill(hiss, path->frame, ONSET_ZERO_HISS_DBM0, hiss_state);
		for (i = 0; i < path->frame; i++)
			mic[i] = sample_clamp((double)echo[i] + hiss[i]);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (info.diverge_resets != before.diverge_resets) {
			r->zeroed = info.probes == before.probes &&
				    info.coef_energy == 0.0f;
			break;
		}
		far_run = info.coef_peak_tap >= path->frame ? far_run + 1u : 0u;
		r->cleared = r->cleared || far_run >= ONSET_ZERO_CLEAR_FRAMES;
		if (!r->cleared || ++dwell < RAMP_DWELL_FRAMES ||
		    info.erle_db < RAMP_SETTLED_DB)
			continue;
		gain_db += RAMP_STEP_DB;
		gain = pow(10.0, gain_db / 20.0);
		for (i = 0; i < path->taps; i++)
			path->h[i] = (float)(h[i] * gain);
		dwell = 0;
	}
	r->watch_frames = f;
	memcpy(path->h, h, sizeof(h));
}

static void onset_zero_line_run(unsigned rate, uint32_t seed, unsigned phase,
				enum onset_zero_start start,
				struct onset_zero_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct room_noise room;
	struct udsp_resamp up;
	struct onset_zero_far src;
	struct onset_zero_far pre_src;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t hiss_state = 0x51ab0dd5u;
	unsigned pre = start == ONSET_ZERO_RESET ? ONSET_ZERO_PRE_FRAMES : 0u;
	unsigned near0 = pre + ONSET_ZERO_IDLE_FRAMES;
	unsigned far0 = near0 + ONSET_START_LEAD_FRAMES;
	unsigned long from;
	unsigned long to;
	unsigned long n = 0;
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0;
	double loudest = 0.0;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	onset_zero_far_init(&src, rate, phase);
	onset_zero_far_init(&pre_src, rate, 0u);
	room_noise_init(&room, ROOM_HOTH, ONSET_START_SGEN_DBM0, seed);
	udsp_resamp_reset(&up);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, false);
	path_annex_d_init(&path, rate, ANNEX_D_M1, ONSET_START_ERL_DB,
			  PHASE2_DELAY_MS);
	if (start == ONSET_ZERO_WATCHDOG)
		onset_zero_watch_run(ec, &path, &hiss_state, r);
	from = (unsigned long)far0 * path.frame + phase + rate +
	       path_samples_get(PHASE2_DELAY_MS, rate);
	to = from + (unsigned long)rate * 7u / 10u;
	for (f = 0; f < far0 + ONSET_START_FAR_FRAMES; f++) {
		int16_t far[UDSP_FRAME_MAX] = { 0 };
		int16_t near[UDSP_FRAME_MAX] = { 0 };
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];

		if (f < pre)
			onset_zero_far_fill(&pre_src, rate, far);
		else if (f >= far0)
			onset_zero_far_fill(&src, rate, far);
		if (f >= near0)
			onset_start_near_fill(&room, &up, rate, near);
		level_loss_apply(ref, far, path.frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		noise_fill(hiss, path.frame, ONSET_ZERO_HISS_DBM0, &hiss_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + hiss[i] +
					      near[i]);
		if (pre && f == pre)
			udsp_ec_nlms_reset(ec);
		udsp_ec_nlms_info_get(ec, &before);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (f >= far0)
			onset_zero_read(r, &before, &info, &reading,
					path.frame);
		for (i = 0; i < path.frame; i++, n++) {
			meter = a * meter + (1.0 - a) * mic[i] * mic[i];
			if (n >= from && n < to && meter > loudest)
				loudest = meter;
		}
	}
	r->conv_db = PHASE2_FAR_DBM0 + DT_ACT_SINGLE_DB -
		     rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/*
 * Plain onset steps of a zeroed filter under a near noise that starts before
 * the far end. The tracked floor
 * still reads the quiet hiss. The case runs a new filter and a filter that a
 * reset zeroes after it converged. The watchdog zeroes a third filter after
 * its peak stood past the frame. Each runs at every source phase. The steps
 * keep the coefficient energy within ONSET_ZERO_ENERGY, and the combined loss
 * at 1 s reaches X.
 */
static void onset_zero_test(unsigned rate)
{
	static const char *const names[ONSET_ZERO_STARTS] = {
		"new", "reset", "watchdog"
	};
	double x_db = fmin(ONSET_START_X_MAX_DB,
			   PHASE2_FAR_DBM0 + DT_ACT_SINGLE_DB -
			   ONSET_START_SGEN_DBM0 - ONSET_START_X_OFFSET_DB);
	unsigned frame = rate / 100u;
	unsigned j;
	unsigned k;
	unsigned p;

	case_rate(rate, "lec bounds the plain onset steps of a zeroed filter "
		  "under a near noise on a quiet line");
	for (j = 0; j < ONSET_ZERO_STARTS; j++) {
		double energy = 0.0;
		double conv = 0.0;
		unsigned onsets = 0;
		unsigned under = 0;
		unsigned zeroed = 0;
		unsigned watch = 0;

		for (k = 0; k < ONSET_ZERO_SEEDS; k++)
			for (p = 0; p < ONSET_ZERO_PHASES; p++) {
				struct onset_zero_result r;

				onset_zero_line_run(rate, onset_start_seeds[k],
						    p * frame /
						    ONSET_ZERO_PHASES,
						    (enum onset_zero_start)j,
						    &r);
				if ((!k && !p) || r.zero_energy > energy)
					energy = r.zero_energy;
				if ((!k && !p) || r.conv_db < conv)
					conv = r.conv_db;
				onsets += r.zero_onsets;
				under += r.conv_db < x_db;
				zeroed += r.cleared && r.zeroed;
				if (r.watch_frames > watch)
					watch = r.watch_frames;
			}
		printf("# %u Hz, %s filter, %u source phases x %u Hoth seeds: "
		       "%u plain onset steps, coefficient energy up to %.4f "
		       "against %.1f; combined loss at 1 s %.2f dB at least "
		       "against X %.2f dB, %u under\n", rate, names[j],
		       ONSET_ZERO_PHASES, ONSET_ZERO_SEEDS, onsets, energy,
		       ONSET_ZERO_ENERGY, conv, x_db, under);
		if (j == ONSET_ZERO_WATCHDOG) {
			printf("# %u Hz, watchdog filter: %u of %u runs zero "
			       "the filter outside a trial after the peak "
			       "stood past the frame, within %u frames\n", rate,
			       zeroed, ONSET_ZERO_PHASES * ONSET_ZERO_SEEDS,
			       watch);
			CHECK_UINT(zeroed, ONSET_ZERO_PHASES *
					   ONSET_ZERO_SEEDS);
		}
		CHECK(energy <= ONSET_ZERO_ENERGY);
		CHECK(conv >= x_db);
	}
}

/* The line of onset_zero_boot_test, with the processor on. The far end of
 * onset_zero_test runs over the hiss of PHASE2. It runs alone until the peak
 * tap stands past the frame for ONSET_ZERO_CLEAR_FRAMES frames in a row.
 * A white near end at ONSET_ZERO_BOOT_NEAR_DBM0 then starts on the next far
 * onset. */
#define ONSET_ZERO_BOOT_NEAR_DBM0	-35.0
#define ONSET_ZERO_BOOT_MAX_FRAMES	(4u * FRAMES_PER_SECOND)

/* The least move share of the plain step on the first near onset, at 8 and
 * 16 kHz. At 16 kHz the source raised from 8 kHz starts over small samples.
 * The plain step then moves the filter by about its energy, and at 8 kHz by
 * a few percent. The onset floor holds a move under 1 percent. */
static const double onset_zero_boot_move[2] = { 0.03, 0.6 };

/* The first near onset of the line of onset_zero_boot_test. share is its
 * move of the coefficient energy over that energy, and peak its peak tap at
 * the start. active counts the active frames before it. over_db is the
 * microphone over the floor, and fired whether the detector fired. */
struct onset_zero_boot_result {
	double share;
	unsigned peak;
	unsigned active;
	double over_db;
	bool fired;
	bool seen;
};

static void onset_zero_boot_run(unsigned rate,
				struct onset_zero_boot_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info before;
	struct udsp_ec_nlms_info info;
	struct onset_zero_far src;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t hiss_state = 0x51ab0dd5u;
	uint32_t near_state = 0x0badcafeu;
	unsigned far0 = ONSET_ZERO_IDLE_FRAMES;
	unsigned far_run = 0;
	unsigned active = 0;
	bool cleared = false;
	bool talk = false;
	bool far_before = false;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	onset_zero_far_init(&src, rate, 0u);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1, ONSET_START_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; f < far0 + ONSET_ZERO_BOOT_MAX_FRAMES && !r->seen; f++) {
		int16_t far[UDSP_FRAME_MAX] = { 0 };
		int16_t near[UDSP_FRAME_MAX] = { 0 };
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t mic[UDSP_FRAME_MAX];
		bool far_on = false;

		if (f >= far0)
			onset_zero_far_fill(&src, rate, far);
		for (i = 0; i < path.frame; i++)
			far_on = far_on || far[i] != 0;
		if (cleared && far_on && !far_before)
			talk = true;
		far_before = far_on;
		if (talk)
			noise_fill(near, path.frame, ONSET_ZERO_BOOT_NEAR_DBM0,
				   &near_state);
		level_loss_apply(ref, far, path.frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		noise_fill(hiss, path.frame, PHASE2_HISS_DBM0, &hiss_state);
		for (i = 0; i < path.frame; i++)
			mic[i] = sample_clamp((double)echo[i] + hiss[i] +
					      near[i]);
		udsp_ec_nlms_info_get(ec, &before);
		udsp_ec_nlms.run(ec, mic, ref);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (talk && reading.window_run == 1u &&
		    reading.mic_power >= reading.noise_power *
					 MICGATE_SOFT_RATIO) {
			r->share = before.coef_energy > 0.0f ?
				   fabs((double)info.coef_energy -
					before.coef_energy) /
				   before.coef_energy : 0.0;
			r->peak = before.coef_peak_tap;
			r->active = active;
			r->over_db = 10.0 * log10(reading.mic_power /
						  reading.noise_power);
			r->fired = reading.dt_fired;
			r->seen = true;
		}
		far_run = info.coef_peak_tap >= path.frame ? far_run + 1u : 0u;
		cleared = cleared || far_run >= ONSET_ZERO_CLEAR_FRAMES;
		active += reading.ref_power >= reading.ref_bar;
	}
	udsp_ec_nlms.destroy(ec);
}

/*
 * The plain step of the bootstrap with the processor on. The peak of a zeroed
 * filter first stands past the frame for ONSET_ZERO_CLEAR_FRAMES frames in a
 * row. The first near onset of the bootstrap is unfired and on a peak past the
 * frame. It moves the filter by onset_zero_boot_move of its energy at least.
 */
static void onset_zero_boot_test(unsigned rate)
{
	struct onset_zero_boot_result r;

	case_rate(rate, "lec keeps the plain onset step of the bootstrap once "
		  "the peak of a zeroed filter stood past the frame");
	onset_zero_boot_run(rate, &r);
	printf("# %u Hz: first near onset after %u active frames, peak tap %u, "
	       "microphone %.1f dB over the floor, fired %d, move %.2f "
	       "percent against %.0f\n", rate, r.active, r.peak, r.over_db,
	       r.fired, 100.0 * r.share,
	       100.0 * onset_zero_boot_move[rate == UDSP_SAMPLE_RATE_WB]);
	CHECK(r.seen);
	CHECK(r.active < NEAR_START_BOOT_FRAMES);
	CHECK(r.peak >= rate / 100u);
	CHECK(!r.fired);
	CHECK(r.share >= onset_zero_boot_move[rate == UDSP_SAMPLE_RATE_WB]);
}

/* The over-estimate escape (over_get): the ratio in dB, the run of active
 * frames, and the frames within which the trust must hold. */
#define OVER_RATIO_DB		4.0
#define OVER_FRAMES		10u
#define OVER_TRUST_FRAMES	30u

/* The over line: a white far end, a flat 8 ms hybrid, a 48 ms tail, the
 * processor off. After convergence a far pause and a hybrid over_db quieter put
 * the live estimate over_db over the live microphone. */
#define OVER_FAR_DBM0		-20.0
#define OVER_CAL_FRAMES		20u
#define OVER_PAUSE_FRAMES	100u
#define OVER_TALK_DB		16.0
#define OVER_RUN_FRAMES		60u
#define OVER_SETTLE_FRAMES	350u
#define OVER_NONE_DBM0		-200.0

struct over_line {
	struct udsp_ec *ec;
	struct echo_path path;
	struct echo_path base;
	uint32_t state[3];
	unsigned frame;
	double far_dbm0;
	double cal;
	double live[2];
	bool quiet;
};

/* The live estimate and microphone powers of the canceller, rebuilt from a
 * frame reading with its smoothing (LEC_ERLE_ALPHA). */
#define OVER_ALPHA		0.1

static void over_live_add(double live[2],
			  const struct udsp_ec_nlms_frame *reading)
{
	live[0] += (reading->estimate_power - live[0]) * OVER_ALPHA;
	live[1] += (reading->mic_power - live[1]) * OVER_ALPHA;
}

/* Whether the rebuilt live estimate stands at OVER_RATIO_DB or more over the
 * rebuilt live microphone. */
static bool over_live_over(const double live[2])
{
	return live[0] >= live[1] * pow(10.0, OVER_RATIO_DB / 10.0);
}

/* One frame of the over line, with a talker at talk_dbm0 when it is over
 * OVER_NONE_DBM0; active says whether the canceller read the frame as
 * active (its reference at or over its bar). */
static void over_frame_run(struct over_line *l, double talk_dbm0,
			   struct udsp_ec_nlms_frame *reading,
			   struct udsp_ec_nlms_info *info, bool *active)
{
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t talk[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	noise_fill(ref, l->frame, l->far_dbm0, &l->state[0]);
	if (l->quiet)
		memset(ref, 0, l->frame * sizeof(*ref));
	path_run(&l->path, ref, echo);
	noise_fill(hiss, l->frame, CHANGE_FLOOR_DBM0, &l->state[1]);
	if (talk_dbm0 > OVER_NONE_DBM0)
		noise_fill(talk, l->frame, talk_dbm0, &l->state[2]);
	else
		memset(talk, 0, l->frame * sizeof(*talk));
	for (i = 0; i < l->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + hiss[i] + talk[i]);
	udsp_ec_nlms.run(l->ec, mic, ref);
	udsp_ec_nlms_frame_get(l->ec, reading);
	udsp_ec_nlms_info_get(l->ec, info);
	over_live_add(l->live, reading);
	*active = reading->ref_power >= reading->ref_bar;
}

static double over_db_get(double num, double den)
{
	return num > 0.0 && den > 0.0 ? 10.0 * log10(num / den) : -200.0;
}

/* The hybrid of the line becomes the base of over_line_set, and cal the
 * estimate over the microphone of the filter on it. Returns the trust of
 * the last frame. */
static bool over_line_cal(struct over_line *l)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	double est = 0.0;
	double mic = 0.0;
	bool active;
	unsigned f;

	l->base = l->path;
	for (f = 0; f < OVER_CAL_FRAMES; f++) {
		over_frame_run(l, OVER_NONE_DBM0, &reading, &info, &active);
		est += reading.estimate_power;
		mic += reading.mic_power;
	}
	l->cal = mic > 0.0 ? est / mic : 1.0;
	return reading.trusted;
}

/* The converged line on a hybrid at erl_db with a far end at far_dbm0, its
 * calibration, and the trust of its last frame. */
static bool over_line_open(struct over_line *l, unsigned rate, double erl_db,
			   double far_dbm0)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	bool active;
	unsigned f;

	memset(l, 0, sizeof(*l));
	l->frame = rate / 100u;
	l->far_dbm0 = far_dbm0;
	l->state[0] = 0x0f1e2d3cu;
	l->state[1] = 0x4b5a6978u;
	l->state[2] = 0x8796a5b4u;
	l->ec = udsp_ec_nlms.create_rate(48, rate);
	udsp_ec_nlms.nlp(l->ec, false);
	path_flat_init(&l->path, rate, 8, erl_db);
	for (f = 0; f < CHANGE_CONVERGE_FRAMES; f++)
		over_frame_run(l, OVER_NONE_DBM0, &reading, &info, &active);
	return over_line_cal(l);
}

/* The over line of over_arm_test, converged and calibrated. */
static bool over_line_start(struct over_line *l, unsigned rate)
{
	return over_line_open(l, rate, CHANGE_ERL_DB, OVER_FAR_DBM0);
}

/* The hybrid of the line db quieter than it stands. */
static void over_path_step(struct over_line *l, double db)
{
	unsigned k;

	for (k = 0; k < l->path.taps; k++)
		l->path.h[k] = (float)(l->path.h[k] * pow(10.0, -db / 20.0));
}

/* The hybrid over_db quieter than the converged one, against cal. */
static void over_line_set(struct over_line *l, double over_db)
{
	double g = sqrt(l->cal / pow(10.0, over_db / 10.0));
	unsigned k;

	for (k = 0; k < l->path.taps; k++)
		l->path.h[k] = (float)(l->base.h[k] * g);
}

/* A far-end pause in which the hybrid turns over_db quieter. */
static void over_line_pause(struct over_line *l, double over_db)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	bool active;
	unsigned f;

	l->quiet = true;
	for (f = 0; f < OVER_PAUSE_FRAMES; f++)
		over_frame_run(l, OVER_NONE_DBM0, &reading, &info, &active);
	over_line_set(l, over_db);
	l->quiet = false;
}

/*
 * The figures of the over line: trials, the first trial's active frame and arm,
 * and the rebuilt run before it. Then the energy before and at its start, the
 * live ratio and the trust. With talk they add the end and the restored energy;
 * with a step, the hybrid turns quieter after the start.
 */
struct over_result {
	unsigned trials;
	unsigned start;
	unsigned arm;
	unsigned run;
	unsigned end;
	double energy;
	double scaled;
	double live_db;
	double restored;
	bool trusted;
};

static void over_line_run(struct over_line *l, unsigned frames, bool talk,
			  double step, struct over_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	double energy;
	uint32_t probes;
	unsigned active_n = 0;
	unsigned start_f = 0;
	unsigned f;
	bool active;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms_info_get(l->ec, &info);
	probes = info.probes;
	energy = info.coef_energy;
	for (f = 1; f <= frames; f++) {
		bool talker = talk && start_f && f == start_f + 1u;

		if (step > 0.0 && start_f && f == start_f + 1u)
			over_path_step(l, step);

		over_frame_run(l, talker ? l->far_dbm0 + OVER_TALK_DB :
				   OVER_NONE_DBM0, &reading, &info, &active);
		active_n += active;
		if (f == 1u)
			r->trusted = reading.trusted;
		if (!start_f && active)
			r->run = over_live_over(l->live) ? r->run + 1u : 0u;
		if (!start_f && info.probes != probes) {
			start_f = f;
			r->start = active_n;
			r->arm = reading.trial_arm;
			r->energy = energy;
			r->scaled = info.coef_energy;
			r->live_db = over_db_get(l->live[1], l->live[0]);
		}
		if (talker) {
			r->end = reading.trial_end;
			r->restored = info.coef_energy;
		}
		energy = info.coef_energy;
	}
	r->trials = info.probes - probes;
}

/*
 * The legs of over_arm_test, filter frozen: 3.9 dB starts no trial, and 4.1 dB
 * starts one from scaled taps that a talker's refusal restores. The stand-down
 * then holds 6 dB, and the run counts nothing in a trial; a lost trust or a
 * reset filter starts none.
 */
static void over_print(unsigned rate, const char *what,
		       const struct over_result *r)
{
	printf("# %u Hz, %s: %u trials, first %s on active frame %u after a "
	       "run of %u; coefficient energy %+.2f dB on its start frame "
	       "against %+.2f dB of live microphone over live estimate, "
	       "%+.2f dB after the end %s; trusted on the first frame %d\n",
	       rate, what, r->trials,
	       r->start ? lecr_trial_arm_name(r->arm) : "none", r->start,
	       r->run, over_db_get(r->scaled, r->energy), r->live_db,
	       over_db_get(r->restored, r->energy),
	       r->end ? lecr_trial_end_name(r->end) : "-", r->trusted);
}

/*
 * The conditions of the over-estimate escape, filter frozen: over the ratio a
 * trial of the path-change arm starts on the OVER_FRAMES-th active frame from
 * scaled taps. A refusal restores them and stands the arm down. Under the
 * ratio, in a trial or stand-down, or without a recent trust, none starts.
 */
static void over_arm_test(unsigned rate)
{
	struct over_line l;
	struct over_result r;
	double ratio = OVER_RATIO_DB;

	case_rate(rate, "lec starts an escape trial from scaled taps on an "
		  "estimate over the microphone");

	CHECK(over_line_start(&l, rate));
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, ratio - 0.1);
	over_line_run(&l, OVER_RUN_FRAMES, false, 0.0, &r);
	over_print(rate, "3.9 dB", &r);
	CHECK(r.trusted);
	CHECK_UINT(r.trials, 0);
	udsp_ec_nlms.destroy(l.ec);

	CHECK(over_line_start(&l, rate));
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, ratio + 0.1);
	over_line_run(&l, OVER_RUN_FRAMES, true, 0.0, &r);
	over_print(rate, "4.1 dB", &r);
	CHECK(r.trusted);
	CHECK_UINT(r.trials, 1);
	CHECK_UINT(r.run, OVER_FRAMES);
	CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
	CHECK(fabs(over_db_get(r.scaled, r.energy) - r.live_db) <= 0.05);
	CHECK(r.live_db <= -ratio);
	CHECK_UINT(r.end, UDSP_EC_NLMS_TRIAL_END_TALKER);
	CHECK(fabs(over_db_get(r.restored, r.energy)) <= 0.001);
	udsp_ec_nlms.freeze(l.ec, false);
	over_line_run(&l, OVER_SETTLE_FRAMES, false, 0.0, &r);
	CHECK_UINT(r.trials, 0);
	CHECK(over_line_cal(&l));
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, 6.0);
	over_line_run(&l, OVER_RUN_FRAMES, false, 0.0, &r);
	over_print(rate, "stand-down, then 6 dB", &r);
	CHECK(r.trusted);
	CHECK_UINT(r.trials, 0);
	udsp_ec_nlms.destroy(l.ec);

	CHECK(over_line_start(&l, rate));
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, 6.0);
	over_line_run(&l, OVER_RUN_FRAMES, false, 10.0, &r);
	over_print(rate, "6 dB, no re-arm", &r);
	CHECK_UINT(r.trials, 1);
	CHECK_UINT(r.start, OVER_FRAMES);
	CHECK_UINT(r.run, OVER_FRAMES);
	udsp_ec_nlms.destroy(l.ec);

	CHECK(over_line_start(&l, rate));
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, 2.0);
	over_line_run(&l, 50u, false, 0.0, &r);
	CHECK_UINT(r.trials, 0);
	over_line_set(&l, 6.0);
	over_line_run(&l, OVER_RUN_FRAMES, false, 0.0, &r);
	over_print(rate, "2 then 6 dB", &r);
	CHECK(!r.trusted);
	CHECK_UINT(r.trials, 0);
	udsp_ec_nlms.destroy(l.ec);

	CHECK(over_line_start(&l, rate));
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, 8.0);
	over_line_run(&l, 5u, false, 0.0, &r);
	CHECK_UINT(r.trials, 0);
	udsp_ec_nlms_reset(l.ec);
	over_line_run(&l, OVER_RUN_FRAMES, false, 0.0, &r);
	over_print(rate, "8 dB, reset", &r);
	CHECK_UINT(r.trials, 0);
	udsp_ec_nlms.destroy(l.ec);
}

/* G.168 test 12 on the line of phase2_path_run: m1, model b, then m1 10 dB
 * quieter than b and of another shape, 16 source periods each. */
#define RETURN_PHASE_FRAMES	(16u * 70u)
#define RETURN_FRAMES		(3u * FRAMES_PER_SECOND)

/* The first trial after the return, its run, scale and settling with the
 * processor off; with it on, the returned echo from 1 s. */
struct return_result {
	unsigned start;
	unsigned arm;
	unsigned end_kind;
	unsigned run;
	unsigned settled;
	unsigned trials;
	unsigned resets;
	double scale_db;
	double live_db;
	double erle_db;
	double l_ret_dbm0;
	unsigned loud;
};

/* One frame of the line of phase2_frame_run on the source of
 * css_annex_fill, with the output samples in out. */
static void return_frame_run(struct udsp_ec *ec, struct echo_path *path,
			     struct css_annex *css, uint32_t *state,
			     struct phase2_sums *sum, int16_t *out)
{
	int16_t far[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	unsigned i;

	css_annex_fill(far, path->frame, css);
	level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
	path_run(path, far, echo);
	noise_fill(hiss, path->frame, PHASE2_HISS_DBM0, state);
	for (i = 0; i < path->frame; i++)
		out[i] = sample_clamp((double)echo[i] + hiss[i]);
	udsp_ec_nlms.run(ec, out, ref);
	for (i = 0; i < path->frame; i++) {
		sum->echo += (double)echo[i] * echo[i];
		sum->out += (double)out[i] * out[i];
	}
}

/*
 * A line of over_return_test: model and erl_b, the path of model b and its
 * echo return loss; tail_ms, the tail of the canceller (0 for the default
 * of the rate); delay_ms, the delay of the three paths; and the bounds of
 * the line, settled_max for settled and erle_min for erle_db.
 */
struct return_line {
	enum annex_d_model model;
	double erl_b;
	unsigned tail_ms;
	unsigned delay_ms;
	unsigned settled_max;
	double erle_min;
	const char *what;
};

static void return_path_run(unsigned rate, const struct return_line *line,
			    bool nlp, struct return_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct phase2_sums sum = { 0 };
	struct phase2_sums block = { 0 };
	struct phase2_sums window = { 0 };
	struct echo_path path;
	struct echo_path next;
	struct css_annex css;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(line->tail_ms, rate);
	uint32_t state = 0x51ab0dd5u;
	unsigned long from = rate + path_samples_get(line->delay_ms, rate);
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double live[2] = { 0.0, 0.0 };
	double meter = 0.0;
	double loudest = 0.0;
	int16_t out[UDSP_FRAME_MAX];
	double energy;
	uint32_t probes;
	unsigned f;
	unsigned i;

	memset(r, 0, sizeof(*r));
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	udsp_ec_nlms.nlp(ec, nlp);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	path_annex_d_init(&path, rate, ANNEX_D_M1,
			  line->erl_b + PHASE2_DELTA_ERL_DB, line->delay_ms);
	for (f = 0; f < 2u * RETURN_PHASE_FRAMES; f++) {
		if (f == RETURN_PHASE_FRAMES) {
			path_annex_d_init(&next, rate, line->model,
					  line->erl_b, line->delay_ms);
			path_history_carry(&next, &path);
			path = next;
		}
		return_frame_run(ec, &path, &css, &state, &sum, out);
		udsp_ec_nlms_frame_get(ec, &reading);
		over_live_add(live, &reading);
	}
	udsp_ec_nlms_info_get(ec, &info);
	probes = info.probes;
	r->resets = info.diverge_resets;
	energy = info.coef_energy;
	path_annex_d_init(&next, rate, ANNEX_D_M1,
			  line->erl_b + PHASE2_DELTA_ERL_DB, line->delay_ms);
	path_history_carry(&next, &path);
	path = next;
	for (f = 1; f <= RETURN_FRAMES; f++) {
		return_frame_run(ec, &path, &css, &state, &block, out);
		udsp_ec_nlms_frame_get(ec, &reading);
		udsp_ec_nlms_info_get(ec, &info);
		over_live_add(live, &reading);
		for (i = 0; i < path.frame; i++) {
			meter = a * meter + (1.0 - a) * out[i] * out[i];
			if ((unsigned long)(f - 1u) * path.frame + i >= from &&
			    meter > loudest) {
				loudest = meter;
				r->loud = f;
			}
		}
		if (!r->start && reading.ref_power >= reading.ref_bar)
			r->run = over_live_over(live) ? r->run + 1u : 0u;
		if (!r->start && info.probes != probes) {
			r->start = f;
			r->arm = reading.trial_arm;
			r->scale_db = over_db_get(info.coef_energy, energy);
			r->live_db = over_db_get(live[1], live[0]);
		}
		if (r->start && !r->end_kind)
			r->end_kind = reading.trial_end;
		energy = info.coef_energy;
		if (f % PHASE2_BLOCK_FRAMES)
			continue;
		if (f > PHASE2_ANCHOR_FRAMES &&
		    f <= PHASE2_ANCHOR_FRAMES + PHASE2_WINDOW_FRAMES) {
			window.echo += block.echo;
			window.out += block.out;
		}
		if (!r->settled &&
		    phase2_db_get(block.echo, block.out) >= PHASE2_SETTLED_DB)
			r->settled = f;
		memset(&block, 0, sizeof(block));
	}
	udsp_ec_nlms_info_get(ec, &info);
	r->trials = info.probes - probes;
	r->resets = info.diverge_resets - r->resets;
	r->erle_db = phase2_db_get(window.echo, window.out);
	r->l_ret_dbm0 = rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/* The bounds of over_return_test after the return: the first settled block
 * within RETURN_SETTLED_FRAMES and RETURN_ERLE_DB from 1 s, both within 1 s on
 * the short line. RETURN_LRET_DBM0 bounds the processor on. */
#define RETURN_SETTLED_FRAMES	(2u * FRAMES_PER_SECOND)
#define RETURN_ERLE_DB		12.0
#define RETURN_LRET_DBM0	-45.0
#define RETURN_SHORT_TAIL_MS	32u
#define RETURN_SHORT_DELAY_MS	8u

/*
 * The third phase of G.168 test 12 on three lines and the short line: the
 * trusted filter stands 10 dB over the line. The escape starts a kept
 * path-change trial from scaled taps. The scale stands within 1 dB under the
 * live ratio, as the plain update of the start frame comes first.
 */
static void over_return_test(unsigned rate)
{
	static const struct return_line legs[4] = {
		{ ANNEX_D_M4, 6.0, 0u, PHASE2_DELAY_MS, RETURN_SETTLED_FRAMES,
		  RETURN_ERLE_DB, "m4 at 6 dB" },
		{ ANNEX_D_M7, 12.0, 0u, PHASE2_DELAY_MS, RETURN_SETTLED_FRAMES,
		  RETURN_ERLE_DB, "m7 at 12 dB" },
		{ ANNEX_D_M8, 12.0, 0u, PHASE2_DELAY_MS, RETURN_SETTLED_FRAMES,
		  RETURN_ERLE_DB, "m8 at 12 dB" },
		{ ANNEX_D_M4, 6.0, RETURN_SHORT_TAIL_MS, RETURN_SHORT_DELAY_MS,
		  FRAMES_PER_SECOND, PHASE2_SETTLED_DB,
		  "m4 at 6 dB, 32 ms tail, 8 ms delay" },
	};
	unsigned i;

	case_rate(rate, "lec escapes the return of G.168 test 12 to a quieter "
		  "path");
	for (i = 0; i < 4u; i++) {
		struct return_result r;
		struct return_result on;

		return_path_run(rate, &legs[i], false, &r);
		return_path_run(rate, &legs[i], true, &on);
		printf("# %u Hz, %s back to m1: trial of arm %s on frame %u "
		       "after a run of %u active frames, ended %s; taps scaled "
		       "by %+.2f dB of energy against %+.2f dB of live "
		       "microphone over live estimate; 20 dB on frame %u, "
		       "%.2f dB of ERLE at 1 s; %u trials, %u resets; with the "
		       "processor, trial on frame %u, l_ret %.1f dBm0 on frame "
		       "%u\n", rate,
		       legs[i].what, lecr_trial_arm_name(r.arm), r.start, r.run,
		       lecr_trial_end_name(r.end_kind), r.scale_db, r.live_db,
		       r.settled, r.erle_db, r.trials, r.resets, on.start,
		       on.l_ret_dbm0, on.loud);
		CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
		CHECK_UINT(r.run, OVER_FRAMES);
		CHECK(r.live_db <= -OVER_RATIO_DB);
		CHECK(r.scale_db <= r.live_db + 0.05 &&
		      r.scale_db >= r.live_db - 1.0);
		CHECK_UINT(r.end_kind, UDSP_EC_NLMS_TRIAL_END_KEPT);
		CHECK(r.settled > 0u && r.settled <= legs[i].settled_max);
		CHECK(r.erle_db >= legs[i].erle_min);
		CHECK_UINT(r.resets, 0);
		CHECK_UINT(on.start, r.start);
		CHECK(on.l_ret_dbm0 <= RETURN_LRET_DBM0);
	}
}

/* The drop line of over_talker_test: the far end falls by OVER_DROP_DB after
 * the pause and stays active. From OVER_DROP_TALK_FRAME the guard holds the
 * quieter far end while the live estimate carries the louder frames. */
#define OVER_DROP_FAR_DBM0	-10.0
#define OVER_DROP_ERL_DB	6.0
#define OVER_DROP_OVER_DB	10.0
#define OVER_DROP_DB		30.0
#define OVER_DROP_FRAME		4u
#define OVER_DROP_TALK_FRAME	9u
#define OVER_DROP_TALK_DB	14.0

/*
 * The figures of the drop line: trusted, the trust of the converged line;
 * start, the active frame after the pause that starts the first trial (0
 * for none), and trials, the trials started. With talk: talker, the talker
 * verdict of the frame of the talker, and live_db, the rebuilt live
 * estimate over the rebuilt live microphone of that frame.
 */
struct over_drop_result {
	bool trusted;
	unsigned start;
	unsigned trials;
	bool talker;
	double live_db;
};

static void over_drop_run(unsigned rate, bool talk, struct over_drop_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct over_line l;
	unsigned active_n = 0;
	bool dropped = false;
	uint32_t probes;
	unsigned f;
	bool active;

	memset(r, 0, sizeof(*r));
	r->trusted = over_line_open(&l, rate, OVER_DROP_ERL_DB,
				    OVER_DROP_FAR_DBM0);
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, OVER_DROP_OVER_DB);
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	for (f = 0; f < OVER_RUN_FRAMES; f++) {
		bool talker = talk && active_n + 1u == OVER_DROP_TALK_FRAME;

		if (!dropped && active_n + 1u == OVER_DROP_FRAME) {
			l.far_dbm0 -= OVER_DROP_DB;
			dropped = true;
		}
		over_frame_run(&l, talker ? l.far_dbm0 + OVER_DROP_TALK_DB :
				   OVER_NONE_DBM0, &reading, &info, &active);
		active_n += active;
		if (talker) {
			r->talker = reading.talker;
			r->live_db = over_db_get(l.live[0], l.live[1]);
		}
		if (!r->start && info.probes != probes)
			r->start = active_n;
	}
	r->trials = info.probes - probes;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The talker condition of the over-estimate run: without the talker the trial
 * starts on the OVER_FRAMES-th active frame after the pause. A talker on
 * OVER_DROP_TALK_FRAME restarts the run, so the trial starts OVER_FRAMES active
 * frames after it.
 */
static void over_talker_test(unsigned rate)
{
	struct over_drop_result r;

	case_rate(rate, "lec starts the over-estimate run over on a talker");
	over_drop_run(rate, false, &r);
	printf("# %u Hz, no talker: %u trials, first on active frame %u\n",
	       rate, r.trials, r.start);
	CHECK(r.trusted);
	CHECK_UINT(r.start, OVER_FRAMES);
	over_drop_run(rate, true, &r);
	printf("# %u Hz, talker on active frame %u: talker %d, live estimate "
	       "%+.2f dB over live microphone; %u trials, first on active "
	       "frame %u\n", rate, OVER_DROP_TALK_FRAME, r.talker, r.live_db,
	       r.trials, r.start);
	CHECK(r.trusted);
	CHECK(r.talker);
	CHECK(r.live_db >= OVER_RATIO_DB);
	CHECK_UINT(r.start, OVER_DROP_TALK_FRAME + OVER_FRAMES);
}

/* The trial edge of over_edge_test: the over line, the trial kept after its
 * budget, then a pause in which the hybrid turns OVER_EDGE_STEP_DB quieter. The
 * line then stands over the ratio from the first active frame after it. */
#define OVER_EDGE_DB		8.0
#define OVER_EDGE_STEP_DB	6.0
#define OVER_EDGE_FRAMES	(2u * TRIAL_BUDGET)

/*
 * The figures of over_edge_test: trusted, the trust of the converged line;
 * start, the active frame after the first pause that starts the trial, and
 * end and end_kind, the active frame of its end and the end; next, the
 * figures of over_line_run after the second pause.
 */
struct over_edge_result {
	bool trusted;
	unsigned start;
	unsigned end;
	unsigned end_kind;
	struct over_result next;
};

static void over_edge_run(unsigned rate, struct over_edge_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct over_line l;
	unsigned active_n = 0;
	uint32_t probes;
	unsigned f;
	bool active;

	memset(r, 0, sizeof(*r));
	r->trusted = over_line_start(&l, rate);
	udsp_ec_nlms.freeze(l.ec, true);
	over_line_pause(&l, OVER_EDGE_DB);
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	for (f = 0; f < OVER_EDGE_FRAMES && !r->end_kind; f++) {
		over_frame_run(&l, OVER_NONE_DBM0, &reading, &info, &active);
		active_n += active;
		if (!r->start && info.probes != probes) {
			r->start = active_n;
			udsp_ec_nlms.freeze(l.ec, false);
		} else if (r->start && reading.trial_end) {
			r->end = active_n;
			r->end_kind = reading.trial_end;
		}
	}
	udsp_ec_nlms.freeze(l.ec, true);
	l.quiet = true;
	for (f = 0; f < OVER_PAUSE_FRAMES; f++)
		over_frame_run(&l, OVER_NONE_DBM0, &reading, &info, &active);
	l.quiet = false;
	over_path_step(&l, OVER_EDGE_STEP_DB);
	over_line_run(&l, OVER_RUN_FRAMES, false, 0.0, &r->next);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The over-estimate run across a trial edge. The run starts over at the
 * start of a trial and at its end (runs_clear). After a kept trial, the
 * next trial therefore waits for OVER_FRAMES fresh active frames over the
 * ratio. The trust of the kept trial still holds after the pause, so the
 * gate holds for the next run.
 */
static void over_edge_test(unsigned rate)
{
	struct over_edge_result r;

	case_rate(rate, "lec counts the over-estimate run afresh after a kept "
		  "trial");
	over_edge_run(rate, &r);
	printf("# %u Hz: trial on active frame %u, ended %s on active frame "
	       "%u; after the pause, %u trials, first on active frame %u "
	       "after a run of %u, trusted on the first frame %d\n", rate,
	       r.start, r.end_kind ? lecr_trial_end_name(r.end_kind) : "-",
	       r.end, r.next.trials, r.next.start, r.next.run,
	       r.next.trusted);
	CHECK(r.trusted);
	CHECK_UINT(r.start, OVER_FRAMES);
	CHECK_UINT(r.end_kind, UDSP_EC_NLMS_TRIAL_END_KEPT);
	CHECK_UINT(r.end, OVER_FRAMES + TRIAL_BUDGET);
	CHECK(r.next.trusted);
	CHECK_UINT(r.next.trials, 1);
	CHECK_UINT(r.next.arm, UDSP_EC_NLMS_TRIAL_ARM_PATH);
	CHECK_UINT(r.next.start, OVER_FRAMES);
	CHECK_UINT(r.next.run, OVER_FRAMES);
}

/*
 * The growth line of over_watchdog_test: the hybrid grows until the watchdog
 * zeroes the filter, which relearns and freezes. A quieter step then puts the
 * live estimate over the ratio within LEC_OVER_TRUST_FRAMES of the last trust.
 */
#define OVER_GROW_FAR_DBM0	-40.0
#define OVER_GROW_DB		0.0025
#define OVER_GROW_FRAMES	(100u * FRAMES_PER_SECOND)
#define OVER_RELEARN_FRAMES	10u
#define OVER_STEP_FAR_DBM0	-10.0
#define OVER_STEP_DB		15.0

/*
 * The figures of over_watchdog_test: the zeroing frame, the trials of the
 * growth and the last trust before it. Then the first frame and length of the
 * rebuilt run after it, the trials after it, and the coefficient energy of the
 * reset frame.
 */
struct over_watchdog_result {
	unsigned reset;
	unsigned grow_trials;
	unsigned trusted;
	unsigned first;
	unsigned run;
	unsigned next;
	double coef;
};

/* The growth of the line up to the reset of the watchdog. */
static void over_grow_run(struct over_line *l,
			  struct over_watchdog_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	uint32_t probes;
	uint32_t resets;
	unsigned f;
	bool active;

	udsp_ec_nlms_info_get(l->ec, &info);
	probes = info.probes;
	resets = info.diverge_resets;
	for (f = 1; f <= OVER_GROW_FRAMES && !r->reset; f++) {
		over_path_step(l, -OVER_GROW_DB);
		over_frame_run(l, OVER_NONE_DBM0, &reading, &info, &active);
		if (reading.trusted)
			r->trusted = f;
		if (info.diverge_resets != resets) {
			r->reset = f;
			r->coef = info.coef_energy;
			/* The watchdog seeds the live microphone with the
			 * frame and zeroes the live estimate. */
			l->live[0] = 0.0;
			l->live[1] = reading.mic_power;
		}
	}
	r->grow_trials = info.probes - probes;
}

static void over_watchdog_run(unsigned rate, struct over_watchdog_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path back;
	struct over_line l;
	uint32_t probes;
	unsigned f;
	bool active;

	memset(r, 0, sizeof(*r));
	over_line_open(&l, rate, CHANGE_ERL_DB, OVER_GROW_FAR_DBM0);
	over_grow_run(&l, r);
	back = l.base;
	path_history_carry(&back, &l.path);
	l.path = back;
	udsp_ec_nlms_info_get(l.ec, &info);
	probes = info.probes;
	for (f = 1; r->reset && f <= OVER_RELEARN_FRAMES + OVER_RUN_FRAMES;
	     f++) {
		if (f == OVER_RELEARN_FRAMES + 1u) {
			udsp_ec_nlms.freeze(l.ec, true);
			l.far_dbm0 = OVER_STEP_FAR_DBM0;
			over_path_step(&l, OVER_STEP_DB);
		}
		over_frame_run(&l, OVER_NONE_DBM0, &reading, &info, &active);
		if (!active)
			continue;
		r->run = over_live_over(l.live) ? r->run + 1u : 0u;
		if (r->run == 1u && !r->first)
			r->first = f;
	}
	r->next = info.probes - probes;
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * The watchdog and the trust gate of the over-estimate run. The zeroing starts
 * the state of the run over and reads the trust as lost (over_restart). The
 * trust of the relearned filter waits for its own count. A run of that filter
 * within LEC_OVER_TRUST_FRAMES frames of the zeroed one's last trust therefore
 * starts no trial.
 */
static void over_watchdog_test(unsigned rate)
{
	struct over_watchdog_result r;

	case_rate(rate, "lec reads the trust as lost after the watchdog");
	over_watchdog_run(rate, &r);
	printf("# %u Hz: watchdog on frame %u of the growth, %u trials in it, "
	       "coefficient energy %.4f after it; trust last held on frame "
	       "%u; a run over the ratio from frame %u after the reset, %u "
	       "frames long at the end; %u trials after the reset\n", rate,
	       r.reset, r.grow_trials, r.coef, r.trusted, r.first, r.run,
	       r.next);
	CHECK(r.reset > 0u);
	CHECK_UINT(r.grow_trials, 0);
	CHECK(r.coef == 0.0);
	CHECK(r.first > 0u &&
	      r.reset + r.first - r.trusted <= OVER_TRUST_FRAMES);
	CHECK(r.run >= OVER_FRAMES);
	CHECK_UINT(r.next, 0);
}

/* The floor_keep line: white far bursts with pauses of pause_extra plus one
 * quiet frames, and m1 then m4 behind FLOOR_KEEP_DELAY_MS. A line noise rises
 * rise_db at the change and late_db later, under an optional near signal. */
#define FLOOR_KEEP_PERIOD_FRAMES	50u
#define FLOOR_KEEP_DELAY_MS		8u
#define FLOOR_KEEP_FRAMES		(14u * FRAMES_PER_SECOND)
#define FLOOR_KEEP_LATE_FRAMES		(2u * FRAMES_PER_SECOND)
#define FLOOR_KEEP_TALK_FRAMES		700u
#define FLOOR_KEEP_RISE_DB		8.0
#define FLOOR_KEEP_TALK_DB		6.0
#define FLOOR_KEEP_BACK_DB		1.0

/*
 * The stationary near signals of the floor_keep line, each at a constant
 * level: a held vowel in the color of word 0 or word 1 of word_fill
 * (lec_paths.h), white noise, or the Hoth noise of room_noise_fill.
 */
enum floor_keep_held {
	FLOOR_HELD_NONE,
	FLOOR_HELD_VOWEL0,
	FLOOR_HELD_VOWEL1,
	FLOOR_HELD_WHITE,
	FLOOR_HELD_HOTH,
};

struct floor_keep_line {
	unsigned pause_extra;
	double rise_db;
	double late_db;
	unsigned late_after;
	double talk_db;
	enum floor_keep_held held;
	double held_db;
	unsigned held_at;
	unsigned held_frames;
	double pause_db;
	unsigned frames;
};

struct floor_held_source {
	enum floor_keep_held kind;
	uint32_t state;
	double level_dbm0;
	double m1;
	double m2;
	struct room_noise room;
};

static void floor_held_init(struct floor_held_source *h,
			    enum floor_keep_held kind, double level_dbm0)
{
	memset(h, 0, sizeof(*h));
	h->kind = kind;
	h->state = 0x13579bdfu;
	h->level_dbm0 = level_dbm0;
	if (kind == FLOOR_HELD_HOTH)
		room_noise_init(&h->room, ROOM_HOTH, level_dbm0, 0x2468ace1u);
}

/* One frame of the held signal. A vowel frame is the colored noise of
 * word_fill rescaled to the level, without the syllabic envelope. */
static void floor_held_fill(struct floor_held_source *h, int16_t *dst,
			    unsigned frame)
{
	static const double colour[2][2] = { { 1.2, -0.6 }, { 1.7, -0.9 } };
	const double *tap = colour[h->kind == FLOOR_HELD_VOWEL1];
	double energy = 0.0;
	double scale;
	unsigned k;

	if (h->kind == FLOOR_HELD_HOTH) {
		room_noise_fill(dst, frame, &h->room);
		return;
	}
	noise_fill(dst, frame, h->level_dbm0, &h->state);
	if (h->kind == FLOOR_HELD_WHITE)
		return;
	for (k = 0; k < frame; k++) {
		double y = 0.3 * dst[k] + tap[0] * h->m1 + tap[1] * h->m2;

		h->m2 = h->m1;
		h->m1 = y;
		dst[k] = sample_clamp(y);
		energy += y * y;
	}
	scale = energy > 0.0 ?
		dbm0_rms_get(h->level_dbm0) / sqrt(energy / frame) : 0.0;
	for (k = 0; k < frame; k++)
		dst[k] = sample_clamp((double)dst[k] * scale);
}

/* The keep, the rise, the floor levels, waits, frames without a floor, fires
 * and the fall. */
struct floor_keep_result {
	unsigned keep;
	unsigned arm;
	unsigned rise;
	unsigned late_waits;
	unsigned nofloor;
	unsigned fired;
	double loudest_db;
	double end_db;
	double jump_db;
	unsigned fall;
	bool fell;
};

/* One frame of the floor_keep line: a burst frame or a pause frame of the
 * far end through path, the line noise and the talker frame near. */
static void floor_keep_frame(struct udsp_ec *ec, struct echo_path *path,
			     uint32_t state[2], bool burst, double hiss_dbm0,
			     const int16_t *near)
{
	int16_t far[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	if (burst)
		noise_fill(far, path->frame, PHASE2_FAR_DBM0, &state[0]);
	else
		memset(far, 0, sizeof(far));
	level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
	path_run(path, far, echo);
	noise_fill(hiss, path->frame, hiss_dbm0, &state[1]);
	for (i = 0; i < path->frame; i++)
		mic[i] = sample_clamp((double)echo[i] + hiss[i] +
				      (near ? near[i] : 0));
	udsp_ec_nlms.run(ec, mic, ref);
}

static void floor_keep_run(unsigned rate, const struct floor_keep_line *l,
			   struct floor_keep_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct floor_held_source held;
	struct dense_talker talker;
	struct echo_path path;
	struct echo_path next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	uint32_t pause_state = 0x3c6ef372u;
	int16_t near[UDSP_FRAME_MAX];
	unsigned frame = rate / 100u;
	unsigned pause;
	unsigned reads = 0;
	unsigned f;
	uint32_t waits = 0;
	float last = 127.0f;
	float before;

	memset(r, 0, sizeof(*r));
	floor_held_init(&held, l->held, PHASE2_HISS_DBM0 + l->held_db);
	udsp_ec_nlms_info_get(ec, &info);
	pause = (info.taps + frame - 1u) / frame + l->pause_extra;
	dense_talker_init(&talker, rate, PHASE2_HISS_DBM0 + l->talk_db,
			  NEAR_START_SEED);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1,
			  phase2_erl_db[0] + PHASE2_DELTA_ERL_DB,
			  FLOOR_KEEP_DELAY_MS);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		floor_keep_frame(ec, &path, state,
				 f % FLOOR_KEEP_PERIOD_FRAMES <
				 FLOOR_KEEP_PERIOD_FRAMES - pause,
				 PHASE2_HISS_DBM0, NULL);
	udsp_ec_nlms_info_get(ec, &info);
	before = info.noise_dbov;
	path_annex_d_init(&next, rate, phase2_models[0], phase2_erl_db[0],
			  FLOOR_KEEP_DELAY_MS);
	path_history_carry(&next, &path);
	for (f = 0; f < l->frames; f++) {
		unsigned g = PHASE2_CONVERGE_FRAMES + f;
		bool talking = l->talk_db > 0.0 && r->keep && f >= r->keep &&
			       f < r->keep + FLOOR_KEEP_TALK_FRAMES;
		unsigned from = r->keep + l->held_at;
		bool holding = l->held && r->keep && f >= from &&
			       (!l->held_frames || f < from + l->held_frames);
		bool after = l->held && l->held_frames && r->keep &&
			     f >= from + l->held_frames;
		bool late = l->late_after && r->keep &&
			    f >= r->keep + l->late_after;
		bool burst = g % FLOOR_KEEP_PERIOD_FRAMES <
			     FLOOR_KEEP_PERIOD_FRAMES - pause;
		bool pausing = l->pause_db > 0.0 && !burst;

		if (talking)
			dense_talker_fill(&talker, near, frame);
		if (holding)
			floor_held_fill(&held, near, frame);
		if (pausing)
			noise_fill(near, frame, PHASE2_HISS_DBM0 + l->pause_db,
				   &pause_state);
		floor_keep_frame(ec, &next, state, burst,
				 PHASE2_HISS_DBM0 + l->rise_db +
				 (late ? l->late_db : 0.0),
				 talking || holding || pausing ? near : NULL);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (!r->keep &&
		    reading.trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE)
			r->arm = reading.trial_arm;
		if (!r->keep &&
		    reading.trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT)
			r->keep = f + 1u;
		if (r->keep && f >= r->keep) {
			r->nofloor += info.noise_dbov >= 127.0f;
			r->fired += reading.dt_fired;
			if (before - info.noise_dbov > r->loudest_db)
				r->loudest_db = before - info.noise_dbov;
			if (!r->rise && before - info.noise_dbov >= 3.0f)
				r->rise = f + 1u - r->keep;
			if (last < 127.0f && info.noise_dbov < 127.0f &&
			    last - info.noise_dbov > r->jump_db)
				r->jump_db = last - info.noise_dbov;
		}
		if (after && !r->fell) {
			reads += reading.window_run == 0u;
			if (info.noise_dbov < 127.0f &&
			    fabsf(before - info.noise_dbov) <=
			    FLOOR_KEEP_BACK_DB) {
				r->fell = true;
				r->fall = reads;
			}
		}
		last = info.noise_dbov;
		if (f >= l->frames - FLOOR_KEEP_LATE_FRAMES)
			r->late_waits += info.nlp_wait_frames - waits;
		waits = info.nlp_wait_frames;
	}
	r->end_db = before - info.noise_dbov;
	udsp_ec_nlms.destroy(ec);
}

static void floor_keep_print(unsigned rate, const char *what,
			     const struct floor_keep_result *r)
{
	printf("# %u Hz, %s: kept trial of arm %s on frame %u after the "
	       "change; the floor rises %u frames after the keep, loudest "
	       "%.1f dB and at the end %.1f dB over the floor before the "
	       "change; %u engage waits in the last 2 s\n", rate, what,
	       lecr_trial_arm_name(r->arm), r->keep, r->rise, r->loudest_db,
	       r->end_db, r->late_waits);
}

/* The bound of floor_keep_test on the frames from the keep to the rise:
 * LEC_NOISE_RISE_AGE_KEPT of dsp/lec.c. */
#define FLOOR_KEEP_RISE_MAX		700u

/*
 * A background that rises FLOOR_KEEP_RISE_DB with a louder path, 8 readings a
 * second: the change ends in a kept escape trial. The floor rises within
 * FLOOR_KEEP_RISE_MAX frames of the keep and ends within 2 dB of the rise,
 * never 2 dB over it; no frame of the last 2 s waits.
 */
static void floor_keep_test(unsigned rate)
{
	static const struct floor_keep_line line = {
		.pause_extra = 3u, .rise_db = FLOOR_KEEP_RISE_DB,
		.frames = FLOOR_KEEP_FRAMES,
	};
	struct floor_keep_result r;

	floor_keep_run(rate, &line, &r);
	case_rate(rate, "lec takes the louder background of a kept echo path "
		  "change as its floor");
	floor_keep_print(rate, "background 8 dB up at the change", &r);
	CHECK(r.keep > 0u);
	CHECK(r.arm == UDSP_EC_NLMS_TRIAL_ARM_PATH ||
	      r.arm == UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	CHECK(r.rise > 0u && r.rise <= FLOOR_KEEP_RISE_MAX);
	CHECK(r.end_db >= FLOOR_KEEP_RISE_DB - 2.0);
	CHECK(r.loudest_db <= FLOOR_KEEP_RISE_DB + 2.0);
	CHECK_UINT(r.late_waits, 0);
}

/* The bound of floor_keep_flat_test and floor_keep_talker_test on the
 * loudest floor after the keep, over the floor before the change. */
#define FLOOR_KEEP_FLAT_DB		1.0

/*
 * The line of floor_keep_test with a background that does not rise. The
 * floor after the keep must stand FLOOR_KEEP_FLAT_DB or less over the floor
 * before the change, and must not rise.
 */
static void floor_keep_flat_test(unsigned rate)
{
	static const struct floor_keep_line line = {
		.pause_extra = 3u, .frames = FLOOR_KEEP_FRAMES,
	};
	struct floor_keep_result r;

	floor_keep_run(rate, &line, &r);
	case_rate(rate, "lec keeps its floor after a kept echo path change "
		  "over the same background");
	floor_keep_print(rate, "background unchanged", &r);
	CHECK(r.keep > 0u);
	CHECK_UINT(r.rise, 0);
	CHECK(r.loudest_db <= FLOOR_KEEP_FLAT_DB);
}

/*
 * The line of floor_keep_flat_test with a dense talker FLOOR_KEEP_TALK_DB
 * over the line noise for FLOOR_KEEP_TALK_FRAMES from the keep. No rise run
 * holds a reading at the keep, so the window stays closed. The floor must
 * not rise onto the talker: it must stand FLOOR_KEEP_FLAT_DB or less over
 * the floor before the change.
 */
static void floor_keep_talker_test(unsigned rate)
{
	static const struct floor_keep_line line = {
		.pause_extra = 3u, .talk_db = FLOOR_KEEP_TALK_DB,
		.frames = FLOOR_KEEP_FRAMES,
	};
	struct floor_keep_result r;

	floor_keep_run(rate, &line, &r);
	case_rate(rate, "lec keeps its floor under a quiet near talker after "
		  "a kept echo path change");
	floor_keep_print(rate, "talker 6 dB over the line noise", &r);
	CHECK(r.keep > 0u);
	CHECK_UINT(r.rise, 0);
	CHECK(r.loudest_db <= FLOOR_KEEP_FLAT_DB);
}

/*
 * With pauses of 3 quiet frames, no rise run of 50 completes in the window, but
 * the agreement run completes past LEC_NOISE_RISE_AGE. The floor takes the rise
 * within FLOOR_KEEP_RISE_MAX of the keep, as the window bounds the age of both
 * runs.
 */
static void floor_keep_window_test(unsigned rate)
{
	static const struct floor_keep_line line = {
		.pause_extra = 2u, .rise_db = FLOOR_KEEP_RISE_DB,
		.frames = FLOOR_KEEP_FRAMES,
	};
	struct floor_keep_result r;

	floor_keep_run(rate, &line, &r);
	case_rate(rate, "lec re-acquires its floor on sparse pauses within "
		  "the window after a kept echo path change");
	floor_keep_print(rate, "background 8 dB up, sparser pauses", &r);
	CHECK(r.keep > 0u);
	CHECK(r.rise > 0u && r.rise <= FLOOR_KEEP_RISE_MAX);
	CHECK(r.end_db >= FLOOR_KEEP_RISE_DB - 2.0);
	CHECK(r.loudest_db <= FLOOR_KEEP_RISE_DB + 2.0);
}

/*
 * The line of floor_keep_test with a dense talker FLOOR_KEEP_TALK_DB over
 * the louder line noise for FLOOR_KEEP_TALK_FRAMES from the keep. The rise
 * at the change opens the window, and the talker talks through all of it.
 * The floor must not climb onto the talker: it must never stand more than
 * 2 dB over the rise of the background.
 */
static void floor_keep_open_talker_test(unsigned rate)
{
	static const struct floor_keep_line line = {
		.pause_extra = 3u, .rise_db = FLOOR_KEEP_RISE_DB,
		.talk_db = FLOOR_KEEP_RISE_DB + FLOOR_KEEP_TALK_DB,
		.frames = FLOOR_KEEP_FRAMES,
	};
	struct floor_keep_result r;

	floor_keep_run(rate, &line, &r);
	case_rate(rate, "lec keeps its floor off a quiet near talker in an "
		  "open re-acquisition window");
	floor_keep_print(rate, "background 8 dB up, talker 6 dB over it", &r);
	CHECK(r.keep > 0u);
	CHECK(r.loudest_db <= FLOOR_KEEP_RISE_DB + 2.0);
}

/* The level of the pause content of floor_keep_pause_test over the line
 * noise. */
#define FLOOR_KEEP_PAUSE_DB		8.0

/*
 * The line of floor_keep_flat_test with white noise FLOOR_KEEP_PAUSE_DB over
 * the line noise on the far pauses alone, as an echo beyond the window gives.
 * The pauses open the window, and the filter output of the far bursts
 * falsifies their runs. After the keep, no frame must lose the floor or fire
 * the detector, and the floor must not rise.
 */
static void floor_keep_pause_test(unsigned rate)
{
	static const struct floor_keep_line line = {
		.pause_extra = 3u, .pause_db = FLOOR_KEEP_PAUSE_DB,
		.frames = FLOOR_KEEP_FRAMES,
	};
	struct floor_keep_result r;

	floor_keep_run(rate, &line, &r);
	case_rate(rate, "lec keeps its floor through pause content that the "
		  "far bursts falsify after a kept echo path change");
	floor_keep_print(rate, "pause content 8 dB over the line noise", &r);
	printf("# %u Hz: %u frames without a floor and %u fired frames after "
	       "the keep\n", rate, r.nofloor, r.fired);
	CHECK(r.keep > 0u);
	CHECK_UINT(r.nofloor, 0);
	CHECK_UINT(r.fired, 0);
	CHECK_UINT(r.rise, 0);
}

/* The second rise of floor_keep_expiry_test after the keep, past the
 * window of LEC_NOISE_RISE_AGE_KEPT (700 frames), and the length of its
 * line. */
#define FLOOR_KEEP_LATE_RISE_FRAMES	800u
#define FLOOR_KEEP_EXPIRY_FRAMES	(20u * FRAMES_PER_SECOND)

/* A second rise after the window closes, on pauses whose agreement run
 * completes within LEC_NOISE_RISE_AGE: the floor takes both rises; on the
 * pauses of floor_keep_window_test it takes the first only. */
static void floor_keep_expiry_test(unsigned rate)
{
	static const struct floor_keep_line lines[2] = {
		{
			.pause_extra = 3u, .rise_db = FLOOR_KEEP_RISE_DB,
			.late_db = FLOOR_KEEP_RISE_DB,
			.late_after = FLOOR_KEEP_LATE_RISE_FRAMES,
			.frames = FLOOR_KEEP_EXPIRY_FRAMES,
		},
		{
			.pause_extra = 2u, .rise_db = FLOOR_KEEP_RISE_DB,
			.late_db = FLOOR_KEEP_RISE_DB,
			.late_after = FLOOR_KEEP_LATE_RISE_FRAMES,
			.frames = FLOOR_KEEP_EXPIRY_FRAMES,
		},
	};
	struct floor_keep_result r[2];

	floor_keep_run(rate, &lines[0], &r[0]);
	floor_keep_run(rate, &lines[1], &r[1]);
	case_rate(rate, "lec closes the re-acquisition window of its floor "
		  "after a kept echo path change");
	floor_keep_print(rate, "background 8 dB up at the change and 8 dB "
			 "more 8 s after the keep", &r[0]);
	floor_keep_print(rate, "the same on sparser pauses", &r[1]);
	CHECK(r[0].keep > 0u);
	CHECK(r[0].rise > 0u && r[0].rise <= FLOOR_KEEP_RISE_MAX);
	CHECK(r[0].end_db >= 2.0 * FLOOR_KEEP_RISE_DB - 2.0);
	CHECK(r[0].loudest_db <= 2.0 * FLOOR_KEEP_RISE_DB + 2.0);
	CHECK(r[1].keep > 0u);
	CHECK(r[1].rise > 0u && r[1].rise <= FLOOR_KEEP_RISE_MAX);
	CHECK(r[1].loudest_db <= FLOOR_KEEP_RISE_DB + 2.0);
}

/* The held signal of floor_keep_held_test and the length of its lines. */
#define FLOOR_KEEP_HELD_FRAMES		(6u * FRAMES_PER_SECOND)
#define FLOOR_KEEP_HELD_LINE_FRAMES	(12u * FRAMES_PER_SECOND)
/* The rise bound of an agreement run, LEC_NOISE_AGREE_RISE_DB of dsp/lec.c;
 * the creep over it, LEC_NOISE_RISE_DB; and the readings after the signal
 * within which the floor falls back. */
#define FLOOR_KEEP_HELD_STEP_DB		8.0
#define FLOOR_KEEP_HELD_CREEP_DB	3.0
#define FLOOR_KEEP_HELD_FALL		20u

/*
 * A stationary near signal 4 to 40 dB over the line noise starts 0 to 75 frames
 * after the keep. The floor rises at most FLOOR_KEEP_HELD_STEP_DB in a step and
 * FLOOR_KEEP_HELD_CREEP_DB more in all, and falls back within
 * FLOOR_KEEP_HELD_FALL readings after the signal.
 */
static void floor_keep_held_test(unsigned rate)
{
	static const char *const names[4] = {
		"held vowel 0", "held vowel 1", "white noise", "Hoth noise",
	};
	static const double levels[6] = { 4.0, 6.0, 10.0, 20.0, 30.0, 40.0 };
	static const unsigned ats[4] = { 0u, 25u, 50u, 75u };
	struct floor_keep_line line = {
		.pause_extra = 3u, .held_frames = FLOOR_KEEP_HELD_FRAMES,
		.frames = FLOOR_KEEP_HELD_LINE_FRAMES,
	};
	struct floor_keep_result r;
	unsigned legs = 0, keeps = 0, rises = 0, fell = 0;
	unsigned k, n, a, worst = 0, fall = 0;
	double loudest = 0.0, jump = 0.0;

	for (k = 0; k < 4u; k++)
		for (n = 0; n < 6u; n++)
			for (a = 0; a < 4u; a++) {
				line.held = (enum floor_keep_held)(k + 1u);
				line.held_db = levels[n];
				line.held_at = ats[a];
				floor_keep_run(rate, &line, &r);
				legs++;
				keeps += r.keep > 0u;
				rises += r.rise > 0u;
				fell += r.fell;
				if (r.fall > fall)
					fall = r.fall;
				if (r.jump_db > jump)
					jump = r.jump_db;
				if (legs == 1u || r.loudest_db > loudest) {
					loudest = r.loudest_db;
					worst = (k * 6u + n) * 4u + a;
				}
			}
	case_rate(rate, "lec raises its floor by at most 8 dB per agreement "
		  "run under a stationary near signal after a kept echo path "
		  "change");
	printf("# %u Hz: %u lines, %u kept trials, %u floor rises; largest "
	       "step %.2f dB; loudest floor %.1f dB over the floor before the "
	       "change (%s %.0f dB over the line noise, %u frames after the "
	       "keep); back within 1 dB on %u lines, within %u readings after "
	       "the signal\n", rate, legs, keeps, rises, jump, loudest,
	       names[worst / 24u], levels[worst / 4u % 6u], ats[worst % 4u],
	       fell, fall);
	CHECK_UINT(keeps, legs);
	CHECK(jump <= FLOOR_KEEP_HELD_STEP_DB + 0.001);
	CHECK(loudest <= FLOOR_KEEP_HELD_STEP_DB + FLOOR_KEEP_HELD_CREEP_DB);
	CHECK_UINT(fell, legs);
	CHECK(fall <= FLOOR_KEEP_HELD_FALL);
}

/* The line of tone_tail_test, G.168 test 10A: a fresh instance, the 2100 Hz
 * tone through mu-law and m1, then silence. A TONE_TAIL_METER_MS rectangular
 * meter reads Region II; tail_acts counts the acting frames after the tone. */
#define TONE_TAIL_HZ		2100.0
#define TONE_TAIL_DBM0		-13.0
#define TONE_TAIL_ERL_DB	6.0
#define TONE_TAIL_LEAD_FRAMES	20u
#define TONE_TAIL_FRAMES	350u
#define TONE_TAIL_GAP_FRAMES	8u
#define TONE_TAIL_METER_MS	35u
#define TONE_TAIL_REGION2_DBM0	-37.0

struct tone_tail_result {
	double region_dbm0;
	unsigned tail_acts;
	unsigned window;
};

static void tone_tail_line_run(unsigned rate, struct tone_tail_result *r)
{
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	unsigned frame = rate / 100u;
	unsigned meter_n = TONE_TAIL_METER_MS * rate / 1000u;
	double *meter = calloc(meter_n, sizeof(*meter));
	uint32_t state = 0x51ab0dd5u;
	unsigned long phase = 0;
	unsigned end = TONE_TAIL_LEAD_FRAMES + TONE_TAIL_FRAMES;
	unsigned window = window_frames_get(ec);
	unsigned long from = (unsigned long)TONE_TAIL_LEAD_FRAMES * frame +
			     rate * 15u / 100u +
			     path_samples_get(PHASE2_DELAY_MS, rate);
	unsigned long to = (unsigned long)(end + TONE_TAIL_GAP_FRAMES) * frame;
	unsigned long n = 0;
	double sum = 0.0;
	double loudest = 0.0;
	unsigned f;

	memset(r, 0, sizeof(*r));
	r->window = window;
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	path_annex_d_init(&path, rate, ANNEX_D_M1, TONE_TAIL_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; f < end + (window > TONE_TAIL_GAP_FRAMES ? window :
					      TONE_TAIL_GAP_FRAMES); f++) {
		int16_t far[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t out[UDSP_FRAME_MAX];
		unsigned i;

		memset(far, 0, sizeof(far));
		if (f >= TONE_TAIL_LEAD_FRAMES && f < end) {
			tone_fill(far, frame, TONE_TAIL_DBM0, TONE_TAIL_HZ,
				  &phase, rate);
			for (i = 0; i < frame; i++)
				far[i] = tone_ulaw_get(far[i]);
		}
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(&path, far, echo);
		noise_fill(hiss, frame, PHASE2_HISS_DBM0, &state);
		for (i = 0; i < frame; i++)
			out[i] = sample_clamp((double)echo[i] + hiss[i]);
		udsp_ec_nlms.run(ec, out, ref);
		udsp_ec_nlms_info_get(ec, &info);
		if (f >= end && f < end + window && info.nlp_active)
			r->tail_acts++;
		for (i = 0; i < frame; i++, n++) {
			double x = (double)out[i] * out[i];

			sum += x - meter[n % meter_n];
			meter[n % meter_n] = x;
			if (n >= from && n < to && sum / meter_n > loudest)
				loudest = sum / meter_n;
		}
	}
	r->region_dbm0 = rms_dbm0_get(sqrt(loudest));
	free(meter);
	udsp_ec_nlms.destroy(ec);
}

/*
 * The tone tail hold (LEC_TONE_TAIL_BACK): Region II reads
 * TONE_TAIL_REGION2_DBM0 or less, the limit of G.168 test 10A, as the processor
 * acts through the tone's echo tail. The hold starts after a tone alone, so the
 * releases for a near talker stand.
 */
static void tone_tail_test(unsigned rate)
{
	struct tone_tail_result r;

	tone_tail_line_run(rate, &r);
	case_rate(rate, "lec processor holds the echo tail of a far tone");
	printf("# answer tone: Region II peak %.1f dBm0; the processor acts on "
	       "%u of the %u window frames after the tone\n", r.region_dbm0,
	       r.tail_acts, r.window);
	CHECK(r.region_dbm0 <= TONE_TAIL_REGION2_DBM0);
}

/* The line of the escape trial cases: G.168 test 2A with the processor on, m1
 * converged on the composite source, then the model at its ERL. */
#define ESCAPE_TRIAL_FRAMES		(4u * FRAMES_PER_SECOND)
#define ESCAPE_TRIAL_SKIP_FRAMES	5u
#define ESCAPE_TRIAL_TALK_FRAMES	150u
#define ESCAPE_TRIAL_L_ACT_DB		1.49

/* loss_db takes the combined loss at 1 s, act the first acting frame after
 * ESCAPE_TRIAL_SKIP_FRAMES, r the talker's energies and frames when it
 * talks. */
static void escape_trial_leg(unsigned rate, unsigned m, double x_db,
			     unsigned k, uint32_t seed, bool nlp, bool talk,
			     struct near_start_result *r, double *loss_db,
			     unsigned *act)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_frame reading;
	struct dense_talker talker;
	struct css_annex css;
	struct echo_path path, next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0, loudest = 0.0;
	double erl = phase2_erl_db[m];
	unsigned long from = (unsigned long)rate +
			     path_samples_get(PHASE2_DELAY_MS, rate);
	unsigned long to = from + (unsigned long)rate * 7u / 10u;
	int16_t near[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
	unsigned f, i;

	memset(r, 0, sizeof(*r));
	*act = 0;
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	dense_talker_init(&talker, rate, PHASE2_FAR_DBM0 - erl - x_db, seed);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, nlp);
	path_annex_d_init(&path, rate, ANNEX_D_M1, erl + PHASE2_DELTA_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		phase1_frame_run(ec, &path, &css, state, NULL, out);
	path_annex_d_init(&next, rate, phase2_models[m], erl, PHASE2_DELAY_MS);
	path_history_carry(&next, &path);
	for (f = 0; f < ESCAPE_TRIAL_FRAMES; f++) {
		bool talking = false;

		memset(near, 0, sizeof(near));
		if (talk && f >= k && f < k + ESCAPE_TRIAL_TALK_FRAMES) {
			dense_talker_fill(&talker, near, next.frame);
			for (i = 0; i < next.frame; i++)
				talking |= near[i] != 0;
		}
		phase1_frame_run(ec, &next, &css, state, near, out);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (!*act && info.nlp_active && f > ESCAPE_TRIAL_SKIP_FRAMES)
			*act = f;
		r->frames += talking;
		r->fired += talking && reading.dt_fired;
		r->acted += talking && info.nlp_active;
		for (i = 0; talking && i < next.frame; i++) {
			r->near += (double)near[i] * near[i];
			r->product += (double)out[i] * near[i];
		}
		for (i = 0; i < next.frame; i++) {
			unsigned long n = (unsigned long)f * next.frame + i;

			meter = a * meter + (1.0 - a) * out[i] * out[i];
			if (n >= from && n < to && meter > loudest)
				loudest = meter;
		}
	}
	*loss_db = PHASE2_FAR_DBM0 + ESCAPE_TRIAL_L_ACT_DB -
		   rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/*
 * ESCAPE_TRIAL_LOSS_DB: the least combined loss at 1 s of a leg of
 * escape_trial_line_test. ESCAPE_TRIAL_ACT_FRAMES: the latest first act of
 * a leg.
 */
#define ESCAPE_TRIAL_LOSS_DB		42.0
#define ESCAPE_TRIAL_ACT_FRAMES		75u

/*
 * The legs of escape_trial_line_test: m4 and m8 at both rates, and m6 at
 * 8 kHz. On this line m6 at 16 kHz starts no escape trial, so the rule has no
 * frame to act on there.
 */
static const unsigned escape_trial_models[3] = { 0u, 1u, 3u };

/*
 * The escape trial (escape_trial_get) with no talker: each leg reads
 * ESCAPE_TRIAL_LOSS_DB or more of combined loss at 1 s. The processor first
 * acts by ESCAPE_TRIAL_ACT_FRAMES, where without the rule it waits on the long
 * hangover or the engage run.
 */
static void escape_trial_line_test(unsigned rate)
{
	unsigned mi;
	bool ok = true;

	case_rate(rate, "lec acts on the evidence of an escape trial after a "
			"louder echo path");
	for (mi = 0; mi < 3u; mi++) {
		unsigned m = escape_trial_models[mi];
		struct near_start_result r;
		double loss;
		unsigned act;

		if (rate == UDSP_SAMPLE_RATE_WB && m == 1u)
			continue;
		escape_trial_leg(rate, m, 13.0, 0u, 0x0badcafeu, true, false,
				 &r, &loss, &act);
		printf("# %s: combined loss %.1f dB at 1 s, first act on frame "
		       "%u after the change\n",
		       annex_d_tables[phase2_models[m]].name, loss, act);
		ok &= loss >= ESCAPE_TRIAL_LOSS_DB;
		ok &= act > 0u && act <= ESCAPE_TRIAL_ACT_FRAMES;
	}
	CHECK(ok);
}

/*
 * ESCAPE_TRIAL_TAKE_DB: the most near speech that the processor can take
 * from a talker of escape_trial_talker_test.
 */
#define ESCAPE_TRIAL_TAKE_DB		1.0

/*
 * A near talker during the escape trial of the line of escape_trial_leg. The
 * dense talker stands 10 or 16 dB under the changed echo, on two seeds, at m4
 * and m8. It starts at the change or 30 frames after it. The processor must
 * take ESCAPE_TRIAL_TAKE_DB of the talker or less: the near gain of the leg
 * with the processor off, less its gain with the processor on.
 */
static void escape_trial_talker_test(unsigned rate)
{
	static const unsigned models[2] = { 0u, 3u };
	static const double xs[2] = { 10.0, 16.0 };
	static const unsigned ks[2] = { 0u, 30u };
	static const uint32_t seeds[2] = { 0x0badcafeu, 0xe6543829u };
	double worst = -200.0;
	unsigned mi, xi, ki, si, acted = 0;

	case_rate(rate, "lec keeps a near talker through an escape trial");
	for (mi = 0; mi < 2u; mi++)
		for (xi = 0; xi < 2u; xi++)
			for (ki = 0; ki < 2u; ki++)
				for (si = 0; si < 2u; si++) {
					struct near_start_result on, off;
					double loss, take;
					unsigned act;

					escape_trial_leg(rate, models[mi],
							 xs[xi], ks[ki],
							 seeds[si], true, true,
							 &on, &loss, &act);
					escape_trial_leg(rate, models[mi],
							 xs[xi], ks[ki],
							 seeds[si], false, true,
							 &off, &loss, &act);
					take = near_gain_db_get(off.product,
								off.near) -
					       near_gain_db_get(on.product,
								on.near);
					if (take > worst)
						worst = take;
					acted += on.acted;
				}
	printf("# 16 legs: the processor takes up to %.2f dB of the talker, "
	       "and acts on %u talker frames in all\n", worst, acted);
	CHECK(worst <= ESCAPE_TRIAL_TAKE_DB);
}

/* The lines of escape_trial_start_test: m8, and a bare line whose quiet filter
 * stands over ESCAPE_START_COEF_RATIO under the gain's floor. There an inverted
 * path reads -1 and starts a bare coherence trial. */
#define ESCAPE_START_ERL_DB		45.0
#define ESCAPE_START_COEF_RATIO		10.0f
#define ESCAPE_START_FRAMES		20u

/*
 * The first trial after the change: its arm, whether its taps are bare and
 * their energy over the learned gain, and its start frame's reading and
 * hangover. Then the short and long hangovers after fired frames over the next
 * ESCAPE_START_FRAMES - 1 frames.
 */
struct escape_start_result {
	unsigned arm;
	bool bare;
	double taps_db;
	bool fired;
	unsigned hold_left;
	unsigned next_hold;
	unsigned short_holds;
	unsigned long_holds;
};

static void escape_start_run(unsigned rate, bool bare,
			     struct escape_start_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct css_annex css;
	struct echo_path path, next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	int16_t out[UDSP_FRAME_MAX];
	unsigned after = 0, f, k;
	bool fired = false;

	memset(r, 0, sizeof(*r));
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	if (bare) {
		path_annex_d_init(&path, rate, ANNEX_D_M1, ESCAPE_START_ERL_DB,
				  PHASE2_DELAY_MS);
		path_annex_d_init(&next, rate, ANNEX_D_M1, phase2_erl_db[0],
				  PHASE2_DELAY_MS);
		for (k = 0; k < next.taps; k++)
			next.h[k] = -next.h[k];
	} else {
		path_annex_d_init(&path, rate, ANNEX_D_M1,
				  phase2_erl_db[3] + PHASE2_DELTA_ERL_DB,
				  PHASE2_DELAY_MS);
		path_annex_d_init(&next, rate, phase2_models[3],
				  phase2_erl_db[3], PHASE2_DELAY_MS);
	}
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		phase1_frame_run(ec, &path, &css, state, NULL, out);
	path_history_carry(&next, &path);
	for (f = 0; f < ESCAPE_TRIAL_FRAMES && after < ESCAPE_START_FRAMES;
	     f++) {
		phase1_frame_run(ec, &next, &css, state, NULL, out);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (r->arm != UDSP_EC_NLMS_TRIAL_ARM_NONE) {
			if (!after)
				r->next_hold = reading.hold_left;
			else if (fired)
				r->short_holds += reading.hold_left == 4u;
			if (after && fired)
				r->long_holds += reading.hold_left == 24u;
			after++;
		} else if (reading.trial_arm != UDSP_EC_NLMS_TRIAL_ARM_NONE) {
			udsp_ec_nlms_info_get(ec, &info);
			r->arm = reading.trial_arm;
			r->bare = info.coef_energy * ESCAPE_START_COEF_RATIO <
				  info.echo_gain;
			r->taps_db = 10.0 * log10((double)info.coef_energy /
						  (double)info.echo_gain);
			r->fired = reading.dt_fired;
			r->hold_left = reading.hold_left;
		}
		fired = reading.dt_fired;
	}
	udsp_ec_nlms.destroy(ec);
}

/*
 * The start frame of an escape trial, which the detector reads before the trial
 * reads its evidence. A fired start frame inside a running hangover takes the
 * long hangover. After it, a fired frame of the eligible trial takes the short
 * one, and one of the bare trial the long one.
 */
static void escape_trial_start_test(unsigned rate)
{
	unsigned leg;

	case_rate(rate, "lec escape trial reads its evidence after its start "
			"frame");
	for (leg = 0; leg < 2u; leg++) {
		struct escape_start_result r;
		bool bare = leg == 1u;

		escape_start_run(rate, bare, &r);
		printf("# %s trial: arm %s, taps at %+.1f dB against the "
		       "learned gain, start frame fired %d inside a hold of "
		       "%u, next frame holds %u; after it %u short and %u "
		       "long holds\n", bare ? "bare" : "eligible",
		       lecr_trial_arm_name(r.arm), r.taps_db, r.fired,
		       r.hold_left, r.next_hold, r.short_holds, r.long_holds);
		CHECK_UINT(r.arm, UDSP_EC_NLMS_TRIAL_ARM_COHERE);
		CHECK(r.bare == bare);
		CHECK(r.fired);
		CHECK(r.hold_left > 0u);
		CHECK_UINT(r.next_hold, 24);
		if (bare) {
			CHECK_UINT(r.short_holds, 0);
			CHECK(r.long_holds > 0u);
		} else {
			CHECK(r.short_holds > 0u);
			CHECK_UINT(r.long_holds, 0);
		}
	}
}

/* The line of the pause and feed cases: escape_trial_leg with m4, near noise in
 * given pauses, on each frame or one, or frames fed through
 * udsp_ec_nlms_ref_feed. */
#define ESCAPE_PAUSE_MAX		16u

struct escape_pause_plan {
	double near_db;
	unsigned first;
	unsigned last;
	bool single;
	bool feed;
};

/* The loss and act, the pauses, the loudest first and later quiet frames, and
 * each frame's verdict. */
struct escape_pause_result {
	double loss_db;
	unsigned act;
	unsigned pause[ESCAPE_PAUSE_MAX];
	unsigned pauses;
	unsigned fed;
	double first_db;
	double quiet_db;
	uint8_t verdict[ESCAPE_TRIAL_FRAMES];
	bool acted[ESCAPE_TRIAL_FRAMES];
};

/* One frame of the line of escape_pause_leg through udsp_ec_nlms_ref_feed:
 * the far end and its echo path move on, and the canceller takes the
 * reference alone. */
static void escape_feed_frame(struct udsp_ec *ec, struct echo_path *path,
			      struct css_annex *css, uint32_t state[2])
{
	int16_t far[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];

	css_annex_fill(far, path->frame, css);
	level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
	path_run(path, far, echo);
	noise_fill(hiss, path->frame, PHASE2_HISS_DBM0, &state[1]);
	udsp_ec_nlms_ref_feed(ec, ref);
}

static void escape_pause_leg(unsigned rate,
			     const struct escape_pause_plan *plan,
			     struct escape_pause_result *r)
{
	struct udsp_ec_nlms_info info;
	struct udsp_ec_nlms_frame reading;
	struct css_annex css;
	struct echo_path path, next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	uint32_t near_state = 0x7e57ab1eu;
	double a = exp(-1000.0 / (PHASE1_METER_MS * rate));
	double meter = 0.0, loudest = 0.0;
	double erl = phase2_erl_db[0];
	unsigned long from = (unsigned long)rate +
			     path_samples_get(PHASE2_DELAY_MS, rate);
	unsigned long to = from + (unsigned long)rate * 7u / 10u;
	int16_t near[UDSP_FRAME_MAX];
	int16_t out[UDSP_FRAME_MAX];
	bool in_pause = false, was_quiet = false, filled = false;
	bool feeding = false;
	unsigned f, i;

	memset(r, 0, sizeof(*r));
	r->first_db = -200.0;
	r->quiet_db = -200.0;
	css_annex_init(&css, rate, PHASE2_FAR_DBM0, 0x0c55a11eu);
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1, erl + PHASE2_DELTA_ERL_DB,
			  PHASE2_DELAY_MS);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		phase1_frame_run(ec, &path, &css, state, NULL, out);
	path_annex_d_init(&next, rate, phase2_models[0], erl, PHASE2_DELAY_MS);
	path_history_carry(&next, &path);
	for (f = 0; f < ESCAPE_TRIAL_FRAMES; f++) {
		unsigned long at = css.pos % css.half_n;
		bool pause = at >= css.voiced_n + css.noise_n;
		bool quiet;

		if (pause && !in_pause && r->pauses < ESCAPE_PAUSE_MAX) {
			r->pause[r->pauses++] = f;
			filled = false;
		}
		if (plan->feed && in_pause && !pause &&
		    r->pauses == plan->first + 1u)
			feeding = true;
		if (feeding && pause && r->pauses == plan->first + 2u &&
		    at + 3u * next.frame >= css.half_n)
			feeding = false;
		in_pause = pause;
		if (feeding) {
			escape_feed_frame(ec, &next, &css, state);
			r->verdict[f] = LECR_VERDICT_OFF;
			r->fed++;
			was_quiet = false;
			continue;
		}
		memset(near, 0, sizeof(near));
		if (pause && plan->near_db > 0.0 && r->pauses > plan->first &&
		    r->pauses <= plan->last + 1u &&
		    (!plan->single || (was_quiet && !filled))) {
			noise_fill(near, next.frame,
				   PHASE2_HISS_DBM0 + plan->near_db,
				   &near_state);
			filled = true;
		}
		phase1_frame_run(ec, &next, &css, state, near, out);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		quiet = reading.nlp_verdict == LECR_VERDICT_QUIET;
		if (quiet && reading.noise_power > 0.0f) {
			double db = 10.0 * log10(reading.mic_power /
						 reading.noise_power);
			double *keep = was_quiet ? &r->quiet_db : &r->first_db;

			if (db > *keep)
				*keep = db;
		}
		was_quiet = quiet;
		r->verdict[f] = reading.nlp_verdict;
		r->acted[f] = info.nlp_active;
		if (!r->act && info.nlp_active && f > ESCAPE_TRIAL_SKIP_FRAMES)
			r->act = f;
		for (i = 0; i < next.frame; i++) {
			unsigned long n = (unsigned long)f * next.frame + i;

			meter = a * meter + (1.0 - a) * out[i] * out[i];
			if (n >= from && n < to && meter > loudest)
				loudest = meter;
		}
	}
	r->loss_db = PHASE2_FAR_DBM0 + ESCAPE_TRIAL_L_ACT_DB -
		     rms_dbm0_get(sqrt(loudest));
	udsp_ec_nlms.destroy(ec);
}

/*
 * The frames of a leg of escape_pause_leg, [*start, *end). They run from the
 * end of the quiet frames of the pause that starts on frame from. They end at
 * the first quiet frame at or after frame to.
 */
static void escape_pause_span(const struct escape_pause_result *r,
			      unsigned from, unsigned to, unsigned *start,
			      unsigned *end)
{
	unsigned f = from;

	while (f < ESCAPE_TRIAL_FRAMES && r->verdict[f] != LECR_VERDICT_QUIET)
		f++;
	while (f < ESCAPE_TRIAL_FRAMES && r->verdict[f] == LECR_VERDICT_QUIET)
		f++;
	*start = f;
	f = to > f ? to : f;
	while (f < ESCAPE_TRIAL_FRAMES && r->verdict[f] != LECR_VERDICT_QUIET)
		f++;
	*end = f;
}

/*
 * The acted frames and the dt verdicts of a leg of escape_pause_leg over
 * [start, end).
 */
static void escape_pause_count(const struct escape_pause_result *r,
			       unsigned start, unsigned end, unsigned *acted,
			       unsigned *dt)
{
	unsigned f;

	*acted = 0;
	*dt = 0;
	for (f = start; f < end; f++) {
		*acted += r->acted[f];
		*dt += r->verdict[f] == LECR_VERDICT_DT;
	}
}

/* The first pause of a leg of escape_pause_leg that starts after frame act,
 * or the count of its pauses. */
static unsigned escape_pause_after(const struct escape_pause_result *r,
				   unsigned act)
{
	unsigned p;

	for (p = 0; p < r->pauses && r->pause[p] <= act; p++)
		;
	return p;
}

/*
 * ESCAPE_PAUSE_BAR_DB: the bar of LEC_PAUSE_NEAR_RATIO in dsp/lec.c.
 * ESCAPE_PAUSE_UNDER_DB: a near noise in each pause, over the line noise and
 * under that bar. ESCAPE_PAUSE_NEAR_DB: a near noise in one pause, over it.
 */
#define ESCAPE_PAUSE_BAR_DB		18.0
#define ESCAPE_PAUSE_UNDER_DB		14.0
#define ESCAPE_PAUSE_NEAR_DB		30.0

/*
 * The near end of the far pauses in an escape trial (LEC_PAUSE_NEAR_RATIO).
 * With no near end, or noise ESCAPE_PAUSE_UNDER_DB over the line noise, the
 * processor acts by ESCAPE_TRIAL_ACT_FRAMES at the loss. With
 * ESCAPE_PAUSE_NEAR_DB in one pause, each frame to the next pause reads dt.
 */
static void escape_trial_pause_test(unsigned rate)
{
	static struct escape_pause_result line, under, near, one;
	struct escape_pause_plan plan = { 0.0, 0u, 0u, false, false };
	unsigned p, start, end, next_start, next_end;
	unsigned line_acts, line_dt, near_acts, held, one_acts, one_held;
	unsigned again, again_dt;

	case_rate(rate, "lec withdraws the pass of an escape trial on a near "
			"end in a far pause");
	escape_pause_leg(rate, &plan, &line);
	plan.near_db = ESCAPE_PAUSE_UNDER_DB;
	plan.last = ESCAPE_PAUSE_MAX;
	escape_pause_leg(rate, &plan, &under);
	p = escape_pause_after(&line, line.act);
	plan.near_db = ESCAPE_PAUSE_NEAR_DB;
	plan.first = p;
	plan.last = p;
	escape_pause_leg(rate, &plan, &near);
	plan.single = true;
	escape_pause_leg(rate, &plan, &one);
	printf("# line: combined loss %.1f dB at 1 s, first act on frame %u; "
	       "first quiet frames up to %.1f dB over the floor, later quiet "
	       "frames up to %.1f dB\n", line.loss_db, line.act, line.first_db,
	       line.quiet_db);
	printf("# under the bar: combined loss %.1f dB at 1 s, first act on "
	       "frame %u; later quiet frames up to %.1f dB over the floor\n",
	       under.loss_db, under.act, under.quiet_db);
	CHECK(p + 2u < line.pauses);
	if (p + 2u >= line.pauses)
		return;
	escape_pause_span(&near, near.pause[p], near.pause[p + 1u], &start,
			  &end);
	escape_pause_span(&near, near.pause[p + 1u], near.pause[p + 2u],
			  &next_start, &next_end);
	escape_pause_count(&line, start, end, &line_acts, &line_dt);
	escape_pause_count(&near, start, end, &near_acts, &held);
	escape_pause_count(&one, start, end, &one_acts, &one_held);
	escape_pause_count(&near, next_start, next_end, &again, &again_dt);
	printf("# near in the pause of frame %u: later quiet frames up to "
	       "%.1f dB over the floor; frames %u to %u: %u dt verdicts, %u "
	       "acted (line %u); frames %u to %u: %u acted\n", near.pause[p],
	       near.quiet_db, start, end - 1u, held, near_acts, line_acts,
	       next_start, next_end - 1u, again);
	printf("# near on one frame of that pause: up to %.1f dB over the "
	       "floor; frames %u to %u: %u dt verdicts, %u acted\n",
	       one.quiet_db, start, end - 1u, one_held, one_acts);
	if (rate == UDSP_SAMPLE_RATE)
		CHECK(line.first_db >= ESCAPE_PAUSE_BAR_DB);
	CHECK(line.act > 0u && line.act <= ESCAPE_TRIAL_ACT_FRAMES);
	CHECK(line.loss_db >= ESCAPE_TRIAL_LOSS_DB);
	CHECK(under.act > 0u && under.act <= ESCAPE_TRIAL_ACT_FRAMES);
	CHECK(under.loss_db >= ESCAPE_TRIAL_LOSS_DB);
	CHECK(end > start && line_acts > 0u);
	CHECK_UINT(held, end - start);
	CHECK_UINT(near_acts, 0);
	CHECK(again > 0u);
	CHECK(one.quiet_db >= ESCAPE_PAUSE_BAR_DB);
	CHECK_UINT(one_held, end - start);
	CHECK_UINT(one_acts, 0);
}

/*
 * The near end of the far-end pauses across reference feeds (dsp/lec.c,
 * udsp_ec_nlms_ref_feed). After a pause that reads a near end, the far burst
 * and the next pause go through udsp_ec_nlms_ref_feed. The last quiet frames of
 * that pause run with no near end. On the next burst the processor must
 * act, and the dt verdict must not occur.
 */
static void escape_trial_feed_test(unsigned rate)
{
	static struct escape_pause_result line, feed;
	struct escape_pause_plan plan = { 0.0, 0u, 0u, false, false };
	unsigned p, start, end, acts, dt;

	case_rate(rate, "lec reads a fresh far pause after reference feeds in "
			"an escape trial");
	escape_pause_leg(rate, &plan, &line);
	p = escape_pause_after(&line, line.act);
	plan.near_db = ESCAPE_PAUSE_NEAR_DB;
	plan.first = p;
	plan.last = p;
	plan.feed = true;
	escape_pause_leg(rate, &plan, &feed);
	CHECK(p + 2u < feed.pauses);
	if (p + 2u >= feed.pauses)
		return;
	escape_pause_span(&feed, feed.pause[p + 1u], feed.pause[p + 2u],
			  &start, &end);
	escape_pause_count(&feed, start, end, &acts, &dt);
	printf("# near in the pause of frame %u: later quiet frames up to "
	       "%.1f dB over the floor; %u frames fed; frames %u to %u: %u "
	       "acted, %u dt verdicts\n", feed.pause[p], feed.quiet_db,
	       feed.fed, start, end - 1u, acts, dt);
	CHECK(feed.quiet_db >= ESCAPE_PAUSE_BAR_DB);
	CHECK(feed.fed > 0u);
	CHECK(end > start && acts > 0u);
	CHECK_UINT(dt, 0);
}

/* The microphone over the tracked floor from which a tail frame over the
 * bound refuses the resume, LEC_NLP_TAIL_FLOOR_RATIO of dsp/lec.c. */
#define MISS_TAIL_FLOOR_RATIO	16.0

/* The patterns of tail_peak_test, each after a pause of the line floor: a
 * frame over the bound first, second and third in its pause, and a frame 12
 * dB over the floor first and second. */
enum {
	TAIL_LEG_FIRST,
	TAIL_LEG_SECOND,
	TAIL_LEG_THIRD,
	TAIL_LEG_LOUD_FIRST,
	TAIL_LEG_LOUD_SECOND,
	TAIL_LEGS
};

static const char *const tail_patterns[TAIL_LEGS] = {
	"p---", "-p--", "--p-", "P---", "-P--",
};

/* The legs of every phase on the quiet-miss line with its echo product
 * MISS_RES_DEEP_DB under the echo. */
static void tail_phases_run(unsigned rate,
			    struct miss_leg legs[MISS_PHASES][TAIL_LEGS])
{
	unsigned frame = rate / 100u;
	unsigned k;
	unsigned n;

	for (k = 0; k < MISS_PHASES; k++) {
		struct miss_leg floor;
		struct miss_line l;

		miss_line_init(&l, rate, k * frame / MISS_PHASES,
			       MISS_RES_DEEP_DB);
		miss_line_settle(&l);
		for (n = 0; n < TAIL_LEGS; n++) {
			miss_leg_run(&l, miss_patterns[MISS_LEG_FLOOR], &floor);
			miss_leg_run(&l, tail_patterns[n], &legs[k][n]);
		}
		udsp_ec_nlms.destroy(l.ec);
	}
}

/* The figures of leg over every phase: the frame over the bound against the
 * bar and the floor, the first acting frame and the waits. */
static void tail_legs_print(struct miss_leg legs[MISS_PHASES][TAIL_LEGS],
			    unsigned leg)
{
	double lo = 200.0;
	double hi = -200.0;
	double flo = 200.0;
	double fhi = -200.0;
	double erle = 200.0;
	unsigned first = 0;
	unsigned waits = 0;
	unsigned k;

	for (k = 0; k < MISS_PHASES; k++) {
		const struct miss_leg *g = &legs[k][leg];

		lo = g->lo_peak < lo ? g->lo_peak : lo;
		hi = g->hi_peak > hi ? g->hi_peak : hi;
		flo = g->lo_peak_floor < flo ? g->lo_peak_floor : flo;
		fhi = g->hi_peak_floor > fhi ? g->hi_peak_floor : fhi;
		erle = g->onset_erle_db < erle ? g->onset_erle_db : erle;
		first = g->first > first ? g->first : first;
		waits = g->waits > waits ? g->waits : waits;
	}
	printf("# pause %s over %u phases: the frame %.2f to %.2f dB over the "
	       "bar and %.2f to %.2f dB over the floor; the burst acts from "
	       "its frame %u at most, %u wait frames at most; ERLE tracker "
	       "from %.2f dB\n", tail_patterns[leg], MISS_PHASES, lo, hi, flo,
	       fhi, first, waits, erle);
}

/* The checks of the frame of a leg: over the bound, under or over the floor
 * bound. */
static void tail_frame_check(const struct miss_leg *g, bool loud)
{
	CHECK_UINT(g->quiet, MISS_QUIET_FRAMES);
	CHECK(g->lo_peak > 10.0 * log10(MISS_PEAK_RATIO));
	CHECK(g->onset_erle_db >= MISS_RESUME_DB);
	if (loud)
		CHECK(g->lo_peak_floor > 10.0 * log10(MISS_TAIL_FLOOR_RATIO));
	else
		CHECK(g->hi_peak_floor < 10.0 * log10(MISS_TAIL_FLOOR_RATIO));
}

/* The checks of a leg whose burst waits: it acts on its 8th frame after 7
 * wait frames and passes the filter output on the first. */
static void tail_wait_check(const struct miss_leg *g)
{
	CHECK_UINT(g->first, WORDS_ENGAGE_FRAME);
	CHECK_UINT(g->waits, WORDS_ENGAGE_FRAME);
	CHECK(g->edge_db > MISS_EDGE_TOL_DB);
}

/*
 * One frame over the peak bound in a far pause at 8 phases. As one of the first
 * two quiet frames (LEC_NLP_TAIL_QUIET_FRAMES) and under 12 dB over the floor
 * it leaves the resume. Third in its pause, or 12 dB over the floor, it refuses
 * it, and the burst waits 7 frames.
 */
static void tail_peak_test(unsigned rate)
{
	static struct miss_leg legs[MISS_PHASES][TAIL_LEGS];
	unsigned k;

	tail_phases_run(rate, legs);
	case_rate(rate, "lec nlp resumes the engage run after a frame over "
		  "the peak bound among the first two quiet frames of a "
		  "pause");
	tail_legs_print(legs, TAIL_LEG_FIRST);
	tail_legs_print(legs, TAIL_LEG_SECOND);
	for (k = 0; k < MISS_PHASES; k++) {
		const struct miss_leg *first = &legs[k][TAIL_LEG_FIRST];
		const struct miss_leg *second = &legs[k][TAIL_LEG_SECOND];

		tail_frame_check(first, false);
		tail_frame_check(second, false);
		CHECK_UINT(first->first, 0);
		CHECK(first->first_pass);
		CHECK_UINT(first->waits, 0);
		CHECK(fabs(first->edge_db) <= MISS_EDGE_TOL_DB);
		CHECK_UINT(second->first, 0);
		CHECK(second->first_pass);
		CHECK_UINT(second->waits, 0);
		CHECK(fabs(second->edge_db) <= MISS_EDGE_TOL_DB);
	}
	case_rate(rate, "lec nlp waits after a frame over the peak bound "
		  "third in a pause");
	tail_legs_print(legs, TAIL_LEG_THIRD);
	for (k = 0; k < MISS_PHASES; k++) {
		tail_frame_check(&legs[k][TAIL_LEG_THIRD], false);
		tail_wait_check(&legs[k][TAIL_LEG_THIRD]);
	}
	case_rate(rate, "lec nlp waits after a frame 12 dB over the floor "
		  "among the first two quiet frames of a pause");
	tail_legs_print(legs, TAIL_LEG_LOUD_FIRST);
	tail_legs_print(legs, TAIL_LEG_LOUD_SECOND);
	for (k = 0; k < MISS_PHASES; k++) {
		tail_frame_check(&legs[k][TAIL_LEG_LOUD_FIRST], true);
		tail_frame_check(&legs[k][TAIL_LEG_LOUD_SECOND], true);
		tail_wait_check(&legs[k][TAIL_LEG_LOUD_FIRST]);
		tail_wait_check(&legs[k][TAIL_LEG_LOUD_SECOND]);
	}
}

/* The legs of hold_resume_test and hold_refuse_test: a pause, its burst and up
 * to two near pulses on burst or tail frames. HOLD_PULSE_DB, within 3 dB of the
 * active bar and under 12 dB over the floor, takes the short hangover. */
#define HOLD_PULSE_DB		8.0
#define HOLD_AT_TAIL		-1
/* The pulse of the line with its echo product MISS_RES_LOW_DB under the
 * echo, whose bar stands higher. */
#define HOLD_LOW_PULSE_DB	10.7
/* The bar of the onset frame moves with the burst start, so a pulse can
 * stand under it or more than 3 dB over it: the phases whose pulse takes the
 * short hangover, of the 8, at least. */
#define HOLD_MIN_PHASES		4u
#define HOLD_PULSES		2u
/* The quiet frames of a pause on the line of the through-pause legs. The short
 * hold of the last tail frame then reaches the next burst. */
#define HOLD_SPAN_QUIET		2u

struct hold_pulse {
	int at;
	double db;
};

struct hold_spec {
	const char *pattern;
	struct hold_pulse pulse[HOLD_PULSES];
	unsigned pulses;
	const struct hold_spec *setup;
};

/*
 * The figures of a leg: whether the last pulse fired, the hangover it set and
 * its level over bar and floor, and whether the processor acted before it. Then
 * the hold's end, the first acting frame and the waits, the ERLE at the end,
 * and the first pulse's figures on a two-pulse leg.
 */
struct hold_leg {
	bool fired;
	unsigned set;
	bool lead_fired;
	unsigned lead_set;
	double over_bar;
	double over_floor;
	bool acted;
	unsigned end;
	unsigned first;
	unsigned waits;
	double erle_db;
};

/* The pulse level of the frame at, or 0 for none. */
static double hold_pulse_db(const struct hold_spec *sp, int at)
{
	unsigned n;

	for (n = 0; n < sp->pulses; n++) {
		if (sp->pulse[n].at == at)
			return sp->pulse[n].db;
	}
	return 0.0;
}

/* The pause of the period from the frame the line stands on, with the
 * pattern of sp on its quiet frames. Then the burst of the next period, with
 * the pulses of sp. */
static void hold_leg_run(struct miss_line *l, const struct hold_spec *sp,
			 struct hold_leg *g)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned from = l->period - l->quiet;
	int last = sp->pulse[sp->pulses - 1u].at;
	int lead = sp->pulse[0].at;
	bool two = sp->pulses > 1u;
	bool pending = false;
	bool lead_pending = false;
	bool held = false;
	unsigned f;

	memset(g, 0, sizeof(*g));
	g->end = MISS_BURST_FRAMES;
	g->first = MISS_BURST_FRAMES;
	for (f = miss_phase_get(l); f < l->period; f++) {
		int at = (int)f - (int)from;
		bool tail = at < 0;
		char c = tail ? '-' : sp->pattern[at];
		double db = tail ? hold_pulse_db(sp, at) : 0.0;

		miss_frame_run(l, db != 0.0 ? db : miss_level_get(c), &reading,
			       &info);
		/* hold_left reads the hangover that the verdict of the frame
		 * before set. */
		if (pending)
			g->set = reading.hold_left;
		if (lead_pending)
			g->lead_set = reading.hold_left;
		pending = tail && at == last;
		lead_pending = two && tail && at == lead;
		if (lead_pending)
			g->lead_fired = reading.dt_fired;
		if (tail && at == last) {
			g->fired = reading.dt_fired;
			g->over_bar = ratio_db_get(reading.mic_power,
						   reading.bar);
			g->over_floor = ratio_db_get(reading.mic_power,
						     reading.noise_power);
			held = reading.dt_fired;
		}
	}
	for (f = 0; f < MISS_BURST_FRAMES; f++) {
		double db = hold_pulse_db(sp, (int)f);

		miss_frame_run(l, db, &reading, &info);
		if (pending)
			g->set = reading.hold_left;
		if (lead_pending)
			g->lead_set = reading.hold_left;
		pending = (int)f == last;
		lead_pending = two && (int)f == lead;
		if (lead_pending)
			g->lead_fired = reading.dt_fired;
		if ((int)f == last) {
			g->fired = reading.dt_fired;
			g->over_bar = ratio_db_get(reading.mic_power,
						   reading.bar);
			g->over_floor = ratio_db_get(reading.mic_power,
						     reading.noise_power);
			held = reading.dt_fired;
		}
		if ((int)f < last && info.nlp_active)
			g->acted = true;
		if (held && g->end == MISS_BURST_FRAMES &&
		    reading.nlp_verdict != LECR_VERDICT_DT) {
			g->end = f;
			g->erle_db = reading.erle_db;
		}
		if (g->end < MISS_BURST_FRAMES &&
		    g->first == MISS_BURST_FRAMES) {
			if (info.nlp_active)
				g->first = f;
			else if (reading.nlp_verdict == LECR_VERDICT_PASS)
				g->waits++;
		}
	}
}

/* The figures of each leg at every phase. The echo product of the line stands
 * res_db under the echo, and its pauses hold quiet quiet frames. Each leg,
 * or its setup, follows a pause of the line floor. */
static void hold_phases_run(unsigned rate, double res_db, unsigned quiet,
			    const struct hold_spec *specs, unsigned count,
			    struct hold_leg legs[][MISS_PHASES])
{
	static const char floor_quiet[] = "----";
	unsigned frame = rate / 100u;
	unsigned k;
	unsigned n;

	for (k = 0; k < MISS_PHASES; k++) {
		struct miss_leg floor;
		struct hold_leg setup;
		struct miss_line l;

		miss_line_init(&l, rate, k * frame / MISS_PHASES, res_db);
		miss_line_quiet_set(&l, quiet);
		miss_line_settle(&l);
		for (n = 0; n < count; n++) {
			miss_leg_run(&l, floor_quiet + MISS_QUIET_FRAMES - quiet,
				     &floor);
			if (specs[n].setup)
				hold_leg_run(&l, specs[n].setup, &setup);
			hold_leg_run(&l, &specs[n], &legs[n][k]);
		}
		udsp_ec_nlms.destroy(l.ec);
	}
}

static void hold_legs_print(const char *what, const struct hold_leg *legs)
{
	double bar_lo = 200.0, bar_hi = -200.0;
	double floor_lo = 200.0, floor_hi = -200.0;
	double erle = 200.0;
	unsigned set_lo = 255, set_hi = 0;
	unsigned waits = 0;
	unsigned k;

	for (k = 0; k < MISS_PHASES; k++) {
		const struct hold_leg *g = &legs[k];

		bar_lo = fmin(bar_lo, g->over_bar);
		bar_hi = fmax(bar_hi, g->over_bar);
		floor_lo = fmin(floor_lo, g->over_floor);
		floor_hi = fmax(floor_hi, g->over_floor);
		if (g->end < MISS_BURST_FRAMES)
			erle = fmin(erle, g->erle_db);
		set_lo = g->set < set_lo ? g->set : set_lo;
		set_hi = g->set > set_hi ? g->set : set_hi;
		waits = g->waits > waits ? g->waits : waits;
	}
	printf("# %s over %u phases: the pulse %.2f to %.2f dB over the bar "
	       "and %.2f to %.2f dB over the floor, hangover %u to %u; %u wait "
	       "frames at most after the hold; ERLE tracker from %.2f dB\n",
	       what, MISS_PHASES, bar_lo, bar_hi, floor_lo, floor_hi, set_lo,
	       set_hi, waits, erle);
}

static bool hold_short_get(const struct hold_leg *g)
{
	return g->fired && g->set == TAIL_PAIR_SHORT_FRAMES &&
	       g->over_floor < 10.0 * log10(PAIR_SCRIPT_FLOOR_RATIO);
}

/* The phases of a leg whose last pulse took the short hangover, checked to
 * be HOLD_MIN_PHASES or more. */
static unsigned hold_short_count(const struct hold_leg *legs)
{
	unsigned n = 0;
	unsigned k;

	for (k = 0; k < MISS_PHASES; k++)
		n += hold_short_get(&legs[k]);
	printf("# %u of %u phases take the short hangover\n", n, MISS_PHASES);
	CHECK(n >= HOLD_MIN_PHASES);
	return n;
}

/* The checks of a leg that resumes: the first frame after the hold acts. */
static void hold_resume_check(const struct hold_leg *g)
{
	CHECK(g->end < MISS_BURST_FRAMES);
	CHECK_UINT(g->first, g->end);
	CHECK_UINT(g->waits, 0);
	CHECK(g->erle_db >= MISS_RESUME_DB);
}

/* The checks of a leg that waits 7 frames after the hold. */
static void hold_wait_check(const struct hold_leg *g)
{
	CHECK(g->end < MISS_BURST_FRAMES);
	CHECK_UINT(g->first, g->end + WORDS_ENGAGE_FRAME);
	CHECK_UINT(g->waits, WORDS_ENGAGE_FRAME);
}

/* The legs of hold_resume_test: a pulse inside the burst, on the first frame
 * of the burst, and on the last tail frame before a pause of two quiet
 * frames. */
static const struct hold_spec hold_resume_specs[] = {
	{ "----", { { 1, HOLD_PULSE_DB } }, 1u, NULL },
	{ "----", { { 0, HOLD_PULSE_DB } }, 1u, NULL },
	{ "--", { { HOLD_AT_TAIL, HOLD_PULSE_DB } }, 1u, NULL },
};

/*
 * The resume after a short hold (nlp_hold_update) at 8 phases. A pulse near the
 * floor inside an acted burst, after a pause, or on the last tail frame takes
 * the short hangover. The first pass frame after the hold acts, where a line
 * without the resume waits 7 frames.
 */
static void hold_resume_test(unsigned rate)
{
	static struct hold_leg legs[3][MISS_PHASES];
	unsigned k;

	hold_phases_run(rate, MISS_RES_DEEP_DB, MISS_QUIET_FRAMES,
			hold_resume_specs, 2u, legs);
	hold_phases_run(rate, MISS_RES_DEEP_DB, HOLD_SPAN_QUIET,
			&hold_resume_specs[2], 1u, &legs[2]);
	case_rate(rate, "lec nlp resumes the engage run after the short hold "
		  "of a verdict inside an acted burst");
	hold_legs_print("pulse on burst frame 1", legs[0]);
	hold_short_count(legs[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (!hold_short_get(&legs[0][k]))
			continue;
		CHECK(legs[0][k].acted);
		hold_resume_check(&legs[0][k]);
	}
	case_rate(rate, "lec nlp resumes the engage run after a short hold "
		  "from the first frame after a pause");
	hold_legs_print("pulse on burst frame 0", legs[1]);
	hold_short_count(legs[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (hold_short_get(&legs[1][k]))
			hold_resume_check(&legs[1][k]);
	}
	case_rate(rate, "lec nlp resumes the engage run after a short hold "
		  "through a far pause");
	hold_legs_print("pulse on the last tail frame", legs[2]);
	hold_short_count(legs[2]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (hold_short_get(&legs[2][k]))
			hold_resume_check(&legs[2][k]);
	}
}

/* The setup of the legs after a burst without an act: pulses 12 dB over the
 * floor on burst frames 0 and 1, whose long hold covers the burst. */
static const struct hold_spec hold_unacted_setup = {
	"----", { { 0, MISS_LOUD_DB }, { 1, MISS_LOUD_DB } }, 2u, NULL,
};

/* The legs of hold_refuse_test on the line with its echo product
 * MISS_RES_DEEP_DB under the echo: after a burst without an act, a pulse on
 * burst frame 0 and one on burst frame 1. */
static const struct hold_spec hold_refuse_specs[] = {
	{ "----", { { 0, HOLD_PULSE_DB } }, 1u, &hold_unacted_setup },
	{ "----", { { 1, HOLD_PULSE_DB } }, 1u, &hold_unacted_setup },
};

/* The leg of the line with its echo product MISS_RES_LOW_DB under the echo:
 * a pulse on burst frame 1. */
static const struct hold_spec hold_low_spec = {
	"----", { { 1, HOLD_LOW_PULSE_DB } }, 1u, NULL,
};

/* The legs whose pauses hold two quiet frames, a pulse on the last tail frame.
 * One adds a quiet frame 12 dB over the floor in the pause. The other adds a
 * loud pulse on burst frame 0 inside the short hangover. */
static const struct hold_spec hold_refuse_span[] = {
	{ "P-", { { HOLD_AT_TAIL, HOLD_PULSE_DB } }, 1u, NULL },
	{ "--", { { HOLD_AT_TAIL, HOLD_PULSE_DB }, { 0, MISS_LOUD_DB } }, 2u,
	  NULL },
};

/*
 * The conditions of the resume after a short hold at 8 phases. These holds
 * wait 7 frames: through a pause with a loud frame, after or inside a burst
 * without an act, and with the tracker under the resume level. An onset
 * verdict inside a short hangover takes the long one, and no frame after it
 * acts.
 */
static void hold_refuse_test(unsigned rate)
{
	static struct hold_leg legs[2][MISS_PHASES];
	static struct hold_leg span[2][MISS_PHASES];
	static struct hold_leg low[1][MISS_PHASES];
	unsigned k;

	hold_phases_run(rate, MISS_RES_DEEP_DB, MISS_QUIET_FRAMES,
			hold_refuse_specs, 2u, legs);
	hold_phases_run(rate, MISS_RES_DEEP_DB, HOLD_SPAN_QUIET,
			hold_refuse_span, 2u, span);
	hold_phases_run(rate, MISS_RES_LOW_DB, MISS_QUIET_FRAMES,
			&hold_low_spec, 1u, low);
	case_rate(rate, "lec nlp waits after a short hold through a pause "
		  "with a frame over the bound");
	hold_legs_print("pulse on the last tail frame before P-", span[0]);
	hold_short_count(span[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (hold_short_get(&span[0][k]))
			hold_wait_check(&span[0][k]);
	}
	case_rate(rate, "lec nlp waits after a short hold from a far onset "
		  "after a burst without an act");
	hold_legs_print("pulse on burst frame 0 after an unacted burst",
			legs[0]);
	hold_short_count(legs[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (hold_short_get(&legs[0][k]))
			hold_wait_check(&legs[0][k]);
	}
	case_rate(rate, "lec nlp waits after a short hold inside a burst "
		  "without an act");
	hold_legs_print("pulse on burst frame 1 after an unacted burst",
			legs[1]);
	hold_short_count(legs[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (!hold_short_get(&legs[1][k]))
			continue;
		CHECK(!legs[1][k].acted);
		hold_wait_check(&legs[1][k]);
	}
	case_rate(rate, "lec nlp waits after a short hold with the ERLE "
		  "tracker under the resume level");
	hold_legs_print("pulse on burst frame 1, low line", low[0]);
	hold_short_count(low[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		if (!hold_short_get(&low[0][k]))
			continue;
		CHECK(low[0][k].acted);
		hold_wait_check(&low[0][k]);
		CHECK(low[0][k].erle_db < MISS_RESUME_DB);
	}
	case_rate(rate, "lec nlp acts on no frame after a long hold");
	hold_legs_print("pulse 13.5 dB over the floor on burst frame 0 inside "
			"a short hangover", span[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		CHECK(span[1][k].fired);
		CHECK(span[1][k].set > TAIL_PAIR_SHORT_FRAMES);
		CHECK(span[1][k].end < MISS_BURST_FRAMES);
		CHECK_UINT(span[1][k].first, MISS_BURST_FRAMES);
		CHECK_UINT(span[1][k].waits,
			   MISS_BURST_FRAMES - span[1][k].end);
	}
}

/*
 * The resume after a short hold inside an escape trial, on a copy turned over
 * at echo delay. The first pass frame after the trial's short hold carries the
 * copy, and the processor does not act on it. An escape trial acts on its own
 * evidence alone.
 */
static void hold_trial_test(unsigned rate)
{
	static const struct expose_row rows[] = {
		{ "white copy at the echo delay turned over, 3 times the echo",
		  EXPOSE_WHITE, EXPOSE_COPY, EXPOSE_INVERT_LOW_DB,
		  EXPOSE_ECHO_MS, 0, false, true, 4.0, 1 },
		{ "white copy at the echo delay turned over, 12 times the echo",
		  EXPOSE_WHITE, EXPOSE_COPY, EXPOSE_INVERT_HIGH_DB,
		  EXPOSE_ECHO_MS, 0, false, true, 6.0, 1 },
	};
	unsigned i;

	case_rate(rate, "lec nlp resumes no engage run after a short hold "
		  "inside an escape trial");
	for (i = 0; i < 2u; i++) {
		struct expose_count off;
		struct expose_count on;

		expose_row_run(&rows[i], rate, &off, &on);
		printf("# %u Hz, %s: %u first pass frames after a short hold "
		       "inside a trial, the processor acts on %u\n", rate,
		       rows[i].what, on.hold_pass, on.hold_acted);
		CHECK(on.hold_pass >= 1u);
		CHECK_UINT(on.hold_acted, 0);
	}
}

/* The legs of hold_cut_test on the nlp_cut_test fixture: an untrusted frozen
 * filter, a near pulse on a far-silent frame in far noise. It fires on a tail
 * frame, then the processor is off or a frame reads unlike. */
#define HOLD_CUT_ERL_DB		23.0
#define HOLD_CUT_TRAIN_FRAMES	80u
#define HOLD_CUT_NEAR_DBM0	-15.0
#define HOLD_CUT_FRAMES		30u

enum hold_cut {
	HOLD_CUT_NONE,
	HOLD_CUT_OFF,
	HOLD_CUT_UNLIKE,
};

/*
 * The figures of a leg: whether the processor acted before the pulse, whether
 * it fired and the hangover it set, and the dt frames after it. Then the
 * verdict of the frame that follows them, and the pass frames to the first
 * act after the hold. Last, the ERLE on the first pass frame.
 */
struct hold_cut_leg {
	bool engaged;
	bool fired;
	unsigned set;
	unsigned held;
	unsigned verdict;
	unsigned at;
	double erle_db;
};

static void hold_cut_run(unsigned rate, enum hold_cut cut,
			 struct hold_cut_leg *g)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path;
	struct udsp_ec *ec;
	uint32_t state = 0x31415926u;
	uint32_t near_state = 0xa5a5c3c3u;
	unsigned acted = 0;
	unsigned passes = 0;
	unsigned f;

	memset(g, 0, sizeof(*g));
	g->at = HOLD_CUT_FRAMES;
	ec = gate_fixture_rate_create(&path, rate, 8u, HOLD_CUT_ERL_DB,
				      HOLD_CUT_TRAIN_FRAMES, &state, &acted);
	verdict_far_run(ec, &path, &state, FRAMES_PER_SECOND, &acted);
	udsp_ec_nlms_info_get(ec, &info);
	g->engaged = info.nlp_active;
	near_step(ec, &path, HOLD_CUT_NEAR_DBM0, &near_state, &acted);
	udsp_ec_nlms_frame_get(ec, &reading);
	g->fired = reading.dt_fired;
	/* hold_left reads the hangover that the verdict of the frame before
	 * set. The frame that reads 1 is the first after the hangover. */
	do {
		verdict_far_run(ec, &path, &state, 1u, &acted);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (!g->set)
			g->set = reading.hold_left;
		g->held += reading.nlp_verdict == LECR_VERDICT_DT;
	} while (reading.hold_left > 2u && g->held < HOLD_CUT_FRAMES);
	g->verdict = LECR_VERDICT_PASS;
	if (cut == HOLD_CUT_OFF) {
		udsp_ec_nlms.nlp(ec, false);
		verdict_far_run(ec, &path, &state, 1u, &acted);
		udsp_ec_nlms.nlp(ec, true);
	} else if (cut == HOLD_CUT_UNLIKE) {
		near_step(ec, &path, FLAP_NEAR_DBM0, &near_state, &acted);
	}
	if (cut != HOLD_CUT_NONE) {
		udsp_ec_nlms_frame_get(ec, &reading);
		g->verdict = reading.nlp_verdict;
	}
	for (f = 0; f < HOLD_CUT_FRAMES && g->at == HOLD_CUT_FRAMES; f++) {
		verdict_far_run(ec, &path, &state, 1u, &acted);
		udsp_ec_nlms_frame_get(ec, &reading);
		udsp_ec_nlms_info_get(ec, &info);
		if (reading.nlp_verdict != LECR_VERDICT_PASS)
			continue;
		if (!passes)
			g->erle_db = reading.erle_db;
		if (info.nlp_active)
			g->at = passes;
		passes++;
	}
	udsp_ec_nlms.destroy(ec);
}

/*
 * An off or unlike frame between a short hold and the next pass frame ends the
 * hold. Without one the first pass frame acts; after one the burst waits
 * REFUSAL_WAIT pass frames, with the tracker at MISS_RESUME_DB or more on each
 * first pass frame.
 */
static void hold_cut_test(unsigned rate)
{
	static const char *const names[3] = { "no frame", "an off frame",
					       "an unlike frame" };
	static const unsigned verdicts[3] = { LECR_VERDICT_PASS,
					      LECR_VERDICT_OFF,
					      LECR_VERDICT_UNLIKE };
	unsigned k;

	case_rate(rate, "lec nlp ends a short hold on an off or unlike frame "
		  "before the next pass frame");
	for (k = 0; k < 3u; k++) {
		struct hold_cut_leg g;

		hold_cut_run(rate, (enum hold_cut)k, &g);
		printf("# %s after the hold: the pulse fired %u and set %u, %u "
		       "dt frames, then %s; the processor acts on pass frame "
		       "%u after the hold, counted from 0; ERLE tracker %.2f "
		       "dB\n", names[k], g.fired, g.set, g.held,
		       udsp_ec_nlms_verdict_name(g.verdict), g.at, g.erle_db);
		CHECK(g.engaged);
		CHECK(g.fired);
		CHECK_UINT(g.set, TAIL_PAIR_SHORT_FRAMES);
		CHECK(g.erle_db >= MISS_RESUME_DB);
		CHECK_UINT(g.verdict, verdicts[k]);
		CHECK_UINT(g.at, k ? REFUSAL_WAIT : 0u);
	}
}

/* The bar bound of a thin pair (LEC_GEIGEL_PAIR_RATIO) on the late line. Each
 * second verdict stands 3 to 6 dB over the bar and within 12 dB of the floor,
 * and keeps the short hangover. */
static void hangover_pair_bar_test(unsigned rate)
{
	static const struct tail_pair_leg leg = {
		"under the bar bound", { 0.0, 40.0 }, { -60.0, 0.0 },
		{ 40u, 112u },
	};
	struct tail_pair_result r;

	case_rate(rate, "lec hangover of a thin pair 3 to 6 dB over its bar");
	tail_pair_run(rate, &leg, &r);
	tail_pair_print(rate, leg.name, &r);
	CHECK(r.thin >= TAIL_PAIR_MIN);
	CHECK_UINT(r.thin_short, r.thin);
	CHECK(r.bar_lo >= 10.0 * log10(TAIL_PAIR_MARGINAL_RATIO));
	CHECK(r.floor_hi < 10.0 * log10(TAIL_PAIR_FLOOR_RATIO));
	CHECK_UINT(r.active, 0);
}

/* The leg of hold_repeat_test on the line with its echo product
 * MISS_RES_DEEP_DB under the echo: a pulse on the last tail frame, a thin
 * frame, and one on burst frame 1, an active frame, 6 frames later. */
static const struct hold_spec hold_repeat_spec = {
	"----", { { HOLD_AT_TAIL, HOLD_PULSE_DB }, { 1, HOLD_PULSE_DB } }, 2u,
	NULL,
};

/*
 * The repeat count after a short verdict on a thin frame: a verdict on the last
 * tail frame leaves the count running. A verdict on burst frame 1 within 3 dB
 * of the bar and 12 dB of the floor then takes the short hangover. The burst
 * acts from the first pass frame after its hold.
 */
static void hold_repeat_test(unsigned rate)
{
	static struct hold_leg legs[1][MISS_PHASES];
	unsigned n = 0;
	unsigned k;

	hold_phases_run(rate, MISS_RES_DEEP_DB, MISS_QUIET_FRAMES,
			&hold_repeat_spec, 1u, legs);
	case_rate(rate, "lec double talk leaves the repeat count running "
		  "after a short verdict on a tail frame");
	hold_legs_print("pulse on burst frame 1 after a tail verdict", legs[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		const struct hold_leg *g = &legs[0][k];

		if (!g->lead_fired || !g->fired)
			continue;
		n++;
		CHECK_UINT(g->lead_set, TAIL_PAIR_SHORT_FRAMES);
		CHECK_UINT(g->set, TAIL_PAIR_SHORT_FRAMES);
		hold_resume_check(g);
	}
	printf("# %u of %u phases fire on both pulses\n", n, MISS_PHASES);
	CHECK(n >= HOLD_MIN_PHASES);
}

/*
 * The implied floor of the re-acquisition window. The line is the
 * line of floor_keep_test with the background rise_db up at the change. So
 * the rise run at the keep implies a floor over the tracked one. From the
 * keep the background falls back, and the far pauses read high_db and
 * IMPLIED_CREEP_DB over the line noise in turn. The creep readings keep the
 * floor in place. The result counts the engage-wait frames and the bursts
 * that follow a high pause, inside the window and in the IMPLIED_AFTER frames
 * after it.
 */
#define IMPLIED_CREEP_DB	1.0
#define IMPLIED_WINDOW		700u
#define IMPLIED_AFTER		400u
/* The wait of an engage run after a refused resume: 7 frames. */
#define IMPLIED_WAIT		7u

/* The pause frames of implied_run that carry the high level. */
enum implied_fill {
	/* The last two frames of a pause, after the reference window. */
	IMPLIED_FILL_LATE,
	/* The second-last frame of a pause only, a tail frame of the
	 * processor (LEC_NLP_TAIL_QUIET_FRAMES). */
	IMPLIED_FILL_TAIL,
};

/* A line of implied_run. The levels are in dB over the line noise. */
struct implied_line {
	double hiss_dbm0;
	double rise_db;
	double high_db;
	enum implied_fill fill;
};

struct implied_result {
	unsigned keep;
	unsigned waits_in;
	unsigned high_in;
	unsigned waits_out;
	unsigned high_out;
};

static void implied_run(unsigned rate, const struct implied_line *l,
			struct implied_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path, next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	uint32_t pause_state = 0x3c6ef372u;
	int16_t near[UDSP_FRAME_MAX];
	unsigned frame = rate / 100u;
	unsigned pause, f, pauses = 0, in_pause = 0;
	uint32_t waits = 0;
	bool was_burst = true, high = false;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms_info_get(ec, &info);
	pause = (info.taps + frame - 1u) / frame + 2u;
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1,
			  phase2_erl_db[0] + PHASE2_DELTA_ERL_DB,
			  FLOOR_KEEP_DELAY_MS);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		floor_keep_frame(ec, &path, state,
				 f % FLOOR_KEEP_PERIOD_FRAMES <
				 FLOOR_KEEP_PERIOD_FRAMES - pause,
				 l->hiss_dbm0, NULL);
	path_annex_d_init(&next, rate, phase2_models[0], phase2_erl_db[0],
			  FLOOR_KEEP_DELAY_MS);
	path_history_carry(&next, &path);
	for (f = 0; f < 14u * FRAMES_PER_SECOND; f++) {
		unsigned g = PHASE2_CONVERGE_FRAMES + f;
		bool burst = g % FLOOR_KEEP_PERIOD_FRAMES <
			     FLOOR_KEEP_PERIOD_FRAMES - pause;
		bool after = r->keep && f >= r->keep;
		bool late, content;

		if (!burst && was_burst)
			pauses++;
		if (burst && !was_burst && after && high) {
			if (f < r->keep + IMPLIED_WINDOW)
				r->high_in++;
			else if (f < r->keep + IMPLIED_WINDOW + IMPLIED_AFTER)
				r->high_out++;
		}
		was_burst = burst;
		in_pause = burst ? 0u : in_pause + 1u;
		if (!burst)
			high = pauses % 2u;
		late = l->fill == IMPLIED_FILL_LATE ? in_pause + 2u > pause :
						      in_pause + 1u == pause;
		content = after && !burst && (!high || late);
		if (content)
			noise_fill(near, frame, l->hiss_dbm0 +
				   (high ? l->high_db : IMPLIED_CREEP_DB),
				   &pause_state);
		floor_keep_frame(ec, &next, state, burst,
				 l->hiss_dbm0 + (after ? 0.0 : l->rise_db),
				 content ? near : NULL);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (!r->keep &&
		    reading.trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT)
			r->keep = f + 1u;
		if (r->keep && f >= r->keep) {
			if (f < r->keep + IMPLIED_WINDOW)
				r->waits_in += info.nlp_wait_frames - waits;
			else if (f < r->keep + IMPLIED_WINDOW + IMPLIED_AFTER)
				r->waits_out += info.nlp_wait_frames - waits;
		}
		waits = info.nlp_wait_frames;
	}
	udsp_ec_nlms.destroy(ec);
}

static void implied_print(unsigned rate, const char *what,
			  const struct implied_result *r)
{
	printf("# %u Hz, %s: keep on frame %u after the change; engage-wait "
	       "frames %u in the window after %u bursts that follow a high "
	       "pause, %u after it after %u such bursts\n", rate, what,
	       r->keep, r->waits_in, r->high_in, r->waits_out, r->high_out);
}

/* The lines of the implied floor cases (implied_run). */
static const struct implied_line implied_inside = {
	PHASE2_HISS_DBM0, 20.0, 16.0, IMPLIED_FILL_LATE
};

static const struct implied_line implied_cap = {
	PHASE2_HISS_DBM0, 30.0, 28.0, IMPLIED_FILL_LATE
};

/*
 * -57 dBm0: a loud line noise. The first quiet frame after the change still
 * carries echo, and over this noise it stays within the spread bound of the
 * rise run. A rise of 6 dB then implies a floor about 6 dB over the tracked
 * one, under the cap. Its peak bound stands about 15 dB over the line noise, and
 * the peak bound of a floor at the cap 21 dB.
 */
#define IMPLIED_LOUD_HISS_DBM0	-57.0

static const struct implied_line implied_under = {
	IMPLIED_LOUD_HISS_DBM0, 6.0, 11.0, IMPLIED_FILL_LATE
};

static const struct implied_line implied_over = {
	IMPLIED_LOUD_HISS_DBM0, 6.0, 17.0, IMPLIED_FILL_LATE
};

/* 22 dB: over the peak bound of the capped floor and under its tail
 * threshold (LEC_NLP_TAIL_FLOOR_RATIO). */
static const struct implied_line implied_tail = {
	PHASE2_HISS_DBM0, 20.0, 22.0, IMPLIED_FILL_TAIL
};

/*
 * The background 20 dB up at the change; pauses at 16 dB over the line noise
 * in their last two frames. The peak bound reads the implied floor, so no
 * burst after such a pause waits more than one engage run in the window.
 */
static void implied_inside_test(unsigned rate)
{
	struct implied_result r;

	implied_run(rate, &implied_inside, &r);
	case_rate(rate, "lec reads the implied floor in the window after a "
		  "kept echo path change");
	implied_print(rate, "inside the window", &r);
	CHECK(r.keep > 0u);
	CHECK(r.high_in > 0u);
	CHECK(r.waits_in <= IMPLIED_WAIT);
}

/*
 * The line of implied_inside_test after the window. The implied floor ends
 * with the window, so all but one burst after a high pause waits its engage
 * run again.
 */
static void implied_outside_test(unsigned rate)
{
	struct implied_result r;

	implied_run(rate, &implied_inside, &r);
	case_rate(rate, "lec reads no implied floor after the window");
	implied_print(rate, "after the window", &r);
	CHECK(r.high_out > 1u);
	CHECK(r.waits_out >= IMPLIED_WAIT * (r.high_out - 1u));
}

/*
 * The background 30 dB up at the change, so the rise run implies a floor over
 * the cap (LEC_NOISE_IMPLIED_CAP_DB, 12 dB). The pauses read 28 dB over the
 * line noise in their last two frames. They stand over the peak bound of the
 * capped floor, so all but one burst after them waits its engage run.
 */
static void implied_cap_test(unsigned rate)
{
	struct implied_result r;

	implied_run(rate, &implied_cap, &r);
	case_rate(rate, "lec caps the implied floor 12 dB over the tracked "
		  "floor");
	implied_print(rate, "over the cap", &r);
	CHECK(r.high_in > 1u);
	CHECK(r.waits_in >= IMPLIED_WAIT * (r.high_in - 1u));
}

/*
 * The line of keep_renewal_test: the line of implied_run with the background
 * RENEW_RISE_DB up at the change, so the first keep implies a floor. From the
 * first keep the background falls back, and the far pauses read
 * RENEW_LOW_DB and RENEW_HIGH_DB over the line noise in turn. So the next
 * rise run spreads over its bound, and no agreement run completes. A second
 * change, back to m1, RENEW_SECOND frames after the first keep ends in a
 * second keep in the window. After it the pauses read RENEW_MID_DB and
 * IMPLIED_CREEP_DB in turn.
 */
#define RENEW_RISE_DB		20.0
#define RENEW_LOW_DB		4.0
#define RENEW_HIGH_DB		14.0
#define RENEW_MID_DB		10.0
#define RENEW_SECOND		100u

struct renew_result {
	unsigned keep1;
	unsigned keep2;
	unsigned waits_first;
	unsigned waits_late;
	unsigned high_late;
};

static void renew_run(unsigned rate, struct renew_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path, next, back;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	uint32_t pause_state = 0x3c6ef372u;
	int16_t near[UDSP_FRAME_MAX];
	unsigned frame = rate / 100u;
	unsigned pause, f, pauses = 0;
	uint32_t waits = 0;
	bool second = false, was_burst = true, high = false;

	memset(r, 0, sizeof(*r));
	udsp_ec_nlms_info_get(ec, &info);
	pause = (info.taps + frame - 1u) / frame + 2u;
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1,
			  phase2_erl_db[0] + PHASE2_DELTA_ERL_DB,
			  FLOOR_KEEP_DELAY_MS);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES; f++)
		floor_keep_frame(ec, &path, state,
				 f % FLOOR_KEEP_PERIOD_FRAMES <
				 FLOOR_KEEP_PERIOD_FRAMES - pause,
				 PHASE2_HISS_DBM0, NULL);
	path_annex_d_init(&next, rate, phase2_models[0], phase2_erl_db[0],
			  FLOOR_KEEP_DELAY_MS);
	path_history_carry(&next, &path);
	for (f = 0; f < 16u * FRAMES_PER_SECOND; f++) {
		unsigned g = PHASE2_CONVERGE_FRAMES + f;
		bool burst = g % FLOOR_KEEP_PERIOD_FRAMES <
			     FLOOR_KEEP_PERIOD_FRAMES - pause;
		bool after = r->keep1 && f >= r->keep1;
		struct echo_path *p = second ? &back : &next;
		double level;

		if (!burst && was_burst)
			pauses++;
		if (burst && !was_burst && high && r->keep2 &&
		    f >= r->keep1 + IMPLIED_WINDOW &&
		    f < r->keep2 + IMPLIED_WINDOW)
			r->high_late++;
		was_burst = burst;
		if (!burst)
			high = pauses % 2u;
		if (r->keep1 && !second && f == r->keep1 + RENEW_SECOND) {
			path_annex_d_init(&back, rate, ANNEX_D_M1,
					  phase2_erl_db[0] +
					  PHASE2_DELTA_ERL_DB,
					  FLOOR_KEEP_DELAY_MS);
			path_history_carry(&back, &next);
			second = true;
			p = &back;
		}
		if (r->keep2)
			level = high ? RENEW_MID_DB : IMPLIED_CREEP_DB;
		else
			level = high ? RENEW_HIGH_DB : RENEW_LOW_DB;
		if (after && !burst)
			noise_fill(near, frame, PHASE2_HISS_DBM0 + level,
				   &pause_state);
		floor_keep_frame(ec, p, state, burst,
				 PHASE2_HISS_DBM0 +
				 (after ? 0.0 : RENEW_RISE_DB),
				 after && !burst ? near : NULL);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (reading.trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT) {
			if (!r->keep1)
				r->keep1 = f + 1u;
			else if (second && !r->keep2)
				r->keep2 = f + 1u;
		}
		if (r->keep1 && f >= r->keep1 && f < r->keep1 + IMPLIED_WINDOW)
			r->waits_first += info.nlp_wait_frames - waits;
		if (r->keep2 && f >= r->keep1 + IMPLIED_WINDOW &&
		    f < r->keep2 + IMPLIED_WINDOW)
			r->waits_late += info.nlp_wait_frames - waits;
		waits = info.nlp_wait_frames;
	}
	udsp_ec_nlms.destroy(ec);
}

/*
 * A second keep in the re-acquisition window whose rise run implies no floor.
 * After the second keep the bursts act, and a pause at
 * RENEW_MID_DB stands over the peak bound of the floor. From the end of the
 * first window to the end of the second, each burst after such a pause must
 * wait its engage run. The implied floor of the first keep must not carry
 * over.
 */
static void keep_renewal_test(unsigned rate)
{
	struct renew_result r;

	renew_run(rate, &r);
	case_rate(rate, "lec carries no implied floor of an earlier keep past "
		  "a keep whose run implies none");
	printf("# %u Hz: keeps on frames %u and %u after the change; "
	       "engage-wait frames %u in the first window, %u from its end to "
	       "the end of the second after %u bursts that follow a loud "
	       "pause\n", rate, r.keep1, r.keep2, r.waits_first,
	       r.waits_late, r.high_late);
	CHECK(r.keep1 > 0u);
	CHECK(r.keep2 > r.keep1 && r.keep2 < r.keep1 + IMPLIED_WINDOW);
	CHECK(r.high_late > 0u);
	CHECK(r.waits_late >= IMPLIED_WAIT * r.high_late);
}

/*
 * The line of IMPLIED_LOUD_HISS_DBM0 with the background 6 dB up at the
 * change, so the rise run implies a floor under the cap. Pauses at 11 dB stand
 * under the peak bound of that floor, so no burst after them waits more than
 * one engage run. Pauses at 17 dB stand over it and under the peak bound of a
 * floor at the cap. So all but one burst after them waits its engage run.
 */
static void implied_uncapped_test(unsigned rate)
{
	struct implied_result under, over;

	implied_run(rate, &implied_under, &under);
	implied_run(rate, &implied_over, &over);
	case_rate(rate, "lec takes the implied floor from the mean of the rise "
		  "run under the cap");
	implied_print(rate, "under the bound of the run", &under);
	implied_print(rate, "over the bound of the run", &over);
	CHECK(under.keep > 0u && under.high_in > 1u);
	CHECK(under.waits_in <= IMPLIED_WAIT);
	CHECK(over.high_in > 1u);
	CHECK(over.waits_in >= IMPLIED_WAIT * (over.high_in - 1u));
}

/*
 * The line of implied_inside_test with the high level at 22 dB in the
 * second-last frame of each pause only, a tail frame. It stands over the peak
 * bound of the implied floor and under its tail threshold. So no burst after
 * such a pause waits more than one engage run in the window.
 */
static void implied_tail_test(unsigned rate)
{
	struct implied_result r;

	implied_run(rate, &implied_tail, &r);
	case_rate(rate, "lec reads the implied floor in the tail threshold");
	implied_print(rate, "tail frame", &r);
	CHECK(r.keep > 0u && r.high_in > 1u);
	CHECK(r.waits_in <= IMPLIED_WAIT);
}

/*
 * The cross-fade at the start of an act. The line is the line of
 * floor_keep_test on a path whose echo arrives delay_ms after the far end.
 * The filter converges on that path plus an early copy of it, early_ms after
 * the far end and early_db under it. Then the copy leaves the path. So the
 * estimate leads the echo on the first frame of a burst. With early_db 0 the
 * path carries no copy. With late, the path carries a late copy at the end
 * of the filter span until the change. The result reads each frame that
 * starts or ends an act in the FADE_FRAMES after the change, as its output
 * over its input. It keeps the most over the act starts whose estimate
 * stands 3 dB over the input, and over the other act starts. It keeps the
 * least over the act ends whose estimate stands 3 dB over the input.
 */
/* The lead line: the first frame of a burst carries the estimate of the copy
 * and no echo yet. */
#define FADE_DELAY_MS		12u
#define FADE_EARLY_MS		4u
#define FADE_PLAIN_MS		4u
#define FADE_LATE_BACK_MS	2u
#define FADE_LATE_DB		-20.0
/* 1 dB: an act end fades to the filter output, which stands over the input
 * where the estimate outlasts the echo. */
#define FADE_LAG_OVER_DB	1.0
#define FADE_EARLY_DB		-30.0
#define FADE_FRAMES		(4u * FRAMES_PER_SECOND)
/* -20 dB: an ordinary act start fades from the filter output, far under an
 * input that carries the echo. */
#define FADE_PLAIN_UNDER_DB	-20.0
/* The weight line: the echo arrives in the last 2 ms of the first frame of a
 * burst, and the copy 5 ms before it. */
#define FADE_WEIGHT_DELAY_MS	8u
#define FADE_WEIGHT_EARLY_MS	3u
#define FADE_WEIGHT_DB		-10.0
/* -20 dB: the fade from the input keeps little of an echo at the end of the
 * frame. A fade from the filter output keeps the estimate of the copy, about
 * 10 dB under the input. */
#define FADE_WEIGHT_UNDER_DB	-20.0

struct fade_result {
	unsigned lead;
	unsigned plain;
	unsigned lag;
	double lead_over_db;
	double plain_over_db;
	double lag_over_db;
};

/* One frame of the fade line: the far end through path, the line noise, and
 * the input and output powers of the frame. */
static void fade_frame(struct udsp_ec *ec, struct echo_path *path,
			uint32_t state[2], bool burst, double *in_power,
			double *out_power)
{
	int16_t far[UDSP_FRAME_MAX];
	int16_t ref[UDSP_FRAME_MAX];
	int16_t echo[UDSP_FRAME_MAX];
	int16_t hiss[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned i;

	if (burst)
		noise_fill(far, path->frame, PHASE2_FAR_DBM0, &state[0]);
	else
		memset(far, 0, sizeof(far));
	level_loss_apply(ref, far, path->frame, PHASE2_LOSS_HALF_DB);
	path_run(path, far, echo);
	noise_fill(hiss, path->frame, PHASE2_HISS_DBM0, &state[1]);
	*in_power = 0.0;
	*out_power = 0.0;
	for (i = 0; i < path->frame; i++) {
		mic[i] = sample_clamp((double)echo[i] + hiss[i]);
		*in_power += (double)mic[i] * mic[i];
	}
	udsp_ec_nlms.run(ec, mic, ref);
	for (i = 0; i < path->frame; i++)
		*out_power += (double)mic[i] * mic[i];
}

/* The path with a late copy of itself, FADE_LATE_DB under it and shift
 * samples after it. The path grows by shift taps. */
static void fade_late_add(struct echo_path *path, unsigned shift)
{
	double g = pow(10.0, FADE_LATE_DB / 20.0);
	unsigned taps = path->taps;
	unsigned k;

	if (taps + shift > LEC_PATH_TAPS_MAX)
		return;
	for (k = taps; k < taps + shift; k++)
		path->h[k] = 0.0f;
	for (k = taps + shift; k-- > shift;)
		path->h[k] += (float)(g * path->h[k - shift]);
	path->taps = taps + shift;
}

/* The path with an early copy of itself: each tap takes the tap shift
 * samples later, early_db under it. */
static void fade_copy_add(struct echo_path *path, unsigned shift,
			   double early_db)
{
	double g = pow(10.0, early_db / 20.0);
	unsigned k;

	for (k = 0; k + shift < path->taps; k++)
		path->h[k] += (float)(g * path->h[k + shift]);
}

static void fade_run(unsigned rate, unsigned delay_ms, unsigned early_ms,
		     double early_db, bool late, struct fade_result *r)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct echo_path path, next;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du, 0x51ab0dd5u };
	unsigned frame = rate / 100u;
	unsigned pause, f;
	bool was_active = false;
	double in, out;

	memset(r, 0, sizeof(*r));
	r->lead_over_db = -99.0;
	r->plain_over_db = -99.0;
	r->lag_over_db = 99.0;
	udsp_ec_nlms_info_get(ec, &info);
	pause = (info.taps + frame - 1u) / frame + 2u;
	udsp_ec_nlms_ref_loss_set(ec, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(ec, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1, phase2_erl_db[0], delay_ms);
	next = path;
	if (early_db < 0.0)
		fade_copy_add(&path, (delay_ms - early_ms) * rate / 1000u,
			      early_db);
	if (late)
		fade_late_add(&path, info.taps - FADE_LATE_BACK_MS * rate /
				      1000u - delay_ms * rate / 1000u);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES + FADE_FRAMES; f++) {
		bool burst = f % FLOOR_KEEP_PERIOD_FRAMES <
			     FLOOR_KEEP_PERIOD_FRAMES - pause;
		bool after = f >= PHASE2_CONVERGE_FRAMES;
		double over;

		if (f == PHASE2_CONVERGE_FRAMES)
			path_history_carry(&next, &path);
		fade_frame(ec, after ? &next : &path, state, burst, &in, &out);
		udsp_ec_nlms_info_get(ec, &info);
		udsp_ec_nlms_frame_get(ec, &reading);
		if (after && !info.nlp_active && was_active && in > 0.0 &&
		    reading.estimate_power > 2.0f * reading.mic_power) {
			r->lag++;
			r->lag_over_db = fmin(r->lag_over_db,
					      10.0 * log10(out / in));
		}
		if (after && info.nlp_active && !was_active && in > 0.0) {
			over = 10.0 * log10(out / in);
			if (reading.estimate_power > 2.0f * reading.mic_power) {
				r->lead++;
				r->lead_over_db = fmax(r->lead_over_db, over);
			} else {
				r->plain++;
				r->plain_over_db = fmax(r->plain_over_db, over);
			}
		}
		was_active = info.nlp_active;
	}
	udsp_ec_nlms.destroy(ec);
}

static void fade_print(unsigned rate, const char *what,
			const struct fade_result *r)
{
	printf("# %u Hz, %s: %u act starts with a leading estimate, output "
	       "at most %+.1f dB over the input; %u other act starts, at most "
	       "%+.1f dB; %u act ends with a lagging estimate, at least "
	       "%+.1f dB\n", rate, what, r->lead, r->lead_over_db, r->plain,
	       r->plain_over_db, r->lag, r->lag_over_db);
}

/*
 * The lead line of fade_run, with the early copy FADE_EARLY_DB under the
 * path. On each frame that starts an act with a leading estimate, the
 * cross-fade starts from the input. So the output stays under the input.
 */
static void fade_lead_test(unsigned rate)
{
	struct fade_result r;

	fade_run(rate, FADE_DELAY_MS, FADE_EARLY_MS, FADE_EARLY_DB, false, &r);
	case_rate(rate, "lec fades from the input when the estimate leads the "
		  "echo at the start of an act");
	fade_print(rate, "leading estimate", &r);
	CHECK(r.lead >= 2u);
	CHECK(r.lead_over_db <= 0.0);
}

/*
 * The line of fade_run with no early copy: the echo arrives on the first
 * frame of each burst (FADE_PLAIN_MS). The cross-fade starts from the filter
 * output, so a frame that starts an act stays FADE_PLAIN_UNDER_DB under its
 * input.
 */
static void fade_plain_test(unsigned rate)
{
	struct fade_result r;

	fade_run(rate, FADE_PLAIN_MS, 0u, 0.0, false, &r);
	case_rate(rate, "lec fades from the filter output at an ordinary start "
		  "of an act");
	fade_print(rate, "ordinary onset", &r);
	CHECK(r.lead == 0u);
	CHECK(r.plain >= 4u);
	CHECK(r.plain_over_db <= FADE_PLAIN_UNDER_DB);
}

/*
 * The line of fade_run with a late copy, FADE_LATE_DB under the path, that
 * leaves the path. On the first quiet frame after a burst the estimate
 * outlasts the echo, and the act ends there. That fade ends at the filter
 * output, which the next frame carries, so the output stands over the input.
 */
static void fade_tail_test(unsigned rate)
{
	struct fade_result r;

	fade_run(rate, FADE_PLAIN_MS, 0u, 0.0, true, &r);
	case_rate(rate, "lec fades to the filter output at the end of an act");
	fade_print(rate, "lagging estimate", &r);
	CHECK(r.lag >= 1u);
	CHECK(r.lag_over_db >= FADE_LAG_OVER_DB);
}

/*
 * The weight line of fade_run. On the first frame of a burst, the input
 * carries more power than the filter output, but late in the frame. The
 * fade keeps little of the source there. The filter output carries the
 * estimate of the copy early in the frame, where the fade keeps most of the
 * source. So the fade from the input is the quieter one, and the frame stays
 * FADE_WEIGHT_UNDER_DB under its input.
 */
static void fade_weight_test(unsigned rate)
{
	struct fade_result r;

	fade_run(rate, FADE_WEIGHT_DELAY_MS, FADE_WEIGHT_EARLY_MS,
		 FADE_WEIGHT_DB, false, &r);
	case_rate(rate, "lec fades from the input when the echo arrives late in "
		  "the frame");
	fade_print(rate, "echo late in the frame", &r);
	CHECK(r.lead + r.plain >= 4u);
	CHECK(fmax(r.lead_over_db, r.plain_over_db) <= FADE_WEIGHT_UNDER_DB);
}

/*
 * The emit line: a line of fade_run on twin instances, with its own line
 * noise and seeds. The second twin runs with the processor off from the
 * change. So its output is the filter output of the first, and its estimate
 * reads the same on every frame. On each act start of the first twin, its
 * reading names the source of the fade (fade_input). The output less the
 * faded source is the noise term of the fade. With that term the line forms
 * the other fade. The result keeps the most that a frame emits over the
 * other fade and the rounding of fade_emit_read.
 */
/* The line noise of the cross-term lines: loud enough that the rounding to
 * 16 bits does not hide the term of the comfort noise. */
#define FADE_CROSS_HISS_DBM0	-45.0

/*
 * The cross-term lines: the echo arrives delay_ms after the far end, and a
 * copy early_db under it at early_ms, with the seeds of seed. On some act
 * starts of these lines, the two sources carry about the same energy under
 * the fade. There the noise term decides which fade is quieter.
 */
static const struct fade_cross_line {
	unsigned delay_ms;
	unsigned early_ms;
	double early_db;
	unsigned seed;
} fade_cross_lines[] = {
	{ 8u, 3u, -12.0, 0u },
	{ 9u, 3u, -20.0, 1u },
};

struct fade_emit_result {
	unsigned starts;
	unsigned apart;
	double over_db;
};

/* One act start of the emit line: the input (in), the second twin's filter
 * output (res) and the first twin's frame (out). The fade of out started
 * from the input when from_in is set. */
static void fade_emit_read(const int16_t *in, const int16_t *res,
			   const int16_t *out, unsigned frame, bool from_in,
			   struct fade_emit_result *r)
{
	double emit = 0.0, other = 0.0, slack = 0.0;
	unsigned i;

	r->starts++;
	for (i = 0; i < frame; i++) {
		double keep = 1.0 - (double)(i + 1u) / frame;
		double noise = out[i] - keep * (from_in ? in[i] : res[i]);
		double alt = keep * (from_in ? res[i] : in[i]) + noise;

		emit += (double)out[i] * out[i];
		other += alt * alt;
		slack += fabs(out[i] - alt);
	}
	/*
	 * The output rounds its fade to 16 bits, and the other fade here carries
	 * the same rounding. So the frame can stand the sum of |out - alt| over
	 * the other fade. The float sums of the canceller add 1e-6 of it.
	 */
	slack += 1e-6 * other;
	if (emit > 0.0)
		r->over_db = fmax(r->over_db,
				  10.0 * log10(emit / (other + slack)));
}

static void fade_emit_run(unsigned rate, unsigned delay_ms, unsigned early_ms,
			  double early_db, double hiss_dbm0, unsigned seed,
			  struct fade_emit_result *r)
{
	struct udsp_ec_nlms_frame ra, rb;
	struct udsp_ec_nlms_info info;
	struct echo_path path, next;
	struct udsp_ec *a = udsp_ec_nlms.create_rate(0, rate);
	struct udsp_ec *b = udsp_ec_nlms.create_rate(0, rate);
	uint32_t state[2] = { 0x2a5e1c0du ^ (seed * 0x9e3779b9u),
			      0x51ab0dd5u ^ (seed * 0x7f4a7c15u) };
	unsigned frame = rate / 100u;
	unsigned pause, f, i;
	bool was_active = false;

	udsp_ec_nlms_info_get(a, &info);
	pause = (info.taps + frame - 1u) / frame + 2u;
	udsp_ec_nlms_ref_loss_set(a, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms_ref_loss_set(b, PHASE2_LOSS_HALF_DB);
	udsp_ec_nlms.nlp(a, true);
	udsp_ec_nlms.nlp(b, true);
	path_annex_d_init(&path, rate, ANNEX_D_M1, phase2_erl_db[0], delay_ms);
	next = path;
	fade_copy_add(&path, (delay_ms - early_ms) * rate / 1000u, early_db);
	for (f = 0; f < PHASE2_CONVERGE_FRAMES + FADE_FRAMES; f++) {
		int16_t far[UDSP_FRAME_MAX];
		int16_t ref[UDSP_FRAME_MAX];
		int16_t echo[UDSP_FRAME_MAX];
		int16_t hiss[UDSP_FRAME_MAX];
		int16_t in[UDSP_FRAME_MAX];
		int16_t out[UDSP_FRAME_MAX];
		int16_t res[UDSP_FRAME_MAX];
		bool burst = f % FLOOR_KEEP_PERIOD_FRAMES <
			     FLOOR_KEEP_PERIOD_FRAMES - pause;
		bool after = f >= PHASE2_CONVERGE_FRAMES;

		if (f == PHASE2_CONVERGE_FRAMES) {
			path_history_carry(&next, &path);
			udsp_ec_nlms.nlp(b, false);
		}
		if (burst)
			noise_fill(far, frame, PHASE2_FAR_DBM0, &state[0]);
		else
			memset(far, 0, sizeof(far));
		level_loss_apply(ref, far, frame, PHASE2_LOSS_HALF_DB);
		path_run(after ? &next : &path, far, echo);
		noise_fill(hiss, frame, hiss_dbm0, &state[1]);
		for (i = 0; i < frame; i++)
			in[i] = sample_clamp((double)echo[i] + hiss[i]);
		memcpy(out, in, sizeof(in));
		memcpy(res, in, sizeof(in));
		udsp_ec_nlms.run(a, out, ref);
		udsp_ec_nlms.run(b, res, ref);
		udsp_ec_nlms_frame_get(a, &ra);
		udsp_ec_nlms_frame_get(b, &rb);
		udsp_ec_nlms_info_get(a, &info);
		if (after && ra.estimate_power != rb.estimate_power)
			r->apart++;
		if (after && info.nlp_active && !was_active)
			fade_emit_read(in, res, out, frame, ra.fade_input, r);
		was_active = info.nlp_active;
	}
	udsp_ec_nlms.destroy(a);
	udsp_ec_nlms.destroy(b);
}

/*
 * The emit lines: the lead line and the weight line of fade_run, and the
 * cross-term lines. On each act start, the frame emits no more energy than
 * the other fade, up to the rounding to 16 bits.
 */
static void fade_emit_test(unsigned rate)
{
	struct fade_emit_result r;
	unsigned l;

	memset(&r, 0, sizeof(r));
	r.over_db = -99.0;
	fade_emit_run(rate, FADE_DELAY_MS, FADE_EARLY_MS, FADE_EARLY_DB,
		      PHASE2_HISS_DBM0, 0u, &r);
	fade_emit_run(rate, FADE_WEIGHT_DELAY_MS, FADE_WEIGHT_EARLY_MS,
		      FADE_WEIGHT_DB, PHASE2_HISS_DBM0, 0u, &r);
	for (l = 0; l < sizeof(fade_cross_lines) / sizeof(fade_cross_lines[0]);
	     l++)
		fade_emit_run(rate, fade_cross_lines[l].delay_ms,
			      fade_cross_lines[l].early_ms,
			      fade_cross_lines[l].early_db, FADE_CROSS_HISS_DBM0,
			      fade_cross_lines[l].seed, &r);
	case_rate(rate, "lec emits the quieter of the two fades at the start of "
		  "an act");
	printf("# %u Hz: %u act starts; output at most %+.3f dB over the other "
	       "fade and its rounding; estimates apart on %u frames\n", rate,
	       r.starts, r.over_db, r.apart);
	CHECK(r.apart == 0u);
	CHECK(r.starts >= 30u);
	CHECK(r.over_db <= 0.0);
}

/*
 * The resume on a tracker between 22 and 24 dB (LEC_NLP_RESUME_MARGIN_DB of
 * dsp/lec.c). Down to RESUME_MARGIN_DB, the first pass frame after a far pause
 * resumes the run on one of two forms of evidence:
 * - climb: the tracker reads over its reading RESUME_CLIMB_FRAMES frames
 *   before;
 * - dip: the burst before the pause reached MISS_RESUME_DB.
 * A dt, unlike or erle frame that cut the run since the run was last complete
 * refuses both.
 */
#define RESUME_CLIMB_FRAMES	100u
#define RESUME_MARGIN_DB	22.0
/* The echo products of the lines. After MISS_RES_LOW_DB, RESUME_CLIMB_RES_DB
 * takes the tracker from 20.4 dB up to 22.5 to 22.9 dB. After
 * MISS_RES_DEEP_DB, RESUME_DIP_RES_DB takes it from 29.3 dB down to 23.1 to
 * 23.3 dB in one burst. The two low products leave each form within
 * RESUME_NEAR_DB under RESUME_MARGIN_DB. */
#define RESUME_CLIMB_RES_DB	23.4
#define RESUME_DIP_RES_DB	23.2
#define RESUME_LOW_CLIMB_RES_DB	22.2
#define RESUME_LOW_DIP_RES_DB	21.8
#define RESUME_NEAR_DB		0.5
/* The near sound of resume_cut_test on burst frames 2 to 4. Then the pulse
 * over the tracked floor on the first frame of a burst, which takes the short
 * hangover on the line of RESUME_CLIMB_RES_DB. */
#define RESUME_TALK_FROM	2u
#define RESUME_TALK_TO		5u
#define RESUME_PULSE_DB		11.5

/*
 * The tracker readings that the resume reads, as the test keeps them from the
 * readout of each frame: the last RESUME_CLIMB_FRAMES readings with pos on
 * the oldest, the largest reading of the running burst, and that of the
 * burst before the latest pause. Each frame that is not quiet is a burst
 * frame.
 */
struct resume_trace {
	double ring[RESUME_CLIMB_FRAMES];
	unsigned pos;
	double burst_max;
	double pause_max;
	bool in_pause;
};

static void resume_trace_add(struct resume_trace *tr, unsigned verdict,
			     double erle_db)
{
	tr->ring[tr->pos] = erle_db;
	tr->pos = (tr->pos + 1u) % RESUME_CLIMB_FRAMES;
	if (verdict == LECR_VERDICT_QUIET) {
		if (!tr->in_pause)
			tr->pause_max = tr->burst_max;
		tr->in_pause = true;
		return;
	}
	if (tr->in_pause || erle_db > tr->burst_max)
		tr->burst_max = erle_db;
	tr->in_pause = false;
}

/* One frame of the line (miss_frame_run) into the trace. */
static void resume_frame_run(struct miss_line *l, double over_db,
			     struct resume_trace *tr,
			     struct udsp_ec_nlms_frame *reading,
			     struct udsp_ec_nlms_info *info)
{
	miss_frame_run(l, over_db, reading, info);
	resume_trace_add(tr, reading->nlp_verdict, info->erle_db);
}

/* The quiet-miss line at phase k of the rate, with its echo product res_db
 * under the echo, settled as miss_line_settle settles it. The trace starts
 * with the line. */
static void resume_line_init(struct miss_line *l, struct resume_trace *tr,
			     unsigned rate, unsigned k, double res_db)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;

	miss_line_init(l, rate, k * (rate / 100u) / MISS_PHASES, res_db);
	memset(tr, 0, sizeof(*tr));
	while (l->count < MISS_SETTLE_FRAMES ||
	       miss_phase_get(l) != MISS_BURST_FRAMES)
		resume_frame_run(l, 0.0, tr, &reading, &info);
}

/*
 * The figures of a pause and the burst after it: the quiet frames of the
 * pause and the largest reading of the burst before it. For each burst frame:
 * the verdict, the act, the tracker after the frame and the reading
 * RESUME_CLIMB_FRAMES frames before the frame. For the first burst frame:
 * whether the detector fired, its microphone over the tracked floor, and the
 * hangover that it set (hold_left of the frame after it).
 */
struct resume_leg {
	unsigned quiet;
	double pause_max;
	unsigned verdict[MISS_BURST_FRAMES];
	bool active[MISS_BURST_FRAMES];
	double erle_db[MISS_BURST_FRAMES];
	double ago_db[MISS_BURST_FRAMES];
	bool fired;
	unsigned set;
	double over_floor;
};

/* The pause of the period from the frame the line stands on, with pattern on
 * its quiet frames, then the burst of the next period. burst_db, when set,
 * holds the level of each burst frame over the tracked floor. */
static void resume_leg_run(struct miss_line *l, struct resume_trace *tr,
			   const char *pattern, const double *burst_db,
			   struct resume_leg *g)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned from = l->period - l->quiet;
	unsigned f;

	memset(g, 0, sizeof(*g));
	for (f = miss_phase_get(l); f < l->period; f++) {
		char c = f >= from ? pattern[f - from] : '-';

		resume_frame_run(l, miss_level_get(c), tr, &reading, &info);
		g->quiet += reading.nlp_verdict == LECR_VERDICT_QUIET;
	}
	g->pause_max = tr->pause_max;
	for (f = 0; f < MISS_BURST_FRAMES; f++) {
		g->ago_db[f] = tr->ring[tr->pos];
		resume_frame_run(l, burst_db ? burst_db[f] : 0.0, tr, &reading,
				 &info);
		g->verdict[f] = reading.nlp_verdict;
		g->active[f] = info.nlp_active;
		g->erle_db[f] = info.erle_db;
		if (f == 0u) {
			g->fired = reading.dt_fired;
			g->over_floor = ratio_db_get(reading.mic_power,
						     reading.noise_power);
		}
		if (f == 1u)
			g->set = reading.hold_left;
	}
}

/* The first acting frame of a leg from frame from on, MISS_BURST_FRAMES for
 * none, and the pass frames before it that waited. */
static unsigned resume_first_get(const struct resume_leg *g, unsigned from,
				 unsigned *waits)
{
	unsigned f;

	*waits = 0;
	for (f = from; f < MISS_BURST_FRAMES && !g->active[f]; f++)
		*waits += g->verdict[f] == LECR_VERDICT_PASS;
	return f;
}

/* The ranges of the figures of one leg over the phases, from the frame at. */
struct resume_range {
	double erle_lo;
	double erle_hi;
	double ago_lo;
	double ago_hi;
	double max_lo;
	double max_hi;
	unsigned first;
	unsigned waits;
};

static void resume_range_init(struct resume_range *r)
{
	r->erle_lo = r->ago_lo = r->max_lo = 200.0;
	r->erle_hi = r->ago_hi = r->max_hi = -200.0;
	r->first = 0;
	r->waits = 0;
}

static void resume_range_add(struct resume_range *r,
			     const struct resume_leg *g, unsigned at)
{
	unsigned waits;
	unsigned first = resume_first_get(g, at, &waits);

	r->erle_lo = fmin(r->erle_lo, g->erle_db[at]);
	r->erle_hi = fmax(r->erle_hi, g->erle_db[at]);
	r->ago_lo = fmin(r->ago_lo, g->ago_db[at]);
	r->ago_hi = fmax(r->ago_hi, g->ago_db[at]);
	r->max_lo = fmin(r->max_lo, g->pause_max);
	r->max_hi = fmax(r->max_hi, g->pause_max);
	r->first = first - at > r->first ? first - at : r->first;
	r->waits = waits > r->waits ? waits : r->waits;
}

static void resume_range_print(const char *what, const struct resume_range *r)
{
	printf("# %s: tracker %.2f to %.2f dB, %u frames before %.2f to %.2f "
	       "dB; burst before the pause up to %.2f to %.2f dB; acts %u "
	       "frames later at most, %u wait frames at most\n", what,
	       r->erle_lo, r->erle_hi, RESUME_CLIMB_FRAMES, r->ago_lo,
	       r->ago_hi, r->max_lo, r->max_hi, r->first, r->waits);
}

/*
 * The checks of a leg whose onset is a pass frame after a pause of quiet
 * frames, with the tracker under MISS_RESUME_DB: whether it climbs, whether
 * the burst before reached MISS_RESUME_DB, and whether the burst acts on its
 * first frame or waits 7 frames.
 */
static void resume_onset_check(const struct resume_leg *g, bool climb,
			       bool dip, bool resumes)
{
	unsigned waits;
	unsigned first = resume_first_get(g, 0u, &waits);

	CHECK_UINT(g->quiet, MISS_QUIET_FRAMES);
	CHECK_UINT(g->verdict[0], LECR_VERDICT_PASS);
	CHECK(g->erle_db[0] < MISS_RESUME_DB);
	CHECK((g->erle_db[0] > g->ago_db[0]) == climb);
	CHECK((g->pause_max >= MISS_RESUME_DB) == dip);
	CHECK_UINT(first, resumes ? 0u : WORDS_ENGAGE_FRAME);
	CHECK_UINT(waits, resumes ? 0u : WORDS_ENGAGE_FRAME);
}

/* The climb line at phase k: its echo product moves from MISS_RES_LOW_DB to
 * res_db after it settles. A pause of the line floor, then three misses. */
static void resume_climb_run(unsigned rate, unsigned k, double res_db,
			     struct resume_leg *g)
{
	struct resume_trace tr;
	struct resume_leg floor;
	struct miss_line l;

	resume_line_init(&l, &tr, rate, k, MISS_RES_LOW_DB);
	l.res_db = res_db;
	resume_leg_run(&l, &tr, "----", NULL, &floor);
	resume_leg_run(&l, &tr, "-mmm", NULL, g);
	udsp_ec_nlms.destroy(l.ec);
}

/* The dip line at phase k: its echo product moves from MISS_RES_DEEP_DB to
 * res_db after it settles. A pause of the line floor, then two of three
 * misses: the first after the burst whose tracker dips, the second after the
 * burst after it. */
static void resume_dip_run(unsigned rate, unsigned k, double res_db,
			   struct resume_leg g[2])
{
	struct resume_trace tr;
	struct resume_leg floor;
	struct miss_line l;

	resume_line_init(&l, &tr, rate, k, MISS_RES_DEEP_DB);
	l.res_db = res_db;
	resume_leg_run(&l, &tr, "----", NULL, &floor);
	resume_leg_run(&l, &tr, "-mmm", NULL, &g[0]);
	resume_leg_run(&l, &tr, "-mmm", NULL, &g[1]);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * Three quiet misses at 8 phases on a line whose tracker climbs under
 * MISS_RESUME_DB, from under RESUME_MARGIN_DB to over it. The burst before
 * the pause stays under MISS_RESUME_DB, so the climb alone resumes the run at
 * the far onset. A resume without the climb fails here.
 */
static void resume_climb_test(unsigned rate)
{
	struct resume_range r;
	unsigned k;

	resume_range_init(&r);
	case_rate(rate, "lec nlp resumes the engage run at a far onset after "
		  "three quiet misses on a climbing tracker under the resume "
		  "level");
	for (k = 0; k < MISS_PHASES; k++) {
		struct resume_leg g;

		resume_climb_run(rate, k, RESUME_CLIMB_RES_DB, &g);
		resume_range_add(&r, &g, 0u);
		resume_onset_check(&g, true, false, true);
		CHECK(g.erle_db[0] >= RESUME_MARGIN_DB);
	}
	resume_range_print("climb", &r);
}

/*
 * Three quiet misses at 8 phases after a burst whose tracker falls from 29 dB
 * under MISS_RESUME_DB. The burst reached MISS_RESUME_DB, so the dip resumes
 * the run at the far onset. After the next burst, which stays under
 * MISS_RESUME_DB, no dip counts, and the far onset waits 7 frames.
 * A resume without the dip, or with the largest reading of an older burst,
 * fails here.
 */
static void resume_dip_test(unsigned rate)
{
	static struct resume_leg legs[MISS_PHASES][2];
	struct resume_range r[2];
	unsigned k;

	resume_range_init(&r[0]);
	resume_range_init(&r[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		resume_dip_run(rate, k, RESUME_DIP_RES_DB, legs[k]);
		resume_range_add(&r[0], &legs[k][0], 0u);
		resume_range_add(&r[1], &legs[k][1], 0u);
	}
	case_rate(rate, "lec nlp resumes the engage run at a far onset after "
		  "a burst whose tracker dipped under the resume level");
	resume_range_print("dip", &r[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		resume_onset_check(&legs[k][0], false, true, true);
		CHECK(legs[k][0].erle_db[0] >= RESUME_MARGIN_DB);
	}
	case_rate(rate, "lec nlp waits at a far onset after a burst under the "
		  "resume level that follows the dip");
	resume_range_print("burst after the dip", &r[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		resume_onset_check(&legs[k][1], false, false, false);
		CHECK(legs[k][1].erle_db[0] >= RESUME_MARGIN_DB);
	}
}

/*
 * The climb and the dip at 8 phases on lines whose tracker stands within
 * RESUME_NEAR_DB under RESUME_MARGIN_DB at the far onset. No resume follows,
 * and the burst waits 7 frames. A resume without the margin fails here.
 */
static void resume_margin_test(unsigned rate)
{
	struct resume_range r[2];
	unsigned k;

	resume_range_init(&r[0]);
	resume_range_init(&r[1]);
	case_rate(rate, "lec nlp waits at a far onset on a climb or a dip with "
		  "the tracker under 22 dB");
	for (k = 0; k < MISS_PHASES; k++) {
		struct resume_leg climb;
		struct resume_leg dip[2];

		resume_climb_run(rate, k, RESUME_LOW_CLIMB_RES_DB, &climb);
		resume_dip_run(rate, k, RESUME_LOW_DIP_RES_DB, dip);
		resume_range_add(&r[0], &climb, 0u);
		resume_range_add(&r[1], &dip[0], 0u);
		resume_onset_check(&climb, true, false, false);
		resume_onset_check(&dip[0], false, true, false);
		CHECK(climb.erle_db[0] < RESUME_MARGIN_DB);
		CHECK(climb.erle_db[0] >= RESUME_MARGIN_DB - RESUME_NEAR_DB);
		CHECK(dip[0].erle_db[0] < RESUME_MARGIN_DB);
		CHECK(dip[0].erle_db[0] >= RESUME_MARGIN_DB - RESUME_NEAR_DB);
	}
	resume_range_print("climb under the margin", &r[0]);
	resume_range_print("dip under the margin", &r[1]);
}

/*
 * The climb line at 8 phases with a near sound on burst frames 2 to 4 of the
 * burst before three quiet misses. Its dt verdicts cut the run, and the run is
 * not complete again when that burst ends. So the climb resumes no run at the
 * far onset, and the burst waits 7 frames. It then acts, which completes the
 * run, and the climb resumes the run after the next three misses. A cut that
 * sets no flag, or a complete run that leaves it set, fails here.
 */
static void resume_cut_test(unsigned rate)
{
	static struct resume_leg legs[MISS_PHASES][4];
	double talk[MISS_BURST_FRAMES] = { 0.0 };
	struct resume_range r[2];
	unsigned held = 0;
	unsigned waits;
	unsigned k;
	unsigned f;

	for (f = RESUME_TALK_FROM; f < RESUME_TALK_TO; f++)
		talk[f] = MISS_TALK_DB;
	resume_range_init(&r[0]);
	resume_range_init(&r[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		struct resume_trace tr;
		struct miss_line l;

		resume_line_init(&l, &tr, rate, k, MISS_RES_LOW_DB);
		l.res_db = RESUME_CLIMB_RES_DB;
		resume_leg_run(&l, &tr, "----", NULL, &legs[k][0]);
		resume_leg_run(&l, &tr, "----", talk, &legs[k][1]);
		resume_leg_run(&l, &tr, "-mmm", NULL, &legs[k][2]);
		resume_leg_run(&l, &tr, "-mmm", NULL, &legs[k][3]);
		udsp_ec_nlms.destroy(l.ec);
		resume_range_add(&r[0], &legs[k][2], 0u);
		resume_range_add(&r[1], &legs[k][3], 0u);
		held += legs[k][1].verdict[MISS_BURST_FRAMES - 1u] ==
			LECR_VERDICT_DT;
	}
	case_rate(rate, "lec nlp waits at a far onset on a climbing tracker "
		  "under the resume level after a dt verdict cut the run");
	printf("# the near sound holds its burst to the last frame in %u of %u "
	       "phases\n", held, MISS_PHASES);
	resume_range_print("climb after the cut", &r[0]);
	for (k = 0; k < MISS_PHASES; k++) {
		const struct resume_leg *g = legs[k];

		/* The burst of the sound acts before it and not from it on. */
		CHECK(g[1].active[RESUME_TALK_FROM - 1u]);
		CHECK_UINT(g[1].verdict[RESUME_TALK_FROM], LECR_VERDICT_DT);
		CHECK_UINT(resume_first_get(&g[1], RESUME_TALK_FROM, &waits),
			   MISS_BURST_FRAMES);
		resume_onset_check(&g[2], true, false, false);
		CHECK(g[2].erle_db[0] >= RESUME_MARGIN_DB);
	}
	case_rate(rate, "lec nlp resumes the engage run on a climbing tracker "
		  "once the run is complete after the cut");
	resume_range_print("climb after the complete run", &r[1]);
	for (k = 0; k < MISS_PHASES; k++) {
		resume_onset_check(&legs[k][3], true, false, true);
		CHECK(legs[k][3].erle_db[0] >= RESUME_MARGIN_DB);
	}
}

/*
 * The climb line at 8 phases with a pulse on the first frame of the burst
 * after three quiet misses. Where the pulse takes the short hangover, the
 * hold starts on a zeroed run, so no cut sets the flag. The first pass frame
 * after the hold reads a climbing tracker under the resume level. The resume
 * after a short hold takes MISS_RESUME_DB alone, so the burst waits 7 frames.
 * A short-hold resume on the climb fails here.
 */
static void resume_hold_test(unsigned rate)
{
	double pulse[MISS_BURST_FRAMES] = { 0.0 };
	struct resume_range r;
	unsigned held = 0;
	unsigned k;

	pulse[0] = RESUME_PULSE_DB;
	resume_range_init(&r);
	case_rate(rate, "lec nlp waits after a short hold from a far onset on "
		  "a climbing tracker under the resume level");
	for (k = 0; k < MISS_PHASES; k++) {
		struct resume_trace tr;
		struct resume_leg floor;
		struct resume_leg g;
		struct miss_line l;
		unsigned waits;
		unsigned first;
		unsigned end;

		resume_line_init(&l, &tr, rate, k, MISS_RES_LOW_DB);
		l.res_db = RESUME_CLIMB_RES_DB;
		resume_leg_run(&l, &tr, "----", NULL, &floor);
		resume_leg_run(&l, &tr, "-mmm", pulse, &g);
		udsp_ec_nlms.destroy(l.ec);
		if (!g.fired || g.set != TAIL_PAIR_SHORT_FRAMES ||
		    g.over_floor >= 10.0 * log10(PAIR_SCRIPT_FLOOR_RATIO))
			continue;
		held++;
		for (end = 1u; end < MISS_BURST_FRAMES &&
		     g.verdict[end] == LECR_VERDICT_DT; end++)
			;
		resume_range_add(&r, &g, end);
		first = resume_first_get(&g, end, &waits);
		CHECK_UINT(g.quiet, MISS_QUIET_FRAMES);
		CHECK_UINT(end, TAIL_PAIR_SHORT_FRAMES);
		CHECK_UINT(g.verdict[end], LECR_VERDICT_PASS);
		CHECK(g.erle_db[end] >= RESUME_MARGIN_DB);
		CHECK(g.erle_db[end] < MISS_RESUME_DB);
		CHECK(g.erle_db[end] > g.ago_db[end]);
		CHECK_UINT(first, end + WORDS_ENGAGE_FRAME);
		CHECK_UINT(waits, WORDS_ENGAGE_FRAME);
	}
	printf("# %u of %u phases take the short hangover\n", held,
	       MISS_PHASES);
	resume_range_print("first frame after the short hold", &r);
	CHECK(held >= HOLD_MIN_PHASES);
}

/* The fast line of resume_zero_test: the quiet-miss line with a 1 ms echo
 * path and a 16 ms tail. After a zero its tracker reaches 22 dB in about 60
 * frames, inside the RESUME_CLIMB_FRAMES ring. RESUME_GROW_DB grows its path on
 * each frame. The filter follows it without a verdict up to the watchdog
 * bound, and the tracker stays over 24 dB. */
#define RESUME_FAST_DELAY_MS	1.0
#define RESUME_FAST_TAIL_MS	16u
#define RESUME_GROW_DB		0.0025
#define RESUME_GROW_FRAMES	(100u * FRAMES_PER_SECOND)
/* The zero comes RESUME_ZERO_AT frames into a burst that the line holds on.
 * After it the echo product stands RESUME_ZERO_RES_DB under the echo, so the
 * tracker climbs to about 23.5 dB. The near probe follows RESUME_ZERO_NEAR
 * frames of the held burst, so its far onset falls inside the ring. The far
 * probe follows RESUME_ZERO_LONG frames, then RESUME_ZERO_TAIL frames at
 * RESUME_ZERO_DIP_DB, so the tracker dips. */
#define RESUME_ZERO_AT		12u
#define RESUME_ZERO_NEAR	65u
#define RESUME_ZERO_LONG	300u
#define RESUME_ZERO_TAIL	60u
#define RESUME_ZERO_RES_DB	24.0
#define RESUME_ZERO_DIP_DB	22.8

/* The fast line at the rate, settled, its trace from the first frame. */
static void resume_fast_init(struct miss_line *l, struct resume_trace *tr,
			     unsigned rate)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	unsigned delay = path_samples_get(RESUME_FAST_DELAY_MS, rate);

	miss_line_init(l, rate, 0u, MISS_RES_DEEP_DB);
	udsp_ec_nlms.destroy(l->ec);
	path_geometry_set(&l->path, rate);
	l->path.taps = delay + 1u;
	l->path.h[delay] = 1.0f;
	path_erl_set(&l->path, MISS_ERL_DB);
	l->ec = udsp_ec_nlms.create_rate(RESUME_FAST_TAIL_MS, rate);
	udsp_ec_nlms.nlp(l->ec, true);
	miss_line_quiet_set(l, MISS_QUIET_FRAMES);
	memset(tr, 0, sizeof(*tr));
	while (l->count < MISS_SETTLE_FRAMES ||
	       miss_phase_get(l) != MISS_BURST_FRAMES)
		resume_frame_run(l, 0.0, tr, &reading, &info);
}

/* One frame of a burst that the line holds past MISS_BURST_FRAMES, into the
 * trace and, when set, into a second trace. */
static void resume_long_frame_run(struct miss_line *l,
				  struct resume_trace *tr,
				  struct resume_trace *also,
				  struct udsp_ec_nlms_info *info)
{
	struct udsp_ec_nlms_frame reading;
	unsigned long base = l->count - l->count % l->period;

	l->count = base + l->count % MISS_BURST_FRAMES;
	resume_frame_run(l, 0.0, tr, &reading, info);
	if (also)
		resume_trace_add(also, reading.nlp_verdict, info->erle_db);
}

/* The watchdog's zero through the line: the echo path grows by
 * RESUME_GROW_DB on each held burst frame. The watchdog then zeroes the
 * filter outside a trial, and the path returns to its gain. The frames
 * of the growth, or 0 when no such zero came. */
static unsigned resume_watchdog_run(struct miss_line *l,
				    struct resume_trace *tr)
{
	struct udsp_ec_nlms_info info;
	struct echo_path base = l->path;
	double gain = pow(10.0, RESUME_GROW_DB / 20.0);
	uint32_t probes;
	uint32_t resets;
	unsigned f;
	unsigned k;

	udsp_ec_nlms_info_get(l->ec, &info);
	probes = info.probes;
	resets = info.diverge_resets;
	for (f = 1; f <= RESUME_GROW_FRAMES; f++) {
		for (k = 0; k < l->path.taps; k++)
			l->path.h[k] = (float)(l->path.h[k] * gain);
		resume_long_frame_run(l, tr, NULL, &info);
		if (info.diverge_resets == resets)
			continue;
		memcpy(l->path.h, base.h, sizeof(base.h));
		return info.probes == probes && info.coef_energy == 0.0f ? f :
									   0u;
	}
	return 0;
}

/*
 * The figures of a zero: the frames of the watchdog's growth, the tracker
 * before the zero and the act. Then the frame after the zero at which the
 * tracker reached RESUME_MARGIN_DB, and the frames from the zero to the far
 * onset. Last, the pause and the burst after it. stale_ago and stale_max are
 * the evidence that the instance holds when it keeps its readings through the
 * zero: the reading RESUME_CLIMB_FRAMES frames before the onset, and the
 * largest reading of the burst before the pause.
 */
struct resume_zero {
	unsigned grown;
	double pre_zero;
	bool acted;
	unsigned at_margin;
	unsigned onset;
	double stale_ago;
	double stale_max;
	struct resume_leg g;
};

/*
 * The fast line at the rate, its filter zeroed in an acted burst by the reset,
 * or by the watchdog. The burst goes on hold frames after the zero, or for
 * the far probe RESUME_ZERO_LONG frames and RESUME_ZERO_TAIL frames of a dip.
 * Then a pause with pattern on its quiet frames, and the burst after it. The
 * trace starts over at the zero and holds the evidence after it; the stale
 * trace runs through the zero.
 */
static void resume_zero_run(unsigned rate, bool watchdog, unsigned hold,
			    const char *pattern, struct resume_zero *z)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct resume_trace stale;
	struct resume_trace tr;
	struct miss_line l;
	unsigned pause;
	unsigned k;

	memset(z, 0, sizeof(*z));
	resume_fast_init(&l, &tr, rate);
	while (miss_phase_get(&l) != 0u)
		resume_frame_run(&l, 0.0, &tr, &reading, &info);
	for (k = 0; k < RESUME_ZERO_AT; k++)
		resume_long_frame_run(&l, &tr, NULL, &info);
	z->pre_zero = info.erle_db;
	z->acted = info.nlp_active;
	if (watchdog)
		z->grown = resume_watchdog_run(&l, &tr);
	else
		udsp_ec_nlms_reset(l.ec);
	stale = tr;
	memset(&tr, 0, sizeof(tr));
	l.res_db = RESUME_ZERO_RES_DB;
	for (k = 0; k < (hold ? hold : RESUME_ZERO_LONG + RESUME_ZERO_TAIL);
	     k++) {
		if (!hold && k == RESUME_ZERO_LONG)
			l.res_db = RESUME_ZERO_DIP_DB;
		resume_long_frame_run(&l, &tr, &stale, &info);
		if (!z->at_margin && info.erle_db >= RESUME_MARGIN_DB)
			z->at_margin = k + 1u;
	}
	l.count += MISS_BURST_FRAMES - l.count % l.period;
	pause = l.period - MISS_BURST_FRAMES;
	z->onset = k + pause + 1u;
	z->stale_ago = stale.ring[(stale.pos + pause) % RESUME_CLIMB_FRAMES];
	z->stale_max = stale.burst_max;
	resume_leg_run(&l, &tr, pattern, NULL, &z->g);
	udsp_ec_nlms.destroy(l.ec);
}
/*
 * A zero of the filter in an acted burst, by the reset and by the watchdog, on
 * the fast line. Near probe: the far onset after three quiet misses falls
 * within RESUME_CLIMB_FRAMES frames of the zero. The tracker climbs over an
 * empty ring slot, and the burst acts on its first frame. A ring kept through
 * the zero refuses that climb. Far probe: the tracker climbs back under
 * MISS_RESUME_DB and dips before four quiet misses. Only the burst before the
 * zero reached MISS_RESUME_DB, so the burst waits 7 frames; a burst maximum
 * kept through the zero resumes there.
 */
static void resume_zero_test(unsigned rate)
{
	static const char *const paths[2] = { "reset", "watchdog" };
	struct resume_zero near[2];
	struct resume_zero far[2];
	unsigned p;

	for (p = 0; p < 2u; p++) {
		resume_zero_run(rate, p == 1u, RESUME_ZERO_NEAR, "-mmm",
				&near[p]);
		resume_zero_run(rate, p == 1u, 0u, "mmmm", &far[p]);
	}
	case_rate(rate, "lec nlp resume after a zeroed filter reads no tracker "
		  "reading from before the zero");
	for (p = 0; p < 2u; p++) {
		const struct resume_zero *z = &near[p];
		unsigned waits;
		unsigned first = resume_first_get(&z->g, 0u, &waits);

		printf("# %s: tracker %.2f dB before the zero, acted %d, %u "
		       "frames of growth; %.0f dB from frame %u after the "
		       "zero; far onset on frame %u: tracker %.2f dB, %.2f dB "
		       "in a ring kept through the zero; acts from frame %u\n",
		       paths[p], z->pre_zero, z->acted, z->grown,
		       RESUME_MARGIN_DB, z->at_margin, z->onset,
		       z->g.erle_db[0], z->stale_ago, first);
		CHECK(z->acted);
		CHECK(z->pre_zero >= MISS_RESUME_DB);
		CHECK(p == 0u || z->grown > 0u);
		CHECK(z->at_margin > 0u);
		CHECK(z->onset < RESUME_CLIMB_FRAMES);
		CHECK(z->g.erle_db[0] >= RESUME_MARGIN_DB);
		CHECK(z->stale_ago > z->g.erle_db[0]);
		resume_onset_check(&z->g, true, false, true);
	}
	case_rate(rate, "lec nlp resume after a zeroed filter reads no burst "
		  "maximum from before the zero");
	for (p = 0; p < 2u; p++) {
		const struct resume_zero *z = &far[p];

		printf("# %s: tracker %.2f dB before the zero, acted %d; burst "
		       "up to %.2f dB after the zero and %.2f dB through it; "
		       "far onset: tracker %.2f dB, %u frames before %.2f "
		       "dB\n", paths[p], z->pre_zero, z->acted,
		       z->g.pause_max, z->stale_max, z->g.erle_db[0],
		       RESUME_CLIMB_FRAMES, z->g.ago_db[0]);
		CHECK(z->acted);
		CHECK(p == 0u || z->grown > 0u);
		CHECK(z->stale_max >= MISS_RESUME_DB);
		CHECK(z->g.erle_db[0] >= RESUME_MARGIN_DB);
		resume_onset_check(&z->g, false, false, false);
	}
}

/* The cut line of resume_zero_cut_test: the fast line grows to RESUME_CUT_COEF
 * of coefficient energy, then its echo product stands RESUME_CUT_RES_DB under
 * the echo while the growth goes on. After the zero the tracker climbs to
 * RESUME_CUT_CLIMB_DB, under MISS_GATE_DB. The far onset steps
 * RESUME_CUT_STEP_DB over MISS_FAR_DBM0. */
#define RESUME_CUT_COEF		3.97f
#define RESUME_CUT_RES_DB	10.0
#define RESUME_CUT_CLIMB_DB	17.0
#define RESUME_CUT_STEP_DB	16.0

/* One frame of a burst that the line holds past MISS_BURST_FRAMES, its echo
 * path scaled by gain before the frame, into the trace. The verdict of the
 * frame. */
static unsigned resume_held_frame_run(struct miss_line *l,
				      struct resume_trace *tr, double gain,
				      struct udsp_ec_nlms_info *info)
{
	struct udsp_ec_nlms_frame reading;
	unsigned long base = l->count - l->count % l->period;
	unsigned k;

	for (k = 0; k < l->path.taps; k++)
		l->path.h[k] = (float)(l->path.h[k] * gain);
	l->count = base + l->count % MISS_BURST_FRAMES;
	resume_frame_run(l, 0.0, tr, &reading, info);
	return reading.nlp_verdict;
}

/*
 * The figures of a cut and a zero: the frames of the growth to
 * RESUME_CUT_COEF and the act after it. Of the frames at RESUME_CUT_RES_DB,
 * counted from 1: the first that stops the act on an erle verdict, and the one
 * on which the watchdog zeroes the filter. The zero reads 0 for none, and
 * RESUME_GROW_FRAMES for one inside a trial or with coefficient energy left.
 * Then the pass frames from the cut to the end of the climb, and the frames of
 * the climb and the tracker after it. Last, the frames from the zero to the far
 * onset, and the pause and the burst after it.
 */
struct resume_cut_zero {
	unsigned grown;
	bool acted;
	unsigned cut;
	unsigned zero;
	unsigned passes;
	unsigned climb;
	double climbed;
	unsigned onset;
	struct resume_leg g;
};

/*
 * The cut line at the rate. Its path grows by RESUME_GROW_DB on each held
 * burst frame until the watchdog zeroes the filter, and then returns to its
 * gain. The tracker climbs after the zero on the held burst. Then a pause with
 * three quiet misses, and the burst after it at the far step. The trace starts
 * over at the zero.
 */
static void resume_cut_zero_run(unsigned rate, struct resume_cut_zero *z)
{
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct resume_trace tr;
	struct echo_path base;
	struct miss_line l;
	double gain = pow(10.0, RESUME_GROW_DB / 20.0);
	uint32_t probes;
	uint32_t resets;
	unsigned verdict;
	unsigned f;

	memset(z, 0, sizeof(*z));
	resume_fast_init(&l, &tr, rate);
	while (miss_phase_get(&l) != 0u)
		resume_frame_run(&l, 0.0, &tr, &reading, &info);
	base = l.path;
	probes = info.probes;
	resets = info.diverge_resets;
	while (info.coef_energy < RESUME_CUT_COEF &&
	       z->grown < RESUME_GROW_FRAMES) {
		resume_held_frame_run(&l, &tr, gain, &info);
		z->grown++;
	}
	z->acted = info.nlp_active;
	l.res_db = RESUME_CUT_RES_DB;
	for (f = 1; f <= RESUME_GROW_FRAMES && !z->zero; f++) {
		bool acting = info.nlp_active;

		verdict = resume_held_frame_run(&l, &tr, gain, &info);
		if (!z->cut && acting && !info.nlp_active &&
		    verdict == LECR_VERDICT_ERLE)
			z->cut = f;
		z->passes += z->cut && verdict == LECR_VERDICT_PASS;
		if (info.diverge_resets == resets)
			continue;
		z->zero = info.probes == probes && info.coef_energy == 0.0f ?
			  f : RESUME_GROW_FRAMES;
	}
	memcpy(l.path.h, base.h, sizeof(base.h));
	memset(&tr, 0, sizeof(tr));
	l.res_db = MISS_RES_DEEP_DB;
	while (z->climb < RESUME_CLIMB_FRAMES &&
	       info.erle_db < RESUME_CUT_CLIMB_DB) {
		verdict = resume_held_frame_run(&l, &tr, 1.0, &info);
		z->passes += verdict == LECR_VERDICT_PASS;
		z->climb++;
	}
	z->climbed = info.erle_db;
	l.count += MISS_BURST_FRAMES - l.count % l.period;
	z->onset = z->climb + l.period - MISS_BURST_FRAMES + 1u;
	l.far_dbm0 = MISS_FAR_DBM0 + RESUME_CUT_STEP_DB;
	resume_leg_run(&l, &tr, "-mmm", NULL, &z->g);
	udsp_ec_nlms.destroy(l.ec);
}

/*
 * A watchdog zero after a cut, on the cut line. An erle frame stops the act
 * and cuts the run, and the watchdog zeroes the filter later in the held
 * burst. No pass frame follows the cut up to the far onset after three quiet
 * misses. The far step lifts the tracker on the onset to between
 * RESUME_MARGIN_DB and MISS_RESUME_DB, over an empty ring slot. The cut stays
 * through the zero, so the burst waits 7 frames; a cut cleared by the watchdog
 * resumes on the climb.
 */
static void resume_zero_cut_test(unsigned rate)
{
	struct resume_cut_zero z;
	unsigned waits;
	unsigned first;

	resume_cut_zero_run(rate, &z);
	first = resume_first_get(&z.g, 0u, &waits);
	case_rate(rate, "lec nlp resume after a watchdog zero keeps the cut of "
		  "the run");
	printf("# %u frames of growth, acted %d; erle cut on frame %u, zero on "
	       "frame %u, %u pass frames after the cut; tracker %.2f dB after "
	       "%u frames of climb; far onset on frame %u after the zero: "
	       "tracker %.2f dB; acts from frame %u\n", z.grown, z.acted, z.cut,
	       z.zero, z.passes, z.climbed, z.climb, z.onset, z.g.erle_db[0],
	       first);
	CHECK(z.acted);
	CHECK(z.cut > 0u);
	CHECK(z.zero > z.cut && z.zero < RESUME_GROW_FRAMES);
	CHECK_UINT(z.passes, 0u);
	CHECK(z.climbed < MISS_GATE_DB);
	CHECK(z.onset < RESUME_CLIMB_FRAMES);
	CHECK(z.g.erle_db[0] >= RESUME_MARGIN_DB);
	resume_onset_check(&z.g, true, false, false);
}

/* The burst of resume_switch_test starts with the processor switched off, and
 * the switch returns on its frame RESUME_OFF_FRAMES. The line holds the burst
 * to frame RESUME_OFF_HOLD. */
#define RESUME_OFF_FRAMES	20u
#define RESUME_OFF_HOLD		40u

/*
 * The dip line at 8 phases. The burst whose tracker falls from 29 dB starts
 * with the processor switched off, and the switch returns with the tracker
 * under MISS_RESUME_DB. The processor acts again before the burst ends. The
 * frames with the switch off clear the burst maximum, so no dip counts after
 * three quiet misses, and the burst waits 7 frames. The trace of the test
 * keeps the maximum of those frames, as a maximum kept through the switch
 * does, and that maximum resumes the run.
 */
static void resume_switch_test(unsigned rate)
{
	struct resume_range r;
	double on_hi = -200.0;
	unsigned acts_lo = MISS_BURST_FRAMES;
	unsigned k;

	resume_range_init(&r);
	case_rate(rate, "lec nlp waits at a far onset after a dip that the "
		  "processor switch cut");
	for (k = 0; k < MISS_PHASES; k++) {
		struct udsp_ec_nlms_frame reading;
		struct udsp_ec_nlms_info info;
		struct resume_trace tr;
		struct resume_leg floor;
		struct resume_leg g;
		struct miss_line l;
		double on_max = -200.0;
		unsigned acts = 0;
		unsigned f;

		resume_line_init(&l, &tr, rate, k, MISS_RES_DEEP_DB);
		resume_leg_run(&l, &tr, "----", NULL, &floor);
		while (miss_phase_get(&l) != 0u)
			resume_frame_run(&l, 0.0, &tr, &reading, &info);
		l.res_db = RESUME_DIP_RES_DB;
		udsp_ec_nlms.nlp(l.ec, false);
		for (f = 0; f < RESUME_OFF_HOLD; f++) {
			if (f == RESUME_OFF_FRAMES)
				udsp_ec_nlms.nlp(l.ec, true);
			resume_long_frame_run(&l, &tr, NULL, &info);
			if (f < RESUME_OFF_FRAMES)
				continue;
			on_max = fmax(on_max, info.erle_db);
			acts += info.nlp_active;
		}
		l.count += MISS_BURST_FRAMES - l.count % l.period;
		resume_leg_run(&l, &tr, "-mmm", NULL, &g);
		udsp_ec_nlms.destroy(l.ec);
		resume_range_add(&r, &g, 0u);
		on_hi = fmax(on_hi, on_max);
		acts_lo = acts < acts_lo ? acts : acts_lo;
		CHECK(acts > 0u);
		CHECK(on_max < MISS_RESUME_DB);
		CHECK(g.erle_db[0] >= RESUME_MARGIN_DB);
		resume_onset_check(&g, false, true, false);
	}
	printf("# after the switch returns: tracker up to %.2f dB, %u acting "
	       "frames at least\n", on_hi, acts_lo);
	resume_range_print("dip cut by the switch", &r);
}

int main(int argc, char **argv)
{
	int failed;

	if (!path_select(argc, argv))
		return 2;
	tail_clamp_test();
	convergence_test(UDSP_SAMPLE_RATE);
	tail_bound_test();
	doubletalk_test(UDSP_SAMPLE_RATE);
	freeze_test();
	nlp_test();
	saturation_test();
	determinism_test();
	detector_latch_test();
	nlp_near_only_test();
	path_step_test(UDSP_SAMPLE_RATE);
	sustained_doubletalk_test();
	bootstrap_test();
	gain_bound_test();
	lockout_escape_test();
	probe_pause_test();
	probe_rearm_test();
	nlp_combined_loss_test();
	nlp_echo_like_test();
	nlp_hysteresis_test(UDSP_SAMPLE_RATE);
	reset_test();
	reconverge_test(UDSP_SAMPLE_RATE);
	lockout_release_test(UDSP_SAMPLE_RATE);
	near_end_preserve_test(UDSP_SAMPLE_RATE);
	bootstrap_hold_test();
	probe_reject_test();
	probe_freeze_test();
	probe_cooldown_test(UDSP_SAMPLE_RATE);
	latch_refusal_test(UDSP_SAMPLE_RATE);
	watchdog_test();
	/*
	 * Each group of legs runs after the groups above it, so a case keeps
	 * its case number when a group joins the suite. The 16 kHz leg is one
	 * such group.
	 */
	create_rate_test();
	convergence_test(UDSP_SAMPLE_RATE_WB);
	doubletalk_test(UDSP_SAMPLE_RATE_WB);
	path_step_test(UDSP_SAMPLE_RATE_WB);
	reconverge_test(UDSP_SAMPLE_RATE_WB);
	lockout_release_test(UDSP_SAMPLE_RATE_WB);
	near_end_preserve_test(UDSP_SAMPLE_RATE_WB);
	probe_cooldown_test(UDSP_SAMPLE_RATE_WB);
	latch_refusal_test(UDSP_SAMPLE_RATE_WB);
	upper_band_test();
	/* The vector-path legs run last for the same reason. */
	last_tap_test(UDSP_SAMPLE_RATE);
	last_tap_test(UDSP_SAMPLE_RATE_WB);
	last_sample_test(UDSP_SAMPLE_RATE);
	last_sample_test(UDSP_SAMPLE_RATE_WB);
	equivalence_test();
	/* The weak-hybrid legs run after them for the same reason. */
	noise_floor_stale_test();
	weak_hybrid_test(UDSP_SAMPLE_RATE);
	weak_hybrid_test(UDSP_SAMPLE_RATE_WB);
	probe_divergence_test(UDSP_SAMPLE_RATE);
	probe_divergence_test(UDSP_SAMPLE_RATE_WB);
	talkover_test(UDSP_SAMPLE_RATE);
	talkover_test(UDSP_SAMPLE_RATE_WB);
	louder_hybrid_test(UDSP_SAMPLE_RATE);
	louder_hybrid_test(UDSP_SAMPLE_RATE_WB);
	probe_boundary_test(UDSP_SAMPLE_RATE);
	probe_boundary_test(UDSP_SAMPLE_RATE_WB);
	noise_floor_outlier_test(UDSP_SAMPLE_RATE);
	noise_floor_outlier_test(UDSP_SAMPLE_RATE_WB);
	watchdog_recovery_test();
	/* The zero-skip legs run last for the numbering reason above. */
	zero_skip_test(UDSP_SAMPLE_RATE);
	zero_skip_test(UDSP_SAMPLE_RATE_WB);
	/* The verdict and peak legs run last for the numbering reason above. */
	nlp_quiet_count_test();
	nlp_erle_count_test();
	nlp_unlike_count_test();
	nlp_dt_count_test();
	nlp_wait_count_test(UDSP_SAMPLE_RATE);
	peak_tap_test(UDSP_SAMPLE_RATE);
	peak_tap_test(UDSP_SAMPLE_RATE_WB);
	reset_count_test();
	divergence_peak_test();
	/* The feed leg runs last for the numbering reason above. */
	ref_feed_test();
	/* The reference offset and peak legs run last for the numbering reason
	 * above. */
	ref_offset_test(UDSP_SAMPLE_RATE);
	ref_offset_test(UDSP_SAMPLE_RATE_WB);
	peak_zero_test();
	peak_tie_test();
	/* The trial verdict legs run last for the numbering reason above. */
	latch_pause_test();
	probe_syllable_test();
	/* The onset leg runs last for the numbering reason above. */
	probe_onset_test();
	/* The floor window leg runs last for the numbering reason above. */
	noise_floor_window_test();
	/* The processor floor legs run last for the numbering reason above. */
	nlp_floor_test();
	/* The frame reading legs run last for the numbering reason above. */
	reading_verdict_test();
	reading_window_test(UDSP_SAMPLE_RATE);
	reading_window_test(UDSP_SAMPLE_RATE_WB);
	reading_reset_test();
	reading_floor_test();
	/* The hangover legs run last for the numbering reason above. */
	hangover_floor_test();
	hangover_no_floor_test();
	hangover_loud_test();
	hangover_rearm_test();
	hangover_marginal_edge_test();
	hangover_soft_edge_test();
	hangover_talker_test();
	hangover_headroom_test();
	hangover_repeat_test();
	hangover_repeat_start_test();
	hangover_sparse_test();
	/* The replay legs run last for the numbering reason above. */
	replay_call_test(UDSP_SAMPLE_RATE);
	replay_call_test(UDSP_SAMPLE_RATE_WB);
	replay_agree_test();
	replay_verdict_word_test();
	capture_verdict_word_test();
	/* The capture fixture leg runs last for the numbering reason above. */
	capture_fixture_test();
	/* The reader rule, exclusion and bound legs run last for the numbering
	 * reason above. */
	replay_rule_test();
	replay_skip_test();
	replay_bound_test();
	/* The floor tracker legs follow, for the numbering reason above. */
	floor_pause_words_test();
	floor_pause_speech_test();
	floor_rise_test();
	floor_silence_test();
	floor_short_pause_test();
	floor_bound_test();
	floor_run_break_test();
	floor_quiet6_test();
	floor_age_test();
	floor_reset_test();
	/* The seed legs follow, for the numbering reason above. A reset that
	 * keeps the seed run cannot be observed: the reset zeroes the ERLE, so
	 * gate (b) clears the run on the next frame. */
	fan_seed_test(UDSP_SAMPLE_RATE);
	fan_seed_test(UDSP_SAMPLE_RATE_WB);
	fan_turns_test();
	fan_measured_test();
	fan_reseed_test();
	fan_lowpass_test();
	fan_step_test();
	seed_block_test(UDSP_SAMPLE_RATE);
	seed_block_test(UDSP_SAMPLE_RATE_WB);
	seed_trial_test();
	/* The snapshot leg follows, for the numbering reason above. */
	probe_snapshot_test();
	/* The stale-estimate legs follow, for the numbering reason above. */
	ring_trust_test(UDSP_SAMPLE_RATE);
	ring_trust_test(UDSP_SAMPLE_RATE_WB);
	stale_hold_test(UDSP_SAMPLE_RATE);
	stale_hold_test(UDSP_SAMPLE_RATE_WB);
	stale_talker_test();
	stale_quiet_test();
	expose_steady_test();
	expose_turns_test();
	expose_stop_test();
	expose_twin_test();
	change_test(UDSP_SAMPLE_RATE);
	change_test(UDSP_SAMPLE_RATE_WB);
	stale_return_test(UDSP_SAMPLE_RATE);
	stale_return_test(UDSP_SAMPLE_RATE_WB);
	stale_carry_test(UDSP_SAMPLE_RATE);
	stale_carry_test(UDSP_SAMPLE_RATE_WB);
	stale_repay_test(UDSP_SAMPLE_RATE);
	stale_repay_test(UDSP_SAMPLE_RATE_WB);
	/* The trial field leg follows, for the numbering reason above. */
	trial_mark_test();
	/* The talker guard legs follow, for the numbering reason above. */
	latch_talker_test(UDSP_SAMPLE_RATE);
	latch_talker_test(UDSP_SAMPLE_RATE_WB);
	trial_talker_test();
	latch_return_test();
	talker_tail_test(UDSP_SAMPLE_RATE);
	talker_tail_test(UDSP_SAMPLE_RATE_WB);
	/* The far-end floor legs follow, for the numbering reason above. */
	far_floor_talker_test(UDSP_SAMPLE_RATE);
	far_floor_talker_test(UDSP_SAMPLE_RATE_WB);
	far_floor_identity_test();
	far_floor_bar_test();
	far_floor_cap_test();
	far_floor_feed_test();
	far_floor_trial_test();
	far_floor_latch_test(UDSP_SAMPLE_RATE);
	far_floor_latch_test(UDSP_SAMPLE_RATE_WB);
	far_floor_words_test();
	far_floor_toggle_test();
	/* The trust count legs follow, for the numbering reason above. */
	ring_inside_test();
	ring_lockout_test(UDSP_SAMPLE_RATE);
	ring_lockout_test(UDSP_SAMPLE_RATE_WB);
	trust_restart_test(UDSP_SAMPLE_RATE);
	trust_restart_test(UDSP_SAMPLE_RATE_WB);
	trust_window_talker_test(UDSP_SAMPLE_RATE);
	trust_window_talker_test(UDSP_SAMPLE_RATE_WB);
	trust_turn_test(UDSP_SAMPLE_RATE);
	trust_turn_test(UDSP_SAMPLE_RATE_WB);
	/* The reference loss legs follow, for the numbering reason above. */
	ref_loss_setter_test(UDSP_SAMPLE_RATE);
	ref_loss_setter_test(UDSP_SAMPLE_RATE_WB);
	ref_loss_gate_test(UDSP_SAMPLE_RATE);
	ref_loss_gate_test(UDSP_SAMPLE_RATE_WB);
	ref_loss_equiv_test(UDSP_SAMPLE_RATE);
	ref_loss_equiv_test(UDSP_SAMPLE_RATE_WB);
	capture_fixture_loss_test();
	ref_loss_bound_test();
	/* The legs of the engage run through a pause follow, for the numbering
	 * reason above. */
	nlp_hysteresis_test(UDSP_SAMPLE_RATE_WB);
	nlp_spurt_test(UDSP_SAMPLE_RATE);
	nlp_spurt_test(UDSP_SAMPLE_RATE_WB);
	nlp_wait_count_test(UDSP_SAMPLE_RATE_WB);
	state_room_test(UDSP_SAMPLE_RATE);
	state_room_test(UDSP_SAMPLE_RATE_WB);
	state_silence_test(UDSP_SAMPLE_RATE);
	state_silence_test(UDSP_SAMPLE_RATE_WB);
	state_early_test(UDSP_SAMPLE_RATE);
	state_early_test(UDSP_SAMPLE_RATE_WB);
	state_trust_test(UDSP_SAMPLE_RATE);
	state_trust_test(UDSP_SAMPLE_RATE_WB);
	pause_burst_test(UDSP_SAMPLE_RATE);
	pause_burst_test(UDSP_SAMPLE_RATE_WB);
	pause_expose_test(UDSP_SAMPLE_RATE);
	pause_expose_test(UDSP_SAMPLE_RATE_WB);
	nlp_dt_wait_test(UDSP_SAMPLE_RATE);
	nlp_dt_wait_test(UDSP_SAMPLE_RATE_WB);
	nlp_erle_wait_test(UDSP_SAMPLE_RATE);
	nlp_erle_wait_test(UDSP_SAMPLE_RATE_WB);
	/* The pause headroom legs follow, for the numbering reason above. */
	pause_headroom_test(UDSP_SAMPLE_RATE);
	pause_headroom_test(UDSP_SAMPLE_RATE_WB);
	active_headroom_test(UDSP_SAMPLE_RATE);
	active_headroom_test(UDSP_SAMPLE_RATE_WB);
	/* The legs of the cut run follow, for the numbering reason above. */
	nlp_flap_cost_test(UDSP_SAMPLE_RATE);
	nlp_flap_cost_test(UDSP_SAMPLE_RATE_WB);
	nlp_cut_test(UDSP_SAMPLE_RATE);
	nlp_cut_test(UDSP_SAMPLE_RATE_WB);
	cohere_change_test(UDSP_SAMPLE_RATE);
	cohere_change_test(UDSP_SAMPLE_RATE_WB);
	cohere_step_test(UDSP_SAMPLE_RATE);
	cohere_step_test(UDSP_SAMPLE_RATE_WB);
	cohere_shape_test(UDSP_SAMPLE_RATE);
	cohere_shape_test(UDSP_SAMPLE_RATE_WB);
	cohere_precedence_test(UDSP_SAMPLE_RATE);
	cohere_precedence_test(UDSP_SAMPLE_RATE_WB);
	cohere_refuse_test(UDSP_SAMPLE_RATE);
	cohere_refuse_test(UDSP_SAMPLE_RATE_WB);
	cohere_bound_test(UDSP_SAMPLE_RATE);
	cohere_bound_test(UDSP_SAMPLE_RATE_WB);
	cohere_reseed_test(UDSP_SAMPLE_RATE);
	cohere_reseed_test(UDSP_SAMPLE_RATE_WB);
	cohere_expose_test(UDSP_SAMPLE_RATE);
	cohere_expose_test(UDSP_SAMPLE_RATE_WB);
	cohere_talker_test(UDSP_SAMPLE_RATE);
	cohere_talker_test(UDSP_SAMPLE_RATE_WB);
	cohere_pause_test(UDSP_SAMPLE_RATE);
	cohere_pause_test(UDSP_SAMPLE_RATE_WB);
	cohere_near_trial_test(UDSP_SAMPLE_RATE);
	cohere_near_trial_test(UDSP_SAMPLE_RATE_WB);
	cohere_cycle_test(UDSP_SAMPLE_RATE);
	cohere_cycle_test(UDSP_SAMPLE_RATE_WB);
	cohere_diverge_test();
	cohere_gain_test(UDSP_SAMPLE_RATE);
	cohere_gain_test(UDSP_SAMPLE_RATE_WB);
	cohere_share_test(UDSP_SAMPLE_RATE);
	cohere_share_test(UDSP_SAMPLE_RATE_WB);
	cohere_partial_test(UDSP_SAMPLE_RATE);
	cohere_partial_test(UDSP_SAMPLE_RATE_WB);
	noise_creep_test(UDSP_SAMPLE_RATE);
	noise_creep_test(UDSP_SAMPLE_RATE_WB);
	nlp_first_engage_test(UDSP_SAMPLE_RATE);
	nlp_first_engage_test(UDSP_SAMPLE_RATE_WB);
	nlp_gate_hover_test(UDSP_SAMPLE_RATE);
	nlp_gate_hover_test(UDSP_SAMPLE_RATE_WB);
	floor_peak_test(UDSP_SAMPLE_RATE);
	floor_peak_test(UDSP_SAMPLE_RATE_WB);
	floor_peak_sequence_test(UDSP_SAMPLE_RATE);
	floor_peak_sequence_test(UDSP_SAMPLE_RATE_WB);
	floor_peak_talker_test(UDSP_SAMPLE_RATE);
	floor_peak_talker_test(UDSP_SAMPLE_RATE_WB);
	trial_bare_test(UDSP_SAMPLE_RATE);
	trial_bare_test(UDSP_SAMPLE_RATE_WB);
	lull_far_test(UDSP_SAMPLE_RATE);
	lull_far_test(UDSP_SAMPLE_RATE_WB);
	lull_talk_test(UDSP_SAMPLE_RATE);
	lull_talk_test(UDSP_SAMPLE_RATE_WB);
	lull_cap_test(UDSP_SAMPLE_RATE);
	lull_cap_test(UDSP_SAMPLE_RATE_WB);
	lull_ring_test(UDSP_SAMPLE_RATE);
	lull_ring_test(UDSP_SAMPLE_RATE_WB);
	lull_skip_test(UDSP_SAMPLE_RATE);
	lull_skip_test(UDSP_SAMPLE_RATE_WB);
	quiet_miss_test(UDSP_SAMPLE_RATE);
	quiet_miss_test(UDSP_SAMPLE_RATE_WB);
	quiet_miss_limit_test(UDSP_SAMPLE_RATE);
	quiet_miss_limit_test(UDSP_SAMPLE_RATE_WB);
	quiet_miss_talker_test(UDSP_SAMPLE_RATE);
	quiet_miss_talker_test(UDSP_SAMPLE_RATE_WB);
	cohere_tone_test(UDSP_SAMPLE_RATE);
	cohere_tone_test(UDSP_SAMPLE_RATE_WB);
	cohere_near_tone_test(UDSP_SAMPLE_RATE);
	cohere_near_tone_test(UDSP_SAMPLE_RATE_WB);
	phase2_test(UDSP_SAMPLE_RATE);
	phase2_test(UDSP_SAMPLE_RATE_WB);
	dense_change_test(UDSP_SAMPLE_RATE);
	dense_change_test(UDSP_SAMPLE_RATE_WB);
	misadjust_test(UDSP_SAMPLE_RATE);
	misadjust_test(UDSP_SAMPLE_RATE_WB);
	phase1_test(UDSP_SAMPLE_RATE);
	phase1_test(UDSP_SAMPLE_RATE_WB);
	nlp_boot_near_start_test(UDSP_SAMPLE_RATE);
	nlp_boot_near_start_test(UDSP_SAMPLE_RATE_WB);
	cn9a_test(UDSP_SAMPLE_RATE);
	cn9a_test(UDSP_SAMPLE_RATE_WB);
	cn_quiet_test(UDSP_SAMPLE_RATE);
	cn_quiet_test(UDSP_SAMPLE_RATE_WB);
	cn_drop_test();
	dt3a_test(UDSP_SAMPLE_RATE);
	dt3a_test(UDSP_SAMPLE_RATE_WB);
	dt3b_test(UDSP_SAMPLE_RATE);
	dt3b_test(UDSP_SAMPLE_RATE_WB);
	nlp_boot_seeds_test(UDSP_SAMPLE_RATE);
	nlp_boot_seeds_test(UDSP_SAMPLE_RATE_WB);
	onset_mic_test(UDSP_SAMPLE_RATE);
	onset_mic_test(UDSP_SAMPLE_RATE_WB);
	dt3a_phase_test(UDSP_SAMPLE_RATE);
	dt3a_phase_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_test(UDSP_SAMPLE_RATE);
	floor_keep_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_flat_test(UDSP_SAMPLE_RATE);
	floor_keep_flat_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_talker_test(UDSP_SAMPLE_RATE);
	floor_keep_talker_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_window_test(UDSP_SAMPLE_RATE);
	floor_keep_window_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_expiry_test(UDSP_SAMPLE_RATE);
	floor_keep_expiry_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_held_test(UDSP_SAMPLE_RATE);
	floor_keep_held_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_open_talker_test(UDSP_SAMPLE_RATE);
	floor_keep_open_talker_test(UDSP_SAMPLE_RATE_WB);
	floor_keep_pause_test(UDSP_SAMPLE_RATE);
	floor_keep_pause_test(UDSP_SAMPLE_RATE_WB);
	latch_louder_talker_test(UDSP_SAMPLE_RATE);
	latch_louder_talker_test(UDSP_SAMPLE_RATE_WB);
	rollback_output_test(UDSP_SAMPLE_RATE);
	rollback_output_test(UDSP_SAMPLE_RATE_WB);
	rollback_mask_test(UDSP_SAMPLE_RATE);
	rollback_mask_test(UDSP_SAMPLE_RATE_WB);
	onset_peak_test(UDSP_SAMPLE_RATE);
	onset_peak_test(UDSP_SAMPLE_RATE_WB);
	tone_tail_test(UDSP_SAMPLE_RATE);
	tone_tail_test(UDSP_SAMPLE_RATE_WB);
	escape_trial_line_test(UDSP_SAMPLE_RATE);
	escape_trial_line_test(UDSP_SAMPLE_RATE_WB);
	escape_trial_talker_test(UDSP_SAMPLE_RATE);
	escape_trial_talker_test(UDSP_SAMPLE_RATE_WB);
	escape_trial_start_test(UDSP_SAMPLE_RATE);
	escape_trial_start_test(UDSP_SAMPLE_RATE_WB);
	escape_trial_pause_test(UDSP_SAMPLE_RATE);
	escape_trial_pause_test(UDSP_SAMPLE_RATE_WB);
	escape_trial_feed_test(UDSP_SAMPLE_RATE);
	escape_trial_feed_test(UDSP_SAMPLE_RATE_WB);
	hangover_tail_pair_test(UDSP_SAMPLE_RATE);
	hangover_tail_pair_test(UDSP_SAMPLE_RATE_WB);
	hangover_pair_script_test(UDSP_SAMPLE_RATE);
	hangover_pair_script_test(UDSP_SAMPLE_RATE_WB);
	resume_erle_test(UDSP_SAMPLE_RATE);
	resume_erle_test(UDSP_SAMPLE_RATE_WB);
	resume_talker_test(UDSP_SAMPLE_RATE);
	resume_talker_test(UDSP_SAMPLE_RATE_WB);
	resume_floor_test(UDSP_SAMPLE_RATE);
	resume_floor_test(UDSP_SAMPLE_RATE_WB);
	quiet_miss_low_test(UDSP_SAMPLE_RATE);
	quiet_miss_low_test(UDSP_SAMPLE_RATE_WB);
	resume_off_test(UDSP_SAMPLE_RATE);
	resume_off_test(UDSP_SAMPLE_RATE_WB);
	quiet_headroom_low_test(UDSP_SAMPLE_RATE);
	quiet_headroom_low_test(UDSP_SAMPLE_RATE_WB);
	agree_readings_test(UDSP_SAMPLE_RATE);
	agree_readings_test(UDSP_SAMPLE_RATE_WB);
	agree_pauses_test(UDSP_SAMPLE_RATE);
	agree_pauses_test(UDSP_SAMPLE_RATE_WB);
	agree_window_test(UDSP_SAMPLE_RATE);
	agree_window_test(UDSP_SAMPLE_RATE_WB);
	agree_band_test(UDSP_SAMPLE_RATE);
	agree_band_test(UDSP_SAMPLE_RATE_WB);
	agree_falsify_test(UDSP_SAMPLE_RATE);
	agree_falsify_test(UDSP_SAMPLE_RATE_WB);
	agree_cap_test(UDSP_SAMPLE_RATE);
	agree_cap_test(UDSP_SAMPLE_RATE_WB);
	agree_stale_test(UDSP_SAMPLE_RATE);
	agree_stale_test(UDSP_SAMPLE_RATE_WB);
	agree_feed_test(UDSP_SAMPLE_RATE);
	agree_feed_test(UDSP_SAMPLE_RATE_WB);
	tail_peak_test(UDSP_SAMPLE_RATE);
	tail_peak_test(UDSP_SAMPLE_RATE_WB);
	hold_resume_test(UDSP_SAMPLE_RATE);
	hold_resume_test(UDSP_SAMPLE_RATE_WB);
	hold_refuse_test(UDSP_SAMPLE_RATE);
	hold_refuse_test(UDSP_SAMPLE_RATE_WB);
	hold_trial_test(UDSP_SAMPLE_RATE);
	hold_trial_test(UDSP_SAMPLE_RATE_WB);
	hold_cut_test(UDSP_SAMPLE_RATE);
	hold_cut_test(UDSP_SAMPLE_RATE_WB);
	hangover_pair_bar_test(UDSP_SAMPLE_RATE);
	hangover_pair_bar_test(UDSP_SAMPLE_RATE_WB);
	hold_repeat_test(UDSP_SAMPLE_RATE);
	hold_repeat_test(UDSP_SAMPLE_RATE_WB);
	onset_peak_clear_test(UDSP_SAMPLE_RATE);
	onset_peak_clear_test(UDSP_SAMPLE_RATE_WB);
	onset_peak_toggle_test(UDSP_SAMPLE_RATE);
	onset_peak_toggle_test(UDSP_SAMPLE_RATE_WB);
	over_arm_test(UDSP_SAMPLE_RATE);
	over_arm_test(UDSP_SAMPLE_RATE_WB);
	over_return_test(UDSP_SAMPLE_RATE);
	over_return_test(UDSP_SAMPLE_RATE_WB);
	over_talker_test(UDSP_SAMPLE_RATE);
	over_talker_test(UDSP_SAMPLE_RATE_WB);
	over_edge_test(UDSP_SAMPLE_RATE);
	over_edge_test(UDSP_SAMPLE_RATE_WB);
	over_watchdog_test(UDSP_SAMPLE_RATE);
	over_watchdog_test(UDSP_SAMPLE_RATE_WB);
	onset_start_test(UDSP_SAMPLE_RATE);
	onset_start_test(UDSP_SAMPLE_RATE_WB);
	drop_talker_test(UDSP_SAMPLE_RATE);
	drop_talker_test(UDSP_SAMPLE_RATE_WB);
	drop_copy_test(UDSP_SAMPLE_RATE);
	drop_copy_test(UDSP_SAMPLE_RATE_WB);
	drop_misfit_test(UDSP_SAMPLE_RATE);
	drop_misfit_test(UDSP_SAMPLE_RATE_WB);
	drop_misfit_break_test(UDSP_SAMPLE_RATE);
	drop_misfit_break_test(UDSP_SAMPLE_RATE_WB);
	implied_inside_test(UDSP_SAMPLE_RATE);
	implied_inside_test(UDSP_SAMPLE_RATE_WB);
	implied_outside_test(UDSP_SAMPLE_RATE);
	implied_outside_test(UDSP_SAMPLE_RATE_WB);
	implied_cap_test(UDSP_SAMPLE_RATE);
	implied_cap_test(UDSP_SAMPLE_RATE_WB);
	keep_renewal_test(UDSP_SAMPLE_RATE);
	keep_renewal_test(UDSP_SAMPLE_RATE_WB);
	implied_uncapped_test(UDSP_SAMPLE_RATE);
	implied_uncapped_test(UDSP_SAMPLE_RATE_WB);
	implied_tail_test(UDSP_SAMPLE_RATE);
	implied_tail_test(UDSP_SAMPLE_RATE_WB);
	fade_lead_test(UDSP_SAMPLE_RATE);
	fade_lead_test(UDSP_SAMPLE_RATE_WB);
	fade_plain_test(UDSP_SAMPLE_RATE);
	fade_plain_test(UDSP_SAMPLE_RATE_WB);
	fade_tail_test(UDSP_SAMPLE_RATE);
	fade_tail_test(UDSP_SAMPLE_RATE_WB);
	fade_weight_test(UDSP_SAMPLE_RATE);
	fade_weight_test(UDSP_SAMPLE_RATE_WB);
	fade_emit_test(UDSP_SAMPLE_RATE);
	fade_emit_test(UDSP_SAMPLE_RATE_WB);
	onset_zero_test(UDSP_SAMPLE_RATE);
	onset_zero_test(UDSP_SAMPLE_RATE_WB);
	onset_zero_boot_test(UDSP_SAMPLE_RATE);
	onset_zero_boot_test(UDSP_SAMPLE_RATE_WB);
	resume_climb_test(UDSP_SAMPLE_RATE);
	resume_climb_test(UDSP_SAMPLE_RATE_WB);
	resume_dip_test(UDSP_SAMPLE_RATE);
	resume_dip_test(UDSP_SAMPLE_RATE_WB);
	resume_margin_test(UDSP_SAMPLE_RATE);
	resume_margin_test(UDSP_SAMPLE_RATE_WB);
	resume_cut_test(UDSP_SAMPLE_RATE);
	resume_cut_test(UDSP_SAMPLE_RATE_WB);
	resume_hold_test(UDSP_SAMPLE_RATE);
	resume_hold_test(UDSP_SAMPLE_RATE_WB);
	resume_zero_test(UDSP_SAMPLE_RATE);
	resume_zero_test(UDSP_SAMPLE_RATE_WB);
	resume_switch_test(UDSP_SAMPLE_RATE);
	resume_switch_test(UDSP_SAMPLE_RATE_WB);
	resume_zero_cut_test(UDSP_SAMPLE_RATE);
	resume_zero_cut_test(UDSP_SAMPLE_RATE_WB);
	replay_gap_test();
	failed = tu_summary("lec");
	printf("# lec per-tap path: %s\n",
	       udsp_ec_nlms_path_name(udsp_ec_nlms_path_get()));
	return failed;
}
