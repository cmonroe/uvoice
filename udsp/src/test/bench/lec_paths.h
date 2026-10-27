// SPDX-License-Identifier: LGPL-2.1-only
/*
 * udsp - software voice DSP for FXS lines
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * Echo paths, levels and the fixed-seed noise source shared by the LEC unit
 * suite (test/test_lec.c) and the G.168-derived bench (test/bench/bench_lec.c).
 * Both need the same hybrids to mean the same thing, so they read them from
 * one file.
 *
 * Everything here is test scaffolding: it is the oracle side of the suite and
 * shares no code with dsp/lec.c, so a defect in the canceller cannot hide in
 * a matching defect here.
 */
#ifndef UDSP_TEST_LEC_PATHS_H
#define UDSP_TEST_LEC_PATHS_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "../../dsp/dsp.h"
#include "../level.h"

/*
 * 80 ms at 16 kHz: the longest response any case builds is the 60 ms flat
 * hybrid of the tail-bound case (961 taps at 16 kHz), and the wideband tail
 * itself caps at 64 ms (LEC_TAIL_MS_WB_MAX), so this covers both with margin.
 */
#define LEC_PATH_TAPS_MAX	1280u

/*
 * A path is built at one line rate and runs one frame of that rate per call,
 * so the shapes below state their geometry in milliseconds and sample it at
 * the rate they are given. Every constant divides exactly at 8 kHz, so the
 * narrowband shapes sample with no rounding.
 *
 * The drive history is LEC_PATH_TAPS_MAX long whatever the response is, so it
 * does not depend on the shape that happens to be reading it. A path change
 * then carries a history long enough for the longest destination, and the
 * convolution reads the same ring whatever the response.
 */
struct echo_path {
	unsigned frame;
	unsigned taps;
	unsigned pos;
	double cubic;
	float h[LEC_PATH_TAPS_MAX];
	float hist[2u * LEC_PATH_TAPS_MAX];
};

static inline unsigned path_samples_get(double ms, unsigned rate)
{
	return (unsigned)lrint(ms * (double)rate / 1000.0);
}

/*
 * The dispersive hybrid oscillates at 4 kHz, the narrowband band edge, so one
 * sample is a half cycle at 8 kHz and a quarter cycle at 16 kHz: the exact
 * cosine values below keep the taps free of any libm rounding at both rates,
 * where cos(pi * k) would only be exactly +-1 by the grace of the library.
 */
static inline double path_edge_cos(unsigned k, unsigned rate)
{
	static const double quarter[4] = { 1.0, 0.0, -1.0, 0.0 };

	return quarter[(k * (UDSP_SAMPLE_RATE_WB / rate)) % 4u];
}

/* The frame is the line frame of the rate the path is built at, because the
 * canceller the path feeds runs exactly one such frame per call. */
static inline void path_geometry_set(struct echo_path *path, unsigned rate)
{
	memset(path, 0, sizeof(*path));
	path->frame = rate / 100u;
}

static inline uint32_t rng_next(uint32_t *state)
{
	uint32_t value = *state;

	value ^= value << 13;
	value ^= value >> 17;
	value ^= value << 5;
	*state = value;
	return value;
}

static inline int16_t sample_clamp(double value)
{
	if (value > 32767.0)
		return 32767;
	if (value < -32768.0)
		return -32768;
	return (int16_t)lrint(value);
}

/* phase counts the samples the tone has written, so two calls with equal
 * phases write two tones that start together. */
static inline void tone_fill(int16_t *dst, unsigned count, double level_dbm0,
			     double hz, unsigned long *phase, unsigned rate)
{
	double amplitude = dbm0_rms_get(level_dbm0) * sqrt(2.0);
	unsigned i;

	for (i = 0; i < count; i++)
		dst[i] = sample_clamp(amplitude *
				      sin(2.0 * M_PI * hz *
					  (double)(*phase + i) /
					  rate));
	*phase += count;
}

static inline void noise_fill(int16_t *dst, unsigned count, double level_dbm0,
			      uint32_t *state)
{
	double amplitude = dbm0_rms_get(level_dbm0) * sqrt(3.0);
	unsigned i;

	for (i = 0; i < count; i++) {
		double unit = rng_next(state) / 4294967295.0 * 2.0 - 1.0;

		dst[i] = sample_clamp(unit * amplitude);
	}
}

/*
 * Scales the response to a stated echo return loss. For a white reference the
 * echo power is the reference power times the sum of the squared taps, so the
 * sum is what the ERL fixes. Called on a live path it changes the hybrid
 * without touching its history, which is how a path step is staged.
 */
static inline void path_erl_set(struct echo_path *path, double erl_db)
{
	double energy = 0.0;
	double scale;
	unsigned k;

	for (k = 0; k < path->taps; k++)
		energy += (double)path->h[k] * path->h[k];
	scale = sqrt(pow(10.0, -erl_db / 10.0) / energy);
	for (k = 0; k < path->taps; k++)
		path->h[k] = (float)(path->h[k] * scale);
}

/*
 * Moves the far-end history of one hybrid into the hybrid that replaces it, so
 * a path change is only a change of response. Without it the new path starts
 * with an empty delay line and returns no echo at all until its own length has
 * refilled, which is a dropout followed by a change and reads as part of the
 * reconvergence time.
 *
 * src->hist[src->pos + k] is the drive sample delayed by k, so the carry is a
 * straight walk of the shared delays. The whole ring moves, not the source's
 * response length: a destination that reaches further back than the source's
 * own taps would otherwise read zeros there, which is a cold tail inside the
 * new response and not a path change (S7 switches a 25 ms hybrid for a 40 ms
 * one, so a carry of the source's length leaves 15 ms of it cold). The
 * destination ring is written at position 0 and the doubled half is kept in
 * step, the invariant path_run relies on.
 */
static inline void path_history_carry(struct echo_path *dst,
				      const struct echo_path *src)
{
	unsigned k;

	dst->pos = 0;
	for (k = 0; k < LEC_PATH_TAPS_MAX; k++) {
		float value = src->hist[src->pos + k];

		dst->hist[k] = value;
		dst->hist[k + LEC_PATH_TAPS_MAX] = value;
	}
}

static inline void path_flat_init(struct echo_path *path, unsigned rate,
				  unsigned delay_ms, double erl_db)
{
	unsigned delay = path_samples_get(delay_ms, rate);

	path_geometry_set(path, rate);
	path->taps = delay + 1u;
	path->h[delay] = 1.0f;
	path_erl_set(path, erl_db);
}

/* A 3 ms decaying oscillation at the 4 kHz band edge, 0.75 ms of decay. */
static inline void path_dispersive_init(struct echo_path *path, unsigned rate,
					double erl_db)
{
	double per_ms = (double)rate / 1000.0;
	unsigned k;

	path_geometry_set(path, rate);
	path->taps = path_samples_get(3.0, rate);
	for (k = 0; k < path->taps; k++)
		path->h[k] = (float)(exp(-(double)k / (0.75 * per_ms)) *
				     path_edge_cos(k, rate));
	path_erl_set(path, erl_db);
}

/*
 * A composite hybrid derived from G.168, not an Annex D coefficient table
 * (path_annex_d_init): a flat bulk section, a dispersive main reflection from
 * 4 to 12 ms (891 Hz, 1.75 ms of decay) and a second weaker reflection from
 * 20 to 25 ms (637 Hz, 1.25 ms of decay), which is the shape the Annex D
 * models have.
 */
static inline void path_composite_init(struct echo_path *path, unsigned rate,
				       double erl_db)
{
	double per_ms = (double)rate / 1000.0;
	unsigned main_from = path_samples_get(4.0, rate);
	unsigned main_to = path_samples_get(12.0, rate);
	unsigned late_from = path_samples_get(20.0, rate);
	unsigned k;

	path_geometry_set(path, rate);
	path->taps = path_samples_get(25.0, rate);
	for (k = main_from; k < main_to; k++)
		path->h[k] = (float)(exp(-(double)(k - main_from) /
					 (1.75 * per_ms)) *
				     sin(5.6 / per_ms * (double)(k -
								 main_from)));
	for (k = late_from; k < path->taps; k++)
		path->h[k] = (float)(0.35 * exp(-(double)(k - late_from) /
						(1.25 * per_ms)) *
				     sin(4.0 / per_ms * (double)(k -
								 late_from)));
	path_erl_set(path, erl_db);
}

/* A hybrid whose whole response sits at 28 ms, past a 16 or 24 ms tail:
 * 509 Hz with 2.5 ms of decay, running to 40 ms. */
static inline void path_late_init(struct echo_path *path, unsigned rate,
				  double erl_db)
{
	double per_ms = (double)rate / 1000.0;
	unsigned from = path_samples_get(28.0, rate);
	unsigned k;

	path_geometry_set(path, rate);
	path->taps = path_samples_get(40.0, rate);
	for (k = from; k < path->taps; k++)
		path->h[k] = (float)(exp(-(double)(k - from) / (2.5 * per_ms)) *
				     sin(3.2 / per_ms * (double)(k - from)));
	path_erl_set(path, erl_db);
}

/*
 * Wideband only: a 6 kHz resonance from 2 to 10 ms with 1.5 ms of decay, the
 * upper-octave hybrid the two-band fallback decision turns on (a hybrid
 * response crossing 4 kHz loses coherence at the band split). 6 kHz has no
 * narrowband form, so a request at 8 kHz builds nothing: an empty path faults
 * at once in path_run under the sanitizers instead of quietly standing in
 * with a plausible low-band hybrid.
 */
static inline void path_upper_init(struct echo_path *path, unsigned rate,
				   double erl_db)
{
	double per_ms = (double)rate / 1000.0;
	unsigned from = path_samples_get(2.0, rate);
	unsigned k;

	path_geometry_set(path, rate);
	if (rate != UDSP_SAMPLE_RATE_WB)
		return;
	path->taps = path_samples_get(10.0, rate);
	for (k = from; k < path->taps; k++)
		path->h[k] = (float)(exp(-(double)(k - from) / (1.5 * per_ms)) *
				     sin(2.0 * M_PI * 6.0 * (double)(k - from) /
					 per_ms));
	path_erl_set(path, erl_db);
}

/*
 * The weak hybrid of the field lines: a balance network that matches the loop
 * at the low end and mismatches with frequency. The wall-clock-aligned traces
 * of real calls from a cell phone to an AN7581 line read a median raw return
 * loss of 30.4 dB with a tenth percentile of 26.0 dB. Those two numbers differ
 * only because the loss depends on the frequency of the frame: a flat hybrid
 * returns one number whatever the source does.
 *
 * Two sections. The transformer leak sits at 0.5 ms and is a first
 * difference, so the residual rises 6 dB per octave; the balance mismatch
 * runs from 3 to 15 ms, resonates at 2400 Hz and decays over 3 ms. erl_db
 * fixes the white-noise return loss, the quantity path_erl_set normalizes,
 * and a colored frame sees more or less of it.
 *
 * The two sections are equally weighted, which is what calibrates the model
 * against the field: at erl_db 28 the loss this shape returns to the word
 * source, measured in 100 ms blocks the way the traces were measured, reads
 * p10 25.9 and p50 30.6 dB against the field's p10 26.0 and p50 30.4. A
 * weaker resonance narrows that spread and stops modeling the line.
 *
 * 2400 Hz is inside the narrowband band, so the shape exists at both line
 * rates, unlike the upper-octave hybrid above.
 */
static inline void path_weak_init(struct echo_path *path, unsigned rate,
				  double erl_db)
{
	double per_ms = (double)rate / 1000.0;
	unsigned early = path_samples_get(0.5, rate);
	unsigned from = path_samples_get(3.0, rate);
	unsigned k;

	path_geometry_set(path, rate);
	path->taps = path_samples_get(15.0, rate);
	path->h[early] = 1.0f;
	path->h[early + 1u] = -1.0f;
	for (k = from; k < path->taps; k++)
		path->h[k] += (float)(exp(-(double)(k - from) /
					  (3.0 * per_ms)) *
				      sin(2.0 * M_PI * 2400.0 *
					  (double)(k - from) / rate));
	path_erl_set(path, erl_db);
}

/*
 * Composite source derived from the G.168 Annex C description. One 500 ms
 * cycle repeats:
 * - a voiced burst of 100 Hz harmonics, 50 ms;
 * - a pause, 150 ms;
 * - pseudo noise, 200 ms;
 * - a second pause, 100 ms.
 * The bench and the suite need its activity factor, and its alternation
 * between a periodic and a noise-like excitation. phase counts the samples of
 * the 500 ms cycle, so two sources whose phases differ by a quarter cycle
 * never voice together.
 */
static inline void css_fill(int16_t *dst, unsigned count, double level_dbm0,
			    uint32_t *state, unsigned long *phase,
			    unsigned rate)
{
	double amplitude = dbm0_rms_get(level_dbm0);
	unsigned long cycle = rate / 2u;
	unsigned long burst_to = rate / 20u;
	unsigned long noise_from = rate / 5u;
	unsigned long noise_to = 2ul * rate / 5u;
	unsigned i;

	for (i = 0; i < count; i++) {
		unsigned long position = (*phase + i) % cycle;
		double value = 0.0;
		unsigned harmonic;

		if (position < burst_to) {
			for (harmonic = 1; harmonic <= 12u; harmonic++)
				value += sin(2.0 * M_PI * 100.0 * harmonic *
					     (double)position /
					     rate) / harmonic;
			value *= amplitude * 0.8;
		} else if (position >= noise_from && position < noise_to) {
			double unit = rng_next(state) / 4294967295.0 * 2.0 -
				1.0;

			value = unit * amplitude * sqrt(3.0);
		}
		dst[i] = sample_clamp(value);
	}
	*phase = (*phase + count) % cycle;
}

/*
 * A source with the time structure of the single talk CSS of G.168 Annex C.3.1:
 * per 350 ms half period, a voiced burst of 48.62 ms (Table C.1, interpolated
 * from 44.1 kHz), a noise burst of 200 ms (white noise through a one-pole
 * high-pass at 150 Hz and low-pass at 400 Hz) and a pause; the second half
 * inverts the first. Both bursts have unit RMS, so the period RMS is the level.
 * The canceller converges on it as on the CSS of the harness.
 */
#define CSS_ANNEX_C1_RATE	44100u
#define CSS_ANNEX_C1_SAMPLES	134u
#define CSS_ANNEX_VOICED_MS	48.62
#define CSS_ANNEX_NOISE_MS	200.0
#define CSS_ANNEX_HALF_MS	350.0
#define CSS_ANNEX_HIGH_HZ	150.0
#define CSS_ANNEX_LOW_HZ	400.0
#define CSS_ANNEX_VOICED_MAX	1200u
#define CSS_ANNEX_NOISE_MAX	3200u

static const int16_t css_annex_c1[CSS_ANNEX_C1_SAMPLES] = {
	-155, 276, 517, 578, 491, 302, 86, -103, -207, -198, -60, 190, 543,
	948, 1362, 1741, 2043, 2276, 2422, 2500, 2552, 2595, 2655, 2758, 2896,
	3060, 3224, 3370, 3500, 3569, 3603, 3603, 3595, 3586, 3595, 3638, 3724,
	3819, 3922, 4000, 4043, 4034, 3974, 3862, 3724, 3577, 3439, 3336, 3267,
	3224, 3198, 3172, 3129, 3043, 2914, 2750, 2560, 2353, 2155, 1991, 1853,
	1750, 1672, 1603, 1534, 1440, 1310, 1146, 965, 776, 603, 448, 345, 276,
	250, 250, 267, 267, 241, 190, 103, -9, -138, -267, -388, -491, -569,
	-638, -698, -759, -813, -888, -957, -1034, -1103, -1146, -1181, -1190,
	-1198, -1215, -1259, -1327, -1457, -1629, -1853, -2121, -2414, -2707,
	-3017, -3319, -3612, -3913, -4224, -4560, -4922, -5301, -5715, -6137,
	-6560, -6948, -7301, -7568, -7732, -7758, -7620, -7310, -6810, -6155,
	-5344, -4439, -3474, -2508, -1595, -802,
};

/* The two bursts hold unit RMS; scale sets the level. */
struct css_annex {
	double voiced[CSS_ANNEX_VOICED_MAX];
	double noise[CSS_ANNEX_NOISE_MAX];
	unsigned voiced_n;
	unsigned noise_n;
	unsigned half_n;
	unsigned long pos;
	double scale;
};

static inline void css_annex_unit_set(double *x, unsigned n)
{
	double sum = 0.0;
	unsigned k;

	for (k = 0; k < n; k++)
		sum += x[k] * x[k];
	for (k = 0; k < n; k++)
		x[k] /= sqrt(sum / n);
}

/* The noise burst: white noise from seed through the two filters, from
 * filters at rest. */
static inline void css_annex_noise_draw(struct css_annex *s, unsigned rate,
					uint32_t seed)
{
	double high_a = exp(-2.0 * M_PI * CSS_ANNEX_HIGH_HZ / rate);
	double low_a = exp(-2.0 * M_PI * CSS_ANNEX_LOW_HZ / rate);
	double high_x = 0.0;
	double high_y = 0.0;
	double low_y = 0.0;
	uint32_t state = seed;
	unsigned n;

	for (n = 0; n < s->noise_n; n++) {
		double unit = (rng_next(&state) / 4294967295.0 * 2.0 - 1.0) *
			      sqrt(3.0);

		high_y = high_a * (high_y + unit - high_x);
		high_x = unit;
		low_y = (1.0 - low_a) * high_y + low_a * low_y;
		s->noise[n] = low_y;
	}
	css_annex_unit_set(s->noise, s->noise_n);
}

/* The source with the voiced period of table, of table_n samples at
 * CSS_ANNEX_C1_RATE, a voiced burst of voiced_ms and a half period of
 * half_ms. */
static inline void css_annex_setup(struct css_annex *s, unsigned rate,
				   double level_dbm0, uint32_t seed,
				   const int16_t *table, unsigned table_n,
				   double voiced_ms, double half_ms)
{
	unsigned n;

	memset(s, 0, sizeof(*s));
	s->voiced_n = path_samples_get(voiced_ms, rate);
	s->noise_n = path_samples_get(CSS_ANNEX_NOISE_MS, rate);
	s->half_n = path_samples_get(half_ms, rate);
	if (s->voiced_n > CSS_ANNEX_VOICED_MAX)
		s->voiced_n = CSS_ANNEX_VOICED_MAX;
	if (s->noise_n > CSS_ANNEX_NOISE_MAX)
		s->noise_n = CSS_ANNEX_NOISE_MAX;
	for (n = 0; n < s->voiced_n; n++) {
		double t = fmod((double)n * CSS_ANNEX_C1_RATE / rate, table_n);
		unsigned k = (unsigned)t;
		double frac = t - k;

		s->voiced[n] = table[k] * (1.0 - frac) +
			       table[(k + 1u) % table_n] * frac;
	}
	css_annex_unit_set(s->voiced, s->voiced_n);
	css_annex_noise_draw(s, rate, seed);
	s->scale = dbm0_rms_get(level_dbm0) /
		   sqrt((double)(s->voiced_n + s->noise_n) / s->half_n);
}

static inline void css_annex_init(struct css_annex *s, unsigned rate,
				  double level_dbm0, uint32_t seed)
{
	css_annex_setup(s, rate, level_dbm0, seed, css_annex_c1,
			CSS_ANNEX_C1_SAMPLES, CSS_ANNEX_VOICED_MS,
			CSS_ANNEX_HALF_MS);
}

/*
 * A source with the time structure of the double talk CSS of G.168 Annex C.3.2:
 * per 400 ms half period, a voiced burst of 72.70 ms (Table C.3, whose value
 * 168 both editions print as -938), a noise burst of 200 ms and a pause. Its
 * period differs from css_annex_init, so near bursts slide over far ones as in
 * G.168 tests 3A to 3C. The active level stands 1.66 dB over the period RMS
 * (G.168 clause 6.4.1.2).
 */
#define CSS_ANNEX_C3_SAMPLES	229u
#define CSS_ANNEX_DT_VOICED_MS	72.70
#define CSS_ANNEX_DT_HALF_MS	400.0

static const int16_t css_annex_c3[CSS_ANNEX_C3_SAMPLES] = {
	-198, -112, -9, 103, 233, 388, 543, 724, 896, 1060, 1233, 1388, 1517,
	1638, 1747, 1810, 1845, 1845, 1802, 1707, 1569, 1379, 1146, 871, 560,
	233, -121, -491, -871, -1250, -1638, -2043, -2465, -2896, -3345,
	-3819, -4310, -4810, -5319, -5836, -6353, -6853, -7353, -7836, -8292,
	-8715, -9077, -9370, -9542, -9542, -9361, -8956, -8327, -7465, -6396,
	-5163, -3827, -2448, -1103, 155, 1293, 2241, 3034, 3655, 4138, 4517,
	4827, 5094, 5344, 5594, 5827, 6043, 6215, 6344, 6413, 6422, 6379,
	6310, 6215, 6120, 6051, 6000, 5991, 5991, 6000, 6008, 5991, 5939,
	5853, 5715, 5560, 5387, 5215, 5043, 4879, 4732, 4586, 4439, 4276,
	4086, 3870, 3629, 3370, 3086, 2801, 2534, 2267, 2034, 1819, 1612,
	1422, 1224, 1026, 819, 603, 388, 181, 9, -181, -328, -448, -543, -629,
	-707, -784, -871, -948, -1026, -1112, -1181, -1241, -1276, -1293,
	-1302, -1293, -1267, -1250, -1233, -1224, -1224, -1224, -1224, -1215,
	-1198, -1172, -1129, -1077, -1026, -974, -922, -888, -871, -845, -828,
	-810, -793, -767, -741, -698, -672, -638, -603, -595, -586, -595,
	-603, -621, -629, -938, -638, -638, -638, -638, -638, -647, -664,
	-690, -724, -767, -793, -819, -845, -853, -871, -879, -888, -896,
	-922, -948, -974, -1009, -1026, -1052, -1069, -1077, -1069, -1060,
	-1060, -1052, -1043, -1043, -1052, -1060, -1060, -1060, -1052, -1034,
	-1017, -991, -957, -931, -905, -888, -862, -845, -819, -793, -767,
	-724, -672, -621, -560, -509, -457, -397, -345, -276, -207, -112,
};

static inline void css_annex_dt_init(struct css_annex *s, unsigned rate,
				     double level_dbm0, uint32_t seed)
{
	css_annex_setup(s, rate, level_dbm0, seed, css_annex_c3,
			CSS_ANNEX_C3_SAMPLES, CSS_ANNEX_DT_VOICED_MS,
			CSS_ANNEX_DT_HALF_MS);
}

static inline void css_annex_fill(int16_t *dst, unsigned count,
				  struct css_annex *s)
{
	unsigned i;

	for (i = 0; i < count; i++) {
		unsigned long at = s->pos % s->half_n;
		double sign = s->pos < s->half_n ? 1.0 : -1.0;
		double value = 0.0;

		if (at < s->voiced_n)
			value = s->voiced[at];
		else if (at < s->voiced_n + s->noise_n)
			value = s->noise[at - s->voiced_n];
		dst[i] = sample_clamp(sign * s->scale * value);
		s->pos = (s->pos + 1u) % (2u * s->half_n);
	}
}

/*
 * Room noise at the 8 kHz rate of the wire, as the near end of G.168 test 9A
 * plays it: ROOM_WHITE, or ROOM_HOTH, white noise through the two filters of
 * the composite source's noise burst, falling 6 dB per octave over 400 Hz near
 * the Hoth spectrum (IEEE 269). The scale sets the RMS from a first draw of
 * ROOM_CAL_SAMPLES; the source then restarts from rest with its seed.
 */
#define ROOM_CAL_SAMPLES	80000u

enum room_kind {
	ROOM_WHITE,
	ROOM_HOTH,
};

struct room_noise {
	enum room_kind kind;
	double high_a;
	double low_a;
	double high_x;
	double high_y;
	double low_y;
	double scale;
	uint32_t seed;
	uint32_t state;
};

static inline double room_noise_next(struct room_noise *r)
{
	double unit = (rng_next(&r->state) / 4294967295.0 * 2.0 - 1.0) *
		      sqrt(3.0);

	if (r->kind == ROOM_WHITE)
		return unit;
	r->high_y = r->high_a * (r->high_y + unit - r->high_x);
	r->high_x = unit;
	r->low_y = (1.0 - r->low_a) * r->high_y + r->low_a * r->low_y;
	return r->low_y;
}

static inline void room_noise_restart(struct room_noise *r)
{
	r->state = r->seed;
	r->high_x = 0.0;
	r->high_y = 0.0;
	r->low_y = 0.0;
}

static inline void room_noise_init(struct room_noise *r, enum room_kind kind,
				   double level_dbm0, uint32_t seed)
{
	double sum = 0.0;
	unsigned n;

	memset(r, 0, sizeof(*r));
	r->kind = kind;
	r->seed = seed;
	r->high_a = exp(-2.0 * M_PI * CSS_ANNEX_HIGH_HZ / UDSP_SAMPLE_RATE);
	r->low_a = exp(-2.0 * M_PI * CSS_ANNEX_LOW_HZ / UDSP_SAMPLE_RATE);
	room_noise_restart(r);
	for (n = 0; n < ROOM_CAL_SAMPLES; n++) {
		double v = room_noise_next(r);

		sum += v * v;
	}
	r->scale = dbm0_rms_get(level_dbm0) / sqrt(sum / ROOM_CAL_SAMPLES);
	room_noise_restart(r);
}

static inline void room_noise_fill(int16_t *dst, unsigned count,
				   struct room_noise *r)
{
	unsigned i;

	for (i = 0; i < count; i++)
		dst[i] = sample_clamp(r->scale * room_noise_next(r));
}

/*
 * Far-end words: speech-shaped bursts that stop abruptly, with a pause of
 * 300 ms or more after each one. The field echo traces show the far caller
 * hears the last syllable of their own word, so the 100 ms after a word
 * offset is the window a residual has to be measured in. Neither bench source
 * has those properties: the composite source pauses for 150 ms inside a
 * 500 ms cycle and never changes its spectrum, and white noise has no offsets
 * at all.
 *
 * One word is a syllabic raised-cosine envelope at 3.5 Hz over two-pole
 * colored noise, and the color is drawn per word. A hybrid whose return
 * loss depends on frequency then returns a different loss for each word,
 * which is the spread of the reference traces; the level of a word is the
 * envelope whatever its color, because the frame is rescaled after the
 * color filter runs.
 *
 * offset is true on the last frame of a word, so the caller measures the
 * frames that follow it.
 */
/*
 * The unvoiced onset of a word, in 10 ms frames and in dB under the voiced
 * body. A fricative is quieter than the vowel that follows it, and it is the
 * first thing on the line after the pause, so the reference window maximum
 * the detector reads is the fricative's own frame: what the detector then
 * compares is the hybrid's return loss at the fricative's frequencies.
 */
#define WORD_ONSET_FRAMES	4u
#define WORD_ONSET_DB		8.0

struct word_source {
	uint32_t state;
	unsigned rate;
	unsigned pos;
	unsigned word;
	bool talking;
	bool offset;
	double level_dbm0;
	double m1;
	double m2;
};

static inline void word_source_init(struct word_source *w, unsigned rate,
				    double level_dbm0, uint32_t seed)
{
	memset(w, 0, sizeof(*w));
	w->state = seed ? seed : 1u;
	w->rate = rate;
	w->level_dbm0 = level_dbm0;
	w->talking = true;
}

static inline void word_source_restart(struct word_source *w)
{
	w->pos = 0;
	w->talking = true;
	w->offset = false;
}

static inline void word_fill(struct word_source *w, int16_t *dst,
			     unsigned frame)
{
	/* Word and pause lengths in 10 ms frames. The shortest pause is
	 * 300 ms, six times the 48 ms tail, so the echo of one word has ended
	 * long before the next one starts. */
	static const unsigned length[4] = { 18u, 25u, 32u, 21u };
	static const unsigned pause[4] = { 30u, 55u, 38u, 45u };
	static const double colour[4][2] = {
		{ 1.2, -0.6 }, { 1.7, -0.9 }, { 0.5, 0.2 }, { -0.4, -0.3 },
	};
	/* The unvoiced onset: a two-pole resonance near the top of the band,
	 * which is where a fricative puts its energy and where a balance
	 * network matched for the voiceband leaks. */
	static const double onset[2] = { -1.1, -0.5 };
	bool unvoiced = w->pos < WORD_ONSET_FRAMES;
	const double *tap = unvoiced ? onset : colour[w->word % 4u];
	unsigned index = w->word % 4u;
	double energy = 0.0;
	double target;
	double scale;
	unsigned k;

	w->offset = false;
	if (!w->talking) {
		memset(dst, 0, frame * sizeof(*dst));
		if (++w->pos < pause[index])
			return;
		w->pos = 0;
		w->word++;
		w->talking = true;
		return;
	}
	target = dbm0_rms_get(w->level_dbm0 +
			      7.0 * cos(2.0 * M_PI * 3.5 * (double)w->pos *
					(double)frame / (double)w->rate) -
			      (unvoiced ? WORD_ONSET_DB : 0.0));
	noise_fill(dst, frame, w->level_dbm0, &w->state);
	for (k = 0; k < frame; k++) {
		double y = 0.3 * dst[k] + tap[0] * w->m1 + tap[1] * w->m2;

		w->m2 = w->m1;
		w->m1 = y;
		dst[k] = sample_clamp(y);
		energy += y * y;
	}
	scale = energy > 0.0 ? target / sqrt(energy / frame) : 0.0;
	for (k = 0; k < frame; k++)
		dst[k] = sample_clamp((double)dst[k] * scale);
	if (++w->pos < length[index])
		return;
	/* The word stops here. No fade, and the next frame is digital
	 * silence: at that edge the far caller hears the echo of the word. */
	w->pos = 0;
	w->talking = false;
	w->offset = true;
}

/*
 * The non-linearity sits on the path input, where a real hybrid puts it: the
 * part of the distortion orthogonal to the reference is what no linear filter
 * can remove, and it is what the NLP has to hide.
 */
static inline void path_run(struct echo_path *path, const int16_t *ref,
			    int16_t *echo)
{
	unsigned i;
	unsigned k;

	for (i = 0; i < path->frame; i++) {
		double drive = ref[i];
		double acc = 0.0;

		if (path->cubic != 0.0)
			drive -= path->cubic * drive * drive * drive /
				 (32768.0 * 32768.0);
		path->pos = path->pos ? path->pos - 1u : LEC_PATH_TAPS_MAX - 1u;
		path->hist[path->pos] = (float)drive;
		path->hist[path->pos + LEC_PATH_TAPS_MAX] = (float)drive;
		for (k = 0; k < path->taps; k++)
			acc += (double)path->h[k] * path->hist[path->pos + k];
		echo[i] = sample_clamp(acc);
	}
}

#define REF_DELAY_MAX	LEC_PATH_TAPS_MAX

/*
 * Moves the reference of the canceller against the drive of the path. win[k]
 * of dsp/lec.c hist_push is the reference delayed by k, and hist[pos + k] of
 * path_run is the drive delayed by k, so equal inputs put tap 0 of the
 * canceller on tap 0 of the path. A positive offset delays the reference, and
 * tap 0 of the canceller then sits offset samples into the response. A
 * negative offset delays the drive, which adds margin before the response.
 * The delay line sits on the side that lags: a trimmed path would take the
 * early echo out of the microphone, and a late reference must leave it there.
 * Offset 0 copies the drive to both outputs, so a caller at 0 sees the frames
 * it saw without the delay line.
 */
struct ref_delay {
	int offset;
	unsigned pos;
	int16_t ring[REF_DELAY_MAX];
};

/* |offset| must be under REF_DELAY_MAX. */
static inline void ref_delay_init(struct ref_delay *d, int offset)
{
	memset(d, 0, sizeof(*d));
	d->offset = offset;
}

static inline void ref_delay_run(struct ref_delay *d, const int16_t *drive,
				 unsigned count, int16_t *path_in,
				 int16_t *ref)
{
	unsigned lag = (unsigned)(d->offset < 0 ? -d->offset : d->offset);
	unsigned i;

	for (i = 0; i < count; i++) {
		int16_t now = drive[i];
		int16_t late;

		d->ring[d->pos] = now;
		late = d->ring[(d->pos + REF_DELAY_MAX - lag) % REF_DELAY_MAX];
		d->pos = (d->pos + 1u) % REF_DELAY_MAX;
		path_in[i] = d->offset < 0 ? late : now;
		ref[i] = d->offset > 0 ? late : now;
	}
}

/*
 * A captured echo path: the least-squares fit of a canceller-off call trace
 * at the 8 kHz of the PCMU wire, with the levels of its capture. The tables
 * are in lec_paths_field.h. src/test/tools/path_fit.py fits them, and
 * src/test/tools/lec_paths_field_gen.py writes that header from them.
 */
#define FIELD_PATH_RATE		8000u
#define FIELD_PATH_TAPS		512u

_Static_assert(2u * FIELD_PATH_TAPS <= LEC_PATH_TAPS_MAX,
	       "a field path interpolated to 16 kHz must fit in an echo path");

/*
 * The levels are on the far-active frames of the capture, in wire units. The
 * ERLE figures are those of an ideal filter of 48 or 64 ms, with its first
 * tap 2 ms before the onset or at the onset. residual_rms is the rms of what
 * the 64 ms fit from 2 ms before the onset leaves: the near-end floor plus the
 * echo that the fit does not model. The bench injects it as the floor. name
 * has 13 characters at most.
 */
struct field_path {
	const char *name;
	const char *platform;
	const char *backend;
	const char *source;
	unsigned line_rate;
	unsigned onset_tap;
	double onset_ms;
	double uplink_db;
	double far_rms;
	double residual_rms;
	double erle_48_db;
	double erle_48_onset_db;
	double erle_64_db;
	double erle_64_onset_db;
	const float *h;
};

/*
 * An odd tap of the interpolation by two: a half-band lowpass at 4 kHz, the
 * ideal response sampled at the odd lags of 16 kHz under a Blackman window in
 * the cos(pi j / 64) form: a 129-point window, cut at its 127 inner lags. The
 * sign alternates with the lag. It is an explicit flip, the rule
 * of path_edge_cos(), and not the sign of sin(). The table is zero outside
 * its taps, 512 for a field path.
 */
static inline float path_odd_tap_get(const float *h8, unsigned taps,
				     unsigned k)
{
	double sign = 1.0;
	double acc = 0.0;
	unsigned i;

	for (i = 0; i < 32u; i++, sign = -sign) {
		double j = 2.0 * i + 1.0;
		double w = 0.42 + 0.5 * cos(M_PI * j / 64.0) +
			   0.08 * cos(2.0 * M_PI * j / 64.0);
		unsigned after = k + 1u + i;
		double left = k >= i && k - i < taps ? h8[k - i] : 0.0;
		double right = after < taps ? h8[after] : 0.0;

		acc += sign / (M_PI * j) * w * (left + right);
	}
	return (float)acc;
}

static inline float field_odd_tap_get(const float *h8, unsigned k)
{
	return path_odd_tap_get(h8, FIELD_PATH_TAPS, k);
}

/*
 * At 8 kHz the path is the table. At 16 kHz the path is the table
 * interpolated by two. The filter has zero phase, so tap 2k stays on tap k
 * and the onset keeps its place. The band above 4 kHz stays empty, because
 * the PCMU wire carried nothing there. The factor 0.5 on the even taps keeps
 * the response in the band, since the 16 kHz path has two taps per table tap.
 * Any other rate builds nothing, as path_upper_init does.
 *
 * The gain stays that of the table. The taps are the measured path in wire
 * units. path_erl_set fixes a loss for white noise, which the capture did not
 * measure, and at 16 kHz it would put back the 3 dB that the band limit takes
 * out.
 */
static inline void path_field_init(struct echo_path *path, unsigned rate,
				   const struct field_path *table)
{
	unsigned k;

	path_geometry_set(path, rate);
	if (rate == FIELD_PATH_RATE) {
		path->taps = FIELD_PATH_TAPS;
		memcpy(path->h, table->h, FIELD_PATH_TAPS * sizeof(*path->h));
		return;
	}
	if (rate != 2u * FIELD_PATH_RATE)
		return;
	path->taps = 2u * FIELD_PATH_TAPS;
	for (k = 0; k < FIELD_PATH_TAPS; k++) {
		path->h[2u * k] = 0.5f * table->h[k];
		path->h[2u * k + 1u] = field_odd_tap_get(table->h, k);
	}
}

/*
 * The echo path models m1, m4, m6, m7 and m8 of ITU-T G.168 (12/2022) Annex D
 * (Tables D.2, D.5, D.7, D.8 and D.9 at 8 kHz, read in columns), as int32,
 * because some values of m6 and m8 exceed int16. A model at erl_db is Ki
 * 10^(-erl_db / 20) m(k), with Ki and the least ERL of Table D.1a.
 */
static const int32_t annex_d_m1[64] = {
	-436, -829, -2797, -4208, -17968, -11215, 46150, 34480, -10427, 9049,
	-1309, -6320, 390, -8191, -1751, -6051, -3796, -4055, -3948, -2557,
	-3372, -1808, -2259, -1300, -1098, -618, -340, -61, 323, 419, 745, 716,
	946, 880, 1014, 976, 1033, 1091, 1053, 1042, 794, 831, 899, 716, 390,
	313, 304, 304, 73, -119, -109, -176, -359, -407, -512, -580, -704,
	-618, -685, -791, -772, -820, -839, -724,
};
static const int32_t annex_d_m4[128] = {
	160, 312, -241, -415, 897, 908, -1326, -1499, 2405, 3347, -3624, -7733,
	4041, 14484, -1477, -21739, -4470, 25356, 11458, -19696, -11800, 5766,
	789, 6633, 14624, -6975, -17156, -187, 149, 1515, 14907, 4345, -7128,
	-2757, -10185, -7083, 6850, 3944, 6969, 8694, -4068, -3852, -5793,
	-9371, 453, 1060, 3965, 9463, 2393, 2784, -892, -7366, -3376, -5847,
	-2399, 3011, 1537, 6623, 4205, 1602, 1592, -4752, -3646, -5207, -5577,
	-501, -1174, 4041, 5647, 4628, 7252, 2123, 2654, -881, -4113, -3244,
	-7289, -3830, -4600, -2508, 431, -144, 4184, 2372, 4617, 3576, 2382,
	2839, -404, 539, -1803, -1401, -1705, -2269, -783, -1608, -220, -306,
	257, 615, 225, 561, 8, 344, 127, -57, 182, 41, 203, -111, 95, -79, 30,
	84, -13, -68, -241, -68, -24, 19, -57, -24, 30, -68, 84, -155, -68, 19,
};
static const int32_t annex_d_m6[120] = {
	29, 109, -83, 198, -294, -135, -415, -202, -444, -337, -313, -450,
	-105, -503, 145, -490, 267, -231, 340, 77, 343, 783, 158, 1341, 195,
	1798, 344, 1845, 629, 1604, 1182, 940, 5163, 19522, 8421, -50953,
	-9043, 18046, -13553, 13336, -3471, -107, 1788, -7409, 2469, -7994,
	490, -3860, -837, 490, -636, 3682, 1141, 5019, 2635, 5025, 3946, 4414,
	4026, 3005, 3380, 1616, 2007, 158, 388, -1198, -1117, -2134, -2547,
	-2589, -3310, -2778, -3427, -2779, -3116, -2502, -2399, -1956, -1539,
	-1239, -570, -377, 251, 331, 964, 1177, 1449, 1564, 1724, 1871, 1767,
	1802, 1630, 1632, 1379, 1271, 1063, 856, 711, 482, 289, 54, -137, -321,
	-490, -638, -764, -836, -800, -859, -838, -837, -834, -740, -673, -581,
	-493, -436, -327, -201,
};
static const int32_t annex_d_m7[96] = {
	258, -111, 337, -319, 347, -434, 192, -450, -108, -343, -596, -177,
	-1187, -52, -1781, -147, -1959, -326, -1601, -1389, -13620, -720,
	33818, -10683, -6742, 12489, -9862, 8950, -1574, 758, 3526, -3118,
	2421, -8966, -4901, 11385, 18072, -14410, -7473, 19836, -16854, -3115,
	9483, -17799, 7399, -4342, -7415, 7929, -10726, 6239, -2526, -1317,
	5345, -4565, 6868, -2195, 3425, 1969, -109, 3963, -1275, 3087, -892,
	1239, 2, -427, 596, -1184, 551, -1244, 141, -743, -415, -372, -769,
	-183, -785, -270, -659, -377, -523, -325, -245, -255, -60, 35, 218,
	149, 340, 233, 365, 303, 251, 230, 209, 179,
};
static const int32_t annex_d_m8[99] = {
	80, 31, 4, 42, 42, -61, -81, -64, -121, -102, -26, 1002, -9250, -22562,
	39321, 35681, -35289, 25312, -1457, -229, 15659, -6786, 16791, 3860,
	2239, -28730, -11885, 33871, -176, -16421, 18173, -9669, -10163, 9941,
	-19365, 3592, -5907, -10257, 5336, -12933, 4348, -4802, -1791, 3035,
	-4433, 5553, -2596, 3992, 1255, 1450, 4079, 324, 4340, 1059, 3083,
	1917, 1756, 2478, 1027, 1871, 845, 1284, 813, 806, 869, 471, 646, 438,
	449, 432, 473, 394, 452, 538, 717, 723, 850, 756, 753, 899, 555, 669,
	619, 500, 650, 615, 516, 492, 427, 291, 356, 147, 107, -50, -88, -59,
	-238, -165, -183,
};

enum annex_d_model {
	ANNEX_D_M1,
	ANNEX_D_M4,
	ANNEX_D_M6,
	ANNEX_D_M7,
	ANNEX_D_M8,
	ANNEX_D_MODELS,
};

struct annex_d_table {
	const char *name;
	const int32_t *m;
	unsigned taps;
	double ki;
	double erl_min_db;
};

static const struct annex_d_table annex_d_tables[ANNEX_D_MODELS] = {
	{ "m1", annex_d_m1, 64u, 1.39e-5, 6.0 },
	{ "m4", annex_d_m4, 128u, 1.77e-5, 6.0 },
	{ "m6", annex_d_m6, 120u, 1.51e-5, 6.0 },
	{ "m7", annex_d_m7, 96u, 2.33e-5, 11.06 },
	{ "m8", annex_d_m8, 99u, 1.33e-5, 9.27 },
};

/*
 * Model m of Annex D at erl_db after a pure delay of delay_ms, at the line
 * rate. At 16 kHz the 8 kHz response, delay included, is interpolated by two
 * as path_field_init does, so the band above 4 kHz stays empty and the half
 * band rings on both sides of each tap. An ERL under the least ERL of the
 * model or a rate other than 8 or 16 kHz builds nothing.
 */
static inline void path_annex_d_init(struct echo_path *path, unsigned rate,
				     enum annex_d_model m, double erl_db,
				     unsigned delay_ms)
{
	const struct annex_d_table *t = &annex_d_tables[m];
	float h8[LEC_PATH_TAPS_MAX / 2u];
	unsigned delay = path_samples_get(delay_ms, UDSP_SAMPLE_RATE);
	unsigned n = delay + t->taps;
	double scale = t->ki * pow(10.0, -erl_db / 20.0);
	unsigned k;

	path_geometry_set(path, rate);
	if (erl_db < t->erl_min_db - 1e-9 || n + 32u > LEC_PATH_TAPS_MAX / 2u)
		return;
	memset(h8, 0, sizeof(h8));
	for (k = 0; k < t->taps; k++)
		h8[delay + k] = (float)(scale * t->m[k]);
	if (rate == UDSP_SAMPLE_RATE) {
		path->taps = n;
		memcpy(path->h, h8, n * sizeof(*h8));
		return;
	}
	if (rate != UDSP_SAMPLE_RATE_WB)
		return;
	path->taps = 2u * (n + 32u);
	for (k = 0; k < n + 32u; k++) {
		path->h[2u * k] = 0.5f * h8[k];
		path->h[2u * k + 1u] = path_odd_tap_get(h8, n, k);
	}
}

#endif
