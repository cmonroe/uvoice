// SPDX-License-Identifier: LGPL-2.1-only
/*
 * udsp - software voice DSP for FXS lines
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * lec_tone_sweep: the tone line of cohere_tone_test in test/test_lec.c with
 * its stimulus as arguments, for a sweep over canceller variants:
 *
 *   lec_tone_sweep <rate> <digit_dbm0> <floor_dbm0> <ulaw> <lead> <seed>
 *                  <hybrid> <dial_dbm0> <trials> <line_dbm0> [<share>]
 *
 * - digit_dbm0: the level of each tone of a near DTMF digit;
 * - floor_dbm0: the far white floor under the dial tone, -200 for none;
 * - ulaw: 1 gives the far drive a G.711 mu-law round trip;
 * - lead: the samples of a digit in its onset frame;
 * - seed: the seed of the far floor, a C integer constant;
 * - hybrid: 0 flat 8 ms, 1 dispersive, 2 composite, each at TONE_ERL_DB;
 * - dial_dbm0: the level of each tone of the dial tone;
 * - trials: the trials after the 2 s of dial tone that train the filter;
 * - line_dbm0: a white line noise on the microphone, -200 for none;
 * - share: 1 also reads the two-peak share of the reference on the frames
 *   of the runs that the readings of the escape rebuild.
 *
 * The program prints one line: the arguments, then after "|" the coherence
 * trials, the longest run of the readings, the digit frames read and the
 * worst digit tone loss in dB, as tone_line_run reads them. With share, a
 * second "|" part gives the minimum and the median share over those frames,
 * the shares at 0.995 and at 0.999 or more, and the frame count.
 *
 * patch_sweep.py builds the program: it appends this file to a copy of
 * test/test_lec.c whose main it renames. This file therefore reads the
 * static helpers and constants of that suite.
 */

#define SWEEP_SHARES_MAX	200000u
#define SWEEP_NONE_DBM0		-199.0

struct sweep_cfg {
	unsigned rate;
	double digit_dbm0;
	double floor_dbm0;
	int ulaw;
	unsigned lead;
	uint32_t seed;
	int hybrid;
	double dial_dbm0;
	unsigned trials;
	double line_dbm0;
};

struct sweep_shares {
	double *v;
	unsigned n;
};

/* tone_digit_fill of the suite with the level and the lead of c. */
static int sweep_digit_fill(const struct sweep_cfg *c, int16_t *near,
			    unsigned frame, unsigned trial, unsigned tf,
			    unsigned long n0)
{
	static const double row[4] = { 697.0, 770.0, 852.0, 941.0 };
	static const double col[4] = { 1209.0, 1336.0, 1477.0, 1633.0 };
	double a = dbm0_rms_get(c->digit_dbm0) * sqrt(2.0);
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
			   (long)i - (long)frame + (long)c->lead;
		unsigned long k = n0 + i;

		if (pos < 0 || pos >= (long)(TONE_DIGIT_ON * frame))
			continue;
		near[i] = sample_clamp(a * (sin(2.0 * M_PI * row[digit % 4] *
						 (double)k / c->rate) +
					    sin(2.0 * M_PI *
						col[digit / 4 % 4] *
						(double)k / c->rate)));
	}
	return digit;
}

/*
 * The share of the 200 to 3400 Hz energy of a reference frame in its two
 * strongest spectral peaks, each with two bins on each side, in double
 * precision: the measure of ref_tone_share_get in dsp/lec.c. The frame takes
 * a Hann window and zero padding to 128 points at 8 kHz and 256 at 16 kHz.
 */
static double sweep_share_get(const int16_t *x, unsigned frame,
			      unsigned rate)
{
	double re[256];
	double im[256];
	double p[256];
	unsigned n = 128;
	unsigned lo;
	unsigned hi;
	unsigned i;
	unsigned k;
	unsigned q;
	double total = 0.0;
	double top = 0.0;

	while (n < frame)
		n <<= 1;
	for (k = 0; k < n; k++) {
		re[k] = 0.0;
		im[k] = 0.0;
		for (i = 0; i < frame; i++) {
			double w = 0.5 - 0.5 * cos(2.0 * M_PI * i /
						   (frame - 1));

			re[k] += x[i] * w * cos(2.0 * M_PI * k * i / n);
			im[k] -= x[i] * w * sin(2.0 * M_PI * k * i / n);
		}
	}
	lo = (200u * n + rate - 1u) / rate;
	hi = 3400u * n / rate;
	for (i = lo; i <= hi; i++) {
		p[i] = re[i] * re[i] + im[i] * im[i];
		total += p[i];
	}
	for (q = 0; q < 2u; q++) {
		unsigned best = lo;
		unsigned from;
		unsigned to;

		for (i = lo; i <= hi; i++) {
			if (p[i] > p[best])
				best = i;
		}
		from = best >= lo + 2u ? best - 2u : lo;
		to = best + 2u <= hi ? best + 2u : hi;
		for (i = from; i <= to; i++) {
			top += p[i];
			p[i] = 0.0;
		}
	}
	return total > 0.0 ? top / total : 0.0;
}

static int sweep_double_cmp(const void *a, const void *b)
{
	double x = *(const double *)a;
	double y = *(const double *)b;

	return (x > y) - (x < y);
}

static void sweep_path_init(struct echo_path *path, const struct sweep_cfg *c)
{
	if (c->hybrid == 1)
		path_dispersive_init(path, c->rate, TONE_ERL_DB);
	else if (c->hybrid == 2)
		path_composite_init(path, c->rate, TONE_ERL_DB);
	else
		path_flat_init(path, c->rate, 8, TONE_ERL_DB);
}

/* The digit tone loss of the third and fourth frame of a digit. */
static void sweep_loss_add(const int16_t *out, const int16_t *near,
			   unsigned frame, unsigned rate, int digit,
			   struct tone_result *r)
{
	static const double row[4] = { 697.0, 770.0, 852.0, 941.0 };
	static const double col[4] = { 1209.0, 1336.0, 1477.0, 1633.0 };
	double lo = 10.0 * log10(tone_bin_get(out, frame, row[digit % 4], rate) /
				 tone_bin_get(near, frame, row[digit % 4],
					      rate));
	double hi = 10.0 * log10(tone_bin_get(out, frame, col[digit / 4 % 4],
					      rate) /
				 tone_bin_get(near, frame, col[digit / 4 % 4],
					      rate));

	if (lo < r->worst_db)
		r->worst_db = lo;
	if (hi < r->worst_db)
		r->worst_db = hi;
	r->digits++;
}

/* tone_line_run of the suite under c. With shares, it also records the
 * reference share of each frame that extends a run of the readings. */
static void sweep_run(const struct sweep_cfg *c, struct tone_result *r,
		      struct sweep_shares *shares)
{
	unsigned rate = c->rate;
	unsigned frame = rate / 100u;
	struct udsp_ec *ec = udsp_ec_nlms.create_rate(0, rate);
	struct udsp_ec_nlms_frame reading;
	struct udsp_ec_nlms_info info;
	struct cohere_view view;
	struct echo_path path;
	uint32_t state = c->seed;
	uint32_t line_state = c->seed ^ 0x5a5a5a5au;
	double a = dbm0_rms_get(c->dial_dbm0) * sqrt(2.0);
	unsigned long n = 0;
	unsigned total = TONE_WARM_FRAMES + c->trials * TONE_TRIAL_FRAMES;
	unsigned run = 0;
	unsigned f;

	memset(r, 0, sizeof(*r));
	memset(&view, 0, sizeof(view));
	udsp_ec_nlms.nlp(ec, true);
	udsp_ec_nlms_ref_loss_set(ec, TONE_LOSS_HALF_DB);
	sweep_path_init(&path, c);
	for (f = 0; f < total; f++) {
		int16_t drive[UDSP_FRAME_MAX];
		int16_t floor[UDSP_FRAME_MAX];
		int16_t line[UDSP_FRAME_MAX];
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
		int digit = trial ? sweep_digit_fill(c, near, frame, index, tf,
						     n) : -1;
		double ncc;
		double share;
		unsigned i;

		if (!trial)
			memset(near, 0, sizeof(near));
		memset(floor, 0, sizeof(floor));
		memset(line, 0, sizeof(line));
		if (c->floor_dbm0 > SWEEP_NONE_DBM0)
			noise_fill(floor, frame, c->floor_dbm0, &state);
		if (c->line_dbm0 > SWEEP_NONE_DBM0)
			noise_fill(line, frame, c->line_dbm0, &line_state);
		for (i = 0; i < frame; i++, n++) {
			double v = a * (sin(2.0 * M_PI * TONE_LO_HZ *
					    (double)n / rate) +
					sin(2.0 * M_PI * TONE_HI_HZ *
					    (double)n / rate));

			drive[i] = on ? sample_clamp(v + floor[i]) : 0;
			if (c->ulaw)
				drive[i] = tone_ulaw_get(drive[i]);
		}
		level_loss_apply(ref, drive, frame, TONE_LOSS_HALF_DB);
		path_run(&path, drive, echo);
		for (i = 0; i < frame; i++)
			mic[i] = sample_clamp((double)echo[i] + near[i] +
					      line[i]);
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
			    ncc <= COHERE_BAR && share <= COHERE_SHARE_MAX) {
				run++;
				if (shares && shares->n < SWEEP_SHARES_MAX)
					shares->v[shares->n++] =
						sweep_share_get(ref, frame,
								rate);
			} else {
				run = 0;
			}
			if (run > r->run_max)
				r->run_max = run;
		}
		if (digit >= 0) {
			unsigned in = (tf + 1u - TONE_DIGIT_FROM) %
				      TONE_DIGIT_PERIOD;

			if (in >= 2u && in + 1u < TONE_DIGIT_ON)
				sweep_loss_add(out, near, frame, rate, digit,
					       r);
		}
	}
	udsp_ec_nlms.destroy(ec);
}

static void sweep_shares_print(struct sweep_shares *s)
{
	unsigned at995 = 0;
	unsigned at999 = 0;
	unsigned i;

	qsort(s->v, s->n, sizeof(*s->v), sweep_double_cmp);
	for (i = 0; i < s->n; i++) {
		at995 += s->v[i] >= 0.995;
		at999 += s->v[i] >= 0.999;
	}
	printf(" | run-frame ref share min %.5f med %.5f >=0.995 %.3f "
	       ">=0.999 %.3f n %u", s->n ? s->v[0] : 1.0,
	       s->n ? s->v[s->n / 2u] : 0.0,
	       s->n ? (double)at995 / s->n : 0.0,
	       s->n ? (double)at999 / s->n : 0.0, s->n);
}

int main(int argc, char **argv)
{
	struct sweep_shares shares = { NULL, 0 };
	struct sweep_cfg c;
	struct tone_result r;

	if (argc < 11 || argc > 12) {
		fprintf(stderr, "usage: lec_tone_sweep <rate> <digit_dbm0> "
			"<floor_dbm0> <ulaw> <lead> <seed> <hybrid> "
			"<dial_dbm0> <trials> <line_dbm0> [<share>]\n");
		return 2;
	}
	c.rate = (unsigned)strtoul(argv[1], NULL, 10);
	c.digit_dbm0 = atof(argv[2]);
	c.floor_dbm0 = atof(argv[3]);
	c.ulaw = atoi(argv[4]);
	c.lead = (unsigned)strtoul(argv[5], NULL, 10);
	c.seed = (uint32_t)strtoul(argv[6], NULL, 0);
	c.hybrid = atoi(argv[7]);
	c.dial_dbm0 = atof(argv[8]);
	c.trials = (unsigned)strtoul(argv[9], NULL, 10);
	c.line_dbm0 = atof(argv[10]);
	if (c.rate != 8000u && c.rate != 16000u) {
		fprintf(stderr, "lec_tone_sweep: rate 8000 or 16000\n");
		return 2;
	}
	if (argc > 11 && atoi(argv[11])) {
		shares.v = malloc(SWEEP_SHARES_MAX * sizeof(*shares.v));
		if (!shares.v)
			return 2;
	}
	sweep_run(&c, &r, shares.v ? &shares : NULL);
	printf("rate %u digit %.0f floor %.0f ulaw %d lead %u seed 0x%x "
	       "hybrid %d dial %.0f trials %u line %.0f | arms %u run %u "
	       "digits %u worst %.2f", c.rate, c.digit_dbm0, c.floor_dbm0,
	       c.ulaw, c.lead, (unsigned)c.seed, c.hybrid, c.dial_dbm0,
	       c.trials, c.line_dbm0, r.arms, r.run_max, r.digits,
	       r.worst_db);
	if (shares.v)
		sweep_shares_print(&shares);
	printf("\n");
	free(shares.v);
	return 0;
}
