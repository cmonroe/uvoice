// SPDX-License-Identifier: LGPL-2.1-only
/*
 * udsp - software voice DSP for FXS lines
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * lec_hz: the canceller harness. It includes a copy of a canceller source
 * whole and runs one instance frame by frame over a stimulus:
 *
 *   lec_hz <dir> <rate> <ref_loss_half_db> <nlp> [<outdir>]
 *
 * <dir> holds ref.s16 and mic.s16, int16 little-endian at the line rate.
 * The two files hold one even count of bytes, and each must read whole. The
 * instance takes the default tail of the rate, the reference loss in half-dB
 * steps (0 sets none) and the processor switch (0 or 1). The program writes
 * <outdir>/frames.csv, one row per frame, and <outdir>/out.s16, the
 * canceller output. <outdir> defaults to <dir>. The exit status is 0, 2 for
 * a usage error, 3 for an input or output error and 4 when the instance
 * cannot be created.
 *
 * patch_hz.py writes the copy into the build directory and adds two hooks
 * to ec_run: hz_pre before the frame's filter pass and hz_post after
 * reading_fill. The build passes the path of the copy in LEC_HZ_C. The
 * hooks read private state of dsp/lec.c by name, so a change of a name
 * there stops the build of this program, not a run.
 *
 * The columns of frames.csv. A power is a mean square in int16 units. A
 * dBov figure uses the scale of udsp_cn_level_get in dsp/cn.c, where a
 * larger number is quieter and 127 stands for no level:
 * - frame: the frame count after the frame, 1 on the first;
 * - mic, ref, win, est, res, bar: the powers of struct lec_tick
 *   (mic_power, ref_power, window_power, estimate_power, res_power, bar);
 * - classic: the classic bound, echo_bound_factor_get times win;
 * - noise, noise_dbov: the near-end floor as a power and in dBov, and erle:
 *   the ERLE of the belief in dB;
 * - wact, act, bg, talker, fired: window_active, active, background,
 *   talker and dt_fired of the frame;
 * - hang, dt_run, stale_run, path_run, cohere_run: the hangover and the
 *   runs of the trial arms after the frame;
 * - trusted, trust_run, trust_rule, classic_arm: the trust of the frame;
 * - adapt0, adapt: the adapt verdict before the filter pass and after the
 *   rollback; probing, arm, end, probe_frames, bare: the trial state, with
 *   arm and end as the numbers of UDSP_EC_NLMS_TRIAL_ARM_* and
 *   UDSP_EC_NLMS_TRIAL_END_*;
 * - coef, peak, gain: the coefficient energy, its peak tap and echo_gain;
 * - step_scale, talk_scale, step_floor, hist: the step factors, the
 *   denominator floor and the history energy of the filter pass;
 * - no_echo: the frame carries no echo for the floor of its bars
 *   (bar_floor_get); rise_n, rise_age, creep: the floor tracker's rise run
 *   (struct noise_run: frames, age, creep);
 * - verdict, live_res, live_mic, nlp_act, level: the processor verdict
 *   (enum nlp_verdict), the smoothed powers, the processor state and the
 *   level in dBov of the output before the processor;
 * - ncc, share: the coherence readings of cohere_ncc_get, 9 and -1 when
 *   the sums cannot give them;
 * - guard, talker_ratio: the larger window of the frame and the frame
 *   before, and the talker ratio of the instance.
 */
#include <stdio.h>
#include <stdlib.h>

#include LEC_HZ_C

#define HZ_NCC_NONE	9.0f
#define HZ_SHARE_NONE	-1.0f

static FILE *hz_csv;
static bool hz_adapt0;
static float hz_mu_scale;
static float hz_talk;
static float hz_step_floor;
static double hz_hist;
static float hz_win_last;
static float hz_guard;

static void hz_pre(struct udsp_ec *ec, struct lec_tick *t, bool adapt,
		   float step_floor)
{
	hz_adapt0 = adapt;
	hz_mu_scale = step_scale_get(ec);
	hz_talk = talk_scale_get(ec, t->mic_power, t->window_power);
	hz_step_floor = step_floor;
	hz_hist = ec->hist_energy;
	hz_guard = hz_win_last > t->window_power ? hz_win_last :
		   t->window_power;
}

static void hz_post(struct udsp_ec *ec, struct lec_tick *t, bool adapt)
{
	unsigned peak = 0;
	float energy = coef_energy_get(ec, &peak);
	float classic = echo_bound_factor_get(ec) * t->window_power;
	float ncc = 0.0f;
	float share = HZ_SHARE_NONE;

	if (!cohere_ncc_get(ec, &ncc, &share)) {
		ncc = HZ_NCC_NONE;
		share = HZ_SHARE_NONE;
	}
	hz_win_last = t->window_power;
	if (!hz_csv)
		return;
	fprintf(hz_csv,
		"%u,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.3f,%.4f,"
		"%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,"
		"%d,%u,%d,%d,"
		"%d,%d,%d,%d,%d,%u,%d,"
		"%.6g,%u,%.6g,%.5g,%.5g,%.6g,%.6g,%d,%u,%u,%u,%d,"
		"%.6g,%.6g,%d,%d,%.4f,%.4f,%.6g,%.6g\n",
		ec->frames, t->mic_power, t->ref_power, t->window_power,
		t->estimate_power, t->res_power, t->bar, classic,
		ec->noise_power, ec->noise_dbov, ec->belief.erle_db,
		t->window_active, t->active, t->background, t->talker,
		t->dt_fired, ec->belief.dt_hangover, ec->dt_run, ec->stale_run,
		ec->path_run, ec->cohere_run,
		t->trusted, ec->belief.trust_run, ec->belief.trust_rule,
		t->classic_arm,
		hz_adapt0, adapt, t->probing, ec->reading.trial_arm,
		ec->reading.trial_end, ec->probe_frames, ec->probe_bare,
		energy, peak, ec->belief.echo_gain, hz_mu_scale, hz_talk,
		hz_step_floor, hz_hist, t->no_echo, ec->noise_rise.frames,
		ec->noise_rise.age, ec->noise_rise.creep, (int)t->nlp_verdict,
		ec->live_res_power, ec->live_mic_power, ec->nlp_active,
		t->level, ncc, share, hz_guard, ec->talker_ratio);
}

/* The samples of dir/name, or NULL when the file does not open, holds an
 * odd count of bytes or does not read whole. */
static int16_t *pcm_read(const char *dir, const char *name, size_t *n)
{
	char path[1024];
	int16_t *buf = NULL;
	FILE *f;
	long len;

	snprintf(path, sizeof(path), "%s/%s", dir, name);
	f = fopen(path, "rb");
	if (!f)
		return NULL;
	if (fseek(f, 0, SEEK_END) == 0 && (len = ftell(f)) >= 0 &&
	    len % (long)sizeof(*buf) == 0 && fseek(f, 0, SEEK_SET) == 0)
		buf = malloc((size_t)len + sizeof(*buf));
	if (buf) {
		*n = (size_t)len / sizeof(*buf);
		if (fread(buf, sizeof(*buf), *n, f) != *n || ferror(f)) {
			free(buf);
			buf = NULL;
		}
	}
	fclose(f);
	return buf;
}

static FILE *out_open(const char *dir, const char *name, const char *mode)
{
	char path[1024];

	snprintf(path, sizeof(path), "%s/%s", dir, name);
	return fopen(path, mode);
}

static void csv_head_write(FILE *f)
{
	fprintf(f, "frame,mic,ref,win,est,res,bar,classic,noise,noise_dbov,"
		"erle,wact,act,bg,talker,fired,hang,dt_run,stale_run,path_run,"
		"cohere_run,trusted,trust_run,trust_rule,classic_arm,adapt0,"
		"adapt,probing,arm,end,probe_frames,bare,coef,peak,gain,"
		"step_scale,talk_scale,step_floor,hist,no_echo,rise_n,rise_age,"
		"creep,verdict,live_res,live_mic,nlp_act,level,ncc,share,guard,"
		"talker_ratio\n");
}

int main(int argc, char **argv)
{
	const char *dir;
	const char *odir;
	unsigned rate;
	unsigned loss;
	unsigned frame;
	unsigned frames;
	unsigned i;
	size_t nr = 0;
	size_t nm = 0;
	int16_t *ref;
	int16_t *mic;
	struct udsp_ec *ec;
	FILE *fo;
	int rc = 0;

	if (argc < 5 || argc > 6) {
		fprintf(stderr, "usage: lec_hz <dir> <rate> "
			"<ref_loss_half_db> <nlp> [<outdir>]\n");
		return 2;
	}
	dir = argv[1];
	rate = (unsigned)strtoul(argv[2], NULL, 10);
	loss = (unsigned)strtoul(argv[3], NULL, 10);
	odir = argc > 5 ? argv[5] : dir;
	ref = pcm_read(dir, "ref.s16", &nr);
	mic = pcm_read(dir, "mic.s16", &nm);
	if (!ref || !mic || nr != nm) {
		fprintf(stderr, "lec_hz: %s: no ref.s16 and mic.s16 of one "
			"even length, read whole\n", dir);
		free(ref);
		free(mic);
		return 3;
	}
	ec = udsp_ec_nlms.create_rate(0, rate);
	if (!ec)
		return 4;
	udsp_ec_nlms.nlp(ec, atoi(argv[4]) != 0);
	if (loss)
		udsp_ec_nlms_ref_loss_set(ec, loss);
	hz_csv = out_open(odir, "frames.csv", "w");
	if (!hz_csv) {
		udsp_ec_nlms.destroy(ec);
		return 3;
	}
	csv_head_write(hz_csv);
	frame = rate / 100u;
	frames = (unsigned)(nr / frame);
	for (i = 0; i < frames; i++)
		udsp_ec_nlms.run(ec, mic + (size_t)i * frame,
				 ref + (size_t)i * frame);
	if (fclose(hz_csv) != 0)
		rc = 3;
	fo = out_open(odir, "out.s16", "wb");
	if (!fo || fwrite(mic, sizeof(*mic), (size_t)frames * frame, fo) !=
	    (size_t)frames * frame)
		rc = 3;
	if (fo && fclose(fo) != 0)
		rc = 3;
	udsp_ec_nlms.destroy(ec);
	free(ref);
	free(mic);
	return rc;
}
