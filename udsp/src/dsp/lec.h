// SPDX-License-Identifier: LGPL-2.1-only
/*
 * udsp - software voice DSP for FXS lines
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * NLMS line echo canceller: the vtable export and the introspection the
 * engine, the cli and the tests read. include/udsp/ec.h holds the stable
 * behaviour contract and dsp/dsp.h belongs to another part of the library, so
 * the extern for the backend lives here. udsp_codec_builtin and
 * udsp_slic_<name>_descriptor are the same kind of named export for a
 * linked-in backend.
 *
 * create(tail_ms) takes 0 as the contract sentinel for the default tail, the
 * meaning the stable udsp.h gives it at udsp_chan_cfg.ec_tail_ms: 48 ms at
 * 8 kHz and 64 ms at 16 kHz. Any other value outside 16..128 ms clamps, and
 * udsp_ec_nlms_tail_get reports what the instance applied.
 *
 * create_rate(tail_ms, rate_hz) is the same call at an explicit line rate
 * (ec.h). It accepts 8000 and 16000 and returns NULL for every other rate.
 * create(tail_ms) is create_rate(tail_ms, 8000), so the 8 kHz line keeps its
 * behaviour. The frame is rate_hz/100 samples and the tap count is tail_ms
 * times rate_hz/1000. The tail clamps to 16..128 ms at 8 kHz and to 16..64 ms
 * at 16 kHz, because the work per millisecond of wideband tail grounds the
 * ceiling at LEC_TAIL_MS_WB_MAX in dsp/lec.c. A 128 ms request on a wideband
 * line therefore reads back as 64 ms with 1024 taps.
 *
 * freeze(on) holds adaptation and nothing else. It never changes the NLP
 * state, and it is not the fax bypass: the engine bypasses this canceller for
 * a fax profile.
 *
 * Rules.
 * 1. create and destroy run on the control path and may allocate.
 * 2. run, freeze and nlp run on the engine tick. They never allocate, never
 *    take a lock and never log.
 * 3. The introspection calls read plain state, so the thread that owns the
 *    instance can call them safely.
 */
#ifndef UDSP_DSP_LEC_H
#define UDSP_DSP_LEC_H

#include <stdbool.h>
#include <stdint.h>

#include "udsp/ec.h"

/*
 * The mean square of a frame, a float sum of float squares over the count.
 * It is the power that every rule of the canceller reads and that the frame
 * reading reports, so the tests and the replay take it from here and read
 * what the canceller reads, to the bit. n is never 0 on a caller's path.
 */
static inline float udsp_ec_nlms_frame_power_get(const int16_t *samples,
						 unsigned n)
{
	float energy = 0.0f;
	unsigned i;

	for (i = 0; i < n; i++)
		energy += (float)samples[i] * (float)samples[i];
	return energy / n;
}

/*
 * What the instance applied and what it sees. The counters describe one
 * alignment of the reference: the reset zeroes frames and the processor
 * counters together.
 * Like struct udsp_ec_nlms_frame, it is not part of the stable record.
 */
struct udsp_ec_nlms_info {
	/* The clamped tail: a request outside the range reads back what it got,
	 * because a silent clamp hides a request the instance did not honor. */
	unsigned tail_ms;
	unsigned taps;
	uint32_t frames;
	/* Frames whose update did not stand: a silent reference, freeze or
	 * double talk. */
	uint32_t hold_frames;
	uint32_t diverge_resets;
	/* Trials run to escape a hold that no verdict ends: the latch, the
	 * path-change, over-estimate and coherence escapes, and the stale
	 * run. */
	uint32_t probes;
	/* The filter energy that the watchdog bounds; a converged instance
	 * holds it at the echo return loss of the hybrid. */
	float coef_energy;
	float erle_db;
	/* The learned lower bound of the hybrid's power gain,
	 * LEC_GEIGEL_GAIN_MIN over the reference loss until learned; times the
	 * headroom, the bound of the detector and of the processor gate. */
	float echo_gain;
	bool double_talk;
	bool nlp_active;
	/* Frames whose tap loops did not run on a zero window (frame_zero_run);
	 * each is also a hold frame, so the two tell digital silence from a
	 * refusal. */
	uint32_t skip_frames;
	/* The frames that read nlp_active: the frame the processor engages on
	 * and each eligible frame after it. With the next five and the frames
	 * with the processor off, the counters add up to frames. */
	uint32_t nlp_active_frames;
	/* The eligible frames before the engage run completes. */
	uint32_t nlp_wait_frames;
	/* Frames refused on either arm of the ERLE gate, the last condition. */
	uint32_t nlp_erle_frames;
	/* Frames whose microphone the echo estimate does not explain. */
	uint32_t nlp_unlike_frames;
	uint32_t nlp_dt_frames;
	/* Frames of a quiet reference window, the first condition, whether they
	 * keep the engage run or not. */
	uint32_t nlp_quiet_frames;
	/* The tap of the largest magnitude, from tap 0 of the applied
	 * reference; 0 while coef_energy reads 0, so read the two together. */
	unsigned coef_peak_tap;
	/* The tracked near-end floor in the dBov of udsp_cn_level_get, a larger
	 * number quieter; 127 while no floor is measured (far pauses or the
	 * seed). */
	float noise_dbov;
	/* The nlp_dt_frames on which the detector fired; the rest is the
	 * hangover behind a verdict. */
	uint32_t nlp_dt_fired_frames;
	/* The far-end floor after the last frame joined it, a mean square in
	 * int16 units (ref_floor_get); 0 until the history is full; the reset
	 * keeps it. */
	float ref_floor;
};

/* New values go at the end only, so a copy of dsp/lec.c built against this
 * header (the UDSP_REPLAY_LEC_C path of lec_replay) keeps its words. */
enum udsp_ec_nlms_trial_arm {
	UDSP_EC_NLMS_TRIAL_ARM_NONE,
	UDSP_EC_NLMS_TRIAL_ARM_LATCH,
	UDSP_EC_NLMS_TRIAL_ARM_PATH,
	UDSP_EC_NLMS_TRIAL_ARM_STALE,
	UDSP_EC_NLMS_TRIAL_ARM_COHERE,
};

enum udsp_ec_nlms_trial_end {
	UDSP_EC_NLMS_TRIAL_END_NONE,
	UDSP_EC_NLMS_TRIAL_END_KEPT,
	UDSP_EC_NLMS_TRIAL_END_REJECTED,
	UDSP_EC_NLMS_TRIAL_END_DIVERGED,
	UDSP_EC_NLMS_TRIAL_END_TALKER,
};

/*
 * The reading of the detector and the processor on the last frame of run(), for
 * the verdict trace of core/chan.c. It is not part of the stable record, and
 * the reset clears it. Powers are mean squares in int16 units. The detector
 * fires when the window is active and mic_power reaches bar, which the reader
 * divides by, so the tick divides nothing.
 */
struct udsp_ec_nlms_frame {
	/* The frame count of the instance after this frame. */
	uint32_t frames;
	/* ref_power is the frame's own reference, window_power the loudest
	 * frame of the window, bar the echo expectation with every term times
	 * the margin. */
	float mic_power;
	float window_power;
	float estimate_power;
	float bar;
	/* The tracked near-end floor that the bar carries, or the pause level
	 * on a frame without echo (bar_floor_get). */
	float noise_power;
	float ref_power;
	/* The tracked ERLE that the detector read; the frame can move the
	 * tracker after the verdict. */
	float erle_db;
	/* The activity bar: the frame and the window are active at it. It reads
	 * the fixed bar, 2500 less the reference loss, while a trial runs. */
	float ref_bar;
	/* The cap of ref_bar, from the learned gain before the frame ran. */
	float ref_bar_max;
	/* Frames of the active window, 1 on the first, 0 while inactive; this
	 * and the next three saturate at UINT16_MAX. */
	uint16_t window_run;
	/* The latch run at the end of the frame. */
	uint16_t dt_run;
	/* The stale run at the end of the frame (LEC_GEIGEL_STALE_FRAMES). */
	uint16_t stale_run;
	/* The trust count at the end of the frame; the detector read the count
	 * before it, so the trust first holds on the frame after the full
	 * count. */
	uint16_t trust_run;
	/* The hangover before this frame changed it: 0 on a fresh verdict;
	 * after a talker ends a trial, the hangover that the snapshot
	 * restored. */
	uint8_t hold_left;
	/* The processor verdict (udsp_ec_nlms_verdict_name); a trial that ends
	 * in the frame can clear double talk before the gate reads it. */
	uint8_t nlp_verdict;
	/* The way into a trial that starts on this frame, else NONE. */
	uint8_t trial_arm;
	/* The end of a trial on this frame: KEPT or REJECTED by its verdict,
	 * DIVERGED by the watchdog, TALKER by a talker frame, else NONE. */
	uint8_t trial_end;
	bool dt_fired;
	/* The classic bound set the bar: the filter is untrusted, and the bound
	 * stands over the estimate. */
	bool classic_arm;
	/* The trust held: tracked ERLE, energy rule and a complete trust
	 * count. */
	bool trusted;
	/* A trial ran when the frame started; the detector ran under it, except
	 * after a talker ended the trial on the frame, when it ran with
	 * none. */
	bool probing;
	/* The window is active and the frame's own reference is not: the tail
	 * of a talkspurt. */
	bool ref_quiet;
	/* The window is active and mic_power exceeds the guard window by
	 * LEC_TALKER_RATIO, which the reference loss raises. */
	bool talker;
	/* The far-end floor alone holds the window inactive
	 * (background_get). */
	bool background;
	/* The drop hold runs at the end of the frame (LEC_DROP_HOLD_DB). */
	bool drop_hold;
	/* The frame starts an act, and its cross-fade starts from the input
	 * before the filter, not from the filter output. */
	bool fade_input;
};

/*
 * The form of the per-tap loops: scalar, canonical on every target, or the
 * Advanced SIMD form, the default on aarch64 and absent elsewhere. The vector
 * dot product reassociates its sum, so the forms are equivalent, not identical;
 * test/test_lec.c bounds the difference.
 */
enum udsp_ec_nlms_path {
	UDSP_EC_NLMS_PATH_SCALAR,
	UDSP_EC_NLMS_PATH_SIMD,
};

extern const struct udsp_ec_ops udsp_ec_nlms;

unsigned udsp_ec_nlms_tail_get(const struct udsp_ec *ec);
void udsp_ec_nlms_info_get(const struct udsp_ec *ec,
			   struct udsp_ec_nlms_info *info);
void udsp_ec_nlms_frame_get(const struct udsp_ec *ec,
			    struct udsp_ec_nlms_frame *frame);
/* The word of a processor verdict (off, quiet, dt, unlike, erle, pass), and
 * "?" for a value that is not a verdict. */
const char *udsp_ec_nlms_verdict_name(unsigned verdict);
/*
 * Full adaptive-state reset for a recalibration of the loop delay, when the
 * coefficients stand misaligned against the re-timed reference. Tail, switches,
 * freeze, comfort noise sequence and both floors stay; the interpolator starts
 * over, and the bootstrap of a run instance stays spent. RT-safe.
 */
void udsp_ec_nlms_reset(struct udsp_ec *ec);

/*
 * One frame of reference into the history, as run() takes it, for a frame that
 * the tick skips; the far-end floor is the one tracker it feeds. A filter that
 * does not see the downlink resumes on a stale window and adapts against it
 * RT-safe on the terms of run().
 */
void udsp_ec_nlms_ref_feed(struct udsp_ec *ec, const int16_t *ref);

/* The reach of the engine's receive loss, 30 dB in half-dB steps (core/udsp.c
 * UDSP_SLIC_GAIN_MIN); a larger loss drives the scaled rules out of range. */
#define UDSP_EC_NLMS_REF_LOSS_MAX	60u
/*
 * The loss in half-dB steps that the engine applies to the downlink ahead of
 * the reference; the line still carries it. Each absolute rule scales by it; a
 * loss of 0 changes no rule, bit for bit. A loss over UDSP_EC_NLMS_REF_LOSS_MAX
 * clamps to it, and the same loss again changes nothing. A changed loss resets
 * the instance and its far-end floor, not the loss. RT-safe.
 */
void udsp_ec_nlms_ref_loss_set(struct udsp_ec *ec, unsigned half_db);
unsigned udsp_ec_nlms_ref_loss_get(const struct udsp_ec *ec);

/*
 * The process-wide path switch, for the tests and the bench; production never
 * calls it. It takes effect at the next per-tap loop and needs quiescence: no
 * thread inside run(). It is RT-safe, not thread-safe. path_set returns false
 * for a path the build lacks; path_name returns NULL for a non-path.
 */
bool udsp_ec_nlms_path_set(enum udsp_ec_nlms_path path);
enum udsp_ec_nlms_path udsp_ec_nlms_path_get(void);
const char *udsp_ec_nlms_path_name(enum udsp_ec_nlms_path path);

/*
 * The zero-reference skip switch of one instance, for the tests and the bench:
 * on by default and kept by the reset. On, a frame whose window is zero skips
 * the tap loops and counts in skip_frames, with the same samples and state bit
 * for bit. A NULL instance reads off and takes no write.
 */
void udsp_ec_nlms_zero_skip_set(struct udsp_ec *ec, bool on);
bool udsp_ec_nlms_zero_skip_get(const struct udsp_ec *ec);

/*
 * The far-end floor switch of one instance, on the terms of the zero-skip
 * switch above. On, the activity bar of a frame follows the far-end floor
 * (dsp/lec.c, ref_bar_get); off, it is the fixed bar. The floor itself runs
 * either way, so a change of the switch moves the bar on the next frame.
 * test_lec runs twin instances with the switch on and off.
 */
void udsp_ec_nlms_ref_floor_set(struct udsp_ec *ec, bool on);
bool udsp_ec_nlms_ref_floor_get(const struct udsp_ec *ec);

#endif
