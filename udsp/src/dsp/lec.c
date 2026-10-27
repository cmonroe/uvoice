// SPDX-License-Identifier: LGPL-2.1-only
/*
 * udsp - software voice DSP for FXS lines
 * Copyright (C) 2026 Adtran Inc.
 * Author: Chad Monroe <chad.monroe@adtran.com>
 *
 * NLMS line echo canceller behind the stable ec.h vtable: one instance per
 * line, running at the line rate with rate/100 samples per frame (ec.h).
 * create_rate() takes 8000 or 16000 and refuses every other rate; create() is
 * the 8 kHz form of it. The caller feeds the DL frame already delayed by the
 * measured loop delay (ec.h). Tap 0 is therefore the first sample of the hybrid
 * response, and the tail counts from there.
 *
 * Every per-frame constant below counts 10 ms frames, not samples, so it holds
 * its meaning at any line rate. The frame is rate/100 samples, and the tick
 * stays 10 ms (core/clock.h).
 *
 * Single precision state, int16 saturating output: the aarch64 targets have a
 * hard FPU. The rounding noise of a full-tail dot product stays far under the
 * deepest level that the canceller must reach.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include "dsp.h"
#include "lec.h"
#include "resamp.h"

#define LEC_TAIL_MS_MIN		16u
#define LEC_TAIL_MS_MAX		128u
/* The analog line profile: EC on with NLP, 48 ms default tail. */
#define LEC_TAIL_MS_DEFAULT	48u
/* The wideband default is the wideband cap. */
#define LEC_TAIL_MS_WB_DEFAULT	64u

/* Wideband tail ceiling. A millisecond of tail at 16 kHz costs four times the
 * work of a narrowband one. 64 ms therefore holds the wideband worst case at
 * twice the narrowband one, under the tick budget's gate. */
#define LEC_TAIL_MS_WB_MAX	64u

/* The ceiling is the wideband tap count, so one instance layout serves both
 * line rates; the window bound divides by the narrowband frame, which is the
 * smallest frame any rate asks for and therefore the upper bound. */
#define LEC_TAPS_MAX		(LEC_TAIL_MS_MAX * (UDSP_SAMPLE_RATE_WB / 1000u))
#define LEC_WINDOW_FRAMES_MAX	((LEC_TAPS_MAX + UDSP_FRAME_SAMPLES - 1u) / \
				 UDSP_FRAME_SAMPLES)

/* NLMS step; NLMS converges for 0 < mu < 2. 0.25 converges fast on a 48 ms
 * tail, and a double-talk frame that the detector misses cannot move the filter
 * far. An escape trial takes a proportionate step (LEC_IPNLMS_ALPHA). */
#define LEC_STEP		0.25f

/* The proportionate step of an escape trial on taps with echo evidence takes
 * one gain per LEC_IPNLMS_BLOCK taps (block_gain_set). Half of it is
 * a uniform share, so a path in empty blocks still learns. */
#define LEC_IPNLMS_ALPHA	0.0f
#define LEC_IPNLMS_BLOCK	32u
/* The cap of a block gain: the denominator stays the unweighted window energy,
 * so the cap bounds the effective step. */
#define LEC_IPNLMS_GAIN_MAX	(1.0f / LEC_STEP)
#define LEC_IPNLMS_BLOCKS_MAX	(LEC_TAPS_MAX / LEC_IPNLMS_BLOCK)

/* One LSB squared per tap floors the NLMS denominator. The tracked near-end
 * floor joins it: a burst onset after silence has little window energy but near
 * noise in its error, in an unthrottled trial too. */
#define LEC_POWER_PER_TAP	1.0f

/* The fixed activity bar, a mean square of about -50 dBm0. Under it the
 * reference carries no usable excitation, so filter, trackers and detector
 * stand still and the floor tracker reads. */
#define LEC_REF_ACTIVE_POWER	2500.0f

/* The far-end floor (struct ref_floor): the lowest frame power of
 * the reference over LEC_REF_FLOOR_SUBS sub-windows of LEC_REF_FLOOR_SUB_FRAMES
 * frames. The activity bar stands over it, except in a trial (ref_bar_get). */
#define LEC_REF_FLOOR_SUB_FRAMES	50u
#define LEC_REF_FLOOR_SUBS	6u
/* 8 dB: the bar clears a stationary far-end background, so the detector does
 * not read near speech over that background as double talk. */
#define LEC_REF_FLOOR_MARGIN	6.3f
/* The mean square of -65 dBm0 (G.168 Figure 9): the raised bar caps where the
 * loudest echo that the classic bound allows returns at that level. */
#define LEC_REF_RETURN_MAX	82.4f

/* The largest gain of the classic bound, a hot hybrid of 6 dB return loss
 * (echo_bound_factor_get); gain_min scales it by the reference loss. */
#define LEC_GEIGEL_GAIN_MIN	0.25f
/* The margin that a near talker adds over the expected echo before the detector
 * fires. */
#define LEC_GEIGEL_MARGIN	1.41f
/* The tracked ERLE from which the estimate alone sets the expected echo, with
 * the energy rule and the trust count (trust_rule_get). */
#define LEC_GEIGEL_TRUST_DB	18.0f
/* 10 dB: the trust needs coefficient energy no lower than the learned gain over
 * this ratio. A filter that learned a tone carries the energy of the tone alone
 * (trust_rule_get). The rule sets no upper bound. */
#define LEC_GEIGEL_TRUST_COEF_RATIO	10.0f
/* The trust count (trust_update) waits for clear active frames since
 * its last restart. A window active after LEC_GEIGEL_TRUST_PAUSE_FRAMES quiet
 * frames, before the trust held, restarts it. */
#define LEC_GEIGEL_TRUST_FRAMES	200u
#define LEC_GEIGEL_TRUST_PAUSE_FRAMES	200u
/* Fired frames that the classic bar can explain, which start a trial when a
 * trusted estimate goes stale (stale_due_get). */
#define LEC_GEIGEL_STALE_FRAMES	200u
/* A verdict run this long escapes into a trial (the latch, probe_arm). */
#define LEC_GEIGEL_LATCH_MAX	500u
/* The long hangover bridges the gaps of a near-end talker. */
#define LEC_GEIGEL_HANGOVER	24u
/* The short hangover: a floor excursion, a verdict on a thin frame, and the
 * second verdict of a thin pair take it (hangover_get). */
#define LEC_GEIGEL_HANGOVER_SHORT	4u
/* 3 dB: a floor excursion stands within this of the bar. */
#define LEC_GEIGEL_MARGINAL_RATIO	2.0f
/* 12 dB: a verdict within this of the tracked floor can be a floor excursion; a
 * microphone this far over it carries the near end. */
#define LEC_GEIGEL_SOFT_RATIO	16.0f
/* A verdict within this many frames of the last one is a repeat (dt_gap). */
#define LEC_GEIGEL_REPEAT_FRAMES	24u
/* A clear frame repays this much of the latch run, which far-end DTX gaps would
 * otherwise reset. */
#define LEC_GEIGEL_RUN_DECAY	4u
/* 6 dB: the bar bound of a thin pair. A tail frame's bar falls with the window
 * maximum (hangover_get). */
#define LEC_GEIGEL_PAIR_RATIO	4.0f
/* 12 dB: a window-active frame whose microphone exceeds the guard window by
 * this is a near talker (talker_get). No path of the
 * suite returns its echo that far over; talker_ratio adds the loss. */
#define LEC_TALKER_RATIO	16.0f
/* The release of the learned gain rises 2 dB a second, and its bound stands at
 * LEC_GEIGEL_GAIN_HEADROOM times the gain. gain_floor scales its floor by the
 * reference loss (echo_gain_update). */
#define LEC_GEIGEL_GAIN_RISE	1.0046f
#define LEC_GEIGEL_GAIN_HEADROOM 2.0f
#define LEC_GEIGEL_GAIN_FLOOR	0.001f
/* The floor terms of the detector's bar and of the echo-likeness test stand
 * this over the tracked floor, which reads under its mean. A quiet frame's test
 * takes LEC_GEIGEL_PAUSE_HEADROOM. */
#define LEC_GEIGEL_FLOOR_HEADROOM 2.0f
#define LEC_GEIGEL_PAUSE_HEADROOM 2.8f
/* The floor of a frame without echo (bar_floor_get): the mean
 * microphone power of the last LEC_PAUSE_RING quiet-window frames. It applies
 * over the tracked floor, capped at LEC_PAUSE_LEVEL_CAP times it. */
#define LEC_PAUSE_RING		4u
#define LEC_PAUSE_LEVEL_CAP	4.0f
/* The learned gain reads a smoothed ratio after a warm-up, because a raw onset
 * frame without its echo spoils a minimum statistic (echo_gain_update). */
#define LEC_GAIN_AVG_ALPHA	0.04f
#define LEC_GAIN_WARMUP_FRAMES	25u
/* The reseeded gain stands at most this times coefficient energy, half the
 * ratio of the trust's energy rule. The rule then holds with 3 dB to spare
 * while the filter moves on after the keep (echo_gain_reseed). */
#define LEC_GAIN_RESEED_COEF_RATIO	(LEC_GEIGEL_TRUST_COEF_RATIO / 2.0f)

/* Smoothing and floor of the ERLE tracker that gates the processor. */
#define LEC_ERLE_ALPHA		0.1f
#define LEC_ERLE_FLOOR		1.0f
/*
 * The processor gate (nlp_gate_get): 18 dB of ERLE, or
 * LEC_NLP_ERLE_MIN_DB with LEC_NLP_LOSS_DB of combined loss. That loss adds the
 * learned gain's hybrid loss with headroom and the reference loss to the ERLE.
 */
#define LEC_NLP_ERLE_DB		18.0f
#define LEC_NLP_ERLE_MIN_DB	6.0f
#define LEC_NLP_LOSS_DB		33.0f
/* The engage run: the processor acts on a pass frame whose run is complete, so
 * a marginal line does not flap the gate (nlp_apply). */
#define LEC_NLP_ON_FRAMES	8u
/* A dt, unlike or erle frame cuts a complete engage run to this. After one
 * refusal inside far speech, the processor acts again on the 4th pass frame.
 * Other refused frames zero the run (nlp_apply). */
#define LEC_NLP_DECAY_FRAMES	4u

/* Until the processor's 12th acting frame after NEW or a reset, an erle frame
 * counts into the engage run. A first engage then holds through a dip of ERLE
 * to the gate at convergence (nlp_engaged). */
#define LEC_NLP_FIRST_FRAMES	12u

/* 3 dB: a quiet frame up to this over the echo-likeness bar is a floor peak and
 * keeps the engage run; a frame over it zeroes the run (nlp_apply). */
#define LEC_NLP_PEAK_RATIO	2.0f

/* Quiet frames in a row over the echo-likeness bar, each a floor peak, that
 * keep the engage run on a line with a measured floor. Any other frame ends the
 * row (nlp_apply). */
#define LEC_NLP_QUIET_MISSES	2u

/* The ERLE from which the first pass frame after a far pause resumes a complete
 * engage run (nlp_resume_get). The burst before needs an act, and
 * the pause holds only floor peaks on a measured floor. */
#define LEC_NLP_RESUME_DB	(LEC_NLP_ERLE_DB + LEC_NLP_ERLE_MIN_DB)
/* Down to 2 dB under LEC_NLP_RESUME_DB, the resume also takes a tracker that
 * climbs or that just dipped. Climb: it reads over its reading
 * LEC_NLP_RESUME_CLIMB_FRAMES frames before. Dip: the burst before the pause
 * reached LEC_NLP_RESUME_DB. A dt, unlike or erle frame that cut the run since
 * the run was last complete refuses both (nlp_resume_erle_get). */
#define LEC_NLP_RESUME_MARGIN_DB	2.0f
#define LEC_NLP_RESUME_CLIMB_FRAMES	100u

/* The first two quiet frames of a pause are its tail frames. Over the peak
 * bound, one refuses the resume only LEC_NLP_TAIL_FLOOR_RATIO over the tracked
 * floor, as echo past the filter's span stands under it (nlp_pause_update).
 * Inside the re-acquisition window both read the implied floor. */
#define LEC_NLP_TAIL_QUIET_FRAMES	2u
#define LEC_NLP_TAIL_FLOOR_RATIO	16.0f

/* The bootstrap: the first LEC_NLP_BOOT_FRAMES active frames of a new instance
 * pass the combined-loss arm at 27 dB, so the first bursts meet G.168 Figure 9.
 * A reset of a run instance spends them (nlp_loss_gate_get). */
#define LEC_NLP_BOOT_LOSS_DB	27.0f
#define LEC_NLP_BOOT_FRAMES	150u

/* Near-end floor tracker in the dBov of dsp/cn.c, a larger number quieter
 * (noise_quiet_update): fast down, slowly up within the rise bar. */
#define LEC_NOISE_ALPHA_DOWN	0.25f
#define LEC_NOISE_ALPHA_UP	0.02f
/* A louder reading joins a rise run; a run of LEC_NOISE_RISE_FRAMES readings
 * within the spread and the age sets the floor to its mean. */
#define LEC_NOISE_RISE_DB	3.0f
#define LEC_NOISE_RISE_FRAMES	50u
#define LEC_NOISE_SPREAD_DB	8.0f
#define LEC_NOISE_RISE_AGE	500u
/* The age bound of a run in the re-acquisition window. A kept escape trial
 * opens that window on a rise run (noise_rise_reacquire). */
#define LEC_NOISE_RISE_AGE_KEPT	700u
/* 12 dB: the most that the floor implied by the rise run at a keep stands over
 * the tracked floor (noise_rise_implied_get). */
#define LEC_NOISE_IMPLIED_CAP_DB	12.0f
/* A reading within LEC_NOISE_RISE_DB of the floor leaves a rise run alone up to
 * 1 in this share of the run's readings (noise_quiet_update). */
#define LEC_NOISE_CREEP_SHARE	16u
/* A residual this much quieter than the floor is evidence of a stale floor. */
#define LEC_NOISE_STALE_DB	3.0f
/* Active frames in a row of that evidence that drop a stale floor: the
 * tracker's own downward time constant, 1 over LEC_NOISE_ALPHA_DOWN. */
#define LEC_NOISE_STALE_FRAMES	4u

/* The agreement run (noise_agree_add) raises the floor where short
 * far pauses never complete a rise run: 32 readings from LEC_NOISE_AGREE_PAUSES
 * quiet windows or more. */
#define LEC_NOISE_AGREE_READINGS	32u
#define LEC_NOISE_AGREE_PAUSES		10u
/* 8 dB: the readings of an agreement run stand within this of each other, and
 * the floor takes their mean plus LEC_NOISE_AGREE_BIAS_DB. */
#define LEC_NOISE_AGREE_DB		8.0f
#define LEC_NOISE_AGREE_BIAS_DB		1.0f
/* 8 dB: the most that one agreement run raises the floor. */
#define LEC_NOISE_AGREE_RISE_DB		8.0f

/* The seed of the floor under a far end that never pauses (noise_seed_update).
 * LEC_NOISE_SEED_FRAMES qualifying active frames in a row set the
 * floor to their lowest power plus LEC_NOISE_SEED_BIAS_DB. */
#define LEC_NOISE_SEED_ERLE_DB	12.0f
#define LEC_NOISE_ECHO_MARGIN	10.0f
#define LEC_NOISE_SEED_FRAMES	20u
#define LEC_NOISE_SEED_BIAS_DB	1.0f

/* Comfort noise plays at the tracked floor plus 1 dB and never louder than the
 * frame it replaces. At 16 kHz it is drawn at 8 kHz and interpolated, in the
 * band of the near-end noise (cn_fill). */
#define LEC_CN_RAISE_DB		1.0f

/* The trial (probe_start) adapts for LEC_PROBE_FRAMES active frames
 * from a snapshot, whatever the detector says. It stands only when live
 * residual over live microphone falls to LEC_PROBE_IMPROVE of its start. */
#define LEC_PROBE_FRAMES	100u
#define LEC_PROBE_IMPROVE	0.5f
/* Frames in a row with the peak tap at or past the frame length clear the onset
 * peak of an untrusted filter (onset_peak_update). */
#define LEC_ONSET_PEAK_CLEAR_FRAMES	8u
/* A zeroed filter's plain onset step (ec_run) reads a denominator
 * floor of this power ratio of its microphone over the taps at least. It
 * stands 30 dB under the microphone. */
#define LEC_ONSET_ZERO_FLOOR	1.0e-3f

/* The path-change escape (path_change_get): under a hold, live
 * residual over live microphone at 1 dB or more on LEC_PATH_CHANGE_FRAMES
 * active frames starts a trial. No talker makes the residual louder. */
#define LEC_PATH_CHANGE_RATIO	1.26f
#define LEC_PATH_CHANGE_FRAMES	20u

/* The over-estimate escape (over_get): a live estimate 4 dB or more
 * over the live microphone on LEC_OVER_FRAMES active frames, after a recent
 * trust, starts a trial. Its taps are scaled to the returned echo. */
#define LEC_OVER_RATIO		2.5119f
#define LEC_OVER_FRAMES		10u
#define LEC_OVER_TRUST_FRAMES	30u
/* The coherence escape (cohere_get): LEC_COHERE_FRAMES held active
 * frames start a trial. Their residual correlates with the estimate at or under
 * the bar, at a low share and on a broadband reference. */
#define LEC_COHERE_BAR		-0.10f
#define LEC_COHERE_FRAMES	32u
/* A frame moves the run only when both energies reach one LSB squared. An
 * onset frame can carry a reference of zero. */
#define LEC_COHERE_POWER_MIN	1.0f
/* The estimate's share of the microphone energy: at or under 0.5 only for a
 * changed path 3 dB louder or more. */
#define LEC_COHERE_SHARE_MAX	0.5f
/* The latch run waits on a frame that reads an independent near talker over a
 * trusted filter (latch_run_grow). Such a frame reads a correlation
 * within the band of 0 and a share of LEC_LATCH_SHARE_MIN or more. */
#define LEC_LATCH_NCC_BAND	(-LEC_COHERE_BAR)
#define LEC_LATCH_SHARE_MIN	0.10f
/* A tonal reference, 99.5 % or more of its energy from LEC_TONE_BAND_LO_HZ to
 * LEC_TONE_BAND_HI_HZ in its two strongest peaks, zeroes a coherence run. One
 * or two tones identify the path only at their frequencies. */
#define LEC_COHERE_TONE_SHARE	0.995f
#define LEC_TONE_BAND_LO_HZ	200u
#define LEC_TONE_BAND_HI_HZ	3400u
#define LEC_TONE_PEAK_BINS	2u
#define LEC_TONE_FFT_MAX	256u
/* A refused escape trial stands its arms down for one latch cycle. One counter
 * and a mask of the held arms serve the escapes and the stale run. A trial
 * carries a running stand-down (probe_start). */
#define LEC_TRIAL_STANDDOWN_FRAMES	(LEC_GEIGEL_LATCH_MAX + LEC_PROBE_FRAMES)
#define LEC_ARM_PATH		(1u << 0)
#define LEC_ARM_STALE		(1u << 1)
#define LEC_ARM_COHERE		(1u << 2)

/* 6 dB: a passive hybrid only loses energy, so a filter with more gain is not a
 * path and the watchdog resets it; coef_energy_max adds the reference loss. */
#define LEC_COEF_ENERGY_MAX	4.0f

/*
 * What a refused trial gives back (probe_reject): the beliefs that
 * the trial drove along with the coefficients. Measurements of the line, the
 * runs that a refusal zeroes, the processor state and the counters of emitted
 * audio stay out. One struct makes snapshot and restore one assignment each.
 */
struct lec_belief {
	float mic_power_avg;
	float res_power_avg;
	float erle_db;
	float echo_gain;
	float gain_mic_avg;
	float gain_win_avg;
	unsigned gain_warmup;
	unsigned dt_hangover;
	/* The running hangover is the short one of a verdict on a thin
	 * frame. */
	bool dt_thin;
	/* The running hangover is a short one (nlp_hold_update). */
	bool dt_short;
	/* The trust count (LEC_GEIGEL_TRUST_FRAMES), saturating at UINT16_MAX;
	 * a refusal keeps the smaller count, so a restart in the trial survives
	 * and trial frames give no credit. */
	unsigned trust_run;
	/* The energy rule of the frame before. */
	bool trust_rule;
	/* The trust held on a frame since the last restart of the count. */
	bool trust_held;
};

/*
 * The readings that the tracker collects before it moves the floor, in the
 * whole-dB levels of udsp_cn_level_get. quiet and loud are the extremes, so
 * quiet - loud is the spread; age counts the frames since the first reading.
 * creep and stale serve LEC_NOISE_CREEP_SHARE and noise_rise_falsified.
 */
struct noise_run {
	unsigned frames;
	unsigned age;
	float sum;
	int quiet;
	int loud;
	unsigned creep;
	unsigned stale;
	/* The quiet windows the readings come from. */
	unsigned pauses;
};

/*
 * The history of the far-end floor (the LEC_REF_FLOOR_SUB_FRAMES comment):
 * the lowest frame power of each completed sub-window, with next on the
 * oldest, and the lowest power of the run_frames frames of the running
 * sub-window.
 */
struct ref_floor {
	float sub_min[LEC_REF_FLOOR_SUBS];
	float run_min;
	unsigned run_frames;
	unsigned next;
};

struct udsp_ec {
	unsigned frame;
	unsigned taps;
	unsigned tail_ms;
	unsigned window_frames;
	unsigned window_index;
	unsigned hist_pos;
	unsigned dt_run;
	/* The frames since the last verdict other than a short one on a thin
	 * frame, up to LEC_GEIGEL_REPEAT_FRAMES. It starts at the limit, so the
	 * first verdict of an instance is not a repeat. */
	unsigned dt_gap;
	unsigned path_run;
	/* The over-estimate run of over_get. */
	unsigned over_run;
	/* The trust gate that the first frame of the run read. */
	bool over_gate;
	/* Frames since the trust last held, up to LEC_OVER_TRUST_FRAMES + 1. */
	unsigned trust_age;
	/* The trial that the run starts takes scaled taps
	 * (over_scale_apply). */
	bool over_scale;
	unsigned stale_run;
	/* The stand-down of the LEC_TRIAL_STANDDOWN_FRAMES comment: the active
	 * frames left, the arms it holds, and the arms that a refusal of the
	 * running trial stands down. */
	unsigned standdown;
	unsigned standdown_arms;
	unsigned trial_owed;
	/* The LEC_ARM_* bit of the arm that started the running trial, 0 for
	 * the latch arm (echo_gain_reseed). */
	unsigned trial_own;
	unsigned probe_frames;
	/* The running trial started on taps that fail the energy rule, so the
	 * step floor of a bare trial applies (ec_run). */
	bool probe_bare;
	/* probe_bare holds its evidence for the running trial: probe_start
	 * clears it, and ec_run sets it after the start frame's rollback. */
	bool probe_judged;
	unsigned nlp_run;
	/* nlp_acted counts the frames that the processor acts on after NEW or
	 * the last reset, up to LEC_NLP_FIRST_FRAMES. nlp_engaged is set when
	 * the count reaches that bound. */
	unsigned nlp_acted;
	bool nlp_engaged;
	/* The active frames after NEW, up to LEC_NLP_BOOT_FRAMES (the
	 * bootstrap of the combined-loss arm). A reset of an instance that ran
	 * a frame sets it to LEC_NLP_BOOT_FRAMES. */
	unsigned nlp_boot;
	/* The count of quiet frames over the echo-likeness bar in a row,
	 * through the previous frame, up to LEC_NLP_QUIET_MISSES + 1
	 * (nlp_apply). Any other frame resets it. */
	unsigned nlp_quiet_misses;
	/* The far pause of the resume (nlp_resume_get) runs from a quiet frame
	 * up to the next frame that is not quiet. */
	bool nlp_in_pause;
	/* The processor acted after the latest pause. */
	bool nlp_burst_acted;
	/* nlp_burst_acted of the burst before the pause. */
	bool nlp_pause_acted;
	/* A quiet frame of the pause stood over the peak bound or read no
	 * floor; the processor switched off clears the four pause flags. */
	bool nlp_pause_peak;
	/* Quiet frames of the pause, up to LEC_NLP_TAIL_QUIET_FRAMES + 1. */
	unsigned nlp_pause_frames;
	/* The short hold of a resume (nlp_hold_update): its frames run. */
	bool nlp_hold_short;
	/* A quiet frame fell inside it, or it started after a pause. */
	bool nlp_hold_pause;
	/* The processor acted on the burst that it started on. */
	bool nlp_hold_acted;
	/* The largest tracker reading of the burst since the latest pause, and
	 * that of the burst before the pause (LEC_NLP_RESUME_MARGIN_DB). */
	float nlp_burst_erle;
	float nlp_pause_erle;
	/* The tracker readings of the last LEC_NLP_RESUME_CLIMB_FRAMES frames,
	 * and the slot of the oldest (nlp_erle_note). */
	float nlp_erle_ring[LEC_NLP_RESUME_CLIMB_FRAMES];
	unsigned nlp_erle_pos;
	/* A dt, unlike or erle frame cut the engage run since the run was last
	 * complete (nlp_cut_update). */
	bool nlp_run_cut;
	/* The reference of the previous frame is active (t.active), and the
	 * tone tail hold runs (tone_tail_update). */
	bool ref_was_active;
	bool tone_tail;
	/* The microphone powers of the last quiet-window frames, the next slot
	 * and the count (LEC_PAUSE_RING). */
	float pause_ring[LEC_PAUSE_RING];
	unsigned pause_pos;
	unsigned pause_fill;
	/* The window of the previous frame is active, a reference-only frame
	 * included (udsp_ec_nlms_ref_feed). */
	bool window_was_active;
	/* The near-end reading of the far-end pauses (LEC_PAUSE_NEAR_RATIO):
	 * the latest pause still runs, and it read a near end. */
	bool pause_open;
	bool pause_near;
	/* The drop hold runs (LEC_DROP_HOLD_DB). */
	bool drop_hold;
	/* A pause run ended (drop_pause_end). */
	bool drop_pause_seen;
	/* A held active frame outside a trial: frame_filter_run sums the lag
	 * gradient and the energies of the coherence end (drop_grad_add). */
	bool drop_grad_on;
	/* Quiet-window frames of the running pause; the residual and the floor
	 * summed over drop_pause_n of them. */
	unsigned drop_pause_len;
	unsigned drop_pause_n;
	float drop_pause_res;
	float drop_pause_floor;
	/* Active frames since the last pause run and since the start or the
	 * last talker run, up to UINT16_MAX. */
	unsigned drop_since_pause;
	unsigned drop_since_talk;
	/* Clean pause runs, misfit frames and coherent frames in a row. */
	unsigned drop_clean_runs;
	unsigned drop_misfit_run;
	unsigned drop_cohere_run;
	/* The lag sums of drop_grad_add and their smoothed forms. */
	float drop_grad_res;
	float drop_grad_ref;
	float drop_grad_res_avg;
	float drop_grad_ref_avg;
	uint32_t frames;
	uint32_t hold_frames;
	uint32_t skip_frames;
	uint32_t diverge_resets;
	uint32_t probes;
	uint32_t nlp_active_frames;
	uint32_t nlp_wait_frames;
	uint32_t nlp_erle_frames;
	uint32_t nlp_unlike_frames;
	uint32_t nlp_dt_frames;
	uint32_t nlp_quiet_frames;
	uint32_t nlp_dt_fired_frames;
	/* The frames of the active window so far, saturating at UINT16_MAX.
	 * udsp_ec_nlms_ref_feed does not update it, so it can lag by the frames
	 * of a skip. */
	unsigned window_run;
	/* The quiet-window frames in a row, up to
	 * LEC_GEIGEL_TRUST_PAUSE_FRAMES. A skipped frame leaves it, as it
	 * leaves window_run. */
	unsigned quiet_run;
	/* The window maximum of the frame before, the second half of the
	 * guard window of talker_get. */
	float window_prev;
	/* The exact energy of the history window (hist_push). */
	double hist_energy;
	float power_floor;
	/* The reference loss and the rules that ref_loss_scale_set scales by
	 * it. A loss of 0 makes the power factor exactly 1.0f, so each field
	 * then equals its constant. */
	unsigned ref_loss_half_db;
	float ref_loss_db;
	float active_bar;
	float gain_min;
	float gain_floor;
	float talker_ratio;
	float coef_energy_max;
	float step_floor_scale;
	float live_mic_power;
	float live_res_power;
	float live_est_power;
	/* The gain of each block of LEC_IPNLMS_BLOCK taps on the step of the
	 * frame (block_gain_set). */
	float block_gain[LEC_IPNLMS_BLOCKS_MAX];
	/* The smoothed means per sample of the coherence escape: residual times
	 * estimate, residual energy and estimate energy (cohere_update); and
	 * the run at or under its bar. */
	float cohere_cross;
	float cohere_res;
	float cohere_est;
	unsigned cohere_run;
	float probe_ratio_before;
	/* What the instance believes about the line and the filter, and the
	 * snapshot of it that a trial takes (struct lec_belief). */
	struct lec_belief belief;
	struct lec_belief probe_state;
	float noise_dbov;
	float noise_power;
	unsigned noise_stale_run;
	struct noise_run noise_rise;
	struct noise_run noise_seed;
	/* The agreement run beside the rise run (LEC_NOISE_AGREE_READINGS). */
	struct noise_run noise_agree;
	/* The frame before is a quiet-window frame: a reading on this frame
	 * continues the quiet window of the reading before it. */
	bool noise_quiet_last;
	/* The floor that the stale rule dropped, 0 once a floor is set again:
	 * the floor that follows the drop stands no louder
	 * (noise_stale_update). */
	float noise_drop_dbov;
	/* Frames left of the re-acquisition window of the floor
	 * (LEC_NOISE_RISE_AGE_KEPT). */
	unsigned noise_rise_window;
	/* The floor that the rise run at the keep implies, as a power, 0 for
	 * none: the processor reads it inside the window. */
	float noise_implied_power;
	/* Out of struct lec_belief and kept by the reset: it measures the
	 * far end, not the filter. */
	struct ref_floor ref_floor;
	float coef_energy;
	unsigned coef_peak_tap;
	/* The onset peak of onset_peak_update: the peak tap stands under the
	 * frame length because plain onset steps put it there. */
	bool onset_peak;
	/* The frames in a row that end with the peak tap at or past the frame
	 * length on an untrusted filter, up to LEC_ONSET_PEAK_CLEAR_FRAMES
	 * (onset_peak_update). */
	unsigned onset_far_run;
	/* The filter starts from zero coefficients: udsp_ec_nlms_reset and the
	 * watchdog's zeroing set it, and it clears with the onset peak
	 * (onset_peak_update). */
	bool onset_zero;
	/* Set by every write to coef. coef_energy_track recomputes energy and
	 * peak tap on the next read. A frame that left the filter alone reads
	 * the values of the last frame that moved it. */
	bool coef_dirty;
	bool freeze_on;
	bool nlp_on;
	/* The test switches of dsp/lec.h: a frame whose whole window is zero
	 * skips the tap loops bit for bit (frame_zero_run). */
	bool zero_skip_on;
	/* The activity bar follows the far-end floor (ref_bar_get). */
	bool ref_floor_on;
	bool noise_valid;
	bool nlp_active;
	struct udsp_ec_nlms_frame reading;
	struct udsp_cn cn;
	/* The interpolator of the comfort noise on a 16 kHz line (cn_fill). */
	struct udsp_resamp cn_up;
	float window_power[LEC_WINDOW_FRAMES_MAX];
	float *coef;
	float *backup;
	float *probe;
	float *grad;
	float *grad_avg;
	float *hist;
	/* 16 bytes, so the vector loops load the coefficients aligned: the six
	 * sub-arrays start at multiples of the tap count, a multiple of 8
	 * floats. calloc aligns so on both libcs, and this states the need. */
	_Alignas(16) float mem[];
};

/*
 * The first condition that refused the processor on a frame, in the order
 * nlp_gate_get checks them, so that each frame has one verdict and nlp_count
 * one field to count it in.
 */
enum nlp_verdict {
	NLP_VERDICT_OFF,
	NLP_VERDICT_QUIET,
	NLP_VERDICT_DT,
	NLP_VERDICT_UNLIKE,
	NLP_VERDICT_ERLE,
	NLP_VERDICT_PASS,
};

/*
 * The record of one frame: powers, bars, window state and verdicts. Each stage
 * of ec_run reads and writes it, and reading_fill copies it into the frame
 * reading, which no stage reads. trial_open is the state at entry, probing the
 * state after a talker ended a trial; the seed reads the live count.
 */
struct lec_tick {
	float mic_power;
	float ref_power;
	float window_power;
	float estimate_power;
	float res_power;
	/* The mean of the residual times the estimate over the frame
	 * (frame_filter_run), the cross term of the coherence escape. */
	float cross_power;
	float bar_max;
	float ref_bar;
	float bar;
	float noise_power;
	/* The implied floor that the frame reads, a power, 0 outside the
	 * re-acquisition window. The peak bound and the tail threshold read
	 * this one value. */
	float implied_power;
	/* The frame before the filter (frame_undo_save). The cross-fade of a
	 * frame that starts an act can read it (nlp_fade_source_get). */
	const int16_t *input;
	/* The cross-fade starts from input (nlp_fade_source_get). */
	bool fade_input;
	float erle_db;
	int level;
	unsigned hold_left;
	enum nlp_verdict nlp_verdict;
	bool active;
	bool window_active;
	bool talker;
	bool held;
	bool trial_open;
	bool probing;
	bool background;
	bool dt_fired;
	bool echo_like;
	bool peak_like;
	/* The frame carries no echo (LEC_PAUSE_RING). */
	bool no_echo;
	bool classic_arm;
	bool trusted;
};

/*
 * The per-tap loops in two forms. The scalar one is canonical and the only form
 * without Advanced SIMD, which aarch64 makes mandatory. The switch therefore
 * picks the whole loop once per sample. The vector form changes only the order
 * of the dot product; the update steps a block once in both forms.
 */
#if defined(__aarch64__)
static bool lec_simd_on = true;
#endif

static float taps_dot_scalar(const float *coef, const float *win, unsigned taps)
{
	float estimate = 0.0f;
	unsigned k;

	for (k = 0; k < taps; k++)
		estimate += coef[k] * win[k];
	return estimate;
}

static float block_step_get(const float *gain, unsigned k, float step)
{
	return step * gain[k / LEC_IPNLMS_BLOCK];
}

static unsigned block_end_get(unsigned k, unsigned taps)
{
	return k + LEC_IPNLMS_BLOCK < taps ? k + LEC_IPNLMS_BLOCK : taps;
}

static void taps_update_scalar(float *coef, const float *win, unsigned taps,
			       float step, const float *gain)
{
	unsigned k = 0;

	while (k < taps) {
		unsigned end = block_end_get(k, taps);
		float s = block_step_get(gain, k, step);

		for (; k < end; k++)
			coef[k] += s * win[k];
	}
}

/*
 * One pass over the coefficients for the update of a sample and the estimate of
 * the next. It keeps the tap order and operands of two separate loops, so the
 * result is update-then-dot, tap for tap. win[k] == next[k+1] for every k, also
 * where the write index wraps, because the taps+1 ring keeps it.
 */
static float taps_fused_scalar(float *coef, const float *win, const float *next,
			       unsigned taps, float step, const float *gain)
{
	float estimate = 0.0f;
	unsigned k = 0;

	while (k < taps) {
		unsigned end = block_end_get(k, taps);
		float s = block_step_get(gain, k, step);

		for (; k < end; k++) {
			coef[k] += s * win[k];
			estimate += coef[k] * next[k];
		}
	}
	return estimate;
}

#if defined(__aarch64__)
/* The three vector kernels stay out of line: inlined into run(), the fused
 * kernel loses its vector registers to the rest of it. */
/*
 * Eight accumulators of four lanes keep 32 taps of the dot product in flight,
 * because the FMLA latency outruns four. The operands load into named registers
 * and the arrays walk by pointer, so the loads pair. A tap count is a multiple
 * of 8, so the four-wide loop closes the array.
 */
__attribute__((noinline))
static float taps_dot_simd(const float *coef, const float *win, unsigned taps)
{
	float32x4_t acc0 = vdupq_n_f32(0.0f);
	float32x4_t acc1 = vdupq_n_f32(0.0f);
	float32x4_t acc2 = vdupq_n_f32(0.0f);
	float32x4_t acc3 = vdupq_n_f32(0.0f);
	float32x4_t acc4 = vdupq_n_f32(0.0f);
	float32x4_t acc5 = vdupq_n_f32(0.0f);
	float32x4_t acc6 = vdupq_n_f32(0.0f);
	float32x4_t acc7 = vdupq_n_f32(0.0f);
	const float *c = coef;
	const float *w = win;
	unsigned k = 0;

	for (; k + 32u <= taps; k += 32u, c += 32u, w += 32u) {
		float32x4_t c0 = vld1q_f32(c);
		float32x4_t c1 = vld1q_f32(c + 4u);
		float32x4_t c2 = vld1q_f32(c + 8u);
		float32x4_t c3 = vld1q_f32(c + 12u);
		float32x4_t c4 = vld1q_f32(c + 16u);
		float32x4_t c5 = vld1q_f32(c + 20u);
		float32x4_t c6 = vld1q_f32(c + 24u);
		float32x4_t c7 = vld1q_f32(c + 28u);
		float32x4_t w0 = vld1q_f32(w);
		float32x4_t w1 = vld1q_f32(w + 4u);
		float32x4_t w2 = vld1q_f32(w + 8u);
		float32x4_t w3 = vld1q_f32(w + 12u);
		float32x4_t w4 = vld1q_f32(w + 16u);
		float32x4_t w5 = vld1q_f32(w + 20u);
		float32x4_t w6 = vld1q_f32(w + 24u);
		float32x4_t w7 = vld1q_f32(w + 28u);

		acc0 = vfmaq_f32(acc0, c0, w0);
		acc1 = vfmaq_f32(acc1, c1, w1);
		acc2 = vfmaq_f32(acc2, c2, w2);
		acc3 = vfmaq_f32(acc3, c3, w3);
		acc4 = vfmaq_f32(acc4, c4, w4);
		acc5 = vfmaq_f32(acc5, c5, w5);
		acc6 = vfmaq_f32(acc6, c6, w6);
		acc7 = vfmaq_f32(acc7, c7, w7);
	}
	for (; k + 4u <= taps; k += 4u, c += 4u, w += 4u)
		acc0 = vfmaq_f32(acc0, vld1q_f32(c), vld1q_f32(w));
	acc0 = vaddq_f32(acc0, acc4);
	acc1 = vaddq_f32(acc1, acc5);
	acc2 = vaddq_f32(acc2, acc6);
	acc3 = vaddq_f32(acc3, acc7);
	return vaddvq_f32(vaddq_f32(vaddq_f32(acc0, acc1),
				    vaddq_f32(acc2, acc3)));
}

/* The update loads all eight blocks, then accumulates, then stores. Eight
 * independent operations then stand between a store and the multiply-accumulate
 * that it depends on. */
__attribute__((noinline))
static void taps_update_simd(float *coef, const float *win, unsigned taps,
			     float step, const float *block)
{
	float *c = coef;
	const float *w = win;
	unsigned k = 0;

	for (; k + 32u <= taps; k += 32u, c += 32u, w += 32u) {
		float32x4_t gain = vdupq_n_f32(block_step_get(block, k, step));
		float32x4_t c0 = vld1q_f32(c);
		float32x4_t c1 = vld1q_f32(c + 4u);
		float32x4_t c2 = vld1q_f32(c + 8u);
		float32x4_t c3 = vld1q_f32(c + 12u);
		float32x4_t c4 = vld1q_f32(c + 16u);
		float32x4_t c5 = vld1q_f32(c + 20u);
		float32x4_t c6 = vld1q_f32(c + 24u);
		float32x4_t c7 = vld1q_f32(c + 28u);
		float32x4_t w0 = vld1q_f32(w);
		float32x4_t w1 = vld1q_f32(w + 4u);
		float32x4_t w2 = vld1q_f32(w + 8u);
		float32x4_t w3 = vld1q_f32(w + 12u);
		float32x4_t w4 = vld1q_f32(w + 16u);
		float32x4_t w5 = vld1q_f32(w + 20u);
		float32x4_t w6 = vld1q_f32(w + 24u);
		float32x4_t w7 = vld1q_f32(w + 28u);

		vst1q_f32(c, vfmaq_f32(c0, gain, w0));
		vst1q_f32(c + 4u, vfmaq_f32(c1, gain, w1));
		vst1q_f32(c + 8u, vfmaq_f32(c2, gain, w2));
		vst1q_f32(c + 12u, vfmaq_f32(c3, gain, w3));
		vst1q_f32(c + 16u, vfmaq_f32(c4, gain, w4));
		vst1q_f32(c + 20u, vfmaq_f32(c5, gain, w5));
		vst1q_f32(c + 24u, vfmaq_f32(c6, gain, w6));
		vst1q_f32(c + 28u, vfmaq_f32(c7, gain, w7));
	}
	for (; k + 4u <= taps; k += 4u, c += 4u, w += 4u)
		vst1q_f32(c, vfmaq_f32(vld1q_f32(c),
				       vdupq_n_f32(block_step_get(block, k,
								  step)),
				       vld1q_f32(w)));
}

/*
 * The fused pass costs six vector operations per four taps against seven for
 * two passes. The coefficients load once, and win[4j..4j+3] of the update
 * window is vextq_f32 of the two next-window vectors around it. The last block
 * reads one vector past the window end, which the ring pads.
 */
__attribute__((noinline))
static float taps_fused_simd(float *coef, const float *win, const float *next,
			     unsigned taps, float step, const float *block)
{
	float32x4_t acc0 = vdupq_n_f32(0.0f);
	float32x4_t acc1 = vdupq_n_f32(0.0f);
	float32x4_t acc2 = vdupq_n_f32(0.0f);
	float32x4_t acc3 = vdupq_n_f32(0.0f);
	float *c = coef;
	const float *n = next;
	unsigned k = 0;

	for (; k + 32u <= taps; k += 32u, c += 32u, n += 32u) {
		float32x4_t gain = vdupq_n_f32(block_step_get(block, k, step));
		float32x4_t n0 = vld1q_f32(n);
		float32x4_t n1 = vld1q_f32(n + 4u);
		float32x4_t n2 = vld1q_f32(n + 8u);
		float32x4_t n3 = vld1q_f32(n + 12u);
		float32x4_t n4 = vld1q_f32(n + 16u);
		float32x4_t n5 = vld1q_f32(n + 20u);
		float32x4_t n6 = vld1q_f32(n + 24u);
		float32x4_t n7 = vld1q_f32(n + 28u);
		float32x4_t n8 = vld1q_f32(n + 32u);
		float32x4_t c0 = vfmaq_f32(vld1q_f32(c), gain,
					   vextq_f32(n0, n1, 1));
		float32x4_t c1 = vfmaq_f32(vld1q_f32(c + 4u), gain,
					   vextq_f32(n1, n2, 1));
		float32x4_t c2 = vfmaq_f32(vld1q_f32(c + 8u), gain,
					   vextq_f32(n2, n3, 1));
		float32x4_t c3 = vfmaq_f32(vld1q_f32(c + 12u), gain,
					   vextq_f32(n3, n4, 1));
		float32x4_t c4 = vfmaq_f32(vld1q_f32(c + 16u), gain,
					   vextq_f32(n4, n5, 1));
		float32x4_t c5 = vfmaq_f32(vld1q_f32(c + 20u), gain,
					   vextq_f32(n5, n6, 1));
		float32x4_t c6 = vfmaq_f32(vld1q_f32(c + 24u), gain,
					   vextq_f32(n6, n7, 1));
		float32x4_t c7 = vfmaq_f32(vld1q_f32(c + 28u), gain,
					   vextq_f32(n7, n8, 1));

		vst1q_f32(c, c0);
		vst1q_f32(c + 4u, c1);
		vst1q_f32(c + 8u, c2);
		vst1q_f32(c + 12u, c3);
		vst1q_f32(c + 16u, c4);
		vst1q_f32(c + 20u, c5);
		vst1q_f32(c + 24u, c6);
		vst1q_f32(c + 28u, c7);
		acc0 = vfmaq_f32(acc0, c0, n0);
		acc1 = vfmaq_f32(acc1, c1, n1);
		acc2 = vfmaq_f32(acc2, c2, n2);
		acc3 = vfmaq_f32(acc3, c3, n3);
		acc0 = vfmaq_f32(acc0, c4, n4);
		acc1 = vfmaq_f32(acc1, c5, n5);
		acc2 = vfmaq_f32(acc2, c6, n6);
		acc3 = vfmaq_f32(acc3, c7, n7);
	}
	/* The tail loads the update window instead of shifting into it: there
	 * is no next block to straddle, and a tap count is a multiple of 8 so
	 * this runs at most twice. */
	for (; k + 4u <= taps; k += 4u, c += 4u, n += 4u) {
		float32x4_t gain = vdupq_n_f32(block_step_get(block, k, step));
		float32x4_t cv = vfmaq_f32(vld1q_f32(c), gain,
					   vld1q_f32(win + k));

		vst1q_f32(c, cv);
		acc0 = vfmaq_f32(acc0, cv, vld1q_f32(n));
	}
	return vaddvq_f32(vaddq_f32(vaddq_f32(acc0, acc1),
				    vaddq_f32(acc2, acc3)));
}
#endif

static float taps_dot(const float *coef, const float *win, unsigned taps)
{
#if defined(__aarch64__)
	if (lec_simd_on)
		return taps_dot_simd(coef, win, taps);
#endif
	return taps_dot_scalar(coef, win, taps);
}

static void taps_update(float *coef, const float *win, unsigned taps,
			float step, const float *gain)
{
#if defined(__aarch64__)
	if (lec_simd_on) {
		taps_update_simd(coef, win, taps, step, gain);
		return;
	}
#endif
	taps_update_scalar(coef, win, taps, step, gain);
}

static float taps_fused(float *coef, const float *win, const float *next,
			unsigned taps, float step, const float *gain)
{
#if defined(__aarch64__)
	if (lec_simd_on)
		return taps_fused_simd(coef, win, next, taps, step, gain);
#endif
	return taps_fused_scalar(coef, win, next, taps, step, gain);
}

/*
 * coef, backup, probe, grad and grad_avg are one tap array each. The history
 * ring is the doubled taps+1 window of hist_push plus a four-float pad. The
 * vector fused kernel reads one vector past the window end and discards the
 * lanes it does not use.
 */
static unsigned mem_floats_get(unsigned taps)
{
	return 5u * taps + 2u * (taps + 1u) + 4u;
}

/* The loudest echo that the far end can explain in this frame: the window
 * maximum times the learned gain with headroom, capped at gain_min. Near speech
 * cannot move it. The factor alone also caps the activity bar. */
static float echo_bound_factor_get(const struct udsp_ec *ec)
{
	float bound = ec->belief.echo_gain * LEC_GEIGEL_GAIN_HEADROOM;

	if (bound > ec->gain_min)
		bound = ec->gain_min;
	return bound;
}

static float echo_bound_get(const struct udsp_ec *ec, float window_power)
{
	return window_power * echo_bound_factor_get(ec);
}

/*
 * The share of the microphone that the far end can explain, the classic bound
 * over the microphone, as a scale on the step. The estimate arm of
 * the detector rises with an update on near speech and hides it; the bound does
 * not. A microphone at or under the bound returns exactly 1.0f.
 */
static float talk_scale_get(const struct udsp_ec *ec, float mic_power,
			    float window_power)
{
	float explain;

	/* A trial is exempt: it tests what the learned gain cannot hold, a
	 * hybrid louder than any that the tracker saw, under the talker end
	 * and the watchdog. */
	if (ec->probe_frames)
		return 1.0f;
	/* The bound is the detector's own classic arm. A microphone between the
	 * bound and the detector's margin adapts at a step reduced by the
	 * excess. */
	explain = echo_bound_get(ec, window_power);
	/* The false arm of each comparison takes a value that is not a number,
	 * so an unreadable microphone or bound falls on the unscaled step. */
	if (!(explain >= 0.0f) || !(mic_power > explain))
		return 1.0f;
	return explain / mic_power;
}

/*
 * The step scaled by the share of the residual that is still echo: the smoothed
 * live residual of the frames before, less the tracked floor, over that
 * residual. A line with no measurable floor returns exactly 1.0f, and a
 * residual at or under the floor returns 0, so the filter holds.
 */
static float step_scale_get(const struct udsp_ec *ec)
{
	float floor = ec->noise_power;
	float res = ec->live_res_power;

	/* NaN compares false: a floor that is not a number takes the unscaled
	 * step, a residual that is not a number the zero step; divergence_check
	 * reseeds both from finite powers. */
	if (!(floor > 0.0f))
		return 1.0f;
	if (!(res > floor) || !(res < INFINITY))
		return 0.0f;
	/* res > floor > 0 and res is finite, so the quotient is in (0, 1). */
	return (res - floor) / res;
}

/*
 * The window is kept twice, so the tap loop reads it as one forward run, and
 * win[k] is the reference k samples back. The ring holds taps+1 samples, so the
 * fused pass holds the windows of two consecutive samples. The window contents
 * are those of a ring of taps slots, sample for sample.
 */
static const float *hist_push(struct udsp_ec *ec, int16_t sample)
{
	unsigned period = ec->taps + 1u;
	unsigned pos = ec->hist_pos;
	float value = (float)sample;
	float leaving = ec->hist[pos + ec->taps - 1u];

	/*
	 * int16 values are exact in float and their squares are exact in
	 * double, so this sliding sum is the window energy with no drift; a
	 * periodic rescan would buy nothing.
	 */
	ec->hist_energy -= (double)leaving * leaving;
	pos = pos ? pos - 1u : period - 1u;
	ec->hist[pos] = value;
	ec->hist[pos + period] = value;
	ec->hist_energy += (double)value * value;
	ec->hist_pos = pos;
	return &ec->hist[pos];
}

/*
 * A frame of an escape trial: a path-change or coherence trial on taps with
 * echo evidence, read after the start frame's rollback. Its blocks take the
 * proportionate step, a verdict takes the short hangover, and the ERLE tracker
 * stays out. An echo-like frame with enough live ERLE passes the gate at once,
 * unless the far pause read a near end.
 */
static bool escape_trial_get(const struct udsp_ec *ec)
{
	return ec->probe_frames && ec->probe_judged && !ec->probe_bare &&
	       (ec->trial_own & (LEC_ARM_PATH | LEC_ARM_COHERE));
}

/*
 * The block gains of this frame are 1 outside an escape trial. Inside one, a
 * block takes (1 - LEC_IPNLMS_ALPHA) / 2 plus (1 + LEC_IPNLMS_ALPHA) / 2 times
 * its mean tap magnitude over the filter's. LEC_IPNLMS_GAIN_MAX caps the gain.
 * An l1 norm that is zero or not finite gives 1.
 */
static void block_gain_set(struct udsp_ec *ec)
{
	float block[LEC_IPNLMS_BLOCKS_MAX];
	float l1 = 0.0f;
	unsigned blocks = (ec->taps + LEC_IPNLMS_BLOCK - 1u) / LEC_IPNLMS_BLOCK;
	unsigned b;
	unsigned k;

	if (!escape_trial_get(ec)) {
		for (b = 0; b < blocks; b++)
			ec->block_gain[b] = 1.0f;
		return;
	}
	for (b = 0; b < blocks; b++) {
		unsigned end = block_end_get(b * LEC_IPNLMS_BLOCK, ec->taps);

		block[b] = 0.0f;
		for (k = b * LEC_IPNLMS_BLOCK; k < end; k++)
			block[b] += fabsf(ec->coef[k]);
		l1 += block[b];
	}
	for (b = 0; b < blocks; b++) {
		unsigned from = b * LEC_IPNLMS_BLOCK;
		float n = (float)(block_end_get(from, ec->taps) - from);
		float gain = 1.0f;

		if (l1 > 0.0f && l1 < INFINITY)
			gain = (1.0f - LEC_IPNLMS_ALPHA) * 0.5f +
			       (1.0f + LEC_IPNLMS_ALPHA) * 0.5f *
			       (float)ec->taps * block[b] / (n * l1);
		ec->block_gain[b] = gain < LEC_IPNLMS_GAIN_MAX ?
				    gain : LEC_IPNLMS_GAIN_MAX;
	}
}

/* The sums of drop_cohere_update for one sample that drop_grad_on marks. The
 * first is the residual times the reference at each lag, on the update kernel
 * with gains of 1. The others are the residual energy and the mean lag
 * energy. */
static void drop_grad_add(struct udsp_ec *ec, const float *win, float error)
{
	taps_update(ec->grad, win, ec->taps, error, ec->block_gain);
	ec->drop_grad_res += error * error;
	ec->drop_grad_ref += (float)ec->hist_energy / (float)ec->taps;
}

/*
 * One frame of NLMS, sample by sample, in one fused pass per sample in the
 * operation order of two loops. A block update per frame drives a periodic
 * reference past the stability bound. It writes the saturated residual over
 * mic, sums the estimate energy and the residual before saturation times the
 * estimate, and returns the residual energy.
 */
static float frame_filter_run(struct udsp_ec *ec, int16_t *mic,
			      const int16_t *ref, bool adapt, float mu,
			      float step_floor, float *estimate_power,
			      float *cross_power)
{
	const float *win = hist_push(ec, ref[0]);
	float estimate = taps_dot(ec->coef, win, ec->taps);
	float res_power = 0.0f;
	unsigned i;

	if (adapt)
		block_gain_set(ec);
	for (i = 0; i < ec->frame; i++) {
		const float *next;
		float error;
		float step = 0.0f;

		*estimate_power += estimate * estimate;
		error = (float)mic[i] - estimate;
		res_power += error * error;
		*cross_power += error * estimate;
		mic[i] = udsp_sat16_round(error);
		if (ec->drop_grad_on)
			drop_grad_add(ec, win, error);
		if (adapt)
			step = mu * error /
			       ((float)ec->hist_energy + step_floor);
		if (i + 1u == ec->frame) {
			if (adapt)
				taps_update(ec->coef, win, ec->taps, step,
					    ec->block_gain);
			break;
		}
		next = hist_push(ec, ref[i + 1u]);
		estimate = adapt ?
			taps_fused(ec->coef, win, next, ec->taps, step,
				   ec->block_gain) :
			taps_dot(ec->coef, next, ec->taps);
		win = next;
	}
	return res_power;
}

/* A frame over a window of zeros: every estimate is exactly 0.0f, so the
 * residual is mic, sample for sample. The ring pushes and the residual energy
 * keep the order of the full form, and the inactive frame adapts nothing. */
static float frame_zero_run(struct udsp_ec *ec, const int16_t *mic,
			    const int16_t *ref)
{
	float res_power = 0.0f;
	unsigned i;

	for (i = 0; i < ec->frame; i++) {
		float error = (float)mic[i];

		hist_push(ec, ref[i]);
		res_power += error * error;
	}
	return res_power;
}

/*
 * A frame that the detector rolls back emits the output of its start
 * coefficients. It adapted before its verdict existed. frame_rerun runs it
 * again without adaptation on what frame_undo_save keeps. Detector, trackers
 * and trial arms keep the first pass; the floor tracker and the processor read
 * the second.
 */
struct frame_undo {
	float leaving[UDSP_FRAME_MAX];
	int16_t mic[UDSP_FRAME_MAX];
	unsigned pos;
	double energy;
};

/* The state that frame_rerun restores, read before the frame pushes its
 * reference. The pushes walk the ring backwards from hist_pos, so slot i of
 * leaving is the slot that push i writes (hist_push). */
static void frame_undo_save(const struct udsp_ec *ec, const int16_t *mic,
			    struct frame_undo *u)
{
	unsigned period = ec->taps + 1u;
	unsigned pos = ec->hist_pos;
	unsigned i;

	u->pos = pos;
	u->energy = ec->hist_energy;
	for (i = 0; i < ec->frame; i++) {
		pos = pos ? pos - 1u : period - 1u;
		u->leaving[i] = ec->hist[pos];
	}
	memcpy(u->mic, mic, sizeof(*mic) * ec->frame);
}

/* The frame again without adaptation, from the state of u: its output on the
 * coefficients that ec_run restored. The ring ends where the first pass left
 * it. */
static void frame_rerun(struct udsp_ec *ec, int16_t *mic, const int16_t *ref,
			const struct frame_undo *u)
{
	unsigned period = ec->taps + 1u;
	unsigned pos = u->pos;
	float estimate_power = 0.0f;
	float cross_power = 0.0f;
	unsigned i;

	for (i = 0; i < ec->frame; i++) {
		pos = pos ? pos - 1u : period - 1u;
		ec->hist[pos] = u->leaving[i];
		ec->hist[pos + period] = u->leaving[i];
	}
	ec->hist_pos = u->pos;
	ec->hist_energy = u->energy;
	memcpy(mic, u->mic, sizeof(*mic) * ec->frame);
	(void)frame_filter_run(ec, mic, ref, false, 0.0f, 0.0f,
			       &estimate_power, &cross_power);
}

/* The ring entry of one frame of reference, read by window_power_get. */
static void window_push(struct udsp_ec *ec, float ref_power)
{
	ec->window_power[ec->window_index] = ref_power;
	ec->window_index = (ec->window_index + 1u) % ec->window_frames;
}

static float window_power_get(const struct udsp_ec *ec)
{
	float loudest = 0.0f;
	unsigned i;

	for (i = 0; i < ec->window_frames; i++) {
		if (ec->window_power[i] > loudest)
			loudest = ec->window_power[i];
	}
	return loudest;
}

/* One frame of reference into the history of the far-end floor. A full
 * running sub-window replaces the oldest completed one. */
static void ref_floor_add(struct ref_floor *rf, float power)
{
	if (!rf->run_frames || power < rf->run_min)
		rf->run_min = power;
	if (++rf->run_frames < LEC_REF_FLOOR_SUB_FRAMES)
		return;
	rf->sub_min[rf->next] = rf->run_min;
	rf->next = (rf->next + 1u) % LEC_REF_FLOOR_SUBS;
	rf->run_frames = 0;
}

/* The lowest frame power of the history. An empty running sub-window holds
 * no frame, so it does not count. */
static float ref_floor_get(const struct ref_floor *rf)
{
	float lowest = rf->run_frames ? rf->run_min : rf->sub_min[0];
	unsigned i;

	for (i = 0; i < LEC_REF_FLOOR_SUBS; i++) {
		if (rf->sub_min[i] < lowest)
			lowest = rf->sub_min[i];
	}
	return lowest;
}

/*
 * The cap of the activity bar, from the learned gain as it stands. ec_run
 * reads it before the frame runs, because a talker that ends a trial restores
 * another gain. A quotient that is not a number takes the fixed bar, the rule
 * of step_scale_get.
 */
static float ref_bar_max_get(const struct udsp_ec *ec)
{
	float bar_max = LEC_REF_RETURN_MAX / echo_bound_factor_get(ec);

	return bar_max > ec->active_bar ? bar_max : ec->active_bar;
}

/*
 * The activity bar of a frame, by the rule of the LEC_REF_FLOOR_SUB_FRAMES
 * comment; the frame joins the floor after this call. While a trial runs
 * (probing) the bar is the fixed bar, so a background keeps spending the trial
 * budget. Under a raised bar a half-adapted filter stays suspended when the far
 * speech stops.
 */
static float ref_bar_get(const struct udsp_ec *ec, float bar_max,
			 bool probing)
{
	float bar;

	if (probing || !ec->ref_floor_on)
		return ec->active_bar;
	bar = ref_floor_get(&ec->ref_floor) * LEC_REF_FLOOR_MARGIN;
	if (!(bar > ec->active_bar))
		return ec->active_bar;
	return bar < bar_max ? bar : bar_max;
}

/*
 * The verdict of the LEC_TALKER_RATIO comment on this frame. window_prev still
 * holds the window maximum of the frame before. Each comparison is written so
 * that a value that is not a number takes the false arm, the rule of
 * step_scale_get.
 */
static bool talker_get(const struct udsp_ec *ec, const struct lec_tick *t)
{
	float guard = ec->window_prev > t->window_power ? ec->window_prev :
							  t->window_power;

	if (!t->window_active)
		return false;
	return t->mic_power > guard * ec->talker_ratio;
}

/* The peak tap comes from the same pass, the only one over the filter. The
 * compare reads |coef|, not its square, so the compiler still fuses the sum. A
 * filter whose energy reads 0 reports tap 0. */
static float coef_energy_get(const struct udsp_ec *ec, unsigned *peak)
{
	float energy = 0.0f;
	float loudest = 0.0f;
	unsigned best = 0;
	unsigned k;

	for (k = 0; k < ec->taps; k++) {
		energy += ec->coef[k] * ec->coef[k];
		if (fabsf(ec->coef[k]) > loudest) {
			loudest = fabsf(ec->coef[k]);
			best = k;
		}
	}
	if (!(energy > 0.0f))
		best = 0;
	*peak = best;
	return energy;
}

/*
 * The energy of the filter as it stands, recomputed only after a write to
 * the coefficients. The same sum over the same coefficients is the same
 * float, so a cached reading is the reading the sum would give; what the
 * cache removes is one pass over the taps on every frame that held.
 */
static float coef_energy_track(struct udsp_ec *ec)
{
	if (ec->coef_dirty) {
		ec->coef_energy = coef_energy_get(ec, &ec->coef_peak_tap);
		ec->coef_dirty = false;
	}
	return ec->coef_energy;
}

/* For a writer that zeroes the filter: the telemetry of this frame must read it
 * empty, which the dirty flag alone defers to the next pass. */
static void coef_energy_clear(struct udsp_ec *ec)
{
	ec->coef_energy = 0.0f;
	ec->coef_peak_tap = 0;
	ec->coef_dirty = false;
}

/*
 * The live residual over the live microphone, the quantity a trial is judged
 * by. Total in the sense of step_scale_get: a microphone or a residual this
 * code cannot read falls on 1.0f, the ratio of a filter that removes nothing,
 * so an unreadable "before" cannot accept a trial and an unreadable "after"
 * cannot keep one.
 */
static float probe_ratio_get(const struct udsp_ec *ec)
{
	float mic = ec->live_mic_power;
	float res = ec->live_res_power;

	if (!(mic > 0.0f) || !(res >= 0.0f) || !(res < INFINITY))
		return 1.0f;
	return res / mic;
}

/*
 * The coherence run and its sums start over at each trial edge, at a divergence
 * reset and at the reset. There they describe a filter that the line does not
 * have. Sums from zero read the frames after the edge alone. A sum that is not
 * a number fails every read until the next start over.
 */
static void cohere_restart(struct udsp_ec *ec)
{
	ec->cohere_run = 0;
	ec->cohere_cross = 0.0f;
	ec->cohere_res = 0.0f;
	ec->cohere_est = 0.0f;
}

/* The five runs that a trial start and a trial end zero: latch,
 * path-change, over-estimate, stale, and coherence with its sums. */
static void runs_clear(struct udsp_ec *ec)
{
	ec->dt_run = 0;
	ec->path_run = 0;
	ec->over_run = 0;
	ec->stale_run = 0;
	cohere_restart(ec);
}

/* The ERLE tracker as a pair of powers at or over LEC_ERLE_FLOOR and the
 * ratio in dB that the detector and the processor read. */
static void erle_set(struct udsp_ec *ec, float mic_avg, float res_avg)
{
	if (mic_avg < LEC_ERLE_FLOOR)
		mic_avg = LEC_ERLE_FLOOR;
	if (res_avg < LEC_ERLE_FLOOR)
		res_avg = LEC_ERLE_FLOOR;
	ec->belief.mic_power_avg = mic_avg;
	ec->belief.res_power_avg = res_avg;
	ec->belief.erle_db = 10.0f * log10f(mic_avg / res_avg);
}

/*
 * ec_run copies the taps after the rollback of the start frame. The snapshot of
 * every arm is therefore the filter that the start frame ends with. own_arm is
 * the LEC_ARM_* bit of the starting arm, 0 for the latch arm. A refusal stands
 * it down with every arm that stands down at the start.
 */
static void probe_start(struct udsp_ec *ec, unsigned own_arm)
{
	ec->drop_hold = false;
	ec->trial_owed = (ec->standdown ? ec->standdown_arms : 0u) | own_arm;
	ec->trial_own = own_arm;
	ec->probe_state = ec->belief;
	ec->probe_ratio_before = probe_ratio_get(ec);
	ec->probe_frames = LEC_PROBE_FRAMES;
	ec->probe_judged = false;
	runs_clear(ec);
	ec->probes++;
}

/*
 * Undo the trial whole and end it: the filter, the trackers and the detector go
 * back to the start frame. A refused improvement, the watchdog and a talker
 * frame reach it, and each needs the same undo. end is the way out that the
 * trace reads.
 */
static void probe_reject(struct udsp_ec *ec, enum udsp_ec_nlms_trial_end end)
{
	unsigned trust_run;

	ec->reading.trial_end = (uint8_t)end;
	ec->probe_frames = 0;
	memcpy(ec->coef, ec->probe, sizeof(*ec->coef) * ec->taps);
	ec->coef_dirty = true;
	/* The trust count keeps the smaller of the two (struct lec_belief). */
	trust_run = ec->belief.trust_run;
	ec->belief = ec->probe_state;
	if (trust_run < ec->belief.trust_run)
		ec->belief.trust_run = trust_run;
	/* Zeroed, not decayed: dt_run climbed through the trial and the talker
	 * who failed it is still on the line, so a remainder re-arms the next
	 * trial early. */
	runs_clear(ec);
	if (ec->trial_owed) {
		ec->standdown = LEC_TRIAL_STANDDOWN_FRAMES;
		ec->standdown_arms = ec->trial_owed;
	}
	/* The frame-end telemetry has to describe the filter the line ends
	 * the frame with, and this call is the last thing that changes it. */
	coef_energy_track(ec);
}

/*
 * A kept path-change or coherence trial raises the learned gain to its echo
 * evidence. That is the smaller of the microphone and the kept estimate over
 * the window average, capped at LEC_GAIN_RESEED_COEF_RATIO times coefficient
 * energy. A kept latch or stale trial can hold a near signal, so it does not.
 */
static void echo_gain_reseed(struct udsp_ec *ec)
{
	float win = ec->belief.gain_win_avg;
	float ratio = ec->belief.gain_mic_avg / win;
	float est = ec->cohere_est / win;
	float cap = coef_energy_track(ec) * LEC_GAIN_RESEED_COEF_RATIO;

	if (!(ec->trial_own & (LEC_ARM_PATH | LEC_ARM_COHERE)) ||
	    ec->belief.gain_warmup < LEC_GAIN_WARMUP_FRAMES)
		return;
	if (est < ratio)
		ratio = est;
	if (cap < ratio)
		ratio = cap;
	if (ratio > ec->belief.echo_gain && ratio < INFINITY)
		ec->belief.echo_gain = ratio;
}

/*
 * The floor that the rise run at a keep implies, as a power, or 0 for none. It
 * is the mean of the run plus LEC_NOISE_AGREE_BIAS_DB, at most
 * LEC_NOISE_IMPLIED_CAP_DB over the tracked floor. A run over the spread bound
 * implies none, and so does a run that is not louder than the floor.
 */
static float noise_rise_implied_get(const struct udsp_ec *ec)
{
	const struct noise_run *run = &ec->noise_rise;
	float dbov;

	if (!ec->noise_valid ||
	    (float)(run->quiet - run->loud) > LEC_NOISE_SPREAD_DB)
		return 0.0f;
	dbov = run->sum / (float)run->frames + LEC_NOISE_AGREE_BIAS_DB;
	if (!(dbov < ec->noise_dbov))
		return 0.0f;
	dbov = fmaxf(dbov, ec->noise_dbov - LEC_NOISE_IMPLIED_CAP_DB);
	return 32767.0f * 32767.0f * powf(10.0f, dbov * -0.1f);
}

/*
 * The re-acquisition window after a kept path-change or coherence trial opens
 * only on a rise run of LEC_NOISE_STALE_FRAMES readings or more. The window
 * takes the floor that this run implies, so a keep whose run implies none
 * ends the implied floor of an earlier keep. The rise run and the agreement
 * run then start over (LEC_NOISE_RISE_AGE_KEPT). A keep that does not open
 * the window leaves the window and its implied floor as they are.
 */
static void noise_rise_reacquire(struct udsp_ec *ec)
{
	if (!(ec->trial_own & (LEC_ARM_PATH | LEC_ARM_COHERE)) ||
	    ec->noise_rise.frames < LEC_NOISE_STALE_FRAMES)
		return;
	ec->noise_implied_power = noise_rise_implied_get(ec);
	memset(&ec->noise_rise, 0, sizeof(ec->noise_rise));
	memset(&ec->noise_agree, 0, sizeof(ec->noise_agree));
	ec->noise_rise_window = LEC_NOISE_RISE_AGE_KEPT;
}

static void probe_finish(struct udsp_ec *ec)
{
	if (probe_ratio_get(ec) > ec->probe_ratio_before * LEC_PROBE_IMPROVE) {
		probe_reject(ec, UDSP_EC_NLMS_TRIAL_END_REJECTED);
		return;
	}
	ec->reading.trial_end = UDSP_EC_NLMS_TRIAL_END_KEPT;
	/* The trial stands, so the hold misread this line: the tracker and the
	 * learned gain take the live readings, not the values before the
	 * hold. */
	erle_set(ec, ec->live_mic_power, ec->live_res_power);
	echo_gain_reseed(ec);
	noise_rise_reacquire(ec);
	ec->belief.dt_hangover = 0;
	ec->belief.dt_thin = false;
	ec->belief.dt_short = false;
	runs_clear(ec);
}

/* The verdict of the detector stands for as long as its hangover runs. */
static bool double_talk_get(const struct udsp_ec *ec)
{
	return ec->belief.dt_hangover != 0u;
}

/*
 * The run of the path-change escape: active frames under a hold whose live
 * residual stands LEC_PATH_CHANGE_RATIO over the live microphone. A frame whose
 * own reference is silent neither arms nor disarms it, because there the
 * residual becomes the microphone and the ratio means nothing.
 */
static bool path_change_get(struct udsp_ec *ec, bool active)
{
	if (!active)
		return false;
	if (ec->standdown && (ec->standdown_arms & LEC_ARM_PATH)) {
		ec->path_run = 0;
		return false;
	}
	if (!double_talk_get(ec)) {
		ec->path_run = 0;
		return false;
	}
	if (ec->live_res_power < ec->live_mic_power * LEC_PATH_CHANGE_RATIO) {
		ec->path_run = 0;
		return false;
	}
	ec->path_run++;
	return ec->path_run >= LEC_PATH_CHANGE_FRAMES;
}

/*
 * True when the over-estimate run of the LEC_OVER_RATIO comment comes due on
 * this frame. A frame whose reference is silent neither arms nor disarms it,
 * as for the path-change run. A talker, a stand-down of the path-change arm
 * or a frame under the ratio starts the run over. The first frame of a run
 * reads the trust gate, and the run comes due only with that gate held.
 */
static bool over_get(struct udsp_ec *ec, bool active, bool talker)
{
	if (!active)
		return false;
	if (talker || (ec->standdown && (ec->standdown_arms & LEC_ARM_PATH)) ||
	    !(ec->live_est_power >= ec->live_mic_power * LEC_OVER_RATIO)) {
		ec->over_run = 0;
		return false;
	}
	if (!ec->over_run)
		ec->over_gate = ec->trust_age <= LEC_OVER_TRUST_FRAMES;
	ec->over_run++;
	return ec->over_gate && ec->over_run >= LEC_OVER_FRAMES;
}

/*
 * The start taps of a trial of the over-estimate run: the taps times the
 * square root of live microphone over live estimate. ec_run calls it after
 * the snapshot, so a refusal gives the unscaled taps back (probe_reject). A
 * ratio that is not under one, or not a number, leaves the taps as they are.
 */
static void over_scale_apply(struct udsp_ec *ec)
{
	float g = ec->live_est_power > 0.0f ?
		  sqrtf(ec->live_mic_power / ec->live_est_power) : 1.0f;
	unsigned k;

	ec->over_scale = false;
	if (!(g < 1.0f))
		return;
	for (k = 0; k < ec->taps; k++)
		ec->coef[k] *= g;
	ec->coef_dirty = true;
	coef_energy_track(ec);
}

/* The over-estimate escape starts over with the filter, at the reset and at the
 * watchdog's zeroing. Live estimate, run, gate and scale go to zero, and the
 * age reads the trust as lost, so no count of the zeroed filter carries. */
static void over_restart(struct udsp_ec *ec)
{
	ec->live_est_power = 0.0f;
	ec->over_run = 0;
	ec->over_gate = false;
	ec->over_scale = false;
	ec->trust_age = LEC_OVER_TRUST_FRAMES + 1u;
}

/*
 * One active frame into the smoothed sums of the coherence escape, at the
 * rate of the live powers. Like them, the sums are measurements of the line,
 * taken on every active frame, hold or no hold. They stay out of the trial
 * snapshot (struct lec_belief). A frame whose own reference is silent carries
 * no estimate to judge, so it leaves the sums as they are.
 */
static void cohere_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	ec->cohere_cross += (t->cross_power - ec->cohere_cross) *
			    LEC_ERLE_ALPHA;
	ec->cohere_res += (t->res_power - ec->cohere_res) * LEC_ERLE_ALPHA;
	ec->cohere_est += (t->estimate_power - ec->cohere_est) *
			  LEC_ERLE_ALPHA;
}

/*
 * The readings of the coherence escape. ncc is the cross term over the square
 * root of the two energies. share is the estimate energy over the microphone
 * energy, the two energies plus twice the cross term. It returns false under
 * LEC_COHERE_POWER_MIN, on a sum that is not finite, or when rounding leaves
 * that microphone energy at or under zero.
 */
static bool cohere_ncc_get(const struct udsp_ec *ec, float *ncc,
			   float *share)
{
	float res = ec->cohere_res;
	float est = ec->cohere_est;
	float cross = ec->cohere_cross;
	float mic = res + est + 2.0f * cross;

	if (!(res >= LEC_COHERE_POWER_MIN) || !(res < INFINITY) ||
	    !(est >= LEC_COHERE_POWER_MIN) || !(est < INFINITY) ||
	    !(fabsf(cross) < INFINITY) || !(mic > 0.0f))
		return false;
	*ncc = cross / sqrtf(res * est);
	*share = est / mic;
	return true;
}

/*
 * An in-place radix-2 transform of n points, n a power of two. The twiddle
 * of each stage turns by a recurrence, so a stage costs one cosf and one
 * sinf.
 */
static void tone_fft(float *re, float *im, unsigned n)
{
	unsigned half;
	unsigned i;
	unsigned j;
	unsigned k;

	for (i = 1, j = 0; i < n; i++) {
		unsigned bit = n >> 1;
		float t;

		for (; j & bit; bit >>= 1)
			j ^= bit;
		j ^= bit;
		if (i >= j)
			continue;
		t = re[i];
		re[i] = re[j];
		re[j] = t;
		t = im[i];
		im[i] = im[j];
		im[j] = t;
	}
	for (half = 1; half < n; half <<= 1) {
		float step = -3.14159265f / (float)half;
		float sr = cosf(step);
		float si = sinf(step);

		for (k = 0; k < n; k += 2u * half) {
			float wr = 1.0f;
			float wi = 0.0f;

			for (j = 0; j < half; j++) {
				unsigned a = k + j;
				unsigned b = a + half;
				float tr = wr * re[b] - wi * im[b];
				float ti = wr * im[b] + wi * re[b];
				float t = wr * sr - wi * si;

				re[b] = re[a] - tr;
				im[b] = im[a] - ti;
				re[a] += tr;
				im[a] += ti;
				wi = wr * si + wi * sr;
				wr = t;
			}
		}
	}
}

/*
 * The share of the 200 to 3400 Hz energy of the reference frame back frames
 * before the newest in its two strongest peaks. The frame takes a Hann window
 * and zero padding to 128 points at 8 kHz and 256 at 16 kHz. The caller keeps
 * the frame inside the window. A frame with no energy in the band reads 0.
 */
static float ref_tone_share_get(const struct udsp_ec *ec, unsigned back)
{
	float re[LEC_TONE_FFT_MAX];
	float im[LEC_TONE_FFT_MAX];
	const float *x = ec->hist + ec->hist_pos + back * ec->frame;
	unsigned rate = ec->frame * 100u;
	unsigned n = 128u;
	unsigned lo;
	unsigned hi;
	unsigned i;
	unsigned p;
	float c1;
	float c0 = 1.0f;
	float cm;
	float total = 0.0f;
	float top = 0.0f;

	while (n < ec->frame)
		n <<= 1;
	c1 = cosf(2.0f * 3.14159265f / (float)(ec->frame - 1u));
	cm = c1;
	for (i = 0; i < n; i++) {
		float c;

		re[i] = 0.0f;
		im[i] = 0.0f;
		if (i >= ec->frame)
			continue;
		re[i] = x[i] * (0.5f - 0.5f * c0);
		c = 2.0f * c1 * c0 - cm;
		cm = c0;
		c0 = c;
	}
	tone_fft(re, im, n);
	lo = (LEC_TONE_BAND_LO_HZ * n + rate - 1u) / rate;
	hi = LEC_TONE_BAND_HI_HZ * n / rate;
	for (i = lo; i <= hi; i++) {
		re[i] = re[i] * re[i] + im[i] * im[i];
		total += re[i];
	}
	for (p = 0; p < 2u; p++) {
		unsigned best = lo;
		unsigned from;
		unsigned to;

		for (i = lo; i <= hi; i++) {
			if (re[i] > re[best])
				best = i;
		}
		from = best >= lo + LEC_TONE_PEAK_BINS ?
		       best - LEC_TONE_PEAK_BINS : lo;
		to = best + LEC_TONE_PEAK_BINS <= hi ?
		     best + LEC_TONE_PEAK_BINS : hi;
		for (i = from; i <= to; i++) {
			top += re[i];
			re[i] = 0.0f;
		}
	}
	return total > 0.0f ? top / total : 0.0f;
}

/*
 * The coherence run under a hold (LEC_COHERE_BAR). A frame with a silent
 * reference or with no reading leaves it. A talker, no hold, a stand-down of
 * this arm, a reading over the bar or the share, or a tonal reference clears
 * it.
 */
static bool cohere_get(struct udsp_ec *ec, bool active, bool talker)
{
	float share;
	float ncc;

	if (!active)
		return false;
	if (talker || !double_talk_get(ec) ||
	    (ec->standdown && (ec->standdown_arms & LEC_ARM_COHERE))) {
		ec->cohere_run = 0;
		return false;
	}
	if (!cohere_ncc_get(ec, &ncc, &share))
		return false;
	if (!(ncc <= LEC_COHERE_BAR) || !(share <= LEC_COHERE_SHARE_MAX) ||
	    ref_tone_share_get(ec, 0u) >= LEC_COHERE_TONE_SHARE) {
		ec->cohere_run = 0;
		return false;
	}
	ec->cohere_run++;
	return ec->cohere_run >= LEC_COHERE_FRAMES;
}

/* The stale run is due on an active frame outside a stand-down of its arm, and
 * it keeps its count through the stand-down. A signal that holds it up costs
 * one trial per stand-down plus trial. */
static bool stale_due_get(const struct udsp_ec *ec, bool active)
{
	if (!active)
		return false;
	if (ec->standdown && (ec->standdown_arms & LEC_ARM_STALE))
		return false;
	return ec->stale_run >= LEC_GEIGEL_STALE_FRAMES;
}

/*
 * One frame of the latch run: it grows unless the frame reads an
 * independent near talker over a trusted filter (the LEC_LATCH_NCC_BAND
 * comment). trusted is the trust of the frame.
 */
static void latch_run_grow(struct udsp_ec *ec, bool trusted)
{
	float ncc;
	float share;

	if (trusted && cohere_ncc_get(ec, &ncc, &share) &&
	    fabsf(ncc) <= LEC_LATCH_NCC_BAND &&
	    share >= LEC_LATCH_SHARE_MIN)
		return;
	ec->dt_run++;
}

/* Starts the trial of one arm: own_arm is its LEC_ARM_* bit, 0 for the latch
 * arm, and arm the word of the frame reading. */
static void probe_arm_start(struct udsp_ec *ec, unsigned own_arm,
			    enum udsp_ec_nlms_trial_arm arm)
{
	probe_start(ec, own_arm);
	ec->reading.trial_arm = (uint8_t)arm;
}

/*
 * The trial arms in order: path change, over-estimate, coherence, latch and
 * stale. When two are due on one frame, the first starts the trial; the
 * over-estimate run takes the path-change trial from scaled taps. No arm
 * re-arms mid-trial, which would replace the snapshot.
 */
static void probe_arm(struct udsp_ec *ec, bool active, bool talker)
{
	bool path_change;
	bool cohere;
	bool stale;
	bool over;

	if (ec->probe_frames)
		return;
	ec->over_scale = false;
	path_change = path_change_get(ec, active);
	over = over_get(ec, active, talker);
	cohere = cohere_get(ec, active, talker);
	stale = stale_due_get(ec, active);
	/* The latch arm waits for an active frame (the LEC_PROBE_FRAMES
	 * comment): a run that comes due in a far-end pause starts on the first
	 * frame that carries a reference again. */
	if (path_change || over) {
		probe_arm_start(ec, LEC_ARM_PATH, UDSP_EC_NLMS_TRIAL_ARM_PATH);
		ec->over_scale = over;
	} else if (cohere)
		probe_arm_start(ec, LEC_ARM_COHERE,
				UDSP_EC_NLMS_TRIAL_ARM_COHERE);
	else if (active && ec->dt_run >= LEC_GEIGEL_LATCH_MAX)
		probe_arm_start(ec, 0u, UDSP_EC_NLMS_TRIAL_ARM_LATCH);
	else if (stale)
		probe_arm_start(ec, LEC_ARM_STALE,
				UDSP_EC_NLMS_TRIAL_ARM_STALE);
	/* Spent after the arms read it: a stand-down at 1 on the frame a
	 * trial starts is still carried (the LEC_TRIAL_STANDDOWN_FRAMES
	 * comment). */
	if (active && ec->standdown)
		ec->standdown--;
}

/* The energy rule of the LEC_GEIGEL_TRUST_COEF_RATIO comment. coef_energy is
 * the energy of the filter whose estimate this frame judges, before the update
 * of the frame. The read costs no pass over the taps. */
static bool trust_rule_get(const struct udsp_ec *ec)
{
	return ec->coef_energy * LEC_GEIGEL_TRUST_COEF_RATIO >=
	       ec->belief.echo_gain;
}

static void trust_restart(struct udsp_ec *ec)
{
	ec->belief.trust_run = 0;
	ec->belief.trust_held = false;
}

/*
 * The trust of the detector on this frame (LEC_GEIGEL_TRUST_DB). A turn of the
 * energy rule restarts the count before the trust reads it. The turn can come
 * on the first far frame that a tone's filter does not explain. The count moves
 * after the detector (erle_update).
 */
static bool trust_update(struct udsp_ec *ec)
{
	bool rule = trust_rule_get(ec);

	if (rule && !ec->belief.trust_rule)
		trust_restart(ec);
	ec->belief.trust_rule = rule;
	/* The drop hold keeps the trust while the energy rule holds (the
	 * LEC_DROP_HOLD_DB comment). */
	if (ec->drop_hold && rule) {
		ec->belief.trust_held = true;
		return true;
	}
	if (!(ec->belief.erle_db >= LEC_GEIGEL_TRUST_DB) || !rule ||
	    ec->belief.trust_run < LEC_GEIGEL_TRUST_FRAMES)
		return false;
	ec->belief.trust_held = true;
	return true;
}

static unsigned run_decay(unsigned run)
{
	return run > LEC_GEIGEL_RUN_DECAY ? run - LEC_GEIGEL_RUN_DECAY : 0u;
}

/* The window stands under the bar of the frame and at or over the fixed bar,
 * so the far-end floor alone holds it inactive. */
static bool background_get(const struct udsp_ec *ec, const struct lec_tick *t)
{
	return !t->window_active && t->window_power >= ec->active_bar;
}

/* On frames whose own reference is active, not on silent frames inside an
 * active window. There the microphone carries no echo while the window maximum
 * holds the burst before, and the minimum would walk down. */
static void echo_gain_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	float ratio;

	ec->belief.gain_mic_avg += (t->mic_power - ec->belief.gain_mic_avg) *
			    LEC_GAIN_AVG_ALPHA;
	ec->belief.gain_win_avg += (t->window_power - ec->belief.gain_win_avg) *
			    LEC_GAIN_AVG_ALPHA;
	if (ec->belief.gain_warmup < LEC_GAIN_WARMUP_FRAMES) {
		ec->belief.gain_warmup++;
		return;
	}
	ratio = ec->belief.gain_mic_avg / ec->belief.gain_win_avg;
	if (ratio < ec->belief.echo_gain)
		ec->belief.echo_gain = ratio;
	else if (!double_talk_get(ec))
		ec->belief.echo_gain *= LEC_GEIGEL_GAIN_RISE;
	if (ec->belief.echo_gain < ec->gain_floor)
		ec->belief.echo_gain = ec->gain_floor;
}

/* A frame whose window maximum can stand on a few samples of a burst (the
 * LEC_GEIGEL_HANGOVER_SHORT comment): the onset frame of a window on the
 * classic arm, or a tail frame. hangover_get reads it on a fired frame, so
 * the window is active. */
static bool thin_frame_get(const struct udsp_ec *ec, const struct lec_tick *t)
{
	return (t->classic_arm && ec->window_run == 1u) || !t->active;
}

/*
 * The hangover that a verdict sets (LEC_GEIGEL_HANGOVER_SHORT). An escape
 * trial's verdict takes the short one. Outside one, a verdict inside a running
 * hangover takes the long one, apart from the second of a thin pair. No verdict
 * shortens a hangover.
 */
static unsigned hangover_get(const struct udsp_ec *ec,
			     const struct lec_tick *t, float mic_power,
			     float bar)
{
	if (escape_trial_get(ec))
		return LEC_GEIGEL_HANGOVER_SHORT;
	if (ec->belief.dt_hangover) {
		if (ec->belief.dt_thin && thin_frame_get(ec, t) &&
		    mic_power < bar * LEC_GEIGEL_PAIR_RATIO &&
		    mic_power < ec->noise_power * LEC_GEIGEL_SOFT_RATIO)
			return LEC_GEIGEL_HANGOVER_SHORT;
		return LEC_GEIGEL_HANGOVER;
	}
	if (thin_frame_get(ec, t))
		return LEC_GEIGEL_HANGOVER_SHORT;
	if (ec->dt_gap < LEC_GEIGEL_REPEAT_FRAMES)
		return LEC_GEIGEL_HANGOVER;
	if (mic_power >= bar * LEC_GEIGEL_MARGINAL_RATIO)
		return LEC_GEIGEL_HANGOVER;
	if (mic_power >= ec->noise_power * LEC_GEIGEL_SOFT_RATIO)
		return LEC_GEIGEL_HANGOVER;
	return LEC_GEIGEL_HANGOVER_SHORT;
}

/*
 * The floor of this frame's bars: the tracked floor, or on a frame that
 * carries no echo the pause level of LEC_PAUSE_RING within its cap. The call
 * also feeds the ring with the microphone of a quiet-window frame, and the
 * window flag follows the frame.
 */
static float bar_floor_get(struct udsp_ec *ec, struct lec_tick *t)
{
	float floor = ec->noise_power;
	float level = 0.0f;
	unsigned n = ec->pause_fill;
	unsigned k;
	float cap;

	t->no_echo = t->window_active &&
		     (!ec->window_was_active || !t->active ||
		      t->estimate_power <
		      ec->noise_power * LEC_GEIGEL_FLOOR_HEADROOM);
	for (k = 0; k < n; k++)
		level += ec->pause_ring[k];
	if (n)
		level /= (float)n;
	if (!t->window_active) {
		ec->pause_ring[ec->pause_pos] = t->mic_power;
		ec->pause_pos = (ec->pause_pos + 1u) % LEC_PAUSE_RING;
		if (ec->pause_fill < LEC_PAUSE_RING)
			ec->pause_fill++;
	}
	ec->window_was_active = t->window_active;
	cap = floor * LEC_PAUSE_LEVEL_CAP;
	if (!t->no_echo || !(level > floor))
		return floor;
	return level < cap ? level : cap;
}

static void doubletalk_update(struct udsp_ec *ec, struct lec_tick *t)
{
	float mic_power = t->mic_power;
	float window_power = t->window_power;
	float classic;
	float classic_bar;
	float misalign;
	float expected = t->estimate_power;
	float headroom;
	float like_bar;
	float bar;
	float floor;
	bool trusted;
	bool stale_clear;

	/* The trace reads the trackers as the detector read them. */
	t->noise_power = ec->noise_power;
	/* The window can end in noise_floor_update before nlp_apply reads the
	 * tail threshold, so the frame reads the window once, here. */
	t->implied_power = ec->noise_rise_window ? ec->noise_implied_power :
						   0.0f;
	t->erle_db = ec->belief.erle_db;
	t->hold_left = ec->belief.dt_hangover;
	t->dt_fired = false;
	floor = bar_floor_get(ec, t);
	/*
	 * The echo-likeness test of the processor reads the estimate before the
	 * classic merge, plus the floor of bar_floor_get. The floor
	 * takes the detector headroom, or the pause headroom on a quiet frame.
	 */
	headroom = t->window_active ? LEC_GEIGEL_FLOOR_HEADROOM :
				      LEC_GEIGEL_PAUSE_HEADROOM;
	like_bar = (expected + floor * headroom) * LEC_GEIGEL_MARGIN;
	t->echo_like = mic_power <= like_bar;
	/* On a quiet frame, a frame under this bound can be a peak of the
	 * floor (LEC_NLP_PEAK_RATIO, nlp_apply). */
	t->peak_like = mic_power <= like_bar * LEC_NLP_PEAK_RATIO;
	/* Inside the re-acquisition window the peak bound of a quiet frame
	 * reads the implied floor. So the pause content that raised the rise
	 * run does not refuse the resume. */
	if (!t->window_active && t->implied_power > floor)
		t->peak_like = mic_power <=
			       (expected + t->implied_power * headroom) *
			       LEC_GEIGEL_MARGIN * LEC_NLP_PEAK_RATIO;
	classic = echo_bound_get(ec, window_power);
	/*
	 * A converged filter leaves its misalignment orthogonal to the
	 * estimate. The expectation therefore adds the ERLE tracker's own
	 * ratio, capped at the microphone.
	 */
	misalign = ec->belief.res_power_avg < ec->belief.mic_power_avg ?
		   ec->belief.res_power_avg / ec->belief.mic_power_avg : 1.0f;
	expected *= 1.0f + misalign;
	classic_bar = (classic + ec->noise_power * LEC_GEIGEL_FLOOR_HEADROOM) *
		      LEC_GEIGEL_MARGIN;
	trusted = trust_update(ec);
	/* The frames since the trust last held, for the gate of over_get. */
	if (trusted)
		ec->trust_age = 0;
	else if (ec->trust_age <= LEC_OVER_TRUST_FRAMES)
		ec->trust_age++;
	/* The clears of the stale run, of the latch run and of the coherence
	 * run come before probe_arm reads the runs. The LEC_GEIGEL_STALE_FRAMES
	 * and LEC_TALKER_RATIO comments give the reasons. */
	stale_clear = !trusted ||
		      (t->window_active && mic_power >= classic_bar);
	if (stale_clear)
		ec->stale_run = 0;
	if (t->talker) {
		ec->dt_run = 0;
		ec->cohere_run = 0;
	}
	if (!t->trial_open)
		probe_arm(ec, t->active, t->talker);
	t->trusted = trusted;
	t->classic_arm = !trusted && classic > expected;
	if (t->classic_arm)
		expected = classic;
	/* A pure-echo frame carries the echo plus the near-end floor. The bar
	 * adds the floor of bar_floor_get with its headroom. */
	expected += floor * LEC_GEIGEL_FLOOR_HEADROOM;
	bar = expected * LEC_GEIGEL_MARGIN;
	t->bar = bar;
	if (t->window_active && mic_power >= bar) {
		t->dt_fired = true;
		ec->belief.dt_hangover = hangover_get(ec, t, mic_power, bar);
		ec->belief.dt_thin =
			ec->belief.dt_hangover == LEC_GEIGEL_HANGOVER_SHORT &&
			thin_frame_get(ec, t);
		ec->belief.dt_short =
			ec->belief.dt_hangover == LEC_GEIGEL_HANGOVER_SHORT;
		/* A short verdict on a thin frame leaves the repeat count
		 * running (the LEC_GEIGEL_HANGOVER_SHORT comment). */
		if (!ec->belief.dt_thin)
			ec->dt_gap = 0;
		latch_run_grow(ec, trusted);
		if (!stale_clear && !ec->drop_hold)
			ec->stale_run++;
		return;
	}
	if (ec->dt_gap < LEC_GEIGEL_REPEAT_FRAMES)
		ec->dt_gap++;
	if (ec->belief.dt_hangover)
		ec->belief.dt_hangover--;
	/* A far-end background frame under the raised bar leaves the latch run
	 * as it is. The decay serves the DTX gaps of far speech, and a
	 * background is still the far end. */
	if (t->background)
		return;
	if (!double_talk_get(ec)) {
		ec->dt_run = run_decay(ec->dt_run);
		if (t->window_active)
			ec->stale_run = run_decay(ec->stale_run);
		return;
	}
	latch_run_grow(ec, trusted);
}

/* One clear active frame into the ERLE tracker. The trust count is the
 * updates of the tracker since its last restart (the LEC_GEIGEL_TRUST_FRAMES
 * comment), so it moves here. */
static void erle_update(struct udsp_ec *ec, float mic_power, float res_power)
{
	float mic_avg = ec->belief.mic_power_avg;
	float res_avg = ec->belief.res_power_avg;

	mic_avg += (mic_power - mic_avg) * LEC_ERLE_ALPHA;
	res_avg += (res_power - res_avg) * LEC_ERLE_ALPHA;
	erle_set(ec, mic_avg, res_avg);
	if (ec->belief.trust_run < UINT16_MAX)
		ec->belief.trust_run++;
}

/* The floor as a power in the units of udsp_ec_nlms_frame_power_get, so the
 * detector can add it to an echo expectation. It is derived on the silent
 * frames that measure the level, so frames of speech pay nothing for it. */
static void noise_power_set(struct udsp_ec *ec)
{
	/* 127 is the sentinel of udsp_cn_level_get for a line with no
	 * measurable floor, not a very quiet one, so it derives no term. */
	if (ec->noise_dbov >= 127.0f) {
		ec->noise_power = 0.0f;
		return;
	}
	ec->noise_power = 32767.0f * 32767.0f *
			  powf(10.0f, ec->noise_dbov * -0.1f);
}

/* The one writer of the floor: its level, whether the level is a measured
 * floor, and the power the detector adds. 127 with valid false is no floor.
 * A measured floor ends the bound of a stale drop (noise_stale_update). */
static void noise_level_set(struct udsp_ec *ec, float dbov, bool valid)
{
	ec->noise_dbov = dbov;
	ec->noise_valid = valid;
	if (valid)
		ec->noise_drop_dbov = 0.0f;
	noise_power_set(ec);
}

/*
 * A run of active frames whose residual stands LEC_NOISE_STALE_DB under the
 * floor for LEC_NOISE_STALE_FRAMES drops the floor as stale. The residual
 * bounds the floor from above and never moves it. A floor that follows a
 * converging residual leaves the step control nothing to learn. The next
 * floor stands no louder than the dropped one.
 */
static void noise_stale_update(struct udsp_ec *ec, float level)
{
	if (!ec->noise_valid || level <= ec->noise_dbov + LEC_NOISE_STALE_DB) {
		ec->noise_stale_run = 0;
		return;
	}
	if (++ec->noise_stale_run < LEC_NOISE_STALE_FRAMES)
		return;
	ec->noise_stale_run = 0;
	memset(&ec->noise_agree, 0, sizeof(ec->noise_agree));
	ec->noise_drop_dbov = ec->noise_dbov;
	noise_level_set(ec, 127.0f, false);
}

static void noise_run_clear(struct noise_run *run)
{
	memset(run, 0, sizeof(*run));
}

/*
 * Adds one reading to the run. A run that reaches full readings is copied to
 * *done and cleared. The call then returns true when the spread of the run is
 * within LEC_NOISE_SPREAD_DB.
 */
static bool noise_run_add(struct noise_run *run, int level, unsigned full,
			  struct noise_run *done)
{
	if (!run->frames || level > run->quiet)
		run->quiet = level;
	if (!run->frames || level < run->loud)
		run->loud = level;
	run->sum += (float)level;
	if (++run->frames < full)
		return false;
	*done = *run;
	noise_run_clear(run);
	return (float)(done->quiet - done->loud) <= LEC_NOISE_SPREAD_DB;
}

/* True when a collecting rise run survives one more reading within
 * LEC_NOISE_RISE_DB of the floor (LEC_NOISE_CREEP_SHARE), which it counts. */
static bool noise_creep_kept(struct noise_run *run)
{
	if (!run->frames)
		return false;
	run->creep++;
	return LEC_NOISE_CREEP_SHARE * run->creep <= run->frames;
}

/*
 * Adds one reading to the agreement run (LEC_NOISE_AGREE_READINGS). A reading
 * that puts the run more than LEC_NOISE_AGREE_DB apart starts it over from
 * that reading. new_pause says that the reading is the first of its quiet
 * window. Returns true, with the mean of the run plus LEC_NOISE_AGREE_BIAS_DB
 * in *dbov, when the run is complete.
 */
static bool noise_agree_add(struct noise_run *run, int level, bool new_pause,
			    float *dbov)
{
	int quiet = run->frames && run->quiet > level ? run->quiet : level;
	int loud = run->frames && run->loud < level ? run->loud : level;

	if (run->frames && (float)(quiet - loud) > LEC_NOISE_AGREE_DB) {
		noise_run_clear(run);
		quiet = level;
		loud = level;
	}
	if (!run->frames || new_pause)
		run->pauses++;
	run->quiet = quiet;
	run->loud = loud;
	run->sum += (float)level;
	run->frames++;
	if (run->frames < LEC_NOISE_AGREE_READINGS ||
	    run->pauses < LEC_NOISE_AGREE_PAUSES)
		return false;
	*dbov = run->sum / (float)run->frames + LEC_NOISE_AGREE_BIAS_DB;
	return true;
}

/*
 * One reading of a quiet-window frame. new_pause says that the reading is
 * the first of its quiet window.
 */
static void noise_quiet_update(struct udsp_ec *ec, int level, bool new_pause)
{
	float gap = ec->noise_dbov - (float)level;
	struct noise_run done;
	float dbov;

	/*
	 * Digital silence is not a reading of a floor. If it took the first
	 * reading, the first real floor would reach the tracker only through
	 * a full rise run.
	 */
	if (level >= 127 && !ec->noise_valid)
		return;
	/* The first measurable reading is taken whole, but after a stale drop
	 * no louder than the dropped floor; the rise test guards a held
	 * floor. */
	if (!ec->noise_valid) {
		dbov = fmaxf((float)level, ec->noise_drop_dbov);
	} else if (gap < 0.0f) {
		dbov = ec->noise_dbov - gap * LEC_NOISE_ALPHA_DOWN;
	} else if (gap <= LEC_NOISE_RISE_DB) {
		dbov = ec->noise_dbov - gap * LEC_NOISE_ALPHA_UP;
		if (noise_creep_kept(&ec->noise_rise)) {
			noise_level_set(ec, dbov, true);
			return;
		}
	} else if (!noise_agree_add(&ec->noise_agree, level, new_pause,
				    &dbov)) {
		if (!noise_run_add(&ec->noise_rise, level,
				   LEC_NOISE_RISE_FRAMES, &done))
			return;
		dbov = done.sum / (float)done.frames;
	} else if (ec->noise_dbov - dbov > LEC_NOISE_AGREE_RISE_DB) {
		dbov = ec->noise_dbov - LEC_NOISE_AGREE_RISE_DB;
	}
	noise_run_clear(&ec->noise_rise);
	noise_run_clear(&ec->noise_agree);
	noise_level_set(ec, dbov, true);
}

/* The active frames falsify the rise run of the re-acquisition window: the
 * stale rule of noise_stale_update against the run's level. level is the
 * residual of an active frame in dBov; a frame that is not active restarts. */
static bool noise_rise_falsified(struct udsp_ec *ec, float level)
{
	struct noise_run *run = &ec->noise_rise;

	if (!ec->noise_rise_window || !run->frames)
		return false;
	if (level <= run->sum / (float)run->frames + LEC_NOISE_STALE_DB) {
		run->stale = 0;
		return false;
	}
	return ++run->stale >= LEC_NOISE_STALE_FRAMES;
}

/* The active frames falsify the agreement run, in and out of the window: the
 * stale rule against the run's level. level is the residual of an active frame
 * in dBov; a frame that is not active restarts the count. */
static bool noise_agree_falsified(struct noise_run *run, float level)
{
	if (!run->frames)
		return false;
	if (level <= run->sum / (float)run->frames + LEC_NOISE_STALE_DB) {
		run->stale = 0;
		return false;
	}
	return ++run->stale >= LEC_NOISE_STALE_FRAMES;
}

/*
 * The gates of the seed (LEC_NOISE_SEED_ERLE_DB): no floor, no
 * trial, a measurable level and the ERLE. It also needs the echo margin, the
 * microphone near the bar and the residual near its smoothed level. NaN takes
 * the false arm.
 */
static bool noise_seed_qualified(const struct udsp_ec *ec,
				 const struct lec_tick *t)
{
	float leftover;

	if (ec->noise_valid || ec->probe_frames || t->level >= 127)
		return false;
	if (!(ec->belief.erle_db >= LEC_NOISE_SEED_ERLE_DB))
		return false;
	leftover = t->estimate_power * powf(10.0f, ec->belief.erle_db * -0.1f);
	if (!(t->res_power >= leftover * LEC_NOISE_ECHO_MARGIN))
		return false;
	if (!(t->mic_power < t->bar * LEC_GEIGEL_SOFT_RATIO))
		return false;
	return t->res_power <= ec->belief.res_power_avg * LEC_GEIGEL_SOFT_RATIO;
}

/*
 * A seed is a first reading of the floor. Like the first reading of a quiet
 * window, it clears a rise run and an agreement run that the stale rule left
 * behind. It stands no louder than a dropped floor (noise_stale_update).
 */
static void noise_seed_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	struct noise_run done;

	if (!noise_seed_qualified(ec, t)) {
		noise_run_clear(&ec->noise_seed);
		return;
	}
	if (!noise_run_add(&ec->noise_seed, t->level, LEC_NOISE_SEED_FRAMES,
			   &done))
		return;
	noise_run_clear(&ec->noise_rise);
	noise_run_clear(&ec->noise_agree);
	noise_level_set(ec, fmaxf((float)done.quiet - LEC_NOISE_SEED_BIAS_DB,
				  ec->noise_drop_dbov), true);
}

/* One reading of the floor per frame, in its residual: an active frame can drop
 * a measured floor or seed an unmeasured one, and a quiet-window frame measures
 * it. Silent frames inside an active window only break the seed. */
static void noise_floor_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	unsigned age_max = ec->noise_rise_window ? LEC_NOISE_RISE_AGE_KEPT :
						  LEC_NOISE_RISE_AGE;
	bool quiet = !t->active && !t->window_active;
	bool new_pause = quiet && !ec->noise_quiet_last;

	ec->noise_quiet_last = quiet;
	if (ec->noise_rise_window)
		ec->noise_rise_window--;
	if (!ec->noise_rise_window)
		ec->noise_implied_power = 0.0f;
	if (ec->noise_rise.frames && ++ec->noise_rise.age > age_max)
		noise_run_clear(&ec->noise_rise);
	if (ec->noise_agree.frames && ++ec->noise_agree.age > age_max)
		noise_run_clear(&ec->noise_agree);
	if (t->active) {
		if (noise_rise_falsified(ec, (float)t->level)) {
			noise_run_clear(&ec->noise_rise);
			noise_run_clear(&ec->noise_agree);
		}
		if (noise_agree_falsified(&ec->noise_agree, (float)t->level))
			noise_run_clear(&ec->noise_agree);
		noise_stale_update(ec, (float)t->level);
		noise_seed_update(ec, t);
		return;
	}
	noise_run_clear(&ec->noise_seed);
	ec->noise_stale_run = 0;
	ec->noise_rise.stale = 0;
	ec->noise_agree.stale = 0;
	/* The frames after a talkspurt still carry tail echo through a filter
	 * that is not converged. The floor is read once the whole window is
	 * quiet. */
	if (t->window_active)
		return;
	noise_quiet_update(ec, t->level, new_pause);
}

static bool coef_diverged(const struct udsp_ec *ec, float energy)
{
	return !(isfinite(energy) && energy <= ec->coef_energy_max);
}

/*
 * A filter that diverges inside a trial is undone to the snapshot, which the
 * watchdog passed on every frame before the trial, not zeroed. The verdict
 * comes before the trial's own, which reads a smoothed residual that lags a
 * divergent frame. The counter counts it as a reset: the filter blew up.
 */
static bool probe_diverged(struct udsp_ec *ec)
{
	if (!coef_diverged(ec, coef_energy_track(ec)))
		return false;
	ec->diverge_resets++;
	probe_reject(ec, UDSP_EC_NLMS_TRIAL_END_DIVERGED);
	return true;
}

/* The tracker readings of the resume start over with the tracker: the ring
 * and the burst maxima (LEC_NLP_RESUME_MARGIN_DB). */
static void nlp_erle_clear(struct udsp_ec *ec)
{
	memset(ec->nlp_erle_ring, 0, sizeof(ec->nlp_erle_ring));
	ec->nlp_erle_pos = 0;
	ec->nlp_burst_erle = 0.0f;
	ec->nlp_pause_erle = 0.0f;
}

/*
 * Outside a trial no snapshot exists, so a diverged filter is zeroed. The live
 * powers reseed from the microphone, because the residual of a zeroed filter is
 * the microphone; the microphone power is finite, so no non-finite tracker
 * survives this point. Returns whether it zeroed the filter.
 */
static bool divergence_check(struct udsp_ec *ec, float mic_power)
{
	if (!coef_diverged(ec, coef_energy_track(ec)))
		return false;
	ec->diverge_resets++;
	memset(ec->coef, 0, sizeof(*ec->coef) * ec->taps);
	coef_energy_clear(ec);
	/* The sums of the coherence escape read the estimate of the filter
	 * this frame threw away (the runs_clear comment). */
	cohere_restart(ec);
	ec->belief.mic_power_avg = LEC_ERLE_FLOOR;
	ec->belief.res_power_avg = LEC_ERLE_FLOOR;
	ec->belief.erle_db = 0.0f;
	/* The cut flag of the resume stays, as the engage run does. */
	nlp_erle_clear(ec);
	ec->live_mic_power = mic_power;
	ec->live_res_power = mic_power;
	over_restart(ec);
	return true;
}

/* The gate of the combined-loss arm: LEC_NLP_BOOT_LOSS_DB in the bootstrap,
 * LEC_NLP_LOSS_DB after it (the LEC_NLP_BOOT_FRAMES comment). */
static float nlp_loss_gate_get(const struct udsp_ec *ec)
{
	return ec->nlp_boot < LEC_NLP_BOOT_FRAMES ? LEC_NLP_BOOT_LOSS_DB :
						    LEC_NLP_LOSS_DB;
}

/* The tone tail hold starts on the first silent-reference frame after an active
 * one. It needs an act of the processor on the frame before, and tonal frames
 * this far back and one further. The end frame carries the tone's last samples.
 */
#define LEC_TONE_TAIL_BACK	2u

/*
 * The tone tail hold, updated before the gate reads the frame: an unlike frame
 * passes while it runs. A filter trained on a tone can hold its model ahead of
 * the echo, so its estimate falls first when the tone stops. A dt frame still
 * releases the processor. The hold ends when the reference returns or the
 * window goes quiet; a tail under LEC_TONE_TAIL_BACK + 2 frames holds none.
 */
static void tone_tail_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	if (t->active || !t->window_active) {
		ec->tone_tail = false;
	} else if (ec->ref_was_active && ec->nlp_active &&
		   (LEC_TONE_TAIL_BACK + 2u) * ec->frame <= ec->taps) {
		ec->tone_tail =
			ref_tone_share_get(ec, LEC_TONE_TAIL_BACK) >=
				LEC_COHERE_TONE_SHARE &&
			ref_tone_share_get(ec, LEC_TONE_TAIL_BACK + 1u) >=
				LEC_COHERE_TONE_SHARE;
	}
	ec->ref_was_active = t->active;
}

/* 18 dB over the tracked floor: a microphone this loud in a far pause, where no
 * reference of the tail reaches it, reads a near end. That near end withdraws
 * the passes of an escape trial (pause_near_update). */
#define LEC_PAUSE_NEAR_RATIO	63.0957f

/* The near-end reading of the far pauses. It runs before the detector
 * overwrites the window flag of the frame before; a frame outside a pause and
 * the first quiet frame of a pause end the open pause. */
static void pause_near_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	bool near;

	if (t->window_active || ec->window_was_active) {
		ec->pause_open = false;
		return;
	}
	near = t->mic_power > ec->noise_power * LEC_PAUSE_NEAR_RATIO;
	if (!ec->pause_open) {
		ec->pause_near = near;
		ec->pause_open = true;
	} else if (near) {
		ec->pause_near = true;
	}
}

/* The drop hold holds a trusted filter on a near talker under its echo. Such a
 * talker drops the ERLE tracker and fires no verdict (drop_hold_update). */
/* The start: a feed would take the tracker down 6 dB or more from
 * LEC_DROP_HOLD_TRACKER_DB, and a pause run ended within
 * LEC_DROP_HOLD_PAUSE_FRAMES active frames. */
#define LEC_DROP_HOLD_DB		6.0f
#define LEC_DROP_HOLD_TRACKER_DB	28.0f
/* 2.5 dB: the start frame's microphone over its residual; its estimate stands
 * at most LEC_DROP_HOLD_EST_RATIO over the microphone. */
#define LEC_DROP_HOLD_FRAME_RATIO	1.7783f
#define LEC_DROP_HOLD_EST_RATIO		1.26f
#define LEC_DROP_HOLD_PAUSE_FRAMES	100u
/* A pause run reads a talker 15 dB or more over the floor. The hold ends on
 * LEC_DROP_HOLD_CLEAN_RUNS clean runs in a row or LEC_DROP_HOLD_FRAMES_MAX
 * active frames after its start or last talker run. */
#define LEC_DROP_HOLD_TALK_DB		15.0f
#define LEC_DROP_HOLD_CLEAN_RUNS	3u
#define LEC_DROP_HOLD_FRAMES_MAX	300u
/* The misfit end: 8 active frames in a row whose estimate holds
 * LEC_DROP_HOLD_MISFIT_EST or more of the microphone and whose residual stands
 * within LEC_DROP_HOLD_MISFIT_RATIO of it. */
#define LEC_DROP_HOLD_MISFIT_FRAMES	8u
#define LEC_DROP_HOLD_MISFIT_EST	0.25f
#define LEC_DROP_HOLD_MISFIT_RATIO	1.4125f
/* The coherence end (drop_cohere_update) reads a normalized correlation: on a
 * frame with lag sums (drop_grad_on), the largest smoothed sum squared reaches
 * this squared times the smoothed residual and lag energies. */
#define LEC_DROP_HOLD_COHERE_BAR	0.50f
/* Frames in a row that end the hold on that reading: a far-end copy at another
 * lag of the tail. The coherence escape compares with the estimate and cannot
 * see such a copy. */
#define LEC_DROP_HOLD_COHERE_FRAMES	4u

/* The drop that a feed of this frame would take from the tracker, in dB
 * (erle_update). */
static float drop_feed_get(const struct udsp_ec *ec,
			   const struct lec_tick *t)
{
	float mic_avg = ec->belief.mic_power_avg +
			(t->mic_power - ec->belief.mic_power_avg) *
			LEC_ERLE_ALPHA;
	float res_avg = ec->belief.res_power_avg +
			(t->res_power - ec->belief.res_power_avg) *
			LEC_ERLE_ALPHA;

	if (mic_avg < LEC_ERLE_FLOOR)
		mic_avg = LEC_ERLE_FLOOR;
	if (res_avg < LEC_ERLE_FLOOR)
		res_avg = LEC_ERLE_FLOOR;
	return ec->belief.erle_db - 10.0f * log10f(mic_avg / res_avg);
}

/* The start of the drop hold (the LEC_DROP_HOLD_DB comment), read after the
 * detector of the frame. */
static bool drop_hold_start_get(const struct udsp_ec *ec,
				const struct lec_tick *t)
{
	return !ec->drop_hold && t->trusted && t->active && !t->probing &&
	       !ec->probe_frames && !double_talk_get(ec) &&
	       !escape_trial_get(ec) && ec->drop_pause_seen &&
	       ec->drop_since_pause <= LEC_DROP_HOLD_PAUSE_FRAMES &&
	       t->mic_power >= LEC_DROP_HOLD_FRAME_RATIO * t->res_power &&
	       ec->belief.erle_db >= LEC_DROP_HOLD_TRACKER_DB &&
	       drop_feed_get(ec, t) >= LEC_DROP_HOLD_DB &&
	       t->estimate_power <= LEC_DROP_HOLD_EST_RATIO * t->mic_power;
}

static void drop_hold_start(struct udsp_ec *ec)
{
	ec->drop_hold = true;
	ec->drop_misfit_run = 0;
	memset(ec->grad_avg, 0, sizeof(*ec->grad_avg) * ec->taps);
	ec->drop_grad_res_avg = 0.0f;
	ec->drop_grad_ref_avg = 0.0f;
	ec->drop_cohere_run = 0;
	ec->drop_clean_runs = 0;
	ec->drop_since_talk = 0;
}

/* The reading of the LEC_DROP_HOLD_COHERE_BAR comment on a frame that
 * drop_grad_on marks, and the end of the hold it gives. */
static void drop_cohere_update(struct udsp_ec *ec)
{
	float loudest = 0.0f;
	unsigned k;

	if (!ec->drop_grad_on || !ec->drop_hold)
		return;
	for (k = 0; k < ec->taps; k++) {
		ec->grad_avg[k] += (ec->grad[k] - ec->grad_avg[k]) *
				   LEC_ERLE_ALPHA;
		if (fabsf(ec->grad_avg[k]) > loudest)
			loudest = fabsf(ec->grad_avg[k]);
	}
	ec->drop_grad_res_avg += (ec->drop_grad_res - ec->drop_grad_res_avg) *
				 LEC_ERLE_ALPHA;
	ec->drop_grad_ref_avg += (ec->drop_grad_ref - ec->drop_grad_ref_avg) *
				 LEC_ERLE_ALPHA;
	if (ec->drop_grad_res_avg > 0.0f && ec->drop_grad_ref_avg > 0.0f &&
	    loudest * loudest >= LEC_DROP_HOLD_COHERE_BAR *
				 LEC_DROP_HOLD_COHERE_BAR *
				 ec->drop_grad_res_avg *
				 ec->drop_grad_ref_avg) {
		if (++ec->drop_cohere_run >= LEC_DROP_HOLD_COHERE_FRAMES)
			ec->drop_hold = false;
	} else {
		ec->drop_cohere_run = 0;
	}
}

/* A quiet-window frame into the running pause run, the first frame left
 * out. */
static void drop_pause_add(struct udsp_ec *ec, const struct lec_tick *t)
{
	if (ec->drop_pause_len++ >= 1u) {
		ec->drop_pause_res += t->res_power;
		ec->drop_pause_floor += ec->noise_power;
		ec->drop_pause_n++;
	}
}

/*
 * The end of the pause run on the first frame of an active window: a talker
 * run or a clean one. The floor of the run counts at least one unit of power
 * per frame. A floor of digital silence then does not read every residual as
 * a talker.
 */
static void drop_pause_end(struct udsp_ec *ec)
{
	if (ec->drop_pause_n) {
		float floor = ec->drop_pause_floor > (float)ec->drop_pause_n ?
			      ec->drop_pause_floor : (float)ec->drop_pause_n;
		float excess = 10.0f * log10f(ec->drop_pause_res / floor +
					      1e-9f);

		ec->drop_pause_seen = true;
		ec->drop_since_pause = 0;
		if (excess >= LEC_DROP_HOLD_TALK_DB) {
			ec->drop_since_talk = 0;
			ec->drop_clean_runs = 0;
		} else if (ec->drop_hold &&
			   ++ec->drop_clean_runs >= LEC_DROP_HOLD_CLEAN_RUNS) {
			ec->drop_hold = false;
		}
	}
	ec->drop_pause_len = 0;
	ec->drop_pause_n = 0;
	ec->drop_pause_res = 0.0f;
	ec->drop_pause_floor = 0.0f;
}

/* The misfit end of the drop hold (the LEC_DROP_HOLD_DB comment) on an active
 * frame. An active frame that is not a misfit frame clears the run, so the
 * misfit frames must come in a row. */
static void drop_misfit_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	if (!ec->drop_hold)
		return;
	if (t->estimate_power >= LEC_DROP_HOLD_MISFIT_EST * t->mic_power &&
	    t->mic_power < LEC_DROP_HOLD_MISFIT_RATIO * t->res_power) {
		if (++ec->drop_misfit_run >= LEC_DROP_HOLD_MISFIT_FRAMES)
			ec->drop_hold = false;
	} else {
		ec->drop_misfit_run = 0;
	}
}

/*
 * The evidence of the drop hold on a frame, after the filter and before the
 * detector. A talker leaves energy in the far pauses and echo does not, so
 * clean pause runs end the hold. A residual coherent with the reference, misfit
 * frames and LEC_DROP_HOLD_FRAMES_MAX end it too (the counts here).
 */
static void drop_hold_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	drop_cohere_update(ec);
	if (!t->window_active) {
		drop_pause_add(ec, t);
		return;
	}
	drop_pause_end(ec);
	if (!t->active)
		return;
	if (ec->drop_since_pause < UINT16_MAX)
		ec->drop_since_pause++;
	if (ec->drop_since_talk < UINT16_MAX)
		ec->drop_since_talk++;
	drop_misfit_update(ec, t);
	if (ec->drop_hold && ec->drop_since_talk >= LEC_DROP_HOLD_FRAMES_MAX)
		ec->drop_hold = false;
}

/* The pass of an escape trial over the hold: a frame that did not fire, is
 * echo-like and reads a live ERLE of LEC_NLP_ERLE_MIN_DB or more. The pause
 * reading withdraws it first (nlp_gate_get). */
static bool escape_pass_get(const struct udsp_ec *ec,
			    const struct lec_tick *t)
{
	return escape_trial_get(ec) && !t->dt_fired && t->echo_like &&
	       ec->live_mic_power >= ec->live_res_power *
				     powf(10.0f, LEC_NLP_ERLE_MIN_DB * 0.1f);
}

/*
 * The processor replaces the residual only when there is echo to replace and
 * the detector confirmed that nothing else is on the line. It reads the tail
 * window, so the echo of a talkspurt that just ended is covered. Without it,
 * the comfort noise replaces a caller the moment the far end stops.
 */
static enum nlp_verdict nlp_gate_get(const struct udsp_ec *ec,
				     const struct lec_tick *t)
{
	float loss_db;

	if (!ec->nlp_on)
		return NLP_VERDICT_OFF;
	if (!t->window_active)
		return NLP_VERDICT_QUIET;
	/* A near end in the latest pause withdraws every pass of an escape
	 * trial (the LEC_PAUSE_NEAR_RATIO comment). */
	if (escape_trial_get(ec) && ec->pause_near)
		return NLP_VERDICT_DT;
	/* A frame of an escape trial that its own evidence shows to be echo
	 * passes over the hold (the escape_trial_get comment). */
	if (escape_pass_get(ec, t))
		return NLP_VERDICT_PASS;
	if (double_talk_get(ec))
		return NLP_VERDICT_DT;
	/* An unlike frame in the tail of a far tone is that tone's echo
	 * (the LEC_TONE_TAIL_BACK comment). */
	if (!t->echo_like)
		return ec->tone_tail ? NLP_VERDICT_PASS : NLP_VERDICT_UNLIKE;
	if (ec->belief.erle_db >= LEC_NLP_ERLE_DB)
		return NLP_VERDICT_PASS;
	if (ec->belief.erle_db < LEC_NLP_ERLE_MIN_DB)
		return NLP_VERDICT_ERLE;
	/* The combined-loss arm: ERLE plus the hybrid loss of the learned gain
	 * with headroom, the bound the detector reads, plus the reference loss.
	 * Its gate is lower in the bootstrap (nlp_loss_gate_get). */
	loss_db = ec->belief.erle_db -
		  10.0f * log10f(ec->belief.echo_gain *
				 LEC_GEIGEL_GAIN_HEADROOM);
	loss_db += ec->ref_loss_db;
	return loss_db >= nlp_loss_gate_get(ec) ? NLP_VERDICT_PASS :
						  NLP_VERDICT_ERLE;
}

/*
 * One count per frame: nlp_active says only whether the processor acted on
 * the last frame, and the counts say why it stood out of the others. An
 * eligible frame counts as a wait until the engage run completes. A frame
 * with the processor switched off counts nowhere, because no condition
 * refused it.
 */
static void nlp_count(struct udsp_ec *ec, const struct lec_tick *t)
{
	switch (t->nlp_verdict) {
	case NLP_VERDICT_OFF:
		return;
	case NLP_VERDICT_QUIET:
		ec->nlp_quiet_frames++;
		return;
	case NLP_VERDICT_DT:
		ec->nlp_dt_frames++;
		if (t->dt_fired)
			ec->nlp_dt_fired_frames++;
		return;
	case NLP_VERDICT_UNLIKE:
		ec->nlp_unlike_frames++;
		return;
	case NLP_VERDICT_ERLE:
		ec->nlp_erle_frames++;
		return;
	case NLP_VERDICT_PASS:
		break;
	}
	if (ec->nlp_active)
		ec->nlp_active_frames++;
	else
		ec->nlp_wait_frames++;
}

/*
 * The level in dBov at which cn_fill draws and raises the noise of a 16 kHz
 * line; applied after the interpolator, the frame's level holds from the first
 * sample. It avoids clipping and keeps rounding noise negligible.
 */
#define LEC_CN_DRAW_DBOV	15

/*
 * One frame of comfort noise at level_dbov into out. On a 16 kHz line the
 * noise is drawn at 8 kHz and raised by the interpolator. It then holds the
 * band of an 8 kHz line (the LEC_CN_RAISE_DB comment).
 */
static void cn_fill(struct udsp_ec *ec, int level_dbov, int16_t *out)
{
	int16_t half[UDSP_FRAME_MAX / 2u];
	int16_t raised[UDSP_FRAME_MAX];
	float gain;
	unsigned i;

	if (ec->frame <= UDSP_FRAME_SAMPLES) {
		udsp_cn_generate(&ec->cn, level_dbov, out, ec->frame);
		return;
	}
	/* The clamp of udsp_cn_generate: 127 is silence. */
	if (level_dbov < 0)
		level_dbov = 0;
	if (level_dbov > 127)
		level_dbov = 127;
	udsp_cn_generate(&ec->cn, LEC_CN_DRAW_DBOV, half, ec->frame / 2u);
	udsp_resamp_up2(&ec->cn_up, half, ec->frame / 2u, raised);
	gain = powf(10.0f, (float)(LEC_CN_DRAW_DBOV - level_dbov) / 20.0f);
	for (i = 0; i < ec->frame; i++)
		out[i] = udsp_sat16_round(gain * (float)raised[i]);
}

/* The floor of the tail threshold: the tracked floor, or the implied floor of
 * the frame when it is louder. */
static float nlp_tail_floor_get(const struct lec_tick *t)
{
	if (t->implied_power > t->noise_power)
		return t->implied_power;
	return t->noise_power;
}

/* The pause figures of the resume on a quiet frame (LEC_NLP_RESUME_DB). The
 * first quiet frame takes over the act and the largest tracker reading of the
 * burst before. A tail frame over the bound counts only 12 dB or more over the
 * floor. */
static void nlp_pause_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	if (!ec->nlp_in_pause) {
		ec->nlp_pause_acted = ec->nlp_burst_acted;
		ec->nlp_burst_acted = false;
		ec->nlp_pause_peak = false;
		ec->nlp_pause_frames = 0;
		ec->nlp_pause_erle = ec->nlp_burst_erle;
	}
	if (ec->nlp_pause_frames <= LEC_NLP_TAIL_QUIET_FRAMES)
		ec->nlp_pause_frames++;
	if (t->noise_power <= 0.0f)
		ec->nlp_pause_peak = true;
	else if (!t->peak_like &&
		 (ec->nlp_pause_frames > LEC_NLP_TAIL_QUIET_FRAMES ||
		  t->mic_power >=
			  nlp_tail_floor_get(t) * LEC_NLP_TAIL_FLOOR_RATIO))
		ec->nlp_pause_peak = true;
}

/*
 * The resume after a short hold: after a short hangover, the first
 * pass frame resumes a complete run at LEC_NLP_RESUME_DB, outside an escape
 * trial. The evidence is the pause it ran through, or an act on its burst
 * before it. Any other refusal ends the hold.
 */
static void nlp_hold_update(struct udsp_ec *ec, enum nlp_verdict verdict)
{
	if (verdict == NLP_VERDICT_DT) {
		if (!ec->belief.dt_short || escape_trial_get(ec)) {
			ec->nlp_hold_short = false;
			return;
		}
		if (!ec->nlp_hold_short) {
			ec->nlp_hold_short = true;
			ec->nlp_hold_pause = ec->nlp_in_pause;
			ec->nlp_hold_acted = ec->nlp_burst_acted;
		}
		return;
	}
	if (verdict == NLP_VERDICT_QUIET) {
		if (ec->nlp_hold_short)
			ec->nlp_hold_pause = true;
		return;
	}
	ec->nlp_hold_short = false;
}

/* Whether the first pass frame after a short hold resumes a complete engage
 * run (the nlp_hold_update comment). */
static bool nlp_hold_resume_get(const struct udsp_ec *ec)
{
	if (!ec->nlp_hold_short)
		return false;
	if (ec->nlp_hold_pause ?
	    !ec->nlp_pause_acted || ec->nlp_pause_peak : !ec->nlp_hold_acted)
		return false;
	return ec->belief.erle_db >= LEC_NLP_RESUME_DB;
}

/* The ERLE condition of the resume after a far pause: LEC_NLP_RESUME_DB, or
 * the climb or the dip of the LEC_NLP_RESUME_MARGIN_DB comment. The ring slot
 * of the frame still holds the reading LEC_NLP_RESUME_CLIMB_FRAMES frames
 * before it (nlp_erle_note). */
static bool nlp_resume_erle_get(const struct udsp_ec *ec)
{
	float erle = ec->belief.erle_db;

	if (erle >= LEC_NLP_RESUME_DB)
		return true;
	if (ec->nlp_run_cut ||
	    erle < LEC_NLP_RESUME_DB - LEC_NLP_RESUME_MARGIN_DB)
		return false;
	return ec->nlp_pause_erle >= LEC_NLP_RESUME_DB ||
	       erle > ec->nlp_erle_ring[ec->nlp_erle_pos];
}

/* Whether the first pass frame after a far pause resumes a complete engage
 * run (the LEC_NLP_RESUME_DB comment). */
static bool nlp_resume_get(const struct udsp_ec *ec)
{
	return ec->nlp_in_pause && ec->nlp_pause_acted &&
	       !ec->nlp_pause_peak && nlp_resume_erle_get(ec);
}

/* The tracker readings of the resume, after the resume read them: the ring
 * takes the frame's reading. A frame that is not quiet raises the largest
 * reading of its burst, and the first one after a pause starts it. */
static void nlp_erle_note(struct udsp_ec *ec, enum nlp_verdict verdict)
{
	float erle = ec->belief.erle_db;

	ec->nlp_erle_ring[ec->nlp_erle_pos] = erle;
	if (++ec->nlp_erle_pos == LEC_NLP_RESUME_CLIMB_FRAMES)
		ec->nlp_erle_pos = 0;
	if (verdict != NLP_VERDICT_QUIET &&
	    (ec->nlp_in_pause || erle > ec->nlp_burst_erle))
		ec->nlp_burst_erle = erle;
}

/* The cut flag of the resume, after the engage run took the frame: a complete
 * run clears it. A dt, unlike or erle frame that lowered the run under run, its
 * count before the frame, sets it. */
static void nlp_cut_update(struct udsp_ec *ec, enum nlp_verdict verdict,
			   unsigned run)
{
	if (ec->nlp_run >= LEC_NLP_ON_FRAMES)
		ec->nlp_run_cut = false;
	else if (ec->nlp_run < run &&
		 (verdict == NLP_VERDICT_DT || verdict == NLP_VERDICT_UNLIKE ||
		  verdict == NLP_VERDICT_ERLE))
		ec->nlp_run_cut = true;
}

/*
 * The source of a frame's cross-fade. On a frame that starts an act, it is
 * the input or the residual, the one whose fade emits less energy. On other
 * frames it is the residual. The fade emits (1 - mix) * source + mix * noise
 * per sample, as in nlp_apply. Each energy is the sum of its squares over the
 * frame, in int16 units. An estimate that leads the echo makes the fade from
 * the residual the louder one.
 */
static const int16_t *nlp_fade_source_get(const struct udsp_ec *ec,
					  const struct lec_tick *t,
					  float from, float target,
					  const int16_t *out,
					  const int16_t *noise)
{
	float in_energy = 0.0f;
	float out_energy = 0.0f;
	unsigned i;

	if (from != 0.0f)
		return out;
	for (i = 0; i < ec->frame; i++) {
		float mix = from + (target - from) *
			    ((float)(i + 1u) / ec->frame);
		float a = (1.0f - mix) * (float)t->input[i] +
			  mix * (float)noise[i];
		float b = (1.0f - mix) * (float)out[i] + mix * (float)noise[i];

		in_energy += a * a;
		out_energy += b * b;
	}
	return in_energy < out_energy ? t->input : out;
}

static void nlp_apply(struct udsp_ec *ec, struct lec_tick *t, int16_t *out)
{
	enum nlp_verdict verdict;
	int16_t noise[UDSP_FRAME_MAX];
	const int16_t *src;
	float target;
	float cn_dbov;
	float from = ec->nlp_active ? 1.0f : 0.0f;
	int level = t->level;
	bool run_kept;
	unsigned run;
	unsigned i;

	/* The frame counts into the bootstrap before the gate reads it. */
	if (t->active && ec->nlp_boot < LEC_NLP_BOOT_FRAMES)
		ec->nlp_boot++;
	verdict = nlp_gate_get(ec, t);
	t->nlp_verdict = verdict;
	/* The pass of an escape trial acts at once (the escape_trial_get
	 * comment). */
	if (verdict == NLP_VERDICT_PASS && escape_pass_get(ec, t))
		ec->nlp_run = LEC_NLP_ON_FRAMES;
	/* The first pass frame after a far pause can resume a complete run
	 * (the LEC_NLP_RESUME_DB comment). */
	if (verdict == NLP_VERDICT_QUIET)
		nlp_pause_update(ec, t);
	if (verdict == NLP_VERDICT_PASS && nlp_resume_get(ec))
		ec->nlp_run = LEC_NLP_ON_FRAMES;
	/* The first pass frame after a short hold can resume one too (the
	 * nlp_hold_update comment). */
	if (verdict == NLP_VERDICT_PASS && nlp_hold_resume_get(ec))
		ec->nlp_run = LEC_NLP_ON_FRAMES;
	nlp_hold_update(ec, verdict);
	nlp_erle_note(ec, verdict);
	ec->nlp_in_pause = verdict == NLP_VERDICT_QUIET;
	if (verdict == NLP_VERDICT_OFF) {
		ec->nlp_burst_acted = false;
		ec->nlp_pause_acted = false;
		ec->nlp_pause_peak = false;
		ec->nlp_burst_erle = 0.0f;
		ec->nlp_pause_erle = 0.0f;
	}
	/* The engage run: an echo-like quiet frame or a floor peak within the
	 * misses keeps it. A dt, unlike or erle frame cuts a complete run, and
	 * other refusals zero it. */
	run_kept = verdict == NLP_VERDICT_QUIET &&
		   (t->echo_like ||
		    (t->noise_power > 0.0f && t->peak_like &&
		     ec->nlp_quiet_misses < LEC_NLP_QUIET_MISSES));
	if (verdict != NLP_VERDICT_QUIET || t->echo_like)
		ec->nlp_quiet_misses = 0;
	else if (ec->nlp_quiet_misses <= LEC_NLP_QUIET_MISSES)
		ec->nlp_quiet_misses++;
	run = ec->nlp_run;
	if (verdict == NLP_VERDICT_PASS ||
	    (verdict == NLP_VERDICT_ERLE && !ec->nlp_engaged)) {
		if (ec->nlp_run < LEC_NLP_ON_FRAMES)
			ec->nlp_run++;
	} else if (!run_kept) {
		ec->nlp_run = verdict != NLP_VERDICT_QUIET &&
			      verdict != NLP_VERDICT_OFF &&
			      ec->nlp_run >= LEC_NLP_ON_FRAMES ?
			      LEC_NLP_DECAY_FRAMES : 0;
	}
	nlp_cut_update(ec, verdict, run);
	target = verdict == NLP_VERDICT_PASS &&
		 ec->nlp_run >= LEC_NLP_ON_FRAMES ? 1.0f : 0.0f;
	ec->nlp_active = target > 0.0f;
	if (ec->nlp_active)
		ec->nlp_burst_acted = true;
	if (ec->nlp_active && !ec->nlp_engaged &&
	    ++ec->nlp_acted >= LEC_NLP_FIRST_FRAMES)
		ec->nlp_engaged = true;
	nlp_count(ec, t);
	if (target == 0.0f && from == 0.0f)
		return;
	/* Comfort noise at the floor raised by LEC_CN_RAISE_DB, never louder
	 * than the residual. A floor seeded or read on near speech can be
	 * stale, and a line with no measured floor gets silence. */
	cn_dbov = ec->noise_valid ? ec->noise_dbov - LEC_CN_RAISE_DB :
				    ec->noise_dbov;
	if ((float)level < cn_dbov)
		level = (int)lrintf(cn_dbov);
	/*
	 * In the steady engaged state the noise replaces the frame whole: at
	 * a mix of exactly 1 the fade below returns each noise sample as it
	 * is, so the frame takes the noise directly and skips the loop.
	 */
	if (from == target) {
		cn_fill(ec, level, out);
		return;
	}
	cn_fill(ec, level, noise);
	/* One frame of cross-fade at a gate edge, because a switch in one
	 * sample clicks. */
	src = nlp_fade_source_get(ec, t, from, target, out, noise);
	t->fade_input = src == t->input;
	for (i = 0; i < ec->frame; i++) {
		float mix = from + (target - from) *
			((float)(i + 1u) / ec->frame);

		out[i] = udsp_sat16_round((1.0f - mix) * (float)src[i] +
					  mix * (float)noise[i]);
	}
}

/*
 * The rules of the udsp_ec_nlms_ref_loss_set comment in dsp/lec.h. The
 * arithmetic stays in float, and the loss of 0 takes powf(10, -0), which is
 * exactly 1.0f, so every field then equals its constant bit for bit.
 */
static void ref_loss_scale_set(struct udsp_ec *ec, unsigned half_db)
{
	float db = (float)half_db / 2.0f;
	float g2 = powf(10.0f, db * -0.1f);

	ec->ref_loss_half_db = half_db;
	ec->ref_loss_db = db;
	ec->active_bar = LEC_REF_ACTIVE_POWER * g2;
	ec->gain_min = LEC_GEIGEL_GAIN_MIN / g2;
	ec->gain_floor = LEC_GEIGEL_GAIN_FLOOR / g2;
	ec->talker_ratio = LEC_TALKER_RATIO / g2;
	ec->coef_energy_max = LEC_COEF_ENERGY_MAX / g2;
	ec->step_floor_scale = g2;
}

static struct udsp_ec *ec_create_rate(unsigned tail_ms, unsigned rate_hz)
{
	struct udsp_ec *ec;
	unsigned tail_max;
	unsigned taps;

	if (rate_hz != UDSP_SAMPLE_RATE && rate_hz != UDSP_SAMPLE_RATE_WB)
		return NULL;
	tail_max = rate_hz == UDSP_SAMPLE_RATE_WB ? LEC_TAIL_MS_WB_MAX :
						    LEC_TAIL_MS_MAX;
	/* 0 is the contract sentinel of udsp_chan_cfg.ec_tail_ms in the stable
	 * udsp.h, "0 = default"; every other value out of range clamps. */
	if (!tail_ms)
		tail_ms = rate_hz == UDSP_SAMPLE_RATE_WB ? LEC_TAIL_MS_WB_DEFAULT :
							   LEC_TAIL_MS_DEFAULT;
	if (tail_ms < LEC_TAIL_MS_MIN)
		tail_ms = LEC_TAIL_MS_MIN;
	if (tail_ms > tail_max)
		tail_ms = tail_max;
	taps = tail_ms * (rate_hz / 1000u);
	ec = calloc(1, sizeof(*ec) + sizeof(float) * mem_floats_get(taps));
	if (!ec)
		return NULL;
	ec->frame = rate_hz / 100u;
	ec->taps = taps;
	ec->tail_ms = tail_ms;
	ec->coef = ec->mem;
	ec->backup = ec->mem + taps;
	ec->probe = ec->mem + 2u * taps;
	ec->grad = ec->mem + 3u * taps;
	ec->grad_avg = ec->mem + 4u * taps;
	ec->hist = ec->mem + 5u * taps;
	ec->power_floor = LEC_POWER_PER_TAP * (float)taps;
	ec->window_frames = (taps + ec->frame - 1u) / ec->frame;
	ref_loss_scale_set(ec, 0);
	/* The reset is the one definition of the adaptive start state. */
	udsp_ec_nlms_reset(ec);
	/* Silence until a far-end pause or a seed measures the real floor. */
	noise_level_set(ec, 127.0f, false);
	/* The analog line profile: the default is EC on with NLP. */
	ec->nlp_on = true;
	ec->zero_skip_on = true;
	ec->ref_floor_on = true;
	udsp_cn_init(&ec->cn, 0x1ec5eedu);
	udsp_resamp_reset(&ec->cn_up);
	return ec;
}

static struct udsp_ec *ec_create(unsigned tail_ms)
{
	return ec_create_rate(tail_ms, UDSP_SAMPLE_RATE);
}

static void ec_destroy(struct udsp_ec *ec)
{
	free(ec);
}

static uint16_t u16_sat(unsigned v)
{
	return v > UINT16_MAX ? UINT16_MAX : (uint16_t)v;
}

static void window_run_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	if (!t->window_active)
		ec->window_run = 0;
	else if (ec->window_run < UINT16_MAX)
		ec->window_run++;
}

/* An onset frame that carries the near end: the first frame of an active
 * window, with a microphone LEC_GEIGEL_SOFT_RATIO or more over the floor. */
static bool onset_near_get(const struct udsp_ec *ec, const struct lec_tick *t)
{
	return ec->window_run == 1u &&
	       t->mic_power >= ec->noise_power * LEC_GEIGEL_SOFT_RATIO;
}

/*
 * The onset frame that takes a bare trial's step floor outside a trial (section
 * 3): onset_near_get, and a peak tap at or past the frame length or the onset
 * peak. An echo inside the frame needs its step. The bootstrap must be over or
 * the processor off, because a filter that converges in it keeps more of a
 * quiet talker. ec_run reads it on the state that the frame starts with.
 */
static bool onset_floor_get(const struct udsp_ec *ec, const struct lec_tick *t)
{
	return onset_near_get(ec, t) &&
	       (ec->coef_peak_tap >= ec->frame || ec->onset_peak) &&
	       (ec->nlp_boot >= LEC_NLP_BOOT_FRAMES || !ec->nlp_on);
}

/*
 * The onset peak, on the peak tap that the frame ends with. A plain onset step
 * (plain) on an untrusted filter sets it. That step takes the peak tap from at
 * or past the frame length (peak0) to under it. LEC_ONSET_PEAK_CLEAR_FRAMES
 * frames in a row with the peak at or past it clear it, and so does a trusted
 * frame. The same frames clear onset_zero.
 */
static void onset_peak_update(struct udsp_ec *ec, const struct lec_tick *t,
			      unsigned peak0, bool plain)
{
	if (t->trusted) {
		ec->onset_peak = false;
		ec->onset_zero = false;
		ec->onset_far_run = 0;
	} else if (ec->coef_peak_tap >= ec->frame) {
		if (ec->onset_far_run < LEC_ONSET_PEAK_CLEAR_FRAMES)
			ec->onset_far_run++;
		if (ec->onset_far_run >= LEC_ONSET_PEAK_CLEAR_FRAMES) {
			ec->onset_peak = false;
			ec->onset_zero = false;
		}
	} else {
		ec->onset_far_run = 0;
		if (plain && peak0 >= ec->frame)
			ec->onset_peak = true;
	}
}

/* The restart of the trust count on a far return after a pause, once the window
 * state is final and before the detector. The return frame can fire on the
 * estimate of a tone, so the detector judges it without the trust. */
static void quiet_run_update(struct udsp_ec *ec, const struct lec_tick *t)
{
	if (!t->window_active) {
		if (ec->quiet_run < LEC_GEIGEL_TRUST_PAUSE_FRAMES)
			ec->quiet_run++;
		return;
	}
	if (ec->quiet_run >= LEC_GEIGEL_TRUST_PAUSE_FRAMES &&
	    !ec->belief.trust_held)
		trust_restart(ec);
	ec->quiet_run = 0;
}

/* The trial fields of the frame reading, cleared before a talker can end a
 * trial; probe_arm, probe_reject and probe_finish write them mid-frame. A frame
 * starts a trial or ends one, never both, so one clear serves both. */
static void reading_trial_clear(struct udsp_ec *ec)
{
	ec->reading.trial_arm = UDSP_EC_NLMS_TRIAL_ARM_NONE;
	ec->reading.trial_end = UDSP_EC_NLMS_TRIAL_END_NONE;
}

/* The frame reading from the record, at the end of the frame. The trial
 * fields stay as the frame wrote them. */
static void reading_fill(struct udsp_ec *ec, const struct lec_tick *t)
{
	ec->reading.frames = ec->frames;
	ec->reading.mic_power = t->mic_power;
	ec->reading.window_power = t->window_power;
	ec->reading.estimate_power = t->estimate_power;
	ec->reading.bar = t->bar;
	ec->reading.noise_power = t->noise_power;
	ec->reading.ref_power = t->ref_power;
	ec->reading.erle_db = t->erle_db;
	ec->reading.ref_bar = t->ref_bar;
	ec->reading.ref_bar_max = t->bar_max;
	ec->reading.window_run = u16_sat(ec->window_run);
	ec->reading.dt_run = u16_sat(ec->dt_run);
	ec->reading.stale_run = u16_sat(ec->stale_run);
	ec->reading.trust_run = u16_sat(ec->belief.trust_run);
	ec->reading.hold_left = (uint8_t)t->hold_left;
	ec->reading.nlp_verdict = (uint8_t)t->nlp_verdict;
	ec->reading.dt_fired = t->dt_fired;
	ec->reading.classic_arm = t->classic_arm;
	ec->reading.trusted = t->trusted;
	ec->reading.probing = t->trial_open;
	ec->reading.ref_quiet = t->window_active && !t->active;
	ec->reading.talker = t->talker;
	ec->reading.background = t->background;
	ec->reading.drop_hold = ec->drop_hold;
	ec->reading.fade_input = t->fade_input;
}

static void ec_run(struct udsp_ec *ec, int16_t *mic, const int16_t *ref)
{
	struct frame_undo undo;
	struct lec_tick t;
	float step_floor;
	unsigned peak0;
	bool onset_plain;
	bool zero;
	bool adapt;

	memset(&t, 0, sizeof(t));
	t.ref_power = udsp_ec_nlms_frame_power_get(ref, ec->frame);
	t.mic_power = udsp_ec_nlms_frame_power_get(mic, ec->frame);
	t.bar_max = ref_bar_max_get(ec);
	t.trial_open = ec->probe_frames != 0u;
	/* Read before the frame pushes anything: the exact energy of the
	 * history window (hist_push) and of the frame, both zero only when
	 * every sample the tap loops would read is zero. */
	zero = ec->zero_skip_on && ec->hist_energy == 0.0 &&
	       t.ref_power == 0.0f;
	window_push(ec, t.ref_power);
	t.window_power = window_power_get(ec);
	t.ref_bar = ref_bar_get(ec, t.bar_max, t.trial_open);
	t.window_active = t.window_power >= t.ref_bar;
	t.talker = talker_get(ec, &t);
	ec->window_prev = t.window_power;
	reading_trial_clear(ec);
	/* A talker ends a running trial before the frame adapts, and the frame
	 * is held whatever the restored hangover says; the bar follows the end.
	 * The end counts no divergence reset. */
	t.held = t.talker && t.trial_open;
	if (t.held) {
		probe_reject(ec, UDSP_EC_NLMS_TRIAL_END_TALKER);
		t.ref_bar = ref_bar_get(ec, t.bar_max, false);
		t.window_active = t.window_power >= t.ref_bar;
	}
	window_run_update(ec, &t);
	quiet_run_update(ec, &t);
	t.background = background_get(ec, &t);
	ref_floor_add(&ec->ref_floor, t.ref_power);
	t.active = t.ref_power >= t.ref_bar;
	t.probing = ec->probe_frames != 0u;
	adapt = t.active && !ec->freeze_on && !t.held &&
		(t.probing || (!double_talk_get(ec) && !ec->drop_hold));
	/* The denominator floor of this frame: power_floor plus the tracked
	 * near-end floor over the taps (the LEC_POWER_PER_TAP comment), which
	 * step_floor_scale scales by the reference loss. */
	step_floor = (ec->power_floor + (float)ec->taps * ec->noise_power) *
		     ec->step_floor_scale;
	/* The peak tap that the frame starts with, and an onset frame with the
	 * near end that takes the plain step, for onset_peak_update. */
	peak0 = ec->coef_peak_tap;
	onset_plain = !t.probing && onset_near_get(ec, &t) &&
		      !onset_floor_get(ec, &t);
	/* The plain onset step of a zeroed filter. The tracked floor can still
	 * read the line before a near noise. The first samples of a burst then
	 * divide that noise by the floor alone. A floor under the microphone
	 * bounds what the step takes from the near end. */
	if (onset_plain && ec->onset_zero) {
		float zero = (ec->power_floor + (float)ec->taps * t.mic_power *
			      LEC_ONSET_ZERO_FLOOR) * ec->step_floor_scale;

		if (zero > step_floor)
			step_floor = zero;
	}
	/* A trial on taps without echo evidence, and outside a trial the onset
	 * frame of onset_floor_get: a full window of this frame's reference,
	 * less the history the window holds (the LEC_PROBE_FRAMES comment). */
	if (t.probing ? ec->probe_bare : onset_floor_get(ec, &t)) {
		float full = (float)ec->taps * t.ref_power -
			     (float)ec->hist_energy;

		if (full > step_floor)
			step_floor = full;
	}
	/* A held frame sums the evidence of the reference coherence. It does
	 * not adapt, so its block gains are free for drop_grad_add. */
	ec->drop_grad_on = ec->drop_hold && t.active && !t.probing;
	if (ec->drop_grad_on) {
		unsigned b;

		memset(ec->grad, 0, sizeof(*ec->grad) * ec->taps);
		for (b = 0; b < LEC_IPNLMS_BLOCKS_MAX; b++)
			ec->block_gain[b] = 1.0f;
		ec->drop_grad_res = 0.0f;
		ec->drop_grad_ref = 0.0f;
	}
	frame_undo_save(ec, mic, &undo);
	t.input = undo.mic;
	if (adapt)
		memcpy(ec->backup, ec->coef, sizeof(*ec->coef) * ec->taps);
	if (zero) {
		t.res_power = frame_zero_run(ec, mic, ref);
		ec->skip_frames++;
	} else {
		t.res_power = frame_filter_run(ec, mic, ref, adapt,
					       LEC_STEP * step_scale_get(ec) *
					       talk_scale_get(ec, t.mic_power,
							      t.window_power),
					       step_floor, &t.estimate_power,
					       &t.cross_power);
	}
	/* Whatever the verdict below does with the frame's update, the
	 * coefficients moved, and a rollback moves them again. */
	if (adapt)
		ec->coef_dirty = true;
	t.estimate_power /= ec->frame;
	t.res_power /= ec->frame;
	t.cross_power /= ec->frame;
	drop_hold_update(ec, &t);
	/*
	 * Measured every frame, hold or no hold: these are the only powers a
	 * held instance still sees, and the probe judges its trial by them.
	 * The estimate feeds the over-estimate run (over_get).
	 */
	ec->live_mic_power += (t.mic_power - ec->live_mic_power) *
			      LEC_ERLE_ALPHA;
	ec->live_res_power += (t.res_power - ec->live_res_power) *
			      LEC_ERLE_ALPHA;
	ec->live_est_power += (t.estimate_power - ec->live_est_power) *
			      LEC_ERLE_ALPHA;
	if (t.active) {
		cohere_update(ec, &t);
		echo_gain_update(ec, &t);
	}
	pause_near_update(ec, &t);
	doubletalk_update(ec, &t);
	if (drop_hold_start_get(ec, &t))
		drop_hold_start(ec);
	/* A frame of near speech adapted before the detector judged it, so its
	 * update rolls back and its output follows (frame_undo). A trial frame
	 * is exempt, and the start frame of the drop hold rolls back too. */
	if (adapt && (double_talk_get(ec) || ec->drop_hold) && !t.probing) {
		memcpy(ec->coef, ec->backup, sizeof(*ec->coef) * ec->taps);
		ec->coef_dirty = true;
		adapt = false;
		frame_rerun(ec, mic, ref, &undo);
	}
	/* The snapshot of a trial that starts on this frame, and its evidence
	 * (probe_bare), follow the rollback, so neither reads an undone
	 * update. */
	if (!t.probing && ec->probe_frames) {
		memcpy(ec->probe, ec->coef, sizeof(*ec->coef) * ec->taps);
		ec->probe_bare = coef_energy_track(ec) *
				 LEC_GEIGEL_TRUST_COEF_RATIO <
				 ec->belief.echo_gain;
		ec->probe_judged = true;
	}
	/* A trial of the over-estimate run adapts from scaled taps, and the
	 * snapshot above keeps the taps the frame ends with. */
	if (!t.probing && ec->probe_frames && ec->over_scale)
		over_scale_apply(ec);
	if (!adapt)
		ec->hold_frames++;
	/* The trial's budget counts adapted frames, so a far pause or a freeze
	 * cannot spend it on a filter that stands still. */
	if (t.probing && adapt && !probe_diverged(ec) &&
	    --ec->probe_frames == 0u)
		probe_finish(ec);
	/* A frame of an escape trial leaves the tracker to the filter it
	 * describes (the escape_trial_get comment). */
	if (t.active && !double_talk_get(ec) && !escape_trial_get(ec))
		erle_update(ec, t.mic_power, t.res_power);
	/* The level of the residual, read once for the floor tracker and the
	 * processor: nothing writes the frame between the two. */
	t.level = udsp_cn_level_get(mic, ec->frame);
	noise_floor_update(ec, &t);
	/* A filter that the watchdog zeroes keeps no onset peak and no count of
	 * it: the zeroing, not a step, put its peak tap under the frame. It
	 * starts from zero again. */
	if (divergence_check(ec, t.mic_power)) {
		ec->drop_hold = false;
		ec->onset_peak = false;
		ec->onset_zero = true;
		ec->onset_far_run = 0;
	} else {
		onset_peak_update(ec, &t, peak0, onset_plain && adapt);
	}
	tone_tail_update(ec, &t);
	nlp_apply(ec, &t, mic);
	ec->frames++;
	reading_fill(ec, &t);
}

static void ec_freeze(struct udsp_ec *ec, bool on)
{
	ec->freeze_on = on;
}

static void ec_nlp(struct udsp_ec *ec, bool on)
{
	ec->nlp_on = on;
}

unsigned udsp_ec_nlms_tail_get(const struct udsp_ec *ec)
{
	if (!ec)
		return 0;
	return ec->tail_ms;
}

void udsp_ec_nlms_info_get(const struct udsp_ec *ec,
			   struct udsp_ec_nlms_info *info)
{
	memset(info, 0, sizeof(*info));
	if (!ec)
		return;
	info->tail_ms = ec->tail_ms;
	info->taps = ec->taps;
	info->frames = ec->frames;
	info->hold_frames = ec->hold_frames;
	info->skip_frames = ec->skip_frames;
	info->diverge_resets = ec->diverge_resets;
	info->probes = ec->probes;
	info->coef_energy = ec->coef_energy;
	info->erle_db = ec->belief.erle_db;
	info->echo_gain = ec->belief.echo_gain;
	info->double_talk = double_talk_get(ec);
	info->nlp_active = ec->nlp_active;
	info->nlp_active_frames = ec->nlp_active_frames;
	info->nlp_wait_frames = ec->nlp_wait_frames;
	info->nlp_erle_frames = ec->nlp_erle_frames;
	info->nlp_unlike_frames = ec->nlp_unlike_frames;
	info->nlp_dt_frames = ec->nlp_dt_frames;
	info->nlp_quiet_frames = ec->nlp_quiet_frames;
	info->coef_peak_tap = ec->coef_peak_tap;
	info->noise_dbov = ec->noise_dbov;
	info->nlp_dt_fired_frames = ec->nlp_dt_fired_frames;
	info->ref_floor = ref_floor_get(&ec->ref_floor);
}

void udsp_ec_nlms_frame_get(const struct udsp_ec *ec,
			    struct udsp_ec_nlms_frame *frame)
{
	memset(frame, 0, sizeof(*frame));
	if (!ec)
		return;
	*frame = ec->reading;
}

const char *udsp_ec_nlms_verdict_name(unsigned verdict)
{
	switch ((enum nlp_verdict)verdict) {
	case NLP_VERDICT_OFF:
		return "off";
	case NLP_VERDICT_QUIET:
		return "quiet";
	case NLP_VERDICT_DT:
		return "dt";
	case NLP_VERDICT_UNLIKE:
		return "unlike";
	case NLP_VERDICT_ERLE:
		return "erle";
	case NLP_VERDICT_PASS:
		return "pass";
	}
	return "?";
}

bool udsp_ec_nlms_path_set(enum udsp_ec_nlms_path path)
{
	if (!udsp_ec_nlms_path_name(path))
		return false;
#if defined(__aarch64__)
	lec_simd_on = path == UDSP_EC_NLMS_PATH_SIMD;
	return true;
#else
	return path == UDSP_EC_NLMS_PATH_SCALAR;
#endif
}

enum udsp_ec_nlms_path udsp_ec_nlms_path_get(void)
{
#if defined(__aarch64__)
	return lec_simd_on ? UDSP_EC_NLMS_PATH_SIMD : UDSP_EC_NLMS_PATH_SCALAR;
#else
	return UDSP_EC_NLMS_PATH_SCALAR;
#endif
}

void udsp_ec_nlms_zero_skip_set(struct udsp_ec *ec, bool on)
{
	if (ec)
		ec->zero_skip_on = on;
}

bool udsp_ec_nlms_zero_skip_get(const struct udsp_ec *ec)
{
	return ec && ec->zero_skip_on;
}

void udsp_ec_nlms_ref_floor_set(struct udsp_ec *ec, bool on)
{
	if (ec)
		ec->ref_floor_on = on;
}

bool udsp_ec_nlms_ref_floor_get(const struct udsp_ec *ec)
{
	return ec && ec->ref_floor_on;
}

const char *udsp_ec_nlms_path_name(enum udsp_ec_nlms_path path)
{
	switch (path) {
	case UDSP_EC_NLMS_PATH_SCALAR:
		return "scalar";
	case UDSP_EC_NLMS_PATH_SIMD:
		return "simd";
	}
	return NULL;
}

void udsp_ec_nlms_reset(struct udsp_ec *ec)
{
	if (!ec)
		return;
	memset(ec->mem, 0, sizeof(float) * mem_floats_get(ec->taps));
	memset(ec->window_power, 0, sizeof(ec->window_power));
	ec->window_index = 0;
	ec->window_prev = 0.0f;
	ec->hist_pos = 0;
	ec->hist_energy = 0.0;
	ec->belief.dt_hangover = 0;
	ec->belief.dt_thin = false;
	ec->belief.dt_short = false;
	ec->dt_run = 0;
	ec->dt_gap = LEC_GEIGEL_REPEAT_FRAMES;
	ec->path_run = 0;
	ec->stale_run = 0;
	ec->standdown = 0;
	ec->standdown_arms = 0;
	ec->trial_owed = 0;
	ec->probe_frames = 0;
	ec->probe_bare = false;
	ec->probe_judged = false;
	ec->probe_ratio_before = 0.0f;
	memset(&ec->probe_state, 0, sizeof(ec->probe_state));
	ec->drop_hold = false;
	ec->drop_pause_seen = false;
	ec->drop_grad_on = false;
	ec->drop_pause_len = 0;
	ec->drop_pause_n = 0;
	ec->drop_pause_res = 0.0f;
	ec->drop_pause_floor = 0.0f;
	ec->drop_since_pause = 0;
	ec->drop_since_talk = 0;
	ec->drop_clean_runs = 0;
	ec->drop_misfit_run = 0;
	ec->drop_cohere_run = 0;
	ec->drop_grad_res_avg = 0.0f;
	ec->drop_grad_ref_avg = 0.0f;
	/* The processor restarts disengaged like everything else: an engaged
	 * state kept across a reset cross-fades the first frame after it into
	 * comfort noise. */
	ec->nlp_active = false;
	ec->nlp_run = 0;
	ec->nlp_acted = 0;
	ec->nlp_engaged = false;
	udsp_resamp_reset(&ec->cn_up);
	/* The bootstrap of the combined-loss arm counts from NEW alone: a reset
	 * of an instance that ran a frame spends it (LEC_NLP_BOOT_FRAMES). */
	if (ec->frames)
		ec->nlp_boot = LEC_NLP_BOOT_FRAMES;
	ec->nlp_quiet_misses = 0;
	ec->nlp_in_pause = false;
	ec->nlp_burst_acted = false;
	ec->nlp_pause_acted = false;
	ec->nlp_pause_peak = false;
	ec->nlp_pause_frames = 0;
	ec->nlp_hold_short = false;
	ec->nlp_hold_pause = false;
	ec->nlp_hold_acted = false;
	nlp_erle_clear(ec);
	ec->nlp_run_cut = false;
	ec->ref_was_active = false;
	ec->tone_tail = false;
	ec->pause_pos = 0;
	ec->pause_fill = 0;
	ec->window_was_active = false;
	ec->pause_open = false;
	ec->pause_near = false;
	ec->belief.gain_warmup = 0;
	/* The floor survives a reset; the runs of evidence about it do not,
	 * because the filter and the reference alignment that they read are
	 * gone. */
	ec->noise_stale_run = 0;
	noise_run_clear(&ec->noise_rise);
	noise_run_clear(&ec->noise_seed);
	noise_run_clear(&ec->noise_agree);
	ec->noise_rise_window = 0;
	ec->noise_implied_power = 0.0f;
	ec->noise_quiet_last = false;
	ec->belief.gain_mic_avg = 0.0f;
	ec->belief.gain_win_avg = 0.0f;
	ec->belief.echo_gain = ec->gain_min;
	ec->frames = 0;
	ec->hold_frames = 0;
	ec->skip_frames = 0;
	ec->nlp_active_frames = 0;
	ec->nlp_wait_frames = 0;
	ec->nlp_erle_frames = 0;
	ec->nlp_unlike_frames = 0;
	ec->nlp_dt_frames = 0;
	ec->nlp_quiet_frames = 0;
	ec->nlp_dt_fired_frames = 0;
	ec->window_run = 0;
	ec->belief.trust_run = 0;
	ec->quiet_run = 0;
	ec->belief.trust_rule = false;
	ec->belief.trust_held = false;
	memset(&ec->reading, 0, sizeof(ec->reading));
	ec->live_mic_power = 0.0f;
	ec->live_res_power = 0.0f;
	over_restart(ec);
	cohere_restart(ec);
	ec->trial_own = 0;
	ec->belief.mic_power_avg = LEC_ERLE_FLOOR;
	ec->belief.res_power_avg = LEC_ERLE_FLOOR;
	ec->belief.erle_db = 0.0f;
	coef_energy_clear(ec);
	ec->onset_peak = false;
	ec->onset_zero = true;
	ec->onset_far_run = 0;
	/* Lifetime telemetry, both floors (the reference timing changes
	 * neither), the reference loss, the freeze and the switches survive; a
	 * new far-end history would hold the fixed bar for 300 frames. */
}

void udsp_ec_nlms_ref_loss_set(struct udsp_ec *ec, unsigned half_db)
{
	if (!ec)
		return;
	if (half_db > UDSP_EC_NLMS_REF_LOSS_MAX)
		half_db = UDSP_EC_NLMS_REF_LOSS_MAX;
	if (half_db == ec->ref_loss_half_db)
		return;
	ref_loss_scale_set(ec, half_db);
	/* A feed fills the histories before the first run, and a reset keeps
	 * the floor, so every history starts over with a changed loss. */
	udsp_ec_nlms_reset(ec);
	memset(&ec->ref_floor, 0, sizeof(ec->ref_floor));
}

unsigned udsp_ec_nlms_ref_loss_get(const struct udsp_ec *ec)
{
	return ec ? ec->ref_loss_half_db : 0u;
}

/*
 * The history intake of run() alone: the window ring, the window maximum, the
 * window flag at run()'s bar, the far-end floor and one hist_push per sample.
 * It ends the open pause of the near-end reading, and a far frame ends the
 * quiet window of the floor tracker; the pause ring takes nothing.
 */
void udsp_ec_nlms_ref_feed(struct udsp_ec *ec, const int16_t *ref)
{
	float power;
	float bar;
	unsigned i;

	if (!ec)
		return;
	power = udsp_ec_nlms_frame_power_get(ref, ec->frame);
	window_push(ec, power);
	ec->window_prev = window_power_get(ec);
	bar = ref_bar_get(ec, ref_bar_max_get(ec), ec->probe_frames != 0u);
	ec->window_was_active = ec->window_prev >= bar;
	ec->ref_was_active = power >= bar;
	if (ec->ref_was_active)
		ec->noise_quiet_last = false;
	ec->tone_tail = false;
	ec->pause_open = false;
	ref_floor_add(&ec->ref_floor, power);
	for (i = 0; i < ec->frame; i++)
		hist_push(ec, ref[i]);
}

const struct udsp_ec_ops udsp_ec_nlms = {
	.name = "nlms",
	.create = ec_create,
	.destroy = ec_destroy,
	.run = ec_run,
	.freeze = ec_freeze,
	.nlp = ec_nlp,
	.create_rate = ec_create_rate,
};
