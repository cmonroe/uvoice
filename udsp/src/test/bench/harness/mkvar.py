#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Write a canceller variant: a copy of a canceller source with a rule
changed, for a measurement. The copy goes to var/<name>/lec.c of the work
directory; the source stays unchanged.

usage: mkvar.py <name> [--from LEC_C] [--define NAME=VALUE ...]
                [--rule RULE ...]

--from names the source (default dsp/lec.c of this tree). --define sets the
value of a #define of the source; the line must exist once. The rules:

- bare: the step floor of a trial on taps without echo evidence (a full
  window of the frame's reference less the history energy) applies outside
  a trial too, on every frame while the taps fail the energy rule of the
  trust (trust_rule_get). The onset frames of onset_floor_get keep the
  floor as in the source, so outside a trial the floor applies on the
  union of the two sets of frames;
- tone-off: cohere_get drops the tonal test of the reference
  (ref_tone_share_get against LEC_COHERE_TONE_SHARE), and a tonal reference
  then leaves the coherence run as it is;
- cohere-est=K: during a coherence run, a frame whose microphone stands
  over K times the estimate zeroes the run.

Three rules act from the frame that UDSP_VAR_FROM_FRAME names at run time
(the frame count of ec->frames; 0 when the variable is not set). The
harness sets it to the frame of the path change of a conv case, and to the
frame of the dt phase of a dt case (work.switch_env):

- scales-off: the step is LEC_STEP, with no step scale (step_scale_get,
  talk_scale_get);
- hold-off: every active frame adapts at the step of its scales, with no
  hold of the detector, no rollback and no trial start;
- raw: a plain NLMS. Every frame adapts at LEC_STEP, with the denominator
  floor of LEC_POWER_PER_TAP per tap alone, no hold, no rollback and no
  trial start.

scales-off and hold-off combine; raw takes neither.

Three rules change the update for a measurement of a candidate. They write
the scalar form of the tap loops only, so their builds measure the host:

- ipnlms=A[:approx][:bN]: a proportionate update. Each frame takes a gain
  per tap from the coefficients, (1 - A) / 2 + (1 + A) / 2 times the tap
  count times the tap's share of the l1 norm, a mean of 1 (uniform for a
  zero filter), and the step of a sample divides by the window energy that
  the gains weight, plus the denominator floor. A = -1 is the plain NLMS.
  approx divides by the plain window energy instead. bN gives each block of
  N taps the mean gain of its taps. cG caps each gain at G. gate takes the
  gains only on the frames of a trial of the path-change or the coherence
  escape, and gain 1 on the others; gatea also takes them after a kept
  trial of those arms, until the trust holds;
- stepfast=MU:N[:trial]: the step is MU in place of LEC_STEP on a frame
  after a frame whose trust did not hold, and on the N frames after a kept
  trial; with trial, also on each frame of a trial. The step scales still
  apply;
- keep-early=F:R: a trial that has adapted on F frames or more ends kept
  as soon as its live ratio stands at or under R times the ratio before
  it (R at most LEC_PROBE_IMPROVE).

Each edit replaces text that must occur exactly once in the source. An
anchor that is missing or repeated ends the program with status 1 and names
the edit, so a rule that does not fit its source fails before a build.
"""
import argparse
import re
import sys

import work

COHERE_GATE = '\tif (!cohere_ncc_get(ec, &ncc, &share))\n\t\treturn false;\n'
DETECTOR = '\tdoubletalk_update(ec, &t);\n\t/*\n\t * The detector compares'
BARE_OLD = '\tif (t.probing ? ec->probe_bare : onset_floor_get(ec, &t)) {\n'
BARE_NEW = ('\tif (t.probing ? ec->probe_bare :\n'
            '\t    (!trust_rule_get(ec) || onset_floor_get(ec, &t))) {\n')
TONE_OLD = ('\tif (!(ncc <= LEC_COHERE_BAR) || '
            '!(share <= LEC_COHERE_SHARE_MAX) ||\n'
            '\t    ref_tone_share_get(ec) >= LEC_COHERE_TONE_SHARE) {\n')
TONE_NEW = ('\tif (!(ncc <= LEC_COHERE_BAR) ||\n'
            '\t    !(share <= LEC_COHERE_SHARE_MAX)) {\n')
EST_VARS = ('static float var_mic_power;\n'
            'static float var_est_power;\n'
            'static bool cohere_get(struct udsp_ec *ec, bool active, '
            'bool talker)\n')
COHERE_HEAD = ('static bool cohere_get(struct udsp_ec *ec, bool active, '
               'bool talker)\n')
FIRST_FN = 'static float echo_bound_factor_get(const struct udsp_ec *ec)\n{\n'
VAR_ON = '''static unsigned var_from_get(void)
{
	static int known;
	static unsigned from;

	if (!known) {
		const char *text = getenv("UDSP_VAR_FROM_FRAME");

		from = text ? (unsigned)strtoul(text, NULL, 10) : 0u;
		known = 1;
	}
	return from;
}

static bool var_on(const struct udsp_ec *ec)
{
	return ec->frames >= var_from_get();
}

'''
ADAPT = ('\tadapt = t.active && !ec->freeze_on && !t.held &&\n'
         '\t\t(t.probing || !double_talk_get(ec));\n')
MU = ('\t\t\t\t\t       LEC_STEP * step_scale_get(ec) *\n'
      '\t\t\t\t\t       talk_scale_get(ec, t.mic_power,\n'
      '\t\t\t\t\t\t\t      t.window_power),\n')
MU_OFF = ('\t\t\t\t\t       (var_on(ec) ? LEC_STEP :\n'
          '\t\t\t\t\t       LEC_STEP * step_scale_get(ec) *\n'
          '\t\t\t\t\t       talk_scale_get(ec, t.mic_power,\n'
          '\t\t\t\t\t\t\t      t.window_power)),\n')
ROLLBACK = '\tif (adapt && double_talk_get(ec) && !t.probing) {\n'
ROLLBACK_OFF = ('\tif (adapt && double_talk_get(ec) && !t.probing &&\n'
                '\t    !var_on(ec)) {\n')
ARMS = ('\tif (ec->probe_frames)\n\t\treturn;\n'
        '\tpath_change = path_change_get(ec, active);\n')
ARMS_OFF = ('\tif (ec->probe_frames || var_on(ec))\n\t\treturn;\n'
            '\tpath_change = path_change_get(ec, active);\n')
FLOOR = '\t\tif (full > step_floor)\n\t\t\tstep_floor = full;\n\t}\n'
FLOOR_RAW = FLOOR + ('\tif (var_on(ec))\n'
                     '\t\tstep_floor = (float)ec->taps * '
                     'LEC_POWER_PER_TAP;\n')


def replace_once(text, old, new, what):
    count = text.count(old)
    if count != 1:
        sys.exit(f'mkvar.py: {what}: anchor {old.strip()[:50]!r} occurs '
                 f'{count} times, 1 expected')
    return text.replace(old, new)


def define_set(text, name, value):
    """text with the value of #define name set to value."""
    pat = re.compile(r'^(#define[ \t]+' + re.escape(name) + r'[ \t]+)(\S.*)$',
                     re.M)
    found = pat.findall(text)
    if len(found) != 1:
        sys.exit(f'mkvar.py: --define {name}: {len(found)} definitions, 1 '
                 f'expected')
    return pat.sub(lambda m: m.group(1) + value, text)


def rule_apply(text, rule):
    """text with one rule applied."""
    name, _, arg = rule.partition('=')
    if name == 'bare':
        return replace_once(text, BARE_OLD, BARE_NEW, rule)
    if name == 'tone-off':
        return replace_once(text, TONE_OLD, TONE_NEW, rule)
    if name == 'cohere-est' and arg:
        text = replace_once(text, COHERE_HEAD, EST_VARS, rule)
        text = replace_once(
            text, COHERE_GATE, COHERE_GATE +
            '\tif (var_mic_power > %sf * var_est_power) {\n'
            '\t\tec->cohere_run = 0;\n\t\treturn false;\n\t}\n' % float(arg),
            rule)
        return replace_once(
            text, DETECTOR, '\tvar_mic_power = t.mic_power;\n'
            '\tvar_est_power = t.estimate_power;\n' + DETECTOR, rule)
    if name in ('scales-off', 'hold-off', 'raw'):
        return switch_rule(text, name)
    if name in ('ipnlms', 'stepfast', 'keep-early') and arg:
        return candidate_rule(text, name, arg)
    sys.exit(f'mkvar.py: unknown rule {rule!r}')


FILTER_HEAD = 'static float frame_filter_run(struct udsp_ec *ec, int16_t *mic,\n'
IPNLMS = '''static float var_gain[LEC_TAPS_MAX];

#if VAR_GATE
static bool var_gate_get(const struct udsp_ec *ec)
{
	if (ec->probe_frames &&
	    (ec->trial_own & (LEC_ARM_PATH | LEC_ARM_COHERE)))
		return true;
	return VAR_GATE == 2 && ec->var_fast;
}
#endif

static void var_gain_set(const struct udsp_ec *ec)
{
	float l1 = 0.0f;
	unsigned k;

	for (k = 0; k < ec->taps; k++)
		l1 += fabsf(ec->coef[k]);
	for (k = 0; k < ec->taps; k++)
		var_gain[k] = l1 > 0.0f ?
			(1.0f - VAR_ALPHA) * 0.5f + (1.0f + VAR_ALPHA) * 0.5f *
			(float)ec->taps * fabsf(ec->coef[k]) / l1 : 1.0f;
	for (k = 0; VAR_BLOCK > 1u && k < ec->taps; k += VAR_BLOCK) {
		float sum = 0.0f;
		unsigned j;

		for (j = k; j < k + VAR_BLOCK && j < ec->taps; j++)
			sum += var_gain[j];
		sum /= (float)(j - k);
		for (j = k; j < k + VAR_BLOCK && j < ec->taps; j++)
			var_gain[j] = sum;
	}
	for (k = 0; VAR_CAP > 0.0f && k < ec->taps; k++)
		if (var_gain[k] > VAR_CAP)
			var_gain[k] = VAR_CAP;
}

static float frame_filter_run(struct udsp_ec *ec, int16_t *mic,
			      const int16_t *ref, bool adapt, float mu,
			      float step_floor, float *estimate_power,
			      float *cross_power)
{
	const float *win = hist_push(ec, ref[0]);
	float res_power = 0.0f;
	unsigned i;
	unsigned k;

	if (adapt)
		var_gain_set(ec);
#if VAR_GATE
	if (adapt && !var_gate_get(ec)) {
		for (k = 0; k < ec->taps; k++)
			var_gain[k] = 1.0f;
	}
#endif
	for (i = 0; i < ec->frame; i++) {
		float estimate = taps_dot_scalar(ec->coef, win, ec->taps);
		float error;

		*estimate_power += estimate * estimate;
		error = (float)mic[i] - estimate;
		res_power += error * error;
		*cross_power += error * estimate;
		mic[i] = udsp_sat16_round(error);
		if (adapt) {
			float den = 0.0f;
			float step;

			if (VAR_APPROX)
				den = (float)ec->hist_energy;
			for (k = 0; !VAR_APPROX && k < ec->taps; k++)
				den += var_gain[k] * win[k] * win[k];
			step = mu * error / (den + step_floor);
			for (k = 0; k < ec->taps; k++)
				ec->coef[k] += step * var_gain[k] * win[k];
		}
		if (i + 1u == ec->frame)
			break;
		win = hist_push(ec, ref[i + 1u]);
	}
	return res_power;
}
'''
FIELDS = '\tstruct lec_belief belief;\n\tstruct lec_belief probe_state;\n'
FIELDS_VAR = FIELDS + '\tbool var_trusted;\n\tunsigned var_kept;\n'
FRAME_END = '\tec->frames++;\n\treading_fill(ec, &t);\n}\n'
FRAME_END_VAR = ('\tec->frames++;\n'
                 '\tec->var_trusted = t.trusted;\n'
                 '\tif (ec->reading.trial_end == UDSP_EC_NLMS_TRIAL_END_KEPT)\n'
                 '\t\tec->var_kept = ec->frames;\n'
                 '\treading_fill(ec, &t);\n}\n')
STEP_GET = '''static float var_step_get(const struct udsp_ec *ec, bool probing)
{
	if (!ec->var_trusted || (VAR_TRIAL && probing) ||
	    (ec->var_kept && ec->frames < ec->var_kept + VAR_FAST_FRAMES))
		return VAR_FAST_STEP;
	return LEC_STEP;
}

'''
MU_FAST = ('\t\t\t\t\t       var_step_get(ec, t.probing) * '
           'step_scale_get(ec) *\n'
           '\t\t\t\t\t       talk_scale_get(ec, t.mic_power,\n'
           '\t\t\t\t\t\t\t      t.window_power),\n')
COUNTDOWN = ('\t    --ec->probe_frames == 0u)\n\t\tprobe_finish(ec);\n')
COUNTDOWN_EARLY = ('\t    (--ec->probe_frames == 0u || var_keep_early(ec)))\n'
                   '\t\tprobe_finish(ec);\n')
KEEP_EARLY = '''static bool var_keep_early(struct udsp_ec *ec)
{
	if (LEC_PROBE_FRAMES - ec->probe_frames < VAR_KEEP_FRAMES ||
	    !(probe_ratio_get(ec) <= ec->probe_ratio_before * VAR_KEEP_RATIO))
		return false;
	ec->probe_frames = 0;
	return true;
}

'''
EC_RUN = 'static void ec_run(struct udsp_ec *ec, int16_t *mic, const int16_t *ref)\n'


def filter_replace(text, alpha):
    """text with frame_filter_run replaced by the proportionate form."""
    a = text.index(FILTER_HEAD)
    if text.count(FILTER_HEAD) != 1:
        sys.exit('mkvar.py: ipnlms: frame_filter_run occurs more than once')
    b = text.index('\n}\n', a) + 3
    parts = alpha.split(':')
    block = next((int(x[1:]) for x in parts[1:] if x.startswith('b')), 1)
    cap = next((float(x[1:]) for x in parts[1:] if x.startswith('c')), 0.0)
    gate = 2 if 'gatea' in parts else 1 if 'gate' in parts else 0
    text = (text[:a] + '#define VAR_ALPHA %sf\n#define VAR_APPROX %d\n'
            '#define VAR_BLOCK %du\n#define VAR_CAP %sf\n#define VAR_GATE %d\n'
            % (float(parts[0]), 'approx' in parts, block, cap, gate) +
            IPNLMS + text[b:])
    if gate:
        text = replace_once(text, FIELDS, FIELDS + '\tbool var_fast;\n',
                            'ipnlms gate')
        text = replace_once(text, FRAME_END, '\tec->frames++;\n'
                            '\tif (ec->reading.trial_end == '
                            'UDSP_EC_NLMS_TRIAL_END_KEPT &&\n'
                            '\t    (ec->trial_own & (LEC_ARM_PATH | '
                            'LEC_ARM_COHERE)))\n'
                            '\t\tec->var_fast = true;\n'
                            '\tif (t.trusted)\n'
                            '\t\tec->var_fast = false;\n'
                            '\treading_fill(ec, &t);\n}\n', 'ipnlms gate')
    return text


def candidate_rule(text, name, arg):
    """text with a candidate rule: ipnlms, stepfast or keep-early."""
    if name == 'ipnlms':
        return filter_replace(text, arg)
    if name == 'stepfast':
        parts = arg.split(':')
        trial = len(parts) > 2 and parts[2] == 'trial'
        text = replace_once(text, FIELDS, FIELDS_VAR, name)
        text = replace_once(text, FRAME_END, FRAME_END_VAR, name)
        text = replace_once(text, EC_RUN, '#define VAR_FAST_STEP %sf\n'
                            '#define VAR_FAST_FRAMES %du\n'
                            '#define VAR_TRIAL %d\n' % (
                                float(parts[0]), int(parts[1]), trial) +
                            STEP_GET + EC_RUN, name)
        return replace_once(text, MU, MU_FAST, name)
    frames, ratio = arg.split(':')
    text = replace_once(text, EC_RUN, '#define VAR_KEEP_FRAMES %du\n'
                        '#define VAR_KEEP_RATIO %sf\n' % (int(frames),
                                                         float(ratio)) +
                        KEEP_EARLY + EC_RUN, name)
    return replace_once(text, COUNTDOWN, COUNTDOWN_EARLY, name)


def switch_rule(text, name):
    """text with one of the rules that act from UDSP_VAR_FROM_FRAME."""
    if 'var_on(' not in text:
        text = replace_once(text, FIRST_FN, VAR_ON + FIRST_FN, name)
    if name in ('scales-off', 'raw'):
        text = replace_once(text, MU, MU_OFF, name)
    if name == 'scales-off':
        return text
    adapt = ('\tadapt = var_on(ec) ? %s :\n\t\t(t.active && !ec->freeze_on '
             '&& !t.held &&\n\t\t (t.probing || !double_talk_get(ec)));\n' %
             ('t.active && !ec->freeze_on' if name == 'hold-off' else
              '!ec->freeze_on'))
    text = replace_once(text, ADAPT, adapt, name)
    text = replace_once(text, ROLLBACK, ROLLBACK_OFF, name)
    text = replace_once(text, ARMS, ARMS_OFF, name)
    if name == 'raw':
        text = replace_once(text, FLOOR, FLOOR_RAW, name)
    return text


def variant_write(name, src, defines=(), rules=()):
    """Write var/<name>/lec.c and return its path."""
    with open(src) as f:
        text = f.read()
    for d in defines:
        key, _, value = d.partition('=')
        text = define_set(text, key, value)
    for r in rules:
        text = rule_apply(text, r)
    out = work.work_dir() / 'var' / name / 'lec.c'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    return out


def main(argv):
    p = argparse.ArgumentParser(prog='mkvar.py')
    p.add_argument('name')
    p.add_argument('--from', dest='src', default=str(work.UDSP_SRC /
                                                    'dsp/lec.c'))
    p.add_argument('--define', action='append', default=[])
    p.add_argument('--rule', action='append', default=[])
    a = p.parse_args(argv)
    if a.name == work.TREE:
        sys.exit(f'mkvar.py: the name {work.TREE} selects the build '
                 f'directory')
    print('wrote', variant_write(a.name, a.src, a.define, a.rule))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
