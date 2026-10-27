#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""G.168 Tests 2A, 2B and 12 stimuli for lec_hz: convergence on m1, then a
change of the echo path to another Annex D model.

usage: stimconv.py <outdir> [--fs R [R ...]] [--models M [M ...]]
                   [--level DBM0 [DBM0 ...]] [--seeds LIST]
                   [--phase rng|seed|tail] [--peak N] [--erl-i DB]
                   [--erl-ii DB] [--three] [--periods N] [--lead S]
                   [--ref-loss DB] [--line DBM0] [--line-rise DB]
                   [--nlp 0|1] [--name TEMPLATE]

Each case of the grid fs x model x level x seed goes to <outdir>/<name>.
LIST is a comma list of integers and ranges A-B. --phase sets the frame
phase of the far onset as in stim.py: a draw of the case's generator (rng,
the default), the seed itself (seed), or the frame less the seed (tail).

TEMPLATE is a str.format pattern over fs, model, level, seed and phase. The
default is c-{fs}-{model}-l{level:g}-s{seed} for rng,
cp-{fs}-{model}-l{level:g}-s{seed} for seed and
ck-{fs}-{model}-l{level:g}-s{seed} for tail. Two grid points that give one
name stop the program with status 1 before it writes a case.

The schedule is the one that the G.168 bench plays for Tests 2A, 2B
and 12 (G.168 clauses 6.4.2.3 and 6.4.2.13), on the wire plane at 8 kHz:

- lead: line noise alone for lead_s;
- far end: the single talk CSS of siggen.css_single (seed 0) at level, one
  continuous signal over every phase of periods CSS periods, then one
  period of silence;
- phase I: the echo path m1 at erl_i;
- phase II: the model at erl_ii;
- phase III (--three, Test 12): m1 at erl_i again.

Each phase starts at a CSS period start, and the echo path changes there
with its history: the samples of a phase are the far end through the
model of that phase. A pure delay puts the peak of m1 on tap peak of the
canceller, and every model of the case takes the same delay. erl_ii
defaults to the bench's ERL of the model (ERL_II_DB: 6 dB for m4 and m6,
12 dB for m7 and m8, over the Table D.1a minimum) and erl_i to erl_ii plus
DELTA_ERL_DB.

The canceller plane: the reference is the far end less ref_loss_db, the
loss that the engine puts ahead of the reference on the bench line; the
microphone is the echo plus white line noise at line_dbm0, at the wire
level. At 16 kHz the far end is the 8 kHz signal raised to 16 kHz
(siggen.upsample2), each model is its 16 kHz form, and the line noise is
white at 16 kHz. --line-rise raises the line noise by DB over phase II, as
the background of the bench line rises with the louder echo path of that
phase.

Files of a case: ref.s16 and mic.s16 (int16 little-endian), echo.f32 and
noise.f32 (the line noise, float32) and meta.json. meta.json holds the
parameters, t0_sample and t0_frame (the far onset), frames, td_samples
(the pure delay at the line rate), l_rin_act_dbm0 (the active level of the
far end, level plus L_ACT_DB), phases (name, start_sample, end_sample,
model, erl_db of each phase), kind conv and nlp.
"""
import argparse
import json
import os
import sys

import numpy as np

import siggen as sg
from work import int_list, names_check

FS_WIRE = 8000
MODELS = ('m4', 'm6', 'm7', 'm8')
ERL_II_DB = {'m1': 6.0, 'm4': 6.0, 'm6': 6.0, 'm7': 12.0, 'm8': 12.0}
DELTA_ERL_DB = 10.0
# G.168 Annex C.3.1: the active level of a single talk CSS over its RMS
# level of whole periods.
L_ACT_DB = 1.49
# The peak tap of m1 on the bench line at each rate.
PEAK_DEFAULT = {8000: 262, 16000: 524}
TEMPLATES = {
    'rng': 'c-{fs}-{model}-l{level:g}-s{seed}',
    'seed': 'cp-{fs}-{model}-l{level:g}-s{seed}',
    'tail': 'ck-{fs}-{model}-l{level:g}-s{seed}',
}


def phases_get(model, erl_i, erl_ii, three):
    """(name, model, erl_db) of each phase."""
    out = [('I', 'm1', erl_i), ('II', model, erl_ii)]
    if three:
        out.append(('III', 'm1', erl_i))
    return out


def echo_get(far, plan, fs, period, lead):
    """The echo of far through each phase's model, before the pure delay:
    sample n takes the model of the phase that holds n, over the whole
    history of far."""
    echo = np.zeros(len(far))
    for k, (_, model, erl) in enumerate(plan):
        a = lead + k * period
        b = lead + (k + 1) * period if k + 1 < len(plan) else len(far)
        echo[a:b] = sg.fir(sg.hybrid_model(model, erl, fs), far)[a:b]
    return echo


def build(out, fs, model, level, seed, peak=None, erl_i=None, erl_ii=None,
          three=False, periods=16, lead_s=1.75, ref_loss_db=9.0,
          line_dbm0=-73.0, phase=None, nlp=0, css_seed=0,
          line_rise_db=0.0):
    """Write one case into out and return its meta dict. phase None draws
    the frame phase from the case's generator."""
    rng = np.random.default_rng([seed, 11])
    if phase is None:
        phase = int(rng.integers(0, fs // 100))
    erl_ii = ERL_II_DB[model] if erl_ii is None else erl_ii
    erl_i = erl_ii + DELTA_ERL_DB if erl_i is None else erl_i
    peak = PEAK_DEFAULT[fs] if peak is None else peak
    plan = phases_get(model, erl_i, erl_ii, three)
    up = fs // FS_WIRE
    period8 = sg.css_period_get(FS_WIRE)
    lead8 = round(lead_s * FS_WIRE)
    n_css = len(plan) * periods * period8
    far8 = np.zeros(lead8 + n_css + period8)
    far8[lead8:lead8 + n_css] = sg.css_single(level, n_css / FS_WIRE,
                                              FS_WIRE, css_seed)
    far = far8 if fs == FS_WIRE else sg.upsample2(far8)
    delay = peak - int(np.argmax(np.abs(sg.hybrid_model('m1', erl_i, fs))))
    if delay < 0:
        raise ValueError('peak before the model peak')
    echo = echo_get(far, plan, fs, periods * period8 * up, lead8 * up)
    echo = np.concatenate((np.zeros(delay), echo))[:len(far)]
    line_rms = sg.dbm0_to_rms(line_dbm0)
    line = rng.standard_normal(phase + len(far)) * line_rms
    far = np.concatenate((np.zeros(phase), far))
    echo = np.concatenate((np.zeros(phase), echo))
    frame = fs // 100
    total = (len(far) // frame) * frame
    far, echo, line = far[:total], echo[:total], line[:total]
    t0 = phase + lead8 * up
    span = periods * period8 * up
    line[t0 + span:t0 + 2 * span] *= 10 ** (line_rise_db / 20)
    os.makedirs(out, exist_ok=True)
    sg.to_int16(far * 10 ** (-ref_loss_db / 20)).tofile(f'{out}/ref.s16')
    sg.to_int16(echo + line).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    line.astype('<f4').tofile(f'{out}/noise.f32')
    meta = dict(fs=fs, seed=seed, model=model, level=level, erl_i=erl_i,
                erl_ii=erl_ii, three=three, peak=peak, delay=delay,
                td_samples=delay, phase=phase, ref_loss_db=ref_loss_db,
                lead_s=lead_s, line_dbm0=line_dbm0,
                line_rise_db=line_rise_db, periods=periods,
                t0_sample=t0, t0_frame=t0 / frame, frames=total // frame,
                l_rin_act_dbm0=level + L_ACT_DB,
                phases=[dict(name=name, start_sample=t0 + k * span,
                             end_sample=t0 + (k + 1) * span, model=m,
                             erl_db=erl)
                        for k, (name, m, erl) in enumerate(plan)],
                kind='conv', nlp=nlp)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def phase_get(mode, fs, seed):
    """The frame phase of a case under --phase mode."""
    if mode == 'seed':
        return seed % (fs // 100)
    if mode == 'tail':
        return (fs // 100 - seed) % (fs // 100)
    return None


def main(argv):
    p = argparse.ArgumentParser(prog='stimconv.py', description=__doc__.split(
        '\n')[0])
    p.add_argument('outdir')
    p.add_argument('--fs', type=int, nargs='+', default=[8000, 16000])
    p.add_argument('--models', nargs='+', default=list(MODELS))
    p.add_argument('--level', type=float, nargs='+', default=[-10.0])
    p.add_argument('--seeds', type=int_list, default=[0])
    p.add_argument('--phase', choices=sorted(TEMPLATES), default='rng')
    p.add_argument('--peak', type=int)
    p.add_argument('--erl-i', type=float)
    p.add_argument('--erl-ii', type=float)
    p.add_argument('--three', action='store_true')
    p.add_argument('--periods', type=int, default=16)
    p.add_argument('--lead', type=float, default=1.75)
    p.add_argument('--ref-loss', type=float, default=9.0)
    p.add_argument('--line', type=float, default=-73.0)
    p.add_argument('--line-rise', type=float, default=0.0)
    p.add_argument('--nlp', type=int, choices=(0, 1), default=0)
    p.add_argument('--name')
    a = p.parse_args(argv)
    for m in a.models:
        if m not in ERL_II_DB:
            sys.exit(f'stimconv.py: unknown model {m}')
    template = a.name or TEMPLATES[a.phase]
    grid = []
    for fs in a.fs:
        for model in a.models:
            for level in a.level:
                for seed in a.seeds:
                    phase = phase_get(a.phase, fs, seed)
                    name = template.format(fs=fs, model=model, level=level,
                                           seed=seed, phase=phase)
                    grid.append((name, fs, model, level, seed, phase))
    names_check([(g[0], 'fs %d model %s level %g seed %d' % g[1:5])
                 for g in grid])
    for name, fs, model, level, seed, phase in grid:
        m = build(os.path.join(a.outdir, name), fs, model, level, seed,
                  a.peak, a.erl_i, a.erl_ii, a.three, a.periods, a.lead,
                  a.ref_loss, a.line, phase, a.nlp,
                  line_rise_db=a.line_rise)
        print(name, 'phase', m['phase'], 't0_frame', '%.2f' % m['t0_frame'],
              'erl %g/%g' % (m['erl_i'], m['erl_ii']), 'delay', m['delay'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
