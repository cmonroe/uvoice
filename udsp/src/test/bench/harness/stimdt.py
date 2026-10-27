#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""G.168 Tests 3A and 3B stimuli for lec_hz: far CSS through m1 with double
talk CSS at the near end.

usage: stimdt.py <outdir> [--fs R [R ...]] [--level DBM0 [DBM0 ...]]
                 [--x-db X [X ...]] [--seeds LIST] [--variant a|b]
                 [--peak N] [--erl DB] [--periods N] [--lead S]
                 [--ref-loss DB] [--line DBM0] [--bg DBM0] [--bg-lead S]
                 [--near-lag N] [--linear] [--phase rng|seed|tail]
                 [--name TEMPLATE]

Each case of the grid fs x level x X x seed goes to <outdir>/<name>. LIST
is a comma list of integers and ranges A-B. TEMPLATE is a str.format
pattern over variant, fs, level, x and seed. The default is
d{variant}-{fs}-l{level:g}-x{x:g}-s{seed}. Two grid points that give one
name stop the program with status 1 before it writes a case. --phase sets
the frame phase of the far onset as in stim.py: a draw of the case's
generator (rng, the default), the seed itself (seed), or the frame less the
seed (tail). Reason: the bench captures read their far onset 73 and 93
samples into a frame at 16 kHz, and the first frame of each burst then
holds a part of the burst only.

The schedule is the one that the G.168 bench plays for Tests 3A and 3B
(G.168 clauses 6.4.2.4.1 and 6.4.2.4.2), on the wire plane at 8 kHz:

- lead: line noise alone for lead_s;
- converge (variant b only): the far CSS alone for periods CSS periods;
- dt: the far CSS with the near double talk CSS, for periods periods;
- near_probe: the near CSS alone for PROBE_PERIODS periods.

The far end is the single talk CSS of siggen.css_single (seed 0) at level,
through m1 at erl with the peak of m1 on tap peak (stimconv.py). The far
end reaches the line as the bench sends it: G.711 mu-law at 8 kHz, and at
16 kHz raised by the interpolator of the canceller's line
(siggen.dut_up2). --linear takes the CSS itself, raised by the zero-phase
siggen.upsample2. The near end is the double talk CSS of siggen.css_double
(seed seed + NEAR_SEED, so seed 0 plays the near CSS of the bench) at an
active level L_Sgen,act of L_Rin,act less X: 15 dB for Test 3A. It starts
with the dt phase, near-lag samples after the sample where the echo of the
far end starts, and runs to the end of the near_probe phase.

The canceller plane: the reference is the far end less ref_loss_db, and
the microphone is the echo, the near end and the line noise, at the wire
level. The line noise is white at line_dbm0 over the whole case, and a
background at bg_dbm0 joins it bg_lead_s before the far onset. The
background is white noise at 8 kHz. Reason: the bench captures of these
tests read -73.9 dBm0 on the microphone before the bench arms its echo
path, and -64.6 dBm0 of a background under 4 kHz from 0.5 s before the far
onset to the end. The floor tracker of the canceller reads that background
before the far end starts. At 16 kHz the near CSS and the background are
8 kHz signals raised to 16 kHz (siggen.upsample2), and the white line noise
is white at 16 kHz.

Files of a case: ref.s16 and mic.s16 (int16 little-endian), echo.f32,
noise.f32 (the line noise and the background) and near.f32 (the near
end), all float32, and
meta.json. meta.json holds the parameters, t0_sample and t0_frame (the far
onset), frames, td_samples (the pure delay), l_rin_act_dbm0,
l_sgen_act_dbm0, variant, phases (name, start_sample, end_sample of each
phase), kind dt and nlp 0.
"""
import argparse
import json
import os
import sys

import numpy as np

import siggen as sg
import stimconv
from work import int_list, names_check

FS_WIRE = 8000
PROBE_PERIODS = 4
NEAR_SEED = 5
X_3A_DB = 15.0
# G.168 clause 6.4.1.2: the active level of a double talk CSS over its RMS
# level of whole periods.
L_ACT_DT_DB = 1.66
TEMPLATE = 'd{variant}-{fs}-l{level:g}-x{x:g}-s{seed}'
BG_DBM0 = -64.6
BG_LEAD_S = 0.5


def phases_get(variant, t0, span, probe):
    """(name, start, end) of each phase from the far onset t0."""
    out = []
    if variant == 'b':
        out.append(('converge', t0, t0 + span))
        t0 += span
    out.append(('dt', t0, t0 + span))
    out.append(('near_probe', t0 + span, t0 + span + probe))
    return out


def far_line(far8, fs, linear):
    """The far end on the line at fs: through G.711 mu-law and the
    interpolator of the line, or with linear the CSS itself."""
    if linear:
        return far8 if fs == FS_WIRE else sg.upsample2(far8)
    coded = sg.ulaw_decode(sg.ulaw_encode(far8))
    return coded if fs == FS_WIRE else sg.dut_up2(coded)


def build(out, fs, level, x_db, seed, variant='a', peak=None, erl=6.0,
          periods=16, lead_s=1.75, ref_loss_db=9.0, line_dbm0=-73.0,
          bg_dbm0=BG_DBM0, bg_lead_s=BG_LEAD_S, near_lag=0, linear=False,
          phase=None):
    """Write one case into out and return its meta dict. phase None draws
    the frame phase from the case's generator."""
    rng = np.random.default_rng([seed, 13])
    if phase is None:
        phase = int(rng.integers(0, fs // 100))
    up = fs // FS_WIRE
    peak = stimconv.PEAK_DEFAULT[fs] if peak is None else peak
    period8 = sg.css_period_get(FS_WIRE)
    lead8 = round(lead_s * FS_WIRE)
    far_periods = periods * (2 if variant == 'b' else 1)
    n_far = far_periods * period8
    n8 = lead8 + n_far + (PROBE_PERIODS + 1) * period8
    far8 = np.zeros(n8)
    far8[lead8:lead8 + n_far] = sg.css_single(level, n_far / FS_WIRE,
                                              FS_WIRE, 0)
    l_rin_act = level + stimconv.L_ACT_DB
    l_sgen_act = l_rin_act - x_db
    n_near = (periods + PROBE_PERIODS) * period8
    near8 = sg.css_double(l_sgen_act - L_ACT_DT_DB, n_near / FS_WIRE,
                          FS_WIRE, seed + NEAR_SEED)
    far = far_line(far8, fs, linear)
    m1 = sg.hybrid_model('m1', erl, fs)
    delay = peak - int(np.argmax(np.abs(m1)))
    if delay < 0:
        raise ValueError('peak before the model peak')
    echo = np.concatenate((np.zeros(delay), sg.fir(m1, far)))[:len(far)]
    t0 = lead8 * up
    span = periods * period8 * up
    plan = phases_get(variant, t0, span, PROBE_PERIODS * period8 * up)
    near_at = plan[-2][1] + delay + near_lag
    near_up = near8 if fs == FS_WIRE else sg.upsample2(near8)
    near = np.zeros(len(far))
    near[near_at:near_at + len(near_up)] = near_up[:len(far) - near_at]
    line = rng.standard_normal(len(far)) * sg.dbm0_to_rms(line_dbm0)
    bg8 = np.zeros(n8)
    b8 = max(0, lead8 - round(bg_lead_s * FS_WIRE))
    bg8[b8:] = rng.standard_normal(n8 - b8) * sg.dbm0_to_rms(bg_dbm0)
    line += bg8 if fs == FS_WIRE else sg.upsample2(bg8)
    lead = np.zeros(phase)
    far, echo, near = (np.concatenate((lead, x))[:len(far)]
                       for x in (far, echo, near))
    line = np.concatenate((rng.standard_normal(phase) *
                           sg.dbm0_to_rms(line_dbm0), line))[:len(far)]
    t0 += phase
    plan = [(n, a + phase, b + phase) for n, a, b in plan]
    frame = fs // 100
    total = (len(far) // frame) * frame
    far, echo, near, line = far[:total], echo[:total], near[:total], \
        line[:total]
    os.makedirs(out, exist_ok=True)
    sg.to_int16(far * 10 ** (-ref_loss_db / 20)).tofile(f'{out}/ref.s16')
    sg.to_int16(echo + near + line).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    line.astype('<f4').tofile(f'{out}/noise.f32')
    near.astype('<f4').tofile(f'{out}/near.f32')
    meta = dict(fs=fs, seed=seed, level=level, x_db=x_db, variant=variant,
                erl=erl, peak=peak, delay=delay, td_samples=delay,
                ref_loss_db=ref_loss_db, lead_s=lead_s, line_dbm0=line_dbm0,
                bg_dbm0=bg_dbm0, bg_lead_s=bg_lead_s, near_lag=near_lag,
                linear=linear, phase=phase,
                periods=periods, t0_sample=t0, t0_frame=t0 / frame,
                frames=total // frame, l_rin_act_dbm0=l_rin_act,
                l_sgen_act_dbm0=l_sgen_act,
                phases=[dict(name=n, start_sample=a, end_sample=b)
                        for n, a, b in plan],
                kind='dt', nlp=0)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stimdt.py', description=__doc__.split(
        '\n')[0])
    p.add_argument('outdir')
    p.add_argument('--fs', type=int, nargs='+', default=[8000, 16000])
    p.add_argument('--level', type=float, nargs='+', default=[-10.0])
    p.add_argument('--x-db', type=float, nargs='+', default=[X_3A_DB])
    p.add_argument('--seeds', type=int_list, default=[0])
    p.add_argument('--variant', choices=('a', 'b'), default='a')
    p.add_argument('--peak', type=int)
    p.add_argument('--erl', type=float, default=6.0)
    p.add_argument('--periods', type=int, default=16)
    p.add_argument('--lead', type=float, default=1.75)
    p.add_argument('--ref-loss', type=float, default=9.0)
    p.add_argument('--line', type=float, default=-73.0)
    p.add_argument('--bg', type=float, default=BG_DBM0)
    p.add_argument('--bg-lead', type=float, default=BG_LEAD_S)
    p.add_argument('--near-lag', type=int, default=0)
    p.add_argument('--linear', action='store_true')
    p.add_argument('--phase', choices=sorted(stimconv.TEMPLATES),
                   default='rng')
    p.add_argument('--name', default=TEMPLATE)
    a = p.parse_args(argv)
    grid = []
    for fs in a.fs:
        for level in a.level:
            for x in a.x_db:
                for seed in a.seeds:
                    name = a.name.format(variant=a.variant, fs=fs,
                                         level=level, x=x, seed=seed)
                    grid.append((name, fs, level, x, seed))
    names_check([(g[0], 'fs %d level %g X %g seed %d' % g[1:5])
                 for g in grid])
    for name, fs, level, x, seed in grid:
        m = build(os.path.join(a.outdir, name), fs, level, x, seed,
                  a.variant, a.peak, a.erl, a.periods, a.lead, a.ref_loss,
                  a.line, a.bg, a.bg_lead, a.near_lag, a.linear,
                  stimconv.phase_get(a.phase, fs, seed))
        print(name, 't0_frame', '%.2f' % m['t0_frame'], 'L_Sgen,act',
              '%.2f' % m['l_sgen_act_dbm0'], 'delay', m['delay'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
