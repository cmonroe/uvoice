#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""G.168 Test 9A stimuli for lec_hz: comfort noise against near-end noise
steps.

usage: stimcn.py <outdir> [--fs R [R ...]] [--near band|hoth|white ...]
                 [--levels DBM0 [DBM0 ...]] [--seeds LIST] [--erl DB]
                 [--far DBM0] [--peak N] [--ref-loss DB] [--line DBM0]
                 [--name TEMPLATE]

Each case of the grid fs x near x seed goes to <outdir>/<name>. TEMPLATE is
a str.format pattern over fs, near and seed; the default is
n-{fs}-{near}-s{seed}. Two grid points that give one name stop the program
with status 1 before it writes a case.

The schedule is the one that the G.168 bench plays for Test 9A (G.168
clause 6.4.2.10.1), on the wire plane at 8 kHz:

- lead: line noise alone for LEAD_S;
- warmup: far band noise at far_dbm0 for WARMUP_S, through m1 at erl;
- per level of levels, one step of two phases. quiet_<k>: the far end
  silent for QUIET_S, with the near noise at the level. cn_<k>: the far
  band noise again for ON_S, with the near noise on.

The near noise of a step is one signal over both of its phases: band noise
(siggen.band_noise, the bench's), Hoth noise (siggen.hoth_metered) or white
Gaussian noise, each at its level on the wire. The default levels are the
bench's: -45, -50, -45, -55 and -45 dBm0. A pure delay puts the peak of m1
on tap peak of the canceller.

The canceller plane: the reference is the far end less ref_loss_db, the
loss that the engine puts ahead of the reference on the bench line; the
microphone is the echo, the near noise and white line noise at line_dbm0.
At 16 kHz the far end and the near noise are 8 kHz signals raised to
16 kHz (siggen.upsample2), m1 is its 16 kHz form, and the line noise is
white at 16 kHz.

Files of a case: ref.s16 and mic.s16 (int16 little-endian), echo.f32 and
noise.f32 (the near noise plus the line noise, float32) and meta.json.
meta.json holds the parameters, t0_sample and t0_frame (the warmup's far
onset), frames, td_samples (the pure delay at the line rate), phases (name,
start_sample, end_sample and l_sgen_dbm0, None outside a cn phase), kind
cn and nlp 1.
"""
import argparse
import json
import os
import sys

import numpy as np

import siggen as sg
from work import int_list, names_check

FS_WIRE = 8000
LEAD_S = 0.5
WARMUP_S = 5.6
QUIET_S = 10.5
ON_S = 10.5
BENCH_LEVELS = (-45.0, -50.0, -45.0, -55.0, -45.0)
NEAR_KINDS = ('band', 'hoth', 'white')
# The peak tap of m1 on the bench line at each rate.
PEAK_DEFAULT = {8000: 262, 16000: 524}


def near_get(kind, level, dur_s, seed):
    """The near noise of one step at 8 kHz, at level on the wire."""
    if kind == 'band':
        return sg.band_noise(level, dur_s, FS_WIRE, seed)
    if kind == 'hoth':
        return sg.hoth_metered(level, dur_s, seed)
    x = np.random.default_rng(seed).standard_normal(int(round(dur_s *
                                                              FS_WIRE)))
    return x * sg.dbm0_to_rms(level)


def plan_get(levels):
    """(name, seconds, far on, near level or None, l_sgen or None) of each
    phase after the lead."""
    plan = [('warmup', WARMUP_S, True, None, None)]
    for k, level in enumerate(levels):
        plan.append((f'quiet_{k}', QUIET_S, False, level, None))
        plan.append((f'cn_{k}', ON_S, True, level, level))
    return plan


def wire_build(near_kind, levels, seed, far_dbm0):
    """(far, near, plan, starts) at 8 kHz: the far end and the near noise
    over the whole schedule, and the start sample of each phase."""
    plan = plan_get(levels)
    lead = int(round(LEAD_S * FS_WIRE))
    starts = [lead]
    for _, sec, _, _, _ in plan:
        starts.append(starts[-1] + int(round(sec * FS_WIRE)))
    far = np.zeros(starts[-1])
    near = np.zeros(starts[-1])
    for k, (name, sec, on, level, _) in enumerate(plan):
        a, b = starts[k], starts[k + 1]
        if on:
            far[a:b] = sg.band_noise(far_dbm0, sec, FS_WIRE, [seed, 90, k])
        if name.startswith('quiet_'):
            n = starts[k + 2] - a
            near[a:a + n] = near_get(near_kind, level, n / FS_WIRE,
                                     [seed, 91, k])
    return far, near, plan, starts


def build(out, fs, near_kind, seed, levels=BENCH_LEVELS, erl=12.0,
          far_dbm0=-10.0, peak=None, ref_loss_db=9.0, line_dbm0=-73.0):
    """Write one case into out and return its meta dict."""
    rng = np.random.default_rng([seed, 12])
    phase = int(rng.integers(0, fs // 100))
    peak = PEAK_DEFAULT[fs] if peak is None else peak
    far8, near8, plan, starts = wire_build(near_kind, levels, seed, far_dbm0)
    up = fs // FS_WIRE
    far = far8 if up == 1 else sg.upsample2(far8)
    near = near8 if up == 1 else sg.upsample2(near8)
    model = sg.hybrid_model('m1', erl, fs)
    delay = peak - int(np.argmax(np.abs(model)))
    if delay < 0:
        raise ValueError('peak before the model peak')
    echo = np.concatenate((np.zeros(delay), sg.fir(model, far)))[:len(far)]
    line = rng.standard_normal(phase + len(far)) * sg.dbm0_to_rms(line_dbm0)
    pad = np.zeros(phase)
    far = np.concatenate((pad, far))
    echo = np.concatenate((pad, echo))
    noise = np.concatenate((pad, near)) + line
    frame = fs // 100
    total = (len(far) // frame) * frame
    far, echo, noise = far[:total], echo[:total], noise[:total]
    os.makedirs(out, exist_ok=True)
    sg.to_int16(far * 10 ** (-ref_loss_db / 20)).tofile(f'{out}/ref.s16')
    sg.to_int16(echo + noise).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    noise.astype('<f4').tofile(f'{out}/noise.f32')
    t0 = phase + starts[0] * up
    meta = dict(fs=fs, seed=seed, near=near_kind, levels=list(levels),
                erl_db=erl, far_dbm0=far_dbm0, peak=peak, delay=delay,
                td_samples=delay, phase=phase, ref_loss_db=ref_loss_db,
                line_dbm0=line_dbm0, t0_sample=t0, t0_frame=t0 / frame,
                frames=total // frame,
                phases=[dict(name=name, start_sample=phase + starts[k] * up,
                             end_sample=min(total,
                                            phase + starts[k + 1] * up),
                             l_sgen_dbm0=l_sgen)
                        for k, (name, _, _, _, l_sgen) in enumerate(plan)],
                kind='cn', nlp=1)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stimcn.py', description=__doc__.split(
        '\n')[0])
    p.add_argument('outdir')
    p.add_argument('--fs', type=int, nargs='+', default=[8000, 16000])
    p.add_argument('--near', nargs='+', choices=NEAR_KINDS,
                   default=['band'])
    p.add_argument('--levels', type=float, nargs='+',
                   default=list(BENCH_LEVELS))
    p.add_argument('--seeds', type=int_list, default=[0])
    p.add_argument('--erl', type=float, default=12.0)
    p.add_argument('--far', type=float, default=-10.0)
    p.add_argument('--peak', type=int)
    p.add_argument('--ref-loss', type=float, default=9.0)
    p.add_argument('--line', type=float, default=-73.0)
    p.add_argument('--name', default='n-{fs}-{near}-s{seed}')
    a = p.parse_args(argv)
    grid = [(a.name.format(fs=fs, near=near, seed=seed), fs, near, seed)
            for fs in a.fs for near in a.near for seed in a.seeds]
    names_check([(g[0], 'fs %d near %s seed %d' % g[1:]) for g in grid])
    for name, fs, near, seed in grid:
        m = build(os.path.join(a.outdir, name), fs, near, seed, a.levels,
                  a.erl, a.far, a.peak, a.ref_loss, a.line)
        print(name, 'phase', m['phase'], 'frames', m['frames'], 'delay',
              m['delay'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
