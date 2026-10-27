#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""G.168 Test 2C(c) stimuli for lec_hz: echo of a far CSS under near Hoth
noise.

usage: stim.py <outdir> --peak N [N ...] [--fs R [R ...]]
               [--seeds LIST] [--phase rng|seed|tail] [--echo-db D [D ...]]
               [--hoth-shift N] [--name TEMPLATE] [--ref-loss DB]
               [--l-sgen DBM0] [--line DBM0] [--lead S] [--no-hoth]

Each case of the grid fs x peak x echo-db x seed goes to <outdir>/<name>.
LIST is a comma list of integers and ranges A-B. The frame phase of the
first burst comes from --phase:

- rng: a draw of the case's generator (the default);
- seed: the seed itself, so seed k starts k samples into a frame;
- tail: the frame less the seed, so the burst starts in the last k samples
  of a frame.

TEMPLATE is a str.format pattern over fs, peak, echo_db, seed and phase.
The default is w-{fs}-{peak}-e{echo_db:.1f}-s{seed} for rng,
p-{fs}-{peak}-e{echo_db:.1f}-s{seed} for seed and
k-{fs}-{peak}-e{echo_db:.1f}-s{seed} for tail. Two grid points that give
one name stop the program with status 1 before it writes a case.

The wire plane runs at 8 kHz:

- far end: the CSS of siggen.css_single (seed 0) at -10 dBm0 for 16
  periods after a lead of lead_s, then two periods of silence;
- near end: Hoth noise of 300 to 3400 Hz, metered to l_sgen by the G.168
  level meter, from 0.2 s before the first burst to the end, with seed
  34 + 101 seed; white line noise at line_dbm0 under it from the start;
- echo: the far end through m1 of Annex D at ERL 6 dB, after a pure delay
  that puts the peak of m1 on tap peak of the canceller.

The canceller plane: the reference is the far end less ref_loss_db, the
loss that the engine puts ahead of the reference; the microphone is echo
plus noise at the wire level. At 16 kHz the far end, the echo and the Hoth
noise are 8 kHz signals raised to 16 kHz (siggen.upsample2), m1 is its
16 kHz form, and the line noise is white at 16 kHz. echo_db raises the echo
over the model. hoth_shift moves the near noise that many samples earlier.

Files of a case: ref.s16 and mic.s16 (int16 little-endian), echo.f32 and
noise.f32 (Hoth plus line noise, float32) and meta.json. meta.json holds the
parameters, t0_sample and t0_frame (the first burst), frames,
css_end_sample, hoth_frame (the first frame of the Hoth noise), kind 2cc
and nlp 0.
"""
import argparse
import json
import os
import sys

import numpy as np

import siggen as sg
from work import int_list, names_check

FS_WIRE = 8000
BAND_HZ = (300.0, 3400.0)
FAR_DBM0 = -10.0
ERL_DB = 6.0
HOTH_SEED = 34
HOTH_SEED_STEP = 101
TEMPLATES = {
    'rng': 'w-{fs}-{peak}-e{echo_db:.1f}-s{seed}',
    'seed': 'p-{fs}-{peak}-e{echo_db:.1f}-s{seed}',
    'tail': 'k-{fs}-{peak}-e{echo_db:.1f}-s{seed}',
}


def build(out, fs, seed, peak, ref_loss_db=9.0, lead_s=1.97, l_sgen=-30.0,
          line_dbm0=-65.0, periods=16, probe_periods=2, css_seed=0,
          phase=None, sgen_preroll_s=0.2, hoth_on=True, echo_db=0.0,
          hoth_shift=0):
    """Write one 2C(c) case into out and return its meta dict. phase None
    draws the frame phase from the case's generator."""
    rng = np.random.default_rng([seed, 7])
    if phase is None:
        phase = int(rng.integers(0, fs // 100))
    period = sg.css_period_get(FS_WIRE)
    lead = round(lead_s * FS_WIRE)
    n_css = periods * period
    n = lead + n_css + probe_periods * period
    far8 = np.zeros(n)
    far8[lead:lead + n_css] = sg.css_single(FAR_DBM0, n_css / FS_WIRE,
                                            FS_WIRE, css_seed)
    near8 = np.zeros(n)
    if hoth_on:
        h0 = lead - round(sgen_preroll_s * FS_WIRE)
        near8[h0:] = sg.hoth_metered(l_sgen, (n - h0) / FS_WIRE,
                                     HOTH_SEED + HOTH_SEED_STEP * seed,
                                     BAND_HZ)
    g = sg.hybrid_m1(ERL_DB, fs)
    delay = peak - int(np.argmax(np.abs(g)))
    if delay < 0:
        raise ValueError('peak before the model peak')
    if fs == FS_WIRE:
        far, nearc = far8, near8
    else:
        far, nearc = sg.upsample2(far8), sg.upsample2(near8)
    if hoth_shift:
        nearc = np.concatenate((nearc[hoth_shift:], np.zeros(hoth_shift)))
    echo = sg.fir(g, far)
    echo = np.concatenate((np.zeros(delay), echo))[:len(far)] * \
        10 ** (echo_db / 20)
    line_rms = sg.dbm0_to_rms(line_dbm0)
    line = rng.standard_normal(len(far)) * line_rms
    pad = np.zeros(phase)
    far = np.concatenate((pad, far))
    echo = np.concatenate((pad, echo))
    nearc = np.concatenate((pad, nearc))
    line = np.concatenate((rng.standard_normal(phase) * line_rms, line))
    frame = fs // 100
    total = (len(far) // frame) * frame
    far, echo, nearc, line = far[:total], echo[:total], nearc[:total], \
        line[:total]
    noise = nearc + line
    os.makedirs(out, exist_ok=True)
    sg.to_int16(far * 10 ** (-ref_loss_db / 20)).tofile(f'{out}/ref.s16')
    sg.to_int16(echo + noise).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    noise.astype('<f4').tofile(f'{out}/noise.f32')
    t0 = phase + lead * (fs // FS_WIRE)
    meta = dict(fs=fs, seed=seed, peak=peak, delay=delay, phase=phase,
                ref_loss_db=ref_loss_db, lead_s=lead_s, l_sgen=l_sgen,
                line_dbm0=line_dbm0, t0_sample=t0, t0_frame=t0 / frame,
                frames=total // frame,
                css_end_sample=t0 + n_css * (fs // FS_WIRE),
                hoth_on=hoth_on, echo_db=echo_db, hoth_shift=hoth_shift,
                hoth_frame=int((t0 - (sgen_preroll_s * fs) - hoth_shift) //
                               frame),
                kind='2cc', nlp=0)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def phase_get(mode, fs, seed):
    """The frame phase of a case under --phase mode."""
    if mode == 'seed':
        return seed
    if mode == 'tail':
        return fs // 100 - seed
    return None


def main(argv):
    p = argparse.ArgumentParser(prog='stim.py', description=__doc__.split(
        '\n')[0])
    p.add_argument('outdir')
    p.add_argument('--fs', type=int, nargs='+', default=[8000])
    p.add_argument('--peak', type=int, nargs='+', required=True)
    p.add_argument('--seeds', type=int_list, default=[0])
    p.add_argument('--phase', choices=sorted(TEMPLATES), default='rng')
    p.add_argument('--echo-db', type=float, nargs='+', default=[0.0])
    p.add_argument('--hoth-shift', type=int, default=0)
    p.add_argument('--name')
    p.add_argument('--ref-loss', type=float, default=9.0)
    p.add_argument('--l-sgen', type=float, default=-30.0)
    p.add_argument('--line', type=float, default=-65.0)
    p.add_argument('--lead', type=float, default=1.97)
    p.add_argument('--no-hoth', action='store_true')
    a = p.parse_args(argv)
    template = a.name or TEMPLATES[a.phase]
    grid = []
    for fs in a.fs:
        for peak in a.peak:
            for echo_db in a.echo_db:
                for seed in a.seeds:
                    phase = phase_get(a.phase, fs, seed)
                    name = template.format(fs=fs, peak=peak, echo_db=echo_db,
                                           seed=seed, phase=phase)
                    grid.append((name, fs, peak, echo_db, seed, phase))
    names_check([(g[0], 'fs %d peak %d echo_db %g seed %d' % g[1:5])
                 for g in grid])
    for name, fs, peak, echo_db, seed, phase in grid:
        m = build(os.path.join(a.outdir, name), fs, seed, peak, a.ref_loss,
                  a.lead, a.l_sgen, a.line, phase=phase,
                  hoth_on=not a.no_hoth, echo_db=echo_db,
                  hoth_shift=a.hoth_shift)
        print(name, 'phase', m['phase'], 't0_frame', '%.2f' % m['t0_frame'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
