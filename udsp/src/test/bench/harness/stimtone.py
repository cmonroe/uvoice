#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Near single tone over a far CSS, a stimulus for lec_hz.

usage: stimtone.py <outdir> --level DBM0 [DBM0 ...] --hz F [F ...]
                   [--fs R [R ...]] [--seeds LIST] [--peak N]
                   [--name TEMPLATE]

Each case of the grid fs x level x hz x seed goes to <outdir>/<name>. LIST
is a comma list of integers and ranges A-B. TEMPLATE is a str.format pattern
over fs, level, hz and seed; the default is {fs}-{level:g}-{hz:g}-s{seed}.
Two grid points that give one name stop the program with status 1 before
it writes a case.

The far CSS (siggen.css_single at -10 dBm0, the case's seed) starts after
1 s and runs 14 s. A near tone of level and hz plays from 7 s after the CSS
starts, for 4 s. The far end reaches the canceller through m1 of Annex D at
ERL 6 dB with its peak on tap peak (523 at 16 kHz and 262 at 8 kHz by
default), the reference stands 9 dB under the line, and white line noise at
-65 dBm0 lies on the microphone. At 16 kHz both ends are 8 kHz signals
raised to 16 kHz.

Files of a case: ref.s16, mic.s16 and meta.json. meta.json holds tone_from
and tone_to, the first frame of the tone and the frame after its end, kind
tone and nlp 1.
"""
import argparse
import json
import os
import sys

import numpy as np

import siggen as sg
from work import int_list, names_check

FS_WIRE = 8000
LEAD_S = 1.0
DUR_S = 14.0
TONE_AT_S = 7.0
TONE_S = 4.0
FAR_DBM0 = -10.0
LINE_DBM0 = -65.0
REF_LOSS_DB = 9.0
ERL_DB = 6.0
PEAK = {8000: 262, 16000: 523}


def build(out, fs, seed, level_dbm0, hz, peak=None):
    """Write one case into out and return its meta dict."""
    peak = PEAK[fs] if peak is None else peak
    rng = np.random.default_rng([seed, 9])
    phase = int(rng.integers(0, fs // 100))
    n8 = int((LEAD_S + DUR_S) * FS_WIRE)
    lead = int(LEAD_S * FS_WIRE)
    far8 = np.zeros(n8)
    far8[lead:] = sg.css_single(FAR_DBM0, DUR_S, FS_WIRE, seed)[:n8 - lead]
    tone8 = np.zeros(n8)
    a = int((LEAD_S + TONE_AT_S) * FS_WIRE)
    b = a + int(TONE_S * FS_WIRE)
    tone8[a:b] = sg.tone(hz, level_dbm0, TONE_S, FS_WIRE)
    far = far8 if fs == FS_WIRE else sg.upsample2(far8)
    tone = tone8 if fs == FS_WIRE else sg.upsample2(tone8)
    g = sg.hybrid_m1(ERL_DB, fs)
    echo = np.concatenate((np.zeros(peak - int(np.argmax(np.abs(g)))),
                           sg.fir(g, far)))[:len(far)]
    line = rng.standard_normal(len(far)) * sg.dbm0_to_rms(LINE_DBM0)
    pad = np.zeros(phase)
    ref = np.concatenate((pad, far)) * 10 ** (-REF_LOSS_DB / 20)
    mic = np.concatenate((pad, echo + tone + line))
    frame = fs // 100
    total = (len(ref) // frame) * frame
    os.makedirs(out, exist_ok=True)
    sg.to_int16(ref[:total]).tofile(f'{out}/ref.s16')
    sg.to_int16(mic[:total]).tofile(f'{out}/mic.s16')
    meta = dict(fs=fs, seed=seed, tone_dbm0=level_dbm0, tone_hz=hz,
                tone_from=(phase + a * fs // FS_WIRE) // frame,
                tone_to=(phase + b * fs // FS_WIRE) // frame, peak=peak,
                phase=phase, ref_loss_db=REF_LOSS_DB, frames=total // frame,
                kind='tone', nlp=1)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stimtone.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('outdir')
    p.add_argument('--fs', type=int, nargs='+', default=[16000])
    p.add_argument('--level', type=float, nargs='+', required=True)
    p.add_argument('--hz', type=float, nargs='+', required=True)
    p.add_argument('--seeds', type=int_list, default=[0])
    p.add_argument('--peak', type=int)
    p.add_argument('--name', default='{fs}-{level:g}-{hz:g}-s{seed}')
    a = p.parse_args(argv)
    grid = [(a.name.format(fs=fs, level=level, hz=hz, seed=seed), fs, level,
             hz, seed) for fs in a.fs for level in a.level for hz in a.hz
            for seed in a.seeds]
    names_check([(g[0], 'fs %d level %g hz %g seed %d' % g[1:])
                 for g in grid])
    for name, fs, level, hz, seed in grid:
        m = build(os.path.join(a.outdir, name), fs, seed, level, hz, a.peak)
        print(name, 'tone frames', m['tone_from'], m['tone_to'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
