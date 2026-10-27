#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Every coherence trial of a lec_hz run, with the readings of its run.

usage: arm.py <run> [--runs N]

<run> is a run directory, or set/build/case under run/ of the work
directory; its link stim names the stimulus. A trial starts on a frame
whose arm column reads 4 (UDSP_EC_NLMS_TRIAL_ARM_COHERE). Its run is the
frames before it with a cohere_run over 0, or the 40 frames before it. One
JSON line per trial gives:

- the frame, its time and the first frame of the run;
- ncc and share on the start frame, and their largest values over the run;
- the largest talker excess (the microphone over the guard window) over the
  active frames of the run, and the talker ratio, in dB;
- the median and the largest microphone over the estimate in dB;
- the shape (spec.shape) of the reference and of the microphone over the
  run: the two-peak share and the spectral flatness;
- the end of the trial, and the digits of digits.json that it overlaps
  (from 40 frames before it starts to its end).

--runs N also lists each run of N frames or more that ends without a
trial, with the same readings.
"""
import json
import os
import sys

import numpy as np

import spec
import work

ARM_COHERE = 4
RUN_BACK = 40
NCC_NONE = 9


def summarize(rows, ref, mic, fr, fs, a, b):
    run = rows[a:b + 1]
    act = [r for r in run if int(r['act'])]
    ncc = [float(r['ncc']) for r in run if float(r['ncc']) < NCC_NONE]
    sh = [float(r['share']) for r in run if float(r['share']) >= 0]
    tex = [work.db(float(r['mic']) / max(float(r['guard']), 1e-9))
           for r in act]
    eex = [work.db(float(r['mic']) / max(float(r['est']), 1e-9))
           for r in act]
    rs = spec.shape(ref[a * fr:(b + 1) * fr], fs)
    ms = spec.shape(mic[a * fr:(b + 1) * fr], fs)
    return dict(ncc_arm=float(run[-1]['ncc']),
                share_arm=float(run[-1]['share']),
                ncc_max=max(ncc) if ncc else None,
                share_max=max(sh) if sh else None,
                talk_excess_max=max(tex) if tex else None,
                talker_ratio_db=work.db(float(run[-1]['talker_ratio'])),
                est_excess_med=float(np.median(eex)) if eex else None,
                est_excess_max=max(eex) if eex else None,
                ref_top2=rs[0], ref_flat=rs[1], mic_top2=ms[0],
                mic_flat=ms[1])


def rounded(d):
    return {k: (round(v, 3) if isinstance(v, float) else v)
            for k, v in d.items()}


def main(argv):
    if not argv:
        sys.exit('usage: arm.py <run> [--runs N]')
    d = work.run_arg(argv[0])
    minrun = int(argv[argv.index('--runs') + 1]) if '--runs' in argv \
        else None
    with open(d / 'meta.json') as f:
        fs = json.load(f)['fs']
    fr = fs // 100
    rows = work.csv_rows(d / 'frames.csv')
    runs = [int(r['cohere_run']) for r in rows]
    sdir = d / 'stim'
    ref = np.fromfile(sdir / 'ref.s16', '<i2').astype(float)
    mic = np.fromfile(sdir / 'mic.s16', '<i2').astype(float)
    digits = []
    if os.path.exists(d / 'digits.json'):
        with open(d / 'digits.json') as f:
            digits = json.load(f)
    for i, r in enumerate(rows):
        if int(r['arm']) != ARM_COHERE:
            continue
        a = work.run_start(runs, i)
        a = max(0, i - RUN_BACK) if a == i else a
        s = summarize(rows, ref, mic, fr, fs, a, i)
        end = next(((int(x['frame']), int(x['end'])) for x in rows[i + 1:]
                    if int(x['end'])), None)
        f0, f1 = int(r['frame']), end[0] if end else int(r['frame'])
        ov = [(g['level'], g['seq'], g['digit']) for g in digits
              if g['on'] // fr <= f1 and g['off'] // fr >= f0 - RUN_BACK]
        print(json.dumps(rounded(dict(frame=f0, t=f0 / 100,
                                      run_from=int(rows[a]['frame']),
                                      end=end, overlaps=ov, **s))))
    if minrun:
        for i, r in enumerate(rows):
            if int(r['cohere_run']) >= minrun and int(r['arm']) != \
                    ARM_COHERE and (i + 1 >= len(rows) or
                                    int(rows[i + 1]['cohere_run']) == 0):
                a = work.run_start(runs, i)
                s = summarize(rows, ref, mic, fr, fs, a, i)
                print('run without arming', int(rows[a]['frame']),
                      int(r['frame']), 'len', r['cohere_run'],
                      json.dumps(rounded(s)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
