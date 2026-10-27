#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Per-digit table of a lec_hz run of a 16A stimulus at one DTMF level.

usage: an16a.py <run> [<level_dbm0>]

<run> is a run directory, or set/build/case under run/ of the work
directory. It holds meta.json, digits.json, frames.csv, out.s16 and
near.f32. For each digit at the level (default -9 dBm0), a line gives its
trial, digit and dial level, whether it starts under the dial tone, the
processor verdicts over its frames (quiet, dt, unlike, erle, pass), the
frames that the processor acted on, adapted and fired, and the loss in dB
of each of its two tones from the near input to the canceller output (a
single-bin DFT over the on span of the digit).
"""
import collections
import json
import math
import sys

import numpy as np

import lecr
import siggen as sg
import work

Run = collections.namedtuple('Run', 'fs digits rows out near')


def load(d):
    """The files of run directory d that the digit figures read."""
    with open(f'{d}/meta.json') as f:
        fs = json.load(f)['fs']
    with open(f'{d}/digits.json') as f:
        digits = json.load(f)
    return Run(fs, digits, work.csv_rows(f'{d}/frames.csv'),
               np.fromfile(f'{d}/out.s16', '<i2').astype(float),
               np.fromfile(f'{d}/near.f32', '<f4').astype(float))


def tone_loss(run, g):
    """The loss in dB of each tone of digit g from the near input to the
    output, over the on span of the digit."""
    a, b = g['on'], g['off']
    return [10 * math.log10(sg.tone_power(run.out[a:b], f, run.fs) /
                            sg.tone_power(run.near[a:b], f, run.fs))
            for f in (g['f_lo'], g['f_hi'])]


def digit_get(run, g):
    """The dict of digit g: its schedule, the verdict counts, the acted,
    adapted and fired frames over it, and the loss of its tones."""
    fr = run.fs // 100
    blk = run.rows[g['on'] // fr:(g['off'] - 1) // fr + 1]
    cnt = {v: 0 for v in lecr.VERDICTS}
    for r in blk:
        cnt[lecr.VERDICTS[int(r['verdict'])]] += 1
    return dict(g, cnt=cnt, loss=tone_loss(run, g),
                acted=sum(int(r['nlp_act']) for r in blk),
                adapt=sum(int(r['adapt']) for r in blk),
                fired=sum(int(r['fired']) for r in blk))


def table(d, level=-9.0):
    """One dict per digit of run directory d at level."""
    run = load(d)
    return [digit_get(run, g) for g in run.digits if g['level'] == level]


def main(argv):
    if not argv:
        sys.exit('usage: an16a.py <run> [<level_dbm0>]')
    level = float(argv[1]) if len(argv) > 1 else -9.0
    for r in table(work.run_arg(argv[0]), level):
        c = r['cnt']
        print('seq %2d dig %s dial %5.1f ovl %d | q %d dt %d un %d er %d pa %d'
              ' | acted %d adapt %d fired %d | loss %.1f %.1f' % (
                  r['seq'], r['digit'], r['dial'], r['overlap'], c['quiet'],
                  c['dt'], c['unlike'], c['erle'], c['pass'], r['acted'],
                  r['adapt'], r['fired'], r['loss'][0], r['loss'][1]))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
