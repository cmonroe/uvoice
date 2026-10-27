#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The frames of a lec_hz run from one time to another.

usage: trace.py <run> <from_s> <to_s>

<run> is a run directory, or set/build/case under run/ of the work
directory. The times count seconds from the first burst (t0 of meta.json).
A line per frame: the powers of the reference, the window, the microphone,
the estimate, the output, the bar, the classic bound and the floor in dB
against 1 LSB squared; window active and active; fired; the hangover; the
latch run; the adapt verdict before and after the rollback; the trial (A
and the arm number on its start, E and the end number on its end, p while
it probes); the coefficient energy, its peak tap and echo_gain; the step
factors; the step floor in dB; the tracked ERLE and the floor in dBov.
"""
import json
import sys

import work

HEAD = ('frame   t     ref   win   mic   est   res   bar  clas noise  act fire '
        'hang dtrun adapt0/1 trial coefE   peak gain  sscale talk  stepfl  '
        'erle  floor')


def db(v):
    return work.db(float(v), 1e-3)


def line_get(r, t0):
    tr = ''
    if int(r['arm']):
        tr += 'A%s' % r['arm']
    if int(r['end']):
        tr += 'E%s' % r['end']
    if int(r['probing']):
        tr += 'p'
    return ('%5d %6.2f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %d%d '
            '%d %3d %4d %d/%d %-5s %8.4f %4d %5.2f %5.3f %5.3f %7.1f %5.1f '
            '%5.1f' % (
                int(r['frame']), (int(r['frame']) - t0) / 100, db(r['ref']),
                db(r['win']), db(r['mic']), db(r['est']), db(r['res']),
                db(r['bar']), db(r['classic']), db(r['noise']),
                int(r['wact']), int(r['act']), int(r['fired']),
                int(r['hang']), int(r['dt_run']), int(r['adapt0']),
                int(r['adapt']), tr, float(r['coef']), int(r['peak']),
                float(r['gain']), float(r['step_scale']),
                float(r['talk_scale']), db(r['step_floor']),
                float(r['erle']), float(r['noise_dbov'])))


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: trace.py <run> <from_s> <to_s>')
    d = work.run_arg(argv[0])
    with open(d / 'meta.json') as f:
        t0 = json.load(f)['t0_frame']
    rows = work.csv_rows(d / 'frames.csv')
    a = int(t0 + float(argv[1]) * 100)
    b = int(t0 + float(argv[2]) * 100)
    print(HEAD)
    for r in rows[a:b]:
        print(line_get(r, t0))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
