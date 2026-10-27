#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Convergence over the seeds of each geometry of a summary tag.

usage: sweeptab.py <tag> <build>[,<build>...]

A geometry is the case name less its -s<seed> end, for example
w-16000-523-e1.0. For each geometry and build, the line holds:

- conv: the seeds whose tracker ERLE at 6 s is 13 dB or more, of all;
- trk6, tru6, tru10: the median tracker ERLE at 6 s and the median true
  ERLE at 6 s and 10 s;
- trial: the seeds with any trial;
- offpk: the seeds whose filter peak at 5 s stands more than 3 taps off
  the peak of the stimulus.
"""
import statistics as st
import sys

import work

CONV_DB = 13.0
PEAK_TOL = 3


def main(argv):
    if len(argv) != 2:
        sys.exit('usage: sweeptab.py <tag> <build>[,<build>...]')
    rows = {}
    for b in work.names(argv[1]):
        for case, r in work.summaries(argv[0], b):
            rows.setdefault((case.rsplit('-s', 1)[0], b), []).append(r)
    print('%-22s %-6s %5s %6s %6s %6s %6s %6s' % (
        'geometry/offset', 'build', 'conv', 'trk6', 'tru6', 'tru10',
        'trial', 'offpk'))
    for key, b in sorted(rows):
        rs = rows[(key, b)]
        conv = sum(r['erle_trk']['6'] >= CONV_DB for r in rs)
        trial = sum(bool(r['trials']) for r in rs)
        off = 0
        for r in rs:
            c = [c for c in r['seconds'] if abs(c['t'] - 5.0) < 0.6]
            if c and abs(c[0]['peak'] - r['meta']['peak']) > PEAK_TOL:
                off += 1
        print('%-22s %-6s %2d/%-2d %6.1f %6.1f %6.1f %6d %6d' % (
            key, b, conv, len(rs),
            st.median(r['erle_trk']['6'] for r in rs),
            st.median(r['erle_true']['6'] for r in rs),
            st.median(r['erle_true']['10'] for r in rs), trial, off))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
