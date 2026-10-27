#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The first frame where two lec_hz runs of one stimulus differ.

usage: firstdiff.py <run_a> <run_b>

Each run is a run directory, or set/build/case under run/ of the work
directory. The command prints the first frame whose row of frames.csv
differs, each column that differs there with both values, and the trial
state of the two frames before it. It prints "no difference" when every
row is equal, and exits 1 when a frame differs.
"""
import sys

import work


def main(argv):
    if len(argv) != 2:
        sys.exit('usage: firstdiff.py <run_a> <run_b>')
    a = work.csv_rows(work.run_arg(argv[0]) / 'frames.csv')
    b = work.csv_rows(work.run_arg(argv[1]) / 'frames.csv')
    for i, (x, y) in enumerate(zip(a, b)):
        diff = [k for k in x if x[k] != y[k]]
        if not diff:
            continue
        print('first differing frame', x['frame'], 'columns', diff)
        for k in diff:
            print('   %-12s %s -> %s' % (k, x[k], y[k]))
        for j in range(max(0, i - 2), i):
            print('   prev frame', a[j]['frame'], 'arm', a[j]['arm'],
                  'probing', a[j]['probing'], 'bare', a[j]['bare'],
                  'no_echo', a[j]['no_echo'])
        return 1
    if len(a) != len(b):
        print('frames differ in count: %d against %d' % (len(a), len(b)))
        return 1
    print('no difference')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
