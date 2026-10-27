#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""One line per lec_hz run of a set, from the frames of the run.

usage: table.py <set> <build>[,<build>...] [<glob>]

For each build and each case of run/<set>/<build>/ whose name matches glob
(default *), the line holds the tracker and the true ERLE at 1, 2, 4, 6 and
10 s, the active, held (by the detector or by a rollback) and adapted
frames of the first 6 s, the floor in dBov, the coefficient energy and its
peak tap at 5 s, and the trials. A trial reads as its arm letter (L latch,
P path, S stale, C cohere), its time in s and its end letter (k kept,
r refused, d diverged, t talker).
"""
import os
import sys

import analyze
import work

ARMS = 'LPSC'
ENDS = {1: 'k', 2: 'r', 3: 'd', 4: 't'}


def case_key(path):
    """Sort key of a case name: its stem, then its seed."""
    stem, _, seed = os.path.basename(path).rpartition('-s')
    return (stem, int(seed)) if seed.isdigit() else (os.path.basename(path),
                                                       0)


def line_get(build, d):
    r = analyze.summary(d)
    s6 = [c for c in r['seconds'] if 0 <= c['t'] < 6]
    held = sum(c['held_dt'] + c['rollback'] for c in s6)
    act = sum(c['act'] for c in s6)
    adapt = sum(c['adapt'] for c in s6)
    c5 = [c for c in r['seconds'] if abs(c['t'] - 5) < 0.6]
    c5 = c5[0] if c5 else r['seconds'][-1]
    trials = ' '.join('%s%.1f%s' % (ARMS[t['arm'] - 1] if t['arm'] else '',
                                    t['t'], ENDS.get(t['end'], ''))
                      for t in r['trials'])
    return ('%-4s %-20s trk %s | tru %s | act %4d held %4d adapt %4d | '
            'floor %5.1f coef %.3g peak %4d | %s' % (
                build, os.path.basename(d),
                ' '.join('%5.1f' % v for v in r['erle_trk'].values()),
                ' '.join('%5.1f' % v for v in r['erle_true'].values()),
                act, held, adapt, c5['floor_dbov'], c5['coef'], c5['peak'],
                trials))


def main(argv):
    if len(argv) < 2:
        sys.exit('usage: table.py <set> <build>[,<build>...] [<glob>]')
    pattern = argv[2] if len(argv) > 2 else '*'
    for b in work.names(argv[1]):
        dirs = [p for p in work.run_dir(argv[0], b).glob(pattern)
                if (p / 'frames.csv').exists()]
        for d in sorted(dirs, key=case_key):
            print(line_get(b, d))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
