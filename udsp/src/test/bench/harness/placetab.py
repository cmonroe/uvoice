#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Markdown tables of the placement cases of place.py.

usage: placetab.py <build> [<tag>]

The cases come from sum/<tag>/<build>/ (default tag place):
pl-<id>-<E>-<H>-s<phase>.json, the echo peak on tap E and the Hoth noise
placed as a whole-microphone shift for tap H puts it. Per capture id and
case, over the phases:

- the phases under 10 dB of true ERLE and under 13 dB of tracker ERLE at
  6 s;
- the phases with onset lockout: 10 or more fired onset frames in 6 s;
- the first rise run of the floor: the phases where it moved the floor, and
  those where it cleared at 50 readings on its spread; the median spread;
- the phases with the floor at the Hoth noise (under 50 dBov) by 2 s and by
  6 s.

A second table sums the cases over the capture ids.
"""
import statistics as st
import sys

import work

LOCKOUT_FIRED = 10
FLOOR_HOTH_DBOV = 50


def main(argv):
    if not argv:
        sys.exit('usage: placetab.py <build> [<tag>]')
    tag = argv[1] if len(argv) > 1 else 'place'
    cases = {}
    for name, r in work.summaries(tag, argv[0], 'pl-*'):
        _, rid, e, h, _ = name.split('-')
        cases.setdefault((rid, int(e), int(h)), []).append(r)
    print('| capture | echo tap / Hoth as for tap | phases | true < 10 dB at '
          '6 s | tracker < 13 dB at 6 s | onset lockout | first rise run: '
          'moved / cleared on spread | median spread dB | floor at Hoth by '
          '2 s / by 6 s |')
    print('|---|---|---|---|---|---|---|---|---|')
    tot = {}
    for (rid, e, h) in sorted(cases, key=lambda k: (k[0], -k[2], -k[1])):
        rs = cases[(rid, e, h)]
        bad = sum(r['erle_true']['6'] < 10 for r in rs)
        trk = sum(r['erle_trk']['6'] < 13 for r in rs)
        lock = sum(r['onset_fired6'] >= LOCKOUT_FIRED for r in rs)
        mv = sum(1 for r in rs if r['rise1'] and r['rise1']['end'] == 'moved')
        cl = sum(1 for r in rs
                 if r['rise1'] and r['rise1']['end'].startswith('cleared at'))
        spreads = [r['rise1']['spread'] for r in rs if r['rise1']]
        sp = st.median(spreads) if spreads else 0
        f2 = sum(r['floor2'] < FLOOR_HOTH_DBOV for r in rs)
        f6 = sum(r['floor6'] < FLOOR_HOTH_DBOV for r in rs)
        t = tot.setdefault((e, h), [0] * 7)
        for i, v in enumerate((len(rs), bad, trk, lock, cl, f2, f6)):
            t[i] += v
        print('| %s | %d / %d | %d | %d | %d | %d | %d / %d | %.0f | %d / %d |'
              % (rid, e, h, len(rs), bad, trk, lock, mv, cl, sp, f2, f6))
    print()
    print('| totals: echo tap / Hoth as for tap | phases | true < 10 dB | '
          'tracker < 13 dB | onset lockout | first run cleared on spread | '
          'floor at Hoth by 2 s / 6 s |')
    print('|---|---|---|---|---|---|---|')
    for (e, h), t in tot.items():
        print('| %d / %d | %d | %d | %d | %d | %d | %d / %d |' % (
            e, h, t[0], t[1], t[2], t[3], t[4], t[5], t[6]))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
