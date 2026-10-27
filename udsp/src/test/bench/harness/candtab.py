#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""A markdown table of canceller variants over a phase sweep and the bench
placements.

usage: candtab.py <build>[,<build>...] [--phase TAG] [--place TAG]

The phase sweep comes from sum/<TAG>/<build>/ (default tag p): the summaries
of a stim.py --phase seed set. A geometry is one rate, peak tap and echo
level of meta.json. For each geometry the table gives:

- the phases with a tracker ERLE of 13 dB or more and a true ERLE of 10 dB
  or more at 6 s, of all, and the figures of the worst phase;
- the phases with the floor at the Hoth noise (under 50 dBov) by 2 s and by
  6 s, and the median and the latest time of the floor's rise.

The bench columns come from sum/<TAG>/<build>/ (default tag place): the
place.py cases whose echo and Hoth noise stand at one tap E. Each gives the
phases under 10 dB of true ERLE and under 13 dB of tracker ERLE at 6 s, and
the phases with the floor at the Hoth noise by 2 s and by 6 s.
"""
import argparse
import statistics as st
import sys

import work

FLOOR_HOTH_DBOV = 50
TRK_DB = 13.0
TRUE_DB = 10.0


def phase_cell(rs):
    n = len(rs)
    ok = sum(r['erle_trk']['6'] >= TRK_DB and r['erle_true']['6'] >= TRUE_DB
             for r in rs)
    f2 = sum(r['floor2'] < FLOOR_HOTH_DBOV for r in rs)
    f6 = sum(r['floor6'] < FLOOR_HOTH_DBOV for r in rs)
    rt = [r['floor_rise_t'] for r in rs if r['floor_rise_t'] is not None]
    worst = min(rs, key=lambda r: r['erle_trk']['6'])
    rise = ('median rise %.2f s, latest %.2f s' % (st.median(rt), max(rt))
            if rt else 'no rise')
    return ('%d/%d (worst %.1f/%.1f)' % (ok, n, worst['erle_trk']['6'],
                                         worst['erle_true']['6']),
            '%d/%d by 2 s, %d by 6 s, %s' % (f2, n, f6, rise))


def bench_cell(rs):
    bad = sum(r['erle_true']['6'] < TRUE_DB for r in rs)
    trk = sum(r['erle_trk']['6'] < TRK_DB for r in rs)
    f2 = sum(r['floor2'] < FLOOR_HOTH_DBOV for r in rs)
    f6 = sum(r['floor6'] < FLOOR_HOTH_DBOV for r in rs)
    return '%d/%d true<10, %d trk<13, floor by 2 s %d, by 6 s %d' % (
        bad, len(rs), trk, f2, f6)


def main(argv):
    p = argparse.ArgumentParser(prog='candtab.py')
    p.add_argument('builds', type=work.names)
    p.add_argument('--phase', default='p')
    p.add_argument('--place', default='place')
    a = p.parse_args(argv)
    head, rows = None, []
    for b in a.builds:
        syn = {}
        for _, r in work.summaries(a.phase, b):
            m = r['meta']
            syn.setdefault((m['fs'], m['peak'], m.get('echo_db', 0.0)),
                           []).append(r)
        place = {}
        for name, r in work.summaries(a.place, b):
            e, h = name.split('-')[2:4]
            if e == h:
                place.setdefault(int(e), []).append(r)
        cells = []
        for key in sorted(syn, reverse=True):
            cells.extend(phase_cell(syn[key]))
        cells.extend(bench_cell(place[e]) for e in sorted(place,
                                                          reverse=True))
        if head is None:
            head = ['build']
            for fs, peak, echo_db in sorted(syn, reverse=True):
                head += ['%d Hz tap %d echo %+.1f dB phases' % (fs, peak,
                                                                 echo_db),
                         '%d Hz floor' % fs]
            head += ['bench tap %d' % e for e in sorted(place, reverse=True)]
        rows.append([b] + cells)
    if head is None:
        return 1
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    for r in rows:
        print('| ' + ' | '.join(r) + ' |')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
