#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""A markdown table of run summaries, one row per build and case.

usage: report_tab.py <tag> <build>[,<build>...] [<glob>]

The rows come from sum/<tag>/<build>/<case>.json of the work directory,
for the cases whose name matches glob (default *). The columns:

- case: the build and the case name;
- the tracker and the true ERLE at 1, 2, 4, 6 and 10 s;
- the adapted, held and rolled-back frames of the first 6 s;
- the trials: each start as arm@frame, each end as end@frame;
- the longest latch, path, cohere and stale runs;
- the trusted frames, and the trust count at 6 s;
- the coefficient energy and its peak tap at 1, 2, 4, 6 and 10 s;
- the floor in dBov at 1 s and 6 s.

A summary of a conv case holds the figures of convtab.py instead. Such
summaries print in a second table, the one of convtab.py, with one row per
phase of each case. A summary of a cn case holds the figures of cntab.py,
and prints in the table of cntab.py, one row per cn phase. A summary of a
dt case holds the figures of dttab.py, and prints in its table, one row per
case. A summary of a fax case holds the figures of faxtab.py, and prints in
its table, one row per case.
"""
import sys

import cntab
import convtab
import dttab
import faxtab
import work
from table import case_key

ARMS = {1: 'latch', 2: 'path', 3: 'stale', 4: 'cohere'}
ENDS = {1: 'kept', 2: 'refused', 3: 'diverged', 4: 'talker'}
HEAD = ('| case | tracker ERLE 1/2/4/6/10 s | true ERLE 1/2/4/6/10 s | '
        'adapted/held/rolled back, 0-6 s | trials | max runs '
        'latch/path/cohere/stale | trusted frames / trust count at 6 s | '
        'coef energy@peak tap at 1/2/4/6/10 s | floor dBov at 1/6 s |')


def at(r, t):
    """The per-second dict that ends at t seconds."""
    c = [c for c in r['seconds'] if abs(c['t'] - (t - 1)) < 0.6]
    return c[0] if c else r['seconds'][-1]


def row_get(r, label):
    s6 = [c for c in r['seconds'] if 0 <= c['t'] < 6]
    sall = [c for c in r['seconds'] if c['t'] >= 0]
    ad = sum(c['adapt'] for c in s6)
    hd = sum(c['held_dt'] for c in s6)
    rb = sum(c['rollback'] for c in s6)
    tr = ','.join('%s@%d:' % (ARMS[t['arm']], t['frame'])
                  for t in r['trials'] if t['arm'])
    en = ','.join('%s@%d' % (ENDS[t['end']], t['frame'])
                  for t in r['trials'] if t['end'])
    runs = '%d/%d/%d/%d' % (max(c['dt_run_max'] for c in sall),
                            max(c['path_max'] for c in sall),
                            max(c['cohere_max'] for c in sall),
                            max(c['stale_max'] for c in sall))
    trust = '%d/%d' % (sum(c['trusted'] for c in sall), at(r, 6)['trust_run'])
    ce = ' '.join('%.2f@%d' % (at(r, t)['coef'], at(r, t)['peak'])
                  for t in (1, 2, 4, 6, 10))
    return '| %s | %s | %s | %d/%d/%d | %s | %s | %s | %s | %.1f/%.1f |' % (
        label, ' '.join('%.1f' % v for v in r['erle_trk'].values()),
        ' '.join('%.1f' % v for v in r['erle_true'].values()), ad, hd, rb,
        (tr + ' ' + en).strip() or 'none', runs, trust, ce,
        at(r, 1)['floor_dbov'], at(r, 6)['floor_dbov'])


def main(argv):
    if len(argv) < 2:
        sys.exit('usage: report_tab.py <tag> <build>[,<build>...] [<glob>]')
    pattern = argv[2] if len(argv) > 2 else '*'
    rows, conv, cn, dt, fax = [], [], [], [], []
    for b in work.names(argv[1]):
        for case, r in sorted(work.summaries(argv[0], b, pattern),
                              key=lambda cr: case_key(cr[0])):
            if convtab.summary_is(r):
                conv += [convtab.row_get(b, case, r, ph['name'])
                         for ph in r['phases']]
            elif cntab.summary_is(r):
                cn += [cntab.row_get(b, case, ph) for ph in r['cn']]
            elif dttab.summary_is(r):
                dt.append(dttab.row_get(b, case, r))
            elif faxtab.summary_is(r):
                fax.append(faxtab.row_get(b, case, r))
            else:
                rows.append(row_get(r, '%s %s' % (b, case)))
    if rows or not (conv or cn or dt or fax):
        print(HEAD)
        print('|' + '---|' * 9)
        for line in rows:
            print(line)
    if conv:
        print(convtab.HEAD)
        print('|' + '---|' * convtab.cols_get(convtab.HEAD))
        for line in conv:
            print(line)
    if cn:
        print(cntab.HEAD)
        print('|' + '---|' * (cntab.HEAD.count('|') - 1))
        for line in cn:
            print(line)
    if dt:
        print(dttab.HEAD)
        print('|' + '---|' * (dttab.HEAD.count('|') - 1))
        for line in dt:
            print(line)
    if fax:
        print(faxtab.HEAD)
        print('|' + '---|' * (faxtab.HEAD.count('|') - 1))
        for line in fax:
            print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
