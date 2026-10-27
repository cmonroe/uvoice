#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The regions of G.168 Tests 10A and 10B, per lec_hz run of a stim10a.py
case.

usage: faxtab.py <set> <build>[,<build>...] [<glob>]

The rows come from run/<set>/<build>/<case>/ of the work directory, for the
cases whose name matches glob (default *). The meter is siggen.peak_track,
the peak level of G.168 clause 6.4.1.2.2, on the output plane at the Sout
level. td is the td_samples of meta.json.

The regions follow g168.test_10, the evaluator of the G.168 bench, and its
10A case.
Test 10A, from the CED start:

- Region I to 0.15 s plus td, at most max(-37, -13 - A_ECHO) dBm0;
- Region II to the first flag of Sequence No. 1, GAP_S after the seq1 phase
  starts, at most -37 dBm0. It holds the gap after the CED;
- Region III to 1.3 s plus td after that flag, at most max(-37, -13 -
  A_ECHO) dBm0;
- Region IV to the end, at most -24 dBm0, or -37 dBm0 with the processor
  on.

Test 10B: Region I to 1.3 s plus td after the start of Sequence No. 2, then
Region II to the end, at most -24 dBm0, or -37 dBm0 with the processor on.
A_ECHO is A_ECHO_DB, the ERL of the bench path.

A 10A row also gives the frames of the gap, from the CED end to the first
flag, on which the processor acts (nlp_act of frames.csv), against the frames
of the gap.
"""
import argparse
import fnmatch
import json
import sys

import numpy as np

import siggen as sg
import work

GAP_S = 0.075
A_ECHO_DB = 6.0
FLOOR_DBM0 = -37.0
L_RIN_DBM0 = -13.0
ENTER_10A_S = 0.15
ENTER_SEQ_S = 1.3
REGION4_DBM0 = -24.0


def phase_get(meta, name):
    for p in meta['phases']:
        if p['name'] == name:
            return p
    raise KeyError(name)


def regions_get(meta):
    """(region, start, end, cap dBm0) per region, on the samples of the
    plane."""
    fs, td = meta['fs'], meta['td_samples']
    converging = max(FLOOR_DBM0, L_RIN_DBM0 - A_ECHO_DB)
    converged = FLOOR_DBM0 if meta.get('nlp') else REGION4_DBM0
    if meta['test'] == '10b':
        p = phase_get(meta, 'seq2')
        enter = p['start_sample'] + td + round(ENTER_SEQ_S * fs)
        return [(1, p['start_sample'], enter, converging),
                (2, enter, p['end_sample'], converged)]
    ced, seq = phase_get(meta, 'ced'), phase_get(meta, 'seq1')
    flag = seq['start_sample'] + round(GAP_S * fs)
    enter2 = ced['start_sample'] + td + round(ENTER_10A_S * fs)
    enter4 = flag + td + round(ENTER_SEQ_S * fs)
    return [(1, ced['start_sample'], enter2, converging),
            (2, enter2, flag, FLOOR_DBM0),
            (3, flag, enter4, converging),
            (4, enter4, seq['end_sample'], converged)]


def summary(d):
    """The figures of one run directory."""
    d = work.run_arg(str(d))
    meta = json.loads((d / 'meta.json').read_text())
    fs = meta['fs']
    frame = fs // 100
    out = np.fromfile(d / 'out.s16', '<i2').astype(float)
    idx, level = sg.peak_track(out, fs)
    s = dict(case=d.name, fs=fs, test=meta['test'], regions=[])
    for n, a, b, cap in regions_get(meta):
        sel = (idx >= a) & (idx < b)
        if not sel.any():
            s['regions'].append(dict(region=n, peak=None, at_s=None,
                                     cap=cap))
            continue
        k = int(np.argmax(level[sel]))
        s['regions'].append(dict(region=n, peak=float(level[sel][k]),
                                 at_s=(int(idx[sel][k]) -
                                       meta['phases'][0]['start_sample']) /
                                 fs, cap=cap))
    s['gap_acts'] = s['gap_frames'] = None
    if meta['test'] == '10a':
        rows = work.csv_rows(d / 'frames.csv')
        a = phase_get(meta, 'ced')['end_sample'] // frame
        b = (phase_get(meta, 'seq1')['start_sample'] +
             round(GAP_S * fs)) // frame
        s['gap_acts'] = sum(int(r['nlp_act']) for r in rows[a:b])
        s['gap_frames'] = len(rows[a:b])
    return s


def summary_is(s):
    """True for a summary of this module."""
    return isinstance(s, dict) and 'regions' in s and 'gap_acts' in s


def fmt(v, spec='%.1f'):
    return '-' if v is None else spec % v


def row_get(build, case, s):
    """The table row of one summary."""
    cells = []
    for r in s['regions']:
        verdict = '' if r['peak'] is None else (
            ' pass' if r['peak'] <= r['cap'] else ' FAIL')
        cells.append('%s at %s s (%.0f)%s' % (
            fmt(r['peak']), fmt(r['at_s'], '%.3f'), r['cap'], verdict))
    cells += ['-'] * (4 - len(cells))
    gap = '-' if s['gap_acts'] is None else '%d/%d' % (s['gap_acts'],
                                                       s['gap_frames'])
    return '| %s | %s | %s | %s |' % (case, build, ' | '.join(cells), gap)


HEAD = ('| case | build | Region I | Region II | Region III | Region IV | '
        'gap acts |')


def main(argv):
    p = argparse.ArgumentParser(prog='faxtab.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('glob', nargs='?', default='*')
    a = p.parse_args(argv)
    print(HEAD)
    print('|' + '---|' * (HEAD.count('|') - 1))
    for build in a.builds:
        for d in sorted(work.run_dir(a.set, build).glob(a.glob)):
            if (d / 'frames.csv').exists():
                print(row_get(build, d.name, summary(d)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
