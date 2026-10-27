#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Double talk from the start and after convergence: the G.168 Test 3A and
3B figures, per lec_hz run of a stimdt.py or stimdtcap.py case.

usage: dttab.py <set> <build>[,<build>...] [<glob>] [--mean] [--summary]

The rows come from run/<set>/<build>/<case>/ of the work directory, for the
cases whose name matches glob (default *). With --summary, <set> names a
summary tag instead, and the rows come from sum/<set>/<build>/<case>.json:
the summaries that `lec_harness.py run --summary <set>` saved with this
module (summary).

The residual is the output less the near end of the case (near.f32), the
I.7.4 subtraction of g168.test_3 of the G.168 bench. On a stimdt.py
case the near end is the one the case adds, and on a stimdtcap.py case the
microphone less the fitted echo. The meter is siggen.meter_track, the
clause 6.4.1.2.1 level meter, on the plane of the case. td is the
td_samples of meta.json.

The figures of each row, over the dt phase:

- l_res: the loudest meter reading of the residual from the dt start plus
  td plus CONVERGE_S to the dt end, the l_res_dbm0 of Test 3A, and its
  margin over L_Sgen,act. Test 3A asks for a margin of 0 or less;
- rise: the loudest meter reading of the residual from the dt start plus
  td to the dt end, less the Figure 11 line at L_Rin,act, the
  dt_residual_rise_db of Test 3B. Test 3B asks for 10 dB or less at X 0,
  and 3 dB or less at X 6 to 30. On a case that the bench played
  prescale_db under its named level (stimdtcap.py), the line is that of
  L_Rin,act plus prescale_db, less prescale_db, as in the evaluator;
- ERLE of the first second and of the l_res window: the echo (echo.f32)
  over the residual, in dB;
- from frames.csv over the dt phase: fired, held and adapted frames, the
  trials as arm:end, the largest coefficient energy, the coefficient
  energy and the peak tap at the dt end, and the frames on which a
  divergence reset zeroed the filter outside a trial.

--mean prints one row per build and case stem instead (the case name less
its -s<seed>): the mean over the seeds of l_res, its margin, the rise and
the two ERLE figures, and the worst margin and rise.
"""
import argparse
import fnmatch
import json
import sys

import numpy as np

import siggen as sg
import work

CONVERGE_S = 5.0
ERLE_FIRST_S = 1.0
# G.168 Figure 11: L_RES <= FIG11_SLOPE * L_Rin,act + FIG11_AT_0 dBm0.
FIG11_SLOPE = 25.0 / 30.0
FIG11_AT_0 = -30.0
ARMS = {0: '-', 1: 'latch', 2: 'path', 3: 'stale', 4: 'cohere'}
ENDS = {0: '-', 1: 'kept', 2: 'refused', 3: 'diverged', 4: 'talker'}


def db(num, den):
    return 10 * np.log10(num / den) if num > 0 and den > 0 else None


def phase_get(meta, name):
    for p in meta['phases']:
        if p['name'] == name:
            return p
    raise KeyError(name)


def frames_get(rows, a, b):
    """The frame figures of the frames that hold samples a to b - 1."""
    fired = held = adapted = 0
    trials = []
    resets = 0
    coef_max = 0.0
    prev = None
    for r in rows[a:b]:
        coef = float(r['coef'])
        fired += int(r['fired'])
        held += int(r['act']) and not int(r['adapt0'])
        adapted += int(r['adapt'])
        if int(r['arm']):
            trials.append(ARMS[int(r['arm'])])
        if int(r['end']):
            trials.append(ENDS[int(r['end'])])
        if prev is not None and prev > 0.0 and coef == 0.0 and \
                not int(r['probing']):
            resets += 1
        coef_max = max(coef_max, coef)
        prev = coef
    last = rows[min(b, len(rows)) - 1]
    return dict(fired=fired, held=held, adapted=adapted,
                trials=' '.join(trials) or '-', resets=resets,
                coef_max=coef_max, coef_end=float(last['coef']),
                peak_end=int(last['peak']))


def summary(d):
    """The figures of one run directory."""
    d = work.run_arg(str(d))
    meta = json.loads((d / 'meta.json').read_text())
    fs = meta['fs']
    out = np.fromfile(d / 'out.s16', '<i2').astype(float)
    near = np.fromfile(d / 'near.f32', '<f4').astype(float)[:len(out)]
    echo = np.fromfile(d / 'echo.f32', '<f4').astype(float)[:len(out)]
    res = out - near
    dt = phase_get(meta, 'dt')
    td = meta['td_samples']
    a, b = dt['start_sample'], min(dt['end_sample'], len(out))
    idx, level = sg.meter_track(res, fs)
    w = a + td + round(CONVERGE_S * fs)
    sel = (idx >= w) & (idx < b)
    l_res = float(level[sel].max()) if sel.any() else None
    sel = (idx >= a + td) & (idx < b)
    worst = float(level[sel].max()) if sel.any() else None
    pre = meta.get('prescale_db', 0.0)
    line = FIG11_SLOPE * (meta['l_rin_act_dbm0'] + pre) + FIG11_AT_0 - pre
    f1 = a + td + round(ERLE_FIRST_S * fs)
    frame = fs // 100
    rows = work.csv_rows(d / 'frames.csv')
    s = dict(case=d.name, fs=fs, variant=meta.get('variant'),
             level=meta.get('level'), x_db=meta.get('x_db'),
             l_sgen_act=meta['l_sgen_act_dbm0'], l_res=l_res,
             margin=None if l_res is None else
             l_res - meta['l_sgen_act_dbm0'],
             rise=None if worst is None else worst - line,
             erle_1s=db(np.sum(echo[a:f1] ** 2), np.sum(res[a:f1] ** 2)),
             erle_win=db(np.sum(echo[w:b] ** 2), np.sum(res[w:b] ** 2)))
    s.update(frames_get(rows, a // frame, -(-b // frame)))
    return s


def fmt(v, spec='%.1f'):
    return '-' if v is None else spec % v


HEAD = ('| case | build | l_res | margin | rise | ERLE 1 s | ERLE window | '
        'L_Sgen,act | fired/held/adapted | trials | coef max | coef @ peak | '
        'resets |')


def summary_is(s):
    """True for a summary of this module."""
    return 'rise' in s and 'erle_win' in s


def row_get(build, case, s):
    """The table row of one summary."""
    return ('| %s | %s | %s | %s | %s | %s | %s | %s | %d/%d/%d | %s | %.2f '
            '| %.2f @ %d | %d |' % (
                case, build, fmt(s['l_res']), fmt(s['margin'], '%+.1f'),
                fmt(s['rise'], '%+.1f'), fmt(s['erle_1s']),
                fmt(s['erle_win']), fmt(s['l_sgen_act']), s['fired'],
                s['held'], s['adapted'], s['trials'], s['coef_max'],
                s['coef_end'], s['peak_end'], s['resets']))


def mean_print(groups):
    print('| stem | build | runs | l_res | margin | worst margin | rise | '
          'worst rise | ERLE 1 s | ERLE window |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for (stem, build), ss in sorted(groups.items()):
        def m(key):
            v = [s[key] for s in ss if s[key] is not None]
            return float(np.mean(v)) if v else None

        def top(key):
            v = [s[key] for s in ss if s[key] is not None]
            return max(v) if v else None
        print('| %s | %s | %d | %s | %s | %s | %s | %s | %s | %s |' % (
            stem, build, len(ss), fmt(m('l_res')), fmt(m('margin'), '%+.1f'),
            fmt(top('margin'), '%+.1f'), fmt(m('rise'), '%+.1f'),
            fmt(top('rise'), '%+.1f'), fmt(m('erle_1s')),
            fmt(m('erle_win'))))


def stem_get(case):
    head, _, tail = case.rpartition('-s')
    return head if head and tail.isdigit() else case


def main(argv):
    p = argparse.ArgumentParser(prog='dttab.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('glob', nargs='?', default='*')
    p.add_argument('--mean', action='store_true')
    p.add_argument('--summary', action='store_true')
    a = p.parse_args(argv)
    groups = {}
    if not a.mean:
        print(HEAD)
        print('|' + '---|' * (HEAD.count('|') - 1))
    for build in a.builds:
        if a.summary:
            items = work.summaries(a.set, build, a.glob)
        else:
            items = [(d.name, summary(d)) for d in
                     sorted(work.run_dir(a.set, build).glob(a.glob))
                     if (d / 'frames.csv').exists()]
        for case, s in items:
            if not fnmatch.fnmatch(case, a.glob):
                continue
            if a.mean:
                groups.setdefault((stem_get(case), build), []).append(s)
            else:
                print(row_get(build, case, s))
    if a.mean:
        mean_print(groups)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
