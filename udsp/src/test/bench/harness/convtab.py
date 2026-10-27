#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Convergence after a change of the echo path: the G.168 Test 2 figures and
the canceller's budget of the first second, per lec_hz run of a stimconv.py
case.

usage: convtab.py <set> <build>[,<build>...] [<glob>] [--frames S]
                  [--phase NAME] [--mean] [--summary]

The rows come from run/<set>/<build>/<case>/ of the work directory, for the
cases whose name matches glob (default *). With --summary, <set> names a
summary tag instead, and the rows come from sum/<set>/<build>/<case>.json:
the summaries that `lec_harness.py run --summary <set>` saved with this
module (summary). --frames needs the run directories, so it does not take
--summary.

The combined loss is that of g168.test_2, the evaluator of the G.168 bench:
L_Rin,act of meta.json less the loudest reading of the clause 6.4.1.2.1
level meter on the canceller output (siggen.meter_track, every 8 input
samples) over a window. A phase starts at its start_sample, and an offset
T reads at the anchor A = start + td + T fs, with td the td_samples of
meta.json. The window is [A, A + 0.7 s) for T up to 1 s and
[A - 0.7 s, A) for a later T, one CSS period. conv_50ms reads
[start, A] at T = 0.05 s. The ERLE is the combined loss less the ERL of the
phase.

The figures of each row, for the phase of --phase (default II):

- loss at 0.1, 0.25, 0.5, 1, 2, 5 and 10 s, and the ERLE at 1 s;
- for phase I: conv_50ms, conv_1s and conv_10s;
- the budget of the first 100 frames from the frame that holds the phase
  start: active frames, frames held by the detector (active, no adapt
  before the filter pass), frames rolled back, adapted frames, and the
  adapted frames that a step scale throttles (step_scale or talk_scale
  under 0.99), with the mean of each scale over the adapted frames;
- the first trial after the phase start: its arm, its start in seconds and
  in active frames after the phase start, and its end in seconds;
- the excess over the last EXCESS_S seconds of the phase: the energy of the
  output less the line noise (noise.f32) over the energy of the line
  noise, in dB. None (printed -) when the phase is shorter than EXCESS_S,
  so the window never reads the lead or another phase. On a line whose
  noise stands well over the echo that the filter leaves, it reads the
  misadjustment of the update;
- the steady state against Figure 9, as g168.test_2 reads it with the
  processor on: the window is [start + td + STEADY_FROM_S fs, end).
  l_ret is the loudest reading of meter_track in the window, and the peak
  excess is the loudest reading of the clause 6.4.1.2.2 peak level
  (siggen.peak_track) less the Figure 9 bound at L_Rin,act (fig9_bound).
  The last time over the bound is the last reading of meter_track in the
  window over the bound, in seconds after the phase start. A phase that
  ends before the window reads None.

--mean prints one row per build and case stem instead (the case name less
its -s<seed>): the mean over the seeds of each loss and of the ERLE at 1 s,
the least and the most loss at 1 s, the mean budget and the mean start of
the first trial in seconds.

--frames S prints the first S seconds of frames after the phase start of
one run instead: per frame its time, active and window flags, adapt before
and after the rollback, fired, hangover, probing, arm and end, the two
step scales, the ERLE of the tracker, the coefficient energy, the live
ratio of residual over microphone and the coherence readings.
"""
import argparse
import fnmatch
import json
import sys

import numpy as np

import siggen as sg
import work

OFFSETS = (0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0)
CONV = (('conv_50ms', 0.05), ('conv_1s', 1.0), ('conv_10s', 10.0))
WINDOW_S = 0.7
BUDGET_FRAMES = 100
THROTTLE = 0.99
EXCESS_S = 3.0
ARMS = {0: '-', 1: 'latch', 2: 'path', 3: 'stale', 4: 'cohere'}
ENDS = {0: '-', 1: 'kept', 2: 'refused', 3: 'diverged', 4: 'talker'}
STEADY_FROM_S = 1.0
# G.168 Figure 9: the (L_Rin,act, bound) points in dBm0, linear between
# them, no bound outside them.
FIG9 = ((-30.0, -65.0), (-10.0, -65.0), (0.0, -55.0))


def window(start, td, fs, offset):
    """(a, b, inclusive) of the anchor window at offset seconds."""
    anchor = start + td + round(offset * fs)
    width = round(WINDOW_S * fs)
    if offset <= 0.05:
        return start, anchor, True
    if offset <= 1.0:
        return anchor, anchor + width, False
    return anchor - width, anchor, False


def loss_get(idx, level, meta, start, end, offset):
    """The combined loss at offset seconds after start, None when the
    window does not fit before end."""
    a, b, inclusive = window(start, meta['td_samples'], meta['fs'], offset)
    if b > end:
        return None
    sel = (idx >= a) & ((idx <= b) if inclusive else (idx < b))
    if not sel.any():
        return None
    return meta['l_rin_act_dbm0'] - float(level[sel].max())


def budget_get(rows, f0):
    """The budget of the BUDGET_FRAMES frames from frame index f0."""
    blk = rows[f0:f0 + BUDGET_FRAMES]
    act = [r for r in blk if int(r['act'])]
    adapted = [r for r in blk if int(r['adapt'])]
    held = sum(1 for r in act if not int(r['adapt0']))
    rolled = sum(1 for r in blk if int(r['adapt0']) and not int(r['adapt']))
    throttled = sum(1 for r in adapted
                    if float(r['step_scale']) < THROTTLE or
                    float(r['talk_scale']) < THROTTLE)

    def mean(key):
        return float(np.mean([float(r[key]) for r in adapted])) \
            if adapted else 0.0

    return dict(act=len(act), held=held, rolled=rolled, adapted=len(adapted),
                throttled=throttled, step_scale=mean('step_scale'),
                talk_scale=mean('talk_scale'))


def trial_get(rows, f0, end_frame):
    """The first trial that starts at or after frame index f0."""
    act = 0
    for i in range(f0, min(end_frame, len(rows))):
        r = rows[i]
        act += int(r['act'])
        if int(r['arm']):
            out = dict(arm=ARMS[int(r['arm'])], t=(i - f0) / 100.0,
                       act=act, end='-', end_t=None)
            for j in range(i + 1, len(rows)):
                if int(rows[j]['end']):
                    out['end'] = ENDS[int(rows[j]['end'])]
                    out['end_t'] = (j - f0) / 100.0
                    break
            return out
    return None


def excess_get(out, noise, start, end, fs):
    """The excess in dB over the EXCESS_S seconds before end, None when
    that window starts before start, the phase start, or when the line
    noise is silent."""
    a = end - round(EXCESS_S * fs)
    if a < start:
        return None
    n = noise[a:end]
    r = out[a:end] - n
    if not len(n) or n @ n <= 0.0:
        return None
    return 10 * np.log10(max(r @ r, 1e-9) / (n @ n))


def fig9_bound(l_rin_act):
    """The Figure 9 bound on the returned echo at l_rin_act in dBm0, None
    outside the range of the figure."""
    xs, ys = zip(*FIG9)
    if not xs[0] <= l_rin_act <= xs[-1]:
        return None
    return float(np.interp(l_rin_act, xs, ys))


def steady_get(tracks, meta, start, end):
    """The steady state of a phase against Figure 9: l_ret, the peak
    excess, the bound and the last time over the bound. tracks holds
    (idx, level) of meter_track and of peak_track."""
    (idx, level), (pidx, peak) = tracks
    a = start + meta['td_samples'] + round(STEADY_FROM_S * meta['fs'])
    bound = fig9_bound(meta['l_rin_act_dbm0'])
    sel = (idx >= a) & (idx < end)
    psel = (pidx >= a) & (pidx < end)
    if not sel.any() or not psel.any():
        return None
    over = sel & (level > bound) if bound is not None else sel & False
    last = float(idx[over].max() - start) / meta['fs'] if over.any() \
        else None
    return dict(l_ret=float(level[sel].max()),
                excess=None if bound is None else
                float(peak[psel].max()) - bound,
                bound=bound, last_over_s=last)


def summary(d):
    """The summary dict of a run directory of a conv case."""
    with open(f'{d}/meta.json') as f:
        meta = json.load(f)
    rows = work.csv_rows(f'{d}/frames.csv')
    out = np.fromfile(f'{d}/out.s16', '<i2').astype(float)
    noise = np.fromfile(f'{d}/noise.f32', '<f4').astype(float)[:len(out)]
    fs = meta['fs']
    frame = fs // 100
    idx, level = sg.meter_track(out, fs)
    peak = sg.peak_track(out, fs)
    res = dict(meta=meta, phases=[])
    for k, p in enumerate(meta['phases']):
        start, end = p['start_sample'], min(p['end_sample'], len(out))
        f0 = start // frame
        ph = dict(name=p['name'], model=p['model'], erl_db=p['erl_db'],
                  start_frame=f0,
                  loss={str(t): loss_get(idx, level, meta, start, end, t)
                        for t in OFFSETS})
        if k == 0:
            ph['conv'] = {name: loss_get(idx, level, meta, start, end, t)
                          for name, t in CONV}
        ph['budget'] = budget_get(rows, f0)
        ph['trial'] = trial_get(rows, f0, end // frame)
        ph['excess_db'] = excess_get(out, noise, start, end, fs)
        ph['steady'] = steady_get(((idx, level), peak), meta, start, end)
        res['phases'].append(ph)
    return res


def fmt(v, digits=1):
    return '-' if v is None else f'{v:.{digits}f}'


def steady_fmt(st):
    """The Figure 9 cell: l_ret/peak excess, and the last time over the
    bound."""
    if not st:
        return '-'
    last = '' if st['last_over_s'] is None else \
        ' over to %.2f s' % st['last_over_s']
    return '%s/%s%s' % (fmt(st['l_ret']), fmt(st['excess']), last)


def row_get(build, case, r, phase):
    ph = next((p for p in r['phases'] if p['name'] == phase), None)
    if ph is None:
        return f'| {build} {case} | no phase {phase} |'
    loss = ' '.join(fmt(ph['loss'][str(t)]) for t in OFFSETS)
    one = ph['loss']['1.0']
    erle = fmt(None if one is None else one - ph['erl_db'])
    b = ph['budget']
    tr = ph['trial']
    trial = '-' if tr is None else '%s@%.2f s (%d act) %s@%s' % (
        tr['arm'], tr['t'], tr['act'], tr['end'], fmt(tr['end_t'], 2))
    conv = ''
    if 'conv' in ph:
        conv = ' conv ' + '/'.join(fmt(v) for v in ph['conv'].values())
    return ('| %s %s | %s/%s %g dB | %s%s | %s | %d/%d/%d/%d | %d, %.2f x '
            '%.2f | %s | %s | %s |' % (build, case, ph['name'], ph['model'],
                                       ph['erl_db'], loss, conv, erle,
                                       b['act'], b['held'], b['rolled'],
                                       b['adapted'], b['throttled'],
                                       b['step_scale'], b['talk_scale'],
                                       trial, fmt(ph['excess_db']),
                                       steady_fmt(ph.get('steady'))))


HEAD = ('| run | phase | loss at 0.1/0.25/0.5/1/2/5/10 s (dB) | ERLE 1 s | '
        'active/held/rolled back/adapted, 0-1 s | throttled, mean step x talk '
        'scale | first trial | excess dB | Fig. 9 l_ret/peak excess |')


MEAN_HEAD = ('| build | stem | seeds | phase | mean loss at 0.1/0.25/0.5/1/2/'
             '5/10 s (dB) | loss at 1 s, least/most | ERLE 1 s | active/'
             'held/rolled back/adapted, 0-1 s | first trial at (s) | '
             'excess dB | Fig. 9 l_ret/peak excess, mean (most) |')


def cols_get(head):
    """The count of columns of a table head."""
    return head.count('|') - 1


def mean_rows(build, items, phase):
    """The --mean rows of one build: items holds (case, summary)."""
    stems = {}
    for case, r in items:
        stem, _, seed = case.rpartition('-s')
        stems.setdefault(stem if seed.isdigit() else case, []).append(r)
    out = []
    for stem, rs in sorted(stems.items()):
        ph = [next(p for p in r['phases'] if p['name'] == phase) for r in rs]

        def mean(vals):
            vals = [v for v in vals if v is not None]
            return sum(vals) / len(vals) if vals else None

        loss = ' '.join(fmt(mean([p['loss'][str(t)] for p in ph]))
                        for t in OFFSETS)
        one = [p['loss']['1.0'] for p in ph if p['loss']['1.0'] is not None]
        erle = mean([p['loss']['1.0'] - p['erl_db'] for p in ph
                     if p['loss']['1.0'] is not None])
        bud = '/'.join('%.0f' % mean([p['budget'][k] for p in ph])
                       for k in ('act', 'held', 'rolled', 'adapted'))
        trial = mean([p['trial']['t'] for p in ph if p['trial']])
        st = [p.get('steady') for p in ph if p.get('steady')]
        ret = [s['l_ret'] for s in st]
        exc = [s['excess'] for s in st if s['excess'] is not None]
        steady = '%s/%s (%s/%s)' % (
            fmt(mean(ret)), fmt(mean(exc)), fmt(max(ret) if ret else None),
            fmt(max(exc) if exc else None)) if st else '-'
        out.append('| %s | %s | %d | %s/%s | %s | %s/%s | %s | %s | %s | %s '
                   '| %s |'
                   % (build, stem, len(rs), phase, ph[0]['model'], loss,
                      fmt(min(one) if one else None),
                      fmt(max(one) if one else None), fmt(erle), bud,
                      fmt(trial, 2), fmt(mean([p['excess_db'] for p in ph])),
                      steady))
    return out


def runs_get(set_name, build, pattern):
    """(case, summary) of each run directory of a set that matches
    pattern, by name."""
    top = work.run_dir(set_name, build)
    return [(d.name, summary(d))
            for d in (sorted(top.iterdir()) if top.is_dir() else [])
            if fnmatch.fnmatch(d.name, pattern) and
            (d / 'frames.csv').exists()]


def saved_get(tag, build, pattern):
    """(case, summary) of each saved summary of a tag that matches
    pattern, by name. A summary of another kind (analyze.py) exits with
    the name of its reader."""
    items = work.summaries(tag, build, pattern)
    other = [case for case, r in items if not summary_is(r)]
    if other:
        sys.exit(f'convtab.py: sum/{tag}/{build}/{other[0]}.json is not a '
                 'conv summary; read it with report_tab')
    return items


def summary_is(r):
    """True for a summary of this module."""
    return isinstance(r, dict) and 'phases' in r and 'meta' in r


def frames_print(d, phase, seconds):
    r = summary(d)
    ph = next(p for p in r['phases'] if p['name'] == phase)
    rows = work.csv_rows(f'{d}/frames.csv')
    f0 = ph['start_frame']
    print('t act wact adapt0 adapt fired hang probing arm end step talk erle '
          'coef live_ratio ncc share')
    for i in range(f0, min(len(rows), f0 + int(seconds * 100))):
        q = rows[i]
        mic = float(q['live_mic'])
        ratio = float(q['live_res']) / mic if mic > 0 else float('nan')
        print('%.2f %s %s %s %s %s %s %s %s %s %.3f %.3f %.1f %.4g %.3f %s %s'
              % ((i - f0) / 100.0, q['act'], q['wact'], q['adapt0'],
                 q['adapt'], q['fired'], q['hang'], q['probing'], q['arm'],
                 q['end'], float(q['step_scale']), float(q['talk_scale']),
                 float(q['erle']), float(q['coef']), ratio, q['ncc'],
                 q['share']))


def main(argv):
    p = argparse.ArgumentParser(prog='convtab.py')
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('glob', nargs='?', default='*')
    p.add_argument('--frames', type=float)
    p.add_argument('--phase', default='II')
    p.add_argument('--mean', action='store_true')
    p.add_argument('--summary', action='store_true')
    a = p.parse_args(argv)
    if a.frames is not None and a.summary:
        p.error('--frames reads run directories; it does not take --summary')
    if a.frames is not None:
        for b in a.builds:
            for d in sorted(work.run_dir(a.set, b).glob(a.glob)):
                print(f'# {b} {d.name}')
                frames_print(d, a.phase, a.frames)
        return 0
    print(MEAN_HEAD if a.mean else HEAD)
    print('|' + '---|' * cols_get(MEAN_HEAD if a.mean else HEAD))
    for b in a.builds:
        items = saved_get(a.set, b, a.glob) if a.summary else \
            runs_get(a.set, b, a.glob)
        if a.mean:
            print('\n'.join(mean_rows(b, items, a.phase)))
            continue
        for case, r in items:
            print(row_get(b, case, r, a.phase))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
