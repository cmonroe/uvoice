#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The summary of one lec_hz run of a 2C(c) or bench stimulus.

usage: analyze.py <run> [--seconds]

<run> is a run directory, or set/build/case under run/ of the work
directory. It holds meta.json, frames.csv, out.s16, echo.f32 and noise.f32.
The summary (summary()) holds:

- erle_trk: the ERLE that the canceller tracks, from frames.csv, at 1, 2,
  4, 6 and 10 s after the first burst (t0 of meta.json);
- erle_true: the true ERLE over the 0.5 s before each mark: the echo energy
  over the energy of the output less the near noise, in dB;
- trials: the frames where a trial starts (arm) or ends (end);
- seconds: one dict per block of 100 frames. The blocks count from the
  frame that holds t0, so block t (an integer, -1 for the block before it)
  covers the frames int(t0_frame) + 100 t to + 99, and block 0 holds the
  first burst. A dict holds active, adapted, rolled back and held frames,
  fired and classic-arm frames, trusted and probing frames, the longest
  runs of the trial arms, the trust, the coefficient energy and its peak
  tap, echo_gain, the floor in dBov and the ERLE at the end of the block,
  and the mean step factors over its active frames;
- rise1: the first rise run of the floor tracker after the Hoth onset: its
  start, readings, the spread of the readings and how it ended (moved the
  floor, expired at its age, cleared at 50 readings, cleared, running);
- floor2, floor6: the floor in dBov at 2 s and 6 s, and floor_rise_t, the
  first time the floor stands 10 dB over its value at t0;
- onsets6, onset_fired6: the frames over the first 6 s whose window turns
  active, and those of them that fired.

The command prints the ERLE marks and the trials, and with --seconds the
per-second dicts.
"""
import json
import math
import sys

import numpy as np

import work

MARKS = (1, 2, 4, 6, 10)
TRUE_SPAN_S = 0.5
RISE_AGE_END = 499
RISE_READINGS = 49
FLOOR_MOVE_DB = 1.0
FLOOR_RISE_DB = 10.0
ONSET_S = 6


def load(d):
    """(meta, rows, out, echo, noise) of a run directory."""
    with open(f'{d}/meta.json') as f:
        meta = json.load(f)
    rows = work.csv_rows(f'{d}/frames.csv')
    out = np.fromfile(f'{d}/out.s16', '<i2').astype(float)
    echo = np.fromfile(f'{d}/echo.f32', '<f4').astype(float)[:len(out)]
    noise = np.fromfile(f'{d}/noise.f32', '<f4').astype(float)[:len(out)]
    return meta, rows, out, echo, noise


def num(r, k):
    return float(r[k])


def true_erle(meta, out, echo, noise, t, span=TRUE_SPAN_S):
    """ERLE in dB over the span seconds before t seconds after t0."""
    fs = meta['fs']
    a = int(meta['t0_sample'] + (t - span) * fs)
    b = int(meta['t0_sample'] + t * fs)
    e = echo[a:b]
    r = out[a:b] - noise[a:b]
    return 10 * math.log10((e @ e) / max(r @ r, 1e-9))


def second_get(blk, t):
    """The per-second dict of the frames blk of block t."""
    act = [r for r in blk if int(r['act'])]
    last = blk[-1]

    def mean(key):
        return float(np.mean([num(r, key) for r in act])) if act else 0.0

    return dict(
        t=t, frames=len(blk), act=len(act),
        adapt=sum(int(r['adapt']) for r in blk),
        rollback=sum(int(r['adapt0']) and not int(r['adapt'])
                     for r in blk),
        held_dt=sum(int(r['act']) and not int(r['adapt0']) for r in blk),
        fired=sum(int(r['fired']) for r in blk),
        classic=sum(int(r['classic_arm']) for r in blk if int(r['wact'])),
        trusted=sum(int(r['trusted']) for r in blk),
        probing=sum(int(r['probing']) for r in blk),
        dt_run_max=max(int(r['dt_run']) for r in blk),
        path_max=max(int(r['path_run']) for r in blk),
        cohere_max=max(int(r['cohere_run']) for r in blk),
        stale_max=max(int(r['stale_run']) for r in blk),
        trust_run=int(last['trust_run']), trust_rule=int(last['trust_rule']),
        coef=num(last, 'coef'), peak=int(last['peak']),
        gain=num(last, 'gain'), floor_dbov=num(last, 'noise_dbov'),
        erle=num(last, 'erle'), step_scale=mean('step_scale'),
        talk_scale=mean('talk_scale'))


def rise_get(rows, meta, t0f):
    """The first rise run of the floor tracker after the Hoth onset."""
    lv = [int(r['level']) for r in rows]
    rn = [int(r['rise_n']) for r in rows]
    fl = [float(r['noise_dbov']) for r in rows]
    first = None
    for i in range(max(1, meta.get('hoth_frame', 0)), len(rows)):
        if rn[i] == 1 and rn[i - 1] == 0 and fl[i] < 127:
            first = i
            break
    if first is None:
        return None
    reads = [lv[first]]
    i = first + 1
    while i < len(rows) and rn[i] >= rn[i - 1] and rn[i] > 0:
        if rn[i] > rn[i - 1]:
            reads.append(lv[i])
        i += 1
    end = 'running'
    if i < len(rows):
        if abs(fl[i] - fl[i - 1]) > FLOOR_MOVE_DB:
            end = 'moved'
        elif int(rows[i - 1]['rise_age']) >= RISE_AGE_END:
            end = 'expired'
        elif rn[i - 1] >= RISE_READINGS:
            end = 'cleared at 50'
        else:
            end = 'cleared'
    return dict(start=int(rows[first]['frame']), t=(first - t0f) / 100.0,
                n=len(reads), first_levels=reads[:4],
                spread=max(reads) - min(reads), end=end,
                end_frame=int(rows[i - 1]['frame']))


def summary(d):
    """The summary dict of a run directory."""
    meta, rows, out, echo, noise = load(d)
    t0f = meta['t0_frame']
    res = dict(meta=meta)
    res['erle_trk'] = {t: num(rows[int(round(t0f + t * 100)) - 1], 'erle')
                       for t in MARKS}
    res['erle_true'] = {t: true_erle(meta, out, echo, noise, t)
                        for t in MARKS}
    res['trials'] = [dict(frame=int(r['frame']),
                          t=(int(r['frame']) - t0f) / 100.0,
                          arm=int(r['arm']), end=int(r['end']),
                          bare=int(r['bare']), coef=num(r, 'coef'))
                     for r in rows if int(r['arm']) or int(r['end'])]
    base = int(t0f)
    first = base - 100 if base >= 100 else base
    res['seconds'] = [second_get(rows[s0:s0 + 100], (s0 - base) // 100)
                      for s0 in range(first, len(rows), 100)]
    res['rise1'] = rise_get(rows, meta, t0f)
    fl = [float(r['noise_dbov']) for r in rows]
    res['floor2'] = fl[min(len(fl) - 1, int(round(t0f + 200)))]
    res['floor6'] = fl[min(len(fl) - 1, int(round(t0f + 600)))]
    res['floor_rise_t'] = next(((i - t0f) / 100.0
                                for i in range(int(t0f), len(fl))
                                if fl[i] < fl[int(t0f)] - FLOOR_RISE_DB),
                               None)
    on = [i for i in range(int(t0f), min(len(rows), int(t0f) + ONSET_S * 100))
          if int(rows[i]['wact']) and not int(rows[i - 1]['wact'])]
    res['onsets6'] = len(on)
    res['onset_fired6'] = sum(int(rows[i]['fired']) for i in on)
    return res


def main(argv):
    if not argv or argv[0].startswith('-'):
        sys.exit('usage: analyze.py <run> [--seconds]')
    r = summary(work.run_arg(argv[0]))
    m = r['meta']
    print('fs %d seed %d peak %d phase %d' % (m['fs'], m['seed'], m['peak'],
                                              m['phase']))
    print('erle trk ' + ' '.join('%d:%.1f' % (t, v)
                                 for t, v in r['erle_trk'].items()))
    print('erle tru ' + ' '.join('%d:%.1f' % (t, v)
                                 for t, v in r['erle_true'].items()))
    for tr in r['trials']:
        print('trial', tr)
    if '--seconds' in argv:
        for c in r['seconds']:
            print(c)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
