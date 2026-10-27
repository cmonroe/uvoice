#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The comfort noise of each replayed capture against the capture's own
quiet stretches.

usage: fldcn.py <set> <build>[:band|:white][,<build>...] [--json FILE]

The rows come from fld/<set>/<build>/<id>.csv of the work directory (the
replay frames) and from the capture file of each id (fld/<set>/index.json).

The background reads the capture planes alone, not the canceller state. A
frame is far-silent when the frame power of the reference plane stays under
the fixed activity bar (LEC_REF_ACTIVE_POWER of dsp/lec.c, less the
reference loss of the capture) on that frame and on the WINDOW frames before
it. A quiet stretch is a run of far-silent RUN records with consecutive
ticks, less its first SKIP_FRAMES. A stretch of STRETCH_MIN frames or more
is steady when its frame levels (dB of the microphone plane) spread at most
SPREAD_DB from p5 to p95: near speech spreads more. The background is the
median over the steady stretches of the power mean of the microphone plane.
On a 16 kHz capture it is read twice: in the full band, and in the band of
the 8 kHz wire (the microphone plane through siggen.uplink_down2).

The replay rows of a build join the RUN records by tick: the replay writes
no row for a RUN record that it does not replay. The fill frames of a build
are the rows on which its processor acts, on the row of the tick before as
well. The fill level is the median of their frame levels (out_power). On the
8 kHz wire of a 16 kHz line the fill level moves by FILL_WIRE_DB of the form
of the fill: band for the noise of 8 kHz raised to 16 kHz (the default,
dsp/lec.c cn_fill), white for white noise at 16 kHz. The rise is the fill
level less the background, in each band.

Per capture: rate, steady stretches, background full and wire, the floor
bias (the median floor of the first build over the frames of the steady
stretches, less the full-band background), and per build the fill frames,
the fill level and its rise. The last lines give each build's median rise
and the largest one.
"""
import argparse
import json
import math
import sys

import numpy as np

import lecr
import siggen as sg
import work

REF_ACTIVE_POWER = 2500.0
WINDOW = {8000: 5, 16000: 7}
SKIP_FRAMES = 5
STRETCH_MIN = 50
SPREAD_DB = 6.0
# The level of the fill on the 8 kHz wire less its level at 16 kHz, by the
# form of the fill (siggen.uplink_down2 on white noise at 16 kHz, and on
# white noise at 8 kHz raised by the interpolator of dsp/resamp.c).
FILL_WIRE_DB = {'band': -0.15, 'white': -3.16}


def dbm0(power):
    """A mean square in int16 units as dBm0."""
    return 10 * math.log10(max(power, 1e-12)) - \
        20 * math.log10(sg.dbm0_to_rms(0.0))


def background_get(path):
    """The quiet stretches and background of one capture (module
    docstring)."""
    rate, n, loss, recs = lecr.read_lecr(path)
    runs = [r for r in recs if r['kind'] == lecr.KIND_RUN]
    bar = REF_ACTIVE_POWER * 10 ** (-(loss / 2) / 10)
    win = WINDOW[rate]
    ref = np.array([np.mean(r['ref'] ** 2) for r in runs])
    mic = np.array([np.mean(r['mic'] ** 2) for r in runs])
    if rate == 16000:
        low = sg.uplink_down2(np.concatenate([r['mic'] for r in runs]))
        wire = np.mean(low.reshape(len(runs), n // 2) ** 2, axis=1)
    else:
        wire = mic
    ticks = np.array([r['tick'] for r in runs])
    quiet = np.zeros(len(runs), bool)
    for i in range(win, len(runs)):
        quiet[i] = ref[i - win:i + 1].max() < bar and \
            ticks[i] - ticks[i - win] == win
    steady = []
    i = 0
    while i < len(runs):
        if not quiet[i]:
            i += 1
            continue
        k = i
        while k + 1 < len(runs) and quiet[k + 1] and \
                ticks[k + 1] == ticks[k] + 1:
            k += 1
        a, b = i + SKIP_FRAMES, k + 1
        if b - a >= STRETCH_MIN:
            lv = 10 * np.log10(np.maximum(mic[a:b], 1e-12))
            if np.percentile(lv, 95) - np.percentile(lv, 5) <= SPREAD_DB:
                steady.append((a, b, dbm0(float(mic[a:b].mean())),
                               dbm0(float(wire[a:b].mean()))))
        i = k + 1
    out = dict(rate=rate, stretches=len(steady),
               ticks=[int(t) for a, b, _, _ in steady for t in ticks[a:b]])
    if steady:
        out['full'] = float(np.median([s[2] for s in steady]))
        out['wire'] = float(np.median([s[3] for s in steady]))
    return out


def fill_get(set_name, build, cid, bg, form):
    """The fill of one build on one capture, and its rise."""
    rows = work.csv_rows(work.fld_dir(set_name, build) / f'{cid}.csv')
    by_tick = {int(r['tick']): r for r in rows}
    fill = [r for r in rows if int(r['nlp_active']) and
            int(by_tick.get(int(r['tick']) - 1, {}).get('nlp_active', 0))]
    res = dict(frames=len(fill))
    floors = [float(by_tick[t]['noise_dbov']) for t in bg['ticks']
              if t in by_tick and float(by_tick[t]['noise_dbov']) < 127.0]
    if floors and 'full' in bg:
        # dsp/cn.c levels are dBov of a full-scale square, a larger number
        # being quieter.
        res['bias'] = dbm0(32767.0 ** 2 * 10 ** (-np.median(floors) / 10)) \
            - bg['full']
    if len(fill) and 'full' in bg:
        level = float(np.median([dbm0(float(r['out_power']))
                                 for r in fill]))
        shift = FILL_WIRE_DB[form] if bg['rate'] == 16000 else 0.0
        res['fill'] = level
        res['rise_full'] = level - bg['full']
        res['rise_wire'] = level + shift - bg['wire']
    return res


def fmt(v):
    return '-' if v is None else '%.2f' % v


def main(argv):
    p = argparse.ArgumentParser(prog='fldcn.py')
    p.add_argument('set')
    p.add_argument('builds')
    p.add_argument('--json')
    a = p.parse_args(argv)
    builds = []
    for item in work.names(a.builds):
        name, _, form = item.partition(':')
        if form and form not in FILL_WIRE_DB:
            sys.exit(f'fldcn.py: unknown fill form {form}')
        builds.append((name, form or 'band'))
    index = lecr.index_read(a.set)
    res = {}
    print('| capture | rate | steady stretches | background full/wire dBm0 '
          '| floor bias | ' + ' | '.join(
              f'{b}: fill frames, fill dBm0, rise full/wire' for b, _ in
              builds) + ' |')
    print('|' + '---|' * (5 + len(builds)))
    for cid in sorted(index):
        bg = background_get(index[cid])
        res[cid] = {b: fill_get(a.set, b, cid, bg, form)
                    for b, form in builds}
        first = res[cid][builds[0][0]]
        cells = ['%d, %s, %s/%s' % (r['frames'], fmt(r.get('fill')),
                                    fmt(r.get('rise_full')),
                                    fmt(r.get('rise_wire')))
                 for r in (res[cid][b] for b, _ in builds)]
        print('| %s | %d | %d | %s/%s | %s | %s |' % (
            cid, bg['rate'], bg['stretches'], fmt(bg.get('full')),
            fmt(bg.get('wire')), fmt(first.get('bias')), ' | '.join(cells)))
    for b, _ in builds:
        rated = [(r[b]['rise_wire'], r[b]['rise_full'], cid)
                 for cid, r in res.items() if 'rise_wire' in r[b]]
        if not rated:
            continue
        wire = sorted(rated)
        full = sorted(rated, key=lambda x: x[1])
        print(f'# {b}: {len(rated)} captures rated; rise on the wire median '
              f'{np.median([x[0] for x in rated]):.2f}, largest '
              f'{wire[-1][0]:.2f} ({wire[-1][2]}); rise in the full band '
              f'median {np.median([x[1] for x in rated]):.2f}, largest '
              f'{full[-1][1]:.2f} ({full[-1][2]})')
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(res, f)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
