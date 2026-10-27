#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""G.168 Test 9A on a cn run: the comfort noise against the near noise that
it replaces, as g168.test_9a of the G.168 bench reads it, and the
parts of its error.

usage: cntab.py <set> <build>[,<build>...] [<glob>] [--mean] [--summary]

The rows come from run/<set>/<build>/<case>/ of the work directory, for the
cases whose name matches glob (default *), one row per cn phase. With
--summary, <set> names a summary tag, and the rows come from
sum/<set>/<build>/<case>.json: the summaries that `lec_harness.py run
--summary <set>` saved with this module (summary).

The wire plane is the plane that the evaluator reads. At 16 kHz the output
passes siggen.uplink_down2, the decimator that takes the uplink of a 16 kHz
line to the 8 kHz codec of the bench. At 8 kHz it is the output.

Per cn phase, a phase with l_sgen_dbm0 in meta.json:

- l_sgen: L_Sgen,act of the bench, the RMS level of the wire plane over the
  phase before it (the far end silent), from 1 s after its start to its
  end;
- the windows: 700 ms RMS windows of the wire plane, from the cn start plus
  td_samples plus 2 s, whole windows before the cn end;
- err: L_RET - l_sgen of the window with the largest magnitude, the figure
  that the bench checks against +/-2 dB; mean: the power mean of the
  windows less l_sgen.

The parts of the mean error, over the frames that the windows cover:

- near: the RMS level of noise.f32 (the near noise and the line noise) on
  the canceller plane, the level that the comfort noise replaces;
- floor: the mean of the tracked floor (noise_dbov of frames.csv) in dBm0;
- fill: the RMS level of the output over the fill frames, the frames on
  which the processor acts on the frame and on the frame before it; share:
  the fill frames over the window frames;
- wfill: the RMS level of the wire plane over the fill frames;
- bias = floor - near, the tracker's reading under the noise; scale =
  fill - floor, the fill's own level rule; band = wfill - fill, the loss
  of the fill on the way to the wire; bg = l_sgen - near, the same loss
  of the near noise. mean = bias + scale + band - bg, plus the part of the
  frames that are not fill frames.

--mean prints one row per build, case stem (the case name less its
-s<seed>) and phase: the mean over the seeds of each figure, and the err
of the largest magnitude over the seeds.
"""
import argparse
import fnmatch
import json
import math
import sys

import numpy as np

import siggen as sg
import work

WINDOW_S = 0.7
SETTLE_S = 2.0
SKIP_S = 1.0
HEAD = ('| build | case | phase | L_Sgen req/act | err (worst) | mean | '
        'windows | near | floor | fill (share) | wire fill | bias | scale | '
        'band | bg |')
MEAN_HEAD = ('| build | case stem | phase | seeds | L_Sgen act | err (worst) '
             '| mean | bias | scale | band | bg | fill share |')


def level(x):
    """The RMS level of x in dBm0; None for an empty x."""
    if not len(x):
        return None
    rms = math.sqrt(float(np.mean(np.asarray(x, dtype=float) ** 2)))
    return 20 * math.log10(max(rms, 1e-9) / sg.dbm0_to_rms(0.0))


def dbov_dbm0(dbov):
    """A level of udsp_cn_level_get (dsp/cn.c) in dBm0."""
    return 20 * math.log10(32767.0 * 10 ** (-dbov / 20) /
                           sg.dbm0_to_rms(0.0))


def wire_get(out, fs):
    """(wire plane, its rate ratio to the line) of an output."""
    if fs == 16000:
        return sg.uplink_down2(out), 2
    return out, 1


def phase_get(out, wire, ratio, noise, rows, meta, k):
    """The figures of cn phase k of meta (see the module docstring)."""
    fs = meta['fs']
    frame = fs // 100
    ph, quiet = meta['phases'][k], meta['phases'][k - 1]
    fw = fs // ratio
    qa = (quiet['start_sample'] + round(SKIP_S * fs)) // ratio
    qb = min(quiet['end_sample'], len(out)) // ratio
    l_sgen = level(wire[qa:qb])
    width = round(WINDOW_S * fw)
    a = (ph['start_sample'] + meta['td_samples']) // ratio + \
        round(SETTLE_S * fw)
    count = max(0, (min(ph['end_sample'], len(out)) // ratio - a) // width)
    errs = [level(wire[a + j * width:a + (j + 1) * width]) - l_sgen
            for j in range(count)]
    res = dict(name=ph['name'], l_sgen_req=ph['l_sgen_dbm0'], l_sgen=l_sgen,
               windows=count)
    if not count:
        return res
    b = a + count * width
    res['err'] = max(errs, key=abs)
    res['mean'] = level(wire[a:b]) - l_sgen
    f0, f1 = -(-a * ratio // frame), b * ratio // frame
    fill = [f for f in range(f0, f1)
            if int(rows[f]['nlp_act']) and int(rows[f - 1]['nlp_act'])]
    floors = [dbov_dbm0(float(rows[f]['noise_dbov'])) for f in range(f0, f1)
              if float(rows[f]['noise_dbov']) < 127.0]
    res['near'] = level(noise[a * ratio:b * ratio])
    res['floor'] = float(np.mean(floors)) if floors else None
    res['share'] = len(fill) / max(1, f1 - f0)
    if fill:
        idx = np.concatenate([np.arange(f * frame, (f + 1) * frame)
                              for f in fill])
        res['fill'] = level(out[idx])
        widx = np.unique(idx // ratio)
        res['wfill'] = level(wire[widx[widx < len(wire)]])
    res['bias'] = None if res['floor'] is None else res['floor'] - res['near']
    res['scale'] = None if 'fill' not in res or res['floor'] is None else \
        res['fill'] - res['floor']
    res['band'] = res['wfill'] - res['fill'] if 'fill' in res else None
    res['bg'] = l_sgen - res['near']
    return res


def summary(d):
    """The summary dict of a run directory of a cn case."""
    with open(f'{d}/meta.json') as f:
        meta = json.load(f)
    rows = work.csv_rows(f'{d}/frames.csv')
    out = np.fromfile(f'{d}/out.s16', '<i2').astype(float)
    noise = np.fromfile(f'{d}/noise.f32', '<f4').astype(float)[:len(out)]
    wire, ratio = wire_get(out, meta['fs'])
    cn = [phase_get(out, wire, ratio, noise, rows, meta, k)
          for k, p in enumerate(meta['phases'])
          if k > 0 and p.get('l_sgen_dbm0') is not None]
    return dict(meta=meta, cn=cn)


def summary_is(r):
    """True for a summary of this module."""
    return isinstance(r, dict) and 'cn' in r and 'meta' in r


def fmt(v, digits=2):
    return '-' if v is None else f'{v:.{digits}f}'


def row_get(build, case, p):
    """One table row of a cn phase."""
    return ('| %s | %s | %s | %s/%s | %s | %s | %d | %s | %s | %s (%s) | %s '
            '| %s | %s | %s | %s |' % (
                build, case, p['name'], fmt(p['l_sgen_req'], 1),
                fmt(p['l_sgen']), fmt(p.get('err')), fmt(p.get('mean')),
                p['windows'], fmt(p.get('near')), fmt(p.get('floor')),
                fmt(p.get('fill')), fmt(p.get('share')), fmt(p.get('wfill')),
                fmt(p.get('bias')), fmt(p.get('scale')), fmt(p.get('band')),
                fmt(p.get('bg'))))


def mean_rows(build, items):
    """The --mean rows of one build."""
    groups = {}
    for case, r in items:
        stem = case.rsplit('-s', 1)[0]
        for p in r['cn']:
            groups.setdefault((stem, p['name']), []).append(p)
    out = []
    for (stem, name), ps in sorted(groups.items()):
        def mean(key):
            vals = [p[key] for p in ps if p.get(key) is not None]
            return float(np.mean(vals)) if vals else None
        errs = [p['err'] for p in ps if p.get('err') is not None]
        worst = max(errs, key=abs) if errs else None
        out.append('| %s | %s | %s | %d | %s | %s | %s | %s | %s | %s | %s '
                   '| %s |' % (build, stem, name, len(ps),
                               fmt(mean('l_sgen')), fmt(worst),
                               fmt(mean('mean')), fmt(mean('bias')),
                               fmt(mean('scale')), fmt(mean('band')),
                               fmt(mean('bg')), fmt(mean('share'))))
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
    pattern. A summary of another kind exits with a message."""
    items = work.summaries(tag, build, pattern)
    other = [case for case, r in items if not summary_is(r)]
    if other:
        sys.exit(f'cntab.py: sum/{tag}/{build}/{other[0]}.json is not a cn '
                 'summary; read it with report_tab')
    return items


def main(argv):
    p = argparse.ArgumentParser(prog='cntab.py')
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('glob', nargs='?', default='*')
    p.add_argument('--mean', action='store_true')
    p.add_argument('--summary', action='store_true')
    a = p.parse_args(argv)
    head = MEAN_HEAD if a.mean else HEAD
    print(head)
    print('|' + '---|' * (head.count('|') - 1))
    for b in a.builds:
        items = saved_get(a.set, b, a.glob) if a.summary else \
            runs_get(a.set, b, a.glob)
        if a.mean:
            print('\n'.join(mean_rows(b, items)))
            continue
        for case, r in items:
            for ph in r['cn']:
                print(row_get(b, case, ph))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
