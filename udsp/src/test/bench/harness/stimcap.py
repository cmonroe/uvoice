#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Conv cases for lec_hz from the canceller captures of bench runs of G.168
Tests 2A, 2B and 12.

usage: stimcap.py <outdir> <case> [<case> ...] [--td N]

<case> is a case directory of a G.168 bench run whose name carries the test
and its parameters, for example 61.test_2a[l_rin_dbm0=-10,model_b=m4] or
61.test_12[model_b=m7]. It holds internal/lec-l0.lecr, the capture of the
canceller of line 0 (lecr.read_lecr). Test 12 plays at -10 dBm0.

The planes of the RUN records of the capture, in order, give ref.s16 and
mic.s16 of a case, and live.s16, the output of the canceller on the bench.
The far onset is the lag at which the reference best matches the CSS of
seed 0 at the level of the test, less the reference loss of the capture
(siggen.css_single, stimconv.py). The phases follow stimconv.py: 16 CSS
periods each from that onset, m1 at the ERL of the model plus
stimconv.DELTA_ERL_DB, then the model at stimconv.ERL_II_DB, then m1 again
for Test 12. td_samples is --td, the pure delay of the bench path at the
line rate (default TD_DEFAULT: the onset of m1 on the bench line at
16 kHz, 495 samples).

echo.f32 and noise.f32 hold zeros, so convtab.py reads no excess. meta.json
holds kind conv, nlp 1 for Tests 2A and 12 and 0 for Test 2B, the capture
and the correlation of the match. A case goes to <outdir>/<id>-<test>-<model>
-l<level>, with id the first four characters after the last '-' of the run
directory. Two cases that give one name stop the program with status 1
before it writes a case.
"""
import argparse
import json
import os
import pathlib
import re
import sys

import numpy as np

import lecr
import siggen as sg
import stimconv
from work import names_check

TD_DEFAULT = {16000: 495, 8000: 248}
CASE_RE = re.compile(r'test_(2a|2b|12)\[([^\]]*)\]')
NLP = {'2a': 1, '2b': 0, '12': 1}


def case_params(case):
    """(test, model, level) of the name of a G.168 bench case directory."""
    m = CASE_RE.search(pathlib.Path(case).name)
    if not m:
        raise SystemExit(f'stimcap.py: {case}: not a case of Test 2A, 2B '
                         'or 12')
    params = dict(kv.split('=', 1) for kv in m.group(2).split(',') if kv)
    return m.group(1), params['model_b'], float(params.get('l_rin_dbm0',
                                                           -10.0))


def run_id(case):
    """The first four characters after the last '-' of the run directory."""
    for part in reversed(pathlib.Path(case).resolve().parts):
        if '-' in part and part != 'cases' and '[' not in part:
            return part.rsplit('-', 1)[1][:4]
    return pathlib.Path(case).name[:4]


def onset_get(ref, far):
    """(lag, correlation) of the best match of far in ref near its first
    sample over 50 LSB."""
    s0 = int(np.nonzero(np.abs(ref) > 50)[0][0])
    lo = max(0, s0 - 400)
    seg = ref[lo:lo + len(far) + 800]
    c = sg.correlate_valid(seg, far)
    k = int(np.argmax(c))
    hit = seg[k:k + len(far)]
    return lo + k, float(c[k] / np.sqrt(np.dot(far, far) * np.dot(hit, hit)))


def build(case, out, td=None):
    """Write one case into out and return its meta dict."""
    test, model, level = case_params(case)
    rate, frame, loss, recs = lecr.read_lecr(
        f'{case}/internal/lec-l0.lecr')
    runs = [r for r in recs if r['kind'] == lecr.KIND_RUN]
    ref = np.concatenate([r['ref'] for r in runs])
    mic = np.concatenate([r['mic'] for r in runs])
    live = np.concatenate([r['out'] for r in runs])
    far = sg.css_single(level, 2 * sg.css_period_get(8000) / 8000, 8000, 0)
    if rate != 8000:
        far = sg.upsample2(far)
    t0, corr = onset_get(ref, far * 10 ** (-loss / 40))
    erl_ii = stimconv.ERL_II_DB[model]
    erl_i = erl_ii + stimconv.DELTA_ERL_DB
    plan = stimconv.phases_get(model, erl_i, erl_ii, test == '12')
    span = 16 * sg.css_period_get(rate)
    os.makedirs(out, exist_ok=True)
    sg.to_int16(ref).tofile(f'{out}/ref.s16')
    sg.to_int16(mic).tofile(f'{out}/mic.s16')
    sg.to_int16(live).tofile(f'{out}/live.s16')
    np.zeros(len(ref), '<f4').tofile(f'{out}/echo.f32')
    np.zeros(len(ref), '<f4').tofile(f'{out}/noise.f32')
    meta = dict(fs=rate, seed=0, model=model, level=level, erl_i=erl_i,
                erl_ii=erl_ii, three=test == '12',
                td_samples=TD_DEFAULT[rate] if td is None else td,
                ref_loss_db=loss / 2, t0_sample=t0, t0_frame=t0 / frame,
                frames=len(runs),
                l_rin_act_dbm0=level + stimconv.L_ACT_DB,
                phases=[dict(name=name, start_sample=t0 + k * span,
                             end_sample=t0 + (k + 1) * span, model=m,
                             erl_db=erl)
                        for k, (name, m, erl) in enumerate(plan)],
                kind='conv', nlp=NLP[test], capture=str(case),
                correlation=corr)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stimcap.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('outdir')
    p.add_argument('cases', nargs='+')
    p.add_argument('--td', type=int)
    a = p.parse_args(argv)
    grid = []
    for case in a.cases:
        test, model, level = case_params(case)
        grid.append(('%s-%s-%s-l%g' % (run_id(case), test, model, level),
                     case))
    names_check(grid)
    for name, case in grid:
        m = build(case, os.path.join(a.outdir, name), a.td)
        print(name, 't0_frame', '%.2f' % m['t0_frame'], 'correlation',
              '%.4f' % m['correlation'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
