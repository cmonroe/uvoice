#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""dt cases for lec_hz from the canceller captures of bench runs of G.168
Tests 3A and 3B.

usage: stimdtcap.py <outdir> <case> [<case> ...] [--td N] [--taps N]

<case> is a case directory of a G.168 bench run whose name carries the test
and its parameters, for example 61.test_3a[l_rin_dbm0=-10] or
61.test_3b[x_db=6]. It holds internal/lec-l0.lecr, the capture of the
canceller of line 0 (lecr.read_lecr). Test 3B plays at -10 dBm0 unless the
name states l_rin_dbm0, and Test 3A at X 15 dB.

The bench can play a case under its named level, for the headroom of the
FXO TX (tx_prescale_db of the G.168 bench). prescale_db is the named
L_Rin,act less the l_rin_req_dbm0 of the case's schedule,
g168/test_3a_schedule.json or g168/test_3b_schedule.json, and 0 without
one. l_rin_act_dbm0 and l_sgen_act_dbm0 of meta.json are the levels that
the bench played: the named levels less prescale_db.

The planes of the RUN records of the capture, in order, give ref.s16 and
mic.s16 of a case, and live.s16, the output of the canceller on the bench.
The far onset is the lag at which the reference best matches the CSS of
seed 0 (stimcap.onset_get). The phases follow stimdt.py from that onset:
for Test 3B a converge phase of 16 CSS periods, then dt for 16 periods and
near_probe for stimdt.PROBE_PERIODS periods. td_samples is --td, the pure
delay of the bench path at the line rate (stimcap.TD_DEFAULT).

The near end of the microphone comes from a least-squares fit of the echo
path over the phases from the far onset to the end of near_probe: --taps
taps (default TAPS, the 64 ms tail at 16 kHz) from the reference to the
microphone, on the normal equations of the two planes. echo.f32 holds the
fitted echo, and near.f32 the microphone less that echo. noise.f32 holds
zeros. A run then reads the filter output less near.f32: the fitted echo
less the estimate.
The near end is independent of the reference, so the fit error stands
about taps over the fitted samples under the near level (FIT_RIDGE adds a
small ridge to the diagonal). meta.json holds kind dt, nlp 0, the capture,
the correlation of the far match and the peak tap of the fitted path.
A case goes to <outdir>/<id>-<test>-l<level>-x<x>, with id the first four
characters after the last '-' of the run directory. Two cases that give one
name stop the program with status 1 before it writes a case.
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
import stimcap
import stimconv
import stimdt
from work import names_check

TAPS = 1024
FIT_RIDGE = 1e-6
CASE_RE = re.compile(r'test_(3a|3b)\[([^\]]*)\]')


def case_params(case):
    """(test, level, x) of the name of a G.168 bench case directory."""
    m = CASE_RE.search(pathlib.Path(case).name)
    if not m:
        raise SystemExit(f'stimdtcap.py: {case}: not a case of Test 3A or '
                         '3B')
    params = dict(kv.split('=', 1) for kv in m.group(2).split(',') if kv)
    x = stimdt.X_3A_DB if m.group(1) == '3a' else float(params['x_db'])
    return m.group(1), float(params.get('l_rin_dbm0', -10.0)), x


def prescale_get(case, test, level):
    """The dB under its named level at which the bench played case."""
    path = pathlib.Path(case) / 'g168' / f'test_{test}_schedule.json'
    if not path.exists():
        return 0.0
    req = json.loads(path.read_text()).get('l_rin_req_dbm0')
    if req is None:
        return 0.0
    return max(0.0, round(level + stimconv.L_ACT_DB - req, 6))


def path_fit(ref, mic, a, b, taps):
    """The least-squares FIR of taps taps from ref to mic over [a, b)."""
    x = ref[a:b]
    y = mic[a:b]
    size = 1 << (2 * len(x)).bit_length()
    fx = np.fft.rfft(x, size)
    rxx = np.fft.irfft(fx * np.conj(fx), size)[:taps]
    pxy = np.fft.irfft(np.fft.rfft(y, size) * np.conj(fx), size)[:taps]
    idx = np.abs(np.arange(taps)[:, None] - np.arange(taps)[None, :])
    return np.linalg.solve(rxx[idx] + FIT_RIDGE * rxx[0] * np.eye(taps), pxy)


def build(case, out, td=None, taps=TAPS):
    """Write one case into out and return its meta dict."""
    test, level, x_db = case_params(case)
    rate, frame, loss, recs = lecr.read_lecr(
        f'{case}/internal/lec-l0.lecr')
    runs = [r for r in recs if r['kind'] == lecr.KIND_RUN]
    ref = np.concatenate([r['ref'] for r in runs])
    mic = np.concatenate([r['mic'] for r in runs])
    live = np.concatenate([r['out'] for r in runs])
    far = sg.css_single(level, 2 * sg.css_period_get(8000) / 8000, 8000, 0)
    if rate != 8000:
        far = sg.upsample2(far)
    t0, corr = stimcap.onset_get(ref, far * 10 ** (-loss / 40))
    span = 16 * sg.css_period_get(rate)
    plan = stimdt.phases_get('b' if test == '3b' else 'a', t0, span,
                             stimdt.PROBE_PERIODS * sg.css_period_get(rate))
    a, b = plan[0][1], min(plan[-1][2], len(ref))
    h = path_fit(ref, mic, a, b, taps)
    echo = sg.fir(h, ref)
    prescale = prescale_get(case, test, level)
    l_rin_act = level + stimconv.L_ACT_DB - prescale
    os.makedirs(out, exist_ok=True)
    sg.to_int16(ref).tofile(f'{out}/ref.s16')
    sg.to_int16(mic).tofile(f'{out}/mic.s16')
    sg.to_int16(live).tofile(f'{out}/live.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    (mic - echo).astype('<f4').tofile(f'{out}/near.f32')
    np.zeros(len(ref), '<f4').tofile(f'{out}/noise.f32')
    meta = dict(fs=rate, seed=0, level=level, x_db=x_db,
                variant='b' if test == '3b' else 'a',
                td_samples=stimcap.TD_DEFAULT[rate] if td is None else td,
                ref_loss_db=loss / 2, t0_sample=t0, t0_frame=t0 / frame,
                frames=len(runs), l_rin_act_dbm0=l_rin_act,
                l_sgen_act_dbm0=l_rin_act - x_db, prescale_db=prescale,
                phases=[dict(name=n, start_sample=s, end_sample=e)
                        for n, s, e in plan],
                kind='dt', nlp=0, capture=str(case), correlation=corr,
                fit_taps=taps, fit_peak=int(np.argmax(np.abs(h))))
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stimdtcap.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('outdir')
    p.add_argument('cases', nargs='+')
    p.add_argument('--td', type=int)
    p.add_argument('--taps', type=int, default=TAPS)
    a = p.parse_args(argv)
    grid = []
    for case in a.cases:
        test, level, x = case_params(case)
        grid.append(('%s-%s-l%g-x%g' % (stimcap.run_id(case), test, level,
                                        x), case))
    names_check(grid)
    for name, case in grid:
        m = build(case, os.path.join(a.outdir, name), a.td, a.taps)
        print(name, 't0_frame', '%.2f' % m['t0_frame'], 'correlation',
              '%.4f' % m['correlation'], 'fit peak', m['fit_peak'],
              'prescale', '%.2f' % m['prescale_db'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
