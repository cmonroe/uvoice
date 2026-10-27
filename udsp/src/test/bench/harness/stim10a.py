#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Fax cases for lec_hz from the signals of bench Test 10A and 10B runs.

usage: stim10a.py <outdir> <case> [<case> ...] [--fs R [R ...]] [--clause]

<case> is the 10A or 10B case directory of a G.168 bench run, whose name
ends in test_10a or test_10b. It holds audio/test_<t>_wire_rin.wav (the far
signal sent to the gateway), audio/test_<t>_rx.wav (FXO receive: the
gateway's playout on the line), audio/test_<t>_tx.wav (what the FXO port
sent toward the line: the echo that the rig emulated, and the CNG of 10A),
audio/test_<t>_tx_echo.wav (that echo alone) and
g168/test_<t>_schedule.json (calibration.l_up_fxo_db, the FXO transmit to
Sout gain; rin_to_rx, the FXO receive index of the first far sample less
its Rin index; stimulus_idx and phase_rin). All run at 8 kHz. A fax case
has no canceller capture, so the planes come from these signals, as stim 9a
builds them.

The canceller's reference is the wire Rin less 9 dB, the loss of the bench
line ahead of the reference. Its microphone is the FXO transmit signal times
the FXO gain, plus white line noise at -73 dBm0. The echo lags the
reference so that its peak falls on tap peak at the line rate (524 at
16 kHz, 262 at 8 kHz). The lag is rin_to_rx plus the lag of the emulated
echo after FXO receive, read over the first second of the last phase: the
rig computes the echo from FXO receive. Reason: CED and V.21 hold one or two
tones, so their correlation with Rin peaks on many lags. At 16 kHz both
streams are raised from the 8 kHz wire; an odd peak takes one more sample of
lag.

With --clause, the microphone is the echo of G.168 Annex D model m1 at
6 dB ERL over the wire Rin, its peak on tap peak, plus the line noise: the
clause's own path, without the gateway playout or the bench line.

A case goes to <outdir>/<f|c>-<id>-<t>-<fs>, f for the bench planes and c
for the clause path, with <id> the first four characters after the last '-'
of its run directory and <t> 10a or 10b. Files: ref.s16, mic.s16, echo.f32,
noise.f32 (the microphone less the echo) and meta.json, with kind fax,
test, nlp 1 and the phases of the schedule.
"""
import argparse
import json
import os
import pathlib
import sys
import wave

import numpy as np

import siggen as sg
from stimcap import run_id
from work import names_check

FS_WIRE = 8000
REF_LOSS_DB = 9.0
LINE_DBM0 = -73.0
ECHO_ERL_DB = 6.0
LAG_MIN = -400
LAG_MAX = 800
LAG_SPAN_S = 1
PEAK_DEFAULT = {8000: 262, 16000: 524}


def wav_read(path):
    """Samples of a mono 16-bit WAV file at 8 kHz as float64."""
    with wave.open(path) as w:
        if w.getframerate() != FS_WIRE:
            raise ValueError(f'{path}: rate {w.getframerate()}')
        return np.frombuffer(w.readframes(w.getnframes()),
                             dtype='<i2').astype(float)


def test_get(case):
    """10a or 10b, from the name of a case directory."""
    name = pathlib.Path(case).name
    for t in ('10a', '10b'):
        if name.endswith(f'test_{t}'):
            return t
    raise SystemExit(f'stim10a.py: {case}: not a case of Test 10A or 10B')


def case_read(case, t):
    """(rin, rx, tx, tx_echo, schedule) of a fax case directory."""
    rin, rx, tx, te = (wav_read(f'{case}/audio/test_{t}_{n}.wav')
                       for n in ('wire_rin', 'rx', 'tx', 'tx_echo'))
    with open(f'{case}/g168/test_{t}_schedule.json') as f:
        sc = json.load(f)
    return rin, rx, tx, te, sc


def echo_lag_get(rx, te, start):
    """The lag of the emulated echo after FXO receive over LAG_SPAN_S from
    start, an FXO receive index."""
    a, b = start, start + FS_WIRE * LAG_SPAN_S
    return max(range(LAG_MIN, LAG_MAX),
               key=lambda k: abs(np.dot(rx[a:b], te[a + k:b + k])))


def moved(x, shift, n):
    """x read from sample shift on, n samples, zeros outside x."""
    y = np.zeros(n)
    src, dst = max(0, shift), max(0, -shift)
    k = max(0, min(len(x) - src, n - dst))
    y[dst:dst + k] = x[src:src + k]
    return y


def build(case, out, fs, clause=False):
    """Write one case into out and return its meta dict."""
    t = test_get(case)
    rin, rx, tx, te, sc = case_read(case, t)
    peak = PEAK_DEFAULT[fs]
    up = fs // FS_WIRE
    last = sc['phase_rin']['seq1' if t == '10a' else 'seq2'][0]
    lag = sc['rin_to_rx'] + echo_lag_get(rx, te, last + sc['rin_to_rx'])
    n = len(rin)
    if clause:
        x = rin if up == 1 else sg.upsample2(rin)
        model = sg.hybrid_model('m1', ECHO_ERL_DB, fs)
        delay = peak - int(np.argmax(np.abs(model)))
        echo = np.concatenate((np.zeros(delay),
                               sg.fftconvolve(x, model)[:len(x) - delay]))
        ref, mic = x * 10 ** (-REF_LOSS_DB / 20), echo.copy()
    else:
        lup = 10 ** (sc['calibration']['l_up_fxo_db'] / 20)
        shift8 = lag - peak * FS_WIRE // fs
        ref8 = rin * 10 ** (-REF_LOSS_DB / 20)
        mic8 = moved(tx, shift8, n) * lup
        echo8 = moved(te, shift8, n) * lup
        if up == 2:
            ref, mic, echo = (sg.upsample2(v) for v in (ref8, mic8, echo8))
            if peak % 2:
                mic = np.concatenate((np.zeros(1), mic))[:len(mic)]
                echo = np.concatenate((np.zeros(1), echo))[:len(echo)]
        else:
            ref, mic, echo = ref8, mic8, echo8
    st = sc['stimulus_idx']
    rng = np.random.default_rng([st, 13])
    line = rng.standard_normal(len(ref)) * sg.dbm0_to_rms(LINE_DBM0)
    frame = fs // 100
    tot = (len(ref) // frame) * frame
    ref, mic, echo, line = ref[:tot], mic[:tot], echo[:tot], line[:tot]
    os.makedirs(out, exist_ok=True)
    sg.to_int16(ref).tofile(f'{out}/ref.s16')
    sg.to_int16(mic + line).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    (mic - echo + line).astype('<f4').tofile(f'{out}/noise.f32')
    model = sg.hybrid_model('m1', ECHO_ERL_DB, fs)
    delay = peak - int(np.argmax(np.abs(model)))
    phases = [dict(name=name, start_sample=a * up,
                   end_sample=min(tot, b * up))
              for name, (a, b) in sc['phase_rin'].items()]
    meta = dict(fs=fs, peak=peak, lag=lag, delay=delay, td_samples=delay,
                t0_sample=st * up, t0_frame=st * up / frame,
                frames=tot // frame, ref_loss_db=REF_LOSS_DB,
                line_dbm0=LINE_DBM0, case=os.path.abspath(case),
                phases=phases, kind='fax', test=t, clause=clause, nlp=1)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stim10a.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('outdir')
    p.add_argument('cases', nargs='+')
    p.add_argument('--fs', type=int, nargs='+', default=[8000, 16000])
    p.add_argument('--clause', action='store_true')
    a = p.parse_args(argv)
    form = 'c' if a.clause else 'f'
    grid = [('%s-%s-%s-%d' % (form, run_id(case), test_get(case), fs), case,
             fs) for case in a.cases for fs in a.fs]
    names_check([(name, case) for name, case, _ in grid])
    for name, case, fs in grid:
        m = build(case, os.path.join(a.outdir, name), fs, a.clause)
        print(name, 'lag', m['lag'], 'frames', m['frames'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
