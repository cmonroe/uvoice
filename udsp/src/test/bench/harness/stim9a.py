#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Cn cases for lec_hz from the signals of bench Test 9A runs.

usage: stim9a.py <outdir> <case> [<case> ...] [--fs R [R ...]]

A case goes to <outdir>/a-<id>-<fs>, with <id> the first four characters
after the last '-' of its run directory.

<case> is the 9A case directory of a run of the G.168 bench: it holds
audio/test_9a_wire_rin.wav (the far signal sent to the gateway),
audio/test_9a_tx.wav (what the FXO port sent toward the line: the near band
noise and the echo that the rig emulated), audio/test_9a_tx_echo.wav (that
echo alone) and g168/test_9a_schedule.json (calibration.l_up_fxo_db, the FXO
transmit to Sout gain; stimulus_idx, the first far sample; phase_rin, the
Rin samples of each phase; phases, the L_Sgen of each cn phase). All run at
8 kHz. A 9A case has no canceller capture, so the planes come from these
signals as stim bench builds them for Test 2C(c).

The canceller's reference is the wire Rin less 9 dB, the loss of the bench
line ahead of the reference. Its microphone is the FXO transmit signal times
the FXO gain, plus white line noise at -73 dBm0. The echo lags the
reference so that its peak falls on tap peak at the line rate (524 at
16 kHz, 262 at 8 kHz): the lag is the strongest correlation of the echo
with Rin over the first 8 s of the far signal. At 16 kHz both streams are
raised from the 8 kHz wire; an odd peak takes one more sample of lag.

Files: ref.s16, mic.s16, echo.f32, noise.f32 (the microphone less the echo)
and meta.json, with kind cn, nlp 1 and the phases of the schedule.
"""
import argparse
import json
import os
import sys
import wave

import numpy as np

import siggen as sg
from stimcap import run_id
from work import names_check

FS_WIRE = 8000
REF_LOSS_DB = 9.0
LINE_DBM0 = -73.0
LAG_MAX = 1200
LAG_SPAN_S = 8
PEAK_DEFAULT = {8000: 262, 16000: 524}


def wav_read(path):
    """Samples of a mono 16-bit WAV file at 8 kHz as float64."""
    with wave.open(path) as w:
        if w.getframerate() != FS_WIRE:
            raise ValueError(f'{path}: rate {w.getframerate()}')
        return np.frombuffer(w.readframes(w.getnframes()),
                             dtype='<i2').astype(float)


def case_read(case):
    """(rin, tx, tx_echo, schedule) of a 9A case directory."""
    rin = wav_read(f'{case}/audio/test_9a_wire_rin.wav')
    tx = wav_read(f'{case}/audio/test_9a_tx.wav')
    te = wav_read(f'{case}/audio/test_9a_tx_echo.wav')
    with open(f'{case}/g168/test_9a_schedule.json') as f:
        sc = json.load(f)
    return rin, tx, te, sc


def build(case, out, fs, peak=None):
    """Write one case into out and return its meta dict."""
    rin, tx, te, sc = case_read(case)
    peak = PEAK_DEFAULT[fs] if peak is None else peak
    lup = 10 ** (sc['calibration']['l_up_fxo_db'] / 20)
    st = sc['stimulus_idx']
    span = FS_WIRE * LAG_SPAN_S
    lag = max(range(0, LAG_MAX), key=lambda k: abs(np.dot(
        rin[st:st + span], te[st + k:st + k + span])))
    shift8 = lag - peak * FS_WIRE // fs
    n = min(len(rin), len(tx) - shift8)
    ref8 = rin[:n] * 10 ** (-REF_LOSS_DB / 20)
    mic8 = tx[shift8:shift8 + n] * lup
    echo8 = te[shift8:shift8 + n] * lup
    up = fs // FS_WIRE
    if up == 2:
        ref, mic, echo = (sg.upsample2(x) for x in (ref8, mic8, echo8))
        if peak % 2:
            mic = np.concatenate((np.zeros(1), mic))[:len(mic)]
            echo = np.concatenate((np.zeros(1), echo))[:len(echo)]
    else:
        ref, mic, echo = ref8, mic8, echo8
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
    model = sg.hybrid_model('m1', 12.0, fs)
    delay = peak - int(np.argmax(np.abs(model)))
    l_sgen = {p['name']: p['l_sgen_dbm0'] for p in sc['phases']}
    phases = [dict(name=name, start_sample=a * up,
                   end_sample=min(tot, b * up), l_sgen_dbm0=l_sgen.get(name))
              for name, (a, b) in sc['phase_rin'].items()]
    meta = dict(fs=fs, peak=peak, lag=lag, delay=delay, td_samples=delay,
                t0_sample=st * up, t0_frame=st * up / frame,
                frames=tot // frame, ref_loss_db=REF_LOSS_DB,
                line_dbm0=LINE_DBM0, case=os.path.abspath(case),
                phases=phases, kind='cn', near='bench', nlp=1)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stim9a.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('outdir')
    p.add_argument('cases', nargs='+')
    p.add_argument('--fs', type=int, nargs='+', default=[16000])
    a = p.parse_args(argv)
    grid = [('a-%s-%d' % (run_id(case), fs), case, fs) for case in a.cases
            for fs in a.fs]
    names_check([(name, case) for name, case, _ in grid])
    for name, case, fs in grid:
        m = build(case, os.path.join(a.outdir, name), fs)
        print(name, 'lag', m['lag'], 'frames', m['frames'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
