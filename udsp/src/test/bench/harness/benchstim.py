#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""A stimulus for lec_hz from the signals of a bench Test 2C(c) run.

usage: benchstim.py <case> <out> <fs> <peak> <phase>

<case> is the 2C(c) case directory of a run of the G.168 bench: it holds
audio/test_2c_c_wire_rin.wav (the far signal sent to the gateway),
audio/test_2c_c_tx.wav (what the FXO port sent toward the line: the Hoth
noise and the echo that the rig emulated), audio/test_2c_c_tx_echo.wav (that
echo alone) and g168/test_2c_c_schedule.json (calibration.l_up_fxo_db, the
FXO transmit to Sout gain, and stimulus_idx, the first far sample). All run
at 8 kHz.

The canceller's reference is the wire Rin less 9 dB, the loss of the bench
line ahead of the reference. Its microphone is the FXO transmit signal times
the FXO gain, plus white line noise at -65 dBm0. The echo lags the reference
so that its peak falls on tap peak at the line rate: the lag is the strongest
correlation of the echo with Rin over the first 8 s of the stimulus. A lead
of line noise alone makes the instance run 1.97 s before the first burst,
and phase samples of line noise come before the whole stimulus. At 16 kHz
both streams are raised from the 8 kHz wire; an odd peak takes one more
sample of lag.

Files: ref.s16, mic.s16, echo.f32, noise.f32 (the microphone less the echo)
and meta.json, with kind bench and nlp 0.
"""
import json
import os
import sys
import wave

import numpy as np

import siggen as sg

FS_WIRE = 8000
REF_LOSS_DB = 9.0
LINE_DBM0 = -65.0
LEAD_S = 1.97
LAG_MAX = 1200
LAG_SPAN_S = 8


def wav_read(path):
    """Samples of a mono 16-bit WAV file as float64."""
    with wave.open(path) as w:
        return np.frombuffer(w.readframes(w.getnframes()),
                             dtype='<i2').astype(float)


def case_read(case):
    """(rin, tx, tx_echo, l_up, stimulus_idx) of a 2C(c) case directory."""
    rin = wav_read(f'{case}/audio/test_2c_c_wire_rin.wav')
    tx = wav_read(f'{case}/audio/test_2c_c_tx.wav')
    te = wav_read(f'{case}/audio/test_2c_c_tx_echo.wav')
    with open(f'{case}/g168/test_2c_c_schedule.json') as f:
        sc = json.load(f)
    return (rin, tx, te, 10 ** (sc['calibration']['l_up_fxo_db'] / 20),
            sc['stimulus_idx'])


def build(case, out, fs, peak, phase):
    """Write one bench case into out and return its meta dict."""
    rin, tx, te, lup, st = case_read(case)
    span = FS_WIRE * LAG_SPAN_S
    lag = max(range(0, LAG_MAX), key=lambda k: abs(np.dot(
        rin[st:st + span], te[st + k:st + k + span])))
    shift8 = lag - peak * FS_WIRE // fs
    n = min(len(rin), len(tx)) - abs(shift8) - 10
    ref8 = rin[:n] * 10 ** (-REF_LOSS_DB / 20)
    mic8 = tx[shift8:shift8 + n] * lup
    echo8 = te[shift8:shift8 + n] * lup
    lead = int(round(LEAD_S * FS_WIRE)) - st
    ref8 = np.concatenate((np.zeros(lead), ref8))
    mic8 = np.concatenate((np.zeros(lead), mic8))
    echo8 = np.concatenate((np.zeros(lead), echo8))
    if fs == 16000:
        ref, mic, echo = (sg.upsample2(x) for x in (ref8, mic8, echo8))
        if peak % 2:
            mic = np.concatenate((np.zeros(1), mic))[:len(mic)]
            echo = np.concatenate((np.zeros(1), echo))[:len(echo)]
    else:
        ref, mic, echo = ref8, mic8, echo8
    rng = np.random.default_rng([phase, 3])
    line_rms = sg.dbm0_to_rms(LINE_DBM0)
    line = rng.standard_normal(len(ref)) * line_rms
    pad = np.zeros(phase)
    ref = np.concatenate((pad, ref))
    mic = np.concatenate((pad, mic))
    echo = np.concatenate((pad, echo))
    line = np.concatenate((rng.standard_normal(phase) * line_rms, line))
    frame = fs // 100
    tot = (len(ref) // frame) * frame
    ref, mic, echo, line = ref[:tot], mic[:tot], echo[:tot], line[:tot]
    os.makedirs(out, exist_ok=True)
    sg.to_int16(ref).tofile(f'{out}/ref.s16')
    sg.to_int16(mic + line).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    (mic - echo + line).astype('<f4').tofile(f'{out}/noise.f32')
    t0 = phase + (lead + st) * (fs // FS_WIRE)
    meta = dict(fs=fs, seed=phase, peak=peak, phase=phase, lag=lag,
                t0_sample=t0, t0_frame=t0 / frame, frames=tot // frame,
                ref_loss_db=REF_LOSS_DB, case=os.path.abspath(case),
                kind='bench', nlp=0)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    if len(argv) != 5:
        sys.exit('usage: benchstim.py <case> <out> <fs> <peak> <phase>')
    m = build(argv[0], argv[1], int(argv[2]), int(argv[3]), int(argv[4]))
    print(argv[1], 'lag', m['lag'], 't0', m['t0_frame'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
