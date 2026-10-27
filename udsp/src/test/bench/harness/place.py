#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""A placement case of a bench Test 2C(c) run: the echo and the Hoth noise
of the rig placed apart, at 16 kHz.

usage: place.py <case> <id> <E> <H> <phase> <build> [<build> ...]

<case> is the 2C(c) case directory of a G.168 bench run (benchstim.py names
its files), and <id> names the run in the outputs. The reference is the
wire Rin less 9 dB, raised to 16 kHz. The FXO transmit signal splits into
its echo (test_2c_c_tx_echo.wav) and its Hoth noise (the rest). The echo
lags the reference so that its peak falls on tap E. The Hoth noise moves
by the lag that a whole-microphone shift for tap H gives it. The lag of
the echo against Rin is the strongest correlation over the first 8 s of
the stimulus. White line noise at -65 dBm0 joins the Hoth noise, and phase
samples of it come first.

The program runs lec_hz of each build on the case with the reference loss
of 18 half-dB and the processor off. It writes the summary (analyze.py) to
sum/place/<build>/pl-<id>-<E>-<H>-s<phase>.json in the work directory, and
removes the stimulus. The 16 kHz signals of a run stay in benchsig/<id>.npz
for the next case of that run, with the path of the case. A cache entry of
another case stops the program with status 1. hoth_frame of meta.json is
the first frame whose Hoth noise stands over -55 dBm0.
"""
import json
import pathlib
import shutil
import sys

import numpy as np

import analyze
import benchstim
import siggen as sg
import work

FS = 16000
LEAD_S = 1.97
REF_LOSS_DB = 9.0
LINE_DBM0 = -65.0
HOTH_ON_DBM0 = -55.0
LAG_SPAN = FS * 8
LAG_REACH = 2600


def signals_get(case, rid):
    """(rin, hoth, echo, lag, stimulus index, FXO gain) at 16 kHz. The cache
    entry names its case; an entry of another case stops the program with
    status 1."""
    case = str(pathlib.Path(case).resolve())
    cache = work.work_dir() / 'benchsig' / f'{rid}.npz'
    if cache.exists():
        z = np.load(cache)
        held = str(z['case']) if 'case' in z.files else 'no case'
        if held != case:
            sys.exit(f'place.py: {cache} holds the signals of {held}, not of '
                     f'{case}: give each run its own id')
        return (z['rin'], z['hoth'], z['te'], int(z['lag']), int(z['st']),
                float(z['lup']))
    rin, tx, te, lup, st = benchstim.case_read(case)
    n = min(len(rin), len(tx), len(te))
    rin, tx, te = rin[:n], tx[:n], te[:n]
    rin16, hoth16, te16 = (sg.upsample2(x) for x in (rin, tx - te, te))
    a = 2 * st
    c = sg.correlate_valid(te16[a:a + LAG_SPAN + LAG_REACH],
                           rin16[a:a + LAG_SPAN])
    lag16 = int(np.argmax(np.abs(c)))
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, rin=rin16, hoth=hoth16, te=te16, lag=lag16, st=2 * st,
             lup=lup, case=case)
    return rin16, hoth16, te16, lag16, 2 * st, lup


def build(case, rid, e, h, phase, out):
    """Write the stimulus of one placement case into out."""
    rin16, hoth16, te16, lag16, st16, lup = signals_get(case, rid)
    se, sh = lag16 - e, lag16 - h
    if min(se, sh) < 0:
        raise ValueError(f'tap {max(e, h)} lies past the echo lag {lag16}')
    lead = int(round(LEAD_S * FS)) - st16
    n = len(rin16) - max(se, sh) - 10
    ref = np.concatenate((np.zeros(lead),
                          rin16[:n] * 10 ** (-REF_LOSS_DB / 20)))
    echo = np.concatenate((np.zeros(lead), te16[se:se + n] * lup))
    hoth = np.concatenate((np.zeros(lead), hoth16[sh:sh + n] * lup))
    rng = np.random.default_rng([phase, 3])
    line = rng.standard_normal(phase + len(ref)) * sg.dbm0_to_rms(LINE_DBM0)
    pad = np.zeros(phase)
    ref = np.concatenate((pad, ref))
    echo = np.concatenate((pad, echo))
    hoth = np.concatenate((pad, hoth))
    frame = FS // 100
    tot = (len(ref) // frame) * frame
    ref, echo, hoth, line = ref[:tot], echo[:tot], hoth[:tot], line[:tot]
    noise = hoth + line
    out.mkdir(parents=True, exist_ok=True)
    sg.to_int16(ref).tofile(out / 'ref.s16')
    sg.to_int16(echo + noise).tofile(out / 'mic.s16')
    echo.astype('<f4').tofile(out / 'echo.f32')
    noise.astype('<f4').tofile(out / 'noise.f32')
    t0 = phase + lead + st16
    hp = np.array([np.mean(hoth[i:i + frame] ** 2)
                   for i in range(0, tot, frame)])
    on = sg.dbm0_to_rms(HOTH_ON_DBM0) ** 2
    meta = dict(fs=FS, seed=phase, peak=e, hoth_place=h, phase=phase,
                lag16=lag16, t0_sample=t0, t0_frame=t0 / frame,
                frames=tot // frame, hoth_frame=int(np.argmax(hp > on)),
                ref_loss_db=REF_LOSS_DB, case=str(case), kind='bench',
                nlp=0)
    (out / 'meta.json').write_text(json.dumps(meta))


def main(argv):
    if len(argv) < 6:
        sys.exit('usage: place.py <case> <id> <E> <H> <phase> <build> '
                 '[<build> ...]')
    case, rid = argv[0], argv[1]
    e, h, phase = int(argv[2]), int(argv[3]), int(argv[4])
    name = f'pl-{rid}-{e}-{h}-s{phase}'
    o = work.work_dir() / 'tmp' / 'place' / name
    build(case, rid, e, h, phase, o)
    rc = 0
    for b in argv[5:]:
        ob = o / b
        if work.hz_run(b, o, ob):
            rc = 1
            continue
        d = work.sum_dir('place', b)
        d.mkdir(parents=True, exist_ok=True)
        (d / f'{name}.json').write_text(json.dumps(analyze.summary(ob)))
    shutil.rmtree(o)
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
