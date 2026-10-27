#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""G.168 Test 16A stimuli for lec_hz: near DTMF digits over a far dial tone.

usage: stim16a.py <outdir> --peak N [N ...] [--fs R [R ...]]
                  [--seeds LIST] [--nseq N] [--ulaw] [--name TEMPLATE]

Each case of the grid fs x peak x seed goes to <outdir>/<name>. LIST is a
comma list of integers and ranges A-B. TEMPLATE is a str.format pattern over
fs, peak and seed; the default is 16a-{fs}-{peak}-s{seed}, and
16au-{fs}-{peak}-s{seed} with --ulaw. Two grid points that give one name
stop the program with status 1 before it writes a case.

The wire plane runs at 8 kHz. After a silent lead of lead_s, a dial tone of
350 and 440 Hz at -10 dBm0 in total plays for 1.4 s. One block per DTMF
level follows, -27 to -6 dBm0 in 3 dB steps, each of nseq trials:

- the far dial tone at -15 or -10 dBm0 in total, by the trial's parity, for
  1.0 s or 2.03 s;
- five near digits, 50 ms on and 100 ms off, from 0.45 s or 2.0 s; the
  digits rotate through 0123456789#*ABCD by the trial's number;
- 0.4 s of silence after the later of the two.

The far end reaches the canceller through m1 of Annex D at ERL 6 dB, with
the peak on tap peak, and the reference stands ref_loss_db under the line.
White line noise at line_dbm0 lies on the microphone. --ulaw gives the far
end a G.711 mu-law round trip first, as a PCMU far end does. At 16 kHz both
ends are 8 kHz signals raised to 16 kHz.

Files of a case: ref.s16 and mic.s16 (int16 little-endian), echo.f32,
noise.f32 (the digits plus the line noise), near.f32 (the digits alone),
digits.json and meta.json. digits.json holds one entry per digit: level,
seq, dial, digit, f_lo, f_hi, on and off (the half-amplitude edges at the
line rate) and overlap (the digit starts under the dial tone). meta.json
holds kind 16a and nlp 1.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

import siggen as sg
from work import int_list, names_check

FS = 8000
ORDER = '0123456789#*ABCD'
LEVELS = tuple(float(v) for v in range(-27, -5, 3))
TIMING = ((1.0, 0.45), (2.03, 2.0))
DIAL = (-15.0, -10.0)
DIAL_HZ = (350.0, 440.0)
WARM_DBM0 = -10.0
WARM_S = 1.4
TAIL_S = 0.4
OVERLAP_S = 0.05
ERL_DB = 6.0


def build(out, fs, seed, peak, nseq=8, ref_loss_db=9.0, line_dbm0=-65.0,
          lead_s=1.97, ulaw=False):
    """Write one 16A case into out and return its meta dict."""
    rng = np.random.default_rng([seed, 16])
    phase = int(rng.integers(0, fs // 100))
    pos = round(lead_s * FS)
    parts_far, parts_near, digits = [np.zeros(pos)], [np.zeros(pos)], []
    each_db = 10 * math.log10(len(DIAL_HZ))
    warm = sg.tone(DIAL_HZ, WARM_DBM0 - each_db, WARM_S, FS)
    parts_far.append(warm)
    parts_near.append(np.zeros(len(warm)))
    pos += len(warm)
    for n, level in enumerate(LEVELS):
        for j in range(nseq):
            tdt, tad = TIMING[j % 2]
            rin = sg.tone(DIAL_HZ, DIAL[j % 2] - each_db, tdt, FS)
            dg = (ORDER[j:] + ORDER[:j])[:5]
            pcm, tones = sg.dtmf(dg, on_ms=50.0, off_ms=100.0,
                                 level_dbm0=level, fs=FS,
                                 seed=100 * n + j + 1000 * seed)
            at = round(tad * FS) - tones[0]['on']
            m = max(len(rin), at + len(pcm)) + round(TAIL_S * FS)
            near = np.zeros(m)
            near[at:at + len(pcm)] = pcm
            far = np.pad(rin, (0, m - len(rin)))
            for t in tones:
                digits.append(dict(
                    level=level, seq=j, dial=DIAL[j % 2], digit=t['digit'],
                    f_lo=t['f_lo'], f_hi=t['f_hi'], on=pos + at + t['on'],
                    off=pos + at + t['off'],
                    overlap=bool(at + t['on'] < len(rin) +
                                 round(OVERLAP_S * FS))))
            parts_far.append(far)
            parts_near.append(near)
            pos += m
    far8 = np.concatenate(parts_far)
    near8 = np.concatenate(parts_near)
    if ulaw:
        far8 = sg.ulaw_decode(sg.ulaw_encode(far8))
    g = sg.hybrid_m1(ERL_DB, fs)
    delay = peak - int(np.argmax(np.abs(g)))
    if fs == FS:
        far, near, scale = far8, near8, 1
    else:
        far, near, scale = sg.upsample2(far8), sg.upsample2(near8), 2
    echo = sg.fir(g, far)
    echo = np.concatenate((np.zeros(delay), echo))[:len(far)]
    line_rms = sg.dbm0_to_rms(line_dbm0)
    line = rng.standard_normal(len(far)) * line_rms
    pad = np.zeros(phase)
    far = np.concatenate((pad, far))
    echo = np.concatenate((pad, echo))
    near = np.concatenate((pad, near))
    line = np.concatenate((rng.standard_normal(phase) * line_rms, line))
    frame = fs // 100
    total = (len(far) // frame) * frame
    far, echo, near, line = far[:total], echo[:total], near[:total], \
        line[:total]
    os.makedirs(out, exist_ok=True)
    sg.to_int16(far * 10 ** (-ref_loss_db / 20)).tofile(f'{out}/ref.s16')
    sg.to_int16(echo + near + line).tofile(f'{out}/mic.s16')
    echo.astype('<f4').tofile(f'{out}/echo.f32')
    (near + line).astype('<f4').tofile(f'{out}/noise.f32')
    near.astype('<f4').tofile(f'{out}/near.f32')
    for d in digits:
        d['on'] = d['on'] * scale + phase
        d['off'] = d['off'] * scale + phase
    with open(f'{out}/digits.json', 'w') as f:
        json.dump(digits, f)
    t0 = phase + round(lead_s * FS) * scale
    meta = dict(fs=fs, seed=seed, peak=peak, delay=delay, phase=phase,
                ref_loss_db=ref_loss_db, t0_sample=t0, t0_frame=t0 / frame,
                frames=total // frame, kind='16a', nseq=nseq, ulaw=ulaw,
                nlp=1)
    with open(f'{out}/meta.json', 'w') as f:
        json.dump(meta, f, indent=1)
    return meta


def main(argv):
    p = argparse.ArgumentParser(prog='stim16a.py',
                                description=__doc__.split('\n')[0])
    p.add_argument('outdir')
    p.add_argument('--fs', type=int, nargs='+', default=[8000])
    p.add_argument('--peak', type=int, nargs='+', required=True)
    p.add_argument('--seeds', type=int_list, default=[0])
    p.add_argument('--nseq', type=int, default=8)
    p.add_argument('--ulaw', action='store_true')
    p.add_argument('--name')
    a = p.parse_args(argv)
    template = a.name or ('16au' if a.ulaw else '16a') + \
        '-{fs}-{peak}-s{seed}'
    grid = [(template.format(fs=fs, peak=peak, seed=seed), fs, peak, seed)
            for fs in a.fs for peak in a.peak for seed in a.seeds]
    names_check([(g[0], 'fs %d peak %d seed %d' % g[1:]) for g in grid])
    for name, fs, peak, seed in grid:
        m = build(os.path.join(a.outdir, name), fs, seed, peak, a.nseq,
                  ulaw=a.ulaw)
        print(name, 'phase', m['phase'], 'frames', m['frames'])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
