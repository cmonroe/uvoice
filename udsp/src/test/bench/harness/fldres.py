#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The near content of the events of fldev.py, from a fit of the captured
microphone on the captured reference.

usage: fldres.py <set> <base> <var>

The events come from fld/<set>/events-<base>-<var>.json of the work
directory, and the planes from the captures of fld/<set>/index.json. Over
each event and 300 ticks on each side, a tail-long filter from the
reference to the microphone is fitted (Toeplitz normal equations, ridge
1e-3): 48 ms at 8 kHz and 64 ms at 16 kHz. The part of the microphone that
the reference does not explain is the near content.

Per event, one line gives the frames that are near candidates and the
frame with the most unexplained level over the floor. A near candidate:
its unexplained part stands 10 dB or more over the floor of var, carries a
quarter or more of the microphone power, and the microphone stands more
than 6 dB over the estimate. The fit quality is the microphone power over
the unexplained power on the frames of the window whose reference passes
a mean square of 2500 at the line level. The events with their frames go to
fld/<set>/near-<base>-<var>.json.
"""
import json
import math
import sys

import numpy as np

import lecr
import siggen as sg
import work
from fldev import events_read

SPAN_TICKS = 300
RIDGE = 1e-3
TAPS = {8000: 384, 16000: 1024}
ACTIVE_POWER = 2500.0


def fit_unexplained(ref, mic, taps):
    """mic less its least-squares prediction from ref by a taps-long
    filter."""
    n = len(ref)
    cc = sg.fftconvolve(mic, ref[::-1])[n - 1:n - 1 + taps]
    acf = sg.fftconvolve(ref, ref[::-1])[n - 1:n - 1 + taps].copy()
    acf[0] = acf[0] * (1.0 + RIDGE) + 1.0
    h = sg.solve_toeplitz(acf, cc)
    return mic - sg.fftconvolve(ref, h)[:n]


def event_get(e, rate, fr, loss_half_db, runs):
    """The frame labels and the fit quality of event e. The capture's
    reference stands loss_half_db under the line, so the bar of an active
    frame takes the power factor 10^(-loss_half_db / 20)."""
    ticks = [f['tick'] for f in e['frames']]
    seg = [t for t in range(min(ticks) - SPAN_TICKS,
                            max(ticks) + SPAN_TICKS + 1) if t in runs]
    ref = np.concatenate([runs[t]['ref'] for t in seg])
    mic = np.concatenate([runs[t]['mic'] for t in seg])
    unx = fit_unexplained(ref, mic, TAPS[rate])
    bar = ACTIVE_POWER * 10 ** (-loss_half_db / 20.0)
    act = [j for j in range(len(seg))
           if np.mean(ref[j * fr:(j + 1) * fr] ** 2) > bar]
    fitq = 0.0
    if act:
        em = sum(np.mean(mic[j * fr:(j + 1) * fr] ** 2) for j in act)
        eu = sum(np.mean(unx[j * fr:(j + 1) * fr] ** 2) for j in act)
        fitq = 10 * math.log10(em / max(eu, 1e-9))
    labs = []
    for f in e['frames']:
        j = seg.index(f['tick'])
        pm = np.mean(mic[j * fr:(j + 1) * fr] ** 2)
        pu = np.mean(unx[j * fr:(j + 1) * fr] ** 2)
        flo = f['fq'] if f['fq'] < 127 else (f['fa'] if f['fa'] < 127
                                             else 127)
        near = (lecr.dbov(pu) >= -flo + 10 and pu >= 0.25 * pm and
                f['mic'] > f['est'] + 6)
        labs.append(dict(i=f['i'], mic_dbov=round(lecr.dbov(pm), 1),
                         unx=round(lecr.dbov(pu), 1), floor=flo,
                         near=bool(near), est=f['est'], mic=f['mic']))
    return labs, round(fitq, 1)


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: fldres.py <set> <base> <var>')
    set_name, base, var = argv
    src = lecr.index_read(set_name)
    cache, out = {}, []
    for k, e in enumerate(events_read(set_name, base, var)):
        cid = e['cap']
        if cid not in cache:
            rate, fr, loss, recs = lecr.read_lecr(src[cid])
            cache[cid] = (rate, fr, loss, lecr.runs_by_tick(recs))
        labs, fitq = event_get(e, *cache[cid])
        nn = sum(lb['near'] for lb in labs)
        out.append(dict(k=k, cap=cid, start=e['start'], n=e['n'],
                        near_frames=nn, frames=labs, fitq=fitq))
        worst = max(labs, key=lambda lb: lb['unx'] + lb['floor'])
        print('#%-3d %-10s %6d n %3d near-candidate frames %2d | fit %.1f dB '
              '| worst: mic %.1f est %.1f unexplained %.1f floor -%.1f dBov'
              % (k, cid, e['start'], e['n'], nn, fitq, worst['mic_dbov'],
                 worst['est'], worst['unx'], worst['floor']))
    with open(work.fld_dir(set_name) / f'near-{base}-{var}.json', 'w') as f:
        json.dump(out, f)
    print(var, 'events', len(out), 'with near-candidate frames',
          sum(1 for o in out if o['near_frames']), 'frames',
          sum(o['near_frames'] for o in out))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
