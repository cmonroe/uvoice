#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The two-peak share of the reference frames of each coherence trial of a
replayed capture set.

usage: frmax.py <set> <build>

The rows come from fld/<set>/<build>/<id>.coh (lec_replay_coh) and the
planes from the captures of fld/<set>/index.json in the work directory.
For each trial (trial arm 4), the share (siggen.two_peak_share, the measure
of ref_tone_share_get in dsp/lec.c) is read on the active frames of its run.
One line per trial gives the capture, the row, the end of the trial, the
active frames and the largest and the median share.
"""
import sys

import numpy as np

import lecr
import siggen as sg
import work
from armfld import ARM_COHERE, ENDS, coh_read


def main(argv):
    if len(argv) != 2:
        sys.exit('usage: frmax.py <set> <build>')
    set_name, b = argv
    src = lecr.index_read(set_name)
    for p in sorted(work.fld_dir(set_name, b).glob('*.coh')):
        co = coh_read(p)
        arms = [i for i, c in enumerate(co) if c[12] == ARM_COHERE]
        if not arms:
            continue
        rows = work.csv_rows(p.with_suffix('.csv'))
        runs = [int(c[1]) for c in co]
        rate, _, _, recs = lecr.read_lecr(src[p.stem])
        t2r = lecr.runs_by_tick(recs)
        for i in arms:
            a = work.run_start(runs, i)
            act = [j for j in range(a, i + 1) if co[j][9] == '1']
            v = [sg.two_peak_share(t2r[int(rows[j]['tick'])]['ref'], rate)
                 for j in act if int(rows[j]['tick']) in t2r]
            end = next((c[13] for c in co[i + 1:] if c[13] != '0'), None)
            print(f'{set_name} {p.stem[-30:]:30} {i:6} '
                  f'{ENDS.get(end, end) or "open":8} active {len(v):2} ref '
                  f'share max {max(v):.3f} med {np.median(v):.3f}'
                  if v else f'{set_name} {p.stem} {i} no active frame')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
