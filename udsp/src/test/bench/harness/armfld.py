#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Every coherence trial of a replayed capture set, with the readings of
its run.

usage: armfld.py <set> <build>

The rows come from fld/<set>/<build>/<id>.coh (lec_replay_coh, the 15
columns of patch_coh.py) and the planes from the captures of
fld/<set>/index.json in the work directory. A trial starts on a row whose
trial arm reads 4 (UDSP_EC_NLMS_TRIAL_ARM_COHERE); its run is the rows
before it with a cohere_run over 0. Per trial, one line gives the
capture, the row, the length of the run, the end of the trial, the range
of ncc and the largest share, the talker excess against the talker ratio,
the median and the largest microphone over the estimate, the shape of the
reference over the run (spec.shape), its order-4 prediction gain (spec.pg4)
over the frames, its level, and the same figures of the microphone. The
trials go to fld/<set>/arms-<build>.json.
"""
import json
import sys

import numpy as np

import lecr
import spec
import work

ARM_COHERE = '4'
ENDS = {'1': 'kept', '2': 'refused', '3': 'diverged', '4': 'talker'}
COLS = 15


def coh_read(path):
    with open(path) as f:
        return [c for c in (line.strip().split(',') for line in f)
                if len(c) == COLS]


def trial_get(cid, co, i, rate, fr, t2r, rows):
    a = work.run_start([int(c[1]) for c in co], i)
    run = co[a:i]
    ncc = [float(c[2]) for c in run if float(c[2]) < 9]
    sh = [float(c[3]) for c in run if float(c[3]) >= 0]
    act = [c for c in run if c[9] == '1']
    tex = [work.db(float(c[4]) / max(float(c[7]), 1e-9)) for c in act]
    eex = [work.db(float(c[4]) / max(float(c[5]), 1e-9)) for c in act]
    ticks = [int(rows[j]['tick']) for j in range(a, i + 1)]
    refx = np.concatenate([t2r[t]['ref'] for t in ticks if t in t2r])
    micx = np.concatenate([t2r[t]['mic'] for t in ticks if t in t2r])
    rs, ms = spec.shape(refx, rate), spec.shape(micx, rate)
    rpg, mpg = spec.pgframes(refx, fr), spec.pgframes(micx, fr)
    end = next(((j, c[13]) for j, c in enumerate(co[i + 1:], i + 1)
                if c[13] != '0'), None)
    return dict(cap=cid, frame=i, rate=rate, run=len(run),
                end=ENDS.get(end[1], end[1]) if end else 'open',
                end_frame=end[0] if end else None,
                ncc_min=min(ncc) if ncc else None,
                ncc_max=max(ncc) if ncc else None,
                share_max=max(sh) if sh else None,
                talk_excess_max=max(tex) if tex else None,
                talker_ratio_db=work.db(float(co[i][8])),
                est_excess_med=float(np.median(eex)) if eex else None,
                est_excess_max=max(eex) if eex else None,
                ref_top2=rs[0], ref_flat=rs[1], mic_top2=ms[0],
                mic_flat=ms[1],
                ref_db=lecr.dbov(float(np.mean(refx ** 2))),
                ref_pg_med=rpg[0], ref_pg_min=rpg[1], mic_pg_med=mpg[0],
                mic_db=lecr.dbov(float(np.mean(micx ** 2))))


def main(argv):
    if len(argv) != 2:
        sys.exit('usage: armfld.py <set> <build>')
    set_name, b = argv
    src = lecr.index_read(set_name)
    res = []
    for p in sorted(work.fld_dir(set_name, b).glob('*.coh')):
        co = coh_read(p)
        arms = [i for i, c in enumerate(co) if c[12] == ARM_COHERE]
        if not arms:
            continue
        rows = work.csv_rows(p.with_suffix('.csv'))
        rate, fr, _, recs = lecr.read_lecr(src[p.stem])
        t2r = lecr.runs_by_tick(recs)
        for i in arms:
            r = trial_get(p.stem, co, i, rate, fr, t2r, rows)
            res.append(r)
            print('%-26s f %6d run %2d %-8s ncc %.2f..%.2f share<=%.2f talk '
                  '%.1f/%.1f est-ex med %.1f max %.1f | ref top2 %.2f flat '
                  '%.3f pg4 med %.1f min %.1f (%.0f dBov) | mic top2 %.2f pg4 '
                  '%.1f (%.0f dBov)' % (
                      p.stem[-24:], i, r['run'], r['end'], r['ncc_min'] or 0,
                      r['ncc_max'] or 0, r['share_max'] or 0,
                      r['talk_excess_max'] or 0, r['talker_ratio_db'],
                      r['est_excess_med'] or 0, r['est_excess_max'] or 0,
                      r['ref_top2'], r['ref_flat'], r['ref_pg_med'],
                      r['ref_pg_min'], r['ref_db'], r['mic_top2'],
                      r['mic_pg_med'], r['mic_db']))
    with open(work.fld_dir(set_name) / f'arms-{b}.json', 'w') as f:
        json.dump(res, f)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
