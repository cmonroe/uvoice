#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""A synthetic placement case: a 2C(c) stimulus whose Hoth noise moves
apart from its echo.

usage: synplace.py <fs> <peak> <hoth_shift> <seed> <phase> <build>
                   [<build> ...]

The stimulus is that of stim.py with the echo peak on tap peak and the near
noise moved hoth_shift samples earlier. The program runs lec_hz of each
build on it, writes the summary (analyze.py) to
sum/sp/<build>/sp-<fs>-<peak>-h<hoth_shift>-s<seed>-p<phase>.json in the
work directory, and removes the stimulus.
"""
import json
import shutil
import sys

import analyze
import stim
import work


def main(argv):
    if len(argv) < 6:
        sys.exit('usage: synplace.py <fs> <peak> <hoth_shift> <seed> '
                 '<phase> <build> [<build> ...]')
    fs, peak, shift, seed, phase = (int(v) for v in argv[:5])
    name = f'sp-{fs}-{peak}-h{shift}-s{seed}-p{phase}'
    o = work.work_dir() / 'tmp' / 'sp' / name
    stim.build(str(o), fs, seed, peak, phase=phase, hoth_shift=shift)
    rc = 0
    for b in argv[5:]:
        ob = o / b
        if work.hz_run(b, o, ob):
            rc = 1
            continue
        d = work.sum_dir('sp', b)
        d.mkdir(parents=True, exist_ok=True)
        (d / f'{name}.json').write_text(json.dumps(analyze.summary(ob)))
    shutil.rmtree(o)
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
