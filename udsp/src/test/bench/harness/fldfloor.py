#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The floors that var reads louder than base over a replayed capture set.

usage: fldfloor.py <set> <base> <var>

The frame files are fld/<set>/<build>/<id>.csv of the work directory, and
a floor is the noise_dbov column (127 when unmeasured). Per capture with
such frames, one line gives the frames where the floor of var stands 6 dB
or more over that of base, the longest run of them with both floors at its
start, the loudest floor of var there, and the frames whose floor of var
stands over -60 dBov while that of base does not. The last line gives the
totals.
"""
import sys

import work

LOUDER_DB = 6.0
LOUD_DBOV = 60.0


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: fldfloor.py <set> <base> <var>')
    set_name, base, var = argv
    tot = [0, 0]
    for p in sorted(work.fld_dir(set_name, base).glob('*.csv')):
        a = work.csv_rows(p)
        q = work.csv_rows(work.fld_dir(set_name, var) / p.name)
        louder, loud = [], 0
        for i in range(min(len(a), len(q))):
            fa, fq = float(a[i]['noise_dbov']), float(q[i]['noise_dbov'])
            if fq < 127 and fq <= fa - LOUDER_DB:
                louder.append(i)
            if fq < LOUD_DBOV and not fa < LOUD_DBOV:
                loud += 1
        if not louder and not loud:
            continue
        runs = work.index_runs(louder)
        lg = max(runs, key=len) if runs else [0, -1]
        loudest = min((float(q[i]['noise_dbov']) for i in louder),
                      default=127)
        tot[0] += len(louder)
        tot[1] += loud
        print('%-12s frames louder by 6 dB+ %6d, longest %5d (frames %d-%d, '
              'base %.1f var %.1f), loudest var floor -%.1f dBov, frames var '
              '< 60 dBov where base is not: %d' % (
                  p.stem, len(louder), lg[-1] - lg[0] + 1, lg[0], lg[-1],
                  float(a[lg[0]]['noise_dbov']),
                  float(q[lg[0]]['noise_dbov']), loudest, loud))
    print(var, 'totals louder-by-6 frames', tot[0], 'var-under-60 frames',
          tot[1])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
