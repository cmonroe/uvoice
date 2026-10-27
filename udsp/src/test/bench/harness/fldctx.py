#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The frames of one replayed capture from one index to another, on two
builds.

usage: fldctx.py <set> <base> <var> <id> <from> <to>

The frame files are fld/<set>/<build>/<id>.csv of the work directory. A
line per frame index from from to to: the verdict, fired and nlp_active of
both builds, the reference, the microphone and the estimate in dBov, both
outputs and floors, both ERLE figures and the trial arm of both.
"""
import sys

import lecr
import work


def main(argv):
    if len(argv) != 6:
        sys.exit('usage: fldctx.py <set> <base> <var> <id> <from> <to>')
    set_name, base, var, cap = argv[:4]
    a = work.csv_rows(work.fld_dir(set_name, base) / f'{cap}.csv')
    q = work.csv_rows(work.fld_dir(set_name, var) / f'{cap}.csv')
    print('frame   base: v  F act | var: v  F act | ref    mic    est | out '
          'base  var | floor base var | erle base var | trial')
    for i in range(int(argv[4]), min(int(argv[5]) + 1, len(a), len(q))):
        x, y = a[i], q[i]
        print('%6d  %-6s %s %s | %-6s %s %s | %6.1f %6.1f %6.1f | %6.1f '
              '%6.1f | %6.1f %6.1f | %5.1f %5.1f | %s%s' % (
                  i, x['verdict'], x['fired'], x['nlp_active'],
                  y['verdict'], y['fired'], y['nlp_active'],
                  lecr.dbov(float(x['ref_power'])),
                  lecr.dbov(float(x['mic_power'])),
                  lecr.dbov(float(x['estimate_power'])),
                  lecr.dbov(float(x['out_power'])),
                  lecr.dbov(float(y['out_power'])),
                  float(x['noise_dbov']), float(y['noise_dbov']),
                  float(x['erle_db']), float(y['erle_db']), x['trial_arm'],
                  y['trial_arm']))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
