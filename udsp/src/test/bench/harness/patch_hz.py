#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Write the instrumented copy of a canceller source that hz.c includes.

usage: patch_hz.py <lec.c> <out.c>

The copy gets three edits, each at an anchor that must occur exactly once
in the source:

- after the line '#include "lec.h"': the prototypes of hz_pre and hz_post;
- in ec_run, before the line that saves the coefficients of an adapting
  frame ('if (adapt)' and its memcpy to ec->backup): a call to hz_pre with
  the frame record, the adapt verdict and the step floor;
- at the end of ec_run, after 'reading_fill(ec, &t);': a call to hz_post.

An anchor that is missing or occurs twice ends the program with status 1
and names the anchor, so the build fails on a source that the hooks do not
fit. The source itself stays unchanged.
"""
import sys

INCLUDE = '#include "lec.h"\n'
PROTOS = ('struct udsp_ec;\n'
          'struct lec_tick;\n'
          'static void hz_pre(struct udsp_ec *ec, struct lec_tick *t, '
          'bool adapt, float step_floor);\n'
          'static void hz_post(struct udsp_ec *ec, struct lec_tick *t, '
          'bool adapt);\n')
BACKUP = ('\tif (adapt)\n'
          '\t\tmemcpy(ec->backup, ec->coef, sizeof(*ec->coef) * ec->taps);\n')
PRE = '\thz_pre(ec, &t, adapt, step_floor);\n'
FILL = '\treading_fill(ec, &t);\n}\n'
POST = '\treading_fill(ec, &t);\n\thz_post(ec, &t, adapt);\n}\n'


def replace_once(text, old, new):
    """text with its one occurrence of old replaced by new."""
    count = text.count(old)
    if count != 1:
        sys.exit(f'patch_hz.py: anchor {old.strip()[:60]!r} occurs {count} '
                 f'times, 1 expected')
    return text.replace(old, new)


def patch(text):
    """The instrumented form of the canceller source text."""
    text = replace_once(text, INCLUDE, INCLUDE + PROTOS)
    text = replace_once(text, BACKUP, PRE + BACKUP)
    return replace_once(text, FILL, POST)


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: patch_hz.py <lec.c> <out.c>')
    with open(argv[1]) as f:
        text = f.read()
    with open(argv[2], 'w') as f:
        f.write(patch(text))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
