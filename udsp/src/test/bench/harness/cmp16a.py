#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The 16A runs of a set per build: coherence trials and digit tone loss.

usage: cmp16a.py <set> <build>[,<build>...] [--base BUILD] [--level DBM0]
                 [--all]

The runs are run/<set>/<build>/<case>/ of the work directory. For each
build, one line gives:

- the coherence trials (trial arm 4) over all its runs;
- the worst tone loss of a digit at the level (default -9 dBm0);
- without --base: every digit, at any level, that loses more than 1 dB;
- with --base: every digit whose loss passes that of the same digit on the
  base build by more than 1 dB.

A digit prints as (case, level, digit, loss), and with --base the loss on
the base build follows. The line lists the first 10 digits, and every digit
with --all.
"""
import argparse
import sys

import an16a
import work

LEVELS = (-27.0, -24.0, -21.0, -18.0, -15.0, -12.0, -9.0, -6.0)
ARM_COHERE = 4
LOSS_DB = 1.0
SHOW = 10


def losses(run):
    """The worse tone loss of each digit of a loaded run, by (level, seq,
    digit), in the order of digits.json."""
    return {(g['level'], g['seq'], g['digit']): min(an16a.tone_loss(run, g))
            for g in run.digits if g['level'] in LEVELS}


def build_line(set_name, b, base, level, show=SHOW):
    trials = 0
    worst = 0.0
    bad = []
    for d in sorted(work.run_dir(set_name, b).iterdir()):
        run = an16a.load(d)
        trials += sum(1 for r in run.rows if int(r['arm']) == ARM_COHERE)
        lb = losses(run)
        l0 = losses(an16a.load(work.run_dir(set_name, base, d.name))) \
            if base else None
        for k, v in lb.items():
            if k[0] == level:
                worst = max(worst, -v)
            if (l0 is None and v < -LOSS_DB) or \
                    (l0 is not None and v < l0[k] - LOSS_DB):
                bad.append((d.name, k[0], k[2], round(v, 2)) +
                           ((round(l0[k], 2),) if l0 else ()))
    what = ('digits >1 dB worse than %s' % base if base
            else 'digits with loss > 1 dB')
    return ('%s %s: coherence trials %d; worst %g dBm0 digit tone loss %.2f '
            'dB; %s: %d %s' % (set_name, b, trials, level, worst, what,
                               len(bad), bad[:show] if show else bad))


def main(argv):
    p = argparse.ArgumentParser(prog='cmp16a.py')
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('--base')
    p.add_argument('--level', type=float, default=-9.0)
    p.add_argument('--all', action='store_true')
    a = p.parse_args(argv)
    for b in a.builds:
        print(build_line(a.set, b, a.base, a.level, 0 if a.all else SHOW))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
