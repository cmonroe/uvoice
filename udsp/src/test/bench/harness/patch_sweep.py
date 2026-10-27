#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Write the source of lec_tone_sweep: test_lec.c with sweep_tail.c after it.

usage: patch_sweep.py <test_lec.c> <sweep_tail.c> <out.c>

The copy of the suite keeps every case and helper. Its main becomes the
unused static function tl_main, so the main of sweep_tail.c runs instead,
and the references of tl_main keep the cases in use. A #line directive
before the tail makes the compiler name sweep_tail.c in its messages. The
anchor 'int main(int argc, char **argv)' must occur exactly once; otherwise
the program ends with status 1 and the build fails.
"""
import sys

MAIN = 'int main(int argc, char **argv)\n'
TL_MAIN = 'static int __attribute__((unused)) tl_main(int argc, char **argv)\n'


def main(argv):
    if len(argv) != 4:
        sys.exit('usage: patch_sweep.py <test_lec.c> <sweep_tail.c> <out.c>')
    with open(argv[1]) as f:
        suite = f.read()
    count = suite.count(MAIN)
    if count != 1:
        sys.exit(f'patch_sweep.py: anchor {MAIN.strip()!r} occurs {count} '
                 f'times, 1 expected')
    with open(argv[2]) as f:
        tail = f.read()
    with open(argv[3], 'w') as f:
        f.write(suite.replace(MAIN, TL_MAIN))
        f.write(f'#line 1 "{argv[2]}"\n')
        f.write(tail)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
