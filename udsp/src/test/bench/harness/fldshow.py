#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The frames of the events of fldev.py.

usage: fldshow.py <set> <base> <var> [<label> [<frames>]]

The events come from fld/<set>/events-<base>-<var>.json of the work
directory. label keeps the events with a frame of that first label of
fldev.py, or with a DTMF frame for the label dtmf. Each event prints its
labels, the verdict letters around it and its first frames (60 by default):
the verdicts of base and var, the microphone, the estimate and the
reference in dBov, both floors and both outputs.
"""
import sys

from fldev import events_read


def main(argv):
    if len(argv) < 3:
        sys.exit('usage: fldshow.py <set> <base> <var> [<label> [<frames>]]')
    only = argv[3] if len(argv) > 3 else None
    count = int(argv[4]) if len(argv) > 4 else 60
    for k, e in enumerate(events_read(*argv[:3])):
        if only and only not in e['labels'] and \
                not (only == 'dtmf' and e['dtmf']):
            continue
        print('#%d %s frames %d-%d (%d) labels %s ctx %s dtmf %s' % (
            k, e['cap'], e['start'], e['start'] + e['n'] - 1, e['n'],
            ','.join(e['labels']), ''.join(c[0] for c in e['ctx']),
            e['dtmf'] or ''))
        for f in e['frames'][:count]:
            print('    %6d %-6s->%-6s mic %6.1f est %6.1f ref %6.1f floor '
                  '%6.2f/%6.2f out %6.1f->%6.1f %s %s' % (
                      f['i'], f['va'], f['vq'], f['mic'], f['est'], f['ref'],
                      f['fa'], f['fq'], f['oa'], f['oq'], f['lab'],
                      f['dtmf'] or ''))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
