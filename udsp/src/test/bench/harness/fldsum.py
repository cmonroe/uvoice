#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""One line per event of fldev.py, with a frame label and an event class.

usage: fldsum.py <set> <base> <var>

The events come from fld/<set>/events-<base>-<var>.json of the work
directory. The label of a frame, from the levels in dBov:

- leak: the estimate stands more than 3 dB over the microphone: the output
  carries the estimate on a frame whose echo it does not match, a tail or
  a pre-echo frame;
- echo: the microphone stands within 3 dB of the estimate;
- near: the microphone stands more than 6 dB over the estimate and more
  than 10 dB over the floor of var;
- noise: the microphone stands within 6 dB of that floor;
- mixed: the rest.

A * marks a frame where either build reads no floor. The class of an event:

- inspect: a near frame or a DTMF frame;
- residual echo: echo and leak frames make 80 percent of it, echo first;
- estimate on a tail frame: the same with more leak frames than echo;
- floor noise: noise frames make half of it;
- mixed: the rest.

A line gives the event, its labels, the verdict letters around it from
base, the frame with the largest microphone over the estimate and the
output range of both builds. The last lines count the labels and the
classes. The labels carry no near-speech truth: a person checks an
inspect event by listening, or with fldres.py.
"""
import collections
import sys

from fldev import events_read


def label_get(f):
    flo = -f['fq'] if f['fq'] < 127 else (-f['fa'] if f['fa'] < 127
                                          else -200)
    m, e = f['mic'], f['est']
    if e > m + 3:
        lab = 'leak'
    elif abs(m - e) <= 3:
        lab = 'echo'
    elif m > e + 6 and m > flo + 10:
        lab = 'near'
    elif m <= flo + 6:
        lab = 'noise'
    else:
        lab = 'mixed'
    return lab + ('*' if f['fa'] >= 127 or f['fq'] >= 127 else '')


def class_get(e, c):
    """The class of event e with the label counter c."""
    plain = collections.Counter()
    for k, v in c.items():
        plain[k.rstrip('*')] += v
    if plain['near'] or e['dtmf']:
        return 'inspect'
    if plain['echo'] + plain['leak'] >= 0.8 * e['n']:
        return ('residual echo' if plain['echo'] >= plain['leak']
                else 'estimate on a tail frame')
    if plain['noise'] >= 0.5 * e['n']:
        return 'floor noise'
    return 'mixed'


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: fldsum.py <set> <base> <var>')
    labels, classes = collections.Counter(), collections.Counter()
    for k, e in enumerate(events_read(*argv)):
        c = collections.Counter(label_get(f) for f in e['frames'])
        cls = class_get(e, c)
        labels.update(c)
        classes[cls] += 1
        mx = max(e['frames'], key=lambda f: f['mic'] - f['est'])
        print('#%-3d %-12s frames %6d-%-6d n %3d | %s | %s | base ctx %s | '
              'max mic-est %+.1f dB (mic %.1f est %.1f floor %s/%s) | out '
              '%.1f..%.1f -> %.1f..%.1f %s' % (
                  k, e['cap'], e['start'], e['start'] + e['n'] - 1, e['n'],
                  cls, ' '.join('%s:%d' % kv for kv in sorted(c.items())),
                  ''.join(x[0] for x in e['ctx']), mx['mic'] - mx['est'],
                  mx['mic'], mx['est'], mx['fa'], mx['fq'],
                  min(f['oa'] for f in e['frames']),
                  max(f['oa'] for f in e['frames']),
                  min(f['oq'] for f in e['frames']),
                  max(f['oq'] for f in e['frames']),
                  'DTMF %s' % e['dtmf'] if e['dtmf'] else ''))
    print('frame labels', dict(labels))
    print('event classes', dict(classes))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
