#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The frame files of two builds over a replayed capture set, compared.

usage: fldcmp.py <set> <base> <var> [--events]

The frame files are fld/<set>/<build>/<id>.csv of the work directory
(lec_replay --frames). Per capture whose frames differ, one line gives:

- the frames compared, and the frames whose floors (noise_dbov) stand more
  than 1 dB apart, with the largest gap;
- the frames whose verdict differs, and the fired frames that var gains
  and loses;
- the frames on which the processor of var newly acts (nlp_active 1 where
  base reads 0) and those it newly leaves, and the events: runs of newly
  acted frames, their count, the longest, and the output energy that they
  lose against base in dB;
- the frames whose output power differs by more than a thousandth.

--events lists each event with the first four frames: the verdicts, the
microphone and the estimate in dBov, the floors, the output of both builds
and the reference. The last line, totals, sums the counts over the set.
The exit status is 0 when no capture differs and 1 otherwise.
"""
import math
import sys

import lecr
import work


def event_text(a, q, e):
    return '    event at frame %d (%d frames): ' % (e[0], len(e)) + '; '.join(
        'v %s/%s mic %.1f est %.1f floor %s/%s out %.1f->%.1f ref %.1f' % (
            a[j]['verdict'], q[j]['verdict'],
            lecr.dbov(float(a[j]['mic_power'])),
            lecr.dbov(float(a[j]['estimate_power'])),
            a[j]['noise_dbov'][:5], q[j]['noise_dbov'][:5],
            lecr.dbov(float(a[j]['out_power'])),
            lecr.dbov(float(q[j]['out_power'])),
            lecr.dbov(float(a[j]['ref_power']))) for j in e[:4])


def capture_cmp(a, q, tot):
    """The counts of one capture; adds them to tot. Returns the text of a
    capture that differs, or None."""
    n = min(len(a), len(q))
    fd = [i for i in range(n) if abs(float(a[i]['noise_dbov']) -
                                     float(q[i]['noise_dbov'])) > 1.0]
    gap = max([float(a[i]['noise_dbov']) - float(q[i]['noise_dbov'])
               for i in fd], key=abs, default=0.0)
    vt = sum(a[i]['verdict'] != q[i]['verdict'] for i in range(n))
    fg = sum(q[i]['fired'] == '1' and a[i]['fired'] == '0' for i in range(n))
    fl = sum(q[i]['fired'] == '0' and a[i]['fired'] == '1' for i in range(n))
    more = [i for i in range(n)
            if q[i]['nlp_active'] == '1' and a[i]['nlp_active'] == '0']
    fewer = [i for i in range(n)
             if q[i]['nlp_active'] == '0' and a[i]['nlp_active'] == '1']
    outd = sum(abs(float(a[i]['out_power']) - float(q[i]['out_power'])) >
               1e-3 * (float(a[i]['out_power']) + 1) for i in range(n))
    ev = work.index_runs(more)
    tot['cap'] += 1
    tot['frames'] += n
    for k, v in (('fdiff', len(fd)), ('vt', vt), ('fg', fg), ('fl', fl),
                 ('more', len(more)), ('fewer', len(fewer)),
                 ('nev', len(ev)), ('outd', outd)):
        tot[k] += v
    if not (fd or vt or fg or fl or more or fewer or outd or
            len(a) != len(q)):
        return None
    tot['chg'] += 1
    loss = ''
    if more:
        ea = sum(float(a[i]['out_power']) for i in more)
        eq = sum(float(q[i]['out_power']) for i in more)
        loss = '%.1f' % (10 * math.log10(eq / ea)) if ea > 0 and eq > 0 \
            else ''
    return ('frames %6d floor>1dB %5d (gap %+.1f dB) vt %4d fired +%d/-%d '
            'newly acted %d left %d events %d longest %d loss %s out frames '
            'differ %d' % (n, len(fd), gap, vt, fg, fl, len(more),
                           len(fewer), len(ev),
                           max([len(e) for e in ev] or [0]), loss, outd)), ev


def main(argv):
    if len(argv) < 3:
        sys.exit('usage: fldcmp.py <set> <base> <var> [--events]')
    base, var = work.fld_dir(argv[0], argv[1]), work.fld_dir(argv[0], argv[2])
    tot = dict(cap=0, chg=0, frames=0, fdiff=0, vt=0, fg=0, fl=0, more=0,
               fewer=0, nev=0, outd=0)
    for p in sorted(base.glob('*.csv')):
        a, q = work.csv_rows(p), work.csv_rows(var / p.name)
        res = capture_cmp(a, q, tot)
        if res is None:
            continue
        print('%-46s %s' % (p.stem, res[0]))
        if '--events' in argv:
            for e in res[1]:
                print(event_text(a, q, e))
    print('totals', tot)
    return 1 if tot['chg'] else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
