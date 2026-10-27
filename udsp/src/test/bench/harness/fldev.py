#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Every frame of a replayed capture set on which the processor of var
newly acts, grouped in events.

usage: fldev.py <set> <base> <var>

The frame files are fld/<set>/<build>/<id>.csv of the work directory, and
fld/<set>/index.json names the capture of each id. A newly acted frame
reads nlp_active 1 on var and 0 on base. An event is a run of such frames.
Each frame of an event carries its index and tick, both verdicts, the
microphone, the estimate and the reference of base in dBov, both floors
(noise_dbov) and both outputs, dt and fired of base, a first label and the
DTMF check of the captured microphone. The first label:

- no-floor: either build reads no floor (127);
- echo: the microphone stands within 3 dB of the estimate or under it;
- near: the microphone stands more than 6 dB over the estimate and more
  than 10 dB over the floor of var;
- noise: the microphone stands within 6 dB of that floor;
- mixed: the rest.

The DTMF check finds the strongest row and column tone of the microphone
frame (a single-bin DFT at each Q.23 frequency); it reports the pair and
its share of the frame's power when the share passes 0.6, on a frame at
-45 dBm0 or more.

The events go to fld/<set>/events-<base>-<var>.json, with the verdict
letters (F for fired) of the 8 frames around each event. The command
prints the count of events and frames, the frame labels and the events
with DTMF frames.
"""
import collections
import json
import sys

import numpy as np

import lecr
import siggen as sg
import work

DTMF_SHARE = 0.6
DTMF_MIN_DBM0 = -45.0
CONTEXT = 8


def dtmf_get(x, rate):
    """(f_lo, f_hi, share) of a frame that carries a DTMF pair, or None."""
    if np.mean(x ** 2) < sg.dbm0_to_rms(DTMF_MIN_DBM0) ** 2:
        return None
    lo = [sg.tone_power(x, f, rate) for f in sg.DTMF_ROWS]
    hi = [sg.tone_power(x, f, rate) for f in sg.DTMF_COLS]
    il, ih = int(np.argmax(lo)), int(np.argmax(hi))
    share = (lo[il] + hi[ih]) / max(np.mean(x ** 2) / 2, 1e-9)
    if share <= DTMF_SHARE:
        return None
    return (sg.DTMF_ROWS[il], sg.DTMF_COLS[ih], round(share, 2))


def label_get(mic, est, fa, fq):
    if fa >= 127 or fq >= 127:
        return 'no-floor'
    if mic <= est + 3:
        return 'echo'
    if mic > est + 6 and mic > -fq + 10:
        return 'near'
    return 'noise' if mic <= -fq + 6 else 'mixed'


def frame_get(i, x, y, mic_plane, rate):
    mic = lecr.dbov(float(x['mic_power']))
    est = lecr.dbov(float(x['estimate_power']))
    fa, fq = float(x['noise_dbov']), float(y['noise_dbov'])
    return dict(i=i, tick=int(x['tick']), va=x['verdict'], vq=y['verdict'],
                mic=round(mic, 1), est=round(est, 1),
                ref=round(lecr.dbov(float(x['ref_power'])), 1), fa=fa, fq=fq,
                oa=round(lecr.dbov(float(x['out_power'])), 1),
                oq=round(lecr.dbov(float(y['out_power'])), 1), dt=x['dt'],
                fired=x['fired'], lab=label_get(mic, est, fa, fq),
                dtmf=(dtmf_get(mic_plane, rate) if mic_plane is not None
                      else 'nomic'))


def capture_events(cid, path, a, q):
    n = min(len(a), len(q))
    more = [i for i in range(n)
            if q[i]['nlp_active'] == '1' and a[i]['nlp_active'] == '0']
    if not more:
        return []
    rate, _, _, recs = lecr.read_lecr(path)
    mics = {t: r['mic'] for t, r in lecr.runs_by_tick(recs).items()}
    out = []
    for e in work.index_runs(more):
        fr = [frame_get(i, a[i], q[i], mics.get(int(a[i]['tick'])), rate)
              for i in e]
        ctx = [a[j]['verdict'][0] + ('F' if a[j]['fired'] == '1' else '')
               for j in range(max(0, e[0] - CONTEXT),
                              min(n, e[-1] + CONTEXT + 1))]
        out.append(dict(cap=cid, rate=rate, start=e[0], n=len(e), frames=fr,
                        ctx=ctx, labels=sorted(set(f['lab'] for f in fr)),
                        dtmf=[f['dtmf'] for f in fr
                              if f['dtmf'] not in (None, 'nomic')]))
    return out


def events_path(set_name, base, var):
    return work.fld_dir(set_name) / f'events-{base}-{var}.json'


def events_read(set_name, base, var):
    with open(events_path(set_name, base, var)) as f:
        return json.load(f)


def main(argv):
    if len(argv) != 3:
        sys.exit('usage: fldev.py <set> <base> <var>')
    set_name, base, var = argv
    src = lecr.index_read(set_name)
    out = []
    for p in sorted(work.fld_dir(set_name, base).glob('*.csv')):
        out.extend(capture_events(p.stem, src[p.stem], work.csv_rows(p),
                                  work.csv_rows(work.fld_dir(set_name, var) /
                                                p.name)))
    with open(events_path(set_name, base, var), 'w') as f:
        json.dump(out, f)
    labels = collections.Counter(f['lab'] for e in out for f in e['frames'])
    print(var, 'events', len(out), 'frames', sum(e['n'] for e in out),
          'frame labels', dict(labels), 'events with dtmf',
          sum(1 for e in out if e['dtmf']))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
