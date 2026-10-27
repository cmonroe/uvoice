# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Reader of a canceller capture (.lecr), and the capture index of a set.

The header comment of test/bench/lec_replay.h gives the layout: a file
header of 32 bytes, then records of 64 + 6 x samples bytes, little-endian.
read_lecr applies no rule of a lineage; lec_replay does that. It reads the
fields that the analysis scripts use.

The replay command of lec_harness.py writes fld/<set>/index.json in the work
directory: the capture file of each id of the set.
"""
import json
import math
import struct

import numpy as np

import work

MAGIC = b'UDSPLECR'
FILE_HEAD = 32
RECORD_HEAD = 64
KIND_RUN = 0
KIND_FEED = 1
KIND_END = 2
VERDICTS = ('off', 'quiet', 'dt', 'unlike', 'erle', 'pass')
FULL_SCALE_SINE = 32767.0 ** 2 / 2


def read_lecr(path):
    """(rate, samples, ref_loss_half_db, records) of a capture. Each record
    is a dict: tick, frames, kind, flags, verdict, hold_left, noise_ddbov,
    erle_mdb, coef and the planes ref, mic and out as float64."""
    with open(path, 'rb') as f:
        b = f.read()
    if b[:8] != MAGIC:
        raise ValueError(f'{path}: not a .lecr file')
    rate, samples = struct.unpack_from('<IH', b, 12)
    loss, count = struct.unpack_from('<HI', b, 18)
    size = RECORD_HEAD + 6 * samples
    recs = []
    for i in range(count):
        o = FILE_HEAD + i * size
        if o + size > len(b):
            break
        _, tick, _, frames, _, erle, _, coef, _ = struct.unpack_from(
            '<QQIIIiIII', b, o)
        _, flags, _, _, noise, _, _ = struct.unpack_from('<7H', b, o + 44)
        _, kind, verdict, hold_left = struct.unpack_from('<4B', b, o + 58)
        planes = np.frombuffer(b, '<i2', 3 * samples, o + RECORD_HEAD)
        planes = planes.astype(float).reshape(3, samples)
        recs.append(dict(tick=tick, frames=frames, kind=kind, flags=flags,
                         verdict=verdict, hold_left=hold_left,
                         noise_ddbov=noise, erle_mdb=erle, coef=coef,
                         ref=planes[0], mic=planes[1], out=planes[2]))
    return rate, samples, loss, recs


def runs_by_tick(recs):
    """The RUN records of a capture by tick."""
    return {r['tick']: r for r in recs if r['kind'] == KIND_RUN}


def index_read(set_name):
    """The capture file of each id of a replayed set."""
    with open(work.fld_dir(set_name) / 'index.json') as f:
        return json.load(f)


def dbov(power):
    """A mean square in int16 units as dBov against a full-scale sine, the
    unit of dsp/cn.c; -200 for 0."""
    return 10 * math.log10(power / FULL_SCALE_SINE) if power > 0 else -200.0
