#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Spectral and predictive measures of a reference: tonal tests for the
coherence escape of dsp/lec.c.

usage: spec.py <file.wav|file.s16> <rate> [<from_s> <to_s>]

The measures:

- top2: the share of the 200 to 3400 Hz energy of one frame in its two
  strongest peaks (siggen.two_peak_share, the measure of
  ref_tone_share_get in dsp/lec.c);
- shape: the same share over a whole run of samples with no zero padding,
  and the spectral flatness over 200 to 3400 Hz;
- pg4: the prediction gain in dB of an order-4 forward predictor, by the
  autocorrelation method (Levinson recursion);
- pgcov: the prediction gain in dB of an order-P predictor by the
  covariance method: the least-squares error of each sample from its P
  predecessors inside the block.

The command reads a mono 16-bit WAV file or raw int16 at rate, splits it
into 10 ms frames and prints the 5th, 50th and 95th percentiles of top2, pg4
and pgcov (order 4) over the frames whose mean square passes 1, and the
share of frames at 0.995 and at 0.999 or more.
"""
import math
import sys

import numpy as np

import siggen as sg

BAND_HZ = (200, 3400)
PEAK_BINS = 2


def top2(x, rate):
    """Two-peak share of one frame, or None for a frame under a mean square
    of 1."""
    x = np.asarray(x, dtype=float)
    if np.mean(x * x) < 1:
        return None
    return sg.two_peak_share(x, rate)


def shape(x, rate):
    """(two-peak share, spectral flatness) of x over BAND_HZ, with a Hann
    window over all of x."""
    x = np.asarray(x, dtype=float)
    if len(x) < 64 or np.mean(x * x) < 1:
        return (0.0, 0.0)
    p = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), 1 / rate)
    pb = p[(f >= BAND_HZ[0]) & (f <= BAND_HZ[1])]
    tot = pb.sum()
    rest = pb.copy()
    top = 0.0
    for _ in range(2):
        k = int(np.argmax(rest))
        lo, hi = max(0, k - PEAK_BINS), min(len(rest), k + PEAK_BINS + 1)
        top += rest[lo:hi].sum()
        rest[lo:hi] = 0
    flat = math.exp(np.mean(np.log(pb + 1e-9))) / max(np.mean(pb), 1e-9)
    return (top / max(tot, 1e-9), flat)


def pg4(x, order=4):
    """Prediction gain in dB of an order-4 predictor, autocorrelation
    method; 99 when the recursion meets a zero error."""
    x = np.asarray(x, dtype=float)
    if len(x) < 32 or np.mean(x * x) < 1:
        return 0.0
    r = np.array([np.dot(x[:len(x) - k], x[k:]) for k in range(order + 1)])
    r[0] *= 1.0 + 1e-9
    e = r[0]
    a = np.zeros(order + 1)
    a[0] = 1.0
    for m in range(1, order + 1):
        k = -np.dot(a[:m], r[m:0:-1]) / e
        a[:m + 1] = a[:m + 1] + k * a[:m + 1][::-1]
        e *= (1 - k * k)
        if e <= 0:
            return 99.0
    return 10 * math.log10(r[0] / e)


def pgframes(x, fr):
    """(median, minimum) of pg4 over the frames of x with a mean square
    over 1."""
    v = [pg4(x[j:j + fr]) for j in range(0, len(x) - fr + 1, fr)
         if np.mean(x[j:j + fr] ** 2) > 1]
    return (float(np.median(v)), float(min(v))) if v else (0.0, 0.0)


def pgcov(x, order=4):
    """Prediction gain in dB of an order-P predictor, covariance method."""
    x = np.asarray(x, float)
    if len(x) <= 4 * order or np.mean(x * x) < 1:
        return 0.0
    m = np.stack([x[order - k - 1:len(x) - k - 1] for k in range(order)],
                 axis=1)
    y = x[order:]
    rr = m.T @ m
    try:
        a = np.linalg.solve(rr + 1e-9 * np.trace(rr) * np.eye(order),
                            m.T @ y)
    except np.linalg.LinAlgError:
        return 99.0
    e = y - m @ a
    return 10 * math.log10(max(y @ y, 1e-9) / max(e @ e, 1e-9))


def samples_read(path):
    if path.endswith('.wav'):
        from benchstim import wav_read
        return wav_read(path)
    return np.fromfile(path, '<i2').astype(float)


def main(argv):
    if len(argv) not in (2, 4):
        sys.exit('usage: spec.py <file.wav|file.s16> <rate> [<from_s> '
                 '<to_s>]')
    rate = int(argv[1])
    x = samples_read(argv[0])
    if len(argv) == 4:
        x = x[int(float(argv[2]) * rate):int(float(argv[3]) * rate)]
    fr = rate // 100
    frames = [x[j:j + fr] for j in range(0, len(x) - fr + 1, fr)]
    shares = np.array([s for s in (top2(f, rate) for f in frames)
                       if s is not None])
    loud = [f for f in frames if np.mean(f * f) >= 1]
    gains = np.array([pg4(f) for f in loud])
    covs = np.array([pgcov(f) for f in loud])
    if not len(shares):
        print('no frame over a mean square of 1')
        return 1
    print('frames %d top2 p5 %.4f p50 %.4f p95 %.4f at 0.995 %.3f at 0.999 '
          '%.3f | pg4 p5 %.1f p50 %.1f p95 %.1f dB | pgcov p5 %.1f p50 %.1f '
          'p95 %.1f dB' % (
              len(shares), *np.percentile(shares, (5, 50, 95)),
              np.mean(shares >= 0.995), np.mean(shares >= 0.999),
              *np.percentile(gains, (5, 50, 95)),
              *np.percentile(covs, (5, 50, 95))))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
