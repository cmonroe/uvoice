# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Signal generators and filters of the canceller harness, on numpy alone.

Every signal is float64 on the int16 scale. 0 dBm0 is an RMS of
dbm0_to_rms(0), 3.14 dB under a full-scale sine (test/level.h,
dbm0_rms_get). Durations are in seconds (_s) or milliseconds (_ms), rates
in Hz, and an index counts samples from the first sample of the array.

The generators and their sources:

- css_single: the single talk composite source signal of ITU-T G.168
  Annex C.3.1. It is built at 44.1 kHz from the voiced signal of Table C.1
  and a pseudo-noise sequence shaped by Table C.2, then resampled. The PN
  phases come from a seed, so the sequence is not the one of ITU-T P.501.
- css_double: the double talk composite source signal of G.168 Annex
  C.3.2, from the voiced signal of Table C.3 and Gaussian noise shaped by
  Table C.2, with the seed of css_single.
- hybrid_model: an echo path model of G.168 Annex D, scaled by Table D.1a
  for a CSS input: m1 (Table D.2), m4 (Table D.5), m6 (Table D.7), m7
  (Table D.8) and m8 (Table D.9). hybrid_m1 is its m1 form.
- meter_power, meter_track: the level meter of G.168 clause 6.4.1.2.1,
  the 101-tap weighting filter of Table 1 at 8 kHz and a 35 ms one-pole
  filter.
- peak_power, peak_track: the peak level of G.168 clause 6.4.1.2.2, the
  same weighting and a causal 35 ms rectangular mean.
- hoth_noise: white Gaussian noise shaped by the Hoth room noise spectrum
  (IEEE 269, ITU-T P.800), held flat under 100 Hz. HOTH_DB holds the shape
  only, and it is not a certified table.
- band_noise: white Gaussian noise in the band of 300 to 3400 Hz, with the
  zero-phase shape of an order-4 Butterworth band-pass and peaks held at
  11 dB over the RMS: the band noise of G.168 Test 9 on the G.168 bench.
- uplink_down2: the 2:1 decimator of dsp/resamp.c, which takes the uplink of
  a 16 kHz line to an 8 kHz codec. dut_up2 is its interpolator, which
  raises the downlink of an 8 kHz codec to a 16 kHz line.
- dtmf: DTMF digits of ITU-T Q.23 (the 4 by 4 frequency grid) with
  raised-cosine edges.
- tone: a sum of sines, for the dial tone of 350 and 440 Hz of the
  GR-506-CORE precise tone plan.
- ulaw_encode, ulaw_decode: the mu-law of ITU-T G.711.

The filters repeat the arithmetic of scipy.signal (resample_poly, firwin,
kaiserord, lfilter) operation for operation, so their output matches that of
scipy to the bit, and scipy is not a dependency. The functions with a seed
draw from numpy PCG64 generators.
"""
import math

import numpy as np

DBM0_FULL_SCALE_DB = 3.14

CSS_FS = 44100
# Table C.1: 134 samples of the voiced signal at 44.1 kHz (3.04 ms).
CSS_VOICED = (
    -155, 276, 517, 578, 491, 302, 86, -103, -207, -198, -60, 190, 543,
    948, 1362, 1741, 2043, 2276, 2422, 2500, 2552, 2595, 2655, 2758, 2896,
    3060, 3224, 3370, 3500, 3569, 3603, 3603, 3595, 3586, 3595, 3638, 3724,
    3819, 3922, 4000, 4043, 4034, 3974, 3862, 3724, 3577, 3439, 3336, 3267,
    3224, 3198, 3172, 3129, 3043, 2914, 2750, 2560, 2353, 2155, 1991, 1853,
    1750, 1672, 1603, 1534, 1440, 1310, 1146, 965, 776, 603, 448, 345, 276,
    250, 250, 267, 267, 241, 190, 103, -9, -138, -267, -388, -491, -569,
    -638, -698, -759, -813, -888, -957, -1034, -1103, -1146, -1181, -1190,
    -1198, -1215, -1259, -1327, -1457, -1629, -1853, -2121, -2414, -2707,
    -3017, -3319, -3612, -3913, -4224, -4560, -4922, -5301, -5715, -6137,
    -6560, -6948, -7301, -7568, -7732, -7758, -7620, -7310, -6810, -6155,
    -5344, -4439, -3474, -2508, -1595, -802)
CSS_VOICED_REPEATS = 16
# Table C.2: corner frequencies and gains of the band limit of the PN
# sequence, linear in dB over log10 frequency, no response outside.
CSS_SHAPE_HZ = (50.0, 100.0, 200.0, 215.0, 500.0, 1000.0, 2850.0, 3600.0,
                3660.0, 3680.0)
CSS_SHAPE_DB = (-25.8, -12.8, 17.4, 17.8, 12.2, 7.2, 0.0, -2.0, -20.0,
                -30.0)
# C.3.1: an 8192-point spectrum, W(k) constant on the 3715 bins of 0 to
# 20 kHz, 200 ms emitted, crest factor 11 +- 1 dB.
CSS_PN_POINTS = 8192
CSS_PN_BINS = 3715
CSS_PN_W = 305360.0
CSS_PN_SAMPLES = 8820
CSS_PN_CREST_DB = 11.0
CSS_PN_CREST_TOL_DB = 1.0
CSS_PN_DRAWS = 64
# A half period of 350 ms at 44.1 kHz: voiced, PN, pause. The inverted copy
# follows it.
CSS_HALF = 15435
# Table C.3: 229 samples of the double talk voiced signal at 44.1 kHz
# (5.19 ms), read in columns. Value 168 is -938, as both editions of the
# table print it between -629 and -638.
CSS_DT_VOICED = (
    -198, -112, -9, 103, 233, 388, 543, 724, 896, 1060, 1233, 1388, 1517, 1638,
    1747, 1810, 1845, 1845, 1802, 1707, 1569, 1379, 1146, 871, 560, 233, -121,
    -491, -871, -1250, -1638, -2043, -2465, -2896, -3345, -3819, -4310, -4810,
    -5319, -5836, -6353, -6853, -7353, -7836, -8292, -8715, -9077, -9370,
    -9542, -9542, -9361, -8956, -8327, -7465, -6396, -5163, -3827, -2448,
    -1103, 155, 1293, 2241, 3034, 3655, 4138, 4517, 4827, 5094, 5344, 5594,
    5827, 6043, 6215, 6344, 6413, 6422, 6379, 6310, 6215, 6120, 6051, 6000,
    5991, 5991, 6000, 6008, 5991, 5939, 5853, 5715, 5560, 5387, 5215, 5043,
    4879, 4732, 4586, 4439, 4276, 4086, 3870, 3629, 3370, 3086, 2801, 2534,
    2267, 2034, 1819, 1612, 1422, 1224, 1026, 819, 603, 388, 181, 9, -181,
    -328, -448, -543, -629, -707, -784, -871, -948, -1026, -1112, -1181, -1241,
    -1276, -1293, -1302, -1293, -1267, -1250, -1233, -1224, -1224, -1224,
    -1224, -1215, -1198, -1172, -1129, -1077, -1026, -974, -922, -888, -871,
    -845, -828, -810, -793, -767, -741, -698, -672, -638, -603, -595, -586,
    -595, -603, -621, -629, -938, -638, -638, -638, -638, -638, -647, -664,
    -690, -724, -767, -793, -819, -845, -853, -871, -879, -888, -896, -922,
    -948, -974, -1009, -1026, -1052, -1069, -1077, -1069, -1060, -1060, -1052,
    -1043, -1043, -1052, -1060, -1060, -1060, -1052, -1034, -1017, -991, -957,
    -931, -905, -888, -862, -845, -819, -793, -767, -724, -672, -621, -560,
    -509, -457, -397, -345, -276, -207, -112)
CSS_DT_VOICED_REPEATS = 14
# C.3.2: 200 ms of white Gaussian noise shaped by Table C.2, crest factor
# 12 +- 1 dB. A half period of 400 ms at 44.1 kHz: voiced, noise, pause.
CSS_DT_NOISE_SAMPLES = 8820
CSS_DT_CREST_DB = 12.0
CSS_DT_HALF = 17640
# The resampler from 44.1 kHz: a Kaiser low-pass at 0.475 fs, a transition
# of 0.05 fs and 80 dB of design attenuation. Annex C asks for more than
# 60 dB of stopband and less than 0.2 dB of passband ripple.
CSS_STOP_DB = 80.0

# The echo path models of G.168 (12/2022) Annex D at 8 kHz, each table read
# in columns: m1(k) of Table D.2, k = 0 to 63.
M1_TAPS = (
    -436, -829, -2797, -4208, -17968, -11215, 46150, 34480, -10427, 9049,
    -1309, -6320, 390, -8191, -1751, -6051, -3796, -4055, -3948, -2557,
    -3372, -1808, -2259, -1300, -1098, -618, -340, -61, 323, 419, 745, 716,
    946, 880, 1014, 976, 1033, 1091, 1053, 1042, 794, 831, 899, 716, 390,
    313, 304, 304, 73, -119, -109, -176, -359, -407, -512, -580, -704, -618,
    -685, -791, -772, -820, -839, -724)
# m4(k) of Table D.5, k = 0 to 127.
M4_TAPS = (
    160, 312, -241, -415, 897, 908, -1326, -1499, 2405, 3347, -3624, -7733,
    4041, 14484, -1477, -21739, -4470, 25356, 11458, -19696, -11800, 5766, 789,
    6633, 14624, -6975, -17156, -187, 149, 1515, 14907, 4345, -7128, -2757,
    -10185, -7083, 6850, 3944, 6969, 8694, -4068, -3852, -5793, -9371, 453,
    1060, 3965, 9463, 2393, 2784, -892, -7366, -3376, -5847, -2399, 3011, 1537,
    6623, 4205, 1602, 1592, -4752, -3646, -5207, -5577, -501, -1174, 4041,
    5647, 4628, 7252, 2123, 2654, -881, -4113, -3244, -7289, -3830, -4600,
    -2508, 431, -144, 4184, 2372, 4617, 3576, 2382, 2839, -404, 539, -1803,
    -1401, -1705, -2269, -783, -1608, -220, -306, 257, 615, 225, 561, 8, 344,
    127, -57, 182, 41, 203, -111, 95, -79, 30, 84, -13, -68, -241, -68, -24,
    19, -57, -24, 30, -68, 84, -155, -68, 19)
# m6(k) of Table D.7, k = 0 to 119.
M6_TAPS = (
    29, 109, -83, 198, -294, -135, -415, -202, -444, -337, -313, -450, -105,
    -503, 145, -490, 267, -231, 340, 77, 343, 783, 158, 1341, 195, 1798, 344,
    1845, 629, 1604, 1182, 940, 5163, 19522, 8421, -50953, -9043, 18046,
    -13553, 13336, -3471, -107, 1788, -7409, 2469, -7994, 490, -3860, -837,
    490, -636, 3682, 1141, 5019, 2635, 5025, 3946, 4414, 4026, 3005, 3380,
    1616, 2007, 158, 388, -1198, -1117, -2134, -2547, -2589, -3310, -2778,
    -3427, -2779, -3116, -2502, -2399, -1956, -1539, -1239, -570, -377, 251,
    331, 964, 1177, 1449, 1564, 1724, 1871, 1767, 1802, 1630, 1632, 1379, 1271,
    1063, 856, 711, 482, 289, 54, -137, -321, -490, -638, -764, -836, -800,
    -859, -838, -837, -834, -740, -673, -581, -493, -436, -327, -201)
# m7(k) of Table D.8, k = 0 to 95.
M7_TAPS = (
    258, -111, 337, -319, 347, -434, 192, -450, -108, -343, -596, -177, -1187,
    -52, -1781, -147, -1959, -326, -1601, -1389, -13620, -720, 33818, -10683,
    -6742, 12489, -9862, 8950, -1574, 758, 3526, -3118, 2421, -8966, -4901,
    11385, 18072, -14410, -7473, 19836, -16854, -3115, 9483, -17799, 7399,
    -4342, -7415, 7929, -10726, 6239, -2526, -1317, 5345, -4565, 6868, -2195,
    3425, 1969, -109, 3963, -1275, 3087, -892, 1239, 2, -427, 596, -1184, 551,
    -1244, 141, -743, -415, -372, -769, -183, -785, -270, -659, -377, -523,
    -325, -245, -255, -60, 35, 218, 149, 340, 233, 365, 303, 251, 230, 209,
    179)
# m8(k) of Table D.9, k = 0 to 98.
M8_TAPS = (
    80, 31, 4, 42, 42, -61, -81, -64, -121, -102, -26, 1002, -9250, -22562,
    39321, 35681, -35289, 25312, -1457, -229, 15659, -6786, 16791, 3860, 2239,
    -28730, -11885, 33871, -176, -16421, 18173, -9669, -10163, 9941, -19365,
    3592, -5907, -10257, 5336, -12933, 4348, -4802, -1791, 3035, -4433, 5553,
    -2596, 3992, 1255, 1450, 4079, 324, 4340, 1059, 3083, 1917, 1756, 2478,
    1027, 1871, 845, 1284, 813, 806, 869, 471, 646, 438, 449, 432, 473, 394,
    452, 538, 717, 723, 850, 756, 753, 899, 555, 669, 619, 500, 650, 615, 516,
    492, 427, 291, 356, 147, 107, -50, -88, -59, -238, -165, -183)
# Table D.1a: the scaling factor Ki of each model for a CSS input, and the
# minimum ERL in dB that keeps the scaled response at or under 0 dB.
ANNEX_D = {
    'm1': (M1_TAPS, 1.39e-5, 6.0),
    'm4': (M4_TAPS, 1.77e-5, 6.0),
    'm6': (M6_TAPS, 1.51e-5, 6.0),
    'm7': (M7_TAPS, 2.33e-5, 11.06),
    'm8': (M8_TAPS, 1.33e-5, 9.27),
}
M1_KI_CSS = ANNEX_D['m1'][1]
M1_ERL_MIN_DB = ANNEX_D['m1'][2]
# Zero padding of a model before a rate change, so the interpolation keeps
# its ringing: m(0) then falls 1 ms after the start of the response.
M1_PAD_MS = 1.0

METER_FS = 8000
# Table 1: f0 to f50 of the symmetric 101-tap weighting filter at 8 kHz.
METER_HALF = (
    0.0000, 0.0006, 0.0005, 0.0004, 0.0011, -0.0000, 0.0015, -0.0003,
    0.0012, -0.0002, 0.0000, 0.0002, -0.0020, 0.0005, -0.0040, 0.0000,
    -0.0047, -0.0019, -0.0033, -0.0047, -0.0000, -0.0068, 0.0036, -0.0057,
    0.0054, 0.0000, 0.0044, 0.0095, 0.0017, 0.0188, 0.0000, 0.0225, 0.0024,
    0.0163, 0.0092, 0.0000, 0.0164, -0.0210, 0.0161, -0.0375, 0.0000,
    -0.0406, -0.0357, -0.0267, -0.0871, -0.0000, -0.1420, 0.0289, -0.1843,
    0.0475, 0.8006)
METER_TAU_MS = 35.0
PEAK_WINDOW_MS = 35.0
# The output step of meter_track in input samples, and the reading of a
# power under the square of METER_RMS_MIN.
METER_HOP = 8
METER_RMS_MIN = 1e-6
METER_FLOOR_DBM0 = -200.0

# The Hoth spectrum level in dB at the third-octave centers, 100 Hz to
# 8 kHz. Only the shape matters: the generator scales its output.
HOTH_HZ = (100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
           1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000)
HOTH_DB = (32.4, 30.9, 29.1, 27.6, 26.0, 24.4, 22.7, 21.1, 19.5, 17.8,
           16.2, 14.6, 12.9, 11.3, 9.6, 7.8, 5.4, 2.6, -1.3, -6.6)

# Q.23: the row (low group) and column (high group) frequencies, and the
# keypad that maps a digit to one of each.
DTMF_ROWS = (697.0, 770.0, 852.0, 941.0)
DTMF_COLS = (1209.0, 1336.0, 1477.0, 1633.0)
DTMF_KEYS = ('123A', '456B', '789C', '*0#D')
DTMF_FREQS = {k: (DTMF_ROWS[r], DTMF_COLS[c])
              for r, row in enumerate(DTMF_KEYS) for c, k in enumerate(row)}

BAND_HZ = (300.0, 3400.0)
BAND_ORDER = 4
BAND_CREST_DB = 11.0
# resamp_coeff_q15 of dsp/resamp.c: the 47-tap half-band low-pass, Q15, with
# the gain of 2 of the interpolator folded in. udsp_resamp_down2 halves each
# tap and takes 16384 for the center.
RESAMP_Q15 = (-54, 0, 98, 0, -184, 0, 310, 0, -492, 0, 750, 0, -1112, 0,
              1632, 0, -2416, 0, 3752, 0, -6694, 0, 20774, 32767, 20774, 0,
              -6694, 0, 3752, 0, -2416, 0, 1632, 0, -1112, 0, 750, 0, -492,
              0, 310, 0, -184, 0, 98, 0, -54)

ULAW_BIAS = 0x84
ULAW_CLIP = 32635

# The default low-pass of a rate change by an integer factor: a Kaiser
# window of beta 5 over 10 zero crossings of the sinc on each side.
RESAMPLE_BETA = 5.0
RESAMPLE_ZEROS = 10
# Products held in memory at once by resample_poly.
RESAMPLE_CHUNK = 1 << 20


def dbm0_to_rms(level_dbm0):
    """RMS on the int16 scale of a signal at level_dbm0."""
    return (32767.0 * 10.0 ** (-DBM0_FULL_SCALE_DB / 20.0) / math.sqrt(2.0) *
            10.0 ** (level_dbm0 / 20.0))


def amp_for_dbm0(level_dbm0):
    """Peak amplitude on the int16 scale of a sine at level_dbm0."""
    return dbm0_to_rms(level_dbm0) * math.sqrt(2.0)


def to_int16(x):
    """x rounded half to even and saturated to int16, little-endian."""
    return np.clip(np.round(x), -32768, 32767).astype('<i2')


# ------------------------------------------------------------ filters

def fir(taps, x):
    """Causal FIR filter: the first len(x) samples of the convolution."""
    x = np.asarray(x, dtype=float)
    return np.convolve(x, np.asarray(taps, dtype=float))[:len(x)]


def one_pole(b, a, x):
    """y[n] = b x[n] + a y[n - 1], from rest. The order of the operations is
    that of a direct form II transposed section, so the output is exact to
    the bit against such a filter."""
    y = np.empty(len(x))
    acc = 0.0
    for i, v in enumerate(np.asarray(x, dtype=float).tolist()):
        acc = b * v + a * acc
        y[i] = acc
    return y


def kaiser(n, beta):
    """Symmetric Kaiser window of n points."""
    return np.kaiser(n, beta)


def kaiser_order(atten_db, width):
    """Taps and beta of a Kaiser low-pass for atten_db of attenuation and a
    transition width as a fraction of the Nyquist rate (Oppenheim and
    Schafer, Discrete-Time Signal Processing, the Kaiser formulas)."""
    if atten_db > 50.0:
        beta = 0.1102 * (atten_db - 8.7)
    elif atten_db > 21.0:
        beta = (0.5842 * (atten_db - 21.0) ** 0.4 +
                0.07886 * (atten_db - 21.0))
    else:
        beta = 0.0
    taps = (atten_db - 7.95) / 2.285 / (math.pi * width) + 1.0
    return int(math.ceil(taps)), beta


def lowpass(taps, cutoff, beta):
    """Windowed-sinc low-pass of taps points with unit gain at DC. cutoff is
    a fraction of the Nyquist rate."""
    m = np.arange(0, taps, dtype=float) - 0.5 * (taps - 1)
    h = cutoff * np.sinc(cutoff * m)
    h *= kaiser(taps, beta)
    return h / np.sum(h)


def _resample_offsets(h, up, down):
    """The output offset of a polyphase rate change: the index into the
    full convolution of the zero-stuffed input of output sample 0, so the
    output stands centered on the filter."""
    half = (len(h) - 1) // 2
    pre_pad = down - half % down
    pre_remove = (half + pre_pad) // down
    return pre_remove * down - pre_pad


def resample_poly(x, up, down, h=None, first=0, count=None):
    """Rate change of x by up / down through the low-pass h, zero-phase.

    h defaults to the window-method filter of RESAMPLE_BETA over
    RESAMPLE_ZEROS zero crossings. The function scales h by up. The output
    holds ceil(len(x) up / down) samples; first and count select a part of
    it. Each output sample sums its products from the oldest input sample to
    the newest, in sequence, the order of a polyphase FIR."""
    x = np.asarray(x, dtype=float)
    if h is None:
        rate = max(up, down)
        h = lowpass(2 * RESAMPLE_ZEROS * rate + 1, 1.0 / rate,
                    RESAMPLE_BETA)
    h = np.asarray(h, dtype=float) * up
    n_out = -(-len(x) * up // down)
    count = n_out - first if count is None else count
    base = _resample_offsets(h, up, down)
    phase_taps = -(-len(h) // up)
    order = np.arange(phase_taps)[::-1]
    chunk = max(1, RESAMPLE_CHUNK // phase_taps)
    y = np.empty(count)
    for c0 in range(0, count, chunk):
        j = base + (np.arange(c0, min(count, c0 + chunk)) + first) * down
        k = (j % up)[:, None] + up * order[None, :]
        i = (j[:, None] - k) // up
        ok = (k < len(h)) & (i >= 0) & (i < len(x))
        prod = (np.where(ok, h[np.minimum(k, len(h) - 1)], 0.0) *
                np.where(ok, x[np.clip(i, 0, len(x) - 1)], 0.0))
        y[c0:c0 + len(j)] = np.cumsum(prod, axis=1)[:, -1]
    return y


def upsample2(x):
    """x at twice its rate, through the default low-pass of resample_poly."""
    return resample_poly(x, 2, 1)


def uplink_down2(x):
    """x at half its rate through the decimator of udsp_resamp_down2 in
    dsp/resamp.c: the half-band low-pass of RESAMP_Q15 at unit gain, then
    every second sample, in floating point. Output sample j reads input
    samples up to 2 j + 1, as the decimator emits after each second input.
    A white input loses 3.16 dB, and a signal under 3.4 kHz none."""
    h = np.array(RESAMP_Q15, dtype=float) / 65536.0
    h[len(h) // 2] = 16384.0 / 32768.0
    return fir(h, x)[1::2]


def dut_up2(x):
    """x at twice its rate through the interpolator of udsp_resamp_up2 in
    dsp/resamp.c, in floating point: x stuffed with a zero after each
    sample, through the causal FIR of RESAMP_Q15 with its gain of 2. The
    even outputs take the symmetric phase and the odd outputs the pure
    delay of 23 samples, as the interpolator gives them."""
    z = np.zeros(2 * len(x))
    z[0::2] = np.asarray(x, dtype=float)
    return fir(np.array(RESAMP_Q15, dtype=float) / 32768.0, z)


def correlate_valid(a, v):
    """Cross-correlation of a with the shorter v at every lag where v lies
    inside a: c[k] = sum v[n] a[n + k]."""
    a = np.asarray(a, dtype=float)
    v = np.asarray(v, dtype=float)
    n = len(a) + len(v) - 1
    size = 1 << (n - 1).bit_length()
    c = np.fft.irfft(np.fft.rfft(a, size) * np.conj(np.fft.rfft(v, size)),
                     size)
    return c[:len(a) - len(v) + 1]


def fftconvolve(a, b):
    """Full linear convolution of a and b through the FFT."""
    n = len(a) + len(b) - 1
    size = 1 << (n - 1).bit_length()
    return np.fft.irfft(np.fft.rfft(a, size) * np.fft.rfft(b, size),
                        size)[:n]


def solve_toeplitz(r, b):
    """Solution of T x = b for the symmetric Toeplitz matrix T whose first
    column is r."""
    r = np.asarray(r, dtype=float)
    n = len(r)
    idx = np.abs(np.arange(n)[:, None] - np.arange(n)[None, :])
    return np.linalg.solve(r[idx], np.asarray(b, dtype=float))


# ------------------------------------------------------------ G.711

def ulaw_encode(x):
    """int16-scale samples to G.711 mu-law codes."""
    x = np.clip(np.round(x), -32768, 32767).astype(np.int32)
    sign = np.where(x < 0, 0x80, 0)
    mag = np.minimum(np.abs(x), ULAW_CLIP) + ULAW_BIAS
    exp = np.zeros_like(mag)
    for e in range(7, 0, -1):
        exp = np.where((exp == 0) & (mag >= (1 << (e + 7))), e, exp)
    mant = (mag >> (exp + 3)) & 0x0F
    return (~(sign | (exp << 4) | mant)) & 0xFF


def ulaw_decode(u):
    """G.711 mu-law codes to int16-scale samples as float64."""
    u = (~np.asarray(u)) & 0xFF
    sign = u & 0x80
    exp = (u >> 4) & 7
    mant = u & 0x0F
    mag = ((mant << 3) + ULAW_BIAS) << exp
    return np.where(sign, ULAW_BIAS - mag, mag - ULAW_BIAS).astype(float)


# ------------------------------------------------------------ G.168

def css_period_get(fs=8000):
    """Samples of one single talk CSS period (700 ms) at fs."""
    n = 2 * CSS_HALF * fs
    if n % CSS_FS:
        raise ValueError(f'fs={fs} gives no whole CSS period')
    return n // CSS_FS


def _css_shape(freq_hz):
    """Table C.2 amplitude gain at freq_hz."""
    f = np.asarray(freq_hz, dtype=float)
    out = np.zeros(f.shape)
    inside = (f >= CSS_SHAPE_HZ[0]) & (f <= CSS_SHAPE_HZ[-1])
    out[inside] = 10.0 ** (np.interp(np.log10(f[inside]),
                                     np.log10(CSS_SHAPE_HZ),
                                     CSS_SHAPE_DB) / 20.0)
    return out


def _crest_db(x):
    return 20.0 * math.log10(float(np.max(np.abs(x))) /
                             float(np.sqrt(np.mean(x * x))))


def css_pn(seed):
    """The C.3.1 PN sequence at 44.1 kHz: CSS_PN_SAMPLES samples.

    Each bin of the spectrum gets the phase 0 or pi from a PCG64 generator
    seeded with (seed, 1, draw). Draws repeat until the crest factor of the
    emitted sequence lies in CSS_PN_CREST_DB +- CSS_PN_CREST_TOL_DB."""
    shape = _css_shape(np.arange(CSS_PN_POINTS // 2 + 1) * CSS_FS /
                       CSS_PN_POINTS)
    for k in range(CSS_PN_DRAWS):
        bits = np.random.default_rng([seed, 1, k]).integers(0, 2,
                                                            CSS_PN_BINS)
        spectrum = np.zeros(CSS_PN_POINTS // 2 + 1)
        spectrum[1:CSS_PN_BINS + 1] = np.where(bits == 1, -1.0, 1.0) * \
            CSS_PN_W
        x = np.resize(np.fft.irfft(spectrum * shape, CSS_PN_POINTS),
                      CSS_PN_SAMPLES)
        if abs(_crest_db(x) - CSS_PN_CREST_DB) <= CSS_PN_CREST_TOL_DB:
            return x
    raise RuntimeError('no PN draw met the crest factor of C.3.1')


def css_resampler(fs):
    """The low-pass of the rate change from 44.1 kHz to fs, designed at the
    common rate CSS_FS times the numerator of fs / CSS_FS."""
    up = fs // math.gcd(fs, CSS_FS)
    rate = CSS_FS * up
    width = 0.05 * fs / (rate / 2.0)
    taps, beta = kaiser_order(CSS_STOP_DB, width)
    return lowpass(taps | 1, 0.475 * fs / (rate / 2.0), beta)


def css_period(fs=8000, seed=0):
    """One single talk CSS period at fs, as a periodic signal: the voiced
    burst, the PN at the voiced RMS, the pause and the inverted copy."""
    voiced = np.tile(np.array(CSS_VOICED, dtype=float), CSS_VOICED_REPEATS)
    noise = css_pn(seed)
    noise *= np.sqrt(np.mean(voiced ** 2) / np.mean(noise ** 2))
    half = np.concatenate((voiced, noise,
                           np.zeros(CSS_HALF - len(voiced) - len(noise))))
    period = np.concatenate((half, -half))
    g = math.gcd(fs, CSS_FS)
    n = css_period_get(fs)
    return resample_poly(np.tile(period, 3), fs // g, CSS_FS // g,
                         h=css_resampler(fs), first=n, count=n)


def css_single(level_dbm0=-10.0, dur_s=7.0, fs=8000, seed=0):
    """Single talk CSS of G.168 C.3.1 at fs for dur_s from a period start.
    The RMS over whole periods is level_dbm0."""
    period = css_period(fs, seed)
    scaled = period * (dbm0_to_rms(level_dbm0) /
                       np.sqrt(np.mean(period ** 2)))
    n = int(round(dur_s * fs))
    return np.tile(scaled, max(1, -(-n // len(scaled))))[:n]


def css_dt_period_get(fs=8000):
    """Samples of one double talk CSS period (800 ms) at fs."""
    n = 2 * CSS_DT_HALF * fs
    if n % CSS_FS:
        raise ValueError(f'fs={fs} gives no whole CSS period')
    return n // CSS_FS


def css_dt_noise(seed):
    """The C.3.2 noise at 44.1 kHz: CSS_DT_NOISE_SAMPLES samples of white
    Gaussian noise from a PCG64 generator seeded with (seed, 2, draw), shaped
    by Table C.2 with a circular filter in the frequency domain. Draws
    repeat until the crest factor lies in CSS_DT_CREST_DB +-
    CSS_PN_CREST_TOL_DB."""
    n = CSS_DT_NOISE_SAMPLES
    shape = _css_shape(np.fft.rfftfreq(n, 1.0 / CSS_FS))
    for k in range(CSS_PN_DRAWS):
        white = np.random.default_rng([seed, 2, k]).standard_normal(n)
        x = np.fft.irfft(np.fft.rfft(white) * shape, n)
        if abs(_crest_db(x) - CSS_DT_CREST_DB) <= CSS_PN_CREST_TOL_DB:
            return x
    raise RuntimeError('no noise draw met the crest factor of C.3.2')


def css_dt_period(fs=8000, seed=0):
    """One double talk CSS period at fs, as a periodic signal: the voiced
    burst of Table C.3, the noise at the voiced RMS, the pause and the
    inverted copy."""
    voiced = np.tile(np.array(CSS_DT_VOICED, dtype=float),
                     CSS_DT_VOICED_REPEATS)
    noise = css_dt_noise(seed)
    noise *= np.sqrt(np.mean(voiced ** 2) / np.mean(noise ** 2))
    half = np.concatenate((voiced, noise,
                           np.zeros(CSS_DT_HALF - len(voiced) - len(noise))))
    period = np.concatenate((half, -half))
    g = math.gcd(fs, CSS_FS)
    n = css_dt_period_get(fs)
    return resample_poly(np.tile(period, 3), fs // g, CSS_FS // g,
                         h=css_resampler(fs), first=n, count=n)


def css_double(level_dbm0=-10.0, dur_s=8.0, fs=8000, seed=0):
    """Double talk CSS of G.168 C.3.2 at fs for dur_s from a period start.
    The RMS over whole periods is level_dbm0."""
    period = css_dt_period(fs, seed)
    scaled = period * (dbm0_to_rms(level_dbm0) /
                       np.sqrt(np.mean(period ** 2)))
    n = int(round(dur_s * fs))
    return np.tile(scaled, max(1, -(-n // len(scaled))))[:n]


def hybrid_model(name, erl_db=6.0, fs=8000):
    """Echo path model name ('m1', 'm4', 'm6', 'm7' or 'm8') of Annex D at
    erl_db for a CSS input, at fs. At 16 kHz the taps are a band-limited
    interpolation of the 8 kHz taps, and m(0) falls 1 ms after the start of
    the response (M1_PAD_MS)."""
    taps, ki, erl_min = ANNEX_D[name]
    if erl_db < erl_min - 1e-9:
        raise ValueError(f'{name} needs an ERL of {erl_min} dB or more')
    m = np.array(taps, dtype=float)
    if fs != METER_FS:
        g = math.gcd(fs, METER_FS)
        pad = int(round(M1_PAD_MS * METER_FS / 1000.0))
        m = resample_poly(np.pad(m, pad), fs // g, METER_FS // g) * \
            (METER_FS / fs)
    return ki * 10.0 ** (-erl_db / 20.0) * m


def hybrid_m1(erl_db=6.0, fs=8000):
    """Echo path model m1 of Annex D at erl_db for a CSS input, at fs."""
    return hybrid_model('m1', erl_db, fs)


def meter_taps():
    """The 101 taps of the Table 1 weighting filter."""
    return np.array(METER_HALF + METER_HALF[-2::-1])


def meter_power(x):
    """Per-sample mean-square output of the clause 6.4.1.2.1 level meter on
    8 kHz input (int16 scale squared). The meter is causal."""
    w = fir(meter_taps(), x)
    a = math.exp(-1.0 / (METER_TAU_MS * METER_FS / 1000.0))
    return one_pole(1.0 - a, a, w * w)


def peak_power(x):
    """Per-sample mean square of the clause 6.4.1.2.2 peak level on 8 kHz
    input (int16 scale squared): the Table 1 weighting, the square and a
    causal rectangular mean over PEAK_WINDOW_MS. The window holds zeros
    before the first sample."""
    w = fir(meter_taps(), x)
    w2 = w * w
    win = max(1, round(PEAK_WINDOW_MS * METER_FS / 1000.0))
    c = np.concatenate((np.zeros(win + 1), np.cumsum(w2)))
    return (c[win + 1:] - c[1:len(w2) + 1]) / win


def meter_track(x, fs=8000, hop=METER_HOP):
    """(idx, level_dbm0) of the clause 6.4.1.2.1 meter on x at fs: the
    input sample indexes hop - 1, 2 hop - 1 and so on, and the meter level
    there in dBm0. Input at 16 kHz goes to 8 kHz through the zero-phase
    resample_poly, and the 8 kHz power returns to the input clock by linear
    interpolation, so a reading there sees a few samples ahead. A power
    under the square of METER_RMS_MIN reads METER_FLOOR_DBM0. This is the
    reading of LevelMeter.track in the toolkit of the G.168 bench."""
    return _track(x, fs, hop, meter_power)


def peak_track(x, fs=8000, hop=METER_HOP):
    """(idx, level_dbm0) of the clause 6.4.1.2.2 peak level on x at fs, at
    the indexes and by the rate rule of meter_track. This is the reading of
    peak_meter in the toolkit of the G.168 bench."""
    return _track(x, fs, hop, peak_power)


def _track(x, fs, hop, power):
    """The readings of meter_track for the 8 kHz power function power."""
    x = np.asarray(x, dtype=float)
    x8 = x if fs == METER_FS else resample_poly(x, METER_FS // math.gcd(
        fs, METER_FS), fs // math.gcd(fs, METER_FS))
    p8 = power(x8)
    p = p8 if fs == METER_FS else np.interp(
        np.arange(len(x)) * METER_FS / fs, np.arange(len(p8)), p8)
    idx = np.arange(hop - 1, len(p), hop)
    rms = np.sqrt(np.maximum(p[idx], 0.0))
    level = np.where(rms < METER_RMS_MIN, METER_FLOOR_DBM0, 20.0 * np.log10(
        np.maximum(rms, METER_RMS_MIN) / dbm0_to_rms(0.0)))
    return idx, level


# ------------------------------------------------------------ other sources

def _s_curve(u):
    """Raised-cosine step: 0 at u <= -0.5, 1 at u >= 0.5, 0.5 at 0."""
    u = np.clip(u, -0.5, 0.5)
    return 0.5 + 0.5 * np.sin(np.pi * u)


def _sines(freqs, levels_dbm0, n, fs, phases):
    k = np.arange(n)
    out = np.zeros(n)
    for f, lv, ph in zip(freqs, levels_dbm0, phases):
        out += amp_for_dbm0(lv) * np.sin(2.0 * np.pi * f * k / fs + ph)
    return out


def tone(freqs, levels_dbm0, dur_s, fs=8000, ramp_ms=1.0):
    """Sum of sines for dur_s, each at its level in dBm0 and phase 0, with
    raised-cosine ramps of ramp_ms inside the duration."""
    fl = [float(freqs)] if np.isscalar(freqs) else [float(f) for f in freqs]
    lv = ([float(levels_dbm0)] * len(fl) if np.isscalar(levels_dbm0)
          else [float(v) for v in levels_dbm0])
    n = int(round(dur_s * fs))
    x = _sines(fl, lv, n, fs, [0.0] * len(fl))
    r = ramp_ms * fs / 1000.0
    if r >= 1.0 and n > 0:
        k = np.arange(n, dtype=float)
        x *= _s_curve((k - r / 2) / r) * \
            (1.0 - _s_curve((k - (n - 1 - r / 2)) / r))
    return x


def _gate(n, start, end, ramp):
    """Envelope of n samples: 1 from start to end, with raised-cosine edges
    of ramp samples centered on both, so each edge is a half-amplitude
    point."""
    env = np.zeros(n)
    r = max(float(ramp), 1e-9)
    lo = max(0, int(np.floor(start - r / 2)))
    hi = min(n, int(np.ceil(end + r / 2)) + 1)
    if hi > lo:
        k = np.arange(lo, hi, dtype=float)
        env[lo:hi] = _s_curve((k - start) / r) * (1.0 - _s_curve((k - end) /
                                                                 r))
    return env


def dtmf(digits, on_ms=60.0, off_ms=60.0, level_dbm0=-10.0, fs=8000,
         seed=None, ramp_ms=1.0):
    """A DTMF digit sequence of Q.23, and its schedule.

    Each digit is on for on_ms and off for off_ms. Both tones stand at
    level_dbm0. Each digit takes two start phases from a PCG64 generator
    seeded with seed. The schedule holds one dict per digit: digit, on and
    off (the sample indexes of the half-amplitude edges), f_lo and f_hi in
    Hz. The array starts with ceil(ramp / 2) lead samples."""
    rng = np.random.default_rng(seed)
    r = ramp_ms * fs / 1000.0
    lead = int(math.ceil(r / 2.0))
    step = int(round((on_ms + off_ms) / 1000.0 * fs))
    n_on = int(round(on_ms / 1000.0 * fs))
    total = lead + step * len(digits) + lead
    x = np.zeros(total)
    sched = []
    pos = lead
    for d in digits.upper():
        f_lo, f_hi = DTMF_FREQS[d]
        ph = list(rng.uniform(0, 2 * np.pi, 2))
        lo, hi = max(0, pos - lead - 1), min(total, pos + n_on + lead + 1)
        seg = _sines([f_lo, f_hi], [level_dbm0, level_dbm0], hi - lo, fs, ph)
        x[lo:hi] += seg * _gate(hi - lo, pos - lo, pos + n_on - lo, r)
        sched.append(dict(digit=d, on=pos, off=pos + n_on, f_lo=f_lo,
                          f_hi=f_hi))
        pos += step
    return x, sched


def hoth_noise(level_dbm0, dur_s, fs=8000, seed=None, band_hz=None):
    """Hoth room noise at exactly level_dbm0 RMS.

    White Gaussian noise from a PCG64 generator seeded with seed, shaped in
    the frequency domain by HOTH_DB, interpolated in log frequency. band_hz
    (low, high) keeps that band alone before the scaling."""
    n = int(round(dur_s * fs))
    if n == 0:
        return np.zeros(0)
    spec = np.fft.rfft(np.random.default_rng(seed).standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    lf = np.log10(np.clip(f, HOTH_HZ[0], HOTH_HZ[-1]))
    g_db = np.interp(lf, np.log10(np.array(HOTH_HZ, dtype=float)), HOTH_DB)
    shaped = spec * 10.0 ** ((g_db - g_db.max()) / 20.0)
    if band_hz is not None:
        shaped = shaped * ((f >= band_hz[0]) & (f <= band_hz[1]))
    x = np.fft.irfft(shaped, n)
    return x * (dbm0_to_rms(level_dbm0) / max(np.sqrt(np.mean(x * x)),
                                              1e-12))


def band_noise(level_dbm0, dur_s, fs=8000, seed=None, band_hz=BAND_HZ):
    """Band noise at exactly level_dbm0 RMS.

    White Gaussian noise from a PCG64 generator seeded with seed, shaped in
    the frequency domain by the zero-phase response of an order BAND_ORDER
    Butterworth band-pass over band_hz: the magnitude of one pass squared,
    as a forward and a backward pass give, with the edges prewarped for the
    bilinear transform. Peaks are then held at BAND_CREST_DB over the RMS."""
    n = int(round(dur_s * fs))
    if n == 0:
        return np.zeros(0)
    spec = np.fft.rfft(np.random.default_rng(seed).standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    warp = 2.0 * fs * np.tan(np.pi * np.minimum(f, 0.4999 * fs) / fs)
    w1, w2 = (2.0 * fs * math.tan(math.pi * e / fs) for e in band_hz)
    with np.errstate(divide='ignore'):
        u = (warp ** 2 - w1 * w2) / (warp * (w2 - w1))
    shape = np.where(f > 0, 1.0 / (1.0 + np.nan_to_num(u, nan=0.0,
                                                       neginf=-1e30) **
                                   (2 * BAND_ORDER)), 0.0)
    y = np.fft.irfft(spec * shape, n)
    peak = 10.0 ** (BAND_CREST_DB / 20.0) * math.sqrt(float(np.mean(y * y)))
    y = np.clip(y, -peak, peak)
    return y * (dbm0_to_rms(level_dbm0) / max(math.sqrt(float(np.mean(
        y * y))), 1e-12))


def hoth_metered(level_dbm0, dur_s, seed, band_hz=(300.0, 3400.0)):
    """Hoth noise at 8 kHz whose mean meter_power reads level_dbm0."""
    x = hoth_noise(level_dbm0, dur_s, METER_FS, seed=seed, band_hz=band_hz)
    metered = float(np.mean(meter_power(x)))
    return x * dbm0_to_rms(level_dbm0) / max(math.sqrt(metered), 1e-12)


def tone_power(x, f, rate):
    """The power of x at f Hz over all of x: a single-bin DFT, scaled so a
    sine of amplitude A on the bin reads A^2 / 4."""
    n = np.arange(len(x))
    return abs(np.dot(x, np.exp(-2j * np.pi * f * n / rate))) ** 2 / \
        len(x) ** 2


def two_peak_share(x, rate):
    """Share of the 200 to 3400 Hz energy of a frame in its two strongest
    spectral peaks, each with two bins on each side: the measure of
    ref_tone_share_get in dsp/lec.c. The frame takes a Hann window and zero
    padding to 128 points at 8 kHz and 256 at 16 kHz."""
    x = np.asarray(x, dtype=float)
    n = 128
    while n < len(x):
        n *= 2
    p = np.abs(np.fft.fft(x * np.hanning(len(x)), n)) ** 2
    lo = -(-200 * n // rate)
    hi = 3400 * n // rate
    p = p[lo:hi + 1].copy()
    tot = p.sum()
    if tot <= 0:
        return 0.0
    top = 0.0
    for _ in range(2):
        k = int(np.argmax(p))
        a, z = max(0, k - 2), min(len(p), k + 3)
        top += p[a:z].sum()
        p[a:z] = 0
    return float(top / tot)
