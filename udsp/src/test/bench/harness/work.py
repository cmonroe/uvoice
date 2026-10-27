# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""Directories and programs of the canceller harness.

Two directories hold everything that the harness writes:

- the build directory: the CMake build of udsp/src with UDSP_BENCH on. The
  environment variable UDSP_HARNESS_BUILD sets it; the default is
  udsp/build of this tree;
- the work directory: UDSP_HARNESS_WORK, by default harness/ in the build
  directory.

A relative path in either variable counts from the current directory.

The work directory holds:

- var/<name>/lec.c: a canceller variant (mkvar.py);
- b/<name>/: the harness build of a variant (lec_harness.py build);
- stim/<set>/<case>/: the stimuli of a set;
- run/<set>/<build>/<case>/: a run of lec_hz;
- sum/<tag>/<build>/<case>.json: the summary of a run (lec_harness.py run
  --summary);
- fld/<set>/<build>/<id>.csv, .txt and .coh: the replays of a capture set,
  and fld/<set>/index.json, the capture file of each id;
- resid/<set>/<build>/<id>.wav: residuals of lec_replay --residual;
- benchsig/<id>.npz: the bench signals that place.py reads;
- tmp/: the stimuli of place.py and synplace.py while they run;
- test/, sweep/, bench/ and legs/: the outputs of the commands test,
  sweep, bench and leg of lec_harness.py.

A build name selects the programs. The name tree selects the programs of
the build directory: lec_hz, lec_replay_coh and lec_tone_sweep in
test/bench/harness/, lec_replay, test_lec and bench_lec at its top. Any
other name selects b/<name>/ of the work directory.
"""
import concurrent.futures
import csv
import json
import math
import os
import pathlib
import re
import subprocess
import sys

HARNESS = pathlib.Path(__file__).resolve().parent
UDSP_SRC = HARNESS.parents[2]
UDSP = UDSP_SRC.parent
TREE = 'tree'
TOOLS_IN_HARNESS = ('lec_hz', 'lec_replay_coh', 'lec_tone_sweep')


def build_dir():
    """The CMake build directory, as an absolute path. A relative
    UDSP_HARNESS_BUILD counts from the current directory."""
    return pathlib.Path(os.environ.get('UDSP_HARNESS_BUILD',
                                       UDSP / 'build')).resolve()


def work_dir():
    """The work directory, as an absolute path. A relative UDSP_HARNESS_WORK
    counts from the current directory."""
    w = os.environ.get('UDSP_HARNESS_WORK')
    return pathlib.Path(w).resolve() if w else build_dir() / 'harness'


def program(build, name):
    """The path of program name of build."""
    if build == TREE:
        b = build_dir()
        return b / 'test/bench/harness' / name if name in \
            TOOLS_IN_HARNESS else b / name
    return work_dir() / 'b' / build / name


def stim_dir(set_name):
    return work_dir() / 'stim' / set_name


def run_dir(set_name, build=None, case=None):
    d = work_dir() / 'run' / set_name
    if build is not None:
        d = d / build
    return d / case if case is not None else d


def sum_dir(tag, build=None):
    d = work_dir() / 'sum' / tag
    return d / build if build is not None else d


def fld_dir(set_name, build=None):
    d = work_dir() / 'fld' / set_name
    return d / build if build is not None else d


def resid_dir(set_name, build):
    return work_dir() / 'resid' / set_name / build


def run_arg(text):
    """A run directory given as a path, or as set/build/case under run/."""
    p = pathlib.Path(text)
    return p if p.is_dir() else work_dir() / 'run' / text


def int_list(text):
    """Integers of a comma list of values and ranges A-B, all 0 or more."""
    out = []
    for part in text.split(','):
        m = re.fullmatch(r'\s*(\d+)\s*-\s*(\d+)\s*', part)
        if m:
            out.extend(range(int(m.group(1)), int(m.group(2)) + 1))
        elif part.strip():
            out.append(int(part))
    return out


def names_check(points):
    """Stop with status 1 when two grid points write one case name. points
    holds (name, grid point) pairs; the message names both points."""
    seen = {}
    for name, point in points:
        if name in seen:
            sys.exit(f'two grid points write the case {name}: {seen[name]} '
                     f'and {point}')
        seen[name] = point


def csv_rows(path):
    """The rows of a CSV file with a header line, as dicts of strings."""
    with open(path) as f:
        return list(csv.DictReader(f))


def summaries(tag, build, pattern='*'):
    """(case, summary) of each sum/<tag>/<build>/<pattern>.json, by name."""
    out = []
    for p in sorted(sum_dir(tag, build).glob(pattern + '.json')):
        with open(p) as f:
            out.append((p.stem, json.load(f)))
    return out


def index_runs(indexes):
    """The runs of consecutive values of a rising list of indexes."""
    runs = []
    for i in indexes:
        if runs and i == runs[-1][-1] + 1:
            runs[-1].append(i)
        else:
            runs.append([i])
    return runs


def run_start(counts, i):
    """The first index of the run of nonzero counts that ends at i."""
    a = i
    while a > 0 and counts[a - 1] > 0:
        a -= 1
    return a


def db(x, floor=1e-9):
    """10 log10 of a power ratio x, with x held at floor or more."""
    return 10 * math.log10(max(x, floor))


def names(text):
    """The names of a comma list."""
    return [n for n in text.split(',') if n]


def jobs():
    """Parallel jobs: UDSP_HARNESS_JOBS, or the CPU count."""
    return int(os.environ.get('UDSP_HARNESS_JOBS', os.cpu_count() or 1))


def parallel(fn, items):
    """fn over items on jobs() threads, in order of the items."""
    with concurrent.futures.ThreadPoolExecutor(jobs()) as pool:
        return list(pool.map(fn, items))


def run(cmd, out=None, env=None, cwd=None):
    """Run cmd; with out, its stdout and stderr go to that file. Returns
    the exit status."""
    e = dict(os.environ, **env) if env else None
    if out is None:
        return subprocess.run([str(c) for c in cmd], env=e, cwd=cwd).returncode
    with open(out, 'w') as f:
        return subprocess.run([str(c) for c in cmd], stdout=f,
                              stderr=subprocess.STDOUT, env=e,
                              cwd=cwd).returncode


STIM_LINKS = ('echo.f32', 'noise.f32', 'near.f32', 'digits.json')


def hz_run(build, stim, out, nlp=None, loss=None):
    """Run lec_hz of build on the stimulus directory stim into out.

    out gets frames.csv and out.s16, a copy of meta.json, links to the
    stimulus files that the analysis scripts read, and the link stim to the
    stimulus directory. nlp and loss default to the nlp and ref_loss_db
    entries of meta.json. The copy holds the nlp and ref_loss_db that the
    run takes, so an analysis reads the run's own switch and loss. The run
    takes the environment of switch_env. Returns the exit status of
    lec_hz."""
    stim = pathlib.Path(stim).resolve()
    out = pathlib.Path(out)
    out.mkdir(parents=True, exist_ok=True)
    meta = json.loads((stim / 'meta.json').read_text())
    if nlp is None:
        nlp = meta.get('nlp', 0)
    if loss is None:
        loss = int(round(2 * meta.get('ref_loss_db', 9.0)))
    meta.update(nlp=nlp, ref_loss_db=loss / 2)
    (out / 'meta.json').write_text(json.dumps(meta, indent=1))
    for name in STIM_LINKS + ('stim',):
        link = out / name
        src = stim if name == 'stim' else stim / name
        if (name == 'stim' or src.exists()) and not link.is_symlink():
            link.symlink_to(src)
    return run([program(build, 'lec_hz'), stim, meta['fs'], loss, nlp, out],
               env=switch_env(meta))


def switch_env(meta):
    """The environment of a run: UDSP_VAR_FROM_FRAME names the frame from
    which a variant rule of mkvar.py acts. A dt case (kind dt, stimdt.py and
    stimdtcap.py) takes the frame that holds the start of its dt phase, the
    double talk that the case measures. Any other case takes the frame that
    holds the start of its second phase: the path change of a conv case.
    None for a case with one phase."""
    phases = meta.get('phases', ())
    frame = meta['fs'] // 100
    if meta.get('kind') == 'dt':
        start = next(p['start_sample'] for p in phases if p['name'] == 'dt')
        return {'UDSP_VAR_FROM_FRAME': str(start // frame)}
    if len(phases) < 2:
        return None
    return {'UDSP_VAR_FROM_FRAME': str(phases[1]['start_sample'] // frame)}
