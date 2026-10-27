#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-only
#
# udsp - software voice DSP for FXS lines
# Copyright (C) 2026 Adtran Inc.
# Author: Chad Monroe <chad.monroe@adtran.com>
"""The entry point of the canceller harness.

usage: lec_harness.py [--build DIR] [--work DIR] [--jobs N] <command> ...

Commands that build and run:

  variant <name> [mkvar.py options]   write var/<name>/lec.c
  build <name> [--lec-c PATH] [--cmake-arg=ARG ...]
                                      build the harness of a variant
  stim 2cc|16a|tone|conv <set> [options]
                                      write a stimulus set (stim.py,
                                      stim16a.py, stimtone.py, stimconv.py
                                      options)
  stim bench <set> <case> [<case> ...] --fs R --peak N [N ...]
               --phases LIST [--id ID]
                                      stimuli from bench 2C(c) runs
  stim cap <set> <case> [<case> ...] [--td N]
                                      conv cases from the canceller
                                      captures of bench Tests 2A, 2B and
                                      12 (stimcap.py)
  stim cn <set> [options]             Test 9A cases (stimcn.py options)
  stim dt <set> [options]             Test 3A and 3B cases (stimdt.py
                                      options)
  stim dtcap <set> <case> [<case> ...] [--td N] [--taps N]
                                      dt cases from the canceller
                                      captures of bench Tests 3A and 3B
                                      (stimdtcap.py)
  stim 9a <set> <case> [<case> ...] [--fs R [R ...]]
                                      cn cases from the signals of bench
                                      Test 9A runs (stim9a.py)
  stim 10a <set> <case> [<case> ...] [--fs R [R ...]] [--clause]
                                      fax cases from the signals of bench
                                      Test 10A and 10B runs (stim10a.py)
  run <set> <builds> [--match GLOB] [--nlp 0|1] [--loss N]
               [--summary TAG [--keep]]
                                      run lec_hz on a stimulus set
  place <builds> <case> [<case> ...] [--cases E:H,...] [--phases LIST]
               [--id ID]              placement cases (place.py)
  synplace <builds> --fs R --peak N [--shifts LIST] [--seeds LIST]
               [--phases LIST]        synthetic placement cases
  replay <set> <builds> <capture> [<capture> ...] [--coh] [--residual]
               [--skip-s N]           replay a capture set (lec_replay)
  test <builds>                       run test_lec of each build
  sweep scan <build> --rate R [--out FILE]
  sweep eval <file> <builds> [--out FILE]
                                      the tone line sweep (lec_tone_sweep)
  bench <base> <var> [--rates LIST] [--repeat N]
                                      bench_lec of two builds compared
  leg <leg> [<leg> ...] [--out DIR] [--openwrt DIR] [--cflags=FLAGS]
               [--cmake-arg=ARG ...]  plain, asan, asan-lec, tsan, xbuild

The analysis commands run the script of the same name with its own
arguments: analyze, table, trace, firstdiff, report_tab, sweeptab, candtab,
placetab, fldcmp, fldev, fldsum, fldshow, fldctx, fldfloor, fldres, fldcn,
an16a, cmp16a, arm, armfld, frmax, spec, convtab, cntab, dttab and faxtab.

<builds> is a comma list of build names: tree for the programs of the build
directory, any other name for b/<name>/ of the work directory (work.py).
LIST is a comma list of integers and ranges A-B. --build, --work and
--jobs set UDSP_HARNESS_BUILD, UDSP_HARNESS_WORK and UDSP_HARNESS_JOBS for
every command. The exit status is 0 when every step passed, 1 when a step
failed and 2 for a usage error.
"""
import argparse
import concurrent.futures
import importlib
import os
import pathlib
import re
import shutil
import sys

sys.dont_write_bytecode = True

HARNESS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HARNESS))

import work  # noqa: E402

TOOLS = ('analyze', 'table', 'trace', 'firstdiff', 'report_tab', 'sweeptab',
         'candtab', 'placetab', 'fldcmp', 'fldev', 'fldsum', 'fldshow',
         'fldctx', 'fldfloor', 'fldres', 'fldcn', 'an16a', 'cmp16a', 'arm',
         'armfld', 'frmax', 'spec', 'convtab', 'cntab', 'dttab', 'faxtab')
STIM_MODULES = {'2cc': 'stim', '16a': 'stim16a', 'tone': 'stimtone',
                'conv': 'stimconv', 'cap': 'stimcap', 'cn': 'stimcn',
                '9a': 'stim9a', 'dt': 'stimdt', 'dtcap': 'stimdtcap',
                '10a': 'stim10a'}
PLACE_CASES = '644:644,523:644,644:523,523:523'
REPLAY_STATUS = {0: 'PASS', 1: 'FAIL', 3: 'INCOMPLETE'}
SWEEP_SEEDS = ('0x70ae1', '0x13579', '0x2468a')
SWEEP_LEADS = {8000: (1, 2, 3, 4, 5, 6), 16000: (2, 4, 6, 8, 10, 12)}
SWEEP_FIXED = ('-18', '20', '-200')
TSAN_TESTS = 'test_core|test_engine|test_media|test_cli|test_linecap'


def fail(msg):
    print(f'lec_harness.py: {msg}', file=sys.stderr)
    return 1


def pool_map(fn, items):
    """fn over items on jobs() processes, in order of the items."""
    with concurrent.futures.ProcessPoolExecutor(work.jobs()) as pool:
        return list(pool.map(fn, items))


# ------------------------------------------------------------ variant, build

def cmd_variant(argv):
    import mkvar
    return mkvar.main(argv)


CACHE_KEYS = ('CMAKE_C_COMPILER', 'PKG_CONFIG_EXECUTABLE')


def cache_args():
    """The compiler and the pkg-config of the build directory's cache, so
    that a variant builds with the toolchain of the tree build."""
    cache = work.build_dir() / 'CMakeCache.txt'
    if not cache.exists():
        return []
    out = []
    for line in cache.read_text().splitlines():
        key, _, value = line.partition('=')
        name = key.split(':', 1)[0]
        if name in CACHE_KEYS and value and not value.endswith('-NOTFOUND'):
            out.append(f'-D{name}={value}')
    return out


def cmd_build(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py build')
    p.add_argument('name')
    p.add_argument('--lec-c')
    p.add_argument('--cmake-arg', action='append', default=[])
    a = p.parse_args(argv)
    if a.name == work.TREE:
        return fail('tree is the build directory: configure it with '
                    'cmake -DUDSP_BENCH=ON and build it there')
    src = pathlib.Path(a.lec_c) if a.lec_c else \
        work.work_dir() / 'var' / a.name / 'lec.c'
    if not src.is_file():
        return fail(f'{src}: no canceller source')
    out = work.work_dir() / 'b' / a.name
    cmake = os.environ.get('CMAKE', 'cmake')
    rc = work.run([cmake, '-S', HARNESS, '-B', out,
                   f'-DUDSP_HARNESS_LEC_C={src.resolve()}'] +
                  cache_args() + a.cmake_arg)
    if rc == 0:
        rc = work.run([cmake, '--build', out, '-j', str(work.jobs())])
    print(f'build {a.name}: rc {rc}, {out}')
    return 1 if rc else 0


# ------------------------------------------------------------ stimuli

def cmd_stim(argv):
    if len(argv) < 2:
        return fail('usage: stim 2cc|16a|tone|conv|cap|cn|9a|dt|dtcap|10a|'
                    'bench <set> [options]')
    kind, set_name, rest = argv[0], argv[1], argv[2:]
    out = work.stim_dir(set_name)
    if kind == 'bench':
        return stim_bench(out, rest)
    if kind not in STIM_MODULES:
        return fail(f'unknown stimulus kind {kind}')
    return importlib.import_module(STIM_MODULES[kind]).main([str(out)] +
                                                            rest)


def run_id(case):
    """The id of a bench run: the first four characters after the last '-'
    of the run directory that holds the case."""
    for part in reversed(pathlib.Path(case).resolve().parts):
        if '-' in part and part != 'cases':
            return part.rsplit('-', 1)[1][:4]
    return pathlib.Path(case).name[:4]


def bench_case(path):
    """The 2C(c) case directory of a run directory or of a case directory."""
    p = pathlib.Path(path)
    if (p / 'g168' / 'test_2c_c_schedule.json').exists():
        return p
    found = sorted(p.glob('cases/*test_2c*part=c*'))
    if not found:
        raise SystemExit(f'lec_harness.py: {path}: no 2C(c) case')
    return found[0]


def stim_bench(out, argv):
    import benchstim
    p = argparse.ArgumentParser(prog='lec_harness.py stim bench')
    p.add_argument('cases', nargs='+')
    p.add_argument('--fs', type=int, default=16000)
    p.add_argument('--peak', type=int, nargs='+', required=True)
    p.add_argument('--phases', type=work.int_list, default=[0])
    p.add_argument('--id')
    a = p.parse_args(argv)
    grid = []
    for c in a.cases:
        case = bench_case(c)
        rid = a.id or run_id(case)
        grid += [(f'b-{rid}-{a.fs}-{peak}-s{ph}', case, peak, ph)
                 for peak in a.peak for ph in a.phases]
    work.names_check([(g[0], f'{g[1]} peak {g[2]} phase {g[3]}')
                      for g in grid])
    for name, case, peak, ph in grid:
        m = benchstim.build(str(case), str(out / name), a.fs, peak, ph)
        print(name, 'lag', m['lag'], 't0_frame', '%.2f' % m['t0_frame'])
    return 0


# ------------------------------------------------------------ runs

def run_one(job):
    """One lec_hz run of a stimulus; with a tag, its summary replaces the
    run directory unless keep is set."""
    build, stim, out, nlp, loss, tag, keep = job
    rc = work.hz_run(build, stim, out, nlp, loss)
    if rc == 0 and tag:
        import json
        d = work.sum_dir(tag, build)
        d.mkdir(parents=True, exist_ok=True)
        (d / f'{pathlib.Path(stim).name}.json').write_text(
            json.dumps(summary_get(out)))
        if not keep:
            shutil.rmtree(out)
    return rc


def summary_get(out):
    """The summary of a run directory: convtab.py for a conv case, cntab.py
    for a cn case, dttab.py for a dt case, faxtab.py for a fax case,
    analyze.py for the others."""
    import json
    meta = json.loads((pathlib.Path(out) / 'meta.json').read_text())
    if meta.get('kind') == 'conv':
        import convtab
        return convtab.summary(out)
    if meta.get('kind') == 'cn':
        import cntab
        return cntab.summary(out)
    if meta.get('kind') == 'dt':
        import dttab
        return dttab.summary(out)
    if meta.get('kind') == 'fax':
        import faxtab
        return faxtab.summary(out)
    import analyze
    return analyze.summary(out)


def cmd_run(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py run')
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('--match', default='*')
    p.add_argument('--nlp', type=int)
    p.add_argument('--loss', type=int)
    p.add_argument('--summary')
    p.add_argument('--keep', action='store_true')
    a = p.parse_args(argv)
    stims = sorted(d for d in work.stim_dir(a.set).glob(a.match)
                   if (d / 'meta.json').exists())
    if not stims:
        return fail(f'no stimulus in {work.stim_dir(a.set)} matches '
                    f'{a.match}')
    jobs = [(b, s, work.run_dir(a.set, b, s.name), a.nlp, a.loss, a.summary,
             a.keep) for b in a.builds for s in stims]
    rcs = pool_map(run_one, jobs)
    bad = [f'{j[0]} {j[1].name} rc {rc}' for j, rc in zip(jobs, rcs) if rc]
    for line in bad:
        print('FAIL', line)
    print(f'run {a.set}: {len(jobs)} runs, {len(bad)} failed')
    return 1 if bad else 0


def place_one(job):
    import place
    return place.main([str(x) for x in job])


def cmd_place(argv):
    import place
    p = argparse.ArgumentParser(prog='lec_harness.py place')
    p.add_argument('builds', type=work.names)
    p.add_argument('cases', nargs='+')
    p.add_argument('--cases', dest='pairs', default=PLACE_CASES)
    p.add_argument('--phases', type=work.int_list,
                   default=list(range(160)))
    p.add_argument('--id')
    a = p.parse_args(argv)
    pairs = [tuple(int(v) for v in c.split(':')) for c in
             work.names(a.pairs)]
    cases = [bench_case(c) for c in a.cases]
    ids = [a.id or run_id(case) for case in cases]
    work.names_check([(rid, str(case)) for rid, case in zip(ids, cases)])
    jobs = []
    for case, rid in zip(cases, ids):
        place.signals_get(case, rid)
        jobs += [[case, rid, e, h, ph] + a.builds for e, h in pairs
                 for ph in a.phases]
    rcs = pool_map(place_one, jobs)
    print(f'place: {len(jobs)} cases, {sum(1 for r in rcs if r)} failed')
    return 1 if any(rcs) else 0


def synplace_one(job):
    import synplace
    return synplace.main([str(x) for x in job])


def cmd_synplace(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py synplace')
    p.add_argument('builds', type=work.names)
    p.add_argument('--fs', type=int, required=True)
    p.add_argument('--peak', type=int, required=True)
    p.add_argument('--shifts', type=work.int_list, default=[0])
    p.add_argument('--seeds', type=work.int_list, default=[0])
    p.add_argument('--phases', type=work.int_list)
    a = p.parse_args(argv)
    phases = a.phases if a.phases is not None else \
        list(range(a.fs // 100))
    jobs = [[a.fs, a.peak, h, s, ph] + a.builds for h in a.shifts
            for s in a.seeds for ph in phases]
    rcs = pool_map(synplace_one, jobs)
    print(f'synplace: {len(jobs)} cases, {sum(1 for r in rcs if r)} failed')
    return 1 if any(rcs) else 0


# ------------------------------------------------------------ replays

def captures_find(paths):
    """The .lecr files under each path, a file or a directory."""
    out = []
    for p in paths:
        p = pathlib.Path(p)
        out += [p] if p.is_file() else sorted(p.rglob('*.lecr'))
    return out


def captures_index(files):
    """The id of each capture: its file stem when the stems differ.
    Otherwise an id joins with '-' the components of its path under the
    common root that differ between the captures, its stem included when
    stems differ there too."""
    paths = [f.resolve() for f in files]
    stems = [p.stem for p in paths]
    if len(set(stems)) == len(stems):
        return {p.stem: str(p) for p in paths}
    root = pathlib.Path(os.path.commonpath([str(p.parent) for p in paths]))
    rel = [p.with_suffix('').relative_to(root).parts for p in paths]
    depth = max(len(r) for r in rel)
    vary = [k for k in range(depth)
            if len({r[k] if k < len(r) else None for r in rel}) > 1]
    out = {}
    for p, r in zip(paths, rel):
        cid = '-'.join(r[k] for k in vary if k < len(r)) or p.stem
        if cid in out:
            raise SystemExit(f'lec_harness.py: two captures read {cid}: '
                             f'{out[cid]} and {p}')
        out[cid] = str(p)
    return out


def replay_one(job):
    build, cid, path, out, coh, resid, skip = job
    prog = work.program(build, 'lec_replay_coh' if coh else 'lec_replay')
    cmd = [prog, path, '--frames', out / f'{cid}.csv']
    env = {'UDSP_LEC_COH_ROWS': str(out / f'{cid}.coh')} if coh else None
    if resid:
        d = work.resid_dir(out.parent.name, build)
        d.mkdir(parents=True, exist_ok=True)
        cmd += ['--residual', d / f'{cid}.wav']
    if skip is not None:
        cmd += ['--skip-s', skip]
    return work.run(cmd, out / f'{cid}.txt', env=env)


def cmd_replay(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py replay')
    p.add_argument('set')
    p.add_argument('builds', type=work.names)
    p.add_argument('captures', nargs='+')
    p.add_argument('--coh', action='store_true')
    p.add_argument('--residual', action='store_true')
    p.add_argument('--skip-s', type=int)
    a = p.parse_args(argv)
    if a.coh and a.residual:
        return fail('--coh takes no --residual: the shadow instance would '
                    'write rows')
    index = captures_index(captures_find(a.captures))
    if not index:
        return fail('no capture found')
    top = work.fld_dir(a.set)
    top.mkdir(parents=True, exist_ok=True)
    import json
    (top / 'index.json').write_text(json.dumps(index, indent=1))
    jobs = []
    for b in a.builds:
        out = work.fld_dir(a.set, b)
        out.mkdir(parents=True, exist_ok=True)
        jobs += [(b, cid, path, out, a.coh, a.residual, a.skip_s)
                 for cid, path in sorted(index.items())]
    rcs = work.parallel(replay_one, jobs)
    for b in a.builds:
        tally = {}
        for j, rc in zip(jobs, rcs):
            if j[0] == b:
                key = REPLAY_STATUS.get(rc, f'rc {rc}')
                tally[key] = tally.get(key, 0) + 1
        print(f'replay {a.set} {b}: {len(index)} captures, ' +
              ', '.join(f'{k} {v}' for k, v in sorted(tally.items())))
    return 1 if any(rc not in REPLAY_STATUS for rc in rcs) else 0


# ------------------------------------------------------------ test, sweep

def test_one(build):
    prog = work.program(build, 'test_lec')
    out = work.work_dir() / 'test' / f'{build}.out'
    out.parent.mkdir(parents=True, exist_ok=True)
    if not prog.exists():
        return build, None, f'{prog}: no test_lec', ''
    rc = work.run([prog], out, cwd=prog.parent)
    text = out.read_text()
    summary = ' '.join(re.findall(r'^# lec:.*$', text, re.M))
    failed = ' '.join(re.sub(r' - .*', '', line) for line in
                      re.findall(r'^not ok.*$', text, re.M))
    return build, rc, summary, failed


def cmd_test(argv):
    if len(argv) != 1:
        return fail('usage: test <builds>')
    res = work.parallel(test_one, work.names(argv[0]))
    for build, rc, summary, failed in res:
        print(f'test_lec {build}: rc {rc} | {summary} | {failed}')
    return 1 if any(r[1] != 0 for r in res) else 0


def sweep_one(job):
    """(status, text) of one lec_tone_sweep run: its output line, or the
    arguments and the status of a run that failed."""
    prog, args = job
    import subprocess
    try:
        r = subprocess.run([str(prog)] + args, capture_output=True, text=True)
    except OSError as e:
        return 127, ' '.join(args) + f' | {e.strerror}'
    if r.returncode:
        return r.returncode, ' '.join(args) + f' | rc {r.returncode}'
    return 0, r.stdout.strip()


def sweep_grid(rate):
    """The arguments of the stage 1 grid at rate."""
    for ulaw in ('0', '1'):
        for floor in ('-200', '-45', '-40', '-37', '-35'):
            for digit in ('-15', '-12', '-9'):
                for lead in SWEEP_LEADS[rate]:
                    for hybrid in ('0', '1', '2'):
                        for seed in SWEEP_SEEDS:
                            yield [str(rate), digit, floor, ulaw, str(lead),
                                   seed, hybrid, *SWEEP_FIXED]


SWEEP_KEYS = ('rate', 'digit', 'floor', 'ulaw', 'lead', 'seed', 'hybrid',
              'dial', 'trials', 'line')


def sweep_args(line):
    """The arguments of lec_tone_sweep in a config line: the arguments
    themselves, or a line that lec_tone_sweep printed."""
    words = line.split('|', 1)[0].split()
    if words and words[0] == 'rate':
        kv = dict(zip(words[0::2], words[1::2]))
        return [kv[k] for k in SWEEP_KEYS]
    return words


def cmd_sweep(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py sweep')
    p.add_argument('stage', choices=('scan', 'eval'))
    p.add_argument('first')
    p.add_argument('builds', nargs='?', type=work.names)
    p.add_argument('--rate', type=int, default=8000)
    p.add_argument('--out')
    a = p.parse_args(argv)
    if a.stage == 'scan':
        out = pathlib.Path(a.out or work.work_dir() / 'sweep' /
                           f'scan-{a.first}-{a.rate}.txt')
        prog = work.program(a.first, 'lec_tone_sweep')
        res = work.parallel(sweep_one, [(prog, g)
                                        for g in sweep_grid(a.rate)])
        lines = [text for _, text in res]
        armed = sum(1 for rc, text in res
                    if rc == 0 and re.search(r'arms [1-9]', text))
        print(f'scan {a.first} {a.rate}: {len(lines)} configs, {armed} arm')
    else:
        if not a.builds:
            return fail('usage: sweep eval <file> <builds>')
        configs = [sweep_args(line) for line in
                   pathlib.Path(a.first).read_text().splitlines()
                   if line.strip()]
        out = pathlib.Path(a.out or work.work_dir() / 'sweep' / 'eval.txt')
        res = work.parallel(sweep_one, [(work.program(b, 'lec_tone_sweep'),
                                         c) for c in configs
                                        for b in a.builds])
        lines = []
        for i, c in enumerate(configs):
            row = res[i * len(a.builds):(i + 1) * len(a.builds)]
            lines.append(' '.join(c) + ' | ' + ' | '.join(
                f'{b}: ' + text.split('| ', 1)[-1]
                for b, (_, text) in zip(a.builds, row)) + ' |')
        print(f'eval: {len(lines)} rows')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text('\n'.join(lines) + '\n')
    print(out)
    failed = sum(1 for rc, _ in res if rc)
    if failed:
        print(f'sweep {a.stage}: {failed} runs failed')
    return 1 if failed else 0


# ------------------------------------------------------------ bench

TIMING_RATIO_HEADS = ('gain',)


def timing_spans(header):
    """The column spans of a table header that hold timing figures: each
    column whose heading names ns, and in a table with such a column, the
    gain column, the ratio of two of them. Empty for a table with no ns
    column, so the gain of an echo path stays."""
    cells = list(re.finditer(r'[^|]+', header))
    if not any(re.search(r'\bns\b', m.group()) for m in cells):
        return []
    return [m.span() for m in cells if re.search(r'\bns\b', m.group()) or
            m.group().strip() in TIMING_RATIO_HEADS]


def timing_masked(text):
    """text with the timing cells of each table blanked (timing_spans), so
    two runs compare on their non-timing figures."""
    out, spans = [], []
    for line in text.splitlines():
        head = timing_spans(line) if '|' in line else []
        if not line.strip():
            spans = []
        elif head:
            spans = head
            line = ''
        elif '|' in line and spans:
            chars = list(line)
            for s, e in spans:
                for i in range(s, min(e, len(chars))):
                    chars[i] = ' '
            line = ''.join(chars)
        out.append(line)
    return out


def ns_mean(text, head):
    """The mean of the first number in the column whose header holds head,
    over the rows of that table."""
    vals, span = [], None
    for line in text.splitlines():
        if not line.strip():
            span = None
        elif head in line:
            span = next(m.span() for m in re.finditer(r'[^|]+', line)
                        if head in m.group())
        elif span and '|' in line:
            m = re.search(r'\d+', line[span[0]:span[1]])
            if m:
                vals.append(int(m.group()))
    return sum(vals) / len(vals) if vals else float('nan')


def bench_one(job):
    build, rate, out = job
    return work.run([work.program(build, 'bench_lec'), '--rate', rate], out)


def cmd_bench(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py bench')
    p.add_argument('base')
    p.add_argument('var')
    p.add_argument('--rates', type=work.int_list, default=[8000, 16000])
    p.add_argument('--repeat', type=int, default=1)
    a = p.parse_args(argv)
    for b in (a.base, a.var):
        if not work.program(b, 'bench_lec').exists():
            return fail(f'{b}: no bench_lec; the build needs speexdsp')
    d = work.work_dir() / 'bench'
    d.mkdir(parents=True, exist_ok=True)
    rc = 0
    heads = ('ns per run()', 'udsp scalar ns', 'full form ns')
    for rate in a.rates:
        means = {b: [] for b in (a.base, a.var)}
        for i in range(a.repeat):
            for b in (a.var, a.base):
                f = d / f'{b}-{rate}-{i}.txt'
                if bench_one((b, rate, f)):
                    return fail(f'bench_lec of {b} failed, see {f}')
                means[b].append([ns_mean(f.read_text(), h) for h in heads])
            ta = timing_masked((d / f'{a.base}-{rate}-{i}.txt').read_text())
            tb = timing_masked((d / f'{a.var}-{rate}-{i}.txt').read_text())
            diff = [(x, y) for x, y in zip(ta, tb) if x != y]
            if len(ta) != len(tb) or diff:
                rc = 1
                print(f'bench {rate} run {i}: {len(diff)} non-timing lines '
                      f'differ')
                for x, y in diff[:20]:
                    print('  <', x)
                    print('  >', y)
            else:
                print(f'bench {rate} run {i}: non-timing lines identical')
        for b in (a.base, a.var):
            m = [sum(v[k] for v in means[b]) / len(means[b])
                 for k in range(len(heads))]
            print(f'bench {rate} {b}: mean ' + ', '.join(
                f'{h} {v:.0f}' for h, v in zip(heads, m)))
    return rc


# ------------------------------------------------------------ legs

def leg_cmake(name, out, flags, a, tests=None, env=None, bench=True):
    """Configure, build and run ctest for one leg in out/<name>. The C flags
    are those of the leg and of --cflags."""
    b = out / name
    cmake = os.environ.get('CMAKE', 'cmake')
    ctest = os.environ.get('CTEST', 'ctest')
    cfg = [cmake, '-S', work.UDSP_SRC, '-B', b, '-DUDSP_TESTS=ON',
           '-DUDSP_TOOLS=ON'] + (['-DUDSP_BENCH=ON'] if bench else [])
    flags = ' '.join(f for f in (flags, a.cflags) if f)
    if flags:
        cfg.append(f'-DCMAKE_C_FLAGS={flags}')
    if work.run(cfg + cache_args() + a.cmake_arg,
                out / f'{name}-configure.log'):
        return fail(f'leg {name}: configure failed, see '
                    f'{out}/{name}-configure.log')
    if work.run([cmake, '--build', b, '-j', str(work.jobs())],
                out / f'{name}-build.log'):
        return fail(f'leg {name}: build failed, see {out}/{name}-build.log')
    cmd = [ctest, '--test-dir', b, '--output-on-failure']
    if tests:
        cmd += ['-R', tests]
    rc = work.run(cmd, out / f'{name}-ctest.log', env=env)
    text = (out / f'{name}-ctest.log').read_text().splitlines()
    print(f'leg {name}: ctest rc {rc}: ' +
          next((t for t in text if 'tests passed' in t), 'no summary'))
    return 1 if rc else 0


def leg_xbuild(openwrt, out):
    """The package builds of udsp and uvoice in an OpenWrt tree."""
    if not openwrt:
        return fail('leg xbuild: --openwrt names the OpenWrt tree')
    rc = 0
    for pkg in ('udsp', 'uvoice'):
        r = work.run(['make', f'package/{pkg}/clean', f'package/{pkg}/compile',
                      'V=s'], out / f'xbuild-{pkg}.log', cwd=openwrt)
        print(f'leg xbuild {pkg}: rc {r}')
        rc |= r
    for f in pathlib.Path(openwrt).glob('build_dir/target-*/udsp-*/src/dsp/'
                                        'lec.c'):
        same = f.read_bytes() == (work.UDSP_SRC / 'dsp/lec.c').read_bytes()
        print(f'leg xbuild: {f} {"equals" if same else "differs from"} the '
              f'tree lec.c')
    for f in pathlib.Path(openwrt).glob('build_dir/target-*/uvoice-*/**/'
                                        'uvoiced'):
        if f.is_file():
            import hashlib
            print(f'leg xbuild: {f} md5 '
                  f'{hashlib.md5(f.read_bytes()).hexdigest()}')
    return 1 if rc else 0


def cmd_leg(argv):
    p = argparse.ArgumentParser(prog='lec_harness.py leg')
    p.add_argument('legs', nargs='+',
                   choices=('plain', 'asan', 'asan-lec', 'tsan', 'xbuild'))
    p.add_argument('--out')
    p.add_argument('--openwrt')
    p.add_argument('--cmake-arg', action='append', default=[])
    p.add_argument('--cflags')
    a = p.parse_args(argv)
    out = pathlib.Path(a.out) if a.out else work.work_dir() / 'legs'
    out.mkdir(parents=True, exist_ok=True)
    asan = '-g -fsanitize=address,undefined -fno-omit-frame-pointer'
    leak = {'ASAN_OPTIONS': 'detect_leaks=1'}
    rc = 0
    for leg in a.legs:
        if leg == 'plain':
            rc |= leg_cmake('plain', out, None, a)
        elif leg == 'asan':
            rc |= leg_cmake('asan', out, asan, a, env=leak, bench=False)
        elif leg == 'asan-lec':
            rc |= leg_cmake('asan', out, asan, a, tests='^test_lec$',
                            env=leak, bench=False)
        elif leg == 'tsan':
            rc |= leg_cmake('tsan', out, '-g -fsanitize=thread', a,
                            tests=TSAN_TESTS, bench=False)
        else:
            rc |= leg_xbuild(a.openwrt, out)
    return rc


# ------------------------------------------------------------ main

COMMANDS = {'variant': cmd_variant, 'build': cmd_build, 'stim': cmd_stim,
            'run': cmd_run, 'place': cmd_place, 'synplace': cmd_synplace,
            'replay': cmd_replay, 'test': cmd_test, 'sweep': cmd_sweep,
            'bench': cmd_bench, 'leg': cmd_leg}


def main(argv):
    p = argparse.ArgumentParser(
        prog='lec_harness.py', add_help=False,
        usage='lec_harness.py [--build DIR] [--work DIR] [--jobs N] '
              '<command> ...')
    p.add_argument('--build')
    p.add_argument('--work')
    p.add_argument('--jobs', type=int)
    p.add_argument('-h', '--help', action='store_true')
    a, rest = p.parse_known_args(argv)
    if a.help or not rest:
        print(__doc__.split('\n', 2)[2] if a.help else p.format_usage())
        return 0 if a.help else 2
    if a.build:
        os.environ['UDSP_HARNESS_BUILD'] = str(pathlib.Path(a.build).resolve())
    if a.work:
        os.environ['UDSP_HARNESS_WORK'] = str(pathlib.Path(a.work).resolve())
    if a.jobs:
        os.environ['UDSP_HARNESS_JOBS'] = str(a.jobs)
    cmd, args = rest[0], rest[1:]
    if cmd in COMMANDS:
        return COMMANDS[cmd](args)
    if cmd in TOOLS:
        return importlib.import_module(cmd).main(args)
    print(f'lec_harness.py: unknown command {cmd}', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
