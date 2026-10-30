#!/usr/bin/env python3
"""simbench with the ranking's numbers (metrics.py) in every record, measured against the frozen lawn and body space:
bench.py [--impl DIR:MODULE] [--mowers FILE] [--cases FILE] [--twice] [--sim-seed0 K] [simbench args...]"""
import importlib
import json
import multiprocessing
import os
import re
import resource
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [TOOLS, os.path.dirname(TOOLS)]


def case_id(case: str) -> str:
    """A case without the path of an export (the same name on every machine)."""
    return re.sub(r'^export:[^@]*?([^/@]+\.json)/', r'export:\1/', case)


def _children(pid='self'):
    out = []
    try:
        for t in os.listdir(f'/proc/{pid}/task'):
            with open(f'/proc/{pid}/task/{t}/children') as f:
                out += [int(c) for c in f.read().split()]
    except OSError:
        pass
    return out


def usage():
    """CPU s and peak RSS MB of this process and its children, ended and running (a planner's pool)."""
    me, ch = resource.getrusage(resource.RUSAGE_SELF), resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu, kb = me.ru_utime + me.ru_stime + ch.ru_utime + ch.ru_stime, me.ru_maxrss + ch.ru_maxrss
    tick = os.sysconf('SC_CLK_TCK')
    todo = _children()
    while todo:
        pid = todo.pop()
        todo += _children(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                v = f.read().rsplit(')', 1)[1].split()
            cpu += sum(int(x) for x in v[11:15]) / tick  # (utime, stime, cutime, cstime)
            with open(f'/proc/{pid}/status') as f:
                kb += next((int(line.split()[1]) for line in f if line.startswith('VmHWM:')), 0)
        except (OSError, ValueError, IndexError):
            pass
    return cpu, kb / 1024.0


def _option(rest, name, default=None):
    if name not in rest:
        return default
    i = rest.index(name)
    value = rest[i + 1]
    del rest[i:i + 2]
    return value


def main(argv):
    rest = list(argv)
    impl = _option(rest, '--impl')
    mowers = _option(rest, '--mowers')
    cases = _option(rest, '--cases')
    seed0 = int(_option(rest, '--sim-seed0', 0))
    twice = '--twice' in rest
    if twice:
        rest.remove('--twice')
    if cases:
        with open(cases) as f:
            rest += [a for line in f if line.strip() for a in ('--case', line.strip())]
    # (exports by $MOWBITE_EXPORTS: the gardens of exports are kept out of the repo)
    rest = [os.path.expandvars(a) if i and rest[i - 1] == '--case' else a for i, a in enumerate(rest)]
    import frozen
    import gardens
    import metrics
    import simbench
    from shapely.geometry import Polygon
    from mowbite_planner import planner as ours
    from mowbite_planner.export import fingerprint
    from mowbite_planner.geometry import polygons

    if mowers:
        with open(mowers) as f:
            for name, settings in json.load(f).items():
                gardens.MOWERS[name] = ((), (), float(settings.get('mower_width', 0.18)), dict(settings))
    simbench.lawn_of = frozen.lawn_of
    planner, set_workers = simbench.plan_coverage, ours.set_workers
    if impl:
        where, name = impl.rsplit(':', 1)
        sys.path.insert(0, os.path.abspath(where))
        module = importlib.import_module(name)
        planner, set_workers = module.plan_coverage, getattr(module, 'set_workers', set_workers)
    asked = {}  # the case's request (one case a process)

    def planned(*args, **kw):
        cpu0, _ = usage()
        plan = planner(*args, **kw)
        cpu1, rss = usage()
        asked.update(metrics.request(*args, **kw))
        plan._cpu_s, plan._rss_mb = cpu1 - cpu0, rss  # (CPU time: steadier than the wall time on a busy PC)
        plan._start_xy, plan._end_xy = asked['start'], asked['end']
        if twice:
            # (simbench's case processes are daemons, which may not start the planner's pool otherwise)
            me = multiprocessing.current_process()
            daemon = me._config.get('daemon')
            me._config['daemon'] = False
            set_workers(3)
            try:
                plan._fp_w3 = fingerprint(planner(*args, **kw))
            except Exception as e:  # noqa: BLE001
                plan._fp_w3 = f'{type(e).__name__}: {e}'
            finally:
                set_workers(0)
                me._config['daemon'] = daemon
        return plan

    simbench.plan_coverage = planned
    measure, evaluate, simulated = simbench.measure, simbench.evaluate, simbench._simulated

    def measured(rec, plan, lawn=None, zones=None):
        check = measure(rec, plan, lawn, zones)
        rec['case_id'] = case_id(rec['case'])
        t0 = time.perf_counter()
        try:
            points = [Polygon(r).representative_point() for p in polygons(lawn) for r in p.interiors] \
                if lawn is not None else []
            rank = metrics.rank_numbers(plan, points)
            asked['ref'] = metrics.Ref(lawn, asked, rec['mower'])
            rank.update(metrics.plan_numbers(plan, asked['ref'], rec['area']))
        except Exception as e:  # noqa: BLE001
            rank = {'error': f'{type(e).__name__}: {e}'}
        rank['metrics_s'] = round(time.perf_counter() - t0, 2)
        rank['plan_cpu_s'] = round(getattr(plan, '_cpu_s', 0.0), 2)
        rank['plan_rss_mb'] = round(getattr(plan, '_rss_mb', 0.0), 1)
        if hasattr(plan, '_fp_w3'):
            rank['fp_w3'] = plan._fp_w3
        rec['rank'] = rank
        return check

    def evaluated(plan, res, progress=None):
        ev = evaluate(plan, res, progress)
        if asked.get('ref') is not None:
            try:
                ev['rank'] = metrics.sim_numbers(plan, res.samples, asked['ref'])
            except Exception as e:  # noqa: BLE001
                ev['rank'] = {'rank_error': f'{type(e).__name__}: {e}'}
        return ev

    def simulated_seeds(rec, plan, seeds, own=None):
        if seed0 and isinstance(seeds, range) and seeds.start == 0:
            seeds = range(seed0, seed0 + len(seeds))
        sims = simulated(rec, plan, seeds, own)
        got = {s: ev.get('rank') or {} for s, ev, _ in sims}
        for s in rec.get('sim', []):
            s.update(got.get(s['seed'], {}))
        return sims

    simbench.measure, simbench.evaluate, simbench._simulated = measured, evaluated, simulated_seeds
    return simbench.main(rest)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
