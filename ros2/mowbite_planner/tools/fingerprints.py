#!/usr/bin/env python3
"""The fingerprints of the plans of test/test_invariants.py's matrix (shapes x mowers x settings), to compare two
machines: on the mower (arm64: GEOS and numpy with fused multiply-add there) the planner must plan what the simulation
and the acceptance planned on a PC. Two per case: the plan's own (export.fingerprint, to the bit) and one of the poses
rounded to ROUND with all segments, which must be the same (a yaw a bit off on arm64, 03.11.: the plan is the same).

  fingerprints.py [processes]   {case: [fingerprint, rounded] or the error} as JSON
  fingerprints.py A.json B.json the cases planned differently, exit 1 where the rounded ones differ
  fingerprints.py --plans case... those cases' plans (poses, segments) and check_plan's report as JSON, to see where
                                  two machines part
"""
import hashlib
import json
import math
import os
import struct
import sys
from concurrent.futures import ProcessPoolExecutor

ROUND = 1e-9
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', 'test'), HERE]


def rounded(plan) -> str:
    """SHA-256 of the poses rounded to ROUND (the heading as cos and sin: no jump at +-pi) and of all segments."""
    h = hashlib.sha256()
    for p in plan.poses:
        h.update(struct.pack('<4q', *(round(v / ROUND) for v in (p.x, p.y, math.cos(p.yaw), math.sin(p.yaw)))))
    for s in plan.segments:
        h.update(f'{s.kind} {s.start} {s.end} {s.reverse};'.encode())
    return h.hexdigest()


def one(case):
    import test_invariants
    from mowbite_planner.export import fingerprint
    try:
        plan = test_invariants.plan_of(*case)
        return '-'.join(c for c in case if c), [fingerprint(plan), rounded(plan)]
    except Exception as e:  # noqa: BLE001 - a planning error is a result too
        return '-'.join(c for c in case if c), f'{type(e).__name__}: {e}'


def compare(a: str, b: str) -> int:
    with open(a) as f:
        got_a = json.load(f)
    with open(b) as f:
        got_b = json.load(f)
    cases = sorted(set(got_a) | set(got_b))

    def part(c, i):
        a, b = got_a.get(c), got_b.get(c)
        return (a[i] if isinstance(a, list) else a) != (b[i] if isinstance(b, list) else b)
    bits = [c for c in cases if part(c, 0)]
    differ = [c for c in cases if part(c, 1)]
    for case in sorted(set(bits) | set(differ)):
        print(f'{case}: {got_a.get(case)} / {got_b.get(case)}' + ('' if case in differ else ' (the same rounded)'))
    print(f'{len(got_a)} and {len(got_b)} cases, {len(bits)} not the same to the bit, '
          f'{len(differ)} planned differently')
    return 1 if differ else 0


def plans(names) -> dict:
    import test_invariants
    from mowbite_planner.report import check_plan
    got = {}
    for case in test_invariants.CASES:
        name = '-'.join(c for c in case if c)
        if name in names:
            plan = test_invariants.plan_of(*case)
            got[name] = {'poses': [list(p) for p in plan.poses],
                         'segments': [[s.kind, s.start, s.end, s.reverse] for s in plan.segments],
                         'report': check_plan(plan)}
    return got


def main() -> int:
    if sys.argv[1:2] == ['--plans']:
        json.dump(plans(sys.argv[2:]), sys.stdout, default=str)
        print()
        return 0
    if len(sys.argv) == 3:
        return compare(*sys.argv[1:])
    import test_invariants
    with ProcessPoolExecutor(int(sys.argv[1]) if len(sys.argv) > 1 else os.cpu_count()) as pool:
        got = dict(pool.map(one, test_invariants.CASES))
    json.dump(got, sys.stdout, indent=1, sort_keys=True)
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
