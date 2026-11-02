"""What every plan keeps to, over a matrix of shapes, mowers and settings (planner 2's tests, not one garden's): the
fixed rules as the measuring tools count them (tools/metrics.py against what the map alone gives, tools/frozen.py),
the plan's form (planrules.check_invariants), stats that tell what the poses do, the warnings the app reads, the same
plan wherever and however it's planned (what a plan is measured against comes from the map and the request alone,
metrics.Ref). Marked matrix: python3 -m pytest test -m 'not matrix' leaves them out."""

import json
import math
import os
import re
import subprocess
import sys
import random
import threading

import pytest
from shapely.geometry import Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', 'tools'), HERE]

from mowbite_planner import PlannerConfig, PlanningError, plan_coverage  # noqa: E402
from mowbite_planner.config import LANE, PERIMETER, TRANSIT, TURN  # noqa: E402
from mowbite_planner.export import fingerprint, to_openmower_json  # noqa: E402
from planrules import check_invariants  # noqa: E402
import metrics  # noqa: E402

pytestmark = pytest.mark.matrix


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def circle(cx, cy, r, n=16):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


# name: boundary, obstacles, more of plan_coverage's arguments (start, end, drivable, keep_out, ...)
DOCK = (1.0, 0.6, 0.0)
SHAPES = {
    'rect': (rect(0, 0, 8, 5), [], {}),
    'garden': ([(0, 0), (12, 0), (13, 5), (7, 5), (7, 9), (0, 9)], [circle(3, 6, 0.6), rect(8, 1.5, 10, 3)],
               {'start': DOCK, 'end': DOCK[:2]}),
    'L': ([(0, 0), (10, 0), (10, 4), (4, 4), (4, 9), (0, 9)], [], {'start': DOCK, 'end': DOCK[:2]}),
    # a hedge across: two parts with no way between them
    'two': (rect(0, 0, 12, 6), [rect(5.5, -1, 6.5, 7)], {}),
    # two lawns joined by a strip 1.2 m wide
    'neck': ([(0, 0), (5, 0), (5, 2), (7, 2), (7, 0), (12, 0), (12, 5), (7, 5), (7, 3.2), (5, 3.2), (5, 5), (0, 5)],
             [], {}),
    # a bed 0.25 m off the edge: a slot too narrow for the body
    'slot': (rect(0, 0, 9, 6), [rect(0.25, 2, 2, 3.5)], {}),
    # a drive beside the lawn, the dock on it, part of it to keep out of (plan_coverage: nothing goes there on the way)
    'drive': (rect(0, 0, 10, 6), [], {'start': (11.0, 3.0, math.pi), 'end': (11.0, 3.0),
                                      'drivable': [rect(10, 1, 14, 5)], 'keep_out': [rect(10, 4, 14, 5)]}),
    # the next lawn, not to mow, 2 m into this one: with the body like an obstacle (without it it's mowed)
    'keep': (rect(0, 0, 10, 6), [], {'start': (1.0, 1.0, 0.0), 'keep_out': [rect(4, 4, 6, 9)]}),
}
OUTLINE = ((0.43, -0.03), (0.43, 0.03), (0.26, 0.2), (-0.14, 0.2), (-0.14, -0.2), (0.26, -0.2))
BODIES = {
    'bare': {},
    'rect': dict(robot_width=0.41, robot_front=0.43, robot_rear=0.18, blade_ahead=0.185),
    'outline': dict(robot_width=0.4, robot_front=0.43, robot_rear=0.14, robot_outline=OUTLINE, blade_ahead=0.17),
}
BASE = dict(mower_width=0.18, perimeter_passes=3)  # (every calculation with three loops, mkay 31.10.)
SETTINGS = {
    'crosshatch': dict(fill_pattern='crosshatch'),
    'concentric': dict(fill_pattern='concentric'),
    'last': dict(perimeter_order='last'),
    'skip': dict(lane_order='skip'),
    'auto': dict(lane_spacing_mode='auto'),
    'passes1': dict(perimeter_passes=1),
    'passes4': dict(perimeter_passes=4),
    'obstacle0': dict(obstacle_passes=0),
    'on_spot': dict(turn_on_spot=True),
    'radius0': dict(turn_radius=0.0),
    'no_pivot': dict(turn_types=('u_turn', 'bulb', 'detour')),
    'hard': dict(edges='hard'),
    'cw': dict(perimeter_direction='cw'),
    'random': dict(perimeter_direction='random', finished_mows=1),
    'blade_offset': dict(blade_offset=0.04),
    'narrow_loops': dict(narrow_parts='loops'),
    'zones': dict(split_zones=True),
    'headland': dict(headland_turns=True),
    'optimized': dict(route_order='optimized'),
    'margins0': dict(edge_margin=0.0, spin_margin=0.0),
    'outside': dict(perimeter_offset=-0.03),
}
CASES = [(s, b, '') for s in SHAPES for b in BODIES if (s, b) != ('keep', 'bare')] + \
    [('garden', b, k) for k in SETTINGS for b in BODIES]
# the app's patterns for the counted warnings (mowbite-new-planner src/lib/areaPlan.ts, COUNTED)
COUNTED = (re.compile(r"^(\d+) places? driven another way where the body would stick out"),
           re.compile(r"^(\d+) places? left out where the body doesn't fit"))
STATS = ('sections', 'path_length', 'perimeter_length', 'lane_length', 'turn_length', 'transit_length', 'poses',
         'lanes', 'planning_time', 'turns_u_turn', 'turns_bulb', 'turns_detour', 'turns_pivot', 'turns_fallback',
         'chosen', 'leftovers', 'mop_up')
_plans = {}


def request(shape, body, setting):
    boundary, obstacles, more = SHAPES[shape]
    cfg = PlannerConfig(**dict(BASE, **BODIES[body], **SETTINGS.get(setting, {})))
    return metrics.request(boundary, obstacles, cfg, **more)


def plan_of(shape, body, setting=''):
    key = (shape, body, setting)
    if key not in _plans:
        r = request(*key)
        _plans[key] = plan_coverage(r['boundary'], r['obstacles'], r['cfg'], r['start'], r['end'], r['drivable'],
                                    r['keep_out'], r['margins'])
    return _plans[key]


def lawn(r):
    """The lawn as drawn (frozen.lawn_of): the area but its obstacles and areas to keep out of."""
    holes = [Polygon(o).buffer(0) for o in r['obstacles'] + r['keep_out'] if len(o) >= 3]
    area = Polygon(r['boundary']).buffer(0)
    return area.difference(unary_union(holes)) if holes else area


@pytest.mark.parametrize('shape, body, setting', CASES, ids=['-'.join(c for c in case if c) for case in CASES])
def test_every_plan(shape, body, setting):
    """Each plan of the matrix planned once, checked by all of them."""
    plan = plan_of(shape, body, setting)
    keeps_the_rules(plan, request(shape, body, setting))
    stats_tell_what_the_poses_do(plan)
    labels_fit_the_geometry(plan)


def keeps_the_rules(plan, r):
    """The fixed rules (the acceptance's gates): the body no more than 1 cm out of where it may be off the outermost
    loop, that loop no more than 1 cm out of the space the body took while recording, the centre on the allowed areas
    only, never a step against the heading, no segment backwards, no two poses on one spot."""
    check_invariants(plan, r['cfg'], r['obstacles'])
    ref = metrics.Ref(lawn(r), r)
    got = dict(metrics.rules(plan, ref), **metrics.plan_body(plan, ref))
    for key in ('centre_out', 'backwards', 'reverse_segs', 'dup_steps', 'rm_out', 'ring_out', 'blade_over'):
        assert not got.get(key), (key, got.get(key), got.get(key + '_at'))
    assert not got.get('cfg_diff')


def stats_tell_what_the_poses_do(plan):
    """The lengths are the segments' (each kind's on its own), the counts the plan's; every key the node and the app
    read is there, as JSON (no nan); the counted warnings the app reads say what stats['body_fit'] does."""
    st, poses = plan.stats, plan.poses
    length = {k: 0.0 for k in (PERIMETER, LANE, TURN, TRANSIT)}
    for s in plan.segments:
        length[s.kind] += sum(math.dist(poses[i][:2], poses[i + 1][:2]) for i in range(s.start, s.end))
    assert set(STATS) <= set(st)
    assert st['path_length'] == pytest.approx(sum(length.values()), abs=1e-6)
    for kind in length:
        assert st[kind + '_length'] == pytest.approx(length[kind], abs=1e-6), kind
    assert st['poses'] == len(poses) and st['sections'] == len(plan.section_starts)
    # (a turn of the catalogue a turn segment, one none fits a drive; counted on the route before the safety net and the
    # completion stage, which may drive one another way or put a detour into it)
    if not any(s.maneuver in ('body', 'mop_up') for s in plan.segments):
        assert st['turns_u_turn'] + st['turns_bulb'] + st['turns_detour'] + st['turns_pivot'] == \
            sum(1 for s in plan.segments if s.kind == TURN)
        assert st['turns_fallback'] == sum(1 for s in plan.segments if s.maneuver == 'turn_fallback')
    assert st['lanes'] == len(plan.lanes) and st['chosen']['lane_spacing'] == pytest.approx(plan.lane_spacing, abs=1e-3)
    assert st['chosen']['perimeter_passes'] == plan.config.passes
    json.dumps(to_openmower_json(plan), allow_nan=False)
    fit = st.get('body_fit')
    counted = {i: [int(m[1]) for m in (p.match(w) for w in plan.warnings) if m] for i, p in enumerate(COUNTED)}
    assert counted[0] == ([fit['fixed']] if fit and fit['fixed'] else [])
    assert counted[1] == ([fit['left']] if fit and fit['left'] else [])
    if not plan.config.body:
        assert plan.warnings[0] == plan.config.no_body


def labels_fit_the_geometry(plan):
    """Segments say what they are: a turn leads into the lane it names (from a lane, or the safety net's way where
    the body didn't fit at its end), each section starts with its first segment; the loops
    (perimeter_loops) have 3 points at least; a lane or loop mowed in pieces has them one after the other."""
    segs = plan.segments
    for i, s in enumerate(segs):
        if s.kind == TURN:
            assert s.source.startswith('lane-') and i + 1 < len(segs) and segs[i + 1].kind == LANE and \
                segs[i + 1].source == s.source, i
            assert i > 0 and (segs[i - 1].kind == LANE or segs[i - 1].maneuver == 'body'), i
    starts = {s.start for i, s in enumerate(segs) if i == 0 or s.section != segs[i - 1].section}
    assert starts == set(plan.section_starts)
    assert [s.section for s in segs] == sorted(s.section for s in segs)
    for loop in plan.perimeter_loops:
        assert len(loop) >= 3
    mowed = {}
    for s in segs:
        if s.kind in (PERIMETER, LANE) and s.source:
            mowed.setdefault(s.source, []).append(s)
    for source, pieces in mowed.items():  # (a loop or lane in pieces: one after the other, never twice over)
        assert all(a.end <= b.start for a, b in zip(pieces, pieces[1:])), source


# the same plan wherever and however it's planned ---------------------------------------------------------------


def test_same_plan_in_threads():
    """Two areas planned at once in two threads: the plans of each alone (no state shared between plans)."""
    keys = [('garden', 'outline', ''), ('slot', 'rect', '')]
    alone = [fingerprint(plan_of(*k)) for k in keys]
    got = {}

    def run(k):
        r = request(*k)
        got[k] = fingerprint(plan_coverage(r['boundary'], r['obstacles'], r['cfg'], r['start'], r['end'],
                                           r['drivable'], r['keep_out'], r['margins']))

    threads = [threading.Thread(target=run, args=(k,)) for k in keys]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert [got[k] for k in keys] == alone


def test_same_plan_with_another_hash_seed():
    """A process of its own with another PYTHONHASHSEED (the workers' are spawned): the same plan."""
    code = ('import sys; sys.path[:0] = %r; import test_invariants as t; '
            'from mowbite_planner.export import fingerprint; print(fingerprint(t.plan_of("garden", "rect", "")))'
            % [HERE])
    env = dict(os.environ, PYTHONHASHSEED='12345')
    out = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, timeout=600)
    assert out.returncode == 0, out.stderr[-2000:]
    assert out.stdout.split()[-1] == fingerprint(plan_of('garden', 'rect'))


# made up gardens with the body ----------------------------------------------------------------------------------


def random_polygon(rnd, cx, cy, r_min, r_max, n):
    """One that doesn't cross itself (the measuring tools take a crossing outline otherwise than the planner)."""
    while True:
        angles = sorted(rnd.uniform(0, 2 * math.pi) for _ in range(n))
        pts = [(cx + rnd.uniform(r_min, r_max) * math.cos(a), cy + rnd.uniform(r_min, r_max) * math.sin(a))
               for a in angles]
        if Polygon(pts).is_valid:
            return pts


@pytest.mark.parametrize('seed', range(12))
def test_random_gardens_with_the_body(seed):
    """Made up outlines with islands (some close to the edge or each other), a mower with its body, some settings at
    random: a plan that keeps every rule, or a PlanningError, never anything else."""
    rnd = random.Random(7000 + seed)
    boundary = random_polygon(rnd, 0, 0, 3, 8, rnd.randint(5, 14))
    obstacles = [random_polygon(rnd, rnd.uniform(-4, 4), rnd.uniform(-4, 4), 0.2, 1.0, rnd.randint(3, 8))
                 for _ in range(rnd.randint(0, 4))]
    settings = dict(BASE, **BODIES[('rect', 'outline')[seed % 2]], perimeter_passes=rnd.choice([1, 2, 3, 4]),
                    obstacle_passes=rnd.choice([-1, 0, 1]), lane_order=rnd.choice(['snake', 'skip']),
                    turn_on_spot=rnd.random() < 0.3, edges=rnd.choice(['recorded', 'recorded', 'hard']),
                    lane_spacing_mode=rnd.choice(['fixed', 'auto']))
    r = metrics.request(boundary, obstacles, PlannerConfig(**settings))
    try:
        plan = plan_coverage(r['boundary'], r['obstacles'], r['cfg'])
    except PlanningError:
        return
    keeps_the_rules(plan, r)
    stats_tell_what_the_poses_do(plan)
    labels_fit_the_geometry(plan)


def test_same_plan_every_time():
    """A garden of simbench with the blade off the middle (the loops it drives with the blade inside were known by
    their id, a ring made later could get a freed one's and go the other way round): the same plan every time."""
    import gardens
    import simbench
    case = 'model/0@rect+perimeter_passes=3+obstacle_passes=-1+pairs=28'
    name, index, mower_name, _, settings = simbench.parse(case)
    garden = gardens.garden(name)
    area_id = garden.areas[index]
    garden, mower = simbench.with_settings(garden, area_id, settings), gardens.mower_of(mower_name, garden)
    got = {fingerprint(simbench.plan_area(garden, area_id, mower)) for _ in range(12)}
    assert len(got) == 1
