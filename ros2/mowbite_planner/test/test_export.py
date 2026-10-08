"""Tests of the exports (export.py): OpenMower's paths, the Planner Studio payload, the fingerprint."""

import dataclasses
import json
import math
import os
import random
import sys
from types import SimpleNamespace

import pytest
from shapely.geometry import Polygon

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import PlannerConfig, SegmentInfo, plan_coverage  # noqa: E402
from mowbite_planner.export import (SEGMENT_TYPES, fingerprint, plan_pieces, simplified_indices,  # noqa: E402
                                    to_openmower_json, to_studio_payload)
from mowbite_planner.config import LANE, PERIMETER, TRANSIT, TURN  # noqa: E402
from mowbite_planner.report import FTC_SPEEDS, check_plan, drive_time  # noqa: E402


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


AREA = rect(0, 0, 10, 6)
BED = rect(4, 2, 5, 3)
_plans = {}


def plan_of(**kw):
    """A plan of AREA with BED, planned once per setting."""
    key = tuple(sorted(kw.items()))
    if key not in _plans:
        _plans[key] = plan_coverage(AREA, [BED], PlannerConfig(**kw))
    return _plans[key]


def fake(*segments):
    """Just the segments of a plan: (kind, first pose, last pose[, section])."""
    return SimpleNamespace(segments=[SegmentInfo(s[0], s[1], s[2], False, '', s[3] if len(s) > 3 else 0)
                                     for s in segments])


# pieces ----------------------------------------------------------------------------------------------------


def test_plan_pieces():
    plan = fake((PERIMETER, 0, 10), (TRANSIT, 10, 12), (PERIMETER, 12, 20), (TRANSIT, 20, 25), (LANE, 25, 40),
                (TURN, 40, 42), (LANE, 42, 60), (PERIMETER, 61, 62))
    # outline, then fill, the transit with the lanes after it. 2 poses aren't a path for OpenMower
    assert plan_pieces(plan) == [(True, 0, 20), (False, 20, 60)]
    # longer runs in equal pieces of at most max_poses poses, each starting on the pose the last one ended on
    assert plan_pieces(plan, 10) == [(True, 0, 6), (True, 6, 13), (True, 13, 20), (False, 20, 28), (False, 28, 36),
                                     (False, 36, 44), (False, 44, 52), (False, 52, 60)]
    assert plan_pieces(plan, 20) == [(True, 0, 10), (True, 10, 20), (False, 20, 33), (False, 33, 46),
                                     (False, 46, 60)]
    assert plan_pieces(plan, 3) == [(o, a, a + 2) for o, s, e in plan_pieces(plan) for a in range(s, e, 2)]


@pytest.mark.parametrize('max_poses', [1, 3, 4, 5, 9])
def test_plan_pieces_have_3_poses_at_least(max_poses):
    for n in range(3, 40):
        pieces = plan_pieces(fake((PERIMETER, 0, n - 1), (LANE, n - 1, 2 * n)), max_poses)
        assert pieces[0][1] == 0 and pieces[-1][2] == 2 * n
        assert all(b == a for (_, _, b), (_, a, _) in zip(pieces, pieces[1:]))
        sizes = [b - a + 1 for _, a, b in pieces]
        if max_poses > 3:
            assert all(3 <= size <= max_poses for size in sizes)
        else:
            # a run of an odd number of steps can't be cut into pieces of 3: one of them gets 4
            assert set(sizes) <= {3, 4} and sizes.count(4) <= 2


def test_plan_pieces_sections():
    # a part of the area with no way to it is a path of its own, OpenMower drives to its start itself
    plan = fake((PERIMETER, 0, 10, 0), (LANE, 10, 30, 0), (PERIMETER, 31, 40, 1), (TRANSIT, 40, 45, 1),
                (LANE, 45, 60, 1), (LANE, 61, 62, 2))
    assert plan_pieces(plan) == [(True, 0, 10), (False, 10, 30), (True, 31, 40), (False, 40, 60)]
    # one perimeter run across two sections is two paths
    plan = fake((PERIMETER, 0, 10, 0), (PERIMETER, 11, 20, 1), (TRANSIT, 20, 22, 1), (PERIMETER, 22, 30, 1))
    assert plan_pieces(plan) == [(True, 0, 10), (True, 11, 30)]


@pytest.mark.parametrize('kw, outline', [
    ({}, [True, False]),
    (dict(perimeter_order='last'), [False, True]),
    (dict(perimeter_passes=0), [False]),
    (dict(skip_lanes=True), [True]),
])
def test_plan_pieces_of_a_plan(kw, outline):
    plan = plan_of(**kw)
    pieces = plan_pieces(plan)
    assert [o for o, _, _ in pieces] == outline
    assert pieces[0][1] == 0 and pieces[-1][2] == len(plan.poses) - 1
    for (_, _, end), (_, start, _) in zip(pieces, pieces[1:]):
        assert start in (end, end + 1)
    # every segment in one piece, a transit in the one of what comes after it
    for i, s in enumerate(plan.segments):
        piece = next(p for p in pieces if p[1] <= s.start and s.end <= p[2])
        after = next((t.kind for t in plan.segments[i:] if t.kind != TRANSIT), None)
        if after is not None:
            assert piece[0] == (after == PERIMETER)
    # split, every piece is short enough and long enough, together they're the same
    split = plan_pieces(plan, 500)
    assert len(split) > len(pieces)
    assert all(3 <= b - a + 1 <= 500 for _, a, b in split)
    covered = set()
    for _, a, b in split:
        covered.update(range(a, b + 1))
    assert covered == set(range(len(plan.poses)))


def test_concentric_rings_are_fill():
    plan = plan_of(fill_pattern='concentric')
    for is_outline, a, b in plan_pieces(plan):
        kinds = {s.kind for s in plan.segments if a <= s.start and s.end <= b} - {TRANSIT}
        assert kinds == ({PERIMETER} if is_outline else {LANE})


# fingerprint -----------------------------------------------------------------------------------------------


def test_fingerprint():
    plan = plan_of()
    fp = fingerprint(plan)
    assert len(fp) == 64 and int(fp, 16) >= 0
    assert fingerprint(plan_coverage(AREA, [BED], PlannerConfig())) == fp  # planned again
    assert fingerprint(plan_of(perimeter_passes=1)) != fp
    assert fingerprint(plan_coverage(AREA, [rect(4, 2, 5, 3.1)], PlannerConfig())) != fp
    last = plan.poses[-1]
    moved = dataclasses.replace(plan, poses=plan.poses[:-1] + [last._replace(yaw=last.yaw + 1e-12)])
    assert fingerprint(moved) != fp


# simplifying -----------------------------------------------------------------------------------------------


def test_simplified_indices():
    assert simplified_indices([]) == []
    assert simplified_indices([(0, 0)]) == [0]
    assert simplified_indices([(0, 0), (0, 0)]) == [0, 1]
    assert simplified_indices([(0.1 * i, 0.05 * i) for i in range(50)]) == [0, 49]  # straight
    assert simplified_indices([(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]) == [0, 2, 4]  # a corner
    assert simplified_indices([(0, 0), (1, 0.009), (2, 0)]) == [0, 2]
    assert simplified_indices([(0, 0), (1, 0.011), (2, 0)]) == [0, 1, 2]
    assert simplified_indices([(0, 0), (1, 0.03), (2, 0)], 0.05) == [0, 2]
    # a closed loop keeps its corners
    assert simplified_indices([(0, 0), (0.5, 0), (1, 0), (1, 1), (0, 1), (0, 0)]) == [0, 2, 3, 4, 5]


@pytest.mark.parametrize('seed', range(5))
def test_simplified_indices_keep_to_the_tolerance(seed):
    rnd = random.Random(seed)
    pts, x, y, h = [], 0.0, 0.0, 0.0
    for _ in range(300):
        h += rnd.gauss(0, 0.2)
        x, y = x + 0.05 * math.cos(h), y + 0.05 * math.sin(h)
        pts.append((x, y))
    tol = rnd.choice([0.005, 0.01, 0.05])
    keep = simplified_indices(pts, tol)
    assert keep[0] == 0 and keep[-1] == len(pts) - 1 and keep == sorted(set(keep))
    assert len(keep) < len(pts)
    for i, j in zip(keep, keep[1:]):
        (ax, ay), (bx, by) = pts[i], pts[j]
        length = math.hypot(bx - ax, by - ay)
        for px, py in pts[i + 1:j]:
            # distance to the line through the kept points around it, like Douglas-Peucker
            assert abs((by - ay) * (px - ax) - (bx - ax) * (py - ay)) / length <= tol + 1e-12


# OpenMower -------------------------------------------------------------------------------------------------


def test_to_openmower_json():
    plan = plan_of()
    out = to_openmower_json(plan, 'front')
    assert json.loads(json.dumps(out, allow_nan=False)) == out
    assert out['area_id'] == 'front' and out['fingerprint'] == fingerprint(plan)
    assert out['angle'] == plan.angle and out['angles'] == plan.angles and out['warnings'] == []
    pieces = plan_pieces(plan)
    assert len(out['paths']) == len(pieces)
    for path, (is_outline, start, end) in zip(out['paths'], pieces):
        assert path['is_outline'] == is_outline
        assert path['poses'] == end - start + 1
        index = path['pose_index']
        assert index[0] == 0 and index[-1] == path['poses'] - 1 and index == sorted(set(index))
        assert len(index) == len(path['points']) < path['poses']  # simplified
        for k, (x, y) in zip(index, path['points']):
            p = plan.poses[start + k]
            assert (x, y) == (round(p.x, 3), round(p.y, 3))  # mm
        for s in path['segments']:
            assert s['type'] in SEGMENT_TYPES and 0 <= s['start'] < s['end'] <= path['poses'] - 1
    assert sum(len(p['segments']) for p in out['paths']) >= len(plan.segments)
    # unsimplified and split
    full = to_openmower_json(plan, tolerance=0.0, max_poses=500)
    assert len(full['paths']) == len(plan_pieces(plan, 500))
    assert all(p['pose_index'] == list(range(p['poses'])) for p in full['paths'])


def test_body_edges_of_the_answer():
    """With the body checked the answer says where it keeps to: body_space's rings, 1 cm simpler and in mm, the outline
    of the lawn where it was while its centre drove the recorded line, edge_margin off (all of it, body_tolerance
    doesn't count), and the bed's hole inside it."""
    assert 'body_space' not in to_openmower_json(plan_of())
    plan = plan_of(robot_width=0.4, robot_front=0.43, robot_rear=0.14, perimeter_offset=0.0)
    edges = to_openmower_json(plan)['body_space']
    assert json.loads(json.dumps(edges)) == edges and [len(edges['outlines']), len(edges['holes'])] == [1, 1]
    assert all(round(c, 3) == c for ring in edges['outlines'] + edges['holes'] for p in ring for c in p)
    space = Polygon(edges['outlines'][0], edges['holes'])
    assert space.symmetric_difference(plan.body_space).area < 0.01 * plan.body_space.length
    cfg = plan.config
    out = 0.5 * cfg.robot_width - cfg.edge_margin
    outline = Polygon(edges['outlines'][0])
    assert Polygon(AREA).buffer(out - 0.02).within(outline)
    assert outline.within(Polygon(AREA).buffer(out + 0.3, join_style=2))  # (the front past the corners)
    assert Polygon(edges['holes'][0]).within(Polygon(BED))


def test_body_fit_of_the_answer():
    """The answer's stats say where the path goes another way for the body and leaves out what (body_fit): its places
    [x, y, m left out] and their poses [x, y, yaw, m the body sticks out there, 1 where it turns on the spot there] one
    for one, the place where its pose is."""
    plan = plan_of(robot_width=0.4, robot_front=0.43, robot_rear=0.14, perimeter_offset=0.0)
    fit = json.loads(json.dumps(to_openmower_json(plan)['stats']['body_fit']))
    assert fit == plan.stats['body_fit'] and len(fit['poses']) == len(fit['places']) == fit['fixed'] + fit['left'] > 0
    for (x, y, _), (px, py, yaw, d, spin) in zip(fit['places'], fit['poses']):
        assert abs(x - px) <= 0.0051 and abs(y - py) <= 0.0051 and -math.pi <= yaw <= math.pi and d >= 0
        assert spin in (0, 1) and round(d, 3) == d


# Planner Studio --------------------------------------------------------------------------------------------


@pytest.mark.parametrize('kw', [{}, dict(fill_pattern='concentric'), dict(fill_pattern='crosshatch')])
def test_to_studio_payload(kw):
    plan = plan_of(**kw)
    nav = [rect(-4, 0, 0, 2)]
    out = to_studio_payload(plan, AREA, [BED], nav, {'tool_width': 0.2}, ['outline_compensation'])
    json.dumps(out, allow_nan=False)
    assert [s['key'] for s in out['stages']] == ['area', 'levels', 'remaining', 'cells', 'loops', 'lanes', 'route',
                                                 'ends']
    assert [s['n'] for s in out['stages']] == [1, 2, 3, 4, 5, 6, 7, 8]
    objects = out['lineage']['objects']
    ids = [o['id'] for o in objects]
    assert len(ids) == len(set(ids))
    assert sorted(i for s in out['stages'] for i in s['ids']) == sorted(ids)
    assert all(p in ids for o in objects for p in o['parents'])
    for o in objects:
        g = o['geometry']
        assert all(len(c) == 2 and all(isinstance(v, float) and math.isfinite(v) for v in c) for c in g['coords'])
        if g['type'] == 'polygon':
            assert len(g['coords']) >= 4 and g['coords'][0] == g['coords'][-1]
        elif g['type'] == 'line':
            assert len(g['coords']) >= 2
        else:
            assert g['type'] == 'point' and len(g['coords']) == 1
    by_stage = {s['key']: s['ids'] for s in out['stages']}
    assert len(by_stage['route']) == len(plan.segments) and len(by_stage['lanes']) == len(plan.lanes)
    assert len(by_stage['loops']) == len(plan.perimeter_loops)
    by_id = {o['id']: o for o in objects}
    # every lane comes from a cell, every loop from a level (or a strip too narrow for lanes), every segment
    # from the loop or lane it mows or leads to, those from the mowable area
    for i in by_stage['lanes']:
        assert by_id[i]['parents'][0].startswith('cell-')
    for i in by_stage['loops']:
        assert by_id[i]['parents'][0].split('-')[0] in ('level', 'narrow')
    assert all(by_id[i]['parents'] for i in by_stage['route'][1:])
    assert all(by_id[i]['parents'] == by_stage['area'] for i in by_stage['levels'])
    assert all(by_id[i]['parents'][0].startswith('lane-region') for i in by_stage['cells'])
    assert {e['op'] for e in out['lineage']['events']} >= {'difference', 'inset', 'scan', 'route'}
    assert out['report']['hard'] == []
    assert out['report']['soft'][-1] == 'parameter outline_compensation is not used by the MowBite Planner'
    assert out['stats']['left_m2'] < 0.1 and out['stats']['double_mowed_pct'] > 0
    assert out['params'] == {'tool_width': 0.2}
    assert out['domains']['nav_areas'] == [[[float(x), float(y)] for x, y in nav[0]]]
    assert out['domains']['expected_cover_m2'] == pytest.approx(59.0)
    assert set(out['stage_times']) == set(plan.stats['timings'])
    assert out['mowbite']['fingerprint'] == out['stats']['fingerprint'] == fingerprint(plan)


# report ----------------------------------------------------------------------------------------------------


def test_check_plan_finds_nothing_in_a_good_plan():
    report = check_plan(plan_of())
    assert report['hard'] == []
    assert report['area_m2'] == pytest.approx(59.0) and report['left_m2'] < 0.1
    assert report['mowed_m2'] == pytest.approx(59.0, abs=0.1)
    # lanes 0.187 m apart with a 0.22 m blade, and the loops: a good part is mowed twice
    assert 10 < report['double_mowed_pct'] < 80


def test_check_plan_finds_what_is_wrong():
    plan = plan_of()
    poses = list(plan.poses)
    k = plan.segments[-1].start + 2
    poses[k] = poses[k]._replace(x=poses[k].x + 3.0, y=-2.0)  # off the area, and a gap
    report = check_plan(dataclasses.replace(plan, poses=poses))
    assert any('outside' in h for h in report['hard']) and any('gaps' in h for h in report['hard'])
    backwards = dataclasses.replace(plan, segments=[dataclasses.replace(plan.segments[0], reverse=True)] +
                                    plan.segments[1:])
    assert 'drives backwards' in check_plan(backwards)['hard']


def test_check_plan_tells_left_on_purpose_from_left_out():
    # a body that keeps the blade off the edges: a strip left on purpose, not a mistake
    body = plan_coverage(AREA, [BED], PlannerConfig(robot_width=0.5, edges='hard'))
    report = check_plan(body)
    assert report['hard'] == [] and report['edge_left_m2'] > 0.5
    assert any('left on purpose' in s for s in report['soft'])
    # lanes along the edge that a blade offset with no loops leaves out: grass left
    short = plan_coverage(AREA, [], PlannerConfig(blade_offset=0.1, perimeter_passes=0, angle=0.0))
    report = check_plan(short)
    assert any('left unmowed' in h for h in report['hard']) and any('shortened' in s for s in report['soft'])


def test_drive_time():
    """drive_time: fast on a long straight way, slow where it bends within speed_fast_threshold ahead, turning on the
    spot at speed_angular, the FTC's speeds given or the defaults."""
    from mowbite_planner import Pose2D
    straight = [Pose2D(0.1 * i, 0.0, 0.0) for i in range(101)]  # 10 m
    t = drive_time(straight)
    assert t['fast_m'] == pytest.approx(10.0) and t['slow_m'] == 0.0 and t['spin_s'] == 0.0
    assert t['s'] == pytest.approx(10.0 / 0.4 + 0.5 * 0.25 / 0.1)
    bend = straight + [Pose2D(10.0, 0.1 * i, math.pi / 2) for i in range(1, 11)]  # (the corner slow)
    assert drive_time(bend)['slow_m'] == pytest.approx(1.0, abs=0.15)
    back = straight + [Pose2D(10.0, 0.0, math.pi)] + [Pose2D(10.0 - 0.1 * i, 0.0, math.pi) for i in range(1, 11)]
    assert drive_time(back)['spin_s'] == pytest.approx(math.pi / math.radians(FTC_SPEEDS['speed_angular']))
    assert drive_time(straight, {'speed_fast': 0.8, 'speed_slow': 0.3})['s'] == pytest.approx(10.0 / 0.8 + 0.5 * 0.5 / 0.1)

