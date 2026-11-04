"""Tests of starts.py: a plan's sections where a robot that drives to them itself gets to them (MowBite OS)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import PlannerConfig, PlanningError, plan_coverage  # noqa: E402
from mowbite_planner.starts import ARRIVE_ASKS, reached_starts  # noqa: E402

# a hedge across the lawn: two sections, the path jumps from one to the other
LAWN = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
HEDGE = [(4.0, -1.0), (6.0, -1.0), (6.0, 7.0), (4.0, 7.0)]
CFG = PlannerConfig(mower_width=0.3, turn_radius=0.3)
_plans = {}


def plan():
    if 'split' not in _plans:
        _plans['split'] = plan_coverage(LAWN, [HEDGE], CFG)
    return _plans['split']


def sections(p):
    ends = list(p.section_starts[1:]) + [len(p.poses)]
    return [p.poses[a:b] for a, b in zip(p.section_starts, ends)]


def test_every_start_reached_leaves_the_plan():
    p = plan()
    asked = []
    got = reached_starts(p, lambda pose, came: asked.append(pose) or True)
    assert got.poses == p.poses and got.segments == p.segments and got.warnings == p.warnings
    assert len(asked) == len(p.section_starts) == 2
    assert got.stats['starts'] == {'places': [], 'dropped': [], 'untried': [], 'unmowed_m2': 0.0, 'asks': 2}


def ends(p):
    """Each section's last pose."""
    return [max((s for s in p.segments if s.section == p.segments[k].section), key=lambda s: s.end).end
            for k in range(len(p.segments)) if k == 0 or p.segments[k].section != p.segments[k - 1].section]


def test_where_the_robot_comes_from():
    # the first section from wherever the robot is (None), each other one from where the section before it that's kept
    # ends: a section left out leaves the robot where it was
    if 'three' not in _plans:
        _plans['three'] = plan_coverage([(0.0, 0.0), (14.0, 0.0), (14.0, 6.0), (0.0, 6.0)],
                                        [[(4.0, -1.0), (5.0, -1.0), (5.0, 7.0), (4.0, 7.0)],
                                         [(9.0, -1.0), (10.0, -1.0), (10.0, 7.0), (9.0, 7.0)]], CFG)
    p = _plans['three']
    assert len(p.section_starts) == 3
    asked = []
    reached_starts(p, lambda pose, came: asked.append((pose, came)) or True)
    assert [came for _, came in asked] == [None, p.poses[ends(p)[0]], p.poses[ends(p)[1]]]
    gone = {(q.x, q.y, q.yaw) for q in sections(p)[1]}
    asked.clear()
    got = reached_starts(p, lambda pose, came: asked.append((pose, came)) or (pose.x, pose.y, pose.yaw) not in gone)
    assert len(got.section_starts) == 2 and len(got.stats['starts']['dropped']) == 1
    third = {(q.x, q.y, q.yaw) for q in sections(p)[2]}
    froms = [came for pose, came in asked if (pose.x, pose.y, pose.yaw) in third]
    assert froms and all(came is p.poses[ends(p)[0]] for came in froms)


def test_a_section_starts_later_where_it_gets_to():
    p = plan()
    first = p.poses[p.section_starts[1]]
    # the second section's first metre can't be got to
    far = [k for k in range(p.section_starts[1], len(p.poses))
           if math.hypot(p.poses[k].x - first.x, p.poses[k].y - first.y) > 1.0][0]
    blocked = {(q.x, q.y, q.yaw) for q in p.poses[p.section_starts[1]:far]}
    got = reached_starts(p, lambda pose, came: (pose.x, pose.y, pose.yaw) not in blocked)
    assert len(got.section_starts) == 2
    later = got.poses[got.section_starts[1]]
    assert (later.x, later.y, later.yaw) not in blocked and math.hypot(later.x - first.x, later.y - first.y) > 1.0
    # the poses before it gone, the rest as planned, the segments along
    assert got.poses[got.section_starts[1]:] == p.poses[p.poses.index(later):]
    assert got.segments[0].start == 0 and got.segments[-1].end == len(got.poses) - 1
    assert all(a.end in (b.start, b.start - 1) for a, b in zip(got.segments, got.segments[1:]))
    places = got.stats['starts']['places']
    assert len(places) == 1 and places[0][:2] == [round(first.x, 2), round(first.y, 2)] and places[0][2] >= 1.0
    assert got.stats['starts']['unmowed_m2'] > 0.0
    assert any('start later' in w for w in got.warnings)


def test_a_section_it_gets_to_nowhere_is_left_out():
    p = plan()
    gone = {(q.x, q.y, q.yaw) for q in sections(p)[1]}
    got = reached_starts(p, lambda pose, came: (pose.x, pose.y, pose.yaw) not in gone)
    # no start it can't get to: the section is left out, not started as planned
    assert got.section_starts == [0] and got.poses == sections(p)[0]
    assert got.stats['starts']['dropped'] == [[round(sections(p)[1][0].x, 2), round(sections(p)[1][0].y, 2)]]
    assert any('left out' in w for w in got.warnings)
    with pytest.raises(PlanningError, match='no way gets to any section'):
        reached_starts(p, lambda pose, came: False)


def test_the_asks_are_counted():
    p = plan()
    asked = []
    with pytest.raises(PlanningError):
        reached_starts(p, lambda pose, came: bool(asked.append(pose)))
    # every section's first pose and up to ARRIVE_TRIES - 1 later ones, never more than ARRIVE_ASKS (a number: the same
    # plan on any computer; a section not asked about past that is left out too)
    assert 2 < len(asked) <= ARRIVE_ASKS


def test_plan_coverage_passes_arrives():
    seen = []
    got = plan_coverage(LAWN, [HEDGE], CFG, arrives=lambda pose, came: seen.append(pose) or True)
    assert got.poses == plan().poses and len(seen) == 2 and 'starts' in got.stats
    assert 'starts' not in plan().stats
