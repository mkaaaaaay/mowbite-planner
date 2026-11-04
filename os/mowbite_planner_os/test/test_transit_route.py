"""Tests of plan_transit (the node's ~/plan_transit) without ROS."""

import math
import os
import sys

import pytest
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', '..', '..', 'ros2', 'mowbite_planner')]

from mowbite_planner import PlannerConfig, PlanningError  # noqa: E402
from mowbite_planner.footprint import SPIN, Footprint  # noqa: E402
from mowbite_planner.mathutil import angle_diff  # noqa: E402
from mowbite_planner.planner import plan_coverage  # noqa: E402
from mowbite_planner.space import _body_space  # noqa: E402
from mowbite_planner_os.transit_route import plan_transit  # noqa: E402

# Airseekers Tron (mowerBody.ts, not measured yet): its nose swings ~56 cm out turning on the spot
TRON = dict(mower_width=0.22, turn_radius=0.27, robot_width=0.47, robot_front=0.54, robot_rear=0.178)
# two lawns joined by a navigation area, a tree in the first
LAWN = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
NAV = [(10.0, 2.0), (14.0, 2.0), (14.0, 3.2), (10.0, 3.2)]
BACK = [(14.0, -2.0), (20.0, -2.0), (20.0, 8.0), (14.0, 8.0)]
AREAS = [LAWN, NAV, BACK]
TREE = [(5 + 0.6 * math.cos(a * math.pi / 8), 3 + 0.6 * math.sin(a * math.pi / 8)) for a in range(16)]


def transit(start, goal, cfg=None, areas=AREAS, obstacles=(TREE,), **kw):
    return plan_transit(start, goal, areas, list(obstacles), PlannerConfig(**dict(TRON, **(cfg or {}))), **kw)


def check(t, start, goal, cfg=None, areas=AREAS, obstacles=(TREE,)):
    """The way's rules: at least 3 poses, none twice, each facing the next, from start to goal, the body in where it's
    checked, the spins where the heading jumps."""
    poses = t.poses
    assert len(poses) >= 3
    assert (poses[0].x, poses[0].y) == pytest.approx(start[:2])
    assert (poses[-1].x, poses[-1].y) == pytest.approx(goal[:2])
    if len(goal) > 2:
        assert angle_diff(poses[-1].yaw, goal[2]) < 1e-9
    for a, b in zip(poses, poses[1:]):
        assert math.hypot(b.x - a.x, b.y - a.y) > 1e-6
        assert angle_diff(a.yaw, math.atan2(b.y - a.y, b.x - a.x)) < math.radians(6)
    assert t.length == pytest.approx(sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:])))
    spins = [i for i in range(1, len(poses)) if angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN]
    assert t.spins == ([0] if angle_diff(poses[0].yaw, start[2]) > SPIN else []) + spins
    cfg = PlannerConfig(**dict(TRON, **(cfg or {})))
    if any('as the outline was recorded' in w for w in t.warnings):  # (by the edge as the outermost loop)
        cfg = cfg.with_overrides({'edge_margin': min(cfg.recorded_margin, cfg.edge_margin), 'spin_margin': 0.0})
    space, _ = _body_space(list(areas), list(obstacles), [cfg.obstacle_margin] * len(obstacles), cfg)
    fp = Footprint(space, cfg.body_outline, cfg.spin_margin)
    # (the stretches outside, start_outside and goal_outside long, unchecked)
    first = next(i for i, p in enumerate(poses) if math.hypot(p.x - start[0], p.y - start[1]) >= t.start_outside - 1e-9)
    last = max(i for i, p in enumerate(poses) if math.hypot(p.x - goal[0], p.y - goal[1]) >= t.goal_outside - 1e-9)
    before = [poses[first - 1]] if first else []
    assert [i for i in fp.bad_poses(before + poses[first:last + 1]) if i >= len(before)] == []
    if not first and 0 in t.spins:
        assert fp.fits_turn(start[0], start[1], start[2], poses[0].yaw)
    lawn = unary_union([Polygon(a) for a in areas])
    assert all(lawn.covers(Point(p.x, p.y)) for p in poses[first:last + 1])


@pytest.mark.parametrize('start, goal', [
    ((1.0, 1.0, 0.0), (8.0, 5.0)),
    ((2.0, 3.0, 0.0), (8.0, 3.0, 0.0)),  # round the tree
    ((2.0, 3.0, math.pi), (17.0, 5.0, math.pi / 2)),  # facing away, through the navigation area
    ((1.0, 3.0, 0.0), (0.8, 3.0)),  # just behind
    ((17.0, 0.0, -math.pi / 2), (3.0, 5.0, math.pi)),
])
def test_transit(start, goal):
    t = transit(start, goal)
    check(t, start, goal)
    assert t.start_outside == 0 and t.goal_outside == 0
    # arcs, no turning on the spot where there's room
    assert t.spins == []
    assert t.length < 3 * math.hypot(goal[0] - start[0], goal[1] - start[1]) + 4.0
    assert t.warnings == []


@pytest.mark.parametrize('start, goal, cfg, kw', [
    # the goal by the edge without a heading: arriving facing a way the body fits there
    ((9.0, 1.13, -2.75), (18.27, 7.7), {}, {}),
    ((2.09, 4.53, -0.32), (19.76, 5.19), {}, {}),
    # into the navigation area round its corner: the nose would swing over the edge turning on the spot right there
    ((15.44, -1.44, -0.41), (7.53, 0.94), {}, {}),
    # straight before the goal by the tree
    ((1.08, 3.09, -2.04), (4.3, 2.35, -1.57), {}, {'goal_straight': 0.5}),
    # turning on the spot into the goal's heading by the edge doesn't fit: on arcs
    ((12.85, 2.87, -1.0), (14.21, 7.75, 2.5), {'turn_on_spot': True}, {}),
])
def test_tight_places(start, goal, cfg, kw):
    t = transit(start, goal, cfg, **kw)
    check(t, start, goal, cfg)


def test_same_request_same_poses():
    a = transit((2.0, 3.0, math.pi), (17.0, 5.0, math.pi / 2))
    b = transit((2.0, 3.0, math.pi), (17.0, 5.0, math.pi / 2))
    assert a.poses == b.poses


def test_turn_on_spot():
    start, goal = (2.0, 3.0, 0.0), (8.0, 3.0, math.pi)
    arcs = transit(start, goal)
    check(arcs, start, goal)
    assert arcs.spins == []
    spot = transit(start, goal, {'turn_on_spot': True})
    check(spot, start, goal, {'turn_on_spot': True})
    assert spot.spins and spot.length < arcs.length


def test_spin_where_no_arc_fits():
    # a dead end the body only just fits in: no arc turns it round, on the spot it does (a warning)
    areas = [[(0.0, 0.0), (6.0, 0.0), (6.0, 1.3), (0.0, 1.3)]]
    start, goal = (5.0, 0.65, 0.0), (1.0, 0.65)
    t = transit(start, goal, areas=areas, obstacles=())
    check(t, start, goal, areas=areas, obstacles=())
    assert t.spins and any('turns on the spot' in w for w in t.warnings)


def test_dock_straight_before():
    # the dock by the edge of the back lawn: arriving facing +x, 0.5 m straight before it, the nose over the edge
    start, goal = (17.0, 5.0, math.pi), (19.8, 4.0, 0.0)
    t = transit(start, goal, goal_straight=0.5)
    check(t, start, goal)
    assert 0 < t.goal_outside <= 0.5
    straight = [p for p in t.poses if math.hypot(p.x - goal[0], p.y - goal[1]) <= 0.5 + 1e-9]
    assert len(straight) >= 5
    assert all(angle_diff(p.yaw, 0.0) < 1e-9 and abs(p.y - 4.0) < 1e-9 for p in straight)
    assert any('goal lies outside' in w for w in t.warnings)


def test_from_the_dock():
    # backed off the dock facing into the lawn, the rear still over the edge
    start, goal = (19.95, 4.0, math.pi), (2.0, 2.0)
    t = transit(start, goal)
    check(t, start, goal)
    # straight ahead first, no turning till the body is in
    assert t.poses[0].yaw == pytest.approx(math.pi) and 0 not in t.spins


def test_start_a_little_outside():
    # the centre 0.3 m outside the lawn, facing in
    start, goal = (-0.3, 3.0, 0.0), (5.0, 1.0)
    t = transit(start, goal)
    check(t, start, goal)
    assert 0.3 <= t.start_outside <= 0.5 + 1e-9


@pytest.mark.parametrize('start, goal, why', [
    ((25.0, 4.0, 0.0), (2.0, 2.0), 'the start (25.00, 4.00) lies 5.05 m outside'),
    ((19.9, 4.0, 0.0), (2.0, 2.0), 'sticks out of the drivable areas, straight ahead too'),  # facing out of the lawn
    ((2.0, 2.0, 0.0), (5.0, 3.0), 'the goal (5.00, 3.00) lies'),  # in the tree
    ((2.0, 2.0, 0.0), (19.0, 4.0, 0.0), 'goal_straight needs a goal yaw'),
])
def test_refused(start, goal, why):
    kw = {'goal_straight': 0.5} if 'goal_straight' in why else {}
    with pytest.raises(PlanningError, match=why.replace('(', r'\(').replace(')', r'\)')):
        transit(start, goal[:2] if kw else goal, **kw)


def test_no_way():
    # the lawns without the navigation area between them
    with pytest.raises(PlanningError, match='no way from'):
        transit((2.0, 3.0, 0.0), (17.0, 5.0), areas=[LAWN, BACK])


def test_at_the_goal():
    t = transit((2.0, 3.0, 0.0), (2.05, 3.0, 0.1))
    assert t.poses == [] and t.length == 0 and t.warnings == ['at the goal already']
    # there but facing the other way: round on arcs (Dubins' way, where drive_to would turn on the spot)
    for start, goal in (((2.0, 3.0, 0.0), (2.05, 3.0, math.pi)), ((2.0, 3.0, 0.0), (2.0, 3.0, 0.5))):
        t = transit(start, goal)
        check(t, start, goal)
        assert t.spins == [] and t.length < 2.5
    # with turn_on_spot turning there is up to the driver
    t = transit((2.0, 3.0, 0.0), (2.05, 3.0, math.pi), {'turn_on_spot': True})
    assert t.poses == [] and '180 deg off its heading' in t.warnings[0]


def test_without_body():
    cfg = {'robot_width': None, 'robot_front': None, 'robot_rear': None}
    t = plan_transit((2.0, 3.0, 0.0), (8.0, 3.0, 0.0), AREAS, [TREE], PlannerConfig(**dict(TRON, **cfg)))
    assert len(t.poses) >= 3 and t.warnings and 'no body check' in t.warnings[0]
    tree = Polygon(TREE)
    assert min(tree.distance(Point(p.x, p.y)) for p in t.poses) >= 0.11 - 1e-6


def test_by_the_edge_as_recorded():
    # a pose of the outermost loop by the edge (the Tron's body): no way gets there by the rules of the lanes, one by
    # that loop's (a warning); with as_recorded False none (MowBite OS' transit_as_recorded)
    cfg = dict(perimeter_passes=3, turn_radius=0.44, robot_front=0.60, robot_rear=0.12, blade_ahead=0.23)
    start, goal = (3.0, 1.5, 0.0), (2.458, 0.11, 0.0)
    t = transit(start, goal, cfg, areas=[LAWN], max_outside=0.3)
    assert any('as the outline was recorded' in w for w in t.warnings)
    check(t, start, goal, cfg, areas=[LAWN])
    with pytest.raises(PlanningError, match='^no way arrives at the goal|^the goal |^the body sticks out'):
        transit(start, goal, cfg, areas=[LAWN], max_outside=0.3, as_recorded=False)


def test_by_the_edge_only_from_or_to_the_edge(monkeypatch):
    # that loop's rules only where the start or the goal lies by the edge, the body not keeping the usual margins there:
    # none in the middle of the lawn or by an obstacle only, also where the way sticks out on its way
    import mowbite_planner_os.transit_route as route
    real, tries = route._transit, []

    def first_fails(start, goal, drivable, obstacles, cfg, *rest):
        tries.append(cfg.edge_margin)
        if len(tries) == 1:
            raise PlanningError('the body sticks out on the way at (5.00, 3.00)')
        return real(start, goal, drivable, obstacles, cfg, *rest)

    monkeypatch.setattr(route, '_transit', first_fails)
    cfg = dict(perimeter_passes=3, turn_radius=0.44, robot_front=0.60, robot_rear=0.12, blade_ahead=0.23)
    for start, goal in (((2.0, 3.0, 0.0), (8.0, 3.0, 0.0)), ((5.0, 2.2, 0.0), (8.0, 3.0, 0.0))):
        tries.clear()
        with pytest.raises(PlanningError, match='^the body sticks out on the way'):
            transit(start, goal, cfg, areas=[LAWN])
        assert len(tries) == 1
    tries.clear()
    t = transit((3.0, 1.5, 0.0), (2.458, 0.11, 0.0), cfg, areas=[LAWN], max_outside=0.3)
    assert len(tries) == 2 and tries[1] < tries[0]
    assert any('as the outline was recorded' in w for w in t.warnings)


def test_from_and_to_the_poses_of_a_plan():
    # the Tron's sim (MowBite OS): one small lawn; every pose of the plan a start, a goal where a way gets there facing
    # its heading (the rules checked); one by the edge no arc gets to facing along it says so
    ring = [(-1.0, -0.8), (2.0, -0.8), (2.0, 0.8), (-1.0, 0.8)]
    cfg = dict(overlap=0.15, lane_order='skip', turn_on_spot=False, turn_types=['u_turn', 'bulb', 'detour'],
               perimeter_passes=1)
    plan = plan_coverage(ring, [], PlannerConfig(**dict(TRON, **cfg)))
    unreachable = []
    for i, p in enumerate(plan.poses[::4]):
        pose = (p.x, p.y, p.yaw)
        t = transit(pose, (0.5, 0.0, 0.0), cfg, areas=[ring], obstacles=())
        if t.poses:  # (else at the goal already)
            check(t, pose, (0.5, 0.0, 0.0), cfg, areas=[ring], obstacles=())
        try:
            t = transit((0.0, 0.0, 0.0), pose, cfg, areas=[ring], obstacles=(), max_outside=0.3)
        except PlanningError as e:
            assert str(e).startswith(('no way arrives at the goal', 'the body sticks out on the way')), e
            unreachable.append(4 * i)
            continue
        if t.poses:
            check(t, (0.0, 0.0, 0.0), pose, cfg, areas=[ring], obstacles=())
    print(len(plan.poses), unreachable)
    assert len(unreachable) < len(plan.poses[::4]) / 2
