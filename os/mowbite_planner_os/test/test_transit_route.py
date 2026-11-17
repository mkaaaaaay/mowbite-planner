"""Tests of plan_transit (the node's ~/plan_transit) without ROS."""

import math
import os
import sys

import pytest
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', '..', '..', 'ros2', 'mowbite_planner')]

from mowbite_planner import PlannerConfig, PlanningError, progress  # noqa: E402
from mowbite_planner.footprint import SPIN, Footprint  # noqa: E402
from mowbite_planner.mathutil import Pose2D, angle_diff  # noqa: E402
from mowbite_planner.planner import plan_coverage  # noqa: E402
from mowbite_planner.space import _body_space  # noqa: E402
from mowbite_planner_os import transit_route  # noqa: E402
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


@pytest.mark.parametrize('start, goal, why, word', [
    ((25.0, 4.0, 0.0), (2.0, 2.0), 'the start (25.00, 4.00) lies 5.05 m outside', 'START_OUTSIDE'),
    # facing out of the lawn
    ((19.9, 4.0, 0.0), (2.0, 2.0), 'the start (19.90, 4.00): the body is 0.2', 'START_OUTSIDE'),
    ((2.0, 2.0, 0.0), (5.0, 3.0), 'the goal (5.00, 3.00) lies', 'GOAL_OUTSIDE'),  # in the tree
    ((2.0, 2.0, 0.0), (19.0, 4.0, 0.0), 'goal_straight needs a goal yaw', 'BAD_REQUEST'),
    ((2.0, 2.0), (19.0, 4.0, 0.0), 'the start needs x, y and yaw', 'BAD_REQUEST'),
    ((2.0, 2.0, math.inf), (19.0, 4.0, 0.0), 'the start yaw must be a number', 'BAD_REQUEST'),
    ((2.0, 2.0, 0.0), (1e9, 4.0, 0.0), 'the start or goal has a point that is no number or more than', 'BAD_REQUEST'),
])
def test_refused(start, goal, why, word):
    # (word: what ~/plan_transit's code says of it)
    kw = {'goal_straight': 0.5} if 'goal_straight' in why else {}
    with pytest.raises(PlanningError, match=why.replace('(', r'\(').replace(')', r'\)')) as e:
        transit(start, goal[:2] if kw else goal, **kw)
    assert e.value.word == word and word in transit_route.CODES


# a bed recorded the way its points go (the bed on the left), on the first lawn
BED = [(3.0, 2.0), (7.0, 2.0), (7.0, 4.0), (3.0, 4.0)]


def by_the_bed(t, start, goal):
    """A way from or to a pose on BED's line: every body inside where it was while recording, as deep out of it as at
    the start or goal at most; from where it fits with the margins on, as any way's (the body in, keeping them)."""
    cfg = PlannerConfig(**TRON)
    room = transit_route.recording_room(AREAS, [BED], cfg)
    fp = Footprint(room, cfg.body_outline)
    deep = max(transit_route._depth(room, fp.body(*start)), transit_route._depth(room, fp.body(*goal)))
    loose = Footprint(room.buffer(deep + 1e-3), cfg.body_outline)
    assert t.poses[0].x == pytest.approx(start[0]) and (t.poses[-1].x, t.poses[-1].y) == pytest.approx(goal[:2])
    assert loose.all_fit(t.poses) and t.start_outside == t.goal_outside == 0.0
    space, _ = _body_space(AREAS, [BED], [cfg.obstacle_margin], cfg)
    margins = Footprint(space, cfg.body_outline, cfg.spin_margin)
    fits = [margins.fits_pose(p.x, p.y, p.yaw) for p in t.poses]
    first, last = fits.index(True), len(fits) - 1 - fits[::-1].index(True)
    assert 0 < first or last < len(fits) - 1
    assert margins.bad_poses(t.poses[first:last + 1]) == []
    assert [w for w in t.warnings if 'closer to an edge than its margin' in w]


@pytest.mark.parametrize('start', [(5.0, 2.0, 0.0), (5.0, 1.97, 0.0), (5.0, 2.0, math.pi), (7.0, 3.0, 0.5 * math.pi),
                                   (5.0, 4.0, math.pi)],
                         ids=['on_the_line', 'beside_it', 'the_other_way', 'round_the_corner', 'far_side'])
def test_start_on_a_recorded_line(start):
    # the mower right by an obstacle it recorded, closer than obstacle_margin (15.11.): a way out, turning or along the
    # line first where the body stays where it was while recording
    goal = (1.0, 1.0, math.pi)
    with pytest.raises(PlanningError, match=r'^the start \('):  # (with a far wider margin none)
        transit(start, goal, obstacles=[BED], max_outside=0.3, cfg={'obstacle_margin': 0.6})
    t = transit(start, goal, obstacles=[BED], max_outside=0.3)
    by_the_bed(t, start, goal)


def test_goal_on_a_recorded_line():
    # the way to such a pose the other way round: driven backwards out of it (the body turned round), the way turned
    # back; the Tron's nose reaching far ahead finds none by the bed within max_outside (it says so), one as long behind
    # as ahead does
    goal = (5.0, 2.0, 0.0)
    with pytest.raises(PlanningError, match=r'the goal \(5.00, 2.00\): the body is closer to an obstacle or an edge '
                       'than its margin and no way out fits') as e:
        transit((1.0, 1.0, 0.0), goal, obstacles=[BED], max_outside=0.3)
    assert e.value.word == 'GOAL_OUTSIDE'
    even = {'robot_front': 0.3, 'robot_rear': 0.3}
    t = transit((1.0, 1.0, 0.0), goal, even, obstacles=[BED], max_outside=0.3)
    assert (t.poses[-1].x, t.poses[-1].y) == pytest.approx(goal[:2]) and angle_diff(t.poses[-1].yaw, 0.0) < 1e-9
    assert [w for w in t.warnings if w.startswith('the goal lies closer to an edge than its margin')]
    cfg = PlannerConfig(**dict(TRON, **even))
    room = Footprint(transit_route.recording_room(AREAS, [BED], cfg).buffer(1e-3), cfg.body_outline)
    assert room.all_fit(t.poses) and t.goal_outside == 0.0


def test_leave_edge_at_a_slant():
    # standing at a slant by the line: along it as far beside it as the mower is (not as far as the corner is ahead
    # of it), where a way straight out fits only by the next stretch
    square = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    room = Polygon([(-5.0, -5.0), (7.0, -5.0), (7.0, 7.0), (-5.0, 7.0)])
    fits = Footprint(room, ((0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1), (0.1, -0.1)))
    got, why = transit_route.leave_edge(Pose2D(1.0, 0.1, math.radians(45)), room, fits, lambda p, yaw: p[1] > 1.0,
                                        [square], 0.3, 0.1)
    assert why == '' and got[-1].y > 1.0
    line = LineString(square + square[:1])
    assert max(line.distance(Point(p.x, p.y)) for p in got) == pytest.approx(0.1)


def test_backwards():
    # a way out turned round to be a way in: each pose facing the next, on the spot its own turned round, the last
    # one as given (where the way out began facing another way)
    out = [Pose2D(0.0, 0.0, 0.5 * math.pi), Pose2D(0.1, 0.0, 0.0), Pose2D(0.2, 0.0, 0.5 * math.pi),
           Pose2D(0.2, 0.1, 0.5 * math.pi), Pose2D(0.2, 0.1, math.pi)]
    back = transit_route._backwards(out, 1.0)
    assert [(p.x, p.y) for p in back] == [(p.x, p.y) for p in reversed(out)]
    assert [round(p.yaw, 9) for p in back] == [round(y, 9) for y in (0.0, -0.5 * math.pi, math.pi, math.pi, 1.0)]
    assert transit_route._backwards([], 1.0) == []


def test_start_too_deep():
    # the body further into the bed than where it was while recording (START_DEPTH): no way, the start out of the map
    with pytest.raises(PlanningError, match=r'the start \(5.00, 2.15\): the body is 0.1[45] m inside an obstacle or '
                       'past an edge of the map, no way out is planned from there') as e:
        transit((5.0, 2.15, 0.0), (1.0, 1.0), obstacles=[BED], max_outside=0.3)
    assert e.value.word == 'START_OUTSIDE'
    with pytest.raises(PlanningError, match=r'the goal \(5.00, 2.15\): the body is') as e:
        transit((1.0, 1.0, 0.0), (5.0, 2.15, 0.0), obstacles=[BED], max_outside=0.3)
    assert e.value.word == 'GOAL_OUTSIDE'


def test_each_obstacles_own_margin():
    # margins: an obstacle's own instead of obstacle_margin (None: that one), the room the centre has goes by them
    cfg = PlannerConfig(**TRON)
    plain = transit_route.centre_space(AREAS, [TREE, BED], cfg)[0]
    assert transit_route.centre_space(AREAS, [TREE, BED], cfg, margins=[None, None])[0].equals(plain)
    wide = transit_route.centre_space(AREAS, [TREE, BED], cfg, margins=[None, 0.4])[0]
    assert wide.area < plain.area and wide.covers(plain.difference(Polygon(BED).buffer(1.2)))
    with pytest.raises(PlanningError, match='a margin must be a number') as e:
        transit((1.0, 1.0, 0.0), (8.0, 5.0), obstacles=[TREE], margins=[-1.0])
    assert e.value.word == 'BAD_REQUEST'


def test_bad_requests():
    for kw, why in (({'cfg': {'waypoint_spacing': 0.0}}, 'waypoint_spacing'), ({'cfg': {'overlap': 2.0}}, 'overlap'),
                    ({'areas': []}, 'no drivable areas'), ({'max_outside': -0.1}, 'must be >= 0'),
                    ({'areas': [[(0.0, 0.0), (math.nan, 0.0), (1.0, 1.0)]]}, 'a drivable area has a point'),
                    ({'obstacles': [[(0.0, 0.0), (1e9, 0.0), (1.0, 1.0)]]}, 'an obstacle has a point')):
        with pytest.raises(PlanningError, match=why) as e:
            transit((2.0, 3.0, 0.0), (8.0, 3.0, 0.0), **kw)
        assert e.value.word == 'BAD_REQUEST', why


def test_no_way():
    # the lawns without the navigation area between them (no word of its own: ~/plan_transit's NO_WAY)
    with pytest.raises(PlanningError, match='no way from') as e:
        transit((2.0, 3.0, 0.0), (17.0, 5.0), areas=[LAWN, BACK])
    assert e.value.word == ''
    tiny = [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.0, 0.3)]
    with pytest.raises(PlanningError, match='the body fits nowhere in the drivable areas') as e:
        transit((0.1, 0.1, 0.0), (0.2, 0.2), {'edges': 'hard'}, areas=[tiny], obstacles=())
    assert e.value.word == ''


# two lawns joined by a passage (its middle line: out of the one, round, into the other)
FRONT = [(0.0, 2.0), (5.0, 2.0), (5.0, 7.0), (0.0, 7.0)]
REAR = [(0.0, -7.0), (5.0, -7.0), (5.0, -2.0), (0.0, -2.0)]
ROUND = [(4.5, 4.0), (7.0, 4.0)] + [(7.0 + 4.0 * math.sin(math.pi * i / 32), 4.0 * math.cos(math.pi * i / 32))
                                    for i in range(1, 32)] + [(7.0, -4.0), (4.5, -4.0)]
SQUARE = [(4.5, 4.0), (11.0, 4.0), (11.0, -4.0), (4.5, -4.0)]


def passage(width, line, join=1):
    return list(LineString(line).buffer(0.5 * width, cap_style=2, join_style=join).exterior.coords)[:-1]


@pytest.fixture
def ways(monkeypatch):
    """How many ways plan_transit asked for so far (Drive.way)."""
    count = [0]

    class Counted(transit_route.Drive):
        def way(self, *args):
            count[0] += 1
            return super().way(*args)

    monkeypatch.setattr(transit_route, 'Drive', Counted)
    return lambda: count[0]


def test_through_a_winding_passage(ways):
    # the shortest way through bends round every corner of the passage's inner edge, the body touching the edge
    # there: the way that keeps off it, the first one tried
    for width, line, join in ((0.45, ROUND, 1), (0.3, ROUND, 1), (0.5, SQUARE, 2)):
        areas = [FRONT, passage(width, line, join), REAR]
        for start, goal in (((2.0, 4.0, 0.0), (2.0, -4.0)), ((2.0, -4.0, 0.0), (2.0, 4.5, math.pi))):
            before = ways()
            t = transit(start, goal, areas=areas, obstacles=())
            check(t, start, goal, areas=areas, obstacles=())
            assert t.spins == [] and t.warnings == [] and ways() == before + 1
            assert t.length < LineString(line).length + 6.0


def test_lined_up_before_a_gap(ways):
    # a gap between two beds the body just fits through: come at from the side the nose swings out over the far bed
    # turning into it. Lined up with the gap before it, by the start or the goal too
    lawn = [(0.0, 0.0), (12.0, 0.0), (12.0, 6.0), (0.0, 6.0)]
    beds = [[(5.0, 3.15), (8.0, 3.15), (8.0, 6.5), (5.0, 6.5)], [(5.0, -0.5), (8.0, -0.5), (8.0, 2.85), (5.0, 2.85)]]
    for start, goal, few in (((1.0, 0.8, 0.0), (11.0, 3.0), True), ((11.0, 5.2, math.pi), (1.0, 3.0), True),
                             ((1.0, 3.0, 0.0), (11.0, 3.0), True), ((3.0, 1.9, 0.0), (11.0, 3.0), False),
                             ((1.0, 0.8, 0.0), (8.6, 3.0, 0.0), False)):
        before = ways()
        t = transit(start, goal, areas=[lawn], obstacles=beds)
        check(t, start, goal, areas=[lawn], obstacles=beds)
        assert t.spins == []
        through = [p for p in t.poses if 5.0 <= p.x <= 8.0]
        assert through and all(abs(p.y - 3.0) < 0.05 for p in through)
        # (far from both ends: lined up at once, the ends' ways not tried)
        assert not few or ways() <= before + 4
    # a gap the body doesn't fit through: no way
    beds = [[(5.0, 3.05), (8.0, 3.05), (8.0, 6.5), (5.0, 6.5)], [(5.0, -0.5), (8.0, -0.5), (8.0, 2.95), (5.0, 2.95)]]
    with pytest.raises(PlanningError, match='^no way from'):
        transit((1.0, 0.8, 0.0), (11.0, 3.0), areas=[lawn], obstacles=beds)


def test_refused_after_a_few_ways(ways):
    # a passage with square corners the body just fits along: it doesn't get round them, and can't line up before them
    # (the passage's wall). Nothing at the ends changes that: refused without their ways, after the same few ways on
    # any computer
    areas = [FRONT, passage(0.25, SQUARE, 2), REAR]
    with pytest.raises(PlanningError, match=r'^the body sticks out on the way at \d+ poses: \(10\.'):
        transit((2.0, 4.0, 0.0), (2.0, -4.0), areas=areas, obstacles=())
    assert ways() == 2
    # a goal by the edge the body doesn't get to facing along it: the ends' ways tried, a number of them too
    lawn = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
    with pytest.raises(PlanningError, match=r'^the body sticks out on the way at \d+ poses: \(9\.'):
        transit((3.0, 3.0, 0.0), (9.95, 3.0, math.pi / 2), areas=[lawn], obstacles=(), max_outside=0.0)
    assert 10 < ways() - 2 < 100


def test_the_same_answers_from_a_kept_room():
    # the space and the planners kept from one request to the next (the same areas, obstacles and settings): the same
    # answers as made anew for each
    areas = [FRONT, passage(0.45, ROUND), REAR]
    asks = [((2.0, 4.0, 0.0), (2.0, -4.0)), ((2.0, -4.0, 0.0), (2.0, 4.5, math.pi)), ((1.0, 6.0, 1.0), (4.0, 2.2, 0.0)),
            ((2.0, 4.0, 0.0), (4.9, 6.9, 0.0)), ((2.0, 4.0, 0.0), (2.0, -4.0))]

    def answer(start, goal):
        try:
            return transit(start, goal, areas=areas, obstacles=()).poses
        except PlanningError as e:
            return str(e)

    kept = [answer(*ask) for ask in asks]
    assert kept[0] == kept[-1] and transit_route._ROOM.last[1] is transit_route._room(
        areas, [], transit_route._turning(PlannerConfig(**TRON)))[0]
    anew = []
    for ask in reversed(asks):
        transit_route._ROOM.__dict__.clear()
        anew.append(answer(*ask))
    assert kept == anew[::-1]
    # other settings, another room
    transit(asks[0][0], asks[0][1], {'edge_margin': 0.03}, areas=areas, obstacles=())
    assert transit_route._ROOM.last[0][2].edge_margin == 0.03


def test_stopped_anywhere_the_room_stays_good():
    # a search stopped from outside (the node's transit_search_limit) at any of its checks: what's kept of
    # the map is as good as new, the same request and another answered as made anew
    lawn = [(0.0, 0.0), (12.0, 0.0), (12.0, 6.0), (0.0, 6.0)]
    beds = [[(5.0, 3.15), (8.0, 3.15), (8.0, 6.5), (5.0, 6.5)], [(5.0, -0.5), (8.0, -0.5), (8.0, 2.85), (5.0, 2.85)]]
    through = ((2.0, 4.0, 0.0), (2.0, -4.0))
    for areas, obstacles, asks in (
            ([lawn], beds, [((3.0, 1.9, 0.0), (11.0, 3.0)), ((1.0, 0.8, 0.0), (8.6, 3.0, 0.0))]),
            ([FRONT, passage(0.45, ROUND), REAR], [], [through, ((1.0, 6.0, 1.0), (4.0, 2.2, 0.0))]),
            ([FRONT, passage(0.25, SQUARE, 2), REAR], [], [through, ((2.0, 4.0, 0.0), (4.0, 6.0))])):

        def answer(start, goal, stop=None):
            try:
                with progress.stoppable(stop or (lambda: False)):
                    return transit(start, goal, areas=areas, obstacles=obstacles).poses
            except PlanningError as e:
                return str(e)

        want = []
        for ask in asks:
            transit_route._ROOM.__dict__.clear()
            want.append(answer(*ask))
        checks = [0]

        def counted():
            checks[0] += 1
            return False
        transit_route._ROOM.__dict__.clear()
        assert answer(*asks[0], stop=counted) == want[0] and checks[0] > 3
        n = checks[0]
        for at in sorted({1, 2, 3, n // 4, n // 2, n - 1, n}):
            seen = [0]

            def stop():
                seen[0] += 1
                return seen[0] >= at
            transit_route._ROOM.__dict__.clear()
            with pytest.raises(progress.Stopped):
                answer(*asks[0], stop=stop)
            assert [answer(*ask) for ask in asks] == want, at


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


def test_by_the_edge_no_way_past_the_margins():
    # a pose by the edge where the body doesn't keep edge_margin (the Tron's): no way, no second try with less (mkay
    # 10.11.)
    cfg = dict(perimeter_passes=3, turn_radius=0.44, robot_front=0.60, robot_rear=0.12, blade_ahead=0.23)
    with pytest.raises(PlanningError, match='^no way arrives at the goal|^the goal |^the body sticks out'):
        transit((3.0, 1.5, 0.0), (2.458, 0.11, 0.0), cfg, areas=[LAWN], max_outside=0.3)


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
