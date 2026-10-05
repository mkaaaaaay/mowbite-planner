"""Tests of the planner without ROS (python3 -m pytest test/ in the package folder, shapely is enough)."""

import dataclasses
import itertools
import math
import os
import random
import sys
import threading
import time
from types import SimpleNamespace
from typing import Dict, List

import pytest
from shapely.geometry import LineString, MultiPoint, Point, Polygon
from shapely.ops import nearest_points, unary_union
from shapely.prepared import prep

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import (PlannerConfig, PlanningError, Pose2D, footprint, normalize_angle,  # noqa: E402
                             plan_coverage, planner, quaternion_to_yaw, swept_area, transit, yaw_to_quaternion)
from mowbite_planner.maneuvers import (bulb_turn, drive_polyline, dubins, k_turn, omega, path_length,  # noqa: E402
                                       plan_turn, same_position, sample_straight, u_turn)
from mowbite_planner.mathutil import angle_diff  # noqa: E402
from mowbite_planner.planner import (FIT_PLACE, FIT_ROUND, FIT_START, FIT_TRIES, LANE, PERIMETER,  # noqa: E402
                                     TRANSIT, TURN, Lane, _lane_cost, _Route, SPIN_LIMIT, _spins, angle_in_range,
                                     best_order, longest_edge_angle, plan_footprint, scan_lanes, split_cells)
from mowbite_planner.footprint import Footprint  # noqa: E402
from mowbite_planner.report import check_plan  # noqa: E402
from mowbite_planner.transit import TransitPlanner  # noqa: E402

DEG = math.pi / 180.0


def circle(cx, cy, r, n=24):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


GARDEN = [(0, 0), (22, 0), (24, 9), (13, 9), (13, 16), (0, 16)]
TREE = circle(5, 6, 0.9, 16)
BED = rect(15, 2.5, 19, 5)


def check_invariants(plan, cfg, obstacles=(), arc_tol=6 * DEG):
    """What every plan has to keep to, whatever the shape."""
    poses = plan.poses
    assert len(poses) >= 2
    segs = plan.segments
    # the mower's centre stays in robot_space, only transits may use the drivable areas too
    inside = prep(plan.robot_space.buffer(2e-3))
    drivable = inside if plan.transit_space is plan.robot_space else prep(plan.transit_space.buffer(2e-3))
    in_transit = {i for s in segs if s.kind == TRANSIT for i in range(s.start, s.end + 1)}
    step = cfg.waypoint_spacing
    for i, p in enumerate(poses):
        assert math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.yaw)
        assert -math.pi < p.yaw <= math.pi
        at = Point(p.x, p.y)
        assert inside.covers(at) or i in in_transit and drivable.covers(at), f'pose {p} outside the free space'
    # no obstacle closer than the first loop's offset (minus what simplifying the input may move)
    cloud = MultiPoint([(p.x, p.y) for p in poses])
    for o in obstacles:
        poly = Polygon(o)
        if poly.is_valid:
            assert poly.distance(cloud) >= cfg.first_loop_offset - cfg.simplify_tolerance - 2e-3
    # the route is one piece per section, poses at most waypoint_spacing apart, never two on one position
    assert plan.section_starts[0] == 0 and plan.section_starts == sorted(set(plan.section_starts))
    jumps = {s - 1 for s in plan.section_starts[1:]}
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        d = math.hypot(b.x - a.x, b.y - a.y)
        assert d > 1e-7, f'two poses on one position at {a}'
        if step > 0 and i not in jumps:
            assert d <= step + 1e-6, f'gap of {d} m after {a}'
    # segments are in order and cover every pose
    assert segs[0].start == 0 and segs[-1].end == len(poses) - 1
    for s, t in zip(segs, segs[1:]):
        assert s.start < s.end
        assert t.start in (s.end, s.end + 1)
    # every pose faces the way it's driven to the next one, backwards only on the reverse legs of K-turns (between
    # lanes, or in place of a turn on the spot)
    backwards = [False] * (len(poses) - 1)
    for s in segs:
        assert not s.reverse or (s.kind == TURN and s.maneuver == 'k_turn') or s.maneuver == 'k_point'
        for i in range(s.start, s.end):
            backwards[i] = s.reverse
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        if i in jumps:
            continue
        travel = math.atan2(b.y - a.y, b.x - a.x)
        facing = a.yaw + math.pi if backwards[i] else a.yaw
        assert angle_diff(travel, facing) <= arc_tol, f'pose {i} {a} faces {a.yaw}, drives {travel}'
    # straight lanes keep exactly one heading, along one of the sets of lanes (concentric rings bend)
    for s in segs:
        if s.kind == LANE and s.maneuver not in ('ring', 'bend', 'k_point'):
            yaws = {poses[i].yaw for i in range(s.start, s.end)}
            assert len(yaws) == 1
            yaw = yaws.pop()
            # doubled, a and a + pi are the same lanes
            assert min(angle_diff(2 * yaw, 2 * a) for a in plan.angles) < 1e-9


def coverage(plan, cfg):
    return swept_area(plan, cfg.mower_width).intersection(plan.free_space).area / plan.free_space.area


# quaternion ------------------------------------------------------------------------------------------------


@pytest.mark.parametrize('yaw', [0.0, 0.3, math.pi / 2, math.pi, -math.pi, -2.5, 3 * math.pi, -7.0, 1e-12])
def test_quaternion_round_trip(yaw):
    x, y, z, w = yaw_to_quaternion(yaw)
    assert x == 0.0 and y == 0.0
    assert math.isclose(x * x + y * y + z * z + w * w, 1.0, abs_tol=1e-12)
    assert w >= 0.0
    assert angle_diff(quaternion_to_yaw(x, y, z, w), yaw) < 1e-9


def test_quaternion_values():
    assert yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0)
    _, _, z, w = yaw_to_quaternion(math.pi / 2)
    assert math.isclose(z, math.sqrt(0.5)) and math.isclose(w, math.sqrt(0.5))
    # pi and -pi are the same heading and give the same quaternion
    assert yaw_to_quaternion(math.pi) == yaw_to_quaternion(-math.pi)


def test_normalize_angle():
    assert normalize_angle(math.pi) == math.pi
    assert normalize_angle(-math.pi) == math.pi
    assert math.isclose(normalize_angle(3 * math.pi / 2), -math.pi / 2)
    assert math.isclose(normalize_angle(-3 * math.pi / 2), math.pi / 2)
    assert normalize_angle(0.0) == 0.0


# turns -----------------------------------------------------------------------------------------------------


def radii_of(poses):
    """Radius of the circle through every 3 neighbouring points (inf on straight bits)."""
    out = []
    for a, b, c in zip(poses, poses[1:], poses[2:]):
        ab, bc, ca = (math.hypot(b.x - a.x, b.y - a.y), math.hypot(c.x - b.x, c.y - b.y),
                      math.hypot(a.x - c.x, a.y - c.y))
        area2 = abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
        out.append(math.inf if area2 < 1e-12 else ab * bc * ca / (2 * area2))
    return out


def ends(m):
    first, last = m.pieces[0].poses[0], m.pieces[-1].poses[-1]
    return first, last


@pytest.mark.parametrize('dx', [0.0, 0.4, -0.4])
def test_u_turn_geometry(dx):
    m = u_turn(0.6, dx, 0.25, 0.05)
    first, last = ends(m)
    assert (first.x, first.y, first.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert (last.x, last.y) == pytest.approx((dx, 0.6), abs=1e-9)
    assert angle_diff(last.yaw, math.pi) < 1e-9
    assert all(r >= 0.25 - 1e-6 for r in radii_of(m.pieces[0].poses))
    assert u_turn(0.4, 0.0, 0.25, 0.05) is None  # lanes closer than two radii


@pytest.mark.parametrize('d', [0.05, 0.187, 0.3, 0.49])
def test_bulb_turn_geometry(d):
    m = bulb_turn(d, 0.0, 0.25, 0.05)
    first, last = ends(m)
    assert (last.x, last.y) == pytest.approx((0.0, d), abs=1e-9)
    assert angle_diff(last.yaw, math.pi) < 1e-9
    assert len(m.pieces) == 1 and not m.pieces[0].reverse
    poses = m.pieces[0].poses
    assert all(r >= 0.25 - 1e-6 for r in radii_of(poses))
    for a, b in zip(poses, poses[1:]):  # tangent continuous: never more than one arc step at once
        assert angle_diff(a.yaw, b.yaw) <= 10 * DEG + 1e-9


def test_k_turn_geometry():
    m = k_turn(0.187, 0.0, 0.25, 0.05)
    assert [p.reverse for p in m.pieces] == [False, True, False]
    fwd, back, fwd2 = (p.poses for p in m.pieces)
    assert (back[0].x, back[0].y) == pytest.approx((fwd[-1].x, fwd[-1].y))
    assert (fwd2[0].x, fwd2[0].y) == pytest.approx((back[-1].x, back[-1].y))
    assert back[0].y > back[-1].y  # backs up
    assert all(p.yaw == pytest.approx(math.pi / 2) for p in back)  # still facing ahead
    assert fwd2[-1].x == pytest.approx(0.0) and fwd2[-1].y == pytest.approx(0.187)
    assert all(r >= 0.25 - 1e-6 for r in radii_of(fwd) + radii_of(fwd2))
    # needs only one radius of headland
    assert max(p.x for piece in m.pieces for p in piece.poses) == pytest.approx(0.25)


def headland(depth):
    """fits() of a lane end at x = 0 with `depth` of room behind it."""
    space = prep(Polygon(rect(-10, -10, depth, 10)).buffer(1e-6))
    return lambda pts: space.covers(LineString(pts) if len(pts) > 1 else Point(pts[0]))


def test_turn_choice_by_room():
    exit_pose = Pose2D(0.0, 0.0, 0.0)
    entry = (0.0, 0.187)
    # plenty of room: forwards only, full radius
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(5.0)).name == 'bulb'
    # one radius of room: the K-turn keeps the radius
    m = plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(0.26))
    assert m.name == 'k_turn' and m.radius == 0.25
    # no backing up allowed: a smaller bulb
    m = plan_turn(exit_pose, entry, 0.25, False, 0.05, headland(0.26))
    assert m.name == 'bulb' and m.radius < 0.25
    # hardly any room: half circle, then turning on the spot
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(0.1)).name == 'u_turn'
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(0.01)).name == 'pivot'
    # lanes far apart: a plain U-turn
    assert plan_turn(exit_pose, (0.0, 0.6), 0.25, True, 0.05, headland(0.3)).name == 'u_turn'


def test_turn_to_the_right_and_rotated():
    # lane heading 120 deg, next lane to its right
    h = 120 * DEG
    exit_pose = Pose2D(3.0, 1.0, h)
    entry = (3.0 + 0.187 * math.sin(h), 1.0 - 0.187 * math.cos(h))
    m = plan_turn(exit_pose, entry, 0.25, True, 0.05, lambda pts: True)
    last = m.pieces[-1].poses[-1]
    assert (last.x, last.y) == pytest.approx(entry)
    assert angle_diff(last.yaw, h + math.pi) < 1e-9


def test_drive_polyline_rounds_corners_that_fit():
    pts = [(0, 0), (2, 0), (2, 2)]
    sharp = drive_polyline(pts, 0.1)
    assert any(abs(p.x - 2) < 1e-9 and abs(p.y) < 1e-9 for p in sharp)
    # the corner pose already faces the next segment
    corner = next(p for p in sharp if abs(p.x - 2) < 1e-9 and abs(p.y) < 1e-9)
    assert angle_diff(corner.yaw, math.pi / 2) < 1e-9
    rounded = drive_polyline(pts, 0.1, 0.5, lambda p: True)
    assert min(math.hypot(p.x - 2, p.y) for p in rounded) > 0.1
    assert all(math.hypot(b.x - a.x, b.y - a.y) > 1e-7 for a, b in zip(rounded, rounded[1:]))


# lanes -----------------------------------------------------------------------------------------------------


def test_scan_lanes_spacing_never_exceeds_the_lane_spacing():
    rows = scan_lanes(Polygon(rect(0, 0, 10, 3)), 0.0, 0.187, 0.1)
    ys = [r[0].y for r in rows]
    assert ys[0] == pytest.approx(0.0, abs=1e-3) and ys[-1] == pytest.approx(3.0, abs=1e-3)
    assert max(b - a for a, b in zip(ys, ys[1:])) <= 0.187 + 1e-9
    assert all(len(r) == 1 and r[0].x0 == pytest.approx(0) and r[0].x1 == pytest.approx(10) for r in rows)


def test_cells_split_around_an_obstacle():
    region = Polygon(rect(0, 0, 10, 10), [rect(4, 4, 6, 6)])
    cells = split_cells(scan_lanes(region, 0.0, 0.5, 0.1))
    assert len(cells) == 4  # below, left, right, above
    for cell in cells:
        assert [lane.row for lane in cell] == list(range(cell[0].row, cell[0].row + len(cell)))


# whole plans -----------------------------------------------------------------------------------------------


def test_rectangle():
    cfg = PlannerConfig()
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg)
    check_invariants(plan, cfg)
    assert plan.angle == pytest.approx(0.0)  # along the long edge
    assert coverage(plan, cfg) > 0.997
    assert plan.stats['perimeter_loops'] == 2
    assert plan.stats['turns_pivot'] == 0 and plan.stats['turns_fallback'] == 0
    assert not plan.warnings
    # the first loop runs half a mower width inside the outline
    first = plan.perimeter_loops[0]
    assert min(min(p[0], p[1], 10 - p[0], 6 - p[1]) for p in first) == pytest.approx(0.11, abs=1e-6)


def test_lane_angle_follows_the_longest_edge():
    a = 30 * DEG
    c, s = math.cos(a), math.sin(a)
    boundary = [(x * c - y * s + 5, x * s + y * c - 3) for x, y in rect(0, 0, 12, 5)]
    cfg = PlannerConfig()
    plan = plan_coverage(boundary, [], cfg)
    assert plan.angle == pytest.approx(a)
    check_invariants(plan, cfg)
    assert coverage(plan, cfg) > 0.995
    # a fixed angle and an offset win
    assert plan_coverage(boundary, [], PlannerConfig(angle=0.2)).angle == pytest.approx(0.2)
    assert plan_coverage(boundary, [], PlannerConfig(angle_offset=-0.1)).angle == pytest.approx(a - 0.1)
    assert plan_coverage(boundary, [], PlannerConfig(angle=-0.5)).angle == pytest.approx(math.pi - 0.5)


def test_garden_with_obstacles():
    cfg = PlannerConfig()
    plan = plan_coverage(GARDEN, [TREE, BED], cfg)
    check_invariants(plan, cfg, [TREE, BED])
    assert coverage(plan, cfg) > 0.995
    assert plan.stats['cells'] > 3
    assert plan.stats['transit_length'] > 0
    assert plan.stats['turns_fallback'] == 0
    assert not plan.warnings
    # 2 loops along the outline, 2 around each obstacle
    assert plan.stats['perimeter_loops'] == 6
    # the lanes come after all loops
    kinds = [s.kind for s in plan.segments]
    assert max(i for i, k in enumerate(kinds) if k == 'perimeter') < kinds.index('lane')


@pytest.mark.parametrize('kw', [
    dict(allow_reverse=False),
    dict(lane_order='skip'),
    dict(lane_order='skip', allow_reverse=False),
    dict(perimeter_passes=0),
    dict(perimeter_passes=1, turn_radius=0.4),
    dict(perimeter_passes=4, perimeter_offset=0.0, mower_width=0.18, overlap=0.28),
    dict(waypoint_spacing=0.0),
    dict(perimeter_corner_radius=0.0, transit_clearance=0.0),
    dict(angle_strategy='min_width'),
    dict(overlap=0.0, mower_width=0.5),
])
def test_garden_variants(kw):
    cfg = PlannerConfig(**kw)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg)
    check_invariants(plan, cfg, [TREE, BED])
    assert coverage(plan, cfg) > 0.99
    if not cfg.allow_reverse:
        assert plan.stats['turns_k_turn'] == 0


def test_skip_order_turns_forwards_with_the_full_radius():
    cfg = PlannerConfig(lane_order='skip', allow_reverse=False, perimeter_passes=3)
    plan = plan_coverage(rect(0, 0, 12, 8), [], cfg)
    check_invariants(plan, cfg)
    assert coverage(plan, cfg) > 0.997
    assert plan.stats['turns_u_turn'] == plan.stats['lanes'] - 1
    for s in plan.segments:
        if s.kind == TURN:
            assert all(r >= cfg.turn_radius - 1e-6 for r in radii_of(plan.poses[s.start:s.end + 1]))
    # every lane exactly once
    ys = sorted(round(a[1], 6) for a, _ in plan.lanes)
    assert len(ys) == len(set(ys))


def test_k_turns_keep_the_radius():
    cfg = PlannerConfig()
    plan = plan_coverage(rect(0, 0, 12, 8), [], cfg)
    assert plan.stats['turns_k_turn'] == plan.stats['lanes'] - 1
    for s in plan.segments:
        if s.kind == TURN and not s.reverse:
            assert all(r >= cfg.turn_radius - 1e-6 for r in radii_of(plan.poses[s.start:s.end + 1]))


def test_deterministic():
    cfg = PlannerConfig()
    a = plan_coverage(GARDEN, [TREE, BED], cfg)
    b = plan_coverage(list(GARDEN), [list(TREE), list(BED)], cfg)
    assert a.poses == b.poses


def test_start_pose_picks_the_closest_loop_entry():
    cfg = PlannerConfig()
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg, start=(10.5, 3.0, math.pi))
    first = plan.poses[0]
    assert first.x == pytest.approx(10 - 0.11) and first.y == pytest.approx(3.0, abs=0.35)


def test_obstacles_over_the_edge_or_outside():
    cfg = PlannerConfig()
    over = rect(8, 2, 12, 4)
    outside = rect(20, 20, 22, 22)
    plan = plan_coverage(rect(0, 0, 10, 6), [over, outside], cfg)
    check_invariants(plan, cfg, [over, outside])
    assert coverage(plan, cfg) > 0.99


def test_no_way_over_keep_out_areas():
    """The way around the wall through the other lawn, but not over an inactive area or one not to mow in it: the
    centre keeps off it like off an obstacle, nor does the body reach in further than when it was recorded (half its
    width or its reach turning at a corner, obstacle_margin less, away from the wall's own edge). (The places the path
    leaves out at the outline's corners don't make parts of the area.)"""
    wall = rect(4.9, -1, 5.1, 7)
    keep = rect(3, 6.5, 7, 8)  # across the lawn's near half
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    free = plan_coverage(rect(0, 0, 10, 6), [wall], cfg, drivable=[rect(1, 4, 9, 11)])
    assert not any('parts' in w for w in free.warnings) and any(6.5 < p.y < 8 and 3 < p.x < 7 for p in free.poses)
    plan = plan_coverage(rect(0, 0, 10, 6), [wall], cfg, drivable=[rect(1, 4, 9, 11)], keep_out=[keep])
    check_invariants(plan, cfg, [wall])
    assert not any('parts' in w for w in plan.warnings) and max(p.y for p in plan.poses) > 8.0  # round it, further out
    away = Polygon(keep).buffer(cfg.centre_offset - 1e-6)
    assert not any(away.contains(Point(p.x, p.y)) for p in plan.poses)
    reach = math.hypot(0.5 * cfg.robot_width, cfg.robot_front)  # (turning at its corners)
    deep = Polygon(keep).buffer(cfg.obstacle_margin - reach, join_style=2).difference(Polygon(wall).buffer(0.5))
    assert plan.body_space.intersection(deep).area < 1e-6


def test_keep_out_areas_like_obstacles():
    """An area to keep out of reaching into the lawn is recorded like an obstacle: the loops go round it obstacle_margin
    off its line (and simplify_tolerance), the body keeps that far off its real edge (half its width inside the line),
    hard edges or not: with the body checked the lines are where the centre drove."""
    keep = rect(4, 4, 6, 9)  # the next lawn, not to mow, 2 m into this one
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg, start=(1, 1, 0.0), keep_out=[keep])
    check_invariants(plan, cfg, [keep])
    assert check_plan(plan)['body_out'] == []
    line = Polygon(keep).exterior
    loops = [p for s in plan.segments if s.kind == PERIMETER for p in plan.poses[s.start:s.end + 1]]
    assert min(line.distance(Point(p.x, p.y)) for p in loops) == pytest.approx(
        cfg.obstacle_margin + cfg.simplify_tolerance, abs=0.01)
    assert outside_room(plan, recorded_room(cfg, [rect(0, 0, 10, 6)], [keep], [cfg.obstacle_margin])) == []
    hard = plan_coverage(rect(0, 0, 10, 6), [], dataclasses.replace(cfg, edges='hard'), start=(1, 1, 0.0),
                         keep_out=[keep])
    assert hard.poses == plan.poses


@pytest.mark.parametrize('corridor', [False, True])
def test_obstacle_wall_splits_the_area(corridor):
    cfg = PlannerConfig()
    wall = rect(4.9, -1, 5.1, 7)
    # a drivable area (a path, another lawn) around the end of the wall joins the two halves
    drivable = [rect(1, 4, 9, 9)] if corridor else []
    plan = plan_coverage(rect(0, 0, 10, 6), [wall], cfg, drivable=drivable)
    check_invariants(plan, cfg, [wall])
    # both halves are mowed
    assert coverage(plan, cfg) > 0.99
    if corridor:
        assert not plan.warnings
        assert plan.section_starts == [0]
        assert max(p.y for p in plan.poses) > 7.0  # around the wall
    else:
        # the other half can't be reached without crossing the wall: a section of its own
        assert len(plan.section_starts) == 2 and plan.warnings
        bounds = zip(plan.section_starts, plan.section_starts[1:] + [len(plan.poses)])
        sides = [{p.x < 5 for p in plan.poses[a:b]} for a, b in bounds]
        assert sides in ([{True}, {False}], [{False}, {True}])
    # never across the wall
    jumps = {s - 1 for s in plan.section_starts[1:]}
    for i, (a, b) in enumerate(zip(plan.poses, plan.poses[1:])):
        assert i in jumps or not LineString([(a.x, a.y), (b.x, b.y)]).intersects(Polygon(wall))


def test_closed_ring_and_clockwise_input():
    cfg = PlannerConfig()
    # without a start pose the outline's first point is where it starts looking, give both the same
    ccw = plan_coverage(rect(0, 0, 8, 5), [], cfg, start=(0, 0))
    cw = plan_coverage(rect(0, 0, 8, 5)[::-1] + [(0, 5)], [], cfg, start=(0, 0))
    assert cw.poses == ccw.poses


def test_figure_eight_boundary_is_planned_in_both_loops():
    cfg = PlannerConfig()
    plan = plan_coverage([(0, 0), (6, 4), (6, 0), (0, 4)], [], cfg)
    check_invariants(plan, cfg)
    assert plan.free_space.area == pytest.approx(12.0)
    assert len(plan.section_starts) == 2  # the loops touch in a point only
    assert coverage(plan, cfg) > 0.97  # both, the crossing's tips are too pointed for the blade


@pytest.mark.parametrize('boundary, obstacles, kw', [
    ([(0, 0), (1, 1)], [], {}),
    (rect(0, 0, 5, 5), [rect(-1, -1, 6, 6)], {}),
    (rect(0, 0, 0.2, 5), [], {}),
    (rect(0, 0, 5, 5), [], dict(overlap=1.0)),
    (rect(0, 0, 5, 5), [], dict(mower_width=0.0)),
    (rect(0, 0, 5, 5), [], dict(mower_width=float('nan'))),
    (rect(0, 0, 5, 5), [], dict(perimeter_passes=-2)),
    (rect(0, 0, 5, 5), [], dict(lane_order='spiral')),
    (rect(0, 0, 5, 5), [], dict(angle_strategy='random')),
    ([(0, 0), (1, 0), (float('inf'), 1)], [], {}),
    (rect(0, 0, 5, 5), [], dict(fill_pattern='spiral')),
    (rect(0, 0, 5, 5), [], dict(perimeter_order='middle')),
    (rect(0, 0, 5, 5), [], dict(perimeter_direction='left')),
    (rect(0, 0, 5, 5), [], dict(turn_types=('bulb', 'loop'))),
    (rect(0, 0, 5, 5), [], dict(angle_min=0.1)),
    (rect(0, 0, 5, 5), [], dict(angle_step=0.0)),
    (rect(0, 0, 5, 5), [], dict(angle_step=1e-4)),
])
def test_bad_input(boundary, obstacles, kw):
    with pytest.raises(PlanningError):
        plan_coverage(boundary, obstacles, PlannerConfig(**kw))


def test_transit_goes_around_an_obstacle():
    space = Polygon(rect(0, 0, 10, 10), [rect(3, 3, 7, 7)])
    tp = TransitPlanner(space, clearance=0.05)
    path = tp.path((1, 5), (9, 5))
    assert path is not None and len(path) == 4
    line = LineString(path)
    assert not line.intersects(Polygon(rect(3.01, 3.01, 6.99, 6.99)))
    assert line.length == pytest.approx(2 * math.hypot(2, 2) + 4, abs=0.2)
    assert tp.path((1, 5), (5, 5)) is None  # inside the obstacle


def test_big_garden_is_quick():
    boundary = [(0, 0), (40, 0), (42, 12), (35, 25), (20, 22), (5, 26), (-2, 12)]
    obstacles = [circle(10, 8, 1.2), circle(25, 15, 0.6), rect(30, 4, 34, 7), circle(12, 18, 2.0, 30)]
    cfg = PlannerConfig()
    plan = plan_coverage(boundary, obstacles, cfg)
    check_invariants(plan, cfg, obstacles)
    assert plan.stats['planning_time'] < 5.0
    assert coverage(plan, cfg) > 0.99


def random_polygon(rnd, cx, cy, r_min, r_max, n):
    angles = sorted(rnd.uniform(0, 2 * math.pi) for _ in range(n))
    return [(cx + rnd.uniform(r_min, r_max) * math.cos(a), cy + rnd.uniform(r_min, r_max) * math.sin(a))
            for a in angles]


@pytest.mark.parametrize('seed', range(30))
def test_random_gardens(seed):
    rnd = random.Random(seed)
    boundary = random_polygon(rnd, 0, 0, 3, 9, rnd.randint(4, 14))
    obstacles = [random_polygon(rnd, rnd.uniform(-4, 4), rnd.uniform(-4, 4), 0.2, 1.2, rnd.randint(3, 8))
                 for _ in range(rnd.randint(0, 4))]
    cfg = PlannerConfig(
        mower_width=rnd.choice([0.18, 0.22, 0.3]),
        overlap=rnd.choice([0.0, 0.15, 0.3]),
        perimeter_passes=rnd.choice([0, 1, 2, 3]),
        turn_radius=rnd.choice([0.0, 0.2, 0.3]),
        allow_reverse=rnd.random() < 0.5,
        lane_order=rnd.choice(['snake', 'skip']),
        angle=rnd.choice([None, rnd.uniform(-3, 3)]),
    )
    try:
        plan = plan_coverage(boundary, obstacles, cfg)
    except PlanningError:
        return
    check_invariants(plan, cfg, obstacles)


# fill patterns ---------------------------------------------------------------------------------------------


def straight_lanes(plan):
    return [s for s in plan.segments if s.kind == LANE and s.maneuver != 'ring']


@pytest.mark.parametrize('boundary, obstacles', [(rect(0, 0, 10, 6), []), (GARDEN, [TREE, BED])])
def test_crosshatch(boundary, obstacles):
    cfg = PlannerConfig(fill_pattern='crosshatch')
    plan = plan_coverage(boundary, obstacles, cfg)
    check_invariants(plan, cfg, obstacles)
    assert len(plan.angles) == 2 and plan.angles[0] == plan.angle
    assert abs(plan.angles[1] - plan.angles[0]) == pytest.approx(math.pi / 2)
    assert coverage(plan, cfg) > 0.99
    assert not plan.warnings
    # lanes both ways, the second set after the first
    sets = [min(range(2), key=lambda k: angle_diff(2 * plan.poses[s.start].yaw, 2 * plan.angles[k]))
            for s in straight_lanes(plan)]
    assert sets == sorted(sets) and set(sets) == {0, 1}


@pytest.mark.parametrize('boundary, obstacles', [(rect(0, 0, 10, 6), []), (GARDEN, []), (GARDEN, [TREE, BED])])
def test_concentric(boundary, obstacles):
    cfg = PlannerConfig(fill_pattern='concentric')
    plan = plan_coverage(boundary, obstacles, cfg)
    # every pose inside robot_space (there are no drivable areas), lanes don't have to be straight
    check_invariants(plan, cfg, obstacles)
    assert not straight_lanes(plan) and not plan.lanes and plan.stats['lanes'] == 0
    assert any(s.maneuver == 'ring' for s in plan.segments)
    assert coverage(plan, cfg) > 0.99
    assert not plan.warnings


@pytest.mark.parametrize('boundary, obstacles', [(rect(0, 0, 10, 6), []), (GARDEN, [TREE, BED])])
def test_concentric_with_narrow_loops_in_one_go(boundary, obstacles):
    """concentric with narrow_parts loops: every loop is driven on the way round of the one next to it further out,
    a part in one go, no drives from loop to loop (but into that from the outermost), mowed as much."""
    cfg = PlannerConfig(fill_pattern='concentric', narrow_parts='loops')
    plan = plan_coverage(boundary, obstacles, cfg)
    check_invariants(plan, cfg, obstacles)
    apart = plan_coverage(boundary, obstacles, PlannerConfig(fill_pattern='concentric'))
    drives = [sum(1 for s in p.segments if s.kind == TRANSIT) for p in (plan, apart)]
    assert drives[0] <= 1 + 2 * len(obstacles) and drives[0] < drives[1]
    assert plan.stats['path_length'] < apart.stats['path_length']
    assert coverage(plan, cfg) > 0.99 and not plan.warnings


# perimeter order and direction -----------------------------------------------------------------------------


@pytest.mark.parametrize('pattern', ['lanes', 'crosshatch', 'concentric'])
def test_perimeter_last_ends_at_the_dock(pattern):
    cfg = PlannerConfig(perimeter_order='last', fill_pattern=pattern)
    dock = (-0.3, 3.0)  # just outside the left edge
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=dock, end=dock)
    check_invariants(plan, cfg, [TREE, BED])
    kinds = [s.kind for s in plan.segments if s.kind != TRANSIT]
    if pattern == 'concentric':
        # each part is a spiral from the inside out, a ring is mowed before the loops further out next to it
        assert kinds[-2:] == [PERIMETER, PERIMETER]
    else:
        assert max(i for i, k in enumerate(kinds) if k == LANE) < kinds.index(PERIMETER)
    assert plan.stats['perimeter_loops'] == 6
    # the last loop is the outermost, it ends where it's closest to the dock
    last = plan.poses[-1]
    assert math.hypot(last.x - dock[0], last.y - dock[1]) == pytest.approx(0.3 + cfg.first_loop_offset, abs=0.02)


def signed_area(poses):
    return 0.5 * sum(a.x * b.y - b.x * a.y for a, b in zip(poses, poses[1:] + poses[:1]))


@pytest.mark.parametrize('pattern', ['lanes', 'concentric'])
@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_perimeter_direction(direction, pattern):
    cfg = PlannerConfig(perimeter_direction=direction, fill_pattern=pattern)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg)
    check_invariants(plan, cfg, [TREE, BED])
    # every loop the same way round, around the obstacles too (and concentric's rings), a K-turn in it a part of it
    loops: Dict[str, List[int]] = {}
    for s in plan.segments:
        if s.kind == PERIMETER or s.kind == LANE and s.maneuver in ('ring', 'k_point'):
            span = loops.setdefault(s.source, [s.start, s.end])
            span[0], span[1] = min(span[0], s.start), max(span[1], s.end)
    assert len(loops) >= 6
    sign = 1 if direction == 'ccw' else -1
    for start, end in loops.values():
        assert sign * signed_area(plan.poses[start:end + 1]) > 0


# body and blade --------------------------------------------------------------------------------------------

FORWARD = dict(allow_reverse=False, lane_order='skip', mower_width=0.18, overlap=1 - 0.13 / 0.18, perimeter_offset=0.0)


def blade_line(poses, off):
    return [(p.x - off * math.sin(p.yaw), p.y + off * math.cos(p.yaw)) for p in poses]


@pytest.mark.parametrize('off', [0.05, -0.05, 0.08])
@pytest.mark.parametrize('pattern', ['lanes', 'crosshatch'])
def test_blade_offset_covers_everything(off, pattern):
    """The centre runs blade_offset to the side of what the blade mows: all is still mowed, the loops go round
    with the blade on the outer side (the blade up to perimeter_offset, the centre that much further in)."""
    cfg = PlannerConfig(**dict(FORWARD, blade_offset=off, fill_pattern=pattern))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED])
    assert coverage(plan, cfg) > 0.9999
    # the centre keeps |off| from the outline and the obstacles, the blade's line of the outer loop runs on it
    outline = Polygon(GARDEN).exterior
    cloud = MultiPoint([(p.x, p.y) for p in plan.poses])
    assert outline.distance(cloud) == pytest.approx(abs(off), abs=2e-3)
    for o in (TREE, BED):
        assert Polygon(o).distance(cloud) >= abs(off) - cfg.simplify_tolerance - 2e-3
    loops = [s for s in plan.segments if s.kind == PERIMETER]
    outer = max(loops, key=lambda s: abs(signed_area(plan.poses[s.start:s.end + 1])))
    blade = blade_line(plan.poses[outer.start:outer.end + 1], off)
    near = sorted(outline.distance(Point(p)) for p in blade)
    assert near[int(0.9 * len(near))] < 0.005  # but where the corners are rounded
    # with the mowable side on the left the loops along the outline go ccw and the ones around an obstacle cw:
    # a blade on the left goes the other way round
    for s in loops:
        ring = Polygon([(p.x, p.y) for p in plan.poses[s.start:s.end + 1]]).buffer(0)
        around = any(ring.contains(Polygon(o).centroid) for o in (TREE, BED)) and ring.area < 20
        assert (signed_area(plan.poses[s.start:s.end + 1]) > 0) == ((off < 0) != around)


def test_blade_offset_keeps_lanes_a_spacing_apart():
    off = 0.06
    cfg = PlannerConfig(**dict(FORWARD, blade_offset=off, angle=0.0))
    plan = plan_coverage(rect(0, 0, 12, 8), [], cfg)
    check_invariants(plan, cfg)
    ys = sorted({round(y, 6) for s in plan.segments if s.kind == LANE
                 for y in [blade_line(plan.poses[s.start:s.start + 1], off)[0][1]]})
    gaps = [b - a for a, b in zip(ys, ys[1:])]
    assert max(gaps) <= cfg.lane_spacing + 1e-6 and min(gaps) > 0.5 * cfg.lane_spacing
    # every lane's centre off its blade line by the offset, to the right of the way it goes
    for s in plan.segments:
        if s.kind == LANE:
            p = plan.poses[s.start]
            assert any(abs(blade_line([p], off)[0][1] - y) < 1e-6 for y in ys)


def test_blade_offset_with_few_loops_warns():
    cfg = PlannerConfig(**dict(FORWARD, blade_offset=0.08, perimeter_passes=0, angle=0.0))
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg)
    check_invariants(plan, cfg)
    assert any('shortened' in w for w in plan.warnings)


@pytest.mark.parametrize('off', [0.0, 0.1])
def test_robot_width_keeps_the_body_inside(off):
    cfg = PlannerConfig(**dict(FORWARD, robot_width=0.4, blade_offset=off, edges='hard'))
    assert cfg.blade_loop_offset == pytest.approx(0.2 - off) and cfg.centre_offset == pytest.approx(0.2)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED])
    cloud = MultiPoint([(p.x, p.y) for p in plan.poses])
    assert Polygon(GARDEN).exterior.distance(cloud) == pytest.approx(0.2, abs=2e-3)
    for o in (TREE, BED):
        assert Polygon(o).distance(cloud) >= 0.2 - cfg.simplify_tolerance - 2e-3
    # what the body keeps it from is left: a strip along the edges, less with the blade further out
    left = plan.free_space.difference(swept_area(plan, cfg.mower_width)).area
    edges = sum(Polygon(o).length for o in (GARDEN, TREE, BED))
    assert left == pytest.approx(edges * (0.2 - off - 0.09), rel=0.35)


@pytest.mark.parametrize('ahead', [0.17, -0.1])
def test_blade_ahead_mows_the_lanes_where_they_are(ahead):
    """The blade ahead of the centre (OpenMower's base_link): the lanes start and end that much earlier, the
    blade covers them where they are."""
    trees = [circle(4, 4, 0.4), circle(9, 10, 0.5)]
    obstacles = [TREE, BED] + trees
    kw = dict(FORWARD, overlap=1 - 0.14 / 0.18, perimeter_passes=4)
    plan = plan_coverage(GARDEN, obstacles, PlannerConfig(**dict(kw, blade_ahead=ahead, bend_max_gap=3.0)))
    cfg = plan.config
    check_invariants(plan, cfg, obstacles)
    assert coverage(plan, cfg) > 0.9999
    # every straight lane's blade runs between the same ends as the centre of a plan without it, shifted
    plain = plan_coverage(GARDEN, obstacles, PlannerConfig(**dict(kw, bend_max_gap=3.0)))
    for s in plan.segments:
        if s.kind == LANE and not s.maneuver:
            p = plan.poses[s.start]
            starts = [plain.poses[t.start] for t in plain.segments if t.kind == LANE and not t.maneuver]
            assert any(math.hypot(p.x + ahead * math.cos(p.yaw) - q.x, p.y + ahead * math.sin(p.yaw) - q.y) < 1e-6
                       for q in starts)
    with pytest.raises(PlanningError, match='blade_ahead'):
        PlannerConfig(blade_ahead=2.0).validate()


def test_robot_width_narrower_than_the_loop_changes_nothing():
    cfg = PlannerConfig(**FORWARD)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg)
    narrow = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(FORWARD, perimeter_offset=0.1, robot_width=0.1,
                                                                     edges='hard')))
    wider = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(FORWARD, perimeter_offset=0.1)))
    assert [(p.x, p.y) for p in narrow.poses] == [(p.x, p.y) for p in wider.poses]
    assert plan.poses != wider.poses


def test_body_and_blade_settings_out_of_range():
    for kw, match in ((dict(robot_width=-0.1), 'robot_width'), (dict(blade_offset=1.5), 'blade_offset'),
                      (dict(blade_offset=math.nan), 'blade_offset'), (dict(robot_width=True), 'robot_width')):
        with pytest.raises(PlanningError, match=match):
            PlannerConfig(**kw).validate()
    assert PlannerConfig().with_overrides({'robot_width': None, 'blade_offset': 0}).blade_offset == 0.0
    for kw, match in ((dict(robot_front=-0.1), 'robot_front'), (dict(robot_rear=math.inf), 'robot_rear'),
                      (dict(edges='soft'), 'edges')):
        with pytest.raises(PlanningError, match=match):
            PlannerConfig(**kw).validate()
    assert PlannerConfig().with_overrides({'robot_front': 1}).robot_front == 1.0


# the body (robot_width, robot_front, robot_rear) -----------------------------------------------------------

NX60I = dict(robot_width=0.40, robot_front=0.43, robot_rear=0.14)
MOWER = dict(FORWARD, mower_width=0.18, overlap=1 - 0.13 / 0.18, perimeter_offset=0.0, blade_ahead=0.17,
             perimeter_passes=4, lane_overlap_passes=1)


def test_robot_width_alone_changes_nothing_on_recorded_edges():
    plain = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**FORWARD))
    width = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(FORWARD, robot_width=0.4)))
    assert width.poses == plain.poses and width.body_space is None
    assert check_plan(width)['body_out'] == []


def test_footprint():
    fp = Footprint(Polygon(rect(0, 0, 10, 10)), 0.4, 0.43, 0.14)
    assert fp.fits_pose(5, 5, 0.0) and fp.fits_pose(9.5, 5, 0.0) and not fp.fits_pose(9.6, 5, 0.0)
    assert fp.fits_pose(0.2, 5, 0.0) and not fp.fits_pose(0.1, 5, 0.0)  # the rear
    assert fp.fits_turn(0.5, 0.5, 0.0, math.pi) and not fp.fits_turn(0.2, 5, 0.0, math.pi / 2)
    # turning on the spot where the heading jumps from the pose before: half way round the rear corner sticks out
    poses = [Pose2D(9.4, 0.22, 0.0), Pose2D(9.5, 0.22, math.pi / 2), Pose2D(9.5, 0.32, math.pi / 2)]
    assert fp.bad_poses(poses) == [1] and fp.spins(poses, 1) and not fp.poses_fit(poses)
    # about a half turn either way round (the front swings over the edge turning right), less the shorter way only
    assert not fp.fits_turn(5, 0.45, 0.0, math.radians(175)) and fp.fits_turn(5, 0.45, 0.0, math.radians(160))
    # where the heading turns by less than SPIN only the poses count
    assert fp.poses_fit([Pose2D(9.5, 5, 0.0), Pose2D(9.5, 5.01, math.radians(25))])


def test_footprint_fit(monkeypatch):
    """fit() is fits_pose for every pose: all at once with shapely 2, one at a time with shapely 1.7 (their points at
    once with numpy). Whether the body is in the room, also just off a corner of what's taken out of it, the front's
    corner facing it, or the side of a half facing it."""
    room = Polygon(rect(0, 0, 4, 3)).difference(Polygon(rect(1.5, 1, 2.5, 2)))
    rng = random.Random(5)
    poses = [Pose2D(rng.uniform(-0.2, 4.2), rng.uniform(-0.2, 3.2), rng.uniform(-math.pi, math.pi)) for _ in range(500)]
    for (cx, cy), r, a in itertools.product(rect(1.5, 1, 2.5, 2), (0.465, 0.47), range(0, 360, 3)):
        x, y = cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a))
        poses.append(Pose2D(x, y, math.atan2(cy - y, cx - x) - math.atan2(0.2, 0.43)))
    for (cx, cy), r, a in itertools.product(rect(1.5, 1, 2.5, 2), (0.24, 0.25), range(0, 360, 3)):
        # (the front half's middle that far off the corner, the half's corner facing it)
        x, y, h = cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)), math.atan2(0.2, 0.1425)
        poses.append(Pose2D(x - 0.2875 * math.cos(math.radians(a) + math.pi + h),
                            y - 0.2875 * math.sin(math.radians(a) + math.pi + h), math.radians(a) + math.pi + h))
    for mode in ('shapely 2', 'shapely 1.7', 'shapely 1.7 with numpy'):
        if mode != 'shapely 2':
            monkeypatch.setattr(footprint, 'contains_xy', None)
            monkeypatch.setattr(footprint, 'contains_points', None if mode == 'shapely 1.7' else
                                lambda prepared, x, y: footprint.numpy.array([prepared.context.contains(Point(p))
                                                                              for p in zip(x, y)], bool))
        fp = Footprint(room, 0.4, 0.43, 0.14)
        fits = fp.fit(poses)
        assert fits == [fp.fits_pose(p.x, p.y, p.yaw) for p in poses] and 50 < sum(fits[:500]) < 450
        assert fits == [room.covers(fp.body(p.x, p.y, p.yaw)) for p in poses]


def test_zones_keep_off():
    """space.buffer(-d / KEEP_OFF, 4) keeps d off space's edge, also where it bends a little at every point (recorded)
    and round corners of every angle (GEOS rounds them coarser than with 4 sides a quarter circle)."""
    rng = random.Random(3)
    outline = [(10 * math.cos(t) + rng.uniform(-0.03, 0.03), 6 * math.sin(t) + rng.uniform(-0.03, 0.03))
               for t in (2 * math.pi * i / 900 for i in range(900))]
    space = Polygon(outline, [circle(-4, 0, 0.5, 7)[::-1], rect(2, -1, 3, 1)[::-1],
                              [(5, 2), (6.5, 2.2), (5.2, 2.6)]])
    assert space.is_valid
    for d in (0.2456, 0.4742, 1.0):
        # (what's in the zone is no closer to the edge than the zone's own edge)
        assert space.buffer(-d / footprint.KEEP_OFF, 4).boundary.distance(space.boundary) >= d


def test_zone_has_points_for_sure(monkeypatch):
    """A _Zone has a point only where it lies in it, on its edge at most (where the square of the map it's in does): on
    and next to its edge, round its corners and holes, far out; and every point further in than a few squares."""
    rng = random.Random(4)
    geom = Polygon(circle(0, 0, 3, 50), [rect(-1, -1, 0.5, 0.3)[::-1], circle(1.5, 1, 0.4, 7)[::-1]])
    edge = [geom.boundary.interpolate(rng.uniform(0, geom.boundary.length)) for _ in range(3000)]
    pts = [(p.x + rng.gauss(0, 0.03), p.y + rng.gauss(0, 0.03)) for p in edge] + [(p.x, p.y) for p in edge]
    pts += [(rng.uniform(-4, 4), rng.uniform(-4, 4)) for _ in range(5000)] + [(1e6, -1e6), (-3.0, 0.0)]
    far = geom.buffer(-3 * max(footprint.RASTER, footprint.CELL))
    for mode in ('shapely 2', 'shapely 1.7'):
        if mode != 'shapely 2':
            monkeypatch.setattr(footprint, 'contains_xy', None)
        zone = footprint._Zone(geom)
        has = [zone.has(x, y) for x, y in pts]
        assert not any(h and not geom.covers(Point(p)) for p, h in zip(pts, has))
        assert all(h for p, h in zip(pts, has) if far.contains(Point(p))) and sum(has) > 1500


def test_body_turns_and_lane_ends_stay_inside():
    """The turns fit with the body: lane ends pulled back for the U-turns, no turn on the spot or replaced by a
    drive, and the body nowhere it wasn't when the edges were recorded (driving along them, turning at the corners).
    The outer loop, its centre on the line (perimeter_offset 0, no edge_margin), can't turn round a corner or get on
    the line at a slant without its rear or front swinging out: the path gets off the line before a corner, turns
    further in and gets back on after it, easing off and on, nowhere jumping."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, edge_margin=0.0))
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg)
    assert plan.stats['turns_u_turn'] == plan.stats['lanes'] - 1 > 0
    assert plan.stats['turns_pivot'] == plan.stats['turns_fallback'] == plan.stats['turns_detour'] == 0
    report = check_plan(plan)
    # (left: the corners, round them further in)
    assert report['body_out'] == [] and report['left_m2'] < 0.5
    fit = plan.stats['body_fit']
    assert fit['jumps'] == [] and len(plan.section_starts) == 1
    ways = [s for s in plan.segments if s.kind == TRANSIT and s.maneuver == 'body']
    for cx, cy in rect(0, 0, 10, 6)[1:]:
        assert any(min(math.hypot(p.x - cx, p.y - cy) for p in plan.poses[s.start:s.end + 1]) < 0.6 for s in ways)
    # the lanes end where the U-turn's arc keeps the front in: a turn of 0.25 reaches 0.62 past the end, the body
    # may go 0.2 past the edge
    reach = math.hypot(0.25 + 0.2, 0.43)
    ends = [plan.poses[i].x for s in plan.segments if s.maneuver == 'u_turn' for i in (s.start, s.end)]
    assert min(ends) >= reach - 0.2 - 0.02 and max(ends) <= 10 + 0.2 - reach + 0.02
    assert any('lanes shortened' in w for w in plan.warnings)


def test_body_in_the_garden():
    obstacles = [TREE, BED]
    for edges in ('recorded', 'hard'):
        cfg = PlannerConfig(**dict(MOWER, **NX60I, edges=edges))
        plan = plan_coverage(GARDEN, obstacles, cfg, start=(1, 1, 0.0), end=(1, 1))
        check_invariants(plan, cfg, obstacles)
        # (where a lane end is far from the last one's, a drive rather than a turn leaving half the lane out)
        assert plan.stats['turns_fallback'] + plan.stats['turns_pivot'] <= 3
        report = check_plan(plan)
        # where it would stick out the path goes another way
        assert report['body_out'] == [] and not any('body sticks out' in h for h in report['hard'])
        assert plan.stats['body_fit']['fixed'] > 0 and plan.stats['body_fit']['jumps'] == []
        # left: the corners the body keeps the mower out of (rounded further in), and the lane starts of the turns
        assert report['left_m2'] < 1.0


def test_body_bent_lanes():
    """Lanes bending around trees get their ends pulled back too: U-turns, nothing replaced by a drive."""
    trees = [circle(4, 4, 0.4), circle(9, 10, 0.5)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, bend_max_gap=3.0))
    plan = plan_coverage(GARDEN, [TREE, BED] + trees, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED] + trees)
    assert any(s.maneuver == 'bend' for s in plan.segments)
    assert plan.stats['turns_fallback'] == 0 and plan.stats['turns_pivot'] <= 2
    assert not [o for o in check_plan(plan)['body_out'] if o['kind'] in ('turn', 'lane_end')]


def test_body_where_it_was_while_recording():
    """The body may be where it was while the edges were recorded, its centre on them the way their points go: at a
    corner it turned on the spot from one edge's heading to the next one's, its front ahead past the corner and its rear
    behind (recorded the other way round, that's on the other side of the corner), but facing out of the corner its
    front would be where it never was."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, edge_margin=0.0))
    ccw = plan_coverage(rect(0, 0, 10, 6), [], cfg, start=(1, 1, 0.0))
    cw = plan_coverage(rect(0, 0, 10, 6)[::-1], [], cfg, start=(1, 1, 0.0))
    fp = plan_footprint(ccw)
    # (a mm more: right on the edge of the space floats may count it out)
    room = ccw.body_space.buffer(1e-3)
    assert all(room.covers(fp.body(0.0, 0.0, (k / 18 - 0.5) * math.pi)) for k in range(10))
    assert not fp.fits_pose(0.0, 0.0, -0.75 * math.pi)
    # at (10, 0) ccw arrived facing +x, cw facing -y
    ahead, below = Point(10.4, -0.1), Point(10.1, -0.4)
    assert ccw.body_space.contains(ahead) and not ccw.body_space.contains(below)
    assert cw.body_space.contains(below) and not cw.body_space.contains(ahead)


@pytest.mark.parametrize('deg', [0, 33, 101])
def test_body_fits_at_any_angle(deg):
    """Where the body would stick out the path goes another way, whatever the angle: nothing sticks out (also against
    the real edges worked out here on their own), nowhere does it jump, the rest of the path is as planned, and stats
    has every way in place. Left: where the loops along the lines don't turn round a corner or get on their line
    without the body's rear or front swinging out, they get off it before the corner and back on after it."""
    obstacles = [TREE, BED]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=math.radians(deg)))
    plan = plan_coverage(GARDEN, obstacles, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, obstacles)
    # without it: what sticks out are loops, spins and drives, the turns and lane ends fit on their own
    off = plan_coverage(GARDEN, obstacles, dataclasses.replace(cfg, body_fit=False), start=(1, 1, 0.0), end=(1, 1))
    report, before = check_plan(plan), check_plan(off)
    assert report['body_out'] == [] and report['left_m2'] <= before['left_m2'] + 0.8
    assert outside_room(plan, recorded_room(cfg, [GARDEN], obstacles, [cfg.obstacle_margin] * 2)) == []
    assert plan.stats['body_fit']['jumps'] == [] and len(plan.section_starts) == 1
    out = before['body_out']
    assert {'x', 'y', 'yaw', 'kind'} == set(out[0]) and not [o for o in out if o['kind'] in ('turn', 'lane_end')]
    planned = {(p.x, p.y) for p in off.poses}
    ways = [s for s in plan.segments if s.kind == TRANSIT and s.maneuver == 'body']
    assert all((p.x, p.y) in planned for s in plan.segments if s not in ways for p in plan.poses[s.start:s.end + 1])
    fit = plan.stats['body_fit']
    assert fit['fixed'] == len(ways) > 0 and len(fit['places']) == fit['fixed'] + fit['left']
    assert fit['skipped_m'] == pytest.approx(sum(m for _, _, m in fit['places']), abs=0.01)
    for s, (x, y, _) in zip(ways, fit['places']):
        p = plan.poses[(s.start + s.end) // 2]
        assert (x, y) == (round(p.x, 2), round(p.y, 2))
    lines = unary_union([Polygon(ring).exterior for ring in [GARDEN] + obstacles])
    assert all(lines.distance(Point(x, y)) < 0.3 for x, y, _ in fit['places'][fit['fixed']:])
    assert any('places left out' in w for w in plan.warnings) == (fit['left'] > 0)


def test_body_fits_through_narrow_gaps():
    """A table recorded 8 cm off the outline, a sunshade 18 cm: the loops between them and the outline turn where the
    body doesn't fit, the path goes another way there, stats saying where and what that leaves out."""
    table, shade = rect(3, 5.0, 5, 5.92), circle(8, 5.47, 0.35, 16)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3, narrow_parts='loops', min_turn_radius=0.25,
                               perimeter_passes=3))
    plan = plan_coverage(rect(0, 0, 10, 6), [table, shade], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [table, shade])
    assert check_plan(plan)['body_out'] == []
    fit = plan.stats['body_fit']
    gap = Polygon(rect(7, 5.5, 9, 6))
    assert sum(m for x, y, m in fit['places'] if gap.contains(Point(x, y))) > 0.5


def test_body_drives_through_the_area():
    """Where a drive finds no way through the transit space (to a lane's end a little out of it) it goes through the
    mowing area (_Route.go_to), with the body checked up to body_tolerance further out: no finding of check_plan, the
    body keeps the margins there (also against the real edges worked out here on their own). A pose 1 cm further out
    is one."""
    trees = [circle(4, 4, 0.4), circle(8, 4.3, 0.5)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, perimeter_passes=3, lane_overlap_passes=2))
    plan = plan_coverage(rect(0, 0, 12, 8), trees, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, trees)
    transit_space = plan.transit_space.buffer(2e-3)
    out = [i for s in plan.segments if s.kind == TRANSIT for i in range(s.start, s.end + 1)
           if not transit_space.covers(Point(plan.poses[i].x, plan.poses[i].y))]
    assert out
    report = check_plan(plan)
    assert report['body_out'] == [] and not any('poses outside' in h for h in report['hard'])
    assert outside_room(plan, recorded_room(cfg, [rect(0, 0, 12, 8)], trees, [cfg.obstacle_margin] * 2)) == []
    p = plan.poses[out[0]]
    q = nearest_points(plan.robot_space.boundary, Point(p.x, p.y))[0]
    d = math.hypot(q.x - p.x, q.y - p.y)
    poses = list(plan.poses)
    poses[out[0]] = p._replace(x=q.x + 0.01 * (q.x - p.x) / d, y=q.y + 0.01 * (q.y - p.y) / d)
    report = check_plan(dataclasses.replace(plan, poses=poses))
    assert '1 poses outside the space the mower may drive in' in report['hard']


def test_body_fit_starts_later(monkeypatch):
    """Where nothing fits FIT_START after the plan's (a section's) start at most, it starts later instead of jumping
    there (or turning round, FIT_ROUND 0 here), the body fitting from there. At the plan's start, by the dock, that's no
    place left out: the mower drives to its first pose on its own. Places left out further on still are, with a
    warning."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    monkeypatch.setattr(planner, 'FIT_ROUND', 0)
    plans = []
    for start in (0.0, FIT_START):
        monkeypatch.setattr(planner, 'FIT_START', start)
        plans.append(plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1)))
    jumped, later = (p.stats['body_fit'] for p in plans)
    assert len(jumped['jumps']) == len(plans[0].section_starts) - 1 > 0 and later['jumps'] == []
    assert jumped['left'] == len(jumped['jumps']) and any('places left out' in w for w in plans[0].warnings)
    assert len(plans[1].section_starts) == 1 and plans[1].poses[0] != plans[0].poses[0]
    assert plan_footprint(plans[1]).fits_pose(*plans[1].poses[0])
    assert later['left'] == 0 and not any('places left out' in w for w in plans[1].warnings)
    assert path_length(plans[1].poses) < path_length(plans[0].poses)
    # nothing tried for a place: those further on are left out, the path jumping over them, the start still isn't one
    monkeypatch.setattr(planner, 'FIT_PLACE', (0, 0))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    fit = plan.stats['body_fit']
    assert plan.poses[0] == plans[1].poses[0] and fit['left'] == len(fit['jumps']) == len(plan.section_starts) - 1 > 0
    assert any('places left out' in w for w in plan.warnings)


def test_body_fit_late_start_left_out(monkeypatch):
    """Where the body doesn't fit at the plan's start (its first poses here) it starts later, no place left out while
    that leaves FIT_START m of loops and lanes out at most. With more it's a place left out like any other, a warning
    saying so."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    lawn, dock = rect(0, 0, 10, 6), (5, -0.3, 0.0)
    planned = plan_coverage(lawn, [], dataclasses.replace(cfg, body_fit=False), start=dock, end=dock[:2])
    fits_pose, fit = Footprint.fits_pose, Footprint.fit
    for gone in (0.5 * FIT_START, 3.0):
        k = next(k for k in range(len(planned.poses)) if path_length(planned.poses[:k + 1]) > gone)
        head = {(p.x, p.y, p.yaw) for p in planned.poses[:k]}  # the body doesn't fit there
        monkeypatch.setattr(Footprint, 'fits_pose', lambda fp, x, y, yaw: (x, y, yaw) not in head
                            and fits_pose(fp, x, y, yaw))
        monkeypatch.setattr(Footprint, 'fit', lambda fp, poses: [f and (p.x, p.y, p.yaw) not in head
                                                                 for f, p in zip(fit(fp, poses), poses)])
        planner._FITTED.clear()
        plan = plan_coverage(lawn, [], cfg, start=dock, end=dock[:2])
        fitted = plan.stats['body_fit']
        assert plan.poses[0] == planned.poses[k] and fitted['jumps'] == [] and len(plan.section_starts) == 1
        warned = any("places left out where the body doesn't fit" in w for w in plan.warnings)
        if gone < FIT_START:
            assert fitted['left'] == 0 and not warned
            continue
        mid = planned.poses[k // 2]
        assert fitted['left'] == 1 and warned
        assert fitted['places'][-1] == [round(mid.x, 2), round(mid.y, 2), pytest.approx(gone, abs=0.1)]
    planner._FITTED.clear()


def test_body_fit_turns_round():
    """A dead end narrowing to a point, too narrow to turn round in at its end without backing up: the path drives in
    only as far as it can still turn round and turns there, nowhere jumping, nothing sticking out (also against the real
    edges worked out here on their own), stats saying where and what that leaves out."""
    shape = [(0, 0), (6, 0), (6, 3), (3.5, 3), (3.1, 9), (2.9, 9), (2.5, 3), (0, 3)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    plan = plan_coverage(shape, [], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg)
    fit = plan.stats['body_fit']
    assert fit['jumps'] == [] and len(plan.section_starts) == 1 and fit['left'] == 0
    assert check_plan(plan)['body_out'] == [] and outside_room(plan, recorded_room(cfg, [shape], [], [])) == []
    # (0.54 m between the lines there, the body's 0.4 and its margins: 1 m wide, room to turn round one way)
    assert 6.0 < max(p.y for p in plan.poses) < 7.0
    assert any(y > 4.0 and m > 0 for _, y, m in fit['places']) and any('driven another way' in w for w in plan.warnings)


def test_body_fit_bounded(monkeypatch):
    """A dead end too narrow to turn round in without backing up: the plan gets done, nothing sticking out, no more than
    FIT_PLACE ways tried for a place (none: none tried), FIT_ROUND turning round there, nor FIT_TRIES for all. Where it
    may try none it leaves out what doesn't fit and jumps over it, a warning saying where."""
    shape = [(0, 0), (6, 0), (6, 3), (3.3, 3), (3.3, 13), (2.7, 13), (2.7, 3), (0, 3)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    for tries, place, turning in ((FIT_TRIES, FIT_PLACE, FIT_ROUND), (50, FIT_PLACE, FIT_ROUND),
                                  (FIT_TRIES, (0, 0), 0)):
        monkeypatch.setattr(planner, 'FIT_TRIES', tries)
        monkeypatch.setattr(planner, 'FIT_PLACE', place)
        monkeypatch.setattr(planner, 'FIT_ROUND', turning)
        plan = plan_coverage(shape, [], cfg, start=(1, 1, 0.0))
        check_invariants(plan, cfg)
        fit = plan.stats['body_fit']
        assert check_plan(plan)['body_out'] == [] and fit['tries'] <= tries and (fit['tries'] > 0) == (sum(place) > 0)
        assert bool(fit['jumps']) == (tries == 50 or not turning) == (len(plan.section_starts) > 1)
        if fit['jumps']:
            assert all(y > 9 for _, y in fit['jumps'][-1:])
            assert any('places left out' in w and f'({fit["jumps"][-1][0]:.2f}, {fit["jumps"][-1][1]:.2f})' in w
                       for w in plan.warnings)
        if tries == 50:
            assert fit['tries'] == 50


def recorded_room(cfg, outlines, holes, margins):
    """Where the body keeps the margins off the real edges, worked out here on its own: the body every 5 cm along each
    recorded ring the way its points go and turning on the spot at its points every 2 degrees, the outlines grown by
    that and edge_margin in, the holes (obstacles, areas to keep out of) less that and each one's margin out."""
    half, front, rear = 0.5 * cfg.robot_width, cfg.robot_front, cfg.robot_rear

    def body(x, y, h):
        c, s = math.cos(h), math.sin(h)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy)
                        for dx, dy in ((front, half), (-rear, half), (-rear, -half), (front, -half))])

    def swept(ring):
        bodies = []
        for i, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
            c = ring[(i + 2) % len(ring)]
            h, turn = math.atan2(b[1] - a[1], b[0] - a[0]), 0.0
            n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05)))
            bodies += [body(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, h) for k in range(n + 1)]
            turn = normalize_angle(math.atan2(c[1] - b[1], c[0] - b[0]) - h)
            m = max(1, int(math.ceil(abs(turn) / math.radians(2))))
            bodies += [body(b[0], b[1], h + turn * k / m) for k in range(1, m)]
        return unary_union(bodies)

    outer = unary_union([Polygon(o).union(swept(list(o))) for o in outlines]).buffer(-cfg.edge_margin, 16)
    return outer.difference(unary_union([Polygon(w).difference(swept(list(w))).buffer(m, 16)
                                         for w, m in zip(holes, margins)]))


def outside_room(plan, room, slack=1e-3):
    """The poses whose body isn't in room (slack more), turning on the spot every 2 degrees where the heading jumps by
    more than SPIN, both ways round about a half turn (TURN_STEP apart the corners may pass 2 mm further out)."""
    fp, ok, turning = plan_footprint(plan), prep(room.buffer(slack, 16)), prep(room.buffer(slack + 2e-3, 16))
    bad, starts = [], set(plan.section_starts)
    for i, p in enumerate(plan.poses):
        q = plan.poses[i - 1] if i not in starts else None
        if q is None or angle_diff(p.yaw, q.yaw) <= footprint.SPIN:
            if not ok.covers(fp.body(p.x, p.y, p.yaw)):
                bad.append(i)
            continue
        turn = normalize_angle(p.yaw - q.yaw)
        for t in (turn, turn - math.copysign(2 * math.pi, turn)) if abs(turn) > math.pi - footprint.TURN_STEP else \
                (turn,):
            n = int(math.ceil(abs(t) / math.radians(2)))
            if not all(turning.covers(fp.body(p.x, p.y, q.yaw + t * k / n)) for k in range(n + 1)):
                bad.append(i)
                break
    return bad


def test_margins_kept():
    """The body keeps edge_margin off the outline's real edge, obstacle_margin off an obstacle's and an area's to keep
    out of, everywhere (body_tolerance doesn't count): against the real edges worked out here on their own, and along
    the outline's straight edges against the line robot_width / 2 further out. The loops keep the margins themselves
    (and simplify_tolerance), the first one further in, the ones round an obstacle further out."""
    lawn, bed, keep = rect(0, 0, 10, 6), rect(4, 2, 6, 4), rect(7, 4, 11, 9)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3, edge_margin=0.08, obstacle_margin=0.15))
    plan = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0), end=(1, 1), keep_out=[keep])
    check_invariants(plan, cfg, [bed])
    assert check_plan(plan)['body_out'] == []
    assert outside_room(plan, recorded_room(cfg, [lawn], [bed, keep], [cfg.obstacle_margin] * 2)) == []
    corners, fp = MultiPoint(lawn).buffer(1.0), plan_footprint(plan)
    line = Polygon(lawn).buffer(0.5 * cfg.robot_width - cfg.edge_margin + 1e-3, join_style=2)
    assert all(line.covers(fp.body(p.x, p.y, p.yaw)) for p in plan.poses if not corners.contains(Point(p.x, p.y)))
    loops = [p for s in plan.segments if s.kind == PERIMETER for p in plan.poses[s.start:s.end + 1]]
    for ring, margin in ((lawn, cfg.edge_margin), (bed, cfg.obstacle_margin), (keep, cfg.obstacle_margin)):
        near = min(Polygon(ring).exterior.distance(Point(p.x, p.y)) for p in loops)
        assert near == pytest.approx(margin + cfg.simplify_tolerance, abs=0.01)
    free = plan_coverage(lawn, [bed], dataclasses.replace(cfg, edge_margin=0.0, obstacle_margin=0.0), start=(1, 1, 0.0),
                         end=(1, 1), keep_out=[keep])
    assert free.body_space.area > plan.body_space.area + 0.08 * 30 and free.stats['area'] > plan.stats['area']


def test_margin_of_an_obstacle():
    """An obstacle's own margin (an area's to keep out of) beats obstacle_margin: the loops round it go that much
    further out, the body keeps that far off its real edge."""
    a, b, keep = rect(2, 2, 3, 3), rect(6, 2, 7, 3), rect(4, 5, 5, 7)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    plan = plan_coverage(rect(0, 0, 10, 6), [a, b], cfg, start=(1, 1, 0.0), keep_out=[keep], margins=[0.3, None, 0.2])
    check_invariants(plan, cfg, [a, b])
    assert check_plan(plan)['body_out'] == []
    loops = [p for s in plan.segments if s.kind == PERIMETER for p in plan.poses[s.start:s.end + 1]]
    near = [min(Polygon(o).exterior.distance(Point(p.x, p.y)) for p in loops) for o in (a, b, keep)]
    assert near == pytest.approx([0.3, cfg.obstacle_margin, 0.2], abs=0.01)
    room = recorded_room(cfg, [rect(0, 0, 10, 6)], [a, b, keep], [0.3, cfg.obstacle_margin, 0.2])
    assert outside_room(plan, room) == []
    for margins, match in (([0.1] * 4, '4 margins'), ([-0.1], 'margin'), (['far'], 'margin'), ([math.nan], 'margin'),
                           ([True], 'margin')):
        with pytest.raises(PlanningError, match=match):
            plan_coverage(rect(0, 0, 10, 6), [a, b], cfg, keep_out=[keep], margins=margins)
    for kw in (dict(edge_margin=-0.1), dict(obstacle_margin=1.5), dict(edge_margin=math.nan)):
        with pytest.raises(PlanningError, match='margin'):
            PlannerConfig(**kw).validate()


def field_turns(plan, field):
    """The poses of drives in field whose heading changes from the pose before (on an arc, turning on the spot)."""
    return [i for s in plan.segments if s.kind == TRANSIT for i in range(s.start + 1, s.end + 1)
            if angle_diff(plan.poses[i].yaw, plan.poses[i - 1].yaw) > 1e-6
            and field.contains(Point(plan.poses[i].x, plan.poses[i].y))]


def lanes_field(plan, cfg):
    """The lanes' field worked out here on its own: the lanes of the plan's cells 2 turn_radius shorter at either end
    and half a lane spacing wide each side, but for the strips the loops mow (2 cm less all round: a drive along its
    edge may turn there)."""
    strips, trim = [], 2 * cfg.turn_radius
    for a, cells in plan.cells:
        for lane in (lane for cell in cells for lane in cell if lane.x1 - lane.x0 > 2 * trim):
            line = LineString(Lane(lane.row, lane.x0 + trim, lane.x1 - trim, lane.y).ends(a))
            strips.append(line.buffer(0.5 * plan.lane_spacing, cap_style=2))
    reach = cfg.blade_loop_offset + (cfg.passes - 1) * plan.lane_spacing + 0.5 * cfg.mower_width
    return unary_union(strips).intersection(plan.free_space.buffer(-reach)).buffer(-0.02)


def far_turns_in(plan, field, apart):
    """The turns between lanes more than apart from each other that go into field."""
    def far(s):
        a, b = plan.poses[s.start], plan.poses[s.end]
        return abs((b.y - a.y) * math.cos(a.yaw) - (b.x - a.x) * math.sin(a.yaw)) > apart

    return [s for s in plan.segments if s.kind == TURN and far(s)
            and any(field.contains(Point(p.x, p.y)) for p in plan.poses[s.start:s.end + 1])]


def test_headland_turns():
    """headland_turns: in the lanes' field (worked out here on its own) no drive turns, not even on the spot: they go
    along the headland, across the field only straight (there's no way round the tree in the middle), along a lane's
    line where that's not much longer. Nor does a turn between lanes far apart go in there. Without, they do. stats
    says how often the path crosses the field and that a drive turns in it nowhere; as well mowed, nothing sticking
    out."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, min_turn_radius=0.25, angle=0.3))
    plain = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    clean = dataclasses.replace(cfg, headland_turns=True)
    plan = plan_coverage(GARDEN, [TREE, BED], clean, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, clean, [TREE, BED])
    before = lanes_field(plain, cfg)
    assert len(field_turns(plain, before)) > 50 and far_turns_in(plain, before, 4 * cfg.turn_radius)
    assert 'headland_turns' not in plain.stats
    field = lanes_field(plan, clean)
    assert field_turns(plan, field) == [] and far_turns_in(plan, field, 4 * cfg.turn_radius) == []
    across = [p for s in plan.segments if s.kind == TRANSIT for p in plan.poses[s.start:s.end + 1]
              if field.contains(Point(p.x, p.y))]
    assert across and any(angle_diff(2 * p.yaw, 2 * plan.angle) < 1e-9 for p in across)
    found = plan.stats['headland_turns']
    assert found['turns_in_field'] == 0 and found['places'] == [] and found['crossings'] > 0
    assert not any("lanes' field" in w for w in plan.warnings)
    report = check_plan(plan)
    assert report['body_out'] == [] and report['left_m2'] <= check_plan(plain)['left_m2'] + 0.05
    with pytest.raises(PlanningError, match='headland_turns'):
        PlannerConfig(headland_turns=1).validate()


def test_headland_turns_where_nothing_fits(monkeypatch):
    """Where no drive along the headland (nor across the field) fits, it goes the way it would without: stats and a
    warning say where it turns in the lanes' field."""
    monkeypatch.setattr(planner._Route, 'headland_way', lambda self, target, heading, head: None)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, min_turn_radius=0.25, angle=0.3, headland_turns=True))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED])
    found = plan.stats['headland_turns']
    assert found['turns_in_field'] == len(found['places']) > 0 and found['crossings'] == 0
    x, y = found['places'][0]
    assert any("turns in the lanes' field" in w and f'({x:.2f}, {y:.2f})' in w for w in plan.warnings)
    assert len(field_turns(plan, lanes_field(plan, cfg))) > 0


def test_narrow_parts_loops():
    """A winding path 0.5 to 0.8 m wide: narrower than the U-turns need, loops go on in there instead of lanes, the
    pieces they come apart into driven on the way round: shorter, as well mowed (but for what the body doesn't get to
    at the bends and the ends with the margin kept, turning round before an end, the same either way), hardly any
    drives between."""
    left = [(0.4 * math.sin(0.125 * i), 0.25 * i) for i in range(49)]
    path = left + [(x + 0.65 + 0.15 * math.sin(1.3 * y), y) for x, y in left][::-1]
    kw = dict(MOWER, **NX60I, angle=math.pi / 2, perimeter_passes=3)
    lanes = plan_coverage(path, [], PlannerConfig(**kw))
    cfg = PlannerConfig(**dict(kw, narrow_parts='loops'))
    loops = plan_coverage(path, [], cfg)
    check_invariants(loops, cfg)
    assert coverage(loops, cfg) > max(0.92, coverage(lanes, cfg) - 0.005)
    assert loops.stats['path_length'] < 0.85 * lanes.stats['path_length']
    drives = [s for s in loops.segments if s.kind == TRANSIT and s.maneuver != 'body']  # (not the ways in place)
    assert sum(path_length(loops.poses[s.start:s.end + 1]) for s in drives) < 2.0 and loops.stats['lanes'] == 0
    # a wide area keeps its lanes
    wide = plan_coverage(rect(0, 0, 10, 6), [], cfg)
    assert wide.stats['lanes'] > 10
    with pytest.raises(PlanningError, match='narrow_parts'):
        PlannerConfig(narrow_parts='strips').validate()


def test_passes_worked_out():
    """perimeter_passes -1: as many loops as the lanes' turns need, the strip along the edges they leave mowed."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, perimeter_passes=-1))
    # a U-turn of 0.25 reaches 0.62 past the lane end, 0.15 past the edge may be the body's (robot_width / 2,
    # edge_margin less)
    assert cfg.passes == 5
    assert PlannerConfig(**dict(MOWER, **NX60I, perimeter_passes=-1, turn_radius=0.5)).passes == 6
    assert PlannerConfig(**dict(MOWER, perimeter_passes=-1)).passes == 3  # no body: the turn's radius
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    # (left: the corners, the outer loop turns round them further in)
    assert plan.stats['perimeter_loops'] >= 5 and check_plan(plan)['left_m2'] < 0.9
    with pytest.raises(PlanningError, match='perimeter_passes'):
        PlannerConfig(perimeter_passes=-2).validate()


def test_lane_spacing_auto(monkeypatch):
    """lane_spacing_mode auto: the widest spacing that leaves nothing unmowed, the same every time, off again with
    fixed; stats['chosen'] says what it planned with. What the plan leaves isn't worked out where the next spacing's
    loops and lanes leave no less."""
    # (what's left is at the corners the outer loop turns round further in, see
    # test_body_turns_and_lane_ends_stay_inside)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, lane_spacing_mode='auto', edge_margin=0.0))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, plan.config)
    chosen = plan.stats['chosen']
    assert chosen['mode'] == 'auto' and chosen['lane_spacing'] == pytest.approx(0.85 * 0.18)
    assert chosen['perimeter_passes'] == 4 and check_plan(plan)['left_m2'] < 0.8
    swept = []
    monkeypatch.setattr(planner, 'swept_area', lambda *args: swept.append(args) or swept_area(*args))
    again = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    assert [(p.x, p.y) for p in again.poses] == [(p.x, p.y) for p in plan.poses] and not swept
    fixed = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(MOWER, **NX60I)), start=(1, 1, 0.0), end=(1, 1))
    assert fixed.stats['chosen'] == {'lane_spacing': 0.13, 'perimeter_passes': 4, 'mode': 'fixed'}
    # a range of one spacing plans that one
    one = PlannerConfig(**dict(MOWER, lane_spacing_mode='auto', lane_spacing_min=0.12, lane_spacing_max=0.12))
    assert plan_coverage(rect(0, 0, 6, 4), [], one).stats['chosen']['lane_spacing'] == pytest.approx(0.12)
    for kw, match in ((dict(lane_spacing_mode='free'), 'lane_spacing_mode'),
                      (dict(lane_spacing_min=0.15, lane_spacing_max=0.1), 'lane_spacing_min'),
                      (dict(lane_spacing_max=0.5), 'lane_spacing_max')):
        with pytest.raises(PlanningError, match=match):
            PlannerConfig(**dict(MOWER, **kw)).validate()


def test_omega():
    for deg in (60, 90, 150, -120, 180):
        loop = omega(Pose2D(1.0, 2.0, 0.3), math.radians(deg), 0.25, 0.05)
        assert (loop[0].x, loop[0].y, loop[-1].x, loop[-1].y) == (1.0, 2.0, 1.0, 2.0)
        assert loop[-1].yaw == pytest.approx(normalize_angle(0.3 + math.radians(deg)))
        assert all(angle_diff(a.yaw, b.yaw) <= math.radians(10.0) + 1e-9 for a, b in zip(loop, loop[1:]))
        assert all(math.hypot(b.x - a.x, b.y - a.y) <= 0.05 + 1e-6 for a, b in zip(loop, loop[1:]))


def test_mowed_near_lane_ends():
    """mowed() skips the poses whose box is far from the lines asked about: the same as looking at every pose, also
    where they come back near them."""
    space = TransitPlanner(Polygon(rect(-1, -1, 31, 11)), 0.05)
    cfg = PlannerConfig(**MOWER)
    route = _Route(cfg, space, space, None, None, None)
    for k in range(6):
        line = [(0.1 * i, 5 + 4 * math.sin(0.2 * i + k)) for i in range(300)]
        route.add(LANE, [Pose2D(x, y, 0.0) for x, y in (line if k % 2 else line[::-1])])
    ends = [(10, 4, 12, 4.5), (20, 2, 20.3, 1)]
    m = 0.5 * cfg.mower_width + cfg.blade_ahead + cfg.waypoint_spacing
    boxes = [(min(x0, x1) - m, min(y0, y1) - m, max(x0, x1) + m, max(y0, y1) + m) for x0, y0, x1, y1 in ends]
    runs = []
    for seg in route.segments:
        run = []
        for p in seg.poses + [None]:
            if p is not None and any(x0 <= p.x <= x1 and y0 <= p.y <= y1 for x0, y0, x1, y1 in boxes):
                run.append(p)
            elif len(run) > 1:
                runs.append(run)
                run = []
            else:
                run = []
    assert len(runs) > 8 and route.mowed(ends, route.segments).wkb == unary_union([route.blade(r) for r in runs]).wkb


def test_body_space_kept(monkeypatch):
    """The body space of the last plans is taken as it is for the same area again (another angle), by every thread:
    the plan is the same as with it worked out afresh, and gets a copy of it. Another margin works it out again. Each
    thread has its own Footprint (shapely's prepared geometries aren't for two threads at once): plans in two at the
    same time are the same."""
    lawn, bed = rect(0, 0, 6, 4), rect(2.5, 1.5, 3.5, 2.5)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, perimeter_passes=2))
    first = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    kept = planner._BODY_SPACE.last
    cfg = dataclasses.replace(cfg, angle=0.5)
    again = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert planner._BODY_SPACE.last is kept and again.body_space is not first.body_space
    del planner._BODY_SPACE.last
    planner._BODY_SPACES.clear()
    planner._FITTED.clear()
    fresh = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert planner._BODY_SPACE.last is not kept
    assert fresh.poses == again.poses and fresh.body_space.equals(first.body_space)
    other = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0), margins=[0.2])
    assert not other.body_space.equals(fresh.body_space)
    done, worked_out = [], []
    driven = planner._driven
    monkeypatch.setattr(planner, '_driven', lambda *args: worked_out.append(args) or driven(*args))

    def plan():
        done.append((plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0)).poses, planner._BODY_SPACE.last))

    threads = [threading.Thread(target=plan) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert [poses for poses, _ in done] == [fresh.poses] * 2 and not worked_out
    assert len({id(own) for _, own in done} | {id(planner._BODY_SPACE.last)}) == 3


def test_body_fit_kept(monkeypatch):
    """Where _fit_body got to is kept every FIT_KEEP poses: a plan at another angle goes on from where its path (the
    loops) was still the same, the same plan as worked out afresh. Other settings of it start afresh."""
    monkeypatch.setattr(planner, 'FIT_KEEP', 50)
    lawn, bed = rect(0, 0, 6, 4), rect(2.5, 1.5, 3.5, 2.5)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, perimeter_passes=2))
    planner._FITTED.clear()
    plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    (key, (_, _, _, first)), = planner._FITTED.items()
    cfg = dataclasses.replace(cfg, angle=0.5)
    again = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    kept = planner._FITTED[key][3]
    assert list(planner._FITTED) == [key] and len(first) > 5 and kept[0] is first[0] and kept != first
    planner._FITTED.clear()
    fresh = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert (fresh.poses, fresh.segments, fresh.stats['body_fit']) == (again.poses, again.segments,
                                                                     again.stats['body_fit'])
    monkeypatch.setattr(planner, 'FIT_DRIVE', 0.3)
    plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert len(planner._FITTED) == 2


def test_arriving_the_other_way_turns_into_it():
    """A drive that would arrive facing the other way: a U-turn into it from beside it, no turn on the spot."""
    space = TransitPlanner(Polygon(rect(0, 0, 10, 6)), 0.05)
    route = _Route(PlannerConfig(**FORWARD), space, space, (8.0, 3.0), math.pi, None)
    route.started = True
    poses = route.drive_to((2.0, 3.0), 0.0)
    assert (poses[-1].x, poses[-1].y) == pytest.approx((2.0, 3.0)) and _spins(poses, math.pi, 0.0) == 0
    assert all(space.is_free([(p.x, p.y)]) for p in poses)


def test_smooth_spins():
    """Turns on the spot by more than SPIN_LIMIT become an omega where there's room, with allow_reverse a K-turn in
    place where there isn't (backing up a segment of its own); the plan as well mowed."""
    def spins(plan):
        return sum(1 for a, b in zip(plan.poses, plan.poses[1:]) if angle_diff(a.yaw, b.yaw) > SPIN_LIMIT)

    plain = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(FORWARD, smooth_spins=False)), start=(1, 1, 0.0))
    left = check_plan(plain)['left_m2']
    cfg = PlannerConfig(**FORWARD)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg, [TREE, BED])
    assert spins(plan) < spins(plain) / 2 and plan.stats['path_length'] > plain.stats['path_length']
    assert not any(s.reverse for s in plan.segments) and check_plan(plan)['left_m2'] == pytest.approx(left, abs=0.02)
    cfg = PlannerConfig(**dict(FORWARD, allow_reverse=True))
    backing = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0))
    check_invariants(backing, cfg, [TREE, BED])
    assert spins(backing) == 0 and any(s.reverse and s.maneuver == 'k_point' for s in backing.segments)
    assert check_plan(backing)['left_m2'] == pytest.approx(left, abs=0.02)


def test_turn_on_spot():
    """turn_on_spot: where the path would drive a loop (a bulb between lanes, a loop out, a drive into a lane round a
    loop or crossing its own way) it turns on the spot instead, the body fitting turning; as well mowed but for a
    little at the lanes' ends. Without, turns and drives cross themselves."""
    def loops(plan):
        # pieces of turns and drives crossing a later piece of their own
        found = 0
        for s in plan.segments:
            if s.kind in (TURN, TRANSIT):
                pts = [(p.x, p.y) for p in plan.poses[s.start:s.end + 1]]
                lines = [LineString(pts[k:k + 2]) for k in range(len(pts) - 1)]
                found += sum(1 for k, a in enumerate(lines) if any(a.crosses(b) for b in lines[k + 2:]))
        return found

    cfg = PlannerConfig(**dict(MOWER, **NX60I, min_turn_radius=0.25, angle=0.3))
    plain = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    spot = dataclasses.replace(cfg, turn_on_spot=True)
    plan = plan_coverage(GARDEN, [TREE, BED], spot, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, spot, [TREE, BED])
    assert loops(plain) > 0 and plain.stats['turns_bulb'] > 0
    assert loops(plan) == 0 and plan.stats['turns_bulb'] == 0 and plan.stats['turns_pivot'] > 0
    report = check_plan(plan)
    assert report['body_out'] == [] and len(plan.section_starts) == 1
    assert report['left_m2'] <= check_plan(plain)['left_m2'] + 0.2
    with pytest.raises(PlanningError, match='turn_on_spot'):
        PlannerConfig(turn_on_spot=1).validate()


def test_min_turn_radius():
    """No turn, drive, loop out or K-turn tighter than min_turn_radius: turning on the spot instead, but for a U-turn
    between lanes where none of it fits, which is rounded as wide as fits (lanes ending earlier for it)."""
    def arcs(plan):
        # radii of the runs of turning poses that turn 45 degrees at least (not counting turns on the spot)
        out, i, P = [], 1, plan.poses
        while i < len(P):
            if 1e-3 < angle_diff(P[i - 1].yaw, P[i].yaw) <= math.radians(30):
                j, turn, length = i, 0.0, 0.0
                while j < len(P) and 1e-3 < angle_diff(P[j - 1].yaw, P[j].yaw) <= math.radians(30):
                    turn += abs(normalize_angle(P[j].yaw - P[j - 1].yaw))
                    length += math.hypot(P[j].x - P[j - 1].x, P[j].y - P[j - 1].y)
                    j += 1
                if turn > math.radians(45):
                    out.append(length / turn)
                i = j
            else:
                i += 1
        return out

    kw = dict(FORWARD, allow_reverse=True)
    loose = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**kw), start=(1, 1, 0.0))
    cfg = PlannerConfig(**dict(kw, min_turn_radius=0.2))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg, [TREE, BED])
    # the loops' corners keep their rounding, so count only what the turns and loops out make
    turns = [s for s in plan.segments if s.kind in (TURN, TRANSIT) or s.maneuver == 'k_point']
    assert turns and min(arcs(loose)) < 0.15
    tight = [s for s in turns if any(r <= 0.2 - 0.02 for r in arcs(
        dataclasses.replace(plan, poses=plan.poses[s.start:s.end + 1])))]
    assert all(s.maneuver == 'u_turn' for s in tight)
    with pytest.raises(PlanningError, match='min_turn_radius'):
        PlannerConfig(min_turn_radius=-0.1).validate()


def test_turn_radius_at_least_min_turn_radius():
    """A turn_radius below min_turn_radius plans as that: the turns aren't all left to drives and loops."""
    base = dict(MOWER, **NX60I, min_turn_radius=0.3)
    low = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(base, turn_radius=0.25)), start=(1, 1, 0.0))
    same = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(base, turn_radius=0.3)), start=(1, 1, 0.0))
    assert [(p.x, p.y, p.yaw) for p in low.poses] == [(p.x, p.y, p.yaw) for p in same.poses]
    assert low.config.turn_radius == 0.3


def test_angle_increment():
    """angle_steps of angle_increment turn the lanes like angle_offset does, bouncing back in angle_min..angle_max;
    the counts are whole numbers."""
    base = dict(MOWER, angle=0.3)
    steps = plan_coverage(rect(0, 0, 8, 5), [], PlannerConfig(**dict(base, angle_increment=0.1, angle_steps=3)))
    offset = plan_coverage(rect(0, 0, 8, 5), [], PlannerConfig(**dict(base, angle_offset=0.3)))
    assert steps.angle == pytest.approx(offset.angle) == pytest.approx(0.6)
    bounced = plan_coverage(rect(0, 0, 8, 5), [], PlannerConfig(**dict(base, angle_increment=0.1, angle_steps=5,
                                                                         angle_min=0.2, angle_max=0.6)))
    assert bounced.angle == pytest.approx(0.4)  # 0.8 is 0.2 past 0.6: back to 0.4
    for kw in (dict(angle_increment_every=0), dict(angle_steps=-1), dict(angle_steps=1.5)):
        with pytest.raises(PlanningError, match='whole number'):
            PlannerConfig(**kw).validate()


def test_rounded_where_min_turn_radius_doesnt_fit():
    """Lanes 0.27 m apart with a least radius of 0.3: a U-turn as wide as fits (half the way across) before turning
    on the spot, which comes only where that doesn't fit either (spin False: none then); without pivots allowed
    nothing."""
    exit_pose, entry = Pose2D(0.0, 0.0, 0.0), (0.0, 0.27)
    free = lambda pts: True  # noqa: E731
    for spin in (True, False):
        turn = plan_turn(exit_pose, entry, 0.3, False, 0.05, free, None, ('u_turn', 'pivot'), least=0.3, spin=spin)
        assert turn.name == 'u_turn'
        assert max(p.x for p in turn.pieces[0].poses) == pytest.approx(0.135, abs=0.01)
    room = lambda pts: all(x <= 0.05 for x, _ in pts)  # noqa: E731 - a wall 5 cm ahead
    assert plan_turn(exit_pose, entry, 0.3, False, 0.05, room, None, ('u_turn', 'pivot'), least=0.3).name == 'pivot'
    assert plan_turn(exit_pose, entry, 0.3, False, 0.05, room, None, ('u_turn', 'pivot'), least=0.3, spin=False) is None
    assert plan_turn(exit_pose, entry, 0.3, False, 0.05, free, None, ('u_turn',), least=0.3) is None



def test_dubins():
    """Dubins' ways end on the entry facing its way, no arc tighter than the radius. Between lanes square to the edge
    the shortest is the U-turn, where the next lane starts further on a U-turn at a slant, shorter than carrying on
    straight first; loops only with loops. With turning the same ways but those turning further."""
    exit_pose = Pose2D(0.0, 0.0, 0.0)
    ways = dubins(exit_pose, Pose2D(0.0, 0.6, math.pi), 0.25, 0.05)
    assert ways[0].name == 'u_turn' and path_length(ways[0].pieces[0].poses) == pytest.approx(0.25 * math.pi + 0.1,
                                                                                               abs=1e-3)
    slant = dubins(exit_pose, Pose2D(0.5, 0.8, math.pi), 0.25, 0.05, loops=False)
    assert [m.name for m in slant] == ['u_turn']
    poses = slant[0].pieces[0].poses
    assert any(p.yaw == pytest.approx(math.atan2(0.3, 0.5)) for p in poses)  # the diagonal
    assert path_length(poses) < path_length(u_turn(0.8, 0.5, 0.25, 0.05).pieces[0].poses) - 0.1
    rng = random.Random(3)
    for _ in range(200):
        a = Pose2D(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-math.pi, math.pi))
        b = Pose2D(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-math.pi, math.pi))
        ways = dubins(a, b, 0.25, 0.05)
        assert len(ways) >= 2
        for m in ways:
            ps = m.pieces[0].poses
            assert ps[0] == a and ps[-1] == b
            for p, q in zip(ps, ps[1:]):
                d = math.hypot(q.x - p.x, q.y - p.y)
                assert 1e-7 < d <= 0.05 + 1e-6 and angle_diff(p.yaw, q.yaw) <= 2.0 * math.asin(d / 0.5) + 1e-6
        turning = rng.uniform(0.5, 3.0 * math.pi)
        few = [m.pieces[0].poses for m in dubins(a, b, 0.25, 0.05, turning=turning)]
        assert few == [m.pieces[0].poses for m in ways if m.pieces[0].poses in few]
        for ps in (m.pieces[0].poses for m in ways):
            turned = sum(angle_diff(p.yaw, q.yaw) for p, q in zip(ps, ps[1:]))
            assert turned <= turning + 1e-3 if ps in few else turned > turning - 1e-3


def test_earlier_only_where_mowed():
    """A turn that ends the lanes earlier, the one just mowed too, only where what that leaves of them gets mowed: the
    lane just mowed doesn't mow its own end."""
    space = TransitPlanner(Polygon(rect(0, 0, 9.23, 4)))  # a wall 3 cm past the lane's end
    cfg = PlannerConfig(**dict(FORWARD, min_turn_radius=0.25, turn_types=('u_turn', 'pivot')))

    def route(loop_from):
        r = _Route(cfg, space, space, (1.0, 1.0), 0.0, None)
        r.add(PERIMETER, sample_straight(9.2, loop_from, 9.2, 3.0, math.pi / 2, 0.05))  # along the wall
        r.add(LANE, sample_straight(1.0, 1.0, 9.2, 1.0, 0.0, 0.05))
        r.lanes.append(((1.0, 1.0), (9.2, 1.0)))
        return r

    assert route(1.3).round_earlier((9.2, 1.6), math.pi, 5.0) is None  # the loop mows only the next lane's start
    turn, entry = route(0.2).round_earlier((9.2, 1.6), math.pi, 5.0)
    assert turn.name == 'u_turn' and entry[0] < 9.2 - 0.05



def test_drive_into_lane_where_mowed():
    """With min_turn_radius a drive arriving the other way round where a lane starts at the wall ends on a way into
    the lane instead of turning on the spot, the lane starting a little further on, only where the loops along the
    wall mow what that leaves of it; the lane goes on from where the drive ends, forwards."""
    space = TransitPlanner(Polygon(rect(0, 0, 10, 4)))
    cfg = PlannerConfig(**dict(FORWARD, min_turn_radius=0.25))

    def route(loops):
        r = _Route(cfg, space, space, (3.0, 2.3), math.pi, None)
        for x in loops:
            r.add(PERIMETER, sample_straight(x, 0.5, x, 3.5, math.pi / 2, 0.05))
        r.pos, r.heading = (3.0, 2.3), math.pi
        return r

    poses = route(()).drive_to((0.05, 2.0), 0.0, (8.0, 2.0))
    assert _spins(poses, math.pi, 0.0) == 1 and (poses[-1].x, poses[-1].y) == pytest.approx((0.05, 2.0))
    poses = route((0.05, 0.18, 0.31)).drive_to((0.05, 2.0), 0.0, (8.0, 2.0))
    assert _spins(poses, math.pi, 0.0) == 0 and poses[-1].y == 2.0 and 0.1 < poses[-1].x <= 0.55
    assert all(space.is_free([(p.x, p.y)]) for p in poses)
    r = route((0.05, 0.18, 0.31))
    r.drive_lanes([Lane(0, 0.05, 8.0, 2.0)], 1, 0.0, lambda x, y: (x, y))
    drive, lane = r.segments[-2:]
    assert drive.kind == TRANSIT and lane.kind == LANE and same_position(drive.poses[-1], lane.poses[0])
    assert all(p.x < q.x for p, q in zip(lane.poses, lane.poses[1:]))


def test_next_cell_beside_turned_into_earlier():
    """The next cell's first lane right beside the lane just mowed and the other way round, at a wall where no turn
    fits: the lane just mowed ends a little earlier for a U-turn into it, no turning on the spot."""
    space = TransitPlanner(Polygon(rect(0, 0, 9.23, 4)))  # a wall 3 cm past the lane's end
    cfg = PlannerConfig(**dict(FORWARD, min_turn_radius=0.25, turn_types=('u_turn', 'pivot')))
    r = _Route(cfg, space, space, (1.0, 1.0), 0.0, None)
    r.add(PERIMETER, sample_straight(9.2, 0.2, 9.2, 3.0, math.pi / 2, 0.05))  # along the wall
    r.add(LANE, sample_straight(1.0, 1.0, 9.2, 1.0, 0.0, 0.05))
    r.lanes.append(((1.0, 1.0), (9.2, 1.0)))
    r.drive_lanes([Lane(0, 1.0, 9.2, 1.6)], -1, 0.0, lambda x, y: (x, y))
    before, turn, lane = r.segments[-3:]
    assert turn.kind == TURN and turn.maneuver == 'u_turn' and lane.kind == LANE
    assert before.poses[-1].x < 9.2 - 0.05


def test_lanes_entered_without_turning_on_the_spot():
    """Where the lanes end at a slant and obstacles split them into cells, the drives to the next cell and the turns
    into it get into the lanes on a way, not turning on the spot where a lane starts."""
    shape = [(0, 0), (8, 0), (8, 6), (0, 8)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, edges='hard', min_turn_radius=0.25, perimeter_passes=3,
                               lane_overlap_passes=2, overlap=1 - 0.153 / 0.18, narrow_parts='loops', angle=0.6,
                               obstacle_margin=0.05))
    plan = plan_coverage(shape, [circle(3, 3, 0.8, 16), circle(6, 4.5, 0.6, 12)], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg)
    poses = plan.poses
    starts = [s.start for s in plan.segments if s.kind == LANE and s.start]
    assert not [i for i in starts if angle_diff(poses[i - 1].yaw, poses[i].yaw) > SPIN_LIMIT]
    # (and the corners of the walls the body doesn't get into without backing up: the loops leave them out)
    report = check_plan(plan)
    assert report['left_m2'] < 1.0 and report['body_out'] == []


def test_bent_lane_entered_on_its_first_piece():
    """A lane bending round an obstacle starts later only on its first straight piece, not behind it."""
    space = TransitPlanner(Polygon(rect(0, 0, 10, 4)).difference(Polygon(rect(0.62, 1.85, 0.95, 2.15))))
    cfg = PlannerConfig(**dict(FORWARD, min_turn_radius=0.25))
    r = _Route(cfg, space, space, (3.0, 2.3), math.pi, None)
    for x in (0.05, 0.18, 0.31):
        r.add(PERIMETER, sample_straight(x, 0.5, x, 3.5, math.pi / 2, 0.05))
    r.pos, r.heading = (3.0, 2.3), math.pi
    lane = Lane(0, 0.05, 8.0, 2.0, ((0.2, 1.3, ((0.2, 2.0), (0.55, 2.3), (1.05, 2.3), (1.3, 2.0))),))
    r.drive_lanes([lane], 1, 0.0, lambda x, y: (x, y))
    ps = r.segments[-1].poses
    assert ps[0].x <= 0.2 + 1e-9
    assert all(angle_diff(math.atan2(q.y - p.y, q.x - p.x), p.yaw) < math.radians(90) for p, q in zip(ps, ps[1:]))

def test_turns_where_lanes_end_at_a_slant():
    """With min_turn_radius, where the lanes end at a slant the turns between them are U-turns that way, the lanes
    ending a little earlier only where that leaves nothing, else loops into the next lane: hardly a drive in place of a
    turn, none on the spot, as well mowed."""
    kite = [(0, 0), (7, 1.5), (9, 6), (1, 7)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, edges='hard', min_turn_radius=0.25, perimeter_passes=3,
                               lane_overlap_passes=2, overlap=1 - 0.153 / 0.18, narrow_parts='loops', angle=0.0))
    plan = plan_coverage(kite, [], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg)
    st = plan.stats
    assert st['turns_fallback'] <= 1 and st['turns_pivot'] == 0 and st['turns_bulb'] > 0
    # (and the corners of the walls the body doesn't get into without backing up: the loops turn round them further
    # in)
    report = check_plan(plan)
    assert report['left_m2'] < 1.1 and report['body_out'] == []

def test_body_corner_rounding():
    """A loop 0.13 m in from a recorded edge: the body doesn't fit round its corner with the corner radius, it does
    with a larger one."""
    fp = Footprint(Polygon(rect(0, 0, 10, 10)).buffer(0.21, 8, join_style=2), 0.4, 0.43, 0.14)
    loop = [(0.13, 0.13), (9.87, 0.13), (9.87, 5.0)]
    plain = drive_polyline(loop, 0.05, 0.15, lambda pts: True, 1, 0.054)
    rounded = drive_polyline(loop, 0.05, 0.15, lambda pts: True, 1, 0.054, body=fp.poses_fit)
    assert fp.bad_poses(plain) and not fp.bad_poses(rounded)


# drives off the edges --------------------------------------------------------------------------------------


def test_careful_path_keeps_off_the_edge():
    space = Polygon(rect(0, 0, 12, 6))
    a, b = (1.0, 0.05), (11.0, 0.05)  # along the bottom edge
    plain = TransitPlanner(space)
    assert plain.careful_path(a, b) == [a, b]  # no edge_distance: the shortest way
    careful = TransitPlanner(space, edge_distance=0.4, edge_cost=2.0)
    way = careful.careful_path(a, b)
    assert way[0] == a and way[-1] == b and len(way) == 4
    assert careful.edge_length(way) == pytest.approx(0.7, abs=0.01)  # only getting in and out
    assert careful.edge_length([a, b]) == pytest.approx(10.0)
    assert careful.cost(way) < careful.cost([a, b])
    assert prep(space.buffer(1e-6)).covers(LineString(way))
    # where keeping off costs more than it's worth, the shortest way
    cheap = TransitPlanner(space, edge_distance=0.4, edge_cost=0.01)
    assert cheap.careful_path(a, b) == [a, b]
    # across the middle nothing changes, nor where the shrunk space has no way (a passage narrower than 0.8 m)
    assert careful.careful_path((2.0, 3.0), (10.0, 3.0)) == [(2.0, 3.0), (10.0, 3.0)]
    neck = Polygon([(0, 0), (5, 0), (5, 2.7), (7, 2.7), (7, 0), (12, 0), (12, 6), (7, 6), (7, 3.3), (5, 3.3),
                    (5, 6), (0, 6)])
    narrow = TransitPlanner(neck, edge_distance=0.4, edge_cost=2.0)
    assert narrow.careful_path((1.0, 3.0), (11.0, 3.0)) == narrow.path((1.0, 3.0), (11.0, 3.0))


def test_transits_keep_off_the_edges():
    many = [circle(5 + 6 * i, 5 + 5 * j, 0.6, 12) for i in range(3) for j in range(2)]
    boundary = rect(0, 0, 20, 14)
    near = {}
    for d in (0.0, 0.4):
        cfg = PlannerConfig(**dict(FORWARD, transit_edge_distance=d))
        plan = plan_coverage(boundary, many, cfg, start=(1, 1, 0.0), end=(1, 1))
        check_invariants(plan, cfg, many)
        assert coverage(plan, cfg) > 0.9999
        band = TransitPlanner(plan.robot_space, edge_distance=0.4, edge_cost=1.0)
        band.careful_path((1, 1), (2, 2))
        ways = [[(p.x, p.y) for p in plan.poses[s.start:s.end + 1]] for s in plan.segments if s.kind == TRANSIT]
        near[d] = sum(band.edge_length(w) for w in ways if len(w) > 1)
    assert near[0.4] < 0.85 * near[0.0]


def test_edge_settings_out_of_range():
    for kw in (dict(transit_edge_distance=-1), dict(transit_edge_cost=math.inf)):
        with pytest.raises(PlanningError, match=list(kw)[0]):
            PlannerConfig(**kw).validate()


# lanes bending around obstacles ----------------------------------------------------------------------------

TREES = [circle(5 + 6 * i, 5 + 5 * j, 0.4, 12) for i in range(4) for j in range(3)]


def test_lane_pieces():
    lane = Lane(0, 0.0, 10.0, 1.0, ((3.0, 4.0, ((3.0, 1.0), (3.5, 1.4), (4.0, 1.0))), (6.0, 6.5, ((6.0, 1.0),
                                                                                                  (6.5, 1.0)))))
    assert lane.pieces() == [(0.0, 3.0), (4.0, 6.0), (6.5, 10.0)]
    assert Lane(0, 1.0, 2.0, 0.0).pieces() == [(1.0, 2.0)]


def test_lanes_bend_around_small_obstacles():
    boundary = rect(0, 0, 30, 20)
    plain = plan_coverage(boundary, TREES, PlannerConfig(**FORWARD), start=(1, 1, 0.0), end=(1, 1))
    cfg = PlannerConfig(**dict(FORWARD, bend_max_gap=3.0))
    plan = plan_coverage(boundary, TREES, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, TREES)
    assert coverage(plan, cfg) > 0.9999
    # one zigzag past all the trees instead of a cell on every side of each
    assert plain.stats['cells'] > 10 and plan.stats['cells'] == 1
    assert plan.stats['transit_length'] < 0.6 * plain.stats['transit_length']
    assert plan.stats['path_length'] < plain.stats['path_length']
    bends = [s for s in plan.segments if s.maneuver == 'bend']
    assert len(bends) >= 12
    # a bend keeps within bend_max_offset of its lane's line (rounding its corners cuts them, never widens them)
    a = plan.angle
    for s in bends:
        across = [-p.x * math.sin(a) + p.y * math.cos(a) for p in plan.poses[s.start:s.end + 1]]
        line = across[0]
        assert across[-1] == pytest.approx(line, abs=1e-6)
        assert max(abs(v - line) for v in across) <= cfg.bend_max_offset + 1e-6


def test_bends_only_within_their_limits():
    # the big tree of the garden: its middle rows would bend further than 0.5 m off their line
    cells = {}
    for offset in (0.5, 1.0):
        cfg = PlannerConfig(**dict(FORWARD, bend_max_gap=3.0, bend_max_offset=offset))
        plan = plan_coverage(GARDEN, [TREE, BED], cfg)
        check_invariants(plan, cfg, [TREE, BED])
        assert coverage(plan, cfg) > 0.9999
        cells[offset] = plan.stats['cells']
    plain = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**FORWARD)).stats['cells']
    assert cells[1.0] < cells[0.5] <= plain
    # gaps longer than bend_max_gap stay (the middle rows past a tree, not the ones that only touch it), so
    # does every row with a blade offset
    count = {}
    for gap, off in ((3.0, 0.0), (0.5, 0.0), (3.0, 0.05)):
        plan = plan_coverage(rect(0, 0, 30, 20), TREES, PlannerConfig(**dict(FORWARD, bend_max_gap=gap,
                                                                              blade_offset=off)))
        count[gap, off] = sum(1 for s in plan.segments if s.maneuver == 'bend')
    assert 0 < count[0.5, 0.0] < 0.5 * count[3.0, 0.0] and count[3.0, 0.05] == 0


# route order -----------------------------------------------------------------------------------------------


def brute_force(start, link, end):
    import itertools
    n = len(start)
    best = math.inf
    for order in itertools.permutations(range(n)):
        for ways in itertools.product(*[range(len(start[i])) for i in order]):
            c = start[order[0]][ways[0]] + end[order[-1]][ways[-1]] + sum(
                link(order[k], ways[k], order[k + 1], ways[k + 1]) for k in range(n - 1))
            best = min(best, c)
    return best


def order_cost(found, start, link, end):
    return start[found[0][0]][found[0][1]] + end[found[-1][0]][found[-1][1]] + sum(
        link(i, a, j, b) for (i, a), (j, b) in zip(found, found[1:]))


@pytest.mark.parametrize('seed', range(8))
def test_best_order_finds_the_shortest(seed):
    # items with two ends each (done one way or the other), on a plane
    rnd = random.Random(seed)
    n = 6
    ends = [((rnd.uniform(0, 10), rnd.uniform(0, 10)), (rnd.uniform(0, 10), rnd.uniform(0, 10))) for _ in range(n)]
    ways = [[e, e[::-1]] for e in ends]  # (in, out)
    dist = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1])  # noqa: E731
    start = [[dist((0, 0), w[0]) for w in ws] for ws in ways]
    end = [[dist((10, 10), w[1]) for w in ws] for ws in ways]
    link = lambda i, a, j, b: dist(ways[i][a][1], ways[j][b][0])  # noqa: E731
    found = best_order(start, link, end)
    assert sorted(i for i, _ in found) == list(range(n))
    greedy = best_order(start, link, end, budget=1)
    cost = order_cost(found, start, link, end)
    assert cost <= order_cost(greedy, start, link, end) + 1e-9
    assert cost <= 1.05 * brute_force(start, link, end) + 1e-9
    assert best_order(start, link, end) == found  # always the same


def test_best_order_small_cases():
    assert best_order([], None, []) == []
    assert best_order([[1.0, 0.5]], None, [[0.0, 2.0]]) == [(0, 0)]


def test_optimized_order_drives_less():
    many = [circle(5 + 6 * i, 5 + 5 * j, 0.6, 12) for i in range(4) for j in range(3)]
    plans = {}
    for b, obs in ((GARDEN, [TREE, BED]), (rect(0, 0, 30, 20), many)):
        for order in ('nearest', 'optimized'):
            cfg = PlannerConfig(**dict(FORWARD, route_order=order))
            plan = plan_coverage(b, obs, cfg, start=(1, 1, 0.0), end=(1, 1))
            check_invariants(plan, cfg, obs)
            assert coverage(plan, cfg) > 0.9999
            plans[len(obs), order] = plan
        assert plans[len(obs), 'optimized'].stats['transit_length'] < \
            0.9 * plans[len(obs), 'nearest'].stats['transit_length']
        # it still ends as close to the dock as the nearest-first order
        gap = {o: math.hypot(plans[len(obs), o].poses[-1].x - 1, plans[len(obs), o].poses[-1].y - 1)
               for o in ('nearest', 'optimized')}
        assert gap['optimized'] <= gap['nearest'] + 0.2
    with pytest.raises(PlanningError, match='route_order'):
        PlannerConfig(route_order='tsp').validate()


# turn types ------------------------------------------------------------------------------------------------


def test_turn_types_in_order_of_preference():
    exit_pose = Pose2D(0.0, 0.0, 0.0)
    entry = (0.0, 0.187)
    # plenty of room for both: the first one listed
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(5.0)).name == 'bulb'
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(5.0), types=('k_turn', 'bulb')).name == 'k_turn'
    # no backing up: never a K-turn, whatever the list says
    assert plan_turn(exit_pose, entry, 0.25, False, 0.05, headland(5.0), types=('k_turn', 'bulb')).name == 'bulb'
    assert plan_turn(exit_pose, entry, 0.25, False, 0.05, headland(5.0), types=('k_turn',)) is None
    # a larger radius wins over the order: one radius of headland takes a K-turn before a smaller bulb
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(0.26)).name == 'k_turn'
    m = plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(0.26), types=('bulb', 'pivot'))
    assert m.name == 'bulb' and m.radius < 0.25
    # nothing allowed fits
    assert plan_turn(exit_pose, entry, 0.25, True, 0.05, headland(0.01), types=('bulb', 'k_turn')) is None
    assert PlannerConfig(allow_reverse=False).allowed_turns == ('u_turn', 'bulb', 'detour', 'pivot')


@pytest.mark.parametrize('kw, used', [
    (dict(turn_types=('bulb', 'pivot')), 'bulb'),
    (dict(turn_types=('pivot',)), 'pivot'),
    (dict(perimeter_passes=4), 'bulb'),  # a deep headland: a bulb fits as well as a K-turn
    (dict(perimeter_passes=4, turn_types=('k_turn', 'bulb', 'u_turn', 'detour', 'pivot')), 'k_turn'),
    (dict(perimeter_passes=4, turn_types=('k_turn', 'bulb'), allow_reverse=False), 'bulb'),
])
def test_turn_types_of_a_plan(kw, used):
    cfg = PlannerConfig(**kw)
    plan = plan_coverage(rect(0, 0, 12, 8), [], cfg)
    check_invariants(plan, cfg)
    turns = {k[len('turns_'):]: v for k, v in plan.stats.items() if k.startswith('turns_') and v}
    assert turns == {used: plan.stats['lanes'] - 1}


# lane angle ------------------------------------------------------------------------------------------------


@pytest.mark.parametrize('args, expected', [
    ((0.9,), 0.9),  # no range
    ((0.3, 0.2, 0.5), 0.3),
    # bounces back at the ends, min = max holds it
    ((0.6, 0.2, 0.5), 0.4),
    ((0.9, 0.2, 0.5), 0.3),
    ((-0.2269, 0.2, 0.5), 0.3731),
    ((2.0, 1.0, 1.0), 1.0),
    # however the angle is written
    ((-350 * DEG, 0.0, 50 * DEG), 10 * DEG),
    ((-5 * DEG, 0.0, 50 * DEG), 5 * DEG),
    # lanes already in the range stay, 0 and 180 degrees are the same lanes
    ((200 * DEG, 0.0, 50 * DEG), 20 * DEG),
    ((-160 * DEG, 0.0, 50 * DEG), 20 * DEG),
    ((100 * DEG, 0.0, 50 * DEG), 0.0),
    # half a turn or more allows every direction
    ((2.0, 0.0, math.pi), 2.0),
    ((2.0, -170 * DEG, 170 * DEG), 2.0),
    # max < min is a range across +-180 degrees
    ((-175 * DEG, 170 * DEG, -170 * DEG), 185 * DEG),
    ((160 * DEG, 170 * DEG, -170 * DEG), 180 * DEG),
])
def test_angle_in_range(args, expected):
    assert angle_in_range(*args) == pytest.approx(expected, abs=1e-9)


def test_angle_in_range_like_the_mower():
    def mower(angle, lo, hi):
        # OpenMower's version (MowingBehavior.cpp)
        d = hi - lo
        if d >= math.pi:
            return angle
        width = math.fmod(d, math.pi)
        if width < 0:
            width += math.pi
        if width == 0:
            return lo
        mid = lo + width / 2
        angle = mid + math.remainder(angle - mid, math.pi)
        t = math.fmod(angle - lo, 2 * width)
        if t < 0:
            t += 2 * width
        return lo + (t if t <= width else 2 * width - t)

    rnd = random.Random(1)
    for _ in range(2000):
        lo, hi = rnd.uniform(-math.pi, math.pi), rnd.uniform(-math.pi, math.pi)
        a = rnd.uniform(-4 * math.pi, 4 * math.pi)
        assert abs(math.sin((angle_in_range(a, lo, hi) - mower(a, lo, hi)) / 2)) < 1e-9


def in_range(a, lo, hi):
    """Lanes at a lie in the range lo..hi (lanes have no front)."""
    return any(lo - 1e-9 <= a + k * math.pi <= hi + 1e-9 for k in range(-3, 4))


ANGLE_SHAPES = {
    'rectangle': rect(0, 0, 10, 6),
    'garden': GARDEN,
    'triangle': [(0, 0), (10, 0), (3, 7)],
    'L': [(0, 0), (12, 0), (12, 3), (3, 3), (3, 10), (0, 10)],
}


@pytest.mark.parametrize('name', sorted(ANGLE_SHAPES))
def test_optimal_angle(name):
    boundary = ANGLE_SHAPES[name]
    edge = plan_coverage(boundary, [], PlannerConfig())
    cfg = PlannerConfig(angle_strategy='optimal')
    plan = plan_coverage(boundary, [], cfg)
    check_invariants(plan, cfg)
    assert plan.stats['lanes'] <= edge.stats['lanes']
    assert coverage(plan, cfg) > 0.99
    # within a range of angles
    lo, hi = 20 * DEG, 40 * DEG
    ranged = plan_coverage(boundary, [], PlannerConfig(angle_strategy='optimal', angle_min=lo, angle_max=hi))
    assert in_range(ranged.angle, lo, hi)


@pytest.mark.parametrize('strategy', ['longest_edge', 'min_width', 'optimal'])
@pytest.mark.parametrize('lo, hi', [(20, 40), (100, 130), (150, -160)])
def test_angle_range(strategy, lo, hi):
    cfg = PlannerConfig(angle_strategy=strategy, angle_min=lo * DEG, angle_max=hi * DEG)
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg)
    assert in_range(plan.angle, lo * DEG, (hi + 360 if hi < lo else hi) * DEG)
    if strategy == 'longest_edge':
        # the longest edge runs at 0 degrees: 20 below 20 bounces back to 40, 180 (the same lanes) 50 past 130
        # bounces back and forth to 120, 180 lies in 150..-160
        assert plan.angle == pytest.approx({20: 40, 100: 120, 150: 0}[lo] * DEG)


# settings --------------------------------------------------------------------------------------------------


def test_with_overrides():
    base = PlannerConfig(angle=0.3)
    cfg = base.with_overrides({'mower_width': 1, 'perimeter_passes': 3, 'turn_types': 'bulb, pivot',
                               'angle': None, 'overlap': None, 'fill_pattern': 'concentric'})
    assert cfg.mower_width == 1.0 and isinstance(cfg.mower_width, float)
    assert cfg.perimeter_passes == 3 and isinstance(cfg.perimeter_passes, int)
    assert cfg.turn_types == ('bulb', 'pivot')
    assert cfg.angle is None  # None means automatic here
    assert cfg.overlap == base.overlap  # here None keeps it
    assert cfg.fill_pattern == 'concentric'
    assert base.mower_width == 0.22 and base.angle == 0.3  # a copy
    assert base.with_overrides({'turn_types': ['k_turn', 'bulb']}).turn_types == ('k_turn', 'bulb')
    other = base.with_overrides({'perimeter_offset': 0, 'angle_min': -1, 'angle_max': 1, 'allow_reverse': False})
    assert isinstance(other.perimeter_offset, float) and isinstance(other.angle_min, float)
    assert other.allow_reverse is False
    assert base.with_overrides({}) == base
    with pytest.raises(PlanningError, match='blade_speed, colour'):
        base.with_overrides({'colour': 'green', 'blade_speed': 3000, 'overlap': 0.1})
    cfg.validate()


@pytest.mark.parametrize('kw, end', [
    (dict(fill_pattern='crosshatch'), None),
    (dict(fill_pattern='concentric'), None),
    (dict(perimeter_order='last', perimeter_direction='cw'), (-0.3, 3.0)),
    (dict(angle_strategy='optimal', angle_min=0.1, angle_max=0.5), None),
    (dict(turn_types=('k_turn', 'bulb', 'pivot'), lane_order='skip'), (6.0, -0.5)),
    (dict(MOWER, **NX60I), (1.0, 1.0)),
])
def test_deterministic_new_settings(kw, end):
    cfg = PlannerConfig(**kw)
    a = plan_coverage(GARDEN, [TREE, BED], cfg, end=end)
    b = plan_coverage(list(GARDEN), [list(TREE), list(BED)], PlannerConfig(**kw), end=end)
    assert a.poses == b.poses and a.segments == b.segments and a.stats.get('body_fit') == b.stats.get('body_fit')


def test_timings():
    plan = plan_coverage(GARDEN, [TREE, BED], PlannerConfig())
    timings = plan.stats['timings']
    assert set(timings) == {'area', 'perimeter', 'remaining', 'lanes', 'route'}
    assert all(isinstance(t, float) and t >= 0.0 for t in timings.values())
    assert sum(timings.values()) <= plan.stats['planning_time'] + 1e-3


# what a review found ---------------------------------------------------------------------------------------


def uncut(plan, cfg, where=None):
    """What the blade doesn't get to of the free space (or of a part of it), with the pieces at least 1 cm
    wide that don't touch the edge (islands, the edge's own corners are up to perimeter_corner_radius)."""
    left = plan.free_space.difference(swept_area(plan, cfg.mower_width))
    if where is not None:
        left = left.intersection(where)
    edge = plan.free_space.boundary.buffer(0.02)
    return left.area, [p for p in getattr(left, 'geoms', [left])
                       if not p.buffer(-0.005).is_empty and not p.intersects(edge)]


@pytest.mark.parametrize('first_corner', [0, 1])
def test_a_pocket_at_the_start_doesnt_lose_the_lawn(first_corner):
    # a tree close to the fence cuts off a corner, the outline starts there: both parts are mowed, as
    # sections of their own
    boundary = rect(0, 0, 12, 8)
    boundary = boundary[first_corner:] + boundary[:first_corner]
    cfg = PlannerConfig()
    plan = plan_coverage(boundary, [circle(0.95, 0.95, 0.8)], cfg)
    check_invariants(plan, cfg)
    assert len(plan.section_starts) == 2 and plan.stats['sections'] == 2
    assert plan.stats['lanes'] > 30 and coverage(plan, cfg) > 0.99


@pytest.mark.parametrize('width', [0.85, 0.9, 0.95, 1.0])
def test_narrow_strips_are_mowed_in_the_middle(width):
    # wider than the loops reach, too narrow for lanes: a loop of its own in the middle
    cfg = PlannerConfig()
    for boundary in (rect(0, 0, 10, width), [(0, 0), (10, 0), (10, width), (3, width), (3, 6), (0, 6)]):
        plan = plan_coverage(boundary, [], cfg)
        check_invariants(plan, cfg)
        assert not plan.warnings
        assert uncut(plan, cfg)[1] == []


@pytest.mark.parametrize('width', [0.97, 1.0, 1.03, 1.06])
def test_a_sliver_of_lane_region_doesnt_hide_a_strip(width):
    # a strip where the lane region is a few cm wide: its pieces are too short for lanes, so they don't count
    # as mowing what's next to them
    t = Polygon(rect(0, 0, 12, 6)).union(Polygon(rect(6 - width / 2, 5.5, 6 + width / 2, 11)))
    cfg = PlannerConfig()
    plan = plan_coverage(list(t.exterior.coords)[:-1], [], cfg)
    check_invariants(plan, cfg)
    assert uncut(plan, cfg)[1] == []
    assert uncut(plan, cfg, Polygon(rect(5, 6.5, 7, 10)))[0] < 1e-4


@pytest.mark.parametrize('kw', [{}, dict(perimeter_passes=4, perimeter_corner_radius=0.4),
                                dict(perimeter_passes=3, mower_width=0.3)])
def test_rounded_loops_leave_no_tuft_between_them(kw):
    cfg = PlannerConfig(**kw)
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg)
    check_invariants(plan, cfg)
    assert uncut(plan, cfg)[1] == []


def test_skip_order_in_a_small_cell():
    route = SimpleNamespace(cfg=PlannerConfig(lane_order='skip', turn_radius=0.25))
    for n in range(3, 9):
        lanes = [Lane(i, 0.0, 5.0, 0.16 * i) for i in range(n)]
        order = [lane.row for lane in _Route.visit_order(route, lanes)]
        assert sorted(order) == list(range(n))
        # lanes 4 rows apart for a U-turn: in fewer than 8 the steps get as big as they can
        assert min(abs(a - b) for a, b in zip(order, order[1:])) == (n // 2 if n < 8 else 3)


def test_end_point_with_the_perimeter_first():
    for end in [(22.0, 8.0), (0.0, 0.0)]:
        plan = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(), end=end)
        last = plan.poses[-1]
        assert math.hypot(last.x - end[0], last.y - end[1]) < 1.5


def test_many_loops_and_settings_that_arent_planned():
    plan = plan_coverage(rect(0, 0, 10, 6), [], PlannerConfig(perimeter_passes=255))
    assert plan.stats['lanes'] == 0 and plan.stats['loops'] > 10  # loops until nothing is left
    for kw in (dict(perimeter_passes=256), dict(perimeter_passes=2.5), dict(perimeter_passes=True),
               dict(lane_overlap_passes=-1), dict(turn_types=()), dict(perimeter_passes=float('nan'))):
        with pytest.raises(PlanningError):
            plan_coverage(rect(0, 0, 5, 5), [], PlannerConfig(**kw))
    with pytest.raises(PlanningError, match='turn_types'):
        PlannerConfig().with_overrides({'turn_types': 5})
    # a lane spacing in mm, a pose every mm: refused before planning
    with pytest.raises(PlanningError, match='poses'):
        plan_coverage(rect(0, 0, 40, 30), [], PlannerConfig(mower_width=0.002, waypoint_spacing=0.001))


def test_jagged_outline_that_simplifies_into_itself():
    # a recorded outline with a point every 2 cm and a lot of noise
    rnd = random.Random(3)
    boundary = [(5 + (4 + rnd.uniform(-0.03, 0.03)) * math.cos(a), 5 + (4 + rnd.uniform(-0.03, 0.03)) * math.sin(a))
                for a in (2 * math.pi * i / 1250 for i in range(1250))]
    cfg = PlannerConfig(simplify_tolerance=0.05)
    plan = plan_coverage(boundary, [], cfg)
    check_invariants(plan, cfg)


def test_transit_across_parts_and_through_a_narrow_passage():
    # two rooms joined by a door 6 cm wide in robot space, narrower than twice the clearance
    space = Polygon(rect(0, 0, 10, 4)).difference(Polygon(rect(4.9, -1, 5.1, 1.97))).difference(
        Polygon(rect(4.9, 2.03, 5.1, 5)))
    tp = TransitPlanner(space, clearance=0.05)
    path = tp.path((4.0, 1.0), (6.0, 1.0))
    assert path is not None and LineString(path).length < 3.0  # through the door
    # where the shrunk space has a way, but a lot longer: a gap of 6 cm next to a long wall
    space = Polygon(rect(0, 0, 10, 10)).difference(Polygon(rect(4.9, 0.06, 5.1, 9)))
    tp = TransitPlanner(space, clearance=0.05)
    path = tp.path((4.0, 1.0), (6.0, 1.0))
    assert path is not None and LineString(path).length < 3.0  # not round the wall
    # a part of the space with no way to it is known at once
    two = Polygon(rect(0, 0, 4, 4)).union(Polygon(rect(6, 0, 10, 4)))
    tp = TransitPlanner(two, clearance=0.05)
    assert tp.path((1, 1), (9, 1)) is None
    assert tp.part((1, 1)) != tp.part((9, 1)) and tp.part((5, 1)) is None
    assert tp._graphs is None  # no graph needed for that


def test_transit_corners_all_at_once(monkeypatch):
    """Which corners a corner sees, worked out for all of them at once (shapely 2): the same ways as one at a time."""
    obstacles = [circle(2 + 3 * i, 2 + 3 * j, 0.5, 24) for i in range(4) for j in range(3)]
    space = Polygon(rect(0, 0, 13, 10)).difference(unary_union([Polygon(o) for o in obstacles])).buffer(-0.11, 9)
    ends = [((0.5, 0.5 + 0.9 * k), (12.5, 9.5 - 0.9 * k)) for k in range(10)]
    tp = TransitPlanner(space, clearance=0.05)
    at_once = [tp.path(a, b) for a, b in ends]
    monkeypatch.setattr(transit, 'covers', None)
    tp = TransitPlanner(space, clearance=0.05)
    assert [tp.path(a, b) for a, b in ends] == at_once and sum(len(way) > 2 for way in at_once) > 5


def test_transit_with_many_round_obstacles_is_quick():
    obstacles = [circle(2 + 3 * i, 2 + 3 * j, 0.5, 24) for i in range(8) for j in range(6)]
    space = Polygon(rect(0, 0, 25, 19)).difference(unary_union([Polygon(o) for o in obstacles])).buffer(-0.11, 9)
    tp = TransitPlanner(space, clearance=0.05)
    t0 = time.perf_counter()
    for k in range(20):
        assert tp.path((0.5, 0.5 + 0.9 * k), (24.5, 18.5 - 0.9 * k)) is not None
    assert time.perf_counter() - t0 < 5.0


@pytest.mark.parametrize('kw, width', [({}, 0.46), ({}, 0.5), ({}, 0.55), (dict(perimeter_passes=3), 0.85),
                                       (dict(perimeter_passes=3), 0.9), (dict(fill_pattern='concentric'), 0.5),
                                       (dict(fill_pattern='concentric'), 0.8)])
def test_strips_where_the_loops_stop_early_are_mowed_in_the_middle(kw, width):
    # a strip off the lawn too narrow for all the loops: a loop in its middle after the last one that fits
    t = Polygon(rect(0, 0, 6, 4)).union(Polygon(rect(2.5, 3.9, 2.5 + width, 8)))
    cfg = PlannerConfig(**kw)
    plan = plan_coverage(list(t.exterior.coords)[:-1], [], cfg)
    check_invariants(plan, cfg)
    assert uncut(plan, cfg)[1] == []


def test_end_point_with_a_narrow_strip_or_a_single_cell():
    # the strip too narrow for lanes is mowed before the lanes, they end at the end point; one cell does too
    strip = Polygon(rect(0, 0, 8, 6)).union(Polygon(rect(-4, 3, 0.5, 3.9)))
    for boundary in (list(strip.exterior.coords)[:-1], rect(0, 0, 8, 6)):
        for end in [(7.8, 0.2), (7.8, 5.8), (0.2, 0.2)]:
            for start in (None, (7.8, 5.8, math.pi)):
                last = plan_coverage(boundary, [], PlannerConfig(), start=start, end=end).poses[-1]
                assert math.hypot(last.x - end[0], last.y - end[1]) < 0.5


@pytest.mark.parametrize('kw', [dict(perimeter_order='last'), dict(fill_pattern='concentric')])
def test_end_point_in_one_of_parts_joined_by_a_path(kw):
    # two walls across the lawn, a path along the top joins the three parts. Going out the outermost loop
    # closest to the end point comes last, going in (concentric) the innermost one
    walls = [rect(4.8, -1, 5.2, 6.5), rect(9.8, -1, 10.2, 6.5)]
    cfg = PlannerConfig(**kw)
    plan = plan_coverage(rect(0, 0, 15, 6), walls, cfg, drivable=[rect(1, 5, 14, 9)], end=(0.3, 1.0))
    check_invariants(plan, cfg, walls)
    last = plan.poses[-1]
    assert last.x < 4.8
    if cfg.perimeter_order == 'last':
        assert math.hypot(last.x - 0.3, last.y - 1.0) == pytest.approx(0.3 - cfg.first_loop_offset, abs=0.02)


def test_slivers_smaller_than_the_blade_are_no_parts():
    # beds drawn 5 cm apart, the ends of the gap closed: with OpenMower's perimeter_offset 0 the gap (1 dm2)
    # would be a section of its own
    beds = [rect(2, 1, 3.05, 1.25), rect(3.1, 1, 4, 1.25), rect(3, 0.9, 3.15, 1.02), rect(3, 1.23, 3.15, 1.35)]
    cfg = PlannerConfig(perimeter_offset=0.0)
    plan = plan_coverage(rect(0, 0, 6, 4), beds, cfg)
    check_invariants(plan, cfg, beds)
    assert plan.section_starts == [0] and not plan.warnings
    assert len(getattr(plan.free_space, 'geoms', [plan.free_space])) == 1


def recorded(points, rnd, step=0.02, noise=0.01):
    """An outline as a mower records it: a point every `step`, each a little off."""
    out = []
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        k = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for j in range(k):
            out.append((x0 + (x1 - x0) * j / k + rnd.uniform(-noise, noise),
                        y0 + (y1 - y0) * j / k + rnd.uniform(-noise, noise)))
    return out


def test_optimal_angle_of_a_recorded_outline_is_quick():
    boundary = recorded(GARDEN, random.Random(1))
    edge = plan_coverage(boundary, [], PlannerConfig())
    # 90 angles, counted on the rings as they are (read once)
    plan = plan_coverage(boundary, [], PlannerConfig(angle_strategy='optimal', angle_step=2 * DEG))
    assert plan.stats['lanes'] <= edge.stats['lanes']
    assert plan.stats['planning_time'] < 8 * edge.stats['planning_time']


def test_settings_and_points_out_of_range():
    with pytest.raises(PlanningError, match='mower_width'):
        PlannerConfig().with_overrides({'mower_width': 10 ** 400})  # too big for a float
    for kw in (dict(mower_width=True), dict(angle=10 ** 400), dict(turn_radius=10.5)):
        with pytest.raises(PlanningError, match=list(kw)[0]):
            PlannerConfig(**kw).validate()
    PlannerConfig(turn_radius=10).validate()
    square = rect(0, 0, 5, 5)
    for kw in (dict(boundary=rect(1e15, 1e15, 1e15 + 10, 1e15 + 6)), dict(boundary=[(0, 0), (2e7, 0), (0, 1)]),
               dict(obstacles=[[(1, 1), (2, 1), (math.nan, 2)]]), dict(drivable=[rect(0, 0, 1e300, 5)]),
               dict(start=(math.inf, 0)), dict(end=(0, math.nan)), dict(end=(-2e7, 0))):
        with pytest.raises(PlanningError, match='map origin'):
            plan_coverage(**{'boundary': square, **kw})
    # a UTM frame is fine
    plan_coverage([(x + 5e5, y + 5.5e6) for x, y in square])


def test_optimal_angle_counts_on_the_outline_as_it_is():
    # on the outline simplified, an edge recorded along the lanes looked straight: 180 degrees won with 18
    # cells, 5 degrees has 7
    rnd = random.Random(0)
    boundary, obstacles = recorded(GARDEN, rnd), [recorded(TREE, rnd), recorded(BED, rnd)]
    cfg = PlannerConfig(angle_strategy='optimal')
    plan = plan_coverage(boundary, obstacles, cfg)
    check_invariants(plan, cfg, obstacles)

    def cost(a):
        return _lane_cost(plan.lane_region, a, cfg.lane_spacing, cfg.min_lane_length, plan.remaining,
                          plan.robot_space, 0.5 * cfg.mower_width)

    # the angles optimal tries: the longest edge's and every 5 degrees
    fallback = round(math.fmod(longest_edge_angle(plan.area), math.pi) % math.pi, 9)
    assert cost(plan.angle) == min(cost(a) for a in [fallback] + [round(i * math.pi / 36, 9) for i in range(36)])
    assert plan.stats['cells'] < 10


def test_narrow_loops_only_where_the_lanes_leave_something():
    # the lanes reach on into a strip along them: where they mow what the loops leave of a narrow strip, no
    # loop goes down its middle
    cfg = PlannerConfig(perimeter_passes=6, perimeter_corner_radius=0.5, mower_width=0.5, overlap=0.1)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg)
    check_invariants(plan, cfg, [TREE, BED])
    assert coverage(plan, cfg) > 0.998  # as much as with a loop down every narrow strip
    sweeps = [LineString([(p.x, p.y) for p in plan.poses[s.start:s.end + 1]]).buffer(0.5 * cfg.mower_width, 4)
              for s in plan.segments]
    rings = [i for i, s in enumerate(plan.segments) if s.maneuver == 'ring']
    for i in rings:
        alone = sweeps[i].difference(unary_union(sweeps[:i] + sweeps[i + 1:]))
        assert alone.intersection(plan.free_space).area > 1e-3
    # without the outline loops they're not dropped: what the lanes leave there no loop mows
    skipped = plan_coverage(GARDEN, [TREE, BED], cfg.with_overrides({'skip_outline_loops': True}))
    assert sum(s.maneuver == 'ring' for s in skipped.segments) > len(rings)


def test_middle_loops_of_many_levels_are_quick():
    # where a part ends between two loops, the loop halfway in is the outer loop's level shrunk by half a step,
    # not the area shrunk again: quicker than the levels themselves (no perimeter loops, whose strips the
    # remaining area takes off too)
    boundary = recorded([(0, 0), (12, 0), (12, 4), (5, 4), (5, 9), (0, 9)], random.Random(1))
    plan = plan_coverage(boundary, [], PlannerConfig(fill_pattern='concentric', perimeter_passes=0))
    timings = plan.stats['timings']
    assert timings['remaining'] < 0.6 * timings['perimeter']


@pytest.mark.parametrize('boundary, obstacles, kw, end', [
    (rect(0, 0, 8, 6), [], dict(perimeter_order='last', perimeter_passes=0), (7.8, 0.2)),
    (rect(0, 0, 8, 6), [], dict(perimeter_order='last', skip_outline_loops=True), (7.8, 0.2)),
    (rect(0, 0, 8, 6), [], dict(skip_lanes=True), (7.8, 0.2)),
    (GARDEN, [TREE, BED], dict(skip_lanes=True), (19.5, 3.75)),
    (GARDEN, [TREE, BED], dict(skip_lanes=True), (0.5, 15.5)),
    (rect(0, 0, 10, 0.9), [], {}, (9.9, 0.45)),
])
def test_end_point_with_whatever_comes_last(boundary, obstacles, kw, end):
    # the lanes with the loops last but none to mow, the loops without lanes (around the obstacle closest to
    # it, or the obstacles first and the innermost loop last), the loop down a lawn too narrow for lanes
    cfg = PlannerConfig(**kw)
    plan = plan_coverage(boundary, obstacles, cfg, end=end)
    check_invariants(plan, cfg, obstacles)
    last = plan.poses[-1]
    assert math.hypot(last.x - end[0], last.y - end[1]) < 0.5


def test_loop_strips_geos_cant_take_away_at_once():
    # GEOS 3.8 (shapely 1.7, ROS Noetic) fails on taking these loops' strips away from the area all at once
    boundary = [(19.868508, 12.652152), (15.045281, 14.226269), (9.227178, 17.914089), (6.886211, 15.714762),
                (3.9026, 17.253419), (3.542747, 12.94652), (3.318829, 7.659646), (2.081996, 5.661724),
                (11.277457, 1.67257), (11.797843, 1.107177), (15.756713, 3.077641), (19.89437, 8.348051)]
    tree = [(9.486004, 4.508636), (9.385437, 5.014221), (9.099046, 5.442835), (8.670431, 5.729226),
            (8.164846, 5.829793), (7.659261, 5.729226), (7.230647, 5.442835), (6.944256, 5.014221),
            (6.843689, 4.508636), (6.944256, 4.003051), (7.230647, 3.574436), (7.659261, 3.288045),
            (8.164846, 3.187478), (8.670431, 3.288045), (9.099046, 3.574436), (9.385437, 4.003051)]
    bed = [(11.720456, 15.208434), (12.227706, 14.986751), (12.380466, 14.377558), (14.634271, 13.585999),
           (13.952782, 14.298917)]
    obstacles = [rect(17.512173, 11.430021, 19.96184, 12.944767), tree, bed]
    cfg = PlannerConfig(mower_width=0.18, overlap=0.0, perimeter_passes=4)
    plan = plan_coverage(boundary, obstacles, cfg)
    check_invariants(plan, cfg, obstacles)
    assert coverage(plan, cfg) > 0.99
