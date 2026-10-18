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

from mowbite_planner import (PlannerConfig, PlanningError, Pose2D, bodyfit, config, footprint,  # noqa: E402
                             normalize_angle, plan_coverage, planner, quaternion_to_yaw, swept_area, transit,
                             yaw_to_quaternion)
from mowbite_planner.maneuvers import (CORNER_ANGLE, MAX_ARC_STEP, bulb_turn, drive_polyline, dubins,  # noqa: E402
                                       join, omega, path_length, plan_turn, same_position, sample_arc,
                                       sample_straight, u_turn)
from mowbite_planner.mathutil import angle_diff  # noqa: E402
from mowbite_planner.bodyfit import (FIELD_ARC, FIT_LATER, FIT_PLACE, FIT_REACH, FIT_ROUND, FIT_SPOTS,  # noqa: E402
                                     FIT_START, FIT_TRIES, TURN_AHEAD)
from mowbite_planner.config import (LANE, PERIMETER, SMOOTH_RADII, SPIN_LIMIT, TRANSIT, TURN, Lane,  # noqa: E402
                                    PathSegment)
from mowbite_planner.lanes import _lane_cost, angle_in_range, longest_edge_angle, scan_lanes, split_cells  # noqa: E402
from mowbite_planner.planner import plan_footprint  # noqa: E402
from mowbite_planner.route import _Route, _rows, _spins, best_order  # noqa: E402
from mowbite_planner.footprint import Footprint  # noqa: E402
from mowbite_planner.geometry import polygons  # noqa: E402
from mowbite_planner.report import check_plan, drive_time, track_mowed  # noqa: E402
from mowbite_planner.transit import TransitPlanner  # noqa: E402

DEG = math.pi / 180.0


def circle(cx, cy, r, n=24):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


GARDEN = [(0, 0), (22, 0), (24, 9), (13, 9), (13, 16), (0, 16)]
# what every plan without the mower's size warns of
NO_BODY = ['no body check: no mower size set (robot_width, robot_front and robot_rear, or robot_outline)']
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
    # every pose faces the way it's driven to the next one, never backwards
    assert not any(s.reverse for s in segs)
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        if i in jumps:
            continue
        travel = math.atan2(b.y - a.y, b.x - a.x)
        assert angle_diff(travel, a.yaw) <= arc_tol, f'pose {i} {a} faces {a.yaw}, drives {travel}'
    # straight lanes keep exactly one heading, along one of the sets of lanes (concentric rings bend)
    for s in segs:
        if s.kind == LANE and s.maneuver not in ('ring', 'bend', 'mop_up'):
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
    assert len(m.pieces) == 1
    poses = m.pieces[0].poses
    assert all(r >= 0.25 - 1e-6 for r in radii_of(poses))
    for a, b in zip(poses, poses[1:]):  # tangent continuous: never more than one arc step at once
        assert angle_diff(a.yaw, b.yaw) <= 10 * DEG + 1e-9


def headland(depth):
    """fits() of a lane end at x = 0 with `depth` of room behind it."""
    space = prep(Polygon(rect(-10, -10, depth, 10)).buffer(1e-6))
    return lambda pts: space.covers(LineString(pts) if len(pts) > 1 else Point(pts[0]))


def test_turn_choice_by_room():
    exit_pose = Pose2D(0.0, 0.0, 0.0)
    entry = (0.0, 0.187)
    # plenty of room: full radius
    assert plan_turn(exit_pose, entry, 0.25, 0.05, headland(5.0)).name == 'bulb'
    # one radius of room: a smaller bulb
    m = plan_turn(exit_pose, entry, 0.25, 0.05, headland(0.26))
    assert m.name == 'bulb' and m.radius < 0.25
    # hardly any room: half circle, then turning on the spot
    assert plan_turn(exit_pose, entry, 0.25, 0.05, headland(0.1)).name == 'u_turn'
    assert plan_turn(exit_pose, entry, 0.25, 0.05, headland(0.01)).name == 'pivot'
    # lanes far apart: a plain U-turn
    assert plan_turn(exit_pose, (0.0, 0.6), 0.25, 0.05, headland(0.3)).name == 'u_turn'


def test_turn_to_the_right_and_rotated():
    # lane heading 120 deg, next lane to its right
    h = 120 * DEG
    exit_pose = Pose2D(3.0, 1.0, h)
    entry = (3.0 + 0.187 * math.sin(h), 1.0 - 0.187 * math.cos(h))
    m = plan_turn(exit_pose, entry, 0.25, 0.05, lambda pts: True)
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
    assert plan.warnings == NO_BODY
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
    assert plan.warnings == NO_BODY
    # 2 loops along the outline, 2 around each obstacle
    assert plan.stats['perimeter_loops'] == 6
    # the lanes come after all loops
    kinds = [s.kind for s in plan.segments]
    assert max(i for i, k in enumerate(kinds) if k == 'perimeter') < kinds.index('lane')


@pytest.mark.parametrize('kw', [
    dict(),
    dict(lane_order='skip'),
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


@pytest.mark.parametrize('base, obstacles', [
    (dict(perimeter_passes=3), [TREE, BED]),
    (dict(perimeter_passes=3, narrow_parts='loops'), [TREE, BED]),
    (dict(perimeter_passes=2), [rect(1.0, 6, 3, 8), BED]),  # (an island by the edge: its loops run into the outline's)
])
def test_obstacle_passes(base, obstacles):
    """obstacle_passes: the islands get that many loops, the outline perimeter_passes as ever; -1 (the default) as many
    as the outline, the plan as without. With none the lanes go up to the islands (their turns leave a little there),
    with fewer or more nothing more stays than with as many. Not for fill_pattern concentric."""
    def plan(**kw):
        cfg = PlannerConfig(**dict(base, **kw))
        got = plan_coverage(GARDEN, obstacles, cfg, start=(1, 1, 0.0), end=(1, 1))
        check_invariants(got, cfg, obstacles)
        loops = [o['kind'] for o in got.origins.values() if str(o.get('parent', '')).startswith('level-')]
        return got, cfg, loops.count('outline'), loops.count('obstacle')

    same, cfg, outline, islands = plan()
    count = base['perimeter_passes']
    assert (outline, islands) == (count, 2 * count)
    assert plan(obstacle_passes=count)[0].poses == same.poses
    for n in (1, 2, 5):
        got, cfg, outline, islands = plan(obstacle_passes=n)
        assert (outline, islands) == (count, 2 * n)
        assert coverage(got, cfg) > 0.9995 and check_plan(got)['hard'] == []
    none, cfg, outline, islands = plan(obstacle_passes=0)
    assert (outline, islands) == (count, 0) and 0.995 < coverage(none, cfg) < coverage(same, cfg)
    assert none.stats['path_length'] < same.stats['path_length']
    rings = plan(fill_pattern='concentric')[0]
    assert plan(fill_pattern='concentric', obstacle_passes=1)[0].poses == rings.poses
    for bad in (-2, 256, 1.5, True):
        with pytest.raises(PlanningError, match='obstacle_passes'):
            PlannerConfig(obstacle_passes=bad).validate()


def lane_turns(plan):
    """(maneuver, the tightest arc's radius) of every turn between lanes."""
    out = []
    for s in plan.segments:
        if s.kind == TURN:
            radii = [r for r in radii_of(plan.poses[s.start:s.end + 1]) if r < 1e6]
            out.append((s.maneuver, min(radii, default=math.inf)))
    return out


def test_turn_radius_auto():
    """turn_radius_auto: the planner picks every turn's radius, turn_radius the least. With room the bulbs get wider (an
    open lawn, a deep headland); between lots of trees fewer turns are tighter than turn_radius (the lanes end earlier
    for a turn of it where that leaves nothing, the tight half circle only before turning on the spot), no more of them
    on the spot, as much mowed. Off: the plan as ever."""
    open_lawn = dict(perimeter_passes=6, turn_radius=0.25)
    plain = plan_coverage(rect(0, 0, 12, 8), [], PlannerConfig(**open_lawn))
    cfg = PlannerConfig(**dict(open_lawn, turn_radius_auto=True))
    wide = plan_coverage(rect(0, 0, 12, 8), [], cfg)
    check_invariants(wide, cfg)
    assert all(m == 'bulb' and r == pytest.approx(0.25, abs=0.01) for m, r in lane_turns(plain))
    assert all(m == 'bulb' and r > 0.3 for m, r in lane_turns(wide)) and coverage(wide, cfg) > 0.999
    trees = dict(MOWER, **NX60I, turn_radius=0.35)
    plain = plan_coverage(rect(0, 0, 26, 18), TREES, PlannerConfig(**trees), start=(1, 1, 0.0), end=(1, 1))
    cfg = PlannerConfig(**dict(trees, turn_radius_auto=True))
    auto = plan_coverage(rect(0, 0, 26, 18), TREES, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(auto, cfg, TREES)
    tight = [sum(1 for _, r in lane_turns(p) if r < 0.9 * 0.35) for p in (plain, auto)]
    assert tight[1] < 0.5 * tight[0] and auto.stats['turns_pivot'] <= plain.stats['turns_pivot']
    assert check_plan(auto)['body_out'] == [] and coverage(auto, cfg) > coverage(plain, cfg) - 0.001
    assert auto.stats['path_length'] < 1.05 * plain.stats['path_length']
    # a strip with a slanted end, its lanes ending at different heights: the tight half circle only fits with the next
    # lane starting a little later. That before turning on the spot, like without
    strip = dict(MOWER, **NX60I, turn_radius=0.4, perimeter_passes=2, lane_overlap_passes=0, angle=math.pi / 2)
    slanted = [(0, 0), (1.4, 0), (1.4, 4), (0, 6)]
    plain = plan_coverage(slanted, [], PlannerConfig(**strip), start=(1, 1, 0.0), end=(1, 1))
    cfg = PlannerConfig(**dict(strip, turn_radius_auto=True))
    auto = plan_coverage(slanted, [], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(auto, cfg)
    assert [p.stats['turns_pivot'] + p.stats['turns_fallback'] for p in (plain, auto)] == [0, 0]
    assert check_plan(auto)['body_out'] == [] and coverage(auto, cfg) > coverage(plain, cfg) - 0.001
    with pytest.raises(PlanningError, match='turn_radius_auto'):
        PlannerConfig(turn_radius_auto=1).validate()


def test_skip_order_turns_forwards_with_the_full_radius():
    cfg = PlannerConfig(lane_order='skip', perimeter_passes=3)
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
        assert plan.warnings == NO_BODY
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
        lane_order=rnd.choice(['snake', 'skip']),
        angle=rnd.choice([None, rnd.uniform(-3, 3)]),
    )
    try:
        plan = plan_coverage(boundary, obstacles, cfg)
    except PlanningError:
        return
    check_invariants(plan, cfg, obstacles)


@pytest.mark.parametrize('seed', range(40))
def test_random_gardens_with_obstacle_passes(seed):
    """Random gardens with islands, their loops of another number than the outline's (obstacle_passes), with and without
    the body checked, some with turn_radius_auto, some with split_zones: the plan keeps its invariants and mows about
    as much as with as many (but with none)."""
    rnd = random.Random(1000 + seed)
    boundary = random_polygon(rnd, 0, 0, 4, 9, rnd.randint(5, 14))
    obstacles = [random_polygon(rnd, rnd.uniform(-4, 4), rnd.uniform(-4, 4), 0.3, 1.2, rnd.randint(3, 8))
                 for _ in range(rnd.randint(1, 4))]
    base = dict(MOWER, **NX60I) if seed % 4 == 0 else dict(
        mower_width=rnd.choice([0.18, 0.22]), turn_radius=rnd.choice([0.2, 0.3]), lane_order=rnd.choice(['snake', 'skip']))
    base.update(perimeter_passes=rnd.choice([1, 2, 3]), narrow_parts=rnd.choice(['lanes', 'loops']))
    count = rnd.choice([0, 1, 2, 4])
    try:
        same = plan_coverage(boundary, obstacles, PlannerConfig(**base))
        cfg = PlannerConfig(**dict(base, obstacle_passes=count, turn_radius_auto=seed % 3 == 0,
                                   split_zones=seed % 2 == 1))
        plan = plan_coverage(boundary, obstacles, cfg)
    except PlanningError:
        return
    check_invariants(plan, cfg, obstacles)
    assert check_plan(plan)['body_out'] == []
    if count:
        assert coverage(plan, cfg) > coverage(same, cfg) - 0.01


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
    assert plan.warnings == NO_BODY
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
    assert plan.warnings == NO_BODY


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
    assert coverage(plan, cfg) > 0.99 and plan.warnings == NO_BODY


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
    # every loop the same way round, around the obstacles too (and concentric's rings)
    loops: Dict[str, List[int]] = {}
    for s in plan.segments:
        if s.kind == PERIMETER or s.kind == LANE and s.maneuver == 'ring':
            span = loops.setdefault(s.source, [s.start, s.end])
            span[0], span[1] = min(span[0], s.start), max(span[1], s.end)
    assert len(loops) >= 6
    sign = 1 if direction == 'ccw' else -1
    for start, end in loops.values():
        assert sign * signed_area(plan.poses[start:end + 1]) > 0


def test_perimeter_direction_random():
    """random: the plan of ccw or cw (round the obstacles too), the same for the same count of finished_mows, another
    at random for another; over a few mows both. The count is a whole number."""
    ways = {d: plan_coverage(GARDEN, [TREE, BED], PlannerConfig(perimeter_direction=d)).poses for d in ('ccw', 'cw')}
    got = []
    for mows in range(4):
        cfg = PlannerConfig(perimeter_direction='random', finished_mows=mows)
        poses = plan_coverage(GARDEN, [TREE, BED], cfg).poses
        got.append(next(d for d, them in ways.items() if them == poses))
        if not mows:
            assert plan_coverage(GARDEN, [TREE, BED], cfg).poses == poses
    assert set(got) == {'ccw', 'cw'}
    for kw in (dict(finished_mows=-1), dict(finished_mows=0.5)):
        with pytest.raises(PlanningError, match='whole number'):
            PlannerConfig(**kw).validate()


@pytest.mark.parametrize('recorded', [1, -1], ids=['ccw', 'cw'])
def test_outermost_loop_the_way_recorded(recorded):
    """With the body's outline (perimeter_direction auto or random) the outermost loop along the outline goes the way
    the outline was recorded (the way its points go: the body only where it was then), the loops further in as ever
    (random: at random). ccw and cw: every loop that way; without the outline (robot_width...) every loop as ever: the
    mower facing east by the southern edge, ccw."""
    def ways(plan):  # the way round of the loops along the outline (1 ccw, -1 cw), the outermost first
        got = []
        loops = [(o['level'], name) for name, o in plan.origins.items() if o['kind'] == 'outline']
        for _, name in sorted(loops):
            pts = [p for s in plan.segments if s.source == name and s.kind == PERIMETER
                   for p in plan.poses[s.start:s.end + 1]]
            if pts:
                got.append(1 if signed_area(pts) > 0 else -1)
        return got

    lawn, ends = rect(0, 0, 10, 6)[::recorded], dict(start=(1, 1, 0.0), end=(1, 1))
    cfg = PlannerConfig(**dict(MOWER, robot_outline=NX60, angle=0.0))
    plan = plan_coverage(lawn, [], cfg, **ends)
    check_invariants(plan, cfg)
    assert ways(plan)[0] == recorded and len(ways(plan)) == 4
    for mows, way in ((0, 1), (1, -1)):  # (random: 0 ccw, 1 cw)
        got = ways(plan_coverage(lawn, [], dataclasses.replace(cfg, perimeter_direction='random', finished_mows=mows),
                                 **ends))
        assert got[0] == recorded and set(got[1:]) == {way}
    assert set(ways(plan_coverage(lawn, [], dataclasses.replace(cfg, perimeter_direction='cw'), **ends))) == {-1}
    rectangle = dataclasses.replace(cfg, robot_outline=None, **NX60I)
    assert set(ways(plan_coverage(lawn, [], rectangle, **ends))) == {1}


# body and blade --------------------------------------------------------------------------------------------

FORWARD = dict(lane_order='skip', mower_width=0.18, overlap=1 - 0.13 / 0.18, perimeter_offset=0.0)


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
    # (but where a lane ends a little earlier for a U-turn of the turn radius, round_earlier)
    starts = [plain.poses[t.start] for t in plain.segments if t.kind == LANE and not t.maneuver]
    lanes = [plan.poses[s.start] for s in plan.segments if s.kind == LANE and not s.maneuver]
    same = [p for p in lanes if any(math.hypot(p.x + ahead * math.cos(p.yaw) - q.x,
                                               p.y + ahead * math.sin(p.yaw) - q.y) < 1e-6 for q in starts)]
    assert len(same) >= 0.9 * len(lanes)
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


# the body (robot_width, robot_front, robot_rear or robot_outline) ------------------------------------------

NX60I = dict(robot_width=0.40, robot_front=0.43, robot_rear=0.14)
BODY = PlannerConfig(**NX60I).body_outline  # (its corners)
MOWER = dict(FORWARD, mower_width=0.18, overlap=1 - 0.13 / 0.18, perimeter_offset=0.0, blade_ahead=0.17,
             perimeter_passes=4, lane_overlap_passes=1)


def rounded(width, front, rear, r=0.17):
    """The corners of the rectangle with its front corners rounded: arcs of r, a point every 10 degrees."""
    arc = [(front - r + r * math.cos(math.radians(a)), 0.5 * width - r + r * math.sin(math.radians(a)))
           for a in range(0, 91, 10)]
    return [(x, -y) for x, y in arc[::-1]] + arc + [(-rear, 0.5 * width), (-rear, -0.5 * width)]


# outlines of the body (robot_outline): BODY with its front corners rounded, two measured from photos (a YardForce NX60
# scaled to 0.41 x 0.43 / 0.18, a Classic 500B, +-1 cm), one with wheels standing out at the front and the back (lines
# along the heading cross it twice there), a round one (no part further from the centre than the circles round its
# halves)
ROUNDED = rounded(0.40, 0.43, 0.14)
NX60 = [(0.43, 0.047), (0.428, 0.057), (0.398, 0.134), (0.389, 0.148), (0.371, 0.165), (0.203, 0.191), (0.021, 0.205),
        (-0.079, 0.204), (-0.114, 0.194), (-0.137, 0.173), (-0.171, 0.097), (-0.18, 0.055), (-0.18, -0.055),
        (-0.171, -0.097), (-0.137, -0.173), (-0.114, -0.194), (-0.079, -0.204), (0.021, -0.205), (0.203, -0.191),
        (0.371, -0.165), (0.389, -0.148), (0.398, -0.134), (0.428, -0.057), (0.43, -0.047)]
MEASURED = [(0.47, 0.058), (0.455, 0.12), (0.43, 0.167), (0.415, 0.18), (0.363, 0.19), (0.132, 0.212), (0.066, 0.212),
            (-0.043, 0.194), (-0.075, 0.182), (-0.1, 0.07), (-0.1, -0.07), (-0.075, -0.182), (-0.043, -0.194),
            (0.066, -0.212), (0.132, -0.212), (0.363, -0.19), (0.415, -0.18), (0.43, -0.167), (0.455, -0.12),
            (0.47, -0.058)]
WHEELS = [(0.4, -0.18), (0.4, 0.18), (0.34, 0.18), (0.34, 0.22), (0.26, 0.22), (0.26, 0.18), (0.02, 0.18), (0.02, 0.22),
          (-0.12, 0.22), (-0.12, 0.18), (-0.15, 0.18), (-0.15, -0.18), (-0.12, -0.18), (-0.12, -0.22), (0.02, -0.22),
          (0.02, -0.18), (0.26, -0.18), (0.26, -0.22), (0.34, -0.22), (0.34, -0.18)]
ROUND = circle(0, 0, 0.25)


def test_robot_width_alone_changes_nothing_on_recorded_edges():
    plain = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**FORWARD))
    width = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(FORWARD, robot_width=0.4)))
    assert width.poses == plain.poses and width.body_space is None
    assert check_plan(width)['body_out'] == []


def test_footprint():
    fp = Footprint(Polygon(rect(0, 0, 10, 10)), BODY)
    assert fp.fits_pose(5, 5, 0.0) and fp.fits_pose(9.5, 5, 0.0) and not fp.fits_pose(9.6, 5, 0.0)
    assert fp.fits_pose(0.2, 5, 0.0) and not fp.fits_pose(0.1, 5, 0.0)  # the rear
    assert fp.fits_turn(0.5, 0.5, 0.0, math.pi) and not fp.fits_turn(0.2, 5, 0.0, math.pi / 2)
    # turning on the spot where the heading jumps from the pose before: half way round the rear corner sticks out
    poses = [Pose2D(9.4, 0.22, 0.0), Pose2D(9.5, 0.22, math.pi / 2), Pose2D(9.5, 0.32, math.pi / 2)]
    assert fp.bad_poses(poses) == [1] and fp.spins(poses, 1) and not fp.poses_fit(poses)
    # about a half turn either way round (the front swings over the edge turning right), less the shorter way only
    assert not fp.fits_turn(5, 0.45, 0.0, math.radians(175)) and fp.fits_turn(5, 0.45, 0.0, math.radians(160))
    # where the heading turns by less than SPIN only the poses count, at a kink of more it turns there like on the spot
    # (OpenMower's FTC does): between the two headings the front's corner gets further out than at either
    assert footprint.SPIN == math.radians(15.0)
    poses = [Pose2D(9.47, 5, 0.0), Pose2D(9.48, 5.01, math.radians(14))]
    assert fp.poses_fit(poses) and fp.poses_fit([poses[0], poses[1]._replace(yaw=math.radians(28))])
    kink = [Pose2D(9.532, 5, math.radians(10)), Pose2D(9.532, 5.01, math.radians(38))]
    assert fp.fit(kink) == [True, True] and fp.bad_poses(kink) == [1] and fp.spins(kink, 1)


def test_footprint_spin_margin(monkeypatch):
    """With a spin_margin the body keeps that much further off the edges turning on the spot, anywhere on the last
    SPIN_BACK of the way in (where OpenMower's FTC turns): a quarter turn left coming in facing east, the rear corner
    gets 0.244 m behind the point halfway round, the way in leads off the edge behind it. Without one, at the point
    only. Poses that don't turn keep the margins as before. Where it turns whatever way it faces lies that much further
    in. With shapely 1.7 the same, a turn at a time."""
    room = Polygon(rect(0, 0, 10, 10))
    plain, spin = Footprint(room, BODY), Footprint(room, BODY, 0.05)
    rear = math.hypot(0.14, 0.2)
    for off, fits in ((0.03, (True, False)), (0.12, (True, False)), (0.16, (True, True))):
        assert (plain.fits_turn(rear + off, 5, 0.0, math.pi / 2), spin.fits_turn(rear + off, 5, 0.0, math.pi / 2)) \
            == fits, off
    assert footprint.SPIN_BACK == 0.1 and plain.turning is plain and spin.turning.space.equals(room.buffer(-0.05))
    poses = [Pose2D(0.15, 5, 0.0), Pose2D(0.25, 5, 0.0), Pose2D(0.35, 5.03, 0.3)]
    assert spin.fit(poses) == plain.fit(poses) == [True] * 3 and spin.poses_fit(poses)
    poses = [Pose2D(rear + 0.02, 5, 0.0), Pose2D(rear + 0.12, 5, math.pi / 2), Pose2D(rear + 0.12, 5.1, math.pi / 2)]
    assert plain.poses_fit(poses) and spin.bad_poses(poses) == [1]
    assert plain.anywhere.contains(Point(0.6, 5)) and not spin.turning.anywhere.contains(Point(0.6, 5))
    assert spin.turning.anywhere.contains(Point(0.7, 5))
    rng = random.Random(3)
    turns = [(rng.uniform(0.2, 1.0), rng.uniform(0.2, 1.0), rng.uniform(-3, 3), rng.uniform(-3, 3)) for _ in range(300)]
    fits = [spin.fits_turn(*t) for t in turns]
    monkeypatch.setattr(footprint, 'contains_xy', None)
    assert [Footprint(room, BODY, 0.05).fits_turn(*t) for t in turns] == fits and 30 < sum(fits) < 270


def test_footprint_on_turn(monkeypatch):
    """On a turn between lanes (on_turn: all poses but the first, where it comes from, none, or one by one) the body
    keeps spin_margin further off the edges, facing the pose's way: there the mower keeps less exactly to its path.
    Not by default, turning on the spot as before; out_by says how far out of that space. Without a spin_margin the
    margins as everywhere. With shapely 1.7 the same."""
    room = Polygon(rect(0, 0, 10, 10))
    fp = Footprint(room, BODY, 0.05)
    # 3 cm off the bottom edge (0.2 half the body), 4 cm, turning on the spot by 20 degrees there
    poses = [Pose2D(5.0, 0.23, 0.0), Pose2D(5.1, 0.23, 0.0), Pose2D(5.2, 0.24, 0.0),
             Pose2D(5.2, 0.24, math.radians(20))]
    way = poses[:3]
    assert fp.fit(way) == [True] * 3 and fp.fit(way, True) == [True, False, False]
    assert fp.fit(way, [True, False, False]) == [False, True, True] and fp.fit(way, [False] * 3) == [True] * 3
    assert fp.poses_fit(way) and fp.bad_poses(way, True) == [1, 2] and not fp.poses_fit(way, True)
    assert fp.bad_poses(poses) == [3] and fp.bad_poses(poses, True) == [1, 2, 3]
    assert fp.poses_fit([p._replace(y=p.y + 0.03) for p in way], True)  # (6 cm off it)
    out = fp.out_by([(None, way[0]), (way[0], way[1]), (way[1], way[2])], on_turn=[False, True, True])
    assert [d for d, _ in out] == pytest.approx([0.0, 0.02, 0.01], abs=1e-3) and out[1][1][1] < 0.05
    assert fp.out_by([(way[0], way[1])])[0] == (0.0, None)
    plain = Footprint(room, BODY)
    assert plain.fit(way, True) == [True] * 3 and plain.poses_fit(way, True)
    monkeypatch.setattr(footprint, 'contains_xy', None)
    old = Footprint(room, BODY, 0.05)
    assert old.fit(way, True) == [True, False, False] and old.bad_poses(poses, True) == [1, 2, 3]


def test_footprint_out_by(monkeypatch):
    """out_by: how far the body's farthest point lies out of the space and where. The front 3 cm over the edge: 3 cm,
    on the front; the tip of a corner of 90 degrees 3 cm into its side: 2.1 cm, on the side, none of its corners out; a
    post all under it: the post's middle; turning on the spot near an edge: out of the space spin_margin smaller,
    anywhere on the last SPIN_BACK of the way in (the rear corner swinging out there); in it: 0, nowhere. With shapely
    1.7 the same, without numpy no more than that, at least what its corners and the middles give."""
    tip, post = [(5, 5.17), (5.3, 5.47), (5, 5.77), (4.7, 5.47)], rect(7, 4.99, 7.04, 5.03)
    space = Polygon(rect(0, 0, 10, 10)).difference(Polygon(tip)).difference(Polygon(post))
    turns = [(None, Pose2D(9.6, 3, 0.0)), (None, Pose2D(5, 5, 0.0)), (None, Pose2D(6.9, 5.01, 0.0)),
             (None, Pose2D(3, 3, 1.0)), (Pose2D(0.2, 3, 0.0), Pose2D(0.3, 3, math.pi / 2))]
    rear = math.hypot(0.14, 0.2)
    want = [0.03, 0.03 * math.sqrt(0.5), 0.02, 0.0, 0.05 + rear - 0.2]
    got = Footprint(space, BODY, 0.05).out_by(turns)
    assert [d for d, _ in got] == pytest.approx(want, abs=1.5e-3)
    assert got[0][1][0] == pytest.approx(10.03) and got[1][1] == pytest.approx((5, 5.2), abs=3e-3)
    assert got[2][1] == pytest.approx((7.02, 5.01), abs=2e-3) and got[3][1] is None
    assert got[4][1][0] == pytest.approx(0.2 - rear, abs=0.01)
    # (the edge every step, the corners exact: in steps of 2 cm the front still)
    assert Footprint(space, BODY, 0.05).out_by(turns[:1], 0.02)[0][0] == 0.03
    monkeypatch.setattr(footprint, 'contains_xy', None)
    monkeypatch.setattr(footprint, 'contains_points', lambda prepared, x, y: footprint.numpy.array(
        [prepared.context.contains(Point(p)) for p in zip(x, y)], bool))
    old = Footprint(space, BODY, 0.05).out_by(turns)
    assert [d for d, _ in old] == pytest.approx([d for d, _ in got], abs=1e-3)
    assert [at for _, at in old if at is not None] == pytest.approx([at for _, at in got if at is not None], abs=1e-3)
    monkeypatch.setattr(footprint, 'contains_points', None)
    plain = [d for d, _ in Footprint(space, BODY, 0.05).out_by(turns)]
    assert plain[0] == want[0] and 0.01 < plain[1] < got[1][0] and plain[2:] == pytest.approx(want[2:], abs=1.5e-3)


@pytest.mark.parametrize('corners', [BODY, ROUNDED, WHEELS, ROUND], ids=['rectangle', 'rounded', 'wheels', 'round'])
def test_footprint_fit(monkeypatch, corners):
    """fit() is fits_pose for every pose: all at once with shapely 2, one at a time with shapely 1.7 (their points at
    once with numpy). Whether the body is in the room, also just off a corner of what's taken out of it, the front's
    corner facing it, or the side of a half facing it; whatever its outline."""
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
        fp = Footprint(room, corners)
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
    # (left: the corners, round them further in, turning there spin_margin further off the edges)
    assert report['body_out'] == [] and report['left_m2'] < 0.6
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
        # left: the corners the body keeps the mower out of (rounded further in, turning there spin_margin further
        # off the edges), and the lane starts of the turns (all along them spin_margin further off them too). The outer
        # loop starting away from the corner by the dock, a way in to it from where the mower turns: 0.15 m2 of that
        assert report['left_m2'] < 2.15


def test_lane_overlap_limit():
    """lane_overlap_limit: the turns between the lanes go no further out than the lanes reach, on the spot where no
    rounded one fits in there, nothing more left unmowed for it. Without it they take the headland's room. Where the
    rounded ones keep within the lanes' reach anyway (the body checked, the lanes reaching two loops out), the path
    stays as it is. Without the body checked the lanes start blade_ahead further out for the blade's first cut: with
    the limit that much later where the loops mowed, the mower a good deal further off the edge on the lanes."""
    area = rect(0, 0, 10, 6)
    edge = Polygon(area).boundary

    def planned(body=NX60I, **more):
        cfg = PlannerConfig(**dict(MOWER, **body, angle=0.0, perimeter_passes=3, **more))
        plan = plan_coverage(area, [], cfg, start=(1, 1, 0.0), end=(1, 1))
        check_invariants(plan, cfg)

        def past(p):  # how far past where the lanes lie
            return plan.lane_region.distance(Point(p.x, p.y))

        # of every turn, of every lane's start, and how close to the edge the lanes and turns get
        return (plan, [max(past(p) for p in plan.poses[s.start:s.end + 1]) for s in plan.segments if s.kind == TURN],
                [past(plan.poses[s.start]) for s in plan.segments if s.kind == LANE],
                min(edge.distance(Point(p.x, p.y)) for s in plan.segments if s.kind in (LANE, TURN)
                    for p in plan.poses[s.start:s.end + 1]))

    reach = config.TURN_REACH + 1e-6
    free, turns, _, _ = planned(lane_overlap_passes=1)
    assert free.stats['turns_u_turn'] == len(turns) > 30 and min(turns) > reach
    kept, turns, _, _ = planned(lane_overlap_passes=1, lane_overlap_limit=True)
    assert len(turns) > 30 and max(turns) <= reach
    # (one or two rounded ones fit in there, depending on where the loops start, see route.START_CLEAR)
    assert kept.stats['turns_pivot'] >= len(turns) - 2 and kept.stats['turns_fallback'] == 0
    assert check_plan(kept)['left_m2'] <= check_plan(free)['left_m2'] + 1e-3
    free, _, _, _ = planned(lane_overlap_passes=2)
    kept, turns, _, _ = planned(lane_overlap_passes=2, lane_overlap_limit=True)
    assert max(turns) <= reach and kept.stats['turns_u_turn'] == len(turns)
    assert [(p.x, p.y, p.yaw) for p in kept.poses] == [(p.x, p.y, p.yaw) for p in free.poses]

    free, turns, starts, close = planned({}, lane_overlap_passes=2)
    assert min(turns) > reach and close < 0.05
    kept, turns, starts, closer = planned({}, lane_overlap_passes=2, lane_overlap_limit=True)
    assert len(turns) > 30 and max(turns) <= reach and max(starts) <= reach and closer > close + 0.08
    assert kept.stats['turns_u_turn'] > 2 * kept.stats['turns_pivot'] > 0 and kept.stats['turns_fallback'] == 0
    assert check_plan(kept)['left_m2'] <= check_plan(free)['left_m2'] + 1e-3
    # (one loop out the loops haven't mowed every lane's start: those start where they did, the mower still further in)
    _, _, starts, close = planned({}, lane_overlap_passes=1)
    kept, _, later, closer = planned({}, lane_overlap_passes=1, lane_overlap_limit=True)
    assert sum(d > reach for d in later) < sum(d > reach for d in starts) / 2 and closer > close + 0.05
    assert check_plan(kept)['left_m2'] <= 0.01
    with pytest.raises(PlanningError, match='lane_overlap_limit'):
        PlannerConfig(lane_overlap_limit=1).validate()


def test_loop_further_in_round_a_bend(monkeypatch):
    """Round a bend of the edge the body's front swings out on the first loop: that stretch runs a little further in
    (_Route.loops_in) and the blade still mows along the edge, where the loop was left out all along the bend."""
    r = 1.5
    bend = [(4 + r * math.cos(a), r + r * math.sin(a)) for a in (math.radians(d) for d in range(-90, 91, 6))]
    lawn = [(0, 0)] + bend + [(0, 2 * r)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    plan = plan_coverage(lawn, [], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg)
    assert check_plan(plan)['body_out'] == []
    # the strip along the bend, but for where it begins and ends
    disc = Polygon(bend)
    strip = disc.difference(disc.buffer(-0.15)).intersection(Polygon(rect(4.0, 0.3 * r, 6.0, 1.7 * r)))

    def left(p):
        return strip.difference(swept_area(p, cfg.mower_width)).area

    monkeypatch.setattr(_Route, 'loops_in', lambda self, places: {})
    before = plan_coverage(lawn, [], cfg, start=(1, 1, 0.0), end=(1, 1))
    assert left(before) > 0.15 and left(plan) < 0.08
    assert plan.stats['body_fit']['skipped_m'] < before.stats['body_fit']['skipped_m'] - 4.0
    # (the first loop only, the others are as they were)
    loops, were = plan.perimeter_loops, before.perimeter_loops
    assert loops[0] != were[0] and loops[1:] == were[1:]


def test_body_bent_lanes():
    """Lanes bending around trees get their ends pulled back too: U-turns, nothing replaced by a drive."""
    trees = [circle(4, 4, 0.4), circle(9, 10, 0.5)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, bend_max_gap=3.0))
    plan = plan_coverage(GARDEN, [TREE, BED] + trees, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED] + trees)
    assert any(s.maneuver == 'bend' for s in plan.segments)
    # (one turn radius: where no arc of it fits between the bends a drive)
    assert plan.stats['turns_fallback'] + plan.stats['turns_pivot'] <= 4
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


def test_robot_outline_settings():
    """robot_outline: [[x, y], ...] (JSON) as tuples, the body checked without robot_width, robot_front and robot_rear
    (it beats them), counter-clockwise whichever way round it's given, a closing point once; None: the rectangle again.
    The loops perimeter_passes -1 works out: one less where the rounded front reaches less far out turning."""
    square = [[0.4, 0.2], [-0.1, 0.2], [-0.1, -0.2], [0.4, -0.2]]
    cfg = PlannerConfig().with_overrides({'robot_outline': square})
    cfg.validate()
    assert cfg.robot_outline == tuple(map(tuple, square)) and cfg.body and cfg.robot_width is None
    assert cfg.body_outline == cfg.robot_outline and cfg.with_overrides({'robot_outline': None}).robot_outline is None
    cw = cfg.with_overrides({'robot_outline': [[0.4, -0.2], [-0.1, -0.2], [-0.1, 0.2], [0.4, 0.2], [0.4, -0.2]]})
    assert cw.body_outline == ((0.4, -0.2), (0.4, 0.2), (-0.1, 0.2), (-0.1, -0.2))
    diamond = PlannerConfig().with_overrides({'robot_outline': [[1, 0], [0, 1], [-1, 0], [0, -1]]})
    diamond.validate()
    assert diamond.body_outline == ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
    assert PlannerConfig(**NX60I, robot_outline=ROUNDED).body_outline == tuple(ROUNDED)
    auto = dict(MOWER, perimeter_passes=-1)
    assert PlannerConfig(**auto, robot_outline=BODY).passes == PlannerConfig(**dict(auto, **NX60I)).passes == 5
    assert PlannerConfig(**auto, robot_outline=ROUNDED).passes == 4


@pytest.mark.parametrize('outline', [
    [[0.4, 0.2], [-0.1, 0.2]], [[0.4, 0.2], [-0.1, -0.2], [-0.1, 0.2], [0.4, -0.2]],
    [[1.0, 0.2], [0.5, 0.2], [0.5, -0.2], [1.0, -0.2]], [[0.4, 0.2], [-0.1, 'a'], [-0.1, -0.2]],
    [[0.4, 0.2], [-0.1, True], [-0.1, -0.2]], [[0.4, 0.2], [-0.1, math.nan], [-0.1, -0.2]],
    [[0.4, 0.2], [-0.1, math.inf], [-0.1, -0.2]], [[0.4, 0.2, 0.0], [-0.1, 0.2, 0.0], [-0.1, -0.2, 0.0]],
    [None, None, None], [[0.4, 0.2], 5, [-0.1, -0.2]], [], 0.4, 'square',
])
def test_robot_outline_refused(outline):
    """Fewer than 3 points, no numbers, crossing itself, base_link not in it."""
    with pytest.raises(PlanningError, match='robot_outline must be'):
        PlannerConfig().with_overrides({'robot_outline': outline}).validate()


def forget_bodies():
    """No body spaces nor _fit_body's checkpoints kept from plans before."""
    config._FITTED.clear()
    config._BODY_SPACES.clear()
    config._BODY_SPACE.__dict__.pop('last', None)


def test_robot_outline_of_the_rectangle_plans_the_same():
    """robot_outline the corners of the rectangle of robot_width, robot_front and robot_rear: the same plans to the
    last bit (poses, segments, stats, warnings, body space), at other angles, with headland_turns, concentric with the
    loops worked out, in a dead end; given on its own or with them (as JSON gives it)."""
    shape = [(0, 0), (6, 0), (6, 3), (3.5, 3), (3.1, 9), (2.9, 9), (2.5, 3), (0, 3)]
    garden = dict(start=(1, 1, 0.0), end=(1, 1))
    cases = [(GARDEN, [TREE, BED], PlannerConfig(**dict(MOWER, **NX60I, angle=math.radians(33))), garden),
             (GARDEN, [TREE, BED], PlannerConfig(**dict(MOWER, robot_width=0.41, robot_front=0.43, robot_rear=0.18,
                                                        angle=0.4, headland_turns=True, perimeter_passes=-1)), garden),
             (SHADE_LAWN, [SHADE_TABLE, SHADE], PlannerConfig(**dict(SHADE_MOWER, edge_margin=0.04,
                                                                     obstacle_margin=0.04)),
              dict(start=(0.54, 9.5, 0.0), end=(0.54, 9.5))),
             (shape, [], PlannerConfig(**dict(MOWER, **NX60I)), dict(start=(1, 1, 0.0)))]
    for i, (lawn, obstacles, cfg, ends) in enumerate(cases):
        corners = [list(p) for p in cfg.body_outline]
        outline = cfg.with_overrides({'robot_outline': corners}) if i % 2 else \
            dataclasses.replace(cfg, robot_width=None, robot_front=None, robot_rear=None, robot_outline=corners)
        plans = []
        for c in (cfg, outline):
            forget_bodies()
            plans.append(plan_coverage(lawn, obstacles, c, **ends))
        a, b = ({k: v for k, v in p.stats.items() if k not in ('planning_time', 'timings')} for p in plans)
        assert plans[0].poses == plans[1].poses and plans[0].segments == plans[1].segments and a == b
        assert plans[0].warnings == plans[1].warnings and plans[0].body_space.wkb == plans[1].body_space.wkb
        assert plans[1].config.robot_outline is not None and a['body_fit']['fixed'] > 0


def test_robot_outline_rounded_front():
    """The front corners rounded: turning on the spot the body reaches 4 cm less far from the centre, a turn by a wall
    the rectangle's front corner swings over fits. The real edges are where the rounded body was while they were
    recorded, less than the rectangle's. In a dead end too narrow to turn round in at its end the path drives a metre
    further in before it turns round, leaving less unmowed, nothing sticking out (also against the real edges worked
    out here on their own with the rounded body)."""
    room = Polygon(rect(0, 0, 10, 10))
    box, front = Footprint(room, BODY), Footprint(room, ROUNDED)
    assert box.reach == math.hypot(0.43, 0.2) and front.reach < box.reach - 0.04
    assert (box.front, box.rear, box.half) == (0.43, 0.14, 0.2)
    assert (front.front, front.rear, front.half) == pytest.approx((0.43, 0.14, 0.2), abs=1e-12)
    assert front.fits_turn(9.55, 5, 0.0, math.pi / 2) and not box.fits_turn(9.55, 5, 0.0, math.pi / 2)
    shape = [(0, 0), (6, 0), (6, 3), (3.5, 3), (3.1, 9), (2.9, 9), (2.5, 3), (0, 3)]
    plans = [plan_coverage(shape, [], PlannerConfig(**dict(MOWER, **body)), start=(1, 1, 0.0))
             for body in (NX60I, dict(robot_outline=ROUNDED))]
    assert plans[0].body_space.buffer(1e-6).covers(plans[1].body_space)
    assert plans[1].body_space.area < plans[0].body_space.area - 0.05
    plan, cfg = plans[1], plans[1].config
    check_invariants(plan, cfg)
    assert plan.stats['body_fit']['jumps'] == [] and check_plan(plan)['body_out'] == []
    assert outside_room(plan, recorded_room(cfg, [shape], [], []), spin=cfg.spin_margin) == []
    # (the rectangle's plan, starting away from the corner by the dock too, gets further in than it did before that)
    assert max(p.y for p in plan.poses) > max(p.y for p in plans[0].poses) + 0.5
    assert check_plan(plan)['left_m2'] < check_plan(plans[0])['left_m2'] - 0.5


@pytest.mark.parametrize('corners, pieces', [(BODY, 1), (ROUNDED, 1), (NX60, 1), (MEASURED, 1), (WHEELS, 5)],
                         ids=['rectangle', 'rounded', 'nx60', 'measured', 'wheels'])
def test_driven_sweeps_the_outline(corners, pieces):
    """_driven: along an edge the body sweeps from where it was at the edge's start to where it was at its end, whatever
    its outline (the wheels' cut into pieces every line along the heading crosses once): what the body every mm along
    the edges covers (and turning at the points like there), a hair more between them (where a corner of it is the
    leftmost or rightmost)."""
    corners = PlannerConfig(robot_outline=corners).body_outline
    ring = [(0.0, 0.0), (1.0, 0.0), (1.4, 0.7), (0.2, 0.5)]
    swept = planner._driven([ring], corners)
    bodies = []
    for i, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
        h, n = math.atan2(b[1] - a[1], b[0] - a[0]), int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.001))
        turn = normalize_angle(math.atan2(ring[(i + 2) % 4][1] - b[1], ring[(i + 2) % 4][0] - b[0]) - h)
        m = int(math.ceil(abs(turn) / footprint.TURN_STEP))
        for x, y, yaw in [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, h) for k in range(n + 1)] + \
                [(b[0], b[1], h + turn * k / m) for k in range(1, m)]:
            c, s = math.cos(yaw), math.sin(yaw)
            bodies.append(Polygon([(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in corners]))
    sampled = unary_union(bodies)
    assert swept.is_valid and sampled.difference(swept).area < 1e-9 and swept.difference(sampled).area < 1e-4
    assert len(_rows(corners)) == pieces


def sticks_out(room, shape):
    """m the farthest point of shape lies out of room (0.5 mm short of it at most), worked out here on its own: how far
    the room round it has to grow to cover it."""
    x0, y0, x1, y1 = shape.bounds
    near, lo, hi = room.intersection(Polygon(rect(x0 - 1, y0 - 1, x1 + 1, y1 + 1))), 0.0, 1.0
    while hi - lo > 5e-4:
        lo, hi = (lo, 0.5 * (lo + hi)) if near.buffer(0.5 * (lo + hi), 32).covers(shape) else (0.5 * (lo + hi), hi)
    return lo


def check_places(plan, off):
    """Every place of stats['body_fit'] at the pose of the path as planned (off: without body_fit, the poses before
    turning on the spot, TURN_AHEAD, there too) its 'poses' say: where the body sticks out of body_space, or turning on
    the spot there (spin 1, the heading jumping from the pose before by more than SPIN) of it spin_margin smaller,
    anywhere on the last SPIN_BACK of the way in (its hull every 2 degrees, both ways round about a half turn), as far
    as they say (worked out here; of a pose the path gets to twice one of those, 0: less than half a mm)."""
    fit, fp, cfg = plan.stats['body_fit'], plan_footprint(plan), plan.config
    assert len(fit['poses']) == len(fit['places']) > 0

    def out(i):  # m the body sticks out at off.poses[i] (turning there where it does), spin
        p, q = off.poses[i], off.poses[i - 1]
        if i in off.section_starts or angle_diff(p.yaw, q.yaw) <= footprint.SPIN:
            return sticks_out(plan.body_space, fp.body(p.x, p.y, p.yaw)), 0
        dx, dy = footprint.SPIN_BACK * math.cos(q.yaw), footprint.SPIN_BACK * math.sin(q.yaw)
        turn = normalize_angle(p.yaw - q.yaw)
        hulls = [MultiPoint([c for x0, y0 in ((p.x, p.y), (p.x - dx, p.y - dy)) for c in fp.body(
            x0, y0, q.yaw + t * k / n).exterior.coords]).convex_hull
            for t in ((turn, turn - math.copysign(2 * math.pi, turn)) if abs(turn) > math.pi - footprint.TURN_STEP
                      else (turn,)) for n in [int(math.ceil(abs(t) / math.radians(2)))] for k in range(n + 1)]
        return sticks_out(plan.body_space.buffer(-cfg.spin_margin, 16), unary_union(hulls)), 1

    at: Dict[tuple, List[int]] = {}
    for i, p in enumerate(off.poses):
        at.setdefault((round(p.x, 3), round(p.y, 3), round(p.yaw, 3)), []).append(i)
    for (x, y, m), (px, py, yaw, d, spin) in zip(fit['places'], fit['poses']):
        assert [x, y] == pytest.approx([px, py], abs=0.0051) and m >= 0 and d >= 0  # (to cm)
        assert any(s == spin and abs(e - d) < 2.5e-3 for e, s in map(out, at[(px, py, yaw)]))


@pytest.mark.parametrize('deg', [0, 33, 101])
def test_body_fits_at_any_angle(deg):
    """Where the body would stick out the path goes another way, whatever the angle: nothing sticks out (also against
    the real edges worked out here on their own), nowhere does it jump, the rest of the path is as planned, and stats
    has every way in place, at the pose of the path as planned where the body sticks out the most there (check_places).
    Left: where the loops along the lines don't turn round a corner or get on their line without the body's rear or
    front swinging out, they get off it before the corner and back on after it (turning at a kink of more than SPIN
    the body keeps spin_margin further in, like on the spot)."""
    obstacles = [TREE, BED]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=math.radians(deg)))
    plan = plan_coverage(GARDEN, obstacles, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, obstacles)
    # without it: what sticks out are loops, spins and drives, the turns and lane ends fit on their own
    off = plan_coverage(GARDEN, obstacles, dataclasses.replace(cfg, body_fit=False), start=(1, 1, 0.0), end=(1, 1))
    report, before = check_plan(plan), check_plan(off)
    # (0.4 m2 of that: the dock by a corner of the garden, the outer loop starting in it where the mower can't turn, it
    # gets on after the corner from where it can, see bodyfit.FIT_SPOTS)
    assert report['body_out'] == [] and report['left_m2'] <= before['left_m2'] + 1.75
    assert outside_room(plan, recorded_room(cfg, [GARDEN], obstacles, [cfg.obstacle_margin] * 2), spin=0.05) == []
    assert plan.stats['body_fit']['jumps'] == [] and len(plan.section_starts) == 1
    out = before['body_out']
    assert {'x', 'y', 'yaw', 'kind'} == set(out[0]) and not [o for o in out if o['kind'] in ('turn', 'lane_end')]
    planned = {(p.x, p.y) for p in off.poses}
    ways = [s for s in plan.segments if s.kind == TRANSIT and s.maneuver == 'body']
    # (the drives' corners rounded, also into the ends of loops and lanes next to them, turn_radius at most)
    drives = prep(MultiPoint([(p.x, p.y) for s in plan.segments if s.kind == TRANSIT
                              for p in plan.poses[s.start:s.end + 1]]).buffer(cfg.turn_radius + 1e-6))
    assert all((p.x, p.y) in planned or drives.contains(Point(p.x, p.y)) for s in plan.segments if s.kind != TRANSIT
               for p in plan.poses[s.start:s.end + 1])
    fit = plan.stats['body_fit']
    assert fit['fixed'] == len(ways) > 0 and len(fit['places']) == fit['fixed'] + fit['left']
    assert fit['skipped_m'] == pytest.approx(sum(m for _, _, m in fit['places']), abs=0.01)
    check_places(plan, off)
    # (round the way in place, a pose it goes round. The way to the start and the one from the end: their places come
    # after the others')
    mid = ways[1 if ways[0].start == 0 else 0:len(ways) - (ways[-1].end == len(plan.poses) - 1)]
    for s, (x, y, _) in zip(mid, fit['places']):
        assert min(math.hypot(p.x - x, p.y - y) for p in plan.poses[s.start:s.end + 1]) < 4 * cfg.turn_radius
    lines = unary_union([Polygon(ring).exterior for ring in [GARDEN] + obstacles])
    assert all(lines.distance(Point(x, y)) < 0.3 for x, y, _ in fit['places'][fit['fixed']:])
    assert any('places left out' in w for w in plan.warnings) == (fit['left'] > 0)


def test_body_fits_through_narrow_gaps():
    """A table recorded 8 cm off the outline, a sunshade 18 cm: the loops between them and the outline turn where the
    body doesn't fit, the path goes another way there, stats saying where and what that leaves out."""
    table, shade = rect(3, 5.0, 5, 5.92), circle(8, 5.47, 0.35, 16)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3, narrow_parts='loops',
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
    is one. (Drives keeping 5 cm off the edges: farther off they find a way.)"""
    trees = [circle(4, 4, 0.4), circle(8, 4.3, 0.5)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, perimeter_passes=3, lane_overlap_passes=2,
                               transit_clearance=0.05))
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
    place left out: the mower drives to its first pose on its own. Where nothing else fits either (turning round,
    driving on) FIT_LATER after it at most, it starts later too, that a place left out. Places left out further on
    still are, with a warning. (Not starting elsewhere, see route.START_CLEAR: as where the plan doesn't start with a
    loop.)"""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    monkeypatch.setattr(bodyfit, 'FIT_ROUND', 0)
    monkeypatch.setattr(_Route, '_start_at', lambda self, poses, target: None)
    plans = []
    for start in (0.0, FIT_START):
        monkeypatch.setattr(bodyfit, 'FIT_START', start)
        monkeypatch.setattr(bodyfit, 'FIT_LATER', start)
        plans.append(plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1)))
    jumped, later = (p.stats['body_fit'] for p in plans)
    assert len(jumped['jumps']) == len(plans[0].section_starts) - 1 > 0 and later['jumps'] == []
    assert jumped['left'] == len(jumped['jumps']) and any('places left out' in w for w in plans[0].warnings)
    assert len(plans[1].section_starts) == 1 and plans[1].poses[0] != plans[0].poses[0]
    assert plan_footprint(plans[1]).fits_pose(*plans[1].poses[0])
    assert later['left'] == 0 and not any('places left out' in w for w in plans[1].warnings)
    assert path_length(plans[1].poses) < path_length(plans[0].poses)
    # FIT_LATER further on: no jump there either, the start a place left out
    monkeypatch.setattr(bodyfit, 'FIT_START', 0.0)
    monkeypatch.setattr(bodyfit, 'FIT_LATER', FIT_LATER)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    fit = plan.stats['body_fit']
    assert fit['jumps'] == [] and len(plan.section_starts) == 1 and fit['left'] == 1
    assert plan.poses[0] == plans[0].poses[plans[0].section_starts[1]]
    monkeypatch.setattr(bodyfit, 'FIT_START', FIT_START)  # (as for the later plan above)
    monkeypatch.setattr(bodyfit, 'FIT_LATER', FIT_START)
    # nothing tried for a place: those further on are left out, the path jumping over them, the start still isn't one
    monkeypatch.setattr(bodyfit, 'FIT_PLACE', (0, 0))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    fit = plan.stats['body_fit']
    assert plan.poses[0] == plans[1].poses[0] and fit['left'] == len(fit['jumps']) == len(plan.section_starts) - 1 > 0
    assert any('places left out' in w for w in plan.warnings)


def test_body_fit_place_where_it_sticks_out_most():
    """A loop getting closer to the edge till it turns away (the body sticking out further and further, most turning on
    the spot at the corner, its rear corner swinging out 0.244 - 0.15 m): the path goes another way there, the place at
    the corner, not where it starts sticking out, stats['body_fit']['poses'] saying how far (Footprint.out_by of the
    worst of the poses there that don't fit) and that it turns on the spot there."""
    room = Polygon(rect(0, 0, 10, 10))
    transit, fp = TransitPlanner(room, 0.05), Footprint(room, BODY)
    route = _Route(PlannerConfig(**dict(MOWER, **NX60I)), transit, transit, None, None, None, fp)
    loop = join(straight(1, 9.45, 6, 9.85), straight(6, 9.85, 8, 8))
    poses, infos = bodyfit._assemble([PathSegment(PERIMETER, loop)])
    _, _, fit = bodyfit._fit_body(route, poses, infos)
    bad = fp.bad_poses(poses)
    worst = max(bad, key=lambda i: fp.out_by([(poses[i - 1], poses[i])])[0][0])
    corner = poses[worst]
    assert len(fit['places']) == len(fit['poses']) == 1 and (corner.x, corner.y) == pytest.approx((6, 9.85))
    assert worst != bad[0] and fit['places'][0][:2] == [6.0, 9.85]
    d = fp.out_by([(poses[worst - 1], corner)])[0][0]
    assert fit['poses'][0] == [6.0, 9.85, round(corner.yaw, 3), d, 1] and d == round(math.hypot(0.14, 0.2) - 0.15, 3)


def test_plan_starts_where_the_mower_gets_on(monkeypatch):
    """The dock by a corner of the garden: the outer loop doesn't start just before the corner (the body not fitting
    FIT_START after it, the path would start later, after the corner, mower_logic driving there and turning on the spot
    unchecked), but where a way in fits from where the mower turns whatever way it faces (route.START_CLEAR): the plan
    starts there, the way getting onto the loop where it starts (nothing left out at the start), the corner driven
    another way like any other. Hardly more left unmowed than starting later there."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED])
    fit, p, way = plan.stats['body_fit'], plan.poses[0], plan.segments[0]
    assert plan_footprint(plan).turning.anywhere.distance(Point(p.x, p.y)) < 1e-6
    assert way.kind == TRANSIT and way.maneuver == 'body' and fit['unchecked'] == []
    assert plan.segments[1].kind == PERIMETER and plan.segments[1].start == way.end
    start = plan.poses[way.end]
    assert min(math.hypot(start.x - x, start.y - y) for x, y in GARDEN) > 1.0  # (away from the corner)
    monkeypatch.setattr(_Route, '_start_at', lambda self, poses, target: None)
    later = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    assert later.poses[later.segments[0].end] != start
    assert check_plan(plan)['left_m2'] <= check_plan(later)['left_m2'] + 0.2


def test_body_fit_late_start_left_out(monkeypatch):
    """Where the body doesn't fit at the plan's start (its first poses here) it starts later: a way from where the body
    turns whatever way it faces, closest to where it would start, gets it there with the body checked (getting from
    the dock to the path mower_logic doesn't check it), a place driven another way. Where none gets there from that
    point, one from another of those (FIT_SPOTS). Where no such way fits it starts later all the same and says where
    (unchecked, a warning), no place left out while that leaves FIT_START m of loops and lanes out at most. With more
    it's a place left out like any other, a warning saying so. (Not starting elsewhere, see route.START_CLEAR: as where
    the plan doesn't start with a loop.)"""
    monkeypatch.setattr(_Route, '_start_at', lambda self, poses, target: None)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    lawn, dock = rect(0, 0, 10, 6), (5, -0.3, 0.0)
    planned = plan_coverage(lawn, [], dataclasses.replace(cfg, body_fit=False), start=dock, end=dock[:2])
    fits_pose, fit, is_free = Footprint.fits_pose, Footprint.fit, TransitPlanner.is_free
    # (where it turns whatever way it faces closest to where it would start)
    room = plan_footprint(planned).turning.anywhere.intersection(planned.transit_space)
    q = nearest_points(room, Point(planned.poses[0].x, planned.poses[0].y))[0]
    # (lead 'other': no way gets there from that point, False: nor from any other, only that one tried)
    for gone, lead in itertools.product((0.5 * FIT_START, 3.0), (True, 'other', False)):
        k = next(k for k in range(len(planned.poses)) if path_length(planned.poses[:k + 1]) > gone)
        head = {(p.x, p.y, p.yaw) for p in planned.poses[:k]}  # the body doesn't fit there
        monkeypatch.setattr(Footprint, 'fits_pose', lambda fp, x, y, yaw: (x, y, yaw) not in head
                            and fits_pose(fp, x, y, yaw))
        monkeypatch.setattr(Footprint, 'fit', lambda fp, poses, on_turn=False: [
            f and (p.x, p.y, p.yaw) not in head for f, p in zip(fit(fp, poses, on_turn), poses)])
        monkeypatch.setattr(TransitPlanner, 'is_free', lambda t, pts: (lead is True or all(
            math.hypot(x - q.x, y - q.y) > 1e-6 for x, y in pts)) and is_free(t, pts))
        monkeypatch.setattr(bodyfit, 'FIT_SPOTS', FIT_SPOTS if lead else 1)
        config._FITTED.clear()
        plan = plan_coverage(lawn, [], cfg, start=dock, end=dock[:2])
        fitted = plan.stats['body_fit']
        assert fitted['jumps'] == [] and len(plan.section_starts) == 1 and check_plan(plan)['body_out'] == []
        warned = [w for w in plan.warnings if "places left out where the body doesn't fit" in w or 'unchecked' in w or
                  "the body isn't checked" in w]
        if lead:
            p, way = plan.poses[0], plan.segments[0]
            room = plan_footprint(plan).turning.anywhere
            assert room.distance(Point(p.x, p.y)) < 1e-6 and way.kind == TRANSIT and way.maneuver == 'body'
            # (where it gets on the path: the pose as planned there, rounding a corner after the way the closest of
            # those within reach)
            e, reach = plan.poses[way.end], FIT_REACH[0 if lead is True else -1]
            near = [i for i in range(k, len(planned.poses)) if path_length(planned.poses[k:i + 1]) <= reach]
            j = min(near, key=lambda i: math.hypot(planned.poses[i].x - e.x, planned.poses[i].y - e.y))
            assert math.hypot(planned.poses[j].x - e.x, planned.poses[j].y - e.y) < (1e-9 if lead is True else 0.25)
            assert fitted['left'] == 0 and fitted['unchecked'] == [] and not warned
            assert (math.hypot(p.x - q.x, p.y - q.y) < 1e-6) == (lead is True)
            # (at the pose of those left out the body sticks out the most at: nowhere here, it fits but for this test,
            # the first of them. Getting on at a corner, the corner rounded after the way: up to there at most)
            m = path_length(planned.poses[:j + 1])
            assert fitted['places'][-1][:2] == [round(planned.poses[0].x, 2), round(planned.poses[0].y, 2)]
            if lead is True:
                assert fitted['places'][-1][2] == pytest.approx(m, abs=0.15)
            else:
                assert 0 < fitted['places'][-1][2] <= m + 0.15
            assert fitted['poses'][-1][3:] == [0.0, 0]
            continue
        p = planned.poses[k]
        assert plan.poses[0] == p and fitted['unchecked'] == [[round(p.x, 2), round(p.y, 2)]]
        assert any(f'({p.x:.2f}, {p.y:.2f}) the body isn\'t checked' in w for w in warned)
        if gone < FIT_START:
            assert fitted['left'] == 0 and len(warned) == 1
            continue
        first = planned.poses[0]
        assert fitted['left'] == 1 and len(warned) == 2
        assert fitted['places'][-1] == [round(first.x, 2), round(first.y, 2), pytest.approx(gone, abs=0.1)]
    config._FITTED.clear()


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
    # (0.54 m between the lines at 6.5, the body's 0.4 and its margins: 1 m wide, room to turn round one way. Turning
    # it keeps spin_margin further off them, 0.1 m more: about 0.75 m further back, the way in counting too)
    assert 4.5 < max(p.y for p in plan.poses) < 6.0
    assert any(y > 4.0 and m > 0 for _, y, m in fit['places']) and any('driven another way' in w for w in plan.warnings)


# a sunshade recorded 11.6 cm off a table and the outline round them (part of a garden, simplified to 2 cm)
SHADE_LAWN = [(0.54, 9.5), (0.67, 10.24), (0.68, 10.47), (0.54, 11.0), (0.36, 11.97), (0.36, 12.86), (0.53, 13.65),
              (0.59, 13.85), (0.67, 14.0), (4.0, 14.0), (4.0, 9.5)]
SHADE_TABLE = [(3.17, 11.66), (3.61, 12.22), (3.65, 12.44), (3.63, 12.67), (3.57, 12.88), (3.35, 13.29), (3.17, 13.54),
               (2.99, 13.65), (2.66, 13.73), (2.33, 13.7), (2.11, 13.64), (1.67, 13.3), (1.33, 12.89), (1.29, 12.68),
               (1.29, 12.45), (1.4, 12.13), (1.67, 11.77), (2.05, 11.5), (2.47, 11.37), (2.8, 11.39)]
SHADE = [(1.51, 11.79), (1.33, 11.91), (1.12, 11.97), (0.91, 11.94), (0.81, 11.89), (0.69, 11.71), (0.65, 11.51),
         (0.68, 11.28), (0.8, 11.11), (0.98, 11.01), (1.19, 10.98), (1.29, 11.0), (1.47, 11.11), (1.58, 11.3),
         (1.6, 11.45), (1.55, 11.69)]
SHADE_MOWER = dict(FORWARD, perimeter_offset=0.05, blade_ahead=0.185, robot_width=0.41, robot_front=0.43,
                   robot_rear=0.18, narrow_parts='loops', lane_overlap_passes=2,
                   lane_spacing_mode='auto', fill_pattern='concentric', perimeter_passes=-1)


@pytest.mark.parametrize('away', [0.0, 0.1])
def test_body_gap_between_obstacles(away):
    """The sunshade 11.6 cm off the table, margins 4 cm: facing straight across the body (41 cm wide) has 3.6 cm to
    spare between their real edges, less than body_tolerance. The gap is closed, the loops go round both: the path
    doesn't go between them, nowhere does it jump, nothing sticks out (also against the real edges worked out here on
    their own). 10 cm further apart it goes between them."""
    a, b = nearest_points(Polygon(SHADE), Polygon(SHADE_TABLE))
    shade = [(x + away * (a.x - b.x) / a.distance(b), y + away * (a.y - b.y) / a.distance(b)) for x, y in SHADE]
    gap = LineString(nearest_points(Polygon(shade), Polygon(SHADE_TABLE)))
    cfg = PlannerConfig(**dict(SHADE_MOWER, edge_margin=0.04, obstacle_margin=0.04))
    plan = plan_coverage(SHADE_LAWN, [SHADE_TABLE, shade], cfg, start=(0.54, 9.5, 0.0), end=(0.54, 9.5))
    check_invariants(plan, cfg, [SHADE_TABLE, shade])
    assert plan.stats['body_fit']['jumps'] == [] and len(plan.section_starts) == 1
    assert check_plan(plan)['body_out'] == []
    assert outside_room(plan, recorded_room(cfg, [SHADE_LAWN], [SHADE_TABLE, shade], [0.04, 0.04])) == []
    assert LineString([(p.x, p.y) for p in plan.poses]).intersects(gap) == (away > 0)


def test_spin_margin_kept():
    """Turning on the spot the body keeps spin_margin further off the real edges than the margins (worked out here on
    their own), anywhere on the last SPIN_BACK of the way in, where OpenMower's FTC turns: round the sunshade and the
    table, nowhere jumping, nothing sticking out. With spin_margin 0 it turns closer to them than that."""
    cfg = PlannerConfig(**dict(SHADE_MOWER, edge_margin=0.04, obstacle_margin=0.04))
    room = recorded_room(cfg, [SHADE_LAWN], [SHADE_TABLE, SHADE], [0.04, 0.04])
    plans = [plan_coverage(SHADE_LAWN, [SHADE_TABLE, SHADE], dataclasses.replace(cfg, spin_margin=m),
                           start=(0.54, 9.5, 0.0), end=(0.54, 9.5)) for m in (cfg.spin_margin, 0.0)]
    for plan in plans:
        check_invariants(plan, plan.config, [SHADE_TABLE, SHADE])
        assert plan.stats['body_fit']['jumps'] == [] and check_plan(plan)['body_out'] == []
        assert outside_room(plan, room) == []
    assert cfg.spin_margin == 0.05 and outside_room(plans[0], room, spin=0.05) == []
    assert outside_room(plans[1], room, spin=0.05)


def test_turns_keep_spin_margin():
    """All along the turns between lanes (turn_poses) the body keeps spin_margin further off the real edges than the
    margins (worked out here on their own), where the mower keeps less exactly to its path than going straight; with
    spin_margin 0 they come up to the margins."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    room = recorded_room(cfg, [GARDEN], [TREE, BED], [cfg.obstacle_margin] * 2)
    off = []
    for m in (cfg.spin_margin, 0.0):
        plan = plan_coverage(GARDEN, [TREE, BED], dataclasses.replace(cfg, spin_margin=m), start=(1, 1, 0.0),
                             end=(1, 1))
        fp, turns = plan_footprint(plan), turn_poses(plan)
        assert len(turns) > 1000
        off.append(min(room.boundary.distance(fp.body(p.x, p.y, p.yaw)) for p in (plan.poses[i] for i in turns)))
    assert cfg.spin_margin == 0.05 and off[0] > 0.05 - 1e-3 and off[1] < 0.01


@pytest.mark.parametrize('corners', [NX60, MEASURED, WHEELS], ids=['nx60', 'measured', 'wheels'])
def test_robot_outline_kept_off_the_edges(corners):
    """With the body's outline (robot_outline) round the sunshade and the table: nowhere jumping, nothing sticking out,
    the margins kept off the real edges (worked out here on their own with that outline), turning on the spot
    spin_margin further off them; stats has every way in place at the pose where the body sticks out the most there,
    as far as it does (check_places)."""
    cfg = PlannerConfig(**dict(SHADE_MOWER, edge_margin=0.04, obstacle_margin=0.04, robot_outline=corners))
    ends = dict(start=(0.54, 9.5, 0.0), end=(0.54, 9.5))
    plan = plan_coverage(SHADE_LAWN, [SHADE_TABLE, SHADE], cfg, **ends)
    check_invariants(plan, cfg, [SHADE_TABLE, SHADE])
    assert plan.stats['body_fit']['jumps'] == [] and check_plan(plan)['body_out'] == []
    room = recorded_room(cfg, [SHADE_LAWN], [SHADE_TABLE, SHADE], [0.04, 0.04])
    assert outside_room(plan, room, spin=cfg.spin_margin) == []
    check_places(plan, plan_coverage(SHADE_LAWN, [SHADE_TABLE, SHADE], dataclasses.replace(cfg, body_fit=False),
                                     **ends))


# the south end of a path (part of a garden's outline, simplified to 1 cm), too narrow at its end to turn round in
PATH_END = [(6.809, -4.12), (6.631, -4.748), (6.553, -4.971), (6.358, -5.828), (6.239, -6.169), (6.083, -6.829),
            (5.968, -7.125), (5.875, -7.465), (5.825, -7.915), (5.729, -8.381), (5.544, -8.883), (5.352, -9.856),
            (5.249, -10.178), (5.131, -10.633), (4.868, -11.356), (4.874, -11.459), (4.841, -11.58), (4.787, -11.678),
            (4.717, -11.897), (4.325, -12.735), (4.247, -12.814), (4.15, -12.846), (3.92, -12.877), (3.806, -12.852),
            (3.734, -12.778), (3.73, -12.671), (3.839, -12.144), (3.957, -11.963), (4.121, -11.539), (4.343, -10.892),
            (4.434, -10.565), (4.659, -9.941), (4.787, -9.32), (4.991, -8.654), (5.402, -7.768), (5.467, -7.563),
            (5.515, -7.12), (5.599, -6.808), (5.615, -6.69), (5.687, -6.013), (5.694, -5.768), (5.932, -4.71),
            (5.925, -4.491), (5.967, -4.188), (5.973, -4.0), (6.881, -4.0)]


def test_body_fit_turns_round_further_to_the_side():
    """Where the loops go round the end of the path a front rounded a little fits facing their way closer to its edge
    than it turns round there: where nothing else fits the path turns round further to the side, nowhere jumping,
    nothing sticking out (also against the real edges worked out here on their own)."""
    cfg = PlannerConfig(**dict(SHADE_MOWER, fill_pattern='lanes', perimeter_passes=3, edge_margin=0.04,
                               obstacle_margin=0.04, robot_outline=rounded(0.41, 0.43, 0.18, 0.15)))
    plan = plan_coverage(PATH_END, [], cfg, start=(5.973, -4.3, -1.57), end=(5.973, -4.3))
    check_invariants(plan, cfg)
    assert plan.stats['body_fit']['jumps'] == [] and len(plan.section_starts) == 1
    assert check_plan(plan)['body_out'] == []
    assert outside_room(plan, recorded_room(cfg, [PATH_END], [], []), spin=cfg.spin_margin) == []


def test_body_fit_drives_on(monkeypatch):
    """A strip between a table and the outline, closed at its end by a sunshade: where the body doesn't get round
    there and no other way fits nor turning round, the path drives on (through the transit space) to where it fits
    again instead of jumping there, nothing sticking out (also against the real edges worked out here on their own),
    getting back on at the first pose it gets to of every FIT_STEP[-1] m (FIT_ON). Trying twice as far each time from
    the place on (FIT_ON 0) it gets back on further round the loop, more of it left out, more unmowed."""
    lawn, obstacles = rect(0, 0, 5.3, 4.9), [rect(0.28, 2.7, 1.82, 3.86), circle(0.42, 2.25, 0.33, 16)]
    cfg = PlannerConfig(**dict(SHADE_MOWER, edge_margin=0.05, obstacle_margin=0.05, angle=math.radians(30)))
    plan = plan_coverage(lawn, obstacles, cfg, start=(5.0, 0.3, 1.6), end=(5.0, 0.3))
    check_invariants(plan, cfg, obstacles)
    fit, report = plan.stats['body_fit'], check_plan(plan)
    assert fit['jumps'] == [] and len(plan.section_starts) == 1 and fit['fixed'] > 0
    assert report['body_out'] == []
    assert outside_room(plan, recorded_room(cfg, [lawn], obstacles, [0.05, 0.05])) == []
    monkeypatch.setattr(bodyfit, 'FIT_ON', 0.0)
    doubled = plan_coverage(lawn, obstacles, cfg, start=(5.0, 0.3, 1.6), end=(5.0, 0.3))
    fitted, checked = doubled.stats['body_fit'], check_plan(doubled)
    assert fitted['jumps'] == [] and checked['body_out'] == []
    assert fit['skipped_m'] < fitted['skipped_m'] and report['left_m2'] < checked['left_m2']


@pytest.mark.parametrize('lead', [True, 'other', False])
def test_body_fit_ends_before(monkeypatch, lead):
    """Where the body doesn't fit by the path's end (the outer loop, all round its last pose but the last two here) and
    nothing else does, it ends before instead of jumping to its last two poses: no place left out while that leaves
    FIT_START m of loops and lanes out at most. From there a way to where the body turns whatever way it faces, closest
    to where it would end, the body checked (from the path to the dock mower_logic doesn't check it), else to another
    of those (FIT_SPOTS). Where none fits (nothing but the path does near the end here) it says where it ends
    (unchecked, a warning)."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, perimeter_order='last'))
    lawn, dock = rect(0, 0, 10, 6), (5, -0.3, 0.0)
    planned = plan_coverage(lawn, [], dataclasses.replace(cfg, body_fit=False), start=dock, end=dock[:2])
    end, last = planned.poses[-1], set(planned.poses[-2:])

    def bad(x, y, yaw):
        return math.hypot(x - end.x, y - end.y) < 0.3 and Pose2D(x, y, yaw) not in last

    if lead is not True:  # (no way gets to where it turns whatever way it faces closest to the end, False: only that
        # one tried)
        room = plan_footprint(planned).turning.anywhere.intersection(planned.transit_space)
        q, is_free = nearest_points(room, Point(end.x, end.y))[0], TransitPlanner.is_free
        monkeypatch.setattr(TransitPlanner, 'is_free', lambda t, pts: all(
            math.hypot(x - q.x, y - q.y) > 1e-6 for x, y in pts) and is_free(t, pts))
        monkeypatch.setattr(bodyfit, 'FIT_SPOTS', FIT_SPOTS if lead else 1)
    fits_pose, fit = Footprint.fits_pose, Footprint.fit
    monkeypatch.setattr(Footprint, 'fits_pose', lambda fp, x, y, yaw: not bad(x, y, yaw) and fits_pose(fp, x, y, yaw))
    monkeypatch.setattr(Footprint, 'fit', lambda fp, poses, on_turn=False: [
        f and not bad(p.x, p.y, p.yaw) for f, p in zip(fit(fp, poses, on_turn), poses)])
    config._FITTED.clear()
    plan = plan_coverage(lawn, [], cfg, start=dock, end=dock[:2])
    fitted, way = plan.stats['body_fit'], plan.segments[-1]
    assert fitted['jumps'] == [] and len(plan.section_starts) == 1 and fitted['left'] == 0
    assert check_plan(plan)['body_out'] == [] and plan.segments[-2 if lead else -1].kind == PERIMETER
    k = planned.poses.index(plan.poses[way.start - 1 if lead else -1])
    assert 0.3 < path_length(planned.poses[k:]) <= FIT_START + (FIT_REACH[0 if lead is True else -1] if lead else 0.0)
    p = plan.poses[-1]
    if lead:
        room = plan_footprint(plan).turning.anywhere
        assert way.kind == TRANSIT and way.maneuver == 'body' and room.distance(Point(p.x, p.y)) < 1e-6
        assert fitted['unchecked'] == [] and not any("the body isn't checked" in w for w in plan.warnings)
    else:
        assert fitted['unchecked'] == [[round(p.x, 2), round(p.y, 2)]]
        assert any(f"({p.x:.2f}, {p.y:.2f}) the body isn't checked" in w for w in plan.warnings)
    config._FITTED.clear()


def test_body_fit_bounded(monkeypatch):
    """A dead end too narrow to turn round in without backing up: the plan gets done, nothing sticking out, no more than
    FIT_PLACE ways tried for a place (none: none tried), FIT_ROUND turning round there, nor FIT_TRIES for all
    (FIT_TRIES_M 0 here), but for the ways to the start and from the end: FIT_ROUND from each of their points on top.
    Where it may try none it leaves out what doesn't fit and jumps over it, a warning saying where."""
    shape = [(0, 0), (6, 0), (6, 3), (3.3, 3), (3.3, 13), (2.7, 13), (2.7, 3), (0, 3)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    monkeypatch.setattr(bodyfit, 'FIT_TRIES_M', 0.0)
    for tries, place, turning in ((FIT_TRIES, FIT_PLACE, FIT_ROUND), (50, FIT_PLACE, FIT_ROUND),
                                  (FIT_TRIES, (0, 0), 0)):
        monkeypatch.setattr(bodyfit, 'FIT_TRIES', tries)
        monkeypatch.setattr(bodyfit, 'FIT_PLACE', place)
        monkeypatch.setattr(bodyfit, 'FIT_ROUND', turning)
        plan = plan_coverage(shape, [], cfg, start=(1, 1, 0.0))
        check_invariants(plan, cfg)
        fit = plan.stats['body_fit']
        leads = 2 * turning * (FIT_SPOTS + 1)
        assert check_plan(plan)['body_out'] == [] and fit['tries'] <= tries + leads
        assert (fit['tries'] > 0) == (sum(place) > 0)
        assert bool(fit['jumps']) == (tries == 50 or not turning) == (len(plan.section_starts) > 1)
        if fit['jumps']:
            assert all(y > 9 for _, y in fit['jumps'][-1:])
            assert any('places left out' in w and f'({fit["jumps"][-1][0]:.2f}, {fit["jumps"][-1][1]:.2f})' in w
                       for w in plan.warnings)
        if tries == 50:
            assert fit['tries'] >= 50


def test_body_fit_tries_grow_with_the_area(monkeypatch):
    """A bigger area has more places where the body sticks out: FIT_TRIES_M tries a metre of the edges of the space the
    body may take where that's more than FIT_TRIES. And where the plan's tries are used up the way to its start still
    gets its own (from where the body turns whatever way it faces, nothing unchecked: mower_logic would turn on the spot
    there)."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    lawn, dock = rect(0, 0, 10, 6), (5, -0.3, 0.0)
    planned = plan_coverage(lawn, [], dataclasses.replace(cfg, body_fit=False), start=dock, end=dock[:2])
    space = plan_footprint(planned).space
    k = next(k for k in range(len(planned.poses)) if path_length(planned.poses[:k + 1]) > 3.0)
    head = {(p.x, p.y, p.yaw) for p in planned.poses[:k]}  # the body doesn't fit there
    fits_pose, fit = Footprint.fits_pose, Footprint.fit
    monkeypatch.setattr(Footprint, 'fits_pose', lambda fp, x, y, yaw: (x, y, yaw) not in head
                        and fits_pose(fp, x, y, yaw))
    monkeypatch.setattr(Footprint, 'fit', lambda fp, poses, on_turn=False: [
        f and (p.x, p.y, p.yaw) not in head for f, p in zip(fit(fp, poses, on_turn), poses)])
    used = []
    for tries, per_m in ((0, 0.0), (0, 1.0)):
        monkeypatch.setattr(bodyfit, 'FIT_TRIES', tries)
        monkeypatch.setattr(bodyfit, 'FIT_TRIES_M', per_m)
        config._FITTED.clear()
        plan = plan_coverage(lawn, [], cfg, start=dock, end=dock[:2])
        fitted, p = plan.stats['body_fit'], plan.poses[0]
        assert fitted['unchecked'] == [] and plan.segments[0].kind == TRANSIT and plan.segments[0].maneuver == 'body'
        assert plan_footprint(plan).turning.anywhere.distance(Point(p.x, p.y)) < 1e-6
        used.append(fitted['tries'])
    # (none for the plan: the start's way's alone. FIT_TRIES_M 1: a try a metre of the edges for the plan, more used)
    leads = FIT_ROUND * (FIT_SPOTS + 1)
    assert used[0] <= leads and used[0] < used[1] <= int(space.length) + leads
    config._FITTED.clear()


def recorded_room(cfg, outlines, holes, margins, step=0.05):
    """Where the body keeps the margins off the real edges, worked out here on its own: the body (its corners as the
    settings give them) every step m along each recorded ring the way its points go and turning on the spot at its
    points every 2 degrees, the outlines grown by that and edge_margin in, the holes (obstacles, areas to keep out of)
    less that and each one's margin out."""
    half, front, rear = 0.5 * (cfg.robot_width or 0.0), cfg.robot_front, cfg.robot_rear
    corners = cfg.robot_outline or ((front, half), (-rear, half), (-rear, -half), (front, -half))

    def body(x, y, h):
        c, s = math.cos(h), math.sin(h)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in corners])

    def swept(ring):
        bodies = []
        for i, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
            c = ring[(i + 2) % len(ring)]
            h, turn = math.atan2(b[1] - a[1], b[0] - a[0]), 0.0
            n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
            bodies += [body(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, h) for k in range(n + 1)]
            turn = normalize_angle(math.atan2(c[1] - b[1], c[0] - b[0]) - h)
            m = max(1, int(math.ceil(abs(turn) / math.radians(2))))
            bodies += [body(b[0], b[1], h + turn * k / m) for k in range(1, m)]
        return unary_union(bodies)

    outer = unary_union([Polygon(o).union(swept(list(o))) for o in outlines]).buffer(-cfg.edge_margin, 16)
    return outer.difference(unary_union([Polygon(w).difference(swept(list(w))).buffer(m, 16)
                                         for w, m in zip(holes, margins)]))


def outside_room(plan, room, slack=1e-3, spin=0.0):
    """The poses whose body isn't in room (slack more), turning on the spot every 2 degrees where the heading jumps by
    more than SPIN, both ways round about a half turn (TURN_STEP apart the corners may pass 2 mm further out). spin:
    turning it keeps that much further in, anywhere on the last SPIN_BACK of the way in (the hull of the bodies at both
    ends), on the turns between lanes at the pose (turn_poses)."""
    fp, ok, turning = plan_footprint(plan), prep(room.buffer(slack, 16)), prep(room.buffer(slack + 2e-3 - spin, 16))
    bad, starts, turns = [], set(plan.section_starts), set(turn_poses(plan)) if spin else set()
    on = prep(room.buffer(slack - spin, 16))
    for i, p in enumerate(plan.poses):
        q = plan.poses[i - 1] if i not in starts else None
        if q is None or angle_diff(p.yaw, q.yaw) <= footprint.SPIN:
            if not (on if i in turns else ok).covers(fp.body(p.x, p.y, p.yaw)):
                bad.append(i)
            continue
        turn = normalize_angle(p.yaw - q.yaw)
        for t in (turn, turn - math.copysign(2 * math.pi, turn)) if abs(turn) > math.pi - footprint.TURN_STEP else \
                (turn,):
            n = int(math.ceil(abs(t) / math.radians(2)))
            back = footprint.SPIN_BACK if spin else 0.0
            x, y = p.x - back * math.cos(q.yaw), p.y - back * math.sin(q.yaw)
            if not all(turning.covers(MultiPoint(list(fp.body(p.x, p.y, h).exterior.coords) +
                                                 list(fp.body(x, y, h).exterior.coords)).convex_hull)
                       for h in (q.yaw + t * k / n for k in range(n + 1))):
                bad.append(i)
                break
    return bad


def turn_poses(plan):
    """The poses on the turns between lanes (the way to them part of a turn), not turning on the spot there."""
    starts, out = set(plan.section_starts), []
    for seg in plan.segments:
        if seg.kind == 'turn':
            out += [i for i in range(seg.start + 1, seg.end + 1) if i not in starts and
                    angle_diff(plan.poses[i].yaw, plan.poses[i - 1].yaw) <= footprint.SPIN]
    return out


U_LAWN = [(0, 0), (14, 0), (14, 10), (10, 10), (10, 3), (4, 3), (4, 10), (0, 10)]


@pytest.mark.parametrize('base', [dict(angle=0.0), dict(MOWER, **NX60I, angle=0.0)])
def test_split_zones(base):
    """split_zones: a part of the lawn better mowed another way gets lanes of its own direction. A U's two wings across
    the lanes: along them a lot fewer lanes and turns, as much mowed, stats['zones'] says which. The rest keeps its
    direction and is mowed first. A plain lawn stays as it is (not turned as a whole), so does one where zones gain
    nothing; not with crosshatch or headland_turns."""
    plain = plan_coverage(U_LAWN, [], PlannerConfig(**base), start=(1, 1, 0.0), end=(1, 1))
    cfg = PlannerConfig(**dict(base, split_zones=True))
    plan = plan_coverage(U_LAWN, [], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg)
    zones = plan.stats['zones']
    assert [round(math.degrees(z['angle'])) for z in zones] == [90, 90] and 'zones' not in plain.stats
    assert all(len(z['outlines']) == 1 and Polygon(z['outlines'][0]).area > 15.0 for z in zones)
    assert plan.stats['lanes'] < 0.65 * plain.stats['lanes'] and sorted(plan.angles) == pytest.approx([0.0] + [math.pi / 2] * 2)
    assert coverage(plan, cfg) > coverage(plain, cfg) - 0.0005 and check_plan(plan)['body_out'] == []
    assert drive_time(plan.poses)['s'] < drive_time(plain.poses)['s']
    # the rest (the lanes at 0) before the zones
    yaws = [round(math.degrees(plan.poses[s.start].yaw)) % 180 for s in straight_lanes(plan)]
    assert yaws.index(90) > max(i for i, y in enumerate(yaws) if y == 0)
    for kw, area, obstacles in ((dict(angle=0.3), rect(0, 0, 10, 6), []), (dict(base), GARDEN, [TREE, BED])):
        same = plan_coverage(area, obstacles, PlannerConfig(**kw))
        split = plan_coverage(area, obstacles, PlannerConfig(**dict(kw, split_zones=True)))
        assert split.poses == same.poses and split.stats['zones'] == []
    for kw in (dict(fill_pattern='crosshatch'), dict(headland_turns=True)):
        same = plan_coverage(U_LAWN, [], PlannerConfig(**dict(base, **kw)))
        assert plan_coverage(U_LAWN, [], PlannerConfig(**dict(base, split_zones=True, **kw))).poses == same.poses


def test_leftovers():
    """mowed (where the blade really went on a mow, report.track_mowed of its track): the plan is only passes over what
    that left, the nearest to the start first, drives between them, the body fitting everywhere. Here a mow that
    missed every 15th lane: most of that gets mowed, about what the plan itself leaves stays. Nothing left: no plan."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    runs, lanes = [[]], 0
    for s in plan.segments:
        lanes += s.kind == LANE and not s.maneuver
        if s.kind == LANE and not s.maneuver and lanes % 15 == 0:
            runs.append([])
        else:
            runs[-1] += [(p.x, p.y) for p in plan.poses[s.start:s.end + 1]]
    mowed = track_mowed(runs, cfg.mower_width, cfg.blade_ahead, cfg.blade_offset)

    def left(*geoms):
        return sum(p.area for p in polygons(plan.free_space.difference(unary_union(geoms))) if p.area >= 0.02)

    assert left(mowed) > 8.0 and left(mowed) == pytest.approx(left(swept_area(plan, cfg.mower_width)), abs=12.0)
    more = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1), mowed=mowed)
    check_invariants(more, cfg, [TREE, BED])
    stats = more.stats['leftovers']
    assert stats['left_m2'] == pytest.approx(left(mowed), abs=0.01) and stats['passes'] >= lanes // 15
    assert stats['area'] > 0.7 * stats['left_m2'] and check_plan(more)['body_out'] == []
    assert left(mowed, swept_area(more, cfg.mower_width)) < left(swept_area(plan, cfg.mower_width)) + 0.5
    assert {(s.kind, s.maneuver) for s in more.segments} == {(LANE, 'leftover'), (TRANSIT, 'leftover')}
    assert more.stats['path_length'] < 0.2 * plan.stats['path_length'] and plan.stats['leftovers'] is None
    assert math.hypot(more.poses[0].x - 1, more.poses[0].y - 1) < 3.0  # (the nearest to the start first)
    with pytest.raises(PlanningError, match='nothing left to mow'):
        plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1), mowed=Polygon(GARDEN).buffer(1.0))
    with pytest.raises(PlanningError, match='body'):
        plan_coverage(GARDEN, [TREE, BED], PlannerConfig(), mowed=mowed)


def test_mop_up():
    """mop_up: at the end a round over the biggest pieces left unmowed where the blade gets to (a pass over each, the
    body fitting, turning where they meet too), back to where the path ended: less left unmowed, nothing sticking out,
    no jump. stats['mop_up'] says how many and how much; off, the plan as without. A piece no way back fits from (by a
    corner) it leaves out: the plan ends where it would without."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3))
    corner = plan_coverage(GARDEN, [TREE, BED], dataclasses.replace(cfg, mop_up=True), start=(1, 1, 0.0), end=(1, 1))
    assert corner.stats['mop_up']['pieces'] == 0 and corner.poses == plan_coverage(
        GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1)).poses
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.5))
    plain = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    mop = plan_coverage(GARDEN, [TREE, BED], dataclasses.replace(cfg, mop_up=True), start=(1, 1, 0.0), end=(1, 1))
    check_invariants(mop, mop.config, [TREE, BED])
    report = check_plan(mop)
    assert report['body_out'] == [] and mop.stats['body_fit']['jumps'] == [] and len(mop.section_starts) == 1
    assert mop.stats['mop_up']['pieces'] > 0 and mop.stats['mop_up']['area'] > 0.0 and plain.stats['mop_up'] is None
    assert report['left_m2'] < check_plan(plain)['left_m2'] - 0.5 * mop.stats['mop_up']['area']
    passes = [s for s in mop.segments if s.maneuver == 'mop_up' and s.kind == LANE]
    assert len(passes) == mop.stats['mop_up']['pieces'] and mop.poses[:len(plain.poses) - 1] == plain.poses[:-1]
    assert math.hypot(mop.poses[-1].x - plain.poses[-1].x, mop.poses[-1].y - plain.poses[-1].y) < 0.05


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
    for kw in (dict(edge_margin=-0.1), dict(obstacle_margin=1.5), dict(edge_margin=math.nan), dict(spin_margin=-0.01),
               dict(spin_margin=1.5), dict(spin_margin=True)):
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
    out. That with headland_corners 'sharp'; with 'rounded' (the default) the drives' corners at the field's edge are
    arcs, into the field (FIELD_ARC at most) only where none fits out of it, those counted on their own, not as turning
    in it."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3))
    plain = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    clean = dataclasses.replace(cfg, headland_turns=True, headland_corners='sharp')
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
    rounded = plan_coverage(GARDEN, [TREE, BED], dataclasses.replace(clean, headland_corners='rounded'),
                            start=(1, 1, 0.0), end=(1, 1))
    check_invariants(rounded, clean, [TREE, BED])
    assert len(transit_corners(rounded)) < 0.8 * len(transit_corners(plan))
    inside = field_turns(rounded, field)
    deepest = max(field.boundary.distance(Point(rounded.poses[i].x, rounded.poses[i].y)) for i in inside)
    found = rounded.stats['headland_turns']
    assert inside and deepest < FIELD_ARC and found['turns_in_field'] == 0 and check_plan(rounded)['body_out'] == []
    # (those arcs counted on their own: one of them for every run of poses here, the field here 2 cm smaller)
    runs = sum(1 for i in inside if i - 1 not in inside)
    assert found['arcs_in_field'] >= runs and deepest + 0.02 - 0.01 < found['arc_depth'] <= FIELD_ARC
    with pytest.raises(PlanningError, match='headland_corners'):
        PlannerConfig(headland_corners='round').validate()


def test_headland_turns_where_nothing_fits(monkeypatch):
    """Where no drive along the headland (nor across the field) fits, it goes the way it would without: stats and a
    warning say where it turns in the lanes' field."""
    monkeypatch.setattr(_Route, 'headland_way', lambda self, target, heading, head: None)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3, headland_turns=True))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED])
    found = plan.stats['headland_turns']
    assert found['turns_in_field'] == len(found['places']) > 0 and found['crossings'] == 0
    x, y = found['places'][0]
    assert any("turns in the lanes' field" in w and f'({x:.2f}, {y:.2f})' in w for w in plan.warnings)
    assert len(field_turns(plan, lanes_field(plan, cfg))) > 0


def straight(x0, y0, x1, y1):
    return sample_straight(x0, y0, x1, y1, math.atan2(y1 - y0, x1 - x0), 0.1)


def corner_route(space, pieces, **kw):
    """A route through space (the body fitting anywhere) and the poses and segments of pieces [(kind, poses, section
    or none)] put together like a plan's, for _round_corners and _turn_at."""
    transit = TransitPlanner(space, 0.05)
    route = _Route(PlannerConfig(**dict(MOWER, **NX60I, **kw)), transit, transit, None, None, None,
                   Footprint(Polygon(rect(-50, -50, 50, 50)), BODY))
    return (route,) + bodyfit._assemble([PathSegment(piece[0], piece[1], section=piece[2] if piece[2:] else 0)
                                         for piece in pieces])


def arc_radius(poses, centre):
    """The distances from centre of the poses turning (heading between the first one's and the last one's)."""
    return {round(math.hypot(p.x - centre[0], p.y - centre[1]), 6) for p in poses[1:-1]
            if 1e-6 < angle_diff(p.yaw, poses[0].yaw) and 1e-6 < angle_diff(p.yaw, poses[-1].yaw)}


def test_round_corners():
    """A drive's corner (east, then north at (5, 5)): an arc in its place tangent to both ways, of turn_radius, the
    lanes before and after as they were. At a lane's end the lane ends turn_radius earlier for it (as much as a right
    angle of turn_radius takes, no more where it turns further round) where what that leaves of it is mowed elsewhere
    (a loop along the lane, in a section of its own here), else an S from the lane's end onto the drive. Where the arc
    doesn't fit (an obstacle in the corner) a tighter one, down to SMOOTH_RADII[-1] of turn_radius; where none does
    the corner stays, as it does where a drive along the headland goes with headland_corners 'sharp'."""
    r = 0.25
    lane, north = straight(1, 5, 4, 5), straight(5, 7, 5, 9)
    drive = join(straight(4, 5, 5, 5), straight(5, 5, 5, 7))
    room = Polygon(rect(0, 0, 10, 10))
    route, poses, infos = corner_route(room, [(LANE, lane), (TRANSIT, drive), (LANE, north)])
    out, segs = bodyfit._round_corners(route, poses, infos)
    assert [s.kind for s in segs] == [LANE, TRANSIT, LANE] and segs[0].start == 0 and segs[-1].end == len(out) - 1
    assert all(t.start == s.end for s, t in zip(segs, segs[1:]))
    assert out[:segs[0].end + 1] == lane and out[segs[2].start:] == north
    assert max(angle_diff(q.yaw, p.yaw) for p, q in zip(out, out[1:])) <= MAX_ARC_STEP + 1e-9
    way = out[segs[1].start:segs[1].end + 1]
    assert arc_radius(way, (5 - r, 5 + r)) == {r} and any(math.hypot(p.x - 5 + r, p.y - 5) < 1e-9 for p in way)
    # at a lane's end
    loop = (PERIMETER, straight(6, 5, 3, 5), 1)
    route, poses, infos = corner_route(room, [(LANE, straight(1, 5, 5, 5)), (TRANSIT, straight(5, 5, 5, 7)), loop])
    out, segs = bodyfit._round_corners(route, poses, infos)
    end = out[segs[0].end]
    assert (end.x, end.y, end.yaw) == pytest.approx((5 - r, 5, 0)) and out[:segs[0].end] == poses[:segs[0].end]
    assert arc_radius(out[segs[1].start:segs[1].end + 1], (5 - r, 5 + r)) == {r}
    route, poses, infos = corner_route(room, [(LANE, straight(1, 5, 5, 5)), (TRANSIT, straight(5, 5, 5, 7))])
    out, segs = bodyfit._round_corners(route, poses, infos)
    assert out[:segs[0].end] == poses[:infos[0].end] and (out[segs[0].end].x, out[segs[0].end].y) == (5, 5)
    assert max(angle_diff(q.yaw, p.yaw) for p, q in zip(out, out[1:])) <= MAX_ARC_STEP + 1e-9
    assert any(p.x > 5.01 for p in out[segs[1].start:]) and out[-1] == poses[-1]
    # turning round at a lane's end by 150 degrees: the lane no more than turn_radius shorter
    h = math.radians(150)
    route, poses, infos = corner_route(room, [(LANE, straight(1, 5, 5, 5)),
                                              (TRANSIT, straight(5, 5, 5 + 3 * math.cos(h), 5 + 3 * math.sin(h)))])
    out, segs = bodyfit._round_corners(route, poses, infos)
    kept = [p for p in poses[:infos[0].end] if p.x < 5 - r - 1e-9]
    assert out[segs[0].end].x >= 5 - r - 1e-9 and out[:len(kept)] == kept
    # an obstacle in the corner: the tightest
    for block, radius in ((rect(0, 5.03, 4.97, 10), SMOOTH_RADII[-1] * r), (rect(0, 5.005, 4.995, 10), None)):
        route, poses, infos = corner_route(room.difference(Polygon(block)),
                                           [(LANE, lane), (TRANSIT, drive), (LANE, north)])
        out, segs = bodyfit._round_corners(route, poses, infos)
        if radius is None:
            assert (out, segs) == (poses, infos)
        else:
            assert arc_radius(out[segs[1].start:segs[1].end + 1], (5 - radius, 5 + radius)) == {round(radius, 6)}
    route, poses, infos = corner_route(room, [(LANE, lane), (TRANSIT, drive), (LANE, north)])
    route.sharp.add((5.0, 5.0))
    assert bodyfit._round_corners(route, poses, infos) == (poses, infos)


def test_round_corners_s_and_field(monkeypatch):
    """A drive leaving a loop where it bends (no straight way in to round it from): an S onto the drive's way further
    on, the loop as it was. With headland_turns an arc keeps out of the lanes' field where the corner is out of it;
    where none does, with headland_corners 'rounded' the widest into it that gets no more than FIELD_ARC in (its poses
    in route.rounded: no turn in the field for stats, counted on their own), with 'sharp' the corner stays, as it does
    where any arc would get further in (an arc of r gets 0.293 r - 0.02 in here)."""
    loop = sample_arc(3.0, 4.6, -0.6, 1.0, 0.6, 0.1)
    q = loop[-1]
    route, poses, infos = corner_route(Polygon(rect(0, 0, 10, 10)), [(PERIMETER, loop),
                                                                       (TRANSIT, straight(q.x, q.y, q.x, q.y + 2))])
    out, segs = bodyfit._round_corners(route, poses, infos)
    assert out[:len(loop) - 1] == loop[:-1] and segs[0].end == len(loop) - 1
    assert max(angle_diff(b.yaw, a.yaw) for a, b in zip(out, out[1:])) <= MAX_ARC_STEP + 1e-9
    way = out[segs[1].start:]
    assert abs(way[-1].x - q.x) < 1e-9 and any(p.x > q.x + 0.01 for p in way)  # (out to the right, back onto it)
    field = Polygon(rect(0, 5.02, 4.98, 10))
    lane, drive, north = straight(1, 5, 4, 5), join(straight(4, 5, 5, 5), straight(5, 5, 5, 7)), straight(5, 7, 5, 9)
    for corners, deep, r in (('rounded', FIELD_ARC, 0.25), ('sharp', FIELD_ARC, None), ('rounded', 0.03, 0.125),
                             ('rounded', 0.005, None)):
        monkeypatch.setattr(bodyfit, 'FIELD_ARC', deep)
        route, poses, infos = corner_route(Polygon(rect(0, 0, 10, 10)), [(LANE, lane), (TRANSIT, drive),
                                                                         (LANE, north)], headland_corners=corners)
        route.field, route._field = field, prep(field)
        out, segs = bodyfit._round_corners(route, poses, infos)
        assert (out == poses) == (r is None)
        if r is not None:
            way = out[segs[1].start:segs[1].end + 1]
            assert arc_radius(way, (5 - r, 5 + r)) == {r} and not route.clear([(p.x, p.y) for p in way])
            assert all((p.x, p.y) in route.rounded for p in way if field.contains(Point(p.x, p.y)))
            # (along the way from pose to pose)
            turning, arcs, depth = bodyfit._field_turns(route, out, segs)
            assert turning == [] and arcs == 1 and depth == pytest.approx(0.293 * r - 0.02, abs=2e-3) and depth <= deep


def test_turn_at():
    """Where the path turns on the spot a pose TURN_AHEAD before the point, facing the way it came, the segments moved
    on: the one before ends at the point; also at a kink of more than SPIN (OpenMower's FTC turns there the same way),
    not at one of less. Not where the way in is no longer than twice that."""
    pivot = [Pose2D(5, 5, math.pi / 2)] + straight(5, 5, 5, 7)
    route, poses, infos = corner_route(Polygon(rect(0, 0, 10, 10)), [(LANE, straight(1, 5, 5, 5)), (TURN, pivot)])
    out, segs = bodyfit._turn_at(route, poses, infos)
    k = infos[1].start
    assert out[:k] == poses[:k] and out[k + 1:] == poses[k:] and len(out) == len(poses) + 1
    assert (out[k].x, out[k].y, out[k].yaw) == pytest.approx((5 - TURN_AHEAD, 5, 0))
    assert [(s.start, s.end) for s in segs] == [(0, k + 1), (k + 1, len(out) - 1)]
    short = straight(1, 5, 4.96, 5) + [Pose2D(5, 5, 0.0)]
    route, poses, infos = corner_route(Polygon(rect(0, 0, 10, 10)), [(LANE, short), (TURN, pivot)])
    assert bodyfit._turn_at(route, poses, infos) == (poses, infos)
    for turn, ahead in ((20, True), (12, False)):
        h = math.radians(turn)
        drive = join(straight(1, 5, 5, 5), straight(5, 5, 5 + 2 * math.cos(h), 5 + 2 * math.sin(h)))
        route, poses, infos = corner_route(Polygon(rect(0, 0, 10, 10)), [(TRANSIT, drive)])
        out, _ = bodyfit._turn_at(route, poses, infos)
        assert (len(out) == len(poses) + 1) == ahead
        assert any(p == pytest.approx((5 - TURN_AHEAD, 5, 0.0)) for p in out) == ahead


def transit_corners(plan):
    """The poses of drives (ways round the body too) where the heading jumps by more than CORNER_ANGLE."""
    return [i for s in plan.segments if s.kind == TRANSIT for i in range(s.start + 1, s.end + 1)
            if angle_diff(plan.poses[i].yaw, plan.poses[i - 1].yaw) > CORNER_ANGLE]


def test_drives_rounded(monkeypatch):
    """With the body checked the corners of drives and of the ways round the body are arcs where the body fits on one:
    far fewer of them left, nothing sticking out (also against the real edges worked out here on their own, turning on
    the spot spin_margin further off), as well mowed. Where it still turns on the spot a pose TURN_AHEAD before the
    point. Without the rounding and that pose (body_fit off: the same loops and lanes otherwise), the loops and lanes
    are the same but for the ends a drive's corner was rounded into, turn_radius at most."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3))
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED])
    off = plan_coverage(GARDEN, [TREE, BED], dataclasses.replace(cfg, body_fit=False), start=(1, 1, 0.0), end=(1, 1))
    monkeypatch.setattr(planner, '_round_corners', lambda route, poses, infos: (poses, infos))
    monkeypatch.setattr(planner, '_turn_at', lambda route, poses, infos: (poses, infos))
    sharp = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0), end=(1, 1))
    assert len(transit_corners(plan)) < 0.5 * len(transit_corners(sharp))
    report = check_plan(plan)
    assert report['body_out'] == [] and report['left_m2'] <= check_plan(sharp)['left_m2'] + 0.05
    assert outside_room(plan, recorded_room(cfg, [GARDEN], [TREE, BED], [cfg.obstacle_margin] * 2), spin=0.05) == []
    spins = [(plan.poses[i - 1], plan.poses[i]) for i in range(1, len(plan.poses)) if i not in plan.section_starts
             and angle_diff(plan.poses[i].yaw, plan.poses[i - 1].yaw) > footprint.SPIN]
    assert spins and all(math.hypot(p.x - q.x, p.y - q.y) <= 2 * TURN_AHEAD + 1e-9 for q, p in spins)
    assert any(abs(math.hypot(p.x - q.x, p.y - q.y) - TURN_AHEAD) < 1e-9 for q, p in spins)
    before = plan_coverage(GARDEN, [TREE, BED], dataclasses.replace(cfg, body_fit=False), start=(1, 1, 0.0),
                           end=(1, 1))
    drives = [(p.x, p.y) for s in before.segments if s.kind == TRANSIT for p in before.poses[s.start:s.end + 1]]
    near = prep(MultiPoint(drives).buffer(cfg.turn_radius + 1e-6))
    planned = {(p.x, p.y) for p in before.poses}
    ahead = {i - 1 for i in range(1, len(off.poses)) if abs(math.hypot(off.poses[i].x - off.poses[i - 1].x,
                                                                         off.poses[i].y - off.poses[i - 1].y)
                                                              - TURN_AHEAD) < 1e-9}
    for one, other, poses in ((off, planned, before.poses), (before, {(p.x, p.y) for p in off.poses}, off.poses)):
        for s in one.segments:
            for i in range(s.start, s.end + 1):
                p = one.poses[i]
                if s.kind != TRANSIT and (p.x, p.y) not in other and not (one is off and i in ahead):
                    assert near.contains(Point(p.x, p.y)), (p, s.kind)


def test_narrow_parts_loops():
    """A winding path 0.5 to 0.8 m wide: narrower than the U-turns need, loops go on in there instead of lanes, the
    pieces they come apart into driven on the way round: shorter, as well mowed (but for what the body doesn't get to
    at the bends and the ends with the margin kept, turning there like on the spot, turning round before an end, and at
    the start, getting there from where it turns whatever way it faces, the same either way), hardly any drives
    between."""
    left = [(0.4 * math.sin(0.125 * i), 0.25 * i) for i in range(49)]
    path = left + [(x + 0.65 + 0.15 * math.sin(1.3 * y), y) for x, y in left][::-1]
    # (turning without spin_margin: with it the body hardly turns round in there, either way)
    kw = dict(MOWER, **NX60I, angle=math.pi / 2, perimeter_passes=3, spin_margin=0.0)
    lanes = plan_coverage(path, [], PlannerConfig(**kw))
    cfg = PlannerConfig(**dict(kw, narrow_parts='loops'))
    loops = plan_coverage(path, [], cfg)
    check_invariants(loops, cfg)
    # (the lanes' plan mows a little more on its drives, their corners rounded)
    assert coverage(loops, cfg) > max(0.885, coverage(lanes, cfg) - 0.01)
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
    # (left: the corners, the outer loop turns round them further in, spin_margin further off the edges, at a kink too,
    # and starting away from the corner by the dock, see route.START_CLEAR)
    assert plan.stats['perimeter_loops'] >= 5 and check_plan(plan)['left_m2'] < 1.6
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
    assert chosen['perimeter_passes'] == 4 and check_plan(plan)['left_m2'] < 1.15
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
    kept = config._BODY_SPACE.last
    cfg = dataclasses.replace(cfg, angle=0.5)
    again = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert config._BODY_SPACE.last is kept and again.body_space is not first.body_space
    del config._BODY_SPACE.last
    config._BODY_SPACES.clear()
    config._FITTED.clear()
    fresh = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert config._BODY_SPACE.last is not kept
    assert fresh.poses == again.poses and fresh.body_space.equals(first.body_space)
    other = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0), margins=[0.2])
    assert not other.body_space.equals(fresh.body_space)
    # (another spin_margin: the same space, another Footprint)
    kept, spaces = config._BODY_SPACE.last, len(config._BODY_SPACES)
    plan_coverage(lawn, [bed], dataclasses.replace(cfg, spin_margin=0.0), start=(1, 1, 0.0), margins=[0.2])
    assert config._BODY_SPACE.last[2].spin_margin == 0.0 and kept[2].spin_margin == cfg.spin_margin == 0.05
    assert config._BODY_SPACE.last[1].equals(kept[1]) and len(config._BODY_SPACES) == spaces
    done, worked_out = [], []
    driven = planner._driven
    monkeypatch.setattr(planner, '_driven', lambda *args: worked_out.append(args) or driven(*args))

    def plan():
        done.append((plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0)).poses, config._BODY_SPACE.last))

    threads = [threading.Thread(target=plan) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert [poses for poses, _ in done] == [fresh.poses] * 2 and not worked_out
    assert len({id(own) for _, own in done} | {id(config._BODY_SPACE.last)}) == 3
    # (another outline of the body: another space)
    rounded = plan_coverage(lawn, [bed], dataclasses.replace(cfg, robot_outline=ROUNDED), start=(1, 1, 0.0))
    assert not rounded.body_space.equals(fresh.body_space) and config._BODY_SPACE.last[2].corners == tuple(ROUNDED)


def test_body_fit_kept(monkeypatch):
    """Where _fit_body got to is kept every FIT_KEEP poses: a plan at another angle goes on from where its path (the
    loops) was still the same, the same plan as worked out afresh. Other settings of it start afresh."""
    monkeypatch.setattr(bodyfit, 'FIT_KEEP', 50)
    lawn, bed = rect(0, 0, 6, 4), rect(2.5, 1.5, 3.5, 2.5)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0, perimeter_passes=2))
    config._FITTED.clear()
    plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    (key, (_, _, _, first)), = config._FITTED.items()
    cfg = dataclasses.replace(cfg, angle=0.5)
    again = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    kept = config._FITTED[key][3]
    assert list(config._FITTED) == [key] and len(first) > 5 and kept[0] is first[0] and kept[-1] is not first[-1]
    config._FITTED.clear()
    fresh = plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    assert (fresh.poses, fresh.segments, fresh.stats['body_fit']) == (again.poses, again.segments,
                                                                     again.stats['body_fit'])
    monkeypatch.setattr(bodyfit, 'FIT_DRIVE', 0.3)
    plan_coverage(lawn, [bed], cfg, start=(1, 1, 0.0))
    plan_coverage(lawn, [bed], dataclasses.replace(cfg, spin_margin=0.0), start=(1, 1, 0.0))
    plan_coverage(lawn, [bed], dataclasses.replace(cfg, headland_corners='sharp'), start=(1, 1, 0.0))
    assert len(config._FITTED) == 4
    before = set(config._FITTED)
    plan_coverage(lawn, [bed], dataclasses.replace(cfg, robot_outline=ROUNDED), start=(1, 1, 0.0))
    assert set(config._FITTED) - before


def test_body_fit_kept_drive_on(monkeypatch):
    """Driving on reads the poses up to the section's end, the lanes too: a plan at another angle doesn't go on from
    where one before got to past a place it tried to drive on at (on a loop, nothing found: the path jumps there), it
    tries there again."""
    monkeypatch.setattr(bodyfit, 'FIT_KEEP', 50)
    monkeypatch.setattr(bodyfit, 'FIT_PLACE', (0, 0))  # (no other way tried, driving on finds none)
    monkeypatch.setattr(bodyfit, 'FIT_ROUND', 0)
    tried: List[List[tuple]] = []
    drive_to = _Route.drive_to

    def spy(self, target, heading, head=None, space=None, start=None):
        if start is not None and space is self.transit:  # (driving on)
            tried[-1].append(start[0])
        return drive_to(self, target, heading, head, space, start)

    monkeypatch.setattr(_Route, 'drive_to', spy)
    config._FITTED.clear()
    for angle in (0.0, 0.5):
        tried.append([])
        plan = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(MOWER, **NX60I, angle=angle)),
                             start=(1, 1, 0.0), end=(1, 1))
        assert plan.stats['body_fit']['jumps']
    assert tried[0] and tried[0][0] in tried[1]
    config._FITTED.clear()


def test_arriving_the_other_way_turns_into_it():
    """A drive that would arrive facing the other way: a U-turn into it from beside it, no turn on the spot."""
    space = TransitPlanner(Polygon(rect(0, 0, 10, 6)), 0.05)
    route = _Route(PlannerConfig(**FORWARD), space, space, (8.0, 3.0), math.pi, None)
    route.started = True
    poses = route.drive_to((2.0, 3.0), 0.0)
    assert (poses[-1].x, poses[-1].y) == pytest.approx((2.0, 3.0)) and _spins(poses, math.pi, 0.0) == 0
    assert all(space.is_free([(p.x, p.y)]) for p in poses)


def test_smooth_spins():
    """Without turn_on_spot turns on the spot by more than SPIN_LIMIT become an omega where there's room, never
    backing up."""
    def spins(plan):
        return sum(1 for a, b in zip(plan.poses, plan.poses[1:]) if angle_diff(a.yaw, b.yaw) > SPIN_LIMIT)

    plain = plan_coverage(GARDEN, [TREE, BED], PlannerConfig(**dict(FORWARD, turn_on_spot=True)), start=(1, 1, 0.0))
    cfg = PlannerConfig(**FORWARD)
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg, [TREE, BED])
    assert spins(plan) < spins(plain) / 2 and plan.stats['path_length'] > plain.stats['path_length']
    assert not any(s.reverse for s in plan.segments)


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

    # (without spin_margin: with it no turn on the spot between the lanes fits by the edges here)
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.3, spin_margin=0.0))
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


def test_one_turn_radius():
    """No turn, drive or loop out tighter than turn_radius (min_turn_radius is it): turning on the spot instead, but
    for a U-turn between lanes where none of it fits, which is rounded as wide as fits (lanes ending earlier for it).
    min_turn_radius and smooth_spins of before are left out of the settings."""
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

    cfg = PlannerConfig(**dict(FORWARD, turn_radius=0.2))
    assert cfg.min_turn_radius == 0.2
    plan = plan_coverage(GARDEN, [TREE, BED], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg, [TREE, BED])
    # the loops' corners keep their rounding, so count only what the turns and loops out make
    turns = [s for s in plan.segments if s.kind == TURN]  # (a drive's corner may get a tighter arc, _round_corners)
    tight = [s for s in turns if any(r <= 0.2 - 0.02 for r in arcs(
        dataclasses.replace(plan, poses=plan.poses[s.start:s.end + 1])))]
    assert turns and all(s.maneuver == 'u_turn' for s in tight)
    assert cfg.with_overrides({'min_turn_radius': 0.4, 'smooth_spins': False, 'allow_reverse': True,
                               'waypoint_spacing': 0.5}) == cfg


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
        turn = plan_turn(exit_pose, entry, 0.3, 0.05, free, None, ('u_turn', 'pivot'), least=0.3, spin=spin)
        assert turn.name == 'u_turn'
        assert max(p.x for p in turn.pieces[0].poses) == pytest.approx(0.135, abs=0.01)
    room = lambda pts: all(x <= 0.05 for x, _ in pts)  # noqa: E731 - a wall 5 cm ahead
    assert plan_turn(exit_pose, entry, 0.3, 0.05, room, None, ('u_turn', 'pivot'), least=0.3).name == 'pivot'
    assert plan_turn(exit_pose, entry, 0.3, 0.05, room, None, ('u_turn', 'pivot'), least=0.3, spin=False) is None
    assert plan_turn(exit_pose, entry, 0.3, 0.05, free, None, ('u_turn',), least=0.3) is None



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
    cfg = PlannerConfig(**dict(FORWARD, turn_types=('u_turn', 'pivot')))

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
    cfg = PlannerConfig(**dict(FORWARD))

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
    cfg = PlannerConfig(**dict(FORWARD, turn_types=('u_turn', 'pivot')))
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
    cfg = PlannerConfig(**dict(MOWER, **NX60I, edges='hard', perimeter_passes=3,
                               lane_overlap_passes=2, overlap=1 - 0.153 / 0.18, narrow_parts='loops', angle=0.6,
                               obstacle_margin=0.05))
    plan = plan_coverage(shape, [circle(3, 3, 0.8, 16), circle(6, 4.5, 0.6, 12)], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg)
    poses = plan.poses
    starts = [s.start for s in plan.segments if s.kind == LANE and s.start]
    assert not [i for i in starts if angle_diff(poses[i - 1].yaw, poses[i].yaw) > SPIN_LIMIT]
    # (and the corners of the walls the body doesn't get into without backing up: the loops leave them out, and the
    # turns keep spin_margin further off the walls; the outer loop starting away from the corner by the dock)
    report = check_plan(plan)
    assert report['left_m2'] < 1.45 and report['body_out'] == []


def test_bent_lane_entered_on_its_first_piece():
    """A lane bending round an obstacle starts later only on its first straight piece, not behind it."""
    space = TransitPlanner(Polygon(rect(0, 0, 10, 4)).difference(Polygon(rect(0.62, 1.85, 0.95, 2.15))))
    cfg = PlannerConfig(**dict(FORWARD))
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
    cfg = PlannerConfig(**dict(MOWER, **NX60I, edges='hard', perimeter_passes=3,
                               lane_overlap_passes=2, overlap=1 - 0.153 / 0.18, narrow_parts='loops', angle=0.0))
    plan = plan_coverage(kite, [], cfg, start=(1, 1, 0.0))
    check_invariants(plan, cfg)
    st = plan.stats
    assert st['turns_fallback'] <= 1 and st['turns_pivot'] == 0 and st['turns_bulb'] > 0
    # (and the corners of the walls the body doesn't get into without backing up: the loops turn round them further
    # in)
    report = check_plan(plan)
    assert report['left_m2'] < 1.3 and report['body_out'] == []

def test_body_corner_rounding():
    """A loop 0.13 m in from a recorded edge: the body doesn't fit round its corner with the corner radius, it does
    with a larger one."""
    fp = Footprint(Polygon(rect(0, 0, 10, 10)).buffer(0.21, 8, join_style=2), BODY)
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
        cfg = PlannerConfig(**dict(FORWARD, transit_edge_distance=d, transit_clearance=0.05))
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
    assert plan.stats['transit_length'] < 0.65 * plain.stats['transit_length']
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
    assert plan_turn(exit_pose, entry, 0.25, 0.05, headland(5.0)).name == 'bulb'
    assert plan_turn(exit_pose, entry, 0.25, 0.05, headland(5.0), types=('pivot', 'bulb')).name == 'bulb'
    # a larger radius wins over the order: a smaller bulb before turning on the spot, whatever the list says
    m = plan_turn(exit_pose, entry, 0.25, 0.05, headland(0.26), types=('bulb', 'pivot'))
    assert m.name == 'bulb' and m.radius < 0.25
    # nothing allowed fits
    assert plan_turn(exit_pose, entry, 0.25, 0.05, headland(0.01), types=('bulb',)) is None
    assert PlannerConfig().allowed_turns == ('u_turn', 'bulb', 'detour', 'pivot')
    assert PlannerConfig(turn_on_spot=True).allowed_turns == ('u_turn', 'detour', 'pivot')


@pytest.mark.parametrize('kw, used', [
    (dict(turn_types=('bulb', 'pivot'), perimeter_passes=4), 'bulb'),
    (dict(turn_types=('pivot',)), 'pivot'),
    (dict(perimeter_passes=4), 'bulb'),
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
    assert base.with_overrides({'turn_types': ['k_turn', 'bulb']}).turn_types == ('bulb',)  # (no setting any more)
    other = base.with_overrides({'perimeter_offset': 0, 'angle_min': -1, 'angle_max': 1, 'allow_reverse': True})
    assert isinstance(other.perimeter_offset, float) and isinstance(other.angle_min, float)
    assert not hasattr(other, 'allow_reverse')  # (gone, a stored one of before is left out)
    assert base.with_overrides({}) == base
    with pytest.raises(PlanningError, match='blade_speed, colour'):
        base.with_overrides({'colour': 'green', 'blade_speed': 3000, 'overlap': 0.1})
    cfg.validate()


@pytest.mark.parametrize('kw, end', [
    (dict(fill_pattern='crosshatch'), None),
    (dict(fill_pattern='concentric'), None),
    (dict(perimeter_order='last', perimeter_direction='cw'), (-0.3, 3.0)),
    (dict(angle_strategy='optimal', angle_min=0.1, angle_max=0.5), None),
    (dict(turn_types=('bulb', 'pivot'), lane_order='skip'), (6.0, -0.5)),
    (dict(MOWER, **NX60I), (1.0, 1.0)),
    (dict(MOWER, robot_outline=ROUNDED), (1.0, 1.0)),
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
        assert plan.warnings == NO_BODY
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


def test_transit_keeps_its_clearance_where_there_is_room():
    """careful_path() (the drives) keeps the clearance off a corner where that's not a lot longer, path() (detours,
    bent lanes) NARROW_CLEARANCE; through a passage narrower than twice the clearance careful_path() still goes."""
    block = Polygon(rect(4, 0, 6, 2.5))
    tp = TransitPlanner(Polygon(rect(0, 0, 10, 5)).difference(block), clearance=0.15)
    short, careful = tp.path((1, 1), (9, 1)), tp.careful_path((1, 1), (9, 1))
    assert 0.04 < LineString(short).distance(block) < 0.06
    assert 0.14 < LineString(careful).distance(block) < 0.16
    assert LineString(careful).length < LineString(short).length + 0.3
    space = Polygon(rect(0, 0, 10, 4)).difference(Polygon(rect(4.9, -1, 5.1, 1.9))).difference(
        Polygon(rect(4.9, 2.1, 5.1, 5)))
    way = TransitPlanner(space, clearance=0.15).careful_path((4.0, 1.0), (6.0, 1.0))
    assert way is not None and LineString(way).length < 3.0  # through the door 20 cm wide


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
    assert plan.section_starts == [0] and plan.warnings == NO_BODY
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


def test_no_body_check_is_a_warning():
    """Without the mower's size, or with only a part of it, the body isn't checked: every plan says so first."""
    lawn = rect(0, 0, 6, 4)
    assert plan_coverage(lawn, [], PlannerConfig()).warnings == NO_BODY
    assert plan_coverage(lawn, [], PlannerConfig(robot_front=0.43, robot_rear=0.14)).warnings == [
        'no body check: robot_front, robot_rear set but not robot_width (it takes all three, or robot_outline)']
    for body in (NX60I, {'robot_outline': BODY}):
        cfg = PlannerConfig(**body)
        assert cfg.no_body is None
        assert not any('no body check' in w for w in plan_coverage(lawn, [], cfg).warnings)


def test_settings_at_their_limits():
    """Settings at the limits validate() gives them plan, without a crash: no loops and narrow_parts loops with a part
    too narrow for lanes, a body that ends at base_link (robot_rear or robot_front 0), turn_radius 0 with the body
    checked (no rounded way of no radius: every pose faces the way it drives on). Sizes in mm and no waypoint_spacing
    with the body checked are refused."""
    hourglass = [(0, 0), (4, 1.75), (8, 0), (8, 4), (4, 2.25), (0, 4)]
    cfg = PlannerConfig(perimeter_passes=0, narrow_parts='loops')
    check_invariants(plan_coverage(hourglass, [], cfg), cfg)
    for more in ({'robot_rear': 0.0}, {'robot_front': 0.0}, {'turn_radius': 0.0}):
        cfg = PlannerConfig(**dict(MOWER, **dict(NX60I, **more)))
        plan = plan_coverage(rect(0, 0, 10, 6), [], cfg, start=(1, 1, 0.0), end=(1, 1))
        assert check_plan(plan)['body_out'] == []
        jumps = {s - 1 for s in plan.section_starts[1:]}
        assert not [i for i, (a, b) in enumerate(zip(plan.poses, plan.poses[1:]))
                    if i not in jumps and math.hypot(b.x - a.x, b.y - a.y) > 1e-6 and
                    angle_diff(math.atan2(b.y - a.y, b.x - a.x), a.yaw) > 6.5 * DEG], more
    for bad in (dict(NX60I, robot_width=400.0), dict(NX60I, robot_front=430.0), dict(NX60I, waypoint_spacing=0.0),
                {'robot_outline': [(430, 200), (-140, 200), (-140, -200), (430, -200)]}):
        with pytest.raises(PlanningError):
            PlannerConfig(**bad).validate()


def test_mowing_passes_stay_on_the_lawn():
    """The passes over what a mow left (and mop_up's) stay in the mowing area, a drivable area next to it (another
    lawn, a navigation area) is for the ways between them only."""
    lawn = [(0, 0), (14, 0), (14, 10), (10, 10), (10, 3), (4, 3), (4, 10), (0, 10)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    # (a mow that left a strip across the left arm, up to its edge at the opening the drivable area lies in)
    mowed = Polygon(rect(-1, -1, 15, 11)).difference(Polygon(rect(1.9, 7.8, 4.5, 8.2)))
    plan = plan_coverage(lawn, [], cfg, start=(1, 1, 0.0), end=(1, 1), drivable=[rect(3.5, 2.5, 10.5, 11)],
                         mowed=mowed)
    inside = prep(plan.robot_space.buffer(2e-3))
    passes = [s for s in plan.segments if s.kind == LANE]
    assert passes and all(inside.covers(Point(p.x, p.y)) for s in passes for p in plan.poses[s.start:s.end + 1])


def test_island_loops_without_outline_loops():
    """Loops round the islands but none along the outline (perimeter_passes 0, obstacle_passes 1): an island gets its
    loop where there's room round it, none where it lies so close to the edge that the loop would go back along the
    island itself."""
    cfg = PlannerConfig(perimeter_passes=0, obstacle_passes=1)
    for bed, looped in ((rect(3, 2, 5, 3.5), True), (rect(3, 0.13, 5, 1.63), False)):
        plan = plan_coverage(rect(0, 0, 8, 6), [bed], cfg, start=(1, 1, 0.0))
        check_invariants(plan, cfg, [bed])
        assert any(s.kind == PERIMETER for s in plan.segments) is looped
