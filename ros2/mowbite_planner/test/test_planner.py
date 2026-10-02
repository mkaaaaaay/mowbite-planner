"""Tests of the planner without ROS (python3 -m pytest test/ in the package folder, shapely is enough)."""

import math
import os
import random
import sys
import time
from types import SimpleNamespace

import pytest
from shapely.geometry import LineString, MultiPoint, Point, Polygon
from shapely.ops import unary_union
from shapely.prepared import prep

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import (PlannerConfig, PlanningError, Pose2D, normalize_angle,  # noqa: E402
                             plan_coverage, quaternion_to_yaw, swept_area, yaw_to_quaternion)
from mowbite_planner.maneuvers import bulb_turn, drive_polyline, k_turn, plan_turn, u_turn  # noqa: E402
from mowbite_planner.mathutil import angle_diff  # noqa: E402
from mowbite_planner.planner import (LANE, PERIMETER, TRANSIT, TURN, Lane, _lane_cost, _Route,  # noqa: E402
                                     angle_in_range, best_order, longest_edge_angle, scan_lanes, split_cells)
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
        assert (drivable if i in in_transit else inside).covers(Point(p.x, p.y)), f'pose {p} outside the free space'
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
    # every pose faces the way it's driven to the next one, backwards only on the reverse legs of K-turns
    backwards = [False] * (len(poses) - 1)
    for s in segs:
        assert not s.reverse or (s.kind == TURN and s.maneuver == 'k_turn')
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
        if s.kind == LANE and s.maneuver not in ('ring', 'bend'):
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
    centre keeps off it like off an obstacle, nor does the body reach in."""
    wall = rect(4.9, -1, 5.1, 7)
    keep = rect(3, 6.5, 7, 8)  # across the lawn's near half
    cfg = PlannerConfig(**dict(MOWER, **NX60I))
    free = plan_coverage(rect(0, 0, 10, 6), [wall], cfg, drivable=[rect(1, 4, 9, 11)])
    assert free.section_starts == [0] and any(6.5 < p.y < 8 and 3 < p.x < 7 for p in free.poses)
    plan = plan_coverage(rect(0, 0, 10, 6), [wall], cfg, drivable=[rect(1, 4, 9, 11)], keep_out=[keep])
    check_invariants(plan, cfg, [wall])
    assert plan.section_starts == [0] and max(p.y for p in plan.poses) > 8.0  # round it, further out
    away = Polygon(keep).buffer(cfg.centre_offset - 1e-6)
    assert not any(away.contains(Point(p.x, p.y)) for p in plan.poses)
    assert plan.body_space.intersection(Polygon(keep)).area < 1e-6


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
    (rect(0, 0, 5, 5), [], dict(perimeter_passes=-1)),
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
    loops = [s for s in plan.segments if s.kind == PERIMETER or s.maneuver == 'ring']
    assert len(loops) >= 6
    sign = 1 if direction == 'ccw' else -1
    for s in loops:
        assert sign * signed_area(plan.poses[s.start:s.end + 1]) > 0


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


def test_body_turns_and_lane_ends_stay_inside():
    """The turns fit with the body: lane ends pulled back for the U-turns, no turn on the spot or replaced by a
    drive, and the body nowhere it wasn't when the edges were recorded (driving along them, turning at the corners)
    but for a few poses where the outer loop, its centre on the line (perimeter_offset 0), rounds a corner."""
    cfg = PlannerConfig(**dict(MOWER, **NX60I, angle=0.0))
    plan = plan_coverage(rect(0, 0, 10, 6), [], cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg)
    plain = plan_coverage(rect(0, 0, 10, 6), [], PlannerConfig(**dict(MOWER, angle=0.0)), start=(1, 1, 0.0),
                          end=(1, 1))
    assert plan.stats['turns_u_turn'] == plain.stats['turns_u_turn'] + plain.stats['turns_pivot'] > 0
    assert plan.stats['turns_pivot'] == plan.stats['turns_fallback'] == plan.stats['turns_detour'] == 0
    out = check_plan(plan)['body_out']
    assert len(out) <= 12 and {o['kind'] for o in out} <= {'loop', 'spin'}
    edge = Polygon(rect(0, 0, 10, 6)).exterior
    assert all(edge.distance(Point(o['x'], o['y'])) < 0.05 for o in out)  # (its corner arcs a few cm in)
    # the lanes end where the U-turn's arc keeps the front in: a turn of 0.25 reaches 0.62 past the end, the body
    # may go 0.21 past the edge
    ends = [x for a, b in plan.lanes for x in (a[0], b[0])]
    assert max(ends) <= 10 + 0.21 - math.hypot(0.25 + 0.2, 0.43) + 0.02
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
        assert {'x', 'y', 'yaw', 'kind'} == set(report['body_out'][0])
        assert any('body sticks out' in h for h in report['hard'])
        # left: the corners the body keeps the mower out of, and the lane starts of the turns
        assert report['left_m2'] < (0.5 if edges == 'recorded' else 1.5)
        assert sum(o['kind'] in ('turn', 'lane_end') for o in report['body_out']) == 0


def test_body_bent_lanes():
    """Lanes bending around trees get their ends pulled back too: U-turns, nothing replaced by a drive."""
    trees = [circle(4, 4, 0.4), circle(9, 10, 0.5)]
    cfg = PlannerConfig(**dict(MOWER, **NX60I, bend_max_gap=3.0))
    plan = plan_coverage(GARDEN, [TREE, BED] + trees, cfg, start=(1, 1, 0.0), end=(1, 1))
    check_invariants(plan, cfg, [TREE, BED] + trees)
    assert any(s.maneuver == 'bend' for s in plan.segments)
    assert plan.stats['turns_fallback'] == 0 and plan.stats['turns_pivot'] <= 2
    assert not [o for o in check_plan(plan)['body_out'] if o['kind'] in ('turn', 'lane_end')]


def test_narrow_parts_loops():
    """A winding path 0.5 to 0.8 m wide: narrower than the U-turns need, loops go on in there instead of lanes, the
    pieces they come apart into driven on the way round: shorter, as well mowed, hardly any drives between."""
    left = [(0.4 * math.sin(0.125 * i), 0.25 * i) for i in range(49)]
    path = left + [(x + 0.65 + 0.15 * math.sin(1.3 * y), y) for x, y in left][::-1]
    kw = dict(MOWER, **NX60I, angle=math.pi / 2, perimeter_passes=3)
    lanes = plan_coverage(path, [], PlannerConfig(**kw))
    cfg = PlannerConfig(**dict(kw, narrow_parts='loops'))
    loops = plan_coverage(path, [], cfg)
    check_invariants(loops, cfg)
    assert coverage(loops, cfg) > 0.99
    assert loops.stats['path_length'] < 0.85 * lanes.stats['path_length']
    assert loops.stats['transit_length'] < 2.0 and loops.stats['lanes'] == 0
    # a wide area keeps its lanes
    wide = plan_coverage(rect(0, 0, 10, 6), [], cfg)
    assert wide.stats['lanes'] > 10
    with pytest.raises(PlanningError, match='narrow_parts'):
        PlannerConfig(narrow_parts='strips').validate()


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
])
def test_deterministic_new_settings(kw, end):
    cfg = PlannerConfig(**kw)
    a = plan_coverage(GARDEN, [TREE, BED], cfg, end=end)
    b = plan_coverage(list(GARDEN), [list(TREE), list(BED)], PlannerConfig(**kw), end=end)
    assert a.poses == b.poses and a.segments == b.segments


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
    assert rings
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
