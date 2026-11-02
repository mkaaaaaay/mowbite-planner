"""What every plan keeps to, whatever the shape: the tests of the planner and the invariants over the matrix
(test_invariants.py) check their plans with it."""

import math

from shapely.geometry import MultiPoint, Point, Polygon
from shapely.prepared import prep

from mowbite_planner.backing import CORNER_STEP
from mowbite_planner.config import LANE, TRANSIT
from mowbite_planner.mathutil import angle_diff

DEG = math.pi / 180.0


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
    # without the body no obstacle closer than the first loop's offset (minus what simplifying the input may move; with
    # the body it's the body that keeps off them, the measuring tools' rules)
    cloud = MultiPoint([(p.x, p.y) for p in poses])
    for o in obstacles if not cfg.body else ():
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
    # every pose faces the way it's driven to the next one, never backwards (drive slope or reverse: against it in the
    # segments driven backwards)
    back = [False] * len(poses)
    for s in segs:
        back[s.start:s.end] = [s.reverse] * (s.end - s.start)
    assert plan.config.drive != 'forward' or not any(back)
    for i, (a, b) in enumerate(zip(poses, poses[1:])):
        if i in jumps:
            continue
        travel = math.atan2(b.y - a.y, b.x - a.x) + (math.pi if back[i] else 0.0)
        if back[i] and math.hypot(b.x - a.x, b.y - a.y) < 2 * CORNER_STEP and i:
            # (a sharp corner's turn pose backing up, backing.corners: facing against the ways in and out)
            before = math.atan2(a.y - poses[i - 1].y, a.x - poses[i - 1].x) + math.pi
            assert angle_diff(travel, a.yaw) < math.pi / 2 and angle_diff(before, a.yaw) < math.pi / 2, i
            continue
        assert angle_diff(travel, a.yaw) <= arc_tol, f'pose {i} {a} faces {a.yaw}, drives {travel}'
    # straight lanes keep exactly one heading, along one of the sets of lanes (concentric rings bend)
    for s in segs:
        if s.kind == LANE and s.maneuver not in ('ring', 'bend', 'mop_up', 'leftover'):
            yaws = {poses[i].yaw for i in range(s.start, s.end)}
            assert len(yaws) == 1
            yaw = yaws.pop()
            # doubled, a and a + pi are the same lanes
            assert min(angle_diff(2 * yaw, 2 * a) for a in plan.angles) < 1e-9
