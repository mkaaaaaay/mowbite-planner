"""The lanes: their sets across the area, and each as the centre drives it, its ends pulled back for the body."""

import dataclasses
import math
from typing import Callable, Dict, List, Optional, Tuple

from .config import XY, Lane, PlannerConfig
from .footprint import Footprint
from .geometry import polygonal, union
from .lanes import scan_lanes, split_cells, zone_angles
from .maneuvers import sample_arc
from .mathutil import Pose2D, normalize_angle
from .transit import TransitPlanner

# m, lane ends are pulled back to this when the body doesn't fit at them
BODY_STEP = 0.01

Passes = List[Tuple[float, List[List[Lane]]]]


@dataclasses.dataclass(frozen=True)
class Swath:
    """A lane as the centre drives it: from a to b facing yaw."""
    a: XY
    b: XY
    yaw: float


def lane_sets(lane_region, remaining, robot_space, angle: float, cfg: PlannerConfig) -> Tuple[Passes, list]:
    """The sets of lanes [(angle, cells)], for crosshatch a second across, with split_zones the zones mowed another
    way each a set of its own (the rest's lanes end where a zone begins), and the zones [(angle, zone)]."""
    half, spacing = 0.5 * cfg.mower_width, cfg.lane_spacing
    angles = [angle]
    zones: list = []
    if cfg.fill_pattern == 'crosshatch':
        second = math.fmod(angle + cfg.crosshatch_angle, math.pi)
        angles.append(second + math.pi if second < 0 else second)
    elif cfg.split_zones and not cfg.headland_turns:
        zones = zone_angles(lane_region, angle, spacing, cfg.min_lane_length, remaining, robot_space, half,
                            cfg.angle_min, cfg.angle_max)
    taken = union(zone for _, zone in zones)
    todo = [(a, lane_region, remaining) for a in angles] if not zones else \
        [(angle, polygonal(lane_region.difference(taken)),
          polygonal(remaining.difference(taken.buffer(spacing, 4))))] + \
        [(a, zone, polygonal(remaining.intersection(zone.buffer(spacing, 4)))) for a, zone in zones]
    return [(a, split_cells(scan_lanes(where, a, spacing, cfg.min_lane_length, mow, robot_space, half)))
            for a, where, mow in todo], zones


def to_map(angle: float) -> Callable[[float, float], XY]:
    """From the frame of lanes at angle into the map."""
    c, s = math.cos(angle), math.sin(angle)
    return lambda x, y: (x * c - y * s, x * s + y * c)


def centre_line(lane: Lane, direction: int, angle: float, cfg: PlannerConfig, area: TransitPlanner,
                count: Callable[[str], None]) -> Optional[Swath]:
    """Where the centre drives lane going +x (direction 1) or -x: blade_offset to the blade's right, blade_ahead behind
    it, shortened to what the centre may drive where that takes it out. None where too little is left."""
    xa, xb = (lane.x0, lane.x1) if direction > 0 else (lane.x1, lane.x0)
    y = lane.y - cfg.blade_offset * direction
    back = cfg.blade_ahead * direction
    m = to_map(angle)
    a, b = m(xa - back, y), m(xb - back, y)
    if (cfg.blade_offset != 0 or back != 0) and not area.is_free([a, b]):
        count('lanes_shortened')
        piece = area.clip(a, b, cfg.min_lane_length)
        if piece is None:
            return None
        a, b = piece
    return Swath(a, b, normalize_angle(angle if direction > 0 else angle + math.pi))


def body_ends(fp: Footprint, cfg: PlannerConfig, s: Swath, arrive: float, leave: float,
              count: Callable[[str], None]) -> Optional[Swath]:
    """s pulled back at its ends until the body fits there, and on the quarter arcs of turn_radius the turns arrive and
    leave on: arrive, leave 1 from or to the left, -1 the right, 0 no turn. None if less than min_lane_length is
    left."""
    a, b, yaw = s.a, s.b, s.yaw
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    ux, uy = math.cos(yaw), math.sin(yaw)
    r, quarter = cfg.turn_radius, 0.5 * math.pi
    known: Dict[float, bool] = {}

    def fits(t: float) -> bool:
        if t not in known:
            known[t] = fits_at(t)
        return known[t]

    def fits_at(t: float) -> bool:
        x, y = a[0] + t * ux, a[1] + t * uy
        if not fp.fits_pose(x, y, yaw):
            return False
        if r <= 1e-3:
            return True
        # leaving at the end, arriving at the start (worked out backwards, facing the other way)
        if t > 0.5 * length:
            return not leave or fp.poses_fit(sample_arc(x, y, yaw, r, leave * quarter, cfg.waypoint_spacing), True)
        if not arrive:
            return True
        back = sample_arc(x, y, yaw + math.pi, r, -arrive * quarter, cfg.waypoint_spacing)
        return fp.poses_fit([Pose2D(p.x, p.y, p.yaw + math.pi) for p in back], True)

    def pose_fits(t: float) -> bool:
        return fp.fits_pose(a[0] + t * ux, a[1] + t * uy, yaw)

    t0, t1 = 0.0, length
    if fits(t0) and fits(t1):
        return s
    # the body and a turn reach no further than this past the lane's end: where the turn's arc doesn't fit that far
    # in either, pulling the lane back further makes no room for it (an edge all along the lane, the turn towards
    # it), a lane longer than twice that goes on as far as the body fits and the turn finds another way (a shorter
    # one is pulled back to where it fits, or left out: no turn that doesn't fit for a bit of lane)
    mid = 0.5 * length
    near = min(mid, 2.0 * (fp.reach + cfg.turn_radius))
    if not (pose_fits(mid) if near < mid else fits(mid)):
        t0 = t1 = mid
    else:
        t0 = t0 if fits(t0) else _edge(near, t0, fits) if fits(near) else \
            t0 if pose_fits(t0) else _edge(mid, t0, pose_fits)
        t1 = t1 if fits(t1) else _edge(length - near, t1, fits) if fits(length - near) else \
            t1 if pose_fits(t1) else _edge(mid, t1, pose_fits)
    count('lanes_body')
    if t1 - t0 < max(cfg.min_lane_length, 1e-6):
        return None
    return Swath((a[0] + t0 * ux, a[1] + t0 * uy), (a[0] + t1 * ux, a[1] + t1 * uy), yaw)


def _edge(inside: float, outside: float, fits: Callable[[float], bool]) -> float:
    """Where between a point that fits and one that doesn't it stops fitting, the side that fits (halving)."""
    while abs(outside - inside) > BODY_STEP:
        t = 0.5 * (inside + outside)
        inside, outside = (t, outside) if fits(t) else (inside, t)
    return inside
