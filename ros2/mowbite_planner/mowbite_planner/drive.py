"""A drive through a space to a point, arriving facing a heading, on arcs where they fit, the body checked; whether a
drive keeps spin_margin where it turns as a turn does (keeps_margin)."""

import math
from typing import List, Optional, Sequence

import numpy
from shapely.geometry import LineString

from .config import SPIN_LIMIT, XY, PlannerConfig
from .footprint import Footprint
from .maneuvers import connect, join, path_length, plan_turn, same_position, sample_straight
from .mathutil import Pose2D, angle_diff, normalize_angle
from .transit import TransitPlanner


TURN_DEG, TURN_WIN = math.radians(90.0), 0.3  # a pose on a turn: the way turns this much within this much each side
EPS = 1e-9  # (a corner of exactly 90 deg is one, however the headings add up)
BODY_OUT = 0.01  # m the body may stick out there (the ranking's rule, design 2.4)
# poses of the path before a drive its check looks back at (turning: TURN_WIN each side)
MARGIN_BEFORE = 6


def turning(poses: Sequence[Pose2D]) -> numpy.ndarray:
    """Which poses lie on a turn, from the way alone: it turns TURN_DEG at least within TURN_WIN before and after."""
    if not poses:
        return numpy.zeros(0, bool)
    xy = numpy.array([(p.x, p.y) for p in poses])
    s = numpy.concatenate([[0.0], numpy.cumsum(numpy.hypot(*numpy.diff(xy, axis=0).T))])
    yaw = numpy.array([p.yaw for p in poses])
    h = numpy.concatenate([[yaw[0]], yaw[0] + numpy.cumsum(numpy.remainder(numpy.diff(yaw) + math.pi, 2.0 * math.pi)
                                                           - math.pi)])
    lo = numpy.searchsorted(s, s - TURN_WIN - EPS, 'left')
    hi = numpy.searchsorted(s, s + TURN_WIN + EPS, 'right')
    return numpy.array([h[a:b].max() - h[a:b].min() >= TURN_DEG - EPS for a, b in zip(lo, hi)], bool)


def keeps_margin(fp: Optional[Footprint], step: float, before: Sequence[Pose2D], drive: List[Pose2D],
                 yaw: Optional[float]) -> bool:
    """Whether the body keeps spin_margin (but BODY_OUT) on drive where it turns as a turn does, found from the way
    with the poses before and straight on from its end facing yaw (turning)."""
    if fp is None or fp.turning is fp or not drive:
        return True
    end = drive[-1]
    after = sample_straight(end.x, end.y, end.x + TURN_WIN * math.cos(yaw), end.y + TURN_WIN * math.sin(yaw), yaw,
                            step)[1:] if yaw is not None else []
    window = list(before) + list(drive) + after
    turn = turning(window).tolist()
    lo, hi = len(before), len(before) + len(drive)
    bad = [k for k in fp.bad_poses(window, turn) if lo <= k < hi]
    out = fp.out_by([(window[k - 1] if k else None, window[k]) for k in bad], on_turn=[turn[k] for k in bad]) \
        if bad else []
    return not any(xy is not None and d > BODY_OUT for d, xy in out)


class _OnTurns:
    """A footprint's check with spin_margin kept where the way turns as a turn does (turning)."""

    def __init__(self, fp: Footprint):
        self.fp = fp

    def poses_fit(self, poses: Sequence[Pose2D]) -> bool:
        return self.fp.poses_fit(poses, turning(poses).tolist())


class Drive:
    """Drives through space (shortest ways, is_free) with the body checked where there's a footprint."""

    def __init__(self, space: TransitPlanner, footprint: Optional[Footprint], cfg: PlannerConfig,
                 margin: bool = False):
        # (margin: spin_margin kept where it turns as a turn does, see keeps_margin)
        self.space, self.cfg = space, cfg
        self.body = (_OnTurns(footprint).poses_fit if margin else footprint.poses_fit) if footprint is not None else None

    def way(self, pos: XY, facing: Optional[float], target: XY, heading: Optional[float]) -> Optional[List[Pose2D]]:
        """From pos (facing that way) to target, arriving facing heading, on arcs where they fit (see _Route.drive_to);
        None where there's no way."""
        cfg, space, body = self.cfg, self.space, self.body
        r = cfg.turn_radius
        step = cfg.waypoint_spacing

        def drive(b: XY, h: Optional[float]) -> Optional[List[Pose2D]]:
            return connect(pos, facing, b, h, r, step, space.is_free, space.careful_path, body, cfg.min_turn_radius)

        best = drive(target, heading)
        if best is not None and cfg.turn_on_spot:
            # (on the spot where it leaves and arrives where the arcs there make it cross its own way, or turn it round
            # more than half a turn all together where on the spot they turn it a quarter turn less: no pirouettes)
            crosses = len(best) > 3 and not LineString([(p.x, p.y) for p in best]).is_simple
            if crosses or _arcs(best) > math.pi:
                spot = connect(pos, None, target, None, r, step, space.is_free, space.careful_path, body,
                               cfg.min_turn_radius)
                if spot is not None and (crosses or _arcs(spot) < _arcs(best) - 0.5 * math.pi):
                    best = spot
            return best
        if best is None or heading is None or r <= 1e-3:
            return best
        best_spins = _spins(best, facing, heading)
        if not best_spins:
            return best
        nx, ny = -math.sin(heading), math.cos(heading)
        back = normalize_angle(heading + math.pi)
        for d in (2.0 * r, r):
            for side in (1.0, -1.0):
                x = (target[0] + side * d * nx, target[1] + side * d * ny)
                if not space.is_free([x]):
                    continue
                turn = plan_turn(Pose2D(x[0], x[1], back), target, r, step, space.is_free, None,
                                 tuple(t for t in cfg.allowed_turns if t in ('u_turn', 'bulb')), body,
                                 cfg.min_turn_radius)
                way = drive(x, back) if turn is not None else None
                if way is None:
                    continue
                poses = join(way, turn.pieces[0].poses)
                spins = _spins(poses, facing, heading)
                if spins < best_spins or spins == best_spins and path_length(poses) < path_length(best):
                    best, best_spins = poses, spins
        return best


def _spins(poses: Sequence[Pose2D], heading_in: Optional[float], heading_out: Optional[float]) -> int:
    """How often the mower turns on the spot by more than SPIN_LIMIT driving poses, coming in facing heading_in and
    going on facing heading_out."""
    yaws = ([heading_in] if heading_in is not None else []) + [p.yaw for p in poses] + \
        ([heading_out] if heading_out is not None else [])
    return sum(1 for a, b in zip(yaws, yaws[1:]) if angle_diff(a, b) > SPIN_LIMIT)


def _arcs(poses: Sequence[Pose2D]) -> float:
    """rad the way turns on its arcs all together (not on the spot)."""
    return sum(angle_diff(a.yaw, b.yaw) for a, b in zip(poses, poses[1:]) if not same_position(a, b))
