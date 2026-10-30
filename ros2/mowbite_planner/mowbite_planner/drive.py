"""A drive through a space to a point, arriving facing a heading, on arcs where they fit, the body checked."""

import math
from typing import List, Optional, Sequence

from shapely.geometry import LineString

from .config import SPIN_LIMIT, XY, PlannerConfig
from .footprint import Footprint
from .maneuvers import connect, join, path_length, plan_turn, same_position
from .mathutil import Pose2D, angle_diff, normalize_angle
from .transit import TransitPlanner


class Drive:
    """Drives through space (shortest ways, is_free) with the body checked where there's a footprint."""

    def __init__(self, space: TransitPlanner, footprint: Optional[Footprint], cfg: PlannerConfig):
        self.space, self.cfg = space, cfg
        self.body = footprint.poses_fit if footprint is not None else None

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
