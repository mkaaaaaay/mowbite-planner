"""Where the blade mows along poses, and where a plan's body sticks out."""

import math
from typing import Any, Dict, List, Optional, Sequence

from shapely.geometry import LineString, MultiLineString, Polygon

from .config import PERIMETER, SWEPT_PIECE, TRANSIT, TURN, XY, CoveragePlan, PlannerConfig, SegmentInfo
from .footprint import Footprint
from .geometry import union
from .mathutil import Pose2D


def blade(cfg: PlannerConfig, *runs: Sequence[Pose2D], whole: bool = False):
    """Where the blade mows driving poses (each run of them). whole: the whole path (quicker in pieces, see
    swept_area)."""
    half, ahead, off = 0.5 * cfg.mower_width, cfg.blade_ahead, cfg.blade_offset
    lines = [[(p.x + ahead * math.cos(p.yaw) - off * math.sin(p.yaw),
               p.y + ahead * math.sin(p.yaw) + off * math.cos(p.yaw)) for p in poses] for poses in runs]
    if whole:
        # (straight runs of poses one straight line each first, where the pieces are cut)
        return union(LineString(piece).buffer(half, 4) for line in lines
                     for piece in _pieces(list(LineString(line).simplify(0.0).coords), SWEPT_PIECE, 2.0 * half))
    return (LineString(lines[0]) if len(lines) == 1 else MultiLineString(lines)).buffer(half, 4) if lines else \
        Polygon()


def plan_footprint(plan: CoveragePlan) -> Optional[Footprint]:
    cfg = plan.config
    if cfg is None or not cfg.body or plan.body_space is None:
        return None
    return Footprint(plan.body_space, cfg.body_outline, cfg.spin_margin)


def body_out(poses: List[Pose2D], segments: List[SegmentInfo], section_starts: List[int],
             footprint: Footprint) -> List[Dict[str, Any]]:
    """The poses where the body sticks out: [{x, y, yaw, kind}], kind 'spin' (turning on the spot there), 'turn',
    'lane_end', 'loop' or 'transit' (what the pose belongs to). On the turns between lanes spin_margin further in
    (footprint.OnTurn)."""
    kind_at: Dict[int, str] = {}
    on_turn = [False] * len(poses)  # (on the way to it: a turn between lanes)
    for seg in segments:
        kind = {TURN: 'turn', TRANSIT: 'transit', PERIMETER: 'loop'}.get(seg.kind, 'loop' if seg.maneuver == 'ring'
                                                                         else 'lane_end')
        for i in range(seg.start, seg.end + 1):
            kind_at[i] = kind
        on_turn[seg.start + 1:seg.end + 1] = [seg.kind == TURN] * (seg.end - seg.start)
    out = []
    for a, b in zip(section_starts, list(section_starts[1:]) + [len(poses)]):
        part = poses[a:b]
        for i in footprint.bad_poses(part, on_turn[a:b]):
            p = part[i]
            out.append({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(p.yaw, 3),
                        'kind': 'spin' if footprint.spins(part, i) else kind_at.get(a + i, 'transit')})
    return out


def blade_points(plan: CoveragePlan, start: int, end: int) -> List[XY]:
    """Where the blade's centre is at poses start..end: blade_offset to the left of each, blade_ahead ahead."""
    off = plan.config.blade_offset if plan.config else 0.0
    ahead = plan.config.blade_ahead if plan.config else 0.0
    return [(p.x + ahead * math.cos(p.yaw) - off * math.sin(p.yaw),
             p.y + ahead * math.sin(p.yaw) + off * math.cos(p.yaw)) for p in plan.poses[start:end + 1]]


def swept_area(plan: CoveragePlan, mower_width: float, simplify: float = 0.0):
    """What the blade covers along the whole path (a disk of mower_width), for checks and plots. simplify: the
    blade's way that much simpler first (quicker, for comparing plans)."""
    ends = list(plan.section_starts[1:]) + [len(plan.poses)]
    lines = [LineString(blade_points(plan, a, b - 1)) for a, b in zip(plan.section_starts, ends) if b - a >= 2]
    if simplify > 0:
        lines = [line.simplify(simplify) for line in lines]
    if not lines:
        return Polygon()
    # (GEOS takes a lot longer for one long way crossing itself all over than for pieces of it and their union. Cut
    # in the middle of a piece at least the width long, the two ends' round caps lie in the strip of the other half:
    # the union is the same area)
    return union(LineString(piece).buffer(0.5 * mower_width, 4) for line in lines
                 for piece in _pieces(list(line.coords), SWEPT_PIECE, mower_width))


def _pieces(pts: List[XY], n: int, cut: float) -> List[List[XY]]:
    """pts in pieces of at least n points, cut in the middle of a straight piece at least `cut` long."""
    out, cur = [], [pts[0]]
    for p, q in zip(pts, pts[1:]):
        if len(cur) >= n and math.hypot(q[0] - p[0], q[1] - p[1]) >= cut:
            m = (0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1]))
            out.append(cur + [m])
            cur = [m]
        cur.append(q)
    out.append(cur)
    return out
