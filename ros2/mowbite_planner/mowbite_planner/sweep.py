"""Where the blade mows along poses, and where a plan's body sticks out."""

import math
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from shapely.geometry import LineString, MultiLineString, Polygon

from .config import PERIMETER, SWEPT_PIECE, SWEPT_SLACK, TRANSIT, TURN, XY, CoveragePlan, PlannerConfig, SegmentInfo
from .footprint import BLADE_SLACK, SPIN, SPIN_BACK, Blade, Footprint
from .geometry import union
from .mathutil import Pose2D, normalize_angle


# blade's answers kept (the same poses again, to the bit: the same geometry; the safety net and the completion stage
# ask about the same stretches again and again), cleared at this many
BLADES_KEPT = 256
_blades: Dict[tuple, Any] = {}


def blade(cfg: PlannerConfig, *runs: Sequence[Pose2D], whole: bool = False):
    """Where the blade mows driving poses (each run of them). whole: the whole path (quicker in pieces, see
    swept_area)."""
    half, ahead, off = 0.5 * cfg.mower_width, cfg.blade_ahead, cfg.blade_offset
    key = (half, ahead, off, whole, tuple(tuple(poses) for poses in runs))
    got = _blades.get(key)
    if got is not None:
        return got
    lines = [[(p.x + ahead * math.cos(p.yaw) - off * math.sin(p.yaw),
               p.y + ahead * math.sin(p.yaw) + off * math.cos(p.yaw)) for p in poses] for poses in runs]
    if whole:
        # (straight runs of poses one straight line each first, where the pieces are cut)
        got = union(LineString(piece).buffer(half, 4) for line in lines
                    for piece in _pieces(list(LineString(line).simplify(0.0).coords), SWEPT_PIECE, 2.0 * half))
    else:
        got = (LineString(lines[0]) if len(lines) == 1 else MultiLineString(lines)).buffer(half, 4) if lines else \
            Polygon()
    if len(_blades) >= BLADES_KEPT:
        _blades.clear()
    _blades[key] = got
    return got


def plan_footprint(plan: CoveragePlan, blade: bool = True) -> Optional[Footprint]:
    """The plan's Footprint, blade: with the blade kept off the areas not to mow (as planned)."""
    cfg = plan.config
    if cfg is None or not cfg.body or plan.body_space is None:
        return None
    return Footprint(plan.body_space, cfg.body_outline, cfg.spin_margin,
                     blade=Blade.of(plan.unmowed, cfg) if blade else None)


def blade_over(plan: CoveragePlan) -> List[Dict[str, Any]]:
    """The poses where the blade reaches over an area not to mow (plan.unmowed) further than BLADE_SLACK: [{x, y, yaw,
    kind, m}], kind as body_out has it, m how far its disc reaches over; turning on the spot where the heading jumps
    by more than SPIN (anywhere on the last SPIN_BACK of the way in with a spin_margin, like Footprint.fits_turn)."""
    cfg = plan.config
    blade = Blade.of(plan.unmowed, cfg) if cfg is not None else None
    if blade is None:
        return []
    kind_at: Dict[int, str] = {}
    for seg in plan.segments:
        kind = {TURN: 'turn', TRANSIT: 'transit', PERIMETER: 'loop'}.get(seg.kind, 'loop' if seg.maneuver == 'ring'
                                                                         else 'lane')
        for i in range(seg.start, seg.end + 1):
            kind_at[i] = kind
    starts, back = set(plan.section_starts), SPIN_BACK if cfg.spin_margin > 0 else 0.0
    out = []
    for i, p in enumerate(plan.poses):
        before = plan.poses[i - 1] if i and i not in starts else None
        spin = before is not None and abs(normalize_angle(p.yaw - before.yaw)) > SPIN
        d, at = blade.depth(p.x, p.y, before.yaw, p.yaw, back * math.cos(before.yaw), back * math.sin(before.yaw)) \
            if spin else blade.depth(p.x, p.y, p.yaw)
        if at is not None and d > 0.0:
            out.append({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(p.yaw, 3),
                        'kind': 'spin' if spin else kind_at.get(i, 'transit'), 'm': round(d + BLADE_SLACK, 3)})
    return out


def recorded_footprint(plan: CoveragePlan) -> Optional[Tuple[Footprint, Set[str]]]:
    """(where the body was while the outline was recorded, the loops driven the way it was by name) as body_out takes
    it, None where there are none."""
    cfg = plan.config
    if cfg is None or not cfg.body or plan.recorded_space is None or not plan.recorded_loops:
        return None
    return Footprint(plan.recorded_space, cfg.body_outline, 0.0), set(plan.recorded_loops)


def body_out(poses: List[Pose2D], segments: List[SegmentInfo], section_starts: List[int],
             footprint: Footprint, recorded: Optional[Tuple[Footprint, Set[str]]] = None) -> List[Dict[str, Any]]:
    """The poses where the body sticks out: [{x, y, yaw, kind}], kind 'spin' (turning on the spot there), 'turn',
    'lane_end', 'loop' or 'transit' (what the pose belongs to). On the turns between lanes spin_margin further in
    (footprint.OnTurn). recorded (recorded_footprint): those loops' poses in where the body was then instead."""
    kind_at: Dict[int, str] = {}
    on_turn = [False] * len(poses)  # (on the way to it: a turn between lanes)
    lenient = [False] * len(poses)
    for seg in segments:
        kind = {TURN: 'turn', TRANSIT: 'transit', PERIMETER: 'loop'}.get(seg.kind, 'loop' if seg.maneuver == 'ring'
                                                                         else 'lane_end')
        for i in range(seg.start, seg.end + 1):
            kind_at[i] = kind
        on_turn[seg.start + 1:seg.end + 1] = [seg.kind == TURN] * (seg.end - seg.start)
        if recorded is not None and seg.kind == PERIMETER and seg.source in recorded[1]:
            lenient[seg.start:seg.end + 1] = [True] * (seg.end + 1 - seg.start)
    # (the safety net's ways from a pose of such a loop to one of it: in there too, as bodyfit checks them)
    for seg in segments if recorded is not None else ():
        if seg.maneuver == 'body' and lenient[seg.start] and lenient[seg.end]:
            lenient[seg.start:seg.end + 1] = [True] * (seg.end + 1 - seg.start)
    out = []
    for a, b in zip(section_starts, list(section_starts[1:]) + [len(poses)]):
        part = poses[a:b]
        bad = footprint.bad_poses(part, on_turn[a:b])
        mild = [i for i in range(len(part)) if lenient[a + i]]
        if mild:
            rec = recorded[0]
            fit = dict(zip(mild, rec.fit([part[i] for i in mild])))
            mine = {i for i in mild if not (rec.fits_turn(part[i].x, part[i].y, part[i - 1].yaw, part[i].yaw)
                                            if rec.spins(part, i) else fit[i])}
            bad = sorted(mine.union(i for i in bad if not lenient[a + i]))
        for i in bad:
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
    return union(LineString(piece).buffer(0.5 * mower_width + SWEPT_SLACK, 4) for line in lines
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
