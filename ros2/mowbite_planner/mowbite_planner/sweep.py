"""Where the blade mows along poses, and where a plan's body sticks out."""

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, MultiLineString, Polygon

from .config import PERIMETER, SWEPT_PIECE, SWEPT_SLACK, TRANSIT, TURN, XY, CoveragePlan, PlannerConfig, SegmentInfo
from .footprint import BLADE_SLACK, SPIN, SPIN_BACK, Blade, Footprint
from .geometry import union
from .mathutil import Pose2D, angle_diff, normalize_angle
from .progress import check
from .space import loop_footprint


# blade's answers kept (the same poses again, to the bit: the same geometry; the safety net and the completion stage
# ask about the same stretches again and again), cleared at this many
BLADES_KEPT = 1024
_blades: Dict[tuple, Any] = {}
# m: where a section starts closer than SECTION_NEAR to where the one before ends, body_out checks the way over as
# mower_logic drives it (turning on the spot to face the start, straight there, poses SECTION_STEP apart, turning to the
# start's heading); further apart the parts have no way between them anyway (MowBite OS plans its own, starts.py)
SECTION_NEAR = 1.0
SECTION_STEP = 0.05


def blade(cfg: PlannerConfig, *runs: Sequence[Pose2D], whole: bool = False):
    """Where the blade mows driving poses (each run of them). whole: the whole path (quicker in pieces, see
    swept_area)."""
    check()  # (a planning stopped from outside ends here, progress.stoppable)
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
    backwards = _backwards(plan.segments, len(plan.poses))
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
        way = -back if backwards[i] else back
        d, at = blade.depth(p.x, p.y, before.yaw, p.yaw, way * math.cos(before.yaw), way * math.sin(before.yaw)) \
            if spin else blade.depth(p.x, p.y, p.yaw)
        if at is not None and d > 0.0:
            out.append({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(p.yaw, 3),
                        'kind': 'spin' if spin else kind_at.get(i, 'transit'), 'm': round(d + BLADE_SLACK, 3)})
    return out


def _backwards(segments: List[SegmentInfo], n: int) -> List[bool]:
    """For each of n poses whether the mower came to it backing up (the step to it in a segment driven backwards)."""
    out = [False] * n
    for seg in segments:
        if seg.reverse:
            out[seg.start + 1:seg.end + 1] = [True] * (seg.end - seg.start)
    return out


def on_recorded(segments: Sequence, names, n: int) -> List[bool]:
    """For each of n poses whether it's the recorded loop's (recorded_loop; names: its pieces, CoveragePlan's
    recorded_loops): of a PERIMETER segment of one of them, its ways on and off too."""
    out = [False] * n
    for seg in segments if names else ():
        if seg.kind == PERIMETER and seg.source in names:
            out[seg.start:seg.end + 1] = [True] * (seg.end + 1 - seg.start)
    return out


def recorded_footprint(plan: CoveragePlan) -> Optional[Tuple[Footprint, Tuple[str, ...]]]:
    """(the body's check on the recorded loop, its pieces by name) as body_out takes it, None where the plan has
    none."""
    cfg = plan.config
    if cfg is None or not cfg.body or plan.recorded_space is None or not plan.recorded_loops:
        return None
    return loop_footprint(plan.recorded_space, cfg), tuple(plan.recorded_loops)


def body_out(poses: List[Pose2D], segments: List[SegmentInfo], section_starts: List[int],
             footprint: Footprint, recorded: Optional[Tuple[Footprint, Sequence[str]]] = None,
             between: bool = False) -> List[Dict[str, Any]]:
    """The poses where the body sticks out: [{x, y, yaw, kind}], kind 'spin' (turning on the spot there), 'turn',
    'lane_end', 'loop' or 'transit' (what the pose belongs to). On the turns between lanes spin_margin further in
    (footprint.OnTurn). Turning on the spot where it came backing up, the way in is ahead of it (segments driven
    backwards). recorded (recorded_footprint): the recorded loop's poses in its own space instead. between: where a
    section starts close to where the one before ends, the way over too (SECTION_NEAR: a plan mower_logic drives; one
    that went through starts.reached_starts has its ways there checked by the robot's arrives)."""
    kind_at: Dict[int, str] = {}
    on_turn = [False] * len(poses)  # (on the way to it: a turn between lanes)
    for seg in segments:
        kind = {TURN: 'turn', TRANSIT: 'transit', PERIMETER: 'loop'}.get(seg.kind, 'loop' if seg.maneuver == 'ring'
                                                                         else 'lane_end')
        for i in range(seg.start, seg.end + 1):
            kind_at[i] = kind
        on_turn[seg.start + 1:seg.end + 1] = [seg.kind == TURN] * (seg.end - seg.start)
    out = []
    backwards = _backwards(segments, len(poses))
    mild = on_recorded(segments, recorded[1], len(poses)) if recorded is not None else None
    for a, b in zip(section_starts, list(section_starts[1:]) + [len(poses)]):
        part = poses[a:b]
        bad = footprint.bad_poses(part, on_turn[a:b], backwards[a:b] if any(backwards[a:b]) else None)
        mine = [i for i in range(len(part)) if mild[a + i]] if mild is not None else []
        if mine:
            rec = recorded[0]
            fit = dict(zip(mine, rec.fit([part[i] for i in mine])))
            own = [i for i in mine if not (rec.fits_turn(part[i].x, part[i].y, part[i - 1].yaw, part[i].yaw,
                                                         backwards[a + i]) if rec.spins(part, i) else fit[i])]
            bad = sorted(own + [i for i in bad if not mild[a + i]])
        for i in bad:
            p = part[i]
            out.append({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(p.yaw, 3),
                        'kind': 'spin' if footprint.spins(part, i) else kind_at.get(a + i, 'transit')})
    for a in section_starts[1:] if between else ():  # (the ways over close to the sections' ends, see SECTION_NEAR)
        if 0 < a < len(poses) and math.hypot(poses[a].x - poses[a - 1].x, poses[a].y - poses[a - 1].y) <= SECTION_NEAR:
            out.extend({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(p.yaw, 3), 'kind': kind}
                       for p, kind in way_over(footprint, poses[a - 1], poses[a], backwards[a - 1]))
    return out


def way_over(footprint: Footprint, e: Pose2D, s: Pose2D, backwards: bool = False) -> List[Tuple[Pose2D, str]]:
    """Where the body sticks out driving from e (where a section ends; backwards: it got there backing up) to s (where
    the next one starts) as mower_logic does: turning on the spot to face s, straight there (poses SECTION_STEP apart),
    turning into s's heading. [(pose, 'spin' or 'transit')], none where it fits all along."""
    d = math.hypot(s.x - e.x, s.y - e.y)
    h = math.atan2(s.y - e.y, s.x - e.x) if d > 1e-6 else s.yaw
    k = max(1, int(math.ceil(d / SECTION_STEP)))
    wrong = [(e, 'spin')] if angle_diff(h, e.yaw) > SPIN and not footprint.fits_turn(e.x, e.y, e.yaw, h, backwards) \
        else []
    wrong += [(p, 'transit') for p in (Pose2D(e.x + (s.x - e.x) * j / k, e.y + (s.y - e.y) * j / k, h)
                                       for j in range(1, k)) if not footprint.fits_pose(p.x, p.y, p.yaw)]
    if angle_diff(s.yaw, h) > SPIN and not footprint.fits_turn(s.x, s.y, h, s.yaw):
        wrong.append((s, 'spin'))
    return wrong


def fits_over(footprint: Footprint, e: Pose2D, s: Pose2D, backwards: bool = False) -> bool:
    """not way_over(footprint, e, s, backwards), quicker: the turns first, the way between as all_fit has it."""
    d = math.hypot(s.x - e.x, s.y - e.y)
    h = math.atan2(s.y - e.y, s.x - e.x) if d > 1e-6 else s.yaw
    if angle_diff(h, e.yaw) > SPIN and not footprint.fits_turn(e.x, e.y, e.yaw, h, backwards) or \
            angle_diff(s.yaw, h) > SPIN and not footprint.fits_turn(s.x, s.y, h, s.yaw):
        return False
    k = max(1, int(math.ceil(d / SECTION_STEP)))
    return k < 2 or footprint.all_fit([Pose2D(e.x + (s.x - e.x) * j / k, e.y + (s.y - e.y) * j / k, h)
                                       for j in range(1, k)])


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
