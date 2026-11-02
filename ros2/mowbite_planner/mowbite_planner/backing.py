"""drive slope and reverse (PlannerConfig.drive): plans the mower drives partly or all backwards, for OpenMower's FTC
with back_up_with_plan. slope: lanes only, up a lane forwards, an S of one lane spacing backing down to the next
one, down it backwards, an S forwards to the next one, up it (no turning on the slope). reverse: the planner's
whole plan (loops too) for the body turned round, every pose turned round after: the body where the plan for the
turned one has it. Both with the body checked (body_fit, spin_margin at least DRIVE_SPIN_MARGIN) whatever the
settings say: backing up the mower feels nothing behind it, no bumper there."""

import dataclasses
import math
from typing import Callable, List, Optional, Tuple

from shapely.geometry import LineString

from .config import DRIVE_SPIN_MARGIN, LANE, TURN, CoveragePlan, PlannerConfig, PlanningError, SegmentInfo
from .handover import _lengths
from .mathutil import Pose2D, normalize_angle
from .sweep import body_out, plan_footprint, recorded_footprint

# m along the lane the S takes to get one lane spacing across (longer, flatter, where the body doesn't fit: by
# SHIFT_MORE up to SHIFT_MAX), m between poses on it
SHIFT = 0.6
SHIFT_MORE = 0.2
SHIFT_MAX = 1.4
STEP = 0.05
# m a lane end comes in each time the body doesn't fit there, at most so many times
TRIM = 0.05
TRIMS = 20
# m on from a sharp corner's turn pose (corners) to the pose facing the next leg (FTC: within 1 mm the same place);
# cos of half the corner below which no heading lies between both ways (about 178 degrees)
CORNER_STEP = 0.002
CORNER_COS = 0.02


def unavailable(cfg: PlannerConfig) -> str:
    """Why drive slope or reverse can't be planned with cfg ('' where it can): without the body's size it can't be
    checked."""
    return '' if cfg.body else 'no body set (robot_width, robot_front, robot_rear or robot_outline) to check'


def forced(cfg: PlannerConfig) -> List[str]:
    """What slope and reverse turn on that cfg has off, why: for the plan's warnings and planner.settings."""
    out = []
    if not cfg.body_fit:
        out.append('body_fit on')
    if cfg.spin_margin < DRIVE_SPIN_MARGIN:
        out.append('spin_margin %.2f' % DRIVE_SPIN_MARGIN)
    return out


WHY = 'backing up the mower feels nothing behind it (no bumper there)'


def settings(cfg: PlannerConfig) -> PlannerConfig:
    """What a plan with cfg.drive (slope, reverse) is planned with: the body checked, spin_margin at least
    DRIVE_SPIN_MARGIN; slope lanes only; reverse the body turned round (its front and rear swapped, the blade's place
    and the outline turned with it). drive stays (planner._as_recorded goes by it)."""
    safe = cfg.with_overrides({'body_fit': True, 'spin_margin': max(cfg.spin_margin, DRIVE_SPIN_MARGIN)})
    if cfg.drive == 'slope':
        return safe.with_overrides({'perimeter_passes': 0, 'obstacle_passes': 0, 'fill_pattern': 'lanes'})
    outline = None if cfg.robot_outline is None else tuple((-x, -y) for x, y in cfg.robot_outline)
    return dataclasses.replace(safe, robot_front=cfg.robot_rear, robot_rear=cfg.robot_front,
                               robot_front_width=cfg.robot_rear_width, robot_rear_width=cfg.robot_front_width,
                               robot_outline=outline, blade_ahead=-cfg.blade_ahead, blade_offset=-cfg.blade_offset)


def planned(plan: Callable[[PlannerConfig], CoveragePlan], cfg: PlannerConfig, mowed=None) -> CoveragePlan:
    """The plan for cfg.drive slope or reverse as the mower drives it, plan(c) planning with c as plan_coverage does
    with drive forward. Its config: cfg with the body check turned on; its warnings say where that was."""
    if cfg.drive == 'slope' and mowed is not None:
        raise PlanningError('drive slope: no leftovers planned (the lanes go up and down only, no turns there)')
    used = settings(cfg)
    own = cfg.with_overrides({'body_fit': used.body_fit, 'spin_margin': used.spin_margin})
    base = plan(used)
    out = updown(base) if cfg.drive == 'slope' else reverse(base)
    notes = forced(cfg)
    warnings = ['drive %s: %s, %s' % (cfg.drive, ' and '.join(notes), WHY)] if notes else []
    out = dataclasses.replace(out, config=own, warnings=list(out.warnings) + warnings)
    return corners(out) if cfg.drive == 'reverse' else out


def reverse(plan: CoveragePlan) -> CoveragePlan:
    """The plan for the body turned round driven backwards by the mower as it is: every pose turned round, every
    segment backwards."""
    poses = [Pose2D(p.x, p.y, normalize_angle(p.yaw + math.pi)) for p in plan.poses]
    segments = [dataclasses.replace(s, reverse=True) for s in plan.segments]
    stats = dict(plan.stats, drive='reverse', backing_m=plan.stats.get('path_length'))
    return dataclasses.replace(plan, poses=poses, segments=segments, stats=stats)


def corners(plan: CoveragePlan) -> CoveragePlan:
    """FTC's back_up_with_plan backs up from a pose where it and the next one elsewhere both face against the way
    between them. Turned round, the pose at a corner sharper than 90 degrees faces against the next leg only: FTC
    stops there and starts again. At each such corner the pose faces between both ways instead, one CORNER_STEP on
    along the next leg faces against it (turning there, next to the corner), where the body fits turning so; else the
    corner as it was. stats['corners']: how many."""
    fp = plan_footprint(plan)
    if fp is None:
        return plan
    poses, starts = plan.poses, set(plan.section_starts)
    back = [False] * len(poses)
    for s in plan.segments:
        back[s.start:s.end] = [s.reverse] * (s.end - s.start)
    turn: dict = {}  # pose: (its heading, the pose after it)
    for i in range(1, len(poses) - 1):
        if i in starts or i + 1 in starts or not (back[i - 1] and back[i]):
            continue
        a, c, b = poses[i - 1], poses[i], poses[i + 1]
        (x1, y1), (x2, y2) = (c.x - a.x, c.y - a.y), (b.x - c.x, b.y - c.y)
        n1, n2 = math.hypot(x1, y1), math.hypot(x2, y2)
        if n1 <= 2 * CORNER_STEP or n2 <= 2 * CORNER_STEP or x1 * math.cos(c.yaw) + y1 * math.sin(c.yaw) < 0:
            continue
        hx, hy = -(x1 / n1 + x2 / n2), -(y1 / n1 + y2 / n2)
        if math.hypot(hx, hy) < 2 * CORNER_COS:
            continue
        h = math.atan2(hy, hx)
        q = Pose2D(c.x + CORNER_STEP * x2 / n2, c.y + CORNER_STEP * y2 / n2, c.yaw)
        if fp.fits_turn(c.x, c.y, a.yaw, h, True) and fp.fits_turn(q.x, q.y, h, c.yaw, True):
            turn[i] = (h, q)
    if not turn:
        return dataclasses.replace(plan, stats=dict(plan.stats, corners=0))
    out, new = [], []
    for i, p in enumerate(poses):
        new.append(len(out))
        if i in turn:
            out += [Pose2D(p.x, p.y, turn[i][0]), turn[i][1]]
        else:
            out.append(p)
    # (the pose after a turn pose: in the segment of the step from it, which goes on from there)
    segments = [dataclasses.replace(s, start=new[s.start], end=new[s.end]) for s in plan.segments]
    stats = dict(plan.stats, poses=len(out), corners=len(turn))
    return dataclasses.replace(plan, poses=out, segments=segments, stats=stats,
                               section_starts=[new[j] for j in plan.section_starts])


def _frame(angle: float):
    c, s = math.cos(angle), math.sin(angle)
    return (lambda x, y: (x * c + y * s, -x * s + y * c)), (lambda u, v: (u * c - v * s, u * s + v * c))


def _lanes(base: CoveragePlan, to_lane) -> List[float]:
    """The lanes' positions across, sorted, as the planner laid them."""
    return sorted({round(to_lane(*a)[1], 4) for a, _ in base.lanes})


def _span(region, v: float, to_map, to_lane) -> Optional[Tuple[float, float]]:
    """The longest stretch of the line across v inside region: (u0, u1)."""
    minx, miny, maxx, maxy = region.bounds
    r = math.hypot(maxx - minx, maxy - miny) + 1.0
    cu, _ = to_lane(0.5 * (minx + maxx), 0.5 * (miny + maxy))
    cut = region.intersection(LineString([to_map(cu - r, v), to_map(cu + r, v)]))
    parts = [g for g in getattr(cut, 'geoms', [cut]) if g.geom_type == 'LineString' and not g.is_empty]
    if not parts:
        return None
    best = max(parts, key=lambda g: g.length)
    us = sorted(to_lane(x, y)[0] for x, y in best.coords)
    return us[0], us[-1]


def _path(spans: List[Tuple[float, float, float]], shifts: List[float], step: float):
    """[(u, v, yaw in the lane frame, back, kind, (what, lane))] from the lanes (v, u0, u1), every second one
    backwards, the S to lane k shifts[k] long; what: 'lane' or 's'."""
    out: List[Tuple[float, float, float, bool, str, Tuple[str, int]]] = []

    def add(u, v, travel, back, kind, tag):
        yaw = travel + (math.pi if back else 0.0)
        out.append((u, v, math.atan2(math.sin(yaw), math.cos(yaw)), back, kind, tag))

    def straight(v, ua, ub, back, kind, tag, first=False):
        if abs(ub - ua) < 1e-6 and not first:  # (FTC skips a pose doubled)
            return
        n = max(1, int(math.ceil(abs(ub - ua) / step)))
        travel = 0.0 if ub >= ua else math.pi
        for j in range(0 if first else 1, n + 1):
            add(ua + (ub - ua) * j / n, v, travel, back, kind, tag)

    for k, (v, u0, u1) in enumerate(spans):
        back = k % 2 == 1
        if k == 0:
            straight(v, u0, u1, False, LANE, ('lane', 0), first=True)
            continue
        pv, pu0, pu1 = spans[k - 1]
        d, shift = v - pv, shifts[k]
        if back:  # from the top of the lane before: back to where both reach, the S down, the lane down
            here, sign = min(pu1, u1), -1.0
            straight(pv, pu1, here, True, TURN, ('lane', k - 1))
        else:
            here, sign = max(pu0, u0), 1.0
            straight(pv, pu0, here, False, TURN, ('lane', k - 1))
        n = max(2, int(math.ceil(shift / STEP)))
        for j in range(1, n + 1):
            t = j / n
            dv = d * 0.5 * (1.0 - math.cos(math.pi * t))
            ddv = d * 0.5 * math.pi / shift * math.sin(math.pi * t)
            add(here + sign * shift * t, pv + dv, math.atan2(ddv, sign), back, TURN, ('s', k))
        straight(v, here + sign * shift, u0 if back else u1, back, LANE, ('lane', k))
    return out


def _segments(local) -> List[SegmentInfo]:
    segments: List[SegmentInfo] = []
    for i, (_, _, _, back, kind, _) in enumerate(local):
        if i == 0:
            continue
        last = segments[-1] if segments else None
        if last is not None and last.kind == kind and last.reverse == back:
            last.end = i
        else:
            segments.append(SegmentInfo(kind, i - 1, i, back, 's_offset' if kind == TURN else '', source='slope'))
    return segments


def updown(base: CoveragePlan) -> CoveragePlan:
    """The lanes of base (planned with settings(cfg) for slope) driven up forwards and down backwards instead, each as
    long as the body fits (checked as check_plan does), PlanningError where it doesn't. Where fewer than two lanes are
    long enough for that, base as it is, said in its warnings."""
    to_lane, to_map = _frame(base.angle)
    region = base.robot_space if base.robot_space is not None else base.lane_region
    spans = []
    for v in _lanes(base, to_lane):
        got = _span(region, v, to_map, to_lane)
        if got is not None and got[1] - got[0] > 2 * SHIFT + 0.2:
            spans.append([v, got[0], got[1]])
    if len(spans) < 2:
        return dataclasses.replace(base, warnings=list(base.warnings) + [
            'drive slope: too small for lanes up and down, planned with turns between the lanes'])
    fp = plan_footprint(base, blade=False)
    rec = recorded_footprint(base)
    step = min(STEP * 2, base.config.waypoint_spacing or STEP * 2) if base.config else STEP * 2
    shifts = [SHIFT] * len(spans)
    straight = fp.spun(base.config.spin_margin) if fp is not None and base.config else fp
    for _ in range(TRIMS + 1):
        built = [tuple(s) for s in spans]
        local = _path(built, shifts, step)
        poses = [Pose2D(*to_map(u, v), normalize_angle(yaw + base.angle))
                 for u, v, yaw, _, _, _ in local]
        segments = _segments(local)
        out = body_out(poses, segments, [0], fp, rec) if fp is not None else []
        if not out:
            break
        # where the body would fit facing up the lane, an S too steep gets flatter, at its flattest the lane at the
        # edge goes (its nose turns out backing up); else the lane end near it comes in
        longer, ends, drop = set(), set(), set()
        for b in out:
            i = min(range(len(poses)), key=lambda j: (poses[j].x - b['x']) ** 2 + (poses[j].y - b['y']) ** 2)
            u, _, _, _, _, (what, k) = local[i]
            steep = what == 's' and straight.fits_pose(poses[i].x, poses[i].y, base.angle)
            if steep and shifts[k] + SHIFT_MORE <= SHIFT_MAX + 1e-9:
                longer.add(k)
            elif steep and (k - 1 == 0 or k == len(spans) - 1):
                drop.add(k - 1 if k - 1 == 0 else k)
            elif what == 's':
                ends.update((j, 1 if k % 2 == 0 else 2) for j in (k - 1, k))
            else:
                ends.add((k, 1 if abs(u - spans[k][1]) < abs(u - spans[k][2]) else 2))
        for k in longer:
            shifts[k] += SHIFT_MORE
        for k, end in ends:
            spans[k][end] += TRIM if end == 1 else -TRIM
        keep = [j for j, s in enumerate(spans) if j not in drop and s[2] - s[1] > 2 * SHIFT + 0.2]
        spans, shifts = [spans[j] for j in keep], [shifts[j] for j in keep]
        if len(spans) < 2:
            break
    if out:
        raise PlanningError('drive slope: the body sticks out at %d poses going up and down the lanes' % len(out))
    lanes = [(to_map(u0, v), to_map(u1, v)) for v, u0, u1 in built]
    stats = dict(base.stats, **_lengths(poses, segments), lanes=len(lanes), drive='slope', backing_m=sum(
        math.hypot(poses[i + 1].x - poses[i].x, poses[i + 1].y - poses[i].y)
        for s in segments if s.reverse for i in range(s.start, s.end)))
    # (what the planner warns of its own lanes and turns doesn't hold for these)
    return dataclasses.replace(base, poses=poses, segments=segments, lanes=lanes, perimeter_loops=[], stats=stats,
                               warnings=[], section_starts=[0], origins={})
