"""The plan as it's handed on: the drives with the blade on where their centre stays where the loops' may, the blade
keeps off the areas not to mow and its middle on the lawn (those between two pieces of loops as parts of them,
_ring_ways; the others leading to something driven with it, _blade_on), only the poses it needs (_compact); the path's
stats that change with it (_lengths)."""

import dataclasses
import math
from typing import Any, Dict, List, Optional

from shapely.geometry import LineString, MultiPoint
from shapely.prepared import prep

from .config import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan, SegmentInfo
from .footprint import SPIN, Blade
from .geometry import QUAD_SEGS
from .maneuvers import path_length
from .mathutil import angle_diff

# m the blade's middle may get past the lawn as drawn driving a drive with the blade on (Master 31.10.: none over the
# lawn's edge), rad between the headings it's checked at turning on the spot
BLADE_PAST = 0.01
BLADE_TURN = math.radians(10.0)
# m, rad: the poses handed on, of a stretch without a turn on the spot those closer than COMPACT_OFF to the line between
# the ones kept round them left out, the ones kept waypoint_spacing apart at most and facing COMPACT_TURN at most off
# the way between them (the loops shrunk from a recorded edge have a pose every mm or less: the answer many times what
# they need, mowed and checked the same)
COMPACT_OFF = 1e-3
COMPACT_TURN = math.radians(5.0)


def _ring_ways(plan: CoveragePlan, lawn=None) -> CoveragePlan:
    """plan with the drives between two pieces of loops (the safety net's ways round a place, the hop from one loop to
    the next, the completion stage's moves from a loop, also one after the other with its strokes between them) parts
    of the loops (PERIMETER, or a ring's LANE where they lead to one: the blade keeps running, on MowBite OS too; mkay
    31.10.: the blade stays on all along the loops) where all of their centre lies where the loops' may (robot_space as
    drawn, a mm more), the blade keeps off the areas not to mow and its middle on the lawn (as drawn, BLADE_PAST more).
    stats['ring_ways']: how many became loops, how many and m of such drives stay drives for that."""
    if not plan.segments:
        return plan
    space = plan.free_space.buffer(-plan.config.centre_offset + 1e-3, QUAD_SEGS) if plan.free_space is not None \
        else None
    inside = prep(space) if space is not None and not space.is_empty else None
    blade = Blade.of(plan.unmowed, plan.config)
    on_lawn = _on_lawn(lawn)
    segments = list(plan.segments)

    def ring(seg) -> bool:
        return seg.kind == PERIMETER or seg.kind == LANE and seg.maneuver == 'ring'

    def between(seg) -> bool:  # (a drive, or a stroke of the completion stage)
        return seg.kind == TRANSIT or seg.maneuver == 'mop_up' and not ring(seg)

    done = {'loops': 0, 'kept': 0, 'kept_m': 0.0}
    i = 1
    while i < len(segments) - 1:
        j = i  # (segments i to j - 1: such a run after a piece of a loop)
        while j < len(segments) and between(segments[j]) and segments[j].section == segments[i - 1].section:
            j += 1
        if j == i or not ring(segments[i - 1]) or j == len(segments) or not ring(segments[j]) or \
                segments[j].section != segments[i - 1].section:
            i = j + 1
            continue
        for k in range(i, j):
            seg = segments[k]
            if seg.kind != TRANSIT:
                continue
            pts = [(p.x, p.y) for p in plan.poses[seg.start:seg.end + 1]]
            if inside is not None and len(pts) >= 2 and inside.covers(LineString(pts)) and \
                    _blade_kept(blade, plan, seg) and on_lawn(plan, seg):
                # (of the kind of the loop it leads to: plan_pieces puts a drive with what comes after it)
                lane = segments[j].kind == LANE
                segments[k] = dataclasses.replace(seg, kind=LANE if lane else PERIMETER,
                                                  maneuver='ring' if lane else seg.maneuver, blade=True)
                done['loops'] += 1
            else:
                done['kept'] += 1
                done['kept_m'] = round(done['kept_m'] + path_length(plan.poses[seg.start:seg.end + 1]), 2)
        i = j + 1
    stats = dict(plan.stats, ring_ways=done, **_lengths(plan.poses, segments))
    return dataclasses.replace(plan, segments=segments, stats=stats)


def _blade_on(plan: CoveragePlan, lawn=None) -> CoveragePlan:
    """plan with the other drives whose centre lies all where the loops' may (as _ring_ways has it) and that lead to
    something in their section driven with the blade on (SegmentInfo.blade, mkay 31.10.: the blade never off on the
    lawn; MowBite OS gets them of the kind of what comes after them, config.out_kinds) where the blade keeps off the
    areas not to mow and its middle on the lawn (as _ring_ways has it); the others (leaving the area, the last one, by
    the edge) with it off. stats['blade_on']: how many and m with the blade on, how many and m off."""
    if not plan.segments or plan.free_space is None:
        return plan
    space = plan.free_space.buffer(-plan.config.centre_offset + 1e-3, QUAD_SEGS)
    inside = prep(space) if not space.is_empty else None
    blade = Blade.of(plan.unmowed, plan.config)
    on_lawn = _on_lawn(lawn)
    segments = list(plan.segments)
    done = {'on': 0, 'on_m': 0.0, 'off': 0, 'off_m': 0.0}
    for k, seg in enumerate(segments):
        if seg.kind != TRANSIT:
            continue
        after = next((s for s in segments[k + 1:] if s.kind != TRANSIT), None)
        pts = [(p.x, p.y) for p in plan.poses[seg.start:seg.end + 1]]
        m = path_length(plan.poses[seg.start:seg.end + 1])
        if after is not None and after.section == seg.section and inside is not None and len(pts) >= 2 and \
                inside.covers(LineString(pts)) and _blade_kept(blade, plan, seg) and on_lawn(plan, seg):
            segments[k] = dataclasses.replace(seg, blade=True)
            done['on'] += 1
            done['on_m'] = round(done['on_m'] + m, 2)
        else:
            done['off'] += 1
            done['off_m'] = round(done['off_m'] + m, 2)
    return dataclasses.replace(plan, segments=segments, stats=dict(plan.stats, blade_on=done))


def _compact(plan: CoveragePlan) -> CoveragePlan:
    """plan with the poses it needs (see COMPACT_OFF): every segment's first and last, both of a turn on the spot (the
    heading jumping by more than footprint.SPIN), the others where leaving them out would take the way more than
    COMPACT_OFF off them, the kept ones further than waypoint_spacing apart or facing more than COMPACT_TURN off the
    way to the next."""
    poses, segments = plan.poses, plan.segments
    n = len(poses)
    if n < 3 or not segments:
        return plan
    step = plan.config.waypoint_spacing if plan.config.waypoint_spacing > 0 else math.inf
    keep = [False] * n
    for seg in segments:
        keep[seg.start] = keep[seg.end] = True
    for i in range(1, n):
        if angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN:
            keep[i - 1] = keep[i] = True

    def fine(k: int, j: int) -> bool:  # (poses[k] to poses[j] in place of those between)
        a, b = poses[k], poses[j]
        dx, dy = b.x - a.x, b.y - a.y
        d = math.hypot(dx, dy)
        if d > step + 1e-9 or d < 1e-9:
            return False
        h = math.atan2(dy, dx)
        if angle_diff(a.yaw, h) > COMPACT_TURN or angle_diff(b.yaw, h) > COMPACT_TURN:
            return False
        return all(abs((q.x - a.x) * dy - (q.y - a.y) * dx) <= COMPACT_OFF * d for q in poses[k + 1:j])

    k = 0
    for j in range(1, n):
        if keep[j] or j + 1 == n or not fine(k, j + 1):
            keep[j] = True
            k = j
    if all(keep):
        return plan
    index = [0] * n  # (of each pose kept, its place among them)
    kept: List[Any] = []
    for i, p in enumerate(poses):
        if keep[i]:
            index[i] = len(kept)
            kept.append(p)
    segments = [dataclasses.replace(seg, start=index[seg.start], end=index[seg.end]) for seg in segments]
    return dataclasses.replace(plan, poses=kept, segments=segments, stats=dict(plan.stats, **_lengths(kept, segments)),
                               section_starts=[s.start for i, s in enumerate(segments)
                                               if i == 0 or s.section != segments[i - 1].section])


def _on_lawn(lawn):
    """on_lawn(plan, seg): whether the blade's middle stays on lawn (BLADE_PAST more) driving seg of plan, turning on
    the spot where the heading jumps (BLADE_TURN apart), from the pose before it on where its section doesn't start
    there. Always where there's no lawn given."""
    if lawn is None or lawn.is_empty:
        return lambda plan, seg: True
    kept = prep(lawn.buffer(BLADE_PAST, QUAD_SEGS))

    def on_lawn(plan: CoveragePlan, seg: SegmentInfo) -> bool:
        cfg = plan.config
        ahead, off = cfg.blade_ahead, cfg.blade_offset
        if ahead == 0 and off == 0:
            return True
        first = seg.start - 1 if seg.start > 0 and seg.start not in plan.section_starts else seg.start
        way = plan.poses[first:seg.end + 1]
        at = [(p.x, p.y, p.yaw) for p in way]
        for p, q in zip(way, way[1:]):
            turn = angle_diff(q.yaw, p.yaw)
            if turn > SPIN:  # (on the spot from p's heading round to q's)
                n = int(math.ceil(turn / BLADE_TURN))
                d = math.remainder(q.yaw - p.yaw, 2.0 * math.pi)
                at += [(q.x, q.y, p.yaw + d * k / n) for k in range(1, n)]
        return kept.covers(MultiPoint([(x + ahead * math.cos(h) - off * math.sin(h),
                                        y + ahead * math.sin(h) + off * math.cos(h)) for x, y, h in at]))

    return on_lawn


def _blade_kept(blade: Optional[Blade], plan: CoveragePlan, seg: SegmentInfo) -> bool:
    """Whether the blade keeps off the areas not to mow driving seg of plan (turning on the spot where the heading
    jumps, from the pose before it on where its section doesn't start there)."""
    if blade is None:
        return True
    first = seg.start - 1 if seg.start > 0 and seg.start not in plan.section_starts else seg.start
    way = plan.poses[first:seg.end + 1]
    return all(blade.off([(p.x, p.y, p.yaw) for p in way])) and \
        all(blade.off_turning(q.x, q.y, p.yaw, q.yaw) for p, q in zip(way, way[1:]) if angle_diff(q.yaw, p.yaw) > SPIN)


def _lengths(poses, segments) -> Dict[str, Any]:
    """The path's stats that change with it: its sections, m of each kind and all together, the poses."""
    lengths: Dict[str, float] = {PERIMETER: 0.0, LANE: 0.0, TURN: 0.0, TRANSIT: 0.0}
    for seg in segments:
        lengths[seg.kind] += path_length(poses[seg.start:seg.end + 1])
    return {'sections': len([i for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section]),
            'path_length': sum(lengths.values()), 'perimeter_length': lengths[PERIMETER], 'lane_length': lengths[LANE],
            'turn_length': lengths[TURN], 'transit_length': lengths[TRANSIT], 'poses': len(poses)}
