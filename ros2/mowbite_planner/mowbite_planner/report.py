"""A plan checked: what's wrong with it (hard: the mower can't or mustn't drive it like that, or grass is
left) and what's worth knowing (soft), with how much is mowed once, more than once and not at all."""

import math
from typing import Any, Dict, List

from shapely.geometry import LineString, Point
from shapely.prepared import prep

from .geometry import inset, polygons, union
from .config import LANE, PERIMETER, TRANSIT, TURN, CoveragePlan
from .planner import blade_points, body_out, plan_footprint, swept_area

# m2, a piece left unmowed smaller than this is a tuft at a corner, not a mistake
TUFT = 0.01
# m2, unmowed pieces count as a mistake from this size on (or all together from LEFT_TOTAL or LEFT_SHARE of the
# area), smaller ones are tufts at sharp corners
LEFT_PIECE = 0.05
LEFT_TOTAL = 0.1
LEFT_SHARE = 5e-4
# m, poses may be this far off the space they belong in (rounding, simplifying)
TOLERANCE = 2e-3




def check_plan(plan: CoveragePlan) -> Dict[str, Any]:
    """{'hard': [...], 'soft': [...], 'body_out': [...], 'mowed_m2', 'left_m2', 'left_pieces', 'double_mowed_pct',
    ...}. body_out: the poses where the mower's body sticks out (see planner.body_out), empty when it isn't checked.

    double_mowed_pct: how much more is mowed than the area, every further pass over a spot counted again
    (the sum of what each segment mows on its own, less what they mow together)."""
    cfg = plan.config
    hard: List[str] = []
    soft: List[str] = []
    poses = plan.poses
    width = cfg.mower_width if cfg else 0.0
    step = cfg.waypoint_spacing if cfg else 0.0

    # where the mower's centre may go: the area shrunk (robot_space), on transits the drivable areas too, or robot_space
    # where there's no way through them (planner._Route.go_to: with the body checked that's body_tolerance further out
    # than transit_space has it, the body checked there)
    inside = prep(plan.robot_space.buffer(TOLERANCE)) if plan.robot_space is not None else None
    drivable = prep(union([plan.transit_space, plan.robot_space]).buffer(TOLERANCE)) \
        if plan.transit_space is not None else inside
    jumps = {s - 1 for s in plan.section_starts[1:]}
    outside = 0
    for seg in plan.segments:
        where = drivable if seg.kind == TRANSIT else inside
        if where is None:
            continue
        outside += sum(1 for p in poses[seg.start:seg.end + 1] if not where.covers(Point(p.x, p.y)))
    if outside:
        hard.append(f'{outside} poses outside the space the mower may drive in')
    gaps = [i for i, (a, b) in enumerate(zip(poses, poses[1:]))
            if i not in jumps and step > 0 and math.hypot(b.x - a.x, b.y - a.y) > step + 1e-6]
    if gaps:
        hard.append(f'{len(gaps)} gaps between poses wider than waypoint_spacing {step} m, the first after pose '
                    f'{gaps[0]}')
    if any(s.reverse for s in plan.segments):
        hard.append('drives backwards')
    footprint = plan_footprint(plan)
    body = body_out(poses, plan.segments, plan.section_starts, footprint) if footprint is not None else []
    if body:
        hard.append(f'the body sticks out past the edges at {len(body)} poses, the first ({body[0]["kind"]}) at '
                    f'({body[0]["x"]:.2f}, {body[0]["y"]:.2f})')

    mowed = swept_area(plan, width).intersection(plan.free_space) if width > 0 else None
    out: Dict[str, Any] = {'hard': hard, 'soft': soft, 'body_out': body}
    if mowed is not None:
        # what's meant to be mowed: with the first loop further in than half the blade (perimeter_offset,
        # robot_width) a strip along the edges stays
        margin = max(0.0, cfg.blade_loop_offset - 0.5 * width) if cfg else 0.0
        target = inset(plan.free_space, margin) if margin > 1e-9 else plan.free_space
        left = target.difference(mowed)
        pieces = [p for p in polygons(left) if p.area >= TUFT]
        out.update(mowed_m2=round(mowed.area, 3), left_m2=round(left.area, 3), left_pieces=len(pieces),
                   area_m2=round(plan.free_space.area, 3))
        if margin > 1e-9:
            edge = plan.free_space.area - target.area
            out['edge_left_m2'] = round(edge, 3)
            soft.append(f'{edge:.2f} m2 along the edges left on purpose: the blade keeps {margin:.2f} m off them '
                        '(perimeter_offset, robot_width, edge_margin)')
        if pieces:
            biggest = max(pieces, key=lambda p: p.area)
            c = biggest.representative_point()
            total = sum(p.area for p in pieces)
            where = hard if biggest.area >= LEFT_PIECE or total >= max(LEFT_TOTAL, LEFT_SHARE * target.area) \
                else soft
            where.append(f'{len(pieces)} pieces left unmowed, {total:.2f} m2, the biggest {biggest.area:.2f} m2 at '
                         f'({c.x:.2f}, {c.y:.2f})')
        # what each segment mows on its own, added up
        alone = 0.0
        for seg in plan.segments:
            line = blade_points(plan, seg.start, seg.end)
            if len(line) >= 2:
                alone += LineString(line).buffer(0.5 * width, 4).intersection(plan.free_space).area
        if mowed.area > 0:
            out['double_mowed_pct'] = round(100.0 * max(0.0, alone - mowed.area) / mowed.area, 1)

    soft.extend(plan.warnings)
    stats = plan.stats
    for key, what in (('turns_pivot', 'turns on the spot'), ('turns_fallback', 'turns replaced by a drive'),
                      ('turns_detour', 'turns as a detour along the edge')):
        if stats.get(key):
            soft.append(f'{stats[key]} {what}')
    if stats.get('sections', 1) > 1:
        soft.append(f'{stats["sections"]} sections: the path jumps between parts with no way between them')
    kinds = {PERIMETER: 0.0, LANE: 0.0, TURN: 0.0, TRANSIT: 0.0}
    for seg in plan.segments:
        kinds[seg.kind] += sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses[seg.start:seg.end],
                                                                                poses[seg.start + 1:seg.end + 1]))
    total = sum(kinds.values())
    if total > 0 and kinds[TRANSIT] / total > 0.1:
        soft.append(f'{100 * kinds[TRANSIT] / total:.0f} % of the way are drives between the parts of the plan')
    return out


def left_unmowed(plan: CoveragePlan):
    """What the plan leaves (pieces of at least TUFT), for plots."""
    width = plan.config.mower_width if plan.config else 0.0
    left = plan.free_space.difference(swept_area(plan, width))
    return union(p for p in polygons(left) if p.area >= TUFT)
