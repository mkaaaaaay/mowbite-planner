"""Where a robot that drives to the plan's sections itself gets to them (MowBite OS: the OS side passes arrives, its
~/plan_transit in it). A section starts at its first pose arrives says it gets to facing its heading: the poses before
it left out; where it gets to none of those tried, or the asks are spent, the section is left out (no start it can't
get to, the OS takes the plan as it is)."""

import dataclasses
import math
from typing import Any, Callable, Dict, List, Optional, Tuple

from .config import CoveragePlan, PlanningError, SegmentInfo
from .maneuvers import same_position
from .mathutil import Pose2D
from .sweep import swept_area

# m of path apart a section's later poses are tried (then the ones between the last it doesn't get to and the first it
# does), the poses tried at most for a section, the poses asked about at most for a plan (a number, not a time: the
# same plan on any computer)
ARRIVE_STEP = 0.3
ARRIVE_TRIES = 12
ARRIVE_ASKS = 40

Arrives = Callable[[Pose2D], bool]


class _Spent(Exception):
    pass


def reached_starts(plan: CoveragePlan, arrives: Arrives) -> CoveragePlan:
    """plan with every section starting where arrives(pose) says the robot gets to it, see the module. stats['starts']:
    {'places': [[x, y, m of path left out]] where a section starts later (its first pose as planned), 'dropped' and
    'untried': [[x, y]] of the sections left out (no pose tried it gets to; not asked about past ARRIVE_ASKS),
    'unmowed_m2': what that leaves unmowed more, 'asks'}; the warnings say where. Raises PlanningError where it gets to
    no section at all."""
    poses, segments = plan.poses, plan.segments
    known: Dict[Tuple[float, float, float], bool] = {}
    asks = [0]

    def gets_to(p: Pose2D) -> bool:
        key = (round(p.x, 3), round(p.y, 3), round(p.yaw, 3))
        if key not in known:
            if asks[0] >= ARRIVE_ASKS:
                raise _Spent()
            asks[0] += 1
            known[key] = bool(arrives(p))
        return known[key]

    firsts = [k for k, s in enumerate(segments) if k == 0 or s.section != segments[k - 1].section]
    cut: List[Tuple[int, int]] = []  # pose ranges [a, b) left out
    places: List[List[float]] = []
    dropped: List[List[float]] = []
    untried: List[List[float]] = []
    for n, k in enumerate(firsts):
        a = segments[k].start
        last = segments[firsts[n + 1] - 1].end if n + 1 < len(firsts) else len(poses) - 1
        if a and same_position((poses[a - 1].x, poses[a - 1].y), (poses[a].x, poses[a].y)):
            continue  # (there already)
        where = [round(poses[a].x, 2), round(poses[a].y, 2)]
        try:
            got = _first_reached(poses, a, last, gets_to)
        except _Spent:
            untried.append(where)
            cut.append((a, last + 1))
            continue
        if got is None:
            dropped.append(where)
            cut.append((a, last + 1))
        elif got > a:
            m = sum(math.hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y) for i in range(a + 1, got + 1))
            places.append(where + [round(m, 2)])
            cut.append((a, got))
    starts: Dict[str, Any] = {'places': places, 'dropped': dropped, 'untried': untried, 'unmowed_m2': 0.0,
                              'asks': asks[0]}
    if not cut:
        return dataclasses.replace(plan, stats=dict(plan.stats, starts=starts))
    out = _without(plan, cut)
    if not out.poses:
        raise PlanningError('no way gets to any section of the plan facing its heading: at ' +
                            ', '.join(f'({x:.2f}, {y:.2f})' for x, y in dropped + untried))
    width = plan.config.mower_width if plan.config is not None else 0.0
    if width > 0:
        starts['unmowed_m2'] = round(swept_area(plan, width).difference(swept_area(out, width)).area, 3)
    warnings = list(plan.warnings)
    if places:
        warnings.append(f'{len(places)} sections start later where no way gets to their start facing its heading, '
                        f'{sum(m for _, _, m in places):.1f} m of loops and lanes left out: at ' +
                        ', '.join(f'({x:.2f}, {y:.2f})' for x, y, _ in places))
    if dropped:
        warnings.append(f'{len(dropped)} sections left out: no way gets to them facing their heading: at ' +
                        ', '.join(f'({x:.2f}, {y:.2f})' for x, y in dropped))
    if untried:
        warnings.append(f'{len(untried)} sections left out: the ways to them not tried past {ARRIVE_ASKS}: at ' +
                        ', '.join(f'({x:.2f}, {y:.2f})' for x, y in untried))
    if places or dropped or untried:
        warnings.append(f'{starts["unmowed_m2"]:.2f} m2 more unmowed where sections start later or are left out')
    return dataclasses.replace(out, stats=dict(plan.stats, starts=starts), warnings=warnings)


def _first_reached(poses: List[Pose2D], a: int, last: int, gets_to: Callable[[Pose2D], bool]) -> Optional[int]:
    """The section's (poses a..last) first pose gets_to, None where none of those tried."""
    if gets_to(poses[a]):
        return a
    d, bad, tried = 0.0, a, 1
    for i in range(a + 1, last):
        d += math.hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y)
        if d < ARRIVE_STEP:
            continue
        if tried >= ARRIVE_TRIES:
            return None
        d, tried = 0.0, tried + 1
        if gets_to(poses[i]):
            return next((j for j in range(bad + 1, i) if gets_to(poses[j])), i)
        bad = i
    return None


def _without(plan: CoveragePlan, cut: List[Tuple[int, int]]) -> CoveragePlan:
    """plan without the poses in the ranges cut, its segments and sections along."""
    gone = [False] * len(plan.poses)
    for a, b in cut:
        for i in range(a, b):
            gone[i] = True
    index, poses = [], []
    for i, p in enumerate(plan.poses):
        index.append(len(poses))
        if not gone[i]:
            poses.append(p)
    segments: List[SegmentInfo] = []
    for s in plan.segments:
        kept = [i for i in range(s.start, s.end + 1) if not gone[i]]
        if kept:
            segments.append(dataclasses.replace(s, start=index[kept[0]], end=index[kept[-1]]))
    firsts = [s.start for k, s in enumerate(segments) if k == 0 or s.section != segments[k - 1].section]
    return dataclasses.replace(plan, poses=poses, segments=segments, section_starts=firsts or [0])
