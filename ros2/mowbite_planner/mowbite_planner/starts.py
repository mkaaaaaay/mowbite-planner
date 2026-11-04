"""Where a robot that drives to the plan's sections itself gets to them, and away from them again (MowBite OS: the OS
side passes arrives, its ~/plan_transit in it). A section starts at its first pose arrives says it gets to facing its
heading from where the robot is then (came_from: where the section before it that's kept ends; None for the first, the
robot can be anywhere then): the poses before it left out; where it gets to none of those tried, or the asks are spent,
the section is left out (no start it can't get to, the OS takes the plan as it is).

Where a section ends the robot has to get away again driving forwards: it does where its body turns on the spot there
whatever way, where the next section is got to from there, or home is (the position of the plan's first pose: where it
begins next to the dock). Where none of that holds it's a dead end (a strip narrower than the body turns in): the
section ends at the last pose before it it gets away from (left out where there's none: MowBite OS till M8), or with
back_out (a robot that backs up where the plan does) it backs up along its own way to there instead.

backed_out: back_out without arrives (OpenMower: mower_logic drives to the next section or home itself, its FTC turning
on the spot where the section ends first): where that turn doesn't fit, the section backs up along its way to the last
pose it fits at."""

import dataclasses
import math
from typing import Any, Callable, Dict, List, Optional, Tuple

from .backing import CORNER_COS, CORNER_STEP
from .config import CoveragePlan, PlanningError, SegmentInfo
from .maneuvers import path_length, same_position
from .mathutil import Pose2D, angle_diff
from .sweep import body_out, plan_footprint, recorded_footprint, swept_area

# m of path apart a section's later poses are tried (then the ones between the last it doesn't get to and the first it
# does), the poses tried at most for a section (and asked about for the way out of a dead end), the poses asked about at
# most for a plan (a number, not a time: the same plan on any computer; many of them cheap where arrives knows at once
# it doesn't get there, MowBite OS bounds the ways it plans itself)
ARRIVE_STEP = 0.3
ARRIVE_TRIES = 12
ARRIVE_ASKS = 400
# m of path a section backs up out of a dead end at most (back_out; further: it ends earlier)
BACK_OUT_MAX = 30.0

Arrives = Callable[[Pose2D, Optional[Pose2D]], bool]  # (pose, came_from); pose.yaw None: there facing any way


class _Spent(Exception):
    pass


@dataclasses.dataclass
class _Kept:
    section: int
    first: int  # its first pose kept
    last: int  # its last pose (where it ends earlier: that one)
    at: int  # where the robot is after it (its last pose, or the one it backs up to)
    place: Optional[list] = None  # its entry of 'places' (it starts later)
    out: bool = False  # it's known to get away from where it ends


def reached_starts(plan: CoveragePlan, arrives: Arrives, back_out: bool = False) -> CoveragePlan:
    """plan with every section starting where arrives(pose, came_from) says the robot gets to it and ending where it
    gets away from (back_out: backing up to there out of a dead end), see the module. stats['starts']: {'places': [[x,
    y, m of path left out]] where a section starts later (its first pose as planned), 'dropped' and 'untried': [[x, y]]
    of the sections left out (no pose tried it gets to; not asked about past ARRIVE_ASKS), 'ends': [[x, y, m of path
    left out]] where a section ends earlier (its last pose as planned), 'trapped': [[x, y]] of the sections left out
    where no pose gets away (their last pose), 'unmowed_m2': what that leaves unmowed more, 'asks'}; back_out:
    stats['back_out'] {'places': [[x, y, m backing up]] (the last pose), 'stuck': [], 'corners'}. The warnings say
    where. Raises PlanningError where no section is left."""
    walk = _Walk(plan, arrives, back_out)
    walk.run()
    return walk.result()


class _Walk:
    """reached_starts' way through the sections, the robot where it is after each."""

    def __init__(self, plan: CoveragePlan, arrives: Arrives, back_out: bool) -> None:
        self.plan, self.arrives, self.back_out, self.poses = plan, arrives, back_out, plan.poses
        self.bounds = _bounds(plan)
        self.known: Dict[tuple, bool] = {}
        self.asks = 0
        self.kept: List[_Kept] = []
        self.cut: List[Tuple[int, int]] = []  # pose ranges [a, b) left out
        self.backs: Dict[int, int] = {}  # a section's last pose: the pose it backs up to from there
        self.got: Dict[str, list] = {k: [] for k in ('places', 'dropped', 'untried', 'ends', 'trapped', 'backed')}
        self.home: Optional[Pose2D] = None
        self._fp: Any = False

    def gets_to(self, p: Pose2D, c: Optional[Pose2D]) -> bool:
        key = _key(p) + (() if c is None else _key(c))
        if key not in self.known:
            if self.asks >= ARRIVE_ASKS:
                raise _Spent()
            self.asks += 1
            self.known[key] = bool(self.arrives(p, c))
        return self.known[key]

    def came(self) -> Optional[Pose2D]:
        return self.poses[self.kept[-1].at] if self.kept else None

    def run(self) -> None:
        n = 0
        while n < len(self.bounds):
            if self.section(n):
                n += 1
        while self.kept and not self.kept[-1].out:  # (where the plan ends: home from there)
            if self.gets_away(self.kept[-1]):
                self.kept[-1].out = True
            else:
                self.dead_end()

    def section(self, n: int) -> bool:
        """Section n from where the robot is: True where that's settled (kept, or left out), False where the section
        before it turned out to end in a dead end (changed: n again from where the robot is now)."""
        a, last = self.bounds[n]
        c = self.came()
        if a and c is not None and same_position((c.x, c.y), (self.poses[a].x, self.poses[a].y)):
            self.keep(n, a)  # (there already)
            return True
        try:
            got = _first_reached(self.poses, a, last, lambda p: self.gets_to(p, c))
            spent = False
        except _Spent:
            got, spent = None, True
        if got is not None:
            self.keep(n, got)
            return True
        if self.kept and not self.kept[-1].out:
            if not self.gets_away(self.kept[-1]):
                self.dead_end()
                return False
            self.kept[-1].out = True
        self.got['untried' if spent else 'dropped'].append(_at(self.poses[a]))
        self.cut.append((a, last + 1))
        return True

    def keep(self, n: int, got: int) -> None:
        a, last = self.bounds[n]
        if self.kept:
            self.kept[-1].out = True  # (it gets to this one)
        place = None
        if got > a:
            place = _at(self.poses[a]) + [round(path_length(self.poses[a:got + 1]), 2)]
            self.got['places'].append(place)
            self.cut.append((a, got))
        if self.home is None:
            self.home = Pose2D(self.poses[got].x, self.poses[got].y, None)
        self.kept.append(_Kept(n, got, last, last, place))

    def footprint(self):
        if self._fp is False:
            self._fp = plan_footprint(self.plan)
        return self._fp

    def gets_away(self, sec: _Kept) -> bool:
        """Whether the robot gets away from where it is after sec (where it ends, or where it backed up to)."""
        return self.away(sec.at, sec.at != sec.last)

    def away(self, i: int, backwards: bool = False, asks: Optional[int] = None) -> bool:
        """Whether the robot gets away from pose i driving forwards: its body turning there whatever way (having backed
        up to it: backwards), or home (asks: no more asked of arrives than that many in all, else no; what it said
        already counts)."""
        fp, p = self.footprint(), self.poses[i]
        if fp is not None and fp.fits_turn(p.x, p.y, p.yaw, p.yaw + math.pi, backwards):
            return True
        if self.home is None:
            return False
        known = self.known.get(_key(self.home) + _key(p))
        if known is not None or asks is not None and self.asks >= asks:
            return bool(known)
        try:
            return self.gets_to(self.home, p)
        except _Spent:
            return False

    def dead_end(self) -> None:
        """The last section kept ends where the robot doesn't get away: back_out backs up from there to the last pose
        before it gets away from (BACK_OUT_MAX m at most), else the section ends there, else it's left out."""
        sec = self.kept[-1]
        e = sec.last
        to = self.back_to(sec, self.back_out)
        if to is not None and self.back_out:
            m = path_length(self.poses[to:e + 1])
            if m <= BACK_OUT_MAX and self.fits_back(to, e):
                self.backs[e] = to
                self.got['backed'].append(_at(self.poses[e]) + [round(m, 2)])
                sec.at, sec.out = to, True
                return
            to = self.back_to(sec, False)  # (backing up that far, or not fitting: it ends earlier instead)
        if to is not None and to > sec.first:
            self.cut.append((to + 1, e + 1))
            self.got['ends'].append(_at(self.poses[e]) + [round(path_length(self.poses[to:e + 1]), 2)])
            sec.last = sec.at = to
            sec.out = True
            return
        self.kept.pop()
        if sec.place is not None:
            self.got['places'].remove(sec.place)
        self.cut.append((sec.first, e + 1))
        self.got['trapped'].append(_at(self.poses[e]))
        if self.kept:
            self.kept[-1].out = False  # (known only to get to this one: away from there again)
        else:
            self.home = None

    def back_to(self, sec: _Kept, backwards: bool) -> Optional[int]:
        """The last pose of sec before its end the robot gets away from (having backed up to it: backwards):
        ARRIVE_STEP m back, then twice as far each time, then halfway between the closest that does and the furthest
        that doesn't, ARRIVE_STEP apart at least; ARRIVE_TRIES asked about at most. None where there's none."""
        first, e = sec.first, sec.last
        back = [0.0] * (e - first + 1)  # m of path back from the end, by pose (from first)
        for i in range(e - 1, first - 1, -1):
            p, q = self.poses[i], self.poses[i + 1]
            back[i - first] = back[i - first + 1] + math.hypot(q.x - p.x, q.y - p.y)
        asks = self.asks + ARRIVE_TRIES

        def at(d: float) -> int:  # (the pose closest to the end that far back at least, first at the most)
            return next((i for i in range(e - 1, first - 1, -1) if back[i - first] >= d), first)

        bad, good, step = e, None, ARRIVE_STEP
        while good is None and bad > first:
            i = at(step)
            if self.away(i, backwards, asks):
                good = i
            else:
                bad, step = i, step * 2.0
        while good is not None and back[good - first] - back[bad - first] > ARRIVE_STEP:
            i = at(0.5 * (back[good - first] + back[bad - first]))
            if i in (good, bad):
                break
            if self.away(i, backwards, asks):
                good = i
            else:
                bad = i
        return good

    def fits_back(self, to: int, e: int) -> bool:
        return _fits_back(self.plan, self.footprint(), to, e)

    def result(self) -> CoveragePlan:
        plan, got = self.plan, self.got
        starts: Dict[str, Any] = {k: got[k] for k in ('places', 'dropped', 'untried', 'ends', 'trapped')}
        starts.update(unmowed_m2=0.0, asks=self.asks)
        stats = dict(plan.stats, starts=starts)
        if self.back_out:
            stats['back_out'] = {'places': got['backed'], 'stuck': []}
        if not self.cut and not self.backs:
            return dataclasses.replace(plan, stats=stats)
        out = _rebuilt(dataclasses.replace(plan, stats=stats), self.cut, self.backs)
        if not out.poses:
            raise PlanningError('no way gets to any section of the plan facing its heading' +
                                (' and away from it again driving forwards' if got['trapped'] else '') + ': at ' +
                                ', '.join(f'({x:.2f}, {y:.2f})' for x, y in got['dropped'] + got['untried'] +
                                          got['trapped']))
        width = plan.config.mower_width if plan.config is not None else 0.0
        if width > 0 and self.cut:
            starts['unmowed_m2'] = round(swept_area(plan, width).difference(swept_area(out, width)).area, 3)
        return dataclasses.replace(out, warnings=list(plan.warnings) + _told(starts, got['backed']))


def _told(starts: Dict[str, Any], backed: List[list]) -> List[str]:
    """The warnings of reached_starts."""
    def where(xys):
        return ', '.join(f'({p[0]:.2f}, {p[1]:.2f})' for p in xys)

    out = []
    places, dropped, untried, ends, trapped = (starts[k] for k in ('places', 'dropped', 'untried', 'ends', 'trapped'))
    if places:
        out.append(f'{len(places)} sections start later where no way gets to their start facing its heading, '
                   f'{sum(p[2] for p in places):.1f} m of loops and lanes left out: at {where(places)}')
    if dropped:
        out.append(f'{len(dropped)} sections left out: no way gets to them facing their heading: at {where(dropped)}')
    if untried:
        out.append(f'{len(untried)} sections left out: the ways to them not tried past {ARRIVE_ASKS}: at '
                   f'{where(untried)}')
    if ends:
        out.append(f'{len(ends)} sections end earlier where no way gets away from their end driving forwards, '
                   f'{sum(p[2] for p in ends):.1f} m of loops and lanes left out: at {where(ends)}')
    if trapped:
        out.append(f'{len(trapped)} sections left out: no way gets away from them driving forwards: at '
                   f'{where(trapped)}')
    if backed:
        out.append(f'{len(backed)} sections back up out of a dead end where they end, {sum(p[2] for p in backed):.1f} '
                   f'm backing up: at {where(backed)}')
    if places or dropped or untried or ends or trapped:
        out.append(f'{starts["unmowed_m2"]:.2f} m2 more unmowed where sections start later, end earlier or are left '
                   'out')
    return out


def backed_out(plan: CoveragePlan) -> CoveragePlan:
    """back_out without arrives, see the module: where the body doesn't turn on the spot whatever way at a section's
    last pose (none where the next section goes on from there facing its way), it backs up along the section's way to
    the last pose before it where it does having backed up and fits all along (BACK_OUT_MAX m at most), stays where
    there's none. stats['back_out']: {'places': [[x, y, m backing up]], 'stuck': [[x, y]] (the last pose), and where
    it backs up 'corners' (turned backing up, see _back)}."""
    fp = plan_footprint(plan)
    places: List[list] = []
    stuck: List[list] = []
    backs: Dict[int, int] = {}
    bounds = _bounds(plan) if fp is not None and plan.poses else []
    for n, (first, e) in enumerate(bounds):
        p = plan.poses[e]
        if n + 1 < len(bounds):
            q = plan.poses[bounds[n + 1][0]]
            if same_position((p.x, p.y), (q.x, q.y)) and angle_diff(p.yaw, q.yaw) < 1e-6:
                continue
        if fp.fits_turn(p.x, p.y, p.yaw, p.yaw + math.pi):
            continue
        to, d = None, 0.0
        for i in range(e - 1, first - 1, -1):
            q, r = plan.poses[i], plan.poses[i + 1]
            d += math.hypot(r.x - q.x, r.y - q.y)
            if d > BACK_OUT_MAX:
                break
            if fp.fits_turn(q.x, q.y, q.yaw, q.yaw + math.pi, True):
                to = i
                break
        if to is not None and _fits_back(plan, fp, to, e):
            backs[e] = to
            places.append(_at(p) + [round(d, 2)])
        else:
            stuck.append(_at(p))
    stats = dict(plan.stats, back_out={'places': places, 'stuck': stuck})
    warnings = list(plan.warnings)
    if places:
        warnings.append(f'{len(places)} sections back up out of a dead end where they end, '
                        f'{sum(m for _, _, m in places):.1f} m backing up: at ' +
                        ', '.join(f'({x:.2f}, {y:.2f})' for x, y, _ in places))
    if stuck:
        warnings.append(f'{len(stuck)} sections end where the body doesn\'t turn on the spot and backing up gets to no '
                        'place it does: at ' + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in stuck))
    out = dataclasses.replace(plan, stats=stats, warnings=warnings)
    return _rebuilt(out, [], backs) if backs else out


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


def _bounds(plan: CoveragePlan) -> List[Tuple[int, int]]:
    """Each section's first and last pose."""
    segments = plan.segments
    firsts = [k for k, s in enumerate(segments) if k == 0 or s.section != segments[k - 1].section]
    return [(segments[k].start, segments[firsts[n + 1] - 1].end if n + 1 < len(firsts) else len(plan.poses) - 1)
            for n, k in enumerate(firsts)]


def _key(p: Pose2D) -> tuple:
    return (round(p.x, 3), round(p.y, 3)) + (() if p.yaw is None else (round(p.yaw, 3),))


def _at(p: Pose2D) -> list:
    return [round(p.x, 2), round(p.y, 2)]


def _back(plan: CoveragePlan, to: int, e: int) -> Optional[Tuple[List[Pose2D], List[SegmentInfo], int]]:
    """Backing up from pose e to pose to of the same section: the poses (e first), each facing the way it did, the
    segments (indices from e on) driven backwards over each one of plan's there, of its kind and source (its body's
    rules: the outermost loop as recorded), maneuver 'back_out', the blade on, and the corners turned. FTC backs up from
    a pose where it and the next one elsewhere both face against the way between them: at a corner sharper than a
    quarter turn the pose would face along the next step, so it faces between both ways there and one CORNER_STEP on
    along the next leg faces against it (as backing.corners). None where a corner turns about half round (no heading
    between both ways)."""
    src = [plan.poses[i] for i in range(e, to - 1, -1)]
    poses: List[Pose2D] = []
    where: List[int] = []  # each of src in poses
    turned = 0
    for k, c in enumerate(src):
        where.append(len(poses))
        if 0 < k < len(src) - 1:
            a, b = src[k - 1], src[k + 1]
            (x1, y1), (x2, y2) = (c.x - a.x, c.y - a.y), (b.x - c.x, b.y - c.y)
            n1, n2 = math.hypot(x1, y1), math.hypot(x2, y2)
            if n1 > 2 * CORNER_STEP and n2 > 2 * CORNER_STEP and x2 * math.cos(c.yaw) + y2 * math.sin(c.yaw) >= 0:
                hx, hy = -(x1 / n1 + x2 / n2), -(y1 / n1 + y2 / n2)
                if math.hypot(hx, hy) < 2 * CORNER_COS:
                    return None
                poses += [Pose2D(c.x, c.y, math.atan2(hy, hx)),
                          Pose2D(c.x + CORNER_STEP * x2 / n2, c.y + CORNER_STEP * y2 / n2, math.atan2(-y2, -x2))]
                turned += 1
                continue
        poses.append(c)
    segments = [SegmentInfo(s.kind, where[e - min(e, s.end)], where[e - max(to, s.start)], True, 'back_out', s.section,
                            s.source, True) for s in reversed(plan.segments) if s.start < e and s.end > to]
    # (a step between two segments, the next one starting right after: with the one it leads to, every step backing up)
    for a, b in zip(segments, segments[1:]):
        b.start = a.end
    segments[0].start, segments[-1].end = 0, len(poses) - 1
    return poses, segments, turned


def _fits_back(plan: CoveragePlan, fp, to: int, e: int) -> bool:
    """Whether the body fits all along backing up from pose e to pose to (where it was going forwards, the corners
    turned), checked as body_out checks a plan."""
    got = _back(plan, to, e) if fp is not None else None
    return got is not None and not body_out(got[0], got[1], [0], fp, recorded_footprint(plan))


def _rebuilt(plan: CoveragePlan, cut: List[Tuple[int, int]], backs: Dict[int, int]) -> CoveragePlan:
    """plan without the poses in the ranges cut, from each pose e of backs backing up to pose backs[e] (see _back: it
    fits), its segments and sections along; stats['back_out']['corners']: the corners turned backing up."""
    gone = [False] * len(plan.poses)
    for a, b in cut:
        for i in range(a, b):
            gone[i] = True
    index, poses, backing = [], [], {}
    for i, p in enumerate(plan.poses):
        index.append(len(poses))
        if not gone[i]:
            if i in backs:
                backing[i] = (len(poses), _back(plan, backs[i], i))
                poses.extend(backing[i][1][0])
            else:
                poses.append(p)
    segments: List[SegmentInfo] = []
    for k, s in enumerate(plan.segments):
        kept = [i for i in range(s.start, s.end + 1) if not gone[i]]
        if not kept:
            continue
        segments.append(dataclasses.replace(s, start=index[kept[0]], end=index[kept[-1]]))
        e = kept[-1]
        if e in backing and (k + 1 == len(plan.segments) or plan.segments[k + 1].start > e or
                             plan.segments[k + 1].section != s.section):
            at, (_, way, _) = backing[e]
            segments.extend(dataclasses.replace(w, start=w.start + at, end=w.end + at) for w in way)
    firsts = [s.start for k, s in enumerate(segments) if k == 0 or s.section != segments[k - 1].section]
    stats = plan.stats
    if backing:
        stats = dict(stats, back_out=dict(stats.get('back_out') or {}, corners=sum(b[2] for _, b in backing.values())))
    return dataclasses.replace(plan, poses=poses, segments=segments, section_starts=firsts or [0], stats=stats)
