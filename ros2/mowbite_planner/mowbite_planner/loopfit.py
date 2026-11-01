"""The loops' repairs after the body's check: where the safety net (bodyfit._fit_body) leaves a loop out for long, the
loop further in there (loop_in, loop_round), where it leaves a loop's start or end out the loop on past its start
(loops_on), each kept where that leaves less unmowed (fit_rings); where a loop turns on the spot by more than SPIN_LIMIT
an omega in its place (_smooth_spins). The route's methods for those (LoopFits, rings._Route has them)."""

import bisect
import itertools
import math
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

from .bodyfit import _assemble, _fit_body, _loop_ends, _round_corners, _unmowed
from .config import (LANE, LOOP_EASE, LOOP_GAIN, LOOP_IN, LOOP_JOIN, LOOP_LEFT, LOOP_ON, LOOP_ROUND, LOOP_ROUND_GAP,
                     LOOP_ROUND_REACH, LOOP_ROUND_TRIES, LOOP_TRIES, PERIMETER, ROUND_BATCH, SMOOTH_RADII, SPIN_LIMIT,
                     TRANSIT, XY, PlannerConfig)
from .drive import _spins
from .footprint import SPIN
from .maneuvers import dubins, dubins_lead, join, omega, same_position, sample_straight
from .mathutil import Pose2D, angle_diff, normalize_angle


class LoopFits:
    """The loops' repairs, methods of the route (rings._Route)."""

    def loops_in(self, places: Sequence[Sequence[float]]) -> Dict[int, List[Pose2D]]:
        """The loops further in round those places (loop_in), their corners too tight for that rounded (loop_round):
        [x, y, m], where the body sticks out on the path and m of it are left out for that. Their segments get the new
        poses, returns the poses they had (by segment)."""
        before: Dict[int, List[Pose2D]] = {}
        for k, seg in enumerate(self.segments):
            side = self.loop_side.get(seg.source) if seg.kind == PERIMETER else None
            if side is None or len(seg.poses) < 3:
                continue
            at = [0.0]
            for a, b in zip(seg.poses, seg.poses[1:]):
                at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
            # (a place is given to the cm)
            near = [(at[i], m, (x, y)) for x, y, m in places for i, p in enumerate(seg.poses)
                    if abs(p.x - x) < 0.006 and abs(p.y - y) < 0.006]
            if not near:
                continue
            poses = self.loop_in(seg.poses, side, lambda i: any(abs(at[i] - d) <= m for d, m, _ in near))
            poses = self.loop_round(poses, [(xy, m) for _, m, xy in near])
            if poses is not seg.poses:
                before[k], seg.poses = seg.poses, poses
                self.loops[int(seg.source.split('-')[1])] = [(p.x, p.y) for p in poses]
        return before

    def loops_on(self, ends: Dict[str, float]) -> Dict[int, List[Pose2D]]:
        """The loops (by name) on past their start by those m, along themselves (a loop ends where it starts): what's
        left out where it gets onto one is mowed at its end, what's left out where it gets off it was at its start. The
        drive from there to the next loop anew, arriving the way that one starts (none where something else comes next
        in its section, or where with turn_on_spot off that drive turns on the spot by more than SPIN_LIMIT). Their
        segments get the new poses, returns the poses they had (by segment)."""
        before: Dict[int, List[Pose2D]] = {}
        for k, seg in enumerate(self.segments):
            by = ends.get(seg.source) if seg.kind == PERIMETER else None
            if not by or len(seg.poses) < 3 or not same_position(seg.poses[0], seg.poses[-1]):
                continue
            at = [0.0]
            for a, b in zip(seg.poses, seg.poses[1:]):
                at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
            # (the loop itself: after an omega in place of turning onto it, see _smooth_spins)
            first = max(i for i in range(len(seg.poses) - 1)
                        if same_position(seg.poses[i], seg.poses[0]) and at[i] < 0.5 * at[-1])
            if by >= at[-1] - at[first]:
                continue
            more = [p for i, p in enumerate(seg.poses) if i > first and at[i - 1] - at[first] < by]
            poses = seg.poses + more
            nxt = self.segments[k + 1:k + 3]
            if nxt and nxt[0].section == seg.section:
                if len(nxt) < 2 or nxt[0].kind != TRANSIT or nxt[1].kind != PERIMETER or \
                        nxt[1].source not in self.loop_side:
                    continue
                drive, after = nxt
                way = self.drive_from(poses[-1], (drive.poses[-1].x, drive.poses[-1].y), after.poses[0].yaw)
                if way is None or not self.cfg.turn_on_spot and _spins(way, poses[-1].yaw, after.poses[0].yaw):
                    continue
                before[k + 1], drive.poses = drive.poses, way
            before[k], seg.poses = seg.poses, poses
            self.loops[int(seg.source.split('-')[1])] = [(p.x, p.y) for p in poses]
        return before

    def drive_from(self, start: Pose2D, target: XY, heading: Optional[float]) -> Optional[List[Pose2D]]:
        """drive_to from start (facing its way) instead of from where the mower is."""
        here = self.pos, self.heading
        self.pos, self.heading = (start.x, start.y), start.yaw
        try:
            return self.drive_to(target, heading)
        finally:
            self.pos, self.heading = here

    def loops_back(self, before: Dict[int, List[Pose2D]]) -> None:
        """The loops (and the drives after them) as they were before loops_in or loops_on."""
        for k, poses in before.items():
            seg = self.segments[k]
            seg.poses = poses
            if seg.kind == PERIMETER:
                self.loops[int(seg.source.split('-')[1])] = [(p.x, p.y) for p in poses]

    def loop_in(self, poses: List[Pose2D], side: float, here=None) -> List[Pose2D]:
        """With the body checked: where it would stick out on a loop (round a bend of the edge the front swings out),
        that stretch runs further in, as little as it takes (see LOOP_IN), so the blade still mows along the edge there
        (side: 1 with the mowable side on the left, -1 on the right; here: only stretches with a pose it says so for,
        by index). Not at the loop's ends, where it turns on the spot or where a pose fits no further in either: what
        still sticks out, bodyfit._fit_body drives another way. poses itself where nothing is moved."""
        fp = self.footprint
        if fp is None or self.cfg.blade_offset != 0 or len(poses) < 3:
            return poses
        fit = fp.fit(poses)
        if all(fit):
            return poses
        shift = _Shift(self, poses, side, fit)
        at, stay = shift.at, shift.stay
        # the stretches: poses that don't fit one after the other
        runs: List[List[int]] = []
        for i in sorted(shift.need):
            if runs and i == runs[-1][1] + 1:
                runs[-1][1] = i
            else:
                runs.append([i, i])
        if here is not None:
            runs = [run for run in runs if any(here(i) for i in range(run[0], run[1] + 1))]
        out: List[Pose2D] = []
        k = 0  # (shift.done: the poses before it are in out)
        while k < len(runs):
            # the ones close after it with nothing between them that stays: their ways in and out would meet
            end = k
            while end + 1 < len(runs) and at[runs[end + 1][0]] - at[runs[end][1]] <= 2.0 * LOOP_EASE[0] and \
                    bisect.bisect_left(stay, runs[end + 1][0]) == bisect.bisect_left(stay, runs[end][1]):
                end += 1
            got = None
            for last in list(range(k, min(end, k + LOOP_JOIN) + 1)) + [end] * (end > k + LOOP_JOIN):
                got = shift.further_in(runs[k][0], runs[last][1])
                if got is not None:
                    break
            if got is None:
                k += 1
                continue
            j0, j1, new = got
            out += poses[shift.done:j0] + new
            shift.done, k = j1 + 1, last + 1
            self._count('loops_in')
        return out + poses[shift.done:] if shift.done else poses

    def loop_round(self, poses: List[Pose2D], places: Sequence[Tuple[XY, float]]) -> List[Pose2D]:
        """With the body checked: where a stretch of a loop still sticks out within m of a place ((x, y), m), a corner
        too tight for the body (its front swings out further than loop_in takes it in), Dubins' way from the loop's
        way before it onto its way after it in its place (see LOOP_ROUND): the corner rounded off, the loop on along
        the edge on both sides. Not over the loop's ends or where it turns on the spot. poses itself where nothing
        changes."""
        fp, n = self.footprint, len(poses)
        if fp is None or n < 3:
            return poses
        fit = fp.fit(poses)
        at = [0.0]
        for a, b in zip(poses, poses[1:]):
            at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
        runs: List[List[int]] = []
        for i in range(1, n - 1):
            if fit[i]:
                continue
            if runs and at[i] - at[runs[-1][1]] <= LOOP_ROUND_GAP:
                runs[-1][1] = i
            else:
                runs.append([i, i])
        runs = [run for run in runs if any(math.hypot(poses[i].x - x, poses[i].y - y) <= m
                                           for i in range(run[0], run[1] + 1) for (x, y), m in places)]
        if not runs:
            return poses
        spins = [i for i in range(1, n) if same_position(poses[i - 1], poses[i]) and
                 angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN]
        out: List[Pose2D] = []
        done = 0  # (the poses before it are in out)
        for i0, i1 in runs:
            if i0 <= done:
                continue
            # (from where to where: what it leaves of the loop, the least first)
            ends = sorted((at[i0] - at[j0] + at[j1] - at[i1], j0, j1)
                          for j0 in range(i0 - 1, max(done, 1) - 1, -1)
                          if fit[j0] and at[i0] - at[j0] <= LOOP_ROUND_REACH
                          for j1 in range(i1 + 1, n - 1) if fit[j1] and at[j1] - at[i1] <= LOOP_ROUND_REACH and
                          not any(j0 < k <= j1 for k in spins))
            got, batch, size = None, [], 1
            # (the tries in their order a batch at a time, twice as many each time: the first that fits, as one at a
            # time)
            for item in itertools.chain(_round_tries(poses, ends), [None]):
                if item is not None:
                    batch.append(item)
                    if len(batch) < size:
                        continue
                got = self._rounded(poses, batch)
                if got is not None:
                    break
                batch, size = [], min(2 * size, ROUND_BATCH)
            if got is None:
                continue
            j0, j1, way = got
            out += poses[done:j0] + way
            done = j1 + 1
            self._count('loops_rounded')
        return out + poses[done:] if done else poses

    def _rounded(self, poses: List[Pose2D], tries: List[Tuple[int, int, float, float]]
                 ) -> Optional[Tuple[int, int, List[Pose2D]]]:
        """Of loop_round's tries (j0, j1, radius, turning at most) the first whose Dubins' way from pose j0 to pose j1
        the body fits in the area: (j0, j1, the way), else None. The poses inside the ways' first arcs all at once first
        (most that don't fit stick out right where they leave the loop), the whole ways of the others after."""
        fp = self.footprint
        leads = [dubins_lead(poses[j0], poses[j1], r, self.step, turning) for j0, j1, r, turning in tries]
        ways = []
        for (j0, j1, r, turning), lead, fits in zip(tries, leads, fp.all_fit_each([lead or [] for lead in leads])):
            if lead is not None and fits:
                m = dubins(poses[j0], poses[j1], r, self.step, loops=False, turning=turning)[:1]
                way = join(*(piece.poses for piece in m[0].pieces)) if m else None
                if way:
                    ways.append((j0, j1, way))
        for (j0, j1, way), fits in zip(ways, fp.all_fit_each([way for _, _, way in ways])):
            if fits and self.area.is_free([(p.x, p.y) for p in way]):
                return j0, j1, way
        return None


class _Shift:
    """A loop's stretches further in (LoopFits.loop_in): how far each pose that sticks out fits further in (need), the
    poses that stay where they are (stay: the loop's ends, where it turns on the spot, the poses that fit no further in
    either), the ways in and out tried (LOOP_TRIES at most)."""

    def __init__(self, route, poses: List[Pose2D], side: float, fit: List[bool]) -> None:
        self.route, self.poses, self.fp = route, poses, route.footprint
        n, (self.step, self.most) = len(poses), LOOP_IN
        self.at = [0.0]
        for a, b in zip(poses, poses[1:]):
            self.at.append(self.at[-1] + math.hypot(b.x - a.x, b.y - a.y))
        # across the way the loop goes at each pose (over the poses next to it: at a bend halfway), to the mowable side
        self.normal = []
        for i in range(n):
            a, b = poses[max(0, i - 1)], poses[min(n - 1, i + 1)]
            d = math.hypot(b.x - a.x, b.y - a.y) or 1.0
            self.normal.append((-side * (b.y - a.y) / d, side * (b.x - a.x) / d))
        # how far in each pose that sticks out fits, facing the way it does
        self.need: Dict[int, float] = {}
        left, by = [i for i in range(n) if not fit[i]], self.step
        while left and by <= self.most + 1e-9:
            ok = self.fp.fit([Pose2D(poses[i].x + by * self.normal[i][0], poses[i].y + by * self.normal[i][1],
                                     poses[i].yaw) for i in left])
            self.need.update((i, by) for i, f in zip(left, ok) if f)
            left, by = [i for i, f in zip(left, ok) if not f], by + self.step
        self.stay = sorted({0, n - 1, *left, *(k for i in range(1, n) if angle_diff(poses[i].yaw, poses[i - 1].yaw) >
                                               SPIN for k in (i - 1, i))})
        for i in self.stay:
            self.need.pop(i, None)
        self.tries, self.done = 0, 0

    def moved(self, j0: int, i0: int, i1: int, j1: int, by: float) -> Optional[List[Pose2D]]:
        """Poses j0..j1 with i0..i1 that far in, the others on the way there and back, each facing the next (the last
        as it did). Without the ones that would go backwards: on the inside of a bend they get closer."""
        at, poses, normal = self.at, self.poses, self.normal
        pts: List[Tuple[int, float, float]] = []
        for j in range(j0, j1 + 1):
            t = (at[j] - at[j0]) / (at[i0] - at[j0]) if j < i0 else \
                (at[j1] - at[j]) / (at[j1] - at[i1]) if j > i1 else 1.0
            o, p = by * t * t * (3.0 - 2.0 * t), poses[j]
            x, y = p.x + o * normal[j][0], p.y + o * normal[j][1]
            if pts:
                k, u, v = pts[-1]
                if (x - u) * (p.x - poses[k].x) + (y - v) * (p.y - poses[k].y) <= 0.0:
                    continue
            pts.append((j, x, y))
        # (step apart at most: on the outside of a bend they get further apart)
        new = join(*(sample_straight(x, y, u, v, math.atan2(v - y, u - x), self.route.step)[:-1]
                     for (_, x, y), (_, u, v) in zip(pts, pts[1:])), [poses[j1]])
        if _spins_of(poses[j0 - 1:j0] + new) > _spins_of(poses[j0 - 1:j1 + 1]) or not self.fp.all_fit(new) or \
                not self.route.area.is_free([(p.x, p.y) for p in new]):
            return None
        return new

    def further_in(self, i0: int, i1: int) -> Optional[Tuple[int, int, List[Pose2D]]]:
        """The stretch i0..i1 as little further in as fits, over the shortest way there and back, between the poses
        that stay and what's moved already: (the first and the last pose it replaces, the poses in their place)."""
        at, stay, step = self.at, self.stay, self.step
        lo, hi = stay[bisect.bisect_left(stay, i0) - 1], stay[bisect.bisect_left(stay, i1)]
        for k in range(int(round(max(self.need[i] for i in range(i0, i1 + 1) if i in self.need) / step)),
                       int(round(self.most / step)) + 1):
            for ease in LOOP_EASE:
                j0, j1 = bisect.bisect_left(at, at[i0] - ease), bisect.bisect_right(at, at[i1] + ease) - 1
                if max(lo, self.done - 1) < j0 < i0 and i1 < j1 < hi and self.tries < LOOP_TRIES:
                    self.tries += 1
                    new = self.moved(j0, i0, i1, j1, k * step)
                    if new is not None:
                        return j0, j1, new
        return None


def _spins_of(path: Sequence[Pose2D]) -> int:
    return sum(1 for a, b in zip(path, path[1:]) if angle_diff(a.yaw, b.yaw) > SPIN)


def _smooth_spins(route, cfg: PlannerConfig) -> None:
    """Where the loops (and the drives between them) turn on the spot by more than SPIN_LIMIT (a sharp corner), an
    omega (maneuvers.omega) of turn_radius (down to SMOOTH_RADII of it) in its place if it fits in the space of the
    segment (the drivable areas for drives, with headland_turns out of the lanes' field where the turn is) and the body
    fits: the wheels keep rolling. Not on the lanes' turns and drives: no loops there."""
    inserts: Dict[Tuple[int, int], List[Pose2D]] = {}
    before: Optional[Pose2D] = None  # the last pose before the position the mower is at
    here: Optional[Tuple[Pose2D, int, int]] = None  # the pose it leaves this position with, where it is

    def smooth() -> None:
        if before is None or here is None:
            return
        p, i, j = here
        turn = normalize_angle(p.yaw - before.yaw)
        if abs(turn) <= SPIN_LIMIT:
            return

        def straight(k: int) -> bool:
            return route.segments[k].kind == LANE and route.segments[k].maneuver not in ('ring', 'bend')

        if j == 0 and i > 0 and straight(i):  # where a lane starts: on the end of the segment before
            i, j = i - 1, len(route.segments[i - 1].poses)
        if straight(i) or not route.segments[i].source.startswith('loop'):  # (a lane keeps its heading)
            return
        space = route.transit.is_free if route.segments[i].kind == TRANSIT else route.area.is_free
        clear = route.clear([(p.x, p.y)])  # (headland_turns: out of the lanes' field the loop stays out of it)

        def fits(pts: Sequence[XY]) -> bool:
            return space(pts) and (not clear or route.clear(pts))

        at = Pose2D(p.x, p.y, before.yaw)
        body = route.body
        for share in SMOOTH_RADII:
            r = share * cfg.turn_radius
            loop = omega(at, turn, r, route.step) if r >= max(1e-3, cfg.min_turn_radius - 1e-9) else None
            if loop and fits([(q.x, q.y) for q in loop]) and (body is None or body([before] + loop)):
                inserts[(i, j)] = loop
                route._count('spins_smoothed')
                return

    for i, seg in enumerate(route.segments):
        for j, p in enumerate(seg.poses):
            if here is not None and route.segments[here[1]].section != seg.section:
                before = here = None  # the path jumps to the next section
            if here is not None and same_position(here[0], p):
                here = (p, i, j)  # the same position, now left facing p's way
                continue
            smooth()
            before, here = (here[0] if here is not None else None), (p, i, j)
    for (i, j), loop in sorted(inserts.items(), reverse=True):
        # (the last pose is where the segment goes on from: one pose there when they're put together)
        route.segments[i].poses[j:j] = loop


def fit_rings(route, planned, free) -> Tuple[Any, Tuple[List[Pose2D], List[Any], Dict[str, Any]]]:
    """_fit_body on the planned path (poses, segments), then where that leaves a loop out for long at a place the loop
    further in there instead, then where it leaves a loop's start or end out the loop on past its start (LOOP_ON):
    each kept where that leaves LOOP_GAIN m2 less unmowed (and no more places out). The planned path that's fitted
    and what _fit_body made of it."""
    poses, segments, fitted = _fit_body(route, *planned)
    for change in (lambda: route.loops_in([p for p in fitted['places'] if p[2] >= LOOP_LEFT]),
                   lambda: route.loops_on({name: m + LOOP_ON for name, m in _loop_ends(
                       *planned, fitted['spans'], route.loop_side).items()})):
        before = change()
        if not before:
            continue
        again_planned = _round_corners(route, *_assemble(route.segments))
        again = _fit_body(route, *again_planned)
        if again[2]['left'] <= fitted['left'] and len(again[2]['unchecked']) <= len(fitted['unchecked']) and \
                _unmowed(route, again[0], again[1], free).area <= \
                _unmowed(route, poses, segments, free).area - LOOP_GAIN:
            planned, (poses, segments, fitted) = again_planned, again
        else:
            route.loops_back(before)
    return planned, (poses, segments, fitted)


def _round_tries(poses: List[Pose2D], ends) -> Iterator[Tuple[int, int, float, float]]:
    """loop_round's tries in their order: (j0, j1, radius, turning at most) of a Dubins' way from pose j0 to pose j1,
    each pair's LOOP_ROUND radii, the widest first, LOOP_ROUND_TRIES at most (but a pair's radii all)."""
    tries = 0
    for _, j0, j1 in ends:
        turn = sum(angle_diff(poses[k].yaw, poses[k + 1].yaw) for k in range(j0, j1))
        for r in LOOP_ROUND:
            tries += 1
            yield j0, j1, r, turn + 0.5 * math.pi
        if tries >= LOOP_ROUND_TRIES:
            return
