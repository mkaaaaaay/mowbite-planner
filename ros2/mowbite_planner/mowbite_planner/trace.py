"""Where the body doesn't fit along a loop as it's driven (at a pose, or turning on the spot at a corner), that stretch
driven anew as the mower would (repaired): from TRACE_BEFORE m before the place along the loop (pursuit.py: pure
pursuit, no turn tighter than TRACE_R), the body checked every TRACE_STEP like the safety net checks it, where it would
stick out the heading closest to the pursuit's that fits, back onto the loop past the place (TRACE_FAR m at most); where
it gets stuck again from further back with the pursuit looking further ahead. Such a loop fits all along: the safety net
has nothing to do on it. traced: a whole ring driven that way (the levels' check whether an island's ring can be
driven)."""

import bisect
import math
from typing import Callable, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon
from shapely.prepared import prep

from .config import XY
from .footprint import SPIN
from .objective import EARLIER_LEFT
from .mathutil import Pose2D, angle_diff, normalize_angle
from .pursuit import TRACE_LOOK, TRACE_PAST, TRACE_R, TRACE_STEP, _Driver, _inside, _Ring

try:  # shapely 2 tests a point without making a geometry of it
    from shapely import contains_xy
except ImportError:  # shapely 1.7 (ROS Noetic)
    contains_xy = None

# m before a place the repair starts, how far past it it may take to get back onto the loop (TRACE_JOIN m and
# TRACE_JOIN_TURN off at most, then onto it over TRACE_BLEND m, on along it from a pose of it TRACE_ON m on at least),
# how much of the loop after that has to fit
TRACE_BEFORE = 0.5
TRACE_FAR = 4.0
TRACE_JOIN = 0.01
TRACE_JOIN_TURN = math.radians(3.0)
TRACE_CLEAR = 0.3
TRACE_ON = 0.05
TRACE_BLEND = 0.21
# m before and after such places a stretch may be cut straight at most (_chamfer), from and to poses that far apart,
# where the loop turns a corner there (its heading that much off from before to after)
TRACE_CHAMFER = 1.0
TRACE_CHAMFER_STEP = 0.05
TRACE_CORNER_TURN = math.radians(30.0)
# a whole ring: its end blended into its start (it comes back to it a few mm off) over TRACE_CLOSE m, where in its
# first TRACE_MATCH m
TRACE_CLOSE = 0.3
TRACE_MATCH = 3.0
# rad a pose handed on turns at most from the one before (more would be a turn on the spot there), and faces at most
# that far off the way to the next one
TRACE_KEEP_TURN = math.radians(10.0)
TRACE_FACING = math.radians(5.0)
# a dead end the body can't turn round in (the loop turns HAIR_TURN towards the lawn's side from HAIR_LEN / 2 m before
# such places to HAIR_LEN / 2 m after them, there HAIR_WIDE m apart at most): in along one side as deep as it turns on
# the spot onto the way out (of the points HAIR_STEP apart in there the deepest first, HAIR_TRIES at most), round there,
# back out along the other side (mkay 31.10.), the way in an arc off the loop from a pose HAIR_SHORT m off at least,
# turning HAIR_ARC at most, its last pose HAIR_LAST before that point (facing the way it comes: the turn is from there)
HAIR_TURN = math.radians(150.0)
HAIR_LEN = 2.5
HAIR_WIDE = 1.5
HAIR_LAST = 0.01
# share of the lane spacing a turn round keeps its point and ways off the loops driven before (but for HAIR_FREE m
# where it leaves the loop): the inner loops' turns round in a dead end one in another, not across
HAIR_KEEP = 0.75
HAIR_FREE = 0.15
# ways out driven at most for a dead end
HAIR_DRIVES = 8
HAIR_STEP = 0.05
HAIR_TRIES = 40
HAIR_SHORT = 0.3
HAIR_ARC = math.radians(120.0)


def _bad(ps: List[Pose2D], footprint) -> List[int]:
    """Indices of the poses where the body doesn't fit (or turning on the spot there)."""
    fit = footprint.fit(ps)
    return [i for i in range(len(ps)) if not (footprint.fits_turn(ps[i].x, ps[i].y, ps[i - 1].yaw, ps[i].yaw)
                                              if footprint.spins(ps, i) else fit[i])]


def _clear(u: float, marks: List[float], length: float) -> float:
    """How far round a ring that long from u the closest of marks (sorted) is."""
    k = bisect.bisect_left(marks, u)
    return min(min(abs(u - marks[j]), length - abs(u - marks[j])) for j in (k - 1, k % len(marks)))


def _lengths(ps: List[Pose2D]) -> List[float]:
    at = [0.0]
    for a, b in zip(ps, ps[1:]):
        at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return at


class _Mend:
    """A loop (open, from a pose far from any place where the body doesn't fit: its poses, m along it at each, the
    places by index) and the ways anew round those places."""

    def __init__(self, poses: List[Pose2D], footprint, inside: Callable, only: Optional[Callable] = None) -> None:
        loop = poses[:-1]
        n = len(loop)
        at = _lengths(loop + loop[:1])
        marks = sorted(at[i % n] for i in _bad(poses, footprint) if only is None or only(poses[i]))
        # (from the pose furthest round from any such place: no stretch goes over the loop's end)
        s0 = max(range(n), key=lambda i: _clear(at[i], marks, at[n])) if marks else 0
        self.loop = loop[s0:] + loop[:s0]
        self.n, self.at = n, _lengths(self.loop + self.loop[:1])
        self.bad = [i for i in _bad(self.loop + self.loop[:1], footprint) if 0 < i < n and
                    (only is None or only(self.loop[i]))]
        self.ring = _Ring([(p.x, p.y) for p in self.loop])
        self.fits, self.inside = footprint.fits_pose, inside
        self.driver = _Driver(self.ring, self.fits, inside)

    def anew(self, s: int, k: int) -> Tuple[Optional[Tuple[int, int, List[Pose2D]]], int]:
        """The way anew from pose s past the places from bad[k] on that it passes (and back onto the loop): ((from, to,
        the poses in between) or None, the last of those places by index in bad)."""
        last = [k]
        got = self.driver.drive(self.loop[s], self.at[s], self._joined(last),
                                self.at[self.bad[last[-1]]] - self.at[s] + TRACE_FAR)
        if got is None:
            return None, last[-1]
        j, on = self._on(got[1][-1])
        return (s, j, got[0][1:] + on), last[-1]

    def _joined(self, last: List[int]) -> Callable:
        """done() for the driver: where it's back on the loop (TRACE_JOIN off, TRACE_JOIN_TURN at most) past the places
        from bad[last[-1]] on it passes (last grows by those), blended onto it: the index of the last pose."""
        at, bad, ring, n = self.at, self.bad, self.ring, self.n

        def joined(ps: List[Pose2D], along: list) -> Optional[int]:
            u = along[-1]
            while last[-1] + 1 < len(bad) and at[bad[last[-1] + 1]] <= u + TRACE_CLEAR:
                last.append(last[-1] + 1)
            if u < at[bad[last[-1]]] + TRACE_PAST or u > at[n] - TRACE_STEP:
                return None
            p = ps[-1]
            uu, d = ring.project(p.x, p.y, u, 0.2)
            if d > TRACE_JOIN or angle_diff(p.yaw, ring.heading(uu)) > TRACE_JOIN_TURN:
                return None
            blend = _blended(ring, p, uu)
            # (onto it before its end: past that it'd be on its start again, a step back)
            if uu + TRACE_STEP * len(blend) > at[n] - TRACE_STEP or \
                    not all(self.fits(q.x, q.y, q.yaw) for q in blend) or \
                    not all(self.inside([q.x for q in blend], [q.y for q in blend])):
                return None
            ps += blend
            along += [uu + TRACE_STEP * m for m in range(1, len(blend) + 1)]
            return len(ps) - 1

        return joined

    def _on(self, end: float) -> Tuple[int, List[Pose2D]]:
        """From end (m along the loop) on along it to its pose j, TRACE_STEP apart (no gap where it gets back onto
        it): (j, the poses before it)."""
        at, ring, n = self.at, self.ring, self.n
        j = min(bisect.bisect_left(at, end + TRACE_ON), n)
        us = [end + TRACE_STEP * m for m in range(1, int((at[j] - end) / TRACE_STEP) + 1)]
        return j, [Pose2D(*ring.at(u), self.loop[min(n - 1, bisect.bisect_right(at, u) - 1)].yaw) for u in us
                   if u < at[j] - 1e-6]

    def turnaround(self, k: int, last: int, ends: Tuple[int, int], footprint,
                   taken: Optional[Callable] = None) -> Optional[Tuple[int, int, List[Pose2D]]]:
        """In a dead end (ends: as _dead_end found it round the places bad[k] to bad[last]) as deep as the mower turns
        on the spot (see HAIR_TURN): off the loop along an arc to there, round onto the way out, back onto the loop
        past bad[last] as the driver drives it (see anew), all of it off what taken says (the loops before: theirs
        go round further in, one in another). (from, to, the poses in between) or None where there's no such
        place."""
        loop, at, ring, first = self.loop, self.at, self.ring, self.bad[k]
        a, b = ends
        found = self._turn_points(ends, taken)
        if found is None:
            return None
        grid, deep = found

        def clear(way: List[Pose2D]) -> bool:
            return taken is None or not way or not any(taken([q.x for q in way], [q.y for q in way]))

        # (the way out: the driver keeping off what's taken too)
        driver, drives = self.driver, 0
        if taken is not None:
            driver = _Driver(ring, self.fits, lambda xs, ys: [bool(a) and not b for a, b in zip(self.inside(xs, ys),
                                                                                                taken(xs, ys))])

        # (the tip: the deepest pose of it, the places' first at the earliest)
        tip = max(max(range(a, b + 1), key=lambda i: deep(loop[i].x, loop[i].y)), first)
        ins = []
        for i in range(first - 1, a - 1, -1):  # (HAIR_STEP apart along the loop)
            if not ins or at[ins[-1]] - at[i] >= HAIR_STEP - 1e-9:
                ins.append(i)
        for x, y in grid[:HAIR_TRIES]:
            into = None
            for i in ins:
                p = loop[i]
                arc = _arc(p.x, p.y, p.yaw, x, y) if math.hypot(x - p.x, y - p.y) >= HAIR_SHORT else None
                if arc is None:
                    continue
                # (the turn on the spot from the pose before: one HAIR_LAST before the point, facing the arc's way)
                h0 = arc[-1].yaw
                arc[-1] = Pose2D(x - HAIR_LAST * math.cos(h0), y - HAIR_LAST * math.sin(h0), h0)
                if clear([q for q in arc if math.hypot(q.x - p.x, q.y - p.y) > HAIR_FREE]) and \
                        all(self.inside([q.x for q in arc], [q.y for q in arc])) and all(footprint.fit(arc)):
                    into = (i, arc)
                    break
            if into is None:
                continue
            # (onto the other side past the places, where the loop doesn't fit the driver mostly wouldn't either;
            # else from the closest point of it past the tip)
            span = max(at[b] - at[self.bad[last]], 1e-3) / 1.25
            aims = [ring.project(x, y, at[self.bad[last]] + 0.25 * span, span)[0],
                    ring.project(x, y, at[tip], at[b] - at[tip])[0]]
            for u in aims if abs(aims[1] - aims[0]) > TRACE_STEP else aims[:1]:
                tx, ty = ring.at(u + TRACE_LOOK)
                h0, h1 = into[1][-1].yaw, math.atan2(ty - y, tx - x)
                if angle_diff(h0, h1) > SPIN and not footprint.fits_turn(x, y, h0, h1):
                    continue
                drives += 1
                if drives > HAIR_DRIVES:
                    return None
                got = driver.drive(Pose2D(x, y, h1), u, self._joined([last]),
                                   max(0.0, at[self.bad[last]] - u) + math.hypot(tx - x, ty - y) + TRACE_FAR)
                if got is not None:
                    j, on = self._on(got[1][-1])
                    return into[0], j, into[1] + got[0] + on
        return None

    def _turn_points(self, ends: Tuple[int, int], taken: Optional[Callable]) -> Optional[Tuple[List[XY], Callable]]:
        """Where in a dead end (ends: as _dead_end found it) the mower may turn round: the points HAIR_STEP apart in
        there where the centre may go and the body fits facing in and out, off what taken says, the deepest first; and
        deep(x, y), how deep a point is. None where its ends face the same way or enclose nothing."""
        loop = self.loop
        a, b = ends
        pa, pb = loop[a], loop[b]
        # (how deep a point is: how far in from the mouth, along the way in less the way out)
        ux, uy = math.cos(pa.yaw) - math.cos(pb.yaw), math.sin(pa.yaw) - math.sin(pb.yaw)
        norm = math.hypot(ux, uy)
        region = Polygon([(p.x, p.y) for p in loop[a:b + 1]]).buffer(0)
        if norm < 1e-9 or region.is_empty:
            return None
        ux, uy, mx, my = ux / norm, uy / norm, 0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y)
        into_yaw = math.atan2(uy, ux)

        def deep(x: float, y: float) -> float:
            return (x - mx) * ux + (y - my) * uy

        x0, y0, x1, y1 = region.bounds
        grid = [(x0 + HAIR_STEP * (i + 0.5), y0 + HAIR_STEP * (j + 0.5)) for i in range(int((x1 - x0) / HAIR_STEP))
                for j in range(int((y1 - y0) / HAIR_STEP))]
        xs, ys = [x for x, _ in grid], [y for _, y in grid]
        if contains_xy is not None:
            prep(region)
            within = [bool(v) for v in contains_xy(region, xs, ys)]
        else:
            kept = prep(region)
            within = [kept.contains(Point(x, y)) for x, y in grid]
        # (the deepest first; where the body doesn't fit facing in or out it doesn't turn there)
        grid = sorted((p for p, ok, w in zip(grid, self.inside(xs, ys), within) if ok and w), key=lambda p: -deep(*p))
        grid = [(x, y) for x, y in grid if self.fits(x, y, into_yaw) and self.fits(x, y, into_yaw + math.pi)]
        if taken is not None and grid:
            grid = [p for p, t in zip(grid, taken([x for x, _ in grid], [y for _, y in grid])) if not t]
        return grid, deep


def repaired(poses: List[Pose2D], footprint, pts: Sequence[XY], hole: bool, step: float, width: float,
             ahead: float, room=None, only: Optional[Callable] = None,
             taken: Optional[Callable] = None) -> Tuple[List[Pose2D], int, int]:
    """The closed loop poses (the last the first again; driven round pts, the ring of a level, see _inside) with the
    stretches where the body doesn't fit driven anew (see the module), or cut straight turning on the spot at both ends
    (_chamfer) where that leaves less unmowed (_lost: the blade width wide, ahead of the centre): (the poses, stretches
    driven anew, poses left as they were for the safety net). Places too close to where the loop was opened for that
    get another go, the loop opened elsewhere. only(pose): the places to look at (default all). taken(xs, ys): which
    of those points a turn round in a dead end keeps off (see HAIR_KEEP)."""
    if len(poses) < 4 or footprint is None or not _bad(poses, footprint):
        return poses, 0, 0
    inside = _inside(pts, hole, room)
    # (the lawn's side of the loop: rings come with it on the left, driven the other way round on the right)
    side = 1.0 if (sum(ax * by - bx * ay for (ax, ay), (bx, by) in zip(pts, list(pts[1:]) + [pts[0]])) > 0) != hole \
        else -1.0
    out, fixed, left, cramped = _mended(poses, footprint, inside, width, ahead, only, side, taken)
    if cramped:
        out, more, left, _ = _mended(out + out[:1], footprint, inside, width, ahead, only, side, taken)
        fixed += more
    return _thinned(out + out[:1], step), fixed, left


def _mended(poses: List[Pose2D], footprint, inside: Callable, width: float, ahead: float,
            only: Optional[Callable] = None, side: float = 1.0,
            taken: Optional[Callable] = None) -> Tuple[List[Pose2D], int, int, int]:
    """repaired's ways once round, the loop opened furthest from the places: (the open loop, stretches driven anew,
    poses left, of those where there was no room before or after). side: the lawn's (1 left, -1 right)."""
    mend = _Mend(poses, footprint, inside, only)
    loop, at, bad, n = mend.loop, mend.at, mend.bad, mend.n
    out: List[Pose2D] = []
    done_to, fixed, left, cramped, k = 0, 0, 0, 0, 0
    while k < len(bad):
        s = max(done_to, bisect.bisect_left(at, at[bad[k]] - TRACE_BEFORE))
        # (no room before it or to get back on after it before the loop's end: left as it is)
        if s >= bad[k] or at[bad[k]] + TRACE_PAST + TRACE_BLEND + TRACE_ON > at[n]:
            left += 1
            cramped += 1
            k += 1
            continue
        way, last = mend.anew(s, k)
        ways = [way] if way is not None else []  # (from, to (indices of loop), the poses in between)
        cut = _chamfer(loop, at, bad[k], bad[last], done_to, footprint, inside)
        if cut is not None:
            ways.append(cut)
        ends = _dead_end(loop, at, bad[k], bad[last], done_to, side)
        turn = None if ends is None else mend.turnaround(k, last, ends, footprint, taken)
        if turn is not None:
            ways.append(turn)
        if not ways:
            left += last + 1 - k
            k = last + 1
            continue
        a, j, way = min(ways, key=lambda w: _lost(loop, w, width, ahead))
        out += loop[done_to:a + 1] + way
        done_to = j
        fixed += 1
        k = next((i for i, c in enumerate(bad) if c >= j), len(bad))
    return out + loop[done_to:], fixed, left, cramped


def _blended(ring: _Ring, p: Pose2D, u: float) -> List[Pose2D]:
    """From p (at u along the ring, a little off it) onto the ring over TRACE_BLEND m: the offset going down evenly,
    each pose facing the next."""
    m = max(1, int(TRACE_BLEND / TRACE_STEP))
    x0, y0 = ring.at(u)
    ex, ey = p.x - x0, p.y - y0
    pts = [ring.at(u + TRACE_STEP * k) for k in range(1, m + 2)]
    pts = [(x + ex * (1.0 - k / m), y + ey * (1.0 - k / m)) if k < m else (x, y) for k, (x, y) in enumerate(pts, 1)]
    return [Pose2D(a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0])) for a, b in zip(pts, pts[1:])]


def _chamfer(loop: List[Pose2D], at: list, first: int, last: int, done_to: int, footprint,
             inside: Callable) -> Optional[Tuple[int, int, List[Pose2D]]]:
    """Cutting the stretch from before first to after last (indices of loop) straight where it turns a corner (the
    heading TRACE_CORNER_TURN off within it): on the spot at a pose before (TRACE_CHAMFER m at most, TRACE_CHAMFER_STEP
    apart) towards one after, there round on the spot onto the loop again; the shortest that fits (from, to, the poses
    in between: the turned one at from, the straight, the turned one at to). None where it doesn't turn a corner."""
    if angle_diff(loop[max(done_to, first - 1)].yaw, loop[min(len(loop) - 1, last + 1)].yaw) < TRACE_CORNER_TURN:
        return None

    def every(idx) -> List[int]:  # (TRACE_CHAMFER_STEP apart along the loop)
        got: List[int] = []
        for i in idx:
            if not got or abs(at[i] - at[got[-1]]) >= TRACE_CHAMFER_STEP - 1e-9:
                got.append(i)
        return got

    before = every(range(first - 1, max(done_to, bisect.bisect_left(at, at[first] - TRACE_CHAMFER)) - 1, -1))
    after = every(range(last + 1, min(len(loop), bisect.bisect_right(at, at[last] + TRACE_CHAMFER))))
    for _, i, j in sorted((at[first] - at[i] + at[j] - at[last], i, j) for i in before for j in after):
        p, q = loop[i], loop[j]
        d = math.hypot(q.x - p.x, q.y - p.y)
        if d < TRACE_STEP:
            continue
        h = math.atan2(q.y - p.y, q.x - p.x)
        n = int(d / TRACE_STEP)
        way = [Pose2D(p.x, p.y, h)] + [Pose2D(p.x + (q.x - p.x) * m / n, p.y + (q.y - p.y) * m / n, h)
                                       for m in range(1, n)] + [Pose2D(q.x, q.y, h), q]
        if not all(inside([w.x for w in way[1:-1]], [w.y for w in way[1:-1]])) or not all(footprint.fit(way[:-1])):
            continue
        if angle_diff(p.yaw, h) > SPIN and not footprint.fits_turn(p.x, p.y, p.yaw, h):
            continue
        if angle_diff(h, q.yaw) > SPIN and not footprint.fits_turn(q.x, q.y, h, q.yaw):
            continue
        return i, j + 1, way
    return None


def _dead_end(loop: List[Pose2D], at: list, first: int, last: int, done_to: int,
              side: float) -> Optional[Tuple[int, int]]:
    """(the poses HAIR_LEN / 2 before first and after last, indices of loop) where the loop turns HAIR_TURN towards the
    lawn's side (side) from the one to the other, HAIR_WIDE apart at most: a dead end round the places first to last.
    Else None."""
    a = max(done_to, bisect.bisect_left(at, at[first] - 0.5 * HAIR_LEN))
    b = min(len(loop) - 1, bisect.bisect_right(at, at[last] + 0.5 * HAIR_LEN) - 1)
    if math.hypot(loop[a].x - loop[b].x, loop[a].y - loop[b].y) > HAIR_WIDE:
        return None
    turn = sum(normalize_angle(loop[k + 1].yaw - loop[k].yaw) for k in range(a, b))
    return (a, b) if side * turn >= HAIR_TURN else None


def _arc(x: float, y: float, yaw: float, tx: float, ty: float) -> Optional[List[Pose2D]]:
    """The poses about TRACE_STEP apart on the arc from (x, y) facing yaw to (tx, ty), the last there (each facing along
    it), None where it'd turn more than HAIR_ARC or tighter than TRACE_R."""
    d = math.hypot(tx - x, ty - y)
    half = normalize_angle(math.atan2(ty - y, tx - x) - yaw)
    if d < 1e-6 or 2.0 * abs(half) > HAIR_ARC:
        return None
    if abs(half) < 1e-6:
        m = max(1, int(math.ceil(d / TRACE_STEP)))
        return [Pose2D(x + (tx - x) * k / m, y + (ty - y) * k / m, yaw) for k in range(1, m + 1)]
    r = 0.5 * d / math.sin(half)
    if abs(r) < TRACE_R:
        return None
    m = max(1, int(math.ceil(abs(2.0 * half * r) / TRACE_STEP)))
    out = []
    for k in range(1, m + 1):
        h = yaw + 2.0 * half * k / m
        out.append(Pose2D(x + r * (math.sin(h) - math.sin(yaw)), y - r * (math.cos(h) - math.cos(yaw)),
                          normalize_angle(h)))
    out[-1] = Pose2D(tx, ty, out[-1].yaw)
    return out


def _lost(loop: List[Pose2D], way: Tuple[int, int, List[Pose2D]], width: float, ahead: float) -> float:
    """m2 the way mows less than the loop from its from to its to would (the blade, `ahead` of the centre), and
    EARLIER_LEFT for each turn on the spot on it."""
    a, j, poses = way

    def blade(ps):
        pts = [(p.x + ahead * math.cos(p.yaw), p.y + ahead * math.sin(p.yaw)) for p in ps]
        return LineString(pts).buffer(0.5 * width, 4) if len(pts) > 1 else Polygon()

    planned = blade(loop[a:j + 1])
    driven = blade([loop[a]] + poses)
    spins = sum(1 for p, q in zip([loop[a]] + poses, poses) if angle_diff(p.yaw, q.yaw) > SPIN)
    return planned.difference(driven).area + spins * EARLIER_LEFT


def _thinned(poses: List[Pose2D], step: float) -> List[Pose2D]:
    """The poses `step` apart at most (all of a turn on the spot; closer where they turn: TRACE_KEEP_TURN at most from
    one to the next, each facing TRACE_FACING at most off the way to the next), the first and the last kept."""
    kept = [poses[0]]
    gone = 0.0
    for j in range(1, len(poses)):
        a, b = poses[j - 1], poses[j]
        d = math.hypot(b.x - a.x, b.y - a.y)
        if d < 1e-6 or angle_diff(b.yaw, a.yaw) > TRACE_KEEP_TURN:  # (a turn on the spot: as it is)
            if a is not kept[-1]:
                kept.append(a)
            kept.append(b)
            gone = 0.0
            continue
        k = kept[-1]
        if a is not k and (gone + d > step + 1e-9 or angle_diff(b.yaw, k.yaw) > TRACE_KEEP_TURN or
                           angle_diff(math.atan2(b.y - k.y, b.x - k.x), k.yaw) > TRACE_FACING):
            kept.append(a)
            gone = 0.0
        gone += d
        if gone >= step - 1e-9:
            kept.append(b)
            gone = 0.0
    if kept[-1] is not poses[-1]:
        kept.append(poses[-1])
    return kept


def _start(ring: _Ring, fits) -> Optional[float]:
    """Where to start a whole ring: of the places every 5 cm where the body fits facing along, the straightest (the
    least turning within TRACE_LOOK either side)."""
    best = None
    for i in range(int(ring.length / 0.05)):
        u = i * 0.05
        x, y = ring.at(u)
        bend = abs(normalize_angle(ring.heading(u + TRACE_LOOK) - ring.heading(u - TRACE_LOOK)))
        if (best is None or bend < best[0] - 1e-9) and fits(x, y, ring.heading(u)):
            best = (bend, u)
    return None if best is None else best[1]


def traced(pts: Sequence[XY], fits, step: float, hole: bool = False) -> Optional[List[Pose2D]]:
    """The loop round pts (closed, the way it's to be driven) driven as the mower would all the way round (see
    _Driver), its poses `step` apart (closer where it turns), the last one the first again. None where it gets stuck
    or there's nowhere to start."""
    ring = _Ring(pts)
    if ring.length < 4.0 * TRACE_LOOK:
        return None
    u = _start(ring, fits)
    if u is None:
        return None
    x, y = ring.at(u)
    goal = ring.length + TRACE_MATCH + 2.0 * TRACE_LOOK
    got = _Driver(ring, fits, _inside(pts, hole)).drive(
        Pose2D(x, y, ring.heading(u)), u, lambda ps, along: len(ps) - 1 if len(ps) * TRACE_STEP >= goal else None,
        goal + 2.0 * TRACE_STEP)
    return None if got is None else _closed(got[0], fits, step)


def _closed(poses: List[Pose2D], fits, step: float) -> Optional[List[Pose2D]]:
    """The loop of the poses driven round once and on: from the pose of its first TRACE_MATCH m the last ones come back
    to closest, round to there, the end blended into it; see _thinned."""
    n = len(poses)
    span = int(TRACE_MATCH / TRACE_STEP) + 1
    best = None
    for i in range(0, min(n, span)):
        for k in range(max(i + 1, n - 2 * span), n):
            d = math.hypot(poses[k].x - poses[i].x, poses[k].y - poses[i].y) + \
                0.05 * abs(normalize_angle(poses[k].yaw - poses[i].yaw))
            if best is None or d < best[0]:
                best = (d, i, k)
    if best is None:
        return None
    _, i, k = best
    loop = poses[i:k + 1]
    dx, dy = poses[i].x - poses[k].x, poses[i].y - poses[k].y
    m = min(len(loop) - 1, int(TRACE_CLOSE / TRACE_STEP))
    blended = list(loop)
    for j in range(len(loop) - m, len(loop)):
        f = (j - (len(loop) - m - 1)) / m
        blended[j] = Pose2D(loop[j].x + f * dx, loop[j].y + f * dy, loop[j].yaw)
    for j in range(len(loop) - m - 1, len(loop) - 1):
        a, b = blended[j], blended[j + 1]
        blended[j] = Pose2D(a.x, a.y, math.atan2(b.y - a.y, b.x - a.x))
    blended[-1] = Pose2D(poses[i].x, poses[i].y, poses[i].yaw)
    if all(fits(p.x, p.y, p.yaw) for p in blended[len(loop) - m - 1:]):
        loop = blended
    return _thinned(loop, step)


def turned_to(loop: List[Pose2D], target: XY) -> List[Pose2D]:
    """The closed loop starting at its pose closest to target."""
    j = min(range(len(loop) - 1), key=lambda i: (math.hypot(loop[i].x - target[0], loop[i].y - target[1]), i))
    return loop[j:-1] + loop[:j] + [loop[j]]
