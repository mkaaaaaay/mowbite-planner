"""Driving along a ring as the mower would (_Driver, for trace.py): pure pursuit, no turn tighter than TRACE_R, the body
checked every TRACE_STEP, where it would stick out the heading closest to the pursuit's that fits; where it gets stuck
from further back with the pursuit looking further ahead. The ring (_Ring), where the centre may go driving it
(_inside)."""

import bisect
import math
from typing import Callable, List, Optional, Sequence, Tuple

from shapely.geometry import Point, Polygon
from shapely.prepared import prep

from .config import XY
from .mathutil import Pose2D, normalize_angle

try:  # shapely 2 tests a point without making a geometry of it
    from shapely import contains_xy
except ImportError:  # shapely 1.7 (ROS Noetic)
    contains_xy = None

# m between the poses checked
TRACE_STEP = 0.03
# m the pursuit looks ahead along the loop, the tightest turn (each step an arc: a pose faces the way to the next
# within half of TRACE_STEP / TRACE_R)
TRACE_LOOK = 0.25
TRACE_R = 0.2
# m the centre may go outside the ring on the side away from the lawn: cutting a corner of an outline it turns away
# from the lawn at (the body checked all the same); round an island TRACE_IN at most (it'd cut its corners towards it)
TRACE_OUT = 0.1
TRACE_IN = 0.01
# poses checked straight ahead of each one taken (a step that fits with none fitting after it gets it stuck)
TRACE_AHEAD = 2
# rad between the headings tried
TRACE_TURN = math.radians(1.0)
# where it gets stuck: back to TRACE_BACK times the pursuit's look before there, it looks TRACE_GROW times further from
# there to TRACE_PAST m after (each time it gets stuck there again further); so TRACE_TRIES times at most
TRACE_BACK = 2.0
TRACE_GROW = 1.5
TRACE_PAST = 0.5
TRACE_TRIES = 12


class _Ring:
    """A closed ring: points at a length along it, the way it goes there, the length closest to a point near one."""

    def __init__(self, pts: Sequence[XY]) -> None:
        self.pts = [(float(x), float(y)) for x, y in pts] + [(float(pts[0][0]), float(pts[0][1]))]
        self.cum = [0.0]
        for (ax, ay), (bx, by) in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(bx - ax, by - ay))
        self.length = self.cum[-1]

    def at(self, u: float) -> XY:
        u %= self.length
        k = min(bisect.bisect_right(self.cum, u) - 1, len(self.pts) - 2)
        f = (u - self.cum[k]) / max(self.cum[k + 1] - self.cum[k], 1e-12)
        (ax, ay), (bx, by) = self.pts[k], self.pts[k + 1]
        return ax + f * (bx - ax), ay + f * (by - ay)

    def heading(self, u: float) -> float:
        (ax, ay), (bx, by) = self.at(u - 0.02), self.at(u + 0.02)
        return math.atan2(by - ay, bx - ax)

    def project(self, x: float, y: float, u0: float, span: float) -> Tuple[float, float]:
        """(the length along it of the point closest to (x, y) from u0 - span / 4 to u0 + span, the distance)."""
        best, found = math.inf, u0
        lo, hi = u0 - 0.25 * span, u0 + span
        n = len(self.pts) - 1
        for off in (-self.length, 0.0, self.length):
            a = max(0, bisect.bisect_right(self.cum, lo - off) - 1)
            b = min(n, bisect.bisect_left(self.cum, hi - off) + 1)
            for k in range(a, b):
                (ax, ay), (bx, by) = self.pts[k], self.pts[k + 1]
                dx, dy = bx - ax, by - ay
                ll = dx * dx + dy * dy
                t = 0.0 if ll < 1e-18 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / ll))
                d = (ax + t * dx - x) ** 2 + (ay + t * dy - y) ** 2
                u = self.cum[k] + t * math.sqrt(ll) + off
                if lo <= u <= hi and d < best:
                    best, found = d, u
        return found, math.sqrt(best)


def _within(u: float, a: float, b: float, length: float) -> bool:
    """Whether u lies from a to b along a ring that long (any time round)."""
    return (u - a) % length <= b - a


class _Driver:
    """Drives along a ring (the side to keep to: inside(x, y)), the body fitting (fits(x, y, yaw))."""

    def __init__(self, ring: _Ring, fits: Callable, inside: Callable) -> None:
        self.ring, self.fits, self.inside = ring, fits, inside
        self.inside_many = lambda xs, ys: [bool(v) for v in inside(xs, ys)] if contains_xy is not None else \
            inside(xs, ys)
        self.most = TRACE_STEP / TRACE_R
        self.grid = [-self.most + j * TRACE_TURN for j in range(int(2.0 * self.most / TRACE_TURN) + 1)]

    def _step(self, p: Pose2D, want: float) -> Optional[Pose2D]:
        """The next pose from p, on an arc turning the heading by the turn closest to want (TRACE_TURN apart, as far
        as the tightest turn) where it keeps inside and the body fits, and TRACE_AHEAD poses straight on fit too (else
        the first that fits only there)."""
        turns = [want] + sorted(self.grid, key=lambda t: abs(t - want))
        nxt = [(p.x + TRACE_STEP * math.cos(p.yaw + 0.5 * t), p.y + TRACE_STEP * math.sin(p.yaw + 0.5 * t),
                normalize_angle(p.yaw + t)) for t in turns]
        first = None
        ok = self.inside_many([x for x, _, _ in nxt], [y for _, y, _ in nxt])
        for (x, y, h), good in zip(nxt, ok):
            if not good or not self.fits(x, y, h):
                continue
            first = first or Pose2D(x, y, h)
            c, s = math.cos(h), math.sin(h)
            if all(self.fits(x + k * TRACE_STEP * c, y + k * TRACE_STEP * s, h) for k in range(1, TRACE_AHEAD + 1)):
                return Pose2D(x, y, h)
        return first

    def drive(self, start: Pose2D, u: float, done: Callable, limit: float) -> Optional[Tuple[List[Pose2D], list]]:
        """From start (at u along the ring) on until done(poses, along) says where to stop (an index, or None to go
        on), limit m at most: (poses, along) up to there, None where it gets stuck or doesn't stop by then."""
        ring = self.ring
        poses, along = [start], [u]
        wide: List[Tuple[float, float, float]] = []  # (from, to, how much further it looks)
        tries = 0
        while len(poses) * TRACE_STEP < limit:
            p = poses[-1]
            u, _ = ring.project(p.x, p.y, along[-1], 0.6 + TRACE_LOOK)
            grow = max([g for a, b, g in wide if _within(u, a, b, ring.length)], default=1.0)
            tx, ty = ring.at(u + TRACE_LOOK * grow)
            want = max(-self.most, min(self.most, normalize_angle(math.atan2(ty - p.y, tx - p.x) - p.yaw)))
            found = self._step(p, want)
            if found is not None:
                poses.append(found)
                along.append(u)
                end = done(poses, along)
                if end is not None:
                    return poses[:end + 1], along[:end + 1]
                continue
            tries += 1
            if tries > TRACE_TRIES:
                return None
            grow *= TRACE_GROW
            back = TRACE_BACK * TRACE_LOOK * grow
            wide.append((u - back, u + TRACE_PAST + TRACE_LOOK * grow, grow))
            k = len(along) - 1
            while k > 0 and along[k] > u - back - 0.1:
                k -= 1
            del poses[k + 1:], along[k + 1:]
        return None


def _inside(pts: Sequence[XY], hole: bool, room=None) -> Callable:
    """inside(x, y) of a ring driven: within TRACE_OUT outside an outline, TRACE_IN into an island (hole), and in room
    (where the loops' centres may be) where given."""
    shape = Polygon(pts)
    region = shape.buffer(TRACE_OUT, 4) if not hole else \
        (room if room is not None else shape.buffer(TRACE_OUT + 1.0, 4)).difference(shape.buffer(-TRACE_IN, 4))
    if room is not None:
        region = region.intersection(room)
    if contains_xy is not None:
        prep(region)  # (shapely 2 prepares it in place: the many points tested much quicker)
        return lambda x, y: contains_xy(region, x, y)
    kept = prep(region)
    return lambda x, y: [kept.contains(Point(a, b)) for a, b in zip(x, y)] if isinstance(x, list) else \
        kept.contains(Point(x, y))
