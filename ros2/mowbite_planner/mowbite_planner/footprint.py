"""The mower's body around the point its path is for (OpenMower's base_link, the middle between the drive
wheels): its outline (robot_outline, or a rectangle robot_front ahead of it, robot_rear behind, robot_width wide).
Checks whether it stays in the space it may take at poses, turning on the spot where the heading jumps, spin_margin
further in there and on the turns between lanes."""

import math
from typing import List, Optional, Sequence, Tuple, Union

from shapely.geometry import MultiPoint, Point, Polygon, box
from shapely.ops import clip_by_rect, polylabel, unary_union
from shapely.prepared import prep

from .geometry import QUAD_SEGS, polygons as pieces
from .mathutil import angle_diff, normalize_angle

try:  # shapely 2 tests points without making a geometry of each and bodies all at once, much quicker
    import numpy
    from shapely import (contains_xy, convex_hull, covers, distance, get_coordinates, multipoints, points, polygons,
                         segmentize)
    contains_points = None
except ImportError:  # shapely 1.7 (ROS Noetic)
    contains_xy = None
    try:  # many points at once (a call takes as long as about 60 of them)
        import numpy
        from shapely.vectorized import contains as contains_points
    except ImportError:
        contains_points = None

# rad, a heading that changes more than this from one pose to the next is a turn on the spot there: at a kink that sharp
# OpenMower's FTC turns the same way, over the way from the pose before (in a simulation of it the body swung out over
# the real edge at one of 29 degrees, at gentler ones a few cm into its margins at most)
SPIN = math.radians(15.0)
# rad, a turn on the spot is checked with the body this far apart (its corners pass < 2 mm outside of them)
TURN_STEP = math.radians(10.0)
# m, with a spin_margin a turn on the spot is checked anywhere on this much of the way in to it: OpenMower's FTC turns
# over the way from the pose before (waypoint_spacing), halfway round the mower was up to 7 cm before the point
SPIN_BACK = 0.1
# outside(): at most about this many bodies along the poses, turning on the spot one every this many rad
OUTSIDE_BODIES = 12
OUTSIDE_TURN = math.radians(20.0)
# m, out_by() looks at points this far apart along the body's edge (turning on the spot along the way its corners go,
# OUT_ARC apart at least: how far out they are changes smoothly there), at the space's edges OUT_NEAR round them first
OUT_STEP = 0.004
OUT_ARC = 0.01
OUT_NEAR = 0.15
# fits_turn's answers kept (the same turn again, to the bit: the same answer), cleared at this many
TURNS_KEPT = 100000
# m, with shapely 1.7 squares of the map this big are looked up once whether all of them lies in a _Zone
CELL = 0.1
# m, with shapely 2 a _Zone knows of every square this big at once whether all of it lies in it
RASTER = 0.03
# space.buffer(-d / KEEP_OFF, 4) keeps d off space's edge: GEOS rounds the corners with sides of up to 1.5 times a
# quarter circle's fourth (their middles 1 - cos closer), on an edge simplified by 1 % of the distance first
KEEP_OFF = math.cos(1.5 * math.pi / 16) - 0.01
# which poses lie on a turn between lanes, where the body keeps spin_margin further off the edges (on a U-turn or a loop
# the mower keeps less exactly to its path than going straight, a few cm out, its corners swinging further): all but
# the first (True: the turn, where it comes from first), none, or those of these that are True (one for each pose)
OnTurn = Union[bool, Sequence[bool]]


class _Zone:
    """An area points are looked up in again and again, whether they're in it for sure."""

    def __init__(self, geom):
        self.geom = geom
        self.prepared = prep(geom)  # (shapely 2 prepares geom itself)
        self._cells: dict = {}
        self._squares = None

    def has(self, x: float, y: float) -> bool:
        # (a point looked up takes about as long as checking the body: whether its square of the map lies all in it)
        if contains_xy is not None:
            if self._squares is None:
                self._squares = self._raster()
            x0, y0, nx, ny, inside = self._squares
            i, j = math.floor((x - x0) / RASTER), math.floor((y - y0) / RASTER)
            return 0 <= i < nx and 0 <= j < ny and inside[j * nx + i] == 1
        key = (math.floor(x / CELL), math.floor(y / CELL))
        inside = self._cells.get(key)
        if inside is None:
            square = box(key[0] * CELL, key[1] * CELL, (key[0] + 1) * CELL, (key[1] + 1) * CELL)
            inside = self._cells[key] = self.prepared.covers(square)
        return inside

    def _raster(self):
        # (x0, y0, nx, ny, of every RASTER square row by row 1 where all of it lies in the zone): none near its edge,
        # where a point of it (RASTER / 2 apart, every point of the edge RASTER / 4 from one) lies in the square or one
        # next to it. The others lie on one side of the edge, along a row the same as the first after one near it
        x0, y0, x1, y1 = self.geom.bounds
        x0, y0 = x0 - 2.0 * RASTER, y0 - 2.0 * RASTER
        nx, ny = int((x1 - x0) / RASTER) + 3, int((y1 - y0) / RASTER) + 3
        near = numpy.zeros((ny + 2, nx + 2), bool)
        edge = get_coordinates(segmentize(self.geom.boundary, 0.5 * RASTER))
        near[((edge[:, 1] - y0) / RASTER).astype(int) + 1, ((edge[:, 0] - x0) / RASTER).astype(int) + 1] = True
        near = near[:-2] | near[1:-1] | near[2:]
        near = near[:, :-2] | near[:, 1:-1] | near[:, 2:]
        first = ~near
        first[:, 1:] &= near[:, :-1]
        j, i = numpy.nonzero(first)
        inside = contains_xy(self.geom, x0 + (i + 0.5) * RASTER, y0 + (j + 0.5) * RASTER)
        return x0, y0, nx, ny, (~near.ravel() & inside[numpy.cumsum(first.ravel()) - 1]).tobytes()

    def inside(self, x, y):
        """Whether the points (arrays of x and y) lie in it, exactly (shapely 1.7, with numpy)."""
        return contains_points(self.prepared, x, y)


class Footprint:
    def __init__(self, space, corners: Sequence[Tuple[float, float]], spin_margin: float = 0.0, back: float = 0.0):
        # the body's corners round the centre (x ahead, y left), how far it reaches ahead, behind and to either side
        self.corners = tuple(corners)
        xs, ys = [x for x, _ in self.corners], [y for _, y in self.corners]
        self.front, self.rear, self.half = max(xs), -min(xs), max(max(ys), -min(ys))
        # the farthest any part of the body gets from the centre
        self.reach = max(math.hypot(x, y) for x, y in self.corners)
        self.space = space
        self._space = prep(space)
        # the front half and the rear half (of the rectangle round the body) each lie in a circle round their middle:
        # with both middles in here the body fits
        quarter = 0.25 * (self.front + self.rear)
        self._middles = (self.front - quarter, quarter - self.rear)  # m ahead of the centre
        radius = math.hypot(quarter, self.half)
        # (back: the zones that much further in, a centre in one fits anywhere within back of it, see turning)
        self.back = back
        halves = space.buffer(-(radius + back) / KEEP_OFF, 4)
        self._halves = None if halves.is_empty else _Zone(halves)
        # a centre in here fits whatever way it faces, reach off the edges (that much less off the zone above, quicker
        # than off the edges themselves; off them where the body doesn't reach as far as the circles, a round one)
        self.anywhere = halves.buffer(-(self.reach - radius) / KEEP_OFF, 4) if self.reach >= radius else \
            space.buffer(-(self.reach + back) / KEEP_OFF, 4)
        self._safe = None if self.anywhere.is_empty else _Zone(self.anywhere)
        # turning on the spot (fits_turn) the body keeps spin_margin further off the edges, from SPIN_BACK before on,
        # on a turn between lanes (OnTurn) at the pose
        self.spin_margin = spin_margin
        self.turning = Footprint(space.buffer(-spin_margin, QUAD_SEGS), self.corners, back=SPIN_BACK) \
            if spin_margin > 0 else self
        self._edge = None  # (space's boundary, see out_by)
        self._turns_kept: dict = {}

    def body(self, x: float, y: float, yaw: float) -> Polygon:
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in self.corners]
        return polygons(corners) if contains_xy is not None else Polygon(corners)  # (shapely 2's class is slower)

    @staticmethod
    def _turns(yaw0: float, yaw1: float) -> List[List[float]]:
        """The headings turning on the spot from yaw0 to yaw1, TURN_STEP apart: the shorter way round, both ways where
        they're within TURN_STEP as long (about a half turn: the mower may take either)."""
        turn = normalize_angle(yaw1 - yaw0)
        turns = (turn, turn - math.copysign(2.0 * math.pi, turn)) if abs(turn) > math.pi - TURN_STEP else (turn,)
        return [[yaw0 + t * i / n for i in range(n + 1)] for t in turns
                for n in [max(1, int(math.ceil(abs(t) / TURN_STEP)))]]

    def _corners_at(self, x: float, y: float, yaws: Sequence, dx: float, dy: float):
        """The body's corners facing each of yaws at (x, y) and (dx, dy) back from there, twice its corners a heading
        one after the other (with shapely 2 an array, all at once)."""
        if contains_xy is None:
            return [(x - k * dx + math.cos(h) * fx - math.sin(h) * fy, y - k * dy + math.sin(h) * fx + math.cos(h) * fy)
                    for h in yaws for k in (0, 1) for fx, fy in self.corners]
        h = numpy.array(yaws, float)[:, None]
        (fx, fy), c, s = numpy.array(self.corners).T, numpy.cos(h), numpy.sin(h)
        corners = numpy.stack([x + c * fx - s * fy, y + s * fx + c * fy], -1)
        return numpy.concatenate([corners, corners - (dx, dy)], 1).reshape(-1, 2)

    def _hulls(self, x: float, y: float, turns: Sequence, dx: float, dy: float) -> list:
        """The hull of the body facing a and b of each (a, b) of turns, at (x, y) and (dx, dy) back from there."""
        corners = self._corners_at(x, y, [h for turn in turns for h in turn], dx, dy)
        n = 4 * len(self.corners)  # (a turn's: facing a and b, at (x, y) and back)
        if contains_xy is None:
            return [MultiPoint(corners[n * k:n * k + n]).convex_hull for k in range(len(turns))]
        return list(convex_hull(multipoints(corners.reshape(len(turns), n, 2))))

    def _turned(self, x: float, y: float, turns: Sequence, dx: float, dy: float) -> bool:
        """Whether the body fits turning from a to b of each (a, b) of turns, anywhere from (x, y) to (dx, dy) back:
        the hulls of it facing a and b at both ends."""
        hulls = self._hulls(x, y, turns, dx, dy)
        return bool(covers(self.space, hulls).all()) if contains_xy is not None else \
            all(self._space.covers(hull) for hull in hulls)

    def _safe_at(self, x: float, y: float) -> bool:
        return self._safe is not None and self._safe.has(x, y)

    def _halves_at(self, x: float, y: float, yaw: float) -> bool:
        if self._halves is None:
            return False
        (front, rear), c, s = self._middles, math.cos(yaw), math.sin(yaw)
        return self._halves.has(x + c * front, y + s * front) and self._halves.has(x + c * rear, y + s * rear)

    def fits_pose(self, x: float, y: float, yaw: float) -> bool:
        return self._safe_at(x, y) or self._halves_at(x, y, yaw) or self._space.covers(self.body(x, y, yaw))

    def fit(self, poses: Sequence, on_turn: OnTurn = False) -> List[bool]:
        """fits_pose for each of the poses (with .x, .y, .yaw), on a turn between lanes (on_turn) spin_margin further
        off the edges. With shapely 2 the bodies of those not in a zone all at once (with shapely 1.7 the points in the
        zones, where numpy is there)."""
        turn = self._on_turn(poses, on_turn)
        if any(turn) and self.turning is not self:
            on = iter(self.turning.fit([p for p, t in zip(poses, turn) if t]))
            rest = iter(self.fit([p for p, t in zip(poses, turn) if not t]))
            return [next(on) if t else next(rest) for t in turn]
        if contains_xy is not None:
            fit = [self._safe_at(p.x, p.y) or self._halves_at(p.x, p.y, p.yaw) for p in poses]
            rest = [p for p, f in zip(poses, fit) if not f]
            if rest:
                x, y, yaw = (column[:, None] for column in numpy.array([(p.x, p.y, p.yaw) for p in rest]).T)
                (dx, dy), c, s = numpy.array(self.corners).T, numpy.cos(yaw), numpy.sin(yaw)
                corners = numpy.stack([x + c * dx - s * dy, y + s * dx + c * dy], -1)
                ok = iter(covers(self.space, polygons(corners)).tolist())  # (prep() above prepared space)
                fit = [f or next(ok) for f in fit]
            return fit
        if contains_points is None or not poses:
            return [self.fits_pose(p.x, p.y, p.yaw) for p in poses]
        x, y, yaw = numpy.array([(p.x, p.y, p.yaw) for p in poses]).T
        fit = self._safe.inside(x, y) if self._safe is not None else numpy.zeros(len(x), bool)
        if self._halves is not None:
            (front, rear), c, s = self._middles, numpy.cos(yaw), numpy.sin(yaw)
            fit |= self._halves.inside(x + c * front, y + s * front) & self._halves.inside(x + c * rear, y + s * rear)
        return [f or self._space.covers(self.body(p.x, p.y, p.yaw)) for f, p in zip(fit.tolist(), poses)]

    def fits_turn(self, x: float, y: float, yaw0: float, yaw1: float) -> bool:
        """Turning on the spot here from yaw0 to yaw1, the shorter way round (body poses TURN_STEP apart), both ways
        round where they're within TURN_STEP as long (about a half turn: the mower may take either). With a
        spin_margin the body keeps that much further off the edges, turning anywhere on the last SPIN_BACK of the way
        in (facing yaw0), checked from one of those poses to the next (the hull of both: between them a corner may
        get closer to an edge than at either)."""
        key = (x, y, yaw0, yaw1)
        got = self._turns_kept.get(key)
        if got is None:
            if len(self._turns_kept) >= TURNS_KEPT:
                self._turns_kept.clear()
            got = self._turns_kept[key] = self._fits_turn(x, y, yaw0, yaw1)
        return got

    def _fits_turn(self, x: float, y: float, yaw0: float, yaw1: float) -> bool:
        fp = self.turning
        if fp._safe_at(x, y):
            return True
        dx, dy = fp.back * math.cos(yaw0), fp.back * math.sin(yaw0)
        for yaws in self._turns(yaw0, yaw1):
            if fp is self:
                if not all(self._halves_at(x, y, yaw) or self._space.covers(self.body(x, y, yaw)) for yaw in yaws):
                    return False
                continue
            rest = [(a, b) for a, b in zip(yaws, yaws[1:]) if not (fp._halves_at(x, y, a) and fp._halves_at(x, y, b))]
            if rest and not fp._turned(x, y, rest, dx, dy):
                return False
        return True

    def spins(self, poses: Sequence, i: int) -> bool:
        """Whether the mower turns on the spot at pose i: it arrives facing the way the pose before points and
        leaves facing pose i's way."""
        return i > 0 and angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN

    @staticmethod
    def _on_turn(poses: Sequence, on_turn: OnTurn) -> List[bool]:
        if on_turn is True or on_turn is False:
            return [on_turn and i > 0 for i in range(len(poses))]
        return [bool(t) for t in on_turn]

    def _bad(self, poses: Sequence, last_first: bool = False, on_turn: OnTurn = False):
        # (shapely 1.7 one at a time: poses_fit stops at the first that doesn't fit)
        turn = self._on_turn(poses, on_turn)
        fit = self.fit(poses, turn) if contains_xy is not None else None
        for i in range(len(poses) - 1, -1, -1) if last_first else range(len(poses)):
            p = poses[i]
            if self.spins(poses, i):
                ok = self.fits_turn(p.x, p.y, poses[i - 1].yaw, p.yaw)
            else:
                ok = fit[i] if fit is not None else (self.turning if turn[i] else self).fits_pose(p.x, p.y, p.yaw)
            if not ok:
                yield i

    def bad_poses(self, poses: Sequence, on_turn: OnTurn = False) -> list:
        """Indices of the poses (with .x, .y, .yaw) where the body sticks out: facing the pose's way (on a turn
        between lanes, on_turn, spin_margin further in), or turning on the spot where the heading jumps by more than
        SPIN from the pose before."""
        return list(self._bad(poses, on_turn=on_turn))

    def outside(self, poses: Sequence) -> float:
        """m2, about how much of the body sticks out at the poses all together, turning on the spot where the heading
        jumps by more than SPIN (OUTSIDE_BODIES bodies at most and every OUTSIDE_TURN turning: to compare ways)."""
        bodies = []
        every = max(1, int(math.ceil(len(poses) / OUTSIDE_BODIES)))
        for i, p in enumerate(poses):
            if self.spins(poses, i):
                turn = normalize_angle(p.yaw - poses[i - 1].yaw)
                n = max(1, int(math.ceil(abs(turn) / OUTSIDE_TURN)))
                bodies += [self.body(p.x, p.y, poses[i - 1].yaw + turn * k / n) for k in range(n + 1)]
            elif (i % every == 0 or i == len(poses) - 1) and not self._safe_at(p.x, p.y):
                bodies.append(self.body(p.x, p.y, p.yaw))
        if not bodies:
            return 0.0
        swept = unary_union(bodies)
        # (only the space around them: the whole of it can be a big polygon)
        return swept.difference(self.space.intersection(box(*swept.bounds))).area

    def out_by(self, turns: Sequence, step: float = OUT_STEP, around: Optional[Sequence[float]] = None,
               on_turn: OnTurn = False) -> List[Tuple]:
        """(m, (x, y)) for each pose of turns ((the pose before, None where a section starts, the pose)): how far the
        body's farthest point lies out of the space and where (0, None: it's in it), turning on the spot where the
        heading jumps by more than SPIN: out of the space spin_margin smaller, anywhere on the last SPIN_BACK of the way
        in (like fits_turn), on a turn between lanes (on_turn: all, none, or one for each of turns) out of that space
        at the pose. Of points `step` apart along its edges (turning: facing the way in and out, at the point and back,
        its corners every OUT_ARC round at least), step / 2 short of the farthest at most; around (x, y, r): of those
        within r of there only. Where none of those lies out of it (or without numpy) of the corners and the middle of
        what does."""
        got: List[Tuple] = [(0.0, None)] * len(turns)
        spin = [before is not None and angle_diff(p.yaw, before.yaw) > SPIN for before, p in turns]
        turn = [bool(on_turn)] * len(turns) if on_turn is True or on_turn is False else [bool(t) for t in on_turn]
        if contains_xy is not None or contains_points is not None:
            # (the ends of the edges the points lie on, whose: the bodies not turning all at once)
            still = [k for k, turning in enumerate(spin) if not turning]
            x, y, h = numpy.array([(turns[k][1].x, turns[k][1].y, turns[k][1].yaw) for k in still]).reshape(-1, 3).T
            (fx, fy), cos, sin = numpy.array(self.corners).T, numpy.cos(h)[:, None], numpy.sin(h)[:, None]
            c = numpy.stack([x[:, None] + cos * fx - sin * fy, y[:, None] + sin * fx + cos * fy], -1)
            m = len(self.corners)
            ends, owner = [(c.reshape(-1, 2), numpy.roll(c, -1, 1).reshape(-1, 2))], [numpy.repeat(still, m)]
            for k in (k for k, turning in enumerate(spin) if turning):
                (before, p), yaw0 = turns[k], turns[k][0].yaw
                dx, dy = self.turning.back * math.cos(yaw0), self.turning.back * math.sin(yaw0)
                for c in (self._corners_at(p.x, p.y, [yaw0], dx, dy), self._corners_at(p.x, p.y, [p.yaw], dx, dy)):
                    ends += [(c[:m], numpy.roll(c[:m], -1, 0)), (c[m:], numpy.roll(c[m:], -1, 0)), (c[:m], c[m:])]
                for t in self._turns(yaw0, p.yaw):  # (its corners turning either way it may)
                    n = int(math.ceil(abs(t[-1] - yaw0) * self.reach / max(step, OUT_ARC)))
                    c = self._corners_at(p.x, p.y, yaw0 + (t[-1] - yaw0) * numpy.arange(n + 1) / n, dx, dy)
                    ends.append((c[:-2 * m], c[2 * m:]))
                owner += [numpy.full(len(e[0]), k) for e in ends[len(owner):]]
            a, b = (numpy.concatenate([e[i] for e in ends]).reshape(-1, 2) for i in (0, 1))
            n = numpy.maximum(1, numpy.ceil(numpy.hypot(*(b - a).T) / step)).astype(int)
            j = numpy.repeat(numpy.arange(len(a)), n)
            xy = a[j] + ((numpy.arange(n.sum()) - numpy.repeat(numpy.cumsum(n) - n, n)) / n[j])[:, None] * (b - a)[j]
            owner = numpy.concatenate(owner).astype(int)[j]
            if around is not None:
                near = numpy.hypot(xy[:, 0] - around[0], xy[:, 1] - around[1]) <= around[2]
                xy, owner = xy[near], owner[near]
            spaces = [self.turning if turning or t else self for turning, t in zip(spin, turn)]
            for fp in set(spaces):  # (the points out of each space, how far from its edges round them)
                mine = numpy.array([space is fp for space in spaces], bool)[owner]
                q, o = xy[mine], owner[mine]
                inside = contains_xy(fp.space, q[:, 0], q[:, 1]) if contains_xy is not None else \
                    contains_points(fp._space, q[:, 0], q[:, 1])
                q, o = q[~inside], o[~inside]
                if not len(q):
                    continue
                if fp._edge is None:
                    fp._edge = fp.space.boundary
                (x0, y0), (x1, y1) = q.min(0), q.max(0)
                for r in (OUT_NEAR, fp.reach + fp.back):  # (all of them within reach where they're further out)
                    edge = clip_by_rect(fp._edge, x0 - r, y0 - r, x1 + r, y1 + r)
                    d = distance(edge, points(q)) if contains_xy is not None else \
                        numpy.array([edge.distance(Point(*xy)) for xy in q])
                    if d.max() < r:
                        break
                order = numpy.lexsort((d, o))  # (by whose, the farthest last)
                for i in order[numpy.r_[o[order][1:] != o[order][:-1], True]]:
                    got[o[i]] = (round(float(d[i]), 3), (float(q[i][0]), float(q[i][1])))
        for k, ((before, p), turning) in enumerate(zip(turns, spin)):
            if got[k][1] is None:  # (the corners and the middle of what lies out of the space)
                fp = self.turning if turning or turn[k] else self
                shape = unary_union([h for t in self._turns(before.yaw, p.yaw) for h in self._hulls(
                    p.x, p.y, list(zip(t, t[1:])), fp.back * math.cos(before.yaw), fp.back * math.sin(before.yaw))]) \
                    if turning else self.body(p.x, p.y, p.yaw)
                for piece in pieces(shape.difference(fp.space)):
                    for q in [polylabel(piece, OUT_STEP)] + [Point(xy) for xy in piece.exterior.coords]:
                        if fp.space.distance(q) > got[k][0]:
                            got[k] = (round(fp.space.distance(q), 3), (q.x, q.y))
        return got

    def poses_fit(self, poses: Sequence, on_turn: OnTurn = False) -> bool:
        # (the last ones first: a way or a turn that sticks out mostly does so further on)
        return next(self._bad(poses, True, on_turn), None) is None
