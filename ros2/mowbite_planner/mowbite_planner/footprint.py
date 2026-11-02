"""The mower's body around the point its path is for (OpenMower's base_link, the middle between the drive
wheels): its outline (robot_outline, or a rectangle robot_front ahead of it, robot_rear behind, robot_width wide,
its corners rounded by robot_front_width and robot_rear_width).
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
    from shapely import (contains_xy, convex_hull, covers, distance, get_coordinates, linestrings, points, polygons,
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
# m, out_by() looks at points this far apart along the body's edge (turning on the spot along the way its corners go,
# OUT_ARC apart at least: how far out they are changes smoothly there), at the space's edges OUT_NEAR round them first
OUT_STEP = 0.004
OUT_ARC = 0.01
OUT_NEAR = 0.15
# m, how far the blade's disc may reach over an area not to mow (Blade): a loop along its line half the mower width
# off it comes a mm or two closer here and there
BLADE_SLACK = 0.002
# rad between the headings the blade's checked at turning on the spot (its middle 0.2 mm off the arc between, 0.185 m
# ahead)
BLADE_TURN = math.radians(5.0)
# fits_turn's answers kept (the same turn again, to the bit: the same answer), cleared at this many
TURNS_KEPT = 100000
# fit's and fits_pose's answers kept for poses the same way (most poses are asked about again and again: the ways
# tried round a place share most of them), cleared at this many
POSES_KEPT = 100000
# poses_fit and all_fit: every SCOUT-th pose (of those not turning on the spot) looked at first, one of them out and
# the whole is: most ways that don't fit are found so with a part of the work, the answer the same
SCOUT = 4
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


def swung(corners: Sequence[Tuple[float, float]], turn: float) -> List[Tuple[float, float]]:
    """The outline round the body turned by up to turn either way about its centre (the convex hull of the three; at
    the headings between, its corners pass outside by reach (1 - cos(turn / 2)) at most: 1 mm at 8 degrees)."""
    pts = [(c * x - s * y, s * x + c * y) for a in (-turn, 0.0, turn) for c, s in [(math.cos(a), math.sin(a))]
           for x, y in corners]
    hull = MultiPoint(pts).convex_hull
    return list(hull.exterior.coords)[:-1] if hull.geom_type == 'Polygon' else list(corners)


class Blade:
    """The blade kept off the areas not to mow next to the area (away; mkay 31.10.: the body may reach over into
    one, the blade not): its disc, half m round its middle ahead m ahead of the mower's centre and left m to its left,
    at a pose facing its way (and turned up to turn either way about the centre, see strokes.TRACK_TURN), turning on
    the spot at the headings between, BLADE_TURN apart."""

    def __init__(self, away, half: float, ahead: float, left: float, turn: float = 0.0) -> None:
        self.away, self.half, self.ahead, self.left, self.turn = away, half, ahead, left, turn
        self.key = (away.wkb, half, ahead, left, turn)
        # a blade's middle in here reaches over (GEOS's rounded corners lie inside the circle)
        self.zone = away.buffer(half - BLADE_SLACK, QUAD_SEGS)
        self._zone = prep(self.zone)
        # a centre outside this, the blade's off them whichever way it faces
        self.near = away.buffer((half + math.hypot(ahead, left)) / KEEP_OFF, 4)
        self._near = prep(self.near)

    @classmethod
    def of(cls, away, cfg) -> Optional['Blade']:
        """The blade of cfg kept off away (a geometry), None where there's nothing."""
        if away is None or away.is_empty:
            return None
        return cls(away, 0.5 * cfg.mower_width, cfg.blade_ahead, cfg.blade_offset)

    def turned(self, turn: float) -> 'Blade':
        return self if turn == self.turn else Blade(self.away, self.half, self.ahead, self.left, turn)

    def _at(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        """The blade's middle at (x, y) facing yaw."""
        c, s = math.cos(yaw), math.sin(yaw)
        return x + self.ahead * c - self.left * s, y + self.ahead * s + self.left * c

    def _headings(self, yaw0: float, yaw1: Optional[float] = None) -> List[float]:
        """Facing yaw0 (and turned either way), or turning on the spot from yaw0 to yaw1 (the shorter way round, both
        ways where they're about as long, see Footprint._turns) and turned on past both ends."""
        if yaw1 is None:
            return [yaw0 - self.turn, yaw0, yaw0 + self.turn] if self.turn else [yaw0]
        out = []
        for t in Footprint._turns(yaw0, yaw1):
            turn = t[-1] - t[0]
            a, b = yaw0 - math.copysign(self.turn, turn), yaw0 + turn + math.copysign(self.turn, turn)
            n = max(1, int(math.ceil(abs(b - a) / BLADE_TURN)))
            out += [a + (b - a) * i / n for i in range(n + 1)]
        return out

    def _over(self, xs: Sequence[float], ys: Sequence[float]) -> List[bool]:
        """Whether the blade's middles at xs, ys lie in zone."""
        if contains_xy is not None:
            return contains_xy(self.zone, numpy.asarray(xs, float), numpy.asarray(ys, float)).tolist()
        return [self._zone.contains(Point(x, y)) for x, y in zip(xs, ys)]

    def _near_at(self, xs: Sequence[float], ys: Sequence[float]) -> List[bool]:
        if contains_xy is not None:
            return contains_xy(self.near, numpy.asarray(xs, float), numpy.asarray(ys, float)).tolist()
        return [self._near.contains(Point(x, y)) for x, y in zip(xs, ys)]

    def off(self, poses: Sequence[Tuple[float, float, float]]) -> List[bool]:
        """For each (x, y, yaw) of poses: whether the blade's off the areas there."""
        if not poses:
            return []
        near = self._near_at([p[0] for p in poses], [p[1] for p in poses])
        out = [not n for n in near]
        which = [k for k, n in enumerate(near) if n]
        if which:
            xs, ys, owner = [], [], []
            for k in which:
                x, y, yaw = poses[k]
                for h in self._headings(yaw):
                    bx, by = self._at(x, y, h)
                    xs.append(bx)
                    ys.append(by)
                    owner.append(k)
            over = set(k for k, o in zip(owner, self._over(xs, ys)) if o)
            for k in which:
                out[k] = k not in over
        return out

    def off_turning(self, x: float, y: float, yaw0: float, yaw1: float, dx: float = 0.0, dy: float = 0.0) -> bool:
        """Whether the blade's off the areas turning on the spot from yaw0 to yaw1 anywhere from (x, y) to (dx, dy)
        back from there."""
        ends = [(x, y)] + ([(x - dx, y - dy)] if dx or dy else [])
        ends = [e for e, n in zip(ends, self._near_at([e[0] for e in ends], [e[1] for e in ends])) if n]
        if not ends:
            return True
        hs = self._headings(yaw0, yaw1)
        xs, ys = zip(*[self._at(ex, ey, h) for ex, ey in ends for h in hs])
        return not any(self._over(xs, ys))

    def depth(self, x: float, y: float, yaw0: float, yaw1: Optional[float] = None, dx: float = 0.0,
              dy: float = 0.0) -> Tuple[float, Optional[Tuple[float, float]]]:
        """(m, (x, y)): how far the blade's middle lies in zone at its deepest, at a pose facing yaw0 (turning on the
        spot from yaw0 to yaw1 anywhere from (x, y) to (dx, dy) back), and where; (0, None) where it's off the
        areas."""
        ends = [(x, y)] + ([(x - dx, y - dy)] if yaw1 is not None and (dx or dy) else [])
        pts = [self._at(ex, ey, h) for ex, ey in ends for h in self._headings(yaw0, yaw1)]
        best: Tuple[float, Optional[Tuple[float, float]]] = (0.0, None)
        for (bx, by), over in zip(pts, self._over([p[0] for p in pts], [p[1] for p in pts])):
            if over:
                q = Point(bx, by)
                d = self.half - BLADE_SLACK + (self.away.boundary.distance(q) if self.away.contains(q) else
                                               -self.away.distance(q))
                if best[1] is None or d > best[0]:
                    best = (max(0.0, d), (float(bx), float(by)))
        return best


class Footprint:
    def __init__(self, space, corners: Sequence[Tuple[float, float]], spin_margin: float = 0.0, back: float = 0.0,
                 blade: Optional[Blade] = None):
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
        # (blade: and the blade off the areas not to mow next to the area, see Blade)
        self.blade = blade
        if blade is not None:
            self.anywhere = self.anywhere.difference(blade.near.buffer(back / KEEP_OFF, 4) if back else blade.near)
        self._safe = None if self.anywhere.is_empty else _Zone(self.anywhere)
        # turning on the spot (fits_turn) the body keeps spin_margin further off the edges, from SPIN_BACK before on,
        # on a turn between lanes (OnTurn) at the pose
        self.spin_margin = spin_margin
        self.turning = Footprint(space.buffer(-spin_margin, QUAD_SEGS), self.corners, back=SPIN_BACK, blade=blade) \
            if spin_margin > 0 else self
        self._xy = numpy.array(self.corners).T if contains_xy is not None or contains_points is not None else None
        self._edge = None  # (space's boundary, see out_by)
        self._turns_kept: dict = {}
        self._fit_kept: dict = {}
        self._pose_kept: dict = {}
        self._plain: Optional['Footprint'] = None
        self._spun: dict = {}
        self._swung: dict = {}

    def spun(self, spin_margin: float) -> 'Footprint':
        """The same body in the same space keeping spin_margin turning (itself where that's its own)."""
        if spin_margin == self.spin_margin:
            return self
        if spin_margin not in self._spun:
            self._spun[spin_margin] = Footprint(self.space, self.corners, spin_margin, self.back, self.blade)
        return self._spun[spin_margin]

    def swung(self, turn: float) -> 'Footprint':
        """The body turned up to turn either way about its centre (swung) in the same space, without spin_margin and
        blade (one for each turn)."""
        if turn not in self._swung:
            self._swung[turn] = Footprint(self.space, swung(self.corners, turn))
        return self._swung[turn]

    @property
    def plain(self) -> 'Footprint':
        """The body's alone, without the blade (itself where it has none)."""
        if self.blade is None:
            return self
        if self._plain is None:
            self._plain = Footprint(self.space, self.corners, self.spin_margin, self.back)
        return self._plain

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
        (fx, fy), c, s = self._xy, numpy.cos(h), numpy.sin(h)
        corners = numpy.stack([x + c * fx - s * fy, y + s * fx + c * fy], -1)
        return numpy.concatenate([corners, corners - (dx, dy)], 1).reshape(-1, 2)

    def _hulls(self, x: float, y: float, turns: Sequence, dx: float, dy: float) -> list:
        """The hull of the body facing a and b of each (a, b) of turns, at (x, y) and (dx, dy) back from there."""
        corners = self._corners_at(x, y, [h for turn in turns for h in turn], dx, dy)
        n = 4 * len(self.corners)  # (a turn's: facing a and b, at (x, y) and back)
        if contains_xy is None:
            return [MultiPoint(corners[n * k:n * k + n]).convex_hull for k in range(len(turns))]
        # (GEOS takes the points of a line in the same order as those of a multipoint, the same hull, much quicker)
        return list(convex_hull(linestrings(corners.reshape(len(turns), n, 2))))

    def _turned(self, x: float, y: float, turns: Sequence, dx: float, dy: float) -> bool:
        """Whether the body fits turning from a to b of each (a, b) of turns, anywhere from (x, y) to (dx, dy) back:
        the hulls of it facing a and b at both ends. (With shapely 2 the first turn's on its own first: where the body
        sticks out turning, mostly already there.)"""
        if contains_xy is None:
            return all(self._space.covers(hull) for hull in self._hulls(x, y, turns, dx, dy))
        corners = self._corners_at(x, y, [h for turn in turns for h in turn], dx, dy)
        corners = corners.reshape(len(turns), 4 * len(self.corners), 2)
        return all(bool(covers(self.space, convex_hull(linestrings(part))).all()) for part in (corners[:1], corners[1:])
                   if len(part))

    def _safe_at(self, x: float, y: float) -> bool:
        return self._safe is not None and self._safe.has(x, y)

    def _halves_at(self, x: float, y: float, yaw: float) -> bool:
        if self._halves is None:
            return False
        (front, rear), c, s = self._middles, math.cos(yaw), math.sin(yaw)
        return self._halves.has(x + c * front, y + s * front) and self._halves.has(x + c * rear, y + s * rear)

    def fits_pose(self, x: float, y: float, yaw: float) -> bool:
        key = (x, y, yaw)
        got = self._pose_kept.get(key)
        if got is None:
            if len(self._pose_kept) >= POSES_KEPT:
                self._pose_kept.clear()
            got = self._pose_kept[key] = self._safe_at(x, y) or (
                self._halves_at(x, y, yaw) or self._space.covers(self.body(x, y, yaw))) and \
                (self.blade is None or self.blade.off([key])[0])
        return got

    def fit(self, poses: Sequence, on_turn: OnTurn = False) -> List[bool]:
        """fits_pose for each of the poses (with .x, .y, .yaw), on a turn between lanes (on_turn) spin_margin further
        off the edges. With shapely 2 the bodies of those not in a zone all at once (with shapely 1.7 the points in the
        zones, where numpy is there)."""
        if on_turn is not False:
            turn = self._on_turn(poses, on_turn)
            if any(turn) and self.turning is not self:
                on = iter(self.turning.fit([p for p, t in zip(poses, turn) if t]))
                rest = iter(self.fit([p for p, t in zip(poses, turn) if not t]))
                return [next(on) if t else next(rest) for t in turn]
        keys, kept = [(p.x, p.y, p.yaw) for p in poses], self._fit_kept
        fit = list(map(kept.get, keys))
        if None in fit:
            new = list(dict.fromkeys(k for k, f in zip(keys, fit) if f is None))
            got = dict(zip(new, self._fit(new)))
            if len(kept) + len(got) > POSES_KEPT:
                kept.clear()
            kept.update(got)
            fit = [got[k] if f is None else f for k, f in zip(keys, fit)]
        return fit

    def _fit(self, poses: List[Tuple[float, float, float]]) -> List[bool]:
        # (fit of each (x, y, yaw), none twice)
        if contains_xy is not None:
            fit = [self._safe_at(x, y) or self._halves_at(x, y, yaw) for x, y, yaw in poses]
            rest = [p for p, f in zip(poses, fit) if not f]
            if rest:
                x, y, yaw = (column[:, None] for column in numpy.array(rest).T)
                (dx, dy), c, s = self._xy, numpy.cos(yaw), numpy.sin(yaw)
                corners = numpy.stack([x + c * dx - s * dy, y + s * dx + c * dy], -1)
                ok = iter(covers(self.space, polygons(corners)).tolist())  # (prep() above prepared space)
                fit = [f or next(ok) for f in fit]
            return self._blade_off(poses, fit)
        if contains_points is None:
            return [self.fits_pose(*p) for p in poses]
        x, y, yaw = numpy.array(poses).T
        fit = self._safe.inside(x, y) if self._safe is not None else numpy.zeros(len(x), bool)
        if self._halves is not None:
            (front, rear), c, s = self._middles, numpy.cos(yaw), numpy.sin(yaw)
            fit |= self._halves.inside(x + c * front, y + s * front) & self._halves.inside(x + c * rear, y + s * rear)
        return self._blade_off(poses, [f or self._space.covers(self.body(*p)) for f, p in zip(fit.tolist(), poses)])

    def _blade_off(self, poses: List[Tuple[float, float, float]], fit: List[bool]) -> List[bool]:
        """fit, where the body fits out of the safe zone the blade off the areas not to mow too."""
        if self.blade is None:
            return fit
        which = [k for k, f in enumerate(fit) if f and not self._safe_at(poses[k][0], poses[k][1])]
        for k, off in zip(which, self.blade.off([poses[k] for k in which])):
            fit[k] = off
        return fit

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
        if self.blade is not None and not self.blade.off_turning(x, y, yaw0, yaw1, dx, dy):
            return False
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
            (fx, fy), cos, sin = self._xy, numpy.cos(h)[:, None], numpy.sin(h)[:, None]
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
            owner = numpy.concatenate(owner).astype(int)
            if around is not None and numpy.isfinite(a).all() and numpy.isfinite(b).all():
                # (the edges all further than r from there left out first: none of their points would be kept)
                u, w = b - a, numpy.array(around[:2], float) - a
                t = numpy.clip((u * w).sum(1) / numpy.maximum((u * u).sum(1), 1e-300), 0.0, 1.0)
                keep = numpy.hypot(*(w - t[:, None] * u).T) <= around[2] + 1e-9
                a, b, owner = a[keep], b[keep], owner[keep]
            n = numpy.maximum(1, numpy.ceil(numpy.hypot(*(b - a).T) / step)).astype(int)
            j = numpy.repeat(numpy.arange(len(a)), n)
            xy = a[j] + ((numpy.arange(n.sum()) - numpy.repeat(numpy.cumsum(n) - n, n)) / n[j])[:, None] * (b - a)[j]
            owner = owner[j]
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
                    if contains_xy is None:
                        for q in [polylabel(piece, OUT_STEP)] + [Point(xy) for xy in piece.exterior.coords]:
                            if fp.space.distance(q) > got[k][0]:
                                got[k] = (round(fp.space.distance(q), 3), (q.x, q.y))
                        continue
                    xy = numpy.concatenate([get_coordinates(polylabel(piece, OUT_STEP)),
                                            get_coordinates(piece.exterior)])
                    for d, q in zip(distance(fp.space, points(xy)).tolist(), xy.tolist()):  # (all at once, the same)
                        if d > got[k][0]:
                            got[k] = (round(d, 3), (q[0], q[1]))
        for k, ((before, p), turning) in enumerate(zip(turns, spin) if self.blade is not None else ()):
            # (the blade's middle as far in its zone, where that's further)
            back = self.turning.back if turning else 0.0
            d, at = self.blade.depth(p.x, p.y, before.yaw if turning else p.yaw, p.yaw if turning else None,
                                     back * math.cos(before.yaw) if turning else 0.0,
                                     back * math.sin(before.yaw) if turning else 0.0)
            if at is not None and (got[k][1] is None or d > got[k][0]):
                got[k] = (round(d, 3), at)
        return got

    def poses_fit(self, poses: Sequence, on_turn: OnTurn = False) -> bool:
        # (the last ones first: a way or a turn that sticks out mostly does so further on)
        turn = self._on_turn(poses, on_turn)
        if len(poses) > 2 * SCOUT:
            some = [i for i in range(len(poses) - 1, -1, -SCOUT) if not self.spins(poses, i)]
            if not all(self.fit([poses[i] for i in some], [turn[i] for i in some])):
                return False
        return next(self._bad(poses, True, turn), None) is None

    def all_fit(self, poses: Sequence) -> bool:
        """all(fit(poses)), every SCOUT-th looked at first (not kept: these are seldom asked about again)."""
        poses = [(p.x, p.y, p.yaw) for p in poses]
        if len(poses) > 2 * SCOUT:
            if not all(self._fit(poses[SCOUT // 2::SCOUT])):
                return False
            poses = [p for k, p in enumerate(poses) if k % SCOUT != SCOUT // 2]
        return all(self._fit(poses)) if poses else True

    def all_fit_each(self, ways: Sequence[Sequence]) -> List[bool]:
        """[all_fit(way) for way in ways], the poses of all of them looked at at once (many short ways: quicker)."""
        ways = [[(p.x, p.y, p.yaw) for p in way] for way in ways]
        out = [True] * len(ways)
        for scouts in (True, False):
            which, poses = [], []
            for k, way in enumerate(ways):
                if not out[k] or len(way) <= 2 * SCOUT and scouts:
                    continue
                some = way if len(way) <= 2 * SCOUT else way[SCOUT // 2::SCOUT] if scouts else \
                    [p for i, p in enumerate(way) if i % SCOUT != SCOUT // 2]
                which += [k] * len(some)
                poses += some
            for k, fit in zip(which, self._fit(poses) if poses else []):
                out[k] = out[k] and fit
        return out
