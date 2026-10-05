"""The mower's body around the point its path is for (OpenMower's base_link, the middle between the drive
wheels): a rectangle robot_front ahead of it, robot_rear behind, robot_width wide. Checks whether it stays in
the space it may take at poses, turning on the spot where the heading jumps."""

import math
from typing import List, Sequence

from shapely.geometry import Polygon, box
from shapely.ops import unary_union
from shapely.prepared import prep

from .mathutil import angle_diff, normalize_angle

try:  # shapely 2 tests points without making a geometry of each and bodies all at once, much quicker
    import numpy
    from shapely import contains_xy, covers, get_coordinates, polygons, segmentize
    contains_points = None
except ImportError:  # shapely 1.7 (ROS Noetic)
    contains_xy = None
    try:  # many points at once (a call takes as long as about 60 of them)
        import numpy
        from shapely.vectorized import contains as contains_points
    except ImportError:
        contains_points = None

# rad, a heading that changes more than this from one pose to the next is a turn on the spot there
SPIN = math.radians(30.0)
# rad, a turn on the spot is checked with the body this far apart (its corners pass < 2 mm outside of them)
TURN_STEP = math.radians(10.0)
# outside(): at most about this many bodies along the poses, turning on the spot one every this many rad
OUTSIDE_BODIES = 12
OUTSIDE_TURN = math.radians(20.0)
# m, with shapely 1.7 squares of the map this big are looked up once whether all of them lies in a _Zone
CELL = 0.1
# m, with shapely 2 a _Zone knows of every square this big at once whether all of it lies in it
RASTER = 0.03
# space.buffer(-d / KEEP_OFF, 4) keeps d off space's edge: GEOS rounds the corners with sides of up to 1.5 times a
# quarter circle's fourth (their middles 1 - cos closer), on an edge simplified by 1 % of the distance first
KEEP_OFF = math.cos(1.5 * math.pi / 16) - 0.01


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
    def __init__(self, space, width: float, front: float, rear: float):
        self.half = 0.5 * width
        self.front, self.rear = front, rear
        self._corners = ((front, self.half), (-rear, self.half), (-rear, -self.half), (front, -self.half))
        # the farthest any part of the body gets from the centre
        self.reach = max(math.hypot(front, self.half), math.hypot(rear, self.half))
        self.space = space
        self._space = prep(space)
        # the front half and the rear half each lie in a circle round their middle: with both middles in here the body
        # fits
        quarter = 0.25 * (front + rear)
        self._middles = (front - quarter, quarter - rear)  # m ahead of the centre
        radius = math.hypot(quarter, self.half)
        halves = space.buffer(-radius / KEEP_OFF, 4)
        self._halves = None if halves.is_empty else _Zone(halves)
        # a centre in here fits whatever way it faces, reach off the edges (that much less off the zone above, quicker
        # than off the edges themselves)
        self.anywhere = halves.buffer(-(self.reach - radius) / KEEP_OFF, 4)
        self._safe = None if self.anywhere.is_empty else _Zone(self.anywhere)

    def body(self, x: float, y: float, yaw: float) -> Polygon:
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in self._corners]
        return polygons(corners) if contains_xy is not None else Polygon(corners)  # (shapely 2's class is slower)

    def _safe_at(self, x: float, y: float) -> bool:
        return self._safe is not None and self._safe.has(x, y)

    def _halves_at(self, x: float, y: float, yaw: float) -> bool:
        if self._halves is None:
            return False
        (front, rear), c, s = self._middles, math.cos(yaw), math.sin(yaw)
        return self._halves.has(x + c * front, y + s * front) and self._halves.has(x + c * rear, y + s * rear)

    def fits_pose(self, x: float, y: float, yaw: float) -> bool:
        return self._safe_at(x, y) or self._halves_at(x, y, yaw) or self._space.covers(self.body(x, y, yaw))

    def fit(self, poses: Sequence) -> List[bool]:
        """fits_pose for each of the poses (with .x, .y, .yaw), with shapely 2 the bodies of those not in a zone all
        at once (with shapely 1.7 the points in the zones, where numpy is there)."""
        if contains_xy is not None:
            fit = [self._safe_at(p.x, p.y) or self._halves_at(p.x, p.y, p.yaw) for p in poses]
            rest = [p for p, f in zip(poses, fit) if not f]
            if rest:
                x, y, yaw = (column[:, None] for column in numpy.array([(p.x, p.y, p.yaw) for p in rest]).T)
                (dx, dy), c, s = numpy.array(self._corners).T, numpy.cos(yaw), numpy.sin(yaw)
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
        round where they're within TURN_STEP as long (about a half turn: the mower may take either)."""
        turn = normalize_angle(yaw1 - yaw0)
        turns = (turn, turn - math.copysign(2.0 * math.pi, turn)) if abs(turn) > math.pi - TURN_STEP else (turn,)
        for turn in turns:
            n = max(1, int(math.ceil(abs(turn) / TURN_STEP)))
            if not all(self.fits_pose(x, y, yaw0 + turn * i / n) for i in range(n + 1)):
                return False
        return True

    def spins(self, poses: Sequence, i: int) -> bool:
        """Whether the mower turns on the spot at pose i: it arrives facing the way the pose before points and
        leaves facing pose i's way."""
        return i > 0 and angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN

    def _bad(self, poses: Sequence, last_first: bool = False):
        # (shapely 1.7 one at a time: poses_fit stops at the first that doesn't fit)
        fit = self.fit(poses) if contains_xy is not None else None
        for i in range(len(poses) - 1, -1, -1) if last_first else range(len(poses)):
            p = poses[i]
            if self.spins(poses, i):
                ok = self.fits_turn(p.x, p.y, poses[i - 1].yaw, p.yaw)
            else:
                ok = fit[i] if fit is not None else self.fits_pose(p.x, p.y, p.yaw)
            if not ok:
                yield i

    def bad_poses(self, poses: Sequence) -> list:
        """Indices of the poses (with .x, .y, .yaw) where the body sticks out: facing the pose's way, or turning on
        the spot where the heading jumps by more than SPIN from the pose before."""
        return list(self._bad(poses))

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

    def poses_fit(self, poses: Sequence) -> bool:
        # (the last ones first: a way or a turn that sticks out mostly does so further on)
        return next(self._bad(poses, True), None) is None
