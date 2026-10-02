"""The mower's body around the point its path is for (OpenMower's base_link, the middle between the drive
wheels): a rectangle robot_front ahead of it, robot_rear behind, robot_width wide. Checks whether it stays in
the space it may take at poses, turning on the spot where the heading jumps."""

import math
from typing import Sequence

from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely.prepared import prep

from .mathutil import angle_diff, normalize_angle

try:  # shapely 2 tests points without making a geometry of each, much quicker
    from shapely import contains_xy, prepare
except ImportError:  # shapely 1.7 (ROS Noetic)
    contains_xy = None

# rad, a heading that changes more than this from one pose to the next is a turn on the spot there
SPIN = math.radians(30.0)
# rad, a turn on the spot is checked with the body this far apart (its corners pass < 2 mm outside of them)
TURN_STEP = math.radians(10.0)
# outside(): at most about this many bodies along the poses, turning on the spot one every this many rad
OUTSIDE_BODIES = 12
OUTSIDE_TURN = math.radians(20.0)
# m, squares of the map this big are looked up once whether all of them lies where the body fits whatever way it faces
CELL = 0.1


class Footprint:
    def __init__(self, space, width: float, front: float, rear: float):
        self.half = 0.5 * width
        self.front, self.rear = front, rear
        # the farthest any part of the body gets from the centre
        self.reach = max(math.hypot(front, self.half), math.hypot(rear, self.half))
        self.space = space
        self._space = prep(space)
        inner = space.buffer(-self.reach, 4)
        # a centre in here fits whatever way it faces
        self._safe = None if inner.is_empty else inner if contains_xy is not None else prep(inner)
        if self._safe is not None and contains_xy is not None:
            prepare(inner)
        self._cells: dict = {}

    def body(self, x: float, y: float, yaw: float) -> Polygon:
        c, s = math.cos(yaw), math.sin(yaw)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy)
                        for dx, dy in ((self.front, self.half), (-self.rear, self.half), (-self.rear, -self.half),
                                       (self.front, -self.half))])

    def _safe_at(self, x: float, y: float) -> bool:
        if self._safe is None:
            return False
        if contains_xy is not None:
            return bool(contains_xy(self._safe, x, y))
        # (shapely 1.7 makes a geometry of every point: most are in a square already known to be all safe or not
        # safe at all)
        key = (math.floor(x / CELL), math.floor(y / CELL))
        safe = self._cells.get(key)
        if safe is None:  # 1 all safe, 0 none of it, -1 some
            square = box(key[0] * CELL, key[1] * CELL, (key[0] + 1) * CELL, (key[1] + 1) * CELL)
            safe = self._cells[key] = 1 if self._safe.covers(square) else 0 if self._safe.disjoint(square) else -1
        return safe == 1 if safe >= 0 else self._safe.covers(Point(x, y))

    def fits_pose(self, x: float, y: float, yaw: float) -> bool:
        return self._safe_at(x, y) or self._space.covers(self.body(x, y, yaw))

    def fits_turn(self, x: float, y: float, yaw0: float, yaw1: float) -> bool:
        """Turning on the spot here from yaw0 to yaw1, the shorter way round (body poses TURN_STEP apart)."""
        turn = normalize_angle(yaw1 - yaw0)
        n = max(1, int(math.ceil(abs(turn) / TURN_STEP)))
        return all(self.fits_pose(x, y, yaw0 + turn * i / n) for i in range(n + 1))

    def spins(self, poses: Sequence, i: int) -> bool:
        """Whether the mower turns on the spot at pose i: it arrives facing the way the pose before points and
        leaves facing pose i's way."""
        return i > 0 and angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN

    def _bad(self, poses: Sequence):
        for i, p in enumerate(poses):
            if self._safe_at(p.x, p.y):
                continue
            if not (self.fits_turn(p.x, p.y, poses[i - 1].yaw, p.yaw) if self.spins(poses, i)
                    else self._space.covers(self.body(p.x, p.y, p.yaw))):
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
        # all of them where the body fits whatever way it faces: one check for the lot
        if len(poses) > 2 and self._safe is not None and \
                self._safe.covers(LineString([(p.x, p.y) for p in poses])):
            return True
        return next(self._bad(poses), None) is None
