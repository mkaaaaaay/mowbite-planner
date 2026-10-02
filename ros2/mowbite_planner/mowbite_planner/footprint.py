"""The mower's body around the point its path is for (OpenMower's base_link, the middle between the drive
wheels): a rectangle robot_front ahead of it, robot_rear behind, robot_width wide. Checks whether it stays in
the space it may take, for points along a path (the heading is the way the path goes there) or for poses."""

import math
from typing import Optional, Sequence, Tuple

from shapely.geometry import Point, Polygon
from shapely.prepared import prep

from .mathutil import angle_diff

XY = Tuple[float, float]
# rad, a heading that changes more than this from one pose to the next is a turn on the spot there
SPIN = math.radians(30.0)
SAME = 1e-6


class Footprint:
    def __init__(self, space, width: float, front: float, rear: float):
        self.half = 0.5 * width
        self.front, self.rear = front, rear
        # the farthest any part of the body gets from the centre, turning on the spot it sweeps a disk this big
        self.reach = max(math.hypot(front, self.half), math.hypot(rear, self.half))
        self.space = space
        self._space = prep(space)
        inner = space.buffer(-self.reach, 4)
        # a centre in here fits whatever way it faces
        self._safe = prep(inner) if not inner.is_empty else None

    def body(self, x: float, y: float, yaw: float) -> Polygon:
        c, s = math.cos(yaw), math.sin(yaw)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy)
                        for dx, dy in ((self.front, self.half), (-self.rear, self.half), (-self.rear, -self.half),
                                       (self.front, -self.half))])

    def _safe_at(self, x: float, y: float) -> bool:
        return self._safe is not None and self._safe.covers(Point(x, y))

    def fits_pose(self, x: float, y: float, yaw: float) -> bool:
        return self._safe_at(x, y) or self._space.covers(self.body(x, y, yaw))

    def fits_spin(self, x: float, y: float) -> bool:
        """Turning on the spot here."""
        return self._safe_at(x, y) or self._space.covers(Point(x, y).buffer(self.reach, 8))

    def fits(self, points: Sequence[XY]) -> bool:
        """Driving along the points, facing the way they go (where a point repeats, it turns on the spot)."""
        pts = list(points)
        for i, (x, y) in enumerate(pts):
            if self._safe_at(x, y):
                continue
            yaw: Optional[float] = None
            for q in pts[i + 1:i + 3]:
                if abs(q[0] - x) > SAME or abs(q[1] - y) > SAME:
                    yaw = math.atan2(q[1] - y, q[0] - x)
                    break
            if yaw is None:
                for q in reversed(pts[max(0, i - 2):i]):
                    if abs(q[0] - x) > SAME or abs(q[1] - y) > SAME:
                        yaw = math.atan2(y - q[1], x - q[0])
                        break
            spins = any(abs(q[0] - x) <= SAME and abs(q[1] - y) <= SAME
                        for q in pts[max(0, i - 1):i] + pts[i + 1:i + 2])
            if spins or yaw is None:
                if not self.fits_spin(x, y):
                    return False
            elif not self._space.covers(self.body(x, y, yaw)):
                return False
        return True

    def bad_poses(self, poses: Sequence) -> list:
        """Indices of the poses (with .x, .y, .yaw) where the body sticks out: facing the pose's way, or turning on
        the spot where the heading jumps by more than SPIN to the next one."""
        out = []
        for i, p in enumerate(poses):
            if self._safe_at(p.x, p.y):
                continue
            spin = i + 1 < len(poses) and angle_diff(poses[i + 1].yaw, p.yaw) > SPIN
            if not (self.fits_spin(p.x, p.y) if spin else self._space.covers(self.body(p.x, p.y, p.yaw))):
                out.append(i)
        return out
