"""Small 2D pose helpers shared by the planner and the ROS node (no ROS imports here)."""

import math
from typing import NamedTuple, Tuple

TWO_PI = 2.0 * math.pi


class Pose2D(NamedTuple):
    x: float
    y: float
    # the way the mower faces (rad, 0 = +x of the map frame, ccw). That's the direction it drives in,
    # except on the backward legs of a K-turn, where it drives against it
    yaw: float


def normalize_angle(a: float) -> float:
    """Wraps an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


def angle_diff(a: float, b: float) -> float:
    """Smallest absolute difference between two angles (0..pi)."""
    return abs(normalize_angle(a - b))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation by yaw around +z.

    The general Euler -> quaternion formula with roll = pitch = 0 leaves only
    x = 0, y = 0, z = sin(yaw / 2), w = cos(yaw / 2). The yaw is wrapped to (-pi, pi] first so w is
    never negative, the same rotation always gives the same quaternion.
    """
    half = 0.5 * normalize_angle(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw of a quaternion (the inverse of yaw_to_quaternion for planar rotations)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
