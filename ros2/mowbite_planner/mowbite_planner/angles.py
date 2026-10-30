"""The lanes' direction from an outline and within a range, as the app works it out too (src/lib/mowStripes.ts)."""

import math
from typing import Optional, Sequence

from shapely.geometry import Polygon

from .config import XY
from .geometry import polygons


# recorded outlines have a point every few cm, for finding their longest edge they're simplified this much
ANGLE_SIMPLIFY_TOLERANCE = 0.10


def _largest(geom) -> Optional[Polygon]:
    parts = list(polygons(geom))
    return max(parts, key=lambda p: p.area) if parts else None


def _longest_edge_direction(coords: Sequence[XY]) -> float:
    best, angle = -1.0, 0.0
    for (x0, y0), (x1, y1) in zip(coords, coords[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        if length > best:
            best, angle = length, math.atan2(y1 - y0, x1 - x0)
    return angle


def longest_edge_angle(area) -> float:
    """Direction of the longest edge of the (largest part of the) outline. Lanes along it give long lanes
    and few turns for the usual garden shapes."""
    main = _largest(area)
    if main is None:
        return 0.0
    ring = main.exterior
    simple = ring.simplify(ANGLE_SIMPLIFY_TOLERANCE, preserve_topology=False)
    coords = list(simple.coords) if len(simple.coords) >= 4 else list(ring.coords)
    return _longest_edge_direction(coords)


def min_width_angle(area) -> float:
    """Direction of the long side of the smallest rectangle around the area: the fewest lanes for convex
    shapes."""
    rect = area.minimum_rotated_rectangle
    if rect.is_empty or not hasattr(rect, 'exterior'):
        return longest_edge_angle(area)
    return _longest_edge_direction(list(rect.exterior.coords))


def angle_in_range(angle: float, lo: Optional[float] = None, hi: Optional[float] = None) -> float:
    """Like MowBite's angleInRange (src/lib/mowStripes.ts): it's about the direction of the lanes, 0 and
    180 degrees give the same ones, so an angle whose lanes already lie in the range stays. Past an end it
    bounces back. hi < lo is a range across the ends (e.g. 170 to -170 degrees around 180)."""
    if lo is None or hi is None:
        return angle
    half = math.pi
    d = hi - lo
    # half a turn or more allows every direction
    if d >= half:
        return angle
    width = math.fmod(math.fmod(d, half) + half, half)
    if width == 0:
        return lo
    # the same lanes can be written +-180 degrees apart, use the one closest to the middle of the range
    mid = lo + width / 2
    r = math.fmod(angle - mid, half)
    angle = mid + (r - half if r > half / 2 else r + half if r < -half / 2 else r)
    t = math.fmod(angle - lo, 2 * width)
    if t < 0:
        t += 2 * width
    return lo + (t if t <= width else 2 * width - t)
