"""shapely helpers that work the same on shapely 1.8 (Ubuntu 22.04 / Humble) and 2.x (Ubuntu 24.04 / Jazzy).

Both versions take the buffer resolution as the second positional argument (1.8 calls it `resolution`,
2.x `quad_segs`) and the join style as a plain number, so that's how buffer() is called everywhere.
"""

import math
from typing import Iterable, Iterator, List, Sequence, Tuple

from shapely.errors import ShapelyError
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import polygonize, unary_union

try:
    from shapely.validation import make_valid
except ImportError:  # shapely before 1.8 (Ubuntu 20.04, ROS Noetic)
    make_valid = None

XY = Tuple[float, float]

JOIN_ROUND = 1
JOIN_MITRE = 2
# segments per quarter circle of a round buffer: 10 degrees per segment, like the arcs of the turns
QUAD_SEGS = 9


def polygons(geom) -> Iterator[Polygon]:
    """The polygons in any geometry (Polygon, MultiPolygon, GeometryCollection), empty ones skipped."""
    if geom is None or geom.is_empty:
        return
    if isinstance(geom, Polygon):
        yield geom
    elif hasattr(geom, 'geoms'):
        for g in geom.geoms:
            yield from polygons(g)


def polygonal(geom):
    """Only the area of a geometry, as one oriented Polygon or MultiPolygon (outlines ccw, holes cw)."""
    parts = [orient(p, 1.0) for p in polygons(geom) if p.area > 1e-9]
    if not parts:
        return Polygon()
    return parts[0] if len(parts) == 1 else MultiPolygon(parts)


def clean_polygon(points: Sequence[Sequence[float]]):
    """A valid area from a list of (x, y): closing point optional, any orientation, self intersections
    fixed. Empty if there's no area."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3 or not all(math.isfinite(c) for p in pts for c in p):
        return Polygon()
    poly = Polygon(pts)
    if not poly.is_valid:
        # a figure eight becomes its two loops (buffer(0) would drop one of them). Without make_valid the
        # faces the outline encloses are the same
        if make_valid is not None:
            poly = make_valid(poly)
        else:
            poly = unary_union(list(polygonize(unary_union(LineString(pts + [pts[0]])))))
    return polygonal(poly)


def union(geoms: Iterable) -> object:
    geoms = [g for g in geoms if g is not None and not g.is_empty]
    return unary_union(geoms) if geoms else Polygon()


def subtract(geom, parts: Sequence) -> object:
    """geom minus all the parts. Where GEOS can't take their union away (a TopologyException, seen with
    shapely 1.7 on a few dozen loop strips), one part after the other."""
    whole = union(parts)
    if whole.is_empty:
        return geom
    try:
        return geom.difference(whole)
    except ShapelyError:
        for part in parts:
            geom = geom.difference(part)
        return geom


def inset(geom, distance: float):
    """The area shrunk by `distance` (grown if negative), round where it bends around inside corners."""
    if distance == 0:
        return polygonal(geom)
    return polygonal(geom.buffer(-distance, QUAD_SEGS, join_style=JOIN_ROUND))


def rings(geom) -> List[Tuple[str, List[XY]]]:
    """('outline' | 'hole', closed point list without the repeated end) of every ring, outlines ccw, holes cw."""
    out = []
    for poly in polygons(geom):
        poly = orient(poly, 1.0)
        out.append(('outline', list(poly.exterior.coords)[:-1]))
        for hole in poly.interiors:
            out.append(('hole', list(hole.coords)[:-1]))
    return out
