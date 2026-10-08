"""Shortest drives through the free space, e.g. around an obstacle to the next lanes that aren't mowed yet.

The shortest way between two points in a polygon with holes is a polyline whose corners are reflex
vertices of the polygon: it runs straight where it can and bends around obstacle corners and the
inside corners of the outline. So the graph only needs those vertices as nodes (a visibility graph),
searched with A*. A way only ever bends around a corner, it doesn't head into it: an edge has to leave
both of its corners tangentially (both neighbours of the corner on the same side of it), which rules
out most edges before the expensive check whether they stay inside. Which nodes see each other is
worked out only when the search gets there and kept for the next search.
"""

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point
from shapely.geometry.polygon import orient
from shapely.ops import nearest_points
from shapely.prepared import prep

from .geometry import JOIN_MITRE, polygonal, polygons

try:  # shapely 2 makes a line of an array much quicker than of points with its class, and tests many at once
    import numpy
    from shapely import covers, intersects_xy, linestrings
except ImportError:  # shapely 1.7
    covers = None

XY = Tuple[float, float]
# corner, the corner before it, the corner after it on its ring
Node = Tuple[XY, XY, XY]

# in the graph that keeps its distance from the edge, corners turning less than this aren't nodes (a recorded
# outline has a dent every few cm, GPS noise; the distance leaves room to drive past them straight)
MIN_REFLEX = math.radians(2.0)
# m, path() keeps this far off the edge (at most the clearance), careful_path() first the clearance and this where
# that's a lot longer (or there's none, a narrow passage), each before a way along the edge
NARROW_CLEARANCE = 0.05
# lines this close to running along a corner's edge count as touching it (sine of the angle between them)
TANGENT_TOLERANCE = 1e-4


def _cross(o: XY, a: XY, b: XY) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def reflex_nodes(geom, min_turn: float = 0.0) -> List[Node]:
    """Vertices where the region's inside angle is over 180 degrees (outline notches, obstacle corners), with
    their neighbours. Corners turning by less than min_turn are left out."""
    out: List[Node] = []
    for poly in polygons(geom):
        # outline ccw, holes cw: walking along either, the inside of the region is on the left and a right
        # turn bends around something
        poly = orient(poly, 1.0)
        for ring in [poly.exterior, *poly.interiors]:
            c = list(ring.coords)[:-1]
            n = len(c)
            for i in range(n):
                p, v, q = c[i - 1], c[i], c[(i + 1) % n]
                cross = _cross(p, v, q)
                if cross < 0 and -math.atan2(cross, (v[0] - p[0]) * (q[0] - v[0]) + (v[1] - p[1]) * (q[1] - v[1])) \
                        >= min_turn:
                    out.append((v, p, q))
    return out


def reflex_vertices(geom) -> List[XY]:
    return [v for v, _, _ in reflex_nodes(geom)]


def _tangent(node: Node, other: XY) -> bool:
    """Does the line from the corner to `other` only touch the region's boundary there (both neighbours of
    the corner on one side of it)?"""
    v, p, q = node
    a, b = _cross(v, other, p), _cross(v, other, q)
    d = math.hypot(other[0] - v[0], other[1] - v[1])
    ta = TANGENT_TOLERANCE * d * math.hypot(p[0] - v[0], p[1] - v[1])
    tb = TANGENT_TOLERANCE * d * math.hypot(q[0] - v[0], q[1] - v[1])
    return not ((a > ta and b < -tb) or (a < -ta and b > tb))


def _tangents(v, p, q, other):
    """_tangent of arrays (numpy, x and y last): of corners v (p, q their neighbours) and others."""
    a = (other[..., 0] - v[..., 0]) * (p[..., 1] - v[..., 1]) - (other[..., 1] - v[..., 1]) * (p[..., 0] - v[..., 0])
    b = (other[..., 0] - v[..., 0]) * (q[..., 1] - v[..., 1]) - (other[..., 1] - v[..., 1]) * (q[..., 0] - v[..., 0])
    d = numpy.hypot(other[..., 0] - v[..., 0], other[..., 1] - v[..., 1])
    ta = TANGENT_TOLERANCE * d * numpy.hypot(p[..., 0] - v[..., 0], p[..., 1] - v[..., 1])
    tb = TANGENT_TOLERANCE * d * numpy.hypot(q[..., 0] - v[..., 0], q[..., 1] - v[..., 1])
    return ~(((a > ta) & (b < -tb)) | ((a < -ta) & (b > tb)))


class _Graph:
    def __init__(self, nodes: List[Node], inside=None, clearance: float = 0.0):
        self.nodes = nodes
        self.clearance = clearance  # (how far the corners are off the edge)
        # points in here only head for a corner tangentially. One outside (closer to the edge than the shrunk
        # space) may have to get to the nearest corner across its wedge first
        self.inside = inside
        self.neighbours: Dict[int, List[Tuple[int, float]]] = {}
        self.corners = numpy.array(nodes, dtype=float).reshape(-1, 3, 2) if covers is not None else None

    def tangent_from(self, p: XY) -> bool:
        return self.inside is None or self.inside.covers(Point(p))


class TransitPlanner:
    """Shortest paths for the mower's centre inside `space` (a Polygon or MultiPolygon).

    Graph nodes come from the space shrunk by NARROW_CLEARANCE (careful_path(): by `clearance` first), so drives
    don't scrape along the very edge. Where a way only exists closer to the edge (a narrow passage), or is a lot
    shorter there, the unshrunk corners are used. Points in different parts of the space have no way between them, that's known at once.

    careful_path() also weighs how much of a way runs along the edge: every metre closer than edge_distance
    counts 1 + edge_cost times, a way through the space shrunk by edge_distance wins where it's cheaper so.
    """

    def __init__(self, space, clearance: float = 0.05, tolerance: float = 1e-3, edge_distance: float = 0.0,
                 edge_cost: float = 0.0):
        self._space = space
        self._tolerance = tolerance
        self._edge_distance, self._edge_cost = edge_distance, edge_cost
        self._inner: Optional['TransitPlanner'] = None  # the space shrunk by edge_distance, made when needed
        self._band = None  # what's closer to the edge than that
        # a hair wider, so a drive exactly along the edge (the outer perimeter loop) still counts as inside
        wider = space.buffer(tolerance, 4)
        self._check = prep(wider)
        self._wider = wider if covers is not None else None  # (prep() above prepared it)
        self._polys = list(polygons(wider))
        self._parts = [prep(p) for p in self._polys]
        self._clearance = clearance
        self._graphs: Optional[List[_Graph]] = None
        self._visible: Dict[Tuple[int, int, int], bool] = {}
        # a way along strips (see without()) is a lot longer than straight anyway: the edge hugged only where there's
        # no other
        self._strips = False

    @property
    def space(self):
        """The space the ways stay in."""
        return self._space

    def is_free(self, points: Sequence[XY]) -> bool:
        pts = list(points)
        if not pts:
            return True
        if len(pts) == 1 or all(abs(p[0] - pts[0][0]) < 1e-9 and abs(p[1] - pts[0][1]) < 1e-9 for p in pts):
            if self._wider is not None:
                return bool(intersects_xy(self._wider, pts[0][0], pts[0][1]))
            return self._check.covers(Point(pts[0]))
        if self._wider is not None:
            return bool(covers(self._wider, linestrings(numpy.asarray(pts, dtype=float))))
        return self._check.covers(LineString(pts))

    def free_from(self, p: XY, others: Sequence[XY]) -> List[bool]:
        """is_free([p, q]) for each q of others, with shapely 2 all at once."""
        if self._wider is None or len(others) < 2:
            return [self.is_free([p, q]) for q in others]
        lines = numpy.empty((len(others), 2, 2))
        lines[:, 0] = p
        lines[:, 1] = others
        free = covers(self._wider, linestrings(lines))
        same = (abs(lines[:, 1, 0] - p[0]) < 1e-9) & (abs(lines[:, 1, 1] - p[1]) < 1e-9)
        if same.any():  # (a point: is_free tests it)
            free[same] = bool(intersects_xy(self._wider, p[0], p[1]))
        return free.tolist()

    def clip(self, a: XY, b: XY, min_length: float = 0.0) -> Optional[Tuple[XY, XY]]:
        """The longest piece of the line from a to b inside the space, from a's side to b's, None if there's
        none at least min_length long."""
        cut = self._space.intersection(LineString([a, b]))
        pieces = [g for g in getattr(cut, 'geoms', [cut]) if g.geom_type == 'LineString' and not g.is_empty]
        if not pieces:
            return None
        best = max(pieces, key=lambda g: g.length)
        if best.length < max(min_length, 1e-6):
            return None
        p, q = best.coords[0], best.coords[-1]
        if math.hypot(p[0] - a[0], p[1] - a[1]) > math.hypot(q[0] - a[0], q[1] - a[1]):
            p, q = q, p
        return (p[0], p[1]), (q[0], q[1])

    def without(self, geom) -> 'TransitPlanner':
        """A planner like this one for its space but geom (the strips round it, along which the way that keeps its
        distance from the edge is taken where there is one)."""
        out = TransitPlanner(polygonal(self._space.difference(geom)), self._clearance, self._tolerance,
                             self._edge_distance, self._edge_cost)
        out._strips = True
        return out

    def parts(self) -> list:
        """The connected parts of the space (a hair wider), part() gives indices into this."""
        return list(self._polys)

    def part(self, p: Optional[XY]) -> Optional[int]:
        """Which connected part of the space p lies in, None if outside."""
        if p is None:
            return None
        pt = Point(p)
        return next((i for i, part in enumerate(self._parts) if part.covers(pt)), None)

    def _build_graphs(self) -> List[_Graph]:
        graphs = []
        for clearance in sorted({self._clearance, min(self._clearance, NARROW_CLEARANCE)}, reverse=True):
            if clearance <= 0:
                continue
            shrunk = self._space.buffer(-clearance, 4, join_style=JOIN_MITRE, mitre_limit=3.0)
            # simplifying keeps the real corners (as vertices of the shrunk outline, so still inside) and drops
            # the dents, which would be hundreds of useless nodes
            shrunk = shrunk.simplify(0.5 * clearance, preserve_topology=True)
            nodes = [n for n in reflex_nodes(shrunk, MIN_REFLEX) if self.is_free([n[0]])]
            if nodes:
                graphs.append(_Graph(nodes, prep(shrunk), clearance))
        # every corner of the space itself: exact, a way along the edge bends at each of them
        graphs.append(_Graph([n for n in reflex_nodes(self._space) if self.is_free([n[0]])]))
        return graphs

    def path(self, a: XY, b: XY, wide: bool = False) -> Optional[List[XY]]:
        """Shortest polyline from a to b inside the space, None if there's no way (or a or b lie outside). wide: one
        keeping the clearance off the edge first."""
        a = (float(a[0]), float(a[1]))
        b = (float(b[0]), float(b[1]))
        if self.is_free([a, b]):
            return [a, b]
        pa = self.part(a)
        if pa is None or pa != self.part(b):
            return None
        if self._graphs is None:
            self._graphs = self._build_graphs()
        best = None
        straight = math.hypot(b[0] - a[0], b[1] - a[1])
        for gi, graph in enumerate(self._graphs):
            if graph.clearance > NARROW_CLEARANCE and not wide:
                continue
            found = self._astar(gi, graph, a, b)
            if found is not None and (best is None or _length(found) < _length(best)):
                best = found
            # keep the way that keeps its distance from the edge unless hugging it is a lot shorter
            if best is not None and (self._strips or _length(best) <= 1.2 * straight + 0.5):
                break
        return best

    def edge_length(self, pts: Sequence[XY]) -> float:
        """How much of the way runs closer to the space's edge than edge_distance."""
        if self._band is None or len(pts) < 2:
            return 0.0
        return LineString(pts).intersection(self._band).length

    def cost(self, pts: Sequence[XY]) -> float:
        return _length(pts) + self._edge_cost * self.edge_length(pts)

    def careful_path(self, a: XY, b: XY) -> Optional[List[XY]]:
        """Like path(), but keeping the clearance off the edge where that's not a lot longer, and a way that keeps
        edge_distance off the edge where that's cheaper (see cost()): straight from a into the shrunk space, through
        it, straight out to b."""
        best = self.path(a, b, True)
        if best is None or self._edge_distance <= 0.0 or self._edge_cost <= 0.0:
            return best
        if self._inner is None:
            inner = self._space.buffer(-self._edge_distance, 4)
            if inner.is_empty:
                return best
            self._inner = TransitPlanner(inner, self._clearance, self._tolerance)
            # (not its edge, ways through the shrunk space may run along that)
            self._band = self._space.difference(inner.buffer(self._tolerance, 4))
        inner = self._inner
        ends = []
        for p in (a, b):
            q = p if inner.is_free([p]) else nearest_points(inner._space, Point(p))[0].coords[0]
            ends.append((float(q[0]), float(q[1])))
        (a2, b2) = ends
        middle = inner.path(a2, b2, True) if inner.part(a2) is not None and inner.part(a2) == inner.part(b2) else None
        if middle is None or not self.is_free([a, a2]) or not self.is_free([b2, b]):
            return best
        way = [a] + [p for p in middle if p != a and p != b] + [b]
        way = [p for i, p in enumerate(way)
               if i == 0 or math.hypot(p[0] - way[i - 1][0], p[1] - way[i - 1][1]) > 1e-9]
        return way if self.cost(way) < self.cost(best) - 1e-9 else best

    def _sees(self, gi: int, i: int, j: int) -> bool:
        key = (gi, i, j) if i < j else (gi, j, i)
        seen = self._visible.get(key)
        if seen is None:
            nodes = self._graphs[gi].nodes
            seen = self.is_free([nodes[i][0], nodes[j][0]])
            self._visible[key] = seen
        return seen

    def _neighbours(self, gi: int, graph: _Graph, i: int) -> List[Tuple[int, float]]:
        out = graph.neighbours.get(i)
        if out is None:
            node = graph.nodes[i]
            xi, yi = node[0]
            if graph.corners is not None:
                # (shapely 2: at once the corners whose line to this one leaves both tangentially, then which it sees)
                c = graph.corners
                near = _tangents(c[i, 0], c[i, 1], c[i, 2], c[:, 0]) & _tangents(c[:, 0], c[:, 1], c[:, 2], c[i, 0])
                near[i] = False
                near = numpy.nonzero(near)[0].tolist()
                todo = [j for j in near if (gi, min(i, j), max(i, j)) not in self._visible]
                for j, seen in zip(todo, self.free_from(node[0], [graph.nodes[j][0] for j in todo])):
                    self._visible[(gi, min(i, j), max(i, j))] = seen
                out = [(j, math.hypot(graph.nodes[j][0][0] - xi, graph.nodes[j][0][1] - yi)) for j in near
                       if self._visible[(gi, min(i, j), max(i, j))]]
            else:
                out = [(j, math.hypot(other[0][0] - xi, other[0][1] - yi)) for j, other in enumerate(graph.nodes)
                       if j != i and _tangent(node, other[0]) and _tangent(other, node[0]) and self._sees(gi, i, j)]
            graph.neighbours[i] = out
        return out

    def _astar(self, gi: int, graph: _Graph, a: XY, b: XY) -> Optional[List[XY]]:
        nodes = graph.nodes
        if not nodes:
            return None
        tb, ta = graph.tangent_from(b), graph.tangent_from(a)

        def h(j: int) -> float:
            return math.hypot(b[0] - nodes[j][0][0], b[1] - nodes[j][0][1])

        to_goal: Dict[int, Optional[float]] = {}  # corner -> how far b is straight on, None where it doesn't see b

        def goal(j: int) -> Optional[float]:
            # (worked out where the search gets to, and from the closest corner on till one sees b)
            if j not in to_goal:
                to_goal[j] = h(j) if (not tb or _tangent(nodes[j], b)) and self.is_free([nodes[j][0], b]) else None
            return to_goal[j]

        if all(goal(j) is None for j in sorted(range(len(nodes)), key=h)):
            return None

        g: Dict[int, float] = {}
        came: Dict[int, int] = {}
        heap: List[Tuple[float, float, int]] = []
        near = [j for j, n in enumerate(nodes) if not ta or _tangent(n, a)]
        for j, free in zip(near, self.free_from(a, [nodes[j][0] for j in near])):
            if free:
                n = nodes[j]
                d = math.hypot(n[0][0] - a[0], n[0][1] - a[1])
                g[j] = d
                came[j] = -1
                heapq.heappush(heap, (d + h(j), d, j))
        best_len, best_last = math.inf, None
        closed = set()
        while heap:
            f, gj, j = heapq.heappop(heap)
            if f >= best_len:
                break
            if j in closed or gj > g.get(j, math.inf):
                continue
            closed.add(j)
            straight = goal(j)
            if straight is not None and gj + straight < best_len:
                best_len, best_last = gj + straight, j
            for k, d in self._neighbours(gi, graph, j):
                ng = gj + d
                if ng < g.get(k, math.inf) - 1e-12:
                    g[k] = ng
                    came[k] = j
                    heapq.heappush(heap, (ng + h(k), ng, k))
        if best_last is None:
            return None
        chain = [b]
        j = best_last
        while j != -1:
            chain.append(nodes[j][0])
            j = came[j]
        chain.append(a)
        return chain[::-1]


def _length(pts: Sequence[XY]) -> float:
    return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
