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
from shapely.prepared import prep

from .geometry import JOIN_MITRE, polygons

XY = Tuple[float, float]
# corner, the corner before it, the corner after it on its ring
Node = Tuple[XY, XY, XY]

# in the graph that keeps its distance from the edge, corners turning less than this aren't nodes (a recorded
# outline has a dent every few cm, GPS noise; the distance leaves room to drive past them straight)
MIN_REFLEX = math.radians(2.0)
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


class _Graph:
    def __init__(self, nodes: List[Node], inside=None):
        self.nodes = nodes
        # points in here only head for a corner tangentially. One outside (closer to the edge than the shrunk
        # space) may have to get to the nearest corner across its wedge first
        self.inside = inside
        self.neighbours: Dict[int, List[Tuple[int, float]]] = {}

    def tangent_from(self, p: XY) -> bool:
        return self.inside is None or self.inside.covers(Point(p))


class TransitPlanner:
    """Shortest paths for the mower's centre inside `space` (a Polygon or MultiPolygon).

    Graph nodes come from the space shrunk by `clearance`, so drives don't scrape along the very edge.
    Where a way only exists closer to the edge (a narrow passage), or is a lot shorter there, the unshrunk
    corners are used. Points in different parts of the space have no way between them, that's known at once.
    """

    def __init__(self, space, clearance: float = 0.05, tolerance: float = 1e-3):
        self._space = space
        # a hair wider, so a drive exactly along the edge (the outer perimeter loop) still counts as inside
        wider = space.buffer(tolerance, 4)
        self._check = prep(wider)
        self._polys = list(polygons(wider))
        self._parts = [prep(p) for p in self._polys]
        self._clearance = clearance
        self._graphs: Optional[List[_Graph]] = None
        self._visible: Dict[Tuple[int, int, int], bool] = {}

    def is_free(self, points: Sequence[XY]) -> bool:
        pts = list(points)
        if not pts:
            return True
        if len(pts) == 1 or all(abs(p[0] - pts[0][0]) < 1e-9 and abs(p[1] - pts[0][1]) < 1e-9 for p in pts):
            return self._check.covers(Point(pts[0]))
        return self._check.covers(LineString(pts))

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
        if self._clearance > 0:
            shrunk = self._space.buffer(-self._clearance, 4, join_style=JOIN_MITRE, mitre_limit=3.0)
            # simplifying keeps the real corners (as vertices of the shrunk outline, so still inside) and drops
            # the dents, which would be hundreds of useless nodes
            shrunk = shrunk.simplify(0.5 * self._clearance, preserve_topology=True)
            nodes = [n for n in reflex_nodes(shrunk, MIN_REFLEX) if self.is_free([n[0]])]
            if nodes:
                graphs.append(_Graph(nodes, prep(shrunk)))
        # every corner of the space itself: exact, a way along the edge bends at each of them
        graphs.append(_Graph([n for n in reflex_nodes(self._space) if self.is_free([n[0]])]))
        return graphs

    def path(self, a: XY, b: XY) -> Optional[List[XY]]:
        """Shortest polyline from a to b inside the space, None if there's no way (or a or b lie outside)."""
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
            found = self._astar(gi, graph, a, b)
            if found is not None and (best is None or _length(found) < _length(best)):
                best = found
            # keep the way that keeps its distance from the edge unless hugging it is a lot shorter
            if best is not None and _length(best) <= 1.2 * straight + 0.5:
                break
        return best

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
            out = [(j, math.hypot(other[0][0] - xi, other[0][1] - yi)) for j, other in enumerate(graph.nodes)
                   if j != i and _tangent(node, other[0]) and _tangent(other, node[0]) and self._sees(gi, i, j)]
            graph.neighbours[i] = out
        return out

    def _astar(self, gi: int, graph: _Graph, a: XY, b: XY) -> Optional[List[XY]]:
        nodes = graph.nodes
        if not nodes:
            return None
        tb, ta = graph.tangent_from(b), graph.tangent_from(a)
        to_goal = {j: math.hypot(b[0] - n[0][0], b[1] - n[0][1]) for j, n in enumerate(nodes)
                   if (not tb or _tangent(n, b)) and self.is_free([n[0], b])}
        if not to_goal:
            return None

        def h(j: int) -> float:
            return math.hypot(b[0] - nodes[j][0][0], b[1] - nodes[j][0][1])

        g: Dict[int, float] = {}
        came: Dict[int, int] = {}
        heap: List[Tuple[float, float, int]] = []
        for j, n in enumerate(nodes):
            if (not ta or _tangent(n, a)) and self.is_free([a, n[0]]):
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
            if j in to_goal and gj + to_goal[j] < best_len:
                best_len, best_last = gj + to_goal[j], j
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
