"""What a plan is measured against, as the planner of d4c74c0 worked it out: the lawn as drawn and the space the body
may be in. Kept as it is, so a planner never sets its own yardstick."""

import math
from typing import List, Sequence, Tuple

from shapely import wkb
from shapely.geometry import Polygon
from shapely.ops import clip_by_rect, unary_union

from mowbite_planner.geometry import clean_polygon, inset, rings, subtract, union
from mowbite_planner.mapfile import area_job
from mowbite_planner.mathutil import normalize_angle

XY = Tuple[float, float]
DRAWN = 1e-4  # m2 (planner.DRAWN)
TURN_STEP = math.radians(10.0)  # (footprint.TURN_STEP)


def lawn_of(garden, area_id: str):
    """The lawn as drawn: the area but its obstacles and the mowing areas lying in it (see lawn_left_m2)."""
    job = area_job(garden.map, area_id)
    lawn = Polygon(job.boundary).buffer(0)
    holes = [Polygon(r).buffer(0) for r in list(job.obstacles) + list(job.nested) if len(r) >= 3]
    return lawn.difference(unary_union(holes)) if holes else lawn


def body_space(outlines: List[Sequence[Sequence[float]]], away_from: List[Sequence[Sequence[float]]],
               margin_of: List[float], cfg):
    """Where the body may be (planner._body_space without its caches and Footprint)."""
    outlines, away_from = ([[(float(p[0]), float(p[1])) for p in ring] for ring in rs]
                           for rs in (outlines, away_from))
    hard = cfg.edges == 'hard'
    outer = union([clean_polygon(o) for o in outlines] + ([] if hard else [_driven(outlines, cfg.body_outline)]))
    band = None if hard else _driven(away_from, cfg.body_outline)
    inner = []
    for w in away_from:
        w = clean_polygon(w)
        rest = w if band is None else w.difference(band)
        inner.append(rest if rest.area >= DRAWN else w)
    # (through WKB like the planner's, which keeps it so)
    return wkb.loads(subtract(inset(outer, cfg.edge_margin), [inset(w, -m) for w, m in zip(inner, margin_of)]).wkb)


def _driven(rings: Sequence[Sequence[Sequence[float]]], corners: Sequence[XY]):
    """Where a body of these corners (counter-clockwise round its centre, see PlannerConfig.body_outline) was while the
    centre drove along each of the rings the way its points go: along every edge from where it was at the edge's start
    to where it was at its end (_rows), so past a corner and on the outside of a bend the front reaches out, and at
    every point turning on the spot from the heading of the edge before to the next one's (TURN_STEP apart)."""
    parts, rows = [], _rows(corners)

    def body(x: float, y: float, h: float, points: Sequence[XY]) -> Polygon:
        c, s = math.cos(h), math.sin(h)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in points])

    for ring in rings:
        heads = []  # every edge's heading and end
        for (ax, ay), (bx, by) in zip(ring, list(ring[1:]) + list(ring[:1])):
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-9:
                heads.append((math.atan2(by - ay, bx - ax), bx, by))
                parts += [body(bx, by, heads[-1][0], [(dx - length * back, dy) for dx, dy, back in row])
                          for row in rows]
        for (h, x, y), (h1, _, _) in zip(heads, heads[1:] + heads[:1]):
            turn = normalize_angle(h1 - h)
            n = int(math.ceil(abs(turn) / TURN_STEP))
            parts += [body(x, y, h + turn * k / n, corners) for k in range(1, n)]
    return union(parts)


def _rows(corners: Sequence[XY]) -> List[List[Tuple[float, float, int]]]:
    """The body of these corners (counter-clockwise) in pieces every line along its heading crosses once (itself where
    it is like that, else cut along the heading where its edges turn from going left to going right or back), each as
    its corners (x, y, 1 on its rear side; the leftmost and rightmost on both sides): driving straight ahead it swept
    the polygon of a piece's corners where it got to, those on the rear side as far back as it drove."""
    ys = [y for _, y in corners]
    goes = [(b > a, b) for a, b in zip(ys, ys[1:] + ys[:1]) if b != a]  # (to the left, to where)
    turns = sorted({y for (left, y), (on, _) in zip(goes, goes[1:] + goes[:1]) if left != on})
    pieces = [list(corners)]
    if len(turns) > 2:
        x0, _, x1, _ = Polygon(corners).bounds
        pieces = [ring for a, b in zip(turns, turns[1:])
                  for _, ring in rings(clip_by_rect(Polygon(corners), x0 - 1.0, a, x1 + 1.0, b))]
    rows = []
    for piece in pieces:
        row: List[Tuple[float, float, int]] = []
        for i, (x, y) in enumerate(piece):
            into, out = y - piece[i - 1][1], piece[(i + 1) % len(piece)][1] - y
            if into == out == 0:  # (in the middle of an edge along it)
                continue
            if into >= 0 and out >= 0 or into <= 0 and out <= 0:
                row.append((x, y, int(into <= 0 and out <= 0)))
            else:
                row += [(x, y, 0), (x, y, 1)] if into > 0 else [(x, y, 1), (x, y, 0)]
        rows.append(row)
    return rows
