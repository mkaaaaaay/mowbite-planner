"""The lanes' angle, the lanes across an area and the cells they're mowed in (planner.py, steps 3 and 4)."""

import bisect
import math
from typing import Dict, List, Optional, Sequence, Tuple

from shapely.geometry import Polygon

from .config import MAX_ROWS, ON_EDGE, XY, Lane, PlannerConfig, PlanningError
from .geometry import polygons, rings
from .mathutil import angle_diff
from .transit import TransitPlanner


# recorded outlines have a point every few cm, for finding their longest edge they're simplified this much
ANGLE_SIMPLIFY_TOLERANCE = 0.10


# lane angle ------------------------------------------------------------------------------------------------


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


def _lane_cost(region, angle: float, spacing: float, min_length: float, target=None, room=None,
               reach: float = 0.0) -> float:
    # every lane costs a turn and every group of lanes a drive to it, a few turns' worth
    rows = scan_lanes(region, angle, spacing, min_length, target, room, reach)
    return sum(len(r) for r in rows) + 3.0 * len(split_cells(rows))


def optimal_angle(region, cfg: PlannerConfig, fallback: float, target=None, room=None) -> float:
    """The lane direction with the fewest lanes and groups of lanes, tried angle_step apart (within the
    angle range if there is one). Ties go to the one closest to the fallback (the longest edge). target and
    room as for scan_lanes, so the lanes counted are the ones the plan gets."""
    if region.is_empty:
        return fallback
    candidates = {round(math.fmod(fallback, math.pi) % math.pi, 9)}
    n = max(1, int(round(math.pi / cfg.angle_step)))
    candidates.update(round(i * math.pi / n, 9) for i in range(n))
    if cfg.angle_min is not None:
        candidates = {angle_in_range(a, cfg.angle_min, cfg.angle_max) for a in candidates}
    # a recorded outline has a point every few cm: its rings are read once, not for every angle. Simplified
    # they would count other lanes than the plan gets
    region, target, room = (None if g is None else _ring_list(g) for g in (region, target, room))
    best = None
    for a in sorted(candidates):
        cost = _lane_cost(region, a, cfg.lane_spacing, cfg.min_lane_length, target, room, 0.5 * cfg.mower_width)
        key = (cost, angle_diff(2 * a, 2 * fallback))
        if best is None or key < best[0]:
            best = (key, a)
    return best[1]


def lane_angle(area, free, cfg: PlannerConfig, lane_region=None, target=None, room=None) -> float:
    if cfg.angle is not None:
        a = cfg.angle
    elif cfg.angle_strategy == 'min_width':
        a = min_width_angle(free)
    elif cfg.angle_strategy == 'optimal' and lane_region is not None:
        a = optimal_angle(lane_region, cfg, longest_edge_angle(area), target, room)
    else:
        a = longest_edge_angle(area)
    a = angle_in_range(a + cfg.angle_offset + cfg.angle_steps * cfg.angle_increment, cfg.angle_min, cfg.angle_max)
    # lanes have no front, 30 and 210 degrees are the same lanes
    a = math.fmod(a, math.pi)
    return a + math.pi if a < 0 else a


# lanes and cells -------------------------------------------------------------------------------------------



def bend_lanes(rows: List[List[Lane]], angle: float, space: TransitPlanner, max_gap: float,
               max_offset: float) -> List[List[Lane]]:
    """Joins the pieces of a row with at most max_gap between them (where a small obstacle splits the row)
    into one lane that bends around it, on the shortest way through space, if that way never leaves the
    row's line by more than max_offset nor goes back. Fewer cells: the zigzag goes on past the obstacle."""
    c, s = math.cos(angle), math.sin(angle)
    out = []
    for row in rows:
        joined: List[Lane] = []
        for lane in row:
            prev = joined[-1] if joined else None
            if prev is not None and 0.0 < lane.x0 - prev.x1 <= max_gap:
                p = (prev.x1 * c - prev.y * s, prev.x1 * s + prev.y * c)
                q = (lane.x0 * c - lane.y * s, lane.x0 * s + lane.y * c)
                way = space.path(p, q)
                if way is not None:
                    local = [(x * c + y * s, -x * s + y * c) for x, y in way]
                    if all(abs(v - lane.y) <= max_offset for _, v in local) and \
                            all(u1 >= u0 - 1e-9 for (u0, _), (u1, _) in zip(local, local[1:])):
                        joined[-1] = Lane(prev.row, prev.x0, lane.x1, prev.y,
                                          prev.bends + ((prev.x1, lane.x0, tuple(way)),) + lane.bends)
                        continue
            joined.append(lane)
        out.append(joined)
    return out


def _ring_list(geom) -> List[List[XY]]:
    """The rings of a geometry as point lists (a list of them stays as it is)."""
    return geom if isinstance(geom, list) else [ring for _, ring in rings(geom)]


class _Scanner:
    """Where horizontal lines in the lane frame cross a polygon's rings (even-odd)."""

    def __init__(self, ring_list: List[List[XY]], c: float, s: float):
        # every edge that isn't horizontal: its lowest and highest y, its first point and the step to the next
        self.edges = []
        self.lo, self.hi = math.inf, -math.inf
        for ring in ring_list:
            local = [(x * c + y * s, -x * s + y * c) for x, y in ring]
            if not local:
                continue
            for (ax, ay), (bx, by) in zip(local, local[1:] + local[:1]):
                if ay < by:
                    self.edges.append((ay, by, ax, ay, bx - ax, by - ay))
                elif by < ay:
                    self.edges.append((by, ay, ax, ay, bx - ax, by - ay))
            ys = [y for _, y in local]
            self.lo, self.hi = min(self.lo, min(ys)), max(self.hi, max(ys))

    def intervals(self, y0: float, step: float, count: int) -> List[List[Tuple[float, float]]]:
        """Inside intervals on the lines y0 + j * step, j = 0..count-1. The crossings are found per edge for
        the lines it spans, not per line for every edge."""
        crossings: List[List[float]] = [[] for _ in range(count)]
        for lo, hi, ax, ay, dx, dy in self.edges:
            j = int((lo - y0) // step)
            if j < 0:
                j = 0
            y = y0 + j * step
            # half open, so a line through a vertex crosses exactly one of its two edges
            while y < hi and j < count:
                if y >= lo:
                    crossings[j].append(ax + (y - ay) * dx / dy)
                j += 1
                y = y0 + j * step
        return [_inside(xs) for xs in crossings]

    def at(self, ys: Sequence[float]) -> List[List[Tuple[float, float]]]:
        """Inside intervals on the lines y of ys (in order)."""
        crossings: List[List[float]] = [[] for _ in ys]
        for lo, hi, ax, ay, dx, dy in self.edges:
            for j in range(bisect.bisect_left(ys, lo), bisect.bisect_left(ys, hi)):
                crossings[j].append(ax + (ys[j] - ay) * dx / dy)
        return [_inside(xs) for xs in crossings]


def _inside(xs: List[float]) -> List[Tuple[float, float]]:
    """The inside intervals between the crossings of a line (even-odd)."""
    xs.sort()
    return [(xs[k], xs[k + 1]) for k in range(0, len(xs) - 1, 2) if xs[k + 1] > xs[k]]


Intervals = List[Tuple[float, float]]


def _intersect(a: Intervals, b: Intervals) -> Intervals:
    out, i, j = [], 0, 0
    while i < len(a) and j < len(b):
        lo, hi = max(a[i][0], b[j][0]), min(a[i][1], b[j][1])
        if hi > lo:
            out.append((lo, hi))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def _subtract(a: Intervals, b: Intervals) -> Intervals:
    out = []
    for lo, hi in a:
        cur = lo
        for blo, bhi in b:
            if bhi <= cur:
                continue
            if blo >= hi:
                break
            if blo > cur:
                out.append((cur, blo))
            cur = max(cur, bhi)
        if cur < hi:
            out.append((cur, hi))
    return out


def _union(a: Intervals, b: Intervals) -> Intervals:
    out: Intervals = []
    for lo, hi in sorted(a + b):
        if out and lo <= out[-1][1] + 1e-9:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    return out


def _widen(a: Intervals, d: float) -> Intervals:
    return _union([(lo - d, hi + d) for lo, hi in a], [])


# slivers of the area to mow narrower than this next to a lane's strip aren't worth stretching a lane for
COVER_TOLERANCE = 0.01


def scan_lanes(region, angle: float, spacing: float, min_length: float, target=None, room=None,
               reach: float = 0.0) -> List[List[Lane]]:
    """The pieces of parallel lines (at `angle`, at most `spacing` apart) inside the region, row by row.

    The first and the last line lie on the region's edge, the ones between are spread evenly, so the
    spacing is never more than asked for.

    With target (what the lanes have to mow), room (where the mower may drive) and reach (half the
    cutting width): where a part of the region ends between two rows, e.g. under an edge running along
    the lanes, the last row's strip can stop short of the perimeter loops' strip. Where target reaches past
    a lane's strip and the row on that side has no lane, that row's lane is carried on through the
    headland (as far as room allows), so nothing is left between the lanes and the loops.

    region, target and room are geometries or lists of their rings (point lists).
    """
    c, s = math.cos(angle), math.sin(angle)
    region, target, room = (None if g is None else _ring_list(g) for g in (region, target, room))
    lanes = _Scanner(region, c, s)
    if not lanes.edges:
        return []
    usable = lanes.hi - lanes.lo - 2.0 * ON_EDGE
    if usable <= 0.0:
        count, step, y0 = 1, spacing, 0.5 * (lanes.lo + lanes.hi)
    else:
        count = int(math.ceil(usable / spacing - 1e-9)) + 1
        if count > MAX_ROWS:
            raise PlanningError(f'{count} lanes, more than {MAX_ROWS}: the area is too big for this lane spacing')
        step = usable / (count - 1)
        y0 = lanes.lo + ON_EDGE
    # one more row on either side, which only gets lanes stretched into it
    n, base = count + 2, y0 - step
    # pieces too short to be lanes don't mow what's next to them either
    rows = [[]] + [[(lo, hi) for lo, hi in r if hi - lo >= min_length] for r in lanes.intervals(y0, step, count)] \
        + [[]]

    free = None
    if target and room is not None:
        tgt = _Scanner(target, c, s)
        beyond = reach + COVER_TOLERANCE
        on = tgt.intervals(base, step, n)  # to mow on row j's line
        above = tgt.intervals(base + beyond, step, n)  # to mow above row j's strip
        below = tgt.intervals(base - beyond, step, n)  # to mow below row j's strip
        free = _Scanner(room, c, s).intervals(base, step, n)
        for _ in range(3):
            grown = [list(r) for r in rows]
            more = 0.0
            for j in range(n - 1):
                # above row j where row j + 1 has no lane: row j + 1 goes on there, and the other way round
                for src, dst, todo in ((j, j + 1, above[j]), (j + 1, j, below[j + 1])):
                    extra = _intersect(_subtract(_intersect(todo, rows[src]), rows[dst]), free[dst])
                    if extra:
                        more += sum(hi - lo for lo, hi in extra)
                        grown[dst] = _union(grown[dst], extra)
            for j in range(n):
                # on the row's own line, out of reach of its lanes' ends (e.g. under an obstacle that starts
                # level with the row): a lane there, joined to the lanes next to it
                todo = _subtract(_intersect(on[j], free[j]), _widen(rows[j], reach))
                todo = [(lo, hi) for lo, hi in todo if hi - lo > COVER_TOLERANCE]
                if todo:
                    extra = _intersect(_widen(todo, reach), free[j])
                    more += sum(hi - lo for lo, hi in todo)
                    grown[j] = _union(grown[j], extra)
            rows = grown
            if more < 1e-6:
                break

    out = []
    for j, intervals in enumerate(rows):
        if free is not None and any(hi - lo < min_length for lo, hi in intervals):
            # a filled in piece too short for a lane is lengthened to one, as far as there's room
            longer = []
            for lo, hi in intervals:
                pad = 0.5 * max(0.0, min_length - (hi - lo))
                longer.extend(_intersect([(lo - pad, hi + pad)], free[j]) if pad > 0.0 else [(lo, hi)])
            intervals = _union(longer, [])
        y = base + j * step
        out.append([Lane(j, lo, hi, y) for lo, hi in intervals if hi - lo >= min_length - 1e-9])
    # the outer rows only if something was stretched into them
    if not out[0]:
        out = out[1:]
    if out and not out[-1]:
        out = out[:-1]
    return out


def split_cells(rows: List[List[Lane]]) -> List[List[Lane]]:
    """Groups the lanes into cells that can be mowed in one zigzag (boustrophedon decomposition).

    A lane continues the cell of the lane in the row before when they overlap and neither of them
    overlaps anything else in the other row. Where a row splits (an obstacle starts) or two pieces
    join (it ends), new cells begin.
    """
    cells: List[List[Lane]] = []
    open_cells: Dict[int, List[Lane]] = {}
    prev: List[Lane] = []
    for lanes in rows:
        succ: Dict[int, List[int]] = {i: [] for i in range(len(prev))}
        pred: Dict[int, List[int]] = {k: [] for k in range(len(lanes))}
        for i, p in enumerate(prev):
            for k, q in enumerate(lanes):
                if p.x0 < q.x1 and q.x0 < p.x1:
                    succ[i].append(k)
                    pred[k].append(i)
        now_open: Dict[int, List[Lane]] = {}
        for k, q in enumerate(lanes):
            ps = pred[k]
            if len(ps) == 1 and len(succ[ps[0]]) == 1 and ps[0] in open_cells:
                cell = open_cells[ps[0]]
                cell.append(q)
            else:
                cell = [q]
                cells.append(cell)
            now_open[k] = cell
        open_cells = now_open
        prev = lanes
    return cells
