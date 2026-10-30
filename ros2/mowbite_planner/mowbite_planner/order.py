"""The order things are mowed in: the cells of a set of lanes, the lanes of a cell."""

import math
from typing import Iterator, List, Optional, Tuple

from .config import ORDER_BUDGET, XY, Lane, PlannerConfig
from .mathutil import angle_diff

# a way a cell is mowed: its lanes in order, the first one's direction (1: +x), where it starts and ends
Option = Tuple[List[Lane], int, XY, XY]


def best_order(start: List[List[float]], link, end: List[List[float]],
               first: Optional[List[int]] = None, budget: int = ORDER_BUDGET) -> List[Tuple[int, int]]:
    """The order of n items (cells), each done one of several ways (options), with the lowest total cost:
    start[i][a] to begin with item i done way a, link(i, a, j, b) from i done way a to j done way b, end[i][a]
    to finish with it. Starts from `first` (default: the cheapest next one each time), then moves runs of up to
    three items elsewhere and reverses runs while that helps, at most `budget` orders tried. For every order
    the best ways follow exactly (dynamic programming). Returns [(item, way)] in order."""
    n = len(start)
    if n == 0:
        return []
    k = [len(o) for o in start]
    links = {(i, j): [[link(i, a, j, b) for b in range(k[j])] for a in range(k[i])]
             for i in range(n) for j in range(n) if i != j}

    def solve(order: List[int]) -> Tuple[float, List[int]]:
        cost = list(start[order[0]])
        back = []
        for i, j in zip(order, order[1:]):
            m = links[i, j]
            nxt, came = [], []
            for b in range(k[j]):
                c, a = min((cost[a] + m[a][b], a) for a in range(k[i]))
                nxt.append(c)
                came.append(a)
            cost = nxt
            back.append(came)
        last = order[-1]
        total, way = min((cost[b] + end[last][b], b) for b in range(k[last]))
        ways = [way]
        for came in reversed(back):
            ways.append(came[ways[-1]])
        return total, ways[::-1]

    order = list(first) if first is not None else _greedy(start, links, k)
    best, ways = solve(order)
    tried = 1
    improved = True
    while improved and tried < budget:
        improved = False
        moves = []
        for size in (1, 2, 3):
            for i in range(n - size + 1):
                for j in range(n - size + 1):
                    if j != i:
                        moves.append(('move', size, i, j))
        for i in range(n - 1):
            for j in range(i + 2, n + 1):
                moves.append(('reverse', 0, i, j))
        for kind, size, i, j in moves:
            if tried >= budget:
                break
            if kind == 'move':
                run, rest = order[i:i + size], order[:i] + order[i + size:]
                candidate = rest[:j] + run + rest[j:]
            else:
                candidate = order[:i] + order[i:j][::-1] + order[j:]
            if candidate == order:
                continue
            tried += 1
            cost, cw = solve(candidate)
            if cost < best - 1e-9:
                order, best, ways = candidate, cost, cw
                improved = True
                break
    return list(zip(order, ways))


def _greedy(start: List[List[float]], links, k: List[int]) -> List[int]:
    """best_order's first order: the cheapest next one each time."""
    left, first, way = set(range(len(start))), [], None
    while left:
        if not first:
            i = min(left, key=lambda i: (min(start[i]), i))
            way = min(range(k[i]), key=lambda a: (start[i][a], a))
        else:
            p = first[-1]
            i, way = min(((j, b) for j in left for b in range(k[j])), key=lambda jb: (links[p, jb[0]][way][jb[1]], jb))
        first.append(i)
        left.remove(i)
    return first


def visit_order(lanes: List[Lane], cfg: PlannerConfig) -> List[Lane]:
    """The cell's lanes (one per row, in row order) in the order they're mowed."""
    if cfg.lane_order != 'skip' or len(lanes) < 3:
        return lanes
    spacing = abs(lanes[1].y - lanes[0].y)
    if spacing <= 0.0:
        return lanes
    # lanes k apart leave room for a U-turn of turn_radius. Every pass goes the same way, the turn from the end of one
    # pass to the start of the next is a long U-turn along the headland (blade_offset: the centre lines are twice the
    # offset closer every other turn)
    k = max(1, int(math.ceil((2.0 * cfg.turn_radius + 2.0 * abs(cfg.blade_offset)) / spacing - 1e-9)))
    n = len(lanes)
    if n < 2 * k:
        # too few lanes for that: from the middle, every step about half the cell wide (the smallest step as big as it
        # gets: n // 2)
        m = n // 2
        order = [i for pair in zip(range(m, n), range(m)) for i in pair] + ([n - 1] if n % 2 else [])
        return [lanes[i] for i in order]
    return [lanes[i] for p in range(k) for i in range(p, n, k)]


def options(cell: List[Lane], angle: float, cfg: PlannerConfig) -> Iterator[Option]:
    """The ways to mow a cell: from its first or last row, starting from either end."""
    c, s = math.cos(angle), math.sin(angle)
    for from_last in (False, True):
        lanes = visit_order(cell[::-1] if from_last else cell, cfg)
        for direction in (1, -1):
            first, last = lanes[0], lanes[-1]
            out = direction if len(lanes) % 2 else -direction
            x0 = first.x0 if direction > 0 else first.x1
            x1 = last.x1 if out > 0 else last.x0
            yield lanes, direction, (x0 * c - first.y * s, x0 * s + first.y * c), \
                (x1 * c - last.y * s, x1 * s + last.y * c)


def distance(a: Optional[XY], b: XY) -> float:
    return 0.0 if a is None else math.hypot(b[0] - a[0], b[1] - a[1])


def turning(heading: Optional[float], angle: float, direction: int, r: float) -> float:
    """About what turning round to a lane going direction costs: driving the turn's arc."""
    return 0.0 if heading is None else r * angle_diff(angle if direction > 0 else angle + math.pi, heading)


def final_cell(cells: List[List[Lane]], end: XY, angle: float, cfg: PlannerConfig) -> int:
    """The cell that can end closest to end."""
    return min(range(len(cells)), key=lambda i: min(distance(end, o[3]) for o in options(cells[i], angle, cfg)))


def nearest_cell(cells: List[List[Lane]], pos: Optional[XY], heading: Optional[float], angle: float,
                 cfg: PlannerConfig, end: Optional[XY] = None) -> Tuple[int, List[Lane], int]:
    """The cell to mow next from pos (facing heading) and how: (index, lanes in order, first direction). end: the last
    cell, ending as close to it as it can, getting to it second."""
    best = None
    for ci, cell in enumerate(cells):
        for lanes, direction, entry, exit_ in options(cell, angle, cfg):
            cost = distance(pos, entry) + turning(heading, angle, direction, cfg.turn_radius)
            if end is not None:
                cost += 1000.0 * distance(end, exit_)
            if best is None or cost < best[0] - 1e-9:
                best = (cost, ci, lanes, direction)
    return best[1], best[2], best[3]


def planned_cells(cells: List[List[Lane]], pos: Optional[XY], heading: Optional[float], end: Optional[XY],
                  angle: float, cfg: PlannerConfig) -> List[Tuple[List[Lane], int]]:
    """route_order 'optimized': the cells in the order best_order() finds, drives estimated as the straight distance
    plus turning, with end the last one ending as close to it as it can: [(lanes in order, first direction)]."""
    r = cfg.turn_radius
    opts = [list(options(cell, angle, cfg)) for cell in cells]

    def heading_of(direction: int) -> float:
        return angle if direction > 0 else angle + math.pi

    def out(o) -> float:  # the way the last lane goes
        return heading_of(o[1] if len(o[0]) % 2 else -o[1])

    start = [[distance(pos, o[2]) + (0.0 if heading is None else r * angle_diff(heading_of(o[1]), heading))
              for o in os] for os in opts]
    last = [[1000.0 * distance(end, o[3]) if end is not None else 0.0 for o in os] for os in opts]

    def link(i: int, a: int, j: int, b: int) -> float:
        p, q = opts[i][a], opts[j][b]
        return math.hypot(q[2][0] - p[3][0], q[2][1] - p[3][1]) + r * angle_diff(heading_of(q[1]), out(p))

    return [(opts[i][a][0], opts[i][a][1]) for i, a in best_order(start, link, last)]
