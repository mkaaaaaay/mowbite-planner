"""The order things are mowed in: groups (cells) each done one of several ways, a generalised travelling salesman
from a start to an end; exactly up to EXACT groups, else the cheapest next each time and moves of runs of them
(best_order); the drives between guessed (Guess), what each guess costs in the ranking's seconds."""

import math
from typing import Callable, Dict, List, Optional, Tuple

from .config import ORDER_BUDGET, XY, PlannerConfig
from .mathutil import Pose2D, angle_diff
from .objective import SPIN_MIN, Cost
from .report import FTC_SPEEDS
from .transit import TransitPlanner

# groups ordered exactly (dynamic programming over the subsets) up to this many
EXACT = 8
# a drive the transit space has no way for is guessed this many times as long as straight there
NO_WAY = 3.0

Link = Callable[[int, int, int, int], float]


def plan_order(start: List[List[float]], link: Link, end: List[List[float]], improve: bool = True,
               budget: int = ORDER_BUDGET) -> List[Tuple[int, int]]:
    """[(group, way)] in order, from start[i][a] (beginning with group i done way a) by link(i, a, j, b) to end[i][a]
    (finishing with it), the lowest total. improve False: the cheapest next each time (route_order nearest)."""
    if improve and len(start) <= EXACT:
        return exact_order(start, link, end)
    # (the group that ends closest to the end last, from the start)
    last = min(range(len(start)), key=lambda i: (min(end[i]), i)) if len(start) > 1 and any(map(any, end)) else None
    return best_order(start, link, end, budget=budget if improve else 1, last=last)


def exact_order(start: List[List[float]], link: Link, end: List[List[float]]) -> List[Tuple[int, int]]:
    """The cheapest order of all, by dynamic programming over the groups mowed so far (2^n of them)."""
    n = len(start)
    if n == 0:
        return []
    states = [(i, a) for i in range(n) for a in range(len(start[i]))]
    links = {(s, t): link(s[0], s[1], t[0], t[1]) for s in states for t in states if s[0] != t[0]}
    best: Dict[Tuple[int, int, int], Tuple[float, Optional[Tuple[int, int, int]]]] = {
        (1 << i, i, a): (start[i][a], None) for i, a in states}
    for mask in range(1, 1 << n):
        for i, a in states:
            here = best.get((mask, i, a))
            if here is None:
                continue
            for j, b in states:
                if mask >> j & 1:
                    continue
                key, c = (mask | 1 << j, j, b), here[0] + links[(i, a), (j, b)]
                if key not in best or c < best[key][0] - 1e-9:
                    best[key] = (c, (mask, i, a))
    full = (1 << n) - 1
    key = min(((full, i, a) for i, a in states if (full, i, a) in best),
              key=lambda k: (best[k][0] + end[k[1]][k[2]], k[1], k[2]))
    out = []
    while key is not None:
        out.append((key[1], key[2]))
        key = best[key][1]
    return out[::-1]


def best_order(start: List[List[float]], link, end: List[List[float]],
               first: Optional[List[int]] = None, budget: int = ORDER_BUDGET,
               last: Optional[int] = None) -> List[Tuple[int, int]]:
    """Like plan_order: from `first` (default: the cheapest next each time, item last at the end), runs of up to three
    moved and runs reversed while that helps, at most `budget` orders tried, the ways exactly (dynamic programming)."""
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

    order = list(first) if first is not None else _greedy(start, links, k, last)
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


def _greedy(start: List[List[float]], links, k: List[int], last: Optional[int] = None) -> List[int]:
    """best_order's first order: the cheapest next one each time, last (an item) at the end."""
    left, first, way = set(range(len(start))) - {last}, [], None
    if not left:
        return [last]
    while left:
        if not first:
            i = min(left, key=lambda i: (min(start[i]), i))
            way = min(range(k[i]), key=lambda a: (start[i][a], a))
        else:
            p = first[-1]
            i, way = min(((j, b) for j in left for b in range(k[j])), key=lambda jb: (links[p, jb[0]][way][jb[1]], jb))
        first.append(i)
        left.remove(i)
    return first + ([last] if last is not None else [])


class Guess:
    """About what a drive costs: the way through the transit space the drives take (straight where that's free, off the
    edges with transit_edge_distance, that counted), on arcs of turn_radius where it turns, on the spot where it turns
    round on a way too short for them."""

    def __init__(self, transit: TransitPlanner, cfg: PlannerConfig) -> None:
        self.transit, self.r = transit, max(cfg.turn_radius, 1e-3)
        self.careful = cfg.transit_edge_distance > 0.0 and cfg.transit_edge_cost > 0.0
        self.fast, self.slow = FTC_SPEEDS['speed_fast'], FTC_SPEEDS['speed_slow']
        self.omega = math.radians(FTC_SPEEDS['speed_angular'])
        self.ways: Dict[Tuple[float, float, float, float], Optional[List[XY]]] = {}

    def way(self, a: XY, b: XY) -> Optional[List[XY]]:
        key = (round(a[0], 4), round(a[1], 4), round(b[0], 4), round(b[1], 4))
        if key not in self.ways:
            self.ways[key] = self.transit.careful_path(a, b) if self.careful else self.transit.path(a, b)
        return self.ways[key]

    def __call__(self, a: Pose2D, b: Pose2D, dock: bool = False, straight: bool = False) -> Cost:
        """From a to b, facing their yaw there (None: any way); dock: a way to or from the dock; straight: straight
        there, the way not looked for (quicker)."""
        pts = [(a.x, a.y), (b.x, b.y)] if straight and not self.careful else self.way((a.x, a.y), (b.x, b.y))
        stretch = 1.0 if pts is not None else NO_WAY
        pts = pts or [(a.x, a.y), (b.x, b.y)]
        legs = [(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]) if math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-6]
        if not legs:
            turn = angle_diff(a.yaw, b.yaw) if a.yaw is not None and b.yaw is not None else 0.0
            return Cost(spins=int(turn >= SPIN_MIN), time_s=turn / self.omega)
        # (a way along the edges as dear as its cost there says, see TransitPlanner.cost)
        length = stretch * (self.transit.cost(pts) if self.careful else sum(math.hypot(x, y) for x, y in legs))
        yaws = [math.atan2(y, x) for x, y in legs]
        ends = [angle_diff(a.yaw, yaws[0]) if a.yaw is not None else 0.0,
                angle_diff(yaws[-1], b.yaw) if b.yaw is not None else 0.0]
        bends = sum(angle_diff(p, q) for p, q in zip(yaws, yaws[1:]))
        arc = min(length, self.r * (sum(ends) + bends))
        # (turning round at an end of a way too short for its arc: on the spot)
        spins = sum(1 for e in ends if e > 0.5 * math.pi and length < 2.0 * self.r)
        t = (length - arc) / self.fast + arc / self.slow + spins * math.pi / self.omega
        return Cost(spins=spins, time_s=t, transit_s=t, dock_s=t if dock else 0.0)
