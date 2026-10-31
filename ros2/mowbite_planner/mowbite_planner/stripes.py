"""The lanes as runs the body fits all along, and the ways to mow a cell: its lanes one after the other, neighbour
after neighbour (snake) or, with lane_order skip, rows apart for U-turns in passes all going the same way across (no
spiral), the turns between from the catalogue."""

import dataclasses
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .config import XY, Lane, PlannerConfig
from .footprint import Footprint
from .maneuvers import sample_straight
from .mathutil import Pose2D
from .objective import AREA_EPS, LOOP_REACH, S_PER_M2, Cost, better, counted, seconds
from .swaths import BODY_STEP, body_ends, centre_line
from .transit import TransitPlanner
from .turns import Catalog, Turn


@dataclasses.dataclass(frozen=True)
class Run:
    """A lane driven one way: the centre from a to b facing yaw, the body fitting all along (trimmed: shorter for it,
    clipped: shorter where blade_offset or blade_ahead take the centre out)."""
    a: XY
    b: XY
    yaw: float
    trimmed: bool = False
    clipped: bool = False

    @property
    def length(self) -> float:
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])

    def start(self, cut: float = 0.0) -> Pose2D:
        return Pose2D(self.a[0] + cut * math.cos(self.yaw), self.a[1] + cut * math.sin(self.yaw), self.yaw)

    def end(self, cut: float = 0.0) -> Pose2D:
        return Pose2D(self.b[0] - cut * math.cos(self.yaw), self.b[1] - cut * math.sin(self.yaw), self.yaw)


def run_of(lane: Lane, direction: int, angle: float, cfg: PlannerConfig, area: TransitPlanner,
           fp: Optional[Footprint], arrive: float = 0.0, leave: float = 0.0,
           known: Optional[Dict[Tuple[float, float], bool]] = None) -> Tuple[Optional[Run], str]:
    """lane driven +x (direction 1) or -x as the centre drives it, its ends pulled back for the body and the turns' arcs
    (swaths), the longest stretch the body fits at every pose: (that or None, why not: a count's key). known: see
    body_ends (the same lane and direction)."""
    said: List[str] = []
    s = centre_line(lane, direction, angle, cfg, area, said.append)
    if s is None:
        return None, 'lanes_shortened'
    clipped = bool(said)
    if fp is not None:
        s = body_ends(fp, cfg, s, arrive, leave, said.append, known)
    if s is None:
        return None, 'lanes_body'
    run = Run(s.a, s.b, s.yaw, 'lanes_body' in said, clipped)
    if fp is None:
        return run, ''
    poses = sample_straight(run.a[0], run.a[1], run.b[0], run.b[1], run.yaw, cfg.waypoint_spacing)
    ok = fp.fit(poses)
    if all(ok):
        return run, ''
    best, i = None, 0
    while i < len(ok):  # (the longest stretch of poses that fit)
        j = i
        while j < len(ok) and ok[j]:
            j += 1
        if j > i and (best is None or j - i > best[1] - best[0] + 1):
            best = (i, j - 1)
        i = j + 1
    if best is None:
        return None, 'lanes_body'
    ux, uy = math.cos(run.yaw), math.sin(run.yaw)

    def along(k: int) -> float:
        return (poses[k].x - run.a[0]) * ux + (poses[k].y - run.a[1]) * uy

    def edge(inside: float, outside: float) -> float:
        while abs(outside - inside) > BODY_STEP:
            t = 0.5 * (inside + outside)
            fits = fp.fits_pose(run.a[0] + t * ux, run.a[1] + t * uy, run.yaw)
            inside, outside = (t, outside) if fits else (inside, t)
        return inside

    i, j = best
    t0 = along(i) if i == 0 else edge(along(i), along(i - 1))
    t1 = along(j) if j == len(poses) - 1 else edge(along(j), along(j + 1))
    if t1 - t0 < max(cfg.min_lane_length, 1e-6):
        return None, 'lanes_body'
    return Run((run.a[0] + t0 * ux, run.a[1] + t0 * uy), (run.a[0] + t1 * ux, run.a[1] + t1 * uy), run.yaw, True,
               clipped), ''


@dataclasses.dataclass
class Way:
    """A way to mow a cell: its lanes in order, each a run one way, the turn after each but the last (None: a drive),
    what that costs (skipped: m2 of it the lanes left out leave, left_out: why each, wide: rows apart). turns None: not
    worked out yet."""
    runs: List[Tuple[Lane, Run]]
    turns: Optional[List[Optional[Turn]]]
    cost: Cost
    skipped: float = 0.0
    left_out: Tuple[str, ...] = ()
    wide: bool = False

    @property
    def entry(self) -> Pose2D:
        return self.runs[0][1].start(0.0)

    @property
    def exit(self) -> Pose2D:
        return self.runs[-1][1].end(0.0)


GuessFn = Callable[[Pose2D, Pose2D], Cost]
Runs = Callable[[Lane, int, float, float], Tuple[Optional[Run], str]]


class Cells:
    """The ways to mow cells: their runs, the turns from the catalogue, the drives guessed, band(lane) m2 a lane left
    out leaves."""

    def __init__(self, cfg: PlannerConfig, runs: Runs, catalog: Catalog, guess: GuessFn,
                 band: Callable[[Lane], float]) -> None:
        self.cfg, self.runs, self.catalog, self.guess, self.band = cfg, runs, catalog, guess, band

    def ways(self, cell: Sequence[Lane]) -> List[Way]:
        """The ways to mow the cell (its lanes one a row) in its pattern, from either end, either way first, turns
        worked out later (realize). With lane_order skip the rows apart (the pattern is rank 4), neighbour after
        neighbour only where that leaves fewer lanes out or makes fewer loops and turns on the spot (_kept)."""
        n = len(cell)
        order = list(range(n))
        cost = Cost(time_s=self._least(cell, order))
        k = _jump(self.cfg, abs(cell[1].y - cell[0].y)) if n > 1 else 1
        if self.cfg.lane_order == 'skip' and 1 < k < n:
            rows = [i for p in range(k) for i in range(p, n, k)]
            way = self._way(cell, rows, 1, math.inf, True)
            if way is None or way.skipped > AREA_EPS or way.cost.loops or way.cost.spins:
                snake = self._way(cell, order, 1, math.inf, False)
                if way is None or (snake is not None and better(_kept(snake), _kept(way))):
                    way, rows = snake, order
            if way is not None:
                order, cost = rows, dataclasses.replace(way.cost, left_m2=way.cost.left_m2 - way.skipped)
        out = []
        orders = (order, order[::-1]) if n > 1 else (order,)
        for o, first in ((o, d) for o in orders for d in (1, -1)):
            picked, skipped, why = self._picked(cell, o, first)
            if picked:
                out.append(Way(picked, None, cost + Cost(left_m2=skipped), skipped, why, order != sorted(order)))
        return out

    def _least(self, cell: Sequence[Lane], order: List[int]) -> float:
        """A lower bound (s) of the turns of the cell's lanes in order (Catalog.bound), from the lanes' ends."""
        total, direction = 0.0, 1
        for i, k in zip(order, order[1:]):
            a, b = cell[i], cell[k]
            dx = b.x1 - a.x1 if direction > 0 else a.x0 - b.x0
            total += self.catalog.bound(abs(b.y - a.y), dx)
            direction = -direction
        return total

    def realize(self, way: Way) -> Way:
        """way with its turns worked out."""
        if way.turns is not None:
            return way
        got = self._turned(way.runs, math.inf, way.wide)
        turns, cost = got if got is not None else ([None] * (len(way.runs) - 1), Cost())
        return Way(way.runs, turns, cost + Cost(left_m2=way.skipped), way.skipped, way.left_out, way.wide)

    def _picked(self, cell: Sequence[Lane], order: List[int],
                first: int) -> Tuple[List[Tuple[Lane, Run]], float, Tuple[str, ...]]:
        """The cell's lanes in order with their runs, the first driven first (1: +x), each next the other way; the
        lanes with no run that way left out: (lanes and runs, m2 those leave, why each)."""
        picked: List[Tuple[Lane, Run]] = []
        skipped, why = 0.0, []
        for i, k in enumerate(order):
            lane, direction = cell[k], first if len(picked) % 2 == 0 else -first
            # (the sides the turns come from and go to: left of a lane going +x is +y)
            near = [cell[order[j]] if 0 <= j < len(order) else None for j in (i - 1, i + 1)]
            side = [0.0 if o is None or (j == 0 and not picked) else direction * math.copysign(1.0, o.y - lane.y)
                    for j, o in enumerate(near)]
            run, reason = self.runs(lane, direction, side[0], side[1])
            if run is None:
                skipped += self.band(lane)
                why.append(reason)
            else:
                picked.append((lane, run))
        return picked, skipped, tuple(why)

    def _way(self, cell: Sequence[Lane], order: List[int], first: int, bound: float, wide: bool) -> Optional[Way]:
        picked, skipped, why = self._picked(cell, order, first)
        got = self._turned(picked, bound - S_PER_M2 * skipped, wide) if picked else None
        if got is None:
            return None
        turns, cost = got
        return Way(picked, turns, cost + Cost(left_m2=skipped), skipped, why, wide)

    def _turned(self, picked: List[Tuple[Lane, Run]], bound: float,
                wide: bool) -> Optional[Tuple[List[Optional[Turn]], Cost]]:
        """The turns between the lanes (from the catalogue, the drives where none fits guessed) and what they cost,
        None as soon as that's more than bound."""
        cfg = self.cfg
        turns: List[Optional[Turn]] = []
        cost, cut = Cost(), 0.0  # (cut: how much later the lane starts)
        for (_, u), (_, v) in zip(picked, picked[1:]):
            room = max(0.0, u.length - cut - cfg.min_lane_length)
            t = self.catalog.turn(u.end(), v.start(), room, 0.5 * max(0.0, v.length - cfg.min_lane_length), wide)
            turns.append(t)
            cost = cost + (t.cost if t is not None else self.guess(u.end(), v.start()))
            if seconds(cost) > bound + 1e-9:
                return None
            cut = t.cut_entry if t is not None else 0.0
        return turns, cost + Cost(loops=_loops_round(picked, turns))


def _kept(way: Way) -> Cost:
    """What a pattern is given up for: the lanes it leaves out, its loops and turns on the spot. Not what the turns
    leave where the lanes end earlier for them: of the rows apart the other passes' turns and the completion stage mow
    nearly all of it (torture/1@mkay: 935 cm2 counted, 7 cm2 more left in the plan)."""
    return Cost(left_m2=way.skipped, loops=way.cost.loops, spins=way.cost.spins)


def _jump(cfg: PlannerConfig, spacing: float) -> int:
    """Rows apart a U-turn of the least radius fits between (lane_order skip)."""
    if spacing <= 0.0:
        return 1
    return max(1, int(math.ceil((2.0 * cfg.min_turn_radius + 2.0 * abs(cfg.blade_offset)) / spacing - 1e-9)))


def _loops_round(picked: List[Tuple[Lane, Run]], turns: List[Optional[Turn]]) -> int:
    """Loops two turns make round a lane too short to be between them all the way round."""
    n = 0
    for i in range(1, len(picked) - 1):
        a, b = turns[i - 1], turns[i]
        if a is not None and b is not None and \
                picked[i][1].length - a.cut_entry - b.cut_exit < 2.0 * LOOP_REACH:
            n += counted(list(a.poses) + list(b.poses)).loops
    return n


def lane_poses(run: Run, cut_start: float, cut_end: float, step: float) -> List[Pose2D]:
    """The run's poses from cut_start in to cut_end before its end."""
    a, b = run.start(cut_start), run.end(cut_end)
    return sample_straight(a.x, a.y, b.x, b.y, run.yaw, step)


def kept_runs(cfg: PlannerConfig, angle: float, area: TransitPlanner, fp: Optional[Footprint]) -> Runs:
    """run_of for every lane, direction and side of the turns, worked out once."""
    known: Dict[Tuple[int, int, float, float], Tuple[Optional[Run], str]] = {}
    ends: Dict[Tuple[int, int], Dict[Tuple[float, float], bool]] = {}  # (body_ends' of each lane and direction)

    def runs(lane: Lane, direction: int, arrive: float, leave: float) -> Tuple[Optional[Run], str]:
        key = (id(lane), direction, arrive, leave)
        if key not in known:
            known[key] = run_of(lane, direction, angle, cfg, area, fp, arrive, leave,
                                ends.setdefault((id(lane), direction), {}))
        return known[key]

    return runs
