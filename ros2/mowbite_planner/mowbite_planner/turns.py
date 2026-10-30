"""The turns from the end of one lane into the next, driven the other way, as a catalogue: each family a function of
where the one ends and the other starts, tried with the lanes ending earlier (starting later) where the body or the
space needs that, the best by objective.better. No loops: none goes round where a plainer one gets there."""

import dataclasses
import math
from typing import Callable, Dict, Iterator, List, Optional, Protocol, Sequence, Tuple

from shapely.geometry import LineString

from .config import ROUND_STEP, XY, PlannerConfig
from .bodyfit import TURN_AHEAD
from .footprint import SPIN, Footprint
from .geometry import subtract
from .maneuvers import Fits, _to_map, bulb_turn, detour_turn, dubins, pivot_turn, turn_radii, u_turn
from .mathutil import Pose2D, angle_diff
from .objective import AREA_EPS, KINK_ANGLE, KINK_LEN, KINK_S, SPIN_S, Cost, better, counted
from .report import FTC_SPEEDS
from .sweep import blade

# m, a turn moves this much further back into the lanes at a time where it doesn't fit at their ends
CUT_STEP = ROUND_STEP


class TurnRoute(Protocol):
    """What the turns need of the route (rings_old._Route has it)."""
    cfg: PlannerConfig
    footprint: Optional[Footprint]
    step: float
    rounds: bool  # arcs allowed (see min_turn_radius)

    def turn_space(self, at: Pose2D, a: XY) -> Fits: ...

    def detour_path(self, a: XY, b: XY) -> Optional[List[XY]]: ...


@dataclasses.dataclass(frozen=True)
class Turn:
    """From a lane ending cut_exit earlier into the next starting cut_entry later: the poses from the one's end to
    the other's start, the stats name of its family, its cost (left_m2: what the cuts leave)."""
    name: str
    poses: Tuple[Pose2D, ...]
    cut_exit: float
    cut_entry: float
    cost: Cost


Family = Callable[['Catalog', Pose2D, Pose2D, float, float, float], Iterator[List[Pose2D]]]


def _mapped(m, at: Pose2D, side: float) -> List[Pose2D]:
    return _to_map(m, at, side).pieces[0].poses


def _u(c: 'Catalog', at: Pose2D, into: Pose2D, d: float, dx: float, side: float) -> Iterator[List[Pose2D]]:
    """A U-turn of turn_radius (no wider: it would take longer and more headland)."""
    r = c.cfg.turn_radius
    m = u_turn(d, dx, r, c.step) if r > 1e-3 and d >= 2.0 * r - 1e-9 else None
    if m is not None:
        yield _mapped(m, at, side)


def _slant(c: 'Catalog', at: Pose2D, into: Pose2D, d: float, dx: float, side: float) -> Iterator[List[Pose2D]]:
    """Where the lanes end at a slant: arc, diagonal, arc (Dubins' U-turn that way)."""
    if abs(dx) < 1e-3 or c.cfg.turn_radius <= 1e-3:
        return
    for m in dubins(at, into, c.cfg.turn_radius, c.step, False):
        yield m.pieces[0].poses


def _bulb(c: 'Catalog', at: Pose2D, into: Pose2D, d: float, dx: float, side: float) -> Iterator[List[Pose2D]]:
    """Out the other way a bit, round, back in: lanes closer than two turn_radius."""
    r = c.cfg.turn_radius
    m = bulb_turn(d, dx, r, c.step) if r > 1e-3 and d < 2.0 * r else None
    if m is not None:
        yield _mapped(m, at, side)


def _half(c: 'Catalog', at: Pose2D, into: Pose2D, d: float, dx: float, side: float) -> Iterator[List[Pose2D]]:
    """Tighter than turn_radius: a U-turn as wide as fits, the widest a half circle as wide as the lanes are apart."""
    for r in turn_radii(c.cfg.turn_radius, d):
        m = u_turn(d, dx, r, c.step) if r < c.least - 1e-9 and d >= 2.0 * r - 1e-9 else None
        if m is not None:
            yield _mapped(m, at, side)


def _detour(c: 'Catalog', at: Pose2D, into: Pose2D, d: float, dx: float, side: float) -> Iterator[List[Pose2D]]:
    """Quarter arc out, the shortest way along the headland, quarter arc in: lanes far apart."""
    m = detour_turn(at, (into.x, into.y), c.cfg.turn_radius, c.step, c.route.turn_space(at, (into.x, into.y)),
                    c.route.detour_path, c.body, c.least)
    if m is not None:
        yield m.pieces[0].poses


def _pivot(c: 'Catalog', at: Pose2D, into: Pose2D, d: float, dx: float, side: float) -> Iterator[List[Pose2D]]:
    """On the spot at both corners (turn_on_spot)."""
    yield _mapped(pivot_turn(d, dx, c.step), at, side)


# (family, the turn type that allows it) in tiers: at the lanes' ends the gentle ones (not tighter than turn_radius)
# first, the gentlest first, tighter ones only where none of those fits; where nothing fits there, the lanes end
# earlier, each family as little as it needs, tighter ones only where the gentle ones leave some unmowed. Between rows
# apart (lane_order skip: wide turns) the gentle ones ending earlier before tighter ones at the ends: there the other
# passes' turns mow what the lanes leave ending earlier, between neighbours nothing does. On the spot (turn_on_spot,
# where the lanes end) against the best of all.
# The stats count each by its turn type
GENTLE: Tuple[Tuple[Family, str], ...] = ((_u, 'u_turn'), (_slant, 'u_turn'), (_bulb, 'bulb'), (_detour, 'detour'))
TIGHT: Tuple[Tuple[Family, str], ...] = ((_half, 'u_turn'),)
SPOT: Tuple[Tuple[Family, str], ...] = ((_pivot, 'pivot'),)


class Catalog:
    """The turns of one plan. left(geometry): m2 of it the lanes are there to mow (not the loops'), spacing: the
    lanes' (a lane answers for its band that wide)."""

    def __init__(self, route: TurnRoute, left: Callable[[object], float], spacing: float) -> None:
        cfg = route.cfg
        self.route, self.cfg, self.step, self.fp = route, cfg, route.step, route.footprint
        self.least = cfg.min_turn_radius if route.rounds else 0.0
        self.left, self.band = left, 0.5 * spacing
        # (on the spot with turn_on_spot, and where it's all that's allowed or there's no radius)
        spot = cfg.turn_on_spot or cfg.turn_radius <= 1e-3 or not set(cfg.allowed_turns) & {'u_turn', 'bulb', 'detour'}
        self.tiers = [[(f, name) for f, name in tier if name in cfg.allowed_turns] for tier in (GENTLE, TIGHT)]
        self.stages = {True: [[tier] for tier in self.tiers], False: [self.tiers]}  # (by wide, see turn)
        self.spot = [(f, name) for f, name in SPOT if name in cfg.allowed_turns and spot]
        self.reach = (self.fp.reach if self.fp is not None else 0.0) + cfg.turn_radius
        self.kept: Dict[tuple, Optional[Turn]] = {}

    def bound(self, d: float, dx: float) -> float:
        """A lower bound (s, objective.seconds) of a turn between lanes d apart, ending dx apart along them, by the
        families' shapes alone: no body, no space, no cuts."""
        slow, fast = FTC_SPEEDS['speed_slow'], FTC_SPEEDS['speed_fast']
        r, names = max(self.least, 1e-3), {name for tier in self.tiers + [self.spot] for _, name in tier}
        out = [math.inf]
        if 'u_turn' in names or 'detour' in names:
            out.append(math.pi * r / slow + max(0.0, d - 2.0 * r) / fast if d >= 2.0 * r - 1e-9 else math.inf)
        if 'u_turn' in names and d < 2.0 * r:  # (tighter than r: a half circle)
            h = 0.5 * d
            out.append(math.pi * h / slow + (KINK_S if h * KINK_ANGLE < KINK_LEN else 0.0))
        if 'bulb' in names and d < 2.0 * r:
            theta = math.acos(min(1.0, (d + 2.0 * r) / (4.0 * r)))
            out.append(r * (math.pi + 4.0 * theta) / slow)
        if self.spot:
            out.append(2.0 * SPIN_S + d / fast)
        return min(out) + abs(dx) / fast

    def body(self, poses: Sequence[Pose2D]) -> bool:
        return self.fp is None or self.fp.poses_fit(poses, True)

    def turn(self, end: Pose2D, start: Pose2D, room_end: float, room_start: float,
             wide: bool = False) -> Optional[Turn]:
        """The best turn from a lane ending at end into one starting at start the other way round, the one ending
        up to room_end earlier, the other starting up to room_start later (wide: lanes rows apart). None where none
        fits."""
        key = (end, start, round(room_end, 6), round(room_start, 6), wide)
        if key not in self.kept:
            self.kept[key] = self._best(end, start, room_end, room_start, wide)
        return self.kept[key]

    def _best(self, end: Pose2D, start: Pose2D, room_end: float, room_start: float, wide: bool) -> Optional[Turn]:
        if angle_diff(start.yaw, end.yaw + math.pi) > 1e-3:
            return None
        c, s = math.cos(end.yaw), math.sin(end.yaw)
        vx, vy = start.x - end.x, start.y - end.y
        dx, lateral = vx * c + vy * s, -vx * s + vy * c
        if abs(lateral) < 1e-6:
            return None
        frame = (abs(lateral), dx, 1.0 if lateral >= 0.0 else -1.0)
        best = None
        for tiers in self.stages[wide]:
            best = self._at_ends(end, start, frame, tiers)
            if best is None:  # (the lanes end earlier for it: the least far back each fits, the best of those)
                most = (min(room_end, self.reach), min(room_start, self.reach))
                for tier in tiers:
                    for family, name in tier:
                        if family is _detour and best is not None:
                            continue  # (a detour only where none of the others fits)
                        got = self._first(family, name, end, start, frame, most)
                        if got is not None and (best is None or better(got.cost, best.cost)):
                            best = got
                    if best is not None and best.cost.left_m2 < AREA_EPS:
                        break  # (tighter ones only where the gentle ones leave some unmowed)
            if best is not None:
                break
        for family, name in self.spot:  # (where the lanes end: turning on the spot needs no room ahead)
            got = self._fitted(family, name, end, start, frame, 0.0, 0.0)
            if got is not None and (best is None or better(got.cost, best.cost)):
                best = got
        return best

    def _at_ends(self, end: Pose2D, start: Pose2D, frame, tiers) -> Optional[Turn]:
        """The best turn at the lanes' ends of the first of tiers with one that fits there."""
        for tier in tiers:
            best = None
            for family, name in tier:
                got = self._fitted(family, name, end, start, frame, 0.0, 0.0)
                if got is not None and (best is None or better(got.cost, best.cost)):
                    best = got
                if best is not None and not (best.cost.spins or best.cost.kinks):
                    break  # (the gentlest come first)
            if best is not None:
                return best
        return None

    def _first(self, family: Family, name: str, end: Pose2D, start: Pose2D, frame, most) -> Optional[Turn]:
        """The family's turn the least far back that fits (not where the lanes end, _at_ends tried that), halving up
        to as far back as most allows, assuming further back fits where nearer does (what they leave counted for that
        one only)."""
        levels = list(_levels(frame[1], *most))
        tried: Dict[int, Optional[Turn]] = {0: None}

        def at(k: int) -> Optional[Turn]:
            if k not in tried:
                tried[k] = self._fitted(family, name, end, start, frame, *levels[k], left=False)
            return tried[k]

        lo, hi = 0, len(levels) - 1
        if hi <= 0 or at(hi) is None:
            return None
        while hi - lo > 1:
            k = (lo + hi) // 2
            lo, hi = (lo, k) if at(k) is not None else (k, hi)
        t = at(hi)
        return t if t is None else dataclasses.replace(t, cost=self._left(t, end, start))

    def _fitted(self, family: Family, name: str, end: Pose2D, start: Pose2D, frame, de: float,
                dn: float, left: bool = True) -> Optional[Turn]:
        d, dx, side = frame
        ux, uy = math.cos(end.yaw), math.sin(end.yaw)
        at = Pose2D(end.x - de * ux, end.y - de * uy, end.yaw)
        into = Pose2D(start.x - dn * ux, start.y - dn * uy, start.yaw)
        fits = self.route.turn_space(at, (into.x, into.y))
        for poses in family(self, at, into, d, dx + de - dn, side):
            if fits([(p.x, p.y) for p in poses]) and self.body([at] + poses):
                cost = counted(_as_driven([at] + poses))
                if cost.loops:
                    continue  # (a detour's way round: no loops between lanes)
                t = Turn(name, tuple(poses), de, dn, cost)
                return dataclasses.replace(t, cost=self._left(t, end, start)) if left else t
        return None

    def _left(self, t: Turn, end: Pose2D, start: Pose2D) -> Cost:
        """t's cost with what the lanes leave where they end earlier for it (start later)."""
        de, dn = t.cut_exit, t.cut_entry
        if de <= 1e-9 and dn <= 1e-9:
            return t.cost
        ux, uy = math.cos(end.yaw), math.sin(end.yaw)
        at = Pose2D(end.x - de * ux, end.y - de * uy, end.yaw)
        into = Pose2D(start.x - dn * ux, start.y - dn * uy, start.yaw)
        return dataclasses.replace(t.cost, left_m2=self._cut_left(end, at, start, into, t.poses))

    def _cut_left(self, end: Pose2D, at: Pose2D, start: Pose2D, into: Pose2D, poses: Sequence[Pose2D]) -> float:
        """m2 the lanes' bands where they end earlier (start later) leave that the turn doesn't mow."""
        cfg = self.cfg
        strips = [LineString([_blade_at(cfg, p), _blade_at(cfg, q)]).buffer(self.band, cap_style=2)
                  for p, q in ((at, end), (start, into)) if math.hypot(q.x - p.x, q.y - p.y) > 1e-9]
        cut = strips[0] if len(strips) == 1 else strips[0].union(strips[1])
        return self.left(subtract(cut, [blade(cfg, list(poses))]))


def _as_driven(poses: Sequence[Pose2D]) -> List[Pose2D]:
    """poses with a pose TURN_AHEAD before each point the heading jumps at, facing the way it came (as the plan has
    them, bodyfit._turn_at): counted, those are turns on the spot."""
    out: List[Pose2D] = []
    for q, p in zip(poses, poses[1:]):
        d = math.hypot(p.x - q.x, p.y - q.y)
        if angle_diff(p.yaw, q.yaw) > SPIN and d > 2.0 * TURN_AHEAD:
            f = TURN_AHEAD / d
            out.append(Pose2D(p.x + f * (q.x - p.x), p.y + f * (q.y - p.y), q.yaw))
        out.append(p)
    return poses[:1] + out


def _blade_at(cfg: PlannerConfig, p: Pose2D) -> XY:
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return p.x + cfg.blade_ahead * c - cfg.blade_offset * s, p.y + cfg.blade_ahead * s + cfg.blade_offset * c


def _levels(dx: float, most_end: float, most_start: float) -> Iterator[Tuple[float, float]]:
    """(m the lane before ends earlier, m the next starts later): the longer one first, then both, CUT_STEP at a
    time (the turn moves back from level with the end of the longer lane)."""
    k = 0
    while True:
        level = max(0.0, dx) - k * CUT_STEP
        de, dn = max(0.0, -level), max(0.0, dx - level)
        if de > most_end + 1e-9 or dn > most_start + 1e-9:
            return
        yield de, dn
        k += 1
