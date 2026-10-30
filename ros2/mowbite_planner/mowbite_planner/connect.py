"""The turns from one lane into the next: plan_turn's, a rounded one with the lanes ending earlier in place of turning
on the spot or a loop (round_earlier), the half circle (turn_radius_auto). Worked out, not driven: Turn says what
changes."""

import dataclasses
import math
from typing import Any, Callable, List, Optional, Protocol, Sequence, Tuple

from .config import ROUND_STEP, XY, PlannerConfig
from .footprint import Footprint
from .geometry import subtract, union
from .maneuvers import Fits, Maneuver, dubins, plan_turn, turn_radii
from .mathutil import Pose2D, angle_diff
from .objective import EARLIER_LEFT, ROUND_LEFT
from .sweep import blade

# what's mowed already near lines (x0, y0, x1, y1), the lane just mowed not counted
Mowed = Callable[[List[Tuple[float, float, float, float]]], Any]


class Ways(Protocol):
    """Where turns may go (rings_old._Route has it)."""
    cfg: PlannerConfig
    footprint: Optional[Footprint]
    step: float
    rounds: bool  # a turn may be rounded off (an arc allowed, see min_turn_radius)

    def body(self, poses: Sequence[Pose2D]) -> bool: ...

    def turn_body(self, poses: Sequence[Pose2D]) -> bool: ...

    def turn_space(self, at: Pose2D, a: XY) -> Fits: ...

    def detour_path(self, a: XY, b: XY) -> Optional[List[XY]]: ...


@dataclasses.dataclass
class Turn:
    """A turn into the lane starting at entry; lane: the poses of the lane before where it ends earlier for it."""
    maneuver: Maneuver
    entry: XY
    lane: Optional[List[Pose2D]] = None
    counts: Tuple[str, ...] = ()


def _plan(ways: Ways, at: Pose2D, entry: XY, types: Sequence[str]) -> Optional[Maneuver]:
    cfg = ways.cfg
    return plan_turn(at, entry, cfg.turn_radius, ways.step, ways.turn_space(at, entry), ways.detour_path, types,
                     ways.turn_body, cfg.min_turn_radius if ways.rounds else 0.0, tight=not cfg.turn_radius_auto,
                     wider=cfg.turn_radius_auto)


def between(ways: Ways, lane: List[Pose2D], a: XY, yaw: float, head: XY, mowed: Mowed) -> Optional[Turn]:
    """The turn from the end of lane (its poses) into the next, from a facing yaw straight to head. Where the body
    may not fit a turn level with a (the lanes end at different places), the gentle turns level with lane's end first,
    the next lane starting there."""
    cfg, fp = ways.cfg, ways.footprint
    at = lane[-1]
    ux, uy = math.cos(yaw), math.sin(yaw)
    level = (at.x - a[0]) * ux + (at.y - a[1]) * uy
    rest = math.hypot(head[0] - a[0], head[1] - a[1]) - cfg.min_lane_length
    # (not by more than a turn reaches: what it leaves of the lane would stay unmowed)
    most = min(rest, fp.reach + cfg.turn_radius) if fp is not None else 0.0
    turn, types, counts = None, cfg.allowed_turns, ()
    if fp is not None and 1e-6 < level < most:
        gentle = tuple(t for t in cfg.allowed_turns if t not in ('detour', 'pivot'))
        turn = _plan(ways, at, a, gentle)
        if turn is None:
            types = tuple(t for t in types if t not in gentle)  # (those don't fit at a)
            start = (a[0] + level * ux, a[1] + level * uy)
            turn = _plan(ways, at, start, gentle)
            if turn is not None:
                a, counts = start, ('lanes_body',)
    if turn is None:
        turn = _plan(ways, at, a, types)
    return _rounded(ways, Turn(turn, a, None, counts) if turn is not None else None, at, lane, a, yaw, rest, mowed,
                    'pivot' in types, None)


def into_next(ways: Ways, at: Optional[Pose2D], lane: Optional[List[Pose2D]], a: XY, yaw: float, head: XY,
              mowed: Mowed) -> Optional[Turn]:
    """Where the next lane starts right next to at (the end of the path), the other way round: the turn into it, not a
    drive. lane: the poses of the lane the path ends with (None where it ends otherwise). None where none fits."""
    cfg = ways.cfg
    if at is None or angle_diff(yaw, at.yaw + math.pi) > 1e-3 or \
            math.hypot(a[0] - at.x, a[1] - at.y) > 4.0 * cfg.turn_radius:
        return None
    space = ways.turn_space(at, a)
    turn = plan_turn(at, a, cfg.turn_radius, ways.step, space, ways.detour_path, cfg.allowed_turns, ways.turn_body,
                     cfg.min_turn_radius if ways.rounds else 0.0, tight=not cfg.turn_radius_auto,
                     wider=cfg.turn_radius_auto)
    rest = math.hypot(head[0] - a[0], head[1] - a[1]) - cfg.min_lane_length
    return _rounded(ways, Turn(turn, a) if turn is not None else None, at, lane, a, yaw, rest, mowed,
                    'pivot' in cfg.allowed_turns, space)


def _rounded(ways: Ways, turn: Optional[Turn], at: Pose2D, lane: Optional[List[Pose2D]], a: XY, yaw: float,
             rest: float, mowed: Mowed, pivots: bool, space: Optional[Fits]) -> Optional[Turn]:
    """turn, where it's none or on the spot: round_earlier in its place, then the half circle (space: where that goes,
    None: from where lane ends then)."""
    if (turn is None or turn.maneuver.name == 'pivot') and ways.rounds:
        earlier = round_earlier(ways, lane, a, yaw, rest, mowed)
        if earlier is not None:
            turn = dataclasses.replace(earlier, counts=(turn.counts if turn else ()) + ('lanes_earlier',))
            a = turn.entry
    if (turn is None or turn.maneuver.name == 'pivot') and pivots:
        now = turn.lane if turn is not None and turn.lane is not None else lane
        fits = space if space is not None else ways.turn_space(now[-1], a)
        half = half_circle(ways, at, now, a, yaw, rest, fits, mowed)
        if half is not None:
            turn = dataclasses.replace(half, lane=half.lane or (turn.lane if turn else None),
                                       counts=(turn.counts if turn else ()) + half.counts)
    if turn is None:
        return None
    return dataclasses.replace(turn, counts=turn.counts + ('turns_' + turn.maneuver.name,))


def half_circle(ways: Ways, at: Pose2D, lane: Optional[List[Pose2D]], a: XY, yaw: float, rest: float, fits: Fits,
                mowed: Mowed) -> Optional[Turn]:
    """turn_radius_auto, where no turn of turn_radius gets from at into the next lane (from a, facing yaw, rest m of it
    to spare), the lanes ending earlier for one neither: only now the half circle tighter than turn_radius, as wide as
    the lanes are apart, there or with the lanes ending earlier (round_earlier), before turning on the spot."""
    cfg = ways.cfg
    if not cfg.turn_radius_auto:
        return None  # (plan_turn and round_earlier tried it)
    half = plan_turn(at, a, cfg.turn_radius, ways.step, fits, None, ('pivot',), ways.turn_body,
                     cfg.min_turn_radius if ways.rounds else 0.0, False)
    found = Turn(half, a) if half is not None else \
        round_earlier(ways, lane, a, yaw, rest, mowed, True) if ways.rounds else None
    return None if found is None else dataclasses.replace(found, counts=('turns_tight',))


def round_earlier(ways: Ways, lane: Optional[List[Pose2D]], a: XY, yaw: float, rest: float, mowed: Mowed,
                  half: bool = False) -> Optional[Turn]:
    """In place of turning on the spot (or a drive) from lane (its poses, just mowed) into the next (from a, facing
    yaw, rest m of it to spare): a rounded turn level with the end of the shorter of the two, the longer one ending (or
    starting) earlier, then both, ROUND_STEP at a time up to turn_radius more: a U-turn (at a slant where they end at
    one), a bulb, a detour between lanes far apart. The first that fits, where the turn or what's mowed already (the
    loops next to them) mows what they leave (but EARLIER_LEFT, ROUND_LEFT where it would else loop). Else a loop into
    the next lane (Dubins' shortest way that fits). None where none fits. half: only the half circle tighter than
    turn_radius (see half_circle, the rest is tried by then)."""
    if lane is None or len(lane) < 2:
        return None
    cfg = ways.cfg
    first, end = lane[0], lane[-1]
    h = end.yaw
    if any(angle_diff(p.yaw, h) > 1e-6 for p in lane):
        return None  # a bent lane
    length = math.hypot(end.x - first.x, end.y - first.y)
    r, hx, hy, ux, uy = cfg.turn_radius, math.cos(h), math.sin(h), math.cos(yaw), math.sin(yaw)
    dx = (a[0] - end.x) * hx + (a[1] - end.y) * hy  # how much further on the next lane starts
    most_e = min(max(0.0, -dx) + r, length - cfg.min_lane_length)
    most_n = min(max(0.0, dx) + r, rest)
    fits = ways.turn_space(end, a)
    loop: List[Optional[Maneuver]] = []
    done: List[Any] = []

    def looped() -> Optional[Maneuver]:
        # the loop into the next lane where nothing else fits, worked out once
        if not loop:
            loop.append(_shortest(ways, fits, end, a, yaw, True) if 'bulb' in cfg.allowed_turns and not half else None)
        return loop[0]

    def unmowed(region):
        # what of region isn't mowed already (but by the lane just mowed): the path near where the two lanes end and
        # start as far as they may get shorter, worked out once
        if not done:
            done.append(mowed([(end.x - most_e * hx, end.y - most_e * hy, end.x, end.y),
                               (a[0], a[1], a[0] + most_n * ux, a[1] + most_n * uy)]))
        return subtract(region, done)

    for de, dn, exit_pose, entry, turn in _earlier_turns(ways, fits, end, a, yaw, dx, most_e, most_n, half):
        left = [blade(cfg, [exit_pose, end])] if de > 1e-9 else []
        left += [blade(cfg, [Pose2D(a[0], a[1], yaw), Pose2D(entry[0], entry[1], yaw)])] if dn > 1e-9 else []
        if left:
            there = subtract(union(left), [blade(cfg, [p for piece in turn.pieces for p in piece.poses]),
                                           blade(cfg, [first, exit_pose])])
            if there.area > EARLIER_LEFT and unmowed(there).area > EARLIER_LEFT and (
                    looped() is None or there.area > ROUND_LEFT and unmowed(there).area > ROUND_LEFT):
                break  # (further in it leaves more)
        if de <= 1e-9:
            return Turn(turn, entry)
        return Turn(turn, entry, [p for p in lane if (p.x - first.x) * hx + (p.y - first.y) * hy < length - de - 1e-9]
                    + [exit_pose])
    turn = looped()
    return None if turn is None else Turn(turn, a)


def _earlier_turns(ways: Ways, fits: Fits, end: Pose2D, a: XY, yaw: float, dx: float, most_e: float, most_n: float,
                   half: bool):
    """round_earlier's turns that fit, the lanes ending (starting) ROUND_STEP earlier (later) at a time: (how much
    earlier the lane before ends, how much later the next starts, from where, into where, the turn)."""
    cfg = ways.cfg
    r, least, hx, hy = cfg.turn_radius, cfg.min_turn_radius, math.cos(end.yaw), math.sin(end.yaw)
    ux, uy = math.cos(yaw), math.sin(yaw)
    types = tuple(t for t in cfg.allowed_turns if t != 'pivot' and (
        abs((a[1] - end.y) * hx - (a[0] - end.x) * hy) > 4.0 * r or t != 'detour')) + ('pivot',)
    k = 0
    while True:
        level = max(0.0, dx) - k * ROUND_STEP  # where the turn goes, along the lane just mowed from its end
        de, dn = max(0.0, -level), max(0.0, dx - level)  # how much earlier it ends and the next one starts
        if de > most_e + 1e-9 or dn > most_n + 1e-9:
            return
        exit_pose = Pose2D(end.x - de * hx, end.y - de * hy, end.yaw)
        entry = (a[0] + dn * ux, a[1] + dn * uy)
        turn = _shortest(ways, fits, exit_pose, entry, yaw, False) if 'u_turn' in cfg.allowed_turns and not half \
            else None
        if turn is None and k and half:
            turn = plan_turn(exit_pose, entry, r, ways.step, fits, None, ('pivot',), ways.turn_body, least, False)
        elif turn is None and k:
            turn = plan_turn(exit_pose, entry, r, ways.step, fits, ways.detour_path, types, ways.turn_body, least,
                             False, not cfg.turn_radius_auto, cfg.turn_radius_auto)
        k += 1
        if turn is not None:
            yield de, dn, exit_pose, entry, turn


def _shortest(ways: Ways, fits: Fits, exit_pose: Pose2D, entry: XY, yaw: float, loops: bool) -> Optional[Maneuver]:
    """Dubins' shortest way from exit_pose into the lane from entry that fits, the widest first."""
    r = ways.cfg.turn_radius
    for rr in turn_radii(r, 4.0 * r, ways.cfg.min_turn_radius):
        for m in dubins(exit_pose, Pose2D(entry[0], entry[1], yaw), rr, ways.step, loops):
            poses = m.pieces[0].poses
            if fits([(p.x, p.y) for p in poses]) and (ways.footprint is None or ways.turn_body(poses)):
                return m
    return None
