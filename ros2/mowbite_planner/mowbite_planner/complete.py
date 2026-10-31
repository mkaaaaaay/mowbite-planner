"""The completion stage: what the path leaves of the lawn mowed by moves the body fits (strokes.py), put into the
path as detours from its loops and drives where the ranking says they're worth it (objective.better), the rest one
after the other from its end (mop_up); or what a mow left, as a path of its own (mowed). Every detour's body is checked
before it's taken like check_plan does (body_out with on_turn, the turns also found from the way), over a window round
it."""

import dataclasses
import math
import types
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import numpy

from .bodyfit import _assemble, _turn_at
from .config import LANE, PERIMETER, TRANSIT, TURN, XY, PathSegment, PlannerConfig, SegmentInfo
from .geometry import polygons
from .footprint import Footprint
from .maneuvers import same_position
from .mathutil import Pose2D
from .objective import S_PER_M2, Cost, better, counted, seconds
from .report import drive_time
from .strokes import ACCESS, BUDGET, COST, MOVE_MIN, SMALL, Kit, Move, Rest, moves_for
from .sweep import blade
from .transit import TransitPlanner

PIECES = 60  # pieces worked on at most, the biggest first
STROKES = 1000  # moves built at most for a plan
DRIVES = 600  # drives worked out at most for a plan
LEAST = 0.01  # m2 a piece mop_up works on at least
LEAST_MOWED = 0.02  # and one of what a mow left (the node's LEFT_PIECE)
# path poses a detour to a move may leave from: NEAR at most, the two closest, then the closest the mower turns any way
# at, SPREAD m of path apart, FAR m there and back at most, KEEP m of path off another detour (their body windows)
NEAR = 5
SPREAD = 1.0
FAR = 8.0
KEEP = 2.0
MERGE = 3.0  # m from a detour's last move to the next one it may take on
LINKED = 7  # of the places for a move, the best this many by straight ways get drive.py's ways too
WINDOW = 12  # poses each side a detour's cost is taken over
CHECK_M = 0.7  # m of path each side of a detour its body is checked over
TURN_DEG, TURN_WIN = math.radians(90.0), 0.3  # a pose on a turn: the way turns this much within this much each side
EPS = 1e-9  # (a corner of exactly 90 deg is one, however the headings add up)
BODY_OUT = 0.01  # m the body may stick out there (the ranking's rule, design 2.4)
SPIN_OUT = 0.002  # m turning on the spot (footprint.TURN_STEP's)
FROM = (PERIMETER, TRANSIT)  # detours leave from loops and drives, lanes and their turns keep the pattern
HOME_SPEED = 0.35  # m/s on the way home from where the path ends (the ranking's)
REACH_SLACK = 0.01  # m the blade's reach is taken further (buffer's corners lie inside the circle)

Window = Tuple[List[Pose2D], List[bool]]  # poses and whether each is on a turn between lanes (body_out's on_turn)


@dataclasses.dataclass(eq=False)
class Detour:
    at: int  # the path's pose it leaves from and gets back to
    parts: List[Tuple[str, List[Pose2D]]]  # (kind, poses) one after the other
    moves: List[Move]
    cost: Cost = Cost()  # what it costs more than the path without
    gain: float = 0.0  # m2 its moves mow of what's left


def _minus(a: Cost, b: Cost) -> Cost:
    return Cost(*(x - y for x, y in zip(dataclasses.astuple(a), dataclasses.astuple(b))))


def _joined(*parts: Window) -> Window:
    """The parts one after the other, one pose a position (the later, see maneuvers.join)."""
    poses: List[Pose2D] = []
    flags: List[bool] = []
    for ps, fs in parts:
        for p, f in zip(ps, fs):
            if poses and same_position(poses[-1], p):
                poses[-1], flags[-1] = p, f
            else:
                poses.append(p)
                flags.append(f)
    return poses, flags


def _plain(poses: Sequence[Pose2D]) -> Window:
    return list(poses), [False] * len(poses)


def _ahead(fp: Footprint, w: Window) -> Window:
    """w with _turn_at's pose before each turn on the spot (where the body fits), with the flag of the pose after."""
    poses, flags = w
    if not poses:
        return [], []
    ahead, _ = _turn_at(types.SimpleNamespace(body=fp.poses_fit), poses,
                        [SegmentInfo(TRANSIT, 0, len(poses) - 1, False, '', 0)])
    on, j = [], 0
    for q in ahead:
        on.append(flags[min(j, len(flags) - 1)])
        j += q is poses[min(j, len(poses) - 1)]
    return ahead, on


def _counted(fp: Footprint, poses: List[Pose2D], transit_s: float = 0.0) -> Cost:
    """objective.counted of poses as they'll be in the plan (_turn_at's poses in)."""
    return counted(_ahead(fp, _plain(poses))[0], transit_s)


def turning(poses: Sequence[Pose2D]) -> numpy.ndarray:
    """Which poses lie on a turn, from the way alone: it turns TURN_DEG at least within TURN_WIN before and after."""
    if not poses:
        return numpy.zeros(0, bool)
    xy = numpy.array([(p.x, p.y) for p in poses])
    s = numpy.concatenate([[0.0], numpy.cumsum(numpy.hypot(*numpy.diff(xy, axis=0).T))])
    yaw = numpy.array([p.yaw for p in poses])
    h = numpy.concatenate([[yaw[0]], yaw[0] + numpy.cumsum(numpy.remainder(numpy.diff(yaw) + math.pi, 2.0 * math.pi)
                                                           - math.pi)])
    lo = numpy.searchsorted(s, s - TURN_WIN - EPS, 'left')
    hi = numpy.searchsorted(s, s + TURN_WIN + EPS, 'right')
    return numpy.array([h[a:b].max() - h[a:b].min() >= TURN_DEG - EPS for a, b in zip(lo, hi)], bool)


class Placer:
    """Puts moves into the path: detours from its loops and drives out to them and back facing as before, the rest one
    after the other from its end (or from start: a path of its own)."""

    def __init__(self, kit: Kit, poses: List[Pose2D], segments: List[SegmentInfo], rest: Rest):
        self.kit, self.rest, self.P = kit, rest, poses
        n = len(poses)
        self.xy = numpy.array([(p.x, p.y) for p in poses]).reshape(-1, 2)
        self.run = numpy.concatenate([[0.0], numpy.cumsum(numpy.hypot(*numpy.diff(self.xy, axis=0).T))])[:n]
        self.ok, self.on_turn = numpy.zeros(n, bool), [False] * n
        self.section = numpy.zeros((n, 2), int)  # (first, last) of the section of each pose
        for k, s in enumerate(segments):
            if s.kind in FROM:
                self.ok[s.start + 1:s.end + 1] = True
            self.on_turn[s.start + 1:s.end + 1] = [s.kind == TURN] * (s.end - s.start)
        first = 0
        for k, s in enumerate(segments):  # (not where the path jumps)
            if k == 0 or segments[k - 1].section != s.section:
                self.ok[s.start], first = False, s.start
            if k == len(segments) - 1 or segments[k + 1].section != s.section:
                self.ok[s.end] = False
                self.section[first:s.end + 1] = (first, s.end)
        self.detours: List[Detour] = []
        self.chain: Optional[Detour] = None
        self.failed: Dict[str, int] = {}
        # (what _priced and _link work out again and again: the path's windows' costs, the straight ways)
        self.windows: Dict[Tuple[int, int], Cost] = {}
        self.lines: Dict[tuple, Optional[Tuple[List[Pose2D], float]]] = {}

    def _fail(self, why: str) -> None:
        self.failed[why] = self.failed.get(why, 0) + 1

    def _link(self, before: Sequence[Pose2D], b: XY, yaw: Optional[float], drives: bool) -> Optional[List[Pose2D]]:
        """From before's last to b (facing yaw): the cheaper of straight on (turning on the spot) and drive.py's way
        (drives), the body checked from before on."""
        kit = self.kit
        key = (tuple(before), b, yaw)
        if key not in self.lines:
            self.lines[key] = self._linked(before, kit.line(before[-1], b, yaw))
        ways = [w for w in [self.lines[key]] + ([self._linked(before, kit.drive(before[-1], b, yaw))] if drives else [])
                if w is not None]
        return min(ways, key=lambda w: w[1])[0] if ways else None

    def _linked(self, before: Sequence[Pose2D], way: Optional[List[Pose2D]]) -> Optional[Tuple[List[Pose2D], float]]:
        """(way, what it costs on from before) where it's allowed and the body fits from before on, else None."""
        kit = self.kit
        if way is None or not kit.inside(way, True).all():
            return None
        joined = _joined(_plain(before), _plain(way))[0]
        if not kit.fp.poses_fit(joined) or kit.turns_in_field(joined[len(before) - 1:]):
            return None
        return way, seconds(_counted(kit.fp, joined))

    def _around(self, i: int, n: int) -> Tuple[int, int]:
        lo, hi = self.section[i]
        return max(lo, i - n), min(hi, i + n)

    def _priced(self, i: int, parts: List[Tuple[str, List[Pose2D]]], moves: List[Move]) -> Detour:
        """The detour from pose i with its cost over WINDOW poses each side and its moves' gain."""
        P, fp = self.P, self.kit.fp
        lo, hi = self._around(i, WINDOW)
        new = _joined(_plain(P[lo:i + 1]), *[_plain(p) for _, p in parts], _plain(P[i + 1:hi + 1]))[0]
        drives = sum(drive_time(p)['s'] for k, p in parts if k == TRANSIT)
        lines = numpy.concatenate([m.lines for m in moves])
        if (lo, hi) not in self.windows:
            self.windows[lo, hi] = _counted(fp, P[lo:hi + 1])
        return Detour(i, parts, list(moves), _minus(_counted(fp, new, drives), self.windows[lo, hi]),
                      self.rest.gain(lines))

    def _out(self, i: int, prior: Optional[Detour], m: Move, drives: bool) -> Optional[Detour]:
        """prior (None: a new detour from pose i) on to m, then back to pose i facing as it did."""
        P = self.P
        parts = list(prior.parts[:-1]) if prior is not None else []
        before = list(prior.moves[-1].poses[-2:]) if prior is not None else list(P[max(0, i - 1):i + 1])
        way = self._link(before, (m.first.x, m.first.y), m.first.yaw, drives)
        if way is None:
            return self._fail('out')
        parts += [(TRANSIT, way), ('', m.poses)]
        way = self._link(m.poses[-2:], (P[i].x, P[i].y), P[i].yaw, drives)
        if way is None:
            return self._fail('back')
        parts.append((TRANSIT, way))
        d = self._priced(i, parts, (prior.moves if prior is not None else []) + [m])
        if prior is not None:
            d.cost, d.gain = _minus(d.cost, prior.cost), self.rest.gain(m.lines)
        return d

    def _bad(self, poses: List[Pose2D], flags: List[bool]) -> Set[Pose2D]:
        """The poses where the body sticks out (with _turn_at's poses before turning on the spot): as body_out has it
        (flags: on a turn between lanes), and on a turn found from the way more than BODY_OUT."""
        if not poses:
            return set()
        fp = self.kit.fp
        ahead, on = _ahead(fp, (poses, flags))
        bad = set(fp.bad_poses(ahead, on))
        turn = (numpy.array(on, bool) | turning(ahead)).tolist()
        more = [k for k in fp.bad_poses(ahead, turn) if k not in bad]
        if more:
            out = fp.out_by([(ahead[k - 1] if k else None, ahead[k]) for k in more], on_turn=[turn[k] for k in more])
            bad.update(k for k, (d, xy) in zip(more, out) if xy is not None and d > BODY_OUT)
        if fp.turning is fp:  # (without spin_margin a turn on the spot is checked at its headings: here along its arcs)
            spins = [k for k in range(1, len(ahead)) if k not in bad and fp.spins(ahead, k)]
            if spins:
                bad.update(k for k, (d, _) in zip(spins, fp.out_by([(ahead[k - 1], ahead[k]) for k in spins]))
                           if d > SPIN_OUT)
        return {ahead[k] for k in bad}

    def _fits(self, before: Window, ways: List[Pose2D], after: Window) -> bool:
        """Whether nothing of before, ways and after sticks out that didn't of before and after."""
        new = _joined(before, _plain(ways), after)
        return not (self._bad(*new) - self._bad(*_joined(before, after)))

    def _window(self, i: int) -> Tuple[Window, Window]:
        lo, hi = self.section[i]
        a = max(lo, int(numpy.searchsorted(self.run, self.run[i] - CHECK_M, 'left')) - 1)
        b = min(hi, int(numpy.searchsorted(self.run, self.run[i] + CHECK_M, 'right')))
        P, t = self.P, self.on_turn
        return (list(P[a:i + 1]), t[a:i + 1]), (list(P[i + 1:b + 1]), t[i + 1:b + 1])

    def _worth(self, d: Optional[Detour]) -> bool:
        return d is not None and d.gain >= MOVE_MIN and better(d.cost, Cost(left_m2=d.gain)) is True

    @staticmethod
    def _value(d: Detour) -> float:
        return seconds(d.cost) - S_PER_M2 * d.gain

    def near(self, m: Move) -> List[int]:
        """Path poses to try a detour to m from: the two closest, then the closest where the mower turns any way."""
        d = numpy.hypot(*(self.xy - (m.first.x, m.first.y)).T) + numpy.hypot(*(self.xy - (m.last.x, m.last.y)).T)
        d[~self.ok] = numpy.inf
        for o in self.detours:
            d[numpy.abs(self.run - self.run[o.at]) < KEEP] = numpy.inf
        if len(d):  # (nor where the chain at the end begins)
            d[(self.section[:, 1] == len(d) - 1) & (self.run > self.run[-1] - KEEP)] = numpy.inf
        out: List[int] = []
        checks = 3 * NEAR
        for i in numpy.argsort(d, kind='stable'):
            if d[i] > FAR or len(out) >= NEAR or checks <= 0:
                break
            i = int(i)
            if any(abs(self.run[i] - self.run[j]) < SPREAD for j in out):
                continue
            if len(out) >= 2:
                checks -= 1
                if not self.kit.turns_at(self.xy[i:i + 1])[0]:
                    continue
            out.append(i)
        return out

    def _best(self, m: Move) -> Optional[Detour]:
        """The cheapest worthwhile detour taking m whose body fits: on from one close by or a new one."""
        places = [(d.at, d) for d in self.detours
                  if math.hypot(d.moves[-1].last.x - m.first.x, d.moves[-1].last.y - m.first.y) <= MERGE]
        places += [(i, None) for i in self.near(m)]
        guess = []
        for k, (i, prior) in enumerate(places):
            d = self._out(i, prior, m, False)
            guess.append((self._value(d) if self._worth(d) else math.inf, k))
        found = []
        for _, k in sorted(guess)[:LINKED]:
            i, prior = places[k]
            d = self._out(i, prior, m, True)
            if self._worth(d):
                found.append((self._value(d), k, d, prior))
        for _, _, d, prior in sorted(found, key=lambda f: f[:2]):
            before, after = self._window(d.at)
            ways = _joined(*[_plain(p) for _, p in d.parts])[0]
            if self._fits(before, ways, after):
                if prior is not None:
                    self.detours.remove(prior)
                    d = dataclasses.replace(d, cost=prior.cost + d.cost, gain=prior.gain + d.gain)
                return d
            self._fail('body')
        return None

    def place(self, moves: List[Move]) -> List[Move]:
        """Each move, the most mowing first, where it costs least; those with no place on the way (for the chain)."""
        later = []
        for m in sorted(moves, key=lambda m: (-m.gain, m.piece)):
            if self.rest.gain(m.lines) < MOVE_MIN:
                continue
            d = self._best(m)
            if d is None:
                later.append(m)
                continue
            self.detours.append(d)
            self.rest.take(m.lines)
        return later

    def _home(self, a: XY, b: Optional[XY]) -> float:
        """s of the way from a to b (home), 0 without b."""
        if b is None:
            return 0.0
        way = self.kit.transit.path(a, b) or [a, b]
        return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(way, way[1:])) / HOME_SPEED

    def _onto_chain(self, tail: Window, m: Move, home: Optional[XY], gone: float, start: Optional[XY]):
        """m on at the chain's end: (the way there, m as a Detour with what it costs, the way home from its end counted
        in place of the one before (gone), the new tail, the way home from there), else why not."""
        first = not tail[0]
        if first and not self.kit.turns_at(numpy.array([[m.first.x, m.first.y]]))[0]:
            return ACCESS  # (a path of its own starts where the mower turns whatever way it gets there)
        way = [] if first else self._link(tail[0][-2:], (m.first.x, m.first.y), m.first.yaw, True)
        if way is None:
            return ACCESS
        new = _joined(tail, _plain(way), _plain(m.poses))
        back = self._home((m.last.x, m.last.y), home)
        there = self._home((m.first.x, m.first.y), start) if first else 0.0
        trip = back - gone + there
        cost = _minus(_counted(self.kit.fp, new[0], drive_time(way)['s'] if way else 0.0),
                      _counted(self.kit.fp, tail[0])) + Cost(time_s=trip, dock_s=trip)
        d = Detour(-1, [], [m], cost, self.rest.gain(m.lines))
        if not self._worth(d):
            return COST
        if not self._fits(tail, _joined(_plain(way), _plain(m.poses))[0], ([], [])):
            return ACCESS
        return way, d, new, back

    def chained(self, moves: List[Move], why: Dict[int, str], start: Optional[XY] = None,
                home: Optional[XY] = None) -> None:
        """The moves one after the other from the path's end (start: as a path of its own from there), the nearest
        next where its way there fits and it's worth it, the way home (to home) from where it ends counted."""
        lo = int(self.section[-1][0]) if start is None else 0
        tail = _tail((self.P[lo:], self.on_turn[lo:]), CHECK_M, WINDOW) if start is None else ([], [])
        here = (self.P[-1].x, self.P[-1].y) if start is None else start
        chain, gone = Detour(len(self.P) - 1, [], []), self._home(here, home)
        pending = list(moves)
        while pending:
            m = min(pending, key=lambda m: (math.hypot(m.first.x - here[0], m.first.y - here[1]), m.piece))
            pending.remove(m)
            if self.rest.gain(m.lines) < MOVE_MIN:
                continue
            got = self._onto_chain(tail, m, home, gone, start)
            if isinstance(got, str):
                why[m.piece] = got
                self._fail('chain')
                continue
            way, d, new, gone = got
            chain.parts += ([(TRANSIT, way)] if way else []) + [('', m.poses)]
            chain.moves.append(m)
            chain.cost, chain.gain = chain.cost + d.cost, chain.gain + d.gain
            self.rest.take(m.lines)
            tail = _tail(new, CHECK_M, WINDOW)
            here = (m.last.x, m.last.y)
        self.chain = chain if chain.moves else None


def _tail(w: Window, metres: float, n: int) -> Window:
    """The last poses of w: n at least, metres of path at least."""
    poses, flags = w
    k, run = len(poses) - 1, 0.0
    while k > 0 and (len(poses) - k < n or run < metres):
        run += math.hypot(poses[k].x - poses[k - 1].x, poses[k].y - poses[k - 1].y)
        k -= 1
    return poses[k:], flags[k:]


def _outline(segments: List[SegmentInfo]) -> List[bool]:
    """For each segment whether OpenMower's path it's in is an outline one (export.plan_pieces: a drive goes with what
    comes after it): a move put in there of that kind keeps the path in one piece."""
    out, following, section = [], False, None
    for seg in reversed(segments):
        if seg.section != section:
            section, following = seg.section, False
        if seg.kind != TRANSIT:
            following = seg.kind == PERIMETER
        out.append(following)
    return out[::-1]


def _spliced(poses: List[Pose2D], segments: List[SegmentInfo], placer: Placer, maneuver: str,
             stroke: Optional[str] = None) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    """The path with the detours put in and the chain at its end: drives TRANSIT, moves of kind stroke, else of the
    kind OpenMower's path they're put in has (_outline)."""
    at = {d.at: d for d in placer.detours}
    segs: List[PathSegment] = []
    outline = _outline(segments)

    def put(d: Detour, section: int, loop: bool) -> None:
        kind = stroke or (PERIMETER if loop else LANE)
        segs.extend(PathSegment(k or kind, p, False, maneuver, section, maneuver) for k, p in d.parts)

    for info, loop in zip(segments, outline):
        cur = info.start
        for i in sorted(k for k in at if info.start <= k <= info.end):
            segs.append(PathSegment(info.kind, poses[cur:i + 1], info.reverse, info.maneuver, info.section,
                                    info.source))
            put(at.pop(i), info.section, loop)
            cur = i
        segs.append(PathSegment(info.kind, poses[cur:info.end + 1], info.reverse, info.maneuver, info.section,
                                info.source))
    if placer.chain is not None:
        put(placer.chain, segments[-1].section if segments else 0, bool(segments) and segments[-1].kind == PERIMETER)
    return _assemble(segs)


def _moves(kit: Kit, rest: Rest) -> Tuple[List[Move], Dict[int, str], int]:
    """The moves over the pieces of rest (the biggest PIECES), why pieces stay, the strokes built."""
    found = dataclasses.replace(rest)  # (taken from as moves are found, rest as they're placed)
    why: Dict[int, str] = {}
    moves: List[Move] = []
    tries = [STROKES]
    for k in range(len(rest.pieces)):
        if k >= PIECES or tries[0] <= 0:
            why[k] = BUDGET
            continue
        got, no = moves_for(kit, found, k, tries)
        moves += got
        if no:
            why[k] = no
    return moves, why, STROKES - tries[0]


def _stays(rest: Rest, why: Dict[int, str]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for k in range(len(rest.pieces)):
        a = rest.area(k)
        if a > 0:
            out[why.get(k) or SMALL] = round(out.get(why.get(k) or SMALL, 0.0) + a, 3)
    return out


def _reachable(cfg: PlannerConfig, left, drive_space):
    """Of left what the blade gets to at all, its centre in drive_space."""
    reach = math.hypot(cfg.blade_ahead, cfg.blade_offset) + 0.5 * cfg.mower_width + REACH_SLACK
    return left.intersection(drive_space.buffer(reach, 4))


def _left(geom, least: float) -> float:
    return sum(p.area for p in polygons(geom) if p.area >= least)


def complete(cfg: PlannerConfig, fp: Footprint, transit: TransitPlanner, drive_space, transit_space, lawn,
             poses: List[Pose2D], segments: List[SegmentInfo], field=None,
             home: Optional[XY] = None) -> Tuple[List[Pose2D], List[SegmentInfo], Dict[str, Any]]:
    """mop_up: what the path leaves of lawn mowed by detours and a chain at its end (the centre in drive_space mowing,
    transit_space too on drives; headland_turns: field, the lanes' field the drives don't turn in; home: where the
    mower goes after). (poses, segments, stats)."""
    kit = Kit.of(cfg, fp, transit, drive_space, transit_space, DRIVES, field)
    starts = [s.start for k, s in enumerate(segments) if k == 0 or s.section != segments[k - 1].section]
    ends = starts[1:] + [len(poses)]
    left = lawn.difference(blade(cfg, *[poses[a:b] for a, b in zip(starts, ends) if b - a > 1], whole=True))
    rest = Rest.of(_reachable(cfg, left, drive_space), LEAST, kit.half)
    moves, why, built = _moves(kit, rest)
    placer = Placer(kit, poses, segments, rest)
    placer.chained(placer.place(moves), why, home=home)
    out, infos = _spliced(poses, segments, placer, 'mop_up') if placer.detours or placer.chain else (poses, segments)
    done = [m for d in placer.detours + ([placer.chain] if placer.chain else []) for m in d.moves]
    total = _left(left, LEAST)
    far = total - sum(p.area for p in rest.pieces)
    stays = dict(_stays(rest, why), **({'reach': round(far, 3)} if far > 1e-3 else {}))
    return out, infos, {
        'pieces': len({m.piece for m in done}), 'passes': len(done),
        'area': round(sum(d.gain for d in placer.detours) + (placer.chain.gain if placer.chain else 0.0), 3),
        'left_m2': round(total, 3), 'stays_m2': stays, 'moves': len(moves), 'detours': len(placer.detours),
        'at_end': len(placer.chain.moves) if placer.chain else 0, 'strokes': built, 'drives': DRIVES - kit.drives[0],
        'failed': dict(sorted(placer.failed.items()))}


def leftovers(cfg: PlannerConfig, fp: Footprint, transit: TransitPlanner, space, mowed, start: XY,
              home: Optional[XY] = None) -> Tuple[List[Pose2D], List[SegmentInfo], Dict[str, Any]]:
    """What a mow left of space.free (mowed: where the blade went) as a path of its own: the moves one after the
    other, the nearest start first, the way there and home (to home) counted. (poses, segments, stats {left_m2, pieces,
    passes, area})."""
    kit = Kit.of(cfg, fp, transit, space.drive_space, space.transit_space, DRIVES)
    left = space.free.difference(mowed)
    rest = Rest.of(_reachable(cfg, left, space.drive_space), LEAST_MOWED, kit.half)
    moves, why, _ = _moves(kit, rest)
    placer = Placer(kit, [], [], rest)
    placer.chained(moves, why, start, home)
    poses, infos = _spliced([], [], placer, 'leftover', LANE) if placer.chain else ([], [])
    chain = placer.chain.moves if placer.chain else []
    return poses, infos, {'left_m2': round(_left(left, LEAST_MOWED), 3), 'pieces': len({m.piece for m in chain}),
                          'passes': len(chain), 'area': round(placer.chain.gain if placer.chain else 0.0, 3)}
