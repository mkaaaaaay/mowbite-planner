"""Moves over what a path leaves unmowed: straight in, a stroke over the piece and out, forwards, turning on the spot
where the body fits there (the completion stage's moves, see complete.py)."""

import dataclasses
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy
import shapely

from .config import XY, PlannerConfig
from .drive import Drive
from .footprint import SPIN, Footprint
from .geometry import QUAD_SEGS, polygons
from .maneuvers import join, same_position, sample_straight
from .mathutil import Pose2D, angle_diff, normalize_angle
from .transit import TransitPlanner

# m, what's left as points this far apart, further where there'd be more than MAX_POINTS
GRID = 0.02
MAX_POINTS = 150000
CHUNK = 1000000  # lines x points worked out at a time at most (memory)
# a piece's moves: poses facing HEADINGS ways round TARGETS of its points (and along it), the blade's middle on a point
# or ACROSS of half its width beside it, where the body fits; the SEEDS mowing the most on SEED_LEN m run on as far as
# the body fits (REACH m at most), the CHECKS of those mowing the most built first; ROUNDS moves a piece at most
HEADINGS = 24
ACROSS = (0.0, 0.5, -0.5, 0.9, -0.9)
TARGETS = 20
SEEDS = 30
SEED_LEN = 0.6
SAMPLE = 400  # points of a piece the seeds are ranked by at most
BATCH = 64  # seeds whose body is checked at a time
CHECKS = 8
ROUNDS = 8
REACH = 3.0
# rad, the legs' headings against the stroke: easing on and off it first, then turning on the spot
TURNS = tuple(math.radians(a) for a in (10, -10, 25, -25, 45, -45, 90, -90, 135, -135, 170, -170))
SPOTS = (0.0, 0.5, 1.0)  # where along the stroke's part before (after) the piece a leg may turn onto (off) it
BACK = (2, 4, 7, 11)  # or this many poses into the piece (mowing less of it)
LEGS = 28  # legs tried to get on or off a stroke at most
MOVE_MIN = 0.005  # m2 a move mows of what's left at least
SLACK = 1e-3  # m, check_plan allows 2 mm

# why a piece stays: no pose with the blade over it fits the body; no way forwards onto a stroke over it and off again
# (no room to turn); a move fits but no way there from the path does (complete.py), or it isn't worth the way; the
# budget; nothing worth a move
BODY, TURN, ACCESS, COST, BUDGET, SMALL = 'body', 'turn', 'access', 'cost', 'budget', 'small'


@dataclasses.dataclass
class Kit:
    """What the stage works with: the body, where the centre may be, the drives, where the blade is."""
    cfg: PlannerConfig
    fp: Footprint
    transit: TransitPlanner  # drives, through the drivable areas too
    mow: object  # where the centre may be mowing, prepared
    roam: object  # and on a drive
    free_zone: object  # the transit space, prepared
    turn_zone: object  # where the body turns round whatever way it faces, spin_margin kept (prepared)
    drives: List[int]  # drives left to work out in this plan
    field: object = None  # headland_turns: the lanes' field (a hair smaller, prepared), drives turn out of it

    @classmethod
    def of(cls, cfg: PlannerConfig, fp: Footprint, transit: TransitPlanner, drive_space, transit_space,
           drives: int, field=None) -> 'Kit':
        zones = [drive_space.buffer(SLACK), shapely.union(transit_space, drive_space).buffer(SLACK),
                 transit_space.buffer(SLACK, 4), fp.turning.space.buffer(-fp.reach, QUAD_SEGS)]
        inner = field.buffer(-SLACK, 1) if field is not None and not field.is_empty else None
        for z in zones + ([inner] if inner is not None else []):
            shapely.prepare(z)
        return cls(cfg, fp, transit, *zones, [drives], inner)

    def turns_in_field(self, poses: Sequence[Pose2D]) -> bool:
        """Whether the heading changes from one of poses to the next in the field (headland_turns)."""
        if self.field is None or len(poses) < 2:
            return False
        xy = numpy.array([(p.x, p.y) for p in poses[1:]])
        turn = numpy.array([angle_diff(a.yaw, b.yaw) > 1e-6 for a, b in zip(poses, poses[1:])])
        return bool((turn & shapely.contains_xy(self.field, xy[:, 0], xy[:, 1])).any())

    @property
    def step(self) -> float:
        return self.cfg.waypoint_spacing or 0.1

    @property
    def half(self) -> float:
        return 0.5 * self.cfg.mower_width

    def blade(self, x: float, y: float, yaw: float) -> XY:
        c, s, ahead, off = math.cos(yaw), math.sin(yaw), self.cfg.blade_ahead, self.cfg.blade_offset
        return x + ahead * c - off * s, y + ahead * s + off * c

    def centre(self, bx: float, by: float, yaw: float) -> XY:
        """The mower's centre with the blade's at (bx, by)."""
        c, s, ahead, off = math.cos(yaw), math.sin(yaw), self.cfg.blade_ahead, self.cfg.blade_offset
        return bx - ahead * c + off * s, by - ahead * s - off * c

    def lines(self, poses: Sequence[Pose2D]) -> numpy.ndarray:
        """(n, 4) the blade's middle from pose to pose (turning on the spot nothing)."""
        if len(poses) < 2:
            return numpy.zeros((0, 4))
        b = numpy.array([self.blade(p.x, p.y, p.yaw) for p in poses])
        return numpy.concatenate([b[:-1], b[1:]], 1)

    def free_at(self, xy: numpy.ndarray) -> numpy.ndarray:
        return shapely.intersects_xy(self.free_zone, xy[:, 0], xy[:, 1])

    def turns_at(self, xy: numpy.ndarray) -> numpy.ndarray:
        """Whether the mower turns round on the spot at each of xy (its turns are checked where they're driven)."""
        return shapely.intersects_xy(self.turn_zone, xy[:, 0], xy[:, 1])

    def drive(self, a: Pose2D, b: XY, yaw: Optional[float]) -> Optional[List[Pose2D]]:
        """drive.py's way from a to b, turning on the spot at a first where it leaves another way than a faces and at b
        to yaw (None: any) where it arrives another way; None where there's none or no drive is left in the budget."""
        if self.drives[0] <= 0:
            return None
        self.drives[0] -= 1
        way = Drive(self.transit, self.fp, self.cfg).way((a.x, a.y), a.yaw, b, yaw)
        if not way or not same_position(way[0], a):
            return None
        if len(way) > 1 and not same_position(way[0], way[1]):
            h = math.atan2(way[1].y - way[0].y, way[1].x - way[0].x)
            if angle_diff(h, way[0].yaw) > 1e-6:
                way = [way[0], Pose2D(way[0].x, way[0].y, h)] + way[1:]
        if yaw is not None and angle_diff(way[-1].yaw, yaw) > 1e-6:
            way = way + [Pose2D(way[-1].x, way[-1].y, yaw)]
        return way

    def line(self, a: Pose2D, b: XY, yaw: Optional[float]) -> List[Pose2D]:
        """Straight from a to b, turning on the spot at a and to yaw at b."""
        if math.hypot(b[0] - a.x, b[1] - a.y) < 1e-6:
            return [Pose2D(b[0], b[1], a.yaw if yaw is None else yaw)]
        way = sample_straight(a.x, a.y, b[0], b[1], math.atan2(b[1] - a.y, b[0] - a.x), self.step)
        return way if yaw is None else way + [Pose2D(b[0], b[1], yaw)]

    def inside(self, poses: Sequence[Pose2D], drive: bool = False) -> numpy.ndarray:
        """Whether the centre of each pose lies where it may, mowing or on a drive."""
        xy = numpy.array([(p.x, p.y) for p in poses]).reshape(-1, 2)
        return shapely.contains_xy(self.roam if drive else self.mow, xy[:, 0], xy[:, 1])

    def allowed(self, poses: Sequence[Pose2D]) -> bool:
        return bool(self.inside(poses).all()) and self.fp.poses_fit(poses)

    def fit_along(self, poses: Sequence[Pose2D]) -> numpy.ndarray:
        """For poses not turning: whether the centre may be there mowing and the body fits."""
        ok = self.inside(poses)
        if ok.any():
            ok[ok] = numpy.array(self.fp.fit([p for p, o in zip(poses, ok) if o]), bool)
        return ok


@dataclasses.dataclass
class Rest:
    """What's left as points on a grid: a way's gain is the points under its blade."""
    pieces: List  # polygons, biggest first
    pts: numpy.ndarray  # (n, 2) the points still left
    piece: numpy.ndarray  # (n,) the piece each lies in
    cell: float  # m2 a point stands for
    half: float  # half the blade's width

    @classmethod
    def of(cls, left, least: float, half: float) -> 'Rest':
        pieces = sorted((p for p in polygons(left) if p.area >= least), key=lambda p: -p.area)
        total = sum(p.area for p in pieces)
        g = max(GRID, math.sqrt(total / MAX_POINTS)) if total > 0 else GRID
        pts, owner = [numpy.zeros((0, 2))], [numpy.zeros(0, int)]
        for k, poly in enumerate(pieces):
            x0, y0, x1, y1 = poly.bounds
            gx, gy = (a.ravel() for a in numpy.meshgrid(numpy.arange(math.floor(x0 / g) * g + 0.5 * g, x1, g),
                                                        numpy.arange(math.floor(y0 / g) * g + 0.5 * g, y1, g)))
            inside = shapely.contains_xy(poly, gx, gy)
            pts.append(numpy.stack([gx[inside], gy[inside]], 1))
            owner.append(numpy.full(int(inside.sum()), k))
        return cls(pieces, numpy.concatenate(pts), numpy.concatenate(owner), g * g, half)

    def area(self, k: Optional[int] = None) -> float:
        return self.cell * (len(self.pts) if k is None else int((self.piece == k).sum()))

    def _near(self, lines: numpy.ndarray, q: numpy.ndarray):
        """For chunks of lines: (first line, (lines, points) whether each of q lies within the blade's reach)."""
        a, d = lines[:, :2], lines[:, 2:4] - lines[:, :2]
        n = numpy.maximum((d * d).sum(1), 1e-12)
        step = max(1, min(64, CHUNK // max(1, len(q))))
        for i in range(0, len(lines), step):
            aa, dd, nn = a[i:i + step], d[i:i + step], n[i:i + step]
            t = numpy.clip(((q[None, :, 0] - aa[:, None, 0]) * dd[:, None, 0] +
                            (q[None, :, 1] - aa[:, None, 1]) * dd[:, None, 1]) / nn[:, None], 0.0, 1.0)
            dx = q[None, :, 0] - aa[:, None, 0] - t * dd[:, None, 0]
            dy = q[None, :, 1] - aa[:, None, 1] - t * dd[:, None, 1]
            yield i, dx * dx + dy * dy <= self.half * self.half

    def covered(self, lines: numpy.ndarray) -> numpy.ndarray:
        """Which of the points left the blade covers on lines."""
        hit = numpy.zeros(len(self.pts), bool)
        if not len(lines) or not len(self.pts):
            return hit
        lo = numpy.minimum(lines[:, :2], lines[:, 2:4]).min(0) - self.half
        hi = numpy.maximum(lines[:, :2], lines[:, 2:4]).max(0) + self.half
        near = numpy.nonzero(((self.pts >= lo) & (self.pts <= hi)).all(1))[0]
        if len(near):
            got = numpy.zeros(len(near), bool)
            for _, m in self._near(lines, self.pts[near]):
                got |= m.any(0)
            hit[near] = got
        return hit

    def counts(self, lines: numpy.ndarray, pts: numpy.ndarray) -> numpy.ndarray:
        """For each line on its own: how many of pts the blade covers on it."""
        out = numpy.zeros(len(lines), int)
        for i, m in self._near(lines, pts):
            out[i:i + len(m)] = m.sum(1)
        return out

    def gain(self, lines: numpy.ndarray) -> float:
        return self.cell * int(self.covered(lines).sum())

    def take(self, lines: numpy.ndarray) -> Dict[int, float]:
        """Takes what the blade covers on lines out: the m2 of each piece it stood for."""
        hit = self.covered(lines)
        got = {int(k): self.cell * int(n) for k, n in zip(*numpy.unique(self.piece[hit], return_counts=True))}
        self.pts, self.piece = self.pts[~hit], self.piece[~hit]
        return got


@dataclasses.dataclass
class Move:
    poses: List[Pose2D]  # forwards, the body checked, turning on the spot where the heading jumps
    lines: numpy.ndarray  # the blade's way (Kit.lines)
    piece: int
    gain: float  # m2 of what was left it mowed when it was found

    @property
    def first(self) -> Pose2D:
        return self.poses[0]

    @property
    def last(self) -> Pose2D:
        return self.poses[-1]


def _ray(kit: Kit, at: XY, yaw: float, sign: float, n: int) -> List[Pose2D]:
    c, s = math.cos(yaw), math.sin(yaw)
    return [Pose2D(at[0] + sign * k * kit.step * c, at[1] + sign * k * kit.step * s, yaw) for k in range(n)]


def _runs(kit: Kit, rays: Sequence[List[Pose2D]]) -> List[int]:
    """How many poses of each ray fit the body in a row from its first (all of them at once)."""
    fit = kit.fit_along([p for r in rays for p in r])
    out, i = [], 0
    for r in rays:
        f = fit[i:i + len(r)]
        out.append(int(numpy.argmin(f)) if not f.all() else len(r))
        i += len(r)
    return out


def _end(kit: Kit, ray: Sequence[Pose2D], good: int) -> Tuple[int, bool]:
    """Where a leg along the ray ends: the first pose in the transit space where the mower turns any way (True), else
    the first in the transit space (False), (-1, False) if none of the first good is in it."""
    if good < 2:
        return -1, False
    xy = numpy.array([(p.x, p.y) for p in ray[1:good]])
    free = kit.free_at(xy)
    if not free.any():
        return -1, False
    first = int(numpy.argmax(free))
    run = free[first:]
    stop = first + (len(run) if run.all() else int(numpy.argmin(run)))
    turn = kit.turns_at(xy[first:stop])
    return (first + 1 + int(numpy.argmax(turn)), True) if turn.any() else (first + 1, False)


def _legs(kit: Kit, at: Pose2D, yaws: Sequence[float], sign: float) -> List[Tuple[Optional[List[Pose2D]], bool]]:
    """For each yaw: straight from the transit space to at (sign -1) or from at into it (1) in driving order, and
    whether the mower turns any way at its far end. The bodies of all of them checked at once."""
    rays = [_ray(kit, (at.x, at.y), h, sign, int(REACH / kit.step) + 1) for h in yaws]
    out = []
    for ray, good in zip(rays, _runs(kit, rays)):
        k, free = _end(kit, ray, good)
        out.append(((ray[k::-1] if sign < 0 else ray[:k + 1]), free) if k >= 0 else (None, False))
    return out


def _spots(k0: int, k1: int, end: int) -> List[int]:
    """Stroke poses to turn at, from k0 (where the piece starts or ends) out to the stroke's end, then towards k1 (the
    piece's other end: mowing less of it)."""
    out = [int(round(k0 + f * (end - k0))) for f in SPOTS]
    step = 1 if k1 > k0 else -1
    out += [k0 + step * d for d in BACK if 0 < d <= abs(k1 - k0)]
    return list(dict.fromkeys(out))


def _onto(kit: Kit, chord: List[Pose2D], k0: int, k1: int, sign: float) -> Tuple[Optional[List[Pose2D]], int]:
    """Getting onto the stroke at or before pose k0 (sign -1) or off it at or after k0 (1), k1 the piece's other end:
    (the leg, the stroke pose it turns at); an empty leg where the stroke's own line reaches the transit space. (None,
    -1) where nothing fits."""
    ray = chord[k0::-1] if sign < 0 else chord[k0:]
    k, free = _end(kit, [ray[0]] + list(ray), len(ray) + 1)
    if free:
        return [], k0 + int(sign) * (k - 1)
    # (getting off where the mower can't turn needs the path right ahead: only the way in may)
    fallback = ([], k0 + int(sign) * (k - 1)) if k >= 0 and sign < 0 else (None, -1)
    yaw, legs = chord[0].yaw, LEGS
    for s in _spots(k0, k1, 0 if sign < 0 else len(chord) - 1):
        if legs <= 0:
            break
        at, turns = chord[s], TURNS[:legs]
        legs -= len(turns)
        hs = [normalize_angle(yaw - t) if sign < 0 else normalize_angle(yaw + t) for t in turns]
        got = _legs(kit, at, hs, sign)
        # (the shortest leg ending where the mower turns any way that the turn onto it fits, at the first place that
        # has one; else on the way in the shortest of the others)
        for unfree, _, i in sorted((not free, len(leg), i) for i, (leg, free) in enumerate(got)
                                   if leg is not None and (free or sign < 0)):
            if unfree and fallback[0] is not None:
                break
            if abs(turns[i]) > SPIN and not (kit.fp.fits_turn(at.x, at.y, hs[i], yaw) if sign < 0 else
                                             kit.fp.fits_turn(at.x, at.y, yaw, hs[i])):
                continue
            if not unfree:
                return got[i][0], s
            fallback = (got[i][0], s)
    return fallback


def _seeds(kit: Kit, rest: Rest, pts: numpy.ndarray) -> List[Pose2D]:
    """Poses with the blade over points of pts where the body fits, the most mowing first (one a line)."""
    targets = pts[::max(1, len(pts) // TARGETS)]
    cov = numpy.cov(pts.T) if len(pts) > 2 else numpy.zeros((2, 2))
    axis = 0.5 * math.atan2(2.0 * cov[0, 1], cov[0, 0] - cov[1, 1])
    yaws = [2.0 * math.pi * k / HEADINGS for k in range(HEADINGS)] + [axis, axis + math.pi]
    h, t, a = (g.ravel() for g in numpy.meshgrid(yaws, numpy.arange(len(targets)), ACROSS, indexing='ij'))
    c, s, d = numpy.cos(h), numpy.sin(h), 0.5 * SEED_LEN
    bx, by = targets[t, 0] - a * kit.half * s, targets[t, 1] + a * kit.half * c
    gain = rest.counts(numpy.stack([bx - d * c, by - d * s, bx + d * c, by + d * s], 1),
                       pts[::max(1, len(pts) // SAMPLE)])
    ahead, off = kit.cfg.blade_ahead, kit.cfg.blade_offset
    cx, cy = bx - ahead * c + off * s, by - ahead * s - off * c
    order = [int(i) for i in numpy.argsort(-gain, kind='stable') if gain[i] > 0]
    out: List[Pose2D] = []
    seen = set()
    for j in range(0, len(order), BATCH):
        poses = [Pose2D(float(cx[i]), float(cy[i]), normalize_angle(float(h[i]))) for i in order[j:j + BATCH]]
        for p, fit in zip(poses, kit.fit_along(poses)):
            key = (round(p.yaw, 3), round(-math.sin(p.yaw) * p.x + math.cos(p.yaw) * p.y, 2))
            if fit and key not in seen:
                seen.add(key)
                out.append(p)
                if len(out) >= SEEDS:
                    return out
    return out


def _chords(kit: Kit, seeds: List[Pose2D]) -> List[List[Pose2D]]:
    """From each seed on both ways as far as the body fits."""
    n = int(REACH / kit.step) + 1
    on = [_ray(kit, (p.x, p.y), p.yaw, 1.0, n) for p in seeds]
    back = [_ray(kit, (p.x, p.y), p.yaw, -1.0, n) for p in seeds]
    runs = _runs(kit, on + back)
    return [b[:rb][::-1] + f[1:rf] for f, b, rf, rb in zip(on, back, runs[:len(seeds)], runs[len(seeds):])]


def _build(kit: Kit, chord: List[Pose2D], k0: int, k1: int) -> Optional[List[Pose2D]]:
    """The move along the chord mowing its poses k0..k1: in, the stroke, out."""
    leg_in, s0 = _onto(kit, chord, k0, k1, -1.0)
    if leg_in is None:
        return None
    leg_out, s1 = _onto(kit, chord, k1, max(k0, s0), 1.0)
    if leg_out is None or s1 <= s0:
        return None
    way = join(leg_in, chord[s0:s1 + 1], leg_out)
    return way if kit.allowed(way) else None


def _round(kit: Kit, rest: Rest, k: int, pts: numpy.ndarray, tries: List[int]) -> Tuple[Optional[List[Pose2D]], str]:
    """One move over piece k (of pts, what's left of it) mowing the most, and else why there's none."""
    seeds = _seeds(kit, rest, pts)
    if not seeds:
        return None, BODY
    chords = _chords(kit, seeds)
    ends = numpy.array([kit.blade(c[0].x, c[0].y, c[0].yaw) + kit.blade(c[-1].x, c[-1].y, c[-1].yaw)
                        for c in chords])
    gain = rest.cell * rest.counts(ends, pts)  # (a chord is straight: its blade's way one line)
    why = SMALL
    for i in sorted(range(len(chords)), key=lambda i: (-gain[i], i))[:CHECKS]:
        if gain[i] < MOVE_MIN:
            break
        if tries[0] <= 0:
            return None, BUDGET
        tries[0] -= 1
        b = numpy.array([kit.blade(p.x, p.y, p.yaw) for p in chords[i]])
        q = pts[((pts >= b.min(0) - kit.half) & (pts <= b.max(0) + kit.half)).all(1)]
        hit = numpy.nonzero((((b[:, None, :] - q[None, :, :]) ** 2).sum(-1) <= kit.half ** 2).any(1))[0]
        if not len(hit):
            continue
        way = _build(kit, chords[i], int(hit[0]), int(hit[-1]))
        if way is not None and rest.gain(kit.lines(way)) >= MOVE_MIN:
            return way, ''
        why = TURN
    return None, why


def moves_for(kit: Kit, rest: Rest, k: int, tries: List[int]) -> Tuple[List[Move], str]:
    """The moves over piece k (what they mow taken out of rest) and why the rest of it stays. tries[0]: strokes left
    to build in this plan."""
    out: List[Move] = []
    for _ in range(ROUNDS):
        pts = rest.pts[rest.piece == k]
        if len(pts) * rest.cell < MOVE_MIN:
            return out, ''
        way, why = _round(kit, rest, k, pts, tries)
        if way is None:
            return out, why
        lines = kit.lines(way)
        out.append(Move(poses=way, lines=lines, piece=k, gain=sum(rest.take(lines).values())))
    return out, BUDGET
