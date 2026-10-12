"""The path made to fit the body (_fit_body), with headland_turns kept out of the lanes' field (_field_turns), its
corners rounded (_round_corners), mop_up, and a pose before turning on the spot (_turn_at)."""

import bisect
import dataclasses
import hashlib
import math
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points
from shapely.prepared import prep

from .config import (_FITTED, _KEPT_LOCK, EARLIER_LEFT, LANE, PERIMETER, SMOOTH_RADII, TRANSIT, TURN, XY, Lane,
                     PathSegment, SegmentInfo, _keep)
from .footprint import SPIN
from .geometry import QUAD_SEGS, polygons, subtract, union
from .maneuvers import CORNER_ANGLE, dubins, join, path_length, same_position, sample_arc, sample_straight
from .mathutil import Pose2D, angle_diff, normalize_angle
from .route import _Route
from .transit import TransitPlanner


# m, with the body checked: where the path still turns on the spot a pose this much before the point, facing the way
# it came (where the way in is longer than twice that). OpenMower's FTC turns over the way from the pose before to the
# next: from 10 cm before it it turns some 5 cm early and drifts, from this close about at the point (a simulation of
# it, the mower lagging 0.1 s)
TURN_AHEAD = 0.025
# m of path: where the body still sticks out, the path from up to FIT_REACH before to FIT_REACH after it may go another
# way (_fit_body), leaving it and getting back on FIT_STEP apart at least, FIT_PLACE ways tried at most there. The
# further only where none closer fits
FIT_REACH = (3.0, 8.0)
FIT_STEP = (0.2, 0.5)
FIT_PLACE = (30, 60)
# m from where the headings meet towards the closest point it can turn round at whatever way it faces: where
# _fit_body's ways may turn on the spot, the first it can turn at (the last: that point)
FIT_VIA = (0.0, 0.02, 0.04, 0.06, 0.1, 0.15, 0.25, 0.5, 1.0)
# the further ways get off the path and back on it easing off and onto its line: t m along it FIT_EASE_OFF m *
# (e^(t / l) - 1) off it at most, l FIT_EASE times how far the body reaches behind the centre getting off, ahead of it
# getting back on (the rear, the front swinging out FIT_EASE_OFF m at most)
FIT_EASE = 1.5
FIT_EASE_OFF = 0.01
# what a metre driven counts against a metre of loops and lanes left out, picking one of _fit_body's ways
FIT_DRIVE = 0.25
# shares of turn_radius: the arcs of Dubins' ways _fit_body tries (arc, straight, arc or three arcs, turning half round
# at most all together, the shortest FIT_DUBINS of each), before the ways turning on the spot at a corner
FIT_ARCS = (1.0, 0.75, 0.5)
FIT_DUBINS = 2
# mop_up: pieces left unmowed of at least MOP_MIN m2, the biggest MOP_MAX, a straight pass of MOP_PASS m over each with
# the blade's middle on it (one of MOP_HEADINGS directions round it), where that mows MOP_GAIN of MOP_MIN at least
MOP_MIN = 0.02
MOP_MAX = 20
MOP_PASS = 0.4
MOP_HEADINGS = 16
MOP_GAIN = 0.5
# ways _fit_body tries at most for a plan, past that the path jumps over a place (a count, not a time: the same plan
# every time)
FIT_TRIES = 12000
# m, where nothing fits that soon after a section begins it begins later. FIT_LATER: where nothing else does either
# (turning round, driving on), getting to a section's start is up to whoever drives the path anyway
FIT_START = 1.0
FIT_LATER = 8.0
# where none of those fits the path turns round before the place, at the first point of FIT_TURNS a way fits through:
# the poses there and round it (FIT_STEP[0] apart, FIT_AROUND m before and after at most, the closest first) moved
# FIT_IN m to either side, as little as the body fits facing that pose's way, turned half and all the way the path turns
# over the place. In from the latest pose before it fits from, out to the earliest after, straight or eased. Where none
# does, through turn_room like the further ways (simplified ROOM_SIMPLIFY m: there it turns whatever way it faces).
# Where driving on doesn't fit either (and the path wouldn't end before), moved further to the side than that (a smaller
# body fits closer to the edge than it turns round). FIT_ROUND ways and halves of them tried at most for each (for those
# further ones together)
FIT_AROUND = 1.5
FIT_IN = (0.03, 0.06, 0.1, 0.2, 0.3, 0.45)
FIT_TURNS = 6
FIT_ROUND = 600
ROOM_SIMPLIFY = 0.005
# headland_turns: a way of _fit_body that turns in the lanes' field counts as this many m of loops and lanes left out
FIT_FIELD = 2.0
# m, _fit_body finds where the body sticks out the most at a place looking along its edge this far apart first
# (Footprint.out_by), closer only where that may be the most
PLACE_STEP = 0.02
# _fit_body keeps where it got to every FIT_KEEP poses: a plan whose path starts the same (the loops, at another lane
# angle) goes on from the last of those up to which it reads the same poses
FIT_KEEP = 1000
# m, with headland_corners 'rounded' an arc rounding a drive's corner out of the lanes' field into it (where none fits
# out of it) reaches this far into it at most, else the corner stays
FIELD_ARC = 0.12


def _fit_body(route: '_Route', poses: List[Pose2D],
              infos: List[SegmentInfo]) -> Tuple[List[Pose2D], List[SegmentInfo], Dict[str, Any]]:
    """Where the body still sticks out (at a pose, or turning on the spot there), the path from a pose before it to one
    after it (FIT_REACH before and after at most) another way that fits: straight (turning on the spot at both ends),
    Dubins' ways (FIT_ARCS), or in to the first point from where the two headings meet on towards where it can turn
    round whatever way it faces (FIT_VIA) that it can turn at, and out again (straight, the further ways easing off the
    path's line and onto it, FIT_EASE). Of those the one that leaves the least of the loops and lanes out, a metre
    driven counting FIT_DRIVE of that (with headland_turns one into the lanes' field FIT_FIELD more), FIT_PLACE ways
    tried at most for a place (and the Dubins' ways) and FIT_TRIES for all. Where none fits it turns round before the
    place, or goes through turn_room (see FIT_AROUND), or drives on through the transit space to one of the next poses
    that fit, or turns round further to the side, and where none of that fits the path jumps from the pose before to the
    next one that fits (a section starts later where the body doesn't fit at its start, nothing fits FIT_START after it
    or nothing else FIT_LATER after it, the path ends before where it would jump last).
    Where the path starts later in its first section or ends before, a way from where the body turns whatever way it
    faces closest to its start, or to the one closest to its end. Returns the poses, the segments (the ways in place
    TRANSIT 'body', a jump within a section starts one of its own) and {'fixed': ways in place, 'left': places left out
    (not the path starting later in its first section or ending before, FIT_START m of loops and lanes at most),
    'places': [[x, y, m of loops and lanes left out]] of both, at the pose of the path as planned where the body sticks
    out the most there, 'poses': [[x, y, yaw, m, spin]] of those poses (how far it sticks out of the space that applies
    there, see Footprint.out_by, spin 1 where that's turning on the spot), 'skipped_m': those m added up, 'jumps': [[x,
    y]] of the places the path jumps over, 'unchecked': [[x, y]] where it starts later or ends before with no such way,
    'tries': ways tried}."""
    fp, step = route.footprint, route.step
    n = len(poses)
    section, mows = [0] * n, [False] * n  # (mows: on the way to it the blade mows a loop or a lane)
    for info in infos:
        section[info.start:info.end + 1] = [info.section] * (info.end + 1 - info.start)
        mows[info.start + 1:info.end + 1] = [info.kind in (PERIMETER, LANE)] * (info.end - info.start)
    starts = {i for i in range(n) if i == 0 or section[i] != section[i - 1]}
    on_turn = [False] * n  # (on the way to it: a turn between lanes, spin_margin all along it too)
    for info in infos:
        on_turn[info.start + 1:info.end + 1] = [info.kind == TURN] * (info.end - info.start)
    fit, turn_room = fp.fit(poses, on_turn), fp.turning.anywhere
    tries, most = [0], [0]  # (most: the tries this place may take up to)
    turns: Dict[Tuple[float, ...], bool] = {}  # fits_turn where and which way, rounded (ways share their corner)

    def hop(i: int) -> float:  # m from the pose before
        return math.hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y)

    def fits(prev: Optional[Pose2D], i: int) -> bool:
        p = poses[i]
        if prev is not None and angle_diff(p.yaw, prev.yaw) > SPIN:
            return fp.fits_turn(p.x, p.y, prev.yaw, p.yaw)
        return fit[i]

    def spent() -> bool:
        return tries[0] >= min(FIT_TRIES, most[0])

    def ok(before: List[Pose2D], way: List[Pose2D]) -> bool:
        if spent():
            return False
        tries[0] += 1
        # (where it leaves and gets back on first: a way that sticks out mostly does so there)
        return len(way) > 1 and all(fp.fits_pose(q.x, q.y, q.yaw) for q in (way[0], way[-2])) and \
            route.transit.is_free([(q.x, q.y) for q in way]) and fp.poses_fit(before + way)

    def straight(a: Pose2D, b: Pose2D) -> List[Pose2D]:
        return join(sample_straight(a.x, a.y, b.x, b.y, math.atan2(b.y - a.y, b.x - a.x), step), [b])

    def ways(a: Pose2D, b: Pose2D, eased: bool) -> Iterator[List[Pose2D]]:
        yield straight(a, b)
        # (round: no turning on the spot, the body needs no room round where it would. Tries of their own, the ways
        # after them get as many as before; with headland_turns not into the lanes' field)
        for share in FIT_ARCS:
            for m in dubins(a, b, share * route.cfg.turn_radius, step, turning=math.pi)[:FIT_DUBINS]:
                way = m.pieces[0].poses
                if route.field is None or route.clear([(q.x, q.y) for q in way]):
                    most[0] += 1
                    yield way
        # in to the first point that it can turn at from where the two headings meet (a corner) on towards where it
        # can turn round whatever way it faces, and out again: straight, or (eased) off a's line and onto b's no
        # faster than the rear and the front swing out
        ux, uy, vx, vy = math.cos(a.yaw), math.sin(a.yaw), math.cos(b.yaw), math.sin(b.yaw)
        wx, wy, det = b.x - a.x, b.y - a.y, ux * vy - uy * vx
        t, u = ((wx * vy - wy * vx) / det, (ux * wy - uy * wx) / det) if abs(det) > 1e-6 else (-1.0, -1.0)
        c = Point(a.x + t * ux, a.y + t * uy) if t > 0 and u > 0 else Point(a.x + 0.5 * wx, a.y + 0.5 * wy)
        far = c if turn_room.is_empty or turn_room.contains(c) else nearest_points(turn_room, c)[0]
        (cx, cy), (fx, fy) = c.coords[0], far.coords[0]
        span, chord = math.hypot(fx - cx, fy - cy), math.hypot(wx, wy)
        behind = Pose2D(b.x, b.y, b.yaw + math.pi)  # (getting back on b's line: getting off it the other way)
        vias = []  # (where it turns, which way in and out)
        for d in FIT_VIA:
            f = min(1.0, d / span) if span > 0 else 1.0
            x, y = cx + f * (fx - cx), cy + f * (fy - cy)
            if min(d, span) >= chord or min(math.hypot(x - a.x, y - a.y), math.hypot(b.x - x, b.y - y)) < 1e-3:
                break
            if not eased:
                vias.append((x, y, math.atan2(y - a.y, x - a.x), math.atan2(b.y - y, b.x - x)))
            else:
                # (the last points only, which way it turns there: all of them for the way it takes)
                leave, back = ease(a, x, y, fp.rear, 1), ease(behind, x, y, fp.front, 1)
                if leave and back:
                    vias.append((x, y, math.atan2(y - leave[0][1], x - leave[0][0]),
                                 math.atan2(back[0][1] - y, back[0][0] - x)))
            if f >= 1.0:
                break
        if not eased:
            # (where it leaves and gets back on first, all at once)
            ends = fp.fit([q for _, _, into, out_of in vias for q in (
                Pose2D(a.x, a.y, into), Pose2D(b.x - step * math.cos(out_of), b.y - step * math.sin(out_of), out_of))])
            vias = [via for k, via in enumerate(vias) if ends[2 * k] and ends[2 * k + 1]]
        for x, y, into, out_of in vias:
            if turns_at(x, y, into, out_of):
                if eased:
                    yield join(along(ease(a, x, y, fp.rear)), along(ease(behind, x, y, fp.front)[::-1])[:-1], [b])
                else:
                    yield join(straight(a, Pose2D(x, y, into)), straight(Pose2D(x, y, out_of), b))
                break

    def ease(p: Pose2D, x: float, y: float, end: float, last: int = 0) -> Optional[List[XY]]:
        # points from p to (x, y) (the last ones only), off p's line as FIT_EASE says (end: m the body reaches out
        # that way), then beside it. None where (x, y) isn't ahead
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        ahead, off = (x - p.x) * c + (y - p.y) * s, (y - p.y) * c - (x - p.x) * s
        if ahead < step:
            return None
        k, pts = int(math.ceil(ahead / step)), []
        for i in range(k - last if last else 0, k):
            t = ahead * i / k
            o = math.copysign(min(abs(off), FIT_EASE_OFF * math.expm1(min(30.0, t / (FIT_EASE * end)))), off)
            pts.append((p.x + t * c - o * s, p.y + t * s + o * c))
        return pts + [(x, y)]

    def along(pts: List[XY]) -> List[Pose2D]:  # step apart at most, facing where they go
        return join(*(sample_straight(p[0], p[1], q[0], q[1], math.atan2(q[1] - p[1], q[0] - p[0]), step)
                      for p, q in zip(pts, pts[1:])))

    def turns_at(x: float, y: float, yaw0: float, yaw1: float) -> bool:
        key = (round(x, 2), round(y, 2), round(yaw0, 1), round(yaw1, 1))
        if key not in turns:
            turns[key] = fp.fits_turn(x, y, yaw0, yaw1)
        return turns[key]

    def legs(rs: List[int], x: float, y: float, into: bool) -> Iterator[Tuple[int, List[Pose2D]]]:
        # from out[r] to (x, y) (into), or from there to poses[r], for r of rs in turn: eased off the pose's line (onto
        # it) or straight, where they fit (where they leave and get there first, a few at once)
        for chunk in (rs[c:c + 8] for c in range(0, len(rs), 8)):
            found = []  # (r, the points in the order it drives them)
            for r in chunk:
                p = out[r] if into else poses[r]
                if math.hypot(x - p.x, y - p.y) >= 2.0 * step:
                    for pts in (ease(p if into else Pose2D(p.x, p.y, p.yaw + math.pi), x, y,
                                     fp.rear if into else fp.front), [(p.x, p.y), (x, y)]):
                        if pts:
                            found.append((r, pts if into else pts[::-1]))
            there = fp.fit([Pose2D(w[0], w[1], math.atan2(v[1] - u[1], v[0] - u[0])) for _, go in found
                            for u, v, w in ((go[0], go[1], go[0]), (go[-2], go[-1], go[-1]))])
            for k, (r, go) in enumerate(found):
                if there[2 * k] and there[2 * k + 1] and not spent():
                    tries[0] += 1
                    way = along(go) if into else join(along(go), [poses[r]])
                    if route.transit.is_free(go) and fp.poses_fit(([out[r - 1]] if into and r > first else []) + way):
                        yield r, way

    def turn_round(i: int, wider: int = 0) -> Optional[Tuple[float, int, int, List[Pose2D]]]:
        # (see FIT_AROUND) the place: from i to where the poses fit again for FIT_STEP[0] m, the turn the path makes
        # over it. wider: of the ways to the side the body fits at that many left out
        end, run, d = None, 0.0, 0.0
        for j in range(i + 1, n):
            d += hop(j)
            if j in starts or d > FIT_REACH[-1]:
                return None
            if not fit[j]:
                end = None
            elif end is None:
                end, run = j, 0.0
            else:
                run += hop(j)
                if run >= FIT_STEP[0]:
                    break
        else:
            return None
        turn = normalize_angle(poses[i].yaw - out[-1].yaw) + sum(
            normalize_angle(poses[j].yaw - poses[j - 1].yaw) for j in range(i + 1, end + 1))
        near, last, d = [], -FIT_STEP[0], 0.0  # (m from the place, pose)
        for q in range(i, end):
            d += hop(q) if q > i else 0.0
            if d >= last + FIT_STEP[0]:
                near.append((0.0, q))
                last = d
        for q, ahead in ((i - 1, -1), (end, 1)):
            d, last = 0.0, 0.0
            while 0 < q < n - 1 and d < FIT_AROUND:
                if d >= last:
                    near.append((d, q))
                    last = d + FIT_STEP[0]
                q += ahead
                d += hop(max(q, q - ahead))
        seen[0] = max([seen[0], end] + [q for _, q in near])
        backs, ons = ends(i, FIT_REACH[0], FIT_REACH[-1], 0.0)
        ons = [o for o in ons if o[2] >= end]
        side, points = (1.0 if turn >= 0 else -1.0), 0
        for _, q in sorted(near):
            if points == FIT_TURNS:
                break
            p = poses[q]
            c, s = math.cos(p.yaw), math.sin(p.yaw)
            moved = [(p.x - off * s, p.y + off * c) for off in (f * sign for f in FIT_IN for sign in (side, -side))]
            there = fp.turning.fit([Pose2D(x, y, p.yaw + turn * f) for x, y in moved for f in (0.0, 0.5, 1.0)])
            x, y = ([xy for k, xy in enumerate(moved) if all(there[3 * k:3 * k + 3])][wider:] + [(None, None)])[0]
            if x is None or not route.transit.is_free([(x, y)]):
                continue
            points += 1
            # in from the latest poses it fits from (3 at most), out to the earliest after the place it turns into from
            # one of those
            ins = []
            for k, into in legs([k for _, _, k in backs], x, y, True):
                ins.append((next(m for m, _, r in backs if r == k), k, into))
                if len(ins) == 3:
                    break
            for j, onto in legs([j for _, _, j in ons], x, y, False) if ins else ():
                ma = next(m for m, _, r in ons if r == j)
                for mb, k, into in ins:
                    if (angle_diff(into[-1].yaw, onto[0].yaw) <= SPIN or turns_at(x, y, into[-1].yaw, onto[0].yaw)) \
                            and ok([out[k - 1]] if k > first else [], join(into, onto)):
                        return mb + ma, k, j, join(into, onto)
        return None

    def picked(rs: range, at: List[Pose2D], fits_at: Optional[List[bool]], most: int,
               far: float = math.inf) -> List[int]:
        # of the poses at[r] in the transit space (that fit) the first, then FIT_STEP[-1] m of path on, twice as far
        # each time (most of them, up to far m of path)
        got, d, nxt, last = [], 0.0, 0.0, 0
        for r in rs:
            d += math.hypot(at[r].x - at[last].x, at[r].y - at[last].y) if got else 0.0
            last = r
            if d > far:
                break
            if (fits_at is None or fits_at[r]) and d >= nxt and route.transit.is_free([(at[r].x, at[r].y)]):
                got.append(r)
                nxt = max(FIT_STEP[-1], 2.0 * d)
                if len(got) == most:
                    break
        return got

    def drive_on(i: int) -> Optional[Tuple[float, int, int, List[Pose2D]]]:
        # a drive (_Route.drive_to) through the transit space from one of the latest poses in it (FIT_TURNS of them,
        # picked()) to one of the next ones in it that fit (up to the section's end)
        end = next((j for j in range(i + 1, n) if j in starts), n)
        seen[0] = max(seen[0], end - 1)
        backs = picked(range(len(out) - 1, first - 1, -1), out, None, FIT_TURNS)
        for j in picked(range(i + 1, end), poses, fit, n):
            for k in backs:
                way = route.drive_to((poses[j].x, poses[j].y), poses[j].yaw, None, route.transit,
                                     ((out[k].x, out[k].y), out[k].yaw))
                if way and ok([out[k - 1]] if k > first else [], join(way, [poses[j]])):
                    return 0.0, k, j, join(way, [poses[j]])
        return None

    def through_room(k: int, j: int) -> Iterator[List[Pose2D]]:
        # in to where it turns whatever way it faces (the closest point of turn_room to FIT_STEP[-1] ahead of out[k]),
        # through there and out from the closest to FIT_STEP[-1] behind poses[j]
        if room[0] is None:
            room[0] = TransitPlanner(turn_room.simplify(ROOM_SIMPLIFY), 0.0)
        space, both = room[0].space, []
        for r, into in ((k, True), (j, False)):
            if (r, into) not in rooms:
                p, ahead = (out[r], FIT_STEP[-1]) if into else (poses[r], -FIT_STEP[-1])
                at = Point(p.x + ahead * math.cos(p.yaw), p.y + ahead * math.sin(p.yaw))
                at = at if space.contains(at) else nearest_points(space, at)[0]
                rooms[(r, into)] = next(((at.x, at.y, way) for _, way in legs([r], at.x, at.y, into)), None)
            both.append(rooms[(r, into)])
        mid = room[0].path(both[0][:2], both[1][:2]) if None not in both else None
        if mid is not None:
            yield join(both[0][2], along(mid), both[1][2])

    def ends(i: int, back: float, on: float, every: float) -> Tuple[List[Tuple[float, float, int]], ...]:
        # where to leave the path (back along what's planned already, `back` m at most) and where to get back on (`on`
        # m), `every` apart: (m of loops and lanes left out, m of path, index)
        backs, mown, d, k = [], 0.0, 0.0, len(out) - 1
        while k >= first and d <= back:
            if not backs or d >= backs[-1][1] + every - 1e-9:
                backs.append((mown, d, k))
            if k > first:
                d += math.hypot(out[k].x - out[k - 1].x, out[k].y - out[k - 1].y)
                mown += cut[k]
            k -= 1
        ons, d, j = [], 0.0, i
        mown = hop(i) if mows[i] else 0.0
        while j + 1 < n and j + 1 not in starts and d <= on:
            j += 1
            seen[0] = max(seen[0], j)
            d += hop(j)
            mown += hop(j) if mows[j] else 0.0
            if (not ons or d >= ons[-1][1] + every - 1e-9) and fit[j]:
                ons.append((mown, d, j))
        return backs, ons

    def search(i: int, reach: float, every: float,
               eased: Optional[bool]) -> Optional[Tuple[float, int, int, List[Pose2D]]]:
        # the best way: (cost, where it leaves, gets back on, poses)
        backs, ons = ends(i, reach, reach, every)
        best = None
        # (by what they cost at least: straight from one to the other)
        pairs = sorted((mb + ma + FIT_DRIVE * math.hypot(poses[j].x - out[k].x, poses[j].y - out[k].y), mb + ma, k, j)
                       for mb, _, k in backs for ma, _, j in ons)
        for least, lost, k, j in pairs:
            if best is not None and least >= best[0] or spent():
                break
            before = [out[k - 1]] if k > first else []
            for way in through_room(k, j) if eased is None else ways(out[k], poses[j], eased):
                field = 0.0 if route.field is None or route.clear([(q.x, q.y) for q in way]) else FIT_FIELD
                cost = lost + FIT_DRIVE * path_length(way) + field
                if (best is None or cost < best[0]) and ok(before, way):
                    best = (cost, k, j, way)
                    if not field:
                        break  # (the next way is longer)
        return best

    # turning round: the turn_room's TransitPlanner (made when needed), the ways in and out of it of a place
    room: List[Optional[TransitPlanner]] = [None]
    rooms: Dict[Tuple[int, bool], Optional[Tuple[float, float, List[Pose2D]]]] = {}
    out: List[Pose2D] = []
    orig: List[Optional[int]] = []  # which of poses each one is, None: of a way in place of some
    cut: List[float] = []  # m the blade mows of a loop or a lane on the way to each
    begins, jumps = set(), []  # where in out the sections begin, and where the path jumps (out, poses left out)
    first, i = 0, 0
    seen = [0]  # the furthest of poses read (with its section and the next one's, whether the blade mows on the way)
    # (kept only as far as a plan at another lane angle reads the same: up to the first lane, rings are loops)
    lanes = min((info.start for info in infos if info.kind == LANE and info.maneuver != 'ring'), default=n)
    # where a plan before with the same body, spaces and settings got to, as far as it read the same poses
    hashed = hashlib.sha1(repr((step, fp.corners, fp.spin_margin, fp.turning.back, FIT_REACH, FIT_STEP,
                                FIT_PLACE, FIT_VIA, FIT_EASE, FIT_EASE_OFF, FIT_DRIVE, FIT_TRIES, FIT_START, FIT_LATER,
                                FIT_FIELD, FIT_AROUND, FIT_IN, FIT_TURNS, FIT_ROUND, ROOM_SIMPLIFY,
                                route.cfg.turn_radius, route.cfg.min_turn_radius, route.cfg.turn_on_spot,
                                route.cfg.allowed_turns, route.cfg.headland_corners)).encode())
    for geom in (fp.space, route.transit.space, route.field):
        hashed.update(b'-' if geom is None else geom.wkb)
    key = hashed.hexdigest()
    with _KEPT_LOCK:
        earlier = _FITTED.get(key)

    def held(got: tuple) -> tuple:  # (a copy of where it got to: the ones kept never change)
        out, orig, cut, begins, jumps, first, i, tried, turns = got
        return (list(out), list(orig), list(cut), set(begins), [(r, list(gone)) for r, gone in jumps], first, i, tried,
                dict(turns))

    kept: List[Tuple[int, tuple]] = []
    if earlier is not None:
        for k in range(len(earlier[3]) - 1, -1, -1):
            h, got = earlier[3][k]
            if all(now[:h + 2] == then[:h + 2] for now, then in zip((poses, section, mows), earlier)):
                out, orig, cut, begins, jumps, first, i, tries[0], turns = held(got)
                seen[0], kept = h, earlier[3][:k + 1]
                break
    mark = (i // FIT_KEEP + 1) * FIT_KEEP

    def alone(upto: float = 0.0) -> None:
        # (what's driven since the section began or the path jumped is left out too where it's upto long at most: a
        # pose on its own)
        if len(out) > first and path_length(out[first:]) <= upto:
            begins.discard(first)
            if not jumps or jumps[-1][0] != first:
                jumps.append((first, []))
            jumps[-1][1].extend(o for o in orig[first:] if o is not None)
            del out[first:], orig[first:], cut[first:]

    while i < n:
        seen[0] = max(seen[0], i)
        if i >= mark and seen[0] + 2 <= lanes:  # (for a plan after this one)
            mark = (i // FIT_KEEP + 1) * FIT_KEEP
            kept.append((seen[0], held((out, orig, cut, begins, jumps, first, i, tries[0], turns))))
        if i in starts:
            alone()
            first = len(out)
        p = poses[i]
        prev = out[-1] if len(out) > first else None
        best = None
        if not fits(prev, i):
            if prev is None:  # the section starts later
                if not jumps or jumps[-1][0] != len(out):
                    jumps.append((len(out), []))
                jumps[-1][1].append(i)
                i += 1
                continue
            for further, (reach, every, tried) in enumerate(zip(FIT_REACH, FIT_STEP, FIT_PLACE)):
                if best is None and tries[0] < FIT_TRIES:
                    most[0] = tries[0] + tried
                    best = search(i, reach, every, further > 0)
            if best is None and path_length(out[first:]) > FIT_START:  # (else the section starts later)
                most[0] = tries[0] + FIT_ROUND
                best = turn_round(i)
                if best is None and not turn_room.is_empty:
                    rooms.clear()
                    most[0] = tries[0] + FIT_ROUND
                    best = search(i, FIT_REACH[-1], FIT_STEP[-1], None)
                if best is None:
                    most[0] = tries[0] + FIT_ROUND
                    best = drive_on(i)
                if best is None and sum(hop(o) for o in range(i, n) if mows[o]) > FIT_START:  # (else it ends before)
                    most[0] = tries[0] + FIT_ROUND
                    best = next(filter(None, (turn_round(i, wider) for wider in range(1, 2 * len(FIT_IN)))), None)
            if best is None:
                # nothing fits: the path jumps from the pose before to the next one that fits (the section starts later
                # where it began FIT_LATER before at most)
                alone(FIT_LATER)
                if len(out) == first:
                    continue
                if not jumps or jumps[-1][0] != len(out):
                    jumps.append((len(out), []))
                first = len(out)
                continue
        if best is None:
            if prev is None:
                begins.add(len(out))
            out.append(p)
            orig.append(i)
            cut.append(math.hypot(p.x - prev.x, p.y - prev.y) if prev is not None and mows[i] else 0.0)
            i += 1
            continue
        _, k, j, way = best
        del out[k + 1:], orig[k + 1:], cut[k + 1:]
        out[k] = way[0]
        out.extend(way[1:])
        orig.extend([None] * (len(way) - 2) + [j])
        cut.extend([0.0] * (len(way) - 1))
        i = j + 1
    _keep(_FITTED, key, (poses[:lanes], section[:lanes], mows[:lanes], kept))
    alone()
    # (where the body doesn't fit by the path's end, the dock, it ends before: no place left out where the path would
    # jump last and that and what's after leave FIT_START m of loops and lanes out at most, it gets to the end itself)
    ended = jumps and 0 < jumps[-1][0] < len(out) and \
        sum(cut[jumps[-1][0]:]) + sum(hop(o) for o in jumps[-1][1] if mows[o]) <= FIT_START
    if ended:
        r = jumps.pop()[0]
        del out[r:], orig[r:], cut[r:]
        begins.discard(r)
    # (the jumps within a section start one of its own)
    splits = sorted(r for r, gone in jumps if r < len(out) and not (gone and gone[0] in starts))

    def part(r: int) -> int:  # the section of out[r]
        return section[orig[r]] + bisect.bisect_right(splits, r)

    # the segments: the planned ones where their poses are still one after the other, the ways in place between
    at = {o: r for r, o in enumerate(orig) if o is not None}
    new: List[SegmentInfo] = []
    for info in infos:
        run: List[int] = []
        for o in range(info.start, info.end + 2):
            r = at.get(o) if o <= info.end else None
            if r is not None and run and r == run[-1] + 1 and r not in begins:
                run.append(r)
                continue
            if len(run) > 1:
                new.append(dataclasses.replace(info, start=run[0], end=run[-1], section=part(run[0])))
            run = [r] if r is not None else []

    def planned(r: int) -> bool:  # out[r - 1] to out[r] as planned, or the path jumps there
        return r in begins or orig[r] is not None and orig[r - 1] is not None and orig[r] == orig[r - 1] + 1

    def mown(gone: Sequence[int]) -> float:
        return round(sum(hop(o) for o in gone if mows[o]), 2)

    worst: List[List[float]] = []

    def place_of(a: int, b: int, m: float) -> List[float]:
        # [x, y, m of loops and lanes left out] of a place over poses[a..b]: where of those the body sticks out the most
        # (Footprint.out_by: PLACE_STEP apart along its edges first, then closer round the farthest of those that may be
        # it), [x, y, yaw, how far, 1 where it turns on the spot there else 0] of that pose to worst
        bad = [(poses[o - 1] if o not in starts else None, o) for o in range(a, b + 1)
               if not fits(poses[o - 1] if o not in starts else None, o)]
        best = None
        for (rough, at), (before, o) in sorted(zip(fp.out_by([(q, poses[o]) for q, o in bad], PLACE_STEP,
                                                             on_turn=[on_turn[o] for _, o in bad]), bad),
                                               key=lambda g: (-g[0][0], g[1][1])):
            if best is not None and rough + 0.5 * PLACE_STEP <= best[0]:
                break
            d = max(rough, fp.out_by([(before, poses[o])], around=at and (at[0], at[1], PLACE_STEP),
                                     on_turn=[on_turn[o]])[0][0])
            if best is None or d > best[0]:
                best = (d, before, o)
        d, before, o = best or (0.0, None, (a + b) // 2)
        p = poses[o]
        spin = before is not None and angle_diff(p.yaw, before.yaw) > SPIN
        worst.append([round(p.x, 3), round(p.y, 3), round(p.yaw, 3), d, int(spin)])
        return [round(p.x, 2), round(p.y, 2), m]

    places = []
    r = 1
    while r < len(out):
        if planned(r):
            r += 1
            continue
        s = r - 1
        while r < len(out) and not planned(r):
            r += 1
        new.append(SegmentInfo(TRANSIT, s, r - 1, False, 'body', part(s)))
        places.append(place_of(orig[s] + 1, orig[r - 1], mown(range(orig[s] + 1, orig[r - 1] + 1))))
    fixed = len(places)

    def lead(at: Pose2D, rs: range, start: bool) -> Optional[Tuple[int, List[Pose2D]]]:
        # (k, the way) from the point closest to `at` where the body turns whatever way it faces (in the transit space)
        # to out[k] (start, but for out[k]) or from where out[k] is to there, k of rs (picked(), FIT_REACH[0] m of path
        # at most): eased onto or off the path's line or straight (legs()), else a drive
        room = turn_room.intersection(route.transit.space)
        if room.is_empty:
            return None
        q, most[0] = nearest_points(room, Point(at.x, at.y))[0], tries[0] + FIT_ROUND
        ks = picked(rs, out, None, FIT_TURNS, FIT_REACH[0])
        of = {orig[k]: k for k in ks if orig[k] is not None and out[k] == poses[orig[k]]}
        got = next(legs(list(of), q.x, q.y, False) if start else legs(ks, q.x, q.y, True), None)
        if got:
            return (of[got[0]], got[1][:-1]) if start else got
        for k in ks:
            p = out[k]
            if start:
                way = route.drive_to((p.x, p.y), p.yaw, None, route.transit, ((q.x, q.y), None))
                if way and ok([], way[:-1] + out[k:k + 2]):
                    return k, way[:-1]
            else:
                way = route.drive_to((q.x, q.y), None, None, route.transit, ((p.x, p.y), p.yaw))
                if way and ok(out[k - 1:k] if k else [], way):
                    return k, way
        return None

    # where the path starts later in its first section or ends before (by the dock: getting from there to the path and
    # back the body isn't checked), a way from where it turns whatever way it faces closest to where it would start, or
    # to the one closest to where it would end: a place driven another way. Where none fits it says where (unchecked),
    # it starts later all the same (no place left out where that leaves FIT_START m of loops and lanes out at most) or
    # ends before
    bounds = sorted(begins | set(splits) | {0, len(out)})
    leads, unchecked = [None, None], []
    if jumps and jumps[0][0] == 0 and out and section[orig[0]] == section[0]:
        gone = jumps[0][1]
        leads[0] = lead(poses[0], range(0, next(b for b in bounds if b > 0)), True)
        if leads[0] or mown(gone) <= FIT_START:
            jumps = jumps[1:]
        if leads[0]:
            fixed += 1
            places.append(place_of(0, orig[0], round(mown(gone) + sum(cut[1:leads[0][0] + 1]), 2)))
        else:
            unchecked.append([round(out[0].x, 2), round(out[0].y, 2)])
    if ended:
        first = max(b for b in bounds if b < len(out))  # (the last section's, see legs())
        leads[1] = lead(poses[-1], range(len(out) - 1, first - 1, -1), False)
        if leads[1]:
            fixed += 1
            places.append(place_of(orig[-1] + 1, n - 1,
                                   round(mown(range(orig[-1] + 1, n)) + sum(cut[leads[1][0] + 1:]), 2)))
        else:
            unchecked.append([round(out[-1].x, 2), round(out[-1].y, 2)])
    for r, gone in jumps:
        places.append(place_of(orig[r - 1] + 1 if r else 0, orig[r] if r < len(out) else n - 1, mown(gone)))
    new.sort(key=lambda info: (info.start, info.end))
    over = [place[:2] for (r, _), place in zip(jumps, places[fixed:]) if r in splits]
    if leads[1]:  # (the segments as far as out[k], then the way from there)
        k, way = leads[1]
        at = SegmentInfo(TRANSIT, k, k + len(way) - 1, False, 'body', new[-1].section)
        new = [dataclasses.replace(info, end=min(info.end, k)) for info in new if info.start < k] + [at]
        out[k:] = way
    if leads[0]:  # (the way, then the segments from out[k] on)
        k, way = leads[0]
        new = [SegmentInfo(TRANSIT, 0, len(way), False, 'body', new[0].section)] + [
            dataclasses.replace(info, start=max(info.start, k) + len(way) - k, end=info.end + len(way) - k)
            for info in new if info.end > k]
        out[:k] = way
    return out, new, {'fixed': fixed, 'left': len(jumps), 'skipped_m': round(sum(m for _, _, m in places), 2),
                      'places': places, 'poses': worst, 'jumps': over, 'unchecked': unchecked, 'tries': tries[0]}


def _lanes_field(passes: List[Tuple[float, List[List[Lane]]]], trim: float, half: float):
    """Where the lanes of every set lie, each cell's from lane to lane and half further out, but for trim at their
    ends."""
    parts = []
    for a, cells in passes:
        for cell in cells:
            ends = []
            for lane in cell:
                x0, x1 = lane.x0 + trim + half, lane.x1 - trim - half
                if x0 > x1:
                    x0 = x1 = 0.5 * (lane.x0 + lane.x1)
                ends.append(Lane(lane.row, x0, x1, lane.y).ends(a))
            if len(ends) > 1:
                parts.append(Polygon([e[0] for e in ends] + [e[1] for e in ends[::-1]]).buffer(half, 1))
    return union(parts)


def _field_turns(route: '_Route', poses: List[Pose2D],
                 infos: List[SegmentInfo]) -> Tuple[List[List[float]], int, float]:
    """[x, y] of every place where a drive turns in the lanes' field (headland_turns): the first pose there whose
    heading changes, those less than a metre apart one place. Not on the arcs rounding a corner out of it into it: how
    many of those turn in it, and how far in the furthest gets (m, along the way from pose to pose, to a few mm)."""
    places: List[List[float]] = []
    last: Optional[Pose2D] = None
    arcs, deepest, on = 0, 0.0, False  # (on: the pose before turns on one of those arcs in the field)
    for info in infos:
        if info.kind != TRANSIT:
            continue
        for i in range(info.start + 1, info.end + 1):
            p = poses[i]
            turns = angle_diff(p.yaw, poses[i - 1].yaw) > 1e-6 and not route.clear([(p.x, p.y)])
            if turns and (p.x, p.y) in route.rounded:
                arcs += not on
                q = poses[i - 1]
                way = [(q.x + f * (p.x - q.x), q.y + f * (p.y - q.y)) for f in (0.25, 0.5, 0.75, 1.0)]
                deepest = max([deepest] + [route.field.boundary.distance(Point(xy)) for xy in way
                                           if not route.clear([xy])])
            elif turns:
                if last is None or math.hypot(p.x - last.x, p.y - last.y) >= 1.0:
                    places.append([round(p.x, 2), round(p.y, 2)])
                last = p
            on = turns and (p.x, p.y) in route.rounded
    return places, arcs, round(deepest, 3)





def _assemble(segments: List[PathSegment]) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    poses: List[Pose2D] = []
    infos: List[SegmentInfo] = []
    for seg in segments:
        start = None
        for p in seg.poses:
            if poses and same_position(poses[-1], p):
                # one pose per position, with the heading the mower leaves it with
                poses[-1] = p
                if start is None:
                    start = len(poses) - 1
                continue
            poses.append(p)
            if start is None:
                start = len(poses) - 1
        if start is not None and len(poses) - 1 > start:
            infos.append(SegmentInfo(seg.kind, start, len(poses) - 1, seg.reverse, seg.maneuver, seg.section,
                                     seg.source))
    return poses, infos


def _round_corners(route: '_Route', poses: List[Pose2D],
                   infos: List[SegmentInfo]) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    """Where a drive or a way of _fit_body (TRANSIT) bends at a pose by more than CORNER_ANGLE (turns on the spot there
    from footprint.SPIN on), an arc in its place, of SMOOTH_RADII of turn_radius (tighter than min_turn_radius too): the
    widest that keeps in the transit space and the body fits on. Tangent to the straight ends of the ways in and out,
    into the loop, lane or turn next to it turn_radius at most (as much as a right angle of that takes) and only where
    the blade mows what it leaves of it on the arc or on what no rounding leaves out (but EARLIER_LEFT); else an S
    from as far back on the way in as that allows (or where the drive's straight way in starts) onto the drive's way out
    further on, or the other way round, turning 90 degrees more at most. With headland_turns out of the lanes' field
    where the corner is out of it, with headland_corners 'rounded' into it where nothing else fits (an arc only,
    FIELD_ARC deep at most, its points in route.rounded), with 'sharp' not where a drive along the headland goes
    (route.sharp). The arc goes with the drive, the piece next to it ends or starts where it does."""
    out = list(poses)
    own = [0] * len(out)  # the segment the way to each pose is part of (of one a segment starts with: that one)
    shared = [k > 0 and infos[k - 1].end == info.start for k, info in enumerate(infos)]
    for k, info in enumerate(infos):
        own[info.start + shared[k]:info.end + 1] = [k] * (info.end + 1 - info.start - shared[k])

    def straight(j: int, h: float) -> float:  # m from out[j] to out[j + 1] along h on the way, else 0
        p, q = out[j], out[j + 1]
        a, b = infos[own[j]], infos[own[j + 1]]
        if a.section != b.section or angle_diff(p.yaw, h) > 1e-6 or \
                abs((q.y - p.y) * math.cos(h) - (q.x - p.x) * math.sin(h)) > 1e-6:
            return 0.0
        return math.hypot(q.x - p.x, q.y - p.y)

    def leg(j: int, h: float, step: int, drive: bool = False) -> float:
        # m straight along h from out[j] back (step -1) or on (1), drive: on drives only
        d, j = 0.0, j - 1 if step < 0 else j
        while 0 <= j < len(out) - 1:
            hop = straight(j, h)
            if hop <= 0.0 or drive and infos[own[j + 1]].kind != TRANSIT:
                break
            d, j = d + hop, j + step
        return d

    def ways(p: Pose2D, u: float, w: float, r: float, s: bool) -> Iterator[Tuple[float, float, List[Pose2D]]]:
        # (m before p it leaves the way, m after p it gets back on, the poses between), s: the S's too
        turn = normalize_angle(w - u)
        trim = r * math.tan(0.5 * abs(turn))
        if trim <= min(back, on) + 1e-9:
            arc = sample_arc(p.x - trim * math.cos(u), p.y - trim * math.sin(u), u, r, turn, route.step)
            yield trim, trim, [arc[0]._replace(yaw=u)] + arc[1:-1] + [arc[-1]._replace(yaw=w)]  # (yaws as they were)
        # an S from as far back on the way in as it may (or where the drive's straight way in starts) onto the drive's
        # way out further on, or the other way round
        for a, b in [(f, d) for f in sorted({min(back, trim), min(behind, trim)}, reverse=True)
                     for d in (trim + 2.0 * r, ahead) if s and trim < d <= ahead] + \
                [(d, f) for f in sorted({min(on, trim), min(ahead, trim)}, reverse=True)
                 for d in (trim + 2.0 * r, behind) if s and trim < d <= behind]:
            for m in dubins(Pose2D(p.x - a * math.cos(u), p.y - a * math.sin(u), u),
                            Pose2D(p.x + b * math.cos(w), p.y + b * math.sin(w), w), r, route.step,
                            turning=abs(turn) + 0.5 * math.pi)[:1]:
                if m.name == 'u_turn' or not route.cfg.turn_on_spot:  # (no loop with turn_on_spot)
                    yield a, b, m.pieces[0].poses

    def fit(a: float, b: float, arc: List[Pose2D], out_of: bool) -> Optional[Tuple[int, int, List[Pose2D]]]:
        # (the poses before and after it that stay, those and it), where it fits
        s, e = i - 1, i + 1
        while s > 0 and math.hypot(out[s].x - p.x, out[s].y - p.y) < a - 1e-9:
            s -= 1
        while e < len(out) - 1 and math.hypot(out[e].x - p.x, out[e].y - p.y) < b - 1e-9:
            e += 1
        new = join(out[s:s + 1], arc, out[e:e + 1])
        pts = [(q.x, q.y) for q in arc]
        # (where it ends at the next corner that one turns from the arc's last but one pose)
        if not route.transit.is_free(pts) or out_of and not route.clear(pts) or \
                deep is not None and clear and deep.intersects(LineString(pts)) or \
                not route.body(new[1:] if same_position(arc[-1], out[e]) else arc):
            return None
        # of the pieces next to the drive it leaves out the blade mows all but EARLIER_LEFT on it or elsewhere
        left = [route.blade([arc[0], Pose2D(p.x - behind * math.cos(u), p.y - behind * math.sin(u), u)])] \
            if a > behind + 1e-9 else []
        left += [route.blade([Pose2D(p.x + ahead * math.cos(w), p.y + ahead * math.sin(w), w), arc[-1]])] \
            if b > ahead + 1e-9 else []
        if left:
            left = subtract(union(left), [route.blade(arc)])
            if left.area > EARLIER_LEFT and subtract(left, [mowed()]).area > EARLIER_LEFT:
                return None
        return s, e, new

    # (the poses as they were by the square metre they're in, those further than turn_radius along the path from a
    # drive: no rounding leaves them out)
    grid: Dict[Tuple[int, int], List[int]] = {}
    for j, q in enumerate(poses):
        grid.setdefault((math.floor(q.x), math.floor(q.y)), []).append(j)
    gap = [math.inf] * len(poses)
    for info in infos:
        if info.kind == TRANSIT:
            gap[info.start:info.end + 1] = [0.0] * (info.end + 1 - info.start)
    for js in (range(1, len(poses)), range(len(poses) - 2, -1, -1)):
        for j in js:
            q, o = poses[j], poses[j - js.step]
            gap[j] = min(gap[j], gap[j - js.step] + math.hypot(q.x - o.x, q.y - o.y))

    def mowed():
        # what the blade mows round the corner on the poses as they were that no rounding leaves out (worked out once a
        # corner)
        if not here:
            reach = min(max(back, on), route.cfg.turn_radius * math.tan(0.5 * angle_diff(w, u))) + \
                route.cfg.mower_width + 2.0 * (abs(route.cfg.blade_ahead) + abs(route.cfg.blade_offset)) + route.step
            x0, y0, x1, y1 = (math.floor(v) for v in (p.x - reach, p.y - reach, p.x + reach, p.y + reach))
            runs: List[List[Pose2D]] = []
            for j in sorted(j for gx in range(x0, x1 + 1) for gy in range(y0, y1 + 1) for j in grid.get((gx, gy), ())
                            if gap[j] > route.cfg.turn_radius + 1e-6
                            and math.hypot(poses[j].x - p.x, poses[j].y - p.y) <= reach):
                if not runs or runs[-1][-1] != j - 1:
                    runs.append([])
                runs[-1].append(j)
            here.append(route.blade(*[poses[run[0]:run[-1] + 1] for run in runs if len(run) > 1]))
        return here[0]

    strict = route.cfg.headland_corners == 'sharp'
    # (what of the lanes' field is more than FIELD_ARC in: an arc into it from a corner out of it keeps out of that)
    deep = prep(route.field.buffer(-FIELD_ARC, QUAD_SEGS)) if route.field is not None and not strict else None
    i = 1
    while i < len(out) - 1:
        p, u, w = out[i], out[i - 1].yaw, out[i].yaw
        into, onto = infos[own[i]], infos[own[i + 1]]  # (the ways to and from it)
        k = own[i + 1] if onto.kind == TRANSIT else own[i]
        if angle_diff(w, u) <= CORNER_ANGLE or angle_diff(w, u) > math.pi - 1e-3 or infos[k].kind != TRANSIT or \
                not infos[own[i - 1]].section == into.section == onto.section or \
                (p.x, p.y) in route.sharp:
            i += 1
            continue
        # (into a loop, lane or turn next to it turn_radius at most, as much as a right angle of that takes: no more of
        # it left out)
        behind, ahead = leg(i, u, -1, True), leg(i, w, 1, True)
        back = min(leg(i, u, -1), behind + route.cfg.turn_radius)
        on = min(leg(i, w, 1), ahead + route.cfg.turn_radius)
        # (with headland_turns out of the lanes' field where the corner is, with headland_corners 'rounded' into it
        # where nothing else fits, no S then)
        clear, here = route.clear([(p.x, p.y)]), []
        found = next(((got, arc, out_of) for out_of in ([True, False] if clear and not strict else [clear])
                      for r in (f * route.cfg.turn_radius for f in SMOOTH_RADII) if r >= 1e-3
                      for a, b, arc in ways(p, u, w, r, out_of or not clear) for got in [fit(a, b, arc, out_of)]
                      if got), None)
        if found is None:
            i += 1
            continue
        (s, e, new), arc, out_of = found
        if clear and not out_of:
            route.rounded.update((q.x, q.y) for q in arc)
        # (the way to the arc's start is the piece's it lies on, the way on from its end too)
        mid = [k] * (len(new) - 2) if same_position(out[s], arc[0]) else [own[s + 1]] + [k] * (len(new) - 3)
        own[s:e + 1] = [own[s]] + mid + [k if same_position(arc[-1], out[e]) else own[e]]
        out[s:e + 1] = new
        i = s + len(new) - 1
    lo: Dict[int, int] = {}
    hi: Dict[int, int] = {}
    for j, k in enumerate(own):
        lo.setdefault(k, j)
        hi[k] = j
    kept = []
    for k, info in enumerate(infos):
        if k in lo:
            start = lo[k] - shared[k]
            if hi[k] > start:
                kept.append(dataclasses.replace(info, start=start, end=hi[k]))
    return out, kept


def _mop_up(route: '_Route', poses: List[Pose2D], infos: List[SegmentInfo],
            free) -> Tuple[List[Pose2D], List[SegmentInfo], Dict[str, Any]]:
    """mop_up: of what the path leaves unmowed of free the pieces of MOP_MIN at least (the biggest MOP_MAX), each with a
    pass of MOP_PASS where the body fits that mows the most of it (MOP_GAIN of MOP_MIN at least); from the path's end a
    drive to the nearest next (through the transit space from where it is, the body checked) and its pass, then back to
    where the path ended. Returns the poses, the segments (drives TRANSIT, passes LANE, 'mop_up') and {pieces, area: m2
    mowed of them}."""
    cfg, fp = route.cfg, route.footprint
    starts = [k for k, info in enumerate(infos) if k == 0 or info.section != infos[k - 1].section]
    bounds = [infos[k].start for k in starts] + [len(poses)]
    mowed = route.blade(*[poses[a:b] for a, b in zip(bounds, bounds[1:]) if b - a > 1], whole=True)
    pieces = sorted((p for p in polygons(subtract(free, [mowed])) if p.area >= MOP_MIN), key=lambda p: -p.area)
    passes = []
    for piece in pieces[:MOP_MAX]:
        best = None
        for q in (piece.representative_point(), piece.centroid):
            for k in range(MOP_HEADINGS):
                h = 2.0 * math.pi * k / MOP_HEADINGS
                c, s = math.cos(h), math.sin(h)
                # (the mower's centre with the blade's on q, the pass's middle)
                x = q.x - cfg.blade_ahead * c + cfg.blade_offset * s
                y = q.y - cfg.blade_ahead * s - cfg.blade_offset * c
                d = 0.5 * MOP_PASS
                a, b = (x - d * c, y - d * s), (x + d * c, y + d * s)
                if not route.transit.is_free([a, b]):
                    continue
                way = sample_straight(a[0], a[1], b[0], b[1], h, route.step)
                if not fp.poses_fit(way):
                    continue
                gain = piece.intersection(route.blade(way)).area
                if best is None or gain > best[0]:
                    best = (gain, way)
        if best is not None and best[0] >= MOP_GAIN * MOP_MIN:
            passes.append(best)
    out, new, section = list(poses), list(infos), infos[-1].section if infos else 0
    end, area, done = poses[-1], 0.0, 0

    def leave(way: Optional[List[Pose2D]], at: Pose2D) -> Optional[List[Pose2D]]:
        # a drive from where the mower is: turning on the spot first where it doesn't leave the way it faces
        if not way or not same_position(way[0], at):
            return None
        if len(way) > 1 and not same_position(way[0], way[1]):
            yaw = math.atan2(way[1].y - way[0].y, way[1].x - way[0].x)
            if angle_diff(yaw, way[0].yaw) > 1e-6:
                way = [way[0], Pose2D(way[0].x, way[0].y, yaw)] + way[1:]
        return way

    def add(kind: str, way: List[Pose2D]) -> None:
        merged = join([out[-1]], way)
        if len(merged) > 1:
            new.append(SegmentInfo(kind, len(out) - 1, len(out) + len(merged) - 2, False, 'mop_up', section))
            out[-1] = merged[0]  # (facing the way it leaves: turning on the spot there)
            out.extend(merged[1:])

    while passes:
        here = out[-1]
        passes.sort(key=lambda g: math.hypot(g[1][0].x - here.x, g[1][0].y - here.y))
        gain, way = passes.pop(0)
        drive = leave(route.drive_to((way[0].x, way[0].y), way[0].yaw, None, route.transit,
                                     ((here.x, here.y), here.yaw)), here)
        # (the body checked from the pose before it leaves on, as the path will be: one pose on a position, the later,
        # turning on the spot there, where the drive and the pass meet too)
        if not drive or not fp.poses_fit(join(out[-2:], drive, way)):
            continue
        add(TRANSIT, drive)
        add(LANE, way)
        area, done = area + gain, done + 1
    if done:
        back = leave(route.drive_to((end.x, end.y), None, None, route.transit, ((out[-1].x, out[-1].y), out[-1].yaw)),
                     out[-1])
        if back and fp.poses_fit(join(out[-2:], back)):
            add(TRANSIT, back)
    return out, new, {'pieces': done, 'area': round(area, 3)}


def _turn_at(route: '_Route', poses: List[Pose2D],
             infos: List[SegmentInfo]) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    """Where the path turns on the spot (the heading jumping by more than footprint.SPIN) and the way in
    is longer than twice TURN_AHEAD, a pose TURN_AHEAD before the point on it, facing the way it came (where the body
    fits so)."""
    first = {info.start for k, info in enumerate(infos) if k == 0 or infos[k - 1].section != info.section}
    out, at = [], []  # (at: where the poses are put in, by the index of the one after)
    for i, p in enumerate(poses):
        q = poses[i - 1] if i else p
        d = math.hypot(p.x - q.x, p.y - q.y)
        if i not in first and angle_diff(p.yaw, q.yaw) > SPIN and d > 2.0 * TURN_AHEAD:
            f = TURN_AHEAD / d
            ahead = Pose2D(p.x + f * (q.x - p.x), p.y + f * (q.y - p.y), q.yaw)
            if route.body([ahead]):
                out.append(ahead)
                at.append(i)
        out.append(p)

    def moved(j: int) -> int:
        return j + bisect.bisect_right(at, j)

    return out, [dataclasses.replace(info, start=moved(info.start), end=moved(info.end)) for info in infos]
