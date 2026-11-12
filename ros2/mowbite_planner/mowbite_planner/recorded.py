"""The recorded loop (recorded_loop): the line the area was recorded on (space.Recorded) driven as the outermost loop the
way it was recorded, the body where it was then (space.recorded_space). Where the body doesn't fit so, that stretch is
driven anew further in (trace.repaired), and left out where that doesn't fit either: the loop in pieces, the first loop
as it is without recorded_loop driven along what's left out (_recorded_else). The loop's centre runs outside where the
drives go, so each piece has a way onto it from where the drives go and one off it again to there (as lead.py's onto a
plan's first loop). sweep.on_recorded: which poses of a path are its."""

import math
from typing import List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point
from shapely.ops import substring
from shapely.prepared import prep

from .config import PERIMETER, XY
from .footprint import SPIN
from .geometry import union
from .lead import LEAD_IN, LEAD_LEN, LEAD_SPREAD, LEAD_TRIES, START_SWING, START_SWUNG
from .levels import CORNER_CUT
from .maneuvers import (clear_of_corners, closest_on_polyline, drive_polyline, join, path_length,
                        sample_straight)
from .mathutil import Pose2D, angle_diff
from .trace import _bad, repaired

# m, a piece of the loop shorter than this isn't driven (its ways on and off are longer than it), nor a stretch of the
# first loop in its place
PIECE_MIN = 2.0
# m: along what's left out of the loop the first loop as without it is driven, where that's further from the pieces
# driven than it runs inside the loop's line and ELSE_OFF more (a piece driven anew runs further in), looked at
# ELSE_STEP m apart, ELSE_ON m on along it at both ends
ELSE_OFF = 0.1
ELSE_STEP = 0.05
ELSE_ON = 0.3
# m either side of a point of the loop its heading there is taken over for the ways on and off
LEAD_SPAN = 0.25
# m, a way on or off for a body that reaches far ahead of its centre (behind it: off): t m from the loop it runs
# LEAD_NEAR (e^(t / l) - 1) off it, l LEAD_REACH times what the body reaches ahead (LEAD_GENTLE at least). Coming onto a
# straight line any faster, the corner ahead on the line's outer side swings out past where it is on the line
LEAD_NEAR = 0.0025
LEAD_REACH = 1.15
LEAD_GENTLE = 0.3


class RecordedLoops:
    """Driving the recorded loop, methods of the route (rings._Route)."""

    def drive_recorded(self, ring: List[XY], target: Optional[XY] = None) -> None:
        """The recorded loop round ring (as rings() gives it) the way the outline was recorded, from its point closest
        to target (default: where the mower is): each piece of it the body fits on (_recorded_pieces), on round from
        there, then the first loop as without it along what's left out (_recorded_else)."""
        cfg, fp = self.cfg, self.recorded_fp
        closed = ring + [ring[0]]
        target = self.pos if target is None else target
        i, entry, _ = (0, closed[0], 0.0) if target is None else closest_on_polyline(closed, target)
        entry = clear_of_corners(closed, i, entry, 2.0 * cfg.perimeter_corner_radius)
        loop = [entry] + closed[i + 1:] + closed[1:i + 1] + [entry]
        if not self.recorded:  # (rings come ccw)
            loop = loop[::-1]
        room = prep(self.recorded_room)

        def fits(pts: Sequence[XY]) -> bool:
            return room.covers(LineString(pts) if len(pts) > 1 else Point(pts[0]))

        poses = drive_polyline(loop, self.step, cfg.perimeter_corner_radius, fits, 0, CORNER_CUT * cfg.mower_width, 0,
                               cfg.lane_spacing, cfg.mower_width, fp.poses_fit)
        fixed, done, driven = 0, 0.0, []
        if len(poses) > 3:
            poses, fixed, _ = repaired(poses, fp, loop[:-1], False, self.step, cfg.mower_width, cfg.blade_ahead,
                                       self.recorded_room, taken=self._taken(cfg.lane_spacing))
            driven = self._recorded_pieces(poses, 1.0 if self.recorded else -1.0, fits)
        for lead, piece, off in driven:
            self.source = ''  # (the drive there is none of the loop's)
            self.go_to((lead[0].x, lead[0].y), lead[0].yaw)
            name = f'loop-{len(self.loops)}'
            self.source = name
            self.origins[name] = {'parent': 'recorded', 'kind': 'outline', 'level': 0}
            self.recorded_loops.add(name)
            self.add(PERIMETER, lead, maneuver='lead')
            self.add(PERIMETER, piece)
            self.add(PERIMETER, off, maneuver='lead')
            self.loops.append([(p.x, p.y) for p in piece])
            done += path_length(piece)
        got, left = self.recorded_done, max(0.0, LineString(closed).length - done)
        got['m'], got['left_m'], got['repaired'] = got['m'] + done, got['left_m'] + left, got['repaired'] + fixed
        got['pieces'] = len(self.recorded_loops)
        if left > PIECE_MIN:
            got['else_m'] += self._recorded_else(ring, [piece for _, piece, _ in driven])

    def _recorded_pieces(self, poses: List[Pose2D], side: float, fits) -> List[Tuple[list, list, list]]:
        """The pieces of the closed loop poses (the lawn on the left with side 1, else on the right) the body fits on
        all along, PIECE_MIN long at least, each with its way on and its way off: [(the way on, the piece, the way
        off)] in the order they're driven. One piece where the loop fits all the way (_recorded_whole)."""
        ring = poses[:-1]
        n = len(ring)
        bad = sorted({i % n for i in _bad(poses, self.recorded_fp)})
        if not bad:
            got = self._recorded_whole(ring, side, fits)
            return [got] if got is not None else []
        out = []
        for a, b in zip(bad, bad[1:] + [bad[0] + n]):
            got = self._recorded_open(ring, a + 1, b - a - 1, side, fits) if b - a > 2 else None
            if got is not None:
                out.append(got)
        return out

    def _recorded_picks(self, ring: List[Pose2D], a: int, count: int) -> Tuple[List[float], List[int]]:
        """(m along the count poses of ring from its pose a on at each, those of them LEAD_SPREAD apart: where a piece
        may start and end)."""
        n, at = len(ring), [0.0]
        for k in range(1, count):
            p, q = ring[(a + k - 1) % n], ring[(a + k) % n]
            at.append(at[-1] + math.hypot(q.x - p.x, q.y - p.y))
        picks = [0]
        for k in range(1, count):
            if at[k] - at[picks[-1]] >= LEAD_SPREAD:
                picks.append(k)
        return at, picks

    def _recorded_lead_at(self, ring: List[Pose2D], k: int, side: float, fits, onto: bool,
                          start: bool = False) -> Optional[List[Pose2D]]:
        """_recorded_lead at pose k of ring: back along the loop for the way on, on along it for the way off."""
        n, step = len(ring), -1 if onto else 1
        return self._recorded_lead([ring[(k + step * (j + 1)) % n] for j in range(n - 1)], ring[k % n], side, fits,
                                   onto, start)

    def _recorded_whole(self, ring: List[Pose2D], side: float, fits) -> Optional[Tuple[list, list, list]]:
        """(the way on, the piece, the way off) of a loop the body fits on all the way (ring, from where it would
        start): the whole loop from its pose closest to there a way on and a way off both fit at (LEAD_TRIES poses
        LEAD_SPREAD apart looked at), else from a pose a way on fits at round to the last one before it a way off
        fits at (every other of those poses, what's between the two left out: the least). The plan's first from where
        the body turns whatever way it faces where there's such a way (_recorded_lead). None where there's none."""
        n = len(ring)
        at, picks = self._recorded_picks(ring, 0, n)
        length = at[-1] + math.hypot(ring[0].x - ring[-1].x, ring[0].y - ring[-1].y)
        near = sorted(picks, key=lambda k: (min(at[k], length - at[k]), k))[:LEAD_TRIES]
        for start in ([True, False] if not self.started and not self.segments else [False]):
            ons = {k: self._recorded_lead_at(ring, k, side, fits, True, start) for k in near}
            for k in near:  # (the closest to where it would start first, either way round)
                off = ons[k] and self._recorded_lead_at(ring, k, side, fits, False)
                piece = [ring[(k + j) % n] for j in range(n + 1)]
                if off and self.recorded_fp.poses_fit(join(ons[k], piece, off)):
                    return ons[k], piece, off
            ons.update((k, self._recorded_lead_at(ring, k, side, fits, True, start)) for k in picks[::2]
                       if k not in ons)
            offs = {k: self._recorded_lead_at(ring, k, side, fits, False) for k in picks[::2]}
            # (from on round to off: what's left out is from off on to on)
            for gap, on, off in sorted(((at[on] - at[off]) % length, on, off) for on in ons if ons[on]
                                       for off in offs if offs[off] and off != on):
                if length - gap < PIECE_MIN:
                    break
                piece = [ring[(on + j) % n] for j in range((off - on) % n + 1)]
                if self.recorded_fp.poses_fit(join(ons[on], piece, offs[off])):
                    return ons[on], piece, offs[off]
        return None

    def _recorded_open(self, ring: List[Pose2D], a: int, count: int, side: float,
                       fits) -> Optional[Tuple[list, list, list]]:
        """(the way on, the piece, the way off) of the piece of count poses of ring from its pose a on: it starts at
        the first of its poses, LEAD_SPREAD apart, a way onto it fits at and ends at the last one a way off fits at
        (LEAD_TRIES looked at each). None where there are none or that leaves less than PIECE_MIN."""
        n = len(ring)
        at, picks = self._recorded_picks(ring, a, count)
        for start in ([True, False] if not self.started and not self.segments else [False]):
            first = last = lead = off = None
            for k in picks[:LEAD_TRIES]:
                if at[-1] - at[k] < PIECE_MIN:
                    break
                lead = self._recorded_lead_at(ring, a + k, side, fits, True, start)
                if lead:
                    first = k
                    break
            for k in picks[::-1][:LEAD_TRIES] if first is not None else ():
                if at[k] - at[first] < PIECE_MIN:
                    break
                off = self._recorded_lead_at(ring, a + k, side, fits, False)
                if off:
                    last = k
                    break
            if last is not None:
                piece = [ring[(a + k) % n] for k in range(first, last + 1)]
                if self.recorded_fp.poses_fit(join(lead, piece, off)):
                    return lead, piece, off
        return None

    def _recorded_lead(self, beyond: Sequence[Pose2D], at: Pose2D, side: float, fits, onto: bool,
                       start: bool = False) -> Optional[List[Pose2D]]:
        """The way onto the loop at its pose at (onto), or off it there: along the loop beyond that pose (its poses
        from the one next to at on, away from it), LEAD_IN off it on the lawn's side at its far end, getting onto the
        loop smoothly over LEAD_LEN (as lead.way_onto). The far end where the drives go and the body fits as on any
        drive (start, a plan's first pose: where it turns whatever way it faces, turned START_SWING its first
        START_SWUNG too, mower_logic gets there by itself that far off its heading), the centre inside the loop's line
        all the way, no turn on the spot, the body fitting as on the loop. None where none fits."""
        fp, step = self.footprint, self.step
        xs = [x for x, _ in self.cfg.body_outline]
        gentle = max(LEAD_GENTLE, LEAD_REACH * (max(xs) if onto else -min(xs)))
        # (m long, m off the loop t m along it from the loop's pose: lead.py's, then the gentle ones)
        shapes = [(length, lambda t, off=off, length=length: off * (t / length) ** 2 * (3.0 - 2.0 * t / length))
                  for length in LEAD_LEN for off in LEAD_IN] + \
            [(gentle * math.log1p(off / LEAD_NEAR), lambda t: LEAD_NEAR * math.expm1(t / gentle)) for off in LEAD_IN]
        line, d, last = [(0.0, at.x, at.y)], 0.0, at
        for p in beyond:
            d += math.hypot(p.x - last.x, p.y - last.y)
            line.append((d, p.x, p.y))
            last = p
            if d > max(length for length, _ in shapes) + LEAD_SPAN:
                break
        # (m along, the point, the way the loop goes there over LEAD_SPAN either side: where a recorded line's points
        # zigzag, LEAD_IN off it along each one's own heading they'd lie one behind the other)
        back, a, b, sign = [], 0, 0, -1.0 if onto else 1.0
        for t, x, y in line:
            while line[a][0] < t - LEAD_SPAN:
                a += 1
            while b < len(line) - 1 and line[b][0] < t + LEAD_SPAN:
                b += 1
            if b > a:
                back.append((t, x, y, math.atan2(sign * (line[b][2] - line[a][2]), sign * (line[b][1] - line[a][1]))))
        for length, off in shapes:
            pts = []
            for t, x, y, yaw in back:
                if t > length + 1e-9:
                    break
                o = off(t)
                pts.append((x - o * side * math.sin(yaw), y + o * side * math.cos(yaw)))
            if len(pts) < 3 or not self.transit.is_free(pts[-1:]) or not fits(pts) or \
                    start and not fp.turning.anywhere.contains(Point(pts[-1])):
                continue
            if onto:
                pts = pts[::-1]
            way = join(*(sample_straight(p[0], p[1], q[0], q[1], math.atan2(q[1] - p[1], q[0] - p[0]), step)
                         for p, q in zip(pts, pts[1:])))
            if onto:
                way[-1] = at  # (ending on the loop's pose)
            far = way[0] if onto else way[-1]
            if any(angle_diff(p.yaw, q.yaw) > SPIN for p, q in zip(way, way[1:])) or \
                    not onto and angle_diff(at.yaw, way[0].yaw) > SPIN or not fp.fits_pose(far.x, far.y, far.yaw):
                continue
            if start:
                j, gone = 1, 0.0
                while j < len(way) and gone <= START_SWUNG:
                    gone += math.hypot(way[j].x - way[j - 1].x, way[j].y - way[j - 1].y)
                    j += 1
                if not all(fp.swung(START_SWING).fit(way[:j])):
                    continue
            if self.recorded_fp.poses_fit(way):
                return way
        return None

    def _recorded_else(self, ring: List[XY], pieces: List[List[Pose2D]]) -> float:
        """Along what's left out of the recorded loop round ring (pieces: what's driven of it) the first loop as it is
        without recorded_loop (recorded_first() gives its rings): its stretches further from the pieces than it runs
        inside the loop's line (see ELSE_OFF), PIECE_MIN long at least, the way the outline was recorded; m of them."""
        if self._recorded_first is None:
            self._recorded_first = self.recorded_first()
        cfg, mine = self.cfg, prep(LineString(ring + ring[:1]).buffer(0.5, 4))
        near = abs(cfg.centre_offset - cfg.recorded_loop_margin) + ELSE_OFF
        done = prep(union(LineString([(p.x, p.y) for p in piece]).buffer(near, 4) for piece in pieces))
        total = 0.0
        for first in self._recorded_first:
            line = LineString((first + first[:1])[::1 if self.recorded else -1])
            if line.length < PIECE_MIN or not mine.intersects(line):
                continue
            k, length = max(8, int(line.length / ELSE_STEP)), line.length
            free = [not done.contains(line.interpolate(j * length / k)) for j in range(k)]
            if all(free):  # (all of it: a loop as any)
                self.drive_ring(first, False, level=0, parent='instead')
                total += length
                continue
            j0, a = free.index(False), None  # (from a point that's done: no stretch goes over where it starts)
            for j in range(j0, j0 + k + 1):
                if free[j % k] and a is None:
                    a = j
                elif not free[j % k] and a is not None:
                    u0, u1 = a * length / k - ELSE_ON, (j - 1) * length / k + ELSE_ON
                    if u1 - u0 >= PIECE_MIN:
                        total += self._recorded_stretch(_along(line, u0 % length, u0 % length + u1 - u0))
                    a = None
        return total

    def _recorded_stretch(self, pts: List[XY]) -> float:
        """A stretch of the first loop as without recorded_loop (pts, the way it's driven) as a loop of that level; m
        of it."""
        cfg = self.cfg
        poses = drive_polyline(pts, self.step, cfg.perimeter_corner_radius, self.area.is_free, 0,
                               CORNER_CUT * cfg.mower_width, 0, cfg.lane_spacing, cfg.mower_width, self.body)
        if len(poses) < 2:
            return 0.0
        name = f'loop-{len(self.loops)}'
        self.source = name
        self.origins[name] = {'parent': 'instead', 'kind': 'outline', 'level': 0}
        self.go_to((poses[0].x, poses[0].y), poses[0].yaw)
        self.add(PERIMETER, poses)
        self.loops.append([(p.x, p.y) for p in poses])
        self.loop_side[name] = 1.0 if self.recorded else -1.0
        return path_length(poses)


def _along(line: LineString, u0: float, u1: float) -> List[XY]:
    """The points of the closed line from u0 m along it to u1 (past its end: on from its start)."""
    if u1 <= line.length:
        return list(substring(line, u0, u1).coords)
    return list(substring(line, u0, line.length).coords) + list(substring(line, 0.0, u1 - line.length).coords)[1:]
