"""The route through an area (_Route, planner.py step 4): the loops, the lanes and the turns and drives between
them, then the turns on the spot rounded off where that fits (_smooth_spins)."""

import bisect
import math
import random
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.ops import clip_by_rect, nearest_points

from .config import (EARLIER_LEFT, FIT_EASE, FIT_EASE_OFF, LANE, LOOP_EASE, LOOP_IN, LOOP_JOIN, LOOP_ROUND,
                     LOOP_ROUND_GAP, LOOP_ROUND_REACH, LOOP_ROUND_TRIES, LOOP_TRIES, ORDER_BUDGET, PERIMETER,
                     ROUND_STEP, SMOOTH_RADII, SPIN_LIMIT, SWEPT_PIECE, TRANSIT, TURN, TURN_REACH, XY, Lane,
                     PathSegment, PlannerConfig)
from .footprint import SPIN, TURN_STEP, Footprint
from .geometry import QUAD_SEGS, inset, polygonal, polygons, rings, subtract, union
from .lanes import _ring_list, _Scanner
from .maneuvers import (CORNER_ANGLE, Fits, Maneuver, closest_on_polyline, connect, drive_polyline, dubins, join,
                        omega, path_length, plan_turn, same_position, sample_arc, sample_straight, turn_radii)
from .mathutil import Pose2D, angle_diff, normalize_angle
from .transit import TransitPlanner

try:  # shapely 2 tests many points at once
    import numpy
    from shapely import covers, distance, points
except ImportError:  # shapely 1.7 (ROS Noetic)
    covers = None


# rounding a loop's corner may cut it by this share of the mower width at most, 90 degree corners keep the
# full perimeter_corner_radius, sharper ones get less
CORNER_CUT = 0.3
# m, lane ends are pulled back to this when the body doesn't fit at them
BODY_STEP = 0.01
# headland_turns: a drive along the headland more than HEADLAND_SLACK m and HEADLAND_DETOUR times as long as the
# shortest way crosses the lanes' field where that's shorter, along a lane's line (the CROSSINGS closest tried)
HEADLAND_DETOUR = 2.0
HEADLAND_SLACK = 2.0
CROSSINGS = 3
# poses of a segment _Route.mowed() looks at together: it skips those whose box is far from where it looks
MOWED_CHUNK = 32
# m, mower_logic drives to the plan's first pose itself and turns on the spot there unchecked: with the body set the
# plan's first loop starts where a place the body turns whatever way it faces is START_ROOM away at most and its poses
# fit before as far as easing onto its line from there takes (FIT_EASE, where bodyfit's way in gets on) and
# START_CLEAR[0] more, START_CLEAR[1] after (more than bodyfit's FIT_START: a tight place that close it would leave out,
# starting later), at the closest such point to where it would start (the loop all there, the tight place further round,
# driven another way like any other)
START_CLEAR = (0.5, 1.5)
START_ROOM = 1.0


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

    if first is None:
        left, first, way = set(range(n)), [], None
        while left:
            if not first:
                i = min(left, key=lambda i: (min(start[i]), i))
                way = min(range(k[i]), key=lambda a: (start[i][a], a))
            else:
                p = first[-1]
                i, way = min(((j, b) for j in left for b in range(k[j])),
                             key=lambda jb: (links[p, jb[0]][way][jb[1]], jb))
            first.append(i)
            left.remove(i)
    order = list(first)
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


class _Route:
    def __init__(self, cfg: PlannerConfig, transit: TransitPlanner, area: TransitPlanner, start: Optional[XY],
                 heading: Optional[float], end: Optional[XY], footprint: Optional[Footprint] = None):
        self.cfg = cfg
        self.footprint = footprint
        self.body = footprint.poses_fit if footprint is not None else None
        # (a turn between lanes: spin_margin all along it too)
        self.turn_body = (lambda poses: footprint.poses_fit(poses, True)) if footprint is not None else None
        # a turn between lanes rounded as wide as fits where it would turn on the spot (round_earlier): where an arc of
        # one may be (only pivots allowed: on the spot)
        self.rounds = cfg.min_turn_radius > 0 and any(t in cfg.allowed_turns for t in ('u_turn', 'bulb', 'detour'))
        self.transit = transit  # shortest ways, through the drivable areas too
        self.area = area  # turns and loops stay inside the mowing area
        self.step = cfg.waypoint_spacing
        self.segments: List[PathSegment] = []
        self.warnings: List[str] = []
        self.lanes: List[Tuple[XY, XY]] = []
        self.loops: List[List[XY]] = []
        self.counts: Dict[str, int] = {}
        # until the first pose, pos is only where to start looking (the start pose or the outline's first
        # point): the path begins where the first loop does, getting there is up to whoever drives it
        self.pos = start
        self.heading = heading
        self.end = end
        self.started = False
        self.section = 0
        # rings (by id) the centre drives with the blade on their inner side, see centre_rings
        self.blade_in: set = set()
        # with robot_outline: whether the outline was recorded ccw, its outermost loop goes that way (see drive_ring)
        self.recorded: Optional[bool] = None
        # what the segments added now mow or lead to, and where every loop and lane comes from
        self.source = ''
        self.origins: Dict[str, Dict[str, Any]] = {}
        self.cell_of: Dict[int, str] = {}  # lane (by id) -> its cell, 'cell-<set>-<index>' in CoveragePlan.cells
        self._chunks: Dict[int, Tuple[PathSegment, List[Tuple[float, float, float, float]]]] = {}  # see chunks()
        # headland_turns: the lanes' field (and prepared), the transit space but the field, the lanes' direction and the
        # lines they lie on (lane frame y) of every set, the field's pieces on those (see rows())
        self.field = self._field = self.headland = None
        # lane_overlap_limit: where the lanes lie (and prepared), see in_reach
        self.reach = self._reach = None
        self.loop_counts: Optional[Tuple[int, int]] = None  # (obstacle_passes: see left_out)
        self.loop_side: Dict[str, float] = {}  # loop -> 1 driven with the mowable side on its left, -1 on its right
        self.angle = 0.0
        self.lane_ys: Dict[float, List[float]] = {}
        self._rows: Dict[float, List[Tuple[float, List[Tuple[float, float]]]]] = {}
        # headland_corners 'sharp': the points (x, y) of the drives along the headland, 'rounded': of the arcs rounding
        # a corner out of the lanes' field into it (_round_corners)
        self.sharp: set = set()
        self.rounded: set = set()

    def _count(self, key: str) -> None:
        self.counts[key] = self.counts.get(key, 0) + 1

    def add(self, kind: str, poses: List[Pose2D], maneuver: str = '') -> None:
        if not poses:
            return
        self.segments.append(PathSegment(kind, poses, False, maneuver, self.section, self.source))
        last = poses[-1]
        self.pos = (last.x, last.y)
        self.heading = last.yaw
        self.started = True

    def new_section(self) -> None:
        """What comes next is a section of its own: the path jumps there."""
        if self.started:
            self.section += 1
            self.started = False

    def go_to(self, target: XY, heading: Optional[float] = None, head: Optional[XY] = None) -> None:
        """Drives to target, to arrive facing heading if given (head: see drive_to). Where there's no way there (can't
        happen within a part of the area, but never leave anything out) the path goes on there as a new section."""
        if not self.started:
            self.pos = target
            return
        dist = math.hypot(target[0] - self.pos[0], target[1] - self.pos[1])
        if dist < 1e-6:
            return
        poses = self.drive_to(target, heading, head)
        if poses is None and self.area is not self.transit:
            # (a target a little outside the transit space, a lane's end: the way through the mowing area)
            poses = self.drive_to(target, heading, head, self.area)
        if poses is None:
            self.warnings.append(f'no way found from ({self.pos[0]:.2f}, {self.pos[1]:.2f}) to ({target[0]:.2f}, '
                                 f'{target[1]:.2f}): the path jumps there')
            self.new_section()
            self.pos = target
            return
        self.add(TRANSIT, poses)

    def drive_to(self, target: XY, heading: Optional[float], head: Optional[XY] = None,
                 space: Optional[TransitPlanner] = None, start: Optional[Tuple[XY, Optional[float]]] = None
                 ) -> Optional[List[Pose2D]]:
        """The drive from where the mower is (start: from there, facing that way) to target through space (default
        self.transit, with headland_turns headland_way first), arriving facing heading: leaving and arriving on arcs
        where they fit. Where it would arrive the other way round (turning on the spot by more than SPIN_LIMIT there),
        via a U-turn into it from beside it, like between lanes, if that turns on the spot less. Where it still would
        and a lane starts there (straight to head), with min_turn_radius into_lane. With turn_on_spot neither, and on
        the spot where it leaves and arrives where the arcs there would make it cross its own way or turn it round
        more than half a turn (a quarter turn more than on the spot: a pirouette). None if there's no way."""
        if space is None and self.field is not None:
            way = self.headland_way(target, heading, head)
            if way is not None:
                if self.cfg.headland_corners == 'sharp':
                    self.sharp.update((p.x, p.y) for p in way)
                return way
        space = space or self.transit
        pos, facing = start or (self.pos, self.heading)
        r = self.cfg.turn_radius

        def drive(b: XY, h: Optional[float]) -> Optional[List[Pose2D]]:
            return connect(pos, facing, b, h, r, self.step, space.is_free, space.careful_path, self.body,
                           self.cfg.min_turn_radius)

        best = drive(target, heading)
        if best is not None and self.cfg.turn_on_spot:
            # (on the spot where it leaves and arrives where the arcs there make it cross its own way, or turn it round
            # more than half a turn all together where on the spot they turn it a quarter turn less: no pirouettes)
            crosses = len(best) > 3 and not LineString([(p.x, p.y) for p in best]).is_simple
            if crosses or _arcs(best) > math.pi:
                spot = connect(pos, None, target, None, r, self.step, space.is_free, space.careful_path, self.body,
                               self.cfg.min_turn_radius)
                if spot is not None and (crosses or _arcs(spot) < _arcs(best) - 0.5 * math.pi):
                    best = spot
            return best
        if best is None or heading is None or r <= 1e-3:
            return best
        best_spins = _spins(best, facing, heading)
        if not best_spins:
            return best
        nx, ny = -math.sin(heading), math.cos(heading)
        back = normalize_angle(heading + math.pi)
        for d in (2.0 * r, r):
            for side in (1.0, -1.0):
                x = (target[0] + side * d * nx, target[1] + side * d * ny)
                if not space.is_free([x]):
                    continue
                turn = plan_turn(Pose2D(x[0], x[1], back), target, r, self.step, space.is_free, None,
                                 tuple(t for t in self.cfg.allowed_turns if t in ('u_turn', 'bulb')), self.body,
                                 self.cfg.min_turn_radius)
                way = drive(x, back) if turn is not None else None
                if way is None:
                    continue
                poses = join(way, turn.pieces[0].poses)
                spins = _spins(poses, facing, heading)
                if spins < best_spins or spins == best_spins and path_length(poses) < path_length(best):
                    best, best_spins = poses, spins
        if head is not None and self.cfg.min_turn_radius > 0 and angle_diff(best[-1].yaw, heading) > SPIN_LIMIT:
            return self.into_lane(best, target, heading, head, space) or best
        return best

    def into_lane(self, drive: List[Pose2D], a: XY, yaw: float, head: XY,
                  space: TransitPlanner) -> Optional[List[Pose2D]]:
        """In place of the drive's end turning on the spot into the lane from a (facing yaw, its straight piece ending
        at head): Dubins' shortest way into it that fits in space, going round once at most on top of what it has to,
        from a pose of the drive's last 4 turn_radius (one every turn_radius back, the latest first, its start too),
        where none does into the lane ROUND_STEP further on at a up to 2 turn_radius, the first whose way or
        what's mowed already (the loops) mows what it leaves of the lane (but EARLIER_LEFT). None where none fits."""
        r = self.cfg.turn_radius
        starts, way = [], 0.0
        for k in range(len(drive) - 2, -1, -1):
            way += math.hypot(drive[k + 1].x - drive[k].x, drive[k + 1].y - drive[k].y)
            if way >= (len(starts) + 1) * r - 1e-9 or k == 0:
                starts.append(k)
        rest = min(2.0 * r, math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length)
        mowed = []
        for i in range(int(max(0.0, rest) / ROUND_STEP + 1e-9) + 1):
            entry = Pose2D(a[0] + i * ROUND_STEP * math.cos(yaw), a[1] + i * ROUND_STEP * math.sin(yaw), yaw)
            for k in starts[:4]:
                most = angle_diff(drive[k].yaw, yaw) + 2.0 * math.pi + 1e-6
                # (none turning a lot further, give or take how much dubins() rounds their ends)
                for m in dubins(drive[k], entry, r, self.step, turning=most + 0.01):
                    poses = m.pieces[0].poses
                    if sum(angle_diff(p.yaw, q.yaw) for p, q in zip(poses, poses[1:])) > most:
                        continue
                    if not space.is_free([(p.x, p.y) for p in poses]) or self.body is not None and not self.body(poses):
                        continue
                    if i:
                        left = subtract(self.blade([Pose2D(a[0], a[1], yaw), entry]), [
                            self.blade(poses), self.blade([entry, Pose2D(head[0], head[1], yaw)])])
                        if left.area > EARLIER_LEFT:
                            if not mowed:  # (worked out once, as far as it may start later)
                                mowed.append(self.mowed([(a[0], a[1], a[0] + rest * math.cos(yaw),
                                                          a[1] + rest * math.sin(yaw))], self.segments))
                            if subtract(left, mowed).area > EARLIER_LEFT:
                                continue
                    return join(drive[:k], poses)
        return None

    # headland_turns

    def detour_path(self, a: XY, b: XY) -> Optional[List[XY]]:
        """The shortest way a detour between lanes takes, with headland_turns through the headland."""
        return (self.headland or self.transit).path(a, b)

    def turn_space(self, a: XY, reach: bool = True) -> Fits:
        """What a turn from where the mower is into a lane starting at a keeps in: the mowing area, with headland_turns
        out of the lanes' field too where the lanes are more than 4 turn_radius apart (a long U-turn), with
        lane_overlap_limit within the lanes' reach (in_reach, reach False: not that)."""
        (x, y), h = self.pos, self.heading
        ux, uy = math.cos(h), math.sin(h)
        near = self.field is None or abs((a[1] - y) * ux - (a[0] - x) * uy) <= 4.0 * self.cfg.turn_radius
        reach = reach and self._reach is not None
        if near and not reach:
            return self.area.is_free
        # the way turning on the spot takes: straight on level with the lane that ends further out, across there
        on = (a[0] - x) * ux + (a[1] - y) * uy
        spot = [(x, y)]
        for p in ((x + on * ux, y + on * uy) if on >= 0.0 else (a[0] - on * ux, a[1] - on * uy), (a[0], a[1])):
            if p != spot[-1]:  # (a point twice: GEOS 3.11 warns measuring the distance to such a line)
                spot.append(p)

        def fits(pts: Sequence[XY]) -> bool:
            return self.area.is_free(pts) and (near or self.clear(pts)) and (not reach or self.in_reach(pts, spot))

        return fits

    def in_reach(self, pts: Sequence[XY], spot: Sequence[XY]) -> bool:
        """lane_overlap_limit: whether pts (of a turn between two lanes) keep where the lanes lie or on the way turning
        on the spot takes (spot), give or take TURN_REACH. Always without."""
        if self._reach is None:
            return True
        way = LineString(spot) if len(spot) > 1 else Point(spot[0])
        if covers is not None:  # (prep() prepared reach)
            at = points(numpy.asarray(pts, float))
            out = at[~covers(self.reach, at)]
            return not len(out) or bool((distance(way, out) <= TURN_REACH).all())
        if self._reach.covers(LineString(pts) if len(pts) > 1 else Point(pts[0])):
            return True
        return all(self._reach.covers(Point(p)) or way.distance(Point(p)) <= TURN_REACH for p in pts)

    def clear(self, pts: Sequence[XY]) -> bool:
        """Whether pts keep out of the lanes' field (on its edge at most), always without headland_turns."""
        return self._field is None or not self._field.intersects(LineString(pts) if len(pts) > 1 else Point(pts[0]))

    def headland_way(self, target: XY, heading: Optional[float], head: Optional[XY]) -> Optional[List[Pose2D]]:
        """The drive to target (arriving facing heading, a lane to head starting there) turning only out of the lanes'
        field: out of it straight on (from a lane's end), along the headland, into it straight along the next lane.
        Where that's more than HEADLAND_DETOUR times as long as the shortest way (but HEADLAND_SLACK) or there's none,
        crossing() where that's shorter. None where neither fits."""
        if head is not None and not self.transit.is_free([target]):
            # (a lane's start a hair out of the transit space, blade_ahead moves it there: it starts where it gets in)
            piece = self.transit.clip(target, head)
            if piece is None or math.hypot(piece[0][0] - target[0], piece[0][1] - target[1]) > ROUND_STEP:
                return None
            target = piece[0]
        out, into = self.off_field(self.pos, self.heading), self.off_field(target, heading, True)
        if out is None or into is None:
            return None
        (a, lead), (b, tail) = out, into
        way = self.through_headland(a, self.heading, [], b, heading, head)
        # (not longer than that than straight there, it isn't than the shortest way either)
        if way is None or path_length(way) > HEADLAND_SLACK + HEADLAND_DETOUR * math.hypot(b[0] - a[0], b[1] - a[1]):
            short = self.transit.careful_path(a, b) or self.area.careful_path(a, b)
            if short is not None and (way is None or
                                      path_length(way) > HEADLAND_SLACK + HEADLAND_DETOUR * _length(short)):
                across = self.crossing(a, self.heading, short, heading, head)
                if across is not None and (way is None or path_length(across) < path_length(way)):
                    way = across
                    self._count('crossings')
        return None if way is None else join(lead, way, tail)

    def off_field(self, p: XY, yaw: Optional[float], back: bool = False) -> Optional[Tuple[XY, List[Pose2D]]]:
        """Where the line from p facing yaw (back: the other way) leaves the lanes' field, and the poses from p to there
        (back: from there to p) facing yaw; p and none where it's out of it. None where they don't fit."""
        if self.clear([p]):
            return p, []
        if yaw is None:
            return None
        d = -1.0 if back else 1.0
        x0, y0, x1, y1 = self.field.bounds
        far = math.hypot(x1 - x0, y1 - y0)
        line = LineString([p, (p[0] + d * far * math.cos(yaw), p[1] + d * far * math.sin(yaw))])
        cut = self.field.intersection(line)
        piece = min((g for g in getattr(cut, 'geoms', [cut]) if g.geom_type == 'LineString' and not g.is_empty),
                    key=lambda g: g.distance(Point(p)), default=None)  # (the one from p)
        if piece is None:
            return None
        q = max(piece.coords, key=lambda c: math.hypot(c[0] - p[0], c[1] - p[1]))[:2]
        poses = sample_straight(q[0], q[1], p[0], p[1], yaw, self.step) if back else \
            sample_straight(p[0], p[1], q[0], q[1], yaw, self.step)
        if not self.transit.is_free([p, q]) or self.body is not None and not self.body(poses):
            return None
        return q, poses

    def through_headland(self, a: XY, ha: Optional[float], pieces: List[Tuple[XY, XY]], b: XY, hb: Optional[float],
                         head: Optional[XY]) -> Optional[List[Pose2D]]:
        """From a (facing ha) to b (arriving facing hb, a lane to head starting there) through the headland
        (drive_to()), but for the pieces [(p, q)] on the way straight across the lanes' field. None where that doesn't
        fit."""
        way, h = [], ha
        for p, q in pieces + [(b, None)]:
            yaw = hb if q is None else math.atan2(q[1] - p[1], q[0] - p[0])
            line = [] if q is None else sample_straight(p[0], p[1], q[0], q[1], yaw, self.step)
            if line and (not self.transit.is_free([p, q]) or self.body is not None and not self.body(line)):
                return None
            there = self.drive_to(p, yaw, head if q is None else None, self.headland, (a, h))
            if there is None:
                return None
            way, a, h = join(way, there, line), q, yaw
        return way

    def crossing(self, a: XY, ha: Optional[float], short: List[XY], hb: Optional[float],
                 head: Optional[XY]) -> Optional[List[Pose2D]]:
        """From a (facing ha) along short (the shortest way to its end, arriving facing hb), across the lanes' field
        only straight: where short crosses it once along a lane's line (of the CROSSINGS lines across short there
        closest to straight there and on the shortest way that fits) where that's not more than HEADLAND_SLACK and
        HEADLAND_DETOUR times as long as straight where short crosses it, else that. None where neither fits."""
        line = LineString(short)
        cut = self.field.intersection(line)
        pieces = sorted((sorted((g.coords[0][:2], g.coords[-1][:2]), key=lambda e: line.project(Point(e)))
                         for g in getattr(cut, 'geoms', [cut]) if g.geom_type == 'LineString' and g.length > 1e-6),
                        key=lambda piece: line.project(Point(piece[0])))
        if not pieces:  # (that's the headland's way)
            return None
        found = []
        if len(pieces) == 1:
            (e0, e1), c, s = pieces[0], math.cos(self.angle), math.sin(self.angle)
            (u0, v0), (u1, v1) = ((x * c + y * s, -x * s + y * c) for x, y in (e0, e1))
            rows, near = self.rows(), self.cfg.lane_spacing  # (the lines a lane spacing on either side too)
            ys = [y for y, _ in rows]
            for y, row in rows[bisect.bisect_left(ys, min(v0, v1) - near):
                               bisect.bisect_right(ys, max(v0, v1) + near)]:
                x = u0 + (y - v0) * (u1 - u0) / (v1 - v0) if abs(v1 - v0) > 1e-9 else 0.5 * (u0 + u1)
                for x0, x1 in row:
                    if x0 - 1e-6 <= x <= x1 + 1e-6:
                        p, q = (x0, x1) if u1 >= u0 else (x1, x0)
                        p, q = (p * c - y * s, p * s + y * c), (q * c - y * s, q * s + y * c)
                        found.append((math.hypot(p[0] - e0[0], p[1] - e0[1]) + abs(x1 - x0) +
                                      math.hypot(e1[0] - q[0], e1[1] - q[1]), p, q))
        best = self.through_headland(a, ha, pieces, short[-1], hb, head)
        most = math.inf if best is None else HEADLAND_SLACK + HEADLAND_DETOUR * path_length(best)
        for _, p, q in sorted(found)[:CROSSINGS]:
            way = self.through_headland(a, ha, [(p, q)], short[-1], hb, head)
            if way is not None and path_length(way) <= most:
                best, most = way, path_length(way)
        return best

    def rows(self) -> List[Tuple[float, List[Tuple[float, float]]]]:
        """The lanes' field on the lines the lanes (at self.angle) lie on: [(y, [(x0, x1)] along them)] in the lane
        frame, worked out once."""
        if self.angle not in self._rows:
            ys = self.lane_ys.get(self.angle, [])
            scan = _Scanner(_ring_list(self.field), math.cos(self.angle), math.sin(self.angle))
            self._rows[self.angle] = list(zip(ys, scan.at(ys)))
        return self._rows[self.angle]

    def _distance(self, a: Optional[XY], b: XY) -> float:
        return 0.0 if a is None else math.hypot(b[0] - a[0], b[1] - a[1])

    # loops

    def centre_rings(self, geom) -> List[Tuple[str, List[XY]]]:
        """The rings of geom as rings() gives them, for the mower's centre to drive so the blade mows along
        geom's: with blade_offset the centre runs that much further in (the blade on the outer side). Around the
        parts of geom narrower than twice the offset (a strip, the thin end of a loop) it runs that much further
        out, with the blade on the inner side, as far as it may go there."""
        off = abs(self.cfg.blade_offset)
        if off == 0:
            return rings(geom)
        half = 0.5 * self.cfg.mower_width
        core = inset(geom, off)
        found = rings(core)
        thin = geom if core.is_empty else polygonal(geom.difference(core.buffer(off, QUAD_SEGS)))
        for part in polygons(thin):
            if part.area < half * half:  # the tips of sharp corners, the loops round them anyway
                continue
            for kind, ring in rings(inset(part, -off)):
                if self.area.is_free(ring + ring[:1]):
                    found.append((kind, ring))
                    self.blade_in.add(id(ring))
        return found

    def left_out(self, kind: str, depth: float) -> bool:
        """Whether the ring ('outline' or 'hole') of loop level depth isn't driven: with obstacle_passes (loop_counts:
        how many along the outline and round the islands) past its side's number, where it only says where that side's
        lanes begin."""
        if self.loop_counts is None or depth != int(depth) or depth >= max(self.loop_counts):
            return False
        return depth >= self.loop_counts[1 if kind == 'hole' else 0]

    def _ring_distance(self, ring: List[XY]) -> float:
        if self.pos is None:
            return 0.0
        return closest_on_polyline(ring + [ring[0]], self.pos)[2]

    def _end_distance(self, ring: List[XY]) -> float:
        return closest_on_polyline(ring + [ring[0]], self.end)[2]

    def _start_at(self, poses: List[Pose2D], target: Optional[XY]) -> Optional[XY]:
        """Where the plan's first loop (its poses round from where it would start) starts instead (see START_CLEAR):
        the closest pose to target where a place the body turns whatever way it faces is START_ROOM away at most and
        they fit before and after as far as that says. None: where it would (that's one, or there's none)."""
        fp = self.footprint
        room = fp.turning.anywhere.intersection(self.transit.space)
        n = len(poses)
        if room.is_empty or n < 3:
            return None
        bad = [not f for f in fp.fit(poses)]
        # (m of the way round at each pose, three times round: the poses before and after one of the middle round are
        # there. The last pose is the first again)
        at = [0.0]
        for a, b in zip(poses, poses[1:]):
            at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
        whole = at[-1]
        if whole <= 2.0 * sum(START_CLEAR):
            return None
        rounds = at[:-1] + [whole + s for s in at[:-1]] + [2.0 * whole + s for s in at[:-1]]
        ahead = [0]  # bad poses up to each of rounds
        for k in range(len(rounds)):
            ahead.append(ahead[-1] + bad[k % (n - 1)])
        if covers is not None:
            off = [float(d) for d in distance(room, points([(p.x, p.y) for p in poses]))]
        else:
            off = [room.distance(Point(p.x, p.y)) for p in poses]

        def good(j: int) -> bool:
            if off[j] > START_ROOM:
                return False
            s = at[j] + whole
            back = FIT_EASE * fp.front * math.log1p(off[j] / FIT_EASE_OFF) + START_CLEAR[0]
            a = bisect.bisect_left(rounds, s - back)
            b = bisect.bisect_right(rounds, s + START_CLEAR[1])
            return ahead[b] - ahead[a] == 0

        if good(0):
            return None
        fine = [j for j in range(n - 1) if good(j)]
        if not fine:
            return None
        if target is None:
            j = fine[0]
        else:
            j = min(fine, key=lambda j: (math.hypot(poses[j].x - target[0], poses[j].y - target[1]), j))
        return poses[j].x, poses[j].y

    def drive_ring(self, ring: List[XY], hole: bool, kind: str = PERIMETER, maneuver: str = '',
                   target: Optional[XY] = None, level: int = 0, spacing: Optional[float] = None,
                   radius: Optional[float] = None, parent: Optional[str] = None) -> None:
        """One loop. ring comes as rings() gives it (outline ccw, hole cw: the mowable side on the left).
        It starts closest to target (default: a little ahead of the mower; the plan's first with the body set where
        the mower gets onto it, see START_CLEAR). Its corners are rounded with
        perimeter_corner_radius (or radius), less the further in it is (`level` loops `spacing` apart), so
        no tuft stays between the loops (but for a few cm2 at a corner of the outline made of short edges,
        see inner_radius)."""
        radius = self.cfg.perimeter_corner_radius if radius is None else radius
        spacing = self.cfg.lane_spacing if spacing is None else spacing
        closed = ring + [ring[0]]
        if target is None:
            target = self.pos
            if target is not None and self.heading is not None and self.started:
                # aim a little ahead: from one loop to the next that's a slanted hop, not two right angles
                lead = 2.0 * self.cfg.lane_spacing
                target = (target[0] + lead * math.cos(self.heading), target[1] + lead * math.sin(self.heading))
        i, entry, _ = (0, closed[0], 0.0) if target is None else closest_on_polyline(closed, target)
        entry = _clear_of_corners(closed, i, entry, 2.0 * radius)
        loop = [entry] + closed[i + 1:] + closed[1:i + 1] + [entry]
        direction = self.cfg.perimeter_direction
        if direction == 'random':  # (the same for each count of finished mows, on Python 3.8 as on 3.13)
            direction = 'cw' if random.Random(self.cfg.finished_mows).random() < 0.5 else 'ccw'
        if self.cfg.blade_offset != 0:
            # rings come with the mowable side on the left: the blade goes on the other side, but for the rings
            # driven with the blade on the inner one
            flip = (self.cfg.blade_offset > 0) != (id(ring) in self.blade_in)
        elif self.recorded is not None and self.cfg.perimeter_direction in ('auto', 'random') and kind == PERIMETER \
                and not hole and level == 0:
            # the outermost loop the way the outline was recorded: the body where it was then (its real edges), the
            # front nowhere else (rings come ccw)
            flip = not self.recorded
        elif direction == 'auto':
            flip = False
            # the start pose's heading counts too: no half turn on the spot before the first loop
            if self.heading is not None:
                # go round the way the mower already faces
                fwd = _first_direction(loop)
                back = _first_direction(loop[::-1])
                h = (math.cos(self.heading), math.sin(self.heading))
                flip = fwd is not None and back is not None and \
                    h[0] * back[0] + h[1] * back[1] > h[0] * fwd[0] + h[1] * fwd[1] + 1e-9
        else:
            # outlines come ccw, holes cw
            flip = (direction == 'cw') != hole
        if flip:
            loop = loop[::-1]

        def driven(loop: List[XY]) -> List[Pose2D]:
            # with the mowable side on the left only left turns go round a corner of it and get rounded
            # (right turns too: the arc cuts their corner on the side mowed already or keeps off the edge where it
            # doesn't fit)
            return drive_polyline(loop, self.step, radius, self.area.is_free, 0, CORNER_CUT * self.cfg.mower_width,
                                  level, spacing, self.cfg.mower_width, self.body)

        poses = driven(loop)
        if not self.segments and self.footprint is not None:
            # the plan's first loop: where mower_logic gets to it on its own (see START_CLEAR), the same way round
            better = self._start_at(poses, target)
            if better is not None:
                i, entry, _ = closest_on_polyline(closed, better)
                entry = _clear_of_corners(closed, i, entry, 2.0 * radius)
                loop = [entry] + closed[i + 1:] + closed[1:i + 1] + [entry]
                if flip:
                    loop = loop[::-1]
                poses = driven(loop)
        first = _first_direction(loop)
        name = f'loop-{len(self.loops)}'
        self.source = name
        self.origins[name] = {'parent': parent or f'level-{level}', 'kind': 'obstacle' if hole else 'outline',
                              'level': level}
        self.go_to(entry, math.atan2(first[1], first[0]) if first else None)
        self.add(kind, poses, maneuver=maneuver)
        self.loops.append([(p.x, p.y) for p in poses])
        self.loop_side[name] = -1.0 if flip else 1.0

    def loops_in(self, places: Sequence[Sequence[float]]) -> Dict[int, List[Pose2D]]:
        """The loops further in round those places (loop_in), their corners too tight for that rounded (loop_round):
        [x, y, m], where the body sticks out on the path and m of it are left out for that. Their segments get the new
        poses, returns the poses they had (by segment)."""
        before: Dict[int, List[Pose2D]] = {}
        for k, seg in enumerate(self.segments):
            side = self.loop_side.get(seg.source) if seg.kind == PERIMETER else None
            if side is None or len(seg.poses) < 3:
                continue
            at = [0.0]
            for a, b in zip(seg.poses, seg.poses[1:]):
                at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
            # (a place is given to the cm)
            near = [(at[i], m, (x, y)) for x, y, m in places for i, p in enumerate(seg.poses)
                    if abs(p.x - x) < 0.006 and abs(p.y - y) < 0.006]
            if not near:
                continue
            poses = self.loop_in(seg.poses, side, lambda i: any(abs(at[i] - d) <= m for d, m, _ in near))
            poses = self.loop_round(poses, [(xy, m) for _, m, xy in near])
            if poses is not seg.poses:
                before[k], seg.poses = seg.poses, poses
                self.loops[int(seg.source.split('-')[1])] = [(p.x, p.y) for p in poses]
        return before

    def loops_on(self, ends: Dict[str, float]) -> Dict[int, List[Pose2D]]:
        """The loops (by name) on past their start by those m, along themselves (a loop ends where it starts): what's
        left out where it gets onto one is mowed at its end, what's left out where it gets off it was at its start. The
        drive from there to the next loop anew, arriving the way that one starts (none where something else comes next
        in its section, or where with turn_on_spot off that drive turns on the spot by more than SPIN_LIMIT). Their
        segments get the new poses, returns the poses they had (by segment)."""
        before: Dict[int, List[Pose2D]] = {}
        for k, seg in enumerate(self.segments):
            by = ends.get(seg.source) if seg.kind == PERIMETER else None
            if not by or len(seg.poses) < 3 or not same_position(seg.poses[0], seg.poses[-1]):
                continue
            at = [0.0]
            for a, b in zip(seg.poses, seg.poses[1:]):
                at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
            # (the loop itself: after an omega in place of turning onto it, see _smooth_spins)
            first = max(i for i in range(len(seg.poses) - 1)
                        if same_position(seg.poses[i], seg.poses[0]) and at[i] < 0.5 * at[-1])
            if by >= at[-1] - at[first]:
                continue
            more = [p for i, p in enumerate(seg.poses) if i > first and at[i - 1] - at[first] < by]
            poses = seg.poses + more
            nxt = self.segments[k + 1:k + 3]
            if nxt and nxt[0].section == seg.section:
                if len(nxt) < 2 or nxt[0].kind != TRANSIT or nxt[1].kind != PERIMETER or \
                        nxt[1].source not in self.loop_side:
                    continue
                drive, after = nxt
                way = self.drive_from(poses[-1], (drive.poses[-1].x, drive.poses[-1].y), after.poses[0].yaw)
                if way is None or not self.cfg.turn_on_spot and _spins(way, poses[-1].yaw, after.poses[0].yaw):
                    continue
                before[k + 1], drive.poses = drive.poses, way
            before[k], seg.poses = seg.poses, poses
            self.loops[int(seg.source.split('-')[1])] = [(p.x, p.y) for p in poses]
        return before

    def drive_from(self, start: Pose2D, target: XY, heading: Optional[float]) -> Optional[List[Pose2D]]:
        """drive_to from start (facing its way) instead of from where the mower is."""
        here = self.pos, self.heading
        self.pos, self.heading = (start.x, start.y), start.yaw
        try:
            return self.drive_to(target, heading)
        finally:
            self.pos, self.heading = here

    def loops_back(self, before: Dict[int, List[Pose2D]]) -> None:
        """The loops (and the drives after them) as they were before loops_in or loops_on."""
        for k, poses in before.items():
            seg = self.segments[k]
            seg.poses = poses
            if seg.kind == PERIMETER:
                self.loops[int(seg.source.split('-')[1])] = [(p.x, p.y) for p in poses]

    def loop_in(self, poses: List[Pose2D], side: float, here=None) -> List[Pose2D]:
        """With the body checked: where it would stick out on a loop (round a bend of the edge the front swings out),
        that stretch runs further in, as little as it takes (see LOOP_IN), so the blade still mows along the edge there
        (side: 1 with the mowable side on the left, -1 on the right; here: only stretches with a pose it says so for,
        by index). Not at the loop's ends, where it turns on the spot or where a pose fits no further in either: what
        still sticks out, bodyfit._fit_body drives another way. poses itself where nothing is moved."""
        fp = self.footprint
        if fp is None or self.cfg.blade_offset != 0 or len(poses) < 3:
            return poses
        fit = fp.fit(poses)
        if all(fit):
            return poses
        n, (step, most) = len(poses), LOOP_IN
        at = [0.0]
        for a, b in zip(poses, poses[1:]):
            at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
        # across the way the loop goes at each pose (over the poses next to it: at a bend halfway), to the mowable side
        normal = []
        for i in range(n):
            a, b = poses[max(0, i - 1)], poses[min(n - 1, i + 1)]
            d = math.hypot(b.x - a.x, b.y - a.y) or 1.0
            normal.append((-side * (b.y - a.y) / d, side * (b.x - a.x) / d))
        # how far in each pose that sticks out fits, facing the way it does
        need: Dict[int, float] = {}
        left, by = [i for i in range(n) if not fit[i]], step
        while left and by <= most + 1e-9:
            ok = fp.fit([Pose2D(poses[i].x + by * normal[i][0], poses[i].y + by * normal[i][1], poses[i].yaw)
                         for i in left])
            need.update((i, by) for i, f in zip(left, ok) if f)
            left, by = [i for i, f in zip(left, ok) if not f], by + step
        # what stays where it is: the loop's ends, where it turns on the spot, the poses that fit no further in either
        stay = sorted({0, n - 1, *left, *(k for i in range(1, n) if angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN
                                          for k in (i - 1, i))})
        for i in stay:
            need.pop(i, None)
        tries = [0]

        def spins(path: Sequence[Pose2D]) -> int:
            return sum(1 for a, b in zip(path, path[1:]) if angle_diff(a.yaw, b.yaw) > SPIN)

        def moved(j0: int, i0: int, i1: int, j1: int, by: float) -> Optional[List[Pose2D]]:
            # poses j0..j1 with i0..i1 that far in, the others on the way there and back, each facing the next (the
            # last as it did). Without the ones that would go backwards: on the inside of a bend they get closer
            pts: List[Tuple[int, float, float]] = []
            for j in range(j0, j1 + 1):
                t = (at[j] - at[j0]) / (at[i0] - at[j0]) if j < i0 else \
                    (at[j1] - at[j]) / (at[j1] - at[i1]) if j > i1 else 1.0
                o, p = by * t * t * (3.0 - 2.0 * t), poses[j]
                x, y = p.x + o * normal[j][0], p.y + o * normal[j][1]
                if pts:
                    k, u, v = pts[-1]
                    if (x - u) * (p.x - poses[k].x) + (y - v) * (p.y - poses[k].y) <= 0.0:
                        continue
                pts.append((j, x, y))
            # (step apart at most: on the outside of a bend they get further apart)
            new = join(*(sample_straight(x, y, u, v, math.atan2(v - y, u - x), self.step)[:-1]
                         for (_, x, y), (_, u, v) in zip(pts, pts[1:])), [poses[j1]])
            if spins(poses[j0 - 1:j0] + new) > spins(poses[j0 - 1:j1 + 1]) or not fp.all_fit(new) or \
                    not self.area.is_free([(p.x, p.y) for p in new]):
                return None
            return new

        def further_in(i0: int, i1: int) -> Optional[Tuple[int, int, List[Pose2D]]]:
            # the stretch i0..i1 as little further in as fits, over the shortest way there and back, between the poses
            # that stay and what's moved already: (the first and the last pose it replaces, the poses in their place)
            lo, hi = stay[bisect.bisect_left(stay, i0) - 1], stay[bisect.bisect_left(stay, i1)]
            for k in range(int(round(max(need[i] for i in range(i0, i1 + 1) if i in need) / step)),
                           int(round(most / step)) + 1):
                for ease in LOOP_EASE:
                    j0, j1 = bisect.bisect_left(at, at[i0] - ease), bisect.bisect_right(at, at[i1] + ease) - 1
                    if max(lo, done - 1) < j0 < i0 and i1 < j1 < hi and tries[0] < LOOP_TRIES:
                        tries[0] += 1
                        new = moved(j0, i0, i1, j1, k * step)
                        if new is not None:
                            return j0, j1, new
            return None

        # the stretches: poses that don't fit one after the other
        runs: List[List[int]] = []
        for i in sorted(need):
            if runs and i == runs[-1][1] + 1:
                runs[-1][1] = i
            else:
                runs.append([i, i])
        if here is not None:
            runs = [run for run in runs if any(here(i) for i in range(run[0], run[1] + 1))]
        out: List[Pose2D] = []
        done, k = 0, 0  # (done: the poses before it are in out)
        while k < len(runs):
            # the ones close after it with nothing between them that stays: their ways in and out would meet
            end = k
            while end + 1 < len(runs) and at[runs[end + 1][0]] - at[runs[end][1]] <= 2.0 * LOOP_EASE[0] and \
                    bisect.bisect_left(stay, runs[end + 1][0]) == bisect.bisect_left(stay, runs[end][1]):
                end += 1
            got = None
            for last in list(range(k, min(end, k + LOOP_JOIN) + 1)) + [end] * (end > k + LOOP_JOIN):
                got = further_in(runs[k][0], runs[last][1])
                if got is not None:
                    break
            if got is None:
                k += 1
                continue
            j0, j1, new = got
            out += poses[done:j0] + new
            done, k = j1 + 1, last + 1
            self._count('loops_in')
        return out + poses[done:] if done else poses

    def loop_round(self, poses: List[Pose2D], places: Sequence[Tuple[XY, float]]) -> List[Pose2D]:
        """With the body checked: where a stretch of a loop still sticks out within m of a place ((x, y), m), a corner
        too tight for the body (its front swings out further than loop_in takes it in), Dubins' way from the loop's
        way before it onto its way after it in its place (see LOOP_ROUND): the corner rounded off, the loop on along
        the edge on both sides. Not over the loop's ends or where it turns on the spot. poses itself where nothing
        changes."""
        fp, n = self.footprint, len(poses)
        if fp is None or n < 3:
            return poses
        fit = fp.fit(poses)
        at = [0.0]
        for a, b in zip(poses, poses[1:]):
            at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
        runs: List[List[int]] = []
        for i in range(1, n - 1):
            if fit[i]:
                continue
            if runs and at[i] - at[runs[-1][1]] <= LOOP_ROUND_GAP:
                runs[-1][1] = i
            else:
                runs.append([i, i])
        runs = [run for run in runs if any(math.hypot(poses[i].x - x, poses[i].y - y) <= m
                                           for i in range(run[0], run[1] + 1) for (x, y), m in places)]
        if not runs:
            return poses
        spins = [i for i in range(1, n) if same_position(poses[i - 1], poses[i]) and
                 angle_diff(poses[i].yaw, poses[i - 1].yaw) > SPIN]
        out: List[Pose2D] = []
        done = 0  # (the poses before it are in out)
        for i0, i1 in runs:
            if i0 <= done:
                continue
            # (from where to where: what it leaves of the loop, the least first)
            ends = sorted((at[i0] - at[j0] + at[j1] - at[i1], j0, j1)
                          for j0 in range(i0 - 1, max(done, 1) - 1, -1)
                          if fit[j0] and at[i0] - at[j0] <= LOOP_ROUND_REACH
                          for j1 in range(i1 + 1, n - 1) if fit[j1] and at[j1] - at[i1] <= LOOP_ROUND_REACH and
                          not any(j0 < k <= j1 for k in spins))
            got, tries = None, 0
            for _, j0, j1 in ends:
                turn = sum(angle_diff(poses[k].yaw, poses[k + 1].yaw) for k in range(j0, j1))
                for r in LOOP_ROUND:
                    tries += 1
                    m = dubins(poses[j0], poses[j1], r, self.step, loops=False, turning=turn + 0.5 * math.pi)[:1]
                    way = join(*(piece.poses for piece in m[0].pieces)) if m else None
                    if way and fp.all_fit(way) and self.area.is_free([(p.x, p.y) for p in way]):
                        got = j0, j1, way
                        break
                if got is not None or tries >= LOOP_ROUND_TRIES:
                    break
            if got is None:
                continue
            j0, j1, way = got
            out += poses[done:j0] + way
            done = j1 + 1
            self._count('loops_rounded')
        return out + poses[done:] if done else poses

    def perimeter(self, levels: List[object], base_holes: List[List[XY]], last: bool = False,
                  finish: bool = False) -> None:
        """The loops along the outline (level by level) and around the obstacles (each from its outermost
        loop in towards it, closest obstacle first, like OpenMower). last: the obstacles first, then the
        outline from the inside out, ending on the outermost loop. finish (nothing comes after the loops):
        of the last level's loops (or obstacles) the one closest to the end point comes last and ends as close
        to it as it gets, the obstacles go first if that's closer."""
        hole_polys = [Polygon(h) for h in base_holes]
        hole_marks = [p.representative_point() for p in hole_polys]
        outlines: List[Tuple[int, List[List[XY]]]] = []
        groups: Dict[int, List[Tuple[int, List[XY]]]] = {}
        spare = len(hole_polys)
        for k, level in enumerate(levels):
            level_rings = [(kind, ring) for kind, ring in self.centre_rings(level) if not self.left_out(kind, k)]
            outlines.append((k, [ring for kind, ring in level_rings if kind == 'outline']))
            for kind, ring in level_rings:
                if kind != 'hole':
                    continue
                poly = Polygon(ring)
                # every hole of a shrunk area goes around at least one obstacle, loops around the same one
                # are mowed one after another
                key = next((i for i, m in enumerate(hole_marks) if poly.contains(m)), None)
                if key is None:
                    key, spare = spare, spare + 1
                groups.setdefault(key, []).append((k, ring))
        # levels of the other parts of the area are empty here
        outlines = [] if self.cfg.skip_outline_loops else [(k, found) for k, found in outlines if found]
        if self.cfg.skip_obstacle_loops:
            groups = {}
        finish = finish and self.end is not None
        # nothing after the loops, and the innermost loop comes closer to the end point than the loops around
        # the obstacles: those go first
        obstacles_first = finish and not last and outlines and groups and \
            min(map(self._end_distance, outlines[-1][1])) < min(self._end_distance(g[0][1]) for g in groups.values())

        def outline_loops(finish: bool):
            for li, (k, level_rings) in enumerate(outlines[::-1] if last else outlines):
                todo = list(level_rings)
                final = finish and li == len(outlines) - 1
                keep = min(todo, key=self._end_distance, default=None) if final else None
                while todo:
                    ring = min([r for r in todo if r is not keep] or todo, key=self._ring_distance)
                    todo.remove(ring)
                    self.drive_ring(ring, False, target=self.end if final and not todo else None, level=k)

        def obstacle_loops(finish: bool):
            todo_groups = [group[::-1] for group in groups.values()]
            keep = min(todo_groups, key=lambda g: self._end_distance(g[-1][1]), default=None) if finish else None
            while todo_groups:
                group = min([g for g in todo_groups if g is not keep] or todo_groups,
                            key=lambda g: self._ring_distance(g[0][1]))
                todo_groups.remove(group)
                for i, (k, ring) in enumerate(group):
                    final = group is keep and i == len(group) - 1
                    self.drive_ring(ring, True, target=self.end if final else None, level=k)

        if last or obstacles_first:
            obstacle_loops(finish and not outlines)
            outline_loops(finish)
        else:
            outline_loops(finish and not groups)
            obstacle_loops(finish)

    def extra_loops(self, geom, finish: bool = False) -> None:
        """Loops around the bits of the area no lane gets to (strips too narrow for one), closest first.
        finish: the one closest to the end point last, ending as close to it as it gets."""
        todo = [(kind == 'hole', ring) for kind, ring in self.centre_rings(geom)]
        keep = min(todo, key=lambda t: self._end_distance(t[1]), default=None) \
            if finish and self.end is not None else None
        while todo:
            item = min([t for t in todo if t is not keep] or todo, key=lambda t: self._ring_distance(t[1]))
            todo.remove(item)
            self.drive_ring(item[1], item[0], LANE, 'ring', target=self.end if item is keep else None, radius=0.0,
                            parent='narrow')

    def concentric(self, levels: List[object], middles: List[object], perimeter: int, last: bool,
                   spacing: float, finish: bool = True, spliced: bool = False) -> None:
        """Every loop of every level (and the loops halfway in where a part ends, middles[k] after level k).
        A loop is only mowed after the loops further out next to it (with last: further in), and the closest
        of the ready ones is next: so each part is mowed as a spiral, in (or out with last), before the mower
        moves on. Loops of the first `perimeter` levels count as perimeter loops, the rest as lanes. finish: they
        end the plan (else the one closest to the end point isn't kept for last). spliced: every loop further in is
        driven from the loop closest to it on the way round (in, round, back out where it left), so a part
        of loops is one drive without trips between them (the loops in a narrow part lie scattered along it)."""
        loops = []  # (depth, hole, ring, line), depth k for level k, k + 0.5 for middles[k]
        for depth, geom in [(k, g) for k, g in enumerate(levels)] + [(k + 0.5, g) for k, g in enumerate(middles)]:
            for kind, ring in self.centre_rings(geom):
                if depth < perimeter:
                    skip = self.cfg.skip_obstacle_loops if kind == 'hole' else self.cfg.skip_outline_loops
                    skip = skip or self.left_out(kind, depth)
                else:
                    skip = self.cfg.skip_lanes
                if not skip:
                    loops.append((depth, kind == 'hole', ring, LineString(ring + [ring[0]])))
        loops.sort(key=lambda loop: loop[0])
        if spliced:
            # from the innermost out, each loop into the closest loop further out (of those about as close the one
            # furthest in: not across the loops between), not into the outermost ones: going in and out there
            # would be at the very edge
            items = [[depth, hole, list(ring), line] for depth, hole, ring, line in loops]
            for item in sorted(items, key=lambda it: -it[0]):
                outer = [o for o in items if 0 < o[0] < item[0] and o[2] is not None]
                if not outer:
                    continue
                gaps = [o[3].distance(item[3]) for o in outer]
                closest = min(gaps)
                parent = max((o for o, g in zip(outer, gaps) if g <= closest + 0.5 * spacing), key=lambda o: o[0])
                # (the next closest where the way over to that one crosses a gap closed between obstacles, see _gaps;
                # none: a loop of its own)
                for o in [parent] + [o for _, o in sorted(zip(gaps, outer), key=lambda go: go[0]) if o is not parent]:
                    ring = _splice(o[2], item[2], spacing)
                    if self.area.is_free(ring + ring[:1]):
                        o[2], o[3], item[2] = ring, LineString(ring + ring[:1]), None
                        break
            loops = [tuple(it) for it in items if it[2] is not None]
        near = 1.5 * self.cfg.lane_spacing
        waits = []
        for depth, _, _, line in loops:
            before = (lambda d: depth < d <= depth + 1) if last else (lambda d: depth - 1 <= d < depth)
            waits.append({j for j, (dj, _, _, other) in enumerate(loops)
                          if before(dj) and line.distance(other) <= near})
        # with an end point, of the loops nothing waits for (going out the outermost ones, going in the
        # innermost ones) the one closest to it comes last
        reserved = None
        if self.end is not None and finish:
            end, waited = Point(self.end), set().union(*waits)
            reserved = min((j for j in range(len(loops)) if j not in waited), default=None,
                           key=lambda j: (loops[j][3].distance(end), j))
        done = set()
        while len(done) < len(loops):
            ready = [i for i in range(len(loops)) if i not in done and i != reserved and waits[i] <= done]
            if not ready:  # only the reserved one left (or, can't happen, levels that don't fit together)
                ready = [i for i in range(len(loops)) if i not in done and i != reserved] or [reserved]
            i = min(ready, key=lambda j: (self._ring_distance(loops[j][2]), j))
            done.add(i)
            self._ring(loops[i], perimeter, spacing, len(done) == len(loops) and self.end is not None)

    def _ring(self, loop, perimeter: int, spacing: float, final: bool) -> None:
        depth, hole, ring, _ = loop
        # a middle loop (depth k + 0.5) is half a spacing further in, rounded like the next level's
        level = int(math.ceil(depth))
        parent = f'level-{depth}' if depth == level else f'middle-{int(depth)}'
        if depth < perimeter:
            self.drive_ring(ring, hole, target=self.end if final else None, level=level, spacing=spacing,
                            parent=parent)
        else:
            self.drive_ring(ring, hole, LANE, 'ring', target=self.end if final else None, level=level,
                            spacing=spacing, parent=parent)

    # lanes

    def cells(self, cells: List[List[Lane]], angle: float, finish: bool = False) -> None:
        """Mows the cells, the closest one not mowed yet next. finish: the cell that can end closest to the
        end point goes last."""
        c, s = math.cos(angle), math.sin(angle)
        self.angle = angle

        def to_map(x: float, y: float) -> XY:
            return x * c - y * s, x * s + y * c

        def options(cell: List[Lane]):
            # first or last row, starting from either end: (order of lanes, first direction, entry, exit)
            for from_last in (False, True):
                lanes = self.visit_order(cell[::-1] if from_last else cell)
                for direction in (1, -1):
                    first = lanes[0]
                    out = direction if len(lanes) % 2 else -direction
                    entry = to_map(first.x0 if direction > 0 else first.x1, first.y)
                    exit_ = to_map(lanes[-1].x1 if out > 0 else lanes[-1].x0, lanes[-1].y)
                    yield lanes, direction, entry, exit_

        def turning(direction: int) -> float:
            # turning round to the first lane costs about as much as driving the turn's arc
            if self.heading is None:
                return 0.0
            return self.cfg.turn_radius * angle_diff(angle if direction > 0 else angle + math.pi, self.heading)

        pending = [list(cell) for cell in cells if cell]
        if self.cfg.route_order == 'optimized' and len(pending) > 1:
            pending = self.optimized_cells(pending, options, angle, finish and self.end is not None, to_map)
            finish = False
        reserved = None
        if finish and self.end is not None and pending:
            ri = min(range(len(pending)),
                     key=lambda i: min(self._distance(self.end, o[3]) for o in options(pending[i])))
            reserved = pending.pop(ri)
        while pending or reserved:
            if not pending:
                pending, reserved, last = [reserved], None, True
            else:
                last = False
            best = None
            for ci, cell in enumerate(pending):
                for lanes, direction, entry, exit_ in options(cell):
                    cost = self._distance(self.pos, entry) + turning(direction)
                    if last:
                        # the last cell ends as close to the end point as it can, getting to it comes second
                        cost += 1000.0 * self._distance(self.end, exit_)
                    if best is None or cost < best[0] - 1e-9:
                        best = (cost, ci, lanes, direction)
            _, ci, lanes, direction = best
            pending.pop(ci)
            rest = self.drive_lanes(lanes, direction, angle, to_map)
            if rest:
                pending.append(sorted(rest, key=lambda lane: lane.row))

    def optimized_cells(self, cells: List[List[Lane]], options, angle: float, finish: bool,
                        to_map) -> List[List[Lane]]:
        """Mows the cells in the order best_order() finds, drives estimated as the straight distance plus
        turning (turn_radius times the angle), with finish the last one ending as close to the end point as it
        can. Returns the lanes it couldn't get to, for the nearest-first order to pick up."""
        r = self.cfg.turn_radius
        opts = [list(options(cell)) for cell in cells]

        def heading(direction: int) -> float:
            return angle if direction > 0 else angle + math.pi

        def out(o) -> float:  # the way the last lane goes
            return heading(o[1] if len(o[0]) % 2 else -o[1])

        start = [[self._distance(self.pos, o[2]) + (0.0 if self.heading is None else
                                                    r * angle_diff(heading(o[1]), self.heading)) for o in os]
                 for os in opts]
        end = [[1000.0 * self._distance(self.end, o[3]) if finish else 0.0 for o in os] for os in opts]

        def link(i: int, a: int, j: int, b: int) -> float:
            p, q = opts[i][a], opts[j][b]
            return math.hypot(q[2][0] - p[3][0], q[2][1] - p[3][1]) + r * angle_diff(heading(q[1]), out(p))

        left = []
        for i, a in best_order(start, link, end):
            lanes, direction, _, _ = opts[i][a]
            rest = self.drive_lanes(lanes, direction, angle, to_map)
            if rest:
                left.append(sorted(rest, key=lambda lane: lane.row))
        return left

    def _turn_into(self, a: XY, yaw: float, head: XY) -> Optional[XY]:
        """The next cell's first lane right next to where the mower is and the other way round: that's a turn
        between lanes, not a drive, round_earlier where it would turn on the spot. Where the lane starts then, None if
        no turn got there."""
        if not self.started or self.heading is None or angle_diff(yaw, self.heading + math.pi) > 1e-3 or \
                math.hypot(a[0] - self.pos[0], a[1] - self.pos[1]) > 4.0 * self.cfg.turn_radius:
            return None
        space = self.turn_space(a)
        turn = plan_turn(Pose2D(self.pos[0], self.pos[1], self.heading), a, self.cfg.turn_radius,
                         self.step, space, self.detour_path,
                         self.cfg.allowed_turns, self.turn_body, self.cfg.min_turn_radius if self.rounds else 0.0,
                         tight=not self.cfg.turn_radius_auto, wider=self.cfg.turn_radius_auto)
        rest = math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length
        if (turn is None or turn.name == 'pivot') and self.rounds:
            earlier = self.round_earlier(a, yaw, rest)
            if earlier is not None:
                turn, a = earlier
                self._count('lanes_earlier')
        if (turn is None or turn.name == 'pivot') and 'pivot' in self.cfg.allowed_turns:
            half = self.half_circle(Pose2D(self.pos[0], self.pos[1], self.heading), a, yaw, rest, space)
            if half is not None:
                turn, a = half
        if turn is None and self._reach is not None:
            turn = plan_turn(Pose2D(self.pos[0], self.pos[1], self.heading), a, self.cfg.turn_radius,
                             self.step, self.turn_space(a, False), self.detour_path,
                             self.cfg.allowed_turns, self.turn_body, self.cfg.min_turn_radius if self.rounds else 0.0,
                             tight=not self.cfg.turn_radius_auto, wider=self.cfg.turn_radius_auto)
        if turn is None:
            return None
        for piece in turn.pieces:
            self.add(TURN, piece.poses, turn.name)
        self._count('turns_' + turn.name)
        return a

    def visit_order(self, lanes: List[Lane]) -> List[Lane]:
        """The cell's lanes (one per row, in row order) in the order they're mowed."""
        if self.cfg.lane_order != 'skip' or len(lanes) < 3:
            return lanes
        spacing = abs(lanes[1].y - lanes[0].y)
        if spacing <= 0.0:
            return lanes
        # lanes k apart leave room for a U-turn of turn_radius. Every pass goes the same way, the turn from
        # the end of one pass to the start of the next is a long U-turn along the headland
        # (blade_offset: the centre lines are twice the offset closer every other turn)
        k = max(1, int(math.ceil((2.0 * self.cfg.turn_radius + 2.0 * abs(self.cfg.blade_offset)) / spacing - 1e-9)))
        n = len(lanes)
        if n < 2 * k:
            # too few lanes for that: from the middle, every step about half the cell wide (the smallest step
            # as big as it gets: n // 2)
            m = n // 2
            order = [i for pair in zip(range(m, n), range(m)) for i in pair] + ([n - 1] if n % 2 else [])
            return [lanes[i] for i in order]
        return [lanes[i] for p in range(k) for i in range(p, n, k)]

    def drive_lanes(self, lanes: List[Lane], direction: int, angle: float, to_map) -> Optional[List[Lane]]:
        """Mows the lanes as a zigzag. Returns the lanes it couldn't get to, None when all are done."""
        driven = 0
        for i, lane in enumerate(lanes):
            xa, xb = (lane.x0, lane.x1) if direction > 0 else (lane.x1, lane.x0)
            # the blade on the lane: the centre blade_offset to its right (left of the lanes going +x is +y) and
            # blade_ahead behind it
            y = lane.y - self.cfg.blade_offset * direction
            back = self.cfg.blade_ahead * direction
            a, b = to_map(xa - back, y), to_map(xb - back, y)
            if (self.cfg.blade_offset != 0 or back != 0) and not self.area.is_free([a] if lane.bends else [a, b]) \
                    or lane.bends and not self.area.is_free([b]):
                # shifted towards an edge, with fewer loops than the offset needs: as much of it as the centre may
                # drive (of a lane bending around obstacles the pieces at its ends)
                self._count('lanes_shortened')
                if lane.bends:
                    pieces = lane.pieces()
                    first, last = pieces[0] if direction > 0 else pieces[-1], pieces[-1] if direction > 0 else pieces[0]
                    ends = (self.area.clip(a, to_map((first[1] if direction > 0 else first[0]) - back, y)),
                            self.area.clip(to_map((last[0] if direction > 0 else last[1]) - back, y), b))
                    if ends[0] is None or ends[1] is None:
                        continue
                    a, b = ends[0][0], ends[1][1]
                else:
                    piece = self.area.clip(a, b, self.cfg.min_lane_length)
                    if piece is None:
                        continue
                    a, b = piece
            yaw = normalize_angle(angle if direction > 0 else angle + math.pi)
            pieces = lane.pieces()[::direction]  # (of a lane bending round obstacles: where its first piece ends)
            head = to_map((pieces[0][1] if direction > 0 else pieces[0][0]) - back, y) if lane.bends else b
            if self.footprint is not None:
                # the side the turns come from and go to (left of the lanes going +x is +y)
                side = [0.0 if o is None else direction * math.copysign(1.0, o.y - lane.y)
                        for o in (lanes[i - 1] if driven else None, lanes[i + 1] if i + 1 < len(lanes) else None)]
                if lane.bends:
                    # the straight pieces at its ends
                    last = to_map((pieces[-1][0] if direction > 0 else pieces[-1][1]) - back, y)
                    ends = self.body_ends(a, head, yaw, side[0], 0.0), self.body_ends(last, b, yaw, 0.0, side[1])
                    head = ends[0][1] if ends[0] is not None else head
                    ends = None if None in ends else (ends[0][0], ends[1][1])
                else:
                    ends = self.body_ends(a, b, yaw, side[0], side[1])
                if ends is None:
                    continue
                a, b = ends
                head = head if lane.bends else b  # (where the body lets it end)
            if self._reach is not None:
                a = self.start_in_reach(a, head, yaw)
            name = f'lane-{len(self.lanes)}'
            self.source = name
            self.origins[name] = {'parent': self.cell_of.get(id(lane), ''),
                                  'kind': 'bent' if lane.bends else 'straight', 'row': lane.row}
            start = self._turn_into(a, yaw, head) if driven == 0 else None
            if start is not None:
                a = start
            elif driven == 0:
                self.go_to(a, yaw, head)
                a = a if same_position(self.pos, a) else self.pos  # (into_lane may start it further on)
            else:
                exit_pose = Pose2D(self.pos[0], self.pos[1], self.heading)

                def turn_to(entry: XY, types, reach: bool = True) -> Optional[Maneuver]:
                    return plan_turn(exit_pose, entry, self.cfg.turn_radius, self.step,
                                     self.turn_space(entry, reach), self.detour_path, types, self.turn_body,
                                     self.cfg.min_turn_radius if self.rounds else 0.0,
                                     tight=not self.cfg.turn_radius_auto, wider=self.cfg.turn_radius_auto)

                # the turn goes level with the lane end further out, where the body may not fit (body_ends only
                # made room for a turn level with each end): this lane starts level with the last one's end then,
                # before a detour or a pivot
                ux, uy = math.cos(yaw), math.sin(yaw)
                level = (self.pos[0] - a[0]) * ux + (self.pos[1] - a[1]) * uy
                rest = math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length
                # (not by more than a turn reaches: what it leaves of the lane would stay unmowed)
                most = min(rest, self.footprint.reach + self.cfg.turn_radius) if self.footprint is not None else 0.0

                def find(a: XY, reach: bool) -> Tuple[Optional[Maneuver], XY]:
                    turn, types = None, self.cfg.allowed_turns
                    if self.footprint is not None and 1e-6 < level < most:
                        gentle = tuple(t for t in self.cfg.allowed_turns if t not in ('detour', 'pivot'))
                        turn = turn_to(a, gentle, reach)
                        if turn is None:
                            types = tuple(t for t in types if t not in gentle)  # (those don't fit at a)
                            start = (a[0] + level * ux, a[1] + level * uy)
                            turn = turn_to(start, gentle, reach)
                            if turn is not None:
                                a = start
                                self._count('lanes_body')
                    if turn is None:
                        turn = turn_to(a, types, reach)
                    if (turn is None or turn.name == 'pivot') and self.rounds:
                        earlier = self.round_earlier(a, yaw, rest, reach=reach)
                        if earlier is not None:
                            turn, a = earlier
                            self._count('lanes_earlier')
                    if (turn is None or turn.name == 'pivot') and 'pivot' in types:
                        half = self.half_circle(exit_pose, a, yaw, rest, self.turn_space(a, reach), reach)
                        if half is not None:
                            turn, a = half
                    return turn, a

                turn, a = find(a, True)
                if turn is None and self._reach is not None:
                    # (none within the lanes' reach, on the spot neither: one that takes its room, before a drive)
                    turn, a = find(a, False)
                if turn is not None:
                    for piece in turn.pieces:
                        self.add(TURN, piece.poses, turn.name)
                    self._count('turns_' + turn.name)
                else:
                    poses = self.drive_to(a, yaw, head)
                    if poses is None:
                        return lanes[i:]
                    a = a if same_position(poses[-1], a) else (poses[-1].x, poses[-1].y)
                    self.add(TRANSIT, poses, maneuver='turn_fallback')
                    self._count('turns_fallback')
            if lane.bends:
                self.bent_lane(lane, direction, a, b, to_map)
            else:
                self.add(LANE, sample_straight(a[0], a[1], b[0], b[1], yaw, self.step))
            self.lanes.append((a, b))
            direction = -direction
            driven += 1
        return None

    def start_in_reach(self, a: XY, head: XY, yaw: float) -> XY:
        """lane_overlap_limit: where the lane from a (facing yaw, straight to head) starts for the centre to keep where
        the lanes lie. It starts blade_ahead further out for the blade's first cut: that much later (TURN_REACH less
        where that leaves something), where what the lane then leaves is mowed already (the loops, but EARLIER_LEFT).
        a where it isn't, or where a is in the lanes' reach as it is."""
        back = self.cfg.blade_ahead
        if back <= TURN_REACH or self._reach.covers(Point(a)):
            return a
        ux, uy = math.cos(yaw), math.sin(yaw)
        rest = math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length
        end = Pose2D(head[0], head[1], yaw)
        mowed = None
        for later in (back, back - TURN_REACH):
            at = Pose2D(a[0] + later * ux, a[1] + later * uy, yaw)
            if later > rest or not self._reach.covers(Point(at.x, at.y)):
                continue
            left = subtract(self.blade([Pose2D(a[0], a[1], yaw), at]), [self.blade([at, end])])
            if left.area > EARLIER_LEFT:
                if mowed is None:
                    mowed = self.mowed([(a[0], a[1], a[0] + back * ux, a[1] + back * uy)], self.segments)
                if subtract(left, [mowed]).area > EARLIER_LEFT:
                    continue
            self._count('lanes_later')
            return (at.x, at.y)
        return a

    def half_circle(self, exit_pose: Pose2D, a: XY, yaw: float, rest: float,
                    fits: Fits, reach: bool = True) -> Optional[Tuple[Maneuver, XY]]:
        """turn_radius_auto, where no turn of turn_radius gets from exit_pose into the next lane (from a, facing yaw,
        rest m of it to spare), the lanes ending earlier for one neither: only now the half circle tighter than
        turn_radius, as wide as the lanes are apart, there or with the lanes ending earlier (round_earlier), before
        turning on the spot. The turn and where the lane starts then, None where none fits. reach: as for turn_space
        (fits is of the same)."""
        if not self.cfg.turn_radius_auto:
            return None  # (plan_turn and round_earlier tried it)
        half = plan_turn(exit_pose, a, self.cfg.turn_radius, self.step, fits, None, ('pivot',), self.turn_body,
                         self.cfg.min_turn_radius if self.rounds else 0.0, False)
        found = (half, a) if half is not None else self.round_earlier(a, yaw, rest, True, reach) if self.rounds \
            else None
        if found is not None:
            self._count('turns_tight')
        return found

    def round_earlier(self, a: XY, yaw: float, rest: float, half: bool = False,
                      reach: bool = True) -> Optional[Tuple[Maneuver, XY]]:
        """In place of turning on the spot (or a drive) from the lane just mowed into the next (from a, facing yaw, rest
        m of it to spare): a rounded turn level with the end of the shorter of the two, the longer one ending (or
        starting) earlier, then both, ROUND_STEP at a time up to turn_radius more: a U-turn (at a slant where they end
        at one), a bulb, a detour between lanes far apart. The first that fits, where the turn or what's mowed already
        (the loops next to them) mows what they leave (but EARLIER_LEFT): the lane just mowed is shortened then. Else a
        loop into the next lane (Dubins' shortest way that fits). None where none fits. half: only the half circle
        tighter than turn_radius (see half_circle, the rest is tried by then). reach: as for turn_space."""
        last = self.segments[-1] if self.segments else None
        if last is None or last.kind != LANE or last.maneuver or len(last.poses) < 2:
            return None
        first, end = last.poses[0], last.poses[-1]
        h = end.yaw
        if any(angle_diff(p.yaw, h) > 1e-6 for p in last.poses):
            return None  # a bent lane
        length = math.hypot(end.x - first.x, end.y - first.y)
        r, least = self.cfg.turn_radius, self.cfg.min_turn_radius
        hx, hy, ux, uy = math.cos(h), math.sin(h), math.cos(yaw), math.sin(yaw)
        dx = (a[0] - end.x) * hx + (a[1] - end.y) * hy  # how much further on the next lane starts
        far = abs((a[1] - end.y) * hx - (a[0] - end.x) * hy) > 4.0 * r
        types = tuple(t for t in self.cfg.allowed_turns if t != 'pivot' and (far or t != 'detour')) + ('pivot',)
        most_e = min(max(0.0, -dx) + r, length - self.cfg.min_lane_length)
        most_n = min(max(0.0, dx) + r, rest)
        fits = self.turn_space(a, reach)

        def shortest(exit_pose: Pose2D, entry: XY, loops: bool) -> Optional[Maneuver]:
            # Dubins' shortest way that fits, the widest first
            for rr in turn_radii(r, 4.0 * r, least):
                for m in dubins(exit_pose, Pose2D(entry[0], entry[1], yaw), rr, self.step, loops):
                    poses = m.pieces[0].poses
                    if fits([(p.x, p.y) for p in poses]) and (self.body is None or self.turn_body(poses)):
                        return m
            return None

        mowed = []

        def unmowed(region):
            # what of region isn't mowed already (but by the lane just mowed): the path near where the two lanes end
            # and start as far as they may get shorter, worked out once
            if not mowed:
                mowed.append(self.mowed([(end.x - most_e * hx, end.y - most_e * hy, end.x, end.y),
                                         (a[0], a[1], a[0] + most_n * ux, a[1] + most_n * uy)], self.segments[:-1]))
            return subtract(region, mowed)

        k = 0
        while True:
            level = max(0.0, dx) - k * ROUND_STEP  # where the turn goes, along the lane just mowed from its end
            de, dn = max(0.0, -level), max(0.0, dx - level)  # how much earlier it ends and the next one starts
            if de > most_e + 1e-9 or dn > most_n + 1e-9:
                break
            exit_pose = Pose2D(end.x - de * hx, end.y - de * hy, h)
            entry = (a[0] + dn * ux, a[1] + dn * uy)
            turn = shortest(exit_pose, entry, False) if 'u_turn' in self.cfg.allowed_turns and not half else None
            if turn is None and k and half:
                turn = plan_turn(exit_pose, entry, r, self.step, fits, None, ('pivot',), self.turn_body, least, False)
            elif turn is None and k:
                turn = plan_turn(exit_pose, entry, r, self.step, fits, self.detour_path, types,
                                 self.turn_body, least, False, not self.cfg.turn_radius_auto,
                                 self.cfg.turn_radius_auto)
            k += 1
            if turn is None:
                continue
            left = [self.blade([exit_pose, end])] if de > 1e-9 else []
            left += [self.blade([Pose2D(a[0], a[1], yaw), Pose2D(entry[0], entry[1], yaw)])] if dn > 1e-9 else []
            if left:
                there = subtract(union(left), [self.blade([p for piece in turn.pieces for p in piece.poses]),
                                               self.blade([first, exit_pose])])
                if there.area > EARLIER_LEFT and unmowed(there).area > EARLIER_LEFT:
                    break  # (further in it leaves more)
            if de > 1e-9:
                last.poses[:] = [p for p in last.poses
                                 if (p.x - first.x) * hx + (p.y - first.y) * hy < length - de - 1e-9] + [exit_pose]
                self._chunks.pop(id(last), None)
                self.pos, self.heading = (exit_pose.x, exit_pose.y), h
                self.lanes[-1] = (self.lanes[-1][0], (exit_pose.x, exit_pose.y))
            return turn, entry
        turn = shortest(end, a, True) if 'bulb' in self.cfg.allowed_turns and not half else None
        return None if turn is None else (turn, a)

    def blade(self, *runs: Sequence[Pose2D], whole: bool = False):
        """Where the blade mows driving poses (each run of them). whole: the whole path (quicker in pieces, see
        swept_area)."""
        half, ahead, off = 0.5 * self.cfg.mower_width, self.cfg.blade_ahead, self.cfg.blade_offset
        lines = [[(p.x + ahead * math.cos(p.yaw) - off * math.sin(p.yaw),
                   p.y + ahead * math.sin(p.yaw) + off * math.cos(p.yaw)) for p in poses] for poses in runs]
        if whole:
            # (straight runs of poses one straight line each first, where the pieces are cut)
            return union(LineString(piece).buffer(half, 4) for line in lines
                         for piece in _pieces(list(LineString(line).simplify(0.0).coords), SWEPT_PIECE, 2.0 * half))
        return (LineString(lines[0]) if len(lines) == 1 else MultiLineString(lines)).buffer(half, 4) if lines else \
            Polygon()

    def mowed(self, ends: List[Tuple[float, float, float, float]], segments: List[PathSegment]):
        """What the blade mowed driving segments, of their poses near the lines in ends (x0, y0, x1, y1)."""
        m = 0.5 * self.cfg.mower_width + abs(self.cfg.blade_ahead) + abs(self.cfg.blade_offset) + self.step
        boxes = [(min(x0, x1) - m, min(y0, y1) - m, max(x0, x1) + m, max(y0, y1) + m) for x0, y0, x1, y1 in ends]
        runs: List[List[Pose2D]] = []
        for seg in segments:
            run: List[Pose2D] = []
            for k, (cx0, cy0, cx1, cy1) in enumerate(self.chunks(seg)):
                if not any(cx0 <= x1 and x0 <= cx1 and cy0 <= y1 and y0 <= cy1 for x0, y0, x1, y1 in boxes):
                    if run:  # (none of these poses is in a box)
                        runs.append(run)
                        run = []
                    continue
                for p in seg.poses[k * MOWED_CHUNK:(k + 1) * MOWED_CHUNK]:
                    if any(x0 <= p.x <= x1 and y0 <= p.y <= y1 for x0, y0, x1, y1 in boxes):
                        run.append(p)
                    elif run:
                        runs.append(run)
                        run = []
            runs.append(run)
        return union(self.blade(run) for run in runs if len(run) >= 2)

    def chunks(self, seg: PathSegment) -> List[Tuple[float, float, float, float]]:
        """The box (x0, y0, x1, y1) round every MOWED_CHUNK poses of seg, worked out once (round_earlier forgets it
        where it shortens a lane)."""
        known = self._chunks.get(id(seg))
        if known is None:
            boxes = []
            for k in range(0, len(seg.poses), MOWED_CHUNK):
                xs, ys = [p.x for p in seg.poses[k:k + MOWED_CHUNK]], [p.y for p in seg.poses[k:k + MOWED_CHUNK]]
                boxes.append((min(xs), min(ys), max(xs), max(ys)))
            known = self._chunks[id(seg)] = (seg, boxes)
        return known[1]

    def body_ends(self, a: XY, b: XY, yaw: float, arrive: float, leave: float) -> Optional[Tuple[XY, XY]]:
        """The lane from a to b pulled back at its ends until the body fits there, and on the quarter arcs of
        turn_radius the turns arrive and leave on: arrive, leave 1 from or to the left, -1 the right, 0 no turn.
        None if less than min_lane_length is left."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        ux, uy = math.cos(yaw), math.sin(yaw)
        r, quarter = self.cfg.turn_radius, 0.5 * math.pi
        known: Dict[float, bool] = {}

        def fits(t: float) -> bool:
            if t not in known:
                known[t] = fits_at(t)
            return known[t]

        def fits_at(t: float) -> bool:
            x, y = a[0] + t * ux, a[1] + t * uy
            if not self.footprint.fits_pose(x, y, yaw):
                return False
            if r <= 1e-3:
                return True
            # leaving at the end, arriving at the start (worked out backwards, facing the other way)
            if t > 0.5 * length:
                return not leave or self.footprint.poses_fit(sample_arc(x, y, yaw, r, leave * quarter, self.step), True)
            return not arrive or self.footprint.poses_fit(
                [Pose2D(p.x, p.y, p.yaw + math.pi) for p in sample_arc(x, y, yaw + math.pi, r, -arrive * quarter,
                                                                         self.step)], True)

        def edge(inside: float, outside: float) -> float:
            # where between a point that fits and one that doesn't it stops fitting, the side that fits
            while abs(outside - inside) > BODY_STEP:
                t = 0.5 * (inside + outside)
                inside, outside = (t, outside) if fits(t) else (inside, t)
            return inside

        t0, t1 = 0.0, length
        if fits(t0) and fits(t1):
            return a, b
        # the body and a turn reach no further than this past the lane's end
        mid = 0.5 * length
        near = min(mid, 2.0 * (self.footprint.reach + self.cfg.turn_radius))
        if not fits(mid):
            t0 = t1 = mid
        else:
            t0 = t0 if fits(t0) else edge(near if fits(near) else mid, t0)
            t1 = t1 if fits(t1) else edge(length - near if fits(length - near) else mid, t1)
        self._count('lanes_body')
        if t1 - t0 < max(self.cfg.min_lane_length, 1e-6):
            return None
        return (a[0] + t0 * ux, a[1] + t0 * uy), (a[0] + t1 * ux, a[1] + t1 * uy)

    def bent_lane(self, lane: Lane, direction: int, a: XY, b: XY, to_map) -> None:
        """A lane that bends around obstacles (see bend_lanes), from a to b, its corners rounded. With
        blade_ahead the centre runs that much behind the blade: the ways around the obstacles from there."""
        back = self.cfg.blade_ahead * direction
        bends = []
        for x0, x1, way in lane.bends:
            if back != 0:
                found = self.area.path(to_map(x0 - back, lane.y), to_map(x1 - back, lane.y))
                way = found if found is not None else way
            bends.append(list(way))
        if direction < 0:
            bends = [way[::-1] for way in bends[::-1]]
        pts = [a] + [p for way in bends for p in way] + [b]
        pts = [p for i, p in enumerate(pts) if i == 0 or not same_position(p, pts[i - 1])]
        self.add(LANE, drive_polyline(pts, self.step, self.cfg.turn_radius, self.area.is_free, body=self.body),
                 maneuver='bend')
        self._count('bends')


def _driven(rings: Sequence[Sequence[Sequence[float]]], corners: Sequence[XY]):
    """Where a body of these corners (counter-clockwise round its centre, see PlannerConfig.body_outline) was while the
    centre drove along each of the rings the way its points go: along every edge from where it was at the edge's start
    to where it was at its end (_rows), so past a corner and on the outside of a bend the front reaches out, and at
    every point turning on the spot from the heading of the edge before to the next one's (TURN_STEP apart)."""
    parts, rows = [], _rows(corners)

    def body(x: float, y: float, h: float, points: Sequence[XY]) -> Polygon:
        c, s = math.cos(h), math.sin(h)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in points])

    for ring in rings:
        heads = []  # every edge's heading and end
        for (ax, ay), (bx, by) in zip(ring, list(ring[1:]) + list(ring[:1])):
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-9:
                heads.append((math.atan2(by - ay, bx - ax), bx, by))
                parts += [body(bx, by, heads[-1][0], [(dx - length * back, dy) for dx, dy, back in row])
                          for row in rows]
        for (h, x, y), (h1, _, _) in zip(heads, heads[1:] + heads[:1]):
            turn = normalize_angle(h1 - h)
            n = int(math.ceil(abs(turn) / TURN_STEP))
            parts += [body(x, y, h + turn * k / n, corners) for k in range(1, n)]
    return union(parts)


def _rows(corners: Sequence[XY]) -> List[List[Tuple[float, float, int]]]:
    """The body of these corners (counter-clockwise) in pieces every line along its heading crosses once (itself where
    it is like that, else cut along the heading where its edges turn from going left to going right or back), each as
    its corners (x, y, 1 on its rear side; the leftmost and rightmost on both sides): driving straight ahead it swept
    the polygon of a piece's corners where it got to, those on the rear side as far back as it drove."""
    ys = [y for _, y in corners]
    goes = [(b > a, b) for a, b in zip(ys, ys[1:] + ys[:1]) if b != a]  # (to the left, to where)
    turns = sorted({y for (left, y), (on, _) in zip(goes, goes[1:] + goes[:1]) if left != on})
    pieces = [list(corners)]
    if len(turns) > 2:
        x0, _, x1, _ = Polygon(corners).bounds
        pieces = [ring for a, b in zip(turns, turns[1:])
                  for _, ring in rings(clip_by_rect(Polygon(corners), x0 - 1.0, a, x1 + 1.0, b))]
    rows = []
    for piece in pieces:
        row: List[Tuple[float, float, int]] = []
        for i, (x, y) in enumerate(piece):
            into, out = y - piece[i - 1][1], piece[(i + 1) % len(piece)][1] - y
            if into == out == 0:  # (in the middle of an edge along it)
                continue
            if into >= 0 and out >= 0 or into <= 0 and out <= 0:
                row.append((x, y, int(into <= 0 and out <= 0)))
            else:
                row += [(x, y, 0), (x, y, 1)] if into > 0 else [(x, y, 1), (x, y, 0)]
        rows.append(row)
    return rows


def _length(pts: Sequence[XY]) -> float:
    return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))


def _spins(poses: Sequence[Pose2D], heading_in: Optional[float], heading_out: Optional[float]) -> int:
    """How often the mower turns on the spot by more than SPIN_LIMIT driving poses, coming in facing heading_in and
    going on facing heading_out."""
    yaws = ([heading_in] if heading_in is not None else []) + [p.yaw for p in poses] + \
        ([heading_out] if heading_out is not None else [])
    return sum(1 for a, b in zip(yaws, yaws[1:]) if angle_diff(a, b) > SPIN_LIMIT)


def _arcs(poses: Sequence[Pose2D]) -> float:
    """rad the way turns on its arcs all together (not on the spot)."""
    return sum(angle_diff(a.yaw, b.yaw) for a, b in zip(poses, poses[1:]) if not same_position(a, b))


def _smooth_spins(route: '_Route', cfg: PlannerConfig) -> None:
    """Where the path turns on the spot by more than SPIN_LIMIT (a sharp corner of a loop, a drive arriving across
    a lane, a pivot), an omega (maneuvers.omega) of turn_radius (down to SMOOTH_RADII of it) in its place if it fits in
    the space of the segment (the drivable areas for drives, with headland_turns out of the lanes' field where the turn
    is) and the body fits: the wheels keep rolling."""
    inserts: Dict[Tuple[int, int], List[Pose2D]] = {}
    before: Optional[Pose2D] = None  # the last pose before the position the mower is at
    here: Optional[Tuple[Pose2D, int, int]] = None  # the pose it leaves this position with, where it is

    def smooth() -> None:
        if before is None or here is None:
            return
        p, i, j = here
        turn = normalize_angle(p.yaw - before.yaw)
        if abs(turn) <= SPIN_LIMIT:
            return

        def straight(k: int) -> bool:
            return route.segments[k].kind == LANE and route.segments[k].maneuver not in ('ring', 'bend')

        if j == 0 and i > 0 and straight(i):  # where a lane starts: on the end of the segment before
            i, j = i - 1, len(route.segments[i - 1].poses)
        if straight(i):  # (a lane keeps its heading)
            return
        space = route.transit.is_free if route.segments[i].kind == TRANSIT else route.area.is_free
        clear = route.clear([(p.x, p.y)])  # (headland_turns: out of the lanes' field the loop stays out of it)

        def fits(pts: Sequence[XY]) -> bool:
            return space(pts) and (not clear or route.clear(pts)) and \
                (route.segments[i].kind != TURN or route.in_reach(pts, [(p.x, p.y)]))

        at = Pose2D(p.x, p.y, before.yaw)
        body = route.turn_body if route.segments[i].kind == TURN else route.body  # (on a turn spin_margin too)
        for share in SMOOTH_RADII:
            r = share * cfg.turn_radius
            loop = omega(at, turn, r, route.step) if r >= max(1e-3, cfg.min_turn_radius - 1e-9) else None
            if loop and fits([(q.x, q.y) for q in loop]) and (body is None or body([before] + loop)):
                inserts[(i, j)] = loop
                route._count('spins_smoothed')
                return

    for i, seg in enumerate(route.segments):
        for j, p in enumerate(seg.poses):
            if here is not None and route.segments[here[1]].section != seg.section:
                before = here = None  # the path jumps to the next section
            if here is not None and same_position(here[0], p):
                here = (p, i, j)  # the same position, now left facing p's way
                continue
            smooth()
            before, here = (here[0] if here is not None else None), (p, i, j)
    for (i, j), loop in sorted(inserts.items(), reverse=True):
        # (the last pose is where the segment goes on from: one pose there when they're put together)
        route.segments[i].poses[j:j] = loop


def _splice(ring: List[XY], other: List[XY], lead: float = 0.0) -> List[XY]:
    """ring with a trip round other from the closest point between them: over to other, round it and back, on
    along ring. lead: it leaves ring that far before that point and gets back that far after it, and goes round other
    from that far after to that far before its point: the hops over go at a slant, not at right angles."""
    closed, round_ = ring + ring[:1], other + other[:1]
    p, q = nearest_points(LineString(closed), LineString(round_))

    def from_to(pts: List[XY], near) -> List[XY]:
        # pts (closed) from `lead` after the point closest to near round to `lead` before it, by the way along them
        i, at, _ = closest_on_polyline(pts, (near.x, near.y))
        cum = [0.0]
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.hypot(bx - ax, by - ay))
        n = cum[-1]
        d = min(lead, 0.25 * n)
        t0 = cum[i] + math.hypot(at[0] - pts[i][0], at[1] - pts[i][1]) + d

        def point(t: float) -> XY:
            t %= n
            k = min(bisect.bisect_right(cum, t) - 1, len(pts) - 2)
            f = (t - cum[k]) / max(cum[k + 1] - cum[k], 1e-12)
            return pts[k][0] + f * (pts[k + 1][0] - pts[k][0]), pts[k][1] + f * (pts[k + 1][1] - pts[k][1])

        span = n - 2.0 * d
        inner = sorted(((cum[k] - t0) % n, pts[k]) for k in range(len(pts) - 1))
        return [point(t0)] + [pt for t, pt in inner if 1e-9 < t < span - 1e-9] + [point(t0 + span)]

    return from_to(closed, p) + from_to(round_, q)


def _clear_of_corners(closed: List[XY], i: int, entry: XY, keep: float) -> XY:
    """Moves a loop's start/end point on segment i away from a corner next to it: the loop starts and ends
    there, so a corner right at it couldn't be rounded."""
    n = len(closed) - 1
    (ax, ay), (bx, by) = closed[i], closed[i + 1]
    length = math.hypot(bx - ax, by - ay)
    if keep <= 0.0 or length < 1e-9:
        return entry
    t = math.hypot(entry[0] - ax, entry[1] - ay)

    def corner(k: int) -> bool:
        p, q, r = closed[(k - 1) % n], closed[k % n], closed[(k + 1) % n]
        return angle_diff(math.atan2(r[1] - q[1], r[0] - q[0]), math.atan2(q[1] - p[1], q[0] - p[0])) > CORNER_ANGLE

    if t < keep and corner(i):
        t = min(keep, 0.5 * length)
    elif length - t < keep and corner(i + 1):
        t = max(length - keep, 0.5 * length)
    else:
        return entry
    return ax + (bx - ax) * t / length, ay + (by - ay) * t / length


def _first_direction(loop: List[XY]) -> Optional[XY]:
    for p in loop[1:]:
        dx, dy = p[0] - loop[0][0], p[1] - loop[0][1]
        d = math.hypot(dx, dy)
        if d > 1e-7:
            return dx / d, dy / d
    return None


def _pieces(pts: List[XY], n: int, cut: float) -> List[List[XY]]:
    """pts in pieces of at least n points, cut in the middle of a straight piece at least `cut` long."""
    out, cur = [], [pts[0]]
    for p, q in zip(pts, pts[1:]):
        if len(cur) >= n and math.hypot(q[0] - p[0], q[1] - p[1]) >= cut:
            m = (0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1]))
            out.append(cur + [m])
            cur = [m]
        cur.append(q)
    out.append(cur)
    return out
