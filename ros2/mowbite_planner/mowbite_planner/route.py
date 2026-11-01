"""The route the path is put together on: where the mower is, the segments so far, the drives between them (with
headland_turns along the headland), what the blade mowed near a place. The loops' methods come on top (rings._Route)."""

import bisect
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point

from .config import ROUND_STEP, SPIN_LIMIT, TRANSIT, XY, PathSegment, PlannerConfig
from .drive import Drive
from .footprint import Footprint
from .geometry import subtract, union
from .lanes import _ring_list, _Scanner
from .maneuvers import Fits, dubins, join, path_length, sample_straight
from .mathutil import Pose2D, angle_diff
from .objective import EARLIER_LEFT
from .sweep import blade
from .transit import TransitPlanner

# headland_turns: a drive along the headland more than HEADLAND_SLACK m and HEADLAND_DETOUR times as long as the
# shortest way crosses the lanes' field where that's shorter, along a lane's line (the CROSSINGS closest tried)
HEADLAND_DETOUR = 2.0
HEADLAND_SLACK = 2.0
CROSSINGS = 3
# poses of a segment Route.mowed() looks at together: it skips those whose box is far from where it looks
MOWED_CHUNK = 32


class Route:
    def __init__(self, cfg: PlannerConfig, transit: TransitPlanner, area: TransitPlanner, start: Optional[XY],
                 heading: Optional[float], end: Optional[XY], footprint: Optional[Footprint] = None):
        self.cfg = cfg
        self.footprint = footprint
        self.body = footprint.poses_fit if footprint is not None else None
        # (a turn between lanes: spin_margin all along it too)
        self.turn_body = (lambda poses: footprint.poses_fit(poses, True)) if footprint is not None else None
        # a turn between lanes may have arcs (turns.py, see min_turn_radius; only pivots allowed: on the spot)
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
        # with the body checked: whether the outline was recorded ccw, its outermost loop goes that way (see drive_ring)
        self.recorded: Optional[bool] = None
        # what the segments added now mow or lead to, and where every loop and lane comes from
        self.source = ''
        self.origins: Dict[str, Dict[str, Any]] = {}
        self._chunks: Dict[int, Tuple[PathSegment, List[Tuple[float, float, float, float]]]] = {}  # see chunks()
        # headland_turns: the lanes' field (and prepared), the transit space but the field, the lanes' direction and the
        # lines they lie on (lane frame y) of every set, the field's pieces on those (see rows())
        self.field = self._field = self.headland = None
        self.loop_counts: Optional[Tuple[int, int]] = None  # (obstacle_passes: see left_out)
        # with the loops' levels from the body (levels.body_levels): the islands' loops, (count, islands, most)
        self.owners: Optional[list] = None
        self.ring_counts: Tuple[int, int, int] = (0, 0, 0)
        # the body where it was while the outline was recorded (no edge_margin, no spin_margin): the outermost loop
        # driven the way it was recorded is checked in that (rings.drive_ring, the safety net), the loops (by name)
        # driven so
        self.recorded_fp: Optional[Footprint] = None
        self.recorded_loops: set = set()
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

    def end_lane(self, poses: List[Pose2D]) -> None:
        """The lane the path ends with is poses now, ending earlier (for a turn)."""
        last, end = self.segments[-1], poses[-1]
        last.poses[:] = poses
        self._chunks.pop(id(last), None)
        self.pos, self.heading = (end.x, end.y), end.yaw
        self.lanes[-1] = (self.lanes[-1][0], (end.x, end.y))

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
        best = Drive(space, self.footprint, self.cfg).way(pos, facing, target, heading)
        if best is not None and not self.cfg.turn_on_spot and heading is not None and self.cfg.turn_radius > 1e-3 and \
                head is not None and self.cfg.min_turn_radius > 0 and angle_diff(best[-1].yaw, heading) > SPIN_LIMIT:
            return self.into_lane(best, target, heading, head, space) or best
        return best

    def into_lane(self, drive: List[Pose2D], a: XY, yaw: float, head: XY,
                  space: TransitPlanner) -> Optional[List[Pose2D]]:
        """In place of the drive's end turning on the spot into the lane from a (facing yaw, its straight piece ending
        at head): Dubins' shortest way into it that fits in space, turning a quarter turn at most more than it has to
        (no loop), from a pose of the drive's last 4 turn_radius (one every turn_radius back, the latest first, its
        start too), where none does into the lane ROUND_STEP further on at a up to 2 turn_radius, the first whose way
        or what's mowed already (the loops) mows what it leaves of the lane (but EARLIER_LEFT). None where none fits."""
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
                most = angle_diff(drive[k].yaw, yaw) + 0.5 * math.pi + 1e-6
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

    def turn_space(self, at: Pose2D, a: XY) -> Fits:
        """What a turn from at into a lane starting at a keeps in: the mowing area, with headland_turns out of the
        lanes' field too where the lanes are more than 4 turn_radius apart (a long U-turn)."""
        ux, uy = math.cos(at.yaw), math.sin(at.yaw)
        if self.field is None or abs((a[1] - at.y) * ux - (a[0] - at.x) * uy) <= 4.0 * self.cfg.turn_radius:
            return self.area.is_free
        return lambda pts: self.area.is_free(pts) and self.clear(pts)

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

    def blade(self, *runs: Sequence[Pose2D], whole: bool = False):
        """Where the blade mows driving poses (see sweep.blade)."""
        return blade(self.cfg, *runs, whole=whole)

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
        """The box (x0, y0, x1, y1) round every MOWED_CHUNK poses of seg, worked out once (end_lane forgets it
        where it shortens a lane)."""
        known = self._chunks.get(id(seg))
        if known is None:
            boxes = []
            for k in range(0, len(seg.poses), MOWED_CHUNK):
                xs, ys = [p.x for p in seg.poses[k:k + MOWED_CHUNK]], [p.y for p in seg.poses[k:k + MOWED_CHUNK]]
                boxes.append((min(xs), min(ys), max(xs), max(ys)))
            known = self._chunks[id(seg)] = (seg, boxes)
        return known[1]

    def _distance(self, a: Optional[XY], b: XY) -> float:
        return 0.0 if a is None else math.hypot(b[0] - a[0], b[1] - a[1])


def _length(pts: Sequence[XY]) -> float:
    return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
