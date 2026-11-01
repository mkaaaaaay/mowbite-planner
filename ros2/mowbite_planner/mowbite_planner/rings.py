"""The loops: each ring of the levels (levels.py) driven along, its corners rounded where the body fits
(maneuvers.drive_polyline), the plan's first starting where mower_logic gets onto it, the loops along the outline and
round the obstacles in OpenMower's order (perimeter), part by part (concentric), round the strips too narrow for lanes
(extra_loops). The route the path is put together on with its loops (_Route: route.Route, Loops, loopfit.LoopFits)."""

import bisect
import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.ops import nearest_points
from shapely.prepared import prep

from .config import FIT_EASE, FIT_EASE_OFF, LANE, PERIMETER, XY
from .geometry import QUAD_SEGS, inset, polygonal, polygons, rings
from .lead import lead_in
from .levels import CORNER_CUT, centre_line, left_out as levels_left_out
from .loopfit import LoopFits
from .maneuvers import CORNER_ANGLE, closest_on_polyline, drive_polyline
from .mathutil import Pose2D, angle_diff
from .route import Route
from .trace import HAIR_KEEP, _bad, repaired, turned_to

try:  # shapely 2 tests many points at once
    from shapely import covers, distance, dwithin, points, prepare
except ImportError:  # shapely 1.7 (ROS Noetic)
    covers = None

# m, mower_logic drives to the plan's first pose itself and turns on the spot there unchecked: with the body set the
# plan's first loop starts where a place the body turns whatever way it faces is START_ROOM away at most and its poses
# fit before as far as easing onto its line from there takes (FIT_EASE, where bodyfit's way in gets on) and
# START_CLEAR[0] more, START_CLEAR[1] after (more than bodyfit's FIT_START: a tight place that close it would leave out,
# starting later), at the closest such point to where it would start (the loop all there, the tight place further round,
# driven another way like any other)
START_CLEAR = (0.5, 1.5)
START_ROOM = 1.0
# a loop after the first starts where it would where the way there fits the body and turns ENTRY_TURN rad at most (no
# loop on the way: on the spot where it has to turn round), else at the closest of its poses (ENTRY_SPREAD m along it
# apart, ENTRY_TRIES at most) that does: no safety net's way round to it across the other loops
ENTRY_TRIES = 16
ENTRY_SPREAD = 0.5
ENTRY_TURN = math.radians(200.0)


class Loops:
    """Driving the loops, methods of the route (_Route)."""

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

    def ring_out(self, kind: str, ring: List[XY], depth: float) -> bool:
        """left_out of that ring: with the levels from the body (owners) each island's own."""
        if self.owners is None:
            return self.left_out(kind, depth)
        count, islands, most = self.ring_counts
        return levels_left_out(count, islands, self.owners, most, kind, ring, depth)

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

    def _flip(self, ring: List[XY], loop: List[XY], hole: bool, kind: str, level: int) -> bool:
        """Whether a loop (ring as rings() gives it, loop: from where it starts) is driven the other way round."""
        direction = self.cfg.perimeter_direction
        if direction == 'random':  # (the same for each count of finished mows, on Python 3.8 as on 3.13)
            direction = 'cw' if random.Random(self.cfg.finished_mows).random() < 0.5 else 'ccw'
        if self.cfg.blade_offset != 0:
            # rings come with the mowable side on the left: the blade goes on the other side, but for the rings
            # driven with the blade on the inner one
            return (self.cfg.blade_offset > 0) != (id(ring) in self.blade_in)
        if self.recorded is not None and self.cfg.perimeter_direction in ('auto', 'random') and kind == PERIMETER \
                and not hole and level == 0:
            # the outermost loop the way the outline was recorded: the body where it was then (its real edges), the
            # front nowhere else (rings come ccw)
            return not self.recorded
        if direction == 'auto':
            # the start pose's heading counts too: no half turn on the spot before the first loop
            if self.heading is None:
                return False
            # go round the way the mower already faces
            fwd = _first_direction(loop)
            back = _first_direction(loop[::-1])
            h = (math.cos(self.heading), math.sin(self.heading))
            return fwd is not None and back is not None and \
                h[0] * back[0] + h[1] * back[1] > h[0] * fwd[0] + h[1] * fwd[1] + 1e-9
        # outlines come ccw, holes cw
        return (direction == 'cw') != hole

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
        flip = self._flip(ring, loop, hole, kind, level)
        if flip:
            loop = loop[::-1]
        # the outermost loop the way the outline was recorded: where the body was then (route.recorded_fp)
        as_recorded = kind == PERIMETER and not hole and level == 0 and self.recorded is not None and \
            self.recorded_fp is not None and flip == (not self.recorded)
        args = (loop, closed, entry, target, hole, flip, as_recorded, radius, level, spacing)
        poses, flip, counted = self._loop_poses(*args)
        if as_recorded and self.recorded_centre is not None and not self._centre_kept(poses):
            # (its centre further past the lines than RECORDED_CENTRE: again, every way checked so)
            poses, flip, counted = self._loop_poses(*args, strict=True)
        if counted is not None:
            self.counts['loops_repaired'] = self.counts.get('loops_repaired', 0) + counted[0]
            self.counts['loops_left'] = self.counts.get('loops_left', 0) + counted[1]
        entry = (poses[0].x, poses[0].y)
        name = f'loop-{len(self.loops)}'
        if as_recorded:
            self.recorded_loops.add(name)
        self.source = name
        self.origins[name] = {'parent': parent or f'level-{level}', 'kind': 'obstacle' if hole else 'outline',
                              'level': level}
        poses, lead = lead_in(self, poses, -1.0 if flip else 1.0, self.recorded_fp if as_recorded else self.footprint)
        self.go_to((poses[0].x, poses[0].y), poses[0].yaw)
        self.add(kind, lead or [], maneuver='lead')  # (the plan's first loop's way onto it, see lead.py)
        self.add(kind, poses, maneuver=maneuver)
        self.loops.append([(p.x, p.y) for p in poses])
        self.loop_side[name] = -1.0 if flip else 1.0

    def _loop_poses(self, loop: List[XY], closed: List[XY], entry: XY, target: Optional[XY], hole: bool, flip: bool,
                    as_recorded: bool, radius: float, level: int, spacing: float, strict: bool = False):
        """drive_ring's poses round loop (closed: the ring closed, from entry; flip: driven the other way round):
        (the poses, flip, (stretches driven anew, poses left as they were) where the loop was traced, else None).
        strict: the outermost loop driven the way it was recorded with its centre kept in route.recorded_centre."""
        body, fits, room = self._checks(as_recorded, strict)

        def driven(loop: List[XY]) -> List[Pose2D]:
            # with the mowable side on the left only left turns go round a corner of it and get rounded
            # (right turns too: the arc cuts their corner on the side mowed already or keeps off the edge where it
            # doesn't fit)
            return drive_polyline(loop, self.step, radius, fits, 0, CORNER_CUT * self.cfg.mower_width,
                                  level, spacing, self.cfg.mower_width, body)

        poses = driven(loop)
        # the plan's first loop: where mower_logic gets to it on its own (see START_CLEAR), the same way round, found
        # on the loop as planned (a stretch driven anew below is no line to ease onto)
        better = self._start_at(poses, target) if not self.segments and self.footprint is not None else None
        was_traced = self.owners is not None and self.footprint is not None and self.cfg.blade_offset == 0 and \
            len(poses) > 3 and math.hypot(poses[0].x - poses[-1].x, poses[0].y - poses[-1].y) < 1e-6
        if was_traced:
            # with the levels from the body: where the body doesn't fit along it, that stretch driven anew as the
            # mower would (trace.py), not the safety net's way round
            fp = self.recorded_fp if as_recorded else self.footprint
            poses, fixed, left = self._repaired(poses, fp, loop, hole, spacing, room)
            poses = turned_to(poses, entry if better is None else better)
            if better is None and self.started and self.segments:
                got = self._fitting_entry(poses)
                if got is poses and hole and not as_recorded:
                    # (no way onto an island's loop fits: the other way round, where one onto that does)
                    back, more, less = self._repaired(driven(loop[::-1]), fp, loop[::-1], hole, spacing, room)
                    back = turned_to(back, entry)
                    turned = self._fitting_entry(back)
                    if turned is not back and less <= left:
                        got, fixed, left, flip = turned, more, less, not flip
                poses = got
            return poses, flip, (fixed, left)
        elif better is not None:
            i, entry, _ = closest_on_polyline(closed, better)
            entry = _clear_of_corners(closed, i, entry, 2.0 * radius)
            loop = [entry] + closed[i + 1:] + closed[1:i + 1] + [entry]
            if flip:
                loop = loop[::-1]
            poses = driven(loop)
        return poses, flip, None

    def _checks(self, as_recorded: bool, strict: bool = False):
        """A loop's (body(poses), fits(points) of the centre, room: where the centre may go, None: the area's space):
        the outermost one driven the way the outline was recorded where the body was then (route.recorded_fp), strict:
        its centre in route.recorded_centre too."""
        if not as_recorded:
            return self.body, self.area.is_free, None
        if not strict:
            return self.recorded_fp.poses_fit, self.area.is_free, None
        room = self.area.space.intersection(self.recorded_centre)
        centre = prep(room)

        def fits(points: List[XY]) -> bool:
            return self.area.is_free(points) and centre.covers(LineString(points))
        return self.recorded_fp.poses_fit, fits, room

    def _centre_kept(self, poses: List[Pose2D]) -> bool:
        """Whether the poses' centres stay in route.recorded_centre."""
        if self._recorded_centre is None:
            self._recorded_centre = prep(self.recorded_centre)
        return self._recorded_centre.covers(LineString([(p.x, p.y) for p in poses])) if len(poses) > 1 else True

    def _repaired(self, poses: List[Pose2D], footprint, loop: List[XY], hole: bool, spacing: float, room=None):
        """trace.repaired of the closed loop's poses (loop: its points, the first again last), the turns round in
        dead ends off the loops driven so far. room: where the centre may go (default the area's space)."""
        return repaired(poses, footprint, loop[:-1], hole, self.step, self.cfg.mower_width, self.cfg.blade_ahead,
                        (self.area.space if room is None else room).buffer(1e-3, 4), taken=self._taken(spacing))

    def _taken(self, spacing: float):
        """taken(xs, ys) for repaired: which of the points lie within HAIR_KEEP lane spacings of the loops driven so
        far (worked out the first time it's asked)."""
        far, kept = HAIR_KEEP * spacing, []

        def taken(xs, ys) -> List[bool]:
            if not kept:
                # (a turn on the spot is a point twice in a row: left out, the distances are the same, and shapely 2.0
                # on GEOS 3.11 doesn't take a segment of no length for an invalid value)
                runs = [[q for k, q in enumerate(loop) if k == 0 or q != loop[k - 1]] for loop in self.loops]
                lines = MultiLineString([run for run in runs if len(run) > 1])
                if covers is None:  # (shapely 1.7: whether a point is in the band round them)
                    lines = prep(lines.buffer(far, 4)) if not lines.is_empty else None
                elif not lines.is_empty:
                    prepare(lines)
                kept.append(None if lines is None or (covers is not None and lines.is_empty) else lines)
            lines = kept[0]
            if lines is None:
                return [False] * len(xs)
            if covers is None:
                return [lines.contains(Point(x, y)) for x, y in zip(xs, ys)]
            return [bool(v) for v in dwithin(lines, points(list(zip(xs, ys))), far)]

        return taken

    def _fitting_entry(self, poses: List[Pose2D]) -> List[Pose2D]:
        """The closed loop poses starting where the way there from the mower fits the body (see ENTRY_TRIES); as they
        are where none does."""
        fp = self.footprint
        n = len(poses) - 1
        if fp is None or n < 3 or self.pos is None:
            return poses
        at = [0.0]
        for a, b in zip(poses, poses[1:]):
            at.append(at[-1] + math.hypot(b.x - a.x, b.y - a.y))
        # (where it would start first: as it is where the way there fits)
        order = [0] + sorted(range(1, n), key=lambda j: (math.hypot(poses[j].x - self.pos[0],
                                                                  poses[j].y - self.pos[1]), j))
        tried: List[int] = []
        for j in order:
            if len(tried) >= ENTRY_TRIES:
                break
            if any(min(abs(at[j] - at[k]), at[n] - abs(at[j] - at[k])) < ENTRY_SPREAD for k in tried):
                continue
            tried.append(j)
            way = self.drive_to((poses[j].x, poses[j].y), poses[j].yaw)
            if way is None:
                continue
            ring = poses[j:-1] + poses[:j + 1]
            seq = [Pose2D(self.pos[0], self.pos[1], way[0].yaw if self.heading is None else self.heading)] + way + \
                ring[1:4]
            if sum(angle_diff(b.yaw, a.yaw) for a, b in zip(seq, seq[1:])) <= ENTRY_TURN and not _bad(seq, fp):
                return ring
        return poses

    def perimeter(self, levels: List[object], base_holes: List[List[XY]], last: bool = False,
                  finish: bool = False, own: Sequence[Tuple[int, List[XY]]] = ()) -> None:
        """The loops along the outline (level by level) and around the obstacles (each from its outermost
        loop in towards it, closest obstacle first, like OpenMower). last: the obstacles first, then the
        outline from the inside out, ending on the outermost loop. finish (nothing comes after the loops):
        of the last level's loops (or obstacles) the one closest to the end point comes last and ends as close
        to it as it gets, the obstacles go first if that's closer. own: (level, ring) of islands' loops on their own
        (levels.Rings.own_rings), with that level's."""
        hole_polys = [Polygon(h) for h in base_holes]
        hole_marks = [p.representative_point() for p in hole_polys]
        outlines: List[Tuple[int, List[List[XY]]]] = []
        groups: Dict[int, List[Tuple[int, List[XY]]]] = {}
        spare = len(hole_polys)
        for k, level in enumerate(levels):
            level_rings = [(kind, ring) for kind, ring in self.centre_rings(level) if not self.ring_out(kind, ring, k)]
            level_rings += [('hole', ring) for j, ring in own if j == k]
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
        """Loops around the bits of the area no lane gets to (strips too narrow for one), closest first, a thin one's
        middle line once (with the body checked, levels.centre_line). finish: the one closest to the end point last,
        ending as close to it as it gets."""
        todo = [(kind == 'hole', ring) for kind, ring in self.centre_rings(geom)]
        keep = min(todo, key=lambda t: self._end_distance(t[1]), default=None) \
            if finish and self.end is not None else None
        while todo:
            item = min([t for t in todo if t is not keep] or todo, key=lambda t: self._ring_distance(t[1]))
            todo.remove(item)
            line = None if item[0] or self.owners is None else centre_line(item[1], 0.5 * self.cfg.mower_width)
            if line is not None:
                self.drive_line(line, LANE, 'ring', 'narrow')
                continue
            self.drive_ring(item[1], item[0], LANE, 'ring', target=self.end if item is keep else None, radius=0.0,
                            parent='narrow')

    def drive_line(self, pts: List[XY], kind: str, maneuver: str, parent: str) -> None:
        """A thin piece's middle line driven once, from its end closer to the mower."""
        if self.pos is not None and math.hypot(pts[-1][0] - self.pos[0], pts[-1][1] - self.pos[1]) < \
                math.hypot(pts[0][0] - self.pos[0], pts[0][1] - self.pos[1]):
            pts = pts[::-1]
        poses = drive_polyline(pts, self.step, self.cfg.perimeter_corner_radius, self.area.is_free, 0,
                               CORNER_CUT * self.cfg.mower_width, 0, self.cfg.lane_spacing, self.cfg.mower_width,
                               self.body)
        name = f'loop-{len(self.loops)}'
        self.source = name
        self.origins[name] = {'parent': parent, 'kind': 'outline', 'level': 0}
        self.go_to(pts[0], poses[0].yaw)
        self.add(kind, poses, maneuver=maneuver)
        self.loops.append([(p.x, p.y) for p in poses])

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


class _Route(LoopFits, Loops, Route):
    """The route the path is put together on (route.Route) with its loops (Loops) and their repairs (LoopFits)."""
