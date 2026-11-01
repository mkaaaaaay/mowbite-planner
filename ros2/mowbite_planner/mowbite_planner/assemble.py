"""The path put together on the route: the parts of the area one after the other, in each the loops (rings) and
the lanes, the cells in the order order.py finds, each one of the ways stripes.py has (its lanes and the turns between
from the catalogue in turns.py), the drives between through drive.py."""

import math
from typing import Any, Dict, List, Optional, Tuple

from shapely.geometry import Point, Polygon, box

from .drive import MARGIN_BEFORE, keeps_margin
from .config import LANE, TRANSIT, TURN, XY, Lane
from .geometry import polygonal, polygons, rings, union
from .maneuvers import same_position
from .mathutil import Pose2D, angle_diff
from .objective import END_S, Cost, seconds
from .order import EXACT, Guess, plan_order
from .levels import Rings
from .rings import _Route
from .stripes import Cells, Run, Runs, Way, kept_runs, lane_poses
from .swaths import Passes
from .turns import Catalog, Turn

Work = Dict[int, Dict[str, Any]]


class LaneKit:
    """What mowing the lanes of a plan keeps: the catalogue of turns, the guesses of drives, the runs of each set of
    lanes, the lane the path ends with (its run and how much later it starts, its segment)."""

    def __init__(self, route: _Route, remaining) -> None:
        cfg = route.cfg

        def left(geom) -> float:  # (of what the lanes are there to mow, near it first: quicker)
            return 0.0 if geom.is_empty else geom.intersection(remaining.intersection(box(*geom.bounds))).area

        self.route = route
        self.catalog = Catalog(route, left, cfg.lane_spacing)
        self.guess = Guess(route.transit, cfg)
        self._runs: Dict[float, Runs] = {}
        self.last: Optional[Tuple[Run, float, Any]] = None

    def runs(self, angle: float) -> Runs:
        if angle not in self._runs:
            r = self.route
            self._runs[angle] = kept_runs(r.cfg, angle, r.area, r.footprint)
        return self._runs[angle]

    def ending(self) -> Optional[Tuple[Run, float]]:
        """The lane the path ends with (its run, how much later it starts), None where it ends otherwise."""
        if self.last is None or not self.route.segments or self.route.segments[-1] is not self.last[2]:
            return None
        return self.last[0], self.last[1]

    def link(self, end: Pose2D, start: Pose2D, room_end: float, room_start: float) -> Optional[Turn]:
        """The turn from a lane's end into a lane starting right next to it the other way round, None where there's
        none (they're further apart, or none fits)."""
        r = self.route.cfg.turn_radius
        if math.hypot(start.x - end.x, start.y - end.y) > 4.0 * r or angle_diff(start.yaw, end.yaw + math.pi) > 1e-3:
            return None
        return self.catalog.turn(end, start, room_end, room_start)


def mow_parts(route: _Route, rings_of: Rings, passes: Passes, zones: list, narrow, holes: List[List[XY]],
              start: XY, end: Optional[XY], cell_of: Dict[int, str]) -> None:
    """The parts of the area there's no way between (in the transit space) one after the other, each completely: loops
    and lanes of one part, then the next as a new section of the path, the one closest to end last."""
    parts = route.transit.parts()
    work = _work(route, parts, rings_of, passes, narrow)
    lanes = LaneKit(route, rings_of.remaining)

    def distance(k: int, p: Optional[XY]) -> float:
        return 0.0 if p is None else parts[k].distance(Point(p))

    pending = set(work)
    final = min(pending, key=lambda k: distance(k, end)) if end is not None and len(pending) > 1 else None
    first = True
    while pending:
        k = min([q for q in pending if q != final] or list(pending),
                key=lambda q: (distance(q, start if first else route.pos), q))
        pending.remove(k)
        # the end point only counts in the part mowed last
        route.end = end if not pending else None
        if not first:
            route.new_section()
        first = False
        _mow_part(route, work[k], rings_of, passes, zones, holes, cell_of, lanes)


def _work(route: _Route, parts: list, rings_of: Rings, passes: Passes, narrow) -> Work:
    """What's to mow in each part: {part: {'levels': {level: geometry}, 'middles': {k: geometry}, 'cells': [cells of
    each set], 'narrow': geometry or None}}."""
    def part_of(p: XY) -> int:
        k = route.transit.part(p)
        if k is None:
            k = min(range(len(parts)), key=lambda i: parts[i].distance(Point(p)))
        return k

    def split(geom) -> Dict[int, Any]:
        out: Dict[int, List[Polygon]] = {}
        for poly in polygons(geom):
            out.setdefault(part_of(poly.representative_point().coords[0]), []).append(poly)
        return {k: polygonal(union(v)) for k, v in out.items()}

    work: Work = {}

    def todo(k: int) -> Dict[str, Any]:
        return work.setdefault(k, {'levels': {}, 'middles': {}, 'cells': [[] for _ in passes], 'narrow': None,
                                   'own': []})

    for key, geoms in (('levels', rings_of.drive_levels), ('middles', rings_of.middles)):
        for i, geom in enumerate(geoms):
            for k, g in split(geom).items():
                todo(k)[key][i] = g
    for pi, (a, cells) in enumerate(passes):
        for cell in cells:
            if cell:
                lane = cell[0]
                x, y = 0.5 * (lane.x0 + lane.x1), lane.y
                todo(part_of((x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))))['cells'][pi] \
                    .append(cell)
    for k, geom in split(narrow).items():
        todo(k)['narrow'] = geom
    for level, ring in rings_of.own_rings:
        todo(part_of(ring[0]))['own'].append((level, ring))
    return work


def _mow_part(route: _Route, w: Dict[str, Any], rings_of: Rings, passes: Passes, zones: list,
              holes: List[List[XY]], cell_of: Dict[int, str], lanes: LaneKit) -> None:
    """One part: the loops first (perimeter_order 'last': last), the end point counting for what's mowed last: the
    loops with last, else the lanes, else the strips too narrow for lanes, else the loops."""
    cfg = route.cfg
    last, empty = cfg.perimeter_order == 'last', Polygon()
    skip = {'outline': cfg.skip_outline_loops, 'hole': cfg.skip_obstacle_loops}
    spiral = rings_of.ringed and cfg.fill_pattern == 'concentric'
    loops_here = not spiral and any(not skip[kind] and not route.ring_out(kind, ring, i)
                                    for i, g in w['levels'].items() if i < rings_of.perimeter for kind, ring in rings(g))
    loops_end = loops_here and (last or not any(w['cells']) and w['narrow'] is None) and not rings_of.ringed

    def loops() -> None:
        levels = [w['levels'].get(i, empty) for i in range(len(rings_of.levels))]
        middles = [w['middles'].get(i, empty) for i in range(len(rings_of.middles))]
        if spiral:
            route.concentric(levels, middles, rings_of.most, last, rings_of.loop_spacing, last or not any(w['cells']),
                             cfg.narrow_parts == 'loops')
            return
        route.perimeter(levels[:rings_of.perimeter], holes, last, loops_end, w['own'])
        if rings_of.ringed:
            # narrow_parts 'loops': the narrow parts' own loops after the loops along the edges, one after the other
            route.concentric([empty] * rings_of.most + levels[rings_of.most:], middles, rings_of.most, last,
                             rings_of.loop_spacing, last or not any(w['cells']), False)

    if not last:
        loops()
    _mow_lanes(route, w, passes, zones, loops_end, cell_of, lanes)
    if last:
        loops()


def _mow_lanes(route: _Route, w: Dict[str, Any], passes: Passes, zones: list, loops_end: bool,
               cell_of: Dict[int, str], lanes: LaneKit) -> None:
    """The strips too narrow for lanes first (so the lanes can end at the end point), then the sets of lanes; with
    split_zones the rest first, then the zones, the nearest next (the one closest to the end point last)."""
    last_pass = max((i for i, cells in enumerate(w['cells']) if cells), default=None)
    if w['narrow'] is not None:
        route.extra_loops(w['narrow'], finish=not loops_end and last_pass is None)
    for i in range(len(passes)):
        if not zones or i == 0:
            mow_cells(route, w['cells'][i], passes[i][0], not loops_end and i == last_pass and not zones, cell_of,
                      lanes)
    later = [i for i in range(len(passes)) if zones and i > 0 and w['cells'][i]]

    def near(i: int, p: Optional[XY]) -> float:
        a = passes[i][0]
        return min(route._distance(p, lane.ends(a)[k]) for cell in w['cells'][i] for lane in cell for k in (0, 1))

    final = min(later, key=lambda i: (near(i, route.end), i)) if later and route.end is not None and not loops_end \
        else None
    while later:
        i = min([j for j in later if j != final] or later, key=lambda j: (near(j, route.pos), j))
        later.remove(i)
        mow_cells(route, w['cells'][i], passes[i][0], not loops_end and not later, cell_of, lanes)


def mow_cells(route: _Route, cells: List[List[Lane]], angle: float, finish: bool, cell_of: Dict[int, str],
              lanes: LaneKit) -> None:
    """Mows the cells, each one of its ways (Cells.ways), in the order plan_order finds (route_order 'nearest': the
    cheapest next each time) from where the path is, finish: ending as close to the end point as it gets."""
    cfg = route.cfg
    route.angle = angle
    guess = lanes.guess
    band = cfg.lane_spacing

    def lost(lane: Lane) -> float:
        return (lane.x1 - lane.x0) * band

    runs = lanes.runs(angle)
    of = Cells(cfg, runs, lanes.catalog, guess, lost)
    groups = []
    for cell in cells:
        ways = of.ways(cell) if cell else []
        if ways:
            groups.append(ways)
        for lane in cell if not ways else ():  # (none of its lanes has a run)
            route._count(runs(lane, 1, 0.0, 0.0)[1])
    if not groups:
        return
    end = Pose2D(route.end[0], route.end[1], None) if finish and route.end is not None else None
    # (what each way costs on its own, the lanes it leaves out: on getting into it)
    alone = [[seconds(w.cost) for w in g] for g in groups]
    start = [[seconds(_into(route, lanes, w)) + alone[i][a] for a, w in enumerate(g)] for i, g in enumerate(groups)]
    out = [[END_S * math.hypot(w.exit.x - end.x, w.exit.y - end.y) + seconds(guess(w.exit, end, True))
            if end is not None else 0.0 for w in g] for g in groups]

    # (with more than EXACT groups the drives between them guessed straight, a lot of them)
    exact = len(groups) <= EXACT

    def link(i: int, a: int, j: int, b: int) -> float:
        return seconds(_between(lanes, groups[i][a], groups[j][b], exact)) + alone[j][b]

    for i, a in plan_order(start, link, out, cfg.route_order == 'optimized'):
        _drive_way(route, lanes, of.realize(groups[i][a]), cell_of)


def _room(run: Run, cut: float, least: float) -> float:
    return max(0.0, run.length - cut - least)


def _into(route: _Route, lanes: LaneKit, w: Way) -> Cost:
    """About what getting into w's first lane costs from where the path ends (from where it starts looking, the dock:
    where it doesn't yet)."""
    if not route.started:
        return lanes.guess(Pose2D(route.pos[0], route.pos[1], None), w.entry, True) if route.pos is not None \
            else Cost()
    ending = lanes.ending()
    if ending is not None:
        run, cut = ending
        t = lanes.link(run.end(), w.entry, _room(run, cut, route.cfg.min_lane_length),
                       0.5 * _room(w.runs[0][1], 0.0, route.cfg.min_lane_length))
        if t is not None:
            return t.cost
    return lanes.guess(Pose2D(route.pos[0], route.pos[1], route.heading), w.entry)


def _between(lanes: LaneKit, a: Way, b: Way, exact: bool) -> Cost:
    """About what getting from the end of way a into way b costs: a turn where b starts right next to it, else a
    drive (exact False: guessed straight there)."""
    least = lanes.route.cfg.min_lane_length
    u, v = a.runs[-1][1], b.runs[0][1]
    t = lanes.link(u.end(), v.start(), _room(u, 0.0, least), 0.5 * _room(v, 0.0, least))
    return t.cost if t is not None else lanes.guess(u.end(), v.start(), straight=not exact)


def _drive_way(route: _Route, lanes: LaneKit, way: Way, cell_of: Dict[int, str]) -> None:
    """way's lanes and turns onto the route, into its first lane by a turn where it starts right next to the lane the
    path ends with, else by a drive (or the path starts there)."""
    cfg = route.cfg
    cuts = [[0.0, 0.0] for _ in way.runs]
    for i, t in enumerate(way.turns):
        if t is not None:
            cuts[i][1], cuts[i + 1][0] = t.cut_exit, t.cut_entry
    into: Optional[Turn] = None
    ending = lanes.ending()
    first = way.runs[0][1]
    if route.started and ending is not None:
        run, cut = ending
        into = lanes.link(run.end(), first.start(), _room(run, cut, cfg.min_lane_length),
                          0.5 * _room(first, cuts[0][1], cfg.min_lane_length))
        if into is not None:
            route.end_lane(lane_poses(run, cut, into.cut_exit, route.step))
            cuts[0][0] = into.cut_entry
    for i, (lane, run) in enumerate(way.runs):
        poses = lane_poses(run, cuts[i][0], cuts[i][1], route.step)
        name = f'lane-{len(route.lanes)}'
        route.source = name
        route.origins[name] = {'parent': cell_of.get(id(lane), ''), 'kind': 'straight', 'row': lane.row}
        turn = into if i == 0 else way.turns[i - 1]
        head = (poses[-1].x, poses[-1].y)
        if turn is not None:
            route.add(TURN, list(turn.poses), turn.name)
            route._count('turns_' + turn.name)
        else:
            _drive_into(route, poses[0], head, '' if i == 0 else 'turn_fallback')
        if route.started and not same_position(route.pos, poses[0]):
            # (a drive into the lane gets in further on: it starts there)
            later = (route.pos[0] - poses[0].x) * math.cos(run.yaw) + (route.pos[1] - poses[0].y) * math.sin(run.yaw)
            cuts[i][0] += max(0.0, later)
            poses = lane_poses(run, cuts[i][0], cuts[i][1], route.step)
        for key, said in (('lanes_body', run.trimmed), ('lanes_shortened', run.clipped)):
            if said:
                route._count(key)
        route.add(LANE, poses)
        route.lanes.append(((poses[0].x, poses[0].y), (poses[-1].x, poses[-1].y)))
    for key in way.left_out:
        route._count(key)
    lanes.last = (way.runs[-1][1], cuts[-1][0], route.segments[-1])


def _drive_into(route: _Route, at: Pose2D, head: XY, maneuver: str) -> None:
    """A drive into a lane starting at (ending at head), into_lane's then plain, each through the transit space then the
    mowing area: the first keeping spin_margin on its turns, else the first found (none: the path jumps there)."""
    target = (at.x, at.y)
    if not route.started or same_position(route.pos, target):
        route.go_to(target, at.yaw, head)
        return
    before = route.segments[-1].poses[-MARGIN_BEFORE:] if route.segments else []
    found = kept = None
    for h, space in ((head, None), (head, route.area), (None, None), (None, route.area)):
        poses = None if space is not None and route.area is route.transit else route.drive_to(target, at.yaw, h, space)
        if poses is not None and _keeps_margin(route, before, poses, at.yaw):
            kept = poses
            break
        found = found or poses
    if kept is None and found is not None:  # (one with spin_margin on its arcs, where one fits)
        kept = route.margin_drive(target, at.yaw, before)
    if (kept or found) is None:
        route.go_to(target, at.yaw, head)
        return
    route.add(TRANSIT, kept or found, maneuver=maneuver)
    if maneuver:
        route._count('turns_fallback')


def _keeps_margin(route: _Route, before: List[Pose2D], drive: List[Pose2D], yaw: float) -> bool:
    """drive.keeps_margin with the lane on from the drive's end facing yaw."""
    return keeps_margin(route.footprint, route.step, before, drive, yaw)
