"""The path put together on the route: the parts of the area one after the other, in each the loops (rings_old) and
the lanes in the order order.py gives, each lane as swaths.py has it, the turns between as connect.py finds them."""

import math
from typing import Any, Dict, List, Optional

from shapely.geometry import Point, Polygon

from .config import LANE, TRANSIT, TURN, XY, Lane
from .connect import Turn, between, into_next
from .geometry import polygonal, polygons, rings, union
from .maneuvers import same_position, sample_straight
from .mathutil import Pose2D
from .order import final_cell, nearest_cell, planned_cells
from .rings_old import Rings, _Route
from .swaths import Passes, body_ends, centre_line

Work = Dict[int, Dict[str, Any]]


def mow_parts(route: _Route, rings_of: Rings, passes: Passes, zones: list, narrow, holes: List[List[XY]],
              start: XY, end: Optional[XY], cell_of: Dict[int, str]) -> None:
    """The parts of the area there's no way between (in the transit space) one after the other, each completely: loops
    and lanes of one part, then the next as a new section of the path, the one closest to end last."""
    parts = route.transit.parts()
    work = _work(route, parts, rings_of, passes, narrow)

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
        _mow_part(route, work[k], rings_of, passes, zones, holes, cell_of)


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
        return work.setdefault(k, {'levels': {}, 'middles': {}, 'cells': [[] for _ in passes], 'narrow': None})

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
    return work


def _mow_part(route: _Route, w: Dict[str, Any], rings_of: Rings, passes: Passes, zones: list,
              holes: List[List[XY]], cell_of: Dict[int, str]) -> None:
    """One part: the loops first (perimeter_order 'last': last), the end point counting for what's mowed last: the
    loops with last, else the lanes, else the strips too narrow for lanes, else the loops."""
    cfg = route.cfg
    last, empty = cfg.perimeter_order == 'last', Polygon()
    skip = {'outline': cfg.skip_outline_loops, 'hole': cfg.skip_obstacle_loops}
    loops_here = not rings_of.ringed and any(not skip[kind] and not route.left_out(kind, i)
                                             for i, g in w['levels'].items() for kind, _ in rings(g))
    loops_end = loops_here and (last or not any(w['cells']) and w['narrow'] is None)

    def loops() -> None:
        if rings_of.ringed:
            route.concentric([w['levels'].get(i, empty) for i in range(len(rings_of.levels))],
                             [w['middles'].get(i, empty) for i in range(len(rings_of.middles))], rings_of.most, last,
                             rings_of.loop_spacing, last or not any(w['cells']),
                             cfg.fill_pattern != 'concentric' or cfg.narrow_parts == 'loops')
        else:
            route.perimeter([w['levels'].get(i, empty) for i in range(rings_of.perimeter)], holes, last, loops_end)

    if not last:
        loops()
    _mow_lanes(route, w, passes, zones, loops_end, cell_of)
    if last:
        loops()


def _mow_lanes(route: _Route, w: Dict[str, Any], passes: Passes, zones: list, loops_end: bool,
               cell_of: Dict[int, str]) -> None:
    """The strips too narrow for lanes first (so the lanes can end at the end point), then the sets of lanes; with
    split_zones the rest first, then the zones, the nearest next (the one closest to the end point last)."""
    last_pass = max((i for i, cells in enumerate(w['cells']) if cells), default=None)
    if w['narrow'] is not None:
        route.extra_loops(w['narrow'], finish=not loops_end and last_pass is None)
    for i in range(len(passes)):
        if not zones or i == 0:
            mow_cells(route, w['cells'][i], passes[i][0], not loops_end and i == last_pass and not zones, cell_of)
    later = [i for i in range(len(passes)) if zones and i > 0 and w['cells'][i]]

    def near(i: int, p: Optional[XY]) -> float:
        a = passes[i][0]
        return min(route._distance(p, lane.ends(a)[k]) for cell in w['cells'][i] for lane in cell for k in (0, 1))

    final = min(later, key=lambda i: (near(i, route.end), i)) if later and route.end is not None and not loops_end \
        else None
    while later:
        i = min([j for j in later if j != final] or later, key=lambda j: (near(j, route.pos), j))
        later.remove(i)
        mow_cells(route, w['cells'][i], passes[i][0], not loops_end and not later, cell_of)


def mow_cells(route: _Route, cells: List[List[Lane]], angle: float, finish: bool, cell_of: Dict[int, str]) -> None:
    """Mows the cells, the closest one not mowed yet next (route_order 'optimized': in planned_cells' order first,
    then what that couldn't get to). finish: the cell that can end closest to the end point goes last."""
    cfg = route.cfg
    route.angle = angle
    pending = [list(cell) for cell in cells if cell]
    if cfg.route_order == 'optimized' and len(pending) > 1:
        left = []
        for lanes, direction in planned_cells(pending, route.pos, route.heading,
                                              route.end if finish else None, angle, cfg):
            rest = _drive_lanes(route, lanes, direction, angle, cell_of)
            if rest:
                left.append(sorted(rest, key=lambda lane: lane.row))
        pending, finish = left, False
    reserved = pending.pop(final_cell(pending, route.end, angle, cfg)) if finish and route.end is not None and \
        pending else None
    while pending or reserved:
        end = None
        if not pending:
            pending, reserved, end = [reserved], None, route.end
        ci, lanes, direction = nearest_cell(pending, route.pos, route.heading, angle, cfg, end)
        pending.pop(ci)
        rest = _drive_lanes(route, lanes, direction, angle, cell_of)
        if rest:
            pending.append(sorted(rest, key=lambda lane: lane.row))


def _drive_lanes(route: _Route, lanes: List[Lane], direction: int, angle: float,
                 cell_of: Dict[int, str]) -> Optional[List[Lane]]:
    """Mows the lanes as a zigzag. Returns the lanes it couldn't get to, None when all are done."""
    cfg = route.cfg
    driven = 0
    for i, lane in enumerate(lanes):
        s = centre_line(lane, direction, angle, cfg, route.area, route._count)
        if s is not None and route.footprint is not None:
            # the side the turns come from and go to (left of the lanes going +x is +y)
            side = [0.0 if o is None else direction * math.copysign(1.0, o.y - lane.y)
                    for o in (lanes[i - 1] if driven else None, lanes[i + 1] if i + 1 < len(lanes) else None)]
            s = body_ends(route.footprint, cfg, s, side[0], side[1], route._count)
        if s is None:
            continue
        name = f'lane-{len(route.lanes)}'
        route.source = name
        route.origins[name] = {'parent': cell_of.get(id(lane), ''), 'kind': 'straight', 'row': lane.row}
        a = _get_in(route, s.a, s.yaw, s.b) if driven == 0 else _turn(route, s.a, s.yaw, s.b)
        if a is None:
            return lanes[i:]
        route.add(LANE, sample_straight(a[0], a[1], s.b[0], s.b[1], s.yaw, route.step))
        route.lanes.append((a, s.b))
        direction = -direction
        driven += 1
    return None


def _mowed(route: _Route):
    return lambda ends: route.mowed(ends, route.segments[:-1])


def _get_in(route: _Route, a: XY, yaw: float, b: XY) -> XY:
    """Into a cell's first lane from a (to b): a turn where it starts right next to where the path ends, else a drive.
    Where it starts then."""
    at = Pose2D(route.pos[0], route.pos[1], route.heading) if route.started and route.heading is not None else None
    last = route.segments[-1] if route.segments else None
    lane = last.poses if last is not None and last.kind == LANE and not last.maneuver else None
    turn = into_next(route, at, lane, a, yaw, b, _mowed(route))
    if turn is not None:
        return _apply(route, turn)
    route.go_to(a, yaw, b)
    return a if same_position(route.pos, a) else route.pos  # (into_lane may start it further on)


def _turn(route: _Route, a: XY, yaw: float, b: XY) -> Optional[XY]:
    """From the lane just mowed into the next from a (to b): a turn, else a drive. Where it starts then, None where
    there's no way."""
    turn = between(route, route.segments[-1].poses, a, yaw, b, _mowed(route))
    if turn is not None:
        return _apply(route, turn)
    poses = route.drive_to(a, yaw, b)
    if poses is None:
        return None
    route.add(TRANSIT, poses, maneuver='turn_fallback')
    route._count('turns_fallback')
    return a if same_position(poses[-1], a) else (poses[-1].x, poses[-1].y)


def _apply(route: _Route, turn: Turn) -> XY:
    if turn.lane is not None:
        route.end_lane(turn.lane)
    for piece in turn.maneuver.pieces:
        route.add(TURN, piece.poses, turn.maneuver.name)
    for key in turn.counts:
        route._count(key)
    return turn.entry
