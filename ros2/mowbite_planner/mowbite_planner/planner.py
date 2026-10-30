"""Coverage path for one mowing area, without any ROS: shapely in, a list of poses out (plan_coverage).

The stages: the spaces from the map (space.py), the loops (rings_old.py) and the lanes (swaths.py), the path put
together (assemble.py: order.py, connect.py, drive.py), the body's check as a safety net (bodyfit.py), counted by where
what it changes comes from. The settings and what a plan is made of are in config.py.
"""

import dataclasses
import math
import multiprocessing
import threading
import time
import types
from concurrent.futures import Future, ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Polygon
from shapely.prepared import prep

from .assemble import mow_parts
from .bodyfit import _assemble, _field_turns, _lanes_field, _round_corners, _turn_at
from .complete import complete, leftovers
from .config import (_BODY_SPACE, LANE, LEFT_PIECE, LEFT_TOTAL, LEFT_TUFT, PERIMETER, SPACING_CANDIDATES, TRANSIT,
                     TURN, XY, ZERO_TURN, CoveragePlan, Lane, NothingLeft, PlannerConfig, PlanningError)
from .geometry import clean_polygon, polygonal, polygons, rings, subtract
from .lanes import lane_angle
from .maneuvers import path_length
from .objective import LEFT_GAIN
from .rings_old import Rings, _Route, _smooth_spins, fit_rings, narrow_left, ring_levels
from .space import _check_coordinates, _edge_slot_count, build_space, margins_of
from .sweep import plan_footprint, swept_area
from .swaths import lane_sets
from .transit import TransitPlanner

# a plan this big is a mistake (a lane spacing in mm, a waypoint every mm), not a lawn: about 200 km of path
MAX_POSES = 2000000


def _point(p: Optional[Sequence[float]]) -> Optional[XY]:
    return (float(p[0]), float(p[1])) if p is not None and len(p) >= 2 else None


# set_workers: processes of their own plan (an area's plans round its slots by the edge at once, the plans of callers on
# threads of their own at once), the same plans as in this one. Started fresh (spawn), not forked from a process with
# threads of its own
_WORKERS = 0
_POOL: Optional[ProcessPoolExecutor] = None
_POOL_LOCK = threading.Lock()


def set_workers(n: int) -> None:
    """Plans in up to n processes of their own from now on, 0 (the default): all in this one."""
    global _WORKERS, _POOL
    with _POOL_LOCK:
        if _POOL is not None and int(n) != _WORKERS:
            _POOL.shutdown()  # (it ends what it's at: not waiting, Python 3.8 complains when the pipes close)
            _POOL = None
        _WORKERS = max(0, int(n))


def _pool() -> Optional[ProcessPoolExecutor]:
    global _POOL
    with _POOL_LOCK:
        if _POOL is None and _WORKERS > 0:
            _POOL = ProcessPoolExecutor(_WORKERS, mp_context=multiprocessing.get_context('spawn'))
        return _POOL


def _planned(args: tuple, kw: Dict[str, Any]) -> Tuple[CoveragePlan, Any]:
    """_plan_coverage (in a process of the pool): the plan and its probe as _plan_coverage left it."""
    return _plan_coverage(*args, **kw), kw.get('probe')


def _later(args: tuple, kw: Dict[str, Any]) -> 'Future':
    """_planned in a process of the pool where there's one (a future of it), else at once."""
    pool = _pool()
    if pool is not None:
        try:
            return pool.submit(_planned, args, kw)
        except (BrokenProcessPool, RuntimeError):  # (one of them died, or it's shut down: in this one)
            set_workers(_WORKERS)
    done: Future = Future()
    try:
        done.set_result(_planned(args, kw))
    except BaseException as e:  # noqa: BLE001 - handed on by result()
        done.set_exception(e)
    return done


def _result(future: 'Future', args: tuple, kw: Dict[str, Any]) -> Tuple[CoveragePlan, Any]:
    try:
        return future.result()
    except BrokenProcessPool:  # (a process of the pool died, say out of memory: in this one, the pool anew)
        with _POOL_LOCK:
            global _POOL
            _POOL = None
        return _planned(args, kw)


def plan_coverage(boundary: Sequence[Sequence[float]], obstacles: Sequence[Sequence[Sequence[float]]] = (),
                  config: Optional[PlannerConfig] = None, start: Optional[Sequence[float]] = None,
                  end: Optional[Sequence[float]] = None,
                  drivable: Sequence[Sequence[Sequence[float]]] = (),
                  keep_out: Sequence[Sequence[Sequence[float]]] = (),
                  margins: Sequence[Optional[float]] = (), mowed=None,
                  reach_over: Sequence[Sequence[Sequence[float]]] = ()) -> CoveragePlan:
    """Plans the coverage path of one area, see _plan_coverage. With lane_spacing_mode 'auto' the best of
    SPACING_CANDIDATES lane spacings (see PlannerConfig), the same input always picks the same. stats['chosen']: the
    lane spacing and loops it was planned with. mowed (a geometry: where the blade went already, see
    report.track_mowed): only what that left, see _plan_coverage. A slot by the edge too narrow for the body (see
    _edge_gaps) is closed where that leaves less unmowed than going through it (stats['edge_slots']: how many). With
    set_workers the plans in processes of their own, both ways round the slots at once. The plan's free_space,
    robot_space and transit_space are the map's as drawn, slots open."""
    cfg = config or PlannerConfig()
    cfg.validate()
    area = _Area(boundary, obstacles, start, end, drivable, keep_out, margins, reach_over)
    if mowed is not None:
        plan = area.one(cfg, mowed=mowed)
    elif cfg.lane_spacing_mode != 'auto':
        plan = area.plan(cfg)
    else:
        plan = _auto(area, cfg)
    return area.as_drawn(plan if mowed is not None else _completed(area, plan))


class _Area:
    """One area's input, planned with a config: in a process of the pool where there's one, both ways round the slots
    by the edge."""

    def __init__(self, boundary, obstacles, start, end, drivable, keep_out, margins, reach_over) -> None:
        self.boundary, self.obstacles, self.start, self.end = boundary, obstacles, start, end
        self.drivable, self.keep_out, self.margins, self.reach_over = drivable, keep_out, margins, reach_over
        self.choice: Dict[str, bool] = {}  # 'edge_slots': closed or not, worked out with the first plan

    def one(self, c: PlannerConfig, **more: Any) -> Any:
        return _plan_coverage(self.boundary, self.obstacles, c, self.start, self.end, self.drivable, self.keep_out,
                              self.margins, reach_over=self.reach_over, **more)

    def later(self, c: PlannerConfig, **more: Any) -> Tuple['Future', tuple, Dict[str, Any]]:
        args = (self.boundary, self.obstacles, c, self.start, self.end, self.drivable, self.keep_out, self.margins)
        kw = dict(reach_over=self.reach_over, **more)
        return _later(args, kw), args, kw

    @staticmethod
    def got(planning: Tuple['Future', tuple, Dict[str, Any]], probe: Any) -> CoveragePlan:
        plan, filled = _result(*planning)
        if isinstance(probe, dict) and filled is not probe:  # (filled in another process)
            probe.update(filled)
        return plan

    def slotted(self, c: PlannerConfig) -> bool:
        try:
            return _edge_slot_count(self.boundary, self.obstacles, c, self.drivable, self.keep_out, self.margins,
                                    self.reach_over) > 0
        except Exception:  # noqa: BLE001 - (what's wrong with the input planning says)
            return False

    def plan(self, c: PlannerConfig, probe: Any = False) -> CoveragePlan:
        """The slots closed or not, whichever leaves less of the lawn (with them open) unmowed (LEFT_GAIN less to close
        them): worked out with the first plan, the same for the other lane spacings."""
        if 'edge_slots' in self.choice:
            return self.got(self.later(c, probe=probe, edge_slots=self.choice['edge_slots']), probe)
        mine = {} if isinstance(probe, dict) else probe
        closing = self.later(c, probe=mine)
        # (in processes of their own both at once where it'll have slots)
        opening = self.later(c, probe=probe, edge_slots=False) if _pool() is not None and self.slotted(c) else None
        closed = self.got(closing, mine)
        opened = closed if not closed.stats['edge_slots'] else \
            self.got(opening or self.later(c, probe=probe, edge_slots=False), probe)

        def left(plan: CoveragePlan) -> float:
            return sum(p.area for p in polygons(opened.free_space.difference(swept_area(plan, c.mower_width, 0.01)))
                       if p.area >= LEFT_TUFT)

        self.choice['edge_slots'] = closed is opened or left(closed) < left(opened) - LEFT_GAIN
        if self.choice['edge_slots'] and isinstance(probe, dict):
            probe.clear()
            probe.update(mine)
        return closed if self.choice['edge_slots'] else opened

    def as_drawn(self, plan: CoveragePlan) -> CoveragePlan:
        """plan with the spaces it's measured against as drawn: the slots by the edge open."""
        if not plan.stats.get('edge_slots'):
            return plan
        cfg = plan.config
        margin_of = margins_of(self.boundary, self.obstacles, self.drivable, self.keep_out, self.reach_over,
                               self.margins, cfg)
        space, _ = build_space(self.boundary, self.obstacles, self.drivable, self.keep_out, self.reach_over, margin_of,
                               cfg, False)
        plan.stats['area'] = space.free.area
        return dataclasses.replace(plan, free_space=space.free, robot_space=space.drive_space,
                                   transit_space=space.transit_space)


def _auto(area: _Area, cfg: PlannerConfig) -> CoveragePlan:
    """lane_spacing_mode 'auto': the widest lane spacing of SPACING_CANDIDATES after which a narrower one doesn't leave
    LEFT_GAIN less unmowed."""
    t0 = time.perf_counter()
    lo, hi = cfg.spacing_range
    best, best_left, best_spacing, error = None, None, 0.0, None
    probed: Dict[float, float] = {}
    kept: Dict[float, Dict[str, Any]] = {}  # of the spacings planned through, what quick() needs

    def fixed(spacing: float) -> PlannerConfig:
        return dataclasses.replace(cfg, lane_spacing_mode='fixed', overlap=1.0 - spacing / cfg.mower_width)

    def quick(spacing: float) -> float:
        if spacing not in probed:
            probed[spacing] = _left_alone(*kept[spacing]['alone']) if 'alone' in kept.get(spacing, {}) else \
                area.one(fixed(spacing), probe=True, edge_slots=area.choice.get('edge_slots', True))
        return probed[spacing]

    def gain(spacing: float, narrower: float) -> bool:
        # a narrower spacing is only planned through (turns, drives) where its loops and lanes alone leave LEFT_GAIN
        # less than the last one's
        return quick(narrower) <= quick(spacing) - LEFT_GAIN

    spacings = [hi - (hi - lo) * i / max(1, SPACING_CANDIDATES - 1) for i in range(SPACING_CANDIDATES)]
    for i, spacing in enumerate(spacings):
        try:
            if best is not None and not gain(best_spacing, spacing):
                break
            kept[spacing] = {}
            plan = area.plan(fixed(spacing), probe=kept[spacing])
        except PlanningError as e:  # this spacing doesn't fit (too many poses), a narrower one may not either
            error = error or e
            continue
        if best is None and i + 1 < len(spacings):
            # (the first plan, where the next spacing gains nothing: what it leaves, the whole path's swept area (slow),
            # doesn't count)
            try:
                enough = not gain(spacing, spacings[i + 1])
            except PlanningError:  # (the next round skips that spacing)
                enough = False
            if enough:
                best = plan
                break
        left = [p for p in polygons(plan.free_space.difference(swept_area(plan, cfg.mower_width, 0.01)))
                if p.area >= LEFT_TUFT]
        pieces = [p.area for p in left]
        if best is not None and sum(pieces) > best_left - LEFT_GAIN:
            break  # narrower doesn't leave less (what's left is at corners, not between the lanes): the wider one
        best, best_left, best_spacing = plan, sum(pieces), spacing
        if max(pieces, default=0.0) < LEFT_PIECE and sum(pieces) < LEFT_TOTAL or hi - lo < 1e-9:
            break
        # narrower lanes only mow what's left between lanes: strips along them (not what's at corners, the loops, the
        # ends of lanes and turns)
        if not any(_strip_between_lanes(p, plan.angles) for p in left):
            break
    if best is None:
        raise error or PlanningError('no lane spacing fits')
    best.stats['planning_time'] = time.perf_counter() - t0
    best.stats['chosen']['mode'] = 'auto'
    return best


def _left_alone(free, loop_rings: List[List[XY]], passes, half: float) -> float:
    """(for lane_spacing_mode 'auto') m2 the loops (their rings) and the lanes alone leave, no turns, no drives: quick
    to compare."""
    bands = [LineString(ring + [ring[0]]).buffer(half, 4) for ring in loop_rings]
    bands += [LineString(Lane(0, x0, x1, lane.y).ends(a)).buffer(half, 4) for a, cells in passes for cell in cells
              for lane in cell for x0, x1 in lane.pieces()]
    return sum(p.area for p in polygons(subtract(free, bands)) if p.area >= LEFT_TUFT)


def _strip_between_lanes(piece, angles: Sequence[float]) -> bool:
    """Whether a piece left unmowed is a strip along the lanes (of one of the angles): three times as long as wide."""
    for a in angles:
        c, s = math.cos(a), math.sin(a)
        pts = list(piece.exterior.coords)
        along = [x * c + y * s for x, y in pts]
        across = [-x * s + y * c for x, y in pts]
        if max(along) - min(along) > 3.0 * (max(across) - min(across)):
            return True
    return False


def _completed(area: _Area, plan: CoveragePlan) -> CoveragePlan:
    """mop_up on the plan picked: what it leaves of the lawn as drawn mowed where that's worth it (complete.py)."""
    cfg = plan.config
    if not cfg.mop_up or plan.body_space is None or not plan.poses:
        return plan
    t0 = time.perf_counter()
    found = getattr(_BODY_SPACE, 'last', None)  # (the Footprint planned with, its turns worked out already)
    fp = found[2] if found is not None and found[2].spin_margin == cfg.spin_margin and \
        found[2].corners == tuple(cfg.body_outline) and found[1].wkb == plan.body_space.wkb else plan_footprint(plan)
    edges = dict(edge_distance=cfg.transit_edge_distance, edge_cost=cfg.transit_edge_cost)
    transit = TransitPlanner(plan.transit_space, cfg.transit_clearance, **edges)
    lawn = subtract(clean_polygon(area.boundary),
                    [clean_polygon(o) for o in list(area.obstacles) + list(area.keep_out)])
    field = polygonal(_lanes_field(plan.cells, 2.0 * cfg.turn_radius, 0.5 * cfg.lane_spacing).intersection(
        plan.remaining)) if cfg.headland_turns and plan.cells else None
    poses, segments, done = complete(cfg, fp, transit, plan.robot_space, plan.transit_space, lawn, plan.poses,
                                     plan.segments, field, _point(area.end))
    poses, segments = _turn_at(types.SimpleNamespace(body=fp.poses_fit), poses, segments)
    took = time.perf_counter() - t0
    stats = dict(plan.stats, mop_up=done, planning_time=plan.stats['planning_time'] + took,
                 timings=dict(plan.stats['timings'], mop_up=round(took, 4)), **_lengths(poses, segments))
    return dataclasses.replace(plan, poses=poses, segments=segments, stats=stats, section_starts=[
        s.start for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section])


class _Timer:
    def __init__(self, t0: float) -> None:
        self.t0, self.lap, self.timings = t0, t0, {}

    def took(self, key: str) -> None:
        now = time.perf_counter()
        self.timings[key] = round(now - self.lap, 4)
        self.lap = now


def _plan_coverage(boundary: Sequence[Sequence[float]], obstacles: Sequence[Sequence[Sequence[float]]] = (),
                   config: Optional[PlannerConfig] = None, start: Optional[Sequence[float]] = None,
                   end: Optional[Sequence[float]] = None,
                   drivable: Sequence[Sequence[Sequence[float]]] = (),
                   keep_out: Sequence[Sequence[Sequence[float]]] = (), margins: Sequence[Optional[float]] = (),
                   probe: Any = False, mowed=None, edge_slots: bool = True,
                   reach_over: Sequence[Sequence[Sequence[float]]] = ()) -> CoveragePlan:
    """Plans the coverage path of one area.

    boundary: [(x, y), ...] of the outline, obstacles: a list of those for no-go zones, all in metres in one flat frame
    (map). start: (x, y) or (x, y, yaw) of the mower, the path starts on the loop (or lane) closest to it (default: the
    outline's first point). end: (x, y) to finish close to, e.g. the docking station. drivable: areas the mower may
    drive through between the parts of this one but doesn't mow (navigation areas, other lawns). keep_out: where
    nothing goes on the way through those (inactive areas, ones not to mow), with the body checked recorded like the
    obstacles (see edges). reach_over: areas next to this one the mower drives on otherwise (navigation areas): with the
    body checked it may reach over the edge into them, the path keeps out of them (a slot by the edge they lie beyond is
    none, see _edge_gaps). margins: the obstacle_margin of each obstacle, then of each area to keep out of (None, or
    none given: the setting). mowed (a geometry, where the blade went on a mow): no loops and lanes, only moves over
    what that left (complete.leftovers), the one nearest start first; stats['leftovers'] says how much. edge_slots: the
    slots by the edge too narrow for the body closed (see _edge_gaps, plan_coverage tries both). Raises PlanningError
    when there's nothing to plan, or a setting or point is out of range (every coordinate within MAX_COORDINATE of the
    origin).
    """
    timer = _Timer(time.perf_counter())
    cfg = config or PlannerConfig()
    cfg.validate()
    if cfg.body and cfg.turn_radius == 0:  # (see turn_radius)
        cfg = dataclasses.replace(cfg, turn_radius=ZERO_TURN, turn_on_spot=True, lane_order='snake',
                                  turn_radius_auto=False)
    margin_of = margins_of(boundary, obstacles, drivable, keep_out, reach_over, margins, cfg)
    _check_coordinates('start or end', [p for p in (start, end) if p is not None])
    space, footprint = build_space(boundary, obstacles, drivable, keep_out, reach_over, margin_of, cfg, edge_slots)
    if cfg.waypoint_spacing > 0:
        # every square metre takes about 1 / lane spacing metres of path
        estimate = space.free.area / cfg.lane_spacing / cfg.waypoint_spacing
        if estimate > MAX_POSES:
            raise PlanningError(f'about {estimate:.0f} poses, more than {MAX_POSES}: lane spacing {cfg.lane_spacing} m '
                                f'and waypoint_spacing {cfg.waypoint_spacing} m are too small for '
                                f'{space.free.area:.0f} m2')
    timer.took('area')
    rings_of = ring_levels(space, cfg, timer.took)
    timer.took('remaining')
    angle = lane_angle(space.area, space.free, cfg, rings_of.lane_region, rings_of.remaining, space.robot_space)
    passes, zones = [], []
    if cfg.fill_pattern != 'concentric' and not cfg.skip_lanes and not rings_of.lane_region.is_empty:
        passes, zones = lane_sets(rings_of.lane_region, rings_of.remaining, space.robot_space, angle, cfg)
    narrow = rings_of.narrow
    if not narrow.is_empty and passes and not (cfg.skip_outline_loops or cfg.skip_obstacle_loops):
        narrow = narrow_left(narrow, passes, rings_of.remaining, 0.5 * cfg.mower_width)
    timer.took('lanes')
    loop_rings = rings_of.loop_rings(narrow)
    if probe is True:
        return _left_alone(space.free, loop_rings, passes, 0.5 * cfg.mower_width)
    if isinstance(probe, dict):  # (what _left_alone needs: plan_coverage may ask for this spacing's later)
        probe['alone'] = (space.free, loop_rings, passes, 0.5 * cfg.mower_width)
    route, start_xy = _new_route(cfg, space, footprint, rings_of, passes, start, end, boundary)
    if mowed is None:
        mow_parts(route, rings_of, passes, zones, narrow, [ring for kind, ring in rings(space.free) if kind == 'hole'],
                  start_xy, _point(end), {id(lane): f'cell-{pi}-{ci}' for pi, (_, cells) in enumerate(passes)
                                          for ci, cell in enumerate(cells) for lane in cell})
    poses, segments, done = _finish(route, cfg, space, footprint, mowed, start_xy, timer)
    return CoveragePlan(
        poses=poses, segments=segments, angle=angle, lane_spacing=cfg.lane_spacing, perimeter_loops=route.loops,
        lanes=route.lanes, stats=_stats(route, cfg, space, segments, poses, passes, zones, rings_of, done, timer),
        warnings=route.warnings, area=space.area, free_space=space.free, robot_space=space.drive_space,
        lane_region=rings_of.lane_region, remaining=rings_of.remaining, angles=[a for a, _ in passes] or [angle],
        transit_space=space.transit_space, body_space=space.body_space, config=cfg,
        section_starts=[s.start for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section],
        levels=rings_of.levels, narrow=narrow, cells=passes, origins=route.origins)


def _new_route(cfg: PlannerConfig, space, footprint, rings_of: Rings, passes, start, end,
               boundary) -> Tuple[_Route, XY]:
    """The route the path is put together on, and where it starts looking."""
    start_xy = _point(start)
    start_heading = float(start[2]) if start is not None and len(start) >= 3 and start[2] is not None else None
    if start_xy is None:
        corner = list(polygons(space.area))[0].exterior.coords[0]
        start_xy = (corner[0], corner[1])
    # turns and loops stay inside the mowing area
    edges = dict(edge_distance=cfg.transit_edge_distance, edge_cost=cfg.transit_edge_cost)
    area_space = TransitPlanner(space.drive_space, cfg.transit_clearance, **edges)
    transit = area_space if space.transit_space is space.drive_space else \
        TransitPlanner(space.transit_space, cfg.transit_clearance, **edges)
    route = _Route(cfg, transit, area_space, start_xy, start_heading, _point(end), footprint)
    if cfg.robot_outline is not None:  # (the way the outline's points go)
        ring = list(boundary)
        route.recorded = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(ring, ring[1:] + ring[:1])) > 0
    if not cfg.body:
        route.warnings.append(cfg.no_body)
    route.loop_counts = (rings_of.count, rings_of.islands) if rings_of.islands != rings_of.count else None
    # headland_turns: the lanes' field, where they lie but for what the loops mow and 2 turn_radius at their ends (where
    # the turns go, the lanes ending earlier for the body)
    field = polygonal(_lanes_field(passes, 2.0 * cfg.turn_radius, 0.5 * cfg.lane_spacing).intersection(
        rings_of.remaining)) if cfg.headland_turns and passes else Polygon()
    if not field.is_empty:
        route.field, route._field, route.headland = field, prep(field.buffer(-1e-3, 1)), transit.without(field)
        route.angle = passes[0][0]
        route.lane_ys = {a: sorted({lane.y for cell in cells for lane in cell}) for a, cells in passes}
    return route, start_xy


def _finish(route: _Route, cfg: PlannerConfig, space, footprint, mowed, start_xy: XY,
            timer: _Timer) -> Tuple[list, list, Dict[str, Any]]:
    """The route's path with its turns on the spot rounded off, the body's safety net, what a mow left (mowed), the
    turns on the spot marked: (poses, segments, {'fitted', 'leftover', 'turning', 'parts'}). mop_up comes later, on
    the plan picked (_completed)."""
    if not cfg.turn_on_spot:
        _smooth_spins(route, cfg)
    poses, segments = _assemble(route.segments)
    if footprint is not None and poses:
        poses, segments = _round_corners(route, poses, segments)
    timer.took('route')
    done: Dict[str, Any] = {'fitted': None, 'leftover': None, 'turning': ([], 0, 0.0),
                            'parts': len({s.section for s in segments})}
    if footprint is not None and cfg.body_fit and poses:
        poses, segments, done['fitted'] = _net(route, (poses, segments), space.free)
        timer.took('body')
    if mowed is not None:
        # what a mow left: moves over that alone, the one nearest the start (the dock) first
        if footprint is None:
            raise PlanningError('what a mow left is only planned with the body set (robot_width, robot_front, '
                                'robot_rear)')
        poses, segments, done['leftover'] = leftovers(cfg, footprint, route.transit, space, mowed, start_xy,
                                                      route.end)
        if not poses:
            raise NothingLeft(f'nothing left to mow where the mower gets to ({done["leftover"]["left_m2"]:.2f} m2 '
                              'left)')
        timer.took('leftovers')
    if route.field is not None and mowed is None:
        done['turning'] = _field_turns(route, poses, segments)
    if not poses:
        raise PlanningError('nothing to mow: the area is too small for one loop or lane')
    if footprint is not None:
        poses, segments = _turn_at(route, poses, segments)
    _warn(route, cfg, done)
    return poses, segments, done


def _net(route: _Route, planned, free) -> Tuple[list, list, Dict[str, Any]]:
    """_fit_body as the safety net (with the old loops' repairs, rings_old.fit_rings), what it changes counted by where
    it comes from (stats['body_fit']['origins'])."""
    planned, (poses, segments, fitted) = fit_rings(route, planned, free)
    fitted['origins'] = _origins(*planned, fitted)
    del fitted['spans']
    poses, segments = _round_corners(route, poses, segments)
    if fitted['fixed']:
        route.warnings.append(f'{fitted["fixed"]} places driven another way where the body would stick out, '
                              f'{fitted["skipped_m"]:.1f} m of loops and lanes left out there')
    if fitted['left']:
        jumps = ', '.join(f'({x:.2f}, {y:.2f})' for x, y in fitted['jumps'])
        route.warnings.append(f'{fitted["left"]} places left out where the body doesn\'t fit and no other way does'
                              + (f', the path jumps over them at {jumps}' if jumps else ''))
    if fitted['unchecked']:
        route.warnings.append('the path starts later or ends before where the body doesn\'t fit and no way does '
                              'from or to the dock\'s side: getting to or from ' + ', '.join(
                                  f'({x:.2f}, {y:.2f})' for x, y in fitted['unchecked']) + ' the body isn\'t '
                              'checked')
    return poses, segments, fitted


# where what the safety net changes comes from: the old loops (and the drives to them), the lanes, the turns and drives
# between lanes
ORIGINS = ('rings', 'swaths', 'connectors')


def _origins(poses, segments, fitted: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
    """{origin: {'fixed': ways in place, 'left': places left out, 'm': m of loops and lanes left out}} of
    _fit_body's places, each by the segment of the planned path where the body sticks out the most there."""
    out = {o: {'fixed': 0, 'left': 0, 'm': 0.0} for o in ORIGINS}
    for k, ((a, b), (x, y, m), worst) in enumerate(zip(fitted['spans'], fitted['places'], fitted['poses'])):
        o = min(range(max(0, a), min(b, len(poses) - 1) + 1),
                key=lambda i: math.hypot(poses[i].x - worst[0], poses[i].y - worst[1]), default=a)
        seg = next((s for s in segments if s.start <= o <= s.end), None)
        origin = 'connectors' if seg is None else 'rings' if seg.kind == PERIMETER or seg.maneuver == 'ring' or \
            seg.kind != LANE and seg.source.startswith('loop') else 'swaths' if seg.kind == LANE else 'connectors'
        out[origin]['fixed' if k < fitted['fixed'] else 'left'] += 1
        out[origin]['m'] = round(out[origin]['m'] + m, 2)
    return out


def _warn(route: _Route, cfg: PlannerConfig, done: Dict[str, Any]) -> None:
    turning = done['turning'][0]
    if turning:
        route.warnings.append(f'{len(turning)} places where a drive turns in the lanes\' field, no way along the '
                              'headland fits there: at ' + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in turning))
    if route.counts.get('lanes_shortened'):
        route.warnings.append(f'{route.counts["lanes_shortened"]} lanes along an edge shortened or left out: with '
                              f'blade_offset {cfg.blade_offset} m the centre runs that far off the blade\'s line, '
                              'more perimeter loops leave room for it')
    if route.counts.get('lanes_body'):
        route.warnings.append(f'{route.counts["lanes_body"]} lanes shortened or left out where the body would stick '
                              'out at their ends')
    if done['parts'] > 1:
        route.warnings.append(f'the area has {done["parts"]} parts with no way between them: the path '
                              'jumps from one to the next')


def _lengths(poses, segments) -> Dict[str, Any]:
    """The path's stats that change with it: its sections, m of each kind and all together, the poses."""
    lengths: Dict[str, float] = {PERIMETER: 0.0, LANE: 0.0, TURN: 0.0, TRANSIT: 0.0}
    for seg in segments:
        lengths[seg.kind] += path_length(poses[seg.start:seg.end + 1])
    return {'sections': len([i for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section]),
            'path_length': sum(lengths.values()), 'perimeter_length': lengths[PERIMETER], 'lane_length': lengths[LANE],
            'turn_length': lengths[TURN], 'transit_length': lengths[TRANSIT], 'poses': len(poses)}


def _stats(route: _Route, cfg: PlannerConfig, space, segments, poses, passes, zones, rings_of: Rings,
           done: Dict[str, Any], timer: _Timer) -> Dict[str, Any]:
    stats: Dict[str, Any] = {
        'mop_up': None,
        'leftovers': done['leftover'],
        **_lengths(poses, segments),
        'perimeter_loops': len({s.source for s in segments if s.kind == PERIMETER}),
        'loops': len(route.loops),
        'lanes': len(route.lanes),
        'cells': sum(len(cells) for _, cells in passes),
        'area': space.free.area,
        'edge_slots': len(space.slots),
        'planning_time': time.perf_counter() - timer.t0,
        'timings': timer.timings,
        'chosen': {'lane_spacing': round(cfg.lane_spacing, 4), 'perimeter_passes': rings_of.count, 'mode': 'fixed'},
    }
    for key in ('turns_u_turn', 'turns_bulb', 'turns_detour', 'turns_pivot', 'turns_fallback'):
        stats[key] = route.counts.get(key, 0)
    if cfg.split_zones:
        # the zones mowed another way than the rest: their lanes' direction, how many, their outlines
        stats['zones'] = [{'angle': round(a, 6), 'lanes': sum(len(cell) for cell in cells),
                           'outlines': [[[round(x, 3), round(y, 3)] for x, y in poly.exterior.coords[:-1]]
                                        for poly in polygons(zone)]}
                          for (a, zone), (_, cells) in zip(zones, passes[1:])]
    if done['fitted'] is not None:
        stats['body_fit'] = done['fitted']
    if cfg.headland_turns:
        turning, arcs, deepest = done['turning']
        stats['headland_turns'] = {'turns_in_field': len(turning), 'places': turning, 'arcs_in_field': arcs,
                                   'arc_depth': deepest, 'crossings': route.counts.get('crossings', 0)}
    return stats
