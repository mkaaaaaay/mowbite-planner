"""Coverage path for one mowing area, without any ROS: shapely in, a list of poses out (plan_coverage).

The stages: the spaces from the map (space.py), the loops (levels.py, rings.py) and the lanes (swaths.py), the path put
together (assemble.py: stripes.py, turns.py, order.py, drive.py), the body's check as a safety net (bodyfit.py), counted
by where what it changes comes from, the plan as it's handed on (handover.py: the blade on along the drives, the poses
it needs). The settings and what a plan is made of are in config.py.
"""

import dataclasses
import math
import multiprocessing
import threading
import time
import types
from concurrent.futures import Future, ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from typing import Any, Dict, Optional, Sequence, Tuple

from shapely.geometry import Polygon
from shapely.prepared import prep

from .assemble import mow_parts
from .bodyfit import _assemble, _field_turns, _lanes_field, _round_corners, _turn_at
from .complete import complete, leftovers
from .config import (_BODY_SPACE, LANE, LEFT_PIECE, LEFT_TOTAL, LEFT_TUFT, PERIMETER, SPACING_CANDIDATES, TRACK, XY,
                     RECORDED_CENTRE, ZERO_TURN, CoveragePlan, NothingLeft, PlannerConfig, PlanningError)
from .geometry import QUAD_SEGS, clean_polygon, polygonal, polygons, rings, subtract, union
from .handover import _blade_on, _compact, _lengths, _ring_ways
from .lanes import lane_angle
from .objective import LEFT_GAIN
from .progress import Progress, Report
from .levels import Rings, narrow_left, ring_levels
from .loopfit import _smooth_spins, fit_rings
from .rings import _Route
from .space import _check_coordinates, build_space, margins_of, recorded_space
from .starts import Arrives, reached_starts
from .spacing import left_alone, left_whole, strip_between_lanes, turn_places
from .sweep import plan_footprint, swept_area
from .swaths import lane_sets
from .transit import TransitPlanner
from .footprint import Blade, Footprint

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
    """_plan_coverage (in a process of the pool): the plan and its probe as _plan_coverage left it. completed: with
    mop_up (_completed)."""
    kw = dict(kw)
    if not kw.pop('completed', False):
        return _plan_coverage(*args, **kw), kw.get('probe')
    boundary, obstacles, _, start, end, drivable, keep_out, margins = args
    area = _Area(boundary, obstacles, start, end, drivable, keep_out, margins, kw.get('reach_over', ()),
                 kw.get('unmowed', ()))
    return _final(area, _completed(area, _plan_coverage(*args, **kw))), kw.get('probe')


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
                  reach_over: Sequence[Sequence[Sequence[float]]] = (),
                  unmowed: Sequence[Sequence[Sequence[float]]] = (), arrives: Optional[Arrives] = None,
                  progress: Optional[Report] = None) -> CoveragePlan:
    """Plans the coverage path of one area, see _plan_coverage. With lane_spacing_mode 'auto' the best of
    SPACING_CANDIDATES lane spacings (see PlannerConfig), the same input always picks the same. stats['chosen']: the
    lane spacing and loops it was planned with. mowed (a geometry: where the blade went already, see
    report.track_mowed): only what that left, see _plan_coverage. With set_workers the plans in processes of their
    own. The slots by the edge stay open (the loops go round one too narrow for the body, levels.body_free): the plan's
    free_space, robot_space and transit_space are the map's as drawn. arrives(pose) (a robot that drives to the plan's
    sections itself, MowBite OS: whether it gets to the pose facing its heading): every section starts where it does,
    see starts.reached_starts. progress(stage, done, of): how far it got (progress.Progress)."""
    cfg = config or PlannerConfig()
    cfg.validate()
    area = _Area(boundary, obstacles, start, end, drivable, keep_out, margins, reach_over, unmowed)
    auto = mowed is None and cfg.lane_spacing_mode == 'auto'
    area.progress, area.stage = Progress(progress, 1 if mowed is not None else SPACING_CANDIDATES + 3 if auto else 3), \
        'spacing' if auto else 'route'
    area.progress.at('angles', 0)
    if mowed is not None:
        plan = _compact(area.one(cfg, mowed=mowed))
    elif not auto:
        plan = _final(area, _completed(area, area.plan(cfg)))
    else:
        plan = _auto(area, cfg)
    plan = plan if arrives is None else reached_starts(plan, arrives)
    area.progress.at('done', area.progress.of)
    return plan


class _Area:
    """One area's input, planned with a config: in a process of the pool where there's one."""

    def __init__(self, boundary, obstacles, start, end, drivable, keep_out, margins, reach_over, unmowed=()) -> None:
        self.boundary, self.obstacles, self.start, self.end = boundary, obstacles, start, end
        self.drivable, self.keep_out, self.margins, self.reach_over = drivable, keep_out, margins, reach_over
        self.unmowed = unmowed
        self._lawn = None
        self.progress, self.stage = Progress(None, 1), 'route'  # (plan_coverage's, a plan planned through a step)

    def lawn(self):
        """The lawn as drawn: the area but its obstacles and the areas to keep out of or not to mow."""
        if self._lawn is None:
            self._lawn = subtract(clean_polygon(self.boundary), [clean_polygon(o) for o in list(self.obstacles) +
                                                                 list(self.keep_out) + list(self.unmowed)])
        return self._lawn

    def one(self, c: PlannerConfig, **more: Any) -> Any:
        return _plan_coverage(self.boundary, self.obstacles, c, self.start, self.end, self.drivable, self.keep_out,
                              self.margins, reach_over=self.reach_over, unmowed=self.unmowed, **more)

    def later(self, c: PlannerConfig, **more: Any) -> Tuple['Future', tuple, Dict[str, Any]]:
        args = (self.boundary, self.obstacles, c, self.start, self.end, self.drivable, self.keep_out, self.margins)
        kw = dict(reach_over=self.reach_over, unmowed=self.unmowed, **more)
        return _later(args, kw), args, kw

    def got(self, planning: Tuple['Future', tuple, Dict[str, Any]], probe: Any) -> CoveragePlan:
        plan, filled = _result(*planning)
        if isinstance(probe, dict) and filled is not probe:  # (filled in another process)
            probe.update(filled)
        self.progress.at(self.stage)
        return plan

    def plan(self, c: PlannerConfig, probe: Any = False) -> CoveragePlan:
        """The plan with c (in a process of the pool where there's one). The slots by the edge stay open: where the
        body doesn't fit across one the loops go round it (levels.body_free), the lanes and the completion stage
        check the body themselves."""
        return self.got(self.later(c, probe=probe), probe)


def _auto(area: _Area, cfg: PlannerConfig) -> CoveragePlan:
    """lane_spacing_mode 'auto', with mop_up: the widest lane spacing of SPACING_CANDIDATES in spacing_range after which
    a narrower one doesn't leave LEFT_GAIN less unmowed, the blade TRACK narrower each side (the mower off its way), and
    its plan not LEFT_GAIN more with the blade as wide as it is (the lanes turned another way, their ends where nothing
    mows them). The widest asked (spacing_asked, wider than the blade TRACK narrower covers) where its plan leaves
    LEFT_GAIN less with the whole blade: planned at once with the others where there are processes for it (see
    set_workers)."""
    t0 = time.perf_counter()
    lo, hi = cfg.spacing_range
    widest = cfg.spacing_asked[1]
    best, best_left, best_whole, best_spacing, error = None, None, None, 0.0, None
    probed: Dict[float, float] = {}
    kept: Dict[float, Dict[str, Any]] = {}  # of the spacings planned through, what quick() needs

    def fixed(spacing: float) -> PlannerConfig:
        return dataclasses.replace(cfg, lane_spacing_mode='fixed', overlap=1.0 - spacing / cfg.mower_width)

    def quick(spacing: float) -> float:
        if spacing not in probed:
            probed[spacing] = left_alone(*kept[spacing]['alone']) if 'alone' in kept.get(spacing, {}) else \
                area.one(fixed(spacing), probe=True)
        return probed[spacing]

    def gain(spacing: float, narrower: float) -> bool:
        # a narrower spacing is only planned through (turns, drives) where its loops and lanes alone leave LEFT_GAIN
        # less than the last one's
        return quick(narrower) <= quick(spacing) - LEFT_GAIN

    # (on real gardens the narrower lanes' plan left up to 0.2 m2 more than the widest's by the turns and corners now
    # and then, in the simulation too)
    wide = area.later(fixed(widest), completed=True) if widest > hi + 1e-9 else None
    spacings = [hi - (hi - lo) * i / max(1, SPACING_CANDIDATES - 1) for i in range(SPACING_CANDIDATES)]
    # (with processes for it and none for the widest's the second at once too: planned through more often than not;
    # three at once on a CM4's 4 cores took longer than two)
    ahead = {s: area.later(fixed(s), probe=kept.setdefault(s, {})) for s in spacings[1:2] if _pool() and not wide}
    for i, spacing in enumerate(spacings):
        try:
            if best is not None and not gain(best_spacing, spacing):
                break
            plan = area.got(ahead[spacing], kept[spacing]) if spacing in ahead else \
                area.plan(fixed(spacing), probe=kept.setdefault(spacing, {}))
        except PlanningError as e:  # this spacing doesn't fit (too many poses), a narrower one may not either
            error = error or e
            if best is None and _pool():  # (the narrower ones are planned through till one fits: those at once)
                ahead.update({s: area.later(fixed(s), probe=kept.setdefault(s, {})) for s in spacings[i + 1:]
                              if s not in ahead})
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
        left = [p for p in polygons(plan.free_space.difference(swept_area(plan, cfg.mower_width - 2.0 * TRACK, 0.01)))
                if p.area >= LEFT_TUFT]
        pieces = [p.area for p in left]
        mine = None
        if best is not None:
            if sum(pieces) > best_left - LEFT_GAIN:
                break  # narrower doesn't leave less (what's left is at corners, not between the lanes): the wider one
            best_whole, mine = left_whole(best) if best_whole is None else best_whole, left_whole(plan)
            if mine > best_whole + LEFT_GAIN:
                break  # (nor where it leaves more with the whole blade)
        best, best_left, best_whole, best_spacing = plan, sum(pieces), mine, spacing
        if max(pieces, default=0.0) < LEFT_PIECE and sum(pieces) < LEFT_TOTAL or hi - lo < 1e-9:
            break
        # narrower lanes only mow what's left between lanes: strips along them (not what's at corners, the loops, the
        # ends of lanes and turns)
        if not any(strip_between_lanes(p, plan.angles) for p in left):
            break
    for planning in ahead.values():
        planning[0].cancel()  # (where it isn't planned through: not begun, else the process goes on to the next)
    best, failed = _or_widest(area, best, wide)
    error = error or failed
    if best is None:
        raise error or PlanningError('no lane spacing fits')
    best.stats['planning_time'] = time.perf_counter() - t0
    best.stats['chosen']['mode'] = 'auto'
    return best


def _or_widest(area: _Area, best: Optional[CoveragePlan], wide) -> Tuple[Optional[CoveragePlan],
                                                                         Optional[PlanningError]]:
    """(lane_spacing_mode 'auto') best with mop_up as handed out, or the widest asked's (wide: being planned) where it
    has fewer places in spin_margin (turn_places: a fixed rule before what's left) or leaves LEFT_GAIN less with the
    whole blade (left_whole); and why the widest's failed."""
    best = _final(area, _completed(area, best)) if best is not None else None
    if wide is None:
        return best, None
    try:
        other = area.got(wide, None)
    except PlanningError as e:
        return best, e
    fp = _footprint(other) if best is not None else None
    if best is None or turn_places(other, fp) < turn_places(best, fp) or \
            left_whole(other) + LEFT_GAIN < left_whole(best):
        return other, None
    return best, None


def _completed(area: _Area, plan: CoveragePlan) -> CoveragePlan:
    """mop_up on the plan picked: what it leaves of the lawn as drawn mowed where that's worth it (complete.py)."""
    cfg = plan.config
    if not cfg.mop_up or plan.body_space is None or not plan.poses:
        return plan
    area.progress.at('mop_up', area.progress.of - 2)
    t0 = time.perf_counter()
    fp = _footprint(plan)
    edges = dict(edge_distance=cfg.transit_edge_distance, edge_cost=cfg.transit_edge_cost)
    transit = TransitPlanner(plan.transit_space, cfg.transit_clearance, **edges)
    lawn = area.lawn()
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


def _final(area: _Area, plan: CoveragePlan) -> CoveragePlan:
    """The plan as it's handed out: the drives between pieces of loops of their kind, the blade on along them where it
    stays on the lawn, the poses compacted (handover)."""
    area.progress.at('route', area.progress.of - 1)
    lawn = area.lawn()
    return _compact(_blade_on(_ring_ways(plan, lawn), lawn))


def _footprint(plan: CoveragePlan) -> Optional[Footprint]:
    """The plan's Footprint: the one it was planned with where this thread has it (its turns worked out already). None
    without the body."""
    cfg = plan.config
    if cfg is None or not cfg.body or plan.body_space is None:
        return None
    found = getattr(_BODY_SPACE, 'last', None)
    blade = Blade.of(plan.unmowed, cfg)
    return found[2] if found is not None and found[2].spin_margin == cfg.spin_margin and \
        found[2].corners == tuple(cfg.body_outline) and found[1].wkb == plan.body_space.wkb and \
        (found[2].blade and found[2].blade.key) == (blade and blade.key) else plan_footprint(plan)


class _Timer:
    def __init__(self, t0: float) -> None:
        self.t0, self.lap, self.timings = t0, t0, {}

    def took(self, key: str) -> None:
        now = time.perf_counter()
        self.timings[key] = round(now - self.lap, 4)
        self.lap = now


def _as_recorded(route: _Route, cfg: PlannerConfig, footprint, outlines, away_from, margin_of) -> None:
    """The outermost loop the way the outline was recorded (rings.drive_ring): the body where it was then, the centre
    no further than RECORDED_CENTRE past the lines, a negative perimeter_offset that far (drive_space reaches
    body_tolerance past the first loop, with edge_margin 0 past the lines: rounding a corner cut it out)."""
    route.recorded_fp = Footprint(recorded_space(outlines, away_from, margin_of, cfg), cfg.body_outline, 0.0,
                                  blade=footprint.blade)
    out = max(RECORDED_CENTRE, -(cfg.perimeter_offset or 0.0))
    route.recorded_centre = union([clean_polygon(r) for r in outlines]).buffer(out, QUAD_SEGS)


def _plan_coverage(boundary: Sequence[Sequence[float]], obstacles: Sequence[Sequence[Sequence[float]]] = (),
                   config: Optional[PlannerConfig] = None, start: Optional[Sequence[float]] = None,
                   end: Optional[Sequence[float]] = None,
                   drivable: Sequence[Sequence[Sequence[float]]] = (),
                   keep_out: Sequence[Sequence[Sequence[float]]] = (), margins: Sequence[Optional[float]] = (),
                   probe: Any = False, mowed=None, edge_slots: bool = False,
                   reach_over: Sequence[Sequence[Sequence[float]]] = (),
                   unmowed: Sequence[Sequence[Sequence[float]]] = ()) -> CoveragePlan:
    """Plans the coverage path of one area.

    boundary: [(x, y), ...] of the outline, obstacles: a list of those for no-go zones, all in metres in one flat frame
    (map). start: (x, y) or (x, y, yaw) of the mower, the path starts on the loop (or lane) closest to it (default: the
    outline's first point). end: (x, y) to finish close to, e.g. the docking station. drivable: areas the mower may
    drive through between the parts of this one but doesn't mow (navigation areas, other lawns). keep_out: where
    nothing goes on the way through those (inactive areas, ones not to mow), with the body checked recorded like the
    obstacles (see edges). reach_over: areas next to this one the mower drives on otherwise (navigation areas): with the
    body checked it may reach over the edge into them, the path keeps out of them (a slot by the edge they lie beyond is
    none, see _edge_gaps). unmowed: areas next to this one not to mow (mowing areas set so): like reach_over, but the
    blade keeps off them too (the first loop half the mower width off their lines) and nothing goes over them on the
    way (mkay 31.10.). margins: the obstacle_margin of each obstacle, then of each area to keep out of (None, or
    none given: the setting). mowed (a geometry, where the blade went on a mow): no loops and lanes, only moves over
    what that left (complete.leftovers), the one nearest start first; stats['leftovers'] says how much. edge_slots: the
    slots by the edge too narrow for the body closed in free (see _edge_gaps; plan_coverage leaves that to the loops'
    levels). Raises PlanningError when there's nothing to plan, or a setting or point is out of range (every coordinate
    within MAX_COORDINATE of the origin).
    """
    timer = _Timer(time.perf_counter())
    cfg = config or PlannerConfig()
    cfg.validate()
    if cfg.body and cfg.turn_radius == 0:  # (see turn_radius)
        cfg = dataclasses.replace(cfg, turn_radius=ZERO_TURN, turn_on_spot=True, lane_order='snake')
    reach_over = list(reach_over) + list(unmowed)
    margin_of = margins_of(boundary, obstacles, drivable, keep_out, reach_over, margins, cfg)
    _check_coordinates('start or end', [p for p in (start, end) if p is not None])
    space, footprint = build_space(boundary, obstacles, drivable, keep_out, reach_over, margin_of, cfg, edge_slots,
                                   unmowed)
    if cfg.waypoint_spacing > 0:
        # every square metre takes about 1 / lane spacing metres of path
        estimate = space.free.area / cfg.lane_spacing / cfg.waypoint_spacing
        if estimate > MAX_POSES:
            raise PlanningError(f'about {estimate:.0f} poses, more than {MAX_POSES}: lane spacing {cfg.lane_spacing} m '
                                f'and waypoint_spacing {cfg.waypoint_spacing} m are too small for '
                                f'{space.free.area:.0f} m2')
    timer.took('area')
    rings_of = ring_levels(space, cfg, timer.took, footprint)
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
        return left_alone(space.free, loop_rings, passes, 0.5 * cfg.mower_width)
    if isinstance(probe, dict):  # (what _left_alone needs: plan_coverage may ask for this spacing's later)
        probe['alone'] = (space.free, loop_rings, passes, 0.5 * cfg.mower_width)
    route, start_xy = _new_route(cfg, space, footprint, rings_of, passes, start, end, boundary)
    if footprint is not None and cfg.edges == 'recorded' and rings_of.owners is not None:
        _as_recorded(route, cfg, footprint, [boundary] + list(drivable) + list(reach_over),
                     list(obstacles) + list(keep_out), margin_of)
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
        levels=rings_of.levels, narrow=narrow, cells=passes, origins=route.origins,
        recorded_space=route.recorded_fp.space if route.recorded_fp is not None else None,
        recorded_loops=sorted(route.recorded_loops), unmowed=space.unmowed)


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
    if cfg.body:  # (the way the outline's points go; mkay 31.10.: also without robot_outline)
        ring = list(boundary)
        route.recorded = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(ring, ring[1:] + ring[:1])) > 0
    if not cfg.body:
        route.warnings.append(cfg.no_body)
    route.loop_counts = (rings_of.count, rings_of.islands) if rings_of.islands != rings_of.count else None
    route.owners, route.ring_counts = rings_of.owners, (rings_of.count, rings_of.islands, rings_of.most)
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
    """_fit_body as the safety net (with the loops' repairs, loopfit.fit_rings), what it changes counted by where
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
