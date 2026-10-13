"""Coverage path for one mowing area, without any ROS: shapely in, a list of poses out.

1. Perimeter: loops along the outline, the first one mower_width / 2 inside it (perimeter_offset),
   every further one a lane spacing further in. Obstacles get theirs on the same go: the loops are the
   rings of the mowable area (outline minus obstacles) shrunk step by step, so where an obstacle comes
   close to the outline the loops go around both instead of crossing each other.
2. Remaining area: the area minus the obstacles and the strips the loops mow.
3. Lanes: parallel lines across what's left, at `angle`, lane spacing apart. Their centres stay inside
   the area shrunk by perimeter_offset + perimeter_passes * spacing, which is the remaining area shrunk
   by mower_width / 2 - overlap: lanes and the innermost loop overlap by the lane overlap, and the
   loops leave a headland for the turns. Where part of the area ends between two rows (an edge running
   along the lanes), the neighbouring row's lane is carried on through the headland, so nothing of the
   remaining area is left between the lanes and the loops.
4. Route: the lanes are split into cells where an obstacle (or a bend of the outline) splits a row. Each
   cell is mowed in a zigzag with gentle turns (maneuvers.py), and the next cell is the closest one not
   mowed yet, reached on the shortest way through the free space, along obstacles and the perimeter
   (transit.py). lane_order 'skip' mows every k-th lane of a cell, then every k-th starting one further,
   and so on, with k so that lanes k apart leave room for a plain U-turn of turn_radius: forwards only,
   for controllers that can't back up (OpenMower's FTC runs with forward_only). With headland_turns the drives
   and turns keep out of the lanes' field (what the loops leave, but the lanes' ends): along the headland, across
   the field only straight.

The settings and what a plan is made of are in config.py, the lanes (steps 3 and 4's cells) in lanes.py, the route
in route.py, the path made to fit the body in bodyfit.py; this module puts them together (plan_coverage).
"""

import bisect
import dataclasses
import hashlib
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely import wkb
from shapely.geometry import LineString, Point, Polygon
from shapely.prepared import prep

from .bodyfit import (_assemble, _field_turns, _fit_body, _lanes_field, _leftovers, _mop_up, _round_corners,
                      _turn_at, _unmowed)
from .config import (_BODY_SPACE, _BODY_SPACES, _KEPT_LOCK, LANE, LEFT_PIECE, LEFT_TOTAL, LEFT_TUFT, LOOP_GAIN,
                     LOOP_LEFT, MAX_BLADE_OFFSET, MAX_ROWS, ON_EDGE, PERIMETER, SPACING_CANDIDATES, SWEPT_PIECE,
                     TRANSIT, TURN, TURN_REACH, XY, CoveragePlan, Lane, PlannerConfig, PlanningError, SegmentInfo,
                     _keep)
from .footprint import Footprint
from .geometry import QUAD_SEGS, clean_polygon, inset, polygonal, polygons, rings, subtract, union
from .lanes import bend_lanes, lane_angle, scan_lanes, split_cells, zone_angles
from .maneuvers import path_length
from .mathutil import Pose2D
from .route import _driven, _pieces, _Route, _smooth_spins
from .transit import TransitPlanner

# a plan this big is a mistake (a lane spacing in mm, a waypoint every mm), not a lawn: about 200 km of path
MAX_POSES = 2000000
# m, coordinates further out are a mistake (or the wrong frame), not a lawn
MAX_COORDINATE = 1e7
# m2, a narrower spacing has to leave this much less unmowed to be taken
LEFT_GAIN = 0.05


# entry point -----------------------------------------------------------------------------------------------


def _point(p: Optional[Sequence[float]]) -> Optional[XY]:
    return (float(p[0]), float(p[1])) if p is not None and len(p) >= 2 else None


def _check_coordinates(what: str, points: Sequence[Sequence[float]]) -> None:
    """Refuses points that are no numbers or lie absurdly far out (GEOS can take forever on those)."""
    for p in points:
        # false for nan too
        if not all(abs(c) <= MAX_COORDINATE for c in p[:2]):
            raise PlanningError(f'{what} has a point that is no number or more than {MAX_COORDINATE:.0f} m from '
                                f'the map origin: {list(p[:2])}')


def plan_coverage(boundary: Sequence[Sequence[float]], obstacles: Sequence[Sequence[Sequence[float]]] = (),
                  config: Optional[PlannerConfig] = None, start: Optional[Sequence[float]] = None,
                  end: Optional[Sequence[float]] = None,
                  drivable: Sequence[Sequence[Sequence[float]]] = (),
                  keep_out: Sequence[Sequence[Sequence[float]]] = (),
                  margins: Sequence[Optional[float]] = (), mowed=None) -> CoveragePlan:
    """Plans the coverage path of one area, see _plan_coverage. With lane_spacing_mode 'auto' the best of
    SPACING_CANDIDATES lane spacings (see PlannerConfig), the same input always picks the same. stats['chosen']: the
    lane spacing and loops it was planned with. mowed (a geometry: where the blade went already, see
    report.track_mowed): only what that left, see _plan_coverage."""
    cfg = config or PlannerConfig()
    cfg.validate()
    if cfg.lane_spacing_mode != 'auto' or mowed is not None:
        return _plan_coverage(boundary, obstacles, cfg, start, end, drivable, keep_out, margins, mowed=mowed)
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
                _plan_coverage(boundary, obstacles, fixed(spacing), start, end, drivable, keep_out, margins, probe=True)
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
            plan = _plan_coverage(boundary, obstacles, fixed(spacing), start, end, drivable, keep_out, margins,
                                  probe=kept[spacing])
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


def _plan_coverage(boundary: Sequence[Sequence[float]], obstacles: Sequence[Sequence[Sequence[float]]] = (),
                   config: Optional[PlannerConfig] = None, start: Optional[Sequence[float]] = None,
                   end: Optional[Sequence[float]] = None,
                   drivable: Sequence[Sequence[Sequence[float]]] = (),
                   keep_out: Sequence[Sequence[Sequence[float]]] = (), margins: Sequence[Optional[float]] = (),
                   probe: Any = False, mowed=None) -> CoveragePlan:
    """Plans the coverage path of one area.

    boundary: [(x, y), ...] of the outline, obstacles: a list of those for no-go zones, all in metres in
    one flat frame (map). start: (x, y) or (x, y, yaw) of the mower, the path starts on the loop (or
    lane) closest to it (default: the outline's first point). end: (x, y) to finish close to, e.g. the
    docking station. drivable: areas the mower may drive through between the parts of this one but doesn't
    mow (navigation areas, other lawns). keep_out: where nothing goes on the way through those (inactive areas, ones
    not to mow), with the body checked recorded like the obstacles (see edges). margins: the obstacle_margin of each
    obstacle, then of each area to keep out of (None, or none given: the setting). mowed (a geometry, where the blade
    went on a mow): no loops and lanes, only passes over what that left (bodyfit._leftovers), the one nearest start
    first; stats['leftovers'] says how much. Raises PlanningError when there's nothing to plan, or a setting or point is
    out of range (every coordinate within MAX_COORDINATE of the origin).
    """
    t0 = time.perf_counter()
    cfg = config or PlannerConfig()
    cfg.validate()
    _check_coordinates('the boundary', boundary)
    for o in obstacles:
        _check_coordinates('an obstacle', o)
    for d in drivable:
        _check_coordinates('a drivable area', d)
    for k in keep_out:
        _check_coordinates('a keep out area', k)
    if len(margins) > len(obstacles) + len(keep_out):
        raise PlanningError(f'{len(margins)} margins for {len(obstacles) + len(keep_out)} obstacles and areas to keep '
                            'out of')
    margin_of = [cfg.obstacle_margin if m is None else m for m in margins]
    margin_of += [cfg.obstacle_margin] * (len(obstacles) + len(keep_out) - len(margin_of))
    for m in margin_of:
        if isinstance(m, bool) or not isinstance(m, (int, float)) or not 0.0 <= m <= MAX_BLADE_OFFSET:
            raise PlanningError(f'a margin must be a number 0..{MAX_BLADE_OFFSET}, got {m!r}')
    _check_coordinates('start or end', [p for p in (start, end) if p is not None])
    half = 0.5 * cfg.mower_width
    # the blade's first loop, the mower's centre is `off` further in (blade_offset, the loops go round with the
    # blade on their outer side)
    first = cfg.blade_loop_offset
    off = abs(cfg.blade_offset)
    centre = first + off
    spacing = cfg.lane_spacing
    concentric = cfg.fill_pattern == 'concentric'

    area = clean_polygon(boundary)
    if area.is_empty:
        raise PlanningError('the boundary needs at least 3 points that enclose an area')
    # with the body checked the loops keep it an obstacle's margin (an area's to keep out of) off its real edge: the
    # obstacle that much bigger where that's further than they keep off the outline
    away_from = [clean_polygon(w) for w in list(obstacles) + list(keep_out)]
    keep = [m + cfg.simplify_tolerance for m in margin_of]
    grown = [w.buffer(k - centre, QUAD_SEGS) if cfg.body and k > centre + 1e-9 else w for w, k in zip(away_from, keep)]
    # (and the gaps between two too narrow for the body, see _gaps)
    gaps = [g for g in _gaps(away_from, margin_of, [max(k, centre) for k in keep], cfg) if g.intersects(area)] \
        if cfg.body else []
    blocked = union(grown[:len(obstacles)] + gaps)
    free = polygonal(area.difference(blocked)) if not blocked.is_empty else area
    if cfg.simplify_tolerance > 0 and not free.is_empty:
        simple = free.simplify(cfg.simplify_tolerance, preserve_topology=True)
        # simplifying a very jagged outline can make it cross itself, then it stays as it is
        if simple.is_valid:
            free = polygonal(simple)
    if free.is_empty:
        raise PlanningError('the obstacles cover the whole area')
    # with the body checked the loops go round an area to keep out of like round an obstacle (see edges)
    if cfg.body and keep_out:
        free = polygonal(free.difference(union(grown[len(obstacles):])))
    # slivers smaller than the blade (where a noisy outline crosses itself) aren't parts of their own
    free = polygonal(union(p for p in polygons(free) if p.area >= half * half))
    if free.is_empty:
        raise PlanningError('the areas to keep out of cover the whole area')
    body_space, footprint = _body_space([boundary] + list(drivable), list(obstacles) + list(keep_out), margin_of, cfg) \
        if cfg.body else (None, None)
    robot_space = inset(free, centre)
    if robot_space.is_empty:
        raise PlanningError(f'the area is too narrow for a first loop {centre} m inside the outline')
    blade_space = robot_space if off == 0 else inset(free, first)
    if cfg.waypoint_spacing > 0:
        # every square metre takes about 1 / lane spacing metres of path
        estimate = free.area / spacing / cfg.waypoint_spacing
        if estimate > MAX_POSES:
            raise PlanningError(f'about {estimate:.0f} poses, more than {MAX_POSES}: lane spacing {spacing} m and '
                                f'waypoint_spacing {cfg.waypoint_spacing} m are too small for {free.area:.0f} m2')
    timings: Dict[str, float] = {}
    lap = [t0]

    def took(key: str) -> None:
        now = time.perf_counter()
        timings[key] = round(now - lap[0], 4)
        lap[0] = now

    took('area')

    # 1. perimeter loops, level 0 runs perimeter_offset inside the edges. concentric goes on until
    # nothing is left, the last bit (narrower than a lane spacing) gets one more loop halfway in
    count = cfg.passes
    # obstacle_passes: the islands (the holes of the area) with loops of their own number. Level k then is the outline
    # min(k, count) loops in and the islands min(k, islands) loops out: past its number a side stays where its lanes
    # would begin, its rings there aren't driven (see _Route.left_out)
    islands = cfg.island_passes
    most = max(count, islands)
    # concentric loops meet at the corners: at a right angle the corners of two loops are spacing * sqrt(2)
    # apart, which the blade only covers up to mower_width / sqrt(2)
    loop_spacing = min(spacing, cfg.mower_width / math.sqrt(2.0)) if concentric else spacing
    shell = holes_in = None
    if islands != count:
        shell = polygonal(union(Polygon(p.exterior) for p in polygons(free)))
        holes_in = polygonal(shell.difference(free))

    def shrunk(out: int, isl: int):
        # the area with its outline `out` loops in and its islands `isl` loops out
        if out == isl:
            return blade_space if out == 0 else inset(free, first + out * loop_spacing)
        outer = inset(shell, first + out * loop_spacing)
        return outer if holes_in.is_empty or outer.is_empty else polygonal(outer.difference(
            holes_in.buffer(first + isl * loop_spacing, QUAD_SEGS)))

    def level_at(k: int):
        return shrunk(min(k, count), min(k, islands)) if k < most else shrunk(count + k - most, islands + k - most)

    def driven(k: int, kind: str) -> bool:  # (whether the ring of loop level k is driven: not past its side's number)
        return k >= most or k < (islands if kind == 'hole' else count)

    levels = []
    while concentric or len(levels) < most:
        level = level_at(len(levels))
        if level.is_empty:
            break
        levels.append(level)
        if len(levels) > MAX_ROWS:
            raise PlanningError('too many loops: the area is too big for this lane spacing')
    # what the route drives the loops of: the levels, but with more loops round the islands than along the outline those
    # further ones each round its island alone (the island that much bigger). Where an island lies by the edge the
    # level's ring runs into the outline's there: the loop is the strip from the loop before to this one then, as far
    # as the outline's lanes begin, round that (along this one and back along the one before)
    drive_levels = list(levels)
    if islands > count and not holes_in.is_empty:
        inside = inset(shell, first + count * loop_spacing)
        before = holes_in.buffer(first + (count - 1) * loop_spacing, QUAD_SEGS) if count else holes_in
        for k in range(count, min(most, len(levels))):
            blobs = holes_in.buffer(first + k * loop_spacing, QUAD_SEGS)
            strips = [Polygon(strip.exterior) for a in polygons(inside)
                      for strip in polygons(blobs.difference(before).intersection(a)) if strip.area >= half * half]
            if not k:
                # (no loop before the first one: by the edge there's none to go back along, the island gets none)
                strips = [s for s in strips if robot_space.buffer(1e-6).covers(s.exterior)]
            drive_levels[k] = polygonal(inside.buffer(1.0, QUAD_SEGS).difference(union(strips)))
            before = blobs
    perimeter_levels = drive_levels[:most]
    took('perimeter')

    def middle(outer, inner, step: float):
        # where the area ends inside a loop level (outer, loops `step` apart) and `inner`, the next level in, doesn't
        # get there, what's left is narrower than a step: one more loop halfway in mows it (insets add up:
        # that's the level shrunk by half a step, quicker than free shrunk again)
        half_in = inset(outer, 0.5 * step)
        if not half_in.is_empty and not inner.is_empty:
            half_in = half_in.difference(inner.buffer(step, QUAD_SEGS))
        return polygonal(union(p for p in polygons(half_in) if p.area >= half * half))

    middles = [middle(outer, inner, loop_spacing) for outer, inner in zip(levels, levels[1:] + [Polygon()])] \
        if concentric else []

    # 2. what's left after the loops (kept for checks and plots, the lanes use the same edge directly)
    remaining = polygonal(subtract(free, [LineString(ring + [ring[0]]).buffer(half, QUAD_SEGS)
                                          for k, level in enumerate(perimeter_levels)
                                          for kind, ring in rings(level) if driven(k, kind)]))
    # (the lanes reach lane_overlap_passes loops further out, of the outline's and of the islands')
    lane_out, lane_isl = max(0, count - int(cfg.lane_overlap_passes)), max(0, islands - int(cfg.lane_overlap_passes))
    lane_passes = min(lane_out, lane_isl)
    lane_region = inset(free, first + lane_passes * spacing) if lane_out == lane_isl else shrunk(lane_out, lane_isl)
    # where the area is too narrow for lanes but wider than the loops reach (a strip of 0.8 to 1 m with the
    # defaults, or narrower ones where the loops stop earlier), what the loops leave in the middle gets a
    # loop of its own, halfway between the last loop there and the next one in (or where the lanes would be)
    towards = levels[:lane_passes] if lane_out == lane_isl else \
        [shrunk(min(k, lane_out), min(k, lane_isl)) for k in range(max(lane_out, lane_isl))]
    narrow = Polygon() if concentric or cfg.skip_lanes else polygonal(union(
        middle(outer, inner, spacing) for outer, inner in zip(towards, towards[1:] + [lane_region])))
    # the loops go on in where the lane region is narrower than a U-turn, the lanes only where it's wider: all the
    # loops are driven part by part like concentric's
    ringed = concentric
    if cfg.narrow_parts == 'loops' and not concentric and not cfg.skip_lanes and cfg.turn_radius > 0 and \
            len(levels) == most and not lane_region.is_empty:
        r = cfg.turn_radius
        opened = lane_region.buffer(-r, QUAD_SEGS).buffer(r, QUAD_SEGS)
        # what that takes away: narrow parts, and the tips of corners (smaller than r * r), which keep their lanes
        cut = [p for p in polygons(lane_region.difference(opened)) if p.area >= r * r]
        if cut:
            # (opened by a hair: GEOS can leave a spike without area there, the first row of lanes would go by it)
            wide = polygonal(subtract(lane_region, cut).buffer(-1e-6, 1, join_style=2).buffer(1e-6, 1, join_style=2))
            # bits left between narrow parts too small for more than a few lanes go to the loops as well
            wide = polygonal(union(p for p in polygons(wide) if p.area >= (4.0 * r) ** 2))
            keep_off = None if wide.is_empty else wide.buffer(half, QUAD_SEGS)  # the lanes mow that far round it
            while len(levels) < MAX_ROWS:
                level = level_at(len(levels))
                if keep_off is not None and not level.is_empty:
                    level = polygonal(union(p for p in polygons(level.difference(keep_off)) if p.area >= half * half))
                if level.is_empty:
                    break
                levels.append(level)
            middles = [middle(outer, union([inner, wide]) if k + 1 >= lane_passes else inner, spacing)
                       for k, (outer, inner) in enumerate(zip(levels, levels[1:] + [Polygon()]))]
            lane_region, narrow, ringed = wide, Polygon(), True
    drive_levels += levels[len(drive_levels):]
    took('remaining')
    angle = lane_angle(area, free, cfg, lane_region, remaining, robot_space)

    # recorded edges are where the centre drove, give or take body_tolerance: with the body checked the centre may go
    # that far past the first loop as well (rounding a corner, a turn, a loop out), the body keeping the margins. The
    # loops and lanes still keep to robot_space
    drive_space = robot_space
    if cfg.body and cfg.body_tolerance > 0:
        drive_space = polygonal(robot_space.buffer(cfg.body_tolerance, QUAD_SEGS))

    # turns and loops stay inside the mowing area, so do the bends of lanes around obstacles
    edges = dict(edge_distance=cfg.transit_edge_distance, edge_cost=cfg.transit_edge_cost)
    area_space = TransitPlanner(drive_space, cfg.transit_clearance, **edges)

    # 3. lanes, for crosshatch a second time across them. split_zones: the zones mowed another way than the rest, each
    # a set of lanes of its own (the rest's lanes end where a zone begins)
    passes: List[Tuple[float, List[List[Lane]]]] = []
    zones: List[Tuple[float, object]] = []
    if not concentric and not cfg.skip_lanes and not lane_region.is_empty:
        angles = [angle]
        if cfg.fill_pattern == 'crosshatch':
            second = math.fmod(angle + cfg.crosshatch_angle, math.pi)
            angles.append(second + math.pi if second < 0 else second)
        elif cfg.split_zones and not cfg.headland_turns:
            zones = zone_angles(lane_region, angle, spacing, cfg.min_lane_length, remaining, robot_space, half,
                                cfg.angle_min, cfg.angle_max)
        taken = union(zone for _, zone in zones)
        todo_lanes = [(a, lane_region, remaining) for a in angles] if not zones else \
            [(angle, polygonal(lane_region.difference(taken)),
              polygonal(remaining.difference(taken.buffer(spacing, 4))))] + \
            [(a, zone, polygonal(remaining.intersection(zone.buffer(spacing, 4)))) for a, zone in zones]
        for a, where, mow in todo_lanes:
            rows = scan_lanes(where, a, spacing, cfg.min_lane_length, mow, robot_space, half)
            if cfg.bend_max_gap > 0 and cfg.blade_offset == 0:
                rows = bend_lanes(rows, a, area_space, cfg.bend_max_gap, cfg.bend_max_offset)
            passes.append((a, split_cells(rows)))
    # (remaining counts every loop, so not when some are left out)
    if not narrow.is_empty and passes and not (cfg.skip_outline_loops or cfg.skip_obstacle_loops):
        # a narrow piece needs no loop where the lanes already mow what the loops leave around it (they reach
        # on into a strip along them, see scan_lanes). The lanes of each pass by row, so only those next to a
        # piece are looked at
        by_row = [(a, sorted((lane.y, x0, x1) for cell in cells for lane in cell for x0, x1 in lane.pieces()))
                  for a, cells in passes]

        def left(piece) -> float:
            strip = piece.buffer(half, QUAD_SEGS)
            near = []
            for a, lanes in by_row:
                c, s = math.cos(a), math.sin(a)
                xs, ys = zip(*[(x * c + y * s, -x * s + y * c) for x, y in strip.exterior.coords])
                first = bisect.bisect_left(lanes, (min(ys) - half,))
                for y, x0, x1 in lanes[first:bisect.bisect_right(lanes, (max(ys) + half, math.inf))]:
                    if x0 < max(xs) + half and x1 > min(xs) - half:
                        line = LineString(Lane(0, x0, x1, y).ends(a))
                        if line.distance(strip) < half:
                            near.append(line.buffer(half, QUAD_SEGS))
            return subtract(strip.intersection(remaining), near).area

        narrow = polygonal(union(p for p in polygons(narrow) if left(p) > 1e-4))  # more than a cm2
    took('lanes')
    loop_rings = [ring for k, geom in enumerate(drive_levels) for kind, ring in rings(geom) if driven(k, kind)] + \
        [ring for geom in middles + [narrow] for _, ring in rings(geom)]
    if probe is True:
        return _left_alone(free, loop_rings, passes, half)
    if isinstance(probe, dict):  # (what _left_alone needs: plan_coverage may ask for this spacing's later)
        probe['alone'] = (free, loop_rings, passes, half)

    # 4. route
    start_xy = _point(start)
    start_heading = float(start[2]) if start is not None and len(start) >= 3 and start[2] is not None else None
    end_xy = _point(end)
    if start_xy is None:
        corner = list(polygons(area))[0].exterior.coords[0]
        start_xy = (corner[0], corner[1])
    transit_space = robot_space if cfg.body else drive_space
    extra_space = union(clean_polygon(d) for d in drivable)
    if not extra_space.is_empty:
        # drivable areas are recorded like outlines, the mower's centre may go anywhere in them, but not
        # closer to an obstacle (or an area to keep out of) than in the mowing area
        away = union(grown)
        keep_off = away.buffer(max(centre, half), QUAD_SEGS) if not away.is_empty else None
        if keep_off is not None:
            extra_space = extra_space.difference(keep_off)
        transit_space = polygonal(union([transit_space, extra_space]))
    transit = area_space if transit_space is drive_space else TransitPlanner(transit_space, cfg.transit_clearance,
                                                                             **edges)
    route = _Route(cfg, transit, area_space, start_xy, start_heading, end_xy, footprint)
    route.loop_counts = (count, islands) if islands != count else None
    if cfg.lane_overlap_limit and passes and not lane_region.is_empty:
        # as far out as the lanes reach (_Route.in_reach)
        route.reach = lane_region.buffer(TURN_REACH, QUAD_SEGS)
        route._reach = prep(route.reach)
    route.cell_of = {id(lane): f'cell-{pi}-{ci}' for pi, (_, cells) in enumerate(passes)
                     for ci, cell in enumerate(cells) for lane in cell}
    # headland_turns: the lanes' field, where they lie but for what the loops mow and 2 turn_radius at their ends (where
    # the turns go, the lanes ending earlier for the body)
    field = polygonal(_lanes_field(passes, 2.0 * cfg.turn_radius, 0.5 * spacing).intersection(remaining)) \
        if cfg.headland_turns and passes else Polygon()
    if not field.is_empty:
        route.field, route._field, route.headland = field, prep(field.buffer(-1e-3, 1)), transit.without(field)
        route.angle = passes[0][0]
        route.lane_ys = {a: sorted({lane.y for cell in cells for lane in cell}) for a, cells in passes}
    last = cfg.perimeter_order == 'last'
    holes = [ring for kind, ring in rings(free) if kind == 'hole']

    # the parts of the area there's no way between (in the transit space) are mowed one after the other,
    # each completely: loops and lanes of one part, then the next part as a new section of the path
    parts = transit.parts()

    def part_of(p: XY) -> int:
        k = transit.part(p)
        if k is None:
            k = min(range(len(parts)), key=lambda i: parts[i].distance(Point(p)))
        return k

    def split(geom) -> Dict[int, object]:
        out: Dict[int, List[Polygon]] = {}
        for poly in polygons(geom):
            out.setdefault(part_of(poly.representative_point().coords[0]), []).append(poly)
        return {k: polygonal(union(v)) for k, v in out.items()}

    def lane_part(a: float, cell: List[Lane]) -> int:
        lane = cell[0]
        x, y = 0.5 * (lane.x0 + lane.x1), lane.y
        return part_of((x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)))

    work: Dict[int, Dict[str, Any]] = {}

    def todo(k: int) -> Dict[str, Any]:
        return work.setdefault(k, {'levels': {}, 'middles': {}, 'cells': [[] for _ in passes], 'narrow': None})

    # (mowed: no loops, no lanes, only what's left of them, see below)
    for i, level in enumerate(drive_levels if mowed is None else []):
        for k, geom in split(level).items():
            todo(k)['levels'][i] = geom
    for i, middle in enumerate(middles if mowed is None else []):
        for k, geom in split(middle).items():
            todo(k)['middles'][i] = geom
    for pi, (a, cells) in enumerate(passes if mowed is None else []):
        for cell in cells:
            if cell:
                todo(lane_part(a, cell))['cells'][pi].append(cell)
    for k, geom in split(narrow if mowed is None else Polygon()).items():
        todo(k)['narrow'] = geom

    def part_distance(k: int, p: Optional[XY]) -> float:
        return 0.0 if p is None else parts[k].distance(Point(p))

    order: List[int] = []
    pending = set(work)
    final_part = None
    if end_xy is not None and len(pending) > 1:
        final_part = min(pending, key=lambda k: part_distance(k, end_xy))
    while pending:
        candidates = [k for k in pending if k != final_part] or list(pending)
        k = min(candidates, key=lambda k: (part_distance(k, route.pos if order else start_xy), k))
        pending.remove(k)
        order.append(k)
        w = work[k]
        empty = Polygon()
        # the end point only counts in the part mowed last
        route.end = end_xy if not pending else None
        if order[1:]:
            route.new_section()

        # the end point counts for what's mowed last: the loops with last, else the lanes, else the strips too
        # narrow for lanes, else the loops
        skip = {'outline': cfg.skip_outline_loops, 'hole': cfg.skip_obstacle_loops}
        loops_here = not ringed and any(not skip[kind] and not route.left_out(kind, i)
                                        for i, g in w['levels'].items() for kind, _ in rings(g))
        loops_end = loops_here and (last or not any(w['cells']) and w['narrow'] is None)
        last_pass = max((i for i, cells in enumerate(w['cells']) if cells), default=None)

        def loops():
            if ringed:
                route.concentric([w['levels'].get(i, empty) for i in range(len(levels))],
                                 [w['middles'].get(i, empty) for i in range(len(middles))], most, last,
                                 loop_spacing, last or not any(w['cells']),
                                 not concentric or cfg.narrow_parts == 'loops')
            else:
                route.perimeter([w['levels'].get(i, empty) for i in range(len(perimeter_levels))], holes, last,
                                loops_end)

        def lanes():
            # the strips too narrow for lanes first, so the lanes can end at the end point
            if w['narrow'] is not None:
                route.extra_loops(w['narrow'], finish=not loops_end and last_pass is None)
            sets = [i for i in range(len(passes)) if not zones or i == 0]
            for i in sets:
                route.cells(w['cells'][i], passes[i][0], finish=not loops_end and i == last_pass and not zones)
            # split_zones: the rest first, then the zones, the nearest next (the one closest to the end point last)
            later = [i for i in range(len(passes)) if zones and i > 0 and w['cells'][i]]

            def near(i: int, p: Optional[XY]) -> float:
                a = passes[i][0]
                return min(route._distance(p, lane.ends(a)[k]) for cell in w['cells'][i] for lane in cell
                           for k in (0, 1))

            final = min(later, key=lambda i: (near(i, route.end), i)) if later and route.end is not None and \
                not loops_end else None
            while later:
                i = min([j for j in later if j != final] or later, key=lambda j: (near(j, route.pos), j))
                later.remove(i)
                route.cells(w['cells'][i], passes[i][0], finish=not loops_end and not later)

        if last:
            lanes()
            loops()
        else:
            loops()
            lanes()
    if not cfg.turn_on_spot:
        _smooth_spins(route, cfg)
    poses, segments = _assemble(route.segments)
    if footprint is not None and poses:
        poses, segments = _round_corners(route, poses, segments)
    took('route')
    fitted, parts = None, len({s.section for s in segments})
    if footprint is not None and cfg.body_fit and poses:
        poses, segments, fitted = _fit_body(route, poses, segments)
        # where that leaves a loop out for long at a place, the loop further in there instead, if that leaves LOOP_GAIN
        # m2 less unmowed (and no more places out)
        before = route.loops_in([p for p in fitted['places'] if p[2] >= LOOP_LEFT])
        if before:
            again = _fit_body(route, *_round_corners(route, *_assemble(route.segments)))
            if again[2]['left'] <= fitted['left'] and len(again[2]['unchecked']) <= len(fitted['unchecked']) and \
                    _unmowed(route, again[0], again[1], free).area <= \
                    _unmowed(route, poses, segments, free).area - LOOP_GAIN:
                poses, segments, fitted = again
            else:
                route.loops_back(before)
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
        took('body')
    mop = None
    if cfg.mop_up and footprint is not None and poses:
        poses, segments, mop = _mop_up(route, poses, segments, free)
        took('mop_up')
    leftover = None
    if mowed is not None:
        # what a mow left: passes over that alone, the one nearest the start (the dock) first
        if footprint is None:
            raise PlanningError('what a mow left is only planned with the body set (robot_width, robot_front, '
                                'robot_rear)')
        poses, segments, leftover = _leftovers(route, free, mowed, start_xy)
        if not poses:
            raise PlanningError(f'nothing left to mow where the mower gets to ({leftover["left_m2"]:.2f} m2 left)')
        took('leftovers')
    turning, arcs, deepest = _field_turns(route, poses, segments) if route.field is not None and mowed is None \
        else ([], 0, 0.0)
    if turning:
        route.warnings.append(f'{len(turning)} places where a drive turns in the lanes\' field, no way along the '
                              'headland fits there: at ' + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in turning))
    if not poses:
        raise PlanningError('nothing to mow: the area is too small for one loop or lane')
    if footprint is not None:
        poses, segments = _turn_at(route, poses, segments)
    if route.counts.get('lanes_shortened'):
        route.warnings.append(f'{route.counts["lanes_shortened"]} lanes along an edge shortened or left out: with '
                              f'blade_offset {cfg.blade_offset} m the centre runs that far off the blade\'s line, '
                              'more perimeter loops leave room for it')
    section_starts = [s.start for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section]
    if route.counts.get('lanes_body'):
        route.warnings.append(f'{route.counts["lanes_body"]} lanes shortened or left out where the body would stick '
                              'out at their ends')
    if parts > 1:
        route.warnings.append(f'the area has {parts} parts with no way between them: the path '
                              'jumps from one to the next')

    lengths: Dict[str, float] = {PERIMETER: 0.0, LANE: 0.0, TURN: 0.0, TRANSIT: 0.0}
    for seg in segments:
        lengths[seg.kind] += path_length(poses[seg.start:seg.end + 1])
    stats: Dict[str, Any] = {
        'mop_up': mop,
        'leftovers': leftover,
        'sections': len(section_starts),
        'path_length': sum(lengths.values()),
        'perimeter_length': lengths[PERIMETER],
        'lane_length': lengths[LANE],
        'turn_length': lengths[TURN],
        'transit_length': lengths[TRANSIT],
        'perimeter_loops': len({s.source for s in segments if s.kind == PERIMETER}),
        'loops': len(route.loops),
        'lanes': len(route.lanes),
        'cells': sum(len(cells) for _, cells in passes),
        'poses': len(poses),
        'area': free.area,
        'planning_time': time.perf_counter() - t0,
        'timings': timings,
        'chosen': {'lane_spacing': round(spacing, 4), 'perimeter_passes': count, 'mode': 'fixed'},
    }
    for key in ('turns_u_turn', 'turns_bulb', 'turns_detour', 'turns_pivot', 'turns_fallback'):
        stats[key] = route.counts.get(key, 0)
    if cfg.split_zones:
        # the zones mowed another way than the rest: their lanes' direction, how many, their outlines
        stats['zones'] = [{'angle': round(a, 6), 'lanes': sum(len(cell) for cell in cells),
                           'outlines': [[[round(x, 3), round(y, 3)] for x, y in poly.exterior.coords[:-1]]
                                        for poly in polygons(zone)]}
                          for (a, zone), (_, cells) in zip(zones, passes[1:])]
    if fitted is not None:
        stats['body_fit'] = fitted
    if cfg.headland_turns:
        stats['headland_turns'] = {'turns_in_field': len(turning), 'places': turning, 'arcs_in_field': arcs,
                                   'arc_depth': deepest, 'crossings': route.counts.get('crossings', 0)}

    return CoveragePlan(
        poses=poses, segments=segments, angle=angle, lane_spacing=spacing, perimeter_loops=route.loops,
        lanes=route.lanes, stats=stats, warnings=route.warnings, area=area, free_space=free,
        robot_space=drive_space, lane_region=lane_region, remaining=remaining,
        angles=[a for a, _ in passes] or [angle], transit_space=transit_space, body_space=body_space, config=cfg,
        section_starts=section_starts, levels=levels, narrow=narrow, cells=passes, origins=route.origins)


def _gaps(shapes: List[object], margin_of: List[float], off: List[float], cfg: PlannerConfig) -> List[Polygon]:
    """The gaps between two obstacles or areas to keep out of where the body, facing straight across, has less than
    body_tolerance to spare with each one's margin kept (their real edges robot_width / 2 inside the lines, which lie
    about body_tolerance off where the mower was): the hull round the parts of either closer than that to the other,
    less both. Not where the centre keeps off already (off: how far from each one's line). The loops go round both
    there, nothing drives in where the body would get through facing straight across at best."""
    gaps = []
    for i, a in enumerate(shapes):
        for j in range(i + 1, len(shapes)):
            b, width = shapes[j], margin_of[i] + margin_of[j] + cfg.body_tolerance
            if not a.is_empty and not b.is_empty and off[i] + off[j] <= a.distance(b) < width:
                near = union([a.intersection(b.buffer(width, QUAD_SEGS)), b.intersection(a.buffer(width, QUAD_SEGS))])
                gaps += [p for p in polygons(near.convex_hull.difference(union([a, b])))
                         if p.distance(a) < ON_EDGE and p.distance(b) < ON_EDGE]
    return gaps


def _body_space(outlines: List[Sequence[Sequence[float]]], away_from: List[Sequence[Sequence[float]]],
                margin_of: List[float], cfg: PlannerConfig) -> Tuple[object, Footprint]:
    """Where the body may be, and its Footprint there: where the edges really are, where the body was while its centre
    drove them (the outline and the drivable areas, the obstacles and areas to keep out of, each the way its points go),
    into an obstacle or an area to keep out of only along its own edge, edge_margin and each one's margin off that. The
    last ones worked out are kept (_BODY_SPACES), the last one of this thread with its Footprint (_BODY_SPACE), the plan
    gets a copy."""
    outlines, away_from = ([[(float(p[0]), float(p[1])) for p in ring] for ring in rings]
                          for rings in (outlines, away_from))
    key = (outlines, away_from, tuple(margin_of), cfg.edge_margin, cfg.body_outline)
    found = getattr(_BODY_SPACE, 'last', None)
    if found is None or found[0] != (key, cfg.spin_margin):
        shared = hashlib.sha1(repr(key).encode()).hexdigest()
        with _KEPT_LOCK:
            space = _BODY_SPACES.get(shared)
        if space is None:
            outer = union([clean_polygon(o) for o in outlines] + [_driven(outlines, cfg.body_outline)])
            band = _driven(away_from, cfg.body_outline)
            inner = [clean_polygon(w).difference(band) for w in away_from]
            space = subtract(inset(outer, cfg.edge_margin), [inset(w, -m) for w, m in zip(inner, margin_of)]).wkb
            _keep(_BODY_SPACES, shared, space)
        space = wkb.loads(space)
        found = _BODY_SPACE.last = ((key, cfg.spin_margin), space, Footprint(space, cfg.body_outline, cfg.spin_margin))
    return wkb.loads(found[1].wkb), found[2]



def plan_footprint(plan: CoveragePlan) -> Optional[Footprint]:
    cfg = plan.config
    if cfg is None or not cfg.body or plan.body_space is None:
        return None
    return Footprint(plan.body_space, cfg.body_outline, cfg.spin_margin)


def body_out(poses: List[Pose2D], segments: List[SegmentInfo], section_starts: List[int],
             footprint: Footprint) -> List[Dict[str, Any]]:
    """The poses where the body sticks out: [{x, y, yaw, kind}], kind 'spin' (turning on the spot there), 'turn',
    'lane_end', 'loop' or 'transit' (what the pose belongs to). On the turns between lanes spin_margin further in
    (footprint.OnTurn)."""
    kind_at: Dict[int, str] = {}
    on_turn = [False] * len(poses)  # (on the way to it: a turn between lanes)
    for seg in segments:
        kind = {TURN: 'turn', TRANSIT: 'transit', PERIMETER: 'loop'}.get(seg.kind, 'loop' if seg.maneuver == 'ring'
                                                                         else 'lane_end')
        for i in range(seg.start, seg.end + 1):
            kind_at[i] = kind
        on_turn[seg.start + 1:seg.end + 1] = [seg.kind == TURN] * (seg.end - seg.start)
    out = []
    for a, b in zip(section_starts, list(section_starts[1:]) + [len(poses)]):
        part = poses[a:b]
        for i in footprint.bad_poses(part, on_turn[a:b]):
            p = part[i]
            out.append({'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(p.yaw, 3),
                        'kind': 'spin' if footprint.spins(part, i) else kind_at.get(a + i, 'transit')})
    return out


def blade_points(plan: CoveragePlan, start: int, end: int) -> List[XY]:
    """Where the blade's centre is at poses start..end: blade_offset to the left of each, blade_ahead ahead."""
    off = plan.config.blade_offset if plan.config else 0.0
    ahead = plan.config.blade_ahead if plan.config else 0.0
    return [(p.x + ahead * math.cos(p.yaw) - off * math.sin(p.yaw),
             p.y + ahead * math.sin(p.yaw) + off * math.cos(p.yaw)) for p in plan.poses[start:end + 1]]


def swept_area(plan: CoveragePlan, mower_width: float, simplify: float = 0.0):
    """What the blade covers along the whole path (a disk of mower_width), for checks and plots. simplify: the
    blade's way that much simpler first (quicker, for comparing plans)."""
    ends = list(plan.section_starts[1:]) + [len(plan.poses)]
    lines = [LineString(blade_points(plan, a, b - 1)) for a, b in zip(plan.section_starts, ends) if b - a >= 2]
    if simplify > 0:
        lines = [line.simplify(simplify) for line in lines]
    if not lines:
        return Polygon()
    # (GEOS takes a lot longer for one long way crossing itself all over than for pieces of it and their union. Cut
    # in the middle of a piece at least the width long, the two ends' round caps lie in the strip of the other half:
    # the union is the same area)
    return union(LineString(piece).buffer(0.5 * mower_width, 4) for line in lines
                 for piece in _pieces(list(line.coords), SWEPT_PIECE, mower_width))


