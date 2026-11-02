"""A stand-in for plan_coverage for the tests of what takes plans (the nodes, the MQTT service): a valid plan made the
plainest way from the same input, the same every time, so those tests don't hang on how the planner plans. The
loops a lane spacing apart from the centre_offset in, straight lanes at the planner's angle one row after the other,
drives between them on the kernel's transit ways, a section for each part with no way between them. Its own warnings,
none of the planner's texts."""

import math
import random
from typing import List, Optional

from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

from mowbite_planner import CoveragePlan, NothingLeft, PlannerConfig, PlanningError, Pose2D, SegmentInfo
from mowbite_planner.config import LANE, PERIMETER, TRANSIT, TURN, Lane
from mowbite_planner.geometry import inset, polygons, rings, union
from mowbite_planner.lanes import lane_angle
from mowbite_planner.space import _check_coordinates, build_space, margins_of
from mowbite_planner.transit import TransitPlanner

KINDS = (PERIMETER, LANE, TURN, TRANSIT)
TURN_GAP = 0.5  # m: a drive between the ends of two lanes this close is a turn
LEAST = 0.01  # m2: what a mow left of less isn't a piece (leftovers)
MAX_POSES = 200000  # a plan with more is refused (a test's area that big is a mistake, not a plan to make)
STATS_KEYS = ('sections', 'path_length', 'perimeter_length', 'lane_length', 'turn_length', 'transit_length', 'poses',
              'perimeter_loops', 'loops', 'lanes', 'cells', 'area', 'edge_slots', 'planning_time', 'timings', 'chosen',
              'turns_u_turn', 'turns_bulb', 'turns_detour', 'turns_pivot', 'turns_fallback', 'mop_up', 'leftovers')


def stub_plan_coverage(boundary, obstacles=(), config: Optional[PlannerConfig] = None, start=None, end=None,
                       drivable=(), keep_out=(), margins=(), mowed=None, reach_over=(), unmowed=(),
                       arrives=None, progress=None) -> CoveragePlan:
    cfg = config or PlannerConfig()
    cfg.validate()
    if progress is not None:  # (like plan_coverage: from angles to done, here in two steps)
        progress('angles', 0, 2)
        progress('route', 1, 2)
    # the planner's checks of the input (a point 1e39 m out would be poses all the way) and its spaces
    reach_over = list(reach_over) + list(unmowed)
    margin_of = margins_of(boundary, obstacles, drivable, keep_out, reach_over, margins, cfg)
    _check_coordinates('start or end', [p for p in (start, end) if p is not None])
    space, _ = build_space(boundary, obstacles, drivable, keep_out, reach_over, margin_of, cfg, True, unmowed)
    area, free, robot = space.area, space.free, space.robot_space
    if robot.is_empty:
        raise PlanningError('stub: the area is too small')
    spacing = cfg.lane_spacing if cfg.lane_spacing_mode != 'auto' else cfg.spacing_range[1]
    transit = TransitPlanner(space.transit_space)
    angle = lane_angle(area, free, cfg)
    angles = [angle] + ([angle + cfg.crosshatch_angle] if cfg.fill_pattern == 'crosshatch' else [])
    pieces: List[tuple] = []  # (kind, [xy...], source)
    loops: List[List[tuple]] = []
    lanes: List[tuple] = []
    levels: List[list] = []  # (of every part, level k: its k-th loops' polygon)
    cells = [(a, []) for a in angles]
    origins = {}
    if mowed is not None:
        left = [p for p in polygons(free.difference(mowed)) if p.area >= LEAST]
        if not left:
            raise NothingLeft('stub: nothing left')
        for p in sorted(left, key=lambda p: p.representative_point().coords[0]):  # (lanes across each piece)
            across = _scan(p.buffer(0.5 * spacing).intersection(robot), angle, spacing) or \
                [ab for ab in [_across(robot, p.representative_point(), angle, spacing)] if ab[0] is not None]
            for a, b in across:
                lanes.append((a, b))
                origins[f'lane-{len(lanes) - 1}'] = {'parent': '', 'kind': 'leftover'}
                pieces.append((LANE, [a, b], f'lane-{len(lanes) - 1}'))
    else:
        passes = 10 ** 6 if cfg.fill_pattern == 'concentric' else max(cfg.passes, 0)
        way = cfg.perimeter_direction
        if way == 'random':  # (as the planner: the same for each count of finished mows)
            way = 'cw' if random.Random(cfg.finished_mows).random() < 0.5 else 'ccw'
        near = (lambda p: p.distance(Point(start[:2]))) if start is not None else (lambda p: p.bounds[:2])
        reach = list(polygons(space.transit_space))  # (the parts with a way between them: their loops, then lanes)
        group = {}
        for part in sorted(polygons(robot), key=near):  # (the one nearest start first)
            at = part.representative_point()
            group.setdefault(min(range(len(reach)), key=lambda i: reach[i].distance(at)), []).append(part)
        for parts in group.values():
            loop_pieces, lane_pieces = [], []
            for part in parts:
                k, level = 0, part
                while k < passes and not level.is_empty:
                    levels += [[]] * (k + 1 - len(levels))
                    levels[k] = levels[k] + [level]
                    for _, ring in rings(level):
                        loops.append(ring)
                        origins[f'loop-{len(loops) - 1}'] = {'parent': f'level-{k}', 'kind': 'outline', 'level': k}
                        pts = ring if way != 'cw' else ring[::-1]
                        loop_pieces.append((PERIMETER, pts + [pts[0]], f'loop-{len(loops) - 1}'))
                    k += 1
                    level = inset(part, k * spacing)
                if cfg.fill_pattern != 'concentric' and not cfg.skip_lanes:
                    for pi, a in enumerate(angles):  # (each part's lanes of a set a cell of their own)
                        c, si = math.cos(a), math.sin(a)
                        cells[pi][1].append([])
                        for row, (a_, b_) in enumerate(_scan(inset(part, max(k - 0.5, 0.0) * spacing), a, spacing)):
                            lanes.append((a_, b_))
                            cells[pi][1][-1].append(Lane(row, a_[0] * c + a_[1] * si, b_[0] * c + b_[1] * si,
                                                         -a_[0] * si + a_[1] * c))
                            origins[f'lane-{len(lanes) - 1}'] = {'parent': f'cell-{pi}-{len(cells[pi][1]) - 1}',
                                                                 'kind': 'straight', 'row': row}
                            lane_pieces.append((LANE, [a_, b_], f'lane-{len(lanes) - 1}'))
            if cfg.perimeter_order == 'last' and end is not None:  # (the loop nearest end last)
                loop_pieces.sort(key=lambda lp: -LineString(lp[1]).distance(Point(end[:2])))
            pieces += lane_pieces + loop_pieces if cfg.perimeter_order == 'last' else loop_pieces + lane_pieces
    poses, segments, sections, here = [], [], 0, start
    for kind, pts, source in pieces:
        anchor = end if kind == PERIMETER and cfg.perimeter_order == 'last' and end is not None else here
        if kind == PERIMETER and anchor is not None:  # (a loop starts where it's nearest, with the order last ends)
            pts = _from_nearest(pts, anchor[:2])
        if poses:
            way = transit.path(poses[-1][:2], pts[0])
            if way is None:
                sections += 1
            else:
                kind_of = TURN if kind == LANE and math.dist(way[0], way[-1]) < TURN_GAP else TRANSIT
                # (the blade on along a drive on the lawn, off where it leaves it)
                on = kind_of == TRANSIT and free.buffer(1e-6).covers(LineString(way))
                _add(poses, segments, kind_of, way, cfg, sections, source, on)
        _add(poses, segments, kind, pts, cfg, sections, source)
        here = poses[-1][:2]
    if not poses:
        raise PlanningError('stub: nothing to plan')
    if end is not None and segments[-1].kind != PERIMETER:  # (and on to end, as near as it gets)
        at = nearest_points(space.transit_space, Point(end[:2]))[0].coords[0]
        way = transit.path(poses[-1][:2], at) if math.dist(poses[-1][:2], at) > 1e-3 else None
        if way is not None:
            _add(poses, segments, TRANSIT, way, cfg, sections, '')
    if mowed is not None:  # (a plan for what a mow left: every segment the planner's leftover)
        for s in segments:
            s.maneuver = 'leftover'
    starts = [0] + [s.start for a, s in zip(segments, segments[1:]) if s.section != a.section]
    out = [Pose2D(x, y, h) for x, y, h in poses]
    length = {k: 0.0 for k in KINDS}
    for s in segments:
        length[s.kind] += sum(math.dist(out[i][:2], out[i + 1][:2]) for i in range(s.start, s.end))
    turns = sum(1 for s in segments if s.kind == TURN)
    stats = {'sections': len(starts), 'path_length': sum(length.values()), 'perimeter_length': length[PERIMETER],
             'lane_length': length[LANE], 'turn_length': length[TURN], 'transit_length': length[TRANSIT],
             'poses': len(out), 'perimeter_loops': len(loops), 'loops': len(loops), 'lanes': len(lanes),
             'cells': sum(len(c) for _, c in cells),
             'area': free.area, 'edge_slots': 0, 'planning_time': 0.0, 'timings': {},
             'chosen': {'lane_spacing': round(spacing, 3), 'perimeter_passes': cfg.passes,
                        'mode': cfg.lane_spacing_mode},
             'turns_u_turn': turns, 'turns_bulb': 0, 'turns_detour': 0, 'turns_pivot': 0, 'turns_fallback': 0,
             'mop_up': None, 'leftovers': None}
    if mowed is not None:
        left_space = free.difference(mowed)
        swept = union([LineString(ab).buffer(0.5 * cfg.mower_width) for ab in lanes]).intersection(left_space)
        stats['leftovers'] = {'left_m2': round(left_space.area, 3), 'pieces': len(left), 'passes': len(lanes),
                              'area': round(swept.area, 3)}
    warnings = [w for w in [cfg.no_body] if w] + (['stub: %d sections' % len(starts)] if len(starts) > 1 else [])
    if progress is not None:
        progress('done', 2, 2)
    return CoveragePlan(poses=out, segments=segments, angle=angle, lane_spacing=spacing, perimeter_loops=loops,
                        lanes=lanes, stats=stats, warnings=warnings, area=area, free_space=free,
                        robot_space=space.drive_space, lane_region=robot, remaining=robot, angles=angles,
                        transit_space=space.transit_space, body_space=space.body_space, config=cfg,
                        section_starts=starts, unmowed=space.unmowed,
                        levels=[union(ls) for ls in levels], origins=origins, cells=cells)


class Spy:
    """stub_plan_coverage that keeps what it was asked: calls, each (args, kwargs) as plan_coverage got them."""

    def __init__(self):
        self.calls = []

    def __call__(self, *args, **kw):
        self.calls.append((args, kw))
        return stub_plan_coverage(*args, **kw)

    def config(self, i: int = -1) -> PlannerConfig:
        args, kw = self.calls[i]
        return kw['config'] if 'config' in kw else args[2] if len(args) > 2 else PlannerConfig()


def _add(poses, segments, kind, pts, cfg, section, source, blade=False):
    """pts as poses at most waypoint_spacing apart, each facing the next, the segment from the last pose on."""
    xy = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, math.ceil(math.dist(a, b) / cfg.waypoint_spacing)) if cfg.waypoint_spacing > 0 else 1
        if n > MAX_POSES:
            raise PlanningError('stub: more than %d poses' % MAX_POSES)
        xy += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    xy.append(tuple(pts[-1]))
    xy = [p for i, p in enumerate(xy) if i == 0 or math.dist(p, xy[i - 1]) > 1e-6]
    if len(poses) + len(xy) > MAX_POSES:
        raise PlanningError('stub: more than %d poses' % MAX_POSES)
    on = bool(segments) and segments[-1].section == section  # (else a section of its own starts here)
    if on and math.dist(xy[0], poses[-1][:2]) <= 1e-6:
        xy = xy[1:]
    if not xy:
        return
    first = len(poses) - 1 if on else len(poses)
    for p in xy:
        if poses and (on or len(poses) > first):
            x, y, _ = poses[-1]
            poses[-1] = (x, y, math.atan2(p[1] - y, p[0] - x))
        poses.append((p[0], p[1], poses[-1][2] if len(poses) > first else 0.0))
    segments.append(SegmentInfo(kind, first, len(poses) - 1, False, 'u_turn' if kind == TURN else '', section,
                                source, blade))


def _from_nearest(pts, xy):
    """The closed loop pts started at its point nearest xy."""
    line = LineString(pts)
    d = line.project(Point(xy))
    at = line.interpolate(d).coords[0]
    run, i = 0.0, 0
    while i + 1 < len(pts) - 1 and run + math.dist(pts[i], pts[i + 1]) <= d:
        run += math.dist(pts[i], pts[i + 1])
        i += 1
    ring = [at] + pts[i + 1:-1] + pts[:i + 1] + [at]
    return [p for k, p in enumerate(ring) if k == 0 or math.dist(p, ring[k - 1]) > 1e-6]


def _scan(region, angle, spacing):
    """Straight lanes across region spacing apart, every other row the other way."""
    out, c, s = [], math.cos(angle), math.sin(angle)
    if region.is_empty:
        return out
    pts = [p for poly in polygons(region) for p in poly.exterior.coords]
    ys = [-x * s + y * c for x, y in pts]
    xs = [x * c + y * s for x, y in pts]
    n = max(1, math.ceil((max(ys) - min(ys)) / spacing))
    step = (max(ys) - min(ys)) / n  # (rows no further apart than spacing, the first and last half a row in)
    for row in range(n):
        v = min(ys) + (row + 0.5) * step
        line = LineString([((xs_ * c - v * s), (xs_ * s + v * c)) for xs_ in (min(xs) - 1, max(xs) + 1)])
        cut = [g for g in getattr(region.intersection(line), 'geoms', [region.intersection(line)])
               if isinstance(g, LineString) and g.length > 1e-3]
        cut.sort(key=lambda g: g.coords[0][0] * c + g.coords[0][1] * s)
        for g in cut[::-1] if row % 2 else cut:
            a, b = g.coords[0], g.coords[-1]
            out.append((b, a) if row % 2 else (a, b))
    return out


def _across(space, point: Point, angle, spacing):
    """A short lane through point (or the nearest of space) along angle, inside space."""
    if not space.covers(point):
        point = nearest_points(space, point)[0]
    c, s = math.cos(angle), math.sin(angle)
    line = LineString([(point.x - spacing * c, point.y - spacing * s), (point.x + spacing * c, point.y + spacing * s)])
    cut = space.intersection(line)
    if cut.is_empty or cut.length < 1e-3:
        return None, None
    g = cut if isinstance(cut, LineString) else max(cut.geoms, key=lambda g: g.length)
    return tuple(g.coords[0]), tuple(g.coords[-1])
