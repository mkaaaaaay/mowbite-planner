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
   for controllers that can't back up (OpenMower's FTC runs with forward_only).
"""

import bisect
import dataclasses
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon

from .geometry import QUAD_SEGS, clean_polygon, inset, polygonal, polygons, rings, subtract, union
from .maneuvers import (CORNER_ANGLE, closest_on_polyline, connect, drive_polyline, path_length, plan_turn,
                        same_position, sample_straight)
from .mathutil import Pose2D, angle_diff, normalize_angle
from .transit import TransitPlanner

XY = Tuple[float, float]

PERIMETER = 'perimeter'
LANE = 'lane'
TURN = 'turn'
TRANSIT = 'transit'

ANGLE_STRATEGIES = ('longest_edge', 'min_width', 'optimal')
LANE_ORDERS = ('snake', 'skip')
# lanes: parallel lanes; crosshatch: lanes, then lanes across them; concentric: loops further and further in
FILL_PATTERNS = ('lanes', 'crosshatch', 'concentric')
PERIMETER_ORDERS = ('first', 'last')
PERIMETER_DIRECTIONS = ('auto', 'ccw', 'cw')
TURN_TYPES = ('u_turn', 'bulb', 'k_turn', 'detour', 'pivot')
# recorded outlines have a point every few cm, for finding their longest edge they're simplified this much
ANGLE_SIMPLIFY_TOLERANCE = 0.10
# rounding a loop's corner may cut it by this share of the mower width at most, 90 degree corners keep the
# full perimeter_corner_radius, sharper ones get less
CORNER_CUT = 0.3
# lanes this close to the edge of their region count as on it, keeps a lane along a straight edge
EDGE_MARGIN = 1e-4
MAX_ROWS = 20000
# a plan this big is a mistake (a lane spacing in mm, a waypoint every mm), not a lawn: about 200 km of path
MAX_POSES = 2000000
# rad, between the angles 'optimal' tries at least
MIN_ANGLE_STEP = math.radians(0.5)
# loops along the edge (OpenMower's outline_count is a uint8), they stop anyway where nothing is left
MAX_PASSES = 255
# m, wider turns are a mistake, and planning takes longer the wider they are
MAX_TURN_RADIUS = 10.0
# m, coordinates further out are a mistake (or the wrong frame), not a lawn
MAX_COORDINATE = 1e7


class PlanningError(ValueError):
    """The request can't be planned (bad parameters, no area left to mow)."""


@dataclass
class PlannerConfig:
    mower_width: float = 0.22  # m, cutting width
    overlap: float = 0.15  # 0..<1, share of the cutting width neighbouring lanes overlap
    perimeter_passes: int = 2  # loops along the outline and around every obstacle
    fill_pattern: str = 'lanes'  # 'lanes', 'crosshatch' or 'concentric'
    crosshatch_angle: float = math.pi / 2  # rad, direction of crosshatch's second lanes to the first
    perimeter_order: str = 'first'  # loops 'first' or 'last' (smooths the marks the turns leave)
    perimeter_direction: str = 'auto'  # 'ccw', 'cw' around every loop, 'auto': the way the mower faces
    angle: Optional[float] = None  # rad, lane direction. None: worked out by angle_strategy
    # 'longest_edge', 'min_width' (narrowest width) or 'optimal' (tries angle_step apart, fewest lanes and
    # groups of lanes)
    angle_strategy: str = 'longest_edge'
    angle_offset: float = 0.0  # rad, added to the lane direction (also to a fixed angle)
    # rad, the final lane direction stays within, bouncing back at the ends like MowBite's angle range
    angle_min: Optional[float] = None
    angle_max: Optional[float] = None
    angle_step: float = math.radians(5.0)  # rad, between the angles 'optimal' tries
    # m, the first perimeter loop runs this far inside the outline. None: mower_width / 2, the blade's edge
    # on the outline. OpenMower records outlines driving the mower's centre along the edge and uses 0,
    # negative goes outside the outline (OpenMower's outline_offset allows that too)
    perimeter_offset: Optional[float] = None
    # m, radius of the turns between lanes. Tighter ones (down to half the lane spacing, then turning on
    # the spot) only where the headland is too small
    turn_radius: float = 0.25
    allow_reverse: bool = True  # K-turns, which back up a little
    # the kinds of turns allowed, preferred first ('u_turn', 'bulb', 'k_turn', 'detour', 'pivot'). Larger
    # radii come first, the order counts between turns of the same radius
    turn_types: Tuple[str, ...] = TURN_TYPES
    lane_order: str = 'snake'  # 'snake': neighbour after neighbour, 'skip': lanes far enough apart for U-turns
    perimeter_corner_radius: float = 0.15  # m, outline corners of the loops are rounded this much, 0 = sharp
    waypoint_spacing: float = 0.10  # m, distance between poses, 0 = only where the direction changes
    transit_clearance: float = 0.05  # m, transits keep this far from the outer loop and obstacle loops
    simplify_tolerance: float = 0.01  # m, the outline and obstacles are simplified this much first
    min_lane_length: float = 0.10  # m, shorter pieces of a lane are left out
    # lanes reach this many perimeter loops further out (OpenMower's outline_overlap_count)
    lane_overlap_passes: int = 0
    # leave parts out (OpenMower's skip_area_outline, skip_obstacle_outlines, skip_fill), the rest is
    # planned as if they were there
    skip_outline_loops: bool = False
    skip_obstacle_loops: bool = False
    skip_lanes: bool = False

    @property
    def lane_spacing(self) -> float:
        return self.mower_width * (1.0 - self.overlap)

    @property
    def first_loop_offset(self) -> float:
        return 0.5 * self.mower_width if self.perimeter_offset is None else self.perimeter_offset

    @property
    def allowed_turns(self) -> Tuple[str, ...]:
        return tuple(t for t in self.turn_types if t != 'k_turn' or self.allow_reverse)

    @classmethod
    def names(cls) -> Tuple[str, ...]:
        return tuple(f.name for f in dataclasses.fields(cls))

    def with_overrides(self, overrides: Mapping[str, Any]) -> 'PlannerConfig':
        """A copy with some settings changed, e.g. per area. Unknown names are an error, ints are fine for
        floats, None keeps the setting."""
        unknown = sorted(set(overrides) - set(self.names()))
        if unknown:
            raise PlanningError(f'unknown planner settings: {", ".join(unknown)}')
        values = {}
        for name, value in overrides.items():
            if value is None and name not in _OPTIONAL:
                continue
            if name == 'turn_types':
                if isinstance(value, str):
                    value = [t.strip() for t in value.split(',') if t.strip()]
                if not isinstance(value, (list, tuple)):
                    raise PlanningError(f'turn_types must be a list of turn types, got {value!r}')
                value = tuple(value)
            elif name in _FLOATS and isinstance(value, int) and not isinstance(value, bool):
                try:
                    value = float(value)
                except OverflowError:
                    raise PlanningError(f'{name} must be a finite number, got {value}') from None
            values[name] = value
        return dataclasses.replace(self, **values)

    def validate(self) -> None:
        def finite(name, lo=None, hi=None, lo_open=False, hi_open=False):
            v = getattr(self, name)
            try:
                # true is no number of metres
                ok = isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            except OverflowError:  # an int too big for a float
                ok = False
            if not ok:
                raise PlanningError(f'{name} must be a finite number, got {v!r}')
            if lo is not None and (v <= lo if lo_open else v < lo):
                raise PlanningError(f'{name} must be {">" if lo_open else ">="} {lo}, got {v}')
            if hi is not None and (v >= hi if hi_open else v > hi):
                raise PlanningError(f'{name} must be {"<" if hi_open else "<="} {hi}, got {v}')

        finite('mower_width', 0.0, lo_open=True)
        finite('overlap', 0.0, 1.0, hi_open=True)
        finite('turn_radius', 0.0, MAX_TURN_RADIUS)
        finite('perimeter_corner_radius', 0.0)
        finite('waypoint_spacing', 0.0)
        finite('transit_clearance', 0.0)
        finite('simplify_tolerance', 0.0)
        finite('min_lane_length', 0.0)
        finite('angle_offset')
        finite('crosshatch_angle')
        # finer steps only take longer: half a degree turns a 10 m lane by 9 cm at its end
        finite('angle_step', MIN_ANGLE_STEP)
        for name in ('angle', 'angle_min', 'angle_max'):
            if getattr(self, name) is not None:
                finite(name)
        if (self.angle_min is None) != (self.angle_max is None):
            raise PlanningError('angle_min and angle_max go together')
        if self.perimeter_offset is not None:
            finite('perimeter_offset')
        for name in ('perimeter_passes', 'lane_overlap_passes'):
            v = getattr(self, name)
            whole = isinstance(v, int) or isinstance(v, float) and math.isfinite(v) and v == int(v)
            if isinstance(v, bool) or not whole or not 0 <= v <= MAX_PASSES:
                raise PlanningError(f'{name} must be a whole number 0..{MAX_PASSES}, got {v!r}')
        if self.lane_order not in LANE_ORDERS:
            raise PlanningError(f'lane_order must be one of {", ".join(LANE_ORDERS)}, got {self.lane_order!r}')
        for name, allowed in (('angle_strategy', ANGLE_STRATEGIES), ('fill_pattern', FILL_PATTERNS),
                              ('perimeter_order', PERIMETER_ORDERS), ('perimeter_direction', PERIMETER_DIRECTIONS)):
            if getattr(self, name) not in allowed:
                raise PlanningError(f'{name} must be one of {", ".join(allowed)}, got {getattr(self, name)!r}')
        if not isinstance(self.turn_types, (tuple, list)) or not self.turn_types or \
                any(t not in TURN_TYPES for t in self.turn_types):
            raise PlanningError(f'turn_types must be some of {", ".join(TURN_TYPES)}, got {self.turn_types!r}')
        for name in ('allow_reverse', 'skip_outline_loops', 'skip_obstacle_loops', 'skip_lanes'):
            if not isinstance(getattr(self, name), bool):
                raise PlanningError(f'{name} must be true or false, got {getattr(self, name)!r}')


# settings where None means something (worked out, mower_width / 2), and the ones that take a float
_OPTIONAL = ('angle', 'angle_min', 'angle_max', 'perimeter_offset')
_FLOATS = tuple(f.name for f in dataclasses.fields(PlannerConfig) if isinstance(f.default, float)) + _OPTIONAL


@dataclass
class PathSegment:
    kind: str  # PERIMETER, LANE, TURN or TRANSIT
    poses: List[Pose2D]
    reverse: bool = False  # driven backwards (the middle leg of a K-turn)
    # turns: 'u_turn', 'bulb', 'k_turn', 'detour', 'pivot'; transits replacing a turn: 'turn_fallback'
    maneuver: str = ''
    section: int = 0


@dataclass
class SegmentInfo:
    kind: str
    start: int  # index of the first pose in CoveragePlan.poses
    end: int  # index of the last pose, the next segment starts here or right after
    reverse: bool
    maneuver: str
    # parts of the area with no way between them (e.g. a hedge across it) are sections of their own: the
    # path jumps from the end of one to the start of the next, getting there is up to whoever drives it
    section: int = 0


@dataclass
class CoveragePlan:
    poses: List[Pose2D]
    segments: List[SegmentInfo]
    angle: float  # rad, lane direction, 0..pi
    lane_spacing: float
    perimeter_loops: List[List[XY]]
    lanes: List[Tuple[XY, XY]]  # in the order they're mowed, each from where it starts
    stats: Dict[str, Any]
    warnings: List[str]
    # the geometry it was planned on, for checks and plots
    area: object = None  # outline as given (cleaned), obstacles not taken out
    free_space: object = None  # area minus obstacles: what gets mowed
    robot_space: object = None  # where the mower's centre may go: free_space shrunk by perimeter_offset
    lane_region: object = None  # where the lane centres lie
    remaining: object = None  # free_space minus the strips the perimeter loops mow
    angles: List[float] = dataclasses.field(default_factory=list)  # of every set of lanes (crosshatch: 2)
    transit_space: object = None  # robot_space and the drivable areas
    config: Optional[PlannerConfig] = None  # what it was planned with
    section_starts: List[int] = dataclasses.field(default_factory=lambda: [0])  # first pose of every section


# lane angle ------------------------------------------------------------------------------------------------


def _largest(geom) -> Optional[Polygon]:
    parts = list(polygons(geom))
    return max(parts, key=lambda p: p.area) if parts else None


def _longest_edge_direction(coords: Sequence[XY]) -> float:
    best, angle = -1.0, 0.0
    for (x0, y0), (x1, y1) in zip(coords, coords[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        if length > best:
            best, angle = length, math.atan2(y1 - y0, x1 - x0)
    return angle


def longest_edge_angle(area) -> float:
    """Direction of the longest edge of the (largest part of the) outline. Lanes along it give long lanes
    and few turns for the usual garden shapes."""
    main = _largest(area)
    if main is None:
        return 0.0
    ring = main.exterior
    simple = ring.simplify(ANGLE_SIMPLIFY_TOLERANCE, preserve_topology=False)
    coords = list(simple.coords) if len(simple.coords) >= 4 else list(ring.coords)
    return _longest_edge_direction(coords)


def min_width_angle(area) -> float:
    """Direction of the long side of the smallest rectangle around the area: the fewest lanes for convex
    shapes."""
    rect = area.minimum_rotated_rectangle
    if rect.is_empty or not hasattr(rect, 'exterior'):
        return longest_edge_angle(area)
    return _longest_edge_direction(list(rect.exterior.coords))


def angle_in_range(angle: float, lo: Optional[float] = None, hi: Optional[float] = None) -> float:
    """Like MowBite's angleInRange (src/lib/mowStripes.ts): it's about the direction of the lanes, 0 and
    180 degrees give the same ones, so an angle whose lanes already lie in the range stays. Past an end it
    bounces back. hi < lo is a range across the ends (e.g. 170 to -170 degrees around 180)."""
    if lo is None or hi is None:
        return angle
    half = math.pi
    d = hi - lo
    # half a turn or more allows every direction
    if d >= half:
        return angle
    width = math.fmod(math.fmod(d, half) + half, half)
    if width == 0:
        return lo
    # the same lanes can be written +-180 degrees apart, use the one closest to the middle of the range
    mid = lo + width / 2
    r = math.fmod(angle - mid, half)
    angle = mid + (r - half if r > half / 2 else r + half if r < -half / 2 else r)
    t = math.fmod(angle - lo, 2 * width)
    if t < 0:
        t += 2 * width
    return lo + (t if t <= width else 2 * width - t)


def _lane_cost(region, angle: float, spacing: float, min_length: float, target=None, room=None,
               reach: float = 0.0) -> float:
    # every lane costs a turn and every group of lanes a drive to it, a few turns' worth
    rows = scan_lanes(region, angle, spacing, min_length, target, room, reach)
    return sum(len(r) for r in rows) + 3.0 * len(split_cells(rows))


def optimal_angle(region, cfg: PlannerConfig, fallback: float, target=None, room=None) -> float:
    """The lane direction with the fewest lanes and groups of lanes, tried angle_step apart (within the
    angle range if there is one). Ties go to the one closest to the fallback (the longest edge). target and
    room as for scan_lanes, so the lanes counted are the ones the plan gets."""
    if region.is_empty:
        return fallback
    candidates = {round(math.fmod(fallback, math.pi) % math.pi, 9)}
    n = max(1, int(round(math.pi / cfg.angle_step)))
    candidates.update(round(i * math.pi / n, 9) for i in range(n))
    if cfg.angle_min is not None:
        candidates = {angle_in_range(a, cfg.angle_min, cfg.angle_max) for a in candidates}
    # a recorded outline has a point every few cm: its rings are read once, not for every angle. Simplified
    # they would count other lanes than the plan gets
    region, target, room = (None if g is None else _ring_list(g) for g in (region, target, room))
    best = None
    for a in sorted(candidates):
        cost = _lane_cost(region, a, cfg.lane_spacing, cfg.min_lane_length, target, room, 0.5 * cfg.mower_width)
        key = (cost, angle_diff(2 * a, 2 * fallback))
        if best is None or key < best[0]:
            best = (key, a)
    return best[1]


def lane_angle(area, free, cfg: PlannerConfig, lane_region=None, target=None, room=None) -> float:
    if cfg.angle is not None:
        a = cfg.angle
    elif cfg.angle_strategy == 'min_width':
        a = min_width_angle(free)
    elif cfg.angle_strategy == 'optimal' and lane_region is not None:
        a = optimal_angle(lane_region, cfg, longest_edge_angle(area), target, room)
    else:
        a = longest_edge_angle(area)
    a = angle_in_range(a + cfg.angle_offset, cfg.angle_min, cfg.angle_max)
    # lanes have no front, 30 and 210 degrees are the same lanes
    a = math.fmod(a, math.pi)
    return a + math.pi if a < 0 else a


# lanes and cells -------------------------------------------------------------------------------------------


@dataclass
class Lane:
    """A piece of a lane in the lane frame: x along the lanes, y across, row = index of the line."""

    row: int
    x0: float
    x1: float
    y: float

    def ends(self, angle: float) -> Tuple[XY, XY]:
        """Where it starts and ends in the map, for lanes at angle."""
        c, s = math.cos(angle), math.sin(angle)
        return ((self.x0 * c - self.y * s, self.x0 * s + self.y * c),
                (self.x1 * c - self.y * s, self.x1 * s + self.y * c))


def _ring_list(geom) -> List[List[XY]]:
    """The rings of a geometry as point lists (a list of them stays as it is)."""
    return geom if isinstance(geom, list) else [ring for _, ring in rings(geom)]


class _Scanner:
    """Where horizontal lines in the lane frame cross a polygon's rings (even-odd)."""

    def __init__(self, ring_list: List[List[XY]], c: float, s: float):
        # every edge that isn't horizontal: its lowest and highest y, its first point and the step to the next
        self.edges = []
        self.lo, self.hi = math.inf, -math.inf
        for ring in ring_list:
            local = [(x * c + y * s, -x * s + y * c) for x, y in ring]
            if not local:
                continue
            for (ax, ay), (bx, by) in zip(local, local[1:] + local[:1]):
                if ay < by:
                    self.edges.append((ay, by, ax, ay, bx - ax, by - ay))
                elif by < ay:
                    self.edges.append((by, ay, ax, ay, bx - ax, by - ay))
            ys = [y for _, y in local]
            self.lo, self.hi = min(self.lo, min(ys)), max(self.hi, max(ys))

    def intervals(self, y0: float, step: float, count: int) -> List[List[Tuple[float, float]]]:
        """Inside intervals on the lines y0 + j * step, j = 0..count-1. The crossings are found per edge for
        the lines it spans, not per line for every edge."""
        crossings: List[List[float]] = [[] for _ in range(count)]
        for lo, hi, ax, ay, dx, dy in self.edges:
            j = int((lo - y0) // step)
            if j < 0:
                j = 0
            y = y0 + j * step
            # half open, so a line through a vertex crosses exactly one of its two edges
            while y < hi and j < count:
                if y >= lo:
                    crossings[j].append(ax + (y - ay) * dx / dy)
                j += 1
                y = y0 + j * step
        out = []
        for xs in crossings:
            xs.sort()
            out.append([(xs[k], xs[k + 1]) for k in range(0, len(xs) - 1, 2) if xs[k + 1] > xs[k]])
        return out


Intervals = List[Tuple[float, float]]


def _intersect(a: Intervals, b: Intervals) -> Intervals:
    out, i, j = [], 0, 0
    while i < len(a) and j < len(b):
        lo, hi = max(a[i][0], b[j][0]), min(a[i][1], b[j][1])
        if hi > lo:
            out.append((lo, hi))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def _subtract(a: Intervals, b: Intervals) -> Intervals:
    out = []
    for lo, hi in a:
        cur = lo
        for blo, bhi in b:
            if bhi <= cur:
                continue
            if blo >= hi:
                break
            if blo > cur:
                out.append((cur, blo))
            cur = max(cur, bhi)
        if cur < hi:
            out.append((cur, hi))
    return out


def _union(a: Intervals, b: Intervals) -> Intervals:
    out: Intervals = []
    for lo, hi in sorted(a + b):
        if out and lo <= out[-1][1] + 1e-9:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    return out


def _widen(a: Intervals, d: float) -> Intervals:
    return _union([(lo - d, hi + d) for lo, hi in a], [])


# slivers of the area to mow narrower than this next to a lane's strip aren't worth stretching a lane for
COVER_TOLERANCE = 0.01


def scan_lanes(region, angle: float, spacing: float, min_length: float, target=None, room=None,
               reach: float = 0.0) -> List[List[Lane]]:
    """The pieces of parallel lines (at `angle`, at most `spacing` apart) inside the region, row by row.

    The first and the last line lie on the region's edge, the ones between are spread evenly, so the
    spacing is never more than asked for.

    With target (what the lanes have to mow), room (where the mower may drive) and reach (half the
    cutting width): where a part of the region ends between two rows, e.g. under an edge running along
    the lanes, the last row's strip can stop short of the perimeter loops' strip. Where target reaches past
    a lane's strip and the row on that side has no lane, that row's lane is carried on through the
    headland (as far as room allows), so nothing is left between the lanes and the loops.

    region, target and room are geometries or lists of their rings (point lists).
    """
    c, s = math.cos(angle), math.sin(angle)
    region, target, room = (None if g is None else _ring_list(g) for g in (region, target, room))
    lanes = _Scanner(region, c, s)
    if not lanes.edges:
        return []
    usable = lanes.hi - lanes.lo - 2.0 * EDGE_MARGIN
    if usable <= 0.0:
        count, step, y0 = 1, spacing, 0.5 * (lanes.lo + lanes.hi)
    else:
        count = int(math.ceil(usable / spacing - 1e-9)) + 1
        if count > MAX_ROWS:
            raise PlanningError(f'{count} lanes, more than {MAX_ROWS}: the area is too big for this lane spacing')
        step = usable / (count - 1)
        y0 = lanes.lo + EDGE_MARGIN
    # one more row on either side, which only gets lanes stretched into it
    n, base = count + 2, y0 - step
    # pieces too short to be lanes don't mow what's next to them either
    rows = [[]] + [[(lo, hi) for lo, hi in r if hi - lo >= min_length] for r in lanes.intervals(y0, step, count)] \
        + [[]]

    free = None
    if target and room is not None:
        tgt = _Scanner(target, c, s)
        beyond = reach + COVER_TOLERANCE
        on = tgt.intervals(base, step, n)  # to mow on row j's line
        above = tgt.intervals(base + beyond, step, n)  # to mow above row j's strip
        below = tgt.intervals(base - beyond, step, n)  # to mow below row j's strip
        free = _Scanner(room, c, s).intervals(base, step, n)
        for _ in range(3):
            grown = [list(r) for r in rows]
            more = 0.0
            for j in range(n - 1):
                # above row j where row j + 1 has no lane: row j + 1 goes on there, and the other way round
                for src, dst, todo in ((j, j + 1, above[j]), (j + 1, j, below[j + 1])):
                    extra = _intersect(_subtract(_intersect(todo, rows[src]), rows[dst]), free[dst])
                    if extra:
                        more += sum(hi - lo for lo, hi in extra)
                        grown[dst] = _union(grown[dst], extra)
            for j in range(n):
                # on the row's own line, out of reach of its lanes' ends (e.g. under an obstacle that starts
                # level with the row): a lane there, joined to the lanes next to it
                todo = _subtract(_intersect(on[j], free[j]), _widen(rows[j], reach))
                todo = [(lo, hi) for lo, hi in todo if hi - lo > COVER_TOLERANCE]
                if todo:
                    extra = _intersect(_widen(todo, reach), free[j])
                    more += sum(hi - lo for lo, hi in todo)
                    grown[j] = _union(grown[j], extra)
            rows = grown
            if more < 1e-6:
                break

    out = []
    for j, intervals in enumerate(rows):
        if free is not None and any(hi - lo < min_length for lo, hi in intervals):
            # a filled in piece too short for a lane is lengthened to one, as far as there's room
            longer = []
            for lo, hi in intervals:
                pad = 0.5 * max(0.0, min_length - (hi - lo))
                longer.extend(_intersect([(lo - pad, hi + pad)], free[j]) if pad > 0.0 else [(lo, hi)])
            intervals = _union(longer, [])
        y = base + j * step
        out.append([Lane(j, lo, hi, y) for lo, hi in intervals if hi - lo >= min_length])
    # the outer rows only if something was stretched into them
    if not out[0]:
        out = out[1:]
    if out and not out[-1]:
        out = out[:-1]
    return out


def split_cells(rows: List[List[Lane]]) -> List[List[Lane]]:
    """Groups the lanes into cells that can be mowed in one zigzag (boustrophedon decomposition).

    A lane continues the cell of the lane in the row before when they overlap and neither of them
    overlaps anything else in the other row. Where a row splits (an obstacle starts) or two pieces
    join (it ends), new cells begin.
    """
    cells: List[List[Lane]] = []
    open_cells: Dict[int, List[Lane]] = {}
    prev: List[Lane] = []
    for lanes in rows:
        succ: Dict[int, List[int]] = {i: [] for i in range(len(prev))}
        pred: Dict[int, List[int]] = {k: [] for k in range(len(lanes))}
        for i, p in enumerate(prev):
            for k, q in enumerate(lanes):
                if p.x0 < q.x1 and q.x0 < p.x1:
                    succ[i].append(k)
                    pred[k].append(i)
        now_open: Dict[int, List[Lane]] = {}
        for k, q in enumerate(lanes):
            ps = pred[k]
            if len(ps) == 1 and len(succ[ps[0]]) == 1 and ps[0] in open_cells:
                cell = open_cells[ps[0]]
                cell.append(q)
            else:
                cell = [q]
                cells.append(cell)
            now_open[k] = cell
        open_cells = now_open
        prev = lanes
    return cells


# route -----------------------------------------------------------------------------------------------------


class _Route:
    def __init__(self, cfg: PlannerConfig, transit: TransitPlanner, area: TransitPlanner, start: Optional[XY],
                 heading: Optional[float], end: Optional[XY]):
        self.cfg = cfg
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

    def _count(self, key: str) -> None:
        self.counts[key] = self.counts.get(key, 0) + 1

    def add(self, kind: str, poses: List[Pose2D], reverse: bool = False, maneuver: str = '') -> None:
        if not poses:
            return
        self.segments.append(PathSegment(kind, poses, reverse, maneuver, self.section))
        last = poses[-1]
        self.pos = (last.x, last.y)
        self.heading = last.yaw
        self.started = True

    def new_section(self) -> None:
        """What comes next is a section of its own: the path jumps there."""
        if self.started:
            self.section += 1
            self.started = False

    def go_to(self, target: XY, heading: Optional[float] = None) -> None:
        """Drives to target, to arrive facing heading if given. Where there's no way there (can't happen
        within a part of the area, but never leave anything out) the path goes on there as a new section."""
        if not self.started:
            self.pos = target
            return
        dist = math.hypot(target[0] - self.pos[0], target[1] - self.pos[1])
        if dist < 1e-6:
            return
        # leaving and arriving on arcs where they fit
        poses = connect(self.pos, self.heading, target, heading, self.cfg.turn_radius, self.step,
                        self.transit.is_free, self.transit.path)
        if poses is None:
            self.warnings.append(f'no way found from ({self.pos[0]:.2f}, {self.pos[1]:.2f}) to ({target[0]:.2f}, '
                                 f'{target[1]:.2f}): the path jumps there')
            self.new_section()
            self.pos = target
            return
        self.add(TRANSIT, poses)

    def _distance(self, a: Optional[XY], b: XY) -> float:
        return 0.0 if a is None else math.hypot(b[0] - a[0], b[1] - a[1])

    # loops

    def _ring_distance(self, ring: List[XY]) -> float:
        if self.pos is None:
            return 0.0
        return closest_on_polyline(ring + [ring[0]], self.pos)[2]

    def _end_distance(self, ring: List[XY]) -> float:
        return closest_on_polyline(ring + [ring[0]], self.end)[2]

    def drive_ring(self, ring: List[XY], hole: bool, kind: str = PERIMETER, maneuver: str = '',
                   target: Optional[XY] = None, level: int = 0, spacing: Optional[float] = None,
                   radius: Optional[float] = None) -> None:
        """One loop. ring comes as rings() gives it (outline ccw, hole cw: the mowable side on the left).
        It starts closest to target (default: a little ahead of the mower). Its corners are rounded with
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
        if direction == 'auto':
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
        first = _first_direction(loop)
        self.go_to(entry, math.atan2(first[1], first[0]) if first else None)
        # with the mowable side on the left only left turns go round a corner of it and get rounded
        poses = drive_polyline(loop, self.step, radius, self.area.is_free, -1 if flip else 1,
                               CORNER_CUT * self.cfg.mower_width, level, spacing, self.cfg.mower_width)
        self.add(kind, poses, maneuver=maneuver)
        self.loops.append([(p.x, p.y) for p in poses])

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
            outlines.append((k, [ring for kind, ring in rings(level) if kind == 'outline']))
            for kind, ring in rings(level):
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
        todo = [(kind == 'hole', ring) for kind, ring in rings(geom)]
        keep = min(todo, key=lambda t: self._end_distance(t[1]), default=None) \
            if finish and self.end is not None else None
        while todo:
            item = min([t for t in todo if t is not keep] or todo, key=lambda t: self._ring_distance(t[1]))
            todo.remove(item)
            self.drive_ring(item[1], item[0], LANE, 'ring', target=self.end if item is keep else None, radius=0.0)

    def concentric(self, levels: List[object], middles: List[object], perimeter: int, last: bool,
                   spacing: float) -> None:
        """Every loop of every level (and the loops halfway in where a part ends, middles[k] after level k).
        A loop is only mowed after the loops further out next to it (with last: further in), and the closest
        of the ready ones is next: so each part is mowed as a spiral, in (or out with last), before the mower
        moves on. Loops of the first `perimeter` levels count as perimeter loops, the rest as lanes."""
        loops = []  # (depth, hole, ring, line), depth k for level k, k + 0.5 for middles[k]
        for depth, geom in [(k, g) for k, g in enumerate(levels)] + [(k + 0.5, g) for k, g in enumerate(middles)]:
            for kind, ring in rings(geom):
                if depth < perimeter:
                    skip = self.cfg.skip_obstacle_loops if kind == 'hole' else self.cfg.skip_outline_loops
                else:
                    skip = self.cfg.skip_lanes
                if not skip:
                    loops.append((depth, kind == 'hole', ring, LineString(ring + [ring[0]])))
        loops.sort(key=lambda loop: loop[0])
        near = 1.5 * self.cfg.lane_spacing
        waits = []
        for depth, _, _, line in loops:
            before = (lambda d: depth < d <= depth + 1) if last else (lambda d: depth - 1 <= d < depth)
            waits.append({j for j, (dj, _, _, other) in enumerate(loops)
                          if before(dj) and line.distance(other) <= near})
        # with an end point, of the loops nothing waits for (going out the outermost ones, going in the
        # innermost ones) the one closest to it comes last
        reserved = None
        if self.end is not None:
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
            depth, hole, ring, _ = loops[i]
            final = len(done) == len(loops) and self.end is not None
            # a middle loop (depth k + 0.5) is half a spacing further in, rounded like the next level's
            level, gap = int(math.ceil(depth)), spacing
            if depth < perimeter:
                self.drive_ring(ring, hole, target=self.end if final else None, level=level, spacing=gap)
            else:
                self.drive_ring(ring, hole, LANE, 'ring', target=self.end if final else None, level=level,
                                spacing=gap)

    # lanes

    def cells(self, cells: List[List[Lane]], angle: float, finish: bool = False) -> None:
        """Mows the cells, the closest one not mowed yet next. finish: the cell that can end closest to the
        end point goes last."""
        c, s = math.cos(angle), math.sin(angle)

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

    def _turn_into(self, a: XY, yaw: float) -> bool:
        """The next cell's first lane right next to where the mower is and the other way round: that's a turn
        between lanes, not a drive. True if a turn got there."""
        if not self.started or self.heading is None or angle_diff(yaw, self.heading + math.pi) > 1e-3 or \
                math.hypot(a[0] - self.pos[0], a[1] - self.pos[1]) > 4.0 * self.cfg.turn_radius:
            return False
        turn = plan_turn(Pose2D(self.pos[0], self.pos[1], self.heading), a, self.cfg.turn_radius,
                         self.cfg.allow_reverse, self.step, self.area.is_free, self.transit.path,
                         self.cfg.allowed_turns)
        if turn is None:
            return False
        for piece in turn.pieces:
            self.add(TURN, piece.poses, piece.reverse, turn.name)
        self._count('turns_' + turn.name)
        return True

    def visit_order(self, lanes: List[Lane]) -> List[Lane]:
        """The cell's lanes (one per row, in row order) in the order they're mowed."""
        if self.cfg.lane_order != 'skip' or len(lanes) < 3:
            return lanes
        spacing = abs(lanes[1].y - lanes[0].y)
        if spacing <= 0.0:
            return lanes
        # lanes k apart leave room for a U-turn of turn_radius. Every pass goes the same way, the turn from
        # the end of one pass to the start of the next is a long U-turn along the headland
        k = max(1, int(math.ceil(2.0 * self.cfg.turn_radius / spacing - 1e-9)))
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
        for i, lane in enumerate(lanes):
            xa, xb = (lane.x0, lane.x1) if direction > 0 else (lane.x1, lane.x0)
            a, b = to_map(xa, lane.y), to_map(xb, lane.y)
            yaw = normalize_angle(angle if direction > 0 else angle + math.pi)
            if i == 0 and self._turn_into(a, yaw):
                pass
            elif i == 0:
                self.go_to(a, yaw)
            else:
                turn = plan_turn(Pose2D(self.pos[0], self.pos[1], self.heading), a, self.cfg.turn_radius,
                                 self.cfg.allow_reverse, self.step, self.area.is_free, self.transit.path,
                                 self.cfg.allowed_turns)
                if turn is not None:
                    for piece in turn.pieces:
                        self.add(TURN, piece.poses, piece.reverse, turn.name)
                    self._count('turns_' + turn.name)
                else:
                    pts = self.transit.path(self.pos, a)
                    if pts is None:
                        return lanes[i:]
                    self.add(TRANSIT, drive_polyline(pts, self.step, self.cfg.turn_radius,
                                                     self.transit.is_free), maneuver='turn_fallback')
                    self._count('turns_fallback')
            self.add(LANE, sample_straight(a[0], a[1], b[0], b[1], yaw, self.step))
            self.lanes.append((a, b))
            direction = -direction
        return None


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


def _assemble(segments: List[PathSegment]) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    poses: List[Pose2D] = []
    infos: List[SegmentInfo] = []
    for seg in segments:
        start = None
        for p in seg.poses:
            if poses and same_position(poses[-1], p):
                # one pose per position, with the heading the mower leaves it with
                poses[-1] = p
                if start is None:
                    start = len(poses) - 1
                continue
            poses.append(p)
            if start is None:
                start = len(poses) - 1
        if start is not None and len(poses) - 1 > start:
            infos.append(SegmentInfo(seg.kind, start, len(poses) - 1, seg.reverse, seg.maneuver, seg.section))
    return poses, infos


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
                  drivable: Sequence[Sequence[Sequence[float]]] = ()) -> CoveragePlan:
    """Plans the coverage path of one area.

    boundary: [(x, y), ...] of the outline, obstacles: a list of those for no-go zones, all in metres in
    one flat frame (map). start: (x, y) or (x, y, yaw) of the mower, the path starts on the loop (or
    lane) closest to it (default: the outline's first point). end: (x, y) to finish close to, e.g. the
    docking station. drivable: areas the mower may drive through between the parts of this one but doesn't
    mow (navigation areas, other lawns). Raises PlanningError when there's nothing to plan, or a setting or
    point is out of range (every coordinate within MAX_COORDINATE of the origin).
    """
    t0 = time.perf_counter()
    cfg = config or PlannerConfig()
    cfg.validate()
    _check_coordinates('the boundary', boundary)
    for o in obstacles:
        _check_coordinates('an obstacle', o)
    for d in drivable:
        _check_coordinates('a drivable area', d)
    _check_coordinates('start or end', [p for p in (start, end) if p is not None])
    half = 0.5 * cfg.mower_width
    first = cfg.first_loop_offset
    spacing = cfg.lane_spacing
    concentric = cfg.fill_pattern == 'concentric'

    area = clean_polygon(boundary)
    if area.is_empty:
        raise PlanningError('the boundary needs at least 3 points that enclose an area')
    blocked = union(clean_polygon(o) for o in obstacles)
    free = polygonal(area.difference(blocked)) if not blocked.is_empty else area
    if cfg.simplify_tolerance > 0 and not free.is_empty:
        simple = free.simplify(cfg.simplify_tolerance, preserve_topology=True)
        # simplifying a very jagged outline can make it cross itself, then it stays as it is
        if simple.is_valid:
            free = polygonal(simple)
    if free.is_empty:
        raise PlanningError('the obstacles cover the whole area')
    # slivers smaller than the blade (where a noisy outline crosses itself) aren't parts of their own
    free = polygonal(union(p for p in polygons(free) if p.area >= half * half))
    robot_space = inset(free, first)
    if robot_space.is_empty:
        raise PlanningError(f'the area is too narrow for a first loop {first} m inside the outline')
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
    count = int(cfg.perimeter_passes)
    # concentric loops meet at the corners: at a right angle the corners of two loops are spacing * sqrt(2)
    # apart, which the blade only covers up to mower_width / sqrt(2)
    loop_spacing = min(spacing, cfg.mower_width / math.sqrt(2.0)) if concentric else spacing
    levels = []
    while concentric or len(levels) < count:
        level = robot_space if not levels else inset(free, first + len(levels) * loop_spacing)
        if level.is_empty:
            break
        levels.append(level)
        if len(levels) > MAX_ROWS:
            raise PlanningError('too many loops: the area is too big for this lane spacing')
    perimeter_levels = levels[:count]
    took('perimeter')

    def middle(k: int, inner, step: float):
        # where the area ends inside loop level k (loops `step` apart) and `inner`, the next level in, doesn't
        # get there, what's left is narrower than a step: one more loop halfway in mows it (insets add up:
        # that's level k shrunk by half a step, quicker than free shrunk again)
        half_in = inset(levels[k], 0.5 * step)
        if not half_in.is_empty and not inner.is_empty:
            half_in = half_in.difference(inner.buffer(step, QUAD_SEGS))
        return polygonal(union(p for p in polygons(half_in) if p.area >= half * half))

    middles = [middle(k, inner, loop_spacing) for k, inner in enumerate(levels[1:] + [Polygon()])] \
        if concentric else []

    # 2. what's left after the loops (kept for checks and plots, the lanes use the same edge directly)
    remaining = polygonal(subtract(free, [LineString(ring + [ring[0]]).buffer(half, QUAD_SEGS)
                                          for level in perimeter_levels for _, ring in rings(level)]))
    lane_passes = max(0, count - int(cfg.lane_overlap_passes))
    lane_region = inset(free, first + lane_passes * spacing)
    # where the area is too narrow for lanes but wider than the loops reach (a strip of 0.8 to 1 m with the
    # defaults, or narrower ones where the loops stop earlier), what the loops leave in the middle gets a
    # loop of its own, halfway between the last loop there and the next one in (or where the lanes would be)
    narrow = Polygon() if concentric or cfg.skip_lanes else polygonal(union(
        middle(k, inner, spacing) for k, inner in enumerate((levels[:lane_passes] + [lane_region])[1:])))
    took('remaining')
    angle = lane_angle(area, free, cfg, lane_region, remaining, robot_space)

    # 3. lanes, for crosshatch a second time across them
    passes: List[Tuple[float, List[List[Lane]]]] = []
    if not concentric and not cfg.skip_lanes and not lane_region.is_empty:
        angles = [angle]
        if cfg.fill_pattern == 'crosshatch':
            second = math.fmod(angle + cfg.crosshatch_angle, math.pi)
            angles.append(second + math.pi if second < 0 else second)
        for a in angles:
            rows = scan_lanes(lane_region, a, spacing, cfg.min_lane_length, remaining, robot_space, half)
            passes.append((a, split_cells(rows)))
    # (remaining counts every loop, so not when some are left out)
    if not narrow.is_empty and passes and not (cfg.skip_outline_loops or cfg.skip_obstacle_loops):
        # a narrow piece needs no loop where the lanes already mow what the loops leave around it (they reach
        # on into a strip along them, see scan_lanes). The lanes of each pass by row, so only those next to a
        # piece are looked at
        by_row = [(a, sorted((lane.y, lane.x0, lane.x1) for cell in cells for lane in cell)) for a, cells in passes]

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

    # 4. route
    start_xy = _point(start)
    start_heading = float(start[2]) if start is not None and len(start) >= 3 and start[2] is not None else None
    end_xy = _point(end)
    if start_xy is None:
        corner = list(polygons(area))[0].exterior.coords[0]
        start_xy = (corner[0], corner[1])
    area_space = TransitPlanner(robot_space, cfg.transit_clearance)
    transit_space = robot_space
    extra_space = union(clean_polygon(d) for d in drivable)
    if not extra_space.is_empty:
        # drivable areas are recorded like outlines, the mower's centre may go anywhere in them, but not
        # closer to an obstacle than in the mowing area
        keep_off = blocked.buffer(max(first, half), QUAD_SEGS) if not blocked.is_empty else None
        if keep_off is not None:
            extra_space = extra_space.difference(keep_off)
        transit_space = polygonal(union([robot_space, extra_space]))
    transit = area_space if transit_space is robot_space else TransitPlanner(transit_space, cfg.transit_clearance)
    route = _Route(cfg, transit, area_space, start_xy, start_heading, end_xy)
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

    for i, level in enumerate(levels):
        for k, geom in split(level).items():
            todo(k)['levels'][i] = geom
    for i, middle in enumerate(middles):
        for k, geom in split(middle).items():
            todo(k)['middles'][i] = geom
    for pi, (a, cells) in enumerate(passes):
        for cell in cells:
            if cell:
                todo(lane_part(a, cell))['cells'][pi].append(cell)
    for k, geom in split(narrow).items():
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
        loops_here = not concentric and any(not skip[kind] for g in w['levels'].values() for kind, _ in rings(g))
        loops_end = loops_here and (last or not any(w['cells']) and w['narrow'] is None)
        last_pass = max((i for i, cells in enumerate(w['cells']) if cells), default=None)

        def loops():
            if concentric:
                route.concentric([w['levels'].get(i, empty) for i in range(len(levels))],
                                 [w['middles'].get(i, empty) for i in range(len(middles))], count, last,
                                 loop_spacing)
            else:
                route.perimeter([w['levels'].get(i, empty) for i in range(len(perimeter_levels))], holes, last,
                                loops_end)

        def lanes():
            # the strips too narrow for lanes first, so the lanes can end at the end point
            if w['narrow'] is not None:
                route.extra_loops(w['narrow'], finish=not loops_end and last_pass is None)
            for i, (a, _) in enumerate(passes):
                route.cells(w['cells'][i], a, finish=not loops_end and i == last_pass)

        if last:
            lanes()
            loops()
        else:
            loops()
            lanes()
    poses, segments = _assemble(route.segments)
    took('route')
    if not poses:
        raise PlanningError('nothing to mow: the area is too small for one loop or lane')
    section_starts = [s.start for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section]
    if len(section_starts) > 1:
        route.warnings.append(f'the area has {len(section_starts)} parts with no way between them: the path '
                              'jumps from one to the next')

    lengths: Dict[str, float] = {PERIMETER: 0.0, LANE: 0.0, TURN: 0.0, TRANSIT: 0.0}
    for seg in segments:
        lengths[seg.kind] += path_length(poses[seg.start:seg.end + 1])
    stats: Dict[str, Any] = {
        'sections': len(section_starts),
        'path_length': sum(lengths.values()),
        'perimeter_length': lengths[PERIMETER],
        'lane_length': lengths[LANE],
        'turn_length': lengths[TURN],
        'transit_length': lengths[TRANSIT],
        'perimeter_loops': sum(1 for s in segments if s.kind == PERIMETER),
        'loops': len(route.loops),
        'lanes': len(route.lanes),
        'cells': sum(len(cells) for _, cells in passes),
        'poses': len(poses),
        'area': free.area,
        'planning_time': time.perf_counter() - t0,
        'timings': timings,
    }
    for key in ('turns_u_turn', 'turns_bulb', 'turns_k_turn', 'turns_detour', 'turns_pivot', 'turns_fallback'):
        stats[key] = route.counts.get(key, 0)

    return CoveragePlan(
        poses=poses, segments=segments, angle=angle, lane_spacing=spacing, perimeter_loops=route.loops,
        lanes=route.lanes, stats=stats, warnings=route.warnings, area=area, free_space=free,
        robot_space=robot_space, lane_region=lane_region, remaining=remaining,
        angles=[a for a, _ in passes] or [angle], transit_space=transit_space, config=cfg,
        section_starts=section_starts)


def swept_area(plan: CoveragePlan, mower_width: float):
    """What the blade covers along the whole path (a disk of mower_width), for checks and plots."""
    ends = list(plan.section_starts[1:]) + [len(plan.poses)]
    lines = [LineString([(p.x, p.y) for p in plan.poses[a:b]])
             for a, b in zip(plan.section_starts, ends) if b - a >= 2]
    if not lines:
        return Polygon()
    return union(line.buffer(0.5 * mower_width, 4) for line in lines)
