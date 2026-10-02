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
from shapely.ops import nearest_points

from .footprint import SPIN, Footprint
from .geometry import QUAD_SEGS, clean_polygon, inset, polygonal, polygons, rings, subtract, union
from .maneuvers import (CORNER_ANGLE, Maneuver, closest_on_polyline, connect, drive_polyline, path_length,
                        plan_turn, same_position, sample_arc, sample_straight)
from .mathutil import Pose2D, angle_diff, normalize_angle
from .transit import TransitPlanner

XY = Tuple[float, float]

PERIMETER = 'perimeter'
LANE = 'lane'
TURN = 'turn'
TRANSIT = 'transit'

ANGLE_STRATEGIES = ('longest_edge', 'min_width', 'optimal')
# 'nearest': the closest cell not mowed yet next, 'optimized': the order with the shortest drives between them
ROUTE_ORDERS = ('nearest', 'optimized')
# orders route_order 'optimized' tries at most for one set of cells, so it doesn't take long with many
ORDER_BUDGET = 4000
LANE_ORDERS = ('snake', 'skip')
# lanes: parallel lanes; crosshatch: lanes, then lanes across them; concentric: loops further and further in
FILL_PATTERNS = ('lanes', 'crosshatch', 'concentric')
PERIMETER_ORDERS = ('first', 'last')
# where the lane region is narrower than a U-turn: 'lanes' anyway, or 'loops' further in instead
NARROW_PARTS = ('lanes', 'loops')
PERIMETER_DIRECTIONS = ('auto', 'ccw', 'cw')
TURN_TYPES = ('u_turn', 'bulb', 'k_turn', 'detour', 'pivot')
# 'recorded': the edges are where the mower's centre drove when they were recorded, 'hard': a wall
EDGES = ('recorded', 'hard')
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
# m, a blade further off the mower's centre is a mistake
MAX_BLADE_OFFSET = 1.0
# m, lane ends are pulled back to this when the body doesn't fit at them
BODY_STEP = 0.01


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
    # mowing areas of the map lying in the one planned (an area of its own with another angle or pattern): left out
    # and mowed round like an obstacle, they get their own plan. For the map's areas (mapfile.area_job), not here
    nested_areas: bool = True
    # where the lanes would lie in a part narrower than a U-turn (2 * turn_radius: a path, a strip between beds, and
    # what's left between such parts too small for more than a few lanes): 'lanes' anyway, 'loops': the loops go on
    # in there instead until it's mowed, the pieces they come apart into driven on the way round (in from the loop
    # next to them and back out), so no trips along a path from piece to piece
    narrow_parts: str = 'lanes'
    route_order: str = 'nearest'  # cells: 'nearest' one next, or 'optimized' (shortest drives between them)
    perimeter_corner_radius: float = 0.15  # m, outline corners of the loops are rounded this much, 0 = sharp
    waypoint_spacing: float = 0.10  # m, distance between poses, 0 = only where the direction changes
    transit_clearance: float = 0.05  # m, transits keep this far from the outer loop and obstacle loops
    simplify_tolerance: float = 0.01  # m, the outline and obstacles are simplified this much first
    min_lane_length: float = 0.10  # m, shorter pieces of a lane are left out
    # m, a row split by an obstacle with at most this much between its pieces (a tree, a bed) becomes one lane
    # that bends around it, if the bend stays within bend_max_offset of the row's line: fewer cells, the
    # zigzag goes on past the obstacle. 0: the row is split
    bend_max_gap: float = 0.0
    bend_max_offset: float = 0.5  # m
    # m, drives between the parts of the plan keep this far off the edges (a wall, a bed) where that doesn't
    # cost much more (transit_edge_cost), 0: the shortest way
    transit_edge_distance: float = 0.0
    # every metre of a drive closer to the edge than transit_edge_distance counts this much more
    transit_edge_cost: float = 2.0
    # lanes reach this many perimeter loops further out (OpenMower's outline_overlap_count)
    lane_overlap_passes: int = 0
    # leave parts out (OpenMower's skip_area_outline, skip_obstacle_outlines, skip_fill), the rest is
    # planned as if they were there
    skip_outline_loops: bool = False
    skip_obstacle_loops: bool = False
    skip_lanes: bool = False
    # m, the mower's body around its centre (base_link): robot_width wide, robot_front ahead of it, robot_rear
    # behind. With all three the turns, lane ends, loops and drives keep the body in the space it may take (see
    # edges). None: not checked
    robot_width: Optional[float] = None
    robot_front: Optional[float] = None
    robot_rear: Optional[float] = None
    # 'recorded': the outline and obstacles are where the centre drove, the body may be where it was when they were
    # recorded (half its width each side, the front past corners and on the outside of bends, turning at corners).
    # 'hard': they're walls, nothing of the body goes past them, the centre keeps robot_width / 2 off them (the first
    # loop moves further in where perimeter_offset would come closer)
    edges: str = 'recorded'
    # m, the body may reach this much further than the edges allow before it counts as sticking out (a recorded edge
    # is a few cm off where the mower was)
    body_tolerance: float = 0.05
    # m, how far the blade's centre sits to the left of the mower's centre (negative: right). The loops go
    # round with the blade on their outer side, so it mows up to perimeter_offset while the body stays further
    # in, the lanes move so the blade's lines stay a lane spacing apart
    blade_offset: float = 0.0
    # m, how far the blade's centre sits ahead of the mower's centre (the point the path is for: OpenMower's
    # base_link between the drive wheels; negative: behind it). The lanes start and end that much earlier, so the
    # blade mows them where they are
    blade_ahead: float = 0.0

    @property
    def lane_spacing(self) -> float:
        return self.mower_width * (1.0 - self.overlap)

    @property
    def first_loop_offset(self) -> float:
        return 0.5 * self.mower_width if self.perimeter_offset is None else self.perimeter_offset

    @property
    def blade_loop_offset(self) -> float:
        """m, how far inside the outline (and off the obstacles) the blade's centre runs on the first loop:
        first_loop_offset, further in where the body (robot_width, hard edges) wouldn't fit there."""
        if self.robot_width is None or self.edges != 'hard':
            return self.first_loop_offset
        return max(self.first_loop_offset, 0.5 * self.robot_width - abs(self.blade_offset))

    @property
    def centre_offset(self) -> float:
        """m, how far the mower's centre keeps off the outline and the obstacles."""
        return self.blade_loop_offset + abs(self.blade_offset)

    @property
    def body(self) -> bool:
        """Whether the body is checked: robot_width, robot_front and robot_rear set."""
        return None not in (self.robot_width, self.robot_front, self.robot_rear)

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
        finite('bend_max_gap', 0.0)
        finite('bend_max_offset', 0.0)
        finite('transit_edge_distance', 0.0)
        finite('transit_edge_cost', 0.0)
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
        for name in ('robot_width', 'robot_front', 'robot_rear'):
            if getattr(self, name) is not None:
                finite(name, 0.0)
        finite('body_tolerance', 0.0, MAX_BLADE_OFFSET)
        finite('blade_offset', -MAX_BLADE_OFFSET, MAX_BLADE_OFFSET)
        finite('blade_ahead', -MAX_BLADE_OFFSET, MAX_BLADE_OFFSET)
        for name in ('perimeter_passes', 'lane_overlap_passes'):
            v = getattr(self, name)
            whole = isinstance(v, int) or isinstance(v, float) and math.isfinite(v) and v == int(v)
            if isinstance(v, bool) or not whole or not 0 <= v <= MAX_PASSES:
                raise PlanningError(f'{name} must be a whole number 0..{MAX_PASSES}, got {v!r}')
        if self.route_order not in ROUTE_ORDERS:
            raise PlanningError(f'route_order must be one of {", ".join(ROUTE_ORDERS)}, got {self.route_order!r}')
        if self.lane_order not in LANE_ORDERS:
            raise PlanningError(f'lane_order must be one of {", ".join(LANE_ORDERS)}, got {self.lane_order!r}')
        for name, allowed in (('angle_strategy', ANGLE_STRATEGIES), ('fill_pattern', FILL_PATTERNS),
                              ('perimeter_order', PERIMETER_ORDERS), ('perimeter_direction', PERIMETER_DIRECTIONS),
                              ('edges', EDGES), ('narrow_parts', NARROW_PARTS)):
            if getattr(self, name) not in allowed:
                raise PlanningError(f'{name} must be one of {", ".join(allowed)}, got {getattr(self, name)!r}')
        if not isinstance(self.turn_types, (tuple, list)) or not self.turn_types or \
                any(t not in TURN_TYPES for t in self.turn_types):
            raise PlanningError(f'turn_types must be some of {", ".join(TURN_TYPES)}, got {self.turn_types!r}')
        for name in ('allow_reverse', 'skip_outline_loops', 'skip_obstacle_loops', 'skip_lanes', 'nested_areas'):
            if not isinstance(getattr(self, name), bool):
                raise PlanningError(f'{name} must be true or false, got {getattr(self, name)!r}')


# settings where None means something (worked out, mower_width / 2), and the ones that take a float
_OPTIONAL = ('angle', 'angle_min', 'angle_max', 'perimeter_offset', 'robot_width', 'robot_front', 'robot_rear')
_FLOATS = tuple(f.name for f in dataclasses.fields(PlannerConfig) if isinstance(f.default, float)) + _OPTIONAL


@dataclass
class PathSegment:
    kind: str  # PERIMETER, LANE, TURN or TRANSIT
    poses: List[Pose2D]
    reverse: bool = False  # driven backwards (the middle leg of a K-turn)
    # turns: 'u_turn', 'bulb', 'k_turn', 'detour', 'pivot'; transits replacing a turn: 'turn_fallback'
    maneuver: str = ''
    section: int = 0
    source: str = ''  # what it mows or leads to: 'loop-<i>', 'lane-<i>' (see CoveragePlan.origins)


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
    source: str = ''  # the loop or lane it mows or leads to: 'loop-<i>', 'lane-<i>'


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
    robot_space: object = None  # where the mower's centre may go: free_space shrunk by centre_offset
    lane_region: object = None  # where the lane centres lie
    remaining: object = None  # free_space minus the strips the perimeter loops mow
    angles: List[float] = dataclasses.field(default_factory=list)  # of every set of lanes (crosshatch: 2)
    transit_space: object = None  # robot_space and the drivable areas
    body_space: object = None  # where the mower's body may be (robot_width, robot_front, robot_rear set)
    config: Optional[PlannerConfig] = None  # what it was planned with
    section_starts: List[int] = dataclasses.field(default_factory=lambda: [0])  # first pose of every section
    # where things come from, for following a plan through its stages (export's Planner Studio payload):
    # the loop levels (level k: free_space shrunk by the k-th loop's offset), the strips too narrow for lanes,
    # the lanes of every set as cells (lane frame, see Lane), and for every 'loop-<i>' and 'lane-<i>' (index
    # into perimeter_loops, lanes) what it was made from: {'parent': 'level-<k>' | 'middle-<k>' | 'narrow' |
    # 'cell-<set>-<index>' (into cells), 'kind': ...}
    levels: List[object] = dataclasses.field(default_factory=list)
    narrow: object = None
    cells: List[Tuple[float, List[List['Lane']]]] = dataclasses.field(default_factory=list)
    origins: Dict[str, Dict[str, Any]] = dataclasses.field(default_factory=dict)


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
    # where it bends around an obstacle (bend_max_gap): from x, to x, the way between in the map (from the
    # piece before to the one after), in order along x
    bends: Tuple[Tuple[float, float, Tuple[XY, ...]], ...] = ()

    def ends(self, angle: float) -> Tuple[XY, XY]:
        """Where it starts and ends in the map, for lanes at angle."""
        c, s = math.cos(angle), math.sin(angle)
        return ((self.x0 * c - self.y * s, self.x0 * s + self.y * c),
                (self.x1 * c - self.y * s, self.x1 * s + self.y * c))

    def pieces(self) -> List[Tuple[float, float]]:
        """The straight pieces, without the bends."""
        xs = [self.x0] + [x for x0, x1, _ in self.bends for x in (x0, x1)] + [self.x1]
        return list(zip(xs[::2], xs[1::2]))


def bend_lanes(rows: List[List[Lane]], angle: float, space: TransitPlanner, max_gap: float,
               max_offset: float) -> List[List[Lane]]:
    """Joins the pieces of a row with at most max_gap between them (where a small obstacle splits the row)
    into one lane that bends around it, on the shortest way through space, if that way never leaves the
    row's line by more than max_offset nor goes back. Fewer cells: the zigzag goes on past the obstacle."""
    c, s = math.cos(angle), math.sin(angle)
    out = []
    for row in rows:
        joined: List[Lane] = []
        for lane in row:
            prev = joined[-1] if joined else None
            if prev is not None and 0.0 < lane.x0 - prev.x1 <= max_gap:
                p = (prev.x1 * c - prev.y * s, prev.x1 * s + prev.y * c)
                q = (lane.x0 * c - lane.y * s, lane.x0 * s + lane.y * c)
                way = space.path(p, q)
                if way is not None:
                    local = [(x * c + y * s, -x * s + y * c) for x, y in way]
                    if all(abs(v - lane.y) <= max_offset for _, v in local) and \
                            all(u1 >= u0 - 1e-9 for (u0, _), (u1, _) in zip(local, local[1:])):
                        joined[-1] = Lane(prev.row, prev.x0, lane.x1, prev.y,
                                          prev.bends + ((prev.x1, lane.x0, tuple(way)),) + lane.bends)
                        continue
            joined.append(lane)
        out.append(joined)
    return out


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
        # what the segments added now mow or lead to, and where every loop and lane comes from
        self.source = ''
        self.origins: Dict[str, Dict[str, Any]] = {}
        self.cell_of: Dict[int, str] = {}  # lane (by id) -> its cell, 'cell-<set>-<index>' in CoveragePlan.cells

    def _count(self, key: str) -> None:
        self.counts[key] = self.counts.get(key, 0) + 1

    def add(self, kind: str, poses: List[Pose2D], reverse: bool = False, maneuver: str = '') -> None:
        if not poses:
            return
        self.segments.append(PathSegment(kind, poses, reverse, maneuver, self.section, self.source))
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
                        self.transit.is_free, self.transit.careful_path, self.body)
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

    def _ring_distance(self, ring: List[XY]) -> float:
        if self.pos is None:
            return 0.0
        return closest_on_polyline(ring + [ring[0]], self.pos)[2]

    def _end_distance(self, ring: List[XY]) -> float:
        return closest_on_polyline(ring + [ring[0]], self.end)[2]

    def drive_ring(self, ring: List[XY], hole: bool, kind: str = PERIMETER, maneuver: str = '',
                   target: Optional[XY] = None, level: int = 0, spacing: Optional[float] = None,
                   radius: Optional[float] = None, parent: Optional[str] = None) -> None:
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
        if self.cfg.blade_offset != 0:
            # rings come with the mowable side on the left: the blade goes on the other side, but for the rings
            # driven with the blade on the inner one
            flip = (self.cfg.blade_offset > 0) != (id(ring) in self.blade_in)
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
        first = _first_direction(loop)
        name = f'loop-{len(self.loops)}'
        self.source = name
        self.origins[name] = {'parent': parent or f'level-{level}', 'kind': 'obstacle' if hole else 'outline',
                              'level': level}
        self.go_to(entry, math.atan2(first[1], first[0]) if first else None)
        # with the mowable side on the left only left turns go round a corner of it and get rounded
        poses = drive_polyline(loop, self.step, radius, self.area.is_free, -1 if flip else 1,
                               CORNER_CUT * self.cfg.mower_width, level, spacing, self.cfg.mower_width, self.body)
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
            level_rings = self.centre_rings(level)
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
                parent[2] = _splice(parent[2], item[2])
                parent[3] = LineString(parent[2] + parent[2][:1])
                item[2] = None
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

    def _turn_into(self, a: XY, yaw: float) -> bool:
        """The next cell's first lane right next to where the mower is and the other way round: that's a turn
        between lanes, not a drive. True if a turn got there."""
        if not self.started or self.heading is None or angle_diff(yaw, self.heading + math.pi) > 1e-3 or \
                math.hypot(a[0] - self.pos[0], a[1] - self.pos[1]) > 4.0 * self.cfg.turn_radius:
            return False
        turn = plan_turn(Pose2D(self.pos[0], self.pos[1], self.heading), a, self.cfg.turn_radius,
                         self.cfg.allow_reverse, self.step, self.area.is_free, self.transit.path,
                         self.cfg.allowed_turns, self.body)
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
            head = b  # where the lane's first straight piece ends
            if self.footprint is not None:
                # the side the turns come from and go to (left of the lanes going +x is +y)
                side = [0.0 if o is None else direction * math.copysign(1.0, o.y - lane.y)
                        for o in (lanes[i - 1] if driven else None, lanes[i + 1] if i + 1 < len(lanes) else None)]
                if lane.bends:
                    # the straight pieces at its ends
                    pieces = lane.pieces()[::direction]
                    first = to_map((pieces[0][1] if direction > 0 else pieces[0][0]) - back, y)
                    last = to_map((pieces[-1][0] if direction > 0 else pieces[-1][1]) - back, y)
                    ends = self.body_ends(a, first, yaw, side[0], 0.0), self.body_ends(last, b, yaw, 0.0, side[1])
                    head = ends[0][1] if ends[0] is not None else head
                    ends = None if None in ends else (ends[0][0], ends[1][1])
                else:
                    ends = self.body_ends(a, b, yaw, side[0], side[1])
                if ends is None:
                    continue
                a, b = ends
            name = f'lane-{len(self.lanes)}'
            self.source = name
            self.origins[name] = {'parent': self.cell_of.get(id(lane), ''),
                                  'kind': 'bent' if lane.bends else 'straight', 'row': lane.row}
            if driven == 0 and self._turn_into(a, yaw):
                pass
            elif driven == 0:
                self.go_to(a, yaw)
            else:
                exit_pose = Pose2D(self.pos[0], self.pos[1], self.heading)

                def turn_to(entry: XY, types) -> Optional[Maneuver]:
                    return plan_turn(exit_pose, entry, self.cfg.turn_radius, self.cfg.allow_reverse, self.step,
                                     self.area.is_free, self.transit.path, types, self.body)

                # the turn goes level with the lane end further out, where the body may not fit (body_ends only
                # made room for a turn level with each end): this lane starts level with the last one's end then,
                # before a detour or a pivot
                ux, uy = math.cos(yaw), math.sin(yaw)
                level = (self.pos[0] - a[0]) * ux + (self.pos[1] - a[1]) * uy
                rest = math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length
                # (not by more than a turn reaches: what it leaves of the lane would stay unmowed)
                most = min(rest, self.footprint.reach + self.cfg.turn_radius) if self.footprint is not None else 0.0
                turn = None
                if self.footprint is not None and 1e-6 < level < most:
                    gentle = tuple(t for t in self.cfg.allowed_turns if t not in ('detour', 'pivot'))
                    turn = turn_to(a, gentle)
                    if turn is None:
                        start = (a[0] + level * ux, a[1] + level * uy)
                        turn = turn_to(start, gentle)
                        if turn is not None:
                            a = start
                            self._count('lanes_body')
                if turn is None:
                    turn = turn_to(a, self.cfg.allowed_turns)
                if turn is not None:
                    for piece in turn.pieces:
                        self.add(TURN, piece.poses, piece.reverse, turn.name)
                    self._count('turns_' + turn.name)
                else:
                    pts = self.transit.path(self.pos, a)
                    if pts is None:
                        return lanes[i:]
                    self.add(TRANSIT, drive_polyline(pts, self.step, self.cfg.turn_radius, self.transit.is_free,
                                                     body=self.body), maneuver='turn_fallback')
                    self._count('turns_fallback')
            if lane.bends:
                self.bent_lane(lane, direction, a, b, to_map)
            else:
                self.add(LANE, sample_straight(a[0], a[1], b[0], b[1], yaw, self.step))
            self.lanes.append((a, b))
            direction = -direction
            driven += 1
        return None

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
                return not leave or self.footprint.poses_fit(sample_arc(x, y, yaw, r, leave * quarter, self.step))
            return not arrive or self.footprint.poses_fit(
                [Pose2D(p.x, p.y, p.yaw + math.pi) for p in sample_arc(x, y, yaw + math.pi, r, -arrive * quarter,
                                                                         self.step)])

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


DRIVEN_SIMPLIFY = 0.02


def _driven(geoms, half: float, ahead: float):
    """Where a body half wide each side and reaching ahead in front (and behind, the way round isn't known) was
    while its centre drove along the rings of geoms: every edge that much wider and longer, so past a corner and on
    the outside of a bend the front reaches out too, and at a corner (turning by more than SPIN) all round it, turning
    there."""
    parts = []
    reach = math.hypot(half, ahead)
    for geom in geoms:
        # (a recorded edge has a point every few cm: 2 cm simpler is a lot fewer pieces, and no more than that off)
        simple = geom.simplify(DRIVEN_SIMPLIFY, preserve_topology=True)
        for _, ring in rings(simple if simple.is_valid else geom):
            n = len(ring)
            for i, (bx, by) in enumerate(ring):
                (ax, ay), (cx, cy) = ring[i - 1], ring[(i + 1) % n]
                if angle_diff(math.atan2(cy - by, cx - bx), math.atan2(by - ay, bx - ax)) > SPIN:
                    parts.append(Point(bx, by).buffer(reach, 4))  # (a 16-gon, inside the circle)
            for (ax, ay), (bx, by) in zip(ring, ring[1:] + ring[:1]):
                length = math.hypot(bx - ax, by - ay)
                if length < 1e-9:
                    continue
                ux, uy = (bx - ax) / length, (by - ay) / length
                a, b = (ax - ahead * ux, ay - ahead * uy), (bx + ahead * ux, by + ahead * uy)
                parts.append(Polygon([(a[0] - half * uy, a[1] + half * ux), (b[0] - half * uy, b[1] + half * ux),
                                      (b[0] + half * uy, b[1] - half * ux), (a[0] + half * uy, a[1] - half * ux)]))
    return union(parts)


def _splice(ring: List[XY], other: List[XY]) -> List[XY]:
    """ring with a trip round other from the closest point between them: there, over to other, round it and
    back, on along ring."""
    closed, round_ = ring + ring[:1], other + other[:1]
    p, q = nearest_points(LineString(closed), LineString(round_))
    i, p, _ = closest_on_polyline(closed, (p.x, p.y))
    j, q, _ = closest_on_polyline(round_, (q.x, q.y))
    return ring[:i + 1] + [p, q] + round_[j + 1:] + round_[1:j + 1] + [q, p] + ring[i + 1:]


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
            infos.append(SegmentInfo(seg.kind, start, len(poses) - 1, seg.reverse, seg.maneuver, seg.section,
                                     seg.source))
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
                  drivable: Sequence[Sequence[Sequence[float]]] = (),
                  keep_out: Sequence[Sequence[Sequence[float]]] = ()) -> CoveragePlan:
    """Plans the coverage path of one area.

    boundary: [(x, y), ...] of the outline, obstacles: a list of those for no-go zones, all in metres in
    one flat frame (map). start: (x, y) or (x, y, yaw) of the mower, the path starts on the loop (or
    lane) closest to it (default: the outline's first point). end: (x, y) to finish close to, e.g. the
    docking station. drivable: areas the mower may drive through between the parts of this one but doesn't
    mow (navigation areas, other lawns). keep_out: where nothing goes on the way through those (inactive areas, ones
    not to mow), nor the body. Raises PlanningError when there's nothing to plan, or a setting or
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
    for k in keep_out:
        _check_coordinates('a keep out area', k)
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
    count = int(cfg.perimeter_passes)
    # concentric loops meet at the corners: at a right angle the corners of two loops are spacing * sqrt(2)
    # apart, which the blade only covers up to mower_width / sqrt(2)
    loop_spacing = min(spacing, cfg.mower_width / math.sqrt(2.0)) if concentric else spacing
    levels = []
    while concentric or len(levels) < count:
        level = blade_space if not levels else inset(free, first + len(levels) * loop_spacing)
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
    # the loops go on in where the lane region is narrower than a U-turn, the lanes only where it's wider: all the
    # loops are driven part by part like concentric's
    ringed = concentric
    if cfg.narrow_parts == 'loops' and not concentric and not cfg.skip_lanes and cfg.turn_radius > 0 and \
            len(levels) == count and not lane_region.is_empty:
        r = cfg.turn_radius
        opened = lane_region.buffer(-r, QUAD_SEGS).buffer(r, QUAD_SEGS)
        # what that takes away: narrow parts, and the tips of corners (smaller than r * r), which keep their lanes
        cut = [p for p in polygons(lane_region.difference(opened)) if p.area >= r * r]
        if cut:
            wide = polygonal(subtract(lane_region, cut))
            # bits left between narrow parts too small for more than a few lanes go to the loops as well
            wide = polygonal(union(p for p in polygons(wide) if p.area >= (4.0 * r) ** 2))
            keep_off = None if wide.is_empty else wide.buffer(half, QUAD_SEGS)  # the lanes mow that far round it
            while len(levels) < MAX_ROWS:
                level = inset(free, first + len(levels) * spacing)
                if keep_off is not None and not level.is_empty:
                    level = polygonal(union(p for p in polygons(level.difference(keep_off)) if p.area >= half * half))
                if level.is_empty:
                    break
                levels.append(level)
            middles = [middle(k, union([inner, wide]) if k + 1 >= lane_passes else inner, spacing)
                       for k, inner in enumerate(levels[1:] + [Polygon()])]
            lane_region, narrow, ringed = wide, Polygon(), True
    took('remaining')
    angle = lane_angle(area, free, cfg, lane_region, remaining, robot_space)

    # turns and loops stay inside the mowing area, so do the bends of lanes around obstacles
    edges = dict(edge_distance=cfg.transit_edge_distance, edge_cost=cfg.transit_edge_cost)
    area_space = TransitPlanner(robot_space, cfg.transit_clearance, **edges)

    # 3. lanes, for crosshatch a second time across them
    passes: List[Tuple[float, List[List[Lane]]]] = []
    if not concentric and not cfg.skip_lanes and not lane_region.is_empty:
        angles = [angle]
        if cfg.fill_pattern == 'crosshatch':
            second = math.fmod(angle + cfg.crosshatch_angle, math.pi)
            angles.append(second + math.pi if second < 0 else second)
        for a in angles:
            rows = scan_lanes(lane_region, a, spacing, cfg.min_lane_length, remaining, robot_space, half)
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

    # 4. route
    start_xy = _point(start)
    start_heading = float(start[2]) if start is not None and len(start) >= 3 and start[2] is not None else None
    end_xy = _point(end)
    if start_xy is None:
        corner = list(polygons(area))[0].exterior.coords[0]
        start_xy = (corner[0], corner[1])
    transit_space = robot_space
    extra_space = union(clean_polygon(d) for d in drivable)
    if not extra_space.is_empty:
        # drivable areas are recorded like outlines, the mower's centre may go anywhere in them, but not
        # closer to an obstacle (or an area to keep out of) than in the mowing area
        away = union([blocked] + [clean_polygon(k) for k in keep_out]) if keep_out else blocked
        keep_off = away.buffer(max(centre, half), QUAD_SEGS) if not away.is_empty else None
        if keep_off is not None:
            extra_space = extra_space.difference(keep_off)
        transit_space = polygonal(union([robot_space, extra_space]))
    transit = area_space if transit_space is robot_space else TransitPlanner(transit_space, cfg.transit_clearance,
                                                                             **edges)
    body_space = footprint = None
    if cfg.body:
        lawn = [free] + [clean_polygon(d) for d in drivable]
        if cfg.edges == 'recorded':
            # the edges are where the centre drove: the body may be where it was then
            lawn.append(_driven(lawn, 0.5 * cfg.robot_width, max(cfg.robot_front, cfg.robot_rear)))
        body_space = union(lawn).buffer(cfg.body_tolerance, QUAD_SEGS, join_style=2)
        if keep_out:
            body_space = body_space.difference(union(clean_polygon(k) for k in keep_out))
        footprint = Footprint(body_space, cfg.robot_width, cfg.robot_front, cfg.robot_rear)
    route = _Route(cfg, transit, area_space, start_xy, start_heading, end_xy, footprint)
    route.cell_of = {id(lane): f'cell-{pi}-{ci}' for pi, (_, cells) in enumerate(passes)
                     for ci, cell in enumerate(cells) for lane in cell}
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
        loops_here = not ringed and any(not skip[kind] for g in w['levels'].values() for kind, _ in rings(g))
        loops_end = loops_here and (last or not any(w['cells']) and w['narrow'] is None)
        last_pass = max((i for i, cells in enumerate(w['cells']) if cells), default=None)

        def loops():
            if ringed:
                route.concentric([w['levels'].get(i, empty) for i in range(len(levels))],
                                 [w['middles'].get(i, empty) for i in range(len(middles))], count, last,
                                 loop_spacing, last or not any(w['cells']), not concentric)
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
    if route.counts.get('lanes_shortened'):
        route.warnings.append(f'{route.counts["lanes_shortened"]} lanes along an edge shortened or left out: with '
                              f'blade_offset {cfg.blade_offset} m the centre runs that far off the blade\'s line, '
                              'more perimeter loops leave room for it')
    section_starts = [s.start for i, s in enumerate(segments) if i == 0 or s.section != segments[i - 1].section]
    if route.counts.get('lanes_body'):
        route.warnings.append(f'{route.counts["lanes_body"]} lanes shortened or left out where the body would stick '
                              'out at their ends')
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
        angles=[a for a, _ in passes] or [angle], transit_space=transit_space, body_space=body_space, config=cfg,
        section_starts=section_starts, levels=levels, narrow=narrow, cells=passes, origins=route.origins)


def plan_footprint(plan: CoveragePlan) -> Optional[Footprint]:
    cfg = plan.config
    if cfg is None or not cfg.body or plan.body_space is None:
        return None
    return Footprint(plan.body_space, cfg.robot_width, cfg.robot_front, cfg.robot_rear)


def body_out(poses: List[Pose2D], segments: List[SegmentInfo], section_starts: List[int],
             footprint: Footprint) -> List[Dict[str, Any]]:
    """The poses where the body sticks out: [{x, y, yaw, kind}], kind 'spin' (turning on the spot there), 'turn',
    'lane_end', 'loop' or 'transit' (what the pose belongs to)."""
    kind_at: Dict[int, str] = {}
    for seg in segments:
        kind = {TURN: 'turn', TRANSIT: 'transit', PERIMETER: 'loop'}.get(seg.kind, 'loop' if seg.maneuver == 'ring'
                                                                         else 'lane_end')
        for i in range(seg.start, seg.end + 1):
            kind_at[i] = kind
    out = []
    for a, b in zip(section_starts, list(section_starts[1:]) + [len(poses)]):
        part = poses[a:b]
        for i in footprint.bad_poses(part):
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


def swept_area(plan: CoveragePlan, mower_width: float):
    """What the blade covers along the whole path (a disk of mower_width), for checks and plots."""
    ends = list(plan.section_starts[1:]) + [len(plan.poses)]
    lines = [LineString(blade_points(plan, a, b - 1)) for a, b in zip(plan.section_starts, ends) if b - a >= 2]
    if not lines:
        return Polygon()
    return union(line.buffer(0.5 * mower_width, 4) for line in lines)
