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
"""

import bisect
import collections
import dataclasses
import hashlib
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from shapely import wkb
from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.ops import clip_by_rect, nearest_points
from shapely.prepared import prep

from .footprint import SPIN, TURN_STEP, Footprint
from .geometry import QUAD_SEGS, clean_polygon, inset, polygonal, polygons, rings, subtract, union
from .maneuvers import (CORNER_ANGLE, Fits, Maneuver, closest_on_polyline, connect, drive_polyline, dubins, join,
                        k_point, omega, path_length, plan_turn, same_position, sample_arc, sample_straight, turn_radii)
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
# 'fixed': overlap gives the lane spacing, 'auto': the planner picks it (see plan_coverage)
LANE_SPACING_MODES = ('fixed', 'auto')
# lane spacings lane_spacing_mode 'auto' tries, from lane_spacing_max down to lane_spacing_min
SPACING_CANDIDATES = 5
PERIMETER_DIRECTIONS = ('auto', 'ccw', 'cw')
TURN_TYPES = ('u_turn', 'bulb', 'detour', 'pivot')
# 'recorded': the edges are where the mower's centre drove when they were recorded, 'hard': a wall
EDGES = ('recorded', 'hard')
# headland_turns: the drives along the headland 'rounded' where an arc fits at their corners, or 'sharp' (on the spot)
HEADLAND_CORNERS = ('rounded', 'sharp')
# recorded outlines have a point every few cm, for finding their longest edge they're simplified this much
ANGLE_SIMPLIFY_TOLERANCE = 0.10
# rounding a loop's corner may cut it by this share of the mower width at most, 90 degree corners keep the
# full perimeter_corner_radius, sharper ones get less
CORNER_CUT = 0.3
# lanes this close to the edge of their region count as on it, keeps a lane along a straight edge
ON_EDGE = 1e-4
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
# rad, smooth_spins replaces turns on the spot by more than this, with an omega of these shares of turn_radius
SPIN_LIMIT = math.radians(60.0)
SMOOTH_RADII = (1.0, 0.75, 0.5, 0.35)
# m, with the body checked: where the path still turns on the spot a pose this much before the point, facing the way
# it came (where the way in is longer than twice that). OpenMower's FTC turns over the way from the pose before to the
# next: from 10 cm before it it turns some 5 cm early and drifts, from this close about at the point (a simulation of
# it, the mower lagging 0.1 s)
TURN_AHEAD = 0.025
# m2, lane_spacing_mode 'auto': a plan leaves nothing unmowed with no piece this big and not LEFT_TOTAL together of
# pieces from LEFT_TUFT on (tufts at corners), like the report
LEFT_TUFT = 0.01
LEFT_PIECE = 0.05
LEFT_TOTAL = 0.1
# m2, a narrower spacing has to leave this much less unmowed to be taken
LEFT_GAIN = 0.05
# m, a blade further off the mower's centre is a mistake
MAX_BLADE_OFFSET = 1.0
# m, lane ends are pulled back to this when the body doesn't fit at them
BODY_STEP = 0.01
# m, where the only turn between two lanes is on the spot (or a drive), they end this much earlier at a time for a
# rounded one (_Route.round_earlier), m2 of what they leave its blade may not mow
ROUND_STEP = 0.05
EARLIER_LEFT = 1e-3
# m of path: where the body still sticks out, the path from up to FIT_REACH before to FIT_REACH after it may go another
# way (_fit_body), leaving it and getting back on FIT_STEP apart at least, FIT_PLACE ways tried at most there. The
# further only where none closer fits
FIT_REACH = (3.0, 8.0)
FIT_STEP = (0.2, 0.5)
FIT_PLACE = (30, 60)
# m from where the headings meet towards the closest point it can turn round at whatever way it faces: where
# _fit_body's ways may turn on the spot, the first it can turn at (the last: that point)
FIT_VIA = (0.0, 0.02, 0.04, 0.06, 0.1, 0.15, 0.25, 0.5, 1.0)
# the further ways get off the path and back on it easing off and onto its line: t m along it FIT_EASE_OFF m *
# (e^(t / l) - 1) off it at most, l FIT_EASE times how far the body reaches behind the centre getting off, ahead of it
# getting back on (the rear, the front swinging out FIT_EASE_OFF m at most)
FIT_EASE = 1.5
FIT_EASE_OFF = 0.01
# what a metre driven counts against a metre of loops and lanes left out, picking one of _fit_body's ways
FIT_DRIVE = 0.25
# shares of turn_radius: the arcs of Dubins' ways _fit_body tries (arc, straight, arc or three arcs, turning half round
# at most all together, the shortest FIT_DUBINS of each), before the ways turning on the spot at a corner
FIT_ARCS = (1.0, 0.75, 0.5)
FIT_DUBINS = 2
# mop_up: pieces left unmowed of at least MOP_MIN m2, the biggest MOP_MAX, a straight pass of MOP_PASS m over each with
# the blade's middle on it (one of MOP_HEADINGS directions round it), where that mows MOP_GAIN of MOP_MIN at least
MOP_MIN = 0.02
MOP_MAX = 20
MOP_PASS = 0.4
MOP_HEADINGS = 16
MOP_GAIN = 0.5
# ways _fit_body tries at most for a plan, past that the path jumps over a place (a count, not a time: the same plan
# every time)
FIT_TRIES = 12000
# m, where nothing fits that soon after a section begins it begins later. FIT_LATER: where nothing else does either
# (turning round, driving on), getting to a section's start is up to whoever drives the path anyway
FIT_START = 1.0
FIT_LATER = 8.0
# where none of those fits the path turns round before the place, at the first point of FIT_TURNS a way fits through:
# the poses there and round it (FIT_STEP[0] apart, FIT_AROUND m before and after at most, the closest first) moved
# FIT_IN m to either side, as little as the body fits facing that pose's way, turned half and all the way the path turns
# over the place. In from the latest pose before it fits from, out to the earliest after, straight or eased. Where none
# does, through turn_room like the further ways (simplified ROOM_SIMPLIFY m: there it turns whatever way it faces).
# Where driving on doesn't fit either (and the path wouldn't end before), moved further to the side than that (a smaller
# body fits closer to the edge than it turns round). FIT_ROUND ways and halves of them tried at most for each (for those
# further ones together)
FIT_AROUND = 1.5
FIT_IN = (0.03, 0.06, 0.1, 0.2, 0.3, 0.45)
FIT_TURNS = 6
FIT_ROUND = 600
ROOM_SIMPLIFY = 0.005
# headland_turns: a way of _fit_body that turns in the lanes' field counts as this many m of loops and lanes left out
FIT_FIELD = 2.0
# m, _fit_body finds where the body sticks out the most at a place looking along its edge this far apart first
# (Footprint.out_by), closer only where that may be the most
PLACE_STEP = 0.02
# _fit_body keeps where it got to every FIT_KEEP poses: a plan whose path starts the same (the loops, at another lane
# angle) goes on from the last of those up to which it reads the same poses
FIT_KEEP = 1000
# headland_turns: a drive along the headland more than HEADLAND_SLACK m and HEADLAND_DETOUR times as long as the
# shortest way crosses the lanes' field where that's shorter, along a lane's line (the CROSSINGS closest tried)
HEADLAND_DETOUR = 2.0
HEADLAND_SLACK = 2.0
CROSSINGS = 3
# m, with headland_corners 'rounded' an arc rounding a drive's corner out of the lanes' field into it (where none fits
# out of it) reaches this far into it at most, else the corner stays
FIELD_ARC = 0.12
# poses of a segment _Route.mowed() looks at together: it skips those whose box is far from where it looks
MOWED_CHUNK = 32
# swept_area: points per piece of the way at least
SWEPT_PIECE = 400
# kept for planning the same area again (another angle or setting, lane_spacing_mode 'auto'), the last KEPT of each for
# all threads: where _fit_body got to (poses, their sections, whether the blade mows on the way to each, [(the furthest
# pose read, where it got to)]) and the body spaces (_body_space, as WKB), by a hash of what they depend on, the last
# used last. The body space a thread last worked with (.last: what of, the space, its Footprint)
KEPT = 4
_FITTED, _BODY_SPACES = collections.OrderedDict(), collections.OrderedDict()
_KEPT_LOCK = threading.Lock()
_BODY_SPACE = threading.local()


class PlanningError(ValueError):
    """The request can't be planned (bad parameters, no area left to mow)."""


@dataclass
class PlannerConfig:
    mower_width: float = 0.22  # m, cutting width
    overlap: float = 0.15  # 0..<1, share of the cutting width neighbouring lanes overlap
    # 'fixed': the lane spacing follows from overlap. 'auto': the planner tries up to SPACING_CANDIDATES spacings from
    # lane_spacing_max down to lane_spacing_min (m, None: 85 % and 50 % of mower_width) and keeps the widest that
    # leaves nothing unmowed (no piece of LEFT_PIECE, not LEFT_TOTAL together): the fewest lanes and turns. Where one
    # leaves something, the next narrower only if it leaves LEFT_GAIN less (the angle stays as it is)
    lane_spacing_mode: str = 'fixed'
    lane_spacing_min: Optional[float] = None
    lane_spacing_max: Optional[float] = None
    # loops along the outline and around every obstacle, -1: as many as the lanes' turns need (see passes)
    perimeter_passes: int = 2
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
    # rad, the lanes turn this much further every angle_increment_every finished mows (angle_steps of those so far, the
    # ROS 1 node counts them), on top of angle_offset and within angle_min..angle_max (bouncing back at the ends).
    # 0: they don't
    angle_increment: float = 0.0
    angle_increment_every: int = 1
    angle_steps: int = 0
    # m, the first perimeter loop runs this far inside the outline. None: mower_width / 2, the blade's edge
    # on the outline. OpenMower records outlines driving the mower's centre along the edge and uses 0,
    # negative goes outside the outline (OpenMower's outline_offset allows that too)
    perimeter_offset: Optional[float] = None
    # m, radius of the turns between lanes, and no arc of a turn, a drive or a loop out is tighter (min_turn_radius):
    # where none fits the drives turn on the spot instead (with the body checked a drive's corner gets a tighter arc
    # where that fits, see _round_corners), a turn between lanes is rounded as wide as fits (at a slant where the lanes
    # end at one, the lanes ending a little earlier for it where that's what it takes, else a loop into the next lane,
    # see _Route.round_earlier), on the spot only where nothing does, a drive into a lane ending on a way into it where
    # one fits (the lane starting later for it where that leaves nothing, see _Route.into_lane)
    turn_radius: float = 0.25
    allow_reverse: bool = False  # K-turns, which back up a little (fixed, see INTERNAL: OpenMower drives forwards)
    # the kinds of turns allowed, preferred first ('u_turn', 'bulb', 'detour', 'pivot'). Larger
    # radii come first, the order counts between turns of the same radius
    turn_types: Tuple[str, ...] = TURN_TYPES
    lane_order: str = 'snake'  # 'snake': neighbour after neighbour, 'skip': lanes far enough apart for U-turns
    # mowing areas of the map lying in the one planned (an area of its own with another angle or pattern): left out
    # and mowed round like an obstacle, they get their own plan. For the map's areas (mapfile.area_job), not here
    nested_areas: bool = True
    # where the lanes would lie in a part narrower than a U-turn (2 * turn_radius: a path, a strip between beds, and
    # what's left between such parts too small for more than a few lanes): 'lanes' anyway, 'loops': the loops go on
    # in there instead until it's mowed, the pieces they come apart into driven on the way round (in from the loop
    # next to them and back out), so no trips along a path from piece to piece. With fill_pattern concentric 'loops'
    # drives all the loops that way, a part of the area in one go
    narrow_parts: str = 'lanes'
    # turning on the spot where the path would drive a loop (for lawns that take it). Off: where the path would turn on
    # the spot by more than SPIN_LIMIT a loop out (an omega) in its place where it fits (_smooth_spins). On: no loop
    # out, no bulb between lanes (on the spot where no U-turn fits, no loop into the next lane either), a drive
    # arriving on the spot rather than into a lane round a loop or from beside it, or on arcs that cross its own way.
    # The body fits turning like everywhere
    turn_on_spot: bool = False
    # the lanes' field kept clean (where they lie but for what the loops mow and 2 turn_radius at their ends, where the
    # turns go): in there the path only runs straight along the lanes. Drives go along the headland round it (out of the
    # field straight on from a lane's end, into it straight along the next lane), across it only straight (along a
    # lane's line where that's not much longer) where the headland's way is more than HEADLAND_DETOUR times as long or
    # there's none. U-turns between lanes far apart, detours, loops out and ways round the body keep out of it too.
    # Where nothing does the way it takes without, stats['headland_turns'] and a warning saying where
    headland_turns: bool = False
    # with headland_turns and the body checked, the corners of the drives along the headland: 'rounded' (an arc where
    # one fits like at every corner of a drive, into the lanes' field only where none fits out of it and FIELD_ARC deep
    # at most, stats['headland_turns'] says how many and how deep, see _round_corners) or 'sharp' (turning on the spot
    # there, no arc into the field anywhere)
    headland_corners: str = 'rounded'
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
    # behind, or robot_outline (beats them): its corners [(x, y), ...] (x ahead of the centre, y to the left), 3 at
    # least round the centre, the outline not crossing itself. With all three (or robot_outline) the turns, lane ends,
    # loops and drives keep the body in the space it may take (see edges), the corners of drives are rounded where the
    # body fits on an arc (_round_corners) and the path turns on the spot at the point (TURN_AHEAD). None: not checked
    robot_width: Optional[float] = None
    robot_front: Optional[float] = None
    robot_rear: Optional[float] = None
    robot_outline: Optional[Tuple[XY, ...]] = None
    # 'recorded': the outline, obstacles and areas to keep out of are where the centre drove. 'hard': they're walls,
    # the centre keeps robot_width / 2 off them (the first loop moves further in where perimeter_offset would come
    # closer). With the body checked they're always where the centre drove: the body may be where it was while they
    # were recorded (half its width each side, the front ahead and the rear behind, turning on the spot at every
    # point), their real edges
    edges: str = 'recorded'
    # m, with the body checked a turn, a loop out or a drive may take the centre this much further out than the first
    # loop (a recorded edge is a few cm off where the mower was), the body still keeping the margins
    body_tolerance: float = 0.05
    # m, with the body checked: how far it keeps off the outline's real edge everywhere, obstacle_margin off an
    # obstacle's and an area's to keep out of (one's own margin beats it, see plan_coverage). The loops keep the centre
    # that far off the lines: the first one further in where perimeter_offset would come closer, the ones round an
    # obstacle further out (round both of two too close for the body to get through between, _gaps)
    edge_margin: float = 0.05
    obstacle_margin: float = 0.10
    # m, with the body checked: turning on the spot (the heading jumping by more than footprint.SPIN from one pose to
    # the next) it keeps this much further off the edges than those margins, anywhere on the last footprint.SPIN_BACK
    # of the way in: OpenMower's FTC turns over the way to the point and drifts a few cm doing so. On the turns between
    # the lanes too (U-turns, loops, detours, all of them): there the mower keeps less exactly to its path, its corners
    # swinging further. 0: only at the point, the margins as everywhere
    spin_margin: float = 0.05
    # where the body would still stick out (a turn on the spot, a loop in a tight spot), the path goes another way there
    # that fits, leaving a little of a loop or lane out where it has to (_fit_body, stats['body_fit']). False: the path
    # as planned
    body_fit: bool = True
    # with the body checked, at the path's end a round over what it leaves unmowed where the blade gets to (the
    # biggest pieces, a short pass over each, then back to where the path ended, see _mop_up). Off: the path ends
    mop_up: bool = False
    # m, how far the blade's centre sits to the left of the mower's centre (negative: right). The loops go
    # round with the blade on their outer side, so it mows up to perimeter_offset while the body stays further
    # in, the lanes move so the blade's lines stay a lane spacing apart
    blade_offset: float = 0.0
    # m, how far the blade's centre sits ahead of the mower's centre (the point the path is for: OpenMower's
    # base_link between the drive wheels; negative: behind it). The lanes start and end that much earlier, so the
    # blade mows them where they are
    blade_ahead: float = 0.0

    @property
    def passes(self) -> int:
        """The loops, perimeter_passes -1 worked out: as many as mow the strip along the edges the lanes don't get to.
        A lane ends where its turn (a quarter arc of turn_radius, the body round it) stays in what the edges allow,
        the next one starts there too (the turn goes level with it), and the blade there is blade_ahead further in."""
        if self.perimeter_passes >= 0:
            return int(self.perimeter_passes)
        half = 0.5 * self.mower_width
        if self.body:
            out = max(math.hypot(self.turn_radius + abs(y), x) for x, y in self.body_outline)
            allowed = max(abs(y) for _, y in self.body_outline) - self.edge_margin
            end = max(self.centre_offset, out - allowed)
        else:
            end = self.centre_offset + self.turn_radius
        strip = end + abs(self.blade_ahead) - half  # from the edge in, what the lanes leave
        return max(1, int(math.ceil((strip - half - self.blade_loop_offset) / self.lane_spacing - 1e-9)) + 1)

    @property
    def spacing_range(self) -> Tuple[float, float]:
        lo = 0.5 * self.mower_width if self.lane_spacing_min is None else self.lane_spacing_min
        hi = 0.85 * self.mower_width if self.lane_spacing_max is None else self.lane_spacing_max
        return lo, hi

    @property
    def lane_spacing(self) -> float:
        return self.mower_width * (1.0 - self.overlap)

    @property
    def first_loop_offset(self) -> float:
        return 0.5 * self.mower_width if self.perimeter_offset is None else self.perimeter_offset

    @property
    def blade_loop_offset(self) -> float:
        """m, how far inside the outline (and off the obstacles) the blade's centre runs on the first loop:
        first_loop_offset, further in where the centre keeps edge_margin off the line (the body checked, and
        simplify_tolerance: the simplified line may lie that much further out) or the body (robot_width, hard edges)
        wouldn't fit there."""
        if self.body:
            keep = self.edge_margin + self.simplify_tolerance
        elif self.robot_width is not None and self.edges == 'hard':
            keep = 0.5 * self.robot_width
        else:
            keep = 0.0
        return max(self.first_loop_offset, keep - abs(self.blade_offset)) if keep > 0 else self.first_loop_offset

    @property
    def centre_offset(self) -> float:
        """m, how far the mower's centre keeps off the outline and the obstacles."""
        return self.blade_loop_offset + abs(self.blade_offset)

    @property
    def body(self) -> bool:
        """Whether the body is checked: robot_width, robot_front and robot_rear set, or robot_outline."""
        return self.robot_outline is not None or None not in (self.robot_width, self.robot_front, self.robot_rear)

    @property
    def body_outline(self) -> Tuple[XY, ...]:
        """The body's corners counter-clockwise: robot_outline (a point given twice in a row once), or the rectangle
        of robot_width, robot_front and robot_rear."""
        if self.robot_outline is None:
            half = 0.5 * self.robot_width
            return ((self.robot_front, half), (-self.robot_rear, half), (-self.robot_rear, -half),
                    (self.robot_front, -half))
        pts = [(float(x), float(y)) for x, y in self.robot_outline]
        pts = [p for i, p in enumerate(pts) if p != pts[i - 1]]
        return tuple(pts if Polygon(pts).exterior.is_ccw else pts[::-1])

    @property
    def min_turn_radius(self) -> float:
        """m, no arc tighter than this: the turns' radius (one for all)."""
        return self.turn_radius

    @property
    def allowed_turns(self) -> Tuple[str, ...]:
        return tuple(t for t in self.turn_types if (t != 'k_turn' or self.allow_reverse) and
                     (t != 'bulb' or not self.turn_on_spot))

    @classmethod
    def names(cls) -> Tuple[str, ...]:
        """The settings, not the INTERNAL ones."""
        return tuple(f.name for f in dataclasses.fields(cls) if f.name not in INTERNAL)

    def with_overrides(self, overrides: Mapping[str, Any]) -> 'PlannerConfig':
        """A copy with some settings changed, e.g. per area. Unknown names are an error, ints are fine for
        floats, None keeps the setting."""
        # (the ones that were settings once, from a settings file or a map of then)
        overrides = {n: v for n, v in overrides.items() if n not in INTERNAL and n not in RETIRED}
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
                value = tuple(t for t in value if t not in RETIRED)
            elif name == 'robot_outline' and isinstance(value, (list, tuple)):
                value = tuple(tuple(p) if isinstance(p, (list, tuple)) else p for p in value)
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
        finite('angle_increment')
        for name, lo in (('angle_increment_every', 1), ('angle_steps', 0)):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < lo:
                raise PlanningError(f'{name} must be a whole number >= {lo}, got {v!r}')
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
        if self.robot_outline is not None:
            try:
                pts = [(float(x), float(y)) for x, y in self.robot_outline
                       if not isinstance(x, (bool, str)) and not isinstance(y, (bool, str))]
                ok = len(pts) == len(self.robot_outline) >= 3 and all(math.isfinite(c) for p in pts for c in p)
            except (TypeError, ValueError, OverflowError):
                ok = False
            shape = Polygon(pts) if ok else None
            if shape is None or not shape.is_valid or shape.area <= 0.0 or not shape.covers(Point(0.0, 0.0)):
                raise PlanningError('robot_outline must be [[x, y], ...] (m, x ahead of base_link, y to the left), 3 '
                                    f'points at least round base_link, the outline not crossing itself, got '
                                    f'{self.robot_outline!r}')
        finite('body_tolerance', 0.0, MAX_BLADE_OFFSET)
        finite('edge_margin', 0.0, MAX_BLADE_OFFSET)
        finite('obstacle_margin', 0.0, MAX_BLADE_OFFSET)
        finite('spin_margin', 0.0, MAX_BLADE_OFFSET)
        for name in ('lane_spacing_min', 'lane_spacing_max'):
            if getattr(self, name) is not None:
                finite(name, 0.0, self.mower_width, lo_open=True)
        lo, hi = self.spacing_range
        if lo > hi:
            raise PlanningError(f'lane_spacing_min must be <= lane_spacing_max, got {lo} > {hi}')
        finite('blade_offset', -MAX_BLADE_OFFSET, MAX_BLADE_OFFSET)
        finite('blade_ahead', -MAX_BLADE_OFFSET, MAX_BLADE_OFFSET)
        for name, lo in (('perimeter_passes', -1), ('lane_overlap_passes', 0)):
            v = getattr(self, name)
            whole = isinstance(v, int) or isinstance(v, float) and math.isfinite(v) and v == int(v)
            if isinstance(v, bool) or not whole or not lo <= v <= MAX_PASSES:
                raise PlanningError(f'{name} must be a whole number {lo}..{MAX_PASSES}, got {v!r}')
        if self.route_order not in ROUTE_ORDERS:
            raise PlanningError(f'route_order must be one of {", ".join(ROUTE_ORDERS)}, got {self.route_order!r}')
        if self.lane_order not in LANE_ORDERS:
            raise PlanningError(f'lane_order must be one of {", ".join(LANE_ORDERS)}, got {self.lane_order!r}')
        for name, allowed in (('angle_strategy', ANGLE_STRATEGIES), ('fill_pattern', FILL_PATTERNS),
                              ('perimeter_order', PERIMETER_ORDERS), ('perimeter_direction', PERIMETER_DIRECTIONS),
                              ('edges', EDGES), ('narrow_parts', NARROW_PARTS),
                              ('lane_spacing_mode', LANE_SPACING_MODES), ('headland_corners', HEADLAND_CORNERS)):
            if getattr(self, name) not in allowed:
                raise PlanningError(f'{name} must be one of {", ".join(allowed)}, got {getattr(self, name)!r}')
        if not isinstance(self.turn_types, (tuple, list)) or not self.turn_types or \
                any(t not in TURN_TYPES + ('k_turn',) for t in self.turn_types):
            raise PlanningError(f'turn_types must be some of {", ".join(TURN_TYPES)}, got {self.turn_types!r}')
        for name in ('allow_reverse', 'skip_outline_loops', 'skip_obstacle_loops', 'skip_lanes', 'nested_areas',
                     'turn_on_spot', 'headland_turns', 'body_fit', 'mop_up'):
            if not isinstance(getattr(self, name), bool):
                raise PlanningError(f'{name} must be true or false, got {getattr(self, name)!r}')


# fixed, no settings (with_overrides leaves them out): OpenMower's FTC drives forwards (no K-turns), the rest is how
# the planner works inside
INTERNAL = ('allow_reverse', 'waypoint_spacing', 'simplify_tolerance', 'min_lane_length', 'transit_clearance',
            'body_tolerance', 'perimeter_corner_radius')
# settings (and a turn type) of before, with_overrides leaves them out: min_turn_radius is turn_radius, smooth_spins
# not turn_on_spot
RETIRED = ('min_turn_radius', 'smooth_spins', 'k_turn')
# settings where None means something (worked out, mower_width / 2), and the ones that take a float
_OPTIONAL = ('angle', 'angle_min', 'angle_max', 'perimeter_offset', 'robot_width', 'robot_front', 'robot_rear',
             'robot_outline', 'lane_spacing_min', 'lane_spacing_max')
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
    free_space: object = None  # area minus obstacles (bigger by what their margins add): what gets mowed
    # where the mower's centre may go: free_space shrunk by centre_offset (with the body checked grown by
    # body_tolerance)
    robot_space: object = None
    lane_region: object = None  # where the lane centres lie
    remaining: object = None  # free_space minus the strips the perimeter loops mow
    angles: List[float] = dataclasses.field(default_factory=list)  # of every set of lanes (crosshatch: 2)
    transit_space: object = None  # robot_space (with the body checked not grown) and the drivable areas
    # where the mower's body may be (robot_width, robot_front, robot_rear set): the real edges, the margins off them
    body_space: object = None
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
    a = angle_in_range(a + cfg.angle_offset + cfg.angle_steps * cfg.angle_increment, cfg.angle_min, cfg.angle_max)
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
        return [_inside(xs) for xs in crossings]

    def at(self, ys: Sequence[float]) -> List[List[Tuple[float, float]]]:
        """Inside intervals on the lines y of ys (in order)."""
        crossings: List[List[float]] = [[] for _ in ys]
        for lo, hi, ax, ay, dx, dy in self.edges:
            for j in range(bisect.bisect_left(ys, lo), bisect.bisect_left(ys, hi)):
                crossings[j].append(ax + (ys[j] - ay) * dx / dy)
        return [_inside(xs) for xs in crossings]


def _inside(xs: List[float]) -> List[Tuple[float, float]]:
    """The inside intervals between the crossings of a line (even-odd)."""
    xs.sort()
    return [(xs[k], xs[k + 1]) for k in range(0, len(xs) - 1, 2) if xs[k + 1] > xs[k]]


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
    usable = lanes.hi - lanes.lo - 2.0 * ON_EDGE
    if usable <= 0.0:
        count, step, y0 = 1, spacing, 0.5 * (lanes.lo + lanes.hi)
    else:
        count = int(math.ceil(usable / spacing - 1e-9)) + 1
        if count > MAX_ROWS:
            raise PlanningError(f'{count} lanes, more than {MAX_ROWS}: the area is too big for this lane spacing')
        step = usable / (count - 1)
        y0 = lanes.lo + ON_EDGE
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
        out.append([Lane(j, lo, hi, y) for lo, hi in intervals if hi - lo >= min_length - 1e-9])
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
        # (a turn between lanes: spin_margin all along it too)
        self.turn_body = (lambda poses: footprint.poses_fit(poses, True)) if footprint is not None else None
        # a turn between lanes rounded as wide as fits where it would turn on the spot (round_earlier): where an arc of
        # one may be (only pivots allowed: on the spot)
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
        # what the segments added now mow or lead to, and where every loop and lane comes from
        self.source = ''
        self.origins: Dict[str, Dict[str, Any]] = {}
        self.cell_of: Dict[int, str] = {}  # lane (by id) -> its cell, 'cell-<set>-<index>' in CoveragePlan.cells
        self._chunks: Dict[int, Tuple[PathSegment, List[Tuple[float, float, float, float]]]] = {}  # see chunks()
        # headland_turns: the lanes' field (and prepared), the transit space but the field, the lanes' direction and the
        # lines they lie on (lane frame y) of every set, the field's pieces on those (see rows())
        self.field = self._field = self.headland = None
        self.angle = 0.0
        self.lane_ys: Dict[float, List[float]] = {}
        self._rows: Dict[float, List[Tuple[float, List[Tuple[float, float]]]]] = {}
        # headland_corners 'sharp': the points (x, y) of the drives along the headland, 'rounded': of the arcs rounding
        # a corner out of the lanes' field into it (_round_corners)
        self.sharp: set = set()
        self.rounded: set = set()

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
        the spot where it leaves and arrives where the arcs there would make it cross its own way. None if there's no
        way."""
        if space is None and self.field is not None:
            way = self.headland_way(target, heading, head)
            if way is not None:
                if self.cfg.headland_corners == 'sharp':
                    self.sharp.update((p.x, p.y) for p in way)
                return way
        space = space or self.transit
        pos, facing = start or (self.pos, self.heading)
        r = self.cfg.turn_radius

        def drive(b: XY, h: Optional[float]) -> Optional[List[Pose2D]]:
            return connect(pos, facing, b, h, r, self.step, space.is_free, space.careful_path, self.body,
                           self.cfg.min_turn_radius)

        best = drive(target, heading)
        if best is not None and self.cfg.turn_on_spot:
            # (on the spot where it leaves and arrives where the arcs there make it cross its own way)
            if len(best) > 3 and not LineString([(p.x, p.y) for p in best]).is_simple:
                best = connect(pos, None, target, None, r, self.step, space.is_free, space.careful_path, self.body,
                               self.cfg.min_turn_radius) or best
            return best
        if best is None or heading is None or r <= 1e-3:
            return best
        best_spins = _spins(best, facing, heading)
        if not best_spins:
            return best
        nx, ny = -math.sin(heading), math.cos(heading)
        back = normalize_angle(heading + math.pi)
        for d in (2.0 * r, r):
            for side in (1.0, -1.0):
                x = (target[0] + side * d * nx, target[1] + side * d * ny)
                if not space.is_free([x]):
                    continue
                turn = plan_turn(Pose2D(x[0], x[1], back), target, r, False, self.step, space.is_free, None,
                                 tuple(t for t in self.cfg.allowed_turns if t in ('u_turn', 'bulb')), self.body,
                                 self.cfg.min_turn_radius)
                way = drive(x, back) if turn is not None else None
                if way is None:
                    continue
                poses = join(way, turn.pieces[0].poses)
                spins = _spins(poses, facing, heading)
                if spins < best_spins or spins == best_spins and path_length(poses) < path_length(best):
                    best, best_spins = poses, spins
        if head is not None and self.cfg.min_turn_radius > 0 and angle_diff(best[-1].yaw, heading) > SPIN_LIMIT:
            return self.into_lane(best, target, heading, head, space) or best
        return best

    def into_lane(self, drive: List[Pose2D], a: XY, yaw: float, head: XY,
                  space: TransitPlanner) -> Optional[List[Pose2D]]:
        """In place of the drive's end turning on the spot into the lane from a (facing yaw, its straight piece ending
        at head): Dubins' shortest way into it that fits in space, going round once at most on top of what it has to,
        from a pose of the drive's last 4 turn_radius (one every turn_radius back, the latest first, its start too),
        where none does into the lane ROUND_STEP further on at a time, up to 2 turn_radius, the first whose way or
        what's mowed already (the loops) mows what it leaves of the lane (but EARLIER_LEFT). None where none fits."""
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
                most = angle_diff(drive[k].yaw, yaw) + 2.0 * math.pi + 1e-6
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

    def turn_space(self, a: XY) -> Fits:
        """What a turn from where the mower is into a lane starting at a keeps in: the mowing area, with headland_turns
        out of the lanes' field too where the lanes are more than 4 turn_radius apart (a long U-turn)."""
        (x, y), h = self.pos, self.heading
        if self.field is None or abs((a[1] - y) * math.cos(h) - (a[0] - x) * math.sin(h)) <= 4.0 * self.cfg.turn_radius:
            return self.area.is_free

        def fits(pts: Sequence[XY]) -> bool:
            return self.area.is_free(pts) and self.clear(pts)

        return fits

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
        # (right turns too: the arc cuts their corner on the side mowed already or keeps off the edge where it
        # doesn't fit)
        poses = drive_polyline(loop, self.step, radius, self.area.is_free, 0,
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

    # lanes

    def cells(self, cells: List[List[Lane]], angle: float, finish: bool = False) -> None:
        """Mows the cells, the closest one not mowed yet next. finish: the cell that can end closest to the
        end point goes last."""
        c, s = math.cos(angle), math.sin(angle)
        self.angle = angle

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

    def _turn_into(self, a: XY, yaw: float, head: XY) -> Optional[XY]:
        """The next cell's first lane right next to where the mower is and the other way round: that's a turn
        between lanes, not a drive, round_earlier where it would turn on the spot. Where the lane starts then, None if
        no turn got there."""
        if not self.started or self.heading is None or angle_diff(yaw, self.heading + math.pi) > 1e-3 or \
                math.hypot(a[0] - self.pos[0], a[1] - self.pos[1]) > 4.0 * self.cfg.turn_radius:
            return None
        turn = plan_turn(Pose2D(self.pos[0], self.pos[1], self.heading), a, self.cfg.turn_radius,
                         self.cfg.allow_reverse, self.step, self.area.is_free, self.detour_path,
                         self.cfg.allowed_turns, self.turn_body, self.cfg.min_turn_radius if self.rounds else 0.0)
        if (turn is None or turn.name == 'pivot') and self.rounds:
            earlier = self.round_earlier(a, yaw, math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length)
            if earlier is not None:
                turn, a = earlier
                self._count('lanes_earlier')
        if turn is None:
            return None
        for piece in turn.pieces:
            self.add(TURN, piece.poses, piece.reverse, turn.name)
        self._count('turns_' + turn.name)
        return a

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
            pieces = lane.pieces()[::direction]  # (of a lane bending round obstacles: where its first piece ends)
            head = to_map((pieces[0][1] if direction > 0 else pieces[0][0]) - back, y) if lane.bends else b
            if self.footprint is not None:
                # the side the turns come from and go to (left of the lanes going +x is +y)
                side = [0.0 if o is None else direction * math.copysign(1.0, o.y - lane.y)
                        for o in (lanes[i - 1] if driven else None, lanes[i + 1] if i + 1 < len(lanes) else None)]
                if lane.bends:
                    # the straight pieces at its ends
                    last = to_map((pieces[-1][0] if direction > 0 else pieces[-1][1]) - back, y)
                    ends = self.body_ends(a, head, yaw, side[0], 0.0), self.body_ends(last, b, yaw, 0.0, side[1])
                    head = ends[0][1] if ends[0] is not None else head
                    ends = None if None in ends else (ends[0][0], ends[1][1])
                else:
                    ends = self.body_ends(a, b, yaw, side[0], side[1])
                if ends is None:
                    continue
                a, b = ends
                head = head if lane.bends else b  # (where the body lets it end)
            name = f'lane-{len(self.lanes)}'
            self.source = name
            self.origins[name] = {'parent': self.cell_of.get(id(lane), ''),
                                  'kind': 'bent' if lane.bends else 'straight', 'row': lane.row}
            start = self._turn_into(a, yaw, head) if driven == 0 else None
            if start is not None:
                a = start
            elif driven == 0:
                self.go_to(a, yaw, head)
                a = a if same_position(self.pos, a) else self.pos  # (into_lane may start it further on)
            else:
                exit_pose = Pose2D(self.pos[0], self.pos[1], self.heading)

                def turn_to(entry: XY, types) -> Optional[Maneuver]:
                    return plan_turn(exit_pose, entry, self.cfg.turn_radius, self.cfg.allow_reverse, self.step,
                                     self.turn_space(entry), self.detour_path, types, self.turn_body,
                                     self.cfg.min_turn_radius if self.rounds else 0.0)

                # the turn goes level with the lane end further out, where the body may not fit (body_ends only
                # made room for a turn level with each end): this lane starts level with the last one's end then,
                # before a detour or a pivot
                ux, uy = math.cos(yaw), math.sin(yaw)
                level = (self.pos[0] - a[0]) * ux + (self.pos[1] - a[1]) * uy
                rest = math.hypot(head[0] - a[0], head[1] - a[1]) - self.cfg.min_lane_length
                # (not by more than a turn reaches: what it leaves of the lane would stay unmowed)
                most = min(rest, self.footprint.reach + self.cfg.turn_radius) if self.footprint is not None else 0.0
                turn, types = None, self.cfg.allowed_turns
                if self.footprint is not None and 1e-6 < level < most:
                    gentle = tuple(t for t in self.cfg.allowed_turns if t not in ('detour', 'pivot'))
                    turn = turn_to(a, gentle)
                    if turn is None:
                        types = tuple(t for t in types if t not in gentle)  # (those don't fit at a)
                        start = (a[0] + level * ux, a[1] + level * uy)
                        turn = turn_to(start, gentle)
                        if turn is not None:
                            a = start
                            self._count('lanes_body')
                if turn is None:
                    turn = turn_to(a, types)
                if (turn is None or turn.name == 'pivot') and self.rounds:
                    earlier = self.round_earlier(a, yaw, rest)
                    if earlier is not None:
                        turn, a = earlier
                        self._count('lanes_earlier')
                if turn is not None:
                    for piece in turn.pieces:
                        self.add(TURN, piece.poses, piece.reverse, turn.name)
                    self._count('turns_' + turn.name)
                else:
                    poses = self.drive_to(a, yaw, head)
                    if poses is None:
                        return lanes[i:]
                    a = a if same_position(poses[-1], a) else (poses[-1].x, poses[-1].y)
                    self.add(TRANSIT, poses, maneuver='turn_fallback')
                    self._count('turns_fallback')
            if lane.bends:
                self.bent_lane(lane, direction, a, b, to_map)
            else:
                self.add(LANE, sample_straight(a[0], a[1], b[0], b[1], yaw, self.step))
            self.lanes.append((a, b))
            direction = -direction
            driven += 1
        return None

    def round_earlier(self, a: XY, yaw: float, rest: float) -> Optional[Tuple[Maneuver, XY]]:
        """In place of turning on the spot (or a drive) from the lane just mowed into the next (from a, facing yaw, rest
        m of it to spare): a rounded turn level with the end of the shorter of the two, the longer one ending (or
        starting) earlier, then both, ROUND_STEP at a time up to turn_radius more: a U-turn (at a slant where they end
        at one), a bulb, a detour between lanes far apart. The first that fits, where the turn or what's mowed already
        (the loops next to them) mows what they leave (but EARLIER_LEFT): the lane just mowed is shortened then. Else a
        loop into the next lane (Dubins' shortest way that fits). None where none fits."""
        last = self.segments[-1] if self.segments else None
        if last is None or last.kind != LANE or last.maneuver or len(last.poses) < 2:
            return None
        first, end = last.poses[0], last.poses[-1]
        h = end.yaw
        if any(angle_diff(p.yaw, h) > 1e-6 for p in last.poses):
            return None  # a bent lane
        length = math.hypot(end.x - first.x, end.y - first.y)
        r, least = self.cfg.turn_radius, self.cfg.min_turn_radius
        hx, hy, ux, uy = math.cos(h), math.sin(h), math.cos(yaw), math.sin(yaw)
        dx = (a[0] - end.x) * hx + (a[1] - end.y) * hy  # how much further on the next lane starts
        far = abs((a[1] - end.y) * hx - (a[0] - end.x) * hy) > 4.0 * r
        types = tuple(t for t in self.cfg.allowed_turns if t != 'pivot' and (far or t != 'detour')) + ('pivot',)
        most_e = min(max(0.0, -dx) + r, length - self.cfg.min_lane_length)
        most_n = min(max(0.0, dx) + r, rest)
        fits = self.turn_space(a)

        def shortest(exit_pose: Pose2D, entry: XY, loops: bool) -> Optional[Maneuver]:
            # Dubins' shortest way that fits, the widest first
            for rr in turn_radii(r, 4.0 * r, least):
                for m in dubins(exit_pose, Pose2D(entry[0], entry[1], yaw), rr, self.step, loops):
                    poses = m.pieces[0].poses
                    if fits([(p.x, p.y) for p in poses]) and (self.body is None or self.turn_body(poses)):
                        return m
            return None

        mowed = []

        def unmowed(region):
            # what of region isn't mowed already (but by the lane just mowed): the path near where the two lanes end
            # and start as far as they may get shorter, worked out once
            if not mowed:
                mowed.append(self.mowed([(end.x - most_e * hx, end.y - most_e * hy, end.x, end.y),
                                         (a[0], a[1], a[0] + most_n * ux, a[1] + most_n * uy)], self.segments[:-1]))
            return subtract(region, mowed)

        k = 0
        while True:
            level = max(0.0, dx) - k * ROUND_STEP  # where the turn goes, along the lane just mowed from its end
            de, dn = max(0.0, -level), max(0.0, dx - level)  # how much earlier it ends and the next one starts
            if de > most_e + 1e-9 or dn > most_n + 1e-9:
                break
            exit_pose = Pose2D(end.x - de * hx, end.y - de * hy, h)
            entry = (a[0] + dn * ux, a[1] + dn * uy)
            turn = shortest(exit_pose, entry, False) if 'u_turn' in self.cfg.allowed_turns else None
            if turn is None and k:
                turn = plan_turn(exit_pose, entry, r, self.cfg.allow_reverse, self.step, fits, self.detour_path, types,
                                 self.turn_body, least, False)
            k += 1
            if turn is None:
                continue
            left = [self.blade([exit_pose, end])] if de > 1e-9 else []
            left += [self.blade([Pose2D(a[0], a[1], yaw), Pose2D(entry[0], entry[1], yaw)])] if dn > 1e-9 else []
            if left:
                there = subtract(union(left), [self.blade([p for piece in turn.pieces for p in piece.poses]),
                                               self.blade([first, exit_pose])])
                if there.area > EARLIER_LEFT and unmowed(there).area > EARLIER_LEFT:
                    break  # (further in it leaves more)
            if de > 1e-9:
                last.poses[:] = [p for p in last.poses
                                 if (p.x - first.x) * hx + (p.y - first.y) * hy < length - de - 1e-9] + [exit_pose]
                self._chunks.pop(id(last), None)
                self.pos, self.heading = (exit_pose.x, exit_pose.y), h
                self.lanes[-1] = (self.lanes[-1][0], (exit_pose.x, exit_pose.y))
            return turn, entry
        turn = shortest(end, a, True) if 'bulb' in self.cfg.allowed_turns else None
        return None if turn is None else (turn, a)

    def blade(self, *runs: Sequence[Pose2D]):
        """Where the blade mows driving poses (each run of them)."""
        half, ahead, off = 0.5 * self.cfg.mower_width, self.cfg.blade_ahead, self.cfg.blade_offset
        lines = [[(p.x + ahead * math.cos(p.yaw) - off * math.sin(p.yaw),
                   p.y + ahead * math.sin(p.yaw) + off * math.cos(p.yaw)) for p in poses] for poses in runs]
        return (LineString(lines[0]) if len(lines) == 1 else MultiLineString(lines)).buffer(half, 4) if lines else \
            Polygon()

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
        """The box (x0, y0, x1, y1) round every MOWED_CHUNK poses of seg, worked out once (round_earlier forgets it
        where it shortens a lane)."""
        known = self._chunks.get(id(seg))
        if known is None:
            boxes = []
            for k in range(0, len(seg.poses), MOWED_CHUNK):
                xs, ys = [p.x for p in seg.poses[k:k + MOWED_CHUNK]], [p.y for p in seg.poses[k:k + MOWED_CHUNK]]
                boxes.append((min(xs), min(ys), max(xs), max(ys)))
            known = self._chunks[id(seg)] = (seg, boxes)
        return known[1]

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
                return not leave or self.footprint.poses_fit(sample_arc(x, y, yaw, r, leave * quarter, self.step), True)
            return not arrive or self.footprint.poses_fit(
                [Pose2D(p.x, p.y, p.yaw + math.pi) for p in sample_arc(x, y, yaw + math.pi, r, -arrive * quarter,
                                                                         self.step)], True)

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


def _driven(rings: Sequence[Sequence[Sequence[float]]], corners: Sequence[XY]):
    """Where a body of these corners (counter-clockwise round its centre, see PlannerConfig.body_outline) was while the
    centre drove along each of the rings the way its points go: along every edge from where it was at the edge's start
    to where it was at its end (_rows), so past a corner and on the outside of a bend the front reaches out, and at
    every point turning on the spot from the heading of the edge before to the next one's (TURN_STEP apart)."""
    parts, rows = [], _rows(corners)

    def body(x: float, y: float, h: float, points: Sequence[XY]) -> Polygon:
        c, s = math.cos(h), math.sin(h)
        return Polygon([(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in points])

    for ring in rings:
        heads = []  # every edge's heading and end
        for (ax, ay), (bx, by) in zip(ring, list(ring[1:]) + list(ring[:1])):
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-9:
                heads.append((math.atan2(by - ay, bx - ax), bx, by))
                parts += [body(bx, by, heads[-1][0], [(dx - length * back, dy) for dx, dy, back in row])
                          for row in rows]
        for (h, x, y), (h1, _, _) in zip(heads, heads[1:] + heads[:1]):
            turn = normalize_angle(h1 - h)
            n = int(math.ceil(abs(turn) / TURN_STEP))
            parts += [body(x, y, h + turn * k / n, corners) for k in range(1, n)]
    return union(parts)


def _rows(corners: Sequence[XY]) -> List[List[Tuple[float, float, int]]]:
    """The body of these corners (counter-clockwise) in pieces every line along its heading crosses once (itself where
    it is like that, else cut along the heading where its edges turn from going left to going right or back), each as
    its corners (x, y, 1 on its rear side; the leftmost and rightmost on both sides): driving straight ahead it swept
    the polygon of a piece's corners where it got to, those on the rear side as far back as it drove."""
    ys = [y for _, y in corners]
    goes = [(b > a, b) for a, b in zip(ys, ys[1:] + ys[:1]) if b != a]  # (to the left, to where)
    turns = sorted({y for (left, y), (on, _) in zip(goes, goes[1:] + goes[:1]) if left != on})
    pieces = [list(corners)]
    if len(turns) > 2:
        x0, _, x1, _ = Polygon(corners).bounds
        pieces = [ring for a, b in zip(turns, turns[1:])
                  for _, ring in rings(clip_by_rect(Polygon(corners), x0 - 1.0, a, x1 + 1.0, b))]
    rows = []
    for piece in pieces:
        row: List[Tuple[float, float, int]] = []
        for i, (x, y) in enumerate(piece):
            into, out = y - piece[i - 1][1], piece[(i + 1) % len(piece)][1] - y
            if into == out == 0:  # (in the middle of an edge along it)
                continue
            if into >= 0 and out >= 0 or into <= 0 and out <= 0:
                row.append((x, y, int(into <= 0 and out <= 0)))
            else:
                row += [(x, y, 0), (x, y, 1)] if into > 0 else [(x, y, 1), (x, y, 0)]
        rows.append(row)
    return rows


def _length(pts: Sequence[XY]) -> float:
    return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))


def _spins(poses: Sequence[Pose2D], heading_in: Optional[float], heading_out: Optional[float]) -> int:
    """How often the mower turns on the spot by more than SPIN_LIMIT driving poses, coming in facing heading_in and
    going on facing heading_out."""
    yaws = ([heading_in] if heading_in is not None else []) + [p.yaw for p in poses] + \
        ([heading_out] if heading_out is not None else [])
    return sum(1 for a, b in zip(yaws, yaws[1:]) if angle_diff(a, b) > SPIN_LIMIT)


def _smooth_spins(route: '_Route', cfg: PlannerConfig) -> None:
    """Where the path turns on the spot by more than SPIN_LIMIT (a sharp corner of a loop, a drive arriving across
    a lane, a pivot), an omega (maneuvers.omega) of turn_radius (down to SMOOTH_RADII of it) in its place if it fits in
    the space of the segment (the drivable areas for drives, with headland_turns out of the lanes' field where the turn
    is) and the body fits: the wheels keep rolling. Where none
    fits and it may back up (allow_reverse), a K-turn in place (maneuvers.k_point), forwards first or backwards first
    (where it came from)."""
    inserts: Dict[Tuple[int, int], List[Tuple[List[Pose2D], bool]]] = {}
    before: Optional[Pose2D] = None  # the last pose before the position the mower is at
    here: Optional[Tuple[Pose2D, int, int]] = None  # the pose it leaves this position with, where it is

    def smooth() -> None:
        if before is None or here is None:
            return
        p, i, j = here
        turn = normalize_angle(p.yaw - before.yaw)
        if abs(turn) <= SPIN_LIMIT:
            return

        def straight(k: int) -> bool:
            return route.segments[k].kind == LANE and route.segments[k].maneuver not in ('ring', 'bend')

        if j == 0 and i > 0 and straight(i):  # where a lane starts: on the end of the segment before
            i, j = i - 1, len(route.segments[i - 1].poses)
        if straight(i) or route.segments[i].reverse:  # (a lane keeps its heading)
            return
        space = route.transit.is_free if route.segments[i].kind == TRANSIT else route.area.is_free
        clear = route.clear([(p.x, p.y)])  # (headland_turns: out of the lanes' field the loop stays out of it)

        def fits(pts: Sequence[XY]) -> bool:
            return space(pts) and (not clear or route.clear(pts))

        at = Pose2D(p.x, p.y, before.yaw)
        body = route.turn_body if route.segments[i].kind == TURN else route.body  # (on a turn spin_margin too)
        for share in SMOOTH_RADII:
            r = share * cfg.turn_radius
            loop = omega(at, turn, r, route.step) if r >= max(1e-3, cfg.min_turn_radius - 1e-9) else None
            if loop and fits([(q.x, q.y) for q in loop]) and (body is None or body([before] + loop)):
                inserts[(i, j)] = [(loop, False)]
                route._count('spins_smoothed')
                return
        if not cfg.allow_reverse:
            return
        # where the body sticks out turning on the spot, one that sticks out no more is better still
        fp = route.footprint
        spin: Optional[float] = None
        best = None
        for share in SMOOTH_RADII + (0.2,):
            for back_first in (False, True):
                r = share * cfg.turn_radius
                if r < max(1e-3, cfg.min_turn_radius - 1e-9):
                    continue
                m = k_point(at, turn, r, route.step, back_first)
                poses = [q for piece in m.pieces for q in piece.poses]
                if not fits([(q.x, q.y) for q in poses]):
                    continue
                if fp is None or fp.poses_fit([before] + poses):
                    out = 0.0
                else:
                    if spin is None:
                        spin = fp.outside([before, at, p])
                    out = fp.outside([before] + poses) if spin > 0 else math.inf
                if out <= 1e-9 or out <= spin + 1e-9:
                    best = (out, m)  # the widest that fits (or sticks out no more), back up as little as that
                    break
            if best is not None:
                break
        if best is not None:
            inserts[(i, j)] = [(piece.poses, piece.reverse) for piece in best[1].pieces]
            route._count('spins_backed_up')

    for i, seg in enumerate(route.segments):
        for j, p in enumerate(seg.poses):
            if here is not None and route.segments[here[1]].section != seg.section:
                before = here = None  # the path jumps to the next section
            if here is not None and same_position(here[0], p):
                here = (p, i, j)  # the same position, now left facing p's way
                continue
            smooth()
            before, here = (here[0] if here is not None else None), (p, i, j)
    for (i, j), pieces in sorted(inserts.items(), reverse=True):
        seg = route.segments[i]
        # (the last pose is where the segment goes on from: one pose there when they're put together, a backing up
        # segment ends there)
        if len(pieces) == 1:
            seg.poses[j:j] = pieces[0][0]
            continue
        # backing up is a segment of its own
        parts = ([dataclasses.replace(seg, poses=seg.poses[:j])] if j else []) + \
            [dataclasses.replace(seg, poses=poses, reverse=reverse, maneuver='k_point' if reverse else seg.maneuver)
             for poses, reverse in pieces] + \
            ([dataclasses.replace(seg, poses=seg.poses[j:])] if j < len(seg.poses) else [])
        route.segments[i:i + 1] = parts


def _fit_body(route: '_Route', poses: List[Pose2D],
              infos: List[SegmentInfo]) -> Tuple[List[Pose2D], List[SegmentInfo], Dict[str, Any]]:
    """Where the body still sticks out (at a pose, or turning on the spot there), the path from a pose before it to one
    after it (FIT_REACH before and after at most) another way that fits: straight (turning on the spot at both ends),
    Dubins' ways (FIT_ARCS), or in to the first point from where the two headings meet on towards where it can turn
    round whatever way it faces (FIT_VIA) that it can turn at, and out again (straight, the further ways easing off the
    path's line and onto it, FIT_EASE). Of those the one that leaves the least of the loops and lanes out, a metre
    driven counting FIT_DRIVE of that (with headland_turns one into the lanes' field FIT_FIELD more), FIT_PLACE ways
    tried at most for a place (and the Dubins' ways) and FIT_TRIES for all. Where none fits it turns round before the
    place, or goes through turn_room (see FIT_AROUND), or drives on through the transit space to one of the next poses
    that fit, or turns round further to the side, and where none of that fits the path jumps from the pose before to the
    next one that fits (a section starts later where the body doesn't fit at its start, nothing fits FIT_START after it
    or nothing else FIT_LATER after it, the path ends before where it would jump last).
    Where the path starts later in its first section or ends before, a way from where the body turns whatever way it
    faces closest to its start, or to the one closest to its end. Returns the poses, the segments (the ways in place
    TRANSIT 'body', a jump within a section starts one of its own) and {'fixed': ways in place, 'left': places left out
    (not the path starting later in its first section or ending before, FIT_START m of loops and lanes at most),
    'places': [[x, y, m of loops and lanes left out]] of both, at the pose of the path as planned where the body sticks
    out the most there, 'poses': [[x, y, yaw, m, spin]] of those poses (how far it sticks out of the space that applies
    there, see Footprint.out_by, spin 1 where that's turning on the spot), 'skipped_m': those m added up, 'jumps': [[x,
    y]] of the places the path jumps over, 'unchecked': [[x, y]] where it starts later or ends before with no such way,
    'tries': ways tried}."""
    fp, step = route.footprint, route.step
    n = len(poses)
    section, mows = [0] * n, [False] * n  # (mows: on the way to it the blade mows a loop or a lane)
    for info in infos:
        section[info.start:info.end + 1] = [info.section] * (info.end + 1 - info.start)
        mows[info.start + 1:info.end + 1] = [info.kind in (PERIMETER, LANE)] * (info.end - info.start)
    starts = {i for i in range(n) if i == 0 or section[i] != section[i - 1]}
    on_turn = [False] * n  # (on the way to it: a turn between lanes, spin_margin all along it too)
    for info in infos:
        on_turn[info.start + 1:info.end + 1] = [info.kind == TURN] * (info.end - info.start)
    fit, turn_room = fp.fit(poses, on_turn), fp.turning.anywhere
    tries, most = [0], [0]  # (most: the tries this place may take up to)
    turns: Dict[Tuple[float, ...], bool] = {}  # fits_turn where and which way, rounded (ways share their corner)

    def hop(i: int) -> float:  # m from the pose before
        return math.hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y)

    def fits(prev: Optional[Pose2D], i: int) -> bool:
        p = poses[i]
        if prev is not None and angle_diff(p.yaw, prev.yaw) > SPIN:
            return fp.fits_turn(p.x, p.y, prev.yaw, p.yaw)
        return fit[i]

    def spent() -> bool:
        return tries[0] >= min(FIT_TRIES, most[0])

    def ok(before: List[Pose2D], way: List[Pose2D]) -> bool:
        if spent():
            return False
        tries[0] += 1
        # (where it leaves and gets back on first: a way that sticks out mostly does so there)
        return len(way) > 1 and all(fp.fits_pose(q.x, q.y, q.yaw) for q in (way[0], way[-2])) and \
            route.transit.is_free([(q.x, q.y) for q in way]) and fp.poses_fit(before + way)

    def straight(a: Pose2D, b: Pose2D) -> List[Pose2D]:
        return join(sample_straight(a.x, a.y, b.x, b.y, math.atan2(b.y - a.y, b.x - a.x), step), [b])

    def ways(a: Pose2D, b: Pose2D, eased: bool) -> Iterator[List[Pose2D]]:
        yield straight(a, b)
        # (round: no turning on the spot, the body needs no room round where it would. Tries of their own, the ways
        # after them get as many as before; with headland_turns not into the lanes' field)
        for share in FIT_ARCS:
            for m in dubins(a, b, share * route.cfg.turn_radius, step, turning=math.pi)[:FIT_DUBINS]:
                way = m.pieces[0].poses
                if route.field is None or route.clear([(q.x, q.y) for q in way]):
                    most[0] += 1
                    yield way
        # in to the first point that it can turn at from where the two headings meet (a corner) on towards where it
        # can turn round whatever way it faces, and out again: straight, or (eased) off a's line and onto b's no
        # faster than the rear and the front swing out
        ux, uy, vx, vy = math.cos(a.yaw), math.sin(a.yaw), math.cos(b.yaw), math.sin(b.yaw)
        wx, wy, det = b.x - a.x, b.y - a.y, ux * vy - uy * vx
        t, u = ((wx * vy - wy * vx) / det, (ux * wy - uy * wx) / det) if abs(det) > 1e-6 else (-1.0, -1.0)
        c = Point(a.x + t * ux, a.y + t * uy) if t > 0 and u > 0 else Point(a.x + 0.5 * wx, a.y + 0.5 * wy)
        far = c if turn_room.is_empty or turn_room.contains(c) else nearest_points(turn_room, c)[0]
        (cx, cy), (fx, fy) = c.coords[0], far.coords[0]
        span, chord = math.hypot(fx - cx, fy - cy), math.hypot(wx, wy)
        behind = Pose2D(b.x, b.y, b.yaw + math.pi)  # (getting back on b's line: getting off it the other way)
        vias = []  # (where it turns, which way in and out)
        for d in FIT_VIA:
            f = min(1.0, d / span) if span > 0 else 1.0
            x, y = cx + f * (fx - cx), cy + f * (fy - cy)
            if min(d, span) >= chord or min(math.hypot(x - a.x, y - a.y), math.hypot(b.x - x, b.y - y)) < 1e-3:
                break
            if not eased:
                vias.append((x, y, math.atan2(y - a.y, x - a.x), math.atan2(b.y - y, b.x - x)))
            else:
                # (the last points only, which way it turns there: all of them for the way it takes)
                leave, back = ease(a, x, y, fp.rear, 1), ease(behind, x, y, fp.front, 1)
                if leave and back:
                    vias.append((x, y, math.atan2(y - leave[0][1], x - leave[0][0]),
                                 math.atan2(back[0][1] - y, back[0][0] - x)))
            if f >= 1.0:
                break
        if not eased:
            # (where it leaves and gets back on first, all at once)
            ends = fp.fit([q for _, _, into, out_of in vias for q in (
                Pose2D(a.x, a.y, into), Pose2D(b.x - step * math.cos(out_of), b.y - step * math.sin(out_of), out_of))])
            vias = [via for k, via in enumerate(vias) if ends[2 * k] and ends[2 * k + 1]]
        for x, y, into, out_of in vias:
            if turns_at(x, y, into, out_of):
                if eased:
                    yield join(along(ease(a, x, y, fp.rear)), along(ease(behind, x, y, fp.front)[::-1])[:-1], [b])
                else:
                    yield join(straight(a, Pose2D(x, y, into)), straight(Pose2D(x, y, out_of), b))
                break

    def ease(p: Pose2D, x: float, y: float, end: float, last: int = 0) -> Optional[List[XY]]:
        # points from p to (x, y) (the last ones only), off p's line as FIT_EASE says (end: m the body reaches out
        # that way), then beside it. None where (x, y) isn't ahead
        c, s = math.cos(p.yaw), math.sin(p.yaw)
        ahead, off = (x - p.x) * c + (y - p.y) * s, (y - p.y) * c - (x - p.x) * s
        if ahead < step:
            return None
        k, pts = int(math.ceil(ahead / step)), []
        for i in range(k - last if last else 0, k):
            t = ahead * i / k
            o = math.copysign(min(abs(off), FIT_EASE_OFF * math.expm1(min(30.0, t / (FIT_EASE * end)))), off)
            pts.append((p.x + t * c - o * s, p.y + t * s + o * c))
        return pts + [(x, y)]

    def along(pts: List[XY]) -> List[Pose2D]:  # step apart at most, facing where they go
        return join(*(sample_straight(p[0], p[1], q[0], q[1], math.atan2(q[1] - p[1], q[0] - p[0]), step)
                      for p, q in zip(pts, pts[1:])))

    def turns_at(x: float, y: float, yaw0: float, yaw1: float) -> bool:
        key = (round(x, 2), round(y, 2), round(yaw0, 1), round(yaw1, 1))
        if key not in turns:
            turns[key] = fp.fits_turn(x, y, yaw0, yaw1)
        return turns[key]

    def legs(rs: List[int], x: float, y: float, into: bool) -> Iterator[Tuple[int, List[Pose2D]]]:
        # from out[r] to (x, y) (into), or from there to poses[r], for r of rs in turn: eased off the pose's line (onto
        # it) or straight, where they fit (where they leave and get there first, a few at once)
        for chunk in (rs[c:c + 8] for c in range(0, len(rs), 8)):
            found = []  # (r, the points in the order it drives them)
            for r in chunk:
                p = out[r] if into else poses[r]
                if math.hypot(x - p.x, y - p.y) >= 2.0 * step:
                    for pts in (ease(p if into else Pose2D(p.x, p.y, p.yaw + math.pi), x, y,
                                     fp.rear if into else fp.front), [(p.x, p.y), (x, y)]):
                        if pts:
                            found.append((r, pts if into else pts[::-1]))
            there = fp.fit([Pose2D(w[0], w[1], math.atan2(v[1] - u[1], v[0] - u[0])) for _, go in found
                            for u, v, w in ((go[0], go[1], go[0]), (go[-2], go[-1], go[-1]))])
            for k, (r, go) in enumerate(found):
                if there[2 * k] and there[2 * k + 1] and not spent():
                    tries[0] += 1
                    way = along(go) if into else join(along(go), [poses[r]])
                    if route.transit.is_free(go) and fp.poses_fit(([out[r - 1]] if into and r > first else []) + way):
                        yield r, way

    def turn_round(i: int, wider: int = 0) -> Optional[Tuple[float, int, int, List[Pose2D]]]:
        # (see FIT_AROUND) the place: from i to where the poses fit again for FIT_STEP[0] m, the turn the path makes
        # over it. wider: of the ways to the side the body fits at that many left out
        end, run, d = None, 0.0, 0.0
        for j in range(i + 1, n):
            d += hop(j)
            if j in starts or d > FIT_REACH[-1]:
                return None
            if not fit[j]:
                end = None
            elif end is None:
                end, run = j, 0.0
            else:
                run += hop(j)
                if run >= FIT_STEP[0]:
                    break
        else:
            return None
        turn = normalize_angle(poses[i].yaw - out[-1].yaw) + sum(
            normalize_angle(poses[j].yaw - poses[j - 1].yaw) for j in range(i + 1, end + 1))
        near, last, d = [], -FIT_STEP[0], 0.0  # (m from the place, pose)
        for q in range(i, end):
            d += hop(q) if q > i else 0.0
            if d >= last + FIT_STEP[0]:
                near.append((0.0, q))
                last = d
        for q, ahead in ((i - 1, -1), (end, 1)):
            d, last = 0.0, 0.0
            while 0 < q < n - 1 and d < FIT_AROUND:
                if d >= last:
                    near.append((d, q))
                    last = d + FIT_STEP[0]
                q += ahead
                d += hop(max(q, q - ahead))
        seen[0] = max([seen[0], end] + [q for _, q in near])
        backs, ons = ends(i, FIT_REACH[0], FIT_REACH[-1], 0.0)
        ons = [o for o in ons if o[2] >= end]
        side, points = (1.0 if turn >= 0 else -1.0), 0
        for _, q in sorted(near):
            if points == FIT_TURNS:
                break
            p = poses[q]
            c, s = math.cos(p.yaw), math.sin(p.yaw)
            moved = [(p.x - off * s, p.y + off * c) for off in (f * sign for f in FIT_IN for sign in (side, -side))]
            there = fp.turning.fit([Pose2D(x, y, p.yaw + turn * f) for x, y in moved for f in (0.0, 0.5, 1.0)])
            x, y = ([xy for k, xy in enumerate(moved) if all(there[3 * k:3 * k + 3])][wider:] + [(None, None)])[0]
            if x is None or not route.transit.is_free([(x, y)]):
                continue
            points += 1
            # in from the latest poses it fits from (3 at most), out to the earliest after the place it turns into from
            # one of those
            ins = []
            for k, into in legs([k for _, _, k in backs], x, y, True):
                ins.append((next(m for m, _, r in backs if r == k), k, into))
                if len(ins) == 3:
                    break
            for j, onto in legs([j for _, _, j in ons], x, y, False) if ins else ():
                ma = next(m for m, _, r in ons if r == j)
                for mb, k, into in ins:
                    if (angle_diff(into[-1].yaw, onto[0].yaw) <= SPIN or turns_at(x, y, into[-1].yaw, onto[0].yaw)) \
                            and ok([out[k - 1]] if k > first else [], join(into, onto)):
                        return mb + ma, k, j, join(into, onto)
        return None

    def picked(rs: range, at: List[Pose2D], fits_at: Optional[List[bool]], most: int,
               far: float = math.inf) -> List[int]:
        # of the poses at[r] in the transit space (that fit) the first, then FIT_STEP[-1] m of path on, twice as far
        # each time (most of them, up to far m of path)
        got, d, nxt, last = [], 0.0, 0.0, 0
        for r in rs:
            d += math.hypot(at[r].x - at[last].x, at[r].y - at[last].y) if got else 0.0
            last = r
            if d > far:
                break
            if (fits_at is None or fits_at[r]) and d >= nxt and route.transit.is_free([(at[r].x, at[r].y)]):
                got.append(r)
                nxt = max(FIT_STEP[-1], 2.0 * d)
                if len(got) == most:
                    break
        return got

    def drive_on(i: int) -> Optional[Tuple[float, int, int, List[Pose2D]]]:
        # a drive (_Route.drive_to) through the transit space from one of the latest poses in it (FIT_TURNS of them,
        # picked()) to one of the next ones in it that fit (up to the section's end)
        end = next((j for j in range(i + 1, n) if j in starts), n)
        seen[0] = max(seen[0], end - 1)
        backs = picked(range(len(out) - 1, first - 1, -1), out, None, FIT_TURNS)
        for j in picked(range(i + 1, end), poses, fit, n):
            for k in backs:
                way = route.drive_to((poses[j].x, poses[j].y), poses[j].yaw, None, route.transit,
                                     ((out[k].x, out[k].y), out[k].yaw))
                if way and ok([out[k - 1]] if k > first else [], join(way, [poses[j]])):
                    return 0.0, k, j, join(way, [poses[j]])
        return None

    def through_room(k: int, j: int) -> Iterator[List[Pose2D]]:
        # in to where it turns whatever way it faces (the closest point of turn_room to FIT_STEP[-1] ahead of out[k]),
        # through there and out from the closest to FIT_STEP[-1] behind poses[j]
        if room[0] is None:
            room[0] = TransitPlanner(turn_room.simplify(ROOM_SIMPLIFY), 0.0)
        space, both = room[0].space, []
        for r, into in ((k, True), (j, False)):
            if (r, into) not in rooms:
                p, ahead = (out[r], FIT_STEP[-1]) if into else (poses[r], -FIT_STEP[-1])
                at = Point(p.x + ahead * math.cos(p.yaw), p.y + ahead * math.sin(p.yaw))
                at = at if space.contains(at) else nearest_points(space, at)[0]
                rooms[(r, into)] = next(((at.x, at.y, way) for _, way in legs([r], at.x, at.y, into)), None)
            both.append(rooms[(r, into)])
        mid = room[0].path(both[0][:2], both[1][:2]) if None not in both else None
        if mid is not None:
            yield join(both[0][2], along(mid), both[1][2])

    def ends(i: int, back: float, on: float, every: float) -> Tuple[List[Tuple[float, float, int]], ...]:
        # where to leave the path (back along what's planned already, `back` m at most) and where to get back on (`on`
        # m), `every` apart: (m of loops and lanes left out, m of path, index)
        backs, mown, d, k = [], 0.0, 0.0, len(out) - 1
        while k >= first and d <= back:
            if not backs or d >= backs[-1][1] + every - 1e-9:
                backs.append((mown, d, k))
            if k > first:
                d += math.hypot(out[k].x - out[k - 1].x, out[k].y - out[k - 1].y)
                mown += cut[k]
            k -= 1
        ons, d, j = [], 0.0, i
        mown = hop(i) if mows[i] else 0.0
        while j + 1 < n and j + 1 not in starts and d <= on:
            j += 1
            seen[0] = max(seen[0], j)
            d += hop(j)
            mown += hop(j) if mows[j] else 0.0
            if (not ons or d >= ons[-1][1] + every - 1e-9) and fit[j]:
                ons.append((mown, d, j))
        return backs, ons

    def search(i: int, reach: float, every: float,
               eased: Optional[bool]) -> Optional[Tuple[float, int, int, List[Pose2D]]]:
        # the best way: (cost, where it leaves, gets back on, poses)
        backs, ons = ends(i, reach, reach, every)
        best = None
        # (by what they cost at least: straight from one to the other)
        pairs = sorted((mb + ma + FIT_DRIVE * math.hypot(poses[j].x - out[k].x, poses[j].y - out[k].y), mb + ma, k, j)
                       for mb, _, k in backs for ma, _, j in ons)
        for least, lost, k, j in pairs:
            if best is not None and least >= best[0] or spent():
                break
            before = [out[k - 1]] if k > first else []
            for way in through_room(k, j) if eased is None else ways(out[k], poses[j], eased):
                field = 0.0 if route.field is None or route.clear([(q.x, q.y) for q in way]) else FIT_FIELD
                cost = lost + FIT_DRIVE * path_length(way) + field
                if (best is None or cost < best[0]) and ok(before, way):
                    best = (cost, k, j, way)
                    if not field:
                        break  # (the next way is longer)
        return best

    # turning round: the turn_room's TransitPlanner (made when needed), the ways in and out of it of a place
    room: List[Optional[TransitPlanner]] = [None]
    rooms: Dict[Tuple[int, bool], Optional[Tuple[float, float, List[Pose2D]]]] = {}
    out: List[Pose2D] = []
    orig: List[Optional[int]] = []  # which of poses each one is, None: of a way in place of some
    cut: List[float] = []  # m the blade mows of a loop or a lane on the way to each
    begins, jumps = set(), []  # where in out the sections begin, and where the path jumps (out, poses left out)
    first, i = 0, 0
    seen = [0]  # the furthest of poses read (with its section and the next one's, whether the blade mows on the way)
    # (kept only as far as a plan at another lane angle reads the same: up to the first lane, rings are loops)
    lanes = min((info.start for info in infos if info.kind == LANE and info.maneuver != 'ring'), default=n)
    # where a plan before with the same body, spaces and settings got to, as far as it read the same poses
    hashed = hashlib.sha1(repr((step, fp.corners, fp.spin_margin, fp.turning.back, FIT_REACH, FIT_STEP,
                                FIT_PLACE, FIT_VIA, FIT_EASE, FIT_EASE_OFF, FIT_DRIVE, FIT_TRIES, FIT_START, FIT_LATER,
                                FIT_FIELD, FIT_AROUND, FIT_IN, FIT_TURNS, FIT_ROUND, ROOM_SIMPLIFY,
                                route.cfg.turn_radius, route.cfg.min_turn_radius, route.cfg.turn_on_spot,
                                route.cfg.allowed_turns, route.cfg.headland_corners)).encode())
    for geom in (fp.space, route.transit.space, route.field):
        hashed.update(b'-' if geom is None else geom.wkb)
    key = hashed.hexdigest()
    with _KEPT_LOCK:
        earlier = _FITTED.get(key)

    def held(got: tuple) -> tuple:  # (a copy of where it got to: the ones kept never change)
        out, orig, cut, begins, jumps, first, i, tried, turns = got
        return (list(out), list(orig), list(cut), set(begins), [(r, list(gone)) for r, gone in jumps], first, i, tried,
                dict(turns))

    kept: List[Tuple[int, tuple]] = []
    if earlier is not None:
        for k in range(len(earlier[3]) - 1, -1, -1):
            h, got = earlier[3][k]
            if all(now[:h + 2] == then[:h + 2] for now, then in zip((poses, section, mows), earlier)):
                out, orig, cut, begins, jumps, first, i, tries[0], turns = held(got)
                seen[0], kept = h, earlier[3][:k + 1]
                break
    mark = (i // FIT_KEEP + 1) * FIT_KEEP

    def alone(upto: float = 0.0) -> None:
        # (what's driven since the section began or the path jumped is left out too where it's upto long at most: a
        # pose on its own)
        if len(out) > first and path_length(out[first:]) <= upto:
            begins.discard(first)
            if not jumps or jumps[-1][0] != first:
                jumps.append((first, []))
            jumps[-1][1].extend(o for o in orig[first:] if o is not None)
            del out[first:], orig[first:], cut[first:]

    while i < n:
        seen[0] = max(seen[0], i)
        if i >= mark and seen[0] + 2 <= lanes:  # (for a plan after this one)
            mark = (i // FIT_KEEP + 1) * FIT_KEEP
            kept.append((seen[0], held((out, orig, cut, begins, jumps, first, i, tries[0], turns))))
        if i in starts:
            alone()
            first = len(out)
        p = poses[i]
        prev = out[-1] if len(out) > first else None
        best = None
        if not fits(prev, i):
            if prev is None:  # the section starts later
                if not jumps or jumps[-1][0] != len(out):
                    jumps.append((len(out), []))
                jumps[-1][1].append(i)
                i += 1
                continue
            for further, (reach, every, tried) in enumerate(zip(FIT_REACH, FIT_STEP, FIT_PLACE)):
                if best is None and tries[0] < FIT_TRIES:
                    most[0] = tries[0] + tried
                    best = search(i, reach, every, further > 0)
            if best is None and path_length(out[first:]) > FIT_START:  # (else the section starts later)
                most[0] = tries[0] + FIT_ROUND
                best = turn_round(i)
                if best is None and not turn_room.is_empty:
                    rooms.clear()
                    most[0] = tries[0] + FIT_ROUND
                    best = search(i, FIT_REACH[-1], FIT_STEP[-1], None)
                if best is None:
                    most[0] = tries[0] + FIT_ROUND
                    best = drive_on(i)
                if best is None and sum(hop(o) for o in range(i, n) if mows[o]) > FIT_START:  # (else it ends before)
                    most[0] = tries[0] + FIT_ROUND
                    best = next(filter(None, (turn_round(i, wider) for wider in range(1, 2 * len(FIT_IN)))), None)
            if best is None:
                # nothing fits: the path jumps from the pose before to the next one that fits (the section starts later
                # where it began FIT_LATER before at most)
                alone(FIT_LATER)
                if len(out) == first:
                    continue
                if not jumps or jumps[-1][0] != len(out):
                    jumps.append((len(out), []))
                first = len(out)
                continue
        if best is None:
            if prev is None:
                begins.add(len(out))
            out.append(p)
            orig.append(i)
            cut.append(math.hypot(p.x - prev.x, p.y - prev.y) if prev is not None and mows[i] else 0.0)
            i += 1
            continue
        _, k, j, way = best
        del out[k + 1:], orig[k + 1:], cut[k + 1:]
        out[k] = way[0]
        out.extend(way[1:])
        orig.extend([None] * (len(way) - 2) + [j])
        cut.extend([0.0] * (len(way) - 1))
        i = j + 1
    _keep(_FITTED, key, (poses[:lanes], section[:lanes], mows[:lanes], kept))
    alone()
    # (where the body doesn't fit by the path's end, the dock, it ends before: no place left out where the path would
    # jump last and that and what's after leave FIT_START m of loops and lanes out at most, it gets to the end itself)
    ended = jumps and 0 < jumps[-1][0] < len(out) and \
        sum(cut[jumps[-1][0]:]) + sum(hop(o) for o in jumps[-1][1] if mows[o]) <= FIT_START
    if ended:
        r = jumps.pop()[0]
        del out[r:], orig[r:], cut[r:]
        begins.discard(r)
    # (the jumps within a section start one of its own)
    splits = sorted(r for r, gone in jumps if r < len(out) and not (gone and gone[0] in starts))

    def part(r: int) -> int:  # the section of out[r]
        return section[orig[r]] + bisect.bisect_right(splits, r)

    # the segments: the planned ones where their poses are still one after the other, the ways in place between
    at = {o: r for r, o in enumerate(orig) if o is not None}
    new: List[SegmentInfo] = []
    for info in infos:
        run: List[int] = []
        for o in range(info.start, info.end + 2):
            r = at.get(o) if o <= info.end else None
            if r is not None and run and r == run[-1] + 1 and r not in begins:
                run.append(r)
                continue
            if len(run) > 1:
                new.append(dataclasses.replace(info, start=run[0], end=run[-1], section=part(run[0])))
            run = [r] if r is not None else []

    def planned(r: int) -> bool:  # out[r - 1] to out[r] as planned, or the path jumps there
        return r in begins or orig[r] is not None and orig[r - 1] is not None and orig[r] == orig[r - 1] + 1

    def mown(gone: Sequence[int]) -> float:
        return round(sum(hop(o) for o in gone if mows[o]), 2)

    worst: List[List[float]] = []

    def place_of(a: int, b: int, m: float) -> List[float]:
        # [x, y, m of loops and lanes left out] of a place over poses[a..b]: where of those the body sticks out the most
        # (Footprint.out_by: PLACE_STEP apart along its edges first, then closer round the farthest of those that may be
        # it), [x, y, yaw, how far, 1 where it turns on the spot there else 0] of that pose to worst
        bad = [(poses[o - 1] if o not in starts else None, o) for o in range(a, b + 1)
               if not fits(poses[o - 1] if o not in starts else None, o)]
        best = None
        for (rough, at), (before, o) in sorted(zip(fp.out_by([(q, poses[o]) for q, o in bad], PLACE_STEP,
                                                             on_turn=[on_turn[o] for _, o in bad]), bad),
                                               key=lambda g: (-g[0][0], g[1][1])):
            if best is not None and rough + 0.5 * PLACE_STEP <= best[0]:
                break
            d = max(rough, fp.out_by([(before, poses[o])], around=at and (at[0], at[1], PLACE_STEP),
                                     on_turn=[on_turn[o]])[0][0])
            if best is None or d > best[0]:
                best = (d, before, o)
        d, before, o = best or (0.0, None, (a + b) // 2)
        p = poses[o]
        spin = before is not None and angle_diff(p.yaw, before.yaw) > SPIN
        worst.append([round(p.x, 3), round(p.y, 3), round(p.yaw, 3), d, int(spin)])
        return [round(p.x, 2), round(p.y, 2), m]

    places = []
    r = 1
    while r < len(out):
        if planned(r):
            r += 1
            continue
        s = r - 1
        while r < len(out) and not planned(r):
            r += 1
        new.append(SegmentInfo(TRANSIT, s, r - 1, False, 'body', part(s)))
        places.append(place_of(orig[s] + 1, orig[r - 1], mown(range(orig[s] + 1, orig[r - 1] + 1))))
    fixed = len(places)

    def lead(at: Pose2D, rs: range, start: bool) -> Optional[Tuple[int, List[Pose2D]]]:
        # (k, the way) from the point closest to `at` where the body turns whatever way it faces (in the transit space)
        # to out[k] (start, but for out[k]) or from where out[k] is to there, k of rs (picked(), FIT_REACH[0] m of path
        # at most): eased onto or off the path's line or straight (legs()), else a drive
        room = turn_room.intersection(route.transit.space)
        if room.is_empty:
            return None
        q, most[0] = nearest_points(room, Point(at.x, at.y))[0], tries[0] + FIT_ROUND
        ks = picked(rs, out, None, FIT_TURNS, FIT_REACH[0])
        of = {orig[k]: k for k in ks if orig[k] is not None and out[k] == poses[orig[k]]}
        got = next(legs(list(of), q.x, q.y, False) if start else legs(ks, q.x, q.y, True), None)
        if got:
            return (of[got[0]], got[1][:-1]) if start else got
        for k in ks:
            p = out[k]
            if start:
                way = route.drive_to((p.x, p.y), p.yaw, None, route.transit, ((q.x, q.y), None))
                if way and ok([], way[:-1] + out[k:k + 2]):
                    return k, way[:-1]
            else:
                way = route.drive_to((q.x, q.y), None, None, route.transit, ((p.x, p.y), p.yaw))
                if way and ok(out[k - 1:k] if k else [], way):
                    return k, way
        return None

    # where the path starts later in its first section or ends before (by the dock: getting from there to the path and
    # back the body isn't checked), a way from where it turns whatever way it faces closest to where it would start, or
    # to the one closest to where it would end: a place driven another way. Where none fits it says where (unchecked),
    # it starts later all the same (no place left out where that leaves FIT_START m of loops and lanes out at most) or
    # ends before
    bounds = sorted(begins | set(splits) | {0, len(out)})
    leads, unchecked = [None, None], []
    if jumps and jumps[0][0] == 0 and out and section[orig[0]] == section[0]:
        gone = jumps[0][1]
        leads[0] = lead(poses[0], range(0, next(b for b in bounds if b > 0)), True)
        if leads[0] or mown(gone) <= FIT_START:
            jumps = jumps[1:]
        if leads[0]:
            fixed += 1
            places.append(place_of(0, orig[0], round(mown(gone) + sum(cut[1:leads[0][0] + 1]), 2)))
        else:
            unchecked.append([round(out[0].x, 2), round(out[0].y, 2)])
    if ended:
        first = max(b for b in bounds if b < len(out))  # (the last section's, see legs())
        leads[1] = lead(poses[-1], range(len(out) - 1, first - 1, -1), False)
        if leads[1]:
            fixed += 1
            places.append(place_of(orig[-1] + 1, n - 1,
                                   round(mown(range(orig[-1] + 1, n)) + sum(cut[leads[1][0] + 1:]), 2)))
        else:
            unchecked.append([round(out[-1].x, 2), round(out[-1].y, 2)])
    for r, gone in jumps:
        places.append(place_of(orig[r - 1] + 1 if r else 0, orig[r] if r < len(out) else n - 1, mown(gone)))
    new.sort(key=lambda info: (info.start, info.end))
    over = [place[:2] for (r, _), place in zip(jumps, places[fixed:]) if r in splits]
    if leads[1]:  # (the segments as far as out[k], then the way from there)
        k, way = leads[1]
        at = SegmentInfo(TRANSIT, k, k + len(way) - 1, False, 'body', new[-1].section)
        new = [dataclasses.replace(info, end=min(info.end, k)) for info in new if info.start < k] + [at]
        out[k:] = way
    if leads[0]:  # (the way, then the segments from out[k] on)
        k, way = leads[0]
        new = [SegmentInfo(TRANSIT, 0, len(way), False, 'body', new[0].section)] + [
            dataclasses.replace(info, start=max(info.start, k) + len(way) - k, end=info.end + len(way) - k)
            for info in new if info.end > k]
        out[:k] = way
    return out, new, {'fixed': fixed, 'left': len(jumps), 'skipped_m': round(sum(m for _, _, m in places), 2),
                      'places': places, 'poses': worst, 'jumps': over, 'unchecked': unchecked, 'tries': tries[0]}


def _lanes_field(passes: List[Tuple[float, List[List[Lane]]]], trim: float, half: float):
    """Where the lanes of every set lie, each cell's from lane to lane and half further out, but for trim at their
    ends."""
    parts = []
    for a, cells in passes:
        for cell in cells:
            ends = []
            for lane in cell:
                x0, x1 = lane.x0 + trim + half, lane.x1 - trim - half
                if x0 > x1:
                    x0 = x1 = 0.5 * (lane.x0 + lane.x1)
                ends.append(Lane(lane.row, x0, x1, lane.y).ends(a))
            if len(ends) > 1:
                parts.append(Polygon([e[0] for e in ends] + [e[1] for e in ends[::-1]]).buffer(half, 1))
    return union(parts)


def _field_turns(route: '_Route', poses: List[Pose2D],
                 infos: List[SegmentInfo]) -> Tuple[List[List[float]], int, float]:
    """[x, y] of every place where a drive turns in the lanes' field (headland_turns): the first pose there whose
    heading changes, those less than a metre apart one place. Not on the arcs rounding a corner out of it into it: how
    many of those turn in it, and how far in the furthest gets (m, along the way from pose to pose, to a few mm)."""
    places: List[List[float]] = []
    last: Optional[Pose2D] = None
    arcs, deepest, on = 0, 0.0, False  # (on: the pose before turns on one of those arcs in the field)
    for info in infos:
        if info.kind != TRANSIT:
            continue
        for i in range(info.start + 1, info.end + 1):
            p = poses[i]
            turns = angle_diff(p.yaw, poses[i - 1].yaw) > 1e-6 and not route.clear([(p.x, p.y)])
            if turns and (p.x, p.y) in route.rounded:
                arcs += not on
                q = poses[i - 1]
                way = [(q.x + f * (p.x - q.x), q.y + f * (p.y - q.y)) for f in (0.25, 0.5, 0.75, 1.0)]
                deepest = max([deepest] + [route.field.boundary.distance(Point(xy)) for xy in way
                                           if not route.clear([xy])])
            elif turns:
                if last is None or math.hypot(p.x - last.x, p.y - last.y) >= 1.0:
                    places.append([round(p.x, 2), round(p.y, 2)])
                last = p
            on = turns and (p.x, p.y) in route.rounded
    return places, arcs, round(deepest, 3)


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


def _round_corners(route: '_Route', poses: List[Pose2D],
                   infos: List[SegmentInfo]) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    """Where a drive or a way of _fit_body (TRANSIT) bends at a pose by more than CORNER_ANGLE (turns on the spot there
    from footprint.SPIN on), an arc in its place, of SMOOTH_RADII of turn_radius (tighter than min_turn_radius too): the
    widest that keeps in the transit space and the body fits on. Tangent to the straight ends of the ways in and out,
    into the loop, lane or turn next to it turn_radius at most (as much as a right angle of that takes) and only where
    the blade mows what it leaves of it on the arc or on what no rounding leaves out (but EARLIER_LEFT); else an S
    from as far back on the way in as that allows (or where the drive's straight way in starts) onto the drive's way out
    further on, or the other way round, turning 90 degrees more at most. With headland_turns out of the lanes' field
    where the corner is out of it, with headland_corners 'rounded' into it where nothing else fits (an arc only,
    FIELD_ARC deep at most, its points in route.rounded), with 'sharp' not where a drive along the headland goes
    (route.sharp). The arc goes with the drive, the piece next to it ends or starts where it does."""
    out = list(poses)
    own = [0] * len(out)  # the segment the way to each pose is part of (of one a segment starts with: that one)
    shared = [k > 0 and infos[k - 1].end == info.start for k, info in enumerate(infos)]
    for k, info in enumerate(infos):
        own[info.start + shared[k]:info.end + 1] = [k] * (info.end + 1 - info.start - shared[k])

    def straight(j: int, h: float) -> float:  # m from out[j] to out[j + 1] along h on the way, else 0
        p, q = out[j], out[j + 1]
        a, b = infos[own[j]], infos[own[j + 1]]
        if a.section != b.section or b.reverse or angle_diff(p.yaw, h) > 1e-6 or \
                abs((q.y - p.y) * math.cos(h) - (q.x - p.x) * math.sin(h)) > 1e-6:
            return 0.0
        return math.hypot(q.x - p.x, q.y - p.y)

    def leg(j: int, h: float, step: int, drive: bool = False) -> float:
        # m straight along h from out[j] back (step -1) or on (1), drive: on drives only
        d, j = 0.0, j - 1 if step < 0 else j
        while 0 <= j < len(out) - 1:
            hop = straight(j, h)
            if hop <= 0.0 or drive and infos[own[j + 1]].kind != TRANSIT:
                break
            d, j = d + hop, j + step
        return d

    def ways(p: Pose2D, u: float, w: float, r: float, s: bool) -> Iterator[Tuple[float, float, List[Pose2D]]]:
        # (m before p it leaves the way, m after p it gets back on, the poses between), s: the S's too
        turn = normalize_angle(w - u)
        trim = r * math.tan(0.5 * abs(turn))
        if trim <= min(back, on) + 1e-9:
            arc = sample_arc(p.x - trim * math.cos(u), p.y - trim * math.sin(u), u, r, turn, route.step)
            yield trim, trim, [arc[0]._replace(yaw=u)] + arc[1:-1] + [arc[-1]._replace(yaw=w)]  # (yaws as they were)
        # an S from as far back on the way in as it may (or where the drive's straight way in starts) onto the drive's
        # way out further on, or the other way round
        for a, b in [(f, d) for f in sorted({min(back, trim), min(behind, trim)}, reverse=True)
                     for d in (trim + 2.0 * r, ahead) if s and trim < d <= ahead] + \
                [(d, f) for f in sorted({min(on, trim), min(ahead, trim)}, reverse=True)
                 for d in (trim + 2.0 * r, behind) if s and trim < d <= behind]:
            for m in dubins(Pose2D(p.x - a * math.cos(u), p.y - a * math.sin(u), u),
                            Pose2D(p.x + b * math.cos(w), p.y + b * math.sin(w), w), r, route.step,
                            turning=abs(turn) + 0.5 * math.pi)[:1]:
                if m.name == 'u_turn' or not route.cfg.turn_on_spot:  # (no loop with turn_on_spot)
                    yield a, b, m.pieces[0].poses

    def fit(a: float, b: float, arc: List[Pose2D], out_of: bool) -> Optional[Tuple[int, int, List[Pose2D]]]:
        # (the poses before and after it that stay, those and it), where it fits
        s, e = i - 1, i + 1
        while s > 0 and math.hypot(out[s].x - p.x, out[s].y - p.y) < a - 1e-9:
            s -= 1
        while e < len(out) - 1 and math.hypot(out[e].x - p.x, out[e].y - p.y) < b - 1e-9:
            e += 1
        new = join(out[s:s + 1], arc, out[e:e + 1])
        pts = [(q.x, q.y) for q in arc]
        # (where it ends at the next corner that one turns from the arc's last but one pose)
        if not route.transit.is_free(pts) or out_of and not route.clear(pts) or \
                deep is not None and clear and deep.intersects(LineString(pts)) or \
                not route.body(new[1:] if same_position(arc[-1], out[e]) else arc):
            return None
        # of the pieces next to the drive it leaves out the blade mows all but EARLIER_LEFT on it or elsewhere
        left = [route.blade([arc[0], Pose2D(p.x - behind * math.cos(u), p.y - behind * math.sin(u), u)])] \
            if a > behind + 1e-9 else []
        left += [route.blade([Pose2D(p.x + ahead * math.cos(w), p.y + ahead * math.sin(w), w), arc[-1]])] \
            if b > ahead + 1e-9 else []
        if left:
            left = subtract(union(left), [route.blade(arc)])
            if left.area > EARLIER_LEFT and subtract(left, [mowed()]).area > EARLIER_LEFT:
                return None
        return s, e, new

    # (the poses as they were by the square metre they're in, those further than turn_radius along the path from a
    # drive: no rounding leaves them out)
    grid: Dict[Tuple[int, int], List[int]] = {}
    for j, q in enumerate(poses):
        grid.setdefault((math.floor(q.x), math.floor(q.y)), []).append(j)
    gap = [math.inf] * len(poses)
    for info in infos:
        if info.kind == TRANSIT:
            gap[info.start:info.end + 1] = [0.0] * (info.end + 1 - info.start)
    for js in (range(1, len(poses)), range(len(poses) - 2, -1, -1)):
        for j in js:
            q, o = poses[j], poses[j - js.step]
            gap[j] = min(gap[j], gap[j - js.step] + math.hypot(q.x - o.x, q.y - o.y))

    def mowed():
        # what the blade mows round the corner on the poses as they were that no rounding leaves out (worked out once a
        # corner)
        if not here:
            reach = min(max(back, on), route.cfg.turn_radius * math.tan(0.5 * angle_diff(w, u))) + \
                route.cfg.mower_width + 2.0 * (abs(route.cfg.blade_ahead) + abs(route.cfg.blade_offset)) + route.step
            x0, y0, x1, y1 = (math.floor(v) for v in (p.x - reach, p.y - reach, p.x + reach, p.y + reach))
            runs: List[List[Pose2D]] = []
            for j in sorted(j for gx in range(x0, x1 + 1) for gy in range(y0, y1 + 1) for j in grid.get((gx, gy), ())
                            if gap[j] > route.cfg.turn_radius + 1e-6
                            and math.hypot(poses[j].x - p.x, poses[j].y - p.y) <= reach):
                if not runs or runs[-1][-1] != j - 1:
                    runs.append([])
                runs[-1].append(j)
            here.append(route.blade(*[poses[run[0]:run[-1] + 1] for run in runs if len(run) > 1]))
        return here[0]

    strict = route.cfg.headland_corners == 'sharp'
    # (what of the lanes' field is more than FIELD_ARC in: an arc into it from a corner out of it keeps out of that)
    deep = prep(route.field.buffer(-FIELD_ARC, QUAD_SEGS)) if route.field is not None and not strict else None
    i = 1
    while i < len(out) - 1:
        p, u, w = out[i], out[i - 1].yaw, out[i].yaw
        into, onto = infos[own[i]], infos[own[i + 1]]  # (the ways to and from it)
        k = own[i + 1] if onto.kind == TRANSIT else own[i]
        if angle_diff(w, u) <= CORNER_ANGLE or angle_diff(w, u) > math.pi - 1e-3 or infos[k].kind != TRANSIT or \
                not infos[own[i - 1]].section == into.section == onto.section or into.reverse or onto.reverse or \
                (p.x, p.y) in route.sharp:
            i += 1
            continue
        # (into a loop, lane or turn next to it turn_radius at most, as much as a right angle of that takes: no more of
        # it left out)
        behind, ahead = leg(i, u, -1, True), leg(i, w, 1, True)
        back = min(leg(i, u, -1), behind + route.cfg.turn_radius)
        on = min(leg(i, w, 1), ahead + route.cfg.turn_radius)
        # (with headland_turns out of the lanes' field where the corner is, with headland_corners 'rounded' into it
        # where nothing else fits, no S then)
        clear, here = route.clear([(p.x, p.y)]), []
        found = next(((got, arc, out_of) for out_of in ([True, False] if clear and not strict else [clear])
                      for r in (f * route.cfg.turn_radius for f in SMOOTH_RADII) if r >= 1e-3
                      for a, b, arc in ways(p, u, w, r, out_of or not clear) for got in [fit(a, b, arc, out_of)]
                      if got), None)
        if found is None:
            i += 1
            continue
        (s, e, new), arc, out_of = found
        if clear and not out_of:
            route.rounded.update((q.x, q.y) for q in arc)
        # (the way to the arc's start is the piece's it lies on, the way on from its end too)
        mid = [k] * (len(new) - 2) if same_position(out[s], arc[0]) else [own[s + 1]] + [k] * (len(new) - 3)
        own[s:e + 1] = [own[s]] + mid + [k if same_position(arc[-1], out[e]) else own[e]]
        out[s:e + 1] = new
        i = s + len(new) - 1
    lo: Dict[int, int] = {}
    hi: Dict[int, int] = {}
    for j, k in enumerate(own):
        lo.setdefault(k, j)
        hi[k] = j
    kept = []
    for k, info in enumerate(infos):
        if k in lo:
            start = lo[k] - shared[k]
            if hi[k] > start:
                kept.append(dataclasses.replace(info, start=start, end=hi[k]))
    return out, kept


def _mop_up(route: '_Route', poses: List[Pose2D], infos: List[SegmentInfo],
            free) -> Tuple[List[Pose2D], List[SegmentInfo], Dict[str, Any]]:
    """mop_up: of what the path leaves unmowed of free the pieces of MOP_MIN at least (the biggest MOP_MAX), each with a
    pass of MOP_PASS where the body fits that mows the most of it (MOP_GAIN of MOP_MIN at least); from the path's end a
    drive to the nearest next (through the transit space from where it is, the body checked) and its pass, then back to
    where the path ended. Returns the poses, the segments (drives TRANSIT, passes LANE, 'mop_up') and {pieces, area: m2
    mowed of them}."""
    cfg, fp = route.cfg, route.footprint
    starts = [k for k, info in enumerate(infos) if k == 0 or info.section != infos[k - 1].section]
    bounds = [infos[k].start for k in starts] + [len(poses)]
    mowed = route.blade(*[poses[a:b] for a, b in zip(bounds, bounds[1:]) if b - a > 1])
    pieces = sorted((p for p in polygons(subtract(free, [mowed])) if p.area >= MOP_MIN), key=lambda p: -p.area)
    passes = []
    for piece in pieces[:MOP_MAX]:
        best = None
        for q in (piece.representative_point(), piece.centroid):
            for k in range(MOP_HEADINGS):
                h = 2.0 * math.pi * k / MOP_HEADINGS
                c, s = math.cos(h), math.sin(h)
                # (the mower's centre with the blade's on q, the pass's middle)
                x = q.x - cfg.blade_ahead * c + cfg.blade_offset * s
                y = q.y - cfg.blade_ahead * s - cfg.blade_offset * c
                d = 0.5 * MOP_PASS
                a, b = (x - d * c, y - d * s), (x + d * c, y + d * s)
                if not route.transit.is_free([a, b]):
                    continue
                way = sample_straight(a[0], a[1], b[0], b[1], h, route.step)
                if not fp.poses_fit(way):
                    continue
                gain = piece.intersection(route.blade(way)).area
                if best is None or gain > best[0]:
                    best = (gain, way)
        if best is not None and best[0] >= MOP_GAIN * MOP_MIN:
            passes.append(best)
    out, new, section = list(poses), list(infos), infos[-1].section if infos else 0
    end, area, done = poses[-1], 0.0, 0

    def leave(way: Optional[List[Pose2D]], at: Pose2D) -> Optional[List[Pose2D]]:
        # a drive from where the mower is: turning on the spot first where it doesn't leave the way it faces
        if not way or not same_position(way[0], at):
            return None
        if len(way) > 1 and not same_position(way[0], way[1]):
            yaw = math.atan2(way[1].y - way[0].y, way[1].x - way[0].x)
            if angle_diff(yaw, way[0].yaw) > 1e-6:
                way = [way[0], Pose2D(way[0].x, way[0].y, yaw)] + way[1:]
        return way

    def add(kind: str, way: List[Pose2D]) -> None:
        merged = join([out[-1]], way)
        if len(merged) > 1:
            new.append(SegmentInfo(kind, len(out) - 1, len(out) + len(merged) - 2, False, 'mop_up', section))
            out[-1] = merged[0]  # (facing the way it leaves: turning on the spot there)
            out.extend(merged[1:])

    while passes:
        here = out[-1]
        passes.sort(key=lambda g: math.hypot(g[1][0].x - here.x, g[1][0].y - here.y))
        gain, way = passes.pop(0)
        drive = leave(route.drive_to((way[0].x, way[0].y), way[0].yaw, None, route.transit,
                                     ((here.x, here.y), here.yaw)), here)
        # (the body checked from the pose before it leaves on, turning on the spot where they meet too: one pose on a
        # position, the later)
        if not drive or not fp.poses_fit(out[-2:-1] + drive + way[1:]):
            continue
        add(TRANSIT, drive)
        add(LANE, way)
        area, done = area + gain, done + 1
    if done:
        back = leave(route.drive_to((end.x, end.y), None, None, route.transit, ((out[-1].x, out[-1].y), out[-1].yaw)),
                     out[-1])
        if back and fp.poses_fit(out[-2:-1] + back):
            add(TRANSIT, back)
    return out, new, {'pieces': done, 'area': round(area, 3)}


def _turn_at(route: '_Route', poses: List[Pose2D],
             infos: List[SegmentInfo]) -> Tuple[List[Pose2D], List[SegmentInfo]]:
    """Where the path turns on the spot (the heading jumping by more than footprint.SPIN, not backing up) and the way in
    is longer than twice TURN_AHEAD, a pose TURN_AHEAD before the point on it, facing the way it came (where the body
    fits so)."""
    back = {i for info in infos if info.reverse for i in range(info.start, info.end + 1)}
    first = {info.start for k, info in enumerate(infos) if k == 0 or infos[k - 1].section != info.section}
    out, at = [], []  # (at: where the poses are put in, by the index of the one after)
    for i, p in enumerate(poses):
        q = poses[i - 1] if i else p
        d = math.hypot(p.x - q.x, p.y - q.y)
        if i not in first and angle_diff(p.yaw, q.yaw) > SPIN and d > 2.0 * TURN_AHEAD and not {i - 1, i} & back:
            f = TURN_AHEAD / d
            ahead = Pose2D(p.x + f * (q.x - p.x), p.y + f * (q.y - p.y), q.yaw)
            if route.body([ahead]):
                out.append(ahead)
                at.append(i)
        out.append(p)

    def moved(j: int) -> int:
        return j + bisect.bisect_right(at, j)

    return out, [dataclasses.replace(info, start=moved(info.start), end=moved(info.end)) for info in infos]


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
                  margins: Sequence[Optional[float]] = ()) -> CoveragePlan:
    """Plans the coverage path of one area, see _plan_coverage. With lane_spacing_mode 'auto' the best of
    SPACING_CANDIDATES lane spacings (see PlannerConfig), the same input always picks the same. stats['chosen']: the
    lane spacing and loops it was planned with."""
    cfg = config or PlannerConfig()
    cfg.validate()
    if cfg.lane_spacing_mode != 'auto':
        return _plan_coverage(boundary, obstacles, cfg, start, end, drivable, keep_out, margins)
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


def _left_alone(free, rings_of: List[object], passes, half: float) -> float:
    """(for lane_spacing_mode 'auto') m2 the loops (the rings of rings_of) and the lanes alone leave, no turns, no
    drives: quick to compare."""
    bands = [LineString(ring + [ring[0]]).buffer(half, 4) for geom in rings_of for _, ring in rings(geom)]
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
                   probe: Any = False) -> CoveragePlan:
    """Plans the coverage path of one area.

    boundary: [(x, y), ...] of the outline, obstacles: a list of those for no-go zones, all in metres in
    one flat frame (map). start: (x, y) or (x, y, yaw) of the mower, the path starts on the loop (or
    lane) closest to it (default: the outline's first point). end: (x, y) to finish close to, e.g. the
    docking station. drivable: areas the mower may drive through between the parts of this one but doesn't
    mow (navigation areas, other lawns). keep_out: where nothing goes on the way through those (inactive areas, ones
    not to mow), with the body checked recorded like the obstacles (see edges). margins: the obstacle_margin of each
    obstacle, then of each area to keep out of (None, or none given: the setting). Raises PlanningError when there's
    nothing to plan, or a setting or point is out of range (every coordinate within MAX_COORDINATE of the origin).
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
            # (opened by a hair: GEOS can leave a spike without area there, the first row of lanes would go by it)
            wide = polygonal(subtract(lane_region, cut).buffer(-1e-6, 1, join_style=2).buffer(1e-6, 1, join_style=2))
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

    # recorded edges are where the centre drove, give or take body_tolerance: with the body checked the centre may go
    # that far past the first loop as well (rounding a corner, a turn, a loop out), the body keeping the margins. The
    # loops and lanes still keep to robot_space
    drive_space = robot_space
    if cfg.body and cfg.body_tolerance > 0:
        drive_space = polygonal(robot_space.buffer(cfg.body_tolerance, QUAD_SEGS))

    # turns and loops stay inside the mowing area, so do the bends of lanes around obstacles
    edges = dict(edge_distance=cfg.transit_edge_distance, edge_cost=cfg.transit_edge_cost)
    area_space = TransitPlanner(drive_space, cfg.transit_clearance, **edges)

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
    if probe is True:
        return _left_alone(free, levels + middles + [narrow], passes, half)
    if isinstance(probe, dict):  # (what _left_alone needs: plan_coverage may ask for this spacing's later)
        probe['alone'] = (free, levels + middles + [narrow], passes, half)

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
                                 loop_spacing, last or not any(w['cells']),
                                 not concentric or cfg.narrow_parts == 'loops')
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
    if not cfg.turn_on_spot:
        _smooth_spins(route, cfg)
    poses, segments = _assemble(route.segments)
    if footprint is not None:
        poses, segments = _round_corners(route, poses, segments)
    took('route')
    fitted, parts = None, len({s.section for s in segments})
    if footprint is not None and cfg.body_fit:
        poses, segments, fitted = _fit_body(route, poses, segments)
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
    turning, arcs, deepest = _field_turns(route, poses, segments) if route.field is not None else ([], 0, 0.0)
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
    for key in ('turns_u_turn', 'turns_bulb', 'turns_k_turn', 'turns_detour', 'turns_pivot', 'turns_fallback'):
        stats[key] = route.counts.get(key, 0)
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


def _keep(kept: 'collections.OrderedDict', key: str, value: Any) -> None:
    """value kept for key, the last KEPT only."""
    with _KEPT_LOCK:
        kept[key] = value
        kept.move_to_end(key)
        while len(kept) > KEPT:
            kept.popitem(last=False)


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


def _pieces(pts: List[XY], n: int, cut: float) -> List[List[XY]]:
    """pts in pieces of at least n points, cut in the middle of a straight piece at least `cut` long."""
    out, cur = [], [pts[0]]
    for p, q in zip(pts, pts[1:]):
        if len(cur) >= n and math.hypot(q[0] - p[0], q[1] - p[1]) >= cut:
            m = (0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1]))
            out.append(cur + [m])
            cur = [m]
        cur.append(q)
    out.append(cur)
    return out
