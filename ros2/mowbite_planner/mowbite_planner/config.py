"""The planner's settings (PlannerConfig), what a plan is made of (PathSegment, SegmentInfo, CoveragePlan) and
the constants the planner's modules share."""

import collections
import dataclasses
import math
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple

from shapely.geometry import Point, Polygon

from .mathutil import Pose2D


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
PERIMETER_DIRECTIONS = ('auto', 'ccw', 'cw', 'random')
TURN_TYPES = ('u_turn', 'bulb', 'detour', 'pivot')
# m, with turn_radius 0 and the body checked every curve but the turns at the lanes' ends (the default turn_radius)
ZERO_TURN = 0.25
# 'recorded': the edges are where the mower's centre drove when they were recorded, 'hard': a wall
EDGES = ('recorded', 'hard')
# headland_turns: the drives along the headland 'rounded' where an arc fits at their corners, or 'sharp' (on the spot)
HEADLAND_CORNERS = ('rounded', 'sharp')
# lanes this close to the edge of their region count as on it, keeps a lane along a straight edge
ON_EDGE = 1e-4
MAX_ROWS = 20000
# rad, between the angles 'optimal' tries at least
MIN_ANGLE_STEP = math.radians(0.5)
# loops along the edge (OpenMower's outline_count is a uint8), they stop anyway where nothing is left
MAX_PASSES = 255
# m, wider turns are a mistake, and planning takes longer the wider they are
MAX_TURN_RADIUS = 10.0
# rad, smooth_spins replaces turns on the spot by more than this, with an omega of these shares of turn_radius
SPIN_LIMIT = math.radians(60.0)
SMOOTH_RADII = (1.0, 0.75, 0.5, 0.35)
# m2, lane_spacing_mode 'auto': a plan leaves nothing unmowed with no piece this big and not LEFT_TOTAL together of
# pieces from LEFT_TUFT on (tufts at corners), like the report
LEFT_TUFT = 0.01
LEFT_PIECE = 0.05
LEFT_TOTAL = 0.1
# m, a blade further off the mower's centre is a mistake
MAX_BLADE_OFFSET = 1.0
# m, the body reaches this far from base_link at most (a size in mm is caught, not checked on a grid of that size)
MAX_BODY = 5.0
# segments of a rounded corner of the body (robot_front_width, robot_rear_width): a quarter circle
BODY_ROUND_SEGS = 6
# m, where the only turn between two lanes is on the spot (or a drive), they end this much earlier at a time for a
# rounded one (_Route.round_earlier), m2 of what they leave its blade may not mow
ROUND_STEP = 0.05
EARLIER_LEFT = 1e-3
# m, with the body checked: where the path goes another way at a place and leaves LOOP_LEFT m or more of a loop out
# there (round a bend of the edge the front swings out, and no short way fits), that stretch of the loop runs further in
# instead: a step of LOOP_IN[0] at a time, LOOP_IN[1] at most (the next loop mows what's further in), getting there and
# back over LOOP_EASE m, the least and the shortest that fit. A stretch on its own, else with the next LOOP_JOIN ones
# close after it, else with all of those; LOOP_TRIES tried at most for a loop (_Route.loop_in). Where a stretch there
# still sticks out (a corner too tight for the body, its front swinging out further than LOOP_IN[1]), Dubins' way of
# LOOP_ROUND m (the widest that fits) in its place from the loop's way LOOP_ROUND_REACH m before it at most onto its way
# as far after it, the closest first: the corner rounded off, the loop on along the edge on both sides (stretches less
# than LOOP_ROUND_GAP m apart one, LOOP_ROUND_TRIES tried at most for a stretch, _Route.loop_round: a sharp corner
# where an edge meets a bed only fits further off it than the closest few hundred ways). Kept where the path then
# leaves LOOP_GAIN m2 less unmowed
LOOP_LEFT = 1.5
LOOP_GAIN = 0.05
LOOP_IN = (0.02, 0.1)
LOOP_EASE = (0.5, 1.0, 2.0)
LOOP_JOIN = 3
LOOP_TRIES = 300
LOOP_ROUND = (1.0, 0.75, 0.5, 0.35, 0.25)
LOOP_ROUND_REACH = 3.0
LOOP_ROUND_GAP = 1.0
LOOP_ROUND_TRIES = 6000
# m, with the body checked: where the path goes another way where it gets onto a loop or off it (the body would stick
# out turning on the spot there, by an obstacle) and leaves the loop's start or end out for that, the loop goes on past
# its start by what's left out at both ends and LOOP_ON more: the blade mows at its end what's left out at its start,
# and at its start what's left out at its end (_Route.loops_on). Kept where the path then leaves LOOP_GAIN m2 less
# unmowed
LOOP_ON = 0.5
# the further ways of bodyfit's _fit_body (and the way in to the plan's start, see route's START_CLEAR) get off the path
# and back on it easing off and onto its line: t m along it FIT_EASE_OFF m * (e^(t / l) - 1) off it at most, l
# FIT_EASE times how far the body reaches behind the centre getting off, ahead of it getting back on (the rear, the
# front swinging out FIT_EASE_OFF m at most)
FIT_EASE = 1.5
FIT_EASE_OFF = 0.01
# m, lane_overlap_limit: how much further out than the lanes reach a turn between them may get (a sampled arc)
TURN_REACH = 0.02
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


class NothingLeft(PlanningError):
    """Of what a mow left (plan_coverage's mowed) the mower gets to nothing."""


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
    # loops round the obstacles that lie in the lawn like islands, -1: as many as along the outline. 0: none, the lanes
    # go up to them. Not with fill_pattern 'concentric'. (An obstacle at the edge is part of the outline)
    obstacle_passes: int = -1
    fill_pattern: str = 'lanes'  # 'lanes', 'crosshatch' or 'concentric'
    crosshatch_angle: float = math.pi / 2  # rad, direction of crosshatch's second lanes to the first
    perimeter_order: str = 'first'  # loops 'first' or 'last' (smooths the marks the turns leave)
    # 'ccw', 'cw' around every loop, 'auto': the way the mower faces, 'random': ccw or cw at random for each count of
    # finished_mows (another way round every mow, all of a plan asked for again before the next one finishes the same).
    # With robot_outline (auto, random) the outermost loop along the outline goes the way it was recorded (the way its
    # points go): the body only where it was then, see edges
    perimeter_direction: str = 'auto'
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
    finished_mows: int = 0  # all of them so far, the ROS 1 node counts them (see perimeter_direction)
    # m, the first perimeter loop runs this far inside the outline. None: mower_width / 2, the blade's edge
    # on the outline. OpenMower records outlines driving the mower's centre along the edge and uses 0,
    # negative goes outside the outline (OpenMower's outline_offset allows that too)
    perimeter_offset: Optional[float] = None
    # m, radius of the turns between lanes, and no arc of a turn, a drive or a loop out is tighter (min_turn_radius):
    # where none fits the drives turn on the spot instead (with the body checked a drive's corner gets a tighter arc
    # where that fits, see _round_corners), a turn between lanes is rounded as wide as fits (at a slant where the lanes
    # end at one, the lanes ending a little earlier for it where that's what it takes, else a loop into the next lane,
    # see _Route.round_earlier), on the spot only where nothing does, a drive into a lane ending on a way into it where
    # one fits (the lane starting later for it where that leaves nothing, see _Route.into_lane). 0 with the body
    # checked: on the spot at the lanes' ends, lane after lane (turn_on_spot, lane_order snake), every other curve
    # ZERO_TURN wide (all on the spot, a long front by the edges wouldn't fit: whole loops left out)
    turn_radius: float = 0.25
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
    # arriving on the spot rather than into a lane round a loop or from beside it, or on arcs that cross its own way or
    # turn it round more than half a turn. The body fits turning like everywhere
    turn_on_spot: bool = False
    # the planner picks every turn's radius, turn_radius the least: a wider one where the lanes are further apart or
    # there's room (WIDER_TURNS of turn_radius, the widest that fits), the lanes ending earlier for a turn of it where
    # that leaves nothing unmowed, and one tighter than it (a half circle as wide as the lanes are apart) only where
    # that doesn't fit either, before turning on the spot (off: that half circle before the lanes end earlier)
    turn_radius_auto: bool = False
    # zones with a lane direction of their own: where the obstacles cut the lanes into groups, a group mowed another way
    # where that takes a lot fewer lanes (see lanes.zone_angles), each zone after the rest, the nearest next. Not with
    # fill_pattern crosshatch or concentric, nor with headland_turns
    split_zones: bool = False
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
    # m, transits keep this far from the outer loop and obstacle loops where that's not a lot longer (else
    # transit's NARROW_CLEARANCE)
    transit_clearance: float = 0.15
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
    # the mower gets no further out on the lanes than they reach. The turns between them stay in there, or where
    # turning on the spot would take it (_Route.in_reach): a rounded one where it fits, with the lanes ending earlier
    # where the loops mowed, else on the spot; one that takes its room only where neither fits. And a lane starts
    # blade_ahead later where the loops mowed that (_Route.start_in_reach). Off: the turns take the room there is in
    # the headland
    lane_overlap_limit: bool = False
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
    # m, the body's width right at its front and its back where it's rounded there: the rectangle's corners there a
    # quarter circle each, of half what that is narrower than robot_width. None: square corners
    robot_front_width: Optional[float] = None
    robot_rear_width: Optional[float] = None
    robot_outline: Optional[Tuple[XY, ...]] = None
    # 'recorded': the outline, obstacles and areas to keep out of are where the centre drove, with the body checked the
    # body may be where it was while they were recorded (half its width each side, the front ahead and the rear
    # behind, turning on the spot at every point), their real edges. 'hard': they're walls (lines drawn on the real
    # edge, a bed's border), the centre keeps half the body's width off them (the first loop moves further in where
    # perimeter_offset would come closer), with the body checked the body keeps the margins off the lines themselves
    edges: str = 'recorded'
    # m, with the body checked a turn, a loop out or a drive may take the centre this much further out than the first
    # loop (a recorded edge is a few cm off where the mower was), the body still keeping the margins
    body_tolerance: float = 0.05
    # m, with the body checked: how far it keeps off the outline's real edge everywhere, obstacle_margin off an
    # obstacle's and an area's to keep out of (one's own margin beats it, see plan_coverage). The loops keep the centre
    # that far off the lines: the first one further in where perimeter_offset would come closer, the ones round an
    # obstacle further out (round both of two too close for the body to get through between, or round one too close to
    # the edge, _gaps)
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
    def island_passes(self) -> int:
        """The loops round the islands: obstacle_passes, -1 as many as along the outline (passes)."""
        return self.passes if self.obstacle_passes < 0 or self.fill_pattern == 'concentric' else \
            int(self.obstacle_passes)

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
        simplify_tolerance: the simplified line may lie that much further out; hard edges: half the body's width more)
        or the body (robot_width, hard edges) wouldn't fit there."""
        if self.body:
            keep = self.edge_margin + self.simplify_tolerance + self.wall_half
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
    def wall_half(self) -> float:
        """m, with the body checked and hard edges half the body's width (the lines are walls: the loops keep the
        centre that much further off them than the margins), else 0."""
        if not self.body or self.edges != 'hard':
            return 0.0
        return max(abs(y) for _, y in self.body_outline)

    @property
    def no_body(self) -> Optional[str]:
        """What a plan warns of where the body isn't checked (no size set, or only a part of it), None where it
        is."""
        if self.body:
            return None
        sizes = ('robot_width', 'robot_front', 'robot_rear')
        got = [n for n in sizes if getattr(self, n) is not None]
        if got:
            return (f'no body check: {", ".join(got)} set but not {", ".join(n for n in sizes if n not in got)} (it '
                    'takes all three, or robot_outline)')
        return 'no body check: no mower size set (robot_width, robot_front and robot_rear, or robot_outline)'

    @property
    def body_outline(self) -> Tuple[XY, ...]:
        """The body's corners counter-clockwise: robot_outline (a point given twice in a row once), or the rectangle
        of robot_width, robot_front and robot_rear (its corners rounded, robot_front_width, robot_rear_width)."""
        if self.robot_outline is None:
            return _rounded_box(self.robot_width, self.robot_front, self.robot_rear, self.robot_front_width,
                                self.robot_rear_width)
        pts = [(float(x), float(y)) for x, y in self.robot_outline]
        pts = [p for i, p in enumerate(pts) if p != pts[i - 1]]
        return tuple(pts if Polygon(pts).exterior.is_ccw else pts[::-1])

    @property
    def min_turn_radius(self) -> float:
        """m, no arc tighter than this: the turns' radius (one for all)."""
        return self.turn_radius

    @property
    def allowed_turns(self) -> Tuple[str, ...]:
        return tuple(t for t in self.turn_types if t != 'bulb' or not self.turn_on_spot)

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
        for name, lo in (('angle_increment_every', 1), ('angle_steps', 0), ('finished_mows', 0)):
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
        for name in ('robot_width', 'robot_front', 'robot_rear', 'robot_front_width', 'robot_rear_width'):
            if getattr(self, name) is not None:
                finite(name, 0.0, MAX_BODY if name in ('robot_front', 'robot_rear') else 2.0 * MAX_BODY)
        for name in ('robot_front_width', 'robot_rear_width'):
            if getattr(self, name) is not None and self.robot_width is not None and getattr(self, name) > \
                    self.robot_width:
                raise PlanningError(f'{name} ({getattr(self, name)} m) can\'t be more than robot_width '
                                    f'({self.robot_width} m)')
        if self.robot_outline is not None:
            try:
                pts = [(float(x), float(y)) for x, y in self.robot_outline
                       if not isinstance(x, (bool, str)) and not isinstance(y, (bool, str))]
                ok = len(pts) == len(self.robot_outline) >= 3 and all(abs(c) <= MAX_BODY for p in pts for c in p)
            except (TypeError, ValueError, OverflowError):
                ok = False
            shape = Polygon(pts) if ok else None
            if shape is None or not shape.is_valid or shape.area <= 0.0 or not shape.covers(Point(0.0, 0.0)):
                raise PlanningError('robot_outline must be [[x, y], ...] (m, x ahead of base_link, y to the left), 3 '
                                    f'points at least round base_link, none further than {MAX_BODY} m from it, the '
                                    f'outline not crossing itself, got {self.robot_outline!r}')
        if self.body and not self.waypoint_spacing > 0.0:
            raise PlanningError('waypoint_spacing must be > 0 with the body checked (robot_width, ...)')
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
        for name, lo in (('perimeter_passes', -1), ('obstacle_passes', -1), ('lane_overlap_passes', 0)):
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
                any(t not in TURN_TYPES for t in self.turn_types):
            raise PlanningError(f'turn_types must be some of {", ".join(TURN_TYPES)}, got {self.turn_types!r}')
        for name in ('turn_radius_auto', 'split_zones', 'skip_outline_loops', 'skip_obstacle_loops', 'skip_lanes', 'nested_areas',
                     'turn_on_spot', 'headland_turns', 'body_fit', 'mop_up', 'lane_overlap_limit'):
            if not isinstance(getattr(self, name), bool):
                raise PlanningError(f'{name} must be true or false, got {getattr(self, name)!r}')


# fixed, no settings (with_overrides leaves them out): how the planner works inside
INTERNAL = ('waypoint_spacing', 'simplify_tolerance', 'min_lane_length', 'transit_clearance',
            'body_tolerance', 'perimeter_corner_radius')
# settings (and a turn type) of before, with_overrides leaves them out: min_turn_radius is turn_radius, smooth_spins
# not turn_on_spot, allow_reverse and k_turn (backing up) gone: OpenMower's FTC drives forwards
RETIRED = ('min_turn_radius', 'smooth_spins', 'allow_reverse', 'k_turn')
# settings where None means something (worked out, mower_width / 2), and the ones that take a float
_OPTIONAL = ('angle', 'angle_min', 'angle_max', 'perimeter_offset', 'robot_width', 'robot_front', 'robot_rear',
             'robot_front_width', 'robot_rear_width', 'robot_outline', 'lane_spacing_min', 'lane_spacing_max')
_FLOATS = tuple(f.name for f in dataclasses.fields(PlannerConfig) if isinstance(f.default, float)) + _OPTIONAL


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


@dataclass
class PathSegment:
    kind: str  # PERIMETER, LANE, TURN or TRANSIT
    poses: List[Pose2D]
    reverse: bool = False  # driven backwards: never (OpenMower's segment_reverse)
    # turns: 'u_turn', 'bulb', 'detour', 'pivot'; transits replacing a turn: 'turn_fallback'
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


def _rounded_box(width: float, front: float, rear: float, front_width: Optional[float],
                 rear_width: Optional[float]) -> Tuple[XY, ...]:
    """The rectangle width wide, front ahead of the centre and rear behind it, counter-clockwise, its front corners
    a quarter circle of half what front_width is narrower than width, its back ones of rear_width's (None: square).
    Each a polygon round the circle (BODY_ROUND_SEGS sides on it): the body no smaller than it says."""
    half, length = 0.5 * width, front + rear

    def radius(w: Optional[float], room: float) -> float:
        return 0.0 if w is None else max(0.0, min(half - 0.5 * w, room))

    rf = radius(front_width, length)
    rr = radius(rear_width, length - rf)
    step = 0.5 * math.pi / BODY_ROUND_SEGS
    out: List[XY] = []
    for cx, cy, r, a0 in ((front - rf, half - rf, rf, 0.0), (rr - rear, half - rr, rr, 0.5 * math.pi),
                          (rr - rear, rr - half, rr, math.pi), (front - rf, rf - half, rf, 1.5 * math.pi)):
        if r <= 0.0:
            out.append((cx, cy))
            continue
        # where the sides touch the circle and, between, the corners of the sides round it
        out.append((cx + r * math.cos(a0), cy + r * math.sin(a0)))
        out += [(cx + r / math.cos(0.5 * step) * math.cos(a0 + (k + 0.5) * step),
                 cy + r / math.cos(0.5 * step) * math.sin(a0 + (k + 0.5) * step)) for k in range(BODY_ROUND_SEGS)]
        out.append((cx + r * math.cos(a0 + 0.5 * math.pi), cy + r * math.sin(a0 + 0.5 * math.pi)))
    pts = [(round(x, 9), round(y, 9)) for x, y in out]
    return tuple(p for i, p in enumerate(pts) if p != pts[i - 1])


def _keep(kept: 'collections.OrderedDict', key: str, value: Any) -> None:
    """value kept for key, the last KEPT only."""
    with _KEPT_LOCK:
        kept[key] = value
        kept.move_to_end(key)
        while len(kept) > KEPT:
            kept.popitem(last=False)
