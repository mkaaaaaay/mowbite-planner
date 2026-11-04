"""The MowBite Planner as a ROS 2 node.

Services
  ~/plan_coverage (mowbite_planner_interfaces/srv/PlanCoverage)
      boundary + obstacles as geometry_msgs/Polygon in, the route as nav_msgs/Path out (plus where its
      perimeter loops, lanes, turns and transits are). Any of the node's parameters can be overridden per
      request (overrides).
  ~/plan_from_parameters (std_srvs/srv/Trigger)
      plans the area in the parameters boundary_polygon and obstacles, the result goes out on the topic.

Topic
  ~/coverage_path (nav_msgs/msg/Path), transient local: the last plan, also for subscribers that come
  later (RViz: set the Path display's durability to Transient Local).

Planning is plain Python in the service callback. On the default single threaded executor requests are
handled one after another. Most of the time goes into the messages, about 20 us a pose: a garden of
300 square metres (18000 poses) takes about half a second on a PC, a CM4 needs four to six times as long.

With the parameters rpc or use_map the node is also mowbite_planner.core's PlannerCore (NodeCore here, the ROS 1 node's
logic: the planner.* settings and RPC methods, the plans kept and planned ahead, what follows a mow):
  - rpc: the planner.* methods (planner.settings, .settings.set, .angle.reset, .log, .area.set, .plan, .leftovers,
    .drive; what each answers: see the ROS 1 node's top) as JSON {method, params, id} on ~/rpc/request, the answer
    {id, result} or {id, error: {code, message}} on ~/rpc/response (JSON-RPC's codes). planner.area.set, .plan,
    .leftovers and .drive on a thread of their own each, the others one after another on one.
  - the map: ~/map (std_msgs/String, transient local), OpenMower's JSON. Every value a plan goes by has one place: for
    all areas the planner.* settings (the node's parameters their defaults), the lane spacing, the loops and the angle
    too (no mower_logic here); for an area its properties in the map, OpenMower's where OpenMower has one (angle,
    outline_count, outline_overlap_count, outline_offset) and MowBite's angle_min, angle_max, the others in its
    planner property. planner.area.set writes each to its one place (an area's "the planner works it out" into its
    planner property, OpenMower's away); both there all the same: OpenMower's counts (NodeCore.plan_map).
  - use_map: ~/plan_coverage of an area of the map (its outline as boundary, to 1 mm) is planned like planner.plan
    plans it: the planner.* settings, the area's own over them, of an area's what the request sets over those (a
    preview's edited ones: perimeter_passes, angle, the overrides lane_overlap_passes and perimeter_offset),
    angle_offset_deg on top of the angle (a job's own); not the request's lane spacing (mower_width, overlap: the
    planner's, a warning where it's another). The obstacles, the areas to mow around, the other lawns and the
    navigation areas from the map, the start and the end from the request or else the dock. An outline of no area
    (a preview) as well, its obstacles and drivable areas the request's, with no dock. The same map, settings and
    request give the same plan (kept); armed leftovers (planner.leftovers arm) are the plan of their area until that
    job ends, an area without them gets success with no poses. Planned in the core's worker, before what waits there.
    Without a map as ever.
  - the events of the robot's logic on ~/events (STATE, JOB_COMPLETE, JOB_RESET as OpenMower's events/json):
    angle_increment, perimeter_direction random, the finished mow's planner.drive and timing.
  - the planning's progress (mowbite/planner/progress, see the ROS 1 node's top) as JSON {topic, payload, retain} on
    ~/publish.
  - data_dir: data_dir/planner keeps the settings, plans, angle steps, timing and the maps before (plans and maps
    before older than keep_days or past keep_mb go), data_dir/position_history is the track planner.drive and
    planner.leftovers read. Empty: nothing kept.
  - not here: the slic3r planner, leftovers_auto (the node doesn't send the robot out), the house computer
    (MowbitePlannerNode.cannot: planner.settings says why).
A robot's own node is a class of MowbitePlannerNode: its parameters' defaults, the settings it can't have, its own RPC
methods, its transport (rpc_input, answer, rpc_end, map_input, events_input, send_out), its parameters (robot_param),
writing the map (replace_map), when the map can't be planned (map_unusable), how it gets to a plan's sections
(arrives), drive slope and reverse it can't drive (drive_ignored), backing up out of dead ends it can't drive
(back_out_ignored, by default as drive); main(node_class=it). SIGINT and SIGTERM end it
with destroy_node first (see main).
"""

import glob
import hashlib
import json
import logging
import math
import os
import queue
import re
import signal
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from mowbite_planner_interfaces.srv import PlanCoverage

from .core import (AUTO_VALUES, HISTORY_DIR, LOG, METHOD_NOT_FOUND, MOWER, MOWER_LOGIC, OPENMOWER_PROPS, REQUEST,
                   PlannerCore)
from .core import call as core_call
from .export import ANSWER_TOO_BIG, fingerprint, too_big
from .mapfile import AreaJob, _number, area_job, margins_for, match_area
from .mathutil import Pose2D, quaternion_to_yaw, yaw_to_quaternion
from .config import (ANGLE_STRATEGIES, EDGES, FILL_PATTERNS, HEADLAND_CORNERS, LANE, LANE_ORDERS, MAX_PASSES,
                     NARROW_PARTS, PERIMETER, PERIMETER_DIRECTIONS, PERIMETER_ORDERS, ROUTE_ORDERS, TRANSIT, TURN,
                     TURN_TYPES, CoveragePlan, PlannerConfig, PlanningError, out_kinds)
from .mqtt_service import _float32
from .planner import plan_coverage

XY = Tuple[float, float]

SEGMENT_TYPES = {
    PERIMETER: PlanCoverage.Response.SEGMENT_PERIMETER,
    LANE: PlanCoverage.Response.SEGMENT_LANE,
    TURN: PlanCoverage.Response.SEGMENT_TURN,
    TRANSIT: PlanCoverage.Response.SEGMENT_TRANSIT,
}

# name: (default, description, check, why it fails). Doubles are written with a decimal point, a YAML file
# has to do the same (rclpy refuses an int for a double parameter)
PARAMETERS = {
    'frame_id': ('map', 'frame of the polygons and of the path', lambda v: bool(v), 'must not be empty'),
    'mower_width': (0.22, 'cutting width [m]', lambda v: v > 0.0, 'must be > 0'),
    'overlap': (0.15, 'share of the cutting width neighbouring lanes overlap, 0.15 = 15 %. Lane spacing = '
                'mower_width * (1 - overlap)', lambda v: 0.0 <= v < 1.0, 'must be 0 <= overlap < 1'),
    'perimeter_passes': (2, 'loops along the outline and around every obstacle, -1: as many as the turns of the '
                         'lanes need (from turn_radius, the body and blade_ahead)', lambda v: -1 <= v <= MAX_PASSES,
                         f'must be -1..{MAX_PASSES}'),
    'obstacle_passes': (-1, 'loops round the obstacles lying in the lawn like islands, -1: as many as along the '
                        'outline (perimeter_passes), 0: none', lambda v: -1 <= v <= MAX_PASSES,
                        f'must be -1..{MAX_PASSES}'),
    'perimeter_offset': (-1.0, 'distance of the first perimeter loop from the outline [m]. Negative: '
                         'mower_width / 2, the blade edge on the outline. 0 for outlines recorded by driving '
                         "the mower's centre along the edge (OpenMower's outline_offset)", math.isfinite,
                         'must be a number'),
    'robot_width': (-1.0, "width of the mower's body [m]. With robot_front and robot_rear the turns, lane ends, "
                    'loops and drives keep the body in (see edges). Negative: not set', math.isfinite,
                    'must be a number'),
    'robot_front': (-1.0, "how far the mower's body reaches ahead of its centre (base_link) [m], negative: not set",
                    math.isfinite, 'must be a number'),
    'robot_rear': (-1.0, "how far the mower's body reaches behind its centre [m], negative: not set", math.isfinite,
                   'must be a number'),
    'robot_outline': (Parameter.Type.DOUBLE_ARRAY, "the mower's body as its outline round its centre (x ahead, y to "
                      'the left) [m], [x0, y0, x1, y1, ...], in place of the rectangle of robot_width, robot_front and '
                      'robot_rear. Not set or empty: that rectangle', lambda v: not len(v) or outline_ok(v),
                      'must be x, y of 3 points at least round base_link, the outline not crossing itself'),
    'narrow_parts': ('lanes', 'where the lanes would lie in a part narrower than their turns need (a path): lanes '
                     'anyway, or loops further in, the pieces they come apart into driven on the way round (with '
                     'fill_pattern concentric all the loops)',
                     lambda v: v in NARROW_PARTS, 'must be lanes or loops'),
    'headland_turns': (False, "clean stripes: in the lanes' field the path only runs straight along the lanes, the "
                       'drives and turns go along the headland the loops mow (across the field only straight where '
                       'that way is more than twice as long or there is none)', None, ''),
    'headland_corners': ('rounded', 'with headland_turns and the body set: the drives along the headland rounded at '
                         'their corners where the body fits on an arc (into the field only where none fits out of it, '
                         '12 cm deep at most), or sharp: turning on the spot there', lambda v: v in HEADLAND_CORNERS,
                         'must be rounded or sharp'),
    'edges': ('recorded', 'recorded: the outline and obstacles are where the centre drove, the body may be where '
              'it was then. hard: walls (lines drawn on the real edge), the centre keeps half the body\'s width off '
              'them, with the body set (robot_width, robot_front, robot_rear) the body keeps the margins off the lines',
              lambda v: v in EDGES, 'must be recorded or hard'),
    'body_fit': (True, 'where the body would still stick out, the path goes another way there that fits, leaving a '
                 'little of a loop or lane out where it has to. False: the path as planned', None, ''),
    'mop_up': (True, 'with the body set, what the path leaves unmowed mowed where the body fits and it\'s worth it: '
               'short moves over it on detours from the loops and drives, the rest at the end', None, ''),
    'edge_margin': (0.05, "how far the body keeps off the outline's real edge [m] (where the body was while its "
                    'centre drove the line), everywhere. The first loop moves '
                    'further in for it', lambda v: v >= 0.0, 'must be >= 0'),
    'obstacle_margin': (0.10, "how far the body keeps off an obstacle's real edge [m], like edge_margin, the loops "
                        'round it further out for it', lambda v: v >= 0.0, 'must be >= 0'),
    'spin_margin': (0.05, 'how much further the body keeps off the edges turning on the spot and at a kink of more '
                    "than 15 deg [m], anywhere on the last 0.1 m of the way in (OpenMower's FTC turns over the way to "
                    'the point and drifts a few cm), and all along the turns between lanes. 0: only at the point, '
                    'the margins as everywhere',
                    lambda v: v >= 0.0, 'must be >= 0'),
    'blade_offset': (0.0, "how far the blade's centre sits left of the mower's centre [m], negative: right. The "
                     'loops go round with the blade on their outer side', math.isfinite, 'must be a number'),
    'blade_ahead': (0.0, "how far the blade's centre sits ahead of the mower's centre [m] (OpenMower's base_link "
                    'between the drive wheels), negative: behind. The lanes start and end that much earlier',
                    math.isfinite, 'must be a number'),
    'perimeter_order': ('first', 'perimeter loops first or last (last mows over the marks the turns leave)',
                        lambda v: v in PERIMETER_ORDERS, 'must be first or last'),
    'perimeter_direction': ('auto', 'perimeter loops ccw or cw, auto: the way the mower faces, random: ccw or cw at '
                            'random for each count of finished mows (the ROS 1 node counts them, here none: the '
                            'same way every plan). With the body set the outermost loop of auto and random goes the '
                            'way the outline was recorded', lambda v: v in PERIMETER_DIRECTIONS,
                            'must be auto, ccw, cw or random'),
    'lane_overlap_passes': (0, "lanes reach this many perimeter loops further out (OpenMower's "
                            'outline_overlap_count)', lambda v: 0 <= v <= MAX_PASSES, f'must be 0..{MAX_PASSES}'),
    'lane_overlap_limit': (False, 'the mower gets no further out on the lanes than they reach: the turns between '
                           'them on the spot where no rounded one fits in there, a lane starting later where the '
                           'loops mowed', None, ''),
    'fill_pattern': ('lanes', 'inside the loops: lanes, crosshatch (lanes, then lanes across them) or '
                     'concentric (loops further and further in)', lambda v: v in FILL_PATTERNS,
                     'must be lanes, crosshatch or concentric'),
    'crosshatch_angle_deg': (90.0, "direction of crosshatch's second lanes to the first [deg]", math.isfinite,
                             'must be a number'),
    'angle_strategy': ('longest_edge', 'lane direction: longest_edge (along the longest edge of the '
                       'boundary), min_width (across the narrowest width), optimal (fewest lanes and groups of '
                       'lanes, tried angle_step_deg apart) or fixed (fixed_angle_deg)',
                       lambda v: v in ANGLE_STRATEGIES + ('fixed',),
                       'must be longest_edge, min_width, optimal or fixed'),
    'fixed_angle_deg': (0.0, 'lane direction for angle_strategy fixed [deg], 0 = +x of the map frame, '
                        'counter-clockwise', math.isfinite, 'must be a number'),
    'angle_offset_deg': (0.0, 'added to the lane direction [deg], e.g. to change the stripes from mow to mow',
                         math.isfinite, 'must be a number'),
    'angle_min_deg': (math.nan, 'the lane direction stays within angle_min_deg..angle_max_deg [deg], set both '
                      'or neither, NaN: no range', lambda v: not math.isinf(v), 'must be a number or NaN'),
    'angle_max_deg': (math.nan, 'see angle_min_deg', lambda v: not math.isinf(v), 'must be a number or NaN'),
    'angle_step_deg': (5.0, 'between the lane directions optimal tries [deg]', lambda v: v > 0.0, 'must be > 0'),
    'turn_radius': (0.25, 'radius of the turns between lanes, no arc of a turn, a drive or a loop out is tighter [m]. '
                    'Where none fits a drive turns on the spot',
                    lambda v: v >= 0.0, 'must be >= 0'),
    'turn_on_spot': (False, 'turning on the spot where the path would drive a loop (for lawns that take it): no bulb '
                     'between lanes, a drive arriving on the spot rather than round a loop, the body fitting turning',
                     None, ''),
    'turn_radius_auto': (False, 'the planner picks every turn\'s radius, turn_radius the least: fewer turns tighter '
                         'than it, wider ones where there\'s room', None, ''),
    'split_zones': (False, 'a part of the lawn better mowed another way gets lanes of its own direction, where the '
                    'whole then takes fewer lanes', None, ''),
    'turn_types': (list(TURN_TYPES), 'turns allowed between lanes, preferred first: ' + ', '.join(TURN_TYPES),
                   lambda v: all(t in TURN_TYPES for t in v), 'must be some of ' + ', '.join(TURN_TYPES)),
    'lane_order': ('snake', 'snake: lane after neighbouring lane. skip: lanes far enough apart for a forward '
                   'U-turn of turn_radius, the ones between on the way back', lambda v: v in LANE_ORDERS,
                   'must be snake or skip'),
    'route_order': ('nearest', 'cells: nearest (the closest one not mowed yet next) or optimized (the order that '
                    'leaves the least unmowed, then takes the least time, takes a little longer to plan)',
                    lambda v: v in ROUTE_ORDERS,
                    'must be nearest or optimized'),
    'bend_max_gap': (0.0, 'a row split by an obstacle with at most this much between its pieces becomes one lane '
                     'bending around it [m], 0 = split', lambda v: v >= 0.0, 'must be >= 0'),
    'bend_max_offset': (0.5, "how far a bend may leave its lane's line [m]", lambda v: v >= 0.0, 'must be >= 0'),
    'transit_edge_distance': (0.0, 'transits keep this far off the edges where that costs little more [m], 0 = '
                              'the shortest way', lambda v: v >= 0.0, 'must be >= 0'),
    'transit_edge_cost': (2.0, 'every metre of a transit closer to the edge than transit_edge_distance counts this '
                          'much more', lambda v: v >= 0.0, 'must be >= 0'),
    'publish_path': (True, 'publish every plan on ~/coverage_path', None, ''),
    'rpc': (False, "the planner.* RPC methods of mowbite_planner.core (see the node's top)", None, ''),
    'use_map': (False, '~/plan_coverage of an area of the map planned like planner.plan plans it, with the '
                'planner.* settings (see the node\'s top)', None, ''),
    'data_dir': ('', 'with rpc or use_map: data_dir/planner keeps the planner.* settings, plans, angle steps, timing '
                 'and the maps before, data_dir/position_history is the track of the mows. Empty: nothing kept, no '
                 'track', None, ''),
    'keep_days': (60, 'plans kept and maps before older than this many days go [d]', lambda v: v >= 1,
                  'must be >= 1'),
    'keep_mb': (50.0, 'data_dir/planner takes this much at most, the oldest plans kept and maps before go first [MB]',
                lambda v: v > 0.0, 'must be > 0'),
    'workers': (0, 'with rpc or use_map: processes of their own that plan (the areas planned ahead at once too), 0: '
                'all in this one', lambda v: v >= 0, 'must be >= 0'),
}
RANGES = {
    'mower_width': (0.01, 5.0),
    'overlap': (0.0, 0.95),
    'perimeter_passes': (-1, MAX_PASSES),
    'obstacle_passes': (-1, MAX_PASSES),
    'lane_overlap_passes': (0, MAX_PASSES),
    'angle_step_deg': (0.5, 90.0),
    'turn_radius': (0.0, 5.0),
    'robot_width': (-1.0, 5.0),
    'robot_front': (-1.0, 5.0),
    'edge_margin': (0.0, 1.0),
    'obstacle_margin': (0.0, 1.0),
    'spin_margin': (0.0, 1.0),
    'robot_rear': (-1.0, 5.0),
    'blade_offset': (-1.0, 1.0),
    'blade_ahead': (-1.0, 1.0),
}
# m, float32 has steps of 1 mm or less up to here
FLOAT32_RANGE = 8192.0
# s between two looks at data_dir/planner's size (keep_days, keep_mb), s between two looks whether SIGINT or SIGTERM
# came (see main)
PRUNE_EVERY = 3600.0
STOP_POLL = 0.5
# what the node's parameters don't give the planner.* settings (the blade is PlannerCore's own, the rest no settings)
NOT_LAUNCH = ('mower_width', 'angle_steps', 'finished_mows')
# the overrides of a ~/plan_coverage request that count for an area of the map or an outline (see the top); m, the
# request's lane spacing off the planner's by more: a warning (it's left out)
AREA_OVERRIDES = ('lane_overlap_passes', 'perimeter_offset', 'angle_offset_deg')
SPACING_OFF = 0.001


def invalid(name: str, value) -> Optional[str]:
    """Why parameter name can't take this value, None if it can."""
    _, _, check, why = PARAMETERS[name]
    try:
        if check is not None and not check(value):
            return f'{name} {why}, got {value!r}'
        if name in RANGES and not RANGES[name][0] <= value <= RANGES[name][1]:
            return f'{name} must be {RANGES[name][0]}..{RANGES[name][1]}, got {value!r}'
    except TypeError:
        return f'{name} {why}, got {value!r}'
    return None


def outline_ok(flat: Sequence[float]) -> bool:
    try:
        PlannerConfig(robot_outline=tuple(pairs(flat))).validate()
    except ValueError:  # PlanningError too
        return False
    return True


def parse_polygon(text: str) -> List[XY]:
    """'x0,y0; x1,y1; ...' (any of , ; and spaces between the numbers) -> [(x0, y0), (x1, y1), ...]."""
    nums = [float(t) for t in re.split(r'[\s,;]+', text.strip()) if t]
    if len(nums) % 2:
        raise ValueError(f'odd number of coordinates in {text!r}')
    return list(zip(nums[0::2], nums[1::2]))


def xy(polygon) -> List[XY]:
    return [(float(p.x), float(p.y)) for p in polygon.points]


def pairs(flat: Sequence[float]) -> List[XY]:
    if len(flat) % 2:
        raise ValueError('odd number of coordinates')
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]


def to_path_msg(poses2d: Sequence[Pose2D], frame_id: str, stamp) -> Path:
    """Poses as nav_msgs/Path, one PoseStamped each. The yaw becomes a quaternion around +z.

    Message classes build the defaults of all their fields in the constructor (with keywords too), so
    each pose is one PoseStamped() with x, y, z and w set. All share the path's header (frame and time),
    which is fine for sending."""
    header = Header(frame_id=frame_id, stamp=stamp)
    poses = []
    for p in poses2d:
        ps = PoseStamped()
        ps.header = header
        position, q = ps.pose.position, ps.pose.orientation
        position.x = float(p.x)
        position.y = float(p.y)
        _, _, q.z, q.w = yaw_to_quaternion(p.yaw)
        poses.append(ps)
    return Path(header=header, poses=poses)


def with_warnings(message: str, warnings: Sequence[str]) -> str:
    return message + '. Warnings: ' + '; '.join(warnings) if warnings else message


def summary(plan: CoveragePlan) -> str:
    """What a plan is like, in a line."""
    st = plan.stats
    turns = ', '.join(f"{int(st['turns_' + k])} {k.replace('_', '-')}"
                      for k in ('u_turn', 'bulb', 'detour', 'pivot', 'fallback') if st['turns_' + k])
    return (f"{len(plan.poses)} poses, {st['path_length']:.1f} m ({st['lane_length']:.1f} m lanes, "
            f"{st['perimeter_length']:.1f} m perimeter, {st['turn_length']:.1f} m turns, "
            f"{st['transit_length']:.1f} m transit), {int(st['lanes'])} lanes at "
            f"{math.degrees(plan.angle):.1f} deg, turns: {turns or 'none'}, {1000 * st['planning_time']:.0f} ms")


def float32_warnings(rings: Sequence[Sequence[XY]]) -> List[str]:
    """A warning if points of geometry_msgs/Polygon (float32) lie so far out that they lose millimetres."""
    far = max((abs(c) for ring in rings for p in ring for c in p), default=0.0)
    if far <= FLOAT32_RANGE:
        return []
    step = 2.0 ** (math.frexp(far)[1] - 24)  # between neighbouring float32 values there
    return [f'coordinates up to {far:.0f} m from the map origin: geometry_msgs/Polygon is float32, about '
            f'{1000 * step:.0f} mm resolution there; use a local map frame']


def launch_settings(cfg: PlannerConfig) -> dict:
    """The planner.* settings the node's parameters (as cfg) stand for: the ones that aren't PlannerCore's defaults
    (the planner's with lane_order skip)."""
    default = PlannerConfig(lane_order='skip')

    def plain(v):
        return [plain(x) for x in v] if isinstance(v, (list, tuple)) else v

    return {n: plain(getattr(cfg, n)) for n in PlannerConfig.names()
            if n not in NOT_LAUNCH and getattr(cfg, n) != getattr(default, n)}


def prune(folder: str, days: float, mb: float, now: Optional[float] = None) -> List[str]:
    """data_dir/planner within bounds: the plans kept and the maps before older than days go, then the oldest of them
    while folder takes more than mb MB (the settings, angle steps and timing stay, they're small). The ones gone."""
    now = time.time() if now is None else now
    total = sum(os.path.getsize(os.path.join(root, f)) for root, _, files in os.walk(folder) for f in files)
    old = glob.glob(os.path.join(folder, 'plans', '*.pkl')) + \
        glob.glob(os.path.join(folder, 'mowbite_map_backups', 'map_*.json'))
    gone = []
    for path in sorted(old, key=os.path.getmtime):
        if now - os.path.getmtime(path) > days * 86400.0 or total > mb * 1e6:
            size = os.path.getsize(path)
            os.remove(path)
            total -= size
            gone.append(path)
    return gone


class NodeCore(PlannerCore):
    """PlannerCore in the ROS 2 node (see the top), what only the node can do from node. Without mower_logic: the lane
    spacing, the loops and the angle are the planner.* settings' and the area's (its properties in the map), each in
    one place (area_setting, plan_map)."""

    def __init__(self, node, *args, **kwargs):
        self.node = node
        self.cannot = dict(node.cannot)
        self.clean = None  # (the map plan_map got last, as it gave it)
        PlannerCore.__init__(self, *args, **kwargs)

    def setup(self, width, settings, stored):
        # (none of the lane spacing, the loops, the angle come from mower_logic: they're the planner's own here, an
        # area's own over them)
        return PlannerCore.setup(self, width, settings, stored)._replace(own=frozenset(MOWER_LOGIC), own_angle=True)

    def area_setting(self, props, own, name, value):
        """An area's setting into its one place: OpenMower's property where it has one (and "the planner works it out"
        in place of it into the planner property), else the planner property."""
        key = OPENMOWER_PROPS.get(name)
        auto = name == 'angle_strategy' and value is not None or \
            name == 'perimeter_passes' and value == AUTO_VALUES['perimeter_passes'] and not isinstance(value, bool)
        if key is None or auto:
            PlannerCore.area_setting(self, props, own, name, value)
            if auto:  # (in place of OpenMower's)
                props.pop(OPENMOWER_PROPS['angle' if name == 'angle_strategy' else name], None)
                if name == 'angle_strategy':
                    own.pop('angle', None)
            return
        if value is not None:
            if name in ('angle_min', 'angle_max'):  # (the range as a whole is checked with the area)
                if _number(value) is None:
                    raise PlanningError(f'{name} must be a number (rad)')
            else:
                self.current.base.with_overrides({'mower_width': self.current.mower_width, name: value}).validate()
            props[key] = value
        else:
            props.pop(key, None)
        own.pop(name, None)
        if name == 'angle':  # (a fixed angle in place of the planner's)
            own.pop('angle_strategy', None)

    def plan_map(self, data):
        """The map as the plans take it: of an area's two places for a value (OpenMower's property set again after
        "the planner works it out" went into its planner property) OpenMower's."""
        clean = self.clean
        if data is None or clean is not None and clean[0] is data:
            return None if data is None else clean[1]
        out = json.loads(json.dumps(data))
        for area in out.get('areas') or []:
            props = area.get('properties') if isinstance(area, dict) else None
            own = props.get('planner') if isinstance(props, dict) else None
            if not isinstance(own, dict):
                continue
            for name, key in OPENMOWER_PROPS.items():
                v = _number(props.get(key))
                if v is not None and (v >= 0 or key not in ('outline_count', 'outline_overlap_count')):
                    own.pop(name, None)
                    if name == 'angle':
                        own.pop('angle_strategy', None)
            if not own:
                props.pop('planner')
        self.clean = (data, out)
        return out

    def log(self, level, msg, *args, **kwargs):
        """To the node's log and planner.log's."""
        record = logging.LogRecord('mowbite_planner', level, __file__, 0, msg, args,
                                   sys.exc_info() if kwargs.get('exc_info') else None)
        LOG.handle(record)
        text = LOG.format(record)
        logger = self.node.get_logger()
        (logger.error if level >= logging.ERROR else logger.warning if level >= logging.WARNING else logger.info)(text)

    def loginfo(self, *args, **kwargs):
        self.log(logging.INFO, *args, **kwargs)

    def logwarn(self, *args, **kwargs):
        self.log(logging.WARNING, *args, **kwargs)

    def logerr(self, *args, **kwargs):
        self.log(logging.ERROR, *args, **kwargs)

    def publish(self, topic, payload, retain):
        self.node.send_out(topic, payload, retain)

    def param(self, name, default=None):
        return self.node.robot_param(name, default)

    def replace_map(self, data, based_on):
        self.node.replace_map(data, based_on)

    def map_unusable(self):
        return self.node.map_unusable()

    def arrives(self, boundary, obstacles, cfg, drivable, keep_out):
        return self.node.arrives(boundary, obstacles, cfg, drivable, keep_out)

    def drive_ignored(self):
        return self.node.drive_ignored()

    def back_out_ignored(self):
        return self.node.back_out_ignored()

    def mower_ready(self):
        return self.node.mower_ready()

    def start_mowing(self):
        self.node.start_mowing()


class MowbitePlannerNode(Node):
    """The node, see the top. A robot's own node is a class of it: its parameters' defaults (defaults), the settings
    it can't have (cannot, see PlannerCore), the RPC methods it adds (more_methods) and what the methods marked
    'robot' below do on it (its transport, map, events, parameters...)."""

    defaults = {}
    cannot = {'planner': 'no slic3r planner here',
              'leftovers_auto': 'this node doesn\'t send the robot out',
              'remote_planning': 'no house computer here'}
    # the planner.* methods answered on a thread of their own (they wait for the worker or another node)
    background = ('planner.area.set', 'planner.plan', 'planner.leftovers', 'planner.drive')
    # an RPC method no node here answers gets its error (no other node shares the requests)
    answers_unknown = True

    def __init__(self, node_name='mowbite_planner', **kwargs):
        super().__init__(node_name, **kwargs)
        # registered first, so it also checks the values a launch file or YAML sets at declaration
        self.add_on_set_parameters_callback(self._check_parameters)
        for name, (default, description, _, _) in PARAMETERS.items():
            default = self.defaults.get(name, default)
            descriptor = ParameterDescriptor(description=description)
            if name in RANGES:
                lo, hi = RANGES[name]
                if isinstance(default, int):
                    descriptor.integer_range = [IntegerRange(from_value=lo, to_value=hi, step=1)]
                else:
                    descriptor.floating_point_range = [FloatingPointRange(from_value=lo, to_value=hi, step=0.0)]
            # the default gives the type: NaN is a double, the turn types (not empty) a string array; robot_outline's
            # is its type, it's not set
            self.declare_parameter(name, default, descriptor)
        # lists without a default: declared with their type, an empty default would come out as a byte array
        self.declare_parameter('boundary_polygon', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor(
            description='for ~/plan_from_parameters: boundary as [x0, y0, x1, y1, ...] in metres'))
        self.declare_parameter('obstacles', Parameter.Type.STRING_ARRAY, ParameterDescriptor(
            description='for ~/plan_from_parameters: one string per obstacle, "x0,y0; x1,y1; ..." in metres'))

        self.latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, '~/coverage_path', self.latched)
        self.create_service(PlanCoverage, '~/plan_coverage', self._on_plan_coverage)
        self.create_service(Trigger, '~/plan_from_parameters', self._on_plan_from_parameters)
        self.core = None
        self.rpc_on = False
        if self._param('rpc') or self._param('use_map'):
            self._core_setup()
        self.get_logger().info('MowBite Planner ready: services ~/plan_coverage and ~/plan_from_parameters' +
                               (', the planner.* RPC methods' if self.rpc_on else ''))

    # PlannerCore (rpc, use_map)

    def _core_setup(self):
        data_dir = os.path.expanduser(self._param('data_dir'))
        self.files = os.path.join(data_dir, 'planner') if data_dir else ''
        if self.files:
            try:
                os.makedirs(self.files, exist_ok=True)
            except OSError as e:
                self.get_logger().warning(f'nothing kept, {self.files} not there: {e}')
                self.files = ''
        values = self.values()
        self.core = NodeCore(self, float(values['mower_width']), launch_settings(self.config(values)),
                             os.path.join(self.files, 'mowbite_planner_settings.json') if self.files else '',
                             os.path.join(self.files, 'plans') if self.files else '',
                             os.path.join(data_dir, HISTORY_DIR) if data_dir else '', int(values['workers']))
        self.core.methods.update(self.more_methods())
        self.map_input()
        self.events_input()
        if self.files:
            self.create_timer(PRUNE_EVERY, self._prune)
            self._prune()
        if self._param('rpc'):
            self.rpc_on = True
            self._requests = queue.Queue()
            threading.Thread(target=self._rpc_worker, name='rpc', daemon=True).start()
            self.rpc_input()

    def destroy_node(self):
        if self.rpc_on:
            self.rpc_end()
        return super().destroy_node()

    def _prune(self):
        try:
            gone = prune(self.files, self._param('keep_days'), self._param('keep_mb'))
        except OSError as e:  # (a file the planner itself removed meanwhile: the next time)
            self.get_logger().warning(f'{self.files} not tidied up: {e}')
            return
        if gone:
            self.get_logger().info(f'{len(gone)} plans kept and maps before gone (keep_days, keep_mb)')

    def on_request(self, method: str, params: str, rid: str):
        """An RPC request (params: JSON text, empty: none), answered with answer()."""
        if method not in self.core.methods:
            if self.answers_unknown:
                self.answer(rid, None, (METHOD_NOT_FOUND, f'Method "{method}" not found'))
            return
        if method in self.background:
            threading.Thread(target=self._answer, args=(method, params, rid), name=f'rpc {method}',
                             daemon=True).start()
        else:
            self._requests.put((method, params, rid))

    def _rpc_worker(self):
        while True:
            self._answer(*self._requests.get())

    def _answer(self, method: str, params: str, rid: str):
        try:
            got = json.loads(params) if params else None
        except ValueError:
            got = []  # (an error: by name)
        REQUEST.id = rid
        result, error = core_call(self.core.methods, method, got, self.core.loginfo, self.core.logerr)
        if error is None and too_big(result):  # (over MQTT: the broker would drop it without a word)
            error = (ANSWER_TOO_BIG, too_big(result))
            self.core.logwarn('MowBite Planner: %s: %s', method, error[1])
        if rid:
            self.answer(rid, result, error)

    # what the node does on a robot (a robot's own node: its own)

    def more_methods(self) -> dict:
        """The robot's own RPC methods: name -> callback(params)."""
        return {}

    def rpc_input(self):
        """RPC requests: JSON {method, params, id} on ~/rpc/request."""
        self._response_pub = self.create_publisher(String, '~/rpc/response', 100)
        self.create_subscription(String, '~/rpc/request', self._on_rpc_text, 100)

    def _on_rpc_text(self, msg):
        try:
            req = json.loads(msg.data)
            method, rid = req['method'], str(req.get('id') or '')
            params = json.dumps(req['params']) if req.get('params') is not None else ''
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning(f'~/rpc/request not readable: {msg.data[:200]!r}')
            return
        self.on_request(str(method), params, rid)

    def answer(self, rid: str, result: Optional[str], error: Optional[Tuple[int, str]]):
        """The answer to the request rid: result (JSON text) or error (code, message): JSON {id, result} or {id, error:
        {code, message}} on ~/rpc/response."""
        out = '{"id": %s, "result": %s}' % (json.dumps(rid), result) if error is None else json.dumps(
            {'id': rid, 'error': {'code': error[0], 'message': error[1]}})
        self._response_pub.publish(String(data=out))

    def rpc_end(self):
        """The node ends."""

    def map_input(self):
        """The map: ~/map (OpenMower's JSON, transient local)."""
        self.create_subscription(String, '~/map', lambda msg: self.core.set_map(msg.data), self.latched)

    def events_input(self):
        """The events of the robot's logic: ~/events (JSON like OpenMower's events/json)."""
        self.create_subscription(String, '~/events', lambda msg: self.core.on_event(msg.data), 100)

    def send_out(self, topic: str, payload: str, retain: bool):
        """PlannerCore's publish: JSON {topic, payload, retain} on ~/publish (the planning's progress)."""
        if not hasattr(self, '_publish_pub'):
            self._publish_pub = self.create_publisher(String, '~/publish', 10)
        self._publish_pub.publish(String(data=json.dumps({'topic': topic, 'payload': payload, 'retain': retain})))

    def robot_param(self, name: str, default=None):
        """PlannerCore's param: none here (no FTC_PARAMS: the time a plan takes from the planner's own speeds)."""
        return default

    def replace_map(self, data: dict, based_on: Optional[str]):
        raise PlanningError('the map can\'t be written here')

    def map_unusable(self) -> str:
        return ''

    def arrives(self, boundary, obstacles, cfg: PlannerConfig, drivable, keep_out):
        return None

    def drive_ignored(self) -> str:
        return ''

    def back_out_ignored(self) -> str:
        return self.drive_ignored()

    def mower_ready(self) -> bool:
        return False

    def start_mowing(self):
        pass

    # parameters

    def _check_parameters(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            reason = invalid(p.name, p.value) if p.name in PARAMETERS and p.value is not None else None
            if reason:
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def _param(self, name: str):
        p = self.get_parameter_or(name, None)  # (robot_outline may not be set)
        return None if p is None else p.value

    def _list_param(self, name: str) -> list:
        p = self.get_parameter_or(name, None)
        return [] if p is None or p.value is None else list(p.value)

    def values(self, overrides: Sequence = ()) -> dict:
        """The node's parameters (name: value) with a request's overrides (rcl_interfaces/Parameter) on top.
        Raises ValueError for an unknown name, a wrong type or a bad value."""
        values = {name: self._param(name) for name in PARAMETERS}
        for msg in overrides:
            p = Parameter.from_parameter_msg(msg)
            if p.name not in PARAMETERS:
                raise ValueError(f'no parameter {p.name!r}')
            default = PARAMETERS[p.name][0]
            declared = default if isinstance(default, Parameter.Type) else self.get_parameter(p.name).type_
            if p.type_ != declared:
                raise ValueError(f'{p.name} must be of type {declared.name.lower()}')
            reason = invalid(p.name, p.value)
            if reason:
                raise ValueError(reason)
            values[p.name] = p.value
        return values

    @staticmethod
    def config(values: dict, **overrides) -> PlannerConfig:
        """The planner's settings from parameter values (see values()), overrides (PlannerConfig's names) on
        top."""
        cfg = {name: v for name, v in values.items() if name in PlannerConfig.names()}
        rad = lambda name: None if math.isnan(values[name]) else math.radians(values[name])  # NaN: not set
        fixed = values['angle_strategy'] == 'fixed'
        cfg.update(angle=rad('fixed_angle_deg') if fixed else None,
                   angle_strategy='longest_edge' if fixed else values['angle_strategy'],
                   angle_offset=rad('angle_offset_deg'), angle_min=rad('angle_min_deg'),
                   angle_max=rad('angle_max_deg'), angle_step=rad('angle_step_deg'),
                   crosshatch_angle=rad('crosshatch_angle_deg'),
                   perimeter_offset=None if values['perimeter_offset'] < 0.0 else values['perimeter_offset'],
                   robot_width=None if values['robot_width'] < 0.0 else values['robot_width'],
                   robot_front=None if values['robot_front'] < 0.0 else values['robot_front'],
                   robot_rear=None if values['robot_rear'] < 0.0 else values['robot_rear'],
                   robot_outline=tuple(pairs(values['robot_outline'])) if values['robot_outline'] else None,
                   turn_types=tuple(values['turn_types']))
        cfg.update(overrides)
        return PlannerConfig(**cfg)

    # services

    def _on_plan_coverage(self, request, response):
        try:
            values = self.values(request.overrides)
        except ValueError as e:
            response.success = False
            response.message = f'bad override: {e}'
            self.get_logger().warning(response.message)
            return response
        if self.core is not None and values['use_map']:
            answered = self._plan_area(request, response, values)
            if answered is not None:
                return answered
        overrides = {}
        if request.mower_width > 0.0:
            overrides['mower_width'] = float(request.mower_width)
        if request.overlap >= 0.0:
            overrides['overlap'] = float(request.overlap)
        if request.perimeter_passes >= 0:
            overrides['perimeter_passes'] = int(request.perimeter_passes)
        if request.use_angle:
            overrides['angle'] = float(request.angle)
        start = None
        if request.use_start_pose:
            q = request.start_pose.orientation
            start = (float(request.start_pose.position.x), float(request.start_pose.position.y),
                     quaternion_to_yaw(q.x, q.y, q.z, q.w))
        end = (float(request.end_point.x), float(request.end_point.y)) if request.use_end_point else None
        boundary, obstacles = xy(request.boundary), [xy(poly) for poly in request.obstacles]
        drivable = [xy(poly) for poly in request.drivable_areas]
        plan, path, message = self.plan(boundary, obstacles, values, overrides, start, end, drivable,
                                        float32_warnings([boundary, *obstacles, *drivable]))
        response.success = plan is not None
        response.message = message
        if plan is None:
            return response
        return self._coverage_response(response, plan, path)

    @staticmethod
    def _coverage_response(response, plan: CoveragePlan, path: Path):
        response.path = path
        response.angle = float(plan.angle)
        response.angles = [float(a) for a in plan.angles]
        response.fingerprint = fingerprint(plan)
        response.lane_spacing = float(plan.lane_spacing)
        response.path_length = float(plan.stats['path_length'])
        response.section_start = [int(i) for i in plan.section_starts]
        response.segment_start = [int(s.start) for s in plan.segments]
        response.segment_end = [int(s.end) for s in plan.segments]
        response.segment_type = [int(SEGMENT_TYPES[kind]) for kind in out_kinds(plan.segments)]
        response.segment_reverse = [bool(s.reverse) for s in plan.segments]
        return response

    def _plan_area(self, request, response, values: dict):
        """~/plan_coverage with a map: of an area of it (its outline as boundary) or of an outline of none (a preview),
        planned like planner.plan plans it (see the top), in the core's worker before what waits there. None without
        a map."""
        core = self.core
        data = core.plan_map(core.current_map())
        if data is None:
            return None
        return core.worker.run(MOWER, self._area_plan, request, response, values, data, job='mower')

    def _area_plan(self, request, response, values: dict, data: dict):
        core = self.core
        index = match_area(data, xy(request.boundary))
        job, where = None, 'outline' if index is None else f'area {index}'
        try:
            if index is None:  # (like planner.plan plans an outline: its obstacles and drivable areas, no dock)
                job = AreaJob(area_id='', name='', boundary=xy(request.boundary),
                              obstacles=[xy(p) for p in request.obstacles], mow_around=[],
                              drivable=[xy(p) for p in request.drivable_areas], nav=[])
            else:
                job = area_job(data, index)
                where = f'area {job.area_id}'
            reason = core.map_unusable()
            if reason:
                raise PlanningError(reason)
            armed, plan = core.armed_plan(job) if index is not None else (False, None)
            kept = False
            if armed and plan is None:  # (a plan without paths: the job goes on with the next area)
                response.success = True
                response.message = f'{where}: the leftovers are armed (planner.leftovers), none of them is here: ' \
                                   f'nothing to mow'
                response.path = to_path_msg([], values['frame_id'], self.get_clock().now().to_msg())
                response.fingerprint = hashlib.sha256().hexdigest()  # (export.fingerprint's of no poses)
                self.get_logger().info(response.message)
                return response
            if not armed:
                cfg = self.area_config(request, job, values)
                # (as planner.plan plans it: the outline and the obstacles as float32, the areas to mow around not)
                obstacles = [o if any(o is m for m in job.mow_around) else _float32(o) for o in job.obstacles]
                if cfg.nested_areas:
                    obstacles += job.nested
                start, end = job.dock, job.dock[:2] if job.dock else None
                if request.use_start_pose:
                    q = request.start_pose.orientation
                    start = (float(request.start_pose.position.x), float(request.start_pose.position.y),
                             quaternion_to_yaw(q.x, q.y, q.z, q.w))
                if request.use_end_point:
                    end = (float(request.end_point.x), float(request.end_point.y))
                plan, kept = core.kept_plan(_float32(job.boundary), obstacles, cfg, start, end, job.drivable,
                                            job.keep_out, margins_for(data, obstacles + job.keep_out), job.navigation,
                                            job.unmowed)
        except PlanningError as e:
            response.success = False
            response.message = f'{where}: {e}'
            self.get_logger().warning(response.message)
            return response
        except Exception as e:  # noqa: BLE001 - a broken area must not take the node down
            response.success = False
            response.message = f'{where}: planning failed: {type(e).__name__}: {e}'
            self.get_logger().error(response.message)
            return response
        if not armed:
            core.serve(job.area_id or None, plan)
        path = to_path_msg(plan.poses, values['frame_id'], self.get_clock().now().to_msg())
        if armed:
            what = f'the leftovers ({plan.stats["leftovers"]["passes"]} passes)'
        else:
            what = 'kept from before' if kept else 'planned'
        response.success = True
        response.message = with_warnings(f'{where}, {what}: {summary(plan)}',
                                         [*self.left_out(request, plan), *plan.warnings])
        self.get_logger().info(response.message)
        if values['publish_path']:
            self._path_pub.publish(path)
        return self._coverage_response(response, plan, path)

    def area_config(self, request, job, values: dict) -> PlannerConfig:
        """The settings of a ~/plan_coverage request with the map (see the top): the planner.* settings, the area's own
        over them (its properties), of an area's what the request sets over those (a preview's edited ones),
        angle_offset_deg on top of the angle (a job's own). Not the request's lane spacing, it's the planner's."""
        current = self.core.current
        cfg = current.base.with_overrides({'mower_width': current.mower_width}).with_overrides(job.overrides)
        asked = {}
        if request.perimeter_passes >= 0:
            asked['perimeter_passes'] = int(request.perimeter_passes)
        given = {msg.name for msg in request.overrides}
        if 'lane_overlap_passes' in given:
            asked['lane_overlap_passes'] = values['lane_overlap_passes']
        if 'perimeter_offset' in given:
            asked['perimeter_offset'] = None if values['perimeter_offset'] < 0.0 else values['perimeter_offset']
        if request.use_angle:
            asked['angle'] = float(request.angle)
        if 'angle_offset_deg' in given:
            asked['angle_offset'] = cfg.angle_offset + math.radians(values['angle_offset_deg'])
        angle = self.core.angle
        return cfg.with_overrides(dict(asked, angle_steps=angle['steps'], finished_mows=angle['finished']))

    def left_out(self, request, plan: CoveragePlan) -> List[str]:
        """What of a ~/plan_coverage request with the map doesn't count (see the top), as warnings."""
        out = []
        others = sorted({msg.name for msg in request.overrides} - set(AREA_OVERRIDES))
        if others:
            out.append(f'overrides left out, the planner.* settings count: {", ".join(others)}')
        if request.mower_width > 0.0 and request.overlap >= 0.0 and plan.config.lane_spacing_mode == 'fixed':
            asked = request.mower_width * (1.0 - request.overlap)
            if abs(asked - plan.lane_spacing) > SPACING_OFF:
                out.append(f'the request\'s lane spacing {asked:.3f} m left out, the planner.* settings\' is '
                           f'{plan.lane_spacing:.3f} m')
        return out

    def _on_plan_from_parameters(self, request, response):
        try:
            boundary = pairs(self._list_param('boundary_polygon'))
            obstacles = [parse_polygon(text) for text in self._list_param('obstacles')]
        except ValueError as e:
            response.success = False
            response.message = f'bad boundary_polygon or obstacles parameter: {e}'
            return response
        plan, _, message = self.plan(boundary, obstacles, self.values(), {}, want_path=False)
        response.success = plan is not None
        response.message = message
        return response

    def plan(self, boundary: List[XY], obstacles: List[List[XY]], values: dict, overrides: dict, start=None,
             end=None, drivable=(), warnings: Sequence[str] = (),
             want_path: bool = True) -> Tuple[Optional[CoveragePlan], Optional[Path], str]:
        """Plans and publishes the path with parameter values and PlannerConfig overrides (see config()):
        (the plan, it as nav_msgs/Path, what to tell the caller), the first two None if it failed, the path
        also if it's neither wanted nor published. warnings: about the request, told with the plan's."""
        for w in warnings:
            self.get_logger().warning(w)
        try:
            plan = plan_coverage(boundary, obstacles, self.config(values, **overrides), start, end, drivable)
        except PlanningError as e:
            self.get_logger().warning(f'planning failed: {e}')
            return None, None, with_warnings(str(e), warnings)
        except Exception as e:  # noqa: BLE001 - a broken geometry must not take the node down
            message = f'planning failed: {type(e).__name__}: {e}'
            self.get_logger().error(message)
            return None, None, with_warnings(message, warnings)

        path = None
        if want_path or values['publish_path']:
            path = to_path_msg(plan.poses, values['frame_id'], self.get_clock().now().to_msg())
        message = with_warnings(summary(plan), [*warnings, *plan.warnings])
        self.get_logger().info(message)
        for w in plan.warnings:
            self.get_logger().warning(w)
        if values['publish_path']:
            self._path_pub.publish(path)
        return plan, path, message


def main(args=None, node_class=MowbitePlannerNode):
    """The node (node_class: a robot's own) till SIGINT or SIGTERM. They end it here, not rclpy's handlers: those shut
    the context down first, what destroy_node still sends (rpc_end) would go nowhere and the node end with an error."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    node = node_class()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=STOP_POLL)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
