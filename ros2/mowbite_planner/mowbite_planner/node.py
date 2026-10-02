"""The MowBite Planner as a ROS 2 node.

Services
  ~/plan_coverage (mowbite_planner_interfaces/srv/PlanCoverage)
      boundary + obstacles as geometry_msgs/Polygon in, the route as nav_msgs/Path out (plus where its
      perimeter loops, lanes, turns and transits are). Any of the node's parameters can be overridden per
      request (overrides).
  ~/plan_path (mowbite_planner_interfaces/srv/PlanPath)
      the request and response of OpenMower's slic3r_coverage_planner/PlanPath. Started with the node
      name slic3r_coverage_planner it's at the same name as OpenMower's (slic3r_coverage_planner/plan_path).
  ~/plan_from_parameters (std_srvs/srv/Trigger)
      plans the area in the parameters boundary_polygon and obstacles, the result goes out on the topic.

Topic
  ~/coverage_path (nav_msgs/msg/Path), transient local: the last plan, also for subscribers that come
  later (RViz: set the Path display's durability to Transient Local).

Planning is plain Python in the service callback. On the default single threaded executor requests are
handled one after another. Most of the time goes into the messages, about 20 us a pose: a garden of
300 square metres (18000 poses) takes about half a second on a PC, a CM4 needs four to six times as long.
"""

import math
import re
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from mowbite_planner_interfaces.msg import Path as PlanPiece
from mowbite_planner_interfaces.srv import PlanCoverage, PlanPath

from .export import MAX_PATH_POSES, fingerprint, plan_pieces
from .mathutil import Pose2D, quaternion_to_yaw, yaw_to_quaternion
from .planner import (ANGLE_STRATEGIES, FILL_PATTERNS, LANE, LANE_ORDERS, MAX_PASSES, PERIMETER,
                      PERIMETER_DIRECTIONS, PERIMETER_ORDERS, ROUTE_ORDERS, TRANSIT, TURN, TURN_TYPES, CoveragePlan,
                      PlannerConfig, PlanningError, plan_coverage)

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
    'perimeter_passes': (2, 'loops along the outline and around every obstacle', lambda v: 0 <= v <= MAX_PASSES,
                         f'must be 0..{MAX_PASSES}'),
    'perimeter_offset': (-1.0, 'distance of the first perimeter loop from the outline [m]. Negative: '
                         'mower_width / 2, the blade edge on the outline. 0 for outlines recorded by driving '
                         "the mower's centre along the edge (OpenMower's outline_offset)", math.isfinite,
                         'must be a number'),
    'robot_width': (-1.0, "width of the mower's body [m]: its centre keeps half of it off the outline and the "
                    'obstacles, the first loop moves further in if needed. Negative: only perimeter_offset counts',
                    math.isfinite, 'must be a number'),
    'blade_offset': (0.0, "how far the blade's centre sits left of the mower's centre [m], negative: right. The "
                     'loops go round with the blade on their outer side', math.isfinite, 'must be a number'),
    'blade_ahead': (0.0, "how far the blade's centre sits ahead of the mower's centre [m] (OpenMower's base_link "
                    'between the drive wheels), negative: behind. The lanes start and end that much earlier',
                    math.isfinite, 'must be a number'),
    'perimeter_order': ('first', 'perimeter loops first or last (last mows over the marks the turns leave)',
                        lambda v: v in PERIMETER_ORDERS, 'must be first or last'),
    'perimeter_direction': ('auto', 'perimeter loops ccw or cw, auto: the way the mower faces',
                            lambda v: v in PERIMETER_DIRECTIONS, 'must be auto, ccw or cw'),
    'lane_overlap_passes': (0, "lanes reach this many perimeter loops further out (OpenMower's "
                            'outline_overlap_count)', lambda v: 0 <= v <= MAX_PASSES, f'must be 0..{MAX_PASSES}'),
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
    'turn_radius': (0.25, 'radius of the turns between lanes [m]. Tighter ones (down to half the lane '
                    'spacing, then turning on the spot) only where the headland is too small',
                    lambda v: v >= 0.0, 'must be >= 0'),
    'allow_reverse': (True, 'K-turns (forward arc, a short way back, forward arc) where a forward turn of '
                      'turn_radius does not fit. Off for controllers that only drive forwards (OpenMower FTC)',
                      None, ''),
    'turn_types': (list(TURN_TYPES), 'turns allowed between lanes, preferred first: ' + ', '.join(TURN_TYPES),
                   lambda v: all(t in TURN_TYPES for t in v), 'must be some of ' + ', '.join(TURN_TYPES)),
    'lane_order': ('snake', 'snake: lane after neighbouring lane. skip: lanes far enough apart for a forward '
                   'U-turn of turn_radius, the ones between on the way back', lambda v: v in LANE_ORDERS,
                   'must be snake or skip'),
    'route_order': ('nearest', 'cells: nearest (the closest one not mowed yet next) or optimized (the order with the '
                    'shortest drives between them, takes a little longer to plan)', lambda v: v in ROUTE_ORDERS,
                    'must be nearest or optimized'),
    'perimeter_corner_radius': (0.15, 'outline corners of the perimeter loops are rounded with this radius '
                                '[m], 0 = sharp', lambda v: v >= 0.0, 'must be >= 0'),
    'waypoint_spacing': (0.10, 'distance between path poses [m], 0 = only where the direction changes',
                         lambda v: v >= 0.0, 'must be >= 0'),
    'transit_clearance': (0.05, 'transits keep this far from the outer loop and obstacle loops [m]',
                          lambda v: v >= 0.0, 'must be >= 0'),
    'simplify_tolerance': (0.01, 'the boundary and obstacles are simplified this much first [m]',
                           lambda v: v >= 0.0, 'must be >= 0'),
    'min_lane_length': (0.10, 'shorter pieces of a lane are left out [m]', lambda v: v >= 0.0, 'must be >= 0'),
    'bend_max_gap': (0.0, 'a row split by an obstacle with at most this much between its pieces becomes one lane '
                     'bending around it [m], 0 = split', lambda v: v >= 0.0, 'must be >= 0'),
    'bend_max_offset': (0.5, "how far a bend may leave its lane's line [m]", lambda v: v >= 0.0, 'must be >= 0'),
    'transit_edge_distance': (0.0, 'transits keep this far off the edges where that costs little more [m], 0 = '
                              'the shortest way', lambda v: v >= 0.0, 'must be >= 0'),
    'transit_edge_cost': (2.0, 'every metre of a transit closer to the edge than transit_edge_distance counts this '
                          'much more', lambda v: v >= 0.0, 'must be >= 0'),
    'publish_path': (True, 'publish every plan on ~/coverage_path', None, ''),
    'max_path_poses': (MAX_PATH_POSES, '~/plan_path splits longer paths (OpenMower counts poses in an int16)',
                       lambda v: v >= 4, 'must be >= 4'),
}
RANGES = {
    'mower_width': (0.01, 5.0),
    'overlap': (0.0, 0.95),
    'perimeter_passes': (0, MAX_PASSES),
    'lane_overlap_passes': (0, MAX_PASSES),
    'angle_step_deg': (0.5, 90.0),
    'turn_radius': (0.0, 5.0),
    'robot_width': (-1.0, 5.0),
    'blade_offset': (-1.0, 1.0),
    'blade_ahead': (-1.0, 1.0),
}
FILL_TYPES = {PlanPath.Request.FILL_LINEAR: 'lanes', PlanPath.Request.FILL_CONCENTRIC: 'concentric'}
# m, float32 has steps of 1 mm or less up to here
FLOAT32_RANGE = 8192.0


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


def float32_warnings(rings: Sequence[Sequence[XY]]) -> List[str]:
    """A warning if points of geometry_msgs/Polygon (float32) lie so far out that they lose millimetres."""
    far = max((abs(c) for ring in rings for p in ring for c in p), default=0.0)
    if far <= FLOAT32_RANGE:
        return []
    step = 2.0 ** (math.frexp(far)[1] - 24)  # between neighbouring float32 values there
    return [f'coordinates up to {far:.0f} m from the map origin: geometry_msgs/Polygon is float32, about '
            f'{1000 * step:.0f} mm resolution there; use a local map frame']


class MowbitePlannerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('mowbite_planner', **kwargs)
        # registered first, so it also checks the values a launch file or YAML sets at declaration
        self.add_on_set_parameters_callback(self._check_parameters)
        for name, (default, description, _, _) in PARAMETERS.items():
            descriptor = ParameterDescriptor(description=description)
            if name in RANGES:
                lo, hi = RANGES[name]
                if isinstance(default, int):
                    descriptor.integer_range = [IntegerRange(from_value=lo, to_value=hi, step=1)]
                else:
                    descriptor.floating_point_range = [FloatingPointRange(from_value=lo, to_value=hi, step=0.0)]
            # the default gives the type: NaN is a double, the turn types (not empty) a string array
            self.declare_parameter(name, default, descriptor)
        # lists without a default: declared with their type, an empty default would come out as a byte array
        self.declare_parameter('boundary_polygon', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor(
            description='for ~/plan_from_parameters: boundary as [x0, y0, x1, y1, ...] in metres'))
        self.declare_parameter('obstacles', Parameter.Type.STRING_ARRAY, ParameterDescriptor(
            description='for ~/plan_from_parameters: one string per obstacle, "x0,y0; x1,y1; ..." in metres'))

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, '~/coverage_path', latched)
        self.create_service(PlanCoverage, '~/plan_coverage', self._on_plan_coverage)
        self.create_service(PlanPath, '~/plan_path', self._on_plan_path)
        self.create_service(Trigger, '~/plan_from_parameters', self._on_plan_from_parameters)
        self.get_logger().info('MowBite Planner ready: services ~/plan_coverage, ~/plan_path and '
                               '~/plan_from_parameters')

    # parameters

    def _check_parameters(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            reason = invalid(p.name, p.value) if p.name in PARAMETERS and p.value is not None else None
            if reason:
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def _param(self, name: str):
        return self.get_parameter(name).value

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
            declared = self.get_parameter(p.name).type_
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
        response.path = path
        response.angle = float(plan.angle)
        response.angles = [float(a) for a in plan.angles]
        response.fingerprint = fingerprint(plan)
        response.lane_spacing = float(plan.lane_spacing)
        response.path_length = float(plan.stats['path_length'])
        response.section_start = [int(i) for i in plan.section_starts]
        response.segment_start = [int(s.start) for s in plan.segments]
        response.segment_end = [int(s.end) for s in plan.segments]
        response.segment_type = [int(SEGMENT_TYPES[s.kind]) for s in plan.segments]
        response.segment_reverse = [bool(s.reverse) for s in plan.segments]
        return response

    def _on_plan_path(self, request, response):
        # OpenMower's request: distance is the lane spacing, the blade's width is the parameter
        if request.fill_type not in FILL_TYPES:
            self.get_logger().error(f'plan_path: fill_type {request.fill_type} is not supported')
            return response
        if not request.distance > 0.0:
            self.get_logger().error(f'plan_path: distance must be > 0, got {request.distance}')
            return response
        values = self.values()
        width = max(float(values['mower_width']), float(request.distance))
        overrides = dict(
            fill_pattern=FILL_TYPES[request.fill_type],
            # OpenMower added its mow_angle_offset already, the node's angle offset and range don't apply
            angle=float(request.angle),
            angle_offset=0.0,
            angle_min=None,
            angle_max=None,
            mower_width=width,
            overlap=1.0 - float(request.distance) / width,
            perimeter_offset=float(request.outer_offset),
            perimeter_passes=int(request.outline_count),
            lane_overlap_passes=int(request.outline_overlap_count),
            skip_outline_loops=bool(request.skip_area_outline),
            skip_obstacle_loops=bool(request.skip_obstacle_outlines),
            skip_lanes=bool(request.skip_fill),
        )
        outline, holes = xy(request.outline), [xy(poly) for poly in request.holes]
        plan, path, _ = self.plan(outline, holes, values, overrides, warnings=float32_warnings([outline, *holes]))
        if plan is None:
            return response
        # the pieces share the path's poses, also the one where a piece ends and the next one starts
        response.paths = [PlanPiece(is_outline=1 if is_outline else 0,
                                    path=Path(header=path.header, poses=path.poses[a:b + 1]))
                          for is_outline, a, b in plan_pieces(plan, values['max_path_poses'])]
        return response

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
        st = plan.stats
        turns = ', '.join(f"{int(st['turns_' + k])} {k.replace('_', '-')}"
                          for k in ('u_turn', 'bulb', 'k_turn', 'detour', 'pivot', 'fallback') if st['turns_' + k])
        message = (
            f"{len(plan.poses)} poses, {st['path_length']:.1f} m ({st['lane_length']:.1f} m lanes, "
            f"{st['perimeter_length']:.1f} m perimeter, {st['turn_length']:.1f} m turns, "
            f"{st['transit_length']:.1f} m transit), {int(st['lanes'])} lanes at "
            f"{math.degrees(plan.angle):.1f} deg, turns: {turns or 'none'}, {1000 * st['planning_time']:.0f} ms")
        message = with_warnings(message, [*warnings, *plan.warnings])
        self.get_logger().info(message)
        for w in plan.warnings:
            self.get_logger().warning(w)
        if values['publish_path']:
            self._path_pub.publish(path)
        return plan, path, message


def main(args=None):
    rclpy.init(args=args)
    node = MowbitePlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
