"""The MowBite Planner on MowBite OS: mowbite_planner's ROS 2 node (MowbitePlannerNode, see its top) as MowBite OS'
robot, the node mowbite_planner (executable mowbite_planner_os_node).

  - rpc and use_map on by default. The planner.* methods over mowbite_bridge: registered on bridge/rpc/methods
    (RpcMethods, node_id mowbite_planner, reliable and transient local, depth 50; an empty list when the node ends),
    asked on bridge/rpc/request (RpcRequest, params JSON text), answered on bridge/rpc/response (RpcResponse, result
    JSON text) or bridge/rpc/error (RpcError); reliable, volatile, depth 100. A method of no node here is another
    node's: not answered (the bridge answers it).
  - the map of mowbite_map: map/json (OpenMower's JSON, latched). planner.plan, ~/plan_coverage with the map and
    planning ahead only while map/status says MATCH (the map fits the GNSS datum), else an error with its reason; on
    MATCH again the areas are planned ahead. planner.area.set writes the map with map/replace, expected_sha256 the
    digest of the map/json text it changed, and waits for the map/json that follows; mowbite_map's no comes back with
    its code (-32001: the map changed meanwhile, nothing written).
  - mowbite_logic's events (STATE, JOB_COMPLETE, JOB_RESET) from bridge/publish (events/json): angle_increment,
    perimeter_direction random, finished_mows, the finished mow's planner.drive and timing. The planning's progress
    (mowbite/planner/progress) out on bridge/publish, as api/planner.schema.json says (the planner's part of MowBite OS'
    interface, installed to share/mowbite_planner_os/api, mowbite_api reads it).
  - mowbite_follower's speed_fast, speed_slow and max_angular from bridge/params: how long mowing takes.
  - the body and the blade from robot.geometry: /mowbite_geometry on bridge/params (robot_width, robot_front,
    robot_rear, robot_outline, [] none: the rectangle, mower_width, blade_ahead, blade_offset; version counts up with
    every change).
    Not settable here (planner.settings says from where), the node's parameters, the stored ones, an area's and a
    request's own don't count; a new version: plans with the values before not taken (they're in the plans' keys), the
    areas planned ahead again. None yet, or ones the settings don't go with: nothing planned (planner.plan,
    ~/plan_coverage, ~/plan_transit say why), never without the body.
  - not here: the slic3r planner, leftovers_auto (MowBite OS' schedule starts the mower), the house computer.
    planner.settings says why.
  - back_up_with_plan (false): the robot backs up where a plan's poses face against the way (MowBite OS from M8):
    back_out planned; false: ignored (planner.settings says why). drive slope and reverse stay forwards either way
    (their own step: the sections are got to on the plan as driven forwards).
  - ~/plan_transit (mowbite_planner_os_interfaces/PlanTransit, transit_route.py): a drive from one pose to another
    through the drivable areas. A plan's sections start only where it gets to them facing their heading (arrives, see
    mowbite_planner.starts): the first from ARRIVE_FROM m behind, beside and ahead of it, nearer where the lawn is
    small, the others from where the section before ends, as mowbite_logic drives there. A section ends where the
    robot gets away again driving forwards (to the next one, or to the closest place the body turns whatever way,
    facing any way there), earlier where it ends in a dead end, left out where nothing of it does (back_out: backing up
    out of it).
    With a ~/plan_coverage request's start (use_start_pose: where mowbite_logic undocks to) the first section from
    there the way mowbite_logic asks for (max_outside ARRIVE_OUTSIDE); none got to from there: refused with
    START_UNREACHABLE first (mowbite_planner.starts, the message "area <id>: start unreachable: ...").
    ~/plan_transit in a callback group of its own on a MultiThreadedExecutor: answered while a ~/plan_coverage waits for
    its plan (the other callbacks one after another as before). It plans in the executor's thread, on geometries of
    its own (space._body_space: the kept ones as WKB, each thread its own; mowbite_planner's GEOS rule).
  - planner.calib.evaluate {path} (with calib_root, MowBite OS' data_dir/calibration): the calibration of the recording
    at path (absolute, under calib_root, else -32602) evaluated in a process of its own, CALIB_TIMEOUT s at most: the
    whole mowbite-calib/1 object (mowbite_planner.calib); no recording there, one it can't read (one without its end
    line too: its writer ended before it) or a timeout: -32603. On a thread of its own, the other methods go on.
    Without calib_root not offered.
  - role (planner): transit, a second instance for ~/plan_transit only (MowBite OS' heading home, masterplan 40 V2):
      - a name of its own (the launch's, not mowbite_planner: refused), its own log; one transit at a time, workers 0
      - not there: ~/plan_coverage, ~/plan_from_parameters, ~/coverage_path, the planner.* methods, bridge/rpc/*
      - nothing sent: no bridge/publish (no progress), no map/replace; no events read (finished mows), no calibration
      - nothing planned ahead, no plans kept, no leftovers; map/json, map/status and bridge/params (the body) as above
      - data_dir: nothing written, only the planner's planner/mowbite_planner_settings.json read (again once it changes)
      - the planner's counters stay its own: angle steps, timing, plans kept, maps before (data_dir/planner), leftovers
"""

import hashlib
import json
import math
import os
import threading
from typing import List, Optional

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shapely.geometry import Point
from shapely.ops import nearest_points
from std_msgs.msg import String

from mowbite_msgs.msg import MapStatus, MqttPublish, RpcError, RpcMethods, RpcRequest, RpcResponse
from mowbite_msgs.srv import ReplaceMap
from mowbite_planner import core
from mowbite_planner import node as base
from mowbite_planner.config import INTERNAL, RETIRED, PlannerConfig, PlanningError
from mowbite_planner.geometry import polygons
from mowbite_planner.mathutil import Pose2D, quaternion_to_yaw
from mowbite_planner.starts import _Spent
from mowbite_planner_os_interfaces.srv import PlanTransit

from .transit_route import MAX_OUTSIDE, centre_space, plan_transit

# the node's name at mowbite_bridge, s the empty list of methods may take to be delivered as the node ends,
# mowbite_logic's events
NODE_ID = 'mowbite_planner'
END_ACK = 1.0
EVENTS_TOPIC = 'events/json'
# s planner.area.set waits for mowbite_map to be there and to answer map/replace, then for its new map/json
MAP_FIND = 5.0
MAP_WAIT = 60.0
MAP_SETTLE = 5.0
# MapStatus.datum_state by its value
DATUM_STATES = ('UNKNOWN', 'EMPTY', 'MATCH', 'MISMATCH', 'UNBOUND')
# planner.calib.evaluate: s an evaluation may take (MowBite OS waits 120 s)
CALIB_TIMEOUT = 120.0
# arrives: m from a plan's first section's start the ways to it start (behind it, beside it, ahead of it; where none is
# that far in the lawn, halved till ARRIVE_NEAREST), at least half that far from it and ARRIVE_APART m from each other;
# m the ways from where the section before ends may start and end outside (mowbite_logic's transit_max_outside); ways
# planned at most for a plan (a number, not a time: the same plan on any computer), the sections after that left out
# (untried)
ARRIVE_FROM = 2.0
ARRIVE_NEAREST = 0.5
ARRIVE_APART = 0.3
ARRIVE_OUTSIDE = 0.3
ARRIVE_WAYS = 40
# the speeds of mowbite_follower (bridge/params) the time a plan takes is worked out with (core.FTC_PARAMS')
FOLLOWER = '/mowbite_follower'
# robot.geometry's entry of bridge/params, the settings it gives (the body and the blade), where they come from
GEOMETRY = '/mowbite_geometry'
GEOMETRY_KEYS = ('robot_width', 'robot_front', 'robot_rear', 'robot_outline', 'mower_width', 'blade_ahead',
                 'blade_offset')
FROM_GEOMETRY = 'robot.geometry (/mowbite_geometry)'
# the executor's threads: the node's callbacks and ~/plan_transit's
THREADS = 2
# role: the planner, or a second instance only for ~/plan_transit (see the top); the planner's settings under data_dir
ROLES = ('planner', 'transit')
TRANSIT = 'transit'
SETTINGS_FILE = os.path.join('planner', 'mowbite_planner_settings.json')


class OsNode(base.MowbitePlannerNode):
    """The node on MowBite OS, see the top."""

    defaults = {'rpc': True, 'use_map': True}
    cannot = {'planner': 'MowBite OS: no slic3r planner',
              'leftovers_auto': 'MowBite OS: its schedule starts the mower',
              'remote_planning': 'MowBite OS: no house computer',
              'body_fit': 'MowBite OS: always on, the body never sticks out (G6)',
              'robot_front_width': 'MowBite OS: the body is robot.geometry\'s (its robot_outline for another shape)',
              'robot_rear_width': 'MowBite OS: the body is robot.geometry\'s (its robot_outline for another shape)'}
    given_by = {n: FROM_GEOMETRY for n in GEOMETRY_KEYS}
    background = base.MowbitePlannerNode.background + ('planner.calib.evaluate',)
    answers_unknown = False

    def __init__(self, node_name='mowbite_planner', **kwargs):
        # the others' parameters by node (bridge/params), the latest map/status, how many map/json came
        self.others = {}
        self.map_status = None
        self.maps = threading.Condition()
        self.map_count = 0
        self._calib_root = None
        self._backs_up = None
        # robot.geometry's latest (version, {setting: value}), why nothing is planned with it ('' where it is)
        self.geometry = None
        self.geometry_why = ''
        self._role = None
        super().__init__(node_name, **kwargs)
        self.calib_root()  # (declared with the methods where the core is)
        self.backs_up()
        self.create_service(PlanTransit, '~/plan_transit', self._on_plan_transit,
                            callback_group=MutuallyExclusiveCallbackGroup())
        if self.role() == TRANSIT:
            self.get_logger().info('MowBite Planner ready (role transit): service ~/plan_transit only')
        mine = [n for n in GEOMETRY_KEYS if n != 'mower_width' and self._param(n) is not None and (
            len(self._param(n)) if n == 'robot_outline' else self._param(n) not in (-1.0, 0.0))]
        if mine:
            self.get_logger().info(f'parameters not counting here (from {FROM_GEOMETRY}): {", ".join(mine)}')

    def role(self) -> str:
        if self._role is None:
            role = self.declare_parameter('role', ROLES[0], ParameterDescriptor(
                description='planner: the planner of MowBite OS; transit: a second instance for ~/plan_transit only '
                            '(a name of its own, nothing sent, nothing written; see the node\'s top)')).value
            if role not in ROLES:
                raise ValueError(f'role must be planner or transit, got {role!r}')
            self._role = role
        return self._role

    def plans_coverage(self) -> bool:
        return self.role() != TRANSIT

    def _core_setup(self):
        """As MowbitePlannerNode's; role transit: TransitCore, only the map and bridge/params (see the top)."""
        if self.role() != TRANSIT:
            return super()._core_setup()
        if self.get_name() == NODE_ID:
            raise ValueError(f'role transit needs a name of its own, {NODE_ID} is the planner\'s')
        data_dir = os.path.expanduser(self._param('data_dir'))
        values = self.values()
        if values['workers']:
            self.get_logger().info('workers not taken (role transit: plans in one thread)')
        self.files = ''
        self.core = TransitCore(self, float(values['mower_width']), base.launch_settings(self.config(values)),
                                os.path.join(data_dir, SETTINGS_FILE) if data_dir else '')
        self.create_subscription(String, 'map/json', self._on_map, self.latched)
        self.create_subscription(MapStatus, 'map/status', self._on_map_status, self.latched)
        self.params_input()

    def calib_root(self) -> str:
        if self._calib_root is None:
            self._calib_root = self.declare_parameter('calib_root', '', ParameterDescriptor(
                description='planner.calib.evaluate reads recordings under here only (MowBite OS: data_dir/'
                            'calibration), empty: not offered')).value or ''
        return self._calib_root

    def backs_up(self) -> bool:
        if self._backs_up is None:
            self._backs_up = bool(self.declare_parameter('back_up_with_plan', False, ParameterDescriptor(
                description='the robot backs up where a plan\'s poses face against the way (MowBite OS from M8): '
                            'back_out planned; false: not (drive slope and reverse stay forwards either way)')).value)
        return self._backs_up

    # the body: robot.geometry's (given)

    def values(self, overrides=()) -> dict:
        """As MowbitePlannerNode's, an override of robot.geometry's not taken (ValueError)."""
        mine = sorted({msg.name for msg in overrides} & set(GEOMETRY_KEYS))
        if mine:
            raise ValueError(f'{", ".join(mine)} from {FROM_GEOMETRY} here')
        return super().values(overrides)

    def config(self, values: dict, **overrides) -> PlannerConfig:
        """As MowbitePlannerNode's, robot.geometry's values over it (where the core is)."""
        cfg = super().config(values, **overrides)
        return cfg if self.core is None else self.core.bound(cfg)

    def plan(self, boundary, obstacles, values, overrides, *args, **kwargs):
        """As MowbitePlannerNode's (~/plan_coverage without a map, ~/plan_from_parameters), in the core's worker (no
        planning in another thread, mowbite_planner's GEOS rule); never without the body."""
        why = self.no_body()
        if why:
            self.get_logger().warning(f'planning failed: {why}')
            return None, None, why
        return self.core.worker.run(core.MOWER, lambda: base.MowbitePlannerNode.plan(
            self, boundary, obstacles, values, overrides, *args, **kwargs), job='mower')

    # the transport: mowbite_bridge

    def more_methods(self) -> dict:
        return {'planner.calib.evaluate': self.calib_evaluate} if self.calib_root() else {}

    def rpc_input(self):
        self._response_pub = self.create_publisher(RpcResponse, 'bridge/rpc/response', 100)
        self._error_pub = self.create_publisher(RpcError, 'bridge/rpc/error', 100)
        self._methods_pub = self.create_publisher(RpcMethods, 'bridge/rpc/methods', QoSProfile(
            depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(RpcRequest, 'bridge/rpc/request',
                                 lambda msg: self.on_request(msg.method, msg.params, msg.id), 100)
        self._methods_pub.publish(RpcMethods(node_id=NODE_ID, methods=sorted(self.core.methods)))

    def answer(self, rid: str, result: Optional[str], error):
        if error is None:
            self._response_pub.publish(RpcResponse(result=result, id=rid))
        else:
            self._error_pub.publish(RpcError(code=error[0], message=error[1], id=rid, data=''))

    def rpc_end(self):
        # (mowbite_bridge forgets the methods; delivered before the node goes, END_ACK s at most)
        self._methods_pub.publish(RpcMethods(node_id=NODE_ID, methods=[]))
        acked = getattr(self._methods_pub, 'wait_for_all_acked', None)
        if acked is not None:
            from rclpy.duration import Duration  # (only rclpy's publishers have it)
            acked(Duration(seconds=END_ACK))

    def send_out(self, topic: str, payload: str, retain: bool):
        if self.role() != TRANSIT:  # (role transit sends nothing)
            self._publish_pub.publish(MqttPublish(topic=topic, payload=payload, retain=retain))

    def events_input(self):
        # (before anything is planned: the progress goes out on it)
        self._publish_pub = self.create_publisher(MqttPublish, 'bridge/publish', 100)
        self.create_subscription(MqttPublish, 'bridge/publish', self._on_publish, 100)
        self.params_input()

    def params_input(self):
        self.create_subscription(String, 'bridge/params', self._on_params, QoSProfile(
            depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_publish(self, msg):
        if msg.topic == EVENTS_TOPIC:
            self.core.on_event(msg.payload)

    def _on_params(self, msg):
        try:
            got = json.loads(msg.data)
            node, params = str(got['node']), dict(got['params'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning(f'bridge/params not readable: {msg.data[:200]!r}')
            return
        self.others[node] = params
        if node == GEOMETRY:
            self._on_geometry(params)

    def _on_geometry(self, params: dict):
        """robot.geometry's values: the settings with them from now on (a version or values other than before)."""
        try:
            got = geometry_values(params)
        except ValueError as e:
            self.geometry, self.geometry_why = None, f'the mower\'s dimensions not readable ({FROM_GEOMETRY}): {e}'
            self.get_logger().error(self.geometry_why)
            return
        if got == self.geometry and not self.geometry_why:
            return
        self.geometry, self.geometry_why = got, ''  # (till they're taken: not taken yet, see no_body)
        try:
            self.core.given_again()
        except PlanningError as e:
            self.geometry_why = f'the mower\'s dimensions (version {got[0]}) don\'t go with the planner\'s ' \
                f'settings: {e}'
            self.get_logger().error(self.geometry_why)
            return
        self.geometry_why = ''

    def given(self) -> dict:
        return dict(self.geometry[1]) if self.geometry is not None else {}

    def no_body(self) -> str:
        """Why nothing is planned now for the body ('' where it is): none yet or none readable, or the settings aren't
        robot.geometry's latest (being taken, or the settings don't go with them: till they do)."""
        got = self.geometry
        if got is None:
            return self.geometry_why or f'no dimensions of the mower yet ({FROM_GEOMETRY} on bridge/params)'
        if self.core is not None and {n: v for n, v in self.core.current_given().items() if n in GEOMETRY_KEYS} != \
                got[1]:
            return self.geometry_why or f'the mower\'s dimensions (version {got[0]}) not taken yet'
        return ''

    def robot_param(self, name: str, default=None):
        """OpenMower's FTC speeds as mowbite_follower's (speed_fast, speed_slow, max_angular in rad/s as speed_angular
        in deg/s). No /mower_logic: its values for the plans are the planner's own."""
        follower = self.others.get(FOLLOWER)
        if name != core.FTC_PARAMS or follower is None:
            return default
        speeds = {k: follower[k] for k in ('speed_fast', 'speed_slow') if k in follower}
        if 'max_angular' in follower:
            speeds['speed_angular'] = math.degrees(float(follower['max_angular']))
        return speeds

    # the map: mowbite_map

    def map_input(self):
        self.create_subscription(String, 'map/json', self._on_map, self.latched)
        self.create_subscription(MapStatus, 'map/status', self._on_map_status, self.latched)
        self._replace = self.create_client(ReplaceMap, 'map/replace')

    def _on_map(self, msg):
        self.core.set_map(msg.data)
        with self.maps:
            self.map_count += 1
            self.maps.notify_all()

    def _on_map_status(self, msg):
        was, self.map_status = self.map_status, msg
        if msg.datum_state == MapStatus.DATUM_MATCH and (was is None or was.datum_state != MapStatus.DATUM_MATCH):
            self.core.plan_ahead_later()

    def map_unusable(self) -> str:
        why = self.no_body()
        if why:
            return why
        status = self.map_status
        if status is None:
            return 'no map status from mowbite_map yet (map/status)'
        if status.datum_state == MapStatus.DATUM_MATCH:
            return ''
        state = DATUM_STATES[status.datum_state] if status.datum_state < len(DATUM_STATES) else status.datum_state
        return f'the map can\'t be planned now (map/status {state}): {status.reason or "no reason given"}'

    def replace_map(self, data: dict, based_on: Optional[str]):
        """data written with map/replace where map/json is still the text based_on (None: no check), then its new
        map/json waited for (a planner.area.set right after it changes that one). PlanningError where it isn't
        written, with mowbite_map's code."""
        if not self._replace.wait_for_service(timeout_sec=MAP_FIND):
            raise PlanningError('mowbite_map is not there (map/replace)')
        request = ReplaceMap.Request(map=json.dumps(data), expected_sha256=hashlib.sha256(
            based_on.encode()).hexdigest() if based_on is not None else '')
        with self.maps:
            count = self.map_count
        done = threading.Event()
        future = self._replace.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(MAP_WAIT):
            raise PlanningError(f'mowbite_map didn\'t answer map/replace in {MAP_WAIT:.0f} s')
        answer = future.result()
        if not answer.success:
            e = PlanningError(f'mowbite_map: {answer.message or "map not written"}')
            e.code = answer.code or None
            raise e
        with self.maps:
            self.maps.wait_for(lambda: self.map_count != count, MAP_SETTLE)

    def drive_ignored(self) -> str:
        return 'MowBite OS: drives forwards only for now (M8)'

    def back_out_ignored(self) -> str:
        return '' if self.backs_up() else self.drive_ignored()

    # how it gets to a plan's sections: ~/plan_transit

    def arrives(self, boundary, obstacles, cfg: PlannerConfig, drivable, keep_out):
        """Whether ~/plan_transit gets to a pose facing its heading (yaw None: there facing any way, the way out of
        where a section ends), through the area and the drivable areas but the obstacles and the areas to keep out of,
        as ~/plan_transit. From came_from (where the section before ends) the way mowbite_logic
        asks for (max_outside ARRIVE_OUTSIDE), none where the room the centre has on a way parts the two (nothing
        planned then); the first section's (came_from None) the body checked up to the pose (none of it outside): from
        ARRIVE_FROM m behind, beside or ahead of it (the closest place to there where the body turns whatever way, where
        there's none where the centre may go), from nearer where the lawn is too small for those. A pose none of them
        gets to isn't got to: no start it can't get to (the Tron's case of 30.10.). None without the body: nothing to
        check."""
        if not cfg.body:
            return None
        areas, away = [list(boundary)] + [list(d) for d in drivable], [list(o) for o in obstacles] + \
            [list(k) for k in keep_out]
        space = []  # (worked out once a plan, when it's first asked)
        parts = []  # (the parts of the room a way may take)
        ways = [0]

        def way(start, p: Pose2D, max_outside: float) -> bool:
            if ways[0] >= ARRIVE_WAYS:
                raise _Spent()
            ways[0] += 1
            goal = (p.x, p.y) if p.yaw is None else (p.x, p.y, p.yaw)
            try:
                plan_transit(start, goal, areas, away, cfg, max_outside=max_outside)
                return True
            except PlanningError:
                return False

        def part(q: Pose2D) -> set:
            if not parts:
                parts.extend(polygons(centre_space(areas, away, cfg)[0]))
            point = Point(q.x, q.y)
            return {i for i, a in enumerate(parts) if a.distance(point) <= ARRIVE_OUTSIDE + 1e-9}

        def gets_to(p: Pose2D, came_from: Optional[Pose2D] = None) -> bool:
            if came_from is not None:
                at, to = part(came_from), part(p)
                if at and to and not at & to:
                    return False
                return way((came_from.x, came_from.y, came_from.yaw), p, ARRIVE_OUTSIDE)
            if not space:
                room, fp = centre_space(areas, away, cfg)
                anywhere = fp.turning.anywhere if fp is not None else room
                space.append(room if anywhere.is_empty else anywhere)
            froms: List[tuple] = []
            far = ARRIVE_FROM
            while not froms and far >= ARRIVE_NEAREST - 1e-9:
                for turn in (math.pi, 0.5 * math.pi, -0.5 * math.pi, 0.0):
                    x, y = p.x + far * math.cos(p.yaw + turn), p.y + far * math.sin(p.yaw + turn)
                    q = nearest_points(space[0], Point(x, y))[0]
                    if math.hypot(q.x - p.x, q.y - p.y) >= 0.5 * far and \
                            all(math.hypot(q.x - a, q.y - b) >= ARRIVE_APART for a, b in froms):
                        froms.append((q.x, q.y))
                far *= 0.5
            return any(way((x, y, math.atan2(p.y - y, p.x - x)), p, 0.0) for x, y in froms)
        # (a plan kept with other code isn't taken)
        gets_to.key = {'os': os_code()}
        return gets_to

    def _on_plan_transit(self, request, response):
        if isinstance(self.core, TransitCore):
            self.core.follow()
        try:
            values = self.values(request.overrides)
        except ValueError as e:
            response.success = False
            response.message = f'bad override: {e}'
            self.get_logger().warning(response.message)
            return response
        why = self.no_body()
        if why:
            response.success = False
            response.message = f'planning the transit failed: {why}'
            self.get_logger().warning(response.message)
            return response
        q = request.start.orientation
        start = (float(request.start.position.x), float(request.start.position.y),
                 quaternion_to_yaw(q.x, q.y, q.z, q.w))
        q = request.goal.orientation
        goal = (float(request.goal.position.x), float(request.goal.position.y)) + \
            ((quaternion_to_yaw(q.x, q.y, q.z, q.w),) if request.use_goal_heading else ())
        drivable = [base.xy(poly) for poly in request.drivable_areas]
        obstacles = [base.xy(poly) for poly in request.obstacles]
        warnings = base.float32_warnings([*drivable, *obstacles])
        for w in warnings:
            self.get_logger().warning(w)
        max_outside = float(request.max_outside) if request.max_outside >= 0.0 else MAX_OUTSIDE
        try:
            transit = plan_transit(start, goal, drivable, obstacles, self.transit_config(values),
                                   float(request.goal_straight_length), max_outside)
        except PlanningError as e:
            response.success = False
            response.message = base.with_warnings(f'planning the transit failed: {e}', warnings)
            self.get_logger().warning(response.message)
            return response
        except Exception as e:  # noqa: BLE001 - a broken geometry must not take the node down
            response.success = False
            response.message = base.with_warnings(f'planning the transit failed: {type(e).__name__}: {e}', warnings)
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = base.with_warnings(f'transit {len(transit.poses)} poses, {transit.length:.2f} m, '
                                              f'{len(transit.spins)} turns on the spot',
                                              [*warnings, *transit.warnings])
        self.get_logger().info(response.message)
        response.path = base.to_path_msg(transit.poses, values['frame_id'], self.get_clock().now().to_msg())
        response.path_length = float(transit.length)
        response.spin_at = [int(i) for i in transit.spins]
        response.start_outside = float(transit.start_outside)
        response.goal_outside = float(transit.goal_outside)
        return response

    def transit_config(self, values: dict) -> PlannerConfig:
        """The settings a transit goes by: where the core is the planner.* settings (the body, the margins, the turns:
        as the plans' sections are got to, see arrives), else the node's parameters; what a request's overrides
        (values) change on top."""
        asked = self.config(values)
        if self.core is None:
            return asked
        plain = self.config(self.values())
        current = self.core.current
        return self.core.bound(current.base.with_overrides(dict({'mower_width': current.mower_width}, **{
            n: getattr(asked, n) for n in PlannerConfig.names() if getattr(asked, n) != getattr(plain, n)})))

    # planner.calib.evaluate

    def calib_evaluate(self, params: dict) -> dict:
        """planner.calib.evaluate, see the top."""
        path, root = params.get('path'), self.calib_root()
        if not isinstance(path, str) or not os.path.isabs(path):
            raise PlanningError('path must be an absolute path')
        real, top = os.path.realpath(path), os.path.realpath(root)
        if os.path.commonpath([real, top]) != top:
            raise PlanningError(f'path must lie under {root}')
        if not os.path.isfile(real):
            raise _internal(f'no recording at {path}')
        result = core.run_calib(real, timeout=CALIB_TIMEOUT)
        if 'error' in result:
            raise _internal(f'the calibration not evaluated: {result["error"]}')
        return result


class TransitCore(base.NodeCore):
    """NodeCore of role transit (see the top): no files of its own, no methods, nothing planned ahead; the planner.*
    settings the planner stored at source (only read, see follow)."""

    def __init__(self, node, mower_width: float, settings: dict, source: str):
        self.source, self.seen = source, None
        base.NodeCore.__init__(self, node, mower_width, settings, '', '', '', 0)
        self.methods = {}
        self.follow()

    def plan_ahead_later(self, changed=True):
        pass

    def follow(self):
        """The settings with the planner's stored ones from now on where source changed since (none: the launch's);
        one it can't read or that don't go together: as before, tried again next time."""
        if not self.source:
            return
        try:
            st = os.stat(self.source)
            seen = (st.st_mtime_ns, st.st_size, st.st_ino)
        except OSError:
            seen = None
        if seen == self.seen:
            return
        stored = {}
        try:
            if seen is not None:
                with open(self.source) as f:
                    stored = json.load(f)
                if not isinstance(stored, dict):
                    raise ValueError('not an object')
            known = set(PlannerConfig.names()) | set(core.NODE_SETTINGS) | set(INTERNAL) | set(RETIRED)
            with self.set_lock:
                self.current = self.setup(self.launch_width, self.launch, {
                    n: v for n, v in stored.items() if n in known})
        except (OSError, ValueError) as e:  # (PlanningError too)
            self.logwarn('MowBite Planner: the planner\'s settings not taken (%s): %s', self.source, e)
            return
        self.seen = seen


def geometry_values(params: dict) -> tuple:
    """(version, {setting: value}) of robot.geometry's params (bridge/params): GEOMETRY_KEYS as the planner's
    settings (robot_outline [] none: None, else ((x, y), ...)). ValueError where one is missing or isn't one."""
    version = params.get('version')
    if isinstance(version, bool) or not isinstance(version, int):
        raise ValueError(f'version must be an integer, got {version!r}')
    out = {}
    for name in GEOMETRY_KEYS:
        v = params.get(name)
        if name == 'robot_outline':
            if not isinstance(v, list) or not all(isinstance(p, list) and len(p) == 2 and all(_finite(c) for c in p)
                                                  for p in v):
                raise ValueError(f'robot_outline must be a list of [x, y], got {v!r}')
            out[name] = tuple((float(x), float(y)) for x, y in v) or None
        elif not _finite(v):
            raise ValueError(f'{name} must be a number, got {v!r}')
        else:
            out[name] = float(v)
    return version, out


def os_code() -> str:
    """A digest of this package's code (arrives and ~/plan_transit are here, not in mowbite_planner's code_version)."""
    global _OS_CODE
    if _OS_CODE is None:
        digest = hashlib.sha1()
        here = os.path.dirname(os.path.abspath(__file__))
        for name in sorted(os.listdir(here)):
            if name.endswith('.py'):
                with open(os.path.join(here, name), 'rb') as f:
                    digest.update(name.encode() + b'\0' + f.read())
        _OS_CODE = digest.hexdigest()
    return _OS_CODE


_OS_CODE = None


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _internal(message: str) -> PlanningError:
    """A PlanningError answered as JSON-RPC's internal error (not the caller's mistake)."""
    e = PlanningError(message)
    e.code = core.INTERNAL
    return e


def main(args=None):
    base.main(args, node_class=OsNode, executor_class=lambda: MultiThreadedExecutor(num_threads=THREADS))


if __name__ == '__main__':
    main()
