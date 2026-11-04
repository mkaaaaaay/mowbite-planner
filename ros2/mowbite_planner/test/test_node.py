"""Tests of the ROS 2 node: with a sourced ROS 2 workspace (colcon test) against the real rclpy and the
built interfaces, otherwise against the stand-ins in fake_ros.py."""

import hashlib
import json
import math
import os
import signal
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

try:
    import rclpy
    from mowbite_planner_interfaces.srv import PlanCoverage
    REAL_ROS = True
except ImportError:
    import fake_ros
    fake_ros.install()
    import rclpy
    from mowbite_planner_interfaces.srv import PlanCoverage
    REAL_ROS = False

from geometry_msgs.msg import Point, Point32, Polygon  # noqa: E402
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

import mowbite_planner  # noqa: E402
from mowbite_planner import core as core_module  # noqa: E402
from mowbite_planner import node as node_module  # noqa: E402
from mowbite_planner.config import PlanningError  # noqa: E402
from mowbite_planner.mathutil import angle_diff, quaternion_to_yaw  # noqa: E402
from mowbite_planner.node import MowbitePlannerNode, float32_warnings, pairs, parse_polygon  # noqa: E402
from stubplan import Spy  # noqa: E402

GARDEN = [(0.0, 0.0), (22.0, 0.0), (24.0, 9.0), (13.0, 9.0), (13.0, 16.0), (0.0, 16.0)]
BED = [(15.0, 2.5), (19.0, 2.5), (19.0, 5.0), (15.0, 5.0)]
TREE = [(5 + 0.9 * math.cos(a * math.pi / 8), 6 + 0.9 * math.sin(a * math.pi / 8)) for a in range(16)]
# a bed across the whole lawn splits it in two
LAWN = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
HEDGE = [(4.0, -1.0), (6.0, -1.0), (6.0, 7.0), (4.0, 7.0)]


def polygon(points):
    return Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points])


@pytest.fixture(autouse=True)
def planned(request, monkeypatch):
    """The node plans with the stand-in (stubplan.py), what it asked for in planned.calls; with the mark real_planner
    with plan_coverage itself."""
    spy = Spy()
    if 'real_planner' not in request.keywords:
        monkeypatch.setattr(node_module, 'plan_coverage', spy)
        monkeypatch.setattr(core_module, 'plan_coverage', spy)
    return spy


@pytest.fixture
def node():
    if REAL_ROS:
        rclpy.init()
    n = MowbitePlannerNode()
    yield n
    n.destroy_node()
    if REAL_ROS:
        rclpy.try_shutdown()


def call_plan_coverage(node, **fields):
    request = PlanCoverage.Request()
    request.boundary = polygon(fields.pop('boundary', GARDEN))
    request.obstacles = [polygon(o) for o in fields.pop('obstacles', [TREE, BED])]
    for k, v in fields.items():
        setattr(request, k, v)
    return node._on_plan_coverage(request, PlanCoverage.Response())


def override(name, value):
    """A request's override, rcl_interfaces/Parameter of the value's type (None: not set)."""
    type_ = Parameter.Type.from_parameter_value(value)
    fields = {} if type_ == Parameter.Type.NOT_SET else {type_.name.lower() + '_value': value}
    return ParameterMsg(name=name, value=ParameterValue(type=type_.value, **fields))


def xy(path):
    return [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]


def check_path(path, frame='map'):
    assert path.header.frame_id == frame
    assert len(path.poses) > 10
    for ps in path.poses:
        assert ps.header.frame_id == frame
        assert ps.header.stamp == path.header.stamp
        q = ps.pose.orientation
        assert q.x == 0.0 and q.y == 0.0
        assert math.isclose(q.z * q.z + q.w * q.w, 1.0, abs_tol=1e-9)
    for a, b in zip(path.poses, path.poses[1:]):
        pa, pb = a.pose.position, b.pose.position
        assert math.hypot(pb.x - pa.x, pb.y - pa.y) > 1e-7


@pytest.mark.real_planner  # (one of the four tests with the planner itself)
def test_plan_coverage_service(node):
    res = call_plan_coverage(node)
    assert res.success, res.message
    check_path(res.path)
    n = len(res.path.poses)
    assert res.path_length > 100 and res.lane_spacing == pytest.approx(0.22 * 0.85)
    assert 0.0 <= res.angle < math.pi
    # segments: in order, cover the path, consistent with the reverse flags
    starts, ends = list(res.segment_start), list(res.segment_end)
    assert starts[0] == 0 and ends[-1] == n - 1
    assert all(t in (e, e + 1) for e, t in zip(ends, starts[1:]))
    types = list(res.segment_type)
    assert set(types) <= {PlanCoverage.Response.SEGMENT_PERIMETER, PlanCoverage.Response.SEGMENT_LANE,
                          PlanCoverage.Response.SEGMENT_TURN, PlanCoverage.Response.SEGMENT_TRANSIT}
    assert types[0] == PlanCoverage.Response.SEGMENT_PERIMETER
    # backing up: the middle leg of a K-turn, between lanes or in place of a turn on the spot in any segment
    assert any(res.segment_reverse) == any(t == PlanCoverage.Response.SEGMENT_TURN and rev
                                           for t, rev in zip(types, res.segment_reverse)) or any(res.segment_reverse)
    # the orientation is the driving direction (or its opposite on reverse legs)
    reverse = [False] * n
    for s, e, rev in zip(starts, ends, res.segment_reverse):
        for i in range(s, e):
            reverse[i] = rev
    for i, (a, b) in enumerate(zip(res.path.poses, res.path.poses[1:])):
        pa, pb, q = a.pose.position, b.pose.position, a.pose.orientation
        travel = math.atan2(pb.y - pa.y, pb.x - pa.x)
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w) + (math.pi if reverse[i] else 0.0)
        assert angle_diff(travel, yaw) < math.radians(6)
    if not REAL_ROS:
        published = node.publishers['~/coverage_path'].published
        assert published == [res.path]


def test_plan_coverage_overrides(node):
    res = call_plan_coverage(node, mower_width=0.3, overlap=0.1, perimeter_passes=1, use_angle=True,
                             angle=math.radians(30))
    assert res.success, res.message
    assert res.lane_spacing == pytest.approx(0.27)
    assert res.angle == pytest.approx(math.radians(30))


def test_plan_coverage_start_pose(node):
    request = PlanCoverage.Request()
    request.boundary = polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)])
    request.use_start_pose = True
    request.start_pose.position.x = 10.5
    request.start_pose.position.y = 3.0
    request.start_pose.orientation.z = 1.0
    request.start_pose.orientation.w = 0.0
    res = node._on_plan_coverage(request, PlanCoverage.Response())
    assert res.success, res.message
    first = res.path.poses[0].pose.position
    assert first.x == pytest.approx(10.0 - 0.11) and abs(first.y - 3.0) < 0.35


def test_plan_coverage_bad_input(node):
    res = call_plan_coverage(node, boundary=[(0.0, 0.0), (1.0, 1.0)], obstacles=[])
    assert not res.success and res.message
    res = call_plan_coverage(node, overlap=1.5)
    assert not res.success and 'overlap' in res.message



def test_plan_coverage_concentric(node):
    res = call_plan_coverage(node, overrides=[override('fill_pattern', 'concentric')])
    assert res.success, res.message
    check_path(res.path)
    assert res.segment_type[0] == PlanCoverage.Response.SEGMENT_PERIMETER


def test_angle_offset_and_range(node):
    assert all(r.successful for r in node.set_parameters([
        Parameter('angle_offset_deg', value=30.0), Parameter('angle_min_deg', value=40.0),
        Parameter('angle_max_deg', value=50.0)]))
    res = call_plan_coverage(node, use_angle=True, angle=0.0)
    assert res.success and math.radians(40.0) - 1e-9 <= res.angle <= math.radians(50.0) + 1e-9


def test_many_loops(node):
    res = call_plan_coverage(node, overrides=[override('perimeter_passes', 60)])
    assert res.success, res.message
    check_path(res.path)
    assert node.set_parameters([Parameter('perimeter_passes', value=255)])[0].successful
    assert node.set_parameters([Parameter('lane_overlap_passes', value=255)])[0].successful


@pytest.mark.real_planner  # (one of the four tests with the planner itself)
def test_far_from_the_origin(node):
    # Point32 is float32: 20 km out it has 2 mm steps
    far = [(x + 10000.0, y + 20000.0) for x, y in GARDEN]
    res = call_plan_coverage(node, boundary=far, obstacles=[])
    assert res.success, res.message
    assert 'coordinates up to 20016 m from the map origin' in res.message and 'about 2 mm' in res.message
    assert 'float32' not in call_plan_coverage(node).message
    assert float32_warnings([[(8192.0, -8192.0)]]) == []
    assert 'about 1 mm' in float32_warnings([[(0.0, 0.0)], [(1.0, -8200.0)]])[0]
    assert 'about 500 mm' in float32_warnings([[(500000.0, 5500000.0)]])[0]


def test_plan_coverage_fingerprint_and_angles(node):
    res = call_plan_coverage(node)
    assert res.success, res.message
    assert len(res.fingerprint) == 64 and list(res.angles) == [res.angle]
    assert call_plan_coverage(node).fingerprint == res.fingerprint
    assert call_plan_coverage(node, mower_width=0.25).fingerprint != res.fingerprint


def test_plan_coverage_request_overrides(node):
    res = call_plan_coverage(node, overrides=[
        override('fill_pattern', 'crosshatch'), override('crosshatch_angle_deg', 60.0),
        override('angle_strategy', 'optimal'), override('angle_min_deg', 10.0), override('angle_max_deg', 20.0),
        override('perimeter_passes', 1), override('turn_types', ['u_turn', 'pivot'])])
    assert res.success, res.message
    assert len(res.angles) == 2 and res.angles[0] == res.angle
    assert math.radians(10.0) - 1e-9 <= res.angle <= math.radians(20.0) + 1e-9
    assert angle_diff(2 * res.angles[1], 2 * (res.angle + math.radians(60.0))) < 1e-6
    assert 'bulb' not in res.message and 'k-turn' not in res.message and not any(res.segment_reverse)
    # the request's fields go on top of the overrides, the node's parameters stay as they were
    res = call_plan_coverage(node, overrides=[override('mower_width', 0.4)], mower_width=0.3)
    assert res.success and res.lane_spacing == pytest.approx(0.3 * 0.85)
    assert node.get_parameter('fill_pattern').value == 'lanes'
    assert len(call_plan_coverage(node).angles) == 1


@pytest.mark.parametrize('msg', [
    override('blade_height', 0.05), override('turn_radius', 1), override('fill_pattern', 'spiral'),
    override('mower_width', 6.0), override('turn_types', ['u_turn', 'zigzag']), override('angle_min_deg', math.inf),
    override('overlap', None), override('perimeter_passes', 256),
])
def test_plan_coverage_bad_overrides(node, msg):
    res = call_plan_coverage(node, overrides=[override('turn_radius', 0.3), msg])
    assert not res.success and msg.name in res.message
    if not REAL_ROS:
        assert node.publishers['~/coverage_path'].published == []


def test_split_lawn(node):
    # no way from one half to the other: two sections, the path jumps between them
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[HEDGE])
    assert res.success and 'stub: 2 sections' in res.message, res.message  # (the plan's warnings)
    k = res.section_start[1]
    assert list(res.section_start) == [0, k] and 0 < k < len(res.path.poses)
    pts = xy(res.path)
    first, second = {x < 5.0 for x, _ in pts[:k]}, {x < 5.0 for x, _ in pts[k:]}
    assert len(first) == len(second) == 1 and first != second


def test_plan_coverage_drivable_areas(node):
    # the way round the hedge is outside the lawn
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[HEDGE],
                             drivable_areas=[polygon([(2.0, 4.0), (8.0, 4.0), (8.0, 10.0), (2.0, 10.0)])])
    assert res.success and 'jumps' not in res.message, res.message
    outside = [i for i, (x, y) in enumerate(xy(res.path)) if y > 6.0]
    assert outside
    for i in outside:
        k = max(k for k, s in enumerate(res.segment_start) if s <= i)
        assert res.segment_type[k] == PlanCoverage.Response.SEGMENT_TRANSIT


def test_plan_coverage_end_point(node):
    res = call_plan_coverage(node, use_end_point=True, end_point=Point(x=22.0, y=8.0, z=0.0),
                             overrides=[override('perimeter_order', 'last')])
    assert res.success, res.message
    last = res.path.poses[-1].pose.position
    assert math.hypot(last.x - 22.0, last.y - 8.0) < 1.0


def test_plan_from_parameters(node):
    res = node._on_plan_from_parameters(Trigger.Request(), Trigger.Response())
    assert not res.success  # nothing set yet
    flat = [float(c) for p in GARDEN for c in p]
    results = node.set_parameters([
        Parameter('boundary_polygon', Parameter.Type.DOUBLE_ARRAY, flat),
        Parameter('obstacles', Parameter.Type.STRING_ARRAY, ['15,2.5; 19,2.5; 19,5; 15,5']),
    ])
    assert all(r.successful for r in results)
    res = node._on_plan_from_parameters(Trigger.Request(), Trigger.Response())
    assert res.success, res.message
    if not REAL_ROS:
        published = node.publishers['~/coverage_path'].published
        assert len(published) == 1 and len(published[0].poses) > 100
        assert node.set_parameters([Parameter('publish_path', value=False)])[0].successful
        assert node._on_plan_from_parameters(Trigger.Request(), Trigger.Response()).success
        assert len(published) == 1
    node.set_parameters([Parameter('obstacles', Parameter.Type.STRING_ARRAY, ['1,2,3'])])
    res = node._on_plan_from_parameters(Trigger.Request(), Trigger.Response())
    assert not res.success and 'obstacles' in res.message


@pytest.mark.parametrize('name, value', [
    ('overlap', 1.0), ('mower_width', -0.1), ('lane_order', 'spiral'), ('angle_strategy', 'random'),
    ('turn_radius', -1.0), ('frame_id', ''), ('perimeter_passes', -2), ('fill_pattern', 'spiral'),
    ('crosshatch_angle_deg', math.nan), ('perimeter_order', 'middle'), ('perimeter_direction', 'up'),
    ('turn_types', ['u_turn', 'zigzag']), ('angle_min_deg', -math.inf), ('angle_max_deg', math.inf),
    ('angle_step_deg', 0.0), ('lane_overlap_passes', -1), ('perimeter_passes', 256),
    ('lane_overlap_passes', 256), ('headland_corners', 'round'),
])
def test_bad_parameters_are_refused(node, name, value):
    result = node.set_parameters([Parameter(name, value=value)])[0]
    assert not result.successful


def test_parameters_are_used(node):
    assert all(r.successful for r in node.set_parameters([
        Parameter('frame_id', value='garden'),
        Parameter('lane_order', value='skip'), Parameter('angle_strategy', value='fixed'),
        Parameter('fixed_angle_deg', value=45.0)]))
    res = call_plan_coverage(node)
    assert res.success, res.message
    check_path(res.path, 'garden')
    assert res.angle == pytest.approx(math.radians(45))
    assert not any(res.segment_reverse)


def test_new_parameters_are_used(node):
    assert all(r.successful for r in node.set_parameters([
        Parameter('fill_pattern', value='crosshatch'), Parameter('crosshatch_angle_deg', value=45.0),
        Parameter('angle_strategy', value='optimal'), Parameter('angle_step_deg', value=10.0),
        Parameter('angle_min_deg', value=30.0), Parameter('angle_max_deg', value=40.0),
        Parameter('perimeter_order', value='last'), Parameter('perimeter_direction', value='cw'),
        Parameter('turn_types', value=['detour', 'u_turn']), Parameter('lane_overlap_passes', value=1)]))
    res = call_plan_coverage(node)
    assert res.success, res.message
    assert len(res.angles) == 2 and math.radians(30.0) - 1e-9 <= res.angle <= math.radians(40.0) + 1e-9
    types = list(res.segment_type)
    assert types[-1] == PlanCoverage.Response.SEGMENT_PERIMETER and types[0] != types[-1]
    # a range needs both ends
    assert node.set_parameters([Parameter('angle_max_deg', value=math.nan)])[0].successful
    res = call_plan_coverage(node)
    assert not res.success and 'angle_min' in res.message


def test_headland_corners_parameter(node):
    assert node.config(node.values()).headland_corners == 'rounded'
    assert node.set_parameters([Parameter('headland_corners', value='sharp')])[0].successful
    assert node.config(node.values()).headland_corners == 'sharp'


def test_robot_outline_parameter(node):
    """robot_outline [x0, y0, x1, y1, ...]: the body's outline in place of the rectangle of robot_width, robot_front and
    robot_rear, also as a request's override; refused where it's no outline round base_link; empty: the rectangle."""
    assert node.config(node.values()).robot_outline is None
    flat = [0.43, -0.03, 0.43, 0.03, 0.26, 0.2, -0.14, 0.2, -0.14, -0.2, 0.26, -0.2]
    assert node.set_parameters([Parameter('robot_outline', value=flat)])[0].successful
    cfg = node.config(node.values())
    assert cfg.robot_outline == tuple(zip(flat[0::2], flat[1::2])) and cfg.body
    for bad in ([0.4, 0.2, -0.1], [1.0, 0.2, 0.5, 0.2, 0.5, -0.2], [0.4, 0.2, -0.1, -0.2, -0.1, 0.2, 0.4, -0.2]):
        assert not node.set_parameters([Parameter('robot_outline', value=bad)])[0].successful
    assert node.set_parameters([Parameter('robot_outline', Parameter.Type.DOUBLE_ARRAY, [])])[0].successful
    assert node.config(node.values()).robot_outline is None
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[], overrides=[override('robot_outline', flat)])
    assert res.success, res.message
    res = call_plan_coverage(node, boundary=LAWN, obstacles=[], overrides=[override('robot_outline', [1.0, 0.2])])
    assert not res.success and 'robot_outline' in res.message


def test_parsing():
    assert parse_polygon('1,2; 3,4;5 6') == [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    assert pairs([0.0, 1.0, 2.0, 3.0]) == [(0.0, 1.0), (2.0, 3.0)]
    with pytest.raises(ValueError):
        parse_polygon('1,2,3')
    with pytest.raises(ValueError):
        pairs([1.0])


# PlannerCore in the node (rpc, use_map): its callbacks called as rclpy would, the stand-ins only

fake_only = pytest.mark.skipif(REAL_ROS, reason='calls the callbacks of fake_ros\' node')
MAP_GARDEN = [(0.0, 0.0), (8.0, 0.0), (8.0, 5.0), (0.0, 5.0)]
MAP_BED = [(5.0, 2.0), (6.0, 2.0), (6.0, 3.0), (5.0, 3.0)]
CORE_METHODS = ['planner.angle.reset', 'planner.area.set', 'planner.drive', 'planner.leftovers', 'planner.log',
                'planner.plan', 'planner.settings', 'planner.settings.set']


def ring(points):
    return [{'x': x, 'y': y} for x, y in points + points[:1]]


def the_map(**props):
    garden = dict({'name': 'Garden', 'type': 'mow'}, **props)
    return {'areas': [{'id': 'a', 'outline': ring(MAP_GARDEN), 'properties': garden},
                      {'id': 'b', 'outline': ring(MAP_BED), 'properties': {'name': 'Bed', 'type': 'obstacle'}}],
            'docking_stations': [{'id': 'd', 'position': {'x': 7.5, 'y': 0.5}, 'heading': 0.0}]}


@pytest.fixture
def make(tmp_path, monkeypatch):
    """Nodes with PlannerCore (rpc and use_map on, data_dir in tmp_path), node_class: a robot's own."""
    monkeypatch.setattr(core_module, 'PLAN_AHEAD_DELAY', 1e6)  # (planned ahead only where a test says so)
    monkeypatch.setattr(core_module, 'TRACK_WAIT', 0.0)
    made = []

    def make(node_class=MowbitePlannerNode, **params):
        params = dict({'rpc': True, 'use_map': True, 'data_dir': str(tmp_path)}, **params)
        n = node_class(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        made.append(n)
        return n
    yield make
    for n in made:
        n.destroy_node()


def give(node, topic, text):
    node.subscribed[topic][1](String(data=text))


def send_map(node, data):
    text = json.dumps(data)
    give(node, '~/map', text)
    return text


def rpc(node, method, params=None, rid='r1', wait=60.0):
    """(result, None) or (None, (code, message)) of a request on ~/rpc/request."""
    give(node, '~/rpc/request', json.dumps({'method': method, 'params': params, 'id': rid}))
    end = time.time() + wait
    while time.time() < end:
        for msg in node.publishers['~/rpc/response'].published:
            got = json.loads(msg.data)
            if got['id'] == rid:
                return (got['result'], None) if 'result' in got else (None, (got['error']['code'],
                                                                             got['error']['message']))
        time.sleep(0.01)
    raise AssertionError(f'no answer to {method} ({rid})')


def plan_area(node, boundary=MAP_GARDEN, **fields):
    """~/plan_coverage as a robot's logic asks it for area a: its outline and the map's obstacles."""
    request = PlanCoverage.Request()
    request.boundary = polygon(boundary)
    request.obstacles = [polygon(MAP_BED)]
    for k, v in fields.items():
        setattr(request, k, v)
    return node._on_plan_coverage(request, PlanCoverage.Response())


@fake_only
def test_core_with_rpc_or_use_map(make):
    """PlannerCore only with rpc or use_map: then the map on ~/map, the events on ~/events, with rpc the requests on
    ~/rpc/request."""
    assert make(rpc=False, use_map=False).core is None
    node = make()
    assert sorted(node.core.methods) == CORE_METHODS
    assert {'~/map', '~/events', '~/rpc/request'} <= set(node.subscribed)
    assert node.subscribed['~/map'][2].durability == fake_ros._Policy.TRANSIENT_LOCAL
    quiet = make(rpc=False)
    assert quiet.core is not None and '~/rpc/request' not in quiet.subscribed


@fake_only
def test_settings_over_rpc(make, tmp_path):
    """The node's parameters are the defaults, planner.settings.set is kept in data_dir/planner, the caller's
    mistakes come back as errors with their codes; what this node can't do is ignored with why, not settable."""
    node = make(turn_radius=0.3)
    got, error = rpc(node, 'planner.settings')
    assert error is None and got['settings']['turn_radius']['value'] == 0.3
    assert got['version'] == mowbite_planner.__version__
    for name, why in MowbitePlannerNode.cannot.items():
        assert got['settings'][name]['ignored'] == why and not got['settings'][name]['settable'], name
    got, error = rpc(node, 'planner.settings.set', {'fill_pattern': 'concentric'}, 'r2')
    assert error is None and got['settings']['fill_pattern']['stored']
    with open(tmp_path / 'planner' / 'mowbite_planner_settings.json') as f:
        assert json.load(f) == {'fill_pattern': 'concentric'}
    assert rpc(node, 'planner.settings.set', {'turn_radius': -1}, 'r3')[1][0] == -32602
    assert rpc(node, 'planner.settings.set', {'leftovers_auto': True}, 'r4')[1] == (
        -32602, 'not settable here: leftovers_auto (this node doesn\'t send the robot out)')
    assert rpc(node, 'planner.settings', [1], 'r5')[1] == (-32602, 'params must be an object (by name)')
    assert rpc(node, 'planner.nothing', {}, 'r6')[1] == (-32601, 'Method "planner.nothing" not found')
    give(node, '~/rpc/request', 'not json')
    assert rpc(node, 'planner.log', {'limit': 50}, 'r7')[0]['lines'][-1]['msg'].startswith(
        'MowBite Planner: planner.settings: params must be an object')
    again = make(turn_radius=0.3)
    assert again.core.current.base.fill_pattern == 'concentric'


@fake_only
def test_plan_coverage_of_an_area_is_planner_plans(make):
    """~/plan_coverage of an area of the map: the plan planner.plan gives (the same fingerprint, kept), with the
    planner.* settings and the area's own; an outline of no area the same way; the request's lane spacing and
    overrides of no area's left out with a warning."""
    node = make()
    send_map(node, the_map())
    first = plan_area(node)
    assert first.success and first.message.startswith('area a, planned: ')
    again = plan_area(node)
    assert again.message.startswith('area a, kept from before: ') and again.fingerprint == first.fingerprint
    got, _ = rpc(node, 'planner.plan', {'area_id': 'a'})
    assert got['kept'] is True and got['fingerprint'] == first.fingerprint
    rpc(node, 'planner.settings.set', {'fill_pattern': 'concentric', 'perimeter_passes': 3}, 'r2')
    concentric = plan_area(node)
    assert concentric.fingerprint != first.fingerprint
    assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r3')[0]['fingerprint'] == concentric.fingerprint
    send_map(node, the_map(planner={'fill_pattern': 'lanes'}, outline_count=2))
    assert plan_area(node).fingerprint == first.fingerprint
    odd = plan_area(node, mower_width=0.3, overlap=0.4, overrides=[override('turn_radius', 0.5)])
    assert odd.fingerprint == first.fingerprint
    assert 'overrides left out, the planner.* settings count: turn_radius' in odd.message
    assert "the request's lane spacing 0.180 m left out" in odd.message
    assert plan_area(node, perimeter_passes=1).fingerprint != first.fingerprint
    other = plan_area(node, boundary=[(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)])
    assert other.success and other.message.startswith('outline, planned: ')
    got, _ = rpc(node, 'planner.plan', {'outline': [[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [0.0, 4.0]],
                                        'obstacles': [[list(p) for p in MAP_BED]]}, 'r4')
    assert got['kept'] is True and got['fingerprint'] == other.fingerprint


@fake_only
def test_plan_coverage_angle(make):
    """The angle: the area's (or a preview's, use_angle), else the planner's direction; the planner.* angle_offset on
    top, a request's angle_offset_deg (a job's own) on top of that."""
    node = make()
    send_map(node, the_map())
    plain = plan_area(node)
    fixed = plan_area(node, use_angle=True, angle=0.5)
    assert abs(math.remainder(fixed.angle - 0.5, math.pi)) < 1e-6
    turned = plan_area(node, overrides=[override('angle_offset_deg', 30.0)])
    assert abs(math.remainder(turned.angle - plain.angle - math.radians(30.0), math.pi)) < 1e-6
    rpc(node, 'planner.settings.set', {'angle_offset': math.radians(30.0)})
    got, _ = rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')
    assert got['fingerprint'] == turned.fingerprint == plan_area(node).fingerprint
    send_map(node, the_map(angle=0.3))
    own = plan_area(node)
    assert abs(math.remainder(own.angle - 0.3 - math.radians(30.0), math.pi)) < 1e-6


class Robot(MowbitePlannerNode):
    """A robot's own node: it writes the map (with the text it was based on), says when the map can't be planned,
    and where its parameters are."""

    defaults = {'turn_radius': 0.35}
    cannot = {}
    unusable = ''
    refuse = None

    def replace_map(self, data, based_on):
        if self.refuse:  # (as another node refuses it, its code)
            e = PlanningError(self.refuse[1])
            e.code = self.refuse[0]
            raise e
        self.written = (data, based_on)
        give(self, '~/map', json.dumps(data))

    def map_unusable(self):
        return self.unusable

    def robot_param(self, name, default=None):
        return {'/mower_logic': {'outline_count': 7}}.get(name, default)


@fake_only
def test_area_set_one_place_for_each_value(make):
    """planner.area.set: what OpenMower has an area property for into that one, "the planner works it out" into the
    planner property in place of it; both there (OpenMower's set again): OpenMower's counts. Without a way to write
    the map: an error."""
    assert rpc(make(), 'planner.area.set', {'area_id': 'a', 'settings': {'turn_radius': 0.3}})[1] == (
        -32602, 'no map from OpenMower yet')
    plain = make()
    send_map(plain, the_map())
    assert rpc(plain, 'planner.area.set', {'area_id': 'a', 'settings': {'turn_radius': 0.3}})[1] == (
        -32602, 'the map can\'t be written here')
    node = make(Robot)
    text = send_map(node, the_map())

    def area_set(settings, rid):
        got, error = rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': settings}, rid)
        assert error is None, error
        return {k: v for k, v in got['properties'].items() if k not in ('name', 'type')}
    assert area_set({'perimeter_passes': 4, 'lane_overlap_passes': 1, 'perimeter_offset': 0.05, 'angle_min': 0.1,
                     'angle_max': 0.6, 'turn_radius': 0.3}, 'r1') == {
        'outline_count': 4, 'outline_overlap_count': 1, 'outline_offset': 0.05, 'angle_min': 0.1, 'angle_max': 0.6,
        'planner': {'turn_radius': 0.3}}
    assert json.loads(json.dumps(node.written[0]))['areas'][0]['properties']['outline_count'] == 4
    assert node.written[1] == text
    assert area_set({'perimeter_passes': -1, 'angle': 0.4}, 'r2') == {
        'outline_overlap_count': 1, 'outline_offset': 0.05, 'angle_min': 0.1, 'angle_max': 0.6, 'angle': 0.4,
        'planner': {'turn_radius': 0.3, 'perimeter_passes': -1}}
    assert area_set({'angle_strategy': 'optimal', 'perimeter_passes': None, 'lane_overlap_passes': None,
                     'perimeter_offset': None, 'angle_min': None, 'angle_max': None, 'turn_radius': None}, 'r3') == {
        'planner': {'angle_strategy': 'optimal'}}
    assert area_set({'angle': 0.2}, 'r4') == {'angle': 0.2}
    for bad in ({'perimeter_passes': 'x'}, {'angle_min': 'x'}, {'lane_overlap_passes': -3}):
        assert rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': bad}, 'r5' + str(bad))[1][0] == -32602
    send_map(node, the_map(outline_count=2, angle=0.3, planner={'perimeter_passes': -1, 'angle_strategy': 'optimal',
                                                                'fill_pattern': 'concentric'}))
    clean = node.core.plan_map(node.core.current_map())
    assert clean['areas'][0]['properties']['planner'] == {'fill_pattern': 'concentric'}
    assert node.core.current_map()['areas'][0]['properties']['planner']['perimeter_passes'] == -1
    node.refuse = (-32001, 'the map changed meanwhile')
    assert rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': {'angle': 0.1}}, 'r6')[1] == (
        -32001, 'the map changed meanwhile')


@fake_only
def test_a_robots_own_node(make):
    """A robot's node: its defaults, its settings it can't have, its parameters (mower_logic's never count), the map
    it can't plan now (planner.plan's error, ~/plan_coverage's, nothing planned ahead)."""
    node = make(Robot)
    got, _ = rpc(node, 'planner.settings')
    assert got['settings']['turn_radius']['value'] == 0.35 and got['settings']['leftovers_auto']['settable']
    assert got['settings']['perimeter_passes']['value'] == 2 and 'from' not in got['settings']['perimeter_passes']
    send_map(node, the_map())
    node.unusable = 'the map is being moved'
    assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')[1] == (-32602, 'the map is being moved')
    res = plan_area(node)
    assert not res.success and res.message == 'area a: the map is being moved'
    node.core.plan_ahead()
    assert not node.core.plans
    node.unusable = ''
    assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r3')[1] is None


@fake_only
def test_arrives_in_the_plans_key(make, planned):
    """A robot's arrives: the plan with it planned and kept apart from the one without; its key (where it has one) is
    part of the plan's key, so one kept with another key isn't taken."""
    node = make(Robot)
    send_map(node, the_map())

    def arrives(key):
        def gets_to(pose, came_from):
            return True
        if key is not None:
            gets_to.key = key
        return lambda *args: gets_to

    kept = []
    for rid, how in enumerate((None, arrives(None), arrives('other'), arrives('other'), arrives(None))):
        if how is not None:
            node.arrives = how
        assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r%d' % rid)[1] is None
        kept.append(len(node.core.plans))
    assert callable(planned.calls[-1][1]['arrives'])
    assert kept == [1, 2, 3, 3, 3]


@fake_only
def test_events_and_progress(make):
    """The events on ~/events turn the lanes (angle_increment); a planning's progress goes out on ~/publish."""
    node = make()
    send_map(node, the_map())
    rpc(node, 'planner.settings.set', {'angle_increment': 0.1, 'angle_increment_every': 1})
    give(node, '~/events', json.dumps({'type': 'JOB_COMPLETE', 'job_id': 'j1', 'x': 1.0, 'y': 2.0}))
    node.core.drive_thread.join(60.0)
    give(node, '~/events', 'not json')
    assert node.core.angle == {'steps': 1, 'mows': 0, 'finished': 1}
    rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')
    out = [json.loads(m.data) for m in node.publishers['~/publish'].published]
    assert out and {m['topic'] for m in out} == {'mowbite/planner/progress'} and all(m['retain'] for m in out)
    assert json.loads(out[-1]['payload'])['area_id'] == 'a'


@fake_only
def test_armed_leftovers_with_none_here(make):
    """An area the armed leftovers have nothing in gets a plan without paths, a success: the robot's job goes on
    with the next area (like mower_logic with no paths)."""
    node = make()
    send_map(node, the_map())
    node.core.armed = {'until': time.time() + 60.0, 'plans': {'a': None}, 'served': False}
    got = plan_area(node)
    assert got.success and got.path.poses == [] and got.segment_type == [] and node.core.armed['served']
    assert got.fingerprint == hashlib.sha256().hexdigest() and 'nothing to mow' in got.message


def test_launch_settings():
    """The node's parameters as planner.* settings, the lane spacing and the loops too (not the blade, it's
    PlannerCore's own)."""
    values = dict({name: d for name, (d, _, _, _) in node_module.PARAMETERS.items()}, turn_radius=0.3,
                  lane_order='skip', overlap=0.3, robot_outline=[])
    got = node_module.launch_settings(MowbitePlannerNode.config(values))
    assert got['turn_radius'] == 0.3 and got['overlap'] == 0.3 and 'lane_order' not in got and 'mower_width' not in got


def test_prune(tmp_path):
    """data_dir/planner within keep_days and keep_mb: the plans kept and the maps before go, the oldest first."""
    plans, backups = tmp_path / 'plans', tmp_path / 'mowbite_map_backups'
    plans.mkdir()
    backups.mkdir()
    now = time.time()
    files = {plans / 'old.pkl': (100, 90), plans / 'a.pkl': (600_000, 3), backups / 'map_1.json': (300_000, 2),
             plans / 'b.pkl': (300_000, 1), tmp_path / 'mowbite_planner_settings.json': (10, 200)}
    for path, (size, days) in files.items():
        path.write_bytes(b'x' * size)
        os.utime(path, (now - days * 86400.0, now - days * 86400.0))
    gone = node_module.prune(str(tmp_path), 60, 1.0, now)
    assert gone == [str(plans / 'old.pkl'), str(plans / 'a.pkl')]
    assert sorted(p.name for p in tmp_path.rglob('*') if p.is_file()) == ['b.pkl', 'map_1.json',
                                                                          'mowbite_planner_settings.json']


@fake_only
@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_a_signal_ends_it(make, monkeypatch, sig):
    """main: SIGINT or SIGTERM end it, not rclpy's handlers (they'd shut the context down first): destroy_node's
    rpc_end before rclpy shuts down."""
    node = make()
    handlers, calls = {}, []

    class Executor:
        def add_node(self, n):
            assert n is node

        def spin_once(self, timeout_sec=None):
            calls.append('spin')
            handlers[sig](sig, None)

    monkeypatch.setattr(node_module.signal, 'signal', lambda s, h: handlers.__setitem__(s, h))
    monkeypatch.setattr(node_module.rclpy, 'init', lambda args=None, **kw: calls.append(kw))
    monkeypatch.setattr(node_module.rclpy, 'try_shutdown', lambda: calls.append('shutdown'))
    monkeypatch.setattr(node_module, 'SingleThreadedExecutor', Executor)
    monkeypatch.setattr(node, 'rpc_end', lambda: calls.append('rpc_end'))
    node_module.main(node_class=lambda: node)
    assert calls == [{'signal_handler_options': node_module.SignalHandlerOptions.NO}, 'spin', 'rpc_end', 'shutdown']
    assert set(handlers) == {signal.SIGINT, signal.SIGTERM}


def test_version_in_one_place():
    """The package's version is mowbite_planner.__version__ (planner.settings says it), setup.py and package.xml the
    same."""
    here = os.path.join(HERE, '..')
    with open(os.path.join(here, 'setup.py')) as f:
        assert f"version='{mowbite_planner.__version__}'," in f.read()
    with open(os.path.join(here, 'package.xml')) as f:
        assert f'<version>{mowbite_planner.__version__}</version>' in f.read()


def test_files_version():
    """One number for the files kept next to the settings file, in the package (MowBite OS' table goes by it)."""
    assert type(mowbite_planner.FILES_VERSION) is int and mowbite_planner.FILES_VERSION >= 1
