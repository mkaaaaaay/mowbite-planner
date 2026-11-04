"""Tests of the node on MowBite OS (OsNode): with a sourced ROS 2 workspace (colcon test) against the real rclpy,
mowbite_msgs and interfaces, otherwise against the stand-ins (osfake.py)."""

import gzip
import hashlib
import json
import math
import os
import threading
import time

import pytest

from osfake import HERE, REAL_ROS

import rclpy  # noqa: E402
from geometry_msgs.msg import Point32, Polygon, Pose  # noqa: E402
from mowbite_msgs.msg import MapStatus, MqttPublish, RpcRequest  # noqa: E402
from mowbite_msgs.srv import ReplaceMap  # noqa: E402
from mowbite_planner_interfaces.srv import PlanCoverage  # noqa: E402
from mowbite_planner_os_interfaces.srv import PlanTransit  # noqa: E402
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import calibsim  # noqa: E402
import mowbite_planner  # noqa: E402
from mowbite_planner import core as core_module  # noqa: E402
from mowbite_planner import node as node_module  # noqa: E402
from mowbite_planner.calib import evaluate  # noqa: E402
from mowbite_planner.config import PlannerConfig, PlanningError  # noqa: E402
from mowbite_planner.starts import _Spent  # noqa: E402
from mowbite_planner.mathutil import Pose2D, yaw_to_quaternion  # noqa: E402
from mowbite_planner_os import node as os_module  # noqa: E402
from mowbite_planner_os.node import OsNode  # noqa: E402
from stubplan import Spy  # noqa: E402

fake_only = pytest.mark.skipif(REAL_ROS, reason='looks at the stand-ins\' topics')

GARDEN = [(0.0, 0.0), (8.0, 0.0), (8.0, 5.0), (0.0, 5.0)]
BED = [(5.0, 2.0), (6.0, 2.0), (6.0, 3.0), (5.0, 3.0)]
CORE_METHODS = ['planner.angle.reset', 'planner.area.set', 'planner.drive', 'planner.leftovers', 'planner.log',
                'planner.plan', 'planner.settings', 'planner.settings.set']
# the Tron's body (mowbite-os, tron.yaml)
TRON = {'mower_width': 0.22, 'overlap': 0.15, 'perimeter_passes': 3, 'turn_radius': 0.44, 'lane_order': 'skip',
        'turn_on_spot': False, 'turn_types': ['u_turn', 'bulb', 'detour'], 'robot_width': 0.47, 'robot_front': 0.60,
        'robot_rear': 0.12, 'blade_ahead': 0.23, 'keep_days': 60, 'keep_mb': 50.0}
# the Tron's case of 30.10. (mowbite-os 8a66cdb): the first section of a plan started where no way got to it
TRON_CASE = os.path.join(HERE, 'transit-case-8a66cdb.json')


def ring(points):
    return [{'x': x, 'y': y} for x, y in points + points[:1]]


def the_map(**props):
    garden = dict({'name': 'Garden', 'type': 'mow'}, **props)
    return {'areas': [{'id': 'a', 'outline': ring(GARDEN), 'properties': garden},
                      {'id': 'b', 'outline': ring(BED), 'properties': {'name': 'Bed', 'type': 'obstacle'}}],
            'docking_stations': [{'id': 'd', 'position': {'x': 7.5, 'y': 0.5}, 'heading': 0.0}]}


def polygon(points):
    return Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in points])


def pose(x, y, yaw):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    _, _, p.orientation.z, p.orientation.w = yaw_to_quaternion(yaw)
    return p


@pytest.fixture(autouse=True)
def planned(request, monkeypatch):
    """The node plans with the stand-in (mowbite_planner's stubplan.py), with the mark real_planner with
    plan_coverage itself."""
    spy = Spy()
    if 'real_planner' not in request.keywords:
        monkeypatch.setattr(node_module, 'plan_coverage', spy)
        monkeypatch.setattr(core_module, 'plan_coverage', spy)
    return spy


@pytest.fixture
def make(tmp_path, monkeypatch):
    """OsNodes with data_dir in tmp_path."""
    monkeypatch.setattr(core_module, 'PLAN_AHEAD_DELAY', 1e6)  # (planned ahead only where a test says so)
    monkeypatch.setattr(core_module, 'TRACK_WAIT', 0.0)
    if REAL_ROS:
        rclpy.init()
    made = []

    def make(**params):
        params = dict({'data_dir': str(tmp_path / 'data')}, **params)
        n = OsNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        made.append(n)
        return n
    yield make
    for n in made:
        n.destroy_node()
    if REAL_ROS:
        rclpy.try_shutdown()


def give(node, topic, msg):
    node.subscribed[topic][1](msg)


def send_map(node, data, status=MapStatus.DATUM_MATCH):
    text = json.dumps(data)
    give(node, 'map/json', String(data=text))
    if status is not None:
        give(node, 'map/status', MapStatus(datum_state=status, mowing_allowed=True, reason=''))
    return text


def rpc(node, method, params=None, rid='r1', wait=60.0):
    """(result, None) or (None, (code, message)) of a request on bridge/rpc/request."""
    give(node, 'bridge/rpc/request', RpcRequest(method=method, params=json.dumps(params) if params is not None else '',
                                                id=rid))
    return answer(node, rid, wait)


def answer(node, rid, wait=60.0):
    """The answer to the request rid, see rpc."""
    end = time.time() + wait
    while time.time() < end:
        for msg in node.publishers['bridge/rpc/response'].published:
            if msg.id == rid:
                return json.loads(msg.result), None
        for msg in node.publishers['bridge/rpc/error'].published:
            if msg.id == rid:
                assert msg.data == ''
                return None, (msg.code, msg.message)
        time.sleep(0.01)
    raise AssertionError(f'no answer to {rid}')


def plan_area(node, boundary=GARDEN):
    """~/plan_coverage as mowbite_logic asks it for area a: its outline and the map's obstacles."""
    request = PlanCoverage.Request()
    request.boundary = polygon(boundary)
    request.obstacles = [polygon(BED)]
    return node._on_plan_coverage(request, PlanCoverage.Response())


def override(name, value):
    """A request's override, rcl_interfaces/Parameter of the value's type."""
    type_ = Parameter.Type.from_parameter_value(value)
    return ParameterMsg(name=name, value=ParameterValue(type=type_.value, **{type_.name.lower() + '_value': value}))


def transit(node, start, goal, drivable=(GARDEN,), obstacles=(BED,), **fields):
    request = PlanTransit.Request(start=pose(*start), goal=pose(*goal), use_goal_heading=True,
                                  drivable_areas=[polygon(a) for a in drivable],
                                  obstacles=[polygon(o) for o in obstacles])
    for k, v in fields.items():
        setattr(request, k, v)
    return node._on_plan_transit(request, PlanTransit.Response())


@fake_only
def test_methods_on_the_bridge(make):
    """Registered on bridge/rpc/methods (latched, node_id mowbite_planner), asked on bridge/rpc/request, answered on
    bridge/rpc/response or bridge/rpc/error; another node's method not answered; the empty list as it ends. What MowBite
    OS can't have: ignored with why."""
    node = make()
    methods = node.publishers['bridge/rpc/methods']
    assert methods.qos.depth == 50 and methods.qos.durability.name == 'TRANSIENT_LOCAL'
    assert [(m.node_id, list(m.methods)) for m in methods.published] == [('mowbite_planner', CORE_METHODS)]
    assert node.subscribed['bridge/rpc/request'][2] == 100
    got, error = rpc(node, 'planner.settings')
    assert error is None and got['version'] == mowbite_planner.__version__
    for name, why in OsNode.cannot.items():
        assert why.startswith('MowBite OS: ')
        assert got['settings'][name]['ignored'] == why and not got['settings'][name]['settable'], name
    assert got['settings']['drive']['ignored'] == 'MowBite OS: drives forwards only for now (M8)'
    assert got['settings']['turn_radius']['value'] == 0.25  # (the node's parameters the defaults)
    assert rpc(node, 'planner.settings.set', {'turn_radius': -1}, 'r2')[1][0] == -32602
    assert rpc(node, 'planner.settings', [1], 'r3')[1] == (-32602, 'params must be an object (by name)')
    give(node, 'bridge/rpc/request', RpcRequest(method='map.replace', params='[]', id='r4'))
    time.sleep(0.2)
    assert not [m for m in node.publishers['bridge/rpc/error'].published + node.publishers[
        'bridge/rpc/response'].published if m.id == 'r4']
    node.destroy_node()
    assert [(m.node_id, list(m.methods)) for m in methods.published][-1] == ('mowbite_planner', [])


@fake_only
def test_map_and_its_status(make, monkeypatch):
    """Plans only while map/status says MATCH, else an error with its reason; on MATCH again planned ahead."""
    node = make()
    ahead = []
    monkeypatch.setattr(node.core, 'plan_ahead_later', lambda: ahead.append(1))
    give(node, 'map/json', String(data=json.dumps(the_map())))
    assert rpc(node, 'planner.plan', {'area_id': 'a'})[1] == (
        -32602, 'no map status from mowbite_map yet (map/status)')
    give(node, 'map/status', MapStatus(datum_state=MapStatus.DATUM_MISMATCH, reason='another datum'))
    assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')[1] == (
        -32602, 'the map can\'t be planned now (map/status MISMATCH): another datum')
    res = plan_area(node)
    assert not res.success and 'map/status MISMATCH' in res.message
    ahead.clear()
    give(node, 'map/status', MapStatus(datum_state=MapStatus.DATUM_MATCH))
    give(node, 'map/status', MapStatus(datum_state=MapStatus.DATUM_MATCH))
    assert ahead == [1]
    got, error = rpc(node, 'planner.plan', {'area_id': 'a'}, 'r3')
    assert error is None and got['fingerprint'] == plan_area(node).fingerprint


@fake_only
def test_area_set_over_map_replace(make, monkeypatch):
    """planner.area.set writes the map with map/replace (expected_sha256 the digest of the text it changed), waits for
    the map/json that follows; mowbite_map's no comes back with its code; without mowbite_map an error."""
    monkeypatch.setattr(os_module, 'MAP_FIND', 0.0)
    monkeypatch.setattr(os_module, 'MAP_SETTLE', 0.2)
    node = make()
    text = send_map(node, the_map())
    assert rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': {'turn_radius': 0.3}})[1] == (
        -32602, 'mowbite_map is not there (map/replace)')
    client = node.called['map/replace']
    asked = []

    def stored(request):
        asked.append(request)
        give(node, 'map/json', String(data=request.map))
        return ReplaceMap.Response(success=True, code=0, message='Successfully stored map (2 areas)')
    client.handler = stored
    got, error = rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': {'turn_radius': 0.3}}, 'r2')
    assert error is None and got['properties']['planner'] == {'turn_radius': 0.3}
    assert asked[0].expected_sha256 == hashlib.sha256(text.encode()).hexdigest()
    assert json.loads(asked[0].map)['areas'][0]['properties']['planner'] == {'turn_radius': 0.3}
    assert node.core.map_text == asked[0].map and node.map_count == 2
    client.handler = lambda request: ReplaceMap.Response(success=False, code=-32001,
                                                         message='the map changed meanwhile')
    assert rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': {'turn_radius': 0.4}}, 'r3')[1] == (
        -32001, 'mowbite_map: the map changed meanwhile')
    client.handler = lambda request: ReplaceMap.Response(success=False, code=0, message='')
    assert rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': {'turn_radius': 0.4}}, 'r4')[1] == (
        -32602, 'mowbite_map: map not written')


@fake_only
def test_events_progress_and_finished_mows(make, planned):
    """mowbite_logic's events from bridge/publish (events/json only): the finished mows counted and handed to the plans
    (perimeter_direction random); the planning's progress out on bridge/publish."""
    node = make()
    send_map(node, the_map())
    rpc(node, 'planner.settings.set', {'perimeter_direction': 'random', 'angle_increment': 0.1,
                                       'angle_increment_every': 1})
    give(node, 'bridge/publish', MqttPublish(topic='other/json', payload=json.dumps({'type': 'JOB_COMPLETE'})))
    assert node.core.angle['finished'] == 0
    give(node, 'bridge/publish', MqttPublish(topic='events/json', payload=json.dumps(
        {'type': 'JOB_COMPLETE', 'job_id': 'j1', 'x': 1.0, 'y': 2.0})))
    node.core.drive_thread.join(60.0)
    assert node.core.angle == {'steps': 1, 'mows': 0, 'finished': 1}
    assert plan_area(node).success and planned.config().finished_mows == 1
    rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')
    assert planned.config().finished_mows == 1
    got, _ = rpc(node, 'planner.settings', None, 'r3')
    assert got['finished_mows'] == 1
    out = [m for m in node.publishers['bridge/publish'].published if m.topic == 'mowbite/planner/progress']
    assert out and all(m.retain for m in out) and json.loads(out[-1].payload)['area_id'] == 'a'


@fake_only
def test_follower_speeds(make):
    """mowbite_follower's speeds from bridge/params for the time a plan takes (max_angular as deg/s), nothing else."""
    node = make()
    assert node.robot_param(core_module.FTC_PARAMS, {}) == {}
    params = node.subscribed['bridge/params']
    assert params[2].depth == 50 and params[2].durability.name == 'TRANSIENT_LOCAL'
    give(node, 'bridge/params', String(data=json.dumps({'node': '/ll', 'params': {'board': 'Tron'}})))
    give(node, 'bridge/params', String(data=json.dumps(
        {'node': '/mowbite_follower', 'params': {'speed_fast': 0.5, 'speed_slow': 0.2, 'max_angular': 1.0}})))
    give(node, 'bridge/params', String(data='not json'))
    assert node.core.param(core_module.FTC_PARAMS, {}) == {'speed_fast': 0.5, 'speed_slow': 0.2,
                                                          'speed_angular': pytest.approx(math.degrees(1.0))}
    assert node.core.param('/mower_logic', {}) == {}
    assert 'bridge/params not readable' in node.get_logger().lines[-1][1]


@fake_only
def test_calib_evaluate(make, tmp_path, monkeypatch):
    """planner.calib.evaluate only with calib_root: a recording under it (absolute) evaluated in a process of its own,
    the whole mowbite-calib/1 object; outside it or not absolute -32602, none there or one it can't read -32603; the
    other methods answered meanwhile."""
    plain = make()
    assert 'planner.calib.evaluate' not in plain.core.methods
    give(plain, 'bridge/rpc/request', RpcRequest(method='planner.calib.evaluate', params='{"path": "/x"}', id='p1'))
    time.sleep(0.2)
    assert not plain.publishers['bridge/rpc/error'].published
    root = tmp_path / 'calibration'
    (root / 'recordings').mkdir(parents=True)
    node = make(calib_root=str(root))
    assert list(node.publishers['bridge/rpc/methods'].published[0].methods) == sorted(
        CORE_METHODS + ['planner.calib.evaluate'])
    mower = calibsim.program(0)
    path = mower.recording(str(root / 'recordings' / 'calib-20261104T010203Z.jsonl.gz'))
    got, error = rpc(node, 'planner.calib.evaluate', {'path': path}, 'r1', wait=120.0)
    assert error is None and got == json.loads(json.dumps(evaluate(path)))
    assert got['version'] == 'mowbite-calib/1' and {'ready', 'values', 'warnings', 'based_on', 'info'} <= set(got)
    with open(os.path.join(HERE, '..', 'examples', 'calib-evaluate.json')) as f:  # (the example as it comes)
        example = json.load(f)
    assert set(example['response']['result']) == set(got)
    assert [v['name'] for v in example['response']['result']['values']] == [v['name'] for v in got['values']]
    outside = tmp_path / 'elsewhere.jsonl.gz'
    outside.write_bytes(b'')
    os.symlink(str(outside), str(root / 'link.jsonl.gz'))
    for rid, params in (('r2', {'path': 'recordings/calib.jsonl.gz'}), ('r3', {'path': str(outside)}),
                        ('r4', {'path': str(root / '..' / 'elsewhere.jsonl.gz')}), ('r5', {}),
                        ('r6', {'path': str(root / 'link.jsonl.gz')})):
        assert rpc(node, 'planner.calib.evaluate', params, rid)[1][0] == -32602, params
    assert rpc(node, 'planner.calib.evaluate', {'path': str(root / 'none.jsonl.gz')}, 'r7')[1] == (
        -32603, 'no recording at %s' % (root / 'none.jsonl.gz'))
    broken = root / 'recordings' / 'broken.jsonl.gz'
    with gzip.open(str(broken), 'wt') as f:
        f.write('{"not": "a recording"}\n')
    code, message = rpc(node, 'planner.calib.evaluate', {'path': str(broken)}, 'r8', wait=120.0)[1]
    assert code == -32603 and message.startswith('the calibration not evaluated: ')
    # (CALIB_TIMEOUT s at most, on a thread of its own)
    asked, go = [], threading.Event()
    monkeypatch.setattr(core_module, 'run_calib', lambda p, interim=False, timeout=None: asked.append(timeout) or
                        go.wait(30.0) and {'error': 'the evaluation took over %.0f s' % timeout})
    give(node, 'bridge/rpc/request', RpcRequest(method='planner.calib.evaluate', params=json.dumps({'path': path}),
                                                id='r9'))
    assert rpc(node, 'planner.settings', None, 'r10', wait=10.0)[1] is None
    go.set()
    assert answer(node, 'r9')[1] == (
        -32603, 'the calibration not evaluated: the evaluation took over 120 s')
    assert asked == [120.0]


@fake_only
def test_plan_transit_service(make):
    """~/plan_transit: a drive with the planner.* settings (the body, the turns) and a request's overrides on top."""
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert res.success and res.message.startswith('transit ') and res.path_length > 5.0
    last = res.path.poses[-1].pose.position
    assert (last.x, last.y) == pytest.approx((6.5, 4.0))
    rpc(node, 'planner.settings.set', {'turn_radius': 0.6})
    assert node.transit_config(node.values()).turn_radius == 0.6
    over = override('turn_on_spot', True)
    assert node.transit_config(node.values([over])).turn_on_spot
    spot = transit(node, (1.0, 1.0, 0.0), (1.0, 4.0, math.pi), overrides=[over])
    assert spot.success
    bad = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), overrides=[override('nothing', 1.0)])
    assert not bad.success and bad.message.startswith('bad override')
    out = transit(node, (30.0, 1.0, 0.0), (6.5, 4.0, 0.0))
    assert not out.success and 'the start (30.00, 1.00) lies' in out.message


@fake_only
def test_arrives(make, planned, monkeypatch):
    """With the body set the plans' sections start where ~/plan_transit gets to them (arrives): from behind, beside or
    ahead of them, nearer in a small lawn; a pose none of those gets to isn't got to. Without the body: none."""
    assert make().core.arrives(GARDEN, [], PlannerConfig(), [], []) is None
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    send_map(node, the_map())
    plan_area(node)
    assert callable(planned.calls[-1][1]['arrives'])
    cfg = node.core.current.base.with_overrides({'mower_width': 0.22, **{k: v for k, v in TRON.items()
                                                                         if k in PlannerConfig.names()}})
    gets_to = node.core.arrives(GARDEN, [BED], cfg, [], [])
    assert gets_to(Pose2D(2.0, 2.5, 0.0)) and gets_to(Pose2D(4.0, 1.0, math.pi))
    # (facing the edge right by it, the nose over it: no way gets there)
    assert not gets_to(Pose2D(7.9, 2.5, 0.0))
    # (no way from any of them: not got to, never the start it was asked about; the ways planned for a plan counted)
    asked = []

    def nowhere(start, goal, *args, **kw):
        asked.append(start)
        raise PlanningError(f'the start ({start[0]:.2f}, {start[1]:.2f}) lies where the body sticks out')
    monkeypatch.setattr(os_module, 'plan_transit', nowhere)
    gets_to = node.core.arrives(GARDEN, [BED], cfg, [], [])
    assert not gets_to(Pose2D(2.0, 2.5, 0.0)) and len(asked) == 4
    small = [(0.0, 0.0), (2.6, 0.0), (2.6, 1.6), (0.0, 1.6)]
    asked.clear()
    assert not node.core.arrives(small, [], cfg, [], [])(Pose2D(0.41, 0.11, 0.0)) and asked
    monkeypatch.setattr(os_module, 'ARRIVE_WAYS', 6)
    gets_to = node.core.arrives(GARDEN, [BED], cfg, [], [])
    gets_to(Pose2D(2.0, 2.5, 0.0))
    with pytest.raises(_Spent):
        gets_to(Pose2D(4.0, 1.0, math.pi))


@fake_only
def test_transit_as_recorded(make, monkeypatch):
    """transit_as_recorded (on by default): ~/plan_transit and arrives try a way by the outermost loop's rules where
    no other gets there; off: neither does, the plans got to so kept apart from the others (arrives' key)."""
    asked = []

    def spy(start, goal, *args, **kw):
        asked.append(kw.get('as_recorded'))
        raise PlanningError(f'no way arrives at the goal ({goal[0]:.2f}, {goal[1]:.2f})')
    monkeypatch.setattr(os_module, 'plan_transit', spy)
    body = {k: v for k, v in TRON.items() if k.startswith('robot_')}
    cfg = PlannerConfig(**{k: v for k, v in TRON.items() if k in PlannerConfig.names()})
    for on, params in ((True, {}), (True, {'transit_as_recorded': True}), (False, {'transit_as_recorded': False})):
        node = make(**dict(body, **params))
        assert not transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0)).success
        gets_to = node.core.arrives(GARDEN, [BED], cfg, [], [])
        assert not gets_to(Pose2D(2.0, 2.5, 0.0))
        assert getattr(gets_to, 'key', True) == (True if on else {'as_recorded': False})
        assert len(asked) > 1 and set(asked) == {on}
        asked.clear()


# a lawn a hedge parts: no way from one side to the other
SPLIT = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
HEDGE = [(4.0, -1.0), (6.0, -1.0), (6.0, 7.0), (4.0, 7.0)]


@fake_only
def test_arrives_from_where_the_section_before_ends(make, monkeypatch):
    """A section's start after the first: got to from where the section before ends, the way mowbite_logic asks for
    (max_outside 0.3); not where the room the centre has parts the two, nothing planned then."""
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    cfg = PlannerConfig(**{k: v for k, v in TRON.items() if k in PlannerConfig.names()})
    asked = []
    real = os_module.plan_transit

    def spy(start, goal, *args, **kw):
        asked.append((start, goal, kw.get('max_outside')))
        return real(start, goal, *args, **kw)
    monkeypatch.setattr(os_module, 'plan_transit', spy)
    gets_to = node.core.arrives(SPLIT, [HEDGE], cfg, [], [])
    end = Pose2D(1.0, 1.0, 0.0)
    assert gets_to(Pose2D(2.5, 4.0, 0.5 * math.pi), end)
    assert asked == [((1.0, 1.0, 0.0), (2.5, 4.0, 0.5 * math.pi), 0.3)]
    asked.clear()
    assert not gets_to(Pose2D(8.0, 3.0, 0.0), end) and not asked
    assert gets_to(Pose2D(8.0, 3.0, 0.0)) and asked and all(a[2] == 0.0 for a in asked)


@fake_only
@pytest.mark.real_planner
def test_no_section_across_where_the_body_doesnt_get(make):
    """A plan of a lawn a hedge parts: every section's start got to from where the one before ends (mowbite_logic's
    way), the other side's sections left out, said so."""
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    cfg = PlannerConfig(**{k: v for k, v in TRON.items() if k in PlannerConfig.names()})
    plan = node_module.plan_coverage(SPLIT, [HEDGE], cfg, arrives=node.core.arrives(SPLIT, [HEDGE], cfg, [], []))
    starts = plan.stats['starts']
    assert starts['dropped'] or starts['untried']
    ends = [i - 1 for i in plan.section_starts[1:]]
    for i, e in zip(plan.section_starts[1:], ends):
        a, b = plan.poses[e], plan.poses[i]
        got = transit(node, (a.x, a.y, a.yaw), (b.x, b.y, b.yaw), drivable=(SPLIT,), obstacles=(HEDGE,),
                      max_outside=0.3)
        assert got.success, (i, got.message)
    assert len({p.x < 5.0 for p in plan.poses}) == 1
    assert any('left out' in w for w in plan.warnings)


@fake_only
def test_back_up_with_plan(make, planned):
    """back_up_with_plan (false till M8): back_out ignored with why, planned without; true: planned (with the body set),
    planner.settings says what it forces. drive slope and reverse forwards either way (their own step)."""
    m8 = 'MowBite OS: drives forwards only for now (M8)'
    node = make()
    send_map(node, the_map(planner={'back_out': True, 'drive': 'reverse'}))
    got, _ = rpc(node, 'planner.settings')
    for name in ('drive', 'back_out'):
        assert got['settings'][name]['ignored'] == m8, name
    plan_area(node)
    assert planned.config().back_out is False and planned.config().drive == 'forward'
    body = {k: v for k, v in TRON.items() if k.startswith('robot_')}
    node = make(back_up_with_plan=True, **body)
    send_map(node, the_map(planner={'back_out': True, 'drive': 'reverse'}))
    got, _ = rpc(node, 'planner.settings')
    out = got['settings']['back_out']
    assert 'ignored' not in out and out['forces']['body_fit'] is True and got['settings']['drive']['ignored'] == m8
    plan_area(node)
    assert planned.config().back_out is True and planned.config().drive == 'forward'


@fake_only
def test_home_facing_any_way(make, monkeypatch):
    """Home from where a section ends (starts.py asks with no heading): a way there arriving facing any way."""
    asked = []
    monkeypatch.setattr(os_module, 'plan_transit', lambda start, goal, *args, **kw: asked.append((start, goal)))
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    cfg = PlannerConfig(**{k: v for k, v in TRON.items() if k in PlannerConfig.names()})
    gets_to = node.core.arrives(GARDEN, [BED], cfg, [], [])
    assert gets_to(Pose2D(2.0, 2.5, None), Pose2D(1.0, 1.0, 0.0))
    assert asked == [((1.0, 1.0, 0.0), (2.0, 2.5))]


# a lawn with an arm narrower than the Tron turns in (on the spot, or round on arcs)
ARM = [(0.0, 0.0), (5.0, 0.0), (5.0, 1.55), (8.5, 1.55), (8.5, 2.45), (5.0, 2.45), (5.0, 4.0), (0.0, 4.0)]


@fake_only
@pytest.mark.real_planner
def test_every_section_ends_where_it_gets_away(make):
    """A plan of a lawn with a narrow arm, the dock at its end: every section ends where mowbite_logic's way gets on
    from, to the next section or home (the plan's first pose, facing any way)."""
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    cfg = PlannerConfig(**{k: v for k, v in TRON.items() if k in PlannerConfig.names()})
    plan = node_module.plan_coverage(ARM, [], cfg, (1.0, 1.0, 0.0), (8.0, 2.0),
                                     arrives=node.core.arrives(ARM, [], cfg, [], []))
    assert plan.stats['starts']['asks'] >= len(plan.section_starts)
    for e, i in zip([i - 1 for i in plan.section_starts[1:]] + [len(plan.poses) - 1], plan.section_starts[1:] + [0]):
        a, b = plan.poses[e], plan.poses[i]
        got = transit(node, (a.x, a.y, a.yaw), (b.x, b.y, b.yaw), drivable=(ARM,), obstacles=(), max_outside=0.3,
                      use_goal_heading=i > 0)
        assert got.success, (e, got.message)


def tron_case():
    with open(TRON_CASE) as f:
        return json.load(f)


@fake_only
@pytest.mark.real_planner
@pytest.mark.parametrize('body', [{}, {'robot_front': 0.54, 'robot_rear': 0.178, 'blade_ahead': 0.0}])
def test_tron_case_starts_where_a_way_gets_to(make, body):
    """The Tron's case of 30.10. (its map, its body then and now): every section of the plan starts where a way gets
    to it as mowbite_logic asks ~/plan_transit for it (max_outside 0.3, the area as the drivable one): the first from
    where the mower was, each other one from where the section before it ends."""
    case = tron_case()
    node = make(**dict(TRON, **body))
    data = case['map_json']
    outline = [(p['x'], p['y']) for p in data['areas'][0]['outline']]
    send_map(node, data)
    request = PlanCoverage.Request()
    request.boundary = polygon(outline)
    res = node._on_plan_coverage(request, PlanCoverage.Response())
    assert res.success, res.message
    poses = [(ps.pose.position.x, ps.pose.position.y, 2.0 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w))
             for ps in res.path.poses]
    froms = [tuple(case['pose_at_start_mowing'])] + [poses[i - 1] for i in res.section_start[1:]]
    for start, i in zip(froms, res.section_start):
        got = transit(node, start, poses[i], drivable=(outline,), obstacles=(), max_outside=0.3)
        assert got.success, (i, got.message)
        last = got.path.poses[-1].pose if got.path.poses else None
        assert last is None or (last.position.x, last.position.y) == pytest.approx(poses[i][:2])


def test_main_is_the_os_node(monkeypatch):
    """The executable mowbite_planner_os_node runs OsNode (mowbite_planner's main), the node mowbite_planner."""
    got = []
    monkeypatch.setattr(os_module.base, 'main', lambda args=None, node_class=None: got.append(node_class))
    os_module.main()
    assert got == [OsNode]
    with open(os.path.join(HERE, '..', 'setup.py')) as f:
        assert "'mowbite_planner_os_node = mowbite_planner_os.node:main'," in f.read()


def test_with_the_real_ros(make):
    """Made and asked like on MowBite OS (with a sourced workspace: the real rclpy, mowbite_msgs and interfaces)."""
    node = make(**{k: v for k, v in TRON.items() if k.startswith('robot_')})
    assert node.get_name() == 'mowbite_planner' if REAL_ROS else node.name == 'mowbite_planner'
    assert sorted(node.core.methods) == CORE_METHODS
    res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert res.success, res.message
