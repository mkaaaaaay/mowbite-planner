"""Tests of the node on MowBite OS (OsNode): with a sourced ROS 2 workspace (colcon test) against the real rclpy,
mowbite_msgs and interfaces, otherwise against the stand-ins (osfake.py)."""

import dataclasses
import glob
import gzip
import hashlib
import json
import math
import os
import re
import threading
import time

import pytest
from shapely.geometry import Polygon as Shapely

from osfake import HERE, REAL_ROS

import rclpy  # noqa: E402
from geometry_msgs.msg import Point32, Polygon, Pose  # noqa: E402
from mowbite_msgs.msg import MapStatus, MqttPublish, RpcRequest, RpcResponse  # noqa: E402
from mowbite_msgs.srv import ReplaceMap  # noqa: E402
from mowbite_planner_interfaces.srv import PlanCoverage  # noqa: E402
from mowbite_planner_os_interfaces.srv import CutPlan, PlanTransit  # noqa: E402
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import apicheck  # noqa: E402
import calibsim  # noqa: E402
import mowbite_planner  # noqa: E402
from mowbite_planner import calibread, progress  # noqa: E402
from mowbite_planner import core as core_module  # noqa: E402
# (on MowBite OS' Jazzy too, contract.json as installed there)
from test_contract import test_in_the_package_and_in_share, test_requires_met  # noqa: E402,F401
from mowbite_planner import node as node_module  # noqa: E402
from mowbite_planner import calib as calib_module  # noqa: E402
from mowbite_planner.calib import evaluate  # noqa: E402
from mowbite_planner.config import PlannerConfig, PlanningError  # noqa: E402
from mowbite_planner.geometry import inset  # noqa: E402
from mowbite_planner.planner import plan_coverage as real_plan_coverage  # noqa: E402
from mowbite_planner.report import rules  # noqa: E402
from mowbite_planner.starts import _Spent  # noqa: E402
from mowbite_planner.mathutil import Pose2D, yaw_to_quaternion  # noqa: E402
from mowbite_planner_os import node as os_module  # noqa: E402
from mowbite_planner_os import transit_route  # noqa: E402
from mowbite_planner_os.node import OsNode  # noqa: E402
from stubplan import Spy, stub_plan_coverage  # noqa: E402

fake_only = pytest.mark.skipif(REAL_ROS, reason='looks at the stand-ins\' topics')

GARDEN = [(0.0, 0.0), (8.0, 0.0), (8.0, 5.0), (0.0, 5.0)]
BED = [(5.0, 2.0), (6.0, 2.0), (6.0, 3.0), (5.0, 3.0)]
CORE_METHODS = ['planner.angle.reset', 'planner.area.set', 'planner.drive', 'planner.leftovers', 'planner.log',
                'planner.plan', 'planner.settings', 'planner.settings.set']
# the Tron's parameters (mowbite-os, tron.yaml) and its body (robot.geometry, config/geometry/tron.yaml)
TRON = {'mower_width': 0.22, 'overlap': 0.15, 'perimeter_passes': 3, 'turn_radius': 0.44, 'lane_order': 'skip',
        'turn_on_spot': False, 'turn_types': ['u_turn', 'bulb', 'detour'], 'keep_days': 60, 'keep_mb': 50.0}
TRON_BODY = {'robot_width': 0.47, 'robot_front': 0.60, 'robot_rear': 0.12, 'robot_outline': [], 'mower_width': 0.22,
             'blade_ahead': 0.23, 'blade_offset': 0.0}
# robot.geometry's entry of bridge/params as mowbite_logic publishes it (Geometry::Params)
TRON_GEOMETRY = dict(TRON_BODY, antenna_x=0.0, antenna_y=0.0, track=0.395, wheel_radius=0.1125, dock_contact_x=-0.10,
                     ticks_per_m=1000.0, imu_roll=0.0, imu_pitch=0.0, version=0, stored=[])
# the Tron's case of 30.10. (mowbite-os 8a66cdb): the first section of a plan started where no way got to it
TRON_CASE = os.path.join(HERE, 'transit-case-8a66cdb.json')
# OsNode's last check of every plan (the stand-in's plans go without it, see planned), ~/plan_transit as written
CHECKED = OsNode.checked
TRANSIT_SRV = os.path.join(HERE, '..', '..', 'mowbite_planner_os_interfaces', 'srv', 'PlanTransit.srv')


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
    """The node plans with the stand-in (mowbite_planner's stubplan.py), its plans handed out as they are (it doesn't
    fit the body: no last check, see CHECKED); with the mark real_planner with plan_coverage itself, every plan
    checked."""
    spy = Spy()
    if 'real_planner' not in request.keywords:
        monkeypatch.setattr(node_module, 'plan_coverage', spy)
        monkeypatch.setattr(core_module, 'plan_coverage', spy)
        monkeypatch.setattr(OsNode, 'checked', lambda self, plan: plan)
    return spy


def tron_cfg(**more) -> PlannerConfig:
    """The Tron's settings as a plan has them."""
    body = dict(TRON_BODY, robot_outline=None)
    return PlannerConfig(**{k: v for k, v in dict(TRON, **body, **more).items() if k in PlannerConfig.names()})


def geometry(node, **values):
    """robot.geometry's entry of bridge/params (as mowbite_logic publishes it, latched), the Tron's with values."""
    node._on_params(String(data=json.dumps({'node': '/mowbite_geometry', 'params': dict(TRON_GEOMETRY, **values)})))


@pytest.fixture
def make(tmp_path, monkeypatch):
    """OsNodes with data_dir in tmp_path, the Tron's body from robot.geometry (body None: none yet; else values over
    the Tron's), named node_name. Every message they sent on bridge/publish kept to the planner's part of the interface
    (AM6)."""
    monkeypatch.setattr(core_module, 'PLAN_AHEAD_DELAY', 1e6)  # (planned ahead only where a test says so)
    monkeypatch.setattr(core_module, 'TRACK_WAIT', 0.0)
    if REAL_ROS:
        rclpy.init()
    made = []

    def make(body=(), node_name='mowbite_planner', **params):
        params = dict({'data_dir': str(tmp_path / 'data')}, **params)
        n = OsNode(node_name, parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        made.append(n)
        if body is not None:
            geometry(n, **dict(body))
        return n
    yield make
    for n in made:
        n.destroy_node()
    if REAL_ROS:
        rclpy.try_shutdown()
    else:
        for n in made:
            for m in n.publishers['bridge/publish'].published if 'bridge/publish' in n.publishers else ():
                assert not apicheck.sent(m.topic, m.payload), (m.topic, m.payload)


def give(node, topic, msg):
    node.subscribed[topic][1](msg)


def send_map(node, data, status=MapStatus.DATUM_MATCH):
    text = json.dumps(data)
    give(node, 'map/json', String(data=text))
    if status is not None:
        give(node, 'map/status', MapStatus(datum_state=status, mowing_allowed=True, reason=''))
    return text


def rpc(node, method, params=None, rid='r1', wait=60.0):
    """(result, None) or (None, (code, message)) of a request on bridge/rpc/request; (code, message, data) where the
    error has data."""
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
                return None, (msg.code, msg.message) + ((json.loads(msg.data),) if msg.data else ())
        time.sleep(0.01)
    raise AssertionError(f'no answer to {rid}')


def plan_area(node, boundary=GARDEN, overrides=()):
    """~/plan_coverage as mowbite_logic asks it for area a: its outline and the map's obstacles."""
    request = PlanCoverage.Request()
    request.boundary = polygon(boundary)
    request.obstacles = [polygon(BED)]
    request.overrides = list(overrides)
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


def seen_circle(x, y, r, n=12):
    return [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def test_temporary_obstacles_and_cut_plan(make):
    """interface 3: ~/plan_transit around temporary obstacles (hard lines, no margin), ~/cut_plan cutting a plan by them
    (pieces, the rest no way gets to), BAD_REQUEST past their limits, NOT_READY without the body."""
    node = make()
    seen = seen_circle(3.0, 1.2, 0.35)
    t = transit(node, (1.0, 1.2, 0.0), (6.5, 1.2, 0.0), obstacles=(), temporary_obstacles=[polygon(seen)])
    assert t.success, t.message
    body = tron_cfg().body_outline
    for p in t.path.poses:
        x, y = p.pose.position.x, p.pose.position.y
        q = p.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        corners = [(x + c * math.cos(yaw) - d * math.sin(yaw), y + c * math.sin(yaw) + d * math.cos(yaw))
                   for c, d in body]
        assert not Shapely(corners).intersects(Shapely(seen))
    many = transit(node, (1.0, 1.2, 0.0), (6.5, 1.2, 0.0), obstacles=(),
                   temporary_obstacles=[polygon(seen)] * 17)
    assert (many.success, many.code) == (False, 'BAD_REQUEST')
    lane = [(1.0 + 0.1 * i, 1.2, 0.0) for i in range(56)]
    path = node_module.to_path_msg([Pose2D(*p) for p in lane], 'map', node.get_clock().now().to_msg())
    cut = node._on_cut_plan(CutPlan.Request(start=pose(1.0, 1.2, 0.0), path=path, section_start=[0], from_pose=0,
                                            drivable_areas=[polygon(GARDEN)], obstacles=[],
                                            temporary_obstacles=[polygon(seen)]), CutPlan.Response())
    assert cut.success, cut.message
    assert list(cut.piece_first)[0] == 0 and len(cut.piece_first) == len(cut.piece_last) >= 1
    assert all(t - f >= 2 for f, t in zip(cut.piece_first, cut.piece_last))
    assert cut.left_out_from in (len(lane), *cut.piece_first) or cut.left_out_from > cut.piece_last[-1]
    bad = node._on_cut_plan(CutPlan.Request(start=pose(1.0, 1.2, 0.0), path=path, section_start=[0], from_pose=99,
                                            drivable_areas=[polygon(GARDEN)]), CutPlan.Response())
    assert (bad.success, bad.code) == (False, 'BAD_REQUEST') and 'from_pose 99' in bad.message
    wide = node._on_cut_plan(CutPlan.Request(start=pose(1.0, 1.2, 0.0), path=path, section_start=[0],
                                             drivable_areas=[polygon(GARDEN)], max_outside=5.0), CutPlan.Response())
    assert (wide.success, wide.code) == (False, 'BAD_REQUEST') and 'max_outside 5 m' in wide.message
    bare = make(body=None)
    early = bare._on_cut_plan(CutPlan.Request(start=pose(1.0, 1.2, 0.0), path=path, section_start=[0],
                                              drivable_areas=[polygon(GARDEN)]), CutPlan.Response())
    assert (early.success, early.code) == (False, 'NOT_READY')


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
    # (skip_lanes settable here: no mower_logic sets it)
    assert got['settings']['skip_lanes']['settable'] and not got['settings']['skip_outline_loops']['settable']
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
        -32602, 'no map status from mowbite_map yet (map/status)', {'code': 'NOT_READY'})
    give(node, 'map/status', MapStatus(datum_state=MapStatus.DATUM_MISMATCH, reason='another datum'))
    assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')[1] == (
        -32602, 'the map can\'t be planned now (map/status MISMATCH): another datum', {'code': 'NOT_READY'})
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
def test_planning_ahead_and_the_quick_rpcs(make, planned, monkeypatch):
    """While it plans ahead planner.settings, .settings.set and .log answer at once over the bridge; mowbite_logic's
    ~/plan_coverage stops it, it's planned after (masterplan 3.3, 04.11.)."""
    node = make()
    send_map(node, the_map())
    began, go, held = threading.Event(), threading.Event(), [1]

    def slow(*args, **kw):
        if held[0]:
            held[0] -= 1
            began.set()
            while not go.is_set():
                progress.check()
                time.sleep(0.01)
        return planned(*args, **kw)

    monkeypatch.setattr(core_module, 'plan_coverage', slow)
    ahead = threading.Thread(target=node.core.plan_ahead)
    ahead.start()
    assert began.wait(30)
    t0 = time.monotonic()
    assert rpc(node, 'planner.settings', None, 's1', wait=5.0)[1] is None
    assert rpc(node, 'planner.settings.set', {'turn_radius': node.core.current.base.turn_radius}, 's2', 5.0)[1] is None
    lines, error = rpc(node, 'planner.log', {'limit': 50}, 's3', wait=5.0)
    assert error is None and time.monotonic() - t0 < 5.0 and ahead.is_alive()
    assert plan_area(node).success
    ahead.join(60)
    lines, _ = rpc(node, 'planner.log', {'limit': 50}, 's4')
    said = [line['msg'] for line in lines['lines']]
    assert any('planning a ahead stopped (mower_logic waits), it waits again' in m for m in said)
    assert any('1 areas planned ahead' in m for m in said)


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
    with open(os.path.join(HERE, '..', '..', '..', 'ros2', 'mowbite_planner', 'mowbite_planner', 'contract.json')) as f:
        assert json.load(f)['methods']['mowbite-os'] == sorted(CORE_METHODS + ['planner.calib.evaluate'])
    mower = calibsim.program(0)
    path = mower.recording(str(root / 'recordings' / 'calib-20261104T010203Z.jsonl.gz'))
    got, error = rpc(node, 'planner.calib.evaluate', {'path': path}, 'r1', wait=120.0)
    assert error is None and got == json.loads(json.dumps(evaluate(path)))
    assert got['version'] == 'mowbite-calib/1' and {'ready', 'values', 'warnings', 'based_on', 'info'} <= set(got)
    with open(os.path.join(HERE, '..', 'examples', 'calib-evaluate.json')) as f:  # (the example as it comes)
        example = json.load(f)
    assert set(example['response']['result']) == set(got)
    assert [v['name'] for v in example['response']['result']['values']] == [v['name'] for v in got['values']]
    # (several together, calib.MAX_RECORDINGS at most, not with path)
    second = calibsim.program(1).recording(str(root / 'recordings' / 'calib-20261104T020304Z.jsonl.gz'))
    got, error = rpc(node, 'planner.calib.evaluate', {'paths': [path, second]}, 'r1b', wait=240.0)
    assert error is None and got == json.loads(json.dumps(evaluate([path, second])))
    assert os_module.MAX_RECORDINGS == calib_module.MAX_RECORDINGS
    for rid, params in (('r1c', {'paths': [path] * 5}), ('r1d', {'paths': []}), ('r1e', {'paths': path}),
                        ('r1f', {'paths': [path], 'path': path}), ('r1g', {'paths': [path, 'recordings/x']}),
                        ('r1h', {'paths': [path, str(tmp_path / 'elsewhere.jsonl.gz')]})):
        assert rpc(node, 'planner.calib.evaluate', params, rid)[1][0] == -32602, params
    assert rpc(node, 'planner.calib.evaluate', {'paths': [path, str(root / 'none.jsonl.gz')]}, 'r1i')[1] == (
        -32603, 'no recording at %s' % (root / 'none.jsonl.gz'))
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
    unended = mower.recording(str(root / 'recordings' / 'unended.jsonl.gz'), end=None)  # (its writer gone)
    assert rpc(node, 'planner.calib.evaluate', {'path': unended}, 'r8e', wait=120.0)[1] == (
        -32603, 'the calibration not evaluated: ' + calibread.NO_END)
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
    give(node, 'bridge/rpc/request', RpcRequest(method='planner.calib.evaluate',
                                                params=json.dumps({'paths': [path, second]}), id='r11'))
    assert answer(node, 'r11')[1][0] == -32603
    assert asked == [120.0, 240.0]


@fake_only
def test_plan_transit_service(make):
    """~/plan_transit: a drive with the planner.* settings (the body, the turns) and a request's overrides on top."""
    node = make()
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
def test_plan_transit_code(make, monkeypatch):
    """~/plan_transit's code: empty with a way, else why not as a word (transit_route.CODES, the ones the .srv tells
    of), each from where it arises; NO_WAY for a no without a word, INTERNAL for any other error."""
    with open(TRANSIT_SRV) as f:
        assert re.findall(r'^#   ([A-Z_]+) ', f.read(), re.M) == list(transit_route.CODES)
    assert transit(make(None), (1.0, 1.0, 0.0), (6.5, 4.0, 0.0)).code == 'NOT_READY'
    node = make()
    res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert res.success and res.code == ''
    assert transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2), max_outside=os_module.MAX_OUTSIDE_ASKED).success
    far = [(20.0, 0.0), (28.0, 0.0), (28.0, 5.0), (20.0, 5.0)]
    for code, start, goal, fields in (
            ('BAD_REQUEST', (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), {'overrides': [override('nothing', 1.0)]}),
            # (one that doesn't go with the settings)
            ('BAD_REQUEST', (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), {'overrides': [override('angle_min_deg', 10.0)]}),
            ('BAD_REQUEST', (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), {'goal_straight_length': -1.0}),
            ('BAD_REQUEST', (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), {'drivable_areas': []}),
            ('BAD_REQUEST', (math.nan, 1.0, 0.0), (6.5, 4.0, 0.0), {}),
            ('BAD_REQUEST', (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), {'max_outside': os_module.MAX_OUTSIDE_ASKED + 0.01}),
            ('BAD_REQUEST', (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), {'max_outside': math.inf}),
            ('START_OUTSIDE', (30.0, 1.0, 0.0), (6.5, 4.0, 0.0), {}),
            ('START_OUTSIDE', (7.9, 2.5, 0.0), (2.0, 2.0, 0.0), {'max_outside': 0.0}),  # (the nose over the edge)
            ('GOAL_OUTSIDE', (1.0, 1.0, 0.0), (5.5, 2.5, 0.0), {}),  # (in the bed)
            ('NO_WAY', (1.0, 1.0, 0.0), (24.0, 2.5, 0.0), {'drivable_areas': [polygon(GARDEN), polygon(far)]}),
            ('NO_WAY', (1.0, 1.0, 0.0), (7.9, 2.5, 0.5 * math.pi), {'max_outside': 0.0})):
        res = transit(node, start, goal, **fields)
        assert not res.success and res.code == code and res.message and not res.path.poses, (code, res.message)
    for error, code in ((RuntimeError('broken'), 'INTERNAL'), (PlanningError('no'), 'NO_WAY'),
                        (PlanningError('no', word='BODY_OUT'), 'NO_WAY')):
        def failing(*args, error=error, **kw):
            raise error
        monkeypatch.setattr(os_module, 'plan_transit', failing)
        res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0))
        assert not res.success and res.code == code and res.code in transit_route.CODES


@fake_only
def test_transit_search_limit(make, monkeypatch):
    """transit_search_limit (6.0 s, 0: none): ~/plan_transit's search ends once that long has passed since it took the
    request, its code SEARCH_LIMIT; the next request answered as if it hadn't (what it kept of the map still good).
    The ways of a ~/plan_coverage's arrives have no limit."""
    assert make().search_limit() == os_module.SEARCH_LIMIT == 6.0
    assert make(transit_search_limit=0.0).search_limit() == 0.0
    for bad in (-1.0, math.inf, math.nan):
        with pytest.raises(ValueError, match='transit_search_limit must be a number of s >= 0'):
            make(transit_search_limit=bad)
    ask = ((1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))

    def way(res):
        return [(ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.z, ps.pose.orientation.w)
                for ps in res.path.poses]
    want = transit(make(), *ask)
    assert want.success
    for role in ({}, {'role': 'transit', 'node_name': 'mowbite_planner_transit'}):
        node = make(transit_search_limit=1e-9, **role)
        res = transit(node, *ask)
        assert not res.success and res.code == 'SEARCH_LIMIT' and not res.path.poses
        assert res.message == 'planning the transit failed: no way found in 1e-09 s (transit_search_limit)'
        # (looked at on the way in too, before a start too far out is told of)
        assert transit(node, (30.0, 1.0, 0.0), ask[1]).code == 'SEARCH_LIMIT'
        node._search_limit = 0.0
        assert transit(node, (30.0, 1.0, 0.0), ask[1]).code == 'START_OUTSIDE'
        got = transit(node, *ask)
        assert got.success and way(got) == way(want)
    # (its clock from when it took the request, looked at in the search: a search that takes long ends)
    node = make(transit_search_limit=0.3)
    real = os_module.plan_transit
    checks = []

    def slow(*args, **kw):
        for _ in range(200):
            time.sleep(0.01)
            progress.check()
            checks.append(1)
        return real(*args, **kw)
    monkeypatch.setattr(os_module, 'plan_transit', slow)
    t0 = time.monotonic()
    res = transit(node, *ask)
    assert res.code == 'SEARCH_LIMIT' and time.monotonic() - t0 >= 0.3 and len(checks) < 100
    assert 'no way found in 0.3 s' in res.message
    # (arrives: no limit)
    monkeypatch.setattr(os_module, 'plan_transit', real)
    node = make(transit_search_limit=1e-9)
    assert node.core.arrives(GARDEN, [BED], tron_cfg(), [], [])(Pose2D(2.0, 2.5, 0.0))


@fake_only
def test_arrives(make, planned, monkeypatch):
    """With the body set the plans' sections start where ~/plan_transit gets to them (arrives): from behind, beside or
    ahead of them, nearer in a small lawn; a pose none of those gets to isn't got to. Without the body: none."""
    assert make().core.arrives(GARDEN, [], PlannerConfig(), [], []) is None
    node = make()
    send_map(node, the_map())
    plan_area(node)
    assert callable(planned.calls[-1][1]['arrives'])
    cfg = node.core.current.base.with_overrides({k: getattr(tron_cfg(), k) for k in TRON if k in PlannerConfig.names()})
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
def test_recorded_bodies(make, planned, monkeypatch):
    """The ways (~/plan_transit, arrives) go by the body each ring of the map was recorded with (by its outline), as
    the plans do; planner.plan's body_hints tells of the areas recorded without one too (missing)."""
    node = make()
    small = {'robot_width': 0.4, 'robot_front': 0.5, 'robot_rear': 0.1, 'robot_outline': [], 'by': 'os-1'}
    corners = PlannerConfig(robot_width=0.4, robot_front=0.5, robot_rear=0.1).body_outline
    send_map(node, the_map(recorded_body=small))
    asked = []
    real = os_module.plan_transit
    monkeypatch.setattr(os_module, 'plan_transit', lambda *args, **kw: asked.append((args, kw)) or real(*args, **kw))
    assert transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2)).success
    bodies = asked[-1][1]['bodies']
    assert [c for _, c in bodies] == [corners] and bodies[0][0][:4] == GARDEN
    gets_to = node.core.arrives(GARDEN, [BED], tron_cfg(), [], [], (), (), (), bodies)
    assert gets_to(Pose2D(2.0, 2.5, 0.0)) and asked[-1][1]['bodies'] == bodies
    result, _ = rpc(node, 'planner.plan', {'area_id': 'a'})
    assert result['body_hints'] == {'a': 'larger', 'b': 'missing'} and planned.calls[-1][1]['bodies']
    send_map(node, the_map())
    result, _ = rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')
    assert result['body_hints'] == {'a': 'missing', 'b': 'missing'} and 'bodies' not in planned.calls[-1][1]


@fake_only
def test_arrives_as_mowbite_logic_asks(make, monkeypatch):
    """Every way, the first section's too, as mowbite_logic asks ~/plan_transit (masterplan 3.4): through every active
    mowing and navigation area (drivable, reach_over, unmowed), ARRIVE_OUTSIDE at the ends; where the goal lies
    outside, its straight stretch is checked with the body in the plan's body space (the nose over the edge there: not
    got to)."""
    from mowbite_planner_os.transit_route import Transit
    node, cfg = make(), tron_cfg()
    nav, other = [(8.0, 0.0), (10.0, 0.0), (10.0, 5.0), (8.0, 5.0)], [(0.0, 5.0), (8.0, 5.0), (8.0, 7.0), (0.0, 7.0)]
    asked = []
    end = []  # (the goal's stretch outside: these poses, the last the goal's)

    def spy(start, goal, areas, away, cfg, max_outside, margins):
        asked.append((len(areas), max_outside))
        assert margins == [cfg.obstacle_margin] * len(away)
        poses = end or [Pose2D(start[0], start[1], start[2]), Pose2D(goal[0], goal[1], goal[2])]
        return Transit(poses, 1.0, [], 0.0, 0.2 if end else 0.0, [])
    monkeypatch.setattr(os_module, 'plan_transit', spy)
    gets_to = node.core.arrives(GARDEN, [BED], cfg, [], [], [nav], [other])
    assert gets_to(Pose2D(2.0, 2.5, 0.0)) and gets_to(Pose2D(4.0, 1.0, 0.0), Pose2D(2.0, 2.5, 0.0))
    assert set(asked) == {(3, os_module.ARRIVE_OUTSIDE)}
    # (straight on to a goal by the eastern edge: the body fits where the navigation area lies beyond it, else not)
    end.extend(Pose2D(7.7 + 0.05 * k, 2.5, 0.0) for k in range(5))
    assert gets_to(Pose2D(7.9, 2.5, 0.0))
    assert not node.core.arrives(GARDEN, [BED], cfg, [], [], [], [other])(Pose2D(7.9, 2.5, 0.0))


@fake_only
def test_transit_by_its_margins(make, monkeypatch):
    """~/plan_transit and arrives plan every way with the margins as they are, no second try (mkay 10.11.); the plans
    got to so kept by the code alone (arrives' key)."""
    asked = []

    def spy(start, goal, *args, **kw):
        asked.append(kw)
        raise PlanningError(f'no way arrives at the goal ({goal[0]:.2f}, {goal[1]:.2f})')
    monkeypatch.setattr(os_module, 'plan_transit', spy)
    node = make()
    assert not transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0)).success
    gets_to = node.core.arrives(GARDEN, [BED], tron_cfg(), [], [])
    assert not gets_to(Pose2D(2.0, 2.5, 0.0))
    assert gets_to.key == {'os': os_module.os_code()}
    assert len(asked) > 1 and all('as_recorded' not in kw for kw in asked)


# a lawn a hedge parts: no way from one side to the other
SPLIT = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
HEDGE = [(4.0, -1.0), (6.0, -1.0), (6.0, 7.0), (4.0, 7.0)]


@fake_only
def test_arrives_from_where_the_section_before_ends(make, monkeypatch):
    """A section's start after the first: got to from where the section before ends, the way mowbite_logic asks for
    (max_outside 0.3, the first section's too); not where the room the centre has parts the two, nothing planned
    then."""
    node = make()
    cfg = tron_cfg()
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
    assert gets_to(Pose2D(8.0, 3.0, 0.0)) and asked and all(a[2] == os_module.ARRIVE_OUTSIDE for a in asked)


@fake_only
@pytest.mark.real_planner
def test_no_section_across_where_the_body_doesnt_get(make):
    """A plan of a lawn a hedge parts: every section's start got to from where the one before ends (mowbite_logic's
    way), the other side's sections left out, said so."""
    node = make()
    cfg = tron_cfg()
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
    node = make(back_up_with_plan=True)
    send_map(node, the_map(planner={'back_out': True, 'drive': 'reverse'}))
    got, _ = rpc(node, 'planner.settings')
    out = got['settings']['back_out']
    assert 'ignored' not in out and out['forces']['body_fit'] is True and got['settings']['drive']['ignored'] == m8
    plan_area(node)
    assert planned.config().back_out is True and planned.config().drive == 'forward'


@fake_only
def test_recorded_loop_only_where_the_map_has_the_recording_body(make, planned):
    """recorded_loop on MowBite OS: planned only on an area the map has the body it was recorded with of
    (recorded_body: recorded by driving along it); one without (drawn) is planned without it, the answer says so."""
    why = 'recorded_loop ignored: MowBite OS: only on an area recorded by driving along it (recorded_body in the ' \
        'map), this one has none'
    node = make()
    send_map(node, the_map(planner={'recorded_loop': True, 'recorded_loop_margin': 0.02}))
    got = plan_area(node)
    assert got.success and why in got.message and planned.config().recorded_loop is False
    result, _ = rpc(node, 'planner.plan', {'area_id': 'a'})
    assert why in result['warnings'] and result['body_hints']['a'] == 'missing'
    assert 'ignored' not in rpc(node, 'planner.settings', None, 'r2')[0]['settings']['recorded_loop']
    own = {k: TRON_BODY[k] for k in ('robot_width', 'robot_front', 'robot_rear', 'robot_outline')}
    send_map(node, the_map(planner={'recorded_loop': True, 'recorded_loop_margin': 0.02}, recorded_body=own))
    got = plan_area(node)
    cfg = planned.config()
    assert got.success and 'recorded_loop' not in got.message
    assert cfg.recorded_loop is True and cfg.recorded_loop_margin == 0.02


@fake_only
@pytest.mark.real_planner
def test_a_plan_with_the_recorded_loop_passes_the_last_check(make):
    """The node's last check of a plan (checked) takes the recorded loop in its own space: the plan is given out,
    though on that loop the body is past the margins all the others keep."""
    node = make()
    own = {k: TRON_BODY[k] for k in ('robot_width', 'robot_front', 'robot_rear', 'robot_outline')}
    send_map(node, the_map(planner={'recorded_loop': True}, recorded_body=own))
    got = plan_area(node)
    assert got.success and got.code == '' and 'recorded_loop' not in got.message, got.message
    plan = next(reversed(node.core.plans.values()))[0]
    assert plan.recorded_loops and rules(plan)['body_out'] == []
    assert rules(dataclasses.replace(plan, recorded_loops=()))['body_out']


@fake_only
def test_home_facing_any_way(make, monkeypatch):
    """Home from where a section ends (starts.py asks with no heading): a way there arriving facing any way."""
    asked = []
    from mowbite_planner_os.transit_route import Transit
    monkeypatch.setattr(os_module, 'plan_transit', lambda start, goal, *args, **kw: asked.append((start, goal)) or
                        Transit([], 0.0, [], 0.0, 0.0, []))
    node = make()
    cfg = tron_cfg()
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
    node = make()
    cfg = tron_cfg()
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
@pytest.mark.parametrize('body', [{}, {'robot_front': 0.54, 'robot_rear': 0.178, 'blade_ahead': 0.0, 'version': 1}])
def test_tron_case_starts_where_a_way_gets_to(make, body):
    """The Tron's case of 30.10. (its map, 2.6 x 1.6 m, its body then and now): every section of the plan after the
    first starts where a way gets to it as mowbite_logic asks ~/plan_transit for it (max_outside 0.3, the area as the
    drivable one), from where the section before it ends. With the request's start where the mower was then (the
    middle) the first section is got to from there too (the outermost loop keeping the margins like the others, mkay
    10.11.; refused with START_UNREACHABLE before)."""
    case = tron_case()
    node = make(body, **TRON)
    data = case['map_json']
    outline = [(p['x'], p['y']) for p in data['areas'][0]['outline']]
    send_map(node, data)
    request = PlanCoverage.Request()
    request.boundary = polygon(outline)
    res = node._on_plan_coverage(request, PlanCoverage.Response())
    assert res.success, res.message
    poses = [(ps.pose.position.x, ps.pose.position.y, 2.0 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w))
             for ps in res.path.poses]
    for i in res.section_start[1:]:
        got = transit(node, poses[i - 1], poses[i], drivable=(outline,), obstacles=(), max_outside=0.3)
        assert got.success, (i, got.message)
    request.use_start_pose, request.start_pose = True, pose(*case['pose_at_start_mowing'])
    res = node._on_plan_coverage(request, PlanCoverage.Response())
    assert res.success, res.message
    first = res.path.poses[0].pose
    got = transit(node, case['pose_at_start_mowing'], (first.position.x, first.position.y,
                                                       2.0 * math.atan2(first.orientation.z, first.orientation.w)),
                  drivable=(outline,), obstacles=(), max_outside=0.3)
    assert got.success, got.message


@fake_only
@pytest.mark.real_planner
def test_the_first_section_from_the_start(make, monkeypatch):
    """With the request's start (use_start_pose, where mowbite_logic undocks to) the first section starts where
    mowbite_logic's way from there gets to it (max_outside 0.3), the others from where the section before ends."""
    node = make(**TRON)
    send_map(node, the_map())
    start = (7.0, 0.6, math.pi)
    request = PlanCoverage.Request()
    request.boundary, request.obstacles = polygon(GARDEN), [polygon(BED)]
    request.use_start_pose, request.start_pose = True, pose(*start)
    res = node._on_plan_coverage(request, PlanCoverage.Response())
    assert res.success, res.message
    poses = [(ps.pose.position.x, ps.pose.position.y, 2.0 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w))
             for ps in res.path.poses]
    for frm, i in zip([start] + [poses[i - 1] for i in res.section_start[1:]], res.section_start):
        got = transit(node, frm, poses[i], max_outside=0.3)
        assert got.success, (i, got.message)
    # (no way from the start to any section: refused, the code START_UNREACHABLE, kept with the no)

    def nowhere(start, goal, *args, **kw):
        raise PlanningError(f'no way arrives at the goal ({goal[0]:.2f}, {goal[1]:.2f})')
    monkeypatch.setattr(os_module, 'plan_transit', nowhere)
    request.start_pose = pose(6.0, 0.6, math.pi)
    for again in range(2):
        res = node._on_plan_coverage(request, PlanCoverage.Response())
        assert not res.success and res.code == 'START_UNREACHABLE' and not res.path.poses
        assert res.message.startswith('area a: start unreachable: no way from the start (6.00, 0.60) gets to any')


@fake_only
def test_plan_coverage_code(make, planned):
    """~/plan_coverage's code on MowBite OS: NOT_READY without the body, without the map or while it can't be planned,
    the others as mowbite_planner's node gives them."""
    bare = make(None)
    send_map(bare, the_map())
    res = plan_area(bare)
    assert not res.success and res.code == 'NOT_READY' and 'no dimensions of the mower yet' in res.message
    bare.core.map = None
    res = plan_area(bare)
    assert not res.success and res.code == 'NOT_READY' and 'no map yet' in res.message
    node = make()
    send_map(node, the_map(), MapStatus.DATUM_MISMATCH)
    res = plan_area(node)
    assert not res.success and res.code == 'NOT_READY' and 'map/status MISMATCH' in res.message
    give(node, 'map/status', MapStatus(datum_state=MapStatus.DATUM_MATCH))
    res = plan_area(node)
    assert res.success and res.code == '' and node.core.pinned  # (the job's plan, kept till it ends)
    assert rpc(node, 'planner.settings')[0]['job_open'] is True
    request = PlanCoverage.Request(boundary=polygon(GARDEN), overrides=[override('robot_width', 0.3)])
    assert node._on_plan_coverage(request, PlanCoverage.Response()).code == 'BAD_REQUEST'
    # (an outline of no area of the map: not planned without the map's areas to keep out of, margins and dock)
    for again in range(2):
        res = plan_area(node, [(0.0, 0.0), (1.0, 1.0)])
        assert not res.success and res.code == 'BAD_REQUEST' and 'no area of the map' in res.message, res.message


@fake_only
def test_preview(make, planned):
    """The override preview (mowbite_logic's mowing.plan): an outline of no area planned like planner.plan plans it,
    an area's plan as ever; neither kept for the job (pin) nor timed as a mow (served), armed leftovers left out.
    Without it the outline refused as before."""
    node = make()
    send_map(node, the_map())
    outline, preview = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)], [override('preview', True)]
    assert plan_area(node, outline).code == 'BAD_REQUEST'
    got = plan_area(node, outline, preview)
    assert got.success and got.message.startswith('outline, planned: ') and 'overrides left out' not in got.message
    assert not node.core.served and not node.core.pinned
    area = plan_area(node, overrides=preview)
    assert area.success and not node.core.served and not node.core.pinned
    assert rpc(node, 'planner.settings')[0]['job_open'] is False
    node.core.armed_plan = lambda job: (True, None)  # (armed leftovers, none of them in this area)
    assert plan_area(node, overrides=preview).fingerprint == area.fingerprint
    assert plan_area(node).message.endswith('nothing to mow')
    del node.core.armed_plan
    job = plan_area(node)
    assert job.fingerprint == area.fingerprint and node.core.pinned and len(node.core.served) == 1
    # (never driven: planned whatever the map's state, an empty one as the first area is drawn too, or none yet)
    for data, status in ((the_map(), MapStatus.DATUM_MISMATCH), ({'areas': []}, MapStatus.DATUM_EMPTY)):
        send_map(node, data, status)
        assert plan_area(node).code == ('NOT_READY' if data['areas'] else 'BAD_REQUEST')
        assert plan_area(node, outline, preview).success and plan_area(node, overrides=preview).success
    fresh = make()
    assert plan_area(fresh, outline).code == 'NOT_READY' and plan_area(fresh, outline, preview).success


@fake_only
def test_every_plan_checked_once_more(make, planned, monkeypatch, tmp_path):
    """Every plan checked once more before it's kept or given out (the body's check of the tests, report.rules'):
    one the body sticks out on (the stand-in's, it doesn't fit the body) is refused with the code BODY_OUT, telling
    where; nothing published, not kept as a plan, the no kept (planner.plan the same, the word as its error's data,
    nothing planned again); the planner's own plans pass as they are. mowbite_planner's own nodes don't check."""
    monkeypatch.setattr(OsNode, 'checked', CHECKED)
    node = make()
    send_map(node, the_map())
    res = plan_area(node)
    args, kw = planned.calls[-1]
    plan = stub_plan_coverage(*args, **{k: v for k, v in kw.items() if k != 'progress'})
    out = rules(plan)['body_out']
    assert out and not res.success and res.code == 'BODY_OUT' and not res.path.poses and res.fingerprint == ''
    places = []  # (a pose of every place: BODY_OUT_APART m apart)
    for o in out:
        if all(math.hypot(o['x'] - p['x'], o['y'] - p['y']) >= os_module.BODY_OUT_APART for p in places):
            places.append(o)
    assert len(out) > len(places) > os_module.BODY_OUT_SHOWN and {o['kind'] for o in out} <= set(
        os_module.BODY_OUT_KINDS)
    no = f'the plan is not given out, the body would stick out past the edges at {len(out)} poses: at ' + ', '.join(
        f'({o["x"]:.2f}, {o["y"]:.2f}) {os_module.BODY_OUT_KINDS[o["kind"]]}'
        for o in places[:os_module.BODY_OUT_SHOWN]) + f' and {len(places) - os_module.BODY_OUT_SHOWN} more places'
    assert res.message == 'area a: ' + no
    assert ('error', 'MowBite Planner: ' + no) in node.get_logger().lines
    assert rpc(node, 'planner.plan', {'area_id': 'a'})[1] == (-32602, no, {'code': 'BODY_OUT'})
    assert plan_area(node).code == 'BODY_OUT' and len(planned.calls) == 1
    assert not node.core.plans and len(node.core.refused) == 1
    assert not glob.glob(str(tmp_path / 'data' / 'planner' / 'plans' / '*.pkl'))
    assert not node.publishers['~/coverage_path'].published
    shown = [json.loads(m.payload) for m in node.publishers['bridge/publish'].published
             if m.topic == 'mowbite/planner/progress']
    assert shown[-1]['error'] == no and not shown[-1]['running']
    # (use_map off, and what a mow left)
    res = plan_area(node, overrides=[override('use_map', False)])
    assert not res.success and res.code == 'BODY_OUT' and res.message.startswith('the plan is not given out, ')
    assert not node.publishers['~/coverage_path'].published
    with pytest.raises(PlanningError, match='^the plan is not given out') as e:
        node.core.left_plan(args[0], args[1], args[2], None, None, [], [], mowed=inset(plan.free_space, 0.5))
    assert e.value.word == 'BODY_OUT'
    # (the planner's own plan: as it is; made to stick out: refused; no check without MowBite OS)
    cfg = tron_cfg()
    good = real_plan_coverage(GARDEN, [BED], cfg, arrives=node.core.arrives(GARDEN, [BED], cfg, [], []))
    assert rules(good)['body_out'] == [] and node.checked(good) is good
    bad = dataclasses.replace(good, body_space=inset(good.body_space, 0.2))
    with pytest.raises(PlanningError, match=r'would stick out past the edges at %d poses: at \(' % len(
            rules(bad)['body_out'])) as e:
        node.checked(bad)
    assert e.value.word == 'BODY_OUT' and len(rules(bad)['body_out']) > 0
    assert node_module.MowbitePlannerNode.checked(node, bad) is bad
    assert core_module.PlannerCore.checked(node.core, bad) is bad


@fake_only
def test_body_from_robot_geometry(make, planned, tmp_path):
    """The body and the blade from robot.geometry (/mowbite_geometry on bridge/params): not settable, the node's
    parameters, the stored ones, an area's and a request's own don't count; none yet or none readable: nothing planned,
    said why; a new version: planned with it, the plans with the values before not taken."""
    os.makedirs(tmp_path / 'data' / 'planner')
    with open(tmp_path / 'data' / 'planner' / 'mowbite_planner_settings.json', 'w') as f:
        json.dump({'robot_width': 0.9, 'blade_ahead': 0.4, 'turn_radius': 0.3}, f)
    node = make(None, robot_width=0.6, robot_front=0.7, robot_rear=0.2, blade_ahead=0.1)
    assert 'parameters not counting here (from robot.geometry (/mowbite_geometry)): robot_width, robot_front, ' \
        'robot_rear, blade_ahead' in [m for _, m in node.get_logger().lines]
    send_map(node, the_map())
    none = 'no dimensions of the mower yet (robot.geometry (/mowbite_geometry) on bridge/params)'
    assert rpc(node, 'planner.plan', {'area_id': 'a'})[1] == (-32602, none, {'code': 'NOT_READY'})
    assert plan_area(node).message == 'area a: ' + none
    assert transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0)).message == 'planning the transit failed: ' + none
    plain = PlanCoverage.Request(boundary=polygon(GARDEN))
    assert node.plan(GARDEN, [], node.values(), {})[2:] == (none, 'NOT_READY')
    assert not planned.calls and node.core.current.base.robot_width is None and plain
    got, _ = rpc(node, 'planner.settings', None, 'r2')
    for name in os_module.GEOMETRY_KEYS:
        assert got['settings'][name]['settable'] is False, name
        assert got['settings'][name]['from'] == 'robot.geometry (/mowbite_geometry)', name
    assert rpc(node, 'planner.settings.set', {'robot_width': 0.5, 'turn_radius': 0.4}, 'r3')[1] == (
        -32602, 'not settable here: robot_width (from robot.geometry (/mowbite_geometry))')
    geometry(node)
    current = node.core.current
    assert (current.base.robot_width, current.base.robot_front, current.base.robot_rear, current.base.robot_outline,
            current.mower_width, current.base.blade_ahead, current.base.turn_radius) == (
        0.47, 0.6, 0.12, None, 0.22, 0.23, 0.3)
    got, _ = rpc(node, 'planner.settings', None, 'r4')
    assert got['settings']['robot_width']['value'] == 0.47 and got['settings']['robot_outline']['value'] is None
    with open(tmp_path / 'data' / 'planner' / 'mowbite_planner_settings.json') as f:
        assert json.load(f) == {'robot_width': 0.9, 'blade_ahead': 0.4, 'turn_radius': 0.3}  # (as it was)
    send_map(node, the_map(planner={'robot_width': 0.3, 'robot_front': 0.3, 'robot_rear': 0.1, 'mower_width': 0.4}))
    first = plan_area(node)
    assert first.success and (planned.config().robot_width, planned.config().mower_width) == (0.47, 0.22)
    assert got['settings']['body_fit']['settable'] is False and got['settings']['robot_front_width']['ignored']
    send_map(node, the_map(planner={'body_fit': False, 'robot_front_width': 0.1}))
    assert plan_area(node).success and planned.config().body_fit is True
    assert planned.config().robot_front_width is None
    assert rpc(node, 'planner.area.set', {'area_id': 'a', 'settings': {'blade_ahead': 0.1}}, 'r5')[1] == (
        -32602, 'not settings an area can have: blade_ahead')
    bad = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0), overrides=[override('robot_width', 0.3)])
    assert bad.message == 'bad override: robot_width from robot.geometry (/mowbite_geometry) here'
    assert transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2)).success
    assert node.transit_config(node.values()).robot_width == 0.47
    # (latched again: nothing new; a new version: planned with it, the plan before not taken)
    geometry(node)
    assert plan_area(node).message.startswith('area a, kept from before: ')
    geometry(node, version=1, robot_outline=[[0.6, 0.2], [-0.12, 0.235], [-0.12, -0.235], [0.6, -0.2]])
    assert node.core.current.base.robot_outline == ((0.6, 0.2), (-0.12, 0.235), (-0.12, -0.235), (0.6, -0.2))
    again = plan_area(node)
    assert again.message.startswith('area a, planned: ') and planned.config().robot_outline is not None
    assert len(node.core.plans) == 2
    # (one the settings don't go with, or not readable: nothing planned, said why)
    geometry(node, version=2, blade_ahead=1.2)
    assert node.core.current.base.robot_outline is not None
    res = plan_area(node)
    assert not res.success and "the mower's dimensions (version 2) don't go with the planner's settings" in res.message
    node._on_params(String(data=json.dumps({'node': '/mowbite_geometry', 'params': {'version': 3}})))
    assert "the mower's dimensions not readable" in plan_area(node).message
    assert "the mower's dimensions not readable" in transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, 0.0)).message
    geometry(node, version=4)
    assert plan_area(node).success
    # (the settings go with the new ones once they're changed; the moment they're taken nothing is planned)
    assert rpc(node, 'planner.settings.set', {'lane_spacing_max': 0.2}, 'r6')[1] is None
    geometry(node, version=5, mower_width=0.18)
    assert "don't go with the planner's settings: lane_spacing_max" in plan_area(node).message
    assert rpc(node, 'planner.settings.set', {'lane_spacing_max': None}, 'r7')[1] is None
    assert plan_area(node).success and planned.config().mower_width == 0.18
    node.geometry, node.geometry_why = (6, dict(node.geometry[1], robot_width=0.5)), ''  # (as _on_geometry, before)
    assert plan_area(node).message == "area a: the mower's dimensions (version 6) not taken yet"
    # (use_map off: in the worker, with the body; without a map NOT_READY)
    node.core.map = None
    assert plan_area(node).code == 'NOT_READY'
    threads = []
    real = node_module.plan_coverage

    def where(*args, **kw):
        threads.append(threading.current_thread().name)
        return real(*args, **kw)
    node_module.plan_coverage = where
    try:
        geometry(node, version=6, robot_width=0.5)
        assert plan_area(node, overrides=[override('use_map', False)]).success and threads == ['planner worker'] and \
            planned.config().robot_width == 0.5
    finally:
        node_module.plan_coverage = real


@fake_only
def test_stored_before_the_geometry(make, tmp_path):
    """Settings stored that go with the mower's own blade but not yet with the node's mower_width (robot.geometry
    comes after the start): kept, taken once it's there; a change before that takes them as well."""
    stored = {'lane_spacing_max': 0.25, 'perimeter_passes': 4, 'angle': 30.0}
    path = tmp_path / 'data' / 'planner' / 'mowbite_planner_settings.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(stored))
    node = make(body=None)
    # (only the one that doesn't go with it left out till then)
    assert node.core.current.stored == {'perimeter_passes': 4, 'angle': 30.0}
    assert node.core.left_out == {'lane_spacing_max': 0.25}
    geometry(node, mower_width=0.281)
    assert node.core.current.stored == stored and node.core.current.base.perimeter_passes == 4
    assert json.loads(path.read_text()) == stored
    path.write_text(json.dumps(stored))
    later = make(body=None)
    refused = rpc(later, 'planner.settings.set', {'overlap': 0.1}, 'r2')[1]  # (0.25 is more than 0.22 still)
    assert refused[1].startswith('the settings stored before (lane_spacing_max) don\'t go with the mower: ')
    assert json.loads(path.read_text()) == stored
    geometry(later, mower_width=0.281)
    assert rpc(later, 'planner.settings.set', {'overlap': 0.1}, 'r3')[1] is None
    assert json.loads(path.read_text()) == dict(stored, overlap=0.1)


@fake_only
def test_margins_as_set(make, planned, tmp_path):
    """edge_margin, obstacle_margin and spin_margin as set, no floors (mkay 10.11.): stored, an area's, an obstacle's
    and a transit's own taken as they are."""
    os.makedirs(tmp_path / 'data' / 'planner')
    with open(tmp_path / 'data' / 'planner' / 'mowbite_planner_settings.json', 'w') as f:
        json.dump({'spin_margin': 0.0, 'edge_margin': 0.08}, f)
    node = make(edge_margin=0.01)
    base = node.core.current.base
    assert (base.edge_margin, base.obstacle_margin, base.spin_margin) == (0.08, 0.1, 0.0)
    got, _ = rpc(node, 'planner.settings')
    assert all(got['settings'][n]['min'] == 0.0 for n in ('edge_margin', 'obstacle_margin', 'spin_margin'))
    assert got['settings']['edge_margin']['max'] == 1.0
    assert rpc(node, 'planner.settings.set', {'spin_margin': 0.0, 'obstacle_margin': 0.2}, 'r2')[1] is None
    data = the_map(planner={'edge_margin': 0.0, 'spin_margin': 0.01})
    data['areas'][1]['properties']['margin'] = 0.02
    send_map(node, data)
    plan_area(node)
    cfg = planned.config()
    assert (cfg.edge_margin, cfg.obstacle_margin, cfg.spin_margin) == (0.0, 0.2, 0.01)
    assert list(planned.calls[-1][0][7]) == [0.02]
    over = [override('edge_margin', 0.0), override('obstacle_margin', 0.03), override('spin_margin', 0.0)]
    cfg = node.transit_config(node.values(over))
    assert (cfg.edge_margin, cfg.obstacle_margin, cfg.spin_margin) == (0.0, 0.03, 0.0)


@fake_only
def test_plan_transit_in_a_group_of_its_own(make, monkeypatch):
    """~/plan_transit in a callback group of its own (answered while ~/plan_coverage waits for its plan, see
    test_plan_transit_while_a_plan_is_held), ~/plan_coverage and ~/plan_from_parameters in one of theirs, the other
    callbacks in the node's own; the executor runs all three. Planned in
    the node's transit thread whichever thread takes the request: what it keeps of the map is made once."""
    node = make()
    group = node.service_groups['~/plan_transit']
    assert isinstance(group, os_module.MutuallyExclusiveCallbackGroup)
    planning = node.service_groups['~/plan_coverage']
    assert isinstance(planning, os_module.MutuallyExclusiveCallbackGroup) and planning is not group
    assert node.service_groups['~/plan_from_parameters'] is planning
    assert node.service_groups['~/cut_plan'] is group  # (in plan_transit's thread)
    assert all(g is None for name, g in node.service_groups.items() if name not in (
        '~/plan_transit', '~/cut_plan', '~/plan_coverage', '~/plan_from_parameters'))
    assert os_module.THREADS >= 3
    real, where, rooms = os_module.plan_transit, [], []

    def spy(*args, **kw):
        where.append(threading.current_thread())
        try:
            return real(*args, **kw)
        finally:
            rooms.append(transit_route._ROOM.last[1])
    monkeypatch.setattr(os_module, 'plan_transit', spy)
    answers = [transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))]
    other = threading.Thread(target=lambda: answers.append(transit(node, (6.5, 4.0, 0.0), (1.0, 1.0, math.pi))))
    other.start()
    other.join(60.0)
    assert [a.success for a in answers] == [True, True]
    assert where == [node._transit.thread] * 2 and where[0].name == 'planner transit' and where[0].daemon
    assert rooms[0] is rooms[1] and node._transit.thread is not node.core.worker.thread


@pytest.mark.skipif(not REAL_ROS, reason='a real executor')
def test_plan_transit_while_a_plan_is_held(make, monkeypatch):
    """On MowBite OS' executor (main's) ~/plan_transit answers at once while a ~/plan_coverage waits for its plan (held
    here till the transit is answered): mowbite_logic's next approach never waits for a planner.plan of another. An RPC
    is answered meanwhile too."""
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    node = make()
    node._on_map(String(data=json.dumps(the_map())))
    node._on_map_status(MapStatus(datum_state=MapStatus.DATUM_MATCH, mowing_allowed=True, reason=''))
    held, go = threading.Event(), threading.Event()
    real = core_module.plan_coverage

    def holding(*args, **kw):
        held.set()
        go.wait(30.0)
        return real(*args, **kw)
    monkeypatch.setattr(core_module, 'plan_coverage', holding)
    caller = rclpy.create_node('caller')
    coverage = caller.create_client(PlanCoverage, '/mowbite_planner/plan_coverage')
    transits = caller.create_client(PlanTransit, '/mowbite_planner/plan_transit')
    executor = MultiThreadedExecutor(num_threads=os_module.THREADS)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        assert coverage.wait_for_service(timeout_sec=10.0) and transits.wait_for_service(timeout_sec=10.0)
        request = PlanCoverage.Request(boundary=polygon(GARDEN), obstacles=[polygon(BED)])
        planning = coverage.call_async(request)
        side = SingleThreadedExecutor()
        side.add_node(caller)
        assert held.wait(30.0)
        t0 = time.monotonic()
        answer = transits.call_async(PlanTransit.Request(
            start=pose(1.0, 1.0, 0.0), goal=pose(6.5, 4.0, math.pi / 2), use_goal_heading=True,
            drivable_areas=[polygon(GARDEN)], obstacles=[polygon(BED)]))
        side.spin_until_future_complete(answer, timeout_sec=10.0)
        took = time.monotonic() - t0
        assert answer.done() and answer.result().success, answer.result()
        assert took < 1.0 and not planning.done()
        # (and an RPC: the node's other callbacks go on too)
        answers = []
        caller.create_subscription(RpcResponse, '/bridge/rpc/response', answers.append, 10)
        asks = caller.create_publisher(RpcRequest, '/bridge/rpc/request', 10)
        t0 = time.monotonic()
        while not any(a.id == 'held' for a in answers) and time.monotonic() - t0 < 10.0:
            asks.publish(RpcRequest(method='planner.settings', params='', id='held'))
            side.spin_once(timeout_sec=0.2)
        assert any(a.id == 'held' for a in answers) and not planning.done()
        go.set()
        side.spin_until_future_complete(planning, timeout_sec=60.0)
        assert planning.result().success, planning.result().message
    finally:
        go.set()
        executor.shutdown()
        caller.destroy_node()


def files(folder) -> dict:
    """Every folder and file under folder: its size, mtime and digest."""
    out = {}
    for root, dirs, names in os.walk(folder):
        for d in dirs:
            out[os.path.relpath(os.path.join(root, d), folder)] = 'folder'
        for name in names:
            path = os.path.join(root, name)
            with open(path, 'rb') as f:
                out[os.path.relpath(path, folder)] = (os.path.getsize(path), os.stat(path).st_mtime_ns,
                                                      hashlib.sha256(f.read()).hexdigest())
    return out


@fake_only
def test_role_transit(make):
    """role transit: ~/plan_transit only, a name of its own, nothing sent, no methods, no clients, nothing planned
    ahead, one transit at a time (workers 0); map/json, map/status and bridge/params read. planner by default."""
    assert make().role() == 'planner'
    node = make(role='transit', node_name='mowbite_planner_transit', workers=2)
    assert set(node.services) == {'~/plan_transit'} and not node.publishers
    assert set(node.subscribed) == {'map/json', 'map/status', 'bridge/params'}
    assert not node.called and not node.timed and not node.rpc_on
    assert node.core.methods == {} and node.core.workers == 0 and node.files == ''
    lines = node.get_logger().lines
    assert ('info', 'workers not taken (role transit: plans in one thread)') in lines
    assert ('info', 'MowBite Planner ready (role transit): service ~/plan_transit only') in lines
    send_map(node, the_map())
    give(node, 'map/status', MapStatus(datum_state=MapStatus.DATUM_MISMATCH, reason=''))
    send_map(node, the_map(name='Lawn'))
    geometry(node, version=1, robot_front=0.55)
    assert node.core.ahead is None and node.core.map['areas'][0]['properties']['name'] == 'Lawn'
    res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert res.success, res.message
    node.send_out('mowbite/planner/progress', '{}', True)  # (nowhere)
    with pytest.raises(ValueError, match='role transit needs a name of its own, mowbite_planner is the planner'):
        make(role='transit')
    with pytest.raises(ValueError, match='role must be planner or transit, got \'coverage\''):
        make(role='coverage', node_name='mowbite_planner_coverage')


@fake_only
def test_role_transit_writes_nothing(make, tmp_path):
    """role transit next to the planner in its data_dir: nothing written there, the planner's counters (angle steps,
    timing, plans kept) untouched; the planner.* settings the planner stored taken (again once they change, one not
    readable: the ones before, none: the launch's)."""
    planner = make()
    send_map(planner, the_map())
    assert rpc(planner, 'planner.settings.set', {'turn_radius': 0.6, 'edge_margin': 0.08})[1] is None
    assert plan_area(planner).success
    give(planner, 'bridge/publish', MqttPublish(topic='events/json', payload=json.dumps(
        {'type': 'JOB_COMPLETE', 'job_id': 'j1', 'x': 1.0, 'y': 2.0})))
    planner.core.drive_thread.join(60.0)
    data = str(tmp_path / 'data')
    before = files(data)
    assert {'planner/mowbite_planner_settings.json', 'planner/mowbite_planner_angle.json'} <= set(before)
    assert any(n.startswith('planner/plans/') and n.endswith('.pkl') for n in before)
    node = make(role='transit', node_name='mowbite_planner_transit')
    send_map(node, the_map())
    give(node, 'bridge/params', String(data=json.dumps(
        {'node': '/mowbite_follower', 'params': {'speed_fast': 0.5, 'speed_slow': 0.2, 'max_angular': 1.0}})))
    cfg = node.transit_config(node.values())
    assert (cfg.turn_radius, cfg.edge_margin, cfg.robot_front) == (0.6, 0.08, 0.60)
    assert transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2)).success
    geometry(node, version=1, robot_front=0.55)
    assert transit(node, (1.0, 1.0, 0.0), (1.0, 4.0, math.pi)).success
    assert files(data) == before
    assert rpc(planner, 'planner.settings.set', {'edge_margin': 0.12}, 'r2')[1] is None
    assert node.transit_config(node.values()).edge_margin == 0.08  # (taken with the next transit)
    transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    cfg = node.transit_config(node.values())
    assert (cfg.turn_radius, cfg.edge_margin, cfg.robot_front) == (0.6, 0.12, 0.55)
    settings = os.path.join(data, 'planner', 'mowbite_planner_settings.json')
    with open(settings, 'w') as f:
        f.write('{"edge_margin": ')
    transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert node.transit_config(node.values()).edge_margin == 0.12
    assert any('the planner\'s settings not taken (%s): ' % settings in line for _, line in node.get_logger().lines)
    os.remove(settings)
    transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    cfg = node.transit_config(node.values())
    assert (cfg.turn_radius, cfg.edge_margin, cfg.robot_front) == (0.25, 0.05, 0.55)


@fake_only
def test_transit_keeps_the_strictest_of_the_areas(make):
    """A transit's settings: the strictest margins and edges of the map's areas its drivable areas are, each with its
    own settings over the planner.* ones (TRANSIT_STRICT); a drivable area of no area of the map: the strictest of all
    of them, said; none of the map: the planner.* settings."""
    node = make()
    path = [(8.0, 1.0), (10.0, 1.0), (10.0, 2.0), (8.0, 2.0)]
    data = the_map(planner={'edge_margin': 0.15, 'edges': 'hard'})
    data['areas'].append({'id': 'n', 'outline': ring(path), 'properties': {
        'name': 'Path', 'type': 'nav', 'planner': {'spin_margin': 0.12, 'edge_margin': 0.02}}})
    send_map(node, data)
    plain = node.transit_config(node.values())
    assert (plain.edge_margin, plain.spin_margin, plain.edges) == (0.05, 0.05, 'recorded')
    said = []
    cfg = node.transit_config(node.values(), [GARDEN], said)
    assert (cfg.edge_margin, cfg.spin_margin, cfg.edges, said) == (0.15, 0.05, 'hard', [])
    cfg = node.transit_config(node.values(), [GARDEN, path], said)
    assert (cfg.edge_margin, cfg.spin_margin, cfg.edges, said) == (0.15, 0.12, 'hard', [])
    cfg = node.transit_config(node.values(), [path], said)
    assert (cfg.edge_margin, cfg.spin_margin, cfg.edges, said) == (0.02, 0.12, 'recorded', [])
    cfg = node.transit_config(node.values(), [[(0.0, 0.0), (9.0, 0.0), (9.0, 5.0), (0.0, 5.0)]], said)
    assert (cfg.edge_margin, cfg.spin_margin, cfg.edges) == (0.15, 0.12, 'hard')
    assert said == ['1 of the drivable areas no area of the map (to 1 mm): the strictest settings of its areas kept to']
    res = transit(node, (1.0, 1.0, 0.0), (7.0, 4.0, math.pi / 2), drivable=([(0.0, 0.0), (9.0, 0.0), (9.0, 5.0),
                                                                            (0.0, 5.0)],))
    assert 'no area of the map' in res.message


def test_role_transit_with_the_real_ros(make):
    """role transit made and asked like on MowBite OS (with a sourced workspace: the real rclpy)."""
    node = make(role='transit', node_name='mowbite_planner_transit')
    assert node.get_name() == 'mowbite_planner_transit' and node.core.methods == {}
    res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert res.success, res.message


def test_main_is_the_os_node(monkeypatch):
    """The executable mowbite_planner_os_node runs OsNode (mowbite_planner's main), the node mowbite_planner."""
    got = []
    monkeypatch.setattr(os_module.base, 'main', lambda args=None, node_class=None, executor_class=None: got.append(
        (node_class, executor_class())))
    if REAL_ROS:
        rclpy.init()  # (an executor of the real rclpy needs it)
    try:
        os_module.main()
        assert got[0][0] is OsNode and isinstance(got[0][1], os_module.MultiThreadedExecutor)
        if not REAL_ROS:
            assert got[0][1].num_threads == os_module.THREADS
    finally:
        if REAL_ROS:
            got[0][1].shutdown()
            rclpy.try_shutdown()
    with open(os.path.join(HERE, '..', 'setup.py')) as f:
        assert "'mowbite_planner_os_node = mowbite_planner_os.node:main'," in f.read()


def test_with_the_real_ros(make):
    """Made and asked like on MowBite OS (with a sourced workspace: the real rclpy, mowbite_msgs and interfaces)."""
    node = make()
    assert node.get_name() == 'mowbite_planner' if REAL_ROS else node.name == 'mowbite_planner'
    assert sorted(node.core.methods) == CORE_METHODS
    res = transit(node, (1.0, 1.0, 0.0), (6.5, 4.0, math.pi / 2))
    assert res.success, res.message
