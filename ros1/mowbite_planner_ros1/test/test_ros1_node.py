"""Tests of the ROS 1 node without ROS: rospy and the message packages are stood in for here.

    python3 -m pytest ros1/mowbite_planner_ros1/test   (shapely is enough)
"""

import dataclasses
import hashlib
import importlib.machinery
import importlib.util
import json
import logging
import math
import os
import queue
import re
import shutil
import struct
import sys
import tempfile
import time
import threading
import types
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
# the node keeps the settings changed over RPC in $ROS_HOME, not in the real one
os.environ['ROS_HOME'] = tempfile.mkdtemp()
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'ros2', 'mowbite_planner'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'sim'))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'ros2', 'mowbite_planner', 'test'))


class Msg:
    _fields = {}

    def __init__(self, **kwargs):
        for name, default in self._fields.items():
            setattr(self, name, kwargs.pop(name) if name in kwargs else default())
        assert not kwargs, kwargs


def msg(name, **fields):
    return type(name, (Msg,), {'_fields': fields})


def install():
    point = msg('Point', x=float, y=float, z=float)
    point32 = msg('Point32', x=float, y=float, z=float)
    quat = msg('Quaternion', x=float, y=float, z=float, w=lambda: 1.0)
    pose = msg('Pose', position=point, orientation=quat)
    header = msg('Header', seq=int, stamp=lambda: 0, frame_id=str)
    pose_stamped = msg('PoseStamped', header=header, pose=pose)
    polygon = msg('Polygon', points=list)
    nav_path = msg('Path', header=header, poses=list)
    planner_path = msg('Path', is_outline=int, path=nav_path)
    request = msg('PlanPathRequest', fill_type=int, angle=float, distance=float, outer_offset=float, outline_count=int,
                  outline_overlap_count=int, skip_area_outline=bool, skip_obstacle_outlines=bool, skip_fill=bool,
                  outline=polygon, holes=list)
    request.FILL_LINEAR, request.FILL_CONCENTRIC = 0, 1
    response = msg('PlanPathResponse', paths=list)
    vector3 = msg('Vector3', x=float, y=float, z=float)
    twist = msg('Twist', linear=vector3, angular=vector3)
    twist_stamped = msg('TwistStamped', header=header, twist=twist)
    imu = msg('Imu', header=header, angular_velocity=vector3, linear_acceleration=vector3)
    absolute = msg('AbsolutePose', header=header, flags=int, orientation_valid=int, motion_vector_valid=int,
                   position_accuracy=float, pose=msg('PoseWithCovariance', pose=pose, covariance=list),
                   motion_vector=vector3, vehicle_heading=float, motion_heading=float)
    absolute.FLAG_GPS_RTK_FIXED, absolute.FLAG_GPS_RTK_FLOAT = 2, 4
    rpc_error = msg('RpcError', code=int, message=str, id=str)
    rpc_error.ERROR_INVALID_PARAMS, rpc_error.ERROR_INTERNAL = -32602, -32603

    class Rospy(types.ModuleType):
        params = {}
        log = []
        services = {}
        subscribers = {}
        published = []
        registered = []
        messages = {}  # (the last one on a topic, for wait_for_message)

        class ROSException(Exception):
            pass

        class ServiceException(Exception):
            pass

        class Time:
            @staticmethod
            def now():
                return 123

        def get_param(self, name, default=None):
            return self.params.get(name, default)

        def Subscriber(self, topic, kind, callback, **kwargs):
            self.subscribers[topic] = callback
            return types.SimpleNamespace(unregister=lambda: self.subscribers.pop(topic, None))

        def Publisher(self, topic, kind, **kwargs):
            return types.SimpleNamespace(publish=lambda m: self.published.append((topic, m)))

        def ServiceProxy(self, name, kind):
            return lambda *a: self.registered.append(a)

        def wait_for_service(self, name, timeout=None):
            pass

        def wait_for_message(self, topic, kind, timeout=None):
            if topic not in self.messages:
                raise self.ROSException('timeout exceeded while waiting for message on topic %s' % topic)
            return self.messages[topic]

        def Service(self, name, kind, handler):
            self.services[name] = handler
            return types.SimpleNamespace(resolved_name=name)

        # (and to the logger rosout like rospy's, at INFO as rospy sets it up)
        def loginfo(self, *a, **kwargs):
            self.log.append(('info', a[0] % a[1:] if len(a) > 1 else a[0]))
            logging.getLogger('rosout').info(*a, **kwargs)

        def logwarn(self, *a, **kwargs):
            self.log.append(('warn', a[0] % a[1:] if len(a) > 1 else a[0]))
            logging.getLogger('rosout').warning(*a, **kwargs)

        def logerr(self, *a, **kwargs):
            self.log.append(('error', a[0] % a[1:] if len(a) > 1 else a[0]))
            logging.getLogger('rosout').error(*a, **kwargs)

        def logwarn_throttle(self, period, *a):
            self.logwarn(*a)

    rospy = Rospy('rospy')
    logging.getLogger('rosout').setLevel(logging.INFO)
    modules = {
        'rospy': rospy,
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.SimpleNamespace(Header=header, String=msg('String', data=str), Empty=msg('Empty')),
        'xbot_mqtt': types.ModuleType('xbot_mqtt'),
        'xbot_mqtt.msg': types.SimpleNamespace(RpcRequest=msg('RpcRequest', method=str, params=str, id=str),
                                               RpcResponse=msg('RpcResponse', result=str, id=str), RpcError=rpc_error,
                                               MqttPublish=msg('MqttPublish', topic=str, payload=str, retain=bool)),
        'xbot_mqtt.srv': types.SimpleNamespace(RegisterMethodsSrv=object),
        'geometry_msgs': types.ModuleType('geometry_msgs'),
        'geometry_msgs.msg': types.SimpleNamespace(Point=point, Point32=point32, Pose=pose, PoseStamped=pose_stamped,
                                                   Quaternion=quat, Polygon=polygon, Twist=twist,
                                                   TwistStamped=twist_stamped),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.SimpleNamespace(Imu=imu),
        'xbot_msgs': types.ModuleType('xbot_msgs'),
        'xbot_msgs.msg': types.SimpleNamespace(AbsolutePose=absolute),
        'mower_msgs': types.ModuleType('mower_msgs'),
        'mower_msgs.msg': types.SimpleNamespace(
            HighLevelStatus=msg('HighLevelStatus', state_name=str, is_charging=bool, emergency=bool),
            Status=msg('Status', rain_detected=bool)),
        'nav_msgs': types.ModuleType('nav_msgs'),
        'nav_msgs.msg': types.SimpleNamespace(Path=nav_path),
        'slic3r_coverage_planner': types.ModuleType('slic3r_coverage_planner'),
        'slic3r_coverage_planner.msg': types.SimpleNamespace(Path=planner_path),
        'slic3r_coverage_planner.srv': types.SimpleNamespace(PlanPath=object, PlanPathRequest=request,
                                                             PlanPathResponse=response),
    }
    sys.modules.update(modules)
    path = os.path.join(HERE, '..', 'nodes', 'mowbite_planner_node')
    loader = importlib.machinery.SourceFileLoader('mowbite_planner_node', path)
    spec = importlib.util.spec_from_loader('mowbite_planner_node', loader)
    node = importlib.util.module_from_spec(spec)
    loader.exec_module(node)
    return rospy, node, request, polygon, point32


rospy, node_module, Request, Polygon, Point32 = install()


from mowbite_planner import calibread, mqtt_service  # noqa: E402
import calibsim  # noqa: E402
from stubplan import Spy  # noqa: E402


@pytest.fixture(autouse=True)
def stub_planner(request, monkeypatch):
    """The node and the preview plan with the stand-in (stubplan.py), what they asked for in its calls; with the mark
    real_planner with plan_coverage itself."""
    spy = Spy()
    if 'real_planner' not in request.keywords:
        monkeypatch.setattr(node_module, 'plan_coverage', spy)
        monkeypatch.setattr(mqtt_service, 'plan_coverage', spy)
    return spy

from mowbite_planner.mqtt_service import PlannerService  # noqa: E402

GARDEN = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
BED = [(8.0, 4.0), (11.0, 4.0), (11.0, 6.0), (8.0, 6.0)]


def polygon(points, float32=False):
    def f(v):  # geometry_msgs/Point32 as it comes from mower_logic
        return struct.unpack('<f', struct.pack('<f', v))[0] if float32 else float(v)

    return Polygon(points=[Point32(x=f(x), y=f(y), z=0.0) for x, y in points])


def request(**kw):
    r = Request(fill_type=0, angle=0.0, distance=0.13, outer_offset=0.0, outline_count=3,
                outline=polygon(kw.pop('outline', GARDEN)), holes=[polygon(h) for h in kw.pop('holes', [BED])])
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def area(id_, points, **props):
    return {'id': id_, 'properties': props, 'outline': [{'x': x, 'y': y} for x, y in points]}


def the_map(planner=None, areas=(), **props):
    if planner:
        props['planner'] = planner
    return {'areas': [
        area('a', GARDEN + [GARDEN[0]], name='Garden', type='mow', **props),
        area('bed', BED, type='obstacle'),
        area('nav', [(20, 0), (24, 0), (24, 3), (20, 3)], type='nav'),
        *areas,
    ], 'docking_stations': [{'id': 'd', 'position': {'x': 22.0, 'y': 1.5}, 'heading': math.pi}]}


# the properties of an area mower_map kept before open_mower_ros #364 (MapArea's from_json and to_json), all since
STOCK_PROPERTIES = ('name', 'type', 'active', 'mowable', 'angle', 'outline_count', 'outline_overlap_count',
                    'outline_offset')


def through_mower_map(data, keeps_all):
    """The map as mower_map publishes it on json_map."""
    return dict(data, areas=[dict(a, properties={k: v for k, v in a['properties'].items()
                                                 if keeps_all or k in STOCK_PROPERTIES}) for a in data['areas']])


def send_map(data):
    rospy.subscribers['/mower_map_service/json_map'](types.SimpleNamespace(data=json.dumps(data)))


@pytest.fixture
def planner():
    rospy.params = {}
    rospy.log.clear()
    rospy.published.clear()
    rospy.registered.clear()
    rospy.messages.clear()
    for name in ('mowbite_planner_settings.json', 'mowbite_planner_timing.json', 'mowbite_planner_angle.json'):
        try:
            os.remove(os.path.join(os.environ['ROS_HOME'], name))
        except FileNotFoundError:
            pass
    # no plans kept from other tests, none planned ahead in the background (test_plan_ahead does that itself)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'mowbite_plans'), ignore_errors=True)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'position_history'), ignore_errors=True)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'mowbite_map_backups'), ignore_errors=True)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'mowbite_calib'), ignore_errors=True)
    node_module.PLAN_AHEAD_DELAY = 1e6
    # no mowbite/hold read (no broker here: it would hold leftovers_auto's mower in), test_hold does that itself
    node_module.HOLD = False
    # nor the house computer's broker (remote_planning): the tests of it hand the node one of their own
    node_module.REMOTE = False
    # (a finished mow's track is there at once, test_track_read_when_the_job_ended waits for it)
    node_module.TRACK_WAIT = 0.0
    # (the progress of the tests' nodes before off: a change of what waits is sent a while after, into this test)
    for node in NODES:
        node.progress.publish = lambda payload: None
    NODES[:] = [node_module.MowbitePlannerRos1()]
    return NODES[0]


NODES = []


def poses(res):
    return [ps.pose for p in res.paths for ps in p.path.poses]


def inside(res, rect):
    """The poses well inside a rectangle."""
    (x0, y0), (x1, y1) = rect[0], rect[2]
    return [p for p in poses(res) if x0 + 0.1 < p.position.x < x1 - 0.1 and y0 + 0.1 < p.position.y < y1 - 0.1]


def lane_angle():
    """The lane direction of the last plan in degrees, from the log."""
    return float(re.search(r'lanes at (-?[0-9.]+) deg', [line for _, line in rospy.log if 'lanes at' in line][-1])[1])


def check(res, max_poses=node_module.MAX_PATH_POSES):
    assert res.paths
    for p in res.paths:
        assert p.is_outline in (0, 1)
        assert 3 <= len(p.path.poses) <= max_poses
        assert p.path.header.frame_id == 'map'
        for a, b in zip(p.path.poses, p.path.poses[1:]):
            pa, pb, q = a.pose.position, b.pose.position, a.pose.orientation
            assert math.hypot(pb.x - pa.x, pb.y - pa.y) <= 0.1 + 1e-6
            assert math.isclose(q.z * q.z + q.w * q.w, 1.0)


def test_answers_like_slic3r(planner):
    assert '/slic3r_coverage_planner/plan_path' in rospy.services
    res = planner.handle(request())
    check(res)
    assert [p.is_outline for p in res.paths] == [1, 0]
    # outer_offset 0: the first loop runs on the recorded outline
    first = res.paths[0].path.poses[0].pose.position
    assert min(first.x, first.y, 20 - first.x, 10 - first.y) < 1e-6
    # the default for OpenMower's forward only controller: never backwards (no K-turns)
    for a, b in zip(poses(res), poses(res)[1:]):
        if math.hypot(b.position.x - a.position.x, b.position.y - a.position.y) < 1e-6:
            continue  # (where one path ends the next starts)
        travel = math.atan2(b.position.y - a.position.y, b.position.x - a.position.x)
        yaw = 2.0 * math.atan2(a.orientation.z, a.orientation.w)
        assert abs(math.remainder(travel - yaw, 2.0 * math.pi)) < math.pi / 2


def test_same_request_same_plan(planner, monkeypatch):
    a = [(p.position.x, p.position.y, p.orientation.z) for p in poses(planner.handle(request()))]
    planner.plans.clear()  # planned again, not the kept one
    b = [(p.position.x, p.position.y, p.orientation.z) for p in poses(planner.handle(request()))]
    assert a == b
    # kept: the same request again (mowing.plan, planner.plan of the area, the mow) isn't planned again, another is
    calls, real = [], node_module.plan_coverage
    monkeypatch.setattr(node_module, 'plan_coverage', lambda *args, **kw: calls.append(1) or real(*args, **kw))
    planner.handle(request())
    assert not calls
    send_map(the_map())
    result, error = rpc('planner.plan', {'area_id': 'a'})
    assert error is None and result['paths'] and calls == [1]
    planner.handle(request(angle=0.3))
    rpc('planner.settings.set', {'turn_radius': 0.4})
    planner.handle(request(angle=0.3))
    assert calls == [1, 1, 1]
    for angle in range(node_module.CACHED_PLANS + 2):
        planner.handle(request(angle=0.1 * angle))
    assert len(planner.plans) == node_module.CACHED_PLANS


def test_area_settings_and_dock_from_the_map(planner):
    plain = planner.handle(request())
    send_map(the_map({'fill_pattern': 'crosshatch', 'angle_strategy': 'longest_edge'}))
    res = planner.handle(request(angle=0.7))
    check(res)
    # crosshatch mows the lanes twice, along the long edge (the 0.7 rad from mower_logic is overridden)
    assert len(poses(res)) > 1.6 * len(poses(plain))
    # starts on the loop next to the dock and ends near it
    first, last = poses(res)[0].position, poses(res)[-1].position
    assert math.hypot(first.x - 20.0, first.y - 1.5) < 1.5
    assert math.hypot(last.x - 22.0, last.y - 1.5) < 3.0
    assert any('area a' in line for _, line in rospy.log)


def test_unknown_outline_is_planned_without_the_map(planner):
    send_map(the_map())
    check(planner.handle(request(outline=[(0.0, 0.0), (15.0, 0.0), (15.0, 8.0), (0.0, 8.0)])))
    rospy.subscribers['/mower_map_service/json_map'](types.SimpleNamespace(data='not json'))
    assert any(level == 'warn' for level, _ in rospy.log)


def test_concentric_and_splitting(planner):
    rospy.params = {'~max_path_poses': 400}
    small = node_module.MowbitePlannerRos1()
    res = small.handle(request(fill_type=Request.FILL_CONCENTRIC))
    check(res, 400)
    assert len(res.paths) > 10
    # pieces of one run continue where the last one stopped
    for a, b in zip(res.paths, res.paths[1:]):
        if a.is_outline == b.is_outline:
            pa, pb = a.path.poses[-1].pose.position, b.path.poses[0].pose.position
            assert math.hypot(pb.x - pa.x, pb.y - pa.y) <= 0.1 + 1e-6
    # OpenMower's FTC controller needs 3 poses a path, pieces of at most 3 can't always have them: at least 4
    rospy.params = {'~max_path_poses': 3}
    tiny = node_module.MowbitePlannerRos1()
    assert tiny.max_poses == 4
    check(tiny.handle(request(outline=[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)], holes=[])), 4)


def test_split_area(planner):
    # a bed across the garden from edge to edge: both halves get their loops and lanes
    res = planner.handle(request(holes=[[(9.0, -1.0), (11.0, -1.0), (11.0, 11.0), (9.0, 11.0)]]))
    check(res)
    left = {p.is_outline for p in res.paths if all(ps.pose.position.x < 9.5 for ps in p.path.poses)}
    right = {p.is_outline for p in res.paths if all(ps.pose.position.x > 10.5 for ps in p.path.poses)}
    assert left == right == {0, 1} and len(res.paths) == 4
    assert any('stub: 2 sections' in line for _, line in rospy.log)  # (the plan's warnings logged)


def test_bad_area_gives_no_paths(planner):
    res = planner.handle(request(outline=[(0.0, 0.0), (1.0, 1.0)]))
    assert res.paths == []
    assert any(level == 'warn' for level, _ in rospy.log)


def test_settings_parameter(planner):
    rospy.params = {'~settings': {'lane_order': 'snake', 'turn_types': 'detour,pivot'}}
    p = node_module.MowbitePlannerRos1()
    assert p.base.lane_order == 'snake' and p.base.turn_types == ('detour', 'pivot')
    check(p.handle(request()))


def test_settings_parameter_angle(planner):
    # an angle or angle_strategy in ~settings beats mower_logic's angle; lane spacing (overlap), loops and offset
    # set here beat mower_logic's, the skip flags always come with the request
    for settings, angle in (({}, 0.7), ({'angle': 0.3}, 0.3), ({'angle_strategy': 'longest_edge'}, None),
                            ({'angle': None}, None)):
        rospy.params = {'~settings': dict(settings, perimeter_passes=5, overlap=0.5, skip_lanes=True)}
        cfg = node_module.MowbitePlannerRos1().settings(request(angle=0.7), None)
        assert cfg.angle == angle and cfg.perimeter_passes == 5 and cfg.lane_spacing == pytest.approx(0.09)
        assert not cfg.skip_lanes and cfg.lane_overlap_passes == request().outline_overlap_count
        rospy.params = {'~settings': settings}
        cfg = node_module.MowbitePlannerRos1().settings(request(angle=0.7), None)
        assert cfg.perimeter_passes == 3 and cfg.lane_spacing == pytest.approx(0.13)
    # an area's own outline_count still beats the one set here
    rospy.params = {'~settings': {'perimeter_passes': 5}}
    assert node_module.MowbitePlannerRos1().settings(request(), node_module.area_job(
        the_map(outline_count=2), 0)).perimeter_passes == 2
    rospy.params = {'~settings': {'angle_strategy': 'longest_edge'}}
    node_module.MowbitePlannerRos1().handle(request(angle=0.7))
    assert lane_angle() == 0.0
    # an area with an angle of its own: mower_logic's angle counts (its mow_angle_offset in it), the area's
    # planner property still beats it
    rospy.params = {'~settings': {'angle': 0.3}}
    p = node_module.MowbitePlannerRos1()
    for props, angle in (({}, 0.3), ({'angle': 0.5}, 0.7), ({'angle': 0.5, 'planner': {'angle': 0.2}}, 0.2),
                         ({'angle': 0.5, 'planner': {'angle_strategy': 'min_width'}}, None)):
        assert p.settings(request(angle=0.7), node_module.area_job(the_map(**props), 0)).angle == angle, props


@pytest.mark.parametrize('keeps_all', [False, True])
def test_mowbite_properties_need_mower_map_to_keep_them(planner, keeps_all):
    """An area's planner property, angle range and mow_around reach the node only through a mower_map that keeps the
    extra area properties (open_mower_ros #364), before it they were dropped. An area not to mow keeps the blade off
    it either way ('mowable' is mower_map's own: without mow_around it's an area not to mow, plan_coverage's
    unmowed)."""
    plain = planner.handle(request())
    pond = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    send_map(through_mower_map(the_map({'fill_pattern': 'crosshatch'}, angle_min=0.2, angle_max=0.4, areas=[
        area('pond', pond, type='mow', mowable=False, mow_around=True)]), keeps_all))
    res = planner.handle(request(angle=1.0))
    check(res)
    assert (len(poses(res)) > 1.6 * len(poses(plain))) == keeps_all  # crosshatch
    assert not inside(res, pond)  # mowed around, or not mowed
    if keeps_all:  # in the area's range, not mower_logic's 1 rad
        assert math.degrees(0.2) - 0.05 <= lane_angle() <= math.degrees(0.4) + 0.05
    else:
        assert lane_angle() == round(math.degrees(1.0), 1)


def test_obstacles_come_with_the_request(planner):
    # mowing.plan previews an edit before it's saved: the bed moved, the map still has it at the old place. Like
    # slic3r the node plans the request's holes, from the map only the areas to mow around come on top
    moved = [(x + 6.0, y) for x, y in BED]
    pond = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    send_map(the_map(areas=[area('pond', pond, type='mow', mowable=False, mow_around=True)]))
    res = planner.handle(request(holes=[moved]))
    check(res)
    assert inside(res, BED) and not inside(res, moved) and not inside(res, pond)
    assert inside(planner.handle(request(holes=[])), BED)


def test_margins_of_the_map_obstacles(planner, monkeypatch):
    """An obstacle's own margin in the map: the node finds mower_logic's hole (float32) by its outline, the loops keep
    that much further off it. The plan is kept for the holes in any order, each with its margin; an obstacle the map
    doesn't have (an edit not saved yet) keeps obstacle_margin."""
    pond = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    send_map(the_map(areas=[area('pond', pond, type='obstacle', margin=0.4)]))
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14})
    got, real = [], node_module.plan_coverage
    monkeypatch.setattr(node_module, 'plan_coverage', lambda *args, **kw: got.append(args) or real(*args, **kw))

    def sent(holes):
        r = request()
        r.holes = [polygon(h, True) for h in holes]
        return r

    def near(res, rect):  # how close the poses come to it
        (x0, y0), (x1, y1) = rect[0], rect[2]
        return min(math.hypot(max(x0 - p.position.x, 0.0, p.position.x - x1), max(y0 - p.position.y, 0.0,
                                                                                  p.position.y - y1))
                   for p in poses(res))

    res = planner.handle(sent([BED, pond]))
    check(res)
    margins = {o[0]: m for o, m in zip(got[-1][1], got[-1][7])}
    assert margins == {BED[0]: None, pond[0]: 0.4}
    cfg = got[-1][2]
    for rect, margin in ((BED, cfg.obstacle_margin), (pond, 0.4)):  # (a drive or turn body_tolerance closer)
        assert margin - cfg.body_tolerance - 1e-3 <= near(res, rect) <= margin + 0.01
    assert check(planner.handle(sent([pond, BED]))) is None and len(got) == 1
    moved = [(x + 0.5, y) for x, y in pond]
    planner.handle(sent([BED, moved]))
    assert got[-1][7] == (None, None)


def test_settings_of_before_left_out(planner):
    """Settings that are fixed now or gone (allow_reverse, min_turn_radius, smooth_spins, waypoint_spacing, ...) are
    left out where they still come: from the settings file (it plans as without, they're gone from it after the next
    set) and from an app, the rest set as asked. planner.settings doesn't list them."""
    with open(stored_file(), 'w') as f:
        json.dump({'allow_reverse': True, 'min_turn_radius': 0.3, 'body_tolerance': 0.1, 'turn_radius': 0.3}, f)
    node = node_module.MowbitePlannerRos1()
    assert node.current.stored == {'turn_radius': 0.3} and not hasattr(node.base, 'allow_reverse')
    result, error = rpc('planner.settings.set', {'smooth_spins': False, 'waypoint_spacing': 0.2, 'edge_margin': 0.06})
    assert error is None and result['settings']['edge_margin']['value'] == 0.06
    assert not {'allow_reverse', 'min_turn_radius', 'smooth_spins', 'waypoint_spacing'} & set(result['settings'])
    with open(stored_file()) as f:
        assert json.load(f) == {'edge_margin': 0.06, 'turn_radius': 0.3}
    assert 'can_back_up' not in result


def test_area_in_the_area_mowed_on_its_own(planner):
    # a zone in the garden with an angle of its own: the garden mows round it, unless nested_areas is off
    zone = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    send_map(the_map(areas=[area('zone', zone, type='mow', angle=1.2)]))
    res = planner.handle(request(holes=[]))
    check(res)
    assert not inside(res, zone)
    rpc('planner.settings.set', {'nested_areas': False})
    assert inside(planner.handle(request(holes=[])), zone)


def test_no_way_over_navigation_areas(planner):
    # a hedge across the garden and a drive along its top. mower_logic mows a path from start to end, so each
    # side gets paths of its own and mower_logic drives from one to the other
    hedge = [(9.0, -1.0), (11.0, -1.0), (11.0, 10.0), (9.0, 10.0)]
    drive = [(7.0, 10.0), (13.0, 10.0), (13.0, 13.0), (7.0, 13.0)]
    send_map(the_map(areas=[area('drive', drive, type='nav')]))
    res = planner.handle(request(holes=[hedge]))
    check(res)
    assert [p.is_outline for p in res.paths] == [1, 0, 1, 0]
    assert all(p.position.y < 10.0 + 1e-6 for p in poses(res))
    # another lawn there may be crossed, it's mowed anyway
    send_map(the_map(areas=[area('back', drive, type='mow')]))
    assert [p.is_outline for p in planner.handle(request(holes=[hedge])).paths] == [1, 0]


def test_areas_with_the_same_outline(planner):
    # the area with the request's outline that's mowed counts, not an inactive or not mowable one before it
    # (else the first one with it)
    plain = planner.handle(request())
    crosshatch = {'fill_pattern': 'crosshatch'}
    off, meadow = area('off', GARDEN, type='mow', active=False), area('meadow', GARDEN, type='mow', mowable=False)
    off_crosshatch = area('off', GARDEN, type='mow', active=False, planner=crosshatch)
    garden, lawn = area('a', GARDEN, type='mow', planner=crosshatch), area('b', GARDEN, type='mow')
    for areas, crosshatched in (([off, meadow, garden], True), ([garden, off], True),
                                ([off_crosshatch, meadow], True), ([off_crosshatch, lawn], False)):
        send_map({'areas': areas})
        res = planner.handle(request())
        check(res)
        assert (len(poses(res)) > 1.6 * len(poses(plain))) == crosshatched, [a['id'] for a in areas]


def test_areas_with_the_same_id_or_none(planner):
    # the area with the request's outline counts, not the first one with its id (one that can't be planned)
    plain = planner.handle(request())
    other = area('a', [(30.0, 0.0), (40.0, 0.0), (40.0, 10.0), (30.0, 10.0)], type='mow', planner={'turn_types': 5})
    garden = area('a', GARDEN, type='mow', planner={'fill_pattern': 'crosshatch'})
    for first, second in ((other, garden), ({k: v for k, v in a.items() if k != 'id'} for a in (other, garden))):
        send_map({'areas': [first, second]})
        res = planner.handle(request())
        check(res)
        assert len(poses(res)) > 1.6 * len(poses(plain)) and res.paths[0].is_outline == 1


# the preview over MQTT (ext.mowbite_planner.plan) and what mower_logic mows ---------------------------------

# the first point more than 2 m from the first isn't along the longest edge: OpenMower's own angle differs
LAWN = [(0.0, 4.0), (2.0, 0.0), (20.0, 0.0), (20.0, 12.0), (0.0, 12.0)]
PARAMS = dict(tool_width=0.15, outline_count=2, outline_overlap_count=0, outline_offset=0.0, mow_angle_offset=0.0,
              mow_angle_offset_is_absolute=False)
POND = area('pond', [(2.0, 8.0), (4.0, 8.0), (4.0, 10.0), (2.0, 10.0)], type='mow', mowable=False, mow_around=True)


def mower_logic_request(props, outline, holes, params):
    """What MowingBehavior::plan_area sends for an area (no mow_angle_increment yet), the outline and the holes
    as float32 like the map it gets from mower_map."""
    r = request(outline=[], holes=[], distance=params['tool_width'],
                outline_count=props.get('outline_count', params['outline_count']),
                outline_overlap_count=params['outline_overlap_count'],
                outer_offset=props.get('outline_offset', params['outline_offset']))
    r.outline, r.holes = polygon(outline, True), [polygon(h, True) for h in holes]
    angle = props.get('angle')
    if angle is None:  # OpenMower's own: towards the first point more than 2 m from the first
        (x0, y0), *rest = [(p.x, p.y) for p in r.outline.points]
        angle = next((math.atan2(y - y0, x - x0) for x, y in rest if math.hypot(x - x0, y - y0) > 2.0), 0.0)
    offset = math.fmod(params['mow_angle_offset'] + 180, 360)
    offset = (offset + 360 if offset < 0 else offset) - 180
    r.angle = offset * (math.pi / 180) if params['mow_angle_offset_is_absolute'] else angle + offset * (math.pi / 180)
    return r


@pytest.mark.parametrize('props, params, settings, areas, by', [
    ({'angle': 0.5}, {}, {}, (), 0.0),
    ({'angle': 0.5}, {'mow_angle_offset': 10.0}, {}, (), 0.0),
    ({'angle': 0.5}, {'mow_angle_offset': 30.0, 'mow_angle_offset_is_absolute': True}, {}, (), 0.0),
    ({'angle': 0.5}, {'mow_angle_offset': 180.0}, {}, (), 0.0),  # -180 in OpenMower
    ({}, {}, {}, (), 0.0),  # OpenMower's own angle
    ({}, {'mow_angle_offset': -100.0}, {}, (), 0.0),
    ({}, {'mow_angle_offset': 200.0, 'mow_angle_offset_is_absolute': True}, {}, (), 0.0),
    ({'planner': {'angle_strategy': 'min_width'}}, {'mow_angle_offset': 10.0}, {}, (), 0.0),
    ({'planner': {'angle_strategy': 'min_width'}}, {'mow_angle_offset': 30.0, 'mow_angle_offset_is_absolute': True},
     {}, (), 0.0),
    ({'angle': 0.5, 'planner': {'angle': 0.2}}, {'mow_angle_offset': 10.0}, {}, (), 0.0),
    ({'angle': 0.5, 'planner': {'angle_offset': 0.1}}, {'mow_angle_offset': 10.0}, {}, (), 0.0),
    # angle or angle_strategy in the settings for all areas: the area's own angle still counts
    ({'angle': 0.5}, {'mow_angle_offset': 10.0}, {'angle_strategy': 'min_width'}, (), 0.0),
    ({}, {'mow_angle_offset': 10.0}, {'angle_strategy': 'min_width'}, (), 0.0),
    ({}, {'mow_angle_offset': 30.0, 'mow_angle_offset_is_absolute': True}, {'angle': 0.3}, (), 0.0),
    ({'angle': 0.5, 'angle_min': 0.8, 'angle_max': 1.2, 'outline_count': 3}, {}, {}, (POND,), 0.0),
    # coordinates no float32 has: mower_logic sends the outline and the bed rounded, the node reads the pond
    # from the map
    pytest.param({}, {'mow_angle_offset': 10.0}, {}, (POND,), 0.1,
                 marks=pytest.mark.real_planner),  # (one of the four tests with the planner itself)
])
def test_preview_plans_like_the_node(planner, props, params, settings, areas, by):
    """ext.mowbite_planner.plan with the mower's map and parameters (map/json, params/json) gives the poses the
    node gives mower_logic for that area, with the same settings for all areas (PLANNER_SETTINGS, ~settings)."""
    params = dict(PARAMS, **params)

    def moved(a):  # by `by` m in x and y
        return dict(a, outline=[{'x': p['x'] + by, 'y': p['y'] + by} for p in a['outline']])

    lawn, bed = moved(area('lawn', LAWN, name='Lawn', type='mow', **props)), moved(area('bed', BED, type='obstacle'))
    data = {'areas': [lawn, bed, moved(area('nav', [(20, 0), (24, 0), (24, 3), (20, 3)], type='nav')),
                      *map(moved, areas)],
            'docking_stations': [{'id': 'd', 'position': {'x': 22.0 + by, 'y': 1.5 + by}, 'heading': math.pi}]}
    service = PlannerService(node_module.PlannerConfig(lane_order='skip', mower_width=0.18)
                             .with_overrides(settings), 'angle' in settings or 'angle_strategy' in settings)
    service.on_map(json.dumps(data))
    service.on_params(json.dumps({'/mower_logic/' + k: v for k, v in params.items()}))
    preview = service.handle({'jsonrpc': '2.0', 'id': '1', 'method': 'ext.mowbite_planner.plan',
                              'params': {'area_id': 'lawn', 'poses': True}})['result']

    rospy.params = {'~mower_width': 0.18, '~settings': settings}
    node = node_module.MowbitePlannerRos1()
    send_map(data)
    outline, hole = ([(p['x'], p['y']) for p in a['outline']] for a in (lawn, bed))
    res = node.handle(mower_logic_request(props, outline, [hole], params))
    check(res)
    mowed = []
    for p in res.paths:  # the pieces of a run share the pose where one ends and the next starts
        xy = [(ps.pose.position.x, ps.pose.position.y) for ps in p.path.poses]
        mowed += xy[1:] if mowed and math.hypot(xy[0][0] - mowed[-1][0], xy[0][1] - mowed[-1][1]) < 1e-9 else xy
    assert lane_angle() == pytest.approx(math.degrees(preview['angle']), abs=0.05)
    assert len(mowed) == len(preview['poses'])
    assert max(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(mowed, preview['poses'])) < 1e-3
    # and planner.plan's (planning ahead) is that very plan, kept: mower_logic's request doesn't plan it again
    rospy.params['/mower_logic'] = params
    assert node.rpc_plan({'area_id': 'lawn'})['kept'] is True


# OpenMower RPCs (planner.settings, planner.settings.set, planner.plan)

ANSWERS = ('/xbot/rpc/response', '/xbot/rpc/error')


def answers():
    """The RPC answers published (not the progress)."""
    return [(topic, m) for topic, m in rospy.published if topic in ANSWERS]


def rpc(method, params=None, rid='1'):
    """Hands the node a request like xbot_monitoring does, gives (result, None) or (None, (code, message)); a method
    answered on a thread of its own (node_module.BACKGROUND) once it's there."""
    rospy.published.clear()
    raw = '' if params is None else params if isinstance(params, str) else json.dumps(params)
    rospy.subscribers['/xbot/rpc/request'](node_module.RpcRequest(method=method, params=raw, id=rid))
    for t in threading.enumerate():
        if t.name == 'rpc %s' % method:
            t.join(300.0)
    assert len(answers()) <= 1
    if not answers():
        return None, None
    topic, m = answers()[0]
    assert m.id == rid
    if topic == '/xbot/rpc/response':
        return json.loads(m.result), None
    assert topic == '/xbot/rpc/error'
    return None, (m.code, m.message)


def stored_file():
    return os.path.join(os.environ['ROS_HOME'], 'mowbite_planner_settings.json')


def test_rpc_methods_registered(planner):
    for t in threading.enumerate():  # the first registration runs on a thread of its own
        if t is not threading.current_thread() and t.daemon:
            t.join(1.0)
    methods = ['planner.angle.reset', 'planner.area.set', 'planner.calib.result', 'planner.calib.start',
               'planner.calib.stop', 'planner.drive', 'planner.leftovers', 'planner.log', 'planner.plan',
               'planner.settings', 'planner.settings.set', 'planner.simulate', 'planner.simulate.cancel',
               'planner.simulate.status']
    assert ('mowbite_planner', methods) in rospy.registered
    rospy.registered.clear()
    rospy.subscribers['/xbot/rpc/registry'](None)  # xbot_monitoring started again
    assert rospy.registered == [('mowbite_planner', methods)]


def test_web_page(planner):
    """The settings page: the page itself, the node's map, its RPC methods like over MQTT (the errors too), only
    JSON taken. A port taken: the node plans all the same, the log says so."""
    send_map(the_map())
    page = node_module.WebPage(0, planner.methods, planner.current_map)
    port = page.server.server_address[1]
    base = 'http://127.0.0.1:%d' % port

    def post(body, kind='application/json'):
        req = urllib.request.Request(base + '/rpc', json.dumps(body).encode(), {'Content-Type': kind})
        with urllib.request.urlopen(req, timeout=60) as r:
            return json.load(r)

    try:
        with urllib.request.urlopen(base + '/', timeout=10) as r:
            assert r.status == 200 and b'<title>MowBite Planner</title>' in r.read()
        with urllib.request.urlopen(base + '/map', timeout=10) as r:
            assert [a['id'] for a in json.load(r)['areas']] == ['a', 'bed', 'nav']
        assert post({'method': 'planner.settings'})['result']['settings']['lane_order']['value'] == 'skip'
        got = post({'method': 'planner.settings.set', 'params': {'turn_radius': 0.3}})['result']['settings']
        assert got['turn_radius']['value'] == 0.3 and got['turn_radius']['stored']
        plan = post({'method': 'planner.plan', 'params': {'area_id': 'a', 'report': True}})['result']
        assert plan['area_id'] == 'a' and plan['paths'] and 'left_m2' in plan['report']
        assert post({'method': 'planner.nothing'})['error']['code'] == node_module.METHOD_NOT_FOUND
        bad = post({'method': 'planner.settings.set', 'params': {'turn_radius': -1}})['error']
        assert bad['code'] == node_module.RpcError.ERROR_INVALID_PARAMS and 'turn_radius' in bad['message']
        assert post({'method': 'planner.settings', 'params': [1]})['error']['message'] == \
            'params must be an object (by name)'
        for url, body, code in ((base + '/rpc', b'{"method": "planner.settings"}', 415), (base + '/nothing', None,
                                                                                          404)):
            with pytest.raises(urllib.error.HTTPError) as e:
                urllib.request.urlopen(urllib.request.Request(url, body, {'Content-Type': 'text/plain'}), timeout=10)
            assert e.value.code == code
        rospy.params['~web_port'] = port
        taken = node_module.MowbitePlannerRos1()
        assert taken.web is None and any('no settings page on port %d' % port in line for _, line in rospy.log)
    finally:
        page.server.shutdown()
        page.server.server_close()


def test_area_set(planner, monkeypatch):
    """planner.area.set: an area's own settings and an obstacle's margin into the map with OpenMower's map.replace
    (the whole map, only that area changed), the map before backed up. A setting back to all areas' takes OpenMower's
    property for it along, the planner object goes when it's empty. Bad values and OpenMower's no change nothing."""
    send_map(the_map(planner={'turn_radius': 0.3}, outline_count=4, angle=0.5))
    answers = []

    def openmower():  # mower_map: answers what it's asked (answers[0] an error, else the result)
        for _ in range(200):
            asked = [m for t, m in rospy.published if t == '/xbot/rpc/request']
            if asked:
                m = asked[-1]
                if answers and answers[0]:
                    rospy.subscribers['/xbot/rpc/error'](node_module.RpcError(code=-32602, message=answers[0],
                                                                                id=m.id))
                else:
                    rospy.subscribers['/xbot/rpc/response'](node_module.RpcResponse(result='"stored"', id=m.id))
                return m
            time.sleep(0.01)

    def area_set(params, error=None):
        rospy.published.clear()
        answers[:] = [error]
        t = threading.Thread(target=openmower)
        t.start()
        try:
            return planner.rpc_area_set(params)
        finally:
            t.join()

    got = area_set({'area_id': 'a', 'settings': {'turn_radius': None, 'perimeter_passes': None,
                                                 'fill_pattern': 'crosshatch'}})
    sent = [m for t, m in rospy.published if t == '/xbot/rpc/request'][-1]
    assert sent.method == 'map.replace'
    written = json.loads(sent.params)[0]
    assert got['properties'] == written['areas'][0]['properties']
    assert written['areas'][0]['properties'] == {'name': 'Garden', 'type': 'mow', 'angle': 0.5,
                                                 'planner': {'fill_pattern': 'crosshatch'}}
    assert written['areas'][1:] == the_map()['areas'][1:] and planner.current_map() == written
    backups = os.listdir(os.path.join(os.environ['ROS_HOME'], 'mowbite_map_backups'))
    assert len(backups) == 1
    with open(os.path.join(os.environ['ROS_HOME'], 'mowbite_map_backups', backups[0])) as f:
        assert json.load(f)['areas'][0]['properties']['planner'] == {'turn_radius': 0.3}
    monkeypatch.setattr(node_module, 'MAP_BACKUPS', 2)
    got = area_set({'area_id': 'a', 'settings': {'fill_pattern': None, 'angle': None}})
    assert got['properties'] == {'name': 'Garden', 'type': 'mow'}
    got = area_set({'area_id': 'bed', 'margin': 0.2})
    assert got['properties'] == {'type': 'obstacle', 'margin': 0.2}
    backups = sorted(os.listdir(os.path.join(os.environ['ROS_HOME'], 'mowbite_map_backups')))
    assert len(backups) == 2 and backups[0].startswith('map_') and backups[0] != backups[1]
    for params, message in (({'area_id': 'x', 'settings': {'turn_radius': 0.3}}, "no area 'x'"),
                            ({'area_id': 'a', 'settings': {'turn_radius': -1}}, 'turn_radius'),
                            ({'area_id': 'a', 'settings': {'planner': 'slic3r'}}, 'not settings an area can have'),
                            ({'area_id': 'bed', 'settings': {'turn_radius': 0.3}}, 'only for mowing areas'),
                            ({'area_id': 'a', 'margin': 0.2}, 'only for obstacles'),
                            ({'area_id': 'bed', 'margin': 2}, 'margin must be 0 to')):
        rospy.published.clear()
        with pytest.raises(node_module.PlanningError, match=message):
            planner.rpc_area_set(params)
        assert not rospy.published
    with pytest.raises(node_module.PlanningError, match='OpenMower: Invalid map'):
        area_set({'area_id': 'bed', 'margin': None}, 'Invalid map')
    assert planner.current_map()['areas'][1]['properties'] == {'type': 'obstacle', 'margin': 0.2}


def test_rpc_settings(planner):
    assert rpc('planner.settings', 'null') == rpc('planner.settings', {})
    result, error = rpc('planner.settings')
    assert error is None and result['file'] == stored_file() and result['own_angle'] is False
    s = result['settings']
    assert set(s) == set(node_module.PlannerConfig.names()) | {'planner', 'leftovers_auto', 'remote_planning'}
    assert s['leftovers_auto'] == {'value': False, 'default': False, 'stored': False, 'type': 'boolean',
                                   'settable': True}
    # (no house computer here: see test_remote_planning)
    assert s['remote_planning'] == {'value': False, 'default': False, 'stored': False, 'type': 'boolean',
                                    'settable': True, 'ignored': 'no house computer'}
    assert result['remote'] == {'reachable': False, 'arch': None, 'code_match': False, 'stack_match': False,
                                'why': 'no house computer'}
    assert s['planner'] == {'value': 'mowbite', 'default': 'mowbite', 'stored': False, 'type': 'string',
                            'settable': True, 'choices': ['mowbite', 'slic3r']}
    assert s['turn_radius'] == {'value': 0.25, 'default': 0.25, 'stored': False, 'type': 'number', 'settable': True,
                                'unit': 'm', 'min': 0.0, 'max': 10.0}
    assert s['mower_width']['value'] == 0.18 and s['mower_width']['settable']
    assert s['lane_order']['value'] == 'skip' and s['lane_order']['choices'] == ['snake', 'skip']
    assert s['turn_types']['type'] == 'list' and s['turn_types']['value'] == s['turn_types']['choices']
    assert s['angle'] == {'value': None, 'default': None, 'stored': False, 'type': 'number', 'settable': True,
                          'nullable': True, 'unit': 'rad'}
    assert s['perimeter_passes']['type'] == 'integer' and s['perimeter_passes']['max'] == 255
    assert not s['skip_lanes']['settable'] and s['skip_lanes']['from']
    # mower_logic's until set here
    for name in ('overlap', 'perimeter_offset', 'perimeter_passes', 'lane_overlap_passes'):
        assert s[name]['settable'] and s[name]['from'].startswith('/mower_logic/'), name
    assert s['overlap']['from'] == '/mower_logic/tool_width'
    assert s['blade_offset'] == {'value': 0.0, 'default': 0.0, 'stored': False, 'type': 'number', 'settable': True,
                                 'unit': 'm', 'min': -1.0, 'max': 1.0}
    assert s['robot_width']['nullable'] and s['robot_width']['min'] == 0.0 and 'max' not in s['robot_width']
    for name in ('robot_front', 'robot_rear'):
        assert s[name]['nullable'] and s[name]['min'] == 0.0 and s[name]['unit'] == 'm', name
    assert s['edges'] == {'value': 'recorded', 'default': 'recorded', 'stored': False, 'type': 'string',
                          'settable': True, 'choices': ['recorded', 'hard']}
    assert s['blade_ahead']['min'] == -1.0 and s['blade_offset']['min'] == -1.0
    assert s['perimeter_passes']['auto_value'] == -1 and s['perimeter_passes']['min'] == -1
    # (the islands' loops: -1 as many as the outline's)
    assert s['obstacle_passes'] == {'value': -1, 'default': -1, 'stored': False, 'type': 'integer', 'settable': True,
                                    'auto_value': -1, 'min': -1, 'max': 255}
    assert s['lane_spacing_mode']['choices'] == ['fixed', 'auto'] and s['lane_spacing_mode']['value'] == 'fixed'
    assert s['lane_spacing_min']['nullable'] and s['lane_spacing_min']['unit'] == 'm'
    # (what auto goes by: half the blade, and no wider than the blade covers with the mower 2 cm off each side)
    assert s['lane_spacing_min']['effective'] == 0.09 and 'effective_why' not in s['lane_spacing_min']
    assert s['lane_spacing_max']['effective'] == 0.14 and '2 cm' in s['lane_spacing_max']['effective_why']
    assert s['transit_edge_cost']['unit'] == ''
    for name, value in (('headland_turns', False), ('turn_on_spot', False), ('body_fit', True)):
        assert s[name] == {'value': value, 'default': value, 'stored': False, 'type': 'boolean', 'settable': True}
    assert s['headland_corners'] == {'value': 'rounded', 'default': 'rounded', 'stored': False, 'type': 'string',
                                     'settable': True, 'choices': ['rounded', 'sharp']}
    for name, value in (('edge_margin', 0.05), ('obstacle_margin', 0.1), ('spin_margin', 0.05)):
        assert s[name] == {'value': value, 'default': value, 'stored': False, 'type': 'number', 'settable': True,
                           'unit': 'm', 'min': 0.0, 'max': 1.0}, name
    # the ones changing nothing since planner 2 say why, the app hides them; the others don't have the key
    assert s['turn_radius_auto']['ignored'] and s['bend_max_gap']['ignored'] and s['lane_overlap_limit']['ignored']
    assert 'ignored' not in s['turn_radius'] and 'ignored' not in s['headland_turns']


def test_rpc_settings_set(planner):
    result, error = rpc('planner.settings.set', {'turn_radius': 1, 'fill_pattern': 'crosshatch',
                                                 'turn_types': 'u_turn,pivot', 'mower_width': 0.2})
    assert error is None
    s = result['settings']
    assert s['turn_radius']['value'] == 1.0 and s['turn_radius']['stored'] and s['turn_radius']['default'] == 0.25
    assert s['turn_types']['value'] == ['u_turn', 'pivot'] and s['mower_width']['value'] == 0.2
    with open(stored_file()) as f:
        assert json.load(f) == {'fill_pattern': 'crosshatch', 'mower_width': 0.2, 'turn_radius': 1.0,
                                'turn_types': ['u_turn', 'pivot']}
    # the next plan uses them, the blade too
    cfg = planner.settings(request(distance=0.13), None)
    assert cfg.turn_radius == 1.0 and cfg.fill_pattern == 'crosshatch' and cfg.mower_width == 0.2
    assert cfg.lane_spacing == pytest.approx(0.13)
    # and so does the node after a restart
    again = node_module.MowbitePlannerRos1()
    assert again.base.turn_radius == 1.0 and again.current.mower_width == 0.2
    # null: back to the default
    result, error = rpc('planner.settings.set', {'turn_radius': None, 'mower_width': None})
    assert result['settings']['turn_radius']['value'] == 0.25 and not result['settings']['turn_radius']['stored']
    assert result['settings']['mower_width']['value'] == 0.18
    with open(stored_file()) as f:
        assert json.load(f) == {'fill_pattern': 'crosshatch', 'turn_types': ['u_turn', 'pivot']}


def test_rpc_settings_set_body(planner):
    # the body's and the turns' settings, the next plan uses them
    body = {'edge_margin': 0.2, 'obstacle_margin': 0.3, 'spin_margin': 0.0, 'body_fit': False, 'headland_turns': True,
            'headland_corners': 'sharp', 'turn_on_spot': True}
    result, error = rpc('planner.settings.set', body)
    assert error is None and {n: result['settings'][n]['value'] for n in body} == body
    assert all(result['settings'][n]['stored'] for n in body)
    cfg = planner.settings(request(), None)
    assert {n: getattr(cfg, n) for n in body} == body


def test_rpc_settings_robot_outline(planner, stub_planner):
    """robot_outline: points [[x, y], ...] in m round base_link, set over RPC (stored as such) or in ~settings (the
    launch file), the next plan uses it: another plan than the rectangle's, kept for it; null: the rectangle again."""
    s = rpc('planner.settings')[0]['settings']['robot_outline']
    assert s == {'value': None, 'default': None, 'stored': False, 'type': 'points', 'settable': True, 'nullable': True,
                 'unit': 'm'}
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14})
    send_map(the_map())
    rectangle, _ = rpc('planner.plan', {'area_id': 'a'})
    outline = [[0.43, -0.03], [0.43, 0.03], [0.26, 0.2], [-0.14, 0.2], [-0.14, -0.2], [0.26, -0.2]]
    result, error = rpc('planner.settings.set', {'robot_outline': outline})
    assert error is None and result['settings']['robot_outline']['value'] == outline
    assert result['settings']['robot_outline']['stored']
    with open(stored_file()) as f:
        assert json.load(f)['robot_outline'] == outline
    assert planner.settings(request(), None).robot_outline == tuple(map(tuple, outline))
    cut, _ = rpc('planner.plan', {'area_id': 'a'})
    assert stub_planner.config().robot_outline == tuple(map(tuple, outline)) and not cut['kept']
    assert rpc('planner.plan', {'area_id': 'a'})[0]['kept']
    result, _ = rpc('planner.settings.set', {'robot_outline': None})
    assert result['settings']['robot_outline']['value'] is None
    again, _ = rpc('planner.plan', {'area_id': 'a'})
    assert again['fingerprint'] == rectangle['fingerprint'] and again['kept']
    rospy.params = {'~settings': {'robot_outline': outline}}
    assert node_module.MowbitePlannerRos1().base.robot_outline == tuple(map(tuple, outline))


def test_rpc_settings_angle(planner):
    # an angle or angle_strategy stored: the lanes go that way unless the area has an angle of its own
    result, _ = rpc('planner.settings.set', {'angle_strategy': 'min_width'})
    assert result['own_angle'] is True
    assert planner.settings(request(angle=0.7), None).angle is None
    rpc('planner.settings.set', {'angle_strategy': None})
    assert planner.settings(request(angle=0.7), None).angle == 0.7


@pytest.mark.parametrize('params, message', [
    ({'turn_radius': -1}, 'turn_radius must be >= 0.0'),
    ({'turn_radius': 11}, 'turn_radius must be <= 10.0'),
    ({'turn_radius': True}, 'turn_radius must be a finite number'),
    ({'turn_radius': '0.3'}, 'turn_radius must be a finite number'),
    ({'fill_pattern': 'spiral'}, 'fill_pattern must be one of'),
    ({'turn_types': []}, 'turn_types must be some of'),
    ({'angle_min': 0.1}, 'angle_min and angle_max go together'),
    ({'mower_width': 0}, 'mower_width must be > 0.0'),
    ({'edge_margin': 1.01}, 'edge_margin must be <= 1.0'),
    ({'obstacle_margin': -0.1}, 'obstacle_margin must be >= 0.0'),
    ({'spin_margin': 1.5}, 'spin_margin must be <= 1.0'),
    ({'headland_corners': 'round'}, 'headland_corners must be one of rounded, sharp'),
    ({'robot_outline': [[1.0, 0.2], [0.5, 0.2], [0.5, -0.2]]}, 'robot_outline must be'),
    ({'body_fit': 1}, 'body_fit must be true or false'),
    ({'allow_reverse': True, 'skip_lanes': True}, 'not settable here: skip_lanes'),
    ({'nope': 1}, 'unknown planner settings: nope'),
    ({'planner': 'fast'}, 'planner must be one of mowbite, slic3r'),
    ({'leftovers_auto': 1}, 'leftovers_auto must be true or false'),
])
def test_rpc_settings_set_refused(planner, params, message):
    rpc('planner.settings.set', {'turn_radius': 0.3})
    before = open(stored_file()).read()
    result, error = rpc('planner.settings.set', params)
    assert result is None and error[0] == -32602 and message in error[1], error
    # nothing changed
    assert open(stored_file()).read() == before and planner.base.turn_radius == 0.3


def test_stored_settings_that_dont_fit_are_left_out(planner):
    for content in ('{"turn_radius": -2}', '[1]', 'not json', '{"overlap_typo": 1}'):
        with open(stored_file(), 'w') as f:
            f.write(content)
        rospy.log.clear()
        node = node_module.MowbitePlannerRos1()
        assert node.base.turn_radius == 0.25 and node.current.stored == {}
        assert any(level == 'warn' and 'stored settings' in line for level, line in rospy.log), content
        check(node.handle(request()))


def test_stored_setting_of_a_newer_version_is_left_out(planner):
    """A stored setting this version doesn't know (a newer one stored it, then back to this one): the others still
    count, the body's size among them."""
    with open(stored_file(), 'w') as f:
        json.dump({'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'of_a_newer_version': True}, f)
    rospy.log.clear()
    node = node_module.MowbitePlannerRos1()
    assert node.base.robot_width == 0.4 and node.base.body
    assert sorted(node.current.stored) == ['robot_front', 'robot_rear', 'robot_width']
    assert any(level == 'warn' and 'of_a_newer_version' in line for level, line in rospy.log)


def test_lane_spacing_range_goes_by_the_blade(planner):
    """lane_spacing_min and lane_spacing_max are checked against the blade's width (mower_width as launched or
    set), for all areas and in an area's own planner property."""
    rospy.params = {'~mower_width': 0.3}
    node = node_module.MowbitePlannerRos1()
    result, error = rpc('planner.settings.set', {'lane_spacing_max': 0.25})
    assert error is None and result['settings']['lane_spacing_max']['value'] == 0.25
    assert 'lane_spacing_max must be <= 0.3' in rpc('planner.settings.set', {'lane_spacing_max': 0.35})[1][1]
    rpc('planner.settings.set', {'lane_spacing_max': None})
    send_map(the_map({'lane_spacing_max': 0.25}))
    check(node.handle(request()))
    send_map(the_map({'lane_spacing_max': 0.35}))
    assert node.handle(request()).paths == []


def test_no_settings_file(planner):
    rospy.params = {'~settings_file': ''}
    node = node_module.MowbitePlannerRos1()
    result, _ = rpc('planner.settings.set', {'turn_radius': 0.4})
    assert result['file'] is None and node.base.turn_radius == 0.4 and not os.path.exists(stored_file())


def test_rpc_plan(planner, stub_planner):
    send_map(the_map())
    rospy.params['/mower_logic'] = {'tool_width': 0.13, 'outline_count': 2, 'mow_angle_offset': 0.0}
    result, error = rpc('planner.plan', {'area_id': 'a'})
    assert error is None and result['area_id'] == 'a' and result['paths']
    assert result['lane_spacing'] == pytest.approx(0.13)
    # trying a setting gives what setting it gives
    tried, _ = rpc('planner.plan', {'area_id': 'a', 'settings': {'turn_radius': 0.6}})
    assert stub_planner.config().turn_radius == 0.6 and not tried['kept']
    rpc('planner.settings.set', {'turn_radius': 0.6})
    after, _ = rpc('planner.plan', {'area_id': 'a'})
    assert after['fingerprint'] == tried['fingerprint']
    # the outline of an area like mowing.plan takes it
    outline, _ = rpc('planner.plan', {'outline': [[0, 0], [6, 0], [6, 4], [0, 4]]})
    assert outline['area_id'] is None and outline['paths']
    # the body checked, where it sticks out in the report
    body = {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'}
    checked, error = rpc('planner.plan', {'area_id': 'a', 'settings': body, 'report': True})
    assert error is None and isinstance(checked['report']['body_out'], list)
    assert all(set(o) == {'x', 'y', 'yaw', 'kind'} for o in checked['report']['body_out'])
    assert stub_planner.config().robot_width == 0.4 and not checked['kept']
    # the planner picks the lane spacing, chosen says which, without report too
    auto, error = rpc('planner.plan', {'area_id': 'a', 'settings': {'lane_spacing_mode': 'auto'}})
    assert error is None and auto['stats']['chosen']['mode'] == 'auto' and result['stats']['chosen']['mode'] == 'fixed'


@pytest.mark.parametrize('method, params, code, message', [
    ('planner.plan', {'area_id': 'nope'}, -32602, 'nope'),
    ('planner.plan', '[1, 2]', -32602, 'params must be an object'),
    ('planner.plan', 'not json', -32602, 'params must be an object'),
    ('planner.settings.set', {'turn_radius': 'x'}, -32602, 'turn_radius'),
    ('planner.log', {'limit': 0}, -32602, 'limit'),
    ('planner.log', {'limit': 2.5}, -32602, 'limit'),
    ('planner.log', {'since': 'yesterday'}, -32602, 'since'),
])
def test_rpc_errors(planner, method, params, code, message):
    send_map(the_map())
    result, error = rpc(method, params)
    assert result is None and error[0] == code and message in error[1], error


def test_rpc_plan_poses_rounded_and_too_big(planner, monkeypatch):
    """poses: true gives every pose, x and y to mm, yaw to 1e-4 rad (Master 31.10.), the fingerprint the unrounded
    ones'; an answer bigger than MQTT takes (export.MAX_ANSWER) is an error saying so, not dropped by the broker."""
    from mowbite_planner import export
    send_map(the_map())
    rospy.params['/mower_logic'] = {'tool_width': 0.13, 'outline_count': 2, 'mow_angle_offset': 0.0}
    full, error = rpc('planner.plan', {'area_id': 'a', 'poses': True})
    assert error is None and full['poses']
    assert all(round(x, 3) == x and round(y, 3) == y and round(yaw, 4) == yaw for x, y, yaw in full['poses'])
    plain, _ = rpc('planner.plan', {'area_id': 'a'})
    assert 'poses' not in plain and plain['fingerprint'] == full['fingerprint']
    monkeypatch.setattr(export, 'MAX_ANSWER', len(json.dumps(plain)) + 10)
    assert rpc('planner.plan', {'area_id': 'a'})[0] == plain
    result, error = rpc('planner.plan', {'area_id': 'a', 'poses': True})
    assert result is None and error[0] == export.ANSWER_TOO_BIG and 'too big' in error[1] and 'poses' in error[1]
    assert any(level == 'warn' for level, _ in rospy.log)


def test_rpc_plan_without_map(planner):
    result, error = rpc('planner.plan', {'area_id': 'a'})
    assert error[0] == -32602 and 'no map' in error[1]


def test_rpc_internal_error_and_others(planner, monkeypatch):
    monkeypatch.setitem(planner.rpc.methods, 'planner.settings', lambda p: 1 / 0)
    result, error = rpc('planner.settings')
    assert error[0] == -32603 and 'division by zero' in error[1]
    assert any(level == 'error' for level, _ in rospy.log)
    # with its traceback
    line = rpc('planner.log', {'limit': 1})[0]['lines'][0]
    assert line['level'] == 'ERROR' and 'Traceback' in line['msg'] and 'ZeroDivisionError' in line['msg']
    # another node's method, a notification (no id): no answer
    assert rpc('mowing.plan', {}) == (None, None)
    assert rpc('planner.settings.set', {'turn_radius': 0.5}, rid='') == (None, None)
    assert planner.base.turn_radius == 0.5


def rpc_waited(method, params=None, rid='1'):
    """rpc() for a method answered on a thread of its own (node_module.BACKGROUND): its answer once it's there."""
    rospy.published.clear()
    rospy.subscribers['/xbot/rpc/request'](node_module.RpcRequest(method=method, params=json.dumps(params), id=rid))
    for t in threading.enumerate():
        if t.name == 'rpc %s' % method:
            t.join(300.0)
    assert len(answers()) == 1
    topic, m = answers()[0]
    if topic == '/xbot/rpc/response':
        return json.loads(m.result), None
    return None, (m.code, m.message)


def test_rpc_simulate(planner):
    """planner.simulate: the plan planner.plan gives, driven by a simulated mower. An ideal one mows what the plan
    mows and takes about as long as the model says; this mower's FTC values count (slower: longer); a realistic one
    runs from its seeds, gives each run's numbers and its track and events on request."""
    send_map(the_map())
    rospy.params['/mower_logic'] = {'tool_width': 0.13, 'outline_count': 2, 'mow_angle_offset': 0.0}
    plan, _ = rpc('planner.plan', {'area_id': 'a'})
    ideal, error = rpc_waited('planner.simulate', {'area_id': 'a', 'model': 'ideal'})
    assert error is None and ideal['area_id'] == 'a' and ideal['fingerprint'] == plan['fingerprint']
    assert ideal['model'] == {'seed': 0} and ideal['nav_errors'] == [] and not ideal['aborted']
    assert abs(ideal['left_m2'] - ideal['planned_left_m2']) < 0.05
    assert ideal['tracking']['lane']['p95'] < 0.01
    assert 0.85 * ideal['duration']['model_min'] < ideal['duration']['mow_min'] < 1.1 * ideal['duration']['model_min']
    assert 'track' not in ideal and 'runs' not in ideal
    rospy.params[node_module.FTC_PARAMS] = {'speed_fast': 0.3, 'forward_only': True, 'debug_pid': True,
                                            'back_up_with_plan': True, 'speed_reverse': 1}
    slower, _ = rpc_waited('planner.simulate', {'area_id': 'a', 'model': 'ideal'})
    assert slower['model'] == {'speed_fast': 0.3, 'back_up_with_plan': True, 'speed_reverse': 1.0, 'seed': 0}
    assert slower['duration']['mow_min'] > ideal['duration']['mow_min']
    small = {'outline': [[0, 0], [6, 0], [6, 4], [0, 4]]}
    real, error = rpc_waited('planner.simulate', dict(small, seeds=2, track=True, playback=2,
                                                      model={'preset': 'realistic', 'seed': 7}))
    assert error is None and real['area_id'] is None and real['model']['seed'] == 7
    assert real['model']['positioning'] is True and real['model']['speed_fast'] == 0.3
    assert [r['seed'] for r in real['runs']] == [7, 8]
    assert real['track'] and all(set(seg) == {'blades', 'points'} for seg in real['track'])
    assert any(seg['blades'] for seg in real['track'])
    assert real['events'][0]['type'] == 'STATE' and real['events'][0]['t'] >= 0
    assert all('job_id' not in e for e in real['events'])
    times = [p[0] for p in real['playback']]
    assert times[0] == 0 and all(b - a >= 2.0 - 0.1 - 1e-6 for a, b in zip(times, times[1:-1]))  # (to 0.1 s)
    assert times[-1] == pytest.approx(real['duration']['min'] * 60.0, abs=3.1)  # (the minutes to 0.1)
    assert {p[4] for p in real['playback']} == {0, 1}
    assert len(json.dumps(real)) < 200000
    assert any('simulated' in line for _, line in rospy.log)


def test_rpc_simulate_status_and_cancel(planner):
    """While a simulation runs, planner.simulate.status says how far it got (planning, simulating, evaluating, the
    fraction growing), another planner.simulate is refused, planner.simulate.cancel ends it (its answer: the error
    "cancelled"); afterwards the status says it doesn't run, a new one runs."""
    send_map(the_map())
    assert rpc('planner.simulate.status')[0] == {'running': False}
    assert rpc('planner.simulate.cancel')[0] == {'cancelled': False}
    rospy.published.clear()
    request = node_module.RpcRequest(method='planner.simulate', params=json.dumps({'area_id': 'a'}), id='long')
    rospy.subscribers['/xbot/rpc/request'](request)
    seen = []
    for _ in range(600):
        status = planner.rpc_simulate_status({})
        if status.get('running') and status['phase'] == 'simulating' and status['fraction'] > 0.05:
            seen.append(status)
            break
        if status.get('running'):
            seen.append(status)
        time.sleep(0.05)
    assert seen and seen[-1]['phase'] == 'simulating' and seen[-1]['area_id'] == 'a'
    assert all(a['fraction'] <= b['fraction'] for a, b in zip(seen, seen[1:]))
    with pytest.raises(node_module.PlanningError, match='running'):
        planner.rpc_simulate({'area_id': 'a'})
    assert planner.rpc_simulate_cancel({}) == {'cancelled': True}
    for t in threading.enumerate():
        if t.name == 'rpc planner.simulate':
            t.join(300.0)
    long = [m for _, m in answers() if m.id == 'long']
    assert len(long) == 1 and long[0].code == -32602 and 'cancelled' in long[0].message
    assert planner.rpc_simulate_status({})['running'] is False
    done, error = rpc_waited('planner.simulate', {'outline': [[0, 0], [4, 0], [4, 3], [0, 3]], 'model': 'ideal'})
    assert error is None and done['duration']['min'] > 0


@pytest.mark.parametrize('params, message', [
    ({'area_id': 'a', 'seeds': 0}, 'seeds'),
    ({'area_id': 'a', 'seeds': True}, 'seeds'),
    ({'area_id': 'a', 'model': 'fast'}, 'preset'),
    ({'area_id': 'a', 'model': ['ideal']}, 'model must be'),
    ({'area_id': 'a', 'model': {'gps_sigma': 'a lot'}}, 'gps_sigma'),
    ({'area_id': 'a', 'model': {'wheels': 3}}, 'wheels'),
    ({'area_id': 'a', 'playback': 0.1}, 'playback'),
    ({'area_id': 'a', 'playback': 'yes'}, 'playback'),
    ({'area_id': 'nope'}, 'nope'),
])
def test_rpc_simulate_errors(planner, params, message):
    send_map(the_map())
    result, error = rpc_waited('planner.simulate', params)
    assert result is None and error[0] == -32602 and message in error[1], error


def test_rpc_log(planner):
    """planner.log: what the node logs (rospy.loginfo... write to rosout), since: only the newer lines, the oldest
    first, the last LOG_LINES."""
    before = rpc('planner.log')[0]['lines']
    assert before[-1]['level'] == 'INFO' and before[-1]['msg'].startswith('MowBite Planner answers')
    rpc('planner.settings.set', {'turn_radius': 0.4})
    try:
        1 / 0
    except ZeroDivisionError:  # (rospy's own errors in callbacks come with their traceback)
        logging.getLogger('rosout').error('bad callback', exc_info=True)
    logging.getLogger('rosout').critical('fatal')  # rospy.logfatal
    lines = rpc('planner.log', {'since': before[-1]['t']})[0]['lines']
    assert [(line['level'], line['msg'].split('\n')[0][:26]) for line in lines] == [
        ('INFO', 'MowBite Planner: settings '), ('ERROR', 'bad callback'), ('ERROR', 'fatal')]
    assert 'Traceback' in lines[1]['msg'] and 'ZeroDivisionError' in lines[1]['msg']
    assert [line['t'] for line in lines] == sorted(line['t'] for line in lines)
    assert rpc('planner.log', {'limit': 1})[0]['lines'] == lines[-1:]
    assert rpc('planner.log', {'since': lines[-1]['t']})[0]['lines'] == []
    for i in range(node_module.LOG_LINES + 5):
        rospy.loginfo('MowBite Planner: line %d', i)
    lines = rpc('planner.log', {'since': None, 'limit': None})[0]['lines']
    assert len(lines) == node_module.LOG_LINES
    assert lines[-1]['msg'] == 'MowBite Planner: line %d' % (node_module.LOG_LINES + 4)


def test_plans_of_another_code_dropped(planner):
    """At the start the kept plans of another planner code go (none of them is asked for again), its own stay."""
    os.makedirs(planner.plans_dir, exist_ok=True)
    own = planner.plan_file('a' * 40)
    for name in ('0123456789ab-' + 'a' * 40 + '.pkl', 'b' * 40 + '.pkl', os.path.basename(own) + '.tmp'):
        open(os.path.join(planner.plans_dir, name), 'w').close()
    open(own, 'w').close()
    node_module.MowbitePlannerRos1()
    assert os.listdir(planner.plans_dir) == [os.path.basename(own)]


def test_warnings_logged_once_a_plan(planner):
    """A plan's warnings go to the log once, asked for again (a preview, the mow) only how many. Another plan, its own
    warnings again."""
    planner.handle(request())  # (no body size set: the plan warns of it)
    planner.handle(request())
    planner.handle(request(angle=0.5))
    first, again, other = [line for _, line in rospy.log if 'lanes at' in line][-3:]
    assert 'no body check' in first and 'kept from before' not in first
    assert 'kept from before' in again and 'no body check' not in again
    assert re.search(r', \d+ warnings as before$', again)
    assert 'no body check' in other


def test_plan_ahead(planner, monkeypatch):
    """After a new map (or settings) every active mowing area is planned ahead: planner.plan finds it, so does the next
    start of the node (kept on disk, for this planner code only), whichever way the same inputs come."""
    calls, real = [], node_module.plan_coverage
    monkeypatch.setattr(node_module, 'plan_coverage', lambda *args, **kw: calls.append(1) or real(*args, **kw))
    data = the_map(areas=[area('off', [(30, 0), (40, 0), (40, 8), (30, 8)], type='mow', active=False)])
    send_map(data)
    planner.plan_ahead()
    assert calls == [1] and any('1 areas planned ahead' in line for _, line in rospy.log)
    result, error = rpc('planner.plan', {'area_id': 'a'})
    assert error is None and result['paths'] and calls == [1] and result['kept'] is True
    again = node_module.MowbitePlannerRos1()
    send_map(data)
    assert again.rpc_plan({'area_id': 'a'})['paths'] and calls == [1]
    # the same inputs as keywords or in another order of the obstacles: the same plan
    cfg = planner.current.base
    one = planner.plan_coverage(GARDEN, [BED, [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)]], cfg, None, None, [], [])
    two = planner.plan_coverage(GARDEN, [[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)], BED], cfg, start=None, end=None,
                                drivable=(), keep_out=())
    assert one is two and calls == [1, 1]
    # another planner code: planned again
    monkeypatch.setattr(again, 'code', 'other')
    again.plans.clear()
    again.rpc_plan({'area_id': 'a'})
    assert calls == [1, 1, 1]


@pytest.mark.real_planner  # (one of the four tests with the planner itself: the workers don't see the stand-in)
def test_workers(planner):
    """workers: the plans in processes of their own, the areas planned ahead at once, the same plans as in the node's
    own process."""
    second = area('b', [(0, 12), (8, 12), (8, 18), (0, 18)], name='Back', type='mow')
    send_map(the_map(areas=[second]))
    alone = {a: planner.rpc_plan({'area_id': a})['fingerprint'] for a in ('a', 'b')}
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'mowbite_plans'))  # (planned anew, not read back)
    rospy.params['~workers'] = 2
    try:
        together = node_module.MowbitePlannerRos1()
        assert together.workers == 2
        send_map(the_map(areas=[second]))
        together.plans.clear()
        together.plan_ahead()
        assert any('2 areas planned ahead' in line for _, line in rospy.log)
        got = {a: together.rpc_plan({'area_id': a}) for a in ('a', 'b')}
        assert {a: g['fingerprint'] for a, g in got.items()} == alone and all(g['kept'] for g in got.values())
    finally:
        node_module.set_workers(0)


def test_plan_ahead_one_at_a_time(planner, monkeypatch):
    """A plan asked for while it's being planned (planning ahead, then the app or mower_logic) is waited for, not
    planned twice. Planning ahead after another change ends early: the latest plans the areas as they are then."""
    calls, real, started = [], node_module.plan_coverage, threading.Event()

    def slow(*args, **kw):
        calls.append(1)
        started.set()
        time.sleep(0.3)
        return real(*args, **kw)

    monkeypatch.setattr(node_module, 'plan_coverage', slow)
    send_map(the_map())
    ahead = threading.Thread(target=planner.plan_ahead)
    ahead.start()
    assert started.wait(30.0)
    assert planner.rpc_plan({'area_id': 'a'})['kept'] is True
    ahead.join(60.0)
    assert calls == [1] and planner.planning == {}
    # not the latest any more: nothing planned
    rospy.log.clear()
    planner.plans.clear()
    shutil.rmtree(planner.plans_dir)
    planner.ahead_count = 2
    planner.plan_ahead(1)
    assert calls == [1] and not any('planned ahead' in line for _, line in rospy.log)
    planner.plan_ahead(2)
    assert calls == [1, 1] and any('1 areas planned ahead' in line for _, line in rospy.log)


def test_code_version_of_the_code_itself(monkeypatch, tmp_path):
    """Built with catkin the package found first is a stand-in pointing to the code: a change in the code counts."""
    stand_in, code = tmp_path / 'devel', tmp_path / 'src'
    stand_in.mkdir()
    code.mkdir()
    (stand_in / '__init__.py').write_text('# points to src\n')
    (code / 'planner.py').write_text('A = 1\n')
    monkeypatch.setattr(node_module.mowbite_planner, '__path__', [str(stand_in), str(code)])
    before = node_module.code_version()
    (code / 'planner.py').write_text('A = 2\n')
    assert node_module.code_version() != before


def event(kind):
    rospy.subscribers['/xbot_monitoring/mqtt_publish'](node_module.MqttPublish(
        topic='events/json', payload=json.dumps({'id': 'x', 't': 1.0, 'type': kind})))


def test_angle_increment(planner):
    """Every angle_increment_every finished mows (JOB_COMPLETE) the lanes turn angle_increment further, the steps kept
    over a restart, for mower_logic's plans and planner.plan alike, back to none with planner.angle.reset. Nothing is
    counted while nothing turns, nor for JOB_RESET."""
    send_map(the_map())
    event('JOB_COMPLETE')
    assert rpc('planner.settings')[0]['angle_steps'] == 0
    rpc('planner.settings.set', {'angle_increment': math.radians(10), 'angle_increment_every': 2})
    for kind in ('JOB_COMPLETE', 'JOB_RESET', 'JOB_COMPLETE', 'JOB_COMPLETE'):
        event(kind)
    result = rpc('planner.settings')[0]
    assert (result['angle_steps'], result['angle_mows']) == (1, 1)
    assert result['angle_turned'] == pytest.approx(math.radians(10))
    planner.handle(request(angle=0.3))
    assert lane_angle() == pytest.approx(math.degrees(0.3) + 10, abs=0.05)
    turned = rpc('planner.plan', {'area_id': 'a'})[0]['angle']
    result, error = rpc('planner.angle.reset', {})
    assert error is None and (result['angle_steps'], result['angle_mows']) == (0, 0)
    assert rpc('planner.plan', {'area_id': 'a'})[0]['angle'] == pytest.approx(turned - math.radians(10))
    planner.handle(request(angle=0.3))
    assert lane_angle() == pytest.approx(math.degrees(0.3), abs=0.05)
    # kept over a restart (and all finished mows, perimeter_direction random's, the one before turning too)
    event('JOB_COMPLETE')
    event('JOB_COMPLETE')
    assert node_module.MowbitePlannerRos1().angle == {'steps': 1, 'mows': 0, 'finished': 6}


def test_perimeter_direction_random(planner):
    """perimeter_direction random: every finished mow counts (JOB_COMPLETE, with nothing turning too, not JOB_RESET),
    mower_logic's plans and planner.plan's go round the way the count gives (0 ccw, 1 cw, 2 ccw): asked for again
    before the next mow finishes, the same plan. The count is kept over a restart, planner.angle.reset leaves it; with
    another way round it doesn't count, the plan kept stays the one."""
    def way(pts):  # 1: ccw, -1: cw (all of the loops go the same way round)
        return 1 if sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1])) > 0 else -1

    def mower_logic():
        res = planner.handle(request())
        assert res.paths[0].is_outline
        return (way([(ps.pose.position.x, ps.pose.position.y) for ps in res.paths[0].path.poses]),
                [(p.position.x, p.position.y, p.orientation.z) for p in poses(res)])

    def preview():
        result, error = rpc('planner.plan', {'area_id': 'a'})
        assert error is None and result['paths'][0]['is_outline']
        return way(result['paths'][0]['points'])

    send_map(the_map())
    rpc('planner.settings.set', {'perimeter_direction': 'random'})
    assert rpc('planner.settings')[0]['finished_mows'] == 0
    first, plan = mower_logic()
    assert first == 1 == preview() and mower_logic() == (first, plan)
    event('JOB_COMPLETE')
    assert mower_logic()[0] == -1 == preview()
    event('JOB_RESET')
    assert mower_logic()[0] == -1 and rpc('planner.angle.reset', {})[0]['finished_mows'] == 1
    event('JOB_COMPLETE')
    assert mower_logic()[0] == 1 == preview()
    rpc('planner.settings.set', {'perimeter_direction': 'cw'})
    assert preview() == -1
    event('JOB_COMPLETE')
    result, _ = rpc('planner.plan', {'area_id': 'a'})
    assert way(result['paths'][0]['points']) == -1 and result['kept'] is True
    assert node_module.MowbitePlannerRos1().angle['finished'] == 3  # (kept over a restart)


def test_master_restarted(monkeypatch):
    """A new /run_id is a restarted master, one that's down isn't (yet)."""
    monkeypatch.setitem(rospy.params, '/run_id', 'a')
    assert not node_module.master_restarted('a')
    monkeypatch.setitem(rospy.params, '/run_id', 'b')
    assert node_module.master_restarted('a')

    def down(name, default=None):
        raise ConnectionRefusedError
    monkeypatch.setattr(rospy, 'get_param', down)
    assert not node_module.master_restarted('a')


def test_main_ends_when_the_master_restarts(monkeypatch):
    """In a container of its own the node ends with 1 once the master has another /run_id, to register again."""
    ids = iter(['a', 'a', 'b'])
    shut = []
    monkeypatch.setattr(rospy, 'get_param', lambda name, default=None: next(ids))
    monkeypatch.setattr(rospy, 'init_node', lambda name: None, raising=False)
    monkeypatch.setattr(rospy, 'is_shutdown', lambda: False, raising=False)
    monkeypatch.setattr(rospy, 'signal_shutdown', shut.append, raising=False)
    monkeypatch.setattr(node_module, 'MowbitePlannerRos1', lambda: None)
    monkeypatch.setattr(node_module, 'MASTER_CHECK', 0.0)
    with pytest.raises(SystemExit) as end:
        node_module.main()
    assert end.value.code == 1 and shut == ['ROS master restarted']
    assert ('warn', 'MowBite Planner: the ROS master restarted, ending to register again') in rospy.log


def slic3r_path(is_outline, points):
    """A path as OpenMower's slic3r planner answers it, poses facing the next point."""
    poses = []
    for (x, y), (nx, ny) in zip(points, points[1:] + [points[-1]]):
        yaw = math.atan2(ny - y, nx - x) if (nx, ny) != (x, y) else 0.0
        poses.append(node_module.PoseStamped(pose=node_module.Pose(
            position=node_module.Point(x=x, y=y, z=0.0),
            orientation=node_module.Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))
    return node_module.PlannerPath(is_outline=is_outline, path=node_module.Path(poses=poses))


def test_planner_slic3r(planner, monkeypatch):
    """planner slic3r (for all areas, kept like the settings): mower_logic's request goes to OpenMower's slic3r planner
    as it came and its answer back as it is; planner.plan asks it like mower_logic would (OpenMower's settings, none of
    the node's) and gives its paths as a plan, nothing planned ahead. Where the slic3r planner doesn't answer the
    MowBite Planner plans, and the log says why. planner null: the MowBite Planner again."""
    asked = []
    answer = node_module.PlanPathResponse(paths=[slic3r_path(1, [(1.0, 1.0), (2.0, 1.0), (3.0, 1.0)]),
                                                 slic3r_path(0, [(1.0, 2.0), (2.0, 2.0), (3.0, 2.0), (4.0, 2.0)])])

    def proxy(name, kind):  # (the node's others as they were)
        return (lambda req: asked.append(req) or answer) if name == '/slic3r_classic/plan_path' else real(name, kind)

    real = rospy.ServiceProxy
    monkeypatch.setattr(rospy, 'ServiceProxy', proxy, raising=False)
    rospy.params['/mower_logic'] = {'tool_width': 0.14, 'outline_count': 2, 'outline_offset': 0.0,
                                    'outline_overlap_count': 1}
    result, error = rpc('planner.settings.set', {'planner': 'slic3r', 'perimeter_passes': -1})
    assert error is None and result['settings']['planner']['value'] == 'slic3r'
    assert result['settings']['planner']['stored']
    with open(stored_file()) as f:
        assert json.load(f) == {'perimeter_passes': -1, 'planner': 'slic3r'}
    req = request()
    assert planner.handle(req) is answer and asked == [req]
    send_map(the_map())
    result, error = rpc('planner.plan', {'area_id': 'a', 'report': True})
    assert error is None and [p['is_outline'] for p in result['paths']] == [1, 0]
    sent = asked[-1]
    assert sent.distance == pytest.approx(0.14) and sent.outline_count == 2 and len(sent.holes) == 1
    assert result['stats']['path_length'] == pytest.approx(5.0) and 'report' in result
    assert any('slic3r' in w for w in result['warnings'])
    asked.clear()
    planner.plan_ahead()
    assert not asked
    # no slic3r planner: the MowBite Planner's plan, and why in the log
    monkeypatch.setattr(rospy, 'wait_for_service', lambda name, timeout=None: (_ for _ in ()).throw(
        rospy.ROSException('timeout')), raising=False)
    planner.classic = None
    check(planner.handle(request()))
    assert any(level == 'error' and 'no answer from the slic3r planner' in line for level, line in rospy.log)
    # after a restart still, then back to the MowBite Planner
    monkeypatch.setattr(rospy, 'wait_for_service', lambda name, timeout=None: None, raising=False)
    again = node_module.MowbitePlannerRos1()
    assert again.current.planner == 'slic3r' and again.handle(req) is answer
    asked.clear()
    rpc('planner.settings.set', {'planner': None})
    assert again.current.planner == 'mowbite'
    check(again.handle(request()))
    assert not asked


def track_file(job, segments):
    """A position_history file of OpenMower for job: [(blades, [(x, y), ...]), ...]."""
    folder = os.path.join(os.environ['ROS_HOME'], 'position_history')
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, '1791377699_%s.jsonl' % job)
    with open(path, 'w') as f:
        for blades, pts in segments:
            f.write(json.dumps({'type': 'new_segment', 'attributes': {'job_id': job, 'blades': blades},
                                'points': []}) + '\n')
            f.write(json.dumps({'type': 'append_points', 'points': [list(p) for p in pts]}) + '\n')
    return path


def test_duration(planner):
    """planner.plan says how long mowing it takes (duration): the model with OpenMower's FTC speeds, times what the
    finished mows took against it. A finished job (JOB_COMPLETE) counts with the plan mower_logic got for its area (the
    blade running in its track from start to end), kept over a restart; one far off doesn't, nor one with an area
    planned twice (resumed)."""
    send_map(the_map())
    d = rpc('planner.plan', {'area_id': 'a'})[0]['duration']
    assert d['runs'] == 0 and d['factor'] == 1.0 and d['min'] == d['model_min'] > 1.0
    rospy.params[node_module.FTC_PARAMS] = {'speed_fast': 0.8, 'speed_slow': 0.3, 'speed_angular': 90.0}
    assert rpc('planner.plan', {'area_id': 'a'})[0]['duration']['model_min'] == pytest.approx(0.5 * d['model_min'],
                                                                                              rel=0.1)
    del rospy.params[node_module.FTC_PARAMS]

    def mow(job, share, plans=1):
        start = time.time()
        for _ in range(plans):
            planner.handle(request())
        took = share * planner.served[-1][2]
        folder = os.path.join(os.environ['ROS_HOME'], 'position_history')
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, '%d_%s.jsonl' % (start, job)), 'w') as f:
            for t, blades in ((start, False), (start + 20.0, True), (start + 20.0 + took, False)):
                f.write(json.dumps({'type': 'new_segment', 'attributes': {'job_id': job, 'blades': blades},
                                    'started_at': t, 'points': [[1.0, 5.0]]}) + '\n')
        rospy.subscribers['/xbot_monitoring/mqtt_publish'](node_module.MqttPublish(
            topic='events/json', payload=json.dumps({'type': 'JOB_COMPLETE', 'job_id': job})))
        planner.drive_thread.join(60.0)

    mow('j1', 0.9)
    d = rpc('planner.plan', {'area_id': 'a'})[0]['duration']
    assert d['runs'] == 1 and d['factor'] == pytest.approx(0.9, abs=1e-3)
    assert d['min'] == pytest.approx(0.9 * d['model_min'], abs=0.1)
    assert any('job j1 took' in line and 'factor now 0.90' in line for _, line in rospy.log)
    mow('j2', 5.0)
    assert any(level == 'warn' and 'job j2 took' in line and 'not counted' in line for level, line in rospy.log)
    mow('j3', 1.0, plans=2)
    assert any('job j3 not timed' in line for _, line in rospy.log)
    again = node_module.MowbitePlannerRos1()
    assert again.duration(100.0)['runs'] == 1 and again.duration(600.0)['min'] == pytest.approx(9.0)
    # a track whose blade never stopped running: no time
    unended = os.path.join(os.environ['ROS_HOME'], 'position_history', '1_open.jsonl')
    with open(unended, 'w') as f:
        f.write(json.dumps({'type': 'new_segment', 'attributes': {'blades': True}, 'started_at': 1.0, 'points': []}))
    assert node_module.mowing_time(unended) is None


def test_leftovers(planner):
    """planner.leftovers: what a mow's track left of the area and the passes over it. Armed, mower_logic's next plan
    for the area is those passes alone (again while that job runs), an area with nothing left or hardly mowed gets no
    paths; when
    that job is finished it's over and counts as no mow (the lanes don't turn on). Not armed: a look only."""
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded',
                                 'blade_ahead': 0.17, 'angle_increment': 0.1})
    send_map(the_map())
    full, _ = rpc('planner.plan', {'area_id': 'a', 'poses': True})
    # the track: the plan as driven, but for every 12th lane
    runs, lanes = [[]], 0
    for seg in full['segments']:
        lanes += seg['type'] == 'lane'
        if seg['type'] == 'lane' and lanes % 12 == 0:
            runs.append([])
        else:
            runs[-1] += [p[:2] for p in full['poses'][seg['start']:seg['end'] + 1]]
    folder = os.path.join(os.environ['ROS_HOME'], 'position_history')
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, '1791377699_mown.jsonl'), 'w') as f:
        for run in [r for r in runs if r]:
            f.write(json.dumps({'type': 'new_segment', 'attributes': {'blades': True}, 'points': []}) + '\n')
            f.write(json.dumps({'type': 'append_points', 'points': run}) + '\n')
    # (the job went on after a restart: a file of its own, hardly anything in it. Both count for the job)
    with open(os.path.join(folder, '1791377800_mown.jsonl'), 'w') as f:
        f.write(json.dumps({'type': 'new_segment', 'attributes': {'blades': False}, 'points': [[1.0, 5.0]]}) + '\n')
    os.utime(os.path.join(folder, '1791377699_mown.jsonl'), (1.0, 1.0))
    assert 'most of the area' in rpc('planner.leftovers', {'file': '1791377800_mown.jsonl'})[0]['areas']['a']['message']
    assert rpc('planner.leftovers')[0]['files'] == ['1791377699_mown.jsonl', '1791377800_mown.jsonl']
    result, error = rpc('planner.leftovers', {'job_id': 'mown'})
    assert error is None and result['file'] == '1791377800_mown.jsonl' and result['armed'] == 0.0
    assert result['files'] == ['1791377699_mown.jsonl', '1791377800_mown.jsonl']
    a = result['areas']['a']
    assert 'stats' in a, a
    left = a['stats']['leftovers']
    assert a['left_m2'] > 1.0 and left['passes'] >= 3 and left['area'] > 0.6 * a['left_m2'] and a['duration']['min'] > 0
    assert a['paths'] and {s['maneuver'] for path in a['paths'] for s in path['segments']} == {'leftover'}
    whole = len(poses(planner.handle(request())))  # (not armed: the plan as ever)
    assert whole > 5 * sum(path['poses'] for path in a['paths'])
    result, _ = rpc('planner.leftovers', {'job_id': 'mown', 'arm': True})
    assert 0.0 < result['armed'] <= node_module.LEFTOVERS_ARMED
    first = poses(planner.handle(request()))
    assert 3 <= len(first) < 0.2 * whole and len(poses(planner.handle(request()))) == len(first)
    steps = rpc('planner.settings')[0]['angle_steps']
    event('JOB_COMPLETE')
    assert any('the leftovers are mowed' in line for _, line in rospy.log)
    assert len(poses(planner.handle(request()))) == whole and rpc('planner.settings')[0]['angle_steps'] == steps
    # all mowed (a track to and fro across all of it, a point every 0.5 m): nothing to do, armed the area gets no paths
    xs = [0.5 * i for i in range(-1, 42)]
    zigzag = [[x, -0.1 + 0.1 * k] for k in range(103) for x in (xs if k % 2 == 0 else xs[::-1])]
    with open(os.path.join(folder, '1791377700_all.jsonl'), 'w') as f:
        f.write(json.dumps({'type': 'new_segment', 'attributes': {'blades': True}, 'points': []}) + '\n')
        f.write(json.dumps({'type': 'append_points', 'points': zigzag}) + '\n')
    result, _ = rpc('planner.leftovers', {'job_id': 'all', 'arm': True})
    assert result['areas']['a']['paths'] == [] and 'nothing left' in result['areas']['a']['message']
    assert result['areas']['a']['reason'] == 'nothing_left'
    assert planner.handle(request()).paths == []
    # a track that isn't a mow of the area (hardly anything of it mowed, say the job skipped it): no leftovers, armed
    # no paths either (not the whole plan: the leftovers' job mows leftovers only)
    with open(os.path.join(folder, '1791377701_short.jsonl'), 'w') as f:
        f.write(json.dumps({'type': 'new_segment', 'attributes': {'blades': True}, 'points': []}) + '\n')
        f.write(json.dumps({'type': 'append_points', 'points': [p[:2] for p in full['poses'][:200]]}) + '\n')
    result, _ = rpc('planner.leftovers', {'job_id': 'short', 'arm': True})
    assert result['areas']['a']['paths'] == [] and 'most of the area is left' in result['areas']['a']['message']
    assert result['areas']['a']['reason'] == 'most_left' and 0.0 < result['armed']
    assert planner.handle(request()).paths == []
    # asked for by the area's name: armed for its id. The leftovers' job thrown away (JOB_RESET): the plan as ever
    result, _ = rpc('planner.leftovers', {'job_id': 'mown', 'arm': True, 'area_id': 'Garden'})
    assert list(result['areas']) == ['a'] and len(poses(planner.handle(request()))) == len(first)
    event('JOB_RESET')
    assert len(poses(planner.handle(request()))) == whole
    # an area that can't be planned: the others' leftovers still
    send_map(the_map(areas=[area('tiny', [(30, 0), (30.15, 0), (30.15, 0.15), (30, 0.15)], type='mow')]))
    result, error = rpc('planner.leftovers', {'job_id': 'mown'})
    assert error is None and list(result['areas']) == ['a'] and result['areas']['a']['paths']
    assert rpc('planner.leftovers', {'job_id': 'mown', 'area_id': 'tiny'})[0] is None
    assert list(rpc('planner.drive', {'job_id': 'mown'})[0]['areas']) == ['a']
    send_map(the_map())
    # the map changed while they were planned: not armed (they'd be the old map's)
    real = planner.rpc_plan

    def plan_then_new_map(*args, **kw):
        got = real(*args, **kw)
        send_map(the_map(areas=[area('more', [(30, 0), (40, 0), (40, 8), (30, 8)], type='mow', active=False)]))
        return got

    planner.rpc_plan = plan_then_new_map
    result, error = rpc('planner.leftovers', {'job_id': 'mown', 'arm': True, 'area_id': 'a'})
    assert result is None and 'changed meanwhile' in error[1] and planner.armed is None
    del planner.rpc_plan
    send_map(the_map())
    rpc('planner.leftovers', {'job_id': 'all', 'arm': True})
    rpc('planner.settings.set', {'turn_radius': 0.3})  # (new settings: the leftovers were the old ones')
    assert len(poses(planner.handle(request()))) > 100
    assert rpc('planner.leftovers', {'job_id': 'nope'})[1][1].startswith('no track')


def test_leftovers_auto(planner, monkeypatch):
    """leftovers_auto: after a finished mow its leftovers by themselves. Where their passes mow enough the mower is sent
    out for them once it's idle in its dock (no emergency, no rain with rain_mode on) and mower_logic's next plan is
    those passes; that job done starts nothing again. Little left, the mower not ready in time or the setting off: it
    stays in."""
    monkeypatch.setattr(node_module, 'READY_POLL', 0.01)
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded',
                                 'blade_ahead': 0.17})
    send_map(the_map())
    full, _ = rpc('planner.plan', {'area_id': 'a', 'poses': True})
    # the tracks: the plan as driven but for every 12th lane, and all of it
    runs, lanes = [[]], 0
    for seg in full['segments']:
        lanes += seg['type'] == 'lane'
        if seg['type'] == 'lane' and lanes % 12 == 0:
            runs.append([])
        else:
            runs[-1] += [p[:2] for p in full['poses'][seg['start']:seg['end'] + 1]]
    track_file('mown', [(True, run) for run in runs if run])
    track_file('all', [(True, [p[:2] for p in full['poses']])])
    state, status, start = '/mower_logic/current_state', '/ll/mower_status', 'mower_logic:idle/start_mowing'
    idle = node_module.HighLevelStatus(state_name='IDLE', is_charging=True)

    def finished(job):
        """A mow of that job finished: the actions the node sent after it."""
        rospy.published.clear()
        rospy.subscribers['/xbot_monitoring/mqtt_publish'](node_module.MqttPublish(
            topic='events/json', payload=json.dumps({'type': 'JOB_COMPLETE', 'job_id': job})))
        planner.drive_thread.join(120.0)
        return [m.data for topic, m in rospy.published if topic == '/xbot/action']

    rospy.messages[state] = idle
    whole = len(poses(planner.handle(request())))
    assert finished('all') == [] and planner.armed is None  # (off)
    result, error = rpc('planner.settings.set', {'leftovers_auto': True})
    assert error is None and result['settings']['leftovers_auto'] == {
        'value': True, 'default': False, 'stored': True, 'type': 'boolean', 'settable': True}
    with open(stored_file()) as f:
        assert json.load(f)['leftovers_auto'] is True
    # still docking when the mow's event comes: sent out when it's in
    rospy.messages[state] = node_module.HighLevelStatus(state_name='DOCKING')
    timer = threading.Timer(0.2, rospy.messages.update, [{state: idle}])
    timer.start()
    assert finished('mown') == [start]
    timer.join()
    assert any('the mower sent out for' in line for _, line in rospy.log)
    assert 3 <= len(poses(planner.handle(request()))) < 0.2 * whole
    # the leftovers' job done: no mow of its own, nothing sent again
    assert finished('mown') == [] and any('the leftovers are mowed' in line for _, line in rospy.log)
    assert len(poses(planner.handle(request()))) == whole
    # nothing left: the mower stays in, the next plan is the whole one
    assert finished('all') == [] and planner.armed is None
    assert any('the mower stays in' in line for _, line in rospy.log)
    # ready: idle in its dock, no emergency, no rain (that only counts with mower_logic's rain_mode on)
    rain = node_module.Status(rain_detected=True)
    for messages, rain_mode, ready in (
            ({state: node_module.HighLevelStatus(state_name='MOWING')}, 0, False),
            ({state: node_module.HighLevelStatus(state_name='IDLE')}, 0, False),
            ({state: node_module.HighLevelStatus(state_name='IDLE', is_charging=True, emergency=True)}, 0, False),
            ({}, 0, False), ({state: idle}, 1, False), ({state: idle, status: rain}, 1, False),
            ({state: idle, status: rain}, 0, True), ({state: idle, status: node_module.Status()}, 1, True)):
        rospy.messages.clear()
        rospy.messages.update(messages)
        rospy.params['/mower_logic/rain_mode'] = rain_mode
        assert planner.mower_ready() is ready, (messages, rain_mode)
    # not ready while the leftovers are armed: not sent out
    monkeypatch.setattr(node_module, 'LEFTOVERS_ARMED', 0.2)
    rospy.messages[state] = node_module.HighLevelStatus(state_name='IDLE')
    assert finished('mown') == [] and any('the mower not sent out' in line for _, line in rospy.log)
    # planner slic3r: no leftovers
    rospy.messages[state] = idle
    rpc('planner.settings.set', {'planner': 'slic3r'})
    assert finished('mown') == [] and any('no leftovers mowed by itself' in line for _, line in rospy.log)


def test_track_read_when_the_job_ended(planner, monkeypatch):
    """mower_logic's JOB_COMPLETE comes before xbot_monitoring has written all of the job's track (every 30 s, the rest
    when the job ends): what follows a mow waits for mower_logic's next STATE event of another job or none."""
    monkeypatch.setattr(node_module, 'TRACK_WAIT', 60.0)
    monkeypatch.setattr(node_module, 'TRACK_SETTLE', 0.0)
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'})
    send_map(the_map())

    def send(kind, **more):
        rospy.subscribers['/xbot_monitoring/mqtt_publish'](node_module.MqttPublish(
            topic='events/json', payload=json.dumps(dict(more, type=kind))))

    inside = [(x / 10.0, 5.0) for x in range(10, 61)]
    out = [(19.0, 6.0), (19.5, 6.0), (20.05, 6.0), (20.1, 6.0), (19.5, 6.4), (18.0, 6.4)]
    track_file('job1', [(False, [(1.0, 5.0)]), (True, inside)])  # (as far as it's written yet)
    send('STATE', state='MOWING', job_id='job1')
    send('JOB_COMPLETE', job_id='job1')
    send('STATE', state='DOCKING', job_id='job1')
    time.sleep(0.3)
    assert planner.drive_thread.is_alive() and planner.drive is None
    track_file('job1', [(False, [(1.0, 5.0)]), (True, inside), (True, out)])
    send('STATE', state='IDLE')
    planner.drive_thread.join(60.0)
    assert not planner.drive_thread.is_alive() and len(planner.drive['areas']['a']['places']) == 1
    # a job that doesn't end in time: its track as it is
    monkeypatch.setattr(node_module, 'TRACK_WAIT', 0.2)
    send('JOB_COMPLETE', job_id='job1')
    planner.drive_thread.join(60.0)
    assert not planner.drive_thread.is_alive()
    assert any(level == 'warn' and 'job job1 not ended' in line for level, line in rospy.log)


def test_planner_drive(planner):
    """planner.drive: the driven track (OpenMower's position_history) against the plan's body_space, the places where
    the body got more than DRIVE_OFF into the margins, the furthest point of each; after a finished mow by itself, in
    the log and as the answer without params. Without a track an error."""
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'})
    send_map(the_map())
    result, error = rpc('planner.drive', {'job_id': 'nope'})
    assert result is None and 'no track' in error[1]
    inside = [(x / 10.0, 5.0) for x in range(10, 61)]
    # (over the edge where no navigation area lies beyond it)
    out = [(19.0, 6.0), (19.5, 6.0), (20.05, 6.0), (20.1, 6.0), (19.5, 6.4), (18.0, 6.4)]
    track_file('job1', [(False, [(1.0, 5.0)]), (True, inside), (True, out)])
    result, error = rpc('planner.drive', {'job_id': 'job1'})
    assert error is None and result['file'] == '1791377699_job1.jsonl'
    a = result['areas']['a']
    assert a['checked'] > 50 and len(a['places']) == 1 and a['places'][0][:2] == [20.05, 6.0] and a['worst'] > 0.2
    assert node_module.DRIVE_OFF == 0.01
    # after a finished mow by itself
    rospy.subscribers['/xbot_monitoring/mqtt_publish'](node_module.MqttPublish(
        topic='events/json', payload=json.dumps({'type': 'JOB_COMPLETE', 'job_id': 'job1'})))
    planner.drive_thread.join(60.0)
    assert any(level == 'warn' and 'got into the margins at 1 places' in line for level, line in rospy.log)
    assert rpc('planner.drive')[0] == result


def test_planner_drive_openmowers_drives(planner):
    """planner.drive: OpenMower's own drives (the blade off) over the navigation area next to the lawn aren't places,
    out of the lawns and navigation areas they are; the plan's own (the blade on) out of its body_space ever, the
    navigation area in it (reach_over)."""
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'})
    send_map(the_map())
    over_nav = [(19.0, 2.0), (19.5, 2.0), (20.05, 2.0), (20.1, 2.0), (20.3, 2.0)]
    away = [(10.0, -0.4), (11.0, -0.4)]
    track_file('job2', [(False, over_nav), (True, [(x / 10.0, 5.0) for x in range(10, 31)]), (False, away)])
    result, error = rpc('planner.drive', {'job_id': 'job2'})
    places = result['areas']['a']['places']
    assert error is None and len(places) == 1 and places[0][1] == -0.4 and places[0][2] > 0.3
    track_file('job3', [(True, over_nav), (True, [(19.0, 6.0), (19.5, 6.0), (20.05, 6.0), (20.3, 6.0)])])
    assert [p[:2] for p in rpc('planner.drive', {'job_id': 'job3'})[0]['areas']['a']['places']] == [[20.3, 6.0]]


def test_planner_drive_slope(planner, stub_planner, monkeypatch):
    """planner.drive with drive slope: the mower faces up the lanes (the plan's angle) all along them, the body checked
    facing so where it backs down; going up it faces the way it drives."""
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'})
    send_map(the_map())
    track_file('down', [(True, [(19.9 - x / 10.0, 5.0) for x in range(13)])])
    track_file('up', [(True, [(18.4 + x / 10.0, 5.0) for x in range(13)])])
    assert rpc('planner.drive', {'job_id': 'down'})[0]['areas']['a']['places'] == []

    def slope(*args, **kw):  # (the stand-in's plan as backing.updown's says: up the lanes at angle 0)
        plan = stub_planner(*args, **kw)
        return plan if stub_planner.config().drive != 'slope' else \
            dataclasses.replace(plan, angle=0.0, stats=dict(plan.stats, drive='slope'))
    monkeypatch.setattr(node_module, 'plan_coverage', slope)
    rospy.params['/move_base_flex/FTCPlanner/back_up_with_plan'] = True
    rpc('planner.settings.set', {'drive': 'slope'})
    places = rpc('planner.drive', {'job_id': 'down'})[0]['areas']['a']['places']
    assert len(places) == 1 and places[0][:2] == [19.9, 5.0] and places[0][2] > 0.1
    assert rpc('planner.drive', {'job_id': 'up'})[0]['areas']['a']['places'] == []


def test_body_reaches_over_into_navigation_areas(planner):
    """The map's active navigation areas go with every plan (reach_over): the body may reach over the edge into the one
    next to the lawn, beyond the half of its width it may anyway, not into the inactive one."""
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'})
    shed = area('shed', [(-4, 6), (0, 6), (0, 9), (-4, 9)], type='nav', active=False)
    send_map(the_map(areas=[shed]))
    keep = []
    planner.rpc_plan({'area_id': 'a'}, keep)
    space = keep[-1].body_space
    from shapely.geometry import Point as Spot
    assert space.contains(Spot(20.5, 1.5)) and not space.contains(Spot(20.5, 5.0))
    assert not space.contains(Spot(-0.5, 7.5))


def test_drive(planner, stub_planner):
    """drive (slope, reverse) for all areas (planner.settings.set) and an area's own (beats it, null: as for all):
    planned so only with OpenMower's FTC backing up (back_up_with_plan true), the body set and planner mowbite, else
    forwards, said in the plan's warnings and as ignored in planner.settings. Where it's planned so planner.settings
    says what it forces (body_fit, spin_margin), for all areas at body_fit and spin_margin too."""
    send_map(the_map({'drive': 'reverse'}))
    result, _ = rpc('planner.settings')
    drive = result['settings']['drive']
    assert drive['choices'] == ['forward', 'slope', 'reverse'] and drive['value'] == 'forward' and drive['settable']
    assert drive['ignored'] == node_module.NO_BACKING
    plan, error = rpc('planner.plan', {'area_id': 'a'})
    assert error is None and stub_planner.config().drive == 'forward'
    assert plan['warnings'][-1] == 'drive reverse ignored: ' + node_module.NO_BACKING
    rospy.params['/move_base_flex/FTCPlanner/back_up_with_plan'] = True
    result, _ = rpc('planner.settings')
    assert 'no body set' in result['settings']['drive']['ignored']  # (the planner plans it forwards then)
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'spin_margin': 0.0})
    result, _ = rpc('planner.settings')
    drive, spin = result['settings']['drive'], result['settings']['spin_margin']
    assert 'ignored' not in drive and drive['forces'] == {'body_fit': True, 'spin_margin': 0.05}
    assert 'no bumper' in drive['forces_why'] and 'effective' not in spin
    plan, error = rpc('planner.plan', {'area_id': 'a'})
    assert error is None and stub_planner.config().drive == 'reverse'
    assert not any(w.startswith('drive') for w in plan['warnings'])
    result, error = rpc('planner.settings.set', {'drive': 'slope'})
    assert error is None and result['settings']['drive']['value'] == 'slope'
    spin, fit = result['settings']['spin_margin'], result['settings']['body_fit']
    assert spin['effective'] == 0.05 and 'drive slope' in spin['effective_why'] and fit['effective'] is True
    rpc('planner.plan', {'area_id': 'a'})
    assert stub_planner.config().drive == 'reverse'  # (the area's own)
    send_map(the_map({'drive': None}))
    rpc('planner.plan', {'area_id': 'a'})
    assert stub_planner.config().drive == 'slope'
    assert rpc('planner.settings.set', {'drive': 'backwards'})[1][1].startswith('drive must be one of')
    rpc('planner.settings.set', {'planner': 'slic3r'})
    result, _ = rpc('planner.settings')
    assert result['settings']['drive']['ignored'] == node_module.NO_BACKING_SLIC3R


# the worker and the progress (one planning at a time, the quick RPCs at once meanwhile)

def test_worker_most_urgent_first():
    """The worker's jobs one at a time, the most urgent first (of those the one asked for first); a job a job asks for
    runs at once, an exception goes to whoever waits."""
    worker = node_module.Worker()
    go, order = threading.Event(), []
    first = threading.Thread(target=worker.run, args=(node_module.AHEAD, go.wait, 30))
    first.start()
    time.sleep(0.2)
    waiting = []
    for urgency, name in ((node_module.AHEAD, 'ahead'), (node_module.ASKED, 'asked'), (node_module.MOWER, 'mower'),
                          (node_module.ASKED, 'asked 2'), (node_module.AFTER_MOW, 'after the mow')):
        waiting.append(threading.Thread(target=worker.run, args=(urgency, order.append, name)))
        waiting[-1].start()
        time.sleep(0.05)
    go.set()
    for t in [first] + waiting:
        t.join(30)
    assert order == ['mower', 'asked', 'asked 2', 'after the mow', 'ahead']
    assert worker.run(node_module.ASKED, lambda: worker.run(node_module.AHEAD, lambda: 7)) == 7
    with pytest.raises(ZeroDivisionError):
        worker.run(node_module.ASKED, lambda: 1 / 0)


def test_worker_joins_moves_up_and_supersedes():
    """The same job (same) asked for again joins the one waiting or running: planned once, the same result for each;
    asked for more urgently it's moved up; who waits with a slot for a job not begun is superseded by the next request
    with it (the job dropped where nobody waits for it any more); waiting(): in the order they'll run, with the RPC ids
    waiting for each."""
    worker = node_module.Worker()
    go, calls, got, threads = threading.Event(), [], {}, []

    def job(name):
        calls.append(name)
        return name

    def ask(key, urgency, fn, *args, **what):
        def asked():
            try:
                got[key] = worker.submit(urgency, fn, *args, **what)
            except node_module.Superseded as e:
                got[key] = e
        threads.append(threading.Thread(target=asked))
        threads[-1].start()
        time.sleep(0.1)

    ask('block', node_module.ASKED, go.wait, 30, same='block')
    ask('ahead a', node_module.AHEAD, job, 'a', area_id='a', same='a')
    ask('ahead b', node_module.AHEAD, job, 'b', area_id='b', same='b', rid='r1')
    ask('asked a', node_module.ASKED, job, 'a', area_id='a', same='a', rid='r2')
    ask('view 1', node_module.ASKED, job, 'c', area_id='c', same='c', rid='r3', slot='view')
    ask('view 2', node_module.ASKED, job, 'd', area_id='d', same='d', rid='r4', slot='view')
    ask('block 2', node_module.ASKED, job, 'x', same='block', rid='r5')
    assert worker.waiting() == (3, [{'area_id': 'a', 'job': 'plan', 'rank': 'asked', 'ids': ['r2']},
                                    {'area_id': 'd', 'job': 'plan', 'rank': 'asked', 'ids': ['r4']},
                                    {'area_id': 'b', 'job': 'plan', 'rank': 'ahead', 'ids': ['r1']}])
    assert isinstance(got['view 1'], node_module.Superseded)
    go.set()
    for t in threads:
        t.join(30)
    assert calls == ['a', 'd', 'b'] and worker.waiting() == (0, [])
    assert got['block'] == (True, False) and got['block 2'] == (True, True)
    assert got['ahead a'] == ('a', False) and got['asked a'] == ('a', True) and got['view 2'] == ('d', False)


def test_queue_over_rpc(planner, stub_planner, monkeypatch):
    """planner.plan's ahead and slot over RPC: the same request joins the one planned (kept), a request with a slot
    still waiting gets SUPERSEDED, the waiting ones in the progress (asked before ahead, their RPC ids), none at the
    end; ahead not true or false, a slot not a string: the caller's mistake."""
    send_map(the_map())
    go, began = threading.Event(), threading.Event()

    def slow(*args, **kw):
        began.set()
        assert go.wait(60)
        return stub_planner(*args, **kw)

    monkeypatch.setattr(node_module, 'plan_coverage', slow)
    rospy.published.clear()

    def send(rid, params):
        rospy.subscribers['/xbot/rpc/request'](node_module.RpcRequest(method='planner.plan', params=json.dumps(params),
                                                                      id=rid))
        time.sleep(0.1)

    small = [[0, 0], [6, 0], [6, 4], [0, 4]]
    send('p1', {'area_id': 'a'})
    assert began.wait(60)
    send('p2', {'outline': small, 'ahead': True})
    send('p3', {'outline': [[0, 0], [5, 0], [5, 4], [0, 4]], 'slot': 'view'})
    send('p4', {'outline': [[0, 0], [4, 0], [4, 4], [0, 4]], 'slot': 'view'})
    send('p5', {'area_id': 'a', 'ahead': True})
    time.sleep(1.5)
    waiting = [m['waiting'] for m in progress_messages() if m['running']][-1]
    assert [(w['ids'], w['rank'], w['job']) for w in waiting] == [(['p4'], 'asked', 'plan'), (['p2'], 'ahead', 'plan')]
    go.set()
    for t in threading.enumerate():
        if t.name == 'rpc planner.plan':
            t.join(60)
    got = {m.id: (topic, m) for topic, m in answers()}
    assert set(got) == {'p1', 'p2', 'p3', 'p4', 'p5'} and len(stub_planner.calls) == 3
    assert got['p3'][0] == '/xbot/rpc/error' and got['p3'][1].code == node_module.SUPERSEDED
    assert json.loads(got['p5'][1].result)['kept'] and not json.loads(got['p1'][1].result)['kept']
    time.sleep(1.5)  # (the last planning's of an outline: the queue told after it)
    assert progress_messages()[-1]['waiting_n'] == 0
    assert rpc('planner.plan', {'area_id': 'a', 'ahead': 'yes'})[1][0] == node_module.RpcError.ERROR_INVALID_PARAMS
    assert rpc('planner.plan', {'area_id': 'a', 'slot': 3})[1][0] == node_module.RpcError.ERROR_INVALID_PARAMS


def test_quick_rpcs_answer_while_it_plans(planner, stub_planner, monkeypatch):
    """While a planning runs, planner.settings and planner.log answer at once; planner.plan once it's planned."""
    send_map(the_map())
    go, began = threading.Event(), threading.Event()

    def slow(*args, **kw):
        began.set()
        assert go.wait(60)
        return stub_planner(*args, **kw)

    monkeypatch.setattr(node_module, 'plan_coverage', slow)
    rospy.published.clear()
    rospy.subscribers['/xbot/rpc/request'](node_module.RpcRequest(method='planner.plan',
                                                                  params=json.dumps({'area_id': 'a'}), id='plan'))
    assert began.wait(60)
    t0 = time.time()
    settings, error = rpc('planner.settings')
    assert error is None and 'turn_radius' in settings['settings']
    assert rpc('planner.log', {'limit': 1})[1] is None and time.time() - t0 < 5.0
    go.set()
    for t in threading.enumerate():
        if t.name == 'rpc planner.plan':
            t.join(60)
    assert 'plan' in [m.id for _, m in answers()]


def test_one_planning_at_a_time(planner, stub_planner, monkeypatch):
    """mower_logic's plan_path, planner.plan and a preview asked for at once: planned one after the other (two threads at
    one plan's shapes can crash GEOS), each gets its answer."""
    send_map(the_map())
    lock, now, most = threading.Lock(), [0], [0]

    def counted(*args, **kw):
        with lock:
            now[0] += 1
            most[0] = max(most[0], now[0])
        try:
            time.sleep(0.2)
            return stub_planner(*args, **kw)
        finally:
            with lock:
                now[0] -= 1

    monkeypatch.setattr(node_module, 'plan_coverage', counted)
    got = {}
    asks = {'plan_path': lambda: rospy.services['/slic3r_coverage_planner/plan_path'](request()),
            'plan': lambda: planner.methods['planner.plan']({'area_id': 'a', 'settings': {'turn_radius': 0.5}}),
            'preview': lambda: planner.methods['planner.plan']({'outline': [[0, 0], [6, 0], [6, 4], [0, 4]]})}
    threads = [threading.Thread(target=lambda n=n, f=f: got.update({n: f()})) for n, f in asks.items()]
    for t in threads:
        t.start()
    for t in threads:
        t.join(120)
    assert most[0] == 1 and len(stub_planner.calls) == 3 and set(got) == set(asks) and got['plan_path'].paths


def progress_messages():
    return [json.loads(m.payload) for topic, m in rospy.published if topic == '/xbot_monitoring/mqtt_publish'
            and m.topic == node_module.PROGRESS_TOPIC and m.retain]


def test_progress(planner, stub_planner, monkeypatch):
    """How far a planning got on mowbite/planner/progress (retained, through xbot_monitoring): from its start to done,
    done growing, at most one message a second and one more at its end, the next planning of the area with what this
    one took as estimate_s; a plan kept from before none."""
    send_map(the_map())

    def slow(*args, **kw):
        time.sleep(2.6)
        return stub_planner(*args, **kw)

    monkeypatch.setattr(node_module, 'plan_coverage', slow)
    rospy.published.clear()
    result, error = rpc('planner.plan', {'area_id': 'a'})
    seen = progress_messages()
    assert error is None and len(seen) >= 3
    first, last = seen[0], seen[-1]
    assert set(last) == {'area_id', 'key', 'running', 'stage', 'done', 'of', 'started', 'elapsed_s', 'estimate_s',
                         'error', 'waiting_n', 'waiting', 'planned_on', 'remote_why'}
    assert last['planned_on'] == 'mower' and last['remote_why'] is None
    assert last['waiting_n'] == 0 and last['waiting'] == []
    assert first['area_id'] == 'a' and first['running'] and first['stage'] == 'angles' and first['estimate_s'] is None
    assert last == dict(last, area_id='a', key=first['key'], running=False, stage='done', done=last['of'], error=None)
    assert [m['done'] for m in seen] == sorted(m['done'] for m in seen) and seen[-2]['done'] > 0
    times = [m['started'] + m['elapsed_s'] for m in seen[:-1]]
    assert all(b - a >= 0.9 for a, b in zip(times, times[1:]))
    rospy.published.clear()
    assert rpc('planner.plan', {'area_id': 'a'})[0]['kept'] and not progress_messages()
    rpc('planner.plan', {'area_id': 'a', 'settings': {'turn_radius': 0.5}})
    assert 2.0 < progress_messages()[0]['estimate_s'] < 10.0


def test_progress_of_a_failed_planning(planner, monkeypatch):
    """A planning that fails: its last progress message says why."""
    send_map(the_map())

    def failing(*args, **kw):
        raise node_module.PlanningError('no lanes fit')

    monkeypatch.setattr(node_module, 'plan_coverage', failing)
    rospy.published.clear()
    assert rpc('planner.plan', {'area_id': 'a'})[1][1] == 'no lanes fit'
    last = progress_messages()[-1]
    assert last['running'] is False and last['error'] == 'no lanes fit' and last['stage'] != 'done'


def test_a_no_kept(planner, monkeypatch):
    """The planner's no to a request is kept: the same request again gets it at once (an area too small took a minute
    each time on the mower), another is planned; a crash isn't kept."""
    send_map(the_map())
    calls = []

    def failing(*args, **kw):
        calls.append(1)
        raise node_module.NothingLeft('nothing left')

    monkeypatch.setattr(node_module, 'plan_coverage', failing)
    for _ in range(3):
        assert rpc('planner.plan', {'area_id': 'a'})[1][1] == 'nothing left'
    for _ in range(2):
        assert not poses(planner.handle(request(angle=0.3)))
    assert calls == [1, 1]

    def crashing(*args, **kw):
        calls.append(1)
        raise RuntimeError('out of memory')

    monkeypatch.setattr(node_module, 'plan_coverage', crashing)
    for _ in range(2):
        assert not poses(planner.handle(request(angle=0.5)))
    assert len(calls) == 4


class HoldClient:
    """paho-mqtt's client as HoldReader uses it, its callbacks called by the test."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        return lambda *a: self.calls.append((name,) + a)


def code(failure=False):
    return types.SimpleNamespace(is_failure=failure)


def test_hold():
    """mowbite/hold: held while the MowBite Updater's hold is still to end, at most 30 minutes away (a later one is
    none of its own); held too when it can't tell: not connected, no answer yet after subscribing, a payload it can't
    read. None, an empty one or one that ended: not held."""
    client = HoldClient()
    hold = node_module.HoldReader(client)
    assert ('connect_async', '127.0.0.1', 1883) in client.calls and ('loop_start',) in client.calls
    now = time.time()
    assert hold.held(now) == 'not connected yet'
    hold.on_connect(client, None, {}, code(True))
    assert hold.held(now).startswith('connection refused')
    hold.on_connect(client, None, {}, code())
    assert client.calls[-1] == ('subscribe', 'mowbite/hold', 1)
    assert hold.held(now) == 'not subscribed yet'
    hold.on_subscribe(client, None, 1, [code(True)])
    assert hold.held(now).startswith('subscription refused')
    hold.on_subscribe(client, None, 1, [code()])
    # the retained hold comes right after the subscription: none within HOLD_SETTLE s, there is none
    now = time.time()
    assert hold.held(now) == 'no answer from the broker yet'
    assert hold.held(now + node_module.HOLD_SETTLE) == ''

    def payload(data):
        hold.on_message(client, None, types.SimpleNamespace(payload=data.encode()))
        return hold.held(now)

    assert payload('{"until": %d}' % (now + 600)).startswith('held by the MowBite Updater until ')
    assert payload('{"until": %.1f}' % (now + 1799)) != ''
    assert payload('{"until": %d}' % (now - 1)) == ''  # ended
    assert payload('{"until": %d}' % (now + 1801)) == ''  # too far off
    assert payload('') == ''  # taken away
    for broken in ('{"until": "%d"}' % (now + 600), '{"until": true}', '{"to": 1}', '[1]', 'off', '{"until": NaN}',
                   '\xff'):
        assert payload(broken).startswith('unreadable mowbite/hold'), broken
    payload('{"until": %d}' % (now + 600))
    hold.on_disconnect(client, None, {}, code(True))
    assert hold.held(now) == 'no connection to 127.0.0.1:1883'
    # connected again: what it had before doesn't count, the retained hold comes again
    hold.on_connect(client, None, {}, code())
    hold.on_subscribe(client, None, 2, [code()])
    assert hold.held(time.time() + node_module.HOLD_SETTLE) == ''


def test_hold_without_paho(monkeypatch):
    monkeypatch.setitem(sys.modules, 'paho', None)
    rospy.log.clear()
    assert node_module.HoldReader().held() == 'no paho-mqtt'
    assert any(level == 'error' and 'paho-mqtt' in line for level, line in rospy.log)


def test_leftovers_auto_held(planner, monkeypatch):
    """leftovers_auto sends the mower out only once the MowBite Updater's hold ended, not at all while it lasts
    longer than the leftovers stay armed."""
    monkeypatch.setattr(node_module, 'READY_POLL', 0.01)
    rospy.messages['/mower_logic/current_state'] = node_module.HighLevelStatus(state_name='IDLE', is_charging=True)
    client = HoldClient()
    planner.hold = node_module.HoldReader(client)
    planner.hold.on_connect(client, None, {}, code())
    planner.hold.on_subscribe(client, None, 1, [code()])
    planner.hold.subscribed -= node_module.HOLD_SETTLE

    def run(hold_s, armed_s):
        planner.hold.on_message(client, None, types.SimpleNamespace(
            payload=json.dumps({'until': time.time() + hold_s}).encode()))
        armed = {'until': time.time() + armed_s, 'served': False,
                 'plans': {'a': types.SimpleNamespace(stats={'leftovers': {'area': 1.0}})}}
        monkeypatch.setattr(planner, 'rpc_leftovers', lambda params: setattr(planner, 'armed', armed))
        rospy.log.clear()
        rospy.published.clear()
        planner.leftovers_after('job')
        return [m.data for topic, m in rospy.published if topic == '/xbot/action']

    assert run(0.3, 5.0) == ['mower_logic:idle/start_mowing']
    assert any('not sent out for the leftovers yet, held: held by the MowBite Updater' in line
               for _, line in rospy.log)
    assert run(600.0, 0.3) == []
    assert any('not sent out for the leftovers, held (held by the MowBite Updater' in line for _, line in rospy.log)
    assert run(-1.0, 5.0) == ['mower_logic:idle/start_mowing']
    assert not any('held' in line for _, line in rospy.log)


# planner.calib.*: OpenMower's parameters the recording takes as old
CALIB = {'/ll/services/diff_drive/ticks_per_m': 2025.0, '/ll/services/diff_drive/wheel_distance_m': 0.3556,
         '/xbot_positioning/antenna_offset_x': -0.1, '/xbot_positioning/antenna_offset_y': 0.0}
CALIB_RESULT = {'ready': True, 'version': 'mowbite-calib/1', 'based_on': {'m': 42.0, 'turns': 6.1, 'runs': 7},
                'warnings': [], 'values': [], 'info': {}}


def gps_message(fixed=True, x=1.0, y=2.0):
    m = node_module.AbsolutePose(flags=2 if fixed else 4, position_accuracy=0.014, motion_vector_valid=1)
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    return m


def calib_ready(monkeypatch, result=None):
    """OpenMower's topics and parameters there, the steps by hand, the evaluation result (a stand-in) where given."""
    rospy.params.update(CALIB)
    imu = node_module.Imu()
    imu.header.frame_id = 'base_link'
    rospy.messages.update({node_module.CALIB_GPS: gps_message(), node_module.CALIB_IMU: imu,
                           node_module.CALIB_TWIST: node_module.TwistStamped()})
    monkeypatch.setattr(node_module, 'CALIB_TICK', 3600.0)
    asked = []
    if result is not None:
        monkeypatch.setattr(node_module, 'run_calib', lambda path, interim=False: asked.append((path, interim)) or
                            dict(result))
    return asked


def calib_progress():
    return [json.loads(m.payload) for topic, m in rospy.published
            if topic == '/xbot_monitoring/mqtt_publish' and m.topic == node_module.CALIB_TOPIC]


def calib_threads(name):
    for t in threading.enumerate():
        if t.name == name:
            t.join(120.0)


def test_calib_refused(planner, monkeypatch):
    """planner.calib.start says why it doesn't record: the mode, a topic without messages, no RTK fixed, a parameter
    not set, too little disk; result and stop without one."""
    assert rpc('planner.calib.start', {'mode': 'walk'})[1][1] == 'mode must be one of joystick, mow'
    assert rpc('planner.calib.start', {'mode': 'joystick'})[1][1] == 'nothing on /ll/position/gps'
    calib_ready(monkeypatch)
    rospy.messages[node_module.CALIB_GPS] = gps_message(fixed=False)
    assert rpc('planner.calib.start', {'mode': 'joystick'})[1][1] == 'no RTK fixed'
    rospy.messages[node_module.CALIB_GPS] = gps_message()
    del rospy.params['/xbot_positioning/antenna_offset_y']
    assert rpc('planner.calib.start', {'mode': 'mow'})[1][1] == \
        "OpenMower's parameter /xbot_positioning/antenna_offset_y isn't set"
    rospy.params.update(CALIB)
    monkeypatch.setattr(node_module.calibrecord, 'FREE', 1 << 60)
    assert rpc('planner.calib.start', {'mode': 'mow'})[1][1].startswith('only ')
    assert rpc('planner.calib.result')[1][1] == 'no calibration recorded'
    assert rpc('planner.calib.stop')[1][1] == 'no calibration is recorded'
    assert not planner.calib.recorder.recording and not calib_progress()


def test_calib_records_and_evaluates(planner, monkeypatch):
    """Recorded what comes on OpenMower's topics (the old values its parameters), stopped, evaluated (in a process of
    its own, here a stand-in), the result kept; the progress all along."""
    asked = calib_ready(monkeypatch, CALIB_RESULT)
    assert rpc('planner.calib.start', {'mode': 'joystick'}) == ({'ok': True}, None)
    counted = {'m': 0.0, 'turns': 0.0, 'runs': 0, 'still_s': 0.0, 'turns_left': 0, 'turns_right': 0,
               'rotation_left': 0.0, 'rotation_right': 0.0, 'tight_left': 0.0, 'tight_right': 0.0}
    need = node_module.calibrecord.needed('joystick')
    assert calib_progress()[-1] == dict(counted, phase='recording', ready=False, error=None, mode='joystick',
                                        need=need)
    assert rpc('planner.calib.start', {'mode': 'joystick'})[1][1] == 'a calibration is recorded already'
    assert rpc('planner.calib.result')[1][1] == 'the calibration is still recorded'
    imu = node_module.Imu()
    imu.angular_velocity.z, imu.linear_acceleration.z = 0.01, 9.81
    twist, cmd = node_module.TwistStamped(), node_module.Twist()
    twist.twist.linear.x = cmd.linear.x = 0.2
    rospy.subscribers[node_module.CALIB_GPS](gps_message(x=3.0))
    rospy.subscribers[node_module.CALIB_IMU](imu)
    rospy.subscribers[node_module.CALIB_TWIST](twist)
    rospy.subscribers[node_module.CALIB_CMD](cmd)
    planner.calib.step(time.time())
    assert rpc('planner.calib.stop') == ({'ok': True}, None)
    assert node_module.CALIB_GPS not in rospy.subscribers  # (no longer)
    calib_threads('calibration evaluation')
    path = planner.calib.recorder.path
    assert asked == [(path, False)]
    rec = calibread.read(path)
    assert rec.mode == 'joystick' and rec.source == 'openmower' and rec.imu_frame == 'base_link'
    assert rec.params == {'ticks_per_m': 2025.0, 'wheel_distance_m': 0.3556, 'antenna_offset_x': -0.1,
                          'antenna_offset_y': 0.0}
    assert list(rec.columns['gps']['x']) == [3.0] and list(rec.columns['imu']['az']) == [9.81]
    assert list(rec.columns['twist']['v']) == [0.2] and list(rec.columns['cmd']['v']) == [0.2]
    assert rec.end['why'] == 'stop'
    # (m, turns and runs then the evaluation's)
    assert calib_progress()[-1] == dict(counted, m=42.0, turns=6.1, runs=7, phase='done', ready=True, error=None,
                                        mode='joystick', need=need)
    assert rpc('planner.calib.result') == (CALIB_RESULT, None)


def test_calib_mow_ends_with_the_mow(planner, monkeypatch):
    """Mode mow: the mow finishing (JOB_COMPLETE) ends it; an evaluation that fails says why."""
    calib_ready(monkeypatch, {'error': 'no header: the recording is empty'})
    assert rpc('planner.calib.start', {'mode': 'mow'}) == ({'ok': True}, None)
    event('JOB_COMPLETE')
    calib_threads('calibration evaluation')
    assert not planner.calib.recorder.recording
    assert calibread.read(planner.calib.recorder.path).end['why'] == 'stop'
    assert calib_progress()[-1]['phase'] == 'done' and calib_progress()[-1]['mode'] == 'mow'
    assert calib_progress()[-1]['error'] == 'no header: the recording is empty'
    assert rpc('planner.calib.result')[1][1] == 'no header: the recording is empty'


def test_calib_ends_by_itself(planner, monkeypatch):
    """With enough runs and turns it's evaluated as it stands, ready: it ends (enough); at its time limit (time)."""
    asked = calib_ready(monkeypatch, CALIB_RESULT)
    monkeypatch.setattr(node_module.calibrecord.Counts, 'enough', lambda self: True)
    assert rpc('planner.calib.start', {'mode': 'joystick'}) == ({'ok': True}, None)
    calib = planner.calib
    calib.step(time.time())
    assert asked == []  # (not before CALIB_CHECK s)
    calib.checked = 0.0
    calib.step(time.time())
    calib_threads('calibration check')
    assert asked == [(calib.recorder.path, True)] and calib.enough
    calib.step(time.time())
    calib_threads('calibration evaluation')
    assert calibread.read(calib.recorder.path).end['why'] == 'enough'
    monkeypatch.setitem(node_module.calibrecord.LIMIT_S, 'joystick', 0.0)
    assert rpc('planner.calib.start', {'mode': 'joystick'}) == ({'ok': True}, None)
    calib.step(time.time())
    calib_threads('calibration evaluation')
    assert calibread.read(calib.recorder.path).end['why'] == 'time'
    # (a check of a recording ended since: it doesn't end the next one)
    assert rpc('planner.calib.start', {'mode': 'joystick'}) == ({'ok': True}, None)
    calib.check(calib.recorder.started - 1.0)
    assert not calib.enough and not calib.ready and calib.recorder.recording
    assert rpc('planner.calib.stop') == ({'ok': True}, None)
    calib_threads('calibration evaluation')


def test_calib_evaluated_after_a_restart(planner):
    """A recording without its result (the node restarted while it recorded) is evaluated as the node starts, for
    real (python3 -m mowbite_planner.calib)."""
    folder = os.path.join(os.environ['ROS_HOME'], 'mowbite_calib')
    os.makedirs(folder)
    calibsim.joystick(0).recording(os.path.join(folder, 'recording.jsonl.gz'), end=None)
    NODES[:] = [node_module.MowbitePlannerRos1()]
    calib_threads('calibration evaluation')
    result, error = rpc('planner.calib.result')
    assert error is None and result['ready'] and result['based_on']['runs'] == 7
    assert any(w.startswith('the recording stops in the middle') for w in result['warnings'])
    assert [v['name'] for v in result['values'] if v['ok']] == ['ticks_per_m', 'wheel_distance_m',
                                                                 'antenna_offset_x', 'antenna_offset_y']


# remote_planning ---------------------------------------------------------------------------------------------------

class Broker:
    """A broker for the tests: what a client publishes goes to whoever subscribed to its topic (+: one level), in order
    on a thread of the broker's (like paho's network loop); the retained messages come to a subscription at once."""

    def __init__(self):
        self.subs, self.retained = [], {}
        self.queue = queue.Queue()
        threading.Thread(target=self.deliver, daemon=True).start()

    @staticmethod
    def matches(pattern, topic):
        a, b = pattern.split('/'), topic.split('/')
        return len(a) == len(b) and all(x in ('+', y) for x, y in zip(a, b))

    def publish(self, topic, payload, retain=False):
        payload = payload.encode() if isinstance(payload, str) else bytes(payload)
        if retain:
            self.retained[topic] = payload
        for client, pattern in list(self.subs):
            if self.matches(pattern, topic):
                self.queue.put((client, topic, payload, False))

    def subscribe(self, client, pattern):
        self.subs.append((client, pattern))
        for topic, payload in self.retained.items():
            if payload and self.matches(pattern, topic):
                self.queue.put((client, topic, payload, True))

    def deliver(self):
        while True:
            client, topic, payload, retain = self.queue.get()
            client.on_message(client, None, types.SimpleNamespace(topic=topic, payload=payload, retain=retain))
            self.queue.task_done()


class Client:
    """paho-mqtt's client as RemotePlanning uses it, on a Broker."""

    def __init__(self, broker, on_message=None):
        self.broker, self.on_message = broker, on_message

    def reconnect_delay_set(self, *a):
        pass

    def connect_async(self, *a):
        pass

    def loop_start(self):
        self.on_connect(self, None, {}, code())

    def subscribe(self, topics, qos=None):
        for topic in [topics] if isinstance(topics, str) else [t for t, _ in topics]:
            self.broker.subscribe(self, topic)

    def publish(self, topic, payload, qos=0, retain=False):
        self.broker.publish(topic, payload, retain)


SMALL = [(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)]
SMALL_MAP = {'areas': [area('s', SMALL + [SMALL[0]], name='Small', type='mow')]}


def a_house(broker, answer=None):
    """A house computer on broker (mowbite_planner.house, planning on threads of its own; answer(house, request):
    answers in its place), stopped by the event it gives."""
    from mowbite_planner.house import House
    house = House(lambda topic, payload, retain: broker.publish(topic, payload, retain))
    got = (lambda raw: answer(house, json.loads(raw))) if answer else house.on_request
    broker.subscribe(Client(broker, lambda client, userdata, msg: got(msg.payload)), node_module.TOPIC + 'request')
    stop = threading.Event()
    threading.Thread(target=house.work, daemon=True).start()

    def tick():
        while not stop.wait(0.2):
            house.tick()

    threading.Thread(target=tick, daemon=True).start()
    house.hello()
    return house, stop


def with_house(planner, broker):
    planner.remote = node_module.RemotePlanning(planner.code, Client(broker))
    broker.queue.join()


@pytest.mark.real_planner
def test_remote_planning(planner):
    """remote_planning on: the plan made on the house computer, the same as here to the bit, taken where the mower's
    own geometry and body find nothing wrong with it; planned_on says where (planner.plan, the progress), kept with
    that in memory and on disk. planner.settings tells the house computer's state."""
    broker = Broker()
    house, stop = a_house(broker)
    try:
        with_house(planner, broker)
        assert json.loads(broker.retained[node_module.TOPIC + 'want']) == {'code': planner.code,
                                                                           'stack': house.stack}
        result, error = rpc('planner.settings.set', {'remote_planning': True})
        assert error is None and result['settings']['remote_planning'] == {
            'value': True, 'default': False, 'stored': True, 'type': 'boolean', 'settable': True}
        assert result['remote'] == {'reachable': True, 'arch': house.arch, 'code_match': True, 'stack_match': True,
                                    'why': None}
        send_map(SMALL_MAP)
        rospy.published.clear()
        got, error = rpc('planner.plan', {'area_id': 's'})
        assert error is None and got['planned_on'] == 'house' and got['kept'] is False
        seen = progress_messages()
        assert seen[-1]['planned_on'] == 'house' and seen[-1]['remote_why'] is None and seen[-1]['stage'] == 'done'
        again, _ = rpc('planner.plan', {'area_id': 's'})
        assert again['kept'] and again['planned_on'] == 'house' and again['fingerprint'] == got['fingerprint']
        restarted = node_module.MowbitePlannerRos1()  # (the RPCs go to it from now on)
        send_map(SMALL_MAP)
        kept = restarted.rpc_plan({'area_id': 's'})
        assert kept['kept'] and kept['planned_on'] == 'house' and kept['fingerprint'] == got['fingerprint']
        # here: the same plan
        rpc('planner.settings.set', {'remote_planning': False})
        restarted.plans.clear()
        shutil.rmtree(restarted.plans_dir)
        here, _ = rpc('planner.plan', {'area_id': 's'})
        assert here['planned_on'] == 'mower' and here['fingerprint'] == got['fingerprint']
    finally:
        stop.set()


def test_remote_planning_falls_back(planner, monkeypatch):
    """Planned here where the house computer isn't there, plans with other code, sends a plan broken or one the
    checks here refuse, or says nothing for REMOTE_STALL s (then not asked for REMOTE_QUIET s); the planner's no there
    is the no here. Why: planner.settings (ignored), the log (once) and the progress (remote_why)."""
    monkeypatch.setattr(node_module, 'REMOTE_CATCH_UP', 0.0)
    monkeypatch.setattr(node_module, 'REMOTE_SETTLE', 0.0)
    monkeypatch.setattr(node_module, 'REMOTE_STALL', 0.3)
    broker = Broker()
    with_house(planner, broker)
    result, _ = rpc('planner.settings.set', {'remote_planning': True})
    assert result['settings']['remote_planning']['ignored'] == 'no house computer'
    assert result['remote'] == {'reachable': False, 'arch': None, 'code_match': False, 'stack_match': False,
                                'why': 'no house computer'}
    send_map(the_map())

    def planned(**settings):
        rospy.published.clear()
        got, error = rpc('planner.plan', {'area_id': 'a', 'settings': settings})
        assert error is None
        return got['planned_on'], progress_messages()[-1]['remote_why']

    assert planned() == ('mower', 'no house computer')
    old = dict(json.loads(broker.retained.get(node_module.TOPIC + 'want')), code='other', at=time.time())
    broker.publish(node_module.TOPIC + 'hello', json.dumps(old), retain=True)
    broker.queue.join()
    assert rpc('planner.settings')[0]['settings']['remote_planning']['ignored'] == 'house computer out of date'
    assert planned(turn_radius=0.3) == ('mower', 'house computer out of date')
    assert sum('not on the house computer' in line for _, line in rospy.log) == 2

    def answering(how):
        def answer(house, request):
            rid, key = request['id'], request['key']

            def say(**more):
                broker.publish(node_module.TOPIC + 'answer/' + rid, json.dumps(dict(id=rid, key=key, code=house.code,
                                                                                    **more)))

            if how == 'broken':
                say(ok=True, parts=1, bytes=3, sha256='0' * 64)
                broker.publish(node_module.TOPIC + 'answer/%s/0' % rid, b'abc')
            elif how == 'no':
                say(ok=False, error='the area is too small', refused=True)
            elif how == 'wrong':  # (a plan of another area, beside this one: refused by the checks here)
                plan = node_module.plan_coverage([(x + 30.0, y) for x, y in SMALL], [],
                                                 node_module.PlannerConfig(mower_width=0.18))
                from mowbite_planner.remote import pack, plan_json
                data = pack(dict(plan_json(plan, plan.config), spacing=None))
                say(ok=True, parts=1, bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
                broker.publish(node_module.TOPIC + 'answer/%s/0' % rid, data)
        return answer

    for how, why in (('broken', 'its answer is broken'), ('wrong', 'its plan')):
        broker = Broker()
        a_house(broker, answering(how))
        with_house(planner, broker)
        on, said = planned(turn_radius=0.3 + len(how) / 100.0)
        assert on == 'mower' and said.startswith(why), said
    broker = Broker()
    a_house(broker, answering('no'))
    with_house(planner, broker)
    for _ in range(2):
        assert rpc('planner.plan', {'area_id': 'a', 'settings': {'turn_radius': 0.4}})[1][1] == 'the area is too small'
    broker = Broker()
    a_house(broker, lambda house, request: None)  # (says nothing)
    with_house(planner, broker)
    assert planned(turn_radius=0.5) == ('mower', 'no word from it for 0 s')
    t0 = time.time()
    on, said = planned(turn_radius=0.6)
    assert on == 'mower' and said.startswith('it didn\'t answer, asked again from ') and time.time() - t0 < 5.0


def test_remote_answers_of_others_dropped(planner):
    """Only what comes for the request waiting counts: progress and answers of another id, a header of another key
    or code or not readable, a part too big are dropped (no word from the house: REMOTE_STALL s later it's planned
    here)."""
    broker = Broker()
    remote = node_module.RemotePlanning('c' * 40, Client(broker))
    asked = {'id': 'a' * 32, 'key': 'k' * 40, 'heard': 0.0, 'answer': None, 'parts': {}, 'step': lambda *a: None}
    remote.open = asked

    def message(topic, payload):
        remote.on_message(None, None, types.SimpleNamespace(
            topic=node_module.TOPIC + topic, payload=payload.encode() if isinstance(payload, str) else payload,
            retain=False))

    message('answer/' + 'b' * 32, json.dumps({'key': 'k' * 40, 'code': 'c' * 40, 'ok': False, 'error': 'x'}))
    message('progress/' + 'b' * 32, json.dumps({'stage': 'route', 'done': 1, 'of': 2}))
    assert asked['answer'] is None and asked['heard'] == 0.0
    message('answer/%s/0' % ('a' * 32), b'x' * (node_module.PART + 1))
    for header in ({'key': 'other', 'code': 'c' * 40, 'ok': True}, {'key': 'k' * 40, 'code': 'other', 'ok': True},
                   {'key': 'k' * 40, 'code': 'c' * 40, 'ok': True, 'parts': 99, 'bytes': 3, 'sha256': '0' * 64},
                   'not json'):
        message('answer/' + 'a' * 32, header if isinstance(header, str) else json.dumps(header))
    assert asked['answer'] is None and asked['parts'] == {} and asked['heard'] == 0.0
    message('progress/' + 'a' * 32, json.dumps({'stage': 'route', 'done': 1, 'of': 2}))
    assert asked['heard'] > 0.0
    # a hello retained long ago: no house; one sent now: there
    message('hello', json.dumps({'code': 'c' * 40, 'stack': remote.stack, 'at': time.time()}))
    assert remote.state()[0] == ''
    remote.on_message(None, None, types.SimpleNamespace(topic=node_module.TOPIC + 'hello', retain=True, payload=(
        json.dumps({'code': 'c' * 40, 'stack': remote.stack, 'at': time.time() - 3600}).encode())))
    assert remote.state()[0] == 'no house computer'
    message('hello', b'')
    assert remote.state()[0] == 'no house computer'


def test_the_jobs_plans_kept(planner, monkeypatch):
    """The plans mower_logic got are kept (in memory, in plans_dir, over a restart) till the job ends (JOB_COMPLETE,
    JOB_RESET), however many are planned meanwhile; then they go like any other."""
    monkeypatch.setattr(node_module, 'CACHED_PLANS', 2)
    monkeypatch.setattr(node_module, 'DISK_PLANS', 2)
    send_map(the_map())
    planner.handle(request())
    key, = planner.pinned

    def others(*angles):
        for angle in angles:
            planner.plan_coverage(SMALL, [], planner.current.base.with_overrides({'angle': angle}))

    others(0.1, 0.2, 0.3)
    assert key in planner.plans and os.path.exists(planner.plan_file(key))
    event('JOB_COMPLETE')
    others(0.4, 0.5)
    assert key not in planner.plans and not os.path.exists(planner.plan_file(key)) and planner.pinned == set()
    planner.handle(request())
    restarted = node_module.MowbitePlannerRos1()  # (the events go to it from now on)
    assert restarted.pinned == {key}
    event('JOB_RESET')
    assert restarted.pinned == set() and node_module.MowbitePlannerRos1().pinned == set()
