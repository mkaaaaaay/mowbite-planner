"""Tests of the ROS 1 node without ROS: rospy and the message packages are stood in for here.

    python3 -m pytest ros1/mowbite_planner_ros1/test   (shapely is enough)
"""

import importlib.machinery
import importlib.util
import json
import math
import os
import re
import shutil
import struct
import sys
import tempfile
import time
import threading
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
# the node keeps the settings changed over RPC in $ROS_HOME, not in the real one
os.environ['ROS_HOME'] = tempfile.mkdtemp()
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'ros2', 'mowbite_planner'))


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
    rpc_error = msg('RpcError', code=int, message=str, id=str)
    rpc_error.ERROR_INVALID_PARAMS, rpc_error.ERROR_INTERNAL = -32602, -32603

    class Rospy(types.ModuleType):
        params = {}
        log = []
        services = {}
        subscribers = {}
        published = []
        registered = []

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

        def Publisher(self, topic, kind, **kwargs):
            return types.SimpleNamespace(publish=lambda m: self.published.append((topic, m)))

        def ServiceProxy(self, name, kind):
            return lambda *a: self.registered.append(a)

        def wait_for_service(self, name, timeout=None):
            pass

        def Service(self, name, kind, handler):
            self.services[name] = handler
            return types.SimpleNamespace(resolved_name=name)

        def loginfo(self, *a):
            self.log.append(('info', a[0] % a[1:] if len(a) > 1 else a[0]))

        def logwarn(self, *a):
            self.log.append(('warn', a[0] % a[1:] if len(a) > 1 else a[0]))

        def logerr(self, *a):
            self.log.append(('error', a[0] % a[1:] if len(a) > 1 else a[0]))

        def logwarn_throttle(self, period, *a):
            self.logwarn(*a)

    rospy = Rospy('rospy')
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
                                                   Quaternion=quat, Polygon=polygon),
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
    for name in ('mowbite_planner_settings.json', 'mowbite_planner_timing.json'):
        try:
            os.remove(os.path.join(os.environ['ROS_HOME'], name))
        except FileNotFoundError:
            pass
    # no plans kept from other tests, none planned ahead in the background (test_plan_ahead does that itself)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'mowbite_plans'), ignore_errors=True)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'position_history'), ignore_errors=True)
    node_module.PLAN_AHEAD_DELAY = 1e6
    return node_module.MowbitePlannerRos1()


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
    assert any('2 parts with no way between them' in line for _, line in rospy.log)


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
    extra area properties (open_mower_ros #364), before it they were dropped."""
    plain = planner.handle(request())
    pond = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    send_map(through_mower_map(the_map({'fill_pattern': 'crosshatch'}, angle_min=0.2, angle_max=0.4, areas=[
        area('pond', pond, type='mow', mowable=False, mow_around=True)]), keeps_all))
    res = planner.handle(request(angle=1.0))
    check(res)
    assert (len(poses(res)) > 1.6 * len(poses(plain))) == keeps_all  # crosshatch
    assert (not inside(res, pond)) == keeps_all  # mowed around
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
    ({}, {'mow_angle_offset': 10.0}, {}, (POND,), 0.1),
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


# OpenMower RPCs (planner.settings, planner.settings.set, planner.plan)

def rpc(method, params=None, rid='1'):
    """Hands the node a request like xbot_monitoring does, gives (result, None) or (None, (code, message))."""
    rospy.published.clear()
    raw = '' if params is None else params if isinstance(params, str) else json.dumps(params)
    rospy.subscribers['/xbot/rpc/request'](node_module.RpcRequest(method=method, params=raw, id=rid))
    assert len(rospy.published) <= 1
    if not rospy.published:
        return None, None
    topic, m = rospy.published[0]
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
    methods = ['planner.angle.reset', 'planner.drive', 'planner.plan', 'planner.settings', 'planner.settings.set']
    assert ('mowbite_planner', methods) in rospy.registered
    rospy.registered.clear()
    rospy.subscribers['/xbot/rpc/registry'](None)  # xbot_monitoring started again
    assert rospy.registered == [('mowbite_planner', methods)]


def test_rpc_settings(planner):
    assert rpc('planner.settings', 'null') == rpc('planner.settings', {})
    result, error = rpc('planner.settings')
    assert error is None and result['file'] == stored_file() and result['own_angle'] is False
    s = result['settings']
    assert set(s) == set(node_module.PlannerConfig.names()) | {'planner'}
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
    assert s['transit_edge_cost']['unit'] == ''
    for name, value in (('headland_turns', False), ('turn_on_spot', False), ('body_fit', True)):
        assert s[name] == {'value': value, 'default': value, 'stored': False, 'type': 'boolean', 'settable': True}
    assert s['headland_corners'] == {'value': 'rounded', 'default': 'rounded', 'stored': False, 'type': 'string',
                                     'settable': True, 'choices': ['rounded', 'sharp']}
    for name, value in (('edge_margin', 0.05), ('obstacle_margin', 0.1), ('spin_margin', 0.05)):
        assert s[name] == {'value': value, 'default': value, 'stored': False, 'type': 'number', 'settable': True,
                           'unit': 'm', 'min': 0.0, 'max': 1.0}, name


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


def test_rpc_settings_robot_outline(planner):
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
    assert cut['fingerprint'] != rectangle['fingerprint'] and not cut['kept']
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


def test_no_settings_file(planner):
    rospy.params = {'~settings_file': ''}
    node = node_module.MowbitePlannerRos1()
    result, _ = rpc('planner.settings.set', {'turn_radius': 0.4})
    assert result['file'] is None and node.base.turn_radius == 0.4 and not os.path.exists(stored_file())


def test_rpc_plan(planner):
    send_map(the_map())
    rospy.params['/mower_logic'] = {'tool_width': 0.13, 'outline_count': 2, 'mow_angle_offset': 0.0}
    result, error = rpc('planner.plan', {'area_id': 'a'})
    assert error is None and result['area_id'] == 'a' and result['paths']
    assert result['lane_spacing'] == pytest.approx(0.13)
    # trying a setting gives what setting it gives
    tried, _ = rpc('planner.plan', {'area_id': 'a', 'settings': {'turn_radius': 0.6}})
    assert tried['fingerprint'] != result['fingerprint']
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
    assert result['fingerprint'] != checked['fingerprint']
    # the planner picks the lane spacing, chosen says which, without report too
    auto, error = rpc('planner.plan', {'area_id': 'a', 'settings': {'lane_spacing_mode': 'auto'}})
    assert error is None and auto['stats']['chosen']['mode'] == 'auto' and result['stats']['chosen']['mode'] == 'fixed'


@pytest.mark.parametrize('method, params, code, message', [
    ('planner.plan', {'area_id': 'nope'}, -32602, 'nope'),
    ('planner.plan', '[1, 2]', -32602, 'params must be an object'),
    ('planner.plan', 'not json', -32602, 'params must be an object'),
    ('planner.settings.set', {'turn_radius': 'x'}, -32602, 'turn_radius'),
])
def test_rpc_errors(planner, method, params, code, message):
    send_map(the_map())
    result, error = rpc(method, params)
    assert result is None and error[0] == code and message in error[1], error


def test_rpc_plan_without_map(planner):
    result, error = rpc('planner.plan', {'area_id': 'a'})
    assert error[0] == -32602 and 'no map' in error[1]


def test_rpc_internal_error_and_others(planner, monkeypatch):
    monkeypatch.setitem(planner.rpc.methods, 'planner.settings', lambda p: 1 / 0)
    result, error = rpc('planner.settings')
    assert error[0] == -32603 and 'division by zero' in error[1]
    assert any(level == 'error' for level, _ in rospy.log)
    # another node's method, a notification (no id): no answer
    assert rpc('mowing.plan', {}) == (None, None)
    assert rpc('planner.settings.set', {'turn_radius': 0.5}, rid='') == (None, None)
    assert planner.base.turn_radius == 0.5


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
    # kept over a restart
    event('JOB_COMPLETE')
    event('JOB_COMPLETE')
    assert node_module.MowbitePlannerRos1().angle == {'steps': 1, 'mows': 0}


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


def test_planner_drive(planner):
    """planner.drive: the driven track (OpenMower's position_history) against the plan's body_space, the places where
    the body got more than DRIVE_OFF into the margins, the furthest point of each; after a finished mow by itself, in
    the log and as the answer without params. Without a track an error."""
    rpc('planner.settings.set', {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'edges': 'recorded'})
    send_map(the_map())
    result, error = rpc('planner.drive', {'job_id': 'nope'})
    assert result is None and 'no track' in error[1]
    inside = [(x / 10.0, 5.0) for x in range(10, 61)]
    out = [(19.0, 2.0), (19.5, 2.0), (20.05, 2.0), (20.1, 2.0), (19.5, 2.4), (18.0, 2.4)]
    track_file('job1', [(False, [(1.0, 5.0)]), (True, inside), (True, out)])
    result, error = rpc('planner.drive', {'job_id': 'job1'})
    assert error is None and result['file'] == '1791377699_job1.jsonl'
    a = result['areas']['a']
    assert a['checked'] > 50 and len(a['places']) == 1 and a['places'][0][:2] == [20.05, 2.0] and a['worst'] > 0.2
    assert node_module.DRIVE_OFF == 0.01
    # after a finished mow by itself
    rospy.subscribers['/xbot_monitoring/mqtt_publish'](node_module.MqttPublish(
        topic='events/json', payload=json.dumps({'type': 'JOB_COMPLETE', 'job_id': 'job1'})))
    planner.drive_thread.join(60.0)
    assert any(level == 'warn' and 'got into the margins at 1 places' in line for level, line in rospy.log)
    assert rpc('planner.drive')[0] == result
