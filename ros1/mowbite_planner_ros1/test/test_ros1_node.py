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


# the properties of an area stock mower_map keeps (MapArea's from_json and to_json), openmower.patch keeps all
STOCK_PROPERTIES = ('name', 'type', 'active', 'mowable', 'angle', 'outline_count', 'outline_overlap_count',
                    'outline_offset')


def through_mower_map(data, patched):
    """The map as mower_map publishes it on json_map."""
    return dict(data, areas=[dict(a, properties={k: v for k, v in a['properties'].items()
                                                 if patched or k in STOCK_PROPERTIES}) for a in data['areas']])


def send_map(data):
    rospy.subscribers['/mower_map_service/json_map'](types.SimpleNamespace(data=json.dumps(data)))


@pytest.fixture
def planner():
    rospy.params = {}
    rospy.log.clear()
    rospy.published.clear()
    rospy.registered.clear()
    try:
        os.remove(os.path.join(os.environ['ROS_HOME'], 'mowbite_planner_settings.json'))
    except FileNotFoundError:
        pass
    # no plans kept from other tests, none planned ahead in the background (test_plan_ahead does that itself)
    shutil.rmtree(os.path.join(os.environ['ROS_HOME'], 'mowbite_plans'), ignore_errors=True)
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
    rospy.params = {'~settings': {'lane_order': 'snake', 'allow_reverse': True, 'turn_types': 'k_turn,pivot'}}
    p = node_module.MowbitePlannerRos1()
    assert p.base.lane_order == 'snake' and p.base.turn_types == ('k_turn', 'pivot')
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


@pytest.mark.parametrize('patched', [False, True])
def test_mowbite_properties_need_the_patched_mower_map(planner, patched):
    """An area's planner property, angle range and mow_around reach the node only through mower_map with
    openmower.patch: stock mower_map drops the properties it doesn't know."""
    plain = planner.handle(request())
    pond = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    send_map(through_mower_map(the_map({'fill_pattern': 'crosshatch'}, angle_min=0.2, angle_max=0.4, areas=[
        area('pond', pond, type='mow', mowable=False, mow_around=True)]), patched))
    res = planner.handle(request(angle=1.0))
    check(res)
    assert (len(poses(res)) > 1.6 * len(poses(plain))) == patched  # crosshatch
    assert (not inside(res, pond)) == patched  # mowed around
    if patched:  # in the area's range, not mower_logic's 1 rad
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


def test_backing_up_only_where_the_ftc_controller_does(planner, monkeypatch):
    # allow_reverse counts only with OpenMower's FTC controller backing up where the plan does, else it would hold the
    # mower at every backing up leg until it times out
    got, real = [], node_module.plan_coverage
    monkeypatch.setattr(node_module, 'plan_coverage', lambda *args, **kw: got.append(args[2]) or real(*args, **kw))
    rpc('planner.settings.set', {'allow_reverse': True})
    planner.handle(request())
    assert got[-1].allow_reverse is False and any('allow_reverse left out' in m for _, m in rospy.log)
    rospy.params[node_module.BACK_UP_PARAM] = True
    planner.handle(request(angle=0.4))
    assert got[-1].allow_reverse is True
    assert rpc('planner.settings')[0]['can_back_up'] is True


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
    service = PlannerService(node_module.PlannerConfig(allow_reverse=False, lane_order='skip', mower_width=0.18)
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


def test_patch_writes_the_known_properties_first():
    """MowBite's scheduler (docker/scheduler.sh) reads type, active and mowable of an area up to the first
    closing brace in its properties: the patched mower_map writes the ones it doesn't know (a planner object)
    after them."""
    with open(os.path.join(HERE, '..', '..', 'openmower.patch')) as f:
        patch = f.read()
    hunk = [h for h in patch.split('\n@@ ') if 'properties.update(data.other_properties);' in h]
    assert len(hunk) == 1 and 'void to_json(json& j, const MapArea& data)' in hunk[0].splitlines()[0]
    lines = [line[1:].strip() for line in hunk[0].splitlines()[1:] if line[:1] in (' ', '+') and '//' not in line]
    update = lines.index('properties.update(data.other_properties);')
    assert lines[update - 1].startswith('if (!std::isnan(data.outline_offset)) properties["outline_offset"]')
    assert lines[update + 1] == 'j["properties"] = properties;'
    assert 'json properties = data.other_properties' not in patch


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
    methods = ['planner.angle.reset', 'planner.plan', 'planner.settings', 'planner.settings.set']
    assert ('mowbite_planner', methods) in rospy.registered
    rospy.registered.clear()
    rospy.subscribers['/xbot/rpc/registry'](None)  # xbot_monitoring started again
    assert rospy.registered == [('mowbite_planner', methods)]


def test_rpc_settings(planner):
    assert rpc('planner.settings', 'null') == rpc('planner.settings', {})
    result, error = rpc('planner.settings')
    assert error is None and result['file'] == stored_file() and result['own_angle'] is False
    s = result['settings']
    assert set(s) == set(node_module.PlannerConfig.names())
    assert s['turn_radius'] == {'value': 0.25, 'default': 0.25, 'stored': False, 'type': 'number', 'settable': True,
                                'unit': 'm', 'min': 0.0, 'max': 10.0}
    assert s['mower_width']['value'] == 0.18 and s['mower_width']['settable']
    assert s['lane_order']['value'] == 'skip' and s['lane_order']['choices'] == ['snake', 'skip']
    assert s['turn_types']['type'] == 'list' and s['turn_types']['value'] == s['turn_types']['choices']
    assert s['angle'] == {'value': None, 'default': None, 'stored': False, 'type': 'number', 'settable': True,
                          'nullable': True, 'unit': 'rad'}
    assert s['perimeter_passes']['type'] == 'integer' and s['perimeter_passes']['max'] == 255
    assert not s['skip_lanes']['settable'] and s['skip_lanes']['from']
    assert s['allow_reverse']['settable'] and s['allow_reverse']['value'] is False
    assert result['can_back_up'] is False
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
    ({'body_fit': 1}, 'body_fit must be true or false'),
    ({'allow_reverse': True, 'skip_lanes': True}, 'not settable here: skip_lanes'),
    ({'nope': 1}, 'unknown planner settings: nope'),
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
