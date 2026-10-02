"""Tests of the ROS 1 node without ROS: rospy and the message packages are stood in for here.

    python3 -m pytest ros1/mowbite_planner_ros1/test   (shapely is enough)
"""

import importlib.machinery
import importlib.util
import json
import math
import os
import re
import struct
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
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

    class Rospy(types.ModuleType):
        params = {}
        log = []
        services = {}
        subscribers = {}

        class Time:
            @staticmethod
            def now():
                return 123

        def get_param(self, name, default=None):
            return self.params.get(name, default)

        def Subscriber(self, topic, kind, callback):
            self.subscribers[topic] = callback

        def Service(self, name, kind, handler):
            self.services[name] = handler
            return types.SimpleNamespace(resolved_name=name)

        def loginfo(self, *a):
            self.log.append(('info', a[0] % a[1:] if len(a) > 1 else a[0]))

        def logwarn(self, *a):
            self.log.append(('warn', a[0] % a[1:] if len(a) > 1 else a[0]))

        def logerr(self, *a):
            self.log.append(('error', a[0] % a[1:] if len(a) > 1 else a[0]))

    rospy = Rospy('rospy')
    modules = {
        'rospy': rospy,
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.SimpleNamespace(Header=header, String=msg('String', data=str)),
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


def test_same_request_same_plan(planner):
    a = [(p.position.x, p.position.y, p.orientation.z) for p in poses(planner.handle(request()))]
    b = [(p.position.x, p.position.y, p.orientation.z) for p in poses(planner.handle(request()))]
    assert a == b


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
    # an angle or angle_strategy in ~settings beats mower_logic's angle, the rest always comes with the request
    for settings, angle in (({}, 0.7), ({'angle': 0.3}, 0.3), ({'angle_strategy': 'longest_edge'}, None),
                            ({'angle': None}, None)):
        rospy.params = {'~settings': dict(settings, perimeter_passes=5, overlap=0.5, skip_lanes=True)}
        cfg = node_module.MowbitePlannerRos1().settings(request(angle=0.7), None)
        assert cfg.angle == angle and cfg.perimeter_passes == 3 and cfg.lane_spacing == pytest.approx(0.13)
        assert not cfg.skip_lanes
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
