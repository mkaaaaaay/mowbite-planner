"""Tests of the JSON-RPC service over MQTT, without a broker: PlannerService.handle() directly, the paho layer
with a stand-in client. With paho-mqtt and a broker (MOWBITE_TEST_MQTT=host:port, default 127.0.0.1:1883)
one test also goes over the network."""

import json
import math
import os
import signal
import socket
import sys
import threading
import types
import uuid

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import PlannerConfig, mqtt_service  # noqa: E402
from mowbite_planner.mqtt_service import MqttBridge, PlannerService, main  # noqa: E402


def rect(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def outline(points):
    return [{'x': x, 'y': y} for x, y in points]


def area(id_, kind, points, **props):
    return {'id': id_, 'properties': {'name': id_.title(), 'type': kind, **props}, 'outline': outline(points)}


MAP = {
    'areas': [
        area('front', 'mow', rect(0, 0, 10, 6), angle=0.5, outline_count=1),
        area('bed', 'obstacle', rect(4, 2, 5, 3)),
        area('path', 'nav', rect(-4, 0, 0, 2)),
        area('back', 'mow', rect(0, 8, 6, 12)),
    ],
    'docking_stations': [{'id': 'dock', 'position': {'x': -3.0, 'y': 1.0}, 'heading': math.pi}],
}
PARAMS = {'/mower_logic/tool_width': 0.2, '/mower_logic/outline_count': 3, '/mower_logic/mqtt_password': None}


def call(service, method, params=None, rid='1'):
    request = {'jsonrpc': '2.0', 'id': rid, 'method': method}
    if params is not None:
        request['params'] = params
    response = service.handle(request)
    if response is not None:
        json.dumps(response, allow_nan=False)  # what goes out has to be valid JSON
        assert response['jsonrpc'] == '2.0' and response['id'] == rid
    return response


def result(service, method, params=None):
    response = call(service, method, params)
    assert 'error' not in response, response
    return response['result']


def error(service, method, params=None):
    response = call(service, method, params)
    assert 'result' not in response, response
    return response['error']


@pytest.fixture
def service():
    s = PlannerService()
    s.on_map(json.dumps(MAP).encode())
    s.on_params(json.dumps(PARAMS))
    return s


@pytest.fixture
def planned(monkeypatch):
    """What plan_coverage() got: (boundary, obstacles, config, start, end, drivable, keep_out, margins) of every
    call."""
    calls, real = [], mqtt_service.plan_coverage

    def spy(*args):
        calls.append(args)
        return real(*args)

    monkeypatch.setattr(mqtt_service, 'plan_coverage', spy)
    return calls


def poses_in(plan, polygon):
    x0, y0 = polygon[0]
    x1, y1 = polygon[2]
    return [p for p in plan['poses'] if x0 < p[0] < x1 and y0 < p[1] < y1]


# the service ----------------------------------------------------------------------------------------------


def test_info():
    s = PlannerService(PlannerConfig(turn_radius=0.3))
    info = result(s, 'ext.mowbite_planner.info')
    assert info['methods'] == ['ext.mowbite_planner.info', 'ext.mowbite_planner.plan', 'ext.planner.plan']
    assert set(info['settings']) == set(PlannerConfig.names())
    assert info['settings']['turn_radius'] == 0.3 and info['settings']['angle'] is None
    assert info['settings']['turn_types'] == list(PlannerConfig().turn_types)
    assert info['map'] is False and info['params'] is False and info['version']
    s.on_map(json.dumps(MAP))
    s.on_params(b'{}')
    info = result(s, 'ext.mowbite_planner.info', {})
    assert info['map'] is True and info['params'] is True
    s.on_map(b'')  # retained map cleared
    s.on_params(b'not json')
    info = result(s, 'ext.mowbite_planner.info')
    assert info['map'] is False and info['params'] is False


def test_foreign_requests_are_not_answered(service):
    assert call(service, 'mowing.plan', {'area_id': 'front'}) is None
    assert call(service, 'ext.someone_else.plan', {}) is None
    assert call(service, 'rpc.methods') is None
    assert service.handle({'jsonrpc': '2.0', 'method': 'ext.mowbite_planner.info'}) is None  # a notification
    # xbot_monitoring answers these itself (id not a string, not JSON-RPC 2.0), there must be one answer only
    for rid in (1, None, 1.5, ['1'], {'id': '1'}, True):
        assert service.handle({'jsonrpc': '2.0', 'id': rid, 'method': 'ext.mowbite_planner.info'}) is None
    for version in (None, '1.0', 2.0, 2):
        assert service.handle({'jsonrpc': version, 'id': '1', 'method': 'ext.mowbite_planner.info'}) is None
    assert service.handle({'id': '1', 'method': 'ext.mowbite_planner.info'}) is None
    assert service.handle({'jsonrpc': '2.0', 'id': '1', 'method': ['ext.mowbite_planner.info']}) is None
    assert service.handle(['ext.mowbite_planner.info']) is None
    assert service.handle(None) is None


def test_plan_outline():
    s = PlannerService()
    res = result(s, 'ext.mowbite_planner.plan', {
        'outline': outline(rect(0, 0, 10, 6)), 'obstacles': [rect(4, 2, 5, 3)], 'angle': 0.0, 'outline_count': 1,
        'settings': {'mower_width': 0.3, 'overlap': 0.0}, 'start': {'x': 10, 'y': 6, 'heading': 0}, 'poses': True})
    assert res['area_id'] is None and res['paths'] and len(res['fingerprint']) == 64
    assert res['lane_spacing'] == pytest.approx(0.3)
    assert res['angle'] == pytest.approx(0.0)
    assert sum(p['is_outline'] for p in res['paths']) >= 1
    assert math.hypot(res['poses'][0][0] - 10, res['poses'][0][1] - 6) < 0.6
    assert not poses_in(res, rect(4.1, 2.1, 4.9, 2.9))
    # the same request, the same plan
    again = result(s, 'ext.mowbite_planner.plan', {
        'outline': outline(rect(0, 0, 10, 6)), 'obstacles': [rect(4, 2, 5, 3)], 'angle': 0.0, 'outline_count': 1,
        'settings': {'mower_width': 0.3, 'overlap': 0.0}, 'start': [10, 6, 0]})
    assert again['fingerprint'] == res['fingerprint'] and 'poses' not in again


def test_plan_with_report_and_trace():
    s = PlannerService()
    res = result(s, 'ext.mowbite_planner.plan', {
        'outline': outline(rect(0, 0, 10, 6)), 'obstacles': [rect(4, 2, 5, 3)], 'report': True, 'trace': True})
    json.dumps(res, allow_nan=False)
    assert res['report']['hard'] == [] and res['report']['left_m2'] < 0.1 and 'double_mowed_pct' in res['report']
    assert [st['key'] for st in res['stages']][-2:] == ['route', 'ends']
    assert {o['id'] for o in res['lineage']['objects']} >= {'level-0', 'lane-0', 'loop-0', 'seg-0'}
    plain = result(s, 'ext.mowbite_planner.plan', {'outline': outline(rect(0, 0, 10, 6)),
                                                   'obstacles': [rect(4, 2, 5, 3)]})
    assert 'report' not in plain and 'lineage' not in plain and plain['fingerprint'] == res['fingerprint']


def test_nested_area_mowed_on_its_own(service, planned):
    """A zone in the front lawn with an angle of its own: the lawn mows round it, the zone its own lanes. With
    nested_areas off the lawn mows over it as before."""
    zone = rect(6, 1, 9, 5)
    service.on_map(json.dumps(dict(MAP, areas=MAP['areas'] + [area('zone', 'mow', zone, angle=1.4)])).encode())
    lawn = result(service, 'ext.mowbite_planner.plan', {'area_id': 'front', 'poses': True})
    assert not poses_in(lawn, rect(6.3, 1.3, 8.7, 4.7))
    assert [tuple(p) for p in zone] in [[tuple(q) for q in o] for o in planned[-1][1]]
    own = result(service, 'ext.mowbite_planner.plan', {'area_id': 'zone', 'poses': True})
    assert own['angle'] == pytest.approx(1.4) and poses_in(own, rect(6.3, 1.3, 8.7, 4.7))
    over = result(service, 'ext.mowbite_planner.plan', {'area_id': 'front', 'poses': True,
                                                         'settings': {'nested_areas': False}})
    assert poses_in(over, rect(6.3, 1.3, 8.7, 4.7))


def test_plan_area_of_the_map(service, planned):
    res = result(service, 'ext.mowbite_planner.plan', {'area_id': 'front', 'poses': True})
    assert res['area_id'] == 'front'
    _, obstacles, _, start, end, drivable, keep_out, margins = planned[0]
    assert keep_out == [] and margins == [None]
    assert start == (-3.0, 1.0, math.pi) and end == (-3.0, 1.0)  # from the dock and back
    # the bed; the other lawn, not the path: the mower mows a path from start to end
    assert obstacles == [[tuple(p) for p in rect(4, 2, 5, 3)]] and drivable == [[tuple(p) for p in rect(0, 8, 6, 12)]]
    assert res['angle'] == pytest.approx(0.5)  # the area's own angle
    assert res['lane_spacing'] == pytest.approx(0.2)  # the mower's tool_width
    assert not poses_in(res, rect(4.1, 2.1, 4.9, 2.9))  # the obstacle
    first = res['poses'][0]
    assert first[0] < 0.5 and first[1] < 2.5  # starts next to the dock
    # outline_count of the area (1) wins over the mower's (3), the request's over both, its settings over all
    plans = [result(service, 'ext.mowbite_planner.plan', {'area_id': 'front', **params}) for params in (
        {'outline_count': 2, 'angle': 0.0}, {'outline_count': 2, 'settings': {'perimeter_passes': 3}})]
    loops = [plan['stats']['perimeter_loops'] for plan in [res] + plans]
    assert loops == sorted(set(loops))
    assert plans[0]['angle'] == pytest.approx(0.0) and plans[1]['angle'] == pytest.approx(0.5)
    # by name too, and a start of the request's own
    other = result(service, 'ext.mowbite_planner.plan', {'area_id': 'Back', 'start': [6, 12]})
    assert other['area_id'] == 'back' and planned[-1][3:5] == ((6.0, 12.0), (-3.0, 1.0))
    # an angle_strategy in the request's settings works the angle out, the area's angle doesn't count then
    res = result(service, 'ext.mowbite_planner.plan', {'area_id': 'front', 'settings': {'angle_strategy': 'min_width'}})
    assert planned[-1][2].angle is None and res['angle'] == pytest.approx(0.0)  # across the 6 m


def test_plan_with_the_margins_of_the_map(planned):
    """An obstacle's or area's own margin comes from the map, a request's obstacle has it if its outline is the
    map's. With the body checked the answer says where the body keeps to."""
    s = PlannerService(PlannerConfig(robot_width=0.4, robot_front=0.43, robot_rear=0.14))
    bed, pond = rect(4, 2, 5, 3), rect(7, 3, 8, 4)
    s.on_map(json.dumps(dict(MAP, areas=[MAP['areas'][0], area('bed', 'obstacle', bed, margin=0.3),
                                         area('pond', 'mow', pond, mowable=False, margin=0.2)])))
    res = result(s, 'ext.mowbite_planner.plan', {'area_id': 'front'})
    assert planned[-1][1] == [[tuple(p) for p in bed]] and planned[-1][6] == [[tuple(p) for p in pond]]
    assert planned[-1][7] == [0.3, 0.2]
    assert [len(res['body_space'][k]) for k in ('outlines', 'holes')] == [1, 2]
    moved = [[x + 0.5, y] for x, y in bed]
    for obstacles, margins in (([bed], [0.3]), ([moved], [None])):
        result(s, 'ext.mowbite_planner.plan', {'outline': rect(0, 0, 10, 6), 'obstacles': obstacles})
        assert planned[-1][7] == margins


def test_plan_like_the_mower(planned):
    """The preview plans what the ROS 1 node gives mower_logic: parts only reachable over a navigation area
    get paths of their own, mow_around areas are left out, an absolute mow_angle_offset is the angle."""
    s = PlannerService(PlannerConfig(lane_order='skip'))
    # a hedge across the lawn up to 20 cm before its end, the drive across the top
    s.on_map(json.dumps({'areas': [area('lawn', 'mow', rect(0, 0, 20, 10), angle=1.0),
                                   area('hedge', 'obstacle', rect(9, -1, 11, 9.8)),
                                   area('drive', 'nav', rect(7, 9.5, 13, 13)),
                                   area('pond', 'mow', rect(2, 2, 4, 4), mowable=False, mow_around=True)],
                         'docking_stations': []}))
    s.on_params(json.dumps({'/mower_logic/mow_angle_offset': 30.0, '/mower_logic/mow_angle_offset_is_absolute': True}))
    res = result(s, 'ext.mowbite_planner.plan', {'area_id': 'lawn', 'poses': True})
    assert planned[-1][5] == [] and len(planned[-1][1]) == 2  # nothing to drive over; the hedge and the pond
    assert not [p for p in res['poses'] if p[1] > 10.0] and not poses_in(res, rect(2.1, 2.1, 3.9, 3.9))
    assert [p['is_outline'] for p in res['paths']] == [True, False, True, False]  # each side its loops and lanes
    assert res['angle'] == pytest.approx(math.radians(30))


def test_plan_with_the_area_properties_in_the_request(planned):
    """The area's own properties repeated in the request change nothing: the area is no other lawn to drive
    over."""
    # two lawns joined by a neck 0.1 m wide, too narrow for the mower: no way between them
    lawn = [[0, 0], [8, 0], [8, 3.95], [10, 3.95], [10, 0], [18, 0], [18, 8], [10, 8], [10, 4.05], [8, 4.05],
            [8, 8], [0, 8]]
    s = PlannerService(PlannerConfig(lane_order='skip', mower_width=0.18))
    s.on_map(json.dumps({'areas': [area('lawn', 'mow', lawn, angle=0.0, outline_count=2)],
                         'docking_stations': [{'id': 'd', 'position': {'x': 1.0, 'y': 1.0}, 'heading': 0.0}]}))
    s.on_params(json.dumps({'/mower_logic/tool_width': 0.15, '/mower_logic/outline_offset': 0.09}))
    plain = result(s, 'ext.mowbite_planner.plan', {'area_id': 'lawn', 'poses': True})
    assert planned[-1][5] == [] and any('no way between them' in w for w in plain['warnings'])
    for same in ({'angle': 0.0}, {'outline_count': 2}):
        res = result(s, 'ext.mowbite_planner.plan', {'area_id': 'lawn', 'poses': True, **same})
        assert planned[-1][5] == [] and res['poses'] == plain['poses'], same


def test_plan_angle_like_the_mower(planned):
    """The lanes go the way mower_logic sends the ROS 1 node: the area's angle or OpenMower's own (towards the
    first point more than 2 m from the first) plus mow_angle_offset. PLANNER_SETTINGS with angle or
    angle_strategy count like the node's settings: only for areas without an angle of their own."""
    lawn = [[0, 4], [2, 0], [20, 0], [20, 12], [0, 12]]
    auto, ten = math.atan2(-4, 2), math.radians(10)
    the_map = json.dumps({'areas': [area('auto', 'mow', lawn), area('own', 'mow', lawn, angle=0.5),
                                    area('fixed', 'mow', lawn, angle=0.5, planner={'angle': 0.2})]})
    for own_angle, base in ((False, PlannerConfig()), (True, PlannerConfig(angle_strategy='min_width'))):
        s = PlannerService(base, own_angle)
        s.on_map(the_map)
        s.on_params(json.dumps({'/mower_logic/mow_angle_offset': 10.0}))
        auto_angle = None if own_angle else pytest.approx(auto + ten)
        for params, angle in (({'area_id': 'auto'}, auto_angle), ({'outline': lawn}, auto_angle),  # an area too
                              ({'area_id': 'own'}, pytest.approx(0.5 + ten)), ({'area_id': 'fixed'}, 0.2),
                              ({'area_id': 'auto', 'angle': 1.0}, pytest.approx(1.0 + ten))):  # like the area's
            result(s, 'ext.mowbite_planner.plan', params)
            cfg = planned[-1][2]
            assert (cfg.angle, cfg.angle_offset, cfg.angle_strategy) == (angle, 0.0, base.angle_strategy), params


def test_plan_errors(service):
    e = error(service, 'ext.mowbite_planner.plan', {'area_id': 'nowhere'})
    assert e['code'] == -32602 and 'Front' in e['message'] and 'Back' in e['message']
    e = error(PlannerService(), 'ext.mowbite_planner.plan', {'area_id': 'front'})
    assert e['code'] == -32602 and 'map' in e['message']
    square = rect(0, 0, 9, 9)
    for params in ({'outline': [[0, 0], [1, 'a'], [1, 1]]}, {'outline': None}, {'outline': [[0, 0], [1, 1]]},
                   {'outline': square, 'obstacles': {'a': 1}}, {'outline': square, 'start': 'dock'},
                   {'outline': square, 'start': '12'}, {'outline': square, 'end': [1, math.nan]},
                   {'outline': square, 'settings': [1]}, {'outline': square, 'settings': {'no_such_setting': 1}},
                   {'outline': square, 'settings': {'overlap': 2.0}},
                   # odd types: strings, booleans, huge numbers, points without x or y, settings of the wrong kind
                   {'outline': [[0, 0], [9, '0'], [9, 9]]}, {'outline': [[0, 0], [9, True], [9, 9]]},
                   {'outline': [[0, 0], [10 ** 400, 0], [9, 9]]}, {'outline': [{'x': 0}, {'x': 9, 'y': 0}, [9, 9]]},
                   {'outline': [[0, 0], [1e39, 0], [9, 9]]},  # no float32 either
                   {'outline': square, 'obstacles': [[[1, 1], [1e300, 1], [1, 2]]]},
                   {'outline': 5}, {'outline': square, 'obstacles': [5]}, {'outline': square, 'nav_areas': 'x'},
                   {'outline': square, 'start': [1, '2']}, {'outline': square, 'start': {'x': 1}},
                   {'outline': square, 'end': [1, 2, 3, 4]}, {'outline': square, 'outline_count': 300},
                   {'outline': square, 'settings': {'turn_types': 5}},
                   {'outline': square, 'settings': {'perimeter_passes': 'x'}},
                   {'outline': square, 'settings': {'mower_width': -1}},
                   {'outline': square, 'settings': {'fill_pattern': ['lanes']}},
                   {'outline': square, 'settings': {'turn_on_spot': 'no'}},
                   {'area_id': ['front']}, {'area_id': {'id': 'front'}}, {'area_id': 4}):
        assert error(service, 'ext.mowbite_planner.plan', params)['code'] == -32602, params
    assert error(service, 'ext.mowbite_planner.plan', [{'area_id': 'front'}])['code'] == -32602  # by position


def test_broken_maps(service, planned):
    """A broken map from the mower: what's broken about other areas is left out, a broken area to plan is an
    error that names it, never an internal error."""
    junk = [None, 'x', [1], {'id': 'nav', 'properties': {'type': 'nav'}, 'outline': [{'x': 0}]},
            {'id': 'props', 'properties': ['mow'], 'outline': outline(rect(0, 0, 1, 1))},
            {'id': 'turns', 'properties': {'type': 'mow', 'planner': {'turn_types': 5}},
             'outline': outline(rect(0, 0, 1, 1))},
            {'id': 'nan', 'properties': {'type': 'mow'}, 'outline': [[0, 0], [1, None], [1, 1]]},
            {'id': 'two', 'properties': {'type': 'mow'}, 'outline': [[0, 0], [1, 1]]},
            {'id': 'none', 'properties': {'type': 'mow'}}]
    service.on_map(json.dumps(dict(MAP, areas=MAP['areas'] + junk, docking_stations=[None, {'position': 'dock'}])))
    assert result(service, 'ext.mowbite_planner.plan', {'area_id': 'back'})['area_id'] == 'back'
    assert planned[-1][3] is None  # no docking station to start from
    for which, what in (('turns', 'turn_types'), ('nan', 'point 1'), ('two', 'fewer than 3'), ('none', 'outline'),
                        ('props', 'no mowing area')):
        e = error(service, 'ext.mowbite_planner.plan', {'area_id': which})
        assert e['code'] == -32602 and repr(which) in e['message'] and what in e['message'], e
    for data in ({'areas': 5}, {'areas': {'front': MAP['areas'][0]}}, {'areas': [None]}):
        service.on_map(json.dumps(data))
        e = error(service, 'ext.mowbite_planner.plan', {'area_id': 'front'})
        assert e['code'] == -32602 and 'no mowing area' in e['message']
    # the mower's parameters with odd types are left out
    service.on_map(json.dumps(MAP))
    service.on_params(json.dumps({'/mower_logic/tool_width': 'wide', '/mower_logic/outline_count': [3]}))
    assert result(service, 'ext.mowbite_planner.plan', {'area_id': 'back', 'settings': {'mower_width': 0.5}})['paths']


def test_internal_errors(service, monkeypatch):
    def broken(*args, **kwargs):
        raise RuntimeError('boom')

    monkeypatch.setattr(mqtt_service, 'plan_coverage', broken)
    e = error(service, 'ext.mowbite_planner.plan', {'area_id': 'front'})
    assert e['code'] == -32603 and 'boom' in e['message']


def test_no_nan_in_results(service, monkeypatch):
    monkeypatch.setattr(mqtt_service, 'to_openmower_json',
                        lambda plan, area_id: {'a': float('nan'), 'b': [float('inf'), (1.5, -math.inf)], 'c': 'x'})
    assert result(service, 'ext.mowbite_planner.plan', {'area_id': 'front'}) == {
        'a': None, 'b': [None, [1.5, None]], 'c': 'x'}


def test_studio_plan(service, planned):
    params = {'tool_width': 0.25, 'headland_rounds': 1, 'min_turning_radius': 0.3, 'ring_order': 'last',
              'angle_deg': 30, 'start_point': [9, 5], 'end_point': [1, 1], 'overlap': 0.1, 'speed': 0.4,
              'wiggle': True}
    res = result(service, 'ext.planner.plan', {'area': rect(0, 0, 10, 6), 'obstacles': [rect(4, 2, 5, 3)],
                                               'nav_areas': [rect(-4, 0, 0, 2)], 'params': params})
    cfg, start, end = planned[0][2:5]
    assert (cfg.mower_width, cfg.perimeter_passes, cfg.turn_radius, cfg.perimeter_order) == (0.25, 1, 0.3, 'last')
    assert cfg.angle == pytest.approx(math.radians(30)) and start == (9.0, 5.0) and end == (1.0, 1.0)
    assert res['params'] == params and res['angle_deg'] == pytest.approx(30)
    assert [s for s in res['report']['soft'] if 'speed' in s] and [s for s in res['report']['soft'] if 'wiggle' in s]
    assert not [s for s in res['report']['soft'] if 'tool_width' in s or 'overlap' in s]
    mowbite = res['mowbite']
    assert mowbite['lane_spacing'] == pytest.approx(0.25 * 0.9)
    assert not mowbite['paths'][0]['is_outline'] and mowbite['paths'][-1]['is_outline']  # loops last
    assert res['domains']['area'] == rect(0, 0, 10, 6) and len(res['domains']['nav_areas']) == 1
    assert res['stages'] and res['lineage']['objects']
    # the mower's parameters don't count here, angle_deg null is worked out
    res = result(service, 'ext.planner.plan', {'area': rect(0, 0, 10, 6), 'params': {'angle_deg': None}})
    assert res['mowbite']['lane_spacing'] == pytest.approx(PlannerConfig().lane_spacing)
    for params in ({'area': [[0, 0], [1, 1]]}, {'area': rect(0, 0, 9, 9), 'params': {'ring_order': 'middle'}},
                   {'area': rect(0, 0, 9, 9), 'params': {'end_point': [1]}}, {'area': rect(0, 0, 9, 9), 'params': 3},
                   {'area': 5}, {'area': [[0, 0], [9, '0'], [9, 9]]}, {'area': rect(0, 0, 9, 9), 'obstacles': [[1]]},
                   {'area': rect(0, 0, 9, 9), 'params': {'start_point': {'x': 1, 'y': None}}},
                   {'area': rect(0, 0, 9, 9), 'params': {'tool_width': 'wide'}},
                   {'area': rect(0, 0, 9, 9), 'params': {'turn_types': 5}},
                   {'area': rect(0, 0, 9, 9), 'params': {'headland_rounds': 1.5}}):
        assert error(service, 'ext.planner.plan', params)['code'] == -32602, params


# the paho layer -------------------------------------------------------------------------------------------


class Message:
    def __init__(self, topic, payload):
        self.topic, self.payload = topic, payload


class FakeClient:
    """What MqttBridge and main() use of paho's Client."""

    def __init__(self, *args, **kwargs):
        self.args, self.subscribed, self.published = args, [], []
        self.on_connect = self.on_message = None
        self.user = self.connected = None

    def subscribe(self, topics):
        self.subscribed.extend(topics)

    def publish(self, topic, payload):
        self.published.append((topic, json.loads(payload)))

    def username_pw_set(self, user, password):
        self.user = (user, password)

    def connect_async(self, host, port, keepalive):
        self.connected = (host, port)

    def loop_forever(self, retry_first_connection):
        self.on_connect(self, None, {}, 0, None)

    def disconnect(self):
        pass


def test_bridge():
    client = FakeClient()
    bridge = MqttBridge(PlannerService(), client, 'openmower')
    bridge.start()
    client.on_connect(client, None, {}, 5)  # refused
    assert not client.subscribed
    client.on_connect(client, None, {}, 0)
    topics = sorted(t for t, _ in client.subscribed)
    assert topics == ['openmower/map/json', 'openmower/params/json', 'openmower/rpc/request']
    deliver = lambda topic, data: client.on_message(client, None, Message('openmower/' + topic, data))
    deliver('map/json', json.dumps(MAP).encode())
    deliver('params/json', json.dumps(PARAMS).encode())
    for rid, method, params in (('1', 'mowing.plan', {}), ('2', 'ext.mowbite_planner.info', None),
                                ('3', 'ext.mowbite_planner.plan', {'area_id': 'back'}),
                                ('4', 'ext.mowbite_planner.plan', {'area_id': 'nowhere'})):
        deliver('rpc/request', json.dumps({'jsonrpc': '2.0', 'id': rid, 'method': method, 'params': params}).encode())
    deliver('rpc/request', b'{"jsonrpc": "2.0", "id": "5", "method": ')  # xbot_monitoring answers that
    deliver('rpc/response', b'{}')
    bridge.requests.join()
    assert [t for t, _ in client.published] == ['openmower/rpc/response'] * 3
    info, plan, missing = [r for _, r in client.published]
    assert info['id'] == '2' and info['result']['map'] and info['result']['params']
    assert plan['id'] == '3' and plan['result']['area_id'] == 'back' and plan['result']['paths']
    assert missing['id'] == '4' and missing['error']['code'] == -32602


def test_bridge_publishes_an_error_for_what_is_no_json(monkeypatch):
    monkeypatch.setattr(PlannerService, 'info', lambda self, p: {'what': object()})
    client = FakeClient()
    bridge = MqttBridge(PlannerService(), client)
    bridge.answer(json.dumps({'jsonrpc': '2.0', 'id': 'x', 'method': 'ext.mowbite_planner.info'}))
    [(topic, response)] = client.published
    assert topic == 'rpc/response' and response['id'] == 'x' and response['error']['code'] == -32603


def fake_paho(monkeypatch, with_api_version):
    clients = []

    def make(*args):
        clients.append(FakeClient(*args))
        return clients[-1]

    client_mod = types.ModuleType('paho.mqtt.client')
    client_mod.Client = make
    if with_api_version:
        client_mod.CallbackAPIVersion = types.SimpleNamespace(VERSION2='v2')
    mqtt_mod = types.ModuleType('paho.mqtt')
    mqtt_mod.client = client_mod
    paho = types.ModuleType('paho')
    paho.mqtt = mqtt_mod
    for name, mod in (('paho', paho), ('paho.mqtt', mqtt_mod), ('paho.mqtt.client', client_mod)):
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.setattr(signal, 'signal', lambda *args: None)
    return clients


@pytest.mark.parametrize('paho2', [True, False])
def test_main(monkeypatch, paho2):
    clients = fake_paho(monkeypatch, paho2)
    for key in ('MOWER_MQTT_HOST', 'MOWER_MQTT_PORT', 'MOWER_MQTT_USER', 'MOWER_MQTT_PASSWORD', 'MOWER_MQTT_PREFIX',
                'PLANNER_SETTINGS'):
        monkeypatch.delenv(key, raising=False)
    main()
    client = clients[0]
    assert client.args == (('v2',) if paho2 else ())
    assert client.connected == ('127.0.0.1', 1883) and client.user is None
    assert sorted(t for t, _ in client.subscribed) == ['map/json', 'params/json', 'rpc/request']

    monkeypatch.setenv('MOWER_MQTT_HOST', 'mower.local')
    monkeypatch.setenv('MOWER_MQTT_PORT', '1884')
    monkeypatch.setenv('MOWER_MQTT_USER', 'me')
    monkeypatch.setenv('MOWER_MQTT_PASSWORD', 'secret')
    monkeypatch.setenv('MOWER_MQTT_PREFIX', 'garden/')
    monkeypatch.setenv('PLANNER_SETTINGS', '{"turn_radius": 0.4, "lane_order": "skip"}')
    main()
    client = clients[1]
    assert client.connected == ('mower.local', 1884) and client.user == ('me', 'secret')
    assert 'garden/rpc/request' in [t for t, _ in client.subscribed]
    service = client.on_message.__self__.service
    assert service.base.turn_radius == 0.4 and service.base.lane_order == 'skip' and not service.own_angle
    # an angle or angle_strategy for all areas, like the ROS 1 node's settings
    for settings in ('{"angle": 0.3}', '{"angle_strategy": "optimal"}', '{"angle": null}'):
        monkeypatch.setenv('PLANNER_SETTINGS', settings)
        main()
        assert clients[-1].on_message.__self__.service.own_angle

    for bad in ('{"turn_radius": -1}', '{"no_such_setting": 1}', '[1]', '{'):
        monkeypatch.setenv('PLANNER_SETTINGS', bad)
        with pytest.raises(SystemExit):
            main()


# over a real broker -------------------------------------------------------------------------------------


def broker():
    host, _, port = os.environ.get('MOWBITE_TEST_MQTT', '127.0.0.1:1883').partition(':')
    try:
        socket.create_connection((host, int(port or 1883)), timeout=0.5).close()
    except OSError:
        return None
    return host, int(port or 1883)


def test_over_a_broker():
    mqtt = pytest.importorskip('paho.mqtt.client')
    address = broker()
    if address is None:
        pytest.skip('no MQTT broker (MOWBITE_TEST_MQTT=host:port)')
    prefix = f'mowbite-test-{uuid.uuid4().hex[:8]}/'

    def client():
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) if hasattr(mqtt, 'CallbackAPIVersion') else mqtt.Client()

    planner, mower = client(), client()
    subscribed = threading.Semaphore(0)
    for c in (planner, mower):
        c.on_subscribe = lambda *args: subscribed.release()
    MqttBridge(PlannerService(), planner, prefix).start()  # subscribes once connected
    answers, got = [], threading.Event()
    mower.on_message = lambda c, u, msg: (answers.append(json.loads(msg.payload)), got.set())
    for c in (planner, mower):
        c.connect(*address)
        c.loop_start()
    mower.subscribe(prefix + 'rpc/response', 1)
    assert subscribed.acquire(timeout=5) and subscribed.acquire(timeout=5)
    try:
        mower.publish(prefix + 'map/json', json.dumps(MAP), qos=1, retain=True).wait_for_publish()
        request = {'jsonrpc': '2.0', 'id': 'a', 'method': 'ext.mowbite_planner.plan', 'params': {'area_id': 'back'}}
        for _ in range(50):  # until the map is there
            mower.publish(prefix + 'rpc/request', json.dumps(request), qos=1)
            assert got.wait(5)
            if 'result' in answers[-1]:
                break
            got.clear()
            threading.Event().wait(0.1)
        assert answers[-1]['id'] == 'a' and answers[-1]['result']['area_id'] == 'back'
    finally:
        mower.publish(prefix + 'map/json', b'', qos=1, retain=True).wait_for_publish()  # clear the retained map
        for c in (mower, planner):
            c.disconnect()
            c.loop_stop()
