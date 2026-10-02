"""The MowBite Planner as a JSON-RPC 2.0 service over MQTT, for OpenMower as it is today (ROS 1 on OpenMower
OS): requests on <prefix>rpc/request, answers on <prefix>rpc/response. OpenMower's xbot_monitoring leaves the
methods starting with ext. to other services, these are this one's:

  ext.mowbite_planner.plan  an area of the mower's map by its id, or an outline, like OpenMower's mowing.plan
  ext.planner.plan          the request and answer of OpenMower's Planner Studio (xtech/openmower-app)
  ext.mowbite_planner.info  version, the settings and their defaults, whether map and parameters are in

The mower's map (map/json) and its ROS parameters (params/json) are retained topics, the latest are kept.

Environment: MOWER_MQTT_HOST (127.0.0.1), MOWER_MQTT_PORT (1883), MOWER_MQTT_USER, MOWER_MQTT_PASSWORD,
MOWER_MQTT_PREFIX (''), PLANNER_SETTINGS (a JSON object of planner settings, e.g. {"turn_radius": 0.3}, like
the ROS 1 node's settings: with angle or angle_strategy in it the lanes go that way, unless the area has an
angle of its own). paho-mqtt (1.6 or 2.x) is only needed by main().
"""

import dataclasses
import json
import logging
import math
import os
import queue
import signal
import struct
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

from .export import to_openmower_json, to_poses_json, to_studio_payload, to_trace_json
from .mapfile import AreaJob, _number, area_job, area_settings, settings_for
from .planner import PlannerConfig, PlanningError, plan_coverage
from .report import check_plan

log = logging.getLogger('mowbite_planner')

XY = Tuple[float, float]
TOPICS = ('rpc/request', 'map/json', 'params/json')
# what a plan request carries like the properties of an area in map.json, they replace the area's
AREA_PROPERTIES = ('angle', 'outline_count', 'outline_overlap_count', 'outline_offset', 'angle_min', 'angle_max')
# Planner Studio's names for the planner's settings
STUDIO_NAMES = {'tool_width': 'mower_width', 'headland_rounds': 'perimeter_passes',
                'min_turning_radius': 'turn_radius', 'ring_order': 'perimeter_order',
                'disk_lateral_offset': 'blade_offset', 'transit_wall_fade': 'transit_edge_distance',
                'transit_wall_risk': 'transit_edge_cost', 'merge_max_gap': 'bend_max_gap',
                'bend_max_dev': 'bend_max_offset'}
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


def _version() -> str:
    from . import __version__
    return __version__


def _point(p: Any) -> Tuple[float, ...]:
    """[x, y], {x, y} or with a heading ([x, y, yaw], {x, y, heading}), finite numbers."""
    if isinstance(p, Mapping):
        p = [p.get('x'), p.get('y')] + ([p['heading']] if p.get('heading') is not None else [])
    out = tuple(map(_number, p)) if isinstance(p, (list, tuple)) and 2 <= len(p) <= 3 else (None,)
    if None in out:
        raise ValueError(p)
    return out


def _ring(value: Any, what: str) -> List[XY]:
    try:
        return [_point(p)[:2] for p in value]
    except (TypeError, ValueError):  # TypeError: not a list
        raise PlanningError(f'{what} must be a list of points, [x, y] or {{"x": x, "y": y}}') from None


def _float32(ring: List[XY]) -> List[XY]:
    """A ring as geometry_msgs/Point32 carries it. One too far out for float32 stays, the planner refuses it."""
    try:
        return [struct.unpack('<2f', struct.pack('<2f', x, y)) for x, y in ring]
    except OverflowError:
        return ring


def _rings(value: Any, what: str) -> List[List[XY]]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise PlanningError(f'{what} must be a list of point lists')
    return [_ring(r, f'{what}[{i}]') for i, r in enumerate(value)]


def _pose(value: Any, what: str) -> Optional[Tuple[float, ...]]:
    if value is None:
        return None
    try:
        return _point(value)
    except ValueError:
        raise PlanningError(f'{what} must be [x, y] or {{"x": x, "y": y}}, a heading may follow') from None


def _object(value: Any, what: str) -> Mapping[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise PlanningError(f'{what} must be an object')
    return value


def _finite(v: Any) -> Any:
    """v with NaN and infinity as None (JSON has neither), tuples as lists."""
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, Mapping):
        return {k: _finite(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_finite(x) for x in v]
    return v


def _error(rid: Any, code: int, message: str) -> Dict[str, Any]:
    return {'jsonrpc': '2.0', 'id': rid, 'error': {'code': code, 'message': message}}


def _retained(payload: Any, topic: str) -> Optional[Mapping[str, Any]]:
    """A retained JSON object, None when it was cleared (empty payload) or isn't one."""
    if not payload:
        return None
    try:
        data = json.loads(payload) if isinstance(payload, (bytes, str)) else payload
    except ValueError:
        data = None
    if not isinstance(data, Mapping):
        log.warning('%s is not a JSON object, ignored', topic)
        return None
    return data


class PlannerService:
    """Answers the JSON-RPC requests, without any transport: handle() gives the response, None for what
    isn't this service's to answer (other methods, notifications, and what xbot_monitoring refuses itself:
    an id that isn't a string, no jsonrpc 2.0). own_angle: base has an angle or angle_strategy (see
    settings_for)."""

    def __init__(self, base: Optional[PlannerConfig] = None, own_angle: bool = False):
        self.base = base or PlannerConfig()
        self.own_angle = own_angle
        self.map: Optional[Mapping[str, Any]] = None
        self.params: Optional[Mapping[str, Any]] = None
        self.methods: Dict[str, Callable[[Mapping[str, Any]], Any]] = {
            'ext.mowbite_planner.plan': self.plan, 'ext.planner.plan': self.studio_plan,
            'ext.mowbite_planner.info': self.info}

    def on_map(self, payload: Any) -> None:
        self.map = _retained(payload, 'map/json')

    def on_params(self, payload: Any) -> None:
        self.params = _retained(payload, 'params/json')

    def handle(self, request: Any) -> Optional[Dict[str, Any]]:
        method = request.get('method') if isinstance(request, Mapping) else None
        if (not isinstance(method, str) or method not in self.methods or not isinstance(request.get('id'), str)
                or request.get('jsonrpc') != '2.0'):
            return None
        rid, t0 = request['id'], time.perf_counter()
        try:
            result = _finite(self.methods[method](_object(request.get('params'), 'params (by name)')))
        except PlanningError as e:
            log.info('%s %s: %s', method, rid, e)
            return _error(rid, INVALID_PARAMS, str(e))
        except Exception as e:
            log.exception('%s %s failed', method, rid)
            return _error(rid, INTERNAL_ERROR, f'internal error: {e}')
        log.info('%s %s: %.2f s', method, rid, time.perf_counter() - t0)
        return {'jsonrpc': '2.0', 'id': rid, 'result': result}

    def plan(self, p: Mapping[str, Any]) -> Dict[str, Any]:
        """ext.mowbite_planner.plan: {area_id} of the mower's map or {outline, obstacles, nav_areas}, angle,
        outline_count, ... in place of the area's properties (like OpenMower's mowing.plan), settings (any
        planner setting), start, end, poses, report (check_plan), trace (stages and lineage). Planned with the
        settings the ROS 1 node would use."""
        props = {k: p[k] for k in AREA_PROPERTIES if k in p}
        area_id, start, end = None, None, None
        if p.get('area_id') is not None:
            data = self.map  # the network thread may replace it meanwhile
            if data is None:
                raise PlanningError('no map from the mower yet (map/json)')
            # like the mower: only the other lawns for ways between the parts, a path is mowed from start to end
            job = area_job(data, p['area_id'], props)
            area_id = job.area_id
            if job.dock:  # from the dock and back
                start, end = job.dock, job.dock[:2]
        else:  # an area of its own, like mowing.plan plans an outline
            area = {'properties': props}
            job = AreaJob(area_id='', name='', boundary=_ring(p.get('outline'), 'outline'),
                          obstacles=_rings(p.get('obstacles'), 'obstacles'), mow_around=[],
                          drivable=_rings(p.get('nav_areas'), 'nav_areas'), nav=[], overrides=area_settings(area),
                          planner=area_settings(area, openmower=False))
        # mower_logic hands the node the outline and the obstacles as float32, the areas to mow around, the
        # other lawns and the dock the node reads from the map
        job = dataclasses.replace(job, boundary=_float32(job.boundary), obstacles=[
            o if any(o is m for m in job.mow_around) else _float32(o) for o in job.obstacles])
        settings = dict(_object(p.get('settings'), 'settings'))
        if 'angle_strategy' in settings:
            settings.setdefault('angle', None)  # the strategy works it out, not the area's angle
        cfg = settings_for(self.base, self.params, job, settings, self.own_angle)
        start = _pose(p['start'], 'start') if p.get('start') is not None else start
        end = _pose(p['end'], 'end') if p.get('end') is not None else end
        obstacles = job.obstacles + job.nested if cfg.nested_areas else job.obstacles
        plan = plan_coverage(job.boundary, obstacles, cfg, start, end, job.drivable, job.keep_out)
        result = to_openmower_json(plan, area_id)
        if p.get('poses'):
            result.update(to_poses_json(plan))
        if p.get('report'):  # what's wrong with it, what's worth knowing, how much is mowed
            result['report'] = check_plan(plan)
        if p.get('trace'):  # where everything comes from, stage by stage
            result.update(to_trace_json(plan))
        return result

    def studio_plan(self, p: Mapping[str, Any]) -> Dict[str, Any]:
        """ext.planner.plan: Planner Studio's {area, obstacles, nav_areas, params}. Its parameters are
        mapped onto the planner's settings, the ones the planner doesn't know are listed as not used."""
        area = _ring(p.get('area'), 'area')
        obstacles, nav = _rings(p.get('obstacles'), 'obstacles'), _rings(p.get('nav_areas'), 'nav_areas')
        params = _object(p.get('params'), 'params')
        settings: Dict[str, Any] = {}
        ends: Dict[str, Any] = {}
        ignored = []
        for name, value in params.items():
            if name == 'angle_deg':  # null: worked out
                deg = _number(value)
                settings['angle'] = None if deg is None else math.radians(deg)
            elif name in ('start_point', 'end_point'):
                ends[name] = _pose(value, name)
            elif name == 'order':  # Planner Studio's route solver
                settings['route_order'] = {'greedy': 'nearest', 'ortools': 'optimized'}.get(value, value)
            elif name in STUDIO_NAMES or name in PlannerConfig.names():
                settings[STUDIO_NAMES.get(name, name)] = value
            else:
                ignored.append(name)
        cfg = self.base.with_overrides(settings)
        plan = plan_coverage(area, obstacles, cfg, ends.get('start_point'), ends.get('end_point'), nav)
        return to_studio_payload(plan, area, obstacles, nav, params, ignored)

    def info(self, p: Mapping[str, Any]) -> Dict[str, Any]:
        """ext.mowbite_planner.info: what this service is, its settings, what it got from the mower."""
        return {'name': 'mowbite_planner', 'version': _version(), 'methods': sorted(self.methods),
                'settings': {name: getattr(self.base, name) for name in PlannerConfig.names()},
                'map': self.map is not None, 'params': self.params is not None}


class MqttBridge:
    """The service on a paho-mqtt client (1.6 or 2.x). Requests are planned one after another in a worker
    thread, the network loop keeps the connection alive meanwhile."""

    def __init__(self, service: PlannerService, client: Any, prefix: str = ''):
        self.service, self.client = service, client
        self.prefix = prefix + '/' if prefix and not prefix.endswith('/') else prefix  # like OpenMower
        self.requests: 'queue.Queue[bytes]' = queue.Queue()
        client.on_connect = self.on_connect
        client.on_connect_fail = lambda *args: log.warning('MQTT broker not reachable, trying again')
        client.on_message = self.on_message

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self.work, name='planner', daemon=True)
        thread.start()
        return thread

    def on_connect(self, client: Any, userdata: Any, flags: Any, rc: Any, properties: Any = None) -> None:
        if rc != 0:  # paho 2.x gives a ReasonCode, it compares with ints too
            log.warning('MQTT connection refused: %s', rc)
            return
        log.info('connected, answering on %srpc/request', self.prefix)
        client.subscribe([(self.prefix + topic, 1) for topic in TOPICS])  # again after every reconnect

    def on_message(self, client: Any, userdata: Any, msg: Any) -> None:
        topic = msg.topic[len(self.prefix):]
        if topic == 'rpc/request':
            self.requests.put(msg.payload)
        elif topic == 'map/json':
            self.service.on_map(msg.payload)
        elif topic == 'params/json':
            self.service.on_params(msg.payload)

    def work(self) -> None:
        while True:
            payload = self.requests.get()
            try:
                self.answer(payload)
            except Exception:
                log.exception('answering a request failed')
            finally:
                self.requests.task_done()

    def answer(self, payload: bytes) -> None:
        try:
            request = json.loads(payload)
        except ValueError:
            return  # xbot_monitoring answers that one
        response = self.service.handle(request)
        if response is None:
            return
        try:
            data = json.dumps(response, allow_nan=False)
        except (TypeError, ValueError) as e:
            log.exception('response of %s is not JSON', request.get('method'))
            data = json.dumps(_error(response.get('id'), INTERNAL_ERROR, f'internal error: {e}'))
        self.client.publish(self.prefix + 'rpc/response', data)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    env = os.environ
    try:
        settings = _object(json.loads(env.get('PLANNER_SETTINGS') or '{}'), 'PLANNER_SETTINGS')
        base = PlannerConfig().with_overrides(settings)
        base.validate()
    except ValueError as e:  # bad JSON or a bad setting
        raise SystemExit(f'PLANNER_SETTINGS: {e}')
    import paho.mqtt.client as mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) if hasattr(mqtt, 'CallbackAPIVersion') else mqtt.Client()
    if env.get('MOWER_MQTT_USER'):
        client.username_pw_set(env['MOWER_MQTT_USER'], env.get('MOWER_MQTT_PASSWORD') or None)
    service = PlannerService(base, 'angle' in settings or 'angle_strategy' in settings)
    MqttBridge(service, client, env.get('MOWER_MQTT_PREFIX', '')).start()
    signal.signal(signal.SIGTERM, lambda *_: client.disconnect())  # docker stop, as PID 1
    host, port = env.get('MOWER_MQTT_HOST') or '127.0.0.1', int(env.get('MOWER_MQTT_PORT') or 1883)
    log.info('MowBite Planner %s, MQTT broker %s:%d', _version(), host, port)
    client.connect_async(host, port, keepalive=60)
    try:
        client.loop_forever(retry_first_connection=True)  # reconnects on its own
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
