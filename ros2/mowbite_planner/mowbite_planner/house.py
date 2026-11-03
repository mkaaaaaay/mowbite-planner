"""The house computer's side of remote_planning (remote.py): plans what the mower's planner node asks, with the code
the mower runs, on the same stack, over the mower's broker. No ROS: Python, shapely and numpy, in a container of its own
(house/: the Dockerfile and compose.yaml; its watcher fetches the code the mower runs and starts this with it).

  python3 -m mowbite_planner.house   MOWER_MQTT_HOST (the mower), MOWER_MQTT_PORT (1883), MOWER_MQTT_USER,
                                     MOWER_MQTT_PASSWORD, HOUSE_WORKERS (processes that plan, planner.set_workers: 3)

It says hello (retained, every remote.HELLO_EVERY s, emptied when it's gone: its code, stack, arch, whether it's busy).
A request (remote.request_json's form with its id, key, code and stack) is planned only with the same code and stack
and where the key worked out from what it read is the mower's; else the answer says why (and the mower plans itself).
One request at a time, in the order they came; one the mower gave up on (its wait_s over) is dropped. Its progress
every PROGRESS_EVERY s (waiting ones too), then the answer: a header (answer/<id>) and the plan packed in parts
(answer/<id>/<part>), or the header with the error (refused: the planner's no to the request)."""

import hashlib
import json
import logging
import os
import platform
import re
import signal
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .config import PlanningError
from .export import MAX_ANSWER
from .planner import plan_coverage, set_workers
from .remote import (HELLO_EVERY, MAX_PACKED, PART, TOPIC, RemoteError, code_version, loads, pack, plan_json,
                     plan_key, request_from, request_json, stack)

log = logging.getLogger('mowbite_planner.house')

# s between progress messages, requests remembered (a message the broker sent again is no new one)
PROGRESS_EVERY = 1.0
SEEN = 64


class House:
    """The service without its transport: publish(topic, payload, retain) sends. on_request takes a request, work()
    plans them one after the other (one(), on a thread of its own, the only one that plans: GEOS), tick() sends the
    progress of the one planned and of those waiting."""

    def __init__(self, publish: Callable[[str, Any, bool], None], code: Optional[str] = None) -> None:
        self.publish = publish
        self.code, self.stack, self.arch = code or code_version(), stack(), platform.machine()
        self.since = time.time()
        self.cond = threading.Condition()
        self.waiting: List[Dict[str, Any]] = []
        self.planning: Optional[Dict[str, Any]] = None
        self.seen: List[str] = []

    def hello(self) -> None:
        with self.cond:
            busy = self.planning is not None or bool(self.waiting)
        self.publish(TOPIC + 'hello', json.dumps({'code': self.code, 'stack': self.stack, 'arch': self.arch,
                                                  'busy': busy, 'since': round(self.since), 'at': time.time()}), True)

    def on_request(self, payload: bytes) -> None:
        """A request: waits for its turn (one not read is dropped, its id and key can't be told)."""
        try:
            request = loads(payload) if len(payload) <= MAX_ANSWER else None
            if not isinstance(request, dict) or not isinstance(request.get('id'), str) or \
                    not re.fullmatch(r'[0-9a-f]{32}', request['id']) or not isinstance(request.get('key'), str) or \
                    not re.fullmatch(r'[0-9a-f]{40}', request['key']):
                raise RemoteError('no request')
        except RemoteError as e:
            log.warning('a request not taken: %s', e)
            return
        wait = request.get('wait_s')
        until = time.monotonic() + (wait if isinstance(wait, (int, float)) and not isinstance(wait, bool) else 0.0)
        with self.cond:
            if request['id'] in self.seen:
                return
            self.seen = (self.seen + [request['id']])[-SEEN:]
            self.waiting.append({'request': request, 'until': until, 'stage': 'waiting', 'done': 0, 'of': 1})
            self.cond.notify_all()
        self.tick()

    def work(self) -> None:
        while True:
            self.one()

    def one(self) -> None:
        """The next request (waited for) planned and answered, dropped where the mower gave up on it meanwhile."""
        with self.cond:
            while not self.waiting:
                self.cond.wait()
            job = self.waiting.pop(0)
            if time.monotonic() > job['until']:
                log.info('request %s dropped: the mower planned it itself meanwhile', job['request']['id'])
                return
            self.planning = job
        try:
            self.answer(job)
        except Exception:  # noqa: BLE001 - the next request still gets its answer
            log.exception('request %s failed', job['request']['id'])
        finally:
            with self.cond:
                self.planning = None

    def tick(self) -> None:
        """The progress of the request planned and of those waiting."""
        with self.cond:
            jobs = ([self.planning] if self.planning is not None else []) + list(self.waiting)
            sent = [(job['request']['id'], {'stage': job['stage'], 'done': job['done'], 'of': job['of']})
                    for job in jobs]
        for rid, progress in sent:
            self.publish(TOPIC + 'progress/' + rid, json.dumps(progress), False)

    def answer(self, job: Dict[str, Any]) -> None:
        """Plans the request where it's to be planned here and answers it."""
        request = job['request']
        rid, key = request['id'], request['key']

        def say(ok: bool, **more: Any) -> None:
            self.publish(TOPIC + 'answer/' + rid, json.dumps(dict(id=rid, key=key, code=self.code, ok=ok, **more)),
                         False)

        if request.get('code') != self.code:
            return say(False, error='the house computer plans with another code')
        if request.get('stack') != self.stack:
            return say(False, error='the house computer plans on another stack')
        try:
            args, reach_over, unmowed = request_from(request.get('form'))
        except RemoteError as e:
            return say(False, error='request not read: %s' % e)
        if plan_key(self.code, request_json(args, reach_over, unmowed)) != key:
            return say(False, error='the key of what it read isn\'t the request\'s')

        def step(stage: str, done: int, of: int) -> None:
            with self.cond:
                job.update(stage=stage, done=done, of=of)

        t0 = time.perf_counter()
        try:
            plan = plan_coverage(*args, reach_over=reach_over, unmowed=unmowed, progress=step)
        except PlanningError as e:
            return say(False, error=str(e), refused=True)
        except Exception as e:  # noqa: BLE001 - the mower plans it itself then
            log.exception('request %s: planning failed', rid)
            return say(False, error='planning failed: %s: %s' % (type(e).__name__, e))
        try:
            data = pack(plan_json(plan, args[2]))
        except (ValueError, PlanningError) as e:
            return say(False, error='plan not sendable: %s' % e)
        if len(data) > MAX_PACKED:
            return say(False, error='plan too big: %d bytes' % len(data))
        parts = [data[i:i + PART] for i in range(0, len(data), PART)]
        say(True, parts=len(parts), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
        for k, part in enumerate(parts):
            self.publish(TOPIC + 'answer/%s/%d' % (rid, k), part, False)
        log.info('request %s planned in %.1f s, %d poses, %d bytes', rid, time.perf_counter() - t0, len(plan.poses),
                 len(data))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    env = os.environ
    import paho.mqtt.client as mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) if hasattr(mqtt, 'CallbackAPIVersion') else mqtt.Client()
    if env.get('MOWER_MQTT_USER'):
        client.username_pw_set(env['MOWER_MQTT_USER'], env.get('MOWER_MQTT_PASSWORD') or None)
    client.will_set(TOPIC + 'hello', b'', qos=1, retain=True)
    house = House(lambda topic, payload, retain: client.publish(topic, payload, qos=1, retain=retain))
    set_workers(int(env.get('HOUSE_WORKERS') or 3))

    def on_connect(client: Any, userdata: Any, flags: Any, rc: Any, properties: Any = None) -> None:
        if rc != 0:
            log.warning('MQTT connection refused: %s', rc)
            return
        client.subscribe(TOPIC + 'request', 1)
        house.hello()

    def every(seconds: float, fn: Callable[[], None]) -> None:
        def loop() -> None:
            while True:
                time.sleep(seconds)
                fn()
        threading.Thread(target=loop, daemon=True).start()

    client.on_connect = on_connect
    client.on_message = lambda client, userdata, msg: house.on_request(bytes(msg.payload))
    threading.Thread(target=house.work, name='planner', daemon=True).start()
    every(PROGRESS_EVERY, house.tick)
    every(HELLO_EVERY, house.hello)

    def stop(*_: Any) -> None:  # (docker stop: gone at once, not after the keepalive)
        client.publish(TOPIC + 'hello', b'', qos=1, retain=True)
        client.disconnect()

    signal.signal(signal.SIGTERM, stop)
    host, port = env.get('MOWER_MQTT_HOST') or '127.0.0.1', int(env.get('MOWER_MQTT_PORT') or 1883)
    log.info('MowBite Planner house computer, code %s, %s, broker %s:%d', house.code[:12], house.stack, host, port)
    client.connect_async(host, port, keepalive=30)
    try:
        client.loop_forever(retry_first_connection=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
