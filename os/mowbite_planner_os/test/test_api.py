"""The planner's part of MowBite OS' app interface (api/planner.schema.json, AM6): its form, where it's installed, the
messages the node sends kept to it (every one in test_os_node.py too)."""

import json
import os
import time

from osfake import HERE, REAL_ROS

import pytest

import apicheck  # noqa: E402
from mowbite_planner import core as core_module  # noqa: E402
from mowbite_planner.config import PlanningError  # noqa: E402
from test_os_node import make, planned, plan_area, rpc, send_map, the_map  # noqa: E402,F401


def test_the_form():
    """JSON Schema 2020-12 with version (an integer), events (none yet), topics (mowbite/planner/progress) and
    x-reasons; no calib on MowBite OS. mowbite_api sets the envelope (id, t, type)."""
    part = apicheck.part()
    assert part['$schema'] == 'https://json-schema.org/draft/2020-12/schema'
    assert type(part['version']) is int and part['version'] >= 1
    assert part['events'] == {} and part['x-reasons'] == {}
    assert list(part['topics']) == [core_module.PROGRESS_TOPIC]
    assert not [t for t in part['topics'] if 'calib' in t]
    for name, schema in part['topics'].items():
        assert schema['type'] == 'object' and set(schema['required']) == set(schema['properties']), name
    with open(os.path.join(HERE, '..', 'setup.py')) as f:
        assert "('share/' + package_name + '/api', ['api/planner.schema.json'])," in f.read()


def test_the_check():
    schema = apicheck.part()['topics'][core_module.PROGRESS_TOPIC]
    assert apicheck.problems({'area_id': 'a'}, schema)
    assert apicheck.sent('mowbite/planner/calib', '{}') == ["mowbite/planner/calib: no topic of the planner's part"]
    good = {'area_id': 'a', 'key': 'k', 'running': False, 'stage': 'done', 'done': 1000, 'of': 1000, 'started': 1.0,
            'elapsed_s': 0.5, 'estimate_s': None, 'error': None, 'waiting_n': 1,
            'waiting': [{'area_id': None, 'job': 'plan', 'rank': 'ahead', 'ids': ['r1']}], 'planned_on': 'mower',
            'remote_why': None}
    assert apicheck.problems(good, schema) == []
    for change, where in (({'done': -1}, '/done'), ({'running': 1}, '/running'), ({'planned_on': 'pc'}, '/planned_on'),
                          ({'more': 1}, '/more'), ({'waiting': [{'job': 'plan'}]}, '/waiting/0/area_id')):
        assert any(p.startswith(where) for p in apicheck.problems(dict(good, **change), schema)), change


@pytest.mark.skipif(REAL_ROS, reason='looks at the stand-ins\' topics')
def test_what_the_node_sends(make, monkeypatch):  # noqa: F811
    """A planning's progress as it runs, ends well or fails, and what waits while nothing's planned: each message as
    the part says."""
    monkeypatch.setattr(core_module, 'PROGRESS_EVERY', 0.05)
    node = make()
    send_map(node, the_map())
    assert rpc(node, 'planner.plan', {'area_id': 'a'})[1] is None
    real = core_module.plan_coverage

    def failing(*args, **kw):
        time.sleep(0.2)
        raise PlanningError('no room')
    monkeypatch.setattr(core_module, 'plan_coverage', failing)
    send_map(node, the_map(planner={'turn_radius': 0.3}))
    assert rpc(node, 'planner.plan', {'area_id': 'a'}, 'r2')[1] == (-32602, 'no room')
    monkeypatch.setattr(core_module, 'plan_coverage', real)
    node.core.progress.queue_changed()
    time.sleep(0.3)
    out = [json.loads(m.payload) for m in node.publishers['bridge/publish'].published
           if m.topic == core_module.PROGRESS_TOPIC]
    assert {m['running'] for m in out} == {True, False} and any(m['error'] == 'no room' for m in out)
    assert any(m['stage'] == 'done' for m in out)
    for m in node.publishers['bridge/publish'].published:
        assert apicheck.sent(m.topic, m.payload) == [], m.payload
