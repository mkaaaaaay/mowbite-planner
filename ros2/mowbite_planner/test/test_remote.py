"""Planning in the house (remote.py): the plan's form read back on the mower is the plan planned there, every field to
the bit, its config and geometry worked out on the mower; the readers refuse what isn't a request or a plan; the mower
refuses a plan that breaks a rule. The matrix does the same for every plan of its (test_invariants.py)."""

import dataclasses
import json
import math
import os
import sys
import zlib

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import PlannerConfig, Pose2D, plan_coverage, remote  # noqa: E402
from mowbite_planner.config import LANE, TRANSIT  # noqa: E402
from mowbite_planner.export import fingerprint  # noqa: E402
from mowbite_planner.remote import RemoteError  # noqa: E402

LAWN = [(0.0, 0.0), (5.0, 0.0), (5.0, 4.0), (0.0, 4.0)]
TREE = [(2.5 + 0.4 * math.cos(2 * math.pi * i / 12), 2.0 + 0.4 * math.sin(2 * math.pi * i / 12)) for i in range(12)]
BODY = dict(mower_width=0.18, perimeter_passes=3, robot_width=0.41, robot_front=0.43, robot_rear=0.18,
            blade_ahead=0.185)
CASES = {
    # the lane spacing auto took and the body turned round: config and geometry from two configs
    'reverse_auto': (dict(BODY, drive='reverse', lane_spacing_mode='auto'), {}),
    'radius0': (dict(BODY, turn_radius=0.0), {}),
    # no body: no body space, a drive beside the lawn with a part to keep out of
    'bare': (dict(mower_width=0.18, perimeter_passes=3),
             {'start': (5.5, 2.0, math.pi), 'end': (5.5, 2.0), 'drivable': [[(5, 1), (7, 1), (7, 3), (5, 3)]],
              'keep_out': [[(5, 2.5), (7, 2.5), (7, 3), (5, 3)]]}),
}
_plans = {}


def args_of(name):
    settings, more = CASES[name]
    return (remote.canon(LAWN), [remote.canon(TREE)], PlannerConfig(**settings), remote.canon(more.get('start')),
            remote.canon(more.get('end')), remote.canon(more.get('drivable', [])),
            remote.canon(more.get('keep_out', [])), (None,) * (1 + len(more.get('keep_out', []))))


def planned(name):
    if name not in _plans:
        _plans[name] = plan_coverage(*args_of(name))
    return _plans[name]


def read_back(name, form=None):
    plan = planned(name)
    form = form if form is not None else remote.unpack(remote.pack(remote.plan_json(plan, args_of(name)[2])))
    return remote.plan_from(form, args_of(name))


def wkb(geom):
    return None if geom is None else geom.wkb


@pytest.mark.parametrize('name', CASES)
def test_the_plan_read_back_is_the_plan(name):
    plan, got = planned(name), read_back(name)
    assert fingerprint(got) == fingerprint(plan)
    for f in dataclasses.fields(plan):
        a, b = getattr(plan, f.name), getattr(got, f.name)
        if f.name == 'levels':
            a, b = [wkb(g) for g in a], [wkb(g) for g in b]
        elif f.name not in remote.SENT + ('lane_spacing', 'config'):
            a, b = wkb(a), wkb(b)
        assert a == b, f.name
    assert remote.refused(got) == ''


def test_the_request_read_back_has_its_key():
    """The house works out the request's key from what it read: the mower's (the config's tuples and ints as they
    were); another code or setting, another key."""
    cfg = PlannerConfig(**dict(BODY, robot_outline=((0.43, -0.2), (0.43, 0.2), (-0.14, 0.2), (-0.14, -0.2)),
                               turn_types=('u_turn', 'bulb'), perimeter_passes=-1))
    args = (remote.canon(LAWN), [remote.canon(TREE)], cfg, (1.0, 1.0, 0.5), (1.0, 1.0), (), (), (0.2,))
    form = remote.request_json(args, (remote.canon(LAWN),), ())
    back = remote.request_from(json.loads(json.dumps(form)))
    assert back[0][2] == cfg and remote.plan_key('c', remote.request_json(*back)) == remote.plan_key('c', form)
    assert remote.plan_key('d', form) != remote.plan_key('c', form)
    other = remote.request_json((*args[:2], dataclasses.replace(cfg, turn_radius=0.3), *args[3:]))
    assert remote.plan_key('c', other) != remote.plan_key('c', remote.request_json(args))
    for bad in ({'config': {'nope': 1}}, {'config': {'turn_radius': [1]}}, {'boundary': [[0, 'x']]},
                {'margins': [True]}, {'start': [1, [2]]}):
        with pytest.raises(RemoteError):
            remote.request_from(dict(form, **bad))


def test_the_reader_is_strict(monkeypatch):
    good = remote.plan_json(planned('radius0'), args_of('radius0')[2])
    for text in (b'{"a": NaN}', b'{"a": Infinity}', b'{"a": 1e999}', b'[' * 100 + b']' * 100, b'{"a": 1',
                 b'{"a": 12345678901234567890}'):
        with pytest.raises(RemoteError):
            remote.unpack(zlib.compress(text))
    with pytest.raises(RemoteError):  # (not zlib, cut off)
        remote.unpack(b'not zlib')
    with pytest.raises(RemoteError):
        remote.unpack(remote.pack(good)[:-10])
    monkeypatch.setattr(remote, 'MAX_TEXT', 1000)  # (a small packet unpacking into a lot)
    with pytest.raises(RemoteError):
        remote.unpack(zlib.compress(b'[' + b'0,' * 100000 + b'0]'))
    monkeypatch.undo()
    for change in ({'format': 'other'}, {'spacing': 0.1}, {'poses': good['poses'][:-1]},
                   {'segments': [['nope', 0, 1, False, '', 0, '', False]]},
                   {'segments': [good['segments'][0][:7]]}, {'cells': [[0.0]]}, {'warnings': [1]},
                   {'stats': []}, {'section_starts': [-1]}):
        with pytest.raises(RemoteError):
            read_back('radius0', dict(good, **change))


def test_a_plan_breaking_a_rule_is_refused():
    """Its form (segments, sections, headings) and report.rules with the mower's geometry and body."""
    got = read_back('radius0')
    poses, segs = got.poses, got.segments
    lane = next(s for s in segs if s.kind == LANE)
    i = (lane.start + lane.end) // 2
    p = poses[i]
    out = list(poses)
    out[i] = Pose2D(p.x - 3.0 * math.cos(p.yaw + 0.5 * math.pi), p.y - 3.0 * math.sin(p.yaw + 0.5 * math.pi), p.yaw)
    for broken in (dict(poses=out), dict(poses=poses[:i] + [Pose2D(p.x, p.y, p.yaw + 0.5)] + poses[i + 1:]),
                   dict(section_starts=[0, i]), dict(segments=[segs[1], segs[0]] + segs[2:]),
                   dict(segments=[dataclasses.replace(s, reverse=True) for s in segs]),
                   dict(segments=[dataclasses.replace(s, kind=TRANSIT) for s in segs], poses=out),
                   dict(poses=poses[:i] + [poses[i - 1]] + poses[i + 1:])):
        assert remote.refused(dataclasses.replace(got, **broken)), broken.keys()
