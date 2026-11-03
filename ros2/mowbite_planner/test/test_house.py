"""The house computer (house.py, house/watch.py): it plans only a request of its own code and stack whose key it works
out itself, answers the plan in parts the mower reads back to the bit; its watcher runs it on the code the mower wants,
from the planner's repository only."""

import hashlib
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

import mowbite_planner  # noqa: E402
from mowbite_planner import PlannerConfig, plan_coverage, remote  # noqa: E402
from mowbite_planner.export import fingerprint  # noqa: E402
from mowbite_planner.house import House  # noqa: E402

LAWN = [(0.0, 0.0), (5.0, 0.0), (5.0, 4.0), (0.0, 4.0)]
TREE = [(2.5 + 0.4 * math.cos(2 * math.pi * i / 12), 2.0 + 0.4 * math.sin(2 * math.pi * i / 12)) for i in range(12)]
CFG = PlannerConfig(mower_width=0.18, perimeter_passes=3, robot_width=0.41, robot_front=0.43, robot_rear=0.18,
                    blade_ahead=0.185)
ARGS = (remote.canon(LAWN), [remote.canon(TREE)], CFG, None, None, (), (), (None,))


def a_request(rid='a' * 32, args=ARGS, **change):
    form = remote.request_json(args)
    request = dict(id=rid, key=remote.plan_key(remote.code_version(), form), code=remote.code_version(),
                   stack=remote.stack(), wait_s=60.0, form=form)
    return json.dumps(dict(request, **change)).encode()


def served(payload):
    """What the house publishes for a request, planned at once."""
    published = []
    house = House(lambda topic, data, retain: published.append((topic, data, retain)))
    house.on_request(payload)
    while house.waiting:
        house.one()
    return house, [(t[len(remote.TOPIC):], d) for t, d, _ in published if not t.startswith(remote.TOPIC + 'progress')]


def test_the_house_plans_what_the_mower_does():
    house, got = served(a_request())
    (topic, header), parts = got[0], got[1:]
    header = json.loads(header)
    assert topic == 'answer/' + 'a' * 32 and header['ok'] and header['parts'] == len(parts)
    data = b''.join(d for _, d in parts)
    assert len(data) == header['bytes'] and hashlib.sha256(data).hexdigest() == header['sha256']
    plan = remote.plan_from(remote.unpack(data), ARGS)
    assert fingerprint(plan) == fingerprint(plan_coverage(*ARGS)) and remote.refused(plan) == ''
    hellos = []
    House(lambda topic, data, retain: hellos.append((topic, json.loads(data), retain))).hello()
    (topic, hello, retain), = hellos
    assert topic == remote.TOPIC + 'hello' and retain and hello['code'] == house.code and \
        hello['stack'] == remote.stack() and hello['busy'] is False


@pytest.mark.parametrize('change, error', [
    (dict(code='0' * 40), 'the house computer plans with another code'),
    (dict(stack={'python': '2.7'}), 'the house computer plans on another stack'),
    (dict(key='0' * 40), 'the key of what it read isn\'t the request\'s'),
    (dict(form={'boundary': 'x'}), 'request not read: '),
])
def test_the_house_refuses(change, error):
    _, got = served(a_request(**change))
    (topic, header), = got
    assert json.loads(header)['ok'] is False and json.loads(header)['error'].startswith(error)


def test_the_planners_no_and_what_isnt_a_request():
    tiny = (remote.canon([(0.0, 0.0), (0.1, 0.0), (0.1, 0.1)]), [], CFG, None, None, (), (), ())
    _, got = served(a_request(args=tiny))
    header = json.loads(got[0][1])
    assert header['ok'] is False and header['refused'] is True
    for payload in (b'not json', json.dumps({'id': 'x', 'key': 'k'}).encode(), a_request(rid='A' * 32)):
        assert served(payload)[1] == []
    assert served(a_request(wait_s=-1.0))[1] == []  # (the mower planned it itself meanwhile)
    house = House(lambda topic, data, retain: None)
    house.on_request(a_request())
    house.on_request(a_request())
    assert len(house.waiting) == 1


# the watcher ------------------------------------------------------------------------------------------------------

def watch_module():
    spec = importlib.util.spec_from_file_location('watch', os.path.join(HERE, '..', '..', '..', 'house', 'watch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def upstream(tmp_path):
    """A repository like the planner's, two commits of its package: [(commit, digest of the package)]."""
    if shutil.which('git') is None:
        pytest.skip('no git')
    path = tmp_path / 'upstream'
    package = path / 'ros2' / 'mowbite_planner' / 'mowbite_planner'
    package.mkdir(parents=True)

    def git(*args):
        return subprocess.run(['git', '-C', str(path), '-c', 'user.name=t', '-c', 'user.email=t@t'] + list(args),
                              capture_output=True, check=True).stdout.decode().strip()

    git('init', '-q')
    commits = []
    for n in (1, 2):
        (package / '__init__.py').write_text('A = %d\n' % n)
        (package / 'house.py').write_text('B = 1\n')
        (package / 'notes.txt').write_text('not code\n')
        git('add', '-A')
        git('commit', '-q', '-m', 'c%d' % n)
        commits.append(git('rev-parse', 'HEAD'))
    return path, commits


def test_the_watcher_finds_the_code_wanted(upstream, tmp_path, monkeypatch):
    watch = watch_module()
    path, (first, second) = upstream
    repo = watch.Repo(str(tmp_path / 'repo.git'), str(path))
    repo.fetch()
    out = tmp_path / 'out'
    repo.export(first, str(out))
    code = watch.digest_of(str(out / 'mowbite_planner'))
    monkeypatch.setattr(mowbite_planner, '__path__', [str(out / 'mowbite_planner')])
    assert code == remote.code_version()  # (the digest the mower has)
    assert repo.find(code) == first and repo.find('0' * 40) is None
    started = []
    watcher = watch.Watcher(str(tmp_path / 'data'), repo, start=lambda folder: started.append(folder) or Proc())
    assert watch.wanted(b'') is None and watch.wanted(b'{"code": "x"}') is None
    watcher.on_want(json.dumps({'code': code, 'stack': {}}).encode())
    watcher.step(100.0)
    assert started == [str(tmp_path / 'data' / 'code' / code)] and watcher.running == code
    watcher.step(101.0)
    assert len(started) == 1
    watcher.proc.code = 1  # (it ended: again after RESTART s)
    watcher.step(102.0)
    watcher.step(102.0 + watch.RESTART)
    assert len(started) == 2
    watcher.on_want(json.dumps({'code': '0' * 40}).encode())  # (no commit has it: as it was)
    watcher.step(200.0)
    assert len(started) == 2 and watcher.running == code


class Proc:
    code = None

    def poll(self):
        return self.code

    def terminate(self):
        self.code = -15

    def wait(self, timeout=None):
        return self.code
