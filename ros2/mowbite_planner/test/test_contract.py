"""contract.json as the code has it, interface one up with the services, requires met where the tests run."""
import os
import platform
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', 'tools')]

import contract  # noqa: E402


def version(text):
    return tuple(int(n) for n in re.findall(r'\d+', text)[:3])


def test_contract_is_the_codes():
    before = contract.read()
    assert contract.text(contract.make(before)) == contract.text(before), \
        'contract.json is not the code\'s: tools/contract.py --write (a service changed: interface one up)'


def test_interface_up_with_a_service_changed():
    before = contract.read()
    assert contract.make(before)['interface'] == before['interface']
    changed = dict(before, interfaces=dict(before['interfaces'], **{'mowbite_planner_interfaces/srv/PlanCoverage': ''}))
    assert contract.make(changed)['interface'] == before['interface'] + 1
    assert contract.make(None)['interface'] == 1


def test_comments_not_counted(tmp_path, monkeypatch):
    srv = tmp_path / 'a' / 'srv'
    srv.mkdir(parents=True)
    monkeypatch.setattr(contract, 'ROOT', str(tmp_path))
    monkeypatch.setattr(contract, 'INTERFACES', ('a',))
    (srv / 'S.srv').write_text('# x\nfloat64 a -1.0\n---\nbool ok\n')
    first = contract.digests()
    (srv / 'S.srv').write_text('# y\n\nfloat64  a -1.0  # m\n---\nbool ok\n')
    assert contract.digests() == first
    (srv / 'S.srv').write_text('float64 a 0.0\n---\nbool ok\n')
    assert contract.digests() != first


def test_requires_met():
    import numpy
    import shapely
    have = {'numpy': numpy.__version__, 'python': platform.python_version(), 'shapely': shapely.__version__}
    for name, least in contract.read()['requires'].items():
        assert version(have[name]) >= version(least), (name, have[name], least)
