"""contract.json as the code has it, interface one up with the services, requires met where the tests run; in the
package and in share/."""
import json
import os
import platform
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', 'tools')]

import contract  # noqa: E402


def version(text):
    return tuple(int(n) for n in re.findall(r'\d+', text)[:3])


def test_contract_is_the_codes():
    before = contract.read()
    assert contract.text(contract.make(before)) == contract.text(before), \
        'contract.json is not the code\'s: tools/contract.py --write (a service changed: interface one up)'


@pytest.mark.skipif(not os.path.isdir(os.path.join(contract.ROOT, contract.OS)), reason='interface only with os/')
def test_interface_up_with_a_service_changed():
    before = contract.read()
    assert contract.make(before)['interface'] == before['interface']
    changed = dict(before, interfaces=dict(before['interfaces'], **{'mowbite_planner_interfaces/srv/PlanCoverage': ''}))
    assert contract.make(changed)['interface'] == before['interface'] + 1
    # (a word of a service's code more or less too)
    changed = dict(before, service_codes=dict(before['service_codes'], **{
        'mowbite_planner_os_interfaces/srv/PlanTransit': before['service_codes'][
            'mowbite_planner_os_interfaces/srv/PlanTransit'][1:]}))
    assert contract.make(changed)['interface'] == before['interface'] + 1
    assert contract.make(None)['interface'] == 1


@pytest.mark.skipif(not os.path.isdir(os.path.join(contract.ROOT, contract.OS)), reason='the services\' codes with os/')
def test_the_services_codes():
    """service_codes: the words each service's code says where it fails, the code's own lists."""
    codes = contract.service_codes()  # (os/'s package on the path from here on)
    from mowbite_planner.config import COVERAGE_CODES
    from mowbite_planner_os.transit_route import CODES
    assert contract.read()['service_codes'] == codes == {
        'mowbite_planner_interfaces/srv/PlanCoverage': sorted(COVERAGE_CODES),
        'mowbite_planner_os_interfaces/srv/PlanTransit': sorted(CODES)}
    assert contract.read()['interface'] >= 2  # (1: before code)


def test_no_interface_without_os(tmp_path, monkeypatch):
    monkeypatch.setattr(contract, 'ROOT', str(tmp_path))
    got = contract.make(contract.read())
    assert 'interface' not in got and 'service_codes' not in got and 'interfaces' not in contract.make(None)


def test_in_the_package_and_in_share():
    """One file, in the package (mowbite_planner/contract.json: package data, MowBite OS reads it there) and from
    there to share/mowbite_planner as before; where the package is installed (colcon) both are the source's."""
    assert os.path.samefile(contract.FILE, os.path.join(HERE, '..', 'mowbite_planner', 'contract.json'))
    assert not os.path.exists(os.path.join(HERE, '..', 'contract.json'))
    with open(os.path.join(HERE, '..', 'setup.py')) as f:
        setup = f.read()
    assert "package_data={package_name: ['contract.json']}," in setup
    assert "('share/' + package_name, ['package.xml', package_name + '/contract.json'])," in setup
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:  # (no ROS 2 here: the source's alone)
        return
    try:
        share = get_package_share_directory('mowbite_planner')
    except PackageNotFoundError:  # (not installed)
        return
    here = os.path.realpath(os.path.join(HERE, '..', 'mowbite_planner'))
    packaged = [os.path.join(p, 'mowbite_planner', 'contract.json') for p in sys.path
                if os.path.isfile(os.path.join(p, 'mowbite_planner', '__init__.py')) and
                os.path.realpath(os.path.join(p, 'mowbite_planner')) != here]
    assert packaged, 'mowbite_planner installed without its package'
    for path in [os.path.join(share, 'contract.json')] + packaged:
        with open(path) as f:
            assert json.load(f) == contract.read(), path


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
