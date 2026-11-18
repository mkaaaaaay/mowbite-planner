#!/usr/bin/env python3
"""The planner's contract.json (in the package, mowbite_planner/contract.json, and installed to share/mowbite_planner
from there), made from the code: what a robot and its apps may rely on. interface (only with os/, MowBite OS' update
reads it) counts up with every change of the services the OS builds (the .srv beside this package and os/'s,
comments not counted) or of the words their code says (service_codes); requires: the least Python and packages it
runs with; map_fields: the properties of a map's areas it reads that a robot or app writes (mapfile.MAP_FIELDS).

  contract.py           the contract as the code has it now
  contract.py --write   into contract.json, interface one up where the services changed
"""
import ast
import hashlib
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
ROOT = os.path.join(PKG, '..', '..')
FILE = os.path.join(PKG, 'mowbite_planner', 'contract.json')
sys.path.insert(0, PKG)
SCHEMA = 1
OS = 'os/mowbite_planner_os_interfaces'
INTERFACES = ('ros2/mowbite_planner_interfaces', OS)
# os/'s Python package (its ~/plan_transit's words: transit_route.CODES)
OS_PACKAGE = 'os/mowbite_planner_os'
# the robots' nodes: the planner.* methods they add to the core's
ROBOTS = {'openmower': 'ros1/mowbite_planner_ros1/nodes/mowbite_planner_node',
          'mowbite-os': 'os/mowbite_planner_os/mowbite_planner_os/node.py'}
# what a setting says that's the same on every robot
SETTING_KEYS = ('type', 'choices', 'unit', 'min', 'max', 'nullable')


def digests() -> dict:
    """{package/srv/Name: SHA-256 of its lines without comments} of the services there are."""
    out = {}
    for pkg in INTERFACES:
        for kind in ('msg', 'srv', 'action'):
            where = os.path.join(ROOT, pkg, kind)
            for name in sorted(os.listdir(where)) if os.path.isdir(where) else ():
                with open(os.path.join(where, name)) as f:
                    lines = [' '.join(re.sub(r'#.*', '', line).split()) for line in f]
                text = '\n'.join(line for line in lines if line)
                key = '%s/%s/%s' % (os.path.basename(pkg), kind, os.path.splitext(name)[0])
                out[key] = hashlib.sha256(text.encode()).hexdigest()
    return out


def service_codes() -> dict:
    """{package/srv/Name: the words its code says where it fails} as the code has them."""
    from mowbite_planner import config
    where = os.path.abspath(os.path.join(ROOT, OS_PACKAGE))
    if where not in sys.path:
        sys.path.insert(0, where)
    from mowbite_planner_os import transit_route
    return {'mowbite_planner_interfaces/srv/PlanCoverage': sorted(config.COVERAGE_CODES),
            'mowbite_planner_os_interfaces/srv/PlanTransit': sorted(transit_route.CODES),
            'mowbite_planner_os_interfaces/srv/CutPlan': sorted(transit_route.CUT_CODES)}


def added(path: str) -> set:
    """The planner.* keys of the dicts in the robot's node at path."""
    with open(path) as f:
        tree = ast.parse(f.read(), path)
    return {k.value for node in ast.walk(tree) if isinstance(node, ast.Dict) for k in node.keys
            if isinstance(k, ast.Constant) and isinstance(k.value, str) and k.value.startswith('planner.')}


def make(before=None) -> dict:
    """The contract, interface from before's (one up where the services or their codes' words aren't its)."""
    import mowbite_planner
    from mowbite_planner import core, export, mapfile
    planner = core.PlannerCore(0.18, {}, '', '', '')
    settings = {name: {k: entry[k] for k in SETTING_KEYS if k in entry}
                for name, entry in sorted(planner.rpc_settings()['settings'].items())}
    methods = {robot: sorted(set(planner.methods) | added(os.path.join(ROOT, path)))
               for robot, path in sorted(ROBOTS.items()) if os.path.exists(os.path.join(ROOT, path))}
    codes = {'ANSWER_TOO_BIG': export.ANSWER_TOO_BIG, 'INTERNAL': core.INTERNAL,
             'INVALID_PARAMS': core.INVALID_PARAMS, 'METHOD_NOT_FOUND': core.METHOD_NOT_FOUND,
             'SUPERSEDED': core.SUPERSEDED}
    out = {'schema': SCHEMA, 'owner': 'mowbite-planner', 'requires': dict(sorted(mowbite_planner.REQUIRES.items())),
           'files_version': mowbite_planner.FILES_VERSION, 'methods': methods, 'codes': codes,
           'topics': [core.PROGRESS_TOPIC], 'settings': settings, 'map_fields': list(mapfile.MAP_FIELDS)}
    if os.path.isdir(os.path.join(ROOT, OS)):
        services, words = digests(), service_codes()
        interface = (before or {}).get('interface', 0)
        changed = not before or before.get('interfaces') != services or before.get('service_codes') != words
        out.update(interface=interface + changed, interfaces=services, service_codes=words)
    return out


def text(contract: dict) -> str:
    return json.dumps(contract, indent=2, sort_keys=True) + '\n'


def read() -> dict:
    with open(FILE) as f:
        return json.load(f)


def main() -> int:
    before = read() if os.path.exists(FILE) else None
    out = text(make(before))
    if sys.argv[1:] == ['--write']:
        with open(FILE, 'w') as f:
            f.write(out)
    else:
        sys.stdout.write(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
