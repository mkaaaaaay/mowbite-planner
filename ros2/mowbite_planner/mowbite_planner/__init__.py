"""MowBite Planner: coverage paths for robot mowers (perimeter loops, boustrophedon lanes, gentle turns).

planner.py and the modules it uses need only shapely, node.py wraps them as a ROS 2 node.
"""

import itertools

from .config import CoveragePlan, NothingLeft, PlannerConfig, PlanningError, SegmentInfo
from .mathutil import Pose2D, normalize_angle, quaternion_to_yaw, yaw_to_quaternion
from .planner import plan_coverage
from .sweep import swept_area

# also in setup.py and package.xml
__version__ = '0.5.0'
# the files kept next to the settings file (it, mowbite_planner_angle.json, mowbite_planner_timing.json,
# mowbite_planner_took.json, mowbite_map_backups; not the plans, kept by the code that planned them): one more where a planner before can't read
# them (MowBite OS keeps data_dir for the image it came from)
FILES_VERSION = 1
# the least it runs with: the oldest the tests run on (OpenMower's Python 3.8 and numpy, MowBite OS' shapely)
REQUIRES = {'numpy': '1.17.4', 'python': '3.8', 'shapely': '2.0.3'}


def unmet(found=None) -> str:
    """Why the planner doesn't run here: what's older than REQUIRES (found: {name: version}, by default what's
    installed, remote.stack), '' where nothing is. The nodes end with it at their start."""
    if found is None:
        from .remote import stack
        found = stack()
    old = []
    for name, least in sorted(REQUIRES.items()):
        got = str(found.get(name) or '')
        have, need = _numbers(got), _numbers(least)
        width = max(len(have), len(need))
        if have + (0,) * (width - len(have)) < need + (0,) * (width - len(need)):
            old.append(f'{name} {got or "missing"} (needs {least} or newer)')
    return 'MowBite Planner: too old here: ' + ', '.join(old) if old else ''


def _numbers(version: str) -> tuple:
    """A version's leading numbers: '2.0.3rc1' (2, 0, 3), '1.26.4+dfsg' (1, 26, 4), '' ()."""
    out = []
    for part in version.split('.'):
        digits = ''.join(itertools.takewhile(str.isdigit, part))
        if not digits:
            break
        out.append(int(digits))
    return tuple(out)


__all__ = [
    '__version__',
    'FILES_VERSION',
    'CoveragePlan',
    'NothingLeft',
    'PlannerConfig',
    'PlanningError',
    'Pose2D',
    'SegmentInfo',
    'normalize_angle',
    'plan_coverage',
    'quaternion_to_yaw',
    'swept_area',
    'unmet',
    'yaw_to_quaternion',
]
