"""MowBite Planner: coverage paths for robot mowers (perimeter loops, boustrophedon lanes, gentle turns).

planner.py and the modules it uses need only shapely, node.py wraps them as a ROS 2 node.
"""

from .config import CoveragePlan, NothingLeft, PlannerConfig, PlanningError, SegmentInfo
from .mathutil import Pose2D, normalize_angle, quaternion_to_yaw, yaw_to_quaternion
from .planner import plan_coverage
from .sweep import swept_area

# also in setup.py and package.xml
__version__ = '0.5.0'
# the files kept next to the settings file (it, mowbite_planner_angle.json, mowbite_planner_timing.json,
# mowbite_map_backups; not the plans, kept by the code that planned them): one more where a planner before can't read
# them (MowBite OS keeps data_dir for the image it came from)
FILES_VERSION = 1
# the least it runs with: the oldest the tests run on (OpenMower's Python 3.8 and numpy, MowBite OS' shapely)
REQUIRES = {'numpy': '1.17.4', 'python': '3.8', 'shapely': '2.0.3'}

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
    'yaw_to_quaternion',
]
