"""MowBite Planner: coverage paths for robot mowers (perimeter loops, boustrophedon lanes, gentle turns).

planner.py and the modules it uses need only shapely, node.py wraps them as a ROS 2 node.
"""

from .mathutil import Pose2D, normalize_angle, quaternion_to_yaw, yaw_to_quaternion
from .planner import CoveragePlan, PlannerConfig, PlanningError, SegmentInfo, plan_coverage, swept_area

# also in setup.py and package.xml
__version__ = '0.1.0'

__all__ = [
    '__version__',
    'CoveragePlan',
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
