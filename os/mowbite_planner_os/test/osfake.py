"""What the tests of mowbite_planner_os import first: mowbite_planner from the repository where it isn't installed, and
with a sourced ROS 2 workspace (colcon test) the real rclpy, mowbite_msgs and interfaces, else the stand-ins of
mowbite_planner's tests (ros2/mowbite_planner/test/fake_ros.py) with mowbite_msgs from os/ci and
mowbite_planner_os_interfaces from its .srv."""

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
OS = os.path.join(HERE, '..', '..')
ROS2_TEST = os.path.join(OS, '..', 'ros2', 'mowbite_planner', 'test')
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(OS, '..', 'ros2', 'mowbite_planner'), ROS2_TEST, HERE]

try:
    import rclpy  # noqa: F401
    import mowbite_msgs.msg  # noqa: F401
    import mowbite_planner_os_interfaces.srv  # noqa: F401
    REAL_ROS = True
except ImportError:
    REAL_ROS = False
    for name in [m for m in sys.modules if m.split('.')[0] in ('rclpy', 'mowbite_msgs')]:
        del sys.modules[name]
    import fake_ros
    fake_ros.install()
    for package, folder in (('mowbite_msgs', os.path.join(OS, 'ci', 'mowbite_msgs')),
                            ('mowbite_planner_os_interfaces', os.path.join(OS, 'mowbite_planner_os_interfaces'))):
        msgs, srvs = fake_ros._interfaces(package, folder)
        sys.modules[package] = types.ModuleType(package)
        sys.modules[package + '.msg'], sys.modules[package + '.srv'] = msgs, srvs
