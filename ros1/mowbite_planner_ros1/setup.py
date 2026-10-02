# installs the planner (plain python, shared with the ROS 2 package) for catkin_python_setup()
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(
    packages=['mowbite_planner'],
    package_dir={'mowbite_planner': '../../ros2/mowbite_planner/mowbite_planner'},
))
