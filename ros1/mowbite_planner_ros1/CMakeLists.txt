cmake_minimum_required(VERSION 3.0.2)
project(mowbite_planner_ros1)

find_package(catkin REQUIRED)

# the planner itself (ros2/mowbite_planner/mowbite_planner, plain python), see setup.py
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS nodes/mowbite_planner_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
