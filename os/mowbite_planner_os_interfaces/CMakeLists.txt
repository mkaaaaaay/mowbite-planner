cmake_minimum_required(VERSION 3.8)
project(mowbite_planner_os_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# MowBite OS' own services of the planner (mowbite_planner_os), next to mowbite_planner_interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CutPlan.srv"
  "srv/PlanTransit.srv"
  DEPENDENCIES geometry_msgs nav_msgs rcl_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
