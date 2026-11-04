cmake_minimum_required(VERSION 3.8)
project(mowbite_msgs)

# the messages of MowBite OS (mowbite-os, src/mowbite_msgs) mowbite_planner_os uses, as they are there: os.yml builds
# the node with these, on MowBite OS its own mowbite_msgs is there
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapStatus.msg"
  "msg/MqttPublish.msg"
  "msg/RpcError.msg"
  "msg/RpcMethods.msg"
  "msg/RpcRequest.msg"
  "msg/RpcResponse.msg"
  "srv/ReplaceMap.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
