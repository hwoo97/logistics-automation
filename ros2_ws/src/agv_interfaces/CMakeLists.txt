cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineState.msg"
  "msg/StopLineState.msg"
  "msg/MarkerState.msg"
  "msg/Status.msg"
  "msg/Command.msg"
  "srv/Classify.srv"
  "action/Drive.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
