cmake_minimum_required(VERSION 3.16)
project(perimeter_defense_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "msg/PredictedPath.msg"
  "msg/PredictedPathArray.msg"
  "msg/Alert.msg"
  "msg/ThreatLevel.msg"
  "msg/RoleAssignment.msg"
  "msg/Transponder.msg"
  "msg/Custody.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
