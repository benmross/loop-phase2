cmake_minimum_required(VERSION 3.8)
project(s1_gui_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Target.msg
  msg/MissionState.msg
  srv/SetTargets.srv
  srv/SetStart.srv
  srv/MissionCommand.srv
  srv/ExportLog.srv
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
