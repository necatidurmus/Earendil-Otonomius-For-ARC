cmake_minimum_required(VERSION 3.8)
project(earendil_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionState.msg"
  "msg/RscpCommand.msg"
  "msg/TaskStatus.msg"
  "msg/LocalizationMode.msg"
  "srv/ExecuteTask.srv"
  "srv/SetMissionStage.srv"
  "action/NavigateToWaypoint.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
