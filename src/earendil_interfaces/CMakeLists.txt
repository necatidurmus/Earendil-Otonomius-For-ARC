cmake_minimum_required(VERSION 3.8)
project(earendil_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages — existing
  "msg/VehicleState.msg"
  "msg/MissionStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/HealthDiag.msg"
  "msg/Waypoint.msg"

  # Messages — RSCP bridge
  "msg/RscpCommand.msg"
  "msg/RscpStatus.msg"
  "msg/MissionStage.msg"

  # Messages — Mission lifecycle (dahili, RSCP'den bağımsız)
  "msg/MissionCommand.msg"

  # Messages — STM bridge
  "msg/StmStatus.msg"
  "msg/StmFaultFlags.msg"

  # Messages — Jetson
  "msg/JetsonArucoDetections.msg"

  # Services
  "srv/EmergencyStop.srv"
  "srv/SetMission.srv"
  "srv/SetSpeedLimit.srv"

  # Actions
  "action/NavigateWaypoint.action"
  "action/ExecuteMission.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    builtin_interfaces
)

ament_package()
