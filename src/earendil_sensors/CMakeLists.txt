cmake_minimum_required(VERSION 3.8)
project(earendil_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(earendil_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/gps_adapter.py
  ${PROJECT_NAME}/imu_adapter.py
  ${PROJECT_NAME}/lidar_adapter.py
  ${PROJECT_NAME}/encoder_adapter.py
  ${PROJECT_NAME}/diagnostics_node.py
  ${PROJECT_NAME}/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
