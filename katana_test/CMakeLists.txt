cmake_minimum_required(VERSION 3.8)
project(katana_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(ros2control_tester src/ros2control_tester.cpp)

ament_target_dependencies(ros2control_tester
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  std_srvs
)

install(TARGETS ros2control_tester
  DESTINATION lib/${PROJECT_NAME})

ament_package()
