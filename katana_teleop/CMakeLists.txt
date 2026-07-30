cmake_minimum_required(VERSION 3.14)
project(katana_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ── katana_teleop_key (ros2_control keyboard teleop) ─────────────────────────

add_executable(katana_teleop_key src/katana_teleop_key.cpp)

ament_target_dependencies(katana_teleop_key
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  std_srvs
)

# ── Install ───────────────────────────────────────────────────────────────────

install(TARGETS katana_teleop_key
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
