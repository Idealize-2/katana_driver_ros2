cmake_minimum_required(VERSION 3.22)
project(katana400_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)

# Dependencies for moveit_pose_mover executable
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ── moveit_pose_mover ─────────────────────────────────────────────────────────
#
# Interactive MoveIt 2 Cartesian commander:
#   [p] single pose goal, [c] Cartesian path, [j] joint positions, [q] quit
#
# Run with:
#   ros2 launch katana400_moveit_config moveit_pose_mover.launch.py

add_executable(moveit_pose_mover src/moveit_pose_mover.cpp)
ament_target_dependencies(moveit_pose_mover
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
)

install(TARGETS moveit_pose_mover
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config / launch / setup ───────────────────────────────────────────

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
