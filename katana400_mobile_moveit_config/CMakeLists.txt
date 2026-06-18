cmake_minimum_required(VERSION 3.22)
project(katana400_mobile_moveit_config)
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

add_executable(mobile_moveit_pose_mover src/mobile_moveit_pose_mover.cpp)
ament_target_dependencies(mobile_moveit_pose_mover
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
)

add_executable(base_arm_pose_mover src/base_arm_pose_mover.cpp)
ament_target_dependencies(base_arm_pose_mover
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  
  tf2
)



if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(TARGETS mobile_moveit_pose_mover base_arm_pose_mover
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/eef_marker.py
  scripts/mobile_base_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
ament_package()