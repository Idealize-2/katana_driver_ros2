cmake_minimum_required(VERSION 3.8)
project(katana_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────

# ROS 2 deps (used by tutorials 1-3)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# KNI SDK (used by tutorial 4 — ik_pose_mover)
find_package(kni REQUIRED)
get_filename_component(kni_PREFIX "${kni_DIR}/../../.." ABSOLUTE)
set(KNI_INCLUDE_DIR "${kni_PREFIX}/include")
set(KNI_LIB_DIR     "${kni_PREFIX}/lib")

# ── Shared library (KatanaArmClient — used by tutorials 1-3) ─────────────────
add_library(katana_arm_client SHARED
  src/katana_arm_client.cpp
)
target_include_directories(katana_arm_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(katana_arm_client
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  trajectory_msgs
)

# ── Macro: build a ROS 2 tutorial executable (links against katana_arm_client) ──
macro(ros2_tutorial name)
  add_executable(${name} src/${name}.cpp)
  target_include_directories(${name} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(${name}
    rclcpp rclcpp_action control_msgs sensor_msgs trajectory_msgs
  )
  target_link_libraries(${name} katana_arm_client)
endmacro()

# ── Tutorial targets ──────────────────────────────────────────────────────────

# Tutorial 1: Hardcoded trajectory via ros2_control action server
ros2_tutorial(follow_joint_trajectory_client)

# Tutorial 2: 3D IK (simplified, no offsets) via ros2_control
ros2_tutorial(sixR_inverse_kinematics)

# Tutorial 3: Full 3D IK (with DH offsets) via ros2_control
ros2_tutorial(offset_inverse_kinematics)

# Tutorial 5: Multi-waypoint trajectory (current → calibration → straight_up → calibration)
ros2_tutorial(pr2_joint_trajectory_client)

# Tutorial 6: 2D planar IK (user enters X Y in cm, pan fixed at 0)
ros2_tutorial(test_inverse_kinematics)

# Tutorial 4: Calibrate → limp & observe → IK check → moveRobotTo (direct KNI)
add_executable(ik_pose_mover src/ik_pose_mover.cpp)
target_include_directories(ik_pose_mover PRIVATE ${KNI_INCLUDE_DIR})
target_link_libraries(ik_pose_mover
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  katana_arm_client
  follow_joint_trajectory_client
  sixR_inverse_kinematics
  offset_inverse_kinematics
  pr2_joint_trajectory_client
  test_inverse_kinematics
  ik_pose_mover
  EXPORT export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION lib/${PROJECT_NAME}  # ros2 run looks here
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(katana_arm_client)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
