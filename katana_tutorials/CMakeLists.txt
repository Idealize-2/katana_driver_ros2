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

# KNI SDK (used by tutorials 4-5)
find_package(kni REQUIRED)

# Derive KNI install paths from kni_DIR (same trick as katana_driver)
# kni_DIR = <prefix>/share/kni/cmake  → prefix is 3 levels up
get_filename_component(kni_PREFIX "${kni_DIR}/../../.." ABSOLUTE)
set(KNI_INCLUDE_DIR "${kni_PREFIX}/include")
set(KNI_LIB_DIR     "${kni_PREFIX}/lib")
message(STATUS "[katana_tutorials] kni include: ${KNI_INCLUDE_DIR}")

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

# ── Macro helpers ─────────────────────────────────────────────────────────────

# Macro: build a ROS 2 tutorial executable (links against katana_arm_client)
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

# Macro: build a standalone KNI tutorial executable (no ROS 2 deps)
macro(kni_tutorial name)
  add_executable(${name} src/${name}.cpp)
  target_include_directories(${name} PRIVATE
    ${KNI_INCLUDE_DIR}
  )
  target_link_libraries(${name}
    ${KNI_LIB_DIR}/libKNIBase.so
    ${KNI_LIB_DIR}/libKNI_LM.so
    ${KNI_LIB_DIR}/libKNI_InvKin.so
    ${KNI_LIB_DIR}/libknicommon.a
  )
endmacro()

# ── Tutorial targets ──────────────────────────────────────────────────────────

# Tutorial 1: Hardcoded trajectory via ros2_control action server
ros2_tutorial(follow_joint_trajectory_client)

# Tutorial 2: 3D IK (simplified, no offsets) via ros2_control
ros2_tutorial(sixR_inverse_kinematics)

# Tutorial 3: Full 3D IK (with DH offsets) via ros2_control
ros2_tutorial(offset_inverse_kinematics)

# Tutorial 4: Direct KNI encoder reader (no ros2_control needed)
kni_tutorial(encoder_reader)

# Tutorial 5: Disable motors + keep reading encoders (direct KNI)
kni_tutorial(arm_disable_encoder)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  katana_arm_client
  follow_joint_trajectory_client
  sixR_inverse_kinematics
  offset_inverse_kinematics
  encoder_reader
  arm_disable_encoder
  EXPORT export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION lib/${PROJECT_NAME}  # ros2 run looks here
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(katana_arm_client)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
