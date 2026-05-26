cmake_minimum_required(VERSION 3.8)
project(katana_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kni REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

# Derive KNI library directory from the CMake config location
# kni_DIR = <prefix>/share/kni/cmake → go up 3 levels
get_filename_component(kni_PREFIX "${kni_DIR}/../../.." ABSOLUTE)
set(KNI_INCLUDE_DIR "${kni_PREFIX}/include")
set(KNI_LIB_DIR     "${kni_PREFIX}/lib")

# ── ros2control_tester ───────────────────────────────────────────────────────

add_executable(ros2control_tester src/ros2control_tester.cpp)

ament_target_dependencies(ros2control_tester
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  std_srvs
)

# ── kni_gripper_test (dual-mode: KNI direct + ros2_control) ──────────────────

add_executable(kni_gripper_test src/kni_gripper_test.cpp)

target_include_directories(kni_gripper_test PRIVATE ${KNI_INCLUDE_DIR})

ament_target_dependencies(kni_gripper_test
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  ament_index_cpp
  kni
)

target_link_libraries(kni_gripper_test
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# ── encoder_reader (direct KNI, no ROS) ──────────────────────────────────────

add_executable(encoder_reader src/encoder_reader.cpp)
target_include_directories(encoder_reader PRIVATE ${KNI_INCLUDE_DIR})
target_link_libraries(encoder_reader
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# ── arm_disable_encoder (direct KNI, no ROS) ─────────────────────────────────

add_executable(arm_disable_encoder src/arm_disable_encoder.cpp)
target_include_directories(arm_disable_encoder PRIVATE ${KNI_INCLUDE_DIR})
target_link_libraries(arm_disable_encoder
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# ── pose_runner (MoveIt + yaml-cpp interactive pose runner) ──────────────────

add_executable(pose_runner src/pose_runner.cpp)
ament_target_dependencies(pose_runner
  rclcpp
  moveit_ros_planning_interface
  ament_index_cpp
)
target_link_libraries(pose_runner ${YAML_CPP_LIBRARIES})

# ── install ───────────────────────────────────────────────────────────────────

install(TARGETS
  ros2control_tester
  kni_gripper_test
  encoder_reader
  arm_disable_encoder
  pose_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
