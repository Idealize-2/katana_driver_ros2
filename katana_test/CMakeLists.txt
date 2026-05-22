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

# ── kni_spline_gripper_test (direct sendSplineToMotor teleop for gripper) ─────

add_executable(kni_spline_gripper_test src/kni_spline_gripper_test.cpp)

target_include_directories(kni_spline_gripper_test PRIVATE ${KNI_INCLUDE_DIR})

ament_target_dependencies(kni_spline_gripper_test
  rclcpp
  ament_index_cpp
  kni
)

target_link_libraries(kni_spline_gripper_test
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# ── install ───────────────────────────────────────────────────────────────────

install(TARGETS
  ros2control_tester
  kni_gripper_test
  kni_spline_gripper_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
