cmake_minimum_required(VERSION 3.8)
project(katana_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kni REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

# Derive KNI library directory from the CMake config location
# kni_DIR = <prefix>/share/kni/cmake → go up 3 levels
get_filename_component(kni_PREFIX "${kni_DIR}/../../.." ABSOLUTE)
set(KNI_INCLUDE_DIR "${kni_PREFIX}/include")
set(KNI_LIB_DIR     "${kni_PREFIX}/lib")

# ── arm_disable_encoder (direct KNI, no ROS) ─────────────────────────────────

add_executable(arm_disable_encoder src/arm_disable_encoder.cpp)
target_include_directories(arm_disable_encoder PRIVATE ${KNI_INCLUDE_DIR})
ament_target_dependencies(arm_disable_encoder
  ament_index_cpp
)
target_link_libraries(arm_disable_encoder
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# ── katana_kni_teleop (low-level KNI keyboard teleop, diagnostic) ─────────────
# Moved from katana_teleop; bypasses ros2_control and talks directly to hardware.

add_executable(katana_kni_teleop src/katana_kni_teleop.cpp)
target_include_directories(katana_kni_teleop PRIVATE ${KNI_INCLUDE_DIR})
ament_target_dependencies(katana_kni_teleop
  ament_index_cpp
)
target_link_libraries(katana_kni_teleop
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
  arm_disable_encoder
  katana_kni_teleop
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
