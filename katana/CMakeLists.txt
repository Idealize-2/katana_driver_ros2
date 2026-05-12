cmake_minimum_required(VERSION 3.8)
project(katana)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kni REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(katana_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ARMADILLO REQUIRED armadillo)
include_directories(include
                    ${CMAKE_CURRENT_SOURCE_DIR}/../kni/KNI_4.3.0/include)

add_executable(katana src/KatanaNode.cpp
                      src/AbstractKatana.cpp
                      src/Katana.cpp
                      src/Katana300.cpp
                      src/SimulatedKatana.cpp
                      src/KNIConverter.cpp
                      src/spline_functions.cpp)
ament_target_dependencies(katana
  rclcpp
  rclcpp_action
  kni
  std_srvs
  sensor_msgs
  urdf
  tf2
  tf2_ros
  katana_msgs
  geometry_msgs
)
target_link_libraries(katana ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES})
target_include_directories(katana PUBLIC ${ARMADILLO_INCLUDE_DIRS})

# add_executable(katana_arm_kinematics src/KNIConverter.cpp
#                       src/KNIKinematics.cpp)
# ament_target_dependencies(katana_arm_kinematics
#   rclcpp
#   kni
#   tf2
#   katana_msgs
# )

install(TARGETS katana
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS nodes/fake_katana_joint_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
