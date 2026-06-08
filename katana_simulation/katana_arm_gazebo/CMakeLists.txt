cmake_minimum_required(VERSION 3.8)
project(katana_arm_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  worlds
  meshes
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
