cmake_minimum_required(VERSION 3.8)
project(katana_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  collada
  launch
  meshes
  params
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
