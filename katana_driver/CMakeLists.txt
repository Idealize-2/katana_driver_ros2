cmake_minimum_required(VERSION 3.8)
project(katana_driver)

# Enforce C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(kni REQUIRED)

# Derive kni install prefix from the kni CMake config dir
# kni_DIR = <prefix>/share/kni/cmake  →  go up 3 levels for prefix
get_filename_component(kni_PREFIX "${kni_DIR}/../../.." ABSOLUTE)
set(KNI_INCLUDE_DIR "${kni_PREFIX}/include")
set(KNI_LIB_DIR     "${kni_PREFIX}/lib")
message(STATUS "kni prefix: ${kni_PREFIX}")
message(STATUS "KNI_INCLUDE_DIR: ${KNI_INCLUDE_DIR}")

# ── Shared library (the plugin itself) ──────────────────────────────────────
add_library(katana_hardware_interface SHARED
  src/katana_hardware_interface.cpp
)

target_include_directories(katana_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${KNI_INCLUDE_DIR}    # resolves to install/kni/include at build time
)

ament_target_dependencies(katana_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  kni
)

# Link all KNI sub-libraries using full paths so no -L flag gymnastics needed
target_link_libraries(katana_hardware_interface
  ${KNI_LIB_DIR}/libKNIBase.so
  ${KNI_LIB_DIR}/libKNI_LM.so
  ${KNI_LIB_DIR}/libKNI_InvKin.so
  ${KNI_LIB_DIR}/libknicommon.a
)

# Register the pluginlib plugin description file
pluginlib_export_plugin_description_file(
  hardware_interface katana_hardware.xml
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS katana_hardware_interface
  EXPORT export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES katana_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(katana_hardware_interface)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  kni
)

ament_package()
