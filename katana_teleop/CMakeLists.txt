cmake_minimum_required(VERSION 3.14)
project(katana_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kni REQUIRED)

# Derive kni install prefix from the CMake config dir
# kni_DIR = <prefix>/share/kni/cmake  →  go up 3 levels for prefix
get_filename_component(kni_PREFIX "${kni_DIR}/../../.." ABSOLUTE)
set(KNI_INCLUDE_DIR "${kni_PREFIX}/include")
set(KNI_LIB_DIR     "${kni_PREFIX}/lib")

add_executable(katana_teleop_key src/katana_teleop_key.cpp)

target_include_directories(katana_teleop_key PRIVATE
    ${KNI_INCLUDE_DIR}
)

ament_target_dependencies(katana_teleop_key
    ament_index_cpp
)

target_link_libraries(katana_teleop_key
    ${KNI_LIB_DIR}/libKNIBase.so
    ${KNI_LIB_DIR}/libKNI_LM.so
    ${KNI_LIB_DIR}/libKNI_InvKin.so
    ${KNI_LIB_DIR}/libknicommon.a
)

install(TARGETS katana_teleop_key
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
