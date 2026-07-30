cmake_minimum_required(VERSION 3.8)
project(kni)

add_compile_options(-std=c++11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)

include_directories(KNI_4.3.0/include ${Boost_INCLUDE_DIRS})

# knicommon
aux_source_directory(KNI_4.3.0/src/common/ knicommon_SOURCES)
add_library(knicommon STATIC ${knicommon_SOURCES})

# KNIBase
aux_source_directory(KNI_4.3.0/src/Base/ KNIBase_SOURCES)
add_library(KNIBase SHARED ${KNIBase_SOURCES})
target_link_libraries(KNIBase knicommon)

# roboop
aux_source_directory(KNI_4.3.0/lib/kinematics/roboop/source/ roboop_SOURCES)
aux_source_directory(KNI_4.3.0/lib/kinematics/roboop/newmat/ roboop_SOURCES)
include_directories(include KNI_4.3.0/lib/kinematics/roboop/newmat/)
add_library(roboop STATIC ${roboop_SOURCES})

# Kinematics
aux_source_directory(KNI_4.3.0/lib/kinematics/ Kinematics_SOURCES)
aux_source_directory(KNI_4.3.0/lib/kinematics/AnalyticalGuess/src/ Kinematics_SOURCES)
include_directories(include KNI_4.3.0/lib/kinematics/AnalyticalGuess/include/)
add_library(Kinematics SHARED ${Kinematics_SOURCES})
target_link_libraries(Kinematics roboop)

# KNI_InvKin
aux_source_directory(KNI_4.3.0/src/InvKin/ KNI_InvKin_SOURCES)
add_library(KNI_InvKin SHARED ${KNI_InvKin_SOURCES})
target_link_libraries(KNI_InvKin Kinematics)

# KNI_LM
aux_source_directory(KNI_4.3.0/src/LM/ KNI_LM_SOURCES)
add_library(KNI_LM SHARED ${KNI_LM_SOURCES})
target_link_libraries(KNI_LM knicommon KNIBase)

set_target_properties(knicommon KNIBase roboop Kinematics KNI_InvKin KNI_LM 
  PROPERTIES COMPILE_FLAGS "-fPIC")

install(TARGETS knicommon KNIBase roboop Kinematics KNI_InvKin KNI_LM
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY KNI_4.3.0/include/
  DESTINATION include)

install(DIRECTORY KNI_4.3.0/configfiles300
  DESTINATION share/${PROJECT_NAME}/KNI_4.3.0/)

install(DIRECTORY KNI_4.3.0/configfiles400
  DESTINATION share/${PROJECT_NAME}/KNI_4.3.0/)

install(DIRECTORY KNI_4.3.0/configfiles450
  DESTINATION share/${PROJECT_NAME}/KNI_4.3.0/)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(knicommon KNIBase roboop Kinematics KNI_InvKin KNI_LM)

ament_package()
