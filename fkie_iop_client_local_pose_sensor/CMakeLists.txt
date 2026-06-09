cmake_minimum_required(VERSION 3.5)
project(fkie_iop_client_local_pose_sensor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(fkie_iop_ocu_slavelib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

###############
## setup IOP ##
###############
iop_init()
iop_export_service(
  urn_jaus_jss_mobility_LocalPoseSensorClient
)
## Specify additional locations of header files
iop_code_generator(
# IDLS: the service definition files located in fkie_iop_builder/jsidl
# OWN_IDLS: the service definition files located in PROJECT_DIR/jsidl
# OVERRIDES: files which override the generated files while build. The generated files are copied to the same location with .gen extension
# EXTERN_SERVICES: the generated folder name of a service which is used from other library. This folder must be deleted while build.
  IDLS
    urn.jaus.jss.core/AccessControlClient.xml
    urn.jaus.jss.core/EventsClient.xml
    urn.jaus.jss.core/DiscoveryClient.xml
    urn.jaus.jss.core/Transport.xml
    urn.jaus.jss.mobility/LocalPoseSensorClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_mobility_LocalPoseSensorClient/LocalPoseSensorClient_ReceiveFSM.h
    src/urn_jaus_jss_mobility_LocalPoseSensorClient/LocalPoseSensorClient_ReceiveFSM.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_DiscoveryClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

###########
## Build ##
###########
set(dependencies
  fkie_iop_ocu_slavelib
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

include_directories(
  ${IOP_COMPONENT_INCLUDE_DIRS}
  ${fkie_iop_ocu_slavelib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            ${cppfiles}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  fkie_iop_ocu_slavelib::fkie_iop_ocu_slavelib
)

#############
## Install ##
#############

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS}
  DESTINATION include/${PROJECT_NAME}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS} ${fkie_iop_ocu_slavelib_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
