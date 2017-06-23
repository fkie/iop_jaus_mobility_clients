cmake_minimum_required(VERSION 2.8.3)
project(iop_client_global_pose_sensor_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                iop_ocu_control_layerlib_fkie
                roscpp
                sensor_msgs
                )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

iop_init(COMPONENT_ID 138)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include/${PROJECT_NAME}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        iop_ocu_control_layerlib_fkie
        sensor_msgs
    DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
iop_code_generator(
# IDLS: the service definition files located in iop_builder_fkie/jsidl
# OWN_IDLS: the service definition files located in PROJECT_DIR/jsidl
# OVERRIDES: files which override the generated files while build. The generated files are copied to the same location with .gen extension
# EXTERN_SERVICES: the generated folder name of a service which is used from other library. This folder must be deleted while build.
  IDLS
    urn.jaus.jss.core-v1.0/AccessControlClient.xml
    urn.jaus.jss.core-v1.0/EventsClient.xml
    urn.jaus.jss.core-v1.0/DiscoveryClient.xml
    urn.jaus.jss.core-v1.0/ManagementClient.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.mobility/GlobalPoseSensorClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_mobility_GlobalPoseSensorClient/GlobalPoseSensorClient_ReceiveFSM.h
    src/urn_jaus_jss_mobility_GlobalPoseSensorClient/GlobalPoseSensorClient_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControlClient
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_DiscoveryClient
    urn_jaus_jss_core_ManagementClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS}
                    include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
            src/GlobalPoseSensorClientPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(
  DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

