cmake_minimum_required(VERSION 2.8.3)
project(hector_joy_tcp_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  geometry_msgs
  hardware_interface
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_joy_tcp_teleop
  CATKIN_DEPENDS controller_interface geometry_msgs hardware_interface sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/joy_tcp_controller.h
  src/joy_tcp_controller.cpp

  include/${PROJECT_NAME}/controller_mapping/controller_mapper_factory.h
  include/${PROJECT_NAME}/controller_mapping/controller_mapper_base.h
  include/${PROJECT_NAME}/controller_mapping/axis_mapper.h
  include/${PROJECT_NAME}/controller_mapping/button_axis_mapper.h
  include/${PROJECT_NAME}/controller_mapping/dual_axis_mapper.h
  include/${PROJECT_NAME}/controller_mapping/button_mapper.h
  src/controller_mapping/controller_mapper_base.cpp
  src/controller_mapping/axis_mapper.cpp
  src/controller_mapping/button_axis_mapper.cpp
  src/controller_mapping/dual_axis_mapper.cpp
  src/controller_mapping/button_mapper.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
