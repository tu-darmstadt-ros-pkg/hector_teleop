cmake_minimum_required(VERSION 2.8.3)
project(hector_joy_teleop_plugin)

add_compile_options(-std=c++11)

## only needed when using CLion
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/ros/melodic/)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(HEADER_FILES
        include/hector_joy_teleop_plugin/joy_teleop_plugin.h
        include/hector_joy_teleop_plugin/plugin_base.h)

set(SOURCES
        src/joy_teleop_plugin_node.cpp
        src/joy_teleop_plugin.cpp
        src/plugin_base.cpp)


add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )