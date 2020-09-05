cmake_minimum_required(VERSION 3.0.2)
project(hector_joy_teleop_plugins)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        pluginlib
        flexbe_msgs
        actionlib
        hector_joy_teleop_plugin_interface
        hector_joy_teleop_plugin_msgs
        )


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES hector_joy_teleop_plugins
        CATKIN_DEPENDS
        geometry_msgs
        roscpp
        sensor_msgs
        pluginlib
        flexbe_msgs
        actionlib
        hector_joy_teleop_plugin_interface
        hector_joy_teleop_plugin_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


set(HEADER_FILES
        include/${PROJECT_NAME}/behavior_teleop.h
        include/${PROJECT_NAME}/change_profile.h
        include/${PROJECT_NAME}/drive_teleop.h
        include/${PROJECT_NAME}/flipper_teleop.h
        include/${PROJECT_NAME}/manage_properties.h
        include/${PROJECT_NAME}/manipulator_teleop.h
        include/${PROJECT_NAME}/sensorhead_teleop.h

        include/${PROJECT_NAME}/controller_helper.h
        )


set(SOURCES
        src/behavior_teleop.cpp
        src/change_profile.cpp
        src/drive_teleop.cpp
        src/flipper_teleop.cpp
        src/manage_properties.cpp
        src/manipulator_teleop.cpp
        src/sensorhead_teleop.cpp

        src/controller_helper.cpp
        )


add_library(${PROJECT_NAME} ${SOURCES} ${HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
        teleop_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

