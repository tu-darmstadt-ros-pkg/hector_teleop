cmake_minimum_required(VERSION 2.8.3)
project(hector_joy_teleop_with_plugins)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs
        roscpp
        sensor_msgs
        pluginlib
        hector_joy_teleop_plugin_interface
        )

add_service_files(FILES
        LoadTeleopPlugin.srv
        )

generate_messages()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        roscpp
        sensor_msgs
        pluginlib
        hector_joy_teleop_plugin_interface
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


set(HEADER_FILES
        include/${PROJECT_NAME}/joy_teleop.h
        )

set(SOURCES
        src/joy_teleop.cpp
        )


add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/joy_teleop_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})





#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
#
## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
#        PATTERN ".svn" EXCLUDE
#        )