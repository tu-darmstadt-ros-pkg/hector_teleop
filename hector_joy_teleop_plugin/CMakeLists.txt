cmake_minimum_required(VERSION 2.8.3)
project(hector_joy_teleop_plugin)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs
        roscpp
        sensor_msgs
        )

add_service_files(FILES
        LoadTeleopPlugin.srv
        )

generate_messages()

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime geometry_msgs roscpp sensor_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


set(HEADER_FILES
        include/hector_joy_teleop_plugin/joy_teleop_plugin.h
        include/hector_joy_teleop_plugin/plugin_base.h
        include/hector_joy_teleop_plugin/plugins/drive_plugin.h
        )

set(SOURCES
        src/joy_teleop_plugin_node.cpp
        src/joy_teleop_plugin.cpp
        src/plugin_base.cpp
        src/plugins/drive_plugin.cpp)


add_executable(${PROJECT_NAME}_node ${SOURCES} ${HEADER_FILES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})