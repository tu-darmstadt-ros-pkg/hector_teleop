cmake_minimum_required(VERSION 2.8.3)
project(hector_joy_teleop_plugin_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        )

add_service_files(FILES
        LoadTeleopPlugin.srv
        SetProperty.srv
        )

generate_messages()

catkin_package(
        CATKIN_DEPENDS message_runtime
)


include_directories(
        ${catkin_INCLUDE_DIRS}
)




#install(DIRECTORY
#  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
