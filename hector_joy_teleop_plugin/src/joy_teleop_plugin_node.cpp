

#include <ros/ros.h>

#include "hector_joy_teleop_plugin/joy_teleop_plugin.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // TODO rate ok?
    ros::Rate rate(25.0);

    JoyTeleopPlugin jtp = JoyTeleopPlugin(nh, pnh, rate);



    ros::spin();

//    while(ros::ok()) {
////        ros::spinOnce();
////        rate.sleep();
////    }

    return 0;
}