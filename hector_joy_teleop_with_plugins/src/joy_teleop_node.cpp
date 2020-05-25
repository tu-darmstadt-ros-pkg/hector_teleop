

#include <ros/ros.h>

#include "hector_joy_teleop_with_plugins/joy_teleop.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    hector_joy_teleop_with_plugins::JoyTeleop joy_teleop(nh, pnh);

    ros::Rate rate(25.0);

    while (ros::ok())
    {
        ros::spinOnce();

        joy_teleop.executePeriodically(rate);

        rate.sleep();
    }

    return 0;
}