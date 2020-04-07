
#include "hector_joy_teleop_plugin/plugins/drive_plugin.h"

DrivePlugin::DrivePlugin(ros::NodeHandle& nh, ros::NodeHandle& pnh) : PluginBase(nh, pnh, "DrivePlugin")
{
    // get values from common config file
    speed_forward_    = pnh_.param(plugin_name_ + "/" + "speed_forward", 0);
    speed_backward_   = pnh_.param(plugin_name_ + "/" + "speed_backward", 0);
    turn_speed_       = pnh_.param(plugin_name_ + "/" + "turn_speed", 0);
    slow_factor_      = pnh_.param(plugin_name_ + "/" + "slow_factor", 0);
    very_slow_factor_ = pnh_.param(plugin_name_ + "/" + "very_slow_factor", 0);

    motionCommandOutput = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
}

void DrivePlugin::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // compute linear speed (forward/backward)
    try
    {
        int speed = axes_.at("speed");
        float value = msg->axes[speed];
        if (value >= 0.0)
            motionCommand.linear.x = value * speed_forward_;
        else
            motionCommand.linear.x = value * speed_backward_;
    }
    catch (std::out_of_range& e)
    {
        ROS_ERROR_STREAM("The required axis/button mapping for value \"speed\" is missing (maybe misspelled?).");
    }


    // compute angular speed (left/right turn)
    try
    {
        int steer = axes_.at("steer");
        motionCommand.angular.z = msg->axes[steer] * turn_speed_;
    }
    catch (std::out_of_range& e)
    {
        ROS_ERROR_STREAM("The required axis/button mapping for value \"steer\" is missing (maybe misspelled?).");
    }


    // compute slow linear and angular speed
    try
    {
        int slow = axes_.at("slow");
        if (msg->buttons[slow])
        {
            motionCommand.linear.x *= slow_factor_;
            motionCommand.angular.z *= slow_factor_;
        }
    }
    catch (std::out_of_range& e)
    {
        ROS_ERROR_STREAM("The required axis/button mapping for value \"slow\" is missing (maybe misspelled?).");
    }


    // compute very slow linear and angular speed
    try
    {
        int very_slow = axes_.at("very_slow");
        if (msg->buttons[very_slow])
        {
            motionCommand.linear.x *= very_slow_factor_;
            motionCommand.angular.z *= very_slow_factor_;
        }
    }
    catch (std::out_of_range& e)
    {
        ROS_ERROR_STREAM("The required axis/button mapping for value \"very_slow\" is missing (maybe misspelled?).");
    }


    motionCommandOutput.publish(motionCommand);
}
