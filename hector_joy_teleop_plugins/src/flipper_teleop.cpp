
#include "hector_joy_teleop_plugins/flipper_teleop.h"

namespace hector_joy_teleop_plugins
{

void FlipperTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    TeleopBase::initializeBase(nh, pnh, "hector_joy_teleop_plugins::FlipperTeleop");

    speed_ = pnh_.param<float>(getParameterServerPrefix() + "/" + "speed", 0.0);

    flipper_front_pub_ =
        nh_.advertise<std_msgs::Float64>("/flipper_control/flipper_front_velocity_controller/command", 10, false);
    flipper_back_pub_ =
        nh_.advertise<std_msgs::Float64>("/flipper_control/flipper_back_velocity_controller/command", 10, false);

    switch_controller_client_ = pnh_.serviceClient<controller_manager_msgs::SwitchController>(
        "/flipper_control/controller_manager/switch_controller");
}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    // front flipper
    float front_up_joystick, front_down_joystick, front_joystick;
    if (getJoyMeasurement("front_up", msg, front_up_joystick, false)
        && getJoyMeasurement("front_down", msg, front_down_joystick, false))
    {
        if (front_up_joystick == 0.0)
        {
            if (front_down_joystick == 1.0) // NOTE: set to 1.0 as here is LT is used which is 1.0 by default
            {
                flipper_front_command_.data = 0.0;
            } else
            {
                flipper_front_command_.data = -speed_;
            }
        } else
        {
            flipper_front_command_.data = speed_;
        }

    } else if (getJoyMeasurement("front", msg, front_joystick, false))
    {
        if (front_joystick == 0.0)
        {
            flipper_front_command_.data = 0.0;
        } else if (front_joystick > 0.0)
        {
            flipper_front_command_.data = speed_;
        } else
        {
            flipper_front_command_.data = -speed_;
        }
    } else
    {
        ROS_ERROR_STREAM(plugin_name_
                             << ": The required axis/button mapping for value \"front\" or \"front_up\" and \"front_down\" are missing (maybe misspelled?).");
    }



    // back flipper
    float back_up_joystick, back_down_joystick, back_joystick;
    if (getJoyMeasurement("back_up", msg, back_up_joystick, false)
        && getJoyMeasurement("back_down", msg, back_down_joystick, false))
    {
        if (back_up_joystick == 0.0)
        {
            if (back_down_joystick == 1.0) // NOTE: set to 1.0 as here is RT is used which is 1.0 by default
            {
                flipper_back_command_.data = 0.0;
            } else
            {
                flipper_back_command_.data = -speed_;
            }
        } else
        {
            flipper_back_command_.data = speed_;
        }

    } else if (getJoyMeasurement("back", msg, back_joystick, false))
    {
        if (back_joystick == 0.0)
        {
            flipper_back_command_.data = 0.0;
        } else if (back_joystick > 0.0)
        {
            flipper_back_command_.data = speed_;
        } else
        {
            flipper_back_command_.data = -speed_;
        }
    } else
    {
        ROS_ERROR_STREAM(plugin_name_
                             << ": The required axis/button mapping for value \"back\" or \"back_up\" and \"back_down\" are missing (maybe misspelled?).");
    }

    flipper_front_pub_.publish(flipper_front_command_);
    flipper_back_pub_.publish(flipper_back_command_);

}

std::string FlipperTeleop::onLoad()
{
    switch_controller_srv_.request.start_controllers =
        {"flipper_front_velocity_controller", "flipper_back_velocity_controller"};
    switch_controller_srv_.request.stop_controllers = {"flipper_traj_controller"};
    switch_controller_srv_.request.strictness = 1;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if (!switch_controller_client_.call(switch_controller_srv_))
    {
        return "Failed to switch to flipper velocity controller!";

    }
    return "";
}

std::string FlipperTeleop::onUnload()
{
    switch_controller_srv_.request.start_controllers = {"flipper_traj_controller"};
    switch_controller_srv_.request.stop_controllers =
        {"flipper_front_velocity_controller", "flipper_back_velocity_controller"};
    switch_controller_srv_.request.strictness = 1;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if (!switch_controller_client_.call(switch_controller_srv_))
    {
        return "Failed to switch to flipper trajectory controller!";
    }
    return "";
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)