
#include "hector_joy_teleop_plugins/flipper_teleop.h"

namespace hector_joy_teleop_plugins
{

void FlipperTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    TeleopBase::initializeBase(nh, pnh, "hector_joy_teleop_plugins::FlipperTeleop");

    speed_ = pnh_.param(getParameterServerPrefix() + "/" + "speed", 0.0);

    flipper_front_pub_ = nh_.advertise<std_msgs::Float64>("/flipper_control/flipper_front_velocity_controller/command", 10, false);
    flipper_back_pub_ = nh_.advertise<std_msgs::Float64>("/flipper_control/flipper_back_velocity_controller/command", 10, false);

    switch_controller_client_  = pnh_.serviceClient<controller_manager_msgs::SwitchController>("/flipper_control/controller_manager/switch_controller");
}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    try
    {
        int front_mapping = axes_.at("front");
        float front_joystick = msg->axes[front_mapping];

        flipper_front_command_.data = front_joystick * speed_;
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("front");
    }

    try
    {
        int back_mapping = axes_.at("back");
        float back_joystick = msg->axes[back_mapping];

        flipper_back_command_.data = back_joystick * speed_;
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("back");
    }

    flipper_front_pub_.publish(flipper_front_command_);
    flipper_back_pub_.publish(flipper_back_command_);

}

void FlipperTeleop::onLoad()
{
    switch_controller_srv_.request.start_controllers = {"flipper_front_velocity_controller","flipper_back_velocity_controller"};
    switch_controller_srv_.request.stop_controllers = {"flipper_traj_controller"};
    switch_controller_srv_.request.strictness = 1;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if(!switch_controller_client_.call(switch_controller_srv_))
    {
        ROS_ERROR_STREAM("Failed to switch to flipper velocity controller!");
    }
}

void FlipperTeleop::onUnload()
{
    switch_controller_srv_.request.start_controllers = {"flipper_traj_controller"};
    switch_controller_srv_.request.stop_controllers = {"flipper_front_velocity_controller","flipper_back_velocity_controller"};
    switch_controller_srv_.request.strictness = 1;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if(!switch_controller_client_.call(switch_controller_srv_))
    {
        ROS_ERROR_STREAM("Failed to switch to flipper trajectory controller!");
    }
}

}


PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)