
#include "hector_joy_teleop_plugins/flipper_teleop.h"

namespace hector_joy_teleop_plugins
{

void FlipperTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map)
{
    TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::FlipperTeleop");

    speed_ = pnh_.param<float>(getParameterServerPrefix() + "/" + "speed", 0.0);
    flipper_front_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "flipper_front_command_topic",
                                                           "/flipper_control/flipper_front_velocity_controller/command");
    flipper_back_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "flipper_back_command_topic",
                                                           "/flipper_control/flipper_back_velocity_controller/command");
    standard_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "standard_controllers",
                                                             {"flipper_traj_controller"});
    teleop_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "teleop_controllers",
                                                              {"flipper_front_velocity_controller", "flipper_back_velocity_controller"});
    controller_manager_switch_service_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_switch_service",
                                                                 "/flipper_control/controller_manager/switch_controller");
    flipper_front_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_front_command_topic_, 10, false);
    flipper_back_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_back_command_topic_, 10, false);

    switch_controller_client_ = pnh_.serviceClient<controller_manager_msgs::SwitchController>(controller_manager_switch_service_);
}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // if the direction value is available in map and it is -1.0, reverse mode is active (handled when published!)
    auto iter = property_map_->find("direction");
    bool reverse_direction = (iter != property_map_->end()) && (iter->second == -1.0);

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


    if(!reverse_direction)
    {
        flipper_front_pub_.publish(flipper_front_command_);
        flipper_back_pub_.publish(flipper_back_command_);
    } else
    {
        // in reverse mode also reverse button mapping for front and back flippers, hence swap commands
        flipper_front_pub_.publish(flipper_back_command_);
        flipper_back_pub_.publish(flipper_front_command_);
    }


}

std::string FlipperTeleop::onLoad()
{
    switch_controller_srv_.request.start_controllers = teleop_controllers_;
    switch_controller_srv_.request.stop_controllers = standard_controllers_;
    switch_controller_srv_.request.strictness = 2;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;


    if (!switch_controller_client_.call(switch_controller_srv_) || !switch_controller_srv_.response.ok)
    {
      std::stringstream ss;
      for (const std::string& s: teleop_controllers_) {
        ss << s << " ";
      }
        return "Failed to switch to flipper velocity controllers: " + ss.str();
    }
    return "";
}

std::string FlipperTeleop::onUnload()
{
    switch_controller_srv_.request.start_controllers = standard_controllers_;
    switch_controller_srv_.request.stop_controllers = teleop_controllers_;
    switch_controller_srv_.request.strictness = 2;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if (!switch_controller_client_.call(switch_controller_srv_) || !switch_controller_srv_.response.ok)
    {
        return "Failed to switch to flipper trajectory controller!";
    }
    return "";
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
