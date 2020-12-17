
#include "hector_joy_teleop_plugins/flipper_teleop.h"

namespace hector_joy_teleop_plugins
{

void FlipperTeleop::initialize(ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               std::shared_ptr<std::map<std::string, double>> property_map)
{
    TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::FlipperTeleop");

    speed_ = pnh_.param<float>(getParameterServerPrefix() + "/" + "speed", 0.0);


    // get flipper topics
    flipper_front_command_topic_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "flipper_front_command_topic",
                                "/flipper_control/flipper_front_velocity_controller/command");
    flipper_back_command_topic_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "flipper_back_command_topic",
                                "/flipper_control/flipper_back_velocity_controller/command");

    // init publisher
    flipper_front_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_front_command_topic_, 10, false);
    flipper_back_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_back_command_topic_, 10, false);


    // get names for switching controllers
    standard_controllers_ =
        pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "standard_controllers",
                                             {});
    teleop_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "teleop_controllers",
                                                               {});
    // get values for switching controllers
    std::string controller_manager_switch_service =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_switch_service",
                                "/flipper_control/controller_manager/switch_controller");
    std::string controller_manager_list_service =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_list_service",
                                "/flipper_control/controller_manager/list_controllers");


    int num_tries_switch_controller =
        pnh_.param<int>(getParameterServerPrefix() + "/" + "num_tries_switch_controller", 5);
    int sleep_time = pnh_.param<int>(getParameterServerPrefix() + "/" + "sleep_between_tries_sec", 1);


    // init ControllerHelper for switching services later
    controller_helper_ = ControllerHelper(pnh,
                                          controller_manager_switch_service,
                                          controller_manager_list_service,
                                          num_tries_switch_controller,
                                          sleep_time,
                                          plugin_name_);

}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // if the direction value is available in map and it is -1.0, reverse mode is active (handled when published!)
    auto iter = property_map_->find("direction");
    bool reverse_direction = (iter != property_map_->end()) && (iter->second == -1.0);

    // map trigger axis
    sensor_msgs::JoyPtr mappedMsg = mapTriggerAxes(msg);


    // check if last command was zero
    bool last_cmd_zero = abs(flipper_front_command_.data) < 0.05 && abs(flipper_back_command_.data) < 0.05;


    // compute flipper commands
    joyToFlipperCommand(mappedMsg);


    // check if current command is zero
    bool current_cmd_zero = abs(flipper_front_command_.data) < 0.05 && abs(flipper_back_command_.data) < 0.05;

    // if last command was zero and current is not, switch controllers
    if(last_cmd_zero && !current_cmd_zero)
    {
        std::string result = controller_helper_.switchControllers(teleop_controllers_, standard_controllers_);

        if(!result.empty())
        {
            ROS_ERROR_STREAM(result);
        }
    }

    // publish
    if (!reverse_direction)
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

void FlipperTeleop::joyToFlipperCommand(const sensor_msgs::JoyConstPtr& msg)
{
    // front flipper
    float front_joystick;
    if (getJoyMeasurement("front", msg, front_joystick))
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
    }

    // back flipper
    float back_joystick;
    if (getJoyMeasurement("back", msg, back_joystick))
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
    }
}

std::string FlipperTeleop::onLoad()
{
    return controller_helper_.switchControllers(teleop_controllers_, standard_controllers_);
}

std::string FlipperTeleop::onUnload()
{
    return controller_helper_.switchControllers(standard_controllers_, teleop_controllers_);
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
