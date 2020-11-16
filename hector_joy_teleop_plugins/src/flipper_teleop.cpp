
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


    // get names for switching controllers
    standard_controllers_ =
        pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "standard_controllers",
                                             {"flipper_traj_controller"});
    teleop_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "teleop_controllers",
                                                               {"flipper_front_velocity_controller",
                                                                "flipper_back_velocity_controller"});


    // get services
    controller_manager_switch_service_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_switch_service",
                                "/flipper_control/controller_manager/switch_controller");
    controller_manager_list_service_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_list_service",
                                "/flipper_control/controller_manager/list_controllers");


    // init publisher and service
    flipper_front_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_front_command_topic_, 10, false);
    flipper_back_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_back_command_topic_, 10, false);

    switch_controller_client_ =
        pnh_.serviceClient<controller_manager_msgs::SwitchController>(controller_manager_switch_service_);

    list_controllers_client_ =
        pnh_.serviceClient<controller_manager_msgs::ListControllers>(controller_manager_list_service_);


    // get values for controller switch retries
    num_tries_switch_controller_ = pnh_.param<int>(getParameterServerPrefix() + "/" + "num_tries_switch_controller", 5);
    int sleep_time = pnh_.param<int>(getParameterServerPrefix() + "/" + "sleep_between_tries_sec", 1);
    sleep_between_tries_sec_ = ros::Duration(sleep_time);
}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // if the direction value is available in map and it is -1.0, reverse mode is active (handled when published!)
    auto iter = property_map_->find("direction");
    bool reverse_direction = (iter != property_map_->end()) && (iter->second == -1.0);

    // map trigger axis
    sensor_msgs::JoyPtr mappedMsg = mapTriggerAxes(msg);

    // compute flipper commands
    joyToFlipperCommand(mappedMsg);

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
    return switchControllers(teleop_controllers_, standard_controllers_);
}

std::string FlipperTeleop::onUnload()
{
    return switchControllers(standard_controllers_, teleop_controllers_);
}

std::string FlipperTeleop::switchControllers(std::vector<std::string> start_controllers,
                                             std::vector<std::string> stop_controllers)
{

    // get controller list to check if the controllers are already running or stopped
    if (list_controllers_client_.call(list_controllers_srv_) && !list_controllers_srv_.response.controller.empty())
    {

        for (auto& controller : list_controllers_srv_.response.controller)
        {

            // check if current controller name is in one of the start/stop lists
            auto start_iter = std::find(start_controllers.begin(), start_controllers.end(), controller.name);
            auto stop_iter = std::find(stop_controllers.begin(), stop_controllers.end(), controller.name);

            // if the controller was found in one of the lists and has the desired state, don't try to change the state later
            if (start_iter != start_controllers.end() && controller.state == "running")
            {
                start_controllers.erase(start_iter);
            } else if (stop_iter != stop_controllers.end()
                && (controller.state == "stopped" || controller.state == "initialized"))
            {
                stop_controllers.erase(stop_iter);
            }
        }
    }

    // if everything is running/stopped, return
    if (start_controllers.empty() && stop_controllers.empty())
    {
        return "";
    }

    // fill service request
    switch_controller_srv_.request.start_controllers = start_controllers;
    switch_controller_srv_.request.stop_controllers = stop_controllers;
    switch_controller_srv_.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;


    // check if service exists
    if (!switch_controller_client_.exists())
    {
        // use a sleep here, as waitForExistence use real time if ros time is not started yet
        ros::Duration(1).sleep();

        if (!switch_controller_client_.waitForExistence(ros::Duration(2)))
        {
            return "hector_joy_teleop_with_plugins: Service " + switch_controller_client_.getService()
                + " does not exist (timed out after 2 sec).";
        }
    }


    // try to switch controllers several times
    for (int i = 0; i < num_tries_switch_controller_; i++)
    {
        // try to switch controller, if successful: break
        if (switch_controller_client_.call(switch_controller_srv_) && switch_controller_srv_.response.ok)
        {
            break;
        }

        // if controllers are not switched after the last try, send error message
        if (i == (num_tries_switch_controller_ - 1))
        {
            std::stringstream ss;
            for (const std::string& s: start_controllers)
            {
                ss << s << " ";
            }
            return "Failed to switch to flipper controllers: " + ss.str();
        } else
        {
            ROS_INFO_STREAM(
                "hector_joy_teleop_with_plugins: Failed to switch to flipper velocity controllers. Try again in "
                    << sleep_between_tries_sec_.sec << " second(s).");

            // otherwise wait desired time and retry
            sleep_between_tries_sec_.sleep();
        }

    }

    return "";
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
