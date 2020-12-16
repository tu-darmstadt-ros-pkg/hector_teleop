#include "hector_joy_teleop_plugins/controller_helper.h"

ControllerHelper::ControllerHelper(ros::NodeHandle& pnh,
                                   std::string controller_manager_switch_service,
                                   std::string controller_manager_list_service,
                                   int num_tries_switch_controller,
                                   int sleep_time_sec,
                                   std::string plugin_name)
{

    switch_controller_client_ =
        pnh.serviceClient<controller_manager_msgs::SwitchController>(controller_manager_switch_service);

    list_controllers_client_ =
        pnh.serviceClient<controller_manager_msgs::ListControllers>(controller_manager_list_service);


    num_tries_switch_controller_ = num_tries_switch_controller;
    sleep_between_tries_sec_ = ros::Duration(sleep_time_sec);

    plugin_name_ = plugin_name;
}



std::string ControllerHelper::switchControllers(std::vector<std::string> start_controllers,
                                                std::vector<std::string> stop_controllers)
{

    // if nothing shall be started/stopped, return
    if (start_controllers.empty() && stop_controllers.empty())
    {
        return "";
    }

    // get controller list to check if the controllers are already running or stopped
    if (list_controllers_client_.call(list_controllers_srv_) && !list_controllers_srv_.response.controller.empty())
    {

        for (auto& controller : list_controllers_srv_.response.controller)
        {

            // check if current controller name is in one of the start/stop lists
            auto start_iter = std::find(start_controllers.begin(), start_controllers.end(), controller.name);
            auto stop_iter = std::find(stop_controllers.begin(), stop_controllers.end(), controller.name);

            // if the controller was found in one of the lists and has the desired state, do not try to change the state later
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
        // wait until rostime has started
        ros::Time::waitForValid();

        if (!switch_controller_client_.waitForExistence(ros::Duration(2)))
        {
            return "hector_joy_teleop_with_plugins: Service " + switch_controller_client_.getService()
                + " requested from plugin " + plugin_name_ + " does not exist (timed out after 2 sec).";
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
            return "Failed to switch to controllers for plugin " + plugin_name_ + " : " + ss.str();
        } else
        {
            ROS_INFO_STREAM(
                "hector_joy_teleop_with_plugins: Failed to switch controllers for plugin " + plugin_name_ + ". Try again in "
                    << sleep_between_tries_sec_.sec << " second(s).");

            // otherwise wait desired time and retry
            sleep_between_tries_sec_.sleep();
        }

    }

    return "";
}