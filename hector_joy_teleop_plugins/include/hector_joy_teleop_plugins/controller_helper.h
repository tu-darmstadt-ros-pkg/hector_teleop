#pragma once

#include <string>

#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

class ControllerHelper
{
 public:

  ControllerHelper() = default;


  ControllerHelper(ros::NodeHandle& pnh,
                   std::string controller_manager_switch_service,
                   std::string controller_manager_list_service,
                   int num_tries_switch_controller,
                   int sleep_time_sec,
                   std::string plugin_name);


  /**
   * Switch to given controllers. Checks if controllers are already started/stopped. If switching was not possible,
   * it is tried again after sleep_time_sec seconds for num_tries_switch_controller times.
   * @param start_controllers list of controllers to start  may be empty
   * @param stop_controllers list of controllers to stop, may be empty
   * @return Error string if something went wrong, empty string otherwise
   */
  std::string switchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers);

 private:

  ros::ServiceClient switch_controller_client_;
  controller_manager_msgs::SwitchController switch_controller_srv_;

  ros::ServiceClient list_controllers_client_;
  controller_manager_msgs::ListControllers list_controllers_srv_;

  int num_tries_switch_controller_;
  ros::Duration sleep_between_tries_sec_;

  std::string plugin_name_;
};