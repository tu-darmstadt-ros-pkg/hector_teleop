#pragma once

#include <std_msgs/Float64.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

namespace hector_joy_teleop_plugins
{

class FlipperTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{

 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  std::string onLoad() override;

  std::string onUnload() override;

 private:

  void joyToFlipperCommand(const sensor_msgs::JoyConstPtr& msg);

  std::string switchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers);


  float speed_;
  std::string flipper_front_command_topic_;
  std::string flipper_back_command_topic_;

  std::string controller_manager_switch_service_;
  std::string controller_manager_list_service_;

  std::vector<std::string> standard_controllers_;
  std::vector<std::string> teleop_controllers_;

  std_msgs::Float64 flipper_front_command_;
  std_msgs::Float64 flipper_back_command_;

  ros::Publisher flipper_front_pub_;
  ros::Publisher flipper_back_pub_;

  ros::ServiceClient switch_controller_client_;
  controller_manager_msgs::SwitchController switch_controller_srv_;

  ros::ServiceClient list_controllers_client_;
  controller_manager_msgs::ListControllers list_controllers_srv_;

  int num_tries_switch_controller_;
  ros::Duration sleep_between_tries_sec_;


};

}
