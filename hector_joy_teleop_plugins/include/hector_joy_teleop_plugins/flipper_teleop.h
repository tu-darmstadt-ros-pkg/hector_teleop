#pragma once

#include <std_msgs/Float64.h>
#include <controller_manager_msgs/SwitchController.h>

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

namespace hector_joy_teleop_plugins
{

class FlipperTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{

 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  std::string onLoad() override;

  std::string onUnload() override;

 private:

  float speed_;

  std_msgs::Float64 flipper_front_command_;
  std_msgs::Float64 flipper_back_command_;
  ros::Publisher flipper_front_pub_;
  ros::Publisher flipper_back_pub_;

  ros::ServiceClient switch_controller_client_;
  controller_manager_msgs::SwitchController switch_controller_srv_;

};

}