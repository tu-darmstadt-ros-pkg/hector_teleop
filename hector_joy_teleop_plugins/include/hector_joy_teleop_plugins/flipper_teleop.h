#pragma once

#include <std_msgs/Float64.h>

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

#include <hector_joy_teleop_plugins/controller_helper.h>

namespace hector_joy_teleop_plugins
{

class FlipperTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string, double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  std::string onLoad() override;

  std::string onUnload() override;

  void stabilityMarginCallback(const std_msgs::Float64ConstPtr& msg);

 private:

  void joyToFlipperCommand(const sensor_msgs::JoyConstPtr& msg);

  float speed_;
  float flipper_front_factor_;
  float flipper_back_factor_;

  bool critical_stability_reached_;
  double critical_stability_lower_threshold_;
  double critical_stability_upper_threshold_;
  std::string stability_margin_topic_;

  std::string flipper_front_command_topic_;
  std::string flipper_back_command_topic_;

  std_msgs::Float64 flipper_front_command_;
  std_msgs::Float64 flipper_back_command_;

  ros::Publisher flipper_front_pub_;
  ros::Publisher flipper_back_pub_;

  ros::Subscriber stability_margin_sub_;


  std::vector<std::string> standard_controllers_;
  std::vector<std::string> teleop_controllers_;

  ControllerHelper controller_helper_;

};

}
