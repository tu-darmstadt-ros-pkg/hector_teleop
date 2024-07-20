#pragma once

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>
#include <hector_joy_teleop_plugins/common.h>


namespace hector_joy_teleop_plugins
{

class DriveTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{

 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string, double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void stabilityMarginCallback(const std_msgs::Float64ConstPtr& msg);

 private:
  double max_linear_speed_;
  double max_angular_speed_;

  double slow_factor_;
  double normal_factor_;
  double fast_factor_;

  bool wb_assistance_enabled_;

  bool critical_stability_reached_;
  double critical_stability_lower_threshold_;
  double critical_stability_upper_threshold_;
  std::string stability_margin_topic_;

  bool use_wb_assistance_;

  std::string drive_command_topic_;
  ResponseCurveMode response_curve_;
  ros::Publisher drive_pub_;
  ros::Publisher wb_assistance_enable_pub_;
  ros::Subscriber stability_margin_sub_;
  geometry_msgs::Twist drive_command_;
};

}