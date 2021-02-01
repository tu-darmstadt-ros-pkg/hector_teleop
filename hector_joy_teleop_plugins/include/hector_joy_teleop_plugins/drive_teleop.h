#pragma once

#include <geometry_msgs/Twist.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

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

  void executePeriodically(const ros::Rate& rate) override;

 private:
  double speed_;
  double turn_speed_;
  double slow_factor_;
  double very_slow_factor_;

  std::string drive_command_topic_;
  ros::Publisher drive_pub_;
  geometry_msgs::Twist drive_command_;
};

}