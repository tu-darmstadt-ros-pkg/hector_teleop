#pragma once

#include <geometry_msgs/Twist.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>



namespace hector_joy_teleop_plugins
{

 class DriveTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{

 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

   void executePeriodically(const ros::Rate& rate) override;

 private:
  double speed_forward_;
  double speed_backward_;
  double turn_speed_;
  double slow_factor_;
  double very_slow_factor_;

  ros::Publisher motionCommandOutput;
  geometry_msgs::Twist motionCommand;
};

}