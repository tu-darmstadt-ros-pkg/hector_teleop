#pragma once

#include <geometry_msgs/QuaternionStamped.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

namespace hector_joy_teleop_plugins
{

class FlipperTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{

 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

 private:

  geometry_msgs::QuaternionStamped sensorheadCommand;
  ros::Publisher sensorheadCommandOutput;

};

}