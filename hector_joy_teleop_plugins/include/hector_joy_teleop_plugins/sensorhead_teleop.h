#pragma once

#include <geometry_msgs/QuaternionStamped.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

namespace hector_joy_teleop_plugins
{

class SensorheadTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

 private:

  void publishCommand();

  geometry_msgs::QuaternionStamped sensorheadCommand;
  ros::Publisher sensorhead_command_output;

  std::string sensorhead_mode_;
  double sensorhead_speed_;

  double sensorhead_pan_ = 0;
  double sensorhead_tilt_ = 0;

  double sensorhead_pan_speed_ = 0;
  double sensorhead_tilt_speed_ = 0;
  double sensorhead_max_pan_;
  double sensorhead_max_tilt_down_;
  double sensorhead_max_tilt_up_;
};

}