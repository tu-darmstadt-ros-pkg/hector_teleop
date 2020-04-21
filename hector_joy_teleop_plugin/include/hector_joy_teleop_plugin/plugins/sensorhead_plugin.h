#pragma once

#include <geometry_msgs/QuaternionStamped.h>
#include "hector_joy_teleop_plugin/plugin_base.h"


class SensorheadPlugin : public PluginBase
{
 public:
  SensorheadPlugin(ros::NodeHandle& nh, ros::NodeHandle& pnh, const ros::Rate& rate);

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

 private:
  const ros::Rate& rate_;

  geometry_msgs::QuaternionStamped sensorheadCommand;
  ros::Publisher sensorheadCommandOutput;


  std::string sensorhead_mode_;
  double sensorhead_speed_;

  double sensorhead_pan_ = 0;
  double sensorhead_tilt_ = 0;

  double sensorhead_pan_speed_;
  double sensorhead_tilt_speed_;
  double sensorhead_max_pan_;
  double sensorhead_max_tilt_down_;
  double sensorhead_max_tilt_up_;
};
