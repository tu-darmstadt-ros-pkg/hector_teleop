#pragma once

#include <geometry_msgs/Twist.h>
#include "hector_joy_teleop_plugin/plugin_base.h"

class DrivePlugin : public PluginBase
{

 public:
  DrivePlugin(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

 private:
  double speed_forward_;
  double speed_backward_;
  double turn_speed_;
  double slow_factor_;
  double very_slow_factor_;

  ros::Publisher motionCommandOutput;
  geometry_msgs::Twist motionCommand;
};
