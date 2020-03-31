#pragma once

#include <map>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "plugin_base.h"

class JoyTeleopPlugin
{
 public:
  JoyTeleopPlugin(ros::NodeHandle& nh);
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);

 private:

  bool updateMapping(std::unique_ptr<PluginBase>& plugin);

  ros::Subscriber joy_sub_;

  // store current gamepad mapping (index in vector is index in joy message array, string is pluginname)
  std::map<int, std::string> axes_;
  std::map<int, std::string> buttons_;

  std::vector<std::unique_ptr<PluginBase>> plugins_;

};
