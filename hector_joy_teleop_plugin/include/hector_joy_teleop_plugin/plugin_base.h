#pragma once

#include <map>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class PluginBase
{
 public:
  PluginBase(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string plugin_name);

  bool isActive();

  void setActive(bool active);

  std::string getPluginName();

  /**
   * get button mappings of plugin
   * @return first: axes map, second: buttons map
   */
  std::pair<std::map<std::string, int>&, std::map<std::string, int>&> getMapping();

  /**
   * Convert and forward joy message to respective topic/package. Details in each plugin.
   */
  virtual void forwardMsg(const sensor_msgs::JoyConstPtr& msg) = 0;

 protected:
  bool active_ = false;

  // mapping of usage names to message array index
  std::map<std::string, int> axes_;
  std::map<std::string, int> buttons_;
  std::string plugin_name_;

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

};
