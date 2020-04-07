#pragma once

#include <map>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "hector_joy_teleop_plugin/plugins/drive_plugin.h"
#include "hector_joy_teleop_plugin/plugin_base.h"

#include "hector_joy_teleop_plugin/LoadTeleopPlugin.h"

class JoyTeleopPlugin
{
 public:
  JoyTeleopPlugin(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:

  /**
   * forward message to all active plugins
   */
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * Callback for service to load and unload plugins
   */
  bool LoadPluginServiceCB(hector_joy_teleop_plugin::LoadTeleopPlugin::Request& request,
                           hector_joy_teleop_plugin::LoadTeleopPlugin::Response& response);

  /**
   * add mapping of passed plugin to current mapping, if not possible it prints a ROS_ERROR, restores old mapping and returns the name of the overlapping plugin
   * @param plugin plugin whose mapping shall be added
   * @return empty string if was sucessful, otherwise name of plugin whose button mapping is overlapping
   */
  std::string addMapping(std::unique_ptr<PluginBase>& plugin);

  /**
   * remove mapping of passed plugin from current mapping
   * @param plugin_name plugin whose mapping shall be removed
   */
  void removeMapping(std::string plugin_name);

  /**
   * method called by service callback and constructor for initial loaded plugins
   * @param plugin
   * @return name of plugin whose mapping overlaps with "plugin" or empty string if loading was successful
   */
  std::string loadPlugin(std::unique_ptr<PluginBase>& plugin);

  /**
   * unloads a plugin, set it unactive, removes its button mapping from the current mapping
   * @param plugin
   */
  void unloadPlugin(std::unique_ptr<PluginBase>& plugin);

  /**
   * Factory for plugins, checks if an instance of plugin_name was created before,
   * if yes, it returns its index in vector plugins_ otherwise creates it and return its index.
   * If plugin_name is no valid name, -1 is returned.
   * @param plugin_name name of plugin of which an instance should be returned
   * @return index of (new or old) instance in vector, -1 if plugin_name was not found
   */
  int pluginFactory(std::string plugin_name);


  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  ros::Subscriber joy_sub_;

  ros::ServiceServer load_plugin_service_;

  // store current gamepad mapping (index in vector is index in joy message array, string is pluginname)
  std::map<int, std::string> axes_;
  std::map<int, std::string> buttons_;

  std::vector<std::unique_ptr<PluginBase>> plugins_;

};
