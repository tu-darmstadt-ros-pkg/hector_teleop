#pragma once

#include <map>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <xmlrpcpp/XmlRpcException.h>

#include <pluginlib/class_loader.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>

#include "hector_joy_teleop_plugin_msgs/LoadTeleopPlugin.h"


namespace hector_joy_teleop_with_plugins
{

typedef std::shared_ptr<hector_joy_teleop_plugin_interface::TeleopBase> TeleopBasePtr;
typedef pluginlib::ClassLoader<hector_joy_teleop_plugin_interface::TeleopBase> TeleopPluginClassLoader;



class JoyTeleop
{
 public:
  JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  /**
   * Method which is called in the while(ros::ok) {ros::SpinOnce(); // called here; ros::Sleep();} loop of node, e.g.
   * for publishing messages periodically. Calls executePeriodically method of plugins.
   * @param rate rate which is used for node and therefore for calling this method
   */
  void executePeriodically(const ros::Rate& rate);

 private:

  /**
   * forward message to all active plugins
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * Callback for service to load and unload plugins
   */
  bool loadPluginServiceCB(hector_joy_teleop_plugin_msgs::LoadTeleopPlugin::Request& request,
                           hector_joy_teleop_plugin_msgs::LoadTeleopPlugin::Response& response);

  /**
   * add mapping of passed plugin to current mapping, if not possible it prints a ROS_ERROR, restores old mapping and returns the name of the overlapping plugin
   * @param plugin plugin whose mapping shall be added
   * @return empty string if was successful, otherwise name of plugin whose button mapping is overlapping
   */
  std::string addMapping(TeleopBasePtr& plugin);

  /**
   * remove mapping of passed plugin from current mapping
   * @param plugin_name plugin whose mapping shall be removed
   */
  void removeMapping(std::string plugin_name);

  /**
   * Factory for plugins, checks if an instance of plugin_name was created before,
   * if yes, it returns its index in vector plugins_ otherwise creates it and return its index.
   * If plugin_name is no valid name, -1 is returned.
   * @param teleop_plugin_name name of plugin of which an instance should be returned
   * @return index of (new or old) instance in vector, -1 if plugin_name was not found
   */
  int pluginFactory(std::string teleop_plugin_name);

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  std::shared_ptr<std::map<std::string, double>> property_map_;

  ros::Subscriber joy_sub_;


  double joy_timeout_;
  ros::Time last_joy_msg_received_;
  bool timed_out_ = false;

  ros::ServiceServer load_plugin_service_;

  // store current gamepad mapping (index in vector is index in joy message array, string is pluginname)
  std::map<int, std::string> axes_;
  std::map<int, std::string> buttons_;

  std::vector<TeleopBasePtr> plugins_;

  std::pair<std::string, int> top_plugin_; //<< Name and index of top plugin

  std::map<std::string, std::string> plugin_names_types_; //<<< all available plugins with name and type

  TeleopPluginClassLoader teleop_plugin_class_loader_;

};

}
