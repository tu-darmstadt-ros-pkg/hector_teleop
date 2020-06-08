#pragma once

#include <map>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace hector_joy_teleop_plugin_interface
{

class TeleopBase
{
 public:
  void initializeBase(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string plugin_name);

  bool isActive();

  void setActive(bool active);

  std::string getPluginName();

  /**
   * get button mappings of plugin
   * @return first: axes map, second: buttons map
   */
  std::pair<std::map<std::string, int>&, std::map<std::string, int>&> getMapping();

  /**
   * initialize method for plugins
   * @param nh nodehandle
   * @param pnh nodehandle for private namespace
   */
  virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) = 0;

  /**
   * Convert and forward joy message to respective topic/package. Details in each plugin.
   */
  virtual void forwardMsg(const sensor_msgs::JoyConstPtr& msg) = 0;

  /**
   * Method which is called in the while(ros::ok) {ros::SpinOnce(); // called here; ros::Sleep();} loop of node, e.g.
   * for publishing messages periodically.
   * Can be overwritten by plugins if needed.
   * @param rate rate which is used for node and therefore for calling this method
   */
  virtual void executePeriodically(const ros::Rate& rate);

  /**
   * Method which is called after a plugin is loaded successfully, right before active_ is set to true.
   * Can be overwritten by plugins if needed.
   */
  virtual void onLoad();

  /**
   * Method which is called before a plugin is unloaded, right after active_ is set to false.
   * Can be overwritten by plugins if needed.
   */
  virtual void onUnload();

  virtual ~TeleopBase();

 protected:

  void printMissingParameter(std::string param_name);

  /**
   * Get the string "<namespace>/<classname>" as prefix for getting parameters from parameter server
   * @return
   */
  std::string getParameterServerPrefix();

  bool active_ = false;

  // mapping of usage names to message array index
  std::map<std::string, int> axes_;
  std::map<std::string, int> buttons_;
  std::string plugin_namespace_; // <<< plugin namespace (first part from <namespace>::<classname> of complete pluginname used for service calls)
  std::string plugin_name_; //<<< plugin name: <classname>

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

};
}