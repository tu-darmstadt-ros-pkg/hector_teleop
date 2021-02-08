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
  void initializeBase(ros::NodeHandle& nh,
                      ros::NodeHandle& pnh,
                      std::shared_ptr<std::map<std::string, double>> property_map,
                      std::string plugin_name,
                      std::string plugin_type);

  bool isActive();

  void setActive(bool active);

  std::string getPluginName();

  std::string getPluginType();

  /**
   * get button mappings of plugin
   * @return first: axes map, second: buttons map
   */
  std::pair<std::map<std::string, int>&, std::map<std::string, int>&> getMapping();

  /**
   * initialize method for plugins
   * @param nh nodehandle
   * @param pnh nodehandle for private namespace
   * @param property_map shared pointer to property map
   * @param plugin_name name of plugin (not type!)
   */
  virtual void initialize(ros::NodeHandle& nh,
                          ros::NodeHandle& pnh,
                          std::shared_ptr<std::map<std::string, double>> property_map,
                          std::string plugin_name) = 0;

  /**
   * Convert and forward joy message to respective topic/package. Details in each plugin.
   */
  virtual void forwardMsg(const sensor_msgs::JoyConstPtr& msg) = 0;

  /**
   * if the value of used_mags_ is set in forwardMsg implementation, this method can be used to check,
   * whether the buttons for this plugin were pressed or not.
   * The method resets used_msgs_ to false.
   * @return true if in forwardMsg the message was used (buttons/axes of this plugin pressed)
   */
  virtual bool hasUsedMsg();

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
   * @return empty string if successful, otherwise an error string describing the problem that occurred
   */
  virtual std::string onLoad();

  /**
   * Method which is called before a plugin is unloaded, right after active_ is set to false.
   * Can be overwritten by plugins if needed.
   * @return empty string if successful, otherwise an error string describing the problem that occurred
   */
  virtual std::string onUnload();

  virtual ~TeleopBase();

 protected:

  /**
   * get the joy measurements for a given name from axes or buttons list of message. Returns false if name cannot be found.
   * @param name name of mapping for measurement
   * @param msg message of which the measurement will be taken
   * @param result measurement for name
   * @param print_missing_parameter true by default, if false there will no message be printed if the parameter is
   *                                        missing, but false will be returned
   *                                        (set to false e.g. when there are alternative names which are checked one by one)
   * @return false if name was not found in buttons_, axes_ or axis_split mapping
   */
  bool getJoyMeasurement(std::string name,
                         const sensor_msgs::JoyConstPtr& msg,
                         float& result,
                         bool print_missing_parameter = true);

  /**
   * Map the trigger axes from [-1,1] (with 1 as default) to [0,1] (with 0 as default)
   * (the index of the trigger axes are defined with the left_trigger and right_trigger parameters on parameter server)
   * @param msg original message
   * @return message with mapped trigger axes
   */
  sensor_msgs::JoyPtr mapTriggerAxes(const sensor_msgs::JoyConstPtr& msg);

  /**
   * Get the string "<namespace>/<plugin_type_>/<plugin_name>" as prefix for getting parameters from parameter server
   * @return
   */
  std::string getParameterServerPrefix();

  bool active_ = false;

  // mapping of usage names to message array index
  std::map<std::string, int> axes_;
  std::map<std::string, int> buttons_;

  // contains entries where an axis is used as two buttons or two partial axes,
  // first part of pair is the index in axes array, second part a bool if it is the < 0 part of axis (false) or the > 0 part (true)
  // WARNING: if an axis is used as two buttons or partial axes there must be (up to) two entries in this map and additionally one entry
  // in the axes map to check overlapping mappings
  std::map<std::string, std::pair<int, bool>> axis_split_;

  std::string plugin_namespace_; //<<< plugin namespace (first part from <namespace>::<classname> of complete plugintype used for service calls)
  std::string plugin_name_; //<<< plugin name
  std::string plugin_type_; //<<< <namespace>::<classname>

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::shared_ptr<std::map<std::string, double>> property_map_;

  bool used_msg_ = false;

  int left_trigger_;
  int right_trigger_;

};
}