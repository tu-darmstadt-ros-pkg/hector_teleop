#pragma once

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcException.h>

#include <hector_joy_teleop_plugin_msgs/LoadTeleopPlugin.h>

namespace hector_joy_teleop_plugins
{

struct Profile
{
  std::string name;
  std::vector<std::string> plugins;
};

class ChangeProfile : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string, double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  std::string onLoad() override;

 private:

  void iterProfile(sensor_msgs::JoyPtr& msg);

  /**
   * set current profile, unload all plugins and load new ones, publishes new profile name
   * @param iter iterator to profiles_ with new profile
   */
  void setCurrentProfile(std::vector<Profile>::iterator iter);

  /**
   * set next profile in list as current, if end is reached set first
   */
  void nextProfile();

  /**
   * set previous profile in list as current, if begin is reached set first
   */
  void previousProfile();

  int change_forward_;
  int change_backward_;
  bool use_buttons_to_iter_;
  int reload_profile_;

  std::vector<Profile> profiles_; ///< list of profiles in same order as in config file

  std::vector<Profile>::iterator current_profile_; ///< Iterator to currently loaded profile

  std::string current_profile_topic_;
  ros::Publisher current_profile_pub_;
  std_msgs::String current_profile_msg_;

  ros::ServiceClient load_teleop_plugins_srv_client_;
  hector_joy_teleop_plugin_msgs::LoadTeleopPlugin load_teleop_plugin_srv_msg_;


};

}