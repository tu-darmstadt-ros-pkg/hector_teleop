#pragma once

#include <std_msgs/Float64.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

#include "hector_joy_teleop_plugin_msgs/SetProperty.h"

namespace hector_joy_teleop_plugins
{

class ManageProperties : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string, double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

 private:

  void publishPropertyChanged(std::string property_name);

  /**
   * Callback for service to load and unload plugins
   */
  bool SetPropertyServiceCB(hector_joy_teleop_plugin_msgs::SetProperty::Request& request,
                            hector_joy_teleop_plugin_msgs::SetProperty::Response& response);

  ros::Publisher direction_status_pub_;

  ros::ServiceServer set_property_service_;


  bool direction_finished_;
};

}
