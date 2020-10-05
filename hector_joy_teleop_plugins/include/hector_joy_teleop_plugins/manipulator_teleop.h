#pragma once

#include <geometry_msgs/Twist.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>
#include <controller_manager_msgs/SwitchController.h>

namespace hector_joy_teleop_plugins
{

class ManipulatorTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

  std::string onLoad() override;

  std::string onUnload() override;


 private:

  geometry_msgs::Twist joyToTwist(const sensor_msgs::JoyConstPtr& msg);

  double max_speed_linear_;
  double max_speed_angular_;

  std::string manipulator_command_topic_;

  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_command_;


  std::string controller_manager_switch_service_;
  std::vector<std::string> standard_controllers_;
  std::vector<std::string> teleop_controllers_;

  ros::ServiceClient switch_controller_client_;
  controller_manager_msgs::SwitchController switch_controller_srv_;


};

}