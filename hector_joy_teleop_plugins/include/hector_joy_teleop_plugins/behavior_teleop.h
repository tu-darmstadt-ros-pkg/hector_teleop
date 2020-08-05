#pragma once

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

#include <actionlib/client/simple_action_client.h>
#include <flexbe_msgs/BehaviorExecutionAction.h>


namespace hector_joy_teleop_plugins
{

class BehaviorTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{

 public:

  BehaviorTeleop();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

 private:

  void doneCB(const actionlib::SimpleClientGoalState& state, const flexbe_msgs::BehaviorExecutionResultConstPtr& result);

  actionlib::SimpleActionClient<flexbe_msgs::BehaviorExecutionAction> action_client_;

  bool behavior_started = false;
};

}