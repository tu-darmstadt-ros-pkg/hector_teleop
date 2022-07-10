#pragma once

#include <geometry_msgs/Twist.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>
#include <controller_manager_msgs/SwitchController.h>

#include <hector_joy_teleop_plugins/controller_helper.h>
#include <hector_joy_teleop_plugins/common.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace hector_joy_teleop_plugins
{

class ManipulatorTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string,
                  double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

  std::string onLoad() override;

  std::string onUnload() override;

  bool setHoldMode(bool enabled);
  bool loadDrivingPlugin(bool enabled);


 private:

  /**
   * Checks if message contains pressed buttons for hold_pose, reset_pose, move_tool_center, reset_tool_center and calls
   * the respective services.
   * @param msg
   * @return true if message was used, false otherwise
   */
  bool joyToSpecial(const sensor_msgs::JoyConstPtr& msg);
  geometry_msgs::Twist joyToTwist(const sensor_msgs::JoyConstPtr& msg);
  std_msgs::Float64 joyToGripper(const sensor_msgs::JoyConstPtr& msg);

  double max_speed_linear_;
  double max_speed_angular_;
  double max_gripper_speed_;
  ResponseCurveMode response_curve_;
  std::string driving_plugin_name_;
  std::string manipulator_command_topic_;
  std::string gripper_command_topic_;

  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_command_;

  ros::Publisher gripper_pub_;
  std_msgs::Float64 gripper_command_;

  bool hold_pose_ = false;
  bool move_tool_center_ = false;

  bool hold_finished_;
  bool move_tc_finished_;

  ros::ServiceClient hold_pose_srv_client_;
  ros::ServiceClient reset_pose_srv_client_;
  ros::ServiceClient move_tc_srv_client_;
  ros::ServiceClient reset_tc_srv_client_;
  ros::ServiceClient load_teleop_plugins_srv_client_;



  std::vector<std::string> standard_controllers_;
  std::vector<std::string> teleop_controllers_;

  ControllerHelper controller_helper_;
};

}