#pragma once

#include <std_msgs/Float64.h>

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

#include <hector_joy_teleop_plugins/controller_helper.h>

#include <flipper_auto_control_msgs/requestAction.h>
#include <flipper_auto_control_msgs/requestGoal.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>



namespace hector_joy_teleop_plugins
{

class FlipperTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
typedef actionlib::SimpleActionClient<flipper_auto_control_msgs::requestAction> Client;
 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string, double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  

  std::string onLoad() override;

  std::string onUnload() override;

 private:

  void joyToFlipperCommand(const sensor_msgs::JoyConstPtr& msg);

  /**
  * This function checks if a R1 or L1 have been double-clicked and triggers the flipper_auto_lower feature
  * @param msg the msg of the controller
  * @param val the value of R1 or L1
  */
  void triggerFlipperAuto (const sensor_msgs::JoyConstPtr& msg, float val, std::string dir);

  float speed_;
  float flipper_front_factor_;
  float flipper_back_factor_;
  bool flipper_auto_lower_feature_ = true;
  double f_first_,b_first_;
  bool f_set_,b_set_,f_inter_,b_inter_,flipper_auto_lower_feature_running_;

  std::string flipper_front_command_topic_;
  std::string flipper_back_command_topic_;

  std_msgs::Float64 flipper_front_command_;
  std_msgs::Float64 flipper_back_command_;

  ros::Publisher flipper_front_pub_;
  ros::Publisher flipper_back_pub_;


  std::vector<std::string> standard_controllers_;
  std::vector<std::string> teleop_controllers_;

  ControllerHelper controller_helper_;
  Client* client_;

};

}
