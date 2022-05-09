#pragma once

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

namespace hector_joy_teleop_plugins
{

class SensorheadTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
 public:
  void initialize(ros::NodeHandle& nh,
                  ros::NodeHandle& pnh,
                  std::shared_ptr<std::map<std::string, double>> property_map,
                  std::string plugin_name) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

 private:


  void publishTwistCommand();

  std::string sensorhead_mode_;
  double sensorhead_speed_;

  //double sensorhead_pan_ = 0;
  //double sensorhead_tilt_ = 0;

  //double sensorhead_pan_speed_ = 0;
  //double sensorhead_tilt_speed_ = 0;

  //double sensorhead_max_pan_;
  //double sensorhead_min_pan_;
  //double sensorhead_max_tilt_;
  //double sensorhead_min_tilt_;

  bool sensorhead_tilt_inverted_;
  //float pan_joystick_;
  //float tilt_joystick_;
  double pan_twist_;
  double tilt_twist_;
  bool reset_=false;

  std::string sensorhead_command_topic_;
  geometry_msgs::Twist sensorhead_command_twist_;
  ros::Publisher sensorhead_pub_twist_;
};

}