#pragma once

#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <hector_joy_teleop_plugin_interface/teleop_base.h>
#include <pluginlib/class_list_macros.h>

namespace hector_joy_teleop_plugins
{

class ImageProjectionTeleop : public hector_joy_teleop_plugin_interface::TeleopBase
{
  const std::string HFOV_PARAMETER_NAME = "horizontal_fov";
public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map) override;

  void forwardMsg(const sensor_msgs::JoyConstPtr& msg) override;

  void executePeriodically(const ros::Rate& rate) override;

private:
  void poseUpdateCallback(const dynamic_reconfigure::ConfigConstPtr& config_ptr);
  void hfovUpdateCallback(const dynamic_reconfigure::ConfigConstPtr& config_ptr);
  void resetCommands();

  void publishPoseCommand();
  void publishHFOVCommand();
  double constrainAngle(double angle);
  double limitValue(double value, double min, double max);

  double speed_factor_;
  double zoom_speed_factor_;

  double pan_;
  double tilt_;
  double hfov_; /// horizontal fov of projection in degree, acts like a zoom

  double pan_speed_;
  double tilt_speed_;
  double hfov_speed_;

  bool default_pose_received_;
  bool default_hfov_received_;
  geometry_msgs::Point default_position_;
  double default_pan_;
  double default_tilt_;
  double default_hfov_;

  geometry_msgs::Pose command_pose_;

  std::string pose_command_topic_;
  std::string hfov_command_topic_;
  std::string pose_update_topic_;
  std::string hfov_update_topic_;

  ros::ServiceClient hfov_client_;
  ros::Publisher pose_cmd_pub_;
  ros::Subscriber pose_update_sub_;
  ros::Subscriber hfov_update_sub_;
};

}
