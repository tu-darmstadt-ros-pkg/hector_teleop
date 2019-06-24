#ifndef HECTOR_JOY_TCP_TELEOP_JOY_TCP_CONTROLLER_H
#define HECTOR_JOY_TCP_TELEOP_JOY_TCP_CONTROLLER_H

#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <eigen3/Eigen/Eigen>

#include <hector_joy_tcp_teleop/controller_mapping/controller_mapper_factory.h>

#include <pluginlib/class_list_macros.h>

namespace hector_joy_tcp_teleop {

class JoyTcpController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
public:
  bool init(hardware_interface::JointStateInterface* /*hw*/, ros::NodeHandle& nh) override;

  void starting(const ros::Time&) override;
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
  void stopping(const ros::Time&) override;

private:
  void joyCb(const sensor_msgs::JoyConstPtr& joy_ptr);
  geometry_msgs::Twist joyToTwist(const sensor_msgs::Joy& joy);
  void loadJoystickConfig(const ros::NodeHandle& nh);


  ros::NodeHandle nh_;
  // Config
  std::map<std::string, std::shared_ptr<ControllerMapperBase>> config_;
  std::string joy_topic_;
  std::string twist_topic_;
  double max_speed_linear_;
  double max_speed_angular_;

  geometry_msgs::Twist twist_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
};

}

#endif
