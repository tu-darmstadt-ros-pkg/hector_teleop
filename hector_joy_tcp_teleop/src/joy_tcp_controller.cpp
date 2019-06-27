#include <hector_joy_tcp_teleop/joy_tcp_controller.h>

namespace hector_joy_tcp_teleop {

bool JoyTcpController::init(hardware_interface::JointStateInterface* /*hw*/, ros::NodeHandle& nh)
{
  nh_ = nh;

  // Load params
  joy_topic_ = nh.param<std::string>("joy_topic", "/joy");
  twist_topic_ = nh.param<std::string>("twist_topic", "/cmd_tcp_twist");
  loadJoystickConfig(nh_);
  nh.param("max_speed_linear", max_speed_linear_, 0.05);
  nh.param("max_speed_angular", max_speed_angular_, 0.01);

  return true;
}

void JoyTcpController::starting(const ros::Time&)
{
  // Reset twist
  twist_.linear.x = twist_.linear.y = twist_.linear.z = 0;
  twist_.angular.x = twist_.angular.y = twist_.angular.z = 0;

  // Set up subscribers and publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_, 10, false);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 10, &JoyTcpController::joyCb, this);
}

void JoyTcpController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  twist_pub_.publish(twist_);
}

void JoyTcpController::stopping(const ros::Time&)
{
  // Shutdown subscribers / publishers
  joy_sub_.shutdown();
  twist_pub_.shutdown();
}

void JoyTcpController::joyCb(const sensor_msgs::JoyConstPtr& joy_ptr)
{
  twist_ = joyToTwist(*joy_ptr);
}

geometry_msgs::Twist JoyTcpController::joyToTwist(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.x = max_speed_linear_ * config_["translate_x"]->computeCommand(joy);
  twist.linear.y = max_speed_linear_ * config_["translate_y"]->computeCommand(joy);
  twist.linear.z = max_speed_linear_ * config_["translate_z"]->computeCommand(joy);

  twist.angular.x = max_speed_angular_ * config_["rotate_roll"]->computeCommand(joy);
  twist.angular.y = max_speed_angular_ * config_["rotate_pitch"]->computeCommand(joy);
  twist.angular.z = max_speed_angular_ * config_["rotate_yaw"]->computeCommand(joy);

  return twist;
}

void JoyTcpController::loadJoystickConfig(const ros::NodeHandle& nh)
{
  ROS_INFO_STREAM("Loading controller config from namespace " << nh.getNamespace() + "/controller_configuration");
  XmlRpc::XmlRpcValue mapping;
  nh.getParam("controller_configuration", mapping);
  ROS_ASSERT(mapping.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = mapping.begin(); it != mapping.end(); ++it)
  {
    std::string action_name = it->first;
    ros::NodeHandle action_nh(nh, "controller_configuration/" + action_name);
    std::shared_ptr<ControllerMapperBase> controller_mapper = createControllerMapper(action_nh);
    config_.emplace(action_name, controller_mapper);
    ROS_INFO_STREAM("Added action: " << action_name);
  }
  if (config_.empty()) {
    ROS_WARN_STREAM("No controller configuration defined");
  }
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_tcp_teleop::JoyTcpController, controller_interface::ControllerBase);
