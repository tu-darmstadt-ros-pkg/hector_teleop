

#include "hector_joy_teleop_plugins/image_projection_teleop.h"

namespace hector_joy_teleop_plugins
{

void ImageProjectionTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map)
{
  TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::SensorheadTeleop");

  pan_ = 0;
  tilt_ = 0;
  hfov_ = 130;

  pan_speed_ = 0;
  tilt_speed_ = 0;
  hfov_speed_ = 0;

  default_pose_received_ = false;
  default_hfov_received_ = false;

  default_pan_ = 0;
  default_tilt_ = 0;
  default_hfov_ = 130;

  speed_factor_ = pnh_.param(getParameterServerPrefix() + "/" + "speed_factor", 60.0);
  zoom_speed_factor_ = pnh_.param(getParameterServerPrefix() + "/" + "zoom_speed_factor", 30.0);


  pose_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "pose_command_topic", "camera/command");
  hfov_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "hfov_command_topic", "camera/set_parameters");
  pose_update_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "pose_update_topic", "camera/parameter_updates");
  hfov_update_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "hfov_update_topic", "camera/projection_parameters/parameter_updates");

  pose_cmd_pub_ = nh_.advertise<geometry_msgs::Pose>(pose_command_topic_, 10, false);
  hfov_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(hfov_update_topic_);
  pose_update_sub_ = nh_.subscribe<dynamic_reconfigure::Config>(pose_update_topic_, 10, &ImageProjectionTeleop::poseUpdateCallback, this);
  hfov_update_sub_ = nh_.subscribe<dynamic_reconfigure::Config>(hfov_update_topic_, 10, &ImageProjectionTeleop::hfovUpdateCallback, this);
}

void ImageProjectionTeleop::executePeriodically(const ros::Rate& rate)
{
  // The publishing needs to be done here, as the joy messages are published only once when holding a joystick at a
  // fixed position

  // if there are no changes, don't publish anything
  if (pan_speed_ == 0.0 && tilt_speed_ == 0.0 && hfov_speed_ == 0.0)
  {
    return;
  }

  double dt = rate.expectedCycleTime().toSec();

  pan_ += dt * pan_speed_;
  pan_ = constrainAngle(pan_);

  tilt_ += dt * tilt_speed_;
  tilt_ = constrainAngle(tilt_);

  hfov_ += dt * hfov_speed_;
  hfov_ = limitValue(hfov_, 1, 359);

  publishCommand();
}

void ImageProjectionTeleop::poseUpdateCallback(const dynamic_reconfigure::ConfigConstPtr& config_ptr)
{
  if (default_pose_received_) {
    return;
  }
  for (const dynamic_reconfigure::DoubleParameter& double_param: config_ptr->doubles) {
    if (double_param.name == "pose_x") {
      default_position_.x = double_param.value;
      continue;
    }
    if (double_param.name == "pose_y") {
      default_position_.y = double_param.value;
      continue;
    }
    if (double_param.name == "pose_z") {
      default_position_.z = double_param.value;
      continue;
    }
    if (double_param.name == "pose_pitch") {
      default_tilt_ = double_param.value;
      continue;
    }
    if (double_param.name == "pose_yaw") {
      default_pan_ = double_param.value;
    }
  }
  default_pose_received_ = true;
  pose_update_sub_.shutdown();
}

void ImageProjectionTeleop::hfovUpdateCallback(const dynamic_reconfigure::ConfigConstPtr& config_ptr)
{
  if (default_hfov_received_) {
    return;
  }
  for (const dynamic_reconfigure::DoubleParameter& double_param: config_ptr->doubles) {
    if (double_param.name == HFOV_PARAMETER_NAME) {
      default_hfov_received_ = true;
      default_hfov_ = double_param.value;
      hfov_update_sub_.shutdown();
      break;
    }
  }
  if (!default_hfov_received_) {
    ROS_ERROR_STREAM("Could not find '" << HFOV_PARAMETER_NAME << "' in parameter update");
  }
}

void ImageProjectionTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
  // pan
  float pan_joystick;
  if (getJoyMeasurement("pan", msg, pan_joystick))
  {
    pan_speed_ = static_cast<double>(pan_joystick) * speed_factor_ * M_PI / 180.0;
  }

  // tilt
  float tilt_joystick;
  if (getJoyMeasurement("tilt", msg, tilt_joystick))
  {
    tilt_speed_ = static_cast<double>(tilt_joystick) * speed_factor_ * M_PI / 180.0;
  }

  // hfov
  float hfov_joystick;
  if (getJoyMeasurement("hfov", msg, hfov_joystick))
  {
    hfov_speed_ = static_cast<double>(hfov_joystick) * zoom_speed_factor_;
  }

  // reset  position (and publish immediately as the position is not computed using the rate)
  float reset_joystick;
  if (getJoyMeasurement("reset", msg, reset_joystick))
  {
    if(reset_joystick == 0.0f)
    {
      pan_ = default_pan_;
      tilt_ = default_tilt_;
      hfov_ = default_hfov_;
      publishCommand();
    }
  }
}

void ImageProjectionTeleop::publishCommand()
{
  // publish pose command
  command_pose_.position = default_position_;
  command_pose_.orientation.w = cos(pan_ / 2) * cos(tilt_ / 2);
  command_pose_.orientation.x = -sin(pan_ / 2) * sin(tilt_ / 2);
  command_pose_.orientation.y = cos(pan_ / 2) * sin(tilt_ / 2);
  command_pose_.orientation.z = sin(pan_ / 2) * cos(tilt_ / 2);

  pose_cmd_pub_.publish(command_pose_);

  // publish hfov command
  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = HFOV_PARAMETER_NAME;
  double_param.value = hfov_;
  dynamic_reconfigure::Reconfigure reconfigure_srv;
  reconfigure_srv.request.config.doubles.push_back(double_param);
  if (!hfov_client_.call(reconfigure_srv)) {
    ROS_ERROR_STREAM("Failed to update horizontal fov");
  }
}

double ImageProjectionTeleop::constrainAngle(double angle)
{
  return remainder(angle, 2 * M_PI);
}

double ImageProjectionTeleop::limitValue(double value, double min, double max)
{
  return std::max(std::min(value, max), min);
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::ImageProjectionTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
