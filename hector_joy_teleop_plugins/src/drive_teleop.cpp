
#include "hector_joy_teleop_plugins/drive_teleop.h"

namespace hector_joy_teleop_plugins
{

void DriveTeleop::initialize(ros::NodeHandle& nh,
                             ros::NodeHandle& pnh,
                             std::shared_ptr<std::map<std::string, double>> property_map,
                             std::string plugin_name)
{
    TeleopBase::initializeBase(nh, pnh, property_map, plugin_name, "hector_joy_teleop_plugins::DriveTeleop");

    critical_stability_reached_ = false;

    // get values from common config file
    ros::NodeHandle param_nh(pnh_, getParameterServerPrefix());
    max_linear_speed_ = param_nh.param<double>("max_linear_speed", 0.0);
    max_angular_speed_ = param_nh.param<double>("max_angular_speed", 0.0);

    slow_factor_ = param_nh.param<double>("slow_factor", 0.5);
    normal_factor_ = param_nh.param<double>("normal_factor", 0.75);
    fast_factor_ = param_nh.param<double>("fast_factor", 1.0);

    critical_stability_lower_threshold_ = param_nh.param<double>("critical_stability_lower_threshold", 0.5);
    critical_stability_upper_threshold_ = param_nh.param<double>("critical_stability_upper_threshold", 0.7);
    stability_margin_topic_ = param_nh.param<std::string>("stability_margin_topic", "stability_margin");

    std::string response_curve = param_nh.param<std::string>("response_curve", "linear");
    if (response_curve == "parabola")
    {
        response_curve_ = ResponseCurveMode::Parabola;
    }
    else if (response_curve == "linear")
    {
        response_curve_ = ResponseCurveMode::Linear;
    }
    else
    {
        ROS_ERROR_NAMED("hector_joy_teleop_with_plugins", "Response curve mode '%s' is unknown. Using linear.", response_curve.c_str());
        response_curve_ = ResponseCurveMode::Linear;
    }

    drive_command_topic_ = param_nh.param<std::string>("drive_command_topic", "cmd_vel");
    drive_pub_ = nh_.advertise<geometry_msgs::Twist>(drive_command_topic_, 10, false);
    stability_margin_sub_ = nh_.subscribe(stability_margin_topic_, 10, &DriveTeleop::stabilityMarginCallback, this);
}

void DriveTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    bool last_command_zero = false;
    if (drive_command_.linear.x == 0 && drive_command_.angular.z == 0)
    {
      last_command_zero = true;
    }

    // if the direction value is available in map use it, otherwise use default value of 1.0
    // (1.0 => forward mode, -1.0 => reverse mode)
    auto iter = property_map_->find("direction");
    double direction = (iter != property_map_->end()) ? iter->second : 1.0;

    // compute linear speed (forward/backward)
    float speed_joystick;
    if (getJoyMeasurement("speed", msg, speed_joystick))
    {
        speed_joystick = applyResponseCurve(speed_joystick, response_curve_);
        drive_command_.linear.x = speed_joystick * max_linear_speed_ * direction;
    }


    // compute angular speed (left/right turn)
    float steer_joystick;
    if (getJoyMeasurement("steer", msg, steer_joystick))
    {
        steer_joystick = applyResponseCurve(steer_joystick, response_curve_);
        drive_command_.angular.z = steer_joystick * max_angular_speed_;
    }


    // compute slow / normal / fast linear and angular speed
    float slow_joystick, fast_joystick;
    if (getJoyMeasurement("slow", msg, slow_joystick, false) && slow_joystick == 1.0)
    {
        drive_command_.linear.x *= slow_factor_;
        drive_command_.angular.z *= slow_factor_;
    }
    else if (getJoyMeasurement("fast", msg, fast_joystick, false) && fast_joystick == 1.0)
    {
        drive_command_.linear.x *= fast_factor_;
        drive_command_.angular.z *= fast_factor_;
    }
    else
    {
        drive_command_.linear.x *= normal_factor_;
        drive_command_.angular.z *= normal_factor_;
    }

    // Check stability margin
    if (critical_stability_reached_) {
      float override_joystick;
      bool override_button_pressed = getJoyMeasurement("stability_override", msg, override_joystick, false) &&
                                     override_joystick == 1.0;
      if (!override_button_pressed) {
        drive_command_.linear.x = 0.0;
        drive_command_.angular.z = 0.0;
      }
    }

    // only publish a zero command, if last command was not zero to avoid interrupting other controllers
    if (last_command_zero && drive_command_.linear.x == 0 && drive_command_.angular.z == 0)
    {
        return;
    }


    drive_pub_.publish(drive_command_);
}

void DriveTeleop::executePeriodically(const ros::Rate& rate)
{
    // The publishing needs to be done here, as the joy messages are published only once when holding a joystick at a
    // fixed position. So if the drive message is only send once, after a timeout of a few seconds the robot stops as
    // there are no new commands. Not done for zero commands to avoid disturbing other modules sending on cmd_vel.

    if (drive_command_.linear.x != 0.0 || drive_command_.angular.z != 0.0)
    {
        drive_pub_.publish(drive_command_);
    }
}
void DriveTeleop::stabilityMarginCallback(const std_msgs::Float64ConstPtr& msg)
{
  if (msg->data < critical_stability_lower_threshold_)
  {
    critical_stability_reached_ = true;
  } else if (msg->data > critical_stability_upper_threshold_)
  {
    critical_stability_reached_ = false;
  }
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::DriveTeleop, hector_joy_teleop_plugin_interface::TeleopBase)