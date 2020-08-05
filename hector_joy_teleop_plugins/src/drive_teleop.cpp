
#include "hector_joy_teleop_plugins/drive_teleop.h"

namespace hector_joy_teleop_plugins
{

void DriveTeleop::initialize(ros::NodeHandle& nh,
                             ros::NodeHandle& pnh,
                             std::shared_ptr<std::map<std::string, double>> property_map)
{
    TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::DriveTeleop");

    // get values from common config file
    speed_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "speed", 0.0);
    turn_speed_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "turn_speed", 0.0);
    slow_factor_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "slow_factor", 1.0);
    very_slow_factor_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "very_slow_factor", 1.0);

    motionCommandOutput = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
}

void DriveTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    // if the direction value is available in map use it, otherwise use default value of 1.0
    // (1.0 => forward mode, -1.0 => reverse mode)
    auto iter = property_map_->find("direction");
    double direction = (iter != property_map_->end()) ? iter->second : 1.0;

    // compute linear speed (forward/backward)
    float speed_joystick;
    if (getJoyMeasurement("speed", msg, speed_joystick))
    {
        motionCommand.linear.x = speed_joystick * speed_ * direction;
    }


    // compute angular speed (left/right turn)
    float steer_joystick;
    if (getJoyMeasurement("steer", msg, steer_joystick))
    {
        motionCommand.angular.z = steer_joystick * turn_speed_;
    }


    // compute slow linear and angular speed
    float slow_joystick;
    if (getJoyMeasurement("slow", msg, slow_joystick))
    {
        if (slow_joystick == 1.0)
        {
            motionCommand.linear.x *= slow_factor_;
            motionCommand.angular.z *= slow_factor_;
        }
    }


    // compute very slow linear and angular speed
    float very_slow_joystick;
    if (getJoyMeasurement("very_slow", msg, very_slow_joystick))
    {
        if (very_slow_joystick == 1.0)
        {
            motionCommand.linear.x *= very_slow_factor_;
            motionCommand.angular.z *= very_slow_factor_;
        }
    }

    motionCommandOutput.publish(motionCommand);
}

void DriveTeleop::executePeriodically(const ros::Rate& rate)
{
    // The publishing needs to be done here, as the joy messages are published only once when holding a joystick at a
    // fixed position. So if the drive message is only send once, after a timeout of a few seconds the robot stops as
    // there are no new commands. Not done for zero commands to avoid disturbing other modules sending on cmd_vel.

    if (motionCommand.linear.x != 0.0 || motionCommand.angular.z != 0.0)
    {
        motionCommandOutput.publish(motionCommand);
    }
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::DriveTeleop, hector_joy_teleop_plugin_interface::TeleopBase)