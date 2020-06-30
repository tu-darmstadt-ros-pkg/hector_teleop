
#include "hector_joy_teleop_plugins/drive_teleop.h"

namespace hector_joy_teleop_plugins
{

void DriveTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    TeleopBase::initializeBase(nh, pnh, "hector_joy_teleop_plugins::DriveTeleop");


    // get values from common config file
    speed_forward_ = pnh_.param(getParameterServerPrefix() + "/" + "speed_forward", 0);
    speed_backward_ = pnh_.param(getParameterServerPrefix() + "/" + "speed_backward", 0);
    turn_speed_ = pnh_.param(getParameterServerPrefix() + "/" + "turn_speed", 0);
    slow_factor_ = pnh_.param(getParameterServerPrefix() + "/" + "slow_factor", 0);
    very_slow_factor_ = pnh_.param(getParameterServerPrefix() + "/" + "very_slow_factor", 0);

    motionCommandOutput = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
}

void DriveTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    // compute linear speed (forward/backward)
    try
    {
        int speed_mapping = axes_.at("speed");
        float speed_joystick = msg->axes[speed_mapping];
        if (speed_joystick >= 0.0)
            motionCommand.linear.x = speed_joystick * speed_forward_;
        else
            motionCommand.linear.x = speed_joystick * speed_backward_;
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("speed");
    }


    // compute angular speed (left/right turn)
    try
    {
        int steer_mapping = axes_.at("steer");
        motionCommand.angular.z = msg->axes[steer_mapping] * turn_speed_;
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("steer");
    }


    // compute slow linear and angular speed
    try
    {
        int slow_mapping = axes_.at("slow");
        if (msg->buttons[slow_mapping])
        {
            motionCommand.linear.x *= slow_factor_;
            motionCommand.angular.z *= slow_factor_;
        }
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("slow");
    }


    // compute very slow linear and angular speed
    try
    {
        int very_slow_mapping = axes_.at("very_slow");
        if (msg->buttons[very_slow_mapping])
        {
            motionCommand.linear.x *= very_slow_factor_;
            motionCommand.angular.z *= very_slow_factor_;
        }
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("very_slow");
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