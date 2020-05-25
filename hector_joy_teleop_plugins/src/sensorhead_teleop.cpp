

#include "hector_joy_teleop_plugins/sensorhead_teleop.h"

namespace hector_joy_teleop_plugins
{

void SensorheadTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    TeleopBase::initializeBase(nh, pnh, "hector_joy_teleop_plugins::SensorheadTeleop");

    sensorhead_command_output = nh_.advertise<geometry_msgs::QuaternionStamped>("camera/command", 10, false);

    sensorhead_mode_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_mode", std::string("base_stabilized"));
    sensorhead_speed_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_speed", 60.0);
    sensorhead_max_pan_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_pan", 120.0);
    sensorhead_max_tilt_down_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_tilt_down", 30.0);
    sensorhead_max_tilt_up_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_tilt_up", 30.0);

}

void SensorheadTeleop::executePeriodically(const ros::Rate& rate)
{
    // The publishing needs to be done here, as the joy messages are published only once when holding a joystick at a
    // fixed position. As for the sensorhead here is only a trajectory controller, the goal position is computed every
    // loop using the time gained from rate,the velocity gained from joy message and the current position from the last position
    // and is published.

    // if there are no changes, don't publish anything
    if (sensorhead_pan_speed_ == 0.0 && sensorhead_tilt_speed_ == 0.0)
    {
        return;
    }

    double dt = rate.expectedCycleTime().toSec();

    sensorhead_pan_ += dt * sensorhead_pan_speed_;
    if (sensorhead_pan_ > sensorhead_max_pan_ * M_PI / 180.0)
    {
        sensorhead_pan_ = sensorhead_max_pan_ * M_PI / 180.0;
    }

    if (sensorhead_pan_ < -sensorhead_max_pan_ * M_PI / 180.0)
    {
        sensorhead_pan_ = -sensorhead_max_pan_ * M_PI / 180.0;
    }

    sensorhead_tilt_ += dt * sensorhead_tilt_speed_;
    if (sensorhead_tilt_ > sensorhead_max_tilt_down_ * M_PI / 180.0)
    {
        sensorhead_tilt_ = sensorhead_max_tilt_down_ * M_PI / 180.0;
    }
    if (sensorhead_tilt_ < -sensorhead_max_tilt_up_ * M_PI / 180.0)
    {
        sensorhead_tilt_ = -sensorhead_max_tilt_up_ * M_PI / 180.0;
    }

    publishCommand();

}

void SensorheadTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // pan sensorhead
    try
    {
        int pan_mapping = axes_.at("pan");
        sensorhead_pan_speed_ = msg->axes[pan_mapping] * sensorhead_speed_ * M_PI / 180.0;
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("pan");
    }

    // tilt sensorhead
    try
    {
        int tilt_mapping = axes_.at("tilt");
        sensorhead_tilt_speed_ = msg->axes[tilt_mapping] * sensorhead_speed_ * M_PI / 180.0;
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("tilt");
    }

    // reset sensorhead position (and publish immediately as the position is not computed using the rate)
    try
    {
        int reset_mapping = buttons_.at("reset");
        if (msg->buttons[reset_mapping])
        {
            sensorhead_pan_ = 0;
            sensorhead_tilt_ = 0;
            publishCommand();
        }
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("reset");
    }

}

void SensorheadTeleop::publishCommand()
{
    // publish command
    sensorheadCommand.header.stamp = ros::Time::now();
    sensorheadCommand.header.frame_id = sensorhead_mode_;
    sensorheadCommand.quaternion.w = cos(sensorhead_pan_ / 2) * cos(sensorhead_tilt_ / 2);
    sensorheadCommand.quaternion.x = -sin(sensorhead_pan_ / 2) * sin(sensorhead_tilt_ / 2);
    sensorheadCommand.quaternion.y = cos(sensorhead_pan_ / 2) * sin(sensorhead_tilt_ / 2);
    sensorheadCommand.quaternion.z = sin(sensorhead_pan_ / 2) * cos(sensorhead_tilt_ / 2);

    sensorhead_command_output.publish(sensorheadCommand);
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::SensorheadTeleop, hector_joy_teleop_plugin_interface::TeleopBase)