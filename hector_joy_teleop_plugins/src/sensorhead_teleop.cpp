

#include "hector_joy_teleop_plugins/sensorhead_teleop.h"


namespace hector_joy_teleop_plugins
{

void SensorheadTeleop::initialize(ros::NodeHandle& nh,
                                  ros::NodeHandle& pnh,
                                  std::shared_ptr<std::map<std::string, double>> property_map,
                                  std::string plugin_name)
{
    TeleopBase::initializeBase(nh, pnh, property_map, plugin_name, "hector_joy_teleop_plugins::SensorheadTeleop");

    sensorhead_mode_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_mode", std::string("base_stabilized"));
    sensorhead_speed_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_speed", 60.0);

    sensorhead_max_tilt_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_tilt", 0.73);
    sensorhead_min_tilt_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_min_tilt", -1.27);
    sensorhead_max_pan_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_pan", 1.57);
    sensorhead_min_pan_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_min_pan", -1.57);

    sensorhead_tilt_speed_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_tilt_inverted", false);

    sensorhead_command_topic_ =
            pnh_.param<std::string>(getParameterServerPrefix() + "/" + "sensorhead_command_topic", "camera/command");

    sensorhead_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(sensorhead_command_topic_, 10, false);

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
    if (sensorhead_pan_ > sensorhead_max_pan_)
    {
        sensorhead_pan_ = sensorhead_max_pan_;
    }
    if (sensorhead_pan_ < sensorhead_min_pan_)
    {
        sensorhead_pan_ = sensorhead_min_pan_;
    }

    sensorhead_tilt_ -= dt * sensorhead_tilt_speed_;
    if (sensorhead_tilt_ > sensorhead_max_tilt_)
    {
        sensorhead_tilt_ = sensorhead_max_tilt_;
    }
    if (sensorhead_tilt_ < sensorhead_min_tilt_)
    {
        sensorhead_tilt_ = sensorhead_min_tilt_;
    }

    publishCommand();

}

void SensorheadTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    // pan sensorhead
    float pan_joystick;
    if (getJoyMeasurement("pan", msg, pan_joystick))
    {
        sensorhead_pan_speed_ = pan_joystick * sensorhead_speed_ * M_PI / 180.0;
    }

    // tilt sensorhead
    float tilt_joystick;
    if (getJoyMeasurement("tilt", msg, tilt_joystick))
    {
        if(sensorhead_tilt_inverted_)
        {
            sensorhead_tilt_speed_ = tilt_joystick * sensorhead_speed_ * M_PI / 180.0;
        }
        else
        {
          sensorhead_tilt_speed_ = -tilt_joystick * sensorhead_speed_ * M_PI / 180.0;
        }

    }


    // reset sensorhead position (and publish immediately as the position is not computed using the rate)
    float reset_joystick;
    if (getJoyMeasurement("reset", msg, reset_joystick))
    {
        if (reset_joystick)
        {
            sensorhead_pan_ = 0;
            sensorhead_tilt_ = 0;
            publishCommand();
        }

    }
}

void SensorheadTeleop::publishCommand()
{
    // publish command
    sensorhead_command_.header.stamp = ros::Time::now();
    sensorhead_command_.header.frame_id = sensorhead_mode_;
    sensorhead_command_.quaternion.w = cos(sensorhead_pan_ / 2) * cos(sensorhead_tilt_ / 2);
    sensorhead_command_.quaternion.x = -sin(sensorhead_pan_ / 2) * sin(sensorhead_tilt_ / 2);
    sensorhead_command_.quaternion.y = cos(sensorhead_pan_ / 2) * sin(sensorhead_tilt_ / 2);
    sensorhead_command_.quaternion.z = sin(sensorhead_pan_ / 2) * cos(sensorhead_tilt_ / 2);

    sensorhead_pub_.publish(sensorhead_command_);
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::SensorheadTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
