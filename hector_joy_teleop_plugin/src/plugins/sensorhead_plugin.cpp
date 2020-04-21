

#include "hector_joy_teleop_plugin/plugins/sensorhead_plugin.h"

SensorheadPlugin::SensorheadPlugin(ros::NodeHandle& nh, ros::NodeHandle& pnh, const ros::Rate& rate) : PluginBase(nh,
                                                                                                                  pnh,
                                                                                                                  "SensorheadPlugin"),
                                                                                                       rate_(rate)
{
    sensorheadCommandOutput = nh_.advertise<geometry_msgs::QuaternionStamped>("camera/command", 10, false);

    sensorhead_mode_            = pnh_.param(plugin_name_ + "/" + "sensorhead_mode", std::string("base_stabilized"));
    sensorhead_speed_           = pnh_.param(plugin_name_ + "/" + "sensorhead_speed", 60.0);
    sensorhead_max_pan_         = pnh_.param(plugin_name_ + "/" + "sensorhead_max_pan", 120.0);
    sensorhead_max_tilt_down_   = pnh_.param(plugin_name_ + "/" + "sensorhead_max_tilt_down", 30.0);
    sensorhead_max_tilt_up_     = pnh_.param(plugin_name_ + "/" + "sensorhead_max_tilt_up", 30.0);

}

void SensorheadPlugin::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    //TODO Problem: using rate (in dt) only works right if as in old package the publishing is called in an extra
    // method which is called in the ros::ok loop using ros::SpinOnce instead of ros::spin in joy_teleop_plugin_node

    double dt = rate_.expectedCycleTime().toSec();

    // pan sensorhead
    try
    {
        int pan_mapping = axes_.at("pan");
        sensorhead_pan_speed_ = msg->axes[pan_mapping] * sensorhead_speed_ * M_PI / 180.0;
        sensorhead_pan_ += dt * sensorhead_pan_speed_;
        if (sensorhead_pan_ > sensorhead_max_pan_ * M_PI / 180.0)
        {
            sensorhead_pan_ = sensorhead_max_pan_ * M_PI / 180.0;
        }

        if (sensorhead_pan_ < -sensorhead_max_pan_ * M_PI / 180.0)
        {
            sensorhead_pan_ = -sensorhead_max_pan_ * M_PI / 180.0;
        }
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
        sensorhead_tilt_ += dt * sensorhead_tilt_speed_;
        if (sensorhead_tilt_ > sensorhead_max_tilt_down_ * M_PI / 180.0)
        {
            sensorhead_tilt_ = sensorhead_max_tilt_down_ * M_PI / 180.0;
        }
        if (sensorhead_tilt_ < -sensorhead_max_tilt_up_ * M_PI / 180.0)
        {
            sensorhead_tilt_ = -sensorhead_max_tilt_up_ * M_PI / 180.0;
        }
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("tilt");
    }

    // reset sensorhead position
    try
    {
        int reset_mapping = buttons_.at("reset");
        if (msg->buttons[reset_mapping])
        {
            sensorhead_pan_ = 0;
            sensorhead_tilt_ = 0;
        }
    }
    catch (std::out_of_range& e)
    {
        printMissingParameter("reset");
    }

    // publish command
    sensorheadCommand.header.stamp = ros::Time::now();
    sensorheadCommand.header.frame_id = sensorhead_mode_;
    sensorheadCommand.quaternion.w =  cos(sensorhead_pan_ / 2) * cos(sensorhead_tilt_ / 2);
    sensorheadCommand.quaternion.x = -sin(sensorhead_pan_ / 2) * sin(sensorhead_tilt_ / 2);
    sensorheadCommand.quaternion.y =  cos(sensorhead_pan_ / 2) * sin(sensorhead_tilt_ / 2);
    sensorheadCommand.quaternion.z =  sin(sensorhead_pan_ / 2) * cos(sensorhead_tilt_ / 2);

    sensorheadCommandOutput.publish(sensorheadCommand);

}
