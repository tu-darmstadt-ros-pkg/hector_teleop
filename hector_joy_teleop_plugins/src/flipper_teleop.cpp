
#include "hector_joy_teleop_plugins/flipper_teleop.h"

namespace hector_joy_teleop_plugins
{

void FlipperTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    TeleopBase::initializeBase(nh, pnh, "hector_joy_teleop_plugins::FlipperTeleop");


    nh_.advertise<geometry_msgs::QuaternionStamped>("/flipper_control/flipper_traj_controller/command", 10, false);

}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

}

void FlipperTeleop::executePeriodically(const ros::Rate& rate)
{
    double dt = rate.expectedCycleTime().toSec();
}

}


PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)