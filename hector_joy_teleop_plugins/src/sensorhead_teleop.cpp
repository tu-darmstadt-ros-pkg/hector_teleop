

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

  /*  sensorhead_max_tilt_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_tilt", 0.73);
    sensorhead_min_tilt_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_min_tilt", -1.27);
    sensorhead_max_pan_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_max_pan", 1.57);
    sensorhead_min_pan_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_min_pan", -1.57);

    sensorhead_tilt_speed_ = pnh_.param(getParameterServerPrefix() + "/" + "sensorhead_tilt_inverted", false);*/

    sensorhead_command_topic_ =
            pnh_.param<std::string>(getParameterServerPrefix() + "/" + "sensorhead_command_topic", "camera/command");


    sensorhead_pub_twist_ = nh_.advertise<geometry_msgs::Twist>(sensorhead_command_topic_+"_twist", 10, false);

}

void SensorheadTeleop::executePeriodically(const ros::Rate& rate)
{
    // The publishing needs to be done here, as the joy messages are published only once when holding a joystick at a
    // fixed position. As for the sensorhead here is only a trajectory controller, the goal position is computed every
    // loop using the time gained from rate,the velocity gained from joy message and the current position from the last position
    // and is published.

    // if there are no changes, don't publish anything
    if (tilt_twist_ == 0.0 && pan_twist_ == 0.0)
    {
        return;
    }

    // client must check collision for collisions and limits !!
    publishTwistCommand();//send twist command for sensor head orientation

}

void SensorheadTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    // pan sensorhead
    float pan_joystick;
    if (getJoyMeasurement("pan", msg, pan_joystick))
    {
        pan_twist_ = pan_joystick * sensorhead_speed_ * M_PI / 180.0 ;
    }

    // tilt sensorhead
    float tilt_joystick;
    if (getJoyMeasurement("tilt", msg, tilt_joystick))
    {
        if(sensorhead_tilt_inverted_)
        {
            tilt_twist_ = tilt_joystick * sensorhead_speed_ * M_PI / 180.0;
        }
        else
        {
          tilt_twist_ = -tilt_joystick * sensorhead_speed_ * M_PI / 180.0;
        }

    }
    // ROS_INFO_STREAM("pan_joystick "<<pan_joystick<<"  tilt_joystick: "<<tilt_joystick);

    // reset sensorhead position (and publish immediately as the position is not computed using the rate)
    float reset_joystick;
    if (getJoyMeasurement("reset", msg, reset_joystick))
    {
        if (reset_joystick)
        {
            //reset
            reset_ = true;
            publishTwistCommand();// use quaternion msg to reset orientation
        }

    }
}


void SensorheadTeleop::publishTwistCommand()
{
  // publish command
  if(reset_){
    ROS_WARN_STREAM("[publishTwistCommand] reset sensorhead");
    sensorhead_command_twist_.linear.x = -1;
    reset_ = false;
  }else{
    sensorhead_command_twist_.linear.x = 0;
    sensorhead_command_twist_.angular.z = 0.2 * pan_twist_;
    sensorhead_command_twist_.angular.y = 0.2 * tilt_twist_;
    }
    sensorhead_pub_twist_.publish( sensorhead_command_twist_ );
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::SensorheadTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
