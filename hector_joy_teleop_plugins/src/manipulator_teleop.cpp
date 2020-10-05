#include "hector_joy_teleop_plugins/manipulator_teleop.h"

namespace hector_joy_teleop_plugins
{

void ManipulatorTeleop::initialize(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh,
                                   std::shared_ptr<std::map<std::string, double>> property_map)
{
    TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::ManipulatorTeleop");

    // get values from common config file
    max_speed_linear_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "max_speed_linear", 0.05);
    max_speed_angular_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "max_speed_angular", 0.01);

    manipulator_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "manipulator_command_topic",
                                                         "/manipulator_arm_control/twist_cmd");

    // get parameter for switching controllers
    standard_controllers_ =
        pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "standard_controllers",
                                             {"manipulator_arm_traj_controller", "gripper_traj_controller"});

    teleop_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "teleop_controllers",
                                                               {"arm_joystick_control"});

    controller_manager_switch_service_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_switch_service",
                                "/manipulator_arm_control/controller_manager/switch_controller");


    // init topics and service
    switch_controller_client_ =
        pnh_.serviceClient<controller_manager_msgs::SwitchController>(controller_manager_switch_service_);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(manipulator_command_topic_, 10, false);
}

void ManipulatorTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // map trigger axes from [-1,1] to [0,1]
    auto mappedMsg = mapTriggerAxes(msg);

    twist_command_ = joyToTwist(mappedMsg);
}

void ManipulatorTeleop::executePeriodically(const ros::Rate& rate)
{
    twist_pub_.publish(twist_command_);
}

geometry_msgs::Twist ManipulatorTeleop::joyToTwist(const sensor_msgs::JoyConstPtr& msg)
{
    geometry_msgs::Twist twist;


    // linear

    float translate_x_joystick;
    if (getJoyMeasurement("translate_x", msg, translate_x_joystick))
    {
        twist.linear.x = max_speed_linear_ * translate_x_joystick;
    }

    float translate_y_joystick;
    if (getJoyMeasurement("translate_y", msg, translate_y_joystick))
    {
        twist.linear.y = max_speed_linear_ * translate_y_joystick;
    }

    float translate_z_joystick;
    if (getJoyMeasurement("translate_z", msg, translate_z_joystick))
    {
        twist.linear.z = max_speed_linear_ * translate_z_joystick;
    }


    // angular

    float rotate_x_joystick;
    if (getJoyMeasurement("rotate_roll", msg, rotate_x_joystick))
    {
        twist.angular.x = max_speed_angular_ * rotate_x_joystick;
    }

    float rotate_y_joystick;
    if (getJoyMeasurement("rotate_pitch", msg, rotate_y_joystick))
    {
        twist.angular.y = max_speed_angular_ * rotate_y_joystick;
    }

    float rotate_z_joystick;
    if (getJoyMeasurement("rotate_yaw", msg, rotate_z_joystick))
    {
        twist.angular.z = max_speed_angular_ * rotate_z_joystick;
    }

    return twist;
}

std::string ManipulatorTeleop::onLoad()
{
    switch_controller_srv_.request.start_controllers = teleop_controllers_;
    switch_controller_srv_.request.stop_controllers = standard_controllers_;
    switch_controller_srv_.request.strictness = 2;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if (!switch_controller_client_.call(switch_controller_srv_) || !switch_controller_srv_.response.ok)
    {
        std::stringstream ss;
        for (const std::string& s: teleop_controllers_)
        {
            ss << s << " ";
        }
        return "Failed to switch arm controllers: " + ss.str();
    }
    return "";
}

std::string ManipulatorTeleop::onUnload()
{
    switch_controller_srv_.request.start_controllers = standard_controllers_;
    switch_controller_srv_.request.stop_controllers = teleop_controllers_;
    switch_controller_srv_.request.strictness = 2;
    switch_controller_srv_.request.start_asap = false;
    switch_controller_srv_.request.timeout = 0.0;

    if (!switch_controller_client_.call(switch_controller_srv_) || !switch_controller_srv_.response.ok)
    {
        return "Failed to switch arm controllers!";
    }
    return "";
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::ManipulatorTeleop, hector_joy_teleop_plugin_interface::TeleopBase)