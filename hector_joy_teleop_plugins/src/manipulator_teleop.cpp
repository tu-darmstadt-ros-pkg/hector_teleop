#include "hector_joy_teleop_plugins/manipulator_teleop.h"

namespace hector_joy_teleop_plugins
{

void ManipulatorTeleop::initialize(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh,
                                   std::shared_ptr<std::map<std::string, double>> property_map,
                                   std::string plugin_name)
{
    TeleopBase::initializeBase(nh, pnh, property_map, plugin_name, "hector_joy_teleop_plugins::ManipulatorTeleop");

    // get values from common config file
    max_speed_linear_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "max_speed_linear", 0.1);
    max_speed_angular_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "max_speed_angular", 0.4);
    max_gripper_speed_ = pnh_.param<double>(getParameterServerPrefix() + "/" + "max_gripper_speed", 1);
    std::string response_curve = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "response_curve", "linear");
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


    // get topic names
    manipulator_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "manipulator_command_topic",
                                                         "/manipulator_arm_control/twist_cmd");
    gripper_command_topic_ = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "gripper_command_topic",
                                                         "/manipulator_arm_control/gripper_cmd");


    // init topics
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(manipulator_command_topic_, 10, false);

    gripper_pub_ = nh_.advertise<std_msgs::Float64>(gripper_command_topic_, 10, false);


    // get service names
    std::string hold_pose_srv = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "hold_pose_service",
                                                         "/manipulator_arm_control/arm_tcp_controller/hold_pose");

    std::string reset_pose_srv = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "reset_pose_service",
                                                        "/manipulator_arm_control/arm_tcp_controller/reset_pose");

    std::string move_tc_srv = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "move_tool_center_service",
                                                        "/manipulator_arm_control/arm_tcp_controller/move_tool_center");

    std::string reset_tc_srv = pnh_.param<std::string>(getParameterServerPrefix() + "/" + "reset_tool_center_service",
                                                        "/manipulator_arm_control/arm_tcp_controller/reset_tool_center");

    // init service clients
    hold_pose_srv_client_ = pnh.serviceClient<std_srvs::SetBool>(hold_pose_srv);
    reset_pose_srv_client_ = pnh.serviceClient<std_srvs::Empty>(reset_pose_srv);
    move_tc_srv_client_ = pnh.serviceClient<std_srvs::SetBool>(move_tc_srv);
    reset_tc_srv_client_ = pnh.serviceClient<std_srvs::Empty>(reset_tc_srv);

    hold_finished_ = true;
    move_tc_finished_ = true;

    // get parameter for switching controllers
    standard_controllers_ =
        pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "standard_controllers",
                                             {});

    teleop_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "teleop_controllers",
                                                               {});

    // get values for switching controllers
    std::string controller_manager_switch_service =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_switch_service",
                                "/manipulator_arm_control/controller_manager/switch_controller");
    std::string controller_manager_list_service =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_list_service",
                                "/manipulator_arm_control/controller_manager/list_controllers");

    int num_tries_switch_controller =
        pnh_.param<int>(getParameterServerPrefix() + "/" + "num_tries_switch_controller", 5);
    int sleep_time = pnh_.param<int>(getParameterServerPrefix() + "/" + "sleep_between_tries_sec", 1);


    // init ControllerHelper for switching services later
    controller_helper_ = ControllerHelper(pnh,
                                          controller_manager_switch_service,
                                          controller_manager_list_service,
                                          num_tries_switch_controller,
                                          sleep_time,
                                          plugin_name_);
}

void ManipulatorTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // map trigger axes from [-1,1] to [0,1]
    auto mappedMsg = mapTriggerAxes(msg);

    // if one of the special buttons was pressed, do not continue to twist and gripper command
    if(joyToSpecial(mappedMsg))
    {
        return;
    }

    twist_command_ = joyToTwist(mappedMsg);

    gripper_command_ = joyToGripper(mappedMsg);
    gripper_pub_.publish(gripper_command_);

}

void ManipulatorTeleop::executePeriodically(const ros::Rate& rate)
{
    twist_pub_.publish(twist_command_);
}


bool ManipulatorTeleop::joyToSpecial(const sensor_msgs::JoyConstPtr& msg)
{
    float hold_pose_joy;
    if (getJoyMeasurement("hold_pose", msg, hold_pose_joy))
    {
        if(hold_pose_joy != 0 && hold_finished_)
        {
            std_srvs::SetBool srv;
            srv.request.data = !hold_pose_;

            // send value
            if (hold_pose_srv_client_.call(srv) && srv.response.success)
            {
              hold_pose_ = !hold_pose_;
              hold_finished_ = false;
              return true;
            } else
            {
              ROS_ERROR_STREAM("Manipulator_teleop: Unable to send value " << !hold_pose_ << " to service " << hold_pose_srv_client_.getService() << ". Please try again.");
            }
        }
        else if(hold_pose_joy == 0 && !hold_finished_)
        {
            hold_finished_ = true;
        }

    }


    float reset_pose_joy;
    if (getJoyMeasurement("reset_pose", msg, reset_pose_joy) && reset_pose_joy != 0)
    {
        std_srvs::Empty srv;

        // send value
        if (reset_pose_srv_client_.call(srv))
        {
            return true;
        } else
        {
            ROS_ERROR_STREAM("Manipulator_teleop: Unable to call service " << reset_pose_srv_client_.getService() << ". Please try again.");
        }
    }


    float move_tool_center_joy;
    if (getJoyMeasurement("move_tool_center", msg, move_tool_center_joy, false))
    {
        if(move_tool_center_joy != 0 && move_tc_finished_)
        {
            std_srvs::SetBool srv;
            srv.request.data = !move_tool_center_;

            // send value
            if (move_tc_srv_client_.call(srv) && srv.response.success)
            {
              move_tool_center_ = !move_tool_center_;
              move_tc_finished_ = false;
              return true;
            } else
            {
              ROS_ERROR_STREAM("Manipulator_teleop: Unable to send value " << !move_tool_center_ << " to service " << move_tc_srv_client_.getService() << ". Please try again.");
            }
        }
        else if(move_tool_center_joy == 0 && !move_tc_finished_)
        {
            move_tc_finished_ = true;
        }

    }


    float reset_tool_center_joy;
    if (getJoyMeasurement("reset_tool_center", msg, reset_tool_center_joy, false) && reset_tool_center_joy != 0)
    {
        std_srvs::Empty srv;

        // send value
        if (reset_tc_srv_client_.call(srv))
        {
            return true;
        } else
        {
            ROS_ERROR_STREAM("Manipulator_teleop: Unable to call service " << reset_tc_srv_client_.getService() << ". Please try again.");
        }
    }


    return false;
}


geometry_msgs::Twist ManipulatorTeleop::joyToTwist(const sensor_msgs::JoyConstPtr& msg)
{
    geometry_msgs::Twist twist;


    // linear
    float translate_x_joystick;
    if (getJoyMeasurement("translate_x", msg, translate_x_joystick))
    {
        twist.linear.x = max_speed_linear_ * applyResponseCurve(translate_x_joystick, response_curve_);
    }

    float translate_y_joystick;
    if (getJoyMeasurement("translate_y", msg, translate_y_joystick))
    {
        twist.linear.y = max_speed_linear_ * applyResponseCurve(translate_y_joystick, response_curve_);
    }

    float translate_z_joystick;
    if (getJoyMeasurement("translate_z", msg, translate_z_joystick))
    {
        twist.linear.z = max_speed_linear_ * applyResponseCurve(translate_z_joystick, response_curve_);
    }


    // angular
    float rotate_x_joystick;
    if (getJoyMeasurement("rotate_roll", msg, rotate_x_joystick))
    {
        twist.angular.x = max_speed_angular_ * applyResponseCurve(rotate_x_joystick, response_curve_);
    }

    float rotate_y_joystick;
    if (getJoyMeasurement("rotate_pitch", msg, rotate_y_joystick))
    {
        twist.angular.y = max_speed_angular_ * applyResponseCurve(rotate_y_joystick, response_curve_);
    }

    float rotate_z_joystick;
    if (getJoyMeasurement("rotate_yaw", msg, rotate_z_joystick))
    {
        twist.angular.z = max_speed_angular_ * applyResponseCurve(rotate_z_joystick, response_curve_);
    }

    return twist;
}

std_msgs::Float64 ManipulatorTeleop::joyToGripper(const sensor_msgs::JoyConstPtr& msg)
{
    std_msgs::Float64 command;

    float joystick;
    if (getJoyMeasurement("gripper", msg, joystick))
    {
        command.data = max_gripper_speed_ * joystick;
    }

    return command;
}



std::string ManipulatorTeleop::onLoad()
{
    // As the controllers should only be loaded in the unfold arm behavior, comment it here out.
    return controller_helper_.switchControllers(teleop_controllers_, standard_controllers_);
}

std::string ManipulatorTeleop::onUnload()
{
    // do not switch controllers if hold pose is activated
    if(hold_pose_)
    {
        return "";
    }

    return controller_helper_.switchControllers(standard_controllers_, teleop_controllers_);
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::ManipulatorTeleop, hector_joy_teleop_plugin_interface::TeleopBase)