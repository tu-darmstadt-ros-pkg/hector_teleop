#include "hector_joy_teleop_plugins/behavior_teleop.h"

namespace hector_joy_teleop_plugins
{

BehaviorTeleop::BehaviorTeleop() : action_client_("/flexbe/execute_behavior")
{
}

void BehaviorTeleop::initialize(ros::NodeHandle& nh,
                                ros::NodeHandle& pnh,
                                std::shared_ptr<std::map<std::string, double>> property_map,
                                std::string plugin_name)
{
    TeleopBase::initializeBase(nh, pnh, property_map, plugin_name, "hector_joy_teleop_plugins::BehaviorTeleop");
}

void BehaviorTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    if (!action_client_.isServerConnected())
    {
        ROS_ERROR_STREAM(
            "hector_joy_teleop_with_plugins: Behavior Plugin: Actionserver not connected! Please execute '$ rosnode kill /behavior_execution_server', it will respawn.");
        return;
    }

    std::string name;

    for (const auto& behavior : axes_)
    {
        if (msg->axes[behavior.second] != 0)
        {
            name = behavior.first;

            // if there is a semicolon in string check if first or second behavior is requested
            auto pos_delimiter = name.find(';');
            if (pos_delimiter != std::string::npos)
            {
                if (msg->axes[behavior.second] < 0)
                {
                    name = name.substr(0, pos_delimiter);
                } else
                {
                    name = name.substr(pos_delimiter + 1, std::string::npos);
                }
            }
        }
    }

    for (const auto& behavior : buttons_)
    {
        if (msg->buttons[behavior.second] != 0)
        {
            name = behavior.first;
        }
    }


    flexbe_msgs::BehaviorExecutionGoal goal;

    // if a name was given and no behavior is currently running and no behavior button is pressed, start behavior
    if (!name.empty() && !behavior_running && !behavior_button_pressed)
    {
        ROS_WARN_STREAM("hector_joy_teleop_with_plugins: Behavior Plugin: Try to start behavior : " + name);
        goal.behavior_name = name;


        action_client_.sendGoal(goal, boost::bind(&BehaviorTeleop::doneCB, this, _1, _2));
        behavior_running = true;
        behavior_button_pressed = true;

        return;
    }

    // if no behavior was given, cancel goal (if behavior is running)
    if (name.empty())
    {
        //ROS_WARN_STREAM("Behavior name was empty");
        if (behavior_running)
        {
            ROS_WARN_STREAM("hector_joy_teleop_with_plugins: Behavior Plugin: Cancel goal");
            action_client_.cancelGoal();
        }

        behavior_button_pressed = false;

        return;
    }

}

void BehaviorTeleop::doneCB(const actionlib::SimpleClientGoalState& state,
                            const flexbe_msgs::BehaviorExecutionResultConstPtr& result)
{
    if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("hector_joy_teleop_with_plugins: Behavior Plugin: Behavior succeeded.");
    }

    behavior_running = false;
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::BehaviorTeleop, hector_joy_teleop_plugin_interface::TeleopBase)