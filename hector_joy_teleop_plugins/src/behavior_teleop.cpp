#include "hector_joy_teleop_plugins/behavior_teleop.h"

namespace hector_joy_teleop_plugins
{

BehaviorTeleop::BehaviorTeleop() : action_client_("/flexbe/execute_behavior")
{
}

void BehaviorTeleop::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<std::map<std::string, double>> property_map)
{
    TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::BehaviorTeleop");
}

void BehaviorTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{

    if (!action_client_.isServerConnected())
    {
        ROS_ERROR_STREAM(
            "Actionserver not connected! Please execute '$ rosnode kill /behavior_execution_server', it will respawn.");
        return;
    } else
    {
        ROS_ERROR_STREAM("Server is connected");
    }

    flexbe_msgs::BehaviorExecutionGoal goal;

    std::string name;

    for (const auto& behavior : axes_)
    {
        if (msg->axes[behavior.second] != 0)
        {
            name = behavior.first;
            auto pos_delimiter = name.find(';');

            // if there is a semicolon in string check if first or second behavior is requested
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

    if (!name.empty())
    {
        goal.behavior_name = name;
        action_client_.sendGoal(goal, boost::bind(&BehaviorTeleop::doneCB, this, _1, _2));
        behavior_started = true;
    } else
    {
        if (behavior_started)
        {
            action_client_.cancelGoal();
        }
    }

}

void BehaviorTeleop::doneCB(const actionlib::SimpleClientGoalState& state,
                            const flexbe_msgs::BehaviorExecutionResultConstPtr& result)
{
    if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {

    }
    behavior_started = false;
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::BehaviorTeleop, hector_joy_teleop_plugin_interface::TeleopBase)