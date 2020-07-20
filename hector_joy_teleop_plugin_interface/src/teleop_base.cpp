
#include "hector_joy_teleop_plugin_interface/teleop_base.h"

namespace hector_joy_teleop_plugin_interface
{

void TeleopBase::initializeBase(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string plugin_name)
{
    nh_ = nh;
    pnh_ = pnh;

    std::string delimiter = ":";
    plugin_namespace_ = plugin_name.substr(0, plugin_name.find(delimiter));
    plugin_name_ = plugin_name.substr(plugin_name.find_last_of(delimiter)+1);


    // get axes mapping from parameter server
    std::vector<std::string> axes;
    if (!pnh_.getParam(getParameterServerPrefix() + "/" + "axes", axes))
    {
        ROS_ERROR_STREAM("Failed to get parameter \"axes\" for " << plugin_name_ << " from server.");
    }

    // store mapping
    for (size_t i = 0; i < axes.size(); i++)
    {
        if (!axes[i].empty())
        {
            axes_[axes[i]] = i;
        }
    }

    // get buttons mapping from parameter server
    std::vector<std::string> buttons;
    if (!pnh_.getParam(getParameterServerPrefix() + "/" + "buttons", buttons))
    {
        ROS_ERROR_STREAM("Failed to get parameter \"buttons\" for " << plugin_name_ << " from server.");
    }

    // store mapping
    for (size_t i = 0; i < buttons.size(); i++)
    {
        if (!buttons[i].empty())
        {
            buttons_[buttons[i]] = i;
        }
    }

}

bool TeleopBase::isActive()
{
    return active_;
}

void TeleopBase::setActive(bool active)
{
    active_ = active;
}

std::pair<std::map<std::string, int>&, std::map<std::string, int>&> TeleopBase::getMapping()
{
    return {axes_, buttons_};
}

std::string TeleopBase::getPluginName()
{
    return plugin_namespace_ + "::" + plugin_name_;
}

void TeleopBase::printMissingParameter(std::string param_name)
{
    ROS_ERROR_STREAM(plugin_name_ << ": The required axis/button mapping for value \"" << param_name
                                  << "\" is missing (maybe misspelled?).");
}

bool TeleopBase::getJoyMeasurement(std::string name, const sensor_msgs::JoyConstPtr& msg, float& result, bool print_missing_parameter)
{
    auto it_axes = axes_.find(name);
    if(it_axes != axes_.end())
    {
        result = msg->axes[it_axes->second];
        return true;
    }

    auto it_buttons = buttons_.find(name);
    if(it_buttons != buttons_.end())
    {
        result = msg->buttons[it_buttons->second];
        return true;
    }

    if(print_missing_parameter)
    {
        printMissingParameter(name);
    }

    return false;
}


void TeleopBase::executePeriodically(const ros::Rate& rate)
{
    return;
}

std::string TeleopBase::onLoad()
{
    return "";
}

std::string TeleopBase::onUnload()
{
    return "";
}


TeleopBase::~TeleopBase()
{

}
std::string TeleopBase::getParameterServerPrefix()
{
    return plugin_namespace_ + "/" + plugin_name_;
}



}