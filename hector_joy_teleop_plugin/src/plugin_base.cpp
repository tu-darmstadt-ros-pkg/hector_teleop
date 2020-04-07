
#include "../include/hector_joy_teleop_plugin/plugin_base.h"

PluginBase::PluginBase(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::string plugin_name): nh_(nh), pnh_(pnh), plugin_name_(plugin_name)
{
    // get axes mapping from parameter server
    std::vector<std::string> axes;
    if( !pnh_.getParam(plugin_name_ + "/" + "axes", axes) )
    {
        ROS_ERROR_STREAM("Failed to get parameter \"axes\" for teleop_plugin " << plugin_name_ << " from server.");
    }

    // store mapping
    for(size_t i = 0; i < axes.size(); i++)
    {
        if(!axes[i].empty())
        {
            axes_[axes[i]] = i;
        }
    }

    // get buttons mapping from parameter server
    std::vector<std::string> buttons;
    if( !pnh_.getParam(plugin_name_ + "/" + "buttons", buttons) )
    {
        ROS_ERROR_STREAM("Failed to get parameter \"buttons\" for joy_teleop_plugin " << plugin_name_ << " from server.");
    }

    // store mapping
    for(size_t i = 0; i < buttons.size(); i++)
    {
        if(!buttons[i].empty())
        {
            buttons_[buttons[i]] = i;
        }
    }


}

bool PluginBase::isActive()
{
    return active_;
}

void PluginBase::setActive(bool active)
{
    active_ = active;
}

std::pair<std::map<std::string, int>&, std::map<std::string, int>&> PluginBase::getMapping()
{
    return {axes_, buttons_};
}

std::string PluginBase::getPluginName()
{
    return plugin_name_;
}
