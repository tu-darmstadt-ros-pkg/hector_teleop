#include "../include/hector_joy_teleop_plugin/joy_teleop_plugin.h"

JoyTeleopPlugin::JoyTeleopPlugin(ros::NodeHandle& nh)
{
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleopPlugin::JoyCallback, this);

    //TODO

}

void JoyTeleopPlugin::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    //TODO
}

bool JoyTeleopPlugin::updateMapping(std::unique_ptr<PluginBase>& plugin)
{
    std::pair<std::map<std::string, int>&, std::map<std::string, int>&> axesButtons = plugin->getMapping();


    // backup current button mapping
    std::map<int, std::string> axes_old(axes_);
    std::map<int, std::string> buttons_old(buttons_);

    // try to add all button mappings of passed maps
    for (auto const& x : axesButtons.first)
    {
        std::pair<std::map<int,std::string>::iterator, bool> res = axes_.insert({x.second, plugin->getPluginName()});
        if (!res.second)
        {
            // TODO restore old, return false, error overlapping! (with *(res.first)[]...
        }
    }



    //TODO
    return false;
}