
#include "../include/hector_joy_teleop_plugin/plugin_base.h"

PluginBase::PluginBase()
{
    //TODO get button configuration from yaml file
}



bool PluginBase::isActive()
{
    return active_;
}


bool PluginBase::setActive(bool active)
{
    //TODO
    active_ = active;
    return false;
}


std::pair<std::map<std::string, int>&, std::map<std::string, int>&> PluginBase::getMapping()
{
    return {axes_, buttons_};
}


std::string PluginBase::getPluginName()
{
    return plugin_name_;
}
