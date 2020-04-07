
#include "hector_joy_teleop_plugin/joy_teleop_plugin.h"

JoyTeleopPlugin::JoyTeleopPlugin(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{

    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleopPlugin::JoyCallback, this);

    load_plugin_service_ = pnh_.advertiseService("LoadPlugin", &JoyTeleopPlugin::LoadPluginServiceCB, this);

    // if there are initial plugins given, load them
    std::vector<std::string> init_plugins;

    bool res_init_plugins = pnh_.getParam("init_plugins", init_plugins);

    if (res_init_plugins)
    {
        for (auto& name: init_plugins)
        {
            hector_joy_teleop_plugin::LoadTeleopPlugin::Request req;
            req.load = true;
            req.plugin_name = name;

            hector_joy_teleop_plugin::LoadTeleopPlugin::Response res;

            LoadPluginServiceCB(req, res);

            if (res.result == res.SUCCESS)
            {
                break;
            } else if (res.result == res.UNKNOWN_PLUGINNAME)
            {
                ROS_ERROR_STREAM(
                    "joy_teleop_plugin: plugin name \"" << name
                                                        << "\" unknown which was given in parameter \"init_plugins\".");
            } else if (res.OVERLAPPING_BUTTON_MAPPING)
            {
                ROS_ERROR_STREAM(
                    "joy_teleop_plugin: Plugin \"" << name
                                                   << "\" given in parameter \"init_plugins\" cannot be loaded "
                                                   << "due to overlapping button mappings with already loaded plugin \""
                                                   << res.overlapping_plugin << "\".");
            }
        }
    } else
    {
        ROS_WARN_STREAM("No plugins to be loaded initially given.");
    }

}

bool JoyTeleopPlugin::LoadPluginServiceCB(hector_joy_teleop_plugin::LoadTeleopPlugin::Request& request,
                                          hector_joy_teleop_plugin::LoadTeleopPlugin::Response& response)
{
    int plugin_idx = pluginFactory(request.plugin_name);

    // plugin name not found
    if (plugin_idx == -1)
    {
        response.result = response.UNKNOWN_PLUGINNAME;
        ROS_WARN_STREAM("joy_teleop_plugin: No plugin with name \"" << request.plugin_name << "\" found.");
    } else
    {
        // load plugin
        if (request.load)
        {
            // check if plugin was loaded before
            if (plugins_[plugin_idx]->isActive())
            {
                response.result = response.SUCCESS;
                ROS_INFO_STREAM("joy_teleop_plugin: Plugin \"" << request.plugin_name << "\" was already loaded.");
            } else
            {
                // try to add mapping of given plugin
                std::string res_add_mapping = addMapping(plugins_[plugin_idx]);

                // if was successful, set plugin active
                if (res_add_mapping.empty())
                {
                    plugins_[plugin_idx]->setActive(true);
                    response.result = response.SUCCESS;
                    ROS_INFO_STREAM("joy_teleop_plugin: Plugin \"" << request.plugin_name << "\" loaded successfully.");
                } else
                {
                    response.result = response.OVERLAPPING_BUTTON_MAPPING;
                    response.overlapping_plugin = res_add_mapping;
                    ROS_WARN_STREAM("joy_teleop_plugin: Plugin \"" << request.plugin_name
                                                                   << "\" cannot be loaded due to overlapping button mappings with plugin \""
                                                                   << res_add_mapping << "\".");
                }

            }

        } else // unload plugin
        {
            plugins_[plugin_idx]->setActive(false);
            removeMapping(plugins_[plugin_idx]->getPluginName());
            response.result = response.SUCCESS;
            ROS_INFO_STREAM("joy_teleop_plugin: Plugin \"" << request.plugin_name << "\" unloaded.");
        }
    }

    // return always true, so that caller of request gets error code
    return true;

}

void JoyTeleopPlugin::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    // check that no mapping indices are greater than message array lengths
    if (!axes_.empty() && axes_.rbegin()->first >= msg->axes.size())
    {
        ROS_ERROR_STREAM("Invalid axes mapping of plugin " << axes_.rbegin()->second << " (exceeds message length).");
        return;
    }
    if (!buttons_.empty() && buttons_.rbegin()->first >= msg->buttons.size())
    {
        ROS_ERROR_STREAM(
            "Invalid buttons mapping of plugin " << buttons_.rbegin()->second << " (exceeds message length).");
        return;
    }

    // forward message to all active plugins so that they can convert and forward its parts to the respecitve topic/package
    for (auto& plugin : plugins_)
    {
        if (plugin->isActive())
        {
            plugin->forwardMsg(msg);
        }
    }
}

std::string JoyTeleopPlugin::addMapping(std::unique_ptr<PluginBase>& plugin)
{
    std::pair<std::map<std::string, int>&, std::map<std::string, int>&> axesButtons = plugin->getMapping();

    // backup current button mapping
    std::map<int, std::string> axes_old(axes_);
    std::map<int, std::string> buttons_old(buttons_);

    // try to add all axes mappings of passed map
    for (auto const& x : axesButtons.first)
    {
        std::pair<std::map<int, std::string>::iterator, bool> res = axes_.insert({x.second, plugin->getPluginName()});

        if (!res.second)
        {
            // if insertion failed because element was present before restore old mapping and return false
            axes_ = axes_old;
            buttons_ = buttons_old;

            ROS_ERROR_STREAM("Plugin \"" << plugin->getPluginName()
                                         << "\" cannot be loaded, axes mapping overlaps with already loaded plugin \""
                                         << (*(res.first)).second << "\".");

            return (*(res.first)).second;
        }
    }

    // try to add all button mappings of passed map
    for (auto const& x : axesButtons.second)
    {
        std::pair<std::map<int, std::string>::iterator, bool>
            res = buttons_.insert({x.second, plugin->getPluginName()});

        if (!res.second)
        {
            // if insertion failed because element was present before restore old mapping and return false
            axes_ = axes_old;
            buttons_ = buttons_old;

            ROS_ERROR_STREAM("Plugin \"" << plugin->getPluginName()
                                         << "\" cannot be loaded, buttons mapping overlaps with already loaded plugin \""
                                         << (*(res.first)).second << "\".");

            return (*(res.first)).second;
        }
    }

    return "";
}

void JoyTeleopPlugin::removeMapping(std::string plugin_name)
{
    for (auto const& x : axes_)
    {
        if (x.second == plugin_name)
        {
            axes_.erase(x.first);
        }
    }

    for (auto const& x : buttons_)
    {
        if (x.second == plugin_name)
        {
            buttons_.erase(x.first);
        }
    }
}

int JoyTeleopPlugin::pluginFactory(std::string plugin_name)
{
    // TODO maybe lower case all?
    if (plugin_name == "Drive" || plugin_name == "DrivePlugin")
    {
        // check if plugin was instanciated before, if yes, return its index
        for (size_t i = 0; i < plugins_.size(); i++)
        {
            if (plugins_[i]->getPluginName() == "DrivePlugin")
            {
                return i;
            }
        }
        // otherwise create an instance and return its index
        plugins_.push_back(std::make_unique<DrivePlugin>(nh_, pnh_));
        return (plugins_.size() - 1);
    }

    // NOTE: for new plugins add a new else if(...) case

    // no plugin with given name found
    return -1;
}
