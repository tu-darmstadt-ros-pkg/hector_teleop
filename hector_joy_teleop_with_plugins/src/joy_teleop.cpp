
#include "hector_joy_teleop_with_plugins/joy_teleop.h"

namespace hector_joy_teleop_with_plugins
{

JoyTeleop::JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      teleop_plugin_class_loader_("hector_joy_teleop_plugin_interface",
                                  "hector_joy_teleop_plugin_interface::TeleopBase")
{

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleop::JoyCallback, this);

    load_plugin_service_ = pnh_.advertiseService("LoadPlugin", &JoyTeleop::LoadPluginServiceCB, this);

    // if there are initial plugins given, load them
    std::vector<std::string> init_plugins;

    bool res_init_plugins = pnh_.getParam("init_plugins", init_plugins);

    if (res_init_plugins)
    {
        for (auto& name: init_plugins)
        {
            LoadTeleopPlugin::Request req;
            req.load = true;
            req.plugin_name = name;

            LoadTeleopPlugin::Response res;

            LoadPluginServiceCB(req, res);

            if (res.result == res.SUCCESS)
            {
                break;
            } else if (res.result == res.UNKNOWN_PLUGINNAME)
            {
                ROS_ERROR_STREAM(
                    "joy_teleop_with_plugins: plugin name \"" << name
                                                        << "\" unknown which was given in parameter \"init_plugins\".");
            } else if (res.OVERLAPPING_BUTTON_MAPPING)
            {
                ROS_ERROR_STREAM(
                    "joy_teleop_with_plugins: Plugin \"" << name
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

void JoyTeleop::executePeriodically(const ros::Rate& rate)
{
    for (auto& plugin : plugins_)
    {
        if (plugin->isActive())
        {
            plugin->executePeriodically(rate);
        }
    }
}

bool JoyTeleop::LoadPluginServiceCB(LoadTeleopPlugin::Request& request, LoadTeleopPlugin::Response& response)
{
    int plugin_idx = pluginFactory(request.plugin_name);

    // plugin name not found
    if (plugin_idx == -1)
    {
        response.result = response.UNKNOWN_PLUGINNAME;
        ROS_WARN_STREAM("joy_teleop_with_plugins: No plugin with name \"" << request.plugin_name << "\" found.");
    } else
    {
        // load plugin
        if (request.load)
        {
            // check if plugin was loaded before
            if (plugins_[plugin_idx]->isActive())
            {
                response.result = response.SUCCESS;
                ROS_INFO_STREAM("joy_teleop_with_plugins: Plugin \"" << request.plugin_name << "\" was already loaded.");
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

void JoyTeleop::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
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

std::string JoyTeleop::addMapping(TeleopBasePtr& plugin)
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
            axes_.clear();
            buttons_.clear();

            axes_ = axes_old;
            buttons_ = buttons_old;

            ROS_ERROR_STREAM("Plugin \"" << plugin->getPluginName()
                                         << "\" cannot be loaded, axes mapping overlaps at index " << x.second
                                         << " with already loaded plugin \""
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
            // if insertion failed because element was present before restore old mapping and return name of overlapping plugin
            axes_.clear();
            buttons_.clear();

            axes_ = axes_old;
            buttons_ = buttons_old;

            ROS_ERROR_STREAM("Plugin \"" << plugin->getPluginName()
                                         << "\" cannot be loaded, buttons mapping overlaps at index " << x.second
                                         << " with already loaded plugin \""
                                         << (*(res.first)).second << "\".");

            return (*(res.first)).second;
        }
    }

    return "";
}

void JoyTeleop::removeMapping(std::string plugin_name)
{
    std::vector<int> axes_to_delete;
    for (auto const& x : axes_)
    {
        if (x.second == plugin_name)
        {
            axes_to_delete.push_back(x.first);
        }
    }
    for (int i : axes_to_delete)
    {
        axes_.erase(i);
    }

    std::vector<int> buttons_to_delete;
    for (auto const& x : buttons_)
    {
        if (x.second == plugin_name)
        {
            buttons_to_delete.push_back(x.first);
        }
    }
    for (int i: buttons_to_delete)
    {
        buttons_.erase(i);
    }
}

int JoyTeleop::pluginFactory(std::string teleop_plugin_name)
{
    // check if plugin was instantiated before, if yes, return its index
    for (size_t i = 0; i < plugins_.size(); i++)
    {
        if (plugins_[i]->getPluginName() == teleop_plugin_name)
        {
            return i;
        }
    }

    // otherwise create an instance and return its index
    try {

        TeleopBasePtr ptr;

        hector_joy_teleop_plugin_interface::TeleopBase* projection_ptr = teleop_plugin_class_loader_.createUnmanagedInstance(teleop_plugin_name);
        ptr.reset(projection_ptr, std::bind(&TeleopPluginClassLoader ::unloadLibraryForClass, &teleop_plugin_class_loader_, teleop_plugin_name));

        ptr->initialize(nh_, pnh_);

        plugins_.push_back(ptr);

        return ((int) plugins_.size() - 1);

    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR_STREAM("The plugin failed to load: " << ex.what());
    }

    return -1;

}

}
