

#include "hector_joy_teleop_with_plugins/joy_teleop.h"

namespace hector_joy_teleop_with_plugins
{

JoyTeleop::JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      teleop_plugin_class_loader_("hector_joy_teleop_plugin_interface",
                                  "hector_joy_teleop_plugin_interface::TeleopBase")
{

    // get joy timeout parameter
    joy_timeout_ = pnh.param<double>("joy_timeout", 2.0);


    try
    {
        // get list of available plugins with name and type from parameter server
        XmlRpc::XmlRpcValue plugin_list_ps;

        pnh.getParam("plugins", plugin_list_ps);

        // add all plugins to map
        for (int i = 0; i < plugin_list_ps.size(); i++)
        {
            std::string name = plugin_list_ps[i]["name"];
            std::string type = plugin_list_ps[i]["type"];
            plugin_names_types_.emplace(name, type);
        }
    }
    catch (const XmlRpc::XmlRpcException& e)
    {
        ROS_ERROR_STREAM(
            "hector_joy_teleop_with_plugins: Error while getting parameter \"plugins\" from parameterserver or parsing it: "
                << e.getCode() << ", " << e.getMessage());
        throw;
    }

    // create an empty property map
    property_map_ = std::make_shared<std::map<std::string, double>>();

    // get the name of the top plugin and init its vector index with -1
    top_plugin_ = std::make_pair(pnh_.param<std::string>("top_plugin", "Profile"), -1);

    // setup topics and services
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleop::JoyCallback, this);

    load_plugin_service_ = pnh_.advertiseService("load_plugin", &JoyTeleop::LoadPluginServiceCB, this);

}

void JoyTeleop::executePeriodically(const ros::Rate& rate)
{
    // if last joy message was received more than joy_timeout_ seconds ago, send a zero message
    if(!last_joy_msg_received_.isZero() && ros::Time::now().toSec() - last_joy_msg_received_.toSec() > joy_timeout_)
    {
        sensor_msgs::JoyPtr zeroMsg = boost::make_shared<sensor_msgs::Joy>();
        zeroMsg->header.stamp = ros::Time::now();

        // resize the arrays, so that at each required position there is a zero
        if(!axes_.empty())
        {
            zeroMsg->axes.resize((axes_.rbegin()->first) + 1);
            // set trigger values to 1 as expected as default value
            zeroMsg->axes[pnh_.param<int>("left_trigger", 2)] = 1.0;
            zeroMsg->axes[pnh_.param<int>("right_trigger", 5)] = 1.0;
        }
        if(!buttons_.empty())
        {
            zeroMsg->buttons.resize((buttons_.rbegin()->first) + 1);
        }

        JoyCallback(zeroMsg);

        ROS_WARN_STREAM_NAMED("hector_joy_teleop_with_plugins", "Timeout! More than " << joy_timeout_ << " seconds since last joy message.");
    }

    for (auto& plugin : plugins_)
    {
        if (plugin->isActive())
        {
            plugin->executePeriodically(rate);
        }
    }
}

bool JoyTeleop::LoadPluginServiceCB(hector_joy_teleop_plugin_msgs::LoadTeleopPlugin::Request& request,
                                    hector_joy_teleop_plugin_msgs::LoadTeleopPlugin::Response& response)
{

    // special request: unload all loaded plugins
    std::string pluginname_lower = request.plugin_name;
    std::transform(pluginname_lower.begin(), pluginname_lower.end(), pluginname_lower.begin(), ::tolower);

    if (pluginname_lower == "all")
    {
        if (!request.load)
        {

            for (TeleopBasePtr& plugin: plugins_)
            {
                // unload all active plugins (except of top_plugin)
                if (plugin->isActive() && (plugin->getPluginName() != top_plugin_.first))
                {
                    plugin->setActive(false);
                    std::string res_unload = plugin->onUnload();

                    // if there was no error while unloading, continue to unload plugin
                    if (res_unload.empty())
                    {
                        removeMapping(plugin->getPluginName());

                        // set result only to true if there was no plugin before which returned errors
                        if (response.result != response.PLUGIN_LOAD_ERROR)
                        {
                            response.result = response.SUCCESS;
                        }

                        ROS_INFO_STREAM(
                            "joy_teleop_with_plugins: Plugin \"" << plugin->getPluginName() << "\" unloaded.");
                    } else
                    {
                        // if plugin could not be unloaded send error
                        plugin->setActive(true);
                        response.result = response.PLUGIN_LOAD_ERROR;

                        // add new error message to existing one
                        if (response.error_msg.empty())
                        {
                            response.error_msg = res_unload;
                        } else
                        {
                            response.error_msg = response.error_msg + "; " + res_unload;
                        }

                        ROS_ERROR_STREAM("joy_teleop_with_plugins: An error occured while unloading plugin "
                                             << request.plugin_name << " in onUnload(): " << res_unload << ".");
                    }
                }
            }
        } else
        {
            response.result = response.SUCCESS;
            ROS_WARN_STREAM(
                "joy_teleop_with_plugins: All plugins cannot be loaded simultaneously, please load them one by one.");
        }

        return true;
    }

    // load/unload requested plugin: get index of earlier instantiated plugin or create a new instance otherwise
    int plugin_idx = pluginFactory(request.plugin_name);

    // plugin name not found
    if (plugin_idx == -1)
    {
        response.result = response.UNKNOWN_PLUGINNAME;
        response.error_msg = request.plugin_name;
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
                ROS_INFO_STREAM(
                    "joy_teleop_with_plugins: Plugin \"" << request.plugin_name << "\" was already loaded.");
            } else
            {
                // try to add mapping of given plugin
                std::string res_add_mapping = addMapping(plugins_[plugin_idx]);

                // if was successful, set plugin active
                if (res_add_mapping.empty())
                {
                    std::string res_load = plugins_[plugin_idx]->onLoad();

                    // check if there was an error while loading the plugin
                    if (res_load.empty())
                    {
                        // if it is the top_plugin, store its index
                        if (plugins_[plugin_idx]->getPluginName() == top_plugin_.first)
                        {
                            top_plugin_.second = plugin_idx;
                        }

                        plugins_[plugin_idx]->setActive(true);
                        response.result = response.SUCCESS;
                        ROS_INFO_STREAM(
                            "joy_teleop_with_plugins: Plugin \"" << request.plugin_name
                                                                 << "\" loaded successfully.");
                    } else
                    {

                        // remove already loaded mapping
                        removeMapping(plugins_[plugin_idx]->getPluginName());

                        response.result = response.PLUGIN_LOAD_ERROR;
                        response.error_msg = res_load;
                        ROS_ERROR_STREAM("joy_teleop_with_plugins: An error occured while loading plugin "
                                             << request.plugin_name << " in onLoad():  " << res_load << ".");
                    }

                } else
                {
                    response.result = response.OVERLAPPING_BUTTON_MAPPING;
                    response.error_msg = res_add_mapping;
                    ROS_WARN_STREAM("joy_teleop_with_plugins: Plugin \"" << request.plugin_name
                                                                         << "\" cannot be loaded due to overlapping button mappings with plugin \""
                                                                         << res_add_mapping << "\".");
                }

            }

        } else // unload plugin
        {
            if (plugins_[plugin_idx]->isActive())
            {
                plugins_[plugin_idx]->setActive(false);
                std::string res_unload = plugins_[plugin_idx]->onUnload();

                // check if there was a problem while unloading
                if (res_unload.empty())
                {
                    // if it is the top_plugin, reset its index
                    if (plugins_[plugin_idx]->getPluginName() == top_plugin_.first)
                    {
                        top_plugin_.second = -1;
                    }

                    removeMapping(plugins_[plugin_idx]->getPluginName());
                    response.result = response.SUCCESS;
                    ROS_INFO_STREAM("joy_teleop_with_plugins: Plugin \"" << request.plugin_name << "\" unloaded.");
                } else
                {
                    plugins_[plugin_idx]->setActive(true);
                    response.result = response.PLUGIN_LOAD_ERROR;
                    response.error_msg = res_unload;
                    ROS_ERROR_STREAM("joy_teleop_with_plugins: An error occured while unloading plugin "
                                         << request.plugin_name << " in onUnload(): " << res_unload << ".");
                }
            } else
            {
                response.result = response.SUCCESS;
                ROS_INFO_STREAM(
                    "joy_teleop_with_plugins: Plugin \"" << request.plugin_name << "\" was already unloaded.");
            }
        }
    }

    // return always true, so that caller of request gets error code
    return true;

}

void JoyTeleop::JoyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    last_joy_msg_received_ = ros::Time::now();

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


    // execute forwardMsg for top_plugin first (if it is loaded) and check if it used the message (= a button/axes was pressed which is used by top plugin
    bool top_plugin_executed = false;

    if (top_plugin_.second >= 0 && top_plugin_.second < plugins_.size())
    {
        plugins_[top_plugin_.second]->forwardMsg(msg);
        top_plugin_executed = plugins_[top_plugin_.second]->hasUsedMsg();
    }


    // only forward the message to the other plugins if the top-plugin was not triggered
    if (!top_plugin_executed)
    {
        // forward message to all active plugins so that they can convert and forward its parts to the respecitve topic/package
        for (auto& plugin : plugins_)
        {
            if (plugin->isActive() && plugin->getPluginName() != top_plugin_.first)
            {
                plugin->forwardMsg(msg);
            }
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
            // get name from iterator
            std::string overlapping_plugin_name = (res.first)->second;

            // if insertion failed because element was present before restore old mapping and return false
            axes_.clear();
            buttons_.clear();

            axes_ = axes_old;
            buttons_ = buttons_old;

            ROS_ERROR_STREAM("Plugin \"" << plugin->getPluginName()
                                         << "\" cannot be loaded, axes mapping overlaps at index " << x.second
                                         << " with already loaded plugin \""
                                         << overlapping_plugin_name << "\".");

            return overlapping_plugin_name;
        }
    }

    // try to add all button mappings of passed map
    for (auto const& x : axesButtons.second)
    {
        std::pair<std::map<int, std::string>::iterator, bool>
            res = buttons_.insert({x.second, plugin->getPluginName()});

        if (!res.second)
        {
            // get name from iterator
            std::string overlapping_plugin_name = (res.first)->second;

            // if insertion failed because element was present before restore old mapping and return name of overlapping plugin
            axes_.clear();
            buttons_.clear();

            axes_ = axes_old;
            buttons_ = buttons_old;

            ROS_ERROR_STREAM("Plugin \"" << plugin->getPluginName()
                                         << "\" cannot be loaded, buttons mapping overlaps at index " << x.second
                                         << " with already loaded plugin \""
                                         << overlapping_plugin_name << "\".");

            return overlapping_plugin_name;
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
    try
    {
        // get plugin type from map
        std::string plugin_type = plugin_names_types_[teleop_plugin_name];

        TeleopBasePtr ptr;

        // create instance
        hector_joy_teleop_plugin_interface::TeleopBase* projection_ptr =
            teleop_plugin_class_loader_.createUnmanagedInstance(plugin_type);

        // reset pointer including unloader
        ptr.reset(projection_ptr,
                  std::bind(&TeleopPluginClassLoader::unloadLibraryForClass,
                            &teleop_plugin_class_loader_,
                            plugin_type));

        // initialize plugin
        ptr->initialize(nh_, pnh_, property_map_, teleop_plugin_name);

        // add plugin to plugins_list
        plugins_.push_back(ptr);

        // return the plugin's id
        return ((int) plugins_.size() - 1);

    } catch (pluginlib::PluginlibException& ex)
    {
        ROS_ERROR_STREAM("The plugin \"" << teleop_plugin_name << "\" failed to load: " << ex.what());
    }

    return -1;

}

}
