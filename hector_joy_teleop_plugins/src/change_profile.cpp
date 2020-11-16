
#include "hector_joy_teleop_plugins/change_profile.h"

namespace hector_joy_teleop_plugins
{

void ChangeProfile::initialize(ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               std::shared_ptr<std::map<std::string, double>> property_map)
{
    TeleopBase::initializeBase(nh, pnh, property_map, "hector_joy_teleop_plugins::ChangeProfile");

    // get values from common config file
    change_forward_ = pnh_.param<int>(getParameterServerPrefix() + "/" + "change_backward", 5);
    change_backward_ = pnh_.param<int>(getParameterServerPrefix() + "/" + "change_forward", 4);
    use_buttons_to_iter_ = pnh_.param<bool>(getParameterServerPrefix() + "/" + "use_buttons_to_iter", true);

    reload_profile_ = pnh_.param<int>(getParameterServerPrefix() + "/" + "reload_profile", 6);

    // get profile list
    std::vector<std::string> profile_list;
    pnh_.getParam(getParameterServerPrefix() + "/" + "profiles", profile_list);

    // get plugin list for each profile and add to vector
    for (auto& profile_name : profile_list)
    {
        std::vector<std::string> plugin_list;
        bool res = pnh_.getParam(getParameterServerPrefix() + "/" + profile_name, plugin_list);

        if (!res)
        {
            ROS_WARN_STREAM("No plugins found for profile " << profile_name
                                                            << " in config file. Profile will not be loaded in list.");
            continue;
        }

        Profile tmp_profile{profile_name, plugin_list};

        profiles_.push_back(tmp_profile);
    }

    load_teleop_plugins_srv_client_ = pnh_.serviceClient<hector_joy_teleop_plugin_msgs::LoadTeleopPlugin>(
        "/hector_joy_teleop_with_plugins/load_plugin");

    // TODO Service for changing profile
}

std::string ChangeProfile::onLoad()
{
    if (profiles_.empty())
    {
        return "No profiles found. Stop loading plugin ChangeProfile.";
    }

    if(!load_teleop_plugins_srv_client_.waitForExistence(ros::Duration(5)))
    {
        return "hector_joy_teleop_with_plugins: Service " + load_teleop_plugins_srv_client_.getService() + " does not exist (timed out after 5 sec).";
    }

    // set first profile as current profile
    setCurrentProfile(profiles_.begin());

    return "";
}

void ChangeProfile::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    sensor_msgs::JoyPtr mappedMsg = mapTriggerAxes(msg);

    iterProfile(mappedMsg);
}

void ChangeProfile::iterProfile(sensor_msgs::JoyPtr& msg)
{
    float enabled = false;
    if (getJoyMeasurement("enable_change", msg, enabled))
    {
        // if enabled is pressed
        if (enabled != 0.0)
        {
            used_msg_ = true;

            // reload profile
            if (reload_profile_ >= 0 && reload_profile_ < msg->buttons.size() && msg->buttons[reload_profile_])
            {
                ROS_INFO_STREAM("Reload profile " + current_profile_->name);
                setCurrentProfile(current_profile_);
            } else
            {
                // search in buttons or axes vector for change direction in profile list
                if (use_buttons_to_iter_)
                {
                    if (change_forward_ >= 0 && change_forward_ < msg->buttons.size() && msg->buttons[change_forward_])
                    {
                        nextProfile();

                    } else if (change_backward_ >= 0 && change_backward_ < msg->buttons.size()
                        && msg->buttons[change_backward_])
                    {
                        previousProfile();
                    }
                } else
                {
                    if (abs(change_forward_) > msg->axes.size())
                    {
                        return;
                    }

                    float axis_value = msg->axes[abs(change_forward_)];

                    if ((axis_value > 0 && change_forward_ > 0) || (axis_value < 0 && change_forward_ < 0))
                    {
                        nextProfile();

                    } else if ((axis_value > 0 && change_backward_ > 0) || (axis_value < 0 && change_backward_ < 0))
                    {
                        previousProfile();
                    }
                }
            }

        }
    }
}

void ChangeProfile::nextProfile()
{
    // if current is last element go to begin
    if (current_profile_ == --profiles_.end())
    {
        setCurrentProfile(profiles_.begin());
    } else
    {
        setCurrentProfile(++current_profile_);
    }
}

void ChangeProfile::previousProfile()
{
    // if current is first element go to end
    if (current_profile_ == profiles_.begin())
    {
        setCurrentProfile(--profiles_.end());
    } else
    {
        setCurrentProfile(--current_profile_);
    }
}

void ChangeProfile::setCurrentProfile(std::vector<Profile>::iterator new_position)
{
    current_profile_ = new_position;

    ROS_INFO_STREAM("hector_joy_teleop_with_plugins: Current profile: " << current_profile_->name);

    // unload all plugins
    load_teleop_plugin_srv_msg_.request.plugin_name = "all";
    load_teleop_plugin_srv_msg_.request.load = false;

    load_teleop_plugins_srv_client_.call(load_teleop_plugin_srv_msg_);

    if (load_teleop_plugin_srv_msg_.response.result != load_teleop_plugin_srv_msg_.response.SUCCESS)
    {
        ROS_WARN_STREAM(
            "Error changing profile when unloading all plugins. No new plugins were loaded. Please reload profile.");
        return;
    }

    // load requested plugins
    for (auto& plugin : current_profile_->plugins)
    {
        load_teleop_plugin_srv_msg_.request.plugin_name = plugin;
        load_teleop_plugin_srv_msg_.request.load = true;

        load_teleop_plugins_srv_client_.call(load_teleop_plugin_srv_msg_);

        if (load_teleop_plugin_srv_msg_.response.result != load_teleop_plugin_srv_msg_.response.SUCCESS)
        {
            ROS_WARN_STREAM(
                "Error changing profile: The plugin " + plugin + " could not be loaded. Please reload profile.");
        }
    }

    // TODO publish current profile

}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::ChangeProfile, hector_joy_teleop_plugin_interface::TeleopBase)