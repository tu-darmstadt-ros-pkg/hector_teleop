
#include "hector_joy_teleop_plugin_interface/teleop_base.h"

namespace hector_joy_teleop_plugin_interface
{

void TeleopBase::initializeBase(ros::NodeHandle& nh,
                                ros::NodeHandle& pnh,
                                std::shared_ptr<std::map<std::string, double>> property_map,
                                std::string plugin_name)
{
    nh_ = nh;
    pnh_ = pnh;

    // get indices for left and right trigger
    left_trigger_ = pnh_.param<double>("left_trigger", 2);
    right_trigger_ = pnh_.param<double>("right_trigger", 5);

    property_map_ = property_map;

    std::string delimiter = ":";
    plugin_namespace_ = plugin_name.substr(0, plugin_name.find(delimiter));
    plugin_name_ = plugin_name.substr(plugin_name.find_last_of(delimiter) + 1);


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
            std::string name = axes[i];
            axes_[name] = i;


            /// NOTE here a Problem occurs if two plugins which both use one part of same axis are loaded parallel
            // if there is a semicolon in the name the axis is split into two buttons or partial axes
            // (but due to check for overlapping button mappings it is stored in axes_ map AND in axis_split_ map)
            auto pos_delimiter = name.find(';');
            if (pos_delimiter != std::string::npos)
            {
                if (!name.substr(0, pos_delimiter).empty())
                {
                    axis_split_[name.substr(0, pos_delimiter)] = std::make_pair<int, bool>(i, false);
                }

                if (!name.substr(pos_delimiter + 1, std::string::npos).empty())
                {
                    axis_split_[name.substr(pos_delimiter + 1, std::string::npos)] =
                        std::make_pair<int, bool>(i, true);
                }
            }
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


bool TeleopBase::getJoyMeasurement(std::string name,
                                   const sensor_msgs::JoyConstPtr& msg,
                                   float& result,
                                   bool print_missing_parameter)
{
    // also search for name with suffixes inc and dec if two buttons or partial axes were used to create one axis
    std::string name_inc = name + "_inc";
    std::string name_dec = name + "_dec";

    float inc_result = 0, dec_result = 0;
    bool inc_found = false;
    bool dec_found = false;


    // axes: name
    auto it_axes = axes_.find(name);
    if (it_axes != axes_.end())
    {
        result = msg->axes[it_axes->second];
        return true;
    }

    // axes: name_inc
    it_axes = axes_.find(name_inc);
    if (it_axes != axes_.end())
    {
        // use whole axes here as it was written in configuration file without a semicolon (if partial axes requested see below in axis_split)
        inc_result = msg->axes[it_axes->second];
        inc_found = true;
    }

    // axes: name_dec
    it_axes = axes_.find(name_dec);
    if (it_axes != axes_.end())
    {
        // use whole axes here as it was written in configuration file without a semicolon (if partial axes requested see below in axis_split)
        dec_result = msg->axes[it_axes->second];
        dec_found = true;
    }


    // buttons: name
    auto it_buttons = buttons_.find(name);
    if (it_buttons != buttons_.end())
    {
        result = msg->buttons[it_buttons->second];
        return true;
    }

    // buttons: name_inc
    it_buttons = buttons_.find(name_inc);
    if (it_buttons != buttons_.end())
    {
        inc_result = msg->buttons[it_buttons->second];
        inc_found = true;
    }

    // buttons: name_dec
    it_buttons = buttons_.find(name_dec);
    if (it_buttons != buttons_.end())
    {
        dec_result = msg->buttons[it_buttons->second];
        dec_found = true;
    }


    // search in axis_split map if an axis was split into two buttons or two partial axes
    // axis_split: name
    auto it_axis_split = axis_split_.find(name);
    if (it_axis_split != axis_split_.end())
    {
        double axis_result = msg->axes[it_axis_split->second.first];

        // requested part of axis is pressed,
        // as the name itself without suffix can only be used here if it is not requested as partial axis, it must be a button request
        if ((axis_result < 0 && !it_axis_split->second.second)
            || (axis_result > 0 && it_axis_split->second.second))
        {
            // return "button pressed"
            result = 1;
        } else
        {
            result = 0;
        }
        return true;
    }


    // axis_split: name_inc
    it_axis_split = axis_split_.find(name_inc);
    if (it_axis_split != axis_split_.end())
    {
        float axis_result = msg->axes[it_axis_split->second.first];

        // if requested part of axis is pressed return its absolute value (otherwise its value is 0)
        if ((axis_result < 0 && !it_axis_split->second.second)
            || (axis_result > 0 && it_axis_split->second.second))
        {
            // get absolute value of axes
            inc_result = abs(axis_result);
        }

        inc_found = true;
    }

    // axis_split: name_dec
    it_axis_split = axis_split_.find(name_dec);
    if (it_axis_split != axis_split_.end())
    {
        float axis_result = msg->axes[it_axis_split->second.first];

        // if requested part of axis is pressed return its absolute value (otherwise its value is 0)
        if ((axis_result < 0 && !it_axis_split->second.second)
            || (axis_result > 0 && it_axis_split->second.second))
        {
            // get absolute value of axes
            dec_result = abs(axis_result);
        }

        dec_found = true;
    }


    // if values for both parts of axis are found, combine them into one axis result
    if(inc_found && dec_found)
    {
        result = inc_result - dec_result;
        return true;
    }


    // if name was found in none of these maps print a message (if requested) and return false
    if (print_missing_parameter)
    {
        ROS_ERROR_STREAM(plugin_name_ << ": The required axis/button mapping for value \"" << name
                                      << "\" or at least one of its parts \"" << name_inc << "\" or \"" << name_dec << "\" is missing or maybe misspelled.");
    }

    return false;
}

bool TeleopBase::getJoyMeasurement(std::string name, const sensor_msgs::JoyConstPtr& msg, bool& result, bool print_missing_parameter)
{
  float value;
  if (!getJoyMeasurement(name, msg, value, print_missing_parameter)) {
    return false;
  } else {
    result = (value != 0.0f);
    return true;
  }
}

sensor_msgs::JoyPtr TeleopBase::mapTriggerAxes(const sensor_msgs::JoyConstPtr& msg)
{
    // copy const message into a new non-const one
    sensor_msgs::JoyPtr newMsg = boost::make_shared<sensor_msgs::Joy>(*msg);

    // map trigger from [-1,1] (with 1 as default) to [0,1] (with 0 as default)
    newMsg->axes[left_trigger_] = (msg->axes[left_trigger_] - 1) * (-1) * (0.5);
    newMsg->axes[right_trigger_] = (msg->axes[right_trigger_] - 1) * (-1) * (0.5);

    return newMsg;
}

bool TeleopBase::hasUsedMsg()
{
    // store value
    bool tmp = used_msg_;

    // reset variable
    used_msg_ = false;

    return tmp;
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

std::string TeleopBase::getParameterServerPrefix()
{
    return plugin_namespace_ + "/" + plugin_name_;
}

TeleopBase::~TeleopBase()
{

}

}