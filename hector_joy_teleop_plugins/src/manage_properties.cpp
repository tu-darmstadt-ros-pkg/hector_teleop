#include "hector_joy_teleop_plugins/manage_properties.h"

namespace hector_joy_teleop_plugins
{

void ManageProperties::initialize(ros::NodeHandle& nh,
                                  ros::NodeHandle& pnh,
                                  std::shared_ptr<std::map<std::string, double>> property_map,
                                  std::string plugin_name)
{
    TeleopBase::initializeBase(nh, pnh, property_map, plugin_name, "hector_joy_teleop_plugins::ManageProperties");

    // get start values from config file
    property_map_->emplace("direction", pnh_.param<double>(getParameterServerPrefix() + "/" + "direction", 1.0));


    // setup service and topics
    current_direction_topic_ = pnh.param<std::string>(getParameterServerPrefix() + "/" + "current_direction_topic", "/joy_teleop_direction");
    current_direction_pub_ = nh_.advertise<std_msgs::Float64>(current_direction_topic_, 10, true);

    set_property_service_ = pnh_.advertiseService("set_property", &ManageProperties::SetPropertyServiceCB, this);

    direction_finished_ = true;
}

void ManageProperties::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // change direction parameter
    float direction;
    if (getJoyMeasurement("direction", msg, direction))
    {
        // if direction button is pressed (and was 0 before): reverse current direction
        if(direction != 0 && direction_finished_)
        {
            direction_finished_ = false;
            if (property_map_->at("direction") == 1.0)
            {
                property_map_->at("direction") = -1.0;
            } else
            {
                property_map_->at("direction") = 1.0;
            }

            // publish change for UI
            publishPropertyChanged("direction");

        }
        else if(direction == 0 && !direction_finished_)
        {
            direction_finished_ = true;
        }

    }

}

void ManageProperties::executePeriodically(const ros::Rate& rate)
{
}

void ManageProperties::publishPropertyChanged(std::string property_name)
{
    if (property_name == "direction")
    {
        std_msgs::Float64 msg;
        msg.data = property_map_->at("direction");

        current_direction_pub_.publish(msg);
    }
}

bool ManageProperties::SetPropertyServiceCB(hector_joy_teleop_plugin_msgs::SetProperty::Request& request,
                                            hector_joy_teleop_plugin_msgs::SetProperty::Response& response)
{
    auto iter = property_map_->find(request.property_name);

    // Check if property exists
    if (iter == property_map_->end())
    {
        response.result = response.UNKNOWN_PROPERTYNAME;
    } else
    {
        if (request.property_name == "direction")
        {
            // for direction ensure that only +/-1.0 are set as values
            iter->second = (request.value == -1.0) ? -1.0 : 1.0;
        } else
        {
            // for all properties without a value check set the send value
            iter->second = request.value;
        }

        response.result = response.SUCCESS;
        publishPropertyChanged(request.property_name);

    }

    return true;
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::ManageProperties, hector_joy_teleop_plugin_interface::TeleopBase)