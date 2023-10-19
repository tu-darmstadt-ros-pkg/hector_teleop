
#include "hector_joy_teleop_plugins/flipper_teleop.h"




namespace hector_joy_teleop_plugins
{
void FlipperTeleop::initialize(ros::NodeHandle& nh,
                               ros::NodeHandle& pnh,
                               std::shared_ptr<std::map<std::string, double>> property_map,
                               std::string plugin_name)
{
    TeleopBase::initializeBase(nh, pnh, property_map, plugin_name, "hector_joy_teleop_plugins::FlipperTeleop");

    speed_ = pnh_.param<float>(getParameterServerPrefix() + "/" + "speed", 0.0);

    // factors to adapt commands (e.g. to inverse an axis)
    flipper_front_factor_ = pnh_.param<float>(getParameterServerPrefix() + "/" + "flipper_front_factor", 1.0);
    flipper_back_factor_  = pnh_.param<float>(getParameterServerPrefix() + "/" + "flipper_back_factor", 1.0);


    // get flipper topics
    flipper_front_command_topic_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "flipper_front_command_topic",
                                "/flipper_control/flipper_front_velocity_controller/command");
    flipper_back_command_topic_ =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "flipper_back_command_topic",
                                "/flipper_control/flipper_back_velocity_controller/command");

    // init publisher
    flipper_front_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_front_command_topic_, 10, false);
    flipper_back_pub_ =
        nh_.advertise<std_msgs::Float64>(flipper_back_command_topic_, 10, false);


    // get names for switching controllers
    standard_controllers_ =
        pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "standard_controllers",
                                             {});
    teleop_controllers_ = pnh_.param<std::vector<std::string>>(getParameterServerPrefix() + "/" + "teleop_controllers",
                                                               {});
    // get values for switching controllers
    std::string controller_manager_switch_service =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_switch_service",
                                "/flipper_control/controller_manager/switch_controller");
    std::string controller_manager_list_service =
        pnh_.param<std::string>(getParameterServerPrefix() + "/" + "controller_manager_list_service",
                                "/flipper_control/controller_manager/list_controllers");


    int num_tries_switch_controller =
        pnh_.param<int>(getParameterServerPrefix() + "/" + "num_tries_switch_controller", 5);
    int sleep_time = pnh_.param<int>(getParameterServerPrefix() + "/" + "sleep_between_tries_sec", 1);


    // init ControllerHelper for switching services later
    controller_helper_ = ControllerHelper(pnh,
                                          controller_manager_switch_service,
                                          controller_manager_list_service,
                                          num_tries_switch_controller,
                                          sleep_time,
                                          plugin_name_);
    //subscribe to action server for flipper_auto_lower_feature
    client_ = new Client("lower_flipper",true);
    
    ROS_INFO("Waiting for action server (flipper_auto_lower_feature) to start.");
    if(!client_->waitForServer(ros::Duration(10))) {
        ROS_ERROR("Couldn't connect to action server (flipper_auto_lower_feature) in time.");
         flipper_auto_lower_feature_ = false;
    }
    else{
        ROS_INFO("Action server started (flipper_auto_lower_feature).");
    } 
 

}

void FlipperTeleop::forwardMsg(const sensor_msgs::JoyConstPtr& msg)
{
    // if the direction value is available in map and it is -1.0, reverse mode is active (handled when published!)
    auto iter = property_map_->find("direction");
    bool reverse_direction = (iter != property_map_->end()) && (iter->second == -1.0);

    // map trigger axis
    sensor_msgs::JoyPtr mappedMsg = mapTriggerAxes(msg);


    // check if last command was zero
    bool last_cmd_zero = abs(flipper_front_command_.data) < 0.05 && abs(flipper_back_command_.data) < 0.05;


    // compute flipper commands
    joyToFlipperCommand(mappedMsg);


    // check if current command is zero
    bool current_cmd_zero = abs(flipper_front_command_.data) < 0.05 && abs(flipper_back_command_.data) < 0.05;

    // if last command was zero and current is not, switch controllers
    if(last_cmd_zero && !current_cmd_zero)
    {
        std::string result = controller_helper_.switchControllers(teleop_controllers_, standard_controllers_);

        if(!result.empty())
        {
            ROS_ERROR_STREAM(result);
        }
    }

    // publish
    if(!flipper_auto_lower_feature_running_) {
        if (!reverse_direction)
        {
            flipper_front_command_.data *= flipper_front_factor_;
            flipper_back_command_.data *= flipper_back_factor_;

            flipper_front_pub_.publish(flipper_front_command_);
            flipper_back_pub_.publish(flipper_back_command_);
            //ROS_ERROR("Published front: %f, Published back: %f \n",flipper_front_command_.data,flipper_back_command_.data); 
        } else
        {
            // in reverse mode also reverse button mapping for front and back flippers, hence swap commands (and factors)
            flipper_front_command_.data *= flipper_back_factor_;
            flipper_back_command_.data *= flipper_front_factor_;

            flipper_front_pub_.publish(flipper_back_command_);
            flipper_back_pub_.publish(flipper_front_command_);
        }

    }

}




void FlipperTeleop::triggerFlipperAuto (std::vector<float> val) {
    std::cout << "cross " << val[0] << " r2 " << val[1] << " r1 " << val[2] << " l2 " << val[3] << " l1 " << val[4] << std::endl;
    // bool* volatile set;
    // bool* volatile inter;
    // double* volatile first;
    // //data-race solution
    // if(dir == "front") {
    //     set = &f_set_;
    //     inter = &f_inter_;
    //     first = &f_first_;
    // } 
    // else {
    //     set = &b_set_;
    //     inter = &b_inter_;
    //     first = &b_first_;
    // }
    // //end data-race solution
    // //check if action is already in progress and it needs to be canceled
    // if(flipper_auto_lower_feature_running_) {
    //     if(val) {
    //         client_->cancelAllGoals();
    //         //client_->waitForResult() does this work?
    //         ROS_INFO("Goal canceled successfully (flipper_auto_lower_feature).");
    //         //setting up method for next event
    //         *first = msg->header.stamp.toSec();
    //         *inter = false;
    //         flipper_auto_lower_feature_running_ = false;
    //         return;
    //     }
    //     if(client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED || client_->getState() == actionlib::SimpleClientGoalState::ABORTED) {
    //         //setting up method for next event
    //         *first = msg->header.stamp.toSec();
    //         *inter = false;
    //         flipper_auto_lower_feature_running_ = false;
    //         return;
    //     }
    //     //still running -> do nothing
    //     return;
    // }
    // //init first R1 or L1 value
    // if(!(*set) && val == 1) {
    //     *first = msg->header.stamp.toSec();
    //     *set = true;
    //     ROS_ERROR("Detected first click with dir: %s", dir.c_str());
    //     return;
    // }
    // //detect if button has been released
    // if((*set) && val == 0) {
    //     *inter = true;
    //     //ROS_ERROR("Detected intermediate");
    //     return;
    // }
    // //std::cout << *inter << std::endl;
    // //compute if second click was fast enough and trigger feature 
    // if((*inter) && (*set) && val == 1) {
    //     //ROS_ERROR("Detected double click: %s",dir.c_str());
    //     double interval = abs((double)(*first) - msg->header.stamp.toSec());
    //     //ROS_ERROR("Detected double click: %s , within %f not sure if inter", dir.c_str(),interval);
    //     if(interval < 0.2) { //0.1 worked fine but can be changed (time between two clicks)
    //         ROS_ERROR("Detected double click: %s , within %f", dir.c_str(),interval);
    //         flipper_auto_lower_feature_running_ = true;
    //         flipper_auto_control_msgs::LowerFlipperGoal goal;
    //         goal.flipper = dir; 
    //         client_->sendGoal(goal);
    //     }
    // }
    // //setting up method for next event
    // *first = msg->header.stamp.toSec();
    // *inter = false;
}


void FlipperTeleop::joyToFlipperCommand(const sensor_msgs::JoyConstPtr& msg)
{
    //flipper auto lower feature
    float cross_down, trigger_right,trigger_left,button_right,button_left;
    //if(flipper_auto_lower_feature_ && getJoyMeasurement("cross_down_up",msg,cross_down) && (getJoyMeasurement("front_dec",msg,trigger_right) || getJoyMeasurement("back_dec",msg,trigger_left)) && (getJoyMeasurement("back_inc",msg,button_left) || getJoyMeasurement("front_inc",msg,button_right))) {
    //    FlipperTeleop::triggerFlipperAuto(std::vector<float>({cross_down,trigger_right,button_right,trigger_left,button_left}));
    //}
    // front flipper
    float front_joystick;
    if (getJoyMeasurement("front", msg, front_joystick))
    {   
        flipper_front_command_.data = front_joystick * speed_;
    }

    // back flipper
    float back_joystick;
    if (getJoyMeasurement("back", msg, back_joystick))
    {
        flipper_back_command_.data = back_joystick * speed_;
    }
}





std::string FlipperTeleop::onLoad()
{
    return controller_helper_.switchControllers(teleop_controllers_, standard_controllers_);
}

std::string FlipperTeleop::onUnload()
{
    return controller_helper_.switchControllers(standard_controllers_, teleop_controllers_);
}

}

PLUGINLIB_EXPORT_CLASS(hector_joy_teleop_plugins::FlipperTeleop, hector_joy_teleop_plugin_interface::TeleopBase)
