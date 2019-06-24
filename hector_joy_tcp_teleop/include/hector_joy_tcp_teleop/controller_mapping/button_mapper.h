#ifndef MOVEIT_JOYSTICK_CONTROL_BUTTON_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_BUTTON_MAPPER

#include <hector_joy_tcp_teleop/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace hector_joy_tcp_teleop {

class ButtonMapper : public ControllerMapperBase {
public:
  ButtonMapper(size_t button_index);
  double computeCommand(const sensor_msgs::Joy& joy) const override;

private:
  size_t button_index_;
};

}

#endif
