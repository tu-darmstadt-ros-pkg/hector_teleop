#ifndef MOVEIT_JOYSTICK_CONTROL_BUTTON_AXIS_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_BUTTON_AXIS_MAPPER

#include <hector_joy_tcp_teleop/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace hector_joy_tcp_teleop {

class ButtonAxisMapper : public ControllerMapperBase {
public:
  ButtonAxisMapper(size_t button_inc_index, size_t button_dec_index);
  double computeCommand(const sensor_msgs::Joy& joy) const override;

private:
  size_t button_inc_index_;
  size_t button_dec_index_;
};

}

#endif
