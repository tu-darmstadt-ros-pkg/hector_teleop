#ifndef MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER
#define MOVEIT_JOYSTICK_CONTROL_AXIS_MAPPER

#include <hector_joy_tcp_teleop/controller_mapping/controller_mapper_base.h>

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace hector_joy_tcp_teleop {

class AxisMapper : public ControllerMapperBase {
public:
  AxisMapper(size_t axis_index);
  virtual double computeCommand(const sensor_msgs::Joy& joy) const override;
private:
  size_t axis_index_;
};

}

#endif
