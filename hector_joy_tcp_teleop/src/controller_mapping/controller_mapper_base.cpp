#include <hector_joy_tcp_teleop/controller_mapping/controller_mapper_base.h>

namespace hector_joy_tcp_teleop {

ControllerMapperBase::~ControllerMapperBase() {}

bool ControllerMapperBase::isPressed(const sensor_msgs::Joy& joy) const
{
  double cmd = computeCommand(joy);
  return (cmd > 0.0);
}

double ControllerMapperBase::scale() const {
  return scale_;
}

void ControllerMapperBase::setScale(double scale) {
  scale_ = scale;
}

}
