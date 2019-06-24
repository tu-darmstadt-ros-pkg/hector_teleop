#include <hector_joy_tcp_teleop/controller_mapping/axis_mapper.h>

namespace hector_joy_tcp_teleop {

AxisMapper::AxisMapper(size_t axis_index)
  : ControllerMapperBase(), axis_index_(axis_index)
{}

double AxisMapper::computeCommand(const sensor_msgs::Joy& joy) const
{
  if (axis_index_ < joy.axes.size()) {
    return scale() * static_cast<double>(joy.axes[axis_index_]);
  } else {
    ROS_ERROR_STREAM("Axis index " << axis_index_ << " is out of bounds (Size: " << joy.axes.size() << ").");
    return 0.0;
  }
}

}
