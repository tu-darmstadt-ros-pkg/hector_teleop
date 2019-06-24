#include <hector_joy_tcp_teleop/controller_mapping/dual_axis_mapper.h>

namespace hector_joy_tcp_teleop {

DualAxisMapper::DualAxisMapper(size_t axis_inc_index, size_t axis_dec_index)
  : ControllerMapperBase(), axis_inc_index_(axis_inc_index), axis_dec_index_(axis_dec_index)
{}

double DualAxisMapper::computeCommand(const sensor_msgs::Joy& joy) const
{
  if (axis_inc_index_ >= joy.axes.size()) {
    ROS_ERROR_STREAM("Axis index " << axis_inc_index_ << " is out of bounds (Size: " << joy.axes.size() << ").");
    return 0.0;
  }
  if (axis_dec_index_ >= joy.axes.size()) {
    ROS_ERROR_STREAM("Axis index " << axis_dec_index_ << " is out of bounds (Size: " << joy.axes.size() << ").");
    return 0.0;
  }
  return scale() * static_cast<double>(joy.axes[axis_inc_index_] - joy.axes[axis_dec_index_]);
}

}
