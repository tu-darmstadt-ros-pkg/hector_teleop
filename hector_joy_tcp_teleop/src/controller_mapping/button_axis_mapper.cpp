#include <hector_joy_tcp_teleop/controller_mapping/button_axis_mapper.h>

namespace hector_joy_tcp_teleop {

ButtonAxisMapper::ButtonAxisMapper(size_t button_inc_index, size_t button_dec_index)
  : button_inc_index_(button_inc_index), button_dec_index_(button_dec_index)
{}

double ButtonAxisMapper::computeCommand(const sensor_msgs::Joy& joy) const
{
  if (button_inc_index_ >= joy.buttons.size()) {
    ROS_ERROR_STREAM("Button index " << button_inc_index_ << " is out of bounds (Size: " << joy.buttons.size() << ").");
    return 0.0;
  }
  if (button_dec_index_ >= joy.buttons.size()) {
    ROS_ERROR_STREAM("Button index " << button_dec_index_ << " is out of bounds (Size: " << joy.buttons.size() << ").");
    return 0.0;
  }
  return scale() * static_cast<double>(joy.buttons[button_inc_index_] - joy.buttons[button_dec_index_]);
}

}
