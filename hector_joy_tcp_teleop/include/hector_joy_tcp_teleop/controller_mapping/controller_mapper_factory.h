#ifndef MOVEIT_JOYSTICK_CONTROL_CONTROLLER_MAPPER_FACTORY
#define MOVEIT_JOYSTICK_CONTROL_CONTROLLER_MAPPER_FACTORY

#include <ros/ros.h>

#include <hector_joy_tcp_teleop/controller_mapping/axis_mapper.h>
#include <hector_joy_tcp_teleop/controller_mapping/button_axis_mapper.h>
#include <hector_joy_tcp_teleop/controller_mapping/button_mapper.h>
#include <hector_joy_tcp_teleop/controller_mapping/dual_axis_mapper.h>

namespace hector_joy_tcp_teleop {

std::shared_ptr<ControllerMapperBase> switchController(const ros::NodeHandle& nh) {
  // Simple axis mapping
  int axis_index;
  if (nh.getParam("axis_index", axis_index)) {
    if (axis_index >= 0)  {
      return std::make_shared<AxisMapper>(axis_index);
    } else {
      ROS_ERROR_STREAM("axis_index has to be non-negative.");
    }
  }
  // Simple button mapping
  int button_index;
  if (nh.getParam("button_index", button_index)) {
    if (button_index >= 0) {
      return std::make_shared<ButtonMapper>(button_index);
    } else {
      ROS_ERROR_STREAM("button_index has to be non-negative.");
    }
  }
  // Two buttons as one axis
  int button_inc_index, button_dec_index;
  if (nh.getParam("button_inc_index", button_inc_index) && nh.getParam("button_dec_index", button_dec_index)) {
    if (button_inc_index >= 0 && button_dec_index >= 0) {
      return std::make_shared<ButtonAxisMapper>(button_inc_index, button_dec_index);
    } else {
      ROS_ERROR_STREAM("Button indices need to be non-negative.");
    }
  }
  // Two axis as one axis
  int axis_inc_index, axis_dec_index;
  if (nh.getParam("axis_inc_index", axis_inc_index) && nh.getParam("axis_dec_index", axis_dec_index)) {
    if (axis_inc_index >= 0 && axis_dec_index >= 0) {
      return std::make_shared<DualAxisMapper>(axis_inc_index, axis_dec_index);
    } else {
      ROS_ERROR_STREAM("Axis indices need to be non-negative.");
    }
  }

  ROS_ERROR_STREAM("None of the required parameters found to create a controller mapper.");
  return std::shared_ptr<ControllerMapperBase>();
}

std::shared_ptr<ControllerMapperBase> createControllerMapper(const ros::NodeHandle& nh) {
  std::shared_ptr<ControllerMapperBase> controller_mapper = switchController(nh);

  if (!controller_mapper) {
    return controller_mapper;
  }

  double scale;
  nh.param<double>("scale", scale, 1.0);
  controller_mapper->setScale(scale);

  return controller_mapper;
}



}

#endif
