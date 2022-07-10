
#ifndef HECTOR_JOY_TELEOP_PLUGINS_COMMON_H
#define HECTOR_JOY_TELEOP_PLUGINS_COMMON_H

namespace hector_joy_teleop_plugins {

enum ResponseCurveMode {
  Linear,
  Parabola
};

template <typename Scalar>
Scalar applyResponseCurve(Scalar value, ResponseCurveMode curve)
{
  switch (curve)
  {
    case Parabola:
      return (value < 0 ? Scalar(-1) : Scalar(1)) * value * value;
    case Linear:
    default:
      return value;
  }
}

}

#endif  // HECTOR_JOY_TELEOP_PLUGINS_COMMON_H
