#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <monstertruck_msgs/MotionCommand.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace JoyTeleop {

  ros::Subscriber joyInput;
  ros::Publisher motionCommandOutput;
  ros::Publisher cameraCommandOutput;

  monstertruck_msgs::MotionCommand motionCommand;
  geometry_msgs::QuaternionStamped cameraCommand;

  int axis_speed;
  double speedForward;
  double speedBackward;
  int axis_steer;
  double steerAngle;
  int button_crab;
  int button_slow;
  int button_brake;
  double slowFactor;

  int axis_cameraPan;
  int axis_cameraTilt;
  int button_cameraReset;
  std::string cameraMode;
  double cameraSpeed;
  double cameraMaxPan;
  double cameraMaxTiltDown;
  double cameraMaxTiltUp;

  double cameraPan = 0;
  double cameraTilt = 0;
  double cameraPanSpeed = 0;
  double cameraTiltSpeed = 0;

  void publishCamera();

  void joyCallback(const sensor_msgs::JoyConstPtr joystick) {
    if (axis_speed > 0 && (size_t) axis_speed <= joystick->axes.size()) {
      float value = joystick->axes[axis_speed-1];
      if (value >= 0.0)
        motionCommand.speed = value * speedForward;
      else
        motionCommand.speed = value * speedBackward;

      if (button_slow > 0 && (size_t) button_slow <= joystick->buttons.size() && joystick->buttons[button_slow-1]) {
        motionCommand.speed *= slowFactor;
      }
    }

    if (button_brake > 0 && (size_t) button_brake <= joystick->buttons.size()) {
      motionCommand.brake = joystick->buttons[button_brake-1];
    }

    if (axis_steer > 0 && (size_t) axis_steer <= joystick->axes.size()) {
      motionCommand.steerAngleFront = joystick->axes[axis_steer-1] * steerAngle * M_PI/180.0;
      motionCommand.steerAngleRear  = motionCommand.steerAngleFront;

      if (button_crab > 0 && (size_t) button_crab <= joystick->buttons.size()) {
        if (joystick->buttons[button_crab-1]) {
          if (motionCommand.speed >= 0.0) {
            motionCommand.steerAngleRear  = -motionCommand.steerAngleRear;
          } else {
            motionCommand.steerAngleFront = -motionCommand.steerAngleFront;
          }
        }
      }
    }

    motionCommandOutput.publish(motionCommand);

    if (axis_cameraPan > 0 && (size_t) axis_cameraPan <= joystick->axes.size()) {
      cameraPanSpeed = joystick->axes[axis_cameraPan-1] * cameraSpeed * M_PI/180.0;
    }
    if (axis_cameraTilt > 0 && (size_t) axis_cameraTilt <= joystick->axes.size()) {
      cameraTiltSpeed = joystick->axes[axis_cameraTilt-1] * cameraSpeed * M_PI/180.0;
    }
    if (button_cameraReset > 0 && (size_t) button_cameraReset <= joystick->buttons.size() && joystick->buttons[button_cameraReset-1]) {
      cameraPan = 0;
      cameraTilt = 0;
      publishCamera();
    }
  }

  void moveCamera(double dt) {
    if (cameraPanSpeed == 0.0 && cameraTiltSpeed == 0.0) return;

    cameraPan += dt * cameraPanSpeed;
    if (cameraPan >  cameraMaxPan*M_PI/180.0) cameraPan =  cameraMaxPan*M_PI/180.0;
    if (cameraPan < -cameraMaxPan*M_PI/180.0) cameraPan = -cameraMaxPan*M_PI/180.0;

    cameraTilt += dt * cameraTiltSpeed;
    if (cameraTilt >  cameraMaxTiltDown*M_PI/180.0) cameraTilt =  cameraMaxTiltDown*M_PI/180.0;
    if (cameraTilt < -cameraMaxTiltUp  *M_PI/180.0) cameraTilt = -cameraMaxTiltUp  *M_PI/180.0;

    publishCamera();
  }

  void publishCamera() {
    cameraCommand.header.stamp = ros::Time::now();
    cameraCommand.header.frame_id = cameraMode;
    cameraCommand.quaternion.w =  cos(cameraPan/2)*cos(cameraTilt/2);
    cameraCommand.quaternion.x = -sin(cameraPan/2)*sin(cameraTilt/2);
    cameraCommand.quaternion.y =  cos(cameraPan/2)*sin(cameraTilt/2);
    cameraCommand.quaternion.z =  sin(cameraPan/2)*cos(cameraTilt/2);

    cameraCommandOutput.publish(cameraCommand);
  }
}; // namespace JoyTeleop

using namespace JoyTeleop;

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle n;

  joyInput = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  motionCommandOutput = n.advertise<monstertruck_msgs::MotionCommand>("drive", 10, false);
  cameraCommandOutput = n.advertise<geometry_msgs::QuaternionStamped>("camera/command", 10, false);

  ros::param::param("~axis_speed", JoyTeleop::axis_speed, 0);
  ros::param::param("~speed_forward", JoyTeleop::speedForward, 1.0);
  ros::param::param("~speed_backward", JoyTeleop::speedBackward, 1.0);
  ros::param::param("~axis_steer", JoyTeleop::axis_steer, 0);
  ros::param::param("~steer_angle", JoyTeleop::steerAngle, 30.0);
  ros::param::param("~button_brake", JoyTeleop::button_brake, 0);
  ros::param::param("~button_crab", JoyTeleop::button_crab, 0);
  ros::param::param("~button_slow", JoyTeleop::button_slow, 0);
  ros::param::param("~slow_factor", JoyTeleop::slowFactor, 0.5);

  ros::param::param("~axis_camera_pan", JoyTeleop::axis_cameraPan, 0);
  ros::param::param("~axis_camera_tilt", JoyTeleop::axis_cameraTilt, 0);
  ros::param::param("~button_camera_reset", JoyTeleop::button_cameraReset, 0);
  ros::param::param("~camera_mode", JoyTeleop::cameraMode, std::string("base_stabilized"));
  ros::param::param("~camera_speed", JoyTeleop::cameraSpeed, 60.0);
  ros::param::param("~camera_max_pan", JoyTeleop::cameraMaxPan, 120.0);
  ros::param::param("~camera_max_tilt_down", JoyTeleop::cameraMaxTiltDown, 30.0);
  ros::param::param("~camera_max_tilt_up", JoyTeleop::cameraMaxTiltUp, 30.0);

  ros::Rate rate(50.0);
  while(ros::ok()) {
    if (motionCommand.steerAngleFront != 0.0 ||
        motionCommand.steerAngleRear != 0.0 ||
        motionCommand.speed != 0.0)
      motionCommandOutput.publish(motionCommand);
    moveCamera(rate.expectedCycleTime().toSec());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
