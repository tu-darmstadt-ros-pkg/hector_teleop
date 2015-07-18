//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Float64.h>

namespace JoyTeleop {

ros::Subscriber joyInput;
ros::Publisher motionCommandOutput;
ros::Publisher cameraCommandOutput;

geometry_msgs::Twist motionCommand;
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

ros::Publisher flipperCmdAbsPub_;
ros::Publisher flipperCmdRelPub_;

double flipperFrontCommand;
double lastFlipperFrontCommand;
bool flipper_front_stop;
bool flipper_front_up;
bool flipper_front_down;
double currentStateFlipperFront;

int button_flipper_front_down;
int button_flipper_front_up;

int axis_flipper_front_up_down;
int axis_flipper_front_left_right;

double flipper_sensitivity;
double flipper_front_position_absolute_up;
double flipper_front_position_absolute_down;
double flipper_front_position_absolute_center;


void publishCamera();

void publishFlipperFrontAbsolute(double position){
    std_msgs::Float64 msg;
    msg.data = position;
    flipperCmdAbsPub_.publish(msg);
}

void publishFlipperFrontRelative(double movement){
    std_msgs::Float64 msg;
    msg.data = movement;
    flipperCmdRelPub_.publish(msg);
}

void joyCallback(const sensor_msgs::JoyConstPtr joystick) {
    if (axis_speed > 0 && (size_t) axis_speed <= joystick->axes.size()) {
        float value = joystick->axes[axis_speed-1];
        if (value >= 0.0)
            motionCommand.linear.x = value * speedForward;
        else
            motionCommand.linear.x = value * speedBackward;

        if (button_slow > 0 && (size_t) button_slow <= joystick->buttons.size() && joystick->buttons[button_slow-1]) {
            motionCommand.linear.x *= slowFactor;
        }
    }

    //Brake not applicable on many vehicles
    //if (button_brake > 0 && (size_t) button_brake <= joystick->buttons.size()) {
    //  motionCommand.brake = joystick->buttons[button_brake-1];
    //}

    if (axis_steer > 0 && (size_t) axis_steer <= joystick->axes.size()) {
        motionCommand.angular.z = joystick->axes[axis_steer-1] * steerAngle * M_PI/180.0;

        //Angle not applicable on many vehicles
        //motionCommand.steerAngleRear  = motionCommand.steerAngleFront;

        /*
      if (button_crab > 0 && (size_t) button_crab <= joystick->buttons.size()) {
        if (joystick->buttons[button_crab-1]) {
          if (motionCommand.speed >= 0.0) {
            motionCommand.steerAngleRear  = -motionCommand.steerAngleRear;
          } else {
            motionCommand.steerAngleFront = -motionCommand.steerAngleFront;
          }
        }
      }
      */
    }

    bool current_flipper_up = false;
    bool current_flipper_down = false;
    if (button_flipper_front_down > 0 && (size_t) button_flipper_front_down <= joystick->buttons.size()){
        current_flipper_up = joystick->buttons[button_flipper_front_down-1];
    }


    if (button_flipper_front_up > 0 && (size_t) button_flipper_front_up <= joystick->buttons.size())
    {
        current_flipper_down = joystick->buttons[button_flipper_front_up-1];
    }

    if((flipper_front_down && !current_flipper_down) || (flipper_front_up && !current_flipper_up)){
        flipper_front_stop = true;
    }
    flipper_front_down = current_flipper_down;
    flipper_front_up = current_flipper_up;


    if (axis_flipper_front_up_down > 0 && (size_t) axis_flipper_front_up_down <= joystick->axes.size()) {
        if(fabs(joystick->axes[axis_flipper_front_up_down-1]) > 0.1){
            if(joystick->axes[axis_flipper_front_up_down-1] > 0){
                publishFlipperFrontAbsolute(flipper_front_position_absolute_up);
            }else{
                publishFlipperFrontAbsolute(flipper_front_position_absolute_down);
            }
        }
    }

    if (axis_flipper_front_left_right > 0 && (size_t) axis_flipper_front_left_right <= joystick->axes.size()) {
        if(fabs(joystick->axes[axis_flipper_front_left_right-1]) > 0.1){
            publishFlipperFrontAbsolute(flipper_front_position_absolute_center);
        }
    }

    if (axis_cameraPan > 0 && (size_t) axis_cameraPan <= joystick->axes.size()) {
        cameraPanSpeed = joystick->axes[axis_cameraPan-1] * cameraSpeed * M_PI/180.0;
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

void flipperFrontCallback(const std_msgs::Float64ConstPtr state_front_flipper) {
    currentStateFlipperFront = state_front_flipper->data;
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
    motionCommandOutput = n.advertise<geometry_msgs::Twist>("cmd_vel", 10, false);
    cameraCommandOutput = n.advertise<geometry_msgs::QuaternionStamped>("camera/command", 10, false);

    flipperCmdAbsPub_ = n.advertise<std_msgs::Float64>("/flipper_control/command/absolute", 10, false);
    flipperCmdRelPub_ = n.advertise<std_msgs::Float64>("/flipper_control/command/relative", 10, false);

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

    ros::param::param("~button_flipper_front_down", JoyTeleop::button_flipper_front_down, 5);
    ros::param::param("~button_flipper_front_up", JoyTeleop::button_flipper_front_up, 7);
    ros::param::param("~axis_flipper_front_up_down", JoyTeleop::axis_flipper_front_up_down, 6);
    ros::param::param("~axis_flipper_front_left_right", JoyTeleop::axis_flipper_front_left_right, 5);
    ros::param::param("~flipper_sensitivity", JoyTeleop::flipper_sensitivity, 0.2);
    ros::param::param("~flipper_front_position_absolute_up", JoyTeleop::flipper_front_position_absolute_up, 1.0);
    ros::param::param("~flipper_front_position_absolute_down", JoyTeleop::flipper_front_position_absolute_down, -1.0);
    ros::param::param("~flipper_front_position_absolute_center", JoyTeleop::flipper_front_position_absolute_center, 0.0);

    flipper_front_down = false;
    flipper_front_up = false;

    ros::Rate rate(25.0);
    while(ros::ok()) {
        if (motionCommand.linear.x != 0.0 ||
                motionCommand.angular.z != 0.0)
            motionCommandOutput.publish(motionCommand);
        moveCamera(rate.expectedCycleTime().toSec());

        if(flipper_front_stop){
          publishFlipperFrontRelative(0);
          flipper_front_stop = false;
        } else if(flipper_front_up){
            publishFlipperFrontRelative(1.0*flipper_sensitivity);
        }else if(flipper_front_down){
            publishFlipperFrontRelative(-1.0*flipper_sensitivity);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
