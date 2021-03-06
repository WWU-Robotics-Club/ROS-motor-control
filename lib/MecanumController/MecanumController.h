#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_

#include  <Arduino.h>
#include "MecanumController.h"
#include "Motor.h"

#define NUM_MOTORS 4

// #define MEC_DEBUG

struct Pose2D {
  double x;
  double y;
  double theta;
};

/*
top view motor arrangement as they are numbered on the PCB:
 front
\\ 3   1 //


// 4   2 \\

All units in m, m/s, radians, radians/s, etc
x = forward,
y = left
z = up
theta is counter-clockwise around z (right hand rule)

Some kinematics based on https://www.hindawi.com/journals/jr/2018/9373580/
*/
class MecanumController {
 public:
  MecanumController(Motor* motors, float baseWidth, float baseLength,
    float wheelRadius);
  void init();
  void update();
  // set position relative to current one
  void move(const Pose2D &relativePos, double speed);
  // turn on or off motor drivers
  void setStandby(bool standby);
  // get total distance travelled relative to local frame in revs
  void getWheelRotations(double* rotations);
  // get total distance travelled relative to local frame.
  // So driving in an arc will only change theta and x
  Pose2D getPosition();
  // set x,y velocity in m/s and rotation rad/s
  void setVelocity(const Pose2D &vel);
  Pose2D getVelocity();
  void setAcceleration(double accel);  //  set acceleration in rad/s^2
  void setOutputLimit(int16_t limit);  // set the max analogWrite value
  void setSpeedLimit(double limit);  // set the max target m/s
  enum WheelPositions {W_FR = 0, W_BR = 1, W_FL = 2, W_BL = 3};

 private:
  enum ControlModes { VEL_CONTROL, POS_CONTROL };
  Motor* motors;
  int controlMode = POS_CONTROL;
  // Convert pose in m or m/s to wheel revs or revs/s
  void poseToWheels(const Pose2D &pose, double* wheels);
  // Convert wheel revs or revs/s to pose in m or m/s
  Pose2D wheelsToPose(const double* wheels);
  const double baseWidth;
  const double baseLength;
  const double wheelRadius;
  double speedLimitMps = 1.5;  // m/s
  double speedLimitRadps;  // rads/s
  int32_t lastPrintTime = 0;
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_
