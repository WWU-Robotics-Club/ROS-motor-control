#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_

#include  <Arduino.h>
#include "MecanumController.h"
#include "Motor.h"

#define NUM_MOTORS 4
#define SPEED_LIMIT 1.5

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

All units in m, m/s, etc
*/
class MecanumController {
 public:
  MecanumController(Motor* motors, float baseWidth, float baseLength, float wheelRadius);
  void init();
  void update();
  void move(const Pose2D &relativePos, double speed); // set position relative to current one
  void setStandby(bool standby); // turn on or off motor drivers
  double* getWheelRotations(); // get total distance travelled relative to local frame
  void setPositionSpeedLimit(double limit);  // set max target revs/s used by move
  void setVelocity(const Pose2D &vel);
  Pose2D getVelocity();
  void setAcceleration(double accel);  //  set acceleration in revs/s^2
  void setOutputLimit(int16_t limit);  // set the max analogWrite value
  enum WheelPositions {W_FR = 0, W_BR = 1, W_FL = 2, W_BL = 3};
 private:
  enum ControlModes { VEL_CONTROL, POS_CONTROL };
  Motor* motors;
  int controlMode = POS_CONTROL;
  double* poseToWheels(const Pose2D &pose); // Convert pose in m or m/s to wheel revs or revs/s
  Pose2D wheelsToPose(const double* wheels); // Convert wheel revs or revs/s to pose in m or m/s
  const double baseWidth;
  const double baseLength;
  const double wheelRadius;
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_