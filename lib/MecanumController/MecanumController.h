#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_

#include  <Arduino.h>
#include "MecanumController.h"
#include "Motor.h"

#define NUM_MOTORS 4

struct Vector2 {
  double x;
  double y;
};

struct Pose2D {
  double x;
  double y;
  double theta;
};

class MecanumController {
 public:
  MecanumController(Motor* motors);
  void init();
  void setOutputLimit(int16_t limit);  // set the max analogWrite value
  void setPositionMoveVelocity(double limit);  // set max target revs/s used by setPosition
  void setVelocity(Pose2D vel);
  void setAcceleration(double accel);  //  set acceleration in revs/s^2
  void update();
  //Pose2D getVelocity();
  void setPosition(Vector2 pos);
  //Vector2 getPosition();
  //void setPositionRelative(Vector2 pos);
  //void setRotation(double rot);
  //void setRotationRelative(double rot);
 private:
  enum ControlModes { VEL_CONTROL, POS_CONTROL };
  Motor* motors;
  int controlMode = POS_CONTROL;
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MECANUMCONTROLLER_MECANUMCONTROLLER_H_