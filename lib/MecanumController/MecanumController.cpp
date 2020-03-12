#include "MecanumController.h"

MecanumController::MecanumController(Motor* motors) {
  this->motors = motors;
}

void MecanumController::init() {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].init();
  }
}

void MecanumController::setOutputLimit(int16_t limit) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setOutputLimit(limit);
  }
}

void MecanumController::setPositionMoveVelocity(double limit) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setPositionMoveVelocity(limit);
  }
}

void MecanumController::setPosition(Vector2 pos) {
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    // can only go forward/backward for now
    motors[i].setPosition(pos.x);
  }
}

void MecanumController::setVelocity(Pose2D vel) {
  double xComp[4] = {vel.x, vel.x, vel.x, vel.x};
  double yComp[4] = {-vel.y, vel.y, -vel.y, vel.y};
  double rotComp[4] = {-vel.theta, -vel.theta, vel.theta, vel.theta};
  //double scalar = SPEED_LIMIT/max(vel.x + vel.y + vel.theta, SPEED_LIMIT) * 0.7;
  //float signs[4] = {1.0,-1.0,-1.0,1.0};
  float speeds[4];
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    speeds[i] = (xComp[i] + yComp[i] + rotComp[i]);// * scalar * signs[i];
    motors[i].setVelocity(speeds[i]);
    //Serial.print(speeds[i]);
    //Serial.print(",");
  }
}

void MecanumController::setAcceleration(double accel) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setAcceleration(accel);
  }
}

void MecanumController::update() {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
  }
}