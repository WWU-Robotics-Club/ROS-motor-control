#include "MecanumController.h"

MecanumController::MecanumController(Motor* motors, float baseWidth, float baseLength, float wheelRadius)
: baseWidth(baseWidth), baseLength(baseLength), wheelRadius(wheelRadius) {
  this->motors = motors;
}

void MecanumController::init() {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].init();
  }
}

void MecanumController::update() {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
  }
}

// speed is the max speed for a motor
void MecanumController::move(const Pose2D &relativePos, double speed) {
  speed = min(speed, SPEED_LIMIT);
  double* wheelRots = getWheelRotations();
  double* relativeWheelRots = poseToWheels(relativePos);
  // find largest rotation a motor will have to make. For scaling speeds
  double maxRot = 0.0;
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    if (relativeWheelRots[i] > maxRot) {
      maxRot = relativeWheelRots[i];
    }
  }
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setPosition(wheelRots[i] + relativeWheelRots[i]);
    // scale speeds so the wheels reach their position at the same time
    motors[i].setPositionSpeed(speed * relativeWheelRots[i] / maxRot);
  }
}

double* MecanumController::getWheelRotations() {
  double positions[4];
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    positions[i] = motors[i].getPosition();
  }
  return positions;
}

void MecanumController::setStandby(bool standby) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setStandby(standby);
  }
}

void MecanumController::setVelocity(const Pose2D &vel) {
  double* speeds = poseToWheels(vel);
  // scale speeds so they don't go over a limit
  double highestSpeed = 0.0;
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    if (speeds[i] > highestSpeed) {
      highestSpeed = speeds[i];
    }
  }
  // this is just 1 if everything is below the limit
  double scalar = min(highestSpeed, SPEED_LIMIT) / highestSpeed;
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    motors[i].setVelocity(speeds[i] * scalar);
    //Serial.print(speeds[i]);
    //Serial.print(",");
  }
}

Pose2D MecanumController::getVelocity() {
  double speeds[4];
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    speeds[i] = motors[i].getVelocity();
  }
  return wheelsToPose(speeds);
}

void MecanumController::setAcceleration(double accel) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setAcceleration(accel);
  }
}

void MecanumController::setOutputLimit(int16_t limit) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setOutputLimit(limit);
  }
}

double* MecanumController::poseToWheels(const Pose2D &pose) {
  double xComp[4] = {pose.x, pose.x, pose.x, pose.x};
  double yComp[4] = {-pose.y, pose.y, pose.y, -pose.y};
  double rotComp[4] = {pose.theta, pose.theta, -pose.theta, -pose.theta};
  //double scalar = SPEED_LIMIT/max(pose.x + pose.y + pose.theta, SPEED_LIMIT);
  double wheels[NUM_MOTORS];
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    wheels[i] = (xComp[i] + yComp[i] + rotComp[i]) / wheelRadius;// * scalar;
  }
  return wheels;
}

Pose2D MecanumController::wheelsToPose(const double* wheels) {
  Pose2D pose;
  // difference of front and back should be the horizontal
  pose.x = wheels[W_FL] - wheels[W_BL];
  // both sides should match but average just in case
  pose.x = (pose.x + wheels[W_BR] - wheels[W_FR]) * 0.5;
  // get median of left and right pair to cancel out horizontal component
  double left = (wheels[W_FL] + wheels[W_BL]) * 0.5;
  double right = (wheels[W_FR] + wheels[W_BR]) * 0.5;
  // rotation is the difference over radius of turn
  pose.theta = (right - left) / (baseWidth + baseLength);
  // median of left and right to cancel out rotation
  pose.y = (left + right) * 0.5;
  // convert to meters or meters/s. 
  // this could be done at the start by multiplying each wheel by wheelRadius
  pose.x *= wheelRadius;
  pose.y *= wheelRadius;
  pose.theta *= wheelRadius;
  return pose;
}