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
  speed = min(speed, speedLimit);
  double wheelRots[NUM_MOTORS];
  getWheelRotations(wheelRots);
#ifdef MEC_DEBUG
  Serial.print("current rads: ");
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(wheelRots[i]);
    Serial.print("  ");
  }
  Serial.println("");
#endif
  double relativeWheelRots[NUM_MOTORS];
  poseToWheels(relativePos, relativeWheelRots);
  // find largest rotation a motor will have to make. For scaling speeds
  double maxRot = 0.0;
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    if (relativeWheelRots[i] > maxRot) {
      maxRot = relativeWheelRots[i];
    }
  }
#ifdef MEC_DEBUG
  Serial.print("new rads: ");
#endif
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setPosition(wheelRots[i] + relativeWheelRots[i]);
#ifdef MEC_DEBUG
    Serial.print(wheelRots[i] + relativeWheelRots[i]);
    Serial.print("  ");
#endif
    // scale speeds so the wheels reach their position at the same time
    motors[i].setPositionSpeed(speed * relativeWheelRots[i] / maxRot);
  }
#ifdef MEC_DEBUG
    Serial.println("");
#endif
}

void MecanumController::getWheelRotations(double* rotations) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    rotations[i] = motors[i].getPosition();
  }
}

Pose2D MecanumController::getPosition() {
  double rotations[NUM_MOTORS];
  getWheelRotations(rotations);
  return wheelsToPose(rotations);
}

void MecanumController::setStandby(bool standby) {
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setStandby(standby);
  }
}

void MecanumController::setVelocity(const Pose2D &vel) {
  double speeds[NUM_MOTORS];
  poseToWheels(vel, speeds);
#ifdef MEC_DEBUG
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(speeds[i]);
    Serial.print("  ");
  }
  Serial.println("");
#endif
  // scale speeds so they don't go over a limit
  double highestSpeed = 0.0;
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    if (speeds[i] > highestSpeed) {
      highestSpeed = speeds[i];
    }
  }
  double scalar = 1.0;
  if (highestSpeed > speedLimit) {
    scalar = speedLimit / highestSpeed;
  }
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    motors[i].setVelocity(speeds[i] * scalar);
#ifdef MEC_DEBUG
    Serial.print(speeds[i] * scalar);
    Serial.print("  ");
#endif
  }
#ifdef MEC_DEBUG
  Serial.println();
#endif
}

Pose2D MecanumController::getVelocity() {
  double speeds[NUM_MOTORS];
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

void MecanumController::setSpeedLimit(double limit) {
  speedLimit = limit;
}

void MecanumController::poseToWheels(const Pose2D &pose, double* wheels) {
  double xComp[4] = {pose.x, pose.x, pose.x, pose.x};
  double yComp[4] = {pose.y, -pose.y, -pose.y, pose.y};
  double thetaAdjusted = pose.theta * (baseWidth + baseLength);
  double rotComp[4] = {thetaAdjusted, thetaAdjusted, -thetaAdjusted, -thetaAdjusted};
  //double scalar = speedLimit/max(pose.x + pose.y + pose.theta, speedLimit);
  for (unsigned int i=0; i<NUM_MOTORS; i++) {
    wheels[i] = (xComp[i] + yComp[i] + rotComp[i]) / wheelRadius;// * scalar;
  }
}

Pose2D MecanumController::wheelsToPose(const double* wheels) {
  Pose2D pose;
  // difference of front and back should be the horizontal
  pose.y = wheels[W_BL] - wheels[W_FL];
  // both sides should match but average just in case
  pose.y = (pose.y + wheels[W_FR] - wheels[W_BR]) * 0.5;
  // get median of left and right pair to cancel out horizontal component
  double left = (wheels[W_FL] + wheels[W_BL]) * 0.5;
  double right = (wheels[W_FR] + wheels[W_BR]) * 0.5;
  // rotation in radians is the difference over radius of turn
  pose.theta = (right - left) / (baseWidth + baseLength);
  // median of left and right to cancel out rotation
  pose.x = (left + right) * 0.5;
  // convert to meters or meters/s. 
  // this could be done at the start by multiplying each wheel by wheelRadius
  pose.x *= wheelRadius;
  pose.y *= wheelRadius;
  pose.theta *= wheelRadius;
  return pose;
}