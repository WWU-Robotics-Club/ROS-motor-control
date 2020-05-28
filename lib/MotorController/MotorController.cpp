#include "MotorController.h"

MotorController::MotorController(int feedbackDir, int direction,
  double vkp, double vki, double vkd,
  double pkp, double pki, double pkd)
  : feedbackDir(feedbackDir), direction(direction) {
  velPid = new PID(&velInput, &velOutput, &velSetpoint, vkp, vki, vkd,
    P_ON_E, feedbackDir);
  setOutputLimit(outputLimit);
  // position output controls velocity target setpoint
  posPid = new PID(&posInput, &velTargetSetpoint, &posSetpoint, pkp, pki, pkd,
    P_ON_E, DIRECT);
  setPositionSpeed(posMoveVelocity);
  setSampleTimeMs(sampleTimeMs);
}

void MotorController::init() {
  initHardware();
  setStandby(false);
}

void MotorController::setStandby(bool standby) {
  setStandbyHardware(standby);
  setPidEnabled(!standby);  // disable PID if in standby
}

void MotorController::setSampleTimeMs(uint16_t ms) {
  sampleTimeMs = ms;
  velPid->SetSampleTime(sampleTimeMs);
  posPid->SetSampleTime(sampleTimeMs);
}

void MotorController::setOutputLimit(int16_t limit) {
  outputLimit = limit;
  velPid->SetOutputLimits(-outputLimit, outputLimit);
}

void MotorController::setPositionSpeed(double limit) {
  posMoveVelocity = abs(limit);
  posPid->SetOutputLimits(-posMoveVelocity, posMoveVelocity);
}

// Enable PID control or pass
void MotorController::setPidEnabled(bool enable) {
  velPid->SetMode(enable ? AUTOMATIC : MANUAL);
  posPid->SetMode(enable ? AUTOMATIC : MANUAL);
}

void MotorController::setVelTunings(double kp, double ki, double kd) {
  velPid->SetTunings(kp, ki, kd);
}

void MotorController::setPosTunings(double kp, double ki, double kd) {
  posPid->SetTunings(kp, ki, kd);
}

// set desired position in radians
void MotorController::setPosition(double pos) {
  posSetpoint = pos;
  controlMode = POS_CONTROL;
}

double MotorController::getVelocity() {
  return lastVel;
}

// Update the target velocity
void MotorController::setVelocity(double vel) {
  controlMode = VEL_CONTROL;
  velTargetSetpoint = vel;
  updateAcceleration();
}

void MotorController::setAcceleration(double acc) {
  // convert rads/s^2 to rads/s/SAMPLE_TIME
  accel = acc * sampleTimeMs * 0.001;
  updateAcceleration();
}

void MotorController::updateAcceleration() {
  if (velSetpoint < velTargetSetpoint) {
    accel = abs(accel);
  } else {
    accel = -abs(accel);
  }
}

void MotorController::update() {
  double currentPos = getPosition();
  if (direction == REVERSE) {
    currentPos = -currentPos;
  }
  int32_t currentTime = millis();
  double dT = (currentTime - lastTime) / 1000.0;  // elapsed time in seconds
  if (controlMode == POS_CONTROL) {
    posInput = currentPos;
    // Compute() sets velTargetSetpoint
    if (posPid->Compute()) {
      updateAcceleration();
    }
  }
  velInput = (currentPos - lastPos)/dT;  // rads/s
  if (velPid->Compute()) {
    // handle acceleration
    if (velSetpoint != velTargetSetpoint) {
      // add acceleration if not at target yet
      velSetpoint = velSetpoint + accel;
      // don't allow overshoot
      if ((accel > 0.0 && velSetpoint - velTargetSetpoint > 0.0)
        || (accel < 0.0 && velSetpoint - velTargetSetpoint < 0.0)) {
        velSetpoint = velTargetSetpoint;
      }
    }

    lastPos = currentPos;
    lastVel = velInput;
    lastTime = currentTime;
    // if error is 0, the integral never goes to 0 even if output is zero
    // prevents wasting power
    if (velSetpoint == 0.0 && lastVel == 0.0) {
      velOutput = 0.0;
    }
    if (direction == REVERSE) {
      write(-velOutput);
    } else {
      write(velOutput);
    }

#if defined(MOTOR_DEBUG)
    if (currentTime - lastPrintTime > 200) {
      lastPrintTime = currentTime;
      // Serial.print(acc >  ? "R," : "F,");
      Serial.print(posSetpoint);
      Serial.print(",");
      Serial.print(currentPos);
      Serial.print(",");
      Serial.print(velInput);
      Serial.print(",");
      Serial.print(velOutput);
      Serial.print(",");
      Serial.print(velSetpoint);
      Serial.print(",");
      Serial.print(velTargetSetpoint);
      Serial.print(",");
      Serial.println(accel);
    }
#endif
  }
}

// probably never going to be used
MotorController::~MotorController() {
  delete velPid;
  delete posPid;
}