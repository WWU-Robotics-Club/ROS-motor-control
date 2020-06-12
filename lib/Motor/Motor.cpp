#include "Motor.h"

Motor::Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, int8_t encoderChannel,
  int feedbackDir, int direction,
  double vkp, double vki, double vkd,
  double pkp, double pki, double pkd)
  : IN1(IN1), IN2(IN2), PWM(PWM), STBY(STBY),
    A(encoderA), B(encoderB), encoderChannel(encoderChannel),
    feedbackDir(feedbackDir), direction(direction) {
  velPid = new PID(&velInput, &velOutput, &velSetpoint,
    vkp, vki, vkd, P_ON_E, feedbackDir);
  setOutputLimit(outputLimit);
  // position output controls velocity target setpoint
  posPid = new PID(&posInput, &velTargetSetpoint, &posSetpoint,
    pkp, pki, pkd, P_ON_E, DIRECT);
  setPositionSpeed(posMoveVelocity);
  setSampleTimeMs(sampleTimeMs);
}

void Motor::init() {
#ifdef USING_QUADENCODER
    encoder = new Encoder(encoderChannel, A, B);
    encoder->setInitConfig();
    encoder->init();
#else
    encoder = new Encoder(A, B);
#endif
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  // STBY enables the driver
  digitalWrite(STBY, HIGH);

  setPidEnabled(true);
}

void Motor::setStandby(bool standby) {
  setPidEnabled(!standby);  // disable PID if in standby
  digitalWrite(STBY, !standby);  // disable driver if in standby
}

void Motor::setSampleTimeMs(uint16_t ms) {
  sampleTimeMs = ms;
  velPid->SetSampleTime(sampleTimeMs);
  posPid->SetSampleTime(sampleTimeMs);
}

void Motor::setOutputLimit(int16_t limit) {
  outputLimit = limit;
  velPid->SetOutputLimits(-outputLimit, outputLimit);
}

void Motor::setPositionSpeed(double limit) {
  posMoveVelocity = abs(limit);
  posPid->SetOutputLimits(-posMoveVelocity, posMoveVelocity);
}

// Enable PID control or pass
void Motor::setPidEnabled(bool enable) {
  velPid->SetMode(enable ? AUTOMATIC : MANUAL);
  posPid->SetMode(enable ? AUTOMATIC : MANUAL);
}

void Motor::setVelTunings(double kp, double ki, double kd) {
  velPid->SetTunings(kp, ki, kd);
}

void Motor::setPosTunings(double kp, double ki, double kd) {
  posPid->SetTunings(kp, ki, kd);
}

double Motor::getPosition() {
  if (direction == REVERSE) {
    return -static_cast<double>(encoder->read()) * radsPerCount;
  } else {
    return static_cast<double>(encoder->read()) * radsPerCount;
  }
}

// set desired position in radians
void Motor::setPosition(double pos) {
  posSetpoint = pos;
  controlMode = POS_CONTROL;
}

double Motor::getVelocity() {
  return lastVel;
}

// Update the target velocity
void Motor::setVelocity(double vel) {
  controlMode = VEL_CONTROL;
  velTargetSetpoint = vel;
  updateAcceleration();
}

void Motor::setAcceleration(double acc) {
  // convert rads/s^2 to rads/s/SAMPLE_TIME
  accel = acc * sampleTimeMs * 0.001;
  updateAcceleration();
}

void Motor::updateAcceleration() {
  if (velSetpoint < velTargetSetpoint) {
    accel = abs(accel);
  } else {
    accel = -abs(accel);
  }
}

void Motor::update() {
  double currentPos = getPosition();
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
    // just turn on brakes if we want to stop and velocity is low.
    // prevents wasting power at idle
    if (velSetpoint == 0.0 && abs(lastVel) < 0.5) {
      velOutput = 0.0;
    }
    write(velOutput);
    // debug
    if (currentTime - lastPrintTime > 200) {
      lastPrintTime = currentTime;

#if defined(MOTOR_DEBUG)
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
#endif
    }
  }
}

// Set motor power from -OUTPUT_LIMIT to OUTPUT_LIMIT
void Motor::write(int16_t power) {
  if (direction == REVERSE) {
    power = -power;
  }
  // Both high when vel == 0 which makes it brake.
  // Both low would be free spin which is kinda the same on gear motors
  digitalWrite(IN1, power >= 0);
  digitalWrite(IN2, power <= 0);
  analogWrite(PWM, min(abs(power), outputLimit));
}

// probably never going to be used
Motor::~Motor() {
  delete velPid;
  delete posPid;
  delete encoder;
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(PWM, INPUT);
  pinMode(STBY, INPUT);
}
