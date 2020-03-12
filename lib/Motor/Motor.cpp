#include "Motor.h"

Motor::Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, int feedbackDir, int direction,
  double vkp, double vki, double vkd,
  double pkp, double pki, double pkd)
  : IN1(IN1), IN2(IN2), PWM(PWM), STBY(STBY),
    A(encoderA), B(encoderB), feedbackDir(feedbackDir), direction(direction)
{
  encoder = new Encoder(A, B);
  velPid = new PID(&velInput, &velOutput, &velSetpoint, vkp, vki, vkd, P_ON_E, feedbackDir);
  setOutputLimit(outputLimit);
  // position output controls velocity setpoint
  posPid = new PID(&posInput, &velSetpoint, &posSetpoint, pkp, pki, pkd, P_ON_E, DIRECT);
  setPositionMoveVelocity(posMoveVelocity);
  setSampleTimeMs(sampleTimeMs);
}

void Motor::init() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  // STBY enables the driver
  digitalWrite(STBY, HIGH);
  encoder = new Encoder(A, B);
  setPidEnabled(true);
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

void Motor::setPositionMoveVelocity(double limit) {
  posMoveVelocity = limit;
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
  return (double)encoder->read() / countsPerRev;
}

// set desired position in revolutions
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
  // invert if this motor is reversed
  if (direction == REVERSE) {
    velTargetSetpoint = -vel;
  } else {
    velTargetSetpoint = vel;
  }
  updateAcceleration();
}

void Motor::setAcceleration(double acc) {
  // convert revs/s^2 to revs/s/SAMPLE_TIME
  accel = acc * 0.001 * sampleTimeMs;
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
  double dT = (currentTime - lastTime) / 1000.0; // elapsed time in seconds
  if (controlMode == POS_CONTROL) {
    posInput = currentPos;
    // Compute() sets velSetpoint
    if(posPid->Compute()) {
      // invert if this motor is reversed
      if (direction == REVERSE) {
        velSetpoint = -velSetpoint;
      }
      updateAcceleration();
    }
  }
  velInput = (currentPos - lastPos)/dT; // revs/s
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
    write(velOutput);
    // debug
    if (currentTime - lastPrintTime > 200) {
      lastPrintTime = currentTime;
      
      //Serial.print(acc >  ? "R," : "F,");
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
  }
}

// Set motor power from -OUTPUT_LIMIT to OUTPUT_LIMIT
void Motor::write(int16_t power) {
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