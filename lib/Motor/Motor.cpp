#include "Motor.h"

Motor::Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, bool reverse,
  double vkp, double vki, double vkd,
  double pkp, double pki, double pkd)
  : IN1(IN1), IN2(IN2), PWM(PWM), STBY(STBY),
    A(encoderA), B(encoderB), reverse(reverse)
{
  encoder = new Encoder(A, B);
  velPid = new PID(&velInput, &velOutput, &velSetpoint, vkp, vki, vkd, P_ON_E, reverse);
  velPid->SetSampleTime(SAMPLE_TIME);
  velPid->SetOutputLimits(-OUTPUT_LIMIT, OUTPUT_LIMIT);
  // position output controls velocity setpoint
  posPid = new PID(&posInput, &velSetpoint, &posSetpoint, pkp, pki, pkd, P_ON_E, DIRECT);
  posPid->SetSampleTime(SAMPLE_TIME);
  posPid->SetOutputLimits(-SPEED_LIMIT, SPEED_LIMIT);
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
  return (double)encoder->read() / COUNTS_PER_REV;
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
  velSetpoint = vel;
  controlMode = VEL_CONTROL;
}

void Motor::update() {
  double currentPos = getPosition();
  int32_t currentTime = millis();
  double dT = (currentTime - lastTime) / 1000.0; // elapsed time in seconds
  if (controlMode == POS_CONTROL) {
    posInput = currentPos;
    // this sets velSetpoint
    posPid->Compute();
  }
  velInput = (currentPos - lastPos)/dT; // revs/s
  if (velPid->Compute()) {
    int32_t timeSincePrint = currentTime - lastPrintTime;

    lastPos = currentPos;
    lastVel = velInput;
    lastTime = currentTime;
    // if error is 0, the integral never goes to 0 even if output is zero
    // prevents wasting power
    if (velSetpoint == 0.0 && lastVel == 0.0) {
      velOutput = 0.0;
    }
    write(velOutput);
  
    if (timeSincePrint > 200) {
      lastPrintTime = currentTime;
      
      Serial.print(reverse ? "R," : "F,");
      Serial.print(posSetpoint);
      Serial.print(",");
      Serial.print(posInput);
      Serial.print(",");
      Serial.print(velSetpoint);
      Serial.print(",");
      Serial.print(getPosition());
      Serial.print(",");
      Serial.print(velInput);
      Serial.print(",");
      Serial.println(velOutput);
    }
  }
}

// Set motor power from -OUTPUT_LIMIT to OUTPUT_LIMIT
void Motor::write(int16_t power) {
  // Both high when vel == 0 which makes it brake.
  // Both low would be free spin which is kinda the same on gear motors
  digitalWrite(IN1, power >= 0);
  digitalWrite(IN2, power <= 0);
  analogWrite(PWM, min(abs(power), OUTPUT_LIMIT));
}
