#include "Motor.h"

Motor::Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t encoderA, uint8_t encoderB, double kp, double ki, double kd)
  : IN1(IN1), IN2(IN2), PWM(PWM), STBY(STBY), A(encoderA), B(encoderB)
{
  pid = new PID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
  pid->SetSampleTime(SAMPLE_TIME);
  pid->SetOutputLimits(-OUTPUT_LIMIT, OUTPUT_LIMIT);
  encoder = new Encoder(A, B);
}

void Motor::init() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  // STBY enables the driver
  digitalWrite(STBY, HIGH);
  // enable pid control
  setPidEnabled(true);
}

// Enable PID control or pass 
void Motor::setPidEnabled(bool enable)
{
  pid->SetMode(enable ? AUTOMATIC : MANUAL);
}

void Motor::setTunings(double kp, double ki, double kd)
{
  pid->SetTunings(kp, ki, kd);
}

long Motor::getPosition()
{
  return encoder->read();
}

// Update the target velocity
void Motor::setVelocity(double vel)
{
  setpoint = vel;
}

void Motor::update()
{
  long currentPos = encoder->read();
  long currentTime = millis();
  long dT = currentTime - lastTime;
  input = (double)(currentPos - lastPos)/(double)dT;
  lastPos = currentPos;
  lastTime = currentTime;
  pid->Compute();
  write(output);

  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(getPosition());
  Serial.print(",");
  Serial.print(input);
  Serial.print(",");
  Serial.println(output);
}

// Set motor power
void Motor::write(double power)
{
  // Todo: check all this. Maybe add something to handle both sides better
  // Both high when vel == 0 which makes it brake.
  // Both low would be free spin which is kinda the same on gear motors
  digitalWrite(IN1, power >= 0);
  digitalWrite(IN2, power <= 0);
  analogWrite(PWM, power); // todo:  figure out units and make any needed conversions
}
