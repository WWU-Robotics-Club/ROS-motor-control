#include "Motor.h"

Motor::Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, int8_t encoderChannel,
  int feedbackDir = DIRECT, int direction = DIRECT,
  double vkp = 65, double vki = 50, double vkd = 0,
  double pkp = 8, double pki = 0, double pkd = 0)
  : MotorController(feedbackDir, direction, vkp, vki, vkd, pkp, pki, pkd),
    IN1(IN1), IN2(IN2), PWM(PWM), STBY(STBY),
    A(encoderA), B(encoderB), encoderChannel(encoderChannel) {
}

void Motor::initHardware() {
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
  setStandby(false);
}

void Motor::setStandbyHardware(bool standby) {
  // driver is disabled if STBY is low
  digitalWrite(STBY, !standby);
}

double Motor::getPosition() {
  return static_cast<double>(encoder->read()) * radsPerCount;
}

// Set motor power
void Motor::write(int16_t power) {
  // Both high when vel == 0 which makes it brake.
  // Both low would be free spin which is kinda the same on gear motors
  digitalWrite(IN1, power >= 0);
  digitalWrite(IN2, power <= 0);
  analogWrite(PWM, abs(power));
}

// probably never going to be used
Motor::~Motor() {
  delete encoder;
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(PWM, INPUT);
  pinMode(STBY, INPUT);
}
