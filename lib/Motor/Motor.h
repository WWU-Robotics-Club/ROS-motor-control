#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_

#include <Arduino.h>
#include "MotorController.h"

// #define USE_QUADENCODER

#if defined(USE_QUADENCODER) && defined(__IMXRT1062__)  // Teensy 4.0
  // https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library
  #include "Quadencoder.h"
  #define Encoder QuadEncoder
  #define USING_QUADENCODER
#else  // Teensy 3.2, etc*/
  // https://www.pjrc.com/teensy/td_libs_Encoder.html
  #include <Encoder.h>
#endif

/*
 * Motor driver class
 * See https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all for motor driver info
 * This provides implementations for the functions MotorController uses to interact with hardware
 */

class Motor : public MotorController {
 public:
  Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, int8_t encoderChannel = -1,
  int feedbackDir = DIRECT, int direction = DIRECT,
  double vkp = 65, double vki = 50, double vkd = 0,
  double pkp = 8, double pki = 0, double pkd = 0);
  ~Motor();
  void initHardware();
  void setStandbyHardware(bool standby);
  double getPosition();  // get position in rad
  void write(int16_t power);
  
 private:
  // Motor driver pin numbers
  const uint8_t IN1, IN2, PWM, STBY;
  Encoder* encoder;
  // Encoder pin numbers
  const uint8_t A, B;
  // Used with the QuadEncoder library
  const int8_t encoderChannel;

  // Config
  const double countsPerRev = 1440.0;  // encoder counts per revolution
  const double radsPerCount = 2.0 * 3.14159 / countsPerRev;
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
