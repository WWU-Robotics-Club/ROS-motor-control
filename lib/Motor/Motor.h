#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_

#include <Arduino.h>
// https://playground.arduino.cc/Code/PIDLibrary/
#include <PID_v1.h>

//#define USE_QUADENCODER

#if defined(USE_QUADENCODER) && defined(__IMXRT1062__) // Teensy 4.0
  // https://github.com/mjs513/Teensy-4.x-Quad-Encoder-Library
  #include "Quadencoder.h"
  #define Encoder QuadEncoder
  #define USING_QUADENCODER
#else // Teensy 3.2, etc*/
  // https://www.pjrc.com/teensy/td_libs_Encoder.html
  #include <Encoder.h>
#endif

//#define MOTOR_DEBUG

/*
 * Motor driver class
 * See https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all for motor driver info
 */

class Motor {
 public:
  Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, int8_t encoderChannel = -1, int feedbackDir = DIRECT, int direction = DIRECT,
  double vkp=65, double vki=50, double vkd=0,
  double pkp=8, double pki=0, double pkd=0);
  ~Motor();
  void init();
  void setStandby(bool standby);
  void setSampleTimeMs(uint16_t ms);
  void setCountsPerRev(uint16_t counts);
  void setOutputLimit(int16_t limit);  // set the max analogWrite value
  void setPositionSpeed(double limit);  // set max target rad/s used by setPosition
  void setPidEnabled(bool enable);
  void setVelTunings(double kp, double ki, double kd);
  void setPosTunings(double kp, double ki, double kd);
  double getPosition();  // get position in rad
  void setPosition(double pos); // set position in rad
  double getVelocity();  // get velocity in rad/s
  void setVelocity(double vel);  //  set velocity in rad/s
  void setAcceleration(double accel);  //  set acceleration in rad/s^2
  void update();  //  update the pids and output

 private:
  void write(int16_t power);
  void updateAcceleration();  //  call when changing velocity setpoint
  // Motor driver pin numbers
  const uint8_t IN1, IN2, PWM, STBY;
  // Encoder pin numbers
  Encoder* encoder;
  const uint8_t A, B;
  const int8_t encoderChannel;

  double lastPos = 0.0;  // position in rad
  double lastVel = 0.0;  // velocity in rad/s
  int32_t lastTime = 0;  // time in millis
  int32_t lastPrintTime = 0;  // time in millis

  // PID
  enum ControlModes { VEL_CONTROL, POS_CONTROL };
  int controlMode = POS_CONTROL;
  PID* velPid;
  int feedbackDir;  // 0 for normal feedback, 1 to reverse feedback
  double velTargetSetpoint = 0;  // target velocity in rad/s
  double velSetpoint = 0;  // current velocity setpoint
  double velInput = 0;  // current velocity in rad/s
  double velOutput;  // output level from -255 to 255
  PID* posPid;
  double posSetpoint = 0;  // target position in rad
  double posInput = 0;  // position in rad
  // position output is the velocity setpoint

  // Config
  uint16_t sampleTimeMs = 20;  // PID sample time in ms
  const double countsPerRev = 1440.0;  // encoder counts per revolution
  const double radsPerCount = 2.0 * 3.14159 / countsPerRev;
  int16_t outputLimit = 160;
  double posMoveVelocity = 25.0;  // rad/s
  double accel = 0.6;  // target accelleration in rad/s/sample_time
  int direction;  // 0 for normal, 1 to reverse direction
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
