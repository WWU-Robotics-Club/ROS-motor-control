#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_

#include <Arduino.h>
// https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>
// https://playground.arduino.cc/Code/PIDLibrary/
#include <PID_v1.h>

/*
 * Motor driver class
 * See https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all for motor driver info
 */

class Motor {
 public:
  Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, int feedbackDir = DIRECT, int direction = DIRECT,
  double vkp=200, double vki=50, double vkd=0,
  double pkp=15, double pki=0, double pkd=0);
  ~Motor();
  void init();
  void setSampleTimeMs(uint16_t ms);
  void setCountsPerRev(uint16_t counts);
  void setOutputLimit(int16_t limit);  // set the max analogWrite value
  void setPositionMoveVelocity(double limit);  // set max target revs/s used by setPosition
  void setPidEnabled(bool enable);
  void setVelTunings(double kp, double ki, double kd);
  void setPosTunings(double kp, double ki, double kd);
  double getPosition();  // get position in revs
  void setPosition(double pos); // set position in revs
  double getVelocity();  // get velocity in revs/s
  void setVelocity(double vel);  //  set velocity in revs/s
  void setAcceleration(double accel);  //  set acceleration in revs/s^2
  void update();  //  update the pids and output

 private:
  void write(int16_t power);
  // Motor driver pin numbers
  const uint8_t IN1, IN2, PWM, STBY;
  // Encoder pin numbers
  Encoder* encoder;
  const uint8_t A, B;

  double lastPos = 0.0;  // position in revs
  double lastVel = 0.0;  // velocity in revs/s
  int32_t lastTime = 0;  // time in millis
  int32_t lastPrintTime = 0;  // time in millis

  // PID
  enum ControlModes { VEL_CONTROL, POS_CONTROL };
  int controlMode = POS_CONTROL;
  PID* velPid;
  int feedbackDir;  // 0 for normal feedback, 1 to reverse feedback
  double velTargetSetpoint = 0;  // target velocity in revs/s
  double velSetpoint = 0;  // current velocity setpoint
  double velInput = 0;  // current velocity in revs/s
  double velOutput;  // output level from -255 to 255
  PID* posPid;
  double posSetpoint = 0;  // target position in revs
  double posInput = 0;  // position in revs
  // position output is the velocity setpoint

  // Config
  uint16_t sampleTimeMs = 20;  // PID sample time in ms
  uint16_t countsPerRev = 1440;  // encoder counts per revolution
  int16_t outputLimit = 160;
  double posMoveVelocity = 4.0;  // revs/s
  double accel = 0.1;  // target accelleration in revs/s/sample_time
  void updateAcceleration();  //  call when changing velocity setpoint
  int direction;  // 0 for normal, 1 to reverse direction
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
