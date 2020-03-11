#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_

// https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>
// https://playground.arduino.cc/Code/PIDLibrary/
#include <PID_v1.h>

// PID sample time in ms
#define SAMPLE_TIME 20
#define OUTPUT_LIMIT 255

/*
 * Motor driver class
 * See https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all for motor driver info
 */

class Motor {
public:
  Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, bool reverse=0,
  double kp=200, double ki=50, double kd=0);
  void init();
  void setPidEnabled(bool enable);
  void setTunings(double kp, double ki, double kd);
  int32_t getPosition();
  void setVelocity(double vel);  // todo: define units
  void update();  // update the pids and outpute

private:
  void write(double power);  // todo: pick better name?
  // Motor driver pin numbers
  const uint8_t IN1, IN2, PWM, STBY;
  // Encoder pin numbers
  Encoder* encoder;
  const uint8_t A, B;
  int32_t lastPos = 0;
  int32_t lastTime = 0;
  // PID
  PID* pid;
  bool reverse;
  double setpoint = 0; // target velocity // todo: define units
  double input = 0; // current velocity // todo: define units
  double output; // output level // todo: pick what this is. 0-1 or 0-255 or what
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
