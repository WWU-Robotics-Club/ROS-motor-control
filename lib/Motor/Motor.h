#ifndef ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
#define ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_

// https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>
// https://playground.arduino.cc/Code/PIDLibrary/
#include <PID_v1.h>


#define SAMPLE_TIME 20 // PID sample time in ms
#define OUTPUT_LIMIT 160
#define SPEED_LIMIT 4.0 // revs/s
#define COUNTS_PER_REV 1440 // encoder counts per revolution

/*
 * Motor driver class
 * See https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all for motor driver info
 */

class Motor {
public:
  Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY,
  uint8_t encoderA, uint8_t encoderB, bool reverse=false,
  double vkp=200, double vki=50, double vkd=0,
  double pkp=15, double pki=0, double pkd=0);
  void init();
  void setPidEnabled(bool enable);
  void setVelTunings(double kp, double ki, double kd);
  void setPosTunings(double kp, double ki, double kd);
  double getPosition(); // get position in revs
  void setPosition(double pos); // set position in revs
  double getVelocity(); // get velocity in revs/s
  void setVelocity(double vel);  //  set velocity in revs/s
  void update();  //  update the pids and output

private:
  void write(int16_t power);
  // Motor driver pin numbers
  const uint8_t IN1, IN2, PWM, STBY;
  // Encoder pin numbers
  Encoder* encoder;
  const uint8_t A, B;

  double lastPos = 0.0; // position in revs
  double lastVel = 0.0; // velocity in revs/s
  int32_t lastTime = 0; // time in millis
  int32_t lastPrintTime = 0; // time in millis

  // PID
  enum ControlModes { VEL_CONTROL, POS_CONTROL };
  int controlMode;
  PID* velPid;
  bool reverse;
  double velSetpoint = 0;  // target velocity in revs/s
  double velInput = 0;  // current velocity in revs/s
  double velOutput;  // output level from -255 to 255
  PID* posPid;
  double posSetpoint = 0;  // target position in revs
  double posInput = 0;  // position in revs
  // position output is the velocity setpoint
};

#endif  // ARDUINO_ROS_MOTOR_CONTROL_LIB_MOTOR_MOTOR_H_
