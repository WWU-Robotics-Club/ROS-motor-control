// https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>
// https://playground.arduino.cc/Code/PIDLibrary/
#include <PID_v1.h>


/*
 * Motor driver class
 * See https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all for motor driver info
 */

class Motor
{
public:
  Motor(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t encoderA, uint8_t encoderB, double kp=2, double ki=5, double kd=1);
  void setPidEnabled(bool enable);
  void setTunings(double kp, double ki, double kd);
  long getPosition();
  void setVelocity(double vel); // todo: define units
  void update(); // update the pids and outpute
private:
  void write(double power); // todo: pick better name?
  // Motor driver pin numbers
  const uint8_t IN1;
  const uint8_t IN2;
  const uint8_t PWM;
  const uint8_t STBY;
  // Encoder pin numbers
  Encoder* encoder;
  const uint8_t A;
  const uint8_t B;
  long lastPos = 0;
  // PID
  PID* pid;
  bool pidEnabled = true;
  double setpoint; // target velocity // todo: define units
  double input; // current velocity // todo: define units
  double output; // output level // todo: pick what this is. 0-1 or 0-255 or what
};
