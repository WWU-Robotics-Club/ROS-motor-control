#include "Motor.h"

#define NUM_MOTORS 4

// Written for Teensy 3.2
// Pinout: https://www.pjrc.com/teensy/card7a_rev1.png

void setup() {
  // share STBY pin
  // 3rd pin needs to be PWM capable https://www.pjrc.com/teensy/td_pulse.html
  // the last 2 need to have hardware interrupts
  Motor motors[NUM_MOTORS] = {
    Motor(3, 4, 5, 2, 6, 7),
    Motor(8, 9, 10, 2, 11, 12),
    Motor(14, 15, 20, 2, 16, 17),
    Motor(18, 19, 21, 2, 22, 23)
  };
  // pins 5, 10, 20, 21 share the same timer. Set their PWM frequency to something inaudible
  analogWriteFrequency(5, 36000);
  
  // Turn on all the motors
  for (unsigned int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].setPidEnabled(false);
    motors[i].setVelocity(100);
    motors[i].update();
  }
}

void loop() {
}
