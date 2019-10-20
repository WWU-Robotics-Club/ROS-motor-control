#include "Motor.h"

#define NUM_MOTORS 4

// Pinout: https://www.pjrc.com/teensy/card7a_rev1.png

void setup() {
  Motor motors[NUM_MOTORS] = {
    Motor(3, 4, 2, 5, 6, 7),
    Motor(8, 9, 2, 10, 11, 12),
    Motor(14, 15, 2, 16, 17, 18),
    Motor(19, 20, 2, 21, 22, 23)
  };
  
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
