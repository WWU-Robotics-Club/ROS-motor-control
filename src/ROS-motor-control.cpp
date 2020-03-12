// Written for Teensy 3.2
// Pinout: https://www.pjrc.com/teensy/card7a_rev1.png#include <ros.h>

#include "Motor.h"
#include "MecanumController.h"

#define NUM_MOTORS 4

char serialData[32];

// share STBY pin
// 3rd pin needs to be PWM capable https://www.pjrc.com/teensy/td_pulse.html
// the last 2 need to have hardware interrupts
Motor motors[NUM_MOTORS] = {
  Motor(3, 4, 5, 2, 6, 7),
  Motor(8, 9, 10, 2, 11, 12),
  Motor(14, 15, 20, 2, 16, 17),
  Motor(18, 19, 21, 2, 22, 23, REVERSE)
};

//MecanumController mecControl(&motors);

void parseCommand(char* command) {
  /*int numbers[4];
  char* strtokIndx = strtok(command, ","); // get the first part
  numbers[0] = atof(strtokIndx); // convert to float
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  numbers[1] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  numbers[2] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  numbers[3] = atof(strtokIndx);

  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setVelocity(numbers[i]);
  }*/
  //motors[3].setVelocity(atof(command));
  motors[3].setPosition(atof(command));
}

void setup() {  
  // pins 5, 10, 20, 21 share the same timer. Set their PWM frequency to something inaudible
  analogWriteFrequency(5, 36000);
  
  //mecControl.init();

  // Initialize all the motors
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].init();
    motors[i].setAcceleration(3.0);
  }
}

void loop() {
  if(Serial.available() > 0) {
    Serial.readBytesUntil('\n', serialData, 31);
    parseCommand(serialData);
  }

  for (unsigned int i = 3; i < NUM_MOTORS; i++) { // todo change back to 0
    motors[i].update();
  }
  //delay(21);
}
