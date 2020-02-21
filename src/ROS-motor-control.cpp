// Written for Teensy 3.2
// Pinout: https://www.pjrc.com/teensy/card7a_rev1.png

#include "Motor.h"
#include <PulsePosition.h>

#define NUM_MOTORS 4

char serialData[32];

// share STBY pin
// 3rd pin needs to be PWM capable https://www.pjrc.com/teensy/td_pulse.html
// the last 2 need to have hardware interrupts
Motor motors[NUM_MOTORS] = {
  Motor(3, 4, 5, 2, 6, 7),
  Motor(8, 9, 10, 2, 11, 12),
  Motor(14, 15, 20, 2, 16, 17, true),
  Motor(18, 19, 21, 2, 22, 23)
};

PulsePositionInput ppmIn(FALLING);
int count = 0;

void setup() {
  // pins 5, 10, 20, 21 share the same timer. Set their PWM frequency to something inaudible
  analogWriteFrequency(5, 36000);

  //PPMReaderSetup(6, 8); // pin, num channels
  ppmIn.begin(6);

  // Initialize all the motors
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].init();
  }
}

void loop() {
  if (ppmIn.available() >= 8) {
    float x = constrain(ppmIn.read(2) - 1500, -500, 500);
    float y = constrain(ppmIn.read(4) - 1500, -500, 500);
    float rot = constrain(ppmIn.read(1) - 1500, -500, 500);
    Serial.println(String(x) + "," + String(y) + "," + String(rot));
    // Motors 1,2,3,4 arranged like
    // 3  1
    //
    // 4  2
    // 1 negative is forward
    // 2 positive is forward
    // 3 positive is forward
    // 4 negative is forward
    float xComp[4] = {x,x,x,x};
    float yComp[4] = {-y,y,y,-y};
    float rotComp[4] = {-rot, -rot, rot, rot};
    float scalar = 255.0/max(x+y+rot,255) * 0.7;
    float signs[4] = {1.0,-1.0,-1.0,1.0};
    float speeds[4];
    for (unsigned int i=0; i<4; i++) {
      speeds[i] = (xComp[i] + yComp[i] + rotComp[i]) * scalar * signs[i];
      motors[i].write(speeds[i]);
      Serial.print(speeds[i]);
      Serial.print(",");
    }
    Serial.println();
  }

  /*
    Serial.println();
    if(Serial.available() > 0) {
    Serial.readBytesUntil('\n', serialData, 31);
    parseCommand(serialData);
    }

    for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    motors[i].update();
    }*/
  delay(20);
}

void parseCommand(char* command) {
  int numbers[4];
  char* strtokIndx = strtok(command, ","); // get the first part
  numbers[0] = atoi(strtokIndx); // convert to int

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  numbers[1] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  numbers[2] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  numbers[3] = atof(strtokIndx);

  for (unsigned int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i].setVelocity(numbers[i]);
  }
}
