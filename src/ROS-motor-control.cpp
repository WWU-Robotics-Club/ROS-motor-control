// Written for Teensy 3.2 and 4.0
// Pinout: https://www.pjrc.com/teensy/card7a_rev1.png

#include <Arduino.h>
#include "Motor.h"
#include "MecanumController.h"

// comment out to use serial commands
#define USE_ROS

#ifdef USE_ROS
#include "ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
// milliseconds between publishing
#define ROS_PUB_INTERVAL 50
#endif

#define NUM_MOTORS 4
#define BASE_WIDTH 0.210
#define BASE_LENGTH 0.165
#define WHEEL_RADIUS 0.059

#if defined(__IMXRT1062__)  // Teensy 4.0
  Motor motors[NUM_MOTORS] = {
    Motor(14, 15, 8, 31, 0, 1, 1, REVERSE, DIRECT),
    Motor(16, 17, 9, 31, 2, 3, 2, REVERSE, DIRECT),
    Motor(26, 27, 22, 31, 4, 5, 3, REVERSE, REVERSE),
    Motor(28, 29, 23, 31, 7, 30, 4, REVERSE, REVERSE)
  };
#elif defined(__MK20DX256__)  // Teensy 3.2 (not tested much)
  // share STBY pin
  // 3rd pin needs to be PWM capable https://www.pjrc.com/teensy/td_pulse.html
  // the last 2 need to have hardware interrupts
  Motor motors[NUM_MOTORS] = {
    Motor(3, 4, 5, 2, 6, 7, -1),
    Motor(8, 9, 10, 2, 11, 12, -1),
    Motor(14, 15, 20, 2, 16, 17, -1),
    Motor(18, 19, 21, 2, 22, 23, -1)
  };
#else
#error "Motors not defined for this platform"
#endif

MecanumController mecControl(motors, BASE_WIDTH, BASE_LENGTH, WHEEL_RADIUS);

#ifdef USE_ROS
float moveSpeed = 1.0;  // m/s
uint32_t lastPubTime = 0;  // ms

void velocityCallback(const geometry_msgs::Twist& twist) {
  Pose2D vel = {twist.linear.x, twist.linear.y, twist.angular.z};
  mecControl.setVelocity(vel);
}

void moveCallback(const geometry_msgs::Pose& pose) {
  Pose2D position = {pose.position.x, pose.position.y, pose.orientation.z};
  mecControl.move(position, moveSpeed);
}

void moveSpeedCallback(const std_msgs::Float32& speed) {
  moveSpeed = speed.data;
}

ros::NodeHandle node;
geometry_msgs::Twist velMsg;
geometry_msgs::Pose posMsg;
ros::Publisher velocityPub("/wheels/odom_velocity", &velMsg);
ros::Publisher positionPub("/wheels/odom_position", &posMsg);

ros::Subscriber<geometry_msgs::Twist> velocitySub("/wheels/cmd_velocity",
  velocityCallback);
ros::Subscriber<geometry_msgs::Pose> moveSub("/wheels/cmd_move", moveCallback);
ros::Subscriber<std_msgs::Float32> moveSpeedSub("/wheels/cmd_move_speed",
  moveSpeedCallback);

#else
char serialData[32];

void printPose(Pose2D p) {
  Serial.print("x: ");
  Serial.print(p.x);
  Serial.print(", y: ");
  Serial.print(p.y);
  Serial.print(", theta: ");
  Serial.println(p.theta);
}

void parseCommand(char* command) {
  // int numbers[3];
  Pose2D p;
  if (command[0] == 'v') {
    printPose(mecControl.getVelocity());
  } else if (command[0] == 'r') {
    double rots[NUM_MOTORS];
    mecControl.getWheelRotations(rots);
    for (unsigned int i = 0; i < NUM_MOTORS; i++) {
      Serial.print(rots[i]);
      Serial.print("  ");
    }
    Serial.println("");
  } else if (command[0] == 'p') {
    printPose(mecControl.getPosition());
  } else {
    // get the first part
    char* strtokIndx = strtok(command, ",");
    // convert to float
    p.x = atof(strtokIndx);
    // this continues where the previous call left off
    strtokIndx = strtok(NULL, ",");
    p.y = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    p.theta = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    double speed = atof(strtokIndx);
    // mecControl.setVelocity(p);
    mecControl.move(p, speed);
    printPose(p);
    Serial.print("Speed: ");
    Serial.println(speed);
  }
}
#endif

void setup() {
#ifdef USE_ROS
  node.initNode();
  node.subscribe(velocitySub);
  node.subscribe(moveSub);
  node.subscribe(moveSpeedSub);
  node.advertise(velocityPub);
  node.advertise(positionPub);
#endif
#if defined(__IMXRT1062__)  // Teensy 4.0
  // Set their PWM frequency to something inaudible
  analogWriteFrequency(8, 36000);
  analogWriteFrequency(9, 36000);
  analogWriteFrequency(22, 36000);
  analogWriteFrequency(23, 36000);
#elif defined(__MK20DX256__)  // Teensy 3.2
  // pins 5, 10, 20, 21 share the same timer.
  // Set their PWM frequency to something inaudible
  analogWriteFrequency(5, 36000);
#endif
  mecControl.init();
}

void loop() {
#ifdef USE_ROS
  // publish topics every ROS_PUB_INTERVAL milliseconds
  if (millis() - lastPubTime > ROS_PUB_INTERVAL) {
    lastPubTime = millis();
    Pose2D vel = mecControl.getVelocity();
    velMsg.linear.x = vel.x;
    velMsg.linear.y = vel.y;
    velMsg.angular.z = vel.theta;
    Pose2D pos = mecControl.getPosition();
    posMsg.position.x = pos.x;
    posMsg.position.y = pos.y;
    posMsg.orientation.z = pos.theta;
    velocityPub.publish(&velMsg);
    positionPub.publish(&posMsg);
  }
  node.spinOnce();
#else
  if (Serial.available() > 0) {
    Serial.readBytesUntil(';', serialData, 31);
    parseCommand(serialData);
  }
#endif
  mecControl.update();
}
