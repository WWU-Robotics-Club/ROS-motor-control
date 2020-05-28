#include <Arduino.h>
#include <unity.h>

//using namespace fakeit;

#include "MecanumController.h"
#include "MotorController.h"

#define NUM_MOTORS 4
#define BASE_WIDTH 0.5
#define BASE_LENGTH 0.5
#define WHEEL_RADIUS 0.1

// make fake hardware
class MockMotor : public MotorController {
  public:
    MockMotor() : MotorController() {};
    void initHardware() {};
    void setStandbyHardware(bool standby) {};
    double getPosition() {
      return position;
    };
    void write(int16_t power) {
      outputPower = power;
    };
    double position = 0.0;
    int16_t outputPower = 0;
};

/*
MecanumController mecControl(motors, BASE_WIDTH, BASE_LENGTH, WHEEL_RADIUS);
*/
/*
// test converting {x, y, rotation} into wheel rotations
void test_poseToWheels(void) {
  Pose2D pose;
  double wheels[NUM_MOTORS];
  // test forward
  pose.x = 1.0;
  pose.y = 0.0;
  pose.theta = 0.0;
  mecControl.poseToWheels(pose, wheels);
  for (unsigned int i = 0; i < NUM_MOTORS; i++) {
    TEST_ASSERT_EQUAL_DOUBLE(pose.x / WHEEL_RADIUS, wheels[i]);
  }
}
*/

int main(int argc, char **argv) {
  //ArduinoFakeReset();

  MockMotor testMotors = MockMotor();/*[NUM_MOTORS] = {
    MockMotor(),
    MockMotor(),
    MockMotor(),
    MockMotor(),
  };*/

  UNITY_BEGIN();
  TEST_ASSERT_TRUE(true);
  //mecControl.init();
  //RUN_TEST(test_poseToWheels);
  //TEST_ASSERT_EQUAL_INT16(1,1);
  //TEST_ASSERT_EQUAL_INT16(1,1);
  UNITY_END();
}
