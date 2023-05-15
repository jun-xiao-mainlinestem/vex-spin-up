#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  vexcodeInit();
  // menu();
}

void resetMotors() {
  leftMotorA.resetPosition();
  leftMotorB.resetPosition();
  rightMotorA.resetPosition();
  rightMotorB.resetPosition();
}


void autonomous(void) {

}

bool holdRobotStopped = true;

void usercontrol(void) {

  long timeCount = 0;


  while (true) {

    if (Controller1.ButtonB.pressing()) {

      Drivetrain.stop(hold);
      holdRobotStopped = false;

    } else if (!holdRobotStopped) {
      Drivetrain.stop(coast);
      holdRobotStopped = true;
    }

    timeCount++;
    if (timeCount == 90 * 50)
      Controller1.rumble("----");
    wait(20, msec);
  }
}

int main() {
  resetMotors();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  Drivetrain.setHeading(0, degrees);

  while (true) {
    wait(100, msec);
  }
}