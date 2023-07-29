#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

// A global instance of competition
competition Competition;

void spinRoll(float multiple = 1) {
  roller.spinFor(forward, 90 * multiple, degrees, 100, velocityUnits::pct);
}

void rollReverse() { spinRoll(-2); }

void shootDisks(int n = 2) {
  shooter.spin(forward, 10, voltageUnits::volt);

  for (int i = 0; i < n; i++) {
    while (shooter.velocity(rpm) < 410) {
      wait(10, msec);
    }
    indexer.spinTo(90, degrees, 20, velocityUnits::pct);
    indexer.spinTo(0, degrees, 100, velocityUnits::pct);
  }

  shooter.stop(coast);
}

bool feedingIsOn = false;
void turnOnfeeding() {
  roller.spin(forward, 12, voltageUnits::volt);
  feedingIsOn = true;
}

void turnOffFeeding() {
  roller.stop(coast);
  feedingIsOn = false;
}

void toggleFeeding() {
  if (!feedingIsOn) {
    turnOnfeeding();
  } else {
    turnOffFeeding();
  }
}

void shootRollFar() {
  Drivetrain.setHeading(180, degrees);
  Drivetrain.turnFor(20, degrees, 50, velocityUnits::pct, false);
  shootDisks();
  quickTurn(-20, 50);
  double d = frontDistance.objectDistance(inches);

  Drivetrain.driveFor(reverse, d - 34, inches, 50, velocityUnits::pct);
  d = sideDistance.objectDistance(inches);
  slideFor(-(d - 3.8 + 2), 50);
  spinRoll();
}

void afterFar() {
  // collect 3 disks
  slideFor(5, 50);
  quickTurn(45);
  Drivetrain.driveFor(forward, 64, inches, 60, velocityUnits::pct);
  quickTurn(-135, 40);

  // drive to along the edge of the low goal
  double d = frontDistance.objectDistance(inches);
  Drivetrain.driveFor(forward, d - 13, inches, 70, velocityUnits::pct);

  // push in 5 disks
  slideFor(-3);
  quickTurn(90, 45);
  slideFor(-5);
  Drivetrain.driveFor(forward, 26, inches, 100, velocityUnits::pct);
}

void far() {
  shootRollFar();
  afterFar();
}

void pre_auton(void) {
  vexcodeInit();
  piston.set(false);
}

bool pistonStatus = false;
void togglePiston() {
    pistonStatus = !pistonStatus;
    piston.set(pistonStatus);
}

float DISTANCE_FEED;
void feedShoot() {
  toggleFeeding();

  double d = sideDistance.objectDistance(inches);
  DISTANCE_FEED = d;

  Drivetrain.turnFor(-27, degrees, 50, velocityUnits::pct, false);
  shootDisks();
  quickTurn(27, 50);

  slideFor(-d, 40);
  wait(19, seconds);
  slideFor(d - 0.7, 40);
  Drivetrain.turnFor(-27, degrees, 50, velocityUnits::pct, false);
  shootDisks(3);
  quickTurn(27, 50);
  slideFor(-d, 40);
  wait(19, seconds);

  slideFor(d - 0.7, 40);
  Drivetrain.turnFor(-27, degrees, 50, velocityUnits::pct, false);
  shootDisks(3);
  quickTurn(27, 50);
  toggleFeeding();
}

void slideToRoller() {
  double d = sideDistance.objectDistance(inches);
  slideFor(-(d - 3.8 + 2), 50);
}

void roller1() {
  double d = frontDistance.objectDistance(inches);

  Drivetrain.driveFor(forward, d - 35 + 4, inches, 50, velocityUnits::pct);
  slideToRoller();
  spinRoll(2);
}

void roller2() {
  slideFor(26, 50);
  turnNorth();
  slideToRoller();
  spinRoll(2);
}

void feedShoot2() {
  slideFor(66);
  turnNorth();

  Drivetrain.driveFor(forward, 95, inches, 100, velocityUnits::pct);

  turnEast();
  toggleFeeding();

  double d = frontDistance.objectDistance(inches);
  Drivetrain.driveFor(forward, d - 69, inches, 50, velocityUnits::pct);
  turnEast();

  d = sideDistance.objectDistance(inches);
  slideFor(-d, 40);
  wait(2.5, seconds);

  d = DISTANCE_FEED;
  slideFor(d - 0.7, 40);

  double a = -(27 + Drivetrain.heading() - 90);
  Drivetrain.turnFor(a, degrees, 50, velocityUnits::pct, false);

  //  Drivetrain.turnFor(-ANGLE_FEED, degrees, 50, velocityUnits::pct, false);
  shootDisks(3);
  turnEast();

  d = sideDistance.objectDistance(inches);
  slideFor(-d, 40);
  wait(2.5, seconds);
  slideFor(d - 0.7, 40);
  Drivetrain.turnFor(-27, degrees, 50, velocityUnits::pct, false);
  shootDisks(3);
  toggleFeeding();
  turnEast();
}

void roller3() { roller1(); }

void roller4() {
  slideFor(26, 50);
  turnSouth();
  slideToRoller();
  spinRoll(2);
  slideFor(26);
  turnSouth();
  quickTurn(-45);
  slideFor(20, 50);
  piston.set(true);
}

void skills() {
  Drivetrain.setHeading(270, degrees);
  feedShoot();
  roller1();
  roller2();
  feedShoot2();
  roller3();
  roller4();
}

void usercontrol(void) {
  while (true) {
    wait(20, msec);
  }
}

void autonomous(void) { far(); }

int main() {

  Controller1.ButtonR2.pressed(rollReverse);
  Controller1.ButtonA.pressed(toggleFeeding);
  Controller1.ButtonY.pressed(togglePiston);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();
  Drivetrain.setHeading(180, degrees);

  while (true) {
    wait(100, msec);
  }
}