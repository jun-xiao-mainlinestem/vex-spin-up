#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

const int SHOOTINGSPEED = 410;

const float ROLLER_OFFSET = 2;
const double ROTATION_ANGLE = 90;
const float ROLLER_DOUBLE = 2;

const float ANGLE_FEED = 27;
float DISTANCE_FEED = 6.7;

const float DISTANCE_EASY = 35;

const float DISTANCE_ROLLER = 3.8;

const float FEEDDELAY = 2.5;

// A global instance of competition
competition Competition;

void spinRoll(float multiple = 1) {
  if (Controller1.ButtonR1.pressing()) {
    roller.spinFor(forward, -1 * ROTATION_ANGLE * multiple, degrees, 100,
                   velocityUnits::pct);
  } else {
    roller.spinFor(forward, ROTATION_ANGLE * multiple, degrees, 100,
                   velocityUnits::pct);
  }
}

void slideToRoller(double overshot = ROLLER_OFFSET) {
  double d = getDistanceAverage(sideDistance, DISTANCE_ROLLER, 36);
  if (d < 0 || (overshot > 0 && d < DISTANCE_ROLLER))
    return;
  slideFor(-(d - DISTANCE_ROLLER + overshot), 50);
}

void rollRoller() {
  slideToRoller();
  spinRoll(-ROLLER_DOUBLE);
  slideFor(ROLLER_OFFSET);
}

void rollRollerForward() {
  // slideToRoller();
  spinRoll(-ROLLER_DOUBLE);
  // slideFor(ROLLER_OFFSET);
}

void rollRollerReverse() {
  //  slideToRoller();
  spinRoll(ROLLER_DOUBLE);
  //  slideFor(ROLLER_OFFSET);
}

void southRoll() {
  turnWest();
  rollRoller();
}

void westRoll() {
  turnNorth();
  rollRoller();
}

void eastRoll() {
  turnSouth();
  rollRoller();
}

void northRoll() {
  turnEast();
  rollRoller();
}

void rollRollerAuto() {
  int d = getOrientation();
  switch (d) {
  case 0:
    westRoll();
    break;
  case 1:
    northRoll();
    break;
  case 2:
    eastRoll();
    break;
  case 3:
    southRoll();
    break;
  }
}

void shootDisks(int n = 2, int speed = SHOOTINGSPEED, double voltage = 10,
                int timeOut = 5000) {
  shooter.spin(forward, voltage, voltageUnits::volt);
  int counter = 0;

  for (int i = 0; i < n; i++) {
    while (fabs(shooter.velocity(rpm)) < speed && counter < timeOut / 10) {
      wait(10, msec);
      counter++;
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

void onlyShoot() {
  Drivetrain.setHeading(270, degrees);

  Drivetrain.turnFor(-ANGLE_FEED, degrees, 50, velocityUnits::pct, false);
  shootDisks();
  turnWest();
}

void feedShoot() {
  Drivetrain.turnFor(-ANGLE_FEED, degrees, 50, velocityUnits::pct, false);
  shootDisks();
  turnWest();
}
void roller1() {
  double d = getDistanceAverage(frontDistance, 69, 5);
  if (d < 0)
    d = 69;

  Drivetrain.driveFor(forward, d - DISTANCE_EASY + 4, inches, 50,
                      velocityUnits::pct);
  slideToRoller();
  spinRoll(ROLLER_DOUBLE);
}

void roller2() {
  slideFor(26, 50);
  turnNorth();
  slideToRoller();
  spinRoll(ROLLER_DOUBLE);
}

void feedShoot2() {
  slideFor(66);
  turnNorth();

  Drivetrain.driveFor(forward, 95, inches, 100, velocityUnits::pct);

  turnEast();

  double d = getDistanceAverage(frontDistance, 69, 24);
  Drivetrain.driveFor(forward, d - 69, inches, 50, velocityUnits::pct);
  turnEast();

  d = sideDistance.objectDistance(inches);
  slideFor(-(d - DISTANCE_FEED), 40);

  turnEast();
}

void roller3() { roller1(); }

void roller4() {
  slideFor(26, 50);
  turnSouth();
  slideToRoller();
  spinRoll(ROLLER_DOUBLE);
}

void shootDrive() {
  slideFor(26);
  turnSouth();
  quickTurn(-45);
  slideFor(20, 50);

  // Controller1.rumble("..");
  piston.set(true);
  wait(1.5, seconds);
  slideFor(-45);
  slideFor(12);
}

void skillChallenge() {
  Drivetrain.setHeading(270, degrees);
  feedShoot();
  roller1();
  roller2();
  feedShoot2();
  roller3();
  roller4();
  shootDrive();
}

void testFunction() {
  Drivetrain.setHeading(270, degrees);

  Brain.Timer.clear();
  toggleFeeding();

  double d = sideDistance.objectDistance(inches);

  while (true) {
    slideFor(-d, 40);
    wait(FEEDDELAY, seconds);
    slideFor(d - 0.7, 40);
    double a = - (ANGLE_FEED + Drivetrain.heading() - 270);
    Drivetrain.turnFor(a, degrees, 70, velocityUnits::pct);
    wait(3, seconds);
    quickTurn(ANGLE_FEED, 50);
  }

  toggleFeeding();
  int t = Brain.Timer.time(sec);
  Controller1.Screen.print("time: %d", t);
}

void pre_auton(void) {
  vexcodeInit();
  piston.set(false);
}

void resetMotors() {
  leftMotorA.resetPosition();
  leftMotorB.resetPosition();
  rightMotorA.resetPosition();
  rightMotorB.resetPosition();
  roller.resetPosition();
  indexer.setRotation(0, degrees);
  roller.setTimeout(800, msec);
}

void autonomous(void) { skillChallenge(); }

bool pistonStatus = false;
void togglePiston() {
  if (Controller1.ButtonR1.pressing()) {
    Controller1.rumble(".");
    pistonStatus = !pistonStatus;
    piston.set(pistonStatus);
  }
}

void usercontrol(void) {

  feedShoot();

  while (true) {

    wait(20, msec);
  }
}

int main() {
  Controller1.ButtonX.pressed(testFunction);
  
  Controller1.ButtonLeft.pressed(turnNorth);
  Controller1.ButtonRight.pressed(turnSouth);
  Controller1.ButtonUp.pressed(turnEast);
  Controller1.ButtonDown.pressed(turnWest);


  Controller1.ButtonL1.pressed(rollRollerAuto);
  Controller1.ButtonL2.pressed(rollRollerForward);
  Controller1.ButtonR2.pressed(rollRollerReverse);
  Controller1.ButtonA.pressed(toggleFeeding);
  Controller1.ButtonY.pressed(togglePiston);

  resetMotors();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  Drivetrain.setHeading(270, degrees);

  while (true) {
    wait(100, msec);
  }
}