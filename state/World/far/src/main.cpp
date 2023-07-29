#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

const int SHOOTINGSPEED = 385;
const float ROLLER_OFFSET = 2;
const float FAR_AUTON_DRIVE = 61;

const double ROTATION_ANGLE = 90;
const float ROLLER_DOUBLE = 2;

const float ANGLE_EASY = 21;
const float ANGLE_HARD = 18;

const float DISTANCE_EASY = 35;
const float DISTANCE_ROLLER = 3.8;

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

void shootOnce()
{
  shootDisks(1, 300);
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

// robot setup: align to the edge of tile with 1 channel distance
void shootRollNear() {
  Drivetrain.setHeading(270, degrees);
  Drivetrain.turnFor(-ANGLE_EASY, degrees, 70, velocityUnits::pct, false);
  shootDisks();
  quickTurn(ANGLE_EASY, 50);
  double d = getDistanceAverage(frontDistance, 53.5, 5);
  if (d < 0)
    d = 53.5;

  Drivetrain.driveFor(forward, d - DISTANCE_EASY, inches, 50,
                      velocityUnits::pct);
  slideToRoller();
  spinRoll();
}

void afterNear() {
  // colect 3 stack of disks
  slideFor(4, 50);
  quickTurn(142);
  Drivetrain.driveFor(forward, 40, inches, 50, velocityUnits::pct);

  // drive to the edge of low goal
  quickTurn(38, 40);
  turnEast();
  Drivetrain.driveFor(forward, 12, inches, 50, velocityUnits::pct);

  // push 3 disks in
  slideFor(31, 30);
  turnEast();
  Drivetrain.driveFor(forward, 35, inches, 50, velocityUnits::pct);
}

void near() {
  shootRollNear();
  afterNear();
}

void shootRollFar() {
  Drivetrain.setHeading(180, degrees);
  Drivetrain.turnFor(ANGLE_HARD, degrees, 50, velocityUnits::pct, false);
  shootDisks();
  quickTurn(-ANGLE_HARD, 50);

  Drivetrain.driveFor(reverse, 20, inches, 50,
                      velocityUnits::pct);
  slideToRoller();
  spinRoll();
}

void afterFar() {
 
  // collect 3 disks
  slideFor(5, 50);

  quickTurn(45 + 180 - DrivetrainInertial.heading(), 50);


  Drivetrain.driveFor(forward, FAR_AUTON_DRIVE, inches, 75, velocityUnits::pct);


  turnEast();
  wait(500, msec);
  turnEast();

  // drive to along the edge of the low goal
  double d = getDistanceAverage(frontDistance, 57, 10);
  if (d < 0)
    return;
  Drivetrain.driveFor(forward, d - 13, inches, 70, velocityUnits::pct);

  // push in 5 disks
  slideFor(-14, 50);
  quickTurn(90 + 90 - DrivetrainInertial.heading(), 45);
  slideFor(-5);
  turnSouth();
  Drivetrain.driveFor(forward, 38, inches, 100, velocityUnits::pct); 
}

void afterFar2() {
  // collect 2 disks
  slideFor(5, 50);
  quickTurn(45);
  Drivetrain.driveFor(forward, 50, inches, 60, velocityUnits::pct);
  quickTurn(-135, 40);

  // drive to along the edge of the low goal
  double d = getDistanceAverage(frontDistance, 43, 10);
  if (d < 0)
    return;
  Drivetrain.driveFor(forward, d - 13, inches, 70, velocityUnits::pct);

  // push in  disks
  turnSouth();
  Drivetrain.driveFor(forward, 36, inches, 70, velocityUnits::pct);
  turnSouth();
}

void far() {
  shootRollFar();
  afterFar();
}

void shootRollFarTest() {
  Drivetrain.setHeading(180, degrees);
  Drivetrain.turnFor(ANGLE_HARD, degrees, 50, velocityUnits::pct, false);
  shootDisks();
  Drivetrain.turnFor(ANGLE_HARD, degrees, -50, velocityUnits::pct, true);
}


void testFunction() {
  Brain.Timer.clear();

  Drivetrain.setHeading(180, degrees);

  shootRollFarTest();

  int t = Brain.Timer.time(sec);
  Controller1.Screen.print("time: %d", t);
}

void elimination() {}

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

int shootString() {
  Brain.Timer.clear();

  while (!(Brain.Timer.time(sec) > 80)) {
    wait(50, msec);
  }
  Controller1.rumble("---");

  while (!(Brain.Timer.time(sec) > 95)) {
    wait(50, msec);
  }
  Controller1.rumble("---");

  /*while (true) {
    double heading = DrivetrainInertial.heading();
    if (heading > 90 + 40 && heading <= 90 + 50) {
      piston.set(true);
      Controller1.rumble("..");
      Brain.Timer.clear();
      while (true) {

        Drivetrain.stop(hold);
        if (Brain.Timer.time(sec) > 1) {
          Drivetrain.stop(brake);
          return 0;
        }
        wait(10, msec);
      }
    }
    wait(30, msec);
  }*/

  return 0;
}

bool pistonStatus = false;
void togglePiston() {
  if (Controller1.ButtonR1.pressing()) {
    Controller1.rumble(".");
    pistonStatus = !pistonStatus;
    piston.set(pistonStatus);
  }
}

bool holdRobotStopped = true;

void usercontrol(void) {
  task stringLancher(shootString);

  while (true) {

    if (Controller1.ButtonB.pressing()) {

      Drivetrain.stop(hold);
      holdRobotStopped = false;

    } else if (!holdRobotStopped) {
      Drivetrain.stop(coast);
      holdRobotStopped = true;
    }
    wait(20, msec);
  }
}

void autonomous(void) { 
   far();
   }

int main() {
  Controller1.ButtonLeft.pressed(turnWest);
  Controller1.ButtonRight.pressed(turnEast);
  Controller1.ButtonUp.pressed(turnNorth);
  Controller1.ButtonDown.pressed(turnSouth);

//Controller1.ButtonX.pressed(testFunction);


  Controller1.ButtonX.pressed(shootOnce);

  Controller1.ButtonL1.pressed(rollRollerAuto);
  Controller1.ButtonL2.pressed(rollRollerForward);
  Controller1.ButtonR2.pressed(rollRollerReverse);
  Controller1.ButtonA.pressed(toggleFeeding);
  Controller1.ButtonY.pressed(togglePiston);

  resetMotors();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();
  Drivetrain.setHeading(180, degrees);

  while (true) {
    wait(100, msec);
  }
}