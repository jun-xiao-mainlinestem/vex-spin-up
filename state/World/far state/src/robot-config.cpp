#include "vex.h"

#define K_STRAFE 1.12
#define K_TURN 5.26

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT11, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT10, ratio18_1, true);
motor rightMotorB = motor(PORT20, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT8);
distance frontDistance = distance(PORT7);
distance backDistance = distance(PORT17);

distance sideDistance = distance(PORT18);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart,
                                   DrivetrainInertial, 319.19, 320, 40, mm, 1);

motor shooterMotor1 = motor(PORT16, ratio6_1, false);
motor shooterMotor2 = motor(PORT15, ratio6_1, true);
motor_group shooter = motor_group(shooterMotor1, shooterMotor2);
motor roller = motor(PORT3, ratio18_1, true);
motor indexer = motor(PORT5, ratio18_1, false);

digital_out piston = digital_out(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs

bool DrivetrainNeedsToBeStopped = false;

float REDUCE_FACTOR = 0.5;
float NORMAL_FACTOR = 1.0;

double setupPower(int position) {
  double power = position * NORMAL_FACTOR;
  if (Controller1.ButtonR1.pressing())
    power = position * REDUCE_FACTOR;
  power = power * 1.0 / 100 * 12;
  return power;
}


void slideFor(float distance, int speed, bool wait_commplete) {
  float numberDeg;
  numberDeg = (distance / (4 * 3.1415)) * 360 * K_STRAFE;
  leftMotorA.spinFor(numberDeg, deg, speed, velocityUnits::pct, false);
  leftMotorB.spinFor(-numberDeg, deg, speed, velocityUnits::pct, false);
  rightMotorA.spinFor(-numberDeg, deg, speed, velocityUnits::pct, false);
  rightMotorB.spinFor(numberDeg, deg, speed, velocityUnits::pct,
                      wait_commplete);
}

void quickTurn(float degree, int speed, bool wait_commplete) {
  double numberDeg;
  numberDeg = degree * K_TURN;
  rightMotorA.spinFor(-1 * numberDeg, deg, speed, velocityUnits::pct, false);
  leftMotorA.spinFor(numberDeg, deg, speed, velocityUnits::pct, false);
  rightMotorB.spinFor(-1 * numberDeg, deg, speed, velocityUnits::pct, false);
  leftMotorB.spinFor(numberDeg, deg, speed, velocityUnits::pct, wait_commplete);
}

void turnNorth() {
  int orientation = DrivetrainInertial.heading();
  if (orientation <= 180) {
    quickTurn(-orientation);
  } else {
    quickTurn(360 - orientation);
  }
}

void turnSouth() {
  int orientation = 180 - DrivetrainInertial.heading();

  quickTurn(orientation);
}

void turnEast() {
  int orientation = DrivetrainInertial.heading();
  if (orientation < 270)
    quickTurn(90 - orientation);
  else
    quickTurn(180 - (orientation - 270));
}

void turnWest() {
  int orientation = DrivetrainInertial.heading();

  if (orientation > 90)
    quickTurn(270 - orientation);
  else
    quickTurn(-(orientation + 90));
}


// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while (true) {
    if (RemoteControlCodeEnabled) {

      int turnValue = setupPower(Controller1.Axis1.position());
      int axis3Value = setupPower(Controller1.Axis3.position());
      int axis4Value = setupPower(Controller1.Axis4.position());

      if (turnValue == 0 && axis3Value == 0 && axis4Value == 0) {
        if (DrivetrainNeedsToBeStopped) {
          Drivetrain.stop();
          DrivetrainNeedsToBeStopped = false;
        }
      } else {
        int drivetrainLeftASpeed = axis4Value + turnValue + axis3Value;
        int drivetrainLeftBSpeed = axis4Value + turnValue - axis3Value;
        int drivetrainRightASpeed = axis4Value - turnValue - axis3Value;
        int drivetrainRightBSpeed = axis4Value - turnValue + axis3Value;

        leftMotorA.spin(forward, drivetrainLeftASpeed, voltageUnits::volt);
        leftMotorB.spin(forward, drivetrainLeftBSpeed, voltageUnits::volt);
        rightMotorA.spin(forward, drivetrainRightASpeed, voltageUnits::volt);
        rightMotorB.spin(forward, drivetrainRightBSpeed, voltageUnits::volt);

        DrivetrainNeedsToBeStopped = true;
      }

      // wait before repeating the process
      wait(20, msec);
    }
  }
  return 0;
}

void vexcodeInit(void) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();

  wait(50, msec);
  Controller1.rumble("..");

  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

}