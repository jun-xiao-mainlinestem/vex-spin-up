#include "vex.h"

#define K_STRAFE 1.07
#define K_TURN 5.0

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT10, ratio18_1, false);
motor leftMotorB = motor(PORT1, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT20, ratio18_1, true);
motor rightMotorB = motor(PORT11, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT17);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart,
                                   DrivetrainInertial, 319.19, 320, 40, mm, 1);





// optical rollerOptical = optical(PORT12);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs

bool DrivetrainNeedsToBeStopped = false;


float REDUCE_FACTOR = 0.5;
float NORMAL_FACTOR = 1.0;

int setupSpeed(int position) {
  int speed = position * NORMAL_FACTOR;
  if (Controller1.ButtonR1.pressing())
    speed = position * REDUCE_FACTOR;
  return speed;
}

double setupPower(int position) {
  double power = position * NORMAL_FACTOR;
  if (Controller1.ButtonR1.pressing())
    power = position * REDUCE_FACTOR;
  power = power * 1.0 /100 * 12;
  return power;
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

      // change to field view
      double PI = 3.14159;
      double botHeading = (DrivetrainInertial.heading() * PI / 180);
      double rotX = axis4Value * cos(botHeading) - axis3Value * sin(botHeading);
      double rotY = axis4Value * sin(botHeading) + axis3Value * cos(botHeading);

      if (turnValue == 0 && axis3Value == 0 && axis4Value == 0) {
        if (DrivetrainNeedsToBeStopped) {
          Drivetrain.stop();
          DrivetrainNeedsToBeStopped = false;
        }
      } else {
        int drivetrainLeftASpeed = rotY + turnValue + rotX;
        int drivetrainLeftBSpeed = rotY + turnValue - rotX;
        int drivetrainRightASpeed = rotY - turnValue - rotX;
        int drivetrainRightBSpeed = rotY - turnValue + rotX;

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

void checkMotor(vex::motor &m, const char *name, int row) {

  Brain.Screen.setCursor(row, 1);

  int t = 0;
  int s = 0;
  int v = 0;

  t = m.temperature(celsius);
  s = m.velocity(velocityUnits::rpm);
  v = m.voltage();
  Brain.Screen.print("%s: %dC, %dRPM, %dV", name, t, s, v);
}

int displayMotorInfo() {

  while (true) {
    Brain.Screen.clearScreen();
    checkMotor(leftMotorA, "Left A", 1);
    checkMotor(leftMotorB, "Left B", 3);
    checkMotor(rightMotorA, "right A", 5);
    checkMotor(rightMotorB, "right B", 7);

    wait(500, msec);
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

  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
   task checkTemperatureTask(displayMotorInfo);

  wait(50, msec);
  Controller1.rumble("..");
}