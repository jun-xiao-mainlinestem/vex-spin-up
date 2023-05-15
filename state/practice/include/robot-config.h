using namespace vex;

extern brain Brain;

// VEXcode devices

extern motor leftMotorA;
extern motor leftMotorB;
extern motor rightMotorA;
extern motor rightMotorB;

extern controller Controller1;

extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

extern smartdrive Drivetrain;
extern inertial DrivetrainInertial;

extern brain Brain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

int checkTemperature();
