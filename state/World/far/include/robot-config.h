using namespace vex;

extern brain Brain;

// VEXcode devices

extern motor leftMotorA;
extern motor leftMotorB;
extern motor rightMotorA;
extern motor rightMotorB;

extern motor roller;
extern motor indexer;

extern controller Controller1;

extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

extern motor_group shooter;

extern smartdrive Drivetrain;
extern inertial DrivetrainInertial;

//extern bumper backBumper;
extern distance backDistance;
extern distance frontDistance;
extern distance sideDistance;
//extern optical rollerOptical;

extern brain Brain;
extern digital_out piston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
void quickTurn(double angle, int speed = 75, bool wait_complete=true);
void slideFor(double distance, int speed = 75, bool wait_complete=true);

void toggleOrientation();
void toggleTurn();

void turnSouth();
void turnNorth();
void turnWest();
void turnEast();

double getDistanceAverage(distance sensor, double expectedValue, double range = 5, int timeout = 1000);

int checkTemperature();

int getOrientation();

// void slideFor(vex::directionType, float, float, bool );