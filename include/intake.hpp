#include "main.h"

//motors
extern pros::Motor bottomIntake;
extern pros::Motor upperIntake;
extern pros::Motor midIntake;

//functions
extern void setIntake(int bottomIntakeSpeed, int upperIntakeSpeed, int midIntakeSpeed);
extern void intakeLoad();
extern void intakeLongGoal();
extern void intakeCenterGoal();
extern void intakeOuttake();
extern void intakeHalt();
extern void asyncIntakeControl(void * param);

extern void cycleAllianceColor();

//variables
extern int seenColor;
extern int allianceColor;

