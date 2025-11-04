#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "intake.hpp"
#include <cstdint>
#include "subsystems.hpp"

//intake motors
pros::Motor bottomIntake(11);
pros::Motor upperIntake(12);
pros::Motor midIntake(13);

//color sensor
pros::Optical colorSensor(20);

//intake speeds
int bottomIntakeSpeed;
int upperIntakeSpeed;
int midIntakeSpeed;

//set intake speed
void setIntake(int bottomSpeed, int upperSpeed, int midSpeed){
    bottomIntakeSpeed = bottomSpeed;
    upperIntakeSpeed = upperSpeed;
    midIntakeSpeed = midSpeed;
}

//load blocks
void intakeLoad() {
    bottomIntakeSpeed = 127;
    upperIntakeSpeed = 0;
    midIntakeSpeed = 127;
    loadingPiston.extend();
}

//score on long goals
void intakeLongGoal() {
    bottomIntakeSpeed = 127;
    upperIntakeSpeed = 127;
    midIntakeSpeed = -127;
    loadingPiston.retract();
}

//score on upper center goal
void intakeCenterGoal() {
    bottomIntakeSpeed = 127;
    upperIntakeSpeed = -127;
    midIntakeSpeed = -127;
    loadingPiston.retract();
}

//outtake (also bottom center goal)
void intakeOuttake() {
    bottomIntakeSpeed = -100;
    upperIntakeSpeed = -127;
    midIntakeSpeed = -127;
    loadingPiston.retract();
}

//stop intake
void intakeHalt() {
    bottomIntakeSpeed = 0;
    upperIntakeSpeed = 0;
    midIntakeSpeed = 0;
}

//color sort variables
/*
0 = RED
1 = BLUE
2 = NEUTRAL 
*/
bool blockDetected = false;
bool blockInIntake = false;

int allianceColor = 0;
int seenColor = 2;

int elapsedTime = 0;

//cycle alliance color function
void cycleAllianceColor() {
    allianceColor += 1;
    if (allianceColor == 3) {
        allianceColor = 0;
    }
}

//anti jam timer
uint32_t lastCheckTime = pros::millis();
bool intakeJammed = false;

//intake async control
void asyncIntakeControl(void * param) {
    //color sort var
    bool wrongBlock = false;
    elapsedTime = 0;
    
    //while loop
    while(true) {
        //color sensor led
        colorSensor.set_led_pwm(100);

        //run intake
        bottomIntake.move(bottomIntakeSpeed);
        upperIntake.move(upperIntakeSpeed);
        midIntake.move(midIntakeSpeed);

        //anti jam
        if(pros::millis() - lastCheckTime >=300) {
            //check if any motors are jammed
            if(((upperIntake.get_efficiency()/100 <= 0.1) && (upperIntakeSpeed > 10 || upperIntakeSpeed < -10)) || ((bottomIntake.get_efficiency()/100 <= 0.1) && (bottomIntakeSpeed > 10 || bottomIntakeSpeed < -10)) || ((midIntake.get_efficiency()/100 <=0.1) && (midIntakeSpeed > 10 || midIntakeSpeed < -10))) {
                intakeJammed = true;
                //unjam
                bottomIntake.move(-127);
                upperIntake.move(-127);
                midIntake.move(-127);
                pros::delay(200);
                intakeJammed = false;
                lastCheckTime = pros::millis();
            //if not jammed
            } else {
                lastCheckTime = pros::millis();
            }
        }

        //color sort
        if ((colorSensor.get_hue() > 340) || (colorSensor.get_hue() < 20)) {
            //red
            seenColor = 0;
        } else if ((colorSensor.get_hue() > 215) && (colorSensor.get_hue() < 255)) {
            //blue
            seenColor = 1;
        } else {
            //neutral
            seenColor = 2;
        }

        if(colorSensor.get_proximity() > 200) {
            blockDetected = true;
            blockInIntake = true;
        } else {
            blockDetected = false;
        }

        //check if wrong block
        if (!wrongBlock) {
            if (allianceColor == 0 && seenColor == 1) {
                wrongBlock = true;
                elapsedTime = 0;

            } else if (allianceColor == 1 && seenColor == 0) {
                wrongBlock = true;
                elapsedTime = 0;

            } else {
                wrongBlock = false;
            }
        }

        //sort block
        if(wrongBlock && elapsedTime > 0) {
            //do smth
            pros::delay(150);
            wrongBlock = false;
        }

        //increase elapsed time
        elapsedTime += 1;

        //delay
        pros::delay(20);
    }
}