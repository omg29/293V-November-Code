#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

//rake
inline pros::adi::Pneumatics rake('A', false);

//park
inline pros::adi::Pneumatics park('B', false);

//loading piston
inline pros::adi::Pneumatics loadingPiston('C', false);

//descore
inline pros::adi::Pneumatics descore('D', false);