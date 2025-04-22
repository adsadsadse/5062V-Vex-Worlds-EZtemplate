#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(-4);
inline pros::Motor wall_stake(-14);
inline pros::adi::DigitalOut clamp_piston('h');
inline pros::adi::DigitalOut intake_piston('b');
inline pros::adi::DigitalOut doinker_1('b');
inline pros::adi::DigitalOut doinker_2('b');
inline pros::Optical color_sensor(21);
inline pros::Rotation lady_brown_rotation(2);