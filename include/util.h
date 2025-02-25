#ifndef UTIL_H
#define UTIL_H

#include "constants.h"
#include <iostream>

Time calculateSecondMotorActivation(Time initial_burn_time, float motor_acceleration, float upward_drag_coefficient, float downward_drag_coefficient);

double ForwardEulerMethod(double startTime, double endTime, double yInitial);

#endif