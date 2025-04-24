#include <cmath>
#include <iostream>
#include <unistd.h>

#include "motor.h"
#include "constants.h"

float translate_servo_angle_pwm(float angle) { return 318.31 * angle + 1000; }