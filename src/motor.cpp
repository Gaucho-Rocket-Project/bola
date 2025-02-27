#include <cmath>
#include <iostream>
#include <unistd.h>

#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include "wiringPiTest.h"  
#endif

#include "motor.h"
#include "constants.h"

float translate_servo_angle_pwm(float angle) { return 318.31 * angle + 1000; }