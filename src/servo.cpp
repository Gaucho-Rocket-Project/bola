#include <iostream>
#include <unistd.h>
#include <wiringPi.h>

#include "constants.h"
#include "pwm.h"

float pwm_output(double angle) { return 318.31 * angle + 1000; }