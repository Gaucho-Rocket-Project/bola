#include "pwm.h"
#include "setup.h"
#include "servo.h"

#include <iostream>

float pwmTranslate(double angle) { return (318.31 * angle + 1000); }
