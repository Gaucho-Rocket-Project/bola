#include <iostream>

#include "constants.h"
#include "servo.h"
#include "setup.h"

float pwmTranslate(double angle) { return (318.31 * angle + 1000); }
