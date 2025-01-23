#include "servo.h"

double getPWM(double angle) { return 318.31 * angle + 1000; }
