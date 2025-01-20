#include "pwm.h"
#include "setup.h"
#include <iostream>

float pwmTranslate(double angle);

int main() { setup(); }

float pwmTranslate(double angle) { return (318.31 * angle + 1000); }
