#ifndef SERVO_H
#define SERVO_H

#include <iostream>

// given desired angle, output PWM signal output
float pwmOutput(double angle);
double pwmWrite(double startTime, double endTime, int sampleRate);

#endif