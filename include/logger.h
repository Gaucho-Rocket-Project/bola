#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

#define HIGH 1
#define LOW 0

#define OUTPUT 1
#define INPUT 0

#define PWM_OUTPUT 2

// Psuedo wiringPi functions for testing
int digitalRead(int pin);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
void pwmWrite(int pin, int value);
void usleep(int ms);

void wiringPiSetupGpio();

#endif // LOGGER_H