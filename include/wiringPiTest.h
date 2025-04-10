#ifndef WIRINGPI_TEST_H
#define WIRINGPI_TEST_H

#include <iostream>

#define HIGH 1
#define LOW 0

#define OUTPUT 1
#define INPUT 0


// Fake wiringPi functions for testing
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
void digitalRead(int pin);
void pwmWrite(int pin, int value);
void usleep(int ms);

void wiringPiSetupGpio();

#endif // WIRINGPI_TEST_H
