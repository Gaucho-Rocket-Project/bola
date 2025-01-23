#include "pyro.h" c
#include "wiringPi.h"
#include <iostream>

void setupPyroChannel(int pinNumber) { pinMode(pinNumber, OUTPUT); }
void changePyroChannelState(int pinNumber, bool state) {
  if (state)
    digitalWrite(pinNumber, HIGH);
  else
    digitalWrite(pinNumber, LOW);
}