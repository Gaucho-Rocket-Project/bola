#ifndef SETUP_H
#define SETUP_H

#include <iostream>
#include <unistd.h>
#include <wiringPi.h>

#include "constants.h"

void setup() {
  wiringPiSetupGpio();

  // LED Pins
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // PYRO channels
  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  pinMode(PYRO_3, OUTPUT);
}

#endif