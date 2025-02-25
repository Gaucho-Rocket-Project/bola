#include <iostream>
#include <unistd.h>

#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include "wiringPiTest.h"  
#endif

#include "constants.h"
#include "setup.h"

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