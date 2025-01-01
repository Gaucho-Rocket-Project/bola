#include <iostream>
#include <wiringPi.h>

#include "constants.h"



int main(int argc, char **argv) {

  wiringPiSetupGpio();

  // LED Pins
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // PYRO channels
  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  pinMode(PYRO_3, OUTPUT);

  testLED();

  return 0;
}
