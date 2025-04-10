#include <iostream>
#include <unistd.h>

#include "constants.h"
#include "setup.h"

void setup() {
  wiringPiSetupGpio();

  // LED Pins
  pinMode(led_r, OUTPUT);
  pinMode(led_g, OUTPUT);
  pinMode(led_b, OUTPUT);

  // PYRO channels
  pinMode(pyro_1, OUTPUT);
  pinMode(pyro_2, OUTPUT);
  pinMode(pyro_3, OUTPUT);
}
