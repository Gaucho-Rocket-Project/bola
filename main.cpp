#include <iostream>
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

void parseLED(ushort state = 0) {

  switch (state) {
  case RED:
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_R, LOW);
    break;
  case YELLOW:
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, LOW);
    break;
  case GREEN:
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, HIGH);
    break;
  default:
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_R, HIGH);
  }
}

int main() { return 0; }