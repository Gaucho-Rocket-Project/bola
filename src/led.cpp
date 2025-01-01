#include <iostream>
#include <unistd.h>
#include <wiringPi.h>

#include "constants.h"
#include "led.h"

void parseLED(ushort state = 0) {

  switch (state) {
  case RED:

    std::cout << "R( " << LED_R << "): 0" << std::endl;
    std::cout << "G( " << LED_G << "): 1" << std::endl;
    std::cout << "B( " << LED_B << "): 1" << std::endl;

    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    break;
  case YELLOW:

    std::cout << "R( " << LED_R << "): 0" << std::endl;
    std::cout << "G( " << LED_G << "): 0" << std::endl;
    std::cout << "B( " << LED_B << "): 1" << std::endl;

    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
    break;
  case GREEN:

    std::cout << "R( " << LED_R << "): 1" << std::endl;
    std::cout << "G( " << LED_G << "): 0" << std::endl;
    std::cout << "B( " << LED_B << "): 1" << std::endl;

    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
    break;
  default:
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }
}

void testLED() {
  std::cout << "RED" << std::endl;
  parseLED(RED);
  usleep(1000000);
  std::cout << "YELLOW" << std::endl;
  parseLED(YELLOW);
  usleep(1000000);
  std::cout << "GREEN" << std::endl;
  parseLED(GREEN);
  usleep(1000000);
  parseLED();
}