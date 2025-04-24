#include <iostream>



#include "constants.h"
#include "led.h"

void parseLED(led_state state = NONE) {

  switch (state) {
  case RED:

    std::cout << "R( " << led_r << "): 0" << std::endl;
    std::cout << "G( " << led_g << "): 1" << std::endl;
    std::cout << "B( " << led_b << "): 1" << std::endl;

    digitalWrite(led_r, LOW);
    digitalWrite(led_g, HIGH);
    digitalWrite(led_b, HIGH);
    break;
  case YELLOW:

    std::cout << "R( " << led_r << "): 0" << std::endl;
    std::cout << "G( " << led_g << "): 0" << std::endl;
    std::cout << "B( " << led_b << "): 1" << std::endl;

    digitalWrite(led_r, LOW);
    digitalWrite(led_g, LOW);
    digitalWrite(led_b, HIGH);
    break;
  case GREEN:

    std::cout << "R( " << led_r << "): 1" << std::endl;
    std::cout << "G( " << led_g << "): 0" << std::endl;
    std::cout << "B( " << led_b << "): 1" << std::endl;

    digitalWrite(led_r, HIGH);
    digitalWrite(led_g, LOW);
    digitalWrite(led_b, HIGH);
    break;
  default:
    digitalWrite(led_r, HIGH);
    digitalWrite(led_g, HIGH);
    digitalWrite(led_b, HIGH);
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