#include <iostream>

#include "logger.h"  

#include "constants.h"
#include "led.h"
#include "setup.h"

int main() {

  setup();
  testLED();

  return 0;
}
