#include "statemachine.h"
#include "constants.h"
#include <chrono>
#include <ctime>
#include <iostream>

#include <shared_mutex>

#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include "logger.h"  
#endif

// float burnTime(Time initialBurnTime) {
//   std::chrono::duration<double> duration =
//       initialBurnTime -
//       startTime; // need to know why duration is only defined if theres a
//       math
//                  // operation between initialBurnTime and startTime
//   double timeInSeconds = duration.count();
//   return timeInSeconds *
//          ((2 * THRUST_FORCE) / (GRAVITY * TOTAL_ROCKET_MASS) - 1);
// }

sensor_module::sensor_module(state_data &state) : _state(state), _trigger(new sensor_trigger()) {}

sensor_module::~sensor_module() { delete this->_trigger; }

void sensor_module::update_euler_angles() {

}

void sensor_module::update_height() {}

sensor_trigger::sensor_trigger() : _parachute_shunt(1) {}

int sensor_trigger::trigger_landing_legs() {}
int sensor_trigger::trigger_second_motor() {}
int sensor_trigger::trigger_parachute() {}

tvc::tvc(state_data &state) : _state(state) {}