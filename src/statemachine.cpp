#include "statemachine.h"
#include "constants.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <cmath>

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

sensor_module::sensor_module(state_data &state) : _state(state), _trigger(new sensor_trigger(state)) {}

sensor_module::~sensor_module() { delete this->_trigger; }


void sensor_module::update_euler_angles() {
  
}

void sensor_module::update_height() {
  std::unique_lock<std::shared_mutex> lock(_state.mutex);
  state_data &state = _state;

  float pressure_reading = 0; // implement reading from barometer

  _state.height = -((R * T0) / (TOTAL_ROCKET_MASS * GRAVITY)) * ((std::log(pressure_reading / P0)) / (std::log(2.71828)));
}

sensor_trigger::sensor_trigger(state_data &state) : _state(state), _parachute_shunt(1) {}

int sensor_trigger::trigger_landing_legs() {}
int sensor_trigger::trigger_second_motor() {}
int sensor_trigger::trigger_parachute() {
  std::shared_lock<std::shared_mutex> lock(_state.mutex);
    const tvc_data &tvc = _state.tvc_state; // Dereferencing the state to get the tvc_state and then checking the angles

    // Check to see if the angles exceed 45 degrees
    if (std::abs(tvc.euler_angles[0]) > CRITICAL_ANGLE || std::abs(tvc.euler_angles[1]) > CRITICAL_ANGLE) {
        _parachute_shunt = true;  
        return 1;                 
    }

    return 0; 
}

tvc::tvc(state_data &state) : _state(state) {}

void tvc_data::update_control_signal(state_data &state) {
  std::unique_lock<std::shared_mutex> lock(_state.mutex);
  tvc_data &tvc = _state.tvc_state;

  for (uint8_t axis = 0; axis < 2; ++axis) {
    tvc.cs[axis] = (Kp * tvc.euler_angles[axis]) + (Ki * tvc.angle_summations[axis]) + (Kd * tvc.angular_velocities[axis]);
  }
}