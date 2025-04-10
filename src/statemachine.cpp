#include "statemachine.h"
#include "constants.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <cmath>
#include <fstream>

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


//function pointers
icm20948_read_fptr_t read_func = digitalRead;
icm20948_write_fptr_t write_func = digitalWrite;
icm20948_delay_us_fptr_t delay_func = usleep;

void sensor_module::update_euler_angles() {
  
}

void sensor_module::update_height() {
  std::unique_lock<std::shared_mutex> lock(_state.mutex);
  state_data &state = _state;

  float pressure_reading = 0; // implement reading from barometer TODO

  _state.height = -((R * T0) / (TOTAL_ROCKET_MASS * GRAVITY)) * ((std::log(pressure_reading / P0)) / (std::log(2.71828)));
}

void log_height(float height){ //TODO: add timer to count time by the milliseconds right after launch
    std::ofstream log_file("heightlog.txt", std::ios::app);
    if (log_file.is_open()){
        log_file << seconds << "." << milliseconds << " s, Height: " << height << " meters\n"; 
        log_file.close();
    }else{
        std::cout << "Cannot open heightlog.txt" << std::endl;
    }
}

sensor_trigger::sensor_trigger(state_data &state) : _state(state), _parachute_shunt(1) {}

int sensor_trigger::trigger_landing_legs() { return 0; } // TODO
int sensor_trigger::trigger_second_motor() { return 0;} // TODO
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