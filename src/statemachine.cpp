#include "statemachine.h"
#include "constants.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <vector> // added 
#include <cmath> // added

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

sensor_module::sensor_module(state_data &mux) : mux(mux) {
  this->trigger = new sensor_trigger();
}

sensor_module::~sensor_module() { delete this->trigger; }

double roll = 0.0, yaw = 0.0; // initial angles for test
const int pitch = 0;
void sensor_module::update_euler_angles() { // updated
  // roll is x, pitch is y (zeroed out), yaw is z

  // 1. get gyro_data vector 2. make matlab transformation matrix w/ cmath and store
  using namespace std;

  vector<double> angularRates = gyro_data; // tried to take gyro_data as a vector of doubles (idk how cuz its different data type lol)
    
    double p = angularRates[0];
    double q = angularRates[1];
    double r = angularRates[2];
    
    // transformation matrix J from matlab
    vector<vector<double>> J = { // vector of vectors of doubles
        {1, sin(roll) * tan(pitch), cos(roll) * tan(pitch)},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll) / cos(pitch), cos(roll) / cos(pitch)}
    };
    
    // initialize angular rates vector in earth reference frame
    vector<double> earthAngularRates(3, 0.0);
    
    // matrix multiplication
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            earthAngularRates[i] += J[i][j] * angularRates[j];
        }
    } 
    // might not be correct from here on
    //integrate angular rates --> update angles at small steps using euler's method angle_n+1 = angle_n + ω * dt 
    roll  += earthAngularRates[0] * TIME_STEP; // x
    // pitch += earthAngularRates[1] * TIME_STEP; zeroed out
    yaw   += earthAngularRates[2] * TIME_STEP; // z
    
}


void sensor_module::update_height() {}

sensor_trigger::sensor_trigger() : parachute_shunt(1) {}

int sensor_trigger::trigger_landing_legs() {}
int sensor_trigger::trigger_second_motor() {}
int sensor_trigger::trigger_parachute() {}

tvc::tvc(state_data &mux) : mux(mux) {}