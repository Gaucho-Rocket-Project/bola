#include "statemachine.h"
#include "constants.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <cmath>
#include <fstream>
#include <threads.h>
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


//function pointers
//icm20948_read_fptr_t *read_func = digitalRead; //function doens't exist!
signed char write_us(uint8_t addr, const uint8_t *data, uint32_t len) {
    digitalWrite(addr, *data);
    return 0;
}
icm20948_write_fptr_t write_func = write_us;

void delay_us(uint32_t delay) {
    usleep(delay);
}
icm20948_delay_us_fptr_t delay_func = delay_us;

void sensor_module::update_euler_angles() {
    std::shared_lock<std::shared_mutex> lock(_state.mutex);
    // Update euler angles here
    std::shared_lock<std::shared_mutex> unlock(_state.mutex);
}

void sensor_module::update_height() {
  std::shared_lock<std::shared_mutex> lock(_state.mutex);
  state_data &state = _state;

  float pressure_reading = 0; // implement reading from barometer TODO

  _state.height = -((R * T0) / (TOTAL_ROCKET_MASS * GRAVITY)) * ((std::log(pressure_reading / P0)) / (std::log(2.71828)));
  std::shared_lock<std::shared_mutex> unlock(_state.mutex);
}

// missing variables!
// void log_height(float height){ //TODO: add timer to count time by the milliseconds right after launch
//     std::ofstream log_file("heightlog.txt", std::ios::app);
//     if (log_file.is_open()){
//         log_file << seconds << "." << milliseconds << " s, Height: " << height << " meters\n"; 
//         log_file.close();
//     }else{
//         std::cout << "Cannot open heightlog.txt" << std::endl;
//     }
// }

sensor_trigger::sensor_trigger(state_data &state) : _state(state), _parachute_shunt(1) {}

int sensor_trigger::trigger_landing_legs() { return 0; } // TODO
int sensor_trigger::trigger_second_motor() { return 0;} // TODO
int sensor_trigger::trigger_parachute() {

    const tvc_data &tvc = _state.tvc_state; // Dereferencing the state to get the tvc_state and then checking the angles

    // Check to see if the angles exceed 45 degrees
    std::shared_lock<std::shared_mutex> lock(_state.mutex);
    if (std::abs(tvc.euler_angles[0]) > CRITICAL_ANGLE || std::abs(tvc.euler_angles[1]) > CRITICAL_ANGLE) {
        _parachute_shunt = true;  
        return 1;                 
    }
    std::shared_lock<std::shared_mutex> unlock(_state.mutex);

    return 0; 
}

tvc::tvc(state_data &state) : _state(state) {}

ReactionWheelController::ReactionWheelController(state_data& state, int rollPin, int pitchPin, int yawPin)
    : _state(state), _pwmPinRoll(rollPin), _pwmPinPitch(pitchPin), _pwmPinYaw(yawPin) {
    wiringPiSetupGpio();
    pinMode(_pwmPinRoll, PWM_OUTPUT);
    pinMode(_pwmPinPitch, PWM_OUTPUT);
    pinMode(_pwmPinYaw, PWM_OUTPUT);
}

void ReactionWheelController::update() {
    std::shared_lock<std::shared_mutex> lock(_state.mutex);
    const tvc_data& tvc = _state.tvc_state;

    float currentRoll = tvc.euler_angles[0];
    float currentPitch = tvc.euler_angles[1];
    float currentYaw = 0.0; // yaw is unused for now

    compute_pid(currentRoll, 0.0, _prevErrorRoll, _integralRoll, _pwmPinRoll);
    compute_pid(currentPitch, 0.0, _prevErrorPitch, _integralPitch, _pwmPinPitch);
    compute_pid(currentYaw, 0.0, _prevErrorYaw, _integralYaw, _pwmPinYaw);
    std::shared_lock<std::shared_mutex> unlock(_state.mutex);
}

void ReactionWheelController::compute_pid(float currentAngle, float targetAngle, float& prevError, float& integral, int pwmPin) {
    std::shared_lock<std::shared_mutex> lock(_state.mutex);
    float error = targetAngle - currentAngle;
    integral += error * TIME_STEP;
    float derivative = (error - prevError) / TIME_STEP;

    float output = _Kp * error + _Ki * integral + _Kd * derivative;

    int pwmValue = static_cast<int>(fabs(output));
    pwmValue = std::max(0, std::min(pwmValue, 1024));

    if (output < 0) {
        pwmValue = 1024 - pwmValue;
    }

    pwmWrite(pwmPin, pwmValue);
    prevError = error;
    std::shared_lock<std::shared_mutex> unlock(_state.mutex);
}
