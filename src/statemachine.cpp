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

sensor_module::sensor_module(state_data &state) : _state(state), _trigger(std::make_unique<sensor_trigger>(state)) {}

sensor_module::~sensor_module() = default;


//function pointers
signed char read_us(uint8_t addr, uint8_t *data, uint32_t len) {
    int pinState = digitalRead(addr);
    return pinState;
}
icm20948_read_fptr_t read_func = &read_us;

signed char write_us(uint8_t addr, const uint8_t *data, uint32_t len) {
    if (data != nullptr) {
        digitalWrite(addr, *data);
    } else {
        std::cerr << "Error: data pointer is null in write_us function." << std::endl;
    }
    return 0;
}
icm20948_write_fptr_t write_func = write_us;

void delay_us(uint32_t delay) {
void sensor_module::update_euler_angles() {
    // Update euler angles here
}
void sensor_module::update_euler_angles() {
    std::shared_lock<std::shared_mutex> lock(_state.mutex);
    // Update euler angles here
}

void sensor_module::update_height() {
  std::shared_lock<std::shared_mutex> lock(_state.mutex);
  state_data &state = _state;

  float pressure_reading = 0; // implement reading from barometer TODO

  if (pressure_reading > 0) {
      _state.height = -((R * T0) / (TOTAL_ROCKET_MASS * GRAVITY)) * ((std::log(pressure_reading / P0)) / (std::log(2.71828)));
  } else {
      std::cerr << "Error: Invalid pressure reading (zero or negative). Height calculation skipped." << std::endl;
  }
  std::shared_lock<std::shared_mutex> unlock(_state.mutex);
}

void log_height(float height){
    static auto start_time = std::chrono::steady_clock::now();
    static std::ofstream log_file("heightlog.txt", std::ios::app);

    if (!log_file.is_open()) {
        std::cerr << "Cannot open heightlog.txt" << std::endl;
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    long seconds = ms / 1000;
    long milliseconds = ms % 1000;

    log_file << seconds << "." << milliseconds << " s, Height: " << height << " meters\n";
}

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

    return 0; 
}

tvc::tvc(state_data &state) : _state(state) {}

ReactionWheelController::ReactionWheelController(state_data& state, int rollPin, int pitchPin, int yawPin)
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Error: Failed to initialize GPIO using wiringPiSetupGpio." << std::endl;
        std::exit(EXIT_FAILURE); // Exit the program if GPIO setup fails
    }
    pinMode(_pwmPinRoll, PWM_OUTPUT);
    pinMode(_pwmPinPitch, PWM_OUTPUT);
    pinMode(_pwmPinYaw, PWM_OUTPUT);
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

    // Adjust pwmValue for negative output to invert the signal
    pwmValue = (output < 0) ? (1024 - pwmValue) : pwmValue;

    pwmWrite(pwmPin, pwmValue);
    prevError = error;
    std::shared_lock<std::shared_mutex> unlock(_state.mutex);
}
