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

float roll = 0.0, yaw = 0.0;

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

sensor_trigger::sensor_trigger(state_data &state) : _state(state), _parachute_shunt(1) {}

int sensor_trigger::trigger_landing_legs() {}
int sensor_trigger::trigger_second_motor() {}
int sensor_trigger::trigger_parachute() {
  std::shared_lock<std::shared_mutex> lock(_state.mutex);
    const tvc_data &tvc = _state.tvc_state; // Dereferencing the state to get the tvc_state and then checking the angles

    // Check to see if the angles exceed 45 degrees
    if (std::abs(tvc.euler_angles[0]) > CRITICAL_ANGLE || std::abs(tvc.euler_angles[1]) > CRITICAL_ANGLE) {
        this -> _parachute_shunt = true;  
        return 1;                 
    }

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
}

void ReactionWheelController::compute_pid(float currentAngle, float targetAngle, float& prevError, float& integral, int pwmPin) {
    float error = targetAngle - currentAngle;
    integral += error * TIME_STEP;
    float derivative = (error - prevError) / TIME_STEP;

    float output = _Kp * error + _Ki * integral + _Kd * derivative;

    int pwmValue = static_cast<int>(fabs(output));
    pwmValue = std::clamp(pwmValue, 0, 1024);

    if (output < 0) {
        pwmValue = 1024 - pwmValue;
    }

    pwmWrite(pwmPin, pwmValue);
    prevError = error;
}
