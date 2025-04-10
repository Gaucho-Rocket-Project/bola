#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "bmp5.h"
#include "constants.h"
#include "icm20948_api.h"

#include <shared_mutex>
#include <thread>

struct tvc_data {
  float euler_angles[2];
  float angular_velocities[3];
  float angle_summations[2];
  float cs[2];

  public:
  void update_control_signal(state_data &state);
  state_data &_state;
};

struct state_data {
  tvc_data tvc_state;
  float height;
  Time start_time;
  mutable std::shared_mutex mutex;
};

class sensor_trigger {

  bool _parachute_shunt;

public:
  sensor_trigger(state_data &state);
  state_data &_state;

  int trigger_landing_legs();
  int trigger_second_motor();
  int trigger_parachute();
};

class sensor_module {
  icm20948_gyro_t _gyro_data;
  icm20948_accel_t _accelerometer_data;
  state_data &_state;
  sensor_trigger *_trigger;

public:
  sensor_module(state_data &state);

  ~sensor_module();

  void update_euler_angles();
  void update_height();
  void log_height(float height);
};  

class tvc {
  state_data &_state;

public:
  tvc(state_data &state);
};

class statemachine {

    state_data &_state;
    tvc *tvc;
    std::thread* threads;


    public:

    statemachine();
};


#endif