#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "bmp5.h"
#include "constants.h"
#include "icm20948_api.h"

struct tvc_data {
  float euler_angles[2];
  float angular_velocities[3];
  float angle_summations[2];
  float cs[2];
};

struct state_data {
  tvc_data tvc_state;
  float height;
  Time start_time;
};

class sensor_module {
  icm20948_gyro_t gyro_data;
  icm20948_accel_t accelerometer_data;
  state_data &mux;
  sensor_trigger *trigger;

public:
  sensor_module(state_data &mux);

  ~sensor_module();

  void update_euler_angles();
  void update_height();
};

class sensor_trigger {

  bool parachute_shunt;

public:
  sensor_trigger();

  int trigger_landing_legs();
  int trigger_second_motor();
  int trigger_parachute();
};

class tvc {
  state_data &mux;

public:
  tvc(state_data &mux);
};

#endif