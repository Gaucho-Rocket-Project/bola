#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "constants.h"

struct tvc_data {
    float euler_angles[2];
    float angular_velocities[3];
    float sums[2];
};

struct state_data {
    tvc_data tvc_state;
    float height;
    bool parachute_shunt;
    float cs[2];
    Time start_time;
};


#endif