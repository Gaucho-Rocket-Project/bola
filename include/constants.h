#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <chrono>

#ifdef USE_WIRINGPI
#include <wiringPi.h>
#else
#include "logger.h"  
#endif

#define GRAVITY 9.81
#define TOTAL_ROCKET_MASS 10
#define THRUST_FORCE 14.5
#define TIME_STEP 0.01
#define CRITICAL_ANGLE 45

enum led_state {
    NONE = 0,
    RED = 1,
    YELLOW = 2,
    GREEN = 3
};



constexpr int pyro_1 = 16;
constexpr int pyro_2 = 18;
constexpr int pyro_3 = 22;
constexpr int led_r = 26;
constexpr int led_g = 19;
constexpr int led_b = 13;

typedef std::chrono::time_point<std::chrono::system_clock> Time;

#endif

// make sure all time value uses that variable
//get the ghetters and setters,, heading time, formula for the burn time for the second ignition, is 
//header and c++ files, and also the definitions 
//these are private, anyting outside the class cannot access them, getter methods, simplest of them are float getheader(); return this--> heading;, setters are usually void void setHeading(float newheading)
// set this --> heading = new 
