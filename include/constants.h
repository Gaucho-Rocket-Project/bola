#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <chrono>

#define RED 1
#define YELLOW 2
#define GREEN 3

#define LED_B 13
#define LED_G 19
#define LED_R 26

#define PYRO_1 16
#define PYRO_2 18
#define PYRO_3 22
#define GRAVITY 9.81
#define TOTAL_ROCKET_MASS 10
#define THRUST_FORCE 14.5
#define TIME_STEP 0.01

typedef std::chrono::time_point<std::chrono::system_clock> Time;

#endif

// make sure all time value uses that variable
//get the ghetters and setters,, heading time, formula for the burn time for the second ignition, is 
//header and c++ files, and also the definitions 
//these are private, anyting outside the class cannot access them, getter methods, simplest of them are float getheader(); return this--> heading;, setters are usually void void setHeading(float newheading)
// set this --> heading = new 
