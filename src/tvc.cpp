#include <iostream>
#include <ctime>
#include <chrono>
#include "constants.h"


struct TVC 
{
    float heading;
    Time startTime;

    float burnTime(Time initialBurnTime)
    {
        std::chrono::duration<double> duration = initialBurnTime - startTime; //need to know why duration is only defined if theres a math operation between initialBurnTime and startTime
        double timeInSeconds = duration.count();
        return timeInSeconds * ((2 * THRUST_FORCE)/(GRAVITY * TOTAL_ROCKET_MASS) - 1);
    }


};
// a variable, double angle, float, 