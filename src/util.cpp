#include <iostream>
#include <unistd.h>
#include <cmath>
#include <thread>
#include <shared_mutex>
#include "constants.h"
#include "util.h"

std::shared_mutex sharedMutex;

// @param: intial_burn_time
Time calculateSecondMotorActivation(Time initial_burn_time, float motor_acceleration, float upward_drag_coefficient, float downward_drag_coefficient) {
	std::shared_lock<std::shared_mutex> lock(sharedMutex);
	// @todo: write function
	Time placeholder;
	return placeholder;
	// mutex automatically unlocks when function ends
}

double ForwardEulerMethod(double startTime, double endTime, double yInitial) {
	std::shared_lock<std::shared_mutex> lock(sharedMutex);
	// calculate the number of steps
	int numSteps = static_cast<int>((endTime - startTime) / TIME_STEP);

	// initialize the result
	double result = yInitial;
	double currentTime = startTime; // initialize the time in between each step

	// compute euler estimation
	for (int i = 0; i < numSteps; i++) {
		// result += velocityFunction(currentTime) * TIME_STEP;
		currentTime += TIME_STEP;
	}

	return result;
	// mutex automatically unlocks at end of function
}