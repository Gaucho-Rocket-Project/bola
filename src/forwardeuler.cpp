#include <iostream>

// equation to estimate
double velocityFunction(double x) {
	return (x * x * x * x) / 4;
}

double ForwardEulerMethod(double startTime, double endTime, double yInitial, double stepSize) {
	// calculate the number of steps
	int numSteps = static_cast<int>((endTime - startTime) / stepSize);

	// initialize the result
	double result = yInitial;
	double currentTime = startTime; // initialize the time in between each step

	// compute euler estimation
	for (int i = 0; i < numSteps; i++) {
		result += velocityFunction(currentTime) * stepSize;
		currentTime += stepSize;
	}

	return result;
}

int main() {

	std::cout << "estimated value " << ForwardEulerMethod(0, 1, 1.2, 0.2) << std::endl;
	return 0;
}