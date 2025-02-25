#include <cmath> // added
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>

#include "constants.h"

float pwmOutput(double angle) { return 318.31 * angle + 1000; }

double pwmWrite(double startTime, double endTime, int sampleRate) {
  // uint16_t means unsigned and only 16 bits
  uint16_t numPoints = (endTime - startTime) * sampleRate;
  float dt = (endTime - startTime) / numPoints;
  double time[numPoints];
  double acceleration[numPoints];

  // fill time array with equidistant points
  for (uint16_t i = 0; i < numPoints; i++) {
    time[i] = startTime +
              i * dt; // equidistant time points from start time to end time
    acceleration[i] = std::pow(time[i], 3); // a(t) = t^3
  }

  double angularVelocity = 0;

  // use trapezoidal rule to find the area under all the points
  for (uint16_t i = 0; i + 1 < numPoints; i++) {
    angularVelocity += dt * (acceleration[i] + acceleration[i + 1]) / 2;
  }

  return angularVelocity;
}