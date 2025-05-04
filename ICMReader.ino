#include <Arduino.h>
#include "imu_data.h"

// Define global imuData struct instance
ImuData imuData;

void setup() {
  Serial.begin(115200);
  initIMU();
}

void loop() {
  readIMU();

  // Print out current shared IMU data
  Serial.print("Roll: ");
  Serial.print(imuData.roll, 1);
  Serial.print(" Pitch: ");
  Serial.print(imuData.pitch, 1);
  Serial.print(" Yaw: ");
  Serial.println(imuData.yaw, 1);

  delay(100);
}
