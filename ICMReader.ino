#include <Arduino.h>
#include "ICM_20948.h"
#include <SPI.h>

// SPI pin definitions for ESP32
#define SPI_SCLK 25
#define SPI_MISO 34
#define SPI_MOSI 33
#define CS_PIN   26

#define SERIAL_PORT Serial

ICM_20948_SPI imu;  // IMU object

// struct to store IMU output
struct ImuData {
  double roll;
  double pitch;
  double yaw;
};

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);  // Wait for serial port to open

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);  // Start SPI with custom pins

  SERIAL_PORT.println("Initializing ICM-20948...");

  while (!imu.begin(CS_PIN, SPI)) {
    SERIAL_PORT.println("IMU not detected. Retrying...");
    delay(500);
  }

  bool success = true;
  success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
    SERIAL_PORT.println("DMP initialized successfully!");
  } else {
    SERIAL_PORT.println("Failed to initialize DMP. Halting.");
    while (1);
  }
}

void loop() {
  icm_20948_DMP_data_t data;
  imu.readDMPdataFromFIFO(&data);

  if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - (q1 * q1 + q2 * q2 + q3 * q3));

      double qw = q0, qx = q2, qy = q1, qz = -q3;

      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = atan2(t0, t1) * 180.0 / PI;

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = constrain(t2, -1.0, 1.0);
      double pitch = asin(t2) * 180.0 / PI;

      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      ImuData imuData = { roll, pitch, yaw };

      // Print the IMU data
      SERIAL_PORT.print("Roll: "); SERIAL_PORT.print(imuData.roll, 1);
      SERIAL_PORT.print(" | Pitch: "); SERIAL_PORT.print(imuData.pitch, 1);
      SERIAL_PORT.print(" | Yaw: "); SERIAL_PORT.println(imuData.yaw, 1);
    }
  }

  if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(10);
  }
}
