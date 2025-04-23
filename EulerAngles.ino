// === SPI VERSION of Example7_DMP_Quat6_EulerAngles.ino ===
// Make sure to uncomment "#define ICM_20948_USE_DMP" in ICM_20948_C.h

#include "ICM_20948.h"
#include <SPI.h>

#define USE_SPI               
// Enable SPI
#define SERIAL_PORT Serial

// Custom SPI pin definitions based on updated request
#define SPI_SCLK 25
#define SPI_MISO 34
#define SPI_MOSI 33
#define CS_PIN 26              
// Chip Select connected to GPIO26

ICM_20948_SPI myICM;          // IMU object for SPI

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);
  SERIAL_PORT.println(F("ICM-20948 SPI DMP Example"));

  delay(100);

  // Wait for key press
  while (SERIAL_PORT.available()) SERIAL_PORT.read();
  SERIAL_PORT.println(F("Press any key to continue..."));
  while (!SERIAL_PORT.available());

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);  // Custom SPI pins

  bool initialized = false;
  while (!initialized) {
    myICM.begin(CS_PIN, SPI);
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());

    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    } else {
      initialized = true;
    }
  }

  SERIAL_PORT.println(F("Device connected!"));

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
    SERIAL_PORT.println(F(" DMP enabled!"));
  } else {
    SERIAL_PORT.println(F(" Failed to initialize DMP. Check ICM_20948_USE_DMP."));
    while (1);
  }
}

void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = atan2(t0, t1) * 180.0 / PI;

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      SERIAL_PORT.print(F("Roll: ")); SERIAL_PORT.print(roll, 1);
      SERIAL_PORT.print(F(" Pitch: ")); SERIAL_PORT.print(pitch, 1);
      SERIAL_PORT.print(F(" Yaw: ")); SERIAL_PORT.println(yaw, 1);
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(10);
  }
}
