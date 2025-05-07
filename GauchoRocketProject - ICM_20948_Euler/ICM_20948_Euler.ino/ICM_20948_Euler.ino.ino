// === SPI VERSION of Example7_DMP_Quat6_EulerAngles.ino ===
// Make sure to uncomment "#define ICM_20948_USE_DMP" in ICM_20948_C.h

#include "ICM_20948.h"
#include <SPI.h>

#define USE_SPI
#define SERIAL_PORT Serial

// Using default VSPI pins:
// MOSI = GPIO23, MISO = GPIO19, SCLK = GPIO18, CS = GPIO5
#define CS_PIN 5

ICM_20948_SPI myICM;

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT);
  SERIAL_PORT.println(F("ICM-20948 SPI DMP Example"));

  delay(100);

  while (SERIAL_PORT.available()) SERIAL_PORT.read();
  SERIAL_PORT.println(F("Press any key to continue..."));
  while (!SERIAL_PORT.available());

  SPI.begin();  // Use default VSPI pins

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
    SERIAL_PORT.println(F("✅ DMP enabled!"));
  } else {
    SERIAL_PORT.println(F("❌ Failed to initialize DMP. Check ICM_20948_USE_DMP."));
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

      double roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
      double pitch = asin(constrain(2.0 * (qw * qy - qx * qz), -1.0, 1.0)) * 180.0 / PI;
      double yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / PI;

      SERIAL_PORT.print(F("Roll: ")); SERIAL_PORT.print(roll, 1);
      SERIAL_PORT.print(F(" Pitch: ")); SERIAL_PORT.print(pitch, 1);
      SERIAL_PORT.print(F(" Yaw: ")); SERIAL_PORT.println(yaw, 1);
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(10);
  }
}
