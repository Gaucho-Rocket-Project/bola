// === SPI VERSION of Example7_DMP_Quat6_EulerAngles.ino ===
// Make sure to uncomment "#define ICM_20948_USE_DMP" in ICM_20948_C.h

#include "ICM_20948.h"
#include <SPI.h>
#include <ESP32Servo.h>

#define USE_SPI               
// Enable SPI
#define SERIAL_PORT Serial

// Custom SPI pin definitions based on updated request
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define CS_PIN 5              
// Chip Select connected to GPIO26

// which sample are we on?
size_t sampleIndex = 0;
//time step
const float TIME_STEP = 0.01;

//Servos
Servo servoX;
Servo servoY;

//Servo pins (BASED OFF GPIO numbers)
int xPin = 13;
int yPin = 12;

//data struct (not used yet)
struct testStruct{
  float roll;
  float pitch;
  float yaw;
};

//PID constants
float Kp;
float Ki;
float Kd;
float initial_I[2];//x,y
float current_I[2]; //x,y
float initial_angles[2]; //phi,theta
float current_angles[2]; //phi,theta

//helper functions
float calculateI(float I0, float current_angle);
float calculateD(float angle1, float angle2);
float alpha(float Ix, float Dx, float current_phi);
float beta(float Iy, float Dy, float current_theta);

//DEFINTIONS OF HELPER FUNCTIONS

float alpha(){
  //calculate current integration value
  current_I[0] = calculateI(initial_I[0], current_angles[0]);
  //calculate current derivative value
  float Dx = calculateD(initial_angles[0],current_angles[0]);
  //calculate PID sum
  return Kp * current_angles[0] + Ki * current_I[0] + Kd * Dx;
}

float beta(){
  //calculate current integration value
  current_I[1] = calculateI(initial_I[1], current_angles[1]);
  //calculate current derivative value
  float Dy = calculateD(initial_angles[1],current_angles[1]);
  //calculate PID sum
  return Kp * current_angles[1] + Ki * current_I[1] + Kd * Dy;
}

float calculateI(float I0, float current_angle){
  return I0 + current_angle * TIME_STEP;
}

float calculateD(float angle1, float angle2){
  return (angle2 - angle1)/TIME_STEP;
}

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


    // put your setup code here, to run once:
  servoX.setPeriodHertz(50);  // typical servo frequency
  int test1 = servoX.attach(xPin, 500, 2400); // pin, min/max pulse width in µs
  servoY.setPeriodHertz(50);
  int test2 = servoY.attach(yPin, 500, 2400);
  if(test1 == 0){
    Serial.println("X Servo didn't attach");
  }
  else{
    Serial.println("Attached X Servo");
  }
  if(test2 == 0){
    Serial.println("Y Servo didn't attach");
  }
  else{
    Serial.println("Attached Y Servo");
  }
  // start integral terms at zero
  initial_I[0] = initial_I[1] = 0.0f;

  // take your “previous angle” to be the very first sample
  initial_angles[0] = 0.0;
  initial_angles[1] = 0.0;
  
  // (you had Kp, Ki, Kd already set)  
  // initial_I[0] = initial_angles[0];
  // initial_I[1] = initial_angles[1];
  Kp = 3.3125; 
  Ki = 0.2; 
  Kd = 1.3;
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


      // 1) load the new “measured” angles
      current_angles[0] = roll;
      current_angles[1] = pitch;

      // 2) compute PID outputs
      float outX = alpha()+96;
      float outY = beta()+86;

      // 3a) actually drive servos
      servoX.write(outX);
      servoY.write(outY);

      // 4) update “previous” state for next iteration
      initial_I[0]      = current_I[0];
      initial_I[1]      = current_I[1];
      initial_angles[0] = current_angles[0];
      initial_angles[1] = current_angles[1];

      sampleIndex++;

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) {
    delay(10);
  }
  }
  }
}
