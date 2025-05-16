#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h"
#include "BluetoothSerial.h"

// --- SPI pins for VSPI (default) ---
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define CS_PIN    5  // Chip‐select for ICM-20948

// --- Reaction‐wheel ESC on GPIO27 ---
const int  escPin  = 27;
const int  escFreq = 50;  // 50 Hz
const int  escRes  = 16;  // 16-bit

// --- TVC servos on two GPIOs ---
Servo servoX, servoY;
int xPin = 13;  // if no movement, try 25
int yPin = 12;  // or 26

// --- PID constants for reaction wheel (yaw rate) ---
const float Kp_rw = 3.3125, Ki_rw = 0.2, Kd_rw = 1.3;
float prevError = 0, integral = 0;
unsigned long prevTime = 0;

// --- PID constants for TVC (roll/pitch) ---
const float Kp_tvc = 3.3125, Ki_tvc = 0.2, Kd_tvc = 1.3;
const float TIME_STEP = 0.01;
float initial_I[2] = {0,0}, current_I[2];
float initial_ang[2] = {0,0}, current_ang[2];

// --- IMU object ---
ICM_20948_SPI imu;

// --- Helpers ---
uint32_t usToDuty(int us) {
  // maps [0..20000µs] to [0..(1<<escRes)-1]
  return (uint32_t)us * ((1UL<<escRes)-1) / 20000;
}

float calcI(int idx) {
  return initial_I[idx] + current_ang[idx]*TIME_STEP;
}

float calcD(int idx) {
  return (current_ang[idx] - initial_ang[idx]) / TIME_STEP;
}

float pidTVC(int idx) {
  current_I[idx] = calcI(idx);
  float D = calcD(idx);
  if (idx == 0)
    return Kp_tvc*current_ang[0] + Ki_tvc*current_I[0] + Kd_tvc*D;
  else
    return Kp_tvc*current_ang[1] + Ki_tvc*current_I[1] + Kd_tvc*D;
}

// ---------------------------------------------------------------------- //


// Globals

// Bluetooth
BluetoothSerial SerialBT;
char btCmd;
bool launchSequence = false;  // main logic flag

// Step 1 and 2 Globals
float yawRate;
const float CRITICAL_ANGLE = 45.0;  // Can be changed

// Step 5 Timing Globals
unsigned long   lastMotorTime = 0,
                motorInterval = 1000;  // ms between motor triggers
const int       legPin = 14;           // leg pin

// Function Declarations
void handleBT(); // Bluetooth Handling
void readAngle(); // Step 1
bool checkCriticalAngle(); // Step 2 (I'm not completely sure what this is)
void computeAndWritePID(); // Step 3 and 4
unsigned long readTime(); // Step 5
bool checkTime(unsigned long, unsigned long); // Step 6
void triggerMotor(); // Step 7
void triggerLegs(); // Step 8

// --- Setup ---
void setup() {
  Serial.begin(115200);
  while(!Serial);
  // start SPI for IMU
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);

  // init IMU DMP
  while (imu.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed; retrying...");
    delay(500);
  }
  if ( imu.initializeDMP()  != ICM_20948_Stat_Ok ||
       imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok ||
       imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok ||
       imu.enableFIFO() != ICM_20948_Stat_Ok ||
       imu.enableDMP() != ICM_20948_Stat_Ok ||
       imu.resetDMP()  != ICM_20948_Stat_Ok ||
       imu.resetFIFO() != ICM_20948_Stat_Ok ) {
    Serial.println("Failed to configure DMP");
    while(1);
  }
  Serial.println("ICM-20948 DMP ready");

  // --- Bluetooth ---
  SerialBT.begin("Esp32-BT");
  Serial.println("Bluetooth up: device name = Esp32-BT");


  // TVC servos
  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(xPin, 500, 2400);
  servoY.attach(yPin, 500, 2400);

  // Reaction-wheel ESC PWM
  ledcAttach(escPin, escFreq, escRes);
  // arm ESC neutral
  ledcWrite(escPin, usToDuty(1500));
  delay(5000);

  pinMode(legPin, OUTPUT);
  digitalWrite(legPin, LOW);

  prevTime = millis();
}

void loop() {
  handleBT();

  if (!launchSequence) {
    delay(50);
    return;
  }

  // read the angle
  readAngle();

  // check the critical angle
  // Please change this critical angle part I'm not completely sure what it's supposed to do and I couldn't find it in the repo
  if (checkCriticalAngle()) {
    Serial.println("Critical angle reached");

    // 1) Center TVC servos
    servoX.write(90);
    servoY.write(90);

    // 2) Neutralize reaction-wheel / main ESC
    ledcWrite(escPin, usToDuty(1500));
    return;
  }


  computeAndWritePID();

  // grab the current timestamp
  unsigned long now = readTime();

  // spin the motor
  if (checkTime(lastMotorTime, motorInterval)) {
    triggerMotor();
    lastMotorTime = now;
  }

  // fire leg actuators
  triggerLegs();

  delay(10);
}

// ===== FUNCTION DEFINITIONS =====

void handleBT() {
  if (SerialBT.available()) {
    btCmd = SerialBT.read();
    switch (btCmd) {
      case '1':
        Serial.println("BT: start launch sequence");
        launchSequence = true;
        break;
      case '0':
        Serial.println("BT: stop launch sequence");
        launchSequence = false;
        break;
    }
  }
}

void readAngle() {
  icm_20948_DMP_data_t d;
  // — only grab quaternion → roll/pitch —
  if ( imu.readDMPdataFromFIFO(&d)==ICM_20948_Stat_Ok
    && (d.header & DMP_header_bitmap_Quat6) ) {

    double q1 = double(d.Quat6.Data.Q1)/1073741824.0;
    double q2 = double(d.Quat6.Data.Q2)/1073741824.0;
    double q3 = double(d.Quat6.Data.Q3)/1073741824.0;
    double q0 = sqrt(1 - (q1*q1 + q2*q2 + q3*q3));

    // roll
    double t0 = 2*(q0*q2 + q1*q3);
    double t1 = 1-2*(q2*q2 + q3*q3);
    current_ang[0] = atan2(t0,t1) * 180/PI;

    // pitch
    double t2 = 2*(q0*q3 - q1*q2);
    t2 = constrain(t2,-1,1);
    current_ang[1] = asin(t2) * 180/PI;
  }

  // — then separately grab yaw rate —
  if ( imu.dataReady() ) {
    imu.getAGMT();
    yawRate = imu.gyrZ();   // store in your global
  }
}

bool checkCriticalAngle() {
  return fabs(current_ang[0]) > CRITICAL_ANGLE
      || fabs(current_ang[1]) > CRITICAL_ANGLE;
}

void computeAndWritePID() {
  // compute PID for each axis, map to [60..120]° around 90
  float outX = constrain(pidTVC(0), -30, 30) + 90;
  float outY = constrain(pidTVC(1), -30, 30) + 90;

  // write servos
  servoX.write(outX);
  servoY.write(outY);

  // update I/D state for next iteration
  for (int i = 0; i < 2; i++) {
    initial_I[i]   = current_I[i];
    initial_ang[i] = current_ang[i];
  }

  // — 2) Reaction-wheel PID (yaw rate) —
  const float target = 0.0;
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  float err = target - yawRate;
  integral += err * dt;
  float deriv = (err - prevError) / dt;
  prevError = err;

  float u = Kp_rw*err + Ki_rw*integral + Kd_rw*deriv;
  int pulse = constrain(1500 - int(u), 1000, 2000);
  ledcWrite(escPin, usToDuty(pulse));
}


unsigned long readTime() {
  return millis();
}

bool checkTime(unsigned long start, unsigned long interval) {
  return (millis() - start) >= interval;
}

void triggerMotor() {
  // e.g. ledcWrite(escPin, usToDuty(2000)); 
}

void triggerLegs() {
  // e.g. digitalWrite(legPin, HIGH); delay(100); digitalWrite(legPin, LOW);
}