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

// -- MoI values --1
const float I_rocket = 0.002395f; 
const float I_wheel = ;


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

const std::array<std::pair<int, int>, 163> lookupTable = {{
    { -24, 0 }, { -24, 1 }, { -24, 2 }, { -24, 3 }, { -24, 4 },
    { -23, 5 }, { -23, 6 }, { -23, 7 }, { -23, 8 }, { -23, 9 },
    { -22, 10 }, { -22, 11 }, { -22, 12 }, { -22, 13 }, { -22, 14 },
    { -21, 15 }, { -21, 16 }, { -21, 17 }, { -21, 18 }, { -21, 19 },
    { -20, 20 }, { -20, 21 }, { -20, 22 }, { -20, 23 }, { -20, 24 },
    { -19, 25 }, { -19, 26 }, { -19, 27 }, { -19, 28 }, { -19, 29 },
    { -18, 30 }, { -18, 31 }, { -18, 32 }, { -18, 33 }, { -18, 34 },
    { -17, 35 }, { -17, 36 }, { -17, 37 }, { -17, 38 }, { -17, 39 },
    { -16, 40 }, { -16, 41 }, { -16, 42 }, { -16, 43 }, { -16, 44 },
    { -15, 45 }, { -15, 46 }, { -15, 47 }, { -15, 48 }, { -15, 49 },
    { -14, 50 }, { -13, 51 }, { -13, 52 }, { -13, 53 }, { -12, 54 },
    { -12, 55 }, { -12, 56 }, { -11, 57 }, { -11, 58 }, { -11, 59 },
    { -11, 60 }, { -10, 61 }, { -10, 62 }, { -10, 63 }, { -9, 64 },
    { -9, 65 }, { -9, 66 }, { -8, 67 }, { -8, 68 }, { -8, 69 },
    { -7, 70 }, { -7, 71 }, { -7, 72 }, { -6, 73 }, { -6, 74 },
    { -5, 75 }, { -5, 76 }, { -5, 77 }, { -4, 78 }, { -4, 79 },
    { -4, 80 }, { -3, 81 }, { -3, 82 }, { -3, 83 }, { -2, 84 },
    { -2, 85 }, { -2, 86 }, { -1, 87 }, { -1, 88 }, { -1, 89 },
    { 0, 90 }, { 0, 91 }, { 0, 92 }, { 1, 93 }, { 1, 94 },
    { 1, 95 }, { 2, 96 }, { 2, 97 }, { 3, 98 }, { 3, 99 },
    { 4, 100 }, { 4, 101 }, { 4, 102 }, { 4, 103 }, { 4, 104 },
    { 5, 105 }, { 5, 106 }, { 5, 107 }, { 6, 108 }, { 6, 109 },
    { 6, 110 }, { 7, 111 }, { 7, 112 }, { 7, 113 }, { 8, 114 },
    { 8, 115 }, { 8, 116 }, { 8, 117 }, { 9, 118 }, { 9, 119 },
    { 10, 120 }, { 10, 121 }, { 10, 122 }, { 10, 123 }, { 10, 124 },
    { 11, 125 }, { 11, 126 }, { 11, 127 }, { 12, 128 }, { 12, 129 },
    { 12, 130 }, { 13, 131 }, { 13, 132 }, { 13, 133 }, { 13, 134 },
    { 14, 135 }, { 14, 136 }, { 14, 137 }, { 14, 138 }, { 14, 139 },
    { 15, 140 }, { 15, 141 }, { 15, 142 }, { 15, 143 }, { 15, 144 },
    { 15, 145 }, { 15, 146 }, { 15, 147 }, { 15, 148 }, { 15, 149 },
    { 16, 150 }, { 16, 151 }, { 16, 152 }, { 16, 153 }, { 16, 154 },
    { 16, 155 }, { 16, 156 }, { 16, 157 }, { 16, 158 }, { 16, 159 },
    { 16, 160 }, { 16, 161 }, { 16, 162 }
}};

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

  // fire the 2nd motor
  if (checkTime(lastMotorTime, motorInterval)) {
    triggerMotor();
    lastMotorTime = now;
    // fire leg actuators
    triggerLegs();
  }


    // --- Maintain loop rate ---
  long loop_duration_micros = micros() - loop_start_micros;
  long delay_needed_micros = static_cast<long>(TVC_TIME_STEP_TARGET * 1e6f) - loop_duration_micros;
  if (delay_needed_micros > 0) {
    delayMicroseconds(delay_needed_micros);
  }
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
  // 1) TVC control using DMP Game Rotation Vector
  icm_20948_DMP_data_t dmp_data;
  if (imu.readDMPdataFromFIFO(&dmp_data) == ICM_20948_Stat_Ok &&
      (dmp_data.header & DMP_header_bitmap_Quat6)) {

    double q1 = static_cast<double>(dmp_data.Quat6.Data.Q1) / 1073741824.0; // X-axis rotation component
    double q2 = static_cast<double>(dmp_data.Quat6.Data.Q2) / 1073741824.0; // Y-axis rotation component
    double q3 = static_cast<double>(dmp_data.Quat6.Data.Q3) / 1073741824.0; // Z-axis rotation component
    
    double q_sum_sq = q1 * q1 + q2 * q2 + q3 * q3;
    double q0 = (q_sum_sq < 1.0) ? sqrt(1.0 - q_sum_sq) : 0.0;

    // ---- CHOOSE AND USE ONLY ONE EULER ANGLE CONVERSION ----
    // Standard Euler Angle Convention (Confirm your IMU axis mapping to Roll/Pitch vehicle axes)
    // Roll (around IMU X-axis / vehicle's longitudinal axis)
    double t0_roll = 2.0 * (q0 * q1 + q2 * q3);
    double t1_roll = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    float current_roll_raw = atan2(t0_roll, t1_roll) * (180.0 / PI);

    // Pitch (around IMU Y-axis / vehicle's transverse axis)
    // A common formulation for pitch from quaternion: asin(2.0 * (q0*q2 - q1*q3))
    double t2_pitch = 2.0 * (q0 * q2 - q3 * q1); 
    if (t2_pitch > 1.0)  t2_pitch = 1.0;
    if (t2_pitch < -1.0) t2_pitch = -1.0;
    float current_pitch_raw = asin(t2_pitch) * (180.0 / PI);
    // ---- END OF EULER ANGLE CONVERSION ----

    current_roll_lpf = lpf(current_roll_lpf, current_roll_raw, LPF_BETA);
    current_pitch_lpf = lpf(current_pitch_lpf, current_pitch_raw, LPF_BETA);

    // ---------- TVC Limp Mode Logic ------------------------------
    if (!tvc_in_limp_mode &&
        (fabs(current_roll_lpf) > TVC_MAX_ANGLE_LIMIT ||
         fabs(current_pitch_lpf) > TVC_MAX_ANGLE_LIMIT)) {
      Serial.println("!!! TVC Entering LIMP MODE: Angle limit exceeded !!!");
      tvc_in_limp_mode = true;
      servoX.write(90); // Go to neutral
      servoY.write(90);
      tvc_error_integral[0] = 0.0f; // Reset PID state
      tvc_error_integral[1] = 0.0f;
      tvc_prev_error[0] = 0.0f;     // Reset previous error for D term
      tvc_prev_error[1] = 0.0f;
    }

    if (tvc_in_limp_mode &&
        fabs(current_roll_lpf) < TVC_RESET_ANGLE_LIMIT &&
        fabs(current_pitch_lpf) < TVC_RESET_ANGLE_LIMIT) {
      Serial.println("TVC Exiting LIMP MODE: Angles back in range.");
      tvc_in_limp_mode = false;
      // PID state (integrals, prev_errors) will naturally rebuild on next active PID cycle
    }

    // --- TVC PID Control ---
    if (!tvc_in_limp_mode) {
      unsigned long current_tvc_micros = micros();
      float dt_tvc = (tvc_prev_time_micros == 0) ? TVC_TIME_STEP_TARGET : static_cast<float>(current_tvc_micros - tvc_prev_time_micros) * 1e-6f;
      if (dt_tvc <= 0.00001f) { dt_tvc = TVC_TIME_STEP_TARGET; }
      tvc_prev_time_micros = current_tvc_micros;

      float tvc_error[2];
      tvc_error[0] = 0.0f - current_roll_lpf;
      tvc_error[1] = 0.0f - current_pitch_lpf;
      
      float servo_command_angle_calculated[2] = {90.0f, 90.0f}; // Temporary for calculation

      for (int axis = 0; axis < 2; ++axis) {
        if (fabs(tvc_error[axis]) < tvc_deadzone) {
          servo_command_angle_calculated[axis] = 90.0f;
          tvc_error_integral[axis] = 0.0f;
        } else {
          tvc_error_integral[axis] += tvc_error[axis] * dt_tvc;
          // Optional: Clamp tvc_error_integral[axis]
          float derivative = (dt_tvc > 0.00001f) ? (tvc_error[axis] - tvc_prev_error[axis]) / dt_tvc : 0.0f;
          float pid_correction = Kp_tvc * tvc_error[axis] +
                                 Ki_tvc * tvc_error_integral[axis] +
                                 Kd_tvc * derivative;
          float max_deflection = 30.0f;
          pid_correction = constrain(pid_correction, -max_deflection, max_deflection);
          servo_command_angle_calculated[axis] = 90.0f + round(pid_correction);
        }
        tvc_prev_error[axis] = tvc_error[axis];
      }
      servoX.write(static_cast<int>(servo_command_angle_calculated[0]));
      servoY.write(static_cast<int>(servo_command_angle_calculated[1]));

      Serial.print("ACTIVE Roll: "); Serial.print(current_roll_lpf, 1);
      Serial.print(", Pitch: "); Serial.print(current_pitch_lpf, 1);
      Serial.print(" | ServoX: "); Serial.print(servo_command_angle_calculated[0], 1);
      Serial.print(", ServoY: "); Serial.println(servo_command_angle_calculated[1], 1);

    } else { // TVC is in LIMP MODE
      // Servos should already be at 90 from when limp mode was entered.
      // This block ensures they stay there if no other logic writes to them.
      servoX.write(90);
      servoY.write(90);
      Serial.print("LIMP MODE Roll: "); Serial.print(current_roll_lpf, 1);
      Serial.print(", Pitch: "); Serial.print(current_pitch_lpf, 1);
      Serial.println(" | Servos at Neutral.");
    }
  } // End of DMP data processing

  // 2) Reaction-wheel yaw‐rate PID
  // Consider if reaction wheel should also be affected by tvc_in_limp_mode
  if (!tvc_in_limp_mode && imu.dataReady()) { // Only run RW PID if TVC is not in limp mode
    imu.getAGMT();
    float yawRate = imu.gyrZ();
    float targetYawRate = 0.0f;
    
    // — 2) Reaction-wheel linear function —
    float u = (yawRate*I_rocket)/(I_wheel);
    int pulse = constrain(1500 - int(u), 1000, 2000);
    ledcWrite(escPin, usToDuty(pulse));

  } else if (tvc_in_limp_mode) {
    ledcWrite(escPin, usToDuty(1500)); // If TVC is in limp mode, set reaction wheel to neutral for safety
    // Serial.println("Reaction Wheel Neutral due to TVC Limp Mode.");
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
}


//right now 70 degrees is index 0 in the lookup table (can shift offset if needed)
int getTheta4(int theta2) {
  int offset = 70;
  int index; 
  if (theta2 > 110) {
    index = 110 - offset;
  } else if (theta2 < 70) {
    index = 70 - offset;
  } else {
    index = theta2 - offset;
  }
  return lookupTable[index].second;
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