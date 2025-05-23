// === SPI VERSION with TVC + Reaction‐Wheel Control ===
// Make sure to uncomment "#define ICM_20948_USE_DMP" in ICM_20948_C.h
#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h" // Your ICM_20948 library header
#include <cmath> 
#include <iostream>
#include <utility> 
#include <vector>
#include <array> // added for lookup table

// --- SPI pins for VSPI (default) ---
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define CS_PIN 5 // Chip‐select for ICM-20948

// --- Reaction‐wheel ESC on GPIO27 ---
const int escPin = 27;
const int escFreq = 50; // 50 Hz for typical ESC PWM
const int escRes = 16;  // 16-bit PWM resolution

// --- TVC servos on two GPIOs ---
Servo servoX, servoY;
int xPin = 13;
int yPin = 12;

// --- PID constants for reaction wheel (yaw rate) ---
const float Kp_rw = 3.3125f, Ki_rw = 0.2f, Kd_rw = 1.3f;
float prevError_rw = 0.0f, integral_rw = 0.0f;
unsigned long prevTime_rw_micros = 0;

// --- PID constants for TVC (roll/pitch) ---
const float Kp_tvc = 1.5f;
const float Ki_tvc = 0.1f;
const float Kd_tvc = 0.05f; // START VERY LOW (e.g., 0.0) AND TUNE UP
const float TVC_TIME_STEP_TARGET = 0.01f;
const float tvc_deadzone = 2.0f;
const float LPF_BETA = 0.2f;

// Variables for the LPF-based TVC PID
float current_roll_lpf = 0.0f;      
float current_pitch_lpf = 0.0f;
float tvc_error_integral[2] = {0.0f, 0.0f};
float tvc_prev_error[2] = {0.0f, 0.0f};
unsigned long tvc_prev_time_micros = 0;

// TVC Limp Mode (Shutdown)
const float TVC_MAX_ANGLE_LIMIT = 30.0f; // Max filtered angle before TVC enters limp mode
const float TVC_RESET_ANGLE_LIMIT = 25.0f; // Angle below which TVC can exit limp mode
bool tvc_in_limp_mode = false;

// --- IMU object ---
ICM_20948_SPI imu;

// --- Helpers ---
uint32_t usToDuty(int us) {
  return (uint32_t)us * ((1UL << escRes) - 1) / 20000;
}

static float lpf(float prev_lpf_val, float current_measurement, float beta) {
  return beta * current_measurement + (1.0f - beta) * prev_lpf_val;
}

std::array<std::pair<int, int>, 163> lookupTable = {{
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


// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Setup starting...");

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);

  Serial.println("Initializing IMU DMP...");
  while (imu.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("IMU.begin failed; retrying...");
    delay(500);
  }
  if (imu.initializeDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: initializeDMP failed!"); while (1); }
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMPSensor failed!"); while (1); }
  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) { Serial.println("FATAL: setDMPODRrate failed!"); while (1); }
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableFIFO failed!"); while (1); }
  if (imu.enableDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMP failed!"); while (1); }
  if (imu.resetDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: resetDMP failed!"); while (1); }
  if (imu.resetFIFO() != ICM_20948_Stat_Ok) { Serial.println("FATAL: resetFIFO failed!"); while (1); }
  Serial.println("ICM-20948 DMP ready.");

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(xPin, 500, 2400);
  servoY.attach(yPin, 500, 2400);
  servoX.write(90);
  servoY.write(90);

  ledcAttach(escPin, escFreq, escRes);
  Serial.println("Arming Reaction Wheel ESC: Sending 1500us. Please wait ~5 seconds...");
  ledcWrite(escPin, usToDuty(1500));
  delay(5000);
  Serial.println("ESC presumed armed.");

  tvc_prev_time_micros = micros();
  prevTime_rw_micros = micros();
  Serial.println("Setup complete.");
}

// --- Main loop ---
void loop() {
  unsigned long loop_start_micros = micros();

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
    unsigned long current_rw_micros = micros();
    float dt_rw = (prevTime_rw_micros == 0) ? TVC_TIME_STEP_TARGET : static_cast<float>(current_rw_micros - prevTime_rw_micros) * 1e-6f;
    if (dt_rw <= 0.00001f) { dt_rw = TVC_TIME_STEP_TARGET; }
    prevTime_rw_micros = current_rw_micros;

    float error_rw = targetYawRate - yawRate;
    integral_rw += error_rw * dt_rw;
    float derivative_rw = (dt_rw > 0.00001f) ? (error_rw - prevError_rw) / dt_rw : 0.0f;
    prevError_rw = error_rw;
    float pid_output_rw = Kp_rw * error_rw + Ki_rw * integral_rw + Kd_rw * derivative_rw;
    int pulse_rw = static_cast<int>(round(1500.0f - pid_output_rw));
    pulse_rw = constrain(pulse_rw, 1000, 2000);
    ledcWrite(escPin, usToDuty(pulse_rw));
  } else if (tvc_in_limp_mode) {
    // If TVC is in limp mode, set reaction wheel to neutral for safety
    ledcWrite(escPin, usToDuty(1500));
    // Serial.println("Reaction Wheel Neutral due to TVC Limp Mode.");
  }


  // --- Maintain loop rate ---
  long loop_duration_micros = micros() - loop_start_micros;
  long delay_needed_micros = static_cast<long>(TVC_TIME_STEP_TARGET * 1e6f) - loop_duration_micros;
  if (delay_needed_micros > 0) {
    delayMicroseconds(delay_needed_micros);
  }
}