// === SPI VERSION with TVC + Reaction‐Wheel Control ===
// Make sure to uncomment "#define ICM_20948_USE_DMP" in ICM_20948_C.h
#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h" // Your ICM_20948 library header
#include <cmath>       // For fabs, sqrt, atan2, asin, round

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
    // The one you had (q0*q2 - q3*q1) is also a valid rotation sequence.
    // Ensure it matches your physical setup and desired response.
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