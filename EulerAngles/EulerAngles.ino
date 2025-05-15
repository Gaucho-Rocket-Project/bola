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
#define CS_PIN    5  // Chip‐select for ICM-20948

// --- Reaction‐wheel ESC on GPIO27 ---
const int  escPin  = 27;
const int  escFreq = 50;  // 50 Hz for typical ESC PWM
const int  escRes  = 16;  // 16-bit PWM resolution

// --- TVC servos on two GPIOs ---
Servo servoX, servoY;
int xPin = 13;
int yPin = 12;

// --- PID constants for reaction wheel (yaw rate) ---
const float Kp_rw = 3.3125f, Ki_rw = 0.2f, Kd_rw = 1.3f; // Added 'f' for float literals
float prevError_rw = 0.0f, integral_rw = 0.0f;
unsigned long prevTime_rw_micros = 0; // Renamed for clarity and using micros

// --- PID constants for TVC (roll/pitch) ---
// TUNING REQUIRED: Start with Kd_tvc much lower, e.g., 0.1f or even 0.0f initially
const float Kp_tvc = 1.5f;  // Reduced Kp for initial tuning
const float Ki_tvc = 0.1f;  // Reduced Ki for initial tuning
const float Kd_tvc = 0.05f; // Drastically reduced Kd for initial tuning
const float TVC_TIME_STEP_TARGET = 0.01f; // Target loop time for TVC in seconds (for ~100Hz)

const float tvc_deadzone = 2.0f; // Deadzone in degrees for TVC activation
const float LPF_BETA     = 0.2f; // LPF constant (lower = more smoothing, more lag)

// Variables for the LPF-based TVC PID
float current_roll_lpf  = 0.0f;
float current_pitch_lpf = 0.0f;
float tvc_error_integral[2] = {0.0f, 0.0f}; // [0] for roll, [1] for pitch
float tvc_prev_error[2]     = {0.0f, 0.0f};
unsigned long tvc_prev_time_micros = 0;

// --- IMU object ---
ICM_20948_SPI imu;

// --- Helpers ---
uint32_t usToDuty(int us) {
  return (uint32_t)us * ((1UL << escRes) - 1) / 20000;
}

static float lpf(float prev_lpf_val, float current_measurement, float beta) {
  return beta * current_measurement + (1.0f - beta) * prev_lpf_val;
}

// Optional: if you still want the applyCompensation behavior
float applyMinDeflection(float pid_correction, float min_deflection_threshold) {
  if (fabs(pid_correction) < min_deflection_threshold && pid_correction != 0.0f) {
    return (pid_correction > 0.0f) ? min_deflection_threshold : -min_deflection_threshold;
  }
  return pid_correction;
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
  // DMP Initialization with detailed checks
  if (imu.initializeDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: initializeDMP failed!"); while(1); }
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMPSensor failed!"); while(1); }
  // Set DMP ODR for Quat6 (Game Rotation Vector). reg_val=0 means max rate (225Hz or 1125Hz depending on gyro DLPF)
  // reg_val=1 -> ~112.5Hz for 225Hz base, reg_val=4 -> ~45Hz.
  // For a 100Hz loop (TIME_STEP 0.01s), ODR of ~112.5Hz (reg_val=1) or faster is good.
  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) { Serial.println("FATAL: setDMPODRrate failed!"); while(1); }
  if (imu.enableFIFO() != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableFIFO failed!"); while(1); }
  if (imu.enableDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMP failed!"); while(1); }
  if (imu.resetDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: resetDMP failed!"); while(1); }
  if (imu.resetFIFO() != ICM_20948_Stat_Ok) { Serial.println("FATAL: resetFIFO failed!"); while(1); }
  Serial.println("ICM-20948 DMP ready.");

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(xPin, 500, 2400); // Min/max pulse for 0-180 deg
  servoY.attach(yPin, 500, 2400);
  servoX.write(90); // Neutral position
  servoY.write(90);

  ledcAttach(escPin, escFreq, escRes);
  Serial.println("Arming Reaction Wheel ESC: Sending 1500us. Please wait ~5 seconds...");
  ledcWrite(escPin, usToDuty(1500));
  delay(5000); // CRITICAL: Longer delay for ESC arming
  Serial.println("ESC presumed armed.");

  tvc_prev_time_micros = micros();     // Initialize timer for TVC PID
  prevTime_rw_micros = micros(); // Initialize timer for Reaction Wheel PID
  Serial.println("Setup complete.");
}

// --- Main loop ---
void loop() {
  unsigned long loop_start_micros = micros(); // For main loop timing

  // 1) TVC control using DMP Game Rotation Vector (quaternions)
  icm_20948_DMP_data_t dmp_data;
  if (imu.readDMPdataFromFIFO(&dmp_data) == ICM_20948_Stat_Ok &&
      (dmp_data.header & DMP_header_bitmap_Quat6)) {

    double q1 = static_cast<double>(dmp_data.Quat6.Data.Q1) / 1073741824.0;
    double q2 = static_cast<double>(dmp_data.Quat6.Data.Q2) / 1073741824.0;
    double q3 = static_cast<double>(dmp_data.Quat6.Data.Q3) / 1073741824.0;
    
    // CRITICAL: Safe q0 calculation to prevent NaN
    double q_sum_sq = q1 * q1 + q2 * q2 + q3 * q3;
    double q0 = (q_sum_sq < 1.0) ? sqrt(1.0 - q_sum_sq) : 0.0;

    // Convert quaternion to Euler angles (Roll, Pitch) in degrees
    // These formulas are standard for a common aerospace convention (check your IMU frame)
    double t0_roll = 2.0 * (q0 * q1 + q2 * q3); // Note: Original was q0*q2+q1*q3 for roll, often q0*q1+q2*q3 is used for X-axis roll. Verify your axis definition. Assuming user's original was intended.
    double t1_roll = 1.0 - 2.0 * (q1 * q1 + q2 * q2); // Original was q2*q2+q3*q3. If q1 is roll axis, this is more standard.
                                                      // Sticking to user's original quaternion to Euler for now:
    t0_roll = 2.0 * (q0 * q2 + q1 * q3); // Roll calculation using q2, q3 (as per user's initial code)
    t1_roll = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    float current_roll_raw  = atan2(t0_roll, t1_roll) * (180.0 / PI);

    double t2_pitch = 2.0 * (q0 * q3 - q1 * q2); // Pitch calculation using q1, q3
    if (t2_pitch > 1.0) t2_pitch = 1.0;     // Clamp asin argument
    if (t2_pitch < -1.0) t2_pitch = -1.0;
    float current_pitch_raw = asin(t2_pitch) * (180.0 / PI);

    // --- Low-pass filter the raw angles ---
    current_roll_lpf  = lpf(current_roll_lpf,  current_roll_raw,  LPF_BETA);
    current_pitch_lpf = lpf(current_pitch_lpf, current_pitch_raw, LPF_BETA);

    // --- TVC PID Control (New LPF-based version) ---
    unsigned long current_tvc_micros = micros();
    float dt_tvc = (tvc_prev_time_micros == 0) ? TVC_TIME_STEP_TARGET : 
                   static_cast<float>(current_tvc_micros - tvc_prev_time_micros) * 1e-6f;
    if (dt_tvc <= 0.00001f) { // If dt is too small or first run with 0
        dt_tvc = TVC_TIME_STEP_TARGET; // Use target dt
    }
    tvc_prev_time_micros = current_tvc_micros;

    // Target angles are 0 degrees (stabilize to level)
    float tvc_error[2];
    tvc_error[0] = 0.0f - current_roll_lpf;  // Error for roll
    tvc_error[1] = 0.0f - current_pitch_lpf; // Error for pitch

    float servo_command_angle[2] = {90.0f, 90.0f}; // Default to neutral

    for (int axis = 0; axis < 2; ++axis) {
      if (fabs(tvc_error[axis]) < tvc_deadzone) {
        // Inside deadzone: command servo to neutral, reset integral for this axis
        servo_command_angle[axis] = 90.0f;
        tvc_error_integral[axis] = 0.0f; // Anti-windup
        // tvc_prev_error[axis] should still be updated with the current small error for next D-term calculation if it exits deadzone
      } else {
        // Outside deadzone: calculate PID
        tvc_error_integral[axis] += tvc_error[axis] * dt_tvc;
        // Optional: Clamp integral term
        // float max_integral = 50.0f; // Tune this value
        // tvc_error_integral[axis] = constrain(tvc_error_integral[axis], -max_integral, max_integral);

        float derivative = (dt_tvc > 0.00001f) ? (tvc_error[axis] - tvc_prev_error[axis]) / dt_tvc : 0.0f;
        
        float pid_correction = Kp_tvc * tvc_error[axis] + 
                               Ki_tvc * tvc_error_integral[axis] + 
                               Kd_tvc * derivative;

        // Optional: Apply minimum deflection if you want that behavior
        // pid_correction = applyMinDeflection(pid_correction, (axis == 0) ? 2.5f : 0.0f);


        // Constrain PID output to servo travel limits (e.g., +/- 30 degrees deflection)
        float max_deflection = 30.0f;
        pid_correction = constrain(pid_correction, -max_deflection, max_deflection);
        
        servo_command_angle[axis] = 90.0f + round(pid_correction);
      }
      tvc_prev_error[axis] = tvc_error[axis]; // Update previous error for next D-term calculation
    }
    
    servoX.write(static_cast<int>(servo_command_angle[0]));
    servoY.write(static_cast<int>(servo_command_angle[1]));

    // Serial debugging (uncomment what you need, print sparingly to avoid slowing loop)
    // Serial.print("RollRaw: "); Serial.print(current_roll_raw);
    // Serial.print(" RollLPF: "); Serial.print(current_roll_lpf);
    // Serial.print(" ErrRoll: "); Serial.print(tvc_error[0]);
    // Serial.print(" ServX: "); Serial.println(servo_command_angle[0]);
    // Similar for Pitch
  } // End of DMP data processing

  // 2) Reaction-wheel yaw‐rate PID
  if (imu.dataReady()) { // This checks for new raw sensor data, separate from DMP FIFO
    imu.getAGMT();
    float yawRate = imu.gyrZ(); // degrees/sec from raw gyro
    float targetYawRate = 0.0f;
    
    unsigned long current_rw_micros = micros();
    float dt_rw = (prevTime_rw_micros == 0) ? TVC_TIME_STEP_TARGET : // Use a default on first run
                  static_cast<float>(current_rw_micros - prevTime_rw_micros) * 1e-6f;
    if (dt_rw <= 0.00001f) {
        dt_rw = TVC_TIME_STEP_TARGET; // Fallback if dt is too small
    }
    prevTime_rw_micros = current_rw_micros;

    float error_rw = targetYawRate - yawRate;
    integral_rw += error_rw * dt_rw;
    // Optional: Clamp integral_rw
    // integral_rw = constrain(integral_rw, -some_max_rw_integral, some_max_rw_integral);
    
    float derivative_rw = (dt_rw > 0.00001f) ? (error_rw - prevError_rw) / dt_rw : 0.0f;
    prevError_rw = error_rw;

    float pid_output_rw = Kp_rw * error_rw + Ki_rw * integral_rw + Kd_rw * derivative_rw;
    
    // Map PID output to ESC pulse width. The sign depends on motor/prop direction.
    // If positive pid_output_rw means "need more speed", and 1000us is max speed, 2000us is min/reverse.
    int pulse_rw = static_cast<int>(round(1500.0f - pid_output_rw)); // Adjust sign if needed
    pulse_rw = constrain(pulse_rw, 1000, 2000);
    ledcWrite(escPin, usToDuty(pulse_rw));
  }

  // --- Maintain loop rate (approximately) ---
  long loop_duration_micros = micros() - loop_start_micros;
  long delay_needed_micros = static_cast<long>(TVC_TIME_STEP_TARGET * 1e6f) - loop_duration_micros;
  
  if (delay_needed_micros > 0) {
    delayMicroseconds(delay_needed_micros);
  }
  // else {
  //   Serial.println("Warning: Loop took longer than target time step.");
  // }
}
