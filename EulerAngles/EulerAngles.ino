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
int xPin = 13;  // if no movement, try 25
int yPin = 12;  // or 26

// --- PID constants for reaction wheel (yaw rate) ---
const float Kp_rw = 3.3125f, Ki_rw = 0.2f, Kd_rw = 1.3f;
float prevError_rw = 0.0f, integral_rw = 0.0f;
unsigned long prevTime_rw = 0;

// --- PID constants for TVC (roll/pitch) ---
const float Kp_tvc = 3.3125f, Ki_tvc = 0.2f, Kd_tvc = 1.3f;
const float TIME_STEP = 0.01f; // Target loop time in seconds (for 100Hz)
float initial_I[2] = {0.0f, 0.0f}, current_I[2]; // current_I is updated by pidTVC
float initial_ang[2] = {0.0f, 0.0f}, current_ang[2]; // current_ang is the error (actual angle reading)
const float deadzone = 2.0f; // Deadzone in degrees for TVC activation

// --- IMU object ---
ICM_20948_SPI imu;

// --- Helpers ---
uint32_t usToDuty(int us) {
  // maps [0..20000µs] (for 50Hz) to [0..(1<<escRes)-1]
  return (uint32_t)us * ((1UL << escRes) - 1) / 20000;
}

// calcD is used by pidTVC for the derivative term
float calcD(int idx) {
  // Calculates derivative of current_ang[idx]
  return (current_ang[idx] - initial_ang[idx]) / TIME_STEP;
}

// pidTVC calculates the raw PID correction value.
// It also updates the global current_I[idx] based on current_ang[idx].
float pidTVC(int idx) {
  // Integral term update for the current step
  current_I[idx] = initial_I[idx] + current_ang[idx] * TIME_STEP;
  
  // Derivative term for the current step
  float D = calcD(idx); // Uses current_ang[idx] and initial_ang[idx]
  
  // PID calculation: current_ang[idx] is the Proportional error term
  return Kp_tvc * current_ang[idx] + Ki_tvc * current_I[idx] + Kd_tvc * D;
}

// applyCompensation: if output magnitude is less than threshold (and non-zero),
// force it to be at least `threshold` magnitude.
float applyCompensation(float output, float threshold) {
  if (fabs(output) < threshold && output != 0.0f) {
    return (output > 0.0f) ? threshold : -threshold;
  }
  return output;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect (needed for some boards)

  Serial.println("Starting setup...");

  // Start SPI for IMU
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);

  // Initialize IMU and DMP
  Serial.println("Initializing IMU and DMP...");
  while (imu.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("IMU.begin failed; retrying...");
    delay(500);
  }
  Serial.println("IMU.begin successful.");

  if (imu.initializeDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: initializeDMP failed!"); while(1); }
  Serial.println("initializeDMP successful.");
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMPSensor failed!"); while(1); }
  Serial.println("enableDMPSensor successful for Game Rotation Vector.");
  
  // Set DMP Output Data Rate (ODR) for Quat6 (Game Rotation Vector)
  // DMP ODR for Quat6 = DMP_Sample_Rate / (1 + reg_val)
  // Assuming DMP_Sample_Rate is 225Hz (typical with DLPF enabled for gyro)
  reg_val = 0 -> 50Hz
  reg_val = 1 -> 50Hz (closer to 100Hz target loop rate / TIME_STEP = 0.01s)
  reg_val = 2 -> 50Hz
  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) { Serial.println("FATAL: setDMPODRrate failed!"); while(1); }
  Serial.println("setDMPODRrate successful (target ~112.5Hz for Quat6).");

  if (imu.enableFIFO() != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableFIFO failed!"); while(1); }
  Serial.println("enableFIFO successful.");
  if (imu.enableDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMP failed!"); while(1); }
  Serial.println("enableDMP successful.");
  if (imu.resetDMP() != ICM_20948_Stat_Ok) { Serial.println("FATAL: resetDMP failed!"); while(1); }
  Serial.println("resetDMP successful.");
  if (imu.resetFIFO() != ICM_20948_Stat_Ok) { Serial.println("FATAL: resetFIFO failed!"); while(1); }
  Serial.println("resetFIFO successful.");
  
  Serial.println("ICM-20948 DMP ready.");

  // TVC servos
  servoX.setPeriodHertz(50); // Standard servo PWM frequency
  servoY.setPeriodHertz(50);
  // Min/max pulse widths for 0-180 degrees for ESP32Servo. Adjust if needed.
  servoX.attach(xPin, 500, 2400); 
  servoY.attach(yPin, 500, 2400);
  servoX.write(90); // Start servos at neutral position
  servoY.write(90);

  // Reaction-wheel ESC PWM setup
  ledcAttach(escPin, escFreq, escRes);
  ledcWrite(escPin, usToDuty(1500)); // Send neutral signal (1500µs)
  Serial.println("ESC arming: sending 1500us. Please wait ~5 seconds...");
  delay(5000); // Delay for ESC to arm (adjust as per ESC manual)
  Serial.println("ESC presumed armed.");

  prevTime_rw = millis(); // Initialize timer for reaction wheel PID
  Serial.println("Setup complete. Starting main loop...");
}

// --- Main loop ---
void loop() {
  unsigned long loopStartTime = millis();

  // 1) TVC control using DMP Game Rotation Vector (quaternions)
  icm_20948_DMP_data_t dmp_data;
  // Check if new DMP data is available from the FIFO
  if (imu.readDMPdataFromFIFO(&dmp_data) == ICM_20948_Stat_Ok) {
    // Check if the data packet includes the Game Rotation Vector (Quat6)
    if ((dmp_data.header & DMP_header_bitmap_Quat6)) {
      // Extract quaternion components. The DMP output is scaled by 2^30.
      double q1 = static_cast<double>(dmp_data.Quat6.Data.Q1) / 1073741824.0; // Qx
      double q2 = static_cast<double>(dmp_data.Quat6.Data.Q2) / 1073741824.0; // Qy
      double q3 = static_cast<double>(dmp_data.Quat6.Data.Q3) / 1073741824.0; // Qz
      
      // Calculate q0 (Qw) from q1, q2, q3: q0 = sqrt(1 - (q1^2 + q2^2 + q3^2))
      // Ensure the argument to sqrt is non-negative due to potential floating point inaccuracies.
      double q_sum_sq = q1 * q1 + q2 * q2 + q3 * q3;
      double q0 = (q_sum_sq < 1.0) ? sqrt(1.0 - q_sum_sq) : 0.0;

      // Convert quaternion to Euler angles (Roll, Pitch) in degrees
      // These formulas depend on the IMU's orientation and the desired vehicle frame.
      // Using the original formulas provided by the user:
      double t0_roll = 2.0 * (q0 * q2 + q1 * q3);
      double t1_roll = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
      current_ang[0] = atan2(t0_roll, t1_roll) * (180.0 / PI); // Assigned to Roll (X-axis TVC)

      double t2_pitch = 2.0 * (q0 * q3 - q1 * q2);
      // Constrain t2_pitch to avoid domain errors with asin
      if (t2_pitch > 1.0) t2_pitch = 1.0;
      if (t2_pitch < -1.0) t2_pitch = -1.0;
      current_ang[1] = asin(t2_pitch) * (180.0 / PI);        // Assigned to Pitch (Y-axis TVC)

      // Calculate raw PID corrections. pidTVC updates global current_I[] as a side effect.
      float pid_x_correction = pidTVC(0); // For axis 0 (e.g., Roll)
      float pid_y_correction = pidTVC(1); // For axis 1 (e.g., Pitch)

      float servo_x_target_angle_float;
      float servo_y_target_angle_float;

      // X-axis (Roll) TVC logic with deadzone
      if (fabs(current_ang[0]) < deadzone) {
        servo_x_target_angle_float = 90.0f; // Neutral position
        current_I[0] = initial_I[0];       // Prevent integral windup by not accumulating I-term this step
      } else {
        // Outside deadzone: apply compensation and calculate servo angle
        float compensated_pid_x = applyCompensation(pid_x_correction, 0.5f);
        servo_x_target_angle_float = constrain(compensated_pid_x, -30.0f, 30.0f) + 90.0f;
      }
      servoX.write(static_cast<int>(round(servo_x_target_angle_float)));

      // Y-axis (Pitch) TVC logic with deadzone
      if (fabs(current_ang[1]) < deadzone) {
        servo_y_target_angle_float = 90.0f; // Neutral position
        current_I[1] = initial_I[1];       // Prevent integral windup
      } else {
        // Outside deadzone: apply compensation (threshold 0.0f for Y does nothing here)
        float compensated_pid_y = applyCompensation(pid_y_correction, 5.0f); 
        servo_y_target_angle_float = constrain(compensated_pid_y, -30.0f, 30.0f) + 90.0f;
      }
      servoY.write(static_cast<int>(round(servo_y_target_angle_float)));

      // Update PID state variables for the next iteration
      for (int i = 0; i < 2; i++) {
        initial_I[i] = current_I[i];     // initial_I for next step is current_I (which might have been reset if in deadzone)
        initial_ang[i] = current_ang[i]; // initial_ang for next step's D-term is the raw current angle
      }
    } // end if (dmp_data.header & DMP_header_bitmap_Quat6)
  } // end if (imu.readDMPdataFromFIFO)

  // 2) Reaction-wheel yaw‐rate PID
  // This part uses raw gyro data. imu.dataReady() typically signals new raw sensor data.
  if (imu.dataReady()) { 
    imu.getAGMT();       // Fetch Accel, Gyro, Mag, Temp data
    float yawRate = imu.gyrZ(); // Raw gyro Z-axis data in degrees/sec
    float targetYawRate = 0.0f; // Target zero yaw rate
    
    unsigned long currentTime_rw = millis();
    float dt_rw = static_cast<float>(currentTime_rw - prevTime_rw) / 1000.0f;
    prevTime_rw = currentTime_rw;

    // Prevent division by zero or huge derivative if dt_rw is too small (e.g., first loop or consecutive calls)
    if (dt_rw <= 0.0001f) { // if dt is too small, can skip PID or use default dt
        dt_rw = TIME_STEP; // Or simply return/skip update for this iteration
    }

    float error_rw = targetYawRate - yawRate;
    integral_rw += error_rw * dt_rw;
    // Optional: Add integral clamping for reaction wheel to prevent extreme windup
    // float max_integral_rw = 50.0f; // Example value, tune as needed
    // integral_rw = constrain(integral_rw, -max_integral_rw, max_integral_rw); 
    
    float derivative_rw = (error_rw - prevError_rw) / dt_rw;
    prevError_rw = error_rw;

    float output_rw = Kp_rw * error_rw + Ki_rw * integral_rw + Kd_rw * derivative_rw;
    
    // Map PID output to ESC pulse width (e.g., 1000-2000 µs, centered at 1500 µs)
    // The sign of output_rw's effect depends on motor direction and ESC calibration.
    // Assuming positive output_rw means yawRate is too low (or negative), so need to speed up motor (decrease pulse for typical RC ESCs)
    // Or if positive output_rw means yawRate is too high (positive), need to slow down motor (increase pulse)
    // Let's assume: a positive 'error_rw' (target - actual > 0) means actual yawRate is negative or less than target.
    // To correct this, if reaction wheel spins CCW to produce CW rocket yaw, we might need to speed it up.
    // The term `1500 - int(u)` in original implies positive `u` increases speed (reduces pulse width).
    int pulse_rw = static_cast<int>(round(1500.0f - output_rw)); 
    pulse_rw = constrain(pulse_rw, 1000, 2000); // Ensure pulse is within ESC's operating range
    ledcWrite(escPin, usToDuty(pulse_rw));
  }

  // Ensure the main loop runs at approximately the desired frequency (e.g., 100Hz for TIME_STEP = 0.01s)
  long loopProcessingTime = millis() - loopStartTime;
  if (loopProcessingTime < (TIME_STEP * 1000.0f)) {
    delay((TIME_STEP * 1000.0f) - loopProcessingTime);
  } else {
    // Optional: Print a warning if the loop is taking too long
    // Serial.print("Warning: Loop took "); Serial.print(loopProcessingTime); Serial.println(" ms");
  }
}
