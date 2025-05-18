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
const float Kp_rw = 3.3125f, Ki_rw = 0.2f, Kd_rw = 1.3f;
float prevError_rw = 0.0f, integral_rw = 0.0f;
unsigned long prevTime_rw_micros = 0;

// --- PID constants for TVC (roll/pitch) ---
const float Kp_tvc = 1.5f;
const float Ki_tvc = 0.1f;
const float Kd_tvc = 0.05f;
const float TVC_TIME_STEP_TARGET = 0.01f;
const float tvc_deadzone = 2.0f;
const float LPF_BETA     = 0.2f;
float current_roll_lpf  = 0.0f;
float current_pitch_lpf = 0.0f;
float tvc_error_integral[2] = {0.0f, 0.0f};
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
  if (imu.initializeDMP() != ICM_20948_Stat_Ok)      { Serial.println("FATAL: initializeDMP failed!"); while(1); }
  if (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) { Serial.println("FATAL: enableDMPSensor failed!"); while(1); }
  if (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) != ICM_20948_Stat_Ok) { Serial.println("FATAL: setDMPODRrate failed!"); while(1); }
  if (imu.enableFIFO() != ICM_20948_Stat_Ok)       { Serial.println("FATAL: enableFIFO failed!"); while(1); }
  if (imu.enableDMP() != ICM_20948_Stat_Ok)        { Serial.println("FATAL: enableDMP failed!"); while(1); }
  if (imu.resetDMP() != ICM_20948_Stat_Ok)         { Serial.println("FATAL: resetDMP failed!"); while(1); }
  if (imu.resetFIFO() != ICM_20948_Stat_Ok)        { Serial.println("FATAL: resetFIFO failed!"); while(1); }
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

  tvc_prev_time_micros    = micros();
  prevTime_rw_micros      = micros();
  Serial.println("Setup complete.");
}

// --- Main loop ---
void loop() {
  unsigned long loop_start_micros = micros();

  // 1) TVC control via DMP quaternions
  icm_20948_DMP_data_t dmp_data;
  if (imu.readDMPdataFromFIFO(&dmp_data) == ICM_20948_Stat_Ok &&
      (dmp_data.header & DMP_header_bitmap_Quat6)) {

    // --- quaternion fixed-point → floats ---
    double q1 = (double)dmp_data.Quat6.Data.Q1 / 1073741824.0;
    double q2 = (double)dmp_data.Quat6.Data.Q2 / 1073741824.0;
    double q3 = (double)dmp_data.Quat6.Data.Q3 / 1073741824.0;
    double q_sum_sq = q1*q1 + q2*q2 + q3*q3;
    double q0 = (q_sum_sq < 1.0) ? sqrt(1.0 - q_sum_sq) : 0.0;

    // --- STANDARD roll (X-axis) extraction ---
    double t0 =  2.0*(q0*q1 + q2*q3);
    double t1 =  1.0 - 2.0*(q1*q1 + q2*q2);
    float current_roll_raw = atan2(t0, t1) * (180.0/PI);

    // --- STANDARD pitch (Y-axis) extraction ---
    double t2 = 2.0*(q0*q2 - q3*q1);
    // clamp to [-1,1]
    if (t2 >  1.0) t2 =  1.0;
    if (t2 < -1.0) t2 = -1.0;
    float current_pitch_raw = asin(t2) * (180.0/PI);

    // --- Low-pass filter ---
    current_roll_lpf  = lpf(current_roll_lpf,  current_roll_raw,  LPF_BETA);
    current_pitch_lpf = lpf(current_pitch_lpf, current_pitch_raw, LPF_BETA);

    // --- TVC PID to servo angles ---
    unsigned long current_tvc_micros = micros();
    float dt_tvc = (tvc_prev_time_micros == 0)
                  ? TVC_TIME_STEP_TARGET
                  : (current_tvc_micros - tvc_prev_time_micros)*1e-6f;
    if (dt_tvc <= 0.0f) dt_tvc = TVC_TIME_STEP_TARGET;
    tvc_prev_time_micros = current_tvc_micros;

    float servo_command_angle[2] = {90.0f, 90.0f};
    float tvc_error[2] = { -current_roll_lpf, -current_pitch_lpf };

    for (int axis=0; axis<2; ++axis) {
      if (fabs(tvc_error[axis]) < tvc_deadzone) {
        servo_command_angle[axis] = 90.0f;
        tvc_error_integral[axis] = 0.0f;
      } else {
        tvc_error_integral[axis] += tvc_error[axis]*dt_tvc;
        float derivative = (tvc_error[axis]-tvc_prev_error[axis])/dt_tvc;
        float pid = Kp_tvc*tvc_error[axis]
                  + Ki_tvc*tvc_error_integral[axis]
                  + Kd_tvc*derivative;
        pid = constrain(pid, -30.0f, 30.0f);
        servo_command_angle[axis] = 90.0f + round(pid);
      }
      tvc_prev_error[axis] = tvc_error[axis];
    }

    // --- send to servos ---
    servoX.write((int)servo_command_angle[0]);
    servoY.write((int)servo_command_angle[1]);
  }

  // 2) Reaction‐wheel yaw‐rate PID (unchanged)
  if (imu.dataReady()) {
    imu.getAGMT();
    float yawRate = imu.gyrZ();
    float targetYawRate = 0.0f;
    unsigned long now = micros();
    float dt_rw = (prevTime_rw_micros==0 ? TVC_TIME_STEP_TARGET
                   : (now - prevTime_rw_micros)*1e-6f);
    if (dt_rw <= 0.0f) dt_rw = TVC_TIME_STEP_TARGET;
    prevTime_rw_micros = now;

    float error_rw = targetYawRate - yawRate;
    integral_rw += error_rw * dt_rw;
    float derivative_rw = (error_rw - prevError_rw)/dt_rw;
    prevError_rw = error_rw;

    float pid_rw = Kp_rw*error_rw + Ki_rw*integral_rw + Kd_rw*derivative_rw;
    int pulse = round(1500.0f - pid_rw);
    pulse = constrain(pulse, 1000, 2000);
    ledcWrite(escPin, usToDuty(pulse));
  }

  // maintain ~100 Hz loop
  long dt_loop = micros() - loop_start_micros;
  long wait = (long)(TVC_TIME_STEP_TARGET*1e6f) - dt_loop;
  if (wait > 0) delayMicroseconds(wait);
}
