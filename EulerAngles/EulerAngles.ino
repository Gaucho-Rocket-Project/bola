// === SPI VERSION with TVC + Reaction‐Wheel Control (clean) ===
#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h"

/* --------------------   PINS & HW   ------------------------ */
#define SPI_SCLK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define CS_PIN     5      // ICM-20948 CS

const int escPin  = 27;   // reaction wheel ESC
const int escFreq = 50;   // 50 Hz
const int escRes  = 16;   // 16-bit PWM

Servo servoX, servoY;     // TVC
const int xPin = 13;
const int yPin = 12;

/* --------------------   CONSTANTS   ------------------------ */
const float Kp_rw  = 3.3,   Ki_rw  = 0.20, Kd_rw  = 1.3;  // yaw-rate PID
const float Kp_tvc = 3.3,   Ki_tvc = 0.20, Kd_tvc = 1.5;  // roll/pitch PID

const float DEADZONE_DEG = 2.0;   // no TVC movement ±2°
const float LPF_BETA     = 0.2;   // 0…1  (1 = no filtering)
const float INT_LIM_TVC  = 20;    // integral wind-up guard (deg·s)

/* --------------------   GLOBALS   -------------------------- */
ICM_20948_SPI imu;

/* --- TVC PID state --- */
struct AxisPID {
  float errI = 0;
  float prevErr = 0;
};
AxisPID pid[2];   // 0 = roll, 1 = pitch
unsigned long tvcPrevUs = 0;

/* --- Reaction-wheel PID state --- */
float rwErrI = 0, rwPrevErr = 0;
unsigned long rwPrevUs = 0;

/* --------------------   HELPERS   -------------------------- */
uint32_t usToDuty(int us)
{
  uint32_t dutyMax = (1UL << escRes) - 1;
  return (uint32_t)us * dutyMax / 20000;   // 20 000 µs = 50 Hz period
}

static float lpf(float prev, float meas)
{
  return LPF_BETA * meas + (1.0f - LPF_BETA) * prev;
}

/* ----------------------------------------------------------- */
void setup()
{
  Serial.begin(115200);
  while (!Serial);          // wait for console

  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);

  while (imu.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed, retrying…");
    delay(500);
  }
  if ( imu.initializeDMP()  != ICM_20948_Stat_Ok ||
       imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok ||
       imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok ||
       imu.enableFIFO() != ICM_20948_Stat_Ok ||
       imu.enableDMP()  != ICM_20948_Stat_Ok ||
       imu.resetDMP()   != ICM_20948_Stat_Ok ||
       imu.resetFIFO()  != ICM_20948_Stat_Ok ) {
    Serial.println("Failed to configure DMP – halt");
    while (1);
  }
  Serial.println("ICM-20948 DMP ready");

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(xPin, 500, 2400);
  servoY.attach(yPin, 500, 2400);

  ledcAttach(escPin, escFreq, escRes);
  ledcWrite (escPin, usToDuty(1500));   // neutral
  delay(1000);
}

/* -----------  TVC 1-axis controller (roll / pitch)  -------- */
int tvcPID(int axis, float angleDeg, float dt)
{
  float err = -angleDeg;                         // target = 0°

  /* dead-zone */
  if (fabs(err) < DEADZONE_DEG) {
    pid[axis].errI   = 0;
    pid[axis].prevErr = err;
    return 90;                                   // centre servo
  }

  /* P-I-D */
  pid[axis].errI += err * dt;
  pid[axis].errI = constrain(pid[axis].errI, -INT_LIM_TVC, INT_LIM_TVC);

  float d = (dt > 0) ? (err - pid[axis].prevErr) / dt : 0;
  pid[axis].prevErr = err;

  float u = Kp_tvc * err + Ki_tvc * pid[axis].errI + Kd_tvc * d;
  u = constrain(u, -30, 30);                      // mechanical travel

  return 90 + round(u);                           // map to servo cmd
}

/* --------------------   MAIN LOOP   ------------------------ */
void loop()
{
  /* -------------- 1.  TVC (roll/pitch) -------------------- */
  icm_20948_DMP_data_t d;
  if ( imu.readDMPdataFromFIFO(&d) == ICM_20948_Stat_Ok &&
       (d.header & DMP_header_bitmap_Quat6) )
  {
    /* quaternion → Euler (deg) */
    float q1 = (float)d.Quat6.Data.Q1 / 1073741824.0f;
    float q2 = (float)d.Quat6.Data.Q2 / 1073741824.0f;
    float q3 = (float)d.Quat6.Data.Q3 / 1073741824.0f;
    float q0 = sqrtf(1.0f - (q1*q1 + q2*q2 + q3*q3));

    float roll  = atan2f(2*(q0*q2 + q1*q3),
                         1 - 2*(q2*q2 + q3*q3)) * 180.0f/PI;
    float pitch = asinf(constrain(2*(q0*q3 - q1*q2), -1.0f, 1.0f))
                  * 180.0f/PI;

    /* low-pass */
    static float rollLP = 0, pitchLP = 0;
    rollLP  = lpf(rollLP , roll );
    pitchLP = lpf(pitchLP, pitch);

    /* Δt */
    unsigned long nowUs = micros();
    float dt = (tvcPrevUs == 0) ? 0.01f : (nowUs - tvcPrevUs) * 1e-6f;
    tvcPrevUs = nowUs;

    /* PID per axis */
    int cmdRoll  = tvcPID(0, rollLP , dt);
    int cmdPitch = tvcPID(1, pitchLP, dt);

    servoX.write(cmdRoll);
    servoY.write(cmdPitch);
  }

  /* -------------- 2.  Reaction-wheel (yaw rate) ----------- */
  if (imu.dataReady()) {
    imu.getAGMT();
    float yawRate = imu.gyrZ();                    // deg/s
    const float target = 0.0f;

    unsigned long nowUs = micros();
    float dt = (rwPrevUs == 0) ? 0.01f : (nowUs - rwPrevUs) * 1e-6f;
    rwPrevUs = nowUs;

    float err = target - yawRate;
    rwErrI += err * dt;
    rwErrI = constrain(rwErrI, -200.0f, 200.0f);   // wind-up guard

    float d = (dt > 0) ? (err - rwPrevErr) / dt : 0;
    rwPrevErr = err;

    float u = Kp_rw * err + Ki_rw * rwErrI + Kd_rw * d;
    int pulse = constrain(1500 - (int)u, 1000, 2000);
    ledcWrite(escPin, usToDuty(pulse));
  }
}
