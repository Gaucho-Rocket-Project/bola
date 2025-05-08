#include <EEPROM.h>
#include "BluetoothSerial.h"
#include <FastLED.h>
#include <Wire.h>

#define USE_BT      1

#define BUZZER      4
#define VBAT        32
#define INT_LED     2

#define ESC_PIN     18   // PWM output pin for ESC signal
#define ESC_FREQ    50   // 50Hz for RC ESC
#define ESC_RES     16   // 16-bit resolution

#define MPU6050_ADDR  0x68
#define EEPROM_SIZE  64

#define LED_PIN      18
#define NUM_PIXELS   3

CRGB leds[NUM_PIXELS];

#if (USE_BT)
  BluetoothSerial SerialBT;
#endif  

int bat_divider = 198;

int16_t  AcX, AcY, AcZ, GyZ;
float robot_angle, angle_offset, gyroZfilt;
float Acc_angle;            

bool vertical = false;      
bool calibrating = false;
bool calibrated = false;
int calibrating_step = 1;

float target_voltage = 0;
float Gyro_amount = 0.996;
float alpha = 0.3;
int loop_time = 8;

struct AccOffsetsObj {
  int ID;
  int16_t X;
  int16_t Y;
  float off1;
  float off2;
};
AccOffsetsObj offsets;

// Helper: microseconds to duty cycle
uint32_t usToDuty(int us) {
  return (uint32_t(us) * ((1UL << ESC_RES) - 1)) / 20000;
}

void setup() {
  Serial.begin(115200);
  #if (USE_BT)
    SerialBT.begin("ESP32ReactionWheel");
  #endif  

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, offsets);
  calibrated = (offsets.ID == 24);

  pinMode(BUZZER, OUTPUT);
  pinMode(INT_LED, OUTPUT);

  // Setup ESC PWM
  ledcAttach(ESC_PIN, ESC_FREQ, ESC_RES);

  // Arm ESC at neutral
  ledcWrite(ESC_PIN, usToDuty(1500));
  delay(5000); // let ESC arm at neutral

  // ... FastLED and other setup code as before ...
  Wire.begin();
  // ... MPU6050 init if needed ...
  // ... LED startup animation as before ...
  // angle_setup(); // if you want to calibrate
}

void setESC(float voltage) {
  // Map your control output to ESC pulse width (1000-2000us)
  int pulse_us = map(constrain(voltage, -12, 12), -12, 12, 1000, 2000);
  ledcWrite(ESC_PIN, usToDuty(pulse_us));
}

void angle_calc() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  GyZ = Wire.read() << 8 | Wire.read();

  Acc_angle = atan2((float)AcY, (float)AcZ) * 180.0 / PI;

  static float last_angle = 0;
  static unsigned long last_time = 0;
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;

  float gyro_rate = (float)GyZ / 131.0;
  if (dt <= 0 || dt > 0.2) dt = 0.01;

  robot_angle = Gyro_amount * (last_angle + gyro_rate * dt) + (1.0 - Gyro_amount) * Acc_angle;
  last_angle = robot_angle;
}

void loop() {
  angle_calc(); // Update robot_angle and gyroZfilt

  // --- Balancing controller logic ---
  // Example: simple proportional controller (replace with your own)
  // Setpoint is 0 degrees (upright)
  float Kp = 50.0; // Tune this value!
  float setpoint = 0.0;
  float error = setpoint - (robot_angle + angle_offset);

  // Optionally add gyro feedback for damping (PD controller)
  float Kd = 1.0; // Tune this value!
  float control = Kp * error - Kd * gyroZfilt;

  // Map control output to ESC PWM range (1000–2000us)
  int pwmVal = map(constrain(control, -12, 12), -12, 12, 1000, 2000);

  // Optional: dead zone around neutral
  const int deadZone = 25;
  if (abs(pwmVal - 1500) <= deadZone) {
    pwmVal = 1500;
  }

  // Output to ESC
  ledcWrite(ESC_PIN, usToDuty(pwmVal));

  // Debugging
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" Gyro: ");
  Serial.print(gyroZfilt);
  Serial.print(" PWM: ");
  Serial.println(pwmVal);
  Serial.print("\n");

  delay(loop_time);
}