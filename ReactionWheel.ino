#include <Wire.h>
#include <BluetoothSerial.h>
#include <ArduinoOTA.h> // For OTA updates
//https://www.youtube.com/watch?v=MHxw4yaNKVw&t=179s og youtube video, check comments for ideas
#define MPU6050 0x68
#define MOTOR 9
#define DIR 4
#define MOTOR_PWM_CHANNEL 0
#define MOTOR_PWM_FREQ 5000
#define MOTOR_PWM_RES 8

BluetoothSerial SerialBT;

// PID parameters (tune K, others are fractions)
float K_outer = 100;
float Ki_outer = 0.05; // as fraction of K_outer
float Kd_outer = 0.1;  // as fraction of K_outer

float K_inner = 50;
float Ki_inner = 0.02; // as fraction of K_inner
float Kd_inner = 0.05; // as fraction of K_inner

float loop_time = 10; // ms

// IMU offsets and state
int16_t AcX_offset = -750, AcY_offset = 360, AcZ_offset = 0, GyZ_offset = 0;
int32_t GyZ_offset_sum = 0;
int16_t AcX, AcY, AcZ, GyZ;
float target_angle = 0, current_angle = 0, acc_angle = 0;
float target_rate = 0, current_rate = 0;
bool vertical = false;

// PID state
float errorSum_outer = 0, lastError_outer = 0;
float errorSum_inner = 0, lastError_inner = 0;
long current_time, previous_time = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(MOTOR, OUTPUT);
  pinMode(DIR, OUTPUT);
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, 255);

  // OTA setup
  ArduinoOTA.begin();

  angle_setup();
}

void loop() {
  ArduinoOTA.handle(); // OTA handler

  current_time = millis();
  checkWebBluetooth(); // Placeholder for web-based PID tuning

  if (current_time - previous_time >= loop_time) {
    angle_calc();

    // Outer PID: Angle -> Target Rate
    float error_angle = current_angle - target_angle;
    errorSum_outer += error_angle;
    float dError_angle = (error_angle - lastError_outer) / (loop_time / 1000.0);
    target_rate = -K_outer * (1.0 * error_angle + Ki_outer * errorSum_outer + Kd_outer * dError_angle);
    lastError_outer = error_angle;

    // Inner PID: Rate -> Motor PWM
    float error_rate = current_rate - target_rate;
    errorSum_inner += error_rate;
    float dError_rate = (error_rate - lastError_inner) / (loop_time / 1000.0);
    int motor_pwm = constrain(K_inner * (1.0 * error_rate + Ki_inner * errorSum_inner + Kd_inner * dError_rate), -255, 255);
    lastError_inner = error_rate;

    // Motor control
    if (motor_pwm > 0) digitalWrite(DIR, LOW);
    else digitalWrite(DIR, HIGH);
    ledcWrite(MOTOR_PWM_CHANNEL, abs(motor_pwm));

    previous_time = current_time;
  }
}

// --- IMU and PID helper functions remain mostly unchanged ---

void angle_setup() {
  delay(100);
  writeTo(MPU6050, 0x6B, 0);
  writeTo(MPU6050, 0x1C, 0); // 2g
  writeTo(MPU6050, 0x1B, 8); // 500dps
  delay(100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
}

void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  GyZ = Wire.read() << 8 | Wire.read();

  AcX += AcX_offset;
  AcY += AcY_offset;
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;

  current_rate = GyZ / 65.536; // deg/s
  current_angle += current_rate * loop_time / 1000.0;
  acc_angle = atan2(AcY, -AcX) * 57.2958;
  current_angle = current_angle * 0.996 + acc_angle * 0.004;

  if (abs(current_angle) > 12) vertical = false;
  if (abs(current_angle) < 0.3) vertical = true;
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}
// Call this in setup()
void setupBluetooth() {
    SerialBT.begin("ESP32_PID_Tuner"); // Bluetooth device name
}

// Call this in loop()
void checkWebBluetooth() {
    if (SerialBT.available()) {
        String input = SerialBT.readStringUntil('\n');
        input.trim();
        // Expected format: OUTER:K,Ki,Kd;INNER:K,Ki,Kd
        // Example: OUTER:120,0.06,0.12;INNER:60,0.03,0.07
        int outerIdx = input.indexOf("OUTER:");
        int innerIdx = input.indexOf("INNER:");
        if (outerIdx != -1 && innerIdx != -1) {
            String outerParams = input.substring(outerIdx + 6, input.indexOf(';', outerIdx));
            String innerParams = input.substring(innerIdx + 6);

            int k1 = outerParams.indexOf(',');
            int k2 = outerParams.lastIndexOf(',');
            if (k1 != -1 && k2 != -1 && k1 != k2) {
                K_outer = outerParams.substring(0, k1).toFloat();
                Ki_outer = outerParams.substring(k1 + 1, k2).toFloat();
                Kd_outer = outerParams.substring(k2 + 1).toFloat();
            }

            k1 = innerParams.indexOf(',');
            k2 = innerParams.lastIndexOf(',');
            if (k1 != -1 && k2 != -1 && k1 != k2) {
                K_inner = innerParams.substring(0, k1).toFloat();
                Ki_inner = innerParams.substring(k1 + 1, k2).toFloat();
                Kd_inner = innerParams.substring(k2 + 1).toFloat();
            }
            Serial.println("PID parameters updated via Bluetooth.");
        }
    }
}