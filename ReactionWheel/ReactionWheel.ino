#include <Wire.h>
#include <ArduinoOTA.h> // For OTA updates
#define MPU6050 0x68
#define accSens 0
#define gyroSens 1
#define Gyro_amount 0.996
#define MOTOR 9
#define DIR 4

float Kp = 100;
float Ki = 4.8;
float Kd = 9.7;
float loop_time = 10;

int16_t  AcX_offset = -750;
int16_t  AcY_offset = 360;
int16_t  AcZ_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;
int16_t AcX, AcY, AcZ, GyZ, gyroZ;

float target_angle = 0;
float current_angle;
float acc_angle;
bool vertical = false;
float errorSum = 0;
float lastError = 0;
long current_time, previous_time = 0;
uint8_t i2cData[14];

void setup() {
    delay(1000);
    angle_setup();
    Serial.begin(115200); // Faster debug output

    pinMode(MOTOR, OUTPUT);
    pinMode(DIR, OUTPUT);
    analogWrite(MOTOR, 255);

    // OTA setup
    ArduinoOTA.setHostname("esp32-reactionwheel");
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle(); // Call this as often as possible for responsiveness

    current_time = millis();
    checkBluetooth();
    if (current_time - previous_time >= loop_time) {
        angle_calc();
        if (vertical) {
            float error = current_angle - target_angle;
            errorSum += error;
            errorSum = constrain(errorSum, -80, 80);
            float derivative = (error - lastError) / (loop_time/1000);
            int motor_pwm = constrain(Kp * error + Ki * errorSum + Kd * derivative, -255, 255);

            if (motor_pwm > 0) {
                digitalWrite(DIR, LOW);
            } else {
                digitalWrite(DIR, HIGH);
            }
            analogWrite(MOTOR, 1-abs(motor_pwm));

            lastError = error;
            previous_time = current_time;
        } else {
            target_angle = 0;
            analogWrite(MOTOR, 255);
            errorSum = 0;
        }
    }
}

// ... rest of your functions remain unchanged ...
void writeTo(byte device, byte address, byte value) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
}

void angle_setup() {
    Wire.begin();
    delay(100);
    writeTo(MPU6050, 0x6B, 0);
    writeTo(MPU6050, 0x1C, accSens << 3);
    writeTo(MPU6050, 0x1B, gyroSens << 3);
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
    Wire.requestFrom((uint8_t)MPU6050, (size_t)4, (bool)true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MPU6050);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050, (size_t)2, (bool)true);
    GyZ = Wire.read() << 8 | Wire.read();

    AcX += AcX_offset;
    AcY += AcY_offset;
    AcZ += AcZ_offset;
    GyZ -= GyZ_offset;

    current_angle += GyZ * loop_time / 1000 / 65.536;
    acc_angle = atan2(AcY, -AcX) * 57.2958;
    current_angle = current_angle * Gyro_amount + acc_angle * (1.0 - Gyro_amount);

    if (abs(current_angle) > 12) vertical = false;
    if (abs(current_angle) < 0.3) vertical = true;
}

void checkBluetooth(){
    while (Serial.available()) {
        int data = Serial.read();
        if (data == 70) {
            Kp += 1;
        }else if(data == 66) {
            Kp -= 1;
        }else if(data == 76){
            Ki -= 0.1;
        }else if(data == 82){
            Ki += 0.1;
        }else if(data == 88){
            Kd += 0.1;
        }else if(data == 89){
            Kd -= 0.1;
        }

        Serial.print("KP: ");
        Serial.print(Kp);
        Serial.print(" KI: ");
        Serial.print(Ki);
        Serial.print(" KD: ");
        Serial.println(Kd);
    }
}
