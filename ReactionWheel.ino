#include <SimpleFOC.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include <FastLED.h>

#define USE_BT      1

#define BUZZER      4
#define VBAT        32
#define INT_LED     2

#define MPU6050_ADDR  0x68     // Device address
#define ACCEL_CONFIG  0x1C     // Accelerometer configuration address
#define GYRO_CONFIG   0x1B     // Gyro configuration address

#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define accSens       0        // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens      1        // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
float Gyro_amount = 0.996;     // percent of gyro in complementary filter
float alpha = 0.3;             

#define EEPROM_SIZE  64

#define LED_PIN      18   // ESP32 pin that connects to WS2812B
#define NUM_PIXELS   3    // The number of LEDs (pixels) on WS2812B

CRGB leds[NUM_PIXELS];

// encoder instance
Encoder sensor = Encoder(16, 17, 1024);

// Interrupt routine intialisation channel A and B callbacks
void doA() {
  sensor.handleA();
}
void doB() {
  sensor.handleB();
}
void Tuning() {
    // TODO: Implement tuning logic or leave empty for now
  }
  
  float controller(float angle, float gyro, float speed, float position) {
    // TODO: Implement your controller logic
    return 0.0;
  }
BLDCMotor motor = BLDCMotor(7); 
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 23);

//float target_velocity = 0;              // Enter "T+speed" in the serial monitor to make the two motors rotate in closed loop
// include commander interface
//Commander command = Commander(Serial);
//void doMotor(char* cmd) { 
//  command.scalar(&target_velocity, cmd); 
//}

#if (USE_BT)
  BluetoothSerial SerialBT;
#endif  

long dt, currentT, previousT_1, previousT_2, previousT_3;  
int bat_divider = 198;              // must be tuned to measure battery voltage correctly

int16_t  AcX, AcY, AcZ, AcXc, AcYc, GyZ, gyroZ;

struct AccOffsetsObj {
  int ID;
  int16_t X;
  int16_t Y;
  float off1;
  float off2;
};
AccOffsetsObj offsets;

int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;

float robot_angle, angle_offset, gyroZfilt;
float Acc_angle;            

bool vertical = false;      
bool calibrating = false;
bool calibrated = false;
int calibrating_step = 1;

float robot_speed = 0, robot_position = 0;
float target_voltage = 0;

float K1Gain = 21.0; 
float K2Gain = 6.5; 
float K3Gain = 2.1; 
float K4Gain = 0.15; 
int loop_time = 8;

int up_led, up_led_dir;
int led_calibrating_step = 0;
// Placeholder for angle_setup function
void angle_setup() {
    // This function calibrates the angle offsets for two positions.
    // Place the robot in the first calibration position and press a button or send a command to continue.
    // Repeat for the second position.
  
    Serial.println("Starting angle calibration...");
    #if (USE_BT)
      SerialBT.println("Starting angle calibration...");
    #endif
  
    calibrating = true;
    calibrating_step = 1;
  
    // Calibrate first position (e.g., left side)
    Serial.println("Place the robot in the first calibration position and press ENTER.");
    #if (USE_BT)
      SerialBT.println("Place the robot in the first calibration position and press ENTER.");
    #endif
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read(); // Clear buffer
  
    // Read IMU and calculate offset for first position
    offsets.off1 = robot_angle;
    Serial.print("First offset recorded: "); Serial.println(offsets.off1);
  
    calibrating_step = 2;
  
    // Calibrate second position (e.g., right side)
    Serial.println("Place the robot in the second calibration position and press ENTER.");
    #if (USE_BT)
      SerialBT.println("Place the robot in the second calibration position and press ENTER.");
    #endif
    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read(); // Clear buffer
  
    // Read IMU and calculate offset for second position
    offsets.off2 = robot_angle;
    Serial.print("Second offset recorded: "); Serial.println(offsets.off2);
  
    // Save offsets to EEPROM
    offsets.ID = 24; // Mark as calibrated
    EEPROM.put(0, offsets);
    EEPROM.commit();
  
    calibrating = false;
    calibrated = true;
    calibrating_step = 0;
  
    Serial.println("Calibration complete!");
    #if (USE_BT)
      SerialBT.println("Calibration complete!");
    #endif
  }
void setup() {

  Serial.begin(115200);
  #if (USE_BT)
    SerialBT.begin("ESP32ReactionWheel");    // Bluetooth device name
  #endif  
  
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, offsets);
  
  if (offsets.ID == 24) calibrated = true;
    else calibrated = false;

  pinMode(BUZZER, OUTPUT);
  pinMode(INT_LED, OUTPUT);
  
  delay(500);

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_PIXELS); 
  
  for (int i=0;i<=255;i++) {
    leds[0] = CRGB(i, 0, 0);
    leds[1] = CRGB(i, 0, 0);
    leds[2] = CRGB(i, 0, 0);
    FastLED.show();
    delay(5);
  }
  delay(500);
  for (int i=0;i<=255;i++) {
    leds[0] = CRGB(0, i, 0);
    leds[1] = CRGB(0, i, 0);
    leds[2] = CRGB(0, i, 0);
    FastLED.show();
    delay(5);
  }
  delay(500);
  for (int i=0;i<=255;i++) {
    leds[0] = CRGB(0, 0, i);
    leds[1] = CRGB(0, 0, i);
    leds[2] = CRGB(0, 0, i);
    FastLED.show();
    delay(5);
  }
  delay(500);
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();

  delay(500);
  
  sensor.quadrature = Quadrature::ON;
  sensor.init();
  sensor.enableInterrupts(doA, doB);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;

  //PID Settings
  motor.PID_velocity.P = 0.3f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  
  motor.voltage_limit = 12; 

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  
  //Speed Low-pass Filter Time Constant
  motor.LPF_velocity.Tf = 0.06f;

  //Maximum Speed Limit Settings
  motor.velocity_limit = 70;

  motor.useMonitoring(Serial);

  //Initialize the Motor
  motor.init();
 
  //Initialize FOC
  motor.initFOC();

  digitalWrite(INT_LED, HIGH);
  delay(200);
  digitalWrite(INT_LED, LOW);
  delay(500);
  digitalWrite(INT_LED, HIGH);
  delay(500);
  digitalWrite(INT_LED, LOW);
  
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  angle_setup();
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(70);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}
// Placeholder for angle_calc function
void angle_calc() {
    // Read raw accelerometer and gyro data from MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Starting register for Accel data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
  
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47); // Starting register for Gyro Z data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    GyZ = Wire.read() << 8 | Wire.read();
  
    // Calculate the angle from accelerometer (in degrees)
    Acc_angle = atan2((float)AcY, (float)AcZ) * 180.0 / PI;
  
    // Integrate gyro Z to estimate angle change (degrees)
    static float last_angle = 0;
    static unsigned long last_time = 0;
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    last_time = now;
  
    float gyro_rate = (float)GyZ / 131.0; // for 500dps sensitivity
    if (dt <= 0 || dt > 0.2) dt = 0.01;   // avoid large jumps
  
    // Complementary filter
    robot_angle = Gyro_amount * (last_angle + gyro_rate * dt) + (1.0 - Gyro_amount) * Acc_angle;
    last_angle = robot_angle;
  }
  // Placeholder for batteryVoltage function
  void batteryVoltage(double voltage) {
    // Print voltage to Serial and Bluetooth (if enabled)
    Serial.print("Battery Voltage: ");
    Serial.println(voltage, 2);
    #if (USE_BT)
      SerialBT.print("Battery Voltage: ");
      SerialBT.println(voltage, 2);
    #endif
  
    // Low voltage warning threshold (adjust as needed)
    const double LOW_VOLTAGE = 7.0;
    if (voltage < LOW_VOLTAGE) {
      // Trigger buzzer warning
      digitalWrite(BUZZER, HIGH);
      delay(100);
      digitalWrite(BUZZER, LOW);
      delay(100);
    }
  }
void loop() {
    angle_calc();
  currentT = millis();
  
  if (currentT - previousT_1 >= loop_time) {
    angle_calc();
    if (robot_angle < -90 && robot_angle > -140) 
      angle_offset = offsets.off1;
    else if (robot_angle > -35 && robot_angle < 35) 
      angle_offset = 0; 
    else if (robot_angle > 90 && robot_angle < 140) 
      angle_offset = offsets.off2;    
    if (abs(robot_angle + angle_offset) > 8) vertical = false;
    if (abs(robot_angle + angle_offset) < 1) vertical = true;
    gyroZ = GyZ / 131.0; 
    gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
    #if (USE_BT)
      Tuning();  
    #endif  
      
    if (vertical && calibrated && !calibrating) { 
      target_voltage = controller(robot_angle + angle_offset, gyroZfilt, robot_speed, robot_position);
      robot_speed = motor.shaftVelocity();
      robot_position += robot_speed / 30;
      robot_position = constrain(robot_position, -210, 210);
      motor.move(target_voltage);
    } else {
        if (abs(target_voltage) <= 0.3) target_voltage = 0;
        if (target_voltage > 0) target_voltage -= 0.3;
        if (target_voltage < 0) target_voltage += 0.3;
        motor.move(target_voltage);
        robot_position = 0;
        robot_speed = 0;
    }
  previousT_1 = currentT;
  }

  motor.loopFOC(); 
  
  if (currentT - previousT_3 >= 20) {
    if (vertical && calibrated && !calibrating) { 
        if (up_led >= 255) 
          up_led_dir = 0;
        else if (up_led <= 0) 
          up_led_dir = 1; 
        if (up_led_dir == 1) 
          up_led += 5;
        else 
          up_led -= 5;
        if (abs(robot_angle) >= 0 && abs(robot_angle) <= 1)  
          leds[1] = CRGB(up_led, 0, 0);
        else if (abs(robot_angle) > 1 && abs(robot_angle) <= 2.5) 
          leds[1] = CRGB(up_led, up_led, 0);
        else if (abs(robot_angle) > 2.5 && abs(robot_angle) <= 8) 
          leds[1] = CRGB(0, up_led, 0);
        else if (abs(robot_angle + offsets.off2) >= 0 && abs(robot_angle + offsets.off2) <= 1)  
          leds[2] = CRGB(up_led, 0, 0);
        else if (abs(robot_angle + offsets.off2) > 1 && abs(robot_angle + offsets.off2) <= 2.5) 
          leds[2] = CRGB(up_led, up_led, 0);
        else if (abs(robot_angle + offsets.off2) > 2.5 && abs(robot_angle + offsets.off2) <= 8) 
          leds[2] = CRGB(0, up_led, 0);
         else if (abs(robot_angle + offsets.off1) >= 0 && abs(robot_angle + offsets.off1) <= 1)  
          leds[0] = CRGB(up_led, 0, 0);
        else if (abs(robot_angle + offsets.off1) > 1 && abs(robot_angle + offsets.off1) <= 2.5) 
          leds[0] = CRGB(up_led, up_led, 0);
        else if (abs(robot_angle + offsets.off1) > 2.5 && abs(robot_angle + offsets.off1) <= 8) 
          leds[0] = CRGB(0, up_led, 0);  
        FastLED.show();  
    } else if (!calibrated && !calibrating) {
      if (led_calibrating_step >= 40) {
          up_led_dir = 0;
          leds[0] = CRGB(0, 200, 0);
          leds[1] = CRGB(0, 200, 0);
          leds[2] = CRGB(0, 200, 0);
          FastLED.show();
      } else if (led_calibrating_step <= 0) {
          up_led_dir = 1;
          leds[0] = CRGB::Black;
          leds[1] = CRGB::Black;
          leds[2] = CRGB::Black;
          FastLED.show();
      }
        if (up_led_dir == 1) 
          led_calibrating_step++;
        else 
          led_calibrating_step--;
    } else if (calibrating) {
      if (calibrating_step == 1) {
        leds[1] = CRGB(200, 0, 0);
        leds[0] = CRGB::Black;
        leds[2] = CRGB::Black;
      } else if (calibrating_step == 2) {
        leds[2] = CRGB(200, 0, 0);
        leds[1] = CRGB::Black;
        leds[0] = CRGB::Black;
      } else if (calibrating_step == 3) {
        leds[0] = CRGB(200, 0, 0);
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
      }
      FastLED.show();
    } else {
        up_led = 0;
        leds[0] = CRGB::Black;
        leds[1] = CRGB::Black;
        leds[2] = CRGB::Black;
        FastLED.show();
    }
    previousT_3 = currentT;
  }
    batteryVoltage((double)analogRead(VBAT) / bat_divider);
  if (currentT - previousT_2 >= 1500) {
    if (!calibrated && !calibrating) {
      #if (USE_BT)
        SerialBT.println("first you need to calibrate the balancing point...");
      #endif  
      Serial.println("first you need to calibrate the balancing point...");
    }    
    batteryVoltage((double)analogRead(VBAT) / bat_divider);
    previousT_2 = currentT;
  }
}