#include <SimpleFOC.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include <FastLED.h>

const int escPin = 18; // ESC signal pin
const int potPin = 34; // Analog pin for pot (0–4095)
const int escFreq = 50; // 50 Hz refresh
const int escRes = 16; // 16-bit resolution

// dead zone half-width (µs around 1500 that we'll treat as “neutral”)
const int deadZone = 25; // so 1500±25 = [1475–1525] → exactly 1500

uint32_t usToDuty(int us) {
  return (uint32_t(us) * ((1UL << escRes) - 1)) / 20000;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(escPin, escFreq, escRes);

  // Force neutral at startup
  ledcWrite(escPin, usToDuty(1500));
  delay(5000); // let ESC arm at neutral
  Serial.println("ESC armed. Pot controls throttle with dead zone ±25µs.");
}

void loop() {
  int potVal = analogRead(potPin); // 0–4095
  // Map 0–4095 → 1000–2000 µs
  int pwmVal = map(potVal, 0, 4095, 1000, 2000);
  pwmVal = constrain(pwmVal, 1000, 2000);

  // Apply dead zone around neutral
  if (abs(pwmVal - 1500) <= deadZone) {
    pwmVal = 1500;
  }

  ledcWrite(escPin, usToDuty(pwmVal));

  // Debug
  Serial.print("Pot: ");
  Serial.print(potVal);
  Serial.print(" PWM: ");
  Serial.print(pwmVal);
  Serial.print("\n");

  delay(20);
}