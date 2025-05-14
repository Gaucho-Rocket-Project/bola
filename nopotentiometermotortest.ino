const int escPin  = 18;  // GPIO pin connected to ESC signal
const int escFreq = 50;  // 50 Hz standard for RC/ESC
const int escRes  = 16;  // 16-bit resolution

// Convert a pulse width in µs to ESP32 LEDC duty
uint32_t usToDuty(int us) {
  return (uint32_t(us) * ((1UL<<escRes)-1)) / 20000;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(escPin, escFreq, escRes);

  // 🛡 Arm the ESC by sending true zero-throttle (1000µs)
  Serial.println("Arming ESC at 1000 µs (full low)...");
  ledcWrite(escPin, usToDuty(1000));
  delay(5000);  // wait for cell-count beeps and arming
}

void loop() {
  // ===== Forward Ramp & Decel =====
  Serial.println("RAMPING FORWARD");
  for (int us = 1500; us <= 1600; us += 2) {
    ledcWrite(escPin, usToDuty(us));
    delay(50);
  }
  Serial.println("HOLD FORWARD");
  delay(1000);
  Serial.println("DECELERATING TO NEUTRAL");
  for (int us = 1600; us >= 1500; us -= 2) {
    ledcWrite(escPin, usToDuty(us));
    delay(50);
  }

  // Neutral pause
  Serial.println("PAUSE AT NEUTRAL");
  ledcWrite(escPin, usToDuty(1500));
  delay(2000);

  // ===== Reverse Ramp & Decel =====
  Serial.println("RAMPING REVERSE");
  for (int us = 1500; us >= 1400; us -= 2) {
    ledcWrite(escPin, usToDuty(us));
    delay(50);
  }
  Serial.println("HOLD REVERSE");
  delay(1000);
  Serial.println("DECELERATING TO NEUTRAL");
  for (int us = 1400; us <= 1500; us += 2) {
    ledcWrite(escPin, usToDuty(us));
    delay(50);
  }

  // Neutral pause
  Serial.println("PAUSE AT NEUTRAL");
  ledcWrite(escPin, usToDuty(1500));
  delay(2000);
}
