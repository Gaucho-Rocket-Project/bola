const int escPin  = 18; // ESC signal pin
const int escFreq = 50; // 50 Hz ESC refresh rate
const int escRes  = 16; // 16-bit resolution

// Converts microseconds (1000–2000) to LEDC PWM duty cycle
uint32_t usToDuty(int us) {
  return (uint32_t(us) * ((1UL << escRes) - 1)) / 20000;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(escPin, escFreq, escRes);

  // Start by arming ESC at 1000 µs
  Serial.println("ESC armed with 1000 µs. Enter values (1000–2000) below:");
  ledcWrite(escPin, usToDuty(1000));
  delay(3000);
}

void loop() {
  if (Serial.available()) {
    int val = Serial.parseInt();
    val = constrain(val, 1000, 2000);
    Serial.print("Sending PWM: ");
    Serial.println(val);
    ledcWrite(escPin, usToDuty(val));
  }
}
