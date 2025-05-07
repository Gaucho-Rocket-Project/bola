const int escPin  = 18;
const int escFreq = 50;
const int escRes  = 16;

uint32_t usToDuty(int us) {
  return (uint32_t(us) * ((1UL << escRes) - 1)) / 20000;
}

void setup() {
  Serial.begin(115200);
  ledcAttach(escPin, escFreq, escRes);

  // IMPORTANT: ESC expects throttle to be LOW at power-on
  Serial.println("Sending 1000 µs to arm ESC...");
  ledcWrite(escPin, usToDuty(1000));  // zero throttle
  delay(3000);                        // wait 3 seconds for ESC to arm
}

void loop() {
  for (int us = 1000; us <= 2000; us += 10) {
    ledcWrite(escPin, usToDuty(us));
    Serial.print("Throttle: ");
    Serial.println(us);
    delay(50);
  }

  for (int us = 2000; us >= 1000; us -= 10) {
    ledcWrite(escPin, usToDuty(us));
    Serial.print("Throttle: ");
    Serial.println(us);
    delay(50);
  }

  delay(1000);
}