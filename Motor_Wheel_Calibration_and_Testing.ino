const int escPin  = 18; // this is the gpio pin, where im sending PWM to
const int escFreq = 50; // standardd for signals hz
const int escRes  = 16; // bits of resolution for the signal
// helper function converts pulse width to values  (ledcwrite() expects numbers )
uint32_t usToDuty(int us) {
  return (uint32_t(us) * ((1UL << escRes) - 1)) / 20000;
}
// starts serial port so you see debug messages
void setup() {
  Serial.begin(115200); 
  ledcAttach(escPin, escFreq, escRes); // set pin 18 to putput PWM signals at 50hz with 16bit res

  // IMPORTANT: ESC expects throttle to be LOW at power-on
  Serial.println("Sending 1000 µs to arm ESC...");
  ledcWrite(escPin, usToDuty(1000));  // zero throttle
  delay(3000);                        // wait 3 seconds for ESC to arm
}
// increases PWM from 1000micro sec to 2000
void loop() {
  for (int us = 1000; us <= 2000; us += 10) {
    ledcWrite(escPin, usToDuty(us));
    Serial.print("Throttle: ");
    Serial.println(us);
    delay(50); //changes 10 micro sec every 50 ms
  }
//  ramps down the throttle the same way, after a full up/down throttle sweep pauses 1 sec and repeat
  for (int us = 2000; us >= 1000; us -= 10) {
    ledcWrite(escPin, usToDuty(us));
    Serial.print("Throttle: ");
    Serial.println(us);
    delay(50);
  }
  delay(1000); // pauses one second and then repeat
}