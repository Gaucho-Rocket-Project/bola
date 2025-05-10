#include <ESP32Servo.h>

Servo myServo;  // create servo object

int servoPin = 13;  // connect servo signal to GPIO 13

void setup() {
  Serial.begin(115200);
  myServo.setPeriodHertz(50);  // typical servo frequency
  myServo.attach(servoPin, 500, 2400); // pin, min/max pulse width in µs
}

void loop() {
  // sweep from 0 to 180 degrees
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    delay(15);
  }
  // sweep back from 180 to 0 degrees
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    delay(15);
  }
}
