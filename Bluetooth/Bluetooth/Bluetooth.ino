#include "BluetoothSerial.h"
#include "ESP32Servo.h"

char cmd;
BluetoothSerial serialBT;

void setup() {
  // put your setup code here, to run once:
  serialBT.begin("Esp32-BT");
  
  Serial.begin(115200);
  Serial.println("Looking for bluetooth connection");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(serialBT.available()){
    cmd = serialBT.read();
    if(cmd == '1'){
      Serial.println("function to start the launch sequence is running");
    }
    
  }
  delay(20);
}