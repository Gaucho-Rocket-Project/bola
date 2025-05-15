unsigned long Start_Time = millis();
bool Motor_Fired = false;
bool Leg_Deployed = false;
unsigned long Trigger_Time = ;//what time to trigger 2nd motor
Servo Landing_Leg_Servo;
Landing_Leg_Servo.attach(Landing_PIN); //Change to actual Landing Pin

void loop() {
    unsigned long Current_Time = millis();
    
    // 2nd motor trigger 
    if (Motor_Fired == false && (Current_Time - Start_Time >= Trigger_Time)) {
        digitalWrite(MOTOR_PIN, HIGH); //Change to actual MOTOR Pin
        delay(50); // Change to actual ON time
        digitalWrite(MOTOR_PIN, LOW); //Change to actual MOTOR Pin
    
    // Deploy landin leg after motor trigger
        landingLegServo.write(Deploy_Angle); // Change to Deploy angle
        Delay(300); //Make sure Landing Leg deploys
        Leg_Deployed = true;
    }
}