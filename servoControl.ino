#include <ESP32Servo.h>

//time step
const float TIME_STEP = 0.01;

//Servos
Servo servoX;
Servo servoY;

//Servo pins
int xPin = 13;
int yPin = 14;

//data struct (not used yet)
struct testStruct{
  float roll;
  float pitch;
  float yaw;
};

//PID constants
float Kp;
float Ki;
float Kd;
float initial_I[2];//x,y
float current_I[2]; //x,y
float initial_angles[2]; //phi,theta
float current_angles[2]; //phi,theta

//helper functions
float calculateI(float I0, float current_angle);
float calculateD(float angle1, float angle2);
float alpha(float Ix, float Dx, float current_phi);
float beta(float Iy, float Dy, float current_theta);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servoX.setPeriodHertz(50);  // typical servo frequency
  servoX.attach(xPin, 500, 2400); // pin, min/max pulse width in µs
  servoY.setPeriodHertz(50);
  servoY.attach(yPin, 500, 2400);
  initial_I[0] = initial_angles[0];
  initial_I[1] = initial_angles[1];
  Kp = 3.3125; 
  Ki = 0.2; 
  Kd = 1.3; 
}

void loop() {
  // put your main code here, to run repeatedly:
  servoX.write(alpha());
  servoY.write(beta());
  initial_I[0] = current_I[0];
  initial_I[1] = current_I[1];
  initial_angles[0] = current_angles[0];
  initial_angles[1] = current_angles[1];    

}

//DEFINTIONS OF HELPER FUNCTIONS

float alpha(){
  //calculate current integration value
  current_I[0] = calculateI(initial_I[0], current_angles[0]);
  //calculate current derivative value
  float Dx = calculateD(initial_angles[0],current_angles[0]);
  //calculate PID sum
  return Kp * current_angles[0] + Ki * current_I[0] + Kd * Dx;
}

float beta(){
  //calculate current integration value
  current_I[1] = calculateI(initial_I[1], current_angles[1]);
  //calculate current derivative value
  float Dy = calculateD(initial_angles[1],current_angles[1]);
  //calculate PID sum
  return Kp * current_angles[0] + Ki * current_I[1] + Kd * Dy;
}

float calculateI(float I0, float current_angle){
  return I0 + current_angle * TIME_STEP;
}

float calculateD(float angle1, float angle2){
  return (angle2 - angle1)/TIME_STEP;
}