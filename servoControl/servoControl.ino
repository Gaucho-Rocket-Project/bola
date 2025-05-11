#include <ESP32Servo.h>
#include <constant_force.h>

// how many time‐steps do we have?
static const size_t NUM_SAMPLES = sizeof(t)/sizeof(t[0]);

// which sample are we on?
size_t sampleIndex = 0;
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
  // start integral terms at zero
  initial_I[0] = initial_I[1] = 0.0f;

  // take your “previous angle” to be the very first sample
  initial_angles[0] = roll[0];
  initial_angles[1] = pitch[0];
  
  // (you had Kp, Ki, Kd already set)  
  // initial_I[0] = initial_angles[0];
  // initial_I[1] = initial_angles[1];
  Kp = 3.3125; 
  Ki = 0.2; 
  Kd = 1.3;

}

// void loop() {
//   // put your main code here, to run repeatedly:
//   servoX.write(alpha());
//   servoY.write(beta());
//   initial_I[0] = current_I[0];
//   initial_I[1] = current_I[1];
//   initial_angles[0] = current_angles[0];
//   initial_angles[1] = current_angles[1];    

// }

void loop() {
  if (sampleIndex < NUM_SAMPLES) {
    // 1) load the new “measured” angles
    current_angles[0] = roll[sampleIndex];
    current_angles[1] = pitch[sampleIndex];

    // 2) compute PID outputs
    float outX = alpha();
    float outY = beta();

    // 3a) actually drive servos
    servoX.write(outX);
    servoY.write(outY);

    // 3b) or just print for offline analysis
    Serial.printf("t=%.8f  roll→%.8f  pitch→%.8f  → outX=%.8f  outY=%.8f\n",
                  t[sampleIndex],
                  current_angles[0],
                  current_angles[1],
                  outX,
                  outY);

    // 4) update “previous” state for next iteration
    initial_I[0]      = current_I[0];
    initial_I[1]      = current_I[1];
    initial_angles[0] = current_angles[0];
    initial_angles[1] = current_angles[1];

    sampleIndex++;
  }
  // wait one time‐step (0.01 s)
  delay(int(TIME_STEP * 1000));
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
  return Kp * current_angles[1] + Ki * current_I[1] + Kd * Dy;
}

float calculateI(float I0, float current_angle){
  return I0 + current_angle * TIME_STEP;
}

float calculateD(float angle1, float angle2){
  return (angle2 - angle1)/TIME_STEP;
}