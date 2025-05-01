// November 20, 2020 James Cochrane - BD594
// https://YouTube.com/BD594
// Build Your Own - 1D Wannabe Cubli Reaction Wheel
// https://www.youtube.com/watch?v=vqV6nazapFQ
// Final Copy Nov 22, 2020 @ 8:45pm

// My reaction wheel was inspired by the Cubli and ReM-RC's YouTube channel
// https://www.youtube.com/watch?v=n_6p-1J551Y
// https://www.youtube.com/watch?v=wFZl9MwwkB8

// An excellent resource for learning about IMU's and PIDS for the Arduino platform. It consist of 26 lessons created by Paul McWhorter.
// https://www.youtube.com/watch?v=2AO_Gmh5K3Q

// I derived and modified existing code from multiple sources of balance robots and other resources listed on YouTube

// MAKE Sure These Libraries are Installed
#include <MPU6050.h>      // MPU6050 library
#include <Wire.h>        // IIC communication library

// PWM for ESP32
#define NIDEC_PWM_CHANNEL 0
#define NIDEC_PWM_FREQ 20000
#define NIDEC_PWM_RESOLUTION 8

// Pin assignments (change as needed for ESP32)
const int nidecBrake = 8;
const int nidecDirection = 7;
const int ledsDirection = 6;
const int nidecPWM = 9;
const int nidecHalleffect = 12;

//$$$$$$$$$$$$$$$$$$$ MPU 6050 $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
MPU6050 mpu6050;     // MPU6050 name mpu6050 
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ; // 6DOF 3-axis acceleration and 3-axis gyroscope variables
//$$$$$$$$$$$$$$$$$$$ MPU 6050 $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//************** Nidec 24H677H010 BLDC Motor Vars ****************
float loopOnce=0; // Used to reset Nidec jump-up routine
//************** End of Nidec 24H677H010 BLDC Motor Vars ****************

//@@@@@@@@@@@@@@@@@@@@@@@ Begin of Angle Vars @@@@@@@@@@@@@@@@@@@@@@@@@@
float angle0 = -5; //mechanical balance angle (ideally 0 degrees) 
float Gyro_x;
float Gyro_y;
float Gyro_z;  //Angular velocity by gyroscope calculation
//@@@@@@@@@@@@@@@@@@@@@@@@ End of Angle Vars @@@@@@@@@@@@@@@@@@@@@@@@@@@

// Good YouTube video resource for Kalman Filter https://www.youtube.com/watch?v=mwn8xhgNpFY
//######################## Begin of Kalman Filter Vars ##################################################################
float Q_angle = 0.001;    // Covariance of gyroscope noise    
float Q_gyro = 0.003;    // Covariance of gyroscope drift noise
float R_angle = 0.5;    // Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time
float K1 = 0.05; // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate
float K_0,K_1,t_0,t_1;
float angle_err;
float q_bias;    // Gyroscope Drift
float angle;
float angleY_one;
float angle_speed;
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//########################### End of Kalman Filter Vars ###################################################################

// Good YouTube video resource for PID's https://www.youtube.com/watch?v=0vqWyramGy8
////////////////////// Begin of PID parameters ///////////////////////////////
double kp = 28;
double ki = 0; // NOT USED
double kd = 0.62; 
////////////////////// End of PID parameters /////////////////////////////////

///////////////////////////////// Begin of PID speed loop Vars //////////////////////
double kp_speed =  3; 
double ki_speed = 0.072;
double kd_speed = 0; // NOT USED  
double targetAngle = -5; // Angle balance point
int PD_pwm;  //angle output
float pwmOut=0;
float pwmOut2=0; 
///////////////////////////////// End of PID speed loop Vars //////////////////////

////////////////// Begin of ISR speed count var /////////////////////////////
volatile long countHall = 0;// Used to calculate the pulse value calculated by the Hall encoder 
////////////////// End of ISR speed count var ///////////////////////////////

////////////////////// Begin of pulse count /////////////////////////
int rw = 0;
int pulseCount = 0;
int rwPulse;
////////////////////// End of pulse count //////////////////////////

//////////////////////////////// Begin of PI_pwm Vars //////////////////////////
float speeds_filterold=0;
float positions=0;
double PI_pwm;
int cc;
float speeds_filter;
//////////////////////////////// End of PI_pwm Vars /////////////////////////////

int loopcount; // Counter var for D13 onboard LED flash evey 1 second

// Timer for ISR
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ISR routine forward declaration
void IRAM_ATTR ISR_Routine();
void IRAM_ATTR Code_Hall();

void setup()
{
  // PWM setup for ESP32
  ledcSetup(NIDEC_PWM_CHANNEL, NIDEC_PWM_FREQ, NIDEC_PWM_RESOLUTION);
  ledcAttachPin(nidecPWM, NIDEC_PWM_CHANNEL);

  pinMode(nidecDirection, OUTPUT);
  pinMode(ledsDirection, OUTPUT);
  pinMode(nidecBrake, OUTPUT);
  pinMode(nidecHalleffect, INPUT_PULLUP); // Use pull-up if needed
  pinMode(13, OUTPUT);

  digitalWrite(nidecDirection, LOW);
  digitalWrite(ledsDirection, HIGH);
  digitalWrite(nidecBrake, HIGH);
  ledcWrite(NIDEC_PWM_CHANNEL, 0);

  Wire.begin();
  delay(500);
  mpu6050.initialize();
  delay(500);
  Serial.begin(9600);

  // Attach interrupt for Hall effect sensor
  attachInterrupt(digitalPinToInterrupt(nidecHalleffect), Code_Hall, CHANGE);

  // Timer interrupt every 5ms (200Hz)
  timer = timerBegin(0, 80, true); // 80 prescaler: 1us per tick (80MHz/80)
  timerAttachInterrupt(timer, &ISR_Routine, true);
  timerAlarmWrite(timer, 5000, true); // 5000us = 5ms
  timerAlarmEnable(timer);
}

void loop()
{
  // No need to re-attach interrupts in loop for ESP32
}

// ISR for Hall effect
void IRAM_ATTR Code_Hall()
{
  portENTER_CRITICAL_ISR(&timerMux);
  countHall++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

//////////////////// Hall Effect Pulse count ///////////////////////
void countpulse()
{
  rw = countHall;  // Assign the value counted by encoder to rw

  countHall = 0;   // Clear countHall

  pulseCount = rw;    

  if (pwmOut < 0)   //  Reaction wheel turning counter clockwise than pulse is a negative number.
  {

    pulseCount = -pulseCount;
  }
  else if (pwmOut > 0)    //Reaction wheel turning clockwise than pulse is a positive number.
  {
    pulseCount = pulseCount;
  }

  // enter interrupt per 5ms，pulse number plus
  rwPulse += pulseCount;
}

void IRAM_ATTR ISR_Routine()
{
  portENTER_CRITICAL_ISR(&timerMux);
  countpulse();
  mpu6050.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  angle_calculate(accX, accY, accZ, gyroX, gyroY, gyroZ, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  PD();
  ReactionWheelPWM();

  cc++;
  if (cc >= 8)
  {
    SpeedPIout();
    cc = 0;
  }

  loopcount++;
  if (loopcount == 200)
  {
    loopcount = 0;
    digitalWrite(13, !digitalRead(13));
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

///////////////////////////// Tilt calculations ///////////////////////
void angle_calculate(int16_t accX,int16_t accY,int16_t accZ,int16_t gyroX,int16_t gyroY,int16_t gyyroZ,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  float Angle = -atan2(accY , accZ) * (180/ PI);           //Radial rotation angle calculation formula ; negative sign is direction processing
  Gyro_x = -gyroX / 131;              //The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //Kalman Filter
  //rotating angle Z-axis parameter
  Gyro_z = -gyroZ / 131;                      //angle speed of Z-axis
  //accelz = accZ / 1604;

  float angleAx = -atan2(accX, accZ) * (180 / PI); //calculate the inclined angle with x-axis
  Gyro_y = -gyroY / 131.00; //angle speed of Y-axis
  Yiorderfilter(angleAx, Gyro_y); //first-order filtering
}
////////////////////////////////////////////////////////////////

/////////////////////////////// Kalman Filter Calculations /////////////////////
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;          //prior estimate
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //The differential of the covariance of the prior estimate error
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //The integral of the covariance differential of the prior estimate error
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variables in matrix multiplication 
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //denominator
  E = R_angle + C_0 * PCt_0;
  //gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  // Intermediate variables in matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    // Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    // Posterior estimate
  angle_speed = gyro_m - q_bias;   // The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err; // Posterior estimation; get the optimal angle
}

///////////////////// First-order filter /////////////////
void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

////////////////// Angle PD_pwm ////////////////////
void PD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
}

////////////////// Begin of Speed PI_pwm ////////////////////
void SpeedPIout()
{
  float speeds = (rwPulse) * 1.0;      //speed  pulse value
  rwPulse = 0;
  speeds_filterold *= 0.7;         //first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550,3550);    //Anti-integral saturation
  PI_pwm = ki_speed * (targetAngle - positions) + kp_speed * (targetAngle - speeds_filter);      //speed loop control PI
}
////////////////// End of Speed PI_pwm ////////////////////


//////////////////////////// Final PWM Values /////////////////////////////
void ReactionWheelPWM()
{
  pwmOut = -PD_pwm - PI_pwm;
  pwmOut = constrain(pwmOut, -255, 255);
  pwmOut2 = map(pwmOut, -255, 255, -180, 130);

  if (angle >= 25 && loopOnce == 1)
  {
    for (int i = 0; i <= 10; i++)
    {
      delay(200);
      digitalWrite(ledsDirection, LOW);
      delay(200);
      digitalWrite(ledsDirection, HIGH);
    }
    digitalWrite(nidecBrake, HIGH);
    digitalWrite(nidecDirection, LOW);
    ledcWrite(NIDEC_PWM_CHANNEL, 185);
    delay(4000);
    digitalWrite(nidecDirection, HIGH);
    ledcWrite(NIDEC_PWM_CHANNEL, 10);
    pwmOut2 = 0;
    delay(125);
    loopOnce = 0;
  }

  if (angle <= -25 && loopOnce == 0)
  {
    for (int i = 0; i <= 20; i++)
    {
      delay(50);
      digitalWrite(ledsDirection, HIGH);
      delay(50);
      digitalWrite(ledsDirection, LOW);
    }
    loopOnce = 1;
  }

  if (angle >= 20 || angle <= -20)
  {
    digitalWrite(nidecBrake, LOW);
    ledcWrite(NIDEC_PWM_CHANNEL, 0);
  }
  else
  {
    if (pwmOut >= 0)
    {
      digitalWrite(nidecBrake, HIGH);
      digitalWrite(nidecDirection, LOW);
      digitalWrite(ledsDirection, LOW);
      ledcWrite(NIDEC_PWM_CHANNEL, pwmOut2);
    }
    else
    {
      digitalWrite(nidecBrake, HIGH);
      digitalWrite(nidecDirection, HIGH);
      digitalWrite(ledsDirection, HIGH);
      ledcWrite(NIDEC_PWM_CHANNEL, -pwmOut2);
    }
  }
}