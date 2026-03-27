#include <Wire.h>
#include "Balance.h"
#include <math.h>
#include "STEPfile.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

int32_t gYZero;
float phi;
float phiLeft;
float phiRight;
float phiDif;
float phiDot;
float theta;
float thetaDot;
int16_t motorSpeed;

bool isBalancingStatus = false;
float ref;
int32_t startBalancingTime;
int32_t oscillationTime;


void balanceSetup()
{
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    delay(200);
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }
  gYZero = total / CALIBRATION_ITERATIONS;
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  lastMillis = ms;

  if (isBalancingStatus)
  {
    balanceUpdateSensors();
    balance();

    // Stop trying to balance if we have been farther from
    // vertical than STOP_BALANCING_ANGLE for 5 counts.
    if (abs(theta) > STOP_BALANCING_ANGLE/rad2deg)
    {
      if (++count > 5)
      {
        isBalancingStatus = false;
        count = 0;
        motors.setSpeeds(0, 0);
      }
    }
    else
    {
      count = 0;
    }
  }
  else
  {
    lyingDown();
    

    // Start trying to balance if we have been closer to
    // vertical than START_BALANCING_ANGLE for 5 counts.
    if (abs(theta) < START_BALANCING_ANGLE/rad2deg)
    {
      if (++count > 5)
      {
        count = 0;
        beginBalance();
      }
    }
    else
    {
      count = 0;
    }
  }
}

void balanceUpdateSensors()
{
  integrateGyro();
  integrateEncoders();
}

void integrateGyro()
{
  imu.read();
  thetaDot = (imu.g.y - gYZero) / BITS * DPS  / rad2deg;               // [Rad/s]
  theta += thetaDot * UPDATE_TIME_MS / 1000.0;                           // [Rad]
}

void integrateEncoders()
{
  static float lastCountsLeft, phiDotLeft;
  float countsLeft = encoders.getCountsLeft()/TICKS_RADIAN;           // [Rad]
  phiDotLeft = (countsLeft - lastCountsLeft)*1000/UPDATE_TIME_MS;     // [Rad/s]
  phiLeft += countsLeft - lastCountsLeft;                             // Trick to reset the starting position
  lastCountsLeft = countsLeft;

  static float lastCountsRight, phiDotRight;
  float countsRight = encoders.getCountsRight()/TICKS_RADIAN;         // [Rad]
  phiDotRight = (countsRight - lastCountsRight)*1000/UPDATE_TIME_MS;  // [Rad/s]
  phiRight += countsRight - lastCountsRight;                          // Trick to reset the starting position
  lastCountsRight = countsRight;

  phi = (phiLeft + phiRight)/2.0;
  phiDif = phiLeft - phiRight;
  phiDot = (phiDotLeft + phiDotRight)/2.0;
}


void lyingDown()             // Setup when lying down
{
  if (abs(thetaDot) < 0.05)  // use accelerometer for angle measurement when calm
  {
    imu.read();
    theta = atan2(imu.a.z, imu.a.x);                                   // [Rad]
    //thetaDot = (imu.g.y - gYZero) / BITS * DPS  / rad2deg;            // [Rad/s]
  }
  else{
    integrateGyro();
  }
}

void beginBalance()          // Setup before balancing
{
  balanceResetEncoders();
  startBalancingTime = millis();
  oscillationTime = millis();
  isBalancingStatus = true;
}

void balanceResetEncoders()  // Reset the starting position
{
  phiLeft = 0;
  phiRight = 0;
}

void balance()
{
  theta *= 0.999;
  
  float ts = 0 + param;
  // 1. Get normalized action [-1.0 to 1.0]
  float nn_action = run_policy(phi, theta, phiDot, thetaDot);

  // 2. Handle state overrides
  if (ts == 1.0 || ts == 2.0) {
    nn_action = 0.0;          
  }

  // 3. Map to maximum base effort (1.0 = 400 PWM)
  float base_pwm = nn_action * 400.0;

  // 4. Battery Sag Compensation (Targeting 7.2V Nominal)
  float actual_voltage = readBatteryMillivolts() / 1000.0;
  
  // SAFETY: If old batteries sag too hard, prevent a division-by-near-zero explosion
  if (actual_voltage < 5.0) actual_voltage = 5.0; 

  // Scale the PWM. If battery is at 6.0V, it boosts the PWM to simulate 7.2V.
  float motorSpeed = (7.2 / actual_voltage) * base_pwm;

  // 6. Hard constraint to the motor driver's physical limits
  if (motorSpeed > 400.0) motorSpeed = 400.0;
  if (motorSpeed < -400.0) motorSpeed = -400.0;
  
  // (Optional integer cast if your motor driver function strictly requires int16_t)
  // int16_t finalSpeed = (int16_t)motorSpeed;

  avoidOscillations();


  motors.setSpeeds(
    motorSpeed - phiDif * DISTANCE_DIFF_RESPONSE,
    motorSpeed + phiDif * DISTANCE_DIFF_RESPONSE);

  // Print all parameters on Serial Port
  Serial.print(millis());
  Serial.print(","); 
  Serial.print(phi);
  Serial.print(","); 
  Serial.print(theta);
  Serial.print(","); 
  Serial.print(phiDot);
  Serial.print(","); 
  Serial.print(thetaDot);
  Serial.print(","); 
  Serial.print(motorSpeed);
  Serial.print(","); 
  Serial.print(ref);
  Serial.print(","); 
  Serial.println(ts);   
}


void avoidOscillations(){
  static int32_t lastDirection = 0; // bool of Ms is positif or negatif
  int32_t presentDirection = 0;
  if (motorSpeed >= 0){presentDirection = 1;}
  
  if (presentDirection != lastDirection){
    if (millis() - oscillationTime<150){
      
      motorSpeed = 0;
    }
    else{
      oscillationTime = millis();
      lastDirection = presentDirection; 
    }
  }
}


float run_policy(float phi, float theta, float dphi, float dtheta) {
    float input[4] = {phi, theta, dphi, dtheta};
    float layer1[16];
    float layer2[16];
    
    // Layer 0 -> Layer 1
    for (int i = 0; i < 16; i++) {
        float sum = pgm_read_float(&b0[i]);
        for (int j = 0; j < 4; j++) {
            sum += input[j] * pgm_read_float(&W0[i][j]);
        }
        layer1[i] = (sum > 0) ? sum : 0; 
    }

    // Layer 1 -> Layer 2
    for (int i = 0; i < 16; i++) {
        float sum = pgm_read_float(&b1[i]);
        for (int j = 0; j < 16; j++) {
            sum += layer1[j] * pgm_read_float(&W1[i][j]);
        }
        layer2[i] = (sum > 0) ? sum : 0; 
    }

    // Layer 2 -> Output Layer
    float raw_action = pgm_read_float(&b2[0]);
    for (int j = 0; j < 16; j++) {
        raw_action += layer2[j] * pgm_read_float(&W2[0][j]);
    }
    
    if (raw_action > 1.0) raw_action = 1.0;
    if (raw_action < -1.0) raw_action = -1.0;
    
    return raw_action; 
}

