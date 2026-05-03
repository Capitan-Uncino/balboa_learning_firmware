#include <Wire.h>
#include "Balance.h"
#include <math.h>
#include "STEPfile.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

// Instantiate the shared memory struct
RobotData piData;

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

float last_noise = 0;
const float theta_ou = 0.30; // How fast it returns to zero
const float sigma_ou = 0.80; // The "power" of the noise 

// --- I2C INTERRUPT HANDLERS ---

// Triggered automatically when the Raspberry Pi WRITES gains
void receiveEvent(int howMany) {
  // Check if we received the offset byte (1 byte) + 4 floats (16 bytes)
  if (howMany >= 17) {
    uint8_t offset = Wire.read();
    if (offset == 0) {
       Wire.readBytes((char*)&piData.k_phi, 16);
    }
  }
  // Flush any remaining garbage to keep the bus clean
  while(Wire.available()) Wire.read();
}

// Triggered automatically when the Raspberry Pi READS telemetry
void requestEvent() {
  // Send the 5 telemetry floats (20 bytes total) starting from phi
  Wire.write((uint8_t*)&piData.phi, 20);
}

// --- CORE LOGIC ---

float generate_gaussian() {
    float u1 = (float)random(1, 10000) / 10000.0; 
    float u2 = (float)random(0, 10000) / 10000.0;
    float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
    return z0;
}

float generate_exploration_noise() {
    float epsilon = generate_gaussian();
    float dx = theta_ou * (-last_noise) * 0.01 + sigma_ou * epsilon * 0.1;
    last_noise += dx;
    return last_noise;
}

void balanceSetup()
{

  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);



  // 1. Initialize I2C as MASTER first (no address)
  // This allows the Arduino to talk to the IMU sensors without 
  // the Raspberry Pi interfering yet.
  Wire.begin(); 
  
  // 2. CRITICAL: Set I2C Timeout (3ms)
  // If the Pi causes a bus collision, the Arduino will reset its I2C 
  // hardware instead of hanging the CPU (which kills the USB port).
  Wire.setWireTimeout(3000, true); 
  Wire.setClock(400000); 

  // 3. Initialize IMU with a Hardware-Error Heartbeat
  if (!imu.init())
  {
    // If the IMU is missing or the bus is stuck, flash RED rapidly
    while(1) {
      ledRed(millis() % 100 < 50); 
    }
  }
  
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  // 4. Calibrate Gyro (Quiet Period)
  // We do this BEFORE joining the bus as a slave.
  delay(500);
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }
  gYZero = total / CALIBRATION_ITERATIONS;

  // 5. JOIN THE BUS AS SLAVE (Address 8)
  // Only now will the Raspberry Pi be able to "see" the Arduino.
  Wire.begin(8); 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // Re-apply timeout for Slave mode stability
  Wire.setWireTimeout(3000, true); 

  // 6. SIGNAL READINESS TO PI
  // Set the magic number so the Rust code knows we are online but idle.
  noInterrupts();
  piData.theta = 999.0;
  piData.phi = 0.0;
  piData.phiDot = 0.0;
  piData.thetaDot = 0.0;
  piData.u_noisy = 0.0;
  interrupts();
  
  Serial.println("Balance Setup Complete. Slave 08 Online.");
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  lastMillis = ms;

  if (isBalancingStatus)
  {
    balanceUpdateSensors();
    digitalWrite(SYNC_PIN, HIGH);
    balance();


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
    delay(3);
    digitalWrite(SYNC_PIN, LOW);
}

void balanceUpdateSensors()
{
  integrateGyro();
  integrateEncoders();
}

void integrateGyro()
{
  imu.read();
  thetaDot = (imu.g.y - gYZero) / BITS * DPS  / rad2deg;               
  theta += thetaDot * UPDATE_TIME_MS / 1000.0;                           
}

void integrateEncoders()
{
  static float lastCountsLeft, phiDotLeft;
  float countsLeft = encoders.getCountsLeft()/TICKS_RADIAN;           
  phiDotLeft = (countsLeft - lastCountsLeft)*1000/UPDATE_TIME_MS;     
  phiLeft += countsLeft - lastCountsLeft;                             
  lastCountsLeft = countsLeft;

  static float lastCountsRight, phiDotRight;
  float countsRight = encoders.getCountsRight()/TICKS_RADIAN;         
  phiDotRight = (countsRight - lastCountsRight)*1000/UPDATE_TIME_MS;  
  phiRight += countsRight - lastCountsRight;                          
  lastCountsRight = countsRight;

  phi = (phiLeft + phiRight)/2.0;
  phiDif = phiLeft - phiRight;
  phiDot = (phiDotLeft + phiDotRight)/2.0;
}

void lyingDown()             
{
  if (abs(thetaDot) < 0.05)  
  {
    imu.read();
    theta = atan2(imu.a.z, imu.a.x);                                   
  }
  else{
    integrateGyro();
  }
}

void beginBalance()          
{
  balanceResetEncoders();
  startBalancingTime = millis();
  oscillationTime = millis();
  isBalancingStatus = true;
}

void balanceResetEncoders()  
{
  phiLeft = 0;
  phiRight = 0;
}

void balance()
{
  theta *= 0.999;
  
  float ts = 0 + param;
  float u = run_policy_raspberry();

  if (ts == 1.0 || ts == 2.0) {
    u = 0.0;          
  }

  motors.setSpeeds(
    u - phiDif * DISTANCE_DIFF_RESPONSE,
    u + phiDif * DISTANCE_DIFF_RESPONSE);

}

void avoidOscillations(){
  static int32_t lastDirection = 0; 
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

float run_policy_nn() {
    float input[4] = {phi, theta, phiDot, thetaDot};
    float layer1[16];
    float layer2[16];
    
    for (int i = 0; i < 16; i++) {
        float sum = pgm_read_float(&b0[i]);
        for (int j = 0; j < 4; j++) {
            sum += input[j] * pgm_read_float(&W0[i][j]);
        }
        layer1[i] = (sum > 0) ? sum : 0; 
    }

    for (int i = 0; i < 16; i++) {
        float sum = pgm_read_float(&b1[i]);
        for (int j = 0; j < 16; j++) {
            sum += layer1[j] * pgm_read_float(&W1[i][j]);
        }
        layer2[i] = (sum > 0) ? sum : 0; 
    }

    float raw_action = pgm_read_float(&b2[0]);
    for (int j = 0; j < 16; j++) {
        raw_action += layer2[j] * pgm_read_float(&W2[0][j]);
    }
    
    if (raw_action > 1.0) raw_action = 1.0;
    if (raw_action < -1.0) raw_action = -1.0; 

    float base_pwm = raw_action * 400.0;
    float actual_voltage = readBatteryMillivolts() / 1000.0;
    
    if (actual_voltage < 5.0) actual_voltage = 5.0; 

    float motorSpeed = (7.2 / actual_voltage) * base_pwm;

    if (motorSpeed > 400.0) motorSpeed = 400.0;
    if (motorSpeed < -400.0) motorSpeed = -400.0;
    
    return motorSpeed; 
} 

float run_policy_raspberry() {
  // 1. INPUT THE GAINS (The Safe Copy)
  float current_k1, current_k2, current_k3, current_k4;
  
  noInterrupts(); 
  current_k1 = piData.k_phi;
  current_k3 = piData.k_phidot; 
  current_k2 = piData.k_theta;
  current_k4 = piData.k_thetadot;
  interrupts(); 

  // 2. Calculate control effort
  float u_raw = current_k1 * phi + 
                current_k2 * theta + 
                (current_k3 + 0.19) * phiDot + 
                current_k4 * thetaDot; 

  float noise = generate_exploration_noise();
  float u_noisy = u_raw + noise;

  // 3. FAST TX: Update the shared buffer for the Pi
  noInterrupts();
  piData.phi      = phi;
  piData.phiDot   = phiDot;
  piData.theta    = theta;
  piData.thetaDot = thetaDot;
  piData.u_noisy  = u_noisy;
  interrupts();

  // 4. Calculate physical actions
  float u_physical = u_noisy;
  float offset = 0.45; 
  
  if (phiDot > 0) u_physical = u_noisy + offset; 
  else            u_physical = u_noisy - offset; 
  
  motorSpeed = 400 / (readBatteryMillivolts() / 1000.0) * u_physical; 

  avoidOscillations();

  return motorSpeed;
}

