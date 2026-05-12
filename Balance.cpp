#include "Balance.h"
#include <math.h>
#include "STEPfile.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

RobotData pcData;

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
const float theta_ou = 0.30; 
const float sigma_ou = 0.10; 

// --- SERIAL DATA HANDLER ---
// Non-blocking state machine to parse incoming binary gains
void handleSerial() {
  //if (!Serial) return; 
  
  static uint8_t state = 0;
  static uint8_t buffer[16];
  static uint8_t index = 0;
  
  // THE ESCAPE HATCH: Never process more than 32 bytes per tick.
  // This guarantees the function exits quickly and never starves your 10ms loop!
  uint8_t bytes_processed = 0;
  
  while (Serial.available() > 0 && bytes_processed < 2) {
    uint8_t b = Serial.read();
    bytes_processed++; // Count every byte we pull from the buffer
    
    switch(state) {
      case 0: if (b == 0xAA) state = 1; else state = 0; break;
      case 1: if (b == 0xBB) state = 2; else state = 0; break;
      case 2: if (b == 0xCC) state = 3; else state = 0; break;
      case 3: if (b == 0xDD) { state = 4; index = 0; } else { state = 0; } break;
      case 4: // Reading 16 bytes of floats
        buffer[index++] = b;
        if (index == 16) {
          
          // Use a safe temporary array to prevent memory alignment crashes
          float incoming_gains[4];
          memcpy(incoming_gains, buffer, 16); 
          
          noInterrupts();
          pcData.k_phi      = incoming_gains[0];
          pcData.k_theta    = incoming_gains[1];
          pcData.k_phidot   = incoming_gains[2];
          pcData.k_thetadot = incoming_gains[3];
          interrupts();
          
          state = 0; // Reset for next packet
        }
        break;
    }
  } 
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
  Wire.begin(); 
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true); // Prevents I2C hangs from bricking the USB!

  if (!imu.init()) {
    while(1) { ledRed(millis() % 100 < 50); }
  }
  
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s

  delay(500);
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }
  gYZero = total / CALIBRATION_ITERATIONS;

  noInterrupts();
  pcData.theta = 999.0;
  pcData.phi = 0.0;
  pcData.phiDot = 0.0;
  pcData.thetaDot = 0.0;
  pcData.u_noisy = 0.0;
  interrupts();
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
  if (abs(thetaDot) < 0.05)  {
    imu.read();
    theta = atan2(imu.a.z, imu.a.x);                                   
  } else {
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
  float u = run_policy_pc();

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

float run_policy_pc() {
  // 1. INPUT THE GAINS 
  float current_k1, current_k2, current_k3, current_k4;
  
  noInterrupts(); 
  current_k1 = pcData.k_phi;
  current_k2 = pcData.k_theta;    
  current_k3 = pcData.k_phidot;   
  current_k4 = pcData.k_thetadot;
  interrupts();

  // 2. Calculate control effort
  float u_raw = current_k1 * phi + 
                current_k2 * theta + 
                (current_k3 ) * phiDot + 
                current_k4 * thetaDot; 

  float noise = generate_exploration_noise();
  float u_noisy = u_raw + noise;

  // 3. Update the shared buffer
  noInterrupts();
  pcData.phi      = phi;
  pcData.theta    = theta;     
  pcData.phiDot   = phiDot;    
  pcData.thetaDot = thetaDot;
  pcData.u_noisy  = u_noisy;
  interrupts();

// ========================================================
  // 4. TX DATA TO COMPUTER OVER USB SERIAL
  // ========================================================
  
  if (Serial.availableForWrite() >= 24) { // 4 header bytes + 20 data bytes
    const uint8_t header[4] = {0xDD, 0xCC, 0xBB, 0xAA};
    Serial.write(header, 4);
    Serial.write((uint8_t*)&pcData.phi, 20); // Sends the 5 telemetry floats
  }

  // 5. Calculate physical actions
  float u_physical = u_noisy;
  float offset = 0.45; 
  
  if (phiDot > 0) u_physical = u_noisy + offset; 
  else            u_physical = u_noisy - offset; 
  
  motorSpeed = 400 / (readBatteryMillivolts() / 1000.0) * u_physical; 

  avoidOscillations();

  return motorSpeed;
}