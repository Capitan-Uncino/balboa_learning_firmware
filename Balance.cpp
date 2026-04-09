#include <Wire.h>
#include "Balance.h"
#include <math.h>
#include "STEPfile.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

PololuRPiSlave<struct RobotData, 0> piSlave;

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

float generate_gaussian() {
    // Generate two uniform random variables between 0 and 1
    // We start u1 at 1 to strictly avoid log(0) which yields negative infinity
    float u1 = (float)random(1, 10000) / 10000.0; 
    float u2 = (float)random(0, 10000) / 10000.0;

    // Box-Muller transform calculation
    float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
    
    return z0;
}

float generate_exploration_noise() {
    // 1. Get the true Gaussian noise
    float epsilon = generate_gaussian();
    
    // 2. OU Process formula: dx = theta * (-x) * dt + sigma * dW
    // dt = 0.01, and sqrt(dt) = 0.1
    float dx = theta_ou * (-last_noise) * 0.01 + sigma_ou * epsilon * 0.1;
    last_noise += dx;
    
    return last_noise;
}

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
  float u = run_policy_raspberry();

  if (ts == 1.0 || ts == 2.0) {
    u = 0.0;          
  }

  motors.setSpeeds(
    u - phiDif * DISTANCE_DIFF_RESPONSE,
    u + phiDif * DISTANCE_DIFF_RESPONSE);

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


float run_policy_nn() {
    float input[4] = {phi, theta, phiDot, thetaDot};
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

    // Map to maximum base effort (1.0 = 400 PWM)
    float base_pwm = raw_action * 400.0;

    // 4. Battery Sag Compensation (Targeting 7.2V Nominal)
    float actual_voltage = readBatteryMillivolts() / 1000.0;
    
    // SAFETY: If old batteries sag too hard, prevent a division-by-near-zero explosion
    if (actual_voltage < 5.0) actual_voltage = 5.0; 

    // Scale the PWM. If battery is at 6.0V, it boosts the PWM to simulate 7.2V.
    float motorSpeed = (7.2 / actual_voltage) * base_pwm;

    // 6. Hard constraint to the motor driver's physical limits
    if (motorSpeed > 400.0) motorSpeed = 400.0;
    if (motorSpeed < -400.0) motorSpeed = -400.0;
    
    return motorSpeed; 
} 

/*

float run_policy_nn_quantized() {
    float input[4] = {phi, theta, phiDot, thetaDot};
    
    // --- 1. Quantize Inputs to 8-bit ---
    // Find the max absolute value to scale the inputs dynamically
    float in_max = 0.00001f; // Small baseline to prevent division by zero
    for (int j = 0; j < 4; j++) {
        float abs_val = (input[j] > 0) ? input[j] : -input[j];
        if (abs_val > in_max) in_max = abs_val;
    }
    float in_scale = in_max / 127.0f;
    
    int8_t q_in[4];
    for (int j = 0; j < 4; j++) {
        q_in[j] = (int8_t)(input[j] / in_scale);
    }

    // --- 2. Layer 0 -> Layer 1 ---
    float layer1[16];
    float l1_max = 0.00001f; 
    
    for (int i = 0; i < 16; i++) {
        int32_t dot_sum = 0; 
        for (int j = 0; j < 4; j++) {
            // CRITICAL: pgm_read_byte returns unsigned. We MUST cast to int8_t first, 
            // otherwise negative weights become positive garbage!
            int8_t weight = (int8_t)pgm_read_byte(&W0[i][j]);
            // 8-bit x 8-bit natively fits in 16 bits, which is much faster for the chip to calculate
            dot_sum += (int32_t)( (int16_t)q_in[j] * (int16_t)weight );
        }
        
        // De-quantize the sum, apply the scales, and add the scaled float bias
        float val = (dot_sum * in_scale * W0_scale) + ((int8_t)pgm_read_byte(&b0[i]) * b0_scale);
        
        // ReLU Activation
        layer1[i] = (val > 0) ? val : 0; 
        
        // Track the max for the next layer's quantization (ReLU means no negative values)
        if (layer1[i] > l1_max) l1_max = layer1[i];
    }

    // --- 3. Quantize Layer 1 Outputs ---
    float l1_scale = l1_max / 127.0f;
    int8_t q_l1[16];
    for (int j = 0; j < 16; j++) {
        q_l1[j] = (int8_t)(layer1[j] / l1_scale);
    }

    // --- 4. Layer 1 -> Layer 2 ---
    float layer2[16];
    float l2_max = 0.00001f;
    
    for (int i = 0; i < 16; i++) {
        int32_t dot_sum = 0;
        for (int j = 0; j < 16; j++) {
            int8_t weight = (int8_t)pgm_read_byte(&W1[i][j]);
            dot_sum += (int32_t)( (int16_t)q_l1[j] * (int16_t)weight );
        }
        
        float val = (dot_sum * l1_scale * W1_scale) + ((int8_t)pgm_read_byte(&b1[i]) * b1_scale);
        
        // ReLU Activation
        layer2[i] = (val > 0) ? val : 0;
        if (layer2[i] > l2_max) l2_max = layer2[i];
    }

    // --- 5. Quantize Layer 2 Outputs ---
    float l2_scale = l2_max / 127.0f;
    int8_t q_l2[16];
    for (int j = 0; j < 16; j++) {
        q_l2[j] = (int8_t)(layer2[j] / l2_scale);
    }

    // --- 6. Layer 2 -> Output Layer ---
    int32_t out_dot_sum = 0;
    for (int j = 0; j < 16; j++) {
        int8_t weight = (int8_t)pgm_read_byte(&W2[0][j]);
        out_dot_sum += (int32_t)( (int16_t)q_l2[j] * (int16_t)weight );    }
    
    // Final de-quantization to get the raw action
    float raw_action = (out_dot_sum * l2_scale * W2_scale) + ((int8_t)pgm_read_byte(&b2[0]) * b2_scale);
    
    // Tanh-like clipping
    if (raw_action > 1.0f) raw_action = 1.0f;
    if (raw_action < -1.0f) raw_action = -1.0f; 

    // --- Post-Processing ---
    float base_pwm = raw_action * 400.0f;

    // Battery Sag Compensation (Targeting 7.2V Nominal)
    float actual_voltage = readBatteryMillivolts() / 1000.0f;
    if (actual_voltage < 5.0f) actual_voltage = 5.0f; 

    float motorSpeed = (7.2f / actual_voltage) * base_pwm;

    // Hard constraint to the motor driver's physical limits
    if (motorSpeed > 400.0f) motorSpeed = 400.0f;
    if (motorSpeed < -400.0f) motorSpeed = -400.0f;
    
    return motorSpeed; 
}

float run_policy_nn_q10() {
    float input[4] = {phi, theta, phiDot, thetaDot};
    
    // --- 1. Convert Inputs to Q10 Fixed-Point ---
    int32_t q_in[4];
    for (int j = 0; j < 4; j++) {
        // Multiply by 1024.0f to scale to Q10 format
        q_in[j] = (int32_t)(input[j] * 1024.0f);
    }

    // --- 2. Layer 0 -> Layer 1 ---
    int32_t layer1[16];
    for (int i = 0; i < 16; i++) {
        int32_t dot_sum = 0; 
        for (int j = 0; j < 4; j++) {
            // Read 16-bit weight using pgm_read_word
            int16_t weight = (int16_t)pgm_read_word(&W0[i][j]);
            dot_sum += q_in[j] * weight; 
        }
        
        // Bit-shift right by 10 to instantly divide the accumulated product by 1024
        dot_sum >>= 10;
        
        // Add the bias
        dot_sum += (int16_t)pgm_read_word(&b0[i]);
        
        // ReLU Activation
        layer1[i] = (dot_sum > 0) ? dot_sum : 0; 
    }

    // --- 3. Layer 1 -> Layer 2 ---
    int32_t layer2[16];
    for (int i = 0; i < 16; i++) {
        int32_t dot_sum = 0;
        for (int j = 0; j < 16; j++) {
            int16_t weight = (int16_t)pgm_read_word(&W1[i][j]);
            dot_sum += layer1[j] * weight;
        }
        
        dot_sum >>= 10;
        dot_sum += (int16_t)pgm_read_word(&b1[i]);
        
        // ReLU Activation
        layer2[i] = (dot_sum > 0) ? dot_sum : 0;
    }

    // --- 4. Layer 2 -> Output Layer ---
    int32_t out_dot_sum = 0;
    for (int j = 0; j < 16; j++) {
        int16_t weight = (int16_t)pgm_read_word(&W2[0][j]);
        out_dot_sum += layer2[j] * weight;
    }
    
    out_dot_sum >>= 10;
    out_dot_sum += (int16_t)pgm_read_word(&b2[0]);
    
    // --- 5. Final Float Conversion ---
    // Divide by 1024.0f to turn the final integer back into a float action
    float raw_action = (float)out_dot_sum / 1024.0f;
    
    // Tanh-like clipping
    if (raw_action > 1.0f) raw_action = 1.0f;
    if (raw_action < -1.0f) raw_action = -1.0f; 

    // --- Post-Processing ---
    float base_pwm = raw_action * 400.0f;

    // Battery Sag Compensation (Targeting 7.2V Nominal)
    float actual_voltage = readBatteryMillivolts() / 1000.0f;
    if (actual_voltage < 5.0f) actual_voltage = 5.0f; 

    float motorSpeed = (7.2f / actual_voltage) * base_pwm;

    // Hard constraint to the motor driver's physical limits
    if (motorSpeed > 400.0f) motorSpeed = 400.0f;
    if (motorSpeed < -400.0f) motorSpeed = -400.0f;
    
    return motorSpeed; 
}

*/

float run_policy_raspberry() {


  // 2. INPUT THE GAINS (The Safe Copy)
  // Disable interrupts briefly to copy the gains from the shared Pi buffer 
  // into local variables. This prevents data tearing.
  float current_k1, current_k2, current_k3, current_k4;
  
  noInterrupts(); 
  current_k1 = piSlave.buffer.k_phi;
  current_k3 = piSlave.buffer.k_phidot; // Mapped to match your original logic
  current_k2 = piSlave.buffer.k_theta;
  current_k4 = piSlave.buffer.k_thetadot;
  interrupts(); 
  // Interrupts are back on! The Pi is free to communicate again.

  // 3. Calculate control effort: u = -Kx
  float u_raw = current_k1 * phi + 
                current_k2 * theta + 
                (current_k3 + 0.19) * phiDot + 
                current_k4 * thetaDot; 

  float noise = generate_exploration_noise();
  float u_noisy = u_raw + noise;

  // 4. FAST TX: Update the shared buffer for the Pi to read
  // We briefly disable interrupts here too, so the Pi doesn't accidentally
  // read the telemetry while we are only halfway through updating it.
  noInterrupts();
  piSlave.buffer.phi      = phi;
  piSlave.buffer.phiDot   = phiDot;
  piSlave.buffer.theta    = theta;
  piSlave.buffer.thetaDot = thetaDot;
  piSlave.buffer.u_noisy  = u_noisy;
  interrupts();

  // 5. Calculate physical actions
  float u_physical = u_noisy;
  float offset = 0.45; //WARNING MORE MAGIC!
  
  if (phiDot > 0) u_physical = u_noisy + offset; 
  else            u_physical = u_noisy - offset; 
  
  motorSpeed = 400 / (readBatteryMillivolts() / 1000.0) * u_physical; 

  avoidOscillations();

  return motorSpeed;
}


