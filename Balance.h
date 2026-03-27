#pragma once

#include <stdint.h>
#include <LSM6.h>
#include <Balboa32U4.h>
#include <Arduino.h>


const float UPDATE_TIME_MS = 10.0;  // [ms]
const float TICKS_RADIAN = 161.0;   // 12*51.45*41/25
const float BITS = 29000.0;         //±32768.0-> 2**15
const float DPS = 1000;
const float rad2deg = 57.296;        // 180/pi
const int32_t START_BALANCING_ANGLE = 5;     //[degrees]
const int32_t STOP_BALANCING_ANGLE = 60;     //[degrees]
const int16_t DISTANCE_DIFF_RESPONSE = 80;
const uint8_t CALIBRATION_ITERATIONS = 100;  // # measurements to calibrate gyro


extern LSM6 imu;
extern Balboa32U4Motors motors;
extern Balboa32U4Encoders encoders;
extern Balboa32U4ButtonA buttonA;
extern Balboa32U4ButtonB buttonB;
extern Balboa32U4ButtonC buttonC;

extern float param;
void balanceSetup();
void balanceUpdate();
void balance();
void lyingDown();
void beginBalance();
void integrateGyro();
void integrateEncoders();
void avoidOscillations();
void balanceUpdateSensors();
void balanceResetEncoders();
float run_policy(float phi, float theta, float dphi, float dtheta);