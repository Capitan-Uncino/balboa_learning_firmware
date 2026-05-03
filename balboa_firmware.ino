#include <Wire.h>
#include "Balance.h"

Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

float param;

void setup() {

  param = 0;
  randomSeed(analogRead(0));
  
  // This initializes the IMU, joins I2C as Slave 8, and sets the 400kHz clock
  balanceSetup(); 
  
  // Initialize parameters directly inside the native memory map
  piData.k_phi      = 0.18257419;
  piData.k_phidot   = 0.09852314;
  piData.k_theta    = 4.41295298;
  piData.k_thetadot = 0.44153694;
}

void loop()
{ 
  // No update buffers needed; Wire interrupt handlers manage traffic automatically.
  balanceUpdate();

if (isBalancingStatus) {
    ledGreen(1);
    ledRed(0);
  } else {
    ledGreen(0);
    // heartbeat
    ledRed(millis() % 500 < 250); 
    
    noInterrupts();
    piData.theta = 999.0;
    interrupts();
  }

  
  
  if (buttonA.getSingleDebouncedPress())
  {
    param += 1;
  }
  else if (buttonB.getSingleDebouncedPress())
  {
      motors.setSpeeds(0, 0);
      balanceSetup();
      delay(1000); 
  }
  else if (buttonC.getSingleDebouncedPress())
  {
    param -= 1;
  }
}
