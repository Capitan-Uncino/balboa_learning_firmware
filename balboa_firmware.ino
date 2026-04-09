#include <Wire.h>
#include "Balance.h"


Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;


// Usefull variable
float param;

void setup() {
  // Initialize the Pololu I2C Slave library with I2C address 8
  piSlave.init(8);

  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  param = 0;
  randomSeed(analogRead(0));
  balanceSetup(); 
  
  // Initialize parameters directly inside the shared I2C buffer.
  // (Notice I named them explicitly so the math below is impossible to mix up)
  piSlave.buffer.k_phi      = 0.18257419;
  piSlave.buffer.k_phidot   = 0.09852314;
  piSlave.buffer.k_theta    = 4.41295298;
  piSlave.buffer.k_thetadot = 0.44153694;
}

void loop()
{ 
    // 1. Let the library handle any pending background I2C tasks
  piSlave.updateBuffer();
  balanceUpdate();
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
