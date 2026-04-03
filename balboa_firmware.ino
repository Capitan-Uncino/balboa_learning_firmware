#include <Wire.h>
#include "Balance.h"


Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

float k1 = 0.0;
float k2 = 0.0;
float k3 = 0.0;
float k4 = 0.0;

// Usefull variable
float param;

void setup()
{
  // Initialize UART connection to the Raspberry Pi at 115200 baud
  Serial1.begin(115200);

  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  param = 0;
  randomSeed(analogRead(0));
  balanceSetup(); 
  
  // Initialize parameters
  k1 = 0.0;
  k2 = 0.0;
  k3 = 0.0;
  k4 = 0.0;
}

void loop()
{ 
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
