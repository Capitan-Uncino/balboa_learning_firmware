#include <Wire.h>
#include "Balance.h"


Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

// Usefull variable
float param;

void setup()
{
  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  param = 0;
  randomSeed(analogRead(0));
  balanceSetup();
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