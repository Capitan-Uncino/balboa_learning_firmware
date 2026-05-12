#include "Balance.h"

Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

float param;

void setup() {
  // Native USB on the 32U4 runs at full speed regardless of baud rate
  Serial.begin(115200); 

  param = 0;
  randomSeed(analogRead(0));
  
  balanceSetup(); 
  
  // Default Gains
  pcData.k_phi      = 0.134267;
  pcData.k_theta    = 2.202209;
  pcData.k_phidot   = 0.276284;
  pcData.k_thetadot = 0.687669;

  /*
  
    pcData.k_phi      = 0.18257419;
  pcData.k_theta    = 4.41295298;
  pcData.k_phidot   = 0.09852314;
  pcData.k_thetadot = 0.44153694;
  
  */

}

void loop()
{ 
  // Process any incoming USB packets from the PC instantly
  handleSerial();

  // Run the 10ms control loop
  balanceUpdate();

 

  if (isBalancingStatus) {
    ledGreen(1);
    ledRed(0);
  } else {
    ledGreen(0);
    ledRed(millis() % 500 < 250); 
    
    noInterrupts();
    pcData.theta = 999.0;
    interrupts();
  }
  
  if (buttonA.getSingleDebouncedPress()) { param += 1; }
  else if (buttonB.getSingleDebouncedPress()) {
      motors.setSpeeds(0, 0);
      balanceSetup();
      delay(1000); 
  }
  else if (buttonC.getSingleDebouncedPress()) { param -= 1; }
}