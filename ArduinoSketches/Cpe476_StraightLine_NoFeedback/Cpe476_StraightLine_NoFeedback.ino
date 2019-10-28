// This program drives the Romi forward 2 meters without feedback
//
// You can press button A on the Romi to drive both motors forward
//

#include <Romi32U4.h>

Romi32U4Encoders encoders;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

void setup()
{
  //buttonA.waitForButton();
  //delay(1000);
}

void loop()
{
  if(buttonA.isPressed())
  {
    delay(1000);
    // Run left and right motor forward.
    motors.setSpeeds(102, 100);
    delay(9600);
    motors.setSpeeds(0,0);
  }
}
