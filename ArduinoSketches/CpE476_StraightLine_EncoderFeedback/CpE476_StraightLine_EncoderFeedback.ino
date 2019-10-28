// This program shows how to use the encoders as feedback on the Romi 32U4.
// The encoders can tell you how far, and in which direction each
// motor has turned.
//
// You can press button A on the Romi to drive both motors
// forward at full speed.
//

#include <Romi32U4.h>

Romi32U4Encoders encoders;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

void setup()
{
  buttonA.waitForButton();
  delay(1000);
}

void loop()
{
  // Run left and right motor forward.
  motors.setSpeeds(100, 100);
  if (encoders.getCountsLeft() > encoders.getCountsRight())
  {
    // increase speed of right motor momentarily to correct for faster left motor
    motors.setSpeeds(100, 100*4);
  }
  else if (encoders.getCountsLeft() < encoders.getCountsRight())
  {
    // increase speed of left motor momentarily 
    motors.setSpeeds(100*4, 100);
  }
}
