// This program drives the Romi in a box in the forward direction
// This was tested on hardwood floor.
//
// You can press button A on the Romi to make the box
//

#include <Romi32U4.h>

Romi32U4Encoders encoders;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

void setup()
{
}

void loop()
{
  if(buttonA.isPressed())
  {
    delay(1000);

    //////////////////////////////
    //    BEGIN FORWARD BOX     //
    //////////////////////////////
    
    // Run left and right motor forward.
    motors.setSpeeds(100, 101);
    delay(1375); //To travel 1 foot forward
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF FIRST SIDE     //
    //////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // From here, the radius of the robot is 2.875 in                             //
    // To make a full rotation, the wheel must travel 2*pi*r = 6*pi = 18.064in    //
    // To make a 90 degree rotation (360/4) then the wheel must travel            //
    //  18.064in/4 = 4.516 in.                                                    //
    // since it takes 1375ms to travel 12 in, 1375ms/12in = 114.583 ms/in         //
    // So to travel 4.516in, (114.583 ms/in)*(4.516in) = 517.4568ms               //
    // The full equation is ((pi/2)*r)*(time it takes to move 1 foot)             //
    // where r is the radius of the robot.                                        //
    // *It looks as if theres a little error with the above time. 490ms is good   //
    ////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////
    //    BEGIN SECOND SIDE     //
    //////////////////////////////
    motors.setSpeeds(-101, 100); //left rotation
    delay(515);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(100, 101); //forward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF SECOND SIDE    //
    //////////////////////////////

    //////////////////////////////
    //    BEGIN THIRD SIDE      //
    //////////////////////////////
    motors.setSpeeds(-101, 100); //left rotation
    delay(515);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(100, 101); //forward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF THIRD SIDE     //
    //////////////////////////////

    //////////////////////////////
    //    BEGIN FOURTH SIDE     //
    //////////////////////////////
    motors.setSpeeds(-101, 100); //left rotation
    delay(515);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(100, 101); //forward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(-101, 100); //left rotation
    delay(515);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF FORWARD BOX    //
    //////////////////////////////

    //////////////////////////////
    //    BEGIN REVERSE BOX     //
    //////////////////////////////
    motors.setSpeeds(101, -100); //right rotation
    delay(525);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(-99, -102); //backward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF FIRST SIDE     //
    //////////////////////////////

    //////////////////////////////
    //    BEGIN SECOND SIDE     //
    //////////////////////////////
    motors.setSpeeds(101, -100); //right rotation
    delay(525);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(-99, -102); //backward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF SECOND SIDE    //
    //////////////////////////////

    //////////////////////////////
    //    BEGIN THIRD SIDE      //
    //////////////////////////////
    motors.setSpeeds(101, -100); //right rotation
    delay(525);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(-99, -102); //backward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF THIRD SIDE     //
    //////////////////////////////

    //////////////////////////////
    //    BEGIN FOURTH SIDE     //
    //////////////////////////////
    motors.setSpeeds(101, -100); //right rotation
    delay(525);
    motors.setSpeeds(0,0);
    delay(1000);
    motors.setSpeeds(-99, -102); //backward 1 foot
    delay(1375);
    motors.setSpeeds(0,0);
    delay(1000);
    //////////////////////////////
    //    END OF REVERSE BOX    //
    //////////////////////////////
  }
}
