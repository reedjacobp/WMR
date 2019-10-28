int G;
int vel_left;
int vel_right;
int countsLeft;
int countsRight;
int G1 = 0;
int G2 = 0;
int G3 = 0;
int G4 = 0;
int G5 = 0;
int K = 10000;

#include <Wire.h>
#include <LSM6.h>
#include <Romi32U4.h>
#include <PIDController.h>

LSM6 imu;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4Encoders encoders;
PIDController pid_left, pid_right;

void setup() 
{
  Serial.begin(9600);
  pid_left.begin();
  pid_left.setpoint(0);
  pid_left.tune(0, 0, 0);
  pid_left.limit(0,100);
  
  pid_right.begin();
  pid_right.setpoint(0);
  pid_right.tune(0, 0, 0);
  pid_right.limit(-100,0);
  Wire.begin();
  if (!imu.init())
  {
    ledRed(1);
    while (1)
    {
      Serial.println(F("Failed to detect the LSM6."));
      delay(100);
    }
  }

  imu.enableDefault();

  imu.writeReg(LSM6::CTRL2_G, 0b10001000);

  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);

  buttonA.waitForButton();
  delay(2000);
}

void loop() 
{
  imu.read();
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
  // 5 frame averager filter to filter sensor noise
  G5 = G4;
  G4 = G3;
  G3 = G2;
  G2 = G1;
  G1 = (int)imu.g.z;
  G = (G1 + G2 + G3 + G4 + G5)/5;

  int vel_left = 100 - pid_left.compute(G);
  int vel_right = 100 + pid_right.compute(G);

// K is the gain of the controller
//  vel_left = 100 - G;
//  vel_right = 100 + G;
  motors.setSpeeds((int)vel_left, (int)vel_right);
  
//  if (countsLeft > countsRight)
//  {
//    motors.setSpeeds((int)vel_left, (int)vel_right*4);
//  }
//  else if (countsLeft < countsRight)
//  {
//    motors.setSpeeds((int)vel_left*4, (int)vel_right);
//  }

  delay(50); //Run at 20Hz
}
