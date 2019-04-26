
#include <Servo.h>

Servo arm;    // servo object for the arm
Servo pincer; // servo objejct for the pincer

//Defining min and max values for arm/pincer servos
#define ARM_MIN 0
#define ARM_MAX 120
#define PIN_MIN 0
#define PIN_MAX 60

int angleArm = 103;    // angle for arm servo to pickup block
int anglePin = 60;  // angle for the pincer servo to grab the block 
int inAngle = 0;

//function to pick up block
double pickup() {
 pincer.write(constrain(anglePin, PIN_MIN, PIN_MAX));  // opens the pincer the correct amount 
 delay(250);
 
 arm.write(constrain(angleArm, ARM_MIN, ARM_MAX)); //drop arm to get block (drop it like its hot)
 delay(500);
 int dSteps = findSteps(38.1, "distance");
 double trueDist = stepsToDistance(dSteps);
 linear(dSteps);
 pincer.write(constrain(anglePin-27.5, PIN_MIN, PIN_MAX)); //closes the pincer around the block
 delay(250);

 arm.write(-angleArm); //raises the arm to its original position

 return trueDist;
}

//drop block and reraise arm to 0
void deposit() {
  arm.write(constrain(angleArm-20,ARM_MIN, ARM_MAX)); //drops the arm to deposit block into mothership
  delay(250);
  pincer.write(constrain(anglePin, PIN_MIN, PIN_MAX));  // opens the pincer the correct amount 
  delay(250);
  arm.write(constrain(inAngle, ARM_MIN, ARM_MAX));  //raises the arm to return to initial position  
}
