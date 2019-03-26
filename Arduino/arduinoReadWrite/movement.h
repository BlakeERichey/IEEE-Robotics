#include <Servo.h>


// setup for stepper pins & declaring servos
int Pulse_FL = 2;
int Dir_FL = 3;
int Pulse_FR = 4;
int Dir_FR = 5;
int Pulse_BL = 6;
int Dir_BL = 7;
int Pulse_BR = 8;
int Dir_BR  = 11;

int angle;
int dist;
int i;

double angleConversionFactor    = 2.025*8;
double distanceConversionFactor = 1.064 *4;

void linear(int steps) // callable function for forwards and backwards movement
{
  if(steps>=0){
    for ( i = 0; i < steps; ++i){    //speed at 2.083 rev/s tweaking the 2nd delay can tweak the speed
      digitalWrite(Pulse_FL, HIGH);  //currently using 4 microsteps, this is being taken into account in
      digitalWrite(Pulse_FR, HIGH);  // findSteps function
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(200);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(400);
    }
  }
  else{                              // for going backwards if we need to.
    digitalWrite(Dir_FL, LOW);
    digitalWrite(Dir_FR, HIGH);
    digitalWrite(Dir_BL, LOW);
    digitalWrite(Dir_BR, HIGH);
    
    for ( i = 0; i < steps; ++i){    //speed at 2.083 rev/s tweaking the 2nd delay can tweak the speed
      digitalWrite(Pulse_FL, HIGH);  //currently using 4 microsteps, this is being taken into account in
      digitalWrite(Pulse_FR, HIGH);  // findSteps function
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(200);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(400);
    }
    digitalWrite(Dir_FL, HIGH);
    digitalWrite(Dir_FR, LOW);
    digitalWrite(Dir_BL, HIGH);
    digitalWrite(Dir_BR, LOW);
  }
}

//steps: number of steps equals to dAngle
void rotate(int steps)  // callable function for rotation
{                       // number of steps may need tweaking, depends on the weight distribution; 
 //                          which will have to wait until the robot is fully built
  if (steps >= 0) 
  {
    digitalWrite(Dir_FR, HIGH);
    digitalWrite(Dir_BR, HIGH);
    for ( i = 1; i <= steps; ++i)          //speed at 2.083 rev/s tweaking the 2nd delay can tweak the speed
    {
      digitalWrite(Pulse_FL, HIGH);       //currently using 4 microsteps, this is being taken into account in
      digitalWrite(Pulse_FR, HIGH);       // findSteps function
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(200);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(400);
    }
    digitalWrite(Dir_FR, LOW);
    digitalWrite(Dir_BR, LOW);   
  }
  else 
  {
    digitalWrite(Dir_FL, LOW);
    digitalWrite(Dir_BL, LOW);
    for(int i=0; i < abs(steps); i++){
      digitalWrite(Pulse_FL, HIGH);
      digitalWrite(Pulse_FR, HIGH);
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(200);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(400);
    }
    digitalWrite(Dir_FL, HIGH);
    digitalWrite(Dir_BL, HIGH);
  }  
}

double helper_rotate(double olddeg, double newdeg){
  double deldeg = newdeg - olddeg;
  return deldeg;
}

double stepsToDistance(int steps){
  return steps/distanceConversionFactor;
}

double stepsToAngle(int steps){
  return steps/angleConversionFactor;
}

//converts angle or distance to a number of corresponding steps
int findSteps(double val, String type){
  int steps = 0;
  if(type == "distance"){
    //find distance step qty
    steps = (int)floor(val*distanceConversionFactor);
  }
  else if(type == "angle"){
    //find angle step qty
    steps = (int)round(val*angleConversionFactor);

  }  
  return steps;
}
