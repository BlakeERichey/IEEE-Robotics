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

double angleConversionFactor    = 2.025*32;
double distanceConversionFactor = 1.064 *16;

void linear(int steps) // callable function for forwards and backwards movement
{
  int j = 0; 
  unsigned long currTime;                                     // accelerates to 2.083 rev/s, or up to half the steps
  if (steps >=0){                                             // decelerates for the same amount; extra steps are done at a constant speed
   currTime = millis();
   while (millis() - currTime < 1000){
    if(j< (steps/2)){
      digitalWrite(Pulse_FL, HIGH);
      digitalWrite(Pulse_FR, HIGH);
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(100);
      delayMicroseconds(500-(.5*(millis()-currTime)));
      j++;
    }
    else{
      break;
    }
   }
   for(int i =0; i < (steps - (2*j)); i++){
     digitalWrite(Pulse_FL, HIGH);
     digitalWrite(Pulse_FR, HIGH);
     digitalWrite(Pulse_BL, HIGH);
     digitalWrite(Pulse_BR, HIGH);
     delayMicroseconds(50);
     digitalWrite(Pulse_FL, LOW);
     digitalWrite(Pulse_FR, LOW);
     digitalWrite(Pulse_BL, LOW);
     digitalWrite(Pulse_BR, LOW);
     delayMicroseconds(100);
   }
   currTime = millis();
   for (int i = 0; i < j; i++){
     digitalWrite(Pulse_FL, HIGH);
     digitalWrite(Pulse_FR, HIGH);
     digitalWrite(Pulse_BL, HIGH);
     digitalWrite(Pulse_BR, HIGH);
     delayMicroseconds(50);
     digitalWrite(Pulse_FL, LOW);
     digitalWrite(Pulse_FR, LOW);
     digitalWrite(Pulse_BL, LOW);
     digitalWrite(Pulse_BR, LOW);
     delayMicroseconds(100);
     delayMicroseconds(.5*(millis() - currTime));
   }
  }
  else{
    digitalWrite(Dir_FL, LOW);
    digitalWrite(Dir_FR, HIGH);
    digitalWrite(Dir_BL, LOW);
    digitalWrite(Dir_BR, HIGH);
    currTime = millis();
    while (millis() - currTime < 1000){
      if(j< abs(steps/2)){
        digitalWrite(Pulse_FL, HIGH);
        digitalWrite(Pulse_FR, HIGH);
        digitalWrite(Pulse_BL, HIGH);
        digitalWrite(Pulse_BR, HIGH);
        delayMicroseconds(50);
        digitalWrite(Pulse_FL, LOW);
        digitalWrite(Pulse_FR, LOW);
        digitalWrite(Pulse_BL, LOW);
        digitalWrite(Pulse_BR, LOW);
        delayMicroseconds(100);
        delayMicroseconds(500-(.5*(millis()-currTime)));
        j++;
      }
      else{
        break;
      }
    }
    for(int i =0; i < (abs(steps) - (2*j)); i++){
      digitalWrite(Pulse_FL, HIGH);
      digitalWrite(Pulse_FR, HIGH);
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(100);
    }
    currTime = millis();
    for (int i = 0; i < j; i++){
      digitalWrite(Pulse_FL, HIGH);
      digitalWrite(Pulse_FR, HIGH);
      digitalWrite(Pulse_BL, HIGH);
      digitalWrite(Pulse_BR, HIGH);
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(100);
      delayMicroseconds(.5*(millis() - currTime));
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
 if (abs(steps) <= 200){
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
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(600);
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
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(600);
    }
    digitalWrite(Dir_FL, HIGH);
    digitalWrite(Dir_BL, HIGH);
  }
 }
 else{
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
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(250);
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
      delayMicroseconds(50);
      digitalWrite(Pulse_FL, LOW);
      digitalWrite(Pulse_FR, LOW);
      digitalWrite(Pulse_BL, LOW);
      digitalWrite(Pulse_BR, LOW);
      delayMicroseconds(250);
    }
    digitalWrite(Dir_FL, HIGH);
    digitalWrite(Dir_BL, HIGH);
  }
 }
  //double truAngle = stepsToAngle(steps);
  //updateLocation(truAngle, 0);  
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
//val: distance in mm if type = distance
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
