#include <Servo.h>
#include <Arduino.h>
#include <math.h>             //used for arctan function to get angle to blocks
#include "movement.h"

String root = "ard";                //sets arduino to active
int accum = 0;                      //number of blocks picked up
double currentCoord[] = {4, 4};     //location of robot
double curAngle          = 0;       //current Degees Robot is facing 
char rx_byte = 0;                   //byte to be read
double blockX[] = {3,5,2,7,2,7};
double blockY[] = {4,5,1,3,0,6};
void setup() {
  // put your setup code here, to run once:
  pinMode(Pulse_FL, OUTPUT);
  pinMode(Dir_FL, OUTPUT);
  pinMode(Pulse_FR, OUTPUT);
  pinMode(Dir_FR, OUTPUT);
  pinMode(Pulse_BL, OUTPUT);
  pinMode(Dir_BL, OUTPUT);
  pinMode(Pulse_BR, OUTPUT);
  pinMode(Dir_BR, OUTPUT);
  digitalWrite(Dir_FL, HIGH);
  digitalWrite(Dir_FR, LOW);
  digitalWrite(Dir_BL, HIGH);
  digitalWrite(Dir_BR, LOW);
  arm.attach(9);
  pincer.attach(10); 
  arm.write(0);   // initial settings for motors & servos
  // pincer.write(0) // intial servo settings are for no movement. 
  //need to look more into servo settings and getting them to cooperate with the code.
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  if(root == "pi"){
    receiveData();
  }
  else{
    //travel to all locations
    for(int x = 0; x < 6; x++){
      findPath(blockX[x], blockY[x]);
    }
    //travels to 2,1 from 4,4
    //findPath(blockX[2], blockY[2]);
    //sendData(accum++);
  }
}
/*
//prints value to serial monitor
void logVal(String msg, double val){
  Serial.println(msg + String(val));
}

void logVal(String msg, String val){
  Serial.println(msg + val);
}
*/

String coordToString(int x, int y){
  return String(x) + ", " + String(y); 
}

double degToRad(int deg){
  return ((double) deg)/180*M_PI;
}

//finds if doubles are withing a threshold of each other
bool equal(double val, double newVal){
  if(abs(val - newVal) < .0005){ return true; }
  else{ return false; }
}

//find errors & make corrections if errors aren't within some tolerance (angle version)
/*double errorA(double angle, double trudangle){    
  double errorA = angle - trudangle;                                        
  // need to decide on a tolerance
  if(abs(errorA) >= 5){
    int steps = findSteps( errorA, "angle");
    double dangle = rotate(steps);   // may need to check the error again, but I think that wouldn't be necessary depending on the tolerance    
    double del = errorA - dangle;
    curAngle = curAngle - del;
    return (trudangle + dangle);
   }
  else{
    curAngle = curAngle - errorA;
    return trudangle;
  }
}*/

// distance version
/*double errorD(double dist, double trudist){
   double errorD = dist - trudist;    // might want to move the current coordinate updates into this function
   if(abs(errorD) >= 5){              // not sure though
     int steps = findSteps( errorD, "distance");
     double deldist = linear(steps);
     return deldist;
   }
   else {
    return trudist; 
   }
}*/

//find angle from current location to coordinate. 0 is north
double findAngle(int x, int y){
  double delX = (x-currentCoord[0]);  //change in x
  double delY = (y-currentCoord[1]);  //change in y
  if(equal(delX, 0)){
    if(delY > 0){ return 0; }
    else{ return 180; } 
  }
  else if(equal(delY, 0)){ 
    if(delX > 0){ return 90; }
    else{ return -90; }
  }
  else{
    double angle = atan2( (x-currentCoord[0]), (y-currentCoord[1]) );
    angle=angle*180/3.141592653;  //radians => degrees
    return angle;
  }
}

//finds distance to travel
double findDistance(double x1, double x2, double y1, double y2){
  double distance = -1;
  double sqval = sq((x2-x1)) + sq((y2-y1));
  if(sqval > 0){ distance = sqrt(sqval); }
  distance = distance * 304.8;      //convert distance to millimeters
  return (distance);
}

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

//finds path to travel to point (x,y) from currentCoord.
//currently finds straight line
void findPath(int x, int y){
//  logVal("Current x-val: ", currentCoord[0]);
//  logVal("Current y-val: ", currentCoord[1]);
//  logVal("Target location: ", coordToString(x, y));
  double targetAngle = findAngle(x, y);
  double dAngle   = helper_rotate(curAngle, targetAngle); //change in angle
  double distance = findDistance(currentCoord[0], x, currentCoord[1], y);
  //logVal("distance", distance/304.8);
  int    dSteps   = findSteps(distance, "distance");
  int    aSteps   = findSteps(dAngle,   "angle"   );
  
  double trueDistance = stepsToDistance(dSteps);
  double trueAngle    = stepsToAngle(aSteps);

//  logVal("trueAngle", trueAngle);
//  logVal("Steps: ", aSteps);
  //logVal("trueDistance", trueDistance/304.8);
  //run path with true values
  runPath(aSteps, dSteps);   //travels determined distance
  updateLocation(trueAngle, trueDistance);
  delay(7000);
}

//moves robot to new angle and moves distance
void runPath(int aSteps, int dSteps){
  rotate(aSteps); //rotate delta angle 
  linear(dSteps); //travel distance in straight line
}

void updateLocation(double trueAngle, double trueDistance){
  curAngle         = trueAngle;
  double rad       = degToRad(trueAngle);           //angle in radians
  //new location, convert distance to block location
  currentCoord[0] += sin(rad) * trueDistance/304.8;
  currentCoord[1] += cos(rad) * trueDistance/304.8;
//  String display = String(currentCoord[0]) + ", " + String(currentCoord[1]); 
//  logVal("New location: ", display);
}

//gets data from raspberry pi
void receiveData(){
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    root = "ard";
    
    // check if a number was received
    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      Serial.print("Number received: ");
      Serial.println(rx_byte);
    }
    else {
      Serial.println("Not a number.");
    }
  } // end: if (Serial.available() > 0)
}

void sendData(int val){
  Serial.println(val);
  root = "pi"; //sets pi to active
}
