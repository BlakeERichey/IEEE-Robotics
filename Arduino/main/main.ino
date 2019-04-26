#include <Servo.h>
#include <Arduino.h>
#include <math.h>             //used for arctan function to get angle to blocks
#include "movement.h"
#include "direction.h"
#include "distance.h"
#include "arm.h"
#include "color.h"

//----------------State Manager Variables---------------
char   msg            = 0;          //read from pi
String root           = "pi";      //sets arduino to active
int    accum          = 0;          //number of blocks picked up
double curAngle       = 0;          //current Degees Robot is facing 
double currentCoord[] = {3.5, 3.5};     //location of robot
int    blockQty = 0;
double blockX[] = {4,6,2,7,2,7};    //blocks' Xcoordinates
double blockY[] = {2,5,1,3,0,6};    //blocks' Ycoordinates
double MotherX = 3;
double MotherY = 6;
double initialDir = 0;              //direction magnetometer says north is
double forwardDistanceFromArmToBlock = 13.0; //cm, minimum distance to pick up block
double backwardDistanceFromArmToBlock = 12.5;
bool   testCondition = true;        //used to test a single iteration
int LED = 37;
bool nE = false;
bool nW = false;
bool sE = false;

void setup() {
  // put your setup code here, to run once:
  //----------Movement setup----------
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
  
  //----------Arm Setup----------
  arm.attach(9);  // attaches the servo on pin 9 to the servo object arm
  pincer.attach(10);  // attaches the servo on pin 10 to the servo object pincer
  arm.write(inAngle);
  pincer.write(inAngle);

  //IR Sensor Servo
  myservo.attach(12);

  //----------Color Senser----------
  tcs.begin();

  //----------Magnetometer----------
  mag.enableAutoRange(true);
  mag.begin();
  initialDir = getHeading();
  setInitialDir(initialDir);

  //-----------LED Setup------------
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  //----------Serial Setup----------
  Serial.begin(9600);
  delay(2500);
}

void loop() {
  /*  
  while(testCondition == true){
    Serial.println(F("\nSend any character to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    int aSteps = findSteps(1, "angle");
    for(int i = 0; i < 400; i++){
      zeroing();
      rotate(aSteps);
    }
    Serial.print("X offset: ");
    Serial.println(((magMaxX + magMinX)/2));
    Serial.print("Y offset: ");
    Serial.println(((magMaxY + magMinY)/2));
    testCondition = false;
  }
  */
  
  Serial.println("0");
  if(root == "pi"){
    //readBlockData();  //will wait until data received
    Serial.println("1");
    root = "ard";
  }else if(root == "ard"){
    delay(1000);
    Serial.println("2");
    if(testCondition){
      logVal("Traveling!", "");
      findPath(blockX[0], blockY[0]); //4,2
      delay(100);
      sE = true;
      nW = false;
      nE = false;
      findBlock();
      double dist = lowSensor();
      while(dist < 19){
        if(dist > forwardDistanceFromArmToBlock){
          double dtt = (dist - forwardDistanceFromArmToBlock)*10; //distance to travel
          int distanceSteps = findSteps(dtt, "distance");
          logVal("Distancesteps: ", distanceSteps);
          linear(distanceSteps);
          double truDist = stepsToDistance(distanceSteps);
          updateLocation(truDist);
        }
        else if(dist < backwardDistanceFromArmToBlock){
          double dtt = (dist - backwardDistanceFromArmToBlock)*10;
          int distanceSteps = findSteps(dtt, "distance");
          linear(distanceSteps);
          double truDist = stepsToDistance(distanceSteps);
          updateLocation(truDist);
        }
        updateLocation(pickup());
        delay(100);
        dist = lowSensor();
      }
      delay(200);
      returnToCenter();
      delay(50);
      realign();
      delay(200);
      findPath(MotherX, MotherY);
      delay(100);
      int dSteps = findSteps(-50, "distance");
      double distance = stepsToDistance(dSteps);
      linear(-dSteps);
      updateLocation(-distance);
      deposit();
      delay(200);
      linear(2*dSteps);
      updateLocation(2*distance);
      delay(50);
      returnFromMother();
      delay(50);
      realign();
      delay(200);
      findPath(blockX[1], blockY[1]); //6,5
      delay(100);
      nW = false;
      nE = true;
      sE = false;
      findBlock();
      dist = lowSensor();
      while(dist < 19){
        if(dist > forwardDistanceFromArmToBlock){
          double dtt = (dist - forwardDistanceFromArmToBlock)*10; //distance to travel
          int distanceSteps = findSteps(dtt, "distance");
          logVal("Distancesteps: ", distanceSteps);
          linear(distanceSteps);
          double truDist = stepsToDistance(distanceSteps);
          updateLocation(truDist);
        }
        else if(dist < backwardDistanceFromArmToBlock){
          double dtt = (dist - backwardDistanceFromArmToBlock)*10;
          int distanceSteps = findSteps(dtt, "distance");
          linear(distanceSteps);
          double truDist = stepsToDistance(distanceSteps);
          updateLocation(truDist);
        }
        updateLocation(pickup());
        delay(100);
        dist = lowSensor();
      }
      delay(200);
      returnToCenter();
      delay(200);
      findPath(MotherX, MotherY);
      delay(100);
      linear(-dSteps);
      updateLocation(-distance);
      deposit();
      delay(200);
      linear(2*dSteps);
      updateLocation(2*distance);
      delay(50);
      returnFromMother();
      delay(500);
      realign();
      digitalWrite(LED, HIGH);
      testCondition = false;
    }
  }
  
//  
//    updateLocation(pickup());
//    deposit();
  
  /*
  delay(1000);
  
  //testing
  double distance = lowSensor();
  logVal("Distance: ", distance);
  if(distance > 20){
    //Serial.println("object too far");
    rotate(-50);
  }else if(distance > distanceFromArmToBlock){
    //logVal("Object detected!", "");
    //Serial.println("Moving to object");
    double dtt = (distance - distanceFromArmToBlock)*10; //distance to travel
    int distanceSteps = findSteps(dtt, "distance");
    logVal("Distancesteps: ", distanceSteps);
    linear(distanceSteps);
    updateLocation(0, stepsToDistance(distanceSteps));
  }else{
    Serial.println("Picking up object");
    pickup();
    deposit();
  }
  
  /*
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
  */
}

//prints value to serial monitor
void logVal(String msg, double val){
  Serial.println(msg + String(val));
}

void logVal(String msg, String val){
  Serial.println(msg + val);
}


//distance: distance to object
//bool checkIfObstacle(double distance){      // these currently call lowSensor, they need to call the High right sensor
//  double a, b, c, theta;
//  a = distance;
//  b = 8.211;
//  c = sqrt(sq(a)+sq(b));
//  theta = asin(b/c);
//  int steps = findSteps(-theta, "angle");
//  rotate(steps);
//  double dist = highRightSensor();
//  if (dist <= 30){
//    for (int i = 0; i < 19; i++){         // might not need this loop at all
//      dist = dist + highRightSensor();  
//    }
//    dist = dist/20;
//    return true;
//  }
//  else{
//    return false;
//  }
//}


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

bool isNE(int x, int y){
  return (currentCoord[0] < x && currentCoord[1] < y);
  nE = true;
}

bool isNW(int x, int y){
  return (currentCoord[0] > x && currentCoord[1] < y);
  nW = true;
}

bool isSE(int x, int y){
  return (currentCoord[0] < x && currentCoord[1] > y);
  sE = true;
}

//find angle from current location to coordinate. 0 is north
double findAngle(double x, double y){
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
    angle=angle*180/M_PI;  //radians => degrees
    return angle;
  }
}

void turnTo(double targetAngle){
  double delAngle = helper_rotate(curAngle, targetAngle);
  int steps = findSteps(delAngle, "angle");
  rotate(steps);
}

//finds distance to travel
double findDistance(double x1, double x2, double y1, double y2){
  double distance = -1;
  double sqval = sq((x2-x1)) + sq((y2-y1));
  if(sqval > 0){ distance = sqrt(sqval); }
  distance = distance * 304.8;      //convert distance to millimeters
  return (distance);
}

//finds path to travel to point (x,y) from currentCoord.
//currently finds straight line
//goes to corner of sq
void findPath(int x, int y){
  //get destination
  nE = false;
  nW = false;
  sE = false;
  int targetX = x; int targetY = y;
  if(isNE(x, y)){ targetX-=1; targetY-=1; }
  else if(isNW(x, y)){ targetY-=1; }
  else if(isSE(x, y)){ targetX-=1;}
  
  //find how far to move, and angle
  Serial.print("Destionation: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  double targetAngle = findAngle(targetX, targetY);
  logVal("targetAngle", targetAngle);
  double dAngle   = helper_rotate(curAngle, targetAngle); //change in angle
  double distance = findDistance(currentCoord[0], targetX, currentCoord[1], targetY);
  distance -= 127;
  logVal("distance", distance/304.8);
  int    dSteps   = findSteps(distance, "distance");
  int    aSteps   = findSteps(dAngle,   "angle"   );
  
  double trueDistance = stepsToDistance(dSteps);
  double trueAngle    = stepsToAngle(aSteps);
  
  //move
  while(abs(getHeading() - targetAngle) > .5){
    Serial.println("Turning..."); 
    turnTo(targetAngle);
    updateLocation(0);
    logVal("Heading: ", getHeading());
  }
  linear(dSteps);
  updateLocation(trueDistance);
  delay(1000);
}


// function to find and store mothership data.
void findMotherShip(){
  bool found = false;
//  double motherShipAngle, motherShipX, motherShipY;
  double pathX[] = {2, 6};
  double pathY[] = {4, 4};    // set points for checking, if we don't find it on the first 360
  bool isgreen = false;                          
  int steps = findSteps(15.0, "angle");
  for (int i = 0; i <24; i++){            // do 360 degree check on the starting point.
    isgreen = isGreenPresent();
    if( isgreen == true){                 // if green is present in the current direction, break out of the loop
      found = true;
      break;
    }
    rotate(steps);                        // rotate 10 degrees
  }
  if (found != true){                     // if the mothership isn't found, 
    turnTo(0.0);
    delay(250);
    for (int i = 0; i <= 1; i++){         //  go to the next point and do a scan there
      double targetAngle = findAngle(pathX[i], pathY[i]);
      turnTo(targetAngle);
      double distance = findDistance(currentCoord[0], pathX[i], currentCoord[1], pathY[i]);
      int dSteps = findSteps(distance, "distance");
      linear(dSteps);
      turnTo(0.0);
      if (i < 1){ turnTo(150.0); }
      else { turnTo(-30.0); }
      for (int i = 0; i < 16; i++){       // scan from curAngle+120 to curAngle-120
                                          
        isgreen = isGreenPresent();
        if( isgreen == true){
          found = true;                   // if we find it, set found to true, and break out of the loop
          break;
        }
        rotate(steps);                    // first one is CCW
      }
      if( found == true){                 // if we found the mothership, break out of the loop
        break;                            // this may seem redundant, but we have to do this twice since there are 2 for loops
      }
      turnTo(0.0);
    }
  }
  else{
    double dtt= lowSensor();              // dtt = distance to target
                                          // need to step towards the source of the green light, in increments of our sensor range; 14cm, or 140 mm (doing 130 mm for now)
    double distance = 130.0;
    int dSteps = findSteps(distance, "distance");
    int iterations = 0;
    while (dtt > 14 && iterations < 6){
      linear(dSteps);
      delay(150);
      dtt = lowSensor();                   // sensor scan of the lowSensor, and the highSensor   high sensor isn't currently set up, but will be soon(TM)
      iterations += 1;
    }
  }
}

//receives data from pi and loads it into state variables
void receiveBlockData(){
  int qty = 0;
  bool x = true;
  int index = 0;
  bool runBit = true; //continue reading data

  while(runBit){
    if (Serial.available() > 0) { //check if character is available.
      char val = Serial.read();
      if(qty==0){ //received qty value
        qty = int(val)%48;
        blockQty = qty;        
      }else{ //received block location
        if(x){ //load x values
          blockX[index++] = int(val)%48;
          if(index >= qty){
            index = 0;
            x = false;
          }
        }else{ //load y values
          blockY[index++] = int(val)%48;
          if(index >= qty){
            runBit = false;
          }
        }
      }
    }
  }
}


// funtion to rotate up to 180 degrees (90 CCW, then 90 Cw), and find the lowest distance from the lowSensor
// also takes in optional double for the change in angle, this defaults to false & -1 respectively
void rotateScan(double distance, double delAngle = -1){
  double lowDistAngle;
  double lowDist = distance;
  double returnAngle = curAngle;
  int steps = findSteps(delAngle, "angle");           // this might have to change for accuracy reasons, not sure right now
  for(int angle = 0; angle < 90; angle++){      // turn 90 degrees CCW, scan after each degree
    rotate(steps);
    double dist2 = lowSensor();
    if( dist2 < lowDist){                      // if the new scan is lower than the lowest scan
      lowDist = dist2;                         // set the lowest scan to the current scan
      lowDistAngle = curAngle;                  // store the angle of the lowest scan
    }
    else if(dist2 > lowDist){                  // if the distance is growing, break out & go to the next function.
      break;
    }
  }
  turnTo(returnAngle);                          // go back to the original angle
  for(int angle = 0; angle < 90; angle++){      // loop to go 90 degrees CW, still scanning after each degree
    rotate(-steps);                             // the 90 degrees is assumming delAngle is not specified, it will be proportionally more or less depending on specification
    double dist2 = lowSensor();
    if(dist2 < lowDist){
      lowDist = dist2;
      lowDistAngle = curAngle;                  // same stuff
    }
    else if(dist2 > lowDist){
      break;
    }
  }
  if (lowDist < 14){                           // rotate to the angle that had the lowest distance
    turnTo(lowDistAngle);
  }
}


//moves robot to new angle and moves distance
void runPath(int aSteps, int dSteps){
  rotate(aSteps); //rotate delta angle 
  linear(dSteps); //travel distance in straight line
}

//trueAngle:    angle robot rotated
//trueDistance: distance robot moved
void updateLocation(double trueDistance){
  //curAngle         = curAngle + trueAngle;
  double newCurAngle = getHeading();
  double trueAngle   = newCurAngle - curAngle;
  curAngle           = newCurAngle;
  double rad         = degToRad(curAngle);           //angle in radians
  //new location, convert distance to block location
  currentCoord[0] += sin(rad) * trueDistance/304.8;
  currentCoord[1] += cos(rad) * trueDistance/304.8;
  String display = String(currentCoord[0]) + ", " + String(currentCoord[1]); 
  logVal("New location: ", display);
  logVal("New angle: ", curAngle);
}

//blockNum: block we are currently searching for
//loc:      location of block currently being searched for
bool findBlock(){
  //values returned
  int distanceToBlock = 0;
  int angleToBlock    = 0;

  //Values to solve problem
  bool found = false;
  double angleToTurn = -90;        //left of right
  double angleToTurn2 = -45;
  if(nE){
    angleToTurn = 90;
    angleToTurn2 = 135;
  }
  else if(nW){
    angleToTurn = 0;
    angleToTurn2 = 45;
  }
  else if(sE){
    angleToTurn = -179;
    angleToTurn2 = -135;
  }
  double distanceToTravel = 304.8;      //1 ft in mm
  double dSteps = findSteps(175, "distance");
  double distance = 0;  //distance travelled

  //---------------1st Pass---------------
  turnTo(angleToTurn); //turns until it reaches angleToTurn
  updateLocation(0);
  double lowD = lowSensor(); 
  found = lowD < 19;
  double newAngle = 0;
  while ( newAngle > -90 && !found){
    //Serial.println("1");
    int steps = findSteps(1, "angle");
    rotate(-steps);
    double angleRotated = stepsToAngle(steps);
    newAngle -= angleRotated;
    lowD = lowSensor(); 
    found = lowD < 19;
    if(found){
      lowD = lowSensor();
      found = lowD < 19;
    }
  }
  updateLocation(0);
  

  if(!found){//---------------2nd Pass---------------
    turnTo((curAngle + angleToTurn)/2);
    updateLocation(0);
    linear(dSteps);
    updateLocation(stepsToDistance(dSteps));
    turnTo(angleToTurn2);
    updateLocation(0);
    lowD = lowSensor();
    found = lowD < 19;
    newAngle = 0;
    while(newAngle > -180 && !found){
      Serial.println("2");
      int steps = findSteps(1, "angle");
      rotate(-steps);
      double angleRotated = stepsToAngle(steps);
      newAngle -= angleRotated;
      lowD = lowSensor(); 
      found = lowD < 19;
      if(found){
        lowD = lowSensor();
        found = lowD < 19;
      }
    }
    updateLocation(0);
  }
  updateLocation(0);
  if(found){
    Serial.println("Found");
    dSteps = findSteps(20, "distance");
    linear(dSteps);
    updateLocation(stepsToDistance(dSteps));
    lowD = lowSensor();
    zeroInOnBlock(lowD); 
  }else{
    Serial.println("Not Found"); 
    return 0; //false, not found
  }
  return 1;
}

//realigns robot to 0 degrees. Fixing errors in rotation accumulated over time
void realign(){
  turnTo(0);
  updateLocation(0);
  //find difference from 0 degrees. Rotate until 0 degrees reached
  while(abs(curAngle) > 0.35){ 
    double aSteps = findSteps(-curAngle, "angle");
    rotate(aSteps);
    updateLocation(0);
  }
}

// function to return to the center from the mothership
// was having some complications with the normal return to center function specifically from the mothership
void returnFromMother(){
  double tarAngle = findAngle(3.5, 3.5);
  while(abs(getHeading() - tarAngle) > .5){
    Serial.println("Turning..."); 
    turnTo(tarAngle);
    updateLocation(0);
    logVal("Heading: ", getHeading());
  }
  double dist = findDistance(currentCoord[0], 3.5, currentCoord[1], 3.5);
  int dSteps = findSteps(dist, "distance");
  double trueDistance = stepsToDistance(dSteps);
  linear(dSteps);
  updateLocation(trueDistance);
}


void returnToCenter(){
  findPath(4,4);
  /*
  if(equal(currentCoord[0], 3)){
    if(equal(currentCoord[1], 3)){
      turnTo(45);
    }else{ turnTo(135); }
  }else if(equal(currentCoord[0], 4)){
    if(equal(currentCoord[1], 3)){
      turnTo(-45);
    }else{ turnTo(-135); }
  }
  */
  double tarAngle = findAngle( 3.5, 3.5);
  logVal("targetAngle", tarAngle);
  while(abs(getHeading() - tarAngle) > .5){
    Serial.println("Turning..."); 
    turnTo(tarAngle);
    updateLocation(0);
    logVal("Heading: ", getHeading());
  }
  double dist = findDistance(currentCoord[0], 3.5, currentCoord[1], 3.5);
  int dSteps = findSteps(dist, "distance");     //travel to center of square
  double trueDistance = stepsToDistance(dSteps);
  linear(dSteps);
  updateLocation(trueDistance);
}


// function to zero in on a block, to be run either in or after findBlock function
// this attempts to aim at the center of a block, & return the shortest distance from the robot to the block.
// hopefully to deal with angled blocks
void zeroInOnBlock(double distance){
  double CCWAngle, CWAngle;
  double returnAngle = curAngle;
  int delStepsCCW = 0;
  int delStepsCW = 0;
  int steps = findSteps(-.5, "angle");           // this might have to change for accuracy reasons, not sure right now
  for(int angle = 0; angle < 20; angle++){      // turn ~20 degrees CCW, scan after each degree
    rotate(steps);
    double dist2 = lowSensor();
    delStepsCCW += steps;
    if( dist2 < distance){                      // if the new reading is lower than the lowest reading
      distance = dist2;                         // set the lowest reading to the current reading
    }
    else if(dist2 > 18.2){                  // if the distance is growing, break out & go to the next function.
       break;
    }
  }
  updateLocation(0);
  CCWAngle = curAngle;
  //turnTo(returnAngle);                          // go back to the original angle
  rotate(-delStepsCCW);
  updateLocation(0);
  for(int angle = 0; angle < 20; angle++){      // loop to go ~20 degrees CW, still scanning after each degree
    rotate(-steps);
    double dist2 = lowSensor();
    delStepsCW -= steps;
    if(dist2 < distance){
      distance = dist2;                         // same stuff
    }
    else if(dist2 > 18.5){
      dist2 = lowSensor();
      if (dist2 > 18.5){                          // redudancy checking
        dist2 = lowSensor();
        if (dist2 > 18.5){                          // redudancy checking
          break;
        }
      }
    }
  }
  
  updateLocation(0);
  CWAngle = curAngle;
  double tarAngle = (CCWAngle + CWAngle)/2;
  int delSteps = (delStepsCCW + delStepsCW)/2;
  delSteps -= delStepsCW;
  rotate(delSteps);
  //turnTo(tarAngle);
  updateLocation(0);
}


//gets data from raspberry pi
void receiveData(){
  if (Serial.available() > 0) { // is a character available?
    msg = Serial.read();        // get the character
    Serial.println(msg);
  }
}

void sendData(int val){
  Serial.println(val);
  root = "pi"; //sets pi to active
}
