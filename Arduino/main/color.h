/*
 * Adafruit TCS34725 RGB Sensor Demo Code
 * Written by David Archuleta and Nevid Harji...ish
  */
 
#include <Wire.h>                                                 //include the I2C library to communicate with the sensor
#include "Adafruit_TCS34725.h"                                    //include the sensor library//Sloke: what things do these contain?

byte gammatable[256];                                             // our RGB -> eye-recognized gamma color//Sloke: what is this used for?
 
                                                                  //Create an instance of the TCS34725 Sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
 
double distanceArr(int arr[], int arr2[], int arrSize) {
  int index = 0;
  double distance = 0;
  for (index; index < arrSize; index++) {
    distance += pow((arr[index] - arr2[index]), 2);
  }
  distance = pow(distance, 1.0 / arrSize);
  return distance;
}

String classifyColor(int r, int g, int b) {
  int currentArr[] = { r, g, b }; //color passed to function
  String classes[] = { 
    "red", 
    "cyan", 
    "blue",
    //"grey",
    "black", 
    "green", 
    "white",
    //"brown",
    "orange", 
    "yellow", 
    "magenta" 
  };

  int red[]     = { 192,   0,   0};
  int cyan[]    = {   0, 255, 255};
  int blue[]    = {   0,   0, 255};
  //int grey[]    = { 128, 128, 128};
  int black[]   = {   0,   0,   0};
  int green[]   = {   0, 183,   0};
  int white[]   = { 255, 255, 255};
  //int brown[]   = { 101,  67,  33};
  int orange[]  = { 255, 165,   0};
  int yellow[]  = { 255, 255,   0};
  int magenta[] = { 255,   0, 255};


  struct distance //distance from color array
  {
    int rd;
    int cd;
    int bd;
    //int grd;
    int bkd;
    int gd;
    int wd;
    //int brd;
    int od;
    int yd;
    int md;
  };

  distance deltas = { //distances from rgb to each vertex of RGB cube
    distanceArr(currentArr, red,     3), 
    distanceArr(currentArr, cyan,    3),
    distanceArr(currentArr, blue,    3),
    //distanceArr(currentArr, grey,    3),
    distanceArr(currentArr, black,   3),
    distanceArr(currentArr, green,   3),
    distanceArr(currentArr, white,   3),
    //distanceArr(currentArr, brown,   3),
    distanceArr(currentArr, orange,  3),
    distanceArr(currentArr, yellow,  3),
    distanceArr(currentArr, magenta, 3),
  };

  //----------Find minimum distance----------
  double dArr[] = { 
    deltas.rd, 
    deltas.cd, 
    deltas.bd,
    //deltas.grd, 
    deltas.bkd, 
    deltas.gd, 
    deltas.wd,
    //deltas.brd,
    deltas.od, 
    deltas.yd, 
    deltas.md 
  };

  // cout << "Display Distance Array" << endl;
  // for(int index = 0; index < 9; index++){
  //   cout << dArr[index] << " ";
  // }
  // cout << endl;

  int classIndex = 0; //index of closest color for classifier
  double minVal = dArr[0];
  for (int i = 0; i < 9; i++) {
    if (dArr[i] < minVal) {
      minVal = dArr[i];
      classIndex = i;
    }
  }

  return classes[classIndex];
}

String colorSense() {
  uint16_t clear, red, green, blue;                                 //declare variables for the colors// what is clear? I think we used a keyword.
 
  tcs.setInterrupt(false);                                          // turn on LED
 
  delay(60);                                                        // takes 50ms to read
 
  tcs.getRawData(&red, &green, &blue, &clear);                      //read the sensor
 
  tcs.setInterrupt(true);                                           // turn off LED
 
  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;

//  /
  
  //TESTING
  return classifyColor( (int)r, (int)g, (int)b );
}

//takes readings at every .1 sec for a total .6 sec to see if green was seen
bool isGreenPresent(){
  String colors[6];

  for(int index = 0; index < 6; index++){
    colors[index] = colorSense();
    delay(100);
  }

  bool containsGreen = false;
  for(int index = 0; index < 6; index++){
    if(colors[index] == "green"){
      containsGreen = true;
      break;
    }
  }

  return containsGreen;
}
