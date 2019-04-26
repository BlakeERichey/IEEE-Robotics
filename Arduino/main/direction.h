//Handles Orientation of Robot.

#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */ 
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
double compensation_constant = 0; //initial reading for magnetometer

// for zeroing the magnetometer
double magMaxX = 0;
double magMaxY = 0;
double magMinX = 0;
double magMinY = 0;

//arr: array of values to find std dev of
//qty: size of array/number of indexes to include in result
double stDev(double arr[], int qty) {
  double sum = 0;
  for (int index = 0; index < qty; index++) {
    sum += arr[index];
  }

  double meanSample = sum / double(qty);

  double sqDevSum = 0;

  for (int index = 0; index < qty; index++) {
    sqDevSum += pow((meanSample - arr[index]), 2);
  }

  double stDev = sqrt(sqDevSum / double(qty));
  return stDev;
}

void zeroing(){
  for(int index = 0; index < 100; index++){

    sensors_event_t event; mag.getEvent(&event);
    double magX = event.magnetic.x;
    double magY = event.magnetic.y;
    if(magX > magMaxX){
      magMaxX = magX;
    }
    if(magX < magMinX){
      magMinX = magX;
    }
    if(magY > magMaxY){
      magMaxY = magY;
    }
    if(magY < magMinY){
      magMinY = magY;
    }
  }
}

//returns angle robot is facing
double getHeading(){
  double sampleSum = 0;
  double samples[100];
  
  for(int index = 0; index < 100; index++){
    /* Get a new sensor event */ 
    sensors_event_t event; mag.getEvent(&event);
    // Calculate the angle of the vector y,x
    double magX = event.magnetic.x - 9.73;
    double magY = event.magnetic.y + .86;
    double heading = atan2(magY, magX) * 180/M_PI;
    if (heading < -180)
    {
      heading += 360;
    }
    if(heading > 180){
      heading -= 360;
    }
    samples[index] = heading;
    sampleSum += heading;
  }

  double dev = stDev(samples, 100);
  if(dev > 1.5){ return getHeading(); }
  else{ //heading
    double heading = sampleSum/100.0;
    //Serial.print("compensation_constant: ");
    //Serial.println(compensation_constant);
    heading -= compensation_constant;
    if (heading < -180)
    {
      heading += 360;
    }
    if(heading > 180){
      heading -= 360;
    }
    return heading; 
  }
}

setInitialDir(double val){
  compensation_constant = val;
}
