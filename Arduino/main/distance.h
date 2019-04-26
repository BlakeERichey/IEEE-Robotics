//Used to find distance using IR sensors from the robot to an object

//initialize IR variables
Servo myservo; 
int    ir_sensor0 = A1; //center low
//int    ir_sensor1 = A2; //low left side
int    ir_sensor2 = A2; //rotating front sensor
int    pos        =  0; //variable to hold sensor/servo angle
float  volts; 
double cm;
double lightSensorConversionFactor = .0048828125;

//returns distance to an object from bottom, center, sensor, in cm
double lowSensor(){
  double sampleSum = 0;
  int samples[100];

  for(int reading = 0; reading < 100; reading++){
    volts = analogRead(ir_sensor0) * lightSensorConversionFactor;
    cm = 13* pow(volts, -1);    //distance in cm
    samples[reading] = cm;
    sampleSum += cm;
  }

  double meanSample = sampleSum / 100.0;

  double sqDevSum = 0.0;

  for(int sample = 0; sample < 100; sample++){
    sqDevSum += pow((meanSample - double(samples[sample])), 2);
  }

 double stDev = sqrt(sqDevSum/100.0);

 double rv = 100;
 if(stDev < .95){
   rv = meanSample;
 }

 return rv;
}

int lowLeftSensor(){
  volts = analogRead(ir_sensor2) * lightSensorConversionFactor;
  cm = 13* pow(volts, -1);    //distance in cm
  
  return cm/1;
}

//rotating sensor
//returns distance to object
double blockSensor(){
  double sampleSum = 0;
  double samples[100];
  for(int index = 0; index<100; index++){
    volts = analogRead(ir_sensor2) * lightSensorConversionFactor;
    cm = 13* pow(volts, -1);    //distance in cm
    samples[index] = cm;
    sampleSum += cm;
  }

  //std Dev of sample
  double dev = stDev(samples, 100);

  double rv = 100.0;
  if(dev < .95){
    rv = sampleSum / 100.0;
  }
  return rv;
}

//returns angle at which object was found
double blockSensorSweep(){
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    double distance = blockSensor();
    if(distance <= 20){ return pos; }
  }
  pos = 90;
  myservo.write(pos);  //return to center
  return 360; //nothing found
}
