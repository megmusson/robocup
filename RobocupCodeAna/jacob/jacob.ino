/* This is the god code, all code to run Wall-E will be written here*/
//#include <movingAvg.h>
#include <Servo.h>
// Define constants
#define FORWARDFAST 2000
#define FORWARDSLOW 1850
#define STOP 1500
#define BACKWARDFAST 1000
#define BACKWARDSLOW 1150
#define SENSORONE  A1
#define SENSORTWO  A0
#define SENSORTHREE A2
#define SENSORFOUR  A3
#define SENSORLEFT  A4
#define SENSORRIGHT  A5
#define SHORTMAX 590
#define MEDIUMMAX 590
#define FORWARDNUM 500
#define MEDIUMMIN 43.70944904
#define SHORTMIN 43.70944904
#define BACKWARDNUM -500


// Variables for calculating distance from IR sensors
float k1Short = 0.022648669896535;
float k2Short = 11.4466981524272;

float k1Medium = 0.022648669896535;
float k2Medium = 11.4466760237414;

float k1Long = 0.362045928;
float k2Long = 40.5482045360838;

// Define moving average objects
//movingAvg leftSense(10);
//movingAvg rightSense(10);

// Defin motor objects
Servo leftMotor;
Servo rightMotor;

// Define varaibles 
bool sense1;
bool sense2;
bool sense3;
bool sense4;


float senseIn1;
float senseIn2;
float senseIn3;
float senseIn4;
float senseIn5;
float senseIn6;


float sense1data;    // Store the analog values from the sensor 1
float sense2data;   // Store the analog values from the sensor 2
float sense3data;   // Store the analog values from the sensor 3
float sense4data;   // Store the analog values from the sensor 4
float senseLeftdata;   // Store the analog values from the sensor 5
float senseRightdata;   // Store the analog values from the sensor 6

//Define variables
float leftAvg;
float rightAvg;
int leftSpeed;
int rightSpeed;
float leftDist;
float rightDist;


// Functions
// ***********************************************************
// Function to cap the anlougue readings 
// ***********************************************************

float sensorRead(float sensorPinValue, int maxReading, float minReading) {
  float ans;
  if (sensorPinValue > maxReading) {
    ans = maxReading;
  } else if (sensorPinValue < minReading) {
    ans = minReading;
  } else {
    ans = sensorPinValue;
  }
  return ans;
}

// ***********************************************************
// Function to convert analouge sensor readings to distance
// ***********************************************************

float convertToDistance(float k2, float k1, float sensorAnalouge) {
  float voltsPerBit = 0.004882813; // number of volts which corresponds to each bit from analouge 
  float distance = k2/(sensorAnalouge*voltsPerBit) + k1;
  return distance;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Set baud rate to 9600
//  leftSense.begin();
//  rightSense.begin();
  leftMotor.attach(3);
  rightMotor.attach(2);

}

void loop() {
  
  // First step check sensors and cap them if they are above max reading

  // Read the sensor values and determine if values recieved are good, if not cap the values at the maximum range
  senseIn1 = sensorRead(analogRead(SENSORONE), SHORTMAX, SHORTMIN);

  senseIn4 = sensorRead(analogRead(SENSORFOUR), SHORTMAX, SHORTMIN);
  
  senseIn2 = sensorRead(analogRead(SENSORTWO), MEDIUMMAX, MEDIUMMIN);

  senseIn3 = sensorRead(analogRead(SENSORTHREE), MEDIUMMAX, MEDIUMMIN);


  // convert the sensor readings into distance
  sense1data = convertToDistance (k2Short, k1Short, senseIn1);
  sense2data = convertToDistance (k2Medium, k1Medium, senseIn2);
  sense3data = convertToDistance (k2Medium, k1Medium, senseIn3);
  sense4data = convertToDistance (k2Short, k1Short, senseIn4);
  senseLeftdata = convertToDistance (k2Long, k1Long, analogRead(SENSORLEFT));
  senseRightdata = convertToDistance (k2Long, k1Long, analogRead(SENSORRIGHT));
  

  // calculate the error between the IR sensors
  float smallError = abs(sense1data-sense4data); // Good for determining if weight is in front of robot
  float medError = abs(sense2data-sense3data); // Good for detereminig if object is in front of robot
  float longError = abs(senseLeftdata - senseRightdata); // IDK yet still need to figure out why i put this here
  float leftHanderror = abs(sense1data - sense2data);
  float rightHanderror = abs(sense3data - sense4data);


// ***********************************************************
// If no weight on either left or right then avoid obstacles
// ***********************************************************
    if (medError <= 1){
    // Special boundry case if something is directily in the front of the robot, turn left or right depending on suroundings

    // check what is on left side of robot.
     if (senseLeftdata < senseRightdata) {
      // If there is an obstacle on left turn right
    
      leftSpeed = (0.75*FORWARDNUM) + STOP;
      rightSpeed = (0.75*BACKWARDNUM) + STOP;
      
    } else if (senseRightdata < senseLeftdata) {
      
      // obstacle on right, turn left
      
      leftSpeed = (0.75*BACKWARDNUM) + STOP;
      rightSpeed = (0.75*FORWARDNUM) + STOP;
    } else {
      // if both in the very rare case are the same just turn left
      leftSpeed = (0.75*FORWARDNUM) + STOP;
      rightSpeed = (0.75*BACKWARDNUM) + STOP;
    }
  } else if (senseLeftdata <= 20 || (sense2data <= 15)) {
    // If obstacle close to left sensor turn right
      leftSpeed = (0.75*FORWARDNUM) + STOP;
      rightSpeed = (0.85*BACKWARDNUM) + STOP;

  } else if (senseRightdata <= 20 || (sense3data <= 15 )){
    // If obstacle close to right sensor turn left
    leftSpeed = (0.85*BACKWARDNUM) + STOP; // was 60%
    rightSpeed = (0.75*FORWARDNUM) + STOP; // Was 50%
 } else {
  leftSpeed = (0.75*FORWARDNUM) + STOP;
  rightSpeed = (0.75*FORWARDNUM) + STOP;
 }


 // Ccomand motors
 leftMotor.writeMicroseconds(leftSpeed);
 rightMotor.writeMicroseconds(rightSpeed);

 
//} else {
//  // ***********************************************************
//  // Weight is detected, want to turn towards it
//  // ***********************************************************
//  if (smallError < 2) {
//    // Weight should be directly in front of robot
//    leftSpeed = (0.5*FORWARDNUM) + STOP;
//    rightSpeed = (0.5*FORWARDNUM) + STOP;
//  } else if (weightLeft && !weightRight) {
//    // Turn left to reduce small error
//    leftSpeed = (0.5*FORWARDNUM) + STOP;
//    rightSpeed = (0.5*BACKWARDNUM) + STOP;
//  } else if (weightRight && !weightLeft) {
//    // Weight is to right so turn right 
//    leftSpeed = (0.5*BACKWARDNUM) + STOP;
//    rightSpeed = (0.5*FORWARDNUM) + STOP;
//  } else {
//    Serial.print("Small Error: \t");
//    Serial.println(smallError);
//  }
//  


  delay(500);
  Serial.print("Sensor 1 \t ");
  Serial.print(sense1data);
  Serial.print("\t Sensor 2 \t");
  Serial.println(sense2data);
  
//  Serial.print("\t Right error  ");
//  Serial.print(rightHanderror);
//  Serial.print("\t weightRight  ");
//  Serial.println(weightRight);
//  
}
