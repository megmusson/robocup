/********************************************************************************
                                 ROBOCUP TEMPLATE
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include<stdio.h>


//#include <Herkulex.h>             //smart servo
#include <Adafruit_TCS34725.h>      //colour sensor
//#include <Wire.h>                   //for I2C and SPI

// Custom headers
#include "motors.h"
//#include "sensors.h"
#include "weight_collection.h"
//#include "return_to_base.h"
int weight_pos;

#include "IRfilter.h"

// Pin deffinitions
#define IO_POWER  49
#define BAUD_RATE 9600
#define echoPin 27 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 26 //attach pin D3 Arduino to pin Trig of HC-SR04
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
int timecounter = 0;

//**********************************************************************************
// Local Definitions
//**********************************************************************************
#define AVERAGE false
#define DIFF_HEIGHT_RATIO 50
#define SENSOR_RATIO_CAL 20

// states
#define SEARCH_STATE 0
#define FOUND_STATE 1
#define STOP_STATE 2
int state = 0;

#define FORW 0
#define LEFT 1
#define RIGHT 2
#define BACK 3

int directboi = 0;
int timeboi;

// Navigation Sensors Pins
int rightSensePin = A1;
int leftSensePin = A0;
int frontlSensePin  = A3;
int frontrSensePin  = A2;

// Weight Detection Pins
int wdTopRPin = A7;
int wdTopLPin = A4;
int wdBotRPin = A6;
int wdBotLPin = A5;

// Sensor Distances
float rightDistance;
float leftDistance;
float frontDistancer;
float frontDistancel;

float blDistance;
float tlDistance;
float brDistance;
float trDistance;


long compare_right;
long compare_left;
bool spinRightFlag = 0;
bool spinLeftFlag = 0;

// colour sensor
uint16_t r, g, b, c, colorTemp, lux;
bool base = 1; //1 for red base, 0 for green

bool diffHeightTrig = false;

// Variables for calculating distance from IR sensors
float k1Short = 0.022648669896535;
float k2Short = 11.4466981524272;

float k1Medium = 0.022648669896535;
float k2Medium = 11.4466760237414;

float k1Long = 0.362045928;
float k2Long = 40.5482045360838;

// obstacle avoid code
bool frontBlocked = false;
bool sidesBlocked = false;
int frontStop = 10;
int sideStop = 5;
//**********************************************************************************
// Local Definitions
//**********************************************************************************

mySense rightS(rightSensePin, k1Short, k2Short);
mySense leftS(leftSensePin, k1Short, k2Short);
mySense frontLS(frontlSensePin, k1Long, k2Long);
mySense frontRS(frontrSensePin, k1Long, k2Long);

mySense wdTR(wdTopRPin, k1Medium, k2Medium);
mySense wdTL(wdTopLPin, k1Medium, k2Medium);
mySense wdBR(wdBotRPin, k1Medium, k2Medium);
mySense wdBL(wdBotLPin, k1Medium, k2Medium);

// defines variables ULTRASONIC
long duration; // variable for the duration of sound wave travel
int backDistance; // variable for the distance measurement

bool backflag = 0;
int moveSpeed = 100;
int turnSpeed = 0;

//**********************************************************************************
// Function Definitions
//**********************************************************************************

//**********************************************************************************
// SETUP
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  weightservo_setup();

  //  robot_init();
  //  Wire.begin();
  initServo();
  motor_setup();


}

void pollSense() {
  rightS.poll();
  leftS.poll();
  frontLS.poll();
  frontRS.poll();

  wdTR.poll();
  wdTL.poll();
  wdBR.poll();
  wdBL.poll();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work)
// Set as high or low
//**********************************************************************************
void pin_init() {

  pinMode(rightSensePin, INPUT);
  pinMode(leftSensePin, INPUT);
  pinMode(frontlSensePin, INPUT);
  pinMode(frontrSensePin, INPUT);

  pinMode(wdTopRPin, INPUT);
  pinMode(wdTopLPin, INPUT);
  pinMode(wdBotRPin, INPUT);
  pinMode(wdBotLPin, INPUT);


  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT


  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}
//**********************************************************************************
// Set default robot state
//**********************************************************************************
//void robot_init() {
//
//}

void convertAllSense() {
  rightDistance = rightS.Distance();
  leftDistance =  leftS.Distance();
  frontDistancer = frontLS.Distance();
  frontDistancel = frontRS.Distance();

  blDistance = wdTR.Distance();
  tlDistance = wdTL.Distance();
  brDistance = wdBR.Distance();
  trDistance = wdBL.Distance();

}


void driveAvoid(int direct) {
  if (direct == FORW) {
go_forward(moveSpeed);
  } else if (direct == LEFT) {
    turn_left(turnSpeed);
  } else if (direct == RIGHT) {
    turn_right(turnSpeed);
  } else if (direct == BACK) {

  }
}

void driveWeight(int direct) {
  if (direct == FORW) {
    go_forward(moveSpeed);
  } else if (direct == LEFT) {
    turn_left(turnSpeed);
  } else if (direct == RIGHT) {
    turn_right(turnSpeed);
  } else if (direct == BACK) {
    go_back();
  }
}

void loop() {
  timeboi = 0;
  collect_weight();
  pollSense();
  convertAllSense();


  switch (state) {
    case SEARCH_STATE: {


        if (frontDistancer  < frontStop || frontDistancel < frontStop) {
          frontBlocked = true;
          if (rightDistance < sideStop && leftDistance < sideStop) {
            sidesBlocked = true;
            directboi = BACK;
          } else if (rightDistance < sideStop) {
            directboi = LEFT;
          } else if (leftDistance < sideStop) {
            directboi = RIGHT;
          } else {
            directboi = FORW;
          }
        }
        
        if (frontBlocked) {
          if (sidesBlocked) {
            if (rightDistance > sideStop || leftDistance > sideStop) {
              sidesBlocked = false;
            }
          } if (frontDistancer > frontStop || frontDistancel > frontStop) {
            frontBlocked = false;
          }
        }

//        if (diffHeightDist != 0) {
//          // TODO: weight detection
//          state = FOUND_STATE;
//          diffHeightTrig = false;
//        }
        driveAvoid(directboi);
        break;
      }
    case FOUND_STATE: {
        driveWeight(directboi);
        break;
      }
    case STOP_STATE: {
        stationary();
      }
  }



  delay(1);
}
