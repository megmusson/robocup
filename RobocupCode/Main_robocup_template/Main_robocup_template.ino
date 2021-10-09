
/********************************************************************************
 *                               ROBOCUP TEMPLATE                              
 *        
 *  
 *  This is a template program design with modules for 
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks sholud run
 *  
 *  
 *  written by: Logan Chatfield, Ben Fortune, Lachlan McKenzie, Jake Campbell
 *  
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include<stdio.h>
//#include <Herkulex.h>             //smart servo
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler 
#include "IRfilter.h"
#include "circBufT.h"

// Custom headers
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h" 

//**********************************************************************************
// Local Definitions
//**********************************************************************************
#define AVERAGE false
#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20

int analogPin0 = A0; // setting the read pin to A0 RIGHT
int analogPin1 = A1; //LEFT
int analogPin2 = A2; // setting the read pin to A0 //FRONT RIGHT
int analogPin3 = A3; //FRONT LEFT
int toprightSensor = A7; // green LED (differenetial height)
int botrightSensor = A6; // red LED
int topleftSensor = A4;
int bottomleftSensor = A5;

int pos = 0;        // position of servo
bool directFlag = 0;// flag for servo
long compare_right;
long compare_left;
int angle = 0;
bool spinRightFlag = 0;
bool spinLeftFlag = 0;

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo
Servo myServoRight;            // for differential height servo
Servo myServoLeft;
Servo pickupServo;

mySense tr_sensr(toprightSensor);    // (differenetial height) top right
mySense br_sensr(botrightSensor);  // bottom right
mySense tl_sensr(topleftSensor);  //top left
mySense bl_sensr(topleftSensor);  // bottom left

mySense lsensr(analogPin0);
mySense rsensr(analogPin1);
mySense frsensr(analogPin2);
mySense flsensr(analogPin3);

int myServoRightPin = 5;
int myServoLeftPin = 4;
// Serial deffinitions
#define BAUD_RATE 9600

Servo right_motor;
Servo left_motor;



//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  robot_init();
  task_init();
  Wire.begin();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    
    Serial.println("Pins have been initialised \n"); 

    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}

//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
    Serial.println("Robot is ready \n");
}


//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {

  rsensr.poll();
  lsensr.poll();
  frsensr.poll();
  flsensr.poll();

  tr_sensr.poll();
  br_sensr.poll();
  tl_sensr.poll();
  bl_sensr.poll();
  
  
}
