/********************************************************************************
                                 ROBOCUP TEMPLATE
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include<stdio.h>
#include "MedianFilterLib.h"

//#include <Herkulex.h>             //smart servo
<<<<<<< HEAD
//#include <Adafruit_TCS34725.h>      //colour sensor
//#include <Wire.h>                   //for I2C and SPI

// Custom headers
#include "motors.h"
//#include "sensors.h"
//#include "weight_collection.h"
//#include "return_to_base.h"

#include "IRfilter.h"
#include <SharpIR.h>
SharpIR right( SharpIR::GP2Y0A41SK0F, A1 );
SharpIR left( SharpIR::GP2Y0A41SK0F, A0 );
SharpIR front( SharpIR::GP2Y0A21YK0F, A2 );
MedianFilter<int> medianFilter(30);

=======
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
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8

int toprightSensor = A7; // green LED (differenetial height)
int botrightSensor = A6; // red LED
int topleftSensor = A5;
int bottomleftSensor = A4;

mySense tr_sensr(toprightSensor);    // (differenetial height) top right
mySense br_sensr(botrightSensor);  // bottom right
mySense tl_sensr(topleftSensor);  //top left
mySense bl_sensr(bottomleftSensor);  // bottom left

<<<<<<< HEAD
//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Pin deffinitions
#define IO_POWER  49
#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20
#define BAUD_RATE 9600
#define echoPin 27 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 26 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables ULTRASONIC
long duration; // variable for the duration of sound wave travel
int backDistance; // variable for the distance measurement

bool backflag = 0;
int moveSpeed = 100;
int turn = 0;

long compare_right;
long compare_left;
bool spinRightFlag = 0;
bool spinLeftFlag = 0;
=======
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
//void robot_init();
//void task_init(;

//**********************************************************************************
// SETUP
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  //  robot_init();
  //  task_init();
  //  Wire.begin();

  initServo();

}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work)
// Set as high or low
//**********************************************************************************
void pin_init() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(toprightSensor, INPUT);
  pinMode(botrightSensor, INPUT);
  pinMode(topleftSensor, INPUT);
  pinMode(bottomleftSensor, INPUT);
  Serial.println("Pins have been initialised \n");


  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}

<<<<<<< HEAD
//**********************************************************************************
// Set default robot state
//**********************************************************************************
//void robot_init() {
//    Serial.println("Robot is ready \n");
//}


=======
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8

//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {

<<<<<<< HEAD
  // Differential height sensor polling
=======
  rsensr.poll();
  lsensr.poll();
  frsensr.poll();
  flsensr.poll();

>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8
  tr_sensr.poll();
  br_sensr.poll();
  tl_sensr.poll();
  bl_sensr.poll();
<<<<<<< HEAD
  
  int rightDistance = right.getDistance();
  int leftDistance = left.getDistance();
  int frontDistance = front.getDistance();

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  backDistance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  backDistance = medianFilter.AddValue(backDistance);
  //
  //  Serial.print("Distance: ");
  //  Serial.print(backDistance);
  //  Serial.println(" cm");
  //  delayMicroseconds(100);

  //  Serial.print("right:");
  //  Serial.print(rightDistance);
  //  Serial.print("left:");
  //  Serial.print(leftDistance);
  //  Serial.print("front:");
  //  Serial.println(frontDistance);
//
//    Serial.print("Right Bottom:");
//  Serial.print(br_sensr.avg);
//  Serial.print(", ");
//  Serial.print("Right Top:");
//  Serial.print(tr_sensr.avg); 
//  Serial.print(", ");
//  Serial.print("Left Bottom:");
//  Serial.print(bl_sensr.avg);
//  Serial.print(", ");
//  Serial.print("Left Top:");
//  Serial.println(tl_sensr.avg);


  if (frontDistance > 15 && backflag == 0) {
    go_forward(moveSpeed);

    // Check if left sensor detects something, turn right if so
    if (leftDistance < 10 && rightDistance > 10)  {
      turn_right(moveSpeed);
      //      Serial.println(micros());
    }
    if (rightDistance < 10 && leftDistance > 10)  {
      turn_left(moveSpeed);
    }
  }

  // CASE If either front sensor detect somthing
  else if (frontDistance < 15 ) {

    // Check if left sensor detects something, turn right if so
    if (leftDistance < 10 && rightDistance > 10 && backflag == 0)  {
      turn_right(moveSpeed);

    }

    if (rightDistance < 10 && leftDistance > 10 && backflag == 0)  {
      turn_left(moveSpeed);

    }

    // If left and right sensor detect nothing, turn left (CHANGE THIS LATER)
    if (rightDistance > 10 && leftDistance > 10 && backflag == 0 )  {
      turn_right(moveSpeed);
    }

    // If both left and right sensor detects something
    if (rightDistance < 10 && leftDistance < 10 && backflag == 0)  {
      backflag = 1;
    }
  }

  else if (backflag == 1) {
    while (rightDistance < 10 || leftDistance < 10) {
      go_back();
      rightDistance = right.getDistance();
      leftDistance = left.getDistance();
    }

    backflag = 0;
    turn_right(moveSpeed);
    delay(3000);

  }


  compare_right = br_sensr.avg - tr_sensr.avg;
  compare_left = bl_sensr.avg - tl_sensr.avg;
  if (compare_right > DIFF_HEIGHT_RATIO) {
    Serial.print("Weight Detected! ");
    spinRightFlag = 1;
    stationary();   // stop the robot
    spin_right(moveSpeed);
  }
  else if (compare_left > DIFF_HEIGHT_RATIO) {
    Serial.print("Weight Detected! ");
    spinLeftFlag = 1;
    stationary();   // stop the robot
    spin_left(moveSpeed);
  }
=======
  
  
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8
}
