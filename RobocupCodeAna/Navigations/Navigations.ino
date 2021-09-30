// random
#include "IRfilter.h"

#define AVERAGE false
#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20
#include <Servo.h>
#include<stdio.h>

int analogPin0 = A0; // setting the read pin to A0 RIGHT
int analogPin1 = A1; //LEFT
int analogPin2 = A2; // setting the read pin to A0 //FRONT RIGHT
int analogPin3 = A3; //FRONT LEFT
int toprightSensor = A5; // green LED (differenetial height)
int botrightSensor = A4; // red LED
int topleftSensor = A7;
int bottomleftSensor = A6;

int pos = 0;        // position of servo
bool directFlag = 0;// flag for servo
long compare_right;
long compare_left;
int angle = 0;

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

void setup() { // runs once at the begining
  servoMotorLeft.attach(2);  // attaches the servo pin 3 to the servo object
  servoMotorRight.attach(3);  // attaches the servo pin 2 to the servo object
  myServoRight.attach(5);                   // attached servo on pin () to ervo object 
  myServoLeft.attach(7);
  myServoRight.write(90);                    // set servo to mid point
  myServoLeft.write(90);                    // set servo to mid point
  pickupServo.attach(9);
  pickupServo.write(0);
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(analogPin0, INPUT);
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  pinMode(analogPin3, INPUT);
  pinMode(toprightSensor, INPUT);
  pinMode(botrightSensor, INPUT);
  pinMode(topleftSensor, INPUT);
  pinMode(bottomleftSensor, INPUT);


  Serial.begin(9600); // setting the baud rate

  // differential height
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  delay(100);
}
  
void go_forward() {
  Serial.print("GO FORWARD ");
  servoMotorLeft.writeMicroseconds(2000);
  servoMotorRight.writeMicroseconds(2000);
}

void go_back() {
  Serial.print("GO BACKWARDS ");
  servoMotorLeft.writeMicroseconds(1000);
  servoMotorRight.writeMicroseconds(1000);
}

void turn_left(){
  Serial.print("TURN LEFT ");
  servoMotorLeft.writeMicroseconds(1000);      //Forward Slow  
  servoMotorRight.writeMicroseconds(2000);
}

void turn_right(){
  Serial.print("TURN RIGHT ");
  servoMotorRight.writeMicroseconds(1000);      //Forward Slow  
  servoMotorLeft.writeMicroseconds(2000);
}

void stationary(){
  Serial.print("STOP ");
  servoMotorRight.writeMicroseconds(1600);      
  servoMotorLeft.writeMicroseconds(1600);
}


void loop() {
  rsensr.poll();
  lsensr.poll();
  frsensr.poll();
  flsensr.poll();


  Serial.print("Left:");
  Serial.print(lsensr.avg);
  Serial.print(", ");
  Serial.print("Right:");
  Serial.print(rsensr.avg);
  Serial.print(", ");
  Serial.print("Front left::");
  Serial.print(flsensr.avg);
  Serial.print(", ");
  Serial.print("Front right::");
  Serial.println(frsensr.avg);


  ////CASE 1 MOVE FORWARD BOTH SENSORS SEE NOTHING
  if ((frsensr.avg < 350)  && (flsensr.avg < 350)) {
    go_forward();
  }

  // CASE If either front sensor detect somthing
  else if ((frsensr.avg > 350)  || (flsensr.avg > 350)) {
  
    // Check if right sensor detects something, turn left if so
    if ((rsensr.avg > 50) && (lsensr.avg < 50))  {
      turn_left();
    }
  
    // Check if left sensor detects something, turn right if so
    if ((rsensr.avg < 50) && (lsensr.avg > 50))  {
      turn_right();
    }
  
    // If left and right sensor detect nothing, turn left (CHANGE THIS LATER)
    if ((rsensr.avg < 50) && (lsensr.avg < 50))  {
      turn_left();
    }

    // If both left and right sensor detects something
    if ((rsensr.avg > 50) && (lsensr.avg > 50))  {
      go_back(); 
    }
  
  }


  delayMicroseconds(100); 

}
