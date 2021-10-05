// random


#include <Servo.h>
#include<stdio.h>

Servo myServoRight;

int myServoRightPin = 5;

void setup() { // runs once at the begining

  myServoRight.attach(myServoRightPin);                   // attached servo on pin () to ervo object 



  Serial.begin(9600); // setting the baud rate

  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  delay(100);
}



// Servomotor calibration values
int minDegrees;
int maxDegrees;
int minFeedback;
int maxFeedback;
int tolerance = 2; //max feedback measurement error

//
//void calibrate(Servo servo, int analogPin, int minPos, int maxPos){
//  // Move to min position and record feedback value
//  servo.write(minPos);
//  minDegrees = minPos;
//  delay(2000);
//  minFeedback = analogRead(analogPin);
//
//  //Move to max position and record feedback value
//  servo.write(maxPos);
//  maxDegrees = maxPos;
//  delay(2000);
//  maxFeedback = analogRead(analogPin);
//}


void loop() {

myServoRight.write(10);
Serial.println("10");
delay(1000);
myServoRight.write(170);
Serial.println("170");
delay(1000);
}
