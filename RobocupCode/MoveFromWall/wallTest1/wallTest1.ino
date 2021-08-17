// random
#include "IRfilter.h"

#define AVERAGE false
#include <Servo.h>
#include<stdio.h>

int analogPin0 = A0; // setting the read pin to A0 RIGHT
int analogPin1 = A1; //LEFT
int analogPin2 = A2; // setting the read pin to A0 //FRONT RIGHT
int analogPin3 = A3; //FRONT LEFT

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo

mySense lsensr(analogPin0);
mySense rsensr(analogPin1);
mySense frsensr(analogPin2);
mySense flsensr(analogPin3);

void setup() { // runs once at the begining
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(analogPin0, INPUT);
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  pinMode(analogPin3, INPUT);

  Serial.begin(9600); // setting the baud rate
  delay(100); //miliseconds

  servoMotorLeft.attach(2);  // attaches the servo pin 3 to the servo object
  servoMotorRight.attach(3);  // attaches the servo pin 2 to the servo object
}

void turn_left(){
  Serial.print("TURN LEFT");
  servoMotorLeft.writeMicroseconds(1100);      //Forward Slow  
  servoMotorRight.writeMicroseconds(2000);
}

void turn_right(){
  Serial.print("TURN LEFT");
  servoMotorRight.writeMicroseconds(1100);      //Forward Slow  
  servoMotorLeft.writeMicroseconds(2000);
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

  delayMicroseconds(100); 


CASE 1 MOVE FORWARD BOTH SENSORS SEE NOTHING
if ((frsensr.avg < 350)  && (flsensr.avg < 350)) {
  servoMotorLeft.writeMicroseconds(1800);   
  servoMotorRight.writeMicroseconds(1800);
}
else if ((frsensr.avg > 350)  && (flsensr.avg > 350)) {
  servoMotorLeft.writeMicroseconds(1600);   
  servoMotorRight.writeMicroseconds(1600);
}


//CASE Forward sensors see something, check for right and left
if ((frsensr.avg > 350)  && (flsensr.avg > 350)) {
  if ((rsensr.avg > 50) && (lsensr.avg < 50))  {
 turn_left();
}
  if ((rsensr.avg < 50) && (lsensr.avg > 50))  {
 turn_right();
}
}

//CASE Forward sensors see something, nothinbg on sides, turn left
if ((frsensr.avg > 350)  && (flsensr.avg > 350)) {
  if ((rsensr.avg < 50) && (lsensr.avg < 50))  {
 turn_left();
}
}


 
//  Serial.write(13);
//  Serial.write(10);
  delay(10); 
}
