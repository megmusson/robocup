#include "motors.h"
#include "Arduino.h"
#include <Servo.h>

#define STOP_SPEED 1500
#define VARIABLE_SPEED 500
#define TURN_FACTOR 4/5

#define leftMotorPin 2
#define rightMotorPin 3

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo

#define FAST_FORWARD_SPEED 2100
#define SLOW_FORWARD_SPEED 1900
#define BACK_SPEED 1100

bool MOTORSbruh = 1;

void initServo() {
  servoMotorLeft.attach(leftMotorPin);
  servoMotorRight.attach(rightMotorPin);
}

void go_forward(int speedPercent) {
//  Serial.print("GO STRAIGHT ");
  int leftSpeed = 0;
  int rightSpeed = 0;
  leftSpeed = STOP_SPEED + VARIABLE_SPEED * speedPercent / 100;
  rightSpeed = STOP_SPEED + VARIABLE_SPEED * speedPercent / 100;
if (MOTORSbruh == true) {
  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);
}
}

void go_back() {
//  Serial.print("GO BACKWARDS ");
  int leftSpeed = 1100;
  int rightSpeed = 1100;
 if (MOTORSbruh == true) {
  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);
 }

}


void turn_left(int speedPercent) {
//  Serial.print("TURN LEFT ");
//Serial.print(STOP_SPEED + VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
//  Serial.print("\t");
//  Serial.println(STOP_SPEED - VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
if (MOTORSbruh == true) {
  
  
    servoMotorLeft.writeMicroseconds(STOP_SPEED - VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
    servoMotorRight.writeMicroseconds(STOP_SPEED + VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
}
}

void turn_right(int speedPercent) {
//  Serial.print("TURN RIGHT ");
if (MOTORSbruh == true) {
    servoMotorLeft.writeMicroseconds(STOP_SPEED + VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
    servoMotorRight.writeMicroseconds(STOP_SPEED - VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
}
}
void stationary() {
//  Serial.print("STOP ");
if (MOTORSbruh == true) {
  servoMotorRight.writeMicroseconds(STOP_SPEED);
  servoMotorLeft.writeMicroseconds(STOP_SPEED);
}}

void spin_left(int speedPercent){
//    Serial.print("SPIN LEFT ");
  go_back();
  delay(50);
  if (MOTORSbruh == true) {
  servoMotorLeft.writeMicroseconds(BACK_SPEED);      
  servoMotorRight.writeMicroseconds(SLOW_FORWARD_SPEED);
  }
  delay(100);
}


void spin_right(int speedPercent){
//      Serial.print("SPIN right ");

  go_back();
  delay(50);
  if (MOTORSbruh == true) {
  servoMotorRight.writeMicroseconds(BACK_SPEED);      
  servoMotorLeft.writeMicroseconds(SLOW_FORWARD_SPEED);
  }
  delay(100);
}

void lean_left(int speedPercent){
//  Serial.print(STOP_SPEED - VARIABLE_SPEED*speedPercent/100*TURN_FACTOR/2);
//  Serial.println(STOP_SPEED + VARIABLE_SPEED*speedPercent/100*TURN_FACTOR/2);
  if (MOTORSbruh == true) {
  servoMotorLeft.writeMicroseconds(STOP_SPEED - VARIABLE_SPEED*(speedPercent-20)/100*TURN_FACTOR);
    servoMotorRight.writeMicroseconds(STOP_SPEED + VARIABLE_SPEED*(speedPercent-20)/100*TURN_FACTOR);
  }
}
void lean_right(int speedPercent){
  if (MOTORSbruh == true) {
  servoMotorLeft.writeMicroseconds(STOP_SPEED + VARIABLE_SPEED*(speedPercent-20)/100*TURN_FACTOR);
    servoMotorRight.writeMicroseconds(STOP_SPEED - VARIABLE_SPEED*(speedPercent-20)/100*TURN_FACTOR);
  }
}
