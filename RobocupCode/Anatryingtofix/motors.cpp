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


void initServo() {
  servoMotorLeft.attach(leftMotorPin);
  servoMotorRight.attach(rightMotorPin);
}

void go_forward(int speedPercent) {
  Serial.println("GO STRAIGHT ");
  int leftSpeed = 2200;
  int rightSpeed = 2200;
//  leftSpeed = STOP_SPEED + VARIABLE_SPEED * speedPercent / 100;
//  rightSpeed = STOP_SPEED + VARIABLE_SPEED * speedPercent / 100;
  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);
}

void go_back() {
  Serial.println("GO BACKWARDS ");
  int leftSpeed = 1100;
  int rightSpeed = 1100;
  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);

}


void turn_left(int speedPercent) {
  Serial.println("TURN LEFT ");
    servoMotorLeft.writeMicroseconds(1000);
    servoMotorRight.writeMicroseconds(2200);

}

void turn_right(int speedPercent) {
  Serial.println("TURN RIGHT ");
    servoMotorLeft.writeMicroseconds(2200);
    servoMotorRight.writeMicroseconds(1000);
}

void stationary() {
  Serial.println("STOP ");
  servoMotorRight.writeMicroseconds(STOP_SPEED);
  servoMotorLeft.writeMicroseconds(STOP_SPEED);
}

void spin_left(int speedPercent){
    Serial.println("SPIN LEFT ");
  go_back();
  delay(50);
  servoMotorLeft.writeMicroseconds(BACK_SPEED);      
  servoMotorRight.writeMicroseconds(SLOW_FORWARD_SPEED);
  delay(100);
}


void spin_right(int speedPercent){
      Serial.println("SPIN right ");

  go_back();
  delay(50);
  servoMotorRight.writeMicroseconds(BACK_SPEED);      
  servoMotorLeft.writeMicroseconds(SLOW_FORWARD_SPEED);
  delay(100);
}
