#include "motors.h"
#include "Arduino.h"
#include <Servo.h>

<<<<<<< HEAD
/* Check whether the speed value to be written is within the maximum
    and minimum speed caps. Act accordingly.

*/
//void check_speed_limits(/*parameters*/) {
//  Serial.println("Check the motor speed limit \n");
//}
=======
#define STOP_SPEED 1500
#define VARIABLE_SPEED 500
#define TURN_FACTOR 4/5
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8

#define leftMotorPin 2
#define rightMotorPin 3

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo
<<<<<<< HEAD

#define STOP_SPEED 1500
#define VARIABLE_SPEED 500
#define TURN_FACTOR 4/5
#define FAST_FORWARD_SPEED 2100
#define SLOW_FORWARD_SPEED 1900
#define BACK_SPEED 1100

/* In this section, the motor speeds should be updated/written.
  It is also a good idea to check whether value to write is valid.
  It is also a good idea to do so atomically!
*/
//void set_motor(/*parameters*/) {
//
//  Serial.println("Change the motor speed \n");
//
//  check_speed_limits();
//
//}
=======
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8

void initServo() {
  servoMotorLeft.attach(leftMotorPin);
  servoMotorRight.attach(rightMotorPin);
}

void go_forward(int speedPercent) {
<<<<<<< HEAD
  Serial.println("GO STRAIGHT ");

=======
>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8
  int leftSpeed = 0;
  int rightSpeed = 0;

  leftSpeed = STOP_SPEED + VARIABLE_SPEED * speedPercent / 100;
  rightSpeed = STOP_SPEED + VARIABLE_SPEED * speedPercent / 100;

  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);
}

void go_back() {
  Serial.println("GO BACKWARDS ");
<<<<<<< HEAD
  int leftSpeed = 1100;
  int rightSpeed = 1100;

//  leftSpeed = STOP_SPEED - VARIABLE_SPEED * speedPercent / 100;
//  rightSpeed = STOP_SPEED - VARIABLE_SPEED * speedPercent / 100;
  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);

=======

  int leftSpeed = 0;
  int rightSpeed = 0;

  leftSpeed = STOP_SPEED - VARIABLE_SPEED*speedPercent/100;
  rightSpeed = STOP_SPEED - VARIABLE_SPEED*speedPercent/100;

  servoMotorLeft.writeMicroseconds(leftSpeed);
  servoMotorRight.writeMicroseconds(rightSpeed);

>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8
}


void turn_left(int speedPercent) {
  Serial.println("TURN LEFT ");
<<<<<<< HEAD
  servoMotorLeft.writeMicroseconds(BACK_SPEED);      //Forward Slow
  servoMotorRight.writeMicroseconds(SLOW_FORWARD_SPEED);
=======
    servoMotorLeft.writeMicroseconds(STOP_SPEED - VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
    servoMotorRight.writeMicroseconds(STOP_SPEED + VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);

>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8
}

void turn_right(int speedPercent) {
  Serial.println("TURN RIGHT ");
<<<<<<< HEAD
  servoMotorLeft.writeMicroseconds(SLOW_FORWARD_SPEED);
  servoMotorRight.writeMicroseconds(BACK_SPEED);      //Forward Slow
=======
    servoMotorLeft.writeMicroseconds(STOP_SPEED + VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);
    servoMotorRight.writeMicroseconds(STOP_SPEED - VARIABLE_SPEED*speedPercent/100*TURN_FACTOR);

>>>>>>> dc9e197927dad04c1332741e9da8a7c813760de8

}

void stationary() {
  Serial.println("STOP ");
  servoMotorRight.writeMicroseconds(STOP_SPEED);
  servoMotorLeft.writeMicroseconds(STOP_SPEED);
}

void spin_left(int speedPercent){
  go_back();
  delay(100);
  servoMotorLeft.writeMicroseconds(BACK_SPEED);      
  servoMotorRight.writeMicroseconds(SLOW_FORWARD_SPEED);
    delay(100);
}


void spin_right(int speedPercent){
  go_back();
  delay(100);
  servoMotorRight.writeMicroseconds(BACK_SPEED);      
  servoMotorLeft.writeMicroseconds(SLOW_FORWARD_SPEED);
  delay(100);
}
