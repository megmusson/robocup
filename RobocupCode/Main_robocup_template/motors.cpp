#include "motors.h"
#include "Arduino.h"
#include <Servo.h>

/* Check whether the speed value to be written is within the maximum
 *  and minimum speed caps. Act accordingly.
 *
 */
//void check_speed_limits(/*parameters*/) {
//  Serial.println("Check the motor speed limit \n");
//}




/* In this section, the motor speeds should be updated/written.
 *It is also a good idea to check whether value to write is valid.
 *It is also a good idea to do so atomically!
 */
//void set_motor(/*parameters*/) {
//
//  Serial.println("Change the motor speed \n");
//  
//  check_speed_limits();
//
//}

void initServo() {
  servoMotorLeft.attach(LEFT_MOTOR_ADDRESS);
  servoMotorRight.attach(RIGHT_MOTOR_ADDRESS);
}

void goStraight(int speedPercent) {
  int leftSpeed = 0;
  int rightSpeed = 0;

  leftSpeed = STOP_SPEED + VARIABLE_SPEED*speedPercent/100;
  rightSpeed = STOP_SPEED + VARIABLE_SPEED*speedPercent/100;

  leftMotor.writeMicroseconds(leftSpeed);
  rightMotor.writeMicroseconds(rightSpeed);
}

void go_back() {
  Serial.println("GO BACKWARDS ");
  servoMotorLeft.writeMicroseconds(BACK_SPEED);
  servoMotorRight.writeMicroseconds(BACK_SPEED);
}

void go_forward() {
  Serial.println("GO FORWARD ");
  servoMotorLeft.writeMicroseconds(FAST_FORWARD_SPEED);
  servoMotorRight.writeMicroseconds(FAST_FORWARD_SPEED);
}

void turn_left(){
  Serial.println("TURN LEFT ");
  servoMotorLeft.writeMicroseconds(BACK_SPEED);      //Forward Slow  
  servoMotorRight.writeMicroseconds(SLOW_FORWARD_SPEED);
}

void turn_right(){
  Serial.println("TURN RIGHT ");
  servoMotorLeft.writeMicroseconds(SLOW_FORWARD_SPEED);
  servoMotorRight.writeMicroseconds(BACK_SPEED);      //Forward Slow  

}

void stationary(){
  Serial.println("STOP ");
  servoMotorRight.writeMicroseconds(STOP_SPEED);      
  servoMotorLeft.writeMicroseconds(STOP_SPEED);
}
