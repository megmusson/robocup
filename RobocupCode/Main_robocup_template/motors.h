//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

#include "Arduino.h"
#include <Servo.h>

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 2      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 3    //Pin corresponding to the right dc motor
#define MIN_SPEED_CAP 1           //Set the minimum speed value that can be written to the motors
#define MAX_SPEED_CAP 2200          //Set the maximum speed value that can be written to the motors


void initServo();
void checkSpeedLimits(/*parameters*/);
void setMotor(/*parameters*/);

void go_forward(int speedPercent);
void go_back(int speedPercent);

void turn_left(int speedPercent);
void turn_right(int speedPercent);

void stationary();

#endif /* MOTORS_H_ */
