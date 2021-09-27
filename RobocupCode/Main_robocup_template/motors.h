//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

// SET THIS TO REAL VALUES
#define LEFT_MOTOR_ADDRESS 2      //Pin corresponding to the left dc motor
#define RIGHT_MOTOR_ADDRESS 3    //Pin corresponding to the right dc motor
#define MIN_SPEED_CAP 1           //Set the minimum speed value that can be written to the motors
#define MAX_SPEED_CAP 2200          //Set the maximum speed value that can be written to the motors


#define FAST_FORWARD_SPEED 2100
#define SLOW_FORWARD_SPEED 1900
#define STOP_SPEED 1500
#define BACK_SPEED 1100


void go_forward();
void go_back();
void turn_left();
void turn_right();
void stationary();


//void check_speed_limits(/*parameters*/);
//void set_motor(/*parameters*/);


#endif /* MOTORS_H_ */
