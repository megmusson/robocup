
#include "IRfilter.h"

#include <SharpIR.h>
//SharpIR right( SharpIR::GP2Y0A41SK0F, A1 );
//SharpIR left( SharpIR::GP2Y0A41SK0F, A0 );
//SharpIR front( SharpIR::GP2Y0A21YK0F, A2 );

#define AVERAGE false
#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20
#include <Servo.h>
#include<stdio.h>

int analogPin0 = A0; // setting the read pin to A0 RIGHT
int analogPin1 = A1; //LEFT
int analogPin2 = A2; // setting the read pin to A0 //FRONT RIGHT
int analogPin3 = A3; //FRONT LEFT
int toprightSensor = A6; // green LED (differenetial height)
int botrightSensor = A7; // red LED
int topleftSensor = A4;
int bottomleftSensor = A5;

int pos = 0;        // position of servo
bool directFlag = 0;// flag for servo
long compare_right;
long compare_left;
int angle = 0;
bool spinRightFlag = 0;
bool spinLeftFlag = 0;
int weight_pos;


Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo
Servo myServoRight;            // for differential height servo
Servo myServoLeft;
Servo pickupServo;

mySense tr_sensr(toprightSensor);    // (differenetial height) top right
mySense br_sensr(botrightSensor);  // bottom right
mySense tl_sensr(topleftSensor);  //top left
mySense bl_sensr(bottomleftSensor);  // bottom left

mySense lsensr(analogPin0);
mySense rsensr(analogPin1);
mySense frsensr(analogPin2);
mySense flsensr(analogPin3);

int myServoRightPin = 5;
int myServoLeftPin = 4;

void setup() { // runs once at the begining
  servoMotorLeft.attach(2);  // attaches the servo pin 3 to the servo object
  servoMotorRight.attach(3);  // attaches the servo pin 2 to the servo object
  myServoRight.attach(myServoRightPin);                   // attached servo on pin () to ervo object 
  myServoLeft.attach(myServoLeftPin);
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

//void go_forward() {
//  Serial.print("GO FORWARD ");
//  servoMotorLeft.writeMicroseconds(2000);
//  servoMotorRight.writeMicroseconds(2000);
//}
//
//void go_back() {
//  Serial.print("GO BACKWARDS ");
//  servoMotorLeft.writeMicroseconds(1100);
//  servoMotorRight.writeMicroseconds(1100);
//}
//
//void turn_left(){
//  Serial.print("TURN LEFT ");
//  servoMotorLeft.writeMicroseconds(1100);      //Forward Slow  
//  servoMotorRight.writeMicroseconds(2000);
//}
//
//void turn_right(){
//  Serial.print("TURN RIGHT ");
//  servoMotorRight.writeMicroseconds(1100);      //Forward Slow  
//  servoMotorLeft.writeMicroseconds(2000);
//}
//
//void stationary(){
//  Serial.print("STOP ");
//  servoMotorRight.writeMicroseconds(1500);      
//  servoMotorLeft.writeMicroseconds(1500);
//}
//
//void spin_right(){
//  Serial.print("SPIN RIGHT ");
//  servoMotorRight.writeMicroseconds(1000);      //Forward Slow  
//  servoMotorLeft.writeMicroseconds(2000);
//}
//
//void spin_left(){
//  Serial.print("SPIN LEFT ");
//  servoMotorLeft.writeMicroseconds(1000);      //Forward Slow  
//  servoMotorRight.writeMicroseconds(2000);
//}
//
//


void loop() {
//
//go_forward();
//// Obstacle sensor detection polling
//  rsensr.poll();
//  lsensr.poll();
//  frsensr.poll();
//  flsensr.poll();
//
//// Differential height sensor polling
  tr_sensr.poll();
  br_sensr.poll();
  tl_sensr.poll();
  bl_sensr.poll();


//int dis = right.getDistance();
//  Serial.print("rig:");
//
// Serial.print(dis);
//   Serial.print("lef:");
//
// int lef = left.getDistance();
// Serial.print(lef);
//  Serial.print("fro:");
//
//int fro = front.getDistance();
// Serial.println(fro);

//Printing 
//  Serial.print("Left:");
//  Serial.print(lsensr.avg);
//  Serial.print(", ");
//  Serial.print("Right:");
//  Serial.print(rsensr.avg);
//  Serial.print(", ");
//  Serial.print("Front left::");
//  Serial.print(flsensr.avg);
//  Serial.print(", ");
//  Serial.print("Front right::");
//  Serial.println(frsensr.avg);
//
  Serial.print("Right Bottom:");
  Serial.print(br_sensr.avg);
  Serial.print(", ");
  Serial.print("Right Top:");
  Serial.print(tr_sensr.avg); 
  Serial.print(", ");
  Serial.print("Left Bottom:");
  Serial.print(bl_sensr.avg);
  Serial.print(", ");
  Serial.print("Left Top:");
  Serial.println(tl_sensr.avg);

//  delayMicroseconds(100); 
//
//  ////CASE 1 MOVE FORWARD BOTH SENSORS SEE NOTHING, AND NO WEIGHT DETECTED
//  if ((frsensr.avg < 200)  && (flsensr.avg < 200) && (spinRightFlag == 0) && (spinLeftFlag == 0)) {
//    go_forward();
//    if ((rsensr.avg > 200) && (lsensr.avg < 200))  {
//      turn_left();
//    }
//  
//    // Check if left sensor detects something, turn right if so
//    if ((rsensr.avg < 200) && (lsensr.avg > 200) && (spinRightFlag == 0)&& (spinLeftFlag == 0))  {
//      turn_right();
//    }
//  
//  }
//
//  // CASE If either front sensor detect somthing
//  else if ((frsensr.avg > 200)  || (flsensr.avg > 200) && (spinRightFlag == 0)&& (spinLeftFlag == 0)) {
//  
//    // Check if right sensor detects something, turn left if so
//    if ((rsensr.avg > 300) && (lsensr.avg < 300))  {
//      turn_left();
//    }
//  
//    // Check if left sensor detects something, turn right if so
//    if ((rsensr.avg < 300) && (lsensr.avg > 300) && (spinRightFlag == 0)&& (spinLeftFlag == 0))  {
//      turn_right();
//    }
//  
//    // If left and right sensor detect nothing, turn left (CHANGE THIS LATER)
//    if ((rsensr.avg < 300) && (lsensr.avg < 300) && (spinRightFlag == 0)&& (spinLeftFlag == 0))  {
//      turn_left();
//    }
//
//    // If both left and right sensor detects something
//    if ((rsensr.avg > 200) && (lsensr.avg > 200) && (spinRightFlag == 0)&& (spinLeftFlag == 0))  {
//      go_back(); 
//    }
//  
//  }



// Pick up mechanism servo, continuous rotation
//Serial.print(angle);
//angle += 5;
//if (angle >= 360) {
//  angle == 0;
//}
//pickupServo.write(angle); 
//
//
// // DIFFERENTIAL HEIGH SERVO ROTATION
//  if (pos < 1) {
//    directFlag = 0;
//    Serial.println("directFlag = 0");
//  }
//  if (pos > 110) {
//    directFlag = 1;
//  }
//
//  if (directFlag == 0){
//    pos += 5;
//    Serial.print("pos = ");
//    Serial.print(pos );
//  } 
//  if (directFlag == 1) {
//    pos -= 5;
//    Serial.print("pos = ");
//    Serial.println(pos );
//  }
//
////
////
////
//  // Differential Height Sensor Detection AND NAVIGATION
////
//    myServoRight.write(pos);
//    myServoLeft.write(180-pos);
//  compare_right = br_sensr.avg - tr_sensr.avg;
//  compare_left = bl_sensr.avg - tl_sensr.avg;
//  //Serial.print(compare);
//  //Serial.print(" - ");
//  if (compare_right > DIFF_HEIGHT_RATIO) {
////    Serial.print("Weight Detected! ");
//    spinRightFlag = 1;
//    weight_pos = myServoRight.read();   // read weight position from servo
////    Serial.print("Position: ");
////    Serial.print(weight_pos);
//    stationary();   // stop the robot
//
//    
//    if (weight_pos < 30) {
//      spin_right();
//      delay(50);
//      spinRightFlag = 0;
//    }
//    else if (weight_pos < 60) { // && weight_pos > 30
//      spin_right();
//      delay(100);
//      spinRightFlag = 0;
//      
//    }
//
//    else if (weight_pos < 90) { // && weight_pos > 60
//      spin_right();
//      delay(150);
//      spinRightFlag = 0;
//      
//    }
//
//       else if (weight_pos < 130) { // && weight_pos > 90
//      spin_right();
//      delay(200);
//      spinRightFlag = 0;
//      
//    }
//    //int detected_pos = myServoRight.read();
//  } 
//   else if ((compare_left > DIFF_HEIGHT_RATIO)) {
////    Serial.print("Weight Detected! ");
//      spinLeftFlag = 1;
//    weight_pos = myServoLeft.read();   // read weight position from servo
////    Serial.print("Position: ");
////    Serial.print(weight_pos);
//    stationary();   // stop the robot
//    if (weight_pos < 30) {
//      spin_left();
//      delay(50);
//      spinLeftFlag = 0;
//      
//    }
//   else if (weight_pos < 60 && weight_pos > 30) {
//      spin_left();
//      delay(100);
//      spinLeftFlag = 0;
//      
//    }
//
//       else if (weight_pos < 90 && weight_pos > 60) {
//      spin_left();
//      delay(150);
//      spinLeftFlag = 0;
//      
//    }
//
//       else if (weight_pos < 130 && weight_pos > 90) {
//      spin_left();
//      delay(200);
//      spinLeftFlag = 0;
//      
//    }
//    //int detected_pos = myServoRight.read();
//  } 
//  else {
////    Serial.print("No Weight Detected");
////    myServoRight.write(pos);
////    myServoLeft.write(180-pos);
//  }
//
//Serial.print("Flag " ); 
//Serial.print(spinRightFlag );
//Serial.print("position " );
//Serial.println(pos );
//Serial.print("Left Flag " );
//Serial.print(spinLeftFlag );



 
//  Serial.write(13);
//  Serial.write(10);

}
