// random
#include "IRfilter.h"

#define AVERAGE false
#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20
#include <Servo.h>
#include<stdio.h>

#define echoPin 27 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 26 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables ULTRASONIC
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

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

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo
Servo myServoRight;            // for differential height servo
Servo myServoLeft;

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
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(analogPin0, INPUT);
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  pinMode(analogPin3, INPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT


  Serial.begin(9600); // setting the baud rate

  // differential height
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  delay(100);
}

void go_forward() {
  Serial.println("GO FORWARD ");
  servoMotorLeft.writeMicroseconds(2100);
  servoMotorRight.writeMicroseconds(2100);
}

void go_back() {
  Serial.println("GO BACKWARDS ");
  servoMotorLeft.writeMicroseconds(1100);
  servoMotorRight.writeMicroseconds(1100);
}

void turn_left(){
  Serial.println("TURN LEFT ");
  servoMotorLeft.writeMicroseconds(1100);      //Forward Slow  
  servoMotorRight.writeMicroseconds(2000);
}

void turn_right(){
  Serial.println("TURN RIGHT ");
  servoMotorRight.writeMicroseconds(1100);      //Forward Slow  
  servoMotorLeft.writeMicroseconds(2000);
}

void stationary(){
  Serial.println("STOP ");
  servoMotorRight.writeMicroseconds(1500);      
  servoMotorLeft.writeMicroseconds(1500);
}




void loop() {
  rsensr.poll();
  lsensr.poll();
  frsensr.poll();
  flsensr.poll();

//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//  // Reads the echoPin, returns the sound wave travel time in microseconds
//  duration = pulseIn(echoPin, HIGH);
//  // Calculating the distance
//  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");
  
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

  delayMicroseconds(100); 


//  ////CASE 1 MOVE FORWARD BOTH SENSORS SEE NOTHING
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
      while (distance >10){
      go_back(); }

      
    }
   
  
  }

// // DIFFERENTIAL HEIGH SERVO ROTATION
//  if (pos < 1) {
//    directFlag = 0;
//    Serial.println("directFlag = 0");
//  }
//  if (pos > 179) {
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
//    Serial.print(pos );
//  }
////  myServoRight.write(pos);
//  myServoLeft.write(pos);


  //DIFFERENTIAL HEIGHT servo 
//  for (pos = 0; pos <=180; pos += 1) {  // goes from 0 to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);     // tell servo to go to position in variable 'pos'
//    delay(15);              // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -=1) {  // goes from 180 degrees to 0 degrees
//    myservo.write(pos);
//    delay(15);
//  }



//  // Differential Height Sensor Detection
//  tr_sensr.poll();
//  br_sensr.poll();
//  tl_sensr.poll();
//  bl_sensr.poll();
//  
//  compare_right = br_sensr.avg - tr_sensr.avg;
//  compare_left = bl_sensr.avg - tl_sensr.avg;
//  //Serial.print(compare);
//  //Serial.print(" - ");
//  if ((compare_right > DIFF_HEIGHT_RATIO) || (compare_left > DIFF_HEIGHT_RATIO)) {
//    Serial.print("Weight Detected! ");
//    stationary();
//    //int detected_pos = myServoRight.read();
//  } else {
//    Serial.print("No Weight Detected sad emoji ");
//  }
//
//  Serial.print("Bottom Right:");
//  Serial.print(br_sensr.avg);
//  Serial.print(", ");
//  Serial.print("Top Right:");
//  Serial.print(tr_sensr.avg);
//  Serial.print(", ");
//  Serial.print("Bottom Left:");
//  Serial.print(bl_sensr.avg);
//  Serial.print(", ");
//  Serial.print("Top Left:");
//  Serial.print(tl_sensr.avg);
//  Serial.print(", ");





 
//  Serial.write(13);
//  Serial.write(10);

}
