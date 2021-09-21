// random
#include "IRfilter.h"

#define AVERAGE false
#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20
#include <Servo.h>
#include<stdio.h>

int analogPin0 = A0; // setting the read pin to A0 RIGHT
int analogPin1 = A1; //LEFT
int analogPin2 = A2; // setting the read pin to A0 //FRONT RIGHT
int analogPin3 = A3; //FRONT LEFT
int topSensor = A5; // green LED (differenetial height)
int botSensor = A4; // red LED

int pos = 0;        // position of servo
bool directFlag = 0;// flag for servo

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo
Servo myservo;            // for differential height servo

long compare;
mySense tsensr(topSensor);    // (differenetial height)
mySense bsensr(botSensor);

mySense lsensr(analogPin0);
mySense rsensr(analogPin1);
mySense frsensr(analogPin2);
mySense flsensr(analogPin3);

void setup() { // runs once at the begining
  myservo.attach(5);                   // attached servo on pin () to ervo object 
  myservo.write(90);                    // set servo to mid point
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

  // differential height
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600); // setting the baud rate
  delay(100);
}

void go_forward() {
  Serial.print("GO FORWARD ");
  servoMotorLeft.writeMicroseconds(2100);
  servoMotorRight.writeMicroseconds(2100);
}

void go_back() {
  Serial.print("GO BACKWARDS ");
  servoMotorLeft.writeMicroseconds(1100);
  servoMotorRight.writeMicroseconds(1100);
}

void turn_left(){
  Serial.print("TURN LEFT ");
  servoMotorLeft.writeMicroseconds(1100);      //Forward Slow  
  servoMotorRight.writeMicroseconds(2000);
}

void turn_right(){
  Serial.print("TURN RIGHT ");
  servoMotorRight.writeMicroseconds(1100);      //Forward Slow  
  servoMotorLeft.writeMicroseconds(2000);
}

void stationary(){
  Serial.print("STOP ");
  servoMotorRight.writeMicroseconds(1500);      
  servoMotorLeft.writeMicroseconds(1500);
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


  ////CASE 1 MOVE FORWARD BOTH SENSORS SEE NOTHING
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
      go_back(); 
    }
  
  }



 // DIFFERENTIAL HEIGH SERVO ROTATION
  if (pos < 1) {
    directFlag = 0;
    Serial.println("directFlag = 0");
  }
  if (pos > 179) {
    directFlag = 1;
  }

  if (directFlag == 0){
    pos += 5;
    Serial.print("pos = ");
    Serial.print(pos );
  } 
  if (directFlag == 1) {
    pos -= 5;
    Serial.print("pos = ");
    Serial.print(pos );
  }
  myservo.write(pos);


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



  // Differential Height Sensor Detection
  tsensr.poll();
  bsensr.poll();
  
  compare = bsensr.avg - tsensr.avg;
  //Serial.print(compare);
  //Serial.print(" - ");
  if (compare>DIFF_HEIGHT_RATIO) {
    Serial.print("Weight Detected! ");
    stationary();
    //int detected_pos = myservo.read();
  } else {
    Serial.print("No Weight Detected sad emoji ");
  }

  Serial.print("Bottom:");
  Serial.print(bsensr.avg);
  Serial.print(", ");
  Serial.print("Top:");
  Serial.print(tsensr.avg);
  Serial.print(", ");





 
//  Serial.write(13);
//  Serial.write(10);

}
