
<<<<<<< HEAD
#define AVERAGE false
#include <Servo.h>
#include<stdio.h>

int analogPin0 = A0; // setting the read pin to A0
int analogPin1 = A1;
int analogPin2 = A2; // setting the read pin to A0
int analogPin3 = A3;

//int switchPin1 = 43;
//long samples = 10;
long val_left; // assigning val as int
long val_right;
long val_front_left; // assigning val as int
long val_front_right;
//long sum = 0;
Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo
=======
int analogPin = A1; // setting the read pin to A0
int switchPin1 = 43;
long samples = 10;
long val; // assigning val as int
long sum = 0;
>>>>>>> 7fdbbe493ab3806cd777c351cb65b6ac3dc62aad

void setup() { // runs once at the begining
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  //pinMode(switchPin1, INPUT);
  pinMode(analogPin0, INPUT);
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  pinMode(analogPin3, INPUT);

  Serial.begin(9600); // setting the baud rate
  //val = analogRead(analogPin0);

  
  //delay(100); //miliseconds

  servoMotorLeft.attach(3);  // attaches the servo pin 3 to the servo object
  servoMotorRight.attach(2);  // attaches the servo pin 2 to the servo object
}

void loop() {
  // put your main code here, to run repeatedly:
<<<<<<< HEAD
  val_right = analogRead(analogPin0);
  val_left = analogRead(analogPin1);
  val_front_right = analogRead(analogPin2);
  val_front_left = analogRead(analogPin3);
    Serial.print("Left:");
  Serial.print(val_left);
  Serial.print(", ");
        Serial.print("Right:");
  Serial.print(val_right);
  Serial.print(", ");
  Serial.print("Front left::");
  Serial.print(val_front_left);
  Serial.print(", ");
    Serial.print("Front right::");

  Serial.println(val_front_right);

// Serial.print("%d, %d, %d, %d", val_left, val_right,val_front_left , val_front_right);
// Serial.println("LEFT:" + string(val_left) + "RIGHT:" + string(val_right));

// Serial.write(13);
//  Serial.write(10);
  delayMicroseconds(10000000); 

  servoMotorLeft.writeMicroseconds(1800);      //Forward Slow  
  servoMotorRight.writeMicroseconds(1800);      // 


  if (val_right > 50) {
    servoMotorLeft.writeMicroseconds(2000);      //Forward Slow3. 
  } else if (val_left > 50) {
    servoMotorRight.writeMicroseconds(2000);
  }
    else if (val_front_right > 50) {
   servoMotorLeft.writeMicroseconds(2000);      //Forward Slow3. 
  } else if (val_front_left > 50) {
    servoMotorRight.writeMicroseconds(2000);
  }

  
=======
  val = analogRead(analogPin);
  Serial.print(val);
  Serial.write(13);
  Serial.write(10);
  delay(10); 
>>>>>>> 7fdbbe493ab3806cd777c351cb65b6ac3dc62aad
}
