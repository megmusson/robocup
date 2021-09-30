// random

#include "MedianFilterLib.h"
#define AVERAGE false
#include <Servo.h>
#include<stdio.h>

#define echoPin 27 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 26 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables ULTRASONIC
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement


void setup() { // runs once at the begining
                 // set servo to mid point
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // setting the baud rate
  delay(100);
}


//MedianFilter<int> medianFilter(15);



void loop() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
//  distance = medianFilter.AddValue(distance);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delayMicroseconds(100); 

}
