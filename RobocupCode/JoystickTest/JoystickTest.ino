#include <Servo.h>
#include <SharpIR.h>

#define SERVO_RATIO 1

int analogPin = A0; // setting the read pin to A0

Servo myservoA;      // create servo object to control a servo
Servo myservoB;      // create servo object to control a servo


void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600); // setting the baud rate
}

void loop() {
  // put your main code here, to run repeatedly:
  int val; // assigning val as int
  val = analogRead(analogPin) ;  // read the input pin
  Serial.println(val); // printing val to the serial output to the computer
  delay(500); // delaying for half a second
}
