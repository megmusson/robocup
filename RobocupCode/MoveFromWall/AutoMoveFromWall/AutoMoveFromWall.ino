#include <stdio.h>

#define AVERAGE false

int aPin0 = A0; // setting the read pin to A0
int aPin1 = A1;
char string[50];
long samples = 10;
int val0; // assigning val as int
int val1;
long sum = 0;

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  
  pinMode(aPin0, INPUT);
  pinMode(aPin1, INPUT);

  Serial.begin(9600); // setting the baud rate
  delay(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  val0 = analogRead(aPin0);
  val1 = analogRead(aPin1);
  sprintf(string, "Pin0 = %d, Pin1 = %d \n", val0, val1);
  Serial.print(string);
  delay(100);
}
