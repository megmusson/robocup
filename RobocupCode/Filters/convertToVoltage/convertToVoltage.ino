#include "IRfilter.h"
#define CIRCBUFFSIZE 35
int leftsensepin = A0;
int rightsensepin = A1;
int frontleftsensepin = A5;
int frontrightsensepin = A7;

mySense lsensr(leftsensepin);
mySense rsensr(rightsensepin);

mySense flsensr(frontleftsensepin);
mySense frsensr(frontrightsensepin);

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600); // setting the baud rate
  delay(100);
}

void loop() {
  lsensr.poll();
  rsensr.poll();
  Serial.print(lsensr.avg);
  Serial.print(",");
  Serial.println(rsensr.avg);
  
  delay(10);
}
