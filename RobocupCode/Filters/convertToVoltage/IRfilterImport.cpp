#include "IRfilter.h"
#include "Arduino.h"

#define CIRCBUFFSIZE 35


mySense::mySense(int pin) {
  this->pin = pin;
  pinMode(pin, INPUT);
  buff[0] = analogRead(pin);
}

void mySense::poll() {
  val = analogRead(pin);

  runSum += val - buff[buffCount];

  buff[buffCount] = val;
  buffCount++;
  buffCount = buffCount % CIRCBUFFSIZE;

  avg = ((runSum * 1000) / CIRCBUFFSIZE) / 1000;
}
