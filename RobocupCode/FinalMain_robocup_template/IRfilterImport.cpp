#include "IRfilter.h"
#include "Arduino.h"

mySense::mySense(int pin, float k1, float k2) {
  this->pin = pin;
  this->k1 = k1;
  this->k2 = k2;
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

float mySense::Distance() {
  float voltsPerBit = 0.004882813; // number of volts which corresponds to each bit from analouge
  float distance = k2/(avg*voltsPerBit) + k1;
  return distance;
}

int mySense::voltage() {
  int voltage;
  return voltage = avg * (5000 / 1023);
}
