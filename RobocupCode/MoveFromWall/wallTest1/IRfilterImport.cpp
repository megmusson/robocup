#include "IRfilter.h"
#include "Arduino.h"

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

int mySense::voltage() {
  int voltage;
  return voltage = avg * (5000 / 1023);
}
