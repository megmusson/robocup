#include "IRfilter.h"

#define DIFF_HEIGHT_RATIO 80
#define SENSOR_RATIO_CAL 20

int topSensor = A10; // green LED
int botSensor = A11; // red LED

long compare;
mySense tsensr(topSensor);
mySense bsensr(botSensor);

void setup() {
  pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
  digitalWrite(49, 1);                 //Enable IO power on main CPU board
  Serial.begin(9600); // setting the baud rate
  delay(100);
}

void loop() {
  tsensr.poll();
  bsensr.poll();
  
//  Serial.print(tsensr.voltage());
//  Serial.print(",");
//  Serial.println(bsensr.voltage());
  
  compare = bsensr.avg - tsensr.avg;
  Serial.print(compare);
  Serial.print(" - ");
  if (compare>DIFF_HEIGHT_RATIO) {
    Serial.println("Weight Detected!");
  } else {
    Serial.println("No Weight Detected sad emoji");
  }


  delay(100);
}
