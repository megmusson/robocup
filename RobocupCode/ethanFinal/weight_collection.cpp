/************************************
          weight_collection.cpp
 *************************************/

/* This is for functions and tasks for
    finding and collecting weights  */
bool COLLECTBOI = 0;

#include "weight_collection.h"
#include "Arduino.h"
# include "motors.h"
#include <Servo.h>


// Speaker Pins
// Right side
#define S1_PWM 44    // pwm pin motor (digital output)
#define S1_INA 42   // control pin INA (digital output)
#define S1_INB 43    // control pin INB (digital output)
#define S1_EN 45     // current sense pin (analog input)

// Left side
#define S2_PWM 40    // pwm pin motor (digital output)
#define S2_INA 38   // control pin INA (digital output)
#define S2_INB 39    // control pin INB (digital output)
#define S2_EN 41     // current sense pin (analog input)

Servo myServoRight;            // for differential height servo
Servo myServoLeft;
int myServoRightPin = 5;
int myServoLeftPin = 4;
//
int pos = 0;        // position of servo
bool directFlag = 0;// flag for servo
int angle = 0;
//int weight_pos;

//void weight_scan(/* whatever parameters */)
//{
//  /* Use sensors to search for weights,
//   * Switch to WEIGHT_FOUND state if a weight is found   */
//   Serial.println("Looking for weights \n");
//}

void motor_setup() {
  Serial.begin(9600);
  pinMode(S1_INA, OUTPUT);
  pinMode(S1_EN, OUTPUT);
  pinMode(S1_INB, OUTPUT);
  pinMode(S1_PWM, OUTPUT);

  pinMode(S2_INA, OUTPUT);
  pinMode(S2_EN, OUTPUT);
  pinMode(S2_INB, OUTPUT);
  pinMode(S2_PWM, OUTPUT);
  digitalWrite(S1_EN, HIGH); // Set the enable pin high alwaus
  digitalWrite(S1_INB, LOW);
  digitalWrite(S1_PWM, HIGH);

  digitalWrite(S2_EN, HIGH); // Set the enable pin high alwaus
  digitalWrite(S2_INB, LOW);
  digitalWrite(S2_PWM, HIGH);
}

void weightservo_setup() {

  myServoRight.attach(myServoRightPin);                   // attached servo on pin () to ervo object
  myServoLeft.attach(myServoLeftPin);
  myServoRight.write(90);                    // set servo to mid point
  myServoLeft.write(90);                    // set servo to mid point

}

void servo_rotate() {
      myServoRight.write(pos);
    myServoLeft.write(180-pos);
  if (pos < 1) {
    directFlag = 0;
    Serial.println("directFlag = 0");
  }
  if (pos > 100) {
    directFlag = 1;
  }

  if (directFlag == 0) {
    pos += 5;
    //Serial.print("pos = ");
    //Serial.print(pos );
  }
  if (directFlag == 1) {
    pos -= 5;
    //Serial.print("pos = ");
    //Serial.print(pos );
  }
  delay(20);
}


void collect_weight()
{
  digitalWrite(S1_INA, LOW);
  digitalWrite(S1_INB, HIGH);
  if (COLLECTBOI) {
  analogWrite(S1_PWM, 255);
  }else {
  analogWrite(S1_PWM, 0);
}
  digitalWrite(S2_INA, HIGH);
  digitalWrite(S2_INB, LOW);
  if (COLLECTBOI) {
  analogWrite(S2_PWM, 255);
  } else {
    analogWrite(S2_PWM, 0);
  }
}

int readservoleft(){
int value = myServoLeft.read();
}

void servomove(int posi){
        myServoRight.write(posi);
    myServoLeft.write(180-posi);
  
}
