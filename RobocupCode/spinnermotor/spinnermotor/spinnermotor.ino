
#include <Servo.h>
#include<stdio.h>

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

Servo servoMotorLeft;      // create servo object to control a servo
Servo servoMotorRight;      // create servo object to control a servo

void setup() {
  Serial.begin(9600);
  pinMode(S1_INA, OUTPUT);
  pinMode(S1_EN, OUTPUT);
  pinMode(S1_INB, OUTPUT);
  pinMode(S1_PWM, OUTPUT);
  
  pinMode(S2_INA, OUTPUT);
  pinMode(S2_EN, OUTPUT);
  pinMode(S2_INB, OUTPUT);
  pinMode(S2_PWM, OUTPUT);
  servoMotorLeft.attach(3);  // attaches the servo pin 2 to the servo object
  servoMotorRight.attach(3);  // attaches the servo pin 3 to the servo object
  digitalWrite(S1_EN, HIGH); // Set the enable pin high alwaus
  digitalWrite(S1_INB, LOW);
  digitalWrite(S1_PWM, HIGH);
  
  digitalWrite(S2_EN, HIGH); // Set the enable pin high alwaus
  digitalWrite(S2_INB, LOW);
  digitalWrite(S2_PWM, HIGH);
}
void go_forward() {
  Serial.print("GO FORWARD ");
  servoMotorLeft.writeMicroseconds(2000);
  servoMotorRight.writeMicroseconds(2000);
}


void loop() {

  digitalWrite(S1_INA, LOW);
  digitalWrite(S1_INB, HIGH);
  analogWrite(S1_PWM, 255);
  
  digitalWrite(S2_INA, HIGH);
  digitalWrite(S2_INB, LOW);
  analogWrite(S2_PWM, 255);
  go_forward();
  delay(50);

}
