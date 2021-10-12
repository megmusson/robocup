/********************************************************************************
                                 ROBOCUP TEMPLATE
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include<stdio.h>
#include "MedianFilterLib.h"

//#include <Herkulex.h>             //smart servo
#include <Wire.h>                   //for I2C and SPI
#include <Adafruit_TCS34725.h>      //colour sensor


// Custom headers
#include "motors.h"
//#include "sensors.h"
#include "weight_collection.h"
//#include "return_to_base.h"
int weight_pos;


#include "IRfilter.h"
#include <SharpIR.h>
SharpIR right( SharpIR::GP2Y0A41SK0F, A1 );
SharpIR left( SharpIR::GP2Y0A41SK0F, A0 );
SharpIR frontl( SharpIR::GP2Y0A21YK0F, A3 );
SharpIR frontr( SharpIR::GP2Y0A21YK0F, A2 );
SharpIR tr( SharpIR::GP2Y0A21YK0F, A7 );
SharpIR tl( SharpIR::GP2Y0A21YK0F, A4 );
SharpIR br( SharpIR::GP2Y0A21YK0F, A6 );
SharpIR bl( SharpIR::GP2Y0A21YK0F, A5 );


MedianFilter<int> medianFilter(30);
MedianFilter<int> median1Filter(2);
MedianFilter<int> median2Filter(2);
MedianFilter<int> median3Filter(2);
MedianFilter<int> median4Filter(2);


MedianFilter<int> mediantrFilter(10);
MedianFilter<int> mediantlFilter(10);
MedianFilter<int> medianbrFilter(10);
MedianFilter<int> medianblFilter(10);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
int timecounter = 0;

//**********************************************************************************
// Local Definitions
//**********************************************************************************
#define AVERAGE false
#define DIFF_HEIGHT_RATIO 50
#define SENSOR_RATIO_CAL 20

int toprightSensor = A7; // green LED (differenetial height)
int botrightSensor = A6; // red LED
int topleftSensor = A5;
int botleftSensor = A4;

long compare_right;
long compare_left;
bool spinRightFlag = 0;
bool spinLeftFlag = 0;

uint16_t r, g, b, c, colorTemp, lux;
bool base = 1; //1 for red base, 0 for green

//int pos = 0;        // position of servo
//bool directFlag = 0;// flag for servo
//int angle = 0;
//int weight_pos;
//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Pin deffinitions
#define IO_POWER  49
#define BAUD_RATE 9600
#define echoPin 27 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 26 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables ULTRASONIC
long duration; // variable for the duration of sound wave travel
int backDistance; // variable for the distance measurement

bool backflag = 0;
int moveSpeed = 100;
int turn = 0;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
//void robot_init();

//**********************************************************************************
// SETUP
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  weightservo_setup();

  //  robot_init();
  //  Wire.begin();
  initServo();
  motor_setup();


}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work)
// Set as high or low
//**********************************************************************************
void pin_init() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(toprightSensor, INPUT);
  pinMode(botrightSensor, INPUT);
  pinMode(topleftSensor, INPUT);
  pinMode(botleftSensor, INPUT);
  Serial.println("Pins have been initialised \n");

  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}
//**********************************************************************************
// Set default robot state
//**********************************************************************************
//void robot_init() {
//    Serial.println("Robot is ready \n");
//}




void loop() {

  //  collect_weight();
  //  servo_rotate();
  servomove(10);


  int rightDistance = right.getDistance();
  int leftDistance = left.getDistance();
  int frontDistancer = frontr.getDistance();
  int frontDistancel = frontl.getDistance();

  int blDistance = bl.getDistance();
  int tlDistance = tl.getDistance();
  int brDistance = br.getDistance();
  int trDistance = tr.getDistance();

  rightDistance = median1Filter.AddValue(rightDistance);
  leftDistance = median2Filter.AddValue(leftDistance);
  frontDistancer = median3Filter.AddValue(frontDistancer);
  frontDistancel = median4Filter.AddValue(frontDistancel);

  blDistance = medianblFilter.AddValue(blDistance);
  tlDistance = mediantlFilter.AddValue(tlDistance);
  brDistance = medianbrFilter.AddValue(brDistance);
  trDistance = mediantrFilter.AddValue(trDistance);


  //  digitalWrite(trigPin, LOW);
  //  delayMicroseconds(2);
  //  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  //  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(10);
  //  digitalWrite(trigPin, LOW);
  //  // Reads the echoPin, returns the sound wave travel time in microseconds
  //  duration = pulseIn(echoPin, HIGH);
  //  // Calculating the distance
  //  backDistance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  //  backDistance = medianFilter.AddValue(backDistance);
  //
  //  Serial.print("Distance: ");
  //  Serial.print(backDistance);
  //  Serial.println(" cm");
  //  delayMicroseconds(100);
  //
  //    Serial.print("right:");
  //    Serial.print(rightDistance);
  //    Serial.print("left:");
  //      Serial.print(leftDistance);
  //      Serial.print("front right:");
  //      Serial.println(frontDistancer);
  //      Serial.print("front left:");
  //      Serial.println(frontDistancel);

  Serial.print("Rimmght Bottom:");
  Serial.print(brDistance);
  Serial.print(", ");
  Serial.print("Right Top:");
  Serial.print(trDistance);
  Serial.print(", ");
  Serial.print("Left Bottom:");
  Serial.print(blDistance);
  Serial.print(", ");
  Serial.print("Left Top:");
  Serial.println(tlDistance);
  //    Serial.println(frontDistancer);

  //
  //  if (frontDistancel > 20 && frontDistancer > 20 && backflag == 0) {
  ////    rightDistance = right.getDistance();
  ////    leftDistance = left.getDistance();
  //
  //    // Check if left sensor detects something, turn right if so
  //    if (leftDistance < 15 && rightDistance > 15)  {
  //      turn_right(moveSpeed);
  //      //      Serial.println(micros());
  //    }
  //    else if (rightDistance < 15 && leftDistance > 15)  {
  //      turn_left(moveSpeed);
  //
  //    }
  //    else {
  //      go_forward(moveSpeed);
  //    }
  //
  //  }

  //  // CASE If either front sensor detect somthing
  //  else if (frontDistancer < 20 || frontDistancel < 20) {
  ////    rightDistance = right.getDistance();
  ////    leftDistance = left.getDistance();
  //
  //    // Check if left sensor detects something, turn right if so
  //    if (leftDistance < 15 && rightDistance > 15 && backflag == 0)  {
  //      turn_right(moveSpeed);
  //
  //    }
  //
  //    if (rightDistance < 15 && leftDistance > 15 && backflag == 0)  {
  //      turn_left(moveSpeed);
  //
  //    }
  //
  //    // If left and right sensor detect nothing, turn left (CHANGE THIS LATER)
  //    if (rightDistance > 15 && leftDistance > 15 && backflag == 0 )  {
  //      turn_right(moveSpeed);
  //    }
  //
  //    // If both left and right sensor detects something
  //    if (rightDistance < 15 && leftDistance < 15 && backflag == 0)  {
  //      backflag = 1;
  //    }
  //  }
  //
  //  else if (backflag == 1) {
  //    while (rightDistance < 15 || leftDistance < 15) {
  //      go_back();
  //      rightDistance = right.getDistance();
  //      leftDistance = left.getDistance();
  //    }
  //
  //    backflag = 0;
  //    turn_right(moveSpeed);
  //    delay(3000);
  //
  //  }


  //  compare_right =  trDistance - brDistance;
  //  compare_left =  - blDistance ;
  //  Serial.print("Left comparison");
  //  Serial.print(compare_left);

  //  if (compare_right > DIFF_HEIGHT_RATIO) {
  //    Serial.print("Weight Detected! ");
  //    spinRightFlag = 1;
  //    stationary();   // stop the robot
  //    spin_right(moveSpeed);
  //  }
  //  if ((blDistance < 15 && tlDistance > 25)) {
  //    Serial.print("Weight Detected! ");
  //    spinLeftFlag = 1;
  //    weight_pos = readservoleft();   // read weight position from servo
  //    stationary();
  //    spin_left(moveSpeed);
  //    delay(50);
  //    spinLeftFlag = 0;
  //  }

  // stop the robot
  //    if (weight_pos < 30) {
  //      Serial.print("Position <30 ");
  //
  //      spin_left(moveSpeed);
  //      delay(50);
  //      spinLeftFlag = 0;
  //
  //    }
  //    else if (weight_pos < 60 && weight_pos > 30) {
  //      Serial.print("30 < Position <60 ");
  //
  //      spin_left(moveSpeed);
  //      delay(100);
  //      spinLeftFlag = 0;
  //
  //    }
  //
  //    else if (weight_pos < 90 && weight_pos > 60) {
  //      Serial.print("60 < Position < 90 ");
  //
  //      spin_left(moveSpeed);
  //      delay(150);
  //      spinLeftFlag = 0;
  //
  //    }
  //
  //    else if (weight_pos < 130 && weight_pos > 90) {
  //      Serial.print("90 < Position < 120 ");
  //
  //      spin_left(moveSpeed);
  //      delay(200);
  //      spinLeftFlag = 0;
  //
  //    }
  //    //int detected_pos = myServoRight.read();
  //  }
  //  else {
  //    Serial.println("No Weight Detected");
  //        myServoRight.write(pos);
  //        myServoLeft.write(180-pos);
  //  }

  if (timecounter % 15 == 0) {
    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    Serial.print("COLOUR IS READ ");

  }
  if (r > g && r > b && base == 1) {

  }
  if (g > r && g > b && base == 0) {

  }

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  timecounter += 1;
  //  Serial.print("timecounter ");
  //  Serial.print(timecounter);


}
