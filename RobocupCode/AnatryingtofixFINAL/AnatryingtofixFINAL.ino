/********************************************************************************
                                 ROBOCUP TEMPLATE
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include<stdio.h>
#include "MedianFilterLib.h"
#include <Adafruit_TCS34725.h>      //colour sensor
#include "motors.h"
#include "weight_collection.h"
#include "DFRobot_BNO055.h"
#include "Wire.h"

#define GREEN 0
#define BLUE 1
typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********
BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address

void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch (eStatus) {
    case BNO::eStatusOK:    Serial.println("everything ok"); break;
    case BNO::eStatusErr:   Serial.println("unknow error"); break;
    case BNO::eStatusErrDeviceNotDetect:    Serial.println("device not detected"); break;
    case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
    case BNO::eStatusErrDeviceStatus:       Serial.println("device internal status error"); break;
    default: Serial.println("unknow status"); break;
  }
}

#include "IRfilter.h"
#include <SharpIR.h>
SharpIR right( SharpIR::GP2Y0A41SK0F, A1 );
SharpIR left( SharpIR::GP2Y0A41SK0F, A0 );
SharpIR frontl( SharpIR::GP2Y0A21YK0F, A3 );
SharpIR frontr( SharpIR::GP2Y0A21YK0F, A2 );
SharpIR Mr( SharpIR::GP2Y0A21YK0F, A4 );
SharpIR Ml( SharpIR::GP2Y0A21YK0F, A5 );


MedianFilter<int> medianFilter(30);
MedianFilter<int> median1Filter(2);
MedianFilter<int> median2Filter(2);
MedianFilter<int> median3Filter(2);
MedianFilter<int> median4Filter(2);


MedianFilter<int> mediantrFilter(10);
MedianFilter<int> medianMrFilter(10);
MedianFilter<int> medianbrFilter(10);
MedianFilter<int> medianMlFilter(10);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, lux;
int timecounter = 0;
int watchdogtimer = 0;
//**********************************************************************************
// Local Definitions
//**********************************************************************************
#define AVERAGE false

int mirighttSensor = A5;
int mideftSensor = A4;

long compare_right;
long compare_left;

float newYaw = 0;
float oldYaw = 0;
bool base = 1; //1 for red base, 0 for green
bool onBase = false;
//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Pin deffinitions
#define IO_POWER  49
#define BAUD_RATE 9600


bool backflag = 0;
int moveSpeed = 100;
int turn = 0;

#define FRONTLIMIT 15
#define CORNERLIMIT 20
#define SIDELIMIT 13

// IMU stuff

float yaw;
int yawChange;
float watchdog = 0;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
int homeboi;
//**********************************************************************************
// SETUP
//**********************************************************************************
void setup() {

  pin_init();
  weightservo_setup();
  initServo();
  motor_setup();
  randomSeed(analogRead(7));

  Serial.begin(9600);
  bno.reset();
  while (bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");




  tcs.getRawData(&r, &g, &b, &c);
  delay(10);
  tcs.getRawData(&r, &g, &b, &c);
  if (g > 800) {
    homeboi = GREEN;
    Serial.println("home is green");
  } else if (b > 700) {
    homeboi = BLUE;
    Serial.println("home is blue");
  }
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
  pinMode(mirighttSensor, INPUT);
  pinMode(mideftSensor, INPUT);
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



unsigned long timeboi;
void loop() {
  timeboi = millis();
  collect_weight();
  Serial.println(timeboi);

    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
//  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
  //
  int rightDistance = right.getDistance();
  int leftDistance = left.getDistance();
  int frontDistancer = frontr.getDistance();
  int frontDistancel = frontl.getDistance();
  int MlDistance = Ml.getDistance();
  int MrDistance = Mr.getDistance();
  rightDistance = median1Filter.AddValue(rightDistance);
  leftDistance = median2Filter.AddValue(leftDistance);
  frontDistancer = median3Filter.AddValue(frontDistancer);
  frontDistancel = median4Filter.AddValue(frontDistancel);
  MlDistance = medianMlFilter.AddValue(MlDistance);
  MrDistance = medianMrFilter.AddValue(MrDistance);

  //  Serial.print("right: ");
  //  Serial.print(rightDistance);
  //  Serial.print(" left: ");
  //  Serial.print(leftDistance);
  //  Serial.print(" front right: ");
  //  Serial.print(frontDistancer);
  //  Serial.print(" front left: ");
  //  Serial.print(frontDistancel);
  //  Serial.print("Mid Left: ");
  //  Serial.print(MlDistance);
  //  Serial.print(", ");
  //  Serial.print("Mid right: ");
  //  Serial.println(MrDistance);

  if (rightDistance > SIDELIMIT && leftDistance > SIDELIMIT && frontDistancer > FRONTLIMIT && frontDistancel > FRONTLIMIT && MlDistance > CORNERLIMIT && MrDistance > CORNERLIMIT) {
    go_forward(moveSpeed);
  }
  else if ( frontDistancer < FRONTLIMIT && frontDistancel < FRONTLIMIT && MlDistance < CORNERLIMIT && MrDistance < CORNERLIMIT) {
    rightDistance = right.getDistance();
    leftDistance = left.getDistance();
    if ((rightDistance < SIDELIMIT)) {
      turn_left(moveSpeed);
    }
    else if ((leftDistance < SIDELIMIT)) {
      turn_right(moveSpeed);
    }
    else {
      go_back();
      delay(100);
      turn_right(moveSpeed);
      delay(100);


    }

  }
  else if ((MlDistance < CORNERLIMIT) && (MrDistance < CORNERLIMIT)) {
    turn_left(moveSpeed);
    delay(400);
  }

  else if (rightDistance < SIDELIMIT) {
    turn_left(moveSpeed);

  }

  else if (MrDistance < CORNERLIMIT) {
    turn_left(moveSpeed);

  }

  else if (MrDistance < CORNERLIMIT && rightDistance < SIDELIMIT && frontDistancer < FRONTLIMIT) {
    turn_left(moveSpeed);

  }

  else if (frontDistancer < FRONTLIMIT && frontDistancel < FRONTLIMIT) {
    Serial.println("front is bkocked");
    int randNumber = random(0, 10);
    bool inloop = 1;
    if (randNumber % 2 == 0) {
      while ((inloop == 1) && (frontDistancer < FRONTLIMIT) && (frontDistancel < FRONTLIMIT)) {
        frontDistancer = frontr.getDistance();
        frontDistancel = frontl.getDistance();
        //        Serial.print(" front right: ");
        //        Serial.print(frontDistancer);
        //        Serial.print(" front left: ");
        //        Serial.print(frontDistancel);
        turn_right(moveSpeed);
      }
      inloop = 0;
    }
    else if (randNumber % 2 == 1) {
      while ((inloop == 1) && (frontDistancer < FRONTLIMIT) && (frontDistancel < FRONTLIMIT)) {
        //        Serial.print(" front right: ");
        //        Serial.print(frontDistancer);
        //        Serial.print(" front left: ");
        //        Serial.print(frontDistancel);
        frontDistancer = frontr.getDistance();
        frontDistancel = frontl.getDistance();
        turn_left(moveSpeed);
      }
      inloop = 0;
    }
  }


  else if (leftDistance < SIDELIMIT) {
    turn_right(moveSpeed);
  }

  else if (MlDistance < CORNERLIMIT) {
    turn_right(moveSpeed);

  }
  else if (MlDistance < CORNERLIMIT && leftDistance < SIDELIMIT && frontDistancel < FRONTLIMIT) {
    turn_right(moveSpeed);
  }

  BNO::sEulAnalog_t   sEul;
  sEul = bno.getEul();
  newYaw = sEul.head;

  //  Serial.print("Old Yaw = ");
  //  Serial.print(oldYaw);
  //  Serial.print("New Yaw = ");
  //  Serial.println(newYaw);
  //  Serial.println("Watchdogln = ");
  //  Serial.println(watchdogtimer);

  if ((newYaw - oldYaw) > 15) {
    oldYaw = newYaw;
    watchdogtimer = 0;
  }
  if (watchdogtimer > 500) {
    Serial.println("WATCHDOG is Activated");
    go_back();
    delay(400);
    turn_right(moveSpeed);
    delay(400);
    watchdogtimer = 0;

  }


  watchdogtimer += 1;
  //  if (watchdog > 100) {
  //    Serial.println("STUCK"); // can swap code here to make not stuck
  //  }
  //
  //  if (frontDistancel > 20 && frontDistancer > 20 && backflag == 0) {
  //
  //    // Check if left sensor detects something, turn right if so
  //    if (leftDistance < 15 && rightDistance > 15)  {
  //      turn_right(moveSpeed);
  //      //      Serial.println(micros());
  //      if (MlDistance < 15)  {
  //        turn_right(moveSpeed);
  //        //      Serial.println(micros());
  //      }
  //    }
  //    else if (rightDistance < 15 && leftDistance > 15)  {
  //      turn_left(moveSpeed);
  //
  //    }
  //
  //    else {
  //      go_forward(moveSpeed);
  //    }
  //
  //  }
  //
  //  // CASE If either front sensor detect somthing
  //  else if (frontDistancer < 20 || frontDistancel < 20) {
  //    //    rightDistance = right.getDistance();
  //    //    leftDistance = left.getDistance();
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



  //    if (timecounter %15 == 0) {
  //    tcs.getRawData(&r, &g, &b, &c);
  //    colorTemp = tcs.calculateColorTemperature(r, g, b);
  //      Serial.print("COLOUR IS READ ");
  //
  //    }
  //    if (r > g && r > b && base == 1) {
  //
  //    }
  //    if (g > r && g > b && base == 0) {
  //
  //    }

  //    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  //    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  //    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  //    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  //  Serial.println(" ");
  //  timecounter += 1;
  //  //  Serial.print("timecounter ");
  //  //  Serial.print(timecounter);
  //  Serial.println(timecounter);


  tcs.getRawData(&r, &g, &b, &c);
  if (homeboi == BLUE && b > 700) {
    onBase = true;
    Serial.println("on blue");
  } else if (homeboi == GREEN && g > 800) {
    onBase = true;
    Serial.println("home is green");
  } else {
    onBase = false;
  }



  if (timeboi > 100000 && onBase == true) {
    while (1) {
      Serial.println("BASE");
      stationary();
    }
  }
  if (timeboi > 130010) {
    while (1) {
      Serial.println("STOP");
      go_forward(0);
    }
  }

}
