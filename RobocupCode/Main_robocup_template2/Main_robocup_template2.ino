

#include <SharpIR.h>
SharpIR right( SharpIR::GP2Y0A41SK0F, A1 );
SharpIR left( SharpIR::GP2Y0A41SK0F, A0 );
SharpIR front( SharpIR::GP2Y0A21YK0F, A2 );

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Pin deffinitions
#define IO_POWER  49

// Serial deffinitions
#define BAUD_RATE 9600


//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work)
// Set as high or low
//**********************************************************************************
void pin_init() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  Serial.println("Pins have been initialised \n");

  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}

//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  int rightDistance = right.getDistance();
  int leftDistance = left.getDistance();
  int frontDistance = front.getDistance();


  Serial.print("right:");
  Serial.print(rightDistance);
  Serial.print("left:");
  Serial.print(leftDistance);
  Serial.print("front:");
  Serial.println(frontDistance);
}
