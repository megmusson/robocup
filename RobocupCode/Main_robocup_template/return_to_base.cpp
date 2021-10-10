//************************************
//         return_to_base.cpp       
//************************************

 // This file contains functions used to return to and
 // detect bases

#include "return_to_base.h"
#include "Arduino.h"

// Local definitions
//#define 

// Return to home base
void return_to_base(/* Parameters */){
  Serial.println("Returning to base \n");
}

// Detect what base (if any) the robot is above
void detect_base(/* Parameters */){
  Serial.println("Base detected \n");
}

// Unload weights in home base
void unload_weights(/* Parameters */){
  Serial.println("Unloading weights \n");
}

  
//    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
//    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
//    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
//    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
//    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
//    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
//    Serial.println(" ");
