////************************************
////         sensors.cpp       
////************************************
//
// // This file contains functions used to read and average
// // the sensors.
//
//
//#include "sensors.h"
//#include "Arduino.h"
//#include "IRfilter.h"
//
//// Local definitions
//#define BUF_SIZE 4
//
//
//// Read ultrasonic value
////void read_ultrasonic(/* Parameters */){
////  Serial.println("Ultrasonic value \n");
////}
//
//// Read infrared value
////void read_infrared(/* Parameters */){
////  Serial.println("Infrared value \n");  
////}
//
//// Read colour sensor value
////void read_colour(/* Parameters */){
////  Serial.println("colour value \n");  
////}
//
//// Pass in data and average the lot
////void sensor_average(/* Parameters */){
////  Serial.println("Averaging the sensors \n");
////}
//
//int readIRSensor(int IRPin, circBuf_t chosenBuf) {
//  int IRreading = analogRead(IRPin);
//  writeCircBuf(&chosenBuf, IRreading);
//
//  /*Serial.print(IRPin);
//  Serial.print(" ");
//  Serial.println(IRreading);*/
//  
//  return meanBuffer(chosenBuf);
//}
//
//
//int meanBuffer(circBuf_t chosenBuf) {
//  int i = 0;
//    long sum = 0;
//
//    // sums buffer values
//    for (i = 0; i < BUF_SIZE; i++) {
//        sum = sum + readCircBuf (&chosenBuf);
//    }
//
//    return (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
//}
