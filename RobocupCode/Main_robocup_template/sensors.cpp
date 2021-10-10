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
//global 
//
//void run_sensors(){
//  int rightDistance = right.getDistance();
//  int leftDistance = left.getDistance();
//  int frontDistancer = frontr.getDistance();
//  int frontDistancel = frontl.getDistance();
//
//  int blDistance = bl.getDistance();
//  int tlDistance = tl.getDistance();
//  int brDistance = br.getDistance();
//  int trDistance = tr.getDistance();
//
//  rightDistance = median1Filter.AddValue(rightDistance);
//  leftDistance = median2Filter.AddValue(leftDistance);
//  frontDistancer = median3Filter.AddValue(frontDistancer);
//  frontDistancel = median4Filter.AddValue(frontDistancel);
//
//  blDistance = medianblFilter.AddValue(blDistance);
//  tlDistance = mediantlFilter.AddValue(tlDistance);
//  brDistance = medianbrFilter.AddValue(brDistance);
//  trDistance = mediantrFilter.AddValue(trDistance);
//}
