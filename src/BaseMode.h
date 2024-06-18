#include <Arduino.h>
#include <IBus.h>
#include <Motor.h>
#include "Kinematic.h"

#ifndef BASEMODE_H
#define BASEMODE_H
double distance(double x1, double y1, double x2, double y2) ;
int compareNumbers(float previous, float current) ;
void Move_basemode_auto_tranjectory();




#endif