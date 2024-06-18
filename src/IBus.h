#include <Arduino.h>
#include <IBusBM.h>
#ifndef IBUS_H
#define IBUS_H



void InitIbus();
void ReceiveDATA_Ibus(int *MoveX,int *MoveY);
#endif