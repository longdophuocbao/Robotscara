#include "Ibus.h"
IBusBM IBus;
void InitIbus()
{
    IBus.begin(Serial);
}

void ReceiveDATA_Ibus(int *MoveX,int *MoveY)
{
    int sensorValue_5 = IBus.readChannel(5);  
    int sensorValue_0 = IBus.readChannel(0); 
    *MoveX = map(sensorValue_5, 1500, 2000, -30, 30); 
    *MoveY = map(sensorValue_0, 1000, 2000, -30, 30); 
    Serial.print("MoveX: ");
    Serial.print(*MoveX);
    Serial.print("\tMoveY: ");
    Serial.println(*MoveY);
}