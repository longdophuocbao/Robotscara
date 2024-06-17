#include "Serial.h"
#include "constants.h"

const int MAX_DATA_LENGTH = 10; // Kích thước tối đa của dữ liệu nhận được

bool ReceiveSerial()
{
    float nums[3];
    if (Serial.available() > 0)
    {
        digitalWrite(Led_Status_SERIAL,HIGH);
        char input[50] = "0 0 0";
        // Read value until \n
        Serial.readBytesUntil('\n', input, sizeof(input));
        // Take the first token
        char *token = strtok(input, " ");
        int index = 0;

        while (token != NULL && index < 3)
        {
            nums[index] = atoi(token);
            token = strtok(NULL, " ");
            index++;
        }
    }
    float PositionX = nums[2];
    float PositionY = nums[1];
    if ((PositionX >= MinX) && (PositionX <= MaxX) && (PositionY <= MaxY) && (PositionY <= MaxY))
        return true;
    else
        return false;
}
