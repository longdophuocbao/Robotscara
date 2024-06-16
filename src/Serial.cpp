#include "Serial.h"

const int MAX_DATA_LENGTH = 10; // Kích thước tối đa của dữ liệu nhận được

uint8_t angle = 0;

void ReceiveSerial()
{
    if (Serial.available() > 0)
    {
        char input[50] = "0 0 0";
        // Read value until \n
        Serial.readBytesUntil('\n', input, sizeof(input));
        // Take the first token
        char *token = strtok(input, " ");
        uint16_t nums[3];
        int index = 0;

        while (token != NULL && index < 3)
        {
            nums[index] = atoi(token);
            token = strtok(NULL, " ");
            index++;
        }
        Serial.print(nums[0]);
        Serial.print(nums[1]);
        Serial.print(nums[2]);
        if (nums[0] == 1)
        {
            digitalWrite(13, HIGH);
        }
        if (nums[1] == 1)
        {
            digitalWrite(13, LOW);
        }
    }
}
