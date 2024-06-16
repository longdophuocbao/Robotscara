#include "Motor.h"
// #include <Serial_CAN_Module.h>
// #include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp2515.h>
#include "constants.h"
struct can_frame canSent;
struct can_frame canRecei;

MCP2515 mcp2515(10);

int32_t Offset_1 = 0;
int32_t Offset_2 = 0;

int8_t Reverse_1 = 1;
int8_t Reverse_2 = -1;

double Limit_Rad_1_0 = -3;
double Limit_Rad_1_1 = 3;
double Limit_Rad_2_0 = -1.5;
double Limit_Rad_2_1 = 2.8;

/// @brief
void InitCan()
{
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS);
    mcp2515.setNormalMode();
    canRecei.can_id = 0x141;
    canRecei.can_dlc = 8;
    canRecei.data[0] = 0x92;
    canRecei.data[1] = 0x0;
    canRecei.data[2] = 0x0;
    canRecei.data[3] = 0x0;
    canRecei.data[4] = 0x0;
    canRecei.data[5] = 0x0;
    canRecei.data[6] = 0x0;
    canRecei.data[7] = 0x0;
}

/// @brief
/// @param _Motor
/// @return
unsigned long GetEconder(uint16_t _Motor)
{
    canRecei.can_id = _Motor;
    canRecei.data[0] = 0x92;
    unsigned long rawValue = 0;
    mcp2515.sendMessage(&canRecei);
    for (size_t i = 0; i < 5; i++)
    {
        if (mcp2515.readMessage(&canRecei) == MCP2515::ERROR_OK)
        {
            if (canRecei.data[0] == 0x92)
            {
                uint32_t a = canRecei.data[4];
                uint32_t b = canRecei.data[3];
                uint32_t c = canRecei.data[2];
                uint32_t d = canRecei.data[1];
                rawValue = (a << 24) | (b << 16) | (c << 8) | d;
                break;
            }
        }
    }
    return rawValue;
}

/// @brief
/// @param Motor
/// @param angle_Rad
/// @return
int32_t TransferAngle2Pulse(uint16_t Motor, float angle_Rad = 0)
{
    if (Motor == Motor_1)
    {
        return ((int32_t)(angle_Rad * 550039.48) + Offset_1 + (int32_t)772052) * Reverse_1;
    }
    else
    {
        return ((int32_t)(angle_Rad * 504202.86) + abs(Offset_2) - (int32_t)1301427) * Reverse_2;
    }
}

/// @brief
/// @param Motor
/// @param speed
/// @param pulse
void Move(uint16_t Motor, uint16_t speed, int32_t pulse)
{
    //uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);
    canSent.can_id = Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0xA4;
    canSent.data[1] = 0x0;
    canSent.data[2] = (uint8_t)(speed);
    canSent.data[3] = (uint8_t)(speed >> 8);
    canSent.data[4] = (uint8_t)pulse;
    canSent.data[5] = (uint8_t)(pulse >> 8);
    canSent.data[6] = (uint8_t)(pulse >> 16);
    canSent.data[7] = (uint8_t)(pulse >> 24);
    mcp2515.sendMessage(&canSent);
    
}

void Move_Speed(uint16_t _Motor, int32_t _Speed)
{
    //uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);
    canSent.can_id = _Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0xA2;
    canSent.data[1] = 0x0;
    canSent.data[2] = 0x0;
    canSent.data[3] = 0x0;
    canSent.data[4] = (uint8_t)_Speed;
    canSent.data[5] = (uint8_t)(_Speed >> 8);
    canSent.data[6] = (uint8_t)(_Speed >> 16);
    canSent.data[7] = (uint8_t)(_Speed >> 24);
    // canSent.data[4] = 0xE8;
    // canSent.data[5] = 0x3;
    // canSent.data[6] = 0x0;
    // canSent.data[7] = 0x0;
    mcp2515.sendMessage(&canSent);
}

/// @brief
/// @param _Motor
/// @param _Speed
/// @param _Angle
void Move_Rad(uint16_t _Motor, uint16_t _Speed, double _Angle)
{
    int32_t pulse = TransferAngle2Pulse(_Motor, _Angle);
    if (_Motor == Motor_1)
    {
        if (_Angle < Limit_Rad_1_0)
        {
            _Angle = Limit_Rad_1_0;
        }
        else if (_Angle > Limit_Rad_1_1)
        {
            _Angle = Limit_Rad_1_1;
        }
    }
    else
    {
        if (_Angle < Limit_Rad_2_0)
        {
            _Angle = Limit_Rad_2_0;
        }
        else if (_Angle > Limit_Rad_2_1)
        {
            _Angle = Limit_Rad_2_1;
        }
    }

    // Serial.print(_Angle);
    // Serial.print("\t");
    // Serial.println(pulse);
    canSent.can_id = _Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0xA4;
    canSent.data[1] = 0x0;
    canSent.data[2] = _Speed;
    canSent.data[3] = _Speed >> 8;
    canSent.data[4] = pulse;
    canSent.data[5] = pulse >> 8;
    canSent.data[6] = pulse >> 16;
    canSent.data[7] = pulse >> 24;
    mcp2515.sendMessage(&canSent);
}
void Set_PID_RAM(uint16_t _Motor,uint8_t _Kp_Pos,uint8_t _Ki_Pos,uint8_t _Kp_Spe,uint8_t _Ki_Spe,uint8_t _Kp_Tor,uint8_t _Ki_Tor )
{
    canSent.can_id = _Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0x31;
    canSent.data[1] = 0x0;
    canSent.data[2] = _Kp_Pos;
    canSent.data[3] = _Ki_Pos;
    canSent.data[4] = _Kp_Spe;
    canSent.data[5] = _Ki_Spe;
    canSent.data[6] = _Kp_Tor;
    canSent.data[7] = _Ki_Tor;
    mcp2515.sendMessage(&canSent);

    canRecei.can_id = _Motor;
    canRecei.data[0] = 0x31;
    mcp2515.sendMessage(&canRecei);
    while(1)
    {
        if (mcp2515.readMessage(&canRecei) == MCP2515::ERROR_OK)
        {
            if (canRecei.data[0] == 0x31)
            {
                if( (canRecei.data[2] == _Kp_Pos) && (canRecei.data[3] == _Ki_Pos) 
                && (canRecei.data[4] == _Kp_Spe) && (canRecei.data[5] == _Ki_Spe) 
                && (canRecei.data[6] == _Kp_Tor) && (canRecei.data[7] == _Ki_Tor))
                {
                    Serial.print(_Motor,HEX);
                    Serial.println(" Successfully - Set PID");
                    break;
                }
            }
        }
        delay(10);
    }

}

/// @brief
void GotoHOME_1()
{
    Serial.println("RUNNING_gotoHOME_Motor_1");
    // Di chuyển về home với tốc độ cao ban đầu
    const uint16_t initialSpeed = 2000;
    const uint16_t initialStep = 1000;
    const uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);
    Serial.print("initialDelay: ");
    Serial.println(initialDelay);
    int32_t temp = 0;

    while (digitalRead(Limit_1) != 0)
    {
        Move(Motor_1, initialSpeed, temp);
        temp -= initialStep;
        delayMicroseconds(initialDelay);
    }
    Serial.println("1_gotoHOME_Motor_1");
    delay(100);

    temp = temp + 50000;
    Move(Motor_1, 1000, temp);
    delay(500);

    // Tìm vị trí home chính xác với tốc độ và bước rất nhỏ
    const uint16_t finalSpeed = initialSpeed / 20;
    const uint16_t finalStep = initialStep / 40;
    const uint16_t finalDelay = (uint16_t)((((float)finalStep) / ((float)finalSpeed)) * 10000.0);
    Serial.print("finalDelay: ");
    Serial.println(finalDelay);
    while (digitalRead(Limit_1) != 0)
    {
        Move(Motor_1, finalSpeed, temp);
        temp = temp - finalStep;
        delayMicroseconds(finalDelay);
    }
    Serial.println("Finish_gotoHOME_Motor_1");
    Offset_1 = (int32_t)(GetEconder(Motor_1) - 0x100000000);
    delay(100);
}

void GotoHOME_2()
{
    Serial.println("RUNNING_gotoHOME_Motor_2");
    // Di chuyển về home với tốc độ cao ban đầu
    const uint16_t initialSpeed = 2500;
    const uint16_t initialStep = 1000;
    const uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);

    int32_t temp = 0;

    while (digitalRead(Limit_2) != 0)
    {
        Move(Motor_2, initialSpeed, temp);
        temp = temp - initialStep;
        delayMicroseconds(initialDelay);
    }
    Serial.println("1_gotoHOME_Motor_2");
    delay(100);

    temp = temp + 8000;
    Move(Motor_2, 800, temp);
    delay(200);

    // Tìm vị trí home chính xác với tốc độ và bước rất nhỏ
    const uint16_t finalSpeed = initialSpeed / 20;
    const uint16_t finalStep = initialStep / 40;
    const uint16_t finalDelay = (uint16_t)((((float)finalStep) / ((float)finalSpeed)) * 10000.0);

    while (digitalRead(Limit_2) != 0)
    {
        Move(Motor_2, finalSpeed, temp);
        temp = temp - finalStep;
        delayMicroseconds(finalDelay);
    }
    Serial.println("Finish_gotoHOME_Motor_2");
    Offset_2 = (int32_t)(GetEconder(Motor_2) - 0x100000000);
    delay(100);
}