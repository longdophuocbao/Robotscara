#include <Arduino.h>
#include "Constants.h"

#ifndef KINEMATIC_H
#define KINEMATIC_H
 

void Forward_Kinemaic(float _Theta1, float _Theta2, float *_X, float *_Y);
void Inverse_Kinemaic(float _X, float _Y, float *_Theta1, float *_Theta2);

struct StructTrajectory
{
    unsigned int Total_Index;
    unsigned int ACC_Time, DEC_Time;
    float **theta;
    float **speed;

    // Constructor để cấp phát bộ nhớ động
    StructTrajectory(unsigned int totalIndex, unsigned int accTime, unsigned int decTime)
        : Total_Index(totalIndex), ACC_Time(accTime), DEC_Time(decTime)
    {

        // Sử dụng Total_Index để xác định số cột n
        int n = Total_Index + 1;

        // Cấp phát bộ nhớ động cho mảng theta
        theta = new float *[6];
        for (int i = 0; i < 6; ++i)
        {
            theta[i] = new float[n];
        }

        // Cấp phát bộ nhớ động cho mảng speed
        speed = new float *[6];
        for (int i = 0; i < 6; ++i)
        {
            speed[i] = new float[n];
        }
    }

    // Destructor để giải phóng bộ nhớ
    ~StructTrajectory()
    {
        deallocateMemory();
    }

    // Hàm giải phóng bộ nhớ đã cấp phát
    void deallocateMemory()
    {
        for (int i = 0; i < 6; ++i)
        {
            delete[] theta[i];
            delete[] speed[i];
        }
        delete[] theta;
        delete[] speed;
    }
};

#endif