#include "Kinematic.h"

void Forward_Kinemaic(float _Theta1, float _Theta2, float *_X, float *_Y)
{
    *_X= L1*cos(_Theta1)+ L2 * cos(_Theta1+_Theta2);
    *_Y= L1*sin(_Theta1)+ L2 * sin(_Theta1+_Theta2);
}

void Inverse_Kinemaic(float _X, float _Y, float *_Theta1, float *_Theta2)
{
    uint16_t _l1= L1;
    uint16_t _l2= L2;
    *_Theta2 = acos((_X * _X + _Y * _Y - _l1 *_l1 - _l2 * _l2) / (2 * _l1 * _l2));
    *_Theta1 = atan2((_l1 + _l2 * ((_X * _X + _Y * _Y - _l1 * _l1 - _l2 * _l2) / (2 * _l1 * _l2))) * _Y - _l2 * sin(*_Theta2) * _X, (_l1 + _l2 * ((_X * _X + _Y * _Y - _l1 * _l1 - _l2 * _l2) / (2 * _l1 * _l2))) * _X + _l2 * sin(*_Theta2) * _Y);
}
