#include "BaseMode.h"

StructTrajectory Trajectory(1, 0, 0);

double distance(double x1, double y1, double x2, double y2) 
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy); 
}

int compareNumbers(float previous, float current) 
{
    if (current > previous) {
        return -1; // Số sau lớn hơn số trước
    } else if (current < previous) {
        return 1; // Số sau nhỏ hơn số trước
    } else {
        return 0; // Hai số bằng nhau
    }
}

void Move_basemode_auto_tranjectory()
{
    Serial.println("Move_basemode ");
    int32_t Encoder_1 = GetEconder(Motor_1);
    int32_t Encoder_2 = GetEconder(Motor_2);
    float theta1_old = TransferPulse2Angle(Motor_1, Encoder_1 );
    float theta2_old = TransferPulse2Angle(Motor_2, Encoder_2 );
    
    float X_old =0;
    float Y_old =0;
    
    int distance_X_move = 0;
    int distance_Y_move = 0;
    ReceiveDATA_Ibus(&distance_X_move,&distance_Y_move);

    float X_Target = X_old + distance_X_move;
    float Y_Target = Y_old + distance_Y_move;

    Forward_Kinemaic(theta1_old,theta2_old,&X_old,&Y_old);

    int index_trajectory = abs(round(distance(X_old,Y_old, X_Target, Y_Target))) * 2 ;

    float X_temp=0;
    float Y_temp=0;

    float theta1=0;
    float theta2=0;

    for (int Value_Index = 0; Value_Index < index_trajectory + 1; Value_Index++)
    {
        X_temp= X_old + Value_Index*0.5*compareNumbers(X_Target,X_old) ;
        Y_temp= Y_old + Value_Index*0.5*compareNumbers(X_Target,X_old) ;
        Inverse_Kinemaic(X_temp,Y_temp,&theta1,&theta2);

        Trajectory.theta[0][Value_Index] = theta1;
        Trajectory.theta[1][Value_Index] = theta2;

    }
    for (int Index = 0; Index < index_trajectory + 1; Index++)
    {
        Move_Rad(Motor_1,1000,Trajectory.theta[0][Index]);
        Move_Rad(Motor_2,1000,Trajectory.theta[1][Index]);
    }
    Trajectory.deallocateMemory();

}



