// SEND EXAMPLE OF SERIAL CAN MODULE
// unsigned char send(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *buf);
// SUPPORT: joney.sui@longan-labs.cc
#include <Arduino.h>
#include "Serial.h"
#include "Motor.h"
#include "InverseKinematic.h"
#include "constants.h"
#include "math.h"
#include "jointTask.h"

float V[6][3] = {{0, 0, 0},
                 {0, 0, 0},
                 {0, 0, 0},
                 {0, 0, 0},
                 {0, 0, 0},
                 {0, 0, 0}};
// Acceleration
float AC[6][3] = {{0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0}};

float theta1 = 0;
float theta2 = 0;
float theta3 = 0;

float theta1_P;
float theta2_P;

const float L1 = 380;
const float L2 = 380;
const float L3 = 0.0;

float t = 0;
int currentStep = 0;
float t0 = 0;
float tf = 0;

int32_t speed1 = 0;
int32_t speed2 = 0;

float qd_X = 0;
float vd_X = 0;
float ad_X = 0;

float qd_Y = 0;
float vd_Y = 0;
float ad_Y = 0;

float qd_Z = 0;
float vd_Z = 0;
float ad_Z = 0;

//float T[6] = {0, 2, 3, 4, 20, 22};
float T[6] = {0, 2, 3, 4, 6, 22};

float start[3] = {760, 0, 0};                 //0
float temp_taget_Bana[3] = {400, -400, 0};     //1
float taget_Bana[3] = {400, 300, 0};          //2

float tranfer[3] = {350, -100, 0};               //3

float temp_taget_Car[3] = {300, -100, 0};        //4
float taget_Car[3] = {650, 0, 0};             //5

float q[6][3] = {{start[0], start[1], start[2]},
                 {temp_taget_Bana[0], temp_taget_Bana[1], temp_taget_Bana[2]},
                 {taget_Bana[0], taget_Bana[1], taget_Bana[2]},
                 {tranfer[0], tranfer[1], tranfer[2]},
                 {temp_taget_Car[0], temp_taget_Car[1], temp_taget_Car[2]},
                 {taget_Car[0], taget_Car[1], taget_Car[2]}};

float q0 = 0;
float q1 = 0;
float v0 = 0;
float v1 = 0;
float ac0 = 0;
float ac1 = 0;
float b[6][1] = {{0}, {0}, {0}, {1}, {1}, {1}};
float M_inv[6][6] = {{0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0}};
float a[6][1];

float px2 = 0;
float py2 = 0;

bool isRunning = true;
float period_t = period_time;
uint32_t periodTime = (uint32_t)(period_t * 1000000.0);

int mode = 1;
float phi = 0;

bool initi = false;
// Hàm tính toán động học nghịch

bool inverseKinematics(float x, float y, float *theta1, float *theta2)
{
  // Tính toán các giá trị trung gian
  float c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float s2 = sqrt(1 - c2 * c2); // sin(theta2) có thể âm hoặc dương

  // Tính toán theta2 (có 2 nghiệm)
  float theta2_1 = atan2(s2, c2);
  float theta2_2 = atan2(-s2, c2);

  // Tính toán theta1 cho mỗi nghiệm của theta2
  float k1 = L1 + L2 * c2;
  float k2 = L2 * s2;
  float theta1_1 = atan2(y, x) - atan2(k2, k1);
  float theta1_2 = atan2(y, x) - atan2(-k2, k1);

  // Chọn nghiệm phù hợp (ví dụ: nghiệm có theta1 và theta2 đều dương)
  if (theta1_1 >= 0 && theta2_1 >= 0)
  {
    *theta1 = theta1_1;
    *theta2 = theta2_1;
    return true; // Tìm thấy nghiệm
  }
  else if (theta1_2 >= 0 && theta2_2 >= 0)
  {
    *theta1 = theta1_2;
    *theta2 = theta2_2;
    return true; // Tìm thấy nghiệm
  }
  else
  {
    return false; // Không tìm thấy nghiệm phù hợp
  }
}

// Hàm tính toán vận tốc góc từ ma trận Jacobi và ma trận vận tốc dài
void calculateAngularVelocity(float J[3][3], float x_dot[3], float theta_dot[3])
{
  // Tính toán vận tốc góc bằng cách nhân ma trận giả nghịch đảo với ma trận vận tốc dài
  for (int i = 0; i < 3; i++)
  {
    theta_dot[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      theta_dot[i] += J[i][j] * x_dot[j]; // Nhân ma trận và cộng dồn
    }
  }
}

void GoHome()
{
  GotoHOME_1();
  GotoHOME_2();
  Move_Rad(Motor_1, 2000, 0);
  Move_Rad(Motor_2, 3600, 0);
  delay(10000);
  Serial.print("Offset_1: ");
  Serial.print(Offset_1);
  Serial.print("   Offset_2: ");
  Serial.println(Offset_2);
}

void setup()
{
  pinMode(Limit_1, INPUT);
  pinMode(Limit_2, INPUT_PULLUP);

  Serial.begin(9600);
  InitCan();

  

  Serial.println("begin");
}
int32_t previousPulse_1 = 0;
int32_t previousPulse_2 = 0;
float q_J[6][3];
/// @brief
void loop()
{
  if (initi == false)
  {
    initi = true;
    delay(2000);
    Set_PID_RAM(Motor_1,Kp_Pos_1,Ki_Pos_1,Kp_Spe_1,Ki_Spe_1,Kp_Tor_1,Ki_Tor_1);
    Set_PID_RAM(Motor_2,Kp_Pos_2,Ki_Pos_2,Kp_Spe_2,Ki_Spe_2,Kp_Tor_2,Ki_Tor_2);
    delay(1000);
    // Offset_1 = -769545;
    // Offset_2 = -1013331;

    Serial.println("Start_gotoHOME");
    GoHome();
    //delay(5000);
    Serial.print("Run periodTime");
    Serial.println(periodTime);
    Serial.println("Finish");
  }

  if (isRunning == true)
  {
    if (t >= T[currentStep + 1])
    {
      currentStep++;
    }
    if (currentStep >= 3)
    {
      isRunning = false;
      Serial.println("FINISHED");
      return;
    }

    Serial.print("Step: ");
    Serial.print(currentStep);
    Serial.print("\t t: ");
    Serial.println(t);

    t0 = T[currentStep];
    tf = T[currentStep + 1];

    float M[6][6] = {{1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5)},
                     {0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4)},
                     {0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3)},
                     {1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5)},
                     {0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4)},
                     {0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3)}};
    switch (mode)
    {
    case 1:
      px2 = q[currentStep][0];
      py2 = q[currentStep][1];
      q_J[currentStep][1] = acos((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2));
      q_J[currentStep][0] = atan2((L1 + L2 * ((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * py2 - L2 * sin(q_J[currentStep][1]) * px2, (L1 + L2 * ((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * px2 + L2 * sin(q_J[currentStep][1]) * py2);
      q_J[currentStep][2] = phi - q_J[currentStep][1] - q_J[currentStep][0];

      px2 = q[currentStep+1][0];
      py2 = q[currentStep+1][1];
      q_J[currentStep+1][1] = acos((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2));
      q_J[currentStep+1][0] = atan2((L1 + L2 * ((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * py2 - L2 * sin(q_J[currentStep+1][1]) * px2, (L1 + L2 * ((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * px2 + L2 * sin(q_J[currentStep+1][1]) * py2);
      q_J[currentStep+1][2] = phi - q_J[currentStep+1][1] - q_J[currentStep+1][0];
      
      q0 = q_J[currentStep][0];
      q1 = q_J[currentStep + 1][0];
      
      // Serial.print("q_J[currentStep][0]: ");
      // Serial.print(q_J[currentStep][0]);
      // Serial.print("\t q_J[currentStep][1]: ");
      // Serial.print(q_J[currentStep][1]);

      // Serial.print("\t q_J[currentStep+1][0]: ");
      // Serial.print(q_J[currentStep+1][0]);
      // Serial.print("\t q_J[currentStep+1][1]: ");
      // Serial.println(q_J[currentStep+1][1]);
      
      break;

    default:
      break;
    }
    // Trajectory plan for X
    q0 = q_J[currentStep][0];
    q1 = q_J[currentStep + 1][0];
    // q0 = q[currentStep][0];
    // q1 = q[currentStep + 1][0];
    v0 = V[currentStep][0];
    v1 = V[currentStep + 1][0];
    ac0 = AC[currentStep][0];
    ac1 = AC[currentStep + 1][0];

    b[0][0] = q0;
    b[1][0] = v0;
    b[2][0] = ac0;
    b[3][0] = q1;
    b[4][0] = v1;
    b[5][0] = ac1;

    inverseMatrix(M, M_inv);

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 1; j++)
      {
        a[i][j] = 0;
        for (int k = 0; k < 6; k++)
        {
          a[i][j] += M_inv[i][k] * b[k][j];
        }
      }
    }
    
    qd_X = (a[0][0]) + (a[1][0]) * t + (a[2][0]) * t * t + (a[3][0]) * t * t * t + (a[4][0]) * t * t * t * t + (a[5][0]) * t * t * t * t * t;
    vd_X = a[1][0] * 1 + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);
    ad_X = 2 * a[2][0] * 1 + 6 * a[3][0] * t + 12 * a[4][0] * pow(t, 2) + 20 * a[5][0] * pow(t, 3);

    // Trajectory plan for Y
    q0 = q_J[currentStep][1];
    q1 = q_J[currentStep + 1][1];
    // q0 = q[currentStep][1];
    // q1 = q[currentStep + 1][1];
    v0 = V[currentStep][1];
    v1 = V[currentStep + 1][1];
    ac0 = AC[currentStep][1];
    ac1 = AC[currentStep + 1][1];
    b[0][0] = q0;
    b[1][0] = v0;
    b[2][0] = ac0;
    b[3][0] = q1;
    b[4][0] = v1;
    b[5][0] = ac1;

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 1; j++)
      {
        a[i][j] = 0;
        for (int k = 0; k < 6; k++)
        {
          a[i][j] += M_inv[i][k] * b[k][j];
        }
      }
    }

    qd_Y = a[0][0] * 1 + a[1][0] * t + a[2][0] * pow(t, 2) + a[3][0] * pow(t, 3) + a[4][0] * pow(t, 4) + a[5][0] * pow(t, 5);

    // vd_X = a[1][0] * 1 + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);
    vd_Y = a[1][0] * 1 + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);

    ad_Y = 2 * a[2][0] * 1 + 6 * a[3][0] * t + 12 * a[4][0] * pow(t, 2) + 20 * a[5][0] * pow(t, 3);

    // Trajectory plan for Z
    q0 = q[currentStep][2];
    q1 = q[currentStep + 1][2];
    v0 = V[currentStep][2];
    v1 = V[currentStep + 1][2];
    ac0 = AC[currentStep][2];
    ac1 = AC[currentStep + 1][2];
    b[0][0] = q0;
    b[1][0] = v0;
    b[2][0] = ac0;
    b[3][0] = q1;
    b[4][0] = v1;
    b[5][0] = ac1;

    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 1; j++)
      {
        a[i][j] = 0;
        for (int k = 0; k < 6; k++)
        {
          a[i][j] += M_inv[i][k] * b[k][j];
        }
      }
    }
    qd_Z = a[0][0] * 1 + a[1][0] * t + a[2][0] * pow(t, 2) + a[3][0] * pow(t, 3) + a[4][0] * pow(t, 4) + a[5][0] * pow(t, 5);
    vd_Z = a[1][0] * 1 + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);
    ad_Z = 2 * a[2][0] * 1 + 6 * a[3][0] * t + 12 * a[4][0] * pow(t, 2) + 20 * a[5][0] * pow(t, 3);

    // Inverse Kinematic
    // x = qd_X;
    // y = qd_Y;
    // z = qd_Z;
    phi = atan2(qd_Y, qd_X);
    px2 = qd_X - L3 * cos(phi);
    py2 = qd_Y - L3 * sin(phi);
    // float c2 = ((px2*px2 + py2*py2 - L1*L1 - L2*L2)/(2*L1*L2));
    theta2 = acos((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2));

    // float c1 = ((L1 + L2*cos(theta2))*px2 + L2*sin(theta2)*py2)/(px2*px2 +py2*py2);
    theta1 = atan2((L1 + L2 * ((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * py2 - L2 * sin(theta2) * px2, (L1 + L2 * ((px2 * px2 + py2 * py2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))) * px2 + L2 * sin(theta2) * py2);
    theta3 = phi - theta1 - theta2;
    float x_dot[3] = {vd_X, vd_Y, vd_Z};

    float J[3][3] = {
        {-L1 * sin(theta1) - L2 * sin(theta1 + theta2) - L3 * sin(theta1 + theta2 + theta3),
         -L2 * sin(theta1 + theta2) - L3 * sin(theta1 + theta2 + theta3),
         -L3 * sin(theta1 + theta2 + theta3)},
        {L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3),
         L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3),
         L3 * cos(theta1 + theta2 + theta3)},
        {0, 0, 1}};
    float J_inv[3][3];
    inverseMatrix_3(J, J_inv);
    float theta_dot[3] = {0, 0, 0}; // Ma trận vận tốc góc
    calculateAngularVelocity(J_inv, x_dot, theta_dot);
    // velocity theta1
    // w1 = ((cos(theta1 + theta2) / (L1 * sin(theta2))) * vd_X + (sin(theta1 + theta2) / (L1 * sin(theta2))) * vd_X + ((L3 * sin(theta3)) / (L1 * sin(theta2))) * vd_X);

    // speed1 = (uint16_t)((w1 * 60 * 1600) / (2 * 3.1415));
    // speed1=rad_s_to_dps_LSB(theta_dot[0]);
    speed1 = theta_dot[0] * 5500.39;
    //  velocity theta2
    // w2 = ((-(L2 * cos(theta1 + theta2) + L1 * cos(theta1)) / (L1 * L2 * sin(theta2))) * vd_Y + (-(L2 * sin(theta1 + theta2) + L1 * sin(theta1)) / (L1 * L2 * sin(theta2))) * vd_Y + (-(L3 * (L1 * sin(theta2 + theta3) + L2 * sin(theta3))) / (L1 * L2 * sin(theta2))) * vd_Y);
    // speed2 = (uint16_t)((w2 * 60 * 1600) / (2 * 3.1415));
    // speed2 =rad_s_to_dps_LSB(theta_dot[1]);
    speed2 = theta_dot[1] * 5042.03;
    //  GotoPosition(motor1,theta1,400);
    //  GotoPosition(motor2,theta2,400);

    // Serial.print("x: \t");
    // Serial.print(qd_X);
    // Serial.print("  y: \t");
    // Serial.println(qd_Y);

    
    // Serial.print("\t Theta3_dot: ");
    // Serial.println(theta_dot[2]);

    
    // Serial.print("theta1 \t");
    // Serial.print(qd_X);
    // Serial.print("  theta2 \t");
    // Serial.println(qd_Y);

    // Serial.print("Theta1_dot: ");
    // Serial.print(vd_X);
    // Serial.print("\t Theta2_dot: ");
    // Serial.println(vd_Y);

    // Serial.print("Speed1: \t");
    // Serial.print((uint16_t)(abs(vd_X* 5500.39)));
    // Serial.print("  Speed2: \t");
    // Serial.println((uint16_t)(abs(vd_X* 5500.39)));
    // Serial.println(" ");
    // Serial.println(" ");

    if (t == 0)
    {
      previousPulse_2 = GetEconder(Motor_2);
    }
    int32_t currentPulse_2 = TransferAngle2Pulse(Motor_2, theta2);
    // int32_t currentPulse_2 = TransferAngle2Pulse(Motor_2,theta2);

    // Move_Rad(Motor_1, abs(speed1) + 10, theta1);
    // Move_Rad(Motor_2, abs(speed2) + 100, theta2);

    Move_Rad(Motor_1, (uint16_t)(abs(vd_X* 5500.39) + 10), qd_X);
    Move_Rad(Motor_2, (uint16_t)(abs(vd_Y* 5042.03) + 10), qd_Y);
    // Move_Speed(Motor_1,speed1*-10);
    // Move_Speed(Motor_2,speed2);
    // delayMicroseconds((uint16_t)((((float)(currentPulse_2 - previousPulse_2)) / ((float)abs(theta_dot[1]))) * 1.818));
    // Serial.print("delayMicroseconds: \t");
    // Serial.println((uint16_t)((((float)(currentPulse_1 - previousPulse_1)) / ((float)speed1)) * 10000.0));
    delayMicroseconds(periodTime);
    t += period_t;
    previousPulse_2 = currentPulse_2;
  }
}

// END FILE