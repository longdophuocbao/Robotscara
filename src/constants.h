/** @file constants.h
    @brief A Documented file.
    
    Here declare all constant variables
    For schematic please refer to:
    For additional pin functionality refer to:

*/
#ifndef CONSTANTS_H
#define CONSTANTS_H

#define can_tx 9    // tx of serial can module connect to D2
#define can_rx 50    // rx of serial can module connect to D3

#define Limit_1 8   // pin limit switch Motor 1
#define Limit_2 9   // pin limit switch Motor 2
#define B_Start 10
#define Led_Status_SERIAL 11
#define Knift 12

#define Motor_1 0x141   //ID motor 1
#define Motor_2 0x142   //ID motor 2

#define Kp_Pos_1 80   
#define Ki_Pos_1 80   

#define Kp_Spe_1 40   
#define Ki_Spe_1 30 

#define Kp_Tor_1 40   
#define Ki_Tor_1 40 

#define Kp_Pos_2 80   
#define Ki_Pos_2 80   

#define Kp_Spe_2 40   
#define Ki_Spe_2 30 

#define Kp_Tor_2 40   
#define Ki_Tor_2 40 

#define period_time 0.02

// #define Limit_Rad_1_0 -3;
// #define Limit_Rad_1_1 3;
// #define Limit_Rad_2_0 -1.5 ;
// #define Limit_Rad_2_1 2.8;

#define MinX 100
#define MaxX 600
#define MinY -400
#define MaxY 200


#define COUNTS_PER_REV 1296000 
#define GEAR_RATIO (48.0 / 18.0) 


#endif