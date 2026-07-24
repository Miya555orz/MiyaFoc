#ifndef __PAREMETER_H_
#define __PAREMETER_H_

#include "PID.h"

#define POLE_PAIRS 7 
#define R_SHUNT 0.01           
#define OP_GAIN 50             
#define MAX_CURRENT 2          
#define ADC_REFERENCE_VOLT 3.3 
#define ADC_BITS 12            
#define ADC_OFFSET_VOLT 1.65
#define L_D 0.00086
#define L_Q 0.00086
#define R_S 2.55
#define PSI_F 0.0035
#define ELEC_SPEED POLE_PAIRS*Filter_Speed 

extern float Angle;
extern float Elec_Angle;
extern float Encoder_Angle;
extern float Motor_Angle;

extern PID_Handle PID_Speed;
extern PID_Handle PID_Torque_d;
extern PID_Handle PID_Torque_q;
extern PID_Handle PID_Position;
extern float Filter_I_d;
extern float Filter_I_q;
extern float Filter_Speed;
extern float Motor_Position_Rad;

#endif 
