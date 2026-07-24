#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>

typedef struct
{
  float Kp;
	float Ki;
	float Kd;
	
	float Current_Error;
	float Last_Error;
	float Error_Sum;
	
	float Integral;
	float Differential;
	
	float Integral_Max;
	float Integral_Min;
	
	float OutPut;
	
	float OutMax;
	float OutMin;
	
}PID_Handle;


void PID_Init(PID_Handle *PID_Handler,float Kp,float Ki,float Kd,
	            float Intemax,float Intemin,float Outmax,float Outmin);
float PID_Calculate(PID_Handle *PID_Handler,float Current_Error);
float Speed_Loop(PID_Handle *PID_Handler,float Target_Speed);
float Torque_q_Loop(PID_Handle *PID_Handler,float Target_q);
float Torque_d_Loop(PID_Handle *PID_Handler,float Target_d);
float Position_Loop(PID_Handle *PID_Handler,float Target_Position);
void Speed_Contorl(PID_Handle *PID_Handler,float Target_Speed);
void Position_Control(PID_Handle *PID_Handler,float Target_Position);
void Torque_Control(PID_Handle *PID_Handler_q,float Target_q,
                    PID_Handle *PID_Handler_d,float Target_d);
void Toruqe_Position_Speed_Control(PID_Handle *PID_Handler_Position,float Target_Position,
	                                 PID_Handle *PID_Handler_Speed,
																	 PID_Handle *PID_Handler_d,float Target_d,
																	 PID_Handle *PID_Handler_q);
#endif

