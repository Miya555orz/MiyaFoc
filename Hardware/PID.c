#include "PID.h"
#include "Open_Loop.h"
#include "MT6701.h"
#include "Paremeter.h"
#include "adc.h"

float Ud_FF=0;
float Uq_FF=0;

void PID_Init(PID_Handle *PID_Handler,float Kp,float Ki,float Kd,
	            float Intemax,float Intemin,float Outmax,float Outmin)
{
  PID_Handler->Kp=Kp;
	PID_Handler->Ki=Ki;
	PID_Handler->Kd=Kd;
	PID_Handler->Integral_Max=Intemax;
	PID_Handler->Integral_Min=Intemin;
	PID_Handler->OutMax=Outmax;
	PID_Handler->OutMin=Outmin;
	PID_Handler->Current_Error = 0;
  PID_Handler->Last_Error = 0;
  PID_Handler->Error_Sum = 0;
  PID_Handler->Integral = 0;
  PID_Handler->Differential = 0;
  PID_Handler->OutPut = 0;
}

float PID_Calculate(PID_Handle *PID_Handler,float Current_Error)
{
	
  PID_Handler->Current_Error=Current_Error;
	PID_Handler->Error_Sum+=PID_Handler->Current_Error;
	
	if(PID_Handler->Error_Sum > PID_Handler->Integral_Max / PID_Handler->Ki)
        PID_Handler->Error_Sum = PID_Handler->Integral_Max / PID_Handler->Ki;
  if(PID_Handler->Error_Sum < PID_Handler->Integral_Min / PID_Handler->Ki)
        PID_Handler->Error_Sum = PID_Handler->Integral_Min / PID_Handler->Ki;
  PID_Handler->Integral=PID_Handler->Ki*PID_Handler->Error_Sum;
	
	PID_Handler->Differential=PID_Handler->Kd*(PID_Handler->Current_Error-PID_Handler->Last_Error);
	PID_Handler->Last_Error=PID_Handler->Current_Error;
	
	PID_Handler->OutPut=PID_Handler->Kp*PID_Handler->Current_Error+PID_Handler->Integral+PID_Handler->Differential;
	
	if(PID_Handler->OutPut>=PID_Handler->OutMax){PID_Handler->OutPut=PID_Handler->OutMax;}
	if(PID_Handler->OutPut<=PID_Handler->OutMin){PID_Handler->OutPut=PID_Handler->OutMin;}
	
	return PID_Handler->OutPut;
}

float Speed_Loop(PID_Handle *PID_Handler,float Target_Speed)
{
	float Error=Target_Speed-Filter_Speed;
	
  return PID_Calculate(PID_Handler,Error);
}

float Torque_q_Loop(PID_Handle *PID_Handler,float Target_q)
{
  float Error=Target_q-(Filter_I_q/MAX_CURRENT);
	Uq_FF=ELEC_SPEED*L_D*Filter_I_d;
	return PID_Calculate(PID_Handler,Error)+Uq_FF;
}

float Torque_d_Loop(PID_Handle *PID_Handler,float Target_d)
{
  float Error=Target_d-(Filter_I_d/MAX_CURRENT);
	Ud_FF=-ELEC_SPEED*L_Q*Filter_I_q;
	return PID_Calculate(PID_Handler,Error)+Ud_FF;
}

float Position_Loop(PID_Handle *PID_Handler,float Target_Position)
{
  float Error=Target_Position-Motor_Angle;
  return PID_Calculate(PID_Handler,Error);
}

void Speed_Contorl(PID_Handle *PID_Handler,float Target_Speed)
{
	float d_u=0;
	float d_v=0;
	float d_w=0;
  float d=0;
	float q=Speed_Loop(PID_Handler,Target_Speed);
	SVPWM_Set_Duty(d,q,Elec_Angle,&d_u, &d_v, &d_w);
  PWM_Set_Compare(d_u,d_v,d_w);
}

void Position_Control(PID_Handle *PID_Handler,float Target_Position)
{
  float d_u=0;
	float d_v=0;
	float d_w=0;
  float d=0;
	float q=Position_Loop(PID_Handler,Target_Position);
	SVPWM_Set_Duty(d,q,Elec_Angle,&d_u, &d_v, &d_w);
  PWM_Set_Compare(d_u,d_v,d_w);
}

void Torque_Control(PID_Handle *PID_Handler_q,float Target_q,
	                  PID_Handle *PID_Handler_d,float Target_d)
{
  float d_u=0;
	float d_v=0;
	float d_w=0;
  float d=Torque_d_Loop(PID_Handler_d,Target_d);
	float q=Torque_q_Loop(PID_Handler_q,Target_q);
	SVPWM_Set_Duty(d,q,Elec_Angle,&d_u, &d_v, &d_w);
  PWM_Set_Compare(d_u,d_v,d_w);
}
float target;
void Toruqe_Position_Speed_Control(PID_Handle *PID_Handler_Position,float Target_Position,
	                                 PID_Handle *PID_Handler_Speed,
																	 PID_Handle *PID_Handler_d,float Target_d,
																	 PID_Handle *PID_Handler_q)
{
  float d_u=0;
	float d_v=0;
	float d_w=0;
	float Position_Output=Position_Loop(PID_Handler_Position,Target_Position);
	float Speed_Output=Speed_Loop(PID_Handler_Speed,Position_Output);
	float d=Torque_d_Loop(PID_Handler_d,Target_d);
	float q=Torque_q_Loop(PID_Handler_q,Speed_Output);
	SVPWM_Set_Duty(d,q,Elec_Angle,&d_u, &d_v, &d_w);
  PWM_Set_Compare(d_u,d_v,d_w);
}