#include "PID.h"
#include "Open_Loop.h"
#include "MT6701.h"
#include "Paremeter.h"
#include "adc.h"
#include "stm32f1xx_hal.h"
#include <math.h>

float Ud_FF=0;
float Uq_FF=0;

#define FOC_POSITION_PLAN_PERIOD_MS     1U
#define FOC_POSITION_PLAN_MAX_STEP_MS   20U
#define FOC_POSITION_MAX_SPEED_RAD_S    10.0f
#define FOC_POSITION_MAX_ACCEL_RAD_S2   60.0f
#define FOC_POSITION_ARRIVE_RAD         Deg2Rad(0.8f)
#define FOC_POSITION_TARGET_EPS_RAD     Deg2Rad(0.05f)
#define FOC_POSITION_CURRENT_KP         0.28f
#define FOC_POSITION_SPEED_KD           0.025f
#define FOC_POSITION_HOLD_ERR_RAD       Deg2Rad(0.35f)
#define FOC_POSITION_HOLD_SPEED_RAD_S   0.12f
#define FOC_POSITION_CURRENT_LIMIT      0.35f

static float Position_Profile_Rad;
static float Position_Profile_Speed_Rad_S;
static float Position_Last_Target_Rad;
static uint8_t Position_Target_Changed;
static uint32_t Position_Last_Update_Ms;
static uint8_t Position_Profile_Ready;
extern float Target_d;

static float clamp_abs(float value, float limit)
{
	if(value > limit) {
		return limit;
	}
	if(value < -limit) {
		return -limit;
	}
	return value;
}

static float abs_f(float value)
{
	return (value < 0.0f) ? -value : value;
}

static float sign_f(float value)
{
	if(value > 0.0f) {
		return 1.0f;
	}
	if(value < 0.0f) {
		return -1.0f;
	}
	return 0.0f;
}

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

void PID_Reset(PID_Handle *PID_Handler)
{
	if(PID_Handler == 0) {
		return;
	}
	PID_Handler->Current_Error = 0.0f;
	PID_Handler->Last_Error = 0.0f;
	PID_Handler->Error_Sum = 0.0f;
	PID_Handler->Integral = 0.0f;
	PID_Handler->Differential = 0.0f;
	PID_Handler->OutPut = 0.0f;
}

float PID_Calculate(PID_Handle *PID_Handler,float Current_Error)
{
	
  PID_Handler->Current_Error=Current_Error;
	if(PID_Handler->Ki != 0.0f)
	{
		PID_Handler->Error_Sum+=PID_Handler->Current_Error;
		
		if(PID_Handler->Error_Sum > PID_Handler->Integral_Max / PID_Handler->Ki)
					PID_Handler->Error_Sum = PID_Handler->Integral_Max / PID_Handler->Ki;
		if(PID_Handler->Error_Sum < PID_Handler->Integral_Min / PID_Handler->Ki)
					PID_Handler->Error_Sum = PID_Handler->Integral_Min / PID_Handler->Ki;
		PID_Handler->Integral=PID_Handler->Ki*PID_Handler->Error_Sum;
	}
	else
	{
		PID_Handler->Error_Sum = 0.0f;
		PID_Handler->Integral = 0.0f;
	}
	
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
  float target_rad=Deg2Rad(Target_Position);
	float current_rad=Motor_Position_Rad;
	uint32_t now=HAL_GetTick();
	float Error;

	if(Position_Profile_Ready == 0U)
	{
		Position_Profile_Ready = 1U;
		Position_Profile_Rad = current_rad;
		Position_Profile_Speed_Rad_S = 0.0f;
		Position_Last_Target_Rad = target_rad;
		Position_Last_Update_Ms = now;
		PID_Reset(PID_Handler);
	}

	if(abs_f(target_rad - Position_Last_Target_Rad) > FOC_POSITION_TARGET_EPS_RAD)
	{
		Position_Last_Target_Rad = target_rad;
		Position_Target_Changed = 1U;
	}

	if((now - Position_Last_Update_Ms) >= FOC_POSITION_PLAN_PERIOD_MS)
	{
		uint32_t elapsed_ms = now - Position_Last_Update_Ms;
		float dt_s;
		float distance;
		float stop_distance;
		float desired_speed;
		float speed_delta;
		float next_distance;

		if(elapsed_ms > FOC_POSITION_PLAN_MAX_STEP_MS) {
			elapsed_ms = FOC_POSITION_PLAN_MAX_STEP_MS;
		}
		Position_Last_Update_Ms = now;
		dt_s = (float)elapsed_ms * 0.001f;
		distance = Position_Last_Target_Rad - Position_Profile_Rad;
		stop_distance = (Position_Profile_Speed_Rad_S * Position_Profile_Speed_Rad_S) /
		                (2.0f * FOC_POSITION_MAX_ACCEL_RAD_S2);
		desired_speed = sign_f(distance) * FOC_POSITION_MAX_SPEED_RAD_S;
		if(abs_f(distance) <= stop_distance)
		{
			desired_speed = sign_f(distance) *
			                sqrtf(2.0f * FOC_POSITION_MAX_ACCEL_RAD_S2 * abs_f(distance));
		}

		speed_delta = desired_speed - Position_Profile_Speed_Rad_S;
		speed_delta = clamp_abs(speed_delta, FOC_POSITION_MAX_ACCEL_RAD_S2 * dt_s);
		Position_Profile_Speed_Rad_S += speed_delta;
		Position_Profile_Speed_Rad_S =
		    clamp_abs(Position_Profile_Speed_Rad_S, FOC_POSITION_MAX_SPEED_RAD_S);
		Position_Profile_Rad += Position_Profile_Speed_Rad_S * dt_s;

		next_distance = Position_Last_Target_Rad - Position_Profile_Rad;
		if(sign_f(distance) != 0.0f && sign_f(distance) != sign_f(next_distance))
		{
			Position_Profile_Rad = Position_Last_Target_Rad;
			Position_Profile_Speed_Rad_S = 0.0f;
		}
		if(abs_f(Position_Last_Target_Rad - Position_Profile_Rad) < FOC_POSITION_ARRIVE_RAD &&
		   abs_f(Position_Profile_Speed_Rad_S) < 0.05f)
		{
			Position_Profile_Rad = Position_Last_Target_Rad;
			Position_Profile_Speed_Rad_S = 0.0f;
		}
	}

  Error=Position_Profile_Rad-current_rad;
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
	float target_speed=Position_Loop(PID_Handler,Target_Position);
	float position_error=Position_Profile_Rad - Motor_Position_Rad;
	float speed_error=target_speed - Filter_Speed;
	float target_current;
  float d;
	float q;

	if(Position_Target_Changed != 0U)
	{
		Position_Target_Changed = 0U;
		PID_Reset(&PID_Speed);
		PID_Reset(&PID_Torque_d);
		PID_Reset(&PID_Torque_q);
	}

	target_current = FOC_POSITION_CURRENT_KP * position_error +
	                 FOC_POSITION_SPEED_KD * speed_error;

	if(abs_f(Position_Last_Target_Rad - Motor_Position_Rad) < FOC_POSITION_HOLD_ERR_RAD &&
	   abs_f(Filter_Speed) < FOC_POSITION_HOLD_SPEED_RAD_S &&
	   abs_f(Position_Profile_Speed_Rad_S) < FOC_POSITION_HOLD_SPEED_RAD_S)
	{
		target_current = 0.0f;
		PID_Reset(&PID_Torque_d);
		PID_Reset(&PID_Torque_q);
	}

	target_current = clamp_abs(target_current, FOC_POSITION_CURRENT_LIMIT);
	d=Torque_d_Loop(&PID_Torque_d,Target_d);
	q=Torque_q_Loop(&PID_Torque_q,target_current);
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
