#ifndef __OPEN_LOOP_
#define __OPEN_LOOP_

#define SQRT3 1.7320508
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define Rad60 Deg2Rad(60)
void OpenLoop_Set_PWM_Duty(float Theta,float Intens,float *duty_u,float *duty_v,float *duty_w);
void SVPWM_Set_Duty(float d,float q,float Theta,float *duty_u,float *duty_v,float *duty_w);
void PWM_Set_Compare(float d_u,float d_v,float d_w);


#endif

