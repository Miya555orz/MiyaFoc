#include "Open_Loop.h"
#include "stdint.h"
#include <arm_math.h>
#include "MT6701.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
//数学库中的三角函数所传入的参数必须是弧度制，否则计算错误
//该函数手动构造一个旋转电压矢量，在该场景下强行让α=Intens*cosθ，β=Intens*sinθ,FOC正常情况下不会满足简单的正弦关系，该情况属于特例
void OpenLoop_Set_PWM_Duty(float Theta,float Intens,float *duty_u,float *duty_v,float *duty_w)
{   //提前转为弧度制防止后续计算时因计算规则出现计算错误
   uint8_t Sectors[6][3]={{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
	 uint8_t Sector=1+Theta/Deg2Rad(60);
		
	 float Tm=Intens*sinf(Sector*Rad60-Theta);
	 float Tn=Intens*sinf(Theta-(Sector*Rad60-Rad60));
	 float T0=1-Tm-Tn;
		
	 *duty_u=Tm*Sectors[Sector-1][0]+Tn*Sectors[Sector%6][0]+T0/2;
	 *duty_v=Tm*Sectors[Sector-1][1]+Tn*Sectors[Sector%6][1]+T0/2;
	 *duty_w=Tm*Sectors[Sector-1][2]+Tn*Sectors[Sector%6][2]+T0/2;
}
//SVPWM:通过α和β来计算出各相占空比
void SVPWM_Set_Duty(float d,float q,float Theta,float *duty_u,float *duty_v,float *duty_w)
{
	 uint8_t Sectors[6][3]={{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
	 uint8_t Map_to_Sector[8]={4,6,5,5,3,1,2,2}; 
	 d=max(d,-1);
	 d=min(d,1);
	 q=max(q,-1);
	 q=min(q,1);
   float Alpha=0;
	 float Beta=0;
	 float Sin_Theta=arm_sin_f32(Theta);
	 float Cos_Theta=arm_cos_f32(Theta);
	 arm_inv_park_f32(d,q,&Alpha,&Beta,Sin_Theta,Cos_Theta);
	 bool A=Beta>0;
	 bool B=fabs(Beta)>SQRT3*fabs(Alpha);
	 bool C=Alpha>0;
	 
	 uint8_t Map_Val=4*A+2*B+C;
	 uint8_t Sector=Map_to_Sector[Map_Val];
	 
	 float Tm=arm_sin_f32(Sector*Rad60)*Alpha-arm_cos_f32(Sector*Rad60)*Beta;
	 float Tn=Beta*arm_cos_f32(Sector*Rad60-Rad60)-Alpha*arm_sin_f32(Sector*Rad60-Rad60);
	 float T0=1-Tm-Tn;
	 
	 *duty_u=Tm*Sectors[Sector-1][0]+Tn*Sectors[Sector%6][0]+T0/2;
	 *duty_v=Tm*Sectors[Sector-1][1]+Tn*Sectors[Sector%6][1]+T0/2;
	 *duty_w=Tm*Sectors[Sector-1][2]+Tn*Sectors[Sector%6][2]+T0/2;
}

void PWM_Set_Compare(float d_u,float d_v,float d_w)
{
  d_u=min(d_u,0.9);
	d_v=min(d_v,0.9);
	d_w=min(d_w,0.9);
	
	__disable_irq();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * 3600);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * 3600);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * 3600);
	__enable_irq();
}

