#ifndef __MT6701_H_
#define __MT6701_H_

#include "spi.h"
#include "gpio.h"
#include "arm_math.h"
#include "tim.h"

#define Deg2Rad(deg) ((deg)*PI/180)
#define Rad2Deg(rad) ((rad)*180/PI)
#define CS_Enable HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port,MT6701_CSN_Pin,GPIO_PIN_RESET)
#define CS_Disable HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port,MT6701_CSN_Pin,GPIO_PIN_SET)

float Diff_Indentify(float Diff);


#endif
