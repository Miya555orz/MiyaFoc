/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

typedef enum
{
  CMD_NONE=0,
	CMD_SPEED,
	CMD_POSITION,
	CMD_TORQUE,
	CMD_POSITION_SPEED,
	CMD_POSITION_TORQUE,
	CMD_SPEED_TORQUE,
	CMD_POSITION_SPEED_TORQUE
}CMD_Enum;

typedef struct
{
  CMD_Enum CMD_Type;
	float Target;
}CMD_TypeDef;

typedef enum
{
  UART2_LOG_CSV = 0,
  UART2_LOG_TEXT
}UART2_LogMode_t;

extern CMD_TypeDef CMD;
extern uint8_t UART_RX_BUFFER[64];
extern uint8_t Index;

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART2_StartRx(void);
void UART2_PollRecovery(void);
void UART2_ProcessPendingCommand(void);
UART2_LogMode_t UART2_GetLogMode(void);
HAL_StatusTypeDef UART2_SendBuffer(const uint8_t *data, uint16_t length);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

