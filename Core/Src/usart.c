/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <stdio.h>
#include <string.h>
#include "foc_can.h"

uint8_t UART_RX_BUFFER[64]={0};
uint8_t Index=0;
CMD_TypeDef CMD;

int fputc(int c, FILE *stream)
{
  uint8_t ch[] = {(uint8_t)c};
  HAL_UART_Transmit(&huart2, ch, 1, HAL_MAX_DELAY);
  return c;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART2)
	{ 				 
     if(UART_RX_BUFFER[Index]=='\n')
		 {
			 float target;

			 UART_RX_BUFFER[Index]='\0';
			 if(sscanf((char *)UART_RX_BUFFER,"SPEED:%f",&target)==1)
			 {
				 CMD.Target=target;
				 CMD.CMD_Type=CMD_SPEED;
				 FocCan_NotifySerialCommand();
			 }
			 else if(sscanf((char *)UART_RX_BUFFER,"POSITION:%f",&target)==1)
			 {
				 CMD.Target=target;
			   CMD.CMD_Type=CMD_POSITION;
				 FocCan_NotifySerialCommand();
			 }
			 else if(sscanf((char *)UART_RX_BUFFER,"TORQUE:%f",&target)==1)
			 {
				 CMD.Target=target;
			   CMD.CMD_Type=CMD_TORQUE;
				 FocCan_NotifySerialCommand();
			 }
			 else if(strcmp((char *)UART_RX_BUFFER,"STOP")==0)
			 {
			   FocCan_Stop();
			 }
			 else
			 {
			   FocCan_Stop();
			 }
			 Index=0;
			 memset(UART_RX_BUFFER,0,sizeof(UART_RX_BUFFER));
		 }	
		 else if(UART_RX_BUFFER[Index]=='\r')
		 {
		   
		 }
		 else 
		 {
			 Index++;
			 if(Index>=sizeof(UART_RX_BUFFER)-1)
		   {
		     Index=0;
				 memset(UART_RX_BUFFER,0,sizeof(UART_RX_BUFFER));
		   }
		 }
		 HAL_UART_Receive_IT(&huart2,&UART_RX_BUFFER[Index],1);		 
	}
}

/* USER CODE END 1 */
