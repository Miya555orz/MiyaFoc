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
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "foc_can.h"

uint8_t UART_RX_BUFFER[64]={0};
uint8_t Index=0;
CMD_TypeDef CMD;
static uint8_t uart_rx_byte;
static uint8_t uart_line_buffer[64];
static volatile uint8_t uart_line_ready;
static volatile uint16_t uart_line_length;
static volatile uint32_t uart_rx_error_count;
static volatile uint32_t uart_rx_overflow_count;
static UART2_LogMode_t uart2_log_mode = UART2_LOG_CSV;

int fputc(int c, FILE *stream)
{
  uint8_t ch[] = {(uint8_t)c};
  (void)UART2_SendBuffer(ch, 1U);
  return c;
}

static void UART2_ClearErrors(void)
{
  __HAL_UART_CLEAR_PEFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
}

static void UART2_RearmRx(void)
{
  if (huart2.RxState != HAL_UART_STATE_BUSY_RX)
  {
    (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
  }
}

static void UART2_NormalizeLine(char *line)
{
  char *src = line;
  char *dst = line;

  while (*src == ' ' || *src == '\t')
  {
    src++;
  }
  while (*src != '\0')
  {
    char ch = *src++;
    if (ch == '=')
    {
      ch = ':';
    }
    *dst++ = (char)toupper((unsigned char)ch);
  }
  *dst = '\0';
  while (dst > line &&
         (dst[-1] == ' ' || dst[-1] == '\t' || dst[-1] == '\r' || dst[-1] == '\n'))
  {
    *--dst = '\0';
  }
}

static void UART2_ApplyCommandLine(char *line)
{
  float target;
  unsigned long profile;
  unsigned long loop_mode;
  const FocCanStatus_t *can_status;

  UART2_NormalizeLine(line);
  if(sscanf(line,"SPEED:%f",&target)==1)
  {
    CMD.Target=target;
    CMD.CMD_Type=CMD_SPEED;
    FocCan_NotifySerialCommand();
  }
  else if(sscanf(line,"POSITION:%f",&target)==1)
  {
    CMD.Target=target;
    CMD.CMD_Type=CMD_POSITION;
    FocCan_NotifySerialCommand();
  }
  else if(sscanf(line,"TORQUE:%f",&target)==1)
  {
    CMD.Target=target;
    CMD.CMD_Type=CMD_TORQUE;
    FocCan_NotifySerialCommand();
  }
  else if(strcmp(line,"STOP")==0)
  {
    FocCan_Stop();
  }
  else if(sscanf(line,"CANPROFILE:%lu",&profile)==1 ||
          sscanf(line,"CAN PROFILE %lu",&profile)==1 ||
          sscanf(line,"CAN:%lu",&profile)==1)
  {
    if (FocCan_SetTimingProfile((uint8_t)profile) == HAL_OK)
    {
      printf("can_profile=%lu ok\r\n", profile);
    }
    else
    {
      printf("can_profile=%lu fail\r\n", profile);
    }
  }
  else if(strcmp(line,"CANSTAT")==0 || strcmp(line,"CAN STAT")==0)
  {
    can_status = FocCan_GetStatus();
    printf("can profile=%u loop=%u tx=%lu fail=%lu abort=%lu free=%lu err=0x%08lX txid=0x%03lX rx=%lu rxerr=%lu rxid=0x%03lX lb=%lu btr=0x%08lX esr=0x%08lX tsr=0x%08lX\r\n",
           (unsigned)can_status->timing_profile,
           (unsigned)can_status->loopback_mode,
           (unsigned long)can_status->tx_count,
           (unsigned long)can_status->tx_error_count,
           (unsigned long)can_status->tx_abort_count,
           (unsigned long)can_status->last_tx_free_level,
           (unsigned long)can_status->last_hal_error,
           (unsigned long)can_status->last_tx_id,
           (unsigned long)can_status->rx_count,
           (unsigned long)can_status->rx_error_count,
           (unsigned long)can_status->last_rx_id,
           (unsigned long)can_status->loopback_count,
           (unsigned long)CAN1->BTR,
           (unsigned long)CAN1->ESR,
           (unsigned long)CAN1->TSR);
  }
  else if(strcmp(line,"CANLOOP")==0 || strcmp(line,"CAN LOOP")==0)
  {
    if (FocCan_SetLoopback(1U) == HAL_OK)
    {
      printf("can_loop ok btr=0x%08lX\r\n", (unsigned long)CAN1->BTR);
    }
    else
    {
      printf("can_loop fail btr=0x%08lX\r\n", (unsigned long)CAN1->BTR);
    }
  }
  else if(sscanf(line,"CANLOOP:%lu",&loop_mode)==1 ||
          sscanf(line,"LOOP:%lu",&loop_mode)==1)
  {
    if (FocCan_SetLoopback((loop_mode != 0UL) ? 1U : 0U) == HAL_OK)
    {
      printf("can_loop=%lu ok btr=0x%08lX\r\n", loop_mode, (unsigned long)CAN1->BTR);
    }
    else
    {
      printf("can_loop=%lu fail btr=0x%08lX\r\n", loop_mode, (unsigned long)CAN1->BTR);
    }
  }
  else if(strcmp(line,"CANNORMAL")==0 ||
          strcmp(line,"CAN NORMAL")==0 ||
          strcmp(line,"NORMAL")==0 ||
          strcmp(line,"CANMODE:NORMAL")==0)
  {
    if (FocCan_SetLoopback(0U) == HAL_OK)
    {
      printf("can_normal ok btr=0x%08lX\r\n", (unsigned long)CAN1->BTR);
    }
    else
    {
      printf("can_normal fail btr=0x%08lX\r\n", (unsigned long)CAN1->BTR);
    }
  }
  else if(strcmp(line,"CLKSTAT")==0 || strcmp(line,"CLK STAT")==0)
  {
    printf("clk sys=%lu hclk=%lu pclk1=%lu pclk2=%lu SystemCoreClock=%lu cfgr=0x%08lX\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(),
           (unsigned long)HAL_RCC_GetHCLKFreq(),
           (unsigned long)HAL_RCC_GetPCLK1Freq(),
           (unsigned long)HAL_RCC_GetPCLK2Freq(),
           (unsigned long)SystemCoreClock,
           (unsigned long)RCC->CFGR);
  }
  else if(strcmp(line,"CANREG")==0 || strcmp(line,"CAN REG")==0)
  {
    printf("can mcr=0x%08lX msr=0x%08lX tsr=0x%08lX rf0r=0x%08lX ier=0x%08lX esr=0x%08lX btr=0x%08lX\r\n",
           (unsigned long)CAN1->MCR,
           (unsigned long)CAN1->MSR,
           (unsigned long)CAN1->TSR,
           (unsigned long)CAN1->RF0R,
           (unsigned long)CAN1->IER,
           (unsigned long)CAN1->ESR,
           (unsigned long)CAN1->BTR);
  }
  else if(strcmp(line,"LOG:TEXT")==0)
  {
    uart2_log_mode = UART2_LOG_TEXT;
  }
  else if(strcmp(line,"LOG:CSV")==0)
  {
    uart2_log_mode = UART2_LOG_CSV;
  }
  else
  {
    FocCan_Stop();
  }
}

void UART2_StartRx(void)
{
  __disable_irq();
  Index = 0U;
  memset(UART_RX_BUFFER, 0, sizeof(UART_RX_BUFFER));
  uart_line_ready = 0U;
  uart_line_length = 0U;
  __enable_irq();

  UART2_ClearErrors();
  UART2_RearmRx();
}

void UART2_PollRecovery(void)
{
  if (huart2.ErrorCode != HAL_UART_ERROR_NONE ||
      huart2.RxState != HAL_UART_STATE_BUSY_RX)
  {
    UART2_ClearErrors();
    UART2_RearmRx();
  }
}

void UART2_ProcessPendingCommand(void)
{
  char line[64];
  uint16_t length;

  __disable_irq();
  if (uart_line_ready == 0U)
  {
    __enable_irq();
    return;
  }
  length = uart_line_length;
  if (length >= sizeof(line))
  {
    length = sizeof(line) - 1U;
  }
  memcpy(line, uart_line_buffer, length);
  line[length] = '\0';
  uart_line_ready = 0U;
  __enable_irq();

  UART2_ApplyCommandLine(line);
}

UART2_LogMode_t UART2_GetLogMode(void)
{
  return uart2_log_mode;
}

HAL_StatusTypeDef UART2_SendBuffer(const uint8_t *data, uint16_t length)
{
  uint32_t timeout_ms;

  if (data == 0 || length == 0U)
  {
    return HAL_OK;
  }

  timeout_ms = 4U + ((((uint32_t)length * 10U * 1000U) + 115199U) / 115200U);
  if (timeout_ms < 8U)
  {
    timeout_ms = 8U;
  }
  if (timeout_ms > 50U)
  {
    timeout_ms = 50U;
  }

  return HAL_UART_Transmit(&huart2, (uint8_t *)data, length, timeout_ms);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART2)
	{ 				 
     if(uart_rx_byte=='\n')
		 {
			 UART_RX_BUFFER[Index]='\0';
       if (uart_line_ready == 0U)
       {
         uart_line_length = Index;
         memcpy(uart_line_buffer, UART_RX_BUFFER, (uint16_t)(Index + 1U));
         uart_line_ready = 1U;
       }
			 Index=0;
			 memset(UART_RX_BUFFER,0,sizeof(UART_RX_BUFFER));
		 }	
		 else if(uart_rx_byte=='\r')
		 {
		   
		 }
		 else 
		 {
       UART_RX_BUFFER[Index]=uart_rx_byte;
			 Index++;
			 if(Index>=sizeof(UART_RX_BUFFER)-1)
		   {
		     Index=0;
				 memset(UART_RX_BUFFER,0,sizeof(UART_RX_BUFFER));
         uart_rx_overflow_count++;
		   }
		 }
		 UART2_RearmRx();		 
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART2)
  {
    uart_rx_error_count++;
    UART2_ClearErrors();
    UART2_RearmRx();
  }
}

/* USER CODE END 1 */
