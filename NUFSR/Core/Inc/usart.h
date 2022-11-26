/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

// ISR for UART Tx and Rx operations
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// Variable transmission and receive functions Extension of HAL_UART_T/R_IT functions suited for NUfsr purposes
HAL_StatusTypeDef NUfsr_UART_Transmit(UART_HandleTypeDef *huart, void *pData, uint16_t Byte_Size);
HAL_StatusTypeDef NUfsr_UART_Receive(UART_HandleTypeDef *huart, void *pData, uint16_t Byte_Size);

// Used for polling
HAL_StatusTypeDef NUfsr_UART_Poll_Tx(UART_HandleTypeDef *huart, uint32_t Timeout);
HAL_StatusTypeDef NUfsr_UART_Poll_Rx(UART_HandleTypeDef *huart, uint32_t Timeout);

// Used for one-time status checks, can be used for manual polling
uint32_t NUfsr_UART_Tx_StatusComplete(UART_HandleTypeDef *huart);
uint32_t NUfsr_UART_Rx_StatusComplete(UART_HandleTypeDef *huart);

// Statuses and UART state enum's
typedef volatile enum
{
	Tx_FINISHED = 0U,
	Tx_NOT_FINISHED = !Tx_FINISHED,
}NUfsr_UART_Tx_ITController;

typedef volatile enum
{
	Rx_FINISHED = 0U,
	Rx_NOT_FINISHED = !Rx_FINISHED,
}NUfsr_UART_Rx_ITController;

typedef enum
{
	Rx = 0U,
	Tx = !Rx
}NUfsr_UART_DIR;

// UART Handler structure using UART state structs
typedef struct _NUfsr_UART_IT_StateHandler
{
	NUfsr_UART_Tx_ITController Tx_State;
	NUfsr_UART_Rx_ITController Rx_State;
}NUfsr_UART_IT_StateHandler;

extern NUfsr_UART_IT_StateHandler huart1_ITh;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

