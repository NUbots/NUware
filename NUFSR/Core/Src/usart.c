/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
NUfsr_UART_IT_StateHandler huart1_ITh;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
  huart1_ITh.Tx_State = Tx_FINISHED;
  huart1_ITh.Rx_State = Rx_FINISHED;
  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = DXL_P_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(DXL_P_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DXL_N_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(DXL_N_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, DXL_P_Pin|DXL_N_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ISR */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Tx Complete Signal */
	huart1_ITh.Tx_State = Tx_FINISHED;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Rx Complete Signal */
	huart1_ITh.Rx_State = Rx_FINISHED;
}

/* Tx and Rx operations */
HAL_StatusTypeDef NUfsr_UART_Transmit(UART_HandleTypeDef *huart, void *pData, uint16_t Byte_Size)
{
	// Error checking
	if((huart == NULL) || (pData == NULL) || (Byte_Size == 0U))
		return HAL_ERROR;

	HAL_StatusTypeDef state;

	// Wait for Rx poll
	if((state = NUfsr_UART_Poll_Rx(huart, HAL_MAX_DELAY)) != HAL_OK)
		return state;

	// Update Tx state
	huart1_ITh.Tx_State = Tx_NOT_FINISHED;

	// Set DXL_DIR
	HAL_GPIO_WritePin(GPIOA, DXL_DIR_Pin, Tx);

	// Begin transmission
	return HAL_UART_Transmit_IT(huart, (uint8_t*)pData, Byte_Size);

}

HAL_StatusTypeDef NUfsr_UART_Receive(UART_HandleTypeDef *huart, void *pData, uint16_t Byte_Size)
{
	// Error checking
	if((huart == NULL) || (pData == NULL) || (Byte_Size == 0U))
		return HAL_ERROR;

	HAL_StatusTypeDef state;

	// Wait for Rx poll
	if((state = NUfsr_UART_Poll_Tx(huart, HAL_MAX_DELAY)) != HAL_OK)
		return state;

	// Update Rx state
	huart1_ITh.Rx_State = Tx_NOT_FINISHED;

	// Set DXL_DIR
	HAL_GPIO_WritePin(GPIOA, DXL_DIR_Pin, Rx);

	// Begin transmission
	return HAL_UART_Receive_IT(huart, (uint8_t*)pData, Byte_Size);
}

/* Polling */
HAL_StatusTypeDef NUfsr_UART_Poll_Tx(UART_HandleTypeDef *huart, uint32_t Timeout)
{
	// Error checking
	if((huart == NULL) || (Timeout == 0U))
		return HAL_ERROR;

	uint32_t tickstart = HAL_GetTick();

	// Begin Polling
	while(!NUfsr_UART_Tx_StatusComplete(huart))
	{
		if(Timeout != HAL_MAX_DELAY)
		{
			if(HAL_GetTick() - tickstart > Timeout)
			{
				// Handle Error
				return HAL_ERROR;
			}
		}
	}

	return HAL_OK;
}

HAL_StatusTypeDef NUfsr_UART_Poll_Rx(UART_HandleTypeDef *huart, uint32_t Timeout)
{
	// Error checking
	if(huart == NULL)
		return HAL_ERROR;

	uint32_t tickstart = HAL_GetTick();

	// Begin Polling
	while(!NUfsr_UART_Rx_StatusComplete(huart))
	{
		if(Timeout != HAL_MAX_DELAY)
		{
			if(HAL_GetTick() - tickstart > Timeout)
			{
				// Handle Error
				return HAL_ERROR;
			}
		}
	}

	return HAL_OK;
}

/* Manual Polling */
bool NUfsr_UART_Tx_StatusComplete(UART_HandleTypeDef *huart)
{
	if(huart1_ITh.Tx_State == Tx_FINISHED)
		return true;

	return false;
}

bool NUfsr_UART_Rx_StatusComplete(UART_HandleTypeDef *huart)
{
	if(huart1_ITh.Rx_State == Rx_FINISHED)
		return true;

	return false;
}

/* USER CODE END 1 */
