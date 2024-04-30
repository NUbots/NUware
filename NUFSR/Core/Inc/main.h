/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum
{
  LED_ON = 0U,
  LED_OFF = !LED_ON
} LEDStatus;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define XTAL_32MHZ_IN_Pin GPIO_PIN_0
#define XTAL_32MHZ_IN_GPIO_Port GPIOF
#define XTAL_32MHZ_OUT_Pin GPIO_PIN_1
#define XTAL_32MHZ_OUT_GPIO_Port GPIOF
#define MPU_INT_Pin GPIO_PIN_8
#define MPU_INT_GPIO_Port GPIOD
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define MPU_NSS_Pin GPIO_PIN_6
#define MPU_NSS_GPIO_Port GPIOC
#define MPU_CLK_Pin GPIO_PIN_7
#define MPU_CLK_GPIO_Port GPIOC
#define MPU_MISO_Pin GPIO_PIN_8
#define MPU_MISO_GPIO_Port GPIOC
#define MPU_MOSI_Pin GPIO_PIN_9
#define MPU_MOSI_GPIO_Port GPIOC
#define DXL_DIR1_Pin GPIO_PIN_8
#define DXL_DIR1_GPIO_Port GPIOA
#define UART1_TX_Pin GPIO_PIN_9
#define UART1_TX_GPIO_Port GPIOA
#define UART1_RX_Pin GPIO_PIN_10
#define UART1_RX_GPIO_Port GPIOA
#define JTAG_TMS_Pin GPIO_PIN_13
#define JTAG_TMS_GPIO_Port GPIOA
#define JTAG_TCK_Pin GPIO_PIN_14
#define JTAG_TCK_GPIO_Port GPIOA
#define JTAG_TDI_Pin GPIO_PIN_15
#define JTAG_TDI_GPIO_Port GPIOA
#define JTAG_TDO_Pin GPIO_PIN_3
#define JTAG_TDO_GPIO_Port GPIOB
#define JTAG_RST_Pin GPIO_PIN_4
#define JTAG_RST_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
