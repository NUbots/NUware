/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "settings.h"
#include "rs485.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

#ifdef FIRST_BUZZ
  //Confirm that the programme is running.
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
#endif

#ifdef DXL_PWR
  // Set the Dynamixel power on.
  HAL_GPIO_WritePin(DXL_PWR_EN_GPIO_Port, DXL_PWR_EN_Pin, GPIO_PIN_SET);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#ifdef TEST_UART
  char test_uart_char;
  UART_HandleTypeDef* p_test_huart = &huart1;
  uint16_t test_uart_it_rx_mask = UART1_RX;
  uint16_t test_uart_it_tx_mask = UART1_TX;
#if TEST_UART == 2
  p_test_huart = &huart2; test_uart_it_rx_mask = UART2_RX; test_uart_it_rx_mask = UART2_TX;
#endif
#if TEST_UART == 3
  p_test_huart = &huart3; test_uart_it_rx_mask = UART3_RX; test_uart_it_rx_mask = UART3_TX;
#endif
#if TEST_UART == 4
  p_test_huart = &huart4; test_uart_it_rx_mask = UART4_RX; test_uart_it_rx_mask = UART4_TX;
#endif
#if TEST_UART == 5
  p_test_huart = &huart5; test_uart_it_rx_mask = UART5_RX; test_uart_it_rx_mask = UART5_TX;
#endif
#if TEST_UART == 6
  p_test_huart = &huart6; test_uart_it_rx_mask = UART6_RX; test_uart_it_rx_mask = UART6_TX;
#endif
  // Wait for the first packet.
  RS485_Receive_IT(p_test_huart, (uint8_t*)&test_uart_char, 1);
  //RS485_Receive(p_test_huart, (uint8_t*)&test_uart_char, 1, HAL_MAX_DELAY);
#endif

#ifdef TEST_USB
  char test_usb_str_buffer[] = "NUsense = nuisance!";
#endif

  while (1)
  {
    /* USER CODE END WHILE */
#ifdef TEST_UART
	// Echo whatever is received on the test UART.
	// If a packet has been received, then receive the next one.
	if (uart_it_flags & test_uart_it_rx_mask) {
		RS485_Transmit_IT(p_test_huart, (uint8_t*)&test_uart_char, 1);
		uart_it_flags &= ~test_uart_it_rx_mask;
	}
	//RS485_Transmit(p_test_huart, (uint8_t*)&test_uart_char, 1, HAL_MAX_DELAY);
	// If a packet has been sent, then send the next one.
	if (uart_it_flags & test_uart_it_tx_mask) {
		RS485_Receive_IT(p_test_huart, (uint8_t*)&test_uart_char, 1);
		uart_it_flags &= ~test_uart_it_tx_mask;
	}
	//RS485_Receive(p_test_huart, (uint8_t*)&test_uart_char, 1, HAL_MAX_DELAY);
#endif

#ifdef TEST_USB
	CDC_Transmit_HS((uint8_t*)test_usb_str_buffer, strlen(test_usb_str_buffer));
#endif

#ifdef TEST_IMU

#endif

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
