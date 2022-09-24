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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "imu.h"

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
  /* USER CODE BEGIN 2 */
  //Confirm that the programme is running.
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);

  //Declare some variables.
  uint16_t Rx = 0x0000;
  uint16_t Rx_X = 0x0000;
  uint16_t Rx_Y = 0x0000;
  uint16_t Rx_Z = 0x0000;
  uint16_t Rx_T = 0x0000;
  uint16_t* Ptr_Rx = &Rx;
  uint16_t* Ptr_Rx_X = &Rx_X;
  uint16_t* Ptr_Rx_Y = &Rx_Y;
  uint16_t* Ptr_Rx_Z = &Rx_Z;
  uint16_t* Ptr_Rx_T = &Rx_T;
  char buffer[64];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Wait for five seconds as a buffer.
  HAL_Delay(5000);

  //Read the self-test registers.
  HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);
  NUfsr_IMU_TransmitReceive(IMU_SELF_TEST_X_ACCEL 	| IMU_READ, 0x00, Ptr_Rx_X, 1);
  NUfsr_IMU_TransmitReceive(IMU_SELF_TEST_Y_ACCEL 	| IMU_READ, 0x00, Ptr_Rx_Y, 1);
  NUfsr_IMU_TransmitReceive(IMU_SELF_TEST_Z_ACCEL 	| IMU_READ, 0x00, Ptr_Rx_Z, 1);
  HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET);

  //Print the values of the self-test.
  sprintf(buffer, "ST:\t%d\t%d\t%d\r\n", *Ptr_Rx_X, *Ptr_Rx_Y, *Ptr_Rx_Z);
  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Ask who it is, read all three accelerometers, the temperature, and ask who it is again.
	  HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);

	  NUfsr_IMU_TransmitReceive(WHO_AM_I 		| IMU_READ, 0x00, Ptr_Rx, 1);
	  sprintf(buffer, "IMU:\t%x", *Ptr_Rx);
	  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

	  NUfsr_IMU_TransmitReceive(ACCEL_XOUT_H 	| IMU_READ, 0x00, Ptr_Rx, 2);
	  sprintf(buffer, "\t%d", *Ptr_Rx);
	  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

	  NUfsr_IMU_TransmitReceive(ACCEL_YOUT_H 	| IMU_READ, 0x00, Ptr_Rx, 2);
	  sprintf(buffer, "\t%d", *Ptr_Rx);
	  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

	  NUfsr_IMU_TransmitReceive(ACCEL_ZOUT_H 	| IMU_READ, 0x00, Ptr_Rx, 2);
	  sprintf(buffer, "\t%d", *Ptr_Rx);
	  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

	  NUfsr_IMU_TransmitReceive(TEMP_OUT_H 		| IMU_READ, 0x00, Ptr_Rx, 2);
	  sprintf(buffer, "\t%d", *Ptr_Rx);
	  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

	  NUfsr_IMU_TransmitReceive(WHO_AM_I 		| IMU_READ, 0x00, Ptr_Rx, 1);
	  sprintf(buffer, "\t%x\r\n", *Ptr_Rx);
	  CDC_Transmit_HS((uint8_t*)buffer, strlen(buffer));

	  HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET);

	  //Wait for one second and repeat.
	  HAL_Delay(1000);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
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
