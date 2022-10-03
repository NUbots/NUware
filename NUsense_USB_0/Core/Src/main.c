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
uint8_t SPI_flag, INT_flag;

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
	SPI_flag = 0; INT_flag = 0;

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

  NUfsr_IMU_Init();

  //Confirm that the programme is running.
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);

  //Declare some variables.
  struct IMURawData raw_data;
  uint8_t read_buffer[16] = {
	  WHO_AM_I 		| IMU_READ,
	  ACCEL_XOUT_H 	| IMU_READ,
	  ACCEL_XOUT_L 	| IMU_READ,
	  ACCEL_YOUT_H 	| IMU_READ,
	  ACCEL_YOUT_L 	| IMU_READ,
	  ACCEL_ZOUT_H 	| IMU_READ,
	  ACCEL_ZOUT_L 	| IMU_READ,
	  TEMP_OUT_H 	| IMU_READ,
	  TEMP_OUT_L 	| IMU_READ,
	  GYRO_XOUT_H 	| IMU_READ,
	  GYRO_XOUT_L 	| IMU_READ,
	  GYRO_YOUT_H 	| IMU_READ,
	  GYRO_YOUT_L 	| IMU_READ,
	  GYRO_ZOUT_H 	| IMU_READ,
	  GYRO_ZOUT_L 	| IMU_READ,
	  0x00
  };
  uint8_t reset_buffer[2] = {INT_STATUS | IMU_READ, 0x00};
  char str_buffer[64];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Wait for five seconds as a buffer.
  HAL_Delay(5000);

  //Send the first packet.
  NUfsr_IMU_TransmitReceive_IT(read_buffer, (uint8_t*)&raw_data, 16);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //If a packet has been received, then parse it.
	  if (SPI_flag == 1) {
		  sprintf(str_buffer,
				  "%x\t"
				  "%d\t%d\t%d\t"
				  "%d\t%d\t%d\t"
				  "%d\t"
				  "%d\r\n",
				  raw_data.ID,
				  raw_data.accelerometer.x,raw_data.accelerometer.y,raw_data.accelerometer.z,
				  raw_data.gyroscope.x,raw_data.gyroscope.y,raw_data.gyroscope.z,
				  raw_data.temperature,
				  INT_flag);
		  CDC_Transmit_HS((uint8_t*)str_buffer, strlen(str_buffer));
		  NUfsr_IMU_TransmitReceive_IT(read_buffer, (uint8_t*)&raw_data, 16);

		  NUfsr_IMU_BlockingTransmit(reset_buffer, 2);

		  //Reset the flags.
		  SPI_flag = 0;
		  INT_flag = 0;
	  }

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10)
		INT_flag = 1;
}

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
