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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "settings.h"
#include "usbd_cdc_if.h"
#include "rs485_c.h"
#include "RS485.h"
#include "imu.h"
#include "Port.h"

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
uint8_t calculate_checksum(uint8_t*, uint8_t);
uint16_t update_crc(uint16_t, uint8_t*, uint16_t);

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

  /* NOTE: make sure that MX_DMA_Init is called before everything else. CubeMX
   * has a strange habit of doing things in the wrong order. This is a known
   * bug, that has not been updated strangely enough in version 6.6.0. It
   * should not matter too much anyway; CubeMX should not change the order here
   * given that it also has another strange habit of ignoring main.cpp and
   * creating and overwriting main.c instead despite CubeIDE knowing that this
   * is a C++ project! Oh CubeMX ...
   */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
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

  HAL_GPIO_WritePin(SPARE1_GPIO_Port, SPARE1_Pin, GPIO_PIN_RESET);

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

#ifdef TEST_UART
  /* This example in interrupt-mode is not the most efficient one. It freezes
   * when it is bombarded with characters with no other delay in the main loop.
   * This bug needs further investigation.
   * Update: I think that this has been fixed now that DMA is working although
   * it is safer to keep an eye out for it.
   */
  char str_buffer[] = "NUsense = nuisance!\r\n";
  RS485 link = RS485(TEST_UART);
  char c = 'x';
  // Wait for the first packet.
  link.receive((uint8_t*)&c, 1);
#endif

#ifdef TEST_USB
  char str_buffer[] = "NUsense = nuisance!\r\n";
#endif

#ifdef TEST_IMU
  int16_t acc;
  uint16_t count;
  uint8_t flags;
  uint8_t rx[14];
  struct NU_IMU_raw_data raw_data;
  struct NU_IMU_converted_data converted_data;
  char str[256];

  NU_IMU_Init();
#endif

#ifdef TEST_PORT
  char str_buffer[64];
  int16_t byte = 0xAA;
  Port port(1);
  port.begin_rx();
#endif

#ifdef TEST_MOTOR
#if TEST_MOTOR == 1
  RS485 rs_link(1);
  uint8_t inst_packet[] = {
		  0xFF, 0xFF, 	// header
		  0x01,			// ID
		  0x04,			// packet's length
		  0x03,			// instruction
		  0x19,			// address of data
		  0x01,			// data to be written
		  0xFF			// checksum
  };
  //inst_packet[7] = calculate_checksum(&inst_packet[2], 5);
  uint8_t sts_packet[6+3];
  rs_link.transmit(inst_packet, 8);
#else
  /* Please mind that a lot of this code is very bare-bones. We will need a
   * proper protocol-handler later.
   */
  RS485 rs_link(1);
  uint16_t crc_value;
  uint8_t sts_packet[11+3];
  // Write to the indirect address first to map to the LED.
  uint8_t inst_packet[14] = {
		  0xFF, 0xFF, 0xFD,	// header
		  0x00,				// reserved
		  0x01,				// ID
		  0x07, 0x00,		// packet's length
		  0x03,				// instruction
		  0xA8, 0x00,		// address of data
		  0x41,	0x00,		// data to be written
		  0xFF, 0xFF		// CRC
  };
  crc_value = update_crc(0, inst_packet, 5+7);
  inst_packet[12] = (uint8_t)(crc_value & 0x00FF);
  inst_packet[13] = (uint8_t)((crc_value & 0xFF00) >> 8);
  /* Delay for a bit to give the motor time to boot up. Without this delay, I
   * found that the motor does not respond at all. From some basic testing I
   * think that it is because the motor only boots up until the DXL power is
   * switched on from the DXL_POWER_EN pin. However, it may have something to
   * do with the RS485 transcievers instead.
   */
  HAL_Delay(1000);
  rs_link.transmit(inst_packet, 14);
  while (!rs_link.get_transmit_flag());
  // Wait for the status-packet so that it can be read in the debugger.
  rs_link.receive(sts_packet, 11);
  while (!rs_link.get_receive_flag());
  // Write to the LED.
  inst_packet[5] = 0x06;
  inst_packet[8] = 0xE0;
  inst_packet[10] = 0x01;
  crc_value = update_crc(0, inst_packet, 5+6);
  inst_packet[11] = (uint8_t)(crc_value & 0x00FF);
  inst_packet[12] = (uint8_t)((crc_value & 0xFF00) >> 8);
  rs_link.transmit(inst_packet, 13);
#endif
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef TEST_UART
	// Echo whatever is received on the test UART.
	// If a packet has been received, then send back the character and receive
	// the next one.
	if (link.get_receive_flag()) {
		link.transmit((uint8_t*)&c, 1);
		link.receive((uint8_t*)&c, 1);
	}
	if (link.get_transmit_flag())
		link.transmit((uint8_t*)&str_buffer, strlen(str_buffer));

#endif

#ifdef TEST_USB
	CDC_Transmit_HS((uint8_t*)str_buffer, strlen(str_buffer));
#endif

#ifdef TEST_IMU
	//NU_IMU_ReadSlowly(addresses, (uint8_t*)&raw_data, 16);
	//NU_IMU_ReadFifo(rx, 14);
	//NU_IMU_ReadFifo(rx, 4);
	//NU_IMU_ReadBurst(ACCEL_XOUT_H, rx, 14);

	NU_IMU_ReadBurst(ACCEL_XOUT_H, rx, 14);

	raw_data.accelerometer.x = ((uint16_t)rx[ 0] << 8) | rx[ 1];
	raw_data.accelerometer.y = ((uint16_t)rx[ 2] << 8) | rx[ 3];
	raw_data.accelerometer.z = ((uint16_t)rx[ 4] << 8) | rx[ 5];
	raw_data.temperature =		((uint16_t)rx[ 6] << 8) | rx[ 7];
	raw_data.gyroscope.x = 	((uint16_t)rx[ 8] << 8) | rx[ 9];
	raw_data.gyroscope.y = 	((uint16_t)rx[10] << 8) | rx[11];
	raw_data.gyroscope.z = 	((uint16_t)rx[12] << 8) | rx[13];

	NU_IMU_ConvertRawData(&raw_data, &converted_data);

	sprintf(str, "IMU:\t"
			"ACC (g):\t%.3f\t%.3f\t%.3f\t"
			"TEMP (deg C):\t%.3f\t"
			"GYR (dps):\t%.3f\t%.3f\t%.3f\t"
			"Raw:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
			converted_data.accelerometer.x,
			converted_data.accelerometer.y,
			converted_data.accelerometer.z,
			converted_data.temperature,
			converted_data.gyroscope.x,
			converted_data.gyroscope.y,
			converted_data.gyroscope.z,
			raw_data.accelerometer.x,
			raw_data.accelerometer.y,
			raw_data.accelerometer.z,
			raw_data.temperature,
			raw_data.gyroscope.x,
			raw_data.gyroscope.y,
			raw_data.gyroscope.z
			);

	CDC_Transmit_HS((uint8_t*)str, strlen(str));

	HAL_Delay(100);
#endif

#ifdef TEST_PORT
	byte = port.read();
	if (byte != NO_BYTE_READ) {
		sprintf(str_buffer, "NUsense = nuisance! %c\r\n", (uint8_t)byte);
		port.write((uint8_t*)str_buffer, strlen(str_buffer));
	}
	//test_port.write((uint8_t*)str_buffer, strlen(str_buffer));
	// Always check both interrupts at the end of the context in which one is
	// using this class.
	port.check_rx();
	port.check_tx();
	//HAL_Delay(1);
#endif

#ifdef TEST_MOTOR
#if TEST_MOTOR == 1
	// Version 1.0
	if (rs_link.get_transmit_flag())
		rs_link.receive(sts_packet, 6);

	if (rs_link.get_receive_flag()) {
		inst_packet[6] ^= 0x01;
		inst_packet[7] = calculate_checksum(&inst_packet[2], 5);
		HAL_Delay(500);
		rs_link.transmit(inst_packet, 8);
	}
#else
	// Version 2.0
	if (rs_link.get_transmit_flag())
		rs_link.receive(sts_packet, 11);

	if (rs_link.get_receive_flag()) {
		// Toggle the state of the LED.
		inst_packet[10] ^= 0x01;
		// Recalculate the CRC.
		crc_value = update_crc(0, inst_packet, 5+6);
	    inst_packet[11] = (uint8_t)(crc_value & 0x00FF);
	    inst_packet[12] = (uint8_t)((crc_value & 0xFF00) >> 8);
	    // Wait for a bit to set the blinking frequency of the LED.
		HAL_Delay(500);
		rs_link.transmit(inst_packet, 13);
	}
#endif
#endif
    /* USER CODE END WHILE */

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

uint8_t calculate_checksum(uint8_t* data, uint8_t length) {
	uint8_t checksum = 0;
	for (int i = 0; i < length; i++)
		checksum += data[i];
	return ~checksum;
}

uint16_t update_crc(uint16_t crc_accum, uint8_t* data_blk_ptr, uint16_t data_blk_size)
{
	uint16_t i, j;
	uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
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
