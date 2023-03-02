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
#include "test_hw.hpp"
#include <array>
#include <sstream>
#include "dynamixel/Dynamixel.h"
#include "dynamixel/Packetiser.hpp"
#include "uart/Port.h"
#include "dynamixel/PacketHandler.hpp"
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

  /*
  uint8_t packet[] = {
  	  0xFF, 0xFF, 0xFD, 0x00,
	  0x01,
	  0x04, 0x00,
	  0x55,
	  0x00,
	  0xA1, 0x0C
  };
  uint16_t crc = 0x0000;

  crc = test_hw::update_crc(0x00, packet, sizeof(packet)-2);

  crc = 0x0000;

  for (int i = 0; i < sizeof(packet)-2; i++) {
	  crc = test_hw::update_crc(crc, packet[i]);
  }

  crc = 0x0000;
  */

#ifdef RUN_MAIN
	std::array<uart::Port,NUM_PORTS> ports = {{
			uart::Port(1),uart::Port(2),uart::Port(3),
			uart::Port(4),uart::Port(5),uart::Port(6)
	}};
	std::array<uint16_t,16> read_bank_addresses = {
		  0x00A8,
		  dynamixel::PRESENT_PWM,
		  	  dynamixel::PRESENT_PWM+1,
		  dynamixel::PRESENT_CURRENT,
			  dynamixel::PRESENT_CURRENT+1,
		  dynamixel::PRESENT_VELOCITY,
			  dynamixel::PRESENT_VELOCITY+1,
			  dynamixel::PRESENT_VELOCITY+2,
			  dynamixel::PRESENT_VELOCITY+3,
		  dynamixel::PRESENT_POSITION,
			  dynamixel::PRESENT_POSITION+1,
			  dynamixel::PRESENT_POSITION+2,
			  dynamixel::PRESENT_POSITION+3,
		  dynamixel::PRESENT_INPUT_VOLTAGE,
			  dynamixel::PRESENT_INPUT_VOLTAGE+1,
		  dynamixel::PRESENT_TEMPERATURE
	};

	/*
	dynamixel::Packet<uint8_t,1> short_sts(dynamixel::R_SHOULDER_PITCH+1, dynamixel::STATUS_RETURN, {0x00});
	dynamixel::PacketHandler<uint16_t,read_bank_addresses.size(),1>
			single_write_handler(
					ports[0],
					dynamixel::Packet<uint16_t,read_bank_addresses.size()>(
							dynamixel::R_SHOULDER_PITCH+1,
							dynamixel::WRITE,
							read_bank_addresses
					),
					1
			);
	*/
	const int l = 2+2+1+2*15+1+2*15;
	std::array<uint8_t,l> indirect_address_params;
	indirect_address_params[0] = 0xA8;
	indirect_address_params[1] = 0x00;
	indirect_address_params[2] = ((l-6)/2) & 0x00FF;
	indirect_address_params[3] = (((l-6)/2) >> 8) & 0x00FF;
	for (int i = 0; i < 2; i++) {
		uint16_t base = 4 + i*(1+2*15);
		indirect_address_params[base] =
				i == 0 ?
				dynamixel::R_SHOULDER_PITCH+1 :
				dynamixel::L_SHOULDER_PITCH+1;
		for (int j = 0; j < 15; j++) {
			indirect_address_params[base+2*j+1] = read_bank_addresses[j+1] & 0x00FF;
			indirect_address_params[base+2*j+2] = (read_bank_addresses[j+1] >> 8) & 0x00FF;
		}
	}
	//dynamixel::Packet<uint8_t,1> short_sts(dynamixel::R_SHOULDER_PITCH+1, dynamixel::STATUS_RETURN, {0x00});
	dynamixel::PacketHandler<uint8_t,indirect_address_params.size(),1>
			sync_write_handler(
					ports[0],
					dynamixel::Packet<uint8_t,indirect_address_params.size()>(
							dynamixel::ALL_DEVICES,
							dynamixel::SYNC_WRITE,
							indirect_address_params
					),
					2
			);
	std::array<dynamixel::ServoState, dynamixel::NUMBER_OF_DEVICES> local_cache;

	/* Delay for a bit to give the motor time to boot up. Without this delay, I
	* found that the motor does not respond at all. From some basic testing I
	* think that it is because the motor only boots up until the DXL power is
	* switched on from the DXL_POWER_EN pin. However, it may have something to
	* do with the RS485 transceivers instead.
	*/
	HAL_Delay(1000);

	// Begin the receiving. This should be done only once if we are using the DMA
	// as a buffer.
	for (auto& port : ports) {
	  port.begin_rx();
	  port.flush_rx();
	}

	/*do {
		single_write_handler.send_inst();

		while(!single_write_handler.check_sts())
			ports[0].check_tx();
		short_sts = single_write_handler.get_sts_packet(0);
	} while (
				(short_sts.crc != single_write_handler.get_crc(0))
			||	(short_sts.params[0] != dynamixel::NO_ERROR)
	);*/

	sync_write_handler.send_inst();

	/*
	while (1) {
		sync_write_handler.send_inst();

		while(!sync_write_handler.check_sts())
			ports[0].check_tx();
		for (int i = 0; i < 2; i++) {
			auto sync_write_sts = sync_write_handler.get_sts_packet(i);
			if ((sync_write_sts.crc != sync_write_handler.get_crc(i))
					&& (sync_write_sts.params[0] != dynamixel::NO_ERROR))
				break;
		}
	}
	*/

	/*
	dynamixel::PacketHandler<uint16_t,2,15+1> single_read_handler(
			ports[0],
			dynamixel::Packet<uint16_t,2> (
					dynamixel::L_SHOULDER_PITCH,
					dynamixel::READ,
					{0x00A8+2*28,15}
			),
			1
	);

	single_read_handler.send_inst();
	*/

	dynamixel::PacketHandler<uint16_t,3,15+1> sync_read_handler(
			ports[0],
			dynamixel::Packet<uint16_t,3> (
					dynamixel::ALL_DEVICES,
					dynamixel::SYNC_READ,
					{0x00A8+2*28,15,0x0201}
			),
			2
	);

	sync_read_handler.send_inst();

	while (1) {
		/*
		if (single_read_handler.check_sts()) {
			auto read_sts_packet = single_read_handler.get_sts_packet(0);

			if ((single_read_handler.is_sts_healthy(0))
					&& read_sts_packet.params[0] == dynamixel::NO_ERROR)
			{
				local_cache[0].convert_from_read_bank(
						*(dynamixel::ReadBank*)(read_sts_packet.params.data()+1)
				);

				std::stringstream ss;
				ss << local_cache[0];
				CDC_Transmit_HS((uint8_t*)ss.str().data(), ss.str().size());
			}
			else {
				std::string str = "Error\r\n";
				CDC_Transmit_HS((uint8_t*)str.data(), str.size());
			}

			single_read_handler.reset();

			// Delay so that the LED can be observed to blink at 2 Hz.
			HAL_Delay(500);

			single_read_handler.send_inst();
		}
		*/

		if (sync_read_handler.check_sts()) {
			for (int i = 0; i < 2; i++) {
				auto sync_read_sts = sync_read_handler.get_sts_packet(i);

				if ((sync_read_handler.is_sts_healthy(i))
						&& sync_read_sts.params[0] == dynamixel::NO_ERROR)
				{
					local_cache[0].convert_from_read_bank(
							*(dynamixel::ReadBank*)(sync_read_sts.params.data()+1)
					);

					std::stringstream ss;
					ss << "Servo: " << (i+1) << "\t" << local_cache[0];
					CDC_Transmit_HS((uint8_t*)ss.str().data(), ss.str().size());
				}
				else {
					std::string str = "Error\r\n";
					CDC_Transmit_HS((uint8_t*)str.data(), str.size());
				}
			}

			sync_read_handler.reset();

			// Delay so that the LED can be observed to blink at 2 Hz.
			HAL_Delay(500);

			sync_read_handler.send_inst();
		}

		// Always check the TX interrupts.
		ports[0].check_tx();
	}

#else
#ifdef TEST_UART
  test_hw::uart();
#endif

#ifdef TEST_USB
  test_hw::usb();
#endif

#ifdef TEST_IMU
  test_hw::imu();
#endif

#ifdef TEST_PORT
  test_hw::port();
#endif

#ifdef TEST_MOTOR
#if TEST_MOTOR == 1
  test_hw::motor_v1();
#else
  test_hw::motor_v2();
#endif // TEST_MOTOR == 1
#endif // TEST_MOTOR
#endif // RUN_MAIN

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
