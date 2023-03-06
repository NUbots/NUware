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
#include "dynamixel/Packetiser.hpp"
#include "uart/Port.h"
#include "dynamixel/PacketHandler.hpp"
#include "usbd_cdc_if.h"
#include "dynamixel/Devices.hpp"
#include "dynamixel/Packet.hpp"

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

#ifdef RUN_MAIN
  	// These are the ports on the NUsense board. They are either to be used for
    // sending packets directly or to be passed to a packet-handler.
	std::array<uart::Port,NUM_PORTS> ports = {{
			uart::Port(1),uart::Port(2),uart::Port(3),
			uart::Port(4),uart::Port(5),uart::Port(6)
	}};
	// This is the local storage of each servo's state. This is to be updated
	// regularly by polling the servos constantly and to be spammed to the NUC.
	std::array<dynamixel::ServoState, dynamixel::NUMBER_OF_DEVICES> local_cache;

	/* Delay for a bit to give the motor time to boot up. Without this delay, I
	 * found that the motor does not respond at all. From some basic testing, I
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

	// For now, we are just testing two servos on one port.

	/*
	 * ~~~ ~~~ ~~~ Set up of the Indirect Registers ~~~ ~~~ ~~~
	 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 * Here, the indirect registers are set up by a single sync-write
	 * instruction on each port. These indirect registers are used for the
	 * contiguous read-bank which is read constantly in a loop.
	 */
	// The parameters of the sync-write instruction can be quite complex. Here,
	// they are generated by a helper function.
	auto sync_write_params = dynamixel::make_sync_write_params<2>(
			{
				// For now, we are just testing two servos.
				dynamixel::R_SHOULDER_PITCH,
				dynamixel::L_SHOULDER_PITCH
			}
	);
	// Make the handler for the sync-write instruction.
	dynamixel::PacketHandler<uint8_t,sync_write_params.size(),1>
			sync_write_handler(
					ports[0], // the port
					dynamixel::Packet<uint8_t,sync_write_params.size()>(
							dynamixel::ALL_DEVICES, // the broadcast ID
							dynamixel::SYNC_WRITE, 	// the instruction
							sync_write_params 		// the parameters
					), // the packet
					0 // The sync-write instruction has no statuses in response.
			);

	// Send the sync-write instruction to now set the indirect-register up.
	// There are no status-packets in response.
	sync_write_handler.send_inst();
	// We do not have to check the TX of the port.

	/*
	 * ~~~ ~~~ ~~~ Polling of the Indirect Registers ~~~ ~~~ ~~~
	 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 * Here, the indirect registers are polled in a loop using sync-read
	 * instructions.
	 */
	// Make the handler for sync-read instruction.
	const uint16_t read_bank_length = sizeof(dynamixel::ReadBank);
	dynamixel::PacketHandler<uint8_t,2+2+2,read_bank_length> sync_read_handler(
			ports[0], // the port
			dynamixel::Packet<uint8_t,2+2+2> (
					dynamixel::ALL_DEVICES,	// the broadcast ID
					dynamixel::SYNC_READ,	// the instruction
					{
							// the starting address
							dynamixel::INDIRECT_DATA_1 & 0xFF,
							(dynamixel::INDIRECT_DATA_1 >> 8) & 0xFF,
							// the number of registers
							0x0F, 0x00,
							// the IDs
							// These need to be offset by one always when
							// packed into an array of parameters.
							dynamixel::R_SHOULDER_PITCH+1,
							dynamixel::L_SHOULDER_PITCH+1,
					} // the parameters
			), // the packet
			2 // Again, we are just testing two servos for now.
	);

	// Send the sync-read instruction to begin the loop.
	sync_read_handler.send_inst();

	/*
	 * ~~~ ~~~ ~~~ The Main Loop ~~~ ~~~ ~~~
	 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 */
	while (1) {
		// If all the expected statuses have been received and decoded, then
		// parse it for any errors and print the data.
		if (sync_read_handler.check_sts()) {
			// For now, we are just testing two servos.
			for (int i = 0; i < 2; i++) {
				const auto sync_read_sts = sync_read_handler.get_sts_packet(i);

				// If the status' CRC is right, and there is no error, then
				// parse it as a state and add it to the cache.
				if ((sync_read_handler.is_sts_healthy(i))
						&& sync_read_sts.params[0] == dynamixel::NO_ERROR)
				{
					local_cache[0].convert_from_read_bank(
							*reinterpret_cast<const dynamixel::ReadBank*>(
									sync_read_sts.params.data()+1
							)
					);

					// For now, print the state for testing.
					// Later on, this should be done somewhere else outside of
					// this if-statement.
					std::stringstream ss;
					ss << "Servo: " << (i+1) << "\t" << local_cache[0];
					CDC_Transmit_HS((uint8_t*)ss.str().data(), ss.str().size());
				}
				else {
					// If the CRC is wrong, or there is an error, then print it.
					std::stringstream ss;
					ss << "Servo: " << (i+1) << "\tError\t"
							<< sync_read_sts.params[0] << "\r" << std::endl;
					CDC_Transmit_HS((uint8_t*)ss.str().data(), ss.str().size());
				}
			}

			// Reset the handler.
			sync_read_handler.reset();

			// Delay for a bit.
			HAL_Delay(500);

			// Send the sync-read instruction again to continue the loop.
			sync_read_handler.send_inst();
		}

		// Always check the TX interrupts.
		/*
		 * Well, this is not strictly needed if only one packet is being sent
		 * at a time on each port. In fact, transmission may not even need a
		 * circular buffer in the first place, but it is there for completeness.
		 */
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
