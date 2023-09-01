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
#include <algorithm>
#include "dynamixel/Packetiser.hpp"
#include "dynamixel/PacketHandler.hpp"
#include "uart/Port.hpp"
#include "usbd_cdc_if.h"
#include "dynamixel/Dynamixel.hpp"
#include "dynamixel/DynamixelServo.hpp"
#include "platform/NUsense/NUgus.hpp"
#include "platform/NUsense/process_data.hpp"
#include "platform/ServoState.hpp"

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

  /* USER CODE BEGIN I//nit */

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
  HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);

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
	std::array<platform::ServoState, platform::NUsense::NUMBER_OF_DEVICES> local_cache;

	/*
	 * For now, this is just a very crude way of knowing what devices are on
	 * what port without polling each port. Later we will get polling at
	 * start-up so that the devices do not have to be connected to a specific
	 * port. However, there should still be some thought put into how the
	 * devices are connected and daisy-chained since we now have a split bus
	 * which we should exploit. For example, having all devices daisy-chained
	 * on what port would defeat the main advantage of NUsense in the first
	 * place. Furthermore, the packet-load should maybe be evenly distributed.
	 */
	std::array<std::vector<platform::NUsense::NUgus::ID>,NUM_PORTS>
		chains = {
			std::vector<platform::NUsense::NUgus::ID>{
					platform::NUsense::NUgus::ID::R_SHOULDER_PITCH,
					platform::NUsense::NUgus::ID::R_SHOULDER_ROLL,
					platform::NUsense::NUgus::ID::R_ELBOW,
					platform::NUsense::NUgus::ID::HEAD_YAW
			},
			std::vector<platform::NUsense::NUgus::ID>{
					platform::NUsense::NUgus::ID::L_SHOULDER_PITCH,
					platform::NUsense::NUgus::ID::L_SHOULDER_ROLL,
					platform::NUsense::NUgus::ID::L_ELBOW,
					platform::NUsense::NUgus::ID::HEAD_PITCH
			},
			std::vector<platform::NUsense::NUgus::ID>{
					platform::NUsense::NUgus::ID::R_HIP_YAW,
					platform::NUsense::NUgus::ID::R_HIP_ROLL,
					platform::NUsense::NUgus::ID::R_HIP_PITCH
			},
			std::vector<platform::NUsense::NUgus::ID>{
					platform::NUsense::NUgus::ID::L_HIP_YAW,
					platform::NUsense::NUgus::ID::L_HIP_ROLL,
					platform::NUsense::NUgus::ID::L_HIP_PITCH
			},
			std::vector<platform::NUsense::NUgus::ID>{
					platform::NUsense::NUgus::ID::R_KNEE,
					platform::NUsense::NUgus::ID::R_ANKLE_PITCH,
					platform::NUsense::NUgus::ID::R_ANKLE_ROLL
			},
			std::vector<platform::NUsense::NUgus::ID>{
					platform::NUsense::NUgus::ID::L_KNEE,
					platform::NUsense::NUgus::ID::L_ANKLE_PITCH,
					platform::NUsense::NUgus::ID::L_ANKLE_ROLL
			}
	};

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

	/*
		~~~ ~~~ ~~~ Baisc Set-up ~~~ ~~~ ~~~
		~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		Here, the servos are set up to have no return-delay, to return always
		to a write-instruction (unlike the OpenCR set-up), and to have time-
		based profile-velocity control.
	*/

	// For each port, write for all servos the return-delay-time to be 0 μs.
	for (int i = 0; i < NUM_PORTS; i++) {

		// Re-use the same packet-hanlder for each port.
		dynamixel::PacketHandler packet_handler(ports[i]);

		for (const auto& id : chains[i]) {
			// Send the write-instruction again if there is something wrong 
			// with the returned status.
			do {
				// Reset the packet-handler before a new interaction has begun.
				packet_handler.reset();

				// Send the write-instruction.
				ports[i].write(
					dynamixel::WriteCommand<uint8_t>(
						(uint8_t)id,
						(uint16_t)dynamixel::DynamixelServo::Address::RETURN_DELAY_TIME,
						0x00
					)
				);

				// Wait for the status to be received and decoded.
				while (
					packet_handler.check_sts<0>(id) 
					== dynamixel::PacketHandler::Result::NONE
				);
			} while (
				packet_handler.get_result() 
				!= dynamixel::PacketHandler::Result::SUCCESS
			);
		}
	}

	// For each port, write for all servos the status-return-level to allow all
	// statuses to be returned.
	// Arguably, this is not needed since it is 0x02 by default.
	for (int i = 0; i < NUM_PORTS; i++) {

		// Re-use the same packet-hanlder for each port.
		dynamixel::PacketHandler packet_handler(ports[i]);

		for (const auto& id : chains[i]) {
			// Send the write-instruction again if there is something wrong 
			// with the returned status.
			do {
				// Reset the packet-handler before a new interaction has begun.
				packet_handler.reset();

				// Send the write-instruction.
				ports[i].write(
					dynamixel::WriteCommand<uint8_t>(
						(uint8_t)id,
						(uint16_t)dynamixel::DynamixelServo::Address::STATUS_RETURN_LEVEL,
						0x02
					)
				);

				// Wait for the status to be received and decoded.
				while (
					packet_handler.check_sts<0>(id) 
					== dynamixel::PacketHandler::Result::NONE
				);
			} while (
				packet_handler.get_result() 
				!= dynamixel::PacketHandler::Result::SUCCESS
			);
		}
	}

	// For each port, write for all servos the status-return-level to allow all
	// statuses to be returned.
	// Arguably, this is not needed since it is 0x02 by default.
	for (int i = 0; i < NUM_PORTS; i++) {

		// Re-use the same packet-hanlder for each port.
		dynamixel::PacketHandler packet_handler(ports[i]);

		for (const auto& id : chains[i]) {
			// Send the write-instruction again if there is something wrong 
			// with the returned status.
			do {
				// Reset the packet-handler before a new interaction has begun.
				packet_handler.reset();

				// Send the write-instruction.
				ports[i].write(
					dynamixel::WriteCommand<uint8_t>(
						(uint8_t)id,
						(uint16_t)dynamixel::DynamixelServo::Address::DRIVE_MODE,
						0x04
					)
				);

				// Wait for the status to be received and decoded.
				while (
					packet_handler.check_sts<0>(id) 
					== dynamixel::PacketHandler::Result::NONE
				);
			} while (
				packet_handler.get_result() 
				!= dynamixel::PacketHandler::Result::SUCCESS
			);
		}
	}

	/*
	 * ~~~ ~~~ ~~~ Set-up of the Indirect Registers ~~~ ~~~ ~~~
	 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 * Here, the indirect registers are set up by a sequence of 
	 * write-instructions on each port. These indirect registers are used for 
	 * the contiguous read-bank which is read constantly in a loop and the two 
	 * write-banks.
	 */

	// For each port, write for all servos the addresses of the read-bank to 
	// the indirect registers.
	for (int i = 0; i < NUM_PORTS; i++) {

		// Re-use the same packet-hanlder for each port.
		dynamixel::PacketHandler packet_handler(ports[i]);

		for (const auto& id : chains[i]) {
			// Send the write-instruction again if there is something wrong 
			// with the returned status.
			do {
				// Reset the packet-handler before a new interaction has begun.
				packet_handler.reset();

				// Send the write-instruction.
				ports[i].write(
					dynamixel::WriteCommand<std::array<uint16_t,17>>(
						(uint8_t)id,
						(uint16_t)platform::NUsense::AddressBook::SERVO_READ_ADDRESS,
						{
							uint16_t(dynamixel::DynamixelServo::Address::TORQUE_ENABLE),
							uint16_t(dynamixel::DynamixelServo::Address::HARDWARE_ERROR_STATUS),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_PWM_L),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_PWM_H),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_CURRENT_L),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_CURRENT_H),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_VELOCITY_L),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_VELOCITY_2),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_VELOCITY_3),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_VELOCITY_H),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_POSITION_L),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_POSITION_2),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_POSITION_3),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_POSITION_H),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_INPUT_VOLTAGE_L),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_INPUT_VOLTAGE_H),
							uint16_t(dynamixel::DynamixelServo::Address::PRESENT_TEMPERATURE)
						}
					)
				);

				// Wait for the status to be received and decoded.
				while (
					packet_handler.check_sts<0>(id) 
					== dynamixel::PacketHandler::Result::NONE
				);
			} while (
				packet_handler.get_result() 
				!= dynamixel::PacketHandler::Result::SUCCESS
			);
		}
	}

	// For each port, write for all servos the addresses of the first 
	// write-bank to the indirect registers.
	for (int i = 0; i < NUM_PORTS; i++) {

		// Re-use the same packet-hanlder for each port.
		dynamixel::PacketHandler packet_handler(ports[i]);

		for (const auto& id : chains[i]) {
			// Send the write-instruction again if there is something wrong 
			// with the returned status.
			do {
				// Reset the packet-handler before a new interaction has begun.
				packet_handler.reset();

				// Send the write-instruction.
				ports[i].write(
					dynamixel::WriteCommand<std::array<uint16_t,11>>(
						(uint8_t)id,
						(uint16_t)platform::NUsense::AddressBook::SERVO_WRITE_ADDRESS_1,
						{
							uint16_t(dynamixel::DynamixelServo::Address::TORQUE_ENABLE),
							uint16_t(dynamixel::DynamixelServo::Address::VELOCITY_I_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::VELOCITY_I_GAIN_H),
							uint16_t(dynamixel::DynamixelServo::Address::VELOCITY_P_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::VELOCITY_P_GAIN_H),
							uint16_t(dynamixel::DynamixelServo::Address::POSITION_D_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::POSITION_D_GAIN_H),
							uint16_t(dynamixel::DynamixelServo::Address::POSITION_I_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::POSITION_I_GAIN_H),
							uint16_t(dynamixel::DynamixelServo::Address::POSITION_P_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::POSITION_P_GAIN_H)
						}
					)
				);

				// Wait for the status to be received and decoded.
				while (
					packet_handler.check_sts<0>(id) 
					== dynamixel::PacketHandler::Result::NONE
				);
			} while (
				packet_handler.get_result() 
				!= dynamixel::PacketHandler::Result::SUCCESS
			);
		}
	}

	// For each port, write for all servos the addresses of the second 
	// write-bank to the indirect registers.
	for (int i = 0; i < NUM_PORTS; i++) {

		// Re-use the same packet-hanlder for each port.
		dynamixel::PacketHandler packet_handler(ports[i]);

		for (const auto& id : chains[i]) {
			// Send the write-instruction again if there is something wrong 
			// with the returned status.
			do {
				// Reset the packet-handler before a new interaction has begun.
				packet_handler.reset();

				// Send the write-instruction.
				ports[i].write(
					dynamixel::WriteCommand<std::array<uint16_t,24>>(
						(uint8_t)id,
						(uint16_t)platform::NUsense::AddressBook::SERVO_WRITE_ADDRESS_2,
						{
							uint16_t(dynamixel::DynamixelServo::Address::FEEDFORWARD_1ST_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::FEEDFORWARD_1ST_GAIN_H),
							uint16_t(dynamixel::DynamixelServo::Address::FEEDFORWARD_2ND_GAIN_L),
							uint16_t(dynamixel::DynamixelServo::Address::FEEDFORWARD_2ND_GAIN_H),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_PWM_L),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_PWM_H),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_CURRENT_L),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_CURRENT_H),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_VELOCITY_L),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_VELOCITY_2),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_VELOCITY_3),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_VELOCITY_H),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_ACCELERATION_L),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_ACCELERATION_2),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_ACCELERATION_3),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_ACCELERATION_H),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_VELOCITY_L),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_VELOCITY_2),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_VELOCITY_3),
							uint16_t(dynamixel::DynamixelServo::Address::PROFILE_VELOCITY_H),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_POSITION_L),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_POSITION_2),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_POSITION_3),
							uint16_t(dynamixel::DynamixelServo::Address::GOAL_POSITION_H)
						}
					)
				);

				// Wait for the status to be received and decoded.
				while (
					packet_handler.check_sts<0>(id) 
					== dynamixel::PacketHandler::Result::NONE
				);
			} while (
				packet_handler.get_result() 
				!= dynamixel::PacketHandler::Result::SUCCESS
			);
		}
	}

	// Make a packet-handler for each port.
	std::array<dynamixel::PacketHandler,NUM_PORTS> packet_handlers = {
		dynamixel::PacketHandler(ports[0]),
		dynamixel::PacketHandler(ports[1]),
		dynamixel::PacketHandler(ports[2]),
		dynamixel::PacketHandler(ports[3]),
		dynamixel::PacketHandler(ports[4]),
		dynamixel::PacketHandler(ports[5])
	};

	std::array<uint8_t,NUM_PORTS> current_ids; current_ids.fill(0);

	// Send the first write-instruction to begin the chain-reaction on each 
	// port.
	std::vector<platform::NUsense::NUgus::ID> chain = chains[0];
	for (int i = 0; i < NUM_PORTS; i++) {
		ports[i].write(
			dynamixel::ReadCommand(
				(uint8_t)(chains[i])[current_ids[i]],
				(uint16_t)platform::NUsense::AddressBook::SERVO_READ,
				(uint16_t)sizeof(platform::NUsense::DynamixelServoReadData)
			)
		);
	}
	
	/*
	 * ~~~ ~~~ ~~~ The Main Loop ~~~ ~~~ ~~~
	 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 */
	while (1) {
		// For each port, check whether the expected status has been
		// successfully received. If so, then handle it and send the next read-
		// instruction.
		for (int i = 0; i < NUM_PORTS; i++) {
		//for (int i = 0; i < 1; i++) {
			platform::NUsense::NUgus::ID current_id = (chains[i])[current_ids[i]];

			if (packet_handlers[i].check_sts
				<sizeof(platform::NUsense::DynamixelServoReadData)>
				(current_id)
				== dynamixel::PacketHandler::SUCCESS
			) {
				// Parse and convert the read data to the local cache.
				platform::NUsense::process_servo_data(
					local_cache, 
					*reinterpret_cast<const dynamixel::StatusReturnCommand<sizeof(
						platform::NUsense::DynamixelServoReadData
					)>*>(
						packet_handlers[i].get_sts_packet()
					)
				);

				// For now, print the read-bank for testing.
				// Later on, this should be done somewhere else outside of
				// this if-statement.
				std::stringstream ss;
				ss 	<< "Port: " << i
						// For some ungodly reason, stringstream reads a
						// uint8_t as an ASCII character. Thus, current_id
						// needs to cast to a uint16_t.
					<< " Servo: " << ((uint16_t)current_id)
					<< "\t" << local_cache[(uint8_t)current_id-1];
				CDC_Transmit_HS((uint8_t*)ss.str().data(), ss.str().size());

				// Wait for a little bit.
				HAL_Delay(500);

				// Send a read-instruction for the next servo along the chain.
				packet_handlers[i].reset();
				current_ids[i] = (current_ids[i] + 1) % chains[i].size();
				ports[i].write(
					dynamixel::ReadCommand(
						(uint8_t)(chains[i])[current_ids[i]],
						(uint16_t)platform::NUsense::AddressBook::SERVO_READ,
						(uint16_t)sizeof(platform::NUsense::DynamixelServoReadData)
					)
				);
			}
		}
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
