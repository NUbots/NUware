/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart/RS485.h"
#include "uart/Port.hpp"
#include "stdio.h"
//#include "packet_handler.hpp"
#include "dynamixel/Packetiser.hpp"
//#include "dynamixel/PacketHandler.hpp"
#include "dynamixel/Dynamixel.hpp"

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
uint16_t adc_var[4];
uart::RS485 rs485(1);
char buffer[100];
const uint8_t* rx_msg;
uint8_t tx_msg[100];
uart::RS485::status Status;
/* USER CODE END 0 */
uart::Port port(1);
uint16_t byte;
uint8_t byte8;
uint8_t count;
uint8_t header[4] = {0xff,0xff,0xfd,0x00};
uint8_t device_ID = 21;
uint8_t Read_Command = 2;
uint8_t i;

dynamixel::Packetiser packetiser;

//enum Result { NONE = 0x00, SUCCESS, ERROR, CRC_ERROR, TIMEOUT };
//enum Result result = NONE;



//uart::Port::RingBuffer ringbuffer;


//dynamixel::PacketHandler packethandler;
//dynamixel::Packetiser packetiser;
//dynamixel::PacketHandler.Result =  result;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //  NUfsr_IMU_Init();

  //  uint16_t UART1_RX =

  //  uint8_t count;
  //  uint8_t set_flag;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

  // CONTROL TABLE
  struct {
	  std::array<uint16_t, 4> adcs = {0, 0, 0, 0};
	  uint16_t id = 21;
	  uint16_t broadcast_id = 0xFE;
	  std::array<uint8_t, 3> ping_return = {0, 0, 0};

  } control_table __attribute__((packed));

  uint32_t timeout;
  uint32_t last_value;
  //  	count = 0;
  HAL_TIM_Base_Start(&htim3);
//  HAL_TIM_Base_Start(&htim4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)control_table.adcs.data(), 4);// DMA Start

  port.begin_rx();
//  port.begin_tx();

  while (1)
  {

//	  sprintf(buffer, "Data FSR 1:{%04x}, FSR 2:{%04x}, FSR 3:{%04x}, FSR 4:{%04x}\r\n", adc_var[0], adc_var[1],adc_var[2], adc_var[3]);
//	  sprintf(buffer, "Data %04d %04d %04d %04d\r\n", adc_var[0], adc_var[1], adc_var[2], adc_var[3]);
//	sprintf(buffer, "Data {%08x%08x}\r\n", *(uint32_t*)&adc_var[2], *(uint32_t*)&adc_var[0]);
//  rs485.transmit((const uint8_t*)&buffer,(uint16_t)sizeof(buffer));

//  Port
//	  port.write((const uint8_t*)&buffer,(uint16_t)sizeof(buffer));
//	  byte = port.read();
//	  sprintf(buffer, "Data = %x",byte);
//	  port.write(buffer);

// while (port.peek() != 0xFFFF){
//	 byte = port.read();
//	 sprintf(buffer, "Data = %x\n",byte);
//	   port.write(buffer);



	  	  uint16_t num_bytes = port.get_available_rx();
	  	  for (size_t i = 0; i < num_bytes; ++i) {
	  		  packetiser.decode(port.read());
			  if (packetiser.is_packet_ready()) {
				  // Peek to see if there is a byte on the buffer yet.
				  // READ_COMMAND
				  if (packetiser.get_decoded_packet()[7] == dynamixel::Instruction::READ) {
					  auto inst = reinterpret_cast<const dynamixel::ReadCommand*>(packetiser.get_decoded_packet());
					  if ((inst->crc == packetiser.get_decoded_crc()) && (inst->id == control_table.id)) {
						  dynamixel::StatusReturnCommand<uint16_t, 4> sts(control_table.id, dynamixel::CommandError::NO_ERROR, control_table.adcs);
						  port.write(reinterpret_cast<uint8_t*>(&sts), sizeof(sts));
					  }
				  //
				  }
				  else if (packetiser.get_decoded_packet()[7] == dynamixel::Instruction::PING){
					  auto inst = reinterpret_cast<const dynamixel::PingCommand*>(packetiser.get_decoded_packet());
					  if (inst->crc == packetiser.get_decoded_crc()) {
						  // if ID == 120
						  if (inst->id == control_table.id){
							  dynamixel::StatusReturnCommand<uint8_t, 3> sts(control_table.id, dynamixel::CommandError::NO_ERROR, control_table.ping_return);
							  port.write(reinterpret_cast<uint8_t*>(&sts), sizeof(sts));
						  }
						  // else if ID == 0xFE
						  if (inst->id == control_table.broadcast_id){
							  // Start TIMER with of timeout of 2*id;
							  last_value = HAL_GetTick();
							  timeout = control_table.id*2;

						  }
					  }
				  }
				  packetiser.reset();
			  }
	  	  }
	  	if (HAL_GetTick() - last_value > timeout){
		  timeout = UINT32_MAX;
		  dynamixel::StatusReturnCommand<uint8_t, 3> sts(control_table.id, dynamixel::CommandError::NO_ERROR, control_table.ping_return);
		  port.write(reinterpret_cast<uint8_t*>(&sts), sizeof(sts));
		  packetiser.reset();
	  }


  //	NUfsr_IMU_TransmitReceive(ACCEL_XOUT_L | IMU_READ, 0x00, Ptr_Rx, 1); //Previous code
  //	NUfsr_UART_Transmit(&huart1, (void*)Ptr_Rx, 1);
  }
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV4;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
