/*
 * RS485.cpp
 *
 *  Created on: 16 Dec. 2022
 *      Author: Clayton
 */

#include "RS485.h"

RS485::RS485() {
	huart = &huart1;
	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin; it_rx_mask = UART1_RX; it_tx_mask = UART1_TX;
}


RS485::RS485(uint8_t uart_number) {
	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	switch (uart_number) {
	case 1: huart = &huart1; gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin; it_rx_mask = UART1_RX; it_tx_mask = UART1_TX; break;
	case 2: huart = &huart2; gpio_port = DXL_DIR2_GPIO_Port; gpio_pin = DXL_DIR2_Pin; it_rx_mask = UART2_RX; it_tx_mask = UART2_TX; break;
	case 3: huart = &huart3; gpio_port = DXL_DIR3_GPIO_Port; gpio_pin = DXL_DIR3_Pin; it_rx_mask = UART3_RX; it_tx_mask = UART3_TX; break;
	case 4: huart = &huart4; gpio_port = DXL_DIR4_GPIO_Port; gpio_pin = DXL_DIR4_Pin; it_rx_mask = UART4_RX; it_tx_mask = UART4_TX; break;
	case 5: huart = &huart5; gpio_port = DXL_DIR5_GPIO_Port; gpio_pin = DXL_DIR5_Pin; it_rx_mask = UART5_RX; it_tx_mask = UART5_TX; break;
	case 6: huart = &huart6; gpio_port = DXL_DIR6_GPIO_Port; gpio_pin = DXL_DIR6_Pin; it_rx_mask = UART6_RX; it_tx_mask = UART6_TX; break;
	}
}


RS485::RS485(UART_HandleTypeDef* input_huart) {
	huart = input_huart;
	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	if (huart == &huart1) 		{gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin; it_rx_mask = UART1_RX; it_tx_mask = UART1_TX;}
	else if (huart == &huart2) 	{gpio_port = DXL_DIR2_GPIO_Port; gpio_pin = DXL_DIR2_Pin; it_rx_mask = UART2_RX; it_tx_mask = UART2_TX;}
	else if (huart == &huart3) 	{gpio_port = DXL_DIR3_GPIO_Port; gpio_pin = DXL_DIR3_Pin; it_rx_mask = UART3_RX; it_tx_mask = UART3_TX;}
	else if (huart == &huart4) 	{gpio_port = DXL_DIR4_GPIO_Port; gpio_pin = DXL_DIR4_Pin; it_rx_mask = UART4_RX; it_tx_mask = UART4_TX;}
	else if (huart == &huart5) 	{gpio_port = DXL_DIR5_GPIO_Port; gpio_pin = DXL_DIR5_Pin; it_rx_mask = UART5_RX; it_tx_mask = UART5_TX;}
	else if (huart == &huart6) 	{gpio_port = DXL_DIR6_GPIO_Port; gpio_pin = DXL_DIR6_Pin; it_rx_mask = UART6_RX; it_tx_mask = UART6_TX;}
}

RS485::~RS485() {
	// May do something with this later.
}

RS485::status RS485::receive_poll(uint8_t* data, uint16_t length, uint32_t timeout) {
	// Declare local variables.
	RS485::status status;

	// Set the hardware to the receiving direction and listen.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_RX);
	status = (RS485::status)HAL_UART_Receive(huart, data, length, timeout);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (RS485_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

RS485::status RS485::receive_it(uint8_t* data, uint16_t length) {
	// Declare local variables.
	RS485::status status;

	// Set the hardware to the receiving direction and listen.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_RX);
	status = (RS485::status)HAL_UART_Receive_IT(huart, data, length);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (RS485_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

RS485::status RS485::receive(uint8_t* data, uint16_t length) {
	// Declare local variables.
	RS485::status status;

	// Set the hardware to the receiving direction and listen.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_RX);
	status = (RS485::status)HAL_UART_Receive_DMA(huart, data, length);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (RS485_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

bool RS485::get_receive_flag() {
	if (uart_it_flags & it_rx_mask) {
		uart_it_flags &= ~it_rx_mask;
		return true;
	} else
		return false;
}

RS485::status RS485::transmit_poll(const uint8_t* data, uint16_t length, uint32_t timeout) {
	// Declare local variables.
	RS485::status status;

	// Set the hardware to the receiving direction and send.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_TX);
	status = (RS485::status)HAL_UART_Transmit(huart, data, length, timeout);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (RS485_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

RS485::status RS485::transmit_it(const uint8_t* data, uint16_t length) {
	// Declare local variables.
	RS485::status status;

	// Set the hardware to the receiving direction and send.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_TX);
	status = (RS485::status)HAL_UART_Transmit_IT(huart, data, length);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (RS485_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

RS485::status RS485::transmit(const uint8_t* data, uint16_t length) {
	// Declare local variables.
	RS485::status status;

	// Set the hardware to the receiving direction and send.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_TX);
	status = (RS485::status)HAL_UART_Transmit_DMA(huart, data, length);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (RS485_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

bool RS485::get_transmit_flag() {
	if (uart_it_flags & it_tx_mask) {
		uart_it_flags &= ~it_tx_mask;
		return true;
	} else
		return false;
}
