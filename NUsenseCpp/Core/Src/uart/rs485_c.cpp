/*
 * rs485.c
 *
 *  Created on: Oct 21, 2022
 *      Author: Clayton
 *      Description: the legacy C-like library
 */

#include "rs485_c.h"

namespace uart {

/*
 * @brief		wraps the UART receive-function for RS485 hardware flow-control in polling-mode.
 * @note		The DXL direction pin is reset during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @param		the timeout in milliseconds, use HAL_MAX_DELAY for indefinite polling,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Receive(UART_HandleTypeDef* huart, uint8_t* data, uint16_t length, uint32_t timeout) {
	// Declare local variables.
	GPIO_TypeDef* gpio_port;
	uint16_t gpio_pin;
	HAL_StatusTypeDef status;

	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	if (huart == &huart1) 		{gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin;}
	else if (huart == &huart2) 	{gpio_port = DXL_DIR2_GPIO_Port; gpio_pin = DXL_DIR2_Pin;}
	else if (huart == &huart3) 	{gpio_port = DXL_DIR3_GPIO_Port; gpio_pin = DXL_DIR3_Pin;}
	else if (huart == &huart4) 	{gpio_port = DXL_DIR4_GPIO_Port; gpio_pin = DXL_DIR4_Pin;}
	else if (huart == &huart5) 	{gpio_port = DXL_DIR5_GPIO_Port; gpio_pin = DXL_DIR5_Pin;}
	else if (huart == &huart6) 	{gpio_port = DXL_DIR6_GPIO_Port; gpio_pin = DXL_DIR6_Pin;}

	// Set the hardware to the receiving direction and listen.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_RX);
	status = HAL_UART_Receive(huart, data, length, timeout);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (HAL_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

/*
 * @brief		wraps the UART receive-function for RS485 hardware flow-control in interrupt-mode.
 * @note		The DXL direction pin is reset during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Receive_IT(UART_HandleTypeDef* huart, uint8_t* data, uint16_t length) {
	// Declare local variables.
	GPIO_TypeDef* gpio_port;
	uint16_t gpio_pin;
	HAL_StatusTypeDef status;

	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	if (huart == &huart1) 		{gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin;}
	else if (huart == &huart2) 	{gpio_port = DXL_DIR2_GPIO_Port; gpio_pin = DXL_DIR2_Pin;}
	else if (huart == &huart3) 	{gpio_port = DXL_DIR3_GPIO_Port; gpio_pin = DXL_DIR3_Pin;}
	else if (huart == &huart4) 	{gpio_port = DXL_DIR4_GPIO_Port; gpio_pin = DXL_DIR4_Pin;}
	else if (huart == &huart5) 	{gpio_port = DXL_DIR5_GPIO_Port; gpio_pin = DXL_DIR5_Pin;}
	else if (huart == &huart6) 	{gpio_port = DXL_DIR6_GPIO_Port; gpio_pin = DXL_DIR6_Pin;}

	// Set the hardware to the receiving direction and listen.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_RX);
	status = HAL_UART_Receive_IT(huart, data, length);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (HAL_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

/*
 * @brief		wraps the UART transmit-function for RS485 hardware flow-control in polling-mode.
 * @note		The DXL direction pin is set during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @param		the timeout in milliseconds, use HAL_MAX_DELAY for indefinite polling,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length, uint32_t timeout) {
	// Declare local variables.
	GPIO_TypeDef* gpio_port;
	uint16_t gpio_pin;
	HAL_StatusTypeDef status;

	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	if (huart == &huart1) 		{gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin;}
	else if (huart == &huart2) 	{gpio_port = DXL_DIR2_GPIO_Port; gpio_pin = DXL_DIR2_Pin;}
	else if (huart == &huart3) 	{gpio_port = DXL_DIR3_GPIO_Port; gpio_pin = DXL_DIR3_Pin;}
	else if (huart == &huart4) 	{gpio_port = DXL_DIR4_GPIO_Port; gpio_pin = DXL_DIR4_Pin;}
	else if (huart == &huart5) 	{gpio_port = DXL_DIR5_GPIO_Port; gpio_pin = DXL_DIR5_Pin;}
	else if (huart == &huart6) 	{gpio_port = DXL_DIR6_GPIO_Port; gpio_pin = DXL_DIR6_Pin;}

	// Set the hardware to the receiving direction and send.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_TX);
	status = HAL_UART_Transmit(huart, data, length, timeout);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (HAL_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

/*
 * @brief		wraps the UART transmit-function for RS485 hardware flow-control in interrupt-mode.
 * @note		The DXL direction pin is set during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Transmit_IT(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t length) {
	// Declare local variables.
	GPIO_TypeDef* gpio_port;
	uint16_t gpio_pin;
	HAL_StatusTypeDef status;

	// Map the GPIO port and pin to the correct one corresponding to the given UART interface.
	if (huart == &huart1) 		{gpio_port = DXL_DIR1_GPIO_Port; gpio_pin = DXL_DIR1_Pin;}
	else if (huart == &huart2) 	{gpio_port = DXL_DIR2_GPIO_Port; gpio_pin = DXL_DIR2_Pin;}
	else if (huart == &huart3) 	{gpio_port = DXL_DIR3_GPIO_Port; gpio_pin = DXL_DIR3_Pin;}
	else if (huart == &huart4) 	{gpio_port = DXL_DIR4_GPIO_Port; gpio_pin = DXL_DIR4_Pin;}
	else if (huart == &huart5) 	{gpio_port = DXL_DIR5_GPIO_Port; gpio_pin = DXL_DIR5_Pin;}
	else if (huart == &huart6) 	{gpio_port = DXL_DIR6_GPIO_Port; gpio_pin = DXL_DIR6_Pin;}

	// Set the hardware to the receiving direction and send.
	HAL_GPIO_WritePin(gpio_port, gpio_pin, RS485_TX);
	status = HAL_UART_Transmit_IT(huart, data, length);
#ifdef TEST_UART
	// If this is during a test, then play buzzer when there is an error.
	if (HAL_OK != status) {
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(BUZZER_SIG_GPIO_Port, BUZZER_SIG_Pin, GPIO_PIN_RESET);
	}
#endif

	return status;
}

} // namespace uart
