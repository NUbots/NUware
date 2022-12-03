/*
 * rs485.h
 *
 *  Created on: Oct 21, 2022
 *      Author: Clayton
 */

#include "main.h" // needed for the GPIO labels,
#include "usart.h" // needed for UART handles,
#include "settings.h" // needed for buzz during test,
#include "gpio.h"

#ifndef INC_RS485_H_
#define INC_RS485_H_

#define RS485_RX GPIO_PIN_RESET
#define RS485_TX GPIO_PIN_SET

/*
 * Brief:		wraps the UART receive-function for RS485 hardware flow-control in polling-mode.
 * Note:		The DXL direction pin is reset during this function.
 * Arguments:	the handle for the UART interface,
 * 				the bytes to be sent,
 * 				the number of bytes, i.e. the length of the data,
 * 				the timeout in milliseconds, use HAL_MAX_DELAY for indefinite polling,
 * Returns:		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Receive(UART_HandleTypeDef* huart, uint8_t *pData, uint16_t size, uint32_t timeout);

/*
 * Brief:		wraps the UART receive-function for RS485 hardware flow-control in interrupt-mode.
 * Note:		The DXL direction pin is reset during this function.
 * Arguments:	the handle for the UART interface,
 * 				the bytes to be sent,
 * 				the number of bytes, i.e. the length of the data,
 * Returns:		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Receive_IT(UART_HandleTypeDef* huart, uint8_t *pData, uint16_t size);

/*
 * Brief:		wraps the UART transmit-function for RS485 hardware flow-control in polling-mode.
 * Note:		The DXL direction pin is set during this function.
 * Arguments:	the handle for the UART interface,
 * 				the bytes to be sent,
 * 				the number of bytes, i.e. the length of the data,
 * 				the timeout in milliseconds, use HAL_MAX_DELAY for indefinite polling,
 * Returns:		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef* huart, const uint8_t *pData, uint16_t size, uint32_t timeout);

/*
 * Brief:		wraps the UART transmit-function for RS485 hardware flow-control in interrupt-mode.
 * Note:		The DXL direction pin is set during this function.
 * Arguments:	the handle for the UART interface,
 * 				the bytes to be sent,
 * 				the number of bytes, i.e. the length of the data,
 * Returns:		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Transmit_IT(UART_HandleTypeDef* huart, const uint8_t *pData, uint16_t size);

#endif /* INC_RS485_H_ */
