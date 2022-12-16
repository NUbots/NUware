/*
 * rs485.h
 *
 *  Created on: Oct 21, 2022
 *      Author: Clayton
 *      Description: the legacy C-like library
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
 * @brief		wraps the UART receive-function for RS485 hardware flow-control in polling-mode.
 * @note		The DXL direction pin is reset during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @param		the timeout in milliseconds, use HAL_MAX_DELAY for indefinite polling,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t length, uint32_t timeout);

/*
 * @brief		wraps the UART receive-function for RS485 hardware flow-control in interrupt-mode.
 * @note		The DXL direction pin is reset during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Receive_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t length);

/*
 * @brief		wraps the UART transmit-function for RS485 hardware flow-control in polling-mode.
 * @note		The DXL direction pin is set during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @param		the timeout in milliseconds, use HAL_MAX_DELAY for indefinite polling,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t length, uint32_t timeout);

/*
 * @brief		wraps the UART transmit-function for RS485 hardware flow-control in interrupt-mode.
 * @note		The DXL direction pin is set during this function.
 * @param		the handle for the UART interface,
 * @param		the bytes to be sent,
 * @param		the number of bytes, i.e. the length of the data,
 * @return		the HAL status of the UART,
 */
HAL_StatusTypeDef RS485_Transmit_IT(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t length);

#endif /* INC_RS485_H_ */
