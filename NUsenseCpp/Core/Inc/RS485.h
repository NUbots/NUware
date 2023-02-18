/*
 * RS485.h
 *
 *  Created on: 16 Dec. 2022
 *      Author: Clayton
 */

#include "main.h" 		// needed for the GPIO labels and explicit types,
#include "usart.h" 		// needed for UART handles,
#include "settings.h" 	// needed for buzz during test,
#include "gpio.h"

#ifndef SRC_RS485_H_
#define SRC_RS485_H_

//#define DETECT_IDLE_LINE

#define RS485_RX GPIO_PIN_RESET
#define RS485_TX GPIO_PIN_SET

class RS485 {
public:
	// The kind of status: (shamefully stolen from HAL)
	enum status {
		// Keep these values as they are to be compatible with HAL.
		RS485_OK       = 0x00,
		RS485_ERROR    = 0x01,
		RS485_BUSY     = 0x02,
		RS485_TIMEOUT  = 0x03
		// May add some more when the time comes.
	};
	/*
	 * @brief		maps the GPIO port and pin to the given UART interface.
	 * @param		none,
	 * @return		nothing,
	 */
	RS485();
	/*
	 * @brief		maps the GPIO port and pin to the given UART interface.
	 * @param		the number corresponding to the UART interface,
	 * @return		nothing,
	 */
	RS485(uint8_t uart_number);
	/*
	 * @brief		maps the GPIO port and pin to the given UART interface.
	 * @param		the handle for the UART interface,
	 * @return		nothing,
	 */
	RS485(UART_HandleTypeDef* input_huart);
	/*
	 * @brief
	 * @note		nothing needs to be freed.
	 * @param		none,
	 * @return		nothing,
	 */
	virtual ~RS485();
	/*
	 * @brief		wraps the UART receive-function for RS485 hardware
	 * 				flow-control in polling-mode.
	 * @note		The DXL direction pin is reset during this function.
	 * @param		the bytes to be sent,
	 * @param		the number of bytes, i.e. the length of the data,
	 * @param		the timeout in milliseconds, use HAL_MAX_DELAY for
	 * 				indefinite polling,
	 * @return		the status of the UART,
	 */
	status receive_poll(uint8_t* data, uint16_t length, uint32_t timeout);
	/*
	 * @brief		wraps the UART receive-function for RS485 hardware
	 * 				flow-control in interrupt-mode.
	 * @note		The DXL direction pin is reset during this function.
	 * @param		the bytes to be sent,
	 * @param		the number of bytes, i.e. the length of the data,
	 * @return		the status of the UART,
	 */
	status receive_it(uint8_t* data, uint16_t length);

	/*
	 * @brief		wraps the UART receive-function for RS485 hardware
	 * 				flow-control in dma-mode.
	 * @note		The DXL direction pin is reset during this function.
	 * @param		the bytes to be sent,
	 * @param		the number of bytes, i.e. the length of the data,
	 * @return		the status of the UART,
	 */
	status receive(uint8_t* data, uint16_t length);

	/*
	 * @brief		checks for the interrupt-flags for the receiving to be done.
	 * @param		none,
	 * @retval		#true if the receiving was done, i.e. something has been
	 * 				received,
	 * @retval		#false if nothing has not been received yet,
	 */
	bool get_receive_flag();
	/*
	 * @brief		wraps the UART transmit-function for RS485 hardware
	 * 				flow-control in polling-mode.
	 * @note		The DXL direction pin is set during this function.
	 * @param		the bytes to be sent,
	 * @param		the number of bytes, i.e. the length of the data,
	 * @param		the timeout in milliseconds, use HAL_MAX_DELAY for
	 * 				indefinite polling,
	 * @return		the status of the UART,
	 */
	status transmit_poll(const uint8_t* data, uint16_t length, uint32_t timeout);
	/*
	 * @brief		wraps the UART transmit-function for RS485 hardware
	 * 				flow-control in interrupt-mode.
	 * @note		The DXL direction pin is set during this function.
	 * @param		the bytes to be sent,
	 * @param		the number of bytes, i.e. the length of the data,
	 * @return		the status of the UART,
	 */
	status transmit_it(const uint8_t* data, uint16_t length);

	/*
	 * @brief		wraps the UART transmit-function for RS485 hardware
	 * 				flow-control in dma-mode.
	 * @note		The DXL direction pin is set during this function.
	 * @note		This should be used more preferably now that the bug has
	 * 				been fixed.
	 * @param		the bytes to be sent,
	 * @param		the number of bytes, i.e. the length of the data,
	 * @return		the status of the UART,
	 */
	status transmit(const uint8_t* data, uint16_t length);

	/*
	 * @brief		checks for the interrupt-flags for the transmitting to be
	 * 				done.
	 * @note		The DXL direction pin is reset during this function if the
	 * 				flag has been set by the interrupt.
	 * @param		none,
	 * @retval		#true if the transmitting was done,
	 * @retval		#false if not everything has been transmitted yet,
	 */
	bool get_transmit_flag();
private:
	// The handle of the corresponding UART interface:
	UART_HandleTypeDef* huart;
	// The handles of the corresponding DMA interfaces:
	DMA_HandleTypeDef* hdma_rx, * hdma_tx;
	// The GPIO port of the direction-pin:
	GPIO_TypeDef* gpio_port;
	// The GPIO pin of the direction-pin:
	uint16_t gpio_pin;
	// The mask for the given UART interface's interrupt for receiving:
	uint16_t it_rx_mask;
	// The mask for the given UART interface's interrupt for transmitting:
	uint16_t it_tx_mask;
};

#endif /* SRC_RS485_H_ */
