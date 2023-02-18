/*
 * Port.h
 *
 *  Created on: 18 Jan. 2023
 *      Author: Clayton
 */

#include "stdint.h" 	// needed for explicit type-defines
#include "RS485.h"		// needed for the RS485 interface
#include <deque>
#include <vector>
#include "main.h"		// only used for GPIO labels for debugging

#ifndef SRC_PORT_H_
#define SRC_PORT_H_

//#define USE_QUEUE_CLASS
//#define SEE_STATISTICS
//#define USE_DMA_RX_BUFFER

#define PORT_BUFFER_SIZE 	2048

#define NO_BYTE_READ		0xFFFF

class Port {
private:
	// @brief	The RS485 link:
	// @note	This should be unique for all ports, i.e. only one port for
	// 				each RS485 link.
	RS485 rs_link;
#ifdef USE_QUEUE_CLASS
	// @brief	The buffers for RX and TX,
	std::deque<uint8_t> rx_buffer;
	std::vector<uint8_t> tx_buffer;
	std::vector<uint8_t> bytes_tx;
#else
	// May be replaced with a more C++-like container class, e.g. a queue.
	// However, may be faster than the queue class.
	// Not sure if that would be too much overhead for an embedded system or
	// even overkill for a humble port-class.
	struct RingBuffer
	{
		RingBuffer() : front(0), back(0), size(0) {}
		// @brief	the data of the buffer:
		uint8_t data[PORT_BUFFER_SIZE];
		// @brief	the front of the 'queue' where bytes are read or popped,
		// @note	This is inclusive of the first byte.
		// @note	"I have been waiting for so long; I am nearly at the front
		// 			of the queue."
		volatile uint16_t front;
		// @brief	the back of the 'queue' where bytes are added or pushed,
		// @note	This is inclusive of the last byte.
		// @note	"That rude man just cut in line; he should go at the back
		// 			of the queue."
		volatile uint16_t back;
		// @brief	the number of bytes in the ring-buffer,
		volatile uint16_t size;
		/**
		 * @brief	adds one byte to the back of the queue.
		 * @note	This is a helper function; it is not meant to encapsulate
		 * 			anything.
		 * @param	the byte to be added,
		 * @return	nothing,
		 */
		void push(uint8_t byte) {
			// Move the back backwards (higher) in the array unless the back is
			// already at the front, i.e. the buffer is only one byte long.
			if (size != 0) back = (back + 1) % PORT_BUFFER_SIZE;
			data[back] = byte;
			size++;
		}
		/**
		 * @brief	removes one byte from the front of the queue.
		 * @note	This is a helper function; it is not meant to encapsulate
		 * 			anything.
		 * @param	none,
		 * @return	the byte to be removed,
		 */
		uint8_t pop() {
			uint8_t byte;
			byte = data[front];
			// Update the front to move back (higher) in the array unless the
			// front is already at the back, i.e. the buffer is only one byte
			// long.
			if (size != 1) front = (front + 1) % PORT_BUFFER_SIZE;
			// Either way, decrease the size.
			size--;
			return byte;
		}

	};
	// @brief	The buffer for TX,
	RingBuffer tx_buffer;
#ifdef USE_DMA_RX_BUFFER
	// @brief	The buffer for RX,
	RingBuffer rx_buffer;
#else
	// @brief	The buffer for RX,
	RingBuffer rx_buffer;
#endif
#endif
	// @brief 	The number of bytes just transmitted,
	volatile uint16_t num_bytes_tx;
	// @brief	The received byte:
	uint8_t received_byte;
	// @brief	The state of the half-duplex channel,
	enum {
		RX_IDLE = 0,
		TX_BUSY,
		TX_DONE
	} comm_state;
	//comm_state_type ;
#ifdef SEE_STATISTICS
	uint16_t old_num_bytes_tx[10];
#endif

	/*
	 * @brief		begins transmitting all remaining bytes in the tx-buffer,
	 * @note		This should only be called within the class-functions, not
	 * 				outside of it.
	 * @param		none.
	 * @return		the status,
	 */
	uint8_t begin_tx();
	/*
	 * @brief		handles the receive-complete interrupt,
	 * @param		none.
	 * @return		nothing,
	 */
	void handle_rx();
	/*
	 * @brief		handles the transmit-complete interrupt,
	 * @param		none.
	 * @return		nothing,
	 */
	void handle_tx();
public:
	/**
	 * @brief		constructs the port by mapping the number to the
	 * 				corresponding UART interface.
	 * @param		the number of the corresponding UART interface,
	 * @return		nothing,
	 */
	Port(uint8_t uart_number);
	/**
	 * @brief
	 * @note		nothing needs to be freed.
	 * @param		none,
	 * @return		nothing,
	 */
	virtual ~Port();
	/**
	 * @brief		gets the number of remaining bytes in the rx-buffer, i.e.
	 * 				the bytes received so far.
	 * @param		none,
	 * @return		the number of remaining bytes in the rx-buffer,
	 */
	uint16_t get_available_rx();
	/**
	 * @brief		gets the number of unused bytes in the tx-buffer, i.e. the
	 * 				bytes yet to be added.
	 * @param		none,
	 * @return		the number of unused bytes in the tx-buffer,
	 */
	uint16_t get_available_tx();
	/**
	 * @brief		pops the next byte from the rx-buffer, i.e. the foremost
	 * 				byte received so far.
	 * @param		none,
	 * @return		the next byte,
	 * @retval		#0xFFFF if there is no byte to read,
	 */
	uint16_t read();
	/**
	 * @brief		flushes all the bytes out of the rx-buffer.
	 * @param		none.
	 * @return		nothing,
	 */
	void flush_rx();
	/**
	 * @brief		begins the receival of the first byte.
	 * @note		This should be called only once and before attempting to
	 * 				read the port.
	 * @param		none.
	 * @return		the status,
	 * @retval		#0xFE if the port is already in receiving, i.e. the RX_IDLE
	 * 				state,
	 * 				#0xFF if the port is busy transmitting, i.e. the TX_BUSY
	 * 				state,
	 */
	uint8_t begin_rx();
	/**
	 * @brief		checks and handles the receive-complete interrupt,
	 * @note		This should be called repeatedly within the context of the
	 * 				reading, i.e. the loop.
	 * @param		none.
	 * @return		nothing,
	 */
	void check_rx();
	/**
	 * @brief		pushes the next byte to the tx-buffer, i.e. the next byte
	 * 				to be sent.
	 * @param		the bytes to be pushed to the buffer,
	 * @param		the number of bytes,
	 * @return		the number of bytes pushed,
	 */
	uint16_t write(const uint8_t* data, const uint16_t length);
	/**
	 * @brief		flushes all the bytes out of the tx-buffer, i.e. to send
	 * 				all remaining bytes.
	 * @param		none.
	 * @return		nothing,
	 */
	void flush_tx();
	/**
	 * @brief		checks and handles the transmit-complete interrupt,
	 * @note		This should be called repeatedly within the context of the
	 * 				writing, i.e. the loop.
	 * @param		none.
	 * @return		nothing,
	 */
	void check_tx();
};

/*
extern Port port_1;
extern Port port_2;
extern Port port_3;
extern Port port_4;
extern Port port_5;
extern Port port_6;
*/

#endif /* SRC_PORT_H_ */
