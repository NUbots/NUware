/*
 * Port.cpp
 *
 *  Created on: 18 Jan. 2023
 *      Author: Clayton
 */

#include "Port.h"

/* An alternative way to handle interrupts rather than doing it within the main
 * loop is to make global variables and call the handlers explicitly in the
 * callback-functions. This is how Robotis did it, but I think that it is
 * inefficient and can cause bugs especially when a C++ container class is used
 * instead of a struct.
 */
/*
Port port_1(1);
Port port_2(2);
Port port_3(3);
Port port_4(4);
Port port_5(5);
Port port_6(6);
*/
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* handle) {
	if 		(handle == &huart1) port_1.handle_rx();
	else if (handle == &huart2) port_2.handle_rx();
	else if (handle == &huart3) port_3.handle_rx();
	else if (handle == &huart4) port_4.handle_rx();
	else if (handle == &huart5) port_5.handle_rx();
	else if (handle == &huart6) port_6.handle_rx();
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* handle) {
	if 		(handle == &huart1) port_1.handle_tx();
	else if (handle == &huart2) port_2.handle_tx();
	else if (handle == &huart3) port_3.handle_tx();
	else if (handle == &huart4) port_4.handle_tx();
	else if (handle == &huart5) port_5.handle_tx();
	else if (handle == &huart6) port_6.handle_tx();
}
*/

Port::Port(uint8_t uart_number = 1) :
	rs_link(uart_number),
	num_bytes_tx(0)
{

}

Port::~Port() {
	// May do something with this later.
	// Not really anything to free as of yet.
}

uint16_t Port::get_available_rx() {
#ifdef USE_QUEUE_CLASS
	return rx_buffer.size();
#else
	return rx_buffer.size;
#endif
}

uint16_t Port::get_available_tx() {
#ifdef USE_QUEUE_CLASS
	return PORT_BUFFER_SIZE - tx_buffer.size();
#else
	return PORT_BUFFER_SIZE - tx_buffer.size;
#endif
}

uint16_t Port::read() {
	uint8_t read_byte;

	// If there is no byte to read, then return 0xFF as a value two bytes long
	// to not be confused with a received byte.
	if (!get_available_rx())
		return 0xFF;
#ifdef USE_QUEUE_CLASS
	// Read from the front of the buffer
	read_byte = rx_buffer.front();
	rx_buffer.pop_front();
#else
	// Read from the front of the buffer.
	read_byte = rx_buffer.pop();
#endif
	return read_byte;
}

void Port::flush_rx() {
#ifdef USE_QUEUE_CLASS
	rx_buffer.clear();
#else
	// Just reset the buffer.
	rx_buffer.front = rx_buffer.back;
	rx_buffer.size = 0;
#endif
}

uint8_t Port::begin_rx() {
	// Begin the receival of a byte only if there is no transmission.
	if (comm_state != TX_BUSY) {
		comm_state = RX_IDLE;
		return (uint8_t)rs_link.receive_it(&received_byte, 1);
	}
	else
		return 0xFF;
}

void Port::handle_rx() {

	// If the buffer is not full yet, then add the received byte to it.
#ifdef USE_QUEUE_CLASS
	if (get_available_rx() < PORT_BUFFER_SIZE)
		rx_buffer.push_back(received_byte);
#else
	if (get_available_rx() < (PORT_BUFFER_SIZE)) {
		rx_buffer.push(received_byte);
	}
#endif
	// Begin the receiving again for the next byte.
	// Even if the buffer is full, one would still want to receive the next
	// byte in case that the buffer frees up until then.
	begin_rx();
}

void Port::check_rx() {
	// If there has been a receival, then handle it.
	if (rs_link.get_receive_flag())
		handle_rx();
}

uint16_t Port::write(const uint8_t* data, const uint16_t length) {
	// For each byte, try to add it to the buffer.
	for (int i = 0; i < length; i++) {
		// If it is full, then begin a transmission if not already begun.
		if (!get_available_tx()) {
			while (!num_bytes_tx)
				begin_tx();
			// Wait for a spot in the buffer to show up.
			while (!get_available_tx())
				/* This has to be done inside this loop lest it freezes here.
				 * The alternative to this is to do what Robotis did in their
				 * UARTClass, that is to do the handling inside the
				 * callback-function itself. However, I think that this is
				 * inefficient and has caused bugs when I have tested it.
				 */
				check_tx();
		}
		// Add the byte once there is room.
#ifdef USE_QUEUE_CLASS
		tx_buffer.push_back(data[i]);
#else
		tx_buffer.push(data[i]);
#endif
	}
	// Begin a transmission if not already begun.
	if (!num_bytes_tx) {
		// Keep trying if there is an error.
		while (begin_tx());
	}
	return length;
}

void Port::flush_tx() {
	// Wait until there are no bytes transmitting.
	//while (!num_bytes_tx);
	while (comm_state == TX_BUSY);
}

uint8_t Port::begin_tx() {
	RS485::status status = RS485::RS485_OK;

	// Transmit the bytes after you have calculated how many are to be
	// transmitted.
#ifdef USE_QUEUE_CLASS
	num_bytes_tx = tx_buffer.size();
	bytes_tx = tx_buffer;
	// If there is an error, etc., then set the number of bytes to zero.
	if (RS485::RS485_OK != rs_link.transmit_it((uint8_t*)bytes_tx.data(), num_bytes_tx)) {
		num_bytes_tx = 0;
		comm_state = TX_DONE;
	} else
		comm_state = TX_BUSY;
#else
	// Only send from the front to the end of the actual array.
	// This is less efficient than using a C++ queue.
	num_bytes_tx = tx_buffer.back >= tx_buffer.front ?
			tx_buffer.size
			: PORT_BUFFER_SIZE - tx_buffer.front;
	// If there is an error, etc., then set the number of bytes to zero to say
	// than none have been sent.
	status = rs_link.transmit_it(&tx_buffer.data[tx_buffer.front], num_bytes_tx);
	if (RS485::RS485_OK != status) {
		num_bytes_tx = 0;
		comm_state = TX_DONE;
	} else
		comm_state = TX_BUSY;
#endif

#ifdef SEE_STATISTICS
	for (int i = 0; i < 10-1; i++)
		old_num_bytes_tx[i] = old_num_bytes_tx[i+1];
	old_num_bytes_tx[9] = num_bytes_tx;
#endif

	return status;
}

void Port::handle_tx() {
#ifdef USE_QUEUE_CLASS
	uint16_t nietzsche = 0;
	// Remove the bytes that were just transmitted from the front of the buffer.
	tx_buffer.erase(tx_buffer.begin(),tx_buffer.begin()+num_bytes_tx);
	nietzsche = tx_buffer.size();
	// Send any remaining bytes.
	if (tx_buffer.size() != 0)
		begin_tx();
	else
		num_bytes_tx = 0;
#else
	// Move the front further backwards.
	// Make sure not to move further past the back if all the bytes in the
	// buffer were sent.
	tx_buffer.front = tx_buffer.size != num_bytes_tx?
			(tx_buffer.front + num_bytes_tx) % PORT_BUFFER_SIZE
			// If all bytes were sent, then subtract one from the formula to
			// account for front being moved to the back.
			: (tx_buffer.front + num_bytes_tx  - 1) % PORT_BUFFER_SIZE;
	// Update the size.
	tx_buffer.size -= num_bytes_tx;
	// Send any remaining bytes.
	if (tx_buffer.size)
		begin_tx();
	// If not, then begin receiving again.
	/* I am not sure about the functionality of this. It may be better to not
	 * call begin_rx and instead give the user the agency to begin the receival
	 * in their code. But it should not hurt to listen for bytes in the
	 * meantime whilst there is no transmission. It does however make TX_DONE
	 * useless as a state. May change this functionality later.
	 */
	else {
		num_bytes_tx = 0;
		comm_state = TX_DONE;
		begin_rx();
	}
#endif
}

void Port::check_tx() {
	// If the transmission has been done, then handle it.
	if (rs_link.get_transmit_flag())
		handle_tx();
}
