/*
 * PacketHandler.hpp
 *
 *  Created on: 27 Feb. 2023
 *      Author: Clayton
 */

#include "../uart/Port.h"
#include "Devices.hpp"
#include "Packetiser.hpp"
#include "Packet.hpp"

#ifndef SRC_DYNAMIXEL_PACKETHANDLER_HPP_
#define SRC_DYNAMIXEL_PACKETHANDLER_HPP_

namespace dynamixel {

/*
 * @brief	the handler for any generic packet
 */
class PacketHandler {
public:
	// @brief	the result of whether all the status-packets have been received,
	enum Result {
		NONE = 0x00,
		SUCCESS = 0x01,
		TIMEOUT = 0x02
	};
	/*
	 * @brief	constructs the packet-handler.
	 * @param	the reference to the port to be communicated on,
	 * @param	the expected number of status-packets in response,
	 */
	PacketHandler(uart::Port& port, const uint16_t expected_num_sts = 0) :
		port(port),
		packetiser(),
		encoded_inst_packet(7+1+2),
		expected_num_sts(expected_num_sts),
		sts_packets(0),
		packet_count(0),
		decoded_crcs(0),
		result(NONE)
	{

	};
	/*
	 * @brief	destructs the packet-handler.
	 * @note	nothing needs to be freed as of yet,
	 */
	virtual ~PacketHandler() {};

	/*
	 * @brief	encodes the instruction-packet to be sent.
	 * @param	the type of parameters, either a byte or a half-word,
	 * @param	the number of such parameters in the packet,
	 * @param	the packet itself,
	 */
	template <typename T>
	void set_inst(const Packet<T>& inst_packet) {
		// Encode the instruction-packet into a vector with the calculated CRC
		// and any byte-stuffing if need be.
		encoded_inst_packet.resize(inst_packet.length+7);
		//new (encoded_inst_packet.data()) Packet<T>(inst_packet);
		std::copy_n(reinterpret_cast<const uint8_t*>(&inst_packet), 7+1, encoded_inst_packet.data());
		for (int i = 8; i < inst_packet.length+7-2; i++)
			encoded_inst_packet[i] = reinterpret_cast<const uint8_t*>(inst_packet.params.data())[i-8];
		packetiser.encode(encoded_inst_packet);
	}

	/*
	 * @brief	sends the instruction out on the port.
	 * @note	it will also flush the RX of the port.
	 */
	void send_inst() {
		port.flush_rx();
		port.write(encoded_inst_packet.data(), encoded_inst_packet.size());
	}

	/*
	 * @brief 	checks whether all the expected status-packets have been
	 * 			received and decoded.
	 * @retval	#NONE if not all the packets have been decoded,
	 * 			#SUCCESS if all the expected packets have been decoded,
	 */
	const Result check_sts() {
		/*
		 * The code hence to the check for a ready packet is run through very
		 * often and thus leads to delays in main-loop. For optimising, this is
		 * the second candidate to try to trim down. Decoding in the packetiser
		 * is especially burdensome, but it unfortunately cannot be trim down
		 * as much since it needs to parse through the whole packet one byte at
		 * a time because of byte-stuffing. However, I feel as if this code is
		 * already trimmed down as much as possible.
		 */

		// If none are expected, e.g. for a sync-write instruction, then return
		// early.
		if (expected_num_sts == 0)
			return SUCCESS;

		// Peek to see if there is a byte on the buffer yet.
		uint16_t read_result = port.read();
		if (read_result == NO_BYTE_READ)
			return NONE;
		// If so, then decode it.
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_SET);
		packetiser.decode(read_result);
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);

		/*
		 * The code hence is also run through often and is very burdensome. For
		 * optimising, this is the first candidate to try to trim down. Copying
		 * the raw packet into a packet structure is especially burdensome. To
		 * trim this down, a whole rewrite of the packet structure may be
		 * needed, see Packet.h. Thereafter, a rewrite of the for-loops in main
		 * may need to be replaced with recursion so that template parameters
		 * are not a problem.
		 */

		// Unless the packetiser has a whole packet, return.
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_SET);
		if (!packetiser.is_packet_ready()) {
			//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);
			return NONE;
		}
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);
		// If so, then parse the vector as a packet and add it with the rest.
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_SET);
		const auto& decoded_packet = packetiser.get_decoded_packet();
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_SET);
		dynamixel::Packet<uint8_t> received_packet;
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_SET);
		std::copy(
				decoded_packet.begin(),
				std::next(decoded_packet.begin(), 8),
				reinterpret_cast<uint8_t*>(&received_packet)
		);
		std::copy(
				std::next(decoded_packet.begin(), 8),
				std::next(decoded_packet.begin(), packetiser.get_decoded_length()-2),
				received_packet.params.data()
		);
		std::copy(
				std::next(decoded_packet.begin(), packetiser.get_decoded_length()-2),
				std::next(decoded_packet.begin(), packetiser.get_decoded_length()),
				reinterpret_cast<uint8_t*>(&(received_packet.crc))
		);
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);
		sts_packets.push_back(std::move(received_packet));
		// Also, add the decoded CRC, that is the CRC that we calculated so
		// that we may check it later outside this class.
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_SET);
		decoded_crcs.push_back(packetiser.get_decoded_crc());
		//HAL_GPIO_WritePin(SPARE2_GPIO_Port, SPARE2_Pin, GPIO_PIN_RESET);
		// Increase the count and reset the packetiser for the next packet.
		packet_count++;
		packetiser.reset();

		// If we have however many status-packets that we expected, then return
		// with a success. Else, return with none.
		if (packet_count == expected_num_sts)
			return SUCCESS;
		else
			return NONE;
	}

	/*
	 * @brief	resets the handler.
	 * @note	this is mainly used if the handler is going to send the same
	 * 			instruction again, e.g. in a loop.
	 */
	void reset() {
		packetiser.reset();
		sts_packets.clear();
		packet_count = 0;
		decoded_crcs.clear();
	}

	/*
	 * @brief	gets the number of status-packets decoded so far.
	 * @return	returns the number.
	 */
	const uint16_t get_num_sts() const {
		return sts_packets.size();
	}

	/*
	 * @brief	gets the status-packet corresponding to the index.
	 * @param	the index of the status-packet,
	 * @return	the status-packet as an object,
	 */
	const Packet<uint8_t>& get_sts_packet(const uint16_t index) const {
		if (sts_packets.size() == 0)
			// Yes, I know that this is broken. We will fix it later when we
			// optimise all of this.
			return sts_packets[0];
		else if (index >= sts_packets.size())
			return sts_packets.back();
		else
			return sts_packets[index];
	}

	/*
	 * @brief	gets the decoded CRC of the status-packet corresponding to the
	 * 			index.
	 * @param	the index of the status-packet,
	 * @return	two bytes of the decoded CRC,
	 */
	const uint16_t get_crc(const uint16_t index) const {
		if (index >= decoded_crcs.size())
			return decoded_crcs.back();
		else
			return decoded_crcs[index];
	}

	/*
	 * @brief	gets whether the status-packet corresponding to the index is
	 * 			healthy or not.
	 * @param	the index of the status-packet,
	 * @retval	#true if the status-packet is healthy,
	 * 			#false if it is not,
	 */
	const bool is_sts_healthy(const uint16_t index) const {
		if (sts_packets[index].crc == decoded_crcs[index])
			return true;
		else
			return false;
	}

private:
	// @brief	the reference to the port that will be communicated on,
	uart::Port& port;
	// @brief	the packetiser to encode the instruction and to decode the
	//			status,
	dynamixel::Packetiser packetiser;
	// @brief	the encoded instruction-packet as a vector,
	std::vector<uint8_t> encoded_inst_packet;
	// @brief	the expected number of status-packets in response to the
	//			instruction,
	// @note	this is mainly for sync-read, sync-write, etc.
	const uint8_t expected_num_sts;
	// @brief	the status-packets,
	// @note	each status-packet needs to be one byte longer for the error
	//			which is being packed along with the other parameters.
	std::vector<Packet<uint8_t>> sts_packets;
	// @brief	the number of status-packets decoded so far,
	uint16_t packet_count;
	// @brief	the decoded CRCs from the packetiser,
	std::vector<uint16_t> decoded_crcs;
	// @brief	the result
	// @note 	this is not needed yet.
	Result result;
};

} /* namespace dynamixel */

#endif /* SRC_DYNAMIXEL_PACKETHANDLER_HPP_ */
