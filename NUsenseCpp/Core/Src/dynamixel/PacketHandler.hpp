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
 * @brief	the handler for any generic packet (any except bulk-read and
 * 			bulk-write)
 * @param	the type of parameters in the instruction, either a byte or a
 * 			half-word,
 * @param	the number of such parameters in the instruction,
 * @param	the number of parameters as bytes in the expected status, not
 * 			including the error,
 */
template <typename T, uint16_t N, uint16_t M>
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
	 * @param	the instruction-packet to be sent,
	 * @param	the expected number of status-packets in response,
	 */
	PacketHandler(uart::Port& port, const Packet<T,N>& inst_packet, const uint16_t expected_num_sts) :
		port(port),
		packetiser(),
		inst_packet(inst_packet),
		encoded_inst_packet(sizeof(Packet<T,N>)),
		expected_num_sts(expected_num_sts),
		sts_packets(0),
		packet_count(0),
		decoded_crcs(0)
	{
		// Firstly, encode the instruction-packet into a vector with the
		// calculated CRC and any byte-stuffing if need be.
		new (encoded_inst_packet.data()) Packet<T,N>(inst_packet);
		packetiser.encode(encoded_inst_packet);
	};
	/*
	 * @brief	destructs the packet-handler.
	 * @note	nothing needs to be freed as of yet,
	 */
	virtual ~PacketHandler() {};

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
		// If none are expected, e.g. for a sync-write instruction, then return
		// early.
		if (expected_num_sts == 0)
			return SUCCESS;

		// Peek to see if there is a byte on the buffer yet.
		if (port.peek() == NO_BYTE_READ)
			return NONE;
		// If so, then decode it.
		packetiser.decode(port.read());

		// Unless the packetiser has a whole packet, return.
		if (!packetiser.is_packet_ready())
			return NONE;
		// If so, then add the status-packet with the rest.
		sts_packets.push_back(
				*reinterpret_cast<dynamixel::Packet<uint8_t,M+1>*>(
						packetiser.get_decoded_packet()
				)
		);
		// Also, add the decoded CRC, that is the CRC that we calculated so
		// that we may check it later outside this class.
		decoded_crcs.push_back(packetiser.get_decoded_crc());
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
	const Packet<uint8_t,M+1>& get_sts_packet(const uint16_t index) const {
		if (index >= sts_packets.size())
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
	// @brief	the instruction-packet,
	const Packet<T,N> inst_packet;
	// @brief	the encoded instruction-packet as a vector,
	std::vector<uint8_t> encoded_inst_packet;
	// @brief	the expected number of status-packets in response to the
	//			instruction,
	// @note	this is mainly for sync-read, sync-write, etc.
	const uint8_t expected_num_sts;
	// @brief	the status-packets,
	// @note	each status-packet needs to be one byte longer for the error
	//			which is being packed along with the other parameters.
	std::vector<Packet<uint8_t,M+1>> sts_packets;
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
