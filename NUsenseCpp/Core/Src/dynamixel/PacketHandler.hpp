/*
 * PacketHandler.hpp
 *
 *  Created on: 27 Feb. 2023
 *      Author: Clayton
 */

#include "Dynamixel.h"
#include "../uart/Port.h"
#include "Packetiser.hpp"

#ifndef SRC_DYNAMIXEL_PACKETHANDLER_HPP_
#define SRC_DYNAMIXEL_PACKETHANDLER_HPP_

namespace dynamixel {

/*
 * @brief	the handler for any generic packet (any except bulk-read and
 * 			bulk-write)
 * @param	the type of parameters in the instruction, either a byte or a
 * 			half-word,
 * @param	the number of such parameters in the instruction,
 * @param	the number of parameters as bytes in the expected status,
 */
template <typename T, uint16_t N, uint16_t M>
class PacketHandler {
public:
	enum Result {
		NONE = 0x00,
		SUCCESS = 0x01,
		TIMEOUT = 0x02
	};
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
		new (encoded_inst_packet.data()) Packet<T,N>(inst_packet);
		packetiser.encode(encoded_inst_packet);
	};
	virtual ~PacketHandler() {};

	void send_inst() {
		port.flush_rx();
		port.write(encoded_inst_packet.data(), encoded_inst_packet.size());
	}

	const Result check_sts() {
		if (port.peek() == NO_BYTE_READ)
			return NONE;
		packetiser.decode(port.read());

		if (!packetiser.is_packet_ready())
			return NONE;
		sts_packets.push_back(
				*reinterpret_cast<dynamixel::Packet<uint8_t,M>*>(
						packetiser.get_decoded_packet()
				)
		);
		decoded_crcs.push_back(packetiser.get_decoded_crc());
		packet_count++;
		packetiser.reset();

		if (packet_count != expected_num_sts)
			return NONE;
		else
			return SUCCESS;
	}

	void reset() {
		packet_count = 0;
		sts_packets.clear();
		decoded_crcs.clear();
	}

	const uint16_t get_num_sts() const {
		return sts_packets.size();
	}

	const Packet<uint8_t,M>& get_sts_packet(const uint16_t index) const {
		if (index >= sts_packets.size())
			return sts_packets.back();
		else
			return sts_packets[index];
	}

	const uint16_t get_crc(const uint16_t index) const {
		if (index >= decoded_crcs.size())
			return decoded_crcs.back();
		else
			return decoded_crcs[index];
	}

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
	std::vector<Packet<uint8_t,M>> sts_packets;
	// @brief	the number of status-packets decoded so far,
	uint16_t packet_count;
	// @brief	the decoded CRCs from the packetiser,
	std::vector<uint16_t> decoded_crcs;
	// @brief	the result
	Result result;
};

} /* namespace dynamixel */

#endif /* SRC_DYNAMIXEL_PACKETHANDLER_HPP_ */
