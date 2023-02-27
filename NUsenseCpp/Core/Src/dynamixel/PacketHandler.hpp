/*
 * PacketHandler.hpp
 *
 *  Created on: 27 Feb. 2023
 *      Author: Clayton
 */

#include "Dynamixel.h"
#include "Port.h"
#include "Packetiser.hpp"

#ifndef SRC_DYNAMIXEL_PACKETHANDLER_HPP_
#define SRC_DYNAMIXEL_PACKETHANDLER_HPP_

namespace dynamixel {

template <typename T, uint16_t N, uint16_t M>
class PacketHandler {
public:
	enum Result {
		NONE,
		SUCCESS,
		CRC_ERROR,
		TIMEOUT
	};

	PacketHandler(uart::Port& port, const Packet<T,N>& inst_packet, const uint16_t expected_num_sts) :
		port(port),
		packetiser(),
		inst_packet(inst_packet),
		encoded_inst_packet(sizeof(Packet<T,N>)),
		num_params(N),
		expected_num_sts(expected_num_sts),
		sts_packets(NUMBER_OF_DEVICES),
		decoded_crcs(NUMBER_OF_DEVICES)
	{

	};
	virtual ~PacketHandler() {};

	void send_inst() {
		encoded_inst_packet.data() = &inst_packet;
		packetiser.encode(encoded_inst_packet);
		port.flush_rx();
		port.write(encoded_inst_packet.data(), encoded_inst_packet.size());
	}

	const bool check_sts() {
		static uint16_t packet_count = 0;

		if (port.peek() == NO_BYTE_READ)
			return false;
		packetiser.decode(port.read());

		if (!packetiser.is_packet_ready())
			return false;
		sts_packets.push_back(
				*reinterpret_cast<dynamixel::Packet<uint8_t,M>*>(
						packetiser.get_decoded_packet()
				)
		);
		decoded_crcs.push_back(packetiser.get_decoded_crc());
		packet_count++;

		if (packet_count != expected_num_sts)
			return false;
		else
			return true;
	}

	const uint16_t get_num_sts() {
		return sts_packets.size();
	}

private:
	uart::Port& port;
	dynamixel::Packetiser packetiser;
	const Packet<T,N> inst_packet;
	std::vector<uint8_t> encoded_inst_packet;
	uint8_t num_params;
	uint8_t expected_num_sts;
	std::vector<Packet<uint8_t,M>> sts_packets;
	std::vector<uint16_t> decoded_crcs;
	Result result;
};

} /* namespace dynamixel */

#endif /* SRC_DYNAMIXEL_PACKETHANDLER_HPP_ */
