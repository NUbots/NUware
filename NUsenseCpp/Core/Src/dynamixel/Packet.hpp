/*
 * Packet.hpp
 *
 *  Created on: 2 Mar. 2023
 *      Author: Clayton
 */

#include "stdint.h" 	// needed for explicit type-defines
#include <array>		// needed for the array container inside the packet

#ifndef SRC_DYNAMIXEL_PACKET_HPP_
#define SRC_DYNAMIXEL_PACKET_HPP_

namespace dynamixel {

// sshhh ... most of this is stolen and bastardised from the old NUsense code.

enum Instruction {
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    FACTORY_RESET = 0x06,
	REBOOT = 0x08,
	CLEAR = 0x10,
	CONTROL_TABLE_BACKUP = 0x20,
	STATUS_RETURN = 0x55,
    SYNC_READ = 0X82,
    SYNC_WRITE = 0x83,
	FAST_SYNC_READ = 0x8A,
	BULK_READ = 0x92,
	BULK_WRITE = 0x93,
	FAST_BULK_READ = 0x9A
};

enum Error {
	NO_ERROR = 0x00,
    RESULT_FAIL = 0x01,
    INST_ERR = 0x02,
    CRC_ERR = 0x03,
    DATA_RANGE_ERR = 0x04,
    DATA_LEN_ERR = 0x05,
    DATA_LIM_ERR = 0x06,
    ACCESS_ERR = 0x07
};

/*
 * @brief	a generic packet struct,
 * @param	the type of parameters, either a byte or a half-word,
 * @param	the number of such parameters in the packet,
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
template <typename T, uint16_t N>
struct Packet {
	/*
	 * @brief	constructs the packet,
	 * @param	the id of the device concerned,
	 * @param	the instruction,
	 * @param	the parameters,
	 * @return	none
	 */
	Packet(uint8_t id = 0, Instruction instruction = STATUS_RETURN, const std::array<T,N>& params = {}) :
		magic(0x00FDFFFF),
		id(id),
		length(N+3),
		instruction(instruction),
		params(params),
		crc(0xFFFF)
	{
		// If this has an array of half-words as the parameters, then the
		// length needs to be re-calculated.
		if (std::is_same<T,uint16_t>::value)
			length = 2*N + 3;
	}
	// @brief 	This header will not change, except maybe for rsrv which is 0x00
    uint32_t magic;

    // @brief 	This field can be the ID of the sender or the recepient
    uint8_t id;

    // @brief 	Specifies the length of the fields that follow below it
    uint16_t length;

    // @brief 	Code for the instruction to be executed, as stated in the
    //			Instructions enum
    uint8_t instruction;

    // @brief 	This will include the error field for the status packet, or
    //			just the parameter field for the instruction packet
    std::array<T,N> params;

    // @brief 	CRC for the packet - 2 bytes long
    uint16_t crc;
};
#pragma pack(pop)

} /* namespace dynamixel */

#endif /* SRC_DYNAMIXEL_PACKET_HPP_ */
