/*
 * Dynamixel.h
 *
 *  Created on: 22 Feb. 2023
 *      Author: Clayton
 */

#ifndef INC_DYNAMIXEL_H_
#define INC_DYNAMIXEL_H_

#include "stdint.h" 	// needed for explicit type-defines
#include <array>		// needed for the array container inside the packet

namespace dynamixel {

enum Instruction {
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    FACTORY_RESET = 0x06,
    SYNC_READ = 0X82,
    SYNC_WRITE = 0x83
};

enum Error {
    RESULT_FAIL = 0x01,
    INST_ERR = 0x02,
    CRC_ERR = 0x03,
    DATA_RANGE_ERR = 0x04,
    DATA_LEN_ERR = 0x05,
    DATA_LIM_ERR = 0x06,
    ACCESS_ERR = 0x07
};

// sshhh ... most of this is stolen and bastardised from the old NUsense code.

#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
template <typename T, uint16_t N>
struct Packet {
	Packet(uint8_t id, Instruction instruction, const std::array<T,N>& params) :
		magic(0x00FDFFFF),
		id(id),
		length(N+3),
		instruction(instruction),
		params(params),
		crc(0xFFFF)
	{}
	// @brief This header will not change, except maybe for rsrv which is 0x00
    uint32_t magic;

    // @brief This field can be the ID of the sender or the recepient
    uint8_t id;

    // @brief Specifies the length of the fields that follow below it
    uint16_t length;

    // @brief Code for the instruction to be executed, as stated in the Instructions enum
    uint8_t instruction;

    // @brief This will include the error field for the status packet, or just the parameter field for the instruction packet
    std::array<T,N> params;

    // @brief CRC for the packet - 2 bytes long
    uint16_t crc;
};
#pragma pack(pop)

}

#endif /* INC_DYNAMIXEL_H_ */
