//
// Author: Johanne Montano on 21/02/2023.
//

#ifndef DMXLV2_PACKET_H
#define DMXLV2_PACKET_H

#include <cstdint>
enum Instructions
{
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    FACTORY_RESET = 0x06,
    SYNC_READ = 0X82,
    SYNC_WRITE = 0x83
};

enum Errors
{
    RESULT_FAIL = 0x01,
    INST_ERR = 0x02,
    CRC_ERR = 0x03,
    DATA_RANGE_ERR = 0x04,
    DATA_LEN_ERR = 0x05,
    DATA_LIM_ERR = 0x06,
    ACCESS_ERR = 0x07
};

// @brief The generalised packet structure for the Dynamixel V2 protocol
struct Packet
{

    // @brief This header will not change, except maybe for rsrv which is 0x00
    const uint8_t header[4] = {0x00, 0xFD, 0xFF, 0xFF};

    // @brief This field can be the ID of the sender or the recepient
    uint8_t id;

    // @brief Specifies the length of the fields that follow below it
    uint16_t length;

    // @brief Code for the instruction to be executed, as stated in the Instructions enum
    uint8_t instruction;

    // @brief This will include the error field for the status packet, or just the parameter field for the instruction packet
    uint8_t *parameter;

    // @brief CRC for the packet - 2 bytes long
    uint16_t crc;

private:
    // @brief Encode the packet to be sent to the motors
    // @param
    uint8_t *encode();

    // @brief Decode the packet received from the motors
    // @param
    Packet decode(uint8_t *packet);

    // @brief Calculate the CRC for the packet (function stolen from robotis)
    uint16_t calculate_CRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
};

#endif //DMXLV2_PACKET_H
