#ifndef DYNAMIXEL_BULKREAD_HPP
#define DYNAMIXEL_BULKREAD_HPP

#ifndef DYNAMIXEL_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

#include <array>
#include <type_traits>

namespace dynamixel {
    /**
     * @brief This struct mimics the expected data structure for a Bulk Read command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
     *  you cannot add or remove members from this type. The template argument and parameter allows you to read any
     *  type. For example if you read with a uint16_t then it will read two bytes to the device. And if you use a
     *  struct with 3 uint16_t's in it, then you can directly read a parameter with an x, y and z bytes (e.g. the
     *  accelerometer)
     * @tparam T the type of data to be read from
     * @tparam N the number of devices to read from
     *
     * @author Alex Biddulph
     */
    struct BulkReadData {
        BulkReadData(uint8_t id, uint16_t address, uint16_t size) : id(id), address(address), size(size) {}

        /// The ID of the device that we are communicating with
        uint8_t id;
        /// The starting address to read from
        uint16_t address;
        /// The number of bytes to read
        uint16_t size;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)
    // Check that this struct is not cache aligned
    static_assert(sizeof(BulkReadData) == 5, "The compiler is adding padding to this struct, Bad compiler!");

    template <std::size_t N>
    struct BulkReadCommand {

        BulkReadCommand(const std::array<BulkReadData, N>& data)
            : magic(0x00FDFFFF)
            , id(0xFE)
            , length(3 + N * data[0])
            , instruction(Instruction::BULK_READ)
            , data(data)
            , crc(calculate_crc(this)) {}

        /// Magic number that heads up every packet
        const uint32_t magic;
        /// The ID of the device that we are communicating with
        const uint8_t id;
        /// The total length of the data packet (always 5)
        const uint16_t length;
        /// The instruction that we will be executing
        const uint8_t instruction;
        /// List of device IDs to read from
        const std::array<BulkReadData, N> data;
        /// Our crc for this command
        const uint16_t crc;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

}  // namespace dynamixel

#endif  // DYNAMIXEL_BULKREAD_HPP
