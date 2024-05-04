#ifndef DYNAMIXEL_SYNCWRITE_HPP
#define DYNAMIXEL_SYNCWRITE_HPP

#ifndef DYNAMIXEL_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

#include <array>
#include <type_traits>

namespace dynamixel {

    /**
     * @brief This struct mimics the expected data structure for a Sync Write command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
     *  you cannot add or remove members from this type. The template argument and parameter allows you to read any
     *  type. For example if you read with a uint16_t then it will read two bytes to the device. And if you use a
     *  struct with 3 uint16_t's in it, then you can directly read a parameter with an x, y and z bytes (e.g. the
     *  accelerometer)
     * @tparam T the type of data to be written
     * @tparam N the number of devices to read from
     *
     * @author Alex Biddulph
     */
    template <typename T>
    struct SyncWriteData {
        static_assert(std::is_trivial<T>::value && std::is_standard_layout<T>::value, "Values must be trivial data");
        SyncWriteData() : id(0), data() {}
        SyncWriteData(uint8_t id, T data) : id(id), data(data) {}

        /// The ID of the device that we are communicating with
        uint8_t id;
        /// The bytes that we are writing
        T data;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

    template <typename T, std::size_t N>
    struct SyncWriteCommand {

        SyncWriteCommand(uint16_t address, const SyncWriteData<T> (&data)[N])
            : magic(0x00FDFFFF)
            , id(0xFE)
            , length(3 + sizeof(address) + sizeof(size) + N * sizeof(data[0]))
            , instruction(Instruction::SYNC_WRITE)
            , address(address)
            , size(sizeof(T))
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
        /// The address to read from
        const uint16_t address;
        /// The number of bytes to read
        const uint16_t size;
        /// List of device IDs to read from
        const SyncWriteData<T> data[N];
        /// Our crc for this command
        const uint16_t crc;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

}  // namespace dynamixel

#endif  // DYNAMIXEL_SYNCWRITE_HPP
