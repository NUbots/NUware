#ifndef DYNAMIXEL_REBOOT_HPP
#define DYNAMIXEL_REBOOT_HPP

#ifndef DYNAMIXEL_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

namespace dynamixel {

    /**
     * @brief This struct mimics the expected data structure for a Reboot command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
     *  you cannot add or remove members from this type.
     *
     * @author Alex Biddulph
     */
    struct RebootCommand {
        explicit RebootCommand(uint8_t id)
            : magic(0x00FDFFFF), id(id), length(3), instruction(Instruction::REBOOT), crc(calculate_crc(this)) {}

        /// Magic number that heads up every packet
        const uint32_t magic;
        /// The ID of the device that we are communicating with
        const uint8_t id;
        /// The total length of the data packet (always 3)
        const uint16_t length;
        /// The instruction that we will be executing
        const uint8_t instruction;
        /// Our crc for this command
        const uint16_t crc;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)
    // Check that this struct is not cache alligned
    static_assert(sizeof(RebootCommand) == 10, "The compiler is adding padding to this struct, Bad compiler!");

}  // namespace dynamixel

#endif  // DYNAMIXEL_REBOOT_HPP
