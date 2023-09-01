/*
 * process_data.hpp
 *
 *  Created on: Aug 25, 2023
 *      Author: clayton
 */

#ifndef SRC_PLATFORM_NUSENSE_PROCESS_DATA_HPP_
#define SRC_PLATFORM_NUSENSE_PROCESS_DATA_HPP_

#include <array>
#include "../../dynamixel/Dynamixel.hpp"
#include "NUgus.hpp"
#include "../ServoState.hpp"

namespace platform::NUsense {

    void process_servo_data(
        std::array<platform::ServoState,NUMBER_OF_DEVICES>& servo_states,
        const dynamixel::StatusReturnCommand<sizeof(DynamixelServoReadData)> packet
    );

}



#endif /* SRC_PLATFORM_NUSENSE_PROCESS_DATA_HPP_ */
