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
#include <string>		// needed for printing the servo-state
#include <ostream>

namespace dynamixel {

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

enum Register {
	TORQUE_ENABLE			 = 64,
	LED						 = 65,
	STATUS_RETURN_LEVEL		 = 68,
	REGISTERED_INSTRUCTION	 = 69,
	HARDWARE_ERROR_STATUS	 = 70,
	VELOCITY_I_GAIN			 = 76,
	VELOCITY_P_GAIN			 = 78,
	POSITION_D_GAIN			 = 80,
	POSITION_I_GAIN			 = 82,
	POSITION_P_GAIN			 = 84,
	FEEDFORWARD_2ND_GAIN	 = 88,
	FEEDFORWARD_1ST_GAIN	 = 90,
	BUS_WATCHDOG			 = 98,
	GOAL_PWM				 = 100,
	GOAL_CURRENT			 = 102,
	GOAL_VELOCITY			 = 104,
	PROFILE_ACCELERATION	 = 108,
	PROFILE_VELOCITY		 = 112,
	GOAL_POSITION			 = 116,
	REALTIME_TICK			 = 120,
	MOVING					 = 122,
	MOVING_STATUS			 = 123,
	PRESENT_PWM				 = 124,
	PRESENT_CURRENT			 = 126,
	PRESENT_VELOCITY		 = 128,
	PRESENT_POSITION		 = 132,
	VELOCITY_TRAJECTORY		 = 136,
	POSITION_TRAJECTORY		 = 140,
	PRESENT_INPUT_VOLTAGE	 = 144,
	PRESENT_TEMPERATURE		 = 146
};

enum Device {
	R_SHOULDER_PITCH	 = 0,
	L_SHOULDER_PITCH	 = 1,
	R_SHOULDER_ROLL 	 = 2,
	L_SHOULDER_ROLL 	 = 3,
	R_ELBOW         	 = 4,
	L_ELBOW         	 = 5,
	R_HIP_YAW       	 = 6,
	L_HIP_YAW       	 = 7,
	R_HIP_ROLL      	 = 8,
	L_HIP_ROLL      	 = 9,
	R_HIP_PITCH     	 = 10,
	L_HIP_PITCH     	 = 11,
	R_KNEE          	 = 12,
	L_KNEE          	 = 13,
	R_ANKLE_PITCH   	 = 14,
	L_ANKLE_PITCH   	 = 15,
	R_ANKLE_ROLL    	 = 16,
	L_ANKLE_ROLL    	 = 17,
	HEAD_YAW        	 = 18,
	HEAD_PITCH      	 = 19,
#ifdef USING_FOOT_SENSORS
	R_FOOT_SENSOR		 = 20,
	L_FOOT_SENSOR		 = 21,
	NUMBER_OF_DEVICES	 = 22,
#else
	NUMBER_OF_DEVICES	 = 20,
#endif
	ALL_DEVICES			 = 0xFE
};

// sshhh ... most of this is stolen and bastardised from the old NUsense code.

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
	Packet(uint8_t id = 0x00, Instruction instruction = STATUS_RETURN, const std::array<T,N>& params = {}) :
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

/*
 * @brief	the grouping of read values in the servo's control-table,
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
struct ReadBank {
	int16_t present_pwm;
	int16_t present_current;
	int32_t present_velocity;
	uint32_t present_position;
	uint16_t present_input_voltage;
	uint8_t  present_temperature;
};
#pragma pack(pop)

/*
 * @brief	the state of a servo
 */
struct ServoState {
	// True if we need to write new values to the hardware
	bool dirty = false;

	// Current error state of the servo
	uint8_t errorFlags = 0;

	// True if we simulate where we think the servos should be
	// Note that we still write the commands to hardware
	bool simulated = false;

	bool torqueEnabled = true;

	// Cached values that are never read
	float velocityPGain       = 0.0f;
	float velocityIGain       = 0.0f;
	float velocityDGain       = 0.0f;
	float positionPGain       = 0.0f;
	float positionIGain       = 0.0f;
	float feedforward1stGain  = 0.0f;
	float feedforward2ndGain  = 0.0f;
	float goalPWM             = 0.0f;
	float goalCurrent         = 0.0f;
	float goalVelocity        = 0.0f;
	float goalPosition        = 0.0f;
	float profileAcceleration = 0.0f;
	float profileVelocity     = 0.0f;

	// Values that are either simulated or read
	float presentPWM      = 0.0f; // %
	float presentCurrent  = 0.0f; // mA
	float presentVelocity = 0.0f; // rpm
	float presentPosition = 0.0f; // deg
	float voltage         = 0.0f; // V
	float temperature     = 0.0f; // deg C

	/*
	 * @brief	converts raw bytes of the read-bank to floating-point numbers
	 * 			in this struct,
	 * @param	the read-bank,
	 */
	void convert_from_read_bank(const ReadBank& read_bank) {
		presentPWM		 = read_bank.present_pwm			 * 0.113;
		presentCurrent	 = read_bank.present_current		 * 3.36;
		presentVelocity	 = read_bank.present_velocity		 * 0.229;
		presentPosition	 = read_bank.present_position		 * 0.088;
		voltage			 = read_bank.present_input_voltage	 * 0.1;
		temperature		 = read_bank.present_temperature	 * 1;
	}
};

/*
 * @brief	outputs the read values to a row,
 * @note	this is mainly used for debugging,
 * @param	the output stream,
 * @param, 	the servo-state,
 * @return,	the output stream,
 */
std::ostream & operator << (std::ostream& out, const ServoState& servo_state) {
	out << "PWM (%) " << servo_state.presentPWM << "\t\t";
	out << "Curr. (mA) " << servo_state.presentCurrent << "\t\t";
	out << "Vel. (rpm) " << servo_state.presentVelocity << "\t\t";
	out << "Pos. (deg) " << servo_state.presentPosition << "\t\t";
	out << "Volt. (V) " << servo_state.voltage << "\t\t";
	out << "Temp. (deg C) " << servo_state.temperature << "\r" << std::endl;
	return out;
}

} // namespace dynamixel

#endif /* INC_DYNAMIXEL_H_ */
