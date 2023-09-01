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
#include <ostream>		// needed for outputting the servo-state
#include <iomanip>		// needed to make the output stream nicer
#include "../uart/Port.hpp"

namespace dynamixel {

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
	PRESENT_TEMPERATURE		 = 146,
	INDIRECT_ADDRESS_1		 = 168,
	// ...
	INDIRECT_DATA_1			 = 224,
	// ...
	INDIRECT_ADDRESS_29		 = 578,
	// ...
	INDIRECT_DATA_29		 = 634
	// ...
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

/*#include <cstdint>
 * @brief	the grouping of read values in the servo's control-table,
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
struct ReadBank {
	uint8_t torque_enable;
	uint8_t hardware_error_status;
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
		torqueEnabled	 = read_bank.torque_enable;
		errorFlags		 = read_bank.hardware_error_status;
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
	out << "Torque En. " << std::setw(1)
			<< servo_state.torqueEnabled << "\t";
	out << "Err. Flags 0x" << std::setfill('0') << std::setw(4) << std::hex
			<< (uint16_t)servo_state.errorFlags << "\t" << std::setfill(' ');
	out << "PWM (%) " << std::fixed << std::setw(6) << std::setprecision(2)
			<< servo_state.presentPWM << "\t";
	out << "Curr. (mA) " << std::fixed << std::setw(6) << std::setprecision(2)
			<< servo_state.presentCurrent << "\t";
	out << "Vel. (rpm) " << std::fixed << std::setw(8) << std::setprecision(2)
			<< servo_state.presentVelocity << "\t";
	out << "Pos. (deg) " << std::fixed << std::setw(6) << std::setprecision(2)
			<< servo_state.presentPosition << "\t";
	out << "Volt. (V) " << std::fixed << std::setw(6) << std::setprecision(2)
			<< servo_state.voltage << "\t";
	out << "Temp. (deg C) " << std::fixed << std::setw(6) << std::setprecision(2)
			<< servo_state.temperature << "\r" << std::endl;
	return out;
}

/*
 * Helper functions:
 */

/*
 * @brief 	makes the parameters for a sync-write instruction that sets the
 * 			indirect addresses to a set of servos on one port.
 * @note	the purpose of this helper-function is to wrap the rather complex
 * 			packing for the sync-write instruction for the indirect addresses
 * 			and to genericise this for any number of devices.
 * @param	an array of the servo-ids,
 * @return	the parameters as an array,
 */
const std::vector<uint8_t> make_sync_write_params(const std::vector<Device>& devices) {
	// List all the addresses that will be written to the indirect addresses.
	/*
	 * Note that these are not the addresses that are being written to in this
	 * sync-write instruction, rather they are the values that are written to
	 * the space for indirect addresses in the RAM. Later, we will read and
	 * write from the indirect registers that correspond to these addresses.
	 */
	const uint16_t read_bank_length = 17; // number of half-words, not bytes,
	const std::array<uint16_t,read_bank_length> read_bank_addresses = {
		  dynamixel::TORQUE_ENABLE,
		  dynamixel::HARDWARE_ERROR_STATUS,
		  dynamixel::PRESENT_PWM,
			  dynamixel::PRESENT_PWM+1,
		  dynamixel::PRESENT_CURRENT,
			  dynamixel::PRESENT_CURRENT+1,
		  dynamixel::PRESENT_VELOCITY,
			  dynamixel::PRESENT_VELOCITY+1,
			  dynamixel::PRESENT_VELOCITY+2,
			  dynamixel::PRESENT_VELOCITY+3,
		  dynamixel::PRESENT_POSITION,
			  dynamixel::PRESENT_POSITION+1,
			  dynamixel::PRESENT_POSITION+2,
			  dynamixel::PRESENT_POSITION+3,
		  dynamixel::PRESENT_INPUT_VOLTAGE,
			  dynamixel::PRESENT_INPUT_VOLTAGE+1,
		  dynamixel::PRESENT_TEMPERATURE
	};

	// Pack these addresses as parameters according to the sync-write
	// instruction.
	/*
	 * L starting address
	 * H starting address
	 * L data-length
	 * H data-length
	 * 1st ID
	 * ... bytes ...
	 * 2nd ID
	 * ... bytes ...
	 * ...
	 */
	// The parameters have two bytes for the starting address, two for the
	// data-length, one byte for each ID and however many bytes for each servo.
	// Note that the parameters have to be packed as bytes instead of
	// half-words because the ID is only a byte.
	const uint16_t params_length = 2+2+devices.size()*(1+2*read_bank_length);
	std::vector<uint8_t> params(params_length);
	// The starting-address, which is 0x00A8 and is where the indirect
	// The beginning address:
	params[0] = INDIRECT_ADDRESS_1 & 0xFF;
	params[1] = (INDIRECT_ADDRESS_1 >> 8) & 0xFF;
	// The data-length:
	params[2] = (2*read_bank_length) & 0x00FF;
	params[3] = (2*read_bank_length >> 8) & 0x00FF;
	// For each device, fill the array of parameters with the ID and the
	// indirect addresses.
	for (uint8_t i = 0; i < devices.size(); i++) {
		// The base of the array for the i-th device:
		const uint16_t base = 4 + i*(1+2*read_bank_length);
		// Add the ID.
		params[base] = devices[i] + 1; // offset by one
		// Add all the indirect addresses.
		for (int j = 0; j < read_bank_length; j++) {
			params[base+2*j+1] = read_bank_addresses[j] & 0x00FF;
			params[base+2*j+2] = (read_bank_addresses[j] >> 8) & 0x00FF;
		}
	}

	return params;
}


} // namespace dynamixel

#endif /* INC_DYNAMIXEL_H_ */
