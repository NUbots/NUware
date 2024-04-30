/*
 * Benjamin Young
 * NUfsr IMU source file for basic command sequences.
 *
 */

// TODO: Set up a return system for error checking
// TODO: properly comment/document file contents

#include "imu.h"
#include "spi.h"

//void NUfsr_IMU_Transmit(uint8_t adr, uint8_t dat, int byte_size)
//{
//	uint16_t pak = (adr << 8) | dat;
//	uint8_t* pak_ptr = (uint8_t*)&pak;
//
//	HAL_SPI_Transmit(&hspi1, pak_ptr, byte_size, HAL_MAX_DELAY);
//
//}
//
//void NUfsr_IMU_TransmitReceive(uint8_t adr, uint8_t dat, uint16_t* dat_return, int byte_size)
//{
//	uint16_t pak = (adr << 8) | dat;
//	uint8_t* pak_ptr = (uint8_t*)&pak;
//
//	HAL_SPI_TransmitReceive(&hspi1, pak_ptr, (uint8_t*)dat_return, byte_size, HAL_MAX_DELAY);
//}
//
//void NUfsr_IMU_Init()
//{
//	// Implement series of commands to configure appropriate settings
//
//	// Reset device: 0x6b, 0x80
//	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x80, 2);
//
//	// Turn off sleep mode:
//	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x00, 2);
//
//	// Ensure we are in SPI mode
//	NUfsr_IMU_Transmit(USER_CTRL, 0x10, 2);
//
//	// Gyro Config
//	NUfsr_IMU_Transmit(GYRO_CONFIG, 0x08, 2);
//
//	// Accel Config
//	NUfsr_IMU_Transmit(ACCEL_CONFIG, 0x08, 2);
//
//	// Int config
//	NUfsr_IMU_Transmit(INT_PIN_CFG, 0x20, 2);
//
//	// Interupt settings
//	NUfsr_IMU_Transmit(INT_ENABLE, 0x01, 2);
//
//	// Reset IMU int status
//	NUfsr_IMU_Transmit(INT_STATUS | IMU_READ, 0x00, 2);
//
//}


