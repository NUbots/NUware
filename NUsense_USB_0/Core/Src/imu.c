/*
 * Benjamin Young
 * NUfsr IMU source file for basic command sequences.
 *
 * Addendum:
 * Clayton Carlon
 * 9/9/2022
 * Ported from NUfsr to NUsense for testing.
 */

// TODO: Set up a return system for error checking
// TODO: properly comment/document file contents

#include "imu.h"
#include "spi.h"

void NUfsr_IMU_Transmit(uint8_t adr, uint8_t dat, int byte_size)
{
	uint16_t pak = (adr << 8) | dat;
	uint8_t* pak_ptr = (uint8_t*)&pak;

	HAL_SPI_Transmit(&hspi4, pak_ptr, byte_size, HAL_MAX_DELAY);

}

void NUfsr_IMU_TransmitReceive(uint8_t adr, uint8_t dat, uint16_t* dat_return, int byte_size)
{
	uint16_t pak = (adr << 8) | dat;
	uint8_t* pak_ptr = (uint8_t*)&pak;

	HAL_SPI_TransmitReceive(&hspi4, pak_ptr, (uint8_t*)dat_return, byte_size, HAL_MAX_DELAY);
}

void NUfsr_IMU_Init()
{
	// Implement series of commands to configure appropriate settings

	// Reset device: 0x6b, 0x80
	// CC: maybe 0x81?
	//NUfsr_IMU_Transmit(PWR_MGMT_1, 0x80, 2);
	///*
	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x01, 2);
	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x80, 2);
	HAL_Delay(1);
	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x01, 2);


	// Turn off sleep mode:
	//NUfsr_IMU_Transmit(PWR_MGMT_1, 0x00, 2);

	// Ensure we are in SPI mode
	NUfsr_IMU_Transmit(USER_CTRL, 0x1D, 2);

	// CC: Try to enable the sensors.
	NUfsr_IMU_Transmit(PWR_MGMT_2, 0x00, 2);

	// Gyro Config
	NUfsr_IMU_Transmit(GYRO_CONFIG, 0x08, 2);

	// Accel Config
	NUfsr_IMU_Transmit(ACCEL_CONFIG, 0x08, 2);

	// CC: Try to set the LPF of the accelerometer to 218 Hz.
	NUfsr_IMU_Transmit(ACCEL_CONFIG2, 0x00, 2);

	// CC: Try to set the LPF of the gyroscope to 250 Hz.
	NUfsr_IMU_Transmit(CONFIG, 0x00, 2);

	// CC: Try to set the sample-rate to 100 Hz.
	NUfsr_IMU_Transmit(SMPLRT_DIV, 0x00, 2);

	// Int config
	//NUfsr_IMU_Transmit(INT_PIN_CFG, 0x20, 2);

	// Interupt settings
	//NUfsr_IMU_Transmit(INT_ENABLE, 0x01, 2);

	// Reset IMU int status
	//NUfsr_IMU_Transmit(INT_STATUS | IMU_READ, 0x00, 2);
	//*/

	// From NUFSR branch:
	/*
	// Ensure R/W registers are set from power-up
    HAL_Delay(100);

	// Ensure we are in SPI mode
	NUfsr_IMU_Transmit(USER_CTRL, 0x1D, 2);

	// Reset device: 0x6b, 0x80
	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x00, 2);

	// Turn off sleep mode:
	NUfsr_IMU_Transmit(PWR_MGMT_2, 0x00, 2);

	// Config settings
	NUfsr_IMU_Transmit(CONFIG, 0x00, 2);

	// Gyro Config
	NUfsr_IMU_Transmit(GYRO_CONFIG, 0x00, 2);

	// Accel Config
	NUfsr_IMU_Transmit(ACCEL_CONFIG, 0x00, 2);

	// Int config
	NUfsr_IMU_Transmit(INT_PIN_CFG, 0x20, 2);

	// Interupt settings
	NUfsr_IMU_Transmit(INT_ENABLE, 0x01, 2);

	// Reset IMU int status
	NUfsr_IMU_Transmit(INT_STATUS | IMU_READ, 0x00, 2);
	*/

}

