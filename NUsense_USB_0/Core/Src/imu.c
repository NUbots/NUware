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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* handle) {
	if (handle == &hspi4)
		SPI_flag = 1;

}

void NUfsr_IMU_Init()
{
	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);

	// From NUFSR branch:
	// Ensure that R/W registers are set from power-up.
    HAL_Delay(100);

	// Reset the device.
	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x80, 2);
	HAL_Delay(10);

	// Turn off sleep mode.
	NUfsr_IMU_Transmit(PWR_MGMT_1, 0x00, 2);

	// Enable all axes.
	NUfsr_IMU_Transmit(PWR_MGMT_2, 0x00, 2);

	// Ensure that it is in SPI mode.
	NUfsr_IMU_Transmit(USER_CTRL, 0x10, 2);

	// Configure settings.
	NUfsr_IMU_Transmit(CONFIG, 0x00, 2);

	// Gyro Config:
	// 0x00?
	NUfsr_IMU_Transmit(GYRO_CONFIG, 0x08, 2);

	// Accel Config:
	// 0x00?
	NUfsr_IMU_Transmit(ACCEL_CONFIG, 0x08, 2);

	// Int config:
	NUfsr_IMU_Transmit(INT_PIN_CFG, 0x20, 2);

	// Interupt settings:
	NUfsr_IMU_Transmit(INT_ENABLE, 0x01, 2);

	// Reset the IMU interrupt status.
	NUfsr_IMU_Transmit(INT_STATUS | IMU_READ, 0x00, 2);

}

void NUfsr_IMU_BlockingTransmit(uint8_t* dat, int byte_size)
{
	HAL_SPI_Transmit(&hspi4, dat, byte_size, HAL_MAX_DELAY);

}

void NUfsr_IMU_TransmitReceive_IT(uint8_t* dat, uint8_t* dat_return, int byte_size)
{
	HAL_SPI_TransmitReceive_IT(&hspi4, dat, dat_return, byte_size);
}

//Legacy functions:

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
