/*
 * Benjamin Young
 * NUfsr IMU source file for basic command sequences.
 *
 * Addendum:
 * Clayton Carlon
 * 9/9/2022
 * Ported from NUfsr to NUsense for testing.
 */

#include "imu.h"

/*
 * Brief:		begins the IMU for simple polling.
 * Note:
 * Arguments:	none
 * Returns:		none
 */
void NU_IMU_Init()
{
	// Select the first PLL as the clock and do a soft reset.
	// This may fix the problem of the power-up sequence in Section-4.19 of the IMU's datasheet.
	NU_IMU_WriteReg(PWR_MGMT_1, 	PWR_MGMT_1_CLKSEL_AUTO);
	NU_IMU_WriteReg(PWR_MGMT_1, 	PWR_MGMT_1_DEVICE_RESET | PWR_MGMT_1_CLKSEL_AUTO);
	HAL_Delay(1);
	NU_IMU_WriteReg(PWR_MGMT_1, 	PWR_MGMT_1_CLKSEL_AUTO);

	// Make sure that we are in SPI-mode.
	NU_IMU_WriteReg(USER_CTRL, 		USER_CTRL_I2C_IF_DIS | USER_CTRL_DMP_RST
									| USER_CTRL_FIFO_RST | USER_CTRL_SIG_COND_RST);

	// Turn on all sensors.
	NU_IMU_WriteReg(PWR_MGMT_2, 	0x00);

	// Set the full-scale for the gyroscope.
	NU_IMU_WriteReg(GYRO_CONFIG, 	GYRO_CONFIG_FS_SEL_CHOSEN);

	// Set the full-scale for the accelerometer.
	NU_IMU_WriteReg(ACCEL_CONFIG, 	ACCEL_CONFIG1_FS_SEL_CHOSEN);

	// Set the accelerometer's LPF to 218 Hz and the lowest number of samples.
	NU_IMU_WriteReg(ACCEL_CONFIG2, 	ACCEL_CONFIG2_DEC2_CFG_4SAMPLES
									| ACCEL_CONFIG2_ACCEL_FCHOICE_B_FALSE
									| 0x00);

	// Set the gyroscope's LPF to 250 Hz.
	NU_IMU_WriteReg(CONFIG, 		0x00);

	// Set the sample-rate to 1 kHz.
	NU_IMU_WriteReg(SMPLRT_DIV, 	0x00);

	// Set the offset for gyroscope's x-axis to 90/4 = 22.
	NU_IMU_WriteReg(XG_OFFS_USRH, 	0x00);
	NU_IMU_WriteReg(XG_OFFS_USRL, 	0x16);

	// Set the offset for gyroscope's y-axis to -406/4 = -101.
	NU_IMU_WriteReg(YG_OFFS_USRH, 	0xFF);
	NU_IMU_WriteReg(YG_OFFS_USRL, 	0x9B);

	// Set the offset for gyroscope's z-axis to -61/4 = -15.
	NU_IMU_WriteReg(ZG_OFFS_USRH, 	0xFF);
	NU_IMU_WriteReg(ZG_OFFS_USRL, 	0xF1);
	/*
	// Set the offset for accelerometer's x-axis to 0, for now.
	NU_IMU_WriteReg(XA_OFFSET_H, 	0x00);
	NU_IMU_WriteReg(XA_OFFSET_L, 	0x02);

	// Set the offset for accelerometer's y-axis to 0, for now.
	NU_IMU_WriteReg(YA_OFFSET_H, 	0x00);
	NU_IMU_WriteReg(YA_OFFSET_L, 	0x02);

	// Set the offset for accelerometer's z-axis to 0, for now.
	NU_IMU_WriteReg(ZA_OFFSET_H, 	0x00);
	NU_IMU_WriteReg(ZA_OFFSET_L, 	0x02);
	*/

	/*
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
	*/

}

/*
 * Brief:		writes a byte to a register.
 * Note:		uses polling, should only be used for beginning.
 * Arguments:	the register's address,
 * 				the byte to be sent,
 * Returns:		none
 */
void NU_IMU_WriteReg(uint8_t addr, uint8_t data)
{
	uint8_t packet[2] = {addr | IMU_WRITE, data};

	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, packet, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET);
}

/*
 * Brief:		reads a byte from a register.
 * Note:		uses polling, should only be used for testing and debugging.
 * Arguments:	the register's address,
 * 				a pointer to the byte to be read,
 * Returns:		none
 */
void NU_IMU_ReadReg(uint8_t addr, uint8_t* data)
{
	uint8_t rx_data[2] = {0xFF, 0xFF};
	uint8_t packet[2] = {addr | IMU_READ, 0x00};

	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi4, packet, rx_data, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET);

	*data = rx_data[1];
}

/*
 * Brief:		reads multiple registers in a burst.
 * Note:		Do not use yet! Burst-reading seems not to work with this particular IMU chip.
 * 				Wait for a better function using the FIFO and DMA.
 * Arguments:	an array of the registers' addresses in order to be read,
 * 				an array of the bytes to be read, the first of which is padding,
 * 				the length, i.e. the number of registers to be read,
 * Returns:		none
 */
void NU_IMU_ReadBurst(uint8_t* addrs, uint8_t* data, uint16_t length)
{
	uint8_t packet[length+1];

	for (int i = 0; i < length+1; i++) {
		data[i] = 0xAA;
		if (i < length)
			packet[i] = addrs[i] | IMU_READ;
		else
			packet[i] = 0x00;
	}

	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi4, packet, data, length+1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET);
}

/*
 * Brief:		reads multiple registers in turns as a temporary solution.
 * Note:		Use this instead! yes, it is slow and inefficient, but it works.
 * 				Wait for a better function using the FIFO and DMA.
 * Arguments:	an array of the registers' addresses in order to be read,
 * 				an array of the bytes to be read, the first of which is padding,
 * 				the length, i.e. the number of registers to be read,
 * Returns:		none
 */
void NU_IMU_ReadSlowly(uint8_t* addrs, uint8_t* data, uint16_t length)
{
	uint8_t rx_data[2] = {0xFF, 0xFF};
	uint8_t packet[2] = {0xFF, 0x00};

	for (int i = 0; i < length; i++) {
		packet[0] = addrs[i] | IMU_READ;
		HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi4, packet, rx_data, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_SET);
		data[i+1] = rx_data[1];
	}
}

/*
 * Brief:		converts raw integers into floating decimals.
 * Note:		accelerometer values are in g's, and gyroscope values are in dps.
 * Arguments:	the raw data to be converted from,
 * 				the converted data,
 * Returns:		none
 */
void NU_IMU_ConvertRawData(struct IMURawData* raw_data, struct IMUConvertedData* converted_data) {
	/*
	converted_data->ID = raw_data->ID;
	converted_data->accelerometer.x = (float)(raw_data->accelerometer.x)*4.0/32767.0;
	converted_data->accelerometer.y = (float)(raw_data->accelerometer.y)*4.0/32767.0;
	converted_data->accelerometer.z = (float)(raw_data->accelerometer.z)*4.0/32767.0;
	converted_data->temperature = (float)raw_data->temperature/100.0;
	converted_data->gyroscope.x = (float)(raw_data->gyroscope.x)*500.0/32767.0;
	converted_data->gyroscope.y = (float)(raw_data->gyroscope.y)*500.0/32767.0;
	converted_data->gyroscope.z = (float)(raw_data->gyroscope.z)*500.0/32767.0;
	*/

	converted_data->ID = raw_data->ID;
	converted_data->accelerometer.x = (float)(raw_data->accelerometer.x)/ACCEL_SENSITIVTY_4G;
	converted_data->accelerometer.y = (float)(raw_data->accelerometer.y)/ACCEL_SENSITIVTY_4G;
	converted_data->accelerometer.z = (float)(raw_data->accelerometer.z)/ACCEL_SENSITIVTY_4G;
	// from Section-11.23 from the datasheet
	converted_data->temperature = ((float)raw_data->temperature-ROOM_TEMP_OFFSET)/TEMP_SENSITIVITY + 25.0;
	converted_data->gyroscope.x = (float)(raw_data->gyroscope.x)/GYRO_SENSITIVITY_500DPS;
	converted_data->gyroscope.y = (float)(raw_data->gyroscope.y)/GYRO_SENSITIVITY_500DPS;
	converted_data->gyroscope.z = (float)(raw_data->gyroscope.z)/GYRO_SENSITIVITY_500DPS;

}

/*
 * Brief:		used for blocking to get the next byte from the IMU.
 * Note:		may not be needed strictly.
 * Arguments:	the data to be sent,
 * 				the number of bytes,
 * Returns:		none
 */
void NU_IMU_TransmitReceive_IT(uint8_t* tx_data, uint8_t* rx_data, uint16_t length)
{
	HAL_GPIO_WritePin(MPU_NSS_GPIO_Port, MPU_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_IT(&hspi4, tx_data, rx_data, length);
}

/*
 * Brief:		used for blocking to get the next byte from the IMU.
 * Note:		may not be needed strictly.
 * Arguments:	the data
 * Returns:		none
 */
void NU_IMU_BlockingTransmit(uint8_t* data, uint16_t length)
{
	HAL_SPI_Transmit(&hspi4, data, length, HAL_MAX_DELAY);

}

//Legacy functions (fixed):

void NU_IMU_Transmit(uint8_t adr, uint8_t dat, int byte_size)
{
	uint8_t pak[2] = {adr, dat};

	HAL_SPI_Transmit(&hspi4, pak, byte_size, HAL_MAX_DELAY);

}

void NU_IMU_TransmitReceive(uint8_t adr, uint8_t dat, uint8_t* dat_return, int byte_size)
{
	uint8_t pak[2] = {adr, dat};

	HAL_SPI_TransmitReceive(&hspi4, pak, dat_return, byte_size, HAL_MAX_DELAY);
}
