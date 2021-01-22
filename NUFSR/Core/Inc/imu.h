/***********************************************************/
/*
 * NUbots NUfsr IMU command table
 * Author: Benjamin Young
 * File: Header, uses structs labeled as particular commands
 * 	which will contain the respective register address and an
 * 	enum of data option if the command has a read operation.
 *
 ***********************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _IMU_H_
#define _IMU_H_

/* Includes */
#include "main.h"

/* Common Defines */				/* 7 6 5 4 3 2 1 0 */
#define IMU_READ 0x80;				/* 1 0 0 0 0 0 0 0 -> 0x80 */
#define IMU_WRITE 0x00;				/* 0 0 0 0 0 0 0 0 -> 0x00 */

/* Command Defines */				/* ADR Register Function */
// Self Test
#define IMU_SELF_TEST_X_GYRO 0x00	/* Gyro X Self test Reg */
#define IMU_SELF_TEST_Y_GYRO 0x01	/* Gyro Y Self test Reg */
#define IMU_SELF_TEST_Z_GYRO 0x02	/* Gyro Z Self test Reg */
#define IMU_SELF_TEST_X_ACCEL 0x0D	/* Accel X Self test Reg */
#define IMU_SELF_TEST_Y_ACCEL 0x0E	/* Accel Y Self test Reg */
#define IMU_SELF_TEST_Z_ACCEL 0x0F	/* Accel Z Self test Reg */
// Offset adjustment
#define IMU_XG_OFFS_USRH 0x13		/* Gyro X Offset Adjustment Reg 15:8 */
#define IMU_XG_OFFS_USRL 0x14		/* Gyro X Offset Adjustment Reg 7:0 */
#define IMU_YG_OFFS_USRH 0x15		/* Gyro Y Offset Adjustment Reg 15:8 */
#define IMU_YG_OFFS_USRL 0x16		/* Gyro Y Offset Adjustment Reg 7:0 */
#define IMU_ZG_OFFS_USRH 0x17		/* Gyro Z Offset Adjustment Reg 15:8 */
#define IMU_ZG_OFFS_USRL 0x18		/* Gyro Z Offset Adjustment Reg 7:0 */
// Configuration
#define SMPLRT_DIV 0x19				/* Sample Rate divider */
#define CONFIG 0x1A					/* Configuration */
#define GYRO_CONFIG 0x1B			/* Gyro Configuration */
#define ACCEL_CONFIG 0x1C			/* Accel Configuration */
#define ACCEL_CONFIG2 0x1D			/* Accel Configuration */
#define LP_MODE_CFG 0x1E			/* Low Power Mode Config */
// Interrupt Configuration
#define ACCEL_WOM_X_THR 0x20		/* X-Axis Accel Wake-On Motion Threshold */
#define ACCEL_WOM_Y_THR 0x21		/* Y-Axis Accel Wake-On Motion Threshold */
#define ACCEL_WOM_Z_THR 0x22		/* Z-Axis Accel Wake-On Motion Threshold */
#define FIFO_EN 0x23				/* FIFO Enable */
#define FSYNC_INT 0x36				/* FSYNC Interrupt Status */
#define INT_PIN_CFG 0x37			/* INT/DRDY PIN / BYPASS Enable Config */
#define INT_ENABLE 0x38				/* Interrupt Enable */
#define DMP_INT_STATUS 0x39			/* DMP Interrupt Status */
#define INT_STATUS 0x3A				/* Interrupt Status */
// Accelerometer Measurements
#define ACCEL_XOUT_H 0x3B 			/* X_Accel High byte */
#define ACCEL_XOUT_L 0x3C 			/* X_Accel Low byte */
#define ACCEL_YOUT_H 0x3D 			/* Y_Accel High byte */
#define ACCEL_YOUT_L 0x3E 			/* Y_Accel Low byte */
#define ACCEL_ZOUT_H 0x3F 			/* Z_Accel High byte */
#define ACCEL_ZOUT_L 0x40 			/* Z_Accel Low byte */
// Temperature Measurements
#define TEMP_OUT_H 0x41				/* Temp High Byte */
#define TEMP_OUT_L 0x42				/* Temp Low Byte */
// Gyroscope Measurements
#define GYRO_XOUT_H 0x43 			/* X_GYRO High byte */
#define GYRO_XOUT_L 0x44 			/* X_GYRO Low byte */
#define GYRO_YOUT_H 0x45 			/* Y_GYRO High byte */
#define GYRO_YOUT_L 0x46 			/* Y_GYRO Low byte */
#define GYRO_ZOUT_H 0x47 			/* Z_GYRO High byte */
#define GYRO_ZOUT_L 0x48 			/* Z_GYRO Low byte */
// Additional Options/Configurations
#define SIGNAL_PATH_RESET 0x68 		/* Signal Path Reset */
#define ACCEL_INTEL_CTRL 0x69 		/* Accel Intelligence Control */
#define USER_CTRL 0x6A 				/* User Control */
// Power Management
#define PWR_MGMT_1 0x6B 			/* Power Management 1 */
#define PWR_MGMT_2 0x6C 			/* Power Management 2 */
// FIFO Commands
#define FIFO_COUNTH 0x72			/* High FIFO count byte */
#define FIFO_COUNTL 0x73			/* LOW FIFO count byte */
#define FIFO_R_W 0x74 				/* FIFO buffer Read/Write 8 */
// WHO AM I
#define WHO_AM_I 0x75 				/* Device Information Register */
// ACCEL Offset Reg
#define XA_OFFSET_H 0x77 			/* X_Accel High byte offset calculation */
#define XA_OFFSET_L 0x78 			/* X_Accel Low byte offset calculation */
#define YA_OFFSET_H 0x7A 			/* Y_Accel High byte offset calculation */
#define YA_OFFSET_L 0x7B 			/* Y_Accel Low byte offset calculation */
#define ZA_OFFSET_H 0x7D 			/* Z_Accel High byte offset calculation */
#define ZA_OFFSET_L 0x7E 			/* Z_Accel Low byte offset calculation */

#endif //_IMU_H_


