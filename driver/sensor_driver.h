/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Bosch BME280 I2C Environmental Sensor Driver - Register Map & Data Structures
 *
 * Copyright (c) 2026 Nikunj Malik
 *
 * Hardware Truth Reference:
 * Bosch Sensortec BME280 Datasheet (BST-BME280-DS002-15)
 */

#ifndef _BME280_CUSTOM_SENSOR_DRIVER_H_
#define _BME280_CUSTOM_SENSOR_DRIVER_H_

#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/mutex.h>

#define BME280_DRIVER_NAME		"bme280_custom"
#define BME280_I2C_ADDR_PRIMARY		0x76
#define BME280_I2C_ADDR_SECONDARY	0x77

/* Identification and Reset Registers */
#define BME280_REG_CHIP_ID		0xD0
#define BME280_CHIP_ID_VAL		0x60
#define BMP280_CHIP_ID_VAL		0x58

#define BME280_REG_RESET		0xE0
#define BME280_RESET_VAL		0xB6

/* Control and Status Registers */
#define BME280_REG_CTRL_HUM		0xF2
#define BME280_REG_STATUS		0xF3
#define BME280_REG_CTRL_MEAS		0xF4
#define BME280_REG_CONFIG		0xF5

/* Status Register Bit Masks */
#define BME280_STATUS_MEASURING		BIT(3)
#define BME280_STATUS_IM_UPDATE		BIT(0)

/* Measurement Data Burst Registers (0xF7 through 0xFE = 8 bytes) */
#define BME280_REG_PRESS_MSB		0xF7
#define BME280_REG_PRESS_LSB		0xF8
#define BME280_REG_PRESS_XLSB		0xF9
#define BME280_REG_TEMP_MSB		0xFA
#define BME280_REG_TEMP_LSB		0xFB
#define BME280_REG_TEMP_XLSB		0xFC
#define BME280_REG_HUM_MSB		0xFD
#define BME280_REG_HUM_LSB		0xFE
#define BME280_BURST_READ_LEN		8

/* Uncompensated Power-On-Reset Sentinel Values (Sensor in Sleep / Uninitialized) */
#define BME280_ADC_TEMP_SKIPPED		0x80000
#define BME280_ADC_PRESS_SKIPPED	0x80000
#define BME280_ADC_HUM_SKIPPED		0x8000

/* Calibration Trimming Parameter Register Banks */
#define BME280_REG_CALIB_TEMP_PRESS_START	0x88
#define BME280_CALIB_TEMP_PRESS_LEN		24	/* 0x88..0x9F */
#define BME280_REG_CALIB_DIG_H1			0xA1
#define BME280_REG_CALIB_HUM_START		0xE1
#define BME280_CALIB_HUM_LEN			7	/* 0xE1..0xE7 */

/* Oversampling Settings (osrs_t, osrs_p, osrs_h) */
#define BME280_OSRS_SKIPPED		0x00
#define BME280_OSRS_1X			0x01
#define BME280_OSRS_2X			0x02
#define BME280_OSRS_4X			0x03
#define BME280_OSRS_8X			0x04
#define BME280_OSRS_16X			0x05

/* Sensor Operating Modes (mode[1:0] in 0xF4) */
#define BME280_MODE_SLEEP		0x00
#define BME280_MODE_FORCED		0x01
#define BME280_MODE_NORMAL		0x03
#define BME280_MODE_MASK		0x03

/*
 * Standby Time (t_sb[2:0] in 0xF5 bits 7:5) - Datasheet Table 27
 * Determines the hardware continuous cycling rate in Normal Mode:
 *   T_cycle = T_measure (~37.5 ms at 2x T, 16x P, 1x H) + t_standby
 *
 *   t_sb=0x05 (1000.0 ms) -> ~1 Hz
 *   t_sb=0x04 ( 500.0 ms) -> ~2 Hz
 *   t_sb=0x03 ( 250.0 ms) -> ~4 Hz
 *   t_sb=0x02 ( 125.0 ms) -> ~8 Hz
 *   t_sb=0x01 (  62.5 ms) -> ~16 Hz
 *   t_sb=0x00 (   0.5 ms) -> ~26 Hz
 */
#define BME280_STANDBY_0_5_MS		0x00
#define BME280_STANDBY_62_5_MS		0x01
#define BME280_STANDBY_125_MS		0x02
#define BME280_STANDBY_250_MS		0x03
#define BME280_STANDBY_500_MS		0x04
#define BME280_STANDBY_1000_MS		0x05
#define BME280_STANDBY_62_5_MS_ALT	0x06	/* 10 ms in BME280 */
#define BME280_STANDBY_20_MS		0x07	/* 20 ms in BME280 */

/* IIR Filter Coefficient (filter[2:0] in 0xF5 bits 4:2) */
#define BME280_FILTER_OFF		0x00
#define BME280_FILTER_2X		0x01
#define BME280_FILTER_4X		0x02
#define BME280_FILTER_8X		0x03
#define BME280_FILTER_16X		0x04

/**
 * struct bme280_calib_param - Factory trimming coefficients stored in NVM
 *
 * Read once during driver probe() and used for fixed-point compensation
 * of raw ADC temperature, pressure, and humidity readings per Bosch datasheet
 * Section 4.2.3.
 */
struct bme280_calib_param {
	/* Temperature coefficients */
	u16 dig_T1;
	s16 dig_T2;
	s16 dig_T3;

	/* Pressure coefficients */
	u16 dig_P1;
	s16 dig_P2;
	s16 dig_P3;
	s16 dig_P4;
	s16 dig_P5;
	s16 dig_P6;
	s16 dig_P7;
	s16 dig_P8;
	s16 dig_P9;

	/* Humidity coefficients */
	u8  dig_H1;
	s16 dig_H2;
	u8  dig_H3;
	s16 dig_H4;
	s16 dig_H5;
	s8  dig_H6;

	/* Fine temperature carry-over used by pressure and humidity math */
	s32 t_fine;
};

#endif /* _BME280_CUSTOM_SENSOR_DRIVER_H_ */
