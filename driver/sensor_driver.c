// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bosch BME280 I2C Client Driver with Industrial I/O (IIO) & Fault-Injection
 *
 * Copyright (c) 2026 Nikunj Malik
 *
 * Key Kernel Engineering Features:
 *  - Linux I2C client driver registration (`struct i2c_driver`) with cross-version
 *    compatibility (`LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)` probe shim)
 *  - Device Tree matching (`of_device_id`) and custom DT property configuration
 *  - Managed resource allocation (`devm_iio_device_alloc`, `devm_add_action_or_reset`)
 *  - Mutex-protected atomic 8-byte burst reads (`0xF7..0xFE`)
 *  - Bosch 32-bit/64-bit integer compensation formulas (`div64_s64`, `div_u64`)
 *  - Real hardware reprogramming of `CONFIG` (0xF5 `t_sb[2:0]`) when writing
 *    `sampling_frequency` (including mandatory Sleep Mode transition per Bosch
 *    Datasheet Section 5.4.6) and `IIO_CHAN_INFO_SAMP_FREQ` `read_avail` support
 *  - Debugfs fault injection (`inject_error`, `override_chip_id`) & telemetry counters
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/ktime.h>
#include <asm/unaligned.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "sensor_driver.h"

/**
 * struct bme280_data - Per-device driver private state
 * @client:          Pointer to bound I2C client
 * @lock:            Mutex serializing I2C bus burst transfers and register writes
 * @calib:           Factory calibration parameters read from sensor NVM
 * @osrs_t:          Temperature oversampling setting (0x01..0x05)
 * @osrs_p:          Pressure oversampling setting (0x01..0x05)
 * @osrs_h:          Humidity oversampling setting (0x01..0x05)
 * @filter:          IIR filter coefficient setting (0x00..0x04)
 * @standby:         Normal mode standby interval setting (t_sb[2:0] in 0xF5)
 * @sampling_freq:   Hardware-backed sampling frequency in Hz (1, 2, 4, 8, 16, 26)
 * @config_reg:      Last value written to hardware register 0xF5 (CONFIG)
 * @ctrl_meas_reg:   Last value written to hardware register 0xF4 (CTRL_MEAS)
 * @temp_milli_c:    Last compensated temperature in milli-degrees Celsius
 * @press_pa:        Last compensated pressure in Pascals (Q24.8 format: Pa * 256)
 * @hum_milli_pct:   Last compensated relative humidity in milli-percent
 * @last_update_ns:  Monotonic timestamp (ns) of last successful burst read
 * @total_reads:     Diagnostic counter of successful burst transactions
 * @io_errors:       Diagnostic counter of failed I2C transactions
 * @inject_error:    Debugfs fault-injection errno (0 = normal, negative = force error)
 * @dbg_dir:         Debugfs directory entry for this device instance
 */
struct bme280_data {
	struct i2c_client *client;
	struct mutex lock;
	struct bme280_calib_param calib;

	u8 osrs_t;
	u8 osrs_p;
	u8 osrs_h;
	u8 filter;
	u8 standby;
	int sampling_freq;
	u8 config_reg;
	u8 ctrl_meas_reg;

	s32 temp_milli_c;
	u32 press_pa;
	u32 hum_milli_pct;
	u64 last_update_ns;

	u64 total_reads;
	u64 io_errors;
	s32 inject_error;
	struct dentry *dbg_dir;
};

/*
 * Discrete hardware-supported sampling frequencies (Hz) in Normal Mode corresponding
 * to Bosch BME280 standby timer settings t_sb[2:0]:
 *   1 Hz  -> BME280_STANDBY_1000_MS (0x05)
 *   2 Hz  -> BME280_STANDBY_500_MS  (0x04)
 *   4 Hz  -> BME280_STANDBY_250_MS  (0x03)
 *   8 Hz  -> BME280_STANDBY_125_MS  (0x02)
 *   16 Hz -> BME280_STANDBY_62_5_MS (0x01)
 *   26 Hz -> BME280_STANDBY_0_5_MS  (0x00)
 */
static const int bme280_samp_freq_table[] = { 1, 2, 4, 8, 16, 26 };

static int bme280_standby_to_freq_hz(u8 standby)
{
	switch (standby) {
	case BME280_STANDBY_1000_MS: return 1;
	case BME280_STANDBY_500_MS:  return 2;
	case BME280_STANDBY_250_MS:  return 4;
	case BME280_STANDBY_125_MS:  return 8;
	case BME280_STANDBY_62_5_MS: return 16;
	case BME280_STANDBY_0_5_MS:  return 26;
	default:                     return 10;
	}
}

static int bme280_freq_hz_to_standby(int freq_hz, u8 *standby_out)
{
	switch (freq_hz) {
	case 1:
		*standby_out = BME280_STANDBY_1000_MS;
		return 0;
	case 2:
		*standby_out = BME280_STANDBY_500_MS;
		return 0;
	case 4:
		*standby_out = BME280_STANDBY_250_MS;
		return 0;
	case 8:
		*standby_out = BME280_STANDBY_125_MS;
		return 0;
	case 16:
		*standby_out = BME280_STANDBY_62_5_MS;
		return 0;
	case 26:
		*standby_out = BME280_STANDBY_0_5_MS;
		return 0;
	default:
		return -EINVAL;
	}
}

/* Module parameter to allow testing invalid chip ID rejection at probe time */
static int override_chip_id = -1;
module_param(override_chip_id, int, 0644);
MODULE_PARM_DESC(override_chip_id,
		 "Override hardware Chip ID for fault-injection testing (-1 = use hardware)");

/*
 * Bosch BME280 Fixed-Point Compensation Formulas (Datasheet Section 4.2.3)
 * Note: Floating-point arithmetic is strictly forbidden inside the Linux kernel.
 */

/*
 * bme280_compensate_temp() - Compensate raw 20-bit ADC temperature
 * Returns temperature in 0.01 degC (centi-degrees) and updates calib->t_fine.
 */
static s32 bme280_compensate_temp(struct bme280_calib_param *calib, s32 adc_T)
{
	s32 var1, var2, T;

	var1 = ((((adc_T >> 3) - ((s32)calib->dig_T1 << 1))) *
		((s32)calib->dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((s32)calib->dig_T1)) *
		  ((adc_T >> 4) - ((s32)calib->dig_T1))) >> 12) *
		((s32)calib->dig_T3)) >> 14;

	calib->t_fine = var1 + var2;
	T = (calib->t_fine * 5 + 128) >> 8;
	return T;
}

/*
 * bme280_compensate_press() - Compensate raw 20-bit ADC pressure
 * Must be called after bme280_compensate_temp() so calib->t_fine is current.
 * Returns pressure in Q24.8 format (24 integer bits, 8 fractional bits in Pa).
 */
static u32 bme280_compensate_press(const struct bme280_calib_param *calib, s32 adc_P)
{
	s64 var1, var2, p;

	var1 = ((s64)calib->t_fine) - 128000;
	var2 = var1 * var1 * (s64)calib->dig_P6;
	var2 = var2 + ((var1 * (s64)calib->dig_P5) << 17);
	var2 = var2 + (((s64)calib->dig_P4) << 35);
	var1 = ((var1 * var1 * (s64)calib->dig_P3) >> 8) +
	       ((var1 * (s64)calib->dig_P2) << 12);
	var1 = (((((s64)1) << 47) + var1)) * ((s64)calib->dig_P1) >> 33;

	if (var1 == 0)
		return 0; /* Avoid division by zero if dig_P1 is corrupted */

	p = 1048576 - adc_P;
	p = div64_s64(((p << 31) - var2) * 3125, var1);
	var1 = (((s64)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((s64)calib->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((s64)calib->dig_P7) << 4);

	return (u32)p;
}

/*
 * bme280_compensate_hum() - Compensate raw 16-bit ADC humidity
 * Must be called after bme280_compensate_temp() so calib->t_fine is current.
 * Returns relative humidity in Q22.10 format (1024 = 1.0 %RH).
 */
static u32 bme280_compensate_hum(const struct bme280_calib_param *calib, s32 adc_H)
{
	s32 v_x1_u32r;

	v_x1_u32r = (calib->t_fine - ((s32)76800));
	v_x1_u32r = (((((adc_H << 14) - (((s32)calib->dig_H4) << 20) -
			(((s32)calib->dig_H5) * v_x1_u32r)) + ((s32)16384)) >> 15) *
		     (((((((v_x1_u32r * ((s32)calib->dig_H6)) >> 10) *
			  (((v_x1_u32r * ((s32)calib->dig_H3)) >> 11) + ((s32)32768))) >> 10) +
			((s32)2097152)) * ((s32)calib->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
				   ((s32)calib->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

	return (u32)(v_x1_u32r >> 12);
}

/**
 * bme280_read_calibration() - Read factory calibration trimming bytes from NVM
 * @data: Driver instance state
 *
 * Reads bank 1 (0x88..0x9F, 24 bytes), dig_H1 (0xA1, 1 byte), and
 * bank 2 (0xE1..0xE7, 7 bytes) and unpacks signed/unsigned little-endian words.
 * Uses sign_extend32() on 12-bit signed dig_H4 and dig_H5 per Bosch Datasheet 4.2.2.
 */
static int bme280_read_calibration(struct bme280_data *data)
{
	struct i2c_client *client = data->client;
	struct bme280_calib_param *c = &data->calib;
	u8 buf_tp[BME280_CALIB_TEMP_PRESS_LEN];
	u8 buf_h[BME280_CALIB_HUM_LEN];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client,
					    BME280_REG_CALIB_TEMP_PRESS_START,
					    BME280_CALIB_TEMP_PRESS_LEN,
					    buf_tp);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to read T/P calibration bank (0x88): %d\n", ret);
		return ret;
	}
	if (ret != BME280_CALIB_TEMP_PRESS_LEN)
		return -EIO;

	c->dig_T1 = get_unaligned_le16(&buf_tp[0]);
	c->dig_T2 = (s16)get_unaligned_le16(&buf_tp[2]);
	c->dig_T3 = (s16)get_unaligned_le16(&buf_tp[4]);

	c->dig_P1 = get_unaligned_le16(&buf_tp[6]);
	c->dig_P2 = (s16)get_unaligned_le16(&buf_tp[8]);
	c->dig_P3 = (s16)get_unaligned_le16(&buf_tp[10]);
	c->dig_P4 = (s16)get_unaligned_le16(&buf_tp[12]);
	c->dig_P5 = (s16)get_unaligned_le16(&buf_tp[14]);
	c->dig_P6 = (s16)get_unaligned_le16(&buf_tp[16]);
	c->dig_P7 = (s16)get_unaligned_le16(&buf_tp[18]);
	c->dig_P8 = (s16)get_unaligned_le16(&buf_tp[20]);
	c->dig_P9 = (s16)get_unaligned_le16(&buf_tp[22]);

	/* Validate critical divisor coefficients are non-zero */
	if (c->dig_T1 == 0 || c->dig_P1 == 0) {
		dev_err(&client->dev,
			"Invalid zero calibration parameters (dig_T1=%u, dig_P1=%u)\n",
			c->dig_T1, c->dig_P1);
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(client, BME280_REG_CALIB_DIG_H1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read dig_H1 (0xA1): %d\n", ret);
		return ret;
	}
	c->dig_H1 = (u8)ret;

	ret = i2c_smbus_read_i2c_block_data(client,
					    BME280_REG_CALIB_HUM_START,
					    BME280_CALIB_HUM_LEN,
					    buf_h);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to read Humidity calibration bank (0xE1): %d\n", ret);
		return ret;
	}
	if (ret != BME280_CALIB_HUM_LEN)
		return -EIO;

	c->dig_H2 = (s16)get_unaligned_le16(&buf_h[0]);
	c->dig_H3 = buf_h[2];
	/* dig_H4 (0xE4[7:0] << 4 | 0xE5[3:0]) and dig_H5 (0xE6[7:0] << 4 | 0xE5[7:4]) are 12-bit signed */
	c->dig_H4 = (s16)sign_extend32(((u16)buf_h[3] << 4) | (buf_h[4] & 0x0F), 11);
	c->dig_H5 = (s16)sign_extend32(((u16)buf_h[5] << 4) | ((buf_h[4] >> 4) & 0x0F), 11);
	c->dig_H6 = (s8)buf_h[6];

	return 0;
}

/**
 * bme280_reprogram_standby_locked() - Reprogram hardware standby timer in 0xF5
 * @data:        Driver instance state (caller MUST hold &data->lock)
 * @new_standby: Target t_sb[2:0] code (0x00..0x05)
 *
 * Hardware Truth (Bosch BME280 Datasheet Section 5.4.6 "Register 0xF5 config"):
 * Writes to the `config` register (0xF5) in Normal Mode may be ignored by the
 * sensor hardware. To guarantee deterministic reprogramming, the driver transitions
 * the BME280 to Sleep Mode (`0xF4 = 0x00`), writes the updated `config` register
 * (`0xF5`), and restores Normal Mode in `ctrl_meas` (`0xF4`).
 */
static int bme280_reprogram_standby_locked(struct bme280_data *data, u8 new_standby)
{
	struct i2c_client *client = data->client;
	u8 new_config, ctrl_meas;
	int ret;

	if (unlikely(data->inject_error < 0)) {
		data->io_errors++;
		return data->inject_error;
	}

	/* 1. Enter Sleep Mode so 0xF5 write is guaranteed to latch in hardware */
	ret = i2c_smbus_write_byte_data(client, BME280_REG_CTRL_MEAS,
					BME280_MODE_SLEEP);
	if (ret < 0) {
		data->io_errors++;
		return ret;
	}

	/* 2. Program updated standby timer and IIR filter in 0xF5 */
	new_config = ((new_standby & 0x07) << 5) | ((data->filter & 0x07) << 2);
	ret = i2c_smbus_write_byte_data(client, BME280_REG_CONFIG, new_config);
	if (ret < 0) {
		data->io_errors++;
		return ret;
	}

	/* 3. Restore Normal Mode with configured oversampling ratios in 0xF4 */
	ctrl_meas = ((data->osrs_t & 0x07) << 5) |
		    ((data->osrs_p & 0x07) << 2) |
		    BME280_MODE_NORMAL;
	ret = i2c_smbus_write_byte_data(client, BME280_REG_CTRL_MEAS, ctrl_meas);
	if (ret < 0) {
		data->io_errors++;
		return ret;
	}

	data->standby = new_standby;
	data->sampling_freq = bme280_standby_to_freq_hz(new_standby);
	data->config_reg = new_config;
	data->ctrl_meas_reg = ctrl_meas;

	return 0;
}

/**
 * bme280_configure_chip() - Initialize registers for normal continuous acquisition
 * @data: Driver instance state
 *
 * Important hardware rule (Datasheet 5.4.3): Writes to `ctrl_hum` (0xF2) only
 * take effect after a subsequent write to `ctrl_meas` (0xF4).
 */
static int bme280_configure_chip(struct bme280_data *data)
{
	struct i2c_client *client = data->client;
	u8 config_reg, ctrl_meas_reg;
	int ret, retries = 10;

	/* Issue soft reset */
	ret = i2c_smbus_write_byte_data(client, BME280_REG_RESET, BME280_RESET_VAL);
	if (ret < 0) {
		dev_err(&client->dev, "Soft reset failed: %d\n", ret);
		return ret;
	}

	/* Wait for NVM copy completion (status bit 0 IM_UPDATE == 0) */
	do {
		usleep_range(2000, 3000);
		ret = i2c_smbus_read_byte_data(client, BME280_REG_STATUS);
		if (ret < 0)
			return ret;
	} while ((ret & BME280_STATUS_IM_UPDATE) && --retries > 0);

	if (ret & BME280_STATUS_IM_UPDATE) {
		dev_err(&client->dev, "Timeout waiting for NVM calibration copy\n");
		return -ETIMEDOUT;
	}

	/* 1. Write ctrl_hum (0xF2) first */
	ret = i2c_smbus_write_byte_data(client, BME280_REG_CTRL_HUM,
					data->osrs_h & 0x07);
	if (ret < 0)
		return ret;

	/* 2. Write config (0xF5): standby time and IIR filter */
	config_reg = ((data->standby & 0x07) << 5) | ((data->filter & 0x07) << 2);
	ret = i2c_smbus_write_byte_data(client, BME280_REG_CONFIG, config_reg);
	if (ret < 0)
		return ret;

	/* 3. Write ctrl_meas (0xF4): temp/press oversampling + Normal Mode */
	ctrl_meas_reg = ((data->osrs_t & 0x07) << 5) |
			((data->osrs_p & 0x07) << 2) |
			BME280_MODE_NORMAL;
	ret = i2c_smbus_write_byte_data(client, BME280_REG_CTRL_MEAS, ctrl_meas_reg);
	if (ret < 0)
		return ret;

	data->config_reg = config_reg;
	data->ctrl_meas_reg = ctrl_meas_reg;
	data->sampling_freq = bme280_standby_to_freq_hz(data->standby);

	return 0;
}

/**
 * bme280_read_burst_locked() - Perform atomic 8-byte burst read and compensate
 * @data: Driver instance state (caller MUST hold &data->lock)
 *
 * Reads 0xF7..0xFE in a single I2C block transfer to guarantee that pressure,
 * temperature, and humidity ADC registers belong to the same conversion cycle
 * (Datasheet Section 4.1 "Data register shadowing").
 */
static int bme280_read_burst_locked(struct bme280_data *data)
{
	struct i2c_client *client = data->client;
	u8 buf[BME280_BURST_READ_LEN];
	s32 adc_P, adc_T, adc_H;
	s32 temp_centi;
	u32 press_q24_8, hum_q22_10;
	int ret;

	/* Check debugfs fault-injection hook first */
	if (unlikely(data->inject_error < 0)) {
		data->io_errors++;
		dev_warn_ratelimited(&client->dev,
				     "Simulated fault injection triggered: %d\n",
				     data->inject_error);
		return data->inject_error;
	}

	ret = i2c_smbus_read_i2c_block_data(client, BME280_REG_PRESS_MSB,
					    BME280_BURST_READ_LEN, buf);
	if (ret < 0) {
		data->io_errors++;
		dev_err_ratelimited(&client->dev,
				    "Burst register read failed: %d\n", ret);
		return ret;
	}
	if (ret != BME280_BURST_READ_LEN) {
		data->io_errors++;
		return -EIO;
	}

	/* Unpack 20-bit unsigned ADC values for Pressure & Temperature, 16-bit for Humidity */
	adc_P = (s32)(((u32)buf[0] << 12) | ((u32)buf[1] << 4) | ((u32)buf[2] >> 4));
	adc_T = (s32)(((u32)buf[3] << 12) | ((u32)buf[4] << 4) | ((u32)buf[5] >> 4));
	adc_H = (s32)(((u32)buf[6] << 8) | (u32)buf[7]);

	/*
	 * Guard against uninitialized / brownout reset registers (0x80000).
	 * Never update cached values or return stale readings on invalid ADC state.
	 */
	if (adc_T == BME280_ADC_TEMP_SKIPPED ||
	    adc_P == BME280_ADC_PRESS_SKIPPED ||
	    adc_H == BME280_ADC_HUM_SKIPPED) {
		data->io_errors++;
		dev_warn_ratelimited(&client->dev,
				     "Sensor measurement registers contain reset sentinel (T=0x%x P=0x%x H=0x%x)\n",
				     adc_T, adc_P, adc_H);
		return -ENODATA;
	}

	/* 1. Compensate Temperature first (populates data->calib.t_fine) */
	temp_centi = bme280_compensate_temp(&data->calib, adc_T);
	data->temp_milli_c = temp_centi * 10; /* Convert 0.01 degC -> milli-degC */

	/* 2. Compensate Pressure using updated t_fine */
	press_q24_8 = bme280_compensate_press(&data->calib, adc_P);
	data->press_pa = press_q24_8; /* Stored in Q24.8 Pa */

	/* 3. Compensate Humidity using updated t_fine */
	hum_q22_10 = bme280_compensate_hum(&data->calib, adc_H);
	/* Convert Q22.10 (%RH * 1024) to milli-percent (%RH * 1000) */
	data->hum_milli_pct = (u32)div_u64((u64)hum_q22_10 * 1000ULL, 1024ULL);

	data->last_update_ns = ktime_get_boottime_ns();
	data->total_reads++;

	return 0;
}

/*
 * Linux Industrial I/O (IIO) Channel Definitions and Callbacks
 *
 * Standard Linux IIO ABI units (Documentation/ABI/testing/sysfs-bus-iio):
 *  - in_temp_input:             milli-degrees Celsius (IIO_VAL_INT)
 *  - in_pressure_input:         kilopascals (kPa)     (IIO_VAL_FRACTIONAL: Q24.8 Pa / 256000)
 *  - in_humidityrelative_input: milli-percent (%RH)   (IIO_VAL_INT)
 *  - sampling_frequency:        Hertz (Hz)            (IIO_VAL_INT, backed by 0xF5 t_sb[2:0])
 */
static const struct iio_chan_spec bme280_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
};

static int bme280_osrs_to_ratio(u8 osrs)
{
	switch (osrs) {
	case BME280_OSRS_1X:  return 1;
	case BME280_OSRS_2X:  return 2;
	case BME280_OSRS_4X:  return 4;
	case BME280_OSRS_8X:  return 8;
	case BME280_OSRS_16X: return 16;
	default:              return 0;
	}
}

static int bme280_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = bme280_samp_freq_table;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(bme280_samp_freq_table);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int bme280_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct bme280_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = bme280_read_burst_locked(data);
		if (ret < 0) {
			mutex_unlock(&data->lock);
			return ret;
		}

		switch (chan->type) {
		case IIO_TEMP:
			/* Standard IIO unit: milli-degrees Celsius */
			*val = data->temp_milli_c;
			ret = IIO_VAL_INT;
			break;
		case IIO_PRESSURE:
			/*
			 * Standard IIO unit: kilopascals (kPa).
			 * data->press_pa is Q24.8 Pascals (Pa * 256).
			 * Dividing by (256 * 1000) = 256000 yields kPa via IIO_VAL_FRACTIONAL
			 * (which the kernel IIO core formats as a fixed-point decimal string,
			 * e.g. "100.652343\n").
			 */
			*val = data->press_pa;
			*val2 = 256000;
			ret = IIO_VAL_FRACTIONAL;
			break;
		case IIO_HUMIDITYRELATIVE:
			/* Standard IIO unit: milli-percent relative humidity */
			*val = data->hum_milli_pct;
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (chan->type) {
		case IIO_TEMP:
			*val = bme280_osrs_to_ratio(data->osrs_t);
			ret = IIO_VAL_INT;
			break;
		case IIO_PRESSURE:
			*val = bme280_osrs_to_ratio(data->osrs_p);
			ret = IIO_VAL_INT;
			break;
		case IIO_HUMIDITYRELATIVE:
			*val = bme280_osrs_to_ratio(data->osrs_h);
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = data->sampling_freq;
		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&data->lock);
	return ret;
}

static int bme280_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct bme280_data *data = iio_priv(indio_dev);
	u8 target_standby;
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ || val2 != 0)
		return -EINVAL;

	/* Map requested Hz (1, 2, 4, 8, 16, 26) to hardware standby code t_sb[2:0] */
	ret = bme280_freq_hz_to_standby(val, &target_standby);
	if (ret < 0)
		return ret;

	mutex_lock(&data->lock);
	ret = bme280_reprogram_standby_locked(data, target_standby);
	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_info bme280_iio_info = {
	.read_raw = bme280_read_raw,
	.write_raw = bme280_write_raw,
	.read_avail = bme280_read_avail,
};

/*
 * Debugfs Interface for Automated Fault Injection and Telemetry Verification
 */
static int bme280_dbg_stats_show(struct seq_file *s, void *unused)
{
	struct bme280_data *data = s->private;

	mutex_lock(&data->lock);
	seq_printf(s, "driver: %s\n", BME280_DRIVER_NAME);
	seq_printf(s, "i2c_addr: 0x%02x\n", data->client->addr);
	seq_printf(s, "total_reads: %llu\n", data->total_reads);
	seq_printf(s, "io_errors: %llu\n", data->io_errors);
	seq_printf(s, "inject_error: %d\n", data->inject_error);
	seq_printf(s, "sampling_freq_hz: %d\n", data->sampling_freq);
	seq_printf(s, "standby_code: 0x%02x\n", data->standby);
	seq_printf(s, "reg_config_0xf5: 0x%02x\n", data->config_reg);
	seq_printf(s, "reg_ctrl_meas_0xf4: 0x%02x\n", data->ctrl_meas_reg);
	seq_printf(s, "last_update_ns: %llu\n", data->last_update_ns);
	seq_printf(s, "last_temp_milli_c: %d\n", data->temp_milli_c);
	seq_printf(s, "last_press_q24_8_pa: %u\n", data->press_pa);
	seq_printf(s, "last_hum_milli_pct: %u\n", data->hum_milli_pct);
	seq_printf(s, "calib_dig_T1: %u\n", data->calib.dig_T1);
	seq_printf(s, "calib_dig_P1: %u\n", data->calib.dig_P1);
	seq_printf(s, "calib_dig_H1: %u\n", data->calib.dig_H1);
	mutex_unlock(&data->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bme280_dbg_stats);

static void bme280_debugfs_remove(void *ptr)
{
	struct bme280_data *data = ptr;

	debugfs_remove_recursive(data->dbg_dir);
}

static void bme280_setup_debugfs(struct bme280_data *data)
{
	struct dentry *dir;

	dir = debugfs_create_dir(BME280_DRIVER_NAME, NULL);
	if (IS_ERR_OR_NULL(dir))
		return;

	data->dbg_dir = dir;
	debugfs_create_s32("inject_error", 0600, dir, &data->inject_error);
	debugfs_create_u64("total_reads", 0400, dir, &data->total_reads);
	debugfs_create_u64("io_errors", 0400, dir, &data->io_errors);
	debugfs_create_file("stats", 0400, dir, data, &bme280_dbg_stats_fops);

	devm_add_action_or_reset(&data->client->dev, bme280_debugfs_remove, data);
}

static void bme280_power_down_action(void *ptr)
{
	struct bme280_data *data = ptr;

	/* Put sensor into low-power sleep mode (0xF4 = 0x00) on driver unload */
	i2c_smbus_write_byte_data(data->client, BME280_REG_CTRL_MEAS,
				  BME280_MODE_SLEEP);
}

static void bme280_parse_dt_properties(struct bme280_data *data)
{
	struct device *dev = &data->client->dev;
	u32 val;

	/* Set safe production defaults first (Bosch Indoor Navigation profile) */
	data->osrs_t = BME280_OSRS_2X;
	data->osrs_p = BME280_OSRS_16X;
	data->osrs_h = BME280_OSRS_1X;
	data->filter = BME280_FILTER_16X;
	data->standby = BME280_STANDBY_62_5_MS; /* 16 Hz default continuous rate */
	data->sampling_freq = bme280_standby_to_freq_hz(data->standby);

	if (!device_property_read_u32(dev, "custom,temp-oversampling", &val) &&
	    val >= BME280_OSRS_1X && val <= BME280_OSRS_16X)
		data->osrs_t = (u8)val;

	if (!device_property_read_u32(dev, "custom,pressure-oversampling", &val) &&
	    val >= BME280_OSRS_1X && val <= BME280_OSRS_16X)
		data->osrs_p = (u8)val;

	if (!device_property_read_u32(dev, "custom,humidity-oversampling", &val) &&
	    val >= BME280_OSRS_1X && val <= BME280_OSRS_16X)
		data->osrs_h = (u8)val;

	if (!device_property_read_u32(dev, "custom,iir-filter-coeff", &val) &&
	    val <= BME280_FILTER_16X)
		data->filter = (u8)val;

	if (!device_property_read_u32(dev, "custom,standby-mode", &val) &&
	    val <= BME280_STANDBY_1000_MS) {
		data->standby = (u8)val;
		data->sampling_freq = bme280_standby_to_freq_hz(data->standby);
	}
}

static int bme280_probe_core(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct bme280_data *data;
	int chip_id, ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		dev_err(&client->dev,
			"I2C adapter does not support required SMBus block operations\n");
		return -EOPNOTSUPP;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	i2c_set_clientdata(client, indio_dev);
	mutex_init(&data->lock);

	/* Read and verify hardware Chip ID register (0xD0) */
	if (override_chip_id >= 0) {
		chip_id = override_chip_id;
		dev_warn(&client->dev,
			 "Using overridden Chip ID 0x%02x (fault-injection active)\n",
			 chip_id);
	} else {
		chip_id = i2c_smbus_read_byte_data(client, BME280_REG_CHIP_ID);
		if (chip_id < 0) {
			dev_err(&client->dev,
				"Failed to read Chip ID register (0xD0) at 0x%02x: %d\n",
				client->addr, chip_id);
			return chip_id;
		}
	}

	if (chip_id != BME280_CHIP_ID_VAL) {
		dev_err(&client->dev,
			"Unsupported Chip ID 0x%02x (expected BME280 0x%02x)\n",
			chip_id, BME280_CHIP_ID_VAL);
		return -ENODEV;
	}

	bme280_parse_dt_properties(data);

	ret = bme280_configure_chip(data);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to configure BME280: %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(&client->dev, bme280_power_down_action, data);
	if (ret < 0)
		return ret;

	ret = bme280_read_calibration(data);
	if (ret < 0)
		return ret;

	/* Configure Industrial I/O device metadata */
	indio_dev->name = BME280_DRIVER_NAME;
	indio_dev->info = &bme280_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = bme280_channels;
	indio_dev->num_channels = ARRAY_SIZE(bme280_channels);

	bme280_setup_debugfs(data);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev,
		 "Bosch BME280 initialized at 0x%02x (Chip ID 0x%02x, freq=%d Hz, config=0x%02x)\n",
		 client->addr, chip_id, data->sampling_freq, data->config_reg);
	return 0;
}

/*
 * Linux Kernel Version Compatibility Shim:
 * Linux 6.3+ changed `struct i2c_driver::probe` from:
 *   int (*probe)(struct i2c_client *client, const struct i2c_device_id *id)
 * to:
 *   int (*probe)(struct i2c_client *client)
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int bme280_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	(void)id;
	return bme280_probe_core(client);
}
#else
static int bme280_probe(struct i2c_client *client)
{
	return bme280_probe_core(client);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int bme280_remove(struct i2c_client *client)
{
	dev_info(&client->dev,
		 "Bosch BME280 driver unbound from 0x%02x (entering sleep mode)\n",
		 client->addr);
	return 0;
}
#else
static void bme280_remove(struct i2c_client *client)
{
	dev_info(&client->dev,
		 "Bosch BME280 driver unbound from 0x%02x (entering sleep mode)\n",
		 client->addr);
}
#endif

static const struct i2c_device_id bme280_id_table[] = {
	{ "bme280-sensor", 0 },
	{ "bme280_custom", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bme280_id_table);

static const struct of_device_id bme280_of_match[] = {
	{ .compatible = "custom,bme280-sensor" },
	{ .compatible = "bosch,bme280-custom" },
	{ }
};
MODULE_DEVICE_TABLE(of, bme280_of_match);

static struct i2c_driver bme280_driver = {
	.driver = {
		.name = BME280_DRIVER_NAME,
		.of_match_table = bme280_of_match,
	},
	.probe = bme280_probe,
	.remove = bme280_remove,
	.id_table = bme280_id_table,
};
module_i2c_driver(bme280_driver);

MODULE_AUTHOR("Nikunj Malik <nikunjmalik306@github.com>");
MODULE_DESCRIPTION("Bosch BME280 I2C Client Driver with IIO & Fault-Injection");
MODULE_LICENSE("GPL");
