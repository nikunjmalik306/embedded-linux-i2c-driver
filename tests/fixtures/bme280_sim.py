#!/usr/bin/env python3
"""
Register-Accurate Bosch BME280 Hardware & IIO Sysfs Simulator.

Implements:
1. The exact 32-bit/64-bit integer fixed-point compensation algorithms used in
   `driver/sensor_driver.c` (Bosch BME280 Datasheet Section 4.2.3).
2. An independent double-precision floating-point compensation oracle from
   Bosch BME280 Datasheet Section 8.1 to cross-verify the integer fixed-point math.
3. Hardware register simulation for `CONFIG` (`0xF5`), `CTRL_MEAS` (`0xF4`),
   `CTRL_HUM` (`0xF2`), and `sampling_frequency` <-> `t_sb[2:0]` standby mapping.
4. Mock `/sys/bus/iio/devices/iio:device0` tree for testing `userspace/sensor_reader`
   under both kernel-formatted decimal strings and raw `A/B` fractional strings.
"""

import argparse
import json
from pathlib import Path
from typing import Dict, Optional, Tuple


class BME280Simulator:
    """Simulates a Bosch BME280 I2C slave and Linux IIO sysfs interface."""

    CHIP_ID_BME280 = 0x60
    CHIP_ID_BMP280 = 0x58
    ADC_SKIPPED_20BIT = 0x80000
    ADC_SKIPPED_16BIT = 0x8000

    # Mapping of discrete sampling_frequency (Hz) -> t_sb[2:0] standby code in 0xF5
    FREQ_TO_STANDBY = {
        1: 0x05,   # 1000.0 ms
        2: 0x04,   #  500.0 ms
        4: 0x03,   #  250.0 ms
        8: 0x02,   #  125.0 ms
        16: 0x01,  #   62.5 ms
        26: 0x00,  #    0.5 ms
    }
    STANDBY_TO_FREQ = {v: k for k, v in FREQ_TO_STANDBY.items()}

    def __init__(self, calib_path: Optional[Path] = None):
        if calib_path is None:
            calib_path = Path(__file__).parent / "sample_calibration.json"
        with open(calib_path, "r", encoding="utf-8") as fp:
            data = json.load(fp)

        self.calib: Dict[str, int] = data["calibration"]
        self.reference_vectors = data["reference_vectors"]
        self.chip_id: int = self.CHIP_ID_BME280
        self.t_fine: int = 0
        self.inject_errno: int = 0
        self.bus_connected: bool = True
        self.power_on_reset_sentinel: bool = False

        # Hardware configuration state matching driver defaults
        self.osrs_t: int = 0x02  # 2x
        self.osrs_p: int = 0x05  # 16x
        self.osrs_h: int = 0x01  # 1x
        self.filter: int = 0x04  # 16x
        self.standby: int = 0x01  # 62.5 ms (16 Hz)
        self.sampling_freq: int = 16
        self.reg_config_0xf5: int = ((self.standby & 0x07) << 5) | ((self.filter & 0x07) << 2)
        self.reg_ctrl_meas_0xf4: int = ((self.osrs_t & 0x07) << 5) | ((self.osrs_p & 0x07) << 2) | 0x03

        # Default raw ADC registers
        self.adc_T: int = 519888
        self.adc_P: int = 415148
        self.adc_H: int = 29812

    def probe_check(self) -> int:
        """Simulates kernel driver probe() hardware verification."""
        if not self.bus_connected:
            return -6  # -ENXIO (No such device or address / I2C NACK)
        if self.inject_errno < 0:
            return self.inject_errno
        if self.chip_id != self.CHIP_ID_BME280:
            return -19  # -ENODEV (Wrong Chip ID)
        if self.calib.get("dig_T1", 0) == 0 or self.calib.get("dig_P1", 0) == 0:
            return -22  # -EINVAL (Corrupted NVM calibration)
        return 0

    def set_sampling_frequency(self, freq_hz: int) -> int:
        """
        Simulates `bme280_write_raw(..., IIO_CHAN_INFO_SAMP_FREQ)` reprogramming
        hardware register 0xF5 (`CONFIG`) while transitioning through Sleep Mode.
        """
        if not self.bus_connected:
            return -121  # -EREMOTEIO
        if self.inject_errno < 0:
            return self.inject_errno
        if freq_hz not in self.FREQ_TO_STANDBY:
            return -22  # -EINVAL (Unsupported frequency)

        new_standby = self.FREQ_TO_STANDBY[freq_hz]
        # 1. Enter Sleep Mode (0xF4 = 0x00)
        self.reg_ctrl_meas_0xf4 = 0x00
        # 2. Write new standby time into 0xF5 bits 7:5
        self.standby = new_standby
        self.reg_config_0xf5 = ((self.standby & 0x07) << 5) | ((self.filter & 0x07) << 2)
        # 3. Restore Normal Mode (0xF4)
        self.reg_ctrl_meas_0xf4 = ((self.osrs_t & 0x07) << 5) | ((self.osrs_p & 0x07) << 2) | 0x03
        self.sampling_freq = self.STANDBY_TO_FREQ[self.standby]
        return 0

    def compensate_temp(self, adc_T: int) -> int:
        """Bosch 32-bit fixed-point temperature compensation -> centi-degrees C."""
        c = self.calib
        var1 = (((adc_T >> 3) - (c["dig_T1"] << 1)) * c["dig_T2"]) >> 11
        var2 = (
            ((((adc_T >> 4) - c["dig_T1"]) * ((adc_T >> 4) - c["dig_T1"])) >> 12)
            * c["dig_T3"]
        ) >> 14
        self.t_fine = var1 + var2
        temp_centi = (self.t_fine * 5 + 128) >> 8
        return temp_centi

    def compensate_press(self, adc_P: int) -> int:
        """
        Bosch 64-bit fixed-point pressure compensation -> Q24.8 Pascals.
        Note: Uses C-style truncation toward zero (`int(a / b)`) to match
        the Linux kernel's `div64_s64()` helper in `driver/sensor_driver.c`.
        """
        c = self.calib
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * c["dig_P6"]
        var2 = var2 + ((var1 * c["dig_P5"]) << 17)
        var2 = var2 + (c["dig_P4"] << 35)
        var1 = ((var1 * var1 * c["dig_P3"]) >> 8) + ((var1 * c["dig_P2"]) << 12)
        var1 = (((1 << 47) + var1) * c["dig_P1"]) >> 33
        if var1 == 0:
            return 0
        p = 1048576 - adc_P
        numerator = ((p << 31) - var2) * 3125
        # C `div64_s64` truncates toward zero (unlike Python's `//` floor division for negatives)
        p = int(numerator // var1) if (numerator >= 0) == (var1 >= 0) else -int(abs(numerator) // abs(var1))
        var1 = (c["dig_P9"] * (p >> 13) * (p >> 13)) >> 25
        var2 = (c["dig_P8"] * p) >> 19
        p = ((p + var1 + var2) >> 8) + (c["dig_P7"] << 4)
        return int(p)

    def compensate_hum(self, adc_H: int) -> int:
        """Bosch 32-bit fixed-point humidity compensation -> Q22.10 %RH."""
        c = self.calib
        v_x1 = self.t_fine - 76800
        v_x1 = (
            (((adc_H << 14) - (c["dig_H4"] << 20) - (c["dig_H5"] * v_x1) + 16384) >> 15)
            * (
                (
                    (
                        (((v_x1 * c["dig_H6"]) >> 10) * (((v_x1 * c["dig_H3"]) >> 11) + 32768))
                        >> 10
                    )
                    + 2097152
                )
                * c["dig_H2"]
                + 8192
            )
            >> 14
        )
        v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * c["dig_H1"]) >> 4)
        v_x1 = max(0, min(v_x1, 419430400))
        return int(v_x1 >> 12)

    def compensate_double_precision_oracle(
        self, adc_T: int, adc_P: int, adc_H: int
    ) -> Tuple[float, float, float]:
        """
        Independent double-precision floating-point compensation reference
        directly from Bosch BME280 Datasheet Section 8.1 ("Compensation formula
        in double precision floating point").
        Returns (temperature_degC, pressure_kPa, humidity_pct).
        """
        c = self.calib
        # 1. Temperature (Datasheet 8.1)
        var1_t = (adc_T / 16384.0 - c["dig_T1"] / 1024.0) * c["dig_T2"]
        var2_t = (
            (adc_T / 131072.0 - c["dig_T1"] / 8192.0)
            * (adc_T / 131072.0 - c["dig_T1"] / 8192.0)
        ) * c["dig_T3"]
        t_fine_flt = var1_t + var2_t
        temp_c = t_fine_flt / 5120.0

        # 2. Pressure (Datasheet 8.1)
        var1_p = (t_fine_flt / 2.0) - 64000.0
        var2_p = var1_p * var1_p * c["dig_P6"] / 32768.0
        var2_p = var2_p + var1_p * c["dig_P5"] * 2.0
        var2_p = (var2_p / 4.0) + (c["dig_P4"] * 65536.0)
        var1_p = (c["dig_P3"] * var1_p * var1_p / 524288.0 + c["dig_P2"] * var1_p) / 524288.0
        var1_p = (1.0 + var1_p / 32768.0) * c["dig_P1"]
        if var1_p == 0.0:
            press_pa = 0.0
        else:
            p = 1048576.0 - adc_P
            p = (p - (var2_p / 4096.0)) * 6250.0 / var1_p
            var1_p = c["dig_P9"] * p * p / 2147483648.0
            var2_p = p * c["dig_P8"] / 32768.0
            press_pa = p + (var1_p + var2_p + c["dig_P7"]) / 16.0

        # 3. Humidity (Datasheet 8.1)
        var_h = t_fine_flt - 76800.0
        var_h = (adc_H - (c["dig_H4"] * 64.0 + c["dig_H5"] / 16384.0 * var_h)) * (
            c["dig_H2"]
            / 65536.0
            * (
                1.0
                + c["dig_H6"]
                / 67108864.0
                * var_h
                * (1.0 + c["dig_H3"] / 67108864.0 * var_h)
            )
        )
        var_h = var_h * (1.0 - c["dig_H1"] * var_h / 524288.0)
        hum_pct = max(0.0, min(100.0, var_h))

        return temp_c, press_pa / 1000.0, hum_pct

    def read_burst(self) -> Tuple[int, Optional[Dict[str, float]]]:
        """
        Simulates atomic 8-byte burst read (0xF7..0xFE) and compensation.
        Returns (0, dict) on success or (negative_errno, None) on fault.
        """
        if not self.bus_connected:
            return -121, None  # -EREMOTEIO
        if self.inject_errno < 0:
            return self.inject_errno, None
        if (
            self.power_on_reset_sentinel
            or self.adc_T == self.ADC_SKIPPED_20BIT
            or self.adc_P == self.ADC_SKIPPED_20BIT
            or self.adc_H == self.ADC_SKIPPED_16BIT
        ):
            return -61, None  # -ENODATA

        temp_milli_c = self.compensate_temp(self.adc_T) * 10
        press_q24_8 = self.compensate_press(self.adc_P)
        hum_q22_10 = self.compensate_hum(self.adc_H)
        hum_milli_pct = (hum_q22_10 * 1000) // 1024

        return 0, {
            "in_temp_input": temp_milli_c,
            "in_pressure_input_q24_8": press_q24_8,
            "in_pressure_input_kpa": press_q24_8 / 256000.0,
            "in_humidityrelative_input": hum_milli_pct,
        }

    def materialize_sysfs(
        self,
        root_dir: Path,
        device_name: str = "iio:device0",
        use_fractional_string: bool = False,
    ) -> Path:
        """Creates or updates a mock IIO sysfs directory for userspace testing."""
        dev_dir = root_dir / device_name
        dev_dir.mkdir(parents=True, exist_ok=True)

        (dev_dir / "name").write_text("bme280_custom\n", encoding="utf-8")
        (dev_dir / "sampling_frequency").write_text(f"{self.sampling_freq}\n", encoding="utf-8")
        (dev_dir / "sampling_frequency_available").write_text(
            "1 2 4 8 16 26\n", encoding="utf-8"
        )
        (dev_dir / "in_temp_oversampling_ratio").write_text("2\n", encoding="utf-8")
        (dev_dir / "in_pressure_oversampling_ratio").write_text("16\n", encoding="utf-8")
        (dev_dir / "in_humidityrelative_oversampling_ratio").write_text("1\n", encoding="utf-8")

        rc, vals = self.read_burst()
        if rc < 0 or vals is None:
            err_token = f"ERR:{rc}\n"
            (dev_dir / "in_temp_input").write_text(err_token, encoding="utf-8")
            (dev_dir / "in_pressure_input").write_text(err_token, encoding="utf-8")
            (dev_dir / "in_humidityrelative_input").write_text(err_token, encoding="utf-8")
        else:
            (dev_dir / "in_temp_input").write_text(
                f"{vals['in_temp_input']}\n", encoding="utf-8"
            )
            if use_fractional_string:
                # Test raw numerator/denominator string representation (Q24.8 Pa / 256000)
                (dev_dir / "in_pressure_input").write_text(
                    f"{vals['in_pressure_input_q24_8']}/256000\n", encoding="utf-8"
                )
            else:
                # Standard Linux kernel IIO_VAL_FRACTIONAL sysfs formatting
                (dev_dir / "in_pressure_input").write_text(
                    f"{vals['in_pressure_input_kpa']:.6f}\n", encoding="utf-8"
                )
            (dev_dir / "in_humidityrelative_input").write_text(
                f"{vals['in_humidityrelative_input']}\n", encoding="utf-8"
            )

        return dev_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Create simulated BME280 IIO sysfs tree")
    parser.add_argument("--create-sysfs", required=True, help="Target directory for mock sysfs")
    args = parser.parse_args()

    sim = BME280Simulator()
    dev_path = sim.materialize_sysfs(Path(args.create_sysfs))
    print(f"Created simulated BME280 IIO device at: {dev_path}")


if __name__ == "__main__":
    main()
