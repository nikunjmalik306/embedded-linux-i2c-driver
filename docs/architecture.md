# System Architecture & Kernel Driver Design

## 1. Architectural Overview

The **Embedded Linux BME280 Sensor Driver and Validation Platform** is structured across four decoupled layers following upstream Linux kernel subsystem conventions:

1. **Hardware & Bus Layer:** Bosch BME280 digital environmental sensor connected to the Raspberry Pi 5 RP1 Southbridge `I2C1` controller (`GPIO2` SDA / `GPIO3` SCL) at slave address `0x76`.
2. **Device Tree Hardware Description (`dts/sensor-overlay.dts` & `dts/bindings/custom,bme280-sensor.yaml`):** Declares the hardware topology, bus frequency (`100 kHz`), device node `bme280@76`, compatible string `"custom,bme280-sensor"`, and initial oversampling, IIR filter, and standby timer parameters independently of C source code.
3. **Linux Kernel I²C Client & IIO Driver (`driver/sensor_driver.c`):** Matches `"custom,bme280-sensor"`, validates hardware identity (`0xD0 == 0x60`), reads factory trimming coefficients from NVM, manages power state via `devm_*` actions, and registers an **Industrial I/O (`iio_dev`)** interface with mutex-serialized atomic burst reads, hardware-backed `sampling_frequency` reprogramming (`0xF5`), and `debugfs` fault-injection hooks.
4. **Userspace Telemetry & Validation Layer (`userspace/sensor_reader.c` & `tests/test_regression.py`):** Consumes standard `/sys/bus/iio/devices/iio:deviceX/` attributes using deterministic `CLOCK_MONOTONIC` scheduling, records latency/jitter statistics, and executes automated fault-injection regressions.

---

## 2. Standard Linux IIO ABI Semantics & Units

Per `Documentation/ABI/testing/sysfs-bus-iio`, the driver exposes standard units and value types:

| Sysfs Attribute | IIO Channel / Info Mask | Return Type | Mathematical Representation | Standard Unit |
| :--- | :--- | :--- | :--- | :--- |
| `in_temp_input` | `IIO_TEMP` (`PROCESSED`) | `IIO_VAL_INT` | `temp_centi * 10` | **milli-degrees Celsius** (`25080` = `25.080 °C`) |
| `in_pressure_input` | `IIO_PRESSURE` (`PROCESSED`) | `IIO_VAL_FRACTIONAL` | `*val = press_q24_8`, `*val2 = 256000` | **kilopascals (kPa)** (`100.652343 kPa` = `1006.52 hPa`) |
| `in_humidityrelative_input` | `IIO_HUMIDITYRELATIVE` (`PROCESSED`) | `IIO_VAL_INT` | `(hum_q22_10 * 1000) / 1024` | **milli-percent RH** (`53952` = `53.952 %RH`) |
| `sampling_frequency` | `SAMP_FREQ` (`shared_by_all`) | `IIO_VAL_INT` | Maps `1, 2, 4, 8, 16, 26 Hz` $\leftrightarrow$ `0xF5` `t_sb[2:0]` | **Hertz (Hz)** |
| `sampling_frequency_available` | `read_avail` (`SAMP_FREQ`) | `IIO_AVAIL_LIST` | `"1 2 4 8 16 26"` | **Hertz (Hz)** |

### How `IIO_VAL_FRACTIONAL` Works Between Kernel and Userspace
When `bme280_read_raw()` returns `IIO_VAL_FRACTIONAL` with `*val = data->press_pa` (in Q24.8 Pascals, i.e., `Pa * 256`) and `*val2 = 256000` (`256 * 1000 Pa/kPa`), the Linux kernel's `iio_format_value()` helper performs fixed-point division inside the kernel and prints a decimal string (such as `"100.652343\n"`) to sysfs.
In `userspace/sensor_reader.c`, `parse_iio_number()` handles both this kernel-formatted decimal representation and raw `"numerator/denominator"` strings (`"25766999/256000"`), converting `kPa` to `hPa` (`kPa * 10.0`).

---

## 3. Hardware-Backed `sampling_frequency` Reprogramming (`0xF5`)

In Normal Mode, the BME280 automatically alternates between an active measurement period ($T_{\text{measure}}$) and a quiescent standby period ($t_{\text{standby}}$, controlled by bits `7:5` `t_sb[2:0]` of register `0xF5` `CONFIG`):

$$f_{\text{sample}} = \frac{1}{T_{\text{measure}} + t_{\text{standby}}}$$

At the default Indoor Navigation oversampling profile (`2x` temperature, `16x` pressure, `1x` humidity), $T_{\text{measure}} \approx 37.5\text{ ms}$ (`0.0375 s`):

| `t_sb[2:0]` Code in `0xF5` | Standby Duration ($t_{\text{standby}}$) | Total Cycle Period ($T_{\text{cycle}}$) | Nominal Sampling Frequency |
| :---: | :---: | :---: | :---: |
| `0x05` | `1000.0 ms` | `~1037.5 ms` | **`1 Hz`** |
| `0x04` | `500.0 ms` | `~537.5 ms` | **`2 Hz`** |
| `0x03` | `250.0 ms` | `~287.5 ms` | **`4 Hz`** |
| `0x02` | `125.0 ms` | `~162.5 ms` | **`8 Hz`** |
| `0x01` | `62.5 ms` | `~100.0 ms` | **`16 Hz`** *(Default)* |
| `0x00` | `0.5 ms` | `~38.0 ms` | **`26 Hz`** |

**Critical Hardware Rule (Bosch Datasheet Section 5.4.6):** Writing to `CONFIG` (`0xF5`) while the device is actively cycling in Normal Mode may be ignored by the sensor. Therefore, `bme280_reprogram_standby_locked()` holds `&data->lock` and executes:
1. `i2c_smbus_write_byte_data(client, 0xF4, BME280_MODE_SLEEP)`
2. `i2c_smbus_write_byte_data(client, 0xF5, new_config)`
3. `i2c_smbus_write_byte_data(client, 0xF4, ctrl_meas_normal)`

---

## 4. Linux Kernel Version Compatibility (`5.15 LTS` through `6.12+`)

Upstream Linux kernel APIs evolve across major releases. `driver/sensor_driver.c` uses `<linux/version.h>` guards to compile cleanly across LTS and mainline kernels:

- **Linux 6.3+ `i2c_driver::probe` Signature Change:** Commit `03c835f498b5` ("i2c: Switch .probe() to not take an id parameter") removed `const struct i2c_device_id *id` from `.probe()`. We provide `bme280_probe_core(struct i2c_client *client)` wrapped by a `#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)` shim.
- **Linux 6.1+ `i2c_driver::remove` Return Type Change:** Commit `ed5c2f5fd10d` changed `.remove()` from returning `int` to `void`. Guarded via `#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)`.
- **Unaligned Little-Endian & Signed Bitfield Helpers:** Uses `<asm/unaligned.h>` (`get_unaligned_le16`) and `<linux/bitops.h>` (`sign_extend32(..., 11)`) for 12-bit signed `dig_H4`/`dig_H5` unpacking.
