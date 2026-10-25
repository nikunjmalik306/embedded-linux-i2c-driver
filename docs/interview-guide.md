# Technical Interview Walkthrough & Defense Guide

Use this concise reference during **Embedded Linux, BSP, Kernel Driver, and Firmware Validation** interviews to explain the engineering trade-offs in this repository.

---

## 1. Why did you use the Linux Industrial I/O (IIO) subsystem instead of a custom `/dev/bme280` character device?
- Writing a custom `cdev` with bespoke `ioctl()` structs forces userspace applications to include custom C headers and prevents standard Linux tools (`iio_info`, `libiio`, `iio_readdev`) from discovering the sensor.
- **IIO (`drivers/iio/`)** is the standard upstream Linux subsystem for multi-channel measurement devices (ADCs, IMUs, barometers, hygrometers). By registering an `iio_dev` with `IIO_TEMP`, `IIO_PRESSURE`, and `IIO_HUMIDITYRELATIVE` channels, any standard userspace client can read deterministic sysfs attributes (`in_temp_input`, `in_pressure_input`, `in_humidityrelative_input`) in standard units (`milli-°C`, `kPa`, `milli-%RH`).

---

## 2. Why does the driver use a `struct mutex` instead of a `spinlock_t`?
- **I²C bus transfers sleep:** `i2c_smbus_read_i2c_block_data()` and `i2c_smbus_write_byte_data()` wait on completion interrupts from the I²C bus controller (`i2c-designware` / RP1), which puts the calling kernel thread to sleep for hundreds of microseconds.
- Calling a sleeping function while holding a `spinlock_t` (which disables preemption) triggers a fatal kernel bug (`BUG: scheduling while atomic`).
- Furthermore, `data->calib.t_fine` is updated during temperature compensation and read during pressure and humidity compensation, and changing `sampling_frequency` requires a 3-step `Sleep -> Write 0xF5 -> Normal` register sequence. `mutex_lock(&data->lock)` serializes all of these operations safely in process context.

---

## 3. How does Device Tree matching work from `.dts` to `bme280_probe()`?
1. The Device Tree Overlay (`dts/sensor-overlay.dts`) attaches a child node `bme280@76` to `&i2c1` with `compatible = "custom,bme280-sensor"` and `reg = <0x76>`.
2. When the overlay is loaded (`dtoverlay`), the Linux I²C core instantiates an `i2c_client` at address `0x76` on bus 1.
3. The driver exports `bme280_of_match[]` containing `{ .compatible = "custom,bme280-sensor" }` and registers it via `MODULE_DEVICE_TABLE(of, bme280_of_match)`.
4. When the kernel matches the DT node's `compatible` string against `bme280_driver.driver.of_match_table`, it invokes `bme280_probe(client)`. Inside `probe()`, `device_property_read_u32()` extracts custom DT properties (`custom,temp-oversampling`, `custom,standby-mode`, etc.).

---

## 4. How do `probe()` and `remove()` manage resources without memory leaks?
- We use **Device-Managed (`devm_*`) allocations**:
  - `devm_iio_device_alloc()` allocates both the `struct iio_dev` and our private `struct bme280_data`.
  - `devm_add_action_or_reset()` registers two deterministic cleanup callbacks (`bme280_power_down_action` to put the chip into `MODE_SLEEP` `0x00` and `bme280_debugfs_remove` to remove `/sys/kernel/debug/bme280_custom`).
  - `devm_iio_device_register()` registers the sysfs interface.
- If any step in `bme280_probe()` fails (e.g., wrong Chip ID `-ENODEV` or bus NACK `-ENXIO`), or when `rmmod` unbinds the driver, the kernel device core automatically unwinds all `devm_*` resources in reverse registration order.

---

## 5. How does `sampling_frequency` control the actual hardware?
- In Normal Mode, the BME280 cycles between active measurement ($\approx 37.5\text{ ms}$) and standby ($t_{\text{standby}}$, configured in register `0xF5` bits `7:5`).
- We expose `sampling_frequency_available` (`"1 2 4 8 16 26"`) via `bme280_read_avail()`.
- When userspace writes a valid frequency (e.g., `8` Hz) to `sampling_frequency`, `bme280_write_raw()` maps `8 Hz -> BME280_STANDBY_125_MS (0x02)`. Because Bosch Datasheet Section 5.4.6 specifies that writes to `0xF5` in Normal Mode may be ignored, `bme280_reprogram_standby_locked()` transitions `0xF4` to `MODE_SLEEP`, writes the new standby code to `0xF5`, and restores `MODE_NORMAL` in `0xF4`.

---

## 6. Why is fixed-point arithmetic used for calibration compensation?
- Floating-point operations are avoided in the Linux kernel because saving and restoring FPU/SIMD registers across kernel traps adds overhead and isn't supported on all architectures without explicit `kernel_fpu_begin()`/`end()` guards.
- We implement Bosch's 32-bit (`s32`) and 64-bit (`s64`) integer fixed-point compensation algorithms using `<linux/math64.h>` (`div64_s64` and `div_u64`). Pressure is computed in `Q24.8` Pascals (`Pa * 256`) and returned to the IIO core as `IIO_VAL_FRACTIONAL` with denominator `256000` (`256 * 1000 Pa/kPa`), yielding exact kilopascals without floating-point instructions.
