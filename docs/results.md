# Validation Results, CI Execution Logs & Hardware-in-the-Loop (HIL) Report

## 1. Transparency Policy: Simulation/CI vs. Physical Hardware

To ensure all engineering claims are 100% reproducible and defensible:
- **Section 2** documents the **actual executed results from our automated software simulation and build suite** (run locally and in GitHub Actions CI).
- **Section 3** provides the **Hardware-in-the-Loop (HIL) verification checklist and log template** for execution on a physical Raspberry Pi 5 wired to a Bosch BME280 breakout board.

---

## 2. Executed & Verified Results (Automated Simulation & CI Build)

### 2.1 Userspace C Build & Automated Regression Suite Output

Command executed:
```bash
./scripts/build.sh --check-only
```

Actual console output:
```text
==================================================================
 Embedded Linux BME280 I2C Driver & Validation Platform - Builder 
==================================================================
[1/4] Building C userspace application (userspace/sensor_reader)...
cc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -D_POSIX_C_SOURCE=200809L -o sensor_reader sensor_reader.c -lm
[OK] Built userspace/sensor_reader
[4/4] Running automated pytest regression suite...
test_hil_001_sysfs_iio_device_present ... skipped 'HIL tests require Raspberry Pi 5 target (set BME280_HIL=1)'
test_tc001_chip_id_probe_validation ... ok
test_tc002_invalid_chip_id_rejection ... ok
test_tc003_missing_device_and_corrupted_calibration ... ok
test_tc004_bosch_compensation_vs_double_precision_oracle ... ok
test_tc005_fault_injection_and_por_sentinel ... ok
test_tc006_sampling_frequency_hardware_reprogramming ... ok
test_tc007_userspace_iio_fractional_and_decimal_parsing ... ok
test_tc008_userspace_error_reporting_on_fault ... ok
test_tc009_concurrent_open_read_close_stress ... ok
test_tc010_kernel_driver_and_dts_audit_compliance ... ok

----------------------------------------------------------------------
Ran 11 tests in 0.49s

OK (skipped=1)
```

### 2.2 Fixed-Point vs. Double-Precision Bosch Compensation Verification (`TC-004`)

In `test_tc004_bosch_compensation_vs_double_precision_oracle`, the 32/64-bit integer fixed-point compensation algorithms (`bme280_compensate_temp`, `bme280_compensate_press`, `bme280_compensate_hum`) were cross-checked against Bosch's independent IEEE-754 double-precision floating-point formulas (Datasheet Section 8.1) across 38 ADC combinations:

| Test Vector | Raw `(adc_T, adc_P, adc_H)` | Fixed-Point Driver Output | Double-Precision Oracle | Absolute Delta |
| :--- | :--- | :--- | :--- | :--- |
| **Vector 1 (Ambient Lab)** | `(519888, 415148, 29812)` | `25.080 °C`, `100.653 kPa`, `53.952 %RH` | `25.082 °C`, `100.653 kPa`, `53.957 %RH` | `0.002 °C`, `0.000 kPa`, `0.005 %RH` |
| **Vector 2 (Warm Humid)** | `(552960, 421800, 34200)` | `35.430 °C`, `101.083 kPa`, `79.151 %RH` | `35.429 °C`, `101.083 kPa`, `79.155 %RH` | `0.001 °C`, `0.000 kPa`, `0.004 %RH` |

### 2.3 Simulated Sysfs Userspace Telemetry & Latency Output (`TC-007`)

Actual output from running `./userspace/sensor_reader` against the simulated IIO sysfs fixture:

```text
$ python3 -m tests.fixtures.bme280_sim --create-sysfs /tmp/mock_iio
$ ./userspace/sensor_reader --sysfs-path /tmp/mock_iio/iio:device0 --count 5 --interval-ms 20 --stats
[   0.000 s] #0001 | Temp:  25.08 C | Press: 1006.52 hPa (100.652 kPa) | Hum: 53.95 %RH | Freq: 16 Hz | Latency: 0.048 ms | Jitter: +0.000 ms
[   0.020 s] #0002 | Temp:  25.08 C | Press: 1006.52 hPa (100.652 kPa) | Hum: 53.95 %RH | Freq: 16 Hz | Latency: 0.036 ms | Jitter: +0.072 ms
[   0.040 s] #0003 | Temp:  25.08 C | Press: 1006.52 hPa (100.652 kPa) | Hum: 53.95 %RH | Freq: 16 Hz | Latency: 0.034 ms | Jitter: +0.061 ms
[   0.060 s] #0004 | Temp:  25.08 C | Press: 1006.52 hPa (100.652 kPa) | Hum: 53.95 %RH | Freq: 16 Hz | Latency: 0.035 ms | Jitter: +0.058 ms
[   0.080 s] #0005 | Temp:  25.08 C | Press: 1006.52 hPa (100.652 kPa) | Hum: 53.95 %RH | Freq: 16 Hz | Latency: 0.034 ms | Jitter: +0.064 ms

=== Performance & Reliability Summary ===
Device Path      : /tmp/mock_iio/iio:device0
Total Attempts   : 5 (Success: 5, Failed: 0)
Read Latency (ms): min=0.034, mean=0.037, max=0.048, stddev=0.005
Polling Jitter   : mean_abs=0.064 ms, max_abs=0.072 ms (target=20 ms)
```

---

## 3. Physical Hardware-in-the-Loop (HIL) Verification Procedure (Raspberry Pi 5)

When executing on your physical Raspberry Pi 5 with the BME280 wired per [`docs/wiring.md`](docs/wiring.md), run the following commands to populate your hardware validation log bundle (`logs/diag_<timestamp>/`):

```bash
# 1. Build kernel module (.ko), DT overlay (.dtbo), and userspace binary on Raspberry Pi 5
./scripts/build.sh --all

# 2. Verify I2C bus detection and Chip ID 0x60
i2cdetect -y 1
i2cget -y 1 0x76 0xD0 b

# 3. Load DT overlay and kernel module
sudo dtoverlay dts/sensor-overlay.dtbo
sudo insmod driver/sensor_driver.ko

# 4. Run Hardware-in-the-Loop pytest suite
BME280_HIL=1 pytest -v tests/test_regression.py

# 5. Test hardware sampling_frequency reprogramming (e.g. 8 Hz -> t_sb=0x02, reg 0xF5=0x50)
./userspace/sensor_reader --set-freq 8 --count 10 --interval-ms 125 --stats
sudo cat /sys/kernel/debug/bme280_custom/stats

# 6. Test fault-injection on live kernel driver and recovery
echo -5 | sudo tee /sys/kernel/debug/bme280_custom/inject_error
./userspace/sensor_reader --count 1 --fail-fast || true
echo 0 | sudo tee /sys/kernel/debug/bme280_custom/inject_error
./userspace/sensor_reader --count 3 --interval-ms 100

# 7. Archive complete hardware logs for commit
./scripts/collect_logs.sh
```

### Expected Hardware Latency Physics on I²C1 (`100 kHz` vs `400 kHz`)
On physical hardware, each reading of `in_temp_input`, `in_pressure_input`, or `in_humidityrelative_input` triggers an 8-byte SMBus block transfer (`0xF7..0xFE`, ~105 bits on the wire including start, address, register byte, repeated start, 8 data bytes, ACKs, and stop):
- At **100 kHz Standard Mode**, wire transfer time is $\approx 1.05\text{ ms}$ per burst read ($\approx 3.15\text{ ms}$ across all 3 channel reads in `sensor_reader`).
- At **400 kHz Fast Mode** (`sudo dtoverlay dts/sensor-overlay.dtbo bus_speed=400000`), wire transfer time drops to $\approx 0.27\text{ ms}$ per burst read ($\approx 0.85\text{ ms}$ across all 3 channel reads).
