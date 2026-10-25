# Validation & Fault-Injection Test Plan

## 1. Validation Philosophy

A production-grade kernel driver must be validated not only on the "happy path" (printing room temperature) but specifically under **hardware faults, bus errors, corrupted register states, concurrent userspace contention, and repeated module lifecycle transitions**.

Our validation strategy splits testing into two tiers:
1. **Tier 1 — Simulated Unit & Fault-Injection Tests (Hardware-Independent / CI):**
   - Runs automatically on every commit in GitHub Actions or any Linux VM via `pytest tests/test_regression.py`.
   - Uses `tests/fixtures/bme280_sim.py` to test the exact Bosch fixed-point math, register boundary conditions, error propagation, and multi-threaded stress on `userspace/sensor_reader`.
2. **Tier 2 — Hardware-in-the-Loop (HIL) Target Tests (`BME280_HIL=1` on Raspberry Pi 5):**
   - Exercises physical I²C transactions on `/dev/i2c-1`, `dtoverlay` device matching, `insmod`/`rmmod` lifecycle loops, `debugfs` kernel fault injection, and 24-hour soak telemetry.

---

## 2. Detailed Test Matrix

### 2.1 Normal Operation & Mathematical Verification

| Test ID | Objective | Stimulus / Input | Pass Criteria |
| :--- | :--- | :--- | :--- |
| `TC-001` | Device Tree Matching & Probe | Load `sensor-overlay.dtbo` and `sensor_driver.ko` | `dmesg` logs `Chip ID 0x60, IIO registered`; `/sys/bus/iio/devices/iio:device0/name` == `bme280_custom` |
| `TC-004` | Bosch Compensation Accuracy | Apply factory NVM trimming coefficients (`sample_calibration.json`) and raw ADC vectors | Compensated `T`, `P`, `H` match reference within `±0.01 °C`, `±0.05 kPa`, `±0.05 %RH` |
| `TC-006` | Structured Telemetry Output | Execute `sensor_reader --count 5 --format json` and `--format csv` | Valid JSON/CSV schema, physical bounds `-40..85 °C`, `30..110 kPa`, `0..100 %RH` |

### 2.2 Negative Testing & Fault Injection

| Test ID | Fault Scenario | Injection Mechanism | Expected Kernel & Userspace Behavior |
| :--- | :--- | :--- | :--- |
| `TC-002A` | Wrong Sensor Model (BMP280 `0x58`) | `sudo insmod sensor_driver.ko override_chip_id=0x58` | `bme280_probe()` logs `Unsupported Chip ID 0x58` and returns `-ENODEV` (`-19`); no IIO node created |
| `TC-002B` | Floating Bus (`0xFF`) | `sudo insmod sensor_driver.ko override_chip_id=0xFF` | `bme280_probe()` returns `-ENODEV` (`-19`) cleanly |
| `TC-003` | Missing Hardware / Unplugged Sensor | Load overlay with `addr=0x77` (unpopulated address) | `i2c_smbus_read_byte_data` NACKs (`-EREMOTEIO` / `-ENXIO`); `devm_*` frees all memory with 0 leaks |
| `TC-005A` | Mid-Operation I²C Bus Error | `echo -5 | sudo tee /sys/kernel/debug/bme280_custom/inject_error` | `bme280_read_raw()` increments `io_errors` and returns `-EIO`; `sensor_reader` prints `Input/output error (errno=5)` and never emits stale cached data |
| `TC-005B` | Sensor Brownout / Uninitialized ADC (`0x80000`) | Sensor reset to sleep mode (`ctrl_meas = 0x00`) mid-run | Driver detects `adc_T == 0x80000`, logs rate-limited warning, and returns `-ENODATA` (`-61`) |

### 2.3 Concurrency, Lifecycle & Long-Duration Reliability

| Test ID | Objective | Procedure | Pass Criteria |
| :--- | :--- | :--- | :--- |
| `TC-007` | Module Load/Unload Stress | Loop `insmod` -> read 5 samples -> `rmmod` for 50 iterations | 50/50 iterations succeed; zero kernel warnings, lockdep complaints, or `kmemleak` entries |
| `TC-008` | Multi-Process Concurrent Contention | Spawn 4 parallel `sensor_reader` processes reading 100 samples at 5 ms intervals | `mutex_lock(&data->lock)` serializes burst reads; 0 torn frames or corrupted `t_fine` calculations |
| `TC-009` | Latency, Jitter & 24h Soak | Run `sensor_reader --count 864000 --interval-ms 100 --stats` | Mean read latency `< 3.0 ms`, p99 jitter `< 1.0 ms`, 0 KB RSS memory leak |
