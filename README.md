# Embedded Linux I²C Sensor Driver and Validation Platform (`bme280-custom-iio`)

[![CI - Kernel Module Build, DT Overlay & Automated Validation](https://github.com/nikunjmalik306/embedded-linux-i2c-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/nikunjmalik306/embedded-linux-i2c-driver/actions)
[![License: GPL v2 / MIT](https://img.shields.io/badge/License-GPL%20v2%20%2F%20MIT-blue.svg)](LICENSE)
[![Target: Raspberry Pi 5 / Linux 6.1–6.12](https://img.shields.io/badge/Target-Raspberry%20Pi%205%20(Linux%206.1--6.12)-red.svg)](docs/wiring.md)

An end-to-end **Embedded Linux I²C Sensor Driver, Device Tree Overlay, Userspace IIO Client, and Automated Fault-Injection Validation Platform** targeting the **Bosch BME280** environmental sensor on the **Raspberry Pi 5**.

This project builds on bare-metal/RTOS multi-sensor acquisition concepts from [`stm32f103-freertos-multisensor-daq`](https://github.com/nikunjmalik306/stm32f103-freertos-multisensor-daq) and transitions into the **Linux Kernel Driver & Board Support Package (BSP)** layer: demonstrating bus enumeration via Device Tree, kernel resource management (`devm_*`), SMBus block register access, mutex-synchronized calibration compensation in the **Linux Industrial I/O (IIO)** subsystem, a C11 userspace telemetry client, and an automated Python regression harness with deterministic fault injection.

---

## Table of Contents

1. [Project Objectives & Engineering Scope](#1-project-objectives--engineering-scope)
2. [Custom Implementation vs. Existing Linux Infrastructure](#2-custom-implementation-vs-existing-linux-infrastructure)
3. [Supported Kernel Versions & Target Environment](#3-supported-kernel-versions--target-environment)
4. [System Architecture & Data Flow](#4-system-architecture--data-flow)
5. [Repository Structure](#5-repository-structure)
6. [Build & Deployment Instructions](#6-build--deployment-instructions)
7. [Verification Status: Software Simulation (CI) vs. Physical Hardware (HIL)](#7-verification-status-software-simulation-ci-vs-physical-hardware-hil)
8. [Failure Cases, Known Limitations & Debugging Notes](#8-failure-cases-known-limitations--debugging-notes)
9. [Primary Technical References](#9-primary-technical-references)

---

## 1. Project Objectives & Engineering Scope

Rather than reading an I²C sensor from userspace via `/dev/i2c-X`, this platform implements the complete kernel-to-userspace driver stack required for **Embedded Linux, BSP, Driver Development, Firmware Validation, and Board Bring-up** roles:

- **Device Tree Integration:** Out-of-tree Device Tree Overlay (`dts/sensor-overlay.dts`) and YAML schema (`dts/bindings/custom,bme280-sensor.yaml`) binding compatible string `"custom,bme280-sensor"` at bus address `0x76` with configurable oversampling, IIR filter, and standby timer properties.
- **Linux Kernel I²C Client & IIO Driver (`driver/sensor_driver.c`):**
  - Implements `struct i2c_driver` with cross-kernel version compatibility (`LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)` probe/remove shims supporting Linux 5.15 LTS through 6.12+).
  - Verifies hardware identity (`0xD0 == 0x60`), issues soft-reset (`0xE0 = 0xB6`), and parses 33 bytes of factory trimming parameters (`dig_T1..T3`, `dig_P1..P9`, `dig_H1..H6`) using `<asm/unaligned.h>` (`get_unaligned_le16`) and `sign_extend32()` for signed 12-bit `dig_H4`/`dig_H5`.
  - Exposes standard **Linux Industrial I/O (IIO)** channels (`in_temp_input`, `in_pressure_input`, `in_humidityrelative_input`, `sampling_frequency`, and `sampling_frequency_available`).
  - **Hardware-Backed `sampling_frequency` Control:** Writing to `sampling_frequency` (`1, 2, 4, 8, 16, 26 Hz`) transitions the BME280 into Sleep Mode (`0xF4 = 0x00`), reprograms the hardware standby timer `t_sb[2:0]` in `CONFIG` (`0xF5`), and restores Normal Mode per Bosch Datasheet Section 5.4.6.
  - Enforces concurrency safety using `struct mutex lock` across 8-byte atomic burst reads (`0xF7..0xFE`) and shared `t_fine` thermal compensation state.
- **Userspace C11 Application (`userspace/sensor_reader.c`):**
  - Discovers the sensor dynamically via `/sys/bus/iio/devices/`, handles both kernel-evaluated `IIO_VAL_FRACTIONAL` decimal strings (`"100.652343"`) and rational fractions (`"25766999/256000"`), supports runtime `--set-freq <Hz>`, and logs latency/jitter statistics in `text`, `json`, or `csv`.
- **Automated Python Test Harness (`tests/test_regression.py`):**
  - Strictly separates **Tier 1 Simulated Unit & Fault-Injection Tests** (executed on every push in GitHub Actions along with a real `sensor_driver.ko` kernel build) from **Tier 2 Hardware-in-the-Loop (HIL) Target Tests** (`BME280_HIL=1`).

---

## 2. Custom Implementation vs. Existing Linux Infrastructure

| Layer / Component | Provided by Upstream Linux Kernel / OS | Implemented Custom in This Repository |
| :--- | :--- | :--- |
| **I²C Bus Controller** | `i2c-designware` / RP1 I²C master driver, `i2c-core`, `/dev/i2c-1` | None (uses standard kernel `i2c_smbus_*` APIs) |
| **Device Tree Infrastructure** | Linux OF (`of_match_table`) parser, `dtoverlay` loader, `dtc` compiler | Custom overlay `dts/sensor-overlay.dts` & binding `dts/bindings/custom,bme280-sensor.yaml` (`"custom,bme280-sensor"`) |
| **IIO Framework** | `industrialio` core (`devm_iio_device_alloc`, sysfs attribute formatting) | `driver/sensor_driver.c`: IIO channel specs, `read_raw`/`write_raw`/`read_avail`, `0xF5` standby reprogramming, burst read & Bosch 32/64-bit fixed-point compensation |
| **Fault Injection & Diagnostics** | `debugfs` virtual filesystem API | Custom `debugfs` nodes (`inject_error`, `total_reads`, `io_errors`, `stats`) in `driver/sensor_driver.c` |
| **Userspace Telemetry** | `glibc`, POSIX file I/O, `clock_gettime`, `clock_nanosleep` | `userspace/sensor_reader.c`: dynamic IIO device resolver, fractional/decimal parser, monotonic jitter analyzer, JSON/CSV logger |
| **Validation & CI** | `pytest`/`unittest`, `PyYAML`, GitHub Actions Ubuntu 24.04 runner | `tests/test_regression.py`, `tests/fixtures/bme280_sim.py` (fixed-point + IEEE-754 double-precision Bosch oracle), `tests/test_plan.yaml` |

> **Why `"custom,bme280-sensor"` instead of upstream `"bosch,bme280"`?**
> Upstream Linux includes `drivers/iio/pressure/bmp280-i2c.c` matching `"bosch,bme280"`. Using a distinct compatible string (`"custom,bme280-sensor"`) allows loading and validating our custom driver side-by-side without module symbol or device-matching conflicts against the in-tree `bmp280` driver.

---

## 3. Supported Kernel Versions & Target Environment

| Item | Specification / Version |
| :--- | :--- |
| **Supported Linux Kernels** | **Linux 5.15 LTS, 6.1 LTS, 6.6 LTS (Raspberry Pi OS Bookworm), 6.8 (Ubuntu 24.04 CI), and 6.12+** via `<linux/version.h>` conditional `probe`/`remove` shims |
| **Primary Hardware Target** | Raspberry Pi 5 Model B (Broadcom BCM2712 + RP1 Southbridge, `aarch64`) |
| **CI Build & Verification Host** | GitHub Actions `ubuntu-24.04` (`x86_64`, `linux-headers-$(uname -r)`, `gcc`, `dtc`, `pytest`, `valgrind`) |
| **Sensor Hardware** | Bosch BME280 Digital Humidity, Pressure & Temperature Sensor Breakout (3.3V VDD/VDDIO, `SDO -> GND` for `0x76`) |

---

## 4. System Architecture & Data Flow

```
+-----------------------------------------------------------------------------------+
|                                 USERSPACE LAYER                                   |
|                                                                                   |
|  +-------------------------------+         +-----------------------------------+  |
|  |   userspace/sensor_reader     |         |     tests/test_regression.py      |  |
|  |  - Dynamic IIO path discovery |         |  - Tier 1: Simulated + Math Oracle|  |
|  |  - Fractional/decimal parser  |         |  - Tier 2: Target HIL (BME280_HIL)|  |
|  |  - Monotonic polling & jitter |         |  - Fault injection & stress tests |  |
|  +---------------+---------------+         +-----------------+-----------------+  |
|                  |                                           |                    |
|                  | open() / read() / write()                 | debugfs control    |
+------------------|-------------------------------------------|--------------------+
                   v                                           v
+-----------------------------------------------------------------------------------+
|                              LINUX KERNEL SPACE                                   |
|                                                                                   |
|  /sys/bus/iio/devices/iio:device0/           /sys/kernel/debug/bme280_custom/     |
|   ├── name ("bme280_custom")                  ├── inject_error (0, -EIO, -ETIMEDOUT)|
|   ├── in_temp_input (milli-°C)                ├── total_reads                     |
|   ├── in_pressure_input (kPa, FRACTIONAL)     ├── io_errors                       |
|   ├── in_humidityrelative_input (milli-%RH)   └── stats (registers, calib, last)  |
|   ├── sampling_frequency (1..26 Hz -> 0xF5)                                       |
|   └── sampling_frequency_available ("1 2 4 8 16 26")                              |
|                  ^                                           ^                    |
|                  +---------------------+---------------------+                    |
|                                        |                                          |
|                  +---------------------+---------------------+                    |
|                  |      driver/sensor_driver.c (IIO)         |                    |
|                  |  - bme280_read_raw() / mutex_lock(&lock)  |                    |
|                  |  - bme280_reprogram_standby_locked(0xF5)  |                    |
|                  |  - 8-byte atomic burst read (0xF7..0xFE)  |                    |
|                  |  - Bosch fixed-point calibration engine   |                    |
|                  +---------------------+---------------------+                    |
|                                        | i2c_smbus_read_i2c_block_data()          |
|                  +---------------------+---------------------+                    |
|                  |       Linux I2C Subsystem (i2c-core)      |                    |
|                  +---------------------+---------------------+                    |
+----------------------------------------|------------------------------------------+
                                         | Physical I2C1 Bus (SDA: GPIO2, SCL: GPIO3)
                                         v
+-----------------------------------------------------------------------------------+
|                                HARDWARE LAYER                                     |
|                         Bosch BME280 Sensor (Addr: 0x76)                          |
|     [0xD0: Chip ID 0x60]  [0x88..0xA1, 0xE1..0xE7: Calib]  [0xF7..0xFE: ADC]      |
+-----------------------------------------------------------------------------------+
```

Detailed architectural design, locking semantics, and compensation equations are documented in [`docs/architecture.md`](docs/architecture.md) and [`docs/interview-guide.md`](docs/interview-guide.md).

---

## 5. Repository Structure

```text
embedded-linux-i2c-driver/
├── README.md                        # Project overview, architecture, and verification status
├── LICENSE                          # Dual GPL-2.0 (kernel) / MIT (userspace & tests) license
├── .gitignore                       # Kernel build, DTBO, C binary, and Python artifacts ignore rules
├── docs/
│   ├── architecture.md              # Subsystem design, state machine, locking, and Bosch math
│   ├── wiring.md                    # Pinout, 3.3V electrical verification, and Phase 1 bring-up guide
│   ├── test-plan.md                 # Test matrix separating Tier 1 (Sim/CI) and Tier 2 (HIL)
│   ├── results.md                   # Executed CI/simulation test logs + HIL execution template
│   └── interview-guide.md           # Deep-dive Q&A for kernel/BSP technical interviews
├── driver/
│   ├── Makefile                     # Kbuild out-of-tree kernel module Makefile
│   ├── sensor_driver.h              # BME280 register map, bitfields, calibration structs, limits
│   └── sensor_driver.c              # Linux I2C client + IIO driver with 0xF5 standby control & debugfs
├── dts/
│   ├── sensor-overlay.dts           # Raspberry Pi 5 Device Tree Overlay for i2c1 @ 0x76
│   └── bindings/
│       └── custom,bme280-sensor.yaml # Devicetree YAML binding specification
├── userspace/
│   ├── Makefile                     # Userspace C11 build with -Wall -Wextra -Werror -pedantic
│   └── sensor_reader.c              # C CLI reader with decimal/fractional parser, jitter & JSON/CSV
├── tests/
│   ├── test_plan.yaml               # Declarative test specification (bounds, fault codes, SLA targets)
│   ├── test_regression.py           # Automated suite (10 test cases + double-precision math oracle)
│   └── fixtures/
│       ├── __init__.py
│       ├── bme280_sim.py            # Register-accurate BME280 simulator & floating-point oracle
│       └── sample_calibration.json  # Factory calibration parameters and golden test vectors
├── scripts/
│   ├── build.sh                     # Unified build script (driver .ko, DTBO, userspace, tests)
│   └── collect_logs.sh              # Target diagnostic collector (dmesg, i2cdetect, sysfs, debugfs)
└── .github/
    └── workflows/
        └── ci.yml                   # GitHub Actions CI building sensor_driver.ko, DTBO, C app & tests
```

---

## 6. Build & Deployment Instructions

### 6.1 Build All Components (Kernel `.ko`, DT Overlay, Userspace, and Tests)

```bash
# Install dependencies (Raspberry Pi OS / Debian / Ubuntu)
sudo apt-get update
sudo apt-get install -y build-essential linux-headers-$(uname -r) \
    device-tree-compiler i2c-tools python3 python3-pytest python3-yaml

# Build kernel module, DT overlay, userspace binary, and run automated tests
./scripts/build.sh --all
```

### 6.2 Deploying on Raspberry Pi 5 Hardware (HIL Target)

```bash
# 1. Confirm I2C1 bus is enabled and BME280 responds at 0x76 with Chip ID 0x60
i2cdetect -y 1
i2cget -y 1 0x76 0xD0 b

# 2. Load the Device Tree Overlay and insert the compiled kernel module
sudo dtoverlay dts/sensor-overlay.dtbo
sudo insmod driver/sensor_driver.ko

# 3. Verify driver binding, IIO sysfs creation, and available sampling frequencies
dmesg | tail -n 10
cat /sys/bus/iio/devices/iio:device0/name
cat /sys/bus/iio/devices/iio:device0/sampling_frequency_available

# 4. Reprogram hardware sampling rate to 8 Hz (t_sb = 125 ms in 0xF5) and poll 10 samples
./userspace/sensor_reader --set-freq 8 --count 10 --interval-ms 125 --format text --stats

# 5. Collect target diagnostic bundle (uname, dmesg, i2cdetect, sysfs, debugfs)
./scripts/collect_logs.sh
```

---

## 7. Verification Status: Software Simulation (CI) vs. Physical Hardware (HIL)

To maintain strict engineering transparency, test results are separated into **Tier 1 (Executed & Verified in Automated Simulation / CI)** and **Tier 2 (Physical Raspberry Pi 5 Hardware-in-the-Loop Verification)**:

### Tier 1 — Executed & Verified in Automated CI / Software Simulation

All Tier 1 tests run automatically via `./scripts/build.sh --check-only` and in GitHub Actions (`.github/workflows/ci.yml`):

| Test ID | Verification Scope | Execution Environment | Status |Measured / Verified Outcome |
| :--- | :--- | :--- | :---: | :--- |
| `BUILD-KO` | Out-of-tree `sensor_driver.ko` compilation | GitHub Actions (`linux-headers-$(uname -r)`) | **VERIFIED (CI)** | Clean Kbuild `.ko` compilation, zero unresolved symbols |
| `BUILD-DTBO` | Device Tree Overlay `sensor-overlay.dtbo` | `dtc -@ -I dts -O dtb` | **VERIFIED (CI)** | Compiles cleanly with `__overrides__` symbol table |
| `TC-001` | Valid Chip ID (`0x60`) probe path | `test_regression.py` (`BME280Simulator`) | **PASSED** | `probe_check()` returns `0` |
| `TC-002` | Invalid Chip ID rejection (`0x00`, `0x58`, `0x61`, `0xFF`) | `test_regression.py` (`BME280Simulator`) | **PASSED** | Rejects non-BME280 IDs with `-ENODEV` (`-19`) |
| `TC-003` | Missing slave NACK & corrupted `dig_T1=0` | `test_regression.py` (`BME280Simulator`) | **PASSED** | Returns `-ENXIO` (`-6`) and `-EINVAL` (`-22`) |
| `TC-004` | Fixed-point Bosch math vs. IEEE-754 double oracle | `test_regression.py` (38 ADC combinations) | **PASSED** | Matches Datasheet 8.1 float oracle within `±0.02 °C`, `±0.01 kPa`, `±0.15 %RH` |
| `TC-005` | Mid-transfer `-EIO` & POR sentinel (`0x80000`) | `test_regression.py` (`BME280Simulator`) | **PASSED** | Returns `-EIO` (`-5`) and `-ENODATA` (`-61`); no stale data |
| `TC-006` | Hardware `sampling_frequency` (`0xF5` `t_sb[2:0]`) | `test_regression.py` (`1, 2, 4, 8, 16, 26 Hz`) | **PASSED** | Updates `0xF5` bits `7:5` via Sleep Mode; rejects invalid Hz with `-EINVAL` |
| `TC-007` | Userspace C parser (`IIO_VAL_FRACTIONAL` & `A/B`) | `sensor_reader` + mock sysfs | **PASSED** | Parses `"100.652343"` and `"25766999/256000"` identically (`100.6523 kPa`) |
| `TC-008` | Userspace CLI fail-fast on `-EIO` | `sensor_reader --fail-fast` | **PASSED** | Exits non-zero and logs descriptive `errno` to `stderr` |
| `TC-009` | 4-process concurrent open/read/close stress | `ThreadPoolExecutor` (100 total runs) | **PASSED** | `100/100` valid JSON frames, zero race conditions |
| `TC-010` | Kernel & DT static audit compliance | Source AST/token audit | **PASSED** | Verifies `LINUX_VERSION_CODE` shim, `get_unaligned_le16`, `sign_extend32` |

### Tier 2 — Physical Raspberry Pi 5 Hardware-in-the-Loop (HIL) Validation Matrix

Tier 2 tests (`test_hil_001_sysfs_iio_device_present` and hardware latency/jitter benchmarks) require a physical Raspberry Pi 5 wired to a Bosch BME280 breakout (`BME280_HIL=1`). See [`docs/results.md`](docs/results.md) for the complete command sequence and log capture procedure using `./scripts/collect_logs.sh`.

---

## 8. Failure Cases, Known Limitations & Debugging Notes

1. **Thermal Order-of-Operations Dependency (`t_fine`):**
   - *Hardware behavior:* Bosch BME280 pressure and humidity compensation formulas require `t_fine`, computed from the current temperature ADC reading.
   - *Mitigation:* `bme280_read_burst_locked()` always performs an atomic 8-byte burst read (`0xF7..0xFE`) and computes `t_fine` first under `mutex_lock(&data->lock)`.
2. **Reprogramming `CONFIG` (`0xF5`) in Normal Mode:**
   - *Hardware behavior:* Per Bosch Datasheet Section 5.4.6, writes to `0xF5` (`t_sb` / `filter`) during Normal Mode may be ignored by the BME280 state machine.
   - *Mitigation:* `bme280_reprogram_standby_locked()` explicitly writes `BME280_MODE_SLEEP` (`0x00`) to `ctrl_meas` (`0xF4`), writes the new `config` byte to `0xF5`, and then restores `BME280_MODE_NORMAL` (`0x03`) in `0xF4`.
3. **Power-On-Reset Sentinel (`0x80000`):**
   - *Hardware behavior:* Before the first conversion completes or after a brownout reset, ADC registers contain `0x80000` (`T`/`P`) or `0x8000` (`H`).
   - *Mitigation:* The driver checks for these sentinel values before running compensation math and returns `-ENODATA`.

---

## 9. Primary Technical References

1. [Linux Kernel Documentation — Implementing I²C device drivers](https://docs.kernel.org/i2c/writing-clients.html)
2. [Bootlin — Linux Kernel and Driver Development Training Materials](https://bootlin.com/training/kernel/) ([GitHub Repo](https://github.com/bootlin/training-materials))
3. [Linux Kernel Documentation — Linux and the Devicetree](https://docs.kernel.org/devicetree/usage-model.html)
4. [Linux Kernel Documentation — Industrial I/O (IIO) Subsystem](https://docs.kernel.org/driver-api/iio/index.html)
5. [Linux Kernel Documentation — Linux Kernel Coding Style](https://docs.kernel.org/process/coding-style.html)
6. [Bosch Sensortec — BME280 Combined Humidity and Pressure Sensor Datasheet (BST-BME280-DS002)](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
7. [Linux Foundation — Developing Embedded Linux Device Drivers (LFD435)](https://training.linuxfoundation.org/training/developing-embedded-linux-device-drivers/)
