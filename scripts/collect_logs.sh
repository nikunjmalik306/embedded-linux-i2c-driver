#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Target Diagnostic & Evidence Collector for Raspberry Pi 5 + BME280

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${REPO_ROOT}/logs/diag_${TIMESTAMP}"

mkdir -p "${OUT_DIR}"

echo "[INFO] Collecting system, I2C, IIO, and kernel logs into ${OUT_DIR}..."

# 1. System & Kernel baseline
uname -a > "${OUT_DIR}/uname.txt" 2>&1
cat /proc/device-tree/model > "${OUT_DIR}/board_model.txt" 2>/dev/null || echo "Simulated/VM Host" > "${OUT_DIR}/board_model.txt"
lsmod | grep -E "sensor_driver|bme280|bmp280|industrialio|i2c" > "${OUT_DIR}/lsmod_i2c_iio.txt" 2>&1 || true

# 2. Kernel ring buffer logs for driver & I2C
dmesg | grep -i -E "bme280|i2c|iio" | tail -n 200 > "${OUT_DIR}/dmesg_bme280.log" 2>&1 || true

# 3. I2C Bus Scan (Bus 1)
if command -v i2cdetect >/dev/null 2>&1; then
    i2cdetect -y 1 > "${OUT_DIR}/i2cdetect_bus1.txt" 2>&1 || true
fi

# 4. IIO Sysfs Attribute Dump
for dev in /sys/bus/iio/devices/iio:device*; do
    if [[ -d "${dev}" ]]; then
        bname="$(basename "${dev}")"
        mkdir -p "${OUT_DIR}/${bname}"
        for attr in "${dev}"/*; do
            if [[ -f "${attr}" && -r "${attr}" ]]; then
                aname="$(basename "${attr}")"
                cat "${attr}" > "${OUT_DIR}/${bname}/${aname}.txt" 2>/dev/null || true
            fi
        done
    fi
done

# 5. Debugfs Telemetry Snapshot
if [[ -r /sys/kernel/debug/bme280_custom/stats ]]; then
    cat /sys/kernel/debug/bme280_custom/stats > "${OUT_DIR}/debugfs_stats.txt" 2>&1 || true
fi

echo "[OK] Diagnostic bundle saved to ${OUT_DIR}"
