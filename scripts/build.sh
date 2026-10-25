#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Unified Build & Verification Script for Embedded Linux BME280 I2C Driver Platform

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_USERSPACE=1
BUILD_DRIVER=1
BUILD_DTS=1
RUN_TESTS=1

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --all            Build userspace, DT overlay, kernel driver (if headers exist), and run tests (default)
  --userspace-only Build only the C userspace telemetry application
  --check-only     Compile userspace C app, validate kernel C syntax, and run pytest suite
  --clean          Clean all build artifacts
  -h, --help       Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all)
            BUILD_USERSPACE=1; BUILD_DRIVER=1; BUILD_DTS=1; RUN_TESTS=1
            shift ;;
        --userspace-only)
            BUILD_USERSPACE=1; BUILD_DRIVER=0; BUILD_DTS=0; RUN_TESTS=0
            shift ;;
        --check-only)
            BUILD_USERSPACE=1; BUILD_DRIVER=0; BUILD_DTS=1; RUN_TESTS=1
            shift ;;
        --clean)
            make -C "${REPO_ROOT}/userspace" clean
            make -C "${REPO_ROOT}/driver" clean
            rm -f "${REPO_ROOT}/dts/"*.dtbo
            echo "[OK] Cleaned build artifacts."
            exit 0 ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "[ERROR] Unknown option: $1" >&2
            usage; exit 1 ;;
    esac
done

echo "=================================================================="
echo " Embedded Linux BME280 I2C Driver & Validation Platform - Builder "
echo "=================================================================="

if [[ "${BUILD_USERSPACE}" -eq 1 ]]; then
    echo "[1/4] Building C userspace application (userspace/sensor_reader)..."
    make -C "${REPO_ROOT}/userspace" clean
    make -C "${REPO_ROOT}/userspace"
    echo "[OK] Built ${REPO_ROOT}/userspace/sensor_reader"
fi

if [[ "${BUILD_DTS}" -eq 1 ]]; then
    echo "[2/4] Checking Device Tree Overlay (dts/sensor-overlay.dts)..."
    if command -v dtc >/dev/null 2>&1; then
        dtc -@ -I dts -O dtb -o "${REPO_ROOT}/dts/sensor-overlay.dtbo" "${REPO_ROOT}/dts/sensor-overlay.dts"
        echo "[OK] Compiled ${REPO_ROOT}/dts/sensor-overlay.dtbo"
    else
        echo "[INFO] 'dtc' (Device Tree Compiler) not installed on this host; skipping .dtbo binary generation."
    fi
fi

if [[ "${BUILD_DRIVER}" -eq 1 ]]; then
    echo "[3/4] Building Linux kernel driver (driver/sensor_driver.c)..."
    make -C "${REPO_ROOT}/driver"
fi

if [[ "${RUN_TESTS}" -eq 1 ]]; then
    echo "[4/4] Running automated pytest regression suite..."
    if command -v pytest >/dev/null 2>&1; then
        PYTHONPATH="${REPO_ROOT}" pytest -v "${REPO_ROOT}/tests/test_regression.py"
    else
        PYTHONPATH="${REPO_ROOT}" python3 "${REPO_ROOT}/tests/test_regression.py" -v
    fi
fi

echo "=================================================================="
echo " Build and validation completed!"
echo "=================================================================="
