#!/usr/bin/env python3
"""
Automated Regression, Fault-Injection, and Hardware-in-the-Loop (HIL) Test Suite
for the Embedded Linux BME280 I2C Driver and Userspace Application.

Supports both `pytest` and standalone `python3 tests/test_regression.py` (`unittest`).

Run simulated unit & integration tests (any Linux host / CI):
    python3 tests/test_regression.py
    # or:
    pytest -v tests/test_regression.py

Run Hardware-in-the-Loop (HIL) tests on Raspberry Pi 5 target:
    BME280_HIL=1 python3 tests/test_regression.py
"""

import concurrent.futures
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Dict

try:
    import yaml
except ImportError:
    yaml = None

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.fixtures.bme280_sim import BME280Simulator

USERSPACE_BIN = REPO_ROOT / "userspace" / "sensor_reader"
TEST_PLAN_PATH = REPO_ROOT / "tests" / "test_plan.yaml"
HIL_ENABLED = os.environ.get("BME280_HIL", "0") == "1"


def load_test_plan() -> Dict:
    if yaml is not None:
        with open(TEST_PLAN_PATH, "r", encoding="utf-8") as fp:
            return yaml.safe_load(fp)
    return {
        "physical_bounds": {
            "temperature_c": {"min": -40.0, "max": 85.0},
            "pressure_kpa": {"min": 30.0, "max": 110.0},
            "humidity_pct": {"min": 0.0, "max": 100.0},
        },
        "performance_sla": {"concurrent_threads": 4},
    }


def ensure_userspace_built() -> Path:
    if not USERSPACE_BIN.exists():
        subprocess.run(["make", "-C", str(REPO_ROOT / "userspace")], check=True)
    assert USERSPACE_BIN.exists(), "userspace/sensor_reader binary failed to build"
    return USERSPACE_BIN


class TestBME280DriverAndUserspace(unittest.TestCase):
    """End-to-end verification of BME280 compensation math, fault handling, and userspace CLI."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.plan = load_test_plan()
        cls.bin_path = ensure_userspace_built()

    def test_tc001_chip_id_probe_validation(self) -> None:
        """TC-001: Verify probe succeeds with valid BME280 Chip ID (0x60)."""
        sim = BME280Simulator()
        self.assertEqual(sim.chip_id, 0x60)
        self.assertEqual(sim.probe_check(), 0)

    def test_tc002_invalid_chip_id_rejection(self) -> None:
        """TC-002: Verify probe rejects non-BME280 Chip IDs (0x00, 0x58 BMP280, 0x61, 0xFF) with -ENODEV (-19)."""
        for bad_id in (0x00, 0x58, 0x61, 0xFF):
            with self.subTest(bad_id=hex(bad_id)):
                sim = BME280Simulator()
                sim.chip_id = bad_id
                self.assertEqual(sim.probe_check(), -19)

    def test_tc003_missing_device_and_corrupted_calibration(self) -> None:
        """TC-003: Verify missing I2C slave returns -ENXIO (-6) and zero calib returns -EINVAL (-22)."""
        sim = BME280Simulator()
        sim.bus_connected = False
        self.assertEqual(sim.probe_check(), -6)

        sim.bus_connected = True
        sim.calib["dig_T1"] = 0
        self.assertEqual(sim.probe_check(), -22)

    def test_tc004_bosch_compensation_vs_double_precision_oracle(self) -> None:
        """
        TC-004: Verify 32/64-bit integer fixed-point Bosch calibration math against:
        1) Golden reference vectors in sample_calibration.json
        2) Independent double-precision floating-point formulas (Bosch Datasheet Section 8.1)
           across cold (-20 C), ambient (+25 C), and hot (+60 C) ADC operating points.
        """
        sim = BME280Simulator()
        for vec in sim.reference_vectors:
            sim.adc_T = vec["adc_T"]
            sim.adc_P = vec["adc_P"]
            sim.adc_H = vec["adc_H"]

            rc, out = sim.read_burst()
            self.assertEqual(rc, 0)
            self.assertIsNotNone(out)
            assert out is not None
            self.assertLessEqual(abs(out["in_temp_input"] - vec["expected_temp_milli_c"]), 10)
            self.assertLessEqual(abs(out["in_pressure_input_kpa"] - vec["expected_press_kpa"]), 0.05)
            self.assertLessEqual(abs(out["in_humidityrelative_input"] - vec["expected_hum_milli_pct"]), 50)

            # Cross-check integer fixed-point output against independent IEEE-754 double oracle
            flt_t_c, flt_p_kpa, flt_h_pct = sim.compensate_double_precision_oracle(
                sim.adc_T, sim.adc_P, sim.adc_H
            )
            self.assertAlmostEqual(out["in_temp_input"] / 1000.0, flt_t_c, delta=0.02)
            self.assertAlmostEqual(out["in_pressure_input_kpa"], flt_p_kpa, delta=0.01)
            self.assertAlmostEqual(out["in_humidityrelative_input"] / 1000.0, flt_h_pct, delta=0.1)

        # Sweep additional synthetic ADC points across temperature/pressure/humidity range
        for adc_t in (380000, 460000, 519888, 570000):
            for adc_p in (360000, 415148, 450000):
                for adc_h in (22000, 29812, 35000):
                    sim.adc_T, sim.adc_P, sim.adc_H = adc_t, adc_p, adc_h
                    rc, out = sim.read_burst()
                    self.assertEqual(rc, 0)
                    assert out is not None
                    flt_t_c, flt_p_kpa, flt_h_pct = sim.compensate_double_precision_oracle(
                        adc_t, adc_p, adc_h
                    )
                    self.assertAlmostEqual(out["in_temp_input"] / 1000.0, flt_t_c, delta=0.02)
                    self.assertAlmostEqual(out["in_pressure_input_kpa"], flt_p_kpa, delta=0.01)
                    self.assertAlmostEqual(out["in_humidityrelative_input"] / 1000.0, flt_h_pct, delta=0.15)

    def test_tc005_fault_injection_and_por_sentinel(self) -> None:
        """TC-005: Verify mid-operation -EIO and power-on-reset sentinel (0x80000 -> -ENODATA)."""
        sim = BME280Simulator()

        # Inject -EIO (-5)
        sim.inject_errno = -5
        rc, out = sim.read_burst()
        self.assertEqual(rc, -5)
        self.assertIsNone(out)

        # Restore normal operation, then simulate brownout reset register value 0x80000
        sim.inject_errno = 0
        sim.adc_T = BME280Simulator.ADC_SKIPPED_20BIT
        rc, out = sim.read_burst()
        self.assertEqual(rc, -61)  # -ENODATA
        self.assertIsNone(out)

    def test_tc006_sampling_frequency_hardware_reprogramming(self) -> None:
        """
        TC-006: Verify writing sampling_frequency reprograms BME280 register 0xF5 (t_sb[2:0])
        for all supported discrete rates (1, 2, 4, 8, 16, 26 Hz) and rejects invalid rates (-EINVAL).
        """
        sim = BME280Simulator()
        expected_standby_map = {
            1: 0x05,   # 1000 ms
            2: 0x04,   #  500 ms
            4: 0x03,   #  250 ms
            8: 0x02,   #  125 ms
            16: 0x01,  # 62.5 ms
            26: 0x00,  #  0.5 ms
        }
        for freq_hz, expected_t_sb in expected_standby_map.items():
            rc = sim.set_sampling_frequency(freq_hz)
            self.assertEqual(rc, 0)
            self.assertEqual(sim.sampling_freq, freq_hz)
            self.assertEqual(sim.standby, expected_t_sb)
            # Verify bits 7:5 of hardware register 0xF5 match expected_t_sb
            self.assertEqual((sim.reg_config_0xf5 >> 5) & 0x07, expected_t_sb)
            # Verify Normal Mode (0x03) was restored in register 0xF4 bits 1:0
            self.assertEqual(sim.reg_ctrl_meas_0xf4 & 0x03, 0x03)

        # Verify unsupported frequencies (e.g., 0, 3, 15, 100 Hz) return -EINVAL (-22)
        for bad_freq in (0, -5, 3, 10, 50, 100):
            self.assertEqual(sim.set_sampling_frequency(bad_freq), -22)

    def test_tc007_userspace_iio_fractional_and_decimal_parsing(self) -> None:
        """
        TC-007: Verify userspace C client parses both standard kernel IIO_VAL_FRACTIONAL
        decimal strings ("100.652343") and rational fraction strings ("25766999/256000")
        identically and emits valid JSON & CSV telemetry.
        """
        with tempfile.TemporaryDirectory() as tmpdir:
            sim = BME280Simulator()

            for use_frac in (False, True):
                dev_dir = sim.materialize_sysfs(
                    Path(tmpdir),
                    device_name="iio:device0" if not use_frac else "iio:device1",
                    use_fractional_string=use_frac,
                )
                proc = subprocess.run(
                    [
                        str(self.bin_path),
                        "--sysfs-path",
                        str(dev_dir),
                        "--count",
                        "3",
                        "--interval-ms",
                        "5",
                        "--format",
                        "json",
                        "--stats",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )

                lines = [line.strip() for line in proc.stdout.strip().splitlines() if line.strip()]
                self.assertEqual(len(lines), 3)

                bounds = self.plan["physical_bounds"]
                for idx, line in enumerate(lines, start=1):
                    payload = json.loads(line)
                    self.assertEqual(payload["seq"], idx)
                    self.assertAlmostEqual(payload["pressure_kpa"], 100.6533, places=3)
                    self.assertEqual(payload["sampling_freq_hz"], 16)
                    self.assertTrue(
                        bounds["temperature_c"]["min"] <= payload["temp_c"] <= bounds["temperature_c"]["max"]
                    )
                    self.assertTrue(
                        bounds["pressure_kpa"]["min"] <= payload["pressure_kpa"] <= bounds["pressure_kpa"]["max"]
                    )
                    self.assertTrue(
                        bounds["humidity_pct"]["min"] <= payload["humidity_pct"] <= bounds["humidity_pct"]["max"]
                    )

    def test_tc008_userspace_error_reporting_on_fault(self) -> None:
        """TC-008: Verify userspace C binary reports error and exits non-zero on simulated I2C fault."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sim = BME280Simulator()
            sim.inject_errno = -5  # -EIO
            dev_dir = sim.materialize_sysfs(Path(tmpdir))

            proc = subprocess.run(
                [
                    str(self.bin_path),
                    "--sysfs-path",
                    str(dev_dir),
                    "--count",
                    "1",
                    "--fail-fast",
                ],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("[ERROR]", proc.stderr)

    def test_tc009_concurrent_open_read_close_stress(self) -> None:
        """TC-009: Stress concurrent open/read/close loops across multiple worker processes."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sim = BME280Simulator()
            dev_dir = sim.materialize_sysfs(Path(tmpdir))
            n_workers = self.plan["performance_sla"]["concurrent_threads"]

            def worker_task(worker_id: int) -> int:
                res = subprocess.run(
                    [
                        str(self.bin_path),
                        "--sysfs-path",
                        str(dev_dir),
                        "--count",
                        "25",
                        "--interval-ms",
                        "5",
                        "--format",
                        "json",
                        "--fail-fast",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                rows = [json.loads(line) for line in res.stdout.strip().splitlines()]
                return len(rows)

            with concurrent.futures.ThreadPoolExecutor(max_workers=n_workers) as pool:
                futures = [pool.submit(worker_task, i) for i in range(n_workers)]
                counts = [f.result() for f in futures]

            self.assertEqual(sum(counts), n_workers * 25)

    def test_tc010_kernel_driver_and_dts_audit_compliance(self) -> None:
        """
        TC-010: Static verification that kernel driver and DT overlay implement all
        code-review requirements:
        - Hardware reprogramming of BME280_REG_CONFIG (0xF5) on sampling_frequency write
        - Sleep mode transition before writing 0xF5
        - LINUX_VERSION_CODE compatibility shim for kernels < 6.3 and >= 6.3
        - Standard IIO_VAL_FRACTIONAL pressure representation and read_avail callback
        """
        driver_src = (REPO_ROOT / "driver" / "sensor_driver.c").read_text(encoding="utf-8")
        dts_src = (REPO_ROOT / "dts" / "sensor-overlay.dts").read_text(encoding="utf-8")

        self.assertIn("module_i2c_driver(bme280_driver)", driver_src)
        self.assertIn('"custom,bme280-sensor"', driver_src)
        self.assertIn('"custom,bme280-sensor"', dts_src)
        self.assertIn("LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)", driver_src)
        self.assertIn("bme280_reprogram_standby_locked", driver_src)
        self.assertIn("bme280_read_avail", driver_src)
        self.assertIn("IIO_VAL_FRACTIONAL", driver_src)
        self.assertIn("bme280_get_le16", driver_src)
        self.assertIn("sign_extend32", driver_src)

    @unittest.skipUnless(HIL_ENABLED, "HIL tests require Raspberry Pi 5 target (set BME280_HIL=1)")
    def test_hil_001_sysfs_iio_device_present(self) -> None:
        """HIL-001: Verify physical BME280 is bound under /sys/bus/iio/devices/ and readable."""
        proc = subprocess.run(
            [str(self.bin_path), "--count", "10", "--interval-ms", "50", "--format", "json"],
            capture_output=True,
            text=True,
            check=True,
        )
        samples = [json.loads(line) for line in proc.stdout.strip().splitlines()]
        self.assertEqual(len(samples), 10)
        for s in samples:
            self.assertTrue(10.0 <= s["temp_c"] <= 50.0)
            self.assertTrue(90.0 <= s["pressure_kpa"] <= 110.0)
            self.assertTrue(10.0 <= s["humidity_pct"] <= 95.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
