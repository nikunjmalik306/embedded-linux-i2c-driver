# Hardware Bring-Up Guide & Wiring Specification (Raspberry Pi 5 + BME280)

## 1. Electrical Compatibility Warning (3.3V Logic Only)

- **Raspberry Pi 5 GPIO Voltage:** All GPIO pins on the Raspberry Pi 5 (driven by the RP1 southbridge) operate strictly at **3.3V CMOS logic** and are **NOT 5V tolerant**.
- **Bosch BME280 Operating Range:**
  - `VDD` (Sensor core supply): `1.71 V` to `3.6 V`
  - `VDDIO` (Digital interface supply): `1.2 V` to `3.6 V`
- **Verification Before Wiring:** Inspect your BME280 breakout board. Connect `VIN`/`VCC` strictly to **Raspberry Pi 5 Pin 1 (`3V3 Power`)**, **never** Pin 2 or Pin 4 (`5V`).

---

## 2. Wiring Diagram & Pin Mapping (`i2c1`)

```
+-----------------------------------+           +----------------------------------+
|      Raspberry Pi 5 (J8 Header)   |           |   Bosch BME280 Breakout Board    |
+-----------------------------------+           +----------------------------------+
| Pin 1  (3V3 Power)        [3.3V]  | --------> | VIN / VDD / VDDIO (3.3V Supply)  |
| Pin 3  (GPIO 2 / SDA1)    [SDA]   | <-------> | SDI / SDA         (I2C Data)     |
| Pin 5  (GPIO 3 / SCL1)    [SCL]   | --------> | SCK / SCL         (I2C Clock)    |
| Pin 6  (Ground)           [GND]   | --------> | GND               (Common Ground)|
| Pin 9  (Ground)           [GND]   | --------> | SDO               (Addr = 0x76)  |
+-----------------------------------+           +---+------------------------------+
                                                    | (Tie CSB high to 3.3V if exposed;
                                                    |  most breakouts pull CSB high onboard)
```

| Raspberry Pi 5 Header Pin | BCM / RP1 GPIO | Signal Name | BME280 Breakout Pin | Electrical Notes |
| :---: | :---: | :--- | :--- | :--- |
| **Pin 1** | `3V3` | `3.3V DC` | `VIN` / `VDD` | Regulated 3.3V rail |
| **Pin 3** | `GPIO 2` | `I2C1_SDA` | `SDA` / `SDI` | Internal 1.8 kΩ pull-up to 3.3V on Pi 5 |
| **Pin 5** | `GPIO 3` | `I2C1_SCL` | `SCL` / `SCK` | Internal 1.8 kΩ pull-up to 3.3V on Pi 5 |
| **Pin 6** | `GND` | `Ground` | `GND` | Common signal & power ground |
| **Pin 9** | `GND` | `Ground` | `SDO` | Tying `SDO -> GND` selects I²C address `0x76` (`SDO -> 3V3` selects `0x77`) |

---

## 3. Step-by-Step Phase 1 Target Bring-Up Procedure

### Step 1: Confirm Target Kernel and Enable `i2c1`
Verify kernel version and ensure `dtparam=i2c_arm=on` is enabled in `/boot/firmware/config.txt`:

```bash
$ uname -r
6.6.51+rpt-rpi-2712

$ ls -l /dev/i2c-*
crw-rw---- 1 root i2c 89, 1 Oct 24 09:12 /dev/i2c-1
```

### Step 2: Scan `i2c-1` Bus and Confirm Slave Address `0x76`
Use `i2cdetect` to confirm the BME280 ACKs its 7-bit address (`0x76`):

```bash
$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- 76 --
```

### Step 3: Verify Hardware Chip ID Register (`0xD0 == 0x60`)
Before loading any driver, read register `0xD0` directly via `i2cget` to confirm the sensor is a genuine **BME280 (`0x60`)** and not a BMP280 (`0x58`, which lacks humidity):

```bash
$ i2cget -y 1 0x76 0xD0 b
0x60
```

### Step 4: Capture Baseline with Custom Device Tree Overlay & Kernel Driver

```bash
# Compile and apply the Device Tree Overlay
$ dtc -@ -I dts -O dtb -o dts/sensor-overlay.dtbo dts/sensor-overlay.dts
$ sudo dtoverlay dts/sensor-overlay.dtbo

# Load our custom kernel module
$ sudo insmod driver/sensor_driver.ko

# Inspect kernel log
$ dmesg | tail -n 5
[ 1412.408112] bme280_custom 1-0076: Bosch BME280 sensor initialized at 0x76 (Chip ID 0x60, IIO registered)

# Inspect created IIO sysfs interface
$ head -n 1 /sys/bus/iio/devices/iio:device0/{name,in_temp_input,in_pressure_input,in_humidityrelative_input}
==> /sys/bus/iio/devices/iio:device0/name <==
bme280_custom
==> /sys/bus/iio/devices/iio:device0/in_temp_input <==
25080
==> /sys/bus/iio/devices/iio:device0/in_pressure_input <==
100.652343
==> /sys/bus/iio/devices/iio:device0/in_humidityrelative_input <==
47292
```
