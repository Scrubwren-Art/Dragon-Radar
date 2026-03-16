# DragonFinder

A prop replica of the Dragon Radar from _Dragon Ball Z_, built on an ESP32-S3 with a 480x480 RGB LCD display. The radar displays a rotating grid with animated target detection, gyroscope-based heading tracking, and buzzer feedback when targets are detected.

Most of the driver and peripheral code is derived from the example code provided in the [Waveshare ESP32-S3-LCD-2.8C wiki](https://www.waveshare.com/wiki/ESP32-S3-LCD-2.8C).

| Supported Targets |
| ----------------- |
| ESP32-S3          |

## Features

- Animated radar sweep on startup that progressively reveals 8 targets
- Buzzer beeps as each target is detected during the scan
- Gyroscope-driven heading — rotate the device and the radar rotates with you
- Display auto-sleeps after 30 seconds; boot button toggles it back on

## Hardware

This project was designed around the [ESP32-S3 2.8" Round Display Development Board](https://thepihut.com/products/esp32-s3-development-board-with-2-8-ips-capacitive-touch-round-display-480-x-480) (available from The Pi Hut), which integrates the MCU, display, IMU, RTC, and battery management onto a single board.

| Component                 | Role                                  |
| ------------------------- | ------------------------------------- |
| ESP32-S3 (8MB PSRAM)      | Main MCU                              |
| 480×480 RGB LCD (ST7701S) | Display                               |
| QMI8658 IMU               | Gyroscope / accelerometer for heading |
| PCF85063 RTC              | Real-time clock                       |
| TCA9554 I2C GPIO expander | Additional I/O                        |
| Piezo buzzer              | Audio feedback                        |
| [1200mAh LiPo battery](https://thepihut.com/products/1200mah-3-7v-lipo-battery-1-25mm-picoblade-connector) | Power (monitored via ADC on GPIO4) |

> **Warning:** The battery linked above has its connector pins in the **reverse order** from what the ESP32-S3 board expects. You must swap the wires in the PicoBlade connector before plugging it in, or you risk damaging the board.

Hardware design files are in the [hardware/](hardware/) directory.

## Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html) v5.x **or** [PlatformIO](https://platformio.org/) with the ESP-IDF framework
- USB cable for flashing

## Build and Flash

### VSCode + PlatformIO extension (recommended)

Install the [PlatformIO IDE extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) for VSCode. Once installed, open this repository as a folder and use the PlatformIO toolbar at the bottom of the window to **Build**, **Upload**, and open the **Serial Monitor** — no terminal commands needed.

### PlatformIO CLI

```bash
pio run -e esp32-s3-devkitc-1 -t upload
pio device monitor
```

### ESP-IDF CLI

```bash
idf.py set-target esp32s3
idf.py -p PORT build flash monitor
```

Replace `PORT` with your serial port (e.g. `/dev/ttyUSB0` or `COM3`).

The first build will download LVGL and other managed components into the `components/` directory — this is normal and may take a moment.

To exit the serial monitor, press `Ctrl-]`.

## Configuration

Run `idf.py menuconfig` (or use PlatformIO's project configuration) to adjust settings. Key options:

- `Example Configuration → Use double Frame Buffer` — reduces LCD tear effect at the cost of more PSRAM
- `Example Configuration → Use bounce buffer` — improves PCLK stability by staging data through internal SRAM

The default `sdkconfig` is tuned for the target hardware and should work out of the box.

## Project Structure

```
main/
├── main.c                # App entry point, power management, button handling
├── DragonFinderUI/       # Radar grid rendering and scan animation
├── QMI8658/              # IMU driver (gyroscope heading)
├── LCD_Driver/           # ST7701S RGB panel driver
├── LVGL_Driver/          # LVGL integration and display flush
├── Buzzer/               # Piezo buzzer control
├── BAT_Driver/           # Battery ADC monitoring
├── Button_Driver/        # Boot button input
├── I2C_Driver/           # Shared I2C bus
├── EXIO/                 # TCA9554 GPIO expander
└── PCF85063/             # RTC driver
```

## Optional Features

The following are implemented but disabled by default (commented out in `main.c`):

- **WiFi / BLE** — wireless scanning via `Wireless_Init()`
- **SD card** — file I/O via `SD_Init()`
- **Simulated gestures** — touch simulation for UI testing via `Simulated_Touch_Init()`

## Troubleshooting

**Display doesn't light up** — Check that the backlight enable level matches your panel. Adjust `EXAMPLE_LCD_BK_LIGHT_ON_LEVEL` in the LCD driver.

**Screen tearing or drift** — Enable double frame buffer in menuconfig, or lower the PCLK frequency.

**Out of memory** — Frame buffers are allocated from PSRAM. Ensure PSRAM is enabled in `sdkconfig` (`CONFIG_ESP32S3_SPIRAM_SUPPORT`).

**Gyroscope heading not updating** — Verify I2C wiring to the QMI8658 and that the correct I2C address is set in the driver.

## License

GPL-3.0 — see [LICENSE.MD](LICENSE.MD).
