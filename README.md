# ESP32 GPS Logger

MicroPython GPS track logger for the ESP32. Away from home it records your
position to GPX files on the internal flash; once it sees the home WiFi it
stops logging and uploads the finished tracks to a
[Dawarich](https://dawarich.app) server via its Overland API endpoint.

## Features

- Reads NMEA (`RMC`, `GGA`, `GSA`, `GSV`) from a u-blox NEO-6M/7M/8M over UART, with checksum validation.
- Samples the position every second and records a point only after moving more than 10 m from the last recorded one, to GPX 1.1 files named `/gpx/YYYYMMDD_HHMMSS.gpx`. Samples with HDOP worse than 5 are skipped.
- Power-loss safe: each file is always a complete GPX document, written in batches every 60 s (LittleFS commits on close).
- Short fix losses start a new track segment; a gap over 5 minutes starts a new file.
- Keeps 32 kB of flash free; stops logging when full. Going back offline clears the full flag so logging can resume if space has been freed.
- On WiFi: uploads finished files oldest-first, 50 points per POST. Progress is a `<file>.gpx.pos` sidecar (byte offset, atomic `.tmp` + rename) so flash is not rewritten after every batch. A crash between POST and the offset commit resends the same points; Dawarich deduplicates. Truncated or non-numeric `<trkpt>` lines are skipped. After 3 consecutive HTTP 4xx payload rejections the file is renamed `*.gpx.bad` (quarantine) so upload can continue.
- 128×64 OLED status screen: WiFi state, GPS fix (2D/3D/DGPS), mode (LOG/UPLOAD), free flash and points left to upload. If the display is missing, the logger runs headless.
- Hardware watchdog (~60 s) fed every main-loop iteration; an uncaught exception prints the traceback, flushes, waits ~1 s and `machine.reset()`. The watchdog is created only after the logger starts, so the GPIO25 boot-bypass REPL is not reset.

## Hardware

| Part | Notes |
|------|-------|
| ESP32 dev board | Original ESP32 (Xtensa, `-march=xtensawin`). Use a **WROOM** module, not WROVER: on WROVER boards GPIO16/17 are wired to PSRAM and cannot be used for UART2. |
| GPS module | u-blox NEO-6M / 7M / 8M, UART at 9600 baud |
| OLED display | SSD1306, 128×64, I²C (address 0x3C); optional (headless if absent) |
| Push button | E.g. the centre (M) press of a 5-way joystick module; optional |

### Wiring

![Wiring schematic](docs/wiring.svg)

| ESP32 pin | Connects to | Function |
|-----------|-------------|----------|
| 3V3 | GPS VCC, OLED VCC | Power |
| GND | GPS GND, OLED GND, button | Common ground |
| GPIO16 | GPS TX | UART2 RX (NMEA in) |
| GPIO17 | GPS RX | UART2 TX |
| GPIO21 | OLED SDA | I²C0 data |
| GPIO22 | OLED SCL | I²C0 clock |
| GPIO25 | Button → GND | Boot bypass (internal pull-up, active low) |

Most GPS and OLED breakout boards already have I²C pull-ups and a regulator;
check yours if powering the GPS from 5 V instead.

**ESP32-WROVER:** GPIO16 and GPIO17 are used by the PSRAM. This firmware's
UART2 default will not work on those modules; pick a WROOM board or change
`tx`/`rx` in `gps_logger.py` to free pins.

## Software setup

Requirements on the host: Linux, Python 3, and the ESP32 flashed with
**MicroPython 1.29** (the `.mpy` version must match).

```bash
python3 -m venv .venv
.venv/bin/pip install mpremote "mpy-cross==1.29.*"

# drivers on the device (skip any already frozen into your firmware)
.venv/bin/mpremote mip install ssd1306
.venv/bin/mpremote mip install requests
```

Create `.env` and copy it to the device root:

```ini
SSID=MyHomeWifi
WPA2=wifi-password
GEO_URL=https://dawarich.example.com
API_KEY=your-dawarich-api-key
```

The API key is sent as `Authorization: Bearer <key>` (Dawarich accepts that
header as well as `?api_key=`). Prefer `https://` for `GEO_URL`: an `http://`
URL is still used (typical for a LAN self-host) but the logger prints a
warning, because the key and GPS data then go in the clear.

```bash
.venv/bin/mpremote cp .env :.env
```

Without `.env` the logger still records GPX; it just never goes online.

## Deploy

```bash
./deploy.sh                      # default port /dev/ttyUSB0
PORT=/dev/ttyUSB1 ./deploy.sh
```

The script compiles `gps_logger.py` to `gps_logger.mpy`, copies it with
`main.py`, removes any stray `gps_logger.py` on the device, and resets it.
`tests/` is not copied to the device.

The module must run precompiled: compiling on the device grows the Python
heap and leaves too little ESP-IDF heap (~40 kB contiguous) for mbedTLS, so
HTTPS uploads fail with `ENOMEM`.

## Usage

- Power on: the logger starts automatically, scans once for the home network, then shows the status screen (or runs headless).
- **Skip the logger**: hold the button (GPIO25) while resetting — `main.py` exits to the REPL so you can use `mpremote`. The watchdog is not started in that case.
- An uncaught exception (for example a bad `GEO_URL` that used to kill the loop) is printed, the pending GPX batch is flushed, and the board resets after about a second. If a blocking call hangs beyond ~60 s, the watchdog resets the chip.
- Logs live in `/gpx/` on the device; pull them manually with:

  ```bash
  .venv/bin/mpremote cp :/gpx/20261005_101500.gpx .
  ```

  Upload progress for a file is `/gpx/20261005_101500.gpx.pos`. Files Dawarich
  rejected are renamed with a `.bad` suffix and left on flash.

## Configuration

Constants at the top of `gps_logger.py`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `SAMPLE` | 1000 ms | Interval between position samples |
| `MIN_MOVE` | 10 m | Distance from the last recorded point before a new one is recorded |
| `FLUSH` | 60000 ms | Interval between flash writes |
| `NEW_FILE_GAP` | 300 s | Fix loss that starts a new file |
| `MIN_FREE` | 32 kB | Flash reserve |
| `MAX_HDOP` | 5 | Skip samples with GPS dilution worse than this |
| `UPLOAD_BATCH` | 50 | Points per POST |
| `UPLOAD_RETRY` | 60000 ms | Back-off after a failed upload |
| `HTTP_TIMEOUT` | 15 s | HTTP POST timeout (the watchdog is longer than this) |
| `WDT_TIMEOUT` | 60000 ms | Hardware watchdog; longer than `HTTP_TIMEOUT` and `count_points` |
| `MAX_4XX` | 3 | Consecutive HTTP 4xx responses before quarantining a file |
| `DEVICE_ID` | `esp32` | Device ID sent to Dawarich |
