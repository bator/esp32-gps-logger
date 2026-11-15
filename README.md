# ESP32 GPS Logger

[![CI](https://github.com/bator/esp32-gps-logger/actions/workflows/ci.yml/badge.svg)](https://github.com/bator/esp32-gps-logger/actions/workflows/ci.yml)

A MicroPython GPS track logger for the **Heltec Wireless Tracker** (ESP32-S3 with
a UC6580 receiver on the board). It records your position to the internal flash,
offline or on WiFi, and uploads finished tracks to a [Dawarich](https://dawarich.app)
server through its Overland API.

Tag `c6-last` holds the code for the older hardware (ESP32 / ESP32-C6 with a
u-blox receiver and an OLED).

## At a glance

- **One clean point a second.** The receiver runs at 5 Hz; the five
  fixes of each second are averaged into one point while slow, and the
  second's first fix is the point above 10 km/h (a mean would lag the
  timestamp). Dawarich stores whole seconds, so more than one point a
  second would zigzag there.
- **Bad fixes are skipped.** Samples with HDOP above 5, or whose own error
  estimate (GST) is above 20 m, are not used, and so is the first minute after a
  start unless moving: the receiver starts 15 m off.
- **No zigzag while parked.** A position-based stay filter detects standing
  still. The indoor drift is never written. Away from home the stay shows up
  as one anchor point every 5 minutes; at home nothing is written.
- **Power-loss safe files** on LittleFS, uploaded oldest first in batches of
  50 points, with a byte offset sidecar so nothing is sent twice or lost.
- **Up to three WiFi networks** (home, phone hotspot, another place). The
  radio is on only until everything is uploaded, then sleeps for an hour.
- **Power bank friendly:** a WiFi scan every 5 s keeps power banks that switch
  off at low load awake. The CPU runs at 80 MHz, and the radio is off whenever it
  has nothing to do, including long upload backoffs.
- **Runs unattended:** hardware watchdog, crash recovery, boot-loop guard.

## Quick start

1. **Host tools** (Linux, Python 3):

   ```bash
   python3 -m venv .venv
   .venv/bin/pip install mpremote "mpy-cross==1.29.*"
   ```

2. **Firmware** (MicroPython 1.29 with the logger frozen in; needs ESP-IDF
   v5.5.2 and a MicroPython v1.29.0 tree):

   ```bash
   PORT=/dev/ttyACM0 tools/build_firmware.sh --flash
   ```

3. **`.env`** with your networks and the Dawarich server, copied to the board:

   ```ini
   SSID=MyHomeWifi
   WPA2=wifi-password
   SSID2=PhoneHotspot
   WPA2_2=hotspot-password
   GEO_URL=https://dawarich.example.com
   API_KEY=your-dawarich-api-key
   DEVICE_ID=Dawarich-GPS-tracker
   ```

   ```bash
   .venv/bin/mpremote cp .env :.env
   ```

4. **Deploy** `main.py` and the status page:

   ```bash
   PORT=/dev/ttyACM0 ./deploy.sh
   ```

5. **Later updates over WiFi** (optional): publish with
   `OTA_DEST=user@host:/srv/www/gps-ota tools/build_firmware.sh --publish`; the
   board checks for a newer build every 10 minutes.

The full walk-through, every `.env` key, migration and OTA: **[docs/setup.md](docs/setup.md)**.

## Using it

- **Status page:** `http://<board ip>/` while it is on WiFi (the same data as
  JSON at `/status.json`): position and the day's track on a map, trips with
  their places, satellites, the logger's subsystems (GPS, upload, storage,
  memory, WiFi, battery, firmware) and the diagnostic log.
- **Display:** today's route, the clock, satellites, km, WiFi, what waits for
  upload, the battery, the GPS quality and the free flash.
- **Tracks:** `/gpx/` on the board; uploaded oldest first, deleted once Dawarich
  has them. `tools/trk2gpx.py` turns a track file into GPX.
- **Something wrong?** The [diagnostic log](docs/usage.md#diagnostic-log) on
  the board (and on the status page) says what happened and when.

Details: **[docs/usage.md](docs/usage.md)**.

## Documentation

| Document | What is in it |
|----------|---------------|
| [Hardware](docs/hardware.md) | The board, pins, receiver setup, display |
| [Setup](docs/setup.md) | Host tools, `.env` (all keys), deploy, custom firmware, OTA updates |
| [Using it](docs/usage.md) | Serial console, boot bypass, getting tracks, diagnostic log reference, status page and `status.json` fields |
| [How it works](docs/internals.md) | GPS, points and track files, stays, WiFi, upload and TLS, watchdog and recovery |
| [Configuration](docs/configuration.md) | Every constant in `gps_logger.py` with its default |
| [Development](docs/development.md) | Tests and CI: what each check covers |
| [ADRs](docs/adr/) | Design decisions |
