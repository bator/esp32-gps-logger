# ESP32 GPS Logger

MicroPython GPS track logger for the ESP32. Away from home it records your
position to GPX files on the internal flash; once it sees a configured WiFi
network it stops logging and uploads the finished tracks to a
[Dawarich](https://dawarich.app) server via its Overland API endpoint.

## Features

- Reads NMEA (`RMC`, `GGA`, `GSA`, `GSV`) from a u-blox NEO-6M/7M/8M over UART, with checksum validation.
- Samples the position every second and records a point only after moving more than 10 m from the last recorded one, to GPX 1.1 files named `/gpx/YYYYMMDD_HHMMSS.gpx`. Samples with a fresh HDOP worse than 5 are skipped; 2D fixes omit altitude.
- Power-loss safe: each file is always a complete GPX document, written in batches every 60 s (LittleFS commits on close).
- Short fix losses start a new track segment; a gap over 5 minutes starts a new file.
- Keeps 32 kB of flash free; stops logging when full. Logging resumes if space is freed (checked each sample) or after an online→offline cycle.
- Up to three WiFi networks. After a scan, visible known APs are tried strongest-RSSI first (~12 s connect timeout, WDT fed while waiting). A network that fails to associate is remembered briefly and tried last next time. If the scan fails or none of ours are visible, config order 1→2→3 is used. If all fail, logging continues offline and reconnects with capped exponential backoff.
- On WiFi: uploads finished files oldest-first, 50 points per POST. Progress is a `<file>.gpx.pos` sidecar (byte offset). LittleFS rename is atomic; a crash between POST and the offset commit resends the same points. Dawarich deduplicates on `(user, timestamp, lon, lat)`. Truncated or non-numeric `<trkpt>` lines are skipped. HTTP 400/413/422 appends that batch to `<file>.gpx.bad` and continues with the rest of the file. Other 4xx/5xx use exponential backoff (honours `Retry-After`); 401 retries after 30 minutes (Dawarich also uses 401 for plan/point limits).
- 128×64 OLED status screen: connected SSID (never the password), GPS fix, mode (LOG/UPLOAD), free flash and points left. If the display is missing or drops out at runtime, the logger runs headless.
- Hardware watchdog (120 s) fed in the main loop and inside long scans/`count_points`/POST. Uncaught exceptions print, flush, wait ~1 s and `machine.reset()`. Ctrl-C leaves the WDT armed but a timer keeps feeding it; for `mpremote cp` of large files use the GPIO25 boot-bypass. The watchdog is not started if GPIO25 is held at reset.

## Hardware

| Part | Notes |
|------|-------|
| ESP32 dev board | Original ESP32 (Xtensa, `-march=xtensawin`). Use a **WROOM** module, not WROVER: on WROVER boards GPIO16/17 are wired to PSRAM and cannot be used for UART2. Flash firmware with **LittleFS** (the default ESP32_GENERIC image). Offset sidecars are not crash-atomic on FAT. |
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
# optional extra networks (ignored if empty / omitted)
SSID2=PhoneHotspot
WPA2_2=hotspot-password
SSID3=CabinWifi
WPA2_3=cabin-password

GEO_URL=https://dawarich.example.com
API_KEY=your-dawarich-api-key

# Overland tracker id and DHCP hostname (defaults match the field device)
DEVICE_ID=Dawarich-GPS-tracker
HOSTNAME=Dawarich-GPS-tracker
```

A single-network `.env` with only `SSID` / `WPA2` still works as network #1.

**How a network is chosen**

1. Scan once.
2. Keep the configured SSIDs that the scan actually saw, strongest RSSI first.
3. Try each with a ~12 s connect timeout (WDT fed while waiting). Wrong
   password, no DHCP, or an overloaded AP → mark that SSID “recently failed”
   and move on.
4. If the scan itself fails or none of our SSIDs are visible, try config
   order 1→2→3 instead of RSSI order.
5. If every attempt fails, stay in LOG mode and retry later with capped
   exponential backoff. A recently failed SSID is not tried first on the
   next round.

The API key is sent as `Authorization: Bearer <key>` (Dawarich accepts that
header as well as `?api_key=`). Prefer `https://` for `GEO_URL`: an `http://`
URL is still used (typical for a LAN self-host) but the logger prints a
warning, because the key and GPS data then go in the clear.

**TLS caveat:** the MicroPython `requests` library sets
`ssl.CERT_NONE`, so `https://` does **not** authenticate the server. A MITM
on the path can still see the Bearer key and the points. Pinning Let's
Encrypt ISRG Root X1 (or your own CA) would need a custom `ssl.SSLContext`
and a tiny HTTP client; that is not wired up here because mbedTLS already
sits close to the ~40 kB IDF-heap budget. Use a trusted LAN or a VPN if
that matters.

micropython-lib `requests` also follows 301/302/303 with a GET and no body.
An `http://`→`https://` redirect therefore becomes a 404, not an upload.
Put the https origin in `GEO_URL` directly.

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

- Power on: the logger starts automatically, scans once for configured
  networks, then shows the status screen (or runs headless). The OLED WiFi
  line is the connected SSID.
- **Skip the logger** (REPL / `mpremote`, **no watchdog**): hold GPIO25
  while resetting. Use this for copying files off the device. Ctrl-C from a
  running logger leaves the WDT armed; a timer keeps feeding it so the REPL
  does not reboot after 120 s, but the GPIO25 bypass is still the reliable
  way to pull a large `/gpx` file (`mpremote cp` of ~1 MB can take longer
  than one WDT period if the timer is not running).
- An uncaught exception is printed, the pending GPX batch is flushed, and
  the board resets after about a second. Init (point count, OLED, WiFi)
  runs inside the same handler. If a call hangs past ~120 s, the watchdog
  resets the chip.
- Logs live in `/gpx/` on the device; pull them with the GPIO25 bypass:

  ```bash
  .venv/bin/mpremote cp :/gpx/20261005_101500.gpx .
  ```

  Upload progress for a file is `/gpx/20261005_101500.gpx.pos`. Batches
  Dawarich rejected as payload errors are appended to
  `/gpx/20261005_101500.gpx.bad`. After a successful upload the logger will
  retry a `.bad` file a few times. To re-queue by hand (GPIO25 bypass):

  ```bash
  .venv/bin/mpremote exec "import os; os.rename('/gpx/20261005_101500.gpx.bad', '/gpx/20261005_101500.gpx')"
  ```

  `.bad` files count against free flash. They are capped at 64 kB of
  appended points; pull or delete them if the logger reports flash full.

## Configuration

Constants at the top of `gps_logger.py`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `SAMPLE` | 1000 ms | Interval between position samples |
| `MIN_MOVE` | 10 m | Distance from the last recorded point before a new one is recorded |
| `FLUSH` | 60000 ms | Interval between flash writes |
| `NEW_FILE_GAP` | 300 s | Fix loss that starts a new file |
| `MIN_FREE` | 32 kB | Flash reserve |
| `MAX_HDOP` | 5 | Skip samples with a fresh GGA dilution worse than this |
| `UPLOAD_BATCH` | 50 | Points per POST |
| `CONNECT_TIMEOUT` | 12 s | Per-network wait for DHCP |
| `BACKOFF_START` | 15 s | First retry after net/5xx/4xx |
| `BACKOFF_MAX` | 30 min | Cap on upload backoff |
| `AUTH_BACKOFF` | 30 min | Retry after HTTP 401 |
| `HTTP_TIMEOUT` | 15 s | Per-socket HTTP timeout (wall clock can be longer; WDT is the cap) |
| `WDT_TIMEOUT` | 120 s | Hardware watchdog |
| `MAX_BAD_BYTES` | 64 kB | Cap on `<file>.gpx.bad` |
| `DEVICE_ID` | `Dawarich-GPS-tracker` | Overland tracker id (override with `.env`) |
| `HOSTNAME` | `Dawarich-GPS-tracker` | DHCP hostname (override with `.env`) |
