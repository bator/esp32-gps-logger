# ESP32 GPS Logger

MicroPython GPS track logger for the ESP32. Away from home it records your
position to GPX files on the internal flash. On a **home** WiFi network it
stops logging and uploads finished tracks to a
[Dawarich](https://dawarich.app) server via its Overland API. On a
**non-home** network (a phone hotspot in the car) it keeps logging and
uploads already-finished files in the background.

## Features

- Reads NMEA (`RMC`, `GGA`, `GSA`, `GSV`) from a u-blox NEO-6M/7M/8M over UART, with checksum validation.
- Samples the position every second and records a point only after moving more than 10 m from the last recorded one, to GPX 1.1 files named `/gpx/YYYYMMDD_HHMMSS.gpx`. Samples with a fresh HDOP worse than 5 are skipped; 2D fixes omit altitude.
- Power-loss safe: each file is always a complete GPX document, written in batches every 60 s (LittleFS commits on close).
- Short fix losses start a new track segment; a gap over 5 minutes starts a new file. On a **non-home** network a file that has been open for 5 minutes is closed so it can upload (offline driving stays one file; parking does not open empty files).
- Keeps 32 kB of flash free; stops logging when full. Logging resumes if space is freed (checked every few seconds) or after leaving a home network.
- Up to three WiFi networks. Network **#1 is home** by default (connected → upload only). **#2 and #3 are not home** (log + opportunistic upload). Override with `HOME1=0` / `HOME2=1` / `SSID2_HOME=1`. After a scan, a home AP at ≥ −75 dBm is tried first, then other visible known APs strongest-RSSI first (one SSID, strongest BSSID). Hidden SSIDs (`SSID2_HIDDEN=1`) are tried after visible ones. If the scan sees none of ours, only hidden nets are tried (no blind 1→2→3). If the scan itself fails, config order is used, under backoff. Connect is polled from the main loop so GPS sampling is not blocked. On a non-home net, when nearly stopped, the logger scans for home (starting every ~5 min, backing off to 60 min if home is missing or the switch fails); if a home AP is visible at ≥ −75 dBm it switches between upload batches (and falls back to the hotspot if that connect fails).
- Uploads finished files oldest-first, 50 points per POST. Progress is a `<file>.gpx.pos` sidecar. Offset advances only after HTTP 2xx or a durable write to `<file>.gpx.bad`. If `.bad` is full (64 kB), that file's uploads pause (`BAD FULL` on the OLED) instead of dropping points. Three consecutive 400/422 responses without a success are treated as a server/config problem: backoff, no more quarantine. A `.bad` retry that still 4xxs is left on flash.
- 128×64 OLED: connected SSID (never the password), GPS fix, mode (`LOG` / `LOG+UP` / `UPLOAD` / `BAD FULL`), free flash and points left. Missing or dropping display → headless.
- Hardware watchdog (120 s) fed in the main loop and inside long reads/POST. Uncaught exceptions print, flush, wait ~1 s and reset. Ctrl-C starts a keepalive Timer (deinit on the next `run()`, stops feeding after 10 minutes so an accidental Ctrl-C in the field is not forever-REPL). Repeated boot/import failures delay, then drop to the REPL. GPIO25 at reset skips the logger entirely.

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
# Network #1 is HOME by default: at this SSID the logger stops recording
# and uploads. A single-network .env is unchanged from earlier versions.
SSID=MyHomeWifi
WPA2=wifi-password

# Phone hotspot for a car trip: NOT home, so the trip is still logged
# and finished files upload when the hotspot has data.
SSID2=PhoneHotspot
WPA2_2=hotspot-password
# HOME2=0          # default for #2/#3; set HOME2=1 to treat as home
# SSID2_HIDDEN=1   # try even when the scan does not list it

# Optional third network
SSID3=CabinWifi
WPA2_3=cabin-password
# HOME3=1
# SSID3_HIDDEN=1

GEO_URL=https://dawarich.example.com
API_KEY=your-dawarich-api-key

# [A-Za-z0-9._-]{1,32} — invalid values fall back to the default
DEVICE_ID=Dawarich-GPS-tracker
HOSTNAME=Dawarich-GPS-tracker
```

`HOME1` / `SSID_HOME`, `HOME2` / `SSID2_HOME`, `HOME3` / `SSID3_HOME` accept
`1`/`0` (also `true`/`false`). Network #1 defaults to home; #2 and #3 do not.

**How a network is chosen**

1. Scan once (this is the only blocking radio step).
2. Deduplicate by SSID (strongest BSSID wins).
3. Prefer a home AP at ≥ −75 dBm, then other visible known APs strongest
   RSSI first. Each try is polled from the main loop with a ~12 s
   deadline (WDT fed; GPS keeps sampling). Wrong password / no DHCP →
   mark that SSID “recently failed” and move on. A leftover fail reason
   from the previous SSID (MicroPython 1.29 `connect()` does not always
   clear it) is **not** treated as this attempt failing until CONNECTING
   is seen or the 12 s deadline elapses. Confirm on device with a bad
   password on #1 and a good #2.
4. Then try nets marked `SSIDn_HIDDEN=1` that were not in the scan.
5. If the scan succeeded and listed none of our SSIDs, **do not** blindly
   try 1→2→3; only hidden nets are attempted. If the scan itself failed,
   fall back to config order, still under backoff.
6. If every attempt fails, stay in LOG mode and retry later with capped
   exponential backoff (up to 10 minutes).
7. While connected to a **non-home** net and nearly stopped (< 5 km/h),
   rescan for home (first interval ~5 min, then 10, 20, … up to 60 min
   if home is missing or the switch fails). A home AP at ≥ −75 dBm is
   preferred; if that switch fails, the hotspot is tried again. Offset
   only advances after HTTP 2xx, so a mid-upload drop is retried
   (Dawarich dedups).

OLED / log show the connected SSID, never the password. Mode is `UPLOAD`
on a home net, `LOG+UP` on a hotspot, `LOG` when offline.

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

micropython-lib `requests` follows 301/302/303 with a GET and no body.
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
  networks, then shows the status screen (or runs headless).
- **Skip the logger** (REPL / `mpremote`, **no watchdog**): hold GPIO25
  while resetting. Use this for copying files off the device.
- Ctrl-C from a running logger leaves the WDT armed; a timer feeds it for
  **10 minutes**, then the chip resets (so a stray 0x03 on USB-UART in the
  field does not leave the device in REPL forever). The next `run()` deinit's
  that timer. GPIO25 bypass is still the reliable way to pull a large `/gpx`.
- Repeated import/crash/WDT at boot: increasing delay (5 s, 10 s, … cap
  30 s), then the board stays in the REPL after 5 failures (RTC memory
  counter, `/boot_fail` fallback). The counter is **not** cleared on
  import — only after ~60 s of a healthy main loop, or by `deploy.sh`,
  or by a power cycle. A WDT reset counts. If the device is stuck in
  REPL after a failed flash, power-cycle it or
  `mpremote exec "from machine import RTC; RTC().memory(b'0')"` then
  reset. `./deploy.sh` clears the counter before rebooting.
- Logs live in `/gpx/`; pull them with the GPIO25 bypass:

  ```bash
  .venv/bin/mpremote cp :/gpx/20261005_101500.gpx .
  ```

  Progress is `/gpx/20261005_101500.gpx.pos`. Payload-rejected batches are
  appended to `/gpx/20261005_101500.gpx.bad`. After a successful upload the
  logger retries a `.bad` file a few times per boot. To re-queue by hand:

  ```bash
  .venv/bin/mpremote exec "import os; os.rename('/gpx/20261005_101500.gpx.bad', '/gpx/20261005_101500.gpx')"
  ```

  If `.bad` reaches 64 kB the OLED shows `BAD FULL` and **no more points
  are dropped** — pull or delete the `.bad` file, then reset.

## Configuration

Constants at the top of `gps_logger.py`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `SAMPLE` | 1000 ms | Interval between position samples |
| `MIN_MOVE` | 10 m | Distance from the last recorded point before a new one is recorded |
| `FLUSH` | 60000 ms | Interval between flash writes |
| `NEW_FILE_GAP` | 300 s | Fix loss that starts a new file; also max age of an open file on a hotspot |
| `ROAM_CHECK` | 5 min | First home-roam scan interval on a hotspot |
| `ROAM_BACKOFF_MAX` | 60 min | Cap on roam retry after failed / unseen home |
| `ROAM_MIN_RSSI` | −75 dBm | Minimum home RSSI to switch away from a hotspot (also boot preference) |
| `UPLOAD_GAP_AWAY` | 30 s | Pause between POSTs on a non-home net (UART drain) |
| `BOOT_OK_MS` | 60 s | Healthy `_loop` before the boot-fail counter is cleared |
| `MIN_FREE` | 32 kB | Flash reserve |
| `MAX_HDOP` | 5 | Skip samples with a fresh GGA dilution worse than this |
| `UPLOAD_BATCH` | 50 | Points per POST |
| `CONNECT_TIMEOUT` | 12 s | Per-network wait for DHCP (polled, not a blocking sleep) |
| `BACKOFF_START` | 15 s | First retry after net/5xx/4xx |
| `BACKOFF_MAX` | 30 min | Cap on upload backoff and `Retry-After` |
| `AUTH_BACKOFF` | 30 min | Retry after HTTP 401 |
| `HTTP_TIMEOUT` | 15 s | Per-socket HTTP timeout (WDT 120 s is the wall-clock cap) |
| `WDT_TIMEOUT` | 120 s | Hardware watchdog |
| `KEEPALIVE_MAX` | 10 min | Ctrl-C timer may feed the WDT this long |
| `MAX_BAD_BYTES` | 64 kB | Cap on `<file>.gpx.bad`; uploads pause when full |
| `MAX_GLOBAL_PAYLOAD` | 3 | Consecutive 400/422 without 2xx → backoff, no quarantine |
| `DEVICE_ID` | `Dawarich-GPS-tracker` | Overland tracker id (`.env`, `[A-Za-z0-9._-]{1,32}`) |
| `HOSTNAME` | `Dawarich-GPS-tracker` | DHCP hostname (`.env`, `[A-Za-z0-9-]{1,32}`) |
