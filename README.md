# ESP32 GPS Logger

[![CI](https://github.com/bator/esp32-gps-logger/actions/workflows/ci.yml/badge.svg)](https://github.com/bator/esp32-gps-logger/actions/workflows/ci.yml)

A MicroPython GPS track logger for the ESP32 and ESP32-C6. It records your
position to the internal flash, offline or on WiFi, and uploads finished
tracks to a [Dawarich](https://dawarich.app) server through its Overland API.

- [At a glance](#at-a-glance)
- [Hardware](#hardware)
- [Setup](#setup)
- [Using it](#using-it)
- [How it works](#how-it-works)
- [Configuration](#configuration)
- [Development](#development)

## At a glance

- **One clean point a second.** The u-blox receiver runs at 5 Hz; the five
  fixes of each second are averaged into one point while slow, and the
  second's first fix is the point above 10 km/h (a mean would lag the
  timestamp). Dawarich stores whole seconds, so more than one point a
  second would zigzag there.
- **The receiver is tuned too** (UBX-CFG-NAV5): portable filter (car and
  foot alike), 3D fixes only, satellites below 10° ignored, static hold below 0.5 m/s.
- **Bad fixes are skipped.** Samples with HDOP above 5, or whose own error
  estimate (GST) is above 20 m, are not used. This also hides the first,
  wandering fixes after a cold start.
- **No zigzag while parked.** A position-based stay filter detects standing
  still. The indoor drift is never written. Away from home the stay shows up
  as one anchor point every 5 minutes; at home nothing is written.
- **Power-loss safe files** on LittleFS, uploaded oldest first in batches of
  50 points, with a byte offset sidecar so nothing is sent twice or lost.
- **Up to three WiFi networks** (home, phone hotspot, another place). The
  radio is on only until everything is uploaded, then sleeps for an hour.
- **Power bank friendly:** a short load pulse every 5 s (the radio switched
  on and off, a WiFi scan, or a GPIO-driven resistor) keeps power banks that
  switch off at low load awake. The CPU runs at 80 MHz, and the radio is off
  whenever it has nothing to do, including long upload backoffs.
- **Runs unattended:** hardware watchdog, crash recovery, boot-loop guard,
  optional 128×64 OLED (headless without it).

## Hardware

| Part | Notes |
|------|-------|
| ESP32 or ESP32-C6 board | ESP32: a **WROOM** module (on WROVER, GPIO16/17 belong to the PSRAM). Flash MicroPython **1.29** with **LittleFS** (the default image; on FAT the upload progress is not crash-safe). |
| GPS module | u-blox NEO-6M / 7M / M8 / M10, UART, 9600 baud (on the board now: a u-blox M10, ROM SPG 5.10) |
| OLED display | SSD1306, 128×64, I²C (0x3C). Optional. |
| Push button | Boot bypass, e.g. the centre press of a 5-way joystick. Optional. |

![Wiring schematic](docs/wiring.svg)

| Function | ESP32 | ESP32-C6 |
|----------|-------|----------|
| GPS TX → ESP RX (NMEA in) | GPIO16 (UART2 RX) | GPIO5 (UART1 RX) |
| ESP TX → GPS RX (setup) | GPIO17 (UART2 TX) | GPIO4 (UART1 TX) |
| OLED SDA / SCL | GPIO21 / GPIO22 | GPIO6 / GPIO7 |
| Boot bypass button → GND | GPIO25 | GPIO10 |
| Keep-alive load (optional, `KEEPALIVE_MODE=gpio`) | GPIO26 | GPIO2 |
| Power | 3V3 and GND to GPS, OLED and button | same |

- The firmware detects the C6 by itself (it has no UART2, and GPIO16/17 are
  its console). `deploy.sh` detects it too and compiles for `rv32imc`.
- The ESP TX → GPS RX wire is needed for the 5 Hz setup. Without it the
  receiver stays at 1 Hz and everything else still works.
- Most GPS and OLED breakouts have their own regulator and I²C pull-ups;
  check yours if you power the GPS from 5 V.
- On the original ESP32, `https://` uploads are tight on RAM (about 17 kB
  largest free IDF block with the logger loaded); the C6 has about 190 kB.
- **Keep-alive load** (optional): a logic-level N-MOSFET (IRLZ44N, 2N7000,
  AO3400) switched by the keep-alive GPIO through 100 Ω, with a 100 kΩ
  pull-down, pulls a 22 Ω / 2 W resistor from the 5 V USB rail to GND:
  ~230 mA for 0.1 s every 5 s (~5 mA average) is the load a power bank
  looks for, where a WiFi scan costs ~40 mA average. See
  [Power bank keep-alive](#wifi).

## Setup

### 1. Host tools

Linux with Python 3:

```bash
python3 -m venv .venv
.venv/bin/pip install mpremote "mpy-cross==1.29.*"

# drivers on the device (skip any already frozen into your firmware)
.venv/bin/mpremote mip install ssd1306
```

The upload has its own small HTTP client, so `requests` is not needed.

### 2. `.env`

Create `.env` and copy it to the device root with
`.venv/bin/mpremote cp .env :.env`. Without it the logger still records
tracks; it just never goes online.

```ini
# network #1 is home by default and preferred for connections
SSID=MyHomeWifi
WPA2=wifi-password

# phone hotspot: not home, so a trip is logged and uploads on the way
SSID2=PhoneHotspot
WPA2_2=hotspot-password

# optional third network
SSID3=CabinWifi
WPA2_3=cabin-password

GEO_URL=https://dawarich.example.com
API_KEY=your-dawarich-api-key

DEVICE_ID=Dawarich-GPS-tracker
HOSTNAME=Dawarich-GPS-tracker
# KEEPALIVE=5
# KEEPALIVE_MODE=scan
# CPU_MHZ=80
# GPS_MODEL=0
# DIAG=60
# WEB=1
# WEB_AWAY=0
```

| Key | Meaning |
|-----|---------|
| `SSID`, `WPA2` / `SSID2`, `WPA2_2` / `SSID3`, `WPA2_3` | Networks #1–#3 |
| `HOME1` / `SSID_HOME`, `HOME2` / `SSID2_HOME`, `HOME3` / `SSID3_HOME` | `1`/`0` (or `true`/`false`): treat as home. #1 is home by default, #2 and #3 are not. |
| `SSID_HIDDEN`, `SSID2_HIDDEN`, `SSID3_HIDDEN` | `1`: hidden SSID, tried even when the scan does not list it |
| `IP`, `GW`, `MASK`, `DNS` (and `IP2`…, `IP3`…) | Static IP per network. `GW` is required; `MASK` defaults to `255.255.255.0`, `DNS` to the gateway. An invalid entry falls back to DHCP with a console message. |
| `GEO_URL` | Dawarich origin. Use `https://`; see [Upload](#upload). |
| `API_KEY` | Dawarich API key, sent as `Authorization: Bearer <key>` |
| `DEVICE_ID` | Overland tracker id, `[A-Za-z0-9._-]{1,32}` |
| `HOSTNAME` | DHCP hostname, `[A-Za-z0-9-]{1,32}` (defaults to `DEVICE_ID`) |
| `KEEPALIVE` | Seconds between power bank keep-alive pulses, 5–600; `0` turns it off (e.g. on a car charger). Default 5. |
| `KEEPALIVE_MODE` | What a pulse is: `scan` (default: a full WiFi scan, ~2 s), `radio` (the radio switched on and off, ~0.2 s; too little for a bank that averages the load), `gpio` (the keep-alive pin high for 0.1 s, for a load resistor). |
| `KEEPALIVE_PIN` | GPIO of the `gpio` pulse. Default 26 (C6: 2). A pin that cannot be driven falls back to `scan`. |
| `KEEPALIVE_CONNECTED` | `1`/`0`: pulses also while connected and idle (a power bank at home with the status page on); `scan` or `gpio`, a scan keeps the link. Default 0. |
| `CPU_MHZ`, `CPU_MHZ_UP` | CPU clock in the main loop and during a POST: 20/40/80/160/240 (C6: 80/160 only); `0` leaves the firmware default. Defaults 80 and 160. |
| `GPS_MODEL` | u-blox dynamic model for NAV5: 0 portable (default), 2 stationary, 3 pedestrian, 4 automotive, 5–8 sea/airborne. Read at boot. |
| `DIAG` | Seconds between [diagnostic log](#diagnostic-log) lines, 10–3600; `0` turns it off. Default 60. |
| `WEB` | `1`/`0`: the [status page](#status-page), and the radio staying on at home for it. Default 1. |
| `WEB_AWAY` | `1`/`0`: with `WEB` on, the radio stays on for the page on the non-home networks too (e.g. a phone hotspot), instead of sleeping once all is uploaded. Default 0. |

- Put comments on their own lines. A trailing `# ...` is ignored only on
  flag, IP and name keys (`HOME*`, `*_HIDDEN`, `IP*`/`GW*`/`MASK*`/`DNS*`,
  `DEVICE_ID`, `HOSTNAME`, `KEEPALIVE*`, `CPU_MHZ*`, `GPS_MODEL`, `DIAG`, `WEB`, `WEB_AWAY`, `KEEPALIVE_CONNECTED`). SSIDs, passwords, `GEO_URL` and
  `API_KEY` are taken verbatim, since they may contain `#`.
- Invalid names fall back to the defaults. A UTF-8 BOM (Windows Notepad) is
  ignored.

A static IP for network #1 (a network without `IP` stays on DHCP, and moving
from a static network to a DHCP one hands the interface back to DHCP):

```ini
IP=192.168.1.60
GW=192.168.1.1
# optional, defaults to 255.255.255.0
MASK=255.255.255.0
# optional, defaults to GW
DNS=192.168.1.111
# network #2/#3: IP2 / GW2 / MASK2 / DNS2, IP3 / GW3 / MASK3 / DNS3
```

### 3. Deploy

```bash
./deploy.sh                      # default port /dev/ttyUSB0
PORT=/dev/ttyACM0 ./deploy.sh    # the ESP32-C6 board used here
```

The script asks the board which chip it is, compiles `gps_logger.py` to
`gps_logger.mpy` for it, copies it with `main.py`, removes a stray
`gps_logger.py`, clears the boot-fail counter and resets the board.

The module must run precompiled: compiling on the device grows the Python
heap and leaves too little ESP-IDF heap (~40 kB contiguous) for mbedTLS, and
HTTPS uploads then fail with `ENOMEM`.

## Using it

Power it on: the logger starts by itself, sets up the GPS, connects to WiFi
and shows the status screen (or runs headless).

**OLED:** connected SSID (never the password), GPS state, mode (`LOG`
offline, `LOG+UP` on WiFi, `BAD FULL`), free flash and points left to upload.

**Serial console** (115200 baud) lines worth knowing:

| Line | Meaning |
|------|---------|
| `gps setup ok` / `no ack` / `again` | 5 Hz setup acknowledged, not answered, or sent again |
| `gps restarted` | The receiver printed its start-up banner (power dip); it is set up again at once |
| `gps reset loop? N banners, no NMEA in 10 s` | The receiver keeps restarting before it even finishes its banner (seen on the board): check the GPS power and wiring |
| `gps reset loop? 5 restarts in N s` | The receiver restarted 5 times within 10 minutes, with NMEA in between (seen on the board after a flash): check the GPS power and wiring. While it loops, setups are 30 s apart (`GPS_STORM_WAIT`): each one blocks the loop for seconds |
| `gps nav5 refused` | The receiver NAKed the NAV5 filter settings; the 5 Hz setup goes on without them |
| `wifi try` / `retry` / `fail` / `ok` | Connect attempts; `fail` carries the raw `status()` code and the time |
| `wifi sleep [home]` | Radio off, everything uploaded (`home`: during a home stay) |
| `upload <file> <n> <ok> <http> <mem>` | One POST |
| `stay home\|away kept N` | A stay began; N points of the way in were kept |
| `stay moved` / `stay kept: home in range` / `stay end D` | Anchor moved; a home departure refused; the stay ended D m out |
| `stay end: home without a fix` | An away stay ended at home without a usable fix |
| `wifi home in range` | With `WEB`, a keep-alive scan saw home: a round starts now |
| `upload <file> … <ms> ms` / `GEO_URL redirects (N)` / `parked <file>.err` | POST time; a redirect (not followed); an unreadable track set aside |
| `stay left, wifi: none of N access points in M scans` | An away stay ended without a fix: its access points are gone (also in the diagnostic log as `stay left wifi`) |
| `status temp … hz … used … hdop … points … sent … wifi … pulses … mem … gpsloops …` | Every 10 minutes: chip temperature (the ESP die, not the GPS), fixes in the last second, satellites, HDOP, points, uploads, keep-alive pulses so far, free heap, GPS reset-loop warnings so far |
| `wifi off until upload retry N s` | Uploads back off for N s (server down, 401): the radio sleeps until then |
| `cpu freq N …` | The board refused that CPU clock; it runs at the firmware default |

**Boot bypass:** hold the button (GPIO25, C6: GPIO10) while resetting to
skip the logger: REPL and `mpremote`, no watchdog. Use it to copy files off
the device.

**Ctrl-C** on a running logger leaves the watchdog armed. A timer feeds it
for 10 minutes, then the chip resets, so a stray Ctrl-C in the field does not
leave the device in the REPL for good.

**Boot loop guard:** repeated import, crash or watchdog resets at boot get an
increasing delay (5 s, 10 s, … 30 s); after 5 the board stays in the REPL. The
counter (RTC memory, `/boot_fail` as fallback) is cleared after ~60 s of a
healthy main loop, by `deploy.sh` or by a power cycle. By hand:
`mpremote exec "from machine import RTC; RTC().memory(b'0')"`, then reset.

**Getting tracks:** they live in `/gpx/`. With the boot bypass held:

```bash
.venv/bin/mpremote cp :/gpx/20261005_101500.trk .
python3 tools/trk2gpx.py 20261005_101500.trk > 20261005_101500.gpx
```

Next to a track you may find `<file>.pos` (upload progress) and
`<file>.bad` (batches the server rejected; retried until 8 more rejections
per boot), and `<file>.err` (a track that could not be read three times,
parked so the newer ones upload). `trk2gpx.py` converts `.trk.bad` and
`.trk.err` files the same way.
To queue a `.bad` again:

```bash
.venv/bin/mpremote exec "import os; os.rename('/gpx/20261005_101500.trk.bad', '/gpx/20261005_101500.trk')"
```

**Diagnostic log:** <a id="diagnostic-log"></a>the console is gone once
the board is on a power bank, so the logger also appends to `/diag.log` on
flash: a `boot cause N` line at every start, a `gps <model>` line once
the receiver says what it is (UBX-MON-VER, e.g. `gps u-blox M10, ROM SPG
5.10 (7b202e), PROTVER=34.10`; again after a receiver restart), one status
line a minute, a `stay left wifi` line when an away stay ends without a fix
(see [Stays](#stays)), `gps reset loop …` lines (when the receiver keeps
restarting, so they can be set against where you were), `upload err …`
and `upload aborted …` lines, a `reboot tls memory` line before a TLS
memory reboot, and a `crash` line before a crash reset. Past 32 KB the file
becomes `/diag.1` (the older one is dropped), so the two hold about 7 hours
at one line a minute; below the 32 KB flash reserve nothing is written.
minute. After a walk, plug the board in and read it:

```bash
.venv/bin/mpremote resume fs cat :/diag.log
.venv/bin/mpremote reset
```

| Field | Meaning |
|-------|---------|
| `boot cause N` | Why the chip started: 1 power-on (also a power bank switching back on), 3 watchdog, 5 soft or esptool reset, 6 crash (panic, brownout) |
| `prev_up` | On the boot line: how long the run before lasted, in seconds, from its last lines (to within a minute; `0` = under a minute, `-` = no log) |
| `up` | Seconds since boot (summed from tick differences, so it does not wrap after 12.4 days) |
| `utc` | GPS time, `-` until the receiver has one |
| `nmea ok/bad` | Valid and broken NMEA sentences so far |
| `gpsboots` / `loops` | Receiver restarts and reset-loop warnings so far |
| `sats seen/used` | Satellites with a signal (from the receiver's satellite list, polled once a minute) and used in the fix |
| `fix` | `No_data!`, `Waiting_fix`, `2D_fix`, `3D_fix` |
| `hdop`, `points`, `sent` | Last HDOP, points logged and uploaded since boot |
| `wifi`, `pulses`, `mem` | WiFi state, keep-alive pulses so far, free heap |

**Status page:** <a id="status-page"></a>while the board is connected to
WiFi it serves a page at `http://<board ip>/` (e.g.
`http://192.168.1.60/`, reloads every 15 s) and the same data as JSON at
`/status.json`: fix, satellites seen/used ("used" from the satellite list:
GGA stops at 12) and the satellite list with its age (system and number,
e.g. `GPS 7`, `Galileo 11`, `BeiDou 22`, `GLONASS 3`, `SBAS 123`; signal in
dBHz, elevation, azimuth, used), HDOP, GPS time, position, NMEA counts, the
receiver model, receiver restarts; points logged, waiting and uploaded,
quarantined `.bad` files, upload state, stay; uptime and how long the previous run lasted (e.g. `58 min
14 s`; seconds in the JSON), boot cause, chip temperature, free heap and flash,
WiFi, keep-alive pulses; and the last lines of the diagnostic log. It is
plain HTTP without a password: keep it on your own network. The request
head is read whole (up to 2 KB), the reply goes out in small pieces with
the GPS read in between, and the free flash is measured at most every 30 s. For the page
the radio stays connected at home (idling connected costs ~20–30 mA);
`WEB=0` in `.env` turns both off. On the other networks the radio sleeps
a few seconds after the last upload, so the page is up only briefly; with
`WEB_AWAY=1` it stays connected there too. Those networks use DHCP: find
the board's address in the hotspot or router client list (by its DHCP
hostname, `HOSTNAME`) or in the `web http://…/`
console line.

## How it works

### GPS

- **5 Hz with averaging.** At every boot the logger asks the receiver for 5
  fixes a second (UBX-CFG-RATE) and, to fit 9600 baud, for `RMC` and `GGA`
  every fix, `GST` once a second, `GSA` every 5 s (`GSA_EVERY`) and no
  `GSV`, `GLL` or `VTG` (UBX-CFG-MSG). A multi-GNSS receiver (M8, M10) sends
  one `GSA` per system: once a second that took the M10 to ~104% of the 960
  B/s, and it dropped sentences and the satellite list; now ~850 B/s. More
  than 8 `GSA` in a second (`GSA_MAX`) means the receiver did not take the
  rate, and it is set up again. The setup is not saved in the receiver.
- **Setup retries.** It is sent again while the receiver still sends the
  sentences it turns off, and also while the setup is not acknowledged (an
  ACK lost, a silent receiver that took the blind burst), less and less
  often.
- **Receiver filter** (UBX-CFG-NAV5, sent right after the 1 Hz frame):
  portable dynamic model (`GPS_DYN_MODEL`, `.env` `GPS_MODEL`: 3
  pedestrian, 4 automotive, …), 3D fixes only
  (`GPS_FIX_MODE`: a 2D fix guesses the altitude and is tens of metres off
  horizontally), satellites below 10° ignored (`GPS_MIN_ELEV`, the
  multipath-prone ones) and static hold below 0.5 m/s (`GPS_STATIC_HOLD`:
  the receiver freezes its position while parked; a walk at 1.4 m/s is well
  above). Only these four settings are applied (mask 0x47); the
  receiver's position masks stay as they are. A receiver that refuses NAV5
  (`gps nav5 refused`) still gets the 5 Hz setup.
- **Satellite list without GSV.** `GSV` does not fit next to 5 Hz, so the
  logger polls the satellite list once a minute (`SV_POLL`) and when a
  status page is opened and the list is older than 5 s (waiting up to 0.7 s
  for it). Until the receiver answered one of them it asks for both
  UBX-NAV-SVINFO (u-blox 6/7/M8, ~210 bytes; numbers only: GPS 1–32, SBAS
  e.g. EGNOS 120–158) and UBX-NAV-SAT (M8 and later, with the system of
  each satellite; an M10 knows only this one; elevation `-` while its
  position is not known yet), then only the one that
  answered. The binary answer is cut out of the NMEA stream by its length.
  UBX-MON-VER is asked for with it until the receiver names its model. A
  receiver restart (also a module swapped while running) forgets the list,
  the message and the model, so a new receiver is asked anew.
- **Reliable setup.** Every frame is sent until the receiver acknowledges it
  (UBX-ACK, up to 4 tries). It starts with a switch to 1 Hz, and 5 Hz is
  asked only once every message frame was acknowledged. Reason: at 5 Hz with
  all sentences on, the receiver's output overruns 9600 baud, and then it
  drops incoming frames (measured).
- **When it does not answer:** the message frames are sent once without
  waiting and the rate is left alone. If a frame goes unanswered mid-setup,
  the rest goes out the same way and it stays at 1 Hz. A setup blocks the
  loop about 4 s at most.
- **Retries.** While sentences it turned off keep coming, the setup is sent
  again: after 5 s, and for a receiver that never answers less and less often
  (up to 5 minutes). A start-up banner (power dip) sets it up again at once.
  A receiver that ignores everything (or an unwired ESP TX) stays at 1 Hz,
  and each second's average is then that one fix.
- **Averaging.** The valid `GGA` positions, altitude and HDOP of each UTC
  second are averaged and published when the next second starts (`RMC`
  positions if a second had no `GGA`). `GST` gives the receiver's own error
  estimate of that second. Above `AVG_MAX_SPEED` (10 km/h, the second's top
  `RMC` speed) the position is the second's first fix instead: the mean of
  five fixes lies ~0.4 s along the track but is stamped with the second,
  6 m behind at 50 km/h and 12 m at 100 km/h. Altitude and HDOP stay
  averaged.
- **Midnight.** Only `RMC` carries the date. A second that runs back past
  midnight without its own `RMC` has no date, so it is not a fix rather than
  a day off.

### Points and track files

- **Sampling.** Each averaged second is sampled once. Seconds published
  together after a blocking POST, scan or setup are queued and all sampled.
  A point is recorded when it is more than 3 m from the last one.
- **Filter.** Skipped: fresh HDOP above 5, or a GST estimate above 20 m (the
  larger of latitude and longitude, 1σ). A cold start can show a good HDOP
  while the position walks in from far away (measured: 230 m, over 65 s); GST
  catches it. 2D fixes omit altitude.
- **Format.** `/gpx/YYYYMMDD_HHMMSS.trk`: a `#TRK1` header, one point per
  line (`lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ`, about 46 bytes; ele and hdop
  empty when unknown), `-` for a new segment and an `#END` footer. Older
  `.gpx` files are still uploaded.
- **Power-loss safe.** Points are written in batches every 60 s. Each write
  replaces the footer with the new points plus the footer; LittleFS commits
  on close, so a power cut leaves the old or the new version. A line cut off
  by a power loss is skipped on read.
- **Segments and files.** A short fix loss starts a new segment (the `-`
  goes before the first point after it). A loss over 5 minutes starts a new
  file. On WiFi a file open for 5 minutes is closed so it can upload; driving
  offline stays one file, and parking does not open empty files.
- **Flash.** 32 kB are kept free. Logging stops when full and resumes when
  space is freed.

### Stays

Indoors the GPS wanders tens of metres with a good HDOP and reports 5–20 km/h
while lying still (measured on the board: speed p50 5.4 km/h, drift p90 46 m,
one-minute centroids up to 72 m apart). Neither the 3 m rule nor speed can
tell a stay from a drive, so the stay filter works on positions.

- **Entering.** Two consecutive one-minute centroids within 50 m make a
  stay. A window needs 10 usable fixes; with only a few a minute (indoors) it
  stays open until it has them, up to 5 minutes.
- **No drift on flash.** While a stay may still be proven, its windows'
  lines are held in RAM, and they are dropped when it begins, after a boot or
  a new fix too. The way in is kept up to where it first comes within 50 m of
  the anchor. Closing a file for upload never writes the held lines; they
  move on to the next file.
- **Anchor.** The mean of the two windows, refined by fixes within 50 m. If
  the device settles 50–150 m away (out of the car, into the building), three
  consecutive matching window pairs there (about four minutes) move the
  anchor. A shorter multipath shift does not.
- **Away from home.** The anchor is written as a point at once and every 5
  minutes (no elevation), so Dawarich still sees the stay. These points need
  no fix: they carry the last fix's GPS time plus the time since, and reach
  flash every minute. On WiFi the anchor file is closed every 30 minutes so it
  uploads.
- **At home** (a home network): no anchor points. If the home WiFi drops,
  anchor points start; it is still a stay.
- **Arrival.** Entering a stay closes the current track on any network, so
  the arrival uploads during the stay.
- **Leaving.** 3 consecutive samples beyond 150 m that each gain at least
  2 m (a vehicle), or 10 samples out there that gain at least 5 m in all (a
  walk). A multipath jump that freezes 200 m away does neither, however long
  it lasts, and an excursion that turns back does not count. The way out is
  written from near the anchor, from the last 90 samples, in time order.
- **Leaving without a fix** (away from home). The first 3 WiFi scans of the
  stay (keep-alive pulses, roam and connect rounds) note the access points
  around; later scans that see one of them add the others seen with it (a
  walk into another wing), the 16 most recently seen kept. Our own networks
  and locally administered BSSIDs (phone hotspots, a train's WiFi) are left
  out, since they travel along. When none of them shows up in 6 scans in a
  row over at least 60 s, the stay ends (`stay left, wifi`): no more anchor
  points at the old place. Before, a train ride home without a fix kept
  writing the old anchor every 5 minutes until home. Where no access point
  is around while learning, or with no scans (keep-alive `gpio`/`radio` or
  off, the radio connected), only a fix ends the stay, or:
- **Home without a fix.** An away stay ends once the home WiFi is up and no
  usable fix came for 30 s (`stay end: home without a fix`). Left running,
  it stamped the old place next morning and swallowed a trip that did not
  recede from it.
- **Drift hold.** What a stay still being proven holds in RAM is not the
  previous window when the open one is already 150 m beyond it (a drive),
  and it is written 10 minutes after the fix was lost (`HOLD_RELEASE`): the
  end of a trip into a garage or a house no longer waits in RAM only.
- **Leaving home** is accepted only once the home network is out of range
  (below −75 dBm: the RSSI when connected, else a ~2 s scan). While it is in
  range the departure is a GPS jump (measured: a 194 m jump at a desk) and
  further departures wait 60 s.
- **After a fix gap** (over 30 s): fixes more than 150 m from the open window
  restart both windows; more than 50 m restart only the open one, so two
  places are never mixed while what it held stays held.
- A walk (1.4 m/s) is never a stay: its centroid moves on by more than 50 m a
  minute.

### WiFi

**Choosing a network:**

1. Scan once, keep the strongest BSSID per SSID.
2. Try a home AP at ≥ −75 dBm first, then other visible known APs, strongest
   first, then configured networks the scan missed, in config order. A hidden
   home is preferred when the scan shows a strong empty-SSID AP. If the scan
   fails, config order is used.
3. Each try is polled from the main loop (~12 s deadline), so GPS sampling
   goes on. An AP seen at ≥ −75 dBm gets one immediate retry: on the C6 the
   first association after a cold boot often fails after ~7.5 s with status
   202 although the password is right, and the retry connects within ~1 s.
   Otherwise the SSID is marked failed for 2 minutes and the next one is
   tried. A leftover fail reason from the previous SSID (MicroPython 1.29) is
   not taken as this attempt failing.

**Radio on only when needed:**

- Once no finished track waits, nothing is in RAM and the open track holds
  no points, the radio is turned off (`wifi sleep`). On a home network with
  the [status page](#status-page) on (`WEB`, default) it stays connected;
  with `WEB_AWAY=1` on the other networks as well.
- After an hour it searches again, but only once there is something to
  upload (with `WEB` on, every hour anyway, so it finds home again).
- A new stay wakes it at once, so a home stay is known as one and the arrival
  uploads. Away from home the first anchor point waits 60 s for that round. A
  stay that began at home stays a home stay while the radio sleeps.
- If no network is found and there is something to upload, it retries with
  backoff (15 s doubling to 10 minutes) until one connects, the radio off in
  between; with nothing to upload it sleeps.
- While uploads back off (server down, 5xx, 401) the radio stays on only
  for a short backoff. Once the next attempt is `WIFI_OFF_WAIT` (60 s) or
  more away it is turned off (`wifi off until upload retry`) and a round
  reconnects when the attempt is due. A home stay stays a home stay
  meanwhile.

**Power bank keep-alive:** with the radio asleep the board draws less than
many power banks count as a load, and they switch off. The 20 Ah bank used
here switched off after 1 hour; with a scan every 15 s, after 3 hours, so it
averages the load. Every 5 s (`KEEPALIVE`) one pulse goes out, not while the
radio is on anyway (unless `KEEPALIVE_CONNECTED=1`), and also without WiFi
set up at all (no `.env`, no SSID). What the pulse is (`KEEPALIVE_MODE`):

- `scan` (default): the radio is turned on for one scan (~2 s at ~100 mA,
  about +40 mA on average) and off again. It blocks the main loop for the
  scan (GPS seconds are queued, not lost). The only pulse measured to keep
  the 20 Ah bank on longer.
- `radio`: the radio is turned on and off again, no scan. The start-up
  alone is a 100–200 mA burst for a fraction of a second, ~5 mA on average:
  less than the +13 mA that still let the 20 Ah bank switch off after 3 h,
  so only for banks that react to a burst rather than the average.
- `gpio`: the keep-alive pin (`KEEPALIVE_PIN`) is high for 0.1 s and drives
  a load resistor through a MOSFET (see [Hardware](#hardware)): a ~230 mA
  burst the bank cannot miss, ~5 mA on average, nothing blocked.

With the status page on (`WEB`), a keep-alive scan that sees a home network
at ≥ −75 dBm while the radio sleeps or found nothing starts a round at once
(`wifi home in range`): after a router restart the page used to stay down
for an hour. At most once a minute, doubling while those rounds fail.

**CPU clock:** the main loop runs at 80 MHz (`CPU_MHZ`), which saves
15–25 mA on the original ESP32 against its 240 MHz default. A POST runs at
160 MHz (`CPU_MHZ_UP`) so the TLS handshake stays quick, then the clock
drops back. The UART runs off the fixed APB clock, so 9600 baud is unaffected.
`CPU_MHZ=0` leaves the firmware default.

**Roaming home:** on a non-home network, when nearly stopped (speed below
5 km/h, or in a stay), the logger scans for home every 5 minutes and switches
to a home AP at ≥ −75 dBm. It backs off to 60 minutes only after a failed
switch to a visible home; stopping after a drive resets that only when a
visible home is in view. A hidden home is tried with `bssid=None` (matched by
SSID) when a strong empty-SSID AP is seen; a neighbour's hidden AP can cause
a false attempt, and switching to a hidden home can be slower.

### Upload

- Finished files go oldest first, 50 points per POST, to
  `<GEO_URL>/api/v1/overland/batches`. The file being recorded is never sent.
- Progress is a `<file>.pos` byte offset, written atomically. It advances
  only after HTTP 2xx or a durable copy to `<file>.bad`, so an interrupted
  POST is resent (Dawarich deduplicates).
- 400 / 413 / 422 quarantine the batch to `<file>.bad`. Three in a row
  without a success count as a server problem: backoff, no more quarantine.
  A full `.bad` (64 kB) pauses that file (`BAD FULL`) instead of dropping
  points; pull or delete it, then reset.
- Other errors back off from 15 s up to 30 minutes (`Retry-After` is
  honoured); 401 waits 30 minutes. Between POSTs it pauses 5 s at home, 30 s
  elsewhere. Flash and memory errors back off too, and with every waiting
  track held back (state `held`) the radio sleeps until the first is due.
- An unreadable track (`OSError` on read) is held back so the newer ones
  still go; after three tries it is parked as `<file>.err`.
- **Own HTTP client:** a minimal HTTP/1.0 POST over the `tls` module. The
  body goes out in 1 KB pieces and the wait for the answer (up to 30 s,
  `HTTP_WAIT`) polls, both reading the UART meanwhile, so a slow server
  costs no NMEA (the 4 KB UART buffer holds ~5 s). The upload line on the
  console ends with the POST time.
- **POST guard:** a hardware timer feeds the watchdog during a POST (it
  also runs while a socket or TLS call blocks) and closes the socket after
  60 s (`HTTP_MAX`, `upload aborted after 60 s` in the diagnostic log), so a
  hung call fails instead of the watchdog resetting the board (seen three
  times on 2026-10-30). A write that keeps saying "would block" ends after
  `HTTP_TIMEOUT`. Each new kind of upload error goes to the diagnostic log
  (`upload err OSError(116)`).
- **No redirects:** a 3xx is an error (`GEO_URL redirects (301)`): put the
  final https origin in `GEO_URL`. (micropython-lib `requests` followed one
  as a GET with the old Content-Length and the Bearer header, without the
  timeout.)
- **TLS:** without `/ca.der` the server is not authenticated (`CERT_NONE`):
  `https://` encrypts, but a MITM can still see the key and the points. A
  DER root certificate at `/ca.der` (`CA_FILE`; the one your server's chain
  ends in) turns verification on. With an `http://` `GEO_URL` the logger
  warns that everything goes in the clear.
- **TLS memory:** five TLS allocation failures in a row at home
  (`TLS_REBOOT_N`) restart the board cleanly (`reboot tls memory` in the
  diagnostic log): the IDF heap stays fragmented otherwise.

### Watchdog and recovery

- A 120 s hardware watchdog is fed in the main loop and inside long reads and
  POSTs.
- An uncaught exception is printed, the batch is flushed (without the drift
  held for a stay), and the board resets after ~1 s.
- The [boot loop guard](#using-it) stops repeated crashes; a missing or
  failing OLED is switched off instead of crashing.

## Configuration

Constants at the top of `gps_logger.py`. Defaults are tuned on the ESP32-C6
with a NEO-6M; most never need changing.

**GPS and points**

| Constant | Default | Meaning |
|----------|---------|---------|
| `GPS_HZ` | 5 | Fixes a second asked from the receiver, averaged into one point |
| `GPS_DYN_MODEL` / `GPS_FIX_MODE` | 0 (portable) / 2 (3D only) | UBX-CFG-NAV5 dynamic platform model and fix mode |
| `GPS_MIN_ELEV` / `GPS_STATIC_HOLD` | 10° / 50 cm/s | NAV5 elevation mask and static hold threshold (0 = off) |
| `GPS_LOOP_WINDOW` / `GPS_LOOP_BANNERS` | 10 s / 3 | Banner starts with no NMEA in this window = `gps reset loop?` |
| `GPS_FLAP_WINDOW` / `GPS_FLAP_RESTARTS` | 10 min / 5 | Full restarts within this window, NMEA or not = `gps reset loop?` |
| `GPS_STORM_WAIT` | 30 s | Least time between setups while the receiver keeps restarting |
| `AVG_MAX_SPEED` | 10 km/h | Faster than this a second's point is its first fix, not the mean |
| `GPS_ACK_WAIT` / `GPS_ACK_TRIES` | 1 s / 4 | Wait for a UBX-ACK, and tries per frame |
| `GPS_RETRY` / `GPS_RETRY_MAX` | 5 s / 5 min | Setup sent again while turned-off sentences keep coming (doubling only while it does not answer) |
| `SECONDS_Q` | 8 | Averaged seconds queued for the logger |
| `MAX_HDOP` | 5 | Skip samples with a worse fresh HDOP |
| `GPS_MAX_STD` | 20 m | Skip samples with a worse GST error estimate |
| `MIN_MOVE` | 3 m | Distance from the last point before a new one |
| `SAMPLE` | 1 s | Sampling interval when no averaged second is available (tests) |

**Track files**

| Constant | Default | Meaning |
|----------|---------|---------|
| `FLUSH` | 60 s | Interval between flash writes |
| `NEW_FILE_GAP` | 300 s | Fix loss that starts a new file; also the max age of an open file on WiFi |
| `MIN_FREE` | 32 kB | Flash reserve |
| `MAX_BATCH` | 240 | Points kept in RAM while flash writes fail |
| `HOLD_MAX` | 150 | Lines a stay still being proven keeps in RAM |

**Stays**

| Constant | Default | Meaning |
|----------|---------|---------|
| `STILL_WINDOW` / `STILL_MIN_FIXES` | 60 s / 10 | Centroid window and the usable fixes it needs |
| `STILL_WINDOW_MAX` | 5 min | How long a sparse window may stay open |
| `STILL_RADIUS` | 50 m | Max distance between two consecutive centroids |
| `STILL_NEAR` | 50 m | Fixes this close refine the anchor |
| `STILL_EXIT` / `STILL_EXIT_N` / `STILL_TOL` | 150 m / 3 / 5 m | Leaving: distance, samples in a row, allowed fallback |
| `STILL_STEP` / `STILL_EXIT_SLOW` / `STILL_SLOW_GAIN` | 2 m / 10 / 5 m | Gain per leaving sample, or samples out there and their total gain |
| `STILL_MOVE_HITS` | 3 | Consecutive window pairs 50–150 m away that move the anchor |
| `STILL_HEARTBEAT` | 5 min | Anchor point interval away from home |
| `STILL_FILE` | 30 min | Age at which an away stay's file is closed on WiFi |
| `STILL_HOME_HOLD` | 60 s | Wait after a refused home departure |
| `STILL_GAP` | 30 s | Time without a usable fix that counts as a gap |
| `STILL_DEP_MAX` | 90 | Recent samples kept to write the way out |
| `STILL_MEAN_N` | 1800 | Fixes the anchor mean averages over |
| `HOLD_RELEASE` | 10 min | Without a fix this long, what a stay being proven holds is written |
| `STAY_WIFI` / `STAY_WIFI_LEARN` / `STAY_WIFI_MAX` | on / 3 / 16 | Away stay ends when its access points are gone: on, scans that learn them, how many kept |
| `STAY_WIFI_MISS` / `STAY_WIFI_GONE` | 6 / 60 s | Scans in a row without any of them, and the least time they span |

**WiFi**

| Constant | Default | Meaning |
|----------|---------|---------|
| `CONNECT_TIMEOUT` | 12 s | Per-network wait for an IP (polled) |
| `WIFI_RETRY` / `WIFI_BACKOFF_MAX` | 15 s / 10 min | Reconnect backoff with something to upload |
| `WIFI_SLEEP` / `WIFI_IDLE` | 1 h / 10 s | Radio-off time once all is uploaded; how often that is checked |
| `WIFI_WAKE_WAIT` | 60 s | A new stay away waits this long for its WiFi round |
| `PB_KEEPALIVE` | 5 s | Keep-alive pulse interval while the radio sleeps (`.env` `KEEPALIVE`) |
| `PB_MODE` / `PB_PIN` / `PB_PULSE_MS` | `scan` / 26 (C6: 2) / 100 ms | Pulse kind, gpio and pulse length (`.env` `KEEPALIVE_MODE`, `KEEPALIVE_PIN`) |
| `WIFI_OFF_WAIT` | 60 s | An upload backoff at least this long turns the radio off until it is due |
| `ROAM_CHECK` / `ROAM_BACKOFF_MAX` | 5 min / 60 min | Home-roam scans on another network, and their backoff cap |
| `ROAM_MIN_RSSI` | −75 dBm | Minimum home RSSI for a switch (also boot preference and home range) |

**Upload**

| Constant | Default | Meaning |
|----------|---------|---------|
| `UPLOAD_BATCH` | 50 | Points per POST |
| `UPLOAD_GAP` / `UPLOAD_GAP_AWAY` | 5 s / 30 s | Pause between POSTs at home / elsewhere |
| `BACKOFF_START` / `BACKOFF_MAX` | 15 s / 30 min | Upload backoff (and `Retry-After` cap) |
| `AUTH_BACKOFF` | 30 min | Wait after HTTP 401 |
| `HTTP_TIMEOUT` | 15 s | Per-socket HTTP timeout (connect, TLS) |
| `HTTP_WAIT` / `HTTP_CHUNK` | 30 s / 1 KB | Wait for the server's answer, body piece; the UART is read between |
| `HTTP_MAX` | 60 s | Wall clock for one POST: the guard timer feeds the WDT, then closes the socket |
| `CA_FILE` | `/ca.der` | DER root certificate; if present, the server is verified |
| `TLS_REBOOT_N` | 5 | TLS allocation failures in a row (at home) before a clean reboot |
| `READ_ERR_MAX` | 3 | Read errors before a track is parked as `.err` |
| `MAX_BAD_BYTES` | 64 kB | Cap on a `.bad` file |
| `MAX_GLOBAL_PAYLOAD` | 3 | Payload errors in a row that stop quarantining |

**System**

| Constant | Default | Meaning |
|----------|---------|---------|
| `WDT_TIMEOUT` | 120 s | Hardware watchdog |
| `CPU_MHZ` / `CPU_MHZ_UPLOAD` | 80 / 160 | CPU clock in the loop and during a POST (`.env` `CPU_MHZ`, `CPU_MHZ_UP`); 0 = firmware default |
| `KEEPALIVE_MAX` | 10 min | How long a Ctrl-C timer feeds the watchdog |
| `BOOT_OK_MS` | 60 s | Healthy loop time that clears the boot-fail counter |
| `STATUS` | 10 min | Console `status` line interval |
| `DIAG` / `DIAG_MAX` | 60 s / 32 KB | Diagnostic log interval (`.env` `DIAG`) and size before `/diag.log` becomes `/diag.1` |
| `WEB` / `WEB_PORT` / `WEB_TIMEOUT` | on / 80 / 2 s | Status page (`.env` `WEB`), its port, how long one client may take |
| `WEB_AWAY` | off | Radio kept on for the page on non-home networks too (`.env` `WEB_AWAY`) |
| `SV_POLL` / `SV_FRESH` / `SV_WAIT` | 60 s / 5 s / 0.7 s | Satellite list poll interval, how old it may be for a page, how long a page waits for it |
| `SV_MISS` | 2 | Unanswered list polls after which a page no longer waits |
| `GSA_EVERY` / `GSA_MAX` | 5 s / 8 | GSA round interval; more GSA a second means the rate was not taken |
| `PB_CONNECTED` / `SEEN_WAIT` | off / 60 s | Pulses while connected (`.env` `KEEPALIVE_CONNECTED`); least time between home-seen wakes |
| `WEB_REQ_MAX` / `WEB_CHUNK` | 2 KB / 1 KB | Request head read by the status page; reply bytes per send |
| `DEVICE_ID` / `HOSTNAME` | `Dawarich-GPS-tracker` | Overland id and DHCP hostname (set in `.env`) |

## Development

GitHub Actions (`.github/workflows/ci.yml`) runs on pushes to `master` and on
every pull request:

- `python3 tests/test_gps_logger.py`: the CPython harness with mocked
  `machine`/`network`/UART (exits 1 on any FAIL), plus
  `tests/mpy_unix_check.py` and `tests/mpy_unix_logger.py` (imports
  `gps_logger.py` with stub `machine`/`network` modules and runs its pure
  paths) on a cached MicroPython 1.29 unix build; the smoke also runs
  under CPython.
- `mpy-cross` 1.29 for `xtensawin` (ESP32) and `rv32imc` (ESP32-C6).
- `pyflakes` on `gps_logger.py`, `main.py`, `tools/trk2gpx.py` and
  `tests/mpy_unix_logger.py`.

The same locally:

```bash
.venv/bin/pip install pyflakes "mpy-cross==1.29.*"
.venv/bin/python -m pyflakes gps_logger.py main.py tools/trk2gpx.py tests/mpy_unix_logger.py
.venv/bin/python -m mpy_cross -march=xtensawin -o /tmp/gps_logger.mpy gps_logger.py
MICROPYTHON=/path/to/micropython python3 tests/test_gps_logger.py
```

`tests/device_bench.py` is an optional on-device timing probe; nothing under
`tests/` is copied to the board.
