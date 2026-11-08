# ESP32 GPS Logger

[![CI](https://github.com/bator/esp32-gps-logger/actions/workflows/ci.yml/badge.svg)](https://github.com/bator/esp32-gps-logger/actions/workflows/ci.yml)

A MicroPython GPS track logger for the Heltec Wireless Tracker (ESP32-S3 with a
UC6580 on the board). It records your position to the internal flash, offline or
on WiFi, and uploads finished tracks to a [Dawarich](https://dawarich.app) server
through its Overland API. Tag `c6-last` holds the code for the older hardware
(ESP32 / ESP32-C6 with a u-blox receiver and an OLED).

- [At a glance](#at-a-glance)
- [Hardware](#hardware)
- [Setup](#setup)
- [Using it](#using-it)
- [How it works](#how-it-works)
- [Configuration](#configuration)
- [Development](#development)

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

## Hardware

Flash it with the custom firmware (MicroPython **1.29**, [below](#4-custom-firmware-recommended)).
The receiver is on the board, so nothing is wired. Measured on a board sold as
V1.2 (Heltec documents up to V1.1; its pins are the same), receiver UC6580I,
firmware R6.0.0.0Build3700.

| Function | GPIO |
|----------|------|
| UC6580 TX → ESP RX / ESP TX → UC6580 RX, 115200 baud (UART1) | 33 / 34 |
| Vext: high powers the UC6580, its LNA and the TFT | 3 |
| TFT (ST7735S, 160×80): SCK / MOSI / CS / DC / RST / backlight | 41 / 42 / 38 / 40 / 39 / 21 |
| Boot bypass → GND (GPIO0, the button, is the download mode) | 7 |

- **Build:** `tools/build_firmware.sh` with `--migrate` once, `--flash` after. It
  needs the S3 toolchain: `~/esp/esp-idf/install.sh esp32s3`. The app (~1.4 MB) fits
  two 1792 KB OTA slots, the file system is ~4.4 MB (about 7 days of driving
  offline).
- **Its own OTA folder:** give the board its own `OTA_DEST` and `.env`
  `OTA_URL`. Its manifest names the board (`GPS_LOGGER_S3`), and it refuses any
  other, or one that names none.
- **Setup:** the logger sets the receiver up with Unicore commands (`$CFGNAV`,
  `$CFGMSG`: 5 Hz, GGA and RMC every fix, GST once a second, GSA and GSV every 5 s)
  at every boot, as a
  power cycle brings its defaults back. With a fix it sends GSV and GSA at 1 Hz
  whatever the rate (~2.3 KB/s). The satellite list comes from GSV, decoded every
  5 s.
- **Display:** ink on paper. On the left today's route, a line of dots over a faint
  grid, north up and fitted to the field (at least 500 m across): the latest trip
  red, earlier ones black, a round dot where the day began and a red one where it
  is now, and on an empty day "nincs út". Next to the WiFi bars today's km ("8,6 km",
  whole km from 100). A trip starts when a stay ends or a fix gap starts a new file;
  anchor points and the drift a stay drops add nothing, so the km are those of the
  track, up to a minute late. "Today" is the Hungarian day. On the right the clock
  (Hungarian time), a satellite icon with the satellites in the fix (red without a
  fix), the four WiFi bars and what waits for upload. Below, how good the GPS is (a
  tile in its colour with a word) and how full the flash is with how long it still
  logs (a gauge, amber from 70%, red from 90%; the time assumes moving at 1 Hz,
  standing costs next to nothing). The speed is on the status page and in
  `status.json`. It never scrolls. The backlight is always on. The route survives a
  reboot in `/route.bin` (at most one write per 5 min of driving and one when a stay
  starts) and shows again at once; the first fix or NTP answer empties it if it was
  another day, so until then a board switched on the next morning, away from any
  WiFi, shows yesterday's. Before the first fix the clock comes from NTP on any WiFi
  (`NTP_SERVER`), and `--:--` without one; while the receiver is silent it runs on
  from the last fix by the board's ticks. `gps_tft.py`, its
  anti-aliased fonts `gps_tft_font.py` (`tools/make_tft_font.py`).
- **Antennas:** the one in the box goes on the LoRa socket, which the logger
  does not use. The GNSS works from the board's own antenna, with the GNSS
  socket empty: 22 satellites in use, HDOP under 1, ~3 minutes to the first
  fix after a power cycle (the receiver keeps nothing without power).
- **Accuracy, measured standing outdoors:** half of the fixes within 2.3 m of the
  middle after two minutes, 95% within 3.8 m, HDOP 0.9, ~26 satellites in use. It
  starts 15 m off and takes about a minute to reach 5 m, so the first minute after
  a start (boot or receiver restart; a fix regained later is not held back) counts
  only above 15 km/h. Its GST does not show this (0.3 m while 4 m off).
  It refuses SBAS (`$CFGSYS` bit 20: `$FAIL`).
- **`GPS_SYS`** in `.env`: its `$CFGSYS` mask in hex, set once a boot when the
  receiver has another (it restarts it); without it the receiver's own stays.
- **L5:** every system and band is on by default (`$CFGSYS` `H35155`), but
  the board's antenna is L1 only, per Heltec's FAQ; an external one takes a
  moved resistor.

## Setup

### 1. Host tools

Linux with Python 3:

```bash
python3 -m venv .venv
.venv/bin/pip install mpremote "mpy-cross==1.29.*"
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
# CPU_MHZ=80
# DIAG=60
# WEB=1
# WEB_AWAY=0
```

| Key | Meaning |
|-----|---------|
| `SSID`, `WPA2` / `SSID2`, `WPA2_2` / `SSID3`, `WPA2_3` | Networks #1–#3 |
| `HOME1` / `SSID_HOME`, `HOME2` / `SSID2_HOME`, `HOME3` / `SSID3_HOME` | `1`/`0` (or `true`/`false`): treat as home. #1 is home by default, #2 and #3 are not. |
| `SSID_HIDDEN`, `SSID2_HIDDEN`, `SSID3_HIDDEN` | `1`: hidden SSID, tried in every round although the scan never lists it. Set it for every hidden network: without it a hidden SSID is tried only in the first round after a link loss, a wake or an arrival, and then every 4th round (up to ~40 minutes apart). |
| `IP`, `GW`, `MASK`, `DNS` (and `IP2`…, `IP3`…) | Static IP per network. `GW` is required; `MASK` defaults to `255.255.255.0`, `DNS` to the gateway. An invalid entry falls back to DHCP with a console message. |
| `GEO_URL` | Dawarich origin. Use `https://`; see [Upload](#upload). |
| `API_KEY` | Dawarich API key, sent as `Authorization: Bearer <key>` |
| `DEVICE_ID` | Overland tracker id, `[A-Za-z0-9._-]{1,32}` |
| `HOSTNAME` | DHCP hostname, `[A-Za-z0-9-]{1,32}` (defaults to `DEVICE_ID`) |
| `KEEPALIVE` | Seconds between power bank keep-alive pulses, 5–600; `0` turns it off (e.g. on a car charger). Default 5. |
| `CPU_MHZ`, `CPU_MHZ_UP` | CPU clock in the main loop and during a POST: 20/40/80/160/240; `0` leaves the firmware default. Defaults 80 and 160. |
| `MIN_MOVE` | Metres from the last point before a new one, 0–50 (decimals allowed). Default 1: on foot about one point a second; 3 gave one every ~3 s. Read at boot. |
| `OTA_URL` / `OTA_KEY` | Firmware updates over WiFi: the manifest's URL (http(s), on the LAN) and its HMAC key, the one in `OTA_KEY_FILE` on the build machine. Both or no updates. See [OTA](#5-firmware-updates-over-wifi-ota). |
| `NTP_SERVER` | Where the display's clock and the route's day come from until the first fix. Default `pool.ntp.org`; empty: no NTP, `--:--` until a fix. A fix always wins; NTP never reaches a track, and sets the board clock (TLS certificate dates) only while no fix came since the boot. An answer counts only if it echoes the request (its originate time is the request's transmit time: a stray or late datagram is none), and the time is read past the NTP era change of 2036-02-07 too. |
| `GPS_SYS` | The UC6580's `$CFGSYS` mask in hex (see [Hardware](#hardware)). Read at boot. |
| `DIAG` | Seconds between [diagnostic log](#diagnostic-log) lines, 10–3600; `0` turns it off. Default 60. |
| `WEB` | `1`/`0`: the [status page](#status-page), and the radio staying on at home for it. Default 1. |
| `WEB_AWAY` | `1`/`0`: with `WEB` on, the radio stays on for the page on the non-home networks too (e.g. a phone hotspot), instead of sleeping once all is uploaded. Default 0. |

- Put comments on their own lines. A trailing `# ...` is ignored only on
  flag, IP and name keys (`HOME*`, `*_HIDDEN`, `IP*`/`GW*`/`MASK*`/`DNS*`,
  `DEVICE_ID`, `HOSTNAME`, `KEEPALIVE`, `CPU_MHZ*`, `GPS_SYS`, `MIN_MOVE`, `DIAG`, `WEB`, `WEB_AWAY`). SSIDs, passwords, `GEO_URL` and
  `API_KEY` are taken verbatim, since they may contain `#`.
- Values may be quoted. On the keys above the comment goes first, then the
  quotes: `IP="192.168.1.60" # static` is `192.168.1.60`.
- Invalid names fall back to the defaults. A UTF-8 BOM (Windows Notepad) is
  ignored. A file saved as ANSI does not stop the boot: an SSID or password
  that is not UTF-8 is used byte for byte, any other such value is skipped
  with a console line.

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
PORT=/dev/ttyACM0 ./deploy.sh    # the board's USB Serial/JTAG
```

The script asks the board what it is (anything but the Heltec is refused:
`Unsupported board`), compiles `gps_logger.py` to `gps_logger.mpy` when the
firmware has no frozen logger and checks the free flash (it stops with `Not
enough free flash` when the new files would not fit next to the old ones).
It makes `web.html.gz` with `tools/web_min.py` (~27 KB instead of the ~94 KB
`web.html`: comments and indentation out, gzipped with the firmware's 4 KB
window, `WEB_GZ_WBITS`: see the [status page](#status-page)) and copies
`gps_logger.mpy`, `main.py` and the status page, only as `web.html.gz`, as
`<name>.dpl`, checks their size and SHA-256 on the board, and only then
renames them over the old files, removing an old plain `/web.html` in the
same step (~90 KB of flash back); a failed check (`Copy check FAILED`)
removes the `.dpl` files and keeps the old install, so a copy cut short on
a nearly full flash can no longer leave an unimportable module (not
`<name>.new`: that is how an OTA stages the page it installs). Its first
question to the board is whether an OTA install is being validated
(`/ota.pending`, about 2 minutes): then it stops before anything else, as
its reset would abort the pending firmware, the bootloader would roll back
and the build would be blamed (`FORCE_PENDING=1` deploys anyway, the staged
page left alone, and ends without a reset: the logger runs again, the new
files count from the next boot; `FORCE=1` does not, it is for a stale
frozen logger only). Then it
removes a stray `gps_logger.py`, clears the boot-fail counter and resets
the board. When it cannot tell whether the firmware has the logger frozen
in (no clear answer from the board), it stops instead of guessing.
Each `mpremote` session stops the logger (raw REPL), so every early exit
(an OTA pending, too little flash, a failed copy check, a stale frozen
logger, no clear answer) starts it again: by a reset, or, with an OTA
pending (or no answer to that question), by running `gps_logger.run()`
without one. Left in the REPL, the board waited for the watchdog's reset,
counted as a boot failure, and that reset aborted a pending OTA.

The module must run precompiled: compiling on the device grows the Python
heap and leaves too little ESP-IDF heap (~40 kB contiguous) for mbedTLS, and
HTTPS uploads then fail with `ENOMEM`.

### 4. Custom firmware (recommended)

Even precompiled, the logger's bytecode sits in the Python heap, and any growth
of that heap takes a whole block of the IDF heap that TLS needs.
`tools/build_firmware.sh` builds MicroPython with `gps_logger` frozen into the
firmware, so the bytecode runs from flash, and the heap starts at 96 KB in one
piece (`boards/GPS_LOGGER_S3/mpconfigboard.h`).

It needs ESP-IDF v5.5.2 and a MicroPython v1.29.0 tree (by default
`~/esp/esp-idf` and `~/esp/micropython`; see MicroPython's
`ports/esp32/README.md`):

```bash
tools/build_firmware.sh                          # build only
PORT=/dev/ttyACM0 tools/build_firmware.sh --flash
PORT=/dev/ttyACM0 ./deploy.sh                    # main.py and the page
```

Freezing does not stop the Python heap from growing into the IDF heap
(MicroPython's split heap stays on); it leaves so much of the 96 KB heap
free that an allocation rarely has to.

The logger, `gps_boot.py` and the display's modules are compiled with
`-O3` (`opt=3` in the build's manifest, `-O3` in `deploy.sh`): their line
numbers stay out of the bytecode, ~10 KB of the logger's ~98 KB. The cost: a
traceback on the USB console no longer says where (each frame reads `line 1`,
the function names stay); the crash line in `/diag.log` holds `repr(e)` and
never had a line number. To see them while debugging, build without it.

`--flash` first checks the board's partition table and refuses to flash
when it differs from the build's. Then it backs up the app partition to
`firmware/` and writes only the app at 0x10000: the bootloader, the
partition table and the file system (`.env`, tracks) stay as they are.
Nothing on the file system is touched before or by the write, so a failed
check leaves the board as it was. A failed write does not: it always goes
to the first slot, which is the running one unless an OTA switched to the
second, so the board may not boot an app afterwards. The ROM download mode
over USB is never touched: run `--flash` again, or write the backup back
with the `esptool.py` command it prints (with the second slot running, the
board still boots that one). Run `./deploy.sh` right after: it
removes the old `gps_logger.mpy` (until then it shadows the frozen module
and runs as before) and any stray `gps_logger.py` (`''` comes first in
`sys.path`), and clears the boot-fail counter. Right after esptool's reset
the raw REPL may not answer for a while; if `deploy.sh` cannot reach the
board, run it again.

The firmware also freezes `gps_boot.py`: what `main.py` does around the
logger's start for OTA and the boot-fail retry. Frozen, its code is not
compiled into the heap at every boot; `main.py` runs without it on a
plain firmware. And it carries a small `gps_frozen` module with the commit
(`-dirty` when the tree had uncommitted changes) and the SHA-256 of the
frozen `gps_logger.py`. When `deploy.sh` finds it, it copies only `main.py`
and the page and removes a stray `gps_logger.mpy`, but first compares that
SHA-256 with the repo's `gps_logger.py`: when they differ (or a firmware
built before the SHA was recorded) it stops and asks for
`tools/build_firmware.sh --flash`; `FORCE=1 ./deploy.sh` deploys main.py
and the page anyway. A logger change means building and flashing the
firmware again. In an emergency `FORCE_MPY=1 ./deploy.sh` installs
`gps_logger.mpy` on a frozen firmware too, with a warning: it shadows the
frozen module and takes its ~100 KB of Python heap again. To roll back,
write the backup with the `esptool.py` command `--flash` prints, then run
`./deploy.sh`.

### 5. Firmware updates over WiFi (OTA)

The firmware is built for `boards/GPS_LOGGER_S3`: MicroPython's
`ESP32_GENERIC_S3` with two 1792 KB OTA app slots
(`boards/GPS_LOGGER_S3/partitions-ota.csv`, 8 MB flash; the file system takes
the ~4.4 MB after them) and the bootloader's rollback on. It leaves out what the
logger never uses: BLE, ESP-NOW, Ethernet, mDNS queries, the SD card and I2S (the
mDNS responder stays, for `http://<HOSTNAME>.local/`), and of the frozen packages
all but the port's own modules (asyncio, umqtt, webrepl, mip, requests...).

**Once, over USB:** `--migrate` moves a board to this table. It erases the
whole flash, file system included, so first let every point upload and copy
the files off:

```bash
# /.env holds the WiFi, API and OTA keys: outside the repo, mode 600
B=~/.config/gps-logger/migrate-$(date +%Y%m%d); mkdir -p "$B" && chmod 700 "$B"
.venv/bin/mpremote connect /dev/ttyACM0 cp :/.env "$B/env" + cp -r :/hist "$B/" + reset
chmod 600 "$B/env"
PORT=/dev/ttyACM0 tools/build_firmware.sh --migrate   # 8 MB only; backs up the first 4 MB
# put .env back (with OTA_URL / OTA_KEY added), /hist too, then:
.venv/bin/mpremote connect /dev/ttyACM0 cp "$B/env" :/.env + cp -r "$B/hist" :/
PORT=/dev/ttyACM0 ./deploy.sh
```

The backup `--migrate` writes (`firmware/full-backup-*.bin`, mode 600)
holds the start of the old file system, `/.env` and its keys too: keep it
off shared places and delete it once the board runs. `.gitignore` also
ignores `env.backup`, `hist/` and `*.env*`, for a copy made in the repo
anyway.

**Then:** `tools/build_firmware.sh --publish` writes `firmware/ota/`
(`micropython-<commit>.bin`, the page as `web-<commit>.html.gz` with the
firmware's small window, `manifest.json` naming only `/web.html.gz`; an
older firmware takes such a manifest too) and copies it with `scp` to `OTA_DEST` (`host:/path`), a
folder a web server on the LAN serves: each file as `<name>.tmp`, moved
over the live one with `ssh` (one rename: a board downloading a
republished `.bin` keeps reading the old one, and none reads a half-copied
manifest), the files first, the manifest naming them last, so a board
reading the old manifest meanwhile still finds its files. A build that is
not newer than the published manifest's is refused (a board only takes a
newer one; `FORCE=1` publishes it anyway). The
manifest carries the build (the commit time), the commit, size and SHA-256
of each file, and an HMAC-SHA256 over them with the key in `OTA_KEY_FILE`
(default `~/.config/gps-logger/ota.key`). The key is printable ASCII without
quotes or spaces, 16 characters or more (the board reads it from `.env`):
`openssl rand -hex 32 > ~/.config/gps-logger/ota.key`. Only a clean,
committed tree is published. A `--flash` or dirty build carries the build
time instead of the commit time, so the OTA does not replace what was just
flashed by hand with an older published build.

```bash
OTA_DEST=user@host:/srv/www/gps-ota tools/build_firmware.sh --publish
```

The `OTA_DEST` path is a plain one: no spaces or shell specials (`scp` takes
it as it is, the `ssh` rename shell-quoted; the two agree only on such a
path).

The board, with `OTA_URL` (the manifest's URL) and `OTA_KEY` (the same key)
in `.env`, checks 2 minutes after boot and then every 10 minutes (a ~1 KB
manifest on the LAN), only on a home
network with no track file waiting for upload (`OTA_FIRST`, `OTA_CHECK`; the
points in RAM are written before the reset). A build newer
than its own, whose HMAC checks and that has not failed before, goes into
the other app slot 4 KB at a time, with the GPS read and the watchdog fed
between (~1 min; the HTTP body ends at its Content-Length). It is read back
against its SHA-256, the page files are staged as `<file>.new`, the slot is
set to boot and the board resets. Before any of that the manifest's fields
are checked (whole numbers, a 64-hex SHA-256, no `|` in a string, the
commit 1–20 lowercase hex with an optional `-dirty`, only `/web.html` and
`/web.html.gz`, each once; the published manifest names only the gzip,
older ones both): the HMAC covers them joined by `|`, so a
manifest with a `|` moved between fields would keep it (`bad manifest`;
the format itself is unchanged). The HMAC is compared in constant time.
A refused manifest is one diagnostic line until a good one comes, and no
unverified field goes into it (a commit with newlines wrote forged lines);
any diag line is one line, cut at 320 bytes (`DIAG_CUT`; a status line
runs to ~220). A `/gps_logger.mpy` or `.py`
on flash (`FORCE_MPY=1`) would shadow the new firmware's frozen logger
(`''` comes first in `sys.path`): on a frozen firmware the install moves
it aside as `<name>.ota-old`, the valid new firmware deletes it, and
after a rollback `main.py` puts it back before the import. A Ctrl-C
between `set_boot` and the reset no longer skips the reset (left there,
the next reset booted the new slot with no pending note, valid without a
health check); a boot slot that is set but not the running one at boot
(no reset since) is booted once. The new firmware runs pending:
2 minutes with WiFi connected (`OTA_VALID_AFTER`), and the manifest read
once more (HTTP 200, the fields and the HMAC as above, every minute until
it is: `OTA_PROBE`; a build whose update path is broken would otherwise
only get the next one over USB, and a manifest signed with another key no
longer makes it valid), mark it valid, and only then do the
staged page files replace the old ones (on a file system that does not
rename over a file, the old one is moved aside first and back if the new
one still does not go up; a page that does not go up is a diag line,
`ota <commit> page <path> not put up`). A `/web.html.gz` that went up
removes the old plain `/web.html` (unless the manifest named it too): the
board keeps the page only gzipped. Not earlier: a rollback keeps the old
firmware's files. A crash or reset before that and
the bootloader boots the old slot; no WiFi or no manifest within 10 minutes
(`OTA_HEALTH`) and it resets itself, with the same result (it never sets
the old slot to boot itself: that left the old image pending, and the next
reset fell back to `ota_0`). A download that fails its SHA-256 or is cut
short (a network error, a short body), or a page file that fails its
SHA-256 or cannot be written, is tried again an hour later (`OTA_RETRY`),
not every check (the image came again every 10 minutes, the loop blocked
for it); an error (server down) goes to the diagnostic log once per kind.
Points waiting for upload hold the check back only with a `GEO_URL` and
`API_KEY` to send them to (without, they would hold it for good). A build that crashed (the logger's crash
branch notes it in `/ota.mark`) or whose watchdog fired while the logger
ran is noted in `/ota.bad` and not tried again. Any other rollback may
not be the build's fault and is a try instead, counted per build in
`/ota.tries`: at power-on (the cable pulled; MicroPython 1.29 reports a
brownout as a power-on too, and a build that browned out on a weak supply
was reinstalled after every boot), the no WiFi / no manifest reset (a
router or server outage), and a reset from outside (`mpremote reset`, a
USB-JTAG reset, the watchdog after an `mpremote` session stopped the
logger: Ctrl-C notes `stop`; each of these marked a good build bad at
once). A panic is a try too: it reads as a hard reset, like `mpremote
reset`, and no Python code runs to note it. The build is tried again an
hour after the next boot (`OTA_RETRY`) and is bad after 2 tries
(`OTA_TRIES_MAX`); the health reset removes the note before it counts its
try, so a power cut between them cannot count one attempt twice, and it
writes the points in RAM first. A boot without a pending note confirms
the running image. Points waiting for upload hold the check back, but not
for good: with no successful upload for 24 hours (`OTA_STUCK`; `err
payload` is cleared by a 2xx only, so an upload bug held back the update
that fixes it) it checks anyway, one diag line `ota: uploads stuck`.
Each step is in the
diagnostic log (`ota … installed`, `booted, checking`, `ok`, `rolled back`).
The status page shows the running build under *Systems*: the `FIRMWARE` row
gives the build's date (the commit time, `fw_build` in the JSON, local
time) and the OTA state (`up to date`, `updated`, `checking`, `rolled back`,
…); the commit is in the panel's subtitle.

After the migration, `--flash` over USB still works. It backs up both slots
(after an OTA the second one runs), writes the first slot and erases
otadata, so that slot boots even after an OTA had switched to the other.

## Using it

Power it on: the logger starts by itself, sets up the GPS, connects to WiFi
and shows the route and its state on the display (see [Hardware](#hardware)).

**Serial console** (115200 baud) lines worth knowing:

| Line | Meaning |
|------|---------|
| `gps setup ok` / `no ack` / `again` | 5 Hz setup answered, not answered, or sent again |
| `gps setup not taken: 1 fix/s` / `no GST` | 10 s after an answered setup (once it has 3 valid seconds to tell by) the receiver still sends one fix a second, or no `GST`: it is set up again. Also in the diagnostic log, at most every 10 minutes |
| `gps setup unverified: no fix` | 30 minutes after an answered setup still no fix to check it by: no verdict |
| `gps setup lost: no GST` | No `GST` for 30 s with sentences coming (`UC_GST_LOST`): a restart whose banner was lost; it is set up again |
| `gps refused <command>` | The receiver answered a command `$FAIL`. A refused rate (`CFGNAV`): it stays at its rate, the rest of the setup goes on, and 1 fix/s is then no "not taken" verdict |
| `gps restarted` | The receiver printed its start-up banner (power dip); it is set up again at once |
| `gps reset loop? 5 restarts in N s` | The receiver restarted 5 times within 10 minutes, with NMEA in between (seen on the board after a flash): check the GPS power and wiring. While it loops, setups are 30 s apart (`GPS_STORM_WAIT`): each one blocks the loop for seconds. It counts as over after a quiet of twice its last gap between restarts (10-30 s) |
| `wifi try` / `retry` / `fail` / `ok` | Connect attempts; `fail` carries the raw `status()` code and the time |
| `wifi lost <ssid> rssi N` / `wifi fail <ssid> status N after N ms` / `wifi back <ssid> after T, N failed tries, rssi N` | Also in the diagnostic log (and on the status page): the link dropping with its last RSSI, a failed try, and the reconnect with how long it was down. One line of a kind per 5 minutes (`WIFI_DIAG_GAP`), the ones in between counted as `(+N more)` |
| `wifi sleep [home]` | Radio off, everything uploaded (`home`: during a home stay) |
| `upload <file> <n> <ok> <http> <mem>` | One POST; `mem` is the free Python heap in bytes (`gc.mem_free()` without the largest IDF block) |
| `stay home\|away kept N` | A stay began; N points of the way in were kept |
| `stay moved` / `stay kept: home in range` / `stay end D` | Anchor moved; a home departure refused; the stay ended D m out |
| `stay end: home without a fix` | An away stay ended at home without a usable fix |
| `wifi home in range` | With `WEB`, a keep-alive scan saw home: a round starts now |
| `upload <file> … <ms> ms` / `GEO_URL redirects (N)` / `parked <file>.err` | POST time; a redirect (not followed); an unreadable track set aside |
| `stay left, wifi: none of N access points in M scans` | An away stay ended without a fix: its access points are gone (also in the diagnostic log as `stay left wifi`) |
| `status temp … hz … used … gpsloops … hdop … points … sent … wifi … pulses … mem … idf … heap …` | Every 10 minutes: chip temperature (the ESP die, not the GPS), fixes in the last second, satellites, GPS reset-loop warnings so far, then what the diag line ends with: HDOP, points, uploads, WiFi state, keep-alive pulses so far, free heap, IDF heap free/largest block (KB), Python heap size (KB) |
| `wifi off until upload retry N s` | Uploads back off for N s (server down, 401): the radio sleeps until then |
| `cpu freq N …` | The board refused that CPU clock; it runs at the firmware default |

**Boot bypass:** hold GPIO7 low while resetting to
skip the logger: REPL and `mpremote`, no watchdog. Use it to copy files off
the device.

**Ctrl-C** on a running logger leaves the watchdog armed. A timer feeds it
for 10 minutes, then the chip resets, so a stray Ctrl-C in the field does not
leave the device in the REPL for good.

**Boot loop guard:** failed starts are counted (RTC memory, `/boot_fail` as
fallback); at 5 the board stays in the REPL instead of starting the logger.
What counts, and what follows:

- an import failure of `gps_logger`, or an exception escaping `run()`: the
  count goes up, then `main.py` waits 5 s per failure so far (5 s, 10 s, …
  at most 30 s) and resets;
- a crash inside `run()` (the main loop): the count goes up, the batch is
  flushed, a `crash` line goes to the diagnostic log and the board resets
  after 1 s (no growing delay);
- a watchdog reset (boot cause 3): counted at the next boot, no delay.

A panic or a brownout is not counted: MicroPython 1.29 reports a panic as
boot cause 2 (hard reset), the same as the logger's own `machine.reset()`
(after a crash, the TLS memory reboot, `mpremote reset`), and a brownout as
1 (power on). The counter is cleared after 60 s (`BOOT_OK_MS`) of a healthy
main loop, by `deploy.sh` or by a power cycle. By hand:
`mpremote exec "from machine import RTC; RTC().memory(b'0')"`, then reset.
A bug in the WiFi, upload or OTA tick, the status page's or a status
line's, no longer resets the board: it is
one diagnostic line a kind (`bug <tick> <error>`), that tick pauses 5 s
(`TICK_PAUSE`) and logging goes on; an upload bug also backs the upload
off. Ten MemoryErrors of one tick in a row (`GUARD_MEM_MAX`, counted per
tick: the others running through meanwhile clear nothing) are a memory
reboot, as the crash reset used to be. A bug in the logging itself, and any
bug while an OTA image is pending, still goes to the crash branch, except a
MemoryError in any guarded tick: no reason to blame the new build for a low
heap in its first minutes (it went to `/ota.bad` for good).

In the REPL after 5 a frozen firmware no longer waits for good: after 30
minutes (`BOOT_RETRY_MS`) a timer sets the count back to 4 and resets, one
more start (a failure is another 30 minutes in the REPL). That timer can
reset an `mpremote` session that runs past it: `gps_boot._retry.deinit()`
at the REPL stops it, and the boot bypass skips all of it. A plain firmware
(the `.mpy` from `deploy.sh`) stays in the REPL. A failure that `main.py`
catches while an OTA image is pending (it cannot even import) is marked
as that build's crash, as the logger's own crash branch does.

**Getting tracks:** they live in `/gpx/`. With the boot bypass held:

```bash
.venv/bin/mpremote cp :/gpx/20261005_101500.trb .
python3 tools/trk2gpx.py 20261005_101500.trb > 20261005_101500.gpx
```

New tracks are binary `.trb` files (see Format), older ones `.trk` text;
`trk2gpx.py` converts both. Next to a track you may find `<file>.pos`
(upload progress) and `<file>.bad` (batches the server rejected; retried
until 8 more rejections per boot), and `<file>.err` (a track that could not
be read three times, parked so the newer ones upload). `trk2gpx.py`
converts `.bad` and `.err` files the same way. A `.trb.bad` holds the
rejected points as text lines, like a `.trk`.
To queue a `.bad` again (a `.trb.bad` becomes a `.trk`):

```bash
.venv/bin/mpremote exec "import os; os.rename('/gpx/20261005_101500.trk.bad', '/gpx/20261005_101500.trk')"
```

**Diagnostic log:** <a id="diagnostic-log"></a>the console is gone once
the board is on a power bank, so the logger also appends to `/diag.log` on
flash: a `boot cause N` line at every start, one status line a minute, a `stay left wifi` line when an away
stay ends without a fix (see [Stays](#stays)), `gps reset loop …` lines
(when the receiver keeps restarting, so they can be set against where you
were), `gps setup not taken: …` (at most one per 10 minutes,
`GPS_NOT_TAKEN_SAY`, with `(N more)` for the ones between), `gps setup lost: no
GST`, `gps refused <command>`, `gps silent N s, no bytes from the receiver` and
`gps back after N s` (no byte from the receiver for a minute, `GPS_SILENT`),
`upload err …`
and `upload aborted …` lines, `upload low idf N KB` / `upload low mem N B`
when a POST waits for memory, `upload ca err …` when `/ca.der` cannot be
used, `clock set <UTC>` once a boot when the board clock is set from the
GPS (`clock set <UTC> ntp`: from NTP, no fix since the boot), `clock ntp
<UTC>` when NTP tells the display the time before a fix, `heap grew in <step> to N KB` when the
Python heap grew during an upload step (`read_batch`, `connect`,
`tls_wrap`, `post`, the whole POST: the step that took IDF heap), a
`reboot tls memory` or
`reboot low idf N KB S s` / `reboot low mem N B S s` line before such a
reboot, and a `crash` line
before a crash reset. A reset loop that goes on writes its first warning,
then `gps reset loop N more windows` every 5 minutes (`GPS_LOOP_SAY`) and
once when it ends, not a line every 10 s. Past 160 KB (`DIAG_MAX`) the
file becomes `/diag.1` (the older one is dropped), so the two hold about
26 hours of status lines at ~210 B a minute (~13 h right after the switch,
when `/diag.1` is all there is; a board on NMEA writes ~170 B lines: 16-32
h). They take up to 320 KB of the ~12 MB file system; below the 32 KB flash
reserve nothing is written. Nothing reads a whole file into RAM: the
rotation only looks at the size, the boot reads the last few lines, and the
status page reads both files in 16 KB pieces (`/diag?f=1|0&o=<byte>`, see
the *Event log* below; `/status.json` still carries the last 15 lines,
`WEB_DIAG`). After a walk, plug the board in and read it:

```bash
.venv/bin/mpremote resume fs cat :/diag.log
.venv/bin/mpremote reset
```

| Field | Meaning |
|-------|---------|
| `boot cause N` | Why the chip started (`machine.reset_cause()` of MicroPython 1.29): 1 power-on (also a brownout, or a power bank switching back on), 2 hard reset (the logger's own reset after a crash or for TLS memory, `mpremote reset`, a panic), 3 watchdog, 4 deep sleep, 5 soft reset (Ctrl-D), 0 other (e.g. a reset over USB-JTAG) |
| `prev_up` | On the boot line: how long the run before lasted, in seconds, from its last lines (to within a minute; exact before a deliberate reset, which writes a status line first; `0` = under a minute, `-` = no log) |
| `up` | Seconds since boot (summed from tick differences, so it does not wrap after 12.4 days) |
| `utc` | GPS time, `-` until the receiver has one |
| `nmea ok/bad` | Valid and broken NMEA sentences so far |
| `gpsboots` / `loops` | Receiver restarts and reset-loop warnings so far |
| `sats seen/used` | Satellites with a signal (from the receiver's satellite list, its GSV every 5 s) and used in the fix |
| `fix` | `No_data!`, `Waiting_fix`, `2D_fix`, `3D_fix` |
| `hdop`, `points`, `sent` | Last HDOP, points logged and uploaded since boot |
| `wifi`, `pulses`, `mem` | WiFi state (spaces as `_`), keep-alive pulses so far, free Python heap in bytes |
| `idf F/L` / `heap` | ESP-IDF heap free and its largest block, KB, then the Python heap size, KB. TLS needs ~40 KB in one block; below 48 KB (`TLS_MIN_KB`) no POST is tried (`upload low idf`); at home, if it stays low for 5 minutes with tracks waiting, the board reboots cleanly (at most once per 30 minutes) |

**Status page:** <a id="status-page"></a>while the board is connected to
WiFi it serves a page at `http://<board ip>/` (e.g.
`http://192.168.1.60/`) and the same data as JSON at
`/status.json`: fix, satellites seen/used ("used" from the satellite list:
GGA stops at 12) and the satellite list with its age (system and number,
e.g. `GPS 7`, `Galileo 11`, `BeiDou 22`, `GLONASS 3`; signal in
dBHz, elevation, azimuth, used), HDOP, GPS time,
position (`position` to 5 decimals, `pos6` to the track line's 6; both
`null` once the last second is over 3 s old, `NO_DATA`, so a silent
receiver's last fix is not shown as live), NMEA
counts, the GPS link (`rx_bytes` read since boot,
wrapping at 2^30,
`rx_age_s` / `good_age_s`: seconds since the last byte / the last whole
NMEA sentence, `null` for none yet), the receiver model,
receiver restarts; points logged, waiting and uploaded,
quarantined `.bad` files, upload state, `upload_age_s` (seconds since the
last accepted batch, `null` for none since boot), the day history's
`days`, stay; uptime and how long the previous run lasted (e.g. `58 min
14 s`; seconds in the JSON, `prev_end` the time of its last line), boot
cause, the build (`fw`, `fw_build`), chip temperature, free Python heap
and flash,
WiFi, keep-alive pulses; and the last lines of the diagnostic log (the page
itself no longer uses them: it reads the whole log, below). The
page is a dark telemetry view in one colour world (near-black, white as
the accent; green, amber and red only for meaning). On top, a darkened
grey OSM-carto map (through the homelab Atlas tile proxy,
`atlas.crusand.org/tiles/geoapify/`, which answers pages on a
private-network address such as the board's) is
centred on the position, with a glowing dot, an accuracy ring (HDOP x 2.5 m)
and the coordinates (N/S, E/W) with
± that accuracy; without a fix it says `NO FIX`. Over its lower edge a strip
shows fix, satellites used/seen, HDOP, points waiting
for upload, WiFi signal and the share of bad NMEA sentences. The WiFi cell
is an arc icon (0–3 arcs lit) and the link quality in per cent, 2 x (RSSI +
100) capped at 0–100 (-63 dBm = 74 %), with the dBm beside it on a wide
screen; amber below -67 dBm, red below -80 dBm. Below (on a wide screen in two
columns: the sky plot and *Track* on the left, the plot growing
to fill the column next to *Systems*; signal bars and *Systems* on the right):
- a radar-style sky plot (centre = overhead, edge = horizon, north up;
  colour = system: GPS blue, Galileo green, BeiDou amber, GLONASS pink;
  filled = in the fix, hollow = not used);
- one signal bar per satellite (lines at 15/30/45 dBHz);
- *Systems* (`LOGGER SYSTEM STATUS`, laid out after a drone diagnostics
  design: uppercase labels, a segmented bar, status words):
  - *System readiness*: the share of subsystems nominal as a 10-segment bar
    (green, amber with a warning, red with an error), then a row each with
    its detail and `NOMINAL` / `WARN` / `ERROR`: `GPS RECEIVER` (model, fix,
    satellites used/seen; amber without a 3D fix, a failed setup or reset
    loops), `GPS LINK` (B/s, the last whole frame, restarts, bytes read), `UPLOAD` (state, sent, errors, `.bad`
    files), `MEMORY` (the largest IDF block, red under the 48 KB TLS needs,
    and the Python heap), `FIRMWARE` (the build date, the OTA state);
  - *Storage module*: how long the free flash holds a 1 Hz drive (`≈18
    days`, at 7 bytes a `.trb` point), free MB, points waiting (amber over
    30 min), points logged, track files waiting and `.bad` files;
  - *Uplink*: the network (and keep-alive pulses), the signal (`86%
    strong`, the dBm in its label: strong / good / weak / poor by the same
    tones as the strip) and the last accepted upload;
  - *Device*: how long it has run and since when (local time), the boot
    cause, the run before it and until when, chip temperature and clock,
    GPS time.
  Counters that are 0 are left out of the rows;
- the *Event log*: the whole diagnostic log on the board (`/diag.1`, then
  `/diag.log`: up to ~26 h), aggregated by kind rather than listed line by
  line (laid out after Sentry / Loki "patterns"). The window on top (`10:16–
  16:20 · 6 h 04 min of the diag log`), then:
  - four tiles: the share of status lines (minutes) with a 3D fix, the time
    without a fix or without data, the boots (how many were power-on) and
    the firmware updates (each counted once, `all ok` or how many failed);
  - *Satellites*: seen (green) and used (cyan) as 5-minute means over the
    whole window, a smooth line broken where the log has a gap over 10 min,
    local hours below;
  - *Events, grouped*: a row a kind, the most severe first, then by count:
    an icon by meaning (red: crash, caught bug, GPS reset loop, GPS silent,
    track write error, flash full, upload error, a rolled back update;
    amber: update check, memory reboot, clock back, WiFi failed / lost,
    upload waiting for memory, GPS setup refused, `.env`;
    cyan / green: boots, updates, WiFi back; grey: receiver start, stay
    ended, other), the count (`(+N more)` and `N more windows` included),
    a detail (e.g. `5 restarts in 3–24 s`, `krumpli · status 201 · 12 s
    each`, the silent time summed from `gps back after N s`, the updates
    `2e8bc6e → a3f2271 → 69f8c15`), a per-hour sparkline over the window and
    when it was last seen. On a phone the detail and the sparkline are left
    out. An event line has no time of its own: it takes the time of the next
    status line with GPS time (lines after the last one: the last time).
  The page reads `/diag?f=1&o=0`, then on from each reply's end, then
  `f=0` the same way (`X-Diag-Size` is the file's size; ~16 KB a reply,
  `HIST_REPLY`), parsing as it goes (only the totals are kept, not the
  lines); after that once a minute `f=0` from where it got to. When
  `/diag.log` is smaller than that, it became `/diag.1`: the page reads both
  again;
- *Track*: the day as a timeline (as in Maps Timeline), `‹ day ›` arrows
  step through today and the days the board keeps. On top the km, the
  moving time and the trips; below, a dot and a solid line per trip (start
  time, km, duration, mean km/h) and a hollow dot and a dashed line per stop
  (its length). Over each line the place where it starts or ends, from Atlas'
  Photon, as for the position (a railway station within 150 m, else city and
  street, once a place, kept while the page is open); no answer, as offline:
  the line alone. A trip still going has no end place yet. A trip is a run of
  points without a gap over 5 min that
  covers 100 m at least; a few fixes while standing belong to the stop. A
  trip clicked is drawn white on the map, which zooms to it (clicked again:
  the whole day). `GPX` saves the shown day as GPX 1.1 (built in the
  browser, a segment per gap);
- a folded satellite table (signal, elevation, azimuth, used).

The `GPS LINK` row (bytes a second between two polls, the age of the last
whole frame) turns amber after 3 s, red after 10 s; the waiting points and
the last upload turn amber (also the strip's `UPLINK`) when points have
waited over 30 min without an accepted batch. While the receiver sends nothing whole for over
10 s, a red banner above the strip says `GPS SILENT · 42 S` and whether
bytes still arrive (wiring or baud rate) or nothing does (power or the TX
wire): useful while moving the wires with the page open.

Amber or red marks what needs a look. The map tiles and the fonts (Inter,
JetBrains Mono) load from the internet when the phone is online; without
it the page shows a dark map and the system font. The page is a file on the
board, kept only gzipped (`/web.html.gz`, ~27 KB instead of the ~94 KB
`web.html`, whose comments and indentation `tools/web_min.py` takes out at build
time; copied by `deploy.sh` or an OTA) and loads once: a browser
(`Accept-Encoding: gzip`, in a bare-LF request too; a `gzip;q=0` token
refuses it) gets the file as is; a client without gzip (curl
without `--compressed`) gets it inflated on the board as it goes out
(`deflate.DeflateIO`, no `Content-Length`; a firmware without the
`deflate` module answers 406, one short of RAM for it 503). The file is
made with a 4 KB window (`WEB_GZ_WBITS = 12`), not `gzip -9`'s 32 KB:
inflating takes a buffer of the window's size, and the Python heap has
~15 KB free (the 4 KB window and its state take ~7 KB while a page goes
out, measured on the unix port). It costs ~10% (zlib level 9 on
the stripped page: 24.3 KB with `gzip -9`, 35.8 / 31.3 / 28.5 / 26.8 KB with a
0.5 / 1 / 2 / 4 KB window). A board deployed before this still has the
plain `/web.html` (its `.gz` may have the 32 KB window, too wide to
inflate here): it goes to clients without gzip until the next deploy or
OTA removes it. When you change the page by hand, make the gzip the way
`deploy.sh` does (`python3 tools/web_min.py web.html web.html.gz`) and copy
only `/web.html.gz`; then it asks
for `/status.json` every 15 s (`WEB_REFRESH`, sent in the JSON) and redraws
only the boxes; the map tiles reload only when the position moves to
another tile. When the board does not answer (asleep, out of range) the
panels fade and `LIVE` turns into `OFFLINE · 2 MIN 10 S AGO`. HDOP is
rounded to two decimals on the page; the JSON keeps the receiver's value.

**Day history on the map.** The page draws today's track (since local
midnight, the phone's time zone) on the map, with `TODAY x.xx KM` in the
header (a day stepped to in *Track*: that local day, read once, with its date
in the header; it is drawn also without a fix), and zooms out (17 down to 13) until the track fits around the
position. A gap of over 5 minutes breaks the line. Every point goes to
`/hist/YYYYMMDD.trb` (its UTC date), a binary track like the track files
(see *Binary tracks*): the header, then each append the records a flush
writes, a keyframe first, and one `#END` footer at the end that the next
append overwrites. A `.trb` point goes there once it is uploaded, when its batch is accepted (or
parked as `.bad`), not when it is written: while it waits for WiFi it is on
the flash once, in the track file. Each decoded line is written straight
to its day's file as its record again, just before the batch is
committed (a commit that then fails sends the batch again, and its points
are drawn twice over the same line). So the map shows a trip from when it is
up: at home about a minute after the track closes, on a hotspot as it
uploads; points still waiting are not drawn. (A `.trk` written by older firmware
went there when written, and is not added again.) A track
uploaded after a newer one (held back, retried) lands later in the day
file; the page puts the points in time order again. The newest 7 days are
kept (`HIST_DAYS`; a day written both as text and as `.trb` is one day). The
history yields to the tracks: under 1 MB
free (`HIST_RESERVE`) the oldest days go first (today's stays), also before a
batch would be dropped on a full flash, and nothing is added under
2 × `MIN_FREE`. An append cut short (the flash full mid-write) leaves no
footer: the next one starts with 32 `0xC0` bytes and a newline
(`HIST_SEP`), which end a record cut there (no keyframe is lost to it, no
point made up). The page asks `/hist?d=YYYYMMDD&o=<offset>` and gets the
day's text lines (the line format of a `.trk`), decoded from the `.trb`
record by record: a reply stops before the next keyframe once ~16 KB of
text is out (`HIST_REPLY`, so a long day cannot outrun `WEB_DEADLINE`; it
decodes the records twice, for the `Content-Length` first), with
`X-Hist-Next: <offset>` (where to ask from next, a keyframe) and
`X-Hist-End: <offset>` (where the day's records end now, the footer's
start). The page asks again from `X-Hist-Next` while it is short of
`X-Hist-End`, and after that only for what is new, once a minute, from
there: the next append's keyframe starts there. A day is ~6-7 bytes a point
(~50 as text): a 2 h drive at 1 Hz ~50 KB instead of ~370 KB, the 7 days
of 2 h a day ~0.35 MB instead of ~2.6 MB. Days written as text by older
firmware (`/hist/YYYYMMDD.trk`) are listed and drawn as before (sent as
they are from byte `o`, no `X-Hist-Next`: the page counts the bytes), and
a day that began as text stays text to its end (no conversion); they go
after `HIST_DAYS`.
The board sends a day's lines gathered into 1 KB sends and the JSON piece by piece (a
diagnostic line a piece, gathered into 1 KB sends), so a page view needs
~2 KB of Python heap (building the whole page used to need ~20 KB, enough
to double the Python heap into the IDF heap TLS needs). It is plain HTTP
without a password: keep it on your own network. Against DNS rebinding (a
page of another site that points its own name at the board) a request
whose `Host` is not the board's IP, `HOSTNAME` or `HOSTNAME.local` gets
`403`: open the page by IP or as `http://<HOSTNAME>.local/` (a name the
router's DNS gives it, e.g. `….lan`, is refused too). Only a `GET` is
answered; an empty or cut-off request is just closed. The request head is
read whole (up to 2 KB, within 1 s: `WEB_HEAD_WAIT`, the GPS read between
the pieces; a client dribbling its head held the loop longer than the UART
buffer lasts), the reply goes out in small pieces with the GPS
read in between; each receive or send may wait 2 s (`WEB_TIMEOUT`) and the
whole request 5 s (`WEB_DEADLINE`, which also cuts the waits short near
its end), then the client is dropped, so a slow
phone cannot hold the logger's loop (it was up to ~50 s). The free flash is
measured at most every 30 s and the waiting files are
counted at most every 10 s. For the page
the radio stays connected at home (idling connected costs ~20–30 mA);
`WEB=0` in `.env` turns both off. On the other networks the page is up
only while the radio is on anyway (something still to upload, then up to
`WIFI_IDLE`, 10 s, before it sleeps); with `WEB_AWAY=1` the radio stays
connected for it there too. `WEB_AWAY` decides how long the radio stays
on, not who is served: whenever the board is connected, on any network in
`.env`, the page answers everyone on that network, with no password.
Those networks use DHCP: find
the board's address in the hotspot or router client list (by its DHCP
hostname, `HOSTNAME`) or in the `web http://…/`
console line.

## How it works

### GPS

- **5 Hz with averaging.** At every boot the logger asks the receiver for 5
  fixes a second and for `RMC` and `GGA` every fix, `GST` once a second, `GSA` and
  `GSV` every 5 s, and no `GLL` or `VTG` (see [Hardware](#hardware)). More than 8
  `GSA` in a second (`GSA_MAX`) means the receiver did not take the rate, and it
  is set up again. The setup is not saved in the receiver.
- **Setup and retries.** It is sent again while the receiver still sends the
  sentences it turns off (`GLL`, `VTG`; `GSV` stays on), and also while the setup
  is not answered, less and less often (up to 5 minutes). Each command waits 300 ms
  for its `$OK` (`UC_ACK_WAIT`); the first one that gets no answer ends the setup,
  `$FAIL` is said and the rest goes on. 10 s after an answered setup
  (`GPS_VERIFY`) it is checked: valid seconds still at one fix each, or NMEA but no
  `GST` (without it the `GPS_MAX_STD` filter is off), count as not taken, and it
  is sent again. Only valid seconds tell: before the first fix (a setup at boot,
  indoors) the check waits for 3 of them, up to 30 minutes (`GPS_VERIFY_MAX`). It
  counts as taken only with 5 fixes a second and `GST`. One that answers every
  setup but keeps sending what it turned off gets the second retry after 5 s too,
  then 10, 20, 40 s and every minute (`GPS_IGNORED_MAX`). A start-up banner (power
  dip) sets it up again at once and starts that backoff afresh; no `GST` for 30 s
  with sentences coming (`UC_GST_LOST`) is a restart whose banner was lost.
- **Satellite list.** From `GSV`, one list a `GSV_EVERY` (5 s): the same satellite
  on two bands counts once, with its stronger signal; used from `GSA`. A receiver
  restart forgets the list and the model (`$PDTINFO`), which is asked anew.
- **Averaging.** The valid `GGA` positions, altitude and HDOP of each UTC
  second are averaged and published when the next second starts (`RMC`
  positions if a second had no `GGA`). `GST` gives the receiver's own error
  estimate of that second; a second without one keeps the last estimate for
  6 s (`GST_HOLD`): at 1 Hz `GST` comes only every 5th second, and a lost
  one used to let its seconds through unfiltered. Above `AVG_MAX_SPEED` (10 km/h, the second's top
  `RMC` speed) the position is the second's first fix instead: the mean of
  five fixes lies ~0.4 s along the track but is stamped with the second,
  6 m behind at 50 km/h and 12 m at 100 km/h. Altitude and HDOP stay
  averaged. A second with `RMC` only (its `GGA` lost) keeps the last HDOP
  for 2 s (`STALE`), so the HDOP filter still applies to it.
- **Bad sentences.** The checksum must be exactly two hex digits. The XOR
  checksum misses two flipped bits, so the numbers are checked too: a
  coordinate's degrees must be digits, its minutes 0 to under 60 (no
  exponent), at most 90° / 180° with its own hemisphere letters; a number
  field plain decimal (no exponent, `nan` or `inf`); the time's six digits
  (`HHMMSS`) and a fix's date (`DDMMYY`) digits. Anything else makes the whole sentence bad
  (counted in `nmea` good / bad), none of it used. (One such date in a track
  line became a "last time" no real time ever came after: no line and no anchor
  point until a reboot.)
- **Midnight.** Only `RMC` carries the date, and only with a fix (before one the
  UC6580's `RMC` carries a made-up date). A second that runs back past midnight
  without its own `RMC` has no date, so it is not a fix rather than a day off.
- **Clock.** The board clock (for TLS certificate dates) is set from a fix,
  from NTP until the first one.

### Points and track files

- **Sampling.** Each averaged second is sampled once. Seconds published
  together after a blocking POST, scan or setup are queued and all sampled.
  The queue holds 8 (`SECONDS_Q`). A longer block (a POST over a weak
  hotspot can take up to a minute) thins it evenly instead of dropping
  its start, so the track keeps 8 points spread over the whole block
  rather than a gap (`q_thin` in `/status.json` counts the thinned ones).
  A point is recorded when it is more than 3 m from the last one.
- **Filter.** Skipped: fresh HDOP above 5, or a GST estimate above 20 m (the
  larger of latitude and longitude, 1σ). A cold start can show a good HDOP
  while the position walks in from far away (measured: 230 m, over 65 s); GST
  catches it. The UC6580's does not (0.3 m while 4 m off), so its first minute after
  a start counts only above 15 km/h (`UC_SETTLE`). 2D fixes omit
  altitude. A fix dated before 2020 or from 2088
  on is not used at all: a receiver past a GPS week rollover reports 1999
  (read as 2099) or 2000. An NMEA fix is a GGA quality of 1–5
  (`GGA_FIX_MAX`) and an RMC mode other than `E`: an estimated (dead
  reckoning) position is no fix.
- **Binary tracks (TRKB).** Track files are `/gpx/YYYYMMDD_HHMMSS.trb`: text lines
  (`lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ`, about 46 bytes; ele and hdop empty when
  unknown), each stored as its difference from the line before, ~7 bytes a point
  on a 1 Hz drive (measured in the tests), so the flash holds about 6-7 times as
  many. Lossless: every line decodes byte for byte to its text, so the upload JSON,
  the `.bad` copy and the day history take the text. `-` starts a new segment and an
  `#END` footer ends the file. Reading stops at the footer: bytes after it are left
  over from a longer write that failed (points the logger had dropped), and the
  file counts as complete.
  - Records: a `#TRKB` header; a keyframe, `=` + the whole text line; delta
    records, a flag byte (0x80 | bit 0: a time step other than 1 s
    follows, bits 1-5: lat, lon, ele, hdop/acc, speed changed) and one
    LEB128 varint per set bit (the step in seconds; the zigzag change in
    the field's last decimal: 1e-6° for lat/lon, 0.1 m for ele); `-` and
    `#END` stay text lines (below 0x80).
  - A delta keeps the keyframe's shape: the same fields, empty ones
    included, with the same decimals. Any other line (an empty elevation
    on a 2D fix, a second written twice, a number not written as it
    prints, a change of 2^28 or more, a time from 2088 on) is a keyframe.
    Each 60 s write starts
    with one, and one follows every 64 deltas (`KEY_EVERY`), so a damaged
    record loses at most the points up to the next keyframe.
  - The `.pos` is `offset keyframe-offset`: a reader resumes by decoding
    from that keyframe (at most 64 records). A `.pos` without a valid
    keyframe, or an offset inside a record, resends the file from the
    start (Dawarich deduplicates); the boot-time point count (`left`) then
    counts it from the start too.
  - The decoder writes each line digit by digit into the read buffer (the
    date only when the day changes): a batch of 50 allocates what the
    `.trk` read does (measured on the MicroPython unix port: 19.6 KB each;
    it was 52.6 KB, right before the TLS POST).
  - A write never guesses where a `.trb` ends: if the footer is not where
    the last write left it, the batch goes on in a new file (a guessed
    boundary could split a record and decode garbage as points).
  - A record that is not a delta, a keyframe, `-` or `#...` (a damaged
    flag byte) makes the reader drop the deltas up to the next keyframe:
    reading on from inside a record never decodes as points. Decoded
    values are bounded (9 digits, a delta under 2^28), so a damaged stream
    cannot outgrow the decode buffer. A `.pos` keyframe of `-1` means none
    was read yet (a damaged first keyframe): the reader goes on from the
    offset and drops deltas up to the next keyframe.
  - Known blind spot (the format has no sync marker): in a run of `0xFF`
    bytes (an erased block) each byte is a 1-byte damaged record until the
    last few, whose varints run into whatever follows; if a keyframe
    follows within 31 bytes, its `=` and first bytes are eaten as varints
    and that keyframe is lost with its deltas up to the next one. So flash
    damage costs up to two keyframe spans, not one. The board's reader and
    `tools/trk2gpx.py` take the same records (the upload and the GPX
    agree); a fix would change the format, the `.pos` and both readers.
- **Text tracks.** A `.trk` holds the same lines as text (a `#TRK1` header). None
  is written now, but they are read and uploaded: a `.trb.bad` is text, and it goes
  up again renamed `.trk`. A `#TRK2` file (`lat,lon,ele,acc,speed,YYYYMMDDTHHMMSSZ`,
  an older firmware on another receiver) is read too: readers (upload, point count,
  `.bad` copies, `trk2gpx.py`) tell the two by the line itself (4 or 5 commas). The
  departure ring keeps the fields of a line, so a rebuilt way out is byte for byte
  the line that would have been written.
- **Power-loss safe.** Points are written in batches every 60 s. Each write
  replaces the footer with the new points plus the footer; LittleFS commits
  on close, so a power cut leaves the old or the new version. A line cut off
  by a power loss is skipped on read.
- **Segments and files.** A short fix loss starts a new segment (the `-`
  goes before the first point after it; every loss gets its own, also while
  the points before it are still held in RAM or writes keep failing: losses
  whose points were dropped from a full batch share one). A loss over 5
  minutes starts
  a new file: the old track is closed so it uploads. Only while flash writes
  keep failing does the new trip go on in the same file, after a `-`. Lines
  a stay still being proven held from before the loss move on to the new
  file, and the trip follows them after a `-`. On WiFi a file open for 5
  minutes is closed so it can upload; driving offline stays one file, and
  parking does not open empty files.
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
  uploads. A point due while the device is over 50 m from the anchor waits until
  it has been back within 50 m for 30 s, so a car leaving past the anchor writes
  none. Drift out there delays a point by 5 minutes at most.
- **At home** (a home network): no anchor points. If the home WiFi drops,
  anchor points start; it is still a stay.
- **Arrival.** Entering a stay closes the current track on any network, so
  the arrival uploads during the stay.
- **Leaving.** 3 consecutive samples beyond 150 m that each gain at least
  2 m (a vehicle), or 10 samples out there that gain at least 5 m in all (a
  walk). A multipath jump that freezes 200 m away does neither, however long
  it lasts, and an excursion that turns back does not count. The way out is
  written from near the anchor, from the last 90 samples, in time order
  (kept in preallocated arrays, 28 bytes a sample: as line tuples they took
  ~20 kB of the heap and broke uploads). Each line is rebuilt exactly as it
  would have been written then, the board's single-precision rounding
  included.
- **Leaving after a fix gap** (over 30 s; a metro or a garage, then sitting
  still elsewhere): beyond 150 m the stay ends without waiting for the fixes
  to recede once the device clearly sits elsewhere: 10 fixes within 50 m of
  the first of them over 60 s (`STILL_GAP_STAY`), or one fix more than 1 km
  out (`STILL_GAP_JUMP`, beyond any multipath jump). Fixes coming closer do
  not count (a cold start converging walks in), and a receding run leaves
  as usual. The way out starts at the first fix after the gap. Before,
  nothing receded, so the stay went on and nothing was written at the new
  place; and when one fix beyond 300 m or 3 beyond 150 m ended it, a single
  outlier, a jump frozen for 3 s or a cold start walking in wrote a spike.
  Leaving home keeps its check below.
- **Leaving without a fix** (away from home). The first 3 WiFi scans of the
  stay (keep-alive pulses, roam and connect rounds) note the access points
  around; later scans that see one of them add the others seen with it (a
  walk into another wing), the 16 most recently seen kept. Our own networks
  and locally administered BSSIDs (phone hotspots, a train's WiFi) are left
  out, since they travel along. When none of them shows up in 6 scans in a
  row over at least 60 s, the stay ends (`stay left, wifi`): no more anchor
  points at the old place. A way out is written only if the last fix was
  more than 50 m from the anchor; otherwise the recent fixes are the stay's
  drift. Before, a train ride home without a fix kept
  writing the old anchor every 5 minutes until home. Where no access point
  is around while learning, or with no scans (keep-alive `gpio`/`radio` or
  off, the radio connected), only a fix ends the stay, or:
- **Home without a fix.** An away stay ends once the home WiFi is up and no
  usable fix came for 30 s (`stay end: home without a fix`), counted from
  the fix that began the stay at the earliest: the home WiFi connecting
  just after arriving no longer ended a stay proven seconds before. Left
  running, it stamped the old place next morning and swallowed a trip that
  did not recede from it.
- **Drift hold.** What a stay still being proven holds in RAM is not the
  previous window when the open one is already 150 m beyond it (a drive),
  nor an open window that has spread out (its centroid over 100 m from both
  its first and its latest fix: a drive, which used to wait up to two
  minutes in RAM), and it is written 10 minutes after the fix was lost
  (`HOLD_RELEASE`), or after no usable fix for as long (a receiver that
  keeps a fix indoors at 30–100 m kept it in RAM for hours, and a reboot
  dropped it): the end of a trip into a garage or a house no longer waits
  in RAM only. A window that spread out on its own (its centroid over 75 m
  from both its first and its last fix: an out-and-back turn at driving or
  cycling speed) proves no stay, so a U-turn no longer drops part of the
  trip. A walk turning back within ~80 m still can: indoor drift looks the
  same. A fix back after 5 minutes more than 150 m away writes
  what was held to the old track before it closes.
- **Leaving home** is accepted only once the home network is out of range
  (below −75 dBm: the RSSI when connected, else a ~2 s scan). While it is in
  range the departure is a GPS jump (measured: a 194 m jump at a desk) and
  further departures wait 60 s.
- **After a fix gap** (over 30 s): fixes more than 150 m from the open window
  restart both windows; more than 50 m restart only the open one, so two
  places are never mixed while what it held stays held. The previous window
  is dropped on its own when the fix is more than 150 m from it. Each gap is
  judged once: a fix back after 5 minutes judged it before the old track
  closed and again right after, and the second verdict released into the
  new file the old place's drift the first had held over.
- A walk (1.4 m/s) is never a stay: its centroid moves on by more than 50 m a
  minute.

### WiFi

**Choosing a network:**

1. Scan once, keep the strongest BSSID per SSID.
2. Try a home AP at ≥ −75 dBm first, then other visible known APs, strongest
   first, then configured networks the scan missed, in config order: hidden
   ones every round, the others in the first round (at boot, and again after
   a link loss, a wake from sleep or a new stay) and every 4th
   (`UNLISTED_EVERY`), since each costs a 12 s try with the radio on. A
   hidden home is preferred when the scan shows a strong empty-SSID AP. If
   the scan fails, every network is tried in config order.
3. Each try is polled from the main loop (~12 s deadline), so GPS sampling
   goes on. An AP seen at ≥ −75 dBm (a hidden home: one of its learned
   BSSIDs at ≥ −75 dBm, not a stranger's blank AP) gets one immediate retry:
   the first association after a cold boot can fail after ~7.5 s with status 202
   although the password is right (measured on the earlier ESP32-C6, not confirmed
   on the S3), and the retry connects within ~1 s. A 202 on an AP the scan saw is retried at any
   RSSI: the board's home AP reads −65 to −75 dBm, and a reading just under
   −75 skipped the retry, so the reconnect after a drop failed round after
   round.
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
  between; with nothing to upload it sleeps. With `WEB` on and no keep-alive
  scan that could see home come back (`gpio`, `radio` or off, or a hidden
  home whose BSSIDs are not learned yet), it searches on with that backoff
  for 30 minutes (`WEB_SEARCH`) after the link went down, so the page comes
  back after a router restart (or a router booting slower than the board)
  instead of an hour later.
- Without `GEO_URL`/`API_KEY` nothing is ever sent, so the radio sleeps once
  idle as if all were uploaded, and waiting tracks do not make it search
  away from home.
- While uploads back off (server down, 5xx, 401) the radio stays on only
  for a short backoff. Once the next attempt is `WIFI_OFF_WAIT` (60 s) or
  more away it is turned off (`wifi off until upload retry`) and a round
  reconnects when the attempt is due. A home stay stays a home stay
  meanwhile.

**Power bank keep-alive:** with the radio asleep the board draws less than
many power banks count as a load, and they switch off. The 20 Ah bank used
here switched off after 1 hour; with a scan every 15 s, after 3 hours, so it
averages the load. Every 5 s (`KEEPALIVE`) one pulse goes out, not while the
radio is on anyway, and also without WiFi set up at all (no `.env`, no SSID): the
radio is turned on for one scan (~2 s at ~100 mA, about +40 mA on average) and off
again. It blocks the main loop for the scan (GPS seconds are queued, not lost). The
only pulse measured to keep the 20 Ah bank on longer; the same scan watches a stay's
access points and the home network.

With the status page on (`WEB`), a keep-alive scan that sees a home network
(a hidden home: one of its learned BSSIDs) at ≥ −85 dBm (`SEEN_MIN_RSSI`:
a home it connects to at −78 dBm used to wait an hour) while the radio
sleeps or found nothing starts a round at once
(`wifi home in range`): after a router restart the page used to stay down
for an hour. At most once a minute, doubling while those rounds fail. A
link that dropped somewhere the scan cannot vouch for (a hotspot with
`WEB_AWAY`, a home that dropped below −85 dBm) searches with the backoff
for `WEB_SEARCH` instead of waiting for the scan; a round that fails after
the radio slept on purpose does not count as a drop.

**CPU clock:** the main loop runs at 80 MHz (`CPU_MHZ`). A POST runs at
160 MHz (`CPU_MHZ_UP`) so the TLS handshake stays quick, then the clock
drops back. The UART runs off the fixed APB clock, so its baud rate is unaffected.
`CPU_MHZ=0` leaves the firmware default. A failed clock change is tried
again (3 tries); a clock refused every time is left alone for 30 minutes
(`FREQ_RETRY`), then tried again, so one passing failure of the drop-back
after a POST does not keep the board at 160 MHz.

**Roaming home:** on a non-home network, when nearly stopped (speed below
5 km/h, or in a stay), the logger scans for home every 5 minutes and switches
to a home AP at ≥ −75 dBm. It backs off to 60 minutes only after a failed
switch to a visible home; stopping after a drive resets that only when a
visible home is in view. A hidden home is tried with `bssid=None` (matched by
SSID) when a strong empty-SSID AP is seen, learned or not (another node of a
mesh may not be learned yet); a neighbour's hidden AP can cause a false
attempt, and switching to a hidden home can be slower.

**Hidden home:** while connected to it (and only then) the logger learns
the BSSID it is on: one scan, the record the IDF names with the hidden SSID
if it shows one, else the only empty-SSID AP on the link's channel within
8 dB of its RSSI. Connected through another node, that one is learned too.
Up to 4 are kept, in `/home_aps` (hex, one per line, rewritten only when the
list changes, never into the flash reserve), so they survive a reboot. A
learned BSSID that is not the link for 5 hidden-home connects
(`HOME_AP_MISS`), or that the scan shows with a visible SSID, is dropped,
so a wrong one does not stick. Once some are learned, only those count for
"home in range" (stay departures), the keep-alive wake and the immediate
retry, so a neighbour's hidden AP no longer cancels a departure; any strong
empty-SSID AP still counts for the boot preference and roaming (weaker
evidence: a node not learned yet). Until then (or with two candidates in
that scan) any strong empty-SSID AP counts, as before. A failed hidden-home
try skips that AP for an hour (never one of the learned BSSIDs; cleared when
the hidden home connects), and failed hidden-home roams back off on their
own (10 minutes doubling to 60) without slowing the check for a visible
home.

### Upload

- Finished files go oldest first, 50 points per POST, to
  `<GEO_URL>/api/v1/overland/batches`. The file being recorded is never sent.
- **Fields.** Each point is a GeoJSON feature with `timestamp`, `altitude`,
  `horizontal_accuracy` and `device_id` properties; a TRK2 line of an older track
  also has `speed` (m/s). `horizontal_accuracy` is HDOP x 5 m (`HDOP_M`), on a TRK2
  line the receiver's estimate it holds. Dawarich's Overland
  importer (`app/services/overland/params.rb`) stores `horizontal_accuracy`
  as `accuracy`, `speed` as `velocity`, `altitude` and `vertical_accuracy`;
  a `course` would land only in the raw data, so it is not stored or sent
  (neither is the vertical accuracy, to keep the lines short). A TRK1
  body is the same as before.
- Progress is a `<file>.pos` byte offset, written atomically. It advances
  only after HTTP 2xx or a durable copy to `<file>.bad`, so an interrupted
  POST is resent (Dawarich deduplicates). A 2xx with an HTML body
  (`Content-Type: text/html`) is a captive portal or proxy answering, not
  Dawarich: it counts as a failed POST (`upload err html … (captive
  portal?)` in the diagnostic log). A file that cannot be deleted after its
  last batch is parked as `.err` after 3 tries.
- 400 / 413 / 422 quarantine the batch to `<file>.bad`. Three in a row
  without a success count as a server problem: backoff, no more quarantine
  (rejections of a `.bad` itself do not count here).
  A full `.bad` (64 kB) pauses that file (`BAD FULL`) instead of dropping
  points; pull or delete it, then reset. `BAD FULL` lasts while that `.bad`
  is on flash and its track waits; a copy that fails for another reason
  (the flash reserve, an I/O error) is `fs err` and is tried again.
  Points moved to a `.bad` leave `left` once (a `.bad` is not counted).
- Other errors back off from 15 s up to 30 minutes (`Retry-After` is
  honoured); 401 waits 30 minutes. Between POSTs it pauses 5 s at home, 30 s
  elsewhere. Flash and memory errors back off too, and with every waiting
  track held back (state `held`) the radio sleeps until the first is due.
- An unreadable track (`OSError` on read) is held back so the newer ones
  still go; after three tries it is parked as `<file>.err`. A `.pos` that
  cannot be written is a flash error (`fs err`, retried), not a read error.
- **Reading:** track lines are read with `readinto` into one 512-byte
  buffer (`LINE_MAX`) and a `.trk` point is parsed there and written
  straight into the JSON buffer, so a 50-point batch makes only small,
  short-lived objects (the `float()` checks), no line copies. A `.trb`
  delta is decoded to its text line in 96 bytes past the read buffer, then
  parsed the same way.
- **Own HTTP client:** a minimal HTTP/1.0 POST over the `tls` module. The
  body goes out in 1 KB pieces and the wait for the answer (up to 30 s,
  `HTTP_WAIT`) polls, both reading the UART meanwhile, so a slow server
  costs no NMEA (the 4 KB UART buffer holds ~5 s). The upload line on the
  console ends with the POST time. Answer lines are read 256 bytes at a
  time (`HTTP_LINE`); once the status line is in, an error while reading
  the headers (reset, timeout, the guard closing the socket) keeps the
  status, so an accepted batch is not sent twice.
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
  warns that everything goes in the clear. The `SSLContext` (and the
  certificate) is made once at boot, before the WiFi starts (made at the
  first POST it could grow the Python heap and keep that area), and reused.
  Only a missing `/ca.der` means no verification: one that cannot be read,
  is empty or is not a valid certificate stops the POSTs (state `ca err`,
  `upload ca err …` in the diagnostic log, retried every 30 s), never a
  silent fall back. A `/ca.der` added, removed or replaced by one of another
  size is picked up at the next POST; one of the same size takes a reboot.
  mbedTLS checks the certificate's dates and the board clock starts at
  2000-01-01 at every boot, so before a POST the logger sets it from a
  valid GPS fix (a date in 2020–2087): once a boot (`clock set …` in the
  diagnostic log), again only when it drifted more than 5 s
  (`RTC_DRIFT`). With no fix since the boot (indoors, at home) the NTP
  answer sets it meanwhile (`clock set … ntp`; before, verified POSTs
  failed until the first fix), and a fix sets it again when it differs.
- **TLS memory:** five TLS allocation failures in a row at home
  (`TLS_REBOOT_N`; any other error ends the run) restart the board cleanly
  (`reboot tls memory` in the diagnostic log): the IDF heap stays
  fragmented otherwise. Counted: `MemoryError`, `ENOMEM` and the mbedTLS
  `*_ALLOC_FAILED` codes (SSL, X509, ECP, PK, …, also combined with a
  low-level code); `OSError(16)`, which is how `MPI_ALLOC_FAILED` arrives
  but also `EBUSY`, only from the TLS handshake; `ValueError("invalid
  cert")`, which is how a failed allocation parsing `/ca.der` arrives, only
  while the `SSLContext` is made with the IDF heap's largest block under
  `TLS_MIN_KB` (else the file is bad: `ca err`).
- **Low IDF heap:** below `TLS_MIN_KB` no POST over https is tried (a
  plain `http://` `GEO_URL` needs no TLS block: neither this nor the
  Python heap margin below applies to it); the upload runs
  `gc.collect()` and checks again every 30 s at home (`LOWIDF_RETRY`). Only
  if it stays low for 5 minutes (`LOWIDF_WAIT`) with tracks waiting does
  the board reboot (`reboot low idf` in the diagnostic log), and at most
  once per 30 minutes (`LOWIDF_REBOOT_GAP`: the reboot leaves
  `/lowidf`, which the next boot takes). Away it only waits.
- **Python heap vs IDF heap.** MicroPython 1.29 starts the Python heap
  small and, when an allocation does not fit even after a collection,
  adds an area at least as big as the whole heap so far, taken from the
  largest free IDF block. Measured overnight on 2026-10-31: the heap sat at
  124 KB with 3–8 KB free; each doubling to 248 KB took the 124 KB IDF block
  (IDF free 159 → 31 KB, largest 148 → 16 KB) and TLS could not start until
  the area emptied and was freed again. So the page streams its replies,
  and before giving up on TLS (`TLS_MIN_KB`) the upload runs `gc.collect()`,
  which returns an empty grown area, and measures again. It also wants
  12 KB of free Python heap then, 8 KB on the frozen firmware
  (`TLS_MIN_PY_KB`; else `low mem` and a retry every 30 s,
  `LOWIDF_RETRY`; at home, low for 5 minutes, the same clean reboot as a
  low IDF heap, `reboot low mem` in the diagnostic log, within the same
  once per 30 minutes): the socket and TLS objects must fit without growing the
  heap. A POST itself takes a few KB (sockets, header lines; the
  `SSLContext`, ~3 KB, is made at boot); frozen, no module bytecode
  fragments the heap, so 8 KB leaves room.
  `gc.mem_free()` counts that largest IDF block (in bytes) as free Python
  heap, so the diag, the page and the upload line subtract it to the byte
  (`mem`, `heap`), and `heap grew in <step>` names the upload step during
  which the heap grew.

### Watchdog and recovery

- A 120 s hardware watchdog is fed in the main loop and inside long reads and
  POSTs.
- An uncaught exception is printed, the batch is flushed (without the drift
  held for a stay), and the board resets after ~1 s.
- The [boot loop guard](#using-it) stops repeated crashes (and retries
  every 30 minutes); a failing display goes dark instead of crashing.

## Configuration

Constants at the top of `gps_logger.py`; most never need changing.

**GPS and points**

| Constant | Default | Meaning |
|----------|---------|---------|
| `GPS_HZ` | 5 | Fixes a second asked from the receiver, averaged into one point |
| `GPS_LOOP_WINDOW` | 10 s | How often the receiver's stamps are looked at (`_stamps_old`) |
| `UC_SETTLE` / `UC_SETTLE_KMH` | 60 s / 15 km/h | How long after a start a fix counts only at this speed |
| `GPS_FLAP_WINDOW` / `GPS_FLAP_RESTARTS` | 10 min / 5 | Full restarts within this window, NMEA or not = `gps reset loop?` |
| `GPS_STORM_WAIT` / `GPS_STORM_QUIET` | 30 s / 10 s | Least time between setups while the receiver keeps restarting; quiet this long after a restart (in a storm twice its last gap, up to `GPS_STORM_WAIT`) and it is set up within `GPS_RETRY` |
| `AVG_MAX_SPEED` | 10 km/h | Faster than this a second's point is its first fix, not the mean |
| `UC_ACK_WAIT` / `UC_GST_LOST` | 300 ms / 30 s | Wait for a command's `$OK`; no `GST` this long with sentences coming: set up again |
| `GPS_RETRY` / `GPS_RETRY_MAX` | 5 s / 5 min | Setup sent again while turned-off sentences keep coming (doubling while it does not answer) |
| `GPS_IGNORED_MAX` / `GPS_VERIFY` | 60 s / 10 s | Cap on the doubling wait for a receiver that answers but ignores the setup; when an answered setup is checked (still 1 fix/s or no `GST` = not taken) |
| `GPS_VERIFY_MAX` / `GPS_NOT_TAKEN_SAY` | 30 min / 10 min | Longest the check waits for 3 valid seconds; least time between `gps setup not taken` lines in the diagnostic log |
| `SECONDS_Q` | 8 | Averaged seconds queued for the logger |
| `MAX_HDOP` | 5 | Skip samples with a worse fresh HDOP |
| `GGA_FIX_MAX` | 5 | Highest GGA quality that is a fix (6, estimated, is not) |
| `GPS_MAX_STD` / `GST_HOLD` | 20 m / 6 s | Skip samples with a worse GST error estimate; how long the last estimate covers seconds without one |
| `HDOP_M` | 5 m | Metres per HDOP unit: `horizontal_accuracy` of a TRK1 line |
| `MIN_MOVE` | 1 m | Distance from the last point before a new one (`.env` `MIN_MOVE`) |
| `SAMPLE` | 1 s | Sampling interval when no averaged second is available (tests) |

**Track files**

| Constant | Default | Meaning |
|----------|---------|---------|
| `FLUSH` | 60 s | Interval between flash writes |
| `NEW_FILE_GAP` | 300 s | Fix loss that starts a new file; also the max age of an open file on WiFi |
| `MIN_FREE` | 32 kB | Flash reserve |
| `HIST_RESERVE` | 1 MB | Free flash the day history leaves to the tracks (older days go first) |
| `MAX_BATCH` | 240 | Points kept in RAM while flash writes fail |
| `WFAIL_SWITCH` | 3 | Failed writes in a row to a track file (a bad block) before it is left (cut back to its last good footer, so a partly landed batch is not sent twice) and the points go on in a new file |
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
| `STILL_GAP_STAY` / `STILL_GAP_JUMP` | 60 s / 1 km | After a gap, beyond `STILL_EXIT`: how long `STILL_MIN_FIXES` fixes within `STILL_RADIUS` must span, or one fix's distance, to end the stay |
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
| `WIFI_OFF_WAIT` | 60 s | An upload backoff at least this long turns the radio off until it is due |
| `ROAM_CHECK` / `ROAM_BACKOFF_MAX` | 5 min / 60 min | Home-roam scans on another network, and their backoff cap |
| `ROAM_MIN_RSSI` | −75 dBm | Minimum home RSSI for a switch (also boot preference and home range) |
| `SEEN_MIN_RSSI` | −85 dBm | With `WEB`, a home a keep-alive scan sees at least this strong starts a round while the radio sleeps |
| `UNLISTED_EVERY` | 4 | A network a good scan did not list is tried every Nth round (hidden ones always) |
| `WEB_SEARCH` | 30 min | With `WEB` and no keep-alive scan that can see home (none, or a hidden home not learned yet), failed rounds keep searching this long before sleeping |
| `BLANK_BAD_HOLD` / `HOME_AP_MAX` / `LEARN_RSSI` | 1 h / 4 / 8 dB | Hidden home: a blank AP whose try failed is skipped; learned BSSIDs kept; RSSI match for learning |
| `HOME_AP_MISS` / `HOME_APS_FILE` | 5 / `/home_aps` | Hidden-home connects through other BSSIDs before an unseen learned one is dropped; the file that keeps them across reboots |

**Upload**

| Constant | Default | Meaning |
|----------|---------|---------|
| `UPLOAD_BATCH` | 50 | Points per POST |
| `UPLOAD_GAP` / `UPLOAD_GAP_AWAY` | 5 s / 30 s | Pause between POSTs at home / elsewhere |
| `BACKOFF_START` / `BACKOFF_MAX` | 15 s / 30 min | Upload backoff (and `Retry-After` cap) |
| `AUTH_BACKOFF` | 30 min | Wait after HTTP 401 |
| `HTTP_TIMEOUT` | 15 s | Per-socket HTTP timeout (connect, TLS) |
| `HTTP_WAIT` / `HTTP_CHUNK` | 30 s / 1 KB | Wait for the server's answer, body piece; the UART is read between |
| `HTTP_LINE` / `HTTP_LINES` | 256 B / 64 | Bytes per read of an answer line; reads of the answer's head at most |
| `HTTP_MAX` | 60 s | Wall clock for one POST: the guard timer feeds the WDT, then closes the socket |
| `CA_FILE` | `/ca.der` | DER root certificate; if present, the server is verified (unreadable: no POST) |
| `RTC_DRIFT` | 5 s | Board clock vs GPS time before the clock is set again (certificate dates) |
| `TLS_REBOOT_N` | 5 | TLS allocation failures in a row (at home) before a clean reboot |
| `TLS_MIN_KB` | 48 KB | Largest IDF heap block a POST needs; below it no try (`gc.collect()` and wait) |
| `TLS_MIN_PY_KB` | 12 KB (frozen: 8 KB) | Free Python heap a POST over https needs after `gc.collect()`; below it the POST waits `LOWIDF_RETRY`, at home a clean reboot after `LOWIDF_WAIT` |
| `LOWIDF_RETRY` / `LOWIDF_WAIT` | 30 s / 5 min | Checks while the IDF block is low (at home); how long it may stay low with tracks waiting before a clean reboot |
| `LOWIDF_REBOOT_GAP` / `LOWIDF_MARK` | 30 min / `/lowidf` | At most one low-idf or low-mem reboot this often; the file that tells the next boot |
| `READ_ERR_MAX` | 3 | Read errors before a track is parked as `.err`; an unreadable `.pos` holds its track this many tries, then it is resent from 0 |
| `MAX_BAD_BYTES` | 64 kB | Cap on a `.bad` file |
| `MAX_GLOBAL_PAYLOAD` | 3 | Payload errors in a row that stop quarantining |

**System**

| Constant | Default | Meaning |
|----------|---------|---------|
| `WDT_TIMEOUT` | 120 s | Hardware watchdog |
| `CPU_MHZ` / `CPU_MHZ_UPLOAD` | 80 / 160 | CPU clock in the loop and during a POST (`.env` `CPU_MHZ`, `CPU_MHZ_UP`); 0 = firmware default |
| `FREQ_TRIES` / `FREQ_RETRY` | 3 / 30 min | Tries per clock change; how long a refused clock is left alone before it is tried again |
| `KEEPALIVE_MAX` | 10 min | How long a Ctrl-C timer feeds the watchdog |
| `BOOT_OK_MS` | 60 s | Healthy loop time that clears the boot-fail counter |
| `STATUS` | 10 min | Console `status` line interval |
| `DIAG` / `DIAG_MAX` | 60 s / 160 KB | Diagnostic log interval (`.env` `DIAG`) and size before `/diag.log` becomes `/diag.1` (the two: ~26 h of status lines, up to 320 KB of flash) |
| `WEB` / `WEB_PORT` | on / 80 | Status page (`.env` `WEB`), its port |
| `WEB_TIMEOUT` / `WEB_DEADLINE` / `WEB_HEAD_WAIT` | 2 s / 5 s / 1 s | How long one receive or send may wait; how long a whole request may take; how long its head may take |
| `OTA_FIRST` / `OTA_CHECK` / `OTA_RETRY` / `OTA_VALID_AFTER` / `OTA_HEALTH` / `OTA_MAX` | 2 min / 10 min / 1 h / 2 min / 10 min / 5 min | First update check after boot, then between checks; before a build whose download failed (or that was rolled back as a try) is downloaded again; how long a new firmware runs with WiFi connected before it is valid; how long it may take to get WiFi and read the manifest; wall clock for one download |
| `OTA_PROBE` / `OTA_TRIES_MAX` / `OTA_STUCK` | 1 min / 2 / 24 h | How often a pending firmware reads the manifest until it did; rollbacks that are not a crash (power-on, no WiFi / manifest, a reset from outside, a panic) before a build is bad; how long without a successful upload before waiting points stop holding the update check back |
| `WEB_PAGE_GZ` / `WEB_GZ_WBITS` / `WEB_PAGE` / `WEB_REFRESH` | `/web.html.gz` / 12 / `/web.html` / 15 s | The page on the board, kept only gzipped (as is to clients accepting gzip, inflated for the others); its window (2^12 = 4 KB of heap to inflate; `deploy.sh` and `--publish` use it); an older board's plain page, served while it is there; how often the page polls `/status.json` |
| `KEY_EVERY` | 64 | Delta records between two keyframes of a `.trb` |
| `HIST_DIR` / `HIST_DAYS` / `HIST_REPLY` | `/hist` / 7 / 16 KB | The day history for the page's map (a `.trb` per UTC day), days kept, text bytes about per `/hist` reply (and bytes at most per `/diag` reply) |
| `WEB_AWAY` | off | Radio kept on for the page on non-home networks too (`.env` `WEB_AWAY`) |
| `SV_POLL` | 60 s | A satellite list younger than two of these gives the used count (else GGA) |
| `GSA_EVERY` / `GSA_MAX` | 5 s / 8 | GSA round interval; more GSA a second means the rate was not taken |
| `SEEN_WAIT` | 60 s | Least time between home-seen wakes |
| `WEB_REQ_MAX` / `WEB_CHUNK` | 2 KB / 1 KB | Request head read by the status page; reply bytes per send |
| `DEVICE_ID` / `HOSTNAME` | `Dawarich-GPS-tracker` | Overland id and DHCP hostname (set in `.env`) |

## Development

GitHub Actions (`.github/workflows/ci.yml`) runs on pushes to `master` and on
every pull request:

- `python3 tests/test_gps_logger.py`: the CPython harness with mocked
  `machine`/`network`/UART (exits 1 on any FAIL), plus
  `tests/mpy_unix_check.py` and `tests/mpy_unix_logger.py` (imports
  `gps_logger.py` with stub `machine`/`network` modules and runs its pure
  paths) on a cached MicroPython 1.29 unix build, and the smoke again on a
  single-precision build (`MICROPY_FLOAT_IMPL_FLOAT`, like the board: the
  departure ring must rebuild the lines `_sample_line` writes); the smoke
  also runs under CPython. It also runs:
  - `tests/web_loopback.py` on the unix build and CPython: the status page
    over real loopback sockets (a normal request, a foreign `Host`, an
    empty, an oversized, a stalled and a trickling request, a client
    resetting mid-reply, a reply past the request deadline, the gzipped
    page for a client that accepts it, and for one that does not the page
    inflated from the `.gz` alone, byte for byte, its live heap under
    10 KB on the unix build);
  - `node tests/web_page_test.js`: `web.html`'s view on generated
    `status.json`, and the whole page script (`tick`, `paint`, `placeMap`,
    `stale`) against a stub document, and the same on the page as
    `tools/web_min.py` strips it (it must print the same). With `CI=true`
    (GitHub Actions) a missing `node` is a FAIL, not a SKIP;
  - `deploy.sh` against a fake board (a stand-in `mpremote` that runs the
    board-side code under CPython): copy, check and rename, too little
    flash, a cut copy, frozen firmware (stale, `FORCE`, `FORCE_MPY`, no
    clear answer), an OTA pending (`FORCE_PENDING`);
  - `tools/build_firmware.sh` with a fake ESP-IDF, MicroPython tree,
    `esptool.py`, `mpremote`, `scp` and `ssh`: the `gps_frozen` marker,
    `--flash` touching nothing on the board before a successful write,
    `--migrate`'s flash size check and private backup, `--publish` moving each
    file in from a `.tmp` and refusing a build not newer than the published
    one.
  The page checks run on Node 24 (`actions/setup-node`, pinned).
- `mpy-cross` 1.29 for `xtensawin` on every frozen module (`gps_logger.py`,
  `gps_boot.py`, `gps_tft.py`, `gps_tft_font.py`) and `main.py`. The firmware
  itself (ESP-IDF, `boards/GPS_LOGGER_S3`) is not built in CI: a board or
  freeze-manifest breakage shows at the next local `tools/build_firmware.sh`.
- `pyflakes` on `gps_logger.py`, `gps_tft.py`, `main.py`, `tools/trk2gpx.py`,
  `tools/web_min.py`, `tests/mpy_unix_logger.py` and `tests/web_loopback.py`.
- `shellcheck deploy.sh tools/build_firmware.sh`.

The same locally:

```bash
.venv/bin/pip install pyflakes "mpy-cross==1.29.*"
.venv/bin/python -m pyflakes gps_logger.py gps_tft.py main.py tools/trk2gpx.py tools/web_min.py tests/mpy_unix_logger.py tests/web_loopback.py
shellcheck deploy.sh tools/build_firmware.sh
for m in gps_logger gps_boot gps_tft gps_tft_font main; do .venv/bin/python -m mpy_cross -O3 -march=xtensawin -o /tmp/$m.mpy $m.py; done
MICROPYTHON=/path/to/micropython MICROPYTHON_F32=/path/to/micropython-f32 \
    python3 tests/test_gps_logger.py
```

The single-precision unix build: `make -C ports/unix
BUILD=build-f32 PROG=micropython-f32
CFLAGS_EXTRA=-DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_FLOAT` in a
MicroPython v1.29.0 tree.

`tests/device_bench.py` is an optional on-device timing probe; nothing under
`tests/` is copied to the board.
