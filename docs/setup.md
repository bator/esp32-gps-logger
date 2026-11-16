# Setup

[← README](../README.md)

- [1. Host tools](#1-host-tools)
- [2. `.env`](#2-env)
- [3. Deploy](#3-deploy)
- [4. Custom firmware (recommended)](#4-custom-firmware-recommended)
- [5. Firmware updates over WiFi (OTA)](#5-firmware-updates-over-wifi-ota)

## 1. Host tools

Linux with Python 3:

```bash
python3 -m venv .venv
.venv/bin/pip install mpremote "mpy-cross==1.29.*"
```

The upload has its own small HTTP client, so `requests` is not needed.

## 2. `.env`

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
| `SSID_HIDDEN`, `SSID2_HIDDEN`, `SSID3_HIDDEN` | `1`: hidden SSID, tried although the scan never lists it by name: in every round whose scan shows an AP with an empty SSID (a hidden network still beacons), and in the first round after a link loss, a wake or an arrival and every 4th round otherwise. Set it for every hidden network: without it a hidden SSID is tried only in those first and 4th rounds (up to ~40 minutes apart). |
| `IP`, `GW`, `MASK`, `DNS` (and `IP2`…, `IP3`…) | Static IP per network. `GW` is required; `MASK` defaults to `255.255.255.0`, `DNS` to the gateway. An invalid entry falls back to DHCP with a line in the [diagnostic log](usage.md#diagnostic-log) (`env IP needs a valid IP and GW, using DHCP`). |
| `GEO_URL` | Dawarich origin. Use `https://`; see [Upload](internals.md#upload). |
| `API_KEY` | Dawarich API key, sent as `Authorization: Bearer <key>` |
| `DEVICE_ID` | Overland tracker id, `[A-Za-z0-9._-]{1,32}` |
| `HOSTNAME` | DHCP hostname, `[A-Za-z0-9-]{1,32}` (defaults to `DEVICE_ID`) |
| `KEEPALIVE` | Seconds between power bank keep-alive pulses, 5–600; `0` turns it off (e.g. on a car charger). Default 5. |
| `CPU_MHZ`, `CPU_MHZ_UP` | CPU clock in the main loop and during a POST: 20/40/80/160/240; `0` leaves the firmware default. Defaults 80 and 160. WiFi needs 80 or more: 20/40 are raised to 80 whenever the radio is used (a network set, or `KEEPALIVE` on). |
| `MIN_MOVE` | Metres from the last point before a new one, 0–50 (decimals allowed). Default 1: on foot about one point a second; 3 gave one every ~3 s. Read at boot. |
| `OTA_URL` / `OTA_KEY` | Firmware updates over WiFi: the manifest's URL (http(s), on the LAN) and its HMAC key, the one in `OTA_KEY_FILE` on the build machine. Both or no updates. See [OTA](#5-firmware-updates-over-wifi-ota). |
| `NTP_SERVER` | Where the display's clock and the route's day come from until the first fix. Default `pool.ntp.org`; empty: no NTP, `--:--` until a fix. A fix always wins; NTP never reaches a track, and sets the board clock (TLS certificate dates) only while no fix has. Once a fix has told the time it is not asked any more: that fix's clock (run on by the uptime) sets the board clock instead. An answer counts only if it echoes the request (its originate time is the request's transmit time: a stray or late datagram is none), and the time is read past the NTP era change of 2036-02-07 too, but only for a firmware built less than 8 years before (a server saying 1900 is no 2036). |
| `TFT_LIGHT` / `TFT_IDLE` | The display's backlight: `TFT_LIGHT` 0–100 per cent while it is on (default 30; 0 keeps it dark, the USER button still lights it), `TFT_IDLE` 0–86400 seconds into a stay (the board has not moved) before it goes dark (default 300; 0: never). Read at boot. |
| `GPS_SYS` | The UC6580's `$CFGSYS` mask in hex (see [Hardware](hardware.md#hardware)). Read at boot. |
| `DIAG` | Seconds between [diagnostic log](usage.md#diagnostic-log) lines, 10–3600; `0` turns it off. Default 60. |
| `WEB` | `1`/`0`: the [status page](usage.md#status-page), and the radio staying on at home for it. Default 1. |
| `WEB_AWAY` | `1`/`0`: with `WEB` on, the page is served on the non-home networks too (e.g. a phone hotspot), to everyone there, without a password, and the radio stays on for it instead of sleeping once all is uploaded. `0`: off home nothing is served, also while the radio is on for an upload. Default 0. |
| `WEB_TOKEN` | 1–64 printable ASCII characters, no spaces: `status.json`, `/hist` and `/diag` answer only a request whose `X-GPS` header is this token; open the page as `http://<board ip>/#k=<token>` (bookmark it). Without it `/hist` and `/diag` still want an `X-GPS` header (any value; the page sends one), so a page of another site cannot keep the board busy decoding. Unset by default. |
| `UPLOAD_UNVERIFIED` | Where an upload that is not verified TLS (an `http://` `GEO_URL`, or `https://` without `/ca.der`) may go: `0` nowhere (the default), `home` only on a home network, `1` everywhere. The home LAN is flat: any host on it can answer for the server (ARP, DNS), take the Bearer key and, with a 2xx, make the board delete tracks it never stored. So put the server's root certificate on the board (`ca.der` next to `deploy.sh`, see below) rather than turning this on. While it waits the state is `err unverified`. The old `UPLOAD_AWAY_UNVERIFIED` (`1`: everywhere, `0`: home) is still read when this key is missing. |

- Put comments on their own lines. A trailing `# ...` is ignored only on
  flag, IP and name keys (`HOME*`, `*_HIDDEN`, `IP*`/`GW*`/`MASK*`/`DNS*`,
  `DEVICE_ID`, `HOSTNAME`, `KEEPALIVE`, `CPU_MHZ*`, `GPS_SYS`, `MIN_MOVE`, `DIAG`, `WEB`, `WEB_AWAY`, `UPLOAD_UNVERIFIED`, `UPLOAD_AWAY_UNVERIFIED`). SSIDs, passwords, `GEO_URL`,
  `API_KEY` and `WEB_TOKEN` are taken verbatim, since they may contain `#`.
- **The server's certificate:** `./deploy.sh` copies `ca.der` from the repo
  folder (gitignored; or `CA_DER=/path/to/root.der`) to the board's `/ca.der`
  with the other files, checked by size and SHA-256. It is the DER root
  certificate the server's chain ends in, at most 4 KB, e.g. for a Let's
  Encrypt server `openssl x509 -in /etc/ssl/certs/ISRG_Root_X1.pem -outform der -out ca.der`
  (check which root your server uses: `openssl s_client -connect host:443 -showcerts`).
- Values may be quoted. On the keys above the comment goes first, then the
  quotes: `IP="192.168.1.60" # static` is `192.168.1.60`.
- Invalid names fall back to the defaults. A UTF-8 BOM (Windows Notepad) is
  ignored. A file saved as ANSI does not stop the boot: an SSID or password
  that is not UTF-8 is used byte for byte, any other such value (or a key
  that is not UTF-8) is skipped with a line in the diagnostic log.
- A value out of range or unreadable keeps the default and says so in the
  [diagnostic log](usage.md#diagnostic-log), on the console too (in the field there
  is none): `env KEEPALIVE not 0 or 5-600 s, using 5 s`, the key named,
  never the value. `DIAG` is read first, right after the boot line, so the
  other keys' lines land in the log.

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

## 3. Deploy

```bash
./deploy.sh                      # default port /dev/ttyACM0, the board's USB Serial/JTAG
PORT=/dev/ttyACM1 ./deploy.sh    # another port (one that is not there: "No such port")
VENV=../venv ./deploy.sh         # python, pip and mpremote from another venv
                                 # (default ./.venv; a relative path is
                                 # relative to where you run the script)
```

The script first makes everything it sends, in a temporary directory and
before it talks to the board (a local failure, such as a page
`tools/web_min.py` refuses, then stops it with the board untouched): it
compiles `gps_logger.py`, `gps_clock.py` and `gps_receiver.py` to `.mpy`
(sent only when the firmware has no frozen logger). Then it asks the board
what it is (anything but the Heltec is refused: `Unsupported board`) and
checks the free flash (it stops with `Not
enough free flash` when the new files would not fit next to the old ones).
It makes `web.html.gz` with `tools/web_min.py` (~25 KB instead of the ~88 KB
`web.html`: comments and indentation out, gzipped with the firmware's 4 KB
window, `WEB_GZ_WBITS`: see the [status page](usage.md#status-page)) and copies
`gps_logger.mpy`, `gps_clock.mpy`, `gps_receiver.mpy`, the display's `gps_tft.mpy` and `gps_tft_font.mpy`, `main.py` and the status page, only as `web.html.gz`, as
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

## 4. Custom firmware (recommended)

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

The logger, `gps_clock.py`, `gps_receiver.py`, `gps_boot.py` and the display's modules are compiled with
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
removes the old `gps_logger.mpy`, `gps_clock.mpy` and `gps_receiver.mpy` (until then they shadow the
frozen modules and run as before) and any stray `gps_logger.py`, `gps_clock.py` or `gps_receiver.py` (`''` comes first in
`sys.path`), and clears the boot-fail counter. Right after esptool's reset
the raw REPL may not answer for a while; if `deploy.sh` cannot reach the
board, run it again.

The firmware also freezes `gps_clock.py`, the Clock (date maths, the one test
of a real UTC time, Stamps and Anchors), `gps_receiver.py`, the Receiver (the
UC6580's UART, the NMEA, the seconds it publishes in `st`; the logger imports
both, so a plain board gets a `gps_clock.mpy` and a `gps_receiver.mpy` beside
`gps_logger.mpy`), and `gps_boot.py`: the logger's start itself
(`gps_boot.start()`: the boot-bypass pin, the boot-fail counter and retry,
the OTA glue before the import), which `main.py` only calls. Frozen, its
code is not compiled into the heap at every boot, and an OTA brings a new
one; `main.py` is a file an OTA does not touch, so a change to `main.py`
needs `./deploy.sh`. `main.py` starts the logger itself on a plain
firmware and with an older frozen `gps_boot` without `start()` (the old
firmware after a rollback). And it carries a small `gps_frozen` module with the
commit (`-dirty` when the tree had uncommitted changes), the BUILD time and
five SHA-256s: `SHA` of the frozen `gps_logger.py`, `TFT` of `gps_tft.py` +
`gps_tft_font.py` (cat'ed in that order), `BOOT` of `gps_boot.py`, `CLOCK`
of `gps_clock.py` and `RX` of `gps_receiver.py`. When `deploy.sh` finds it, it
copies only `main.py` and the page and removes a stray `gps_logger.mpy`,
`gps_clock.mpy` and `gps_receiver.mpy`, but first
compares them with the repo's files:
when `SHA` differs (or a firmware built before it was recorded) it stops and
asks for `tools/build_firmware.sh --flash`, naming the file that differs
(`gps_logger.py`, `gps_tft.py / gps_tft_font.py`, `gps_boot.py`,
`gps_clock.py` or `gps_receiver.py`, the first in that order). A marker without
`TFT`, `BOOT`, `CLOCK` or `RX` (`tools/firmware.py` holds what each field hashes,
for both scripts; an older build, read as
`-`) skips that comparison. A marker whose words are not plain
(`A-Za-z0-9._-`) is no answer: it stops like a missing one. `FORCE=1
./deploy.sh` deploys main.py and the page anyway. A logger change means building and flashing the
firmware again. In an emergency `FORCE_MPY=1 ./deploy.sh` installs
`gps_logger.mpy` on a frozen firmware too, with a warning: it shadows the
frozen module and takes its ~100 KB of Python heap again. To roll back,
write the backup with the `esptool.py` command `--flash` prints, then run
`./deploy.sh`.

## 5. Firmware updates over WiFi (OTA)

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
newer one; `FORCE=1` publishes it anyway); the published build itself
again (the same build and commit: a publish whose `ssh` broke after the
renames) only does the cleanup below. `ssh` and `scp` run with
`BatchMode=yes` and a 10 s connect timeout (no password prompt, no hang),
and an `OTA_DEST` folder that is not there is refused (`mkdir` it for a
first publish). The sources are checked again right before the upload: one
edited during the build stops it. `MPY_DIR` must be at the `v1.29.0` tag
and `IDF_DIR` at `v5.5.2` (`FORCE_TOOLS=1` builds anyway). The
manifest carries the build (the commit time), the commit, size and SHA-256
of each file, and an HMAC-SHA256 over them with the key in `OTA_KEY_FILE`
(default `~/.config/gps-logger/ota.key`). The key is one line of printable
ASCII without quotes or spaces, 16 characters or more (the board reads it
from `.env`): `openssl rand -hex 32 > ~/.config/gps-logger/ota.key`; it is
checked as `tools/firmware.py` signs with it, the file stripped (a
two-line key passed the old check and signed with the newline in it: bad
hmac on every board). A key file readable by everyone is refused (`chmod
600` it), one readable by its group is warned about (the mode of the file
a symlink points to); the key itself is never printed. Only a clean,
committed tree is published. A `--flash` or dirty build carries the build
time instead of the commit time, so the OTA does not replace what was just
flashed by hand with an older published build.

```bash
OTA_DEST=user@host:/srv/www/gps-ota tools/build_firmware.sh --publish
```

Once the new manifest is up, the files only the replaced manifest named
are removed from `OTA_DEST`, and any other `micropython-*.bin`,
`web-*.html.gz` or `*.tmp` the new manifest does not name (one a failed
cleanup or upload left; plain names only, nothing else in the folder).
The manifest also carries its end, `expires` (90 days on, seconds since
2020 UTC), under a second HMAC, `hmac2` (the first one stays the string
older boards check, so they still take the manifest that updates them).
A board of this build refuses a manifest without it, a forged one, and,
once it knows the time, an expired one (`ota` state `expired`: publish
again). So an older signed manifest, kept and served again by a spoofed
server on the LAN, is good for 90 days at most, and the board refuses a
build not newer than its own anyway.

The `OTA_DEST` path is a plain one: only letters, digits and `_ . / -` (no
`~`, spaces or shell specials; `scp` takes it as it is, the `ssh` rename
shell-quoted, and the two agree only on such a path). Anything else, or a
host starting with a dash, is refused before the build, long before an
upload.

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
(or `/gps_clock.mpy`, `/gps_receiver.mpy`, `.py`) on flash (`FORCE_MPY=1`) would shadow the new firmware's frozen logger
(`''` comes first in `sys.path`): on a frozen firmware the install moves
it aside as `<name>.ota-old`, the valid new firmware deletes it, and
after a rollback `main.py` puts it back before the import. One that does
not move aside stops the install before `set_boot` (`ota cannot move
aside <path>`, waits `OTA_RETRY` as `shadow err`): it went on, and the old
module ran on the new image. A Ctrl-C
between `set_boot` and the reset no longer skips the reset (left there,
the next reset booted the new slot with no pending note, valid without a
health check); a boot slot that is set but not the running one at boot
(no reset since) is booted once. The new firmware runs pending:
2 minutes (`OTA_VALID_AFTER`), WiFi connected at that point (not only
once before: a build that breaks the reconnect is not confirmed), and the manifest read
once more (HTTP 200, the fields and the HMAC as above, every minute until
it is: `OTA_PROBE`; a build whose update path is broken would otherwise
only get the next one over USB, and a manifest signed with another key no
longer makes it valid), mark it valid (`/ota.mark` says `valid` first: a
power cut between the confirmation and the note's removal booted the
valid image as pending again, and the next boot now ends the confirmation
at once; a confirmation that fails keeps the image pending, one diag line
`ota <commit> not confirmed`, is tried again each tick and ends in the
health reset, where the steps below ran over an unconfirmed image with
`ota ok` in the log), and only then do the
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
SHA-256 or cannot be written, a flash error writing or reading the slot
back, is tried again an hour later (`OTA_RETRY`),
not every check (the image came again every 10 minutes, the loop blocked
for it); an error (server down) goes to the diagnostic log once per kind.
Points waiting for upload hold the check back only with a `GEO_URL` and
`API_KEY` to send them to (without, they would hold it for good). A build that crashed (the logger's crash
branch notes it in `/ota.mark`) or whose watchdog fired while the logger
ran is noted in `/ota.bad` and not tried again (the page then says `ota bad
<commit>`, not `up to date`). Any other rollback may
not be the build's fault and is a try instead, counted per build in
`/ota.tries`: at power-on (the cable pulled; MicroPython 1.29 reports a
brownout as a power-on too, and a build that browned out on a weak supply
was reinstalled after every boot), the no WiFi / no manifest reset (a
router or server outage), and a reset from outside (`mpremote reset`, a
USB-JTAG reset, the watchdog after an `mpremote` session stopped the
logger: Ctrl-C notes `stop`; each of these marked a good build bad at
once). A panic is a try too: it reads as a hard reset, like `mpremote
reset`, and no Python code runs to note it. The rollback's cause is the
old firmware's first boot after it: `gps_boot` notes it in `/ota.mark`
(`cause <n>`) before the import, so a reset of the old firmware before it
read the note (a Ctrl-C, a stall and its watchdog) is not taken for the
new build's. The build is tried again an
hour after the next boot (`OTA_RETRY`); after 2 tries (`OTA_TRIES_MAX`)
it is held a day (`OTA_HOLD`, UTC from the first check with a clock; twice
as long each further round, at most 32 days) and then tried again: never
bad for good, as two power cuts in the 2-minute pending window made a good
build (a build in `/ota.bad` from an older firmware's tries stays there:
remove the file to try it again); the health reset removes the note before it counts its
try, so a power cut between them cannot count one attempt twice, and it
writes the points in RAM first. The install marks itself `staged` in
`/ota.mark` until `set_boot` is through (it hashes the image first): a
power-off before that is no try of a build that never ran (`install cut`,
dropped); the new build turns it into `ran` before its import, so one that
hangs before its own check is a watchdog rollback (bad), not a cut. An
install begun notes itself in `/ota.dl`: one a reset cut waits
`OTA_RETRY` (an hour) before the next, and every third in a row counts as a
try (`ota build N install cut by a reset`): a brownout under the
download's load reinstalled it at every boot. Once confirmed
(`mark_app_valid`), a crash or stop no longer marks the image, and a
`valid` mark left by a power cut gives way to the old firmware's reset
cause. A boot without a pending note confirms the running image and
removes a staged page a power cut left (`<file>.tmp`, `.new`). The pages
must fit in the free flash before the 2 MB download (`no flash for page`).
A pending build without `OTA_URL` (its `.env` reading lost it) is not
confirmed: it rolls back, it could only be updated over USB. A note that is there but cannot be read or parsed is
dropped with the staged pages and the running image is confirmed once
(`ota note unreadable, dropped`): which build it named, and whether this
is it or the old one after a rollback, is not known, and holding it as
pending kept the radio on, blocked the memory reboots and could reset a
good build in a loop when the note would not go. The pending state ends
only after the confirmation steps (page promotion, `.ota-old`, the notes)
ran: a MemoryError in one is retried by the next tick. Points waiting for upload hold the check back, but not
for good: with no successful upload for 24 hours (`OTA_STUCK`; `err
payload` is cleared by a 2xx only, so an upload bug held back the update
that fixes it) it checks anyway, one diag line `ota: uploads stuck`.
Each step is in the
diagnostic log (`ota … installed`, `booted, checking`, `ok`, `rolled back`).
The status page shows the running build under *Logger*: the `Firmware` row
gives the build's date (the commit time, `fw_build` in the JSON, local
time) and the OTA state (`up to date`, `updated`, `checking`, `rolled back`, `ota bad <commit>`,
…); the commit is in the top bar (`FW 02e4317`).

After the migration, `--flash` over USB still works. It backs up both slots
(after an OTA the second one runs), writes the first slot and erases
otadata, so that slot boots even after an OTA had switched to the other.
