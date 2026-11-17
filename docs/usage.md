# Using it

[← README](../README.md)

- [Boot bypass](#boot-bypass)
- [Getting tracks](#getting-tracks)
- [Diagnostic log](#diagnostic-log)
- [Status page](#status-page)
- [Day history on the map](#day-history-on-the-map)

Power it on: the logger starts by itself, sets up the GPS, connects to WiFi
and shows the route and its state on the display (see [Hardware](hardware.md#hardware)).

**Serial console** (115200 baud) lines worth knowing:

| Line | Meaning |
|------|---------|
| `gps setup ok` / `no ack` / `again` | 5 Hz setup answered, not answered, or sent again |
| `gps setup not taken: 1 fix/s` / `no GST` | 10 s after an answered setup (once it has 3 valid seconds to tell by) the receiver still sends one fix a second, or no `GST`: it is set up again. Also in the diagnostic log, at most every 10 minutes |
| `gps setup unverified: no fix` | 30 minutes after an answered setup still no fix to check it by: said once, the check still waits for its 3 valid seconds |
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

## Boot bypass

Hold GPIO7 low while resetting to
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
  at most 30 s) and resets; the error is printed and a `crash boot <n>
  <error>` line goes to the diagnostic log (it was silent), and so does
  `crash boot fail <n>` when the REPL takes over. A count that can be
  stored neither in RTC memory nor in `/boot_fail` goes to the REPL and
  its retry too (it reset every 5 s for good);
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
An NMEA handler that raises anything but a MemoryError (a TypeError, a
KeyError: a bug, not a bad sentence) is not counted as a bad sentence: it
counts apart (`st["bug"]`) and writes `bug nmea <type> <error>` once per
exception type a boot; an exception in the receiver setup writes `bug gps
setup <error>` the same way.
A bug in the WiFi, upload or OTA tick, the status page's or a status
line's, no longer resets the board: it is
one diagnostic line a kind (`bug <tick> <error>`; a kind is the tick and
the exception type, so a message that varies is one kind, and one that
goes on is written again every hour, `BUG_AGAIN`: the one line a boot
rotated out of the log in a day or two), that tick pauses 5 s
(`TICK_PAUSE`) and logging goes on; an upload bug also backs the upload
off. Each tick's bugs since boot are counted: `tick_bugs` in
`/status.json` (`upload:1234 ValueError('x')`, the last one each) and
`bugs upload:1234,wifi:2` at the end of the diag status line. Ten MemoryErrors of one tick, each within 40 minutes of the one before
(`GUARD_MEM_MAX`, `GUARD_MEM_QUIET`: over the longest upload backoff, so a
lasting one in a backed-off tick keeps counting; counted per tick, and a
pass that runs through between them, an upload in its backoff or an idle
WiFi tick, clears nothing; sporadic ones further apart never add up) are a
memory reboot, as the crash reset used to be. A bug in the logging itself,
and any bug while an OTA image is pending, still goes to the crash branch.
A MemoryError in a guarded tick is the exception while an image is
pending: no reason to blame the new build for a low heap in its first
minutes (it went to `/ota.bad` for good), so it is swallowed (gc, the
pause) and counted, but a persistent one is the build's defect: the
`GUARD_PEND_MAX`-th (5) of one tick raises, the image crashes and is
rolled back and marked bad. No memory reboot while an image is pending (it
would abort it).

In the REPL after 5 a frozen firmware no longer waits for good: after 30
minutes (`BOOT_RETRY_MS`) a timer sets the count back to 4 and resets, one
more start (a failure is another 30 minutes in the REPL). That timer can
reset an `mpremote` session that runs past it: `gps_boot._retry.deinit()`
at the REPL stops it, and the boot bypass skips all of it. A plain firmware
(the `.mpy` from `deploy.sh`) stays in the REPL. A failure that `main.py`
catches while an OTA image is pending (it cannot even import) is marked
as that build's crash, as the logger's own crash branch does.

## Getting tracks

They live in `/gpx/`. With the boot bypass held:

```bash
.venv/bin/mpremote cp :/gpx/20261005_101500.trb .
python3 tools/trk2gpx.py 20261005_101500.trb > 20261005_101500.gpx
```

New tracks are binary `.trb` files (see Format), older ones `.trk` text;
`trk2gpx.py` converts both. Next to a track you may find `<file>.pos`
(upload progress) and `<file>.bad` (batches the server rejected; retried
until 8 more rejections per boot), and `<file>.err` (a track that could not
be read three times, parked so the newer ones upload). `trk2gpx.py`
converts `.bad` and `.err` files the same way (a parked `.bad`, `.trk.bad.err`
or `.trb.bad.err`, is text). It exits 1 with a line on stderr when a file
gives no point or is empty; after a damaged record of a `.trb` the GPX starts
a new segment at the next keyframe (no straight line over the lost points). A `.trb.bad` holds the
rejected points as text lines, like a `.trk`.
To queue a `.bad` again (a `.trb.bad` becomes a `.trk`):

```bash
.venv/bin/mpremote exec "import os; os.rename('/gpx/20261005_101500.trk.bad', '/gpx/20261005_101500.trk')"
```

## Diagnostic log

The console is gone once
the board is on a power bank, so the logger also appends to `/diag.log` on
flash: a `boot cause N` line at every start, one status line a minute, a `stay left wifi` line when an away
stay ends without a fix (see [Stays](internals.md#stays)), `gps reset loop …` lines
(when the receiver keeps restarting, so they can be set against where you
were), `gps setup not taken: …` (at most one per 10 minutes,
`GPS_NOT_TAKEN_SAY`, with `(N more)` for the ones between), `gps setup lost: no
GST`, `gps refused <command>`, `gps silent N s, no bytes from the receiver` and
`gps back after N s` (no byte from the receiver for a minute, `GPS_SILENT`),
`time back <false> <real>` (the fixes were ten seconds running more than a minute behind
the last line written, `TIME_BACK`: a time with a corrupt digit had got past the
checksum, so the track would have stopped until it came true; it goes on in a new
segment),
`upload err …`
and `upload aborted …` lines, `upload low idf N KB` / `upload low mem N B`
when a POST waits for memory, `upload ca err …` when `/ca.der` cannot be
used, `upload GEO_URL https without /ca.der: …` at the start when the server
is not verified, `upload: <file> unreadable, parked as .err` (or `… not
parked <error>`), `clock set <UTC>` once a boot when the board clock is set from the
GPS (`clock set <UTC> gps`: the last fix's clock run on, the fix gone;
`clock set <UTC> ntp`: from NTP, no fix since the boot), `clock ntp
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
the *Event log* below; `/status.json?diag` carries the last 15 lines,
`WEB_DIAG`, plain `/status.json` no longer reads them). `/diag` and `/hist`
answer only a request with an `X-GPS` header (the page sends one; with
`curl`, `-H 'X-GPS: 1'`), and with `WEB_TOKEN` set in `.env` all three
want it to be that token (`-H 'X-GPS: <token>'`; the page takes it from
`#k=<token>` in its URL). After a walk, plug the board in and read it:

```bash
.venv/bin/mpremote resume fs cat :/diag.log
.venv/bin/mpremote reset
```

| Field | Meaning |
|-------|---------|
| `boot cause N` | Why the chip started (`machine.reset_cause()` of MicroPython 1.29): 1 power-on (also a brownout, or a power bank switching back on), 2 hard reset (the logger's own reset after a crash or for TLS memory, `mpremote reset`, a panic), 3 watchdog, 4 deep sleep, 5 soft reset (Ctrl-D), 0 other (e.g. a reset over USB-JTAG) |
| `prev_up` | On the boot line: how long the run before lasted, in seconds, from its last lines (to within a minute; exact before a deliberate reset, which writes a status line first; `0` = under a minute, `-` = no log, or more than 40 event lines after its last status line). `/diag.1` only when the whole of `/diag.log` has no status, crash or boot line (just rotated) |
| `up` | Seconds since boot (summed from tick differences, so it does not wrap after 12.4 days) |
| `utc` | GPS time, `-` until the receiver has one |
| `nmea ok/bad` | Valid and broken NMEA sentences so far |
| `gpsboots` / `loops` | Receiver restarts and reset-loop warnings so far |
| `sats seen/used` | Satellites with a signal (from the receiver's satellite list, its GSV every 5 s) and used in the fix |
| `fix` | `No_data!`, `Waiting_fix`, `2D_fix`, `3D_fix` |
| `hdop`, `points`, `sent` | Last HDOP, points logged and uploaded since boot |
| `wifi`, `pulses`, `mem` | WiFi state (spaces as `_`), keep-alive pulses so far, free Python heap in bytes |
| `idf F/L` / `heap` | ESP-IDF heap free and its largest block, KB, then the Python heap size, KB. TLS needs ~40 KB in one block; below 48 KB (`TLS_MIN_KB`) no POST is tried (`upload low idf`); at home, if it stays low for 5 minutes with tracks waiting, the board reboots cleanly (at most once per 30 minutes) |
| `vbat V/S` | The battery: median of 32 reads and their 10th-90th percentile spread, V (a spread over 0.5 V is no battery: the charger's swing on USB) |
| `bugs`, `diag_errs` | Only when there are any: each guarded tick's bugs since boot (`bugs upload:12,wifi:1`), diag lines that failed to write (`diag_errs 3`; also in `/status.json`) |

## Status page

While the board is connected to WiFi it serves a page at
`http://<board ip>/` (e.g. `http://192.168.1.60/`) and the same data as JSON at
`/status.json`. With `?diag` (`/status.json?diag`) the JSON also carries the
last lines of the diagnostic log (the page itself reads the whole log, below).

### status.json fields

| Field | Meaning |
|-------|---------|
| `fix`, `sats` | Fix and satellites seen/used ("used" from the satellite list: GGA stops at 12) |
| `satellites`, `satellites_age_s` | The satellite list (system and number, e.g. `GPS 7`, `Galileo 11`, `BeiDou 22`, `GLONASS 3`; signal in dBHz, elevation, azimuth, used) and its age; none once it is 2 min old (two `SV_POLL`), so a silent receiver's last sky is not shown as live |
| `hdop` | HDOP |
| `utc` | GPS time (UTC; the page shows it in Hungarian time, CET / CEST); `null` once the receiver has been silent over `NO_DATA`, as the position |
| `position`, `pos6` | Position to 5 decimals, and to the track line's 6; both `null` once the last second is over 3 s old (`NO_DATA`), so a silent receiver's last fix is not shown as live |
| `nmea` | NMEA counts |
| `rx_bytes`, `rx_age_s`, `good_age_s` | The GPS link: bytes read since boot (wrapping at 2^30); seconds since the last byte / the last whole NMEA sentence, `null` for none yet |
| `receiver`, `gps_boots` | The receiver model; receiver restarts |
| `points`, `left`, `sent` | Points logged, waiting and uploaded |
| `bad_files`, `err_files` | Quarantined `.bad` files; parked `.err` files |
| `upload`, `tls`, `upload_age_s` | Upload state; whether the upload's TLS is verified; seconds since the last accepted batch (`null` for none since boot) |
| `days`, `stay` | The day history's days; stay |
| `uptime`, `prev_run`, `prev_end` | Uptime and how long the previous run lasted (e.g. `58 min 14 s`; seconds in the JSON), `prev_end` the time of its last line |
| `boot_cause` | Boot cause |
| `fw`, `fw_build` | The build |
| `ota`, `ota_pend` | The update's state (`no flash for page`: the pages would not fit, checked before the download; `install cut`: a power-off before the new slot was set to boot, dropped, no try); while a new build runs pending, `[seconds since its boot, why the manifest read failed, why the confirmation failed]`, else `null` |
| `temp` | Chip temperature |
| `vbat`, `vbat_swing` | Battery: median of 32 reads in V; their 10th-90th percentile spread |
| `vbat_pct` | What is left, 0-100 %, from a 1S LiPo curve: Meshtastic's `OCV_ARRAY` from its 3.30 V on, where the board stops (Heltec's datasheet: 3.3 V at least), as 0 %: 4.19 V 100 %, 3.84 V 60 %, 3.30 V 0 %, linear between. On USB the charger lifts the voltage, so it reads high while charging |
| `vbat_left_s` | About how long the battery lasts, from a sample a minute (`VBAT_EVERY`) over the last 30 min: the least-squares slope carried down to 0 %, a minute at least. `null` until the samples span 10 min and fall at least 2 points, and at 99 % and up (on the charger, or too short to tell). A jump of more than 4 points (`VBAT_JUMP`) off the median of the last 3 samples, on or off the charger, that holds the next sample too starts the samples again (one sample off alone, or less, is noise) |
| `vbat_err` | ADC reads that failed (each kind a `bug vbat` line) |
| `battery` | The page's text, `74 % (3.95 V), ~2 h 10 min left`, or `none, on USB` when the spread is over `VBAT_SWING` 0.5 V: without a battery the charger's output swings, measured 0.84-0.94 V on USB. The verdict, battery or none, turns on the second measurement in a row that says so (`VBAT_AGAIN`), one stray one is let go |
| `mem`, `heap_kb`, `flash_kb` | Free Python heap and flash |
| `wifi`, `pulses` | WiFi; keep-alive pulses |
| `diag` | With `?diag` only: the last diagnostic log lines |

The page's Logger card has a Battery row: icon, per cent, the time left, the
voltage, amber to 20 %, red to 10 %.

No RAM while the JSON is built is no bug: the client is dropped (it is not
counted in `web_errs`, no `web error` line).

### The page

The page follows the phone's light or dark setting in one colour world (green,
amber and red only for meaning). A thin top bar names the board (`device`),
says `LIVE` (or `OFFLINE · 12 MIN AGO`, `NO ANSWER`, `BOARD ERROR`) and
gives the uptime and the firmware's commit. Under it, the position and the
day sit beside the map (on a phone the map comes first):
- the *now* block: moving or standing still (the track's last minute, or
  the board's own `stay`), the fix and the satellites used of those in
  view; the address of the position from Atlas' Photon
  (`atlas.crusand.org/api/v1/photon/reverse`, asked again only after 25 m
  and 20 s) with district and city; how long it has been here, the speed,
  the accuracy (HDOP x 2.5 m) and the age of the last fix; the coordinates
  (N/S, E/W) under them;
- *Day*: a chip a day (the days the board keeps, oldest first, today last) and
  `‹ ›` arrows; on top the km, the moving time and the trips; below, the
  day as a line (as in Maps Timeline): a dot and a solid stretch per trip
  (start time, km, duration, mean km/h) and a dashed stretch per stop (its
  length). Over each line the place where it starts or ends, from Photon,
  as for the position (a railway station within 150 m, else city and
  street, once a place, kept while the page is open, the 400 newest); no
  answer, as offline: the line alone, tried again once a minute (one request
  at a time); a place the geocoder keeps failing on goes to the back of the
  queue after 3 tries, so the others still get their names. A trip still
  going has no end place yet. A trip is a run of points without a gap over
  5 min that covers 100 m at least; a few fixes while standing belong to
  the stop. A trip clicked is drawn darker on the map, which zooms to it
  (clicked again: the whole day). `GPX` saves the shown day as GPX 1.1
  (built in the browser, a segment per gap). A day without a trip says so
  in a line, no row of zeros;
- the map: OSM-carto in its own colours (through the homelab Atlas tile
  proxy, `atlas.crusand.org/tiles/geoapify/`, which answers pages on a
  private-network address such as the board's), the shown day's track
  fitted into it, a blue dot with an accuracy ring and, while moving, a cone
  the way it goes; without a fix it says `NO FIX` in the middle.

Below, three columns split by hairlines (one under the other on a phone):
- *Logger*: how long the free flash holds a 1 Hz drive as the number (`≈6
  days`, at 7 bytes a `.trb` point), then a row a subsystem with its detail,
  a green, amber or red dot and the word (`Warn`, `Error`) only when it is
  not nominal; the heading counts them (`7/7 OK`). `GPS receiver` (model,
  fix, satellites used/seen, HDOP; amber without a 3D fix or after a failed
  setup; the reset loops since boot are counted in it, not coloured), `GPS link` (B/s, the last whole frame, restarts, bytes read,
  the share of broken NMEA sentences), `Upload` (state, points waiting,
  sent, errors, `.bad` files, the last accepted upload), `Storage` (free MB,
  points logged, track files), `Memory` (the largest IDF block, red under
  the 48 KB TLS needs, and the Python heap), `Wi-Fi` (an arc icon with 0–3
  arcs lit, the network, the link quality in per cent, 2 x (RSSI + 100)
  capped at 0–100, -63 dBm = 74 %, and the dBm; amber below -67 dBm, red
  below -80 dBm; keep-alive pulses), `Firmware` (the build date, the OTA
  state). Counters that are 0 are left out of the rows;
- *Sky*: a plot (centre = overhead, edge = horizon, north up; colour =
  system: GPS, Galileo green, BeiDou amber, GLONASS pink; bigger = clearer;
  filled = in the fix, hollow = not used), a bar each satellite in use
  (strongest first), and the signal in words: the overall grade and how
  many are strong (35 dB-Hz up), fair (25 up) or weak, how many more are in
  view;
- *Diag log*: the whole diagnostic log on the board (`/diag.1`, then
  `/diag.log`: up to ~26 h), aggregated rather than listed line by line
  (laid out after Sentry / Loki "patterns"). The window in the heading
  (`10:16–16:20 · 6 h 04 min`), then the typical accuracy (`±5 m`, and the
  share within ±5 m), a summary (the share of minutes with a 3D fix, the
  time without a fix or data, the minutes the receiver sent nothing in how
  many drop-outs, the boots and how many were power-on, the firmware
  updates each counted once with the failed ones, the broken NMEA sentences
  as `1 in N`), the accuracy as a dot matrix (a column a time slice, taller
  = more precise: 12 dots ±2 m, one ±15 m and worse; green ±5 m, lime
  ±12 m, amber worse, a red dot where there was no fix) with the receiver
  link as a strip under it (data every minute, gaps, none) and a red mark
  with its start time under each drop-out; then a row each drop-out (start,
  end, length) and a row a kind of event (the latest first, then the most
  severe and the count): its last time, its name coloured by meaning (red:
  crash, caught bug, GPS reset loop, GPS silent, track write error, flash
  full, upload error, a rolled back update; amber: update check, memory
  reboot, clock back, WiFi failed / lost, upload waiting for memory, GPS
  setup refused, `.env`; green / white: boots, updates, WiFi back; grey:
  receiver start, stay ended, display backlight, other), a detail (e.g. `5
  restarts in 3–24 s`, `krumpli · status 201 · 12 s each`, the silent time
  summed from `gps back after N s`, the updates `2e8bc6e → a3f2271 →
  69f8c15`), a per-hour sparkline (not on a phone) and the count (`(+N
  more)` and `N more windows` included). An event line has no time of its
  own: it takes the time of the next status line with GPS time (lines after
  the last one: the last time). The page reads `/diag?f=1&o=0`, then on
  from each reply's end, then `f=0` the same way (`X-Diag-Size` is the
  file's size; ~16 KB a reply, `HIST_REPLY`), parsing as it goes (only the
  totals are kept, not the lines); after that once a minute `f=0` from
  where it got to. When `/diag.log` is smaller than that, it became
  `/diag.1`: the page reads both again; so it does when the `f=0` answer's
  `X-Diag-Old` (the size of `/diag.1`, 0 without one) is not the size it
  read (a rotation missed while the board was away, its `/diag.log` grown
  past the page's offset again). The minutes are summed from the status
  lines' `up` (the time since the line before, so any `DIAG`; after a boot
  the `up` itself, after a gap one step).

Under the columns the device in one strip: how long it has run and since
when (local time), the boot cause, the run before it and until when, chip
temperature and clock, GPS time, the position; and a folded table of every
satellite (signal, elevation, azimuth, used).

The `GPS link` row (bytes a second between two polls, the age of the last
whole frame) turns amber after 3 s, red after 10 s; the `Upload` row turns
amber when points have waited over 30 min without an accepted batch. While
the receiver sends nothing whole for over 10 s, a red banner over the page
says `GPS SILENT · 42 S` and whether bytes still arrive (wiring or baud
rate) or nothing does (power or the TX wire): useful while moving the wires
with the page open.

Amber or red marks what needs a look. The map tiles and the fonts (Inter,
JetBrains Mono) load from the internet when the phone is online; without
it the page shows an empty map and the system font (the font sheet loads
without blocking the first paint, so an offline phone does not wait for it).
The page is a file on the
board, kept only gzipped (`/web.html.gz`, ~25 KB instead of the ~88 KB
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
0.5 / 1 / 2 / 4 KB window, measured then; 25.3 KB with the 4 KB window on
2026-11-15). A board deployed before this still has the
plain `/web.html` (its `.gz` may have the 32 KB window, too wide to
inflate here): it goes to clients without gzip until the next deploy or
OTA removes it. When you change the page by hand, make the gzip the way
`deploy.sh` does (`python3 tools/web_min.py web.html web.html.gz`) and copy
only `/web.html.gz`; then it asks
for `/status.json` every 15 s (`WEB_REFRESH`, sent in the JSON) and redraws
only the boxes; the map tiles reload only when the position moves to
another tile. When the board does not answer (asleep, out of range) the
panels fade and `LIVE` turns into `OFFLINE · 2 MIN 10 S AGO`. A reply that
is not the JSON is told apart: the board's own error (a bug building the
JSON is a 500, or its text/plain 403, 404, 503) shows as `BOARD ERROR` with
the first words it said (`HTTP 503 ...`; a 500 without the status); a
gateway's or a captive portal's HTML page (502, 504, 403) is not the
board's and stays `OFFLINE`. HDOP is rounded to two decimals on the page;
the JSON keeps the receiver's value.

## Day history on the map

The page draws today's track (since local
midnight, the phone's time zone) on the map, with `TODAY x.xx KM` in the
header (a day stepped to in *Track*: that local day, read once, with its date
in the header; it is drawn also without a fix), and zooms out (17 down to 6) until the track fits around the
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
free (`HIST_RESERVE`) the oldest days go first (today's stays: by the clock's
UTC day, not the newest name; a day more than one after the clock's, from a
time that leapt ahead, gets no file and goes first; a day older than the 7
kept, from a track uploaded days late, gets none either), also before a
batch would be dropped on a full flash, and nothing is added under
2 × `MIN_FREE`. A failed write (or a bug in the copy) costs only the
drawing: it is counted (`hist_errs` in `/status.json`) and in the
diagnostic log as `track hist error`, the first and then one per 10
minutes. An append cut short (the flash full mid-write) leaves no
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
there: the next append's keyframe starts there. A local day that begins
inside a UTC day (east of UTC: its first hour is the UTC day before) asks
that day with `&t=HHMMSS` (the local midnight in UTC) first: the board
answers no lines, only `X-Hist-Next` at the last keyframe before that time,
found by the keyframes' text without decoding the deltas (`HIST_SEEK_MS`;
`X-Hist-Seek: 1` when it ran out of time: ask again from there), so a past
day no longer reads the whole UTC day before it (~16 replies of ~2.4 s for
its last hour). A past local day that ends inside a UTC day stops reading
that day at the first reply whose last point is past the local day's end
(points in time order), not at the UTC day's own end, but only when
`day_span` says that file is in time order: a track uploaded after a newer
one (held back, retried) sits further on in it, and the day is read whole. A day without a file is a `204` (the page asks for today's
every minute before its first point). `status.json`'s `day_span` gives each
`.trb` day's first and last point (seconds into the UTC day; one day read a
request, kept with its file size), the earliest and the latest in time even
when a late upload put them out of file order, and a third item `1` then
(`[first, last, 1]`), so the day picker offers a UTC day's next local day
only when a point is there. A day is ~6-7 bytes a point
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
`WEB=0` in `.env` turns both off. On the other networks the page is
served only with `WEB_AWAY=1`, which also keeps the radio connected for it
there; without it the board serves nothing off home, also while the radio
is on for an upload (the page has no password: a hotspot or a friend's
network got the live position, the 7-day track and the diag log with its
SSIDs). `WEB_AWAY=1` takes a `WEB_TOKEN` (refused without one): the data
then go only to a request with the token. Those networks use DHCP: find
the board's address in the hotspot or router client list (by its DHCP
hostname, `HOSTNAME`) or in the `web http://…/`
console line.
