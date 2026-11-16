# How it works

[← README](../README.md)

- [GPS](#gps)
- [Points and track files](#points-and-track-files)
- [Stays](#stays)
- [WiFi](#wifi)
- [Upload](#upload)
- [Watchdog and recovery](#watchdog-and-recovery)

## GPS

- **5 Hz with averaging.** At every boot the logger asks the receiver for 5
  fixes a second and for `RMC` and `GGA` every fix, `GST` once a second, `GSA` and
  `GSV` every 5 s, and no `GLL` or `VTG` (see [Hardware](hardware.md#hardware)). More than 8
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
  indoors) the check waits for 3 of them, however long that takes (after 30
  minutes, `GPS_VERIFY_MAX`, it says so once: a receiver that answered but stayed
  at 1 fix a second is still caught when the fix comes). It counts as taken only with 5 fixes a second and `GST`. One that answers every
  setup but keeps sending what it turned off gets the second retry after 5 s too,
  then 10, 20, 40 s and every minute (`GPS_IGNORED_MAX`). A start-up banner (power
  dip) sets it up again at once and starts that backoff afresh; no `GST` for 30 s
  with sentences coming (`UC_GST_LOST`) is a restart whose banner was lost.
- **Satellite list.** From `GSV`, one list a `GSV_EVERY` (5 s): a round is the
  `GSV` of one receiver second (any systems, also a `GPS_SYS` mask without GPS),
  closed by the next second's first `GSV`. The same satellite on two bands counts
  once, with its stronger signal; used from `GSA`. A list older than 2 minutes
  is not used for the seen count. A receiver restart forgets the list and the
  model (`$PDTINFO`), which is asked anew.
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
  point until a reboot.) The minutes start with a digit (`float()` took `+5.5`). A time that does
  not exist (hour 24, minute 60, second 61) or an empty time is no second of its
  own (no bad sentence: the checksum held), and its position, speed and GST go
  nowhere. A `GSA` whose later field fails leaves no fix type; an HDOP of 0 or
  below and a negative `GST` are not taken; a `GSV` of a talker of no system is
  not counted as seen. A receiver restart forgets its date (kept, after a day of
  silence the next fix had the old day), and so do bytes again after a minute
  without any (`GPS_SILENT`, on whatever read them: the POST guard's timer
  too): the second left open before the silence is published with its own
  date first, the half line dropped. A float field over 16 characters is a
  bad sentence (39 digits are `inf` in single precision), and a non-finite
  value is `null` in `/status.json`.
- **Midnight.** Only `RMC` carries the date, and only with a fix (before one the
  UC6580's `RMC` carries a made-up date). A second that runs back past midnight
  without its own `RMC` has no date, so it is not a fix rather than a day off.
- **Clock.** The board clock (for TLS certificate dates) is set from a fix,
  from NTP until the first one.

## Points and track files

- **Sampling.** Each averaged second is sampled once. Seconds published
  together after a blocking POST, scan or setup are queued and all sampled.
  The queue holds 8 (`SECONDS_Q`). A longer block (a POST over a weak
  hotspot can take up to a minute) thins it evenly instead of dropping
  its start, so the track keeps 8 points spread over the whole block
  rather than a gap (`q_thin` in `/status.json` counts the thinned ones).
  A point is recorded when it is more than 1 m (`MIN_MOVE`) from the last one.
- **Filter.** Skipped: fresh HDOP above 5, or a GST estimate above 20 m (the
  larger of latitude and longitude, 1σ). A cold start can show a good HDOP
  while the position walks in from far away (measured: 230 m, over 65 s); GST
  catches it. The UC6580's does not (0.3 m while 4 m off), so its first minute after
  a start counts only above 15 km/h (`UC_SETTLE_KMH`). 2D fixes omit
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
  - A track line's number without a digit (".", "+.") is no number: on the
    board MicroPython's `float()` reads them as 0.0, and such a line went up
    as a point at 0,0. A latitude or longitude whose text is past 90 or 180
    is refused even where a single-precision float rounds it onto the limit
    (90.000003 is 90.0 on the board).
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
  - Not caught at all: a flipped bit inside a delta's value (no checksum in
    the format) decodes a wrong but valid number, and it carries on to the
    next keyframe: a fuzz of single flipped bytes made ghost points in about
    half the cases, a few of them hundreds of km off. A check per keyframe
    span (a CRC, or a bound on the step per second with a keyframe for a
    larger one) would change the format and both readers. LittleFS checks
    its metadata, not the file data.
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
  space is freed, with the present second: the seconds that came while it
  was full are dropped, as after a fix loss (kept, they began the new file
  an hour back with a point every 6-11 minutes). A track file is never
  created over an existing one of the same name (a time that went back over
  a track not uploaded yet): the new name goes on a second at a time. A
  piece of a batch that runs out of memory is no failing file: the batch
  waits for the next flush in the same file.

## Stays

Indoors the GPS wanders tens of metres with a good HDOP and reports 5–20 km/h
while lying still (measured on the board: speed p50 5.4 km/h, drift p90 46 m,
one-minute centroids up to 72 m apart). Neither the 1 m rule nor speed can
tell a stay from a drive, so the stay filter works on positions.

- **Entering.** Two consecutive one-minute centroids within 50 m make a
  stay. A window needs 10 usable fixes; with only a few a minute (indoors) it
  stays open until it has them, up to 5 minutes.
- **No drift on flash.** While a stay may still be proven, its windows'
  lines are held in RAM, and they are dropped when it begins, after a boot or
  a new fix too. The way in is kept up to where it first comes within 50 m of
  the anchor, and an excursion out past 150 m that came back (a U-turn past
  the place, back to park) is never dropped; a multipath spike out there is.
  A way out already begun is kept too, a car's included (a stop of ~90 s:
  the first 150-290 m of the drive off went as drift). Closing a file for
  upload never writes the held lines; they move on to the next file.
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
  none. Nor while the device keeps receding, 15 m out already (a walk out
  within 50 m): the way out keeps its first metres. Drift out there delays a
  point by 5 minutes at most.
- **At home** (a home network): no anchor points, also while the home link is
  down (a router restart) or the radio sleeps: a stay that began at home stays
  a home stay (`sleep_home`) until it ends. A non-home network (a phone
  hotspot) connected during it makes it an away stay; its first anchor point
  waits 30 s, as after an excursion, so it never lands in the first metres
  walked out.
- **Arrival.** Entering a stay closes the current track on any network, so
  the arrival uploads during the stay.
- **Leaving.** 3 consecutive samples beyond 150 m that each gain at least
  2 m (a vehicle), or 10 samples out there that gain at least 5 m in all (a
  walk). A multipath jump that freezes 200 m away does neither, however long
  it lasts, and an excursion that turns back does not count. The way out is
  written from near the anchor, from the last 90 samples, in time order
  (kept in preallocated arrays, 28 bytes a sample: as line tuples they took
  ~20 kB of the heap and broke uploads). It begins at the last sample within
  5 m of the anchor, or within 50 m after 30 s without a sample kept: a home
  stay in calm reception wrote up to 90 samples hours old before the way
  out, the new track named after the first. Each line is rebuilt exactly as
  it would have been written then, the board's single-precision rounding
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
  is around while learning, or with no scans (keep-alive off, the radio
  connected), only a fix ends the stay, or:
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
  trip; nor one whose last two fixes are over 150 m from its centroid (a
  car stopped for about a minute and a half and off again). A walk turning
  back within ~80 m still can: indoor drift looks the
  same. A fix back after 5 minutes more than 150 m away writes
  what was held to the old track before it closes.
- **Leaving home** is accepted only once the home network is out of range:
  below −85 dBm (`SEEN_MIN_RSSI`) on the home link or in a ~2 s scan while
  the radio sleeps after it (a hidden home's learned BSSIDs too), below
  −75 dBm (`ROAM_MIN_RSSI`) in a scan otherwise. While it is in
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

## WiFi

### Choosing a network

1. Scan once, keep the strongest BSSID per SSID.
2. Try a home AP at ≥ −75 dBm first, then other visible known APs, strongest
   first, then configured networks the scan missed, in config order: hidden
   ones in every round whose scan shows an empty-SSID AP, all of them in the
   first round (at boot, and again after a link loss, a wake from sleep or a
   new stay) and every 4th (`UNLISTED_EVERY`), since each costs a 12 s try
   with the radio on. A
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

### Radio on only when needed

- Once no finished track waits, nothing is in RAM and the open track holds
  no points, the radio is turned off (`wifi sleep`). On a home network with
  the [status page](usage.md#status-page) on (`WEB`, default) it stays connected;
  with `WEB_AWAY=1` on the other networks as well.
- After an hour it searches again, but only once there is something to
  upload (with `WEB` on, every hour anyway, so it finds home again).
- A new stay wakes it at once, so a home stay is known as one and the arrival
  uploads. Away from home the first anchor point waits 60 s for that round. A
  stay that began at home stays a home stay while the radio sleeps.
- If no network is found and there is something to upload, it retries with
  backoff (15 s doubling to 10 minutes) until one connects, the radio off in
  between; with nothing to upload it sleeps. With `WEB` on and no keep-alive
  scan that could see home come back (`KEEPALIVE=0`, or a hidden home whose
  BSSIDs are not learned yet), it searches on with that backoff
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

### Power bank keep-alive

With the radio asleep the board draws less than
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

### CPU clock

The main loop runs at 80 MHz (`CPU_MHZ`). A POST runs at
160 MHz (`CPU_MHZ_UP`) so the TLS handshake stays quick, then the clock
drops back. The UART runs off the fixed APB clock, so its baud rate is unaffected.
`CPU_MHZ=0` leaves the firmware default. A failed clock change is tried
again (3 tries); a clock refused every time is left alone for 30 minutes
(`FREQ_RETRY`), then tried again (the main loop re-applies `CPU_MHZ` with
each status line, POST or not), so one passing failure of the drop-back
after a POST does not keep the board at 160 MHz.

### Roaming home

On a non-home network, when nearly stopped (speed below
5 km/h, or in a stay), the logger scans for home every 5 minutes and switches
to a home AP at ≥ −75 dBm. It backs off to 60 minutes only after a failed
switch to a visible home; stopping after a drive resets that only when a
visible home is in view. A hidden home is tried with `bssid=None` (matched by
SSID) when a strong empty-SSID AP is seen, learned or not (another node of a
mesh may not be learned yet); a neighbour's hidden AP can cause a false
attempt, and switching to a hidden home can be slower.

### Hidden home

While connected to it (and only then) the logger learns
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

## Upload

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
  POST is resent (Dawarich deduplicates). Only a 2xx with a JSON
  `Content-Type` commits. One that is not JSON (an HTML or plain-text
  body, no `Content-Type` at all, or a `204 No Content`) is a captive
  portal or proxy answering, not Dawarich: it counts as a failed POST
  (`upload err html …` or `upload err not json … (captive portal?)` in the
  diagnostic log). So does a 2xx whose headers could not be read to their
  end (a reset, a timeout, the guard: `upload err head not read …`): it is
  resent and Dawarich drops the duplicate points; a portal's bare `200`
  used to delete the track as sent. A file that cannot be deleted after its
  last batch is parked as `.err` after 3 tries.
- 400 / 413 / 422 quarantine the batch to `<file>.bad`. Three in a row
  without a success count as a server problem: backoff, no more quarantine
  (rejections of a `.bad` itself do not count here).
  A full `.bad` (64 kB) pauses that file (`BAD FULL`) instead of dropping
  points; pull or delete it, then reset. `BAD FULL` lasts while that `.bad`
  is on flash and its track waits; a copy that fails for another reason
  (an I/O error, or the flash under the reserve; once the recording stopped
  for want of space under half of it: the quarantine may use the other
  half then, as a poison batch on a full flash held the only track for
  good, but not while it records, whose new points it would cost) is
  `fs err` and is tried again.
  Points moved to a `.bad` leave `left` once (a `.bad` is not counted).
- Other errors back off from 15 s up to 30 minutes (`Retry-After` is
  honoured); 401 waits 30 minutes. The same batch answered 3 times in a
  row with a server error (5xx, or a proxy's 404 / 403) holds its track
  for 30 minutes while the others go; the first such hold ends with any
  track's 2xx (the server is back, e.g. after a Dawarich restart), a
  second one for that batch only when it runs out. Holds for payload
  errors stay. Between POSTs it pauses 5 s at home, 30 s
  elsewhere. Flash and memory errors back off too, and with every waiting
  track held back (state `held`) the radio sleeps until the first is due.
- An unreadable track (`OSError` on read) is held back so the newer ones
  still go; after three tries it is parked as `<file>.err`
  (`upload: <file> unreadable, parked as .err` in the diagnostic log, or
  `… not parked <error>` when even the rename fails; `err_files` in
  `/status.json` counts the `.err` files). A `.pos` that
  cannot be written is a flash error (`fs err`, retried), not a read error.
- A batch in which no point is valid (an erased flash block reads `0xFF`:
  a damaged 1-byte record each byte) reads on past 50 records, up to 4 KB
  (`UPLOAD_SCAN`), and the next round follows at once: a 4 KB bad block
  costs one round, not ~80 rounds of 50 bytes, 5 s and a `.pos` write each.
- **Reading:** track lines are read with `readinto` into one 512-byte
  buffer (`LINE_MAX`) and a `.trk` point is parsed there and written
  straight into the JSON buffer, so a 50-point batch makes only small,
  short-lived objects (the `float()` checks), no line copies. A `.trb`
  delta is decoded to its text line in 96 bytes past the read buffer, then
  parsed the same way.
- **Own HTTP client:** a minimal HTTP/1.0 POST over the `tls` module. The
  body goes out in 1 KB pieces and the wait for the answer (up to 30 s,
  `HTTP_WAIT`) polls, both reading the UART meanwhile, so a slow server
  costs no NMEA (the UART buffer holds ~6 s: measured full at 8192 B in a
  6.7 s connect). A connect or a TLS handshake that blocks is read through
  too: the POST guard's timer (once a second, running inside blocking
  socket calls: 13 times in that connect) reads the UART, never inside a
  read already. The name lookup cannot be (lwIP: 7 s without an answer,
  nothing else runs): after a lookup that failed slowly (over 2 s,
  `DNS_SLOW`; a fast "no such name" is only that) the network is not asked
  again for 10 min (`DNS_RETRY`) or until it gives a new IP (a reconnect with
  the same one keeps the mark), for no host name
  (one per name blocked the upload's, NTP's and the OTA's lookup: 21 s): the address
  that same network resolved before is used, or the POST fails at once.
  Another network's address is never used (a LAN address would take the
  API key to a stranger's device on a LAN of the same range). NTP goes before the upload in the loop, so the first
  verified POST without a fix has the board clock set. The upload line on the
  console ends with the POST time. Answer lines are read 256 bytes at a
  time (`HTTP_LINE`); once the status line is in, an error while reading
  the headers (reset, timeout, the guard closing the socket) keeps the
  status, but the head is not whole: a 2xx then is resent (see above), a
  5xx stays a 5xx.
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
  warns that everything goes in the clear. Without `/ca.der` the start
  writes `upload GEO_URL https without /ca.der: the server is not verified,
  uploads off|home only|on` to the diagnostic log, and `/status.json` has `tls`:
  `verified`, `unverified`, `off` (http) or `ca err`. An unverified upload
  shows the Bearer key to whoever can answer for the server: the access
  point of a hotspot, and at home any host of the flat LAN (ARP or DNS),
  whose 2xx also made the board delete tracks it never kept. So by default
  (`UPLOAD_UNVERIFIED=0`) none goes, at home neither; `home` lets one go on
  a home network, `1` everywhere. The `SSLContext` (and the
  certificate) is made once at boot, before the WiFi starts (made at the
  first POST it could grow the Python heap and keep that area), and reused.
  Only a missing `/ca.der` means no verification: one that cannot be read,
  is empty or is not a valid certificate stops the POSTs (state `ca err`,
  `upload ca err …` in the diagnostic log, retried on the doubling backoff,
  the radio asleep in between), never a
  silent fall back. A `/ca.der` added, removed or replaced by one of another
  size is picked up at the next POST; one of the same size takes a reboot.
  mbedTLS checks the certificate's dates and the board clock starts at
  2000-01-01 at every boot, so before a POST the logger sets it from a
  valid GPS fix (a date in 2020–2087): once a boot (`clock set …` in the
  diagnostic log), again only when it drifted more than 5 s
  (`RTC_DRIFT`). With the fix gone and the clock not set yet (a board that
  had a fix outdoors and is indoors at home now) the last fix's clock run on
  by the ticks sets it (`clock set … gps`), with no fix since the boot the
  NTP answer does (`clock set … ntp`; before, verified POSTs failed until a
  fix coincided with one), and a fix sets it again when it differs. An
  https OTA GET sets it first too.
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

## Watchdog and recovery

- A 120 s hardware watchdog is fed in the main loop and inside long reads and
  POSTs.
- An uncaught exception is printed, the batch is flushed (without the drift
  held for a stay), and the board resets after ~1 s.
- The [boot loop guard](usage.md#using-it) stops repeated crashes (and retries
  every 30 minutes); a failing display goes dark instead of crashing.
