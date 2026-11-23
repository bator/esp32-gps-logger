# Configuration

[← README](../README.md)

- [GPS and points](#gps-and-points)
- [Track files](#track-files)
- [Stays](#stays)
- [WiFi](#wifi)
- [Upload](#upload)
- [System](#system)

Constants at the top of `gps_logger.py`; most never need changing.

## GPS and points

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

## Track files

| Constant | Default | Meaning |
|----------|---------|---------|
| `FLUSH` | 60 s | Interval between flash writes |
| `NEW_FILE_GAP` | 300 s | Fix loss that starts a new file; also the max age of an open file on WiFi |
| `MIN_FREE` | 32 kB | Flash reserve |
| `HIST_RESERVE` | 1 MB | Free flash the day history leaves to the tracks (older days go first) |
| `MAX_BATCH` | 240 | Points kept in RAM while flash writes fail |
| `WFAIL_SWITCH` | 3 | Failed writes in a row to a track file (a bad block) before it is left (cut back to its last good footer, so a partly landed batch is not sent twice) and the points go on in a new file |
| `HOLD_MAX` | 150 | Lines a stay still being proven keeps in RAM |

## Stays

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
| `HOME_HEARTBEAT` | 0 (off) | Anchor point interval at home (`.env` `HOME_HEARTBEAT`, 600–86400 s) |
| `STILL_FILE` | 30 min | Age at which an away stay's file is closed on WiFi |
| `STILL_HOME_HOLD` | 60 s | Wait after a refused home departure |
| `STILL_GAP` | 30 s | Time without a usable fix that counts as a gap |
| `STILL_GAP_STAY` / `STILL_GAP_JUMP` | 60 s / 1 km | After a gap, beyond `STILL_EXIT`: how long `STILL_MIN_FIXES` fixes within `STILL_RADIUS` must span, or one fix's distance, to end the stay |
| `STILL_DEP_MAX` | 90 | Recent samples kept to write the way out |
| `STILL_MEAN_N` | 1800 | Fixes the anchor mean averages over |
| `HOLD_RELEASE` | 10 min | Without a fix this long, what a stay being proven holds is written (at home: dropped) |
| `STAY_WIFI` / `STAY_WIFI_LEARN` / `STAY_WIFI_MAX` | on / 3 / 16 | Away stay ends when its access points are gone: on, scans that learn them, how many kept |
| `STAY_WIFI_MISS` / `STAY_WIFI_GONE` | 6 / 60 s | Scans in a row without any of them, and the least time they span |

## WiFi

| Constant | Default | Meaning |
|----------|---------|---------|
| `CONNECT_TIMEOUT` | 12 s | Per-network wait for an IP (polled) |
| `WIFI_RETRY` / `WIFI_BACKOFF_MAX` | 15 s / 10 min | Reconnect backoff with something to upload |
| `WIFI_SLEEP` / `WIFI_IDLE` | 1 h / 10 s | Radio-off time once all is uploaded; how often that is checked |
| `WIFI_WAKE_WAIT` | 60 s | A new stay away waits this long for its WiFi round |
| `PB_KEEPALIVE` | 5 s | Keep-alive pulse interval while the radio sleeps (`.env` `KEEPALIVE`) |
| `WIFI_OFF_WAIT` | 60 s | An upload backoff at least this long turns the radio off until it is due |
| `ROAM_CHECK` / `ROAM_BACKOFF_MAX` | 5 min / 60 min | Home-roam scans on another network, and their backoff cap |
| `ROAM_MIN_RSSI` | −75 dBm | Minimum home RSSI for a switch (also boot preference, and home range in a scan away from the home link) |
| `SEEN_MIN_RSSI` | −85 dBm | With `WEB`, a home a keep-alive scan sees at least this strong starts a round while the radio sleeps; also home range on the home link and in the scan while the radio sleeps after it |
| `UNLISTED_EVERY` | 4 | A network a good scan did not list is tried every Nth round (hidden ones also whenever the scan shows an empty-SSID AP) |
| `WEB_SEARCH` | 30 min | With `WEB` and no keep-alive scan that can see home (none, or a hidden home not learned yet), failed rounds keep searching this long before sleeping |
| `BLANK_BAD_HOLD` / `HOME_AP_MAX` / `LEARN_RSSI` | 1 h / 4 / 8 dB | Hidden home: a blank AP whose try failed is skipped; learned BSSIDs kept; RSSI match for learning |
| `HOME_AP_MISS` / `HOME_APS_FILE` | 5 / `/home_aps` | Hidden-home connects through other BSSIDs before an unseen learned one is dropped; the file that keeps them across reboots |

## Upload

| Constant | Default | Meaning |
|----------|---------|---------|
| `UPLOAD_BATCH` | 50 | Points per POST |
| `UPLOAD_GAP` / `UPLOAD_GAP_AWAY` | 5 s / 30 s | Pause between POSTs at home / elsewhere |
| `UPLOAD_SCAN` | 4096 | Bytes a batch with no valid point yet reads on past `UPLOAD_BATCH` records (a damaged stretch) |
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

## System

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
| `OTA_PROBE` / `OTA_TRIES_MAX` / `OTA_HOLD` / `OTA_STUCK` | 1 min / 2 / 24 h / 24 h | How often a pending firmware reads the manifest until it did; rollbacks that are not a crash (power-on, no WiFi / manifest, a reset from outside, a panic) before a build is held; how long it is held then (UTC, doubled each round, 32 times at most); how long without a successful upload before waiting points stop holding the update check back |
| `WEB_PAGE_GZ` / `WEB_GZ_WBITS` / `WEB_PAGE` / `WEB_REFRESH` | `/web.html.gz` / 12 / `/web.html` / 15 s | The page on the board, kept only gzipped (as is to clients accepting gzip, inflated for the others); its window (2^12 = 4 KB of heap to inflate; `deploy.sh` and `--publish` use it); an older board's plain page, served while it is there; how often the page polls `/status.json` |
| `KEY_EVERY` | 64 | Delta records between two keyframes of a `.trb` |
| `HIST_DIR` / `HIST_DAYS` / `HIST_REPLY` | `/hist` / 7 / 16 KB | The day history for the page's map (a `.trb` per UTC day), days kept, text bytes about per `/hist` reply (and bytes at most per `/diag` reply) |
| `WEB_AWAY` | off | The page served, and the radio kept on for it, on non-home networks too (`.env` `WEB_AWAY`) |
| `SV_POLL` | 60 s | A satellite list younger than two of these gives the used count (else GGA) |
| `GSA_EVERY` / `GSA_MAX` | 5 s / 8 | GSA round interval; more GSA a second means the rate was not taken |
| `SEEN_WAIT` | 60 s | Least time between home-seen wakes |
| `WEB_REQ_MAX` / `WEB_CHUNK` | 2 KB / 1 KB | Request head read by the status page; reply bytes per send |
| `DEVICE_ID` / `HOSTNAME` | `Dawarich-GPS-tracker` | Overland id and DHCP hostname (set in `.env`) |
