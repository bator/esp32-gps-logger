# Hardware

[← README](../README.md)

Flash it with the custom firmware (MicroPython **1.29**, [Setup, step 4](setup.md#4-custom-firmware-recommended)).
The receiver is on the board, so nothing is wired. Measured on a board sold as
V1.2 (Heltec documents up to V1.1; its pins are the same), receiver UC6580I,
firmware R6.0.0.0Build3700.

| Function | GPIO |
|----------|------|
| UC6580 TX → ESP RX / ESP TX → UC6580 RX, 115200 baud (UART1) | 33 / 34 |
| Vext: high powers the UC6580, its LNA and the TFT | 3 |
| Battery voltage: VBAT through 390k/100k (×4.9) / divider on (high) | 1 / 2 |
| TFT (ST7735S, 160×80): SCK / MOSI / CS / DC / RST / backlight | 41 / 42 / 38 / 40 / 39 / 21 |
| Boot bypass → GND (GPIO0, the button, is the download mode) | 7 |
| USER button (full backlight for a minute; held at a reset it is the download mode) | 0 |

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
  fix), the four WiFi bars and what waits for upload. Under the route the battery
  when there is one: an outline filled to its charge, the per cent and about how long
  it lasts (`2 óra`, `45 p`, once `vbat_left_s` is known), amber to 20 %, red to
  10 %. Below, how good the GPS is (a
  tile in its colour with a word) and how full the flash is with how long it still
  logs (a gauge, amber from 70%, red from 90%; the time assumes moving at 1 Hz,
  standing costs next to nothing). The speed is on the status page and in
  `status.json`. It never scrolls. The backlight (the one part that wears) is a 2 kHz PWM: `TFT_LIGHT`
  per cent (default 30) by day, dark once a stay has lasted `TFT_IDLE` seconds (default 300:
  the board has not moved, so nobody looks at it; on the home network with no usable fix
  for 30 s, say on a desk indoors, counts as a stay too), and dim again when it moves. It
  lights at that level when the display starts (after `.env` is read). The USER
  button gives full light for a minute, also in the dark; a tap is enough. A change is a
  `tft light N` line in the diag. The route survives a
  reboot in `/route.bin` (at most one write per 5 min of driving and one when a stay
  starts) and shows again at once; the first fix or NTP answer empties it if it was
  another day, so until then a board switched on the next morning, away from any
  WiFi, shows yesterday's. Before the first fix the clock comes from NTP on any WiFi
  (`NTP_SERVER`), and `--:--` without one; while the receiver is silent it runs on
  from the last fix by the board's ticks. `gps_tft.py`, its
  anti-aliased fonts `gps_tft_font.py` (`tools/make_tft_font.py`, from Space
  Grotesk Bold in `tools/fonts/`, SIL Open Font License 1.1: `tools/fonts/OFL.txt`;
  Pillow 11.0.0 wrote the committed file, and CI checks that it still does).
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
- **L5:** every system and band is on by default (`$CFGSYS` `H35155`), and
  the board's own antenna does pass the L5 band. Heltec's FAQ says it is L1
  only, but GSV per signal (measured 2026-11-14) showed GPS L5, Galileo E5a
  and BeiDou B2a, 8-13 dB below L1. Outdoors, 20-28 dB-Hz against 31-45 dB-Hz
  on L1, with 3-5 satellites per system on L5. In one outdoor A/B/A test,
  12 min per mask, L5 gave no gain: RMS around the phase mean was 1.23 and
  2.03 m with L5, 0.79 m with `GPS_SYS=31111` (L1 only). An external
  antenna on the IPEX socket takes a moved resistor.
