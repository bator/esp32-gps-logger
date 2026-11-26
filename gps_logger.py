from machine import Pin, WDT
import machine, sys, time, os, network, gc, math, socket, json, select, struct
import hashlib, binascii
try:
    import esp32                         # mcu_temperature(), OTA partitions
except ImportError:
    esp32 = None
try:
    __import__("gps_frozen")             # firmware of tools/build_firmware.sh:
    FROZEN = True                        # this module's bytecode is in flash
except ImportError:
    FROZEN = False
from gps_clock import (days as _days, civil as _civil, ymd_ok as _ymd_ok,
                       tc_ok as _tc_ok, dt_s as _dt_s, stamp_s as _stamp_s,
                       stamp as _stamp, age_s as _age_s, recent_s as _recent_s,
                       utc_s as _utc_s,
                       age as _age, recent as _recent, waiting as _waiting,
                       EPOCH_DAYS, FIX_S_END)

# Collect garbage early and often. When an allocation doesn't fit, the ESP32
# port grows the Python heap with the largest free IDF block (~60 kB) instead,
# and TLS (mbedTLS lives in the IDF heap) then fails with ENOMEM.
gc.threshold(8192)

GPS_MAX_STD = 20                         # m: receiver's own error estimate
                                         # (GST) above this = not usable
CPU_MHZ = 80                             # CPU clock in the main loop
                                         # (20/40/80/160/240); 0 leaves the
                                         # firmware default
CPU_MHZ_UPLOAD = 160                     # clock during a POST (TLS); 0 = no
                                         # change. .env CPU_MHZ / CPU_MHZ_UP
STATUS = 600000                          # ms between console status lines
DIAG = 60000                             # ms between diagnostic log lines
                                         # (0 = off; .env DIAG=<seconds>)
DIAG_FILE = "/diag.log"                  # survives power-off: what happened
DIAG_OLD = "/diag.1"                     # on a walk, read it back over USB
DIAG_MAX = 163840                        # bytes, then DIAG_FILE -> DIAG_OLD:
                                         # the two ~26 h of ~210 B lines a
                                         # minute (the page reads both)
DIAG_LINE = 220                          # bytes read back per line wanted
DIAG_CUT = 320                           # a longer diag line is cut (the
                                         # status line runs to ~220 B)
                                         # (status lines are ~180)
DIAG_BLOCK = 512                         # bytes per read of the tail
WEB = True                               # status page on port WEB_PORT; the
WEB_PORT = 80                            # radio stays on at home for it
                                         # (.env WEB=0)
WEB_TIMEOUT = 2                          # s one recv/send may wait
WEB_DEADLINE = 5                         # s a whole request may take
WEB_HEAD_WAIT = 1                        # s the request head may take (a
                                         # browser sends it at once; no NMEA
                                         # is read while recv waits)
WEB_REQ_MAX = 2048                       # bytes of a request head read
WEB_CHUNK = 1024                         # reply bytes per send
WEB_REFRESH = 15                         # s between status.json polls of
                                         # the page (WEB_PAGE_GZ)
WEB_PAGE_GZ = "/web.html.gz"             # the status page, kept only gzipped
                                         # (deploy.sh, OTA): sent as is to a
                                         # client that accepts gzip, inflated
                                         # for one that does not (_web_page)
WEB_GZ_WBITS = 12                        # its window: 2**12 = 4 KB of heap to
                                         # inflate it (gzip -9's 32 KB do not
                                         # fit; +11% size). deploy.sh and
                                         # --publish read it from here
WEB_PAGE = "/web.html"                   # the plain page of a board deployed
                                         # before: served as is while there
WEB_DIAG = 15                            # diag.log lines on the page
WEB_TOKEN = None                         # .env WEB_TOKEN: the X-GPS header
                                         # status.json, /hist, /diag need
WEB_TOKEN_LEN = (8, 64)                  # its length, at least and at most
_web_tok_bad = [False]                   # one in .env refused: all closed
WEB_AWAY = False                         # .env WEB_AWAY=1: keep the radio on
                                         # for the page on the other networks too

# The Heltec Wireless Tracker (ESP32-S3, boards/GPS_LOGGER_S3): its UC6580 is on
# the board, UART1 on GPIO34 (TX) / GPIO33 (RX), powered by the Vext rail (GPIO3
# high: the receiver, its LNA and the TFT). gps_tft.py drives the TFT once the
# logger runs (dark until then). main.py keeps the boot-bypass pin.
HELTEC_VEXT = 3                          # high: the rail of the UC6580
HELTEC_TFT_BL = 21                       # the TFT's backlight (given to gps_tft.start)
Pin(HELTEC_VEXT, Pin.OUT, value=1)
Pin(HELTEC_TFT_BL, Pin.OUT, value=0)
# The battery (SH1.25-2; the board charges it and switches between USB and
# it): VBAT through 390k/100k to GPIO1 (ADC1_CH0), GPIO2 high switches the
# divider on (left on: 8.6 uA at 4.2 V). Measured on USB without a battery
# (2026-11-15, 12 rounds): the charger's output swings, 32 reads 3.01-4.29 V,
# median 3.50-3.60 V, 10th-90th percentile spread 0.84-0.94 V, so a wide
# spread is no battery, not its voltage (_vbat). A round takes ~5 ms.
HELTEC_VBAT = 1                          # ADC1_CH0: VBAT / VBAT_DIV
HELTEC_VBAT_CTL = 2                      # high: the divider on
VBAT_DIV = 4.9                           # (390k + 100k) / 100k
VBAT_N = 32                              # reads a measurement
VBAT_SWING = 0.5                         # V of 10th-90th percentile spread
                                         # above which there is no battery
VBAT_AGAIN = 2                           # measurements in a row that turn the
                                         # verdict (battery / none): one stray
                                         # one is let go, its values not used
VBAT_EVERY = 60                          # s between samples of the time left
VBAT_HIST = 31                           # samples kept: 30 min
VBAT_SPAN = 600                          # s of samples before a time left
VBAT_DROP = 2                            # %-points the fit must fall over them
VBAT_RISE = 3                            # %-points the mean of the last
VBAT_RISE_N = 5                          # this many samples lies over the
                                         # mean of the ones before: the
                                         # charger topping up on USB under
                                         # VBAT_JUMP_MV (+3 points are 12-36
                                         # mV along VBAT_OCV, 33-36 at its top)
VBAT_RISE_HOLD = 3600                    # s without a time left after it
                                         # (the settling read as ~15 h left)
VBAT_JUMP_MV = 45                        # mV off the last samples' median (their
                                         # per cent back on VBAT_OCV, _vbat_mv)
                                         # that are the charger, not noise. In
                                         # mV: the curve's points are 4-12 mV,
                                         # and 4 of them (the jump's old size)
                                         # were 16 mV where the cell holds out,
                                         # up to 3 resets an hour at the board's
                                         # noise (6.5 mV sigma, 2026-11-25); at
                                         # 45 none at 12 mV of noise there
                                         # (simulated). The unplug step was 60
                                         # mV (4.07 V on USB, 4.01 V on its own;
                                         # 67 back on VBAT_OCV), the charger's
                                         # lift 180 mV. Whole mV: the device's
                                         # single precision decided the pairs
                                         # exactly 45 mV apart otherwise
VBAT_OCV = (4.13, 4.02, 3.90, 3.80, 3.71, 3.61, 3.55, 3.50, 3.46, 3.41, 3.30)
# V of the cell (602535, 650 mAh) at 100, 90 ... 0 %, linear between, read
# under the board's own load: the discharge of 2026-11-25 (at home on WiFi,
# a 3D fix, 80 MHz; 4.01 -> 3.42 V in 189 min, a sample a minute,
# tests/vbat_discharge_2026-11-25.json), the per cent in proportion to the
# time (a steady draw): 90 % at the unplug, 0 % 219 min after it. Meshtastic's
# resting OCV_ARRAY it replaces read 8 points low the minute the USB came off
# (the load's 60 mV) and up to 20 behind the time, at about 3.6 V. Not
# measured, estimated: 100 % (4.19 V at rest less the 60 mV), the 90 % where
# the test began (4.07 V at rest after a charge) and its end, 0 % at 3.30 V
# (where the board stops, Heltec's datasheet: 3.3 V at least) 219 min in.
# The test ended on the charger at 3.42 V at 189 min; 219 assumes the slope
# doubles under 3.41 V (a LiPo's knee). Every row scales with that end: on
# the end slope carried on (0.13 V/h, 0 % at ~240 min) the rows under 90 %
# would be up to 30 mV lower. Off WiFi the draw is lower, so it reads a
# little high there; on USB the board's load is off the cell, so it reads ~5
# points high (4.07 V at rest is 95 %), more while the charger lifts it
Pin(HELTEC_VBAT_CTL, Pin.OUT, value=1)

# The receiver (gps_receiver.py): its UART, the NMEA, the seconds it publishes
# in st. It writes diag lines and feeds the watchdog through what is bound here.
import gps_receiver as _rcv
from gps_receiver import st, _dlon
_rcv._diag_write = lambda line: _diag_write(line)   # (late: tests replace them)
_rcv.feed = lambda: feed()

# A Fix is a published second as a tuple in _rcv._SNAP's order (_rcv.drain);
# None stands for the latest one, in st
_FX = {k: i for i, k in enumerate(_rcv._SNAP)}

def _fv(fx, k):
    """Field k of the Fix fx, or of the latest second (st) for None."""
    return st.get(k) if fx is None else fx[_FX[k]]


SAMPLE = 1000                            # ms between sampled positions
MIN_MOVE = 1                             # m from the last recorded point (.env
                                         # MIN_MOVE 0-50; 3 m gave a point
                                         # every ~3 s on foot)
FLUSH = 60000                            # ms between batch writes to flash
NEW_FILE_GAP = 300                       # s without fix → new file; also WiFi max age
# Stationary filter, on any network. Indoors the GPS wanders tens of metres
# and reports 5-20 km/h while lying still (measured on the board: speed p50
# 5.4 km/h, drift p90 46 m, minute centroids up to 72 m apart, excursions to
# 80 m), so speed is not used: a stay is two consecutive STILL_WINDOW
# centroids within STILL_RADIUS, and it ends only on samples that keep
# receding beyond STILL_EXIT, well past the drift.
STILL_WINDOW = 60000                     # ms per centroid window
STILL_MIN_FIXES = 10                     # usable fixes a window needs
STILL_WINDOW_MAX = 300000                # ms a sparse window may stay open
                                         # to collect them (indoors)
STILL_RADIUS = 50                        # m between consecutive centroids
STILL_NEAR = 50                          # m: fixes this close refine the anchor
STILL_EXIT = 150                         # m from the anchor to start leaving
STILL_EXIT_N = 3                         # receding samples needed to leave
STILL_TOL = 5                            # m a receding sample may fall back
STILL_STEP = 2                           # m each receding sample must gain
                                         # (a frozen multipath jump gains 0)
STILL_EXIT_SLOW = 10                     # samples beyond STILL_EXIT that leave
                                         # without gaining (a slow walk away)
STILL_SLOW_GAIN = 5                      # m those samples must gain in all
                                         # (a jump frozen for 10 s gains 0)
STILL_FILE = 1800000                     # ms an away stay's file stays open
STILL_HEARTBEAT = 300000                 # ms between anchor points (not home)
HOME_HEARTBEAT = 0                       # ms between anchor points at home,
                                         # 0 = none (.env HOME_HEARTBEAT=<s>):
                                         # a home stay is a visit then, and a
                                         # board gone quiet shows within it
STILL_BACK = 30000                       # ms back within STILL_NEAR before a
                                         # due anchor point is written
STILL_MOVE_HITS = 3                      # matching window pairs that move
                                         # the anchor (a two-window multipath
                                         # shift moved it twice in 2 h)
STILL_HOME_HOLD = 60000                  # ms a departure from home waits
                                         # after the home WiFi was in range
STILL_GAP = 30000                        # ms without a usable fix that counts
                                         # as a gap (the device may have moved)
STILL_GAP_STAY = 60000                   # ms: back after a gap out past
                                         # STILL_EXIT, STILL_MIN_FIXES fixes
                                         # within STILL_RADIUS of the first
                                         # over this long end the stay (a
                                         # multipath jump lasts seconds)
STILL_GAP_JUMP = 1000                    # m: one fix this far after a gap ends
                                         # it (no multipath jump reaches it)
STILL_DEP_MAX = 90                       # recent samples kept for the departure
STILL_MEAN_N = 1800                      # samples the anchor mean averages over
STAY_WIFI = True                         # an away stay ends when none of the
STAY_WIFI_LEARN = 3                      # access points of its first scans
STAY_WIFI_MAX = 16                       # (strongest kept) shows up in
STAY_WIFI_MISS = 6                       # this many scans in a row over at
STAY_WIFI_GONE = 60000                   # least this many ms (no fix needed)
GST_HOLD = 6000                          # ms a GST error estimate is used
                                         # for the seconds without one (1 Hz:
                                         # GST only every 5th second)
LOG_DIR = "/gpx"
HIST_DIR = "/hist"                       # the day history the page draws:
HIST_DAYS = 7                            # each written line, a .trb a UTC
                                         # day, the last HIST_DAYS kept
HIST_REPLY = 16384                       # text bytes about per /hist reply
HIST_SEEK_MS = 1500                      # ms a /hist?t= seek or a day's span
                                         # scans its keyframes at most
MIN_FREE = 32768                         # bytes of flash kept free
HIST_RESERVE = 1048576                   # bytes kept free from the day
                                         # history: older days go first
MAX_BATCH = 240                          # RAM cap while flash writes fail
WFAIL_SWITCH = 3                         # failed writes in a row to a track
                                         # file before it is left (a bad
                                         # block) for a new one
HOLD_RELEASE = 600000                    # ms without a fix after which what a
                                         # stay being proven holds is written
HOLD_MAX = 150                           # lines a stay being proven keeps in
                                         # RAM (two windows and some; a FLUSH
                                         # more still fits under MAX_BATCH)
MAX_HDOP = 5                             # skip samples with worse dilution
HDOP_M = 5                               # metres of UERE per HDOP unit
FULL_CHECK = 5000                        # ms between statvfs polls when full

ENV_FILE = "/.env"                       # SSID, WPA2, GEO_URL, API_KEY lines
WIFI_CHECK = FLUSH                       # ms between checks while connected
WIFI_RETRY = 15000                       # ms between reconnects while down
WIFI_BACKOFF_MAX = 600000                # ms cap on wifi reconnect backoff
WIFI_SLEEP = 3600000                     # ms the radio stays off once all is
                                         # uploaded (then on only with points
                                         # to upload)
WIFI_IDLE = 10000                        # ms between "all uploaded?" checks
PB_KEEPALIVE = 5000                      # ms between power bank keep-alive
                                         # pulses while the radio sleeps (0 =
                                         # off; .env KEEPALIVE=<seconds>).
                                         # 15 s kept the 20 Ah bank on for
                                         # 3 h, not for good: it averages
                                         # A pulse is the radio on, one scan, off
                                         # (~2 s at ~100 mA: +40 mA average at 5 s)
SEEN_WAIT = 60000                        # ms: a keep-alive scan that sees home
                                         # wakes the radio at most this often
                                         # (doubling while rounds fail)
SEEN_MIN_RSSI = -85                      # dBm: a home AP this strong wakes the
                                         # sleeping radio (ROAM_MIN_RSSI is
                                         # for switching: a -78 dBm home that
                                         # connects fine never woke it)
WIFI_OFF_WAIT = 60000                    # ms: an upload backoff at least
                                         # this long turns the radio off
                                         # until it is due (server down, 401)
WIFI_WAKE_WAIT = 60000                   # ms a new stay waits for the WiFi
                                         # round before its first anchor point
CONNECT_TIMEOUT = 12000                  # ms to wait for GOT_IP per network
FAIL_HOLD = 120000                       # ms to demote a network that failed
CLEAR_WAIT = 500                         # ms to wait for STAT_IDLE after clear
LOST_GRACE = 5000                        # ms to let IDF auto-reconnect after drop
ROAM_CHECK = 300000                      # ms between home-roam scans on hotspot
ROAM_BACKOFF_MAX = 3600000               # ms cap on roam retry (60 min)
ROAM_MIN_RSSI = -75                      # do not jump to a weak home AP
WIFI_DIAG_GAP = 300000                   # ms between diag lines of one kind
                                         # (lost / fail / back): the rest
                                         # counted into the next one
BLANK_BAD_HOLD = 3600000                 # ms a blank AP whose hidden-home
                                         # try failed is not tried again
HOME_AP_MAX = 4                          # BSSIDs of the hidden home learned
                                         # while connected to it (kept in
                                         # HOME_APS_FILE across reboots)
HOME_AP_MISS = 5                         # hidden-home connects through other
                                         # BSSIDs before an unseen one is
                                         # dropped (a wrong one cannot stick)
HOME_APS_FILE = "/home_aps"              # learned BSSIDs, hex, one per line
LEARN_RSSI = 8                           # dB: the one blank AP on the link's
                                         # channel this close to its RSSI
UNLISTED_EVERY = 4                       # rounds: a network a good scan did
                                         # not list is tried every Nth round
WEB_SEARCH = 1800000                     # ms: WEB on and no scan keep-alive:
                                         # failed rounds search on (backoff)
                                         # this long before the radio sleeps
FREQ_TRIES = 3                           # machine.freq() tries per change
FREQ_RETRY = 1800000                     # ms a refused clock is left alone,
                                         # then tried again
LINE_MAX = 512                           # bounded track-line readline
BOOT_OK_MS = 60000                       # clear boot-fail after this of _loop

UPLOAD_BATCH = 50                        # points per POST to Dawarich
BODY_CAP = UPLOAD_BATCH * 256 + 32       # JSON buffer; sized for long device_id
UPLOAD_GAP = 5000                        # ms between successful batches
UPLOAD_GAP_AWAY = 30000                  # ms between batches on a non-home net
BACKOFF_START = 15000                    # ms first retry after net/5xx/4xx
BACKOFF_MAX = 1800000                    # ms cap (30 min) on upload backoff
AUTH_BACKOFF = 1800000                   # ms retry after 401 (plan/key/limit)
HTTP_TIMEOUT = 15                        # s per socket op (connect, TLS)
HTTP_WAIT = 30000                        # ms for the server's answer, the
                                         # UART read meanwhile (its 8 KB buffer
                                         # holds ~3.5 s of NMEA)
HTTP_CHUNK = 1024                        # body bytes per write, UART between
HTTP_LINE = 256                          # bytes per read of an answer line
HTTP_LINES = 64                          # reads of the answer's head at most
HTTP_MAX = 60000                         # ms wall clock for one POST: a timer
                                         # feeds the WDT meanwhile and then
                                         # closes the socket (a hung TLS call
                                         # reset the board by the WDT)
CA_FILE = "/ca.der"                      # if present: verify the server
                                         # (unreadable or empty: no POST)
TLS_MIN_KB = 48                          # largest IDF heap block a TLS POST
                                         # needs (measured: at 20 KB every
                                         # handshake failed); below it no
                                         # try (gc.collect() and wait)
LOWIDF_RETRY = 30000                     # ms between checks while it is low
LOWIDF_WAIT = 300000                     # ms low at home with tracks waiting:
                                         # then a clean reboot
LOWIDF_REBOOT_GAP = 1800000              # ms: at most one such reboot this
                                         # often (LOWIDF_MARK spans the boot)
LOWIDF_MARK = "/lowidf"                  # written just before that reboot
TLS_MIN_PY_KB = 8 if FROZEN else 12      # free Python heap a POST needs after
                                         # gc.collect(); below it the POST
                                         # waits (LOWIDF_RETRY): the socket
                                         # and TLS objects would grow the
                                         # heap into the IDF block TLS needs.
                                         # The SSLContext (~3 KB) is made at
                                         # boot, a POST itself takes a few KB
                                         # (sockets, header lines); frozen,
                                         # no module bytecode fragments the
                                         # heap, so 8 KB leaves room
RTC_DRIFT = 5                            # s the board clock may be off the GPS
                                         # time before it is set again
                                         # (mbedTLS checks certificate dates)
TLS_REBOOT_N = 5                         # TLS out of memory in a row (at
                                         # home): a clean reboot, the IDF heap
                                         # stays fragmented otherwise
READ_ERR_MAX = 3                         # unreadable track: tries, then .err
                                         # (unreadable .pos: then from 0)
POS_READ = 40                            # bytes of a .pos read at most
DEFAULT_DEVICE_ID = "Dawarich-GPS-tracker"
DEFAULT_HOSTNAME = "Dawarich-GPS-tracker"
DEVICE_ID = DEFAULT_DEVICE_ID            # Overland tracker id (overridable)
HOSTNAME = DEFAULT_HOSTNAME              # DHCP hostname (overridable)
ID_MAX = 32                              # DEVICE_ID / HOSTNAME length cap
MAX_BAD_BYTES = 65536                    # cap on appended <file>.bad
MAX_BAD_RETRY = 8                        # rejected .bad batches per file per boot
MAX_GLOBAL_PAYLOAD = 3                   # consecutive 400/422 -> global backoff
WDT_TIMEOUT = 120000                     # ms; ESP32 WDT cannot be disarmed
KEEPALIVE_MAX = 600000                   # ms Ctrl-C timer may feed the WDT
PAYLOAD_ERR = (400, 413, 422)            # only these quarantine a batch
UP_WAIT_MAX = max(BACKOFF_MAX, AUTH_BACKOFF)  # longest upload/hold deadline

def _temp():
    try:
        return esp32.mcu_temperature()
    except Exception:
        return None

_freq_bad = []                           # clocks this board refused
_freq_bad_at = [None]                    # Stamp of the last refusal

def _set_freq(mhz):
    """CPU clock: 80 MHz saves current against 240 (the UART
    runs off the fixed APB clock, WiFi needs 80 or more). A failed change
    is tried again (FREQ_TRIES in all): one passing error blacklisted the
    clock for good, and a failed drop-back after a POST left the board at
    the upload clock. Refused every time: left alone for FREQ_RETRY (a
    console line each time), then tried again: one passing failure pinned
    the board at the upload clock until reboot."""
    if not mhz:
        return
    if mhz in _freq_bad:
        if _recent_s(_freq_bad_at[0], FREQ_RETRY // 1000):
            return
        del _freq_bad[:]                 # all of them get a new chance
    for _ in range(FREQ_TRIES):
        try:
            if machine.freq() != mhz * 1000000:
                machine.freq(mhz * 1000000)
            return
        except Exception as e:
            err = e
        time.sleep_ms(10)
    _freq_bad.append(mhz)
    _freq_bad_at[0] = _stamp()
    print("cpu freq", mhz, repr(err))

def _mhz_env(env, key, default):
    v = _plain(env.get(key))
    if not v:
        return default
    try:
        n = int(v)
    except ValueError:
        _env_refused(key, "invalid", default)
        return default
    allowed = (20, 40, 80, 160, 240)
    if n == 0 or n in allowed:
        return n
    _env_refused(key, "not 0 or %s on this board" % "/".join(
        str(m) for m in allowed), default)
    return default

def _words():
    """What the console status line and the diag line both end with: HDOP,
    points logged/sent, WiFi state, keep-alive pulses, free Python heap,
    IDF heap free/largest block (KB), Python heap size (KB: it doubles
    into the IDF heap when an allocation does not fit)."""
    fr, big = idf_heap()
    pf, pk = py_heap()
    return "hdop %s points %d sent %d wifi %s pulses %d mem %d idf %s/%s heap %d" % (
        st["hdop"], log["points"], up["sent"], wifi["state"].replace(" ", "_"),
        wifi["pb_scans"], pf, fr, big, pk)

def _status():
    """One console line: chip temperature (the die, not the GPS module),
    fixes in the last second, satellites, reset loops, then _words."""
    print("status temp", _temp(), "hz", st["epochs"], "used", st["used"],
          "gpsloops", st["loops"], _words())

def _diag_env(env):
    """DIAG=<seconds> in .env: 0 turns the diagnostic log off, otherwise
    10-3600 s between lines; anything else keeps the default."""
    global DIAG
    v = _plain(env.get("DIAG"))
    if not v:
        return
    try:
        n = int(v)
    except ValueError:
        n = -1
    if n == 0:
        DIAG = 0
    elif 10 <= n <= 3600:
        DIAG = n * 1000
    else:
        _env_refused("DIAG", "not 0 or 10-3600 s", "%d s" % (DIAG // 1000))

def _diag_write(line):
    """Append one line to DIAG_FILE (closed again, so a power-off loses at
    most the line being written). Past DIAG_MAX the file becomes DIAG_OLD,
    so both together stay under 2 * DIAG_MAX. A line break in it becomes a
    space and a line is cut at DIAG_CUT (a field from the network, the OTA
    manifest's commit, wrote forged lines and ~4 KB ones). Never raises.
    Not re-entered: the POST guard's timer reads the UART inside a write
    (a receiver line to say), and its rotation between the remove and the
    rename was undone by the outer one (160 KB of diag lost): a line said
    meanwhile waits (DIAG_PEND at most) and goes after it."""
    if not DIAG:
        return
    if _diag_busy[0]:
        if len(_diag_pend) < DIAG_PEND:
            _diag_pend.append(line)
        else:
            _diag_errs[0] += 1
        return
    _diag_busy[0] = True
    try:
        # (lines left waiting by an earlier call first: one said in the
        # few bytecodes after its drain stayed there, then went after a
        # newer line, or with a power-off not at all)
        while _diag_pend:
            _diag_one(_diag_pend.pop(0))
        _diag_one(line)
        while _diag_pend:
            _diag_one(_diag_pend.pop(0))
    finally:
        _diag_busy[0] = False

DIAG_PEND = 4                            # lines said inside a write, kept
_diag_busy = [False]
_diag_pend = []

def _diag_one(line):
    try:
        if "\n" in line or "\r" in line:
            line = line.replace("\r", " ").replace("\n", " ")
        if len(line) > DIAG_CUT:
            line = line[:DIAG_CUT - 3] + "..."
        room = free()
        if room is not None and room < MIN_FREE:
            return                       # the tracks keep the reserve
        # (a stat EIO raises, counted: taken for "not there" the line went
        # on the end of a full file, past DIAG_MAX into the tracks' flash)
        size = _size_or_gone(DIAG_FILE)
        if size is not None and size >= DIAG_MAX:
            _rm(DIAG_OLD)
            try:
                os.rename(DIAG_FILE, DIAG_OLD)
            except OSError as e:
                # (raised, every later line was lost): the full file goes,
                # a line says so; one that cannot go either is no file to
                # grow past DIAG_MAX into the tracks' flash: counted
                _diag_errs[0] += 1
                _rm(DIAG_FILE)
                if _size_or_gone(DIAG_FILE) is not None:
                    return
                line = "bug diag rotate failed %r, %d KB dropped\n%s" % (
                    e, size // 1024, line)
        f = open(DIAG_FILE, "a")
        try:
            f.write(line + "\n")
        finally:
            f.close()
    except Exception as e:
        _diag_errs[0] += 1               # (status.json, the next diag line)
        print("diag", repr(e))

_diag_errs = [0]                         # diag lines that failed to write

def _diag_utc(fx=None):
    t = _fix_s(fx)
    return "-" if t is None else _stamp_s(t)[1]

def _diag_tail(path, n):
    """The last n lines of a diag file, [] if none. Reads only its last
    DIAG_LINE * n bytes (status lines are ~180 B), in DIAG_BLOCK pieces:
    one 3 KB read needs a free block that big, and the heap may not have
    one (it then grows into the IDF heap TLS needs). A line that is not
    UTF-8 is left out."""
    out = []
    try:
        size = _file_size(path) or 0
        start = size - DIAG_LINE * n
        with open(path, "rb") as f:
            if start > 0:
                f.seek(start - 1)        # the byte before: a line that
            part = b""                   # starts at start is a whole one
            cut = start > 0              # up to the first "\n": partial
            while True:
                b = f.read(DIAG_BLOCK)
                if not b:
                    break
                lines = (part + b).split(b"\n")
                part = lines.pop()
                for line in lines:
                    if cut:
                        cut = False
                    elif line:
                        try:
                            out.append(line.decode())
                        except UnicodeError:
                            continue
                        if len(out) > n:
                            out.pop(0)
        if part and not cut:
            try:
                out.append(part.decode())
            except UnicodeError:
                pass
        return out[-n:]
    except OSError:
        return out[-n:]

# Whole seconds since boot: the current Stamp (gps_clock sums the ticks up,
# the main loop calls it so ticks_ms never wraps unseen)
uptime_s = _stamp

# How long the run before this boot lasted, from the last diag lines it
# wrote: up (s, to within DIAG; 0 = it ended before its first status line),
# end (GPS time of its last line, "2026-10-28 12:19:42Z") and how
# ("status", "crash" or "short"). Read once at boot.
prev_run = {"up": None, "end": None, "how": None}

DIAG_PREV = (8, 40)                      # last lines of DIAG_FILE searched
                                         # for the run's end, then more

def _diag_prev():
    prev_run.update(up=None, end=None, how=None)
    # DIAG_OLD only when DIAG_FILE was read whole without the run's end (a
    # rotation leaves the new file with the line that caused it, which need
    # not be a status line): its tail is older than all of DIAG_FILE, and
    # behind 8 event lines a run's last status line read as a run of days
    # before. More events than DIAG_PREV's last: unknown
    size = _file_size(DIAG_FILE) or 0
    for n in DIAG_PREV:
        lines = _diag_tail(DIAG_FILE, n) or []
        if _diag_prev_in(lines):
            return
        if len(lines) < n and size <= DIAG_LINE * n:
            break                        # (all of it was read)
    else:
        return
    _diag_prev_in(_diag_tail(DIAG_OLD, DIAG_PREV[0]) or [])

def _diag_prev_in(lines):
    """prev_run from the last status, crash or boot line in lines: True
    when one was found."""
    for line in reversed(lines):
        f = line.split()
        try:
            if line.startswith("up "):
                prev_run["up"], prev_run["how"] = int(f[1]), "status"
                t = f[3] if len(f) > 3 and f[2] == "utc" else "-"
                if len(t) == 16:
                    prev_run["end"] = "%s-%s-%s %s:%s:%sZ" % (
                        t[0:4], t[4:6], t[6:8], t[9:11], t[11:13], t[13:15])
                return True
            if line.startswith("crash up "):
                prev_run["up"], prev_run["how"] = int(f[2]), "crash"
                return True
            if line.startswith("crash boot"):
                # (gps_boot / main.py: the logger failed to start, an OTA
                # build that would not import; the run before it is not
                # the previous one)
                prev_run["up"], prev_run["how"] = 0, "crash"
                return True
            if line.startswith("boot "):
                prev_run["up"], prev_run["how"] = 0, "short"
                return True
        except (ValueError, IndexError):
            continue
    return False

def _hms(secs):
    """Seconds as '45 s', '12 min 05 s', '3 h 04 min', '2 d 03 h'."""
    if secs is None:
        return "-"
    secs = int(secs)
    if secs < 60:
        return "%d s" % secs
    if secs < 3600:
        return "%d min %02d s" % (secs // 60, secs % 60)
    if secs < 86400:
        return "%d h %02d min" % (secs // 3600, secs // 60 % 60)
    return "%d d %02d h" % (secs // 86400, secs // 3600 % 24)

def _diag_boot():
    """First line after a boot: why the chip (re)started, machine.reset_cause()
    of MicroPython 1.29 (1 power-on, also a brownout or a power bank
    switching back on; 2 hard reset: machine.reset(), also a panic; 3 WDT;
    4 deep sleep; 5 soft reset; 0 other, e.g. USB-JTAG) and how long the run
    before lasted (prev_up s, to within DIAG; from that run's last lines,
    read first)."""
    _diag_prev()
    try:
        cause = machine.reset_cause()
    except Exception:
        cause = None
    _diag_write("boot cause %s prev_up %s mhz %d mem %d" % (
        cause, "-" if prev_run["up"] is None else prev_run["up"],
        machine.freq() // 1000000, py_heap()[0]))

def _idf_info():
    """(free B, largest free block B) of the ESP-IDF heap, or (None,
    None)."""
    try:
        fr = big = 0
        for h in esp32.idf_heap_info(esp32.HEAP_DATA):
            fr += h[1]
            if h[2] > big:
                big = h[2]
        return fr, big
    except Exception:
        return None, None

def idf_heap():
    """(free KB, largest free block KB) of the ESP-IDF heap, or (None,
    None). TLS needs ~40 KB in one block from it; the Python heap grows
    into it (and keeps it) when a big allocation does not fit."""
    fr, big = _idf_info()
    if fr is None:
        return None, None
    return fr // 1024, big // 1024

def py_heap():
    """(free B, size KB) of the Python heap itself. gc.mem_free() adds
    the largest free IDF block in bytes (room the heap could grow into:
    modgc.c in MicroPython 1.29), which hid the heap doubling into the
    IDF heap; taken off to the byte (a KB figure left up to 1 KB)."""
    big = _idf_info()[1] or 0
    free = max(gc.mem_free() - big, 0)
    return free, (gc.mem_alloc() + free) // 1024

_heap_kb = [None]                        # Python heap KB at the last note

def _heap_mark():
    _heap_kb[0] = py_heap()[1]

def _heap_note(phase):
    """One diag line when the Python heap grew (by 2 KB or more: the two
    gc figures and the IDF block are not read at one instant) since the
    last mark or note: which step of an upload grows it into the IDF
    heap."""
    try:
        k0, k = _heap_kb[0], py_heap()[1]
        _heap_kb[0] = k
        if k0 is not None and k >= k0 + 2:
            _say("heap grew in %s to %d KB" % (phase, k))
    except Exception:                    # MemoryError: a note never fails
        pass                             # the upload

def _diag(now):
    """One line every DIAG ms: uptime, GPS time, NMEA sentences good/bad,
    receiver restarts and reset-loop warnings, satellites seen/used, fix,
    then _words, the battery "vbat 3.95/0.04" (median/spread V); with a
    guarded tick's bug since boot "bugs upload:12" (the
    count each, _tick_bugs) and with a failed diag write "diag_errs 3"."""
    b = _tick_bugs(",", False)
    vb, vw = _vbat()
    _diag_write(
        "up %d utc %s nmea %d/%d gpsboots %d loops %d sats %d/%s fix %s %s%s%s%s" % (
            uptime_s(now), _diag_utc(), st["ok"], st["err"], st["gps_boots"],
            st["loops"], _rcv.sv_seen(), _rcv.sv_used(), status().replace(" ", "_"),
            _words(), "" if vb is None else " vbat %.2f/%.2f" % (vb, vw),
            "" if b is None else " bugs " + b,
            " diag_errs %d" % _diag_errs[0] if _diag_errs[0] else ""))

def _diag_final():
    """A status line right before a deliberate reset: the next boot reads
    the run's length and end from it (_diag_prev). The last regular one
    can be up to DIAG older, and an OTA install takes ~1.5 min after it:
    the page showed the run before an update 1 min 05 s long, it was 2.5."""
    try:
        _diag(time.ticks_ms())
    except Exception:
        pass

# --- Watchdog ---------------------------------------------------------------
# Created only from run()/_loop(), so the boot-bypass REPL (main.py's
# SKIP_PIN, GPIO7) is not reset. ESP32 cannot disarm it: Ctrl-C starts a Timer that keeps feeding until
# KEEPALIVE_MAX, then the dog is allowed to reset.

_wdt = None
_keep = None
_keep_from = 0
_running = False
_boot_ok_done = False

def _boot_ok():
    """Clear the boot-fail counter after a stretch of healthy looping."""
    global _boot_ok_done
    if _boot_ok_done:
        return
    _boot_ok_done = True
    try:
        machine.RTC().memory(b"0")
    except Exception:
        pass
    try:
        os.remove("/boot_fail")
    except OSError:
        pass

def _boot_fail_bump():
    n = 0
    try:
        for c in machine.RTC().memory():
            if not 48 <= c <= 57:
                break
            n = n * 10 + c - 48
        n += 1
        machine.RTC().memory(b"%d" % n)
    except Exception:
        pass
    return n

def feed():
    if _wdt is not None:
        _wdt.feed()

def _keep_stop():
    global _keep
    t = _keep
    _keep = None
    if t is None:
        return
    try:
        t.deinit()
    except Exception:
        pass

def _keep_cb(timer):
    if _running:
        return
    if time.ticks_diff(time.ticks_ms(), _keep_from) > KEEPALIVE_MAX:
        _keep_stop()
        return                           # stop feeding: WDT resets the chip
    feed()

def _wdt_keepalive():
    """Feed the WDT after Ctrl-C, but only for KEEPALIVE_MAX ms."""
    global _keep, _keep_from
    if _wdt is None:
        return
    _keep_stop()
    try:
        t = machine.Timer(0)
        period = WDT_TIMEOUT // 3
        if period < 1000:
            period = 1000
        _keep_from = time.ticks_ms()
        t.init(period=period, mode=machine.Timer.PERIODIC, callback=_keep_cb)
        _keep = t
    except Exception as e:
        print("wdt timer", repr(e))

# --- WiFi -------------------------------------------------------------------
# Up to three networks. Network #1 is "home" by default and preferred
# for connections. Logging continues and finished files upload on all networks.
#
# Connect is a state machine in wifi_tick (one status poll per loop), so
# GPS sampling is not blocked. Scan once per round; try strongest visible
# known SSID (deduped by SSID). Remaining configured nets are tried in config
# order after visible ones when the scan failed, when hidden, and every
# UNLISTED_EVERY rounds, so a missed scan does not block fallback for long.

WLAN_ERR = (OSError, RuntimeError)       # the esp32 port raises RuntimeError
SCAN_ERR = WLAN_ERR + (MemoryError,)     # scan() builds its list on the heap
                                         # ("Wifi Unknown Error 0x...") for an
                                         # unmapped IDF error (no memory)

_env_partial = [False]                   # load_env stopped part way (EIO)
_env_skipped = []                        # keys whose value it skipped (not UTF-8)
                                         # or whose garbled key it guessed
# The security-relevant keys in one table (two lists drifted apart: an
# SSID_HOME not UTF-8 was kept as bytes for its "SSID", read as no flag,
# home): (key, its value kept as bytes when not UTF-8 (the radio takes an
# ANSI SSID or password as it is), its line not read fails closed (WEB_TOKEN
# closes the routes, a WPA2 drops its network, a home flag makes it not
# home, UPLOAD_UNVERIFIED stays 0: the old UPLOAD_AWAY_UNVERIFIED is not
# looked at))
_ENV_SEC = (("SSID", 1, 0), ("SSID2", 1, 0), ("SSID3", 1, 0),
            ("WPA2", 1, 1), ("WPA2_2", 1, 1), ("WPA2_3", 1, 1),
            ("WEB_TOKEN", 0, 1), ("SSID_HOME", 0, 1), ("SSID2_HOME", 0, 1),
            ("SSID3_HOME", 0, 1), ("HOME1", 0, 1), ("HOME2", 0, 1),
            ("HOME3", 0, 1), ("SSID_WEB", 0, 1), ("SSID2_WEB", 0, 1),
            ("SSID3_WEB", 0, 1), ("UPLOAD_UNVERIFIED", 0, 1))
_ENV_RAW = tuple(k for k, raw, _ in _ENV_SEC if raw)
# (the keys taken verbatim, a "#" in them no comment: see _plain)
_ENV_VERBATIM = ("GEO_URL", "API_KEY", "WEB_TOKEN", "OTA_KEY", "OTA_URL")
_ENV_GUARD = tuple(k for k, _, closed in _ENV_SEC if closed)

def _env_key_fits(k, b):
    """k (bytes past ASCII 0xFF-marked in it) is b with those in its
    letters' places."""
    return len(k) == len(b) and all(a == c or a > 127 for a, c in zip(k, b))

def _env_key_guess(k):
    """The keys of _ENV_GUARD a key with bytes past ASCII (k) may have been,
    every one that fits ("WPA2_" and a bad byte is WPA2_2 or WPA2_3), by
    the first rule that finds one: the same length with them where its
    letters were (a byte gone bad: SSID\xb2_HOME is SSID2_HOME or
    SSID3_HOME, not SSID_HOME too: network #1 lost its home flag); the
    same without them (an NBSP, a BOM in the middle); a run of them as one
    letter (a UTF-8 letter, two bytes, or U+FFFD, three, where an ANSI
    byte was: none fitted, the key counted as unset, the routes stayed
    open). -> a list, empty for none."""
    out = [n for n in _ENV_GUARD if _env_key_fits(k, n.encode())]
    if not out:
        r = bytes(c for c in k if c < 128).strip()
        out = [n for n in _ENV_GUARD if r == n.encode()]
    if not out:
        q = bytearray()
        for c in k:
            if c < 128 or not q or q[-1] != 255:
                q.append(c if c < 128 else 255)
        out = [n for n in _ENV_GUARD if _env_key_fits(q, n.encode())]
    return out

def load_env(path):
    """KEY=VALUE lines -> dict. Surrounding quotes are stripped.

    Read as bytes: a .env saved as ANSI (Notepad) raised UnicodeError at
    every boot. A value that is not UTF-8 stays bytes for SSID/WPA2 keys
    (connect() takes bytes) and is skipped for the others; never raises.
    A read stopped part way (EIO) sets _env_partial: a key after it is
    missing, not unset (_web_env: a WEB_TOKEN there left every data route
    open)."""
    env = {}
    _env_partial[0] = False
    del _env_skipped[:]
    try:
        with open(path, "rb") as f:
            for line in f:
                if line[:3] == b"\xef\xbb\xbf":
                    line = line[3:]          # UTF-8 BOM (Windows Notepad)
                line = line.strip()
                if not line or line[:1] == b"#" or b"=" not in line:
                    continue
                k, v = line.split(b"=", 1)
                k, v = k.strip(), v.strip()
                if (len(v) >= 2 and v[:1] == v[-1:]
                        and v[:1] in (b'"', b"'")):
                    v = v[1:-1]
                if any(c > 127 for c in k):
                    # (no key is: an ANSI or garbled line. One that was
                    # WEB_TOKEN or a WPA2 is said missing, not unset: the
                    # routes stayed open, the network was tried open; any
                    # other is just skipped, a "Megjegyzés=" closed nothing)
                    g = _env_key_guess(k)
                    if not g:
                        _say("env a key not plain ASCII, its line skipped")
                    else:
                        _env_skipped.extend(g)
                        _say("env %s: its key not plain ASCII, its line "
                             "skipped" % " or ".join(g))
                    continue
                k = k.decode()
                try:
                    v = v.decode()
                except UnicodeError:
                    # (a flag or a name whose comment alone is not UTF-8,
                    # SSID_HOME=1 # otthoni hálózat as ANSI: the value
                    # before the "#"; it made the home network not home)
                    c = v.split(b"#", 1)[0].strip()
                    if k in _ENV_RAW:
                        pass
                    elif (k not in _ENV_VERBATIM and b"#" in v
                          and all(b < 128 for b in c)):
                        v = c.decode()
                        _say("env %s: its comment not UTF-8, dropped" % k)
                    else:
                        _say("env %s not UTF-8, skipped" % k)
                        _env_skipped.append(k)
                        continue
                env[k] = v
    except OSError as e:
        if not _enoent(e):               # (no .env on the device: nothing
            _env_partial[0] = True
            try:                         # to say; an EIO in the middle
                _say("env read stopped by OSError %s, the rest skipped"
                     % (e.args[0] if e.args else "?"))
            except Exception:            # left a part of it unsaid)
                pass
    except Exception as e:               # (the type only: no value)
        _env_partial[0] = True
        try:
            _say("env read stopped by %s, the rest skipped" % type(e).__name__)
        except Exception:
            pass
    return env

def _plain(v):
    """Value of a non-secret key without a trailing "# comment".

    Only for keys whose values can never contain "#" (flags, IPv4
    addresses, names). SSIDs, passwords, GEO_URL and API_KEY are verbatim."""
    if v is None:
        return None
    i = v.find("#")
    v = (v if i < 0 else v[:i]).strip()
    if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
        v = v[1:-1].strip()              # IP="192.168.1.60" # static
    return v

def _truthy(v):
    if v is None or isinstance(v, bytes):   # (bytes: not UTF-8, no flag)
        return None
    s = _plain(str(v)).lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off"):
        return False
    return None

def _ok_char(c, extra):
    """ASCII letter/digit (MicroPython 1.29 has no str.isalnum)."""
    return (("a" <= c <= "z") or ("A" <= c <= "Z") or ("0" <= c <= "9")
            or c in extra)

def _safe_name(s, default, host=False):
    """DEVICE_ID: [A-Za-z0-9._-]{1,32}. HOSTNAME: [A-Za-z0-9-]{1,32}."""
    if not s:
        return default
    if len(s) < 1 or len(s) > ID_MAX:
        print("bad name, using default")
        return default
    extra = "-" if host else "._-"
    for c in s:
        if _ok_char(c, extra):
            continue
        print("bad name, using default")
        return default
    return s

def _ipv4(s):
    """Dotted quad as given, or None when it is not a valid IPv4 address."""
    parts = (s or "").strip().split(".")
    if len(parts) != 4:
        return None
    for p in parts:
        if not p or len(p) > 3:
            return None
        for c in p:
            if c < "0" or c > "9":
                return None
        if int(p) > 255:
            return None
    return ".".join(parts)

def _static_ip(env, keys):
    """(ip, mask, gateway, dns) from IP/MASK/GW/DNS keys, or None for DHCP.

    A static IP needs a gateway; MASK defaults to 255.255.255.0 and DNS to the
    gateway. An invalid entry falls back to DHCP instead of leaving the board
    unreachable."""
    ipk, mk, gk, dk = keys
    raw = _plain(env.get(ipk))
    if not raw:
        return None
    ip = _ipv4(raw)
    gw = _ipv4(_plain(env.get(gk)))
    if not ip or not gw:
        _env_refused(ipk, "needs a valid %s and %s" % (ipk, gk), "DHCP")
        return None
    mask_raw, dns_raw = _plain(env.get(mk)), _plain(env.get(dk))
    mask, dns = _ipv4(mask_raw), _ipv4(dns_raw)
    if mask_raw and not mask:
        _env_refused(mk, "invalid", "255.255.255.0")
    if dns_raw and not dns:
        _env_refused(dk, "invalid", gk)
    return (ip, mask or "255.255.255.0", gw, dns or gw)

def _say(line):
    """On the console and in the diag log: a .env mistake (key names only: an
    SSID or a password never goes into either), a GPS or heap note."""
    print(line)
    _diag_write(line)

def _env_refused(key, why, used):
    """A .env value refused (_say: in the field there is no console): the
    key, why, and what is used instead, never the value itself."""
    _say("env %s %s, using %s" % (key, why, used))

def _load_nets(env):
    """Up to three networks from .env. The same SSID twice: the first entry
    wins, the later one is dropped (it took over the scan match, its
    password and its home flag). An empty WPA2 next to an SSID (an open
    network then) and an SSIDn / WPA2_n key not read here (SSID1, SSID4,
    WPA2_1) are warned about (_say)."""
    nets = []
    known = []
    seen = []
    for x in ("", "2", "3"):
        sk, pk, hk2, hidk = ("SSID" + x, "WPA2_" + x if x else "WPA2",
                             "SSID" + x + "_HOME", "SSID" + x + "_HIDDEN")
        webk = "SSID" + x + "_WEB"
        known += (sk, pk, hk2, hidk, webk)
        s = env.get(sk)
        if not s:
            continue
        sb = s if isinstance(s, bytes) else s.encode()
        if sb in seen:
            _say("env %s: an SSID already set above, ignored" % sk)
            continue
        seen.append(sb)
        p = env.get(pk)
        if p is None and (_env_partial[0] or pk in _env_skipped):
            # (.env read part way, or its line garbled: its password may be
            # there; tried open, an open twin of the home SSID was home)
            _say("env %s not read (.env read part way, or its line garbled): "
                 "%s dropped" % (pk, sk))
            continue
        if p is not None and not p:
            _say("env %s empty: %s is tried as an open network"
                      % (pk, sk))
        home = not x                     # (the first network by default)
        hk3 = "HOME" + (x or "1")
        t, bad = None, None
        for hk in (hk2, hk3):
            # (SSIDn_HOME first; one not read is not looked past: its 0
            # overrode a HOMEn=1)
            if hk in _env_skipped:
                bad = hk
                break
            v = env.get(hk)
            if v is not None:
                t = _truthy(v)
                if t is None and _plain(v):
                    # (n, nem, "0 ; hotspot": home by default, nothing said)
                    bad = hk
                    _env_refused(hk, "not 1/0", "not home")
                break
        if t is None and bad is None and _env_partial[0]:
            bad = hk2
        if t is not None:
            home = t
        elif bad is not None:
            # (its home flag may have been there, a 0: home by default sent
            # the key over unverified TLS with UPLOAD_UNVERIFIED=home on a
            # phone's hotspot)
            if home and (bad in _env_skipped or _env_partial[0]):
                _say("env %s not read (.env read part way, or its line "
                     "garbled): %s not home (no OTA, no home stay there)"
                     % (bad, sk))
            home = False
        hidden = bool(_truthy(env.get(hidk)))
        # SSIDn_WEB=1: the page is served there (one's own phone's hotspot)
        # and keeps the radio on, without the network being home: no home
        # stay, no OTA, no roaming veto. Its line not read, or a value not
        # 1/0: no page there (it would show the position and the track)
        web = False
        if webk in _env_skipped:
            _env_refused(webk, "not read (its line garbled)", "no page there")
        else:
            v = env.get(webk)
            if v is not None:
                t = _truthy(v)
                if t is None:
                    if _plain(v):
                        _env_refused(webk, "not 1/0", "no page there")
                else:
                    web = t
        n = {"ssid": s, "pw": "" if p is None else p,
             "home": home, "web": web, "hidden": hidden, "bssid": None,
             "ip": _static_ip(env, ("IP" + x, "MASK" + x, "GW" + x,
                                    "DNS" + x))}
        if isinstance(s, bytes):
            # not UTF-8 (an ANSI .env): the radio gets the bytes as they
            # are, the screen and the page one char per byte
            n["raw"] = s
            n["ssid"] = "".join(chr(c) for c in s)
        nets.append(n)
    for k in list(env) + _env_skipped:  # (a skipped one too: not UTF-8)
        if k[:4] in ("SSID", "WPA2") and k not in known:
            _say("env %s ignored (SSID, SSID2, SSID3 and WPA2, WPA2_2, "
                      "WPA2_3 only)" % k)
    return nets

def _ssid_b(n):
    """A configured network's SSID as the radio sees it (bytes)."""
    r = n.get("raw")
    return r if r is not None else n["ssid"].encode()

wifi = {}

def wifi_reset():
    wifi.clear()
    wifi.update(wlan=None, ssid=None, nets=[], failed={},
                state="off", ip=None, rssi=None, home=False,
                backoff=WIFI_RETRY, last_check=_stamp(),
                cand=[], cand_i=0, try_at=None,
                roam_at=_stamp(), prev_status=None,
                roam_backoff=ROAM_CHECK, moved_at=None, roaming=False,
                retried=False, idle_at=_stamp(), sleep_home=False,
                lost_at=None, fails=0, diag_at={}, diag_more={}, empty=0,
                pb_at=time.ticks_ms(), pb_scans=0, pb_empty=0, hr_empty=0,
                wake=False, wake_at=None,
                blank_rssi=None, blank_bssid=None, blank_bad=[],
                blank_ours=False, home_aps=[], home_miss={}, learn=False,
                hid_wait=ROAM_CHECK, hid_at=None, blank_any=False,
                rounds=0, hh=None,
                seen_at=None, seen_wait=SEEN_WAIT,
                # the link is down since boot (a board booting faster than
                # the router): _web_search counts from here, as after a
                # drop (lost); down_ssid None: any home network may come up
                down_at=_stamp(), down_home=True, down_rssi=None,
                down_ssid=None)

def _rssi(w):
    try:
        return w.status("rssi")
    except (OSError, ValueError, RuntimeError):
        return None

def _wifi_diag(kind, line):
    """A WiFi event in the diag log (a drop and why the reconnects failed
    were only on the console): one line of a kind per WIFI_DIAG_GAP, the
    ones in between counted into the next."""
    at, more = wifi["diag_at"], wifi["diag_more"]
    if _recent_s(at.get(kind), WIFI_DIAG_GAP // 1000):
        more[kind] = more.get(kind, 0) + 1
        return
    n = more.pop(kind, 0)
    at[kind] = _stamp()
    _diag_write(line + (" (+%d more)" % n if n else ""))

def _mark_failed(ssid, why=""):
    wifi["failed"][ssid] = _stamp()
    wifi["fails"] = wifi["fails"] + 1
    print("wifi fail", ssid, why)
    _wifi_diag("fail", "wifi fail %s %s" % (ssid, why))
    if wifi["roaming"] and _net_home(ssid):
        if not _net_hidden(ssid):
            b = wifi["roam_backoff"]
            wifi["roam_backoff"] = min(max(b * 2, ROAM_CHECK), ROAM_BACKOFF_MAX)
        else:
            # Hidden home: roam_backoff stays (it would delay a visible
            # home switch); hidden tries get their own growing hold, so
            # many neighbours' empty-SSID APs do not drop the hotspot every
            # ROAM_CHECK. That AP is skipped for BLANK_BAD_HOLD (it may be
            # ours, briefly unreachable: not until reboot). A learned BSSID
            # of ours never is: one failed try shut our own router out for
            # an hour (hid_wait throttles the tries already).
            b = wifi["hid_wait"]
            wifi["hid_wait"] = min(max(b * 2, ROAM_CHECK), ROAM_BACKOFF_MAX)
            wifi["hid_at"] = _stamp()
            b = wifi["blank_bssid"]
            if (b is not None and b not in wifi["home_aps"]
                    and not _blank_bad(b)):
                bad = [x for x in wifi["blank_bad"]
                       if _age_s(x[1]) < BLANK_BAD_HOLD // 1000]
                bad.append((b, _stamp()))
                wifi["blank_bad"] = bad[-4:]
        wifi["roaming"] = False

def _blank_bad(bssid):
    """A hidden-home try on this empty-SSID AP failed within BLANK_BAD_HOLD."""
    for b, t in wifi["blank_bad"]:
        if b == bssid and _age_s(t) < BLANK_BAD_HOLD // 1000:
            return True
    return False

def _scan_empty(aps, k="empty"):
    """An empty scan that counts as failed. v1.29 returns [] both when
    esp_wifi_scan_start fails and when no AP is around: the first empty one
    after a non-empty scan is taken as failed (the round then tries every
    network); the next ones in a row are a place without APs (every
    network only each UNLISTED_EVERY rounds again, and an away stay counts
    them as missed APs). The keep-alive scans count in their own k
    ("pb_empty"): one empty pulse made a round's own first empty scan the
    second, and that round tried nothing."""
    if aps:
        wifi[k] = 0
        return False
    n = wifi[k] = wifi[k] + 1
    return n == 1

def _scan_known(w, fx=None, lim=None, k="empty"):
    """Visible known nets, strongest BSSID per SSID, or None if scan fails.
    Hidden APs are listed with an empty SSID; the strongest blank RSSI is
    a hint that *some* hidden AP is nearby (not necessarily ours). One of
    the hidden home's learned BSSIDs (_learn_home) at >= ROAM_MIN_RSSI is
    taken first (blank_ours); else the strongest blank AP stays weak
    evidence (a mesh node not learned yet): enough for the candidate order
    and a roam, not for "home in range" or the keep-alive wake. A blank AP
    whose hidden-home try failed lately (blank_bad) is skipped. A
    MemoryError after the scan (the stay check, the copies below) is a
    failed scan too: it escaped wifi_tick and reset the board. The UART is
    read first and again after it (~2 s; the 8 KB buffer holds ~3.5 s, and
    the connect or the clear that follows can take 500 ms more), and the
    scan counts as a keep-alive pulse (pb_at): a round that found nothing
    was followed by a pulse's scan in the same pass, ~4 s with no read,
    more than the UART buffer holds. lim: the RSSI a learned BSSID of the
    hidden home needs to be taken first (home_in_range's bar; default
    ROAM_MIN_RSSI). k: the empty-scan count it goes in (home_in_range's
    own, as the pulses': its empty scan made the next round's the second,
    and that round tried nothing)."""
    _rcv.read_uart()
    try:
        aps = w.scan()
    except SCAN_ERR:
        aps = None
    _rcv.read_uart()
    wifi["pb_at"] = time.ticks_ms()
    if aps is not None and not _scan_empty(aps, k):
        try:
            return _scan_found(aps, fx, lim)
        except MemoryError:
            gc.collect()
    wifi["blank_rssi"] = None
    wifi["blank_bssid"] = None
    wifi["blank_ours"] = False
    wifi["blank_any"] = False
    return None

def _scan_found(aps, fx=None, lim=None):
    """_scan_known's list from a good scan (aps)."""
    if lim is None:
        lim = ROAM_MIN_RSSI
    by = {}
    for n in wifi["nets"]:
        by[_ssid_b(n)] = n
    mine = wifi["home_aps"]
    _stay_scan(aps, fx)
    _home_seen(aps)
    best = {}
    blank_rssi, blank_bssid = None, None
    own_rssi, own_bssid = None, None
    wifi["blank_any"] = False
    for rec in aps:
        # SSIDs compared as bytes: a neighbour's Latin-1 or cut-off UTF-8
        # SSID raised UnicodeError on decode() and crashed the logger
        ssid = rec[0]
        rssi = rec[3]
        bssid = rec[1]
        if not ssid:
            wifi["blank_any"] = True     # some hidden AP is around
            if _blank_bad(bssid):
                continue                 # its hidden-home try failed
            if bssid in mine:
                if own_rssi is None or rssi > own_rssi:
                    own_rssi, own_bssid = rssi, bssid
            if blank_rssi is None or rssi > blank_rssi:
                blank_rssi, blank_bssid = rssi, bssid
            continue
        if ssid not in by:
            continue
        prev = best.get(ssid)
        if prev is None or rssi > prev[0]:
            best[ssid] = (rssi, bssid, by[ssid])
    if own_rssi is not None and own_rssi >= lim:
        blank_rssi, blank_bssid = own_rssi, own_bssid
    wifi["blank_rssi"] = blank_rssi
    wifi["blank_bssid"] = blank_bssid
    wifi["blank_ours"] = blank_bssid is not None and blank_bssid in mine
    found = []
    items = list(best.values())
    items.sort(key=lambda x: -x[0])
    for rssi, bssid, net in items:
        n = dict(net)
        n["bssid"] = bssid
        n["rssi"] = rssi
        found.append(n)
    return found

def home_in_range(fx=None):
    """True while a home network is in range (>= ROAM_MIN_RSSI): the RSSI
    of the connected home network, else a scan (~2 s, the radio turned on
    for it and off again when it slept). None when that cannot be told
    (no home network configured, a connect in progress, scan failed).
    On the home link the bar is SEEN_MIN_RSSI: ROAM_MIN_RSSI is for
    jumping to a weak AP; associated at -78 dBm the board is at home, and
    that False let a multipath jump end the home stay (_home_veto). The
    same bar holds for the scan while the radio sleeps after the home link
    (sleep_home): a home AP at -78 dBm was in range connected, out of it
    asleep, and the veto let the drift go out (WEB=0 sleeps at home)."""
    w = wifi["wlan"]
    if w is None or not _has_home():
        return None
    state = wifi["state"]
    lim = ROAM_MIN_RSSI
    if state != "connected" and wifi.get("sleep_home"):
        lim = SEEN_MIN_RSSI
    if state == "connected" and wifi["home"]:
        lim = SEEN_MIN_RSSI
        r = _rssi(w)
        if r is not None:
            return r >= lim
        # unknown RSSI: a scan tells (it works while connected)
    if state not in ("connected", "sleep", "not found", "offline"):
        return None
    off = state != "connected"
    try:
        if off:
            w.active(True)
        found = _scan_known(w, fx, lim, "hr_empty")
    except SCAN_ERR:
        found = None
    finally:
        if off:
            try:
                w.active(False)
            except WLAN_ERR:
                pass
    if found is None:
        return None
    for n in found:
        if n.get("home") and n.get("rssi", -100) >= lim:
            return True
    return bool(_blank_home_ok(True, lim))    # (a hidden home: the same bar)

def _home_veto(now, sl, fx=None):
    """A departure from a home stay while the home WiFi is still in range
    is a GPS jump, not a departure (measured: a 194 m multipath jump ended a
    home stay and uploaded 80 drift points). After a veto, departures wait
    STILL_HOME_HOLD before the next check."""
    if _recent_s(sl.get("veto_at"), STILL_HOME_HOLD // 1000):
        return True
    if home_in_range(fx):
        sl["veto_at"] = _stamp()
        _say("stay kept: home in range")
        return True
    return False

def _has_home():
    """Any home network configured (cached per nets list: wifi_tick asks
    every loop pass)."""
    nets = wifi["nets"]
    c = wifi["hh"]
    if c is None or c[0] is not nets:
        c = wifi["hh"] = (nets, any(n.get("home") for n in nets))
    return c[1]

def _blank_home_ok(strict=False, lim=None):
    """True if a hidden home is configured and a strong empty-SSID AP (at
    lim, ROAM_MIN_RSSI by default) was seen. strict ("home in range"): once
    BSSIDs are learned, only one of them counts (_scan_known); else any
    strong blank AP does."""
    r = wifi["blank_rssi"]
    if r is None or r < (ROAM_MIN_RSSI if lim is None else lim):
        return False
    if _blank_bad(wifi["blank_bssid"]):
        return False                     # a hidden-home try there failed
    if strict and wifi["home_aps"] and not wifi["blank_ours"]:
        return False                     # a stranger's (or unlearned) AP
    for n in wifi["nets"]:
        if n.get("home") and n.get("hidden"):
            return True
    return False

def _learn_home(w):
    """Connected to the hidden home: note its BSSID, so a neighbour's
    empty-SSID AP no longer counts as home in range (_scan_known). v1.29
    has no config("bssid") getter: so a scan while connected, and a
    record carrying the hidden SSID (if the IDF names the AP it is on) or
    else the one blank AP on the link's channel within LEARN_RSSI dB of its
    RSSI is ours. Several such blank APs (or none): nothing learned.
    Only on the hidden home itself: a drop before this tick and a hotspot
    connect learned a neighbour's blank AP. A learned BSSID that is not the
    link for HOME_AP_MISS connects, or that the scan shows with another
    SSID, is dropped, so a wrong one does not stick. The list is kept in
    HOME_APS_FILE when it changes."""
    wifi["learn"] = False
    ssid = wifi["ssid"]
    if not (wifi["home"] and _net_hidden(ssid)):
        return
    me = _ssid_b(_net(ssid))             # (hidden home: it is configured)
    try:
        found = []
        other = []                       # BSSIDs of visible networks
        try:
            ch = w.config("channel")
        except (OSError, ValueError, TypeError, RuntimeError):
            ch = None
        r = _rssi(w)
        feed()
        _rcv.read_uart()
        try:
            aps = w.scan()
        except SCAN_ERR:
            return
        feed()
        one, many = None, False
        for rec in aps:
            raw = rec[0]
            if raw:
                if raw == me:
                    found.append(rec[1])  # named by the IDF: ours
                else:
                    other.append(rec[1])
                continue
            if ch is not None and rec[2] != ch:
                continue
            if r is not None and abs(rec[3] - r) > LEARN_RSSI:
                continue
            many = one is not None
            one = rec[1]
        if not found and one is not None and not many:
            found.append(one)
        if not found:
            return                       # cannot tell: nothing changes
        aps = wifi["home_aps"]
        miss = wifi["home_miss"]
        old = list(aps)
        for x in old:
            if x not in found:
                miss[x] = miss.get(x, 0) + 1
        for x in reversed(found):
            miss.pop(x, None)
            if x in aps:
                aps.remove(x)
            else:
                print("wifi home ap learned")
            aps.insert(0, x)
        keep = [x for x in aps if miss.get(x, 0) < HOME_AP_MISS
                and x not in other][:HOME_AP_MAX]
        for x in aps:
            if x not in keep:
                miss.pop(x, None)
        aps[:] = keep
        if aps != old:
            _home_aps_save()
    except MemoryError:
        # the scan's list, the bytes() copies, the save: at ~15 KB free
        # after a dense scan this escaped wifi_tick and reset the board;
        # nothing is learned this time (learn stays cleared)
        gc.collect()

def _home_aps_load():
    """The hidden home's BSSIDs learned before the reboot: the keep-alive
    wake and "home in range" know it before it connected once. A missing
    or damaged file (or a line that is no BSSID) is no error. One that
    cannot be read (EIO) is not saved over this boot (_home_aps_ro): the
    next learnt BSSID wrote the part read as the whole list, the mesh's
    other APs gone for good."""
    aps = wifi["home_aps"] = []
    _home_aps_ro[0] = False
    if not HOME_APS_FILE:
        return
    try:
        f = open(HOME_APS_FILE, "rb")
    except OSError as e:
        if not _enoent(e):
            _home_aps_unread(e)
        return
    try:
        for _ in range(4 * HOME_AP_MAX):     # bounded, garbage lines too
            line = f.readline(40)
            if not line:
                break
            try:
                b = binascii.unhexlify(line.strip())
            except ValueError:
                continue                 # (no BSSID)
            if len(b) == 6 and b not in aps:
                aps.append(b)
                if len(aps) >= HOME_AP_MAX:
                    break
    except OSError as e:
        _home_aps_unread(e)
    except ValueError:
        pass
    finally:
        f.close()

_home_aps_ro = [False]                   # HOME_APS_FILE not read whole: kept

def _home_aps_unread(e):
    _home_aps_ro[0] = True
    _bug_once("bug home aps not read %s: kept as it is this boot" % repr(e)[:50],
              ("home aps read", type(e)))

def _home_aps_save():
    """HOME_APS_FILE rewritten (a few bytes, only when the list changed);
    not into the flash reserve the tracks keep (MIN_FREE). Never raises.
    Written to a .tmp and renamed over: a write error or a power cut in a
    "wb" over the file left it empty, the hidden home's BSSIDs gone after
    the next boot."""
    if not HOME_APS_FILE or _home_aps_ro[0]:
        return
    tmp = HOME_APS_FILE + ".tmp"
    try:
        room = free()
        if room is not None and room < MIN_FREE:
            return
        f = open(tmp, "wb")
        try:
            for b in wifi["home_aps"]:
                f.write(binascii.hexlify(b) + b"\n")
        finally:
            f.close()
        os.rename(tmp, HOME_APS_FILE)
    except (OSError, TypeError, ValueError, MemoryError) as e:
        print("home aps", repr(e))      # (MemoryError: the format, open())
        _rm(tmp)
        if isinstance(e, OSError):       # (no RAM: no diag line either)
            _bug_once("bug home aps not saved %s" % repr(e)[:50],
                      ("home aps", type(e)))

def _hidden_home_target(now, arrival=False):
    """Hidden home when a strong empty-SSID AP was seen.
    Always bssid=None so the IDF probes by SSID (never pin to the
    strongest blank AP — that may be a neighbour). A stop after driving
    must not treat an unverified blank AP as home."""
    if arrival:
        return None
    if not _blank_home_ok():
        return None
    if _recent_s(wifi["hid_at"], wifi["hid_wait"] // 1000):
        return None                      # a hidden-home roam failed lately
    for n in wifi["nets"]:
        if (n.get("home") and n.get("hidden")
                and n["ssid"] != wifi["ssid"]
                and not _recent_s(wifi["failed"].get(n["ssid"]), FAIL_HOLD // 1000)):
            target = dict(n)
            target["bssid"] = None
            return target
    return None

def _candidate_order(w):
    """Visible strongest first (deduped), then unscanned nets in config
    order: all of them when the scan failed, else only hidden ones (a scan
    never lists them by name) and, every UNLISTED_EVERY rounds (the first
    one too), the rest (a scan can miss an AP). A hidden network still
    beacons with an empty SSID: a good scan without any blank AP leaves
    it for that Nth round too. Each try of a network that is not there
    costs CONNECT_TIMEOUT with the radio on."""
    scanned = _scan_known(w)
    rounds = wifi["rounds"]
    wifi["rounds"] = rounds + 1
    if scanned is None:
        cands = [dict(n) for n in wifi["nets"]]
    else:
        seen = set(n["ssid"] for n in scanned)
        every = rounds % UNLISTED_EVERY == 0
        blank = wifi["blank_any"]
        cands = scanned + [dict(n) for n in wifi["nets"]
                           if n["ssid"] not in seen
                           and (every or (blank and n.get("hidden")))]
    if _blank_home_ok() and wifi["blank_ours"]:
        # a hidden home behind one of its learned BSSIDs gets the scan's
        # RSSI, so a failed first association is retried at once
        # (_wifi_failed); a stranger's blank AP is no reason for 2 x 12 s
        for n in cands:
            if n.get("home") and n.get("hidden") and n.get("rssi") is None:
                n["rssi"] = wifi["blank_rssi"]
    good, bad = [], []
    failed = wifi["failed"]
    for n in cands:
        (bad if _recent_s(failed.get(n["ssid"]), FAIL_HOLD // 1000)
         else good).append(n)

    def _home_first(rows):
        homes, rest = [], []
        blank = _blank_home_ok()
        for n in rows:
            rssi = n.get("rssi")
            if n.get("home") and rssi is not None and rssi >= ROAM_MIN_RSSI:
                homes.append(n)
            elif n.get("home") and n.get("hidden") and blank:
                homes.append(n)
            else:
                rest.append(n)
        return homes + rest

    return _home_first(good) + _home_first(bad)

def _wifi_clear(w):
    """Drop the last association so the next status() is not a stale reason."""
    try:
        w.disconnect()
    except WLAN_ERR:
        pass
    try:
        w.active(False)
        w.active(True)
        network.hostname(HOSTNAME)
    except WLAN_ERR:
        pass
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < CLEAR_WAIT:
        feed()
        try:
            s = w.status()
        except WLAN_ERR:
            break
        if s == network.STAT_IDLE or s == network.STAT_CONNECTING:
            break
        time.sleep_ms(50)

def _wifi_ip(w, net):
    """Apply this network's static IP, or DHCP. Once any network has a
    static IP, a DHCP network always asks for DHCP: the interface may keep
    a static address from before (a soft reset too)."""
    cfg = net.get("ip")
    try:
        if cfg:
            w.ifconfig(cfg)
        elif any(n.get("ip") for n in wifi["nets"]):
            w.ifconfig("dhcp")
    except (OSError, ValueError, TypeError, RuntimeError) as e:
        print("wifi ip", repr(e))

def _wifi_connect(w, net):
    wifi["ssid"] = net["ssid"]          # (the password stays in nets only)
    wifi["home"] = bool(net.get("home"))
    wifi["state"] = "connecting"
    wifi["try_at"] = time.ticks_ms()
    wifi["learn"] = False                # only a hidden-home GOT_IP sets it
    print("wifi try", net["ssid"])
    _wifi_clear(w)
    # the reason left after the clear (v1.29 keeps a disconnect reason until
    # GOT_IP): taken before it, the previous SSID's 201/202 masked a real
    # failure of this try with the same code for the whole CONNECT_TIMEOUT
    try:
        wifi["prev_status"] = w.status()
    except WLAN_ERR:
        wifi["prev_status"] = None
    _wifi_ip(w, net)
    ssid = net.get("raw") or net["ssid"]
    try:
        w.connect(ssid, net["pw"], bssid=net.get("bssid"))
    except WLAN_ERR as e:
        _mark_failed(net["ssid"], "connect %r" % (e,))
        return _wifi_next(w)
    return True

def _net(ssid):
    """The configured network ssid ({}: none)."""
    for n in wifi["nets"]:
        if n["ssid"] == ssid:
            return n
    return {}

def _net_home(ssid):
    return bool(_net(ssid).get("home"))

def _net_hidden(ssid):
    return bool(_net(ssid).get("hidden"))

def _net_web(ssid):
    return bool(_net(ssid).get("web"))

def _has_web():
    """Any network with SSIDn_WEB=1 configured."""
    return any(n.get("web") for n in wifi["nets"])

def _web_link():
    """The page is served on the link that is up: home, a network with
    SSIDn_WEB=1 (not home for anything else), or any with WEB_AWAY."""
    return bool(wifi["home"] or WEB_AWAY or _net_web(wifi["ssid"]))

def _wifi_got_ip(w):
    # "connected" last: what raised on the way (a MemoryError formatting the
    # diag line) is done again on the next wifi_tick, not skipped for good
    # (the upload left in the other network's backoff, failed kept)
    try:
        wifi["ip"] = w.ifconfig()[0]
    except WLAN_ERR:
        wifi["ip"] = None
    bad = _dns_bad.get(wifi["ssid"])     # a new IP: its resolver asked again
    if bad is not None and bad[1] != wifi["ip"]:   # (a reconnect with the
        _dns_bad.pop(wifi["ssid"], None)           # same one: not, a flaky
                                                   # link blocked 7 s each)
    wifi["rssi"] = _rssi(w)
    wifi["home"] = _net_home(wifi["ssid"])
    wifi["down_at"] = None
    lost = wifi["lost_at"]
    if lost is not None:
        try:
            _wifi_diag("back", "wifi back %s after %s, %d failed tries, rssi %s"
                       % (wifi["ssid"] or "", _hms(_age_s(lost)),
                          wifi["fails"], wifi["rssi"]))
        except MemoryError:
            pass                         # (only the diag line)
        wifi["lost_at"] = None
    wifi["learn"] = False
    if wifi["home"] and _net_hidden(wifi["ssid"]):
        # our hidden home answered: no blank AP stays blamed for an
        # earlier failed try, hidden roams are due again, and its BSSID is
        # learned on the next wifi_tick (_learn_home)
        wifi["blank_bad"] = []
        wifi["hid_wait"] = ROAM_CHECK
        wifi["hid_at"] = None
        wifi["learn"] = True
    wifi["sleep_home"] = False
    wifi["idle_at"] = _stamp()
    wifi["failed"].pop(wifi["ssid"], None)
    wifi["backoff"] = WIFI_RETRY
    wifi["roam_at"] = _stamp()
    wifi["roaming"] = False
    wifi["seen_wait"] = SEEN_WAIT
    if wifi["home"]:
        wifi["roam_backoff"] = ROAM_CHECK
    _up_net_new(wifi["ssid"])
    if _ntp[0] is None:
        # no answer yet: a backoff grown on a network without internet
        # does not carry over to this one (up to an hour of "--:--")
        _ntp[1], _ntp[2] = None, NTP_RETRY // 2
    elif _ntp_net[0] != wifi["ssid"]:
        # another network: asked again (an answer taken on a foreign one
        # may be a lie; the newer one corrects the board clock, _rtc_sync)
        _ntp[1], _ntp[2] = None, NTP_RETRY // 2
    wifi["state"] = "connected"
    print("wifi ok", wifi["ssid"] or "")
    return True

def _wifi_failed(w, s, elapsed):
    """This attempt failed with raw status s. A candidate the scan showed at
    >= ROAM_MIN_RSSI gets one immediate retry in the same round: on the
    ESP32-C6 (MicroPython 1.29) the first association after a cold boot
    often fails after ~7.5 s with 202 (STAT_WRONG_PASSWORD) although the
    password is right, and the retry connects within ~1 s. Anything else is
    demoted for FAIL_HOLD and the round moves on."""
    ssid = wifi["ssid"]
    why = "status %s after %d ms" % (s, elapsed)
    cands = wifi["cand"]
    i = wifi["cand_i"]
    net = cands[i] if i < len(cands) else None
    rssi = net.get("rssi") if net else None
    # 202 (STAT_WRONG_PASSWORD, also v1.29's CONNECTION_FAIL) on a network
    # the scan saw is retried at any RSSI: the board's home AP reads -65 to
    # -75 dBm, a dip under ROAM_MIN_RSSI skipped the retry the 202 quirk
    # needs, and the reconnect after a drop failed round after round
    quirk = s == getattr(network, "STAT_WRONG_PASSWORD", 202)
    if (net is not None and net["ssid"] == ssid and not wifi["retried"]
            and rssi is not None and (quirk or rssi >= ROAM_MIN_RSSI)):
        wifi["retried"] = True
        print("wifi retry", ssid, why)
        return _wifi_connect(w, net)
    _mark_failed(ssid, why)
    return _wifi_next(w)

def _wifi_next(w):
    i = wifi["cand_i"] + 1
    wifi["cand_i"] = i
    wifi["retried"] = False
    cands = wifi["cand"]
    if i < len(cands):
        return _wifi_connect(w, cands[i])
    # a roam is over: status() raising during it (no _mark_failed) left
    # roaming set, and a later hidden-home failure was taken for a roam
    wifi["roaming"] = False
    _wifi_round_failed(w)
    return False

def _wifi_round_failed(w):
    """No network in this round. With points to upload, search on with the
    usual growing backoff (15 s to 10 min) until one connects, the radio
    off between rounds (_wifi_begin_round turns it on); with nothing to
    upload, sleep (not yet while _web_search)."""
    now = time.ticks_ms()
    if _nothing_to_send() and not _web_search(now) and not _ota_wants_radio():
        _wifi_off(w, "sleep")
        return
    _radio_off(w)
    wifi["state"] = "not found"
    wifi["home"] = False
    wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)
    wifi["last_check"] = _stamp()

def _web_search(now):
    """The status page is on (at home, or WEB_AWAY) and no keep-alive scan
    can see the dropped network come back (_home_seen): a failed round
    keeps searching with the "not found" backoff for WEB_SEARCH after the
    link went down (down_at: boot, lost, a roam), instead of sleeping
    WIFI_SLEEP (1 h, page down) at once (a router restart, a board booting
    faster than the router). Those scans watch home networks only, at
    >= SEEN_MIN_RSSI: not a hidden home whose BSSIDs are not known yet, a
    home link that dropped weaker than that, or the hotspot WEB_AWAY
    served the page on. The hourly round after a deliberate sleep is no
    drop (down_at None). A link off home without WEB_AWAY keeps no page up
    (it is served only while the radio is on anyway, see web_tick): no
    search, keep-alive or not (KEEPALIVE=0 searched 30 min after a hotspot
    drop)."""
    if not WEB or not (WEB_AWAY or _has_home() or _has_web()):
        return False
    if not wifi["down_home"]:
        if not (WEB_AWAY or _net_web(wifi["down_ssid"])):
            return False                 # no page there anyway
    elif PB_KEEPALIVE and _home_seeable(wifi["down_ssid"]):
        r = wifi["down_rssi"]
        if r is None or r >= SEEN_MIN_RSSI:
            return False                 # _home_seen wakes it
    return _recent_s(wifi["down_at"], WEB_SEARCH // 1000)

def _home_seeable(ssid=None):
    """A keep-alive scan can see home come back (_home_seen): for the home
    network ssid (the one that dropped, down_ssid) it is visible, or hidden
    with its BSSIDs known. None (boot: any of them may come up): every
    home network is. Before, a visible home in .env made a dropped hidden
    one with no BSSID learned "seeable": the page was down 1 h."""
    aps = wifi["home_aps"]
    any_ = False
    for n in wifi["nets"]:
        if not n.get("home") or (ssid is not None and n["ssid"] != ssid):
            continue
        if n.get("hidden") and not aps:
            return False
        any_ = True
    return any_

def _radio_off(w):
    try:
        w.disconnect()
    except WLAN_ERR:
        pass
    try:
        w.active(False)
    except WLAN_ERR:
        pass

def _wifi_off(w, state, wake_at=None):
    """Radio off: everything is uploaded. It searches again once WIFI_SLEEP
    has passed and there is something to upload, or when a new stay wakes
    it, or at wake_at (ticks: a backed-off upload due again). A home stay
    stays a home stay while the radio sleeps (sleep_home), so it still
    writes no anchor points (a HOME_HEARTBEAT one asks for a round first);
    leaving clears it (_still_leave)."""
    if state == "sleep":
        wifi["sleep_home"] = bool(at_home() and log["still"]["active"])
    _radio_off(w)
    wifi["lost_at"] = None               # ("back after" measures a reconnect,
                                         # not a sleep)
    wifi.update(state=state, home=False, ip=None, rssi=None, wake=False,
                wake_at=wake_at, backoff=WIFI_RETRY, last_check=_stamp())
    print("wifi", state, "home" if wifi["sleep_home"] else "")

def _upload_wait(now):
    """ms until upload_tick will next send something, 0 when it is due (or
    nothing waits): the time an upload error (server down, 401, 5xx), a
    flash or memory error backed off, or, with every waiting track held
    back (bad payloads), until the first hold ends. Before, "idle" with
    all tracks held kept the radio on for good."""
    state = str(up["state"] or "")
    d = time.ticks_diff(up["next_at"], now)
    wait = d if 0 < d <= UP_WAIT_MAX else 0
    if state.startswith("err") or state in ("fs err", "low mem", "low idf",
                                            "ca err"):
        return wait
    # "bad full": the rejected track is held (BACKOFF_MAX), its 15 s
    # next_at is no reason to keep the radio on: the holds below decide
    held = up["hold"]
    if not held:
        return 0
    first = None
    for p in pending():
        t = held.get(p)
        if t is None:
            return 0                     # one is due
        h = time.ticks_diff(t, now)
        if not 0 < h <= UP_WAIT_MAX:     # (as _due: a hold from before a wrap is over)
            return 0
        first = h if first is None or h < first else first
    if first is None:
        return 0
    return max(wait, min(first, UP_WAIT_MAX))

def _wifi_off_backoff(w, now):
    """Uploads back off for a while (WIFI_OFF_WAIT or more): the radio
    idles connected for nothing (~20-30 mA), so it goes off until the
    attempt is due (wake_at), then a round connects and uploads again. A
    stay that began at home stays a home stay meanwhile (sleep_home)."""
    wait = _upload_wait(now)
    if wait < WIFI_OFF_WAIT:
        return False
    _wifi_off(w, "sleep", time.ticks_add(now, wait))
    print("wifi off until upload retry", wait // 1000, "s")
    return True

def _wifi_begin_round(w):
    try:
        if not w.active():
            w.active(True)               # after _wifi_off
            network.hostname(HOSTNAME)
    except WLAN_ERR:
        pass
    wifi["cand"] = _candidate_order(w)
    wifi["cand_i"] = 0
    wifi["retried"] = False
    if not wifi["cand"]:
        _wifi_round_failed(w)
        return False
    return _wifi_connect(w, wifi["cand"][0])

def _radio_idle_ok():
    """The radio may sleep: everything is uploaded, or only an away stay's
    open anchor file (and its RAM) waits. That file is closed every
    STILL_FILE while connected and wakes the hourly round (_nothing_to_send),
    so the radio and a phone hotspot do not idle on for the whole stay."""
    if _nothing_to_send():
        return True
    sl = log["still"]
    return bool(sl["active"] and not sl["home"] and not pending())

def _ota_wants_radio():
    """A new firmware still has to read the manifest to be valid
    (_ota_probe): the radio stays on and rounds go on for it. A failed
    first read and then the idle sleep was no retry, so a rollback at
    OTA_HEALTH (and two of them marked a good build bad)."""
    return ota.get("pend") is not None

def _nothing_to_send():
    """All uploaded, or no GEO_URL/API_KEY ("no .env": never sends, so the
    waiting tracks are no reason for rounds every backoff or hour)."""
    # nothing left for the network: no finished track waiting, nothing in RAM,
    # and the open track (if any) holds no points yet. An open track with
    # points is closed by log_tick on WiFi, then uploaded.
    return (not log["batch"] and (log["file"] is None or not log["points"])
            and not pending()) or up["state"] == "no .env"

def wifi_wake():
    """A new stay: look for a network now instead of after WIFI_SLEEP or the
    backoff, so a home stay is known as one and the arrival uploads."""
    if wifi["wlan"] is None:
        return False
    if wifi["state"] == "sleep":
        wifi["wake"] = True
        return True
    if wifi["state"] == "not found":
        wifi["rounds"] = 0               # an arrival: try every network
        wifi["last_check"] = _stamp() - wifi["backoff"] // 1000
        return True
    return False

def _wifi_poll(w):
    """Advance a connecting attempt. Never sleeps. try_at is this attempt."""
    now = time.ticks_ms()
    started = wifi["try_at"]
    if started is None:
        wifi["try_at"] = now
        started = now
    elapsed = time.ticks_diff(now, started)
    try:
        s = w.status()
    except WLAN_ERR:
        return _wifi_next(w)
    if s == network.STAT_GOT_IP:
        return _wifi_got_ip(w)
    if s == network.STAT_CONNECTING or s == network.STAT_IDLE:
        if elapsed >= CONNECT_TIMEOUT:
            return _wifi_failed(w, s, elapsed)
        return False
    # v1.29 connect() may leave the reason the clear left (prev_status,
    # taken after it): that one is not this attempt's until CONNECT_TIMEOUT
    # elapses with no GOT_IP; any other failure status is (a fast fail: the
    # radio no longer stays on 12 s for every network that is not there)
    leftover = s == wifi["prev_status"]
    if leftover and elapsed < CONNECT_TIMEOUT:
        return False
    return _wifi_failed(w, s, elapsed)

def _try_roam_home(w, arrival=False):
    """If a home AP is visible and strong enough, switch; else stay put.
    Runs between upload batches (wifi_tick never runs during POST).
    arrival=True is a stop after driving: only a visible home SSID may
    reset backoff and trigger a switch (not an unverified blank AP)."""
    feed()
    _rcv.read_uart()
    found = _scan_known(w)
    feed()
    now = time.ticks_ms()
    target = None
    for n in found or ():
        if (n.get("home") and n.get("rssi", -100) >= ROAM_MIN_RSSI
                and n["ssid"] != wifi["ssid"]
                and not _recent_s(wifi["failed"].get(n["ssid"]), FAIL_HOLD // 1000)):
            target = n
            break
    if target is None:
        target = _hidden_home_target(now, arrival)
    if target is None:
        if not arrival:
            wifi["roam_backoff"] = ROAM_CHECK
        return
    if arrival:
        wifi["roam_backoff"] = ROAM_CHECK
        wifi["roam_at"] = _stamp()
    cur = _net(wifi["ssid"])
    wifi["cand"] = [target] + ([dict(cur)] if cur else [])
    wifi["cand_i"] = 0
    wifi["retried"] = False
    wifi["roaming"] = True
    wifi["down_at"] = _stamp()           # the link is left: if neither
    wifi["down_home"] = False            # connects, a drop off home
    wifi["down_rssi"] = None
    wifi["down_ssid"] = None
    print("wifi roam", target["ssid"])
    _wifi_connect(w, target)

def _gps_sys_env(env):
    """GPS_SYS=<hex CFGSYS mask> in .env: the UC6580's systems and bands (its
    own otherwise; H35155 on the board). A bad value keeps its own."""
    v = _plain(env.get("GPS_SYS"))
    if not v:
        return
    try:
        _rcv.UC_SYS = int(v[1:] if v[0] in "hH" else v, 16)
    except ValueError:
        _env_refused("GPS_SYS", "invalid", "the receiver's own")

def _min_move_env(env):
    """MIN_MOVE=<metres> in .env, 0-50 (decimals allowed): how far from the
    last point a new one has to be. Anything else keeps the default."""
    global MIN_MOVE
    v = _plain(env.get("MIN_MOVE"))
    if not v:
        return
    try:
        m = float(v)
    except ValueError:
        m = -1
    if 0 <= m <= 50:
        MIN_MOVE = m
    else:
        _env_refused("MIN_MOVE", "not 0-50 m", MIN_MOVE)

def _keepalive_env(env):
    """KEEPALIVE=<seconds> in .env: 0 turns the keep-alive off, otherwise
    5-600 s between pulses; anything else keeps the default."""
    global PB_KEEPALIVE
    v = _plain(env.get("KEEPALIVE"))
    if v:
        try:
            n = int(v)
            if n == 0:
                PB_KEEPALIVE = 0
            elif 5 <= n <= 600:
                PB_KEEPALIVE = n * 1000
            else:
                _env_refused("KEEPALIVE", "not 0 or 5-600 s",
                             "%d s" % (PB_KEEPALIVE // 1000))
        except ValueError:
            _env_refused("KEEPALIVE", "invalid", "%d s" % (PB_KEEPALIVE // 1000))

def _home_heartbeat_env(env):
    """HOME_HEARTBEAT=<seconds> in .env: 0 (the default) writes nothing at
    home, as before; 600-86400 s an anchor point that often during a home
    stay. Anything else keeps the default."""
    global HOME_HEARTBEAT
    v = _plain(env.get("HOME_HEARTBEAT"))
    if not v:
        return
    try:
        n = int(v)
    except ValueError:
        n = -1
    if n == 0 or 600 <= n <= 86400:
        HOME_HEARTBEAT = n * 1000
    else:
        _env_refused("HOME_HEARTBEAT", "not 0 or 600-86400 s",
                     "%d s" % (HOME_HEARTBEAT // 1000))

def _pb_pulse_radio(w):
    """Radio on, one scan (it holds ~100 mA ~2 s, and watches the stay's access
    points and the home network: _stay_scan, _home_seen), off again. Only
    the scan's errors are a failed pulse: what the scan leads to (a stay's
    end writes the track) runs after it, its errors a diag line (they
    were swallowed as a failed scan, the stay never ended)."""
    aps = None
    try:
        w.active(True)
        aps = w.scan()
    except SCAN_ERR:
        pass
    finally:
        try:
            w.active(False)
        except WLAN_ERR:
            pass
    if aps is not None and not _scan_empty(aps, "pb_empty"):
        # (its own errors: said once a kind, not a failed pulse, and not
        # raised either: a guarded tick's TICK_PAUSE stretched the pulses
        # past what a power bank counts as a load)
        try:
            _stay_scan(aps)
            _home_seen(aps)
        except MemoryError:
            raise
        except Exception as e:
            _bug_once("bug keepalive %s" % repr(e)[:60], ("keepalive", type(e)))

def keepalive_tick(now):
    """Power bank keep-alive. With the radio asleep the board draws less
    than many power banks count as a load, and they switch off (the
    board's 20 Ah bank after 1 h; with a scan every 15 s after 3 h, so it
    averages the load over a long time). Every PB_KEEPALIVE ms one scan goes
    out (_pb_pulse_radio). Not while the radio is on anyway (connected,
    connecting, lost). GPS seconds published during a pulse are queued, not
    lost."""
    w = wifi["wlan"]
    # radio idle: asleep, nothing found, or no WiFi at all (no .env, an
    # error): a logging-only board needs the pulses most
    if not PB_KEEPALIVE or wifi["state"] not in ("sleep", "not found", "no .env",
                                                 "error", "off"):
        wifi["pb_at"] = now
        return
    if _age(now, wifi["pb_at"]) < PB_KEEPALIVE:
        return
    wifi["pb_at"] = now
    feed()
    if w is None:
        w = _pb_wlan()
    if w is None:
        return
    _rcv.read_uart()                     # (a round's scan may just have run)
    _pb_pulse_radio(w)
    wifi["pb_scans"] += 1
    feed()

_pb_w = None

def _pb_wlan():
    """A station interface only for the pulses when WiFi is not set up (no
    .env or no SSID): created once, None if the chip refuses."""
    global _pb_w
    if _pb_w is None:
        try:
            _pb_w = network.WLAN(network.STA_IF)
        except WLAN_ERR:
            return None
    return _pb_w

def _home_seen(aps):
    """With the status page on (WEB), a keep-alive scan that sees a home
    network at >= SEEN_MIN_RSSI while the radio sleeps or found nothing
    starts a round now. Before, a router restart (or a board booting
    faster than the router) left the page down for WIFI_SLEEP (1 h) while
    every scan saw home. At most every seen_wait ms, doubling while those
    rounds fail (wrong password, weak link)."""
    state = wifi["state"]
    if not WEB or state not in ("sleep", "not found"):
        return
    homes = [_ssid_b(n) for n in wifi["nets"] if n.get("home")]
    if not homes:
        return
    mine = wifi["home_aps"]    # the hidden home's learned BSSIDs
    for rec in aps:
        if rec[3] < SEEN_MIN_RSSI:
            continue
        raw = rec[0]
        if (raw and raw in homes) or (not raw and rec[1] in mine):
            break
    else:
        return
    if _recent_s(wifi["seen_at"], wifi["seen_wait"] // 1000):
        return
    wifi["seen_at"] = _stamp()
    wifi["seen_wait"] = min(wifi["seen_wait"] * 2, WIFI_BACKOFF_MAX)
    if state == "sleep":
        wifi["wake"] = True
    else:
        wifi["last_check"] = _stamp() - wifi["backoff"] // 1000
    print("wifi home in range")

def wifi_start(env=None):
    """WiFi, upload, page, OTA and NTP settings from .env (env: the dict
    run() read already), the first connect round."""
    global DEVICE_ID, HOSTNAME, CPU_MHZ, CPU_MHZ_UPLOAD
    if env is None:
        env = load_env(ENV_FILE)
    CPU_MHZ = _mhz_env(env, "CPU_MHZ", CPU_MHZ)
    CPU_MHZ_UPLOAD = _mhz_env(env, "CPU_MHZ_UP", CPU_MHZ_UPLOAD)
    wifi["nets"] = _load_nets(env)
    _has_home()                          # cached for wifi_tick
    for n in wifi["nets"]:
        if n.get("home") and n.get("hidden"):
            _home_aps_load()             # the hidden home's known BSSIDs
            break
    if wifi["nets"]:
        wifi["ssid"] = wifi["nets"][0]["ssid"]
        wifi["home"] = wifi["nets"][0]["home"]
    up["url"] = env.get("GEO_URL", "").rstrip("/") or None
    up["key"] = env.get("API_KEY") or None
    _keepalive_env(env)
    _home_heartbeat_env(env)
    if wifi["nets"] or PB_KEEPALIVE:
        # the radio is used (a network, or the keep-alive's scans): WiFi
        # needs 80 MHz or more (_set_freq); 20 / 40 are for a board with
        # the radio never on
        if 0 < CPU_MHZ < 80:
            _env_refused("CPU_MHZ", "too slow for WiFi", 80)
            CPU_MHZ = 80
        if 0 < CPU_MHZ_UPLOAD < 80:
            _env_refused("CPU_MHZ_UP", "too slow for WiFi", 80)
            CPU_MHZ_UPLOAD = 80
    _set_freq(CPU_MHZ)
    _web_env(env)
    _ota_env(env)
    _ntp_env(env)
    _up_env(env)                         # (after GEO_URL: its TLS note)
    DEVICE_ID = _safe_name(_plain(env.get("DEVICE_ID")), DEFAULT_DEVICE_ID)
    HOSTNAME = _safe_name(_plain(env.get("HOSTNAME"))
                          or _plain(env.get("DEVICE_ID")),
                          DEFAULT_HOSTNAME, host=True)
    if up["url"] and up["url"].startswith("http://"):
        print("WARNING: GEO_URL is http:// - API key and GPS data are unencrypted")
        print("WARNING: redirects are not followed; use the https origin")
    if not wifi["nets"]:
        wifi["state"] = "no .env"
        return
    if _wifi_make() is None:
        return
    wifi["last_check"] = _stamp()
    # guarded as the loop's wifi_tick: a bug or a MemoryError in the boot's
    # first round was run()'s crash branch, a reset and a boot failure
    _guarded("wifi", _wifi_first, wifi["wlan"])

def _wifi_first(w):
    _wifi_begin_round(w)
    if wifi["state"] == "connecting":
        _wifi_poll(w)
    wifi["last_check"] = _stamp()

def _wifi_make():
    """Create the STA interface; on failure state "error", wlan None
    (wifi_tick tries again after the backoff, doubling)."""
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        network.hostname(HOSTNAME)
        w.active(True)
        return w
    except WLAN_ERR:
        wifi["wlan"] = None
        wifi["state"] = "error"
        wifi["last_check"] = _stamp()
        return None

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
        # the interface failed to come up at boot: it used to stay down
        # for good (the keep-alive's own WLAN showed the chip recovered)
        if (wifi["state"] == "error" and wifi["nets"]
                and _age_s(wifi["last_check"]) >= wifi["backoff"] // 1000):
            wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY),
                                  WIFI_BACKOFF_MAX)
            w = _wifi_make()
            if w is not None:
                print("wifi interface up")
                # the normal states from here (offline: the backoff, the
                # keep-alive): a round that raised left the radio on, the
                # state "error", until the doubled backoff
                wifi["state"] = "offline"
                wifi["last_check"] = _stamp()
                try:
                    _wifi_begin_round(w)
                except Exception:
                    if wifi["state"] in ("not found", "offline"):
                        _radio_off(w)
                    raise
        return
    now = time.ticks_ms()
    try:
        s = w.status()
    except WLAN_ERR:
        s = None
    if s == network.STAT_GOT_IP:
        if wifi["state"] != "connected":
            _wifi_got_ip(w)
        else:
            if wifi["learn"]:
                _learn_home(w)
            if _age_s(wifi["last_check"]) >= WIFI_CHECK // 1000:
                wifi["rssi"] = _rssi(w)
                wifi["last_check"] = _stamp()
            interval = wifi["roam_backoff"]
            if not wifi["home"] and _has_home():
                spd = st.get("speed")
                moving = (has_fix(now) and spd is not None and spd >= 5
                          and not log["still"]["active"])
                if moving:
                    wifi["moved_at"] = _stamp()
                    arrival = False
                elif (wifi["moved_at"] is not None
                      and _age_s(wifi["moved_at"]) >= 60):
                    wifi["moved_at"] = None
                    arrival = True
                else:
                    arrival = False
                ra = wifi["roam_at"]     # (is None: Stamp 0 is a time)
                due = (not moving and ra is not None
                       and _age_s(ra) >= interval // 1000)
                if due or arrival:
                    wifi["roam_at"] = _stamp()
                    _try_roam_home(w, arrival=arrival and not due)
            if (wifi["state"] == "connected"
                    and _age_s(wifi["idle_at"]) >= WIFI_IDLE // 1000):
                wifi["idle_at"] = _stamp()
                if WEB and _web_link():
                    pass                 # the status page keeps it on
                elif _ota_wants_radio():
                    pass                 # so does a firmware to confirm
                elif _radio_idle_ok():
                    _wifi_off(w, "sleep")
                else:
                    _wifi_off_backoff(w, now)
        return
    if wifi["state"] == "connected":
        # v1.29 reports STAT_CONNECTING on beacon timeout while the IDF
        # auto-reconnects. Do not map that to "connecting" with a stale
        # try_at (that would immediately fail the current SSID).
        wifi["state"] = "lost"
        wifi["lost_at"] = wifi["down_at"] = _stamp()
        wifi["fails"] = 0
        # a drop: what dropped decides whether _web_search searches
        wifi["down_home"] = bool(wifi["home"])
        wifi["down_rssi"] = wifi["rssi"]
        wifi["down_ssid"] = wifi["ssid"]   # whose return a scan must see
        ssid, rssi = wifi["ssid"] or "", wifi["rssi"]
        # a stay that began at home stays a home stay while the link is
        # down (a router restart): at_home() reads sleep_home off-line
        wifi["sleep_home"] = bool(wifi["home"] and log["still"]["active"])
        wifi["home"] = False
        wifi["ip"] = wifi["rssi"] = None
        wifi["try_at"] = now
        wifi["cand"] = []
        wifi["cand_i"] = 0
        wifi["learn"] = False            # (it may come back on a hotspot)
        wifi["rounds"] = 0               # the next round tries every network
        print("wifi lost", ssid)
        try:                             # last: a MemoryError formatting it
            _wifi_diag("lost", "wifi lost %s rssi %s" % (ssid, rssi))
        except MemoryError:              # left the drop half recorded (no
            pass                         # sleep_home: the home stay away)
        return
    if wifi["state"] == "lost":
        if (s == network.STAT_CONNECTING or s == network.STAT_IDLE
                or s is None):
            ta = wifi["try_at"]
            if ta is not None and _age(now, ta) < LOST_GRACE:
                return
        wifi["state"] = "offline"
        wifi["last_check"] = _stamp() - wifi["backoff"] // 1000
        _radio_off(w)                    # the IDF still reconnects: a scan
                                         # then fails (wifi still connecting)
        # fall through to a fresh round
    if wifi["state"] == "connecting":
        _wifi_poll(w)
        return
    if wifi["state"] == "sleep":
        # after WIFI_SLEEP, only once there is something to upload; sooner
        # when a backed-off upload is due again (wake_at)
        if not wifi["wake"]:
            at = wifi["wake_at"]
            due = at is not None and not _waiting(now, at, UP_WAIT_MAX)
            if not due and _age_s(wifi["last_check"]) < WIFI_SLEEP // 1000:
                return
            if _age_s(wifi["idle_at"]) < WIFI_IDLE // 1000:
                return
            wifi["idle_at"] = _stamp()
            if (_nothing_to_send() and not _ota_wants_radio()
                    and not (WEB and (WEB_AWAY or _has_home()
                                      or _has_web()))):
                wifi["wake_at"] = None
                return
        wifi["wake"] = False
        wifi["backoff"] = WIFI_RETRY
        wifi["last_check"] = _stamp()
        wifi["rounds"] = 0               # a wake: the first round tries all
        try:
            _wifi_begin_round(w)
        except Exception:
            # (a MemoryError outside the scan's own catch) the wake is not
            # spent on a round that never ran: it came back only after
            # WIFI_SLEEP, an arrival not uploaded, the radio on meanwhile
            if wifi["state"] == "sleep":
                wifi["wake"] = True
                _radio_off(w)
            raise
        return
    if _age_s(wifi["last_check"]) < wifi["backoff"] // 1000:
        return
    wifi["last_check"] = _stamp()
    try:
        _wifi_begin_round(w)
    except Exception:
        # (as the sleep's: a MemoryError outside the scan's own catch) the
        # radio the round turned on goes off again: it stayed on, not
        # associated, until the next round, 10 min with the backoff capped
        if wifi["state"] in ("not found", "offline"):
            _radio_off(w)
        raise

def wifi_stop():
    _web_close()
    w = wifi["wlan"]
    if w is None:
        return
    try:
        w.disconnect()
    except WLAN_ERR:
        pass
    try:                                 # (apart: a failed disconnect left
        w.active(False)                  # the radio on)
    except WLAN_ERR:
        pass
    wifi["state"] = "off"
    wifi["home"] = False

def at_home():
    """Whether the connected network is configured as a home network, or
    the radio sleeps during a stay that began on one."""
    if wifi["state"] == "connected":
        return wifi["home"]
    return wifi["sleep_home"]

# --- GPX logger -------------------------------------------------------------
# Format of the lines (older firmware wrote them as they are, .trk; a rejected
# batch's .bad copy is them too): one point per line, "lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ"
# (~46 bytes; the old GPX XML was ~112). ele/hdop are empty when unknown; the
# time is every line's last 16 bytes (see flush).
# A "#TRK2" file (from an older firmware on another receiver: none is written
# now) has "lat,lon,ele,acc,speed,YYYYMMDDTHHMMSSZ": acc the horizontal accuracy
# (m), speed in m/s, each empty when unknown. Readers tell the two by the line
# (4 or 5 commas), .bad copies included.
# "-" starts a new segment. The file is always complete: each batch
# overwrites the footer with new points plus the footer again. LittleFS
# commits on close, so a power cut leaves either the previous or the new
# version. A line cut short has no newline and is skipped on read.
#
# Track files are binary (.trb, "#TRKB" header), ~6-8 bytes a point
# instead of ~46-51. The lines are the same TRK1/TRK2 text, stored as the
# difference from the line before: a keyframe "=" + the whole text line,
# then delta records, each a flag byte (0x80 | bit 0: the time step is not
# 1 s, bit i: field i (lat, lon, ele, hdop/acc, speed) changed) and one
# LEB128 varint per flag bit (the step in s, the zigzag change in the
# field's last decimal). "-\n" and the "#END\n" footer stay text lines
# (they start below 0x80). A delta keeps the keyframe's shape (which
# fields are empty, their decimals): any other line, or one not written
# exactly as its numbers print, is a keyframe. Each flush starts with one,
# and one follows every KEY_EVERY deltas, so a damaged record costs at
# most those. A reader resumes from the .pos "offset keyframe-offset". The
# decoded line is byte for byte the text line: the upload is the same.

HEADERB = "#TRKB\n"
FOOTER = "#END\n"
NEW_SEG = "-\n"
TRB = ".trb"
_TRACKS = (".trk", TRB)                  # the track files (pending)
_BADS = (".trk.bad", ".trb.bad")
KEY_EVERY = 64                           # deltas between two keyframes
_KB_MAX = 1 << 28                        # a delta at least this big: keyframe
_P10 = (1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000,
        1000000000)
# A line's state: [0] seconds since 2020 (None: not valid, no delta from
# it), [1..5] lat, lon, ele, hdop/acc, speed in their last decimal (None:
# empty), [6..10] their decimals, [11] the number of fields before the time
_ke = [None] * 12                        # the encoder's (flush)
_kt = [None] * 12                        # the line being encoded
_ks = [None] * 12                        # the reader's
_TDIG = (0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14)   # the digits of
                                         # YYYYMMDDTHHMMSSZ

def _kb_num(buf, a, b, s, i):
    """buf[a:b] into s[i] (value) and s[i + 5] (decimals): empty, or a
    number written as it prints ("-?(0|[1-9][0-9]*)(.[0-9]+)?", no "-0",
    at most 9 digits). False for anything else."""
    if a == b:
        s[i] = None
        s[i + 5] = 0
        return True
    neg = buf[a] == 45
    if neg:
        a += 1
    v = 0
    d = -1
    ip = 0
    j = a
    while j < b:
        c = buf[j]
        if c == 46:
            if d >= 0 or j == a or j == b - 1:
                return False
            d = 0
        elif 48 <= c <= 57:
            if d < 0:
                if ip and not v:
                    return False         # a leading zero
                ip += 1
            else:
                d += 1
            if ip + (d if d > 0 else 0) > 9:
                return False
            v = v * 10 + c - 48
        else:
            return False
        j += 1
    if not ip or (neg and not v):
        return False
    s[i] = -v if neg else v
    s[i + 5] = d if d > 0 else 0
    return True

def _kb_time(buf, t):
    """The compact time YYYYMMDDTHHMMSSZ at buf[t:t + 16] -> seconds since
    2020, or None (not a valid UTC time from 2020 up to FIX_S_END: the line
    is then a keyframe). The one check of a real date: _ymd_ok."""
    if buf[t + 8] != 84 or buf[t + 15] != 90:
        return None
    for k in _TDIG:
        if not 48 <= buf[t + k] <= 57:
            return None
    y = ((buf[t] - 48) * 1000 + (buf[t + 1] - 48) * 100
         + (buf[t + 2] - 48) * 10 + buf[t + 3] - 48)
    m = (buf[t + 4] - 48) * 10 + buf[t + 5] - 48
    d = (buf[t + 6] - 48) * 10 + buf[t + 7] - 48
    hh = (buf[t + 9] - 48) * 10 + buf[t + 10] - 48
    mi = (buf[t + 11] - 48) * 10 + buf[t + 12] - 48
    ss = (buf[t + 13] - 48) * 10 + buf[t + 14] - 48
    if not _ymd_ok(y, m, d, hh, mi, ss):
        return None
    t = (_days(y, m, d) - EPOCH_DAYS) * 86400 + hh * 3600 + mi * 60 + ss
    return t if t < FIX_S_END else None  # (_kb_dec reads no delta past it)

def _kb_parse(buf, a, e, s):
    """The text line buf[a:e] (no newline) into the state s; s[0] None
    when it is no TRK1/TRK2 point written as it prints."""
    s[0] = None
    if e - a < 21 or buf[e - 17] != 44:
        return
    t = _kb_time(buf, e - 16)
    if t is None:
        return
    i = 1
    p = a
    while True:
        c = buf.find(b",", p, e - 17)
        q = c if c >= 0 else e - 17
        if i > 5 or not _kb_num(buf, p, q, s, i):
            return
        i += 1
        if c < 0:
            break
        p = c + 1
    if i - 1 < 4 or s[1] is None or s[2] is None:
        return
    s[11] = i - 1
    while i <= 5:
        s[i] = None
        s[i + 5] = 0
        i += 1
    s[0] = t

def _kb_uv(out, v):
    while v >= 128:
        out.append((v & 127) | 128)
        v >>= 7
    out.append(v)

def _kb_delta(bl, s, out, a=0, b=-1):
    """Append the text line bl (bl[a:b], its newline last) as a delta
    record from the state s to out and move s on -> True; False (nothing
    written) when it must be a keyframe."""
    t = _kt
    _kb_parse(bl, a, (b if b >= 0 else len(bl)) - 1, t)
    if t[0] is None or s[0] is None or t[11] != s[11]:
        return False
    dt = t[0] - s[0]
    if dt < 1 or dt >= _KB_MAX:
        return False
    m = 128 if dt == 1 else 129
    i = 1
    while i <= 5:
        v, w = t[i], s[i]
        if (v is None) != (w is None) or t[i + 5] != s[i + 5]:
            return False
        if v is not None and v != w:
            if not -_KB_MAX < v - w < _KB_MAX:
                return False
            m |= 1 << i
        i += 1
    out.append(m)
    if dt != 1:
        _kb_uv(out, dt)
    i = 1
    while i <= 5:
        if m & (1 << i):
            d = t[i] - s[i]
            _kb_uv(out, d << 1 if d >= 0 else ((-d) << 1) - 1)
        i += 1
    i = 0
    while i < 12:
        s[i] = t[i]
        i += 1
    return True

KB_CHUNK = 256                           # bytes a flush encodes at a time
WERR_EVERY = 600000                      # ms between write errors in the diag
_kq = [0, 0, 0]                          # the encoder's place: line, segment
                                         # break, deltas since the keyframe

def _kb_start():
    """A new .trb flush body (see _kb_part): a keyframe first."""
    _ke[0] = None
    _kq[0] = _kq[1] = _kq[2] = 0

def _kb_part(lines, k, seg):
    """The next KB_CHUNK bytes or so of the .trb flush body of lines[:k]
    (text lines; a keyframe first, the segment breaks before the indexes in
    seg), from where _kq left off, and _kq moved on."""
    out = bytearray()
    s = _ke
    i, j, n = _kq
    while i < k and len(out) < KB_CHUNK:
        if seg is not None and j < len(seg) and seg[j] == i:
            out += NEW_SEG.encode()
            j += 1
        bl = lines[i].encode()
        if n < KEY_EVERY and _kb_delta(bl, s, out):
            n += 1
        else:
            out.append(61)               # =
            out += bl
            _kb_parse(bl, 0, len(bl) - 1, s)
            n = 0
        i += 1
    _kq[0], _kq[1], _kq[2] = i, j, n
    return out

log = {}

# The departure ring (see _still_active, _still_leave): the last
# STILL_DEP_MAX usable samples of a stay, preallocated once. Per sample its
# time in seconds from the ring's first sample (_RT; a stay's "rt0"), in
# _RF the latitude, the longitude and the distance from the anchor, in _RE
# the elevation and HDOP fields as the track line writes them (_fields: 8 and 4
# bytes, space padded). A rebuilt line is the line _sample_line would have
# written, rounding included: on the board a float is single
# precision, the very value an array("f") keeps (a test host on doubles swaps
# in an array("d")). The track lines are rebuilt on leaving. As tuples of
# the line, its time and the position they took ~225 B a sample on the
# board: 90 of them grew the heap to 116 of 124 kB and left no block over
# 2 kB, so the next larger allocation took the heap mbedTLS needs.
from array import array
_RT = array("i", bytes(4 * STILL_DEP_MAX))         # (zeros, 4 B each:
_RF = array("f", bytes(4 * 3 * STILL_DEP_MAX))     # a raw initialiser)
RE_W = 12                                # ele 8, hdop 4
_RE = bytearray(RE_W * STILL_DEP_MAX)
_RPAD = b" " * RE_W                      # an empty _RE entry

def log_reset():
    log.clear()
    log.update(file=None, created=False, batch=[], points=0, err=0, herr=0,
               herr_at=None,
               fix=False, lost_at=None, new_seg=False, seg_tc=[],
               full=False, wfail=False, wfails=0,
               still=_still_state(), clock=None, seq=0,
               last_sample=None, last_flush=_stamp(), last_t=None,
               prev=None, full_check=None, file_at=None, good_size=None,
               fmt=None, drop=0, full_tc=None, err_at=None, back=0, hb=[])

def _still_state():
    return {"active": False, "home": False, "win_at": None, "win_tc": None,
            "ref": None, "sum": [0.0, 0.0, 0], "prev": None, "anchor": None,
            "aref": None, "aoff": (0.0, 0.0), "n": 0, "hb_at": None,
            "rn": 0, "ri": 0, "rt0": 0, "gap_n": 0, "gap_at": None,
            "gap_ref": None, "gap_seen": None, "pos": None, "qs": None,
            "far_n": 0, "far_d": 0, "far_slow": 0, "far_0": 0,
            "last_at": None, "last_d": 0, "far_at": None, "file_at": None,
            "last_home": False,
            "veto_at": None,
            "move_c": None, "wins": 0, "hold_tc": None, "aps": {},
            "ap_scans": 0, "ap_miss": 0, "ap_miss_at": None}

def _still_hold():
    """Track time from which a stay still being proven would drop its
    drift (the open window, or the previous one it may pair with), or None.
    Periodic flushes keep those lines in RAM, so a stay that is entered
    drops all of them: before, the minute already on flash was uploaded
    after every boot or new fix. HOLD_RELEASE after the last usable fix
    nothing is held: a fix that stays unusable (indoors, HDOP 9) closes no
    window and is no fix loss either, and the lines held at the door stayed
    in RAM for hours (a reboot dropped them)."""
    sl = log["still"]
    if sl["active"]:
        return None
    la = sl["last_at"]
    if la is not None and _age_s(la) >= HOLD_RELEASE // 1000:
        if sl["last_home"] and at_home():
            _hold_drop(sl, _still_held(sl))
        return None
    return _still_held(sl)

def _hold_drop(sl, t):
    """HOLD_RELEASE without a usable fix, the last one taken at home and
    home still: what a stay being proven held from t on within STILL_NEAR
    of where the fix was lost is drift indoors, dropped (written, it went
    up as a trip: 34 points over 57 m after a boot at home, 2026-11-21);
    a way in farther out is kept, as _still_enter keeps it. Away it is all
    written, as before: it may be the start of a way. Then nothing is held
    any more (else the first fix back, recorded before last_at moves on,
    went too)."""
    c = sl["pos"]
    if t is None or c is None:
        return
    b = log["batch"]
    i = len(b)
    while i > 0 and b[i - 1][-17:-1] >= t:
        i -= 1
    keep = [x for x in b[i:] if _line_m(c, x) > STILL_NEAR]
    n = len(b) - i - len(keep)
    if n:
        _say("stay drift dropped %d lines at home" % n)
        b[i:] = keep
    sl.update(win_at=None, prev=None, hold_tc=None)

def _still_held(sl):
    """_still_hold's time while a stay is being proven, or None."""
    h = sl["hold_tc"]                    # a window restarted after a gap
    c = None
    if sl["win_at"] is not None and sl["sum"][2]:
        r, sm = sl["ref"], sl["sum"]
        c = (r[0] + sm[0] / sm[2], r[1] + sm[1] / sm[2])
    p = sl["prev"]
    if p and c is not None and _dist(p[0][0], p[0][1], c[0], c[1]) > STILL_EXIT:
        p = None                         # moving on (a drive): the previous
                                         # window cannot pair, it is track
    if p:
        t = p[1]
    elif sl["win_at"] is not None:
        if c is not None and _spread(sl["ref"], c, sl["pos"]):
            # the open window has spread out: a drive, not a stay's drift.
            # Before, up to two minutes of a drive sat in RAM only.
            return h
        t = sl["win_tc"]
    else:
        return h
    return h if h is not None and h < t else t

def _spread(r, c, q, m=2 * STILL_RADIUS):
    """True when a window with its first fix r, centroid c and latest fix q
    has spread out: its centroid more than m from both ends, a drive and no
    stay's drift. (Both ends, so one multipath jump at either is still
    drift.)"""
    return (q is not None and _dist(r[0], r[1], c[0], c[1]) > m
            and _dist(c[0], c[1], q[0], q[1]) > m)

def free():
    try:
        s = os.statvfs(LOG_DIR)
        return s[0] * s[3]
    except OSError:
        return None

def has_fix(now):
    return (st["valid"] and st["date"] and st["time"] and st["lat"] is not None
            and st["pos_at"] is not None
            and _age(now, st["pos_at"]) < _rcv.STALE)

def _tcn(tc):
    """A compact time as a track file's name: YYYYMMDD_HHMMSS."""
    return tc[:8] + "_" + tc[9:15]

def start_file(stamp=None, fx=None):
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass                             # already exists
    name = stamp or _tcn(_tc(fx))
    # a file of that name already there (a time that went back over a track
    # not uploaded yet) is not overwritten: the first write is "wb"; the
    # name goes on a second at a time
    for _ in range(60):
        try:
            if _size_or_gone(LOG_DIR + "/" + name + TRB) is None:
                break
        except OSError:
            pass                         # (not known: the next name, not a
                                         # "wb" over one perhaps there)
        s = _kb_time((name[:8] + "T" + name[9:15] + "Z").encode(), 0)
        if s is None:
            break
        name = _tcn(_stamp_s(s + 1)[1])
    # (wfail: a write that failed on the previous file must not keep the
    # next one from closing: log_tick)
    log.update(file=LOG_DIR + "/" + name + TRB,
               created=False, points=0,
               new_seg=False, seg_tc=[], file_at=_stamp(),
               good_size=None, wfail=False, wfails=0)
    # First write happens when there are points (parking must not create
    # an empty / one-point file just because a fix came back).

def flush(hold=None, fx=None):
    """Write pending points to the current file, opened only for the write.
    LittleFS commits on close: drop the RAM batch only after a successful
    close, or if the file size shows the new bytes actually landed. With
    hold (a compact track time), only the lines before it are written: the
    rest is the drift of a stay still being proven (see _still_hold)."""
    log["last_flush"] = _stamp()
    path, batch = log["file"], log["batch"]
    k = len(batch)
    if hold is not None:
        k = 0
        while k < len(batch) and batch[k][-17:-1] < hold:
            k += 1
        # hold at most HOLD_MAX lines, so a drive never reaches the
        # MAX_BATCH cap that drops the oldest ones
        k = max(k, len(batch) - HOLD_MAX)
        if not k:
            return
    if path is None or not k:
        return                           # (nothing to write: no empty file,
                                         # e.g. after the flash was full)
    room = free()
    if room is not None and room < MIN_FREE:
        room = _hist_trim(MIN_FREE + HIST_RESERVE)   # the drawing goes
    if room is not None and room < MIN_FREE:         # before the points
        # Nothing is recorded until space returns (log_tick): the track
        # goes on as after a fix loss, in a new segment (a new file past
        # NEW_FILE_GAP), not joined to the last line written. The dropped
        # lines are counted ("drop"); the diag line waits for the space
        # (_diag_write keeps MIN_FREE for the tracks).
        if not log["full"]:
            print("track full, dropped", len(batch))
            log["full_tc"] = _diag_utc(fx) if _fv(fx, "date") else "-"
        log["full"] = True
        log["drop"] += len(batch)
        del batch[:]
        _seg_break()
        log["prev"] = None
        if log["fix"]:
            log["fix"], log["lost_at"] = False, _stamp()
        return
    creating = not log["created"]
    old = log["good_size"]
    if not creating:
        sz0 = _file_size(path)
        if old is None or (sz0 is not None and old > sz0):
            old = sz0
    f = None
    wrote = False
    close_ok = False
    expected = None
    # A segment break goes before the first line recorded after it (each
    # time in seg_tc), not before the whole batch: lines held or not yet
    # flushed from before the break stay in the old segment. Not at the
    # very start of a file. seg_tc is in time order, as the lines are: one
    # pass over both, each line's time sliced once.
    seg = None
    if log["new_seg"]:
        q = log["seg_tc"]
        j = i = 0
        while i < k and j < len(q) and q[j] is not None:   # (None: no
            t = batch[i][-17:-1]                          # line after it)
            if q[j] <= t:
                if i > 0 or not creating:
                    if seg is None:
                        seg = []
                    seg.append(i)
                while j < len(q) and q[j] is not None and q[j] <= t:
                    j += 1
            i += 1
    # encoded and written KB_CHUNK bytes at a time: one body of the whole batch
    # needed a contiguous block that grew with every MemoryError (the batch
    # waited and grew), until MAX_BATCH dropped its oldest lines. The first chunk
    # is encoded before the file is opened: out of memory there is no write
    # failure.
    _kb_start()
    try:
        body = _kb_part(batch, k, seg)   # (the segment breaks in it)
    except MemoryError as e:             # not the flash: the batch waits
        body = None                      # for the next flush
        gc.collect()
        _werr("encode", e)
        return
    # What is left to do once the write landed is worked out first: a
    # MemoryError after it (the last line's time sliced) left the k lines
    # in the batch, and the crash handler's flush wrote them again.
    nq = 0
    q = log["seg_tc"]
    if log["new_seg"] and q:
        last = batch[k - 1][-17:-1]
        while nq < len(q) and q[nq] is not None and q[nq] <= last:
            nq += 1
    mem = False                          # (a later chunk out of memory)
    try:
        pts = 0
        if creating:
            start = len(HEADERB)
            f = open(path, "wb")
            f.write(HEADERB.encode())
        else:
            f = open(path, "r+b")
            if not _seek_footer(f, old, old == log["good_size"]):
                # the end is not where the last write left it: after a
                # guessed newline a record could be split, and its half
                # read as points. The batch goes on in a new file.
                log["wfails"] = max(log["wfails"], WFAIL_SWITCH - 1)
                raise OSError(5)
            try:
                start = f.tell()
            except (OSError, AttributeError):
                start = (old or 0) - len(FOOTER)
                if start < 0:
                    start = 0
        while True:
            f.write(body)
            pts += len(body)
            if _kq[0] >= k:
                break
            body = None
            body = _kb_part(batch, k, seg)
        body = None
        expected = start + pts + len(FOOTER)
        f.write(FOOTER.encode())
        wrote = True
    except (OSError, MemoryError) as e:
        body = None
        mem = isinstance(e, MemoryError)
        if mem:
            gc.collect()
        _werr("encode" if mem else "write", e)
    if f is not None:
        try:
            f.close()
            close_ok = True
        except OSError as e:
            _werr("close", e)
    landed = wrote and close_ok
    if not landed:
        gone = False                     # (only on ENOENT)
        try:
            sz = os.stat(path)[6]
        except OSError as e:
            sz, gone = None, _enoent(e)
        if expected is not None and sz == expected:
            landed = True
        else:
            log["wfail"] = True          # (retries wait FLUSH: log_tick)
            if creating:
                log["created"] = False
                log["good_size"] = None
            elif gone or sz == 0:
                # Only rewrite if the file is definitely missing or empty.
                # A transient stat failure (sz is None) must not flip
                # created=False — the next wb would wipe the trip.
                log["created"] = False
                log["good_size"] = None
            # a file that keeps failing (a bad block) is left as it is,
            # its points up to the last good footer upload; the batch goes
            # on in a new file named after its first line. A create that
            # keeps failing goes on under another name too (one second
            # on when that is the same: the name itself may be what
            # fails); all it held is in the batch.
            # (a chunk out of memory is no failing file: three in a row
            # left a good file for a new one, the track split, "failing")
            if not mem:
                log["wfails"] = log["wfails"] + 1
            if not mem and log["wfails"] >= WFAIL_SWITCH and batch:
                tc = batch[0][-17:-1]
                name = _tcn(tc)
                if creating:
                    if path[-19:-4] == name:
                        s = _kb_time(tc.encode(), 0)
                        if s is not None:
                            tc = _stamp_s(s + 1)[1]
                            name = _tcn(tc)
                    _rm(path)
                    clean = not _file_size(path)
                else:
                    clean = _cut_to_footer(path, log["good_size"], sz) is not False
                # an old file not cut back to its footer (the cut failed
                # too, the same block) would go up with the new one's lines:
                # the switch waits for a cut, up to WFAIL_SWITCH more tries,
                # then goes on anyway (a file that fails for good)
                if not clean and log["wfails"] < 2 * WFAIL_SWITCH:
                    return
                print("track file failing, new file", path, name)
                _diag_write("track %s failing %s new %s%s" % (
                    "create" if creating else "write", path[-19:], name,
                    "" if clean else ", its tail not cut"))
                seg = log["new_seg"], log["seg_tc"]
                start_file(name, fx)
                log["new_seg"], log["seg_tc"] = seg   # (a fix loss in
            return                                    # the batch stays one)
    _rt_feed(batch, k)                   # (the lines that landed: raises nothing)
    del batch[:k]                        # first: nothing allocated before it
    log["created"] = True
    log["good_size"] = expected
    log["wfail"] = False
    log["wfails"] = 0
    log["points"] += k
    up["left"] += k
    if log["new_seg"] and q:
        del q[:nq]                       # the breaks written
        log["new_seg"] = bool(q)

def _werr(what, e=None):
    """Count a track write error (log["err"]). The first one, then one per
    WERR_EVERY, also goes to the diag log: the count alone was shown
    nowhere. Never raises (it runs when heap is short too)."""
    log["err"] += 1
    try:
        if not _recent_s(log["err_at"], WERR_EVERY // 1000):
            log["err_at"] = _stamp()
            _diag_write("track %s error %d %r %s" % (
                what, log["err"], e, (log["file"] or "-")[-19:]))
    except MemoryError:
        pass

def _herr(what, e=None):
    """Count a day history write error (log["herr"], hist_errs on the
    status page), the first, then one per WERR_EVERY, in the diag log as
    _werr's: the count alone was shown nowhere, and a bug in the history
    copy (it takes any error: the commit after it is what matters) only
    printed. Never raises."""
    log["herr"] += 1
    try:
        if not _recent_s(log["herr_at"], WERR_EVERY // 1000):
            log["herr_at"] = _stamp()
            _diag_write("track hist error %d %r %s" % (log["herr"], e, what))
    except MemoryError:
        pass

def _cut_to_footer(path, good, sz):
    """Before a failing file is left: a failed append that partly landed
    left the start of the batch after the last good footer, without one,
    and the new file takes the whole batch. Write FOOTER back where it was
    (once, best effort): readers stop there, so the tail is not uploaded
    twice. Only a footer written this session (good), the file no shorter
    (a write that landed the footer's bytes at most left the size). A
    footer still there is not written again (a copy-on-write of a failing
    file for nothing). True when the footer is in its place, False when
    the cut failed, None when there is none to make (no footer of this
    session, a file shorter than it: cut short from outside)."""
    n = len(FOOTER)
    if good is None or good < n or sz is None or sz < good:
        return None
    foot = FOOTER.encode()
    try:                                 # (read only first: a file that will
        with open(path, "rb") as f:      # not open for writing, nothing of the
            f.seek(good - n, 0)          # write landed, still has its footer)
            if f.read(n) == foot:
                return True
    except (OSError, MemoryError):
        pass
    f = None
    ok = False
    try:
        f = open(path, "r+b")
        f.seek(good - n, 0)
        f.write(foot)
        ok = None                        # (landed once the close is through)
    except (OSError, MemoryError) as e:  # (it runs when heap is short too)
        _werr("cut", e)
        ok = False
    if f is not None:
        try:
            f.close()
            if ok is None:
                ok = True
        except OSError as e:
            _werr("cut", e)
            ok = False
    return ok is True

# The day history (/hist/YYYYMMDD.trb, a UTC day) is a .trb track: the
# header, then each append (an uploaded batch's _hist_batch) the records a
# flush writes, a keyframe first, so an
# append's start is where a read can start (the page asks from one,
# /hist?o=); one footer at the end, overwritten by the next append, as a
# track flush overwrites it. ~7 bytes a point instead of ~50 as text. An
# append cut short (ENOSPC mid-write, a write error) leaves no footer: the
# next one starts with HIST_SEP. Its 0xC0 bytes are no record a delta
# makes (bit 6: a field no line has, the state goes) and run past 31
# bytes: a delta cut there is a damaged record, not one ended by the next
# keyframe's "=" read as its varint (that keyframe's points lost); its
# newline ends a cut keyframe (no point: a time field ending in 0xC0s). A
# day older firmware began as text (.trk) stays text to its end (no
# conversion of a day in the loop, no second file a day): it is served
# as before, and goes after HIST_DAYS.
HIST_SEP = b"\xc0" * 32 + b"\n"
_kh = [None] * 12                        # the day history encoder's state
_khn = [0]                               # its deltas since the keyframe

def _hist_begin(hf, day):
    """Open the day's file for an append into st ([file, made now, text,
    the .trb records not yet written]): its .trb, a new one with the
    header, an old one at its footer (HIST_SEP first without one); a .trk
    older firmware began the day with at its end (a newline first after a
    cut line, see _tail_nl). A .trb append begins with a keyframe."""
    path = HIST_DIR + "/" + day
    size = _size_or_gone(path + TRB)     # (an EIO: this batch's drawing goes,
    text = False                         # not the day: _hist_batch counts it)
    if size is None:
        n = _size_or_gone(path + ".trk")
        if n is not None:
            path, size, text = path + ".trk", n, True
    if not text:
        path += TRB
    hf[1], hf[2] = size is None, text
    if size is None:
        try:
            os.mkdir(HIST_DIR)
        except OSError:
            pass                         # (there already)
    if text:
        cut = not _tail_nl(path, size)
        hf[0] = open(path, "ab")
        if cut:
            hf[0].write(b"\n")
        return
    _kh[0] = None                        # (a keyframe first)
    _khn[0] = 0
    hf[3] = bytearray()
    if not size:
        hf[0] = open(path, "wb")
        hf[0].write(HEADERB.encode())
        return
    f = hf[0] = open(path, "r+b")
    n = len(FOOTER)
    if size >= n:
        f.seek(size - n)
        if f.read(n) == FOOTER.encode():
            f.seek(size - n)
            return
    f.seek(size)
    f.write(HIST_SEP)

def _hist_line(hf, buf, a, b):
    """The text line buf[a:b] (its newline last) into the open day file:
    a .trk's as it is; a .trb's as the next record (as _kb_part: a delta,
    or a keyframe first and after KEY_EVERY deltas), written a KB_CHUNK at
    a time."""
    if hf[2]:
        hf[0].write(memoryview(buf)[a:b])
        return
    out = hf[3]
    if _khn[0] < KEY_EVERY and _kb_delta(buf, _kh, out, a, b):
        _khn[0] += 1
    else:
        out.append(61)                   # =
        out += memoryview(buf)[a:b]
        _kb_parse(buf, a, b - 1, _kh)
        _khn[0] = 0
    if len(out) >= KB_CHUNK:
        hf[0].write(out)
        hf[3] = bytearray()

def _hist_close(hf, ok=True):
    """End the append in st: a .trb's last records and its footer, only if
    every write before landed (after a failed one the next append's
    HIST_SEP ends what it cut), then close. A new day's file drops the
    days beyond HIST_DAYS."""
    f, new, text, out = hf
    hf[0], hf[1], hf[3] = None, False, None
    if f is not None:
        if ok and not text:
            try:
                out += FOOTER.encode()
                f.write(out)
            except (OSError, MemoryError) as e:
                _herr("close", e)
        try:
            f.close()
        except OSError as e:             # (LittleFS commits on close: the
            _herr("close", e)            # batch's points are not drawn)
    if new:
        try:                             # the days beyond the newest
            ns = _hist_names()           # HIST_DAYS go (a day written as
            ds = sorted(set(n[:8] for n in ns))  # .trk and .trb is one)
            t = _hist_today()
            if t is not None:
                ds = [d for d in ds if d <= t]
            if len(ds) > HIST_DAYS:
                cut = ds[-HIST_DAYS]
                for n in ns:
                    if n[:8] < cut:
                        _rm(HIST_DIR + "/" + n)
        except (OSError, MemoryError) as e:
            _herr("trim", e)

def _tail_nl(path, size):
    """False when a non-empty file does not end in a newline: an append
    cut short (ENOSPC mid-write) left half a line there, and the next
    append would join it to its first line as one ghost point (a TRK2 line
    with a latitude for a longitude). The append then starts with a newline:
    the half line stays a line of its own, skipped by every reader."""
    if not size:
        return True
    f = None
    try:
        f = open(path, "rb")
        f.seek(size - 1)
        return f.read(1) == b"\n"
    except OSError:
        return True                      # (unreadable: append as before)
    finally:
        if f is not None:
            try:
                f.close()
            except OSError:
                pass

def _hist_today():
    """The last UTC day the history takes, YYYYMMDD: the clock's (GPS, else
    NTP) and one more (a clock a second behind, a flipped date digit, an
    NTP-only clock), or None. A line's time that leapt further ahead (a
    digit flipped past the checksum) must not make its day the newest: it
    would stay while today went under pressure."""
    c = log["clock"] or _ntp[0]
    return None if c is None else _stamp_s(_utc_s(c) + 86400)[1][:8]

def _hist_names():
    """The day history's files, the oldest first: YYYYMMDD.trb, and .trk
    from older firmware. OSError without HIST_DIR."""
    return sorted(n for n in os.listdir(HIST_DIR)
                  if len(n) == 12 and n[8:] in (TRB, ".trk")
                  and n[:8].isdigit())

def _hist_trim(want):
    """Older days go, a day at a time (its .trb and .trk both; the newest
    day, today's, stays) until want bytes are free: the day history yields
    to the tracks. -> free bytes, or None."""
    room = free()
    try:
        days = _hist_names()
    except OSError:
        return room
    t = _hist_today()
    if t is not None and room is not None and room < want:
        for n in [n for n in days if n[:8] > t]:   # (after the clock's day
            _rm(HIST_DIR + "/" + n)                # first: a time that leapt)
        days = [n for n in days if n[:8] <= t]
        room = free()
    while room is not None and room < want and days and days[0][:8] != days[-1][:8]:
        d = days[0][:8]
        while days[0][:8] == d:
            _rm(HIST_DIR + "/" + days.pop(0))
        room = free()
    return room

def _dist(lat1, lon1, lat2, lon2):
    """Metres between two nearby positions (equirectangular approximation),
    across 180° too."""
    x = (math.radians(_dlon(lon2 - lon1))
         * math.cos(math.radians((lat1 + lat2) / 2)))
    y = math.radians(lat2 - lat1)
    return 6371000 * math.sqrt(x * x + y * y)

def _fresh_hdop(now, fx=None):
    """HDOP from a GGA in this epoch (not the previous fix)."""
    hdop, at = _fv(fx, "hdop"), _fv(fx, "hdop_at")
    if hdop is None or at is None:
        return None
    if _age(now, at) >= _rcv.STALE:
        return None
    return hdop

def _fresh_std(now, fx=None):
    """The receiver's error estimate (GST) of the last GST_HOLD ms, or
    None. Only this second's left 4 of 5 seconds unfiltered at 1 Hz (GST
    comes once per 5 epochs) and every second whose GST was lost."""
    std, at = _fv(fx, "std"), _fv(fx, "std_at")
    if std is None or not _recent(now, at, GST_HOLD):
        return None
    return std

def _usable(now, fx=None):
    """The current fix is good enough to use: HDOP at most MAX_HDOP and the
    receiver's own error estimate at most GPS_MAX_STD (each when known).
    The estimate catches a cold start converging: measured on the board, a
    good HDOP (1.2-3) while the position walked in from 230 m away. A UC6580's
    estimate does not catch it (0.3 m while 4 m off, measured), so its first
    UC_SETTLE after a start count only moving: 15 m off at first, 5 m after
    a minute (measured), which standing still draws as a walk."""
    if _rcv.unsettled(now):
        return False
    hdop = _fresh_hdop(now, fx)
    if hdop is not None and hdop > MAX_HDOP:
        return False
    std = _fresh_std(now, fx)
    return std is None or std <= GPS_MAX_STD

def _sample_line(now, prev, last_t, fx=None):
    """(time, track line) for the current fix, or None: not usable (try
    again next SAMPLE), same second as last_t, or within MIN_MOVE of
    prev."""
    if not _usable(now, fx):
        return None
    lat, lon = _fv(fx, "lat"), _fv(fx, "lon")
    t = _fv(fx, "date") + "T" + _fv(fx, "time") + "Z"
    if (last_t is not None and t <= last_t) or (prev is not None and _dist(
            prev[0], prev[1], lat, lon) <= MIN_MOVE):
        return None
    return t, _line("%.6f,%.6f" % (lat, lon), _fields(now, fx), _tc(fx))

TIME_BACK = 60                           # s a fix behind the last line ...
TIME_BACK_N = 10                         # ... in a row that drop last_t

def _time_back(t, lt, fx=None):
    """A usable fix at t (ISO), older than the last line written, lt.
    TIME_BACK_N of them in a row over TIME_BACK behind it: lt was a false
    time (a corrupt digit past the checksum, a receiver glitch: plausible
    but ahead), and no line, no anchor point either, could follow it until
    the real time caught up (an hour ahead: an hour) or a reboot. A receiver
    does not step back a minute ten seconds running. It goes: the lines
    stamped after t still in RAM with it, a new segment starts, a diag
    line notes it. (One fix that far behind is a glitch itself: skipped
    as before.)"""
    a, b = _dt_s(t, t, 11), _dt_s(lt, lt, 11)
    if a is None or b is None or b - a <= TIME_BACK:
        log["back"] = 0
        return
    log["back"] += 1
    if log["back"] < TIME_BACK_N:
        return
    tc = _tc(fx)
    bt = log["batch"]
    while bt and bt[-1][-17:-1] > tc:
        bt.pop()
        log["drop"] += 1
    q = log["seg_tc"]
    while q and (q[-1] is None or q[-1] > tc):
        q.pop()
    _say("time back %s %s" % (lt, t))
    log.update(last_t=None, prev=None, back=0)
    _seg_break()

def _line(c, fl, tc):
    """A track line: c "lat,lon", fl from _fields, tc compact time."""
    return "%s,%s,%s,%s\n" % (c, fl[0], fl[1], tc)

def _fields(now, fx=None):
    """The elevation and HDOP fields of a track line for the current fix (""
    when not known). Also what the departure ring keeps (_RE)."""
    ele = ""
    alt = _fv(fx, "alt")
    ft = _fv(fx, "dim")
    if alt is not None and ft != 2 and ft != 0:
        ele = "%.1f" % alt
    hdop = _fresh_hdop(now, fx)
    return ele, "" if hdop is None else "%.1f" % hdop

def _seg_break():
    """Start a new segment with the next line recorded. A break whose line
    is still in RAM stays queued (one per loss: two short losses within
    held lines were one break). Breaks whose lines are gone (the MAX_BATCH
    cap, a full flash) all go before the oldest line left, so one of them
    stands for all: the queue never outgrows the batch. (A cap of 16 that
    dropped the oldest lost real breaks: 18 losses while writes failed
    wrote 16.)"""
    q = log["seg_tc"]
    if not q or q[-1] is not None:
        b = log["batch"]
        t0 = b[0][-17:-1] if b else None
        i = 0
        while i + 1 < len(q) and (t0 is None or q[i + 1] <= t0):
            i += 1
        del q[:i]
        q.append(None)
    log["new_seg"] = True

def _record(s, pos, fx=None):
    """Append a (time, line) from _sample_line; pos is what it wrote."""
    if log["file"] is None:
        start_file(fx=fx)
    q = log["seg_tc"]
    if q and q[-1] is None:
        q[-1] = s[1][-17:-1]             # the new segment starts here
    log["last_t"], log["prev"] = s[0], pos
    log["batch"].append(s[1])
    if len(log["batch"]) > MAX_BATCH:
        log["batch"].pop(0)
        log["drop"] += 1                 # (it went silently)

def _tc(fx=None):
    """Compact UTC time of the current fix, as in a track line."""
    return (_fv(fx, "date").replace("-", "") + "T" + _fv(fx, "time").replace(":", "")
            + "Z")

def _fix_s(fx=None):
    """Seconds since 2020 of the current fix's date and time, or None."""
    return _dt_s(_fv(fx, "date"), _fv(fx, "time"))

def _still_gap(now, pos):
    """A usable fix at pos after a gap: the device may be elsewhere, so an
    open window (or the previous one) from before it must not prove a stay
    here. Beyond STILL_EXIT both restart. Past STILL_RADIUS only the open
    window does: indoors usable fixes come seconds apart and drift past it,
    and dropping the previous window too released the drift held for the
    stay (measured: 13 lines in 14 min). Also run when a fix returns after
    NEW_FILE_GAP, before the old track closes: what a window from before
    the gap held is then written to it, not moved on to the next file.
    Once per gap (gap_seen): _sample runs it again with last_at unchanged,
    and that second verdict, taken from the previous window once the open
    one was gone, released into the new file what the first had held over.
    The previous window is measured on its own: one beyond STILL_EXIT
    cannot pair with anything here, even when the open window is near
    (judged by the open window alone, its lines moved on to the new file)."""
    sl = log["still"]
    la = sl["last_at"]
    if la is None or _age_s(la) <= STILL_GAP // 1000 or sl["gap_seen"] == la:
        return
    sl["gap_seen"] = la
    p = sl["prev"]
    pd = None if p is None else _dist(p[0][0], p[0][1], pos[0], pos[1])
    gd = pd
    if sl["win_at"] is not None and sl["sum"][2]:
        r, sm = sl["ref"], sl["sum"]
        gd = _dist(r[0] + sm[0] / sm[2], r[1] + sm[1] / sm[2], pos[0], pos[1])
    if gd is not None and gd > STILL_EXIT:
        sl.update(win_at=None, prev=None, hold_tc=None)
        return
    if pd is not None and pd > STILL_EXIT:
        sl["prev"] = None
    if gd is not None and gd > STILL_RADIUS:
        # do not mix two places in one window, but keep holding what the
        # restarted window held: a pair that proves a stay drops it
        if sl["win_at"] is not None and sl["hold_tc"] is None:
            sl["hold_tc"] = sl["win_tc"]
        sl["win_at"] = None

def _still_window(now, pos, fx=None):
    """Fold a usable fix into the current centroid window. When the window
    is full and its centroid lies within STILL_RADIUS of the previous one,
    return (anchor, stay start time, fixes); a walk moves the centroid on.
    Sums are offsets from the window's first fix: single-precision floats
    cannot add up 60 latitudes to the metre."""
    sl = log["still"]
    _still_gap(now, pos)
    q = sl["pos"]                        # (the usable fix before this one)
    sl["last_at"], sl["pos"] = _stamp(), pos
    sl["last_home"] = bool(at_home())    # (a hold is dropped only at home)
    if sl["win_at"] is None:
        sl.update(win_at=_stamp(), win_tc=_tc(fx), ref=pos, sum=[0.0, 0.0, 0],
                  qs=[])
    r, sm, qs = sl["ref"], sl["sum"], sl["qs"]
    while len(qs) < 3 and (_age_s(sl["win_at"])
                           >= (len(qs) + 1) * (STILL_WINDOW // 4000)):
        qs.append((sm[0], sm[1], sm[2]))   # the sums up to each quarter's
                                           # end (see _walking)
    sm[0] += pos[0] - r[0]
    sm[1] += _dlon(pos[1] - r[1])        # (across 180° too)
    sm[2] += 1
    if _age_s(sl["win_at"]) < STILL_WINDOW // 1000:
        return None
    if sm[2] < STILL_MIN_FIXES and _age_s(sl["win_at"]) < STILL_WINDOW_MAX // 1000:
        return None                      # sparse fixes: collect more first
    c = None
    if sm[2] >= STILL_MIN_FIXES:
        c = (r[0] + sm[0] / sm[2], _dlon(r[1] + sm[1] / sm[2]))
        if (_spread(r, c, pos, STILL_EXIT // 2) or _walking(r, sm, qs)
                or (q is not None
                    and _dist(c[0], c[1], pos[0], pos[1]) > STILL_EXIT
                    and _dist(c[0], c[1], q[0], q[1]) > STILL_EXIT)):
            # its ends STILL_EXIT apart through the centroid: a drive (a
            # bike from ~10 km/h) pairs with nothing. Out and back with the
            # turn near a window boundary, the way out and the way back had
            # the same centroid, and the pair dropped the held lines as
            # drift. (Indoor drift that strays 80 m still pairs.) A walk
            # spreads less: its quarters advance steadily (_walking), and
            # one such window of a pair is enough to tell. A walk on, a
            # walk in before a stop: the pair waits for two calm windows.
            # Nor a window that ends with the board gone, its last two
            # fixes out past STILL_EXIT: a car stopped for ~90 s and
            # off again (its centroid at the stop), the stay it proved
            # dropped the first 150-290 m of the drive as drift.
            c = None
            sl["hold_tc"] = None
    else:
        # a sparse window timed out at STILL_WINDOW_MAX: it proves no stay
        # and pairs with nothing, so what the restarted window held is
        # track. Kept, an hour of fixes 40 s apart indoors stayed in RAM
        # (HOLD_RELEASE never came: every sparse fix refreshed last_at).
        sl["hold_tc"] = None
    prev = sl["prev"]
    sl["prev"] = (c, sl["win_tc"], sm[2]) if c else None
    sl["win_at"] = None
    sl["wins"] += 1                      # windows closed (see _still_active)
    if c is None or prev is None:
        return None
    if _dist(prev[0][0], prev[0][1], c[0], c[1]) > STILL_RADIUS:
        sl["hold_tc"] = None             # moving: what it held is a track
        return None
    w = prev[2] + sm[2]
    a = (prev[0][0] + (c[0] - prev[0][0]) * sm[2] / w,
         _dlon(prev[0][1] + _dlon(c[1] - prev[0][1]) * sm[2] / w))
    return a, prev[1], w, c

STILL_TREND = 35                         # m a walking window's quarters
                                         # advance at least (_walking)

def _walking(r, sm, qs):
    """True when a window (first fix r, sums sm, qs the sums at the end of
    its first three quarters) is a walk: its four quarter centroids
    advance steadily, the first to the last at least STILL_TREND (a walk
    from ~0.8 m/s), nearly straight (at least 9/10 of the path through the
    four) and no step under 7/10 of an even third. It pairs with nothing:
    at walking pace two windows of an out and back, or a walk and the stop
    after it, have centroids within STILL_RADIUS. Indoor drift wanders to
    and fro (in the tests' energetic drift a window in ~40 looks like
    this: the stay comes a window later); a window with the turn inside
    is not steady either (its neighbours are). A quarter with fewer than
    two fixes (sparse fixes) tells nothing."""
    if len(qs) < 3:
        return False
    q = []
    p0 = p1 = 0.0
    pn = 0
    for x in (qs[0], qs[1], qs[2], sm):
        n = x[2] - pn
        if n < 2:
            return False
        q.append((r[0] + (x[0] - p0) / n, r[1] + (x[1] - p1) / n))
        p0, p1, pn = x[0], x[1], x[2]
    d = _dist(q[0][0], q[0][1], q[3][0], q[3][1])
    if d < STILL_TREND:
        return False
    a = _dist(q[0][0], q[0][1], q[1][0], q[1][1])
    b = _dist(q[1][0], q[1][1], q[2][0], q[2][1])
    c = _dist(q[2][0], q[2][1], q[3][0], q[3][1])
    return 10 * d >= 9 * (a + b + c) and 30 * min(a, b, c) >= 7 * d

def _still_heartbeat(now, fx=None):
    """Write the anchor as a point. It is stamped with the last fix's GPS
    time plus the ticks since, so no fix is needed: the anchor is known, and
    indoors there is rarely a usable fix. Skipped when that time would not
    come after the last line written (tried again a second later, not
    every pass). Its time goes into log["hb"], what tells the route an
    anchor from a fix without height and HDOP (_rt_line)."""
    sl = log["still"]
    c = log["clock"]
    if c is None:
        return
    t, tc = _stamp_s(_utc_s(c))
    if log["last_t"] is not None and t <= log["last_t"]:
        sl["hb_at"] = _stamp() + (1000 - STILL_HEARTBEAT) // 1000
        return
    a = sl["anchor"]
    sl["hb_at"] = _stamp()
    if log["file"] is None:
        start_file(_tcn(tc), fx)
        sl["file_at"] = _stamp()
    h = log["hb"]
    h.append(tc)
    if len(h) > 8:                       # (never fed to the route: no TFT)
        h.pop(0)
    _record((t, _line("%.6f,%.6f" % (a[0], a[1]), ("", ""), tc)), a, fx)

def _beat_held(sl, capped=True):
    """Whether an anchor point now would land in a way out. Away (capped)
    each of these holds it for 2 * STILL_HEARTBEAT since the last one at
    most, and drift out past STILL_NEAR delays it no more than that
    (indoors it stays out there, under STILL_EXIT, for hours: an office
    stay wrote no anchor point for 98 min, 2026-11-20), but not while a
    departure shows (a run out in the ring, receding samples counted): an
    anchor 31 s into the walk off lost its first 50 m. At home (not
    capped, an hour between them) they hold it as long as they last.
    - The last usable sample recently out past STILL_NEAR: an excursion
      or a departure.
    - STILL_BACK after that: the car driving off past the anchor comes
      within STILL_NEAR for seconds, and the anchor point due since the
      walk to it, with the way out after it, read 44 m in a second
      (158 km/h, measured).
    - The departure ring a run already STILL_RUN out from where it began,
      still within STILL_NEAR: a walk out. The anchor stamped 25 s into it
      at 2 m/s, and the way out (only what follows the last line) began
      52 m out, a second after it."""
    recent = _recent_s(sl["last_at"], STILL_GAP // 1000)
    n, i = sl["rn"], sl["ri"]
    run = (n > 1 and recent
           and _RF[3 * ((i + n - 1) % len(_RT)) + 2] - _RF[3 * i + 2]
           >= STILL_RUN)
    hb = sl["hb_at"]
    young = not capped or (hb is not None
                           and _age_s(hb) < 2 * STILL_HEARTBEAT // 1000)
    if recent and sl["last_d"] > STILL_NEAR and (
            young or run or sl["far_n"] or sl["far_slow"]):
        return True
    if young and (run or _recent_s(sl["far_at"], STILL_BACK // 1000)):
        return True
    return not capped and (
        sl["gap_at"] is not None
        or _recent_s(sl["veto_at"], STILL_HOME_HOLD // 1000))

def _still_beat(now):
    """Heartbeat of a stay away from home (at home only with
    HOME_HEARTBEAT, every that often), run every tick, fix or not; never
    in a way out (_beat_held)."""
    sl = log["still"]
    if not sl["active"]:
        return
    if at_home():
        _still_close(now)                # also after a WiFi drop, fix or not
        if not sl["home"] and (sl["last_at"] is None
                               or _age_s(sl["last_at"]) >= STILL_GAP // 1000):
            # an away stay, home now, no usable fix for STILL_GAP: the way
            # home had no fix (a train) and no WiFi verdict (no access
            # points at that place). Left running, it stamped the old place
            # next morning and swallowed a trip that did not recede from it
            _say("stay end: home without a fix")
            log["still"] = _still_state()
            _rt_trip()
        elif sl["home"] and HOME_HEARTBEAT:
            # opt-in: an anchor point every HOME_HEARTBEAT, so a night at
            # home is a visit, not 22 h without a point (2026-11-21). The
            # first one HOME_HEARTBEAT after the stay began; _still_close
            # uploads it on a later tick. Only on the home WiFi: at_home()
            # holds while the radio sleeps (sleep_home), also for a board
            # that left without a fix and no departure, and that stay
            # stamped home all the while; with the radio asleep a round is
            # asked for first. Never in a way out (_beat_held, no cap: the
            # walk out of the door lost its start to an anchor point)
            if sl["hb_at"] is None:
                sl["hb_at"] = _stamp()
            elif (_recent_s(sl["hb_at"], HOME_HEARTBEAT // 1000)
                  or _beat_held(sl, False)):
                pass
            elif wifi["state"] != "connected" or not wifi["home"]:
                if wifi["state"] == "sleep":
                    wifi["wake"] = True
            else:
                h = log["hb"]
                h0 = h[-1] if h else None
                _still_heartbeat(now)
                if (h[-1] if h else None) == h0:
                    # not written (its time not after the last line): again
                    # in a second, not one STILL_HEARTBEAT short of an hour
                    sl["hb_at"] = _stamp() + (1000 - HOME_HEARTBEAT) // 1000
        return
    # Rotate the anchor file so the stay uploads while it lasts (the usual
    # age-close is off during a stay: it would open a file per heartbeat).
    if (log["file"] is not None and sl["file_at"] is not None
            and wifi["state"] == "connected"
            and _age_s(sl["file_at"]) >= STILL_FILE // 1000):
        _still_close(now)
    if _recent_s(sl["hb_at"], STILL_HEARTBEAT // 1000) or _beat_held(sl):
        return
    _still_heartbeat(now)

def _still_close(now=None, fx=None):
    """Home stay: finish the track so it uploads; nothing more is written
    until the stay ends (but a HOME_HEARTBEAT anchor point). A retry from
    the tick (now given) flushes at most every FLUSH while a flush keeps
    failing, as the main path does."""
    if log["file"] is None:
        return
    if (log["batch"] and now is not None
            and _age_s(log["last_flush"]) < FLUSH // 1000):
        return
    _close_file(fx)                      # nothing is held during a stay

def _close_file(fx=None):
    """Close the open track so it uploads, without the drift a stay still
    being proven holds in RAM (a full flush would write it): what is held
    moves on to a new file named after its first line. False when the
    write failed and the file stays open."""
    if log["batch"]:
        flush(_still_hold(), fx)         # (with nothing to write, flush
                                         # would create an empty file)
    b = log["batch"]
    if not b:
        log.update(file=None, created=False, file_at=None, new_seg=False,
                   wfail=False)          # (a failure of a batch now gone)
        del log["seg_tc"][:]
        return True
    hold = _still_hold()
    if hold is not None and b[0][-17:-1] >= hold:
        tc = b[0][-17:-1]
        seg = log["new_seg"], log["seg_tc"]
        path = log["file"]
        if log["wfail"] and path is not None:
            # a failed write left part of these lines in the old file
            # (as flush's own switch to a new file): its footer back, or
            # a create that never landed gone, else they went up twice.
            # When that fails too (the heap still short), the file stays
            # open: the next flush writes from its good size over the tail
            if log["created"]:
                if _cut_to_footer(path, log["good_size"],
                                  _file_size(path)) is False:
                    return False
            else:
                _rm(path)
                if _file_size(path):
                    return False
        start_file(_tcn(tc), fx)
        log["new_seg"], log["seg_tc"] = seg    # a break among the held lines
        return True
    return False

STILL_RUN = 15                           # m a way in or out held among the
                                         # drift came closer or receded

def _still_run(g, n):
    """A way in or out of g m over n lines is a walk, not drift: at least
    STILL_RUN over STILL_RUN // 2 lines, 0.7 m a line on average (drift
    that creeps closer for minutes is no way in)."""
    return g >= STILL_RUN and n >= STILL_RUN // 2 and 10 * g >= 7 * n

def _line_m(c, line):
    """Metres from c to the position of a track line."""
    f = line.split(",", 2)
    return _dist(c[0], c[1], float(f[0]), float(f[1]))

def _still_enter(now, anchor, since_tc, n, c=None, fx=None):
    """Become stationary at anchor. The stay's drift (lines from since_tc
    on, all still in RAM: see _still_hold) is dropped, and the track is
    closed, so the arrival uploads during the stay. Away from home the
    anchor opens a new file, now and then every STILL_HEARTBEAT.
    Kept: an excursion out past STILL_EXIT and back; the way in up to
    where it first comes within STILL_NEAR of c after that,
    the later window's centroid (the anchor, the mean of both, lies back
    along a way in that the earlier window still held), and on while it
    kept coming closer steadily (each line at most STILL_TOL farther and
    2 * STILL_TOL closer than the one before; drift jumps), when that took
    it STILL_RUN closer over STILL_RUN // 2 lines at least. At walking
    pace the last 50 m were dropped as drift (~36 s). Likewise the newest
    lines when they recede from c that way (each line at most STILL_TOL
    closer, its step at most 2 * STILL_TOL longer or shorter than the next
    one's: a car accelerates): the walk or drive on already began in the
    later window (a stop of ~90 s), its start no drift either. Then
    no anchor point is written now: it would come after them, back at the
    stop."""
    log["still"] = sl = _still_state()
    # last_at: the fix that proved it. None read as "no usable fix for
    # STILL_GAP" in _still_beat, so home WiFi coming up just after arriving
    # ended the stay as "home without a fix"
    sl.update(active=True, anchor=anchor, aref=anchor,
              n=min(n, STILL_MEAN_N), last_at=_stamp())
    b = log["batch"]
    i = 0
    while i < len(b) and b[i][-17:-1] < since_tc:
        i += 1
    if c is None:
        c = anchor
    # Never dropped: an excursion out past STILL_EXIT from c that came back
    # (a U-turn 240 m past the place, back to park went whole). Out there
    # by steps a vehicle makes (radially at most STILL_EXIT // 3 a line,
    # from within STILL_NEAR): a multipath spike out and back is drift.
    e, y, ok = i, None, True
    for q in range(i, len(b)):
        z = _line_m(c, b[q])
        if y is not None:
            if ok and y > STILL_EXIT >= z:
                e = q                    # (the first line back)
            ok = z <= STILL_NEAR or (ok and abs(z - y) <= STILL_EXIT // 3)
        y = z
    i = e
    x = None
    while i < len(b):
        x = _line_m(c, b[i])
        if x <= STILL_NEAR:
            break
        i += 1
    k = len(b)
    if i < k:                            # the way in, on while closing in
        j = i
        y = lo = x
        while j + 1 < k:
            z = _line_m(c, b[j + 1])
            if z > y + STILL_TOL or z < y - 2 * STILL_TOL:
                break
            j += 1
            y = z
            if z < lo:
                lo = z
        m = i                            # (arrived: the first line within
        while m < j and _line_m(c, b[m]) > lo + STILL_TOL:   # STILL_TOL of
            m += 1                                           # the closest)
        j = i                            # (where it began: before since_tc
        y = x                            # maybe, in the way in kept)
        while j > 0 and i - j < 2 * STILL_RUN and x - lo < STILL_RUN:
            z = _line_m(c, b[j - 1])
            if y > z + STILL_TOL or y < z - 2 * STILL_TOL:
                break
            j -= 1
            y = z
            if z > x:
                x = z
        if _still_run(x - lo, m - j):
            i = m + 1
    if i < k:                            # the way out, already begun
        j = k - 1
        y = hi = lo = _line_m(c, b[j])
        # (g: the step of the line after. A car's step changes by less than
        # 2 * STILL_TOL a second, drift jumps. Each step at most 10 m, as
        # before, dropped the drive off a car stop.)
        g = None
        while j > i:
            z = _line_m(c, b[j - 1])
            if z > y + STILL_TOL or (g is not None
                                     and abs(y - z - g) > 2 * STILL_TOL):
                break
            g = y - z
            j -= 1
            y = z
            if z < lo:
                lo = z
        m = k - 1                        # (where it left: the last line
        while m > j and _line_m(c, b[m]) > lo + STILL_TOL:   # within
            m -= 1                                           # STILL_TOL of
        if _still_run(hi - lo, k - 1 - m):                   # the closest)
            k = m
    out = len(b) - k
    del b[i:k]
    _still_close(fx=fx)                  # the arrival uploads during the stay
    if log["file"] is not None:
        sl["file_at"] = _stamp()         # close failed: rotate it later
    if at_home():
        sl["home"] = True
    elif out:
        sl["hb_at"] = _stamp()           # (the way out goes on: no anchor
                                         # point after it)
    elif wifi_wake():
        # the radio slept: the first anchor point waits for the round, so
        # a home stay is not stamped with one
        sl["hb_at"] = _stamp() + (WIFI_WAKE_WAIT - STILL_HEARTBEAT) // 1000
    else:
        _still_heartbeat(now, fx)
    _say("stay %s kept %d out %d" % ("home" if sl["home"] else "away", i, out))

def _stay_scan(aps, fx=None):
    """One WiFi scan during an away stay. The first STAY_WIFI_LEARN scans
    note the access points around; later scans that see one of them add
    the others seen with it (a walk into another wing of a big building),
    the STAY_WIFI_MAX most recently seen kept. Left out: our own networks
    and locally administered BSSIDs (phone hotspots, a train's WiFi), as
    they travel along. When none of them shows up in STAY_WIFI_MISS scans
    in a row over STAY_WIFI_GONE ms, the board has left, fix or not, and
    the stay ends: no more heartbeats stamped with the old anchor (without
    a fix on the way, e.g. inside a train, they went on until home).
    Without access points around, nothing changes."""
    sl = log["still"]
    if not STAY_WIFI or not sl["active"] or sl["home"]:
        return
    own = [_ssid_b(n) for n in wifi["nets"]]
    seen = []
    for rec in aps:
        raw = rec[0]
        b = rec[1]
        if (raw and raw in own) or not b or b[0] & 0x02:
            continue                     # (SSIDs compared as bytes: a
        seen.append((rec[3], b))         # Latin-1 one broke decode())
    known = sl["aps"]
    sl["ap_scans"] += 1
    n = sl["ap_scans"]
    learning = n <= STAY_WIFI_LEARN
    if not learning:
        if not known:
            return
        if not any(b in known for r, b in seen):
            sl["ap_miss"] += 1
            if sl["ap_miss_at"] is None:
                sl["ap_miss_at"] = _stamp()
            if (sl["ap_miss"] >= STAY_WIFI_MISS
                    and _age_s(sl["ap_miss_at"]) >= STAY_WIFI_GONE // 1000):
                print("stay left, wifi: none of %d access points in %d scans"
                      % (len(known), sl["ap_miss"]))
                _diag_write("stay left wifi aps %d scans %d"
                            % (len(known), sl["ap_miss"]))
                _still_leave(wifi_seen=True, fx=fx, why="wifi")
            return
        sl["ap_miss"], sl["ap_miss_at"] = 0, None
    for r, b in seen:                    # recency first, then signal
        v = n * 256 + r + 128
        if known.get(b, -1) < v:
            known[b] = v
    if len(known) > STAY_WIFI_MAX:
        keep = sorted(known.items(), key=lambda x: -x[1])
        sl["aps"] = dict(keep[:STAY_WIFI_MAX])

STILL_DEP_STEP = 2                       # m between departure-ring samples

def _ring_add(sl, now, pos, d, fx=None):
    """Keep a usable sample of the stay in the departure ring (see _RT),
    the oldest overwritten when it is full. A sample that falls back from
    the one before by more than STILL_TOL starts it over: the ring is
    always a run that kept receding, the way out (_still_leave). So does
    one within STILL_TOL of the anchor (at the place: the way out begins at
    the last one), and one within STILL_NEAR over STILL_GAP after the one
    before: in calm reception the stay's jitter rarely moved STILL_DEP_STEP
    or fell back, and a home stay (no anchor points, or an hour apart: nothing to cut it)
    wrote up to STILL_DEP_MAX samples hours old before the way out, the
    new track named after the first. One
    within STILL_DEP_STEP of the one before is left out: at walking pace
    the way out to STILL_EXIT and the samples that prove it took ~120 s,
    and the ring kept the last STILL_DEP_MAX (lost: the first ~30 m)."""
    s = _fix_s(fx)
    if s is None:
        return
    cap = len(_RT)
    n, i = sl["rn"], sl["ri"]
    if n:
        j = (i + n - 1) % cap
        k = 3 * j
        if (d + STILL_TOL < _RF[k + 2] or d <= STILL_TOL
                or (d <= STILL_NEAR
                    and s - sl["rt0"] - _RT[j] > STILL_GAP // 1000)):
            n = 0
        elif _dist(_RF[k], _RF[k + 1], pos[0], pos[1]) <= STILL_DEP_STEP:
            return
    if not n:
        sl["rt0"], i = s, 0
    if n < cap:
        j = (i + n) % cap
        n += 1
    else:
        j, i = i, (i + 1) % cap
    sl["rn"], sl["ri"] = n, i
    _RT[j] = s - sl["rt0"]               # (seconds since 2020 overflowed it
                                         # from 2088 on: see _sample)
    k = 3 * j
    _RF[k] = pos[0]
    _RF[k + 1] = pos[1]
    _RF[k + 2] = d
    k = RE_W * j
    _RE[k:k + RE_W] = _RPAD
    o = 0
    for x, w in zip(_fields(now, fx), (8, 4)):
        if len(x) <= w:                  # (beyond 99999.9 m: none)
            _RE[k + o:k + o + len(x)] = x.encode()
        o += w

def _still_leave(wifi_seen=False, fx=None, why="recede"):
    """End the stay and write the way out: the samples in the departure
    ring, a run whose distance from the anchor kept growing (within
    STILL_TOL), so the departure starts near the anchor, not STILL_EXIT
    away. With wifi_seen (the stay's access points are gone, _stay_scan)
    only if the last sample was out past STILL_NEAR: otherwise the ring is
    the stay's drift, and the stay just ends. why goes into the diag
    line (recede, gap, wifi): which of them ended a stay was not known
    afterwards (2026-11-20, an office stay silent for 98 min)."""
    sl = log["still"]
    n, i0, cap = sl["rn"], sl["ri"], len(_RT)
    _rt_trip(sl["rt0"] + _RT[i0] if n else None, fx)
    if (wifi_seen and n
            and _RF[3 * ((i0 + n - 1) % cap) + 2] <= STILL_NEAR):
        n = 0
    i = 0
    prev = None
    while i < n:
        j = (i0 + i) % cap
        i += 1
        p = (_RF[3 * j], _RF[3 * j + 1])
        t, tc = _stamp_s(sl["rt0"] + _RT[j])
        # only what comes after the last line (an anchor point may have
        # been written while the way out began), so the track stays in order
        if (log["last_t"] is not None and t <= log["last_t"]) or (
                prev is not None
                and _dist(prev[0], prev[1], p[0], p[1]) <= MIN_MOVE):
            continue
        k = RE_W * j
        line = _line("%.6f,%.6f" % (p[0], p[1]),
                     (_RE[k:k + 8].decode().strip(),
                      _RE[k + 8:k + 12].decode().strip()), tc)
        if log["file"] is None:
            start_file(_tcn(tc), fx)
        _record((t, line), p, fx)
        prev = p
    _say("stay end %s far %d" % (why, int(sl["far_d"])))
    wifi["sleep_home"] = False           # left: no longer at home
    log["still"] = _still_state()

def _still_recede(sl, d):
    """A sample out past STILL_EXIT: True when the departure is clear.
    Leaving: STILL_EXIT_N samples each gaining STILL_STEP (a vehicle), or
    STILL_EXIT_SLOW samples out there that gained STILL_SLOW_GAIN in all (a
    walk). A multipath jump that freezes 200 m away gains nothing, however
    long it lasts. A sample that falls back by more than STILL_TOL starts
    the count over."""
    if d + STILL_TOL < sl["far_d"]:
        sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
        return False
    sl["far_n"] = sl["far_n"] + 1 if d >= sl["far_d"] + STILL_STEP else 1
    if not sl["far_slow"]:
        sl["far_0"] = d
    sl["far_slow"] += 1
    sl["far_d"] = d
    return sl["far_n"] >= STILL_EXIT_N or (
        sl["far_slow"] >= STILL_EXIT_SLOW
        and d - sl["far_0"] >= STILL_SLOW_GAIN)

def _still_active(now, pos, fx=None):
    """One usable fix while stationary: watch for a departure (consecutive
    samples beyond STILL_EXIT that keep receding; an excursion that turns
    back does not count; after a fix gap, a place out there where the fixes
    stay together), refine the anchor (the heartbeat, the file rotation and
    the home close are _still_beat)."""
    sl = log["still"]
    a = sl["anchor"]
    d = _dist(a[0], a[1], pos[0], pos[1])
    gap = sl["last_at"] is not None and _age_s(sl["last_at"]) > STILL_GAP // 1000
    if d > STILL_EXIT and (gap or sl["gap_at"] is not None):
        # Back after a fix gap out past STILL_EXIT (a metro or a garage,
        # then sitting still elsewhere): there may be no receding run to
        # wait for, and a stay that waited for one wrote nothing at the new
        # place. It ends when the board clearly is elsewhere: one fix past
        # STILL_GAP_JUMP, or STILL_MIN_FIXES within STILL_RADIUS of the
        # first of them over STILL_GAP_STAY. Not on a single fix out past
        # 2 * STILL_EXIT, or STILL_EXIT_N of them (a 320 m outlier, a jump
        # frozen 200 m out for 3 s ended it and wrote a spike). Fixes coming
        # closer are not counted: a cold start converging walks in from
        # 230 m. (Not reset either: indoors at the new place the drift comes
        # closer every other second.) A receding run still leaves as usual.
        # The way out starts at the first fix after the gap.
        g = sl["gap_ref"]
        if g is None or _dist(g[0], g[1], pos[0], pos[1]) > STILL_RADIUS:
            if gap or g is None:         # (a walk on out there keeps
                sl["rn"] = 0             # its way out in the ring)
                sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
            sl.update(gap_ref=pos, gap_at=_stamp(), gap_n=1)
        elif d + STILL_TOL >= sl["last_d"]:
            sl["gap_n"] += 1
        _ring_add(sl, now, pos, d, fx)
        if _still_recede(sl, d) or d > STILL_GAP_JUMP or (
                sl["gap_n"] >= STILL_MIN_FIXES
                and _age_s(sl["gap_at"]) >= STILL_GAP_STAY // 1000):
            if not (sl["home"] and _home_veto(now, sl, fx)):
                sl["far_d"] = d          # (for the "stay end" line)
                _still_leave(fx=fx, why="gap")
                return
            sl.update(gap_ref=pos, gap_at=_stamp(), gap_n=1, far_n=0, far_d=0,
                      far_slow=0)
    elif d > STILL_EXIT and d + STILL_TOL >= sl["far_d"]:
        _ring_add(sl, now, pos, d, fx)
        if _still_recede(sl, d):
            if not (sl["home"] and _home_veto(now, sl, fx)):
                _still_leave(fx=fx)
                return
            sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
    else:
        sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
        sl.update(gap_n=0, gap_at=None, gap_ref=None)
        _ring_add(sl, now, pos, d, fx)
        if d <= STILL_NEAR:
            # pos into the anchor, a running mean over STILL_MEAN_N fixes.
            # Kept as an offset from a fixed reference: in single precision
            # a latitude plus a centimetre-sized step is the same latitude.
            r, o = sl["aref"], sl["aoff"]
            n = min(sl["n"] + 1, STILL_MEAN_N)
            o = (o[0] + (pos[0] - r[0] - o[0]) / n,
                 o[1] + (_dlon(pos[1] - r[1]) - o[1]) / n)
            sl.update(aoff=o, n=n, anchor=(r[0] + o[0], _dlon(r[1] + o[1])))
            sl["win_at"] = sl["prev"] = sl["move_c"] = None
        else:
            # Settled between STILL_NEAR and STILL_EXIT (out of the car and
            # into the building): STILL_MOVE_HITS calm window pairs at the
            # same place move the anchor there (fewer and an indoor
            # multipath shift moves it). Otherwise the anchor would stay
            # behind and no heartbeat would ever be written.
            wins = sl["wins"]
            hit = _still_window(now, pos, fx)
            if not (hit and _dist(a[0], a[1], hit[0][0], hit[0][1])
                    > STILL_NEAR):
                if sl["wins"] != wins:
                    sl["move_c"] = None  # a window without a matching pair:
                                         # the hits must be consecutive
            else:
                mc = sl["move_c"]
                if mc and _dist(mc[0][0], mc[0][1],
                                hit[0][0], hit[0][1]) <= STILL_RADIUS:
                    mc[1] += 1
                else:
                    mc = sl["move_c"] = [hit[0], 1]
                if mc[1] >= STILL_MOVE_HITS:
                    o = sl["anchor"]
                    a = hit[0]
                    sl.update(anchor=a, aref=a, aoff=(0.0, 0.0),
                              n=min(hit[2], STILL_MEAN_N), hb_at=None,
                              win_at=None, prev=None, move_c=None)
                    d = _dist(a[0], a[1], pos[0], pos[1])
                    sl["rn"] = 0         # (distances from the old anchor)
                    _ring_add(sl, now, pos, d, fx)
                    _say("stay moved %d m" % _dist(o[0], o[1], a[0], a[1]))
    sl["last_at"], sl["last_d"] = _stamp(), d
    if d > STILL_NEAR:
        sl["far_at"] = _stamp()
    home = bool(at_home())
    if home != sl["home"]:
        sl["home"] = home
        # (as if the last anchor were STILL_BACK ago: None let the next one
        # go at once, past far_at and STILL_RUN, and a home stay flipped
        # by a phone hotspot put an anchor in the first metres walked)
        sl["hb_at"] = _stamp() - (STILL_HEARTBEAT - STILL_BACK) // 1000

def log_tick():
    """Sample a position every SAMPLE ms and record it if it is more than
    MIN_MOVE m from the last recorded point; write a batch every FLUSH ms."""
    now = time.ticks_ms()
    # Close a long-open file on any WiFi network so it can upload while
    # logging continues. Offline driving keeps one file.
    if (log["file"] is not None and log["file_at"] is not None
            and wifi["state"] == "connected" and not log["still"]["active"]
            and log["points"] + len(log["batch"]) > 0
            and _age_s(log["file_at"]) >= NEW_FILE_GAP
            and _age_s(log["last_flush"]) >= FLUSH // 1000):
        _close_file()
    if log["full"]:
        # nothing is recorded: the queued seconds go as after a fix loss
        # (kept, thinned to its 8, they were written when space came back:
        # an hour old first, then a point every 6-11 min, straight lines)
        _rcv.drop_queue()
        log["seq"] = st.get("seq", log["seq"])
        if log["fix"] and not has_fix(now):
            # nothing is recorded, but NEW_FILE_GAP counts from the loss,
            # not from when space returns
            log["fix"], log["lost_at"] = False, _stamp()
        if _recent_s(log["full_check"], FULL_CHECK // 1000):
            return
        log["full_check"] = _stamp()
        room = free()
        if room is None or room < MIN_FREE:
            return
        log["full"] = False
        _diag_write("track full %s-%s drop %d" % (
            log["full_tc"] or "-", _diag_utc() if st.get("date") else "-",
            log["drop"]))
    if not has_fix(now):
        if log["fix"]:
            _drain_q()                   # valid seconds just before the loss
        else:
            _rcv.drop_queue()            # stale
        # the heartbeat after the queued seconds: a departure in them comes
        # first, and nothing is stamped before lines already written
        _still_beat(now)
        if log["fix"]:
            log["fix"], log["lost_at"] = False, _stamp()
            flush(_still_hold())
        elif log["batch"] and _age_s(log["last_flush"]) >= FLUSH // 1000:
            # anchor points written without a fix; and HOLD_RELEASE after
            # the last usable fix the drift hold is let go (_still_hold):
            # without a new fix the end of a trip would stay in RAM only
            # (lost on power-off)
            flush(_still_hold())
        return
    if not log["fix"]:
        log["fix"] = True
        if log["still"]["active"]:
            pass                         # same place: the stay keeps its file
        elif (log["file"] is not None and log["lost_at"] is not None
                and _age_s(log["lost_at"]) > NEW_FILE_GAP):
            _rt_trip()
            # A new file: the old track closes so it uploads, unless a write
            # failed within FLUSH (a rate limit). Before, a non-empty batch
            # and any flush within FLUSH kept it: held lines and the flush
            # tick without a fix made that the usual case. A window from
            # before the gap that cannot prove a stay here stops holding
            # first.
            if _usable(now):
                _still_gap(now, (st["lat"], st["lon"]))
            closed = (not (log["wfail"]
                           and _age_s(log["last_flush"]) < FLUSH // 1000)
                      and _close_file())
            log["prev"] = None           # first point of the new trip is kept
            if not closed or log["batch"]:
                # write failed: new trip in the same file; or what a stay
                # held moved on to the new file: the trip goes after it
                _seg_break()
        elif log["file"] is not None:
            _seg_break()
            log["prev"] = None           # start the segment with this fix
    if st["seq"]:
        _drain_q()
    elif (log["last_sample"] is None     # (st set directly: by the clock)
          or _age(now, log["last_sample"]) >= SAMPLE):
        log["last_sample"] = now
        _sample(now)
    _still_beat(now)                     # after the seconds (see above)
    if _age_s(log["last_flush"]) >= FLUSH // 1000:
        flush(_still_hold())

def _drain_q():
    """Sample every averaged second not seen yet, oldest first, each as its
    own Fix at the ticks it was published (its fix time and freshness). st
    stays the latest second all the while."""
    s = st["seq"]
    if s == log["seq"]:
        return
    for fx in _rcv.drain(log["seq"]):
        # seen as each one is sampled: a second published meanwhile (a
        # read_uart() under _sample, e.g. the WiFi scan of _home_veto) comes
        # after it, and marking st["seq"] seen at the end lost that second
        log["seq"] = fx[_rcv.F_SEQ]
        _sample(fx[_rcv.F_POS_AT], fx)
    if log["seq"] < s:                   # (all up to s were in the queue)
        log["seq"] = s

def _sample(now, fx=None):
    """One second: the Fix fx (a published second, see _drain_q), or the
    latest one (st) for None. The heartbeat clock, then the stay logic and the
    track when the fix is usable. Everything under it reads that second's
    fields from fx (_fv): st itself always holds the latest."""
    secs = _fix_s(fx)
    if secs is None or not 0 <= secs < FIX_S_END:
        return                           # (no date: a second with GGA only,
                                         # e.g. the first after a warm boot;
                                         # or see FIX_S_END)
    log["clock"] = (secs, _stamp())      # the GPS Anchor: heartbeat, page, RTC
    if not _usable(now, fx):
        return
    lt = log["last_t"]
    if lt is not None:                   # (in a stay too: see _time_back)
        t = _fv(fx, "date") + "T" + _fv(fx, "time") + "Z"
        if t < lt:
            _time_back(t, lt, fx)
        else:
            log["back"] = 0
    pos = (_fv(fx, "lat"), _fv(fx, "lon"))
    if log["still"]["active"]:
        _still_active(now, pos, fx)
        return
    s = _sample_line(now, log["prev"], log["last_t"], fx)
    if s is not None:
        _record(s, pos, fx)
    held = _still_hold()
    hit = _still_window(now, pos, fx)
    if hit:
        since = hit[1] if held is None or hit[1] < held else held
        _still_enter(now, hit[0], since, hit[2], hit[3], fx=fx)

# --- Upload ---------------------------------------------------------------
# Finished track files (never the one being recorded) are sent to Dawarich's
# Overland endpoint, oldest first, UPLOAD_BATCH points per POST. Progress is
# a <file>.pos sidecar (byte offset). LittleFS lfs_rename replaces the
# destination atomically; FAT vfs_fat does remove+rename (not atomic: a crash
# leaves no .pos, so the next boot resends from 0; Dawarich deduplicates via
# index_points_on_user_id_timestamp_lonlat / archival_safe_upsert_all).
#
# Offset advances only after a 2xx or a durable write to <file>.bad.
# If .bad is full, uploads of that file pause (state "bad full"). Three
# consecutive payload errors without a 2xx are treated as a server/config
# problem: backoff, no further quarantine. .bad retries that still 4xx
# never delete the quarantined data. The same batch answered SRV_ERR_MAX
# times in a row with another code (5xx, 404, 403) holds its track while
# others wait; any track's 2xx lifts the first such hold. Only a 2xx with a
# JSON Content-Type and a head read whole commits.
#
# TLS (mbedTLS) needs ~40 kB contiguous from the ESP-IDF heap, outside the
# Python heap. If the Python heap ever grows (see gc.threshold at the top) it
# takes that block, so allocations here stay small: the request body goes
# into a buffer allocated once at import. For the same reason the module must
# be deployed precompiled (gps_logger.mpy): compiling the source on the
# device grows the heap for good.

_body = bytearray(BODY_CAP)

up = {}

def _rm_done(path):
    """Delete an uploaded file. A failure raises (the caller's _fs_err):
    swallowed, the .pos went next and the whole file was sent again and
    again. Already gone is fine. After READ_ERR_MAX failures in a row it is
    parked as .err instead (its last batch went up each time)."""
    try:
        os.remove(path)
    except OSError as e:
        if _enoent(e):
            return
        errs = up["rm_err"]
        errs[path] = errs.get(path, 0) + 1
        if errs[path] < READ_ERR_MAX:
            raise
        errs.pop(path, None)
        try:
            os.rename(path, path + ".err")
        except OSError:
            raise e
        up["err_files"] += 1
        print("upload: cannot delete, parked", path + ".err")
        _diag_write("upload: %s cannot be deleted, parked as .err" % path)
        return
    up["rm_err"].pop(path, None)

def _size_or_gone(path):
    """The file's size, None when it is not there (ENOENT); any other stat
    error raises: taken for "not there", a transient EIO made the next
    write a "wb" over the whole file (a day's history, a pending track)."""
    try:
        return os.stat(path)[6]
    except OSError as e:
        if _enoent(e):
            return None
        raise

def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass

def _rm_gone(path):
    """Remove path; ENOENT (not there) is fine, any other OSError raises:
    for a step that must not be taken as done when the file stayed."""
    try:
        os.remove(path)
    except OSError as e:
        if not _enoent(e):
            raise

def _enoent(e):
    n = getattr(e, "errno", None)
    if n is None:
        try:
            n = e.args[0]
        except (AttributeError, IndexError):
            return False
    return n == 2

def _file_size(path):
    try:
        return os.stat(path)[6]
    except OSError:
        return None

def _seek_footer(f, old, ours=False):
    """Seek to the last known FOOTER, or after the last newline if truncated
    (-> False: a guess).

    A partial append leaves extra bytes after good_size; overwrite those
    from the last good footer. One that landed no more than the footer's
    bytes left the size as it was: with ours (old is the footer this
    session wrote) that is no damage either. An externally truncated file
    is shorter and does not end with FOOTER — resume after the last
    newline."""
    n = len(FOOTER)
    foot = FOOTER.encode()
    if old is not None and old >= n:
        pos = old - n
        f.seek(pos, 0)
        if f.read(n) == foot:
            f.seek(pos, 0)
            return True
        f.seek(0, 2)
        end = f.tell()
        if end > old or (ours and end == old):
            f.seek(pos, 0)
            return True
    else:
        f.seek(-n, 2)
        pos = f.tell()
        if f.read(n) == foot:
            f.seek(pos, 0)
            return True
    f.seek(0, 2)
    end = f.tell()
    win = end if end < LINE_MAX else LINE_MAX
    if win <= 0:
        f.seek(0, 0)
        return False
    start = end - win
    f.seek(start, 0)
    chunk = f.read(win)
    i = chunk.rfind(b"\n")
    if i < 0:
        f.seek(0, 0)
        return False
    f.seek(start + i + 1, 0)
    return False

def up_reset():
    # True once if the last boot was a low-idf reboot (LOWIDF_MARK)
    lowboot = _file_size(LOWIDF_MARK) is not None
    if lowboot:
        _rm(LOWIDF_MARK)
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, fail4=0, backoff=BACKOFF_START, allow_bad=True,
              bad_try={}, skip=None, hold={}, bad_full={}, read_err={},
              pos_err={}, enomem=0, last_err=None,
              guard_hits=_guard["hits"], lowidf_at=None, lowidf_seen=None,
              lowpy_at=None, lowpy_seen=None, ok_at=None,
              lowidf_boot=lowboot, next_at=time.ticks_ms(),
              rm_err={}, env_u=None, env_k=None, srv=None, err_ssid=None,
              srv_hold={}, err_files=0, pos_wr={}, hd=set())
    # the SSLContext now, in the boot's heap before the WiFi starts (made
    # at the first POST it could grow the heap and pin that area)
    _tls_ctx[0] = None
    try:
        _tls_context()
    except Exception as e:               # the first POST tries again
        print("tls context", repr(e))
    try:
        names = os.listdir(LOG_DIR)
    except OSError:
        names = []
    have = set(names)
    for n in names:
        if n.endswith(".tmp"):
            _rm(LOG_DIR + "/" + n)
        elif n.endswith(".pos") and n[:-4] not in have:
            _rm(LOG_DIR + "/" + n)       # orphan .pos or .bad.pos
        elif n.endswith(HIST_DONE):
            if n[:-len(HIST_DONE)] in have:
                up["hd"].add(LOG_DIR + "/" + n[:-len(HIST_DONE)])
            else:
                _rm(LOG_DIR + "/" + n)   # (its track gone)
        elif n.endswith(".err"):
            up["err_files"] += 1         # (parked: the page counts them)
    try:
        up["left"] = count_points()
    except MemoryError:
        gc.collect()
        up["left"] = 0
        print("count MemoryError")

def read_pos(path):
    """(byte offset to resume from, the offset of the keyframe its .trb
    delta records count from, else None). Corrupt / past-EOF sidecars reset
    to 0 (Dawarich dedup handles the resend). Never treat a bad .pos as
    EOF. A .trb offset without a keyframe "=" to start from is one too;
    key -1: none was read before off (_kb_seek then reads the file from
    its start to off, which must be a record start). A .pos that is there
    but cannot be read raises OSError: resending from 0 sent the file (and
    a .trb's history) twice. Read bounded: a .pos is under 24 bytes, a
    corrupt big one raised MemoryError. Read as bytes: one that is not
    UTF-8 (a damaged block) raised UnicodeError read as text, out of
    up_reset at every boot."""
    size = _size_or_gone(path)           # (a stat EIO raises: read as 0
    if size is None:                     # it resent the whole file)
        return 0, None
    try:
        with open(path + ".pos", "rb") as f:
            s = f.read(POS_READ)
    except OSError as e:
        if _enoent(e):
            return 0, None
        raise
    if len(s) >= POS_READ:
        print("pos corrupt", path, len(s))
        return 0, None
    try:
        v = s.split()
        off = int(v[0]) if v else 0
        key = int(v[1]) if len(v) > 1 else None
    except ValueError:                   # (UnicodeError too)
        print("pos corrupt", path)
        return 0, None
    if off < 0 or off > size:
        print("pos corrupt", path, off, size)
        return 0, None
    if off == 0:
        return 0, None
    tb = path.endswith(TRB)
    try:
        with open(path, "rb") as f:
            if tb:                       # (-1: no keyframe read yet)
                ok = key is not None and -1 <= key <= off
                if ok and key >= 0:
                    f.seek(key)
                    ok = f.read(1) == b"="
            else:
                f.seek(off - 1)
                ok = f.read(1) == b"\n"
        if not ok:
            print("pos misaligned", path, off)
            return 0, None
    except OSError as e:                 # (gone: nothing to resend; an EIO
        if _enoent(e):                   # raises, the caller's hold)
            return 0, None
        raise
    return off, key if tb else None

def write_offset(path, off, key=None):
    """Sidecar update: write <file>.pos.tmp, then rename over .pos.
    Atomic on LittleFS; on FAT a crash may drop the .pos (resend from 0).
    key: a .trb's keyframe offset (read_pos)."""
    tmp = path + ".pos.tmp"
    with open(tmp, "w") as f:
        if key is None:
            f.write("%d\n" % off)
        else:
            f.write("%d %d\n" % (off, key))
    os.rename(tmp, path + ".pos")

# Track files are read line by line into one buffer (readinto), never a
# new bytes object per line: f.readline(LINE_MAX) allocated LINE_MAX bytes
# for each line, and the upload reads at the worst moment for the heap.
# A .trb delta record is decoded to its text line in _rbuf[_SCR:], past
# the bytes read from the file.
_SCR = LINE_MAX
_rbuf = bytearray(LINE_MAX + 96)
_rmv = memoryview(_rbuf)
_rlm = _rmv[:LINE_MAX]
_rd = [0, 0, 0, 0, False]                # file offset of _rbuf[0], line start,
                                         # line end, bytes in _rbuf, EOF read
_rk = [None, 0]                          # read_batch: the keyframe at its
                                         # end, the offset it read from

def _rd_open(off):
    _rd[0] = off
    _rd[1] = _rd[2] = _rd[3] = 0
    _rd[4] = False

def _rd_fill(f):
    """Fill _rbuf from f's position: the bytes read (fewer only at EOF)."""
    n = 0
    while n < LINE_MAX:
        k = f.readinto(_rlm[n:] if n else _rlm)
        if not k:
            break
        n += k
    _rd[4] = n < LINE_MAX
    return n

def _rd_line(f):
    """Next line of f (opened with _rd_open) in _rbuf[_rd[1]:_rd[2]], its
    newline included (none on a cut-off last line); it starts at file
    offset _rd[0] + _rd[1]. False at EOF. A line over LINE_MAX bytes is
    drained to its newline and comes back empty (_rd[1] == _rd[2])."""
    a, n = _rd[2], _rd[3]
    i = _rbuf.find(b"\n", a, n)
    if i < 0:
        _rd[0] += a                      # refill from this line's start
        f.seek(_rd[0])
        n = _rd_fill(f)
        a = 0
        i = _rbuf.find(b"\n", 0, n)
        if i < 0 and n == LINE_MAX:
            while True:                  # over-long: drain it, skip it
                _rd[0] += n
                n = _rd_fill(f)
                i = _rbuf.find(b"\n", 0, n)
                if i >= 0 or not n:
                    break
            _rd[3] = n
            _rd[1] = _rd[2] = i + 1 if i >= 0 else 0
            return True
        _rd[3] = n
        if i < 0:
            _rd[1] = 0
            _rd[2] = n
            return n > 0                 # a cut-off last line, or EOF
    _rd[1] = a
    _rd[2] = i + 1
    return True

def _rd_rec(f):
    """Next record of a .trb (opened with _rd_open) in _rbuf[_rd[1]:_rd[2]]:
    a delta record (its first byte 0x80 or more; one cut off at the end of
    the file comes back short, _kb_dec drops it) or a line as _rd_line
    returns it. False at EOF. A flag byte whose varints run past 31 bytes
    (an erased block reads 0xFF) is a damaged record of 1 byte: it ended
    where _rbuf did, so where it ended depended on where the read started
    (a .pos committed in it was no record start read from its keyframe:
    resent from 0, for ever). trk2gpx's trb_lines takes the same end."""
    a, n = _rd[2], _rd[3]
    if n - a < 32 and not _rd[4]:        # (a record is at most 31 bytes)
        _rd[0] += a
        f.seek(_rd[0])
        n = _rd_fill(f)
        _rd[1] = _rd[2] = a = 0
        _rd[3] = n
    if a >= n:
        return False
    m = _rbuf[a]
    if m < 128:
        return _rd_line(f)
    j = a + 1
    i = 0
    e = a + 32 if a + 32 < n else n      # (past 31 bytes it is damaged
    while i < 6 and j < n:               # whatever follows: a run of 0xFF
        if m & (1 << i):                 # was scanned to _rbuf's end for
            while j < e and _rbuf[j] >= 128:    # each of its bytes)
                j += 1
            j += 1
        i += 1
    if j - a > 31:                       # damaged: not where a record ends
        j = a + 1
    _rd[1] = a
    _rd[2] = j if j < n else n
    return True

# The decoded line is written digit by digit (small ints only): a bytes %
# per field, a tuple per record and the date tuple per point made ~2.7
# times the garbage of a .trk read, right before the TLS POST.
_kday = [-1, bytearray(8)]               # the day (since 2020) and its YYYYMMDD
_kr = [0, 0]                             # the decoded point line (_kb_rec)

def _kb_put(p, v, d):
    """The number v (in its last decimal, d decimals) as text at _rbuf[p:]
    -> the end."""
    if v < 0:
        _rbuf[p] = 45                    # -
        p += 1
        v = -v
    n = 1
    t = v
    while t >= 10:
        t //= 10
        n += 1
    if n <= d:
        n = d + 1                        # (0.05)
    e = p + n + (1 if d else 0)
    q = e - 1
    k = 0
    while k < n:
        if d and k == d:
            _rbuf[q] = 46                # .
            q -= 1
        _rbuf[q] = 48 + v % 10
        v //= 10
        q -= 1
        k += 1
    return e

def _kb_2(p, v):
    _rbuf[p] = 48 + v // 10
    _rbuf[p + 1] = 48 + v % 10

def _kb_dec(a, b, s):
    """Apply the delta record _rbuf[a:b] to the state s and write its text
    line to _rbuf[_SCR:] -> its end, or 0: s not valid, or the record
    damaged (s is then not valid either, up to the next keyframe)."""
    if s[0] is None:
        return 0
    m = _rbuf[a]
    nf = s[11]
    if m & 127 & ~((2 << nf) - 1):       # a field this line does not have
        s[0] = None
        return 0
    j = a + 1
    dt = 1
    i = 0
    while i <= nf:
        if m & (1 << i):
            v = sh = 0
            while True:
                if j >= b or sh > 28:
                    s[0] = None
                    return 0
                c = _rbuf[j]
                j += 1
                v |= (c & 127) << sh
                sh += 7
                if c < 128:
                    break
            if v > 2 * _KB_MAX:          # (more than the encoder writes)
                s[0] = None
                return 0
            if not i:
                dt = v
            elif s[i] is None:
                s[0] = None
                return 0
            else:
                s[i] += (v >> 1) ^ -(v & 1)
                if not -_P10[9] < s[i] < _P10[9]:   # (9 digits: the line
                    s[0] = None                     # fits _rbuf[_SCR:])
                    return 0
        i += 1
    if j != b or dt < 1 or s[0] + dt >= FIX_S_END:
        s[0] = None
        return 0
    s[0] += dt
    p = _SCR
    i = 1
    while i <= nf:
        if s[i] is not None:
            p = _kb_put(p, s[i], s[i + 5])
        _rbuf[p] = 44                    # ,
        p += 1
        i += 1
    day = s[0] // 86400
    if day != _kday[0]:                  # (the date only when it changes)
        y, mo, d = _civil(day + EPOCH_DAYS)
        b8 = _kday[1]
        b8[0] = 48 + y // 1000
        b8[1] = 48 + y // 100 % 10
        b8[2] = 48 + y // 10 % 10
        b8[3] = 48 + y % 10
        b8[4] = 48 + mo // 10
        b8[5] = 48 + mo % 10
        b8[6] = 48 + d // 10
        b8[7] = 48 + d % 10
        _kday[0] = day
    _rbuf[p:p + 8] = _kday[1]
    r = s[0] % 86400
    _rbuf[p + 8] = 84                    # T
    _kb_2(p + 9, r // 3600)
    _kb_2(p + 11, r // 60 % 60)
    _kb_2(p + 13, r % 60)
    _rbuf[p + 15] = 90                   # Z
    _rbuf[p + 16] = 10
    return p + 17

def _kb_rec(a, b, s):
    """A .trb record _rbuf[a:b] -> 1: a point, its text line at
    _rbuf[_kr[0]:_kr[1]]; 0: a point that does not decode; -1: not a point
    (a header, "-", a cut-off keyframe). A keyframe sets s, any other line
    but "-" and "#..." makes it not valid."""
    c = _rbuf[a]
    if c >= 128:
        e = _kb_dec(a, b, s)
        if not e:
            return 0
        _kr[0] = _SCR
        _kr[1] = e
        return 1
    if c == 61 and _rbuf[b - 1] == 10:   # =
        _kb_parse(_rbuf, a + 1, b - 1, s)
        _kr[0] = a + 1
        _kr[1] = b
        return 1
    if not (c == 35 or (c == 45 and b - a == 2 and _rbuf[a + 1] == 10)):
        s[0] = None                      # (not "-" or "#": a damaged record,
    return -1                            # the next ones may be read from
                                         # inside one: none until a keyframe)

def _kb_seek(f, off, key, s):
    """Read a .trb from its keyframe at key up to off, the state s at off;
    key -1 (no keyframe before off): from the start of the file (an
    offset -1 was not checked: a corrupt "1234 -1" was read from inside a
    record). -> the offset of the keyframe s is from (with -1 the last one
    read, -1 if none), None when off is not where a record starts read so
    (s then not valid). Feeds the WDT every 64 records: from the start of a
    long day's file this takes as long as count_points' lines."""
    s[0] = None
    k = key
    n = 0
    _rd_open(key if key > 0 else 0)
    while _rd[0] + _rd[2] < off:
        if not _rd_rec(f):
            break
        n += 1
        if n & 63 == 0:
            feed()
            _rcv.read_uart()             # (a long scan: the UART, as the WDT)
        a, b = _rd[1], _rd[2]
        if a < b:
            if _rbuf[a] == 61:           # (as read_batch keeps _rk[0])
                k = _rd[0] + a
            _kb_rec(a, b, s)
        else:
            s[0] = None
    if _rd[0] + _rd[2] != off:
        s[0] = None
        return None
    return k

def _kb_at(f, off, key, s):
    """_kb_seek from the keyframe at key; when that does not lead to off (a
    corrupt .pos), from the start of the file: off a record start read so,
    the upload goes on from off, not from 0 (resent, the same .pos came
    again). None: off is no record start."""
    k = None
    if key is not None and key >= 0:
        k = _kb_seek(f, off, key, s)
    return _kb_seek(f, off, -1, s) if k is None else k

def _rd_point(a, b):
    """_rbuf[a:b] is a point line: "lat,lon,ele,hdop,time" (TRK1) or
    "lat,lon,ele,acc,speed,time" (TRK2). A line without its newline (power cut
    mid-write) is not a point."""
    return (_rbuf[b - 1] == 10 and _rbuf[a] != 35
            and 4 <= _rbuf.count(b",", a, b) <= 5)

def _rd_footer(a, b):
    return b - a == len(_FOOT) and _rbuf.find(_FOOT, a, b) == a

def count_points():
    """Points still to upload in all track files, the one being recorded
    included. Counts point lines from each file's sidecar offset. Runs at
    boot; flush() and commit_offset() keep up["left"] current in between.
    Feeds the WDT every 64 lines so a full flash cannot starve it."""
    try:
        names = [n for n in os.listdir(LOG_DIR)
                 if n.endswith(_TRACKS)]
    except OSError:
        return 0
    total = 0
    nlines = 0
    for n in names:
        path = LOG_DIR + "/" + n
        tb = path.endswith(TRB)
        try:
            try:
                off, key = read_pos(path)
            except OSError:
                # unreadable .pos: the track waits, and may go up from 0
                off, key = 0, None
            with open(path, "rb") as f:
                # a .trb offset that is no record start (its keyframe good)
                # counts from 0, as read_batch then reads the file;
                # _kb_at leaves the reader at a good one
                if tb and off and _kb_at(f, off, key, _ks) is None:
                    off = 0
                if not (tb and off):
                    _rd_open(off)
                while _rd_rec(f) if tb else _rd_line(f):
                    a, b = _rd[1], _rd[2]
                    if a == b:
                        continue
                    if _rd_footer(a, b):
                        break
                    nlines += 1
                    if nlines & 63 == 0:
                        feed()
                        _rcv.read_uart()             # (a long scan: the UART, as the WDT)
                    if tb:
                        c = _rbuf[a]
                        if c >= 128 or (c == 61 and _rbuf[b - 1] == 10):
                            total += 1
                    elif _rd_point(a, b):
                        total += 1
        except Exception as e:           # (it runs in up_reset at boot)
            print("count", path, repr(e))
            gc.collect()
    return total

def _ls(ends):
    """The paths with these endings, oldest first (YYYYMMDD_HHMMSS)."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR) if n.endswith(ends))
    except OSError:
        return []
    return [LOG_DIR + "/" + n for n in names]

def pending():
    """Finished track files. Sidecars (.pos) and quarantined (.bad) files
    are not listed."""
    return [p for p in _ls(_TRACKS) if p != log["file"]]

def pending_bad():
    """Rejected-batch files left for later recovery."""
    return _ls(_BADS)

def _put(pos, b):
    _body[pos:pos + len(b)] = b
    return pos + len(b)

_FOOT = FOOTER.encode()

# Overland JSON of one point, written into _body piece by piece
_J_START = b'{"locations":['
_J_PT = b'{"type":"Feature","geometry":{"type":"Point","coordinates":['
_J_TS = b']},"properties":{"timestamp":"'
_J_ALT = b'"altitude":'
_J_ACC = b'"horizontal_accuracy":'
_J_SPD = b'"speed":'
_J_FIX = len(_J_PT) + 1 + len(_J_TS) + 20 + 2   # "," time '",'
_ISO = (0, 1, 2, 3, -45, 4, 5, -45, 6, 7, 8, 9, 10, -58, 11, 12, -58, 13,
        14, 15)                          # YYYYMMDDTHHMMSSZ -> ISO (-c: chr c)
_pt = [0] * 16                           # the parsed .trk point (_trk_point)
_dev = [None, b""]                       # DEVICE_ID, its JSON tail

def _tail():
    if _dev[0] != DEVICE_ID:
        _dev[0] = DEVICE_ID
        _dev[1] = ('"device_id":"%s"}}' % DEVICE_ID).encode()
    return _dev[1]

def _fnum(a, b):
    """_rbuf[a:b] as a float, None if it is none. A field without a digit
    is none: MicroPython's float() reads ".", "+." and ".e1" as 0.0 (a
    point on the equator or at 0,0 went up; trk2gpx dropped it)."""
    i = a
    while i < b and not 48 <= _rbuf[i] <= 57:
        if _rbuf[i] in (69, 101):        # e E: ".e1" is no number either
            return None
        i += 1
    if i == b:
        return None
    try:
        return float(_rmv[a:b])
    except ValueError:
        return None

def _past(a, b, lim):
    """The text _rbuf[a:b] is past lim (90 or 180) although single
    precision reads it as lim: 90.000003 is 90.0 on the board, and so is
    89.999999, which is not past it. Its integer part decides, at lim
    itself a digit other than 0 after the point. An exponent is past it
    (900000001e-7 is 90.0 in f32 too; the logger writes none)."""
    if _rbuf.find(b"e", a, b) >= 0 or _rbuf.find(b"E", a, b) >= 0:
        return True
    i = a
    if i < b and _rbuf[i] == 45:         # -
        i += 1
    n = 0
    while i < b and 48 <= _rbuf[i] <= 57:
        n = n * 10 + _rbuf[i] - 48
        if n > lim:
            return True
        i += 1
    if n < lim or i >= b or _rbuf[i] != 46:
        return False
    i += 1
    while i < b and 48 <= _rbuf[i] <= 57:
        if _rbuf[i] != 48:
            return True
        i += 1
    return False

def _canon(a, b, dec, imax):
    """_rbuf[a:b] reads exactly as "%.<dec>f" prints it, with at most imax
    integer digits (so the float round trip is exact)."""
    if b - a < dec + 2:
        return False
    if _rbuf[a] == 45:                   # -
        a += 1
    d = b - dec - 1
    k = d - a
    if k < 1 or k > imax or _rbuf[d] != 46 or (k > 1 and _rbuf[a] == 48):
        return False
    for i in range(a, b):
        if i != d and not 48 <= _rbuf[i] <= 57:
            return False
    return True

def _ptnum(i, a, b, v, fmt, dec, imax):
    """A number field of _pt: the file's own bytes when they already read
    as fmt prints them, else fmt % v. -> its length."""
    _pt[i] = a
    _pt[i + 1] = b
    if _canon(a, b, dec, imax):
        _pt[i + 2] = None
        return b - a
    t = fmt % v
    _pt[i + 2] = t
    return len(t)

def _utf8(a, b):
    try:
        bytes(_rmv[a:b]).decode()
        return True
    except UnicodeError:
        return False

def _trk_field(i, a, b, fmt, imax):
    """An optional number field _rbuf[a:b] (one decimal) into _pt[i:i+3]
    -> its JSON length (0: empty or not finite, left out), or -1 when it
    is no number and no UTF-8 either (as the old line.decode() did: the
    point is not valid)."""
    _pt[i] = None
    if b <= a:
        return 0
    v = _fnum(a, b)
    if v is None:
        return 0 if _utf8(a, b) else -1
    if not math.isfinite(v):
        return 0
    return _ptnum(i, a, b, v, fmt, 1, imax)

def _trk_point(a, b):
    """Parse the .trk line _rbuf[a:b] in place into _pt -> -1 (not a point
    line), 0 (a point, not valid) or the length of its Overland JSON.
    A TRK1 line's hdop
    becomes horizontal_accuracy (* HDOP_M); a TRK2 line's acc is that
    already, and its speed (m/s) is "speed"."""
    rb = _rbuf
    e = b - 1
    if rb[e] != 10 or rb[a] == 35:
        return -1
    c1 = rb.find(b",", a, e)
    c2 = rb.find(b",", c1 + 1, e) if c1 >= 0 else -1
    c3 = rb.find(b",", c2 + 1, e) if c2 >= 0 else -1
    c4 = rb.find(b",", c3 + 1, e) if c3 >= 0 else -1
    if c4 < 0:
        return -1
    c5 = rb.find(b",", c4 + 1, e)        # TRK2: acc,speed
    if c5 >= 0 and rb.find(b",", c5 + 1, e) >= 0:
        return -1
    t = (c5 if c5 >= 0 else c4) + 1
    if e - t != 16 or rb[t + 8] != 84 or rb[t + 15] != 90:  # T Z
        return 0
    for k in _TDIG:
        if not 48 <= rb[t + k] <= 57:
            return 0
    if not _tc_ok(rb, t):                # (a .trb keyframe _kb_parse did
        return 0                         # not take is passed here too)
    lat, lon = _fnum(a, c1), _fnum(c1 + 1, c2)
    if (lat is None or lon is None or not math.isfinite(lat) or not math.isfinite(lon)
            or abs(lat) > 90 or abs(lon) > 180
            or (abs(lat) == 90 and _past(a, c1, 90))
            or (abs(lon) == 180 and _past(c1 + 1, c2, 180))):
        return 0
    _pt[6] = t
    need = (_J_FIX + len(_tail()) + _ptnum(0, c1 + 1, c2, lon, b"%.6f", 6, 3)
            + _ptnum(3, a, c1, lat, b"%.6f", 6, 3))
    k = _trk_field(7, c2 + 1, c3, b"%.1f", 6)        # ele
    if k < 0:
        return 0
    if k:
        need += len(_J_ALT) + 1 + k
    _pt[10] = _pt[13] = None
    if c5 >= 0:                          # TRK2: acc, speed
        k = _trk_field(10, c3 + 1, c4, b"%.1f", 6)
        if k < 0:
            return 0
        if k:
            need += len(_J_ACC) + 1 + k
        k = _trk_field(13, c4 + 1, c5, b"%.1f", 6)
        if k < 0:
            return 0
        if k:
            need += len(_J_SPD) + 1 + k
    elif c4 > c3 + 1:                    # TRK1: hdop
        v = _fnum(c3 + 1, c4)
        if v is None:
            if not _utf8(c3 + 1, c4):
                return 0
        elif math.isfinite(v * HDOP_M):        # (1e308: finite, * HDOP_M inf:
            _pt[10] = _pt[11] = 0        # invalid JSON, the batch a 400)
            _pt[12] = b"%.1f" % (v * HDOP_M)
            need += len(_J_ACC) + 1 + len(_pt[12])
    return need

def _span(pos, i):
    t = _pt[i + 2]
    if t is not None:
        return _put(pos, t)
    for k in range(_pt[i], _pt[i + 1]):  # byte by byte: no slice object
        _body[pos] = _rbuf[k]
        pos += 1
    return pos

def _trk_put(pos):
    """Write the point in _pt (from _trk_point) to _body at pos."""
    pos = _span(_put(pos, _J_PT), 0)     # lon
    _body[pos] = 44                      # ,
    pos = _put(_span(pos + 1, 3), _J_TS)  # lat
    t = _pt[6]
    for k in _ISO:
        _body[pos] = _rbuf[t + k] if k >= 0 else -k
        pos += 1
    pos = _put(pos, b'",')
    if _pt[7] is not None:
        pos = _span(_put(pos, _J_ALT), 7)
        _body[pos] = 44
        pos += 1
    if _pt[10] is not None:
        pos = _span(_put(pos, _J_ACC), 10)
        _body[pos] = 44
        pos += 1
    if _pt[13] is not None:
        pos = _span(_put(pos, _J_SPD), 13)
        _body[pos] = 44
        pos += 1
    return _put(pos, _tail())

UPLOAD_SCAN = 4096                       # bytes a batch with no valid point
                                         # yet reads on past UPLOAD_BATCH

def read_batch(path, off=0, key=None):
    """Fill _body with up to UPLOAD_BATCH valid points from byte offset off
    as Overland JSON -> (n, n_ok, body_len, new_off, eof).
    Truncated / non-numeric / inf / over-long points are skipped. While
    no point is valid it reads on past UPLOAD_BATCH, up to UPLOAD_SCAN
    bytes from off: an erased block (0xFF, a damaged 1-byte record each)
    went 50 bytes an upload round, ~8 min and ~80 .pos writes for 4 KB.
    A valid point that does not fit is left unread (eof=False). A .trk is
    parsed in _rbuf and written straight into _body: a few small objects
    per point (the float checks), not ~25 and a line buffer. A .trb is
    read from its keyframe at key (read_pos) and each delta decoded to its
    text line first; the keyframe at new_off goes to _rk[0]."""
    tb = path.endswith(TRB)
    s = _ks
    n, n_ok, pos, sep = 0, 0, _put(0, _J_START), 0
    nlines, eof = 0, False
    _rk[0] = None
    _rk[1] = off
    with open(path, "rb") as f:
        if tb:
            k = _kb_at(f, off, key, s) if off else None
            if k is None and off:
                print("pos misaligned", path, off)
                off = _rk[1] = 0         # (resent: Dawarich deduplicates)
            if not off:
                _rd_open(0)
                s[0] = None
            _rk[0] = k if off else -1
        else:
            _rd_open(off)
        while True:
            if not (_rd_rec(f) if tb else _rd_line(f)):
                eof = True
                break
            a, b = _rd[1], _rd[2]
            nlines += 1
            if nlines & 63 == 0:
                feed()
                _rcv.read_uart()             # (a long scan: the UART, as the WDT)
            if a == b:
                s[0] = None              # (an over-long line in a .trb)
                continue
            if _rd_footer(a, b):
                # the footer ends a .trk: bytes after it are left over from
                # a longer flush that failed (a later, shorter one wrote its
                # footer before them), lines the logger dropped
                eof = True
                break
            elif tb:
                k0 = _rk[0]
                if _rbuf[a] == 61:       # = a keyframe
                    _rk[0] = _rd[0] + a
                r = _kb_rec(a, b, s)
                if r < 0:
                    continue
                need = _trk_point(_kr[0], _kr[1]) if r else 0
                if need < 0:
                    need = 0
            else:
                need = _trk_point(a, b)
                if need < 0:
                    continue
            n += 1
            if need and pos + sep + need + 2 > len(_body):
                if n_ok == 0:
                    print("point too large", need)
                    need = 0
                else:
                    _rd[2] = a           # leave it unread
                    if tb:
                        _rk[0] = k0
                    n -= 1
                    eof = False
                    break
            if need:
                if sep:
                    _body[pos] = 44      # ,
                    pos += 1
                pos = _trk_put(pos)
                sep = 1
                n_ok += 1
            if n >= UPLOAD_BATCH and (n_ok
                                      or _rd[0] + _rd[2] - off >= UPLOAD_SCAN):
                break
        new_off = _rd[0] + _rd[2]
    if not sep:
        return n, 0, 0, new_off, eof
    return n, n_ok, _put(pos, b"]}"), new_off, eof

def commit_offset(path, n, new_off, eof=False, key=None):
    """Record that n points are done. Delete only at true EOF.
    Crash between POST and this call resends (at-least-once). At EOF the
    file goes without a .pos write first: on a full flash that write
    failed, so the file was never deleted and no room came back. key: a
    .trb's keyframe at new_off (_rk after read_batch)."""
    if eof:
        _rm_done(path)
        _rm(path + ".pos")
        _clear_bad_full(path)
    elif key is None:
        write_offset(path, new_off)
    else:
        write_offset(path, new_off, key)
    # a .bad's points left "left" when they moved there (count_points does
    # not count a .bad): not again when they go up
    if not path.endswith(".bad"):
        up["left"] = max(up["left"] - n, 0)

def _track_lines(src, path, off, end, key, fn):
    """fn(a, b) for each point line of the track file src (path) from off
    to end, in _rbuf[a:b]: a .trb's decoded from its keyframe at key."""
    if path.endswith(TRB):
        s = _ks
        if off and _kb_at(src, off, key, s) is None:
            return                       # (read_batch read it from 0)
        if not off:
            _rd_open(0)
            s[0] = None
        while _rd[0] + _rd[2] < end and _rd_rec(src):
            a, b = _rd[1], _rd[2]
            if a == b:
                s[0] = None
            elif not _rd_footer(a, b):
                if _kb_rec(a, b, s) > 0:
                    fn(_kr[0], _kr[1])
        return
    _rd_open(off)
    while _rd[0] + _rd[2] < end and _rd_line(src):
        a, b = _rd[1], _rd[2]
        if a < b and _rd_point(a, b):
            fn(a, b)

_hd8 = bytearray(8)                      # the day of the open history file

def _hist_batch(path, off, end, key):
    """A .trb batch about to be committed (from off to end) into the day
    history: a .trb's points go there once they are up, not while they
    wait (they were on the flash twice, the history and the track). Each
    decoded line goes straight to its day's file as its record again (a
    list of the lines was ~100 objects right after the POST). Before the
    commit: at the end of the file the commit deletes it; one that then
    fails sends the batch again, its points in the history twice (a line
    drawn over itself). Not for other files: a .trk's went when written, a
    .bad's when they left their track. Best effort: a failed write costs only
    the drawing, and it stops while the flash is nearly full."""
    if not HIST_DAYS or not path.endswith(TRB):
        return
    hf = [None, False, False, None]      # the open day file (_hist_begin)
    ok = False
    try:                                 # (the room and the trim too: a
        room = free()                    # MemoryError there, right after a
        if room is not None and room < HIST_RESERVE:   # 2xx, left the batch
            room = _hist_trim(HIST_RESERVE)            # uncommitted: sent and
        if room is not None and room < 2 * MIN_FREE:   # drawn again)
            return
        with open(path, "rb") as src:
            _track_lines(src, path, off, end, key,
                         lambda a, b: _hist_put(hf, a, b))
        ok = True
    except Exception as e:               # (best effort: the commit after
        print("hist", repr(e))           # it is what matters)
        _herr("write", e)
    finally:
        try:
            _hist_close(hf, ok)
        except Exception as e:           # (its day trim: a MemoryError)
            _herr("close", e)

_hist_skip = [None]                      # the last day too old to keep

def _hist_old(day, t):
    """No file of day yet, and HIST_DAYS days up to t newer than it: a
    track uploaded late (days offline) wrote it only for _hist_close to
    remove it again, for every batch."""
    try:
        ns = _hist_names()
    except OSError:
        return False
    ds = sorted(set(n[:8] for n in ns if t is None or n[:8] <= t))
    if day in ds:
        return False
    return len(ds) >= HIST_DAYS and day < ds[-HIST_DAYS]

def _hist_put(hf, a, b):
    """The line _rbuf[a:b] into the day file of its time (opened when the
    day changes)."""
    d = b - 17                           # YYYYMMDD of its time
    if hf[0] is not None:
        k = 0
        while k < 8 and _rbuf[d + k] == _hd8[k]:
            k += 1
        if k == 8:
            _hist_line(hf, _rbuf, a, b)
            return
        _hist_close(hf)
    for k in range(8):
        c = _rbuf[d + k]
        if not 48 <= c <= 57:
            return                       # (no date: not in a day's file)
        _hd8[k] = c
    day = bytes(_hd8).decode()
    if day == _hist_skip[0]:
        _hd8[0] = 0
        return
    t = _hist_today()
    if t is not None and day > t:        # (after the clock's day: a time that
        _hd8[0] = 0                      # leapt; no file for it, and no
        return                           # match for the next line either)
    if _hist_old(day, t):
        _hist_skip[0] = day              # (its other lines: no listdir each)
        _hd8[0] = 0
        return
    _hist_begin(hf, day)
    _hist_line(hf, _rbuf, a, b)

def _append_bad(path, off, new_off, key=None):
    """Copy consumed point lines to <path>.bad. None once durable, else
    why not: "full" (MAX_BAD_BYTES, the documented BAD FULL), "flash" (the
    MIN_FREE reserve) or "err" (I/O, memory: not a full .bad either).
    A .trb's are decoded: the .bad has the text lines."""
    if path.endswith(".bad"):
        return "err"
    bad = path + ".bad"
    sz = _file_size(bad) or 0
    need = max(new_off - off, 0)
    if path.endswith(TRB):
        need *= 8                        # (its text is ~6-8 times as long)
    if sz >= MAX_BAD_BYTES or sz + need > MAX_BAD_BYTES:
        print("bad file full", bad)
        return "full"
    room = free()
    # half the reserve is the quarantine's once the recording stopped for
    # want of space (log["full"]: a poison batch on a full flash held the
    # one track pending for good, its end never freeing the space); while
    # it records the whole reserve stays: under MIN_FREE flush drops the
    # new points, for a .bad of lines the track still holds
    floor = MIN_FREE // 2 if log["full"] else MIN_FREE
    if room is not None and room - need < floor:
        print("bad file: flash reserve", bad)
        return "flash"
    if not sz:
        _rm(bad + ".pos")                # (left from the last .bad of this
                                         # track: it skipped the new one's
                                         # lines, read on from its offset)
    cut = not _tail_nl(bad, sz)
    try:
        with open(path, "rb") as src, open(bad, "ab") as dst:
            if cut:
                dst.write(b"\n")
            _track_lines(src, path, off, new_off, key,
                         lambda a, b: dst.write(_rmv[a:b]))
    except (OSError, MemoryError) as e:
        print("quarantine", repr(e))
        gc.collect()
        return "err"
    return None

def _quarantine_batch(path, off, n, new_off, eof, key=None, key1=None):
    """Park one rejected batch only if the copy landed on flash. key, key1:
    a .trb's keyframes at off and new_off."""
    if path.endswith(".bad"):
        up["state"] = "err 422"
        return False
    why = _append_bad(path, off, new_off, key)
    if why == "full":
        up["state"] = "bad full"
        up["bad_full"][path + ".bad"] = True
        return False
    if why:                              # a flash or memory problem: the
        up["state"] = "fs err"           # .bad itself has room
        return False
    # (its points drawn once: a .pos write that failed brought the batch
    # back, and the next 422 added them to the history again; HIST_DONE as
    # on the 2xx path)
    # (in RAM too, pos_wr: a flash that took no .pos may take no note
    # either)
    pw = up["pos_wr"].get(path)
    hd = path in up["hd"]
    if not (pw and pw[0] == off) and not (hd and _hist_done(path) == off):
        _hist_batch(path, off, new_off, key)
    try:
        commit_offset(path, n, new_off, eof, key1)
    except (OSError, MemoryError) as e:
        # the copy landed but the offset did not: the next try copies the
        # batch again (duplicates in .bad, deduplicated by Dawarich)
        print("quarantine offset", repr(e))
        gc.collect()                     # (first: the note's open() after a
        if not (pw and pw[0] == off):    # MemoryError raised one again)
            up["pos_wr"][path] = [off, 0]
        _hist_done_note(path, off)
        _up_err("low mem" if isinstance(e, MemoryError) else "fs err")
        return False
    up["pos_wr"].pop(path, None)
    if hd:
        up["hd"].discard(path)
        _rm(path + HIST_DONE)
    up["allow_bad"] = False
    up["state"] = "quarantine"
    return True

def _retry_after_ms(v):
    """A Retry-After header's value (bytes or None) in ms, within the backoff's
    limits; None when there is no number."""
    if v is None:
        return None
    try:
        ms = int(v) * 1000
    except (TypeError, ValueError):
        return None
    return min(max(ms, BACKOFF_START), BACKOFF_MAX)

def _up_err(state):
    up["err"] += 1
    up["state"] = state

def _up_net_new(ssid):
    """WiFi got an IP on ssid. Network and server errors collected on
    another network say nothing about this one: the backoff starts over and
    the next batch is due now. (A LAN-only GEO_URL failed on the hotspot all
    the way, BACKOFF_MAX: home again, the first POST waited 30 min.) A 401
    and the per-file holds of payload errors are the server's verdicts and
    stay."""
    es = up.get("err_ssid")
    if es is None or es == ssid:
        return
    up["err_ssid"] = None
    if up["state"] == "err 401":
        return
    up["backoff"] = BACKOFF_START
    up["next_at"] = time.ticks_ms()
    up["srv"] = None

def _schedule(now, ms=None):
    if ms is None:
        b = up["backoff"]
        ms = b
        up["backoff"] = min(max(b * 2, BACKOFF_START), BACKOFF_MAX)
    up["next_at"] = time.ticks_add(now, ms)

SRV_ERR_MAX = 3                          # the same batch answered with a code
                                         # that is no payload error (5xx,
                                         # 404...) in a row: its file held

def _hold_file(path, now, ms, srv=False):
    """Backoff one file so a stuck oldest track does not block the rest.
    srv: a hold for server errors (SRV_ERR_MAX), which a 2xx of any track
    lifts (the server is back: after a 2-minute Dawarich restart the
    oldest track waited BACKOFF_MAX); any other hold (a payload, read or
    .pos error) is the file's own and stays."""
    up["hold"][path] = time.ticks_add(now, ms)
    if srv:
        up["srv_hold"][path] = True
    else:
        up["srv_hold"].pop(path, None)

def _bad_full_left():
    """True while a .bad that hit MAX_BAD_BYTES still exists on flash and
    its track still waits on it (keyed by the .bad: pulled, deleted or
    uploaded, it no longer holds the track up)."""
    bf = up["bad_full"]
    if not bf:
        return False
    keep = {}
    for b in bf:
        if _file_size(b) is not None and _file_size(b[:-4]) is not None:
            keep[b] = True
    up["bad_full"] = keep
    return bool(keep)

def _clear_bad_full(path):
    """path (a track, or its .bad) is done: no BAD FULL for it."""
    bf = up["bad_full"]
    bf.pop(path if path.endswith(".bad") else path + ".bad", None)

def _io_write(s, b):
    """All of b to a socket or TLS stream."""
    w = s.write
    mv = memoryview(b)
    i = 0
    t0 = time.ticks_ms()
    while i < len(mv):
        n = w(mv[i:])
        if not n:
            # would block: wait a little, with the dog fed and the UART
            # read, but not for ever (this spun without a feed before)
            if _age(time.ticks_ms(), t0) >= HTTP_TIMEOUT * 1000:
                raise OSError(110)       # ETIMEDOUT
            feed()
            _rcv.read_uart()
            time.sleep_ms(10)
            continue
        i += n
        t0 = time.ticks_ms()

_tls_ctx = [None, None]                  # the one SSLContext, and the size
                                         # of the CA_FILE it was made with
                                         # (None: there was none)
CA_MAX = 4096                            # bytes of CA_FILE at most (DER roots, one
                                         # ~1-2 KB a root; a PEM bundle
                                         # saved as it hit MemoryError at
                                         # every POST: TLS reboots)

def _ca_size():
    """Size of CA_FILE, None if there is none (ENOENT). Any other error
    raises: no POST then, never an unverified one."""
    try:
        return os.stat(CA_FILE)[6]
    except OSError as e:
        if _enoent(e):
            return None
        raise

def _der_certs(b):
    """The DER certificates one after the other in b (each a SEQUENCE:
    0x30, a long length 0x81 n or 0x82 n n), as bytes; white space after
    the last is dropped (a concat, an editor's newline). b that does not
    start as DER (PEM: modtls reads its chain itself): [b]. Anything else
    after a whole certificate raises ValueError ("ca err"): given whole, mbedtls
    parsed the first root alone and dropped the rest without a word (the
    away server's root gone: every POST there a TLS error that looked like
    the network)."""
    n = len(b)
    if not n or b[0] != 0x30:
        return [b]
    out = []
    i = 0
    while i < n:
        end = None
        if b[i] == 0x30 and i + 2 <= n:
            k = b[i + 1]
            if k == 0x81 and i + 3 <= n:
                end = i + 3 + b[i + 2]
            elif k == 0x82 and i + 4 <= n:
                end = i + 4 + (b[i + 2] << 8 | b[i + 3])
        if end is None or end > n:
            if not out:                  # (no DER at all: modtls judges it)
                return [b]
            if not bytes(b[i:]).strip():
                break
            raise ValueError("%s: %d B after certificate %d not DER"
                             % (CA_FILE, n - i, len(out)))
        out.append(b[i:end])
        i = end
    return out

def _tls_context():
    """The SSLContext every POST wraps its socket with: made (and CA_FILE
    read) once, by up_reset() at boot before the WiFi starts, and kept. It
    is the biggest single Python heap block of a POST; made at the first
    POST it came at the worst time (and could grow the heap into the IDF
    block TLS needs, and keep that area for the rest of the run). Not
    kept if making it failed, so the next POST tries again.
    Without CA_FILE (ENOENT only) the server is not verified; a CA_FILE
    that cannot be read, is empty or over CA_MAX raises: no POST, never a
    silent fall back to an unverified one."""
    ctx = _tls_ctx[0]
    if ctx is not None:
        return ctx
    import tls
    sz = _ca_size()
    if sz is not None and sz > CA_MAX:   # (not read: "ca err", no reboot)
        raise ValueError("%s over %d B" % (CA_FILE, CA_MAX))
    ca = None
    try:
        with open(CA_FILE, "rb") as f:
            ca = f.read()
    except OSError as e:
        if not _enoent(e):
            raise
    if ca is not None and not ca:
        raise ValueError("empty " + CA_FILE)
    ph = _post_phase[0]
    _post_phase[0] = "ctx"               # modtls reports an allocation that
    ctx = tls.SSLContext(tls.PROTOCOL_TLS_CLIENT)   # fails parsing the CA
    if ca:                               # as ValueError("invalid cert")
        ctx.verify_mode = tls.CERT_REQUIRED
        # (positional: modtls of 1.29 takes no keyword, cadata= raised
        # TypeError, every verified context failed, "ca err" for good;
        # measured on the board 2026-11-16). Several roots, one after the
        # other: each its own call (mbedtls parses one DER a call, and
        # appends): at home the server's name may be the LAN proxy's
        # (Let's Encrypt), away the CDN's (another root)
        for c in _der_certs(ca):
            ctx.load_verify_locations(c)
    else:
        ctx.verify_mode = tls.CERT_NONE
    _post_phase[0] = ph
    _tls_ctx[0], _tls_ctx[1] = ctx, (len(ca) if ca else None)
    return ctx

def _tls_ready(now):
    """Before a POST over https: the SSLContext, made again when CA_FILE
    changed size since (added, removed, replaced by one of another size;
    one of the same size takes a reboot). False (no POST, retried after
    LOWIDF_RETRY) when it cannot be made: state "ca err" and a diag line,
    or, out of memory, "low mem" counted in the run TLS_REBOOT_N ends."""
    _post_phase[0] = None
    try:
        if _tls_ctx[0] is not None and _ca_size() != _tls_ctx[1]:
            _tls_ctx[0] = None
            gc.collect()
            print("upload: %s changed, new TLS context" % CA_FILE)
        _tls_context()
        return True
    except Exception as e:               # (MemoryError too)
        oom = _enomem(e)
        up["enomem"] = up["enomem"] + 1 if oom else 0
        _up_err("low mem" if oom else "ca err")
        err = repr(e)[:60]
        print("upload ca", err)
        if err != up["last_err"]:
            up["last_err"] = err
            _diag_write("upload ca err %s" % err)
        gc.collect()
        # a bad CA file stays bad: the doubling backoff (and _upload_wait
        # lets the radio sleep in it); 30 s each kept a hotspot on for good
        _schedule(now, LOWIDF_RETRY if oom else None)
        if oom:
            _tls_reboot()
        return False
    finally:
        _post_phase[0] = None

UPLOAD_UNVERIFIED = "0"                  # .env: where a POST that is not
                                         # verified TLS (https with CA_FILE)
                                         # may go: "0" nowhere, "home" on a
                                         # home network, "1" everywhere
_UNV_SAY = {"0": "off", "home": "home only", "1": "on"}

def _tls_state():
    """How the upload reaches GEO_URL (status.json "tls"): "verified"
    (https, CA_FILE), "unverified" (https without it: CERT_NONE, an evil
    twin hotspot sees the Bearer key), "off" (http, in the clear), "ca err"
    (one that cannot be read: no POST at all), None without a GEO_URL."""
    u = up["url"]
    if not u:
        return None
    if not u.startswith("https"):
        return "off"
    try:
        sz = _ca_size()                  # (the file now, not the context:
    except OSError:                      # one made before CA_FILE went away
        return "ca err"                  # is made again unverified by
    if _tls_ctx[0] is not None and sz == _tls_ctx[1]:   # _tls_ready)
        return "unverified" if _tls_ctx[1] is None else "verified"
    return "unverified" if sz is None else "verified"

def _up_env(env):
    """UPLOAD_UNVERIFIED in .env: 0 (the default) posts only over verified
    TLS: at home too, the LAN is flat and any host on it can answer for
    the server (ARP, DNS), take the Bearer key and a 2xx removes tracks it
    never kept; "home" lets an unverified one (http, or https without
    CA_FILE) go on a home network, 1 everywhere. The old
    UPLOAD_AWAY_UNVERIFIED (1: everywhere, 0: home) is read when the new
    key is not there. And at the start a diag line when https goes without
    CA_FILE (nothing said so at run time)."""
    global UPLOAD_UNVERIFIED
    v = _plain(env.get("UPLOAD_UNVERIFIED"))
    if v:
        v = "home" if v.lower() == "home" else v
        t = v if v == "home" else _truthy(v)
        if t is None:
            _env_refused("UPLOAD_UNVERIFIED", "not 0/home/1", UPLOAD_UNVERIFIED)
        else:
            UPLOAD_UNVERIFIED = t if t == "home" else ("1" if t else "0")
    elif ("UPLOAD_UNVERIFIED" in _env_skipped
          or _env_partial[0] and env.get("UPLOAD_AWAY_UNVERIFIED")):
        # (its line not read, garbled or past an I/O error: it may have been
        # a 0 next to an old UPLOAD_AWAY_UNVERIFIED=1, unverified everywhere)
        UPLOAD_UNVERIFIED = "0"
        _say("env UPLOAD_UNVERIFIED not read (.env read part way, or its "
             "line garbled): 0, UPLOAD_AWAY_UNVERIFIED not used")
    else:
        v = _plain(env.get("UPLOAD_AWAY_UNVERIFIED"))
        t = _truthy(v) if v else None
        if t is not None:
            UPLOAD_UNVERIFIED = "1" if t else "home"
        elif v:
            _env_refused("UPLOAD_AWAY_UNVERIFIED", "not 1/0", UPLOAD_UNVERIFIED)
    ts = _tls_state()
    if ts == "unverified":
        print("WARNING: no %s - the server is not verified" % CA_FILE)
        _diag_write("upload GEO_URL https without %s: the server is not "
                    "verified, uploads %s"
                    % (CA_FILE, _UNV_SAY.get(UPLOAD_UNVERIFIED, "?")))
    elif ts == "off":                    # (http: it was not said at all, and
        print("WARNING: GEO_URL is http - in the clear")   # the default holds it)
        _diag_write("upload GEO_URL http: in the clear, uploads %s"
                    % _UNV_SAY.get(UPLOAD_UNVERIFIED, "?"))

def _away_unverified():
    """The upload would not be verified TLS and UPLOAD_UNVERIFIED does not
    let it go on this network: no POST."""
    m = UPLOAD_UNVERIFIED
    if m == "1" or (m == "home" and at_home()):
        return False
    # ("ca err": a CA_FILE there that cannot be read is no missing one;
    # _tls_ready says so and posts nothing: "put it on the board" misled)
    return _tls_state() not in ("verified", "ca err")

def _hold_unverified(now):
    """UPLOAD_UNVERIFIED and no verified TLS: not on this network. An
    error state, so the radio sleeps out the wait (_upload_wait); a home
    network is a new one (err_ssid, _up_net_new): the tracks are due there
    at once."""
    up["state"] = "err unverified"
    up["err_ssid"] = wifi["ssid"]
    _schedule(now, BACKOFF_MAX)
    if not _unv_said[0]:                 # (once a boot: the tracks wait for
        _unv_said[0] = True              # good, a board updated over the air
        _diag_write("upload err unverified: not verified TLS (tls %s, UPLOAD_UNVERIFIED"
                    "=%s): put %s on the board, or UPLOAD_UNVERIFIED=home|1 "
                    "in .env" % (_tls_state(), UPLOAD_UNVERIFIED, CA_FILE))

_unv_said = [False]                      # _hold_unverified's line this boot

_rtc_set = [False]                       # board clock set from the GPS once
_rtc_ntp = [None]                        # the NTP answer that set it (None: a fix)

def _rtc_sync(now):
    """mbedTLS checks the server certificate's dates against the board
    clock, which starts at 2000-01-01 each boot: with CA_FILE every
    verified POST failed. Set it from the GPS fix (valid, a date in
    2020-2087) before a POST; while it was not set yet and the fix is gone
    (a board that had one outdoors and is indoors now), from the last fix's
    clock run on by the uptime (log["clock"], as _local_hm does), else from
    NTP's answer (ntp_tick: no fix since the boot, or none to set it
    with): once a boot, again only when it drifted more than RTC_DRIFT s.
    Seconds since 2020 (small ints)."""
    src = ""
    if has_fix(now):
        s = _fix_s()
        if s is None:
            return
        s += _age(now, st["pos_at"]) // 1000
        _rtc_ntp[0] = None
    elif (log["clock"] or _ntp[0]) and (
            not _rtc_set[0]
            or _rtc_ntp[0] is not (None if log["clock"] else _ntp[0])):
        # no fix now: the clock a fix gave this boot, else NTP's answer,
        # until a fix sets it (an Anchor: its UTC runs on with the uptime);
        # a newer NTP answer corrects one an older answer set, a fix's
        # Anchor one any answer set (a wrong one stayed the board's clock
        # for the whole run: NTP is not asked once a fix told the time)
        s = _utc_s(log["clock"] or _ntp[0])
        src = " gps" if log["clock"] else " ntp"
        _rtc_ntp[0] = None if log["clock"] else _ntp[0]
    else:
        return
    d = s // 86400
    y, m, dd = _civil(d + EPOCH_DAYS)
    if not 2020 <= y <= 2087:
        return
    rtc = machine.RTC()
    try:
        if _rtc_set[0]:
            t = rtc.datetime()
            c = ((_days(t[0], t[1], t[2]) - EPOCH_DAYS) * 86400
                 + t[4] * 3600 + t[5] * 60 + t[6])
            if -RTC_DRIFT <= c - s <= RTC_DRIFT:
                return
        r = s % 86400
        # weekday 0 = Monday; 2020-01-01 was a Wednesday
        rtc.datetime((y, m, dd, (d + 2) % 7, r // 3600, r // 60 % 60,
                      r % 60, 0))
    except Exception as e:               # no RTC: the POST still goes
        print("rtc", repr(e))
        return
    if not _rtc_set[0]:
        _diag_write("clock set %s%s" % (_stamp_s(s)[0], src))
    _rtc_set[0] = True

def _tls_wrap(s, host):
    """TLS like micropython-lib requests (tls.SSLContext, CERT_NONE), or,
    with CA_FILE on flash (a DER root certificate), verifying the server."""
    return _tls_context().wrap_socket(s, server_hostname=host)

_guard = {"t": None, "until": None, "sock": None, "hits": 0}

def _guard_cb(timer):
    """Runs during blocking socket calls too (they handle pending events):
    feeds the WDT until the POST's deadline, then closes its socket so the
    call returns with an error instead of the WDT resetting the board."""
    until = _guard["until"]
    if until is None:
        return
    if time.ticks_diff(time.ticks_ms(), until) < 0:
        feed()
        # the NMEA too: a connect blocks up to HTTP_TIMEOUT, the UART holds
        # ~6 s (measured: full at 8192 B in a 6.7 s connect, this callback
        # ran 13 times in it); read_uart is not re-entered
        try:
            _rcv.read_uart()
        except Exception:
            pass
        return
    s = _guard["sock"]
    _guard["until"] = _guard["sock"] = None
    _guard["hits"] += 1
    if s is not None:
        try:
            s.close()
        except Exception:
            pass

def _guard_start(sock, ms):
    """Arm the guard for sock. Its timer is made once and kept running
    (an idle callback a second): deinit / init around each POST would
    alloc and free an interrupt handle in the IDF heap TLS needs whole."""
    _guard["sock"] = sock
    _guard["until"] = time.ticks_add(time.ticks_ms(), ms)
    if _guard["t"] is None:
        try:
            t = machine.Timer(1)
            t.init(period=1000, mode=machine.Timer.PERIODIC, callback=_guard_cb)
            _guard["t"] = t
        except Exception as e:
            print("post guard", repr(e))

def _guard_stop():
    _guard["until"] = _guard["sock"] = None

_post_phase = [None]                     # "connect", "tls", "http" in a POST,
                                         # "ctx" while the SSLContext is made

DNS_RETRY = 600                          # s a network whose resolver did not
                                         # answer is not asked again (_resolve)
DNS_SLOW = 2000                          # ms: a failed lookup this long is no
                                         # answer (a fast one: no such name)
DNS_KEEP = 12                            # addresses kept (host, port, kind, SSID):
                                         # 3 networks x GEO_URL, NTP, OTA is 9, and
                                         # the 9th cleared every network's
_dns = {}                                # key -> the last address on that net
_dns_bad = {}                            # SSID -> (Stamp of a slow failure, the IP)

def _link_up():
    """The station still has its IP (unknown: taken as up)."""
    w = wifi["wlan"]
    if w is None:
        return False
    try:
        return w.status() == network.STAT_GOT_IP
    except Exception:
        return True

def _resolve(host, port, kind=None):
    """getaddrinfo's first answer. A lookup that failed slowly (DNS_SLOW: a
    resolver that does not answer; measured 7 s, nothing else runs, the
    UART fills, no socket yet for the guard) is not asked again on that
    network for DNS_RETRY or until it gives a new IP, for no host (one
    silent resolver blocked once per host name: GEO_URL, NTP and OTA were
    21 s): the address that network resolved before is used, or it fails
    at once. A fast failure
    (no such name) is only that. Addresses are kept per network: another
    network's (a LAN address reached by the API key on a stranger's LAN of
    the same range) is never used."""
    if kind is None:
        kind = socket.SOCK_STREAM
    net = wifi["ssid"]
    key = (host, port, kind, net)
    bad = _dns_bad.get(net)
    if bad is not None and _recent_s(bad[0], DNS_RETRY):
        ai = _dns.get(key)
        if ai is None:
            raise OSError(-202)          # (lwIP's own "no answer")
        return ai
    feed()
    t0 = time.ticks_ms()
    try:
        ai = socket.getaddrinfo(host, port, 0, kind)[0]
    except OSError:
        feed()
        if time.ticks_diff(time.ticks_ms(), t0) >= DNS_SLOW and _link_up():
            # (a lookup the link dropped under is no silent resolver: with
            # the same IP back the mark held for DNS_RETRY)
            if len(_dns_bad) >= DNS_KEEP and net not in _dns_bad:
                _dns_bad.clear()
            _dns_bad[net] = (_stamp(), wifi["ip"])
            ai = _dns.get(key)
            if ai is not None:
                return ai
        raise
    feed()
    if len(_dns) >= DNS_KEEP and key not in _dns:
        _dns.clear()                     # (a few networks: no growth)
    _dns[key] = ai
    _dns_bad.pop(net, None)
    return ai

def _http_open(method, url):
    """The start of an HTTP/1.0 request, no redirects: a socket for url's
    host. -> (socket, its address, https?, host, the request's first lines).
    The Host header: host:port unless it is the scheme's default port (RFC
    7230 5.4; a proxy on :8443 routes by it)."""
    proto, _, rest = url.partition("://")
    host, _, path = rest.partition("/")
    dflt = port = 443 if proto == "https" else 80
    if ":" in host:
        host, _, p = host.rpartition(":")
        port = int(p)
    head = "%s /%s HTTP/1.0\r\nHost: %s\r\n" % (
        method, path, host if port == dflt else "%s:%d" % (host, port))
    ai = _resolve(host, port)
    return (socket.socket(ai[0], socket.SOCK_STREAM, ai[2]), ai[-1],
            proto == "https", host, head)

def _http_head(s, want, got):
    """Read an answer's headers, in bounded pieces (a long header comes in
    several, only a piece at a line start is a header or the end). The value
    of a header named in want (lowercase) goes to got at the same index.
    More than HTTP_LINES pieces before the blank line raise: silently
    stopped, the rest of the head read as the body (a GET's sha256 failed
    every check with no clue; a POST's caller catches: the head was not
    read). -> True at the blank line, False when the answer ended before
    it (the head not whole)."""
    bol = True
    for _ in range(HTTP_LINES):
        h = s.readline(HTTP_LINE)
        if not h:
            return False
        if bol and h in (b"\r\n", b"\n"):
            return True
        if bol:
            k, _, v = h.partition(b":")
            k = k.strip().lower()
            if k in want:
                i = want.index(k)
                if got[i] is None:       # the first one: a text/html one and
                    got[i] = v.strip()   # then a JSON one is no JSON answer
        bol = h[-1:] == b"\n"
    raise OSError("head over %d lines" % HTTP_LINES)

def _http_close(s):
    _guard_stop()
    try:
        s.close()
    except OSError:
        pass

def _raw_post(url, data, headers, timeout):
    """A minimal HTTP/1.0 POST, no redirects: micropython-lib requests
    followed one as a GET with the old Content-Length (the server then
    waits for a body) and the Bearer header, without the timeout. The body
    goes out in HTTP_CHUNK pieces and the wait for the answer polls, both
    reading the UART meanwhile, so a slow server costs no NMEA. -> (status
    code, Retry-After, Content-Type), the two headers bytes: Retry-After
    None when not sent; Content-Type b"" when the head was read to its
    blank line without one, None when the head was not read whole (a
    reset, a timeout, the guard, MemoryError, over HTTP_LINES)."""
    _post_phase[0] = "connect"
    s, addr, https, host, hdr = _http_open("POST", url)
    raw = s                              # (the TLS socket has no settimeout)
    _guard_start(s, HTTP_MAX)            # the raw socket: closing it fails
    try:                                 # a TLS call stuck on it
        s.settimeout(timeout)
        s.connect(addr)
        if https:
            _heap_note("connect")
            _post_phase[0] = "tls"       # the handshake
            s = _tls_wrap(s, host)
            _heap_note("tls_wrap")
        _post_phase[0] = "http"
        for k in headers:
            hdr += "%s: %s\r\n" % (k, headers[k])
        hdr += "Content-Length: %d\r\nConnection: close\r\n\r\n" % len(data)
        _io_write(s, hdr.encode())
        mv = memoryview(data)
        for i in range(0, len(mv), HTTP_CHUNK):
            _io_write(s, mv[i:i + HTTP_CHUNK])
            _rcv.read_uart()
            feed()
        poll = select.poll()
        poll.register(s, select.POLLIN)
        t0 = time.ticks_ms()
        while not poll.poll(200):
            _rcv.read_uart()
            feed()
            if _age(time.ticks_ms(), t0) >= HTTP_WAIT:
                raise OSError(110)       # ETIMEDOUT
        # the answer's first byte, not the first POLLIN: a TLS 1.3 server's
        # session tickets right after the handshake are that, and the
        # status line was then waited for in one blocking read, the UART
        # unread for up to HTTP_WAIT. Non-blocking, a read that finds only
        # tickets comes back None (EAGAIN: mbedTLS's WANT_READ, which can
        # be read on from; a timeout on a TLS read cannot)
        raw.setblocking(False)
        first = None
        while first is None:
            try:
                first = s.read(1)
            except OSError as e:
                if e.args[0] != 11:      # EAGAIN
                    raise
            if first is None:
                _rcv.read_uart()
                feed()
                if _age(time.ticks_ms(), t0) >= HTTP_WAIT:
                    raise OSError(110)
                poll.poll(200)
        # what is left of HTTP_WAIT for the rest of the status line
        raw.settimeout(max(HTTP_WAIT - _age(time.ticks_ms(), t0), 1000)
                       / 1000)
        line = first + s.readline(HTTP_LINE - 1) if first else first
        line = line.split(None, 2)
        if len(line) < 2:                # closed before a status line: "no
            raise OSError(104)           # answer", not an IndexError
        code = int(line[1])
        got = [None, None]
        whole = False
        try:
            # the status is in: a reset, a timeout or the guard closing
            # the socket in the headers keeps the code, but the head is
            # not whole (_http_post: a 2xx then is no commit, resent and
            # deduplicated by Dawarich, rather than a portal's bare 200
            # taken for one and the track deleted)
            whole = _http_head(s, (b"retry-after", b"content-type"), got)
        except Exception as e:
            print("upload head", code, repr(e))
        return code, got[0], (got[1] or b"") if whole else None
    finally:
        _http_close(s)

HTTP_POST = _raw_post                    # (tests swap it)

# mbedTLS *_ALLOC_FAILED codes: high-level (SSL, ECP, PK, X509, CIPHER, MD,
# DHM, PEM, PKCS7) and low-level (MPI, ASN1). An error code can be a
# high-level and a low-level one added together.
TLS_ALLOC_HI = (0x7F00, 0x4D80, 0x3F80, 0x2880, 0x6180, 0x5180, 0x3400,
                0x1180, 0x5780)
TLS_ALLOC_LO = (0x10, 0x6A)

def _enomem(e):
    """TLS out of memory: MemoryError, ENOMEM (modtls raises SSL_ALLOC_FAILED
    as it) or an mbedTLS ALLOC_FAILED code. modtls turns -1..-255 into a
    positive errno, so MPI (-0x10) arrives as 16, which is also EBUSY, and
    ASN1 (-0x6A) as 106: those count only from the TLS handshake. A
    failed allocation parsing CA_FILE comes as ValueError("invalid cert"):
    out of memory only while the SSLContext is made, and only with the
    IDF heap's largest block under TLS_MIN_KB (else the file is bad)."""
    if isinstance(e, MemoryError):
        return True
    if isinstance(e, ValueError):
        if _post_phase[0] != "ctx":
            return False
        big = idf_heap()[1]
        return big is None or big < TLS_MIN_KB
    a = getattr(e, "args", None) or (None,)
    n = a[0]
    if not isinstance(n, int):
        return False
    if n == 12:                          # ENOMEM
        return True
    if n in TLS_ALLOC_LO:
        return _post_phase[0] == "tls"
    if n < 0:
        n = -n
        return (n & 0xFF80) in TLS_ALLOC_HI or (n & 0x7F) in TLS_ALLOC_LO
    return False

def _http_post(size):
    """POST _body[:size]. Returns (status_code or None, retry_after_ms or None)."""
    feed()
    code, retry = None, None
    _post_phase[0] = None
    # only a hit during this POST is its abort: not the OTA download's (that
    # shares the guard), or one in the headers of the last, 2xx POST
    up["guard_hits"] = _guard["hits"]
    try:
        code, retry, ctype = HTTP_POST(
            "%s/api/v1/overland/batches" % up["url"],
            memoryview(_body)[:size],
            {"Content-Type": "application/json",
             "Authorization": "Bearer %s" % up["key"]},
            HTTP_TIMEOUT)
        retry = _retry_after_ms(retry)
        ct = ctype.lower() if ctype else ctype   # (b"" and None kept apart)
        if 200 <= code < 300 and (code == 204 or ct is None
                                  or b"json" not in ct):
            # only a JSON 2xx commits. One that is no JSON (HTML, plain
            # text, no Content-Type, a 204 without a body) is a captive
            # portal or a proxy answering, not Dawarich: without CA_FILE
            # nothing else tells, and the batch counted as sent was
            # deleted. A head not read whole (ctype None: a reset, a
            # timeout, the guard) cannot tell either: resent, Dawarich
            # deduplicates the points.
            if ct is None:
                print("upload: %d, head not read" % code)
                _diag_write("upload err head not read %d" % code)
            elif b"html" in ct:
                print("upload: HTML answer, not Dawarich (captive portal?)")
                _diag_write("upload err html %d (captive portal?)" % code)
            else:
                print("upload: %d not JSON, not Dawarich (proxy?)" % code)
                _diag_write("upload err not json %d (captive portal?)" % code)
            code = None
        up["enomem"] = 0
        up["last_err"] = None
    except MemoryError:
        up["enomem"] = up["enomem"] + 1
        raise
    except Exception as e:
        # "in a row": any other failure ends a run of out-of-memory ones
        up["enomem"] = up["enomem"] + 1 if _enomem(e) else 0
        print("upload", repr(e))
        if _guard["hits"] != up["guard_hits"]:
            up["guard_hits"] = _guard["hits"]
            print("upload: POST over %d s, aborted" % (HTTP_MAX // 1000))
            _diag_write("upload aborted after %d s" % (HTTP_MAX // 1000))
        err = repr(e)[:60]
        if err != up["last_err"]:    # one line per kind of error
            up["last_err"] = err
            _diag_write("upload err %s" % err)
        if _post_phase[0] == "tls":
            _ntp_doubt()
    finally:
        feed()
        gc.collect()
    if code is not None and 300 <= code < 400:
        print("GEO_URL redirects (%d): put the final https origin in .env"
              % code)
    return code, retry

def _mem_reboot_soon():
    """The last boot was a memory reboot (LOWIDF_MARK) under
    LOWIDF_REBOOT_GAP ago: no other one yet, for any of the reasons (a
    lasting cause rebooted every few minutes for ever)."""
    return up["lowidf_boot"] and uptime_s() * 1000 < LOWIDF_REBOOT_GAP

def _mem_reboot(why):
    """Write what is in RAM, tell the next boot (LOWIDF_MARK), restart."""
    try:
        flush(_still_hold())
    except Exception:
        pass
    _diag_final()
    _diag_write("reboot " + why)
    try:
        with open(LOWIDF_MARK, "w") as f:
            f.write("1")
    except OSError:
        pass
    machine.reset()

def _reboot_for_memory(why, say=None, home=True):
    """The memory reboot, for each of its reasons (TLS out of memory, a low
    IDF block or Python heap, a tick's lasting MemoryError), under one rule:
    never while an OTA image is pending (a reset aborts it and blames the
    build), at most one per LOWIDF_REBOOT_GAP whatever the reason
    (_mem_reboot_soon: a lasting cause rebooted every few minutes), and for
    the upload's reasons (home) only at home. say: the console line before
    it. -> True: rebooting (_mem_reboot), False: not now."""
    if (home and not at_home()) or _ota_wants_radio() or _mem_reboot_soon():
        return False
    if say:
        print(say)
    _mem_reboot(why)
    return True

def _tls_reboot():
    """TLS_REBOOT_N TLS allocations failed in a row at home: the IDF heap
    is fragmented for good. A memory reboot (_reboot_for_memory: one that
    does not help, a certificate chain over TLS_MIN_KB, rebooted every ~2.5
    min before the gap)."""
    if up["enomem"] < TLS_REBOOT_N:
        return False
    return _reboot_for_memory("tls memory %d" % up["enomem"],
                              "upload: TLS out of memory %d times, reboot" % up["enomem"])

def _low_ms(now, at, seen):
    """ms a low-heap episode (up[at]: its start, up[seen]: its last
    check) has lasted; a gap over LOWIDF_WAIT between checks starts a new
    one."""
    t0, last = up[at], up[seen]
    if t0 is None or last is None or _age(now, last) > LOWIDF_WAIT:
        t0 = up[at] = now
    up[seen] = now
    return _age(now, t0)

def _low_reboot(fmt, n, low_ms):
    """At home, the largest IDF heap block too small for TLS, or the free
    Python heap under TLS_MIN_PY_KB, for LOWIDF_WAIT with tracks waiting
    (gc.collect() did not bring it back): write what is in RAM and
    restart. Its own reason, not the ENOMEM run; at most one per
    LOWIDF_REBOOT_GAP for either (LOWIDF_MARK tells the next boot).
    fmt % n: what is low ("idf %d KB", "mem %d B")."""
    if low_ms < LOWIDF_WAIT:
        return False
    what = fmt % n
    return _reboot_for_memory("low %s %d s" % (what, low_ms // 1000),
                              "upload: low %s for %d s, reboot" % (what, low_ms // 1000))

def _env_bad(url, key):
    """Why GEO_URL / API_KEY cannot go into a request head, or None. Both
    printable ASCII without a space (a CR in the key split the header, a
    space or a "# comment" got a 400 / 401 for ever); GEO_URL http(s)://
    host[:port][/path], host letters, digits, "-", "." and "_" (a LAN or
    Docker name, dawarich_app: "no .env" for the whole boot; userinfo or
    IPv6 made _raw_post's int(port) raise: "err net" for ever)."""
    for v, name in ((key, "API_KEY"), (url, "GEO_URL")):
        for c in v:
            if not "!" <= c <= "~":
                return name + " in .env: a space, control or non-ASCII character"
    proto, _, rest = url.partition("://")
    host = rest.partition("/")[0]
    h, _, p = host.partition(":")
    ok = (proto in ("http", "https") and h
          and (":" not in host or (p.isdigit() and 0 < int(p) < 65536)))
    for c in h:
        if not (c.isalpha() or c.isdigit() or c in "-._"):
            ok = False
    return None if ok else "GEO_URL in .env: not http(s)://host[:port][/path]"

def _due(files, held, now):
    """files without those held back (_hold_file) for now. A hold that ran
    out goes (any, not only one of files): only a 2xx for its file removed
    it, so a deleted or parked track's hold stayed all boot, and past a
    ticks half-wrap it could read as held again."""
    for p in list(held):
        if not _waiting(now, held[p], UP_WAIT_MAX):
            del held[p]
    due = []
    for p in files:
        if p not in held:
            due.append(p)
    return due

def _low_stop(now, state, fmt, n, at, seen):
    """A heap margin for TLS is not met (fmt % n: what is low, up[at] and
    up[seen]: _low_ms): counted, one diag line, and the reboot it may be
    time for (-> False: none, the caller schedules the retry)."""
    _up_err(state)
    if up["last_err"] != state:
        up["last_err"] = state
        _diag_write("upload low " + fmt % n)
    return _low_reboot(fmt, n, _low_ms(now, at, seen))

def upload_tick():
    """Send one batch when WiFi is up and due. Runs on home and hotspot."""
    now = time.ticks_ms()
    if (wifi["state"] != "connected" or up["state"] == "no .env"
            or _waiting(now, up["next_at"], UP_WAIT_MAX)):
        return
    if not up["url"] or not up["key"]:
        up["state"] = "no .env"
        return
    if up["env_u"] is not up["url"] or up["env_k"] is not up["key"]:
        bad = _env_bad(up["url"], up["key"])  # (once: it allocates)
        if bad:
            # never sends (as no .env): the value is never printed
            print("upload:", bad)
            _diag_write("upload " + bad)
            up["state"] = "no .env"
            return
        up["env_u"], up["env_k"] = up["url"], up["key"]
    files = pending()
    up["files"] = len(files)
    skip = up["skip"]
    if skip and skip in files and len(files) > 1:
        # (no comprehension: it would make skip a cell, allocated every call,
        # also on the early returns above, ~40 times a second)
        due = []
        for p in files:
            if p != skip:
                due.append(p)
        files = due
        # one pass only: kept until a 2xx, a passing MemoryError starved that
        # track all boot while the other one kept failing (never held either:
        # the list here is 1 long). Still out of memory: it skips again
        up["skip"] = None
    held = up["hold"]
    files = _due(files, held, now)
    if not files and up["allow_bad"]:
        files = _due([p for p in pending_bad()
                      if up["bad_try"].get(p, 0) < MAX_BAD_RETRY],
                     held, now)
    if not files:
        if _bad_full_left():
            up["state"] = "bad full"
        elif up["files"] and held:
            up["state"] = "held"         # every waiting track held back
        else:
            up["state"] = "idle"
        _schedule(now, BACKOFF_START)
        return
    if _away_unverified():
        _hold_unverified(now)
        return
    path = files[0]
    try:
        off, key = read_pos(path)
    except OSError as e:
        # the .pos is there but unreadable: hold the track (the newer ones
        # go) and try again; only after READ_ERR_MAX tries from 0 after
        # all (resent, Dawarich deduplicates) rather than never
        pe = up["pos_err"]
        pe[path] = pe.get(path, 0) + 1
        print("upload pos read", path, repr(e))
        if pe[path] < READ_ERR_MAX:
            _hold_file(path, now, BACKOFF_MAX)
            _fs_err(now)
            return
        off, key = 0, None
    except MemoryError:
        up["skip"] = path
        _low_mem(now)
        return
    up["pos_err"].pop(path, None)
    try:
        _heap_mark()
        n, n_ok, size, new_off, eof = read_batch(path, off, key)
        key1 = _rk[0]
        if _rk[1] != off:
            off, key = _rk[1], None      # (read from 0: a bad .pos)
        _heap_note("read_batch")
        feed()
    except OSError as e:
        # unreadable (corrupt LittleFS block, EIO): hold it back so the
        # newer tracks still go, and after READ_ERR_MAX tries park it as
        # <file>.err (left for a manual look) instead of blocking for good
        _up_err("fs err")
        re_ = up["read_err"]
        re_[path] = re_.get(path, 0) + 1
        print("upload read", path, repr(e))
        if re_[path] >= READ_ERR_MAX:
            re_.pop(path, None)
            try:
                os.rename(path, path + ".err")
                _rm(path + ".pos")
                up["err_files"] += 1
                print("parked", path + ".err")
                _diag_write("upload: %s unreadable, parked as .err" % path)
                # its points, never sent, leave "left" (it held the OTA);
                # unreadable, it cannot be counted: count the rest
                up["left"] = count_points()
            except OSError as e2:        # (held: tried again after the hold)
                print("upload park", path, repr(e2))
                _diag_write("upload: %s unreadable, not parked %r"
                            % (path, e2))
        _hold_file(path, now, BACKOFF_MAX)
        _schedule(now)
        return
    except MemoryError:
        up["skip"] = path
        _low_mem(now)
        return
    up["read_err"].pop(path, None)       # (in a row: READ_ERR_MAX spread over a
                                         # boot parked a good track as .err)
    if not size:
        # nothing to send: a write error here is a flash error, not an
        # unreadable track (READ_ERR_MAX of them parked a good one as .err)
        try:
            if n:
                commit_offset(path, n, new_off, eof, key1)
            else:
                # n == 0 only at EOF: read_batch leaves its loop early only
                # with a point counted (the batch full, one that does not
                # fit), so a file with no line to count is read to its end
                _rm_done(path)           # empty / EOF track and .bad
                _rm(path + ".pos")
                _clear_bad_full(path)
        except OSError as e:
            print("upload pos", path, repr(e))
            _fs_err(now)
            return
        except MemoryError:
            _low_mem(now)
            return
        # no POST was made: the rest of a damaged stretch is read on at
        # once (the .pos moved, so this ends), not a gap per UPLOAD_SCAN
        _schedule(now, 0 if n and not eof else UPLOAD_GAP)
        return
    gc.collect()                         # frees an empty grown heap area
    # both margins keep the IDF block free for TLS: not for plain http
    # (there they stopped the uploads and rebooted the board for nothing)
    tls = up["url"].startswith("https")
    big = idf_heap()[1] if tls else None
    if big is not None and big < TLS_MIN_KB:
        # TLS cannot get its block: every try would time out or fail to
        # allocate (seen as "err net" until a reboot on 2026-10-30)
        if not _low_stop(now, "low idf", "idf %d KB", big, "lowidf_at",
                         "lowidf_seen"):
            _schedule(now, LOWIDF_RETRY if at_home() else None)
        return
    up["lowidf_at"] = None
    pf = py_heap()[0] if tls else None
    if pf is not None and pf < TLS_MIN_PY_KB * 1024:
        # an allocation that does not fit after a GC grows the heap by up
        # to its own size, out of the IDF block TLS needs: wait instead
        # (a fixed retry: the doubling backoff waited up to BACKOFF_MAX
        # for a heap that is back after the next collect). It retried for
        # ever; at home, low for LOWIDF_WAIT, it reboots as low idf does
        if not _low_stop(now, "low mem", "mem %d B", pf, "lowpy_at",
                         "lowpy_seen"):
            # (away the doubling backoff, as low idf: there is no reboot
            # there, and 30 s, under WIFI_OFF_WAIT, kept the radio on)
            _schedule(now, LOWIDF_RETRY if at_home() else None)
        return
    up["lowpy_at"] = None
    if tls:
        _rtc_sync(now)                   # certificate dates need the time
        if not _tls_ready(now):
            return
        if _away_unverified():           # (CA_FILE gone since the check
            _hold_unverified(now)        # above: the context made now is
            return                       # CERT_NONE)
    if not _bad_full_left():
        up["state"] = "sending"
    _rcv.read_uart()                          # drain NMEA before blocking
    fast = CPU_MHZ_UPLOAD and CPU_MHZ and CPU_MHZ_UPLOAD != CPU_MHZ
    if fast:
        _set_freq(CPU_MHZ_UPLOAD)        # TLS handshake at full speed
    t_post = time.ticks_ms()
    try:
        _heap_mark()
        code, retry = _http_post(size)
    except MemoryError:
        up["skip"] = path
        _low_mem(time.ticks_ms())
        _tls_reboot()
        return
    finally:
        if fast:
            _set_freq(CPU_MHZ)
        _heap_note("post")
    _rcv.read_uart()                          # catch up after a multi-second POST
    now = time.ticks_ms()
    print("upload", path, n, n_ok, code, py_heap()[0],
          "%d ms" % time.ticks_diff(now, t_post))
    if code is None and _tls_reboot():
        return
    gap = UPLOAD_GAP_AWAY if not at_home() else UPLOAD_GAP
    if code is not None and 200 <= code < 300:
        # the day history gets the points once they are up (at flush they
        # were on the flash twice while they waited)
        # (not again for a batch that went up before and whose .pos write
        # failed: each retry of it added the points to the history again)
        # (a reboot between: HIST_DONE's note on the flash says so)
        pw = up["pos_wr"].get(path)
        hd = path in up["hd"]
        if not (pw and pw[0] == off) and not (hd and _hist_done(path) == off):
            _hist_batch(path, off, new_off, key)
        try:
            commit_offset(path, n, new_off, eof, key1)
        except OSError as e:
            _pos_wr_err(path, off, e, now)
            return
        except MemoryError:              # (the batch goes again: no
            up["skip"] = path            # _schedule, no state, it was sent
            if not (pw and pw[0] == off):    # once more after TICK_PAUSE;
                up["pos_wr"][path] = [off, 0]   # its points drawn: not again,
            gc.collect()                        # a reboot between too; the
            _hist_done_note(path, off)          # note's open() after gc)
            _low_mem(now)
            return
        if hd:
            up["hd"].discard(path)
            _rm(path + HIST_DONE)
        up["sent"] += n_ok
        up["ok_at"] = _stamp()
        up["fail4"] = 0
        up["pos_wr"].pop(path, None)     # (per track: two failing ones
                                         # reset each other's count)
        se = up["srv"]
        if se and se[0] == path:         # (another's run goes on: lifted
            up["srv"] = None             # below, its next error holds again)
        up["backoff"] = BACKOFF_START
        up["allow_bad"] = True
        up["skip"] = None
        up["hold"].pop(path, None)
        sh = up["srv_hold"]
        if sh:                           # the server answers: server-error
            for p in sh:                 # holds go, payload holds stay
                up["hold"].pop(p, None)
            sh.clear()
        _clear_bad_full(path)
        if not _bad_full_left():
            up["state"] = "ok"
        _schedule(now, gap)
    elif code in PAYLOAD_ERR:
        up["err"] += 1
        if path.endswith(".bad"):
            # count the rejections of a .bad, not its successes: it used to
            # stop after MAX_BAD_RETRY good batches until the next boot.
            # Not in fail4: rejected .bad batches kept a normal track's
            # bad batch from quarantine until the next 2xx
            bt = up["bad_try"]
            bt[path] = bt.get(path, 0) + 1
        else:
            up["fail4"] = up["fail4"] + 1
        if path.endswith(".bad") or up["fail4"] >= MAX_GLOBAL_PAYLOAD:
            up["state"] = "err payload"
            _hold_file(path, now, retry if retry else BACKOFF_MAX)
            _schedule(now, gap)
            return
        if not _quarantine_batch(path, off, n, new_off, eof, key, key1):
            _hold_file(path, now, BACKOFF_MAX)
            _schedule(now, gap)
            return
        _schedule(now, gap)
    elif code == 401:
        _up_err("err 401")
        _schedule(now, retry if retry else AUTH_BACKOFF)
    else:
        _up_err("err " + ("net" if code is None else str(code)))
        up["err_ssid"] = wifi["ssid"]        # (_up_net_new: a new network)
        _schedule(now, retry)
        if code is not None and len(files) > 1:
            # the same batch answered so SRV_ERR_MAX times in a row (a 5xx
            # from one point, a 404 / 403 of a proxy): hold that track, the
            # others go (only the backoff: files[0] for ever, the rest
            # blocked behind it). The only one waiting: the backoff alone.
            # The first hold of a run a 2xx of another track lifts (the
            # server was down, not this batch: a Dawarich restart held the
            # oldest track 30 min); the run's count stays, so the batch
            # failing once more is held again, by a hold no 2xx lifts
            se = up["srv"]
            if se and se[0] == path and se[1] == off:
                se[2] += 1
            else:
                se = up["srv"] = [path, off, 1]
            if se[2] >= SRV_ERR_MAX:
                _hold_file(path, now, BACKOFF_MAX, se[2] == SRV_ERR_MAX)

def _fs_err(now):
    """A .pos write (or delete) failed: retry later, the track stays."""
    _up_err("fs err")
    _schedule(now)

HIST_DONE = ".hd"                        # <track>.hd: the offset of a batch that
                                         # went up and into the history, its .pos
                                         # not written (a reboot then drew it twice)

def _hist_done_note(path, off):
    """HIST_DONE for the batch at off, once (each path its own note; best
    effort: the flash that took no .pos may take no note either)."""
    if path in up["hd"] and _hist_done(path) == off:
        return
    try:
        with open(path + HIST_DONE, "w") as f:
            f.write("%d\n" % off)
        up["hd"].add(path)
    except (OSError, MemoryError):       # (pos_wr in RAM still holds it)
        pass

def _hist_done(path):
    """The offset in path's HIST_DONE note, None without one."""
    try:
        with open(path + HIST_DONE) as f:
            return int(f.read(16))
    except (OSError, ValueError):
        return None

def _pos_wr_err(path, off, e, now):
    """The batch at off went up (2xx) and its .pos (or the delete at EOF)
    could not be written: sent again at the retry, without an end before.
    Counted per batch; at READ_ERR_MAX in a row a diag line (once) and the
    track is held BACKOFF_MAX so the newer ones still go."""
    pw = up["pos_wr"].get(path)
    if pw and pw[0] == off:
        pw[1] += 1
    else:
        pw = up["pos_wr"][path] = [off, 1]
    _hist_done_note(path, off)
    print("upload pos write", path, repr(e))
    if pw[1] >= READ_ERR_MAX:
        if pw[1] == READ_ERR_MAX:
            _diag_write("upload: %s sent, its .pos not written %r: held"
                        % (path, e))
        _hold_file(path, now, BACKOFF_MAX)
    _fs_err(now)

def _low_mem(now):
    gc.collect()
    _up_err("low mem")
    _schedule(now)
    print("upload MemoryError, free", py_heap()[0])

# --- OTA ---------------------------------------------------------------------
# Firmware updates over WiFi. They need the OTA partition table (two app
# slots: tools/build_firmware.sh --migrate, once over USB). At home and with
# everything uploaded, OTA_FIRST after boot and then every OTA_CHECK, the
# board reads OTA_URL: the manifest tools/build_firmware.sh --publish wrote,
# {"build", "commit", "size", "sha256", "url", "files", "hmac"}. A build
# newer than the running one (FW_BUILD: the commit time), whose HMAC-SHA256
# with OTA_KEY checks, goes into the other app slot OTA_BLOCK bytes at a
# time (the GPS read and the WDT fed between). It is read back against its
# sha256, the files (the status page) are written to flash, the slot is set
# to boot and the board resets. The manifest's fields are checked first
# (_ota_manifest_ok: its HMAC string joins them with "|"). The new firmware
# then runs pending (OTA_PENDING): OTA_VALID_AFTER of running with WiFi
# connected, and the manifest read again (the update path works), marks it
# valid, and only then do the new page files (staged as .new) replace the
# old ones. A crash or reset before that and the bootloader boots the old
# one; no WiFi or manifest in OTA_HEALTH and it resets itself, the same. A
# build that crashed (run()'s crash branch notes it in OTA_MARK) or whose
# watchdog fired while the logger ran is in OTA_BAD and not tried again.
# Any other rollback is a try (OTA_TRIES): a power-on (a brownout too: the
# cable), no WiFi or manifest (the router), and a reset from outside
# (mpremote reset, a USB-JTAG reset, the watchdog after a Ctrl-C stopped
# the logger: OTA_MARK "stop"), and a panic too (MicroPython 1.29 reports
# it as a hard reset, like mpremote's; no Python code ran to note it).
# Tried again after OTA_RETRY; after OTA_TRIES_MAX held for OTA_HOLD of
# UTC (doubling each round), not bad for good: two power cuts in the
# pending window banned a good build forever. The old firmware's own
# reset before ota_boot read the note is not the new build's: gps_boot
# notes the first boot's cause in OTA_MARK ("cause <n>"). No set_boot() back:
# that left the old image pending, and the next reset fell back to ota_0,
# the failed build. A Ctrl-C between set_boot and the reset still resets
# (otadata names the new slot already: left there, the next reset booted
# it with no pending note, valid without a health check); a boot slot that
# is not the running one at boot (set_boot ran, no reset since) is booted
# once by ota_boot. A /gps_logger.mpy or .py on flash (FORCE_MPY=1) comes
# before the frozen logger in sys.path: the install moves it aside
# (<name>.ota-old), the valid firmware deletes it, main.py puts it back
# after a rollback.
OTA_URL = None                           # .env OTA_URL: the manifest
OTA_KEY = None                           # .env OTA_KEY: its HMAC key (bytes)
OTA_FIRST = 120000                       # ms after boot of the first check
OTA_CHECK = 600000                       # ms between checks (a ~1 KB manifest
                                         # on the LAN)
OTA_RETRY = 3600000                      # ms before a build whose download
                                         # failed is downloaded again
OTA_VALID_AFTER = 120000                 # ms a new firmware runs, WiFi
                                         # connected, before it is valid
OTA_HEALTH = 600000                      # ms it may take to get WiFi
OTA_MAX = 300000                         # ms one download may take (the
                                         # guard closes its socket)
OTA_BLOCK = 4096                         # bytes a flash write (an erase page)
OTA_MANIFEST_MAX = 4096                  # bytes a manifest may have
OTA_PENDING = "/ota.pending"             # "<build> <commit>" just installed,
                                         # then the staged files, a line each
OTA_BAD = "/ota.bad"                     # builds that failed their health
OTA_TRIES = "/ota.tries"                 # "<build> <n>": its rollbacks after
                                         # a power-on or with no WiFi/manifest
OTA_TRIES_MAX = 2                        # such rollbacks before it is held
OTA_DL = "/ota.dl"                       # "<build> <n>[ 1]": an install begun
                                         # (the download on), n of them cut
                                         # by a reset in a row, 1: the last
                                         # cut counted; gone with its end
OTA_CUT_PER_TRY = 3                      # cut installs that count one try (a
                                         # brownout under the download's load
                                         # reinstalled it every boot: 600 slot
                                         # erases a day, never held)
OTA_HOLD = 86400                         # s of UTC a build is held after
                                         # OTA_TRIES_MAX tries (doubled each
                                         # round, up to 32 times)
OTA_PROBE = 60000                        # ms between manifest reads of a
                                         # pending firmware
OTA_MARK = "/ota.mark"                   # what ended the pending run: crash,
                                         # stop (Ctrl-C), boot (ota_boot's
                                         # reset into a slot not run yet),
                                         # valid (being confirmed), cause <n>
                                         # (gps_boot: the old firmware's
                                         # first reset cause after it)
OTA_SHADOW = ("/gps_logger.mpy", "/gps_logger.py",
              "/gps_clock.mpy", "/gps_clock.py",
              "/gps_receiver.mpy", "/gps_receiver.py",
              "/gps_tft.mpy", "/gps_tft.py",
              "/gps_tft_font.mpy", "/gps_tft_font.py")  # moved aside (.ota-old)
                                         # for a frozen logger to run
OTA_STUCK = 86400000                     # ms an upload may have no 2xx and
                                         # still hold the update check back
OTA_PAGES = ("/web.html", "/web.html.gz")  # the files a manifest may name:
                                         # build_firmware.sh names the gzip
                                         # alone now (/web.html: the older
                                         # manifests, still "current")
try:
    from gps_frozen import BUILD as FW_BUILD, COMMIT as FW_COMMIT
except ImportError:                      # (not frozen, or a firmware older
    FW_BUILD, FW_COMMIT = 0, "dev"       # than BUILD in gps_frozen)

ota = {"at": None, "state": "off", "boot_at": _stamp(), "pend": None,
       "pend_at": None, "wifi_seen": False}

def _ota_env(env):
    """OTA_URL and OTA_KEY in .env: both or no updates."""
    global OTA_URL, OTA_KEY
    url = _plain(env.get("OTA_URL"))
    key = env.get("OTA_KEY")
    if isinstance(key, str):
        key = key.strip().encode()
    if url and key and url.startswith(("http://", "https://")):
        OTA_URL, OTA_KEY = url, key
        if ota["state"] == "off":        # (not over "pending", "rolled back")
            ota["state"] = "idle"
    elif url or key:
        _env_refused("OTA_URL/OTA_KEY", "need both and an http(s) URL", "no updates")

def _hmac_hex(key, msg):
    """HMAC-SHA256 of msg as hex (no hmac module in MicroPython)."""
    if len(key) > 64:
        key = hashlib.sha256(key).digest()
    key = key + b"\0" * (64 - len(key))
    inner = hashlib.sha256(bytes(b ^ 0x36 for b in key) + msg).digest()
    out = hashlib.sha256(bytes(b ^ 0x5C for b in key) + inner).digest()
    return binascii.hexlify(out).decode()

OTA_BOARD = "GPS_LOGGER_S3"              # (the manifest's "board")

def _ota_eq(a, b):
    """a == b for two strings in a time independent of where they differ
    (!= stops at the first different character: a timing oracle for the
    HMAC)."""
    if type(a) is not str or type(b) is not str or len(a) != len(b):
        return False
    x = 0
    for i in range(len(a)):
        x |= ord(a[i]) ^ ord(b[i])
    return x == 0

def _ota_signed(m):
    """The bytes the manifest's HMAC covers (build_firmware.sh signs the
    same): build|commit|size|sha256|url, then path|size|sha256|url a file,
    then |board=<name> (OTA_BOARD)."""
    t = "%d|%s|%d|%s|%s" % (m["build"], m["commit"], m["size"], m["sha256"],
                            m["url"])
    for f in m.get("files") or ():
        t += "|%s|%d|%s|%s" % (f["path"], f["size"], f["sha256"], f["url"])
    if m.get("board") is not None:
        t += "|board=" + m["board"]
    return t.encode()

def _ota_str(v, hexlen=0):
    """A manifest string field the "|"-joined HMAC string cannot be
    re-split at; hexlen: lowercase hex of that length."""
    if type(v) is not str or "|" in v:
        return False
    return not hexlen or (len(v) == hexlen
                          and all(c in "0123456789abcdef" for c in v))

def _ota_commit(v):
    """A commit as build_firmware.sh writes it: 1-20 lowercase hex (git's
    short hash), maybe "-dirty". It goes to the diag log and the page: a
    newline or 4 KB in it wrote forged diag lines."""
    if type(v) is not str:
        return False
    if v.endswith("-dirty"):
        v = v[:-6]
    return 0 < len(v) <= 20 and _ota_str(v, len(v))

def _ota_manifest_ok(m):
    """The fields of a manifest are what build_firmware.sh writes: ints,
    strings without "|", a commit (_ota_commit), 64 hex sha256 and only
    the OTA_PAGES, each once.
    _ota_signed joins them with "|", unescaped: a manifest moving a "|"
    between fields kept the HMAC (a page path swallowing the next file, a
    commit swallowing the firmware: the page installed as firmware). The
    format stays, boards in the field read the same manifests. A manifest
    for another board (OTA_BOARD: "board") is not this
    board's firmware, whatever its signature."""
    try:
        if type(m) is not dict or m.get("board") != OTA_BOARD:
            return False
        if (type(m["build"]) is not int or type(m["size"]) is not int
                or not _ota_commit(m["commit"]) or not _ota_str(m["url"])
                or not _ota_str(m["sha256"], 64)):
            return False
        files = m.get("files") or []
        if type(files) is not list:
            return False
        seen = []
        for f in files:
            if (type(f) is not dict or f.get("path") not in OTA_PAGES
                    or f["path"] in seen or type(f.get("size")) is not int
                    or not _ota_str(f.get("url"))
                    or not _ota_str(f.get("sha256"), 64)):
                return False
            seen.append(f["path"])
        return True
    except (KeyError, TypeError, AttributeError):
        return False

OTA_EXPIRY = "|expires=%d"               # (hmac2: the signed string, then this)
EPOCH_2020 = 1577836800                  # Unix time of 2020-01-01 (FW_BUILD's)
OTA_SEEN = "/ota.seen"                   # the newest signed build seen

def _ota_seen():
    """The newest build whose manifest checked (hmac, hmac2, fresh) and
    could be installed: an older one is no update after it (a replay of a
    manifest the server replaced, signed and fresh still), and its build
    time (the commit's, signed) is a floor for the clock. 0 without the
    file (or OTA_SEEN None: no record). Any other read error (EIO) is 0
    for the floor with ota["seen_err"] set and nothing kept: taken for "no
    file", a replayed older manifest passed and its build was written
    over the newer one (the record gone for good); _ota_check refuses
    then, and the next check reads again."""
    b = ota.get("seen")
    if b is None:
        b = 0
        if OTA_SEEN:
            try:
                with open(OTA_SEEN) as f:
                    b = int(f.read(24).split()[0])
            except OSError as e:
                if not _enoent(e):
                    ota["seen_err"] = True
                    return 0
            except (ValueError, IndexError):
                pass                     # (written by rename: no torn one)
        ota["seen_err"] = False
        ota["seen"] = b
    return b

def _ota_seen_note(build):
    if not OTA_SEEN or build <= _ota_seen() or ota.get("seen_err"):
        return
    ota["seen"] = build
    tmp = OTA_SEEN + ".tmp"
    try:
        with open(tmp, "w") as f:
            f.write("%d\n" % build)
        os.rename(tmp, OTA_SEEN)
    except OSError as x:                 # (kept for this boot in ota)
        _rm(tmp)
        _bug_once("bug ota seen not saved %s" % repr(x)[:50], ("ota seen", type(x)))

def _clock_floor():
    """Seconds since 2020 no clock can be before: this firmware's build
    time, or the newest signed build seen (_ota_seen); 0 for neither."""
    return max(0, max(FW_BUILD, _ota_seen()) - EPOCH_2020)

def _ota_fresh(m):
    """The manifest's freshness: "expires" (seconds since 2020, UTC) and
    "hmac2", its HMAC over _ota_signed + OTA_EXPIRY (a second one: "hmac"
    stays as it was, so a board of an older build still reads the
    manifest that updates it). Every older signed manifest had no end: one
    left on the server or replayed by the LAN could be served again. ->
    None, or why not (no expiry, its hmac, expired: only with a clock)."""
    e, h = m.get("expires"), m.get("hmac2")
    if type(e) is not int or not _ota_str(h, 64):
        return "no expiry"
    if not _ota_eq(_hmac_hex(OTA_KEY, _ota_signed(m) + (OTA_EXPIRY % e).encode()), h):
        return "bad hmac2"
    c = log.get("clock") or _ntp[0]
    # (no clock: indoors, NTP dropped by the LAN; the floor no replay can
    # turn back, _clock_floor, holds with one too: an NTP answer of the
    # build's time made every old signed manifest fresh)
    now = max(_utc_s(c) if c is not None else 0, _clock_floor())
    if now > e:
        return "expired"
    return None

def _ota_sign(key, m):
    """hmac and hmac2 as tools/firmware.py writes them (tests, simulators)."""
    m["hmac"] = _hmac_hex(key, _ota_signed(m))
    if "expires" in m:
        m["hmac2"] = _hmac_hex(key, _ota_signed(m)
                               + (OTA_EXPIRY % m["expires"]).encode())
    return m

def _ota_url(rel):
    """A manifest URL relative to the manifest's own folder."""
    if rel.startswith(("http://", "https://")):
        return rel
    return OTA_URL[:OTA_URL.rfind("/") + 1] + rel

OTA_GET_SMALL = 30000                    # ms a GET of at most OTA_SMALL_MAX
OTA_SMALL_MAX = 65536                    # bytes may take (manifest, page)

def _get_deadline(limit):
    return OTA_GET_SMALL if limit <= OTA_SMALL_MAX else OTA_MAX

def _raw_get(url, sink, limit):
    """A minimal HTTP/1.0 GET, no redirects: the body goes to sink(memoryview)
    in pieces, the UART read and the WDT fed between, at most limit bytes
    (more: ValueError). The guard closes the socket after OTA_MAX. -> the
    status code (sink only gets a 200's body). A small body (the manifest,
    a page) gets OTA_GET_SMALL: a server that drips one byte each 14 s held
    the loop the image's whole 300 s, each OTA_CHECK."""
    s, addr, https, host, hdr = _http_open("GET", url)
    _guard_start(s, _get_deadline(limit))
    try:
        if https:
            _rtc_sync(time.ticks_ms())   # (the certificate's dates)
        s.settimeout(HTTP_TIMEOUT)
        s.connect(addr)
        if https:
            s = _tls_wrap(s, host)
        _io_write(s, (hdr + "Connection: close\r\n\r\n").encode())
        line = s.readline(HTTP_LINE).split(None, 2)
        if len(line) < 2:                # (closed before a status line)
            raise OSError(104)           # ECONNRESET
        code = int(line[1])
        if code != 200:                  # (its head is of no use: the code
            return code                  # comes back, however long it is)
        cl = [None]
        if not _http_head(s, (b"content-length",), cl):
            raise OSError(5)             # (a head cut short: not an empty
                                         # body's ValueError further on)
        clen = None
        try:
            clen = int(cl[0])
        except (TypeError, ValueError):  # (TypeError: none sent)
            pass
        if clen is not None and clen > limit:
            raise ValueError("ota body %d over %d" % (clen, limit))
        buf = bytearray(1024)
        mv = memoryview(buf)
        got = 0
        # the body ends at Content-Length (a TLS server closing without
        # close_notify raised at EOF), else at EOF
        while clen is None or got < clen:
            k = 1024 if clen is None else min(1024, clen - got)
            n = s.readinto(mv[:k])
            if n is None:
                raise OSError(110)       # (a blocking socket timed out)
            if not n:
                break
            got += n
            if got > limit:
                raise ValueError("ota body over %d" % limit)
            sink(mv[:n])
            _rcv.read_uart()
            feed()
        if clen is not None and got != clen:
            raise ValueError("ota body %d of %d" % (got, clen))
        return code
    finally:
        _http_close(s)

HTTP_GET = _raw_get                      # (tests swap it)

class _SlotWriter:
    """Writes a download into an app slot OTA_BLOCK bytes at a time (the
    last block padded with 0xFF) and hashes it on the way."""
    def __init__(self, part):
        self.part, self.blk, self.n = part, 0, 0
        self.buf = bytearray(OTA_BLOCK)
        self.h = hashlib.sha256()
    def __call__(self, mv):
        i = 0
        while i < len(mv):
            k = min(len(mv) - i, OTA_BLOCK - self.n)
            self.buf[self.n:self.n + k] = mv[i:i + k]
            self.n += k
            i += k
            if self.n == OTA_BLOCK:
                self._put()
    def _put(self):
        self.h.update(memoryview(self.buf)[:self.n])
        for j in range(self.n, OTA_BLOCK):
            self.buf[j] = 0xFF
        self.part.writeblocks(self.blk, self.buf)
        self.blk += 1
        self.n = 0
        _rcv.read_uart()
        feed()
    def close(self):
        if self.n:
            self._put()
        return binascii.hexlify(self.h.digest()).decode()

def _slot_sha(part, size):
    """sha256 of the first size bytes of a slot, read back from flash."""
    h = hashlib.sha256()
    buf = bytearray(OTA_BLOCK)
    blk = 0
    while size > 0:
        part.readblocks(blk, buf)
        h.update(memoryview(buf)[:min(size, OTA_BLOCK)])
        size -= OTA_BLOCK
        blk += 1
        _rcv.read_uart()
        feed()
    return binascii.hexlify(h.digest()).decode()

def _ota_bad():
    """The builds marked bad. A token that is no number (bit rot, a hand
    edit) is skipped, not the whole list: OTA_BAD is only appended to, so
    one bad token forgot every bad build for good, and a crashing build was
    installed again after each rollback."""
    out = []
    try:
        with open(OTA_BAD) as f:
            for x in f.read().split():
                try:
                    out.append(int(x))
                except ValueError:
                    pass
    except OSError:
        pass
    return out

def _ota_mark_bad(build):
    try:
        with open(OTA_BAD, "a") as f:
            f.write("%d\n" % build)
    except OSError as e:
        _diag_write("ota bad mark %d %r" % (build, e))

def _ota_tries_raw():
    """The ints of OTA_TRIES: [build, n] or [build, n, UTC the hold began],
    or None."""
    try:
        with open(OTA_TRIES) as f:
            v = [int(x) for x in f.read().split()]
        return v if 2 <= len(v) <= 3 else None
    except (OSError, ValueError):
        return None

def _ota_tries():
    """(build, n) of OTA_TRIES, or (None, 0)."""
    v = _ota_tries_raw()
    return (v[0], v[1]) if v else (None, 0)

def _ota_tries_put(v):
    """OTA_TRIES as "<build> <n>[ <utc>]" (the two words an older firmware
    reads, after a rollback, until a hold begins). A failed write is a
    diag line: the count is lost, the build installed again sooner."""
    try:
        with open(OTA_TRIES, "w") as f:
            f.write(" ".join("%d" % x for x in v) + "\n")
    except OSError as e:
        _diag_write("ota tries %d not written: %r" % (v[0], e))

def _ota_try(build):
    """A rollback that may not be the build's fault: at power-on (MicroPython
    1.29 reports a brownout as one: a weak supply), or no WiFi or manifest
    in OTA_HEALTH (the router, the server). Every OTA_TRIES_MAX-th of one
    build holds it (_ota_held): never blamed, a brownout reinstalled it
    after every boot; blamed for good (OTA_BAD, as before), two power cuts
    in the pending window or a router outage excluded a good build forever.
    -> its tries now (held at each multiple of OTA_TRIES_MAX)."""
    b, n = _ota_tries()
    n = n + 1 if b == build else 1
    _ota_tries_put([build, n])
    return n

def _ota_hold_s(n):
    """The hold after n tries: OTA_HOLD, doubled each round (32x at most)."""
    return OTA_HOLD << min(n // OTA_TRIES_MAX - 1, 5)

def _ota_held(build):
    """Seconds the build is still held after OTA_TRIES_MAX tries (0: not
    held): OTA_HOLD, doubled each round (up to 32 times). On UTC, the
    Clock's: Stamps start again at each boot, and a board on an hour a day
    would hold it for good. The hold begins at the first check with a
    clock (the rollback is at boot, the clock not known yet); without one
    it holds. A start in the future (a clock that went back) begins
    again."""
    v = _ota_tries_raw()
    if (not v or v[0] != build or v[1] < OTA_TRIES_MAX
            or v[1] % OTA_TRIES_MAX):
        return 0
    hold = _ota_hold_s(v[1])
    now = _utc_s(log["clock"] or _ntp[0])
    if now is None:
        return hold
    if len(v) < 3 or v[2] > now:
        _ota_tries_put([build, v[1], now])
        return hold
    left = v[2] + hold - now
    return left if left > 0 else 0

def _ota_dl_get():
    """[build, n, counted] of OTA_DL, or None."""
    try:
        with open(OTA_DL) as f:
            v = [int(x) for x in f.read().split()]
        if len(v) == 2:
            v.append(0)
        return v if len(v) == 3 else None
    except (OSError, ValueError):
        return None

def _ota_dl_put(build, n, counted):
    """OTA_DL written. Never raises."""
    try:
        with open(OTA_DL, "w") as f:
            f.write("%d %d 1" % (build, n) if counted else
                    "%d %d" % (build, n))
    except Exception as e:
        _diag_write("ota dl note not written: %r" % e)

def _ota_dl_begin(build):
    """An install of build begins: one more in a row when the last one of
    it was cut (its OTA_DL still there). Never raises."""
    v = _ota_dl_get()
    _ota_dl_put(build, v[1] + 1 if v and v[0] == build else 1, 0)

def _ota_cut_seen():
    """At boot, an install that a reset cut (its OTA_DL there, no pending
    note it got to): OTA_RETRY before the next one, not OTA_FIRST, and
    every OTA_CUT_PER_TRY-th a try (held as a rollback's, never blamed).
    The note stays, for the count, marked counted: a later boot (before
    the next install) counted the same cut again, a try each time, and an
    odd try count lifted the hold. A note of the running build or an older
    one (a reset between set_boot and its removal, a newer build flashed
    over USB) is no cut: dropped."""
    v = _ota_dl_get()
    if v is None:
        return
    build, n, counted = v
    if build <= FW_BUILD:                # (a USB flash of a newer one: that
        _rm(OTA_DL)                      # note was dead state for good)
        return
    ota["failed"] = (build, _stamp(), "install cut")
    if counted:
        return
    if n % OTA_CUT_PER_TRY == 0:
        _ota_try(build)
    _ota_dl_put(build, n, 1)
    _diag_write("ota build %d install cut by a reset (%d in a row): next in "
                "%d min" % (build, n, OTA_RETRY // 60000))

def _ota_wait_tried():
    """A build with a try behind it is installed again OTA_RETRY after
    boot, not OTA_FIRST (ota["failed"] is RAM: gone with the reset)."""
    b = _ota_tries()[0]
    if b is not None:
        ota["failed"] = (b, _stamp(), "rolled back")

def _ota_file(f):
    """One manifest file (the page) staged on flash as <path>.new: it
    replaces the old one only once the new firmware is valid (_ota_promote:
    a rollback keeps the old page with the old firmware)."""
    tmp = f["path"] + ".tmp"
    h = hashlib.sha256()
    ok = False
    try:
        with open(tmp, "wb") as out:
            def sink(mv):
                h.update(mv)
                out.write(mv)
            code = HTTP_GET(_ota_url(f["url"]), sink, f["size"])
        if code != 200 or binascii.hexlify(h.digest()).decode() != f["sha256"]:
            raise ValueError("ota file %s: %s" % (f["path"], code))
        os.rename(tmp, f["path"] + ".new")
        ok = True
    finally:
        if not ok:
            _rm(tmp)

def _ota_promote(paths):
    """The staged page files over the old ones (the firmware is valid). A
    file system that does not rename over an existing file: the old one is
    moved aside first (and back if the new one still does not go up). One
    that cannot go up is a diag line: it was silent, and the new firmware
    served the old page until the next update.
    A gzip page that went up removes its plain twin (/web.html.gz:
    /web.html, ~70 KB of flash) unless the manifest named that one too: the
    board keeps the page only gzipped and inflates it for a client without
    gzip (_web_page). Only here, with the firmware valid: a rollback keeps
    the old firmware's files, and the gzip a manifest names is made with
    WEB_GZ_WBITS (one from an older deploy.sh may have a window too wide
    to inflate: its plain page stays until the next deploy). Safe to run
    again (_ota_health tries again after a MemoryError): a page already up
    has no .new."""
    for p in paths:
        if _file_size(p + ".new") is not None:  # (not again after a retry)
            try:
                os.rename(p + ".new", p)
            except OSError:
                try:
                    os.rename(p, p + ".old")
                    try:
                        os.rename(p + ".new", p)
                    except OSError:
                        os.rename(p + ".old", p)
                        raise
                    _rm(p + ".old")
                except OSError as e:
                    _diag_write("ota %s page %s not put up: %r" % (
                        FW_COMMIT, p, e))
                    continue
        if p.endswith(".gz") and p[:-3] not in paths:
            _rm(p[:-3])

def _ota_drop(paths):
    for p in paths:
        _rm(p + ".new")
        _rm(p + ".tmp")                  # (a power cut while one was read)

def _ota_shadow(aside):
    """A /gps_logger.mpy or .py (FORCE_MPY=1) shadows the frozen logger:
    '' comes before .frozen in sys.path, and after an OTA the diag said
    "ota <new> ok" while the old code ran. aside: moved to <name>.ota-old
    (before set_boot); else put back (an install that failed; after a
    rollback main.py does it, before the import). Only on a frozen
    firmware (FW_BUILD): a plain one needs its .mpy to boot. A module that
    is there and does not move (not ENOENT) is a diag line, and aside it
    raises: the install stops before set_boot (it went on, and the old
    module ran on the new image)."""
    for p in OTA_SHADOW:
        try:
            if aside:
                os.rename(p, p + ".ota-old")
            elif _file_size(p) is None:
                os.rename(p + ".ota-old", p)
        except OSError as e:
            if _enoent(e):
                continue
            _diag_write("ota %s %s: %r" % (
                "cannot move aside" if aside else "cannot put back", p, e))
            if aside:
                raise

def _ota_mark(what):
    """What ended a run while an OTA note is there, for the firmware that
    reads the note next (the old one, after a rollback): "crash" (run()'s
    crash branch: the build is bad), "stop" (Ctrl-C, an mpremote session:
    the watchdog reset after it is not the build's fault), "boot"
    (ota_boot's reset into a slot set to boot but not run). Never
    raises. A crash or stop is the pending build's only when it is the one
    running (gps_boot.crash() checks the same): the old firmware back after
    a rollback for no fault of the new one, crashing before ota_boot read
    the note, marked the new build bad for good (a Ctrl-C there turned a
    watchdog verdict on it into a try). A write that fails is a diag
    line (the old firmware then misreads the rollback)."""
    try:
        p = _ota_pending()
        if p is None or (what != "boot" and p[0] != FW_BUILD):
            return
        if what in ("crash", "stop") and (ota.get("confirmed")
                                          or _ota_mark_get() == "valid"):
            return                       # (confirmed: the next boot ends it)
        with open(OTA_MARK, "w") as f:
            f.write(what)
    except Exception as e:
        try:
            _diag_write("ota mark %s not written: %r" % (what, e))
        except Exception:
            pass

def _ota_mark_get():
    try:
        with open(OTA_MARK) as f:
            return f.read().strip()
    except Exception:
        return None

def _ota_boot_slot():
    """The label of the slot otadata boots next, or None."""
    try:
        return esp32.Partition(esp32.Partition.BOOT).info()[4]
    except Exception:
        return None

def _ota_unbooted():
    """Whether the slot set to boot is not the running one: set_boot ran
    and the board has not reset since. (After a rollback the bootloader
    marks the failed slot aborted and otadata boots the running one.)"""
    b = _ota_boot_slot()
    try:
        return b is not None and b != esp32.Partition(
            esp32.Partition.RUNNING).info()[4]
    except Exception:
        return False

def _ota_say(line):
    """A refused manifest's diag line, once until a manifest checks (one
    every OTA_CHECK flooded the log)."""
    if line != ota.get("said"):
        ota["said"] = line
        _diag_write(line)

def _ota_check():
    """Read the manifest; install a newer, signed build (resets the board)."""
    ota["state"] = "checking"
    body = bytearray()
    code = HTTP_GET(OTA_URL, body.extend, OTA_MANIFEST_MAX)
    if code != 200:
        ota["state"] = "manifest %s" % code
        return
    m = json.loads(body)
    if not _ota_manifest_ok(m):
        ota["state"] = "bad manifest"
        _ota_say("ota: manifest fields refused")
        return
    build = m["build"]
    if build <= FW_BUILD:
        ota["state"] = "current"
        return
    if build in _ota_bad():
        # (its own state: "current", the page said "up to date" over a
        # newer build that is never installed; the commit is checked hex)
        ota["state"] = "bad " + m["commit"]
        return
    if not FW_BUILD:
        # a plain firmware (no gps_frozen): the modules deployed as files
        # are not moved aside (_ota_shadow), so the old gps_logger.mpy
        # would run on the new image, pass the health check and never
        # update again
        ota["state"] = "not frozen"
        return
    f = ota.get("failed")
    if (f is not None and f[0] == build
            and _age_s(f[1]) < OTA_RETRY // 1000):
        ota["state"] = f[2]              # (2 MB every OTA_CHECK: no)
        return
    if not _ota_eq(_hmac_hex(OTA_KEY, _ota_signed(m)), m.get("hmac")):
        ota["state"] = "bad hmac"
        # (no unverified string in it: the build is an int)
        _ota_say("ota build %d: manifest hmac does not match" % build)
        return
    why = _ota_fresh(m)
    if why is not None:
        ota["state"] = why
        _ota_say("ota build %d: manifest %s" % (build, why))
        return
    b0 = _ota_seen()
    if ota.get("seen_err"):              # (unread: no replay let through)
        ota["state"] = "seen err"
        _ota_say("ota build %d: %s not read, refused" % (build, OTA_SEEN))
        return
    if build < b0:                       # (a newer one was signed since)
        ota["state"] = "superseded"
        _ota_say("ota build %d: older than build %d seen signed, refused" % (build, b0))
        return
    ota["said"] = None
    if _ota_held(build):                 # (OTA_TRIES_MAX tries: a while)
        ota["state"] = "rolled back"
        return
    try:
        part = esp32.Partition(esp32.Partition.RUNNING).get_next_update()
    except (OSError, AttributeError):
        ota["state"] = "no ota slot"
        return
    if m["size"] > part.info()[3]:
        ota["state"] = "too big"
        return
    # the pages are staged (.tmp, then .new beside the old one) after the
    # 2 MB download: a full flash failed there, halfway, every OTA_RETRY
    room = free()
    # (+3 blocks: LittleFS rounds each file up, and the note, the mark and
    # OTA_DL take theirs: at the bare sum the "installed" diag line fell
    # under MIN_FREE and was not written)
    need = MIN_FREE + 3 * 4096 + sum(f["size"] for f in m.get("files") or ())
    if room is not None and room < need:
        ota["failed"] = (build, _stamp(), "no flash for page")
        ota["state"] = "no flash for page"
        return
    _ota_seen_note(build)                # (only one it could install)
    _ota_dl_begin(build)
    ota["state"] = "downloading"
    w = _SlotWriter(part)
    try:
        code = HTTP_GET(_ota_url(m["url"]), w, m["size"])
        got = w.close()                  # (the last block's write and the
        ok = (code == 200 and got == m["sha256"]  # read-back: a flash
              and _slot_sha(part, m["size"]) == m["sha256"])  # error too)
    except Exception:
        # a download cut short waits OTA_RETRY like a bad one: it came
        # again (2 MB, the loop blocked) every OTA_CHECK (whatever it
        # raised: an empty status line, a MemoryError, the slot's flash)
        ota["failed"] = (build, _stamp(), "download err")
        _rm(OTA_DL)                      # (it ended: no reset cut it)
        raise
    if not ok:
        _rm(OTA_DL)
        ota["failed"] = (build, _stamp(), "bad image")
        ota["state"] = "bad image"
        _diag_write("ota %s: download %s, sha256 mismatch" % (m["commit"],
                                                             code))
        return
    paths = []
    why = "bad page"                     # (the step that failed, for the
    try:                                 # state while it waits OTA_RETRY)
        for f in m.get("files") or ():
            _ota_file(f)
            paths.append(f["path"])
        why = "note err"
        # a mark left over (a confirmation that failed before its health
        # reset, a power cut in _ota_done) must not speak for this build:
        # "valid" confirmed it at its first boot without a health check
        _rm(OTA_MARK)
        # "staged" until set_boot is through (it hashes the 2 MB image
        # first): a power cut before it is no try of a build that never
        # ran (ota_boot drops it; it was a try, two such a day's hold)
        with open(OTA_MARK, "w") as f:
            f.write("staged")
        with open(OTA_PENDING, "w") as f:
            f.write("%d %s\n%s" % (build, m["commit"], "\n".join(paths)))
        if FW_BUILD:
            why = "shadow err"           # (raises: the old module stayed)
            _ota_shadow(True)
        why = "set_boot err"
        part.set_boot()
        _rm(OTA_MARK)
        _rm(OTA_DL)                      # (the pending note takes over)
    except BaseException:
        # (a Ctrl-C too: it is no Exception) nothing boots: no pending note
        # (it would blame a build that never ran), no staged page, the
        # shadowing module back; a page that failed its sha256 or could not
        # be written waits OTA_RETRY too (the firmware came every check).
        # Unless set_boot took effect (a Ctrl-C right after it): reset
        if _ota_boot_slot() != part.info()[4]:
            _rm(OTA_PENDING)
            _rm(OTA_MARK)
            _rm(OTA_DL)
            _ota_drop(paths)
            _ota_shadow(False)
            ota["failed"] = (build, _stamp(), why)
            raise
    try:
        _diag_final()
        _diag_write("ota %s installed (was %s), rebooting" % (m["commit"],
                                                             FW_COMMIT))
        flush(_still_hold())
    except BaseException:
        # (a Ctrl-C too: otadata names the new slot, the reset must come;
        # skipped, the old firmware blamed the build and the next reset
        # booted it with no pending note)
        pass
    machine.reset()

def _ota_valid():
    """Confirm the running image (the bootloader's rollback then keeps it).
    -> None, or why it failed (repr). On a boot without a note that is
    nothing (a non-OTA or never-pending image raises: nothing to confirm);
    for a pending image it is: the confirmation steps ran over an image
    the next reset rolled back, "ota ok" in the diag."""
    try:
        esp32.Partition.mark_app_valid_cancel_rollback()
        return None
    except (OSError, AttributeError) as e:
        return repr(e)[:60]

def _ota_pending():
    """(build, commit, staged paths) of OTA_PENDING, None without one. A
    note that is there but cannot be read or parsed is (None, "?", []):
    an image may be pending, its build unknown (ota_boot drops it)."""
    try:
        with open(OTA_PENDING) as f:
            lines = f.read().split("\n")
        b, _, c = lines[0].strip().partition(" ")
        return int(b), c, [x for x in lines[1:] if x]
    except OSError as e:
        if _enoent(e):
            return None
    except (ValueError, IndexError):
        pass
    return None, "?", []

def ota_boot():
    """At boot: a firmware just installed (OTA_PENDING) runs pending until
    _ota_health; the old one running means the bootloader rolled back:
    bad after a crash (OTA_MARK) or a watchdog reset while the logger ran,
    a try otherwise (a power-off, a reset from outside, a panic). A boot
    slot set but not run yet (a Ctrl-C between set_boot and the reset) is
    booted, once. Without a pending note the running image is confirmed:
    an image left pending would be aborted at the next reset. The new build
    with OTA_MARK "valid" (a power cut while it was being confirmed) ends
    the confirmation at once, without the health checks: held pending, the
    staged page was dropped and a good image reset as a try."""
    p = _ota_pending()
    if p is None:
        _rm(OTA_MARK)                    # (no pending run it could be about)
        _ota_drop(OTA_PAGES)             # (a power cut while staging one:
                                         # ~26 KB left until the next install)
        _ota_wait_tried()
        _ota_cut_seen()
        if esp32 is not None:
            _ota_valid()
        return
    build, c, paths = p
    if build is None:
        # an unreadable note: whose build, and whether this is it or the
        # old one after a rollback, is not known. Held as the running
        # build's, the health check (the radio kept on, no memory reboot,
        # a reset away from WiFi, a try or a bad mark for a build that may
        # be the old good one) looped when the note could not be removed.
        # Dropped instead, with the staged pages (the OTA_PAGES it may
        # name), and the running image confirmed once: it ran this far.
        _ota_drop(OTA_PAGES)
        _rm(OTA_PENDING)
        _rm(OTA_MARK)
        _rm(OTA_DL)
        _ota_wait_tried()
        if esp32 is not None:
            _ota_valid()
        _diag_write("ota note unreadable, dropped (running %s)" % FW_COMMIT)
        return
    mark = _ota_mark_get()
    try:
        cause = machine.reset_cause()
    except Exception:
        cause = None
    if mark and mark.startswith("cause "):
        # gps_boot's note of this firmware's first boot after the rollback:
        # a reset of the old firmware before it got here (a Ctrl-C, a
        # stall and its watchdog) was taken for the new build's
        try:
            cause = int(mark[6:])
        except ValueError:
            pass
        mark = None
    if build == FW_BUILD:
        valid = mark == "valid"          # (a power cut while confirming)
        if not valid:
            _rm(OTA_MARK)                # (a run before this one stopped)
        _rm(OTA_DL)                      # (it booted: a reset after
                                         # set_boot left it, no cut)
        ota.update(pend=build, pend_at=time.ticks_ms(), wifi_seen=False,
                   confirmed=False, done_err=0,
                   probed=False, probe_at=None, probe_err=None,
                   state="pending", paths=paths, valid=valid,
                   valid_err=None)
        _diag_write("ota %s booted, %s" % (
            c, "confirming again" if valid else "checking"))
        if valid:                        # (no health check again: it
            try:                         # passed before the cut; a step
                _ota_health(time.ticks_ms())  # that fails: the next tick)
            except Exception as e:
                _bug_once("bug ota boot %r" % e, ("ota boot", type(e)))
        return
    if mark == "staged" and not (esp32 is not None and _ota_unbooted()):
        # the install cut before set_boot: the build never ran, no try
        _ota_drop(paths)
        _ota_shadow(False)
        _rm(OTA_PENDING)
        _rm(OTA_MARK)
        ota["state"] = "install cut"
        _diag_write("ota %s install cut before set_boot (a power-off): "
                    "dropped, no try (running %s)" % (c, FW_COMMIT))
        _ota_wait_tried()
        _ota_cut_seen()
        if esp32 is not None:
            _ota_valid()
        return
    if esp32 is not None and mark != "boot" and _ota_unbooted():
        # never run (blamed here, a build that never booted went bad):
        # boot it, once; a slot the bootloader still does not boot is a
        # try the next time
        _ota_mark("boot")
        _diag_write("ota %s set to boot, not run yet: rebooting into it" % c)
        machine.reset()
        return
    _ota_drop(paths)
    _ota_shadow(False)                   # (main.py did, before the import)
    _rm(OTA_PENDING)
    _rm(OTA_MARK)
    _rm(OTA_DL)                          # (it booted and rolled back: no cut;
                                         # left, the next boot counted one)
    ota["state"] = "rolled back"
    try:
        wdt = cause == machine.WDT_RESET
        power = cause == machine.PWRON_RESET
    except Exception:
        wdt, power = False, False
    if mark == "crash" or (wdt and mark != "stop"):
        _ota_mark_bad(build)
        _diag_write("ota %s %s, rolled back by the bootloader: bad (running "
                    "%s)" % (c, "crashed" if mark == "crash" else
                             "watchdog reset", FW_COMMIT))
    else:
        why = "a power-off" if power else "a reset (cause %s)" % cause
        n = _ota_try(build)
        _ota_wait_tried()
        if n % OTA_TRIES_MAX == 0:
            _diag_write("ota %s rolled back after %s, %d times: held %d h "
                        "(running %s)" % (c, why, n, _ota_hold_s(n) // 3600,
                                          FW_COMMIT))
        else:
            _diag_write("ota %s rolled back after %s, tried again in %d min "
                        "(running %s)" % (c, why, OTA_RETRY // 60000,
                                          FW_COMMIT))
    if esp32 is not None:
        _ota_valid()

def _ota_probe(now):
    """The pending firmware reads the manifest (HTTP 200, fields as
    _ota_check wants them, its HMAC checks: the key and the check work, and
    a manifest signed with another key does not make the build valid),
    every OTA_PROBE until it did: a build that cannot would never get the
    next one over WiFi, only over USB. None read in OTA_HEALTH is a try,
    like an outage (_ota_try): never blamed, a build whose update path is
    broken would be reinstalled for good."""
    at = ota.get("probe_at")
    if _recent(now, at, OTA_PROBE):
        return
    ota["probe_at"] = now
    body = bytearray()
    try:
        code = HTTP_GET(OTA_URL, body.extend, OTA_MANIFEST_MAX)
        err = "manifest %s" % code
        if code == 200:
            m = json.loads(body)
            if not _ota_manifest_ok(m):
                err = "manifest fields"
            elif not _ota_eq(_hmac_hex(OTA_KEY, _ota_signed(m)),
                             m.get("hmac")):
                err = "manifest hmac"
            else:
                err = None
        ota["probed"] = err is None
        ota["probe_err"] = err           # (None: an earlier one is no reason
                                         # once it was read)
    except Exception as e:
        ota["probe_err"] = repr(e)[:40]

def _ota_health(now):
    """The pending firmware: valid after OTA_VALID_AFTER with WiFi connected
    then (not only once before) and the manifest read; then the staged page
    goes up. Without OTA_URL it is not confirmed: only _ota_check writes
    the pending note, and it needs OTA_URL (deploy.sh does not run with the
    note there): a build that lost it (its .env reading) could be updated
    only over USB from then on. Neither in OTA_HEALTH and it resets, the bootloader then
    boots the old image (a try: _ota_try). OTA_MARK "valid" goes first: a
    power cut between the confirmation and the note's removal booted a
    valid image as pending again (ota_boot finishes it now). A confirmation
    that fails keeps the pending state, the notes and the staged page (one
    diag line), and is tried again each tick until the OTA_HEALTH reset:
    the steps ran over an unconfirmed image, "ota ok" in the diag, and the
    next reset rolled it back."""
    up = wifi["state"] == "connected"
    if up:
        ota["wifi_seen"] = True
        if not OTA_URL:
            ota["probe_err"] = "no OTA_URL"
        elif not ota.get("probed"):
            _ota_probe(now)
    age = _age(now, ota["pend_at"])
    # (WiFi now, not once: connected once and lost by a build that breaks
    # the reconnect was confirmed at OTA_VALID_AFTER without a link)
    if ota.get("valid") or (up and ota.get("probed")
                            and age >= OTA_VALID_AFTER):
        if not ota.get("valid"):
            _ota_mark("valid")
            ota["valid"] = True          # (written once, not every tick)
        err = _ota_valid()
        if err is None:
            # the bootloader keeps it now: no crash, stop or guard verdict
            # may speak against it (run()'s crash branch wrote "crash" over
            # "valid" in a MemoryError's pause of _ota_done, the next boot
            # ran the health check again and "rolled back" the same image)
            ota["confirmed"] = True
        if err is not None:
            if not ota.get("valid_err"):
                ota["valid_err"] = err
                _diag_write("ota %s not confirmed: %s" % (FW_COMMIT, err))
            if age < OTA_HEALTH:
                return
        else:
            _ota_done()
            return
    if age < OTA_HEALTH:
        return
    build = ota["pend"]
    ota["pend"] = None
    ota["valid"] = False
    try:                                 # (the reset whatever fails: the
        _ota_drop(ota.get("paths") or ())   # image ran on unconfirmed)
        _ota_shadow(False)
        # the note first: a power cut between the two then loses the try;
        # the other way round the old firmware found it and counted again
        _rm(OTA_PENDING)
        _rm(OTA_MARK)                    # ("valid" of a failed confirmation:
                                         # it spoke for the next build)
        n = _ota_try(build)
        _diag_final()
        what = "wifi"
        if ota.get("valid_err"):
            what = "confirmation (%s)" % ota["valid_err"]
        elif ota.get("probed"):
            what = "wifi (lost after the manifest)"
        elif ota["wifi_seen"]:
            what = "manifest (%s)" % ota.get("probe_err")
        _diag_write("ota %s no %s in %d s, rolling back%s" % (
            FW_COMMIT, what, OTA_HEALTH // 1000,
            ", held" if n % OTA_TRIES_MAX == 0 else ""))
        flush(_still_hold())             # (as before the other reboots)
    finally:
        machine.reset()

OTA_DONE_MAX = 5                         # _ota_done tries before it gives up

def _ota_done():
    """The confirmed image's steps: the staged page up, the moved-aside
    modules, the notes gone (OTA_MARK "valid" last of them), then the
    pending state ends."""
    _ota_promote(ota.get("paths") or ())
    # (before the note: gps_boot.unshadow puts back what has no note, an
    # .ota-old left by a failed remove went back over the new frozen
    # module while the diag said "ok"; one that fails raises: the guard
    # pauses, the next tick runs the steps again, the note still there)
    # A remove that keeps failing (a damaged block) kept the image pending
    # for good: the radio never slept, no memory reboot, no later update
    # could come. After OTA_DONE_MAX the pending state ends anyway (a diag
    # line names the error): the note left on flash keeps unshadow from
    # putting the .ota-old back, the next boot runs the steps again.
    try:
        for p in OTA_SHADOW:
            _rm_gone(p + ".ota-old")
        _rm_gone(OTA_PENDING)
    except OSError as e:
        n = ota["done_err"] = ota.get("done_err", 0) + 1
        if n < OTA_DONE_MAX:
            raise
        _diag_write("ota %s steps failed %d times (%r): pending ended, "
                    "the rest at the next boot" % (FW_COMMIT, n, e))
        ota["pend"] = None
        ota["valid"] = False
        ota["state"] = "ok (leftover)"
        return
    ota.pop("done_err", None)
    _rm(OTA_TRIES)
    _rm(OTA_DL)
    _rm(OTA_MARK)
    # the pending state ends last: a MemoryError in a step above (the
    # guard swallows it) leaves it set, and the next tick runs the steps
    # again (they are safe to), not a RAM state of "done" over a note,
    # staged pages and .ota-old still on flash
    ota["pend"] = None
    ota["valid"] = False
    ota["state"] = "ok"
    _diag_write("ota %s ok" % FW_COMMIT)

def ota_tick(now):
    if ota["pend"] is not None:
        _ota_health(now)
        return
    if not OTA_URL or esp32 is None or wifi["state"] != "connected":
        return
    # points waiting in files or an upload backing off: not now (the RAM
    # batch is flushed before the reset: a moving board kept one, and
    # _nothing_to_send() held the check back for good). Unless no 2xx came
    # for OTA_STUCK: "err payload" is cleared by a 2xx only, and an upload
    # bug held back the update that fixes it
    if not at_home():
        return
    # (not with no GEO_URL / API_KEY: "left" never falls there, and a
    # logging-only board got its update after OTA_STUCK, as "stuck")
    if up["state"] != "no .env" and (up["left"] or up["files"]
                                     or str(up["state"]).startswith("err")):
        ok = up["ok_at"]
        if ota.get("held_at") is None or ok != ota.get("held_ok"):
            ota["held_at"] = _stamp()
            ota["held_ok"] = ok
            ota["stuck"] = False
            return
        if _age_s(ota["held_at"]) < OTA_STUCK // 1000:
            return
        ota["held_at"] = _stamp() - OTA_STUCK // 1000  # (once a day)
        if not ota.get("stuck"):
            ota["stuck"] = True
            _diag_write("ota: uploads stuck %d h (%s), checking anyway" % (
                OTA_STUCK // 3600000, up["state"]))
    elif ota.get("held_at") is not None:    # (no dict per loop pass)
        ota["held_at"] = None
        ota["stuck"] = False
    at = ota["at"]
    if _age_s(ota["boot_at"] if at is None else at) < (
            OTA_FIRST if at is None else OTA_CHECK) // 1000:
        return
    ota["at"] = _stamp()
    try:
        _ota_check()
    except Exception as e:               # a network or flash error: the next
        ota["state"] = "err"             # check is OTA_CHECK later
        print("ota", repr(e))
        err = repr(e)[:60]
        if err != ota.get("last_err"):   # (one line per kind: a server down
            ota["last_err"] = err        # every 10 min filled the diag log)
            _diag_write("ota error %s" % err)
        return
    ota["last_err"] = None

# --- Status page ------------------------------------------------------------
# A small HTTP page on the LAN while the radio is connected at home, or on
# any network with WEB_AWAY (it stays connected for it, see wifi_tick):
# http://<ip>/ and /status.json.
# Served from the main loop, one client at a time: each recv/send waits at
# most WEB_TIMEOUT s and the whole request at most WEB_DEADLINE s (a slow
# client used to hold the loop for ~50 s: 2 s per 1 KB send).

_web_sock = None
_web_poll = None                         # its poll, None: accept() asks itself

def _web_env(env):
    """WEB=0 in .env: no status page, the radio sleeps at home as before.
    WEB_AWAY=1: the radio stays on for the page on a non-home network too
    (with a WEB_TOKEN only: without one it is refused).
    WEB_TOKEN: the position, the track and the diag go only to a request
    with it as its X-GPS header (the page sends what #k= in its URL gave);
    WEB_TOKEN_LEN characters of letters, digits and ._~+/=- (what a URL
    fragment carries as it is). One that is not: refused, and every data
    route then too (a token taken as "no token" left them open while the
    owner thought them closed)."""
    global WEB, WEB_AWAY, WEB_TOKEN
    v = env.get("WEB_TOKEN")
    if isinstance(v, str) and v.strip():
        v = v.strip()
        if (WEB_TOKEN_LEN[0] <= len(v) <= WEB_TOKEN_LEN[1]
                and all(ch.isalpha() and ord(ch) < 128 or ch.isdigit()
                        or ch in "._~+/=-" for ch in v)):
            WEB_TOKEN = v
            _web_tok_bad[0] = False
        else:
            WEB_TOKEN = None
            _web_tok_bad[0] = True
            _env_refused("WEB_TOKEN", "not %d-%d of A-Za-z0-9._~+/=-"
                         % WEB_TOKEN_LEN, "every data route refused")
    elif _env_partial[0] or "WEB_TOKEN" in _env_skipped:
        # (it may be in the part not read; or its value is not UTF-8)
        WEB_TOKEN = None
        _web_tok_bad[0] = True
        _env_refused("WEB_TOKEN", "not read (.env read part way, or its line garbled)",
                     "every data route refused")
    v = _plain(env.get("WEB"))
    if v:
        t = _truthy(v)
        if t is None:
            _env_refused("WEB", "not 1/0", 1 if WEB else 0)
        else:
            WEB = t
    v = _plain(env.get("WEB_AWAY"))
    if v:
        t = _truthy(v)
        if t is None:
            _env_refused("WEB_AWAY", "not 1/0", 1 if WEB_AWAY else 0)
        elif t and WEB_TOKEN is None:    # (a foreign network: the position
            WEB_AWAY = False             # and the track to whoever is on it)
            _env_refused("WEB_AWAY", "1 without WEB_TOKEN", 0)
        else:
            WEB_AWAY = t

def _web_poller(s):
    """A poll on the listening socket: ipoll(0) with nobody waiting allocates
    nothing, an accept() raises EAGAIN (~96 B, ~40 times a second). None where
    it does not work (CPython's poll has no ipoll): accept() as before."""
    try:
        p = select.poll()
        p.ipoll
        p.register(s, select.POLLIN)
        return p
    except (AttributeError, TypeError, ValueError, OSError):
        return None

WEB_RETRY = 5000                         # ms between listen tries that fail
_web_fail_at = [None]                    # ticks of the last failed one

def _web_open():
    global _web_sock, _web_poll
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(socket.getaddrinfo("0.0.0.0", WEB_PORT)[0][-1])
        s.listen(2)
        s.setblocking(False)
        _web_poll = _web_poller(s)
        _web_sock = s
        _web_acc_err[0] = 0              # (an old socket's errors: not this one's)
        print("web http://%s/" % wifi["ip"])
    except (OSError, MemoryError) as e:  # (no half-open socket left behind)
        if s is not None and _web_sock is not s:
            try:
                s.close()
            except OSError:
                pass
        if isinstance(e, MemoryError):
            raise                        # _guarded: a pause, not 40 tries a s
        print("web", repr(e))
        _bug_once("web error listen %s" % repr(e)[:60], ("web listen", type(e)))

def _web_close():
    global _web_sock, _web_poll
    _web_poll = None
    if _web_sock is not None:
        try:
            _web_sock.close()
        except OSError:
            pass
        _web_sock = None

def _web_diag():
    """The last WEB_DIAG diag lines; just after a rotation the rest from
    DIAG_OLD (the new file holds one line then)."""
    lines = _diag_tail(DIAG_FILE, WEB_DIAG)
    if len(lines) < WEB_DIAG:
        lines = _diag_tail(DIAG_OLD, WEB_DIAG - len(lines)) + lines
    return lines

def _prev_text():
    if prev_run["up"] is None:
        return None
    if prev_run["how"] == "short":
        return "under %d s" % (DIAG // 1000)
    t = _hms(prev_run["up"])
    if prev_run["how"] == "crash":
        return t + ", ended by a crash"
    return t + (" (last line %s)" % prev_run["end"] if prev_run["end"] else "")

def _web_rows():
    """(key, value) pairs of the JSON (web.html labels them)."""
    now = time.ticks_ms()
    view = _rcv.sv_seen()
    t = _fix_s()
    pos = pos6 = None
    # a silent receiver keeps its last second's valid flag: no position
    # once that second is older than NO_DATA (the page would show it live)
    pa = st.get("pos_at")
    if (st["valid"] and st["lat"] is not None and st["lon"] is not None
            and pa is not None and _age(now, pa) <= _rcv.NO_DATA):
        pos = "%.5f, %.5f" % (st["lat"], st["lon"])
        # the track line's precision for the page's static measurement
        pos6 = "%.6f, %.6f" % (st["lat"], st["lon"])
    try:
        cause = machine.reset_cause()
    except Exception:
        cause = None
    flash = _flash_kb()
    files, bad = _file_counts()
    heap = py_heap()
    still = log["still"]
    vb, vw = _vbat()
    vl = _vbat_left()
    return [
        ("fix", status()),
        ("sats", "%d / %s" % (view, _rcv.sv_used())),
        ("hdop", st["hdop"]),
        ("std", st.get("std")),          # (GST: the receiver's own error, m)
        # (as the position: a silent receiver's last second is not shown
        # as live: "GPS time 10:00:00 UTC" under the GPS SILENT banner)
        ("utc", None if t is None or st["rx"] is None
         or _age(now, st["rx"]) > _rcv.NO_DATA else _stamp_s(t)[0]),
        ("position", pos),
        ("pos6", pos6),
        ("nmea", "%d / %d" % (st["ok"], st["err"])),
        ("rx_bytes", st["rx_b"]),
        ("rx_age_s", None if st["rx"] is None else _age(now, st["rx"]) // 1000),
        ("good_age_s", None if st["good"] is None
         else _age(now, st["good"]) // 1000),
        ("gps_boots", st["gps_boots"]),
        ("gps_loops", st["loops"]),
        ("receiver", st["gps_ver"]),
        ("setup_ok", st["setup_ok"]),
        ("points", log["points"]),
        ("left", up["left"]),
        ("files", files),
        ("bad_files", bad),
        ("err_files", up["err_files"]),  # (parked unreadable / undeletable)
        ("sent", up["sent"]),
        ("upload", up["state"]),
        ("tls", _tls_state()),           # (https: is the server verified?)
        ("upload_err", up["err"]),
        ("upload_age_s", None if up["ok_at"] is None
         else _age_s(up["ok_at"])),
        ("days", _hist_days()),
        ("day_span", _hist_spans()),
        ("stay", "yes" if still.get("active") else "no"),
        ("uptime", uptime_s()),
        ("prev_uptime", prev_run["up"]),
        ("prev_run", _prev_text()),
        ("prev_end", None if prev_run["end"] is None
         else prev_run["end"].replace(" ", "T")),
        ("boot_cause", cause),
        ("temp", _temp()),
        ("vbat", vb),                    # (V; the charger's without one)
        ("vbat_swing", vw),
        ("vbat_pct", _vbat_now()),
        ("vbat_left_s", vl),             # (about, from the last 30 min)
        ("vbat_err", _vb_err[0]),        # (ADC reads that failed)
        ("battery", _battery(vb, vw, vl)),
        ("mhz", machine.freq() // 1000000),
        ("mem", heap[0]),
        ("heap_kb", heap[1]),
        ("idf", "%s / %s" % idf_heap()),
        ("flash_kb", flash),
        ("wifi", "%s %s" % (wifi["state"], wifi["ssid"] or "")),
        ("rssi", wifi["rssi"]),
        ("pulses", wifi["pb_scans"]),
        ("fw", FW_COMMIT),
        ("fw_build", FW_BUILD),          # (the commit time, s since 1970)
        ("ota", ota["state"]),
        # a pending build: since when, and what holds its confirmation (the
        # reason showed only after the rollback, in the old firmware's diag)
        ("ota_pend", None if ota.get("pend") is None else
         [_age(now, ota["pend_at"]) // 1000, ota.get("probe_err"),
          ota.get("valid_err")]),
        ("web_errs", _web_errs[0]),      # page requests that hit a bug
        ("tick_bugs", _tick_bugs()),     # guarded ticks' bugs: count, last
        ("diag_errs", _diag_errs[0]),    # diag lines that failed to write
        ("hist_errs", log["herr"]),      # day history write errors
        ("track_drop", log["drop"]),   # points lost (full flash,
        ("write_errs", log["err"]),    # MAX_BATCH, time back), track
        ("q_thin", st.get("q_thin", 0)),      # write errors, seconds thinned
    ]                                    # while the loop was blocked

_vb = [None, None, None, None, None, 0]  # (V, spread V, Stamp, ADC, battery or
                                         # not, contrary measurements in a row)
_vb_err = [0]                            # ADC errors since boot (status.json)
_vb_left = [None]                        # _vbat_fit of the last sample

def _vbat():
    """The battery: (median V, 10th-90th percentile spread V) of VBAT_N
    reads, at most every 30 s (the page asks every WEB_REFRESH, the diag
    line every DIAG, the display every TFT_STATE). (None, None) without an
    ADC (a host's tests) or after a failed read: counted (vbat_err), a diag
    line a kind, the ADC made again next time. A measurement that turns the
    verdict (battery: spread up to VBAT_SWING, or none) counts only as the
    VBAT_AGAIN-th in a row: one stray one keeps the values before."""
    if _recent_s(_vb[2], 30):
        return _vb[0], _vb[1]
    _vb[2] = _stamp()
    adc = getattr(machine, "ADC", None)
    if adc is None:
        _vb[0] = _vb[1] = None
        return None, None
    try:
        if _vb[3] is None:
            _vb[3] = adc(Pin(HELTEC_VBAT), atten=adc.ATTN_2_5DB)
        r = sorted(_vb[3].read_uv() for _ in range(VBAT_N))
    except (OSError, ValueError) as e:
        _vb_err[0] += 1
        _vb[0] = _vb[1] = _vb[3] = None
        _bug_once("bug vbat %s" % repr(e)[:60], ("vbat", type(e)))
        return None, None
    k = VBAT_N // 10
    v = round(r[VBAT_N // 2] * VBAT_DIV / 1e6, 2)
    w = round((r[-1 - k] - r[k]) * VBAT_DIV / 1e6, 2)
    ok = w <= VBAT_SWING
    if _vb[4] is None or ok == _vb[4] or _vb[5] + 1 >= VBAT_AGAIN or _vb[0] is None:
        _vb[0], _vb[1], _vb[4], _vb[5] = v, w, ok, 0
    else:
        _vb[5] += 1
    return _vb[0], _vb[1]

def _vbat_pct(v):
    """What is left of the battery at v V, 0-100 % (VBAT_OCV)."""
    o = VBAT_OCV
    if v >= o[0]:
        return 100
    for i in range(1, len(o)):
        if v >= o[i]:
            return round(100 - 10 * (i - (v - o[i]) / (o[i - 1] - o[i])))
    return 0

def _vbat_v(p):
    """The voltage VBAT_OCV gives p % (_vbat_pct's inverse, linear between)."""
    o = VBAT_OCV
    if p >= 100:
        return o[0]
    if p <= 0:
        return o[-1]
    i = int((100 - p) // 10)
    return o[i] + (o[i + 1] - o[i]) * (100 - 10 * i - p) / 10.0

def _vbat_mv(p):
    """_vbat_v in whole mV, what a jump is measured in: VBAT_OCV's points are
    whole mV apart, so a whole per cent is a whole mV, and the device's
    single precision compares as the tests' double."""
    return int(_vbat_v(p) * 1000 + 0.5)

_vb_hist = []                            # [(uptime s, %)] on the battery

def _vb_ref():
    """What a sample is measured against for a jump: the median of the last
    3 kept (the last one alone: a noisy one made the normal ones after it
    look like jumps, 6 resets in 6 h at 8 mV of noise)."""
    h = _vb_hist
    if len(h) < 3:
        return h[-1][1]
    a, b, c = h[-3][1], h[-2][1], h[-1][1]
    return max(min(a, b), min(max(a, b), c))

_vb_rise = [None]                        # uptime s of the charger's last lift
_vb_jump = [0, None]                     # samples in a row that jumped (over
                                         # VBAT_JUMP_MV off _vb_ref), the last
                                         # one's uptime
def vbat_tick(now):
    """Every VBAT_EVERY s a sample of what is left, and _vbat_fit once (the
    display asks every TFT_STATE). No battery starts the history again, so
    does a jump of more than VBAT_JUMP_MV (against the median of the
    last 3 kept, _vb_ref, both back in mV: _vbat_mv) that holds VBAT_AGAIN
    samples in a row: put on the charger (it lifts the voltage) or taken off
    it (the lift gone at once: the samples on the charger made a steep
    line, half the time left for 25 min). One sample off alone is let go:
    a noisy reading started the samples again
    (simulated: at 8 mV of noise the time left was known a quarter of the
    time)."""
    t = uptime_s(now)
    last = _vb_jump[1] if _vb_jump[1] is not None else (
        _vb_hist[-1][0] if _vb_hist else None)
    if last is not None and t - last < VBAT_EVERY:
        return
    p = _vbat_now()
    if p is None:
        del _vb_hist[:]
        _vb_jump[0], _vb_jump[1] = 0, None
        _vb_left[0] = None
        _vb_rise[0] = None
        return
    if p < 99 and _vb_hist and _vb_hist[-1][1] >= 99:
        # off the top: everything from VBAT_OCV[0] up reads 100 %, the
        # charger's plateau and a full cell just off it alike, so the unplug
        # can be no step at all; the plateau kept made the time left a day
        # for 20 min (and a quarter of the true one on the old curve)
        del _vb_hist[:]
        _vb_jump[0], _vb_jump[1] = 0, None
    r = _vb_ref() if _vb_hist else None
    if r is not None and abs(_vbat_mv(p) - _vbat_mv(r)) > VBAT_JUMP_MV:
        _vb_jump[0] += 1
        _vb_jump[1] = t
        if _vb_jump[0] < VBAT_AGAIN:
            return
        del _vb_hist[:]
        if p > r:
            _vb_rise[0] = t              # put on the charger
    _vb_jump[0], _vb_jump[1] = 0, None
    _vb_hist.append((t, p))
    if len(_vb_hist) > VBAT_HIST:
        del _vb_hist[0]
    h, k = _vb_hist, VBAT_RISE_N
    if len(h) >= 2 * k and (sum(x for _, x in h[-k:])
                            - sum(x for _, x in h[-2 * k:-k])) >= VBAT_RISE * k:
        _vb_rise[0] = t                  # topped up, under VBAT_JUMP_MV
    _vb_left[0] = _vbat_fit()

def _vbat_left():
    """About how long the battery lasts, s (_vbat_fit at the last sample),
    or None."""
    return _vb_left[0]

def _vbat_fit():
    """The least-squares slope of the samples (VBAT_HIST, the last 30 min)
    carried down to 0 %, s, a minute at least. None until they span
    VBAT_SPAN and the fit falls VBAT_DROP points over them (on the charger,
    or too short to tell), at 99 % and up (the charger's lift), and for
    VBAT_RISE_HOLD after the charger lifted it (_vb_rise: on USB a top-up
    94 -> 99 % fell back to 96 % over 30 min, read as ~15 h left while it
    drew 0.13 % an hour, 2026-11-23)."""
    h = _vb_hist
    n = len(h)
    if n < 2 or h[-1][0] - h[0][0] < VBAT_SPAN or h[-1][1] >= 99:
        return None
    if _vb_rise[0] is not None and h[-1][0] - _vb_rise[0] < VBAT_RISE_HOLD:
        return None
    mt = sum(t for t, _ in h) / n
    mp = sum(p for _, p in h) / n
    sxx = sum((t - mt) ** 2 for t, _ in h)
    k = sum((t - mt) * (p - mp) for t, p in h) / sxx   # %/s
    if -k * (h[-1][0] - h[0][0]) < VBAT_DROP:
        return None
    return max(60, int(max(0, mp + k * (h[-1][0] - mt)) / -k))

def _left_text(secs):
    """'~45 min', '~2 h 10 min' (to 10 min from an hour), no-break spaces
    (a phone broke the line inside it)."""
    m = (secs + 30) // 60
    if m < 60:
        return "~%d\u00a0min" % m
    m = (m + 5) // 10 * 10
    return "~%d\u00a0h\u00a0%02d\u00a0min" % (m // 60, m % 60)

def _vbat_now():
    """What is left of the battery, %, or None without one (or an ADC)."""
    v, w = _vbat()
    return None if v is None or w is None or w > VBAT_SWING else _vbat_pct(v)

def _tft_bat():
    """The display's battery: (per cent, about how long it lasts: "2 óra"
    (rounded), "45 p" under an hour, None until _vbat_left knows), or None
    without one."""
    p = _vbat_now()
    if p is None:
        return None
    s = _vbat_left()
    if s is not None:
        s = "%d óra" % ((s + 1800) // 3600) if s >= 3570 else "%d p" % max(1, (s + 30) // 60)
    return p, s

def _battery(v, w, left=None):
    """The page's text: "76 % (3.95 V)", with the time left "76 % (3.95
    V), ~2 h 10 min left", "none, on USB" (the charger's swing without a
    battery), None without a measurement."""
    return None if v is None or w is None else (
        "none, on USB" if w > VBAT_SWING
        else "%d %% (%.2f V)%s" % (_vbat_pct(v), v, "" if left is None
                                   else ", %s left" % _left_text(left)))

_fl = [None, None]                       # (KB, Stamp) of the last statvfs

def _flash_kb():
    """Free flash in KB, at most every 30 s: statvfs walks the whole
    LittleFS (slow, and the page asks every WEB_REFRESH)."""
    if _recent_s(_fl[1], 30):
        return _fl[0]
    try:
        v = os.statvfs("/")
        _fl[0] = v[0] * v[3] // 1024
    except OSError:
        _fl[0] = None
    _fl[1] = _stamp()
    return _fl[0]

_hd = [[], None]                         # (day history days, Stamp)

def _hist_days():
    """The UTC days (YYYYMMDD) of the day history, oldest first, a day
    once (.trb, or an older firmware's .trk), listed at most every 30 s
    (the page picks a day from them)."""
    if _recent_s(_hd[1], 30):
        return _hd[0]
    try:
        _hd[0] = sorted(set(n[:8] for n in _hist_names()))
    except OSError:
        _hd[0] = []
    _hd[1] = _stamp()
    return _hd[0]

_fc = [0, 0, None]                       # (files, bad files, Stamp)

def _file_counts():
    """(track files waiting, .bad files) for the page, counted at most every
    10 s: pending() and pending_bad() list, sort and build the paths of the
    whole log directory, twice per poll. ilistdir counts without a list."""
    if _recent_s(_fc[2], 10):
        return _fc[0], _fc[1]
    n = bad = 0
    cur = log["file"]
    try:
        if hasattr(os, "ilistdir"):
            names = (e[0] for e in os.ilistdir(LOG_DIR))
        else:                            # CPython (tests)
            names = iter(os.listdir(LOG_DIR))
        for name in names:
            if name.endswith(_TRACKS):
                if cur is None or cur != LOG_DIR + "/" + name:
                    n += 1
            elif name.endswith(_BADS):
                bad += 1
    except OSError as e:                 # (no directory: none waiting; another
        if _enoent(e):                   # error unknown, not a partial count
            n = bad = 0                  # as a real one: the page shows "-")
        else:
            n = bad = None
            _bug_once("track dir error %r" % e, ("log dir", type(e)))
    _fc[0], _fc[1], _fc[2] = n, bad, _stamp()
    return n, bad

def _sv_list():
    """The satellite list for the page, used first, then by signal; none
    once it is two SV_POLL old (as sv_used: a silent receiver's last list
    showed as live)."""
    at = st["svs_at"]
    if (st["svs"] is None or at is None
            or _age(time.ticks_ms(), at) >= 2 * _rcv.SV_POLL):
        return []
    return sorted(st["svs"], key=lambda v: (not v[4], -v[1], v[6] or 0,
                                            v[0]))

def _web_json_parts(rows, svs, diag=False):
    """The JSON in small pieces (no 8 KB string and its encoded copy).
    rows, svs: _web_rows() and _sv_list(), built by _web_status before
    the 200 (a bug in one is a 500, not a cut JSON); the rows go first
    and are let go before the diag lines are read. The last WEB_DIAG diag
    lines only with diag (/status.json?diag): the page reads the whole
    log from /diag, and every poll read the tail for nothing."""
    yield '{"device":' + json.dumps(DEVICE_ID) + ',"refresh_s":%d' % (
        WEB_REFRESH)
    for key, v in rows:
        if isinstance(v, float) and not math.isfinite(v):
            v = None                     # (json.dumps writes inf, nan: no JSON)
        yield ",%s:%s" % (json.dumps(key), json.dumps(v))
    rows = None
    # after the 200: an error ends the list instead of the JSON (a cut one
    # was OFFLINE on the page), counted as a web bug; no RAM is no bug
    # (web_tick: the client dropped)
    if diag:
        yield ',"diag":['
        sep = ""
        try:
            for line in _web_diag():     # one piece a line (no ~3 KB string)
                yield sep + json.dumps(line)
                sep = ","
        except MemoryError:
            raise
        except Exception as e:
            _web_bug(e)
        yield ']'
    yield ',"satellites":['
    sep = ""
    try:
        for v in svs:
            yield sep + json.dumps({
                "name": "%s %d" % (_rcv.GNSS.get(v[6], "SV"), v[0]), "system": _rcv.GNSS.get(v[6]), "svid": v[0],
                "cno": v[1], "elev": v[2], "azim": v[3], "used": v[4],
                "unhealthy": v[5], "l1": v[7], "l5": v[8]})
            sep = ","
        at = st["svs_at"]
        age = json.dumps(None if at is None
                         else _age(time.ticks_ms(), at) // 1000)
    except MemoryError:
        raise
    except Exception as e:
        _web_bug(e)
        age = "null"
    yield '],"satellites_age_s":' + age + "}"

_web_end = [None]                        # ticks_ms deadline of the request

def _web_left(c):
    """Before each recv/send: past the request's WEB_DEADLINE raise
    ETIMEDOUT (the client is dropped), else cut the socket timeout to what
    is left of it, at most WEB_TIMEOUT. (No deadline outside web_tick.)"""
    if _web_end[0] is None:
        c.settimeout(WEB_TIMEOUT)
        return
    left = time.ticks_diff(_web_end[0], time.ticks_ms())
    if left <= 0:
        raise OSError(110)               # ETIMEDOUT
    c.settimeout(min(left, WEB_TIMEOUT * 1000) / 1000)

def _web_out(c, b):
    _web_left(c)
    c.sendall(b)

def _web_head_len(d):
    """The length of the request head in d: up to its blank line (CRLF or
    bare LF), or an HTTP/0.9 request line alone ("GET /path"); -1 while it
    is not all there."""
    n = -1
    for end in (b"\r\n\r\n", b"\n\n"):
        i = d.find(end)
        if i >= 0 and (n < 0 or i + len(end) < n):
            n = i + len(end)
    if n < 0:
        i = d.find(b"\n")
        if i >= 0 and d.find(b" HTTP/", 0, i) < 0:
            n = i + 1
    return n

def _web_read(c):
    """The request head: recv until the blank line, at most WEB_REQ_MAX
    bytes. One recv(512) left the rest of a long request (cookies) unread,
    and closing a socket with unread data sends an RST that cut the reply
    short. A bare-LF head and an HTTP/0.9 line end it too (they waited for
    WEB_HEAD_WAIT and got nothing)."""
    data = b""
    end = _web_end[0]
    if end is not None:
        # a client dribbling its head (or an idle preconnect) held the loop
        # for WEB_DEADLINE: longer than the UART buffer lasts
        head = time.ticks_add(time.ticks_ms(), WEB_HEAD_WAIT * 1000)
        if time.ticks_diff(head, end) < 0:
            _web_end[0] = head
    try:
        while len(data) < WEB_REQ_MAX and _web_head_len(data) < 0:
            _web_left(c)
            b = c.recv(512)
            if not b:
                break
            data += b
            _rcv.read_uart()
    finally:
        _web_end[0] = end
    return data

def _web_host_ok(req):
    """DNS rebinding guard: a page of another site that got its name
    pointed at the board sends that name as Host. Allowed: no Host, the
    board's IP, HOSTNAME or HOSTNAME.local (any case, with a port). The
    header's name in any case, in a bare-LF head too, searched over the
    whole head; a head cut at WEB_REQ_MAX without a Host is refused (its
    Host could come after the cut: a long URL hid it)."""
    n = _web_head_len(req)
    r = req.lower()
    i = r.find(b"\nhost:", 0, n if n >= 0 else len(r))
    if i < 0:
        return n >= 0 or len(req) < WEB_REQ_MAX   # (closed: all there)
    j = r.find(b"\n", i + 6)
    if j < 0:
        return False                     # its line cut off
    return _web_name_ok(r[i + 6:j].strip())

def _web_hdr(req, name):
    """The value (bytes, stripped) of header name (lower case, no colon) in
    the request head, None when it has none."""
    n = _web_head_len(req)
    r = req[:n] if n >= 0 else req
    i = r.lower().find(b"\n" + name + b":")
    if i < 0:
        return None
    j = r.find(b"\n", i + 1)
    if j < 0:
        return None                      # its line cut off
    return r[i + len(name) + 2:j].strip()

def _web_name_ok(h):
    """h (bytes, lower case, a host[:port] or [v6]:port) is the board: its
    IP, HOSTNAME or HOSTNAME.local, a trailing dot allowed."""
    k = h.rfind(b":")
    if k >= 0 and b"]" not in h[k:]:
        h = h[:k]
    try:
        h = h.decode()
    except UnicodeError:
        return False
    if h.endswith("."):
        h = h[:-1]
    name = HOSTNAME.lower()
    return h in (wifi["ip"] or "", name, name + ".local")

def _web_same_page(req):
    """A Referer of the board's own page: an older page (one open across an
    OTA, or kept when the new one could not go up) sends no X-GPS. A page
    of another site sends its own Referer, or none (no-referrer)."""
    r = _web_hdr(req, b"referer")
    if not r:
        return False
    r = r.lower()
    i = r.find(b"://")
    if i < 0:
        return False
    h = r[i + 3:]
    j = h.find(b"/")
    return _web_name_ok(h if j < 0 else h[:j])

def _web_key_ok(req, need):
    """The data routes: with WEB_TOKEN the X-GPS header must be it (in a
    time independent of where it differs); a WEB_TOKEN refused: never.
    Without one, need: an X-GPS header, or a Referer of the board's own
    page (/hist, /diag: a page of another site cannot send the header
    without a preflight the board never answers, so its no-cors loop of
    decodes kept the board busy)."""
    if _web_tok_bad[0]:
        return False
    v = _web_hdr(req, b"x-gps")
    if WEB_TOKEN:
        if v is None:
            return False
        try:
            v = v.decode()
        except UnicodeError:
            return False
        return _ota_eq(v, WEB_TOKEN)
    return v is not None or not need or _web_same_page(req)

def _web_head(c, code, ctype, extra=""):
    _web_out(c, ("HTTP/1.0 %s\r\nContent-Type: %s; charset=utf-8\r\n%s"
                 "Cache-Control: no-store\r\nConnection: close\r\n\r\n" % (
                     code, ctype, extra)).encode())

def _web_send(c, code, ctype, parts):
    """A generated reply, piece by piece: never the whole body in RAM.
    Without a Content-Length the closed connection ends it (HTTP/1.0).
    ~1 KB per send, gathered in one fixed WEB_CHUNK buffer: one send per
    small piece waited for an ACK each (measured: 7 KB took 5 s and the
    next requests timed out). A piece longer than WEB_CHUNK goes out in
    WEB_CHUNK slices of itself, not copied."""
    _web_head(c, code, ctype)
    buf = bytearray(WEB_CHUNK)
    mv = memoryview(buf)
    n = 0
    for p in parts:
        b = p.encode()
        k = len(b)
        if n and n + k > WEB_CHUNK:
            _web_out(c, mv[:n])
            n = 0
            _rcv.read_uart()                  # a slow client costs no NMEA
        if k > WEB_CHUNK:
            bv = memoryview(b)
            for i in range(0, k, WEB_CHUNK):
                _web_out(c, bv[i:i + WEB_CHUNK])
                _rcv.read_uart()
            continue
        mv[n:n + k] = b
        n += k
    if n:
        _web_out(c, mv[:n])

def _web_gzip(req):
    """True if the request's Accept-Encoding names gzip (every browser;
    curl only with --compressed), in a bare-LF head too (as _web_host_ok
    reads it); a gzip token with q=0 refuses it."""
    r = req.lower()
    i = r.find(b"\naccept-encoding:")
    if i < 0:
        return False
    j = r.find(b"\n", i + 17)
    for tok in r[i + 17:j if j >= 0 else len(r)].split(b","):
        tok = tok.strip()
        if tok[:4] == b"gzip":
            q = tok.replace(b" ", b"").partition(b";q=")[2]
            return not q or q.strip(b"0.") != b""
    return False

def _web_page(c, gz=False):
    """The status page from flash in WEB_CHUNK reads into one buffer. The
    board keeps it only gzipped (WEB_PAGE_GZ, ~26 KB instead of ~70 KB:
    deploy.sh and the OTA put no plain copy up): sent as is, with its
    Content-Length, to a client that accepts gzip (every browser),
    inflated on the fly for one that does not (curl without --compressed:
    _web_inflate). A plain WEB_PAGE still there (a board deployed before,
    until its next deploy or OTA removes it) goes as is to a client
    without gzip (its gzip may have gzip -9's 32 KB window, too big to
    inflate here), and to one with gzip when there is no gzip. The page
    itself polls /status.json and redraws its boxes."""
    f = None
    for path in (WEB_PAGE_GZ, WEB_PAGE) if gz else (WEB_PAGE, WEB_PAGE_GZ):
        try:
            f = open(path, "rb")
            break
        except OSError:
            pass
    if f is None:
        _web_send(c, "404 Not Found", "text/plain",
                  ("no %s on the board: run deploy.sh; data at "
                   "/status.json" % WEB_PAGE_GZ,))
        return
    try:
        if path == WEB_PAGE_GZ and not gz:
            _web_inflate(c, f)
            return
        _web_head(c, "200 OK", "text/html",
                  "Content-Length: %d\r\n%s" % (
                      os.stat(path)[6], "Content-Encoding: gzip\r\n"
                      if path == WEB_PAGE_GZ else ""))
        buf = bytearray(WEB_CHUNK)
        mv = memoryview(buf)
        while True:
            n = f.readinto(buf)
            if not n:
                break
            _web_out(c, mv[:n])
            _rcv.read_uart()
    finally:
        f.close()

def _web_inflate(c, f):
    """WEB_PAGE_GZ (open in f) inflated for a client without gzip:
    deflate.DeflateIO with a 2**WEB_GZ_WBITS window, the RAM it takes (4 KB
    and ~1 KB of state: gzip -9's 32 KB window would not fit the ~15 KB of
    free heap, so deploy.sh and build_firmware.sh make the file with this
    one; a wider one fails part way). WEB_CHUNK at a time into one buffer,
    a send each, under the request's deadline like the plain page. No
    Content-Length (the inflated size is not on flash): the closed
    connection ends it (HTTP/1.0). The first piece is inflated before the
    200 (uzlib allocates its window then): no RAM for it is a 503, a
    firmware without the deflate module (MICROPY_PY_DEFLATE, on in the
    ESP32 ports) a 406."""
    try:
        import deflate
    except ImportError:
        _web_send(c, "406 Not Acceptable", "text/plain",
                  ("the page is kept gzipped and this firmware cannot "
                   "inflate it: use a browser or curl --compressed",))
        return
    buf = bytearray(WEB_CHUNK)
    mv = memoryview(buf)
    try:
        d = deflate.DeflateIO(f, deflate.GZIP, WEB_GZ_WBITS)
        n = d.readinto(buf)
    except MemoryError:
        d = buf = mv = None              # (for the reply's own buffer)
        gc.collect()
        _web_send(c, "503 Service Unavailable", "text/plain",
                  ("no RAM to inflate the page now: try again, or use "
                   "curl --compressed",))
        return
    _web_head(c, "200 OK", "text/html")
    while n:
        _web_out(c, mv[:n])
        _rcv.read_uart()
        n = d.readinto(buf)

def _web_reply(c, req):
    """Only a GET is answered: an empty or cut-off request (a closed tab, a
    port scan) used to fall back to "/" and get the whole page."""
    if not req.startswith(b"GET "):
        return
    if not _web_host_ok(req):
        _web_send(c, "403 Forbidden", "text/plain",
                  ("open http://%s/ (or %s.local)" % (
                      wifi["ip"] or "<board ip>", HOSTNAME),))
        return
    parts = req.split(None, 2)          # (HTTP/0.9: "GET /\r\n")
    if len(parts) < 2:
        return                           # "GET " and no path: cut off
    path = parts[1]
    parts = None
    data = path.startswith((b"/status.json", b"/hist?", b"/diag?"))
    if data and not _web_key_ok(req, not path.startswith(b"/status.json")):
        # (what to do first: the page shows the first 60 characters)
        _web_send(c, "403 Forbidden", "text/plain",
                  ("WEB_TOKEN in .env refused or not read: data routes closed" if _web_tok_bad[0] else
                   "open the page as http://%s/#k=<WEB_TOKEN> (an X-GPS header)"
                   % (wifi["ip"] or "<board ip>"),))
        return
    if path.startswith(b"/status.json"):
        _web_status(c, path.startswith(b"/status.json?diag"))
    elif path.startswith(b"/hist?"):
        _web_hist(c, path)
    elif path.startswith(b"/diag?"):
        _web_diag_file(c, path)
    elif path == b"/" or path.startswith(b"/?"):
        _web_page(c, _web_gzip(req))
    else:
        _web_send(c, "404 Not Found", "text/plain", ("not found",))

def _web_status(c, diag=False):
    """status.json: its rows and satellite list built before the 200: a
    bug in one is a 500 with the error (it was a 200 with a cut JSON, and
    the page said OFFLINE). No RAM for them is no bug (it was "Caught bug"
    on the page for a day): web_tick drops the client."""
    try:
        parts = _web_json_parts(_web_rows(), _sv_list(), diag)
    except MemoryError:
        raise
    except Exception as e:
        _web_bug(e)
        _web_send(c, "500 Internal Server Error", "text/plain", (repr(e),))
        return
    _web_send(c, "200 OK", "application/json", parts)

def _web_hist(c, path):
    """/hist?d=YYYYMMDD&o=N: the day's history from N on as its text lines,
    about HIST_REPLY bytes (a whole day in one reply could outrun
    WEB_DEADLINE; see _hist_lines): X-Hist-Next is where to ask from next,
    X-Hist-End where the day's records end now (the page asks again while
    short of it, then once a minute from there: an append starts there). A
    .trk day (older firmware) is sent as it is, from byte N, HIST_REPLY
    bytes at most, no X-Hist-Next (the page counts bytes). With t=HHMMSS
    (a .trb day) no lines: X-Hist-Next is where the points from that UTC
    time on begin (_hist_seek), X-Hist-Seek 1 when the seek ran out of
    time (ask again with t from there). X-Hist-Late 0/1: whether the file
    as it is now is in time order (_hist_span's scan of this size; none
    when not scanned yet): the page stops a past day early only on a 0
    (day_span's third item could be 15 s older than the file read, a late
    upload in between cut the day and it was cached so). A day without a file is a 204 (the
    page asks for today's before its first point, every minute: a 404 was
    a console error each time); a bad day, o or t a 404."""
    d, o, t = "", 0, -1
    try:
        for kv in path[6:].decode().split("&"):
            if kv.startswith("d="):
                d = kv[2:]
            elif kv.startswith("o="):
                o = int(kv[2:])
            elif kv.startswith("t="):
                v = kv[2:]
                t = -2
                if len(v) == 6 and v.isdigit() and v < "240000" \
                        and v[2:4] < "60" and v[4:] < "60":
                    t = int(v[:2]) * 3600 + int(v[2:4]) * 60 + int(v[4:])
    except (UnicodeError, ValueError):
        d = ""
    if t == -2:
        d = ""
    f = name = None
    if len(d) == 8 and d.isdigit() and o >= 0:
        for name in (HIST_DIR + "/" + d + TRB, HIST_DIR + "/" + d + ".trk"):
            try:
                f = open(name, "rb")
                break
            except OSError:
                pass
    if f is None:
        if len(d) == 8 and d.isdigit() and o >= 0 and t != -2:
            _web_send(c, "204 No Content", "text/plain", ())
        else:
            _web_send(c, "404 Not Found", "text/plain", ("no such day",))
        return
    try:
        size = os.stat(name)[6]
        if name.endswith(TRB):
            end = _hist_end(f, size)
            o = min(o, end)
            if t >= 0:
                nxt, more = _hist_seek(f, o, end, t)
                _web_head(c, "200 OK", "text/plain",
                          "Content-Length: 0\r\nX-Hist-Next: %d\r\n"
                          "X-Hist-End: %d\r\nX-Hist-Seek: %d\r\n" % (
                              nxt, end, more))
                return
            k, nxt = _hist_lines(f, o, end, None)
            e = _hsp.get(d)
            lh = ("X-Hist-Late: %d\r\n" % e[8]
                  if e is not None and e[0] == size and e[1] else "")
            _web_head(c, "200 OK", "text/plain",
                      "Content-Length: %d\r\nX-Hist-Next: %d\r\n"
                      "X-Hist-End: %d\r\n%s" % (k, nxt, end, lh))
            _hist_lines(f, o, nxt, c)
            return
        _web_bytes(c, f, o, size, "")
    finally:
        f.close()

def _web_bytes(c, f, o, size, extra):
    """The bytes of f (size long) from o (past the end: none), HIST_REPLY at
    most, with their Content-Length: WEB_CHUNK reads into one buffer, a
    send each, the UART read between them."""
    o = min(o, size)
    n = min(size - o, HIST_REPLY)
    _web_head(c, "200 OK", "text/plain", "Content-Length: %d\r\n%s" % (
        n, extra))
    f.seek(o)
    mv = memoryview(bytearray(WEB_CHUNK))
    while n > 0:
        k = f.readinto(mv[:min(n, WEB_CHUNK)])
        if not k:
            break
        _web_out(c, mv[:k])
        n -= k
        _rcv.read_uart()

def _web_diag_file(c, path):
    """/diag?f=<0|1>&o=N: the diag log as it is on flash, DIAG_FILE (f=0)
    or DIAG_OLD (f=1), from byte N, HIST_REPLY bytes at most (the page
    reads both files in pieces, then DIAG_FILE from where it got to once a
    minute), X-Diag-Size the file's size (smaller than where the page got
    to: it rotated, the page reads both again); with f=0 X-Diag-Old
    DIAG_OLD's size (0: none), so a rotation the page missed (DIAG_FILE
    grown past where it got to again) is seen too. A missing file, a bad
    f or o is a 404."""
    name, o = None, -1
    try:
        for kv in path[6:].decode().split("&"):
            if kv == "f=0":
                name = DIAG_FILE
            elif kv == "f=1":
                name = DIAG_OLD
            elif kv.startswith("o="):
                o = int(kv[2:])
    except (UnicodeError, ValueError):
        o = -1
    f = None
    if name is not None and o >= 0:
        try:
            f = open(name, "rb")
        except OSError:
            pass
    if f is None:
        _web_send(c, "404 Not Found", "text/plain", ("no such diag file",))
        return
    try:
        size = os.stat(name)[6]
        extra = "X-Diag-Size: %d\r\n" % size
        if name == DIAG_FILE:
            extra += "X-Diag-Old: %d\r\n" % (_file_size(DIAG_OLD) or 0)
        _web_bytes(c, f, o, size, extra)
    finally:
        f.close()

_kw = [None] * 12                        # the /hist reply's decoder state

def _hist_end(f, size):
    """Where a day history .trb's records end: its footer's start (the next
    append's keyframe goes there), else its size (a cut last append)."""
    n = len(FOOTER)
    if size >= n:
        f.seek(size - n)
        if f.read(n) == FOOTER.encode():
            return size - n
    return size

def _kf_s(a, b):
    """The UTC time of day (s) of the keyframe line _rbuf[a:b] ("=...,
    YYYYMMDDTHHMMSSZ\n"), read off its text (no decoding), or -1."""
    if (b - a < 18 or _rbuf[a] != 61 or _rbuf[b - 1] != 10
            or _rbuf[b - 2] != 90 or _rbuf[b - 9] != 84):
        return -1
    r = 0
    for k in (b - 8, b - 6, b - 4):
        h, l = _rbuf[k] - 48, _rbuf[k + 1] - 48
        if not (0 <= h <= 9 and 0 <= l <= 9):
            return -1
        r = r * 60 + h * 10 + l
    return r

def _hist_seek(f, o, end, t):
    """Where a read of the day history .trb f from o (a record start) must
    begin to hold every point from t (s into the UTC day) on: the last
    keyframe at or before t, scanned up to the first one after t. Only the
    keyframes' text is read, the deltas skipped undecoded (decoding is
    what a /hist reply spends its ~2.4 s on: a local day east of UTC began
    with the whole UTC day before it, ~16 replies for its last hour). A
    keyframe earlier than the one before it (a late append) stops it
    there: what follows is read. -> (offset, more): more is 1 when
    HIST_SEEK_MS ran out first, past a keyframe (ask again from offset);
    the WDT fed and the UART read every 64 records."""
    best, last, i = o, -1, 0
    t0 = time.ticks_ms()
    _rd_open(o)
    while _rd[0] + _rd[2] < end:
        if not _rd_rec(f):
            break
        a, b = _rd[1], _rd[2]
        if a < b and _rbuf[a] == 61:
            k = _kf_s(a, b)
            if k >= 0:
                if k > t or k < last:
                    return best, 0
                best, last = _rd[0] + a, k
        i += 1
        if not i & 63:
            feed()
            _rcv.read_uart()
            if time.ticks_diff(time.ticks_ms(), t0) > HIST_SEEK_MS:
                return best, int(best > o)
    return best, 0

def _hist_span(f, size, e=None):
    """The UTC times of day (s) of a day history .trb's first and last
    point: the earliest keyframe's, the latest keyframe's or decoded
    point's after it (its deltas only, up to the next keyframe earlier in
    time, are decoded). A late append (an earlier track uploaded after a
    later one) puts keyframes out of time order: before, the first in the
    file was taken and only the last one's deltas read (a day the picker
    left out). -> the day's _hsp entry [size, span, first, hi, hk, off,
    more, pk, late]: span [first, last], [first, last, 1] when the file is
    not in time order (late: /hist says so, X-Hist-Late, and the page reads
    a past day's file whole), None without a keyframe or while the
    keyframe scan is not through.
    When HIST_SEEK_MS runs out it stops at a record start (off, more 1)
    and the next call goes on from there; e, the entry of an earlier call,
    also lets a grown file be scanned from where that one got to (a day
    grows by appends at its footer). Before, a day too long for one scan
    had no span for good, and a growing one was read from 0 at each poll."""
    end = _hist_end(f, size)
    first, hi, hk, o, pk, late = -1, -1, -1, 0, -1, 0
    if e is not None and size >= e[0] and 0 < e[5] <= end:
        first, hi, hk, o, pk, late = e[2], e[3], e[4], e[5], e[7], e[8]
    i = 0
    t0 = time.ticks_ms()
    _rd_open(o)
    while _rd[0] + _rd[2] < end:
        if not _rd_rec(f):
            break
        a, b = _rd[1], _rd[2]
        if a < b and _rbuf[a] == 61:
            k = _kf_s(a, b)
            if k >= 0:
                if first < 0 or k < first:
                    first = k
                if k <= pk:              # (equal: an append drawn again;
                    late = 1             # one append's keyframes rise)
                pk = k
                if k > hi:               # (the first of equal ones: a
                    hi, hk = k, _rd[0] + a   # copy drawn again after it)
        i += 1
        if not i & 63:
            feed()
            _rcv.read_uart()
            if time.ticks_diff(time.ticks_ms(), t0) > HIST_SEEK_MS:
                return [size, None, first, hi, hk, _rd[0] + _rd[2], 1, pk,
                        late]
    # where the next append begins; only past a footer: without one the
    # last append may be cut, and the file is read again from its start
    o = end if end < size else 0
    if hk < 0:
        return [size, None, first, hi, hk, o, 0, pk, late]
    s = _kw
    s[0] = None
    t = hi
    _rd_open(hk)
    i = 0
    while _rd[0] + _rd[2] < end:
        if not _rd_rec(f):
            break
        i += 1
        if not i & 63:                   # (a stretch without a keyframe
            feed()                       # after it, damaged: walked unfed)
            _rcv.read_uart()
        a, b = _rd[1], _rd[2]
        if a == b:
            s[0] = None
            continue
        # (the next keyframe earlier in time: a late append's; one of the
        # same time, a batch drawn again, is decoded on: a shorter or a
        # longer copy, the last point the longer one's)
        if _rd[0] + a > hk and _rbuf[a] == 61 and 0 <= _kf_s(a, b) < hi:
            break
        if _kb_rec(a, b, s) > 0 and s[0] is not None:
            t = max(t, s[0] % 86400)
    return [size, [first, t, 1] if late else [first, t], first, hi, hk, o,
            0, pk, late]

_hsp = {}                                # day -> _hist_span's entry

def _hist_spans():
    """{"YYYYMMDD": [first, last(, 1)]}: each .trb day's first and last
    point (s into the UTC day, _hist_span; 1: not in time order) for the
    page's day picker (a UTC day
    offers the local days of both: east of UTC its last hour is the next
    local day, offered only when a point is there). Kept per day with the
    file's size: one day (re)read a call at most, the others as they
    were (a scan HIST_SEEK_MS cut short goes on at the next call); a .trk
    day (older firmware) or one not read yet has none."""
    days = _hist_days()
    for d in list(_hsp):
        if d not in days:
            del _hsp[d]
    out, done = {}, False
    for d in days:
        name = HIST_DIR + "/" + d + TRB
        size = _file_size(name)
        e = _hsp.get(d)
        if size and not done and (e is None or e[0] != size or e[6]):
            done = True
            try:
                with open(name, "rb") as f:
                    e = _hsp[d] = _hist_span(f, size, e)
            except OSError:
                e = _hsp[d] = [size, None, -1, -1, -1, 0, 0, -1, 0]
        if e is not None and e[0] == size and e[1]:
            out[d] = e[1]
    return out

def _hist_lines(f, o, end, c):
    """The text lines of the day history .trb f from o (a keyframe, or
    where an earlier reply stopped) to end. c None: counted -> (their
    bytes, the next o): once HIST_REPLY bytes of text (a 1 Hz drive's
    ~2.3 KB of records) or a quarter of that of the file are read, before
    the next keyframe (or any record while no keyframe holds: none decodes
    there before one, so a stretch without keyframes, an erased block of
    1-byte records, is read in steps too), else end. With c: sent to it,
    gathered into WEB_CHUNK sends. A reply decodes its records twice, the
    length first: the same lines, from the same bytes (a record ends where
    its own bytes say, a keyframe resets the state). A decoded line (at
    _rbuf[_SCR:]) is copied from a view of its length kept for the reply:
    a new view a line was ~12 KB of garbage a reply. The WDT fed and the
    UART read every 64 records."""
    s = _kw
    s[0] = None
    _rd_open(o)
    if c is not None:
        mv = memoryview(bytearray(WEB_CHUNK))
        sv = [None] * (len(_rbuf) - _SCR + 1)
    n = i = q = 0
    while _rd[0] + _rd[2] < end:
        r = _rd[0] + _rd[2]
        if not _rd_rec(f):
            break
        a, b = _rd[1], _rd[2]
        if (c is None and r > o
                and (n >= HIST_REPLY or r - o >= HIST_REPLY >> 2)
                and (a == b or _rbuf[a] == 61 or s[0] is None)):
            return n, r
        i += 1
        if not i & 63:
            feed()
            _rcv.read_uart()
        if a == b:                       # (an over-long line, drained)
            s[0] = None
            continue
        if _kb_rec(a, b, s) <= 0:
            continue
        a, b = _kr
        k = b - a
        if c is None:
            n += k
            continue
        if q + k > WEB_CHUNK:
            _web_out(c, mv[:q])
            q = 0
            _rcv.read_uart()
        if a == _SCR:
            v = sv[k]
            if v is None:
                v = sv[k] = _rmv[a:b]
        else:
            v = _rmv[a:b]                # (a keyframe's, where it was read)
        mv[q:q + k] = v
        q += k
    if q:
        _web_out(c, mv[:q])
    return n, end

_web_errs = [0]                          # page requests that raised a bug
_bugs = {}                               # a caught bug's kind -> Stamp of its
                                         # last diag line
BUG_KINDS = 8                            # at most this many kinds a boot
BUG_AGAIN = 3600                         # s: a kind still happening is
                                         # written again

def _bug_once(line, kind=None):
    """A caught bug's diag line, once a kind (a bug every request or every
    loop must not fill the log), BUG_KINDS kinds at most. The kind is the
    tick and the exception type (kind), not the text: a message that
    varies (an index, a key) filled BUG_KINDS with one bug, and no later
    one was logged. Written again every BUG_AGAIN while it keeps coming:
    the one line a boot rotated out of the diag in a day or two while the
    bug went on every 5 s."""
    if kind is None:
        kind = line
    at = _bugs.get(kind)
    if at is not None and _recent_s(at, BUG_AGAIN):
        return
    if at is None and len(_bugs) >= BUG_KINDS:
        return
    _bugs[kind] = _stamp()
    _diag_write(line)

def _web_bug(e):
    """A bug on a page request: counted (web_errs in the JSON), each new
    kind in diag (only the first since boot was). It must not reset the
    logger (run()'s handler)."""
    print("web error", repr(e))
    _web_errs[0] += 1
    _bug_once("web error %s" % repr(e)[:60], ("web error", type(e)))

# newlib's numbers of the names MicroPython's errno module leaves out
# (MICROPY_PY_ERRNO_LIST: no EINTR, ENFILE, EMFILE, EWOULDBLOCK), the
# same as Linux's: lwIP's accept says ENFILE when it is out of sockets
_ERRNO_MORE = {"EWOULDBLOCK": 11, "EINTR": 4, "ENFILE": 23, "EMFILE": 24}

def _errnos(*names):
    """The errno numbers of names: the board's (newlib: ECONNABORTED 113,
    ETIMEDOUT 116), not Linux's (103, 110: a client's RST closed the page's
    listen socket for WEB_RETRY); a name the module lacks by _ERRNO_MORE."""
    try:
        import errno
    except ImportError:
        errno = None
    out = []
    for n in names:
        v = getattr(errno, n, None) if errno is not None else None
        if v is None:
            v = _ERRNO_MORE.get(n)
        if v is not None and v not in out:
            out.append(v)
    return tuple(out)

# an accept that fails with one of these: a client gone, the IDF short of
# memory or sockets for a moment; no broken listen socket
WEB_ACCEPT_OK = _errnos("EAGAIN", "EWOULDBLOCK", "ETIMEDOUT", "ECONNABORTED",
                        "ECONNRESET", "ENOTCONN", "EINTR", "ENOMEM", "ENFILE",
                        "EMFILE", "ENOBUFS")
WEB_BROKEN_N = 3                         # other accept errors in a row: reopened
_web_acc_err = [0]

def _web_broken(why):
    """The listen socket errs: closed, opened again after WEB_RETRY."""
    _bug_once("web error listen socket %s: opened again" % why,
              ("web listen broken",))
    _web_acc_err[0] = 0
    _web_close()
    _web_fail_at[0] = time.ticks_ms()

def web_tick():
    """Listen while connected, close otherwise; answer one waiting client
    within WEB_DEADLINE. Off home only on a network with SSIDn_WEB=1 or
    with WEB_AWAY (which also keep the radio on for it there, wifi_tick,
    _web_link): the page has no password, and a
    hotspot or a friend's network got the live position, the 7-day track
    and the diag log's SSIDs while the radio was up for the uploads."""
    if not WEB or wifi["state"] != "connected" or not _web_link():
        _web_close()
        return
    if _web_sock is None:
        now = time.ticks_ms()            # (a socket or bind that keeps
        t = _web_fail_at[0]              # failing was tried each loop pass:
        if t is not None and 0 <= time.ticks_diff(now, t) < WEB_RETRY:
            return                       # a socket a 20 ms, the IDF heap)
        _web_open()
        if _web_sock is None:
            _web_fail_at[0] = now
            return
        _web_fail_at[0] = None
    if _web_poll is not None:
        for x in _web_poll.ipoll(0):
            if x[1] & (getattr(select, "POLLERR", 8) | getattr(select, "POLLHUP", 16)):
                _web_broken("poll %d" % x[1])
                return
            break
        else:
            return                       # nobody waiting
    try:
        c = _web_sock.accept()[0]
    except OSError as e:
        # nobody waiting (EAGAIN), or one gone before it was taken; any
        # other error is a socket that will not take one again (a netif
        # reset under it): each pass returned here, the page dead for good
        if (e.args[0] if e.args else None) in WEB_ACCEPT_OK:
            return
        _web_acc_err[0] += 1             # (one is no broken socket either)
        if _web_acc_err[0] >= WEB_BROKEN_N:
            _web_broken(repr(e)[:50])
        return
    _web_acc_err[0] = 0
    _web_end[0] = time.ticks_add(time.ticks_ms(), WEB_DEADLINE * 1000)
    try:
        req = _web_read(c)
        _rcv.read_uart()
        _web_reply(c, req)
    except (OSError, MemoryError) as e:
        print("web client", repr(e))
    except Exception as e:               # a bug on a page request
        _web_bug(e)
    finally:
        _web_end[0] = None
        try:
            c.close()
        except OSError:
            pass
        gc.collect()

# --- GPS state in words ------------------------------------------------------

def status():
    if st["rx"] is None or _age(time.ticks_ms(), st["rx"]) > _rcv.NO_DATA:
        return "No data!"
    if not st["valid"]:
        return "Waiting fix"
    s = {2: "2D fix", 3: "3D fix"}.get(st["fixtype"], "Fix")
    return s + " DGPS" if st["quality"] == 2 else s

# --- The Heltec's TFT (gps_tft.py) -------------------------------------------
TFT_FRAME = 200                          # ms between frames (a row scrolling)
TFT_STATE = 1000                         # ms between new values
TFT_MAX_ERR = 3                          # errors in a row, then it goes dark
TFT_WIFI_UP = 3                          # dB past a limit for one more WiFi bar
TFT_HDOP_GOOD = 1.5                      # strong GPS: HDOP at most this ...
TFT_SATS_GOOD = 10                       # ... with this many satellites in the fix
TFT_HDOP_FAIR = 3.0                      # above this: weak
TFT_STORE = 30000                        # ms between looks at the flash (statvfs
                                         # walks it)
TFT_POINT_B = 7                          # bytes a .trb point of a 1 Hz drive takes
                                         # (as web.html's POINT_B)
_tft = None
_tft_t = [None, None, 0, 0, 0]           # last values, last frame, the errors
                                         # in a row of each (4: today's route)
_tft_b = [None, None, None]              # (2: the WiFi level last shown)

# The backlight is the one part that wears: dim by day, dark in a stay (the
# board sits on a desk, or is not looked at), full after the USER button.
TFT_LIGHT = 30                           # .env TFT_LIGHT: per cent, 0-100
TFT_IDLE = 300000                        # .env TFT_IDLE, in s: ms into a stay it goes
                                         # dark; 0: never
TFT_WAKE = 60000                         # ms of full light after the USER button
_tft_l = [None, None, None]              # the per cent set; the stay's first tick
                                         # (kept TFT_IDLE old once dark); the
                                         # button's last press (None once
                                         # TFT_WAKE old): no stamp ages past 2**29 ms

def _tft_env(env):
    """TFT_LIGHT=<0-100> per cent of backlight, TFT_IDLE=<0-86400> seconds of a
    stay before it goes dark (0: never); anything else keeps the default."""
    global TFT_LIGHT, TFT_IDLE
    v = _plain(env.get("TFT_LIGHT"))
    if v:
        try:
            n = int(v)
        except ValueError:
            n = -1
        if 0 <= n <= 100:
            TFT_LIGHT = n
        else:
            _env_refused("TFT_LIGHT", "not 0-100", TFT_LIGHT)
    v = _plain(env.get("TFT_IDLE"))
    if v:
        try:
            n = int(v)
        except ValueError:
            n = -1
        if 0 <= n <= 86400:
            TFT_IDLE = n * 1000
        else:
            _env_refused("TFT_IDLE", "not 0-86400 s", TFT_IDLE // 1000)

def _tft_light(now):
    """The backlight: full for TFT_WAKE after the USER button, else dark once a
    stay is TFT_IDLE old, else TFT_LIGHT. A stay: the GPS says the board has
    not moved, or it is on the home network and no usable fix came for
    STILL_GAP (on a desk indoors, where none proves a stay). A change is said
    in the diag, the first setting is not."""
    L = _tft_l
    sl = log["still"]
    if sl["active"] or (at_home() and not _recent_s(sl["last_at"], STILL_GAP // 1000)):
        if L[1] is None:
            L[1] = now
    else:
        L[1] = None
    if not _recent(now, L[2], TFT_WAKE):
        L[2] = None                      # (a stamp left alone for 12 days reads young)
    if L[2] is not None:
        pct = 100
    elif TFT_IDLE and L[1] is not None and not _recent(now, L[1], TFT_IDLE):
        pct = 0
        L[1] = time.ticks_add(now, -TFT_IDLE)
    else:
        pct = TFT_LIGHT
    if pct != L[0]:
        _tft.light(pct)
        if L[0] is not None:
            _diag_write("tft light %d" % pct)
        L[0] = pct

def _tft_start():
    """A failure leaves the logger running without it."""
    global _tft
    if _tft is not None:
        return
    try:
        import gps_tft
        gps_tft.start(TFT_LIGHT, HELTEC_TFT_BL)   # (.env read: _tft_env)
        _tft = gps_tft
    except Exception as e:
        print("tft", repr(e))
        return
    try:
        _rt_start()
    except Exception as e:               # (the display goes on without a route)
        print("tft route", repr(e))

_T_DAYS = (0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4)

def _last_sunday(y, m):
    """The day of March's or October's (31 days) last Sunday."""
    yy = y - 1 if m < 3 else y
    return 31 - (yy + yy // 4 - yy // 100 + yy // 400 + _T_DAYS[m - 1] + 31) % 7

def _utc_hours(y, m, day, h):
    """Hours Hungary is ahead of UTC at that UTC hour: CET, CEST from the last
    Sunday of March to October's, 01:00 UTC."""
    if 3 < m < 10:
        return 2
    if m in (3, 10):
        ls = _last_sunday(y, m)
        past = day > ls or (day == ls and h >= 1)
        return 2 if past == (m == 3) else 1
    return 1

# --- NTP --------------------------------------------------------------------
# The display's clock and the route's day before the first fix, which indoors may
# take long or never come: a fix is otherwise the only thing that tells the date.
# Asked on any network until a fix has told the time, and then never again: a fix
# wins, and this reaches neither a track line nor status.json. The board clock (TLS
# dates, _rtc_sync) takes it when no fix has: an https GEO_URL / OTA_URL keeps it
# asked until that clock is set, even after a fix (which _rtc_sync prefers).
NTP_SERVER = "pool.ntp.org"              # .env NTP_SERVER; empty: never asked
NTP_WAIT = 1000                          # ms the answer may take. The name lookup
                                         # is not bound by it (lwIP's own, 7 s
                                         # measured without an answer): _resolve
                                         # asks a failed network again only
                                         # after DNS_RETRY
NTP_ERA = 252460800                      # s (8 years): the next era (2036-02-07 on)
                                         # is believed only for a build this young
NTP_RETRY = 60000                        # ms after a failed try, doubling to ...
NTP_RETRY_MAX = 3600000                  # ... this
NTP_AGAIN = 21600000                     # ms an answer is used before a new one
_NTP_2020 = 3786825600                   # 2020-01-01 in seconds since 1900
_ntp = [None, None, NTP_RETRY // 2]      # the answer's Anchor (seconds since 2020,
                                         # Stamp); Stamp of the last try; the wait
                                         # after it (ms): half, as a failure doubles
                                         # it first (the first retry was 120 s)

def _ntp_env(env):
    global NTP_SERVER
    if "NTP_SERVER" in env:
        NTP_SERVER = _plain(env["NTP_SERVER"]) or None

def _ntp_ask():
    """Seconds since 2020 from NTP_SERVER, None: no answer, or none to believe (not
    synchronised, no server, not an answer to this request, or a time before this
    build was made). The request's ticks go out as its transmit time and must come
    back as the answer's originate time (a late or stray datagram is no answer).
    Past the NTP era (2036-02-07) the seconds count from there: the small value
    is read as the next era, but only up to NTP_ERA after this build was made
    (a server saying 1900-1951 is not 2036-2088)."""
    s = None
    try:
        ai = _resolve(NTP_SERVER, 123, socket.SOCK_DGRAM)
        s = socket.socket(ai[0], socket.SOCK_DGRAM, ai[2])
        s.settimeout(NTP_WAIT / 1000)
        q = bytearray(48)
        q[0] = 0x23                      # version 4, a client
        struct.pack_into(">I", q, 40, time.ticks_ms())
        s.sendto(q, ai[-1])
        b = s.recv(48)
    except (OSError, IndexError, ValueError):
        return None
    finally:
        if s is not None:
            s.close()
    if (len(b) < 48 or b[0] >> 6 == 3 or b[0] & 7 != 4 or not 0 < b[1] < 16
            or b[24:32] != q[40:48]):
        return None
    v = struct.unpack_from(">I", b, 40)[0]
    if not v:
        return None
    t = v - _NTP_2020
    lo = _clock_floor()
    if ota.get("seen_err"):              # (the floor unread: only the build's,
        return None                      # an answer before the newest seen
                                         # passed; asked again later)
    if t < 0:
        t += 1 << 32
        if t > lo + NTP_ERA:
            return None
    # (and no later than NTP_AHEAD after that floor: an answer of 2040 from
    # whoever is on the path made every certificate look expired; nearer
    # ones still do, until the next network or a TLS error asks again,
    # _ntp_doubt, or a fix)
    if lo and t >= lo + NTP_AHEAD:
        return None
    return t if lo <= t < FIX_S_END else None

NTP_AHEAD = 157680000                    # s (5 years) an answer may be past the build

_ntp_net = [None]                        # the network NTP's answer came on

def _ntp_doubt():
    """A verified POST's handshake failed with the board clock set from NTP
    (no fix): the answer may be wrong (a certificate "expired" by a lie):
    asked again at the next ntp_tick, once an answer."""
    a = _ntp[0]
    if a is not None and _rtc_ntp[0] is a and _ntp_doubted[0] is not a:
        _ntp_doubted[0] = a
        _ntp[1], _ntp[2] = None, NTP_RETRY // 2

_ntp_doubted = [None]

def _need_rtc():
    """The board clock matters (a verified TLS request checks the dates of
    the certificate) and nothing has set it yet."""
    return not _rtc_set[0] and ((up["url"] or "").startswith("https")
                                or (OTA_URL or "").startswith("https"))

def ntp_tick(now):
    """Ask NTP while no fix has told the time this boot. Once one has, its
    clock (log["clock"]) is what every reader takes over NTP's, and it is
    also what sets the board clock (_rtc_sync): asking on cost a DNS lookup
    and a UDP wait that could block past what the UART buffer holds, for
    an answer nothing used."""
    n = _ntp
    if log["clock"] is not None:
        if _need_rtc():
            _rtc_sync(now)
        return
    if (not NTP_SERVER or wifi["state"] != "connected"
            or _recent_s(n[1], n[2] // 1000)):
        return
    n[1] = _stamp()
    _rcv.read_uart()
    feed()
    t = _ntp_ask()
    feed()
    _rcv.read_uart()
    if t is None:
        # after an answer (NTP_AGAIN) the first failure retries in
        # NTP_RETRY, not in NTP_RETRY_MAX
        n[2] = NTP_RETRY if n[2] > NTP_RETRY_MAX else min(n[2] * 2,
                                                          NTP_RETRY_MAX)
        return
    n[0], n[2] = (t, _stamp()), NTP_AGAIN
    _ntp_net[0] = wifi["ssid"]
    _tft_t[0] = None                     # the clock and the route's day now,
    _Rt.drawn = None                     # not on the next TFT_STATE / ROUTE_DRAW
    _diag_write("clock ntp %s" % _stamp_s(t)[0])

def _local_hm():
    """"HH:MM" in Hungary from the fix's UTC; with the receiver silent (its time
    stands still then) or no fix yet, from the last fix's Anchor run on by the
    uptime, else NTP's (as _rt_tick); "--:--" without any."""
    d, t = st["date"], st["time"]
    rx = st["rx"]
    now = time.ticks_ms()
    if not d or not t or not _recent(now, rx, _rcv.NO_DATA):
        c = log["clock"] or _ntp[0]
        if c is None:
            return "--:--"
        d = _stamp_s(_utc_s(c))[0]
        t = d[11:]
    try:
        y, m, day = int(d[:4]), int(d[5:7]), int(d[8:10])
        h = int(t[:2])
        return "%02d:%s" % ((h + _utc_hours(y, m, day, h)) % 24, t[3:5])
    except (ValueError, IndexError):
        return "--:--"

def _gps_quality(fix):
    """0 no fix, 1 weak, 2 fair, 3 strong, from HDOP and the satellites in the fix.
    Measured: outdoors HDOP 0.7-1.0 with ~26, by a window 3-9 with 8-14."""
    if not fix:
        return 0
    hdop = st.get("hdop")
    if hdop is None or hdop > TFT_HDOP_FAIR:
        return 1
    return 3 if hdop <= TFT_HDOP_GOOD and (_rcv.sv_used() or 0) >= TFT_SATS_GOOD else 2

def _wifi_level(rssi, last):
    """The bars of WiFi lit: down at once, up only TFT_WIFI_UP dB past the
    limit, so a signal around a limit does not flicker."""
    if rssi is None:
        return 0
    for level, limit in ((3, -60), (2, -70), (1, -80)):
        if rssi >= limit:
            break
    else:
        level = 0
    if last is not None and level > last:
        for up in range(level, last, -1):
            if rssi >= (-60, -70, -80)[3 - up] + TFT_WIFI_UP:
                return up
        return last
    return level

_tft_fs = [None, None]                   # (what _store_state said, its ticks)

def _store_left(room):
    """How long room bytes hold a 1 Hz drive, in the display's words."""
    if room <= 0:
        return "tele"
    h = room // TFT_POINT_B // 3600
    if h < 1:
        return "<1 óra"
    return "%d nap" % (h // 24) if h >= 48 else "%d óra" % h

def _store_state(now):
    """(Per cent of the flash taken, how long a drive still fits in it), at most
    every TFT_STORE, or None when unreadable. Older days of the history count as
    free: they go before any point (_hist_trim). The reserve (MIN_FREE) does not:
    nothing is logged into it. Standing still costs next to nothing, so the time
    is the worst case."""
    c = _tft_fs
    if _recent(now, c[1], TFT_STORE):
        return c[0]
    c[1] = now
    c[0] = None
    try:
        v = os.statvfs(LOG_DIR)
    except OSError:
        return None
    room = v[0] * v[3]
    total = v[0] * v[2]
    if total <= 0:
        return None
    try:
        ns = _hist_names()
        for n in ns:
            if n[:8] != ns[-1][:8]:      # (not the newest day's, .trk or .trb)
                room += _file_size(HIST_DIR + "/" + n) or 0
    except OSError:
        pass
    left = room - MIN_FREE
    if left <= 0:
        pct = 100
    else:
        pct = min(((total - room) * 100 + total - 1) // total, 99)
    c[0] = (pct, _store_left(left))
    return c[0]

# Today on the display: the route as ROUTE_CAP points and the km, of the Hungarian
# day. Fed only by the lines flush() has written (_rt_feed), so what a stay's proof
# dropped as drift or a full flash lost never counts, and the 1 Hz path does nothing
# for it: it shows up to a FLUSH late, and a walk's lines a stay still holds later.
# Anchor points are the stay's place and no way travelled (the car driving off past
# one read 44 m in a second): skipped. A trip starts when a stay ends, when a fix gap
# starts a new file (_rt_trip), or with more than NEW_FILE_GAP between two lines: no
# km and no line across. RAM: the points, 8 B each, and their pixels, 2 B each.
ROUTE_CAP = 64                           # points; when full every other one goes
ROUTE_STEP = 50                          # m between points at first; each halving
                                         # doubles it, so any day fits
ROUTE_STEP_MAX = 1 << 24                 # m: past any way a day holds (and the file's
                                         # field: trip starts add points without one)
ROUTE_SPAN = 4497                        # the field's width shows at least 500 m
                                         # (6 m a pixel), in millionths of a degree
ROUTE_W, ROUTE_H = 83, 45                # the pixels a point can land on (the
                                         # display's battery line is below)
ROUTE_DRAW = 10000                       # ms between looks at the route
ROUTE_SAVE = 300000                      # ms between saves of a changed route (and
                                         # one when a stay starts)
ROUTE_BACK = 300                         # s the clock may be behind the route's
                                         # day before that day counts as false
ROUTE_FILE = "/route.bin"
ROUTE_HEAD = "<4siiiii"                  # magic, day, cm, step, last line's s, n
ROUTE_MAGIC = b"RTE1"
_CM_E6 = 11.1194927                      # cm per millionth of a degree of latitude

class _Rt:
    pts = xy = view = None               # lat << 1 | starts a trip, lon: 1e-6 degrees;
    km = ""                              # the pixels, y | 128 when starting a trip
    day = -1                             # Hungarian day number, -1 not known yet
    waiting = False                      # read from flash, shown, day not proven yet
    n = cm = ver = 0                     # points; centimetres today; views made
    step = ROUTE_STEP
    k = 1.0                              # the cosine of the latitude, lately
    last_s = trip = tail = None          # last line's time; first time of a new trip
    hour = None                          # (the hour hday was worked out for)
    hday = 0
    drawn = None                         # ticks of the last look
    saved = None                         # Stamp of the last save
    moved = unsaved = False
    save_ok = True

def _e6(line, a, b):
    """Millionths of a degree of a track line's coordinate at line[a:b], without a
    float: a single float holds a latitude to only 0.4 m, and these get summed."""
    d = line.find(".", a, b)
    v = int(line[a:d]) * 1000000
    f = int(line[d + 1:d + 7])
    return v - f if line[a] == "-" else v + f

def _rt_step(lat1, lon1, lat2, lon2):
    """Centimetres between two positions in millionths of a degree. A line of a
    drive allocates this path's floats every second of it: as few as can be, the
    cosine kept from the last point added (R.k)."""
    x = lon2 - lon1
    if x > 180000000:
        x -= 360000000
    elif x < -180000000:
        x += 360000000
    x *= _Rt.k
    y = lat2 - lat1
    return int(_CM_E6 * math.sqrt(x * x + y * y) + 0.5)

def _rt_day(s):
    """The Hungarian day of s (seconds since 2020 UTC), counted from 2020. It can
    change only on a UTC hour, so most calls just compare."""
    R = _Rt
    h = s // 3600
    if h != R.hour:
        y, m, d = _civil(s // 86400 + EPOCH_DAYS)
        R.hour = h
        R.hday = (s + _utc_hours(y, m, d, h % 24) * 3600) // 86400
    return R.hday

def _rt_reset(day):
    R = _Rt
    R.day = day
    R.n = R.cm = 0
    R.step = ROUTE_STEP
    R.last_s = R.trip = None
    R.moved = True
    R.unsaved = False                    # (nothing worth a write yet: the old
                                         # day's file is ignored by its day)

def _rt_halve():
    """Every other point goes, in place; the first and the last stay. A trip mark
    moves on to the next point kept. Double the spacing from here on."""
    R = _Rt
    p, n = R.pts, R.n
    j = 1
    mark = 0
    i = 1
    while i < n:
        if i & 1 and i < n - 1:
            mark |= p[2 * i] & 1
        else:
            p[2 * j] = p[2 * i] | mark
            p[2 * j + 1] = p[2 * i + 1]
            j += 1
            mark = 0
        i += 1
    R.n = j
    if R.step < ROUTE_STEP_MAX:
        R.step *= 2

def _rt_add(lat, lon, new):
    R = _Rt
    if R.n >= ROUTE_CAP:
        _rt_halve()
    i = 2 * R.n
    R.pts[i] = lat << 1 | new
    R.pts[i + 1] = lon
    R.n += 1
    R.k = math.cos(math.radians(lat / 1000000))

def _rt_line(line):
    """One line written to the track. The last point is always the last position:
    it moves along until about ROUTE_STEP from the one before (9 millionths of a
    degree a metre, near enough for spacing: no float), then stays. An anchor
    point (its time in log["hb"], _still_heartbeat) is no way travelled: skipped.
    (Told by its empty height and HDOP before, which a fix of a second without
    GGA has too: those were skipped with it.)"""
    R = _Rt
    tc = line[-17:-1]
    h = log["hb"]
    if tc in h:
        h.remove(tc)
        return
    a = line.find(",")
    b = line.find(",", a + 1)
    z = line.encode()
    s = _kb_time(z, len(z) - 17)
    if s is None:
        return
    d = _rt_day(s)
    if R.waiting or d > R.day:
        if d != R.day:
            _rt_reset(d)
        R.waiting = False
    elif d < R.day:
        return                           # (held over midnight: yesterday's)
    lat, lon = _e6(line, 0, a), _e6(line, a + 1, b)
    new = R.n == 0 or (R.last_s is not None and s - R.last_s > NEW_FILE_GAP)
    if R.trip is not None and tc >= R.trip:
        R.trip = None
        new = True
    R.last_s = s
    R.moved = R.unsaved = True
    if new:
        _rt_add(lat, lon, 1)
        return
    p = R.pts
    i = 2 * (R.n - 1)
    la, lo = p[i] >> 1, p[i + 1]
    R.cm += _rt_step(la, lo, lat, lon)
    if p[i] & 1 or abs(la - (p[i - 2] >> 1)) + abs(lo - p[i - 1]) * 3 // 4 >= 9 * R.step:
        _rt_add(lat, lon, 0)
    else:
        p[i] = lat << 1
        p[i + 1] = lon

def _rt_bad(e):
    _tft_t[4] += 1
    _tft_bad(4, e)

def _rt_feed(batch, k):
    """The k lines at batch's start just landed in the track. Nothing leaves this:
    the logging must go on whatever happens here."""
    if _Rt.pts is None or _tft is None:
        return
    try:
        i = 0
        while i < k:
            _rt_line(batch[i])
            i += 1
        _tft_t[4] = 0
    except Exception as e:
        _rt_bad(e)

def _rt_trip(s=None, fx=None):
    """The next line, or the one at s seconds since 2020, starts a new trip."""
    try:
        _Rt.trip = _tc(fx) if s is None else _stamp_s(s)[1]
    except Exception:                    # (no date yet: nothing to split)
        pass

def _rt_project():
    """The points, and the position just recorded, to pixels: north up, true aspect,
    centred, fitted to ROUTE_W x ROUTE_H, not closer than ROUTE_SPAN across. The
    view changes only here, so an unchanged route redraws nothing."""
    R = _Rt
    p, xy, n = R.pts, R.xy, R.n
    t = R.tail
    m = n + 1 if t else n
    if not m:
        R.view = (xy, 0, 0, R.ver)
    else:
        lon0 = p[1] if n else t[1]
        la0 = la1 = p[0] >> 1 if n else t[0]
        lo0 = lo1 = 0
        i = 0
        while i < m:
            la = (p[2 * i] >> 1) if i < n else t[0]
            lo = ((p[2 * i + 1] if i < n else t[1]) - lon0 + 180000000) % 360000000 - 180000000
            la0, la1 = min(la0, la), max(la1, la)
            lo0, lo1 = min(lo0, lo), max(lo1, lo)
            i += 1
        k = math.cos(math.radians((la0 + la1) / 2000000))
        w, h = (lo1 - lo0) * k, la1 - la0
        sc = ROUTE_W / ROUTE_SPAN
        if w * sc > ROUTE_W:
            sc = ROUTE_W / w
        if h * sc > ROUTE_H:
            sc = ROUTE_H / h
        ox, oy = (ROUTE_W - w * sc) / 2 + 0.5, (ROUTE_H - h * sc) / 2 + 0.5
        red = i = 0
        while i < m:
            if i < n:
                v = p[2 * i]
                la, lo, new = v >> 1, p[2 * i + 1], v & 1
            else:                        # (a new trip not on the track yet: no line
                la, lo, new = t[0], t[1], 1 if R.trip is not None or not n else 0
            xy[2 * i] = int(ox + (((lo - lon0 + 180000000) % 360000000 - 180000000) - lo0) * k * sc)
            xy[2 * i + 1] = int(oy + (la1 - la) * sc) | new << 7
            if new:
                red = i
            i += 1
        R.view = (xy, m, red, R.ver)
    R.ver += 1
    c = R.cm + 5000                      # 8,6 km; from 100 km on whole km
    if c < 10000000:
        R.km = "%d,%d km" % (c // 100000, c // 10000 % 10)
    else:
        R.km = "%d km" % ((R.cm + 50000) // 100000)

def _rt_load():
    """The route as last saved. Anything wrong with the file: as if none. It shows at
    once, whatever its day: a board indoors may wait long for a fix, the only thing that
    tells the date but NTP (ntp_tick). The first of them keeps it, or empties it if
    that was another day."""
    R = _Rt
    size = struct.calcsize(ROUTE_HEAD)
    try:
        with open(ROUTE_FILE, "rb") as f:
            b = f.read(size + 8 * ROUTE_CAP + 5)
    except OSError:
        return False
    if len(b) < size + 4:
        return False
    magic, day, cm, step, last, n = struct.unpack_from(ROUTE_HEAD, b, 0)
    if (magic != ROUTE_MAGIC or not 0 < n <= ROUTE_CAP or len(b) != size + 8 * n + 4
            or sum(b[:-4]) != struct.unpack_from("<I", b, len(b) - 4)[0]
            or step < ROUTE_STEP or cm < 0 or day < 0):
        return False
    q = array("i", b[size:size + 8 * n])
    i = 0
    while i < 2 * n:
        R.pts[i] = q[i]
        i += 1
    R.day, R.cm, R.step, R.last_s, R.n = day, cm, step, last, n
    R.k = math.cos(math.radians((R.pts[2 * n - 2] >> 1) / 1000000))
    R.waiting = R.moved = True
    return True

def _rt_save(now):
    """Atomically, as the upload's .pos: a power cut leaves the old file or the new."""
    R = _Rt
    R.saved = _stamp()
    R.save_ok = False                    # (a stay retries as seldom as a drive)
    room = free()
    if room is None or room < MIN_FREE:  # (the flash's last room is for the tracks)
        return
    b = (struct.pack(ROUTE_HEAD, ROUTE_MAGIC, R.day, R.cm, R.step, R.last_s or 0, R.n)
         + bytes(memoryview(R.pts)[:2 * R.n]))
    tmp = ROUTE_FILE + ".tmp"
    with open(tmp, "wb") as f:
        f.write(b)
        f.write(struct.pack("<I", sum(b)))
    os.rename(tmp, ROUTE_FILE)
    R.unsaved = False
    R.save_ok = True

def _rt_start():
    """Allocated only here: on the Heltec, once."""
    R = _Rt
    R.pts = array("i", bytes(8 * ROUTE_CAP))
    R.xy = bytearray(2 * (ROUTE_CAP + 1))
    try:
        _rt_load()
    except (ValueError, TypeError):      # (struct, array: a file that makes no sense)
        R.n = R.cm = 0
        R.waiting = False
        R.day = -1

def _rt_tick(now):
    """Every ROUTE_DRAW: midnight, the current position, the pixels when any of it
    changed, the file when due. Without a fix the clock runs on from the last one."""
    R = _Rt
    if R.pts is None or _recent(now, R.drawn, ROUTE_DRAW):
        return
    R.drawn = now
    c = log["clock"] or _ntp[0]
    if c is not None:
        s = _utc_s(c)
        d = _rt_day(s)
        # the fix's clock more than ROUTE_BACK s before R.day began: R.day
        # came from a false date (a corrupt digit past the checksum, a
        # wrong NTP answer). Days only went forward, so today's route was
        # gone and every line after it dropped until a reboot. (Not NTP's
        # clock alone, and not a second's step back at midnight.)
        back = (d < R.day and c is log["clock"]
                and _rt_day(s + ROUTE_BACK) < R.day)
        if R.waiting or d > R.day or back:
            if d != R.day:
                # back from a false day: today's route is still the saved
                # one (a reset writes nothing until there is something new)
                if not (back and _rt_load() and R.day == d):
                    _rt_reset(d)
            R.waiting = False
            R.moved = True
    if R.day < 0:
        return                           # (no view: nothing saved, no day known)
    t = None
    q = log["prev"]
    if q is not None and has_fix(now) and not log["still"]["active"]:
        t = (int(q[0] * 1000000), int(q[1] * 1000000))
    if t != R.tail:
        R.tail = t
        R.moved = True
    if R.moved:
        R.moved = False
        _rt_project()
    if R.unsaved and (not _recent_s(R.saved, ROUTE_SAVE // 1000)
                      or (log["still"]["active"] and R.save_ok)):
        _rt_save(now)
    _tft_t[4] = 0

def _tft_state(now):
    """What the display shows: set_state's arguments."""
    try:
        _rt_tick(now)
    except Exception as e:
        _rt_bad(e)
    fix = bool(has_fix(now))
    level = None
    if wifi["state"] == "connected":
        level = _wifi_level(wifi["rssi"], _tft_b[2])
    _tft_b[2] = level
    left = up["left"] or 0
    if left:
        text = str(left) if left < 10000 else "%dk" % (left // 1000)
    else:
        text = "kész" if up["url"] else " "
    return (_Rt.view, _Rt.km, fix, _rcv.sv_used() or 0, _local_hm(), level, _gps_quality(fix),
            text, bool(left), _store_state(now), _tft_bat())

def tft_tick(now):
    """New values every TFT_STATE, a frame every TFT_FRAME; TFT_MAX_ERR errors
    of either in a row (each counted when it runs: most ticks do neither) turn
    the display dark for this boot, the logging goes on. Nothing leaves this:
    the handling itself allocates, out of a MemoryError too."""
    t = _tft_t
    if _tft is None:
        return
    try:
        if _tft.pressed():               # (every pass: a tap is short)
            if not _recent(now, _tft_l[2], TFT_WAKE):
                t[0] = None              # lit at once, not up to TFT_STATE later
            _tft_l[2] = now
    except Exception:                    # (no reason to lose the display)
        pass
    if not _recent(now, t[0], TFT_STATE):
        t[0] = now
        try:
            _tft_light(now)
            _tft.set_state(*_tft_state(now))
            t[2] = 0
        except Exception as e:
            t[2] += 1
            _tft_bad(2, e)
    if _tft is not None and not _recent(now, t[1], TFT_FRAME):
        t[1] = now
        try:
            _tft.frame()
            t[3] = 0
        except Exception as e:
            t[3] += 1
            _tft_bad(3, e)

def _tft_bad(i, e):
    global _tft
    gave_up = _tft_t[i] >= TFT_MAX_ERR
    tft = _tft
    if gave_up:
        _tft = None
    try:
        print("tft", repr(e))
        if gave_up:
            _diag_write("bug tft " + repr(e))
    except Exception:
        pass
    if gave_up:
        try:
            tft.off()
        except Exception:
            pass

# --- Main loop --------------------------------------------------------------

def run():
    """Log, and upload when possible, showing the state on the TFT."""
    global _wdt, _running, _boot_ok_done
    _keep_stop()
    _running = False
    _boot_ok_done = False
    try:
        # Arm the dog before any blocking init (count_points, WiFi scan).
        # The boot bypass (main.py's SKIP_PIN) never calls run(), so the
        # REPL stays free.
        if _wdt is None:
            _wdt = WDT(timeout=WDT_TIMEOUT)
        feed()
        _rcv.reset()
        log_reset()
        wifi_reset()
        up_reset()
        env = load_env(ENV_FILE)
        _diag_env(env)                   # (first: the other readers' .env
        _diag_boot()                     # mistakes go to the diag, _env_refused)
        _gps_sys_env(env)
        _min_move_env(env)
        _tft_env(env)
        _tft_start()                     # (at its light: .env is read)
        ota_boot()
        _rcv.gps_setup()
        wifi_start(env)                  # (the one read of .env)
        env = None
        _running = True
        _loop()
    except KeyboardInterrupt:
        _running = False
        _ota_mark("stop")                # (its WDT reset: not the build's)
        print("WDT armed - fed by timer; hold the boot-bypass pin and reset for mpremote")
        _wdt_keepalive()                 # (finally flushes and stops the WiFi)
        raise
    except Exception as e:
        _running = False
        if hasattr(sys, "print_exception"):
            sys.print_exception(e)
        else:
            print("crash", e)
        _diag_write("crash up %d %s" % (uptime_s(), repr(e)))
        _ota_mark("crash")               # (a pending build is bad)
        try:
            flush(_still_hold())  # held drift is dropped
        except Exception:
            pass
        _boot_fail_bump()
        time.sleep(1)
        machine.reset()
    finally:
        _running = False
        try:
            flush(_still_hold())  # held drift is dropped
        except Exception:
            pass
        wifi_stop()

TICK_PAUSE = 5000                        # ms a side tick that raised sits out
_tick_bug = {}                           # its name -> ticks of the bug
_tick_errs = {}                          # its name -> [bugs since boot, the
                                         # last one] (status.json, diag)

def _tick_bugs(sep=", ", full=True):
    """The guarded ticks' bugs since boot, "upload:1234 ValueError('x')"
    (full) or "upload:1234" each, None for none: the diag has one line a
    kind an hour, these say how often and what came last."""
    if not _tick_errs:
        return None
    return sep.join("%s:%d%s" % (k, v[0], " " + v[1] if full else "")
                    for k, v in sorted(_tick_errs.items()))

GUARD_MEM_MAX = 10                       # a tick's guarded MemoryErrors
GUARD_MEM_QUIET = 2400000                # within this gap of each other: a
                                         # memory reboot (_mem_reboot). 40 min,
                                         # over BACKOFF_MAX: a lasting one in a
                                         # backed-off tick (the upload) comes
                                         # at most that far apart and keeps
                                         # counting; sporadic ones further
                                         # apart start again at 1
GUARD_PEND_MAX = 5                       # with an OTA image pending: this many
                                         # of one tick raise (the build crashes
                                         # and rolls back). TICK_PAUSE apart,
                                         # 5 is over 20 s of one tick failing
                                         # for good; a transient one in the
                                         # first minutes (the heap at its
                                         # worst) stays under it
_guard_mem = {"wifi": [0, None], "upload": [0, None],   # name -> [count, Stamp
              "ota": [0, None], "web": [0, None],       # of the last one]:
              "ntp": [0, None], "status": [0, None],    # made here, not under
              "diag": [0, None], "vbat": [0, None],     # a MemoryError
              "keepalive": [0, None]}

def _guarded(name, fn, arg=None):
    """A side tick of the loop (WiFi, upload, OTA): a bug in it is logged
    (once a kind), the tick skipped for TICK_PAUSE (no retry every 20 ms;
    after a MemoryError a gc.collect) and the loop, the logging, goes on.
    It escaped into run()'s crash branch: a reset and a boot-fail count
    each time, and a persistent one left the board in the REPL after
    BOOT_FAIL_MAX boots, logging nothing. Not guarded: log_tick and the
    GPS reads (the logging itself: a reset is their recovery), and no bug
    while an OTA image is pending (its crash rolls it back). Nothing is
    fed here: a tick that hangs still fires the WDT.
    A MemoryError is no crash of a pending image at once, in any tick (the
    page's wrote /ota.mark and blamed a good build; an upload's or a scan's
    would the same, in the first minutes when the heap is at its worst):
    gc, the pause, and the memory reboot waits (it would abort the image).
    It is counted per tick (_guard_mem), not cleared by a pass that runs
    through (an upload in its backoff, an idle WiFi or OTA tick are no
    proof the memory is back): GUARD_MEM_MAX within GUARD_MEM_QUIET of each
    other are a memory reboot, GUARD_PEND_MAX with an image pending raise
    (a persistent one is the build's defect)."""
    at = _tick_bug.get(name)
    if at is not None:
        if _age(time.ticks_ms(), at) < TICK_PAUSE:
            return
        del _tick_bug[name]
    pend = ota["pend"]                   # (before: the OTA health path ends
    try:                                 # the pending state, then resets)
        if arg is None:
            fn()
        else:
            fn(arg)
    except Exception as e:
        pending_ota = ((pend is not None or ota["pend"] is not None)
                       and not ota.get("confirmed"))
        mem = isinstance(e, MemoryError)
        if pending_ota and not mem:
            raise
        now = time.ticks_ms()
        if mem:
            gc.collect()
            # the crash reset was a lasting MemoryError's recovery (the
            # heap fragmented for good)
            c = _guard_mem.get(name)
            if c is None:
                c = _guard_mem[name] = [0, None]
            if not _recent_s(c[1], GUARD_MEM_QUIET // 1000):
                c[0] = 0
            c[0] += 1
            c[1] = _stamp()
            if pending_ota:
                if c[0] >= GUARD_PEND_MAX:
                    raise
            elif c[0] >= GUARD_MEM_MAX:
                n, c[0] = c[0], 0
                if not _reboot_for_memory("guarded memory " + name, home=False):
                    c[0] = n             # (not now: it goes on counting)
        if name == "upload":
            # (after the POST, before the commit: the same batch went up
            # again every TICK_PAUSE for ever)
            _schedule(now)
        try:                             # (the telling allocates: it must not
            _tick_bug[name] = now        # turn a MemoryError into a crash)
            c = _tick_errs.get(name)
            if c is None:
                c = _tick_errs[name] = [0, ""]
            c[0] += 1
            c[1] = repr(e)[:60]
            print("bug", name, c[1])
            _bug_once("bug %s %s" % (name, c[1]), (name, type(e)))
        except MemoryError:
            pass

def _loop():
    global _wdt
    if _wdt is None:
        _wdt = WDT(timeout=WDT_TIMEOUT)
    loop_start = time.ticks_ms()
    status_at = loop_start
    diag_at = loop_start
    while True:
        feed()
        uptime_s()                       # sums ticks before they wrap
        _rcv.read_uart()
        _rcv.gps_check(time.ticks_ms())
        log_tick()
        _guarded("wifi", wifi_tick)
        _guarded("keepalive", keepalive_tick, time.ticks_ms())
        feed()
        # NTP before the upload: on the first connect without a fix the
        # board clock is unset, and a verified POST checked the certificate
        # against 2000-01-01 ("future": a handshake, an error, the backoff)
        _guarded("ntp", ntp_tick, time.ticks_ms())
        _guarded("upload", upload_tick)
        _guarded("web", web_tick)
        _guarded("ota", ota_tick, time.ticks_ms())
        now = time.ticks_ms()
        tft_tick(now)
        if (not _boot_ok_done
                and time.ticks_diff(now, loop_start) >= BOOT_OK_MS):
            _boot_ok()
        if time.ticks_diff(now, status_at) >= STATUS:
            _guarded("status", _status)
            # a refused drop-back is only retried here once no POST runs
            # (all sent, radio asleep, no GEO_URL): FREQ_RETRY gates it
            _set_freq(CPU_MHZ)
            status_at = now
        if DIAG and time.ticks_diff(now, diag_at) >= DIAG:
            _guarded("diag", _diag, now)
            diag_at = now
        _guarded("vbat", vbat_tick, now)
        time.sleep_ms(20)
