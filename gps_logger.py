from machine import Pin, UART, WDT
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

# Collect garbage early and often. When an allocation doesn't fit, the ESP32
# port grows the Python heap with the largest free IDF block (~60 kB) instead,
# and TLS (mbedTLS lives in the IDF heap) then fails with ENOMEM.
gc.threshold(8192)

BAUD = 115200                            # the UC6580's
GPS_HZ = 5                               # fixes a second; one averaged point
                                         # a second is kept
SECONDS_Q = 8                            # published seconds kept for log_tick
GPS_MAX_STD = 20                         # m: receiver's own error estimate
                                         # (GST) above this = not usable
GPS_RETRY = 5000                         # ms: setup sent again if 1 Hz
GPS_RETRY_MAX = 300000                   # sentences still come this long after
GPS_IGNORED_MAX = 60000                  # ms cap between setups a receiver
                                         # answers but does not take
GPS_VERIFY = 10000                       # ms after an answered setup:
                                         # still one fix a second, or no GST
                                         # = not taken, set up again
GPS_VERIFY_MAX = 1800000                 # ms it waits for 3 valid seconds to
                                         # tell (none before the first fix)
GPS_NOT_TAKEN_SAY = 600000               # ms between "setup not taken" diag
                                         # lines (the ones between counted)
AVG_MAX_SPEED = 10                       # km/h: above this a second's point
                                         # is its first fix, not the mean (the
                                         # mean lies ~0.4 s along the track:
                                         # 6 m at 50 km/h, stamped the second)
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
WEB_AWAY = False                         # .env WEB_AWAY=1: keep the radio on
                                         # for the page on the other networks too
SV_POLL = 60000                          # ms: a satellite list younger than
                                         # two of these gives the used count
GPS_LOOP_WINDOW = 10000                  # ms between looks at the stamps
GPS_FLAP_RESTARTS = 5                    # full restarts (NMEA in between)
GPS_FLAP_WINDOW = 600000                 # within this many ms = a reset loop
GPS_STORM_WAIT = 30000                   # ms between setups while it loops:
                                         # each setup blocks the loop seconds
GPS_STORM_QUIET = 10000                  # ms without a restart (in a storm
                                         # twice its last gap, up to
                                         # GPS_STORM_WAIT): the storm is
                                         # over, set it up within GPS_RETRY

# The Heltec Wireless Tracker (ESP32-S3, boards/GPS_LOGGER_S3): its UC6580 is on
# the board, UART1 on GPIO34 (TX) / GPIO33 (RX), powered by the Vext rail (GPIO3
# high: the receiver, its LNA and the TFT). gps_tft.py drives the TFT once the
# logger runs (dark until then). main.py keeps the boot-bypass pin.
HELTEC_VEXT = 3                          # high: the rail of the UC6580
HELTEC_TFT_BL = 21                       # the TFT's backlight (given to gps_tft.start)
GPS_UART, GPS_TX, GPS_RX = 1, 34, 33
Pin(HELTEC_VEXT, Pin.OUT, value=1)
Pin(HELTEC_TFT_BL, Pin.OUT, value=0)

# (rides out HTTP calls; at 115200 the UC6580 sends ~2.3 KB/s: a keep-alive
# scan's pause overran 4 KB, one cut sentence per pulse, measured)
uart = UART(GPS_UART, baudrate=BAUD, tx=GPS_TX, rx=GPS_RX, rxbuf=8192)

NO_DATA = 3000                           # ms of silence = "No data!"
GPS_SILENT = 60000                       # ms without a byte: a diag line

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
STALE = 2000                             # ms: older position = no fix
GST_HOLD = 6000                          # ms a GST error estimate is used
                                         # for the seconds without one (1 Hz:
                                         # GST only every 5th second)
LOG_DIR = "/gpx"
HIST_DIR = "/hist"                       # the day history the page draws:
HIST_DAYS = 7                            # each written line, a .trb a UTC
                                         # day, the last HIST_DAYS kept
HIST_REPLY = 16384                       # text bytes about per /hist reply
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

# --- Ticks ------------------------------------------------------------------
# ticks_ms wraps every 2**30 ms and ticks_diff is signed, so a timestamp older
# than ~6.2 days reads as being in the future. Ages and deadlines go through
# these helpers: a wrapped past timestamp counts as very old, a deadline that
# lies further ahead than it could ever have been scheduled counts as due.

TICKS_OLD = 1 << 29                      # "very old" for a wrapped age
TICKS_SKEW = 1000                        # ms a timestamp may be "ahead"
TICKS_CAP = 86400000                     # ms after which gps_check forgets
                                         # a receiver timestamp (it would
                                         # read as fresh again after 2**30)

def _recent(now, t, ms):
    """t (ticks, or None: never) is under ms old."""
    return t is not None and _age(now, t) < ms

def _age(now, t):
    """ms since ticks t. A timestamp taken just after now (say read_uart()
    ran between the two ticks_ms() calls) is age 0; a wrapped (large
    negative) difference is TICKS_OLD."""
    d = time.ticks_diff(now, t)
    if d >= 0:
        return d
    return 0 if d > -TICKS_SKEW else TICKS_OLD

def _waiting(now, at, cap):
    """True while deadline at (scheduled at most cap ms ahead) is ahead."""
    d = time.ticks_diff(at, now)
    return 0 < d <= cap

# --- NMEA parsing -----------------------------------------------------------
# The receiver runs at GPS_HZ fixes a second (gps_setup). The fixes of one
# UTC second are averaged, and the mean is published to st when the next
# second starts: st always holds the last complete second, so the logger
# records one point a second, the mean of up to GPS_HZ fixes. Dawarich keeps
# whole seconds only and orders the points of one second by longitude, so
# more than one point a second would zigzag there.

st = {}
ep = {}                                  # the second being collected

def reset():
    st.clear()
    st.update(start=time.ticks_ms(), rx=None, rx_b=0, good=None,
              ok=0, err=0,
              quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, used=None, view={},
              hdop=None, ttff=None, pos_at=None, hdop_at=None,
              seq=0, epochs=0, extra=0, setup_at=None, mute_at=None,
              q_thin=0,
              setup_wait=GPS_RETRY, setup_ok=None, std=None, std_at=None,
              gps_boots=0, q=[], loops=0,
              loop_at=time.ticks_ms(), boot_t=[],
              storm_at=None, boot_at=None, boot_last=None, boot_gap=0,
              svs=None, svs_at=None,
              gps_ver=None, gsa_at=None, gsa_n=0, uc_ok=0, uc_fail=0, uc_sys=None,
              uc_sys_sent=False, fix_first=None, settled=False, gst_said=None,
              ack_n=0, rate_nak=False, checked=True,
              hz1=0, hz5=0,
              set_ok=0, gst_at=None,
              nt_said=None, nt_more=0,
              set_boots=0)
    _ep_new(None)

def _ep_new(t):
    # g / r: GGA / RMC positions as [ref lat, ref lon, sum dlat, sum dlon, n]
    # (offsets from the second's first fix: single-precision floats cannot
    # add up latitudes to the metre); alt / hdop: [sum, n]
    ep.clear()
    ep.update(t=t, date=None, g=None, r=None, alt=[0.0, 0], hdop=[0.0, 0],
              std=None, spd=None)

# The XOR checksum misses two flipped bits: a corrupt sentence can pass it
# (fuzzed: lat 1e37 in st). A number not in its NMEA form is a ValueError,
# and handle() counts the sentence bad; the handlers read every field
# before they keep any of it.

def _num(s, conv=float):
    """A number field, None when empty. A float must be plain decimal:
    "1e9", "nan", "inf" are never NMEA (ValueError; looked for in the
    text, no float made for it)."""
    if not s:
        return None
    if conv is float and ("e" in s or "E" in s or "n" in s or "N" in s):
        raise ValueError(s)
    return conv(s)

def _deg(v, hemi, ax):
    """NMEA ddmm.mmmm + N/S (ax "NS", 90 degrees at most) / dddmm.mmmm +
    E/W (ax "EW", 180) -> signed decimal degrees, None when empty. Else
    digits for the degrees, minutes 0 to under 60 without an exponent,
    the hemisphere one of ax (ValueError). No copy of v to check it: the
    two slices are the ones parsed."""
    if not v or not hemi:
        return None
    i = v.find(".")
    d = (i if i >= 0 else len(v)) - 2
    if d < 1 or len(hemi) != 1 or hemi not in ax:
        raise ValueError(v)
    a, mm = v[:d], v[d:]
    m = float(mm)                        # (0 <= m < 60 also ends nan/inf)
    if not a.isdigit() or not 0 <= m < 60 or "e" in mm or "E" in mm:
        raise ValueError(v)
    deg = int(a) + m / 60
    if deg > (90 if ax == "NS" else 180):
        raise ValueError(v)
    return -deg if hemi == ax[1] else deg

def _position(f, i):
    """The sentence's (lat, lon), or None when its fields are empty."""
    lat, lon = _deg(f[i], f[i + 1], "NS"), _deg(f[i + 2], f[i + 3], "EW")
    if lat is None or lon is None:
        return None
    return lat, lon

def _acc(key, pos):
    a = ep[key]
    if a is None:
        ep[key] = [pos[0], pos[1], 0.0, 0.0, 1]
    else:
        a[2] += pos[0] - a[0]
        a[3] += _dlon(pos[1] - a[1])     # (across 180° too: E and W fixes
                                         # of a second averaged to ~36°)
        a[4] += 1

_sec = [None, None]                      # HHMMSS last seen, its "HH:MM:SS"
_day = [None, None]                      # DDMMYY last seen, its YYYY-MM-DD

def _second(s):
    """Start the sentence's UTC second (HHMMSS.ss, the fraction ignored);
    a new second publishes the one before. Its "HH:MM:SS" is made once a
    second, not for each sentence of each epoch (5 allocations each)."""
    if len(s) < 6:
        return
    c = _sec
    if c[0] is None or not s.startswith(c[0]):
        h = s[0:6]
        if not h.isdigit():              # a corrupt digit (see _num): the
            raise ValueError(s)          # sentence is bad; one such time in
        c[0] = h                         # a line stopped the track for good
        c[1] = "%s:%s:%s" % (h[0:2], h[2:4], h[4:6])
    t = c[1]
    if t != ep["t"]:
        if ep["t"] is not None:
            _publish()
        _ep_new(t)

def _publish():
    """Make the finished second the current fix: the mean of its valid GGA
    positions (RMC ones when it had no GGA), or its first fix when it moved
    faster than AVG_MAX_SPEED. A second without one is no fix. Only RMC
    carries the date: a second that runs back past midnight without its own
    RMC has none, or its fix would be dated a day back."""
    t, date = ep["t"], ep["date"]
    if date is None and st["time"] is not None and t < st["time"]:
        st["date"] = None
    elif date is not None:
        st["date"] = date
    st["time"] = t
    now = time.ticks_ms()
    a = ep["g"] or ep["r"]
    st["valid"] = a is not None
    if a is not None and st["fix_first"] is None:
        st["fix_first"] = now
    st["epochs"] = a[4] if a else 0
    if a is None:
        return
    n = a[4]
    if not st["checked"]:                # _setup_check: did GPS_HZ take?
        if n > 1:
            st["hz5"] += 1
        else:
            st["hz1"] += 1
    spd = ep["spd"]
    if n > 1 and spd is not None and spd > AVG_MAX_SPEED:
        # moving: the mean of the second lies ~0.4 s along the track but is
        # stamped with the second, so the point is the second's first fix
        st["lat"], st["lon"] = a[0], a[1]
    else:
        st["lat"], st["lon"] = a[0] + a[2] / n, _dlon(a[1] + a[3] / n)
    alt, hd = ep["alt"], ep["hdop"]
    st["alt"] = alt[0] / alt[1] if alt[1] else None
    if hd[1]:
        st["hdop"], st["hdop_at"] = hd[0] / hd[1], now
    elif not _recent(now, st["hdop_at"], STALE):
        st["hdop"], st["hdop_at"] = None, now
    # (a second without a GGA HDOP, RMC only: the last one and its stamp
    # are kept, _fresh_hdop holds them STALE ms: an unknown HDOP passes
    # _usable, a lost GGA turned the MAX_HDOP gate off)
    if ep["std"] is not None:            # (a second without GST keeps the
        st["std"], st["std_at"] = ep["std"], now   # last one: _fresh_std)
    _published(now)

def _published(now):
    """The second in st is a fix: stamp it and queue it for log_tick."""
    st["pos_at"] = now
    st["seq"] += 1
    if st["ttff"] is None:
        st["ttff"] = time.ticks_diff(now, st["start"]) // 1000
    # queued for log_tick: after a blocking POST, scan or setup read_uart
    # publishes several seconds at once, and each is a point. Full (a POST
    # over a weak hotspot blocks up to HTTP_MAX): the second whose
    # neighbours are closest goes, oldest first. The newest were kept
    # before, so 40 s blocked was a 32 s gap with no segment break; now
    # SECONDS_Q seconds are spread over the whole block, same RAM
    q = st["q"]
    q.append(_snap())
    if len(q) > SECONDS_Q:
        k, g = 1, None
        for i in range(1, len(q) - 1):
            d = q[i + 1][0] - q[i - 1][0]
            if g is None or d < g:
                k, g = i, d
        del q[k]
        st["q_thin"] += 1

# a published second as a tuple in this order (seq first, pos_at at 8): one
# small allocation a second rather than a dict
_SNAP = ("seq", "lat", "lon", "alt", "hdop", "hdop_at", "std", "std_at",
         "pos_at", "time", "date", "valid", "epochs")

def _snap():
    return tuple(st[k] for k in _SNAP)

def _load(snap):
    for i in range(len(_SNAP)):
        st[_SNAP[i]] = snap[i]

# GGA quality 6 / RMC mode E: an estimate (dead reckoning, the last fix
# carried on), no fix. GGA 1-5 are (GPS, DGPS, PPS, RTK fixed / float)
GGA_FIX_MAX = 5
GSA_EVERY = 5                            # s between GSA (fix type) rounds
GSA_MAX = 8                              # GSA in a second beyond this: the
                                         # receiver did not take its rate

def rmc(talker, f):
    kn, pos = _num(f[7]), _position(f, 3)     # (all read first: _num)
    d = f[9]
    date = None
    # (a UC6580 without a fix sends a made-up date: 170126, measured)
    if len(d) == 6 and f[2] == "A":
        c = _day                         # (made once a day, as _second)
        if d != c[0]:
            if not d.isdigit():          # (a corrupt digit: bad sentence)
                raise ValueError(d)
            c[0], c[1] = d, "20%s-%s-%s" % (d[4:6], d[2:4], d[0:2])
        date = c[1]
    _second(f[1])
    st["speed"] = None if kn is None else kn * 1.852
    if kn is not None and (ep["spd"] is None or st["speed"] > ep["spd"]):
        ep["spd"] = st["speed"]          # the second's top speed
    if date is not None:
        ep["date"] = date
    if f[2] == "A" and pos and (len(f) < 13 or f[12] != "E"):
        _acc("r", pos)

def gga(talker, f):
    q, n, pos = _num(f[6], int), _num(f[7], int), _position(f, 2)
    alt, hd = _num(f[9]), _num(f[8])     # (all read first: _num)
    _second(f[1])
    st["quality"], st["used"] = q, n
    if 0 < (q or 0) <= GGA_FIX_MAX and pos:
        _acc("g", pos)
        if alt is not None:
            ep["alt"][0] += alt
            ep["alt"][1] += 1
        if hd is not None:
            ep["hdop"][0] += hd
            ep["hdop"][1] += 1

def gsa(talker, f):
    """Fix type. One per system and round on a multi-GNSS receiver; more
    than GSA_MAX in a second means the GSA rate of the setup was lost:
    gps_check sets it up again."""
    st["fixtype"] = _num(f[2], int)
    now = time.ticks_ms()
    if not _recent(now, st["gsa_at"], 1000):
        st["gsa_at"], st["gsa_n"] = now, 0
    st["gsa_n"] += 1
    if st["gsa_n"] == GSA_MAX + 1:
        _extra(talker, f)
    _uc_gsa(talker, f)

def _extra(talker, f):
    """A sentence gps_setup turns off: the receiver did not take it."""
    at = st["setup_at"]
    if at is not None and _age(time.ticks_ms(), at) >= 1000:
        st["extra"] += 1

def gsv(talker, f):
    _uc_gsv(talker, f)                   # (its GSV stays on: not _extra)
    n = _num(f[3], int)
    if n is not None:
        st["view"][talker] = n          # per constellation, summed on display

def gst(talker, f):
    """The receiver's own error estimate (1-sigma, metres) of the second's
    position: the larger of latitude and longitude. A cold start converging
    shows here while its HDOP already looks good."""
    sl, sn = _num(f[6]), _num(f[7])      # (read first: _num)
    _second(f[1])
    st["gst_at"] = time.ticks_ms()       # it comes (_setup_check)
    if sl is not None and sn is not None:
        ep["std"] = sl if sl > sn else sn

def _gps_restarted():
    """The receiver restarted (power dip, reset), back at its defaults."""
    st["gps_boots"] += 1
    st["extra"] += 1                     # back at its defaults: set it up
    # again within GPS_RETRY, not 5 min; while it keeps restarting (a
    # reset loop in the last GPS_FLAP_WINDOW), every GPS_STORM_WAIT:
    # 2026-10-30 had 541 restarts, each setup blocking the loop
    now = time.ticks_ms()
    st["boot_at"] = now
    last = st["boot_last"]               # the gap sets the storm's quiet
    st["boot_gap"] = 0 if last is None else _age(now, last)
    st["boot_last"] = now
    print("gps restarted")
    _gps_flap(now)                       # first: the restart that makes
    storm = (st["storm_at"] is not None  # it a storm waits too
             and _age(now, st["storm_at"]) < GPS_FLAP_WINDOW)
    st["setup_wait"] = GPS_STORM_WAIT if storm else GPS_RETRY
    st["ack_n"] = 0                      # at its defaults: a fresh start
    # the model asked anew (_uc_setup); the settle rule anew, and not the
    # second it was in: published by the next sentence, its fixes would start
    # the rule before a real fix
    st.update(svs=None, svs_at=None, gps_ver=None, fix_first=None,
              settled=False)
    _ep_new(None)
    _uc_sv.update(new={}, open=False, until=None, used=[], gsa=None)

def _gps_flap(now):
    """A receiver that restarts every few seconds but sends NMEA in between
    (seen on the board after a flash: 51 restarts in 5 min, each after a
    full banner). GPS_FLAP_RESTARTS restarts within GPS_FLAP_WINDOW -> a
    warning and a count, then the next ones are counted afresh (one warning
    per GPS_FLAP_RESTARTS restarts at most)."""
    t = st["boot_t"]
    t.append(now)
    if len(t) > GPS_FLAP_RESTARTS:
        t.pop(0)
    if len(t) == GPS_FLAP_RESTARTS and _age(now, t[0]) <= GPS_FLAP_WINDOW:
        st["loops"] += 1
        st["storm_at"] = now
        print("gps reset loop?", len(t), "restarts in",
              _age(now, t[0]) // 1000, "s: check the GPS power")
        _diag_write("gps reset loop %d restarts in %d s" % (
            len(t), _age(now, t[0]) // 1000))
        del t[:]

HANDLERS = {"RMC": rmc, "GGA": gga, "GSA": gsa, "GSV": gsv, "GST": gst,
            "GLL": _extra, "VTG": _extra}

GNSS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS",
        6: "GLONASS"}
def sv_used():
    """Satellites in the fix: from a satellite list younger than two
    SV_POLL, else GGA."""
    at = st["svs_at"]
    if (st["svs"] is not None and at is not None
            and _age(time.ticks_ms(), at) < 2 * SV_POLL):
        return len([v for v in st["svs"] if v[4]])
    return st["used"]

def sv_seen():
    """Satellites with a signal: from the last satellite list, else the GSV
    count."""
    if st["svs"] is not None:
        return len([v for v in st["svs"] if v[1] > 0])
    n = 0
    for c in st["view"].values():
        n += c
    return n

# --- UC6580 ----------------------------------------------------------------
# The Heltec Wireless Tracker's Unicore UC6580 speaks no UBX: text commands
# ("$CFGMSG,0,7,5*cs"), each answered "$OK*04" or "$FAIL" (UFirebird II
# protocol specification; measured on the board: ~11 ms). Nothing is saved in
# the receiver, so a power cycle brings its defaults back: 1 Hz with GGA, RMC,
# GSA, GSV and TXT, no GST, 11 KB/10 s; all systems and bands on (CFGSYS
# H35155), which stays: setting it restarts the receiver. It prints a plain
# text banner at start-up. Its GSV carries the satellite list, one sentence set
# per signal.
UC_ACK_WAIT = 300                        # ms for a command's $OK
UC_SETTLE = 60000                        # ms after its first fix (_usable)
UC_SETTLE_KMH = 15                       # at this speed a point counts at once
UC_GST_LOST = 30000                      # ms without GST: its setup is gone
UC_SYS = None                            # .env GPS_SYS: its systems and bands
                                         # (CFGSYS mask, hex), None = its own
GSV_EVERY = 5                            # s between satellite lists
UC_GSA_GAP = 150                         # ms without a GSA: the next is a new
                                         # round (fixes come 200 ms apart)

def _uc_msgs():
    """The setup. The rate first: CFGNAV resets the GSV rate (its page). A
    message's rate is in fixes: Rate = 1000 / NavRate / output frequency."""
    ms = 1000 // GPS_HZ
    # (DRNavRate is unused here: what the receiver reports, 1000)
    return ("CFGNAV,%d,%d,1000" % (ms, ms), "CFGMSG,0,0,1", "CFGMSG,0,4,1",
            "CFGMSG,0,1,0", "CFGMSG,0,5,0", "CFGMSG,0,2,%d" % (GPS_HZ * GSA_EVERY),
            "CFGMSG,0,3,%d" % (GPS_HZ * GSV_EVERY),
            "CFGMSG,0,7,%d" % GPS_HZ, "CFGMSG,6,0,0", "CFGMSG,6,1,0")

def _uc_cmd(cmd):
    """One command with its checksum: True for $OK, None for $FAIL (it
    answers, but refuses), False for silence after UC_ACK_WAIT. A question
    (no comma) goes without a checksum: with one it gets no answer (measured
    on PDTINFO and CFGSYS)."""
    c = 0
    for ch in cmd:
        c ^= ord(ch)
    read_uart()
    ok, bad = st["uc_ok"], st["uc_fail"]
    uart.write(("$%s\r\n" % cmd if "," not in cmd
                else "$%s*%02X\r\n" % (cmd, c)).encode())
    t0 = time.ticks_ms()
    while True:
        read_uart()
        if st["uc_fail"] != bad:
            return None
        if st["uc_ok"] != ok:
            return True
        if _age(time.ticks_ms(), t0) >= UC_ACK_WAIT:
            return False
        time.sleep_ms(2)

def _uc_setup():
    """gps_setup on a UC6580: True when it answered every command. A silent
    one is left alone until gps_check retries; one refused (no use sending
    it again) is said once a setup, and a refused rate (st["rate_nak"]) makes
    1 fix/s no "not taken" verdict."""
    try:
        if st["gps_ver"] is None:
            _uc_cmd("PDTINFO")           # (its answer: _uc_line)
        if UC_SYS is not None and not st["uc_sys_sent"]:
            # asked first (only a mask it does not have, and once a boot:
            # setting one restarts the receiver, and that restart brings
            # this setup back)
            _uc_cmd("CFGSYS")
            if st["uc_sys"] not in (None, UC_SYS):
                st["uc_sys_sent"] = True
                mask = "h%X" % UC_SYS
                r = _uc_cmd("CFGSYS," + mask)
                if r is True:            # it restarts: the setup comes back
                    _say("gps systems " + mask)
                    return True
                if r is None:            # refused: its own stay, and the rest
                    print("gps refused", mask)
                    _diag_write("gps refused CFGSYS," + mask)
                else:
                    return False
        for cmd in _uc_msgs():
            r = _uc_cmd(cmd)
            if r is False:
                return False
            if r is None:
                _say("gps refused " + cmd)
                if cmd.startswith("CFGNAV"):
                    st["rate_nak"] = True
        feed()
        return True
    except Exception as e:
        print("gps setup", repr(e))
        return False

def _uc_line(p):
    """A sentence no NMEA handler reads (the view between "$" and "*")."""
    n = len(p)
    if n == 2 and p[0] == 79 and p[1] == 75:      # "OK"
        st["uc_ok"] += 1
    elif n >= 4 and bytes(p[:4]) == b"FAIL":
        st["uc_fail"] += 1
    elif n > 7 and bytes(p[:7]) == b"CFGSYS,":
        try:
            st["uc_sys"] = int(str(p[8:], "ascii"), 16)
        except ValueError:
            pass
    elif n > 8 and bytes(p[:8]) == b"PDTINFO,":
        f = str(p, "ascii").split(",")
        if len(f) > 4:
            st["gps_ver"] = "%s %s, %s" % (f[1], f[2], f[4])

_UC_GNSS = {"GP": 0, "GA": 2, "GB": 3, "GQ": 5, "GL": 6}  # (GNSS's numbers)
_UC_SYS = {1: 0, 2: 6, 3: 2, 4: 3, 5: 5}  # GSA's system id -> gnssId
_uc_sv = {"new": {}, "open": False, "until": None, "used": [], "gsa": None}

def _uc_gsv_skip():
    """True while a list is fresh: with a fix its GSV comes at 1 Hz whatever
    the rate (25 is read back; ~25 sentences a second, measured), each
    decoded and split into ~20 strings for nothing."""
    u = _uc_sv
    if u["until"] is None:
        return False
    if time.ticks_diff(u["until"], time.ticks_ms()) > 0:
        return True
    u["until"] = None
    return False

def _uc_gsa(talker, f):
    """One GSA: the satellites of its system in the fix. A pause before it
    starts a new list: the sentences of a round come in one burst, and a
    round every second by the clock cut one in two (11 seen, 6 in use).
    The system is the last field (NMEA 4.10); before it (18 fields) the
    talker's, and none for GN (mixed): that one adds nothing."""
    n = len(f)
    if n < 18:
        return
    u = _uc_sv
    now = time.ticks_ms()
    if not _recent(now, u["gsa"], UC_GSA_GAP):
        u["used"] = []
    u["gsa"] = now
    g = _UC_SYS.get(_num(f[-1], int)) if n > 18 else _UC_GNSS.get(talker)
    if g is None:
        return
    for x in f[3:15]:
        v = _num(x, int)
        if v is not None:
            _uc_sv["used"].append(g << 8 | v)

def _uc_gsv(talker, f):
    """One GSV sentence into a round's list (the same satellite on two
    bands: its strongest). A round starts with GPS's first sentence; the
    next such one closes it as st["svs"]: (svid, cno, elevation, azimuth,
    used, unhealthy, gnssId), and the next GSV_EVERY s of GSV are not even
    decoded (_uc_gsv_skip)."""
    g = _UC_GNSS.get(talker)
    if g is None:
        return
    u = _uc_sv
    first = g == 0 and f[2] == "1" and f[-1] == "1"
    if first and u["open"]:
        # an empty round too: no signal at all, else the last sky stays
        used = u["used"]
        st["svs"] = [(k & 255, v[0], v[1], v[2], k in used, False, k >> 8)
                     for k, v in u["new"].items()]
        st["svs_at"] = time.ticks_ms()
        u["new"], u["open"] = {}, False
        u["until"] = time.ticks_add(time.ticks_ms(), GSV_EVERY * 1000)
        return
    if first:
        u["open"] = True
    elif not u["open"]:
        return                           # (the rest of a round begun before)
    for i in range(4, len(f) - 3, 4):
        n = _num(f[i], int)
        if n is None:
            continue
        k = g << 8 | n
        cno = _num(f[i + 3], int) or 0
        v = u["new"].get(k)
        if v is None or cno > v[0]:
            u["new"][k] = (cno, _num(f[i + 1], int), _num(f[i + 2], int) or 0)

def gps_setup():
    """Ask the receiver for GPS_HZ fixes a second (_uc_setup). Not saved in
    the receiver, so it is sent at every boot; gps_check sends it again while
    the receiver still sends the sentences it turns off."""
    read_uart()                          # a banner in it came before
    boots = st["gps_boots"]
    st["setup_at"], st["extra"], st["rate_nak"] = time.ticks_ms(), 0, False
    _setup_done(_uc_setup(), boots)

def _setup_done(ok, boots):
    """What a setup leaves for gps_check."""
    # sentences sent while the setup ran are not a receiver that ignored
    # it; a restart meanwhile (its banner was read) is: set up again
    st.update(setup_at=time.ticks_ms(),
              extra=0 if st["gps_boots"] == boots else 1, setup_ok=ok,
              checked=not ok, hz1=0, hz5=0, set_ok=st["ok"],
              set_boots=st["gps_boots"])
    if ok and st["gps_boots"] == boots:
        # it listens: a later revert is quick. One that answers but keeps
        # sending what it turned off (gps_check counts ack_n) less and less
        # often: the first retry after GPS_RETRY, then doubling. (A restart
        # read during the setup set its own wait: _gps_restarted, a storm's
        # GPS_STORM_WAIT, kept.) ack_n counts on while it ignores (43200
        # after a month): the shift stops at 4, past GPS_IGNORED_MAX already,
        # not a long int a setup
        n = min(st["ack_n"], 5)
        st["setup_wait"] = min(GPS_RETRY << (n - 1 if n > 1 else 0),
                               GPS_IGNORED_MAX)
    print("gps setup", "ok" if ok else "no ack")

def _setup_check(now):
    """GPS_VERIFY after an answered setup: did it take? A receiver may answer
    and ignore. Valid seconds all at one fix (GPS_HZ not taken), or NMEA
    coming but no GST (the GPS_MAX_STD filter would be off without a word):
    counted as a sentence it should not send, so gps_check sets it up again.
    Only valid seconds tell: before 3 of them (no fix yet: a setup at boot,
    or indoors) there is no verdict yet, it waits up to GPS_VERIFY_MAX. Taken
    (the backoff starts afresh) only with GPS_HZ fixes a second and GST
    seen."""
    if st["checked"]:
        return
    age = _age(now, st["setup_at"])
    if age < GPS_VERIFY:
        return
    if st["gps_boots"] != st["set_boots"]:
        return                           # restarted (back at 1 Hz): its own
                                         # setup comes
    hz1, hz5 = st["hz1"], st["hz5"]
    if hz1 + hz5 < 3:
        if age >= GPS_VERIFY_MAX:
            st["checked"] = True         # no fix all this time: no verdict
            print("gps setup unverified: no fix")
        return
    st["checked"] = True
    slow = GPS_HZ > 1 and not hz5 and not st["rate_nak"]
    g = st["gst_at"]
    gst = g is not None and time.ticks_diff(g, st["setup_at"]) >= 0
    no_gst = not gst and st["ok"] - st["set_ok"] >= 10
    if slow or no_gst:
        st["extra"] += 1
        why = "1 fix/s" if slow else "no GST"
        print("gps setup not taken:", why)
        _not_taken(now, why)
    elif gst and (hz5 or GPS_HZ == 1 or st["rate_nak"]) and not st["extra"]:
        st["ack_n"] = 0                  # it took this one

def _not_taken(now, why):
    """A diag line for a setup that was not taken, one per GPS_NOT_TAKEN_SAY
    (the ones between counted): a session left at 1 Hz leaves a trace."""
    said = st["nt_said"]
    if _recent(now, said, GPS_NOT_TAKEN_SAY):
        st["nt_more"] += 1
        return
    more = st["nt_more"]
    st["nt_said"], st["nt_more"] = now, 0
    _diag_write("gps setup not taken: " + why
                + (" (%d more)" % more if more else ""))

_STAMPS = ("rx", "good", "pos_at", "hdop_at", "std_at", "gsa_at",
           "gst_at", "boot_at", "boot_last", "nt_said", "gst_said")

def _stamps_old(now):
    """ticks_ms wraps every 2**30 ms, so a timestamp left alone for 12.4
    days reads as fresh again (and from 6.2 days as TICKS_OLD). Every
    GPS_LOOP_WINDOW: the storm mark and the restart times end with
    GPS_FLAP_WINDOW, the receiver's other stamps (and a satellite list)
    with TICKS_CAP; setup_at (None = never set up) stays TICKS_CAP old."""
    t = st["storm_at"]
    if t is not None and _age(now, t) >= GPS_FLAP_WINDOW:
        st["storm_at"] = None
    t = st["boot_t"]
    while t and _age(now, t[0]) > GPS_FLAP_WINDOW:
        t.pop(0)
    for k in _STAMPS:
        t = st[k]
        if t is not None and _age(now, t) >= TICKS_CAP:
            st[k] = None
    t = st["svs_at"]
    if t is not None and _age(now, t) >= TICKS_CAP:
        st["svs"] = st["svs_at"] = None
    # a first fix this old is long past UC_SETTLE: settled, before the raw
    # ticks_diff in _usable wraps negative (6.2 days) and holds it unsettled
    t = st["fix_first"]
    if not st["settled"] and t is not None and _age(now, t) >= TICKS_CAP:
        st["settled"] = True
    t = st["setup_at"]
    if t is not None and _age(now, t) > TICKS_CAP:
        st["setup_at"] = time.ticks_add(now, -TICKS_CAP)

def _gps_mute(now):
    """No byte from the receiver for GPS_SILENT: one diag line when it starts,
    one when bytes come again, with how long it lasted. mute_at: the last
    byte (the boot when none came)."""
    rx = st["rx"]
    at = st["start"] if rx is None else rx
    m = st["mute_at"]
    if m is None:
        if _age(now, at) >= GPS_SILENT:
            st["mute_at"] = at
            _diag_write("gps silent %d s, no bytes from the receiver"
                        % (_age(now, at) // 1000))
    elif _recent(now, rx, GPS_SILENT):
        st["mute_at"] = None
        _diag_write("gps back after %d s" % (_age(rx, m) // 1000))

_usb_in = None

def _usb_drain():
    """The ESP32-S3's USB Serial/JTAG: a byte sent while the board boots
    (mpremote right after a flash) stays in its FIFO, MicroPython's start-up
    clears the interrupt that would read it, and until something reads the
    host's next bytes, its Ctrl-C too, wait: a logger no mpremote could stop
    (measured; only the ROM's download mode cleared it). Polling stdin reads
    it (mp_hal_stdio_poll)."""
    global _usb_in
    try:
        if _usb_in is None:
            _usb_in = select.poll()
            _usb_in.register(sys.stdin, select.POLLIN)
        _usb_in.ipoll(0)                 # (ipoll: no list per loop pass)
    except (OSError, ValueError, AttributeError, MemoryError):
        pass

def _uc_watch(now):
    """A restart whose banner was lost (a blocking POST longer than the UART
    buffer holds): back at its defaults, nothing else tells, as the setup's
    check runs once. No GST for UC_GST_LOST with sentences coming: set it up
    again (GST comes with or without a fix, once it is on)."""
    at = st["setup_at"]
    if st["setup_ok"] is not True or at is None or st["extra"]:
        return
    g = st["gst_at"]
    if _age(now, at) < UC_GST_LOST or _recent(now, g, UC_GST_LOST):
        return
    if not _recent(now, st["rx"], NO_DATA):
        return                           # silent: _gps_mute's case
    st["extra"] = 1                      # (its usual backoff, under a minute)
    print("gps setup lost: no GST")
    said = st["gst_said"]
    if not _recent(now, said, GPS_NOT_TAKEN_SAY):
        st["gst_said"] = now
        _diag_write("gps setup lost: no GST")

def gps_check(now):
    """Send the setup again while the receiver still sends the sentences it
    turns off: a receiver powered up with the board is still booting when
    run() sends it, and stays at 1 Hz. Less and less often, so a receiver
    that never takes it (or an unwired ESP TX) costs next to nothing."""
    _usb_drain()
    _uc_watch(now)
    if _age(now, st["loop_at"]) >= GPS_LOOP_WINDOW:
        st["loop_at"] = now
        _stamps_old(now)
    _gps_mute(now)
    _setup_check(now)
    # also when the setup went out but was not answered: it may sit at 1 Hz
    at = st["setup_at"]
    b = st["boot_at"]
    quiet = GPS_STORM_QUIET
    t = st["storm_at"]
    if _recent(now, t, GPS_FLAP_WINDOW):
        # in a storm, quiet means twice its last gap between restarts (up
        # to GPS_STORM_WAIT): restarts 12 s apart each got a setup, 54 in
        # 10 min, over a third of the loop blocked
        quiet = min(max(quiet, 2 * st["boot_gap"]), GPS_STORM_WAIT)
    if (st["setup_wait"] > GPS_RETRY and b is not None
            and (st["extra"] or st["setup_ok"] is not True)
            and _age(now, b) >= quiet):
        # quiet since the last restart: the storm (or a long backoff) is
        # over, the receiver sits at 1 Hz with every sentence on: set it up
        # now (measured: over a minute at 1 Hz)
        st["setup_wait"] = GPS_RETRY
        st["boot_at"] = None
    if not (at is None
            or (not st["extra"] and st["setup_ok"] is True)
            or _age(now, at) < st["setup_wait"]):
        if st["setup_ok"] is True and st["extra"]:
            st["ack_n"] += 1             # answered, but not taken
        st["setup_wait"] = min(st["setup_wait"] * 2, GPS_RETRY_MAX)
        print("gps setup again")
        gps_setup()

def _temp():
    try:
        return esp32.mcu_temperature()
    except Exception:
        return None

_freq_bad = []                           # clocks this board refused
_freq_bad_at = [None]                    # ticks of the last refusal

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
        at = _freq_bad_at[0]
        if _recent(time.ticks_ms(), at, FREQ_RETRY):
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
    _freq_bad_at[0] = time.ticks_ms()
    print("cpu freq", mhz, repr(err))

def _mhz_env(env, key, default):
    v = _plain(env.get(key))
    if not v:
        return default
    try:
        n = int(v)
    except ValueError:
        print(key, "invalid, using", default)
        return default
    allowed = (20, 40, 80, 160, 240)
    if n == 0 or n in allowed:
        return n
    print(key, "not 0 or", "/".join(str(m) for m in allowed), "on this",
          "board, using", default)
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
        print("DIAG not 0 or 10-3600 s, using", DIAG // 1000, "s")

def _diag_write(line):
    """Append one line to DIAG_FILE (closed again, so a power-off loses at
    most the line being written). Past DIAG_MAX the file becomes DIAG_OLD,
    so both together stay under 2 * DIAG_MAX. A line break in it becomes a
    space and a line is cut at DIAG_CUT (a field from the network, the OTA
    manifest's commit, wrote forged lines and ~4 KB ones). Never raises."""
    if not DIAG:
        return
    try:
        if "\n" in line or "\r" in line:
            line = line.replace("\r", " ").replace("\n", " ")
        if len(line) > DIAG_CUT:
            line = line[:DIAG_CUT - 3] + "..."
        room = free()
        if room is not None and room < MIN_FREE:
            return                       # the tracks keep the reserve
        size = _file_size(DIAG_FILE)
        if size is not None and size >= DIAG_MAX:
            _rm(DIAG_OLD)
            os.rename(DIAG_FILE, DIAG_OLD)
        f = open(DIAG_FILE, "a")
        try:
            f.write(line + "\n")
        finally:
            f.close()
    except Exception as e:
        print("diag", repr(e))

def _diag_utc():
    t = _fix_s()
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

# whole seconds since boot, ticks then, ms over the seconds (one count of
# ms passed the small-int range after 12.4 days: a heap long every loop)
_up = [time.ticks_ms() // 1000, time.ticks_ms(), time.ticks_ms() % 1000]

def uptime_s(now=None):
    """Seconds since boot. ticks_ms wraps every 2^30 ms (12.4 days), so
    the time is summed up from tick differences (called every loop)."""
    if now is None:
        now = time.ticks_ms()
    ms = _up[2] + time.ticks_diff(now, _up[1])
    _up[1] = now
    _up[0] += ms // 1000
    _up[2] = ms % 1000
    return _up[0]

# How long the run before this boot lasted, from the last diag lines it
# wrote: up (s, to within DIAG; 0 = it ended before its first status line),
# end (GPS time of its last line, "2026-10-28 12:19:42Z") and how
# ("status", "crash" or "short"). Read once at boot.
prev_run = {"up": None, "end": None, "how": None}

def _diag_prev():
    prev_run.update(up=None, end=None, how=None)
    # a rotation leaves the new file with the line that caused it, which
    # need not be a status line: then the older file holds the run's end
    lines = (_diag_tail(DIAG_OLD, 8) or []) + (_diag_tail(DIAG_FILE, 8) or [])
    for line in reversed(lines):
        f = line.split()
        try:
            if line.startswith("up "):
                prev_run["up"], prev_run["how"] = int(f[1]), "status"
                t = f[3] if len(f) > 3 and f[2] == "utc" else "-"
                if len(t) == 16:
                    prev_run["end"] = "%s-%s-%s %s:%s:%sZ" % (
                        t[0:4], t[4:6], t[6:8], t[9:11], t[11:13], t[13:15])
                return
            if line.startswith("crash up "):
                prev_run["up"], prev_run["how"] = int(f[2]), "crash"
                return
            if line.startswith("boot "):
                prev_run["up"], prev_run["how"] = 0, "short"
                return
        except (ValueError, IndexError):
            continue

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
    then _words."""
    _diag_write(
        "up %d utc %s nmea %d/%d gpsboots %d loops %d sats %d/%s fix %s %s" % (
            uptime_s(now), _diag_utc(), st["ok"], st["err"], st["gps_boots"],
            st["loops"], sv_seen(), sv_used(), status().replace(" ", "_"),
            _words()))

def _diag_final():
    """A status line right before a deliberate reset: the next boot reads
    the run's length and end from it (_diag_prev). The last regular one
    can be up to DIAG older, and an OTA install takes ~1.5 min after it:
    the page showed the run before an update 1 min 05 s long, it was 2.5."""
    try:
        _diag(time.ticks_ms())
    except Exception:
        pass

def _hex(c):
    """An ASCII hex digit's value, or -1."""
    if 48 <= c <= 57:
        return c - 48
    c |= 32
    return c - 87 if 97 <= c <= 102 else -1

def _space(c):
    return c == 32 or 9 <= c <= 13       # what bytes.strip() strips

# the names in HANDLERS by their 3 bytes as one int (the type is looked up
# in the bytes, not in a slice), and the talkers seen, as str, kept
_HKEY = {ord(k[0]) << 16 | ord(k[1]) << 8 | ord(k[2]): k for k in HANDLERS}
_TALK = {}
TALK_MAX = 8                             # talkers kept (GP GN GL GA GB ...)

def handle(line, s=0, e=None):
    """One NMEA sentence: line[s:e] (read_uart hands over its buffer and
    the line's bounds). The whitespace, the last "$" (garbage or a cut-off
    sentence ran into it), the checksum and the sentence type
    are read from the bytes in place; only a type in HANDLERS is decoded
    and split. That was ~20 allocations a sentence (a copy, slices,
    decode, split, the type and talker), at up to 5 Hz x 2-3 sentences.
    One with a byte over 0x7F goes the old way (decode, the type from the
    text): it counts as it did."""
    if e is None:
        e = len(line)
    v = _rxv if line is _rx else memoryview(line)
    while s < e and _space(v[s]):
        s += 1
    while e > s and _space(v[e - 1]):
        e -= 1
    i = line.rfind(b"$", s, e)
    if i > s:
        s = i
    if s >= e:
        return
    if i < 0:
        # the UC6580's start-up banner, plain text (its FWVer line once)
        if line.find(b"FWVer", s, e) >= 0:
            _gps_restarted()
        return
    star = line.rfind(b"*", s, e)
    if star < 0:
        st["err"] += 1
        return
    p = v[s + 1:star]                    # (one view: checksum and text)
    c = hi = 0
    for b in p:
        c ^= b
        hi |= b
    # exactly two hex digits after the "*" ("*+F", "* 5", "*5" are bad;
    # a non-hex one, -1, makes h negative: no match)
    h = _hex(v[star + 1]) << 4 | _hex(v[star + 2]) if star + 3 == e else -1
    if c != h:
        st["err"] += 1
        return
    st["good"] = time.ticks_ms()         # the last whole frame (GPS link)
    k = -1                               # a 5-character address: its type
    if star - s == 6 or star - s > 6 and v[s + 6] == 44:
        k = v[s + 3] << 16 | v[s + 4] << 8 | v[s + 5]
    name = _HKEY.get(k)
    if name == "GSV" and _uc_gsv_skip():
        st["ok"] += 1
        return
    if name is None and hi < 128:
        _uc_line(p)
        st["ok"] += 1                    # one the logger does not read
        return
    try:
        f = str(p, "utf-8").split(",")
        if hi > 127:
            name = f[0][2:]
            talker = f[0][:2]
        else:
            t = v[s + 1] << 8 | v[s + 2]
            talker = _TALK.get(t)
            if talker is None:
                talker = f[0][:2]
                if len(_TALK) < TALK_MAX:
                    _TALK[t] = talker
        fn = HANDLERS.get(name)
        if fn:
            fn(talker, f)
        st["ok"] += 1
    except (ValueError, IndexError, UnicodeError, MemoryError):
        st["err"] += 1

# What the UART delivers goes into one preallocated buffer, read in place
# and compacted: growing a bytes object by each read and slicing it at each
# line made a new copy of everything left several times a second, and one big
# allocation grows the heap (see Upload) and starves TLS. Line ends are
# searched with bytearray.find (start/end in place, no copy: MicroPython 1.29
# has it), not byte by byte in Python.
RX_READ = 256                            # bytes per readinto: small pieces
RX_READS = 64                            # readintos per read_uart: 16 KB, twice
                                         # the Heltec's whole UART buffer; the
                                         # rest waits there for the next pass
RX_LINE = 256                            # no newline after this many bytes: junk
_rx = bytearray(RX_LINE + RX_READ)
_rxv = memoryview(_rx)
_rx_n = 0                                # bytes in _rx
_rx_s = 0                                # scanned up to here: no line end
                                         # before it

def _rx_drop(r):
    """Drop the first r bytes of _rx, the rest moved to the front (one
    slice store: MicroPython copies it with memmove, overlap is fine)."""
    global _rx_n
    n = _rx_n - r
    _rxv[0:n] = _rxv[r:r + n]
    _rx_n = n

def read_uart():
    """What the UART has: lines to handle(), in place. An error in one (a
    MemoryError while publishing a second) counts it bad (st["err"]) and the
    rest is read: no exception leaves the buffer uncommitted, or the next
    call handled the same sentences again and queued the same seconds twice.
    At most RX_READS reads: a UART never empty (a flood, handlers slower than
    it) must not hold the loop, its feed() and the logging."""
    global _rx_n, _rx_s
    b = _rx
    left = RX_READS
    while left and uart.any():
        left -= 1
        n = _rx_n
        k = uart.readinto(_rxv[n:n + RX_READ])
        if not k:
            return
        st["rx"] = time.ticks_ms()
        # bytes read (the page's GPS link: differences), kept a small int
        st["rx_b"] = (st["rx_b"] + k) & 0x3FFFFFFF
        n += k
        _rx_n = n
        r = 0                            # bytes before r are done with
        s = _rx_s
        while True:
            j = b.find(b"\n", s, n)
            if j < 0:
                s = n
                break
            e = j - 1 if j > r and b[j - 1] == 13 else j
            try:
                handle(b, r, e)          # in place: no copy
            except Exception:            # (a handler's MemoryError): the
                st["err"] += 1           # line is bad, not read again
            r = s = j + 1
        if r:
            _rx_drop(r)
        _rx_s = s - r
        if _rx_n > RX_LINE:              # no newline in sight: garbage or a
            _rx_n = _rx_s = 0            # wrong baud rate

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

def load_env(path):
    """KEY=VALUE lines -> dict. Surrounding quotes are stripped.

    Read as bytes: a .env saved as ANSI (Notepad) raised UnicodeError at
    every boot. A value that is not UTF-8 stays bytes for SSID/WPA2 keys
    (connect() takes bytes) and is skipped for the others; never raises."""
    env = {}
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
                try:
                    k = k.decode()
                except UnicodeError:
                    continue
                try:
                    v = v.decode()
                except UnicodeError:
                    if k[:4] not in ("SSID", "WPA2"):
                        print("env", k, "not UTF-8, skipped")
                        continue
                env[k] = v
    except OSError:
        pass                             # no .env on the device
    except Exception as e:
        print("env", repr(e))
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
        print("static ip", ipk, "needs a valid", ipk, "and", gk, "- using DHCP")
        return None
    mask_raw, dns_raw = _plain(env.get(mk)), _plain(env.get(dk))
    mask, dns = _ipv4(mask_raw), _ipv4(dns_raw)
    if mask_raw and not mask:
        print("static ip", mk, "invalid - using 255.255.255.0")
    if dns_raw and not dns:
        print("static ip", dk, "invalid - using", gk)
    return (ip, mask or "255.255.255.0", gw, dns or gw)

def _say(line):
    """On the console and in the diag log: a .env mistake (key names only: an
    SSID or a password never goes into either), a GPS or heap note."""
    print(line)
    _diag_write(line)

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
        known += (sk, pk, hk2, hidk)
        s = env.get(sk)
        if not s:
            continue
        sb = s if isinstance(s, bytes) else s.encode()
        if sb in seen:
            _say("env %s: an SSID already set above, ignored" % sk)
            continue
        seen.append(sb)
        p = env.get(pk)
        if p is not None and not p:
            _say("env %s empty: %s is tried as an open network"
                      % (pk, sk))
        home = not x                     # (the first network by default)
        t = _truthy(env.get(hk2))
        if t is None:
            t = _truthy(env.get("HOME" + (x or "1")))
        if t is not None:
            home = t
        hidden = bool(_truthy(env.get(hidk)))
        n = {"ssid": s, "pw": "" if p is None else p,
             "home": home, "hidden": hidden, "bssid": None,
             "ip": _static_ip(env, ("IP" + x, "MASK" + x, "GW" + x,
                                    "DNS" + x))}
        if isinstance(s, bytes):
            # not UTF-8 (an ANSI .env): the radio gets the bytes as they
            # are, the screen and the page one char per byte
            n["raw"] = s
            n["ssid"] = "".join(chr(c) for c in s)
        nets.append(n)
    for k in env:
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
                backoff=WIFI_RETRY, last_check=time.ticks_ms(),
                cand=[], cand_i=0, try_at=None,
                roam_at=time.ticks_ms(), prev_status=None,
                roam_backoff=ROAM_CHECK, moved_at=None, roaming=False,
                retried=False, idle_at=time.ticks_ms(), sleep_home=False,
                lost_at=None, fails=0, diag_at={}, diag_more={}, empty=0,
                pb_at=time.ticks_ms(), pb_scans=0, pb_empty=0,
                wake=False, wake_at=None,
                blank_rssi=None, blank_bssid=None, blank_bad=[],
                blank_ours=False, home_aps=[], home_miss={}, learn=False,
                hid_wait=ROAM_CHECK, hid_at=None, blank_any=False,
                rounds=0, hh=None,
                seen_at=None, seen_wait=SEEN_WAIT,
                # the link is down since boot (a board booting faster than
                # the router): _web_search counts from here, as after a
                # drop (lost); down_ssid None: any home network may come up
                down_at=time.ticks_ms(), down_home=True, down_rssi=None,
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
    now = time.ticks_ms()
    at, more = wifi["diag_at"], wifi["diag_more"]
    t = at.get(kind)
    if _recent(now, t, WIFI_DIAG_GAP):
        more[kind] = more.get(kind, 0) + 1
        return
    n = more.pop(kind, 0)
    at[kind] = now
    _diag_write(line + (" (+%d more)" % n if n else ""))

def _mark_failed(ssid, why=""):
    wifi["failed"][ssid] = time.ticks_ms()
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
            now = wifi["failed"][ssid]
            b = wifi["hid_wait"]
            wifi["hid_wait"] = min(max(b * 2, ROAM_CHECK), ROAM_BACKOFF_MAX)
            wifi["hid_at"] = now
            b = wifi["blank_bssid"]
            if (b is not None and b not in wifi["home_aps"]
                    and not _blank_bad(b, now)):
                bad = [x for x in wifi["blank_bad"]
                       if _age(now, x[1]) < BLANK_BAD_HOLD]
                bad.append((b, now))
                wifi["blank_bad"] = bad[-4:]
        wifi["roaming"] = False

def _blank_bad(bssid, now):
    """A hidden-home try on this empty-SSID AP failed within BLANK_BAD_HOLD."""
    for b, t in wifi["blank_bad"]:
        if b == bssid and _age(now, t) < BLANK_BAD_HOLD:
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

def _scan_known(w):
    """Visible known nets, strongest BSSID per SSID, or None if scan fails.
    Hidden APs are listed with an empty SSID; the strongest blank RSSI is
    a hint that *some* hidden AP is nearby (not necessarily ours). One of
    the hidden home's learned BSSIDs (_learn_home) at >= ROAM_MIN_RSSI is
    taken first (blank_ours); else the strongest blank AP stays weak
    evidence (a mesh node not learned yet): enough for the candidate order
    and a roam, not for "home in range" or the keep-alive wake. A blank AP
    whose hidden-home try failed lately (blank_bad) is skipped. A
    MemoryError after the scan (the stay check, the copies below) is a
    failed scan too: it escaped wifi_tick and reset the board."""
    try:
        aps = w.scan()
    except SCAN_ERR:
        aps = None
    if aps is not None and not _scan_empty(aps):
        try:
            return _scan_found(aps)
        except MemoryError:
            gc.collect()
    wifi["blank_rssi"] = None
    wifi["blank_bssid"] = None
    wifi["blank_ours"] = False
    wifi["blank_any"] = False
    return None

def _scan_found(aps):
    """_scan_known's list from a good scan (aps)."""
    by = {}
    for n in wifi["nets"]:
        by[_ssid_b(n)] = n
    mine = wifi["home_aps"]
    _stay_scan(aps)
    _home_seen(aps)
    now = time.ticks_ms()
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
            if _blank_bad(bssid, now):
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
    if own_rssi is not None and own_rssi >= ROAM_MIN_RSSI:
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

def home_in_range():
    """True while a home network is in range (>= ROAM_MIN_RSSI): the RSSI
    of the connected home network, else a scan (~2 s, the radio turned on
    for it and off again when it slept). None when that cannot be told
    (no home network configured, a connect in progress, scan failed).
    On the home link the bar is SEEN_MIN_RSSI: ROAM_MIN_RSSI is for
    jumping to a weak AP; associated at -78 dBm the board is at home, and
    that False let a multipath jump end the home stay (_home_veto)."""
    w = wifi["wlan"]
    if w is None or not _has_home():
        return None
    state = wifi["state"]
    lim = ROAM_MIN_RSSI
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
        found = _scan_known(w)
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
    return bool(_blank_home_ok(True))

def _home_veto(now, sl):
    """A departure from a home stay while the home WiFi is still in range
    is a GPS jump, not a departure (measured: a 194 m multipath jump ended a
    home stay and uploaded 80 drift points). After a veto, departures wait
    STILL_HOME_HOLD before the next check."""
    at = sl.get("veto_at")
    if _recent(now, at, STILL_HOME_HOLD):
        return True
    if home_in_range():
        sl["veto_at"] = now
        print("stay kept: home in range")
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

def _blank_home_ok(strict=False):
    """True if a hidden home is configured and a strong empty-SSID AP was
    seen. strict ("home in range"): once BSSIDs are learned, only one of
    them counts (_scan_known); else any strong blank AP does."""
    r = wifi["blank_rssi"]
    if r is None or r < ROAM_MIN_RSSI:
        return False
    if _blank_bad(wifi["blank_bssid"], time.ticks_ms()):
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
        read_uart()
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
    or damaged file (or a line that is no BSSID) is no error."""
    aps = wifi["home_aps"] = []
    if not HOME_APS_FILE:
        return
    try:
        f = open(HOME_APS_FILE, "rb")
    except OSError:
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
    except (OSError, ValueError):
        pass
    finally:
        f.close()

def _home_aps_save():
    """HOME_APS_FILE rewritten (a few bytes, only when the list changed);
    not into the flash reserve the tracks keep (MIN_FREE). Never raises."""
    if not HOME_APS_FILE:
        return
    try:
        room = free()
        if room is not None and room < MIN_FREE:
            return
        f = open(HOME_APS_FILE, "wb")
        try:
            for b in wifi["home_aps"]:
                f.write(binascii.hexlify(b) + b"\n")
        finally:
            f.close()
    except (OSError, TypeError, ValueError, MemoryError) as e:
        print("home aps", repr(e))      # (MemoryError: the format, open())

def _hidden_home_target(now, arrival=False):
    """Hidden home when a strong empty-SSID AP was seen.
    Always bssid=None so the IDF probes by SSID (never pin to the
    strongest blank AP — that may be a neighbour). A stop after driving
    must not treat an unverified blank AP as home."""
    if arrival:
        return None
    if not _blank_home_ok():
        return None
    at = wifi["hid_at"]
    if _recent(now, at, wifi["hid_wait"]):
        return None                      # a hidden-home roam failed lately
    for n in wifi["nets"]:
        if (n.get("home") and n.get("hidden")
                and n["ssid"] != wifi["ssid"]
                and not _recent(now, wifi["failed"].get(n["ssid"]), FAIL_HOLD)):
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
    now = time.ticks_ms()
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
        (bad if _recent(now, failed.get(n["ssid"]), FAIL_HOLD)
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

def _wifi_got_ip(w):
    wifi["state"] = "connected"
    try:
        wifi["ip"] = w.ifconfig()[0]
    except WLAN_ERR:
        wifi["ip"] = None
    wifi["rssi"] = _rssi(w)
    wifi["home"] = _net_home(wifi["ssid"])
    wifi["down_at"] = None
    lost = wifi["lost_at"]
    if lost is not None:
        _wifi_diag("back", "wifi back %s after %s, %d failed tries, rssi %s"
                   % (wifi["ssid"] or "", _hms(_age(time.ticks_ms(), lost)
                                                   // 1000),
                      wifi["fails"], wifi["rssi"]))
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
    wifi["idle_at"] = time.ticks_ms()
    wifi["failed"].pop(wifi["ssid"], None)
    wifi["backoff"] = WIFI_RETRY
    wifi["roam_at"] = time.ticks_ms()
    wifi["roaming"] = False
    wifi["seen_wait"] = SEEN_WAIT
    if wifi["home"]:
        wifi["roam_backoff"] = ROAM_CHECK
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
    wifi["last_check"] = now

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
    if not WEB or not (WEB_AWAY or _has_home()):
        return False
    if not wifi["down_home"]:
        if not WEB_AWAY:
            return False                 # no page off home anyway
    elif PB_KEEPALIVE and _home_seeable(wifi["down_ssid"]):
        r = wifi["down_rssi"]
        if r is None or r >= SEEN_MIN_RSSI:
            return False                 # _home_seen wakes it
    at = wifi["down_at"]
    return _recent(now, at, WEB_SEARCH)

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
    writes no anchor points; leaving clears it (_still_leave)."""
    if state == "sleep":
        wifi["sleep_home"] = bool(at_home() and log["still"]["active"])
    _radio_off(w)
    wifi["lost_at"] = None               # ("back after" measures a reconnect,
                                         # not a sleep)
    wifi.update(state=state, home=False, ip=None, rssi=None, wake=False,
                wake_at=wake_at, backoff=WIFI_RETRY, last_check=time.ticks_ms())
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
    if state.startswith("err") or state in ("fs err", "low mem", "low idf"):
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
        if h <= 0:
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
        wifi["last_check"] = time.ticks_add(time.ticks_ms(), -wifi["backoff"])
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
    read_uart()
    found = _scan_known(w)
    feed()
    now = time.ticks_ms()
    target = None
    for n in found or ():
        if (n.get("home") and n.get("rssi", -100) >= ROAM_MIN_RSSI
                and n["ssid"] != wifi["ssid"]
                and not _recent(now, wifi["failed"].get(n["ssid"]), FAIL_HOLD)):
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
        wifi["roam_at"] = now
    cur = _net(wifi["ssid"])
    wifi["cand"] = [target] + ([dict(cur)] if cur else [])
    wifi["cand_i"] = 0
    wifi["retried"] = False
    wifi["roaming"] = True
    wifi["down_at"] = now                # the link is left: if neither
    wifi["down_home"] = False            # connects, a drop off home
    wifi["down_rssi"] = None
    wifi["down_ssid"] = None
    print("wifi roam", target["ssid"])
    _wifi_connect(w, target)

def _gps_sys_env(env):
    """GPS_SYS=<hex CFGSYS mask> in .env: the UC6580's systems and bands (its
    own otherwise; H35155 on the board). A bad value keeps its own."""
    global UC_SYS
    v = _plain(env.get("GPS_SYS"))
    if not v:
        return
    try:
        UC_SYS = int(v[1:] if v[0] in "hH" else v, 16)
    except ValueError:
        print("GPS_SYS invalid, using the receiver's own")

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
        print("MIN_MOVE not 0-50 m, using", MIN_MOVE)

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
                print("KEEPALIVE out of 0 or 5-600 s, using",
                      PB_KEEPALIVE // 1000, "s")
        except ValueError:
            print("KEEPALIVE invalid, using", PB_KEEPALIVE // 1000, "s")

def _pb_pulse_radio(w):
    """Radio on, one scan (it holds ~100 mA ~2 s, and watches the stay's access
    points and the home network: _stay_scan, _home_seen), off again."""
    try:
        w.active(True)
        aps = w.scan()
        if not _scan_empty(aps, "pb_empty"):
            _stay_scan(aps)
            _home_seen(aps)
    except SCAN_ERR:
        pass
    finally:
        try:
            w.active(False)
        except WLAN_ERR:
            pass

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
    now = time.ticks_ms()
    at = wifi["seen_at"]
    if _recent(now, at, wifi["seen_wait"]):
        return
    wifi["seen_at"] = now
    wifi["seen_wait"] = min(wifi["seen_wait"] * 2, WIFI_BACKOFF_MAX)
    if state == "sleep":
        wifi["wake"] = True
    else:
        wifi["last_check"] = time.ticks_add(now, -wifi["backoff"])
    print("wifi home in range")

def wifi_start(env=None):
    """WiFi, upload, page, OTA and NTP settings from .env (env: the dict
    run() read already), the first connect round."""
    global DEVICE_ID, HOSTNAME, CPU_MHZ, CPU_MHZ_UPLOAD
    if env is None:
        env = load_env(ENV_FILE)
    CPU_MHZ = _mhz_env(env, "CPU_MHZ", CPU_MHZ)
    CPU_MHZ_UPLOAD = _mhz_env(env, "CPU_MHZ_UP", CPU_MHZ_UPLOAD)
    _set_freq(CPU_MHZ)
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
    _web_env(env)
    _ota_env(env)
    _ntp_env(env)
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
    w = wifi["wlan"]
    _wifi_begin_round(w)
    if wifi["state"] == "connecting":
        _wifi_poll(w)
    wifi["last_check"] = time.ticks_ms()

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
        wifi["last_check"] = time.ticks_ms()
        return None

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
        # the interface failed to come up at boot: it used to stay down
        # for good (the keep-alive's own WLAN showed the chip recovered)
        if wifi["state"] == "error" and wifi["nets"] and _age(
                time.ticks_ms(), wifi["last_check"]) >= wifi["backoff"]:
            wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY),
                                  WIFI_BACKOFF_MAX)
            w = _wifi_make()
            if w is not None:
                print("wifi interface up")
                _wifi_begin_round(w)
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
            if _age(now, wifi["last_check"]) >= WIFI_CHECK:
                wifi["rssi"] = _rssi(w)
                wifi["last_check"] = now
            interval = wifi["roam_backoff"]
            if not wifi["home"] and _has_home():
                spd = st.get("speed")
                moving = (has_fix(now) and spd is not None and spd >= 5
                          and not log["still"]["active"])
                if moving:
                    wifi["moved_at"] = now
                    arrival = False
                elif (wifi["moved_at"] is not None
                      and _age(now, wifi["moved_at"]) >= 60000):
                    wifi["moved_at"] = None
                    arrival = True
                else:
                    arrival = False
                ra = wifi["roam_at"]     # (is None: tick 0 is a time)
                due = (not moving and ra is not None
                       and _age(now, ra) >= interval)
                if due or arrival:
                    wifi["roam_at"] = now
                    _try_roam_home(w, arrival=arrival and not due)
            if (wifi["state"] == "connected"
                    and _age(now, wifi["idle_at"]) >= WIFI_IDLE):
                wifi["idle_at"] = now
                if WEB and (wifi["home"] or WEB_AWAY):
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
        wifi["lost_at"] = now
        wifi["fails"] = 0
        # a drop: what dropped decides whether _web_search searches
        wifi["down_at"] = now
        wifi["down_home"] = bool(wifi["home"])
        wifi["down_rssi"] = wifi["rssi"]
        wifi["down_ssid"] = wifi["ssid"]   # whose return a scan must see
        _wifi_diag("lost", "wifi lost %s rssi %s" % (wifi["ssid"] or "",
                                                    wifi["rssi"]))
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
        print("wifi lost", wifi["ssid"] or "")
        return
    if wifi["state"] == "lost":
        if (s == network.STAT_CONNECTING or s == network.STAT_IDLE
                or s is None):
            ta = wifi["try_at"]
            if ta is not None and _age(now, ta) < LOST_GRACE:
                return
        wifi["state"] = "offline"
        wifi["last_check"] = time.ticks_add(now, -wifi["backoff"])
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
            if not due and _age(now, wifi["last_check"]) < WIFI_SLEEP:
                return
            if _age(now, wifi["idle_at"]) < WIFI_IDLE:
                return
            wifi["idle_at"] = now
            if (_nothing_to_send() and not _ota_wants_radio()
                    and not (WEB and (WEB_AWAY or _has_home()))):
                wifi["wake_at"] = None
                return
        wifi["wake"] = False
        wifi["backoff"] = WIFI_RETRY
        wifi["last_check"] = now
        wifi["rounds"] = 0               # a wake: the first round tries all
        _wifi_begin_round(w)
        return
    if _age(now, wifi["last_check"]) < wifi["backoff"]:
        return
    wifi["last_check"] = now
    _wifi_begin_round(w)

def wifi_stop():
    _web_close()
    w = wifi["wlan"]
    if w is None:
        return
    try:
        w.disconnect()
        w.active(False)
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
               fix=False, lost_at=None, new_seg=False, seg_tc=[],
               full=False, wfail=False, wfails=0,
               still=_still_state(), clock=None, seq=0,
               last_sample=None, last_flush=time.ticks_ms(), last_t=None,
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
    if la is not None and _age(time.ticks_ms(), la) >= HOLD_RELEASE:
        return None
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
            and _age(now, st["pos_at"]) < STALE)

def _tcn(tc):
    """A compact time as a track file's name: YYYYMMDD_HHMMSS."""
    return tc[:8] + "_" + tc[9:15]

def start_file(stamp=None):
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass                             # already exists
    name = stamp or _tcn(_tc())
    # (wfail: a write that failed on the previous file must not keep the
    # next one from closing: log_tick)
    log.update(file=LOG_DIR + "/" + name + TRB,
               created=False, points=0,
               new_seg=False, seg_tc=[], file_at=time.ticks_ms(),
               good_size=None, wfail=False, wfails=0)
    # First write happens when there are points (parking must not create
    # an empty / one-point file just because a fix came back).

def flush(hold=None):
    """Write pending points to the current file, opened only for the write.
    LittleFS commits on close: drop the RAM batch only after a successful
    close, or if the file size shows the new bytes actually landed. With
    hold (a compact track time), only the lines before it are written: the
    rest is the drift of a stay still being proven (see _still_hold)."""
    log["last_flush"] = time.ticks_ms()
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
            log["full_tc"] = _diag_utc() if st.get("date") else "-"
        log["full"] = True
        log["drop"] += len(batch)
        del batch[:]
        _seg_break()
        log["prev"] = None
        if log["fix"]:
            log["fix"], log["lost_at"] = False, time.ticks_ms()
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
    try:
        pts = 0
        if creating:
            start = len(HEADERB)
            f = open(path, "wb")
            f.write(HEADERB.encode())
        else:
            f = open(path, "r+b")
            if not _seek_footer(f, old):
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
        _werr("write", e)
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
            log["wfails"] = log["wfails"] + 1
            if log["wfails"] >= WFAIL_SWITCH and batch:
                tc = batch[0][-17:-1]
                name = _tcn(tc)
                if creating:
                    if path[-19:-4] == name:
                        s = _kb_time(tc.encode(), 0)
                        if s is not None:
                            tc = _stamp_s(s + 1)[1]
                            name = _tcn(tc)
                    _rm(path)
                else:
                    _cut_to_footer(path, log["good_size"], sz)
                print("track file failing, new file", path, name)
                _diag_write("track %s failing %s new %s" % (
                    "create" if creating else "write", path[-19:], name))
                seg = log["new_seg"], log["seg_tc"]
                start_file(name)
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
        now = time.ticks_ms()
        at = log["err_at"]
        if not _recent(now, at, WERR_EVERY):
            log["err_at"] = now
            _diag_write("track %s error %d %r %s" % (
                what, log["err"], e, (log["file"] or "-")[-19:]))
    except MemoryError:
        pass

def _cut_to_footer(path, good, sz):
    """Before a failing file is left: a failed append that partly landed
    left the start of the batch after the last good footer, without one,
    and the new file takes the whole batch. Write FOOTER back where it was
    (once, best effort): readers stop there, so the tail is not uploaded
    twice. Only a footer written this session (good), the file longer."""
    n = len(FOOTER)
    if good is None or good < n or sz is None or sz <= good:
        return
    f = None
    try:
        f = open(path, "r+b")
        f.seek(good - n, 0)
        f.write(FOOTER.encode())
    except (OSError, MemoryError) as e:  # (it runs when heap is short too)
        _werr("cut", e)
    if f is not None:
        try:
            f.close()
        except OSError as e:
            _werr("cut", e)

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

def _hist_begin(st, day):
    """Open the day's file for an append into st ([file, made now, text,
    the .trb records not yet written]): its .trb, a new one with the
    header, an old one at its footer (HIST_SEP first without one); a .trk
    older firmware began the day with at its end (a newline first after a
    cut line, see _tail_nl). A .trb append begins with a keyframe."""
    path = HIST_DIR + "/" + day
    size = _file_size(path + TRB)
    text = False
    if size is None:
        n = _file_size(path + ".trk")
        if n is not None:
            path, size, text = path + ".trk", n, True
    if not text:
        path += TRB
    st[1], st[2] = size is None, text
    if size is None:
        try:
            os.mkdir(HIST_DIR)
        except OSError:
            pass                         # (there already)
    if text:
        cut = not _tail_nl(path, size)
        st[0] = open(path, "ab")
        if cut:
            st[0].write(b"\n")
        return
    _kh[0] = None                        # (a keyframe first)
    _khn[0] = 0
    st[3] = bytearray()
    if not size:
        st[0] = open(path, "wb")
        st[0].write(HEADERB.encode())
        return
    f = st[0] = open(path, "r+b")
    n = len(FOOTER)
    if size >= n:
        f.seek(size - n)
        if f.read(n) == FOOTER.encode():
            f.seek(size - n)
            return
    f.seek(size)
    f.write(HIST_SEP)

def _hist_line(st, buf, a, b):
    """The text line buf[a:b] (its newline last) into the open day file:
    a .trk's as it is; a .trb's as the next record (as _kb_part: a delta,
    or a keyframe first and after KEY_EVERY deltas), written a KB_CHUNK at
    a time."""
    if st[2]:
        st[0].write(memoryview(buf)[a:b])
        return
    out = st[3]
    if _khn[0] < KEY_EVERY and _kb_delta(buf, _kh, out, a, b):
        _khn[0] += 1
    else:
        out.append(61)                   # =
        out += memoryview(buf)[a:b]
        _kb_parse(buf, a, b - 1, _kh)
        _khn[0] = 0
    if len(out) >= KB_CHUNK:
        st[0].write(out)
        st[3] = bytearray()

def _hist_close(st, ok=True):
    """End the append in st: a .trb's last records and its footer, only if
    every write before landed (after a failed one the next append's
    HIST_SEP ends what it cut), then close. A new day's file drops the
    days beyond HIST_DAYS."""
    f, new, text, out = st
    st[0], st[1], st[3] = None, False, None
    if f is not None:
        if ok and not text:
            try:
                out += FOOTER.encode()
                f.write(out)
            except (OSError, MemoryError):
                log["herr"] += 1
        try:
            f.close()
        except OSError:
            pass
    if new:
        try:                             # the days beyond the newest
            ns = _hist_names()           # HIST_DAYS go (a day written as
            ds = sorted(set(n[:8] for n in ns))  # .trk and .trb is one)
            if len(ds) > HIST_DAYS:
                cut = ds[-HIST_DAYS]
                for n in ns:
                    if n[:8] < cut:
                        _rm(HIST_DIR + "/" + n)
        except OSError:
            pass

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
    while room is not None and room < want and days and days[0][:8] != days[-1][:8]:
        d = days[0][:8]
        while days[0][:8] == d:
            _rm(HIST_DIR + "/" + days.pop(0))
        room = free()
    return room

def _dlon(d):
    """A longitude difference across 180° the short way round, or a longitude
    made of a reference plus an offset back within 180°. (Not the modulo: on
    the board's single floats 19.000123 + 180 keeps 1.5 m.)"""
    if d > 180:
        return d - 360
    if d < -180:
        return d + 360
    return d

def _dist(lat1, lon1, lat2, lon2):
    """Metres between two nearby positions (equirectangular approximation),
    across 180° too."""
    x = (math.radians(_dlon(lon2 - lon1))
         * math.cos(math.radians((lat1 + lat2) / 2)))
    y = math.radians(lat2 - lat1)
    return 6371000 * math.sqrt(x * x + y * y)

def _fresh_hdop(now):
    """HDOP from a GGA in this epoch (not the previous fix)."""
    hdop, at = st.get("hdop"), st.get("hdop_at")
    if hdop is None or at is None:
        return None
    if _age(now, at) >= STALE:
        return None
    return hdop

def _fresh_std(now):
    """The receiver's error estimate (GST) of the last GST_HOLD ms, or
    None. Only this second's left 4 of 5 seconds unfiltered at 1 Hz (GST
    comes once per 5 epochs) and every second whose GST was lost."""
    std, at = st.get("std"), st.get("std_at")
    if std is None or not _recent(now, at, GST_HOLD):
        return None
    return std

def _usable(now):
    """The current fix is good enough to use: HDOP at most MAX_HDOP and the
    receiver's own error estimate at most GPS_MAX_STD (each when known).
    The estimate catches a cold start converging: measured on the board, a
    good HDOP (1.2-3) while the position walked in from 230 m away. A UC6580's
    estimate does not catch it (0.3 m while 4 m off, measured), so its first
    UC_SETTLE after a start count only moving: 15 m off at first, 5 m after
    a minute (measured), which standing still draws as a walk."""
    if not st["settled"]:
        a = st["fix_first"]
        if a is not None:
            # not _age: a second queued before a restart (_drain_q) is
            # older than this fix_first, and TICKS_OLD would settle it
            if time.ticks_diff(now, a) >= UC_SETTLE:
                st["settled"] = True
            elif (st.get("speed") or 0) < UC_SETTLE_KMH:
                return False
    hdop = _fresh_hdop(now)
    if hdop is not None and hdop > MAX_HDOP:
        return False
    std = _fresh_std(now)
    return std is None or std <= GPS_MAX_STD

def _sample_line(now, prev, last_t):
    """(time, track line) for the current fix, or None: not usable (try
    again next SAMPLE), same second as last_t, or within MIN_MOVE of
    prev."""
    if not _usable(now):
        return None
    lat, lon = st["lat"], st["lon"]
    t = st["date"] + "T" + st["time"] + "Z"
    if (last_t is not None and t <= last_t) or (prev is not None and _dist(
            prev[0], prev[1], lat, lon) <= MIN_MOVE):
        return None
    return t, _line("%.6f,%.6f" % (lat, lon), _fields(now), _tc())

TIME_BACK = 60                           # s a fix behind the last line ...
TIME_BACK_N = 10                         # ... in a row that drop last_t

def _time_back(t, lt):
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
    tc = _tc()
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

def _fields(now):
    """The elevation and HDOP fields of a track line for the current fix (""
    when not known). Also what the departure ring keeps (_RE)."""
    ele = ""
    if st["alt"] is not None and st.get("fixtype") != 2:
        ele = "%.1f" % st["alt"]
    hdop = _fresh_hdop(now)
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

def _record(s, pos):
    """Append a (time, line) from _sample_line; pos is what it wrote."""
    if log["file"] is None:
        start_file()
    q = log["seg_tc"]
    if q and q[-1] is None:
        q[-1] = s[1][-17:-1]             # the new segment starts here
    log["last_t"], log["prev"] = s[0], pos
    log["batch"].append(s[1])
    if len(log["batch"]) > MAX_BATCH:
        log["batch"].pop(0)
        log["drop"] += 1                 # (it went silently)

def _tc():
    """Compact UTC time of the current fix, as in a track line."""
    return st["date"].replace("-", "") + "T" + st["time"].replace(":", "") + "Z"

# UTC seconds <-> date without time.mktime (its epoch and time zone differ
# between ports); proleptic Gregorian, days since 1970-01-01.

def _days(y, m, d):
    if m <= 2:
        y -= 1
    era = y // 400
    yoe = y - era * 400
    doy = (153 * (m + (-3 if m > 2 else 9)) + 2) // 5 + d - 1
    return era * 146097 + yoe * 365 + yoe // 4 - yoe // 100 + doy - 719468

def _civil(z):
    z += 719468
    era = z // 146097
    doe = z - era * 146097
    yoe = (doe - doe // 1460 + doe // 36524 - doe // 146096) // 365
    doy = doe - (365 * yoe + yoe // 4 - yoe // 100)
    mp = (5 * doy + 2) // 153
    m = mp + 3 if mp < 10 else mp - 9
    return yoe + era * 400 + (1 if m <= 2 else 0), m, doy - (153 * mp + 2) // 5 + 1

# Internally seconds count from 2020 (the clock, the ring, heartbeats):
# from 1970 they pass MicroPython's small-int range (2**30), so every second
# allocated a big int. From 2020 that lasts until 2054.
EPOCH_DAYS = 18262                       # 2020-01-01 in days since 1970
FIX_S_END = 2145916800                   # 2088-01-01 in seconds since 2020:
                                         # a fix dated before 2020 or from
                                         # then on is not used (a receiver
                                         # past a GPS week rollover says 1999,
                                         # read as 2099; the departure ring's
                                         # 32-bit seconds overflowed silently
                                         # on the board and stamped the way
                                         # out 1951)

def _fix_s():
    """Seconds since 2020 of the current fix's date and time, or None."""
    return _dt_s(st["date"], st["time"])

def _dt_s(d, t, i=0):
    """Seconds since 2020 of a date YYYY-MM-DD and, from index i of t, a time
    HH:MM:SS (an ISO time is both: i 11), or None: no number, or no real
    time in 2020-2099 (_ymd_ok: a month 13, an hour 25 past the checksum)."""
    try:
        y, m, dd = int(d[0:4]), int(d[5:7]), int(d[8:10])
        h, mi, ss = int(t[i:i + 2]), int(t[i + 3:i + 5]), int(t[i + 6:i + 8])
    except (TypeError, ValueError, IndexError):
        return None
    if not _ymd_ok(y, m, dd, h, mi, ss):
        return None
    return (_days(y, m, dd) - EPOCH_DAYS) * 86400 + h * 3600 + mi * 60 + ss

def _stamp_s(secs):
    """(ISO time, compact track time) for seconds since 2020."""
    y, m, d = _civil(secs // 86400 + EPOCH_DAYS)
    r = secs % 86400
    v = (y, m, d, r // 3600, r // 60 % 60, r % 60)
    return ("%04d-%02d-%02dT%02d:%02d:%02dZ" % v,
            "%04d%02d%02dT%02d%02d%02dZ" % v)

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
    if la is None or _age(now, la) <= STILL_GAP or sl["gap_seen"] == la:
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

def _still_window(now, pos):
    """Fold a usable fix into the current centroid window. When the window
    is full and its centroid lies within STILL_RADIUS of the previous one,
    return (anchor, stay start time, fixes); a walk moves the centroid on.
    Sums are offsets from the window's first fix: single-precision floats
    cannot add up 60 latitudes to the metre."""
    sl = log["still"]
    _still_gap(now, pos)
    sl["last_at"], sl["pos"] = now, pos
    if sl["win_at"] is None:
        sl.update(win_at=now, win_tc=_tc(), ref=pos, sum=[0.0, 0.0, 0],
                  qs=[])
    r, sm, qs = sl["ref"], sl["sum"], sl["qs"]
    while len(qs) < 3 and (_age(now, sl["win_at"])
                           >= (len(qs) + 1) * (STILL_WINDOW // 4)):
        qs.append((sm[0], sm[1], sm[2]))   # the sums up to each quarter's
                                           # end (see _walking)
    sm[0] += pos[0] - r[0]
    sm[1] += _dlon(pos[1] - r[1])        # (across 180° too)
    sm[2] += 1
    if _age(now, sl["win_at"]) < STILL_WINDOW:
        return None
    if sm[2] < STILL_MIN_FIXES and _age(now, sl["win_at"]) < STILL_WINDOW_MAX:
        return None                      # sparse fixes: collect more first
    c = None
    if sm[2] >= STILL_MIN_FIXES:
        c = (r[0] + sm[0] / sm[2], _dlon(r[1] + sm[1] / sm[2]))
        if _spread(r, c, pos, STILL_EXIT // 2) or _walking(r, sm, qs):
            # its ends STILL_EXIT apart through the centroid: a drive (a
            # bike from ~10 km/h) pairs with nothing. Out and back with the
            # turn near a window boundary, the way out and the way back had
            # the same centroid, and the pair dropped the held lines as
            # drift. (Indoor drift that strays 80 m still pairs.) A walk
            # spreads less: its quarters advance steadily (_walking), and
            # one such window of a pair is enough to tell. A walk on, a
            # walk in before a stop: the pair waits for two calm windows.
            c = None
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

def _still_heartbeat(now):
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
    age = _age(now, c[1])
    if age >= TICKS_OLD:
        return
    t, tc = _stamp_s(c[0] + age // 1000)
    if log["last_t"] is not None and t <= log["last_t"]:
        sl["hb_at"] = time.ticks_add(now, 1000 - STILL_HEARTBEAT)
        return
    a = sl["anchor"]
    sl["hb_at"] = now
    if log["file"] is None:
        start_file(_tcn(tc))
        sl["file_at"] = now
    h = log["hb"]
    h.append(tc)
    if len(h) > 8:                       # (never fed to the route: no TFT)
        h.pop(0)
    _record((t, _line("%.6f,%.6f" % (a[0], a[1]), ("", ""), tc)), a)

def _still_beat(now):
    """Heartbeat of a stay away from home, run every tick, fix or not. Not
    while the last usable sample was recently out past STILL_NEAR: that is
    an excursion or a departure, and an anchor stamped then would land in
    the middle of the way out. Nor for STILL_BACK after that: the car driving
    off past the anchor comes within STILL_NEAR for seconds, and the anchor
    point due since the walk to it, with the way out after it, read 44 m in
    a second (158 km/h, measured). Drift out there every few seconds delays
    it by at most one STILL_HEARTBEAT."""
    sl = log["still"]
    if not sl["active"]:
        return
    if at_home():
        _still_close(now)                # also after a WiFi drop, fix or not
        if not sl["home"] and (sl["last_at"] is None
                               or _age(now, sl["last_at"]) >= STILL_GAP):
            # an away stay, home now, no usable fix for STILL_GAP: the way
            # home had no fix (a train) and no WiFi verdict (no access
            # points at that place). Left running, it stamped the old place
            # next morning and swallowed a trip that did not recede from it
            print("stay end: home without a fix")
            log["still"] = _still_state()
            _rt_trip()
        return
    # Rotate the anchor file so the stay uploads while it lasts (the usual
    # age-close is off during a stay: it would open a file per heartbeat).
    if (log["file"] is not None and sl["file_at"] is not None
            and wifi["state"] == "connected"
            and _age(now, sl["file_at"]) >= STILL_FILE):
        _still_close(now)
    if _recent(now, sl["hb_at"], STILL_HEARTBEAT):
        return
    if (_recent(now, sl["last_at"], STILL_GAP)
            and sl["last_d"] > STILL_NEAR):
        return
    if (_recent(now, sl["far_at"], STILL_BACK)
            and sl["hb_at"] is not None
            and _age(now, sl["hb_at"]) < 2 * STILL_HEARTBEAT):
        return
    # Nor while the departure ring is a run already STILL_RUN out from where
    # it began, still within STILL_NEAR: a walk out. The anchor stamped 25 s
    # into it at 2 m/s, and the way out (only what follows the last line)
    # began 52 m out, a second after it. Late by at most one STILL_HEARTBEAT.
    n, i = sl["rn"], sl["ri"]
    if (n > 1 and _recent(now, sl["last_at"], STILL_GAP)
            and sl["hb_at"] is not None
            and _age(now, sl["hb_at"]) < 2 * STILL_HEARTBEAT
            and _RF[3 * ((i + n - 1) % len(_RT)) + 2] - _RF[3 * i + 2]
            >= STILL_RUN):
        return
    _still_heartbeat(now)

def _still_close(now=None):
    """Home stay: finish the track so it uploads; nothing more is written
    until the stay ends. A retry from the tick (now given) flushes at most
    every FLUSH while a flush keeps failing, as the main path does."""
    if log["file"] is None:
        return
    if (log["batch"] and now is not None
            and _age(now, log["last_flush"]) < FLUSH):
        return
    _close_file()                        # nothing is held during a stay

def _close_file():
    """Close the open track so it uploads, without the drift a stay still
    being proven holds in RAM (a full flush would write it): what is held
    moves on to a new file named after its first line. False when the
    write failed and the file stays open."""
    if log["batch"]:
        flush(_still_hold())             # (with nothing to write, flush
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
        start_file(_tcn(tc))
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

def _still_enter(now, anchor, since_tc, n, c=None):
    """Become stationary at anchor. The stay's drift (lines from since_tc
    on, all still in RAM: see _still_hold) is dropped, and the track is
    closed, so the arrival uploads during the stay. Away from home the
    anchor opens a new file, now and then every STILL_HEARTBEAT.
    Kept: the way in up to where it first comes within STILL_NEAR of c,
    the later window's centroid (the anchor, the mean of both, lies back
    along a way in that the earlier window still held), and on while it
    kept coming closer steadily (each line at most STILL_TOL farther and
    2 * STILL_TOL closer than the one before; drift jumps), when that took
    it STILL_RUN closer over STILL_RUN // 2 lines at least. At walking
    pace the last 50 m were dropped as drift (~36 s). Likewise the newest
    lines when they recede from c that way: the walk on already began in
    the later window (a stop of ~90 s), its start no drift either. Then
    no anchor point is written now: it would come after them, back at the
    stop."""
    log["still"] = sl = _still_state()
    # last_at: the fix that proved it. None read as "no usable fix for
    # STILL_GAP" in _still_beat, so home WiFi coming up just after arriving
    # ended the stay as "home without a fix"
    sl.update(active=True, anchor=anchor, aref=anchor,
              n=min(n, STILL_MEAN_N), last_at=now)
    b = log["batch"]
    i = 0
    while i < len(b) and b[i][-17:-1] < since_tc:
        i += 1
    if c is None:
        c = anchor
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
        while j > i:
            z = _line_m(c, b[j - 1])
            if z > y + STILL_TOL or z < y - 2 * STILL_TOL:
                break
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
    _still_close()                       # the arrival uploads during the stay
    if log["file"] is not None:
        sl["file_at"] = now              # close failed: rotate it later
    if at_home():
        sl["home"] = True
    elif out:
        sl["hb_at"] = now                # (the way out goes on: no anchor
                                         # point after it)
    elif wifi_wake():
        # the radio slept: the first anchor point waits for the round, so
        # a home stay is not stamped with one
        sl["hb_at"] = time.ticks_add(now, WIFI_WAKE_WAIT - STILL_HEARTBEAT)
    else:
        _still_heartbeat(now)
    print("stay", "home" if sl["home"] else "away", "kept", i, out)

def _stay_scan(aps):
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
        now = time.ticks_ms()
        if not any(b in known for r, b in seen):
            sl["ap_miss"] += 1
            if sl["ap_miss_at"] is None:
                sl["ap_miss_at"] = now
            if (sl["ap_miss"] >= STAY_WIFI_MISS
                    and _age(now, sl["ap_miss_at"]) >= STAY_WIFI_GONE):
                print("stay left, wifi: none of %d access points in %d scans"
                      % (len(known), sl["ap_miss"]))
                _diag_write("stay left wifi aps %d scans %d"
                            % (len(known), sl["ap_miss"]))
                _still_leave(wifi_seen=True)
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

def _ring_add(sl, now, pos, d):
    """Keep a usable sample of the stay in the departure ring (see _RT),
    the oldest overwritten when it is full. A sample that falls back from
    the one before by more than STILL_TOL starts it over: the ring is
    always a run that kept receding, the way out (_still_leave). One
    within STILL_DEP_STEP of the one before is left out: at walking pace
    the way out to STILL_EXIT and the samples that prove it took ~120 s,
    and the ring kept the last STILL_DEP_MAX (lost: the first ~30 m)."""
    s = _fix_s()
    if s is None:
        return
    cap = len(_RT)
    n, i = sl["rn"], sl["ri"]
    if n:
        k = 3 * ((i + n - 1) % cap)
        if d + STILL_TOL < _RF[k + 2]:
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
    for x, w in zip(_fields(now), (8, 4)):
        if len(x) <= w:                  # (beyond 99999.9 m: none)
            _RE[k + o:k + o + len(x)] = x.encode()
        o += w

def _still_leave(wifi_seen=False):
    """End the stay and write the way out: the samples in the departure
    ring, a run whose distance from the anchor kept growing (within
    STILL_TOL), so the departure starts near the anchor, not STILL_EXIT
    away. With wifi_seen (the stay's access points are gone, _stay_scan)
    only if the last sample was out past STILL_NEAR: otherwise the ring is
    the stay's drift, and the stay just ends."""
    sl = log["still"]
    n, i0, cap = sl["rn"], sl["ri"], len(_RT)
    _rt_trip(sl["rt0"] + _RT[i0] if n else None)
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
            start_file(_tcn(tc))
        _record((t, line), p)
        prev = p
    print("stay end", int(sl["far_d"]))
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

def _still_active(now, pos):
    """One usable fix while stationary: watch for a departure (consecutive
    samples beyond STILL_EXIT that keep receding; an excursion that turns
    back does not count; after a fix gap, a place out there where the fixes
    stay together), refine the anchor (the heartbeat, the file rotation and
    the home close are _still_beat)."""
    sl = log["still"]
    a = sl["anchor"]
    d = _dist(a[0], a[1], pos[0], pos[1])
    gap = sl["last_at"] is not None and _age(now, sl["last_at"]) > STILL_GAP
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
            sl.update(gap_ref=pos, gap_at=now, gap_n=1)
        elif d + STILL_TOL >= sl["last_d"]:
            sl["gap_n"] += 1
        _ring_add(sl, now, pos, d)
        if _still_recede(sl, d) or d > STILL_GAP_JUMP or (
                sl["gap_n"] >= STILL_MIN_FIXES
                and _age(now, sl["gap_at"]) >= STILL_GAP_STAY):
            if not (sl["home"] and _home_veto(now, sl)):
                sl["far_d"] = d          # (for the "stay end" line)
                _still_leave()
                return
            sl.update(gap_ref=pos, gap_at=now, gap_n=1, far_n=0, far_d=0,
                      far_slow=0)
    elif d > STILL_EXIT and d + STILL_TOL >= sl["far_d"]:
        _ring_add(sl, now, pos, d)
        if _still_recede(sl, d):
            if not (sl["home"] and _home_veto(now, sl)):
                _still_leave()
                return
            sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
    else:
        sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
        sl.update(gap_n=0, gap_at=None, gap_ref=None)
        _ring_add(sl, now, pos, d)
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
            hit = _still_window(now, pos)
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
                    a = hit[0]
                    sl.update(anchor=a, aref=a, aoff=(0.0, 0.0),
                              n=min(hit[2], STILL_MEAN_N), hb_at=None,
                              win_at=None, prev=None, move_c=None)
                    d = _dist(a[0], a[1], pos[0], pos[1])
                    sl["rn"] = 0         # (distances from the old anchor)
                    _ring_add(sl, now, pos, d)
                    print("stay moved")
    sl["last_at"], sl["last_d"] = now, d
    if d > STILL_NEAR:
        sl["far_at"] = now
    home = bool(at_home())
    if home != sl["home"]:
        sl["home"] = home
        sl["hb_at"] = None

def log_tick():
    """Sample a position every SAMPLE ms and record it if it is more than
    MIN_MOVE m from the last recorded point; write a batch every FLUSH ms."""
    now = time.ticks_ms()
    # Close a long-open file on any WiFi network so it can upload while
    # logging continues. Offline driving keeps one file.
    if (log["file"] is not None and log["file_at"] is not None
            and wifi["state"] == "connected" and not log["still"]["active"]
            and log["points"] + len(log["batch"]) > 0
            and _age(now, log["file_at"]) >= NEW_FILE_GAP * 1000
            and _age(now, log["last_flush"]) >= FLUSH):
        _close_file()
    if log["full"]:
        if log["fix"] and not has_fix(now):
            # nothing is recorded, but NEW_FILE_GAP counts from the loss,
            # not from when space returns
            log["fix"], log["lost_at"] = False, now
        last = log["full_check"]
        if _recent(now, last, FULL_CHECK):
            return
        log["full_check"] = now
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
        elif st.get("q"):
            st["q"] = []                 # stale (a new list only if any: this
                                         # runs every pass without a fix)
        # the heartbeat after the queued seconds: a departure in them comes
        # first, and nothing is stamped before lines already written
        _still_beat(now)
        if log["fix"]:
            log["fix"], log["lost_at"] = False, now
            flush(_still_hold())
        elif log["batch"] and _age(now, log["last_flush"]) >= FLUSH:
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
                and _age(now, log["lost_at"]) > NEW_FILE_GAP * 1000):
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
                           and _age(now, log["last_flush"]) < FLUSH)
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
    if _age(now, log["last_flush"]) >= FLUSH:
        flush(_still_hold())

def _drain_q():
    """Sample every averaged second not seen yet, oldest first, each at the
    ticks it was published (its own fix time and freshness)."""
    if st["seq"] == log["seq"]:
        return
    q, st["q"] = st["q"], []
    cur = _snap()
    for snap in q:
        if snap[0] > log["seq"]:
            _load(snap)
            _sample(snap[8])
    _load(cur)
    log["seq"] = cur[0]

def _sample(now):
    """One second of the current fix (st): the heartbeat clock, then the
    stay logic and the track when the fix is usable."""
    secs = _fix_s()
    if secs is None or not 0 <= secs < FIX_S_END:
        return                           # (no date: a second with GGA only,
                                         # e.g. the first after a warm boot;
                                         # or see FIX_S_END)
    log["clock"] = (secs, now)           # GPS time for the heartbeat (2020)
    if not _usable(now):
        return
    lt = log["last_t"]
    if lt is not None:                   # (in a stay too: see _time_back)
        t = st["date"] + "T" + st["time"] + "Z"
        if t < lt:
            _time_back(t, lt)
        else:
            log["back"] = 0
    pos = (st["lat"], st["lon"])
    if log["still"]["active"]:
        _still_active(now, pos)
        return
    s = _sample_line(now, log["prev"], log["last_t"])
    if s is not None:
        _record(s, pos)
    held = _still_hold()
    hit = _still_window(now, pos)
    if hit:
        since = hit[1] if held is None or hit[1] < held else held
        _still_enter(now, hit[0], since, hit[2], hit[3])

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
# others wait.
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
        print("upload: cannot delete, parked", path + ".err")
        _diag_write("upload: %s cannot be deleted, parked as .err" % path)
        return
    up["rm_err"].pop(path, None)

def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass

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

def _seek_footer(f, old):
    """Seek to the last known FOOTER, or after the last newline if truncated
    (-> False: a guess).

    A partial append leaves extra bytes after good_size; overwrite those
    from the last good footer. An externally truncated file is shorter
    and does not end with FOOTER — resume after the last newline."""
    n = len(FOOTER)
    foot = FOOTER.encode()
    if old is not None and old >= n:
        pos = old - n
        f.seek(pos, 0)
        if f.read(n) == foot:
            f.seek(pos, 0)
            return True
        f.seek(0, 2)
        if f.tell() > old:
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
              rm_err={}, env_u=None, env_k=None, srv=None)
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
    size = _file_size(path)
    if size is None:
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
    except OSError:
        return 0, None
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
    while i < 6 and j < n:
        if m & (1 << i):
            while j < n and _rbuf[j] >= 128:
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

def _ymd_ok(y, mo, d, hh, mi, ss):
    """A real UTC time in 2020-2099 (the ranges _kb_time checks, without
    its FIX_S_END bound): month 13 or hour 25 went up as they were."""
    if not (2020 <= y <= 2099 and 1 <= mo <= 12 and d >= 1
            and hh < 24 and mi < 60 and ss < 60):
        return False
    return d <= 28 or d <= _days(y, mo + 1, 1) - _days(y, mo, 1)

def _tc_ok(rb, t):
    """The compact time YYYYMMDDTHHMMSSZ at rb[t:t + 16], its digits
    checked, is a real one (_ymd_ok)."""
    return _ymd_ok((rb[t] - 48) * 1000 + (rb[t + 1] - 48) * 100
                   + (rb[t + 2] - 48) * 10 + rb[t + 3] - 48,
                   (rb[t + 4] - 48) * 10 + rb[t + 5] - 48,
                   (rb[t + 6] - 48) * 10 + rb[t + 7] - 48,
                   (rb[t + 9] - 48) * 10 + rb[t + 10] - 48,
                   (rb[t + 11] - 48) * 10 + rb[t + 12] - 48,
                   (rb[t + 13] - 48) * 10 + rb[t + 14] - 48)

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
    try:
        return float(_rmv[a:b])
    except ValueError:
        return None

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
            or abs(lat) > 90 or abs(lon) > 180):
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

def read_batch(path, off=0, key=None):
    """Fill _body with up to UPLOAD_BATCH valid points from byte offset off
    as Overland JSON -> (n, n_ok, body_len, new_off, eof).
    Truncated / non-numeric / inf / over-long points are skipped.
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
            if n >= UPLOAD_BATCH:
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
    room = free()
    if room is not None and room < HIST_RESERVE:
        room = _hist_trim(HIST_RESERVE)
    if room is not None and room < 2 * MIN_FREE:
        return
    st = [None, False, False, None]      # the open day file (_hist_begin)
    ok = False
    try:
        with open(path, "rb") as src:
            _track_lines(src, path, off, end, key,
                         lambda a, b: _hist_put(st, a, b))
        ok = True
    except Exception as e:               # (best effort: the commit after
        print("hist", repr(e))           # it is what matters)
        log["herr"] += 1
    finally:
        _hist_close(st, ok)

def _hist_put(st, a, b):
    """The line _rbuf[a:b] into the day file of its time (opened when the
    day changes)."""
    d = b - 17                           # YYYYMMDD of its time
    if st[0] is not None:
        k = 0
        while k < 8 and _rbuf[d + k] == _hd8[k]:
            k += 1
        if k == 8:
            _hist_line(st, _rbuf, a, b)
            return
        _hist_close(st)
    for k in range(8):
        c = _rbuf[d + k]
        if not 48 <= c <= 57:
            return                       # (no date: not in a day's file)
        _hd8[k] = c
    _hist_begin(st, bytes(_hd8).decode())
    _hist_line(st, _rbuf, a, b)

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
    if room is not None and room - need < MIN_FREE:
        print("bad file: flash reserve", bad)
        return "flash"
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
    _hist_batch(path, off, new_off, key)
    try:
        commit_offset(path, n, new_off, eof, key1)
    except OSError as e:
        # the copy landed but the offset did not: the next try copies the
        # batch again (duplicates in .bad, deduplicated by Dawarich)
        print("quarantine offset", repr(e))
        _up_err("fs err")
        return False
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

def _schedule(now, ms=None):
    if ms is None:
        b = up["backoff"]
        ms = b
        up["backoff"] = min(max(b * 2, BACKOFF_START), BACKOFF_MAX)
    up["next_at"] = time.ticks_add(now, ms)

SRV_ERR_MAX = 3                          # the same batch answered with a code
                                         # that is no payload error (5xx,
                                         # 404...) in a row: its file held

def _hold_file(path, now, ms):
    """Backoff one file so a stuck oldest track does not block the rest."""
    up["hold"][path] = time.ticks_add(now, ms)

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
            read_uart()
            time.sleep_ms(10)
            continue
        i += n
        t0 = time.ticks_ms()

_tls_ctx = [None, None]                  # the one SSLContext, and the size
                                         # of the CA_FILE it was made with
                                         # (None: there was none)
CA_MAX = 4096                            # bytes of CA_FILE at most (one DER
                                         # root is ~1-2 KB; a PEM bundle
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
        ctx.load_verify_locations(cadata=ca)
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
        _schedule(now, LOWIDF_RETRY)
        if oom:
            _tls_reboot()
        return False
    finally:
        _post_phase[0] = None

_rtc_set = [False]                       # board clock set from the GPS once

def _rtc_sync(now):
    """mbedTLS checks the server certificate's dates against the board
    clock, which starts at 2000-01-01 each boot: with CA_FILE every
    verified POST failed. Set it from the GPS fix (valid, a date in
    2020-2087) before a POST; while it was not set yet and the fix is gone
    (a board that had one outdoors and is indoors now), from the last fix's
    clock run on by the ticks (log["clock"], as _local_hm does), else from
    NTP's answer (ntp_tick: no fix since the boot, or none to set it
    with): once a boot, again only when it drifted more than RTC_DRIFT s.
    Seconds since 2020 (small ints)."""
    src = ""
    if has_fix(now):
        at = st["pos_at"]
        s = _fix_s()
    elif not _rtc_set[0] and (log["clock"] or _ntp[0]):
        # no fix now: the clock a fix gave this boot, else NTP's answer,
        # until a fix sets it
        s, at = log["clock"] or _ntp[0]
        src = " gps" if log["clock"] else " ntp"
    else:
        return
    if s is None or _age(now, at) >= TICKS_OLD:
        return
    s += _age(now, at) // 1000
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
    # (lwIP resolves for as long as its own retries take, ~30 s a server: no
    # socket yet for a timeout or the guard to close, so only the WDT, fed
    # before and after, bounds it)
    feed()
    ai = socket.getaddrinfo(host, port, 0, socket.SOCK_STREAM)[0]
    feed()
    return (socket.socket(ai[0], socket.SOCK_STREAM, ai[2]), ai[-1],
            proto == "https", host, head)

def _http_head(s, want, got):
    """Read an answer's headers, in bounded pieces (a long header comes in
    several, only a piece at a line start is a header or the end). The value
    of a header named in want (lowercase) goes to got at the same index.
    More than HTTP_LINES pieces before the blank line raise: silently
    stopped, the rest of the head read as the body (a GET's sha256 failed
    every check with no clue; a POST keeps its 2xx, the caller catches)."""
    bol = True
    for _ in range(HTTP_LINES):
        h = s.readline(HTTP_LINE)
        if not h or (bol and h in (b"\r\n", b"\n")):
            return
        if bol:
            k, _, v = h.partition(b":")
            k = k.strip().lower()
            if k in want:
                got[want.index(k)] = v.strip()
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
    code, Retry-After, Content-Type), the two headers bytes, None when not
    sent."""
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
            read_uart()
            feed()
        poll = select.poll()
        poll.register(s, select.POLLIN)
        t0 = time.ticks_ms()
        while not poll.poll(200):
            read_uart()
            feed()
            if _age(time.ticks_ms(), t0) >= HTTP_WAIT:
                raise OSError(110)       # ETIMEDOUT
        # what is left of HTTP_WAIT for the status line: a TLS 1.3 server's
        # session tickets right after the handshake are the first POLLIN,
        # and the answer then had only the connect timeout (15 s) to come
        raw.settimeout(max(HTTP_WAIT - _age(time.ticks_ms(), t0), 1000)
                       / 1000)
        line = s.readline(HTTP_LINE)
        code = int(line.split(None, 2)[1])
        got = [None, None]
        try:
            # the status is in: a reset, a timeout or the guard closing
            # the socket in the headers must not turn a 2xx into a resend
            # (a duplicate batch)
            _http_head(s, (b"retry-after", b"content-type"), got)
        except Exception as e:
            print("upload head", code, repr(e))
        return code, got[0], got[1]
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
        if 200 <= code < 300 and ctype and b"html" in ctype.lower():
            # an HTML 2xx is a captive portal (or a proxy) answering, not
            # Dawarich (JSON): without CA_FILE nothing else tells, and the
            # batch counted as sent was deleted
            print("upload: HTML answer, not Dawarich (captive portal?)")
            _diag_write("upload err html %d (captive portal?)" % code)
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

def _tls_reboot():
    """TLS_REBOOT_N TLS allocations failed in a row at home: the IDF heap
    is fragmented for good. Write what is in RAM and restart; at most one
    per LOWIDF_REBOOT_GAP, as _low_reboot (one that does not help, a
    certificate chain over TLS_MIN_KB, rebooted every ~2.5 min)."""
    if up["enomem"] < TLS_REBOOT_N or not at_home():
        return False
    if _ota_wants_radio():
        return False                     # (a reset aborts a pending image)
    if _mem_reboot_soon():
        return False
    print("upload: TLS out of memory %d times, reboot" % up["enomem"])
    _mem_reboot("tls memory %d" % up["enomem"])
    return True

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
    if not at_home() or low_ms < LOWIDF_WAIT or _ota_wants_radio():
        return False                     # (a reset aborts a pending image)
    if _mem_reboot_soon():
        return False                     # rebooted for it a short while ago
    what = fmt % n
    print("upload: low %s for %d s, reboot" % (what, low_ms // 1000))
    _mem_reboot("low %s %d s" % (what, low_ms // 1000))
    return True

def _env_bad(url, key):
    """Why GEO_URL / API_KEY cannot go into a request head, or None. Both
    printable ASCII without a space (a CR in the key split the header, a
    space or a "# comment" got a 400 / 401 for ever); GEO_URL http(s)://
    host[:port][/path], host letters, digits, "-" and "." (userinfo or
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
        if not (c.isalpha() or c.isdigit() or c in "-."):
            ok = False
    return None if ok else "GEO_URL in .env: not http(s)://host[:port][/path]"

def _due(files, held, now):
    """files without those held back (_hold_file) for now."""
    due = []
    for p in files:
        t = held.get(p)
        if t is None or not _waiting(now, t, UP_WAIT_MAX):
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
                print("parked", path + ".err")
                # its points, never sent, leave "left" (it held the OTA);
                # unreadable, it cannot be counted: count the rest
                up["left"] = count_points()
            except OSError:
                pass
        _hold_file(path, now, BACKOFF_MAX)
        _schedule(now)
        return
    except MemoryError:
        up["skip"] = path
        _low_mem(now)
        return
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
        _schedule(now, UPLOAD_GAP)
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
            _schedule(now, LOWIDF_RETRY)
        return
    up["lowpy_at"] = None
    if tls:
        _rtc_sync(now)                   # certificate dates need the time
        if not _tls_ready(now):
            return
    if not _bad_full_left():
        up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
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
    read_uart()                          # catch up after a multi-second POST
    now = time.ticks_ms()
    print("upload", path, n, n_ok, code, py_heap()[0],
          "%d ms" % time.ticks_diff(now, t_post))
    if code is None and _tls_reboot():
        return
    gap = UPLOAD_GAP_AWAY if not at_home() else UPLOAD_GAP
    if code is not None and 200 <= code < 300:
        # the day history gets the points once they are up (at flush they
        # were on the flash twice while they waited)
        _hist_batch(path, off, new_off, key)
        try:
            commit_offset(path, n, new_off, eof, key1)
        except OSError:
            _fs_err(now)
            return
        up["sent"] += n_ok
        up["ok_at"] = now
        up["fail4"] = 0
        up["srv"] = None
        up["backoff"] = BACKOFF_START
        up["allow_bad"] = True
        up["skip"] = None
        up["hold"].pop(path, None)
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
        _schedule(now, retry)
        if code is not None and len(files) > 1:
            # the same batch answered so SRV_ERR_MAX times in a row (a 5xx
            # from one point, a 404 / 403 of a proxy): hold that track, the
            # others go (only the backoff: files[0] for ever, the rest
            # blocked behind it). The only one waiting: the backoff alone
            se = up["srv"]
            if se and se[0] == path and se[1] == off:
                se[2] += 1
            else:
                se = up["srv"] = [path, off, 1]
            if se[2] >= SRV_ERR_MAX:
                up["srv"] = None
                _hold_file(path, now, BACKOFF_MAX)

def _fs_err(now):
    """A .pos write (or delete) failed: retry later, the track stays."""
    _up_err("fs err")
    _schedule(now)

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
# Tried again after OTA_RETRY, bad after OTA_TRIES_MAX. No set_boot() back:
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
OTA_TRIES_MAX = 2                        # such rollbacks before it is bad
OTA_PROBE = 60000                        # ms between manifest reads of a
                                         # pending firmware
OTA_MARK = "/ota.mark"                   # what ended the pending run: crash,
                                         # stop (Ctrl-C), boot (ota_boot's
                                         # reset into a slot not run yet)
OTA_SHADOW = ("/gps_logger.mpy", "/gps_logger.py")  # moved aside (.ota-old)
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

ota = {"at": None, "state": "off", "boot_at": time.ticks_ms(), "pend": None,
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
        print("OTA_URL / OTA_KEY: both needed, http(s) URL - no updates")

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

def _ota_url(rel):
    """A manifest URL relative to the manifest's own folder."""
    if rel.startswith(("http://", "https://")):
        return rel
    return OTA_URL[:OTA_URL.rfind("/") + 1] + rel

def _raw_get(url, sink, limit):
    """A minimal HTTP/1.0 GET, no redirects: the body goes to sink(memoryview)
    in pieces, the UART read and the WDT fed between, at most limit bytes
    (more: ValueError). The guard closes the socket after OTA_MAX. -> the
    status code (sink only gets a 200's body)."""
    s, addr, https, host, hdr = _http_open("GET", url)
    _guard_start(s, OTA_MAX)
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
        _http_head(s, (b"content-length",), cl)
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
            read_uart()
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
        read_uart()
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
        read_uart()
        feed()
    return binascii.hexlify(h.digest()).decode()

def _ota_bad():
    try:
        with open(OTA_BAD) as f:
            return [int(x) for x in f.read().split()]
    except (OSError, ValueError):
        return []

def _ota_mark_bad(build):
    try:
        with open(OTA_BAD, "a") as f:
            f.write("%d\n" % build)
    except OSError:
        pass

def _ota_tries():
    """(build, n) of OTA_TRIES, or (None, 0)."""
    try:
        with open(OTA_TRIES) as f:
            b, n = f.read().split()
        return int(b), int(n)
    except (OSError, ValueError):
        return None, 0

def _ota_try(build):
    """A rollback that may not be the build's fault: at power-on (MicroPython
    1.29 reports a brownout as one: a weak supply), or no WiFi or manifest
    in OTA_HEALTH (the router, the server). The OTA_TRIES_MAX-th of one
    build marks it bad: never blamed, a brownout reinstalled it after every
    boot; blamed at once, a router outage excluded a good build for good.
    -> True: now bad."""
    b, n = _ota_tries()
    n = n + 1 if b == build else 1
    if n >= OTA_TRIES_MAX:
        _ota_mark_bad(build)
        _rm(OTA_TRIES)
        return True
    try:
        with open(OTA_TRIES, "w") as f:
            f.write("%d %d\n" % (build, n))
    except OSError:
        pass
    return False

def _ota_wait_tried():
    """A build with a try behind it is installed again OTA_RETRY after
    boot, not OTA_FIRST (ota["failed"] is RAM: gone with the reset)."""
    b = _ota_tries()[0]
    if b is not None:
        ota["failed"] = (b, time.ticks_ms(), "rolled back")

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

def _ota_shadow(aside):
    """A /gps_logger.mpy or .py (FORCE_MPY=1) shadows the frozen logger:
    '' comes before .frozen in sys.path, and after an OTA the diag said
    "ota <new> ok" while the old code ran. aside: moved to <name>.ota-old
    (before set_boot); else put back (an install that failed; after a
    rollback main.py does it, before the import). Only on a frozen
    firmware (FW_BUILD): a plain one needs its .mpy to boot."""
    for p in OTA_SHADOW:
        try:
            if aside:
                os.rename(p, p + ".ota-old")
            elif _file_size(p) is None:
                os.rename(p + ".ota-old", p)
        except OSError:
            pass

def _ota_mark(what):
    """What ended a run while an OTA note is there, for the firmware that
    reads the note next (the old one, after a rollback): "crash" (run()'s
    crash branch: the build is bad), "stop" (Ctrl-C, an mpremote session:
    the watchdog reset after it is not the build's fault), "boot"
    (ota_boot's reset into a slot set to boot but not run). Never
    raises."""
    try:
        if _file_size(OTA_PENDING) is None:
            return
        with open(OTA_MARK, "w") as f:
            f.write(what)
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
    if build <= FW_BUILD or build in _ota_bad():
        ota["state"] = "current"
        return
    f = ota.get("failed")
    if (f is not None and f[0] == build
            and _age(time.ticks_ms(), f[1]) < OTA_RETRY):
        ota["state"] = f[2]              # (2 MB every OTA_CHECK: no)
        return
    if not _ota_eq(_hmac_hex(OTA_KEY, _ota_signed(m)), m.get("hmac")):
        ota["state"] = "bad hmac"
        # (no unverified string in it: the build is an int)
        _ota_say("ota build %d: manifest hmac does not match" % build)
        return
    ota["said"] = None
    try:
        part = esp32.Partition(esp32.Partition.RUNNING).get_next_update()
    except (OSError, AttributeError):
        ota["state"] = "no ota slot"
        return
    if m["size"] > part.info()[3]:
        ota["state"] = "too big"
        return
    ota["state"] = "downloading"
    w = _SlotWriter(part)
    try:
        code = HTTP_GET(_ota_url(m["url"]), w, m["size"])
    except Exception:
        # a download cut short waits OTA_RETRY like a bad one: it came
        # again (2 MB, the loop blocked) every OTA_CHECK (whatever it
        # raised: an empty status line, a MemoryError)
        ota["failed"] = (build, time.ticks_ms(), "download err")
        raise
    got = w.close()
    if (code != 200 or got != m["sha256"]
            or _slot_sha(part, m["size"]) != m["sha256"]):
        ota["failed"] = (build, time.ticks_ms(), "bad image")
        ota["state"] = "bad image"
        _diag_write("ota %s: download %s, sha256 mismatch" % (m["commit"],
                                                             code))
        return
    paths = []
    try:
        for f in m.get("files") or ():
            _ota_file(f)
            paths.append(f["path"])
        with open(OTA_PENDING, "w") as f:
            f.write("%d %s\n%s" % (build, m["commit"], "\n".join(paths)))
        if FW_BUILD:
            _ota_shadow(True)
        part.set_boot()
    except BaseException:
        # (a Ctrl-C too: it is no Exception) nothing boots: no pending note
        # (it would blame a build that never ran), no staged page, the
        # shadowing module back; a page that failed its sha256 or could not
        # be written waits OTA_RETRY too (the firmware came every check).
        # Unless set_boot took effect (a Ctrl-C right after it): reset
        if _ota_boot_slot() != part.info()[4]:
            _rm(OTA_PENDING)
            _ota_drop(paths)
            _ota_shadow(False)
            ota["failed"] = (build, time.ticks_ms(), "bad page")
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
    A non-OTA or never-pending image raises: nothing to confirm."""
    try:
        esp32.Partition.mark_app_valid_cancel_rollback()
    except (OSError, AttributeError):
        pass

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
    an image left pending would be aborted at the next reset."""
    p = _ota_pending()
    if p is None:
        _ota_wait_tried()
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
        _ota_wait_tried()
        if esp32 is not None:
            _ota_valid()
        _diag_write("ota note unreadable, dropped (running %s)" % FW_COMMIT)
        return
    mark = _ota_mark_get()
    if build == FW_BUILD:
        _rm(OTA_MARK)                    # (a run before this one stopped)
        ota.update(pend=build, pend_at=time.ticks_ms(), wifi_seen=False,
                   probed=False, probe_at=None, probe_err=None,
                   state="pending", paths=paths)
        _diag_write("ota %s booted, checking" % c)
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
    ota["state"] = "rolled back"
    try:
        cause = machine.reset_cause()
        wdt = cause == machine.WDT_RESET
        power = cause == machine.PWRON_RESET
    except Exception:
        cause, wdt, power = None, False, False
    if mark == "crash" or (wdt and mark != "stop"):
        _ota_mark_bad(build)
        _diag_write("ota %s %s, rolled back by the bootloader: bad (running "
                    "%s)" % (c, "crashed" if mark == "crash" else
                             "watchdog reset", FW_COMMIT))
    else:
        why = "a power-off" if power else "a reset (cause %s)" % cause
        if _ota_try(build):
            _diag_write("ota %s rolled back after %s, %d times: bad "
                        "(running %s)" % (c, why, OTA_TRIES_MAX, FW_COMMIT))
        else:
            _ota_wait_tried()
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
        if err:
            ota["probe_err"] = err
    except Exception as e:
        ota["probe_err"] = repr(e)[:40]

def _ota_health(now):
    """The pending firmware: valid after OTA_VALID_AFTER with WiFi connected
    and the manifest read (without OTA_URL WiFi only; then the staged page
    goes up); neither in OTA_HEALTH and it resets, the bootloader then
    boots the old image (a try: _ota_try)."""
    if wifi["state"] == "connected":
        ota["wifi_seen"] = True
        if OTA_URL and not ota.get("probed"):
            _ota_probe(now)
    age = _age(now, ota["pend_at"])
    if (ota["wifi_seen"] and (ota.get("probed") or not OTA_URL)
            and age >= OTA_VALID_AFTER):
        _ota_valid()
        _ota_promote(ota.get("paths") or ())
        for p in OTA_SHADOW:             # (before the note: main.py puts
            _rm(p + ".ota-old")          # back what has no note)
        _rm(OTA_PENDING)
        _rm(OTA_TRIES)
        _rm(OTA_MARK)
        # the pending state ends last: a MemoryError in a step above (the
        # guard swallows it) leaves it set, and the next tick runs the
        # steps again (they are safe to), not a RAM state of "done" over
        # a note, staged pages and .ota-old still on flash
        ota["pend"] = None
        ota["state"] = "ok"
        _diag_write("ota %s ok" % FW_COMMIT)
        return
    if age < OTA_HEALTH:
        return
    build = ota["pend"]
    ota["pend"] = None
    try:                                 # (the reset whatever fails: the
        _ota_drop(ota.get("paths") or ())   # image ran on unconfirmed)
        _ota_shadow(False)
        # the note first: a power cut between the two then loses the try;
        # the other way round the old firmware found it and counted again
        _rm(OTA_PENDING)
        bad = _ota_try(build)
        _diag_final()
        what = "wifi"
        if ota["wifi_seen"]:
            what = "manifest (%s)" % ota.get("probe_err")
        _diag_write("ota %s no %s in %d s, rolling back%s" % (
            FW_COMMIT, what, OTA_HEALTH // 1000, ", bad" if bad else ""))
        flush(_still_hold())             # (as before the other reboots)
    finally:
        machine.reset()

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
            ota["held_at"] = now
            ota["held_ok"] = ok
            ota["stuck"] = False
            return
        if _age(now, ota["held_at"]) < OTA_STUCK:
            return
        ota["held_at"] = time.ticks_add(now, -OTA_STUCK)  # (past a wrap)
        if not ota.get("stuck"):
            ota["stuck"] = True
            _diag_write("ota: uploads stuck %d h (%s), checking anyway" % (
                OTA_STUCK // 3600000, up["state"]))
    elif ota.get("held_at") is not None:    # (no dict per loop pass)
        ota["held_at"] = None
        ota["stuck"] = False
    at = ota["at"]
    if _age(now, ota["boot_at"] if at is None else at) < (
            OTA_FIRST if at is None else OTA_CHECK):
        return
    ota["at"] = now
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
# A small HTTP page on the LAN while the radio is connected (at home, or
# everywhere with WEB_AWAY, it stays connected for it, see wifi_tick):
# http://<ip>/ and /status.json.
# Served from the main loop, one client at a time: each recv/send waits at
# most WEB_TIMEOUT s and the whole request at most WEB_DEADLINE s (a slow
# client used to hold the loop for ~50 s: 2 s per 1 KB send).

_web_sock = None
_web_poll = None                         # its poll, None: accept() asks itself

def _web_env(env):
    """WEB=0 in .env: no status page, the radio sleeps at home as before.
    WEB_AWAY=1: the radio stays on for the page on a non-home network too."""
    global WEB, WEB_AWAY
    v = _plain(env.get("WEB"))
    if v:
        t = _truthy(v)
        if t is None:
            print("WEB not 1/0, using", 1 if WEB else 0)
        else:
            WEB = t
    v = _plain(env.get("WEB_AWAY"))
    if v:
        t = _truthy(v)
        if t is None:
            print("WEB_AWAY not 1/0, using", 1 if WEB_AWAY else 0)
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
    view = sv_seen()
    t = _fix_s()
    pos = pos6 = None
    # a silent receiver keeps its last second's valid flag: no position
    # once that second is older than NO_DATA (the page would show it live)
    pa = st.get("pos_at")
    if (st["valid"] and st["lat"] is not None and st["lon"] is not None
            and pa is not None and _age(now, pa) <= NO_DATA):
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
    return [
        ("fix", status()),
        ("sats", "%d / %s" % (view, sv_used())),
        ("hdop", st["hdop"]),
        ("std", st.get("std")),          # (GST: the receiver's own error, m)
        ("utc", None if t is None else _stamp_s(t)[0]),
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
        ("sent", up["sent"]),
        ("upload", up["state"]),
        ("upload_err", up["err"]),
        ("upload_age_s", None if up["ok_at"] is None
         else _age(now, up["ok_at"]) // 1000),
        ("days", _hist_days()),
        ("stay", "yes" if still.get("active") else "no"),
        ("uptime", uptime_s()),
        ("prev_uptime", prev_run["up"]),
        ("prev_run", _prev_text()),
        ("prev_end", None if prev_run["end"] is None
         else prev_run["end"].replace(" ", "T")),
        ("boot_cause", cause),
        ("temp", _temp()),
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
        ("web_errs", _web_errs[0]),      # page requests that hit a bug
        ("track_drop", log["drop"]),   # points lost (full flash,
        ("write_errs", log["err"]),    # MAX_BATCH, time back), track
        ("q_thin", st.get("q_thin", 0)),      # write errors, seconds thinned
    ]                                    # while the loop was blocked

_fl = [None, None]                       # (KB, ticks) of the last statvfs

def _flash_kb():
    """Free flash in KB, at most every 30 s: statvfs walks the whole
    LittleFS (slow, and the page asks every WEB_REFRESH)."""
    now = time.ticks_ms()
    if _recent(now, _fl[1], 30000):
        return _fl[0]
    try:
        v = os.statvfs("/")
        _fl[0] = v[0] * v[3] // 1024
    except OSError:
        _fl[0] = None
    _fl[1] = now
    return _fl[0]

_hd = [[], None]                         # (day history days, ticks)

def _hist_days():
    """The UTC days (YYYYMMDD) of the day history, oldest first, a day
    once (.trb, or an older firmware's .trk), listed at most every 30 s
    (the page picks a day from them)."""
    now = time.ticks_ms()
    if _recent(now, _hd[1], 30000):
        return _hd[0]
    try:
        _hd[0] = sorted(set(n[:8] for n in _hist_names()))
    except OSError:
        _hd[0] = []
    _hd[1] = now
    return _hd[0]

_fc = [0, 0, None]                       # (files, bad files, ticks)

def _file_counts():
    """(track files waiting, .bad files) for the page, counted at most every
    10 s: pending() and pending_bad() list, sort and build the paths of the
    whole log directory, twice per poll. ilistdir counts without a list."""
    now = time.ticks_ms()
    if _recent(now, _fc[2], 10000):
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
    except OSError:
        pass
    _fc[0], _fc[1], _fc[2] = n, bad, now
    return n, bad

def _sv_list():
    """The satellite list for the page, used first, then by signal."""
    if st["svs"] is None:
        return []
    return sorted(st["svs"], key=lambda v: (not v[4], -v[1], v[6] or 0,
                                            v[0]))

def _web_json_parts(rows, svs):
    """The JSON in small pieces (no 8 KB string and its encoded copy).
    rows, svs: _web_rows() and _sv_list(), built by _web_status before
    the 200 (a bug in one is a 500, not a cut JSON); the rows go first
    and are let go before the diag lines are read."""
    yield '{"device":' + json.dumps(DEVICE_ID) + ',"refresh_s":%d' % (
        WEB_REFRESH)
    for key, v in rows:
        yield ",%s:%s" % (json.dumps(key), json.dumps(v))
    rows = None
    # after the 200: an error ends the list instead of the JSON (a cut one
    # was OFFLINE on the page), counted as a web bug
    yield ',"diag":['
    sep = ""
    try:
        for line in _web_diag():         # one piece a line (no ~3 KB string)
            yield sep + json.dumps(line)
            sep = ","
    except Exception as e:
        _web_bug(e)
    yield '],"satellites":['
    sep = ""
    try:
        for v in svs:
            yield sep + json.dumps({
                "name": "%s %d" % (GNSS.get(v[6], "SV"), v[0]), "system": GNSS.get(v[6]), "svid": v[0],
                "cno": v[1], "elev": v[2], "azim": v[3], "used": v[4],
                "unhealthy": v[5]})
            sep = ","
        at = st["svs_at"]
        age = json.dumps(None if at is None
                         else _age(time.ticks_ms(), at) // 1000)
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
            read_uart()
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
    h = r[i + 6:j].strip()
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
            read_uart()                  # a slow client costs no NMEA
        if k > WEB_CHUNK:
            bv = memoryview(b)
            for i in range(0, k, WEB_CHUNK):
                _web_out(c, bv[i:i + WEB_CHUNK])
                read_uart()
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
            read_uart()
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
        read_uart()
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
    path = parts[1] if len(parts) > 1 else b"/"
    parts = None
    if path.startswith(b"/status.json"):
        _web_status(c)
    elif path.startswith(b"/hist?"):
        _web_hist(c, path)
    elif path.startswith(b"/diag?"):
        _web_diag_file(c, path)
    elif path == b"/" or path.startswith(b"/?"):
        _web_page(c, _web_gzip(req))
    else:
        _web_send(c, "404 Not Found", "text/plain", ("not found",))

def _web_status(c):
    """status.json: its rows and satellite list built before the 200: a
    bug in one is a 500 with the error (it was a 200 with a cut JSON, and
    the page said OFFLINE)."""
    try:
        parts = _web_json_parts(_web_rows(), _sv_list())
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
    bytes at most, no X-Hist-Next (the page counts bytes). A missing day is
    a 404."""
    d, o = "", 0
    try:
        for kv in path[6:].decode().split("&"):
            if kv.startswith("d="):
                d = kv[2:]
            elif kv.startswith("o="):
                o = int(kv[2:])
    except (UnicodeError, ValueError):
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
        _web_send(c, "404 Not Found", "text/plain", ("no such day",))
        return
    try:
        size = os.stat(name)[6]
        if name.endswith(TRB):
            n = len(FOOTER)
            end = size
            if size >= n:
                f.seek(size - n)
                if f.read(n) == FOOTER.encode():
                    end = size - n       # (the next append's keyframe)
            o = min(o, end)
            k, nxt = _hist_lines(f, o, end, None)
            _web_head(c, "200 OK", "text/plain",
                      "Content-Length: %d\r\nX-Hist-Next: %d\r\n"
                      "X-Hist-End: %d\r\n" % (k, nxt, end))
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
        read_uart()

def _web_diag_file(c, path):
    """/diag?f=<0|1>&o=N: the diag log as it is on flash, DIAG_FILE (f=0)
    or DIAG_OLD (f=1), from byte N, HIST_REPLY bytes at most (the page
    reads both files in pieces, then DIAG_FILE from where it got to once a
    minute), X-Diag-Size the file's size (smaller than where the page got
    to: it rotated, the page reads both again). A missing file, a bad f
    or o is a 404."""
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
        _web_bytes(c, f, o, size, "X-Diag-Size: %d\r\n" % size)
    finally:
        f.close()

_kw = [None] * 12                        # the /hist reply's decoder state

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
            read_uart()
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
            read_uart()
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
_bugs = []                               # the kinds of caught bugs in diag
BUG_KINDS = 8                            # at most this many kinds a boot

def _bug_once(line):
    """A caught bug's diag line, once a kind (a bug every request or every
    loop must not fill the log), BUG_KINDS kinds at most."""
    if line in _bugs or len(_bugs) >= BUG_KINDS:
        return
    _bugs.append(line)
    _diag_write(line)

def _web_bug(e):
    """A bug on a page request: counted (web_errs in the JSON), each new
    kind in diag (only the first since boot was). It must not reset the
    logger (run()'s handler)."""
    print("web error", repr(e))
    _web_errs[0] += 1
    _bug_once("web error %s" % repr(e)[:60])

def web_tick():
    """Listen while connected, close otherwise; answer one waiting client
    within WEB_DEADLINE. On any network: WEB_AWAY keeps the radio on for the
    page off home (wifi_tick), without it the page is up as long as the radio
    is, for the uploads (README, status page)."""
    if not WEB or wifi["state"] != "connected":
        _web_close()
        return
    if _web_sock is None:
        _web_open()
        if _web_sock is None:
            return
    if _web_poll is not None:
        for _ in _web_poll.ipoll(0):
            break
        else:
            return                       # nobody waiting
    try:
        c = _web_sock.accept()[0]
    except OSError:
        return                           # nobody waiting (EAGAIN)
    _web_end[0] = time.ticks_add(time.ticks_ms(), WEB_DEADLINE * 1000)
    try:
        req = _web_read(c)
        read_uart()
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
    if st["rx"] is None or _age(time.ticks_ms(), st["rx"]) > NO_DATA:
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
            print("TFT_LIGHT not 0-100, using", TFT_LIGHT)
    v = _plain(env.get("TFT_IDLE"))
    if v:
        try:
            n = int(v)
        except ValueError:
            n = -1
        if 0 <= n <= 86400:
            TFT_IDLE = n * 1000
        else:
            print("TFT_IDLE not 0-86400 s, using", TFT_IDLE // 1000)

def _tft_light(now):
    """The backlight: full for TFT_WAKE after the USER button, else dark once a
    stay is TFT_IDLE old, else TFT_LIGHT. A stay: the GPS says the board has
    not moved, or it is on the home network and no usable fix came for
    STILL_GAP (on a desk indoors, where none proves a stay). A change is said
    in the diag, the first setting is not."""
    L = _tft_l
    sl = log["still"]
    if sl["active"] or (at_home() and not _recent(now, sl["last_at"], STILL_GAP)):
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
                                         # is not bound by it (MicroPython's
                                         # getaddrinfo has no timeout: lwIP's
                                         # own, tens of s), feed() around it; the
                                         # UART buffer holds 3.5 s of NMEA, so a
                                         # slow DNS loses some (no fix yet then)
NTP_ERA = 252460800                      # s (8 years): the next era (2036-02-07 on)
                                         # is believed only for a build this young
NTP_RETRY = 60000                        # ms after a failed try, doubling to ...
NTP_RETRY_MAX = 3600000                  # ... this
NTP_AGAIN = 21600000                     # ms an answer is used before a new one
_NTP_2020 = 3786825600                   # 2020-01-01 in seconds since 1900
_ntp = [None, None, NTP_RETRY]           # (seconds since 2020, ticks) of the answer;
                                         # ticks of the last try; the wait after it

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
        feed()
        ai = socket.getaddrinfo(NTP_SERVER, 123, 0, socket.SOCK_DGRAM)[0]
        feed()
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
    lo = max(0, FW_BUILD - EPOCH_DAYS * 86400)
    if t < 0:
        t += 1 << 32
        if t > lo + NTP_ERA:
            return None
    return t if lo <= t < FIX_S_END else None

def _need_rtc():
    """The board clock matters (a verified TLS request checks the dates of
    the certificate) and nothing has set it yet."""
    return not _rtc_set[0] and ((up["url"] or "").startswith("https")
                                or (OTA_URL or "").startswith("https"))

def ntp_tick(now):
    n = _ntp
    if (not NTP_SERVER or wifi["state"] != "connected"
            or (log["clock"] is not None and not _need_rtc())
            or _recent(now, n[1], n[2])):
        return
    n[1] = now
    read_uart()
    feed()
    t = _ntp_ask()
    feed()
    read_uart()
    if t is None:
        n[2] = min(n[2] * 2, NTP_RETRY_MAX)
        return
    n[0], n[2] = (t, time.ticks_ms()), NTP_AGAIN
    _tft_t[4] = 0
    _diag_write("clock ntp %s" % _stamp_s(t)[0])

CLOCK_ON = 3600000                       # ms a clock stamp runs on before rebased

def _clock_on(now, c):
    """c, a (seconds since 2020, ticks) clock stamp (log["clock"], _ntp[0]), moved up
    to now by whole seconds once it is CLOCK_ON old (called every loop). Only a fix
    renews log["clock"], and past 2**29 ms (6.2 days indoors) its age reads
    TICKS_OLD (--:--, the route's day stuck), past 2**30 young again (12 days
    early)."""
    if c is None:
        return c
    a = _age(now, c[1])
    if a < CLOCK_ON or a >= TICKS_OLD:
        return c
    a //= 1000
    return (c[0] + a, time.ticks_add(c[1], a * 1000))

def _local_hm():
    """"HH:MM" in Hungary from the fix's UTC; with the receiver silent (its time
    stands still then) or no fix yet, from the last fix's clock run on by the
    ticks, else NTP's (as _rt_tick); "--:--" without any."""
    d, t = st["date"], st["time"]
    rx = st["rx"]
    now = time.ticks_ms()
    if not d or not t or not _recent(now, rx, NO_DATA):
        c = log["clock"] or _ntp[0]
        if c is None or _age(now, c[1]) >= TICKS_OLD:
            return "--:--"
        d = _stamp_s(c[0] + _age(now, c[1]) // 1000)[0]
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
    return 3 if hdop <= TFT_HDOP_GOOD and (sv_used() or 0) >= TFT_SATS_GOOD else 2

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
ROUTE_W, ROUTE_H = 83, 51                # the pixels a point can land on
ROUTE_DRAW = 10000                       # ms between looks at the route
ROUTE_SAVE = 300000                      # ms between saves of a changed route (and
                                         # one when a stay starts)
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
    drawn = saved = None                 # ticks of the last look and save
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

def _rt_trip(s=None):
    """The next line, or the one at s seconds since 2020, starts a new trip."""
    try:
        _Rt.trip = _tc() if s is None else _stamp_s(s)[1]
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
    R.saved = now
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
    if c is not None and _age(now, c[1]) < TICKS_OLD:
        d = _rt_day(c[0] + _age(now, c[1]) // 1000)
        if R.waiting or d > R.day:
            if d != R.day:
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
    if R.unsaved and (not _recent(now, R.saved, ROUTE_SAVE)
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
    return (_Rt.view, _Rt.km, fix, sv_used() or 0, _local_hm(), level, _gps_quality(fix),
            text, bool(left), _store_state(now))

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
        reset()
        log_reset()
        wifi_reset()
        up_reset()
        env = load_env(ENV_FILE)
        _gps_sys_env(env)
        _min_move_env(env)
        _tft_env(env)
        _tft_start()                     # (at its light: .env is read)
        _diag_env(env)
        _diag_boot()
        ota_boot()
        gps_setup()
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
_guard_mem = {"wifi": [0, None], "upload": [0, None],   # name -> [count, ticks
              "ota": [0, None], "web": [0, None],       # of the last one]:
              "ntp": [0, None], "status": [0, None],    # made here, not under
              "diag": [0, None]}                        # a MemoryError

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
        pending_ota = pend is not None or ota["pend"] is not None
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
            if not _recent(now, c[1], GUARD_MEM_QUIET):
                c[0] = 0
            c[0] += 1
            c[1] = now
            if pending_ota:
                if c[0] >= GUARD_PEND_MAX:
                    raise
            elif c[0] >= GUARD_MEM_MAX and not _mem_reboot_soon():
                c[0] = 0
                _mem_reboot("guarded memory " + name)
        if name == "upload":
            # (after the POST, before the commit: the same batch went up
            # again every TICK_PAUSE for ever)
            _schedule(now)
        try:                             # (the telling allocates: it must not
            _tick_bug[name] = now        # turn a MemoryError into a crash)
            print("bug", name, repr(e))
            _bug_once("bug %s %s" % (name, repr(e)[:60]))
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
        now = time.ticks_ms()
        log["clock"] = _clock_on(now, log["clock"])
        _ntp[0] = _clock_on(now, _ntp[0])
        read_uart()
        gps_check(time.ticks_ms())
        log_tick()
        _guarded("wifi", wifi_tick)
        keepalive_tick(time.ticks_ms())
        feed()
        _guarded("upload", upload_tick)
        _guarded("web", web_tick)
        _guarded("ota", ota_tick, time.ticks_ms())
        _guarded("ntp", ntp_tick, time.ticks_ms())
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
        time.sleep_ms(20)
