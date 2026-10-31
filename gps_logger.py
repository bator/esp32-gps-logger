from machine import Pin, I2C, UART, WDT
import machine, sys, time, os, network, gc, math, socket, json, select, struct
try:
    import esp32                         # mcu_temperature() on C3/C6/S2/S3
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

BAUD = 9600                              # NEO-6M/7M/8M default
GPS_HZ = 5                               # fixes a second (NEO-6M max 5);
                                         # one averaged point a second is kept
SECONDS_Q = 8                            # published seconds kept for log_tick
GPS_ACK_WAIT = 1000                      # ms to wait for a UBX-ACK per send
GPS_ACK_TRIES = 4                        # sends of one frame before giving up
GPS_SETUP_MAX = 4000                     # ms a setup may block the loop: the
                                         # frames left then go out once,
                                         # without waiting for an ACK
GPS_MAX_STD = 20                         # m: receiver's own error estimate
                                         # (GST) above this = not usable
GPS_PVT = True                           # u-blox M8/M10: UBX-NAV-PVT every
                                         # fix instead of NMEA (.env GPS_PVT:
                                         # auto, 0 = NMEA on every receiver)
PVT_MAX_HACC = 20                        # m: NAV-PVT's horizontal accuracy
                                         # estimate above this = not usable
PVT_MAX_TACC = 1000000000                # ns: NAV-PVT time this accurate
                                         # (and valid) may set the clock
GPS_RETRY = 5000                         # ms: setup sent again if 1 Hz
GPS_RETRY_MAX = 300000                   # sentences still come this long after
GPS_IGNORED_MAX = 60000                  # ms cap between setups a receiver
                                         # acknowledges but does not take
GPS_VERIFY = 10000                       # ms after an acknowledged setup:
                                         # still one fix a second, or no GST
                                         # = not taken, set up again
GPS_VERIFY_MAX = 1800000                 # ms it waits for 3 valid seconds to
                                         # tell (none before the first fix)
GPS_NOT_TAKEN_SAY = 600000               # ms between "setup not taken" diag
                                         # lines (the ones between counted)
# UBX-CFG-NAV5: how the receiver's own filter is tuned (sent with the setup)
GPS_DYN_MODEL = 0                        # 0 portable (car and foot alike),
                                         # 2 stationary, 3 pedestrian,
                                         # 4 automotive; .env GPS_MODEL
GPS_MODELS = (0, 2, 3, 4, 5, 6, 7, 8)    # u-blox dynModel values
GPS_FIX_MODE = 2                         # 1 2D only, 2 3D only, 3 auto. 2D
                                         # fixes guess the altitude and are
                                         # tens of metres off horizontally
GPS_MIN_ELEV = 10                        # deg: ignore satellites lower than
                                         # this (the multipath-prone ones)
GPS_STATIC_HOLD = 50                     # cm/s: below this speed the receiver
                                         # freezes its position (no parked
                                         # zigzag); 0 = off. A walk is 140.
AVG_MAX_SPEED = 10                       # km/h: above this a second's point
                                         # is its first fix, not the mean (the
                                         # mean lies ~0.4 s along the track:
                                         # 6 m at 50 km/h, stamped the second)
CPU_MHZ = 80                             # CPU clock in the main loop (ESP32:
                                         # 20/40/80/160/240, C6: 80/160);
                                         # 0 leaves the firmware default
CPU_MHZ_UPLOAD = 160                     # clock during a POST (TLS); 0 = no
                                         # change. .env CPU_MHZ / CPU_MHZ_UP
STATUS = 600000                          # ms between console status lines
DIAG = 60000                             # ms between diagnostic log lines
                                         # (0 = off; .env DIAG=<seconds>)
DIAG_FILE = "/diag.log"                  # survives power-off: what happened
DIAG_OLD = "/diag.1"                     # on a walk, read it back over USB
DIAG_MAX = 32768                         # bytes, then DIAG_FILE -> DIAG_OLD
DIAG_LINE = 220                          # bytes read back per line wanted
                                         # (status lines are ~180)
DIAG_BLOCK = 512                         # bytes per read of the tail
WEB = True                               # status page on port WEB_PORT; the
WEB_PORT = 80                            # radio stays on at home for it
                                         # (.env WEB=0)
WEB_TIMEOUT = 2                          # s one recv/send may wait
WEB_DEADLINE = 5                         # s a whole request may take
WEB_REQ_MAX = 2048                       # bytes of a request head read
WEB_CHUNK = 1024                         # reply bytes per send
WEB_REFRESH = 15                         # s between status.json polls of
                                         # the page (WEB_PAGE)
WEB_PAGE = "/web.html"                   # the status page, served as is
WEB_PAGE_GZ = "/web.html.gz"             # its gzip (deploy.sh), sent instead
                                         # to a client that accepts gzip
WEB_DIAG = 15                            # diag.log lines on the page
WEB_AWAY = False                         # .env WEB_AWAY=1: keep the radio on
                                         # for the page on the other networks too
SV_POLL = 60000                          # ms between satellite-list polls (NAV-
SV_FRESH = 5000                          # SVINFO or NAV-SAT); a page view asks
SV_WAIT = 700                            # again if older than SV_FRESH and
                                         # waits SV_WAIT ms for the answer
SV_MISS = 2                              # unanswered polls: no more waiting
GPS_LOOP_WINDOW = 10000                  # ms: start-up banners and no NMEA
GPS_LOOP_BANNERS = 3                     # in this window = a reset loop
GPS_LOOP_SAY = 300000                    # ms between its diag lines (the
                                         # windows between are counted)
GPS_FLAP_RESTARTS = 5                    # full restarts (NMEA in between)
GPS_FLAP_WINDOW = 600000                 # within this many ms = a reset loop
GPS_STORM_WAIT = 30000                   # ms between setups while it loops:
                                         # each setup blocks the loop seconds
GPS_STORM_QUIET = 10000                  # ms without a restart (in a storm
                                         # twice its last gap, up to
                                         # GPS_STORM_WAIT): the storm is
                                         # over, set it up within GPS_RETRY

# Board pins. ESP32 (WROOM): UART2 on GPIO16/17. The ESP32-C6 has no UART2 and
# GPIO16/17 are its console UART, so it uses UART1 on GPIO4 (TX) / GPIO5 (RX).
# main.py keeps the matching boot-bypass pin.
IS_C6 = "ESP32C6" in getattr(sys.implementation, "_machine", "")
if IS_C6:
    GPS_UART, GPS_TX, GPS_RX, I2C_SCL, I2C_SDA = 1, 4, 5, 7, 6
else:
    GPS_UART, GPS_TX, GPS_RX, I2C_SCL, I2C_SDA = 2, 17, 16, 22, 21

# OLED is optional: boards without a display (or a failed I²C probe) run
# headless. draw() and the boot banners become no-ops. Runtime I²C errors
# disable the display instead of resetting the chip.
oled = None
oled_err = 0
OLED_MAX_ERR = 3
try:
    import ssd1306
    _i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
    oled = ssd1306.SSD1306_I2C(128, 64, _i2c)
except Exception as e:
    print("oled", repr(e))

uart = UART(GPS_UART, baudrate=BAUD, tx=GPS_TX, rx=GPS_RX, rxbuf=4096)   # rides out HTTP calls

REFRESH = 500                            # ms between screen redraws
NO_DATA = 3000                           # ms of silence = "No data!"

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
LOG_DIR = "/gpx"
MIN_FREE = 32768                         # bytes of flash kept free
MAX_BATCH = 240                          # RAM cap while flash writes fail
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
PB_MODE = "scan"                         # what a pulse is (.env KEEPALIVE_MODE):
                                         # "scan": radio on, one scan (~2 s at
                                         # ~100 mA: +40 mA average at 5 s);
                                         # "radio": radio on and off again, no
                                         # scan (~0.2 s burst, ~5 mA average;
                                         # too little for the 20 Ah bank, which
                                         # averages: +13 mA lasted 3 h only);
                                         # "gpio": PB_PIN high PB_PULSE_MS
                                         # (a load resistor via a MOSFET)
PB_PIN = 2 if IS_C6 else 26
PB_PULSE_MS = 100                        # ms the gpio pulse lasts
PB_CONNECTED = False                     # .env KEEPALIVE_CONNECTED=1: pulses
                                         # also while connected and idle (a
                                         # bank on the status page at home;
                                         # scan or gpio, not radio)
SEEN_WAIT = 60000                        # ms: a keep-alive scan that sees home
                                         # wakes the radio at most this often
                                         # (doubling while rounds fail)
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
                                         # UART read meanwhile (a 4 KB buffer
                                         # holds ~5 s of NMEA)
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
    st.update(start=time.ticks_ms(), rx=None, spin=0, ok=0, err=0,
              status=None, quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, course=None, used=None, view={},
              hdop=None, pdop=None, ttff=None, pos_at=None, hdop_at=None,
              seq=0, epochs=0, extra=0, setup_at=None,
              setup_wait=GPS_RETRY, setup_ok=None, std=None, std_at=None,
              gps_boots=0, q=[], nav5_ok=None, banners=0, loops=0,
              loop_at=time.ticks_ms(), loop_b=0, loop_ok=0, boot_t=[],
              storm_at=None, boot_at=None, boot_last=None, boot_gap=0,
              svs=None, svs_at=None, sv_poll_at=None, sv_msg=None,
              gps_ver=None, sv_miss=0, gsa_at=None, gsa_n=0,
              ack_id=None, ack=None, ack_n=0, checked=True, hz1=0, hz5=0,
              set_ok=0, gst_at=None, loop_more=0, loop_said=None,
              nt_said=None, nt_more=0, ver_said=None,
              # NAV-PVT (M8/M10): mode of the last setup ("pvt"/"nmea"),
              # the receiver's kind from MON-VER (kept over a restart),
              # PVT given up for this boot, the last frame, the source of
              # the last second, the second's integer position (1e-7 deg)
              # and accuracy, the latest epoch's speed/heading/accuracies
              mode=None, gps_kind=None, pvt_bad=False, pvt_at=None,
              set_boots=0, ubx_ok=0, ubx_bad=0, fix_src=None,
              lat_i=None, lon_i=None, hacc=None, mps=None, vacc=None,
              sacc=None, gspeed=None, head=None, tacc=None, t_ok=False,
              t_at=None)
    _ep_new(None)

def _ep_new(t):
    # g / r: GGA / RMC positions as [ref lat, ref lon, sum dlat, sum dlon, n]
    # (offsets from the second's first fix: single-precision floats cannot
    # add up latitudes to the metre); alt / hdop: [sum, n]; p: NAV-PVT fixes
    # (_pvt_acc), pn: NAV-PVT frames, t_ok: their time valid and accurate
    ep.clear()
    ep.update(t=t, date=None, g=None, r=None, alt=[0.0, 0], hdop=[0.0, 0],
              std=None, spd=None, p=None, pn=0, t_ok=False)

def _num(s, conv=float):
    return conv(s) if s else None

def _deg(v, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed decimal degrees."""
    if not v or not hemi:
        return None
    i = v.find(".")
    d = (i if i >= 0 else len(v)) - 2
    deg = int(v[:d]) + float(v[d:]) / 60
    return -deg if hemi in "SW" else deg

def _position(f, i):
    """The sentence's (lat, lon), or None when its fields are empty."""
    lat, lon = _deg(f[i], f[i + 1]), _deg(f[i + 2], f[i + 3])
    if lat is None or lon is None:
        return None
    return lat, lon

def _acc(key, pos):
    a = ep[key]
    if a is None:
        ep[key] = [pos[0], pos[1], 0.0, 0.0, 1]
    else:
        a[2] += pos[0] - a[0]
        a[3] += pos[1] - a[1]
        a[4] += 1

def _second(s):
    """Start the sentence's UTC second (HHMMSS.ss, the fraction ignored);
    a new second publishes the one before."""
    if len(s) < 6:
        return
    _second_t("%s:%s:%s" % (s[0:2], s[2:4], s[4:6]))

def _second_t(t):
    """Start UTC second t ("HH:MM:SS", from NMEA or NAV-PVT)."""
    if t != ep["t"]:
        if ep["t"] is not None:
            _publish()
        _ep_new(t)

def _publish():
    """Make the finished second the current fix: the mean of its valid GGA
    positions (RMC ones when it had no GGA), or its first fix when it moved
    faster than AVG_MAX_SPEED. A second without one is no fix. Only RMC
    carries the date: a second that runs back past midnight without its own
    RMC has none, or its fix would be dated a day back. A second with
    NAV-PVT fixes is published from them (_publish_pvt) instead."""
    t, date = ep["t"], ep["date"]
    if date is None and st["time"] is not None and t < st["time"]:
        st["date"] = None
    elif date is not None:
        st["date"] = date
    st["time"] = t
    now = time.ticks_ms()
    pn = ep["pn"]
    if pn:
        # NAV-PVT comes every epoch, fix or not: the rate shows without a
        # fix too (_setup_check), and so does a valid time (_rtc_sync)
        st["fix_src"], st["t_ok"] = "pvt", ep["t_ok"]
        if ep["t_ok"]:
            st["t_at"] = now
        if not st["checked"]:
            if pn > 1:
                st["hz5"] += 1
            else:
                st["hz1"] += 1
    p = ep["p"]
    if p is not None:
        _publish_pvt(p, now)
        return
    a = ep["g"] or ep["r"]
    st["valid"] = a is not None
    st["epochs"] = a[4] if a else 0
    st["lat_i"] = st["lon_i"] = st["hacc"] = st["mps"] = None
    if a is None:
        return
    st["fix_src"] = "nmea"               # (also NMEA fixes beside NAV-PVT
    n = a[4]                             # ones without a fix: a restart)
    if not st["checked"] and not pn:     # _setup_check: did GPS_HZ take?
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
        st["lat"], st["lon"] = a[0] + a[2] / n, a[1] + a[3] / n
    alt, hd = ep["alt"], ep["hdop"]
    st["alt"] = alt[0] / alt[1] if alt[1] else None
    st["hdop"], st["hdop_at"] = (hd[0] / hd[1] if hd[1] else None), now
    st["std"], st["std_at"] = ep["std"], now
    _published(now)

def _published(now):
    """The second in st is a fix: stamp it and queue it for log_tick."""
    st["pos_at"] = now
    st["seq"] += 1
    if st["ttff"] is None:
        st["ttff"] = time.ticks_diff(now, st["start"]) // 1000
    # queued for log_tick: after a blocking POST, scan or setup read_uart
    # publishes several seconds at once, and each is a point
    q = st["q"]
    q.append(_snap())
    if len(q) > SECONDS_Q:
        q.pop(0)

# a published second as a tuple in this order (seq first, pos_at at 8): one
# small allocation a second rather than a dict
_SNAP = ("seq", "lat", "lon", "alt", "hdop", "hdop_at", "std", "std_at",
         "pos_at", "time", "date", "valid", "epochs", "lat_i", "lon_i",
         "hacc", "mps")

def _snap():
    return tuple(st[k] for k in _SNAP)

def _load(snap):
    for i in range(len(_SNAP)):
        st[_SNAP[i]] = snap[i]

# --- UBX-NAV-PVT ------------------------------------------------------------
# On a u-blox M8/M10 the setup (gps_setup) turns the NMEA off and asks for
# UBX-NAV-PVT every epoch: 100 bytes at GPS_HZ (500 B/s at 9600 baud, ~850
# with the NMEA), and a position in integers of 1e-7 degrees with the
# receiver's own accuracy estimates. The fixes of a second are averaged as
# integer offsets from its first fix, weighted by 1/hAcc^2, and the track
# line is written from the integers (_i7): a single-precision float steps
# ~3.8e-6 degrees (~0.4 m) at 47 degrees, so NMEA points sit on a grid.

# NAV-PVT payload (92 bytes, little endian): year, month, day, hour, min,
# sec, valid (bit 0 validDate, 1 validTime, 2 fullyResolved), tAcc ns,
# nano ns, fixType, flags (bit 0 gnssFixOK, 1 diffSoln), numSV, lon, lat
# (1e-7 deg), hMSL mm, hAcc, vAcc mm, gSpeed mm/s, headMot 1e-5 deg, sAcc
# mm/s, headAcc 1e-5 deg, pDOP 0.01, flags3 (bit 0 invalidLlh). iTOW,
# flags2, height and velN/E/D are skipped (x). The unsigned accuracies are
# read signed: "unknown" (~2**32) is then -1, not a big int.
_PVT = "<4xH6B2i2BxB2i4x3i12x4i2H"
PVT_LEN = 100                            # whole frame: 6 + 92 + 2

def _i7(v):
    """An integer coordinate in 1e-7 degrees as the track line writes it,
    "%.6f" style but without a float: rounded half away from zero (-5 ->
    "-0.000001", -4 -> "0.000000")."""
    q = (v + 5) // 10 if v >= 0 else -((5 - v) // 10)
    if q < 0:
        q = -q
        return "-%d.%06d" % (q // 1000000, q % 1000000)
    return "%d.%06d" % (q // 1000000, q % 1000000)

def _round(x):
    """Nearest int, halves away from zero."""
    return int(x + 0.5) if x >= 0 else -int(0.5 - x)

def _pvt_time(y, mo, d, h, mi, s, nano):
    """(year, month, day, "HH:MM:SS") of an epoch: its second as NMEA's
    hhmmss.ss rounded to 10 ms would say. Epochs are aligned to whole GPS (and so
    UTC) seconds; a sec field one off with nano near +-1e9 is rare."""
    k = 1 if nano >= 995000000 else (-1 if nano < -5000000 else 0)
    if k:
        r = h * 3600 + mi * 60 + s + k
        z = _days(y, mo, d)
        if r < 0:
            r, z = r + 86400, z - 1
        elif r >= 86400:
            r, z = r - 86400, z + 1
        y, mo, d = _civil(z)
        h, mi, s = r // 3600, r // 60 % 60, r % 60
    return y, mo, d, "%02d:%02d:%02d" % (h, mi, s)

def _pvt(fr):
    """One NAV-PVT frame (a memoryview, whole, checksum good): starts its
    second, keeps the latest epoch's fix type, satellites, speed and
    accuracies in st, and adds a fix (gnssFixOK, a valid position, fixType
    2D, 3D or GNSS+DR, as GGA's quality > 0 is today: 2D drops the
    elevation) to the second's mean (_pvt_acc)."""
    (y, mo, d, h, mi, sec, valid, tacc, nano, fix, flags, nsv, lon, lat,
     hmsl, hacc, vacc, gspd, head, sacc, headacc, pdop,
     flags3) = struct.unpack_from(_PVT, fr, 6)
    st["pvt_at"] = time.ticks_ms()
    y, mo, d, t = _pvt_time(y, mo, d, h, mi, sec, nano)
    _second_t(t)
    ep["pn"] += 1
    if valid & 1 and ep["date"] is None:
        ep["date"] = "%04d-%02d-%02d" % (y, mo, d)
    ep["t_ok"] = valid & 7 == 7 and 0 <= tacc < PVT_MAX_TACC
    ok = flags & 1 and not flags3 & 1 and 2 <= fix <= 4
    kmh = gspd * 0.0036
    st["fixtype"], st["used"], st["pdop"] = fix, nsv, pdop / 100
    st["quality"] = (2 if flags & 2 else 1) if ok else 0
    st["speed"], st["gspeed"], st["sacc"] = kmh, gspd / 1000, sacc / 1000
    st["course"] = st["head"] = head / 100000
    st["vacc"] = vacc / 1000 if vacc >= 0 else None
    st["tacc"] = tacc
    if not ok:
        return
    if ep["spd"] is None or kmh > ep["spd"]:
        ep["spd"] = kmh                  # the second's top speed
    _pvt_acc(lat, lon, hacc, hmsl, gspd)

def _pvt_acc(lat, lon, hacc, hmsl, gspd):
    """Add a NAV-PVT fix to the second: ep["p"] = [first lat, first lon,
    sum w, sum w*dlat, sum w*dlon, n, sum hAcc mm, within PVT_MAX_HACC,
    first hAcc, first gSpeed, sum gSpeed, sum hMSL mm], w = 1/hAcc^2 (in
    m; an hAcc of 0 counts as 1 mm). Offsets from the first fix are small
    ints, the sums of floats stay exact enough in single precision. Fixes
    over PVT_MAX_HACC count only in a second that has no better one."""
    if hacc < 1 or hacc > 100000000:
        hacc = 100000000 if hacc < 0 or hacc > 100000000 else 1
    good = hacc <= PVT_MAX_HACC * 1000
    p = ep["p"]
    if p is not None and good and not p[7]:
        p = None                         # the first good one: start over
    elif p is not None and not good and p[7]:
        return                           # a good one came: skip this
    if p is None:
        p = ep["p"] = [lat, lon, 0.0, 0.0, 0.0, 0, 0, good, hacc, gspd, 0, 0]
    m = hacc / 1000
    w = 1 / (m * m)
    p[2] += w
    p[3] += w * (lat - p[0])
    p[4] += w * (lon - p[1])
    p[5] += 1
    p[6] += hacc
    p[10] += gspd
    p[11] += hmsl

def _publish_pvt(p, now):
    """The finished second from its NAV-PVT fixes (see _publish): the
    weighted mean, or the first fix above AVG_MAX_SPEED; lat_i/lon_i keep
    it in 1e-7 degrees, lat/lon (floats) serve the stay logic."""
    n = p[5]
    st["valid"], st["epochs"] = True, n
    spd = ep["spd"]
    if n > 1 and spd is not None and spd > AVG_MAX_SPEED:
        li, lo, h, v = p[0], p[1], p[8], p[9]
    else:
        li = p[0] + _round(p[3] / p[2])
        lo = p[1] + _round(p[4] / p[2])
        h, v = p[6] / n, p[10] / n
    st.update(lat_i=li, lon_i=lo, lat=li / 10000000, lon=lo / 10000000,
              hacc=h / 1000, mps=v / 1000, alt=p[11] / n / 1000,
              hdop=None, hdop_at=now, std=None, std_at=now)
    _published(now)

def rmc(talker, f):
    _second(f[1])
    st["status"] = f[2]
    kn = _num(f[7])
    st["speed"] = None if kn is None else kn * 1.852
    if kn is not None and (ep["spd"] is None or st["speed"] > ep["spd"]):
        ep["spd"] = st["speed"]          # the second's top speed
    st["course"] = _num(f[8])
    d = f[9]
    if len(d) == 6:
        ep["date"] = "20%s-%s-%s" % (d[4:6], d[2:4], d[0:2])
    pos = _position(f, 3)
    if f[2] == "A" and pos:
        _acc("r", pos)

def gga(talker, f):
    _second(f[1])
    st["quality"] = _num(f[6], int)
    st["used"] = _num(f[7], int)
    pos = _position(f, 2)
    if (st["quality"] or 0) > 0 and pos:
        _acc("g", pos)
        alt, hd = _num(f[9]), _num(f[8])
        if alt is not None:
            ep["alt"][0] += alt
            ep["alt"][1] += 1
        if hd is not None:
            ep["hdop"][0] += hd
            ep["hdop"][1] += 1

def gsa(talker, f):
    """Fix type (and PDOP). One per system and round on a multi-GNSS
    receiver; more than GSA_MAX in a second means the GSA rate of the
    setup was lost (a late ACK of another frame looked like its own), and
    that load overruns 9600 baud: gps_check sets it up again."""
    st["fixtype"] = _num(f[2], int)
    if len(f) > 15:
        st["pdop"] = _num(f[15])
    now = time.ticks_ms()
    if st["gsa_at"] is None or _age(now, st["gsa_at"]) >= 1000:
        st["gsa_at"], st["gsa_n"] = now, 0
    st["gsa_n"] += 1
    if st["gsa_n"] == GSA_MAX + 1:
        _extra(talker, f)

def _extra(talker, f):
    """A sentence gps_setup turns off: the receiver did not take it."""
    at = st["setup_at"]
    if at is not None and _age(time.ticks_ms(), at) >= 1000:
        st["extra"] += 1

def gsv(talker, f):
    _extra(talker, f)
    n = _num(f[3], int)
    if n is not None:
        st["view"][talker] = n          # per constellation, summed on display

def gst(talker, f):
    """The receiver's own error estimate (1-sigma, metres) of the second's
    position: the larger of latitude and longitude. A cold start converging
    shows here while its HDOP already looks good."""
    _second(f[1])
    st["gst_at"] = time.ticks_ms()       # it comes (_setup_check)
    sl, sn = _num(f[6]), _num(f[7])
    if sl is not None and sn is not None:
        ep["std"] = sl if sl > sn else sn

def txt(talker, f):
    """A u-blox prints its banner at start-up: the receiver restarted (power
    dip, reset) and is back at its defaults until gps_check sets it up."""
    if "u-blox" in ",".join(f[4:]):
        st["gps_boots"] += 1
        st["extra"] += 1                 # back at its defaults: set it up
        # again within GPS_RETRY, not 5 min; while it keeps restarting (a
        # reset loop in the last GPS_FLAP_WINDOW), every GPS_STORM_WAIT:
        # 2026-10-30 had 541 restarts, each setup blocking the loop
        now = time.ticks_ms()
        st["boot_at"] = now
        last = st["boot_last"]           # the gap sets the storm's quiet
        st["boot_gap"] = 0 if last is None else _age(now, last)
        st["boot_last"] = now
        print("gps restarted")
        _gps_flap(now)                   # first: the restart that makes
        storm = (st["storm_at"] is not None     # it a storm waits too
                 and _age(now, st["storm_at"]) < GPS_FLAP_WINDOW)
        st["setup_wait"] = GPS_STORM_WAIT if storm else GPS_RETRY
        st["ack_n"] = 0                  # at its defaults: a fresh start
        # maybe another receiver (swapped while running): ask anew
        st.update(svs=None, svs_at=None, sv_msg=None, gps_ver=None)

def _gps_flap(now):
    """A receiver that restarts every few seconds but sends NMEA in between
    (seen on the board after a flash: 51 restarts in 5 min, each after a
    full banner) passes _gps_loop_check. GPS_FLAP_RESTARTS restarts within
    GPS_FLAP_WINDOW -> the same warning and count, then the next ones are
    counted afresh (one warning per GPS_FLAP_RESTARTS restarts at most)."""
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
            "GLL": _extra, "VTG": _extra, "TXT": txt}
_PVT_OFF = ("RMC", "GGA", "GSA", "GST")  # sentences the PVT setup also ends

def _ubx(cls, mid, payload):
    """One UBX frame: sync, class, id, length, payload, Fletcher checksum."""
    m = bytes((cls, mid, len(payload) & 255, len(payload) >> 8)) + payload
    a = b = 0
    for x in m:
        a = (a + x) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + m + bytes((a, b))

SV_SVINFO = b"\xb5\x62\x01\x30\x00\x00\x31\x94"   # NAV-SVINFO poll
SV_SAT = b"\xb5\x62\x01\x35\x00\x00\x36\xa3"      # NAV-SAT poll
MON_VER = b"\xb5\x62\x0a\x04\x00\x00\x0e\x34"     # MON-VER poll
GNSS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS",
        6: "GLONASS"}
UBLOX_HW = {"00040007": "u-blox 6", "00070000": "u-blox 7",
            "00080000": "u-blox M8", "00190000": "u-blox F9",
            "000A0000": "u-blox M10"}

def sv_poll(now):
    """Ask for the satellite list. No GSV at 9600 baud (see GPS_MSGS): one
    answer a minute, or per page view. NAV-SVINFO (u-blox 6/7/M8) or
    NAV-SAT (M8 and later; an M10 knows only that); until one answered,
    both. Also asks for MON-VER (the receiver model) until it answers."""
    at, got = st["sv_poll_at"], st["svs_at"]
    if at is not None and (got is None or time.ticks_diff(got, at) < 0):
        st["sv_miss"] += 1               # the last poll got no answer
    st["sv_poll_at"] = now
    m = st["sv_msg"]
    try:
        if m != "sat":
            uart.write(SV_SVINFO)
        if m != "svinfo":
            uart.write(SV_SAT)
        if st["gps_ver"] is None:
            uart.write(MON_VER)
    except OSError:
        pass

def _z(b):
    """A zero-terminated MON-VER string."""
    b = bytes(b)
    i = b.find(b"\0")
    return (b if i < 0 else b[:i]).decode()

def _gps_ver(p):
    """MON-VER payload -> "u-blox M10, ROM SPG 5.10, PROTVER=34.10, ..."."""
    hw = _z(p[30:40])
    out = [UBLOX_HW.get(hw, "hw " + hw), _z(p[0:30])]
    for k in range(40, len(p) - 29, 30):
        e = _z(p[k:k + 30])
        if e.startswith("PROTVER") or e.startswith("MOD="):
            out.append(e)
    return ", ".join(out)

def _gps_kind(p):
    """MON-VER payload -> how NAV-PVT is set up: "m10" (CFG-VALSET: M10,
    and any receiver of protocol 23 or later), "m8" (CFG-MSG: M8,
    protocol 15-22, whose NAV-PVT has the 92-byte payload), or None (u-blox
    6/7 and unknown: NMEA)."""
    hw = _z(p[30:40])
    if hw == "000A0000":
        return "m10"
    if hw == "00080000":
        return "m8"
    pv = 0.0
    for k in range(40, len(p) - 29, 30):
        e = _z(p[k:k + 30])
        if e.startswith("PROTVER"):
            try:
                pv = float(e[7:].strip(" ="))
            except ValueError:
                pass
    return "m10" if pv >= 23 else ("m8" if pv >= 15 else None)

def _ubx_ck(fr, a, b):
    """Fletcher checksum (ck_a, ck_b) of fr[a:b], in place (no copy)."""
    ca = cb = 0
    for k in range(a, b):
        ca = (ca + fr[k]) & 255
        cb = (cb + ca) & 255
    return ca, cb

def _ubx_frame(fr):
    """A whole UBX frame with a good checksum, read from the stream
    (read_uart; a memoryview into its buffer, not kept). NAV-SVINFO and
    NAV-SAT fill st["svs"]: (svid, cno dBHz, elevation, azimuth, used,
    unhealthy, gnssId) per satellite; MON-VER fills st["gps_ver"]; an
    ACK-ACK / ACK-NAK for the frame _ubx_ack waits for (st["ack_id"]) sets
    st["ack"] to 1 / 0. Others (late ACKs) are dropped."""
    cls, mid = fr[2], fr[3]
    if cls == 0x01 and mid == 0x07:
        if len(fr) != PVT_LEN:           # another protocol's NAV-PVT, or
            st["ubx_bad"] += 1           # cut: not read
            return
        st["ubx_ok"] += 1
        _pvt(fr)
        return
    st["ubx_ok"] += 1
    if cls == 0x05:
        if (mid < 2 and len(fr) == 10 and st["ack"] != 1
                and (fr[6] << 8 | fr[7]) == st["ack_id"]):
            st["ack"] = mid
        return
    if cls == 0x0A and mid == 0x04 and len(fr) >= 48:
        try:
            st["gps_ver"] = _gps_ver(fr[6:-2])
            st["gps_kind"] = _gps_kind(fr[6:-2])
        except (UnicodeError, ValueError):
            st["gps_ver"] = "?"
        v = st["gps_ver"]
        print("gps", v)
        if v != st["ver_said"]:          # asked again after every restart:
            st["ver_said"] = v           # a line only when it changed
            _diag_write("gps " + v)
        return
    if cls != 0x01 or len(fr) < 16:
        return
    svs = []
    if mid == 0x30:
        # svid 1-32 GPS, 120-158 SBAS; flags bit 0 used, bit 4 unhealthy
        for k in range(fr[10]):
            o = 14 + 12 * k
            if o + 12 > len(fr) - 2:
                break
            sv = fr[o + 1]
            el = fr[o + 5] - 256 if fr[o + 5] > 127 else fr[o + 5]
            svs.append((sv, fr[o + 4], el, fr[o + 6] | fr[o + 7] << 8,
                        bool(fr[o + 2] & 1), bool(fr[o + 2] & 16),
                        0 if sv <= 32 else (1 if 120 <= sv <= 158 else None)))
        st["sv_msg"] = "svinfo"
    elif mid == 0x35:
        # gnssId, svId, cno, elev, azim; flags bit 3 used, bits 4-5 health
        # (2 = unhealthy)
        for k in range(fr[11]):
            o = 14 + 12 * k
            if o + 12 > len(fr) - 2:
                break
            el = fr[o + 3] - 256 if fr[o + 3] > 127 else fr[o + 3]
            if el < -90:
                el = None                # -91: position not known yet
            svs.append((fr[o + 1], fr[o + 2], el, fr[o + 4] | fr[o + 5] << 8,
                        bool(fr[o + 8] & 8), (fr[o + 8] >> 4) & 3 == 2,
                        fr[o]))
        st["sv_msg"] = "sat"
    else:
        return
    st["svs"], st["svs_at"], st["sv_miss"] = svs, time.ticks_ms(), 0

def sv_used():
    """Satellites in the fix: from a satellite list younger than two polls
    (GGA stops at 12 on u-blox), else GGA."""
    at = st["svs_at"]
    if (st["svs"] is not None and at is not None
            and _age(time.ticks_ms(), at) < 2 * SV_POLL):
        return len([v for v in st["svs"] if v[4]])
    return st["used"]

def sv_seen():
    """Satellites with a signal: from the last satellite list, else the GSV
    count (before the setup turns GSV off)."""
    if st["svs"] is not None:
        return len([v for v in st["svs"] if v[1] > 0])
    n = 0
    for c in st["view"].values():
        n += c
    return n

# NMEA message ids (class 0xF0) and their rate per fix on this port. At
# 9600 baud GPS_HZ fixes fit only with fewer sentences: RMC and GGA every
# fix (~150 B), GST once a second, GSA every GSA_EVERY s, no GSV / GLL /
# VTG. A multi-GNSS receiver (M8, M10) sends one GSA per system: four a
# second took the M10 to ~104% of the 960 B/s and it dropped sentences
# and the satellite list (measured 2026-10-29); now ~850 B/s.
GSA_EVERY = 5                            # s between GSA (fix type) rounds
GSA_MAX = 8                              # GSA in a second beyond this: the
                                         # receiver did not take its rate
GPS_MSGS = ((0x03, 0), (0x01, 0), (0x05, 0), (0x02, GPS_HZ * GSA_EVERY),
            (0x07, GPS_HZ), (0x00, 1), (0x04, 1))

# NAV-PVT setups (st["mode"] "pvt"): NAV-PVT every epoch, every NMEA
# sentence off. M10 (and protocol 23+): one UBX-CFG-VALSET (class 0x06 id
# 0x8A; version 0, layer RAM 0x01, so a restart is back at the NMEA
# defaults) with the CFG-MSGOUT-*_UART1 keys of the u-blox M10 SPG 5.10
# interface description (UBX-21035062), each a U1 output rate per epoch.
# M8: the legacy UBX-CFG-MSG per message.
CFG_MSGOUT_PVT = 0x20910007              # CFG-MSGOUT-UBX_NAV_PVT_UART1
CFG_MSGOUT_NMEA = (0x209100BB,           # CFG-MSGOUT-NMEA_ID_GGA_UART1
                   0x209100AC,           # ..._RMC_UART1
                   0x209100C0,           # ..._GSA_UART1
                   0x209100D4,           # ..._GST_UART1
                   0x209100C5,           # ..._GSV_UART1
                   0x209100CA,           # ..._GLL_UART1
                   0x209100B1)           # ..._VTG_UART1
PVT_NMEA_OFF = (0x00, 0x04, 0x02, 0x07, 0x03, 0x01, 0x05)   # (CFG-MSG ids)

def _valset(keys):
    """UBX-CFG-VALSET to RAM of (key, U1 value) pairs."""
    p = bytearray(4 + 5 * len(keys))
    p[1] = 0x01                          # layers: RAM
    o = 4
    for k, v in keys:
        struct.pack_into("<IB", p, o, k, v)
        o += 5
    return _ubx(0x06, 0x8A, bytes(p))

def _gps_mode():
    """"pvt" when NAV-PVT is to be used: GPS_PVT, a receiver that knows it
    (st["gps_kind"], from MON-VER) and PVT not given up this boot."""
    return ("pvt" if GPS_PVT and not st["pvt_bad"]
            and st["gps_kind"] in ("m8", "m10") else "nmea")

def _setup_msgs(mode):
    """The message frames of a setup in mode (see GPS_MSGS)."""
    if mode == "nmea":
        return [_ubx(0x06, 0x01, bytes((0xF0, mid, rate)))
                for mid, rate in GPS_MSGS]
    if st["gps_kind"] == "m10":
        return [_valset([(CFG_MSGOUT_PVT, 1)]
                        + [(k, 0) for k in CFG_MSGOUT_NMEA])]
    return ([_ubx(0x06, 0x01, bytes((0x01, 0x07, 1)))]
            + [_ubx(0x06, 0x01, bytes((0xF0, mid, 0)))
               for mid in PVT_NMEA_OFF])

def _rate(ms):
    """UBX-CFG-RATE: measRate ms, navRate 1, timeRef 1 (GPS time)."""
    return _ubx(0x06, 0x08, bytes((ms & 255, ms >> 8, 1, 0, 1, 0)))

def _nav5():
    """UBX-CFG-NAV5: dynamic model, fix mode, elevation mask and static
    hold (mask bits 0x01 dyn, 0x02 minEl, 0x04 fixMode, 0x40 staticHold);
    the other fields are left as they are. Not 0x10: that is posMask, and
    it would set the PDOP/TDOP/pAcc/tAcc masks to the zeros below. 36-byte
    payload, u-blox 6 to M10."""
    p = bytearray(36)
    p[0] = 0x47                          # dyn | minEl | fixMode | staticHold
    p[2] = GPS_DYN_MODEL
    p[3] = GPS_FIX_MODE
    p[12] = GPS_MIN_ELEV & 255           # minElev, int8 degrees
    p[22] = min(max(GPS_STATIC_HOLD, 0), 255)   # staticHoldThresh cm/s
    return _ubx(0x06, 0x24, bytes(p))

def _ubx_ack(frame, t0):
    """Send a UBX-CFG frame until the receiver acknowledges it (UBX-ACK-ACK
    for its class and id), at most GPS_ACK_TRIES times, and after the first
    send only until GPS_SETUP_MAX after t0 (the setup's start): True, None
    for a UBX-ACK-NAK (it answers, but refuses the frame), False for
    silence. What is read meanwhile goes through read_uart() as always: the
    NMEA is handled (a start-up banner counts) and _ubx_frame notes the
    answer. An overrun receiver sends the ACK seconds late, so one that
    comes during a later try counts too; what was read before the first
    send (an older ACK for the same id) does not."""
    read_uart()
    st["ack_id"], st["ack"] = frame[2] << 8 | frame[3], None
    try:
        for k in range(GPS_ACK_TRIES):
            if k and _age(time.ticks_ms(), t0) >= GPS_SETUP_MAX:
                break
            uart.write(frame)
            t1 = time.ticks_ms()
            while True:
                read_uart()
                a = st["ack"]
                if a is not None:
                    return True if a else None   # None: answered, refused
                now = time.ticks_ms()
                if (_age(now, t1) >= GPS_ACK_WAIT
                        or _age(now, t0) >= GPS_SETUP_MAX):
                    break
                time.sleep_ms(5)
            feed()
        return False
    finally:
        st["ack_id"] = None

def gps_setup():
    """Ask the receiver for GPS_HZ fixes a second (and the NAV5 filter
    settings), frame by frame until each is acknowledged. First back to 1
    Hz: at GPS_HZ with every sentence on
    (what a receiver that took only part of a setup sends) its output
    overruns 9600 baud, and then it drops incoming frames (measured on the
    board). Not saved in the receiver, so it is sent at every boot. If the
    receiver does not answer at all (still booting, overrun, unwired ESP
    TX, not a u-blox), the message frames are sent once without waiting and
    the rate is left alone; gps_check sends the setup again while the
    receiver still sends the sentences it turns off, and until then each
    second's average is that one fix. The whole setup blocks the loop
    GPS_SETUP_MAX at most (late ACKs took 10-30 s): what is not
    acknowledged by then is sent once without waiting, as if silent."""
    read_uart()                          # a banner in it came before
    t0 = time.ticks_ms()
    boots = st["gps_boots"]
    st["setup_at"], st["extra"] = t0, 0
    ok = False
    mode = "nmea"                        # (until MON-VER names an M8/M10:
                                         # then gps_check sets it up again)
    try:
        # GPS_HZ only once every message frame was acknowledged: GPS_HZ
        # with a sentence still on is the overrun state. Once a frame goes
        # unanswered the other message frames are sent without waiting (a
        # receiver that stops answering blocks the loop ~4 s, not ~4 s per
        # frame) and the receiver stays at 1 Hz until gps_check retries.
        if _ubx_ack(_rate(1000), t0):
            # NAV5 is a filter tuning, not part of the 5 Hz setup: a receiver
            # that refuses it (another model, a NAK) still gets the rest
            r = _ubx_ack(_nav5(), t0)
            st["nav5_ok"] = r is True
            ok = r is not False          # silent: it stopped answering
            if r is None:
                print("gps nav5 refused")
            mode = _gps_mode()
            for fr in _setup_msgs(mode):
                if ok:
                    r = _ubx_ack(fr, t0)
                    ok = r is True
                    if r is None and mode == "pvt":
                        _pvt_off("refused")   # a NAK: NMEA next time
                else:
                    uart.write(fr)
                    time.sleep_ms(30)
            if ok:
                ok = _ubx_ack(_rate(1000 // GPS_HZ), t0) is True
        else:
            # no answer: an overrun receiver ignores single frames but takes
            # a burst (measured): the message frames at once cut its output
            # (no rate change); gps_check then sets it up acknowledged
            mode = _gps_mode()
            for fr in _setup_msgs(mode):
                uart.write(fr)
                time.sleep_ms(30)
    except Exception as e:
        print("gps setup", repr(e))
    # sentences sent while the setup ran are not a receiver that ignored
    # it; a restart meanwhile (its banner was read) is: set up again
    st.update(setup_at=time.ticks_ms(),
              extra=0 if st["gps_boots"] == boots else 1, setup_ok=ok,
              checked=not ok, hz1=0, hz5=0, set_ok=st["ok"], mode=mode,
              set_boots=st["gps_boots"])
    if ok:
        # it listens: a later revert is quick. One that acknowledges but
        # keeps sending what it turned off (gps_check counts ack_n) less
        # and less often: the first retry after GPS_RETRY (that ACK may
        # have been a late one of another frame), then doubling
        n = st["ack_n"]
        st["setup_wait"] = min(GPS_RETRY << (n - 1 if n > 1 else 0),
                               GPS_IGNORED_MAX)
        st["view"] = {}                  # GSV is off now: no frozen count
    print("gps setup" if mode == "nmea" else "gps setup pvt",
          "ok" if ok else "no ack")

def _pvt_off(why):
    """Give NAV-PVT up for this boot (a NAK, or no NAV-PVT after an
    acknowledged setup): the next setup is the NMEA one."""
    if not st["pvt_bad"]:
        st["pvt_bad"] = True
        print("gps pvt", why, "- NMEA")
        _diag_write("gps pvt %s, NMEA this boot" % why)

def _setup_check(now):
    """GPS_VERIFY after an acknowledged setup: did it take? An ACK may be a
    late one of another frame, and a receiver may acknowledge and ignore.
    Valid seconds all at one fix (GPS_HZ not taken), or NMEA coming but no
    GST (the GPS_MAX_STD filter would be off without a word): counted as
    a sentence it should not send, so gps_check sets it up again. Only
    valid seconds tell: before 3 of them (no fix yet: a setup at boot, or
    indoors, where an M10 may send no GST either) there is no verdict yet,
    it waits up to GPS_VERIFY_MAX. Taken (the backoff starts afresh) only
    with GPS_HZ fixes a second and GST seen. A NAV-PVT setup counts its
    epochs (they come without a fix too) and needs no GST; no NAV-PVT at
    all by GPS_VERIFY gives PVT up for this boot (_pvt_off) and the NMEA
    setup follows (gps_check)."""
    if st["checked"]:
        return
    age = _age(now, st["setup_at"])
    if age < GPS_VERIFY:
        return
    pvt = st["mode"] == "pvt"
    if pvt:
        if st["gps_boots"] != st["set_boots"]:
            return                       # restarted: its own setup comes
        a = st["pvt_at"]
        if a is None or time.ticks_diff(a, st["setup_at"]) < 0:
            st["checked"] = True
            _pvt_off("no NAV-PVT")
            return
    hz1, hz5 = st["hz1"], st["hz5"]
    if hz1 + hz5 < 3:
        if age >= GPS_VERIFY_MAX:
            st["checked"] = True         # no fix all this time: no verdict
            print("gps setup unverified: no fix")
        return
    st["checked"] = True
    slow = GPS_HZ > 1 and not hz5
    g = st["gst_at"]
    gst = g is not None and time.ticks_diff(g, st["setup_at"]) >= 0
    no_gst = not pvt and not gst and st["ok"] - st["set_ok"] >= 10
    if slow or no_gst:
        st["extra"] += 1
        why = "1 fix/s" if slow else "no GST"
        print("gps setup not taken:", why)
        _not_taken(now, why)
    elif (gst or pvt) and (hz5 or GPS_HZ == 1) and not st["extra"]:
        st["ack_n"] = 0                  # it took this one

def _not_taken(now, why):
    """A diag line for a setup that was not taken, one per GPS_NOT_TAKEN_SAY
    (the ones between counted): a session left at 1 Hz leaves a trace."""
    said = st["nt_said"]
    if said is not None and _age(now, said) < GPS_NOT_TAKEN_SAY:
        st["nt_more"] += 1
        return
    more = st["nt_more"]
    st["nt_said"], st["nt_more"] = now, 0
    _diag_write("gps setup not taken: " + why
                + (" (%d more)" % more if more else ""))

_STAMPS = ("rx", "pos_at", "hdop_at", "std_at", "gsa_at", "gst_at",
           "boot_at", "boot_last", "loop_said", "nt_said", "pvt_at", "t_at")

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
    t = st["setup_at"]
    if t is not None and _age(now, t) > TICKS_CAP:
        st["setup_at"] = time.ticks_add(now, -TICKS_CAP)

def gps_check(now):
    """Send the setup again while the receiver still sends the sentences it
    turns off: a receiver powered up with the board is still booting when
    run() sends it, and stays at 1 Hz. Less and less often, so a receiver
    that never takes it (or an unwired ESP TX) costs next to nothing."""
    if _age(now, st["loop_at"]) >= GPS_LOOP_WINDOW:
        _stamps_old(now)
        _gps_loop_check(now)
    _setup_check(now)
    # also when the setup went out but was not acknowledged (an ACK lost,
    # a silent receiver that took the blind burst): it may sit at 1 Hz
    at = st["setup_at"]
    b = st["boot_at"]
    quiet = GPS_STORM_QUIET
    t = st["storm_at"]
    if t is not None and _age(now, t) < GPS_FLAP_WINDOW:
        # in a storm, quiet means twice its last gap between restarts (up
        # to GPS_STORM_WAIT): restarts 12 s apart each got a setup, 54 in
        # 10 min, over a third of the loop blocked
        quiet = min(max(quiet, 2 * st["boot_gap"]), GPS_STORM_WAIT)
    if (st["setup_wait"] > GPS_RETRY and b is not None
            and (st["extra"] or st["setup_ok"] is not True)
            and _age(now, b) >= quiet):
        # quiet since the last restart: the storm (or a long backoff) is
        # over, the receiver sits at 1 Hz with every sentence on (or took
        # only the blind burst, unacknowledged): set it up now (measured:
        # over a minute at 1 Hz)
        st["setup_wait"] = GPS_RETRY
        st["boot_at"] = None
    # the satellite poll after a setup that is due, not before it: the
    # receiver is busy with the frames and the answer would be a miss
    poll = st["sv_poll_at"] is None or _age(now, st["sv_poll_at"]) >= SV_POLL
    # the receiver's model came (MON-VER), or PVT was given up: the other
    # mode's setup is due as if this one had not been taken
    other = st["mode"] is not None and st["mode"] != _gps_mode()
    if not (at is None
            or (not st["extra"] and not other and st["setup_ok"] is True)
            or _age(now, at) < st["setup_wait"]):
        if st["setup_ok"] is True and st["extra"]:
            st["ack_n"] += 1             # acknowledged, but not taken
        st["setup_wait"] = min(st["setup_wait"] * 2, GPS_RETRY_MAX)
        print("gps setup again")
        gps_setup()
        now = time.ticks_ms()
    if poll:
        sv_poll(now)

def _loop_diag(now, line):
    st["loop_more"], st["loop_said"] = 0, now
    _diag_write(line)

def _gps_loop_check(now):
    """A receiver in a reset loop (supply dips, a bad module) sends only the
    start of its banner over and over, cut off before the newline, and no
    NMEA (seen on the board: "$GPTXT,01,01,02,u" + 0xFF junk, nothing else,
    no UBX answer). Every GPS_LOOP_WINDOW: GPS_LOOP_BANNERS or more banner
    starts and not one valid sentence -> a console warning, counted. The
    diag log gets the first warning, then (the windows in between counted
    in RAM) one "N more windows" line per GPS_LOOP_SAY and one when it
    ends: a line every window was 12.6 KB an hour."""
    if _age(now, st["loop_at"]) < GPS_LOOP_WINDOW:
        return
    b = st["banners"] - st["loop_b"]
    ok = st["ok"] - st["loop_ok"]
    more = st["loop_more"]
    if b >= GPS_LOOP_BANNERS and ok == 0:
        st["loops"] += 1
        st["storm_at"] = now
        print("gps reset loop?", b, "banners, no NMEA in",
              GPS_LOOP_WINDOW // 1000, "s: check the GPS power")
        said = st["loop_said"]
        if said is not None and _age(now, said) < GPS_LOOP_SAY:
            st["loop_more"] = more + 1
        elif more:
            _loop_diag(now, "gps reset loop %d more windows" % (more + 1))
        else:
            _loop_diag(now, "gps reset loop %d banners, no NMEA" % b)
    elif more:
        _loop_diag(now, "gps reset loop %d more windows" % more)
    st.update(loop_at=now, loop_b=st["banners"], loop_ok=st["ok"])

def _temp():
    try:
        return esp32.mcu_temperature()
    except Exception:
        return None

_freq_bad = []                           # clocks this board refused
_freq_bad_at = [None]                    # ticks of the last refusal

def _set_freq(mhz):
    """CPU clock: 80 MHz saves ~15-25 mA on the ESP32 against 240 (the UART
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
        if at is not None and _age(time.ticks_ms(), at) < FREQ_RETRY:
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
    allowed = (80, 160) if IS_C6 else (20, 40, 80, 160, 240)
    if n == 0 or n in allowed:
        return n
    print(key, "not 0 or", "/".join(str(m) for m in allowed), "on this",
          "board, using", default)
    return default

def _status():
    """One console line: chip temperature (the die, not the GPS module),
    fixes in the last second, satellites, HDOP, points and uploads."""
    print("status temp", _temp(), "hz", st["epochs"], "used", st["used"],
          "hdop", st["hdop"], "points", log["points"], "sent", up["sent"],
          "wifi", wifi["state"], "pulses", wifi["pb_scans"],
          "mem", py_heap()[0], "gpsloops", st["loops"])

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
    so both together stay under 2 * DIAG_MAX. Never raises."""
    if not DIAG:
        return
    try:
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
    t = _fix_secs()
    return "-" if t is None else _stamp_at(t)[1]

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

_up = [time.ticks_ms(), time.ticks_ms()]  # ms since boot, ticks then

def uptime_s(now=None):
    """Seconds since boot. ticks_ms wraps every 2^30 ms (12.4 days), so
    the time is summed up from tick differences (called every loop)."""
    if now is None:
        now = time.ticks_ms()
    _up[0] += time.ticks_diff(now, _up[1])
    _up[1] = now
    return _up[0] // 1000

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
            print("heap grew in %s to %d KB" % (phase, k))
            _diag_write("heap grew in %s to %d KB" % (phase, k))
    except Exception:                    # MemoryError: a note never fails
        pass                             # the upload

def _diag(now):
    """One line every DIAG ms: uptime, GPS time, NMEA sentences good/bad,
    receiver restarts and reset-loop warnings, satellites seen/used, fix,
    HDOP, points logged/sent, WiFi state, keep-alive pulses, free Python
    heap, IDF heap free/largest block (KB), Python heap size (KB: it
    doubles into the IDF heap when an allocation does not fit). On the
    NAV-PVT path also the UBX frames good/bad and hAcc (m): there NMEA
    stops after the setup and HDOP is unknown."""
    view = sv_seen()
    fr, big = idf_heap()
    pf, pk = py_heap()
    pvt = ""
    if st["mode"] == "pvt":
        ha = st["hacc"] if st["lat_i"] is not None else None
        pvt = " ubx %d/%d hacc %s" % (st["ubx_ok"], st["ubx_bad"],
                                     "-" if ha is None else "%.1f" % ha)
    _diag_write(
        "up %d utc %s nmea %d/%d gpsboots %d loops %d sats %d/%s fix %s "
        "hdop %s points %d sent %d wifi %s pulses %d mem %d idf %s/%s "
        "heap %d%s" % (
            uptime_s(now), _diag_utc(), st["ok"], st["err"], st["gps_boots"],
            st["loops"], view, sv_used(), status().replace(" ", "_"),
            st["hdop"], log["points"], up["sent"],
            wifi["state"].replace(" ", "_"), wifi["pb_scans"],
            pf, fr, big, pk, pvt))

def checksum_ok(line):
    star = line.rfind(b"*")
    if not line.startswith(b"$") or star < 0:
        return False
    c = 0
    for b in memoryview(line)[1:star]:   # no copy of the line
        c ^= b
    try:
        return c == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False

def handle(line):
    line = line.strip()
    i = line.rfind(b"$")
    if i > 0:
        line = line[i:]                  # a binary UBX reply or a cut-off
                                         # sentence ran into it
    if not line:
        return
    if not checksum_ok(line):
        st["err"] += 1
        return
    try:
        f = line[1:line.rfind(b"*")].decode().split(",")
        fn = HANDLERS.get(f[0][2:])
        if fn:
            fn(f[0][:2], f)
            if st.get("mode") == "pvt" and f[0][2:] in _PVT_OFF:
                _extra(f[0][:2], f)      # the PVT setup turned it off
        st["ok"] += 1
    except (ValueError, IndexError, UnicodeError):
        st["err"] += 1

# What the UART delivers goes into one preallocated buffer, read in place
# and compacted: growing a bytes object by each read and slicing it at each
# line made a new copy of everything left (up to ~1.3 KB) several times a
# second, and one big allocation grows the heap (see Upload) and starves
# TLS. Line ends, UBX sync bytes and banners are searched with
# bytearray.find (start/end in place, no copy: MicroPython 1.29 has it),
# not byte by byte in Python.
RX_READ = 256                            # bytes per readinto: small pieces
RX_LINE = 256                            # no newline after this many bytes
RX_FRAME = 1100                          # (or a UBX frame this long): junk
_rx = bytearray(RX_FRAME + RX_READ)
_rxv = memoryview(_rx)
_rx_n = 0                                # bytes in _rx
_rx_s = 0                                # scanned up to here: no line end,
                                         # no UBX sync before it
_BANNER = b"TXT,01,01,02,u"              # a u-blox start-up banner (start)
RX_KEEP = len(_BANNER) - 1               # its start kept when junk is dropped

def _rx_drop(r):
    """Drop the first r bytes of _rx, the rest moved to the front (one
    slice store: MicroPython copies it with memmove, overlap is fine)."""
    global _rx_n
    n = _rx_n - r
    _rxv[0:n] = _rxv[r:r + n]
    _rx_n = n

def read_uart():
    global _rx_n, _rx_s
    b = _rx
    while uart.any():
        n = _rx_n
        k = uart.readinto(_rxv[n:n + RX_READ])
        if not k:
            return
        st["rx"] = time.ticks_ms()
        st["spin"] += 1
        # a start-up banner (or its start), also when split across reads:
        # every one that ends in what was just read
        e = n + k
        j = b.find(_BANNER, n - RX_KEEP if n > RX_KEEP else 0, e)
        while j >= 0:
            st["banners"] += 1
            j = b.find(_BANNER, j + RX_KEEP + 1, e)
        n = e
        _rx_n = n
        r = 0                            # bytes before r are done with
        s = _rx_s
        while True:
            # the first line end or UBX sync (0xB5 0x62, never in NMEA)
            e = b.find(b"\n", s, n)
            j = b.find(b"\xb5\x62", s, n if e < 0 else e)
            if j < 0:
                if e < 0:
                    # a 0xB5 last may be a sync byte: looked at again
                    s = n - 1 if n > s and b[n - 1] == 0xB5 else n
                    break
                j = e
            if j == e:
                e = j - 1 if j > r and b[j - 1] == 13 else j
                handle(bytes(_rxv[r:e]))
                r = s = j + 1
                continue
            # a binary UBX answer: whole, by its length (it may hold 0x0A
            # bytes); what came before it is a cut-off line
            r = s = j
            if n < j + 6:
                break
            need = j + 8 + (b[j + 4] | b[j + 5] << 8)
            if need - j > 1024:          # not a frame: skip the sync bytes
                r = s = j + 2
                continue
            if n < need:
                break
            ca, cb = _ubx_ck(b, j + 2, need - 2)
            if ca == b[need - 2] and cb == b[need - 1]:
                _ubx_frame(_rxv[j:need])
                r = s = need
                continue
            # a broken frame, or sync bytes in junk with a plausible length:
            # only they are skipped, the NMEA behind is read (from its $)
            st["err"] += 1
            st["ubx_bad"] += 1
            s = j + 2
            while s < n and b[s] != 0x24 and b[s] != 0xB5:
                s += 1
            r = s
        if r:
            _rx_drop(r)
        _rx_s = s - r
        n = _rx_n
        frame = n > 1 and b[0] == 0xB5 and b[1] == 0x62
        if n > (RX_FRAME if frame else RX_LINE):
            # no newline in sight: garbage or a wrong baud rate. A banner
            # start at its end stays (a reset loop sends "$GPTXT,01,01,02,u"
            # and junk, no newline): counted when its "u" comes next read
            k = RX_KEEP
            while k and b.find(_BANNER[:k], n - k, n) < 0:
                k -= 1
            _rxv[0:k] = _rxv[n - k:n]
            _rx_n = _rx_s = k

# --- Watchdog ---------------------------------------------------------------
# Created only from run()/_loop(), so the GPIO25 boot-bypass REPL is not
# reset. ESP32 cannot disarm it: Ctrl-C starts a Timer that keeps feeding
# until KEEPALIVE_MAX, then the dog is allowed to reset.

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
        m = machine.RTC().memory()
        if m:
            if isinstance(m, bytes):
                m = m.split(b"\x00", 1)[0].split(b"\n", 1)[0].decode()
            s = str(m).strip()
            for c in s:
                if c < "0" or c > "9":
                    break
                n = n * 10 + (ord(c) - 48)
    except Exception:
        try:
            with open("/boot_fail") as f:
                n = int(f.read().strip() or "0")
        except Exception:
            n = 0
    n += 1
    raw = ("%d" % n).encode()
    try:
        machine.RTC().memory(raw)
    except Exception:
        try:
            f = open("/boot_fail", "w")
            f.write("%d" % n)
            f.close()
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
    if v is None:
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

def _load_nets(env):
    nets = []
    rows = (
        ("SSID", "WPA2", "HOME1", "SSID_HOME", "SSID_HIDDEN", True,
         ("IP", "MASK", "GW", "DNS")),
        ("SSID2", "WPA2_2", "HOME2", "SSID2_HOME", "SSID2_HIDDEN", False,
         ("IP2", "MASK2", "GW2", "DNS2")),
        ("SSID3", "WPA2_3", "HOME3", "SSID3_HOME", "SSID3_HIDDEN", False,
         ("IP3", "MASK3", "GW3", "DNS3")),
    )
    for sk, pk, hk, hk2, hidk, default_home, ipkeys in rows:
        s = env.get(sk)
        if not s:
            continue
        p = env.get(pk)
        home = default_home
        t = _truthy(env.get(hk2))
        if t is None:
            t = _truthy(env.get(hk))
        if t is not None:
            home = t
        hidden = bool(_truthy(env.get(hidk)))
        n = {"ssid": s, "pw": "" if p is None else p,
             "home": home, "hidden": hidden, "bssid": None,
             "ip": _static_ip(env, ipkeys)}
        if isinstance(s, bytes):
            # not UTF-8 (an ANSI .env): the radio gets the bytes as they
            # are, the screen and the page one char per byte
            n["raw"] = s
            n["ssid"] = "".join(chr(c) for c in s)
        nets.append(n)
    return nets

def _ssid_b(n):
    """A configured network's SSID as the radio sees it (bytes)."""
    r = n.get("raw")
    return r if r is not None else n["ssid"].encode()

wifi = {}

def wifi_reset():
    wifi.clear()
    wifi.update(wlan=None, ssid=None, pw=None, nets=[], failed={},
                state="off", ip=None, rssi=None, tries=0, home=False,
                backoff=WIFI_RETRY, last_check=time.ticks_ms(),
                cand=[], cand_i=0, try_at=None, saw_connecting=False,
                roam_at=time.ticks_ms(), prev_status=None,
                roam_backoff=ROAM_CHECK, moved_at=None, roaming=False,
                retried=False, idle_at=time.ticks_ms(), sleep_home=False,
                pb_at=time.ticks_ms(), pb_scans=0,
                wake=False, wake_at=None,
                blank_rssi=None, blank_bssid=None, blank_bad=[],
                blank_ours=False, home_aps=[], home_miss={}, learn=False,
                hid_wait=ROAM_CHECK, hid_at=None,
                rounds=0, down_at=None, hh=None,
                seen_at=None, seen_wait=SEEN_WAIT, pb_conn_at=time.ticks_ms())

def _set_hostname(name):
    if not name:
        return
    try:
        network.hostname(name)
        return
    except (AttributeError, OSError, TypeError, ValueError,
            RuntimeError):
        pass
    w = wifi.get("wlan")
    if w is None:
        return
    try:
        w.config(dhcp_hostname=name)
    except (AttributeError, OSError, TypeError, ValueError,
            RuntimeError):
        pass

def _recently_failed(ssid, now):
    t = wifi["failed"].get(ssid)
    return t is not None and _age(now, t) < FAIL_HOLD

def _mark_failed(ssid, why=""):
    wifi["failed"][ssid] = time.ticks_ms()
    print("wifi fail", ssid, why)
    if wifi.get("roaming") and _net_home(ssid):
        if not _net_hidden(ssid):
            b = wifi.get("roam_backoff") or ROAM_CHECK
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
            b = wifi.get("hid_wait") or ROAM_CHECK
            wifi["hid_wait"] = min(max(b * 2, ROAM_CHECK), ROAM_BACKOFF_MAX)
            wifi["hid_at"] = now
            b = wifi.get("blank_bssid")
            if (b is not None and b not in (wifi.get("home_aps") or ())
                    and not _blank_bad(b, now)):
                bad = [x for x in wifi["blank_bad"]
                       if _age(now, x[1]) < BLANK_BAD_HOLD]
                bad.append((b, now))
                wifi["blank_bad"] = bad[-4:]
        wifi["roaming"] = False

def _blank_bad(bssid, now):
    """A hidden-home try on this empty-SSID AP failed within BLANK_BAD_HOLD."""
    for b, t in wifi.get("blank_bad") or ():
        if b == bssid and _age(now, t) < BLANK_BAD_HOLD:
            return True
    return False

def _scan_known(w):
    """Visible known nets, strongest BSSID per SSID, or None if scan fails.
    Hidden APs are listed with an empty SSID; the strongest blank RSSI is
    a hint that *some* hidden AP is nearby (not necessarily ours). One of
    the hidden home's learned BSSIDs (_learn_home) at >= ROAM_MIN_RSSI is
    taken first (blank_ours); else the strongest blank AP stays weak
    evidence (a mesh node not learned yet): enough for the candidate order
    and a roam, not for "home in range" or the keep-alive wake. A blank AP
    whose hidden-home try failed lately (blank_bad) is skipped."""
    by = {}
    for n in wifi["nets"]:
        by[_ssid_b(n)] = n
    mine = wifi.get("home_aps") or ()
    try:
        aps = w.scan()
    except WLAN_ERR:
        wifi["blank_rssi"] = None
        wifi["blank_bssid"] = None
        wifi["blank_ours"] = False
        return None
    _stay_scan(aps)
    _home_seen(aps)
    now = time.ticks_ms()
    best = {}
    blank_rssi, blank_bssid = None, None
    own_rssi, own_bssid = None, None
    for rec in aps:
        # SSIDs compared as bytes: a neighbour's Latin-1 or cut-off UTF-8
        # SSID raised UnicodeError on decode() and crashed the logger
        raw = rec[0]
        ssid = raw if isinstance(raw, bytes) else raw.encode()
        rssi = rec[3]
        bssid = rec[1] if len(rec) > 1 else None
        if not ssid:
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
    (no home network configured, a connect in progress, scan failed)."""
    w = wifi.get("wlan")
    if w is None or not _has_home():
        return None
    state = wifi["state"]
    if state == "connected" and wifi.get("home"):
        try:
            r = w.status("rssi")
        except (OSError, ValueError, RuntimeError):
            r = None
        if r is not None:
            return r >= ROAM_MIN_RSSI
        # unknown RSSI: a scan tells (it works while connected)
    if state not in ("connected", "sleep", "not found", "offline"):
        return None
    off = state != "connected"
    try:
        if off:
            w.active(True)
        found = _scan_known(w)
    except WLAN_ERR:
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
        if n.get("home") and n.get("rssi", -100) >= ROAM_MIN_RSSI:
            return True
    return bool(_blank_home_ok(True))

def _home_veto(now, sl):
    """A departure from a home stay while the home WiFi is still in range
    is a GPS jump, not a departure (measured: a 194 m multipath jump ended a
    home stay and uploaded 80 drift points). After a veto, departures wait
    STILL_HOME_HOLD before the next check."""
    at = sl.get("veto_at")
    if at is not None and _age(now, at) < STILL_HOME_HOLD:
        return True
    if home_in_range():
        sl["veto_at"] = now
        print("stay kept: home in range")
        return True
    return False

def _has_home():
    """Any home network configured (cached per nets list: wifi_tick asks
    every loop pass)."""
    nets = wifi.get("nets") or []
    c = wifi.get("hh")
    if c is None or c[0] is not nets:
        c = wifi["hh"] = (nets, any(n.get("home") for n in nets))
    return c[1]

def _blank_home_ok(strict=False):
    """True if a hidden home is configured and a strong empty-SSID AP was
    seen. strict ("home in range"): once BSSIDs are learned, only one of
    them counts (_scan_known); else any strong blank AP does."""
    r = wifi.get("blank_rssi")
    if r is None or r < ROAM_MIN_RSSI:
        return False
    if _blank_bad(wifi.get("blank_bssid"), time.ticks_ms()):
        return False                     # a hidden-home try there failed
    if strict and wifi.get("home_aps") and not wifi.get("blank_ours"):
        return False                     # a stranger's (or unlearned) AP
    for n in wifi.get("nets") or []:
        if n.get("home") and n.get("hidden"):
            return True
    return False

def _learn_home(w):
    """Connected to the hidden home: note its BSSID, so a neighbour's
    empty-SSID AP no longer counts as home in range (_scan_known). v1.29
    has no config("bssid") getter: then a scan while connected, and a
    record carrying the hidden SSID (if the IDF names the AP it is on) or
    else the one blank AP on the link's channel within LEARN_RSSI dB of its
    RSSI is ours. Several such blank APs (or none): nothing learned.
    Only on the hidden home itself: a drop before this tick and a hotspot
    connect learned a neighbour's blank AP. A learned BSSID that is not the
    link for HOME_AP_MISS connects, or that the scan shows with another
    SSID, is dropped, so a wrong one does not stick. The list is kept in
    HOME_APS_FILE when it changes."""
    wifi["learn"] = False
    ssid = wifi.get("ssid")
    if not (wifi.get("home") and _net_hidden(ssid)):
        return
    me = None
    for n in wifi.get("nets") or []:
        if n["ssid"] == ssid:
            me = _ssid_b(n)
            break
    found = []
    other = []                           # BSSIDs of visible networks
    b = None
    try:
        b = bytes(w.config("bssid"))
    except (OSError, ValueError, TypeError, RuntimeError):
        pass
    if b:
        found.append(b)
    else:
        try:
            ch = w.config("channel")
        except (OSError, ValueError, TypeError, RuntimeError):
            ch = None
        try:
            r = w.status("rssi")
        except (OSError, ValueError, RuntimeError):
            r = None
        feed()
        read_uart()
        try:
            aps = w.scan()
        except WLAN_ERR:
            return
        feed()
        one, many = None, False
        for rec in aps:
            raw = rec[0]
            if raw:
                if not isinstance(raw, bytes):
                    raw = raw.encode()
                if raw == me:
                    found.append(bytes(rec[1]))  # named by the IDF: ours
                else:
                    other.append(bytes(rec[1]))
                continue
            if ch is not None and rec[2] != ch:
                continue
            if r is not None and abs(rec[3] - r) > LEARN_RSSI:
                continue
            many = one is not None
            one = bytes(rec[1])
        if not found and one is not None and not many:
            found.append(one)
    found = [x for x in found if len(x) == 6]
    if not found:
        return                           # cannot tell: nothing changes
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

_HEX = b"0123456789abcdef0123456789ABCDEF"

def _hex_bssid(s):
    """12 hex digits (a HOME_APS_FILE line, bytes) as 6 bytes, else None."""
    s = s.strip()
    if len(s) != 12:
        return None
    out = bytearray(6)
    for i in range(12):
        v = _HEX.find(s[i:i + 1])
        if v < 0:
            return None
        out[i >> 1] = (out[i >> 1] << 4) | (v & 15)
    return bytes(out)

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
            b = _hex_bssid(line)
            if b is not None and b not in aps:
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
        f = open(HOME_APS_FILE, "w")
        try:
            for b in wifi["home_aps"]:
                f.write("%02x%02x%02x%02x%02x%02x\n" % tuple(b))
        finally:
            f.close()
    except (OSError, TypeError, ValueError) as e:
        print("home aps", repr(e))

def _hidden_home_target(now, arrival=False):
    """Hidden home when a strong empty-SSID AP was seen.
    Always bssid=None so the IDF probes by SSID (never pin to the
    strongest blank AP — that may be a neighbour). A stop after driving
    must not treat an unverified blank AP as home."""
    if arrival:
        return None
    if not _blank_home_ok():
        return None
    at = wifi.get("hid_at")
    if at is not None and _age(now, at) < (wifi.get("hid_wait") or ROAM_CHECK):
        return None                      # a hidden-home roam failed lately
    for n in wifi.get("nets") or []:
        if (n.get("home") and n.get("hidden")
                and n["ssid"] != wifi.get("ssid")
                and not _recently_failed(n["ssid"], now)):
            target = dict(n)
            target["bssid"] = None
            return target
    return None

def _candidate_order(w):
    """Visible strongest first (deduped), then unscanned nets in config
    order: all of them when the scan failed, else only hidden ones (a scan
    never lists them by name) and, every UNLISTED_EVERY rounds (the first
    one too), the rest (a scan can miss an AP). Each try of a network that
    is not there costs CONNECT_TIMEOUT with the radio on."""
    now = time.ticks_ms()
    scanned = _scan_known(w)
    rounds = wifi.get("rounds") or 0
    wifi["rounds"] = rounds + 1
    if scanned is None:
        cands = [dict(n) for n in wifi["nets"]]
    else:
        seen = set(n["ssid"] for n in scanned)
        every = rounds % UNLISTED_EVERY == 0
        cands = scanned + [dict(n) for n in wifi["nets"]
                           if n["ssid"] not in seen
                           and (every or n.get("hidden"))]
    if _blank_home_ok() and wifi.get("blank_ours"):
        # a hidden home behind one of its learned BSSIDs gets the scan's
        # RSSI, so a failed first association is retried at once
        # (_wifi_failed); a stranger's blank AP is no reason for 2 x 12 s
        for n in cands:
            if n.get("home") and n.get("hidden") and n.get("rssi") is None:
                n["rssi"] = wifi["blank_rssi"]
    good, bad = [], []
    for n in cands:
        (bad if _recently_failed(n["ssid"], now) else good).append(n)

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
        _set_hostname(HOSTNAME)
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
    a static address the RAM flag no longer knows of (a soft reset)."""
    cfg = net.get("ip")
    try:
        if cfg:
            w.ifconfig(cfg)
            wifi["static"] = True
        elif wifi.get("static") or any(n.get("ip") for n in
                                       wifi.get("nets") or []):
            w.ifconfig("dhcp")
            wifi["static"] = False
    except (OSError, ValueError, TypeError, RuntimeError) as e:
        print("wifi ip", repr(e))

def _wifi_connect(w, net):
    wifi["ssid"] = net["ssid"]
    wifi["pw"] = net["pw"]
    wifi["home"] = bool(net.get("home"))
    wifi["state"] = "connecting"
    wifi["try_at"] = time.ticks_ms()
    wifi["saw_connecting"] = False
    wifi["learn"] = False                # only a hidden-home GOT_IP sets it
    print("wifi try", net["ssid"])
    try:
        wifi["prev_status"] = w.status()
    except WLAN_ERR:
        wifi["prev_status"] = None
    _wifi_clear(w)
    _wifi_ip(w, net)
    ssid = net.get("raw") or net["ssid"]
    try:
        bssid = net.get("bssid")
        if bssid:
            try:
                w.connect(ssid, net["pw"], bssid=bssid)
            except TypeError:
                w.connect(ssid, net["pw"])
        else:
            w.connect(ssid, net["pw"])
    except WLAN_ERR as e:
        _mark_failed(net["ssid"], "connect %r" % (e,))
        return _wifi_next(w)
    wifi["tries"] += 1
    return True

def _net_home(ssid):
    for n in wifi.get("nets") or []:
        if n["ssid"] == ssid:
            return bool(n.get("home"))
    return False

def _net_hidden(ssid):
    for n in wifi.get("nets") or []:
        if n["ssid"] == ssid:
            return bool(n.get("hidden"))
    return False

def _wifi_got_ip(w):
    wifi["state"] = "connected"
    try:
        wifi["ip"] = w.ifconfig()[0]
    except WLAN_ERR:
        wifi["ip"] = None
    try:
        wifi["rssi"] = w.status("rssi")
    except (OSError, ValueError, RuntimeError):
        wifi["rssi"] = None
    wifi["home"] = _net_home(wifi.get("ssid"))
    wifi["down_at"] = None
    wifi["learn"] = False
    if wifi["home"] and _net_hidden(wifi.get("ssid")):
        # our hidden home answered: no blank AP stays blamed for an
        # earlier failed try, hidden roams are due again, and its BSSID is
        # learned on the next wifi_tick (_learn_home)
        wifi["blank_bad"] = []
        wifi["hid_wait"] = ROAM_CHECK
        wifi["hid_at"] = None
        wifi["learn"] = True
    wifi["sleep_home"] = False
    wifi["idle_at"] = time.ticks_ms()
    wifi["failed"].pop(wifi.get("ssid"), None)
    wifi["backoff"] = WIFI_RETRY
    wifi["roam_at"] = time.ticks_ms()
    wifi["roaming"] = False
    wifi["seen_wait"] = SEEN_WAIT
    if wifi["home"]:
        wifi["roam_backoff"] = ROAM_CHECK
    print("wifi ok", wifi.get("ssid") or "")
    return True

def _wifi_failed(w, s, elapsed):
    """This attempt failed with raw status s. A candidate the scan showed at
    >= ROAM_MIN_RSSI gets one immediate retry in the same round: on the
    ESP32-C6 (MicroPython 1.29) the first association after a cold boot
    often fails after ~7.5 s with 202 (STAT_WRONG_PASSWORD) although the
    password is right, and the retry connects within ~1 s. Anything else is
    demoted for FAIL_HOLD and the round moves on."""
    ssid = wifi.get("ssid")
    why = "status %s after %d ms" % (s, elapsed)
    cands = wifi.get("cand") or []
    i = wifi.get("cand_i", 0)
    net = cands[i] if i < len(cands) else None
    rssi = net.get("rssi") if net else None
    if (net is not None and net["ssid"] == ssid and not wifi.get("retried")
            and rssi is not None and rssi >= ROAM_MIN_RSSI):
        wifi["retried"] = True
        print("wifi retry", ssid, why)
        return _wifi_connect(w, net)
    _mark_failed(ssid, why)
    return _wifi_next(w)

def _wifi_next(w):
    i = wifi.get("cand_i", 0) + 1
    wifi["cand_i"] = i
    wifi["retried"] = False
    cands = wifi.get("cand") or []
    if i < len(cands):
        return _wifi_connect(w, cands[i])
    _wifi_round_failed(w)
    return False

def _wifi_round_failed(w):
    """No network in this round. With points to upload, search on with the
    usual growing backoff (15 s to 10 min) until one connects, the radio
    off between rounds (_wifi_begin_round turns it on); with nothing to
    upload, sleep (not yet while _web_search)."""
    now = time.ticks_ms()
    if wifi.get("down_at") is None:
        wifi["down_at"] = now
    if _nothing_to_send() and not _web_search(now):
        _wifi_off(w, "sleep")
        return
    _radio_off(w)
    wifi["state"] = "not found"
    wifi["home"] = False
    wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)
    wifi["last_check"] = time.ticks_ms()

def _web_search(now):
    """The status page is on (at home, or WEB_AWAY) and no keep-alive scan
    can see the home network come back (_home_seen): a failed round keeps
    searching with the "not found" backoff for WEB_SEARCH after the link
    went down, instead of sleeping WIFI_SLEEP (1 h, page down) at once
    (a router restart, a board booting faster than the router). A hidden
    home whose BSSIDs are not known yet is invisible to those scans."""
    if not WEB or not (WEB_AWAY or _has_home()):
        return False
    if PB_KEEPALIVE and PB_MODE == "scan" and _home_seeable():
        return False
    at = wifi.get("down_at")
    return at is not None and _age(now, at) < WEB_SEARCH

def _home_seeable():
    """A keep-alive scan can see home come back (_home_seen): a visible
    home network is configured, or the hidden home's BSSIDs are known."""
    if wifi.get("home_aps"):
        return True
    for n in wifi.get("nets") or []:
        if n.get("home") and not n.get("hidden"):
            return True
    return False

def _radio_off(w):
    try:
        w.disconnect()
    except WLAN_ERR:
        pass
    try:
        w.active(False)
    except WLAN_ERR:
        pass

def _wifi_off(w, state):
    """Radio off: everything is uploaded. It searches again once WIFI_SLEEP
    has passed and there is something to upload, or when a new stay wakes
    it. A home stay stays a home stay while the radio sleeps (sleep_home),
    so it still writes no anchor points; leaving clears it (_still_leave)."""
    if state == "sleep":
        wifi["sleep_home"] = bool(at_home() and log["still"]["active"])
    _radio_off(w)
    wifi.update(state=state, home=False, ip=None, rssi=None, wake=False,
                wake_at=None, backoff=WIFI_RETRY, last_check=time.ticks_ms())
    print("wifi", state, "home" if wifi["sleep_home"] else "")

def _upload_wait(now):
    """ms until upload_tick will next send something, 0 when it is due (or
    nothing waits): the time an upload error (server down, 401, 5xx), a
    flash or memory error backed off, or, with every waiting track held
    back (bad payloads), until the first hold ends. Before, "idle" with
    all tracks held kept the radio on for good."""
    state = str(up.get("state") or "")
    d = time.ticks_diff(up["next_at"], now)
    wait = d if 0 < d <= UP_WAIT_MAX else 0
    if state.startswith("err") or state in ("fs err", "low mem", "low idf"):
        return wait
    # "bad full": the rejected track is held (BACKOFF_MAX), its 15 s
    # next_at is no reason to keep the radio on: the holds below decide
    held = up.get("hold") or {}
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
    _wifi_off(w, "sleep")
    wifi["wake_at"] = time.ticks_add(now, wait)
    print("wifi off until upload retry", wait // 1000, "s")
    return True

def _wifi_begin_round(w):
    try:
        if not w.active():
            w.active(True)               # after _wifi_off
            _set_hostname(HOSTNAME)
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
    STILL_FILE while connected and wakes the hourly round (_all_uploaded),
    so the radio and a phone hotspot do not idle on for the whole stay."""
    if _nothing_to_send():
        return True
    sl = log["still"]
    return bool(sl["active"] and not sl["home"] and not pending())

def _nothing_to_send():
    """All uploaded, or no GEO_URL/API_KEY ("no .env": never sends, so the
    waiting tracks are no reason for rounds every backoff or hour)."""
    return _all_uploaded() or up.get("state") == "no .env"

def _all_uploaded():
    """Nothing left for the network: no finished track waiting, nothing in
    RAM, and the open track (if any) holds no points yet. An open track
    with points is closed by log_tick on WiFi, then uploaded."""
    if log["batch"] or (log["file"] is not None and log["points"]):
        return False
    return not pending()

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
    started = wifi.get("try_at")
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
        wifi["saw_connecting"] = wifi["saw_connecting"] or (s == network.STAT_CONNECTING)
        if elapsed >= CONNECT_TIMEOUT:
            return _wifi_failed(w, s, elapsed)
        return False
    # v1.29 connect() may leave the previous SSID's fail reason. Do not
    # attribute it to this attempt until CONNECTING was seen, or the
    # whole CONNECT_TIMEOUT elapses with no GOT_IP.
    leftover = (s == wifi.get("prev_status") or not wifi["saw_connecting"])
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
                and n["ssid"] != wifi.get("ssid")
                and not _recently_failed(n["ssid"], now)):
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
    cur = None
    for x in wifi.get("nets") or []:
        if x["ssid"] == wifi.get("ssid"):
            cur = dict(x)
            break
    wifi["cand"] = [target] + ([cur] if cur else [])
    wifi["cand_i"] = 0
    wifi["retried"] = False
    wifi["roaming"] = True
    print("wifi roam", target["ssid"])
    _wifi_connect(w, target)

def _gps_model_env(env):
    """GPS_MODEL=<u-blox dynModel> in .env (0 portable, 3 pedestrian, 4
    automotive, ...). Takes effect at the next GPS setup (boot)."""
    global GPS_DYN_MODEL
    v = _plain(env.get("GPS_MODEL"))
    if not v:
        return
    try:
        n = int(v)
    except ValueError:
        n = -1
    if n in GPS_MODELS:
        GPS_DYN_MODEL = n
    else:
        print("GPS_MODEL not one of", GPS_MODELS, "using", GPS_DYN_MODEL)

def _gps_pvt_env(env):
    """GPS_PVT in .env: auto (default) uses NAV-PVT on a u-blox M8/M10, 0
    keeps NMEA on every receiver. Read before the first setup (boot)."""
    global GPS_PVT
    v = _plain(env.get("GPS_PVT"))
    if not v:
        return
    v = v.lower()
    if v in ("auto", "1"):
        GPS_PVT = True
    elif v == "0":
        GPS_PVT = False
    else:
        print("GPS_PVT not auto or 0, using", "auto" if GPS_PVT else "0")

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
    5-600 s between pulses; anything else keeps the default. KEEPALIVE_MODE
    picks the pulse (scan / radio / gpio), KEEPALIVE_PIN the gpio,
    KEEPALIVE_CONNECTED=1 pulses while connected and idle too."""
    global PB_KEEPALIVE, PB_MODE, PB_PIN, PB_CONNECTED
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
    m = _plain(env.get("KEEPALIVE_MODE"))
    if m:
        m = m.lower()
        if m in ("scan", "radio", "gpio"):
            PB_MODE = m
        else:
            print("KEEPALIVE_MODE not scan/radio/gpio, using", PB_MODE)
    v = _plain(env.get("KEEPALIVE_PIN"))
    if v:
        try:
            n = int(v)
        except ValueError:
            n = None
        if n is None or n < 0:
            print("KEEPALIVE_PIN invalid, using", PB_PIN)
        elif _pin_taken(n):
            print("KEEPALIVE_PIN", n, "is in use (GPS, I2C, boot-bypass",
                  "or USB), using", PB_PIN)
        else:
            PB_PIN = n
    v = _plain(env.get("KEEPALIVE_CONNECTED"))
    if v:
        t = _truthy(v)
        if t is None:
            print("KEEPALIVE_CONNECTED not 1/0, using",
                  1 if PB_CONNECTED else 0)
        else:
            PB_CONNECTED = t

def _pin_taken(n):
    """A pin the board already uses or must not drive: GPS UART, I2C
    (OLED), main.py's boot-bypass pin (10 on the C6, 25 on the ESP32); on
    the C6 the USB D-/D+ pins 12/13 and UART0 16/17 (the console), the
    strapping pins 8/9 (a MOSFET gate's pull-down holds 9 low at reset:
    download mode) and the SPI flash pins 24-30; on the ESP32 the strapping
    pin 0 (download mode), UART0 1/3 and the SPI flash pins 6-11 (driving
    one crashes the board). Pulsing one of them breaks it."""
    if n in (GPS_TX, GPS_RX, I2C_SCL, I2C_SDA):
        return True
    if IS_C6:
        return n in (8, 9, 10, 12, 13, 16, 17) or 24 <= n <= 30
    return n in (0, 1, 3, 25) or 6 <= n <= 11

_pb_pin = None

def _pb_pulse_gpio():
    """PB_PIN high for PB_PULSE_MS: a load resistor through a MOSFET draws
    the burst a power bank looks for (22 ohm on 5 V: ~230 mA for 0.1 s,
    ~5 mA average at 5 s). False when the pin cannot be driven."""
    global _pb_pin
    try:
        if _pb_pin is None:
            _pb_pin = Pin(PB_PIN, Pin.OUT)
            _pb_pin.value(0)
        _pb_pin.value(1)
        time.sleep_ms(PB_PULSE_MS)
        _pb_pin.value(0)
        return True
    except Exception as e:
        try:
            _pb_pin.value(0)
        except Exception:
            pass
        print("keepalive gpio", PB_PIN, repr(e))
        return False

def _pb_pulse_radio(w, scan):
    """Radio on (and one scan), off again. The radio start-up alone is a
    ~100-200 mA burst for a fraction of a second; a scan holds it ~2 s."""
    try:
        w.active(True)
        if scan:
            aps = w.scan()
            _stay_scan(aps)
            _home_seen(aps)
    except WLAN_ERR:
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
    averages the load over a long time). Every PB_KEEPALIVE ms one pulse
    goes out (PB_MODE: a gpio load pulse, the radio turned on and off, or
    a full scan). Not while the radio is on anyway (connected, connecting,
    lost). GPS seconds published during a pulse are queued, not lost."""
    global PB_MODE
    w = wifi.get("wlan")
    state = wifi["state"]
    # radio idle: asleep, nothing found, or no WiFi at all (no .env, an
    # error): a logging-only board needs the pulses most
    idle = state in ("sleep", "not found", "no .env", "error", "off")
    conn = (PB_CONNECTED and state == "connected" and PB_MODE != "radio"
            and not str(up.get("state") or "").startswith("send"))
    if not PB_KEEPALIVE or not (idle or conn):
        wifi["pb_at"] = now
        return
    if _age(now, wifi["pb_at"]) < PB_KEEPALIVE:
        return
    wifi["pb_at"] = now
    feed()
    if PB_MODE == "gpio" and _pb_pulse_gpio():
        pass
    else:
        if PB_MODE == "gpio":
            PB_MODE = "scan"             # no such pin: fall back for good
            print("keepalive mode scan")
        if w is None:
            w = _pb_wlan()
        if w is None:
            return
        if conn:
            try:
                _stay_scan(w.scan())     # connected: scan, keep the link
            except WLAN_ERR:
                pass
        else:
            _pb_pulse_radio(w, PB_MODE == "scan")
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
    network at >= ROAM_MIN_RSSI while the radio sleeps or found nothing
    starts a round now. Before, a router restart (or a board booting
    faster than the router) left the page down for WIFI_SLEEP (1 h) while
    every scan saw home. At most every seen_wait ms, doubling while those
    rounds fail (wrong password, weak link)."""
    state = wifi["state"]
    if not WEB or state not in ("sleep", "not found"):
        return
    homes = [_ssid_b(n) for n in wifi.get("nets") or [] if n.get("home")]
    if not homes:
        return
    mine = wifi.get("home_aps") or ()    # the hidden home's learned BSSIDs
    for rec in aps:
        if rec[3] < ROAM_MIN_RSSI:
            continue
        raw = rec[0]
        if not isinstance(raw, bytes):
            raw = raw.encode()
        if (raw and raw in homes) or (not raw and rec[1] in mine):
            break
    else:
        return
    now = time.ticks_ms()
    at = wifi.get("seen_at")
    if at is not None and _age(now, at) < wifi["seen_wait"]:
        return
    wifi["seen_at"] = now
    wifi["seen_wait"] = min(wifi["seen_wait"] * 2, WIFI_BACKOFF_MAX)
    if state == "sleep":
        wifi["wake"] = True
    else:
        wifi["last_check"] = time.ticks_add(now, -wifi["backoff"])
    print("wifi home in range")

def wifi_start():
    global DEVICE_ID, HOSTNAME, CPU_MHZ, CPU_MHZ_UPLOAD
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
        wifi["pw"] = wifi["nets"][0]["pw"]
        wifi["home"] = wifi["nets"][0]["home"]
    up["url"] = env.get("GEO_URL", "").rstrip("/") or None
    up["key"] = env.get("API_KEY") or None
    _keepalive_env(env)
    _web_env(env)
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
    _oled_banner("WiFi scanning...")
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        _set_hostname(HOSTNAME)
        w.active(True)
    except WLAN_ERR:
        wifi["state"] = "error"
        return
    _wifi_begin_round(w)
    if wifi["state"] == "connecting":
        _wifi_poll(w)
    wifi["last_check"] = time.ticks_ms()

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
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
            if wifi.get("learn"):
                _learn_home(w)
            if _age(now, wifi["last_check"]) >= WIFI_CHECK:
                try:
                    wifi["rssi"] = w.status("rssi")
                except (OSError, ValueError, RuntimeError):
                    wifi["rssi"] = None
                wifi["last_check"] = now
            interval = wifi.get("roam_backoff") or ROAM_CHECK
            if not wifi.get("home") and _has_home():
                spd = st.get("speed")
                moving = (has_fix(now) and spd is not None and spd >= 5
                          and not (log.get("still") or {}).get("active"))
                if moving:
                    wifi["moved_at"] = now
                    arrival = False
                elif (wifi.get("moved_at") is not None
                      and _age(now, wifi["moved_at"]) >= 60000):
                    wifi["moved_at"] = None
                    arrival = True
                else:
                    arrival = False
                due = (not moving
                       and _age(now, wifi.get("roam_at") or now) >= interval)
                if due or arrival:
                    wifi["roam_at"] = now
                    _try_roam_home(w, arrival=arrival and not due)
            if (wifi["state"] == "connected"
                    and _age(now, wifi["idle_at"]) >= WIFI_IDLE):
                wifi["idle_at"] = now
                if WEB and (wifi.get("home") or WEB_AWAY):
                    pass                 # the status page keeps it on
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
        wifi["home"] = False
        wifi["ip"] = wifi["rssi"] = None
        wifi["try_at"] = now
        wifi["cand"] = []
        wifi["cand_i"] = 0
        wifi["learn"] = False            # (it may come back on a hotspot)
        wifi["rounds"] = 0               # the next round tries every network
        print("wifi lost", wifi.get("ssid") or "")
        return
    if wifi["state"] == "lost":
        if (s == network.STAT_CONNECTING or s == network.STAT_IDLE
                or s is None):
            if _age(now, wifi["try_at"] or now) < LOST_GRACE:
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
        if not wifi.get("wake"):
            at = wifi.get("wake_at")
            due = at is not None and not _waiting(now, at, UP_WAIT_MAX)
            if not due and _age(now, wifi["last_check"]) < WIFI_SLEEP:
                return
            if _age(now, wifi["idle_at"]) < WIFI_IDLE:
                return
            wifi["idle_at"] = now
            if (_nothing_to_send()
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

def online():
    return wifi["state"] == "connected"

def at_home():
    """Whether the connected network is configured as a home network, or
    the radio sleeps during a stay that began on one."""
    if wifi["state"] == "connected":
        return wifi.get("home")
    return wifi.get("sleep_home")

# --- GPX logger -------------------------------------------------------------
# Format (.trk): one point per line, "lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ"
# (~46 bytes; the old GPX XML was ~112). ele/hdop are empty when unknown.
# A "#TRK2" file (opened while the fixes came from NAV-PVT) has
# "lat,lon,ele,acc,speed,YYYYMMDDTHHMMSSZ": acc the receiver's horizontal
# accuracy (m), speed in m/s, each empty when unknown; the time stays last
# (every line's last 16 bytes, see flush). A file keeps its format for
# life (_trk_fmt): NMEA seconds in a TRK2 file (a receiver restart) write
# acc = HDOP * HDOP_M, NAV-PVT seconds in a TRK1 file hdop = acc / HDOP_M.
# Readers tell the two by the line (4 or 5 commas), .bad copies included.
# "-" starts a new segment. The file is always complete: each batch
# overwrites the footer with new points plus the footer again. LittleFS
# commits on close, so a power cut leaves either the previous or the new
# version. A line cut short has no newline and is skipped on read.
# Older .gpx files (XML) are still read and uploaded.

HEADER = "#TRK1\n"
HEADER2 = "#TRK2\n"
FOOTER = "#END\n"
NEW_SEG = "-\n"

log = {}

# The departure ring (see _still_active, _still_leave): the last
# STILL_DEP_MAX usable samples of a stay, preallocated once. Per sample its
# time in seconds from the ring's first sample (_RT; a stay's "rt0"), in
# _RF the latitude, the longitude and the distance from the anchor, in _RE
# the elevation, HDOP, accuracy and speed fields as the track line writes
# them (_fields: 8, 4, 5 and 5 bytes, space padded, both formats), and in
# _RI the integer position of a NAV-PVT sample (flagged in _RK, printed
# with _i7). A rebuilt line is the line _sample_line would have written
# in the file's format, rounding included: on the board a float is single
# precision, the very value an array("f") keeps; a double (a test host)
# takes three single floats, each what the ones before rounded off, which
# add up to it exactly. The track lines are rebuilt on leaving. As tuples of
# the line, its time and the position they took ~225 B a sample on the
# board: 90 of them grew the heap to 116 of 124 kB and left no block over
# 2 kB, so the next larger allocation took the heap mbedTLS needs.
from array import array
_NP = 1 if 1.0 + 1e-12 == 1.0 else 3     # floats a coordinate takes
_RW = 2 * _NP + 1                        # floats a sample takes (d last)
_RT = array("i", bytes(4 * STILL_DEP_MAX))         # (zeros, 4 B each:
_RF = array("f", bytes(4 * _RW * STILL_DEP_MAX))   # a raw initialiser)
RE_W = 22                                # ele 8, hdop 4, acc 5, speed 5
_RE = bytearray(RE_W * STILL_DEP_MAX)
_RPAD = b" " * RE_W                      # an empty _RE entry
_RI = array("i", bytes(8 * STILL_DEP_MAX))         # lat_i, lon_i
_RK = bytearray(STILL_DEP_MAX)                     # 1: _RI holds it

def log_reset():
    log.clear()
    log.update(file=None, created=False, batch=[], points=0, err=0,
               fix=False, lost_at=None, new_seg=False, seg_tc=[],
               full=False, wfail=False,
               still=_still_state(), clock=None, seq=0,
               last_sample=None, last_flush=time.ticks_ms(), last_t=None,
               prev=None, full_check=None, file_at=None, good_size=None,
               fmt=None)

def _still_state():
    return {"active": False, "home": False, "win_at": None, "win_tc": None,
            "ref": None, "sum": [0.0, 0.0, 0], "prev": None, "anchor": None,
            "aref": None, "aoff": (0.0, 0.0), "n": 0, "hb_at": None,
            "rn": 0, "ri": 0, "rt0": 0, "gap_n": 0, "gap_at": None,
            "gap_ref": None, "gap_seen": None, "pos": None,
            "far_n": 0, "far_d": 0, "far_slow": 0, "far_0": 0,
            "last_at": None, "last_d": 0, "file_at": None, "veto_at": None,
            "move_c": None, "wins": 0, "hold_tc": None, "aps": {},
            "ap_scans": 0, "ap_miss": 0, "ap_miss_at": None}

def _still_hold():
    """Track time from which a stay still being proven would drop its
    drift (the open window, or the previous one it may pair with), or None.
    Periodic flushes keep those lines in RAM, so a stay that is entered
    drops all of them: before, the minute already on flash was uploaded
    after every boot or new fix."""
    sl = log["still"]
    if sl["active"]:
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
        r, q = sl["ref"], sl["pos"]
        if (c is not None and q is not None
                and _dist(r[0], r[1], c[0], c[1]) > 2 * STILL_RADIUS
                and _dist(c[0], c[1], q[0], q[1]) > 2 * STILL_RADIUS):
            # the open window has spread out (its centroid far from both
            # its first and its latest fix): a drive, not a stay's drift.
            # Before, up to two minutes of a drive sat in RAM only. (Both
            # ends, so one multipath jump at either is still drift.)
            return h
        t = sl["win_tc"]
    else:
        return h
    return h if h is not None and h < t else t

def _still_mean(sl, pos):
    """Fold pos into the anchor, a running mean over STILL_MEAN_N fixes.
    Kept as an offset from a fixed reference: in single precision a
    latitude plus a centimetre-sized step is the same latitude."""
    r, o = sl["aref"], sl["aoff"]
    n = min(sl["n"] + 1, STILL_MEAN_N)
    o = (o[0] + (pos[0] - r[0] - o[0]) / n, o[1] + (pos[1] - r[1] - o[1]) / n)
    sl.update(aoff=o, n=n, anchor=(r[0] + o[0], r[1] + o[1]))

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

def _stamp():
    """YYYYMMDD_HHMMSS of the current fix (track file names)."""
    return st["date"].replace("-", "") + "_" + st["time"].replace(":", "")

def start_file(stamp=None):
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass                             # already exists
    name = stamp or _stamp()
    if not log["batch"]:                 # (held lines keep their format)
        log["fmt"] = _src_fmt()
    # (wfail: a write that failed on the previous file must not keep the
    # next one from closing: log_tick)
    log.update(file=LOG_DIR + "/" + name + ".trk", created=False, points=0,
               new_seg=False, seg_tc=[], file_at=time.ticks_ms(),
               good_size=None, wfail=False)
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
        log["full"] = True
        del batch[:]
        return
    creating = not log["created"]
    old = log.get("good_size")
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
    try:
        extra = len(NEW_SEG) * len(seg) if seg is not None else 0
        pts = 0
        i = 0
        while i < k:
            pts += len(batch[i])
            i += 1
        if creating:
            head = HEADER2 if log["fmt"] == 2 else HEADER
            expected = len(head) + extra + pts + len(FOOTER)
            f = open(path, "wb")
            f.write(head.encode())
        else:
            f = open(path, "r+b")
            _seek_footer(f, old)
            try:
                start = f.tell()
            except (OSError, AttributeError):
                start = (old or 0) - len(FOOTER)
                if start < 0:
                    start = 0
            expected = start + extra + pts + len(FOOTER)
        i = j = 0
        while i < k:
            if seg is not None and j < len(seg) and seg[j] == i:
                f.write(NEW_SEG.encode())
                j += 1
            f.write(batch[i].encode())
            i += 1
        f.write(FOOTER.encode())
        wrote = True
    except (OSError, MemoryError):
        log["err"] += 1
    if f is not None:
        try:
            f.close()
            close_ok = True
        except OSError:
            log["err"] += 1
    landed = wrote and close_ok
    if not landed:
        sz, gone = _stat_size(path)
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
            return
    log["created"] = True
    log["good_size"] = expected
    log["wfail"] = False
    log["points"] += k
    up["left"] += k
    q = log["seg_tc"]
    if log["new_seg"] and q:
        last = batch[k - 1][-17:-1]
        i = 0
        while i < len(q) and q[i] is not None and q[i] <= last:
            i += 1
        del q[:i]                        # the breaks written
        log["new_seg"] = bool(q)
    del batch[:k]

def _dist(lat1, lon1, lat2, lon2):
    """Metres between two nearby positions (equirectangular approximation)."""
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
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
    """The receiver's error estimate (GST) of this second, or None."""
    std, at = st.get("std"), st.get("std_at")
    if std is None or at is None or _age(now, at) >= STALE:
        return None
    return std

def _usable(now):
    """The current fix is good enough to use: HDOP at most MAX_HDOP and the
    receiver's own error estimate at most GPS_MAX_STD (each when known).
    The estimate catches a cold start converging: measured on the board, a
    good HDOP (1.2-3) while the position walked in from 230 m away. A
    NAV-PVT second: its horizontal accuracy at most PVT_MAX_HACC."""
    if st.get("lat_i") is not None:
        h = st.get("hacc")
        return h is not None and h <= PVT_MAX_HACC
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
    li = st.get("lat_i")
    if li is None:
        c = "%.6f,%.6f" % (lat, lon)
    else:
        c = _i7(li) + "," + _i7(st["lon_i"])
    return t, _line(c, _fields(now), _tc(), _trk_fmt())

def _line(c, fl, tc, fmt):
    """A track line: c "lat,lon", fl from _fields, tc compact time."""
    if fmt == 2:
        return "%s,%s,%s,%s,%s\n" % (c, fl[0], fl[2], fl[3], tc)
    return "%s,%s,%s,%s\n" % (c, fl[0], fl[1], tc)

def _fields(now):
    """The elevation, HDOP, accuracy and speed fields of a track line for
    the current fix ("" when not known), for either format (_line). Also
    what the departure ring keeps (_RE). From NMEA: acc = HDOP * HDOP_M,
    no speed; from NAV-PVT: hdop = acc / HDOP_M."""
    ele = ""
    if st["alt"] is not None and st.get("fixtype") != 2:
        ele = "%.1f" % st["alt"]
    h = st.get("hacc") if st.get("lat_i") is not None else None
    if h is not None:
        v = st.get("mps")
        return (ele, "%.1f" % (h / HDOP_M), "%.1f" % h,
                "%.1f" % v if v is not None else "")
    hdop = _fresh_hdop(now)
    if hdop is None:
        return ele, "", "", ""
    return ele, "%.1f" % hdop, "%.1f" % (hdop * HDOP_M), ""

def _src_fmt():
    """The track format of the current fix's source: 2 NAV-PVT, 1 NMEA."""
    return 2 if st.get("lat_i") is not None else 1

def _trk_fmt():
    """The format a line is written in: the open file's (or that of the
    lines waiting for one), else the current fix's source's."""
    if log["file"] is not None or log["batch"]:
        return log.get("fmt") or 1
    return _src_fmt()

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
    d, t = st["date"], st["time"]
    try:
        return ((_days(int(d[0:4]), int(d[5:7]), int(d[8:10])) - EPOCH_DAYS)
                * 86400 + int(t[0:2]) * 3600 + int(t[3:5]) * 60
                + int(t[6:8]))
    except (TypeError, ValueError, IndexError):
        return None

def _fix_secs():
    """UTC seconds (since 1970) of the current fix's date and time, or None."""
    s = _fix_s()
    return None if s is None else s + EPOCH_DAYS * 86400

def _stamp_at(secs):
    """(ISO time, compact track time) for UTC seconds since 1970."""
    return _stamp_s(secs - EPOCH_DAYS * 86400)

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
        sl.update(win_at=now, win_tc=_tc(), ref=pos, sum=[0.0, 0.0, 0])
    r, sm = sl["ref"], sl["sum"]
    sm[0] += pos[0] - r[0]
    sm[1] += pos[1] - r[1]
    sm[2] += 1
    if _age(now, sl["win_at"]) < STILL_WINDOW:
        return None
    if sm[2] < STILL_MIN_FIXES and _age(now, sl["win_at"]) < STILL_WINDOW_MAX:
        return None                      # sparse fixes: collect more first
    c = None
    if sm[2] >= STILL_MIN_FIXES:
        c = (r[0] + sm[0] / sm[2], r[1] + sm[1] / sm[2])
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
         prev[0][1] + (c[1] - prev[0][1]) * sm[2] / w)
    return a, prev[1], w

def _still_heartbeat(now):
    """Write the anchor as a point. It is stamped with the last fix's GPS
    time plus the ticks since, so no fix is needed: the anchor is known, and
    indoors there is rarely a usable fix. Skipped when that time would not
    come after the last line written (retried on the next tick)."""
    sl = log["still"]
    c = log["clock"]
    if c is None:
        return
    age = _age(now, c[1])
    if age >= TICKS_OLD:
        return
    t, tc = _stamp_s(c[0] + age // 1000)
    if log["last_t"] is not None and t <= log["last_t"]:
        return
    a = sl["anchor"]
    sl["hb_at"] = now
    if log["file"] is None:
        start_file(tc[:8] + "_" + tc[9:15])
        sl["file_at"] = now
    _record((t, _line("%.6f,%.6f" % (a[0], a[1]), ("", "", "", ""), tc,
                      _trk_fmt())), a)

def _still_beat(now):
    """Heartbeat of a stay away from home, run every tick, fix or not. Not
    while the last usable sample was recently out past STILL_NEAR: that is
    an excursion or a departure, and an anchor stamped then would land in
    the middle of the way out."""
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
        return
    # Rotate the anchor file so the stay uploads while it lasts (the usual
    # age-close is off during a stay: it would open a file per heartbeat).
    if (log["file"] is not None and sl["file_at"] is not None
            and wifi["state"] == "connected"
            and _age(now, sl["file_at"]) >= STILL_FILE):
        _still_close(now)
    if sl["hb_at"] is not None and _age(now, sl["hb_at"]) < STILL_HEARTBEAT:
        return
    if (sl["last_at"] is not None and _age(now, sl["last_at"]) < STILL_GAP
            and sl["last_d"] > STILL_NEAR):
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
        start_file(tc[:8] + "_" + tc[9:15])
        log["new_seg"], log["seg_tc"] = seg    # a break among the held lines
        return True
    return False

def _still_enter(now, anchor, since_tc, n):
    """Become stationary at anchor. The stay's drift (lines from since_tc
    on, all still in RAM: see _still_hold) is dropped, except the way in
    up to where it first comes within STILL_NEAR of the anchor, and the
    track is closed, so the arrival uploads during the stay. Away from
    home the anchor opens a new file, now and then every STILL_HEARTBEAT."""
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
    while i < len(b):
        f = b[i].split(",", 2)
        if _dist(anchor[0], anchor[1], float(f[0]), float(f[1])) <= STILL_NEAR:
            break
        i += 1
    del b[i:]
    _still_close()                       # the arrival uploads during the stay
    if log["file"] is not None:
        sl["file_at"] = now              # close failed: rotate it later
    if at_home():
        sl["home"] = True
    elif wifi_wake():
        # the radio slept: the first anchor point waits for the round, so
        # a home stay is not stamped with one
        sl["hb_at"] = time.ticks_add(now, WIFI_WAKE_WAIT - STILL_HEARTBEAT)
    else:
        _still_heartbeat(now)
    print("stay", "home" if sl["home"] else "away", "kept", i)

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
    own = [_ssid_b(n) for n in wifi.get("nets") or []]
    seen = []
    for rec in aps:
        raw = rec[0]
        if not isinstance(raw, bytes):
            raw = raw.encode()
        b = bytes(rec[1])
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

def _ring_add(sl, now, pos, d):
    """Keep a usable sample of the stay in the departure ring (see _RT),
    the oldest overwritten when it is full. A sample that falls back from
    the one before by more than STILL_TOL starts it over: the ring is
    always a run that kept receding, the way out (_still_leave)."""
    s = _fix_s()
    if s is None:
        return
    cap = len(_RT)
    n, i = sl["rn"], sl["ri"]
    if n and d + STILL_TOL < _RF[_RW * ((i + n - 1) % cap) + _RW - 1]:
        n = 0
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
    k = _RW * j
    _ring_put(k, pos[0])
    _ring_put(k + _NP, pos[1])
    _RF[k + _RW - 1] = d
    li = st.get("lat_i")
    _RK[j] = 0 if li is None else 1
    if li is not None:
        _RI[2 * j], _RI[2 * j + 1] = li, st["lon_i"]
    k = RE_W * j
    _RE[k:k + RE_W] = _RPAD
    o = 0
    for x, w in zip(_fields(now), (8, 4, 5, 5)):
        if len(x) <= w:                  # (beyond 99999.9 m: none)
            _RE[k + o:k + o + len(x)] = x.encode()
        o += w

def _ring_put(k, v):
    """Keep coordinate v in _RF from k on (see _NP)."""
    m = k + _NP
    while k < m:
        _RF[k] = v
        v -= _RF[k]                      # (what single precision lost)
        k += 1

def _ring_deg(k):
    """A coordinate kept in the departure ring from _RF[k] on (_NP floats,
    added up largest first: exact)."""
    v = _RF[k]
    m = k + _NP
    k += 1
    while k < m:
        v += _RF[k]
        k += 1
    return v

def _still_leave(wifi_seen=False):
    """End the stay and write the way out: the samples in the departure
    ring, a run whose distance from the anchor kept growing (within
    STILL_TOL), so the departure starts near the anchor, not STILL_EXIT
    away. With wifi_seen (the stay's access points are gone, _stay_scan)
    only if the last sample was out past STILL_NEAR: otherwise the ring is
    the stay's drift, and the stay just ends."""
    sl = log["still"]
    n, i0, cap = sl["rn"], sl["ri"], len(_RT)
    if (wifi_seen and n
            and _RF[_RW * ((i0 + n - 1) % cap) + _RW - 1] <= STILL_NEAR):
        n = 0
    i = 0
    prev = None
    while i < n:
        j = (i0 + i) % cap
        i += 1
        p = (_ring_deg(_RW * j), _ring_deg(_RW * j + _NP))
        t, tc = _stamp_s(sl["rt0"] + _RT[j])
        # only what comes after the last line (an anchor point may have
        # been written while the way out began), so the track stays in order
        if (log["last_t"] is not None and t <= log["last_t"]) or (
                prev is not None
                and _dist(prev[0], prev[1], p[0], p[1]) <= MIN_MOVE):
            continue
        k = RE_W * j
        if _RK[j]:
            c = _i7(_RI[2 * j]) + "," + _i7(_RI[2 * j + 1])
        else:
            c = "%.6f,%.6f" % (p[0], p[1])
        line = _line(c, (_RE[k:k + 8].decode().strip(),
                         _RE[k + 8:k + 12].decode().strip(),
                         _RE[k + 12:k + 17].decode().strip(),
                         _RE[k + 17:k + 22].decode().strip()), tc,
                     _trk_fmt())
        if log["file"] is None:
            start_file(tc[:8] + "_" + tc[9:15])
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
            _still_mean(sl, pos)
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
    if (log["file"] is not None and log.get("file_at") is not None
            and wifi["state"] == "connected" and not log["still"]["active"]
            and log["points"] + len(log["batch"]) > 0
            and _age(now, log["file_at"]) >= NEW_FILE_GAP * 1000
            and _age(now, log["last_flush"]) >= FLUSH):
        _close_file()
    if log["full"]:
        last = log.get("full_check")
        if last is not None and _age(now, last) < FULL_CHECK:
            return
        log["full_check"] = now
        room = free()
        if room is None or room < MIN_FREE:
            return
        log["full"] = False
    if not has_fix(now):
        if log["fix"]:
            _drain_q()                   # valid seconds just before the loss
        else:
            st["q"] = []                 # stale
        # the heartbeat after the queued seconds: a departure in them comes
        # first, and nothing is stamped before lines already written
        _still_beat(now)
        if log["fix"]:
            log["fix"], log["lost_at"] = False, now
            flush(_still_hold())
        elif log["batch"] and _age(now, log["last_flush"]) >= FLUSH:
            # anchor points written without a fix; and HOLD_RELEASE after
            # the fix was lost the drift hold is let go: without a new fix
            # the end of a trip would stay in RAM only (lost on power-off)
            hold = _still_hold()
            if (hold is not None and log.get("lost_at") is not None
                    and _age(now, log["lost_at"]) >= HOLD_RELEASE):
                hold = None
            flush(hold)
        return
    if not log["fix"]:
        log["fix"] = True
        if log["still"]["active"]:
            pass                         # same place: the stay keeps its file
        elif (log["file"] is not None and log.get("lost_at") is not None
                and _age(now, log["lost_at"]) > NEW_FILE_GAP * 1000):
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
    if secs is not None:
        if not 0 <= secs < FIX_S_END:
            return                       # (see FIX_S_END)
        log["clock"] = (secs, now)       # GPS time for the heartbeat (2020)
    if not st["date"] or not _usable(now):
        return                           # (a second with GGA only, e.g. the
                                         # first one after a warm boot)
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
        _still_enter(now, hit[0], since, hit[2])

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
# never delete the quarantined data.
#
# TLS (mbedTLS) needs ~40 kB contiguous from the ESP-IDF heap, outside the
# Python heap. If the Python heap ever grows (see gc.threshold at the top) it
# takes that block, so allocations here stay small: the request body goes
# into a buffer allocated once at import. For the same reason the module must
# be deployed precompiled (gps_logger.mpy): compiling the source on the
# device grows the heap for good.

_body = bytearray(BODY_CAP)

up = {}

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

def _stat_size(path):
    """(size_or_None, missing). missing is True only on ENOENT."""
    try:
        return os.stat(path)[6], False
    except OSError as e:
        return None, _enoent(e)

def _file_size(path):
    try:
        return os.stat(path)[6]
    except OSError:
        return None

def _seek_footer(f, old):
    """Seek to the last known FOOTER, or after the last newline if truncated.

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
            return
        f.seek(0, 2)
        if f.tell() > old:
            f.seek(pos, 0)
            return
    else:
        f.seek(-n, 2)
        pos = f.tell()
        if f.read(n) == foot:
            f.seek(pos, 0)
            return
    f.seek(0, 2)
    end = f.tell()
    win = end if end < LINE_MAX else LINE_MAX
    if win <= 0:
        f.seek(0, 0)
        return
    start = end - win
    f.seek(start, 0)
    chunk = f.read(win)
    i = chunk.rfind(b"\n")
    if i < 0:
        f.seek(0, 0)
        return
    f.seek(start + i + 1, 0)

def _lowidf_mark_take():
    """True once if the last boot was a low-idf reboot (LOWIDF_MARK)."""
    try:
        os.stat(LOWIDF_MARK)
    except OSError:
        return False
    _rm(LOWIDF_MARK)
    return True

def up_reset():
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, fail4=0, backoff=BACKOFF_START, allow_bad=True,
              bad_try={}, skip=None, hold={}, bad_full={}, read_err={},
              enomem=0, last_err=None, guard_hits=_guard["hits"],
              lowidf_at=None, lowidf_seen=None,
              lowidf_boot=_lowidf_mark_take(), next_at=time.ticks_ms())
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
    _oled_banner("Counting points")
    try:
        up["left"] = count_points()
    except MemoryError:
        gc.collect()
        up["left"] = 0
        print("count MemoryError")

def read_offset(path):
    """Byte offset to resume from. Corrupt / past-EOF sidecars reset to 0
    (Dawarich dedup handles the resend). Never treat a bad .pos as EOF."""
    size = _file_size(path)
    if size is None:
        return 0
    try:
        with open(path + ".pos") as f:
            off = int(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0
    if off < 0 or off > size:
        print("pos corrupt", path, off, size)
        return 0
    if off == 0:
        return 0
    try:
        with open(path, "rb") as f:
            f.seek(off - 1)
            b = f.read(1)
        if b != b"\n":
            print("pos misaligned", path, off)
            return 0
    except OSError:
        return 0
    return off

def write_offset(path, off):
    """Sidecar update: write <file>.pos.tmp, then rename over .pos.
    Atomic on LittleFS; on FAT a crash may drop the .pos (resend from 0)."""
    tmp = path + ".pos.tmp"
    with open(tmp, "w") as f:
        f.write("%d\n" % off)
    os.rename(tmp, path + ".pos")

# Track files are read line by line into one buffer (readinto), never a
# new bytes object per line: f.readline(LINE_MAX) allocated LINE_MAX bytes
# for each line, and the upload reads at the worst moment for the heap.
_rbuf = bytearray(LINE_MAX)
_rmv = memoryview(_rbuf)
_rd = [0, 0, 0, 0]                       # file offset of _rbuf[0], line start,
                                         # line end, bytes in _rbuf

def _rd_open(off):
    _rd[0] = off
    _rd[1] = _rd[2] = _rd[3] = 0

def _rd_fill(f):
    """Fill _rbuf from f's position: the bytes read (fewer only at EOF)."""
    n = 0
    while n < LINE_MAX:
        k = f.readinto(_rmv[n:] if n else _rbuf)
        if not k:
            break
        n += k
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

def _rd_point(a, b, gpx):
    """_rbuf[a:b] is a point line (see _is_point)."""
    if gpx:
        return _rbuf.find(b"<trkpt", a, min(a + 6, b)) == a
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
                 if n.endswith((".gpx", ".trk"))]
    except OSError:
        return 0
    total = 0
    nlines = 0
    for n in names:
        path = LOG_DIR + "/" + n
        gpx = not _is_trk(path)
        try:
            off = read_offset(path)
            with open(path, "rb") as f:
                _rd_open(off)
                while _rd_line(f):
                    a, b = _rd[1], _rd[2]
                    if a == b:
                        continue
                    if not gpx and _rd_footer(a, b):
                        break
                    nlines += 1
                    if nlines & 63 == 0:
                        feed()
                    if _rd_point(a, b, gpx):
                        total += 1
        except (OSError, MemoryError) as e:
            print("count", path, repr(e))
            gc.collect()
    return total

def pending():
    """Finished track files, oldest first (names are YYYYMMDD_HHMMSS).
    Sidecars (.pos) and quarantined (.bad) files are not listed."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR)
                       if n.endswith((".gpx", ".trk")))
    except OSError:
        return []
    return [p for p in (LOG_DIR + "/" + n for n in names) if p != log["file"]]

def pending_bad():
    """Rejected-batch files left for later recovery."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR)
                       if n.endswith((".gpx.bad", ".trk.bad")))
    except OSError:
        return []
    return [LOG_DIR + "/" + n for n in names]

def _attr(line, name):
    i = line.find(name + '="')
    if i < 0:
        return None
    i += len(name) + 2
    j = line.find('"', i)
    if j < 0:
        return None
    return line[i:j]

def _tag(line, name):
    i = line.find("<" + name + ">")
    if i < 0:
        return None
    i += len(name) + 2
    j = line.find("</" + name + ">", i)
    if j < 0:
        return None
    return line[i:j]

def _float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None

def _finite(x):
    try:
        return math.isfinite(x)
    except AttributeError:
        return x == x and x != float("inf") and x != float("-inf")

def _valid_time(t):
    """ISO UTC `YYYY-MM-DDTHH:MM:SSZ` with year 2020-2099."""
    if not t or len(t) != 20:
        return False
    if (t[4] != "-" or t[7] != "-" or t[10] != "T"
            or t[13] != ":" or t[16] != ":" or t[19] != "Z"):
        return False
    for i in (0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18):
        if t[i] < "0" or t[i] > "9":
            return False
    year = int(t[0:4])
    return 2020 <= year <= 2099

def _put(pos, b):
    _body[pos:pos + len(b)] = b
    return pos + len(b)

def _is_trk(path):
    return path.endswith(".trk") or path.endswith(".trk.bad")

def _is_point(line, gpx):
    """One point line: <trkpt> in old .gpx, "lat,lon,ele,hdop,time" (TRK1)
    or "lat,lon,ele,acc,speed,time" (TRK2) in .trk. A line without its
    newline (power cut mid-write) is not a point."""
    if gpx:
        return line.startswith(b"<trkpt")
    return (line[-1:] == b"\n" and 4 <= line.count(b",") <= 5
            and line[:1] != b"#")

def _point(line, gpx):
    """(lat, lon, time, ele, hdop) as strings, time as ISO UTC, or None
    (a TRK2 line: its acc in place of hdop)."""
    if gpx:
        s = line.decode()
        return (_attr(s, "lat"), _attr(s, "lon"), _tag(s, "time"),
                _tag(s, "ele"), _tag(s, "hdop"))
    f = line[:-1].decode().split(",")
    tc = f[-1]
    if len(tc) != 16 or tc[8] != "T" or tc[15] != "Z":
        return None
    t = "%s-%s-%sT%s:%s:%sZ" % (tc[0:4], tc[4:6], tc[6:8],
                                tc[9:11], tc[11:13], tc[13:15])
    return (f[0], f[1], t, f[2], f[3])

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
_TDIG = (0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14)
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
    if not _finite(v):
        return 0
    return _ptnum(i, a, b, v, fmt, 1, imax)

def _trk_point(a, b):
    """Parse the .trk line _rbuf[a:b] in place into _pt -> -1 (not a point
    line), 0 (a point, not valid) or the length of its Overland JSON.
    Same rules as _point and the checks in read_batch. A TRK1 line's hdop
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
    y = (rb[t] - 48) * 1000 + (rb[t + 1] - 48) * 100 + (rb[t + 2] - 48) * 10
    if not 2020 <= y + rb[t + 3] - 48 <= 2099:
        return 0
    lat, lon = _fnum(a, c1), _fnum(c1 + 1, c2)
    if (lat is None or lon is None or not _finite(lat) or not _finite(lon)
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
        elif _finite(v):
            _pt[10] = _pt[11] = 0
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

def _gpx_loc(line):
    """Overland JSON of one legacy .gpx point line, or None if not valid."""
    try:
        p = _point(line, True)
    except UnicodeError:
        return None
    lat_f = lon_f = None
    if p is not None:
        lat_f, lon_f = _float(p[0]), _float(p[1])
    if not (p is not None and lat_f is not None and lon_f is not None
            and _finite(lat_f) and _finite(lon_f)
            and abs(lat_f) <= 90 and abs(lon_f) <= 180
            and _valid_time(p[2])):
        return None
    ele_f = _float(p[3]) if p[3] else None
    if ele_f is not None and not _finite(ele_f):
        ele_f = None
    hdop_f = _float(p[4]) if p[4] else None
    extra = ""
    if ele_f is not None:
        extra += '"altitude":%.1f,' % ele_f
    if hdop_f is not None and _finite(hdop_f):
        extra += '"horizontal_accuracy":%.1f,' % (hdop_f * HDOP_M)
    return ('{"type":"Feature","geometry":{"type":"Point",'
            '"coordinates":[%.6f,%.6f]},"properties":{'
            '"timestamp":"%s",%s"device_id":"%s"}}'
            % (lon_f, lat_f, p[2], extra, DEVICE_ID)).encode()

def read_batch(path, off=0):
    """Fill _body with up to UPLOAD_BATCH valid points from byte offset off
    as Overland JSON -> (n, n_ok, body_len, new_off, eof).
    Truncated / non-numeric / inf / over-long points are skipped.
    A valid point that does not fit is left unread (eof=False). A .trk is
    parsed in _rbuf and written straight into _body: a few small objects
    per point (the float checks), not ~25 and a line buffer."""
    gpx = not _is_trk(path)
    n, n_ok, pos, sep = 0, 0, _put(0, _J_START), 0
    nlines, eof = 0, False
    with open(path, "rb") as f:
        _rd_open(off)
        while True:
            if not _rd_line(f):
                eof = True
                break
            a, b = _rd[1], _rd[2]
            nlines += 1
            if nlines & 63 == 0:
                feed()
            if a == b:
                continue
            loc = None
            if gpx:
                line = bytes(_rmv[a:b])
                if not _is_point(line, True):
                    continue
                loc = _gpx_loc(line)
                need = len(loc) if loc else 0
            elif _rd_footer(a, b):
                # the footer ends a .trk: bytes after it are left over from
                # a longer flush that failed (a later, shorter one wrote its
                # footer before them), lines the logger dropped
                eof = True
                break
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
                    n -= 1
                    eof = False
                    break
            if need:
                if sep:
                    _body[pos] = 44      # ,
                    pos += 1
                pos = _put(pos, loc) if gpx else _trk_put(pos)
                sep = 1
                n_ok += 1
            if n >= UPLOAD_BATCH:
                break
        new_off = _rd[0] + _rd[2]
    if not sep:
        return n, 0, 0, new_off, eof
    return n, n_ok, _put(pos, b"]}"), new_off, eof

def commit_offset(path, n, new_off, eof=False):
    """Record that n points are done. Delete only at true EOF.
    Crash between POST and this call resends (at-least-once). At EOF the
    file goes without a .pos write first: on a full flash that write
    failed, so the file was never deleted and no room came back."""
    if eof:
        _rm(path)
        _rm(path + ".pos")
        _clear_bad_full(path)
    else:
        write_offset(path, new_off)
    up["left"] = max(up["left"] - n, 0)

def _append_bad(path, off, new_off):
    """Copy consumed point lines to <path>.bad. False if not durable."""
    if path.endswith(".bad"):
        return False
    bad = path + ".bad"
    sz = _file_size(bad) or 0
    need = max(new_off - off, 0)
    if sz >= MAX_BAD_BYTES or sz + need > MAX_BAD_BYTES:
        print("bad file full", bad)
        return False
    room = free()
    if room is not None and room - need < MIN_FREE:
        print("bad file: flash reserve", bad)
        return False
    gpx = not _is_trk(path)
    try:
        with open(path, "rb") as src, open(bad, "ab") as dst:
            _rd_open(off)
            while _rd[0] + _rd[2] < new_off and _rd_line(src):
                a, b = _rd[1], _rd[2]
                if a < b and _rd_point(a, b, gpx):
                    dst.write(_rmv[a:b])
    except (OSError, MemoryError) as e:
        print("quarantine", repr(e))
        gc.collect()
        return False
    return True

def _quarantine_batch(path, off, n, new_off, eof):
    """Park one rejected batch only if the copy landed on flash."""
    if path.endswith(".bad"):
        up["state"] = "err 422"
        return False
    if not _append_bad(path, off, new_off):
        up["state"] = "bad full"
        bf = up.get("bad_full") or {}
        bf[path] = True
        up["bad_full"] = bf
        return False
    try:
        commit_offset(path, n, new_off, eof)
    except OSError as e:
        # the copy landed but the offset did not: the next try copies the
        # batch again (duplicates in .bad, deduplicated by Dawarich)
        print("quarantine offset", repr(e))
        up["err"] += 1
        up["state"] = "fs err"
        return False
    up["allow_bad"] = False
    up["state"] = "quarantine"
    return True

def _retry_after_ms(r):
    hdrs = getattr(r, "headers", None) or {}
    v = None
    if isinstance(hdrs, dict):
        for k in hdrs:
            if str(k).lower() == "retry-after":
                v = hdrs[k]
                break
    if v is None:
        return None
    try:
        ms = int(v) * 1000
    except (TypeError, ValueError):
        return None
    return min(max(ms, BACKOFF_START), BACKOFF_MAX)

def _schedule(now, ms=None):
    if ms is None:
        b = up.get("backoff", BACKOFF_START) or BACKOFF_START
        ms = b
        up["backoff"] = min(max(b * 2, BACKOFF_START), BACKOFF_MAX)
    up["next_at"] = time.ticks_add(now, ms)

def _hold_file(path, now, ms):
    """Backoff one file so a stuck oldest track does not block the rest."""
    h = up.get("hold") or {}
    h[path] = time.ticks_add(now, ms)
    up["hold"] = h

def _bad_full_left():
    """True while a .bad that hit MAX_BAD_BYTES still exists on flash."""
    bf = up.get("bad_full") or {}
    if not bf:
        return False
    keep = {}
    for p in bf:
        if _file_size(p) is not None:
            keep[p] = True
    up["bad_full"] = keep
    return bool(keep)

def _clear_bad_full(path):
    bf = up.get("bad_full") or {}
    if path in bf:
        bf.pop(path, None)
        up["bad_full"] = bf

class _Resp:
    def __init__(self, code, retry):
        self.status_code = code
        self.headers = {"Retry-After": retry} if retry is not None else {}

    def close(self):
        pass

def _io_write(s, b):
    """All of b to a socket or TLS stream (MicroPython: write; CPython
    test doubles: sendall)."""
    w = getattr(s, "write", None)
    if w is None:
        s.sendall(b)
        return
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
    that cannot be read, or is empty, raises: no POST, never a silent
    fall back to an unverified one."""
    ctx = _tls_ctx[0]
    if ctx is not None:
        return ctx
    try:
        import tls
    except ImportError:
        import ssl as tls
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
        up["enomem"] = up.get("enomem", 0) + 1 if oom else 0
        up["err"] += 1
        up["state"] = "low mem" if oom else "ca err"
        err = repr(e)[:60]
        print("upload ca", err)
        if err != up.get("last_err"):
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
    2020-2087) before a POST: once a boot, again only when it drifted more
    than RTC_DRIFT s. Seconds since 2020 (small ints). From NAV-PVT only
    when its date and time are valid and fully resolved and tAcc is under
    PVT_MAX_TACC; then no position fix is needed (indoors at home)."""
    if st.get("fix_src") == "pvt":
        at = st.get("t_at")
        if not st.get("t_ok") or at is None or _age(now, at) >= STALE:
            return
    elif not has_fix(now):
        return
    else:
        at = st["pos_at"]
    s = _fix_s()
    if s is None:
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
        _diag_write("clock set %s" % _stamp_s(s)[0])
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

def _raw_post(url, data, headers, timeout):
    """A minimal HTTP/1.0 POST, no redirects: micropython-lib requests
    followed one as a GET with the old Content-Length (the server then
    waits for a body) and the Bearer header, without the timeout. The body
    goes out in HTTP_CHUNK pieces and the wait for the answer polls, both
    reading the UART meanwhile, so a slow server costs no NMEA. -> _Resp."""
    proto, _, rest = url.partition("://")
    host, _, path = rest.partition("/")
    path = "/" + path
    port = 443 if proto == "https" else 80
    if ":" in host:
        host, _, p = host.rpartition(":")
        port = int(p)
    _post_phase[0] = "connect"
    ai = socket.getaddrinfo(host, port, 0, socket.SOCK_STREAM)[0]
    s = socket.socket(ai[0], socket.SOCK_STREAM, ai[2])
    _guard_start(s, HTTP_MAX)            # the raw socket: closing it fails
    try:                                 # a TLS call stuck on it
        s.settimeout(timeout)
        s.connect(ai[-1])
        if proto == "https":
            _heap_note("connect")
            _post_phase[0] = "tls"       # the handshake
            s = _tls_wrap(s, host)
            _heap_note("tls_wrap")
        _post_phase[0] = "http"
        hdr = "POST %s HTTP/1.0\r\nHost: %s\r\n" % (path, host)
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
        line = s.readline(HTTP_LINE)
        code = int(line.split(None, 2)[1])
        retry = None
        try:
            # the status is in: a reset, a timeout or the guard closing
            # the socket in the headers must not turn a 2xx into a resend
            # (a duplicate batch). Bounded reads: a long header comes in
            # pieces, only a piece at a line start is a header or the end.
            bol = True
            for _ in range(HTTP_LINES):
                h = s.readline(HTTP_LINE)
                if not h or (bol and h in (b"\r\n", b"\n")):
                    break
                if bol:
                    k, _, v = h.partition(b":")
                    if k.strip().lower() == b"retry-after":
                        retry = v.strip().decode()
                bol = h[-1:] == b"\n"
        except Exception as e:
            print("upload head", code, repr(e))
        return _Resp(code, retry)
    finally:
        _guard_stop()
        try:
            s.close()
        except OSError:
            pass

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
    r = None
    code, retry = None, None
    _post_phase[0] = None
    try:
        r = HTTP_POST("%s/api/v1/overland/batches" % up["url"],
                      memoryview(_body)[:size],
                      {"Content-Type": "application/json",
                       "Authorization": "Bearer %s" % up["key"]},
                      HTTP_TIMEOUT)
        code = r.status_code
        retry = _retry_after_ms(r)
        up["enomem"] = 0
        up["last_err"] = None
        # the guard may have closed the socket while the headers were read
        # (the status kept): not a sign of the next error's abort
        up["guard_hits"] = _guard["hits"]
    except MemoryError:
        up["enomem"] = up.get("enomem", 0) + 1
        raise
    except Exception as e:
        # "in a row": any other failure ends a run of out-of-memory ones
        up["enomem"] = up.get("enomem", 0) + 1 if _enomem(e) else 0
        print("upload", repr(e))
        if _guard["hits"] != up.get("guard_hits"):
            up["guard_hits"] = _guard["hits"]
            print("upload: POST over %d s, aborted" % (HTTP_MAX // 1000))
            _diag_write("upload aborted after %d s" % (HTTP_MAX // 1000))
        err = repr(e)[:60]
        if err != up.get("last_err"):    # one line per kind of error
            up["last_err"] = err
            _diag_write("upload err %s" % err)
    finally:
        if r is not None:
            try:
                r.close()
            except Exception:
                pass
        feed()
        gc.collect()
    if code is not None and 300 <= code < 400:
        print("GEO_URL redirects (%d): put the final https origin in .env"
              % code)
    return code, retry

def _tls_reboot():
    """TLS_REBOOT_N TLS allocations failed in a row at home: the IDF heap
    is fragmented for good. Write what is in RAM and restart."""
    if up.get("enomem", 0) < TLS_REBOOT_N or not at_home():
        return False
    print("upload: TLS out of memory %d times, reboot" % up["enomem"])
    try:
        flush(_still_hold())
    except Exception:
        pass
    _diag_write("reboot tls memory %d" % up["enomem"])
    machine.reset()
    return True

def _lowidf_reboot(big, low_ms):
    """At home, the largest IDF heap block too small for TLS for
    LOWIDF_WAIT with tracks waiting (gc.collect() did not bring it back):
    write what is in RAM and restart. Its own reason, not the ENOMEM run;
    at most one per LOWIDF_REBOOT_GAP (LOWIDF_MARK tells the next boot)."""
    if not at_home() or low_ms < LOWIDF_WAIT:
        return False
    if up.get("lowidf_boot") and uptime_s() * 1000 < LOWIDF_REBOOT_GAP:
        return False                     # rebooted for it a short while ago
    print("upload: IDF heap block %d KB for %d s, reboot" % (
        big, low_ms // 1000))
    try:
        flush(_still_hold())
    except Exception:
        pass
    _diag_write("reboot low idf %d KB %d s" % (big, low_ms // 1000))
    try:
        with open(LOWIDF_MARK, "w") as f:
            f.write("1")
    except OSError:
        pass
    machine.reset()
    return True

def upload_tick():
    """Send one batch when WiFi is up and due. Runs on home and hotspot."""
    now = time.ticks_ms()
    if (wifi["state"] != "connected" or up["state"] == "no .env"
            or _waiting(now, up["next_at"], UP_WAIT_MAX)):
        return
    if not up["url"] or not up["key"]:
        up["state"] = "no .env"
        return
    files = pending()
    up["files"] = len(files)
    skip = up.get("skip")
    if skip and skip in files and len(files) > 1:
        files = [p for p in files if p != skip]
    held = up.get("hold") or {}
    due = []
    for p in files:
        t = held.get(p)
        if t is None or not _waiting(now, t, UP_WAIT_MAX):
            due.append(p)
    files = due
    from_bad = False
    if not files and up.get("allow_bad"):
        files = pending_bad()
        from_bad = bool(files)
        files = [p for p in files
                 if (up.get("bad_try") or {}).get(p, 0) < MAX_BAD_RETRY]
        due = []
        for p in files:
            t = held.get(p)
            if t is None or not _waiting(now, t, UP_WAIT_MAX):
                due.append(p)
        files = due
    if not files:
        if _bad_full_left():
            up["state"] = "bad full"
        elif up["files"] and held:
            up["state"] = "held"         # every waiting track held back
        else:
            up["state"] = "idle"
        up["next_at"] = time.ticks_add(now, BACKOFF_START)
        return
    path = files[0]
    off = read_offset(path)
    try:
        _heap_mark()
        n, n_ok, size, new_off, eof = read_batch(path, off)
        _heap_note("read_batch")
        feed()
        fsize = _file_size(path) or 0
    except OSError as e:
        # unreadable (corrupt LittleFS block, EIO): hold it back so the
        # newer tracks still go, and after READ_ERR_MAX tries park it as
        # <file>.err (left for a manual look) instead of blocking for good
        up["err"] += 1
        up["state"] = "fs err"
        re_ = up["read_err"]
        re_[path] = re_.get(path, 0) + 1
        print("upload read", path, repr(e))
        if re_[path] >= READ_ERR_MAX:
            re_.pop(path, None)
            try:
                os.rename(path, path + ".err")
                _rm(path + ".pos")
                print("parked", path + ".err")
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
                commit_offset(path, n, new_off, eof)
            elif eof or fsize == 0:
                _rm(path)                # empty / EOF track and .bad
                _rm(path + ".pos")
                _clear_bad_full(path)
            else:
                print("pos skip", path, off, new_off, fsize)
                write_offset(path, 0)
        except OSError as e:
            print("upload pos", path, repr(e))
            _fs_err(now)
            return
        except MemoryError:
            _low_mem(now)
            return
        up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
        return
    gc.collect()                         # frees an empty grown heap area
    big = idf_heap()[1]
    if big is not None and big < TLS_MIN_KB:
        # TLS cannot get its block: every try would time out or fail to
        # allocate (seen as "err net" until a reboot on 2026-10-30)
        up["err"] += 1
        up["state"] = "low idf"
        if up.get("last_err") != "low idf":
            up["last_err"] = "low idf"
            _diag_write("upload low idf %d KB" % big)
        t0, seen = up.get("lowidf_at"), up.get("lowidf_seen")
        if t0 is None or seen is None or _age(now, seen) > LOWIDF_WAIT:
            t0 = up["lowidf_at"] = now   # a new episode (checks had a gap)
        up["lowidf_seen"] = now
        if not _lowidf_reboot(big, _age(now, t0)):
            if at_home():
                _schedule(now, LOWIDF_RETRY)
            else:
                _schedule(now)
        return
    up["lowidf_at"] = None
    pf = py_heap()[0]
    if pf < TLS_MIN_PY_KB * 1024:
        # an allocation that does not fit after a GC grows the heap by up
        # to its own size, out of the IDF block TLS needs: wait instead
        # (a fixed retry: the doubling backoff waited up to BACKOFF_MAX
        # for a heap that is back after the next collect)
        up["err"] += 1
        up["state"] = "low mem"
        if up.get("last_err") != "low mem":
            up["last_err"] = "low mem"
            _diag_write("upload low mem %d B" % pf)
        _schedule(now, LOWIDF_RETRY)
        return
    if up["url"].startswith("https"):
        _rtc_sync(now)                   # certificate dates need the time
        if not _tls_ready(now):
            return
    if not _bad_full_left():
        up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
    if CPU_MHZ_UPLOAD and CPU_MHZ and CPU_MHZ_UPLOAD != CPU_MHZ:
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
        if CPU_MHZ_UPLOAD and CPU_MHZ and CPU_MHZ_UPLOAD != CPU_MHZ:
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
        try:
            commit_offset(path, n, new_off, eof)
        except OSError:
            _fs_err(now)
            return
        up["sent"] += n_ok
        up["fail4"] = 0
        up["backoff"] = BACKOFF_START
        up["allow_bad"] = True
        up["skip"] = None
        held = up.get("hold") or {}
        held.pop(path, None)
        up["hold"] = held
        _clear_bad_full(path)
        if not _bad_full_left():
            up["state"] = "ok"
        up["next_at"] = time.ticks_add(now, gap)
    elif code in PAYLOAD_ERR:
        up["err"] += 1
        if from_bad:
            # count the rejections of a .bad, not its successes: it used to
            # stop after MAX_BAD_RETRY good batches until the next boot.
            # Not in fail4: rejected .bad batches kept a normal track's
            # bad batch from quarantine until the next 2xx
            bt = up.get("bad_try") or {}
            bt[path] = bt.get(path, 0) + 1
            up["bad_try"] = bt
        else:
            up["fail4"] = up.get("fail4", 0) + 1
        if path.endswith(".bad") or up["fail4"] >= MAX_GLOBAL_PAYLOAD:
            up["state"] = "err payload"
            _hold_file(path, now, retry if retry else BACKOFF_MAX)
            _schedule(now, gap)
            return
        if not _quarantine_batch(path, off, n, new_off, eof):
            _hold_file(path, now, BACKOFF_MAX)
            _schedule(now, gap)
            return
        up["next_at"] = time.ticks_add(now, gap)
    elif code == 401:
        up["err"] += 1
        up["state"] = "err 401"
        _schedule(now, retry if retry else AUTH_BACKOFF)
    else:
        up["err"] += 1
        up["state"] = "err " + ("net" if code is None else str(code))
        _schedule(now, retry)

def _fs_err(now):
    """A .pos write (or delete) failed: retry later, the track stays."""
    up["err"] += 1
    up["state"] = "fs err"
    _schedule(now)

def _low_mem(now):
    gc.collect()
    up["err"] += 1
    up["state"] = "low mem"
    _schedule(now)
    print("upload MemoryError, free", py_heap()[0])

# --- Status page ------------------------------------------------------------
# A small HTTP page on the LAN while the radio is connected (at home, or
# everywhere with WEB_AWAY, it stays connected for it, see wifi_tick):
# http://<ip>/ and /status.json.
# Served from the main loop, one client at a time: each recv/send waits at
# most WEB_TIMEOUT s and the whole request at most WEB_DEADLINE s (a slow
# client used to hold the loop for ~50 s: 2 s per 1 KB send).

_web_sock = None

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

def _web_open():
    global _web_sock
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(socket.getaddrinfo("0.0.0.0", WEB_PORT)[0][-1])
        s.listen(2)
        s.setblocking(False)
        _web_sock = s
        print("web http://%s/" % wifi.get("ip"))
    except OSError as e:
        print("web", repr(e))
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

def _web_close():
    global _web_sock
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
    view = sv_seen()
    t = _fix_secs()
    pos = None
    if st["valid"] and st["lat"] is not None and st["lon"] is not None:
        pos = "%.5f, %.5f" % (st["lat"], st["lon"])
    try:
        cause = machine.reset_cause()
    except Exception:
        cause = None
    flash = _flash_kb()
    files, bad = _file_counts()
    heap = py_heap()
    still = log.get("still") or {}
    return [
        ("fix", status()),
        ("sats", "%d / %s" % (view, sv_used())),
        ("hdop", st["hdop"]),
        ("hacc", None if st.get("hacc") is None or st.get("lat_i") is None
         else round(st["hacc"], 1)),
        ("fix_src", st.get("fix_src")),
        ("utc", None if t is None else _stamp_at(t)[0]),
        ("position", pos),
        ("nmea", "%d / %d" % (st["ok"], st["err"])),
        ("ubx", "%d / %d" % (st.get("ubx_ok", 0), st.get("ubx_bad", 0))),
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
        ("stay", "yes" if still.get("active") else "no"),
        ("uptime", uptime_s()),
        ("prev_uptime", prev_run["up"]),
        ("prev_run", _prev_text()),
        ("boot_cause", cause),
        ("temp", _temp()),
        ("mhz", machine.freq() // 1000000),
        ("mem", heap[0]),
        ("heap_kb", heap[1]),
        ("idf", "%s / %s" % idf_heap()),
        ("flash_kb", flash),
        ("wifi", "%s %s" % (wifi["state"], wifi.get("ssid") or "")),
        ("rssi", wifi.get("rssi")),
        ("pulses", wifi["pb_scans"]),
    ]

_fl = [None, None]                       # (KB, ticks) of the last statvfs

def _flash_kb():
    """Free flash in KB, at most every 30 s: statvfs walks the whole
    LittleFS (slow, and the page asks every WEB_REFRESH)."""
    now = time.ticks_ms()
    if _fl[1] is not None and _age(now, _fl[1]) < 30000:
        return _fl[0]
    try:
        v = os.statvfs("/")
        _fl[0] = v[0] * v[3] // 1024
    except OSError:
        _fl[0] = None
    _fl[1] = now
    return _fl[0]

_fc = [0, 0, None]                       # (files, bad files, ticks)

def _file_counts():
    """(track files waiting, .bad files) for the page, counted at most every
    10 s: pending() and pending_bad() list, sort and build the paths of the
    whole log directory, twice per poll. ilistdir counts without a list."""
    now = time.ticks_ms()
    if _fc[2] is not None and _age(now, _fc[2]) < 10000:
        return _fc[0], _fc[1]
    n = bad = 0
    cur = log["file"]
    try:
        if hasattr(os, "ilistdir"):
            names = (e[0] for e in os.ilistdir(LOG_DIR))
        else:                            # CPython (tests)
            names = iter(os.listdir(LOG_DIR))
        for name in names:
            if name.endswith((".gpx", ".trk")):
                if cur is None or cur != LOG_DIR + "/" + name:
                    n += 1
            elif name.endswith((".gpx.bad", ".trk.bad")):
                bad += 1
    except OSError:
        pass
    _fc[0], _fc[1], _fc[2] = n, bad, now
    return n, bad

def _sv_fresh():
    """Before a page: a satellite list older than SV_FRESH is asked for
    again; the answer is read (with the NMEA around it) for up to SV_WAIT
    ms. Silence leaves the older list; after SV_MISS unanswered polls in a
    row the page asks but does not wait (a receiver that drops the answer
    would cost every page view SV_WAIT)."""
    now = time.ticks_ms()
    at = st["svs_at"]
    if at is not None and _age(now, at) < SV_FRESH:
        return
    sv_poll(now)
    if st["sv_miss"] >= SV_MISS:
        return
    while _age(time.ticks_ms(), now) < SV_WAIT:
        read_uart()
        if st["svs_at"] is not None and st["svs_at"] != at:
            return
        time.sleep_ms(20)

def _sv_name(v):
    return "%s %d" % (GNSS.get(v[6], "SV"), v[0])

def _sv_list():
    """The satellite list for the page, used first, then by signal."""
    if st["svs"] is None:
        return []
    return sorted(st["svs"], key=lambda v: (not v[4], -v[1], v[6] or 0,
                                            v[0]))

def _sv_age():
    at = st["svs_at"]
    return None if at is None else _age(time.ticks_ms(), at) // 1000

def _web_json_parts():
    """The JSON in small pieces (no 8 KB string and its encoded copy)."""
    yield '{"device":' + json.dumps(DEVICE_ID) + ',"refresh_s":%d' % (
        WEB_REFRESH)
    yield ',"diag":['
    sep = ""
    for line in _web_diag():             # one piece a line (no ~3 KB string)
        yield sep + json.dumps(line)
        sep = ","
    yield '],"satellites":['
    sep = ""
    for v in _sv_list():
        yield sep + json.dumps({
            "name": _sv_name(v), "system": GNSS.get(v[6]), "svid": v[0],
            "cno": v[1], "elev": v[2], "azim": v[3], "used": v[4],
            "unhealthy": v[5]})
        sep = ","
    yield '],"satellites_age_s":' + json.dumps(_sv_age())
    for key, v in _web_rows():
        yield ",%s:%s" % (json.dumps(key), json.dumps(v))
    yield "}"

def _web_json():
    return "".join(_web_json_parts())

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

def _web_read(c):
    """The request head: recv until the blank line, at most WEB_REQ_MAX
    bytes. One recv(512) left the rest of a long request (cookies) unread,
    and closing a socket with unread data sends an RST that cut the reply
    short."""
    data = b""
    while len(data) < WEB_REQ_MAX and b"\r\n\r\n" not in data:
        _web_left(c)
        b = c.recv(512)
        if not b:
            break
        data += b
    return data

def _web_host_ok(req):
    """DNS rebinding guard: a page of another site that got its name
    pointed at the board sends that name as Host. Allowed: no Host, the
    board's IP, HOSTNAME or HOSTNAME.local (any case, with a port)."""
    i = req.find(b"\r\nHost:")
    if i < 0:
        i = req.find(b"\r\nhost:")
    if i < 0:
        return True
    j = req.find(b"\r\n", i + 7)
    h = req[i + 7:j if j >= 0 else len(req)].strip()
    k = h.rfind(b":")
    if k >= 0 and b"]" not in h[k:]:
        h = h[:k]
    try:
        h = h.decode().lower()
    except UnicodeError:
        return False
    if h.endswith("."):
        h = h[:-1]
    name = HOSTNAME.lower()
    return h in (wifi.get("ip") or "", name, name + ".local")

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
    curl only with --compressed)."""
    r = req.lower()
    i = r.find(b"\r\naccept-encoding:")
    if i < 0:
        return False
    j = r.find(b"\r\n", i + 2)
    return b"gzip" in r[i:j if j >= 0 else len(r)]

def _web_page(c, gz=False):
    """WEB_PAGE from flash in WEB_CHUNK reads into one buffer: its gzip
    (WEB_PAGE_GZ, ~9 KB instead of ~24 KB: a third of the sends) when the
    client accepts gzip and deploy.sh put it there, else the plain page.
    The page itself polls /status.json and redraws its boxes."""
    path, f = WEB_PAGE_GZ, None
    if gz:
        try:
            f = open(path, "rb")
        except OSError:
            pass
    if f is None:
        path = WEB_PAGE
        try:
            f = open(path, "rb")
        except OSError:
            _web_send(c, "404 Not Found", "text/plain",
                      ("no %s on the board: run deploy.sh; data at "
                       "/status.json" % WEB_PAGE,))
            return
    try:
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

def _web_reply(c, req):
    """Only a GET is answered: an empty or cut-off request (a closed tab, a
    port scan) used to fall back to "/" and get the whole page."""
    if not req.startswith(b"GET "):
        return
    if not _web_host_ok(req):
        _web_send(c, "403 Forbidden", "text/plain",
                  ("open http://%s/ (or %s.local)" % (
                      wifi.get("ip") or "<board ip>", HOSTNAME),))
        return
    parts = req.split(b" ", 2)
    path = parts[1] if len(parts) > 1 else b"/"
    if path.startswith(b"/status.json"):
        _sv_fresh()
        _web_send(c, "200 OK", "application/json", _web_json_parts())
    elif path == b"/" or path.startswith(b"/?"):
        _web_page(c, _web_gzip(req))
    else:
        _web_send(c, "404 Not Found", "text/plain", ("not found",))

def web_tick():
    """Listen while connected, close otherwise; answer one waiting client
    within WEB_DEADLINE."""
    if not WEB or wifi["state"] != "connected":
        _web_close()
        return
    if _web_sock is None:
        _web_open()
        if _web_sock is None:
            return
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
    finally:
        _web_end[0] = None
        try:
            c.close()
        except OSError:
            pass
        gc.collect()

# --- Screen -----------------------------------------------------------------

def status():
    if st["rx"] is None or _age(time.ticks_ms(), st["rx"]) > NO_DATA:
        return "No data!"
    if not st["valid"]:
        return "Waiting fix"
    s = {2: "2D fix", 3: "3D fix"}.get(st["fixtype"], "Fix")
    return s + " DGPS" if st["quality"] == 2 else s

def _fmt(fmt, v):
    return fmt % v if v is not None else "--"

def _short(n):
    """Fit a count into the 4 chars left after "Points left "."""
    return str(n) if n < 10000 else "%dk" % (n // 1000)

def _wifi_line():
    if wifi["state"] == "connected" and wifi.get("ssid"):
        ssid = wifi["ssid"]
        if len(ssid) > 11:
            ssid = ssid[:11]
        return "WiFi " + ssid
    return "WiFi " + wifi["state"]

def _mode_line():
    if _bad_full_left() or up.get("state") == "bad full":
        return "Mode BAD FULL"
    if wifi["state"] == "connected":
        return "Mode LOG+UP"
    return "Mode LOG"

def _oled_ok(fn):
    global oled, oled_err
    if oled is None:
        return
    try:
        fn()
        oled_err = 0
    except OSError as e:
        oled_err += 1
        print("oled", repr(e))
        if oled_err >= OLED_MAX_ERR:
            oled = None

def _oled_banner(msg):
    def _do():
        oled.fill(0)
        oled.text(msg, 0, 28)
        oled.show()
    _oled_ok(_do)

def draw():
    def _do():
        oled.fill(0)
        oled.fill_rect(0, 0, 128, 8, 1)
        oled.text("GPS Logger", 0, 0, 0)
        oled.text("|/-\\"[st["spin"] % 4], 120, 0, 0)
        for row, line in enumerate((
                _wifi_line(),
                "GPS " + status(),
                _mode_line(),
                "Flash " + _fmt("%d kB", _flash_kb()),   # cached 30 s
                "Points left " + _short(up["left"]))):
            oled.text(line, 0, 10 + row * 11)
        oled.show()
    _oled_ok(_do)

# --- Main loop --------------------------------------------------------------

def run():
    """Show the logger state on the OLED, logging and uploading when possible."""
    global _wdt, _running, _boot_ok_done
    _keep_stop()
    _running = False
    _boot_ok_done = False
    try:
        # Arm the dog before any blocking init (count_points, WiFi scan).
        # GPIO25 boot-bypass never calls run(), so the REPL stays free.
        if _wdt is None:
            _wdt = WDT(timeout=WDT_TIMEOUT)
        feed()
        reset()
        log_reset()
        wifi_reset()
        up_reset()
        env = load_env(ENV_FILE)
        _gps_model_env(env)              # before the first setup
        _gps_pvt_env(env)
        _min_move_env(env)
        _diag_env(env)
        env = None
        _diag_boot()
        gps_setup()
        wifi_start()
        _running = True
        _loop()
    except KeyboardInterrupt:
        _running = False
        print("WDT armed - fed by timer; hold the boot-bypass pin and reset for mpremote")
        try:
            flush(_still_hold())  # held drift is dropped
        except Exception:
            pass
        wifi_stop()
        _wdt_keepalive()
        raise
    except Exception as e:
        _running = False
        if hasattr(sys, "print_exception"):
            sys.print_exception(e)
        else:
            print("crash", e)
        _diag_write("crash up %d %s" % (uptime_s(), repr(e)))
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

def _loop():
    global _wdt
    if _wdt is None:
        _wdt = WDT(timeout=WDT_TIMEOUT)
    loop_start = time.ticks_ms()
    last = None
    status_at = loop_start
    diag_at = loop_start
    while True:
        feed()
        uptime_s()                       # sums ticks before they wrap
        read_uart()
        gps_check(time.ticks_ms())
        log_tick()
        wifi_tick()
        keepalive_tick(time.ticks_ms())
        feed()
        upload_tick()
        web_tick()
        now = time.ticks_ms()
        if (not _boot_ok_done
                and time.ticks_diff(now, loop_start) >= BOOT_OK_MS):
            _boot_ok()
        if last is None or time.ticks_diff(now, last) >= REFRESH:
            draw()
            last = now
        if time.ticks_diff(now, status_at) >= STATUS:
            _status()
            status_at = now
        if DIAG and time.ticks_diff(now, diag_at) >= DIAG:
            _diag(now)
            diag_at = now
        time.sleep_ms(20)
