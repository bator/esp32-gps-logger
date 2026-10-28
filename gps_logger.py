from machine import Pin, I2C, UART, WDT
import machine, sys, time, os, network, gc, math, socket, json
try:
    import esp32                         # mcu_temperature() on C3/C6/S2/S3
except ImportError:
    esp32 = None

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
GPS_MAX_STD = 20                         # m: receiver's own error estimate
                                         # (GST) above this = not usable
GPS_RETRY = 5000                         # ms: setup sent again if 1 Hz
GPS_RETRY_MAX = 300000                   # sentences still come this long after
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
WEB = True                               # status page on port WEB_PORT; the
WEB_PORT = 80                            # radio stays on at home for it
WEB_TIMEOUT = 2                          # s a client may take (.env WEB=0)
WEB_REFRESH = 15                         # s between page reloads
WEB_DIAG = 15                            # diag.log lines on the page
SV_POLL = 60000                          # ms between UBX-NAV-SVINFO polls (the
SV_FRESH = 5000                          # satellite list); a page view asks
SV_WAIT = 700                            # again if older than SV_FRESH and
                                         # waits SV_WAIT ms for the answer
GPS_LOOP_WINDOW = 10000                  # ms: start-up banners and no NMEA
GPS_LOOP_BANNERS = 3                     # in this window = a reset loop
GPS_FLAP_RESTARTS = 5                    # full restarts (NMEA in between)
GPS_FLAP_WINDOW = 600000                 # within this many ms = a reset loop

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
MIN_MOVE = 3                             # m from the last recorded point
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
STILL_DEP_MAX = 90                       # recent samples kept for the departure
STILL_MEAN_N = 1800                      # samples the anchor mean averages over
STALE = 2000                             # ms: older position = no fix
LOG_DIR = "/gpx"
MIN_FREE = 32768                         # bytes of flash kept free
MAX_BATCH = 240                          # RAM cap while flash writes fail
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
WIFI_OFF_WAIT = 60000                    # ms: an upload backoff at least
                                         # this long turns the radio off
                                         # until it is due (server down, 401)
WIFI_WAKE_WAIT = 60000                   # ms a new stay waits for the WiFi
                                         # round before its first anchor point
CONNECT_TIMEOUT = 12000                  # ms to wait for GOT_IP per network
FAIL_HOLD = 120000                       # ms to demote a network that failed
IGNORE_STALE_MS = 1500                   # leftover fail reason after connect()
CLEAR_WAIT = 500                         # ms to wait for STAT_IDLE after clear
LOST_GRACE = 5000                        # ms to let IDF auto-reconnect after drop
ROAM_CHECK = 300000                      # ms between home-roam scans on hotspot
ROAM_BACKOFF_MAX = 3600000               # ms cap on roam retry (60 min)
ROAM_MIN_RSSI = -75                      # do not jump to a weak home AP
LINE_MAX = 512                           # bounded track-line readline
BOOT_OK_MS = 60000                       # clear boot-fail after this of _loop

UPLOAD_BATCH = 50                        # points per POST to Dawarich
BODY_CAP = UPLOAD_BATCH * 256 + 32       # JSON buffer; sized for long device_id
UPLOAD_GAP = 5000                        # ms between successful batches
UPLOAD_GAP_AWAY = 30000                  # ms between batches on a non-home net
BACKOFF_START = 15000                    # ms first retry after net/5xx/4xx
BACKOFF_MAX = 1800000                    # ms cap (30 min) on upload backoff
AUTH_BACKOFF = 1800000                   # ms retry after 401 (plan/key/limit)
HTTP_TIMEOUT = 15                        # s per socket op, not a wall clock
DEFAULT_DEVICE_ID = "Dawarich-GPS-tracker"
DEFAULT_HOSTNAME = "Dawarich-GPS-tracker"
DEVICE_ID = DEFAULT_DEVICE_ID            # Overland tracker id (overridable)
HOSTNAME = DEFAULT_HOSTNAME              # DHCP hostname (overridable)
ID_MAX = 32                              # DEVICE_ID / HOSTNAME length cap
MAX_BAD_BYTES = 65536                    # cap on appended <file>.bad
MAX_BAD_RETRY = 8                        # successful .bad batches per file per boot
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
              svs=None, svs_at=None, sv_poll_at=None)
    _ep_new(None)

def _ep_new(t):
    # g / r: GGA / RMC positions as [ref lat, ref lon, sum dlat, sum dlon, n]
    # (offsets from the second's first fix: single-precision floats cannot
    # add up latitudes to the metre); alt / hdop: [sum, n]
    ep.clear()
    ep.update(t=t, date=None, g=None, r=None, alt=[0.0, 0], hdop=[0.0, 0],
              std=None, spd=None)

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
    t = "%s:%s:%s" % (s[0:2], s[2:4], s[4:6])
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
    a = ep["g"] or ep["r"]
    st["valid"] = a is not None
    st["epochs"] = a[4] if a else 0
    if a is None:
        return
    n = a[4]
    spd = ep["spd"]
    if n > 1 and spd is not None and spd > AVG_MAX_SPEED:
        # moving: the mean of the second lies ~0.4 s along the track but is
        # stamped with the second, so the point is the second's first fix
        st["lat"], st["lon"] = a[0], a[1]
    else:
        st["lat"], st["lon"] = a[0] + a[2] / n, a[1] + a[3] / n
    alt, hd = ep["alt"], ep["hdop"]
    st["alt"] = alt[0] / alt[1] if alt[1] else None
    now = time.ticks_ms()
    st["hdop"], st["hdop_at"] = (hd[0] / hd[1] if hd[1] else None), now
    st["std"], st["std_at"] = ep["std"], now
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
         "pos_at", "time", "date", "valid", "epochs")

def _snap():
    return tuple(st[k] for k in _SNAP)

def _load(snap):
    for i in range(len(_SNAP)):
        st[_SNAP[i]] = snap[i]

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
    st["fixtype"] = _num(f[2], int)
    if len(f) > 15:
        st["pdop"] = _num(f[15])

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
    sl, sn = _num(f[6]), _num(f[7])
    if sl is not None and sn is not None:
        ep["std"] = sl if sl > sn else sn

def txt(talker, f):
    """A u-blox prints its banner at start-up: the receiver restarted (power
    dip, reset) and is back at its defaults until gps_check sets it up."""
    if "u-blox" in ",".join(f[4:]):
        st["gps_boots"] += 1
        st["extra"] += 1                 # back at its defaults: set it up
        st["setup_wait"] = GPS_RETRY     # again within GPS_RETRY, not 5 min
        print("gps restarted")
        _gps_flap(time.ticks_ms())

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
        print("gps reset loop?", len(t), "restarts in",
              _age(now, t[0]) // 1000, "s: check the GPS power")
        del t[:]

HANDLERS = {"RMC": rmc, "GGA": gga, "GSA": gsa, "GSV": gsv, "GST": gst,
            "GLL": _extra, "VTG": _extra, "TXT": txt}

def _ubx(cls, mid, payload):
    """One UBX frame: sync, class, id, length, payload, Fletcher checksum."""
    m = bytes((cls, mid, len(payload) & 255, len(payload) >> 8)) + payload
    a = b = 0
    for x in m:
        a = (a + x) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + m + bytes((a, b))

def sv_poll(now):
    """Ask for UBX-NAV-SVINFO (the satellite list). No GSV at 9600 baud
    (see GPS_MSGS): one ~210-byte answer a minute, or per page view."""
    st["sv_poll_at"] = now
    try:
        uart.write(b"\xb5\x62\x01\x30\x00\x00\x31\x94")
    except OSError:
        pass

def _ubx_frame(fr):
    """A whole UBX frame read from the stream (read_uart). NAV-SVINFO fills
    st["svs"]: (svid, cno dBHz, elevation, azimuth, used, unhealthy) per
    channel; svid 1-32 GPS, 120-158 SBAS. Others (late ACKs) are dropped."""
    a = b = 0
    for x in fr[2:-2]:
        a = (a + x) & 255
        b = (b + a) & 255
    if a != fr[-2] or b != fr[-1]:
        st["err"] += 1
        return
    if fr[2] != 0x01 or fr[3] != 0x30 or len(fr) < 16:
        return
    n = fr[10]
    svs = []
    for k in range(n):
        o = 14 + 12 * k
        if o + 12 > len(fr) - 2:
            break
        el = fr[o + 5] - 256 if fr[o + 5] > 127 else fr[o + 5]
        svs.append((fr[o + 1], fr[o + 4], el, fr[o + 6] | fr[o + 7] << 8,
                    bool(fr[o + 2] & 1), bool(fr[o + 2] & 16)))
    st["svs"], st["svs_at"] = svs, time.ticks_ms()

def sv_seen():
    """Satellites with a signal: from the last NAV-SVINFO, else the GSV
    count (before the setup turns GSV off)."""
    if st["svs"] is not None:
        return len([v for v in st["svs"] if v[1] > 0])
    n = 0
    for c in st["view"].values():
        n += c
    return n

# NMEA message ids (class 0xF0) and their rate per fix on this port. At
# 9600 baud GPS_HZ fixes fit only with fewer sentences: RMC and GGA every
# fix (~145 B), GSA and GST once a second, no GSV / GLL / VTG (~830 B/s of
# 960).
GPS_MSGS = ((0x03, 0), (0x01, 0), (0x05, 0), (0x02, GPS_HZ),
            (0x07, GPS_HZ), (0x00, 1), (0x04, 1))

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

def _ubx_ack(frame):
    """Send a UBX-CFG frame until the receiver acknowledges it (UBX-ACK-ACK
    for its class and id), at most GPS_ACK_TRIES times: True, None for a
    UBX-ACK-NAK (it answers, but refuses the frame), False for silence. An overrun receiver
    sends the ACK seconds late, so one that comes during a later try counts
    too; what was read before the first send (an older ACK for the same
    id) does not. The NMEA read meanwhile is dropped (a few seconds)."""
    want = b"\xb5\x62\x05\x01\x02\x00" + frame[2:4]
    nak = b"\xb5\x62\x05\x00\x02\x00" + frame[2:4]
    t0 = time.ticks_ms()
    while uart.any() and time.ticks_diff(time.ticks_ms(), t0) < 200:
        uart.read(uart.any())            # a streaming receiver never empties
    raw = b""
    for _ in range(GPS_ACK_TRIES):
        uart.write(frame)
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < GPS_ACK_WAIT:
            n = uart.any()
            if n:
                raw = raw[-16:] + uart.read(n)
                if want in raw:
                    return True
                if nak in raw:
                    return None          # answered, but refused
            time.sleep_ms(5)
        feed()
    return False

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
    second's average is that one fix."""
    st["setup_at"], st["extra"] = time.ticks_ms(), 0
    ok = False
    msgs = [_ubx(0x06, 0x01, bytes((0xF0, mid, rate)))
            for mid, rate in GPS_MSGS]
    try:
        # GPS_HZ only once every message frame was acknowledged: GPS_HZ
        # with a sentence still on is the overrun state. Once a frame goes
        # unanswered the other message frames are sent without waiting (a
        # receiver that stops answering blocks the loop ~4 s, not ~4 s per
        # frame) and the receiver stays at 1 Hz until gps_check retries.
        if _ubx_ack(_rate(1000)):
            # NAV5 is a filter tuning, not part of the 5 Hz setup: a receiver
            # that refuses it (another model, a NAK) still gets the rest
            r = _ubx_ack(_nav5())
            st["nav5_ok"] = r is True
            ok = r is not False          # silent: it stopped answering
            if r is None:
                print("gps nav5 refused")
            for fr in msgs:
                if ok:
                    ok = _ubx_ack(fr) is True
                else:
                    uart.write(fr)
                    time.sleep_ms(30)
            if ok:
                ok = _ubx_ack(_rate(1000 // GPS_HZ)) is True
        else:
            # no answer: an overrun receiver ignores single frames but takes
            # a burst (measured): the message frames at once cut its output
            # (no rate change); gps_check then sets it up acknowledged
            for fr in msgs:
                uart.write(fr)
                time.sleep_ms(30)
    except Exception as e:
        print("gps setup", repr(e))
    # sentences sent while the setup ran are not a receiver that ignored it
    st["setup_at"], st["extra"] = time.ticks_ms(), 0
    st["setup_ok"] = ok
    if ok:
        st["setup_wait"] = GPS_RETRY     # it listens: a later revert is quick
        st["view"] = {}                  # GSV is off now: no frozen count
    print("gps setup", "ok" if ok else "no ack")

def gps_check(now):
    """Send the setup again while the receiver still sends the sentences it
    turns off: a receiver powered up with the board is still booting when
    run() sends it, and stays at 1 Hz. Less and less often, so a receiver
    that never takes it (or an unwired ESP TX) costs next to nothing."""
    _gps_loop_check(now)
    if (st["setup_ok"] and (st["sv_poll_at"] is None
                            or _age(now, st["sv_poll_at"]) >= SV_POLL)):
        sv_poll(now)
    at = st["setup_at"]
    if at is None or not st["extra"] or _age(now, at) < st["setup_wait"]:
        return
    st["setup_wait"] = min(st["setup_wait"] * 2, GPS_RETRY_MAX)
    print("gps setup again")
    gps_setup()

def _gps_loop_check(now):
    """A receiver in a reset loop (supply dips, a bad module) sends only the
    start of its banner over and over, cut off before the newline, and no
    NMEA (seen on the board: "$GPTXT,01,01,02,u" + 0xFF junk, nothing else,
    no UBX answer). Every GPS_LOOP_WINDOW: GPS_LOOP_BANNERS or more banner
    starts and not one valid sentence -> a console warning, counted."""
    if _age(now, st["loop_at"]) < GPS_LOOP_WINDOW:
        return
    b = st["banners"] - st["loop_b"]
    ok = st["ok"] - st["loop_ok"]
    if b >= GPS_LOOP_BANNERS and ok == 0:
        st["loops"] += 1
        print("gps reset loop?", b, "banners, no NMEA in",
              GPS_LOOP_WINDOW // 1000, "s: check the GPS power")
    st.update(loop_at=now, loop_b=st["banners"], loop_ok=st["ok"])

def _temp():
    try:
        return esp32.mcu_temperature()
    except Exception:
        return None

_freq_bad = []                           # clocks this board refused

def _set_freq(mhz):
    """CPU clock: 80 MHz saves ~15-25 mA on the ESP32 against 240 (the UART
    runs off the fixed APB clock, WiFi needs 80 or more). Unsupported on
    this board or value: left alone, with a console line once."""
    if not mhz or mhz in _freq_bad:
        return
    try:
        if machine.freq() != mhz * 1000000:
            machine.freq(mhz * 1000000)
    except Exception as e:
        _freq_bad.append(mhz)
        print("cpu freq", mhz, repr(e))

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
          "mem", gc.mem_free(), "gpsloops", st["loops"])

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
    """The last n lines of a diag file (reads only its tail), [] if none."""
    try:
        size = _file_size(path) or 0
        with open(path) as f:
            if size > 160 * n:
                f.seek(size - 160 * n)
                f.readline()             # a partial line
            lines = f.read().splitlines()
        return lines[-n:]
    except (OSError, ValueError):
        return []

# How long the run before this boot lasted, from the last diag lines it
# wrote: up (s, to within DIAG; 0 = it ended before its first status line),
# end (GPS time of its last line, "2026-10-28 12:19:42Z") and how
# ("status", "crash" or "short"). Read once at boot.
prev_run = {"up": None, "end": None, "how": None}

def _diag_prev():
    prev_run.update(up=None, end=None, how=None)
    lines = _diag_tail(DIAG_FILE, 8) or _diag_tail(DIAG_OLD, 8)
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
    """First line after a boot: why the chip (re)started (1 power-on, also a
    power bank switching back on; 3 WDT; 4 deep sleep; 5 soft or esptool
    reset; 6 crash: panic, brownout) and how long the run before lasted
    (prev_up s, to within DIAG; from that run's last lines, read first)."""
    _diag_prev()
    try:
        cause = machine.reset_cause()
    except Exception:
        cause = None
    _diag_write("boot cause %s prev_up %s mhz %d mem %d" % (
        cause, "-" if prev_run["up"] is None else prev_run["up"],
        machine.freq() // 1000000, gc.mem_free()))

def _diag(now):
    """One line every DIAG ms: uptime, GPS time, NMEA sentences good/bad,
    receiver restarts and reset-loop warnings, satellites seen/used, fix,
    HDOP, points logged/sent, WiFi state, keep-alive pulses, free heap."""
    view = sv_seen()
    _diag_write(
        "up %d utc %s nmea %d/%d gpsboots %d loops %d sats %d/%s fix %s "
        "hdop %s points %d sent %d wifi %s pulses %d mem %d" % (
            now // 1000, _diag_utc(), st["ok"], st["err"], st["gps_boots"],
            st["loops"], view, st["used"], status().replace(" ", "_"),
            st["hdop"], log["points"], up["sent"], wifi["state"],
            wifi["pb_scans"], gc.mem_free()))

def checksum_ok(line):
    star = line.rfind(b"*")
    if not line.startswith(b"$") or star < 0:
        return False
    c = 0
    for b in line[1:star]:
        c ^= b
    try:
        return c == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False

def handle(line):
    line = line.strip()
    i = line.find(b"$")
    if i > 0:
        line = line[i:]                  # a binary UBX reply ran into it
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
        st["ok"] += 1
    except (ValueError, IndexError, UnicodeError):
        st["err"] += 1

_buf = b""
_chunk = bytearray(256)                  # read in small pieces: one big
                                         # allocation grows the heap (see
                                         # Upload) and starves TLS

def read_uart():
    global _buf
    while uart.any():
        k = uart.readinto(_chunk)
        if not k:
            return
        st["rx"] = time.ticks_ms()
        st["spin"] += 1
        piece = _chunk[:k]
        if piece.find(b"TXT,01,01,02,u") >= 0:
            st["banners"] += 1           # a start-up banner (or its start)
        _buf += piece
        while True:
            i = _buf.find(b"\n")
            u = _buf.find(b"\xb5\x62")      # never in NMEA (ASCII)
            if u >= 0 and (i < 0 or u < i):
                # a binary UBX answer: whole, by its length (it may hold
                # 0x0A bytes); what came before it is a cut-off line
                if len(_buf) < u + 6:
                    break
                need = u + 8 + (_buf[u + 4] | _buf[u + 5] << 8)
                if need - u > 1024:      # not a frame: skip the sync bytes
                    _buf = _buf[u + 2:]
                    continue
                if len(_buf) < need:
                    break
                fr, _buf = _buf[u:need], _buf[need:]
                _ubx_frame(fr)
                continue
            if i < 0:
                break
            line, _buf = _buf[:i], _buf[i + 1:]
            handle(line)
        u = _buf.find(b"\xb5\x62")
        if len(_buf) > (1100 if u >= 0 else 256):
            _buf = b""                   # no newline in sight: garbage/baud

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
# order after visible ones, so a missed scan does not block fallback.
# v1.29 connect() may leave a stale status reason: the first IGNORE_STALE_MS
# of a try are not treated as final.

def load_env(path):
    """KEY=VALUE lines -> dict. Surrounding quotes are stripped."""
    env = {}
    try:
        with open(path) as f:
            for line in f:
                if line[:1] == "\ufeff":
                    line = line[1:]          # UTF-8 BOM (Windows Notepad)
                line = line.strip()
                if not line or line[0] == "#" or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip()
                if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
                    v = v[1:-1]
                env[k] = v
    except OSError:
        pass                             # no .env on the device
    return env

def _plain(v):
    """Value of a non-secret key without a trailing "# comment".

    Only for keys whose values can never contain "#" (flags, IPv4
    addresses, names). SSIDs, passwords, GEO_URL and API_KEY are verbatim."""
    if v is None:
        return None
    i = v.find("#")
    return (v if i < 0 else v[:i]).strip()

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
        nets.append({"ssid": s, "pw": "" if p is None else p,
                     "home": home, "hidden": hidden, "bssid": None,
                     "ip": _static_ip(env, ipkeys)})
    return nets

WIFI_STATES = {}
for _n, _s in (("STAT_IDLE", "idle"), ("STAT_CONNECTING", "connecting"),
               ("STAT_GOT_IP", "connected"), ("STAT_NO_AP_FOUND", "no AP"),
               ("STAT_WRONG_PASSWORD", "auth fail"),
               ("STAT_BEACON_TIMEOUT", "lost"), ("STAT_ASSOC_FAIL", "assoc fail"),
               ("STAT_HANDSHAKE_TIMEOUT", "handshake")):
    if hasattr(network, _n):
        WIFI_STATES[getattr(network, _n)] = _s

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
                blank_rssi=None, blank_bssid=None)

def _set_hostname(name):
    if not name:
        return
    try:
        network.hostname(name)
        return
    except (AttributeError, OSError, TypeError, ValueError):
        pass
    w = wifi.get("wlan")
    if w is None:
        return
    try:
        w.config(dhcp_hostname=name)
    except (AttributeError, OSError, TypeError, ValueError):
        pass

def _recently_failed(ssid, now):
    t = wifi["failed"].get(ssid)
    return t is not None and _age(now, t) < FAIL_HOLD

def _mark_failed(ssid, why=""):
    wifi["failed"][ssid] = time.ticks_ms()
    print("wifi fail", ssid, why)
    if wifi.get("roaming") and _net_home(ssid):
        # Hidden home: keep 5 min. A neighbour's empty-SSID AP can trigger
        # a try; growing backoff would delay a real visible-home switch.
        if not _net_hidden(ssid):
            b = wifi.get("roam_backoff") or ROAM_CHECK
            wifi["roam_backoff"] = min(max(b * 2, ROAM_CHECK), ROAM_BACKOFF_MAX)
        wifi["roaming"] = False

def _scan_known(w):
    """Visible known nets, strongest BSSID per SSID, or None if scan fails.
    Hidden APs are listed with an empty SSID; the strongest blank RSSI is
    a hint that *some* hidden AP is nearby (not necessarily ours)."""
    by = {}
    for n in wifi["nets"]:
        by[n["ssid"]] = n
    try:
        aps = w.scan()
    except OSError:
        wifi["blank_rssi"] = None
        wifi["blank_bssid"] = None
        return None
    best = {}
    blank_rssi, blank_bssid = None, None
    for rec in aps:
        raw = rec[0]
        ssid = raw.decode() if isinstance(raw, bytes) else raw
        rssi = rec[3]
        bssid = rec[1] if len(rec) > 1 else None
        if not ssid:
            if blank_rssi is None or rssi > blank_rssi:
                blank_rssi, blank_bssid = rssi, bssid
            continue
        if ssid not in by:
            continue
        prev = best.get(ssid)
        if prev is None or rssi > prev[0]:
            best[ssid] = (rssi, bssid, by[ssid])
    wifi["blank_rssi"] = blank_rssi
    wifi["blank_bssid"] = blank_bssid
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
    if w is None or not any(n.get("home") for n in wifi.get("nets") or []):
        return None
    state = wifi["state"]
    if state == "connected" and wifi.get("home"):
        try:
            r = w.status("rssi")
        except (OSError, ValueError):
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
    except OSError:
        found = None
    finally:
        if off:
            try:
                w.active(False)
            except OSError:
                pass
    if found is None:
        return None
    for n in found:
        if n.get("home") and n.get("rssi", -100) >= ROAM_MIN_RSSI:
            return True
    return bool(_blank_home_ok())

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

def _blank_home_ok():
    """True if a hidden home is configured and a strong empty-SSID AP was seen."""
    r = wifi.get("blank_rssi")
    if r is None or r < ROAM_MIN_RSSI:
        return False
    for n in wifi.get("nets") or []:
        if n.get("home") and n.get("hidden"):
            return True
    return False

def _hidden_home_target(now, arrival=False):
    """Hidden home when a strong empty-SSID AP was seen.
    Always bssid=None so the IDF probes by SSID (never pin to the
    strongest blank AP — that may be a neighbour). A stop after driving
    must not treat an unverified blank AP as home."""
    if arrival:
        return None
    if not _blank_home_ok():
        return None
    for n in wifi.get("nets") or []:
        if (n.get("home") and n.get("hidden")
                and n["ssid"] != wifi.get("ssid")
                and not _recently_failed(n["ssid"], now)):
            target = dict(n)
            target["bssid"] = None
            return target
    return None

def _candidate_order(w):
    """Visible strongest first (deduped), then unscanned nets in config order."""
    now = time.ticks_ms()
    scanned = _scan_known(w)
    if scanned is None:
        cands = [dict(n) for n in wifi["nets"]]
    else:
        seen = set(n["ssid"] for n in scanned)
        cands = scanned + [dict(n) for n in wifi["nets"]
                           if n["ssid"] not in seen]
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
            elif n.get("home") and n.get("hidden") and rssi is None and blank:
                homes.append(n)
            else:
                rest.append(n)
        return homes + rest

    return _home_first(good) + _home_first(bad)

def _wifi_clear(w):
    """Drop the last association so the next status() is not a stale reason."""
    try:
        w.disconnect()
    except OSError:
        pass
    try:
        w.active(False)
        w.active(True)
        _set_hostname(HOSTNAME)
    except OSError:
        pass
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < CLEAR_WAIT:
        feed()
        try:
            s = w.status()
        except OSError:
            break
        if s == network.STAT_IDLE or s == network.STAT_CONNECTING:
            break
        time.sleep_ms(50)

def _wifi_ip(w, net):
    """Apply this network's static IP, or give a previous one back to DHCP.

    Only a network that configured a static IP changes the interface, so the
    DHCP networks behave exactly as before."""
    cfg = net.get("ip")
    try:
        if cfg:
            w.ifconfig(cfg)
            wifi["static"] = True
        elif wifi.get("static"):
            w.ifconfig("dhcp")
            wifi["static"] = False
    except (OSError, ValueError, TypeError) as e:
        print("wifi ip", repr(e))

def _wifi_connect(w, net):
    wifi["ssid"] = net["ssid"]
    wifi["pw"] = net["pw"]
    wifi["home"] = bool(net.get("home"))
    wifi["state"] = "connecting"
    wifi["try_at"] = time.ticks_ms()
    wifi["saw_connecting"] = False
    print("wifi try", net["ssid"])
    try:
        wifi["prev_status"] = w.status()
    except OSError:
        wifi["prev_status"] = None
    _wifi_clear(w)
    _wifi_ip(w, net)
    try:
        bssid = net.get("bssid")
        if bssid:
            try:
                w.connect(net["ssid"], net["pw"], bssid=bssid)
            except TypeError:
                w.connect(net["ssid"], net["pw"])
        else:
            w.connect(net["ssid"], net["pw"])
    except OSError as e:
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
    except OSError:
        wifi["ip"] = None
    try:
        wifi["rssi"] = w.status("rssi")
    except (OSError, ValueError):
        wifi["rssi"] = None
    wifi["home"] = _net_home(wifi.get("ssid"))
    wifi["sleep_home"] = False
    wifi["idle_at"] = time.ticks_ms()
    wifi["failed"].pop(wifi.get("ssid"), None)
    wifi["backoff"] = WIFI_RETRY
    wifi["roam_at"] = time.ticks_ms()
    wifi["roaming"] = False
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
    upload, sleep."""
    if _all_uploaded():
        _wifi_off(w, "sleep")
        return
    _radio_off(w)
    wifi["state"] = "not found"
    wifi["home"] = False
    wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)
    wifi["last_check"] = time.ticks_ms()

def _radio_off(w):
    try:
        w.disconnect()
    except OSError:
        pass
    try:
        w.active(False)
    except OSError:
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
    """ms until the next upload attempt is due, 0 when it is (or nothing
    waits): the time an upload error (server down, 401, 5xx) backed off."""
    if not str(up.get("state") or "").startswith("err"):
        return 0
    d = time.ticks_diff(up["next_at"], now)
    return d if 0 < d <= UP_WAIT_MAX else 0

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
    except OSError:
        pass
    wifi["cand"] = _candidate_order(w)
    wifi["cand_i"] = 0
    wifi["retried"] = False
    if not wifi["cand"]:
        _wifi_round_failed(w)
        return False
    return _wifi_connect(w, wifi["cand"][0])

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
    except OSError:
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

def _keepalive_env(env):
    """KEEPALIVE=<seconds> in .env: 0 turns the keep-alive off, otherwise
    5-600 s between pulses; anything else keeps the default. KEEPALIVE_MODE
    picks the pulse (scan / radio / gpio), KEEPALIVE_PIN the gpio."""
    global PB_KEEPALIVE, PB_MODE, PB_PIN
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
            PB_PIN = int(v)
        except ValueError:
            print("KEEPALIVE_PIN invalid, using", PB_PIN)

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
            w.scan()
    except OSError:
        pass
    finally:
        try:
            w.active(False)
        except OSError:
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
    if (not PB_KEEPALIVE or w is None
            or wifi["state"] not in ("sleep", "not found")):
        wifi["pb_at"] = now
        return
    if _age(now, wifi["pb_at"]) < PB_KEEPALIVE:
        return
    wifi["pb_at"] = now
    feed()
    if PB_MODE == "gpio":
        if not _pb_pulse_gpio():
            PB_MODE = "scan"             # no such pin: fall back for good
            print("keepalive mode scan")
            _pb_pulse_radio(w, True)
    else:
        _pb_pulse_radio(w, PB_MODE == "scan")
    wifi["pb_scans"] += 1
    feed()

def wifi_start():
    global DEVICE_ID, HOSTNAME, CPU_MHZ, CPU_MHZ_UPLOAD
    env = load_env(ENV_FILE)
    CPU_MHZ = _mhz_env(env, "CPU_MHZ", CPU_MHZ)
    CPU_MHZ_UPLOAD = _mhz_env(env, "CPU_MHZ_UP", CPU_MHZ_UPLOAD)
    _set_freq(CPU_MHZ)
    wifi["nets"] = _load_nets(env)
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
        print("WARNING: requests follows http redirects as GET; use https origin")
    if not wifi["nets"]:
        wifi["state"] = "no .env"
        return
    _oled_banner("WiFi scanning...")
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        _set_hostname(HOSTNAME)
        w.active(True)
    except OSError:
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
    except OSError:
        s = None
    if s == network.STAT_GOT_IP:
        if wifi["state"] != "connected":
            _wifi_got_ip(w)
        else:
            if _age(now, wifi["last_check"]) >= WIFI_CHECK:
                try:
                    wifi["rssi"] = w.status("rssi")
                except (OSError, ValueError):
                    wifi["rssi"] = None
                wifi["last_check"] = now
            interval = wifi.get("roam_backoff") or ROAM_CHECK
            if (not wifi.get("home")
                    and any(n.get("home") for n in wifi.get("nets") or [])):
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
                if WEB and wifi.get("home"):
                    pass                 # the status page keeps it on
                elif _all_uploaded():
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
        print("wifi lost", wifi.get("ssid") or "")
        return
    if wifi["state"] == "lost":
        if (s == network.STAT_CONNECTING or s == network.STAT_IDLE
                or s is None):
            if _age(now, wifi["try_at"] or now) < LOST_GRACE:
                return
        wifi["state"] = "offline"
        wifi["last_check"] = time.ticks_add(now, -wifi["backoff"])
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
            if _all_uploaded() and not WEB:
                wifi["wake_at"] = None
                return
        wifi["wake"] = False
        wifi["backoff"] = WIFI_RETRY
        wifi["last_check"] = now
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
    except OSError:
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
# "-" starts a new segment. The file is always complete: each batch
# overwrites the footer with new points plus the footer again. LittleFS
# commits on close, so a power cut leaves either the previous or the new
# version. A line cut short has no newline and is skipped on read.
# Older .gpx files (XML) are still read and uploaded.

HEADER = "#TRK1\n"
FOOTER = "#END\n"
NEW_SEG = "-\n"

log = {}

def log_reset():
    log.clear()
    log.update(file=None, created=False, batch=[], points=0, err=0,
               fix=False, lost_at=None, new_seg=False, seg_tc=None,
               full=False,
               still=_still_state(), clock=None, seq=0,
               last_sample=None, last_flush=time.ticks_ms(), last_t=None,
               prev=None, full_check=None, file_at=None, good_size=None)

def _still_state():
    return {"active": False, "home": False, "win_at": None, "win_tc": None,
            "ref": None, "sum": [0.0, 0.0, 0], "prev": None, "anchor": None,
            "aref": None, "aoff": (0.0, 0.0), "n": 0, "hb_at": None,
            "ring": [], "far_n": 0, "far_d": 0, "far_slow": 0, "far_0": 0,
            "last_at": None, "last_d": 0, "file_at": None, "veto_at": None,
            "move_c": None, "wins": 0, "hold_tc": None}

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
    if sl["prev"]:
        t = sl["prev"][1]
    elif sl["win_at"] is not None:
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
    log.update(file=LOG_DIR + "/" + name + ".trk", created=False, points=0,
               new_seg=False, seg_tc=None, file_at=time.ticks_ms(),
               good_size=None)
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
        while k < len(batch) and batch[k].rstrip()[-16:] < hold:
            k += 1
        # hold at most HOLD_MAX lines, so a drive never reaches the
        # MAX_BATCH cap that drops the oldest ones
        k = max(k, len(batch) - HOLD_MAX)
        if not k:
            return
    if path is None or (log["created"] and not k):
        return
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
    # A segment break goes before the first line recorded after it (seg_tc),
    # not before the whole batch: lines held or not yet flushed from before
    # the break stay in the old segment. Not at the very start of a file.
    seg = None
    if log["new_seg"] and log["seg_tc"] is not None:
        i = 0
        while i < k and batch[i].rstrip()[-16:] < log["seg_tc"]:
            i += 1
        if i < k and (i > 0 or not creating):
            seg = i
    try:
        extra = len(NEW_SEG) if seg is not None else 0
        pts = 0
        i = 0
        while i < k:
            pts += len(batch[i])
            i += 1
        if creating:
            expected = len(HEADER) + extra + pts + len(FOOTER)
            f = open(path, "wb")
            f.write(HEADER.encode())
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
        i = 0
        while i < k:
            if i == seg:
                f.write(NEW_SEG.encode())
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
    log["points"] += k
    up["left"] += k
    if log["seg_tc"] is not None and k and (
            batch[k - 1].rstrip()[-16:] >= log["seg_tc"]):
        log["new_seg"], log["seg_tc"] = False, None   # the break is written
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
    good HDOP (1.2-3) while the position walked in from 230 m away."""
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
    hdop = _fresh_hdop(now)
    lat, lon = st["lat"], st["lon"]
    t = st["date"] + "T" + st["time"] + "Z"
    if t == last_t or (prev is not None and _dist(
            prev[0], prev[1], lat, lon) <= MIN_MOVE):
        return None
    ele = ""
    if st["alt"] is not None and st.get("fixtype") != 2:
        ele = "%.1f" % st["alt"]
    hd = ("%.1f" % hdop) if hdop is not None else ""
    return t, "%.6f,%.6f,%s,%s,%s\n" % (lat, lon, ele, hd, _tc())

def _record(s, pos):
    """Append a (time, line) from _sample_line; pos is what it wrote."""
    if log["file"] is None:
        start_file()
    if log["new_seg"] and log["seg_tc"] is None:
        log["seg_tc"] = s[1].rstrip()[-16:]    # the new segment starts here
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

def _fix_secs():
    """UTC seconds of the current fix's date and time, or None."""
    d, t = st["date"], st["time"]
    try:
        return (_days(int(d[0:4]), int(d[5:7]), int(d[8:10])) * 86400
                + int(t[0:2]) * 3600 + int(t[3:5]) * 60 + int(t[6:8]))
    except (TypeError, ValueError, IndexError):
        return None

def _stamp_at(secs):
    """(ISO time, compact track time) for UTC seconds."""
    y, m, d = _civil(secs // 86400)
    r = secs % 86400
    v = (y, m, d, r // 3600, r // 60 % 60, r % 60)
    return ("%04d-%02d-%02dT%02d:%02d:%02dZ" % v,
            "%04d%02d%02dT%02d%02d%02dZ" % v)

def _still_window(now, pos):
    """Fold a usable fix into the current centroid window. When the window
    is full and its centroid lies within STILL_RADIUS of the previous one,
    return (anchor, stay start time, fixes); a walk moves the centroid on.
    Sums are offsets from the window's first fix: single-precision floats
    cannot add up 60 latitudes to the metre."""
    sl = log["still"]
    if sl["last_at"] is not None and _age(now, sl["last_at"]) > STILL_GAP:
        # After a gap the device may be elsewhere: an open window (or the
        # previous one) from before it must not prove a stay here. Beyond
        # STILL_EXIT both restart. Past STILL_RADIUS only the open window
        # does: indoors usable fixes come seconds apart and drift past it,
        # and dropping the previous window too released the drift held for
        # the stay (measured: 13 lines in 14 min).
        c = sl["prev"][0] if sl["prev"] else None
        if sl["win_at"] is not None and sl["sum"][2]:
            r, sm = sl["ref"], sl["sum"]
            c = (r[0] + sm[0] / sm[2], r[1] + sm[1] / sm[2])
        gd = None if c is None else _dist(c[0], c[1], pos[0], pos[1])
        if gd is not None and gd > STILL_EXIT:
            sl.update(win_at=None, prev=None, hold_tc=None)
        elif gd is not None and gd > STILL_RADIUS:
            # do not mix two places in one window, but keep holding what
            # the restarted window held: a pair that proves a stay drops it
            if sl["win_at"] is not None and sl["hold_tc"] is None:
                sl["hold_tc"] = sl["win_tc"]
            sl["win_at"] = None
    sl["last_at"] = now
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
    t, tc = _stamp_at(c[0] + age // 1000)
    if log["last_t"] is not None and t <= log["last_t"]:
        return
    a = sl["anchor"]
    sl["hb_at"] = now
    if log["file"] is None:
        start_file(tc[:8] + "_" + tc[9:15])
        sl["file_at"] = now
    _record((t, "%.6f,%.6f,,,%s\n" % (a[0], a[1], tc)), a)

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
                   seg_tc=None)
        return True
    hold = _still_hold()
    if hold is not None and b[0].rstrip()[-16:] >= hold:
        tc = b[0].rstrip()[-16:]
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
    sl.update(active=True, anchor=anchor, aref=anchor,
              n=min(n, STILL_MEAN_N))
    b = log["batch"]
    i = 0
    while i < len(b) and b[i].rstrip()[-16:] < since_tc:
        i += 1
    while i < len(b):
        f = b[i].split(",", 2)
        if _dist(anchor[0], anchor[1], float(f[0]), float(f[1])) <= STILL_NEAR:
            break
        i += 1
    del b[i:]
    _still_close()                       # the arrival uploads during the stay
    if at_home():
        sl["home"] = True
    elif wifi_wake():
        # the radio slept: the first anchor point waits for the round, so
        # a home stay is not stamped with one
        sl["hb_at"] = time.ticks_add(now, WIFI_WAKE_WAIT - STILL_HEARTBEAT)
    else:
        _still_heartbeat(now)
    print("stay", "home" if sl["home"] else "away", "kept", i)

def _still_leave():
    """End the stay and write the way out: the trailing run of recent
    samples whose distance from the anchor kept growing (within STILL_TOL),
    so the departure starts near the anchor, not STILL_EXIT away."""
    ring = log["still"]["ring"]
    i = len(ring) - 1
    while i > 0 and ring[i - 1][2] <= ring[i][2] + STILL_TOL:
        i -= 1
    prev = None
    for s, p, d in ring[i:]:
        # only what comes after the last line (an anchor point may have
        # been written while the way out began), so the track stays in order
        if s is None or (log["last_t"] is not None and s[0] <= log["last_t"]) or (
                prev is not None
                and _dist(prev[0], prev[1], p[0], p[1]) <= MIN_MOVE):
            continue
        if log["file"] is None:
            tc = s[1].rstrip()[-16:]
            start_file(tc[:8] + "_" + tc[9:15])
        _record(s, p)
        prev = p
    print("stay end", int(log["still"]["far_d"]))
    wifi["sleep_home"] = False           # left: no longer at home
    log["still"] = _still_state()

def _still_active(now, pos):
    """One usable fix while stationary: watch for a departure (consecutive
    samples beyond STILL_EXIT that keep receding; an excursion that turns
    back does not count), refine the anchor (the heartbeat, the file
    rotation and the home close are _still_beat)."""
    sl = log["still"]
    a = sl["anchor"]
    d = _dist(a[0], a[1], pos[0], pos[1])
    ring = sl["ring"]
    ring.append((_sample_line(now, None, None), pos, d))
    if len(ring) > STILL_DEP_MAX:
        ring.pop(0)
    if d > STILL_EXIT and d + STILL_TOL >= sl["far_d"]:
        # Leaving: STILL_EXIT_N samples each gaining STILL_STEP (a vehicle),
        # or STILL_EXIT_SLOW samples out there that gained STILL_SLOW_GAIN
        # in all (a walk). A multipath jump that freezes 200 m away gains
        # nothing, however long it lasts.
        sl["far_n"] = sl["far_n"] + 1 if d >= sl["far_d"] + STILL_STEP else 1
        if not sl["far_slow"]:
            sl["far_0"] = d
        sl["far_slow"] += 1
        sl["far_d"] = d
        if sl["far_n"] >= STILL_EXIT_N or (
                sl["far_slow"] >= STILL_EXIT_SLOW
                and d - sl["far_0"] >= STILL_SLOW_GAIN):
            if not (sl["home"] and _home_veto(now, sl)):
                _still_leave()
                return
            sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
    else:
        sl["far_n"], sl["far_d"], sl["far_slow"] = 0, 0, 0
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
    _still_beat(now)
    if not has_fix(now):
        if log["fix"]:
            _drain_q()                   # valid seconds just before the loss
        else:
            st["q"] = []                 # stale
        if log["fix"]:
            log["fix"], log["lost_at"] = False, now
            flush(_still_hold())
        elif log["batch"] and _age(now, log["last_flush"]) >= FLUSH:
            flush(_still_hold())         # anchor points written without a fix
        return
    if not log["fix"]:
        log["fix"] = True
        if log["still"]["active"]:
            pass                         # same place: the stay keeps its file
        elif (log["file"] is not None and log.get("lost_at") is not None
                and _age(now, log["lost_at"]) > NEW_FILE_GAP * 1000):
            closed = ((not log["batch"]
                       or _age(now, log["last_flush"]) >= FLUSH)
                      and _close_file())
            log["prev"] = None           # first point of the new trip is kept
            if not closed:
                log["new_seg"] = True    # write failed: new trip in same file
        elif log["file"] is not None:
            log["new_seg"] = True
            log["prev"] = None           # start the segment with this fix
    if st["seq"]:
        _drain_q()
    elif (log["last_sample"] is None     # (st set directly: by the clock)
          or _age(now, log["last_sample"]) >= SAMPLE):
        log["last_sample"] = now
        _sample(now)
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
    secs = _fix_secs()
    if secs is not None:
        log["clock"] = (secs, now)       # GPS time for the heartbeat
    if not _usable(now):
        return
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

def up_reset():
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, fail4=0, backoff=BACKOFF_START, allow_bad=True,
              bad_try={}, skip=None, hold={}, bad_full={},
              next_at=time.ticks_ms())
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

def _readline(f):
    """One track line, at most LINE_MAX bytes. Over-long lines are drained to
    the next newline and returned as b'' (skip). None means true EOF."""
    try:
        line = f.readline(LINE_MAX)
    except MemoryError:
        gc.collect()
        raise
    if not line:
        return None
    if len(line) == LINE_MAX and not line.endswith(b"\n"):
        while True:
            try:
                chunk = f.readline(LINE_MAX)
            except MemoryError:
                gc.collect()
                break
            if not chunk or chunk.endswith(b"\n"):
                break
        return b""
    return line

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
                if off:
                    f.seek(off)
                while True:
                    line = _readline(f)
                    if line is None:
                        break
                    nlines += 1
                    if nlines & 63 == 0:
                        feed()
                    if _is_point(line, gpx):
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
    """One point line: <trkpt> in old .gpx, "lat,lon,ele,hdop,time" in .trk.
    A line without its newline (power cut mid-write) is not a point."""
    if gpx:
        return line.startswith(b"<trkpt")
    return (line[-1:] == b"\n" and line.count(b",") == 4
            and line[:1] != b"#")

def _point(line, gpx):
    """(lat, lon, time, ele, hdop) as strings, time as ISO UTC, or None."""
    if gpx:
        s = line.decode()
        return (_attr(s, "lat"), _attr(s, "lon"), _tag(s, "time"),
                _tag(s, "ele"), _tag(s, "hdop"))
    f = line[:-1].decode().split(",")
    tc = f[4]
    if len(tc) != 16 or tc[8] != "T" or tc[15] != "Z":
        return None
    t = "%s-%s-%sT%s:%s:%sZ" % (tc[0:4], tc[4:6], tc[6:8],
                                tc[9:11], tc[11:13], tc[13:15])
    return (f[0], f[1], t, f[2], f[3])

def read_batch(path, off=0):
    """Fill _body with up to UPLOAD_BATCH valid points from byte offset off
    as Overland JSON -> (n, n_ok, body_len, new_off, eof).
    Truncated / non-numeric / inf / over-long points are skipped.
    A valid point that does not fit is left unread (seek back, eof=False)."""
    gpx = not _is_trk(path)
    n, n_ok, pos, sep = 0, 0, _put(0, b'{"locations":['), b""
    nlines, eof = 0, False
    with open(path, "rb") as f:
        if off:
            f.seek(off)
        while True:
            line_start = f.tell()
            line = _readline(f)
            if line is None:
                eof = True
                break
            nlines += 1
            if nlines & 63 == 0:
                feed()
            if not line or not _is_point(line, gpx):
                continue
            n += 1
            try:
                p = _point(line, gpx)
            except UnicodeError:
                p = None
            lat_f = lon_f = None
            if p is not None:
                lat_f, lon_f = _float(p[0]), _float(p[1])
            ok = (p is not None and lat_f is not None and lon_f is not None
                  and _finite(lat_f) and _finite(lon_f)
                  and abs(lat_f) <= 90 and abs(lon_f) <= 180
                  and _valid_time(p[2]))
            loc = None
            if ok:
                ele_f = _float(p[3]) if p[3] else None
                if ele_f is not None and not _finite(ele_f):
                    ele_f = None
                hdop_f = _float(p[4]) if p[4] else None
                extra = ""
                if ele_f is not None:
                    extra += '"altitude":%.1f,' % ele_f
                if hdop_f is not None and _finite(hdop_f):
                    extra += '"horizontal_accuracy":%.1f,' % (hdop_f * HDOP_M)
                loc = ('{"type":"Feature","geometry":{"type":"Point",'
                       '"coordinates":[%.6f,%.6f]},"properties":{'
                       '"timestamp":"%s",%s"device_id":"%s"}}'
                       % (lon_f, lat_f, p[2], extra, DEVICE_ID)).encode()
                need = len(sep) + len(loc)
                if pos + need + 2 > len(_body):
                    if n_ok == 0:
                        print("point too large", len(loc))
                        loc = None
                    else:
                        f.seek(line_start)
                        n -= 1
                        eof = False
                        break
            if loc:
                pos = _put(_put(pos, sep), loc)
                sep = b","
                n_ok += 1
            if n >= UPLOAD_BATCH:
                break
        new_off = f.tell()
    if not sep:
        return n, 0, 0, new_off, eof
    return n, n_ok, _put(pos, b"]}"), new_off, eof

def commit_offset(path, n, new_off, eof=False):
    """Record that n points are done. Delete only at true EOF.
    Crash between POST and this call resends (at-least-once)."""
    write_offset(path, new_off)
    up["left"] = max(up["left"] - n, 0)
    if not eof:
        return
    _rm(path)
    _rm(path + ".pos")
    _clear_bad_full(path)

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
    gpx = not _is_trk(path)
    try:
        with open(path, "rb") as src, open(bad, "ab") as dst:
            if off:
                src.seek(off)
            while src.tell() < new_off:
                line = _readline(src)
                if line is None:
                    break
                if _is_point(line, gpx):
                    dst.write(line)
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
    commit_offset(path, n, new_off, eof)
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

def _http_post(size):
    """POST _body[:size]. Returns (status_code or None, retry_after_ms or None).
    Always closes the response so a non-OSError does not leak the TLS socket
    until the GC finalizer (micropython-lib requests)."""
    feed()
    r = None
    code, retry = None, None
    try:
        import requests
        r = requests.post("%s/api/v1/overland/batches" % up["url"],
                          data=memoryview(_body)[:size],
                          headers={"Content-Type": "application/json",
                                   "Authorization": "Bearer %s" % up["key"]},
                          timeout=HTTP_TIMEOUT)
        code = r.status_code
        retry = _retry_after_ms(r)
    except MemoryError:
        raise
    except Exception as e:
        print("upload", repr(e))
    finally:
        if r is not None:
            try:
                r.close()
            except Exception:
                pass
        feed()
        gc.collect()
    return code, retry

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
        else:
            up["state"] = "idle"
        up["next_at"] = time.ticks_add(now, BACKOFF_START)
        return
    path = files[0]
    off = read_offset(path)
    try:
        n, n_ok, size, new_off, eof = read_batch(path, off)
        feed()
        fsize = _file_size(path) or 0
        if not n:
            if eof or fsize == 0:
                _rm(path)                # empty / EOF track and .bad
                _rm(path + ".pos")
                _clear_bad_full(path)
            else:
                print("pos skip", path, off, new_off, fsize)
                write_offset(path, 0)
            up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
            return
        if not size:
            commit_offset(path, n, new_off, eof)
            up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
            return
    except OSError:
        up["err"] += 1
        up["state"] = "fs err"
        _schedule(now)
        return
    except MemoryError:
        up["skip"] = path
        _low_mem(now)
        return
    if not _bad_full_left():
        up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
    gc.collect()
    if CPU_MHZ_UPLOAD and CPU_MHZ and CPU_MHZ_UPLOAD != CPU_MHZ:
        _set_freq(CPU_MHZ_UPLOAD)        # TLS handshake at full speed
    try:
        code, retry = _http_post(size)
    except MemoryError:
        up["skip"] = path
        _low_mem(time.ticks_ms())
        return
    finally:
        if CPU_MHZ_UPLOAD and CPU_MHZ and CPU_MHZ_UPLOAD != CPU_MHZ:
            _set_freq(CPU_MHZ)
    read_uart()                          # catch up after a multi-second POST
    now = time.ticks_ms()
    print("upload", path, n, n_ok, code, gc.mem_free())
    gap = UPLOAD_GAP_AWAY if not at_home() else UPLOAD_GAP
    if code is not None and 200 <= code < 300:
        try:
            commit_offset(path, n, new_off, eof)
        except OSError:
            up["err"] += 1
            up["state"] = "fs err"
            _schedule(now)
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
        if from_bad:
            bt = up.get("bad_try") or {}
            bt[path] = bt.get(path, 0) + 1
            up["bad_try"] = bt
        if not _bad_full_left():
            up["state"] = "ok"
        up["next_at"] = time.ticks_add(now, gap)
    elif code in PAYLOAD_ERR:
        up["err"] += 1
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

def _low_mem(now):
    gc.collect()
    up["err"] += 1
    up["state"] = "low mem"
    _schedule(now)
    print("upload MemoryError, free", gc.mem_free())

# --- Status page ------------------------------------------------------------
# A small HTTP page on the LAN while the radio is connected (at home it
# stays connected for it, see wifi_tick): http://<ip>/ and /status.json.
# Served from the main loop, one client at a time, at most WEB_TIMEOUT s.

_web_sock = None

def _web_env(env):
    """WEB=0 in .env: no status page, the radio sleeps at home as before."""
    global WEB
    v = _plain(env.get("WEB"))
    if v:
        t = _truthy(v)
        if t is None:
            print("WEB not 1/0, using", 1 if WEB else 0)
        else:
            WEB = t

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
    return _diag_tail(DIAG_FILE, WEB_DIAG)

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
    """(section, label, key, value) rows for the page and the JSON."""
    view = sv_seen()
    t = _fix_secs()
    pos = None
    if st["valid"] and st["lat"] is not None and st["lon"] is not None:
        pos = "%.5f, %.5f" % (st["lat"], st["lon"])
    try:
        cause = machine.reset_cause()
    except Exception:
        cause = None
    try:
        v = os.statvfs("/")
        flash = v[0] * v[3] // 1024
    except OSError:
        flash = None
    still = log.get("still") or {}
    return [
        ("GPS", "Fix", "fix", status()),
        ("GPS", "Satellites seen / used", "sats", "%d / %s" % (view, st["used"])),
        ("GPS", "HDOP", "hdop", st["hdop"]),
        ("GPS", "GPS time (UTC)", "utc", None if t is None else _stamp_at(t)[0]),
        ("GPS", "Position", "position", pos),
        ("GPS", "NMEA ok / bad", "nmea", "%d / %d" % (st["ok"], st["err"])),
        ("GPS", "Receiver restarts", "gps_boots", st["gps_boots"]),
        ("GPS", "Reset-loop warnings", "gps_loops", st["loops"]),
        ("GPS", "Setup acknowledged", "setup_ok", st["setup_ok"]),
        ("Points", "Logged since boot", "points", log["points"]),
        ("Points", "Waiting for upload", "left", up["left"]),
        ("Points", "Track files waiting", "files", len(pending())),
        ("Points", "Uploaded since boot", "sent", up["sent"]),
        ("Points", "Upload state", "upload", up["state"]),
        ("Points", "Upload errors", "upload_err", up["err"]),
        ("Points", "Stay", "stay", "yes" if still.get("active") else "no"),
        ("Device", "Uptime", "uptime", time.ticks_ms() // 1000),
        ("Device", "Previous run", "prev_uptime", prev_run["up"]),
        ("Device", "Boot cause", "boot_cause", cause),
        ("Device", "Chip temperature (C)", "temp", _temp()),
        ("Device", "CPU (MHz)", "mhz", machine.freq() // 1000000),
        ("Device", "Free heap (B)", "mem", gc.mem_free()),
        ("Device", "Free flash (KB)", "flash_kb", flash),
        ("Device", "WiFi", "wifi", "%s %s" % (wifi["state"],
                                             wifi.get("ssid") or "")),
        ("Device", "RSSI (dBm)", "rssi", wifi.get("rssi")),
        ("Device", "Keep-alive pulses", "pulses", wifi["pb_scans"]),
    ]

def _esc(v):
    return ("-" if v is None else str(v)).replace("&", "&amp;").replace(
        "<", "&lt;").replace(">", "&gt;")

def _sv_fresh():
    """Before a page: a satellite list older than SV_FRESH is asked for
    again; the answer is read (with the NMEA around it) for up to SV_WAIT
    ms. Silence leaves the older list."""
    now = time.ticks_ms()
    at = st["svs_at"]
    if at is not None and _age(now, at) < SV_FRESH:
        return
    sv_poll(now)
    while _age(time.ticks_ms(), now) < SV_WAIT:
        read_uart()
        if st["svs_at"] is not None and st["svs_at"] != at:
            return
        time.sleep_ms(20)

def _sv_name(svid):
    return ("GPS %d" % svid) if svid <= 32 else (
        "SBAS %d" % svid if 120 <= svid <= 158 else "SV %d" % svid)

def _sv_list():
    """The satellite list for the page, used first, then by signal."""
    if st["svs"] is None:
        return []
    return sorted(st["svs"], key=lambda v: (not v[4], -v[1], v[0]))

def _web_json():
    d = {"device": DEVICE_ID, "diag": _web_diag(),
         "satellites": [{"name": _sv_name(v[0]), "svid": v[0], "cno": v[1],
                         "elev": v[2], "azim": v[3], "used": v[4],
                         "unhealthy": v[5]} for v in _sv_list()],
         "satellites_age_s": None if st["svs_at"] is None else
         _age(time.ticks_ms(), st["svs_at"]) // 1000}
    for _sec, _label, key, v in _web_rows():
        d[key] = v
    return json.dumps(d)

def _web_html():
    out = ['<!doctype html><html><head><meta charset="utf-8">'
           '<meta name="viewport" content="width=device-width,'
           'initial-scale=1"><meta http-equiv="refresh" content="%d">'
           '<title>%s</title><style>'
           'body{font:15px system-ui,sans-serif;margin:16px;max-width:640px;'
           'background:#fafafa;color:#222}'
           '@media(prefers-color-scheme:dark){body{background:#16181c;'
           'color:#ddd}td{border-color:#333!important}}'
           'h1{font-size:20px;margin:0 0 4px}h2{font-size:15px;margin:18px 0'
           ' 4px;opacity:.7}table{border-collapse:collapse;width:100%%}'
           'td{padding:4px 6px;border-bottom:1px solid #ddd}'
           'td:last-child{text-align:right;font-variant-numeric:tabular-nums}'
           'pre{font-size:12px;overflow-x:auto;white-space:pre}'
           '</style></head><body><h1>%s</h1>' % (
               WEB_REFRESH, _esc(DEVICE_ID), _esc(DEVICE_ID))]
    sec = None
    for s_, label, key, v in _web_rows():
        if key == "uptime":
            v = _hms(v)                  # the JSON keeps seconds
        elif key == "prev_uptime":
            v = _prev_text()
        if s_ != sec:
            out.append("%s<h2>%s</h2><table>" % ("</table>" if sec else "",
                                                  s_))
            sec = s_
        out.append("<tr><td>%s</td><td>%s</td></tr>" % (label, _esc(v)))
    out.append("</table><h2>Satellites</h2>")
    svs = _sv_list()
    if svs:
        out.append("<table><tr><td>Satellite</td><td>Signal (dBHz)</td>"
                   "<td>Elevation</td><td>Azimuth</td><td>Used</td></tr>")
        for v in svs:
            out.append("<tr><td>%s%s</td><td>%s</td><td>%d&deg;</td>"
                       "<td>%d&deg;</td><td>%s</td></tr>" % (
                           _sv_name(v[0]), " (unhealthy)" if v[5] else "",
                           v[1] if v[1] else "-", v[2], v[3],
                           "yes" if v[4] else ""))
        out.append("</table>")
    else:
        out.append("<p>No satellite list from the receiver yet.</p>")
    out.append("<h2>Diagnostic log</h2><pre>")
    for line in _web_diag():
        out.append(_esc(line) + "\n")
    out.append("</pre></body></html>")
    return "".join(out)

def _web_reply(c, req):
    parts = req.split(b" ", 2)
    path = parts[1] if len(parts) > 1 else b"/"
    if path.startswith(b"/status.json"):
        _sv_fresh()
        code, ctype, body = "200 OK", "application/json", _web_json()
    elif path == b"/" or path.startswith(b"/?"):
        _sv_fresh()
        code, ctype, body = "200 OK", "text/html", _web_html()
    else:
        code, ctype, body = "404 Not Found", "text/plain", "not found"
    body = body.encode()
    c.sendall(("HTTP/1.0 %s\r\nContent-Type: %s; charset=utf-8\r\n"
               "Content-Length: %d\r\nCache-Control: no-store\r\n"
               "Connection: close\r\n\r\n" % (code, ctype, len(body))
               ).encode())
    c.sendall(body)

def web_tick():
    """Listen while connected, close otherwise; answer one waiting client."""
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
    try:
        c.settimeout(WEB_TIMEOUT)
        _web_reply(c, c.recv(512))
    except (OSError, MemoryError) as e:
        print("web client", repr(e))
    finally:
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
        room = free()
        for row, line in enumerate((
                _wifi_line(),
                "GPS " + status(),
                _mode_line(),
                "Flash " + _fmt("%d kB", None if room is None else room // 1024),
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
        _diag_write("crash up %d %s" % (time.ticks_ms() // 1000, repr(e)))
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
