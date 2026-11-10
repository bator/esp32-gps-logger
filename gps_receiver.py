# The Receiver: the UC6580 on UART1, its NMEA read in place, the epochs of a UTC
# second averaged and published to st, its setup and its watchdog. Frozen with
# the logger (tools/build_firmware.sh), or installed beside it as
# gps_receiver.mpy (deploy.sh). The logger reads st and binds what this needs
# from it: the diag log (_diag_write) and the watchdog (feed).
from machine import UART
import sys, time, select
from gps_clock import age as _age, recent as _recent, TICKS_CAP


def _diag_write(line):                   # (bound by the logger)
    pass


def feed():                              # (bound by the logger: the watchdog)
    pass


def _say(line):
    """On the console and in the diag log (as the logger's _say)."""
    print(line)
    _diag_write(line)


def _dlon(d):
    """A longitude difference across 180° the short way round, or a longitude
    made of a reference plus an offset back within 180°. (Not the modulo: on
    the board's single floats 19.000123 + 180 keeps 1.5 m.)"""
    if d > 180:
        return d - 360
    if d < -180:
        return d + 360
    return d

BAUD = 115200                            # the UC6580's
GPS_HZ = 5                               # fixes a second; one averaged point
                                         # a second is kept
SECONDS_Q = 8                            # published seconds kept for log_tick
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
NO_DATA = 3000                           # ms of silence = "No data!"
GPS_SILENT = 60000                       # ms without a byte: a diag line
STALE = 2000                             # ms: older position = no fix

# The Heltec Wireless Tracker: UART1 on GPIO34 (TX) / GPIO33 (RX)
GPS_UART, GPS_TX, GPS_RX = 1, 34, 33
# (rides out HTTP calls; at 115200 the UC6580 sends ~2.3 KB/s: a keep-alive
# scan's pause overran 4 KB, one cut sentence per pulse, measured)
uart = UART(GPS_UART, baudrate=BAUD, tx=GPS_TX, rx=GPS_RX, rxbuf=8192)

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

# a Fix's fields by index (_SNAP's order)
F_SEQ, F_LAT, F_LON, F_ALT, F_HDOP, F_HDOP_AT, F_STD, F_STD_AT, F_POS_AT, \
    F_TIME, F_DATE, F_VALID, F_EPOCHS = range(13)

def drain(seen):
    """The published seconds after seq seen, oldest first, as Fixes (tuples in
    _SNAP's order), and the queue emptied. st stays the latest second."""
    q, st["q"] = st["q"], []
    return [f for f in q if f[F_SEQ] > seen]

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
    if 0 < time.ticks_diff(u["until"], time.ticks_ms()) <= GSV_EVERY * 1000:
        return True                      # (a window from before a wrap is over)
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
