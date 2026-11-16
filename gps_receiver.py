# The Receiver: the UC6580 on UART1, its NMEA read in place, the epochs of a UTC
# second averaged and published to st, its setup and its watchdog. Frozen with
# the logger (tools/build_firmware.sh), or installed beside it as
# gps_receiver.mpy (deploy.sh). The logger reads st and binds what this needs
# from it: the diag log (_diag_write) and the watchdog (feed).
from machine import UART
import sys, time, select
from gps_clock import age as _age, recent as _recent, TICKS_CAP, \
    stamp as _stamp


def _diag_write(line):                   # (bound by the logger)
    pass


def feed():                              # (bound by the logger: the watchdog)
    pass


def _say(line):
    """On the console and in the diag log (as the logger's _say)."""
    print(line)
    _diag_write(line)

_bugs = []                               # (where, exception type) said

def _bug_diag(where, line, e):
    """An exception no input explains (a TypeError, a KeyError: a bug): line
    and the exception on the console and in the diag log, once per place and
    exception type a boot (one every sentence must not fill the log). Never
    raises: read_uart calls it in its handler."""
    try:
        k = (where, type(e))
        if k not in _bugs:
            _bugs.append(k)
            r = repr(e)
            print(line, r)
            _diag_write("%s %s" % (line, r[:60]))
    except Exception:
        pass


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
GPS_VERIFY_MAX = 1800000                 # ms without 3 valid seconds to tell
                                         # (none before the first fix): said
                                         # once, the verdict still waits
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
    st.update(start=time.ticks_ms(), start_s=_stamp(), rx=None, rx_b=0,
              good=None, ok=0, err=0, bug=0,
              quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, used=None, view={},
              hdop=None, ttff=None, pos_at=None, hdop_at=None,
              seq=0, epochs=0, extra=0, setup_at=None, mute_at=None, mute_s=None,
              q_thin=0,
              setup_wait=GPS_RETRY, setup_ok=None, std=None, std_at=None,
              gps_boots=0, q=[], loops=0,
              loop_at=time.ticks_ms(), boot_t=[],
              storm_at=None, boot_at=None, boot_last=None, boot_gap=0,
              svs=None, svs_at=None,
              gps_ver=None, gsa_at=None, gsa_n=0, gsa_t=None, uc_ok=0, uc_fail=0, uc_sys=None,
              uc_sys_sent=False, uc_sys_at=None, uc_sys_n=0, fix_first=None, settled=False, gst_said=None,
              ack_n=0, rate_nak=False, gst_nak=False, gst_fails=0, naks=None,
              ep_at=None, ep5_at=None, ep_run=None, ep1_n=0, rate_fails=0,
              sys_said=False,
              checked=True, unverified=False,
              hz1=0, hz5=0,
              set_ok=0, gst_at=None,
              nt_said=None, nt_more=0,
              set_boots=0, rx_back=None, rx_lost=False)
    _ep_new(None)
    _uc_sv_new()

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

NUM_MAX = 16                             # characters of a float field at most

def _num(s, conv=float):
    """A number field, None when empty. A float must be plain decimal:
    "1e9", "nan", "inf" are never NMEA (ValueError; looked for in the
    text, no float made for it). Nor over NUM_MAX characters: 39 digits are
    inf in single precision, and /status.json had "hdop": inf (no JSON)."""
    if not s:
        return None
    if conv is float and (len(s) > NUM_MAX or "e" in s or "E" in s
                          or "n" in s or "N" in s):
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
    m = float(mm)
    # (float() takes "+5.5" and " 5.5": the minutes start with two digits,
    # under 60 by their text: single precision reads 59.999999 as 60.0)
    if (not a.isdigit() or not mm[:2].isdigit() or mm[:2] >= "60"
            or "e" in mm or "E" in mm):
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
    second, not for each sentence of each epoch (5 allocations each).
    False for an empty time: the sentence's position is then no second's
    (it went into the open one)."""
    if len(s) < 6:
        return False
    c = _sec
    if c[0] is None or not s.startswith(c[0]):
        h = s[0:6]
        if not h.isdigit():              # a corrupt digit (see _num): the
            raise ValueError(s)          # sentence is bad; one such time in
                                         # a line stopped the track for good
        if h[0:2] > "23" or h[2:4] > "59" or h[4:6] > "60":
            # no such time (digits flipped past the checksum: no bad
            # sentence, RG-2): no second of its own, as an empty time.
            # Taken, "25:00:01" made the clock 02:00 and dropped the date
            # of the next good second
            return False
        c[0] = h
        c[1] = "%s:%s:%s" % (h[0:2], h[2:4], h[4:6])
    t = c[1]
    if t != ep["t"]:
        if ep["t"] is not None:
            _publish()
        _ep_new(t)
    return True

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
    # (_uc_watch: GPS_HZ lost without GST) a run of valid seconds: begun
    # after NO_DATA without one, its seconds at one fix counted; one second
    # of several fixes ends it (a fix back in a second's last epoch was one
    # fix: the watch fired at every fix regained); one TICKS_CAP long is
    # begun again (_stamps_old ended it: the watch was off for good after a
    # day of fixes)
    if st["ep_run"] is None or not _recent(now, st["ep_at"], NO_DATA):
        st["ep_run"], st["ep1_n"] = now, 0
    st["ep_at"] = now
    if n > 1:
        st["ep5_at"], st["ep1_n"] = now, 0
    else:
        st["ep1_n"] += 1
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
    if st["ttff"] is None:               # (a Stamp: a Mark of the boot read
        st["ttff"] = _stamp() - st["start_s"]   # negative after 6.2 days)
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
# small allocation a second rather than a dict. fixtype: GSA's when the second
# was published (a queued Fix's altitude is judged by its own)
_SNAP = ("seq", "lat", "lon", "alt", "hdop", "hdop_at", "std", "std_at",
         "pos_at", "time", "date", "valid", "epochs", "fixtype")

def _snap():
    return tuple(st[k] for k in _SNAP)

# a Fix's fields by index (_SNAP's order)
F_SEQ, F_LAT, F_LON, F_ALT, F_HDOP, F_HDOP_AT, F_STD, F_STD_AT, F_POS_AT, \
    F_TIME, F_DATE, F_VALID, F_EPOCHS, F_FIXTYPE = range(14)

def unsettled(now):
    """True while the UC6580's first fix is under UC_SETTLE old and the board
    is slower than UC_SETTLE_KMH: its first minute counts only moving (15 m
    off at first, 5 m after a minute, measured; standing still that draws a
    walk). Settled once that minute is over."""
    if st["settled"]:
        return False
    a = st["fix_first"]
    if a is None:
        return False
    # not _age: a second queued before a restart (drain) is older than this
    # fix_first, and TICKS_OLD would settle it
    if time.ticks_diff(now, a) >= UC_SETTLE:
        st["settled"] = True
        return False
    return (st.get("speed") or 0) < UC_SETTLE_KMH

def drop_queue():
    """Forget the queued seconds (no fix: they are stale). A new list only
    when there is one: this runs every pass without a fix."""
    if st.get("q"):
        st["q"] = []

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
    timed = _second(f[1])
    st["speed"] = None if kn is None else kn * 1.852
    if not timed:
        return
    if kn is not None and (ep["spd"] is None or st["speed"] > ep["spd"]):
        ep["spd"] = st["speed"]          # the second's top speed
    if date is not None:
        ep["date"] = date
    if f[2] == "A" and pos and (len(f) < 13 or f[12] != "E"):
        _acc("r", pos)

def gga(talker, f):
    q, n, pos = _num(f[6], int), _num(f[7], int), _position(f, 2)
    alt, hd = _num(f[9]), _num(f[8])     # (all read first: _num)
    timed = _second(f[1])
    st["quality"], st["used"] = q, n
    if timed and 0 < (q or 0) <= GGA_FIX_MAX and pos:
        _acc("g", pos)
        if alt is not None:
            ep["alt"][0] += alt
            ep["alt"][1] += 1
        if hd is not None and hd > 0:   # (0 or below: no HDOP, not perfect)
            ep["hdop"][0] += hd
            ep["hdop"][1] += 1

def gsa(talker, f):
    """Fix type. One per system and round on a multi-GNSS receiver; more
    than GSA_MAX in a second means the GSA rate of the setup was lost:
    gps_check sets it up again. The receiver's second (the last time
    sentence's), not the uptime's: after a loop blocked by a POST, two
    rounds 5 s apart on the wire were read within one uptime second and
    counted as a lost rate (a needless setup). Before any time sentence,
    the uptime's. Only a 2D or 3D mode is kept: GSA comes every
    GSA_EVERY s, RMC every second, so after a "1" (no fix) round an RMC
    valid again showed "Fix" for up to 5 s (measured indoors: 14 of 201
    rounds). The last dimension stands until the next 2D/3D round."""
    ft = _num(f[2], int)
    _uc_gsa(talker, f)                   # (first: a field that fails there
    if ft in (2, 3):                     # leaves no fix type, no count)
        st["fixtype"] = ft
    t = _sec[0]
    if t is not None:
        if t != st["gsa_t"]:
            st["gsa_t"], st["gsa_n"] = t, 0
    else:
        now = time.ticks_ms()
        if not _recent(now, st["gsa_at"], 1000):
            st["gsa_at"], st["gsa_n"] = now, 0
    st["gsa_n"] += 1
    if st["gsa_n"] == GSA_MAX + 1:
        _extra(talker, f)

def _extra(talker, f):
    """A sentence gps_setup turns off: the receiver did not take it."""
    at = st["setup_at"]
    if at is not None and _age(time.ticks_ms(), at) >= 1000:
        st["extra"] += 1

CNT_MASK = 0x3FFFFFFF                    # sentence counts wrap here (a small
                                         # int: ~40 a second passed 2**30 in
                                         # ~310 days, each += then a long)

_view_at = {}                            # talker -> ticks of its second's
                                         # first GSV set (gsv)
GSV_SET_MS = 2000                        # a second's sets (L1, L5): a loop
                                         # stall between them is no new one
                                         # (GSV comes each GSV_EVERY s)

def gsv(talker, f):
    n = _num(f[3], int)                  # (read first: a bad one changes
    _uc_gsv(talker, f)                   # nothing; its GSV stays on: no _extra)
    if n is not None and talker in _UC_GNSS:
        # per constellation, summed on display (a talker of no system: not
        # summed). A UC6580 sends a set per signal (L1, then L5), each with
        # its own count: the largest of a second's sets, not the last one
        # (L5's fewer stood for the constellation)
        now = time.ticks_ms()
        va = _view_at
        if _recent(now, va.get(talker), GSV_SET_MS):
            n = max(n, st["view"].get(talker, 0))
        else:
            va[talker] = now             # (a new second's first set)
        st["view"][talker] = n

def gst(talker, f):
    """The receiver's own error estimate (1-sigma, metres) of the second's
    position: the larger of latitude and longitude. A cold start converging
    shows here while its HDOP already looks good."""
    sl, sn = _num(f[6]), _num(f[7])      # (read first: _num)
    timed = _second(f[1])
    st["gst_at"] = time.ticks_ms()       # it comes (_setup_check)
    if timed and sl is not None and sn is not None and sl > 0 and sn > 0:
        ep["std"] = sl if sl > sn else sn

def _date_lost():
    """The date kept from before is no longer the receiver's: a restart, a
    silence, a restart whose banner was lost. One kept across a silence of
    a day and more stamped the next fix without its own RMC with the old
    day, its time new; the midnight rule (_publish) only sees seconds in a
    row."""
    st["date"] = None
    _day[0] = _day[1] = None

def _silence_end():
    """Bytes again after GPS_SILENT without any, before they are handled:
    by whoever reads (the POST guard's timer, a read in a scan, gps_check):
    the date kept from before is no longer the receiver's, and neither is
    the second left open then, nor the half line (its date published the
    old day again: a GGA-only second after a day of silence dated a day
    back). The open second is published first, with its own date: it is
    the last one before the silence, not lost with it."""
    global _rx_n, _rx_s
    if ep["t"] is not None:
        try:
            _publish()
        except MemoryError:
            st["err"] = (st["err"] + 1) & CNT_MASK
    _date_lost()
    _ep_new(None)
    _sec[0] = _sec[1] = None
    _rx_n = _rx_s = 0

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
    _date_lost()
    print("gps restarted")
    _gps_flap(now)                       # first: the restart that makes
    storm = (st["storm_at"] is not None  # it a storm waits too
             and _age(now, st["storm_at"]) < GPS_FLAP_WINDOW)
    st["setup_wait"] = GPS_STORM_WAIT if storm else GPS_RETRY
    st["ack_n"] = 0                      # at its defaults: a fresh start
    # the model asked anew (_uc_setup); the settle rule anew, and not the
    # second it was in: published by the next sentence, its fixes would start
    # the rule before a real fix
    st.update(svs=None, svs_at=None, gps_ver=None, fix_first=None, naks=None,
              settled=False)
    # GPS_SYS asked (and set) anew after a restart of its own (a power dip:
    # back at the default mask, kept so for the run); not after the one a
    # mask sent just now makes (UC_SYS_SETTLE), at most UC_SYS_MAX a boot
    sa = st["uc_sys_at"]
    own = sa is None or _age(now, sa) >= UC_SYS_SETTLE
    if own:                              # (asked anew: a mask it had before
        st["uc_sys"] = None              # the restart is no mask it has now)
    if st["uc_sys_sent"] and own:
        st["uc_sys_sent"] = False
    _ep_new(None)
    _sec[0] = _sec[1] = None             # (its time starts afresh too)
    _uc_sv_new()

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
    """Satellites with a signal: from a satellite list younger than two
    SV_POLL (as sv_used: a list no round closes any more is stale), else
    the GSV count."""
    at = st["svs_at"]
    if (st["svs"] is not None and at is not None
            and _age(time.ticks_ms(), at) < 2 * SV_POLL):
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
UC_SYS_SETTLE = 10000                    # ms: a restart this soon after a mask is its own
UC_SYS_MAX = 3                           # masks sent a board boot (a loop's end)
GST_NAK_N = 2                            # setups in a row with GST refused: no GST
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
        if (UC_SYS is not None and not st["uc_sys_sent"]
                and st["uc_sys_n"] >= UC_SYS_MAX and not st["sys_said"]):
            st["sys_said"] = True        # (the default mask from here on)
            _diag_write("gps GPS_SYS sent %d times this boot: not again"
                        % UC_SYS_MAX)
        if (UC_SYS is not None and not st["uc_sys_sent"]
                and st["uc_sys_n"] < UC_SYS_MAX):
            # asked first (only a mask it does not have; setting one
            # restarts the receiver, and that restart brings this setup
            # back: sent again only after a restart of its own,
            # _gps_restarted, UC_SYS_MAX a boot)
            _uc_cmd("CFGSYS")
            if st["uc_sys"] not in (None, UC_SYS):
                st["uc_sys_sent"] = True
                st["uc_sys_n"] += 1
                st["uc_sys_at"] = time.ticks_ms()
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
                # said once a receiver boot (naks; _gps_restarted clears
                # it): a setup that came again each GPS_IGNORED_MAX wrote
                # the same line ~1400 times a day over the real events
                nk = st["naks"]
                if nk is None:
                    nk = st["naks"] = []
                if cmd not in nk:
                    nk.append(cmd)
                    _say("gps refused " + cmd)
                if cmd.startswith("CFGNAV"):
                    # (as GST's below: twice in a row; one FAIL left the
                    # receiver at 1 Hz with no verdict until a restart)
                    st["rate_fails"] += 1
                    st["rate_nak"] = st["rate_fails"] >= GST_NAK_N
                elif cmd.startswith("CFGMSG,0,7,"):
                    # (no GST: no "not taken" for it) only refused twice in
                    # a row: one FAIL (a line hit on the way, a late answer
                    # to the command before) took GST, the GPS_MAX_STD filter
                    # and _uc_watch off until the receiver restarted
                    st["gst_fails"] += 1
                    st["gst_nak"] = st["gst_fails"] >= GST_NAK_N
            elif cmd.startswith("CFGMSG,0,7,"):
                st["gst_fails"], st["gst_nak"] = 0, False
            elif cmd.startswith("CFGNAV"):
                st["rate_fails"], st["rate_nak"] = 0, False
        feed()
        return True
    except Exception as e:               # (an OSError of the UART too: said
        _bug_diag("setup", "bug gps setup", e)   # once a type, not only printed)
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
_uc_sv = {}

def _uc_sv_new():
    """No satellite round begun, no GSA list, no GSV window (reset, a
    receiver restart)."""
    _uc_sv.update(new={}, open=False, until=None, used=[], gsa=None, gt=None,
                  sec=None, head=None, skip_at=None)

_uc_sv_new()
# GSV's signal id (its last field) -> its band, 1 (L1/E1/B1) or 5 (L5/E5a/
# B2a), keyed gnssId << 8 | id (UFirebird II spec, table 1-3); others none
_UC_BAND = {0 << 8 | 1: 1, 0 << 8 | 8: 5, 2 << 8 | 7: 1, 2 << 8 | 1: 5,
            3 << 8 | 1: 1, 3 << 8 | 3: 1, 3 << 8 | 5: 5, 5 << 8 | 1: 1,
            5 << 8 | 8: 5, 6 << 8 | 1: 1}

def _uc_gsv_skip():
    """True while a list is fresh: with a fix its GSV comes at 1 Hz whatever
    the rate (25 is read back; ~25 sentences a second, measured), each
    decoded and split into ~20 strings for nothing."""
    u = _uc_sv
    if u["until"] is None:
        return False
    now = time.ticks_ms()
    d = time.ticks_diff(u["until"], now)
    if 0 < d <= GSV_EVERY * 1000:
        u["sec"] = _sec[0]               # (seen: the window may end mid-round)
        u["skip_at"] = now
        return True                      # (a window from before a wrap is over)
    if (_sec[0] is not None and _sec[0] == u["sec"]
            and _recent(now, u.get("skip_at"), UC_GSA_GAP)):
        # the rest of the burst it ended in, the same second (its L1 sets
        # skipped, its L5 ones read: the view counted L5's fewer)
        u["skip_at"] = now
        return True
    u["until"] = None
    return False

def _uc_gsa(talker, f):
    """One GSA: the satellites of its system in the fix. A new receiver
    second (HHMMSS of the last time sentence; rounds come every GSA_EVERY
    s, at most every second) starts a new list: the sentences of a round
    come in one burst after their epoch's time, and a round every second by
    the uptime cut one in two (11 seen, 6 in use). Told by a pause in the
    reading before (UC_GSA_GAP, still so until a time sentence came), a
    loop pass of over 150 ms in a burst split it, and two rounds read in
    one go were one. The system is the last field (NMEA 4.10);
    before it (18 fields) the talker's, and none for GN (mixed): that one
    adds nothing. Its fields are read first: one that does not parse
    (ValueError) leaves the round as it was, none of its satellites in use."""
    n = len(f)
    if n < 18:
        return
    g = _UC_SYS.get(_num(f[-1], int)) if n > 18 else _UC_GNSS.get(talker)
    ids = None
    if g is not None:
        ids = []
        for x in f[3:15]:
            v = _num(x, int)
            if v is not None:
                ids.append(g << 8 | v)
    u = _uc_sv
    now = time.ticks_ms()
    e = _sec[0]
    if e is not None:
        if e != u["gt"]:
            u["used"] = []
            u["gt"] = e
    elif not _recent(now, u["gsa"], UC_GSA_GAP):
        u["used"] = []
    u["gsa"] = now
    if ids:
        u["used"].extend(ids)

def _uc_gsv(talker, f):
    """One GSV sentence into a round's list (the same satellite on two
    bands: its strongest as its cno, and each band's on its own). A round is
    the GSV of one receiver second (HHMMSS of the last time sentence, as
    GSA's: they come in one burst after their epoch's time); the first GSV of
    the next second closes it as st["svs"]: (svid, cno, elevation, azimuth,
    used, unhealthy, gnssId, L1 cno, L5 cno), and the next GSV_EVERY s of
    GSV are not even decoded (_uc_gsv_skip). Without a time sentence yet (or
    the same second still), the round's first sentence coming again closes
    it. (GPS's ",1" first sentence did: a GPS_SYS mask without GPS never
    made a list, an empty GPS sentence of another signal never closed one.)
    Its satellites are read first: one field that does not parse
    (ValueError) leaves the round as it was (a sentence counted bad went
    into the sky shown)."""
    g = _UC_GNSS.get(talker)
    if g is None:
        return
    sats = []
    for i in range(4, len(f) - 3, 4):
        n = _num(f[i], int)
        if n is not None:
            sats.append((n, _num(f[i + 1], int), _num(f[i + 2], int),
                         _num(f[i + 3], int) or 0))
    u = _uc_sv
    e = _sec[0]
    new = e is not None and e != u["sec"]    # its second's first GSV
    u["sec"] = e
    h = u["head"]
    again = f[2] == "1" and h is not None and f[0] == h[0] and f[-1] == h[1]
    if u["open"] and (new or again):
        # an empty round too: no signal at all, else the last sky stays
        used = u["used"]
        st["svs"] = [(k & 255, v[0], v[1], 0 if v[2] is None else v[2],
                      k in used, False, k >> 8, v[3], v[4])
                     for k, v in u["new"].items()]
        st["svs_at"] = time.ticks_ms()
        u["new"], u["open"] = {}, False
        u["until"] = time.ticks_add(time.ticks_ms(), GSV_EVERY * 1000)
        return
    if not u["open"]:
        if not (new or again or h is None and f[2] == "1"):
            return                       # (the rest of a round begun before)
        u["open"] = True
        if not again:
            u["head"] = (f[0], f[-1])
    sig = f[-1]                          # ("B", BDS B2I, is hex: no band)
    band = _UC_BAND.get(g << 8 | int(sig)) if sig.isdigit() else None
    for n, el, az, cno in sats:
        k = g << 8 | n
        v = u["new"].get(k)
        if v is None:
            v = u["new"][k] = [0, None, None, 0, 0]
        if cno > v[0]:                   # (a weaker or empty band later
            v[0] = cno                   # keeps it)
        if v[1] is None and el is not None:
            v[1] = el                    # (from the first band that has it)
        if v[2] is None and az is not None:
            v[2] = az
        if band == 1 and cno > v[3]:
            v[3] = cno
        elif band == 5 and cno > v[4]:
            v[4] = cno

def gps_setup():
    """Ask the receiver for GPS_HZ fixes a second (_uc_setup). Not saved in
    the receiver, so it is sent at every boot; gps_check sends it again while
    the receiver still sends the sentences it turns off."""
    read_uart()                          # a banner in it came before
    boots = st["gps_boots"]
    st["setup_at"], st["extra"] = time.ticks_ms(), 0
    _setup_done(_uc_setup(), boots)

def _setup_done(ok, boots):
    """What a setup leaves for gps_check."""
    # sentences sent while the setup ran are not a receiver that ignored
    # it; a restart meanwhile (its banner was read) is: set up again
    st.update(setup_at=time.ticks_ms(),
              extra=0 if st["gps_boots"] == boots else 1, setup_ok=ok,
              checked=not ok, unverified=False, hz1=0, hz5=0, set_ok=st["ok"],
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
    or indoors) there is no verdict yet, however long that takes (said once
    after GPS_VERIFY_MAX: it gave up then, and a receiver that answered but
    stayed at 1 Hz was never caught once a fix came). Taken (the backoff
    starts afresh) only with GPS_HZ fixes a second and GST seen."""
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
        if age >= GPS_VERIFY_MAX and not st["unverified"]:
            st["unverified"] = True      # no fix all this time: said once,
            print("gps setup unverified: no fix")   # the verdict waits on
        return
    st["checked"] = True
    slow = GPS_HZ > 1 and not hz5 and not st["rate_nak"]
    g = st["gst_at"]
    gst = g is not None and time.ticks_diff(g, st["setup_at"]) >= 0
    no_gst = (not gst and not st["gst_nak"]
              and (st["ok"] - st["set_ok"]) & CNT_MASK >= 10)
    # what came clears a FAIL before it (a late answer, a hit line): one
    # left counted made a stray FAIL days later "refused" twice, the watch
    # off for good (GST, the rate)
    if gst:
        st["gst_fails"], st["gst_nak"] = 0, False
    if hz5:
        st["rate_fails"], st["rate_nak"] = 0, False
    if slow or no_gst:
        st["extra"] += 1
        why = ("1 fix/s" if slow else
               "GST refused once, asked again" if st["gst_fails"] else "no GST")
        print("gps setup not taken:", why)
        _not_taken(now, why)
    elif ((gst or st["gst_nak"]) and (hz5 or GPS_HZ == 1 or st["rate_nak"])
          and not st["extra"]):
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
           "gst_at", "boot_at", "boot_last", "nt_said", "gst_said", "rx_back",
           "ep_at", "ep5_at", "ep_run", "uc_sys_at")

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
            if k == "rx":
                st["rx_lost"] = True     # (bytes then end a silence: _read_uart)
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
    byte (the boot when none came); mute_s: its Stamp, for the length (the
    ticks wrap: a silence over 6.2 days read as 536870 s)."""
    rx = st["rx"]
    at = st["start"] if rx is None else rx
    m = st["mute_at"]
    if m is None:
        if _age(now, at) >= GPS_SILENT:
            st["mute_at"] = at
            a = _age(now, at) // 1000
            st["mute_s"] = _stamp() - a
            _diag_write("gps silent %d s, no bytes from the receiver" % a)
    elif _recent(now, rx, GPS_SILENT):
        st["mute_at"] = None
        ms = st.get("mute_s")
        n = (_age(rx, m) // 1000 if ms is None
             else max(0, _stamp() - ms - _age(now, rx) // 1000))
        st["mute_s"] = None              # (the date went as the bytes came:
        _diag_write("gps back after %d s" % n)   # _silence_end)

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
    if st["gst_nak"]:
        # (a receiver that refused GST: no GST is no lost setup, it was set
        # up again every ~40 s for good) GPS_HZ tells instead: valid seconds
        # at one fix for UC_GST_LOST (a rate refused tells nothing)
        # (UC_GST_LOST of valid seconds in one run, nearly all at one fix)
        run = st["ep_run"]
        if (GPS_HZ > 1 and not st["rate_nak"] and _age(now, at) >= UC_GST_LOST
                and run is not None and _recent(now, st["ep_at"], NO_DATA)
                and _age(now, run) >= UC_GST_LOST
                and st["ep1_n"] >= UC_GST_LOST // 2000
                and not _recent(now, st["ep5_at"], UC_GST_LOST)
                and not _recent(now, st["rx_back"], UC_GST_LOST)):
            _setup_lost(now, "1 fix/s")
        return
    g = st["gst_at"]
    if _age(now, at) < UC_GST_LOST or _recent(now, g, UC_GST_LOST):
        return
    if _recent(now, st["rx_back"], UC_GST_LOST):
        return                           # (bytes again after a silence: its
                                         # GST a second later, not yet seen)
    if not _recent(now, st["rx"], NO_DATA):
        return                           # silent: _gps_mute's case
    _setup_lost(now, "GST refused once, asked again" if st["gst_fails"] else "no GST")

def _setup_lost(now, why):
    st["extra"] = 1                      # (its usual backoff, under a minute)
    _date_lost()
    print("gps setup lost:", why)
    said = st["gst_said"]
    if not _recent(now, said, GPS_NOT_TAKEN_SAY):
        st["gst_said"] = now
        _diag_write("gps setup lost: " + why)

EP_OVER = 1500                           # ms without a byte: the open second is
                                         # over (a 1 Hz burst's gap is ~0.9 s)

def _ep_over(now):
    """Publish the open second once no byte came for EP_OVER: it waited for
    the next second's first sentence, and at the end of a silence the
    logger had broken the segment already (STALE): the last point before
    the gap went into the new segment, a line over the gap, and after
    GPS_SILENT it was dropped with the queue. A late sentence of it makes
    a second of the same time again: the logger drops it (t <= last_t)."""
    if ep["t"] is None or _recent(now, st["rx"], EP_OVER):
        return
    try:
        _publish()
    except MemoryError:
        st["err"] = (st["err"] + 1) & CNT_MASK
    _ep_new(None)

def gps_check(now):
    """Send the setup again while the receiver still sends the sentences it
    turns off: a receiver powered up with the board is still booting when
    run() sends it, and stays at 1 Hz. Less and less often, so a receiver
    that never takes it (or an unwired ESP TX) costs next to nothing."""
    _usb_drain()
    _ep_over(now)
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
        st["err"] = (st["err"] + 1) & CNT_MASK
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
        st["err"] = (st["err"] + 1) & CNT_MASK
        return
    st["good"] = time.ticks_ms()         # the last whole frame (GPS link)
    k = -1                               # a 5-character address: its type
    if star - s == 6 or star - s > 6 and v[s + 6] == 44:
        k = v[s + 3] << 16 | v[s + 4] << 8 | v[s + 5]
    name = _HKEY.get(k)
    if name == "GSV" and _uc_gsv_skip():
        st["ok"] = (st["ok"] + 1) & CNT_MASK
        return
    if name is None and hi < 128:
        _uc_line(p)
        st["ok"] = (st["ok"] + 1) & CNT_MASK                    # one the logger does not read
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
        st["ok"] = (st["ok"] + 1) & CNT_MASK
    except (ValueError, IndexError, UnicodeError, MemoryError):
        st["err"] = (st["err"] + 1) & CNT_MASK

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

def _rx_type(b, s, e):
    """The sentence type of the line b[s:e] ("RMC"), "?" for an odd byte:
    for a bug's diag line, never raises."""
    try:
        i = b.rfind(b"$", s, e)
        return "".join(chr(c) if 32 < c < 127 else "?"
                       for c in b[i + 3:min(i + 6, e)]) if i >= 0 else "-"
    except Exception:
        return "-"

_reading = [False]                       # read_uart is running

def read_uart():
    """_read_uart, not re-entered: the POST guard's timer reads the UART
    during a blocking connect or TLS handshake too (its 8 KB were full
    after ~6 s, NMEA lost), and a call it lands in is left alone."""
    if _reading[0]:
        return
    _reading[0] = True
    try:
        _read_uart()
    finally:
        _reading[0] = False

def _read_uart():
    """What the UART has: lines to handle(), in place. An error in one (a
    MemoryError while publishing a second) counts it bad (st["err"]), any
    other is a bug (st["bug"], a diag line once a type), and the rest is
    read: no exception leaves the buffer uncommitted, or the next
    call handled the same sentences again and queued the same seconds twice.
    At most RX_READS reads: a UART never empty (a flood, handlers slower than
    it) must not hold the loop, its feed() and the logging."""
    global _rx_n, _rx_s
    b = _rx
    left = RX_READS
    while left and uart.any():
        left -= 1
        now = time.ticks_ms()
        last = st["rx"]                  # (None: none yet, or a day and more:
        if last is None:                 # _stamps_old forgot it, rx_lost)
            gap = TICKS_CAP if st["rx_lost"] else None
        else:
            gap = _age(now, last)
        if gap is not None and gap >= NO_DATA:
            st["rx_back"] = now          # (GST is due a second after this)
            if gap >= GPS_SILENT:
                st["rx_lost"] = False
                _silence_end()
        n = _rx_n
        k = uart.readinto(_rxv[n:n + RX_READ])
        if not k:
            return
        st["rx"] = now
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
            except MemoryError:          # (publishing a second): the line
                st["err"] = (st["err"] + 1) & CNT_MASK           # is bad, not read again
            except Exception as x:       # a bug, not bad NMEA: counted
                st["bug"] += 1           # apart, said once a type
                _bug_diag("nmea", "bug nmea " + _rx_type(b, r, e), x)
            r = s = j + 1
        if r:
            _rx_drop(r)
        _rx_s = s - r
        if _rx_n > RX_LINE:              # no newline in sight: garbage or a
            _rx_n = _rx_s = 0            # wrong baud rate
