# The Clock: date maths, the one test of a real UTC time, and the Stamps.
# Frozen with the logger (tools/build_firmware.sh), or installed beside it as
# gps_clock.mpy (deploy.sh). Importable on a PC: time.ticks_* is only called
# by the Stamp functions, which take it from the time module (tests replace
# those).
import time

# UTC seconds <-> date without time.mktime (its epoch and time zone differ
# between ports); proleptic Gregorian, days since 1970-01-01.

def days(y, m, d):
    if m <= 2:
        y -= 1
    era = y // 400
    yoe = y - era * 400
    doy = (153 * (m + (-3 if m > 2 else 9)) + 2) // 5 + d - 1
    return era * 146097 + yoe * 365 + yoe // 4 - yoe // 100 + doy - 719468

def civil(z):
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

def ymd_ok(y, mo, d, hh, mi, ss):
    """A real UTC time in 2020-2099 (the ranges the track keyframes check,
    without the FIX_S_END bound): month 13 or hour 25 went up as they were."""
    if not (2020 <= y <= 2099 and 1 <= mo <= 12 and d >= 1
            and hh < 24 and mi < 60 and ss < 60):
        return False
    return d <= 28 or d <= days(y, mo + 1, 1) - days(y, mo, 1)

def tc_ok(rb, t):
    """The compact time YYYYMMDDTHHMMSSZ at rb[t:t + 16], its digits
    checked, is a real one (ymd_ok)."""
    return ymd_ok((rb[t] - 48) * 1000 + (rb[t + 1] - 48) * 100
                  + (rb[t + 2] - 48) * 10 + rb[t + 3] - 48,
                  (rb[t + 4] - 48) * 10 + rb[t + 5] - 48,
                  (rb[t + 6] - 48) * 10 + rb[t + 7] - 48,
                  (rb[t + 9] - 48) * 10 + rb[t + 10] - 48,
                  (rb[t + 11] - 48) * 10 + rb[t + 12] - 48,
                  (rb[t + 13] - 48) * 10 + rb[t + 14] - 48)

def dt_s(d, t, i=0):
    """Seconds since 2020 of a date YYYY-MM-DD and, from index i of t, a time
    HH:MM:SS (an ISO time is both: i 11), or None: no number, or no real
    time in 2020-2099 (ymd_ok: a month 13, an hour 25 past the checksum)."""
    try:
        y, m, dd = int(d[0:4]), int(d[5:7]), int(d[8:10])
        h, mi, ss = int(t[i:i + 2]), int(t[i + 3:i + 5]), int(t[i + 6:i + 8])
    except (TypeError, ValueError, IndexError):
        return None
    if not ymd_ok(y, m, dd, h, mi, ss):
        return None
    return (days(y, m, dd) - EPOCH_DAYS) * 86400 + h * 3600 + mi * 60 + ss

def stamp_s(secs):
    """(ISO time, compact track time) for seconds since 2020."""
    y, m, d = civil(secs // 86400 + EPOCH_DAYS)
    r = secs % 86400
    v = (y, m, d, r // 3600, r // 60 % 60, r % 60)
    return ("%04d-%02d-%02dT%02d:%02d:%02dZ" % v,
            "%04d%02d%02dT%02d%02d%02dZ" % v)

# --- Stamps ------------------------------------------------------------------
# A Stamp is a moment in whole seconds of uptime, for anything that can live
# longer than a minute (ADR 0001). time.ticks_ms wraps every 2**30 ms (12.4
# days) and reads wrongly from 2**29, so the uptime is summed up from tick
# differences here: stamp() has to run at least once in that time (the main
# loop does). A None Stamp means never.
NEVER = 1 << 29                          # age_s(None), seconds: older than any

_up = [0, None, 0]                       # uptime s, the ticks then, ms over s

def stamp(now=None):
    """The current Stamp: seconds since boot. now: the loop's ticks_ms."""
    if now is None:
        now = time.ticks_ms()
    u = _up
    if u[1] is None:                     # (first call: the ticks count from boot)
        u[0], u[1], u[2] = now // 1000, now, now % 1000
        return u[0]
    ms = u[2] + time.ticks_diff(now, u[1])
    u[1] = now
    u[0] += ms // 1000
    u[2] = ms % 1000
    return u[0]

def age_s(st):
    """Seconds since Stamp st; NEVER for None. Never negative."""
    if st is None:
        return NEVER
    a = stamp() - st
    return a if a > 0 else 0

def recent_s(st, secs):
    """st (a Stamp, or None: never) is under secs old."""
    return st is not None and age_s(st) < secs

# --- Anchors ------------------------------------------------------------------
# An Anchor ties a UTC time to the uptime: (seconds since 2020, Stamp then),
# from a fix or from NTP. It needs no rebasing: a Stamp does not wrap.

def utc_s(anchor):
    """Seconds since 2020 now by the Anchor, or None without one."""
    if anchor is None:
        return None
    return anchor[0] + age_s(anchor[1])

# --- Marks (ticks) ------------------------------------------------------------------
# ticks_ms wraps every 2**30 ms and ticks_diff is signed, so a timestamp older
# than ~6.2 days reads as being in the future. Ages and deadlines go through
# these helpers: a wrapped past timestamp counts as very old, a deadline that
# lies further ahead than it could ever have been scheduled counts as due.

TICKS_OLD = 1 << 29                      # "very old" for a wrapped age
TICKS_SKEW = 1000                        # ms a timestamp may be "ahead"
TICKS_CAP = 86400000                     # ms after which gps_check forgets
                                         # a receiver timestamp (it would
                                         # read as fresh again after 2**30)

def recent(now, t, ms):
    """t (ticks, or None: never) is under ms old."""
    return t is not None and age(now, t) < ms

def age(now, t):
    """ms since ticks t. A timestamp taken just after now (say read_uart()
    ran between the two ticks_ms() calls) is age 0; a wrapped (large
    negative) difference is TICKS_OLD."""
    d = time.ticks_diff(now, t)
    if d >= 0:
        return d
    return 0 if d > -TICKS_SKEW else TICKS_OLD

def waiting(now, at, cap):
    """True while deadline at (scheduled at most cap ms ahead) is ahead."""
    d = time.ticks_diff(at, now)
    return 0 < d <= cap
