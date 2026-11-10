# The Clock's date maths and the one test of a real UTC time. Frozen with the
# logger (tools/build_firmware.sh), or installed beside it as gps_clock.mpy
# (deploy.sh). Pure: no state, no hardware, importable on a PC.
#
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
