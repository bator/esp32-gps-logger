#!/usr/bin/env python3
"""Convert a .trk or .trb track from the ESP32 to GPX 1.1.

    .venv/bin/mpremote cp :/gpx/20261005_120005.trb .
    python3 tools/trk2gpx.py 20261005_120005.trb > 20261005_120005.gpx

Quarantined (.trk.bad, .trb.bad) and parked unreadable (.trk.err,
.trb.err) files convert the same way; a point with bytes that are not
UTF-8 (a damaged flash block) in a field that is no number is dropped, as
the logger drops it from the upload. A .trb (#TRKB) holds the same text
lines as differences (see trb_lines); its .bad copy is text.

Line format: "lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ" (#TRK1; ele/hdop may be
empty) or "lat,lon,ele,acc,speed,YYYYMMDDTHHMMSSZ" (#TRK2, from UBX-NAV-PVT:
acc the horizontal accuracy in m, speed in m/s, each may be empty), told
apart by the line itself; "-" starts a new segment, "#..." lines are the
header/footer, and a line without its newline (cut off by a power loss) is
skipped. A TRK2 point's acc and speed go into <extensions> (namespace
EXT_NS, declared only when there is one): GPX 1.1 has no accuracy element,
and its <hdop> is not one.
"""
import calendar
import math
import re
import sys
import time
from xml.sax.saxutils import escape

GPX_HEAD = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<gpx version="1.1" creator="esp32 gps_logger" '
            'xmlns="http://www.topografix.com/GPX/1/1">\n'
            '<trk><name>%s</name><trkseg>\n')
GPX_TAIL = "</trkseg></trk></gpx>\n"
EXT_NS = "https://github.com/bator/esp32-gps-logger/trk2"


def _iso(tc):
    """YYYYMMDDTHHMMSSZ -> YYYY-MM-DDTHH:MM:SSZ, or None: also for a time
    that is no UTC time in 2020-2099 (month, day of the month, hour,
    minute, second), as the logger's _tc_ok."""
    if (len(tc) != 16 or tc[8] != "T" or tc[15] != "Z"
            or not tc.isascii() or not (tc[:8] + tc[9:15]).isdigit()):
        return None
    y, mo, d = int(tc[0:4]), int(tc[4:6]), int(tc[6:8])
    if not (2020 <= y <= 2099 and 1 <= mo <= 12
            and 1 <= d <= calendar.monthrange(y, mo)[1]
            and int(tc[9:11]) < 24 and int(tc[11:13]) < 60
            and int(tc[13:15]) < 60):
        return None
    return "%s-%s-%sT%s:%s:%sZ" % (tc[0:4], tc[4:6], tc[6:8],
                                   tc[9:11], tc[11:13], tc[13:15])


def _num(s):
    """A finite number string kept as written when it is a plain decimal,
    else as "%.1f" prints it (as the logger's _ptnum: float() also takes
    "1_0", "1e3", " 10", which are no xsd:decimal), or "" (dropped)."""
    try:
        v = float(s) if s else math.inf
    except ValueError:
        return ""
    if not math.isfinite(v):
        return ""
    return s if _NUM.fullmatch(s) else "%.1f" % v


def _not_utf8(s):
    """s (read with errors="surrogateescape") is no number and holds bytes
    that are not UTF-8: the logger drops such a point (_trk_field), a
    field that is only no number it leaves out."""
    return not _num(s) and any("\udc80" <= c <= "\udcff" for c in s)


def read_points(lines):
    """Yield (new_segment, lat, lon, ele, hdop, iso_time, acc, speed) for
    valid lines (hdop "" on a TRK2 line, acc and speed None on a TRK1
    line)."""
    new_seg = False
    for raw in lines:
        if raw == "#END\n":                # the footer: bytes after it are
            break                          # left from a longer older write
        if raw == "-\n":
            new_seg = True
            continue
        if (not raw.endswith("\n") or raw.startswith("#")
                or raw.count(",") not in (4, 5)):
            continue
        f = raw[:-1].split(",")
        acc = speed = None
        if len(f) == 6:
            lat_s, lon_s, ele, acc, speed, tc = f
            hdop = ""
        else:
            lat_s, lon_s, ele, hdop, tc = f
        try:
            lat, lon = float(lat_s), float(lon_s)
        except ValueError:
            continue
        t = _iso(tc)
        if (t is None or not math.isfinite(lat) or not math.isfinite(lon)
                or abs(lat) > 90 or abs(lon) > 180
                or any(_not_utf8(x) for x in f[2:-1])):
            continue
        yield (new_seg, lat, lon, _num(ele), _num(hdop), t,
               None if acc is None else _num(acc),
               None if speed is None else _num(speed))
        new_seg = False


EPOCH = 1577836800                         # 2020-01-01 UTC, the .trb seconds
# the logger's decoder bounds (gps_logger.py _kb_dec, _kb_time, _rd_line,
# _rd_rec):
# past them a record is damaged, as there
KB_MAX = 1 << 28                           # _KB_MAX: no varint over 2 * it
FIX_S_END = 2145916800                     # 2088-01-01 in s since 2020
LINE_MAX = 512                             # a longer text line is skipped
REC_MAX = 31                               # bytes of a delta record at most
_NUM = re.compile(r"-?(0|[1-9][0-9]*)(\.[0-9]+)?")


def _field(s):
    """(value in the last decimal, decimals) of a number written as it
    prints, None for an empty field; ValueError for anything else."""
    if not s:
        return None
    m = _NUM.fullmatch(s)
    digits = s.lstrip("-").replace(".", "")
    if not m or len(digits) > 9 or (s[0] == "-" and not int(digits)):
        raise ValueError(s)
    return int(s.replace(".", "")), len(m.group(2) or ".") - 1


def _state(text):
    """The .trb state of a text line (no newline): [seconds since 2020,
    fields as _field gives them], or None (no delta can follow it)."""
    f = text.split(",")
    if len(f) not in (5, 6):
        return None
    tc = f[-1]
    try:
        if len(tc) != 16 or not tc.isascii() or not _iso(tc):
            return None
        tm = time.strptime(tc, "%Y%m%dT%H%M%SZ")
        # (strptime takes second 60 and 61; the logger's _kb_time does not)
        if not 2020 <= tm.tm_year <= 2099 or tm.tm_sec > 59:
            return None
        fields = [_field(x) for x in f[:-1]]
    except ValueError:
        return None
    t = calendar.timegm(tm) - EPOCH
    if fields[0] is None or fields[1] is None or t >= FIX_S_END:
        return None
    return [t] + fields


def _text(st):
    out = []
    for fv in st[1:]:
        if fv is None:
            out.append("")
            continue
        v, d = fv
        sign = "-" if v < 0 else ""
        u = abs(v)
        out.append(sign + ("%d.%0*d" % (u // 10 ** d, d, u % 10 ** d)
                           if d else "%d" % u))
    out.append(time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(st[0] + EPOCH)))
    return ",".join(out) + "\n"


def trb_lines(data, errors="replace"):
    """The text lines of a binary .trb track (bytes): its "#TRKB" header,
    each point as its TRK1/TRK2 text line, "-" lines and the "#END" footer
    (nothing after it). A keyframe is "=" + the line; a delta record is a
    flag byte (0x80 | bit 0: a time step other than 1 s follows, bit i:
    the change of field i follows) and LEB128 varints (the step, zigzag
    changes in the field's last decimal). A damaged record, and the deltas
    after it up to the next keyframe, are left out: the same records as the
    logger's decoder (_kb_rec), so both read the same points. A flag byte
    whose varints run past REC_MAX bytes (an erased block reads 0xFF) is a
    damaged record of 1 byte, as the logger's _rd_rec has it. Blind spot
    (no sync marker in the format): the last few 0xFF bytes of such a run
    scan as a record of up to REC_MAX bytes that runs into what follows, so
    a keyframe within 31 bytes after the damage is eaten as varints and
    lost with its deltas up to the next keyframe; the logger loses the same
    (README, "Binary tracks").

    errors: how bytes that are not UTF-8 in a text line are decoded
    ("surrogateescape" lets read_points tell them from a U+FFFD)."""
    st = None
    i, n = 0, len(data)
    while i < n:
        m = data[i]
        if m >= 0x80:
            j = i + 1
            b = 0
            while b < 6 and j < n:       # where the record ends (_rd_rec)
                if m & (1 << b):
                    while j < n and data[j] >= 0x80:
                        j += 1
                    j += 1
                b += 1
            if j - i > REC_MAX:
                st = None
                i += 1
                continue
            j = i + 1
            vals = []
            ok = True
            for b in range(6):
                if not m & (1 << b):
                    vals.append(None)
                    continue
                v = sh = 0
                while True:
                    if j >= n:
                        return           # cut off at the end
                    c = data[j]
                    j += 1
                    v |= (c & 0x7F) << sh
                    sh += 7
                    if c < 0x80:
                        break
                # over 5 bytes or more than the encoder writes: damaged
                # (the record still ends where its varints do)
                if sh > 35 or v > 2 * KB_MAX:
                    ok = False
                vals.append(v)
            i = j
            if not ok or st is None or m & 0x40 or (m & 0x3F) >> len(st):
                st = None
                continue
            dt = 1 if vals[0] is None else vals[0]
            for k in range(1, len(st)):
                if vals[k] is not None:
                    if st[k] is None:
                        ok = False
                        break
                    z = vals[k]
                    st[k] = (st[k][0] + ((z >> 1) ^ -(z & 1)), st[k][1])
                    if not -10 ** 9 < st[k][0] < 10 ** 9:
                        ok = False       # (9 digits, as the logger)
                        break
            if not ok or dt < 1 or st[0] + dt >= FIX_S_END:
                st = None
                continue
            st[0] += dt
            yield _text(st)
            continue
        j = data.find(b"\n", i)
        if j < 0:
            return                       # a line cut off at the end
        line = data[i:j + 1].decode("utf-8", errors)
        ln = j + 1 - i
        i = j + 1
        if ln > LINE_MAX:                # the logger skips it (_rd_line)
            st = None
            continue
        if line == "#END\n":
            yield line
            return
        if line.startswith("="):
            st = _state(line[1:-1])
            yield line[1:]
        elif line == "-\n" or line.startswith("#"):
            yield line
        else:
            # a damaged record (a flag byte below 0x80 read as text): the
            # next records may be read from inside one, none until a
            # keyframe (as _kb_rec)
            st = None


def convert(lines, name):
    """Track lines -> GPX 1.1 text. name becomes the <trk><name>."""
    out = [GPX_HEAD % escape(name)]
    first = True
    ext = False
    for new_seg, lat, lon, ele, hdop, t, acc, speed in read_points(lines):
        if new_seg and not first:
            out.append("</trkseg><trkseg>\n")
        first = False
        # GPX 1.1 wptType order: ele, time, ..., hdop
        tags = "<ele>%s</ele>" % ele if ele else ""
        tags += "<time>%s</time>" % t
        if hdop:
            tags += "<hdop>%s</hdop>" % hdop
        if acc or speed:
            ext = True
            tags += ("<extensions>" + ("<esp:acc>%s</esp:acc>" % acc if acc
                                       else "")
                     + ("<esp:speed>%s</esp:speed>" % speed if speed else "")
                     + "</extensions>")
        out.append('<trkpt lat="%.6f" lon="%.6f">%s</trkpt>\n'
                   % (lat, lon, tags))
    out.append(GPX_TAIL)
    if ext:
        out[0] = out[0].replace(
            'xmlns="http://www.topografix.com/GPX/1/1"',
            'xmlns="http://www.topografix.com/GPX/1/1" xmlns:esp="%s"'
            % EXT_NS, 1)
    return "".join(out)


SUFFIXES = (".trk", ".trk.bad", ".trk.err", ".trb", ".trb.bad", ".trb.err")


def main(argv):
    if len(argv) != 2 or not argv[1].endswith(SUFFIXES):
        sys.stderr.write("usage: trk2gpx.py FILE.trk|.trb[.bad|.err]"
                         " > FILE.gpx\n")
        return 2
    path = argv[1]
    name = path.rsplit("/", 1)[-1]
    name = name[:name.index(".tr")]
    # (bytes that are not UTF-8 never reach the GPX: read_points keeps
    # only numbers and checked times)
    if path.endswith((".trb", ".trb.err")):
        with open(path, "rb") as f:
            sys.stdout.write(convert(trb_lines(f.read(), "surrogateescape"),
                                     name))
        return 0
    # newline="\n": a line ends only at \n, as _rd_line reads it (with
    # universal newlines, and also with newline="", a damaged byte 0x0D
    # splits one point line into two)
    with open(path, encoding="utf-8", errors="surrogateescape",
              newline="\n") as f:
        sys.stdout.write(convert(f, name))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
