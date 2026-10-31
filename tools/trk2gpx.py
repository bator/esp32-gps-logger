#!/usr/bin/env python3
"""Convert a .trk track from the ESP32 (compact format) to GPX 1.1.

    .venv/bin/mpremote cp :/gpx/20261005_120005.trk .
    python3 tools/trk2gpx.py 20261005_120005.trk > 20261005_120005.gpx

Quarantined (.trk.bad) and parked unreadable (.trk.err) files convert the
same way; bytes that are not UTF-8 (a damaged flash block) are replaced.

Line format: "lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ" (#TRK1; ele/hdop may be
empty) or "lat,lon,ele,acc,speed,YYYYMMDDTHHMMSSZ" (#TRK2, from UBX-NAV-PVT:
acc the horizontal accuracy in m, speed in m/s, each may be empty), told
apart by the line itself; "-" starts a new segment, "#..." lines are the
header/footer, and a line without its newline (cut off by a power loss) is
skipped. A TRK2 point's acc and speed go into <extensions> (namespace
EXT_NS, declared only when there is one): GPX 1.1 has no accuracy element,
and its <hdop> is not one.
"""
import math
import sys
from xml.sax.saxutils import escape

GPX_HEAD = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<gpx version="1.1" creator="esp32 gps_logger" '
            'xmlns="http://www.topografix.com/GPX/1/1">\n'
            '<trk><name>%s</name><trkseg>\n')
GPX_TAIL = "</trkseg></trk></gpx>\n"
EXT_NS = "https://github.com/bator/esp32-gps-logger/trk2"


def _iso(tc):
    """YYYYMMDDTHHMMSSZ -> YYYY-MM-DDTHH:MM:SSZ, or None."""
    if (len(tc) != 16 or tc[8] != "T" or tc[15] != "Z"
            or not (tc[:8] + tc[9:15]).isdigit()):
        return None
    return "%s-%s-%sT%s:%s:%sZ" % (tc[0:4], tc[4:6], tc[6:8],
                                   tc[9:11], tc[11:13], tc[13:15])


def _num(s):
    """A finite number string kept as written, or "" (dropped)."""
    try:
        return s if s and math.isfinite(float(s)) else ""
    except ValueError:
        return ""


def read_points(lines):
    """Yield (new_segment, lat, lon, ele, hdop, iso_time, acc, speed) for
    valid lines (hdop "" on a TRK2 line, acc and speed None on a TRK1
    line)."""
    new_seg = False
    for raw in lines:
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
                or abs(lat) > 90 or abs(lon) > 180):
            continue
        yield (new_seg, lat, lon, _num(ele), _num(hdop), t,
               None if acc is None else _num(acc),
               None if speed is None else _num(speed))
        new_seg = False


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


SUFFIXES = (".trk", ".trk.bad", ".trk.err")


def main(argv):
    if len(argv) != 2 or not argv[1].endswith(SUFFIXES):
        sys.stderr.write("usage: trk2gpx.py FILE.trk|.trk.bad|.trk.err"
                         " > FILE.gpx\n")
        return 2
    path = argv[1]
    name = path.rsplit("/", 1)[-1]
    name = name[:name.index(".trk")]
    with open(path, encoding="utf-8", errors="replace") as f:
        sys.stdout.write(convert(f, name))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
