# Smoke test of gps_logger.py itself on the MicroPython 1.29 unix port (and
# CPython): stub machine/network modules, import the module, run a few
# pure paths. Catches CPython-only idioms the CPython suite cannot see.
# Run: /path/to/micropython tests/mpy_unix_logger.py   (or python3)
import sys
import json
import time
import gc

here = __file__
i = here.rfind("/")
script_dir = here[:i] if i >= 0 else "."
i = script_dir.rfind("/")
root = script_dir[:i] if i >= 0 else "."
sys.path.insert(0, root)


class _Obj:
    pass


class _Uart:
    def __init__(self, *a, **k):
        self.out = []

    def any(self):
        return 0

    def read(self, n=None):
        return None

    def readinto(self, buf):
        return 0

    def write(self, b):
        self.out.append(bytes(b))
        return len(b)


class _Dummy:
    def __init__(self, *a, **k):
        pass

    def __call__(self, *a, **k):
        return self

    def __getattr__(self, name):
        return _Dummy()


machine = _Obj()
machine.Pin = _Dummy
machine.I2C = _Dummy
machine.UART = _Uart
machine.WDT = _Dummy
machine.Timer = _Dummy
machine.RTC = _Dummy
machine.freq = lambda *a: 80000000
machine.reset_cause = lambda: 1
machine.reset = lambda: None
sys.modules["machine"] = machine

network = _Obj()
network.STA_IF = 0
network.STAT_IDLE = 0
network.STAT_CONNECTING = 1
network.STAT_GOT_IP = 3
network.WLAN = _Dummy
network.hostname = lambda *a: "x"
sys.modules["network"] = network

if not hasattr(time, "ticks_ms"):       # CPython
    time.ticks_ms = lambda: int(time.time() * 1000) & 0x3FFFFFFF
    time.ticks_diff = lambda a, b: a - b
    time.ticks_add = lambda a, b: a + b
    time.sleep_ms = lambda ms: None
if not hasattr(gc, "threshold"):
    gc.threshold = lambda *a: None
if not hasattr(gc, "mem_alloc"):
    gc.mem_alloc = lambda: 50000
if not hasattr(gc, "mem_free"):
    gc.mem_free = lambda: 100000

import gps_logger as gl                 # noqa: E402

gl.DIAG = 0                             # never write the host's /diag.log
gl.LOG_DIR = "/nonexistent-gps-logger-smoke"
gl.HIST_DIR = "/nonexistent-gps-logger-smoke-hist"
gl.reset()
gl.log_reset()
gl.wifi_reset()
gl.up_reset()
fails = []


def check(name, ok):
    print(("ok  " if ok else "FAIL") + " " + name)
    if not ok:
        fails.append(name)


def nm(body):
    c = 0
    for ch in body.encode():
        c ^= ch
    return ("$%s*%02X" % (body, c)).encode()


gl.handle(nm("GNRMC,120001.00,A,4728.0000,N,01900.0000,E,0.1,,291026,,,A,V"))
gl.handle(nm("GNGGA,120001.00,4728.0000,N,01900.0000,E,1,12,0.9,100.0,M,"
             "40.0,M,,"))
check("NMEA parsed (GN talker)", gl.st["ok"] == 2 and gl.st["err"] == 0)
gl.handle(b"$GNGGA,1200" + nm("GNGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2,1"))
check("cut-off sentence glued to a whole one", gl.st["fixtype"] == 3)


# what a second of NMEA allocates, read as on the board (read_uart, then
# handle): RMC + GGA at 5 Hz, GST, GSA and GSV; each sentence's garbage
# brings the next collection (and its pause) closer
class _Feed(_Uart):
    def __init__(self):
        _Uart.__init__(self)
        self.feed(b"")

    def feed(self, data):
        self.data, self.o = memoryview(data), 0

    def any(self):
        return len(self.data) - self.o

    def readinto(self, buf):
        o = self.o
        n = min(len(buf), len(self.data) - o)
        buf[:n] = self.data[o:o + n]
        self.o = o + n
        return n


# B a sentence on the unix port (64-bit: twice the board's object sizes):
# 2655 when handle() got a copy of each line, sliced and decoded it all,
# and _second / the RMC date formatted their strings every sentence; 1646
# in place
NMEA_BUDGET = 1800


def nmea_second(k):
    out = []
    for f in range(5):
        t = "1203%02d.%02d" % (k, f * 20)
        out.append(nm("GPRMC,%s,A,4728.%04d,N,01900.0000,E,0.1,,291026,,,A"
                      % (t, k * 10 + f)))
        out.append(nm("GPGGA,%s,4728.%04d,N,01900.0000,E,1,12,0.9,100.0,M,"
                      "40.0,M,," % (t, k * 10 + f)))
    out.append(nm("GPGST,1203%02d.00,20,,,,3.0,2.5,6.0" % k))
    out.append(nm("GPGSA,A,3,01,02,03,04,,,,,,,,,1.5,0.9,1.2,1"))
    out.append(nm("GPGSV,3,1,12,01,40,083,46,02,17,308,41,03,07,344,39,04,"
                  "22,228,45"))
    return b"\r\n".join(out) + b"\r\n"


saved_uart = gl.uart
gl.uart = feed = _Feed()
gl.reset()
secs = [nmea_second(k) for k in range(10)]
n_sent = 13 * len(secs)
gc.collect()
gc.disable()
a0 = gc.mem_alloc()
for s in secs:
    feed.feed(s)
    gl.read_uart()
nmea_alloc = gc.mem_alloc() - a0
gc.enable()
gl.uart = saved_uart
print("nmea alloc %d B for %d sentences, %d B each"
      % (nmea_alloc, n_sent, nmea_alloc // n_sent))
check("NMEA read: %d sentences parsed" % n_sent,
      gl.st["ok"] == n_sent and gl.st["err"] == 0 and gl.st["seq"] == 9)
check("NMEA read: under %d B allocated a sentence" % NMEA_BUDGET,
      nmea_alloc <= NMEA_BUDGET * n_sent)
gl.reset()

pl = bytearray(8)
pl[5] = 2
for gnss, sv, cno, el, flags in ((0, 7, 41, 60, 0x1F), (2, 11, 0, -91, 0)):
    pl += bytes((gnss, sv, cno, el & 255, 10, 0, 0, 0, flags, 0, 0, 0))
gl._ubx_frame(gl._ubx(0x01, 0x35, bytes(pl)))
svs = gl.st["svs"]
check("NAV-SAT parsed", svs is not None and len(svs) == 2
      and svs[0][6] == 0 and svs[1][2] is None and svs[0][4])
ver = (b"ROM SPG 5.10 (7b202e)" + b"\0" * 9 + b"000A0000\0\0"
       + b"PROTVER=34.10" + b"\0" * 17)   # (no bytes.ljust in MicroPython)
check("MON-VER model", gl._gps_ver(ver).startswith("u-blox M10, ROM SPG"))

gl.log["still"] = sl = gl._still_state()
sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=10)
gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                    "hidden": False, "bssid": None, "ip": None}]
crash = None
try:
    for _ in range(4):
        gl._stay_scan([(b"K\xe1v\xe9z\xf3", b"\x04\x04\x04\x04\x04\x04", 1,
                        -60, 4, False)])
except Exception as e:
    crash = e
check("non-UTF-8 SSID scan", crash is None and len(sl["aps"]) == 1)

# the departure ring (array module, single-precision offsets) rebuilds the
# way out as regular track lines
gl.log["still"] = sl = gl._still_state()
sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=10)
gl.st.update(valid=True, date="2026-10-29", alt=123.4, hdop=1.2,
             hdop_at=time.ticks_ms(), fixtype=3)
for k in range(5):
    gl.st["time"] = "06:41:%02d" % (15 + k)
    gl._ring_add(sl, time.ticks_ms(), (47.5, 19.0 + k * 0.0002), k * 15.0)
check("seconds since 2020 are small ints", 0 < gl._fix_s() < 2 ** 30)
gl._still_leave()
b = gl.log["batch"]
check("departure ring way out", len(b) == 5 and b[0]
      == "47.500000,19.000000,123.4,1.2,20261029T064115Z\n"
      and b[4] == "47.500000,19.000800,123.4,1.2,20261029T064119Z\n")
gl.log_reset()


# The way out rebuilt from the ring is the line _sample_line writes for the
# same fix, rounding included: run on a single-precision build (the board's
# floats; MICROPYTHON_F32 in the suite) as well as on doubles. Elevations
# and HDOPs that round differently in single and double precision, and
# tracks across the prime meridian and the equator (offsets from a first
# sample of the other sign were rounded there).
def ring_vs_line(lat0, lon0, dlat, dlon, alt, hdop, date="2026-10-29"):
    gl.log_reset()
    gl.log["still"] = sl = gl._still_state()
    sl.update(active=True, anchor=(lat0, lon0), aref=(lat0, lon0), n=10)
    gl.st.update(valid=True, date=date, alt=alt, hdop=hdop, fixtype=3,
                 std=None, std_at=None)
    want = []
    for k in range(gl.STILL_DEP_MAX + 20):          # (the ring wraps)
        now = time.ticks_ms()
        gl.st.update(lat=lat0 + k * dlat, lon=lon0 + k * dlon, hdop_at=now,
                     time="07:%02d:%02d" % (k // 60, k % 60))
        want.append(gl._sample_line(now, None, None)[1])
        gl._ring_add(sl, now, (gl.st["lat"], gl.st["lon"]), 10.0 + k)
    gl._still_leave()
    got = gl.log["batch"]
    want = want[-gl.STILL_DEP_MAX:]
    bad = [(w, g) for w, g in zip(want, got) if w != g]
    if bad or len(got) != len(want):
        print("ring vs line", len(got), len(want), bad[:2])
    return not bad and len(got) == len(want)


ok = True
# (positions with many digits: one that ends in a 5 right after the 6th
# decimal rounds either way on a double host, ring or not)
for case in ((47.49791234567, 19.04024561234, 0.0000271828, 0.0000397141,
              123.45, 1.25),
             (51.47781234567, -0.00213141593, 0.0000123456, 0.0000317654,
              -0.04, 0.95),
             (-0.00071234567, -78.46781234567, 0.0000191919, 0.0000072727,
              8848.86, 2.35),
             (-33.85678441234, 151.21529671234, -0.0000233333, 0.0000301234,
              58.65, 4.45),
             (64.12651234567, -21.81741234567, 0.0000191234, -0.0000277654,
              None, None)):
    ok = ring_vs_line(*case) and ok
check("departure ring rebuilds the lines _sample_line writes", ok)
# seconds in the ring count from its first sample: a fix dated 2099 (a
# receiver past a week rollover) overflowed the 32-bit array and was
# stamped 1963 on the board (CPython raises)
try:
    ok = ring_vs_line(47.5, 19.0, 0.00002, 0.00003, 100.0, 1.0, "2099-08-22")
except OverflowError as e:
    print("ring 2099", repr(e))
    ok = False
check("departure ring keeps times after 2088", ok)
gl.log_reset()


# UBX-NAV-PVT: one struct.unpack_from on the frame's memoryview (as
# read_uart hands it over), the weighted integer mean, and track lines
# written from the integers: on single precision 47.4979123 is a float
# ~0.4 m off, the line still says 47.497912
import struct

saved_svs = gl.st["svs"]


def pvt_frame(sec, nano, lat_i, lon_i, hacc_mm, mps=0.0):
    p = struct.pack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHH4xihH",
                    0, 2026, 10, 31, 12, 0, sec, 7, 40, nano, 3, 1, 0, 14,
                    lon_i, lat_i, 140000, 123460, hacc_mm, 2500, 0, 0, 0,
                    int(mps * 1000), 0, 300, 500000, 120, 0, 0, 0, 0)
    return gl._ubx(0x01, 0x07, p)


gl.reset()
gl.log_reset()
for k in range(5):
    gl._ubx_frame(memoryview(pvt_frame(0, k * 200000000, 474979123 + k,
                                       -3141593 - k, 1250 + 250 * k, 1.44)))
gl._ubx_frame(memoryview(pvt_frame(1, 0, 474979123, -3141593, 1500)))
ws = [1.0 / ((1250 + 250 * k) / 1000.0) ** 2 for k in range(5)]
want_i = 474979123 + gl._round(sum(w * k for k, w in zip(range(5), ws))
                               / sum(ws))
check("NAV-PVT parsed from a memoryview (one unpack_from)",
      gl.st["valid"] and gl.st["lat_i"] == want_i
      and gl.st["lon_i"] == -3141593 - (want_i - 474979123)
      and gl.st["epochs"] == 5 and gl.st["used"] == 14
      and abs(gl.st["hacc"] - 1.75) < 1e-6)
s = gl._sample_line(time.ticks_ms(), None, None)
check("NAV-PVT line from the integers (no float grid)", s is not None
      and s[1] == "47.497912,-0.314159,123.5,2,1.4,20261031T120000Z\n")   # acc 1.75 up
if s is not None:
    print("pvt line", repr(s[1]))
check("integer coordinates: halves away from zero, no -0",
      [gl._i7(v) for v in (-5, -4, 5, 474979125, -1512152967)]
      == ["-0.000001", "0.000000", "0.000001", "47.497913", "-151.215297"])


# the departure ring with NAV-PVT seconds (integers) and NMEA ones mixed,
# rebuilt into a TRK2 and a TRK1 file: the very lines _sample_line wrote
def ring_mixed(fmt):
    gl.reset()
    gl.log_reset()
    gl.log["fmt"] = fmt
    gl.log["batch"] = ["x"]
    gl.log["still"] = sl = gl._still_state()
    sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=10)
    want = []
    for k in range(gl.STILL_DEP_MAX + 7):
        now = time.ticks_ms()
        tm = "10:%02d:%02d" % (k // 60, k % 60)
        if k % 3:
            li, lo = 474979123 + 271 * k, -3141593 - 314 * k
            gl.st.update(valid=True, date="2026-10-31", time=tm, fixtype=3,
                         lat_i=li, lon_i=lo, lat=li / 10000000,
                         lon=lo / 10000000, hacc=0.45 + k % 7 * 1.25,
                         mps=(k % 5 * 0.45) or None, alt=123.45 + k,
                         hdop=None, std=None)
        else:
            gl.st.update(valid=True, date="2026-10-31", time=tm, fixtype=3,
                         lat_i=None, lon_i=None, hacc=None, mps=None,
                         lat=47.49791234567 + k * 0.0000271828,
                         lon=-0.31415 - k * 0.0000314159, alt=-0.04,
                         hdop=1.25 + k % 3, hdop_at=now, std=None)
        want.append(gl._sample_line(now, None, None)[1])
        gl._ring_add(sl, now, (gl.st["lat"], gl.st["lon"]), 10.0 + k)
    gl.log["batch"] = []
    gl.log["file"] = "/nonexistent-gps-logger-smoke/x.trk"
    gl._still_leave()
    got = gl.log["batch"]
    want = want[-gl.STILL_DEP_MAX:]
    bad = [(w, g) for w, g in zip(want, got) if w != g]
    if bad or len(got) != len(want):
        print("ring mixed", fmt, len(got), len(want), bad[:2])
    gl.log_reset()
    return not bad and len(got) == len(want)


check("departure ring rebuilds NAV-PVT (TRK2) and mixed lines exactly",
      ring_mixed(2) and ring_mixed(1))
gl.reset()
gl.st["svs"] = saved_svs

try:
    jd = json.loads("".join(gl._web_json_parts()))
except Exception as e:
    print("json", repr(e))
    jd = None
check("status JSON parts are valid JSON", jd is not None
      and jd.get("satellites") and "uptime" in jd)
check("JSON parts stream", not isinstance(gl._web_json_parts(), list))
check("UTC stamp", gl._stamp_at(1793256075)[0] == "2026-10-29T06:41:15Z")
check("uptime counter", gl.uptime_s() >= 0)
check("upload wait without holds", gl._upload_wait(time.ticks_ms()) == 0)

# read_batch parses a .trk in place (bytearray.find/count, float() of a
# memoryview, bytes %): the MicroPython side of the CPython suite
trk = "/tmp/gps-logger-smoke-%d.trk" % time.ticks_ms()
with open(trk, "wb") as f:
    f.write(b"#TRK1\n47.500000,19.000000,100.0,0.9,20261029T101500Z\n-\n"
            b"47.5,19.1,,,20261029T101501Z\n" + b"x" * 700 + b"\n"
            b"47.6,19.\xff1,,,20261029T101502Z\n"
            b"47.7,19.2,,,2026102XT101503Z\n#END\n"
            b"48.0,20.0,,,20261029T101504Z\n")
try:
    n, n_ok, size, new_off, eof = gl.read_batch(trk, 0)
    locs = json.loads(bytes(gl._body[:size]))["locations"]
finally:
    import os
    os.remove(trk)
check("read_batch .trk in place", (n, n_ok, eof) == (4, 2, True)
      and locs[0]["geometry"]["coordinates"] == [19.0, 47.5]
      and locs[0]["properties"]["timestamp"] == "2026-10-29T10:15:00Z"
      and locs[0]["properties"]["horizontal_accuracy"] == 4.5
      and locs[1]["properties"]["timestamp"] == "2026-10-29T10:15:01Z"
      and "altitude" not in locs[1]["properties"])

# a .trb: flush's encoder, then read_batch decoding it to the same JSON as
# the .trk of the same lines, also resumed mid-file from a keyframe (ints,
# bytearray +=, bytes %, the varints: the MicroPython side)
tl = []
la, lo = 471853560, 184269340
for i in range(130):
    la += 1350 - i * 7
    lo -= 800 + i
    tl.append("%s,%s,%.1f,%d,%.1f,%s\n" % (
        gl._i7(la), gl._i7(lo), 120.0 + i * 0.3 - (i % 7), 2 + i % 3,
        12.0 + (i % 11) * 0.4, gl._stamp_s(215611200 + i + i // 40 * 30)[1]))
tl.append("-12.5,-0.000001,,1.2,20261101T090000Z\n")
base = "/tmp/gps-logger-smoke-%d" % time.ticks_ms()
body = gl._kb_encode(tl, len(tl), [70])
outs = []
try:
    with open(base + ".trb", "wb") as f:
        f.write(b"#TRKB\n" + body + b"#END\n")
    with open(base + ".trk", "wb") as f:
        f.write(("#TRK2\n" + "".join(tl[:70]) + "-\n" + "".join(tl[70:])
                 + "#END\n").encode())
    for ext in (".trb", ".trk"):
        got, off, key = [], 0, None
        while True:
            n, n_ok, size, new_off, eof = gl.read_batch(base + ext, off, key)
            got.append(bytes(gl._body[:size]))
            key = gl._rk[0]
            if eof:
                break
            off = new_off
        outs.append(got)
    n, n_ok, size, new_off, eof = gl.read_batch(base + ".trb", 0)
    mid = gl.read_batch(base + ".trb", new_off, gl._rk[0])
    tail = bytes(gl._body[:mid[2]])
finally:
    import os
    for ext in (".trb", ".trk"):
        os.remove(base + ext)
check("trb: same JSON as the .trk, %d B for %d points"
      % (len(body), len(tl)), outs[0] == outs[1] and len(outs[0]) == 3
      and len(body) < 12 * len(tl))
check("trb: resumed from a keyframe mid-file", tail == outs[1][1])

# the .trb read allocates about what the .trk read does: it runs right
# before a TLS POST, when the heap is tightest (it made ~2.7 times the
# garbage: a bytes % per field, a tuple per record, the date tuple)
if hasattr(gc, "mem_alloc"):
    base = "/tmp/gps-logger-alloc-%d" % time.ticks_ms()
    tl2 = tl[:50]
    try:
        with open(base + ".trb", "wb") as f:
            f.write(b"#TRKB\n" + gl._kb_encode(tl2, len(tl2), None) + b"#END\n")
        with open(base + ".trk", "wb") as f:
            f.write(("#TRK2\n" + "".join(tl2) + "#END\n").encode())
        used = {}
        for ext in (".trk", ".trb"):
            gc.collect()
            gc.disable()
            a0 = gc.mem_alloc()
            gl.read_batch(base + ext, 0)
            used[ext] = gc.mem_alloc() - a0
            gc.enable()
    finally:
        import os
        for ext in (".trb", ".trk"):
            os.remove(base + ext)
    print("read_batch alloc", used)
    check("trb: read_batch allocates at most 1.25 x the .trk read",
          used[".trb"] <= used[".trk"] * 1.25)

# the day history as the page reads it: /hist from each reply's
# X-Hist-Next until X-Hist-End
class _Sink:
    def __init__(self, keep=True):
        self.keep = keep
        self.out = []

    def settimeout(self, t):
        pass

    def sendall(self, b):
        if self.keep:
            self.out.append(bytes(b))


def hist_day(day):
    txt, o = b"", 0
    for _ in range(1000):
        c = _Sink()
        gl._web_hist(c, ("/hist?d=%s&o=%d" % (day, o)).encode())
        head, body = b"".join(c.out).split(b"\r\n\r\n", 1)
        nx = en = None
        for h in head.split(b"\r\n"):
            if h.startswith(b"X-Hist-Next: "):
                nx = int(h[13:])
            elif h.startswith(b"X-Hist-End: "):
                en = int(h[12:])
        txt += body
        if nx is None or nx >= en:
            break
        o = nx
    return txt.decode()


# an uploaded .trb batch goes to the day history without a list of its
# lines (it built ~100 objects right after the POST): what a batch of 50
# allocates beyond one of 10 is under 40 bytes a point
if hasattr(gc, "mem_alloc"):
    import os
    base = "/tmp/gps-logger-hist-%d" % time.ticks_ms()
    gl.HIST_DIR = base + "-hist"
    gl.free = lambda: None
    hist_alloc = {}
    hist_txt = ""
    try:
        for k in (10, 50):
            tl3 = tl[:k]
            with open(base + ".trb", "wb") as f:
                f.write(b"#TRKB\n" + gl._kb_encode(tl3, k, None) + b"#END\n")
            gl._hist_batch(base + ".trb", 0, 1 << 20, None)   # (day file made)
            gc.collect()
            gc.disable()
            a0 = gc.mem_alloc()
            gl._hist_batch(base + ".trb", 0, 1 << 20, None)
            hist_alloc[k] = gc.mem_alloc() - a0
            gc.enable()
        hist_txt = hist_day(tl[0][-17:-9])
        # a /hist reply allocates about what a text day's did (its send
        # buffer, the head), whatever the day's length (no list of lines)
        tl4 = []
        for i in range(2000):
            tl4.append("%s,%s,%.1f,%d,%.1f,%s\n" % (
                gl._i7(471853560 + i * 1311), gl._i7(184269340 - i * 977),
                120.0 + (i % 9) * 0.3, 2 + i % 3, 12.0 + (i % 11) * 0.4,
                gl._stamp_s(215697600 + i + i // 500)[1]))
        for i in range(0, 2000, 50):
            gl._hist_add(tl4[i:i + 50], 50)
        day4 = tl4[0][-17:-9]
        with open(gl.HIST_DIR + "/20261001.trk", "w") as f:
            f.write("".join(tl4[:400]))
        web_alloc = []
        for q in ("20261001&o=0", day4 + "&o=0", day4 + "&o=6000"):
            gc.collect()
            gc.disable()
            a0 = gc.mem_alloc()
            gl._web_hist(_Sink(False), ("/hist?d=" + q).encode())
            web_alloc.append(gc.mem_alloc() - a0)
            gc.enable()
        hist4 = hist_day(day4)
    finally:
        os.remove(base + ".trb")
        for n in os.listdir(gl.HIST_DIR):
            os.remove(gl.HIST_DIR + "/" + n)
        os.rmdir(gl.HIST_DIR)
    print("hist alloc", hist_alloc, "web", web_alloc)
    check("trb: an uploaded batch goes to the day history as its records, "
          "read back as its text, under 40 bytes allocated a point",
          hist_txt == "".join(tl[:10]) * 2 + "".join(tl[:50]) * 2
          and (hist_alloc[50] - hist_alloc[10]) < 40 * 40)
    check("trb: a /hist reply of a 2000-point .trb day decodes it whole, "
          "a reply allocating at most 1.5 KB more than a text day's",
          hist4 == "".join(tl4) and web_alloc[1] == web_alloc[2]
          and web_alloc[1] <= web_alloc[0] + 1536)

# the hidden home's learned BSSIDs survive a reboot (/home_aps: hex lines,
# bytes.find and a bytearray parse, no binascii); garbage lines are skipped
gl.HOME_APS_FILE = "/tmp/gps-logger-smoke-aps-%d" % time.ticks_ms()
gl.free = lambda: None
try:
    gl.wifi["home_aps"] = [b"\x01\x02\x03\xaa\xbb\xcc", b"\xff" * 6]
    gl._home_aps_save()
    with open(gl.HOME_APS_FILE, "ab") as f:
        f.write(b"zz\n\xff\xfe\nABCDEF012345\n")
    gl._home_aps_load()
    aps = gl.wifi["home_aps"]
finally:
    import os
    try:
        os.remove(gl.HOME_APS_FILE)
    except OSError:
        pass
check("home_aps file round trip", aps == [b"\x01\x02\x03\xaa\xbb\xcc",
                                          b"\xff" * 6,
                                          b"\xab\xcd\xef\x01\x23\x45"])

# 2026-11 review: the real things only this port has. The real gc, the real
# poll on a real socket, what the passes without a client allocate
if sys.implementation.name == "micropython":
    check("gc.threshold(8192) set by the import", gc.threshold() == 8192)


    def _alloc(fn, n=100):
        gc.collect()
        gc.disable()
        a0 = gc.mem_alloc()
        for _ in range(n):
            fn()
        a1 = gc.mem_alloc()
        gc.enable()
        return (a1 - a0) // n


    import socket as _socket
    gl.wifi_reset()
    gl.up_reset()
    gl.wifi["state"] = "connected"
    gl.WEB = True
    for k in range(8):                  # a free port
        gl.WEB_PORT = 20000 + (time.ticks_ms() * 7 + k * 1013) % 20000
        gl.web_tick()
        if gl._web_sock is not None:
            break
    poll_up = gl._web_sock is not None and gl._web_poll is not None
    idle = _alloc(gl.web_tick)
    guard = _alloc(lambda: gl._guarded("web", gl.web_tick, None, True))
    print("web idle alloc", idle, "guarded", guard)
    check("web: a real poll on the listening socket; no client: 0 bytes a pass",
          poll_up and idle == 0 and guard == 0)
    # the same poll sees a client come: the socket is taken and answered
    cl = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
    cl.connect(_socket.getaddrinfo("127.0.0.1", gl.WEB_PORT)[0][-1])
    cl.write(b"GET /nope HTTP/1.0\r\n\r\n")
    for _ in range(20):
        gl.web_tick()
    got = cl.read(12)
    cl.close()
    gl._web_close()
    check("web: ipoll(0) wakes for a client, the poll goes with the socket",
          got == b"HTTP/1.0 404" and gl._web_poll is None and gl._web_sock is None)
    gl.WEB = False
    gl.up["next_at"] = time.ticks_add(time.ticks_ms(), 10000000)
    up_alloc = _alloc(gl.upload_tick)
    print("upload_tick alloc", up_alloc)
    check("upload_tick that waits allocates nothing (no cell)", up_alloc == 0)
    gl.wifi_reset()

    # the display on the real framebuf: glyphs outside the band skipped, each
    # byte to the panel as when every glyph went into every band
    sent = []


    class _Spi:
        def __init__(self, *a, **k):
            pass

        def write(self, b):
            sent.append(bytes(b))


    class _Pin(_Dummy):
        OUT = 1


    machine.SPI = _Spi
    import gps_tft as tft
    tft.Pin = _Pin
    tft.time = _Obj()
    tft.time.sleep_ms = lambda ms: None
    tft.start()
    states = (("47", True, 22, "11:42", 3, 3, "1284", True, (42, "7 nap")),
              ("-", False, 0, "--:--", None, 0, "", False, None),
              ("128", True, 9, "23:59", 0, 1, "kész", False, (100, "tele")),
              ("0", True, 12, "00:00", 1, 2, "99999", True, (75, "<1 óra")))


    def panel():
        del sent[:]
        for s in states:
            tft._D.top = tft._D.low = tft._D.lit = None
            tft.set_state(*s)
            tft.frame()
        return b"".join(sent)


    def text_all(fb, font, s, x, base, tone):
        f, mv = font
        p = tft._gs4_palette(tone)
        y = base - f[5]
        h = f[4]
        for ch in s:
            i = tft._index(f, ch)
            w = f[1][i]
            o = f[2][i * 2] | f[2][i * 2 + 1] << 8
            fb.blit((mv[o:o + (w + 1) // 2 * h], w, h, tft.framebuf.GS4_HMSB), x, y, tft.BG, p)
            x += w
        return x


    now_panel = panel()
    text_now = tft._text
    tft._text = text_all
    all_panel = panel()
    tft._text = text_now
    out_band = _alloc(lambda: tft._text(tft._D.fb, tft._NUM, "12", 4, 500, "txt"))
    print("tft panel bytes", len(now_panel), "out of band alloc", out_band)
    check("tft: on the real framebuf the panel gets each byte as before, a text "
          "outside the band allocates nothing",
          len(now_panel) > 50000 and now_panel == all_panel and out_band == 0)

if fails:
    raise SystemExit("mpy logger smoke FAILED: %s" % ", ".join(fails))
print("mpy logger smoke passed")
