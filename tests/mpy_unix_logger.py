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

if fails:
    raise SystemExit("mpy logger smoke FAILED: %s" % ", ".join(fails))
print("mpy logger smoke passed")
